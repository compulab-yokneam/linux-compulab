// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Minimal reproducers for the drivers/staging/neutron bugs fixed in
 * compulab-yokneam/linux-compulab#16. Raw ioctls on /dev/neutron0; no model,
 * no ONNX Runtime, no NPU job runs to completion.
 *
 *   build: make -C tools/testing/selftests/drivers/neutron
 *   run:   sudo ./neutron_repro race|alloc [MiB]|range|dupfree|badbuf|uaf
 *
 * Exit code 0 = fixed behaviour, 1 = bug reproduced.
 * dupfree, badbuf and uaf crash or corrupt a stock kernel: run them last, on a test board.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#include "neutron.h"

#define MiB (1024UL * 1024)
static int dev;

static int buf_create(size_t size)
{
	struct neutron_uapi_buffer_create a = { .size = size };
	int fd = ioctl(dev, NEUTRON_IOCTL_BUFFER_CREATE, &a);

	return fd < 0 ? -errno : fd;
}

static int cache_sync(int fd, uint32_t off, uint32_t size, int from_dev)
{
	struct neutron_uapi_cache_sync a = { .fd = fd, .offset = off, .size = size,
					     .direction = from_dev };

	return ioctl(dev, NEUTRON_IOCTL_CACHE_SYNC, &a) < 0 ? -errno : 0;
}

/* 1. Memory corruption: 4 processes write, sync to device, sync from device, read back. */
static int race(void)
{
	int c, st, bad = 0;

	for (c = 0; c < 4; c++) {
		if (fork())
			continue;
		size_t sz = 8 * MiB, k, lost = 0;
		for (int i = 0; i < 200; i++) {
			int fd = buf_create(sz);
			uint64_t *p = fd < 0 ? MAP_FAILED :
				mmap(NULL, sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

			if (p == MAP_FAILED)
				_exit(2);
			for (k = 0; k < sz / 8; k += 512)
				p[k] = k ^ i;
			cache_sync(fd, 0, sz, 0);	/* clean: CPU -> memory */
			cache_sync(fd, 0, sz, 1);	/* invalidate: memory -> CPU */
			for (k = 0; k < sz / 8; k += 512)
				lost += p[k] != (k ^ i);
			munmap(p, sz);
			close(fd);
		}
		printf("process %d: %zu words lost\n", c, lost);
		fflush(stdout);
		_exit(lost ? 1 : 0);
	}
	for (c = 0; c < 4; c++) {
		wait(&st);
		bad += !WIFEXITED(st) || WEXITSTATUS(st);
	}
	printf("race: %s\n", bad ? "DATA LOST (bug)" : "clean");
	return !!bad;
}

/* 2. ENOMEM: one buffer of the size ONNX Runtime's Neutron EP asks for. */
static int alloc(size_t mib)
{
	struct timespec t0, t1;
	int fd;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	fd = buf_create(mib * MiB);
	clock_gettime(CLOCK_MONOTONIC, &t1);
	printf("alloc %zu MiB: %s after %.1f s\n", mib, fd < 0 ? strerror(-fd) : "ok",
	       t1.tv_sec - t0.tv_sec + (t1.tv_nsec - t0.tv_nsec) / 1e9);
	return fd < 0;
}

/* 3. CACHE_SYNC range check: offset + size wraps around 2^32 and passes the check. */
static int range(void)
{
	int fd = buf_create(16 * MiB), r;

	if (fd < 0)
		return 2;
	r = cache_sync(fd, 0xfffff000u, 0x2000, 0);
	printf("range: wrapping sync returned %s (expected Invalid argument)\n",
	       r ? strerror(-r) : "success");
	return r != -EINVAL;
}

/* 4. Double free: dma_buf_fd() fails when the process has no free fd. */
static int dupfree(void)
{
	struct rlimit rl = { 3, 3 };	/* stdin/out/err only; /dev/neutron0 is fd 3 */
	int i, r = 0;

	setrlimit(RLIMIT_NOFILE, &rl);
	for (i = 0; i < 100 && !r; i++)
		r = buf_create(MiB) == -EMFILE ? 0 : 1;
	printf("dupfree: 100 x BUFFER_CREATE with no free fd -> %s; check dmesg\n",
	       r ? "unexpected result" : "EMFILE");
	return r;
}

/* 5. Bad buf_id: the job is queued with an ERR_PTR buffer and dereferences it. */
static int badbuf(void)
{
	struct neutron_uapi_inference_args a = { .buf_id = 9999, .firmw_id = 0x7e57 };
	int r = ioctl(dev, NEUTRON_IOCTL_INFERENCE_CREATE, &a);

	printf("badbuf: INFERENCE_CREATE returned %s (expected Invalid argument)\n",
	       r < 0 ? strerror(errno) : "an fd");
	return r >= 0;
}

/* 6. Use-after-free: the queued job holds no reference to its buffer. */
static int uaf(void)
{
	for (int i = 0; i < 1000; i++) {
		int fd = buf_create(MiB), inf;
		/* firmw_id that never matches: the job stops at "firmware is not ready" */
		struct neutron_uapi_inference_args a = { .buf_id = fd, .firmw_id = 0x7e57 };

		if (fd < 0)
			return 2;
		inf = ioctl(dev, NEUTRON_IOCTL_INFERENCE_CREATE, &a);
		close(fd);	/* last reference: the buffer is freed while the job is queued */
		usleep(2000);
		if (inf >= 0)
			close(inf);
	}
	printf("uaf: 1000 jobs whose buffer was closed while queued; check dmesg (KASAN)\n");
	return 0;
}

int main(int argc, char **argv)
{
	const char *t = argc > 1 ? argv[1] : "";

	dev = open("/dev/neutron0", O_RDWR | O_CLOEXEC);
	if (dev < 0) {
		perror("/dev/neutron0");
		return 2;
	}
	if (!strcmp(t, "race"))
		return race();
	if (!strcmp(t, "alloc"))
		return alloc(argc > 2 ? strtoul(argv[2], NULL, 0) : 2048);
	if (!strcmp(t, "range"))
		return range();
	if (!strcmp(t, "dupfree"))
		return dupfree();
	if (!strcmp(t, "badbuf"))
		return badbuf();
	if (!strcmp(t, "uaf"))
		return uaf();
	fprintf(stderr, "usage: %s race|alloc [MiB]|range|dupfree|badbuf|uaf\n", argv[0]);
	return 2;
}
