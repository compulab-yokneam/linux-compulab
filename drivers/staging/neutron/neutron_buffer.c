// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023-2026 NXP
 */

/****************************************************************************/

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/scatterlist.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/ktime.h>
#include <linux/sched/signal.h>
#include <linux/cma.h>
#include <linux/dma-map-ops.h>
#include <linux/highmem.h>
#include <linux/bitmap.h>

#include "neutron_buffer.h"
#include "neutron_device.h"
#include "uapi/neutron.h"

/****************************************************************************/

/*
 * Behind an IOMMU the NPU only needs a contiguous IOVA range, not contiguous
 * physical memory. The stock driver asks for one physically contiguous CMA block
 * per buffer (DMA_ATTR_FORCE_CONTIGUOUS); a 2 GiB request (ONNX Runtime's Neutron
 * EP arena) fails with -ENOMEM/-EBUSY as soon as the reusable pool holds pinned
 * or dirty pages, and the session silently falls back to the CPU.
 *
 * Behind an IOMMU, buffers are built from 2 MiB chunks (chunk_order) taken from the
 * NPU's own CMA pool: each chunk needs only one 2 MiB range migrated, the memory
 * stays inside the reservation made for the NPU (unmovable allocations cannot use
 * CMA pageblocks, so taking 2 GiB from ordinary memory starves everything else), and
 * each chunk sits at a 2 MiB IOVA so the SMMU maps it with one 2 MiB block entry.
 * Plain 4 KiB mappings cost the NPU dearly (LLM decode at ~40% speed, SMMU TLB
 * misses). Chunks the pool cannot supply come from ordinary memory, at most
 * fallback_max_mib per buffer. cma_whole tries one contiguous block first.
 */
static bool iommu_noncontig = true;
module_param(iommu_noncontig, bool, 0644);
MODULE_PARM_DESC(iommu_noncontig,
		 "Behind an IOMMU, build buffers from chunks (default: true; 0 = stock; read per allocation)");

static bool cma_whole;
module_param(cma_whole, bool, 0644);
MODULE_PARM_DESC(cma_whole, "Try one contiguous CMA block before chunks (default: false)");

static uint chunk_order = 9;	/* 2 MiB with 4 KiB pages */
module_param(chunk_order, uint, 0644);
MODULE_PARM_DESC(chunk_order, "Page order of the chunks (default 9 = 2 MiB; 0 = single pages)");

static bool cma_chunks = true;
module_param(cma_chunks, bool, 0644);
MODULE_PARM_DESC(cma_chunks, "Take the chunks from the NPU's CMA pool (default: true)");

static uint fallback_max_mib = 256;
module_param(fallback_max_mib, uint, 0644);
MODULE_PARM_DESC(fallback_max_mib,
		 "Most MiB per buffer taken from ordinary (non-CMA) memory (default 256)");

static bool trace_alloc;
module_param(trace_alloc, bool, 0644);
MODULE_PARM_DESC(trace_alloc,
		 "Log each buffer's placement: segments, physical alignment, bytes in 2 MiB-mappable segments");

/* diagnostics: cache maintenance done by neutron_buffer_sync() */
static ulong sync_calls, sync_segments, sync_ns;
module_param(sync_calls, ulong, 0444);
module_param(sync_segments, ulong, 0444);
module_param(sync_ns, ulong, 0444);

/* Largest power of two dividing addr, capped at 1 GiB (for logging). */
static unsigned long neutron_phys_align(phys_addr_t addr)
{
	return addr ? min_t(unsigned long, 1UL << __ffs64(addr), SZ_1G) : SZ_1G;
}

static void neutron_buffer_trace(struct neutron_buffer *buf, u64 ns)
{
	struct device *dev = buf->ndev->dev;
	size_t blk = 0;
	unsigned long min_align = SZ_1G;
	struct scatterlist *sg;
	unsigned int i, nseg;
	phys_addr_t phys;

	if (!buf->sgt) {
		/* contiguous: one segment; the IOMMU can use 2 MiB blocks only if phys is aligned */
		if (is_vmalloc_addr(buf->cpu_addr))
			phys = page_to_phys(vmalloc_to_page(buf->cpu_addr));
		else
			phys = virt_to_phys(buf->cpu_addr);
		min_align = neutron_phys_align(phys);
		if (min_align >= SZ_2M)
			blk = ALIGN_DOWN(buf->size, SZ_2M);
		dev_info(dev, "alloc contiguous size=%zu phys=%pa align=%luK blockable=%zu%% ms=%llu\n",
			 buf->size, &phys, min_align >> 10,
			 buf->size ? blk * 100 / buf->size : 0, ns / NSEC_PER_MSEC);
		return;
	}

	nseg = buf->sgt->orig_nents;
	for_each_sgtable_sg(buf->sgt, sg, i) {
		phys = sg_phys(sg);
		min_align = min(min_align, neutron_phys_align(phys));
		if (IS_ALIGNED(phys, SZ_2M) && sg->length >= SZ_2M)
			blk += ALIGN_DOWN(sg->length, SZ_2M);
	}
	dev_info(dev, "alloc chunked size=%zu segments=%u min_align=%luK blockable=%zu%% chunks=%u cma=%u buddy=%u pages=%u iova=%pad ms=%llu\n",
		 buf->size, nseg, min_align >> 10,
		 buf->size ? blk * 100 / buf->size : 0,
		 buf->chunks, buf->chunks_cma, buf->chunks_big, buf->chunks - buf->chunks_cma - buf->chunks_big,
		 &buf->dma_addr, ns / NSEC_PER_MSEC);
}

static bool neutron_buffer_use_noncontig(struct device *dev)
{
	return iommu_noncontig && device_iommu_mapped(dev);
}

/* Release pages[0..n): CMA chunks are marked in cma_map (one bit per chunk slot). */
static void neutron_buffer_free_pages(struct neutron_buffer *buf, unsigned long n)
{
	unsigned long step = 1UL << buf->order, i = 0;

	while (i < n) {
		unsigned long slot = min(n - i, step);

		if (buf->cma_map && test_bit(i >> buf->order, buf->cma_map) && slot == step) {
			cma_release(buf->cma, buf->pages[i], step);
		} else {
			unsigned long k;

			for (k = 0; k < slot && i + k < n; k++)
				__free_page(buf->pages[i + k]);
		}
		i += slot;
	}
	bitmap_free(buf->cma_map);
	buf->cma_map = NULL;
	kvfree(buf->pages);
	buf->pages = NULL;
}

static void neutron_buffer_free_mem(struct neutron_buffer *buf)
{
	struct device *dev = buf->ndev->dev;

	if (buf->sgt) {
		if (buf->cpu_addr)
			vunmap(buf->cpu_addr);
		dma_unmap_sgtable(dev, buf->sgt, DMA_BIDIRECTIONAL, DMA_ATTR_SKIP_CPU_SYNC);
		sg_free_table(buf->sgt);
		kfree(buf->sgt);
		buf->sgt = NULL;
		neutron_buffer_free_pages(buf, buf->npages);
	} else if (buf->cpu_addr) {
		dma_free_attrs(dev, buf->size, buf->cpu_addr,
			       buf->dma_addr, DMA_ATTR_FORCE_CONTIGUOUS);
	}
	buf->cpu_addr = NULL;
}

/*
 * Chunked allocation: chunk slots of 2^order pages, from the NPU's CMA pool first
 * (cma_alloc: migrates one chunk-sized range, never invokes the OOM killer), else from
 * ordinary memory as one high-order page or single pages, at most fallback_max_mib per
 * buffer. A slot is always filled completely so later chunks keep their alignment.
 * Pages are zeroed: user space maps them.
 */
static int neutron_buffer_alloc_chunked(struct neutron_buffer *buf)
{
	struct device *dev = buf->ndev->dev;
	const gfp_t gfp = GFP_KERNEL | __GFP_ZERO | __GFP_NORETRY | __GFP_NOWARN;
	unsigned int order = min_t(unsigned int, chunk_order, MAX_PAGE_ORDER);
	unsigned long n = PAGE_ALIGN(buf->size) >> PAGE_SHIFT, got = 0, k;
	unsigned long fallback_left = (unsigned long)fallback_max_mib << (20 - PAGE_SHIFT);
	int ret = -ENOMEM;

	buf->order = order;
	buf->cma = cma_chunks ? dev_get_cma_area(dev) : NULL;
	buf->pages = kvmalloc_array(n, sizeof(*buf->pages), GFP_KERNEL);
	buf->cma_map = bitmap_zalloc(DIV_ROUND_UP(n, 1UL << order), GFP_KERNEL);
	if (!buf->pages || !buf->cma_map) {
		kvfree(buf->pages);
		bitmap_free(buf->cma_map);
		buf->pages = NULL;
		buf->cma_map = NULL;
		return -ENOMEM;
	}
	buf->chunks = buf->chunks_big = buf->chunks_cma = 0;

	while (got < n) {
		unsigned long slot = min(n - got, 1UL << order);
		struct page *page = NULL;

		buf->chunks++;
		if (buf->cma && slot == (1UL << order))
			page = cma_alloc(buf->cma, slot, order, true);
		if (page) {
			for (k = 0; k < slot; k++) {
				clear_highpage(page + k);
				buf->pages[got + k] = page + k;
			}
			set_bit(got >> order, buf->cma_map);
			buf->chunks_cma++;
			goto next;
		}
		if (fallback_left < slot)
			goto fail;
		fallback_left -= slot;
		if (order && slot == (1UL << order))
			page = alloc_pages(gfp, order);
		if (page) {
			split_page(page, order);
			for (k = 0; k < slot; k++)
				buf->pages[got + k] = page + k;
			buf->chunks_big++;
		} else {
			for (k = 0; k < slot; k++) {
				buf->pages[got + k] = alloc_page(gfp);
				if (!buf->pages[got + k]) {
					got += k;
					goto fail;
				}
			}
		}
next:
		got += slot;
		if (fatal_signal_pending(current)) {
			ret = -EINTR;
			goto fail;
		}
		cond_resched();
	}
	buf->npages = n;

	buf->sgt = kzalloc(sizeof(*buf->sgt), GFP_KERNEL);
	if (!buf->sgt)
		goto free_pages;
	ret = sg_alloc_table_from_pages(buf->sgt, buf->pages, n, 0, buf->size, GFP_KERNEL);
	if (ret)
		goto free_sgt;
	/* one IOVA range, size-aligned, so 2 MiB chunks land on 2 MiB IOVA boundaries */
	ret = dma_map_sgtable(dev, buf->sgt, DMA_BIDIRECTIONAL, DMA_ATTR_SKIP_CPU_SYNC);
	if (ret)
		goto free_table;
	buf->dma_addr = sg_dma_address(buf->sgt->sgl);

	buf->cpu_addr = vmap(buf->pages, n, VM_MAP, PAGE_KERNEL);
	if (!buf->cpu_addr) {
		ret = -ENOMEM;
		goto unmap;
	}
	/* write the zeroes back, so the NPU never reads stale DRAM through this buffer */
	neutron_buffer_sync(buf, 0, buf->size, DMA_TO_DEVICE);
	return 0;

unmap:
	dma_unmap_sgtable(dev, buf->sgt, DMA_BIDIRECTIONAL, DMA_ATTR_SKIP_CPU_SYNC);
free_table:
	sg_free_table(buf->sgt);
free_sgt:
	kfree(buf->sgt);
	buf->sgt = NULL;
free_pages:
	neutron_buffer_free_pages(buf, n);
	return ret;
fail:
	/* free whole slots done so far plus the partial one */
	neutron_buffer_free_pages(buf, got);
	return ret;
}

static int neutron_buffer_alloc_mem(struct neutron_buffer *buf)
{
	struct device *dev = buf->ndev->dev;
	u64 t0 = ktime_get_ns();
	int ret;

	if (!neutron_buffer_use_noncontig(dev)) {
		/* stock behaviour */
		buf->cpu_addr = dma_alloc_attrs(dev, buf->size, &buf->dma_addr,
						GFP_KERNEL,
						DMA_ATTR_FORCE_CONTIGUOUS);
		ret = buf->cpu_addr ? 0 : -ENOMEM;
		goto out;
	}

	if (cma_whole) {
		/* __GFP_NOWARN: a 2 GiB request that misses CMA must not WARN in the page allocator */
		buf->cpu_addr = dma_alloc_attrs(dev, buf->size, &buf->dma_addr,
						GFP_KERNEL | __GFP_NOWARN,
						DMA_ATTR_FORCE_CONTIGUOUS);
		if (buf->cpu_addr) {
			ret = 0;
			goto out;
		}
	}
	ret = neutron_buffer_alloc_chunked(buf);
out:
	if (!ret && trace_alloc)
		neutron_buffer_trace(buf, ktime_get_ns() - t0);
	return ret;
}

void neutron_buffer_sync(struct neutron_buffer *buf, size_t offset,
			 size_t size, enum dma_data_direction dir)
{
	struct scatterlist *sg;
	size_t pos = 0, end;
	unsigned int i;
	ulong segs = 0;
	u64 t0;

	if (offset >= buf->size || !size)
		return;
	size = min(size, buf->size - offset);
	end = offset + size;
	t0 = ktime_get_ns();

	if (!buf->sgt) {
		neutron_memory_sync(buf->ndev, buf->dma_addr + offset, size, dir);
		segs = 1;
		goto stats;
	}

	/*
	 * dma_sync_single_*() translates only the start IOVA to a physical
	 * address, so sync each physically contiguous segment on its own. The
	 * segments are mapped in order from buf->dma_addr.
	 */
	for_each_sgtable_sg(buf->sgt, sg, i) {
		size_t s = max(pos, offset);
		size_t e = min(pos + sg->length, end);

		if (s < e) {
			neutron_memory_sync(buf->ndev, buf->dma_addr + s,
					    e - s, dir);
			segs++;
		}
		pos += sg->length;
		if (pos >= end)
			break;
	}
stats:
	WRITE_ONCE(sync_calls, sync_calls + 1);
	WRITE_ONCE(sync_segments, sync_segments + segs);
	WRITE_ONCE(sync_ns, sync_ns + (ktime_get_ns() - t0));
}

static void neutron_buffer_destroy(struct kref *kref)
{
	struct neutron_buffer *buf =
		container_of(kref, struct neutron_buffer, kref);

	dev_dbg(buf->ndev->dev, "Buffer destroy. buf=0x%pS\n", buf);

	if (buf->firmware_p)
		release_firmware(buf->firmware_p);

	neutron_buffer_free_mem(buf);
	devm_kfree(buf->ndev->dev, buf);
}

/****************************************************************************/
/* DMA-BUF operations                                                       */
/****************************************************************************/

static struct sg_table *neutron_dmabuf_map(struct dma_buf_attachment *attach,
					   enum dma_data_direction dir)
{
	struct neutron_buffer *buf = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	if (buf->sgt) {
		struct scatterlist *src, *dst;
		unsigned int i;

		/* Copy the page list; the importer maps it for itself. */
		ret = sg_alloc_table(sgt, buf->sgt->orig_nents, GFP_KERNEL);
		if (ret) {
			kfree(sgt);
			return ERR_PTR(ret);
		}
		dst = sgt->sgl;
		for_each_sgtable_sg(buf->sgt, src, i) {
			sg_set_page(dst, sg_page(src), src->length, src->offset);
			dst = sg_next(dst);
		}
		goto map;
	}

	/* Build sg_table from the CMA allocation — dma_get_sgtable_attrs
	 * correctly describes the physical pages regardless of how the
	 * kernel virtual address was mapped.
	 */
	ret = dma_get_sgtable_attrs(buf->ndev->dev, sgt, buf->cpu_addr,
				    buf->dma_addr, buf->size,
				    DMA_ATTR_FORCE_CONTIGUOUS);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}

map:
	/* Create IOMMU mappings for the IMPORTING device (e.g. Mali GPU).
	 * This is critical — without it, the importer's IOMMU has no
	 * translation for these pages and writes go nowhere.
	 */
	ret = dma_map_sgtable(attach->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
	if (ret) {
		sg_free_table(sgt);
		kfree(sgt);
		return ERR_PTR(ret);
	}

	return sgt;
}

static void neutron_dmabuf_unmap(struct dma_buf_attachment *attach,
				 struct sg_table *sgt,
				 enum dma_data_direction dir)
{
	dma_unmap_sgtable(attach->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(sgt);
}

static int neutron_dmabuf_mmap(struct dma_buf *dmabuf,
			       struct vm_area_struct *vma)
{
	struct neutron_buffer *buf = dmabuf->priv;

	/* dma_buf_mmap() sets vma->vm_pgoff to the offset within the buffer.
	 * dma_mmap_attrs() adds vm_pgoff internally, so reset to 0 here.
	 */
	vma->vm_pgoff = 0;

	if (buf->sgt)
		return vm_map_pages(vma, buf->pages, buf->npages);

	return dma_mmap_attrs(buf->ndev->dev, vma, buf->cpu_addr,
			      buf->dma_addr, buf->size,
			      DMA_ATTR_FORCE_CONTIGUOUS);
}

static void neutron_dmabuf_release(struct dma_buf *dmabuf)
{
	struct neutron_buffer *buf = dmabuf->priv;

	neutron_buffer_put(buf);
}

static int neutron_dmabuf_begin_cpu_access(struct dma_buf *dmabuf,
					   enum dma_data_direction dir)
{
	struct neutron_buffer *buf = dmabuf->priv;

	/* the device is flagged coherent, so the DMA API would skip this; do it here */
	if (dir != DMA_TO_DEVICE)
		neutron_buffer_sync(buf, 0, buf->size, DMA_FROM_DEVICE);
	return 0;
}

static int neutron_dmabuf_end_cpu_access(struct dma_buf *dmabuf,
					 enum dma_data_direction dir)
{
	struct neutron_buffer *buf = dmabuf->priv;

	if (dir != DMA_FROM_DEVICE)
		neutron_buffer_sync(buf, 0, buf->size, DMA_TO_DEVICE);
	return 0;
}

static const struct dma_buf_ops neutron_dmabuf_ops = {
	.map_dma_buf      = neutron_dmabuf_map,
	.unmap_dma_buf    = neutron_dmabuf_unmap,
	.mmap             = neutron_dmabuf_mmap,
	.release          = neutron_dmabuf_release,
	.begin_cpu_access = neutron_dmabuf_begin_cpu_access,
	.end_cpu_access   = neutron_dmabuf_end_cpu_access,
};

/****************************************************************************/

int neutron_buffer_create(struct neutron_device *ndev,
			  size_t size, __u64 *addr_out)
{
	struct neutron_buffer *buf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	int ret = -ENOMEM;

	if (!size)
		return -EINVAL;

	buf = devm_kzalloc(ndev->dev, sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf->ndev = ndev;
	buf->size = size;
	kref_init(&buf->kref);

	ret = neutron_buffer_alloc_mem(buf);
	if (ret)
		goto free_buf;

	exp_info.ops = &neutron_dmabuf_ops;
	exp_info.size = PAGE_ALIGN(buf->size);
	exp_info.flags = O_RDWR | O_CLOEXEC;
	exp_info.priv = buf;
	exp_info.exp_name = "neutron";

	buf->dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(buf->dmabuf)) {
		ret = PTR_ERR(buf->dmabuf);
		goto free_dma;
	}

	ret = dma_buf_fd(buf->dmabuf, O_RDWR | O_CLOEXEC);
	if (ret < 0) {
		/* the dma-buf release frees the buffer (kref) */
		dma_buf_put(buf->dmabuf);
		return ret;
	}

	buf->file = buf->dmabuf->file;
	*addr_out = buf->dma_addr;

	dev_dbg(buf->ndev->dev,
		"Buffer create. fd=%d, size=%zu, cpu_addr=0x%pK, dma_addr=0x%llx, %s\n",
		ret, size, buf->cpu_addr, buf->dma_addr,
		buf->sgt ? "non-contiguous" : "contiguous");

	return ret;

free_dma:
	neutron_buffer_free_mem(buf);

free_buf:
	devm_kfree(buf->ndev->dev, buf);

	return ret;
}

/* Returns the buffer with a reference held; release it with neutron_buffer_put(). */
struct neutron_buffer *neutron_buffer_get_from_fd(int fd)
{
	struct neutron_buffer *buf;
	struct dma_buf *dmabuf;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return ERR_PTR(-EINVAL);

	if (dmabuf->ops != &neutron_dmabuf_ops) {
		dma_buf_put(dmabuf);
		return ERR_PTR(-EINVAL);
	}

	/* take the reference while the dma-buf still pins the buffer */
	buf = dmabuf->priv;
	neutron_buffer_get(buf);
	dma_buf_put(dmabuf);

	return buf;
}

void neutron_buffer_get(struct neutron_buffer *buf)
{
	kref_get(&buf->kref);
}

void neutron_buffer_put(struct neutron_buffer *buf)
{
	kref_put(&buf->kref, neutron_buffer_destroy);
}
