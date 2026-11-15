/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2023 NXP
 */

#ifndef NEUTRON_BUFFER_H
#define NEUTRON_BUFFER_H

/****************************************************************************/

#include <linux/dma-direction.h>
#include <linux/kref.h>
#include <linux/types.h>

/****************************************************************************/

struct neutron_device;
struct device;
struct dma_buf;
struct sg_table;
struct cma;
struct page;

/**
 * struct neutron_buffer - Buffer
 * @dev:                Device
 * @file:               File
 * @dmabuf:             DMA-BUF handle
 * @kref:               Reference counting
 * @capacity:           Maximum capacity of the buffer
 * @cpu_addr:           Kernel mapped address
 * @dma_addr:           DMA address
 * @dma_addr_orig:      Original DMA address before range mapping
 * @firmware_name:      Neutron custom firmware file name
 * @firmware_p:         Neutron Custom firmware pointer.
 * @sgt:                Pages of a chunked buffer (device behind an IOMMU), mapped
 *                      at one IOVA range; NULL for a physically contiguous (CMA)
 *                      buffer. Either way the device sees one contiguous range at
 *                      @dma_addr.
 * @pages, @npages:     The chunked buffer's pages (for vmap/mmap/free).
 * @chunks:             Chunk slots used; @chunks_cma came from the CMA pool,
 *                      @chunks_big from ordinary memory as one high-order page.
 * @order, @cma, @cma_map: chunk order, the CMA area, and which slots came from it.
 */
struct neutron_buffer {
	struct neutron_device *ndev;
	struct file           *file;
	struct dma_buf        *dmabuf;
	struct kref           kref;
	size_t                size;
	void                  *cpu_addr;
	dma_addr_t            dma_addr;
	const char            *firmware_name;
	const struct firmware *firmware_p;
	struct sg_table       *sgt;
	struct page           **pages;
	unsigned long         npages;
	unsigned int          chunks;
	unsigned int          chunks_big;
	unsigned int          chunks_cma;
	unsigned int          order;
	struct cma            *cma;
	unsigned long         *cma_map;
};

/****************************************************************************/

/**
 * neutron_buffer_create() - Create buffer
 *
 * This function must be called in the context of a user space process.
 *
 * Return: fd on success, else error code.
 */
int neutron_buffer_create(struct neutron_device *ndev,
			  size_t size, __u64 *addr_out);

struct neutron_buffer *neutron_buffer_get_from_fd(int fd);

/**
 * neutron_buffer_sync() - Cache maintenance for a byte range of a buffer
 *
 * Clamped to the buffer. Works for contiguous and non-contiguous buffers.
 */
void neutron_buffer_sync(struct neutron_buffer *buf, size_t offset,
			 size_t size, enum dma_data_direction dir);

/**
 * neutron_buffer_get() - Gut buffer
 */
void neutron_buffer_get(struct neutron_buffer *buf);

/**
 * neutron_buffer_put() - Put buffer
 */
void neutron_buffer_put(struct neutron_buffer *buf);

#endif /* NEUTRON_BUFFER_H */
