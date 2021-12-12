/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Device tables which are exported to userspace via
 * scripts/mod/file2alias.c.  You must keep that file in sync with this
 * header.
 */

#ifndef DEVICETABLE_H
#define DEVICETABLE_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/uuid.h>
typedef unsigned long kernel_ulong_t;
#endif


#define MHI_NAME_SIZE 32

/**
 * struct mhi_device_id - MHI device identification
 * @chan: MHI channel name
 * @driver_data: driver data;
 */

struct mhi_device_id {
	const char chan[MHI_NAME_SIZE];
	kernel_ulong_t driver_data;
};

#endif /* DEVICETABLE_H */
