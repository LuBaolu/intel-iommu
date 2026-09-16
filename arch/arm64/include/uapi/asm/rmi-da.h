/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */

#ifndef _UAPI__ASM_RMI_DA_H
#define _UAPI__ASM_RMI_DA_H

#include <linux/types.h>

struct arm64_vdev_validate_mmio_guest_req {
	__aligned_u64 gpa_base;
	__aligned_u64 gpa_top;
	__aligned_u64 pa_base;
};

struct arm64_vdev_set_tdi_state_guest_req {
	__u32 tdi_state;
};

#endif
