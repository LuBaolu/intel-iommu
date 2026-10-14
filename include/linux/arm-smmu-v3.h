/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_ARM_SMMU_V3_H
#define _LINUX_ARM_SMMU_V3_H

#include <linux/types.h>

struct device;

/* Parameters needed to create a Realm pSMMU. */
struct arm_smmu_realm_params {
	phys_addr_t psmmu_phys;
	u32 l2_sid;
	u64 irq_flags;
	u64 gerr_addr;
	u64 gerr_data;
	u64 eventq_addr;
	u64 eventq_data;
	u64 priq_addr;
	u64 priq_data;
};

int arm_smmu_get_realm_params(struct device *dev,
			      struct arm_smmu_realm_params *params);

#endif /* _LINUX_ARM_SMMU_V3_H */
