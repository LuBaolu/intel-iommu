/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2026 Intel Corporation */
#ifndef __TDX_HOST_PRIVATE_H
#define __TDX_HOST_PRIVATE_H

#include <linux/dmar.h>

/*
 * Represents a list of pages for TDX module defined IOMMU_MT object.
 * Typically it uses a "root page" as the medium to exchange a list of
 * data pages between host and TDX module.
 */
struct tdx_iommu_pages {
	u64 *root;
	void **pages;
	unsigned int nr_entries;
};

struct tdx_iommu_state {
	struct dmar_drhd_unit *drhd;
	u64 tdx_iommu_id;
	struct tdx_iommu_pages *mt_pages;
};

#ifdef CONFIG_TDX_CONNECT
int tdx_iommu_enable_all(struct device *dev);
#else
static inline int tdx_iommu_enable_all(struct device *dev)
{
	return -EINVAL;
}
#endif /* CONFIG_TDX_CONNECT */
#endif /* __TDX_HOST_PRIVATE_H */
