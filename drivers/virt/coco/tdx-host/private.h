/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2026 Intel Corporation */
#ifndef __TDX_HOST_PRIVATE_H
#define __TDX_HOST_PRIVATE_H

#include <linux/dmar.h>

struct pci_dev;

#ifdef CONFIG_TDX_CONNECT
int tdx_iommu_get(struct pci_dev *pdev);
void tdx_iommu_put(struct pci_dev *pdev);
#else
static inline int tdx_iommu_get(struct pci_dev *pdev)
{
	return -EOPNOTSUPP;
}

static inline void tdx_iommu_put(struct pci_dev *pdev) { }
#endif /* CONFIG_TDX_CONNECT */
#endif /* __TDX_HOST_PRIVATE_H */
