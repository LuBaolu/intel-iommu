/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 ARM Ltd.
 */

#ifndef _VIRT_COCO_RHI_DA_H_
#define _VIRT_COCO_RHI_DA_H_

#include <linux/arm-smccc-rhi.h>

struct pci_dev;
bool rhi_has_da_support(void);
int rhi_vdev_set_tdi_state(struct pci_dev *pdev, enum rhi_tdi_state target_state);
int rhi_update_vdev_interface_report_cache(struct pci_dev *pdev);
int rhi_update_vdev_measurements_cache(struct pci_dev *pdev, const u8 *nonce);
#endif
