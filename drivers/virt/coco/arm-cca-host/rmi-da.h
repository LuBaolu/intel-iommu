/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 ARM Ltd.
 */

#ifndef _VIRT_COCO_RMM_DA_H_
#define _VIRT_COCO_RMM_DA_H_

#include <linux/arm-smccc-rmi.h>
#include <linux/arm-rmi-cmds.h>
#include <linux/pci.h>
#include <linux/pci-ide.h>
#include <linux/pci-tsm.h>

/**
 * struct cca_host_pf0_ep_dsc - PF0 endpoint device security context.
 * @pci: Physical Function 0 TDISP link context
 * @sel_stream: Selective IDE Stream descriptor
 */
struct cca_host_pf0_ep_dsc {
	struct pci_tsm_pf0 pci;
	struct pci_ide *sel_stream;
};

struct cca_host_fn_dsc {
	struct pci_tsm pci;
};

static inline struct cca_host_pf0_ep_dsc *to_cca_pf0_ep_dsc(struct pci_dev *pdev)
{
	struct pci_tsm *tsm = pdev->tsm;

	if (!tsm || !is_pci_tsm_pf0(pdev))
		return NULL;

	return container_of(tsm, struct cca_host_pf0_ep_dsc, pci.base_tsm);
}

static inline struct cca_host_fn_dsc *to_cca_fn_dsc(struct pci_dev *pdev)
{
	struct pci_tsm *tsm = pdev->tsm;

	return container_of(tsm, struct cca_host_fn_dsc, pci);
}

#endif
