// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 ARM Ltd.
 */

#include <linux/arm-rmi-cmds.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>

#include "rmi-da.h"

static int pci_ide_segment(struct pci_dev *pdev)
{
	if (pdev->fm_enabled)
		return pci_domain_nr(pdev->bus);
	return 0;
}

static unsigned int pci_get_max_rid(struct pci_dev *pdev)
{
	int fn;
	int max_rid;
	int slot = PCI_SLOT(pdev->devfn);

	for (fn = 0; fn < 8; fn++) {
		struct pci_dev *fn_dev;

		fn_dev = pci_get_slot(pdev->bus, PCI_DEVFN(slot, fn));
		if (!fn_dev)
			continue;

		max_rid = pci_dev_id(fn_dev);
		pci_dev_put(fn_dev);
	}
	return max_rid;
}

static int init_pdev_params(struct pci_dev *pdev, struct rmi_pdev_params *params)
{
	int rid;
	unsigned long category;
	struct pci_config_window *cfg = pdev->bus->sysdata;

	/* check we are ECAM compliant */
	if (!pdev->bus->ops->map_bus)
		return -EINVAL;

	switch (pci_pcie_type(pdev)) {
	case PCI_EXP_TYPE_ENDPOINT: {
		struct cca_host_pf0_ep_dsc *pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev);

		/* Endpoint needs DOE mailbox */
		if (!pf0_ep_dsc->pci.doe_mb)
			return -EINVAL;

		params->flags = RMI_PDEV_FLAGS_SPDM;
		category = RMI_PDEV_FLAGS_CATEGORY_OFF_CHIP_EP;
		break;
	}
	default:
		return -EINVAL;
	}

	params->flags |= (category << RMI_PDEV_FLAGS_CATEGORY_SHIFT);
	/* assign the ep device with RMM */
	rid = pci_dev_id(pdev);
	params->pdev_id = rid;
	params->hb_base = cfg->res.start;
	params->routing_id = pci_ide_segment(pdev);
	/* slot number for certificate chain default to zero */
	params->id_index = 0;
	params->hash_algo = RMI_HASH_SHA_256;
	/* no multi function device here. */
	params->rid_base = rid;
	params->rid_top = pci_get_max_rid(pdev) + 1;
	return 0;
}

static inline int rmi_pdev_create(unsigned long pdev_phys,
		unsigned long pdev_params_phys, unsigned long *rmi_ret)
{
	struct rmi_sro_state *sro __free(kfree) = kmalloc_obj(*sro);
	if (!sro)
		return -ENOMEM;

	*rmi_ret = rmi_sro_memxfer_cmd(sro, GFP_KERNEL, SMC_RMI_PDEV_CREATE,
				       pdev_phys, pdev_params_phys);

	return 0;
}

int cca_pdev_create(struct pci_dev *pci_dev)
{
	int ret;
	void *rmm_pdev;
	bool should_free = true;
	phys_addr_t rmm_pdev_phys;
	struct rmi_pdev_params *params;
	struct cca_host_pdev_dsc *pdev_dsc = to_cca_pdev_dsc(pci_dev);

	rmm_pdev = (void *)get_zeroed_page(GFP_KERNEL);
	if (!rmm_pdev)
		return -ENOMEM;

	rmm_pdev_phys = virt_to_phys(rmm_pdev);
	if (rmi_delegate_page(rmm_pdev_phys)) {
		ret = -EIO;
		goto err_granule_delegate;
	}

	params = (struct rmi_pdev_params *)get_zeroed_page(GFP_KERNEL);
	if (!params) {
		ret = -ENOMEM;
		goto err_param_alloc;
	}

	ret = init_pdev_params(pci_dev, params);
	if (ret)
		goto err_init_pdev_params;

	{
		unsigned long rmi_ret;

		ret = rmi_pdev_create(rmm_pdev_phys, virt_to_phys(params),
				      &rmi_ret);
		if (ret || rmi_ret) {
			if (!ret)
				ret = -EIO;
			goto err_init_pdev_params;
		}
	}

	pdev_dsc->rmm_pdev = rmm_pdev;
	free_page((unsigned long)params);
	return 0;

err_init_pdev_params:
	free_page((unsigned long)params);
err_param_alloc:
	if (rmi_undelegate_page(rmm_pdev_phys))
		should_free = false;
err_granule_delegate:
	if (should_free)
		free_page((unsigned long)rmm_pdev);
	return ret;
}
