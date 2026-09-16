// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Arm Ltd.
 */

#include <linux/arm-rmi-cmds.h>
#include <linux/arm-smmu-v3.h>
#include <linux/cleanup.h>
#include <linux/iommu.h>
#include <linux/iommufd.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci-tsm.h>
#include <linux/slab.h>
#include <linux/tsm.h>

#include <asm/kvm_emulate.h>
#include <asm/rmi_cmds.h>

#include "rmi-da.h"

struct cca_psmmu {
	struct list_head node;
	phys_addr_t phys;
	unsigned int users;
};

struct cca_viommu {
	struct iommufd_viommu core;
	struct cca_psmmu *psmmu;
};

static LIST_HEAD(cca_psmmus);
static DEFINE_MUTEX(cca_psmmus_lock);

static struct cca_viommu *to_cca_viommu(struct iommufd_viommu *viommu)
{
	return container_of(viommu, struct cca_viommu, core);
}

static struct cca_psmmu *
cca_psmmu_get(const struct arm_smmu_realm_params *params)
{
	struct rmi_psmmu_params *rmi_params;
	struct cca_psmmu *psmmu;
	unsigned long rmi_ret = 0;
	int ret;

	guard(mutex)(&cca_psmmus_lock);
	list_for_each_entry(psmmu, &cca_psmmus, node) {
		if (psmmu->phys == params->psmmu_phys) {
			psmmu->users++;
			return psmmu;
		}
	}

	psmmu = kzalloc(sizeof(*psmmu), GFP_KERNEL);
	if (!psmmu)
		return ERR_PTR(-ENOMEM);
	rmi_params = (void *)get_zeroed_page(GFP_KERNEL);
	if (!rmi_params) {
		kfree(psmmu);
		return ERR_PTR(-ENOMEM);
	}
	rmi_params->flags = params->irq_flags;
	rmi_params->gerr_addr = params->gerr_addr;
	rmi_params->gerr_data = params->gerr_data;
	rmi_params->eventq_addr = params->eventq_addr;
	rmi_params->eventq_data = params->eventq_data;
	rmi_params->priq_addr = params->priq_addr;
	rmi_params->priq_data = params->priq_data;
	ret = rmi_psmmu_activate(params->psmmu_phys,
				 virt_to_phys(rmi_params), &rmi_ret);
	free_page((unsigned long)rmi_params);
	if (ret || rmi_ret) {
		kfree(psmmu);
		return ERR_PTR(-EIO);
	}

	psmmu->phys = params->psmmu_phys;
	psmmu->users = 1;
	list_add(&psmmu->node, &cca_psmmus);
	return psmmu;
}

static void cca_psmmu_put(struct cca_psmmu *psmmu)
{
	unsigned long rmi_ret = 0;

	guard(mutex)(&cca_psmmus_lock);
	if (--psmmu->users)
		return;
	if (rmi_psmmu_deactivate(psmmu->phys, &rmi_ret) || rmi_ret) {
		pr_warn("failed to deactivate Realm pSMMU at %pa\n",
			&psmmu->phys);
	}
	list_del(&psmmu->node);
	kfree(psmmu);
}
