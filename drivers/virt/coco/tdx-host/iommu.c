// SPDX-License-Identifier: GPL-2.0
/*
 * TDX IOMMU Extensions support
 *
 * Copyright (C) 2026 Intel Corporation
 */
#define pr_fmt(fmt) "tdx-host: " fmt

#include <linux/pci.h>
#include <asm/vmx.h>
#include <asm/tdx.h>

#include "private.h"

/* These describe the trusted invalidation queue buffers. */
#define IQ_BUFFERS_NUM		2
#define IQ_BUFFER_PAGES		2
#define IQ_BUFFER_SIZE		SZ_8K

/* The IOMMU_MT root page holds one u64 entry per metadata page. */
#define MT_ROOT_SIZE		SZ_4K
#define MT_MAX_ENTRIES		(MT_ROOT_SIZE / sizeof(u64))

static void *tdx_iommu_alloc_pages(int node, size_t size)
{
	struct folio *folio;

	if (node == NUMA_NO_NODE)
		node = numa_mem_id();

	folio = __folio_alloc_node(GFP_KERNEL | __GFP_ZERO, get_order(size), node);
	if (!folio)
		return NULL;

	return folio_address(folio);
}

static void tdx_iommu_free_pages(void *virt)
{
	if (!virt)
		return;

	folio_put(virt_to_folio(virt));
}

static void free_mt_pages(struct tdx_iommu_pages *array)
{
	unsigned int i;

	if (!array)
		return;

	for (i = 0; i < array->nr_entries; i++)
		tdx_iommu_free_pages(array->pages[i]);

	tdx_iommu_free_pages(array->root);
	kfree(array->pages);
	kfree(array);
}

DEFINE_FREE(free_mt_pages, struct tdx_iommu_pages *, free_mt_pages(_T))

static void **alloc_mt_pages(unsigned int nr_entries, int node)
{
	unsigned int i;
	void **pages;
	void *vaddr;

	pages = kzalloc_objs(*pages, nr_entries);
	if (!pages)
		return NULL;

	/* Allocate two contiguous buffers for the invalidation queue. */
	pages[0] = tdx_iommu_alloc_pages(node, IQ_BUFFER_SIZE);
	if (!pages[0])
		goto free_pages;

	pages[1] = tdx_iommu_alloc_pages(node, IQ_BUFFER_SIZE);
	if (!pages[1])
		goto free_pages;

	/* Allocate the required number of pages for the IOMMU metadata. */
	for (i = IQ_BUFFERS_NUM; i < nr_entries; i++) {
		vaddr = tdx_iommu_alloc_pages(node, SZ_4K);
		if (!vaddr)
			goto free_pages;
		pages[i] = vaddr;
	}

	return pages;

free_pages:
	for (i = 0; i < nr_entries; i++)
		tdx_iommu_free_pages(pages[i]);
	kfree(pages);

	return NULL;
}

static void populate_mt_pages(struct tdx_iommu_pages *array)
{
	unsigned int nr_entries = array->nr_entries;
	void **pages = array->pages;
	u64 *entries = array->root;
	unsigned int i;

	/*
	 * Populate the parameter for the TDH_IOMMU_SETUP SEAMCALL according to
	 * the format defined in "Table 3.35: Structure of IOMMU_MT Parameter"
	 * of the ABI reference specification.
	 */
	for (i = 0; i < nr_entries; i++) {
		entries[i] = __pa(pages[i]);
		if (i < IQ_BUFFERS_NUM)
			entries[i] |= IQ_BUFFER_PAGES;
	}
}

static struct tdx_iommu_pages *
tdx_iommu_alloc_mt_pages(struct dmar_drhd_unit *drhd, unsigned int nr_mt_pages)
{
	unsigned int nr_entries = nr_mt_pages + IQ_BUFFERS_NUM;
	struct tdx_iommu_pages *array;

	if (!nr_mt_pages || nr_mt_pages > MT_MAX_ENTRIES - IQ_BUFFERS_NUM)
		return NULL;

	array = kzalloc_obj(*array);
	if (!array)
		return NULL;

	array->root = tdx_iommu_alloc_pages(drhd->node, MT_ROOT_SIZE);
	if (!array->root)
		goto free_array;

	array->nr_entries = nr_entries;
	array->pages = alloc_mt_pages(nr_entries, drhd->node);
	if (!array->pages)
		goto free_root;

	populate_mt_pages(array);

	return array;

free_root:
	tdx_iommu_free_pages(array->root);
free_array:
	kfree(array);
	return NULL;
}

static DEFINE_XARRAY(tdx_iommu_xa);

static int tdx_iommu_clear(struct tdx_iommu_state *iommu_state)
{
	u64 r;

	if (!iommu_state->mt_pages)
		return 0;

	r = tdh_iommu_clear(iommu_state->tdx_iommu_id);
	if (r) {
		pr_err("TDH.IOMMU.CLEAR failed, regbase 0x%llx tdx_iommu_id 0x%llx status 0x%llx\n",
		       iommu_state->drhd->reg_base_addr, iommu_state->tdx_iommu_id, r);
		return -EFAULT;
	}

	free_mt_pages(iommu_state->mt_pages);
	iommu_state->mt_pages = NULL;
	iommu_state->tdx_iommu_id = 0;

	return 0;
}

static int tdx_iommu_setup(struct dmar_drhd_unit *drhd)
{
	const struct tdx_sys_info *tdx_sysinfo = tdx_get_sysinfo();
	unsigned int mt_page_count = tdx_sysinfo->tdx_connect.iommu_mt_page_count;
	struct tdx_iommu_state *state;
	u64 r, tdx_iommu_id;
	int ret;

	struct tdx_iommu_state *iommu_state __free(kfree) = kzalloc_obj(*iommu_state);
	if (!iommu_state)
		return -ENOMEM;

	struct tdx_iommu_pages *iommu_mt __free(free_mt_pages) =
			tdx_iommu_alloc_mt_pages(drhd, mt_page_count);
	if (!iommu_mt)
		return -ENOMEM;

	r = tdh_iommu_setup(drhd->reg_base_addr, iommu_mt->root, &tdx_iommu_id);
	/* TDX Extension is not supported on this IOMMU. Nothing to do. */
	if ((r & TDX_SEAMCALL_STATUS_MASK) == TDX_OPERAND_INVALID)
		return -EOPNOTSUPP;
	if (r) {
		pr_err("TDH.IOMMU.SETUP failed, regbase 0x%llx status 0x%llx\n",
		       drhd->reg_base_addr, r);
		return -EFAULT;
	}

	state = no_free_ptr(iommu_state);
	state->drhd = drhd;
	state->tdx_iommu_id = tdx_iommu_id;
	state->mt_pages = no_free_ptr(iommu_mt);
	ret = xa_insert(&tdx_iommu_xa, (unsigned long)tdx_iommu_id, state, GFP_KERNEL);
	if (ret) {
		WARN_ON(tdx_iommu_clear(state));
		kfree(state);
		return ret;
	}

	pr_info("Trusted DMA for TEE initialized, regbase 0x%llx\n",
		drhd->reg_base_addr);

	return 0;
}

static void tdx_iommu_disable_all(void)
{
	struct tdx_iommu_state *iommu_state;
	unsigned long tdx_iommu_id;

	xa_for_each(&tdx_iommu_xa, tdx_iommu_id, iommu_state) {
		/*
		 * Clear failed: the IOMMU remains in Secure TDX Mode. Keep
		 * the entry and leave drhd->tdx_mode set so the host stays
		 * confined to the lower half of the DID space.
		 */
		if (tdx_iommu_clear(iommu_state))
			continue;

		xa_erase(&tdx_iommu_xa, tdx_iommu_id);
		dmar_tdxcs_iommu_release(iommu_state->drhd);
		kfree(iommu_state);
	}
}

static void tdx_iommu_disable_all_action(void *unused)
{
	tdx_iommu_disable_all();
}

int tdx_iommu_enable_all(struct device *dev)
{
	int ret;

	ret = dmar_tdxcs_iommu_init_all(tdx_iommu_setup);
	if (ret) {
		tdx_iommu_disable_all();
		return ret;
	}

	return devm_add_action_or_reset(dev, tdx_iommu_disable_all_action, NULL);
}
