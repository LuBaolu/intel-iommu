// SPDX-License-Identifier: GPL-2.0
/*
 * TDX IOMMU Extensions support
 *
 * Copyright (C) 2026 Intel Corporation
 */
#include <linux/pci.h>
#include <asm/vmx.h>
#include <asm/tdx.h>

#include "private.h"

#define IQ_BUFFERS_NUM		2
#define IQ_BUFFER_PAGES		2
#define IQ_BUFFER_SIZE		SZ_8K

static void *tdx_iommu_alloc_pages(int nid, size_t size)
{
	struct folio *folio;

	might_sleep();

	if (nid == NUMA_NO_NODE)
		nid = numa_mem_id();

	folio = __folio_alloc_node(GFP_KERNEL | __GFP_ZERO, get_order(size), nid);
	if (unlikely(!folio))
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
	if (!array)
		return;

	for (int i = 0; i < array->nr_entries; i++)
		tdx_iommu_free_pages(array->pages[i]);

	tdx_iommu_free_pages(array->root);
	kfree(array->pages);
	kfree(array);
}

DEFINE_FREE(free_mt_pages, struct tdx_iommu_pages *, free_mt_pages(_T))

static void **alloc_mt_pages(unsigned int nr_entries, int node)
{
	void **pages;
	void *vaddr;
	int i;

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
	for (i = 0; i < nr_entries; i++) {
		if (!pages[i])
			break;

		tdx_iommu_free_pages(pages[i]);
	}
	kfree(pages);

	return NULL;
}

static void populate_mt_pages(struct tdx_iommu_pages *array)
{
	unsigned int nr_entries = array->nr_entries;
	void **pages = array->pages;
	u64 *entries = array->root;
	int i;

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

	if (!nr_mt_pages || nr_mt_pages > (SZ_4K / sizeof(u64) - IQ_BUFFERS_NUM))
		return NULL;

	array = kzalloc_obj(*array);
	if (!array)
		return NULL;

	array->root = tdx_iommu_alloc_pages(drhd->node, SZ_4K);
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

static void tdx_iommu_clear(struct tdx_iommu_state *iommu_state)
{
	u64 r;

	if (!iommu_state->mt_pages)
		return;

	r = tdh_iommu_clear(iommu_state->tdx_iommu_id);
	if (r) {
		pr_err("TDH.IOMMU.CLEAR failed, status 0x%llx\n", r);
		return;
	}

	free_mt_pages(iommu_state->mt_pages);
	iommu_state->mt_pages = NULL;
	iommu_state->tdx_iommu_id = 0;
	kfree(iommu_state);
}

static int tdx_iommu_setup(struct dmar_drhd_unit *drhd)
{
	const struct tdx_sys_info *tdx_sysinfo = tdx_get_sysinfo();
	unsigned int mt_page_count = tdx_sysinfo->tdx_connect.iommu_mt_page_count;
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
	/* TDX Extension is not supported on this iommu. Nothing to do. */
	if ((r & TDX_SEAMCALL_STATUS_MASK) == TDX_OPERAND_INVALID)
		return 0;
	if (r) {
		pr_err("TDH.IOMMU.SETUP failed, regbase 0x%llx status 0x%llx\n",
		       drhd->reg_base_addr, r);
		return -EFAULT;
	}

	iommu_state->tdx_iommu_id = tdx_iommu_id;
	iommu_state->mt_pages = no_free_ptr(iommu_mt);
	ret = xa_insert(&tdx_iommu_xa, (unsigned long)tdx_iommu_id,
			no_free_ptr(iommu_state), GFP_KERNEL);
	if (ret) {
		tdx_iommu_clear(iommu_state);
		return ret;
	}

	pr_info("Trusted DMA for TEE/IO initialized, regbase 0x%llx\n", drhd->reg_base_addr);

	return 0;
}

static void tdx_iommu_disable_all(void)
{
	struct tdx_iommu_state *iommu_state;
	unsigned long tdx_iommu_id;

	xa_for_each(&tdx_iommu_xa, tdx_iommu_id, iommu_state) {
		tdx_iommu_clear(iommu_state);
		dmar_tdxcs_iommu_exit(iommu_state->drhd);
	}

	xa_destroy(&tdx_iommu_xa);
}

int tdx_iommu_enable_all(void)
{
	int ret;

	ret = dmar_tdxcs_iommu_init(tdx_iommu_setup);
	if (ret)
		tdx_iommu_disable_all();

	return ret;
}
