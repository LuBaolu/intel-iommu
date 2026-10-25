// SPDX-License-Identifier: GPL-2.0
/*
 * TDX IOMMU Extensions support
 *
 * Copyright (C) 2026 Intel Corporation
 */
#define pr_fmt(fmt) "tdx-host: " fmt

#include <linux/dmar.h>
#include <linux/list.h>
#include <linux/mutex.h>
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

/**
 * struct tdx_iommu - Host state of an IOMMU in Secure TDX Mode
 * @node: Link in tdx_iommu_list
 * @params: IOMMU information from the VT-d driver
 * @tdx_iommu_id: Identifier returned by TDH.IOMMU.SETUP
 * @mt_pages: Pages handed to the TDX module by TDH.IOMMU.SETUP
 * @users: Number of devices using trusted DMA through this IOMMU
 * @stuck: TDH.IOMMU.CLEAR failed. The TDX module may still own
 *	   @mt_pages, so they are leaked and the IOMMU is not reused.
 */
struct tdx_iommu {
	struct list_head node;
	struct dmar_tdxc_params params;
	u64 tdx_iommu_id;
	struct tdx_iommu_pages *mt_pages;
	unsigned int users;
	bool stuck;
};

/*
 * Serializes the refcounts together with TDH.IOMMU.SETUP and
 * TDH.IOMMU.CLEAR, so a device can't join an IOMMU that is being
 * cleared, or one whose setup hasn't finished.
 */
static DEFINE_MUTEX(tdx_iommu_lock);
static LIST_HEAD(tdx_iommu_list);

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
	 * Populate the parameter for the TDH.IOMMU.SETUP SEAMCALL according to
	 * the format defined in "Table 3.35: Structure of IOMMU_MT Parameter"
	 * of the ABI reference specification.
	 */
	for (i = 0; i < nr_entries; i++) {
		entries[i] = __pa(pages[i]);
		if (i < IQ_BUFFERS_NUM)
			entries[i] |= IQ_BUFFER_PAGES;
	}
}

static struct tdx_iommu_pages *alloc_iommu_mt(int node, unsigned int nr_mt_pages)
{
	unsigned int nr_entries = nr_mt_pages + IQ_BUFFERS_NUM;
	struct tdx_iommu_pages *array;

	if (!nr_mt_pages || nr_mt_pages > MT_MAX_ENTRIES - IQ_BUFFERS_NUM)
		return NULL;

	array = kzalloc_obj(*array);
	if (!array)
		return NULL;

	array->root = tdx_iommu_alloc_pages(node, MT_ROOT_SIZE);
	if (!array->root)
		goto free_array;

	array->nr_entries = nr_entries;
	array->pages = alloc_mt_pages(nr_entries, node);
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

static struct tdx_iommu *tdx_iommu_find(u64 reg_base_addr)
{
	struct tdx_iommu *tiommu;

	lockdep_assert_held(&tdx_iommu_lock);

	list_for_each_entry(tiommu, &tdx_iommu_list, node)
		if (tiommu->params.reg_base_addr == reg_base_addr)
			return tiommu;

	return NULL;
}

/* Number of 4K pages backing IOMMU_MT entry @i. */
static unsigned int mt_entry_nr_pages(unsigned int i)
{
	return i < IQ_BUFFERS_NUM ? IQ_BUFFER_SIZE >> PAGE_SHIFT : 1;
}

/*
 * Drop the PAMT backing of the first @nr 4K pages handed to the TDX
 * module, in the order mt_pamt_get() took it. Pass UINT_MAX for all.
 */
static void mt_pamt_put(struct tdx_iommu_pages *mt, unsigned int nr)
{
	unsigned int i, j;

	for (i = 0; i < mt->nr_entries && nr; i++) {
		unsigned long pfn = PHYS_PFN(__pa(mt->pages[i]));

		for (j = 0; j < mt_entry_nr_pages(i) && nr; j++, nr--)
			tdx_pamt_put(pfn + j);
	}
}

/*
 * With Dynamic PAMT, every 4K page handed to the TDX module needs PAMT
 * backing first. This covers the IQ buffers and the metadata pages, not
 * the root page, which only carries the list of pages into the
 * SEAMCALL. tdx_pamt_get() is a no-op without Dynamic PAMT.
 */
static int mt_pamt_get(struct tdx_iommu_pages *mt)
{
	unsigned int i, j, nr = 0;
	int ret;

	for (i = 0; i < mt->nr_entries; i++) {
		unsigned long pfn = PHYS_PFN(__pa(mt->pages[i]));

		for (j = 0; j < mt_entry_nr_pages(i); j++) {
			ret = tdx_pamt_get(pfn + j, NULL);
			if (ret) {
				mt_pamt_put(mt, nr);
				return ret;
			}
			nr++;
		}
	}

	return 0;
}

/* Allocate the IOMMU_MT pages and issue TDH.IOMMU.SETUP. */
static int tdx_iommu_setup(struct pci_dev *pdev, struct tdx_iommu *tiommu)
{
	const struct tdx_sys_info *sysinfo = tdx_get_sysinfo();
	unsigned int nr_mt_pages = sysinfo->tdx_connect.iommu_mt_page_count;
	struct dmar_tdxc_params *params = &tiommu->params;
	u64 r, tdx_iommu_id;
	int ret;

	struct tdx_iommu_pages *mt __free(free_mt_pages) =
			alloc_iommu_mt(params->node, nr_mt_pages);
	if (!mt)
		return -ENOMEM;

	/* Before init_prepare(), so nothing allocates with ECMD blocked. */
	ret = mt_pamt_get(mt);
	if (ret)
		return ret;

	ret = dmar_tdxc_init_prepare(pdev, params);
	if (ret)
		goto err_pamt_put;

	r = tdh_iommu_setup(params->reg_base_addr, mt->root, &tdx_iommu_id);
	dmar_tdxc_init_done(pdev, params, !r);
	if (r) {
		pr_err("TDH.IOMMU.SETUP failed, regbase 0x%llx status 0x%llx\n",
		       params->reg_base_addr, r);
		ret = -EIO;
		goto err_pamt_put;
	}

	tiommu->tdx_iommu_id = tdx_iommu_id;
	tiommu->mt_pages = no_free_ptr(mt);

	pr_info("Trusted DMA initialized, regbase 0x%llx\n", params->reg_base_addr);

	return 0;

err_pamt_put:
	mt_pamt_put(mt, UINT_MAX);
	return ret;
}

/* Issue TDH.IOMMU.CLEAR and release what TDH.IOMMU.SETUP took. */
static int tdx_iommu_clear(struct pci_dev *pdev, struct tdx_iommu *tiommu)
{
	u64 r;

	r = tdh_iommu_clear(tiommu->tdx_iommu_id);
	if (r) {
		pr_err("TDH.IOMMU.CLEAR failed, regbase 0x%llx tdx_iommu_id 0x%llx status 0x%llx\n",
		       tiommu->params.reg_base_addr, tiommu->tdx_iommu_id, r);
		/* The pages and their PAMT backing stay with the TDX module. */
		return -EIO;
	}

	dmar_tdxc_release(pdev, &tiommu->params);
	mt_pamt_put(tiommu->mt_pages, UINT_MAX);
	free_mt_pages(tiommu->mt_pages);
	tiommu->mt_pages = NULL;

	return 0;
}

/*
 * tdx_iommu_get() - Enable trusted DMA on the IOMMU translating a device
 * @pdev: PCI device that will use trusted DMA
 *
 * The first device behind an IOMMU transitions that IOMMU into Secure
 * TDX Mode. Later devices only take a reference. Every successful call
 * must be balanced by tdx_iommu_put().
 *
 * Return: 0 on success and error number on failure.
 */
int tdx_iommu_get(struct pci_dev *pdev)
{
	struct dmar_tdxc_params params;
	struct tdx_iommu *tiommu;
	int ret;

	ret = dmar_tdxc_get_params(pdev, &params);
	if (ret)
		return ret;

	guard(mutex)(&tdx_iommu_lock);

	tiommu = tdx_iommu_find(params.reg_base_addr);
	if (tiommu) {
		if (tiommu->stuck)
			return -EBUSY;
		if (WARN_ON(tiommu->users == UINT_MAX))
			return -EOVERFLOW;
		tiommu->users++;
		return 0;
	}

	tiommu = kzalloc_obj(*tiommu);
	if (!tiommu)
		return -ENOMEM;

	tiommu->params = params;
	ret = tdx_iommu_setup(pdev, tiommu);
	if (ret) {
		kfree(tiommu);
		return ret;
	}

	tiommu->users = 1;
	list_add(&tiommu->node, &tdx_iommu_list);

	return 0;
}

/*
 * tdx_iommu_put() - Drop a reference taken by tdx_iommu_get()
 * @pdev: Same device passed to tdx_iommu_get()
 *
 * The last device behind an IOMMU takes that IOMMU out of Secure TDX
 * Mode. If TDH.IOMMU.CLEAR fails, the TDX module may still own the
 * IOMMU_MT pages, so they are leaked. The IOMMU then stays in Secure TDX
 * Mode, and later tdx_iommu_get() calls for it fail.
 */
void tdx_iommu_put(struct pci_dev *pdev)
{
	struct dmar_tdxc_params params;
	struct tdx_iommu *tiommu;

	if (WARN_ON(dmar_tdxc_get_params(pdev, &params)))
		return;

	guard(mutex)(&tdx_iommu_lock);

	tiommu = tdx_iommu_find(params.reg_base_addr);
	if (WARN_ON(!tiommu || tiommu->stuck || !tiommu->users))
		return;

	if (--tiommu->users)
		return;

	if (tdx_iommu_clear(pdev, tiommu)) {
		tiommu->stuck = true;
		return;
	}

	list_del(&tiommu->node);
	kfree(tiommu);
}
