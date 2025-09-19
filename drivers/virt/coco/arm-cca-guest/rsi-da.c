// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 ARM Ltd.
 */

#include <linux/arm-rsi-cmds.h>
#include <linux/cc_shared.h>
#include <linux/mem_encrypt.h>
#include <linux/pci.h>

#include "rsi-da.h"
#include "rhi-da.h"

int cca_device_lock(struct pci_dev *pdev)
{
	int ret;

	ret = rhi_vdev_set_tdi_state(pdev, RHI_DA_TDI_CONFIG_LOCKED);
	if (ret) {
		pci_err(pdev, "failed to lock the device (%d)\n", ret);
		return ret;
	}
	return 0;
}

int cca_device_unlock(struct pci_dev *pdev)
{
	int ret;

	ret = rhi_vdev_set_tdi_state(pdev, RHI_DA_TDI_CONFIG_UNLOCKED);
	if (ret) {
		pci_err(pdev, "failed to unlock the device (%d)\n", ret);
		return ret;
	}
	return 0;
}

struct page *alloc_shared_pages(int nid, gfp_t gfp_mask, unsigned long min_size)
{
	int ret;
	struct page *page;
	int page_order = get_order(ALIGN(min_size, cc_shared_granule_size()));

	page = alloc_pages_node(nid, gfp_mask | __GFP_ZERO, page_order);
	if (!page)
		return NULL;

	ret = set_memory_decrypted((unsigned long)page_address(page),
				   1 << page_order);
	/*
	 * If set_memory_decrypted() fails then we don't know what state the
	 * page is in, so we can't free it. Instead we leak it.
	 * set_memory_decrypted() will already have WARNed.
	 */
	if (ret)
		return NULL;

	return page;
}

int free_shared_pages(struct page *page, unsigned long size)
{
	int ret;
	int page_order = get_order(ALIGN(size, cc_shared_granule_size()));

	ret = set_memory_encrypted((unsigned long)page_address(page), 1 << page_order);
	/* If we fail to mark it encrypted don't free it back */
	if (ret)
		return ret;

	__free_pages(page, page_order);
	return 0;
}

int cca_update_device_object_cache(struct pci_dev *pdev, const u8 *nonce)
{
	int ret;

	ret = rhi_update_vdev_interface_report_cache(pdev);
	if (ret) {
		pci_err(pdev, "failed to get interface report (%d)\n", ret);
		return ret;
	}

	return rhi_update_vdev_measurements_cache(pdev, nonce);
}
