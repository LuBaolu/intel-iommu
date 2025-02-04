// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 ARM Ltd.
 */

#include <linux/arm-rsi-cmds.h>
#include <linux/cc_shared.h>
#include <linux/mem_encrypt.h>
#include <linux/pci.h>
#include <crypto/hash.h>

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

static inline int
rsi_validate_dev_mapping(unsigned long vdev_id, phys_addr_t start_ipa,
		phys_addr_t end_ipa, phys_addr_t io_pa,
		unsigned long flags, unsigned long lock_nonce,
		unsigned long meas_nonce, unsigned long report_nonce)
{
	unsigned long ret;
	phys_addr_t next_ipa;

	while (start_ipa < end_ipa) {
		ret = rsi_vdev_validate_mapping(vdev_id, start_ipa, end_ipa,
						io_pa, &next_ipa, flags,
						lock_nonce, meas_nonce, report_nonce);
		if (ret || next_ipa <= start_ipa || next_ipa > end_ipa)
			return -EINVAL;
		io_pa += next_ipa - start_ipa;
		start_ipa = next_ipa;
	}
	return 0;
}

static inline int rsi_invalidate_dev_mapping(phys_addr_t start_ipa, phys_addr_t end_ipa)
{
	return rsi_set_memory_range(start_ipa, end_ipa, RSI_RIPAS_EMPTY,
				    RSI_CHANGE_DESTROYED);
}

static int cca_apply_evidence_report_range(struct pci_dev *pdev,
		struct pci_tsm_mmio *mmio, bool map)
{
	int i, ret;
	struct resource *res;
	unsigned long mmio_flags = 0; /* non coherent, not limited order */
	int vdev_id = rsi_vdev_id(pdev);
	struct pci_tsm_mmio_entry *entry;
	struct cca_guest_dsc *dsc = to_cca_guest_dsc(pdev);

	for (i = 0; i < mmio->nr; i++) {
		entry = pci_tsm_mmio_entry(mmio, i);
		res = &entry->res;

		if (res->desc != IORES_DESC_ENCRYPTED)
			continue;

		if (map)
			ret = rsi_validate_dev_mapping(vdev_id, res->start,
						       res->end + 1, entry->tsm_offset,
						       mmio_flags,
						       dsc->dev_info.lock_nonce,
						       dsc->dev_info.meas_nonce,
						       dsc->dev_info.report_nonce);
		else
			ret = rsi_invalidate_dev_mapping(res->start, res->end + 1);
		if (ret)
			return ret;
	}
	return 0;
}

int cca_map_evidence_report_range(struct pci_dev *pdev, struct pci_tsm_mmio *mmio)
{
	return cca_apply_evidence_report_range(pdev, mmio, true);
}

int cca_unmap_evidence_report_range(struct pci_dev *pdev)
{
	struct cca_guest_dsc *dsc = to_cca_guest_dsc(pdev);
	struct pci_tsm_mmio *tsm_mmio = dsc->pci.mmio;

	if (tsm_mmio)
		return cca_apply_evidence_report_range(pdev, tsm_mmio, false);
	return 0;
}

int cca_verify_digest(u64 hash_algo, uint8_t *report,
		size_t report_size, uint8_t *report_digest)
{
	u8 digest[SHA512_DIGEST_SIZE];
	size_t digest_size;
	void (*digest_func)(const u8 *data, size_t len, u8 *out);

	switch (hash_algo) {
	case RSI_HASH_SHA_256:
		digest_func = sha256;
		digest_size = SHA256_DIGEST_SIZE;
		break;
	case RSI_HASH_SHA_512:
		digest_func = sha512;
		digest_size = SHA512_DIGEST_SIZE;
		break;
	default:
		return -EINVAL;
	}

	digest_func(report, report_size, digest);
	if (memcmp(report_digest, digest, digest_size))
		return -EINVAL;

	return 0;
}

int cca_verify_digests(u64 hash_algo,
		uint8_t *certificate, size_t certificate_size,
		uint8_t *vca, size_t vca_size,
		uint8_t *interface_report, size_t interface_report_size,
		uint8_t *measurements, size_t measurements_size,
		struct rsi_vdevice_info *dev_info)
{
	int ret;
	struct {
		uint8_t *report;
		size_t size;
		uint8_t *digest;
	} reports[] = {
		{
			certificate,
			certificate_size,
			dev_info->identity_digest
		},
		{
			vca,
			vca_size,
			dev_info->protocol_data_digest
		},
		{
			interface_report,
			interface_report_size,
			dev_info->report_digest
		},
		{
			measurements,
			measurements_size,
			dev_info->meas_digest
		}

	};

	for (int i = 0; i < ARRAY_SIZE(reports); i++) {
		ret = cca_verify_digest(hash_algo, reports[i].report,
					reports[i].size, reports[i].digest);
		if (ret)
			return ret;
	}
	return 0;
}
