/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 ARM Ltd.
 */

#ifndef _VIRT_COCO_RSI_DA_H_
#define _VIRT_COCO_RSI_DA_H_

#include <linux/pci.h>
#include <linux/pci-tsm.h>
#include <linux/arm-smccc-rsi.h>
#include <crypto/sha2.h>

#define MAX_CACHE_OBJ_SIZE	SZ_16M

struct dsm_device_info {
	u64 lock_nonce;
	u64 meas_nonce;
	u64 report_nonce;
	u8 cert_digest[SHA512_DIGEST_SIZE];
	u8 vca_digest[SHA512_DIGEST_SIZE];
	u8 meas_digest[SHA512_DIGEST_SIZE];
	u8 report_digest[SHA512_DIGEST_SIZE];
};

struct cca_guest_dsc {
	struct pci_tsm_devsec pci;
	struct dsm_device_info dev_info;
};

static inline struct cca_guest_dsc *to_cca_guest_dsc(struct pci_dev *pdev)
{
	struct pci_tsm *tsm = pdev->tsm;

	if (!tsm)
		return NULL;
	return container_of(tsm, struct cca_guest_dsc, pci.base_tsm);
}

/*
 * Linux use device requester id as the vdev id.
 */
static inline int rsi_vdev_id(struct pci_dev *pdev)
{
	return (pci_domain_nr(pdev->bus) << 16) |
	       PCI_DEVID(pdev->bus->number, pdev->devfn);
}

int cca_device_lock(struct pci_dev *pdev);
int cca_device_unlock(struct pci_dev *pdev);
int cca_update_device_object_cache(struct pci_dev *pdev, const u8 *nonce);
struct page *alloc_shared_pages(int nid, gfp_t gfp_mask, unsigned long min_size);
int free_shared_pages(struct page *page, unsigned long min_size);
int cca_map_evidence_report_range(struct pci_dev *pdev, struct pci_tsm_mmio *mmio);
int cca_unmap_evidence_report_range(struct pci_dev *pdev);
int cca_verify_digest(u64 hash_algo, uint8_t *report,
		size_t report_size, uint8_t *report_digest);
int cca_verify_digests(u64 hash_algo,
		uint8_t *certificate, size_t certificate_size,
		uint8_t *vca, size_t vca_size,
		uint8_t *interface_report, size_t interface_report_size,
		uint8_t *measurements, size_t measurements_size,
		struct rsi_vdevice_info *dev_info);
int cca_device_accept(struct pci_dev *pdev, unsigned long lock_nonce);

#endif
