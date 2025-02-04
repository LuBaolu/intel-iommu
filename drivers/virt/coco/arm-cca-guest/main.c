// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 ARM Ltd.
 */

#include <linux/arm-rsi-cmds.h>
#include <linux/arm-smccc-bus.h>
#include <linux/arm-smccc-rsi.h>
#include <linux/cc_platform.h>
#include <linux/kernel.h>
#include <linux/device-id/platform.h>
#include <linux/module.h>
#include <linux/smp.h>
#include <linux/tsm.h>
#include <linux/types.h>

#ifdef CONFIG_PCI_TSM
#include "rsi-da.h"
#include "rhi-da.h"
#endif

/**
 * struct arm_cca_token_info - a descriptor for the token buffer.
 * @granule:		PA of the granule to which the token will be written
 * @offset:		Offset within granule to start of buffer in bytes
 */
struct arm_cca_token_info {
	phys_addr_t     granule;
	unsigned long   offset;
};

/**
 * arm_cca_attestation_continue - Retrieve the attestation token data.
 *
 * @info: pointer to the arm_cca_token_info
 *
 * Attestation token generation is a long running operation and therefore
 * the token data may not be retrieved in a single call. Moreover, the
 * token retrieval operation must be requested on the same CPU on which the
 * attestation token generation was initialised.
 * This helper function must therefore be executed on the same CPU multiple
 * times until the entire token data is retrieved.
 */
static unsigned long
arm_cca_attestation_continue(struct arm_cca_token_info *info)
{
	unsigned long ret;
	unsigned long len;
	unsigned long size;

	size = RSI_GRANULE_SIZE - info->offset;
	ret = rsi_attestation_token_continue(info->granule, info->offset, size,
					     &len);
	info->offset += len;
	return ret;
}

/**
 * arm_cca_report_new - Generate a new attestation token.
 *
 * @report: pointer to the TSM report context information.
 * @data:  pointer to the context specific data for this module.
 *
 * Initialise the attestation token generation using the challenge data
 * passed in the TSM descriptor. Allocate memory for the attestation token
 * and retrieve the attestation token on the same CPU on which the
 * attestation token generation was initialised.
 *
 * The challenge data must be at least 32 bytes and no more than 64 bytes. If
 * less than 64 bytes are provided it will be zero padded to 64 bytes.
 *
 * Return:
 * * %0        - Attestation token generated successfully.
 * * %-EINVAL  - A parameter was not valid.
 * * %-ENOMEM  - Out of memory.
 * * %-EFAULT  - Failed to get IPA for memory page(s).
 */
static int arm_cca_report_new(struct tsm_report *report, void *data)
{
	int ret = 0;
	unsigned long rsi_result;
	long max_size;
	unsigned long token_size = 0;
	struct arm_cca_token_info info;
	void *buf;
	u8 *token __free(kvfree) = NULL;
	struct tsm_report_desc *desc = &report->desc;

	if (desc->inblob_len < 32 || desc->inblob_len > 64)
		return -EINVAL;

	/*
	 * The attestation token 'init' and 'continue' calls must be
	 * performed on the same CPU, so disable CPU migration around
	 * those operations.
	 */
	migrate_disable();

	max_size = rsi_attestation_token_init(desc->inblob, desc->inblob_len);
	if (max_size <= 0) {
		ret = -EINVAL;
		goto exit_migrate_enable;
	}

	/* Allocate outblob */
	token = kvzalloc(max_size, GFP_KERNEL);
	if (!token) {
		ret = -ENOMEM;
		goto exit_migrate_enable;
	}

	/*
	 * Since the outblob may not be physically contiguous, use a page
	 * to bounce the buffer from RMM.
	 */
	buf = alloc_pages_exact(RSI_GRANULE_SIZE, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto exit_migrate_enable;
	}

	/* Get the PA of the memory page(s) that were allocated */
	info.granule = (unsigned long)virt_to_phys(buf);

	/* Loop until the token is ready or there is an error */
	do {
		/* Retrieve one RSI_GRANULE_SIZE data per loop iteration */
		info.offset = 0;
		do {
			/*
			 * Retrieve a sub-granule chunk of data per loop
			 * iteration.
			 */
			rsi_result = arm_cca_attestation_continue(&info);
		} while (rsi_result == RSI_INCOMPLETE &&
			 info.offset < RSI_GRANULE_SIZE);

		/* Break out in case of failure */
		if (rsi_result != RSI_SUCCESS && rsi_result != RSI_INCOMPLETE) {
			ret = -ENXIO;
			token_size = 0;
			goto exit_free_granule_page;
		}

		/*
		 * Copy the retrieved token data from the granule
		 * to the token buffer, ensuring that the RMM doesn't
		 * overflow the buffer.
		 */
		if (WARN_ON(token_size + info.offset > max_size))
			break;
		memcpy(&token[token_size], buf, info.offset);
		token_size += info.offset;
	} while (rsi_result == RSI_INCOMPLETE);

	report->outblob = no_free_ptr(token);
exit_free_granule_page:
	report->outblob_len = token_size;
	free_pages_exact(buf, RSI_GRANULE_SIZE);
exit_migrate_enable:
	migrate_enable();
	return ret;
}

static const struct tsm_report_ops arm_cca_tsm_report_ops = {
	.name = KBUILD_MODNAME,
	.report_new = arm_cca_report_new,
};

static void unregister_cca_tsm_report(void *data)
{
	tsm_report_unregister(&arm_cca_tsm_report_ops);
}

#ifdef CONFIG_PCI_TSM

static int __maybe_unused
cca_update_dev_measurements(struct pci_dev *pdev, const u8 *nonce)
{
	int ret;
	void *measurements;
	size_t measurements_size;
	int vdev_id = rsi_vdev_id(pdev);
	struct pci_tsm_evidence *evidence;
	struct rsi_vdevice_info *dev_info;
	struct pci_tsm_evidence_object *obj;
	struct cca_guest_dsc *dsc = to_cca_guest_dsc(pdev);

	/* Regenerate the measurement from the device */
	ret = rhi_update_vdev_measurements_cache(pdev, nonce);
	if (ret) {
		pci_err(pdev, "failed to update device measurements from device (%d)\n", ret);
		return ret;
	}

	ret = rhi_read_cached_object(vdev_id, RHI_DA_OBJECT_MEASUREMENT,
				     &measurements, &measurements_size);
	if (ret) {
		pci_err(pdev, "failed to get device measurements from the host (%d)\n", ret);
		return ret;
	}

	dev_info = kmalloc(sizeof(*dev_info), GFP_KERNEL);
	if (!dev_info) {
		ret = -ENOMEM;
		goto free_measurements;
	}

	if (rsi_vdev_get_info(vdev_id, virt_to_phys(dev_info))) {
		pci_err(pdev, "failed to get device digests (%d)\n", ret);
		ret = -EIO;
		goto free_dev_info;
	}

	/* Make sure no unexpected lock/unlock operation happened from guest */
	if (dsc->dev_info.lock_nonce != dev_info->lock_nonce) {
		pci_err(pdev, "Unexpected lock/unlock operation from host (%d)\n", ret);
		ret = -EIO;
		goto free_dev_info;
	}

	/*
	 * Verify that the digests of the provided reports match with the
	 * digests from RMM
	 */
	ret = cca_verify_digest(dev_info->hash_algo, measurements,
				measurements_size, dev_info->meas_digest);
	if (ret) {
		pci_err(pdev, "RMM provided digest mismatch (%d)\n", ret);
		goto free_dev_info;
	}

	/* fill evidence details */
	evidence = &dsc->pci.base_tsm.evidence;

	/* Now update the evidence under lock. */
	down_write(&evidence->lock);
	evidence->generation = dev_info->meas_nonce;

	obj = &evidence->obj[PCI_TSM_EVIDENCE_TYPE_MEASUREMENTS];
	if (obj->data)
		kvfree(obj->data);
	obj->data = measurements;
	obj->len = measurements_size;

	dsc->dev_info.meas_nonce    = dev_info->meas_nonce;
	memcpy(dsc->dev_info.meas_digest, dev_info->meas_digest, SHA512_DIGEST_SIZE);
	up_write(&evidence->lock);

	kfree(dev_info);
	return 0;

free_dev_info:
	kfree(dev_info);
free_measurements:
	kvfree(measurements);
	return ret;
}

static int cca_collect_dev_evidence(struct pci_dev *pdev, struct cca_guest_dsc *dsc)
{
	int ret;
	int vdev_id = rsi_vdev_id(pdev);
	struct pci_tsm_evidence *evidence;
	struct rsi_vdevice_info *dev_info;
	struct pci_tsm_evidence_object *obj;
	void *certificate, *vca, *interface_report, *measurements;
	size_t certificate_size, vca_size, interface_report_size, measurements_size;

	/* Regenerate interface report and measurement from the device */
	ret = cca_update_device_object_cache(pdev, NULL);
	if (ret) {
		pci_err(pdev, "failed to update device objects from device (%d)\n", ret);
		return ret;
	}

	ret = rhi_read_cached_object(vdev_id, RHI_DA_OBJECT_CERTIFICATE,
				     &certificate, &certificate_size);
	if (ret) {
		pci_err(pdev, "failed to get device certificate from the host (%d)\n", ret);
		return ret;
	}

	ret = rhi_read_cached_object(vdev_id, RHI_DA_OBJECT_VCA, &vca, &vca_size);
	if (ret) {
		pci_err(pdev, "failed to get device VCA from the host (%d)\n", ret);
		goto free_certificate;
	}

	ret = rhi_read_cached_object(vdev_id, RHI_DA_OBJECT_INTERFACE_REPORT,
				     &interface_report, &interface_report_size);
	if (ret) {
		pci_err(pdev, "failed to get interface report from the host (%d)\n", ret);
		goto free_vca;
	}

	ret = rhi_read_cached_object(vdev_id, RHI_DA_OBJECT_MEASUREMENT,
				     &measurements, &measurements_size);
	if (ret) {
		pci_err(pdev, "failed to get device certificate from the host (%d)\n", ret);
		goto free_interface_report;
	}

	dev_info = kmalloc(sizeof(*dev_info), GFP_KERNEL);
	if (!dev_info) {
		ret = -ENOMEM;
		goto free_measurements;
	}

	if (rsi_vdev_get_info(vdev_id, virt_to_phys(dev_info))) {
		pci_err(pdev, "failed to get device digests (%d)\n", ret);
		ret = -EIO;
		goto free_dev_info;
	}

	/* Make sure no unexpected lock/unlock operation happened from guest */
	if (dsc->dev_info.lock_nonce != dev_info->lock_nonce) {
		pci_err(pdev, "Unexpected lock/unlock operation from host (%d)\n", ret);
		ret = -EIO;
		goto free_dev_info;
	}

	/*
	 * Verify that the digests of the provided reports match with the
	 * digests from RMM
	 */
	ret = cca_verify_digests(dev_info->hash_algo, certificate,
				 certificate_size, vca, vca_size,
				 interface_report, interface_report_size,
				 measurements, measurements_size, dev_info);
	if (ret) {
		pci_err(pdev, "RMM provided digest mismatch (%d)\n", ret);
		goto free_dev_info;
	}

	/* fill evidence details */
	evidence = &dsc->pci.base_tsm.evidence;

	/* Now update the evidence under lock. */
	down_write(&evidence->lock);
	evidence->generation = dev_info->meas_nonce;

	/* we default to slot 0 in pdev_create */
	obj = &evidence->obj[PCI_TSM_EVIDENCE_TYPE_CERT0];
	WARN_ON(obj->data);
	obj->data = certificate;
	obj->len = certificate_size;

	obj = &evidence->obj[PCI_TSM_EVIDENCE_TYPE_VCA];
	WARN_ON(obj->data);
	obj->data = vca;
	obj->len = vca_size;

	obj = &evidence->obj[PCI_TSM_EVIDENCE_TYPE_REPORT];
	WARN_ON(obj->data);
	obj->data = interface_report;
	obj->len = interface_report_size;

	obj = &evidence->obj[PCI_TSM_EVIDENCE_TYPE_MEASUREMENTS];
	WARN_ON(obj->data);
	obj->data = measurements;
	obj->len = measurements_size;

	dsc->dev_info.meas_nonce    = dev_info->meas_nonce;
	dsc->dev_info.report_nonce  = dev_info->report_nonce;
	memcpy(dsc->dev_info.cert_digest, dev_info->identity_digest, SHA512_DIGEST_SIZE);
	memcpy(dsc->dev_info.vca_digest, dev_info->protocol_data_digest, SHA512_DIGEST_SIZE);
	memcpy(dsc->dev_info.meas_digest, dev_info->meas_digest, SHA512_DIGEST_SIZE);
	memcpy(dsc->dev_info.report_digest, dev_info->report_digest, SHA512_DIGEST_SIZE);
	up_write(&evidence->lock);

	kfree(dev_info);
	return 0;

free_dev_info:
	kfree(dev_info);
free_measurements:
	kvfree(measurements);
free_interface_report:
	kvfree(interface_report);
free_vca:
	kvfree(vca);
free_certificate:
	kvfree(certificate);
	return ret;
}

static struct pci_tsm *cca_tsm_lock(struct tsm_dev *tsm_dev, struct pci_dev *pdev)
{
	int ret;
	enum hash_algo digest_algo;
	struct cca_guest_dsc *cca_dsc;
	int vdev_id = rsi_vdev_id(pdev);
	struct rsi_vdevice_info *dev_info;

	cca_dsc = kzalloc_obj(struct cca_guest_dsc);
	if (!cca_dsc)
		return ERR_PTR(-ENOMEM);

	ret = pci_tsm_devsec_constructor(pdev, &cca_dsc->pci, tsm_dev);
	if (ret)
		goto free_cca_dsc;

	ret = cca_device_lock(pdev);
	if (ret)
		goto free_cca_dsc;

	dev_info = kmalloc_obj(struct rsi_vdevice_info);
	if (!dev_info) {
		ret = -ENOMEM;
		goto dev_unlock;
	}

	if (rsi_vdev_get_info(vdev_id, virt_to_phys(dev_info))) {
		ret = -EIO;
		goto free_dev_info;
	}

	/* collect the lock nonce */
	cca_dsc->dev_info.lock_nonce = dev_info->lock_nonce;

	switch (dev_info->hash_algo) {
	case RSI_HASH_SHA_256:
		digest_algo = HASH_ALGO_SHA256;
		break;
	case RSI_HASH_SHA_512:
		digest_algo = HASH_ALGO_SHA512;
		break;
	default:
		ret = -EIO;
		goto free_dev_info;
	}
	pci_tsm_init_evidence(&cca_dsc->pci.base_tsm.evidence,
			      dev_info->id_index, digest_algo);

	/* collect evidence without nonce */
	ret = cca_collect_dev_evidence(pdev, cca_dsc);
	if (ret)
		goto free_dev_info;

	kfree(dev_info);
	return &cca_dsc->pci.base_tsm;

free_dev_info:
	kfree(dev_info);
dev_unlock:
	cca_device_unlock(pdev);
free_cca_dsc:
	kfree(cca_dsc);
	return ERR_PTR(ret);
}

static void cca_tsm_unlock(struct pci_tsm *tsm)
{
	long ret;
	struct pci_dev *pdev = tsm->pdev;
	struct cca_guest_dsc *cca_dsc = to_cca_guest_dsc(pdev);

	/* invalidate dev mapping based on interface report */
	ret = cca_unmap_evidence_report_range(tsm->pdev);
	if (ret) {
		pci_err(tsm->pdev, "failed to invalidate the interface report\n");
		goto err_out;
	}

	cca_device_unlock(tsm->pdev);
	if (cca_dsc->pci.mmio)
		pci_tsm_mmio_teardown(cca_dsc->pci.mmio);

err_out:
	/*
	 * No error handling from this function. Leave the device locked
	 */
	if (cca_dsc->pci.mmio)
		pci_tsm_mmio_free(tsm->pdev, cca_dsc->pci.mmio);
	kfree(cca_dsc);
}

static struct pci_tsm_ops cca_devsec_pci_ops = {
	.lock = cca_tsm_lock,
	.unlock = cca_tsm_unlock,
};

static void cca_devsec_tsm_remove(void *tsm_dev)
{
	tsm_unregister(tsm_dev);
}

static int cca_devsec_tsm_register(struct arm_smccc_device *sdev)
{
	struct tsm_dev *tsm_dev;

	tsm_dev = tsm_register(&sdev->dev, &cca_devsec_pci_ops);
	if (IS_ERR(tsm_dev))
		return PTR_ERR(tsm_dev);

	return devm_add_action_or_reset(&sdev->dev, cca_devsec_tsm_remove, tsm_dev);
}
#endif /* CONFIG_PCI_TSM */

static int cca_devsec_tsm_probe(struct arm_smccc_device *sdev)
{
	int ret;

	if (!is_realm_world())
		return -ENODEV;

	ret = tsm_report_register(&arm_cca_tsm_report_ops, NULL);
	if (ret < 0) {
		dev_err_probe(&sdev->dev, ret, "Error registering with TSM\n");
		return ret;
	}

	ret = devm_add_action_or_reset(&sdev->dev, unregister_cca_tsm_report,
				       NULL);
	if (ret < 0) {
		dev_err_probe(&sdev->dev, ret, "Error registering devm action\n");
		return ret;
	}

#ifdef CONFIG_PCI_TSM
	/* Allow tsm report even if tsm_register fails */
	if (rsi_has_da_feature() && rhi_has_da_support())
		cca_devsec_tsm_register(sdev);
#endif

	return 0;
}

static const struct arm_smccc_device_id cca_devsec_tsm_id_table[] = {
	{ .func_id = SMC_RSI_ABI_VERSION },
	{}
};
MODULE_DEVICE_TABLE(arm_smccc, cca_devsec_tsm_id_table);

static struct arm_smccc_driver cca_devsec_tsm_driver = {
	.name = KBUILD_MODNAME,
	.probe = cca_devsec_tsm_probe,
	.id_table = cca_devsec_tsm_id_table,
};
module_arm_smccc_driver(cca_devsec_tsm_driver);
MODULE_AUTHOR("Sami Mujawar <sami.mujawar@arm.com>");
MODULE_AUTHOR("Aneesh Kumar <aneesh.kumar@kernel.org>");
MODULE_DESCRIPTION("Arm CCA Guest TSM Driver");
MODULE_LICENSE("GPL");
