// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 ARM Ltd.
 */

#include <linux/arm-smccc.h>
#include <linux/arm-smccc-bus.h>
#include <linux/pci-tsm.h>
#include <linux/pci-ide.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/tsm.h>
#include <linux/vmalloc.h>
#include <linux/cleanup.h>

#include "rmi-da.h"

static struct pci_tsm *cca_tsm_pci_probe(struct tsm_dev *tsm_dev, struct pci_dev *pdev)
{
	int ret;

	if (!is_pci_tsm_pf0(pdev)) {
		struct cca_host_fn_dsc *fn_dsc __free(kfree) =
			kzalloc(sizeof(*fn_dsc), GFP_KERNEL);

		if (!fn_dsc)
			return NULL;

		ret = pci_tsm_link_constructor(pdev, &fn_dsc->pci, tsm_dev);
		if (ret)
			return NULL;

		return &no_free_ptr(fn_dsc)->pci;
	}

	if (!pdev->ide_cap)
		return NULL;

	struct cca_host_pf0_ep_dsc *pf0_ep_dsc __free(kfree) =
		kzalloc(sizeof(*pf0_ep_dsc), GFP_KERNEL);
	if (!pf0_ep_dsc)
		return NULL;

	ret = pci_tsm_pf0_constructor(pdev, &pf0_ep_dsc->pci, tsm_dev);
	if (ret)
		return NULL;

	pci_dbg(pdev, "tsm enabled\n");
	return &no_free_ptr(pf0_ep_dsc)->pci.base_tsm;
}

static void cca_tsm_pci_remove(struct pci_tsm *tsm)
{
	struct pci_dev *pdev = tsm->pdev;

	if (is_pci_tsm_pf0(pdev)) {
		struct cca_host_pf0_ep_dsc *pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev);

		pci_tsm_pf0_destructor(&pf0_ep_dsc->pci);
		kfree(pf0_ep_dsc);
	} else {
		kfree(to_cca_fn_dsc(pdev));
	}
}

static inline bool cca_pdev_need_sel_ide_streams(struct pci_dev *pdev)
{
	return pci_pcie_type(pdev) == PCI_EXP_TYPE_ENDPOINT;
}

static int __maybe_unused cca_tsm_connect(struct pci_dev *pdev)
{
	struct pci_dev *rp = pcie_find_root_port(pdev);
	struct cca_host_pf0_ep_dsc *pf0_ep_dsc;
	struct pci_ide *ide;
	int ret, stream_id = 0;

	/* Only function 0 supports connect in host */
	if (WARN_ON(!is_pci_tsm_pf0(pdev)))
		return -EIO;

	pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev);
	if (cca_pdev_need_sel_ide_streams(pdev)) {

		ide = pci_ide_stream_alloc(pdev);
		if (!ide) {
			ret = -ENOMEM;
			goto err_stream_alloc;
		}

		pf0_ep_dsc->sel_stream = ide;
		/*
		 * keep the stream id simple by using the host-bridge id
		 */
		stream_id = ide->host_bridge_stream;
		ide->stream_id = stream_id;
		ret = pci_ide_stream_register(ide);
		if (ret)
			goto err_stream;
		/*
		 * Configure IDE capability for target device
		 *
		 * Some test devices work only with DEFAULT_STREAM enabled.
		 * For simplicity, enable DEFAULT_STREAM for all devices. A
		 * future decent solution may be to have a quirk table to
		 * specify which devices need DEFAULT_STREAM.
		 */
		ide->partner[PCI_IDE_EP].default_stream = 1;
		pci_ide_stream_setup(pdev, ide);
		pci_ide_stream_setup(rp, ide);

		/*
		 * Once ide is setup, enable the stream at the endpoint
		 * Root port will be done by RMM
		 */
		pci_ide_stream_enable(pdev, ide);
	}
	return 0;

err_stream:
	if (cca_pdev_need_sel_ide_streams(pdev))
		pci_ide_stream_free(ide);
	pf0_ep_dsc->sel_stream = NULL;
err_stream_alloc:

	return ret;
}

static void __maybe_unused cca_tsm_disconnect(struct pci_dev *pdev)
{
	struct pci_ide *ide;
	struct cca_host_pf0_ep_dsc *pf0_ep_dsc;

	pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev);
	if (!pf0_ep_dsc)
		return;

	if (cca_pdev_need_sel_ide_streams(pdev)) {
		ide = pf0_ep_dsc->sel_stream;

		pci_ide_stream_release(ide);
		pf0_ep_dsc->sel_stream = NULL;
	}

}

static struct pci_tsm_ops cca_link_pci_ops = {
	.probe = cca_tsm_pci_probe,
	.remove = cca_tsm_pci_remove,
};

static void cca_link_tsm_remove(void *tsm_dev)
{
	tsm_unregister(tsm_dev);
}

static bool rmi_has_reg2_feature(unsigned long feature)
{
	return !!u64_get_bits(rmi_feat_reg(2), feature);
}

static int cca_link_tsm_probe(struct arm_smccc_device *sdev)
{
	struct tsm_dev *tsm_dev;

	if (!rmi_has_reg2_feature(RMI_FEATURE_REGISTER_2_DA))
		return -ENODEV;

	tsm_dev = tsm_register(&sdev->dev, &cca_link_pci_ops);
	if (IS_ERR(tsm_dev))
		return PTR_ERR(tsm_dev);

	return devm_add_action_or_reset(&sdev->dev, cca_link_tsm_remove,
					tsm_dev);
}

static const struct arm_smccc_device_id cca_link_tsm_id_table[] = {
	{ .func_id = SMC_RMI_VERSION },
	{}
};
MODULE_DEVICE_TABLE(arm_smccc, cca_link_tsm_id_table);

static struct arm_smccc_driver cca_link_tsm_driver = {
	.name = KBUILD_MODNAME,
	.probe = cca_link_tsm_probe,
	.id_table = cca_link_tsm_id_table,
};
module_arm_smccc_driver(cca_link_tsm_driver);
MODULE_IMPORT_NS("PCI_IDE");
MODULE_AUTHOR("Aneesh Kumar <aneesh.kumar@kernel.org>");
MODULE_DESCRIPTION("ARM CCA Host TSM driver");
MODULE_LICENSE("GPL");
