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
#include <linux/cleanup.h>
#include <linux/pci-doe.h>

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

	struct cca_host_pf0_ep_dsc *pf0_ep_dsc __free(kfree) =
		kzalloc(sizeof(*pf0_ep_dsc), GFP_KERNEL);
	if (!pf0_ep_dsc)
		return NULL;

	/* if device have ide cap, setup doe mailbox */
	if (pdev->ide_cap) {
		struct pci_doe_mb *doe_mb;

		doe_mb = pci_find_doe_mailbox(pdev, PCI_VENDOR_ID_PCI_SIG,
					      PCI_DOE_FEATURE_CMA);
		if (!doe_mb)
			return NULL;
		pf0_ep_dsc->pci.doe_mb = doe_mb;
	}

	ret = pci_tsm_pf0_constructor(pdev, &pf0_ep_dsc->pci, tsm_dev);
	if (ret)
		return NULL;
	mutex_init(&pf0_ep_dsc->pdev.object_lock);

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

static int init_dev_communication_buffers(struct pci_dev *pdev,
		struct cca_host_comm_data *comm_data)
{
	int ret = -ENOMEM;

	comm_data->io_params = (struct rmi_dev_comm_data *)get_zeroed_page(GFP_KERNEL);
	if (!comm_data->io_params)
		goto err_out;

	comm_data->rsp_buff = (void *)__get_free_page(GFP_KERNEL);
	if (!comm_data->rsp_buff)
		goto err_res_buff;

	comm_data->req_buff = (void *)__get_free_page(GFP_KERNEL);
	if (!comm_data->req_buff)
		goto err_req_buff;

	comm_data->work_queue = alloc_ordered_workqueue("%s %s DEV_COMM", 0,
						dev_bus_name(&pdev->dev),
						pci_name(pdev));
	if (!comm_data->work_queue)
		goto err_work_queue;

	comm_data->io_params->enter.status = RMI_DEV_COMM_NONE;
	comm_data->io_params->enter.resp_addr = virt_to_phys(comm_data->rsp_buff);
	comm_data->io_params->enter.req_addr  = virt_to_phys(comm_data->req_buff);
	comm_data->io_params->enter.resp_len = 0;

	return 0;

err_work_queue:
	free_page((unsigned long)comm_data->req_buff);
err_req_buff:
	free_page((unsigned long)comm_data->rsp_buff);
err_res_buff:
	free_page((unsigned long)comm_data->io_params);
err_out:
	return ret;
}

static inline void free_dev_communication_buffers(struct cca_host_comm_data *comm_data)
{
	destroy_workqueue(comm_data->work_queue);

	free_page((unsigned long)comm_data->req_buff);
	free_page((unsigned long)comm_data->rsp_buff);
	free_page((unsigned long)comm_data->io_params);
}

static void cca_root_port_pdev_release(struct kref *kref)
{
	struct cca_host_rp_dsc *rp_dsc = container_of(kref, struct cca_host_rp_dsc,
						      tsm_ref);
	struct pci_dev *rp = rp_dsc->pci.pdev;

	cca_pdev_stop_and_destroy(rp);
	free_dev_communication_buffers(&rp_dsc->pdev.comm_data);
	rp->tsm = NULL;
	kfree(rp_dsc);
}

static inline void cca_root_port_pdev_put(struct cca_host_rp_dsc *rp_dsc)
{
	kref_put(&rp_dsc->tsm_ref, cca_root_port_pdev_release);
}

static int cca_root_port_pdev_create(struct pci_dev *rp, struct tsm_dev *tsm_dev)
{
	int ret;
	struct cca_host_rp_dsc *rp_dsc;

	/* We are under pci_tsm_rwsem. */
	lockdep_assert_held_write(&pci_tsm_rwsem);

	rp_dsc = kzalloc_obj(*rp_dsc);
	if (!rp_dsc)
		return -ENOMEM;

	/* we expect this to be asigned early */
	rp->tsm = &rp_dsc->pci;
	rp->tsm->dsm_dev = rp;
	rp->tsm->pdev = rp;
	rp->tsm->tsm_dev = tsm_dev;
	kref_init(&rp_dsc->tsm_ref);
	mutex_init(&rp_dsc->pdev.object_lock);

	ret = init_dev_communication_buffers(rp, &rp_dsc->pdev.comm_data);
	if (ret)
		goto err_comm_buff;

	ret = cca_pdev_create(rp);
	if (ret)
		goto err_pdev_create;

	/*
	 * device communication is still required even though
	 * there is not identity collection
	 */
	ret = cca_pdev_collect_identity(rp);
	if (ret)
		goto pdev_destroy;

	return 0;

pdev_destroy:
	cca_pdev_stop_and_destroy(rp);
err_pdev_create:
	free_dev_communication_buffers(&rp_dsc->pdev.comm_data);
err_comm_buff:
	kfree(rp_dsc);
	rp->tsm = NULL;
	return ret;
}

static int pci_dev_addr_range(struct pci_dev *pdev, struct rmi_addr_range *pdev_addr)
{
	int naddr = 0;
	struct pci_dev *br;
	struct resource *mem, *pref;

	br = pci_upstream_bridge(pdev);
	if (!br)
		return 0;

	mem = pci_resource_n(br, PCI_BRIDGE_MEM_WINDOW);
	pref = pci_resource_n(br, PCI_BRIDGE_PREF_MEM_WINDOW);
	if (resource_assigned(mem))
		naddr = insert_addr_range_sorted(pdev_addr, naddr,
						 mem->start, mem->end + 1);
	if (resource_assigned(pref))
		naddr = insert_addr_range_sorted(pdev_addr, naddr,
						 pref->start, pref->end + 1);

	return naddr;
}

static int cca_pdev_create_ncoh_stream(struct pci_dev *pdev, unsigned long stream_id)
{
	int ret;
	long stream_handle;
	struct cca_host_rp_dsc *rp_dsc;
	struct rmi_pdev_stream_params *params;
	struct pci_dev *rp = pcie_find_root_port(pdev);
	struct cca_host_pf0_ep_dsc *pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev);

	if (!rp->tsm) {
		ret = cca_root_port_pdev_create(rp, pf0_ep_dsc->pci.base_tsm.tsm_dev);
		if (ret)
			return ret;
		rp_dsc = to_cca_rp_dsc(rp);
	} else {
		rp_dsc = to_cca_rp_dsc(rp);
		/* Make sure they use the same TSM */
		if (rp->tsm->tsm_dev != pf0_ep_dsc->pci.base_tsm.tsm_dev)
			return -EINVAL;

		kref_get(&rp_dsc->tsm_ref);
	}

	params = (struct rmi_pdev_stream_params *)get_zeroed_page(GFP_KERNEL);
	if (!params) {
		cca_root_port_pdev_put(rp_dsc);
		return -ENOMEM;
	}

	params->flags = 0;
	params->stream_type = RMI_PDEV_STREAM_NCOH;
	params->pdev_1 = virt_to_phys(pf0_ep_dsc->pdev.rmm_pdev);
	params->pdev_2 = virt_to_phys(rp_dsc->pdev.rmm_pdev);
	params->ide_sid = stream_id;
	params->num_addr_range = pci_dev_addr_range(pdev, params->addr_range);

	ret = cca_pdev_stream_connect(pdev, rp, params, &stream_handle);
	if (ret)
		cca_root_port_pdev_put(rp_dsc);
	else
		pf0_ep_dsc->stream_handle = stream_handle;

	free_page((unsigned long)params);
	return ret;
}

static int cca_pdev_create_ncoh_sys_stream(struct pci_dev *pdev)
{
	int ret;
	long stream_handle;
	struct rmi_pdev_stream_params *params;
	struct cca_host_pf0_ep_dsc *pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev);

	params = (struct rmi_pdev_stream_params *)get_zeroed_page(GFP_KERNEL);
	if (!params)
		return -ENOMEM;

	params->flags = 0;
	params->stream_type = RMI_PDEV_STREAM_NCOH_SYS;
	params->pdev_1 = virt_to_phys(pf0_ep_dsc->pdev.rmm_pdev);
	params->pdev_2 = 0; /* ignored */
	params->ide_sid = 0; /* ignored */
	params->num_addr_range = pci_dev_addr_range(pdev, params->addr_range);

	ret = cca_pdev_stream_connect(pdev, NULL, params, &stream_handle);
	if (!ret)
		pf0_ep_dsc->stream_handle = stream_handle;

	free_page((unsigned long)params);
	return ret;
}

static int cca_pdev_create_streams(struct pci_dev *pdev, unsigned long stream_id)
{
	switch (pci_pcie_type(pdev)) {
	case PCI_EXP_TYPE_ENDPOINT:
		return cca_pdev_create_ncoh_stream(pdev, stream_id);
	case PCI_EXP_TYPE_RC_END:
		return cca_pdev_create_ncoh_sys_stream(pdev);
	default:
		return -EINVAL;
	}
}

static inline bool cca_pdev_need_sel_ide_streams(struct pci_dev *pdev)
{
	return pci_pcie_type(pdev) == PCI_EXP_TYPE_ENDPOINT;
}

static int cca_tsm_connect(struct pci_dev *pdev)
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

		ret = tsm_ide_stream_register(ide);
		if (ret)
			goto err_tsm;
	}

	ret = init_dev_communication_buffers(pdev, &pf0_ep_dsc->pdev.comm_data);
	if (ret)
		goto err_comm_buff;
	ret = cca_pdev_create(pdev);
	if (ret)
		goto err_pdev_create;

	ret = cca_pdev_collect_identity(pdev);
	if (ret)
		goto pdev_destroy;

	if (cca_pdev_needs_key(pdev)) {
		ret = cca_pdev_set_public_key(pdev);
		if (ret)
			goto pdev_destroy;
	}
	/* Create IDE streams */
	ret = cca_pdev_create_streams(pdev, stream_id);
	if (ret)
		goto pdev_destroy;
	/*
	 * Once ide is setup, enable the stream at the endpoint
	 * Root port will be done by RMM
	 */
	if (cca_pdev_need_sel_ide_streams(pdev))
		pci_ide_stream_enable(pdev, ide);

	return 0;

pdev_destroy:
	cca_pdev_stop_and_destroy(pdev);
err_pdev_create:
	free_dev_communication_buffers(&pf0_ep_dsc->pdev.comm_data);
err_comm_buff:
	if (cca_pdev_need_sel_ide_streams(pdev))
		tsm_ide_stream_unregister(ide);
err_tsm:
	if (cca_pdev_need_sel_ide_streams(pdev)) {
		pci_ide_stream_teardown(rp, ide);
		pci_ide_stream_teardown(pdev, ide);
		pci_ide_stream_unregister(ide);
	}
err_stream:
	if (cca_pdev_need_sel_ide_streams(pdev))
		pci_ide_stream_free(ide);
	pf0_ep_dsc->sel_stream = NULL;
err_stream_alloc:

	return ret;
}

static void cca_tsm_disconnect(struct pci_dev *pdev)
{
	struct pci_ide *ide;
	struct cca_host_pf0_ep_dsc *pf0_ep_dsc;
	struct pci_dev *rp = pcie_find_root_port(pdev);

	pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev);
	if (!pf0_ep_dsc)
		return;

	if (cca_pdev_need_sel_ide_streams(pdev))
		ide = pf0_ep_dsc->sel_stream;

	cca_pdev_disconnect_stream(pdev, rp, pf0_ep_dsc->stream_handle);
	if (rp)
		cca_root_port_pdev_put(to_cca_rp_dsc(rp));

	cca_pdev_stop_and_destroy(pdev);
	free_dev_communication_buffers(&pf0_ep_dsc->pdev.comm_data);

	if (cca_pdev_need_sel_ide_streams(pdev)) {
		pci_ide_stream_release(ide);
		pf0_ep_dsc->sel_stream = NULL;
	}
}

static struct pci_tsm_ops cca_link_pci_ops = {
	.probe = cca_tsm_pci_probe,
	.remove = cca_tsm_pci_remove,
	.connect = cca_tsm_connect,
	.disconnect = cca_tsm_disconnect,
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
