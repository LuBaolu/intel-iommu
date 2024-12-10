// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 ARM Ltd.
 */

#include <linux/arm-rmi-cmds.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/pci-doe.h>
#include <linux/delay.h>

#include "rmi-da.h"

static int pci_ide_segment(struct pci_dev *pdev)
{
	if (pdev->fm_enabled)
		return pci_domain_nr(pdev->bus);
	return 0;
}

static unsigned int pci_get_max_rid(struct pci_dev *pdev)
{
	int fn;
	int max_rid;
	int slot = PCI_SLOT(pdev->devfn);

	for (fn = 0; fn < 8; fn++) {
		struct pci_dev *fn_dev;

		fn_dev = pci_get_slot(pdev->bus, PCI_DEVFN(slot, fn));
		if (!fn_dev)
			continue;

		max_rid = pci_dev_id(fn_dev);
		pci_dev_put(fn_dev);
	}
	return max_rid;
}

static int init_pdev_params(struct pci_dev *pdev, struct rmi_pdev_params *params)
{
	int rid;
	unsigned long category;
	struct pci_config_window *cfg = pdev->bus->sysdata;

	/* check we are ECAM compliant */
	if (!pdev->bus->ops->map_bus)
		return -EINVAL;

	switch (pci_pcie_type(pdev)) {
	case PCI_EXP_TYPE_ENDPOINT: {
		struct cca_host_pf0_ep_dsc *pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev);

		/* Endpoint needs DOE mailbox */
		if (!pf0_ep_dsc->pci.doe_mb)
			return -EINVAL;

		params->flags = RMI_PDEV_FLAGS_SPDM;
		category = RMI_PDEV_FLAGS_CATEGORY_OFF_CHIP_EP;
		break;
	}
	default:
		return -EINVAL;
	}

	params->flags |= (category << RMI_PDEV_FLAGS_CATEGORY_SHIFT);
	/* assign the ep device with RMM */
	rid = pci_dev_id(pdev);
	params->pdev_id = rid;
	params->hb_base = cfg->res.start;
	params->routing_id = pci_ide_segment(pdev);
	/* slot number for certificate chain default to zero */
	params->id_index = 0;
	params->hash_algo = RMI_HASH_SHA_256;
	/* no multi function device here. */
	params->rid_base = rid;
	params->rid_top = pci_get_max_rid(pdev) + 1;
	return 0;
}

static inline int rmi_pdev_create(unsigned long pdev_phys,
		unsigned long pdev_params_phys, unsigned long *rmi_ret)
{
	struct rmi_sro_state *sro __free(kfree) = kmalloc_obj(*sro);
	if (!sro)
		return -ENOMEM;

	*rmi_ret = rmi_sro_memxfer_cmd(sro, GFP_KERNEL, SMC_RMI_PDEV_CREATE,
				       pdev_phys, pdev_params_phys);

	return 0;
}

int cca_pdev_create(struct pci_dev *pci_dev)
{
	int ret;
	void *rmm_pdev;
	bool should_free = true;
	phys_addr_t rmm_pdev_phys;
	struct rmi_pdev_params *params;
	struct cca_host_pdev_dsc *pdev_dsc = to_cca_pdev_dsc(pci_dev);

	rmm_pdev = (void *)get_zeroed_page(GFP_KERNEL);
	if (!rmm_pdev)
		return -ENOMEM;

	rmm_pdev_phys = virt_to_phys(rmm_pdev);
	if (rmi_delegate_page(rmm_pdev_phys)) {
		ret = -EIO;
		goto err_granule_delegate;
	}

	params = (struct rmi_pdev_params *)get_zeroed_page(GFP_KERNEL);
	if (!params) {
		ret = -ENOMEM;
		goto err_param_alloc;
	}

	ret = init_pdev_params(pci_dev, params);
	if (ret)
		goto err_init_pdev_params;

	{
		unsigned long rmi_ret;

		ret = rmi_pdev_create(rmm_pdev_phys, virt_to_phys(params),
				      &rmi_ret);
		if (ret || rmi_ret) {
			if (!ret)
				ret = -EIO;
			goto err_init_pdev_params;
		}
	}

	pdev_dsc->rmm_pdev = rmm_pdev;
	free_page((unsigned long)params);
	return 0;

err_init_pdev_params:
	free_page((unsigned long)params);
err_param_alloc:
	if (rmi_undelegate_page(rmm_pdev_phys))
		should_free = false;
err_granule_delegate:
	if (should_free)
		free_page((unsigned long)rmm_pdev);
	return ret;
}

static int doe_send_req_resp(struct pci_tsm *tsm)
{
	int data_obj_type;
	struct cca_host_comm_data *comm_data = to_cca_comm_data(tsm->pdev);
	struct rmi_dev_comm_exit *io_exit = &comm_data->io_params->exit;
	u8 protocol = io_exit->protocol;

	if (protocol == RMI_PROTOCOL_SPDM)
		data_obj_type = PCI_DOE_FEATURE_CMA;
	else if (protocol == RMI_PROTOCOL_SECURE_SPDM)
		data_obj_type = PCI_DOE_FEATURE_SSESSION;
	else
		return -EINVAL;

	/* delay the send */
	if (io_exit->req_delay)
		fsleep(io_exit->req_delay);

	return pci_tsm_doe_transfer(tsm->dsm_dev, data_obj_type,
				    comm_data->req_buff, io_exit->req_len,
				    comm_data->rsp_buff, PAGE_SIZE);
}

static inline bool pending_dev_communicate(struct rmi_dev_comm_exit *io_exit)
{
	bool pending = io_exit->flags & (RMI_DEV_COMM_EXIT_CACHE_REQ |
					 RMI_DEV_COMM_EXIT_CACHE_RSP |
					 RMI_DEV_COMM_EXIT_SEND |
					 RMI_DEV_COMM_EXIT_WAIT |
					 RMI_DEV_COMM_EXIT_MULTI);
	return pending;
}

static inline gfp_t cache_obj_id_to_gfp_flags(u8 cache_obj_id)
{
	/* These two cache objects are system objects. */
	if (cache_obj_id == RMI_DEV_VCA || cache_obj_id == RMI_DEV_CERTIFICATE)
		return GFP_KERNEL;
	/* rest are per TDI which is associated to a VM */
	return GFP_KERNEL_ACCOUNT;
}

static int _do_dev_communicate(enum dev_comm_type type, struct pci_tsm *tsm)
{
	unsigned long rmi_ret;
	gfp_t cache_alloc_flags;
	int nbytes, cp_len;
	struct cache_object **cache_objp, *cache_obj;
	struct cca_host_pdev_dsc *pdev_dsc = to_cca_pdev_dsc(tsm->dsm_dev);
	struct cca_host_comm_data *comm_data = to_cca_comm_data(tsm->pdev);
	struct rmi_dev_comm_enter *io_enter = &comm_data->io_params->enter;
	struct rmi_dev_comm_exit *io_exit = &comm_data->io_params->exit;

redo_communicate:

	if (type == PDEV_COMMUNICATE)
		rmi_ret = rmi_pdev_communicate(virt_to_phys(pdev_dsc->rmm_pdev),
					       virt_to_phys(comm_data->io_params));
	else
		rmi_ret = RMI_ERROR_INPUT;
	if (rmi_ret != RMI_SUCCESS) {
		if (rmi_ret == RMI_BUSY)
			return -EBUSY;
		return -EIO;
	}

	if (io_exit->flags & RMI_DEV_COMM_EXIT_CACHE_REQ ||
	    io_exit->flags & RMI_DEV_COMM_EXIT_CACHE_RSP) {
		struct cca_host_pf0_ep_dsc *pf0_ep_dsc = to_cca_pf0_ep_dsc(tsm->dsm_dev);

		if (!pf0_ep_dsc) {
			WARN(1,
			     "Device communication got cache request on wrong device\n");
			return -EINVAL;
		}

		switch (io_exit->cache_object_id) {
		case RMI_DEV_VCA:
			cache_objp = &pf0_ep_dsc->vca;
			break;
		case RMI_DEV_CERTIFICATE:
			cache_objp = &pf0_ep_dsc->cert_chain.cache;
			break;
		default:
			return -EINVAL;
		}
		cache_obj = *cache_objp;
		cache_alloc_flags = cache_obj_id_to_gfp_flags(io_exit->cache_object_id);
		int cache_remaining;

		if (io_exit->flags & RMI_DEV_COMM_EXIT_CACHE_REQ)
			cp_len = io_exit->req_cache_len;
		else
			cp_len = io_exit->rsp_cache_len;

		/* response and request len should be <= SZ_4k */
		if (cp_len > CACHE_CHUNK_SIZE)
			return -EINVAL;

		/* new allocation */
		if (!cache_obj) {
			int obj_size = struct_size(cache_obj, buf,
						   CACHE_CHUNK_SIZE);

			cache_obj = kvmalloc(obj_size, cache_alloc_flags);
			if (!cache_obj)
				return -ENOMEM;

			cache_obj->size = CACHE_CHUNK_SIZE;
			cache_obj->offset = 0;
			*cache_objp = cache_obj;
		}

		cache_remaining = cache_obj->size - cache_obj->offset;
		if (cp_len > cache_remaining) {
			struct cache_object *new_obj;
			int new_size = struct_size(cache_obj, buf,
						   cache_obj->size +
						   CACHE_CHUNK_SIZE);

			if (cache_obj->size + CACHE_CHUNK_SIZE > MAX_CACHE_OBJ_SIZE)
				return -EINVAL;

			new_obj = kvrealloc(cache_obj, new_size, cache_alloc_flags);
			if (!new_obj)
				return -ENOMEM;
			new_obj->size = cache_obj->size + CACHE_CHUNK_SIZE;
			*cache_objp = new_obj;
		}

		/* cache object can change above. */
		cache_obj = *cache_objp;
	}


	if (io_exit->flags & RMI_DEV_COMM_EXIT_CACHE_REQ) {
		memcpy(cache_obj->buf + cache_obj->offset,
		       (comm_data->req_buff + io_exit->req_cache_offset), io_exit->req_cache_len);
		cache_obj->offset += io_exit->req_cache_len;
	}

	if (io_exit->flags & RMI_DEV_COMM_EXIT_CACHE_RSP) {
		memcpy(cache_obj->buf + cache_obj->offset,
		       (comm_data->rsp_buff + io_exit->rsp_cache_offset), io_exit->rsp_cache_len);
		cache_obj->offset += io_exit->rsp_cache_len;
	}

	/*
	 * wait for last packet request from RMM.
	 * We should not find this because our device communication is synchronous
	 */
	if (io_exit->flags & RMI_DEV_COMM_EXIT_WAIT)
		return -EIO;

	/* next packet to send */
	if (io_exit->flags & RMI_DEV_COMM_EXIT_SEND) {
		nbytes = doe_send_req_resp(tsm);
		if (nbytes < 0) {
			/* report error back to RMM */
			io_enter->status = RMI_DEV_COMM_ERROR;
		} else {
			/* send response back to RMM */
			io_enter->resp_len = nbytes;
			io_enter->status = RMI_DEV_COMM_RESPONSE;
		}
	} else {
		/* no data transmitted => no data received */
		io_enter->resp_len = 0;
		io_enter->status = RMI_DEV_COMM_NONE;
	}

	if (pending_dev_communicate(io_exit))
		goto redo_communicate;

	return 0;
}

static int do_dev_communicate(enum dev_comm_type type,
		struct pci_tsm *tsm, unsigned long error_state)
{
	int ret, state = error_state;
	struct rmi_dev_comm_enter *io_enter;
	struct cca_host_pdev_dsc *pdev_dsc = to_cca_pdev_dsc(tsm->dsm_dev);

	io_enter = &pdev_dsc->comm_data.io_params->enter;
	io_enter->resp_len = 0;
	io_enter->status = RMI_DEV_COMM_NONE;

	ret = _do_dev_communicate(type, tsm);
	if (ret) {
		if (type == PDEV_COMMUNICATE)
			rmi_pdev_abort(virt_to_phys(pdev_dsc->rmm_pdev));
	} else {
		/*
		 * Some device communication error will transition the
		 * device to error state. Report that.
		 */
		if (type == PDEV_COMMUNICATE) {
			if (rmi_pdev_get_state(virt_to_phys(pdev_dsc->rmm_pdev),
					       (enum rmi_pdev_state *)&state))
				state = error_state;
		}
	}

	if (state == error_state)
		pci_err(tsm->pdev, "device communication error\n");

	return state;
}

static int wait_for_dev_state(enum dev_comm_type type, struct pci_tsm *tsm,
		unsigned long target_state, unsigned long error_state)
{
	int state;

	do {
		state = do_dev_communicate(type, tsm, error_state);

		if (state == target_state || state == error_state)
			return state;
	} while (1);

	/* can't reach */
	return error_state;
}

static int wait_for_pdev_state(struct pci_tsm *tsm, enum rmi_pdev_state target_state)
{
	return wait_for_dev_state(PDEV_COMMUNICATE, tsm, target_state, RMI_PDEV_ERROR);
}

static void pdev_state_transition_workfn(struct work_struct *work)
{
	unsigned long state;
	struct pci_tsm *tsm;
	struct dev_comm_work *setup_work;
	struct cca_host_pdev_dsc *pdev_dsc;

	setup_work = container_of(work, struct dev_comm_work, work);
	tsm = setup_work->tsm;
	pdev_dsc = to_cca_pdev_dsc(tsm->dsm_dev);

	guard(mutex)(&pdev_dsc->object_lock);
	state = wait_for_pdev_state(tsm, setup_work->target_state);
	WARN_ON(state != setup_work->target_state);
}

static int __maybe_unused submit_pdev_state_transition_work(struct pci_dev *pdev,
		enum rmi_pdev_state target_state)
{
	enum rmi_pdev_state state;
	struct dev_comm_work comm_work;
	struct cca_host_pdev_dsc *pdev_dsc = to_cca_pdev_dsc(pdev);
	struct cca_host_comm_data *comm_data = to_cca_comm_data(pdev);

	INIT_WORK_ONSTACK(&comm_work.work, pdev_state_transition_workfn);
	comm_work.tsm = pdev->tsm;
	comm_work.target_state = target_state;

	queue_work(comm_data->work_queue, &comm_work.work);

	flush_work(&comm_work.work);
	destroy_work_on_stack(&comm_work.work);

	/* check if we reached target state */
	if (rmi_pdev_get_state(virt_to_phys(pdev_dsc->rmm_pdev), &state))
		return -EIO;

	if (state != target_state)
		/* no specific error for this */
		return -1;
	return 0;
}
