// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 ARM Ltd.
 */

#include <linux/arm-rmi-cmds.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/pci-doe.h>
#include <linux/delay.h>
#include <crypto/internal/rsa.h>
#include <keys/asymmetric-type.h>
#include <keys/x509-parser.h>
#include <linux/kvm_types.h>
#include <linux/kvm_host.h>
#include <asm/kvm_rmi.h>

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
	case PCI_EXP_TYPE_ROOT_PORT: {
		category = RMI_PDEV_FLAGS_CATEGORY_ROOT_PORT;
		break;
	}
	case PCI_EXP_TYPE_RC_END: {
		struct cca_host_pf0_ep_dsc *pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev);

		/* Use SPDM if present */
		if (pf0_ep_dsc->pci.doe_mb)
			params->flags = RMI_PDEV_FLAGS_SPDM;

		category = RMI_PDEV_FLAGS_CATEGORY_ON_CHIP_EP;
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

static int _do_dev_communicate(enum dev_comm_type type, struct pci_tsm *tsm,
			       struct cca_host_tdi *host_tdi,
			       int *stream_wait)
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
		rmi_ret = rmi_vdev_communicate(virt_to_phys(host_tdi->realm->rd),
					       virt_to_phys(pdev_dsc->rmm_pdev),
					       virt_to_phys(host_tdi->rmm_vdev),
					       virt_to_phys(comm_data->io_params));
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
		case RMI_DEV_INTERFACE_REPORT:
			cache_objp = &host_tdi->interface_report;
			break;
		case RMI_DEV_MEASUREMENTS:
			cache_objp = &host_tdi->measurements;
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

	if (io_exit->flags & RMI_DEV_COMM_EXIT_STREAM_WAIT) {
		if (stream_wait)
			*stream_wait = 1;
		else
			WARN(1, "Unexpected Stream wait status\n");
	}
	return 0;
}

static int do_dev_communicate(enum dev_comm_type type,
		struct pci_tsm *tsm, struct cca_host_tdi *host_tdi,
		unsigned long error_state, int *stream_wait)
{
	int ret, state;
	unsigned long rmi_ret;
	struct rmi_dev_comm_enter *io_enter;
	struct cca_host_pdev_dsc *pdev_dsc = to_cca_pdev_dsc(tsm->dsm_dev);

	io_enter = &pdev_dsc->comm_data.io_params->enter;
	io_enter->resp_len = 0;
	io_enter->status = RMI_DEV_COMM_NONE;
	if (stream_wait)
		*stream_wait = 0;

	ret = _do_dev_communicate(type, tsm, host_tdi, stream_wait);
	if (ret) {
		if (type == PDEV_COMMUNICATE)
			rmi_pdev_abort(virt_to_phys(pdev_dsc->rmm_pdev));
		else
			rmi_vdev_abort(virt_to_phys(host_tdi->rmm_vdev));

		state = error_state;
	} else {
		/*
		 * Some device communication error will transition the
		 * device to error state. Report that.
		 */
		if (type == PDEV_COMMUNICATE)
			rmi_ret = rmi_pdev_get_state(virt_to_phys(pdev_dsc->rmm_pdev),
						     (enum rmi_pdev_state *)&state);
		else
			rmi_ret = rmi_vdev_get_state(virt_to_phys(host_tdi->rmm_vdev),
						     (enum rmi_vdev_state *)&state);
		if (rmi_ret)
			state = error_state;
	}

	if (state == error_state)
		pci_err(tsm->pdev, "device communication error\n");

	return state;
}

static int wait_for_dev_state(enum dev_comm_type type, struct pci_tsm *tsm,
		struct cca_host_tdi *host_tdi, unsigned long target_state,
		unsigned long error_state)
{
	int state;

	do {
		state = do_dev_communicate(type, tsm, host_tdi, error_state, NULL);

		if (state == target_state || state == error_state)
			return state;
	} while (1);

	/* can't reach */
	return error_state;
}

static int wait_for_pdev_state(struct pci_tsm *tsm, enum rmi_pdev_state target_state)
{
	return wait_for_dev_state(PDEV_COMMUNICATE, tsm, NULL, target_state,
				  RMI_PDEV_ERROR);
}

static int wait_for_vdev_state(struct pci_tsm *tsm,
			       struct cca_host_tdi *host_tdi,
			       enum rmi_vdev_state target_state)
{
	return wait_for_dev_state(VDEV_COMMUNICATE, tsm, host_tdi,
				  target_state, RMI_VDEV_ERROR);
}

static int parse_certificate_chain(struct pci_tsm *tsm)
{
	struct cca_host_pf0_ep_dsc *pf0_ep_dsc;
	unsigned int chain_size;
	unsigned int offset = 0;
	u8 *chain_data;

	pf0_ep_dsc = to_cca_pf0_ep_dsc(tsm->pdev);

	/* If device communication didn't results in certificate caching. */
	if (!pf0_ep_dsc->cert_chain.cache || !pf0_ep_dsc->cert_chain.cache->offset)
		return -EINVAL;

	chain_size = pf0_ep_dsc->cert_chain.cache->offset;
	chain_data = pf0_ep_dsc->cert_chain.cache->buf;

	while (offset < chain_size) {
		ssize_t cert_len =
			x509_get_certificate_length(chain_data + offset,
						    chain_size - offset);
		if (cert_len < 0)
			return cert_len;

		struct x509_certificate *cert __free(x509_free_certificate) =
			x509_cert_parse(chain_data + offset, cert_len);

		if (IS_ERR(cert)) {
			pci_warn(tsm->pdev, "parsing of certificate chain not successful\n");
			return PTR_ERR(cert);
		}

		/* The key in the last cert in the chain is used */
		if (offset + cert_len == chain_size) {
			char *signature_algorithm __free(kfree) = NULL;
			void *public_key __free(kfree) =
				kzalloc(cert->pub->keylen, GFP_KERNEL);

			if (!public_key)
				return -ENOMEM;

			if (!strcmp("rsa", cert->pub->pkey_algo)) {
				struct rsa_key rsa_key = {0};
				size_t skip = 0;
				int ret;

				ret = rsa_parse_pub_key(&rsa_key, cert->pub->key,
							cert->pub->keylen);
				if (ret)
					return ret;

				while (skip < rsa_key.n_sz && !rsa_key.n[skip])
					skip++;

				/* check we have 3072 bits len */
				if ((rsa_key.n_sz - skip) != (3072 >> 3))
					return -EINVAL;
			} else if (strcmp("ecdsa-nist-p256", cert->pub->pkey_algo) &&
				   strcmp("ecdsa-nist-p384", cert->pub->pkey_algo)) {
				return -EINVAL;
			}

			signature_algorithm = kstrdup(cert->pub->pkey_algo, GFP_KERNEL);
			if (!signature_algorithm)
				return -ENOMEM;

			memcpy(public_key, cert->pub->key, cert->pub->keylen);
			pf0_ep_dsc->rmi_signature_algorithm =
				no_free_ptr(signature_algorithm);
			pf0_ep_dsc->cert_chain.public_key = no_free_ptr(public_key);
			pf0_ep_dsc->cert_chain.public_key_size = cert->pub->keylen;
			pf0_ep_dsc->cert_chain.valid = true;
			return 0;
		}

		offset += cert_len;
	}

	/* something wrong with chain size and parsing. */
	return -EINVAL;
}

static inline void key_param_free(struct rmi_public_key_params *param)
{
	return free_page((unsigned long)param);
}

static inline int copy_key_part(u8 *buf, const u8 *key_buf, size_t sz)
{
	int skip;

	/* skip leading zero in asn.1 */
	for (skip = 0; skip < sz; skip++)
		if (key_buf[skip])
			break;

	memcpy(buf, key_buf + skip, sz - skip);
	return sz - skip;
}

DEFINE_FREE(key_param_free, struct rmi_public_key_params *, if (_T) key_param_free(_T))
static int pdev_set_public_key(struct pci_tsm *tsm)
{
	struct cca_host_pf0_ep_dsc *pf0_ep_dsc;

	pf0_ep_dsc = to_cca_pf0_ep_dsc(tsm->pdev);
	/* Check that all the necessary information was captured from communication */
	if (!pf0_ep_dsc->cert_chain.valid)
		return -EINVAL;

	u8 *key __free(free_page) = (u8 *)get_zeroed_page(GFP_KERNEL);
	u8 *metadata __free(free_page) = (u8 *)get_zeroed_page(GFP_KERNEL);
	struct rmi_public_key_params *key_params __free(key_param_free) =
		(struct rmi_public_key_params *)get_zeroed_page(GFP_KERNEL);
	if (!key_params || !key || !metadata)
		return -ENOMEM;

	if (!strcmp("ecdsa-nist-p384", pf0_ep_dsc->rmi_signature_algorithm) ||
	    !strcmp("ecdsa-nist-p256", pf0_ep_dsc->rmi_signature_algorithm)) {
		key_params->key_len = pf0_ep_dsc->cert_chain.public_key_size;
		memcpy(key, pf0_ep_dsc->cert_chain.public_key,
		       pf0_ep_dsc->cert_chain.public_key_size);
		key_params->metadata_len = 0;
	} else if (!strcmp("rsa", pf0_ep_dsc->rmi_signature_algorithm)) {
		int ret;
		struct rsa_key rsa_key = {0};

		ret = rsa_parse_pub_key(&rsa_key,
					pf0_ep_dsc->cert_chain.public_key,
					pf0_ep_dsc->cert_chain.public_key_size);
		if (ret)
			return ret;

		key_params->key_len = copy_key_part(key, rsa_key.n, rsa_key.n_sz);
		key_params->metadata_len = copy_key_part(metadata,
							 rsa_key.e, rsa_key.e_sz);
	} else {
		return -EINVAL;
	}

	key_params->key_addr = virt_to_phys(key);
	key_params->metadata_addr = virt_to_phys(metadata);

	if (rmi_pdev_set_pubkey(virt_to_phys(pf0_ep_dsc->pdev.rmm_pdev),
				virt_to_phys(key_params)))
		return -ENXIO;
	return 0;
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

static int submit_pdev_state_transition_work(struct pci_dev *pdev,
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

static void vdev_state_transition_workfn(struct work_struct *work)
{
	unsigned long state;
	struct pci_tsm *tsm;
	struct dev_comm_work *setup_work;
	struct cca_host_pdev_dsc *pdev_dsc;

	setup_work = container_of(work, struct dev_comm_work, work);
	tsm = setup_work->tsm;

	pdev_dsc = to_cca_pdev_dsc(tsm->dsm_dev);
	guard(mutex)(&pdev_dsc->object_lock);

	state = wait_for_vdev_state(tsm, setup_work->host_tdi,
				    setup_work->target_state);
	WARN_ON(state != setup_work->target_state);
}

static int submit_vdev_state_transition_work(struct cca_host_tdi *host_tdi,
					     struct pci_dev *pdev,
					     int target_state)
{
	enum rmi_vdev_state state;
	struct dev_comm_work comm_work;
	struct cca_host_comm_data *comm_data = to_cca_comm_data(pdev);

	INIT_WORK_ONSTACK(&comm_work.work, vdev_state_transition_workfn);
	comm_work.tsm = pdev->tsm;
	comm_work.host_tdi = host_tdi;
	comm_work.target_state = target_state;

	queue_work(comm_data->work_queue, &comm_work.work);

	flush_work(&comm_work.work);
	destroy_work_on_stack(&comm_work.work);

	/* check if we reached target state */
	if (rmi_vdev_get_state(virt_to_phys(host_tdi->rmm_vdev), &state))
		return -ENXIO;

	if (state != target_state)
		/* Protocol didn't take it to expected target state */
		return -EPROTO;
	return 0;
}

static void pdev_collect_identity_workfn(struct work_struct *work)
{
	struct pci_tsm *tsm;
	struct dev_comm_work *setup_work;
	struct cca_host_pdev_dsc *pdev_dsc;

	setup_work = container_of(work, struct dev_comm_work, work);
	tsm = setup_work->tsm;
	pdev_dsc = to_cca_pdev_dsc(tsm->dsm_dev);

	guard(mutex)(&pdev_dsc->object_lock);

	do_dev_communicate(PDEV_COMMUNICATE, tsm, NULL, RMI_PDEV_ERROR, NULL);

	/*
	 * Don't worry about communication error. The caller will look at
	 * device state to find more about error
	 */
}

int cca_pdev_collect_identity(struct pci_dev *pdev)
{
	enum rmi_pdev_state state;
	struct dev_comm_work comm_work;
	struct cca_host_pdev_dsc *pdev_dsc = to_cca_pdev_dsc(pdev);
	struct cca_host_comm_data *comm_data = to_cca_comm_data(pdev);

	/*
	 * Device identity is collected by doing a device communication
	 * after a pdev_create
	 */
	INIT_WORK_ONSTACK(&comm_work.work, pdev_collect_identity_workfn);
	comm_work.tsm = pdev->tsm;

	queue_work(comm_data->work_queue, &comm_work.work);

	flush_work(&comm_work.work);
	destroy_work_on_stack(&comm_work.work);

	/* check for device communication error*/
	if (rmi_pdev_get_state(virt_to_phys(pdev_dsc->rmm_pdev), &state))
		return -EIO;

	if (state == RMI_PDEV_ERROR)
		return -EPROTO;

	return 0;
}

bool cca_pdev_needs_key(struct pci_dev *pdev)
{
	enum rmi_pdev_state state;
	struct cca_host_pdev_dsc *pdev_dsc = to_cca_pdev_dsc(pdev);

	/*
	 * Consider pdev_get_state failure as need key transition
	 * and that will result in device communication failure, which
	 * will handle this error.
	 */
	if (rmi_pdev_get_state(virt_to_phys(pdev_dsc->rmm_pdev), &state))
		return true;

	if (state == RMI_PDEV_NEEDS_KEY)
		return true;
	return false;
}

int cca_pdev_set_public_key(struct pci_dev *pdev)
{
	int ret;

	/*
	 * we now have certificate chain in dsm->cert_chain. Parse that and set
	 * the pubkey.
	 */
	ret = parse_certificate_chain(pdev->tsm);
	if (ret)
		return ret;

	ret = pdev_set_public_key(pdev->tsm);
	if (ret)
		return ret;

	return submit_pdev_state_transition_work(pdev, RMI_PDEV_READY);
}

static inline int rmi_pdev_destroy(unsigned long pdev_phys,
		unsigned long *rmi_ret)
{
	struct rmi_sro_state *sro __free(kfree) = kmalloc_obj(*sro);
	if (!sro)
		return -ENOMEM;

	*rmi_ret = rmi_sro_memxfer_cmd(sro, GFP_KERNEL,
				       SMC_RMI_PDEV_DESTROY, pdev_phys);

	return 0;
}

void cca_pdev_stop_and_destroy(struct pci_dev *pdev)
{
	int ret;
	unsigned long rmi_ret;
	struct cca_host_pdev_dsc *pdev_dsc = to_cca_pdev_dsc(pdev);
	struct cca_host_pf0_ep_dsc *pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev);
	phys_addr_t rmm_pdev_phys = virt_to_phys(pdev_dsc->rmm_pdev);

	if (WARN_ON(rmi_pdev_stop(rmm_pdev_phys)))
		return;

	ret = submit_pdev_state_transition_work(pdev, RMI_PDEV_STOPPED);
	if (ret)
		return;

	ret = rmi_pdev_destroy(rmm_pdev_phys, &rmi_ret);
	if (WARN_ON(ret || rmi_ret))
		return;

	if (pf0_ep_dsc) {
		kfree(pf0_ep_dsc->rmi_signature_algorithm);
		kfree(pf0_ep_dsc->cert_chain.public_key);
		kvfree(pf0_ep_dsc->cert_chain.cache);
		kvfree(pf0_ep_dsc->vca);
		pf0_ep_dsc->cert_chain.cache = NULL;
		pf0_ep_dsc->vca = NULL;
	}

	if (!rmi_undelegate_page(rmm_pdev_phys))
		free_page((unsigned long)pdev_dsc->rmm_pdev);
	pdev_dsc->rmm_pdev = NULL;
}

static void stream_connect_workfn(struct work_struct *work)
{
	int state;
	int peer_wait = 0;
	struct pci_tsm *tsm;
	int my_index, peer_index, target;
	struct stream_connect_work *stream_work;
	struct cca_host_pdev_dsc *pdev_dsc;

	stream_work = container_of(work, struct stream_connect_work, work);
	tsm = stream_work->tsm;
	pdev_dsc = to_cca_pdev_dsc(tsm->dsm_dev);

	my_index = stream_work->my_index;
	peer_index = my_index ^ 0x1;

redo_communicate:
	mutex_lock(&pdev_dsc->object_lock);

	state = do_dev_communicate(PDEV_COMMUNICATE, tsm, NULL,
				   RMI_PDEV_ERROR, &peer_wait);
	if (state != RMI_PDEV_ERROR && peer_wait) {

		if (!stream_work->has_peer) {
			WARN(1, "Unexpected STREAM_WAIT without peer stream\n");
			mutex_unlock(&pdev_dsc->object_lock);
			return;
		}
		/*
		 * Record a fresh target val for this side, then wait until
		 * peer reaches at least the same target.
		 */
		target = atomic_inc_return(&stream_work->sync->val[my_index]);

		wake_up_all(&stream_work->sync->wq);

		mutex_unlock(&pdev_dsc->object_lock);

		/* Wait for peer to make matching progress */
		wait_event(stream_work->sync->wq,
			   atomic_read(&stream_work->sync->val[peer_index]) >= target);
		goto redo_communicate;
	}

	/* Signal peer if it is waiting on me */
	atomic_inc_return(&stream_work->sync->val[my_index]);
	wake_up_all(&stream_work->sync->wq);

	mutex_unlock(&pdev_dsc->object_lock);
}

static int submit_stream_work(struct pci_dev *pdev1, struct pci_dev *pdev2,
		unsigned long stream_handle)
{
	phys_addr_t rmm_pdev1_phys, rmm_pdev2_phys = 0;
	struct cca_host_comm_data *comm_data_pdev1, *comm_data_pdev2;
	struct cca_host_pdev_dsc *pdev_dsc1, *pdev_dsc2 = NULL;
	struct stream_sync sync;
	struct stream_connect_work stream_work_pdev1, stream_work_pdev2;

	comm_data_pdev1 = to_cca_comm_data(pdev1);
	init_waitqueue_head(&sync.wq);
	atomic_set(&sync.val[0], 0);
	atomic_set(&sync.val[1], 0);

	pdev_dsc1 = to_cca_pdev_dsc(pdev1);
	INIT_WORK_ONSTACK(&stream_work_pdev1.work, stream_connect_workfn);
	stream_work_pdev1.tsm = pdev1->tsm;
	stream_work_pdev1.sync = &sync;
	stream_work_pdev1.my_index = 0;
	stream_work_pdev1.has_peer = !!pdev2;
	queue_work(comm_data_pdev1->work_queue, &stream_work_pdev1.work);

	if (pdev2) {
		comm_data_pdev2 = to_cca_comm_data(pdev2);
		pdev_dsc2 = to_cca_pdev_dsc(pdev2);
		INIT_WORK_ONSTACK(&stream_work_pdev2.work, stream_connect_workfn);
		stream_work_pdev2.tsm = pdev2->tsm;
		stream_work_pdev2.sync = &sync;
		stream_work_pdev2.my_index = 1;
		stream_work_pdev2.has_peer = true;
		queue_work(comm_data_pdev2->work_queue, &stream_work_pdev2.work);
	}

	flush_work(&stream_work_pdev1.work);
	if (pdev2) {
		flush_work(&stream_work_pdev2.work);
		destroy_work_on_stack(&stream_work_pdev2.work);
	}

	destroy_work_on_stack(&stream_work_pdev1.work);

	rmm_pdev1_phys = virt_to_phys(pdev_dsc1->rmm_pdev);
	if (pdev2)
		rmm_pdev2_phys = virt_to_phys(pdev_dsc2->rmm_pdev);
	/*
	 * If we had device communication error, this will error out.
	 */
	if (rmi_pdev_stream_complete(rmm_pdev1_phys, rmm_pdev2_phys, stream_handle))
		return -EIO;

	return 0;
}

int cca_pdev_stream_connect(struct pci_dev *pdev1, struct pci_dev *pdev2,
		struct rmi_pdev_stream_params *stream_params,
		unsigned long *stream_handle)
{
	phys_addr_t stream_params_phys = virt_to_phys(stream_params);

	if (rmi_pdev_stream_connect(stream_params_phys, stream_handle))
		return -EIO;

	return submit_stream_work(pdev1, pdev2, *stream_handle);
}

int cca_pdev_disconnect_stream(struct pci_dev *pdev1,
		struct pci_dev *pdev2, unsigned long stream_handle)
{

	phys_addr_t rmm_pdev2_phys = 0;
	struct cca_host_pdev_dsc *pdev_dsc1 = to_cca_pdev_dsc(pdev1);

	if (pdev2)
		rmm_pdev2_phys = virt_to_phys(to_cca_pdev_dsc(pdev2)->rmm_pdev);

	if (rmi_pdev_stream_disconnect(virt_to_phys(pdev_dsc1->rmm_pdev),
				       rmm_pdev2_phys, stream_handle))
		return -EIO;

	return submit_stream_work(pdev1, pdev2, stream_handle);
}

static unsigned long pci_get_tdi_id(struct pci_dev *pdev)
{
	/* requester segment is marked reserved. */
	return pci_dev_id(pdev);
}

static void cca_vdev_assert_locked(struct pci_dev *pdev)
{
	struct cca_host_pf0_ep_dsc *pf0_ep_dsc;

	pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev->tsm->dsm_dev);
	lockdep_assert_held(&pf0_ep_dsc->pci.lock);
}

static void init_vdev_params_mmio_range(struct pci_dev *pdev,
		struct rmi_vdev_params *params)
{
	int index = 0;

	for (int i = 0; i < PCI_STD_NUM_BARS; i++) {
		struct resource *res = &pdev->resource[i];

		if (!(res->flags & IORESOURCE_MEM))
			continue;

		if (resource_size(res) == 0)
			continue;

		index = insert_addr_range_sorted(params->addr_range, index,
						 res->start, res->end + 1);
	}

	params->num_addr_range = index;
}


void *cca_vdev_create(struct cca_host_tdi *host_tdi, struct realm *realm,
		      struct pci_dev *pdev, struct pci_dev *pf0_dev,
		      u32 guest_rid)
{
	phys_addr_t rd_phys = virt_to_phys(realm->rd);
	struct rmi_vdev_params *params = NULL;
	struct cca_host_pdev_dsc *pdev_dsc;
	phys_addr_t rmm_pdev_phys;
	phys_addr_t rmm_vdev_phys;
	bool should_free = true;
	void *rmm_vdev;
	int ret;

	cca_vdev_assert_locked(pdev);

	pdev_dsc = to_cca_pdev_dsc(pf0_dev);
	if (!pdev_dsc->rmm_pdev) {
		ret = -EINVAL;
		goto err_out;
	}

	rmm_vdev = (void *)get_zeroed_page(GFP_KERNEL);
	if (!rmm_vdev) {
		ret =  -ENOMEM;
		goto err_out;
	}

	rmm_vdev_phys = virt_to_phys(rmm_vdev);
	if (rmi_delegate_page(rmm_vdev_phys)) {
		ret = -ENXIO;
		goto err_granule_delegate;
	}

	params = (struct rmi_vdev_params *)get_zeroed_page(GFP_KERNEL);
	if (!params) {
		ret = -ENOMEM;
		goto err_params_alloc;
	}

	params->flags = 0;
	params->vdev_id = guest_rid;
	params->tdi_id = pci_get_tdi_id(pdev);

	init_vdev_params_mmio_range(pdev, params);

	rmm_pdev_phys = virt_to_phys(pdev_dsc->rmm_pdev);
	if (rmi_vdev_create(rd_phys, rmm_pdev_phys,
			    rmm_vdev_phys, virt_to_phys(params))) {
		ret = -ENXIO;
		goto err_vdev_create;
	}

	/* Set up host_tdi before calling device communicate. */
	host_tdi->rmm_vdev = rmm_vdev;
	host_tdi->realm = realm;
	host_tdi->state = RMI_VDEV_NEW;

	ret = submit_vdev_state_transition_work(host_tdi, pdev,
						RMI_VDEV_UNLOCKED);
	/* failure is treated as rmi_vdev_create failure */
	if (ret)
		goto err_vdev_comm;
	host_tdi->state = RMI_VDEV_UNLOCKED;

	free_page((unsigned long)params);
	return rmm_vdev;

err_vdev_comm:
	rmi_vdev_destroy(rd_phys, rmm_pdev_phys, rmm_vdev_phys);
err_vdev_create:
	free_page((unsigned long)params);
err_params_alloc:
	if (rmi_undelegate_page(rmm_vdev_phys))
		should_free = false;
err_granule_delegate:
	if (should_free)
		free_page((unsigned long)rmm_vdev);
err_out:
	return ERR_PTR(ret);
}

int cca_pdev_refresh_stream_key(struct pci_dev *pdev1,
		struct pci_dev *pdev2, unsigned long stream_handle)
{

	phys_addr_t rmm_pdev2_phys = 0;
	struct cca_host_pdev_dsc *pdev_dsc1 = to_cca_pdev_dsc(pdev1);

	if (pdev2)
		rmm_pdev2_phys = virt_to_phys(to_cca_pdev_dsc(pdev2)->rmm_pdev);

	if (rmi_pdev_stream_key_refresh(virt_to_phys(pdev_dsc1->rmm_pdev),
					rmm_pdev2_phys, stream_handle))
		return -EIO;

	return submit_stream_work(pdev1, pdev2, stream_handle);
}


int cca_pdev_purge_stream_key(struct pci_dev *pdev1,
		struct pci_dev *pdev2, unsigned long stream_handle)
{

	phys_addr_t rmm_pdev2_phys = 0;
	struct cca_host_pdev_dsc *pdev_dsc1 = to_cca_pdev_dsc(pdev1);

	if (pdev2)
		rmm_pdev2_phys = virt_to_phys(to_cca_pdev_dsc(pdev2)->rmm_pdev);

	if (rmi_pdev_stream_key_purge(virt_to_phys(pdev_dsc1->rmm_pdev),
				      rmm_pdev2_phys, stream_handle))
		return -EIO;

	return submit_stream_work(pdev1, pdev2, stream_handle);
}

int cca_vdev_device_unlock(struct cca_host_tdi *host_tdi,
			   struct pci_dev *pdev)
{
	struct pci_dev *pf0_dev = pdev->tsm->dsm_dev;
	phys_addr_t rmm_pdev_phys;
	phys_addr_t rmm_vdev_phys;
	struct cca_host_pdev_dsc *pdev_dsc;
	phys_addr_t rd_phys;
	int ret = 0;
	int rc;

	cca_vdev_assert_locked(pdev);

	if (host_tdi->state == RMI_VDEV_UNLOCKED ||
	    host_tdi->state == RMI_VDEV_NEW)
		return -EINVAL;

	rd_phys = virt_to_phys(host_tdi->realm->rd);
	rmm_vdev_phys = virt_to_phys(host_tdi->rmm_vdev);
	pdev_dsc = to_cca_pdev_dsc(pf0_dev);
	rmm_pdev_phys = virt_to_phys(pdev_dsc->rmm_pdev);
	if (rmi_vdev_unlock(rd_phys, rmm_pdev_phys, rmm_vdev_phys))
		return -ENXIO;

	/* RMM now considers the VDEV unlocked, even if communication fails. */
	host_tdi->state = RMI_VDEV_UNLOCKED;

	if (rmm_has_reg2_feature(RMI_FEATURE_REGISTER_2_VDEV_KROU)) {
		struct pci_dev *rp = pcie_find_root_port(pf0_dev);
		struct cca_host_pf0_ep_dsc *pf0_ep_dsc =
			to_cca_pf0_ep_dsc(pf0_dev);
		unsigned long stream_handle = pf0_ep_dsc->stream_handle;

		rc = submit_vdev_state_transition_work(host_tdi, pdev, RMI_VDEV_KEY_REFRESH);
		if (rc && !ret)
			ret = rc;

		rc = cca_pdev_refresh_stream_key(pf0_dev, rp, stream_handle);
		if (rc && !ret)
			ret = rc;

		rc = cca_pdev_purge_stream_key(pf0_dev, rp, stream_handle);
		if (rc && !ret)
			ret = rc;
	}

	rc = submit_vdev_state_transition_work(host_tdi, pdev, RMI_VDEV_UNLOCKED);
	if (rc && !ret)
		ret = rc;

	return ret;
}

void cca_vdev_destroy(struct cca_host_tdi *host_tdi, struct realm *realm,
		      struct pci_dev *pdev, struct pci_dev *pf0_dev)
{
	phys_addr_t rmm_pdev_phys;
	phys_addr_t rmm_vdev_phys;
	struct cca_host_pdev_dsc *pdev_dsc;
	phys_addr_t rd_phys = virt_to_phys(realm->rd);
	int ret;

	cca_vdev_assert_locked(pdev);
	rmm_vdev_phys = virt_to_phys(host_tdi->rmm_vdev);

	if (host_tdi->state != RMI_VDEV_UNLOCKED) {
		ret = realm_dev_mem_unmap(host_tdi->kvm, rmm_vdev_phys);
		if (ret) {
			pci_err(pdev,
				"failed to unmap vdev memory before destroy (%d)\n",
				ret);
			return;
		}

		ret = cca_vdev_device_unlock(host_tdi, pdev);
		if (ret)
			pci_err(pdev, "failed to unlock vdev (%d)\n", ret);
	}

	pdev_dsc = to_cca_pdev_dsc(pf0_dev);
	rmm_pdev_phys = virt_to_phys(pdev_dsc->rmm_pdev);

	/* Try to destroy even in case of error */
	if (rmi_vdev_destroy(rd_phys, rmm_pdev_phys, rmm_vdev_phys))
		pci_err(pdev, "failed to destroy vdev\n");

	if (!rmi_undelegate_page(rmm_vdev_phys))
		free_page((unsigned long)host_tdi->rmm_vdev);

	host_tdi->rmm_vdev = NULL;
	host_tdi->realm = NULL;
}

static void __maybe_unused vdev_fetch_object_workfn(struct work_struct *work)
{
	int state;
	struct pci_tsm *tsm;
	struct cca_host_pdev_dsc *pdev_dsc;
	struct dev_comm_work *setup_work;

	setup_work = container_of(work, struct dev_comm_work, work);
	tsm = setup_work->tsm;
	pdev_dsc = to_cca_pdev_dsc(tsm->dsm_dev);

	guard(mutex)(&pdev_dsc->object_lock);

	if (setup_work->cache_size) {
		memset(setup_work->cache_buf, 0, setup_work->cache_size);
		*setup_work->cache_offset = 0;
	}
	state = do_dev_communicate(VDEV_COMMUNICATE, tsm,
				   setup_work->host_tdi, RMI_VDEV_ERROR, NULL);
	/* return status through dev_comm_work.cache_cache */
	if (state == RMI_VDEV_ERROR)
		setup_work->cache_size = 0;
	else
		/* indicate success. This value is not used. */
		setup_work->cache_size = CACHE_CHUNK_SIZE;
}

int cca_vdev_device_map(struct cca_host_tdi *host_tdi, struct pci_dev *pdev,
			unsigned long gpa_base, unsigned long gpa_top,
			unsigned long pa_base)
{
	struct kvm *kvm;
	phys_addr_t rmm_pdev_phys;
	phys_addr_t rmm_vdev_phys;
	struct cca_host_pdev_dsc *pdev_dsc;

	cca_vdev_assert_locked(pdev);

	pdev_dsc = to_cca_pdev_dsc(pdev->tsm->dsm_dev);
	kvm = host_tdi->kvm;
	rmm_vdev_phys = virt_to_phys(host_tdi->rmm_vdev);
	rmm_pdev_phys = virt_to_phys(pdev_dsc->rmm_pdev);

	return realm_dev_mem_map(kvm, rmm_pdev_phys, rmm_vdev_phys,
				 gpa_base, gpa_top, pa_base);
}

int cca_vdev_device_start(struct cca_host_tdi *host_tdi,
			  struct pci_dev *pdev)
{
	phys_addr_t rmm_pdev_phys;
	phys_addr_t rmm_vdev_phys;
	struct cca_host_pdev_dsc *pdev_dsc;
	struct realm *realm;
	phys_addr_t rd_phys;

	cca_vdev_assert_locked(pdev);

	if (host_tdi->state != RMI_VDEV_LOCKED)
		return -EINVAL;

	rmm_vdev_phys = virt_to_phys(host_tdi->rmm_vdev);
	realm = &host_tdi->kvm->arch.realm;
	rd_phys = virt_to_phys(realm->rd);

	pdev_dsc = to_cca_pdev_dsc(pdev->tsm->dsm_dev);
	rmm_pdev_phys = virt_to_phys(pdev_dsc->rmm_pdev);

	if (rmi_vdev_start(rd_phys, rmm_pdev_phys, rmm_vdev_phys))
		return -ENXIO;
	host_tdi->state = RMI_VDEV_STARTED;
	return submit_vdev_state_transition_work(host_tdi, pdev,
						 RMI_VDEV_STARTED);
}

int cca_vdev_device_lock(struct cca_host_tdi *host_tdi,
			 struct pci_dev *pdev)
{
	struct cca_host_pdev_dsc *pdev_dsc;
	phys_addr_t rmm_pdev_phys;
	phys_addr_t rmm_vdev_phys;
	phys_addr_t rd_phys;

	cca_vdev_assert_locked(pdev);

	if (host_tdi->state != RMI_VDEV_UNLOCKED)
		return -EINVAL;

	rmm_vdev_phys = virt_to_phys(host_tdi->rmm_vdev);
	rd_phys = virt_to_phys(host_tdi->realm->rd);
	pdev_dsc = to_cca_pdev_dsc(pdev->tsm->dsm_dev);
	rmm_pdev_phys = virt_to_phys(pdev_dsc->rmm_pdev);

	if (rmi_vdev_lock(rd_phys, rmm_pdev_phys, rmm_vdev_phys))
		return -ENXIO;

	/* Ensure teardown unlocks the VDEV if device communication fails. */
	host_tdi->state = RMI_VDEV_LOCKED;
	return submit_vdev_state_transition_work(host_tdi, pdev,
						 RMI_VDEV_LOCKED);
}
