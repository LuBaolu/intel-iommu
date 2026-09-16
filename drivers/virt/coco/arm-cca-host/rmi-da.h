/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 ARM Ltd.
 */

#ifndef _VIRT_COCO_RMM_DA_H_
#define _VIRT_COCO_RMM_DA_H_

#include <linux/arm-smccc-rhi.h>
#include <linux/arm-smccc-rmi.h>
#include <linux/arm-rmi-cmds.h>
#include <linux/pci.h>
#include <linux/pci-ide.h>
#include <linux/pci-tsm.h>
#include <linux/sizes.h>
#include <linux/atomic.h>
#include <linux/kref.h>
#include <linux/wait.h>

#define MAX_CACHE_OBJ_SIZE	SZ_16M
#define CACHE_CHUNK_SIZE	SZ_4K
struct cache_object {
	int size;
	int offset;
	u8 buf[] __counted_by(size);
};

struct dev_comm_work {
	struct pci_tsm *tsm;
	int target_state;
	struct work_struct work;
};

struct stream_sync {
	wait_queue_head_t wq;
	atomic_t val[2];
};

struct stream_connect_work {
	struct pci_tsm *tsm;
	struct work_struct work;
	struct stream_sync *sync;
	u8 my_index;
	bool has_peer;
};

struct cca_host_comm_data {
	void *rsp_buff;
	void *req_buff;
	struct rmi_dev_comm_data *io_params;
	/*
	 * Only one device communication request can be active at
	 * a time. This limitation comes from using the DOE mailbox
	 * at the pdev level. Requests such as get_measurements may
	 * span multiple mailbox messages, which must not be
	 * interleaved with other SPDM requests.
	 */
	struct workqueue_struct *work_queue;
};

/**
 * struct cca_host_pdev_dsc - Common RMM pdev context
 * @comm_data: Shared device communication state for the DSM-owned pdev
 * @rmm_pdev: Delegated page backing the RMM pdev object
 * @object_lock: Serializes access to the RMM pdev object and PF0/TDI caches
 */
struct cca_host_pdev_dsc {
	struct cca_host_comm_data comm_data;
	void *rmm_pdev;
	/* lock kept here to simplify the generic lock/unlock paths. */
	struct mutex object_lock;
};

/**
 * struct cca_host_pf0_ep_dsc - PF0 endpoint device security context.
 * @pci: Physical Function 0 TDISP link context
 * @pdev: pdev communication context
 * @sel_stream: Selective IDE Stream descriptor
 * @stream_handle: Stream handle returned by stream connect
 * @rmi_signature_algorithm: Signature algorithm used for public key
 * @cert_chain: cetrificate chain
 * @vca: SPDM's Version-Capabilities-Algorithms cache object
 */
struct cca_host_pf0_ep_dsc {
	struct pci_tsm_pf0 pci;
	struct cca_host_pdev_dsc pdev;
	struct pci_ide *sel_stream;
	unsigned long stream_handle;

	char *rmi_signature_algorithm;
	struct {
		struct cache_object *cache;

		void *public_key;
		size_t public_key_size;

		bool valid;
	} cert_chain;
	struct cache_object *vca;
};

/**
 * struct cca_host_rp_dsc - Root-port pdev context for stream coordination.
 * @pci: Root-port TSM link context
 * @pdev: Common pdev communication context
 * @tsm_ref: Reference count held by connected endpoint streams
 */
struct cca_host_rp_dsc {
	struct pci_tsm pci;
	struct cca_host_pdev_dsc pdev;
	struct kref tsm_ref;
};

struct cca_host_fn_dsc {
	struct pci_tsm pci;
};

enum dev_comm_type {
	PDEV_COMMUNICATE = 0x1,
	VDEV_COMMUNICATE = 0x2,
};

struct cca_host_tdi {
	struct kvm *kvm;
	struct realm *realm;
	void *rmm_vdev;
	/* Protected by the PCI TSM PF0 lock. */
	enum rmi_vdev_state state;
	/* protected by cca_host_pdev_dsc.object_lock */
	struct cache_object *interface_report;
	struct cache_object *measurements;
};

static inline int insert_addr_range_sorted(struct rmi_addr_range *addr_range,
		int nr_addr_range, resource_size_t start, resource_size_t top)
{
	int index = nr_addr_range;

	while (index > 0) {
		struct rmi_addr_range *prev = &addr_range[index - 1];

		if (prev->base < start)
			break;

		if (prev->base == start && prev->top <= top)
			break;

		addr_range[index] = *prev;
		index--;
	}

	addr_range[index].base = start;
	addr_range[index].top = top;

	return nr_addr_range + 1;
}

static inline struct cca_host_pf0_ep_dsc *to_cca_pf0_ep_dsc(struct pci_dev *pdev)
{
	struct pci_tsm *tsm = pdev->tsm;

	if (!tsm || !is_pci_tsm_pf0(pdev))
		return NULL;

	return container_of(tsm, struct cca_host_pf0_ep_dsc, pci.base_tsm);
}

static inline struct cca_host_fn_dsc *to_cca_fn_dsc(struct pci_dev *pdev)
{
	struct pci_tsm *tsm = pdev->tsm;

	return container_of(tsm, struct cca_host_fn_dsc, pci);
}

static inline struct cca_host_rp_dsc *to_cca_rp_dsc(struct pci_dev *pdev)
{
	struct pci_tsm *tsm = pdev->tsm;

	if (!tsm || pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT)
		return NULL;

	return container_of(tsm, struct cca_host_rp_dsc, pci);
}

static inline struct cca_host_pdev_dsc *to_cca_pdev_dsc(struct pci_dev *pdev)
{
	struct cca_host_pf0_ep_dsc *pf0_ep_dsc;
	struct cca_host_rp_dsc *rp_dsc;

	pf0_ep_dsc = to_cca_pf0_ep_dsc(pdev);
	if (pf0_ep_dsc)
		return &pf0_ep_dsc->pdev;

	rp_dsc = to_cca_rp_dsc(pdev);
	if (rp_dsc)
		return &rp_dsc->pdev;

	return NULL;
}

static inline struct cca_host_comm_data *to_cca_comm_data(struct pci_dev *pdev)
{
	struct cca_host_pdev_dsc *pdev_dsc;

	pdev_dsc = to_cca_pdev_dsc(pdev);
	if (pdev_dsc)
		return &pdev_dsc->comm_data;

	if (!pdev->tsm || !pdev->tsm->dsm_dev)
		return NULL;

	pdev_dsc = to_cca_pdev_dsc(pdev->tsm->dsm_dev);
	if (pdev_dsc)
		return &pdev_dsc->comm_data;

	return NULL;
}

int cca_pdev_create(struct pci_dev *pdev);
int cca_pdev_collect_identity(struct pci_dev *pdev);
bool cca_pdev_needs_key(struct pci_dev *pdev);
int cca_pdev_set_public_key(struct pci_dev *pdev);
void cca_pdev_stop_and_destroy(struct pci_dev *pdev);
int cca_pdev_stream_connect(struct pci_dev *pdev1, struct pci_dev *pdev2,
		struct rmi_pdev_stream_params *stream_params,
		unsigned long *stream_handle);
int cca_pdev_disconnect_stream(struct pci_dev *pdev1,
		struct pci_dev *pdev2, unsigned long stream_handle);

#endif
