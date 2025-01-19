// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 ARM Ltd.
 */

#include <linux/string.h>

#include "rsi-da.h"
#include "rhi-da.h"

/**
 * map_rhi_da_error - Map an RHI DA status to Linux errno
 * @rhi_da_error: RHI DA status value to translate
 *
 * Return: 0 for %RHI_DA_SUCCESS, %RHI_DA_INCOMPLETE when the caller must
 * continue the operation with rhi_vdev_continue(), or a negative errno for
 * all other failures. Unknown status codes are mapped to -EIO.
 */
static inline int map_rhi_da_error(unsigned long rhi_da_error)
{
	switch (rhi_da_error) {
	case RHI_DA_SUCCESS:
		return 0;
	case RHI_DA_INCOMPLETE:
		return RHI_DA_INCOMPLETE;
	case RHI_DA_ERROR_BUSY:
		return -EBUSY;
	case RHI_DA_ERROR_INPUT:
	case RHI_DA_ERROR_INVALID_VDEV_ID:
		return -EINVAL;
	case RHI_DA_ERROR_ACCESS_FAILED:
		return -EFAULT;
	case RHI_DA_ERROR_DEVICE:
		return -EIO;
	case RHI_DA_ERROR_INVALID_OBJECT:
		return -EINVAL;
	default:
		return -EIO;
	}
}

bool rhi_has_da_support(void)
{
	int ret;

	struct rsi_host_call *rhi_call __free(kfree) =
		kzalloc(sizeof(*rhi_call), GFP_KERNEL);
	if (!rhi_call)
		return false;

	rhi_call->imm = 0;
	rhi_call->gprs[0] = RHI_DA_FEATURES;

	ret = rsi_host_call(rhi_call);
	if (ret != RSI_SUCCESS || rhi_call->gprs[0] == SMCCC_RET_NOT_SUPPORTED)
		return false;

	/* For base DA to work we need these to be supported */
	if ((rhi_call->gprs[0] & RHI_DA_BASE_FEATURE) == RHI_DA_BASE_FEATURE)
		return true;

	return false;
}

static inline int rhi_vdev_continue(unsigned long vdev_id, unsigned long cookie)
{
	unsigned long ret;

	struct rsi_host_call *rhi_call __free(kfree) =
		kzalloc(sizeof(*rhi_call), GFP_KERNEL);
	if (!rhi_call)
		return -ENOMEM;

	rhi_call->imm = 0;
	rhi_call->gprs[0] = RHI_DA_VDEV_CONTINUE;
	rhi_call->gprs[1] = vdev_id;
	rhi_call->gprs[2] = cookie;

	ret = rsi_host_call(rhi_call);
	if (ret != RSI_SUCCESS)
		return -EIO;

	return map_rhi_da_error(rhi_call->gprs[0]);
}

static int __rhi_vdev_abort(unsigned long vdev_id, unsigned long *da_error)
{
	unsigned long ret;
	struct rsi_host_call *rhi_call __free(kfree) =
		kzalloc(sizeof(struct rsi_host_call), GFP_KERNEL);
	if (!rhi_call)
		return -ENOMEM;

	rhi_call->imm = 0;
	rhi_call->gprs[0] = RHI_DA_VDEV_ABORT;
	rhi_call->gprs[1] = vdev_id;

	ret = rsi_host_call(rhi_call);
	if (ret != RSI_SUCCESS)
		return -EIO;

	*da_error = rhi_call->gprs[0];
	return 0;
}

static bool should_abort_rhi_call_loop(unsigned long vdev_id)
{
	int ret;

	cond_resched();
	if (signal_pending(current)) {
		unsigned long da_error;

		ret = __rhi_vdev_abort(vdev_id, &da_error);
		/* consider all kind of error as not aborted */
		if (!ret && (da_error == RHI_DA_SUCCESS))
			return true;
	}
	return false;
}

static int __rhi_vdev_set_tdi_state(unsigned long vdev_id,
		enum rhi_tdi_state target_state, unsigned long *cookie)
{
	unsigned long ret;

	struct rsi_host_call *rhi_call __free(kfree) =
		kzalloc(sizeof(struct rsi_host_call), GFP_KERNEL);
	if (!rhi_call)
		return -ENOMEM;

	rhi_call->imm = 0;
	rhi_call->gprs[0] = RHI_DA_VDEV_SET_TDI_STATE;
	rhi_call->gprs[1] = vdev_id;
	rhi_call->gprs[2] = target_state;

	ret = rsi_host_call(rhi_call);
	if (ret != RSI_SUCCESS)
		return -EIO;

	*cookie = rhi_call->gprs[1];
	return map_rhi_da_error(rhi_call->gprs[0]);
}

int rhi_vdev_set_tdi_state(struct pci_dev *pdev, enum rhi_tdi_state target_state)
{
	int ret;
	unsigned long cookie;
	int vdev_id = rsi_vdev_id(pdev);

	for (;;) {
		ret = __rhi_vdev_set_tdi_state(vdev_id, target_state, &cookie);
		if (ret != -EBUSY)
			break;
		cond_resched();
	}

	while (ret == RHI_DA_INCOMPLETE) {
		if (should_abort_rhi_call_loop(vdev_id))
			return -EINTR;
		ret = rhi_vdev_continue(vdev_id, cookie);
	}

	return ret;
}

static inline int rhi_vdev_get_interface_report(unsigned long vdev_id,
		unsigned long *cookie)
{
	unsigned long ret;

	struct rsi_host_call *rhi_call __free(kfree) =
		kzalloc(sizeof(struct rsi_host_call), GFP_KERNEL);
	if (!rhi_call)
		return -ENOMEM;

	rhi_call->imm = 0;
	rhi_call->gprs[0] = RHI_DA_VDEV_GET_INTERFACE_REPORT;
	rhi_call->gprs[1] = vdev_id;

	ret = rsi_host_call(rhi_call);
	if (ret != RSI_SUCCESS)
		return -EIO;

	*cookie = rhi_call->gprs[1];
	return map_rhi_da_error(rhi_call->gprs[0]);
}

int rhi_update_vdev_interface_report_cache(struct pci_dev *pdev)
{
	int ret;
	unsigned long cookie;
	int vdev_id = rsi_vdev_id(pdev);

	for (;;) {
		ret = rhi_vdev_get_interface_report(vdev_id, &cookie);
		if (ret != -EBUSY)
			break;
		cond_resched();
	}

	while (ret == RHI_DA_INCOMPLETE) {
		if (should_abort_rhi_call_loop(vdev_id))
			return -EINTR;
		ret = rhi_vdev_continue(vdev_id, cookie);
	}

	return ret;
}

static inline int rhi_vdev_get_measurements(unsigned long vdev_id,
		phys_addr_t vdev_meas_phys, unsigned long *cookie)
{
	unsigned long ret;

	struct rsi_host_call *rhi_call __free(kfree) =
		kzalloc(sizeof(*rhi_call), GFP_KERNEL);
	if (!rhi_call)
		return -ENOMEM;

	rhi_call->imm = 0;
	rhi_call->gprs[0] = RHI_DA_VDEV_GET_MEASUREMENTS;
	rhi_call->gprs[1] = vdev_id;
	rhi_call->gprs[2] = vdev_meas_phys;

	ret = rsi_host_call(rhi_call);
	if (ret != RSI_SUCCESS)
		return -EIO;

	*cookie = rhi_call->gprs[1];
	return map_rhi_da_error(rhi_call->gprs[0]);
}

static inline struct rhi_vdev_measurement_params *alloc_vdev_meas_params(void)
{
	struct page *pages;

	pages = alloc_shared_pages(NUMA_NO_NODE, GFP_KERNEL,
				   sizeof(struct rhi_vdev_measurement_params));
	if (!pages)
		return NULL;
	return page_address(pages);
}

static inline void vdev_meas_params_free(struct rhi_vdev_measurement_params *params)
{
	struct page *pages = virt_to_page(params);

	free_shared_pages(pages, sizeof(struct rhi_vdev_measurement_params));
}

DEFINE_FREE(vdev_meas_params_free, struct rhi_vdev_measurement_params *, if (_T) vdev_meas_params_free(_T))
int rhi_update_vdev_measurements_cache(struct pci_dev *pdev, const u8 *nonce)
{
	int ret;
	unsigned long cookie;
	int vdev_id = rsi_vdev_id(pdev);
	phys_addr_t vdev_meas_phys;

	struct rhi_vdev_measurement_params *dev_meas __free(vdev_meas_params_free) =
		alloc_vdev_meas_params();
	if (!dev_meas)
		return -ENOMEM;

	vdev_meas_phys = virt_to_phys(dev_meas);
	/* request for raw bitstream */
	dev_meas->flags = RHI_VDEV_MEASURE_RAW;
	if (nonce)
		memcpy(dev_meas->nonce, nonce, 32);

	for (;;) {
		ret = rhi_vdev_get_measurements(vdev_id, vdev_meas_phys, &cookie);
		if (ret != -EBUSY)
			break;
		cond_resched();
	}

	while (ret == RHI_DA_INCOMPLETE) {
		if (should_abort_rhi_call_loop(vdev_id))
			return -EINTR;
		ret = rhi_vdev_continue(vdev_id, cookie);
	}

	if (ret)
		pci_err(pdev, "failed to get device measurement (%d)\n", ret);
	return ret;
}

int rhi_read_cached_object(int vdev_id, int da_object_type, void **object, size_t *object_size)
{
	int ret;
	size_t data_size;
	void *data_buf_shared;
	struct page *shared_pages;

	*object_size = 0;
	*object = NULL;

	struct rsi_host_call *rhicall __free(kfree) =
		kzalloc(sizeof(struct rsi_host_call), GFP_KERNEL);
	if (!rhicall)
		return -ENOMEM;

	rhicall->imm = 0;
	rhicall->gprs[0] = RHI_DA_OBJECT_SIZE;
	rhicall->gprs[1] = vdev_id;
	rhicall->gprs[2] = da_object_type;

	ret = rsi_host_call(rhicall);
	if (ret != RSI_SUCCESS)
		return -EIO;

	if (rhicall->gprs[0] != RHI_DA_SUCCESS)
		return -EIO;

	/* validate against the max cache object size used on host. */
	data_size = rhicall->gprs[1];
	if (data_size > MAX_CACHE_OBJ_SIZE || data_size == 0)
		return -EIO;

	shared_pages = alloc_shared_pages(NUMA_NO_NODE, GFP_KERNEL, data_size);
	if (!shared_pages)
		return -ENOMEM;

	data_buf_shared = page_address(shared_pages);

	rhicall->imm = 0;
	rhicall->gprs[0] = RHI_DA_OBJECT_READ;
	rhicall->gprs[1] = vdev_id;
	rhicall->gprs[2] = da_object_type;
	rhicall->gprs[3] = virt_to_phys(data_buf_shared);
	rhicall->gprs[4] = data_size;
	rhicall->gprs[5] = 0; /* offset to read from */
	ret = rsi_host_call(rhicall);
	if (ret != RSI_SUCCESS || rhicall->gprs[0] != RHI_DA_SUCCESS) {
		free_shared_pages(shared_pages, data_size);
		return -EIO;
	}

	if (data_size != rhicall->gprs[1]) {
		/* Short read */
		free_shared_pages(shared_pages, data_size);
		return -EIO;
	}

	void *data_buf_private = kvmemdup(data_buf_shared,
					  data_size, GFP_KERNEL);
	/* free the shared pages irrespective of error condition */
	free_shared_pages(shared_pages, data_size);
	if (!data_buf_private)
		return -ENOMEM;

	*object = data_buf_private;
	*object_size = data_size;
	return 0;
}
