// SPDX-License-Identifier: GPL-2.0
/*
 * Helpers for IOMMU drivers implementing SVA
 */
#include <linux/mutex.h>
#include <linux/iommu.h>
#include <linux/slab.h>
#include <linux/sched/mm.h>

#include "iommu-sva-lib.h"

static DEFINE_MUTEX(iommu_sva_lock);
static DECLARE_IOASID_SET(iommu_sva_pasid);

/**
 * iommu_sva_alloc_pasid - Allocate a PASID for the mm
 * @mm: the mm
 * @min: minimum PASID value (inclusive)
 * @max: maximum PASID value (inclusive)
 *
 * Try to allocate a PASID for this mm, or take a reference to the existing one
 * provided it fits within the [@min, @max] range. On success the PASID is
 * available in mm->pasid and will be available for the lifetime of the mm.
 *
 * Returns 0 on success and < 0 on error.
 */
static int iommu_sva_alloc_pasid(struct mm_struct *mm,
				 ioasid_t min, ioasid_t max)
{
	int ret = 0;
	ioasid_t pasid;

	if (min == INVALID_IOASID || max == INVALID_IOASID ||
	    min == 0 || max < min)
		return -EINVAL;

	mutex_lock(&iommu_sva_lock);
	/* Is a PASID already associated with this mm? */
	if (pasid_valid(mm->pasid)) {
		if (mm->pasid < min || mm->pasid >= max)
			ret = -EOVERFLOW;
		goto out;
	}

	pasid = ioasid_alloc(&iommu_sva_pasid, min, max, mm);
	if (!pasid_valid(pasid))
		ret = -ENOMEM;
	else
		mm_pasid_set(mm, pasid);
out:
	mutex_unlock(&iommu_sva_lock);
	return ret;
}

/*
 * I/O page fault handler for SVA
 *
 * Copied from io-pgfault.c with mmget_not_zero() added before
 * mmap_read_lock().
 */
static enum iommu_page_response_code
iommu_sva_handle_iopf(struct iommu_fault *fault, void *data)
{
	vm_fault_t ret;
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	unsigned int access_flags = 0;
	struct iommu_domain *domain = data;
	unsigned int fault_flags = FAULT_FLAG_REMOTE;
	struct iommu_fault_page_request *prm = &fault->prm;
	enum iommu_page_response_code status = IOMMU_PAGE_RESP_INVALID;

	if (!(prm->flags & IOMMU_FAULT_PAGE_REQUEST_PASID_VALID))
		return status;

	mm = domain->mm;
	if (IS_ERR_OR_NULL(mm) || !mmget_not_zero(mm))
		return status;

	mmap_read_lock(mm);

	vma = find_extend_vma(mm, prm->addr);
	if (!vma)
		/* Unmapped area */
		goto out_put_mm;

	if (prm->perm & IOMMU_FAULT_PERM_READ)
		access_flags |= VM_READ;

	if (prm->perm & IOMMU_FAULT_PERM_WRITE) {
		access_flags |= VM_WRITE;
		fault_flags |= FAULT_FLAG_WRITE;
	}

	if (prm->perm & IOMMU_FAULT_PERM_EXEC) {
		access_flags |= VM_EXEC;
		fault_flags |= FAULT_FLAG_INSTRUCTION;
	}

	if (!(prm->perm & IOMMU_FAULT_PERM_PRIV))
		fault_flags |= FAULT_FLAG_USER;

	if (access_flags & ~vma->vm_flags)
		/* Access fault */
		goto out_put_mm;

	ret = handle_mm_fault(vma, prm->addr, fault_flags, NULL);
	status = ret & VM_FAULT_ERROR ? IOMMU_PAGE_RESP_INVALID :
		IOMMU_PAGE_RESP_SUCCESS;

out_put_mm:
	mmap_read_unlock(mm);
	mmput(mm);

	return status;
}

/*
 * IOMMU SVA driver-oriented interfaces
 */
static struct iommu_domain *
iommu_sva_alloc_domain(struct device *dev, struct mm_struct *mm)
{
	struct bus_type *bus = dev->bus;
	struct iommu_domain *domain;

	if (!bus || !bus->iommu_ops)
		return NULL;

	domain = bus->iommu_ops->domain_alloc(IOMMU_DOMAIN_SVA);
	if (!domain)
		return NULL;

	mmgrab(mm);
	domain->mm = mm;
	domain->type = IOMMU_DOMAIN_SVA;
	domain->iopf_handler = iommu_sva_handle_iopf;
	domain->fault_data = domain;

	return domain;
}

static void iommu_sva_free_domain(struct iommu_domain *domain)
{
	mmdrop(domain->mm);
	iommu_domain_free(domain);
}

/**
 * iommu_sva_bind_device() - Bind a process address space to a device
 * @dev: the device
 * @mm: the mm to bind, caller must hold a reference to mm_users
 * @drvdata: opaque data pointer to pass to bind callback
 *
 * Create a bond between device and address space, allowing the device to access
 * the mm using the returned PASID. If a bond already exists between @device and
 * @mm, it is returned and an additional reference is taken. Caller must call
 * iommu_sva_unbind_device() to release each reference.
 *
 * iommu_dev_enable_feature(dev, IOMMU_DEV_FEAT_SVA) must be called first, to
 * initialize the required SVA features.
 *
 * On error, returns an ERR_PTR value.
 */
struct iommu_sva *
iommu_sva_bind_device(struct device *dev, struct mm_struct *mm, void *drvdata)
{
	int ret = -EINVAL;
	struct iommu_sva *handle;
	struct iommu_domain *domain;

	/*
	 * TODO: Remove the drvdata parameter after kernel PASID support is
	 * enabled for the idxd driver.
	 */
	if (drvdata)
		return ERR_PTR(-EOPNOTSUPP);

	/* Allocate mm->pasid if necessary. */
	ret = iommu_sva_alloc_pasid(mm, 1, (1U << dev->iommu->pasid_bits) - 1);
	if (ret)
		return ERR_PTR(ret);

	mutex_lock(&iommu_sva_lock);
	/* Search for an existing bond. */
	handle = xa_load(&dev->iommu->sva_bonds, mm->pasid);
	if (handle) {
		refcount_inc(&handle->users);
		goto out_success;
	}

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	domain = iommu_sva_alloc_domain(dev, mm);
	if (!domain) {
		ret = -ENODEV;
		goto out_free_bond;
	}

	ret = iommu_attach_device_pasid(domain, dev, mm->pasid);
	if (ret)
		goto out_free_domain;

	handle->dev = dev;
	handle->domain = domain;
	refcount_set(&handle->users, 1);
	ret = xa_err(xa_store(&dev->iommu->sva_bonds, mm->pasid,
			      handle, GFP_KERNEL));
	if (ret)
		goto out_detach_domain;

out_success:
	mutex_unlock(&iommu_sva_lock);
	return handle;

out_detach_domain:
	iommu_detach_device_pasid(domain, dev, mm->pasid);
out_free_domain:
	iommu_sva_free_domain(domain);
out_free_bond:
	kfree(handle);
out_unlock:
	mutex_unlock(&iommu_sva_lock);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(iommu_sva_bind_device);

/**
 * iommu_sva_unbind_device() - Remove a bond created with iommu_sva_bind_device
 * @handle: the handle returned by iommu_sva_bind_device()
 *
 * Put reference to a bond between device and address space. The device should
 * not be issuing any more transaction for this PASID. All outstanding page
 * requests for this PASID must have been flushed to the IOMMU.
 */
void iommu_sva_unbind_device(struct iommu_sva *handle)
{
	struct device *dev = handle->dev;
	struct iommu_domain *domain = handle->domain;
	ioasid_t pasid = iommu_sva_get_pasid(handle);

	mutex_lock(&iommu_sva_lock);
	if (refcount_dec_and_test(&handle->users)) {
		xa_erase(&dev->iommu->sva_bonds, pasid);
		iommu_detach_device_pasid(domain, dev, pasid);
		iommu_sva_free_domain(domain);
		kfree(handle);
	}
	mutex_unlock(&iommu_sva_lock);
}
EXPORT_SYMBOL_GPL(iommu_sva_unbind_device);

u32 iommu_sva_get_pasid(struct iommu_sva *handle)
{
	return handle->domain->mm->pasid;
}
EXPORT_SYMBOL_GPL(iommu_sva_get_pasid);
