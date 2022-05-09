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

/* ioasid_find getter() requires a void * argument */
static bool __mmget_not_zero(void *mm)
{
	return mmget_not_zero(mm);
}

/**
 * iommu_sva_find() - Find mm associated to the given PASID
 * @pasid: Process Address Space ID assigned to the mm
 *
 * On success a reference to the mm is taken, and must be released with mmput().
 *
 * Returns the mm corresponding to this PASID, or an error if not found.
 */
struct mm_struct *iommu_sva_find(ioasid_t pasid)
{
	return ioasid_find(&iommu_sva_pasid, pasid, __mmget_not_zero);
}
EXPORT_SYMBOL_GPL(iommu_sva_find);

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
