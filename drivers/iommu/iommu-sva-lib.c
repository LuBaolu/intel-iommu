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
static DEFINE_XARRAY_ALLOC(sva_domain_array);

struct iommu_sva_cookie {
	struct mm_struct *mm;
	ioasid_t pasid;
	refcount_t users;
	struct list_head bonds;
};

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

static struct iommu_domain *
iommu_sva_alloc_domain(struct device *dev, struct mm_struct *mm)
{
	struct bus_type *bus = dev->bus;
	struct iommu_sva_cookie *cookie;
	struct iommu_domain *domain;
	void *curr;

	if (!bus || !bus->iommu_ops)
		return NULL;

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return NULL;

	domain = bus->iommu_ops->domain_alloc(IOMMU_DOMAIN_SVA);
	if (!domain)
		goto err_domain_alloc;

	cookie->mm = mm;
	cookie->pasid = mm->pasid;
	refcount_set(&cookie->users, 1);
	INIT_LIST_HEAD(&cookie->bonds);
	domain->type = IOMMU_DOMAIN_SVA;
	domain->sva_cookie = cookie;
	curr = xa_store(&sva_domain_array, mm->pasid, domain, GFP_KERNEL);
	if (xa_err(curr))
		goto err_xa_store;

	return domain;
err_xa_store:
	domain->ops->free(domain);
err_domain_alloc:
	kfree(cookie);
	return NULL;
}

static void iommu_sva_free_domain(struct iommu_domain *domain)
{
	xa_erase(&sva_domain_array, domain->sva_cookie->pasid);
	WARN_ON(!list_empty(&domain->sva_cookie->bonds));
	kfree(domain->sva_cookie);
	domain->ops->free(domain);
}

bool iommu_sva_domain_get_user(struct iommu_domain *domain)
{
	struct iommu_sva_cookie *cookie = domain->sva_cookie;

	return refcount_inc_not_zero(&cookie->users);
}

void iommu_sva_domain_put_user(struct iommu_domain *domain)
{
	struct iommu_sva_cookie *cookie = domain->sva_cookie;

	if (refcount_dec_and_test(&cookie->users))
		iommu_sva_free_domain(domain);
}

static struct iommu_domain *
iommu_sva_get_domain(struct device *dev, struct mm_struct *mm)
{
	struct iommu_domain *domain;
	ioasid_t pasid = mm->pasid;

	if (pasid == INVALID_IOASID)
		return NULL;

	domain = xa_load(&sva_domain_array, pasid);
	if (!domain)
		return iommu_sva_alloc_domain(dev, mm);
	iommu_sva_domain_get_user(domain);

	return domain;
}

struct mm_struct *iommu_sva_domain_mm(struct iommu_domain *domain)
{
	return domain->sva_cookie->mm;
}

/**
 * iommu_sva_bind_device() - Bind a process address space to a device
 * @dev: the device
 * @mm: the mm to bind, caller must hold a reference to it
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

	ret = iommu_sva_alloc_pasid(mm, 1, (1U << dev->iommu->pasid_bits) - 1);
	if (ret)
		return ERR_PTR(ret);

	mutex_lock(&iommu_sva_lock);
	domain = iommu_sva_get_domain(dev, mm);
	if (!domain) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	/* Search for an existing bond. */
	list_for_each_entry(handle, &domain->sva_cookie->bonds, node) {
		if (handle->dev == dev && handle->pasid == mm->pasid) {
			refcount_inc(&handle->users);
			mutex_lock(&iommu_sva_lock);

			return handle;
		}
	}

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle) {
		ret = -ENOMEM;
		goto out_put_domain;
	}

	ret = iommu_attach_device_pasid(domain, dev, mm->pasid);
	if (ret)
		goto out_free_handle;

	handle->dev = dev;
	handle->domain = domain;
	handle->pasid = mm->pasid;
	refcount_set(&handle->users, 1);
	list_add_tail(&handle->node, &domain->sva_cookie->bonds);

	mutex_unlock(&iommu_sva_lock);
	return handle;

out_free_handle:
	kfree(handle);
out_put_domain:
	iommu_sva_domain_put_user(domain);
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
	struct mm_struct *mm = iommu_sva_domain_mm(domain);

	mutex_lock(&iommu_sva_lock);
	if (refcount_dec_and_test(&handle->users)) {
		list_del(&handle->node);
		iommu_detach_device_pasid(domain, dev, mm->pasid);
		kfree(handle);
	}

	iommu_sva_domain_put_user(domain);
	mutex_unlock(&iommu_sva_lock);
}
EXPORT_SYMBOL_GPL(iommu_sva_unbind_device);

u32 iommu_sva_get_pasid(struct iommu_sva *handle)
{
	return handle->pasid;
}
EXPORT_SYMBOL_GPL(iommu_sva_get_pasid);
