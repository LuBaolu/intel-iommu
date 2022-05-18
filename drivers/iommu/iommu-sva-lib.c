// SPDX-License-Identifier: GPL-2.0
/*
 * Helpers for IOMMU drivers implementing SVA
 */
#include <linux/mutex.h>
#include <linux/sched/mm.h>
#include <linux/pci.h>
#include <linux/pci-ats.h>

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
int iommu_sva_alloc_pasid(struct mm_struct *mm, ioasid_t min, ioasid_t max)
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
EXPORT_SYMBOL_GPL(iommu_sva_alloc_pasid);

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
struct iommu_domain *
iommu_sva_alloc_domain(struct bus_type *bus, struct mm_struct *mm)
{
	struct iommu_sva_domain *sva_domain;
	struct iommu_domain *domain;

	if (!bus->iommu_ops || !bus->iommu_ops->sva_domain_ops)
		return ERR_PTR(-ENODEV);

	sva_domain = kzalloc(sizeof(*sva_domain), GFP_KERNEL);
	if (!sva_domain)
		return ERR_PTR(-ENOMEM);

	mmgrab(mm);
	sva_domain->mm = mm;

	domain = &sva_domain->domain;
	domain->type = IOMMU_DOMAIN_SVA;
	domain->ops = bus->iommu_ops->sva_domain_ops;

	return domain;
}

void iommu_sva_free_domain(struct iommu_domain *domain)
{
	struct iommu_sva_domain *sva_domain = to_sva_domain(domain);

	mmdrop(sva_domain->mm);
	kfree(sva_domain);
}

int iommu_sva_set_domain(struct iommu_domain *domain, struct device *dev,
			 ioasid_t pasid)
{
	struct bus_type *bus = dev->bus;

	if (!bus || !bus->iommu_ops || !bus->iommu_ops->sva_domain_ops)
		return -ENODEV;

	if (domain->ops != bus->iommu_ops->sva_domain_ops)
		return -EINVAL;

	return iommu_set_device_pasid(domain, dev, pasid);
}

/**
 * iommu_sva_bind_device() - Bind a process address space to a device
 * @dev: the device
 * @mm: the mm to bind, caller must hold a reference to mm_users
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
struct iommu_sva *iommu_sva_bind_device(struct device *dev, struct mm_struct *mm)
{
	struct iommu_sva_domain *sva_domain;
	struct iommu_domain *domain;
	ioasid_t max_pasid = 0;
	int ret = -EINVAL;

	/* Allocate mm->pasid if necessary. */
	if (!dev->iommu->iommu_dev->pasids)
		return ERR_PTR(-EOPNOTSUPP);

	if (dev_is_pci(dev)) {
		max_pasid = pci_max_pasids(to_pci_dev(dev));
		if (max_pasid < 0)
			return ERR_PTR(max_pasid);
	} else {
		ret = device_property_read_u32(dev, "pasid-num-bits",
					       &max_pasid);
		if (ret)
			return ERR_PTR(ret);
		max_pasid = (1UL << max_pasid);
	}
	max_pasid = min_t(u32, max_pasid, dev->iommu->iommu_dev->pasids);
	ret = iommu_sva_alloc_pasid(mm, 1, max_pasid - 1);
	if (ret)
		return ERR_PTR(ret);

	mutex_lock(&iommu_sva_lock);
	/* Search for an existing domain. */
	domain = iommu_get_domain_for_dev_pasid(dev, mm->pasid);
	if (domain) {
		sva_domain = to_sva_domain(domain);
		refcount_inc(&sva_domain->bond.users);
		goto out_success;
	}

	/* Allocate a new domain and set it on device pasid. */
	domain = iommu_sva_alloc_domain(dev->bus, mm);
	if (IS_ERR(domain)) {
		ret = PTR_ERR(domain);
		goto out_unlock;
	}

	ret = iommu_sva_set_domain(domain, dev, mm->pasid);
	if (ret)
		goto out_free_domain;
	sva_domain = to_sva_domain(domain);
	sva_domain->bond.dev = dev;
	refcount_set(&sva_domain->bond.users, 1);

out_success:
	mutex_unlock(&iommu_sva_lock);
	return &sva_domain->bond;

out_free_domain:
	iommu_sva_free_domain(domain);
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
	struct iommu_sva_domain *sva_domain =
			container_of(handle, struct iommu_sva_domain, bond);
	ioasid_t pasid = iommu_sva_get_pasid(handle);

	mutex_lock(&iommu_sva_lock);
	if (refcount_dec_and_test(&sva_domain->bond.users)) {
		iommu_block_device_pasid(&sva_domain->domain, dev, pasid);
		iommu_sva_free_domain(&sva_domain->domain);
	}
	mutex_unlock(&iommu_sva_lock);
}
EXPORT_SYMBOL_GPL(iommu_sva_unbind_device);

u32 iommu_sva_get_pasid(struct iommu_sva *handle)
{
	struct iommu_sva_domain *sva_domain =
			container_of(handle, struct iommu_sva_domain, bond);

	return sva_domain->mm->pasid;
}
EXPORT_SYMBOL_GPL(iommu_sva_get_pasid);
