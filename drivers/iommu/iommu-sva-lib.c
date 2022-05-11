// SPDX-License-Identifier: GPL-2.0
/*
 * Helpers for IOMMU drivers implementing SVA
 */
#include <linux/mutex.h>
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
