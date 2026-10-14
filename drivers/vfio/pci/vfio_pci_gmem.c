// SPDX-License-Identifier: GPL-2.0-only
#include <linux/guest_memfd.h>
#include <linux/iommufd.h>
#include <linux/overflow.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/vfio_pci_core.h>

#include "../vfio.h"
#include "vfio_pci_priv.h"
#include "vfio_pci_gmem.h"

/**
 * vfio_gmem_resolve_bar_range() - validate and resolve a VFIO BAR interval
 * @ctx: attached VFIO provider
 * @offset: byte offset in the VFIO region namespace
 * @size: interval length in bytes
 * @bar: output BAR index
 * @bar_offset: output byte offset within that BAR
 * @pa: output starting physical address
 *
 * Check alignment, mmap eligibility, and wrapper and BAR bounds. These checks
 * establish the provider interval; the VMM approves the protected REC
 * physical-address claim.
 * Output parameters are valid only on success.
 *
 * Return: 0 on success or -EINVAL for an ineligible interval.
 */
static int vfio_gmem_resolve_bar_range(struct vfio_pci_gmem *ctx, u64 offset,
		u64 size, u32 *bar, u64 *bar_offset, phys_addr_t *pa)
{
	struct pci_dev *pdev = ctx->vdev->pdev;
	resource_size_t start, len;
	u32 index = offset >> VFIO_PCI_OFFSET_SHIFT;
	u64 off = offset & ((1ULL << VFIO_PCI_OFFSET_SHIFT) - 1);

	if (!size || !PAGE_ALIGNED(offset | size))
		return -EINVAL;

	if (offset >= ctx->gmem->size ||
	    size > ctx->gmem->size - offset)
		return -EINVAL;

	if (index >= PCI_STD_NUM_BARS)
		return -EINVAL;

	start = pci_resource_start(pdev, index);
	len = pci_resource_len(pdev, index);

	if (!ctx->vdev->bar_mmap_supported[index] ||
	    !(pci_resource_flags(pdev, index) & IORESOURCE_MEM) ||
	    !PAGE_ALIGNED(start))
		return -EINVAL;

	if (off >= len || size > len - off)
		return -EINVAL;

	/* The interval must not cross into the next VFIO region. */
	if (size > (1ULL << VFIO_PCI_OFFSET_SHIFT) - off)
		return -EINVAL;

	if (check_add_overflow(start, off, pa))
		return -EINVAL;

	*bar = index;
	*bar_offset = off;
	return 0;
}

/**
 * vfio_gmem_has_private_mmio() - check for private BAR ranges
 * @data: attached VFIO provider context
 *
 * Return: true if the core has committed private BAR ranges.
 */
static bool vfio_gmem_has_private_mmio(void *data)
{
	struct vfio_pci_gmem *ctx = data;

	lockdep_assert_held(&ctx->ivdev->mmio_lock);
	return ctx->gmem->has_private(ctx->gmem);
}

/**
 * vfio_gmem_attach() - attach the ordinary VFIO PCI fd to guest_memfd
 * @file: open VFIO device file supplying the BAR namespace
 * @gdev: guest_memfd core callbacks and owning VM
 *
 * Require an opened IOMMUFD-backed file and exclude another provider and
 * dma-buf exports. Pin the source file and the existing vDEVICE after its
 * backend validates the VM. VFIO BAR VMAs and provider attachment are mutually
 * exclusive for the entire provider lifetime.
 *
 * Return: Provider context on success, or an ERR_PTR() on failure.
 */
static void *vfio_gmem_attach(struct file *file, struct guest_memfd *gdev)
{
	struct vfio_device_file *df = file->private_data;
	struct vfio_pci_core_device *vdev =
		container_of(df->device, struct vfio_pci_core_device, vdev);
	struct vfio_pci_gmem *ctx;
	struct iommufd_vdevice *ivdev;
	int ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->vdev = vdev;
	ctx->gmem = gdev;

	mutex_lock(&vdev->vdev.dev_set->lock);
	/* Paired with smp_store_release() after vfio_df_open(). */
	if (!smp_load_acquire(&df->access_granted) || !df->iommufd) {
		ret = -EINVAL;
		goto fail;
	}

	down_write(&vdev->memory_lock);
	/*
	 * Allow only one guest_memfd provider per device. Existing dma-buf
	 * exports bypass guest_memfd access checks. A BAR VMA can fault even
	 * if it has never installed a PTE. Exclude both under memory_lock,
	 * paired with mmap admission and dma-buf export.
	 */
	if (vdev->gmem || atomic_read(&vdev->mmap_count) ||
	    !list_empty(&vdev->dmabufs)) {
		up_write(&vdev->memory_lock);
		ret = -EBUSY;
		goto fail;
	}
	vdev->gmem = ctx;
	up_write(&vdev->memory_lock);
	ivdev = iommufd_device_attach_mmio_provider(vdev->vdev.iommufd_device,
						    gdev->kvm, ctx,
						    vfio_gmem_has_private_mmio,
						    &ctx->owner);
	if (IS_ERR(ivdev)) {
		ret = PTR_ERR(ivdev);
		scoped_guard(rwsem_write, &vdev->memory_lock)
			vdev->gmem = NULL;
		goto fail;
	}
	scoped_guard(rwsem_write, &vdev->memory_lock)
		ctx->ivdev = ivdev;
	/* Defer VFIO last-close and IOMMUFD unbind until gmem is released. */
	ctx->resource = get_file(file);
	mutex_unlock(&vdev->vdev.dev_set->lock);
	return ctx;
fail:
	mutex_unlock(&vdev->vdev.dev_set->lock);
	kfree(ctx);
	return ERR_PTR(ret);
}

/**
 * vfio_gmem_bind() - bind an eligible BAR interval to a memslot
 * @data: provider context returned by vfio_gmem_attach()
 * @offset: byte offset in the VFIO region namespace
 * @size: length in bytes
 * @binding: receives the pinned mapping owner and virtual device ID
 *
 * Validate the BAR interval and reserve the BAR through the ordinary
 * VFIO iomap machinery.
 *
 * Called with the inode invalidate lock held;
 * Return: 0 on success or a negative error code.
 */
static int vfio_gmem_bind(void *data, u64 offset, u64 size,
		struct guest_memfd_device_binding *binding)
{
	struct vfio_pci_gmem *ctx = data;
	phys_addr_t pa;
	u64 off;
	u32 bar;
	int ret;

	guard(iommufd_vdevice_mmio)(ctx->ivdev);
	if (ctx->callbacks_detached)
		return -ENODEV;
	ret = vfio_gmem_resolve_bar_range(ctx, offset, size, &bar, &off, &pa);
	if (ret)
		return ret;

	/* Reserve the physical resource with the ordinary VFIO BAR machinery. */
	ret = PTR_ERR_OR_ZERO(vfio_pci_core_get_iomap(ctx->vdev, bar));
	if (!ret) {
		binding->owner = ctx->owner;
		binding->vdev_id = ctx->ivdev->virt_id;
	}
	return ret;
}

/**
 * vfio_gmem_get_pfn() - obtain a shared BAR PFN for a guest fault
 * @data: attached provider context
 * @offset: page-aligned byte offset in the VFIO region namespace
 * @pfn: receives the bare BAR PFN
 *
 * Check shared-access admission, power state and PCI memory decoding under
 * the VFIO memory lock before resolving the resource. The PFN has no RAM
 * page reference to release. The core checks completed attributes and the
 * fault path checks the invalidation sequence before installing a mapping.
 *
 * Guest_memfd calls this only for shared device ranges. Private mappings use
 * the physical address from an approved protected device request through
 * kvm_arch_gmem_device_map(). Installing private mappings through ordinary
 * PFN faults would bypass that approval path.
 *
 * Called with the inode invalidate lock held
 * Return: 0 on success, -EAGAIN while shared access is unavailable, -ENODEV
 * after detachment, or a resource-validation error.
 */
static int vfio_gmem_get_pfn(void *data, u64 offset, unsigned long *pfn)
{
	struct vfio_pci_gmem *ctx = data;
	struct vfio_pci_core_device *vdev = ctx->vdev;
	phys_addr_t pa;
	u64 off;
	u32 bar;
	int ret;

	guard(rwsem_read)(&vdev->memory_lock);
	if (ctx->callbacks_detached)
		return -ENODEV;
	if (vdev->pm_runtime_engaged || !__vfio_pci_memory_enabled(vdev))
		return -EAGAIN;

	ret = vfio_gmem_resolve_bar_range(ctx, offset, PAGE_SIZE,
					  &bar, &off, &pa);
	if (!ret)
		*pfn = PHYS_PFN(pa);
	return ret;
}

/**
 * vfio_gmem_prepare_conversion() - serialize device range conversion
 * @context: attached provider context
 * @req: protected private request, or NULL for shared cleanup
 *
 * Check the virtual device ID before private mapping. The core has already
 * invalidated shared guest mappings; VFIO userspace mappings are excluded.
 * Keep that mutex held across architecture cleanup and attribute publication.
 *
 * Called with the inode invalidate lock held. On success,
 * vfio_gmem_finish_conversion() must release the retained MMIO mutex.
 * Return: 0 with the MMIO mutex held, or a negative error with it released.
 */
static int vfio_gmem_prepare_conversion(void *context,
		const struct guest_memfd_device_request *req)
{
	struct vfio_pci_gmem *ctx = context;

	iommufd_vdevice_mmio_lock(ctx->ivdev);
	if (req && ctx->callbacks_detached) {
		iommufd_vdevice_mmio_unlock(ctx->ivdev);
		return -ENODEV;
	}
	if (req && req->vdev_id != ctx->ivdev->virt_id) {
		iommufd_vdevice_mmio_unlock(ctx->ivdev);
		return -EPERM;
	}
	if (req) {
		/*
		 * VFIO BAR read/write and ioeventfd accesses hold memory_lock for
		 * reading through the MMIO operation. Wait for accesses that passed
		 * the shared-range check before conversion took the inode invalidate
		 * lock. New accesses to the attached range are denied because
		 * host_accessible() cannot acquire that lock for reading with
		 * filemap_invalidate_trylock_shared().
		 */
		down_write(&ctx->vdev->memory_lock);
		up_write(&ctx->vdev->memory_lock);
	}
	return 0; /* Keep mmio_lock held until finish_conversion(). */
}

/**
 * vfio_gmem_finish_conversion() - release the conversion mutex
 * @context: attached provider context
 * @req: protected private request, or NULL for shared cleanup
 *
 * Release the MMIO mutex after the core publishes completed attributes.
 * Subsequent faults and host I/O check their own range; no device-wide private
 * or shared admission state is published here.
 *
 * Called with the inode invalidate lock and the vDEVICE MMIO mutex held
 * after successful preparation.
 */
static void vfio_gmem_finish_conversion(void *context,
		const struct guest_memfd_device_request *req)
{
	struct vfio_pci_gmem *ctx = context;

	iommufd_vdevice_mmio_unlock(ctx->ivdev);
}

/**
 * vfio_gmem_detach() - stop invalidation callbacks before core teardown
 * @data: attached provider context
 *
 * Mark the provider detached under the VFIO memory lock. VFIO userspace
 * mappings remain excluded until release clears the provider association.
 */
static void vfio_gmem_detach(void *data)
{
	struct vfio_pci_gmem *ctx = data;

	guard(iommufd_vdevice_mmio)(ctx->ivdev);
	scoped_guard(rwsem_write, &ctx->vdev->memory_lock)
		ctx->callbacks_detached = true;
}

/**
 * vfio_gmem_release() - release a provider after successful core cleanup
 * @data: provider context to release
 *
 * Detach from the pinned vDEVICE, clear the VFIO provider association and
 * drop the source file reference before freeing the context.
 *
 * Takes the VFIO device-set lock and the vDEVICE MMIO and VFIO memory
 * locks while detaching.
 */
static void vfio_gmem_release(void *data)
{
	struct vfio_pci_gmem *ctx = data;
	struct vfio_pci_core_device *vdev = ctx->vdev;

	mutex_lock(&vdev->vdev.dev_set->lock);
	iommufd_vdevice_detach_mmio_provider(ctx->ivdev);
	down_write(&vdev->memory_lock);
	vdev->gmem = NULL;
	up_write(&vdev->memory_lock);
	mutex_unlock(&vdev->vdev.dev_set->lock);
	fput(ctx->resource);
	kfree(ctx);
}

const struct guest_memfd_provider_operations vfio_pci_gmem_provider_ops = {
	.attach = vfio_gmem_attach,
	.bind = vfio_gmem_bind,
	.get_pfn = vfio_gmem_get_pfn,
	.prepare_conversion = vfio_gmem_prepare_conversion,
	.finish_conversion = vfio_gmem_finish_conversion,
	.detach = vfio_gmem_detach,
	.release = vfio_gmem_release,
};
EXPORT_SYMBOL_GPL(vfio_pci_gmem_provider_ops);

MODULE_IMPORT_NS("IOMMUFD");
