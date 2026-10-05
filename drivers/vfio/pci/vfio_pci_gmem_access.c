// SPDX-License-Identifier: GPL-2.0-only
/* Revoke translations before delegation; shared/private state is per range. */
#include <linux/guest_memfd.h>
#include <linux/iommufd.h>
#include <linux/vfio_pci_core.h>

#include "vfio_pci_gmem.h"
#include "vfio_pci_priv.h"

/**
 * vfio_pci_gmem_invalidate() - notify KVM of revoked BAR mappings
 * @vdev: VFIO PCI device whose BAR translations are being revoked
 *
 * Called by vfio_pci_zap_bars() after it removes host BAR mappings. Notify
 * guest_memfd synchronously so KVM removes the corresponding shared guest
 * mappings. This helper does not remove host mappings or change attributes.
 * Skip notification if no provider is attached.
 *
 * The guest_memfd callback takes SRCU and the KVM MMU lock without taking
 * the inode invalidate lock, which may already be held during conversion.
 *
 */
void vfio_pci_gmem_invalidate(struct vfio_pci_core_device *vdev)
{
	struct vfio_pci_gmem *ctx = vdev->gmem;

	lockdep_assert_held_write(&vdev->memory_lock);
	if (ctx)
		ctx->context.device->invalidate(ctx->context.device);
}

/**
 * vfio_gmem_invalidate_mmio() - revoke host and shared guest BAR translations
 * @data: attached provider context
 *
 * Initiate BAR revocation for conversion or access refresh. Call
 * vfio_pci_zap_and_down_write_memory_lock() to remove host BAR mappings;
 * its BAR zap path also calls vfio_pci_gmem_invalidate() to remove the
 * corresponding shared guest mappings. Release the VFIO memory lock before
 * returning.
 *
 * During private conversion, the caller holds the inode invalidate lock to
 * prevent new guest_memfd faults. Host faults and I/O try that lock when
 * checking range access, so revocation completes before delegation.
 *
 * Caller holds the vDEVICE MMIO mutex; takes the VFIO memory lock for
 * write and releases it before architecture mapping.
 */
void vfio_gmem_invalidate_mmio(void *data)
{
	struct vfio_pci_gmem *ctx = vfio_pci_gmem_from_context(data);
	struct vfio_pci_core_device *vdev = ctx->vdev;

	/* IOMMUFD invokes this during attachment before ctx->ivdev is assigned. */
	if (ctx->ivdev)
		lockdep_assert_held(&ctx->ivdev->mmio_lock);

	vfio_pci_zap_and_down_write_memory_lock(vdev);
	up_write(&vdev->memory_lock);
}

/**
 * vfio_pci_gmem_access_allowed() - check a host BAR access against its range
 * @vdev: VFIO PCI device supplying the BAR
 * @offset: byte offset in the VFIO region namespace
 * @size: access length in bytes
 *
 * Consult completed guest_memfd attributes for the requested interval.
 * Private mappings in another interval do not block this access. Try the
 * inode lock instead of waiting in the reverse order from conversion.
 *
 * Return: true when the requested interval is accessible.
 */
bool vfio_pci_gmem_access_allowed(struct vfio_pci_core_device *vdev,
		u64 offset, u64 size)
{
	struct vfio_pci_gmem *ctx = vdev->gmem;

	lockdep_assert_held(&vdev->memory_lock);
	if (!ctx)
		return true;

	/* Attachment publishes the context before resolving its vDEVICE. */
	if (!ctx->ivdev)
		return false;

	return ctx->context.device->host_accessible(ctx->context.device,
						    offset, size);
}

/**
 * vfio_pci_gmem_io_access_allowed() - resolve a mapped BAR I/O pointer
 * @vdev: VFIO PCI device supplying the iomap
 * @io: address used by ordinary VFIO read, write or ioeventfd
 * @size: access width in bytes
 *
 * Standard BAR iomaps are cached in barmap. Resolve those to file offsets
 * before applying range admission. Generic VFIO's remaining memory I/O uses
 * a temporary ROM iomap, which cannot be bound to a device guest_memfd.
 * Callers supply VFIO's own validated iomap pointers.
 *
 * Return: true when the I/O interval is accessible.
 */
bool vfio_pci_gmem_io_access_allowed(struct vfio_pci_core_device *vdev,
		void __iomem *io, size_t size)
{
	unsigned long addr = (unsigned long)io;
	int bar;

	lockdep_assert_held(&vdev->memory_lock);
	if (!vdev->gmem)
		return vfio_pci_gmem_access_allowed(vdev, 0, size);

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		unsigned long base = (unsigned long)vdev->barmap[bar];
		u64 offset;

		if (!base || addr < base)
			continue;

		offset = addr - base;
		if (offset >= pci_resource_len(vdev->pdev, bar))
			continue;

		if (size > pci_resource_len(vdev->pdev, bar) - offset)
			return false;

		return vfio_pci_gmem_access_allowed(vdev,
				VFIO_PCI_INDEX_TO_OFFSET(bar) + offset, size);
	}
	/* The ROM has no private provider mappings, but lifecycle checks apply. */
	return vfio_pci_gmem_access_allowed(vdev,
			VFIO_PCI_INDEX_TO_OFFSET(VFIO_PCI_ROM_REGION_INDEX), size);
}
