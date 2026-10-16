// SPDX-License-Identifier: GPL-2.0-only
/* Shared/private access and guest revocation are controlled by guest_memfd. */
#include <linux/guest_memfd.h>
#include <linux/iommufd.h>
#include <linux/kvm.h>
#include <linux/vfio_pci_core.h>

#include "vfio_pci_gmem.h"
#include "vfio_pci_priv.h"

/**
 * vfio_pci_gmem_invalidate() - notify KVM of BAR mappings invalidate
 * @vdev: VFIO PCI device whose BAR translations are being revoked
 *
 * Called by vfio_pci_zap_bars() after it removes host BAR mappings. Notify
 * guest_memfd synchronously so KVM removes the corresponding shared guest
 * mappings. This helper does not remove host mappings or change attributes.
 * Skip notification if no provider is attached or its callbacks are detached.
 *
 */
void vfio_pci_gmem_invalidate(struct vfio_pci_core_device *vdev)
{
	struct vfio_pci_gmem *ctx = vdev->gmem;

	lockdep_assert_held_write(&vdev->memory_lock);
	if (ctx && !ctx->callbacks_detached)
		ctx->gmem->invalidate(ctx->gmem);
}

/**
 * vfio_gmem_invalidate_mmio() - revoke VFIO and shared guest BAR mappings
 * @ctx: attached provider context
 *
 * Drain VFIO faults and host I/O, then remove existing BAR PTEs before
 * delegation or teardown. The BAR zap also invalidates shared guest mappings
 * while core callbacks remain attached. During private conversion the inode
 * invalidate lock excludes new access through the host_accessible() check.
 *
 * Caller holds the vDEVICE MMIO mutex, except during attachment before the
 * vDEVICE is assigned. No private conversion can start at that point.
 */
void vfio_gmem_invalidate_mmio(struct vfio_pci_gmem *ctx)
{
	if (ctx->ivdev)
		lockdep_assert_held(&ctx->ivdev->mmio_lock);

	vfio_pci_zap_and_down_write_memory_lock(ctx->vdev);
	up_write(&ctx->vdev->memory_lock);
}

/* Serialize the creation flag check with attachment under memory_lock. */
bool vfio_pci_gmem_mmap_allowed(struct vfio_pci_core_device *vdev)
{
	struct vfio_pci_gmem *ctx = vdev->gmem;

	lockdep_assert_held(&vdev->memory_lock);
	return !ctx || (ctx->gmem->flags & GUEST_MEMFD_FLAG_MMAP);
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
 * Detachment excludes access during teardown.
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

	if (ctx->callbacks_detached)
		return false;
	return ctx->gmem->host_accessible(ctx->gmem,
					  offset, size);
}

/**
 * vfio_pci_gmem_io_access_allowed() - resolve a mapped BAR address
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
