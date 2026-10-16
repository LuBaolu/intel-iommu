/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef VFIO_PCI_GMEM_H
#define VFIO_PCI_GMEM_H

#include <linux/guest_memfd.h>

struct iommufd_vdevice;
struct vfio_pci_core_device;

struct vfio_pci_gmem {
	struct guest_memfd *gmem;
	struct vfio_pci_core_device *vdev;
	struct iommufd_vdevice *ivdev;
	struct file *resource;
	unsigned long owner;
	/*
	 * Disables callbacks and new bind/private requests. Set once with
	 * both the vDEVICE MMIO and VFIO memory locks held; readers hold
	 * either lock. Shared cleanup remains allowed after detachment.
	 */
	bool callbacks_detached;
};

void vfio_gmem_invalidate_mmio(struct vfio_pci_gmem *ctx);

#endif /* VFIO_PCI_GMEM_H */
