/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef VFIO_PCI_GMEM_H
#define VFIO_PCI_GMEM_H

#include <linux/container_of.h>
#include <linux/guest_memfd.h>
#include <linux/pci.h>

struct iommufd_vdevice;
struct vfio_pci_core_device;

struct vfio_pci_gmem {
	struct guest_memfd_device_context context;
	struct vfio_pci_core_device *vdev;
	struct iommufd_vdevice *ivdev;
	struct file *resource;
	unsigned long owner;
	resource_size_t start[PCI_STD_NUM_BARS];
	resource_size_t len[PCI_STD_NUM_BARS];
};

static inline struct vfio_pci_gmem *
vfio_pci_gmem_from_context(struct guest_memfd_device_context *context)
{
	return container_of_const(context, struct vfio_pci_gmem, context);
}

void vfio_gmem_invalidate_mmio(void *data);

#endif /* VFIO_PCI_GMEM_H */
