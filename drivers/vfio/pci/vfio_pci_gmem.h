/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef VFIO_PCI_GMEM_H
#define VFIO_PCI_GMEM_H

#include <linux/guest_memfd.h>

struct iommufd_vdevice;
struct vfio_pci_core_device;

struct vfio_pci_gmem {
	struct guest_memfd *gmem;
	struct iommufd_vdevice *ivdev;
};

#endif /* VFIO_PCI_GMEM_H */
