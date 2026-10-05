/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_GUEST_MEMFD_H
#define _LINUX_GUEST_MEMFD_H

#include <linux/types.h>

struct file;
struct file_operations;
struct folio;
struct mempolicy;
struct kvm;
struct guest_memfd_device;

/* Claims come from a pending architecture exit, never from a userspace PA. */
struct guest_memfd_device_request {
	u64 gpa;
	u64 pa;
	u64 vdev_id;
};

/* Binding data stays valid only while the device guest_memfd is active. */
struct guest_memfd_device_binding {
	/* Architecture-specific mapping owner, e.g. an RMM vDEVICE handle. */
	unsigned long owner;
	u64 vdev_id;
};

/* Embedded by a device provider and returned from attach(). */
struct guest_memfd_device_context {
	struct guest_memfd_device *device;
};

struct guest_memfd_device_operations {
	struct guest_memfd_device_context *(*attach)(struct file *resource,
		struct guest_memfd_device *gdev);
	int (*bind)(void *data, u64 offset, u64 size,
		struct guest_memfd_device_binding *binding);
	int (*get_pfn)(void *data, u64 offset, unsigned long *pfn);
	/* A protected request prepares private mapping; NULL prepares sharing. */
	int (*prepare_conversion)(struct guest_memfd_device_context *context,
		const struct guest_memfd_device_request *req);
	void (*finish_conversion)(struct guest_memfd_device_context *context,
		const struct guest_memfd_device_request *req);
	void (*detach)(void *data);
	void (*release)(void *data);
};

/* A registered file type stores device operations in file->private_data. */
int guest_memfd_register_device_fops(const struct file_operations *fops);
void guest_memfd_unregister_device_fops(const struct file_operations *fops);

struct guest_memfd_device {
	struct kvm *kvm;
	u64 size;
	void *core;
	void (*invalidate)(struct guest_memfd_device *gdev);
	bool (*has_private)(struct guest_memfd_device *gdev);
	bool (*host_accessible)(struct guest_memfd_device *gdev, u64 offset, u64 size);
};

/**
 * struct guest_memfd_provider_operations - Operations for external memory providers
 * @attach: Attach to a resource file, perform filesystem-specific validation,
 *          and return an opaque provider context.
 * @release: Release provider context and unpin any resources.
 * @alloc_folio: Allocate an uninserted folio for a given index and NUMA policy.
 * @invalidate_folio: Notify provider that a folio has been invalidated.
 *
 * Used by filesystems and device drivers that provide memory for guest_memfd.
 */
struct guest_memfd_provider_operations {
	void *(*attach)(struct file *resource_file);
	void (*release)(void *provider);
	struct folio *(*alloc_folio)(void *provider, pgoff_t index,
				     struct mempolicy *mpol);
	void (*invalidate_folio)(void *provider, struct folio *folio);
};

#endif /* _LINUX_GUEST_MEMFD_H */
