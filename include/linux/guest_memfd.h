/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_GUEST_MEMFD_H
#define _LINUX_GUEST_MEMFD_H

#include <linux/types.h>

struct file;
struct folio;
struct mempolicy;
struct kvm;

/* Binding data stays valid only while the provider is active. */
struct guest_memfd_device_binding {
	/* Architecture-specific mapping owner, e.g. an RMM vDEVICE handle. */
	unsigned long owner;
	u64 vdev_id;
};

struct guest_memfd {
	struct kvm *kvm;
	u64 size;
	u64 flags;
};

/**
 * struct guest_memfd_provider_operations - External memory provider operations
 * @attach: Validate the resource and attach it to the core context. Return
 *          an opaque provider or ERR_PTR(). Success pins backing resources
 *          and these operations until release().
 * @release: Release the provider context and backing references. Must not
 *           access the VM or invoke callbacks that do; the file may be gone.
 * @alloc_folio: Allocate an uninserted folio for an index and NUMA policy.
 * @invalidate_folio: Notify the folio provider before releasing a folio.
 * @bind: Validate a PFN interval and return its protected mapping owner.
 * @get_pfn: Resolve a shared resource offset without taking a folio reference.
 *
 * Provider operations supply exactly one of alloc_folio() and get_pfn().
 * PFN providers start shared and require the bind and conversion callbacks
 * supported by the core. Folio providers use core allocation and conversion.
 */
struct guest_memfd_provider_operations {
	void *(*attach)(struct file *resource, struct guest_memfd *gmem);
	void (*release)(void *provider);
	struct folio *(*alloc_folio)(void *provider, pgoff_t index,
				     struct mempolicy *mpol);
	void (*invalidate_folio)(void *provider, struct folio *folio);
	int (*bind)(void *provider, u64 offset, u64 size,
		struct guest_memfd_device_binding *binding);
	int (*get_pfn)(void *provider, u64 offset, unsigned long *pfn);
};

#endif /* _LINUX_GUEST_MEMFD_H */
