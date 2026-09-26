// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023-2026 ARM Ltd.
 */

#include <uapi/linux/psci.h>
#include <linux/kvm_host.h>
#include <linux/arm-rmi-cmds.h>
#include <linux/overflow.h>

#include <asm/daifflags.h>
#include <asm/kvm_emulate.h>
#include <asm/kvm_hyp.h>
#include <asm/kvm_mmu.h>
#include <asm/kvm_pgtable.h>
#include <asm/virt.h>

struct realm_dev_mapping {
	struct list_head node;
	unsigned long vdev_phys;
	unsigned long start;
	unsigned long end;
};

static inline unsigned long rmi_rtt_level_mapsize(int level)
{
	if (WARN_ON(level > KVM_PGTABLE_LAST_LEVEL))
		return PAGE_SIZE;

	return (1UL << ARM64_HW_PGTABLE_LEVEL_SHIFT(level));
}

static bool rmi_has_feature(int reg, unsigned long feature)
{
	return !!u64_get_bits(rmi_feat_reg(reg), feature);
}

bool kvm_rmi_supports_sve(void)
{
	return rmi_has_feature(0, RMI_FEATURE_REGISTER_0_SVE);
}

u32 kvm_rmm_ipa_limit(void)
{
	return u64_get_bits(rmi_feat_reg(0), RMI_FEATURE_REGISTER_0_S2SZ);
}

unsigned int kvm_realm_sve_max_vl(void)
{
	return sve_vl_from_vq(u64_get_bits(rmi_feat_reg(0),
					   RMI_FEATURE_REGISTER_0_SVE_VL) + 1);
}

u64 kvm_realm_reset_id_aa64dfr0_el1(const struct kvm_vcpu *vcpu, u64 val)
{
	u32 bps = u64_get_bits(rmi_feat_reg(0), RMI_FEATURE_REGISTER_0_NUM_BPS);
	u32 wps = u64_get_bits(rmi_feat_reg(0), RMI_FEATURE_REGISTER_0_NUM_WPS);
	u32 ctx_cmps;

	/* Ensure CTX_CMPs is still valid */
	ctx_cmps = FIELD_GET(ID_AA64DFR0_EL1_CTX_CMPs, val);
	ctx_cmps = min(bps, ctx_cmps);

	val &= ~(ID_AA64DFR0_EL1_BRPs_MASK | ID_AA64DFR0_EL1_WRPs_MASK |
		 ID_AA64DFR0_EL1_CTX_CMPs);
	val |= FIELD_PREP(ID_AA64DFR0_EL1_BRPs_MASK, bps) |
	       FIELD_PREP(ID_AA64DFR0_EL1_WRPs_MASK, wps) |
	       FIELD_PREP(ID_AA64DFR0_EL1_CTX_CMPs, ctx_cmps);

	return val;
}

static int get_start_level(struct realm *realm)
{
	return 4 - stage2_pgtable_levels(realm->ia_bits);
}

static int find_map_level(struct realm *realm,
			  unsigned long start,
			  unsigned long end)
{
	int level = KVM_PGTABLE_LAST_LEVEL;

	while (level > get_start_level(realm)) {
		unsigned long map_size = rmi_rtt_level_mapsize(level - 1);

		if (!IS_ALIGNED(start, map_size) ||
		    (start + map_size) > end)
			break;

		level--;
	}

	return level;
}

static unsigned long rmi_range_entry_size(int rmi_size)
{
	if (WARN_ON(rmi_size > KVM_PGTABLE_LAST_LEVEL))
		return 0;

	return kvm_granule_size(KVM_PGTABLE_LAST_LEVEL - rmi_size);
}

static int undelegate_range_desc(unsigned long desc)
{
	unsigned long size = rmi_range_entry_size(RMI_ADDR_RANGE_BLOCK_SIZE(desc));
	unsigned long count = RMI_ADDR_RANGE_COUNT(desc);
	unsigned long addr = RMI_ADDR_RANGE_ADDR(desc);
	unsigned long state = RMI_ADDR_RANGE_STATE(desc);

	if (state == RMI_OP_MEM_UNDELEGATED)
		return 0;

	if (size * count == 0)
		return 0;

	return rmi_undelegate_range(addr, size * count);
}

static phys_addr_t alloc_delegated_granule(struct kvm_mmu_memory_cache *mc)
{
	phys_addr_t phys;
	void *virt;

	if (mc) {
		virt = kvm_mmu_memory_cache_alloc(mc);
	} else {
		virt = (void *)__get_free_page(GFP_ATOMIC | __GFP_ZERO |
					       __GFP_ACCOUNT);
	}

	if (!virt)
		return PHYS_ADDR_MAX;

	phys = virt_to_phys(virt);
	if (rmi_delegate_page(phys)) {
		free_page((unsigned long)virt);
		return PHYS_ADDR_MAX;
	}

	return phys;
}

static phys_addr_t alloc_rtt(struct kvm_mmu_memory_cache *mc)
{
	phys_addr_t phys = alloc_delegated_granule(mc);

	if (phys != PHYS_ADDR_MAX)
		kvm_account_pgtable_pages(phys_to_virt(phys), 1);

	return phys;
}

static void free_rtt(phys_addr_t phys)
{
	if (rmi_free_delegated_page(phys))
		return;

	kvm_account_pgtable_pages(phys_to_virt(phys), -1);
}

int realm_psci_complete(struct kvm_vcpu *source, unsigned long status)
{
	long ret;

	ret = rmi_psci_complete(source->arch.rec.rec_phys, status);
	if (ret)
		return -ENXIO;

	return 0;
}

static long realm_rtt_create(struct realm *realm,
			     unsigned long addr,
			     int level,
			     phys_addr_t phys)
{
	addr = ALIGN_DOWN(addr, rmi_rtt_level_mapsize(level - 1));
	return rmi_rtt_create(virt_to_phys(realm->rd), phys, addr, level);
}

static long realm_rtt_fold(struct realm *realm,
			   unsigned long addr,
			   int level,
			   phys_addr_t *rtt_granule)
{
	unsigned long out_rtt;
	long ret;

	addr = ALIGN_DOWN(addr, rmi_rtt_level_mapsize(level - 1));
	ret = rmi_rtt_fold(virt_to_phys(realm->rd), addr, level, &out_rtt);

	if (rtt_granule)
		*rtt_granule = out_rtt;

	return ret;
}

/*
 * realm_rtt_destroy - Destroy an RTT at @level for @addr.
 *
 * Returns - Result of the RMI_RTT_DESTROY call, and:
 * @rtt_granule:	RTT granule, if the RTT was destroyed.
 * @next_addr:		IPA corresponding to the next possible valid entry we
 *			can target
 */
static long realm_rtt_destroy(struct realm *realm, unsigned long addr,
			      int level, phys_addr_t *rtt_granule,
			      unsigned long *next_addr)
{
	unsigned long out_rtt;
	long ret;

	ret = rmi_rtt_destroy(virt_to_phys(realm->rd), addr, level,
			      &out_rtt, next_addr);

	*rtt_granule = out_rtt;

	return ret;
}

static int realm_create_rtt_levels(struct realm *realm,
				   unsigned long ipa,
				   int level,
				   int max_level,
				   struct kvm_mmu_memory_cache *mc)
{
	while (level++ < max_level) {
		phys_addr_t rtt = alloc_rtt(mc);
		long ret;

		if (rtt == PHYS_ADDR_MAX)
			return -ENOMEM;

		ret = realm_rtt_create(realm, ipa, level, rtt);
		if (ret < 0) {
			free_rtt(rtt);
			return ret;
		}

		if (RMI_RESULT_STATUS(ret) == RMI_ERROR_RTT &&
		    RMI_RESULT_DATA_LEVEL(ret) == level - 1) {
			/* The RTT already exists, continue */
			free_rtt(rtt);
			continue;
		}

		if (ret) {
			WARN(1, "Failed to create RTT at level %d: %ld\n",
			     level, ret);
			free_rtt(rtt);
			return -ENXIO;
		}
	}

	return 0;
}

static int realm_tear_down_rtt_level(struct realm *realm, int level,
				     unsigned long start, unsigned long end)
{
	ssize_t map_size;
	unsigned long addr, next_addr;

	if (WARN_ON(level > KVM_PGTABLE_LAST_LEVEL))
		return -EINVAL;

	map_size = rmi_rtt_level_mapsize(level - 1);

	for (addr = start; addr < end; addr = next_addr) {
		phys_addr_t rtt_granule;
		long ret;
		unsigned long align_addr = ALIGN(addr, map_size);

		next_addr = ALIGN(addr + 1, map_size);

		if (next_addr > end || align_addr != addr) {
			/*
			 * The target range is smaller than what this level
			 * covers, recurse deeper.
			 */
			ret = realm_tear_down_rtt_level(realm,
							level + 1,
							addr,
							min(next_addr, end));
			if (ret)
				return ret;
			continue;
		}

		ret = realm_rtt_destroy(realm, addr, level,
					&rtt_granule, &next_addr);
		if (ret < 0)
			return ret;

		switch (RMI_RESULT_STATUS(ret)) {
		case RMI_SUCCESS:
			free_rtt(rtt_granule);
			break;
		case RMI_ERROR_RTT:
			if (next_addr > addr) {
				/* Missing RTT, skip */
				break;
			}
			/*
			 * We tear down the RTT range for the full IPA
			 * space, after everything is unmapped. Also we
			 * descend down only if we cannot tear down a
			 * top level RTT. Thus RMM must be able to walk
			 * to the requested level. e.g., a block mapping
			 * exists at L1 or L2.
			 */
			if (WARN_ON(RMI_RESULT_DATA_LEVEL(ret) != level))
				return -EBUSY;
			if (WARN_ON(level == KVM_PGTABLE_LAST_LEVEL))
				return -EBUSY;

			/*
			 * The table has active entries in it, recurse deeper
			 * and tear down the RTTs.
			 */
			next_addr = ALIGN(addr + 1, map_size);
			ret = realm_tear_down_rtt_level(realm,
							level + 1,
							addr,
							next_addr);
			if (ret)
				return ret;
			/*
			 * Now that the child RTTs are destroyed,
			 * retry at this level.
			 */
			next_addr = addr;
			break;
		default:
			WARN_ON(1);
			return -ENXIO;
		}
	}

	return 0;
}

static int realm_tear_down_rtt_range(struct realm *realm,
				     unsigned long start, unsigned long end)
{
	/*
	 * Root level RTTs can only be destroyed after the RD is destroyed. So
	 * tear down everything below the root level
	 */
	return realm_tear_down_rtt_level(realm, get_start_level(realm) + 1,
					 start, end);
}

/*
 * Returns 0 on successful fold, a negative value on error, a positive value if
 * we were not able to fold all tables at this level.
 */
static int realm_fold_rtt_level(struct realm *realm, int level,
				unsigned long start, unsigned long end)
{
	int not_folded = 0;
	ssize_t map_size;
	unsigned long addr, next_addr;

	if (WARN_ON(level > KVM_PGTABLE_LAST_LEVEL))
		return -EINVAL;

	map_size = rmi_rtt_level_mapsize(level - 1);

	for (addr = start; addr < end; addr = next_addr) {
		phys_addr_t rtt_granule;
		long ret;
		unsigned long align_addr = ALIGN(addr, map_size);

		next_addr = ALIGN(addr + 1, map_size);

		ret = realm_rtt_fold(realm, align_addr, level, &rtt_granule);
		if (ret < 0)
			return ret;

		switch (RMI_RESULT_STATUS(ret)) {
		case RMI_SUCCESS:
			free_rtt(rtt_granule);
			break;
		case RMI_ERROR_RTT:
			if (level == KVM_PGTABLE_LAST_LEVEL ||
			    RMI_RESULT_DATA_LEVEL(ret) < level) {
				not_folded++;
				break;
			}
			/* Recurse a level deeper */
			ret = realm_fold_rtt_level(realm,
						   level + 1,
						   addr,
						   next_addr);
			if (ret < 0) {
				return ret;
			} else if (ret == 0) {
				/* Try again at this level */
				next_addr = addr;
			}
			break;
		default:
			WARN_ON(1);
			return -ENXIO;
		}
	}

	return not_folded;
}

static int realm_destroy_rtts(struct kvm *kvm)
{
	struct realm *realm = &kvm->arch.realm;
	unsigned int ia_bits = realm->ia_bits;

	lockdep_assert_held(&kvm->arch.config_lock);

	return realm_tear_down_rtt_range(realm, 0, (1UL << ia_bits));
}

static void realm_unmap_stage2(struct kvm *kvm)
{
	struct realm *realm = &kvm->arch.realm;

	lockdep_assert_held(&kvm->arch.config_lock);

	if (realm->stage2_unmapped)
		return;

	write_lock(&kvm->mmu_lock);
	kvm_stage2_unmap_range(&kvm->arch.mmu, 0,
			       BIT(realm->ia_bits - 1), true);
	write_unlock(&kvm->mmu_lock);

	realm->stage2_unmapped = true;
}

int kvm_realm_teardown_stage2(struct kvm *kvm)
{
	lockdep_assert_held(&kvm->arch.config_lock);

	realm_unmap_stage2(kvm);
	return 0;
}

static void realm_unmap_shared_range(struct kvm *kvm,
				     unsigned long start,
				     unsigned long end,
				     bool may_block)
{
	struct realm *realm = &kvm->arch.realm;
	unsigned long rd = virt_to_phys(realm->rd);
	unsigned long next_addr, addr;
	unsigned long shared_bit = BIT(realm->ia_bits - 1);

	if (start >= end)
		return;

	/*
	 * Callers may pass either an IPA range or its shared alias. Add the
	 * shared bit only for the former. Use addition so an exclusive end at
	 * shared_bit advances to the end of the IPA space.
	 */
	if (start < shared_bit) {
		start += shared_bit;
		end += shared_bit;
	}

	for (addr = start; addr < end; addr = next_addr) {
		long ret;

		ret = rmi_rtt_unprot_unmap(rd, addr, end, RMI_ADDR_TYPE_NONE,
					   0, &next_addr, NULL, NULL);
		if (WARN_ON(ret < 0))
			return;

		switch (RMI_RESULT_STATUS(ret)) {
		case RMI_SUCCESS:
			break;
		case RMI_ERROR_RTT: {
			int err_level = RMI_RESULT_DATA_LEVEL(ret);
			int level = find_map_level(realm, addr, end);

			if (err_level >= level) {
				/* Nothing present, so skip */
				next_addr = addr + rmi_rtt_level_mapsize(err_level);
				break;
			}

			ret = realm_create_rtt_levels(realm, addr, err_level,
						      level, NULL);
			if (WARN_ON(ret))
				return;
			/* Retry with the RTT levels in place */
			next_addr = addr;
			break;
		}
		default:
			WARN_ON(1);
			return;
		}

		if (may_block)
			cond_resched_rwlock_write(&kvm->mmu_lock);
	}

	realm_fold_rtt_level(realm, get_start_level(realm) + 1,
			     start, end);
}

static int realm_init_sve_param(struct kvm *kvm, struct realm_params *params)
{
	unsigned long i;
	struct kvm_vcpu *vcpu;
	int vl, last_vl = -1;

	if (!kvm_has_sve(kvm))
		return 0;

	/*
	 * Get the preferred SVE configuration, set by userspace with the
	 * KVM_ARM_VCPU_SVE feature and KVM_REG_ARM64_SVE_VLS pseudo-register.
	 */
	kvm_for_each_vcpu(i, vcpu, kvm) {
		if (!kvm_arm_vcpu_sve_finalized(vcpu))
			return -EINVAL;

		vl = vcpu->arch.sve_max_vl;

		/* We need all vCPUs to have the same SVE config */
		if (last_vl >= 0 && last_vl != vl)
			return -EINVAL;

		last_vl = vl;
	}

	if (last_vl > 0) {
		params->sve_vl = sve_vq_from_vl(last_vl) - 1;
		params->flags0 |= RMI_REALM_PARAM_FLAG_SVE;
	}
	return 0;
}

static int realm_create_rd(struct kvm *kvm)
{
	struct realm *realm = &kvm->arch.realm;
	struct realm_params *params __free(free_page) = NULL;
	void *rd = NULL;
	phys_addr_t rd_phys, params_phys, top_delegated;
	size_t pgd_size = kvm_pgtable_stage2_pgd_size(kvm->arch.mmu.vtcr);
	u64 dfr0 = kvm_read_vm_id_reg(kvm, SYS_ID_AA64DFR0_EL1);
	long rmi_ret;
	int r;

	realm->ia_bits = VTCR_EL2_IPA(kvm->arch.mmu.vtcr);

	if (WARN_ON(realm->rd))
		return -EEXIST;

	params = (void *)get_zeroed_page(GFP_KERNEL_ACCOUNT);
	if (!params)
		return -ENOMEM;

	rd = (void *)__get_free_page(GFP_KERNEL_ACCOUNT);
	if (!rd)
		return -ENOMEM;

	rd_phys = virt_to_phys(rd);
	if (rmi_delegate_page(rd_phys)) {
		r = -ENXIO;
		goto free_rd;
	}

	if (rmi_delegate_range(kvm->arch.mmu.pgd_phys, pgd_size,
			       &top_delegated)) {
		r = -ENXIO;
		goto out_undelegate_tables;
	}

	params->s2sz = VTCR_EL2_IPA(kvm->arch.mmu.vtcr);
	params->rtt_level_start = get_start_level(realm);
	params->rtt_num_start = pgd_size / PAGE_SIZE;
	params->rtt_base = kvm->arch.mmu.pgd_phys;
	params->num_bps = SYS_FIELD_GET(ID_AA64DFR0_EL1, BRPs, dfr0);
	params->num_wps = SYS_FIELD_GET(ID_AA64DFR0_EL1, WRPs, dfr0);
	params->hash_algo = RMI_HASH_SHA_256;

	if (kvm->arch.arm_pmu) {
		params->pmu_num_ctrs = kvm->arch.nr_pmu_counters;
		params->flags0 |= RMI_REALM_PARAM_FLAG_PMU;
	}

	r = realm_init_sve_param(kvm, params);
	if (r)
		goto out_undelegate_tables;

	/* For now default enable DA */
	if (rmm_has_reg2_feature(RMI_FEATURE_REGISTER_2_DA))
		params->flags0 |= RMI_REALM_PARAM_FLAG_DA;
	params_phys = virt_to_phys(params);

	rmi_ret = rmi_realm_create(rd_phys, params_phys, realm->sro);
	if (rmi_ret) {
		r = rmi_ret < 0 ? rmi_ret : -ENXIO;
		goto out_undelegate_tables;
	}

	realm->rd = rd;
	kvm_set_realm_state(kvm, REALM_STATE_NEW);

	return 0;

out_undelegate_tables:
	if (WARN_ON(rmi_undelegate_range(kvm->arch.mmu.pgd_phys,
					 top_delegated - kvm->arch.mmu.pgd_phys))) {
		/* Leak the pages if they cannot be returned */
		kvm->arch.mmu.pgt = NULL;
	}
	if (WARN_ON(rmi_undelegate_page(rd_phys))) {
		/* Leak the page if it isn't returned */
		return r;
	}
free_rd:
	free_page((unsigned long)rd);
	return r;
}

static int rmi_rtt_dev_unmap(unsigned long rd_phys,
		unsigned long base, unsigned long top,
		unsigned long *out_ipa, unsigned long *out_desc,
		unsigned long *rmi_ret);
static int realm_get_ripas(struct kvm *kvm, unsigned long addr,
			   enum rmi_ripas *ripas);
static int __realm_dev_mem_unmap(struct kvm *kvm, unsigned long start,
				 unsigned long end, bool may_block);

static int realm_unmap_private_range(struct kvm *kvm,
				     unsigned long start,
				     unsigned long end,
				     bool may_block,
				     enum rmi_ripas src_ripas)
{
	struct realm *realm = &kvm->arch.realm;
	unsigned long rd = virt_to_phys(realm->rd);
	unsigned long next_addr, addr;
	long ret;

	/* Called with mmu_lock held, so RTT entry can't change. */
	lockdep_assert_held_write(&kvm->mmu_lock);

	if (src_ripas == RMI_DEV)
		return __realm_dev_mem_unmap(kvm, start, end, may_block);

	for (addr = start; addr < end; addr = next_addr) {
		unsigned long out_range;
		unsigned long flags = RMI_ADDR_TYPE_SINGLE;
		/* TODO: Optimise using RMI_ADDR_TYPE_LIST */

retry:
		ret = rmi_rtt_data_unmap(rd, addr, end, flags, 0,
					 &next_addr, &out_range, NULL);
		if (WARN_ON(ret < 0))
			return ret;

		if (RMI_RESULT_STATUS(ret) == RMI_ERROR_RTT) {
			phys_addr_t rtt;

			if (next_addr > addr)
				continue; /* UNASSIGNED */

			rtt = alloc_rtt(NULL);
			if (WARN_ON(rtt == PHYS_ADDR_MAX))
				return -ENOMEM;
			ret = realm_rtt_create(realm, addr,
					       RMI_RESULT_DATA_LEVEL(ret) + 1, rtt);
			if (WARN_ON(ret)) {
				free_rtt(rtt);
				return ret;
			}
			goto retry;
		} else if (WARN_ON(ret)) {
			continue;
		}

		ret = undelegate_range_desc(out_range);
		if (WARN_ON(ret))
			break;

		//FIXME!! where are we freeing the private page?
		if (may_block)
			cond_resched_rwlock_write(&kvm->mmu_lock);
	}

	realm_fold_rtt_level(realm, get_start_level(realm) + 1,
			     start, end);

	return 0;
}

void kvm_realm_unmap_range_filter(struct kvm *kvm, unsigned long start,
				  unsigned long size, bool may_block,
				  enum kvm_gfn_range_filter attr_filter)
{
	unsigned long end = start + size;
	struct realm *realm = &kvm->arch.realm;
	enum rmi_ripas src_ripas;

	if (!kvm_realm_is_created(kvm))
		return;

	end = min(BIT(realm->ia_bits - 1), end);

	if (attr_filter & KVM_FILTER_SHARED)
		realm_unmap_shared_range(kvm, start, end, may_block);
	if (attr_filter & KVM_FILTER_PRIVATE) {
		/* An unmap request won't mix different RIPAS ranges. */
		if (realm_get_ripas(kvm, start, &src_ripas))
			return;
		realm_unmap_private_range(kvm, start, end, may_block,
					  src_ripas);
	}
}

void kvm_realm_unmap_range(struct kvm *kvm, unsigned long start,
			   unsigned long size, bool may_block)
{
	kvm_realm_unmap_range_filter(kvm, start, size, may_block,
				     KVM_FILTER_SHARED | KVM_FILTER_PRIVATE);
}

static int realm_data_map_init(struct kvm *kvm, unsigned long ipa,
			       kvm_pfn_t dst_pfn, kvm_pfn_t src_pfn,
			       unsigned long flags)
{
	struct realm *realm = &kvm->arch.realm;
	phys_addr_t rd = virt_to_phys(realm->rd);
	phys_addr_t dst_phys, src_phys;
	long ret;

	lockdep_assert_held(&kvm->slots_lock);
	lockdep_assert_held(&kvm->arch.config_lock);

	dst_phys = __pfn_to_phys(dst_pfn);
	src_phys = __pfn_to_phys(src_pfn);

	if (rmi_delegate_page(dst_phys))
		return -ENXIO;

retry:
	ret = rmi_rtt_data_map_init(rd, dst_phys, ipa, src_phys, flags);
	if (ret >= 0 && RMI_RESULT_STATUS(ret) == RMI_ERROR_RTT) {
		/* Create missing RTTs and retry */
		int level = RMI_RESULT_DATA_LEVEL(ret);

		if (KVM_BUG_ON(level >= KVM_PGTABLE_LAST_LEVEL, kvm)) {
			ret = -EIO;
			goto out_undelegate;
		}

		ret = realm_create_rtt_levels(realm, ipa, level,
					      level + 1, NULL);
		if (!ret)
			goto retry;
	}

out_undelegate:
	if (ret)
		KVM_BUG_ON(rmi_undelegate_page(dst_phys), kvm);

	return ret <= 0 ? ret : -ENXIO;
}

static unsigned long addr_range_desc(unsigned long phys, unsigned long size,
				     unsigned long state)
{
	unsigned long count = 1;
	unsigned long out;
	int rmi_size;

	for (rmi_size = KVM_PGTABLE_LAST_LEVEL; rmi_size >= 0; rmi_size--) {
		unsigned long entry_size = rmi_range_entry_size(rmi_size);

		if (size == entry_size)
			break;
		if (size > entry_size) {
			rmi_size = -1;
			break;
		}
	}

	if (rmi_size < 0) {
		/*
		 * Only support mapping at the page level granularity when
		 * it's an unusual length. This should get us back onto a larger
		 * block size for the subsequent mappings.
		 */
		rmi_size = 0;
		count = MIN(size >> PAGE_SHIFT, PTRS_PER_PTE - 1);
	}

	WARN_ON(phys & ~PAGE_MASK);

	out = FIELD_PREP(RMI_ADDR_RANGE_BLOCK_SIZE_MASK, rmi_size) |
	      FIELD_PREP(RMI_ADDR_RANGE_COUNT_MASK, count) |
	      FIELD_PREP(RMI_ADDR_RANGE_STATE_MASK, state);
	out |= phys & PAGE_MASK;

	return out;
}

static int realm_map_protected(struct kvm *kvm,
			       unsigned long ipa,
			       kvm_pfn_t pfn,
			       unsigned long map_size,
			       struct kvm_mmu_memory_cache *memcache)
{
	struct realm *realm = &kvm->arch.realm;
	phys_addr_t phys = __pfn_to_phys(pfn);
	phys_addr_t base_phys = phys;
	phys_addr_t rd = virt_to_phys(realm->rd);
	phys_addr_t delegated_phys;
	unsigned long base_ipa = ipa;
	unsigned long ipa_top;
	long ret = 0;

	if (WARN_ON(!IS_ALIGNED(map_size, PAGE_SIZE) ||
		    !IS_ALIGNED(ipa, map_size)))
		return -EINVAL;

	if (rmi_delegate_range(phys, map_size, &delegated_phys)) {
		if (delegated_phys == phys) {
			/*
			 * It's likely we raced with another VCPU on the same
			 * fault. Assume the other VCPU has handled the fault
			 * and return to the guest.
			 */
			return 0;
		}
		/* Partial delegation - map as much as we can */
		map_size = delegated_phys - phys;
	}

	ipa_top = ipa + map_size;

	while (ipa < ipa_top) {
		unsigned long flags = RMI_ADDR_TYPE_SINGLE;
		unsigned long range_desc = addr_range_desc(phys, ipa_top - ipa,
							   RMI_OP_MEM_DELEGATED);
		unsigned long out_top;

		ret = rmi_rtt_data_map(rd, ipa, ipa_top, flags, range_desc,
				       &out_top);
		if (ret < 0)
			goto err_undelegate;

		if (RMI_RESULT_STATUS(ret) == RMI_ERROR_RTT) {
			/* Create missing RTTs and retry */
			int level = RMI_RESULT_DATA_LEVEL(ret);

			if (WARN_ON(level >= KVM_PGTABLE_LAST_LEVEL))
				goto err_undelegate;
			ret = realm_create_rtt_levels(realm, ipa, level,
						      level + 1,
						      memcache);
			if (ret)
				goto err_undelegate;

			continue;
		}

		if (WARN_ON(ret))
			goto err_undelegate;

		phys += out_top - ipa;
		ipa = out_top;
	}

	return 0;

err_undelegate:
	realm_unmap_private_range(kvm, base_ipa, ipa, true, RMI_RAM);
	/* A range which cannot be undelegated remains owned by the RMM. */
	WARN_ON(rmi_undelegate_range(base_phys, map_size));
	return ret < 0 ? ret : -ENXIO;
}

static int realm_map_non_secure(struct kvm *kvm,
				unsigned long ipa,
				kvm_pfn_t pfn,
				unsigned long size,
				enum kvm_pgtable_prot prot,
				struct kvm_mmu_memory_cache *memcache)
{
	struct realm *realm = &kvm->arch.realm;
	unsigned long attr, flags = 0;
	phys_addr_t rd = virt_to_phys(realm->rd);
	phys_addr_t phys = __pfn_to_phys(pfn);
	unsigned long ipa_top = ipa + size;
	long ret;

	if (WARN_ON(!IS_ALIGNED(size, PAGE_SIZE)))
		return -EINVAL;

	switch (prot & (KVM_PGTABLE_PROT_DEVICE | KVM_PGTABLE_PROT_NORMAL_NC)) {
	case KVM_PGTABLE_PROT_DEVICE | KVM_PGTABLE_PROT_NORMAL_NC:
		return -EINVAL;
	case KVM_PGTABLE_PROT_DEVICE:
		attr = MT_S2_FWB_DEVICE_nGnRE;
		break;
	case KVM_PGTABLE_PROT_NORMAL_NC:
		attr = MT_S2_FWB_NORMAL_NC;
		break;
	default:
		attr = MT_S2_FWB_NORMAL;
	}

	flags |= FIELD_PREP(RMI_RTT_UNPROT_MAP_FLAGS_MEMATTR_MASK, attr);

	if (prot & KVM_PGTABLE_PROT_R)
		flags |= FIELD_PREP(RMI_RTT_UNPROT_MAP_FLAGS_S2AP_MASK, RMI_S2AP_DIRECT_READ);
	if (prot & KVM_PGTABLE_PROT_W)
		flags |= FIELD_PREP(RMI_RTT_UNPROT_MAP_FLAGS_S2AP_MASK, RMI_S2AP_DIRECT_WRITE);

	flags |= RMI_ADDR_TYPE_SINGLE;

	while (ipa < ipa_top) {
		unsigned long range_desc = addr_range_desc(phys, ipa_top - ipa,
							   RMI_OP_MEM_UNDELEGATED);
		unsigned long out_top;

		ret = rmi_rtt_unprot_map(rd, ipa, ipa_top, flags, range_desc,
					 &out_top);
		if (ret < 0)
			return ret;

		if (RMI_RESULT_STATUS(ret) == RMI_ERROR_RTT) {
			/* Create missing RTTs and retry */
			int level = RMI_RESULT_DATA_LEVEL(ret);
			int req_level = find_map_level(realm, ipa, ipa_top);

			/*
			 * There already exists a mapping at the level. May be
			 * we are relaxing a permission for the given range ?
			 */
			if (level >= req_level) {
				realm_unmap_shared_range(kvm, ipa, ipa_top, true);
				continue;
			}

			ret = realm_create_rtt_levels(realm, ipa, level,
						      req_level,
						      memcache);
			if (ret)
				return ret;

			ret = rmi_rtt_unprot_map(rd, ipa, ipa_top, flags,
						 range_desc, &out_top);
			if (ret < 0)
				return ret;
		}

		if (ret && RMI_RESULT_STATUS(ret) != RMI_ERROR_RTT)
			return ret;

		phys += out_top - ipa;
		ipa = out_top;
	}

	return 0;
}

int realm_map_ipa(struct kvm *kvm, phys_addr_t ipa,
		  kvm_pfn_t pfn, unsigned long map_size,
		  enum kvm_pgtable_prot prot,
		  struct kvm_mmu_memory_cache *memcache)
{
	struct realm *realm = &kvm->arch.realm;

	if (kvm_realm_state(kvm) != REALM_STATE_ACTIVE)
		return -EBUSY;

	ipa = ALIGN_DOWN(ipa, map_size);
	if (!kvm_realm_is_private_address(realm, ipa)) {
		return realm_map_non_secure(kvm, ipa, pfn, map_size, prot,
					    memcache);
	}

	/* It's impossible to map protected pages read-only. */
	if (WARN_ON(!(prot & KVM_PGTABLE_PROT_W)))
		return -EFAULT;
	return realm_map_protected(kvm, ipa, pfn, map_size, memcache);
}

static int populate_region_cb(struct kvm *kvm, gfn_t gfn, kvm_pfn_t pfn,
			      struct page *src_page, void *opaque)
{
	unsigned long data_flags = *(unsigned long *)opaque;
	phys_addr_t ipa = gfn_to_gpa(gfn);

	return realm_data_map_init(kvm, ipa, pfn, page_to_pfn(src_page),
				   data_flags);
}

enum ripas_action {
	RIPAS_INIT,
	RIPAS_SET,
};

static int ripas_change(struct kvm *kvm,
			struct kvm_vcpu *vcpu,
			unsigned long ipa,
			unsigned long end,
			enum ripas_action action,
			unsigned long *top_ipa)
{
	struct realm *realm = &kvm->arch.realm;
	phys_addr_t rd_phys = virt_to_phys(realm->rd);
	phys_addr_t rec_phys;
	struct kvm_mmu_memory_cache *memcache = NULL;
	long ret = 0;

	if (vcpu) {
		rec_phys = vcpu->arch.rec.rec_phys;
		memcache = &vcpu->arch.mmu_page_cache;

		WARN_ON(action != RIPAS_SET);
	} else {
		WARN_ON(action != RIPAS_INIT);
	}

	while (ipa < end) {
		unsigned long next = ~0;

		switch (action) {
		case RIPAS_INIT:
			ret = rmi_rtt_init_ripas(rd_phys, ipa, end, &next);
			break;
		case RIPAS_SET:
			ret = rmi_rtt_set_ripas(rd_phys, rec_phys, ipa, end,
						&next);
			break;
		}

		if (ret < 0)
			goto out;

		switch (RMI_RESULT_STATUS(ret)) {
		case RMI_SUCCESS:
			ipa = next;
			break;
		case RMI_ERROR_RTT: {
			int err_level = RMI_RESULT_DATA_LEVEL(ret);
			int level = find_map_level(realm, ipa, end);

			/*
			 * If the operation failed at deeper level than
			 * what is required for the address range, this
			 * implies encountering an unexpected entry,
			 * (e.g., RIPAS_DESTROYED), which the RMM prevents
			 * us from modifying. This is only applicable for
			 * RMI_RTT_INIT_RIPAS. All the other requests
			 * are generated by the Realm and thus RMM should
			 * be able to allow the transition.
			 */
			if (action == RIPAS_INIT && WARN_ON_ONCE(err_level >= level))
				return -ENXIO;

			ret = realm_create_rtt_levels(realm, ipa, err_level,
						      level, memcache);
			if (ret)
				goto out;
			/* Retry with the RTT levels in place */
			break;
		}
		default:
			WARN_ON(1);
			ret = -ENXIO;
			goto out;
		}
	}

out:
	if (top_ipa)
		*top_ipa = ipa;

	return ret;
}

static int realm_set_ipa_state(struct kvm_vcpu *vcpu,
			       unsigned long start,
			       unsigned long end,
			       enum rmi_ripas src_ripas,
			       enum rmi_ripas target_ripas,
			       unsigned long *top_ipa)
{
	struct kvm *kvm = vcpu->kvm;
	int ret;

	/*
	 * We use the RIPAS value to decide between a data_destroy or a
	 * dev_mem_unmap. Hence call realm_unmap_private_range() before
	 * ripas_change().
	 *
	 * Technically, for private RAM, we don't need to call
	 * realm_unmap_private_range(), because any RIPAS change via RSI would
	 * trigger a memory fault exit. That would, in turn, invalidate the
	 * guest's memfd range, which then triggers realm_unmap_private_range()
	 * automatically.
	 *
	 * However, this doesn’t apply to RIPAS_DEV, because we currently
	 * lack a user-space API to call realm_dev_mem_unmap() in response to a
	 * memory fault exit. Therefore, the unmap must happen explicitly before
	 * the RIPAS change.
	 */
	if (target_ripas == RMI_EMPTY) {
		ret = realm_unmap_private_range(kvm, start, end, false,
						src_ripas);
		if (ret)
			return ret;
	}
	ret = ripas_change(kvm, vcpu, start, end, RIPAS_SET, top_ipa);

	return ret;
}

static int realm_init_ipa_state(struct kvm *kvm,
				unsigned long gfn,
				unsigned long pages)
{
	return ripas_change(kvm, NULL, gfn_to_gpa(gfn), gfn_to_gpa(gfn + pages),
			    RIPAS_INIT, NULL);
}

static int realm_ensure_created(struct kvm *kvm)
{
	lockdep_assert_held(&kvm->arch.config_lock);

	switch (kvm_realm_state(kvm)) {
	case REALM_STATE_NONE:
		break;
	case REALM_STATE_NEW:
		return 0;
	case REALM_STATE_DEAD:
		return -ENXIO;
	default:
		return -EBUSY;
	}

	return realm_create_rd(kvm);
}

int kvm_realm_ensure_created(struct kvm *kvm)
{
	int ret;

	if (!kvm_vm_is_realm(kvm))
		return -EINVAL;

	guard(mutex)(&kvm->arch.config_lock);
	ret = realm_ensure_created(kvm);

	return ret;
}
EXPORT_SYMBOL_GPL(kvm_realm_ensure_created);

int kvm_arm_rmi_init_ripas(struct kvm *kvm,
			   struct kvm_arm_rmi_init_ripas *args)
{
	struct kvm_memory_slot *memslot;
	unsigned long mmu_seq;
	unsigned long start, end;
	gfn_t start_gfn, end_gfn;
	int ret;

	if (args->flags || memchr_inv(args->reserved, 0,
				      sizeof(args->reserved)) ||
	    !args->size || args->base + args->size < args->base ||
	    !PAGE_ALIGNED(args->base) || !PAGE_ALIGNED(args->size))
		return -EINVAL;

	start = args->base;
	end = start + args->size;
	start_gfn = gpa_to_gfn(start);
	end_gfn = gpa_to_gfn(end);

	mutex_lock(&kvm->slots_lock);
	mutex_lock(&kvm->arch.config_lock);

	ret = realm_ensure_created(kvm);
	if (ret)
		goto out_unlock;

	memslot = gfn_to_memslot(kvm, start_gfn);
	if (!kvm_slot_has_gmem(memslot) ||
	    end_gfn > memslot->base_gfn + memslot->npages) {
		ret = -EINVAL;
		goto out_unlock;
	}

	ret = kvm_gmem_set_attributes(memslot, start_gfn, end_gfn,
				      KVM_MEMORY_ATTRIBUTE_PRIVATE);
	if (ret)
		goto out_unlock;

	mmu_seq = kvm->mmu_invalidate_seq;
	smp_rmb();
	if (!kvm_gmem_range_has_attributes(memslot, start_gfn, end_gfn,
					   KVM_MEMORY_ATTRIBUTE_PRIVATE)) {
		ret = -EAGAIN;
		goto out_unlock;
	}

	write_lock(&kvm->mmu_lock);
	if (mmu_invalidate_retry(kvm, mmu_seq)) {
		write_unlock(&kvm->mmu_lock);
		ret = -EAGAIN;
		goto out_unlock;
	}
	ret = realm_init_ipa_state(kvm, start_gfn, end_gfn - start_gfn);
	write_unlock(&kvm->mmu_lock);

out_unlock:
	mutex_unlock(&kvm->arch.config_lock);
	mutex_unlock(&kvm->slots_lock);

	return ret;
}

int kvm_arm_rmi_populate(struct kvm *kvm,
			 struct kvm_arm_rmi_populate *args)
{
	unsigned long data_flags = 0;
	unsigned long ipa_start = args->base;
	unsigned long ipa_end = ipa_start + args->size;
	long pages_populated;
	int ret;

	if (args->reserved ||
	    (args->flags & ~KVM_ARM_RMI_POPULATE_FLAGS_MEASURE) ||
	    args->base + args->size < args->base ||
	    !IS_ALIGNED(ipa_start, PAGE_SIZE) ||
	    !IS_ALIGNED(ipa_end, PAGE_SIZE) ||
	    !IS_ALIGNED(args->source_uaddr, PAGE_SIZE) ||
	    !args->source_uaddr)
		return -EINVAL;

	if (args->flags & KVM_ARM_RMI_POPULATE_FLAGS_MEASURE)
		data_flags |= RMI_MEASURE_CONTENT;

	mutex_lock(&kvm->slots_lock);
	mutex_lock(&kvm->arch.config_lock);

	ret = realm_ensure_created(kvm);
	if (ret)
		goto out_unlock;

	if (range_overflows(args->base, args->size,
			    BIT_ULL(kvm->arch.realm.ia_bits - 1))) {
		ret = -EINVAL;
		goto out_unlock;
	}

	if (args->size == 0)
		goto out_unlock;

	pages_populated = kvm_gmem_populate(kvm, gpa_to_gfn(ipa_start),
					    u64_to_user_ptr(args->source_uaddr),
					    args->size >> PAGE_SHIFT,
					    false, populate_region_cb,
					    &data_flags);

	if (pages_populated < 0) {
		ret = pages_populated;
		goto out_unlock;
	}

	args->size -= pages_populated << PAGE_SHIFT;
	args->source_uaddr += pages_populated << PAGE_SHIFT;
	args->base += pages_populated << PAGE_SHIFT;

out_unlock:
	mutex_unlock(&kvm->arch.config_lock);
	mutex_unlock(&kvm->slots_lock);
	return ret;
}

static bool realm_ripas_attributes_match(struct kvm_vcpu *vcpu,
					 unsigned long start,
					 unsigned long end,
					 enum rmi_ripas ripas,
					 unsigned long *mmu_seq)
{
	struct kvm *kvm = vcpu->kvm;
	gfn_t gfn = gpa_to_gfn(start);
	gfn_t end_gfn = gpa_to_gfn(end);
	u64 attributes = ripas == RMI_RAM ? KVM_MEMORY_ATTRIBUTE_PRIVATE : 0;
	bool match = true;
	int idx;

	/* RIPAS_DEV is not backed by ordinary guest_memfd RAM. */
	if (ripas == RMI_DEV)
		return true;

	*mmu_seq = kvm->mmu_invalidate_seq;
	smp_rmb();
	idx = srcu_read_lock(&kvm->srcu);
	while (gfn < end_gfn) {
		struct kvm_memory_slot *slot = gfn_to_memslot(kvm, gfn);
		gfn_t slot_end;

		if (!slot || !kvm_slot_has_gmem(slot)) {
			match = false;
			break;
		}

		slot_end = min(end_gfn, slot->base_gfn + slot->npages);
		if (!kvm_gmem_range_has_attributes(slot, gfn, slot_end,
						  attributes)) {
			match = false;
			break;
		}
		gfn = slot_end;
	}
	srcu_read_unlock(&kvm->srcu, idx);

	return match;
}
static int realm_get_ripas(struct kvm *kvm, unsigned long addr,
			   enum rmi_ripas *ripas)
{
	struct realm *realm = &kvm->arch.realm;
	struct rtt_entry rtt_entry;
	unsigned long rd = virt_to_phys(realm->rd);

	lockdep_assert_held_write(&kvm->mmu_lock);

	if (rmi_rtt_read_entry(rd, addr, KVM_PGTABLE_LAST_LEVEL, &rtt_entry))
		return -ENOENT;

	*ripas = rtt_entry.ripas;
	return 0;
}

static int kvm_complete_ripas_change(struct kvm_vcpu *vcpu)
{
	struct kvm *kvm = vcpu->kvm;
	struct realm_rec *rec = &vcpu->arch.rec;
	unsigned long base = rec->run->exit.ripas_base;
	unsigned long top = rec->run->exit.ripas_top;
	enum rmi_ripas target_ripas = rec->run->exit.ripas_value;
	enum rmi_ripas src_ripas = RMI_EMPTY;
	unsigned long top_ipa = base;
	unsigned long mmu_seq = 0;
	bool attributes_match;
	bool dev_ripas_change;
	int ret;

	do {
		attributes_match = realm_ripas_attributes_match(vcpu, base, top,
								target_ripas, &mmu_seq);

		ret = kvm_mmu_topup_memory_cache(&vcpu->arch.mmu_page_cache,
						 kvm_mmu_cache_min_pages(vcpu->arch.hw_mmu));
		if (ret)
			return ret;
		write_lock(&kvm->mmu_lock);
		/*
		 * Device mappings are not backed by guest_memfd. Detect the
		 * source RIPAS while holding mmu_lock so DEV -> EMPTY can unmap
		 * the device mapping without requiring memory attributes.
		 */
		if (target_ripas == RMI_EMPTY) {
			ret = realm_get_ripas(kvm, base, &src_ripas);
			if (ret) {
				write_unlock(&kvm->mmu_lock);
				goto out_handle_error;
			}
		}

		if (target_ripas == RMI_DEV || src_ripas == RMI_DEV)
			dev_ripas_change = true;
		else
			dev_ripas_change = false;

		/* Device RIPAS transitions have no guest_memfd backing. */
		if (!dev_ripas_change) {
			if (!attributes_match) {
				write_unlock(&kvm->mmu_lock);
				kvm_prepare_memory_fault_exit(vcpu, base,
						      top - base, false, false,
						      target_ripas == RMI_RAM);
				kvm_make_request(KVM_REQ_RMI, vcpu);
				rec->run->exit.ripas_base = base;
				return -EFAULT;
			}
			if (mmu_invalidate_retry(kvm, mmu_seq)) {
				write_unlock(&kvm->mmu_lock);
				cond_resched();
				continue;
			}
		}
		ret = realm_set_ipa_state(vcpu, base, top, src_ripas,
					  target_ripas,
					  &top_ipa);
		write_unlock(&kvm->mmu_lock);

out_handle_error:
		if (ret == -ENOMEM) {
			if (top_ipa == base) {
				rec->run->exit.ripas_base = base;
				return ret;
			}
			base = top_ipa;
			continue;
		}

		if (ret) {
			WARN_RATELIMIT(1,
				       "Unable to satisfy RIPAS_CHANGE for %#lx - %#lx, ripas: %#x\n",
				       base, top, target_ripas);
			/* Reject a request that KVM cannot complete. */
			base = top_ipa;
			rec->run->enter.flags |= REC_ENTER_FLAG_RIPAS_RESPONSE;
			rec->run->exit.ripas_base = base;
			return 1;
		}

		base = top_ipa;
	} while (base < top);

	/*
	 * If this function is called again before the REC_ENTER call then
	 * avoid calling realm_set_ipa_state() again by changing to the value
	 * of ripas_base for the part that has already been covered. The RMM
	 * ignores the contains of the rec_exit structure so this doesn't
	 * affect the RMM.
	 */
	WARN_ON_ONCE(base != top);
	rec->run->exit.ripas_base = base;
	return 1;
}

static int kvm_rec_complete_psci(struct kvm_vcpu *vcpu)
{
	struct rec_run *run = vcpu->arch.rec.run;
	unsigned long status = PSCI_RET_DENIED;
	unsigned long ret = vcpu_get_reg(vcpu, 0);
	int r;

	switch (run->exit.gprs[0]) {
	case PSCI_0_2_FN64_CPU_ON: {
		if (ret != PSCI_RET_SUCCESS &&
		    ret != PSCI_RET_ALREADY_ON)
			status = PSCI_RET_DENIED;
		else
			status = PSCI_RET_SUCCESS;
		break;
	}
	default:
		return 1;
	}

	r = realm_psci_complete(vcpu, status);
	return r ?: 1;
}

static void noinstr kvm_rec_complete_sysreg_access(struct kvm_vcpu *vcpu)
{
	struct realm_rec *rec = &vcpu->arch.rec;
	u64 esr = kvm_vcpu_get_esr(vcpu);
	int rt;

	if (ESR_ELx_EC(esr) != ESR_ELx_EC_SYS64 ||
	    (esr & ESR_ELx_SYS64_ISS_DIR_MASK) != ESR_ELx_SYS64_ISS_DIR_READ)
		return;

	rt = kvm_vcpu_sys_get_rt(vcpu);
	if (rt < REC_RUN_GPRS)
		rec->run->enter.gprs[rt] = vcpu_get_reg(vcpu, rt);
}

static int rmi_rtt_dev_map(unsigned long rd_phys, unsigned long vdev_phys,
		unsigned long base, unsigned long top, unsigned long flags,
		unsigned long range_desc, unsigned long *out_top, unsigned long *rmi_ret)
{
	struct rmi_sro_state *sro __free(kfree) = kmalloc_obj(*sro);
	if (!sro)
		return -ENOMEM;

	*rmi_ret = rmi_sro_memxfer_cmd(sro, GFP_KERNEL, SMC_RMI_RTT_DEV_MAP,
				       rd_phys, vdev_phys, base, top, flags, range_desc);
	if (*rmi_ret)
		return 0;

	*out_top = sro->regs.a1;

	return 0;
}

static int rmi_rtt_dev_unmap(unsigned long rd_phys,
		unsigned long base, unsigned long top,
		unsigned long *out_ipa, unsigned long *out_desc,
		unsigned long *rmi_ret)
{
	unsigned long flags = RMI_ADDR_TYPE_SINGLE;
	struct rmi_sro_state *sro __free(kfree) = kmalloc_obj(*sro);
	if (!sro)
		return -ENOMEM;

	pr_info(" Request to unmap dev 0x%lx - 0x%lx\n", base, top);
	*rmi_ret = rmi_sro_memxfer_cmd(sro, GFP_KERNEL, SMC_RMI_RTT_DEV_UNMAP,
				       rd_phys, base, top, flags, 0);
	*out_ipa = sro->regs.a1;
	if (*rmi_ret)
		return 0;

	*out_desc = sro->regs.a2;

	return 0;
}

static bool realm_dev_mapping_overlaps(struct realm *realm,
				       unsigned long start,
				       unsigned long end)
{
	struct realm_dev_mapping *mapping;

	list_for_each_entry(mapping, &realm->dev_mappings, node)
		if (mapping->start < end && start < mapping->end)
			return true;

	return false;
}

static void realm_dev_mappings_free(struct realm *realm)
{
	struct realm_dev_mapping *mapping, *tmp;

	list_for_each_entry_safe(mapping, tmp, &realm->dev_mappings, node) {
		list_del(&mapping->node);
		kfree(mapping);
	}
}

static bool realm_dev_mapping_needs_split(struct realm *realm,
					  unsigned long ipa)
{
	struct realm_dev_mapping *mapping;

	list_for_each_entry(mapping, &realm->dev_mappings, node) {
		if (mapping->start < ipa && ipa < mapping->end)
			return true;
	}

	return false;
}

static void realm_dev_mapping_remove(struct realm *realm,
				     unsigned long start,
				     unsigned long end,
				     struct realm_dev_mapping *split)
{
	struct realm_dev_mapping *mapping, *tmp;

	list_for_each_entry_safe(mapping, tmp, &realm->dev_mappings, node) {
		if (mapping->end <= start || mapping->start >= end)
			continue;

		if (start <= mapping->start && end >= mapping->end) {
			list_del(&mapping->node);
			kfree(mapping);
		} else if (start <= mapping->start) {
			mapping->start = end;
		} else if (end >= mapping->end) {
			mapping->end = start;
		} else {
			if (WARN_ON(!split))
				continue;
			split->vdev_phys = mapping->vdev_phys;
			split->start = end;
			split->end = mapping->end;
			mapping->end = start;
			list_add(&split->node, &mapping->node);
			split = NULL;
		}
	}

	kfree(split);
}

static int __realm_dev_mem_unmap(struct kvm *kvm, unsigned long start,
				 unsigned long end, bool may_block)
{
	struct realm *realm = &kvm->arch.realm;
	struct realm_dev_mapping *split = NULL;
	phys_addr_t rd_phys = virt_to_phys(realm->rd);
	unsigned long ipa = start;
	int ret = 0;

	lockdep_assert_held_write(&kvm->mmu_lock);

	while (ipa < end) {
		unsigned long next_ipa;
		unsigned long range_desc;
		unsigned long rmi_ret;

		split = NULL;
		if (realm_dev_mapping_needs_split(realm, ipa)) {
			split = kmalloc_obj(*split, GFP_ATOMIC);
			if (!split) {
				ret = -ENOMEM;
				break;
			}
		}

retry:
		ret = rmi_rtt_dev_unmap(rd_phys, ipa, end, &next_ipa,
					&range_desc, &rmi_ret);
		if (ret)
			goto out_free_split;

		if (RMI_RESULT_STATUS(rmi_ret) == RMI_ERROR_RTT) {
			phys_addr_t rtt;

			if (next_ipa > ipa) {
				realm_dev_mapping_remove(realm, ipa, next_ipa,
							 split);
				split = NULL;
				ipa = next_ipa;
				continue;
			}

			rtt = alloc_rtt(NULL);
			if (rtt == PHYS_ADDR_MAX) {
				ret = -ENOMEM;
				goto out_free_split;
			}
			ret = realm_rtt_create(realm, ipa,
					       RMI_RESULT_DATA_LEVEL(rmi_ret) + 1, rtt);
			if (ret) {
				free_rtt(rtt);
				goto out_free_split;
			}
			goto retry;
		}

		if (rmi_ret != RMI_SUCCESS || next_ipa <= ipa) {
			ret = -EIO;
			goto out_free_split;
		}

		realm_dev_mapping_remove(realm, ipa, next_ipa, split);
		split = NULL;
		ipa = next_ipa;

		ret = undelegate_range_desc(range_desc);
		if (ret)
			break;

		if (may_block)
			cond_resched_rwlock_write(&kvm->mmu_lock);
	}

	if (ipa > start)
		realm_fold_rtt_level(realm, get_start_level(realm) + 1,
				     start, ipa);

	return ret;

out_free_split:
	kfree(split);
	if (ipa > start)
		realm_fold_rtt_level(realm, get_start_level(realm) + 1,
				     start, ipa);
	return ret;
}

static int rmi_rtt_dev_validate(unsigned long rd_phys, unsigned long rec_phys,
		unsigned long base, unsigned long top, unsigned long *out_top,
		unsigned long *rmi_ret)
{
	struct rmi_sro_state *sro __free(kfree) = kmalloc_obj(*sro);
	if (!sro)
		return -ENOMEM;

	*rmi_ret = rmi_sro_memxfer_cmd(sro, GFP_KERNEL, SMC_RMI_RTT_DEV_VALIDATE,
				       rd_phys, rec_phys, base, top);
	if (*rmi_ret)
		return 0;

	*out_top = sro->regs.a1;

	return 0;
}

/*
 * Even though we can map larger block, since we need to delegate each granule.
 * We map granule size and fold
 */
static int __realm_dev_mem_map(struct kvm *kvm, struct kvm_mmu_memory_cache *cache,
		unsigned long pdev_phys, unsigned long vdev_phys,
		unsigned long start_ipa, unsigned long end_ipa,
		phys_addr_t phys, unsigned long *top_ipa)
{
	int ret = 0;
	unsigned long rmi_ret;
	unsigned long ipa = start_ipa, next_ipa;
	struct realm *realm = &kvm->arch.realm;
	phys_addr_t rd_phys = virt_to_phys(realm->rd);

	*top_ipa = start_ipa;
	if (rmi_delegate_range(phys, end_ipa - start_ipa, NULL))
		return -EINVAL;

	while (ipa < end_ipa) {
		unsigned long flags = RMI_ADDR_TYPE_SINGLE;
		unsigned long range_desc = addr_range_desc(phys, end_ipa - ipa,
							   RMI_OP_MEM_DELEGATED);

		ret = rmi_rtt_dev_map(rd_phys, vdev_phys, ipa, end_ipa, flags,
				      range_desc, &next_ipa, &rmi_ret);
		if (ret)
			goto err_undelegate_tail;

		if (RMI_RESULT_STATUS(rmi_ret) == RMI_ERROR_RTT) {
			/* Create missing RTTs and retry */
			int level = RMI_RESULT_DATA_LEVEL(rmi_ret);

			WARN_ON(level == KVM_PGTABLE_LAST_LEVEL);

			if (kvm_mmu_memory_cache_nr_free_objects(cache) <
			    (KVM_PGTABLE_LAST_LEVEL - level)) {
				ret = -ENOMEM;
				goto err_undelegate_tail;
			}

			ret = realm_create_rtt_levels(realm, ipa, level,
						      KVM_PGTABLE_LAST_LEVEL,
						      cache);
			if (ret)
				goto err_undelegate_tail;

			ret = rmi_rtt_dev_map(rd_phys, vdev_phys, ipa, end_ipa, flags,
					      range_desc, &next_ipa, &rmi_ret);
			if (ret)
				goto err_undelegate_tail;
		}

		if (WARN_ON(rmi_ret != RMI_SUCCESS)) {
			ret = -EIO;
			goto err_undelegate_tail;
		}

		phys += next_ipa - ipa;
		ipa = next_ipa;
	}
	/*
	 * successfully mapped the provided range, return the top_ipa
	 */
	*top_ipa = end_ipa;
	return 0;

err_undelegate_tail:
	*top_ipa = ipa;
	/*
	 * undelegate the tail range. Rest will be done by the caller.
	 */
	if (end_ipa > ipa)
		WARN_ON(rmi_undelegate_range(phys, end_ipa - ipa));

	return ret;
}

int realm_dev_mem_map(struct kvm *kvm, unsigned long pdev_phys,
		unsigned long vdev_phys, unsigned long start_ipa,
		unsigned long end_ipa, unsigned long start_pa)
{
	struct realm_dev_mapping *mapping;
	bool mapping_added;
	int ret = 0;
	unsigned long top_ipa;
	unsigned long base_ipa = start_ipa;
	struct realm *realm = &kvm->arch.realm;
	struct kvm_s2_mmu *mmu = &kvm->arch.mmu;
	struct kvm_mmu_memory_cache cache = { .gfp_zero = __GFP_ZERO };

	do {
		mapping = kmalloc_obj(*mapping);
		if (!mapping) {
			ret = -ENOMEM;
			break;
		}
		mapping_added = false;

		ret = kvm_mmu_topup_memory_cache(&cache,
						 kvm_mmu_cache_min_pages(mmu));
		if (ret) {
			kfree(mapping);
			break;
		}

		write_lock(&kvm->mmu_lock);
		if (realm_dev_mapping_overlaps(realm, start_ipa, end_ipa)) {
			ret = -EEXIST;
			top_ipa = start_ipa;
		} else {
			ret = __realm_dev_mem_map(kvm, &cache, pdev_phys,
						  vdev_phys, start_ipa, end_ipa,
						  start_pa, &top_ipa);
			if (top_ipa > start_ipa) {
				mapping->vdev_phys = vdev_phys;
				mapping->start = start_ipa;
				mapping->end = top_ipa;
				list_add_tail(&mapping->node,
					      &realm->dev_mappings);
				mapping_added = true;
			}
		}
		write_unlock(&kvm->mmu_lock);
		if (!mapping_added)
			kfree(mapping);

		/* update base before we break out of loop*/
		start_pa += top_ipa - start_ipa;
		start_ipa = top_ipa;
		if (ret && ret != -ENOMEM)
			break;
	} while (start_ipa < end_ipa);

	kvm_mmu_free_memory_cache(&cache);

	if (ret && start_ipa > base_ipa) {
		int unmap_ret;

		write_lock(&kvm->mmu_lock);
		unmap_ret = __realm_dev_mem_unmap(kvm, base_ipa, start_ipa, false);
		write_unlock(&kvm->mmu_lock);
		WARN_ON(unmap_ret);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(realm_dev_mem_map);

int realm_dev_mem_unmap(struct kvm *kvm, unsigned long vdev_phys)
{
	struct realm *realm = &kvm->arch.realm;
	struct realm_dev_mapping *mapping, *iter;
	int ret = 0;

	write_lock(&kvm->mmu_lock);
	for (;;) {
		mapping = NULL;
		list_for_each_entry(iter, &realm->dev_mappings, node) {
			if (iter->vdev_phys == vdev_phys) {
				mapping = iter;
				break;
			}
		}
		if (!mapping)
			break;

		ret = __realm_dev_mem_unmap(kvm, mapping->start, mapping->end,
					    false);
		if (ret)
			break;
	}
	write_unlock(&kvm->mmu_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(realm_dev_mem_unmap);

static void kvm_complete_vdev_map_validate(struct kvm_vcpu *vcpu)
{
	struct kvm *kvm = vcpu->kvm;
	struct realm_rec *rec = &vcpu->arch.rec;
	struct kvm_run *run = vcpu->run;
	struct realm *realm = &kvm->arch.realm;
	phys_addr_t rd_phys = virt_to_phys(realm->rd);
	phys_addr_t rec_phys = virt_to_phys(rec->rec_page);

	/* reject the vdev_map validate request  */
	if (run->cca_exit.response) {
		rec->run->enter.flags = REC_ENTER_FLAG_DEV_MEM_RESPONSE;
	} else {
		unsigned long next_ipa;
		unsigned long start_ipa = run->cca_exit.gpa_base;

		while (start_ipa < run->cca_exit.gpa_top) {
			int ret;
			unsigned long rmi_ret;

			ret = rmi_rtt_dev_validate(rd_phys, rec_phys, start_ipa,
					   run->cca_exit.gpa_top, &next_ipa,
					   &rmi_ret);
			if (ret || rmi_ret) {
				rec->run->enter.flags = REC_ENTER_FLAG_DEV_MEM_RESPONSE;
				break;
			}
			start_ipa = next_ipa;
		}
	}
}

int kvm_rec_handle_request(struct kvm_vcpu *vcpu)
{
	struct realm_rec *rec = &vcpu->arch.rec;

	if (kvm_realm_state(vcpu->kvm) != REALM_STATE_ACTIVE)
		return -EINVAL;

	switch (rec->run->exit.exit_reason) {
	case RMI_EXIT_SYNC:
		kvm_rec_complete_sysreg_access(vcpu);
		break;
	case RMI_EXIT_PSCI:
		return kvm_rec_complete_psci(vcpu);
	case RMI_EXIT_RIPAS_CHANGE:
		return kvm_complete_ripas_change(vcpu);
	case RMI_EXIT_HOST_CALL:
		for (int i = 0; i < REC_RUN_GPRS; i++)
			rec->run->enter.gprs[i] = vcpu_get_reg(vcpu, i);
		break;
	case RMI_EXIT_VDEV_VALIDATE_MAPPING:
		kvm_complete_vdev_map_validate(vcpu);
		break;
	default:
		KVM_BUG(1, vcpu->kvm, "Unhandled realm exit_reason");
		return -ENXIO;
	}

	return 1;
}

static void noinstr load_realm_timer_state(struct kvm_vcpu *vcpu)
{
	struct rec_exit *rec_exit = &vcpu->arch.rec.run->exit;

	/*
	 * The RMM reports the EL1 timer state on every REC exit. Install that
	 * state before returning to the generic KVM run loop, which expects
	 * the loaded vCPU's timers to be live.
	 */
	write_sysreg_el0(rec_exit->cntv_cval, SYS_CNTV_CVAL);
	write_sysreg_el0(rec_exit->cntp_cval, SYS_CNTP_CVAL);
	isb();

	write_sysreg_el0(rec_exit->cntv_ctl, SYS_CNTV_CTL);
	write_sysreg_el0(rec_exit->cntp_ctl, SYS_CNTP_CTL);
}

static void noinstr rec_enter_sync(struct kvm_vcpu *vcpu)
{
	struct rec_run *run = vcpu->arch.rec.run;
	struct rec_enter *entry = &run->enter;

	if (vcpu_get_flag(vcpu, PENDING_EXCEPTION)) {
		entry->flags |= REC_ENTER_FLAG_INJECT_SEA;
		vcpu_clear_flag(vcpu, PENDING_EXCEPTION);
		vcpu_clear_flag(vcpu, EXCEPT_MASK);
	} else if (vcpu_get_flag(vcpu, INCREMENT_PC)) {
		if (run->exit.exit_reason == RMI_EXIT_SYNC &&
		    ESR_ELx_EC(run->exit.esr) == ESR_ELx_EC_DABT_LOW &&
		    !esr_dabt_is_cm(run->exit.esr)) {
			entry->flags |= REC_ENTER_FLAG_EMULATED_MMIO;
			entry->gprs[0] = vcpu_get_reg(vcpu, 0);
		}
		vcpu_clear_flag(vcpu, INCREMENT_PC);
	}

	/* sync WFx traps */
	entry->flags &= ~(REC_ENTER_FLAG_TRAP_WFE | REC_ENTER_FLAG_TRAP_WFI);
	if (vcpu->arch.hcr_el2 & HCR_TWE)
		entry->flags |= REC_ENTER_FLAG_TRAP_WFE;
	if (vcpu->arch.hcr_el2 & HCR_TWI)
		entry->flags |= REC_ENTER_FLAG_TRAP_WFI;
}

static void noinstr rec_prepare_exit_state(struct kvm_vcpu *vcpu)
{
	struct realm_rec *rec = &vcpu->arch.rec;
	u64 esr = rec->run->exit.esr;
	bool is_write;
	int rt;

	vcpu->arch.fault.esr_el2 = esr;
	vcpu->arch.fault.far_el2 = rec->run->exit.far;
	/* HPFAR_EL2 is only valid for RMI_EXIT_SYNC */
	vcpu->arch.fault.hpfar_el2 = 0;

	/* Reset the emulation flags for the next run of the REC */
	rec->run->enter.flags = 0;

	is_write = (esr & ESR_ELx_SYS64_ISS_DIR_MASK) ==
		   ESR_ELx_SYS64_ISS_DIR_WRITE;
	if (rec->run->exit.exit_reason != RMI_EXIT_SYNC ||
	    ESR_ELx_EC(esr) != ESR_ELx_EC_SYS64 || !is_write)
		return;

	rt = kvm_vcpu_sys_get_rt(vcpu);
	if (rt < REC_RUN_GPRS)
		vcpu_set_reg(vcpu, rt, rec->run->exit.gprs[rt]);
}

static int noinstr rec_perform_vgic_cpuif_access(struct kvm_vcpu *vcpu)
{
	unsigned long elr, spsr, pc, pstate;
	int handled;

	elr = read_sysreg_el2(SYS_ELR);
	spsr = read_sysreg_el2(SYS_SPSR);
	pc = *vcpu_pc(vcpu);
	pstate = *vcpu_cpsr(vcpu);

	/*
	 * RMM has already completed the trapped Realm instruction. Install
	 * disposable AArch64 exception-return state for the PC adjustment made
	 * by the existing early VGIC dispatcher, and restore both views before
	 * returning to the Realm plumbing.
	 */
	*vcpu_cpsr(vcpu) = PSR_MODE_EL1h;
	write_sysreg_el2(0, SYS_ELR);
	write_sysreg_el2(PSR_MODE_EL1h, SYS_SPSR);

	handled = __vgic_v3_perform_cpuif_access(vcpu);

	write_sysreg_el2(elr, SYS_ELR);
	write_sysreg_el2(spsr, SYS_SPSR);
	*vcpu_pc(vcpu) = pc;
	*vcpu_cpsr(vcpu) = pstate;

	return handled;
}

static bool noinstr rec_handle_vgic_cpuif_exit(struct kvm_vcpu *vcpu)
{
	struct realm_rec *rec = &vcpu->arch.rec;
	u64 esr;

	if (!static_branch_unlikely(&vgic_v3_cpuif_trap))
		return false;

	esr = kvm_vcpu_get_esr(vcpu);
	if (rec->run->exit.exit_reason != RMI_EXIT_SYNC ||
	    ESR_ELx_EC(esr) != ESR_ELx_EC_SYS64)
		return false;

	if (rec_perform_vgic_cpuif_access(vcpu) != 1)
		return false;

	kvm_rec_complete_sysreg_access(vcpu);

	/*
	 * The emulation may have updated ICH_HCR_EL2.EOIcount. Synchronize that
	 * update before reading ICH_HCR_EL2 again to enable the CPU interface.
	 */
	isb();
	sysreg_clear_set_s(SYS_ICH_HCR_EL2, 0, ICH_HCR_EL2_En);

	return true;
}

int noinstr kvm_rec_enter(struct kvm_vcpu *vcpu)
{
	struct realm_rec *rec = &vcpu->arch.rec;
	int ret;

	rec_enter_sync(vcpu);
	local_daif_mask();
	pmr_sync();

	do {
		ret = rmi_rec_enter(rec->rec_phys, rec->run_phys);
		if (ret)
			break;

		load_realm_timer_state(vcpu);
		rec_prepare_exit_state(vcpu);
	} while (rec_handle_vgic_cpuif_exit(vcpu));

	local_daif_restore(DAIF_PROCCTX_NOIRQ);

	return ret;
}

static int kvm_create_rec(struct kvm_vcpu *vcpu)
{
	struct user_pt_regs *vcpu_regs = vcpu_gp_regs(vcpu);
	unsigned long mpidr = kvm_vcpu_get_mpidr_aff(vcpu);
	struct realm *realm = &vcpu->kvm->arch.realm;
	struct realm_rec *rec = &vcpu->arch.rec;
	struct rec_params *params;
	long rmi_ret;
	int r, i;

	if (rec->run)
		return -EBUSY;

	/*
	 * The RMM will report PSCI v1.0 to Realms and the KVM_ARM_VCPU_PSCI_0_2
	 * flag covers v0.2 and onwards.
	 */
	if (!vcpu_has_feature(vcpu, KVM_ARM_VCPU_PSCI_0_2))
		return -EINVAL;

	BUILD_BUG_ON(sizeof(*params) > PAGE_SIZE);
	BUILD_BUG_ON(sizeof(*rec->run) > PAGE_SIZE);

	params = (struct rec_params *)get_zeroed_page(GFP_KERNEL);
	rec->rec_page = (void *)__get_free_page(GFP_KERNEL);
	rec->run = (struct rec_run *)get_zeroed_page(GFP_KERNEL);
	rec->sro = kmalloc_obj(*rec->sro);
	if (!params || !rec->rec_page || !rec->run || !rec->sro) {
		r = -ENOMEM;
		goto out_free_pages;
	}

	for (i = 0; i < ARRAY_SIZE(params->gprs); i++)
		params->gprs[i] = vcpu_regs->regs[i];

	params->pc = vcpu_regs->pc;

	if (vcpu->vcpu_id == 0)
		params->flags |= REC_PARAMS_FLAG_RUNNABLE;

	rec->rec_phys = virt_to_phys(rec->rec_page);
	rec->run_phys = virt_to_phys(rec->run);

	if (rmi_delegate_page(rec->rec_phys)) {
		r = -ENXIO;
		goto out_free_pages;
	}

	params->mpidr = mpidr;

	rmi_ret = rmi_rec_create(virt_to_phys(realm->rd), rec->rec_phys,
				 virt_to_phys(params), rec->sro);
	if (rmi_ret) {
		r = rmi_ret < 0 ? rmi_ret : -ENXIO;
		goto out_undelegate_rmm_rec;
	}

	rec->mpidr = mpidr;

	free_page((unsigned long)params);
	return 0;

out_undelegate_rmm_rec:
	if (WARN_ON(rmi_undelegate_page(rec->rec_phys)))
		rec->rec_page = NULL;
out_free_pages:
	free_page((unsigned long)rec->run);
	free_page((unsigned long)rec->rec_page);
	free_page((unsigned long)params);
	kfree(rec->sro);
	rec->run = NULL;
	rec->rec_page = NULL;
	rec->rec_phys = 0;
	rec->run_phys = 0;
	rec->sro = NULL;
	return r;
}

void kvm_destroy_rec(struct kvm_vcpu *vcpu)
{
	struct realm_rec *rec = &vcpu->arch.rec;

	if (!vcpu_is_rec(vcpu))
		return;

	if (!rec->run) {
		/* Nothing to do if the VCPU hasn't been finalized */
		return;
	}

	if (WARN_ON(rmi_rec_destroy(rec->rec_phys, rec->sro)))
		return;

	free_page((unsigned long)rec->run);
	kfree(rec->sro);
	rmi_free_delegated_page(rec->rec_phys);
	rec->run = NULL;
	rec->sro = NULL;
	rec->rec_page = NULL;
	rec->rec_phys = 0;
	rec->run_phys = 0;
}

int kvm_activate_realm(struct kvm *kvm)
{
	struct realm *realm = &kvm->arch.realm;
	struct kvm_vcpu *vcpu;
	unsigned long i;
	int ret;

	if (kvm_realm_state(kvm) >= REALM_STATE_ACTIVE)
		return 0;

	guard(mutex)(&kvm->slots_lock);
	guard(mutex)(&kvm->arch.config_lock);
	/* Check again with the lock held */
	if (kvm_realm_state(kvm) >= REALM_STATE_ACTIVE)
		return 0;

	if (!irqchip_in_kernel(kvm)) {
		/* Userspace irqchip not yet supported with realms */
		return -EOPNOTSUPP;
	}

	if (!vgic_initialized(kvm))
		return -ENODEV;

	ret = realm_ensure_created(kvm);
	if (ret)
		return ret;

	/* Mark state as dead in case we fail */
	kvm_set_realm_state(kvm, REALM_STATE_DEAD);

	kvm_for_each_vcpu(i, vcpu, kvm) {
		ret = kvm_create_rec(vcpu);
		if (ret)
			goto out_vm_dead;
	}

	ret = rmi_realm_activate(virt_to_phys(realm->rd));
	if (ret) {
		ret = -ENXIO;
		goto out_vm_dead;
	}

	kvm_set_realm_state(kvm, REALM_STATE_ACTIVE);
	return 0;

out_vm_dead:
	kvm_vm_dead(kvm);
	return ret;
}

void kvm_destroy_realm(struct kvm *kvm)
{
	struct realm *realm = &kvm->arch.realm;
	size_t pgd_size = kvm_pgtable_stage2_pgd_size(kvm->arch.mmu.vtcr);

	guard(mutex)(&kvm->arch.config_lock);

	if (!kvm_realm_is_created(kvm)) {
		realm_dev_mappings_free(realm);
		kfree(realm->sro);
		realm->sro = NULL;
		return;
	}

	kvm_set_realm_state(kvm, REALM_STATE_DYING);

	/*
	 * REALM_DESTROY requires the realm to be non-live: all RECs must have
	 * been destroyed and the root RTTs must be empty. Unmap the IPA space
	 * and destroy any non-root RTTs before tearing down the RD. The root
	 * RTT pages are still owned by the RMM at this point, so keep the KVM
	 * pgtable alive until after REALM_DESTROY and undelegation.
	 */
	realm_unmap_stage2(kvm);

	if (realm->rd) {
		phys_addr_t rd_phys = virt_to_phys(realm->rd);

		if (WARN_ON(rmi_realm_terminate(rd_phys, realm->sro)))
			return;

		if (WARN_ON(realm_destroy_rtts(kvm)))
			return;

		if (WARN_ON(rmi_realm_destroy(rd_phys, realm->sro)))
			return;
		rmi_free_delegated_page(rd_phys);
		realm->rd = NULL;
	}

	if (WARN_ON(rmi_undelegate_range(kvm->arch.mmu.pgd_phys,
					 pgd_size)))
		return;

	kvm_set_realm_state(kvm, REALM_STATE_DEAD);

	/* Now that the realm is destroyed, free the entry-level RTTs. */
	kvm_free_stage2_pgd(&kvm->arch.mmu);

	WARN_ON(!list_empty(&realm->dev_mappings));
	realm_dev_mappings_free(realm);
	kfree(realm->sro);
	realm->sro = NULL;
}

int kvm_init_realm(struct kvm *kvm)
{
	struct realm *realm = &kvm->arch.realm;

	INIT_LIST_HEAD(&realm->dev_mappings);
	realm->sro = kmalloc_obj(*realm->sro);
	if (!realm->sro)
		return -ENOMEM;

	return 0;
}

static int rmm_check_features(void)
{
	if (kvm_lpa2_is_enabled() &&
	    !rmi_has_feature(0, RMI_FEATURE_REGISTER_0_LPA2)) {
		kvm_err("RMM doesn't support LPA2\n");
		return -ENXIO;
	}

	if (!rmi_has_feature(1, RMI_FEATURE_REGISTER_1_HASH_SHA_256)) {
		kvm_err("RMM doesn't support SHA-256 measurements\n");
		return -ENXIO;
	}

	return 0;
}

void kvm_init_rmi(void)
{
	if (kvm_get_mode() != KVM_MODE_RMM)
		return;

	if (!is_rmi_available())
		return;

	if (rmm_check_features())
		return;

	kvm_info("Realm guests supported\n");
	static_branch_enable(&kvm_rmi_is_available);
}
