/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2026 ARM Ltd.
 */

#ifndef __LINUX_ARM_RMI_CMDS_H_
#define __LINUX_ARM_RMI_CMDS_H_

#include <linux/arm-smccc-rmi.h>
#include <linux/bug.h>
#include <linux/gfp.h>
#include <linux/processor.h>
#include <linux/string.h>
#include <linux/types.h>

#define RMI_MAX_ADDR_LIST	256

struct rtt_entry {
	unsigned long walk_level;
	unsigned long desc;
	int state;
	int ripas;
};

struct rmi_sro_state {
	struct arm_smccc_1_2_regs regs;
	int addr_count;
	unsigned long addr_list[RMI_MAX_ADDR_LIST];
};

#define RMM_BLOCKED_RETRY_COUNT		2
/*
 * rmi_smccc_invoke: Invoke the RMI call and return the results, retrying the
 * command when status is RMI_BUSY. If we encounter RMI_BLOCKED, we retry
 * it one more time before we give up. The caller is supposed to handle the
 * result and reissue if required.
 *
 * We don't expect to see RMI_BLOCKED on a practical system, except when
 * there are parallel requests that results in long standing operation,
 * with one blocking the other.
 *
 * @regs: Input parameters filled in. Updated with the ouptput results
 * after the call.
 */
static inline void rmi_smccc_invoke(struct arm_smccc_1_2_regs *regs)
{
	struct arm_smccc_1_2_regs args = *regs;
	long status;
	int i = 0;

	while (i < RMM_BLOCKED_RETRY_COUNT) {
		arm_smccc_1_2_invoke(&args, regs);

		status = RMI_RESULT_STATUS(regs->a0);
		if (status != RMI_BUSY && status != RMI_BLOCKED)
			break;
		if (status == RMI_BLOCKED)
			i++;
		cpu_relax();
	}
}

unsigned long rmi_feat_reg(unsigned long index);

int rmi_delegate_range(phys_addr_t phys, unsigned long size,
		       phys_addr_t *out_phys);
int rmi_undelegate_range(phys_addr_t phys, unsigned long size);
int rmi_free_delegated_page(phys_addr_t phys);

static inline int rmi_delegate_page(phys_addr_t phys)
{
	return rmi_delegate_range(phys, PAGE_SIZE, NULL);
}

static inline int rmi_undelegate_page(phys_addr_t phys)
{
	return rmi_undelegate_range(phys, PAGE_SIZE);
}

bool is_rmi_available(void);

long rmi_sro_memxfer_execute(struct rmi_sro_state *sro, gfp_t gfp);
void rmi_sro_free(struct rmi_sro_state *sro);
long rmi_sro_execute(struct arm_smccc_1_2_regs *regs);

/*
 * Resetting the addr_count is sufficient to ignore the addr_list contents.
 */
#define rmi_sro_memxfer_cmd(sro, gfp, ...) ({				\
	struct rmi_sro_state *__sro = (sro);				\
	__sro->addr_count = 0;						\
	__sro->regs = (struct arm_smccc_1_2_regs){ __VA_ARGS__ };	\
	long __ret = rmi_sro_memxfer_execute(__sro, gfp);		\
	rmi_sro_free(__sro);						\
	__ret;								\
})

/**
 * rmi_rtt_data_map_init() - Create a mapping at protected IPA, copying contents
 *			     from a given non-secure source granule.
 * @rd: PA of the RD
 * @data: PA of the target granule mapped in the guest
 * @ipa: IPA at which the granule @data will be mapped in the guest
 * @src: PA of the source granule with contents
 * @flags: RMI_MEASURE_CONTENT if the contents should be measured
 *
 * Create a mapping from Protected IPA space to conventional memory, copying
 * contents from a Non-secure Granule provided by the caller.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_rtt_data_map_init(unsigned long rd, unsigned long data,
					 unsigned long ipa, unsigned long src,
					 unsigned long flags)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_RTT_DATA_MAP_INIT, rd, data, ipa, src, flags
	};

	return rmi_sro_execute(&regs);
}

/**
 * rmi_rtt_data_map() - Create mappings in protected IPA range with unknown contents
 * @rd: PA of the RD
 * @base: Base of the target IPA range
 * @top: Top of the target IPA range
 * @flags: Flags
 * @oaddr: Output address set descriptor
 * @out_top: Top address of range which was processed.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_rtt_data_map(unsigned long rd,
				    unsigned long base,
				    unsigned long top,
				    unsigned long flags,
				    unsigned long oaddr,
				    unsigned long *out_top)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_RTT_DATA_MAP, rd, base, top, flags, oaddr
	};
	long ret;

	ret = rmi_sro_execute(&regs);

	if (ret == RMI_SUCCESS && out_top)
		*out_top = regs.a1;

	return ret;
}

/**
 * rmi_rtt_data_unmap() - Remove mappings to conventional memory at a protected
 *			  IPA range
 * @rd: PA of the RD
 * @base: Base of the target IPA range
 * @top: Top of the target IPA range
 * @flags: Flags
 * @oaddr: Output address set descriptor
 * @out_top: Returns top IPA of range which has been unmapped
 * @out_range: Output address range
 * @out_count: Number of entries in output address list
 *
 * Removes mappings to convention memory with a target Protected IPA range.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_rtt_data_unmap(unsigned long rd,
				      unsigned long base,
				      unsigned long top,
				      unsigned long flags,
				      unsigned long oaddr,
				      unsigned long *out_top,
				      unsigned long *out_range,
				      unsigned long *out_count)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_RTT_DATA_UNMAP, rd, base, top, flags, oaddr
	};
	long ret;

	ret = rmi_sro_execute(&regs);

	if (ret != RMI_SUCCESS)
		return ret;

	if (out_top)
		*out_top = regs.a1;
	if (out_range)
		*out_range = regs.a2;
	if (out_count)
		*out_count = regs.a3;

	return RMI_SUCCESS;
}

/**
 * rmi_psci_complete() - Complete pending PSCI command
 * @calling_rec: PA of the calling REC
 * @status: Status of the PSCI request
 *
 * Completes a pending PSCI command.
 *
 * Return: RMI return code
 */
static inline long rmi_psci_complete(unsigned long calling_rec,
				     unsigned long status)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_PSCI_COMPLETE, calling_rec, status,
	};

	rmi_smccc_invoke(&regs);
	return regs.a0;
}

/**
 * rmi_realm_activate() - Activate a realm
 * @rd: PA of the RD
 *
 * Mark a realm as Active, signalling that creation is completed, allowing
 * execution of the realm.
 *
 * Return: RMI return code
 */
static inline long rmi_realm_activate(unsigned long rd)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_REALM_ACTIVATE, rd,
	};

	rmi_smccc_invoke(&regs);
	return regs.a0;
}

/**
 * rmi_realm_create() - Create a realm
 * @rd: PA of the RD
 * @params: PA of realm parameters
 * @sro: Preallocated SRO context
 *
 * Create a new realm using the given parameters.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_realm_create(unsigned long rd, unsigned long params,
				    struct rmi_sro_state *sro)
{
	return rmi_sro_memxfer_cmd(sro, GFP_KERNEL,
				   SMC_RMI_REALM_CREATE, rd, params);
}

/**
 * rmi_realm_terminate() - Terminate a realm
 * @rd: PA of the RD
 * @sro: Preallocated SRO context
 *
 * Terminates a realm, moving it into a ZOMBIE state
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_realm_terminate(unsigned long rd,
				       struct rmi_sro_state *sro)
{
	return rmi_sro_memxfer_cmd(sro, GFP_KERNEL,
				   SMC_RMI_REALM_TERMINATE, rd);
}

/**
 * rmi_realm_destroy() - Destroy a realm
 * @rd: PA of the RD
 * @sro: Preallocated SRO context
 *
 * Destroys a realm, all objects belonging to the realm must be destroyed first.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_realm_destroy(unsigned long rd,
				     struct rmi_sro_state *sro)
{
	return rmi_sro_memxfer_cmd(sro, GFP_KERNEL,
				   SMC_RMI_REALM_DESTROY, rd);
}

/**
 * rmi_rec_create() - Create a REC
 * @rd: PA of the RD
 * @rec: PA of the target REC
 * @params: PA of REC parameters
 * @sro: Allocated SRO context to be used
 *
 * Create a REC using the parameters specified in the struct rec_params pointed
 * to by @params.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_rec_create(unsigned long rd,
				  unsigned long rec,
				  unsigned long params,
				  struct rmi_sro_state *sro)
{
	long ret;

	sro->addr_count = 0;
	sro->regs = (struct arm_smccc_1_2_regs) {
		SMC_RMI_REC_CREATE, rd, rec, params
	};
	ret = rmi_sro_memxfer_execute(sro, GFP_KERNEL);
	rmi_sro_free(sro);

	return ret;
}

/**
 * rmi_rec_destroy() - Destroy a REC
 * @rec: PA of the target REC
 * @sro: Allocated SRO context to be used
 *
 * Destroys a REC. The REC must not be running.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_rec_destroy(unsigned long rec,
				   struct rmi_sro_state *sro)
{
	return rmi_sro_memxfer_cmd(sro, GFP_KERNEL, SMC_RMI_REC_DESTROY, rec);
}

/**
 * rmi_rec_enter() - Enter a REC
 * @rec: PA of the target REC
 * @run_ptr: PA of RecRun structure
 *
 * Starts (or continues) execution within a REC.
 *
 * Return: RMI return code
 */
static inline long rmi_rec_enter(unsigned long rec, unsigned long run_ptr)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_REC_ENTER, rec, run_ptr,
	};

	rmi_smccc_invoke(&regs);
	return regs.a0;
}

/**
 * rmi_rtt_create() - Creates an RTT
 * @rd: PA of the RD
 * @rtt: PA of the target RTT
 * @ipa: Base of the IPA range described by the RTT
 * @level: Depth of the RTT within the tree
 *
 * Creates an RTT (Realm Translation Table) at the specified level for the
 * translation of the specified address within the realm.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_rtt_create(unsigned long rd, unsigned long rtt,
				  unsigned long ipa, long level)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_RTT_CREATE, rd, rtt, ipa, level
	};

	return rmi_sro_execute(&regs);
}

/**
 * rmi_rtt_destroy() - Destroy an RTT
 * @rd: PA of the RD
 * @ipa: Base of the IPA range described by the RTT
 * @level: RTT level
 * @out_rtt: Pointer to write the PA of the RTT which was destroyed
 * @out_top: Pointer to write the top IPA of non-live RTT entries, from entry
 * at which the RTT walk terminated.
 *
 * Destroys an RTT. The RTT must be non-live, i.e. none of the entries in the
 * table are in ASSIGNED or TABLE state.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code.
 */
static inline long rmi_rtt_destroy(unsigned long rd,
				   unsigned long ipa,
				   long level,
				   unsigned long *out_rtt,
				   unsigned long *out_top)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_RTT_DESTROY, rd, ipa, level
	};
	long ret = rmi_sro_execute(&regs);

	switch (RMI_RESULT_STATUS(ret)) {
	case RMI_SUCCESS:
		if (out_rtt)
			*out_rtt = regs.a1;
		fallthrough;
	case RMI_ERROR_RTT:
		if (out_top)
			*out_top = regs.a2;
		break;
	default:
		break;
	}

	return ret;
}

/**
 * rmi_rtt_fold() - Fold an RTT
 * @rd: PA of the RD
 * @ipa: Base of the IPA range described by the RTT
 * @level: Depth of the RTT within the tree
 * @out_rtt: Pointer to write the PA of the RTT which was destroyed
 *
 * Folds an RTT. If all entries with the RTT are 'homogeneous' the RTT can be
 * folded into the parent and the RTT destroyed.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_rtt_fold(unsigned long rd, unsigned long ipa,
				long level, unsigned long *out_rtt)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_RTT_FOLD, rd, ipa, level
	};
	long ret = rmi_sro_execute(&regs);

	if (ret == RMI_SUCCESS && out_rtt)
		*out_rtt = regs.a1;

	return ret;
}

/**
 * rmi_rtt_init_ripas() - Set RIPAS for new realm
 * @rd: PA of the RD
 * @base: Base of target IPA region
 * @top: Top of target IPA region
 * @out_top: Top IPA of range whose RIPAS was modified
 *
 * Sets the RIPAS of a target IPA range to RAM, for a realm in the NEW state.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_rtt_init_ripas(unsigned long rd, unsigned long base,
				      unsigned long top, unsigned long *out_top)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_RTT_INIT_RIPAS, rd, base, top
	};
	long ret = rmi_sro_execute(&regs);

	if (ret == RMI_SUCCESS && out_top)
		*out_top = regs.a1;

	return ret;
}

/**
 * rmi_rtt_unprot_map() - Map unprotected granules into a realm
 * @rd: PA of the RD
 * @base: Base IPA of the mapping
 * @top: Top of the target IPA range
 * @flags: Flags
 * @oaddr: Output address set descriptor
 * @out_top: Top IPA of range which has been mapped
 *
 * Create mappings to memory within a target unprotected IPA range.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_rtt_unprot_map(unsigned long rd,
				      unsigned long base,
				      unsigned long top,
				      unsigned long flags,
				      unsigned long oaddr,
				      unsigned long *out_top)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_RTT_UNPROT_MAP, rd, base, top, flags, oaddr
	};
	long ret = rmi_sro_execute(&regs);

	if (ret == RMI_SUCCESS && out_top)
		*out_top = regs.a1;

	return ret;
}

/**
 * rmi_rtt_read_entry() - Read an RTTE
 * @rd: PA of the RD
 * @ipa: IPA for which to read the RTTE
 * @level: RTT level at which to read the RTTE
 * @rtt: Output structure describing the RTTE
 *
 * Reads a RTTE (Realm Translation Table Entry).
 *
 * Return: RMI return code
 */
static inline long rmi_rtt_read_entry(unsigned long rd, unsigned long ipa,
		long level, struct rtt_entry *rtt)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_RTT_READ_ENTRY, rd, ipa, level
	};

	rmi_smccc_invoke(&regs);

	rtt->walk_level = regs.a1;
	rtt->state = regs.a2 & 0xff;
	rtt->desc = regs.a3;
	rtt->ripas = regs.a4 & 0xff;

	return regs.a0;
}

/**
 * rmi_rtt_set_ripas() - Set RIPAS for an running realm
 * @rd: PA of the RD
 * @rec: PA of the REC making the request
 * @base: Base of target IPA region
 * @top: Top of target IPA region
 * @out_top: Pointer to write top IPA of range whose RIPAS was modified
 *
 * Completes a request made by the realm to change the RIPAS of a target IPA
 * range.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_rtt_set_ripas(unsigned long rd, unsigned long rec,
				     unsigned long base, unsigned long top,
				     unsigned long *out_top)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_RTT_SET_RIPAS, rd, rec, base, top
	};
	long ret = rmi_sro_execute(&regs);

	if (ret == RMI_SUCCESS && out_top)
		*out_top = regs.a1;

	return ret;
}

/**
 * rmi_rtt_unprot_unmap() - Remove mappings within an unprotected IPA range
 * @rd: PA of the RD
 * @base: Base IPA of the mapping
 * @top: Top of the target IPA range
 * @flags: Flags
 * @oaddr: Output address set descriptor
 * @out_top: Top IPA which has been unmapped
 * @out_range: Output address range
 * @out_count: Number of entries in output address list
 *
 * Removes mappings to memory within a target unprotected IPA range.
 *
 * Return: 0 on success, positive RMI result code or negative Linux error code
 */
static inline long rmi_rtt_unprot_unmap(unsigned long rd,
					unsigned long base,
					unsigned long top,
					unsigned long flags,
					unsigned long oaddr,
					unsigned long *out_top,
					unsigned long *out_range,
					unsigned long *out_count)
{
	struct arm_smccc_1_2_regs regs = {
		SMC_RMI_RTT_UNPROT_UNMAP, rd, base, top, flags, oaddr
	};
	long ret = rmi_sro_execute(&regs);

	if (ret != RMI_SUCCESS)
		return ret;

	if (out_top)
		*out_top = regs.a1;
	if (out_range)
		*out_range = regs.a2;
	if (out_count)
		*out_count = regs.a3;

	return RMI_SUCCESS;
}

static inline unsigned long rmi_pdev_get_state(unsigned long pdev_phys, enum rmi_pdev_state *state)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(SMC_RMI_PDEV_GET_STATE, pdev_phys, &res);

	*state = res.a1;
	return res.a0;
}

static inline unsigned long rmi_pdev_communicate(unsigned long pdev_phys,
						 unsigned long pdev_comm_data_phys)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(SMC_RMI_PDEV_COMMUNICATE,
			     pdev_phys, pdev_comm_data_phys, &res);

	return res.a0;
}

static inline unsigned long rmi_pdev_abort(unsigned long pdev_phys)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(SMC_RMI_PDEV_ABORT, pdev_phys, &res);

	return res.a0;
}

static inline unsigned long rmi_vdev_communicate(unsigned long rd_phys,
		unsigned long pdev_phys, unsigned long vdev_phys,
		unsigned long vdev_comm_data_phys)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(SMC_RMI_VDEV_COMMUNICATE, rd_phys, pdev_phys,
			     vdev_phys, vdev_comm_data_phys, &res);

	return res.a0;
}

static inline unsigned long rmi_vdev_get_state(unsigned long vdev_phys,
		enum rmi_vdev_state *state)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(SMC_RMI_VDEV_GET_STATE, vdev_phys, &res);

	*state = res.a1;
	return res.a0;
}

static inline unsigned long rmi_vdev_abort(unsigned long vdev_phys)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(SMC_RMI_VDEV_ABORT, vdev_phys, &res);

	return res.a0;
}

static inline unsigned long rmi_pdev_stop(unsigned long pdev_phys)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(SMC_RMI_PDEV_STOP, pdev_phys, &res);

	return res.a0;
}

static inline unsigned long rmi_pdev_set_pubkey(unsigned long pdev_phys, unsigned long key_phys)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(SMC_RMI_PDEV_SET_PUBKEY, pdev_phys, key_phys, &res);

	return res.a0;
}

static inline unsigned long rmi_pdev_stream_connect(unsigned long stream_params_phys,
		unsigned long *stream_handle)
{

	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(SMC_RMI_PDEV_STREAM_CONNECT, stream_params_phys, &res);

	*stream_handle = res.a1;
	return res.a0;
}

static inline unsigned long rmi_pdev_stream_complete(unsigned long pdev1_phys,
		unsigned long pdev2_phys, unsigned long stream_handle)
{

	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(SMC_RMI_PDEV_STREAM_COMPLETE, pdev1_phys,
			     pdev2_phys, stream_handle, &res);

	return res.a0;
}

static inline unsigned long rmi_pdev_stream_disconnect(unsigned long pdev1_phys,
		unsigned long pdev2_phys, unsigned long stream_handle)
{

	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(SMC_RMI_PDEV_STREAM_DISCONNECT, pdev1_phys,
			     pdev2_phys, stream_handle, &res);

	return res.a0;
}

#endif
