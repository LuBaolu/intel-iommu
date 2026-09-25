// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2026 ARM Ltd.
 */

#include <linux/kvm_host.h>

#include <linux/arm-smccc-rmi.h>
#include <asm/kvm_emulate.h>
#include <asm/kvm_rmi.h>
#include <asm/kvm_mmu.h>

static int rec_exit_fatal(struct kvm_vcpu *vcpu, const char *reason,
			  unsigned long value)
{
	vcpu_err(vcpu, "%s: %#lx\n", reason, value);
	vcpu->run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
	kvm_vm_dead(vcpu->kvm);
	return ARM_EXCEPTION_EXIT;
}

static void rec_exit_sync(struct kvm_vcpu *vcpu)
{
	struct realm_rec *rec = &vcpu->arch.rec;
	u64 esr = kvm_vcpu_get_esr(vcpu);
	u8 ec = ESR_ELx_EC(esr);

	switch (ec) {
	case ESR_ELx_EC_SYS64:
		if ((esr & ESR_ELx_SYS64_ISS_DIR_MASK) ==
		    ESR_ELx_SYS64_ISS_DIR_READ)
			kvm_make_request(KVM_REQ_RMI, vcpu);
		break;
	case ESR_ELx_EC_DABT_LOW:
		/*
		 * The RMM reports the value of an MMIO write in gprs[0],
		 * irrespective of the register named by the instruction.
		 */
		if (kvm_vcpu_dabt_iswrite(vcpu) &&
		    kvm_vcpu_dabt_isvalid(vcpu))
			vcpu_set_reg(vcpu, kvm_vcpu_dabt_get_rd(vcpu),
				     rec->run->exit.gprs[0]);
		break;
	}
}

static void rec_exit_hvc(struct kvm_vcpu *vcpu)
{
	struct realm_rec *rec = &vcpu->arch.rec;
	int i;

	for (i = 0; i < REC_RUN_GPRS; i++)
		vcpu_set_reg(vcpu, i, rec->run->exit.gprs[i]);

	vcpu->arch.fault.esr_el2 = (ESR_ELx_EC_HVC64 << ESR_ELx_EC_SHIFT) |
				     ESR_ELx_IL;
}

static int rec_exit_ripas_change(struct kvm_vcpu *vcpu)
{
	struct realm_rec *rec = &vcpu->arch.rec;
	struct realm *realm;
	unsigned long base;
	unsigned long ripas;
	unsigned long top;

	realm = &vcpu->kvm->arch.realm;
	base = rec->run->exit.ripas_base;
	top = rec->run->exit.ripas_top;
	ripas = rec->run->exit.ripas_value;

	if (top <= base ||
	    !kvm_realm_is_private_address(realm, base) ||
	    !kvm_realm_is_private_address(realm, top - 1)) {
		vcpu_err(vcpu, "Invalid RIPAS_CHANGE for %#lx - %#lx, ripas: %#lx\n",
			 base, top, ripas);
		/* Set RMI_REJECT bit */
		rec->run->enter.flags = REC_ENTER_FLAG_RIPAS_RESPONSE;
		return -EINVAL;
	}

	/* Exit to VMM, the actual RIPAS change is done on next entry */
	kvm_prepare_memory_fault_exit(vcpu, base, top - base, false, false,
				      ripas == RMI_RAM);
	kvm_make_request(KVM_REQ_RMI, vcpu);

	/*
	 * KVM_EXIT_MEMORY_FAULT requires a return code of -EFAULT, see the
	 * API documentation
	 */
	return -EFAULT;
}

int kvm_rec_exit(struct kvm_vcpu *vcpu, int rec_run_ret)
{
	struct realm_rec *rec = &vcpu->arch.rec;
	unsigned long status;

	if (rec_run_ret < 0)
		return rec_exit_fatal(vcpu, "REC_ENTER failed", rec_run_ret);

	status = RMI_RESULT_STATUS(rec_run_ret);

	/*
	 * If a PSCI_SYSTEM_OFF request raced with a vcpu executing, we might
	 * see the following status code indicating an attempt to run
	 * a REC when the RD state is SYSTEM_OFF.  In this case, we just need to
	 * return to user space which can deal with the system event or will try
	 * to run the KVM VCPU again, at which point we will no longer attempt
	 * to enter the Realm because we will have a sleep request pending on
	 * the VCPU as a result of KVM's PSCI handling.
	 */
	if (status == RMI_ERROR_REALM) {
		vcpu->run->exit_reason = KVM_EXIT_SHUTDOWN;
		return ARM_EXCEPTION_EXIT;
	}

	/*
	 * If a VCPU has been turned on, but the REC state hasn't been updated
	 * we may experience RMI_ERROR_REC. Exit to the userspace with -EAGAIN
	 * for a retry.
	 */
	if (status == RMI_ERROR_REC)
		return -EAGAIN;
	if (rec_run_ret)
		return rec_exit_fatal(vcpu, "Unexpected REC_ENTER status",
				      rec_run_ret);

	switch (rec->run->exit.exit_reason) {
	case RMI_EXIT_SYNC:
		/*
		 * HPFAR_EL2_NS is hijacked to indicate a valid HPFAR value,
		 * see __get_fault_info()
		 */
		vcpu->arch.fault.hpfar_el2 = rec->run->exit.hpfar | HPFAR_EL2_NS;
		rec_exit_sync(vcpu);
		return ARM_EXCEPTION_TRAP;
	case RMI_EXIT_IRQ:
	case RMI_EXIT_FIQ:
		return ARM_EXCEPTION_IRQ;
	case RMI_EXIT_SERROR:
		return ARM_EXCEPTION_EL1_SERROR;
	case RMI_EXIT_PSCI:
	case RMI_EXIT_HOST_CALL:
		rec_exit_hvc(vcpu);
		/*
		 * Queue completion before dispatching the exit through the
		 * generic HVC handling path. The request will be processed after
		 * HVC handling has updated the vCPU state and before the next REC
		 * entry.
		 */
		kvm_make_request(KVM_REQ_RMI, vcpu);
		return ARM_EXCEPTION_TRAP;
	case RMI_EXIT_RIPAS_CHANGE:
		return rec_exit_ripas_change(vcpu);
	}

	return rec_exit_fatal(vcpu, "Unsupported Realm exit reason",
			      rec->run->exit.exit_reason);
}
