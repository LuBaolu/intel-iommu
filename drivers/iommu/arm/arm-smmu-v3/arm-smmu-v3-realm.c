// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 ARM Ltd.
 */

#include <linux/arm-smccc-rmi.h>
#include <linux/interrupt.h>

#include <linux/arm-rmi-cmds.h>

#include "arm-smmu-v3.h"

#define	RMI_PSMMU_IRQ_PEND_GERROR	BIT(0)
#define	RMI_PSMMU_IRQ_PEND_EVENTQ	BIT(1)
#define	RMI_PSMMU_IRQ_PEND_PRIQ		BIT(2)
#define	RMI_PSMMU_IRQ_PEND_CMDQ		BIT(3)

#define	RMI_PSMMU_IRQ_EVENT_NONE	0
#define	RMI_PSMMU_IRQ_EVENT_ERROR	1
#define	RMI_PSMMU_IRQ_EVENT_PSMMU	2
#define	RMI_PSMMU_IRQ_EVENT_VSMMU	3
#define RMI_PSMMU_IRQ_EVENT_MASK	GENMASK(2, 1)
#define RMI_PSMMU_IRQ_EVENT_SHIFT	1
#define RMI_PSMMU_IRQ_EVENT_PENDING	0x1

#define	RMI_PSMMU_IRQ_GERROR	0
#define	RMI_PSMMU_IRQ_EVENTQ	1
#define	RMI_PSMMU_IRQ_PRIQ	2
#define	RMI_PSMMU_IRQ_CMDQ	3

static irqreturn_t arm_smmu_realm_notify_thread(int irq, void *dev)
{
	int rmi_psmmu_event;
	unsigned long notify_flags, psmmu_irq;
	struct arm_smmu_device *smmu = dev;
	struct rmi_psmmu_event_details event;

	if (irq == smmu->realm.evtq_irq) {
		notify_flags =  RMI_PSMMU_IRQ_PEND_EVENTQ;
		psmmu_irq = RMI_PSMMU_IRQ_EVENTQ;
	} else if (irq == smmu->realm.gerr_irq) {
		notify_flags =  RMI_PSMMU_IRQ_PEND_GERROR;
		psmmu_irq = RMI_PSMMU_IRQ_GERROR;
	} else if (irq == smmu->realm.pri_irq) {
		notify_flags =  RMI_PSMMU_IRQ_PEND_PRIQ;
		psmmu_irq = RMI_PSMMU_IRQ_PRIQ;
	} else {
		return IRQ_HANDLED;
	}

	do {
		if (rmi_psmmu_irq_notify(smmu->base_phys,
					 notify_flags, &event)) {
			dev_warn(smmu->dev,
				 "failed to notify RMM of a SMMU event\n");
			/* there is nothing much we could do. Mark it handled. */
			return IRQ_HANDLED;
		}
		rmi_psmmu_event = (event.flags & RMI_PSMMU_IRQ_EVENT_MASK) >>
				  RMI_PSMMU_IRQ_EVENT_SHIFT;
		switch (rmi_psmmu_event) {
		case RMI_PSMMU_IRQ_EVENT_NONE:
			break;
		case RMI_PSMMU_IRQ_EVENT_ERROR:
			dev_warn(smmu->dev, "SMMU Error reported\n");
			rmi_psmmu_event_consume(smmu->base_phys, psmmu_irq);
			break;
		case RMI_PSMMU_IRQ_EVENT_PSMMU:
			dev_warn(smmu->dev,
				 "SMMU event (event num: 0x%llx syndrome 0x%llx "
				 "fetch_addr 0x%llx input_addr 0x%llx) reported\n",
				 event.event_num, event.syndrome,
				 event.fetch_addr, event.input_addr);
			rmi_psmmu_event_consume(smmu->base_phys, psmmu_irq);
			break;
		case RMI_PSMMU_IRQ_EVENT_VSMMU:
			dev_warn(smmu->dev, "Wrong VSMMU event on stream 0x%llx, ignoring\n",
				 event.stream_id);
			rmi_psmmu_event_consume(smmu->base_phys, psmmu_irq);
			break;
		}

	} while (event.flags & RMI_PSMMU_IRQ_EVENT_PENDING);

	return IRQ_HANDLED;
}

void arm_smmu_setup_realm_irqs(struct arm_smmu_device *smmu)
{
	int irq, ret;

	irq = smmu->realm.evtq_irq;
	if (irq) {
		ret = devm_request_threaded_irq(smmu->dev, irq, NULL,
						arm_smmu_realm_notify_thread,
						IRQF_ONESHOT,
						"arm-smmu-v3-realm-evtq",
						smmu);
		if (ret < 0)
			dev_warn(smmu->dev, "failed to enable realm evtq irq\n");
	} else {
		dev_warn(smmu->dev, "no realm evtq irq - events will not be reported!\n");
	}

	irq = smmu->realm.gerr_irq;
	if (irq) {
		ret = devm_request_threaded_irq(smmu->dev, irq, NULL,
						arm_smmu_realm_notify_thread,
						IRQF_ONESHOT,
						"arm-smmu-v3-realm-gerror",
						smmu);
		if (ret < 0)
			dev_warn(smmu->dev, "failed to enable realm gerror irq\n");
	} else {
		dev_warn(smmu->dev, "no realm gerr irq - errors will not be reported!\n");
	}

	if (smmu->features & ARM_SMMU_FEAT_PRI) {
		irq = smmu->realm.pri_irq;
		if (irq) {

			ret = devm_request_threaded_irq(smmu->dev, irq, NULL,
							arm_smmu_realm_notify_thread,
							IRQF_ONESHOT,
							"arm-smmu-v3-realm-priq",
							smmu);
			if (ret < 0)
				dev_warn(smmu->dev,
					 "failed to enable realm priq irq\n");
		} else {
			dev_warn(smmu->dev, "no realm priq irq - PRI will be broken\n");
		}
	}
}
