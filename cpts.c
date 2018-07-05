/*
 * TI Common Platform Time Sync
 *
 * Copyright (C) 2012 Richard Cochran <richardcochran@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <linux/err.h>
#include <linux/if.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/ptp_classify.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/math64.h>

#include "cpts.h"

#ifdef CONFIG_TI_CPTS

#define AM335X_NO_PRESCALER 0xFFFFFFFF

#define cpts_read32(c, r)	__raw_readl(&c->reg->r)
#define cpts_write32(c, v, r)	__raw_writel(v, &c->reg->r)

static int event_expired(struct cpts_event *event)
{
	return time_after(jiffies, event->tmo);
}

static int event_type(struct cpts_event *event)
{
	return (event->high >> EVENT_TYPE_SHIFT) & EVENT_TYPE_MASK;
}

static int event_port(struct cpts_event *event)
{
	return (event->high >> PORT_NUMBER_SHIFT) & PORT_NUMBER_MASK;
}

static int cpts_fifo_pop(struct cpts *cpts, u32 *high, u32 *low)
{
	u32 r = cpts_read32(cpts, intstat_raw);

	if (r & TS_PEND_RAW) {
		*high = cpts_read32(cpts, event_high);
		*low  = cpts_read32(cpts, event_low);
		cpts_write32(cpts, EVENT_POP, event_pop);
		return 0;
	}
	return -1;
}

/*
 * Returns zero if matching event type was found.
 */
static int cpts_fifo_read(struct cpts *cpts, int match)
{
	int i, type = -1;
	u32 hi, lo;
	struct cpts_event *event;
	struct ptp_clock_event pevent;
	for (i = 0; i < CPTS_FIFO_DEPTH; i++) {
		if (cpts_fifo_pop(cpts, &hi, &lo))
			break;
		if (list_empty(&cpts->pool)) {
			pr_err("cpts: event pool is empty\n");
			return -1;
		}
		event = list_first_entry(&cpts->pool, struct cpts_event, list);
		event->tmo = jiffies + 2;
		event->high = hi;
		event->low = lo;
		type = event_type(event);
		switch (type) {
		case CPTS_EV_HW:
			pevent.timestamp = timecounter_cyc2time(&cpts->tc, event->low);
			pevent.type = PTP_CLOCK_EXTTS;
			pevent.index = event_port(event) - 1;
            if(cpts->pins[pevent.index].state.type == PTP_CLK_REQ_PEROUT)
            {
                cpts->pins[pevent.index].capture = pevent.timestamp;
                cpts->pins[pevent.index].new_capture = true;
            }
            else
            {
			    ptp_clock_event(cpts->clock, &pevent);
            }
			break;
		case CPTS_EV_PUSH:
		case CPTS_EV_RX:
		case CPTS_EV_TX:
			list_del_init(&event->list);
			list_add_tail(&event->list, &cpts->events);
			break;
		case CPTS_EV_ROLL:
		case CPTS_EV_HALF:
			break;
		default:
			pr_err("cpts: unknown event type\n");
			break;
		}
		if (type == match)
			break;
	}
	return type == match ? 0 : -1;
}

static cycle_t cpts_systim_read(const struct cyclecounter *cc)
{
	u64 val = 0;
	struct cpts_event *event;
	struct list_head *this, *next;
	struct cpts *cpts = container_of(cc, struct cpts, cc);

	cpts_write32(cpts, TS_PUSH, ts_push);
	if (cpts_fifo_read(cpts, CPTS_EV_PUSH))
		pr_err("cpts: unable to obtain a time stamp\n");

	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_type(event) == CPTS_EV_PUSH) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			val = event->low;
			break;
		}
	}

	return val;
}

/* PTP clock operations */

static int cpts_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u64 adj;
	u32 diff, mult;
	int neg_adj = 0;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}
	mult = cpts->cc_mult;
	adj = mult;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);

	spin_lock_irqsave(&cpts->lock, flags);

	timecounter_read(&cpts->tc);

	cpts->cc.mult = neg_adj ? mult - diff : mult + diff;

	spin_unlock_irqrestore(&cpts->lock, flags);

	return 0;
}

static int cpts_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	spin_lock_irqsave(&cpts->lock, flags);
	timecounter_adjtime(&cpts->tc, delta);
	spin_unlock_irqrestore(&cpts->lock, flags);

	return 0;
}

static int cpts_ptp_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	u64 ns;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	spin_lock_irqsave(&cpts->lock, flags);
	ns = timecounter_read(&cpts->tc);
	spin_unlock_irqrestore(&cpts->lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

static int cpts_ptp_settime(struct ptp_clock_info *ptp,
			    const struct timespec64 *ts)
{
	u64 ns;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	ns = timespec64_to_ns(ts);

	spin_lock_irqsave(&cpts->lock, flags);
	timecounter_init(&cpts->tc, &cpts->cc, ns);
	spin_unlock_irqrestore(&cpts->lock, flags);

	return 0;
}

static int cpts_ptp_verify(struct ptp_clock_info *ptp, unsigned int pin,
                           enum ptp_pin_function func, unsigned int chan)
{
	/* Check number of pins */
	if (pin >= ptp->n_pins || !ptp->pin_config)
		return -EINVAL;

	/* Lock the channel */
	if (chan != ptp->pin_config[pin].chan)
		return -EINVAL;

	/* Check function */
	switch (func) {
		case PTP_PF_NONE:
		case PTP_PF_EXTTS:
        	break;
    	default:
		return -EINVAL;
	}

	return 0;
}

static void cpts_print_timer_value(unsigned long data)
{
    u32 ctrl;
    struct cpts_pin *pin = (struct cpts_pin*)data;

    mod_timer(&pin->extts_state.timer, jiffies + msecs_to_jiffies(1000));

    //ctrl = __omap_dm_timer_read(pin->timer, OMAP_TIMER_CAPTURE_REG, pin->timer->posted);
    //ctrl = omap_dm_timer_read_status(pin->timer);
    ctrl = readl_relaxed(pin->timer->irq_ena);

    pr_info("cpts: %s counter val %u\n", pin->ptp_pin->name, omap_dm_timer_read_counter(pin->timer));
}

static int cpts_start_external_timestamp(struct ptp_extts_request *req, struct cpts_pin *pin)
{
    u32 ctrl, mask = OMAP_TIMER_CTRL_TCM_BOTHEDGES;
    u32 edge = req->flags;

    memset(&pin->extts_state, 0, sizeof(pin->extts_state));

    omap_dm_timer_enable(pin->timer);

    // set timer pin to input and setup edge to capture
	if ((edge & PTP_RISING_EDGE) && (edge & PTP_FALLING_EDGE))
		mask = OMAP_TIMER_CTRL_TCM_BOTHEDGES;
	else if (edge & PTP_FALLING_EDGE)
		mask = OMAP_TIMER_CTRL_TCM_HIGHTOLOW;
	else if (edge & PTP_RISING_EDGE)
		mask = OMAP_TIMER_CTRL_TCM_LOWTOHIGH;
	ctrl = mask | OMAP_TIMER_CTRL_GPOCFG | 1 << 10;
	__omap_dm_timer_write(pin->timer, OMAP_TIMER_CTRL_REG, ctrl, pin->timer->posted);
    pin->timer->context.tclr = ctrl;
    omap_dm_timer_disable(pin->timer);

    pr_info("cpts: start extts timer ctrl %d\n", ctrl);

    // autoload to zero
    omap_dm_timer_set_load(pin->timer, 1, 0);

    // enable the capture interrupt and start the timer
    __omap_dm_timer_write_status(pin->timer, OMAP_TIMER_INT_CAPTURE);
    omap_dm_timer_set_int_enable(pin->timer, OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_OVERFLOW);
    omap_dm_timer_start(pin->timer);

    setup_timer(&pin->extts_state.timer, cpts_print_timer_value, (unsigned long)pin);
    mod_timer(&pin->extts_state.timer, jiffies + msecs_to_jiffies(1000));

    return 0;
}

static int cpts_start_periodic_output(struct ptp_perout_request *req, struct cpts_pin *pin)
{
    u32 ctrl;
    u32 load;
    u64 time_now;
    u32 cycles;
    struct cpts *cpts = container_of(pin, struct cpts, pins[pin->ptp_pin->index]);

    memset(&pin->perout_state, 0, sizeof(pin->perout_state));

    omap_dm_timer_enable(pin->timer);

    ctrl = OMAP_TIMER_CTRL_GPOCFG | OMAP_TIMER_CTRL_PT | 2 << 10;
	__omap_dm_timer_write(pin->timer, OMAP_TIMER_CTRL_REG, ctrl, pin->timer->posted);
    pin->timer->context.tclr = ctrl;

    // setup autoload so we overflow once per second.
    load = 0ul - pin->timer->rate;
    omap_dm_timer_set_load(pin->timer, 1, load);

    // setup an estimation of the time, the interrupt will get it more accurate before we enable the output.
    div64_u64_rem(timecounter_read(&cpts->tc), 1000000000, &time_now);
    cycles = (u32)div_u64(time_now * pin->timer->rate, 1000000000);
	__omap_dm_timer_write(pin->timer, OMAP_TIMER_COUNTER_REG, load + cycles, pin->timer->posted);

    omap_dm_timer_start(pin->timer);

    return 0;
}

static irqreturn_t cpts_timer_interrupt(int irq, void *data)
{
    struct cpts_pin *pin = data;

    u32 irq_status = omap_dm_timer_read_status(pin->timer);
    switch(pin->state.type)
    {
    case PTP_CLK_REQ_EXTTS:
        // overflow first so we don't overwrite the load value
        if(irq_status & OMAP_TIMER_INT_OVERFLOW)
        {
            schedule_work(&pin->overflow_work);
        }
        if(irq_status & OMAP_TIMER_INT_CAPTURE)
        {
            pin->extts_state.capture = __omap_dm_timer_read(pin->timer,
                    OMAP_TIMER_CAPTURE_REG, pin->timer->posted);
            schedule_work(&pin->capture_work);
        }
        break;
    case PTP_CLK_REQ_PEROUT:
    default:
        break;
    }
    // clear interrupts
    __omap_dm_timer_write_status(pin->timer, irq_status);

    return IRQ_HANDLED;
}

static s32 cpts_extts_deficit_avg(struct cpts_pin *pin)
{
    int i;
    s64 sum = 0;

    for(i = 0; i < CPTS_AVERAGE_LEN; ++i)
        sum += pin->extts_state.deficit[i] >= 0 ? pin->extts_state.deficit[i] : -pin->extts_state.deficit[i];

    return (s32)div_s64(sum, CPTS_AVERAGE_LEN);
}

static struct cpts_pin * cpts_get_pin(struct cpts *cpts, unsigned int index)
{
    // resolve the pin
    if (index >= CPTS_NUM_PINS)
        return NULL;
    return cpts->pins + index;
}

static int cpts_enable_pin(struct cpts_pin *pin)
{
	u32 ctrl, mask;
    struct cpts *cpts = container_of(pin, struct cpts, pins[pin->ptp_pin->index]);

    // request the timer for the pin
    if(!pin->timer)
        pin->timer = omap_dm_timer_request_by_node(pin->timerNode);
    if(!pin->timer)
        return -EINVAL;

    //tasklet_enable(&pin->capture_tasklet);
    //tasklet_enable(&pin->overflow_tasklet);
    // request the timer interrupt
    if (request_irq(omap_dm_timer_get_irq(pin->timer), cpts_timer_interrupt, IRQF_TIMER,
                pin->ptp_pin->name, pin))
        return -EIO;

    // enable the pin in the cpts peripheral
    switch (pin->ptp_pin->index) {
    case 0: mask = HW1_TS_PUSH_EN; break;
    case 1: mask = HW2_TS_PUSH_EN; break;
    case 2: mask = HW3_TS_PUSH_EN; break;
    case 3: mask = HW4_TS_PUSH_EN; break;
    default: return -EINVAL;
    }
    ctrl = cpts_read32(cpts, control);
    ctrl |= mask;
    cpts_write32(cpts, ctrl, control);

    // setup the timer clock source
    omap_dm_timer_set_source(pin->timer, OMAP_TIMER_SRC_SYS_CLK);
    pin->timer->rate = clk_get_rate(pin->timer->fclk);
    pr_info("cpts: timer rate: %lu Hz", pin->timer->rate);
    return 0;
}

static int cpts_disable_pin(struct cpts_pin *pin)
{
	u32 ctrl, mask;
    struct cpts *cpts = container_of(pin, struct cpts, pins[pin->ptp_pin->index]);

    // if ther is no timer then there is nothing to disable
    if (!pin->timer)
        return 0;

    free_irq(omap_dm_timer_get_irq(pin->timer), pin);
    //tasklet_disable(&pin->capture_tasklet);
    //tasklet_disable(&pin->overflow_tasklet);
    cancel_work_sync(&pin->capture_work);
    cancel_work_sync(&pin->overflow_work);
    omap_dm_timer_set_int_disable(pin->timer, OMAP_TIMER_INT_CAPTURE |
            OMAP_TIMER_INT_OVERFLOW | OMAP_TIMER_INT_MATCH);
    omap_dm_timer_enable(pin->timer);
    omap_dm_timer_stop(pin->timer);
    omap_dm_timer_free(pin->timer);
    pin->timer = NULL;

    del_timer(&pin->extts_state.timer);

    switch (pin->ptp_pin->index) {
    case 0: mask = HW1_TS_PUSH_EN; break;
    case 1: mask = HW2_TS_PUSH_EN; break;
    case 2: mask = HW3_TS_PUSH_EN; break;
    case 3: mask = HW4_TS_PUSH_EN; break;
    default: return -EINVAL;
    }
    ctrl = cpts_read32(cpts, control);
    ctrl &= ~mask;
    cpts_write32(cpts, ctrl, control);

    return 0;
}

static int cpts_ptp_enable(struct ptp_clock_info *ptp,
			   struct ptp_clock_request *rq, int on)
{
	int err = 0;
    struct cpts_pin *pin;
	struct cpts *cpts = container_of(ptp, struct cpts, info);
        pr_info("cpts: ptp_enable %d\n", rq->type);

	switch(rq->type) {
	case PTP_CLK_REQ_EXTTS:
        pr_info("cpts: request to setup extts on pin %d\n", rq->extts.index);
        pin = cpts_get_pin(cpts, rq->extts.index);
        if(!pin)
        {
            pr_warn("cpts: unable to get pin %d\n", rq->extts.index);
            return -EINVAL;
        }

        pr_info("cpts: check pin on %d\n", on);
        if (on)
        {
            pr_info("cpts: enable pin\n");
            err = cpts_enable_pin(pin);
            if (!err)
            {
            	pr_info("cpts: start pin\n");
                err = cpts_start_external_timestamp(&rq->extts, pin);
                if (!err)
                {
                    pr_info("cpts: start success %d\n", rq->extts.index);
                    pin->state = *rq;
                    return 0;
                }
                pr_warn("cpts: start failed %d\n", err);
            }
            pr_warn("cpts: enable failed %d\n", err);
        }

        cpts_disable_pin(pin);
        return err;
    case PTP_CLK_REQ_PEROUT:
        pin = cpts_get_pin(cpts, rq->perout.index);
        if(!pin)
            return -EINVAL;

        if (on)
        {
            err = cpts_enable_pin(pin);
            if (!err)
            {
                err = cpts_start_periodic_output(&rq->perout, pin);
                if (!err)
                {
                    pin->state = *rq;
                    return 0;
                }
            }
        }

        cpts_disable_pin(pin);
        return err;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static struct ptp_pin_desc cpts_pins[CPTS_NUM_PINS] = {
	{
		.name = "timer4",
		.index = 0,
		.func = PTP_PF_NONE,
		.chan = 0
	},
	{
		.name = "timer5",
		.index = 1,
		.func = PTP_PF_NONE,
		.chan = 1
	},
	{
		.name = "timer6",
		.index = 2,
		.func = PTP_PF_NONE,
		.chan = 2
	},
	{
		.name = "timer7",
		.index = 3,
		.func = PTP_PF_NONE,
		.chan = 3
	}
};

static struct ptp_clock_info cpts_info = {
	.owner		= THIS_MODULE,
	.name		= "CPTS timer",
	.max_adj	= 1000000,
	.n_alarm    	= 0,
	.n_ext_ts	= CPTS_NUM_PINS,
	.n_pins		= CPTS_NUM_PINS,
	.pps		= 0,
	.pin_config 	= cpts_pins,
	.adjfreq	= cpts_ptp_adjfreq,
	.adjtime	= cpts_ptp_adjtime,
	.gettime64	= cpts_ptp_gettime,
	.settime64	= cpts_ptp_settime,
	.enable		= cpts_ptp_enable,
	.verify     	= cpts_ptp_verify,
};

static void cpts_overflow_check(struct work_struct *work)
{
	u32 ctrl;
	struct timespec64 ts;
	struct cpts *cpts = container_of(work, struct cpts, overflow_work.work);

	ctrl = cpts_read32(cpts, control);
	ctrl |= CPTS_EN;
	cpts_write32(cpts, ctrl, control);
	cpts_write32(cpts, TS_PEND_EN, int_enable);
	cpts_ptp_gettime(&cpts->info, &ts);
	pr_info("cpts overflow check at %lld.%09lu\n", ts.tv_sec, ts.tv_nsec);
	schedule_delayed_work(&cpts->overflow_work, CPTS_OVERFLOW_PERIOD);
}

static void cpts_clk_init(struct device *dev, struct cpts *cpts)
{
	cpts->refclk = devm_clk_get(dev, "cpts");
	if (IS_ERR(cpts->refclk)) {
		dev_err(dev, "Failed to get cpts refclk\n");
		cpts->refclk = NULL;
		return;
	}
	clk_prepare_enable(cpts->refclk);
}

static void cpts_clk_release(struct cpts *cpts)
{
	clk_disable(cpts->refclk);
}

static int cpts_match(struct sk_buff *skb, unsigned int ptp_class,
		      u16 ts_seqid, u8 ts_msgtype)
{
	u16 *seqid;
	unsigned int offset = 0;
	u8 *msgtype, *data = skb->data;

	if (ptp_class & PTP_CLASS_VLAN)
		offset += VLAN_HLEN;

	switch (ptp_class & PTP_CLASS_PMASK) {
	case PTP_CLASS_IPV4:
		offset += ETH_HLEN + IPV4_HLEN(data + offset) + UDP_HLEN;
		break;
	case PTP_CLASS_IPV6:
		offset += ETH_HLEN + IP6_HLEN + UDP_HLEN;
		break;
	case PTP_CLASS_L2:
		offset += ETH_HLEN;
		break;
	default:
		return 0;
	}

	if (skb->len + ETH_HLEN < offset + OFF_PTP_SEQUENCE_ID + sizeof(*seqid))
		return 0;

	if (unlikely(ptp_class & PTP_CLASS_V1))
		msgtype = data + offset + OFF_PTP_CONTROL;
	else
		msgtype = data + offset;

	seqid = (u16 *)(data + offset + OFF_PTP_SEQUENCE_ID);

	return (ts_msgtype == (*msgtype & 0xf) && ts_seqid == ntohs(*seqid));
}

static u64 cpts_find_ts(struct cpts *cpts, struct sk_buff *skb, int ev_type)
{
	u64 ns = 0;
	struct cpts_event *event;
	struct list_head *this, *next;
	unsigned int class = ptp_classify_raw(skb);
	unsigned long flags;
	u16 seqid;
	u8 mtype;

	if (class == PTP_CLASS_NONE)
		return 0;

	spin_lock_irqsave(&cpts->lock, flags);
	cpts_fifo_read(cpts, CPTS_EV_PUSH);
	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_expired(event)) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			continue;
		}
		mtype = (event->high >> MESSAGE_TYPE_SHIFT) & MESSAGE_TYPE_MASK;
		seqid = (event->high >> SEQUENCE_ID_SHIFT) & SEQUENCE_ID_MASK;
		if (ev_type == event_type(event) &&
		    cpts_match(skb, class, seqid, mtype)) {
			ns = timecounter_cyc2time(&cpts->tc, event->low);
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			break;
		}
	}
	spin_unlock_irqrestore(&cpts->lock, flags);

	return ns;
}

void cpts_rx_timestamp(struct cpts *cpts, struct sk_buff *skb)
{
	u64 ns;
	struct skb_shared_hwtstamps *ssh;

	if (!cpts->rx_enable)
		return;
	ns = cpts_find_ts(cpts, skb, CPTS_EV_RX);
	if (!ns)
		return;
	ssh = skb_hwtstamps(skb);
	memset(ssh, 0, sizeof(*ssh));
	ssh->hwtstamp = ns_to_ktime(ns);
}

void cpts_tx_timestamp(struct cpts *cpts, struct sk_buff *skb)
{
	u64 ns;
	struct skb_shared_hwtstamps ssh;

	if (!(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS))
		return;
	ns = cpts_find_ts(cpts, skb, CPTS_EV_TX);
	if (!ns)
		return;
	memset(&ssh, 0, sizeof(ssh));
	ssh.hwtstamp = ns_to_ktime(ns);
	skb_tstamp_tx(skb, &ssh);
}

#endif /*CONFIG_TI_CPTS*/

int cpts_register(struct device *dev, struct cpts *cpts,
		  u32 mult, u32 shift)
{
#ifdef CONFIG_TI_CPTS
	int err, i;
	unsigned long flags;
        const char *prop_name;

	cpts->info = cpts_info;
	cpts->clock = ptp_clock_register(&cpts->info, dev);
	if (IS_ERR(cpts->clock)) {
		err = PTR_ERR(cpts->clock);
		cpts->clock = NULL;
		return err;
	}
	spin_lock_init(&cpts->lock);

	cpts->cc.read = cpts_systim_read;
	cpts->cc.mask = CLOCKSOURCE_MASK(32);
	cpts->cc_mult = mult;
	cpts->cc.mult = mult;
	cpts->cc.shift = shift;

	INIT_LIST_HEAD(&cpts->events);
	INIT_LIST_HEAD(&cpts->pool);
	for (i = 0; i < CPTS_MAX_EVENTS; i++)
		list_add(&cpts->pool_data[i].list, &cpts->pool);

    for (i = 0; i < CPTS_NUM_PINS; i++)
    {
        cpts->pins[i].ptp_pin = cpts->info.pin_config + i;
        cpts->pins[i].timerNode = of_find_node_by_name(NULL, "timer");
        while(cpts->pins[i].timerNode)
        {
            if(of_property_read_string(cpts->pins[i].timerNode, "ti,hwmods", &prop_name) == 0 &&
                    strcmp(prop_name, cpts->pins[i].ptp_pin->name) == 0)
                break;
            cpts->pins[i].timerNode = of_find_node_by_name(cpts->pins[i].timerNode, "timer");
        }

        if (!cpts->pins[i].timerNode)
            pr_warn("cpts: unable to find %s in device tree", cpts->pins[i].ptp_pin->name);
        cpts->pins[i].timer = NULL;
        //tasklet_init(&cpts->pins[i].capture_tasklet, cpts_pin_capture_bottom_end,
        //        (unsigned long)(cpts->pins + i));
        //tasklet_init(&cpts->pins[i].overflow_tasklet, cpts_pin_overflow_bottom_end,
        //        (unsigned long)(cpts->pins + i));
        INIT_WORK(&cpts->pins[i].capture_work, cpts_pin_capture_bottom_end);
        INIT_WORK(&cpts->pins[i].overflow_work, cpts_pin_overflow_bottom_end);
    }

	cpts_clk_init(dev, cpts);
	cpts_write32(cpts, CPTS_EN, control);
	cpts_write32(cpts, TS_PEND_EN, int_enable);

	spin_lock_irqsave(&cpts->lock, flags);
	timecounter_init(&cpts->tc, &cpts->cc, ktime_to_ns(ktime_get_real()));
	spin_unlock_irqrestore(&cpts->lock, flags);

	INIT_DELAYED_WORK(&cpts->overflow_work, cpts_overflow_check);
	schedule_delayed_work(&cpts->overflow_work, CPTS_OVERFLOW_PERIOD);

	cpts->phc_index = ptp_clock_index(cpts->clock);
#endif
	return 0;
}

void cpts_unregister(struct cpts *cpts)
{
#ifdef CONFIG_TI_CPTS
    int i;

    for (i = 0; i < CPTS_NUM_PINS; i++)
    {
        cpts_disable_pin(&cpts->pins[i]);
        if(cpts->pins[i].timer) omap_dm_timer_free(cpts->pins[i].timer);
        if(cpts->pins[i].timerNode) of_node_put(cpts->pins[i].timerNode);
    }

	if (cpts->clock) {
		ptp_clock_unregister(cpts->clock);
		cancel_delayed_work_sync(&cpts->overflow_work);
	}
	if (cpts->refclk)
		cpts_clk_release(cpts);
#endif
}
