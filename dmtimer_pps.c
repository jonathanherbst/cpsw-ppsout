/*
 * TI Common Platform Time Sync Hardware Pins
 *
 * Copyright (C) 2012 Richard Cochran <richardcochran@gmail.com>
 * Copyright (C) 2018 Jonathan Herbst <jonathan_herbst@lord.com>
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
#include <linux/module.h>
#include <linux/math64.h>

#include "cpts.h"
#include "cpts_pin.h"

#define TIMER_CLOCK_OFFSET  30
#define TIMER_CLOCK_MASK    0xC0000000

struct ptp_pin_desc cpts_pins[CPTS_NUM_PINS] = {
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

inline unsigned long cpts_pin_index(unsigned long index)
{
	return index & (CPTS_AVERAGE_LEN - 1);
}

inline u32 cpts_pin_ns_to_cycles(struct cpts_pin* pin, u32 period, u64 ns)
{
	return (u32)div64_u64(ns * period, 1000000000);
}

inline u32 cpts_pin_weighted_avg(u32 a, u32 a_weight, u32 b, u32 b_weight)
{
  return (u32)div64_u64((u64)a * a_weight + (u64)b * b_weight,
		 a_weight + b_weight);
}

static void cpts_pin_set_array(void* a, void* v, int v_size, int l)
{
	u8* ptr = a;
	for(; ptr < (u8*)a + v_size * l; ptr += v_size)
	  memcpy(ptr, v, v_size);
}

static s32 cpts_pin_extts_calculate_deficit(struct cpts_pin *pin)
{
	unsigned long index = pin->extts_state.index;
	return pin->extts_state.capture > (0ul - pin->extts_state.period / 2) ?
			-(s32)(0ul - pin->extts_state.capture) :
			(s32)(pin->extts_state.capture -
					pin->extts_state.load[index]);
}

static u32 cpts_pin_extts_calculate_period(struct cpts_pin *pin)
{
	unsigned long indexm1;
	unsigned long index = pin->extts_state.index;
	indexm1 = cpts_pin_index(index - 1);
	pr_info("period = 0 - %u + %d - %d", pin->extts_state.load[indexm1], pin->extts_state.deficit[index], pin->extts_state.deficit[indexm1]);
	return 0ul - pin->extts_state.load[indexm1] +
		pin->extts_state.deficit[index] -
		pin->extts_state.deficit[indexm1];
}

static u32 cpts_pin_extts_calculate_reload(struct cpts_pin *pin)
{
	unsigned long index = pin->extts_state.index;
	return 0ul - (u32)(2 * pin->extts_state.period -
			(0ul - pin->extts_state.load[index]) +
			pin->extts_state.deficit[index]);
}

static void cpts_pin_extts_set_reload(struct cpts_pin *pin)
{
	u32 period;
	unsigned long indexp1;
	unsigned long index = pin->extts_state.index;
	indexp1 = cpts_pin_index(index + 1);

	pin->extts_state.deficit[index] = cpts_pin_extts_calculate_deficit(pin);

	if (pin->extts_state.last_capture_valid) {
		period = cpts_pin_extts_calculate_period(pin);
		pin->extts_state.period = cpts_pin_weighted_avg(
				pin->extts_state.period, 3, period, 1);
	}
	else {
		cpts_pin_set_array(pin->extts_state.deficit,
				&pin->extts_state.deficit[index],
				sizeof(pin->extts_state.deficit[index]),
				CPTS_AVERAGE_LEN);
	}

	pin->extts_state.load[indexp1] = cpts_pin_extts_calculate_reload(pin);
	
	if (pin->extts_state.deficit[index] >= 0) {
		__omap_dm_timer_write(pin->timer, OMAP_TIMER_LOAD_REG,
				pin->extts_state.load[indexp1],
				pin->timer->posted);
		pin->timer->context.tldr = pin->extts_state.load[indexp1];
	}

	pin->extts_state.index = indexp1;
}

static u32 cpts_pin_perout_calculate_period(struct cpts_pin *pin)
{
	unsigned long indexm1 = cpts_pin_index(pin->perout_state.index - 1);
	pr_info("cpts: perout calc period, load %u, cap %llu, last cap %llu\n", pin->perout_state.load[indexm1], pin->perout_state.capture, pin->perout_state.last_capture);
	return (u32)div64_u64((u64)(0ul - pin->perout_state.load[indexm1]) *
			1000000000, pin->perout_state.capture -
			pin->perout_state.last_capture);
}

static s32 cpts_pin_perout_calculate_deficit(struct cpts_pin *pin)
{
	u64 tmp;
	div64_u64_rem(pin->perout_state.capture, 1000000000, &tmp);
	tmp = cpts_pin_ns_to_cycles(pin, pin->perout_state.period, tmp);
	return tmp > pin->perout_state.period / 2 ?
		-(s32)(pin->perout_state.period - tmp) : (s32)tmp ;
}

static u32 cpts_pin_perout_calculate_reload(struct cpts_pin *pin)
{
	unsigned long index = pin->perout_state.index;
	return 0ul - (u32)(2 * pin->perout_state.period -
			(0ul - pin->perout_state.load[index]) -
			pin->perout_state.deficit[index]);
}

static s32 cpts_pin_deficit_avg(const s32 *deficit)
{
	int i;
	s64 sum = 0;

	for (i = 0; i < CPTS_AVERAGE_LEN; ++i)
		sum += deficit[i] >= 0 ? deficit[i] : -deficit[i];

	return (s32)div64_s64(sum, CPTS_AVERAGE_LEN);
}

inline u64 cpts_pin_est_ns(struct cpts_pin* pin, u32 cycles)
{
	return div64_u64((u64)cycles * 1000000000, pin->timer->rate);
}

inline u32 cpts_pin_est_cyc(struct cpts_pin *pin, u64 ns)
{
	return (u32)div64_u64(ns * pin->timer->rate, 1000000000);
}

static void cpts_pin_perout_set_reload(struct cpts_pin* pin)
{
	unsigned long index;
	unsigned long indexp1;
	u32 period;

	index = pin->perout_state.index;
	indexp1 = cpts_pin_index(index + 1);

	period = cpts_pin_perout_calculate_period(pin);
	pr_info("cpts: perout period %u\n", period);
	pin->perout_state.period = cpts_pin_weighted_avg(
			pin->perout_state.period, 3, period, 1);

	pin->perout_state.deficit[index] =
		cpts_pin_perout_calculate_deficit(pin);
	if (!pin->perout_state.deficit_valid) {
		cpts_pin_set_array(pin->perout_state.deficit,
				&pin->perout_state.deficit[index],
				sizeof(pin->perout_state.deficit[index]),
				CPTS_AVERAGE_LEN);
		pin->perout_state.deficit_valid = true;
	}
	pr_info("cpts: perout reload calc period %u, deficit %d, load %u\n", pin->perout_state.period, pin->perout_state.deficit[index], pin->perout_state.load[index]);
	pin->perout_state.load[indexp1] = cpts_pin_perout_calculate_reload(pin);
	pr_info("cpts: perout reload %u\n", pin->perout_state.load[indexp1]);

	__omap_dm_timer_write(pin->timer, OMAP_TIMER_LOAD_REG,
			pin->perout_state.load[indexp1], pin->timer->posted);
	pin->timer->context.tldr = pin->perout_state.load[indexp1];
	
	pin->perout_state.index = indexp1;
}

static void cpts_pin_capture_bottom_half(struct work_struct *work)
{
	unsigned long indexp1;
	struct cpts_pin *pin = container_of(work, struct cpts_pin,
			capture_work);

	pr_info("cpts: capture hit");

	switch(pin->state.type)
	{
	case PTP_CLK_REQ_EXTTS:

		if (pin->extts_state.last_capture_valid &&
				pin->extts_state.period == 0) {
			pin->extts_state.period = pin->extts_state.capture -
				pin->extts_state.last_capture;
			pin->extts_state.load[0] = 
				0ul - pin->extts_state.period;

			// write load and load it.
			__omap_dm_timer_write(pin->timer, OMAP_TIMER_LOAD_REG,
					pin->extts_state.load[0],
					pin->timer->posted);
			omap_dm_timer_trigger(pin->timer);
			pin->timer->context.tldr = pin->extts_state.load[0];

			pr_info("cpts: capture found pulse, %u %u",
					pin->extts_state.period,
					pin->extts_state.load[0]);

			cpts_pin_set_array(pin->extts_state.load,
					&pin->extts_state.load[0],
					sizeof(pin->extts_state.load[0]),
					CPTS_AVERAGE_LEN);
			cpts_pin_set_array(pin->extts_state.deficit,
					&pin->extts_state.load[0],
					sizeof(pin->extts_state.load[0]),
					CPTS_AVERAGE_LEN);

			pin->extts_state.last_capture_valid = false;
			break;
		} else if (pin->extts_state.period != 0) {
			indexp1 = cpts_pin_index(pin->extts_state.index + 1);
			cpts_pin_extts_set_reload(pin);
			pr_info("cpts: capture %s load value %u deficit %d period %u",
					pin->ptp_pin->name,
					pin->extts_state.load[indexp1],
					pin->extts_state.deficit[cpts_pin_index(indexp1 - 1)],
					pin->extts_state.period);
		}
		pin->extts_state.last_capture = pin->extts_state.capture;
		pin->extts_state.last_capture_valid = true;
		break;
	case PTP_CLK_REQ_PEROUT:
	default:
		break;
	}
}

static void cpts_pin_overflow_bottom_half(struct work_struct *work)
{
	u32 ctrl;
	s32 avg;
	u32 match;
	struct cpts_pin *pin = container_of(work, struct cpts_pin,
			overflow_work);
	struct cpts *cpts = container_of(pin, struct cpts,
			pins[pin->ptp_pin->index]);

	// read all events
	cpts_fifo_read(cpts, -1);

	pr_info("cpts: overflow hit");
	ctrl = __omap_dm_timer_read(pin->timer,
		OMAP_TIMER_CTRL_REG, pin->timer->posted);
	pr_info("cpts: ctrl %u\n", ctrl);

	switch(pin->state.type)
	{
	case PTP_CLK_REQ_EXTTS:
		if (pin->extts_state.period != 0)
		{
			__omap_dm_timer_write(pin->timer, OMAP_TIMER_LOAD_REG,
				pin->extts_state.load[pin->extts_state.index],
				pin->timer->posted);
			pin->timer->context.tldr =
				pin->extts_state.load[pin->extts_state.index];
			
			ctrl = __omap_dm_timer_read(pin->timer,
				OMAP_TIMER_CTRL_REG, pin->timer->posted);

			avg = cpts_pin_deficit_avg(pin->extts_state.deficit);
			pr_info("cpts: overflow deficit avg %d", avg);
			if (!(ctrl & OMAP_TIMER_CTRL_CE) &&
					cpts_pin_est_ns(pin, avg) < 500) {
				pr_info("cpts: started timer output, avg %d",
						avg);
				ctrl |= OMAP_TIMER_CTRL_CE;
				__omap_dm_timer_write(pin->timer,
					OMAP_TIMER_CTRL_REG, ctrl,
					pin->timer->posted);
				pin->timer->context.tclr = ctrl;
			}
		}
		break;
	case PTP_CLK_REQ_PEROUT:
		if (pin->perout_state.capture_valid) {
			pin->perout_state.capture_valid = false;

			match = pin->perout_state.load[pin->perout_state.index] +
				pin->perout_state.period / 2;
			__omap_dm_timer_write(pin->timer, OMAP_TIMER_MATCH_REG,
				match, pin->timer->posted);
			pin->timer->context.tmar = match;
			if (pin->perout_state.last_capture_valid)
				cpts_pin_perout_set_reload(pin);

			pin->perout_state.last_capture =
				pin->perout_state.capture;
			pin->perout_state.last_capture_valid = true;

			ctrl = __omap_dm_timer_read(pin->timer,
				OMAP_TIMER_CTRL_REG, pin->timer->posted);
			pr_info("cpts: peroit ctrl %u\n", ctrl);

			avg = cpts_pin_deficit_avg(pin->perout_state.deficit);
			pr_info("cpts: perout overflow deficit avg %d\n", avg);
			/*if (ctrl & OMAP_TIMER_CTRL_GPOCFG &&
					cpts_pin_est_ns(pin, avg) < 500) {
				ctrl &= ~OMAP_TIMER_CTRL_GPOCFG;
				pr_info("cpts: peroit ctrl %u\n", ctrl);
				__omap_dm_timer_write(pin->timer,
					OMAP_TIMER_CTRL_REG, ctrl,
					pin->timer->posted);
				pin->timer->context.tclr = ctrl;
			}*/
		}
		break;
	default:
		break;
	}
}

static int cpts_pin_start_external_timestamp(struct ptp_extts_request *req,
		struct cpts_pin *pin)
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
	__omap_dm_timer_write(pin->timer, OMAP_TIMER_CTRL_REG, ctrl,
			pin->timer->posted);
	pin->timer->context.tclr = ctrl;
	omap_dm_timer_disable(pin->timer);

	pr_info("cpts: start extts timer ctrl %d\n", ctrl);

	// autoload to zero
	omap_dm_timer_set_load(pin->timer, 1, 0);

	// enable the capture interrupt and start the timer
	__omap_dm_timer_write_status(pin->timer, OMAP_TIMER_INT_CAPTURE);
	omap_dm_timer_set_int_enable(pin->timer,
			OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_OVERFLOW);
	omap_dm_timer_start(pin->timer);

	return 0;
}

static int cpts_pin_start_periodic_output(struct ptp_perout_request *req,
		struct cpts_pin *pin)
{
	u32 ctrl;
	u32 load;
	u64 time_now;
	u32 cycles;
	struct cpts *cpts = container_of(pin, struct cpts,
			pins[pin->ptp_pin->index]);

	memset(&pin->perout_state, 0, sizeof(pin->perout_state));

	omap_dm_timer_enable(pin->timer);

	//ctrl = OMAP_TIMER_CTRL_GPOCFG | OMAP_TIMER_CTRL_PT | 2 << 10;
	ctrl = 1 << 10;
	__omap_dm_timer_write(pin->timer, OMAP_TIMER_CTRL_REG, ctrl,
			pin->timer->posted);
	pin->timer->context.tclr = ctrl;

	// setup autoload so we overflow once per second.
	load = 0ul - pin->timer->rate;
	omap_dm_timer_set_load(pin->timer, 1, load);
	pr_info("cpts: perout set load %u\n", load);

	pin->perout_state.period = pin->timer->rate;
	cpts_pin_set_array(pin->perout_state.load, &load, sizeof(load),
			CPTS_AVERAGE_LEN);

	omap_dm_timer_set_int_enable(pin->timer, OMAP_TIMER_INT_OVERFLOW);
	omap_dm_timer_start(pin->timer);

	// setup an estimation of the time, the interrupt will get it more
	// accurate before we enable the output.
	div64_u64_rem(timecounter_read(&cpts->tc), 1000000000, &time_now);
	cycles = cpts_pin_est_cyc(pin, time_now);
	omap_dm_timer_write_counter(pin->timer, load + cycles);
	pr_info("cpts: perout cycles left %u\n", cycles);

	return 0;
}

static irqreturn_t cpts_pin_interrupt(int irq, void *data)
{
	struct cpts_pin *pin = data;

	u32 irq_status = omap_dm_timer_read_status(pin->timer);
	switch(pin->state.type)
	{
	case PTP_CLK_REQ_EXTTS:
		// overflow first so we don't overwrite the load value
		if (irq_status & OMAP_TIMER_INT_OVERFLOW) {
			schedule_work(&pin->overflow_work);
		}
		if (irq_status & OMAP_TIMER_INT_CAPTURE) {
			pin->extts_state.capture = __omap_dm_timer_read(
					pin->timer, OMAP_TIMER_CAPTURE_REG,
					pin->timer->posted);
			schedule_work(&pin->capture_work);
		}
		break;
	case PTP_CLK_REQ_PEROUT:
		// overflow first so we don't overwrite the load value
		if (irq_status & OMAP_TIMER_INT_OVERFLOW) {
			schedule_work(&pin->overflow_work);
		}
	default:
		break;
	}
	// clear interrupts
	__omap_dm_timer_write_status(pin->timer, irq_status);

	return IRQ_HANDLED;
}

static struct cpts_pin * cpts_get_pin(struct cpts *cpts, unsigned int index)
{
	// resolve the pin
	if (index >= CPTS_NUM_PINS)
		return NULL;
	return cpts->pins + index;
}

static int cpts_pin_enable(struct cpts_pin *pin, int timer_source)
{
	int err;
	struct cpts *cpts = container_of(pin, struct cpts,
			pins[pin->ptp_pin->index]);

	// request the timer for the pin
	if (!pin->timer)
		pin->timer = omap_dm_timer_request_by_node(pin->timerNode);
	if (!pin->timer)
		return -EINVAL;

	if (request_irq(omap_dm_timer_get_irq(pin->timer), cpts_pin_interrupt,
				IRQF_TIMER, pin->ptp_pin->name, pin))
		return -EIO;

	err = cpts_set_hardware_push(cpts, pin->ptp_pin->index, true);
	if (err)
		return err;

	// setup the timer clock source
	omap_dm_timer_set_source(pin->timer, timer_source);
	pin->timer->rate = clk_get_rate(pin->timer->fclk);
	pr_info("cpts: timer rate: %lu Hz", pin->timer->rate);
	return 0;
}

static int cpts_pin_disable(struct cpts_pin *pin)
{
	int err;
	struct cpts *cpts = container_of(pin, struct cpts,
			pins[pin->ptp_pin->index]);

	if (!pin->timer)
		return 0;

	free_irq(omap_dm_timer_get_irq(pin->timer), pin);
	cancel_work_sync(&pin->capture_work);
	cancel_work_sync(&pin->overflow_work);
	omap_dm_timer_set_int_disable(pin->timer, OMAP_TIMER_INT_CAPTURE |
			OMAP_TIMER_INT_OVERFLOW | OMAP_TIMER_INT_MATCH);
	omap_dm_timer_enable(pin->timer);
	omap_dm_timer_stop(pin->timer);
	omap_dm_timer_free(pin->timer);
	pin->timer = NULL;

	err = cpts_set_hardware_push(cpts, pin->ptp_pin->index, false);
	if (err)
		return err;

	return 0;
}

int cpts_pin_ptp_enable(struct ptp_clock_info *ptp,
			struct ptp_clock_request *rq, int on)
{
	int err = 0;
    int timer_source;
	struct cpts_pin *pin;
	struct cpts *cpts = container_of(ptp, struct cpts, info);
	pr_info("cpts: ptp_enable %d\n", rq->type);

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		pr_info("cpts: request to setup extts on pin %d\n",
				rq->extts.index);
		pin = cpts_get_pin(cpts, rq->extts.index);
		if (!pin) {
			pr_warn("cpts: unable to get pin %d\n",
					rq->extts.index);
			return -EINVAL;
		}

		cpts_pin_disable(pin);

		pr_info("cpts: check pin on %d\n", on);
		if (!on)
			return 0;
		
		pr_info("cpts: enable pin\n");
		timer_source = (rq->extts.flags & TIMER_CLOCK_MASK) >>
			TIMER_CLOCK_OFFSET;
		err = cpts_pin_enable(pin, timer_source);
		if (err)
			goto extts_error;

		pr_info("cpts: start pin\n");
		err = cpts_pin_start_external_timestamp(&rq->extts, pin);
		if (err)
			goto extts_error;

		pr_info("cpts: start success %d\n", rq->extts.index);
		pin->state = *rq;
		return 0;

extts_error:
		cpts_pin_disable(pin);
		return err;

	case PTP_CLK_REQ_PEROUT:
		pin = cpts_get_pin(cpts, rq->perout.index);
		if (!pin)
			return -EINVAL;

		cpts_pin_disable(pin);

		pr_info("cpts: check pin on %d\n", on);
		if (!on)
			return 0;

		timer_source = (rq->extts.flags & TIMER_CLOCK_MASK) >>
			TIMER_CLOCK_OFFSET;
		err = cpts_pin_enable(pin, timer_source);
		if (err)
			goto perout_error;

		err = cpts_pin_start_periodic_output(&rq->perout, pin);
		if (err)
			goto perout_error;
		
		pr_info("cpts: start success %d\n", rq->perout.index);
		pin->state = *rq;
		return 0;

perout_error:
		pr_info("cpts: error %d\n", err);
		cpts_pin_disable(pin);
		return err;

	default:
		break;
	}
	return -EOPNOTSUPP;
}

int cpts_pin_ptp_verify(struct ptp_clock_info *ptp, unsigned int pin,
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
		case PTP_PF_PEROUT:
        	break;
    	default:
		return -EINVAL;
	}

	return 0;
}

static struct device_node * cpts_pin_find_timer_by_name(
		struct device_node *node, const char* name)
{
	const char* prop_value;
	struct device_node *cursor;

	cursor = of_find_node_by_name(NULL, "timer");
	while (cursor) {
		if (of_property_read_string(cursor, "ti,hwmods", &prop_value) ==
				0 && strcmp(prop_value, name) == 0)
			break;
		cursor = of_find_node_by_name(cursor, "timer");
	}

	return cursor;
}

static cycle_t cpts_pin_systim_read(const struct cyclecounter *cc)
{
	struct cpts_pin_info *pin = container_of(cc, struct cpts_pin_info, cc);
	struct cpts *cpts = container_of(pin, struct cpts, pins_info);

	cpts_systim_read_anycc(cpts, cc);
}

static int cpts_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u64 adj;
	u32 diff, mult;
	int neg_adj = 0;
	unsigned long flags;
	struct cpts_pins_info *pins = container_of(ptp, struct cpts_pins_info,
			ptp_info);
	struct cpts *cpts = container_of(pins, struct cpts, pins_info);

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}
	mult = pins->cc_mult;
	adj = mult;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);

	spin_lock_irqsave(&cpts->lock, flags);

	timecounter_read(&pins->tc);

	pins->cc.mult = neg_adj ? mult - diff : mult + diff;

	spin_unlock_irqrestore(&cpts->lock, flags);

	return 0;
}

static int cpts_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	unsigned long flags;
	struct cpts_pins_info *pins = container_of(ptp, struct cpts_pins_info,
			ptp_info);
	struct cpts *cpts = container_of(pins, struct cpts, pins_info);

	spin_lock_irqsave(&cpts->lock, flags);
	timecounter_adjtime(&pins->tc, delta);
	spin_unlock_irqrestore(&cpts->lock, flags);

	return 0;
}

static int cpts_ptp_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	u64 ns;
	unsigned long flags;
	struct cpts_pins_info *pins = container_of(ptp, struct cpts_pins_info,
			ptp_info);
	struct cpts *cpts = container_of(pins, struct cpts, pins_info);

	spin_lock_irqsave(&cpts->lock, flags);
	ns = timecounter_read(&pins->tc);
	spin_unlock_irqrestore(&cpts->lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

static int cpts_ptp_settime(struct ptp_clock_info *ptp,
			    const struct timespec64 *ts)
{
	u64 ns;
	unsigned long flags;
	struct cpts_pins_info *pins = container_of(ptp, struct cpts_pins_info,
			ptp_info);
	struct cpts *cpts = container_of(pins, struct cpts, pins_info);

	ns = timespec64_to_ns(ts);

	spin_lock_irqsave(&cpts->lock, flags);
	timecounter_init(&pins->tc, &pins->cc, ns);
	spin_unlock_irqrestore(&cpts->lock, flags);

	return 0;
}

static int dmtimer_pps_probe_dt(struct dmtimer_pps *dmtpps,
		struct platform_device *pdev)
{

}

static int dmtimer_pps_probe(struct platform_device *pdev)
{
	struct dmtimer_pps *dmtpps;

	dmtpps = devm_kzalloc(&pdev->dev, sizeof(struct dmtimer_pps),
			GFP_KERNEL);
	if(!dmtpps)
		return -ENOMEM;

	memset(dmtpps, 0, sizeof(struct dmtimer_pps));

	dmtpps->dev = &pdev->dev;

	dmtimer_pps_probe_dt(dmtpps, pdev);
}

int cpts_pin_register(struct cpts *cpts)
{
	int i;

	struct cpts_pin_info* pins = &cpts->pins_info;

	pins->cc.read = cpts_pin_systim_read;
	pins->cc.mask = CLOCKSOURCE_MASK(32);
	pins->cc_mult = mult;
	pins->cc.mult = mult;
	pins->cc.shift = shift;

	spin_lock_irqsave(&cpts->lock, flags);
	timecounter_init(&pins->tc, &pins->cc, ktime_to_ns(ktime_get_real()));
	spin_unlock_irqrestore(&cpts->lock, flags);

	for (i = 0; i < CPTS_NUM_PINS; i++)
	{
		pins->pins[i].ptp_pin = pins->ptp_info.pin_config + i;
		pins->pins[i].timerNode = cpts_pin_find_timer_by_name(NULL,
				pins->pins[i].ptp_pin->name);
		if (!pins->pins[i].timerNode)
			pr_warn("cpts: unable to find %s in device tree",
					pins->pins[i].ptp_pin->name);
		pins->pins[i].timer = NULL;
		INIT_WORK(&pins->pins[i].capture_work,
				cpts_pin_capture_bottom_half);
		INIT_WORK(&pins->pins[i].overflow_work,
				cpts_pin_overflow_bottom_half);
	}

	pins->phc_index = ptp_clock_index(pins->ptp_clock);

	return 0;
}

void cpts_pin_unregister(struct cpts *cpts)
{
	int i;

	for (i = 0; i < CPTS_NUM_PINS; i++)
	{
		cpts_pin_disable(&cpts->pins[i]);
		if(cpts->pins[i].timer)
			omap_dm_timer_free(cpts->pins[i].timer);
		if(cpts->pins[i].timerNode)
			of_node_put(cpts->pins[i].timerNode);
	}
}
