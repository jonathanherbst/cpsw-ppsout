/*
 * Timer for 
 *
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
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/timekeeping.h>
#include <linux/pps_kernel.h>
#include <linux/clk.h>
#include <linux/sysfs.h>

#include <arch/arm/plat-omap/include/plat/dmtimer.h>

#define DMTIMER_PPS_NAME 	"dmtimer-pps"
#define TIMER_CLOCK_OFFSET  	30
#define TIMER_CLOCK_MASK    	0xC0000000
#define CPTS_AVERAGE_LEN 	4  // needs to be power of 2

struct dmtimer_pps_input_state {
	volatile bool new_capture;
	volatile bool new_overflow;
	volatile u32 capture;
	u32 last_capture;
	bool last_capture_valid;
	u32 period;
	unsigned long index;
	u32 load[CPTS_AVERAGE_LEN];
	s32 deficit[CPTS_AVERAGE_LEN];
};

struct dmtimer_pps_output_state {
	u64 capture;
	u64 last_capture;
	bool last_capture_valid;
	u32 period;
	unsigned long index;
	u32 load[CPTS_AVERAGE_LEN];
	s32 deficit[CPTS_AVERAGE_LEN];
	bool deficit_valid;
};

struct dmtimer_pps_settings {
	s32 clock_source;
	s32 pps_mode;
	bool generate;
};

struct dmtimer_pps {
	const struct device *dev;
	int supported_pps_mode;
	struct pps_device *pps;
	struct pps_source_info info;
	struct omap_dm_timer *timer;
	struct mutex timer_mutex;
	struct work_struct input_work;
	struct dmtimer_pps_settings settings;
	union {
		struct dmtimer_pps_input_state input_state;
		struct dmtimer_pps_output_state output_state;
	};
};

inline unsigned long dmtimer_pps_index(unsigned long index)
{
	return index & (CPTS_AVERAGE_LEN - 1);
}

inline u32 dmtimer_pps_ns_to_cycles(u32 period, u64 ns)
{
	return (u32)div64_u64(ns * period, 1000000000);
}

inline u32 dmtimer_pps_weighted_avg(u32 a, u32 a_weight, u32 b, u32 b_weight)
{
  return (u32)div64_u64((u64)a * a_weight + (u64)b * b_weight,
		 a_weight + b_weight);
}

static void dmtimer_pps_set_array(void* a, void* v, int v_size, int l)
{
	u8* ptr = a;
	for(; ptr < (u8*)a + v_size * l; ptr += v_size)
	  memcpy(ptr, v, v_size);
}

static s32 dmtimer_pps_deficit_avg(const s32 *deficit)
{
	int i;
	s64 sum = 0;

	for (i = 0; i < CPTS_AVERAGE_LEN; ++i)
		sum += deficit[i] >= 0 ? deficit[i] : -deficit[i];

	return (s32)div64_s64(sum, CPTS_AVERAGE_LEN);
}

inline u64 dmtimer_pps_est_ns(struct dmtimer_pps* dmtpps, u32 cycles)
{
	return div64_u64((u64)cycles * 1000000000, dmtpps->timer->rate);
}

inline u32 dmtimer_pps_est_cyc(struct dmtimer_pps *dmtpps, u64 ns)
{
	return (u32)div64_u64(ns * dmtpps->timer->rate, 1000000000);
}

inline s64 dmtimer_pps_ktime_now(void)
{
	struct system_time_snapshot snap;

	ktime_get_snapshot(&snap);
	return snap.real.tv64;
}

static s32 dmtimer_pps_input_calculate_deficit(
		struct dmtimer_pps_input_state *state)
{
	unsigned long index = state->index;
	return state->capture > (0ul - state->period / 2) ?
			-(s32)(0ul - state->capture) :
			(s32)(state->capture - state->load[index]);
}

static u32 dmtimer_pps_input_calculate_period(
		struct dmtimer_pps_input_state *state)
{
	unsigned long indexm1;
	unsigned long index = state->index;
	indexm1 = dmtimer_pps_index(index - 1);
	return 0ul - state->load[indexm1] + state->deficit[index] -
		state->deficit[indexm1];
}

static u32 dmtimer_pps_input_calculate_reload(
		struct dmtimer_pps_input_state *state)
{
	u32 period;
	unsigned long indexp1;
	unsigned long index = state->index;
	indexp1 = dmtimer_pps_index(index + 1);

	state->deficit[index] = dmtimer_pps_input_calculate_deficit(state);

	if (state->last_capture_valid) {
		period = dmtimer_pps_input_calculate_period(state);
		state->period = dmtimer_pps_weighted_avg(state->period, 3,
				period, 1);
	}
	else {
		dmtimer_pps_set_array(state->deficit, &state->deficit[index],
				sizeof(state->deficit[index]),
				CPTS_AVERAGE_LEN);
	}

	state->load[indexp1] = 0ul - (u32)(2 * state->period -
			(0ul - state->load[index]) + state->deficit[index]);


	return state->load[indexp1];
}

static u32 dmtimer_pps_output_calculate_period(
		struct dmtimer_pps_output_state *state)
{
	unsigned long indexm1 = dmtimer_pps_index(state->index - 1);
	return (u32)div64_u64((u64)(0ul - state->load[indexm1]) * 1000000000,
			state->capture - state->last_capture);
}

static s32 dmtimer_pps_output_calculate_deficit(
		struct dmtimer_pps_output_state *state)
{
	u64 tmp;
	div64_u64_rem(state->capture, 1000000000, &tmp);
	tmp = dmtimer_pps_ns_to_cycles(state->period, tmp);
	return tmp > state->period / 2 ? -(s32)(state->period - tmp) :
		(s32)tmp;
}

static u32 dmtimer_pps_output_calculate_reload(
		struct dmtimer_pps_output_state *state)
{
	unsigned long index;
	unsigned long indexp1;
	u32 period;

	index = state->index;
	indexp1 = dmtimer_pps_index(index + 1);

	period = dmtimer_pps_output_calculate_period(state);
	state->period = dmtimer_pps_weighted_avg(state->period, 3, period, 1);

	state->deficit[index] =	dmtimer_pps_output_calculate_deficit(state);
	if (!state->deficit_valid) {
		dmtimer_pps_set_array(state->deficit, &state->deficit[index],
				sizeof(state->deficit[index]),
				CPTS_AVERAGE_LEN);
		state->deficit_valid = true;
	}
	state->load[indexp1] = 0ul - (u32)(2 * state->period -
			(0ul - state->load[index]) - state->deficit[index]);

	return state->load[indexp1];
}

static irqreturn_t dmtimer_pps_interrupt(int irq, void *data)
{
	struct pps_event_time pps_time;
	struct dmtimer_pps *dmtpps = data;

	u32 irq_status = omap_dm_timer_read_status(dmtpps->timer);

	if (!dmtpps->settings.generate)
	{
		// overflow first so we don't overwrite the load value
		if (irq_status & OMAP_TIMER_INT_OVERFLOW) {
			dmtpps->input_state.new_overflow = true;
		}
		if (irq_status & OMAP_TIMER_INT_CAPTURE) {
			pps_get_ts(&pps_time);
			pps_event(dmtpps->pps, &pps_time,
					dmtpps->info.mode & PPS_CAPTUREBOTH,
					NULL);
			dmtpps->input_state.capture = __omap_dm_timer_read(
					dmtpps->timer, OMAP_TIMER_CAPTURE_REG,
					dmtpps->timer->posted);
			dmtpps->input_state.new_capture = true;
		}
		schedule_work(&dmtpps->input_work);
	} else {
		if (irq_status & OMAP_TIMER_INT_OVERFLOW) {
			pps_get_ts(&pps_time);
			pps_event(dmtpps->pps, &pps_time,
					dmtpps->info.mode & PPS_CAPTUREBOTH,
					NULL);
		}
	}
	// clear interrupts
	__omap_dm_timer_write_status(dmtpps->timer, irq_status);

	return IRQ_HANDLED;
}

static void dmtimer_pps_input_capture(struct dmtimer_pps *dmtpps)
{
	unsigned long index;
	unsigned long indexp1;
	u32 reload;
	struct dmtimer_pps_input_state *state = &dmtpps->input_state;

	if (state->last_capture_valid && state->period == 0) {
		state->period = state->capture - state->last_capture;
		state->load[0] = 0ul - state->period;

		// write load and load it.
		mutex_lock(&dmtpps->timer_mutex);
		__omap_dm_timer_write(dmtpps->timer, OMAP_TIMER_LOAD_REG,
				state->load[0], dmtpps->timer->posted);
		omap_dm_timer_trigger(dmtpps->timer);
		mutex_unlock(&dmtpps->timer_mutex);
		dmtpps->timer->context.tldr = state->load[0];

		dev_info(dmtpps->dev, "capture found pulse, %u %u",
				state->period, state->load[0]);

		dmtimer_pps_set_array(state->load, &state->load[0],
				sizeof(state->load[0]), CPTS_AVERAGE_LEN);
		dmtimer_pps_set_array(state->deficit, &state->load[0],
				sizeof(state->load[0]), CPTS_AVERAGE_LEN);

		state->last_capture_valid = false;
		return;
	} else if (state->period != 0) {
		index = state->index;
		indexp1 = dmtimer_pps_index(index + 1);
		reload = dmtimer_pps_input_calculate_reload(state);
		// if the capture happened after overflow it's okay set the new
		// load value here
		if (state->deficit[index] >= 0) {
			mutex_lock(&dmtpps->timer_mutex);
			__omap_dm_timer_write(dmtpps->timer,
					OMAP_TIMER_LOAD_REG,
					reload,
					dmtpps->timer->posted);
			dmtpps->timer->context.tldr = state->load[indexp1];
			mutex_unlock(&dmtpps->timer_mutex);
		}
		state->index = indexp1;
		dev_info(dmtpps->dev,
				"input capture load value %u deficit %d period %u",
				state->load[indexp1],
				state->deficit[index],
				state->period);
	}
	state->last_capture = state->capture;
	state->last_capture_valid = true;
}

static void dmtimer_pps_input_overflow(struct dmtimer_pps *dmtpps)
{
	u32 ctrl;
	s32 avg;
	bool start;
	struct dmtimer_pps_input_state *state = &dmtpps->input_state;

	if (state->period != 0) {
		mutex_lock(&dmtpps->timer_mutex);
		__omap_dm_timer_write(dmtpps->timer, OMAP_TIMER_LOAD_REG,
				state->load[state->index],
				dmtpps->timer->posted);
		dmtpps->timer->context.tldr = state->load[state->index];
		
		ctrl = __omap_dm_timer_read(dmtpps->timer,
			OMAP_TIMER_CTRL_REG, dmtpps->timer->posted);

		avg = dmtimer_pps_deficit_avg(state->deficit);
		start = !(ctrl & OMAP_TIMER_CTRL_CE) &&
			dmtimer_pps_est_ns(dmtpps, avg) < 500;
		if (start) {
			ctrl |= OMAP_TIMER_CTRL_CE;
			__omap_dm_timer_write(dmtpps->timer,
				OMAP_TIMER_CTRL_REG, ctrl,
				dmtpps->timer->posted);
			dmtpps->timer->context.tclr = ctrl;
		}
		mutex_unlock(&dmtpps->timer_mutex);

		dev_info(dmtpps->dev, "overflow deficit avg %d, start %d", avg,
				start);
	}
}

static void dmtimer_pps_input_work(struct work_struct *work)
{

	struct dmtimer_pps_input_state *state;
	struct dmtimer_pps *dmtpps = container_of(work, struct dmtimer_pps,
			input_work);
	state = &dmtpps->input_state;

	dev_info(dmtpps->dev, "capture hit");

	if(state->new_capture) {
		state->new_capture = false;
		dmtimer_pps_input_capture(dmtpps);
	}

	if(state->new_overflow) {
		state->new_overflow = false;
		dmtimer_pps_input_overflow(dmtpps);
	}
}

static void dmtimer_pps_output_capture(struct dmtimer_pps *dmtpps)
{
	s32 avg;
	u32 reload;
	unsigned long index;
	unsigned long indexp1;
	struct dmtimer_pps_output_state *state = &dmtpps->output_state;
	
	index = state->index;
	indexp1 = dmtimer_pps_index(index + 1);

	/*match = state->load[index] + state->period / 2;
	__omap_dm_timer_write(dmtpps->timer, OMAP_TIMER_MATCH_REG,
		match, dmtpps->timer->posted);
	dmtpps->timer->context.tmar = match;*/
	if (state->last_capture_valid) {
		reload = dmtimer_pps_output_calculate_reload(state);
		__omap_dm_timer_write(dmtpps->timer, OMAP_TIMER_LOAD_REG,
			reload, dmtpps->timer->posted);
		dmtpps->timer->context.tldr = reload;
	}
	
	state->index = indexp1;

	state->last_capture = state->capture;
	state->last_capture_valid = true;

	avg = dmtimer_pps_deficit_avg(state->deficit);
	dev_info(dmtpps->dev, "output capture deficit avg %d", avg);
	/*if (ctrl & OMAP_TIMER_CTRL_GPOCFG &&
			dmtimer_pps_est_ns(pin, avg) < 500) {
		ctrl &= ~OMAP_TIMER_CTRL_GPOCFG;
		pr_info("cpts: peroit ctrl %u\n", ctrl);
		__omap_dm_timer_write(dmtpps->timer,
			OMAP_TIMER_CTRL_REG, ctrl,
			dmtpps->timer->posted);
		dmtpps->timer->context.tclr = ctrl;
	}*/
}

static int dmtimer_pps_start_input(struct dmtimer_pps *dmtpps)
{
	u32 ctrl, mask = OMAP_TIMER_CTRL_TCM_BOTHEDGES;

	memset(&dmtpps->input_state, 0, sizeof(dmtpps->input_state));

	omap_dm_timer_enable(dmtpps->timer);

	// set timer pin to input and setup edge to capture
	if (dmtpps->settings.pps_mode & PPS_CAPTUREBOTH)
		mask = OMAP_TIMER_CTRL_TCM_BOTHEDGES;
	else if (dmtpps->settings.pps_mode & PPS_CAPTUREASSERT)
		mask = OMAP_TIMER_CTRL_TCM_HIGHTOLOW;
	else if (dmtpps->settings.pps_mode & PPS_CAPTURECLEAR)
		mask = OMAP_TIMER_CTRL_TCM_LOWTOHIGH;
	ctrl = mask | OMAP_TIMER_CTRL_GPOCFG | 1 << 10;
	__omap_dm_timer_write(dmtpps->timer, OMAP_TIMER_CTRL_REG, ctrl,
			dmtpps->timer->posted);
	dmtpps->timer->context.tclr = ctrl;
	omap_dm_timer_disable(dmtpps->timer);

	dev_info(dmtpps->dev, "start output timer ctrl %d", ctrl);

	// autoload to zero
	omap_dm_timer_set_load(dmtpps->timer, 1, 0);

	// enable the capture interrupt and start the timer
	__omap_dm_timer_write_status(dmtpps->timer, OMAP_TIMER_INT_CAPTURE);
	omap_dm_timer_set_int_enable(dmtpps->timer,
			OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_OVERFLOW);
	omap_dm_timer_start(dmtpps->timer);

	return 0;
}

static int dmtimer_pps_start_output(struct dmtimer_pps *dmtpps)
{
	u32 ctrl;
	u32 load;
	u64 time_now;
	u32 cycles;

	memset(&dmtpps->output_state, 0, sizeof(dmtpps->output_state));

	omap_dm_timer_enable(dmtpps->timer);

	//ctrl = OMAP_TIMER_CTRL_GPOCFG | OMAP_TIMER_CTRL_PT | 2 << 10;
	ctrl = 1 << 10;
	__omap_dm_timer_write(dmtpps->timer, OMAP_TIMER_CTRL_REG, ctrl,
			dmtpps->timer->posted);
	dmtpps->timer->context.tclr = ctrl;

	// setup autoload so we overflow once per second.
	load = 0ul - dmtpps->timer->rate;
	omap_dm_timer_set_load(dmtpps->timer, 1, load);

	dmtpps->output_state.period = dmtpps->timer->rate;
	dmtimer_pps_set_array(dmtpps->output_state.load, &load, sizeof(load),
			CPTS_AVERAGE_LEN);

	omap_dm_timer_set_int_enable(dmtpps->timer, OMAP_TIMER_INT_OVERFLOW);
	omap_dm_timer_start(dmtpps->timer);

	// setup an estimation of the time, the interrupt will get it more
	// accurate before we enable the output.
	div64_u64_rem((u64)dmtimer_pps_ktime_now(), 1000000000, &time_now);
	cycles = dmtimer_pps_est_cyc(dmtpps, time_now);
	omap_dm_timer_write_counter(dmtpps->timer, load + cycles);

	return 0;
}

static int dmtimer_pps_stop(struct dmtimer_pps *dmtpps)
{
	mutex_lock(&dmtpps->timer_mutex);
	omap_dm_timer_set_int_disable(dmtpps->timer,
			OMAP_TIMER_INT_CAPTURE |
			OMAP_TIMER_INT_OVERFLOW | OMAP_TIMER_INT_MATCH);
	mutex_unlock(&dmtpps->timer_mutex);

	cancel_work_sync(&dmtpps->input_work);

	omap_dm_timer_enable(dmtpps->timer);
	omap_dm_timer_stop(dmtpps->timer);

	return 0;
}

static int dmtimer_pps_start(struct dmtimer_pps *dmtpps)
{
	dmtimer_pps_stop(dmtpps);

	omap_dm_timer_set_source(dmtpps->timer, dmtpps->settings.clock_source);
	dmtpps->timer->rate = clk_get_rate(dmtpps->timer->fclk);

	if(dmtpps->settings.generate)
		return dmtimer_pps_start_output(dmtpps);
	else
		return dmtimer_pps_start_input(dmtpps);
}

static int dmtimer_pps_probe_dt(struct dmtimer_pps *dmtpps,
		struct platform_device *pdev)
{
	const __be32 *phand;
	struct device_node* timer_node;
	const char *tmp;

	phand = of_get_property(pdev->dev.of_node, "timer", NULL);
	if(!phand) {
		dev_err(&pdev->dev, "Missing timer property in the DT\n");
		return -EINVAL;
	}

	timer_node = of_find_node_by_phandle(be32_to_cpup(phand));
	if(!timer_node) {
		dev_err(&pdev->dev, "Unable to resolve timer\n");
		return -EINVAL;
	}

	of_property_read_string_index(timer_node, "ti,hwmods", 0, &tmp);
	if(!tmp) {
		dev_err(&pdev->dev, "Timer is not a dmtimer\n");
		return -EINVAL;
	}

	dmtpps->timer = omap_dm_timer_request_by_node(timer_node);
	if(!dmtpps->timer) {
		dev_err(&pdev->dev, "Request timer failed\n");
		return -EINVAL;
	}

	dmtpps->settings.clock_source = OMAP_TIMER_SRC_SYS_CLK;
	dmtpps->settings.pps_mode = PPS_CAPTUREASSERT;
	dmtpps->settings.generate = false;

	of_property_read_s32(pdev->dev.of_node, "clock",
			&dmtpps->settings.clock_source);
	of_property_read_s32(pdev->dev.of_node, "ppsmode",
			&dmtpps->settings.pps_mode);
	dmtpps->settings.generate = of_property_read_bool(pdev->dev.of_node,
			"output");

	return 0;
}

static struct dmtimer_pps * dmtpps_from_device(struct device *dev)
{
	struct dmtimer_pps *dmtpps;
	struct platform_device *pdev = container_of(dev,
			struct platform_device, dev);
	dmtpps = platform_get_drvdata(pdev);
	return dmtpps;
}

static ssize_t dmtimer_pps_show_clock_source(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dmtimer_pps *dmtpps = dmtpps_from_device(dev);
	switch (dmtpps->settings.clock_source)
	{
	case OMAP_TIMER_SRC_SYS_CLK:
		return scnprintf(buf, PAGE_SIZE, "system\n");
	case OMAP_TIMER_SRC_32_KHZ:
		return scnprintf(buf, PAGE_SIZE, "32khz\n");
	case OMAP_TIMER_SRC_EXT_CLK:
		return scnprintf(buf, PAGE_SIZE, "external\n");
	default:
		break;
	}
	return scnprintf(buf, PAGE_SIZE, "unknown (%d)\n",
			dmtpps->settings.clock_source);
}

static ssize_t dmtimer_pps_store_clock_source(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dmtimer_pps *dmtpps = dmtpps_from_device(dev);

	dmtimer_pps_stop(dmtpps);

	if (strncmp(buf, "system", min(6u, count)) == 0)
		dmtpps->settings.clock_source = OMAP_TIMER_SRC_SYS_CLK;
	else if (strncmp(buf, "32khz", min(5u, count)) == 0)
		dmtpps->settings.clock_source = OMAP_TIMER_SRC_32_KHZ;
	else if (strncmp(buf, "external", min(8u, count)) == 0)
		dmtpps->settings.clock_source = OMAP_TIMER_SRC_EXT_CLK;
	
	dmtimer_pps_start(dmtpps);

	return count;
}

static DEVICE_ATTR(clock_source, 0664, dmtimer_pps_show_clock_source,
		dmtimer_pps_store_clock_source);

static ssize_t dmtimer_pps_show_edge(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	s32 edge;
	struct dmtimer_pps *dmtpps = dmtpps_from_device(dev);
	edge = dmtpps->settings.pps_mode & 0x0F;

	switch (edge)
	{
	case PPS_CAPTUREASSERT:
		return scnprintf(buf, PAGE_SIZE, "rising\n");
	case PPS_CAPTURECLEAR:
		return scnprintf(buf, PAGE_SIZE, "falling\n");
	case PPS_CAPTUREBOTH:
		return scnprintf(buf, PAGE_SIZE, "both\n");
	default:
		break;
	}
	return scnprintf(buf, PAGE_SIZE, "unknown (%d)\n", edge);
}

static ssize_t dmtimer_pps_store_edge(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	s32 edge = PPS_CAPTUREASSERT;
	struct dmtimer_pps *dmtpps = dmtpps_from_device(dev);

	dmtimer_pps_stop(dmtpps);

	if (strncmp(buf, "rising", min(6u, count)) == 0)
		edge = PPS_CAPTUREASSERT;
	else if (strncmp(buf, "falling", min(7u, count)) == 0)
		edge = PPS_CAPTURECLEAR;
	else if (strncmp(buf, "both", min(4u, count)) == 0)
		edge = PPS_CAPTUREBOTH;
	dmtpps->settings.pps_mode = (dmtpps->settings.pps_mode & ~0x0F) | edge;
	
	dmtimer_pps_start(dmtpps);

	return count;
}

static DEVICE_ATTR(edge, 0664, dmtimer_pps_show_edge, dmtimer_pps_store_edge);

static ssize_t dmtimer_pps_show_direction(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dmtimer_pps *dmtpps = dmtpps_from_device(dev);

	if (dmtpps->settings.generate)
		return scnprintf(buf, PAGE_SIZE, "output\n");
	return scnprintf(buf, PAGE_SIZE, "input\n");
}

static ssize_t dmtimer_pps_store_direction(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dmtimer_pps *dmtpps = dmtpps_from_device(dev);

	dmtimer_pps_stop(dmtpps);

	if (strncmp(buf, "output", min(6u, count)) == 0)
		dmtpps->settings.generate = true;
	else if (strncmp(buf, "input", min(5u, count)) == 0)
		dmtpps->settings.generate = false;
	
	dmtimer_pps_start(dmtpps);

	return count;
}

static DEVICE_ATTR(direction, 0664, dmtimer_pps_show_direction,
		dmtimer_pps_store_direction);

static ssize_t dmtimer_pps_store_capture(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	struct dmtimer_pps *dmtpps = dmtpps_from_device(dev);

	if(!dmtpps->settings.generate)
		return count;

	err = kstrtoull(buf, 10, &dmtpps->output_state.capture);
	if(err)
		return count;
	
	dmtimer_pps_output_capture(dmtpps);
	return count;
}

static DEVICE_ATTR(capture, 0220, NULL, dmtimer_pps_store_capture);

static int dmtimer_pps_probe(struct platform_device *pdev)
{
	int err;
	struct dmtimer_pps *dmtpps = NULL;

	dmtpps = devm_kzalloc(&pdev->dev, sizeof(struct dmtimer_pps),
			GFP_KERNEL);
	if(!dmtpps)
		return -ENOMEM;

	memset(dmtpps, 0, sizeof(struct dmtimer_pps));

	dmtpps->dev = &pdev->dev;

	mutex_init(&dmtpps->timer_mutex);

	err = dmtimer_pps_probe_dt(dmtpps, pdev);
	if(err)
		return err;

	dmtpps->info.mode = PPS_CAPTUREBOTH | PPS_ECHOASSERT |
		PPS_ECHOCLEAR | PPS_CANWAIT | PPS_TSFMT_TSPEC;
	dmtpps->pps = pps_register_source(&dmtpps->info,
			dmtpps->settings.pps_mode);
	if(!dmtpps->pps)
		return -EINVAL;

	device_create_file(&pdev->dev, &dev_attr_clock_source);
	device_create_file(&pdev->dev, &dev_attr_edge);
	device_create_file(&pdev->dev, &dev_attr_direction);
	device_create_file(&pdev->dev, &dev_attr_capture);
		
	err = devm_request_irq(&pdev->dev,
			omap_dm_timer_get_irq(dmtpps->timer),
			dmtimer_pps_interrupt, 0, dev_name(&pdev->dev),
			dmtpps);
	if(err) {
		dev_err(&pdev->dev, "IRQ request failed (%d)\n", err);
		return err;
	}

	INIT_WORK(&dmtpps->input_work, dmtimer_pps_input_work);

	platform_set_drvdata(pdev, dmtpps);

	return dmtimer_pps_start(dmtpps);
}

static int dmtimer_pps_remove(struct platform_device *pdev)
{
	struct dmtimer_pps *dmtpps = platform_get_drvdata(pdev);

	if(dmtpps->timer) {
		devm_free_irq(&pdev->dev, omap_dm_timer_get_irq(dmtpps->timer),
				dmtpps);
		dmtimer_pps_stop(dmtpps);
		omap_dm_timer_free(dmtpps->timer);
		dmtpps->timer = NULL;
	}

	pps_unregister_source(dmtpps->pps);

	return 0;
}

static const struct of_device_id dmtimer_pps_dt_ids[] = {
	{ .compatible = "dmtimer-pps" },
	{},
};
MODULE_DEVICE_TABLE(of, dmtimer_pps_dt_ids);

static struct platform_driver dmtimer_pps_driver = {
	.probe	= dmtimer_pps_probe,
	.remove	= dmtimer_pps_remove,
	.driver	= {
		.name		= DMTIMER_PPS_NAME,
		.of_match_table	= dmtimer_pps_dt_ids,
	},
};

module_platform_driver(dmtimer_pps_driver);
MODULE_AUTHOR("Jonathan Herbst <jonathan_herbst@lord.com>");
MODULE_DESCRIPTION("Use dmtimer as a PPS source and generator");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.1");
