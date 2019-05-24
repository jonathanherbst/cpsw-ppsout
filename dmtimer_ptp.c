#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/clocksource.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/timecounter.h>

#include <arch/arm/plat-omap/include/plat/dmtimer.h>

#define DMTIMER_PTP_NAME 	"dmtimer-ptp"

struct dmtimer_ptp_state {
	volatile bool new_capture;
	volatile bool new_overflow;
	volatile u32 capture;
	volatile struct pps_event_time pps_time;
	u32 counter;
	u32 load[3];
	u32 clock_freq;
};

struct dmtimer_ptp {
	const struct device *dev;
	struct ptp_clock *ptp;
	struct ptp_clock_info info;
	struct omap_dm_timer *timer;
	struct mutex mutex;
	struct work_struct work;
	u32 cc_mult;
	struct cyclecounter cc;
	struct timecounter tc;
	bool enable_pps;
	struct dmtimer_ptp_state state;
};

static inline s64 dmtimer_ptp_ktime_now(void)
{
	struct system_time_snapshot snap;

	ktime_get_snapshot(&snap);
	return snap.real.tv64;
}

static inline cycle_t dmtimer_ptp_timecounter_ns2cyc(
	const struct timecounter *tc, u64 ns)
{
	// do the opposite of cyclecounter_cyc2ns
	ns = div_u64((ns << tc->cc->shift) - tc->frac, tc->cc->mult);
	return (cycle_t) (ns & tc->cc->mask);
}

static cycle_t dmtimer_ptp_read(const struct cyclecounter *cc)
{
	struct dmtimer_ptp *self = container_of(cc, struct dmtimer_ptp, cc);

	u32 timer_value = omap_dm_timer_read_counter(self->timer);
	if (!self->state.new_overflow || (timer_value > (u32)(0u - self->state.clock_freq / 2u)))
		// no new overflow, or there was an overflow but we read the timer before the overflow
		return self->state.counter + (timer_value -
			self->state.load[2]);

	// had an overflow that hasn't been serviced yet
	return self->state.counter + (0u - self->state.load[2]) +
		(timer_value - self->state.load[1]);
}

static int dmtimer_ptp_enable_extts(struct dmtimer_ptp *self,
	struct ptp_extts_request *rq, int on)
{
	u32 ctrl;

	// validate the index
	if(rq->index != 0)
		return -EINVAL;

	mutex_lock(&self->mutex);
	if (on && rq->flags & PTP_ENABLE_FEATURE) { // enable
		// make sure an edge is selected
		if(!(rq->flags & PTP_RISING_EDGE) &&
				!(rq->flags & PTP_FALLING_EDGE))
			return -EINVAL;
		
		// determine edge configuration
		ctrl = self->timer->context.tclr &
			~OMAP_TIMER_CTRL_TCM_BOTHEDGES;
		// make sure pin is set as an input
		ctrl |= OMAP_TIMER_CTRL_GPOCFG;
		if((rq->flags & PTP_RISING_EDGE) &&
				!(rq->flags & PTP_FALLING_EDGE))
			ctrl |= OMAP_TIMER_CTRL_TCM_LOWTOHIGH;
		if(!(rq->flags & PTP_RISING_EDGE) &&
				(rq->flags & PTP_FALLING_EDGE))
			ctrl |= OMAP_TIMER_CTRL_TCM_HIGHTOLOW;
		if((rq->flags & PTP_RISING_EDGE) &&
				(rq->flags & PTP_FALLING_EDGE))
			ctrl |= OMAP_TIMER_CTRL_TCM_BOTHEDGES;
		__omap_dm_timer_write(self->timer, OMAP_TIMER_CTRL_REG, ctrl,
			self->timer->posted);
		self->timer->context.tclr = ctrl;
	} else { // disable
		ctrl = self->timer->context.tclr &
			~OMAP_TIMER_CTRL_TCM_BOTHEDGES;
		__omap_dm_timer_write(self->timer, OMAP_TIMER_CTRL_REG, ctrl,
			self->timer->posted);
		self->timer->context.tclr = ctrl;
	}
	mutex_unlock(&self->mutex);
	
	return 0;
}

static int dmtimer_ptp_enable_perout(struct dmtimer_ptp *self,
	struct ptp_perout_request *rq, int on)
{
	u32 ctrl;

	// validate the index
	if (rq->index != 0)
		return -EINVAL;

	mutex_lock(&self->mutex);
	if (on && !(rq->period.sec == 0 && rq->period.nsec == 0)) { // enable
		ctrl = self->timer->context.tclr;
		// make sure timer pin is set to output
		ctrl &= ~OMAP_TIMER_CTRL_GPOCFG;
		__omap_dm_timer_write(self->timer, OMAP_TIMER_CTRL_REG, ctrl,
			self->timer->posted);
		self->timer->context.tclr = ctrl;
	}
	mutex_unlock(&self->mutex);

	return 0;
}

static int dmtimer_ptp_enable_pps(struct dmtimer_ptp *self, int on)
{
	self->enable_pps = on != 0;
	return 0;
}

static int dmtimer_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u32 mult;
	u64 adj;
	u32 diff;
	struct dmtimer_ptp *self = container_of(ptp, struct dmtimer_ptp, info);

	bool neg_adj = ppb < 0;

	if (neg_adj)
		ppb = -ppb;
	
	mult = self->cc_mult;
	adj = (u64)mult * ppb;
	diff = div_u64(adj, 1000000000ULL);

	mutex_lock(&self->mutex);
	timecounter_read(&self->tc);
	self->cc.mult = neg_adj ? mult - diff : mult + diff;
	mutex_unlock(&self->mutex);

	return 0;
}

static int dmtimer_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct dmtimer_ptp *self = container_of(ptp, struct dmtimer_ptp, info);

	mutex_lock(&self->mutex);
	timecounter_adjtime(&self->tc, delta);
	mutex_unlock(&self->mutex);

	return 0;
}

static int dmtimer_ptp_gettime(struct ptp_clock_info *ptp,
	struct timespec64 *ts)
{
	u64 ns;
	struct dmtimer_ptp *self = container_of(ptp, struct dmtimer_ptp, info);

	mutex_lock(&self->mutex);
	ns = timecounter_read(&self->tc);
	mutex_unlock(&self->mutex);

	*ts = ns_to_timespec64(ns);

	return 0;
}

static int dmtimer_ptp_settime(struct ptp_clock_info *ptp,
	const struct timespec64 *ts)
{
	struct dmtimer_ptp *self = container_of(ptp, struct dmtimer_ptp, info);

	u64 ns = timespec64_to_ns(ts);

	mutex_lock(&self->mutex);
	timecounter_init(&self->tc, &self->cc, ns);
	mutex_unlock(&self->mutex);

	return 0;
}

static int dmtimer_ptp_enable(struct ptp_clock_info *ptp,
	struct ptp_clock_request *rq, int on)
{
	struct dmtimer_ptp* self = container_of(ptp, struct dmtimer_ptp, info);
	
	switch (rq->type)
	{
	case PTP_CLK_REQ_EXTTS:
		return dmtimer_ptp_enable_extts(self, &rq->extts, on);
	case PTP_CLK_REQ_PEROUT:
		return dmtimer_ptp_enable_perout(self, &rq->perout, on);
	case PTP_CLK_REQ_PPS:
		return dmtimer_ptp_enable_pps(self, on);
	}

	return -EINVAL;
}

static int dmtimer_ptp_verify(struct ptp_clock_info *ptp, unsigned int pin,
        enum ptp_pin_function func, unsigned int chan)
{
	// validate the pin
	if (pin >= ptp->n_pins || !ptp->pin_config)
		return -EINVAL;

	// validate the channel
	if (chan != ptp->pin_config[pin].chan)
		return -EINVAL;
	
	// validate the pin can be used for the requested function
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

static irqreturn_t dmtimer_ptp_interrupt(int irq, void *data)
{
	struct dmtimer_ptp *self = data;
	struct pps_event_time pps_time;
	u32 match;
	u32 irq_status = omap_dm_timer_read_status(self->timer);

	// clear interrupts
	__omap_dm_timer_write_status(self->timer, irq_status);

	if (irq_status & OMAP_TIMER_INT_OVERFLOW) {
		pps_get_ts(&pps_time);
		self->state.pps_time = pps_time;
		match = 0u - ((0u - self->state.load[1]) >> 1);
		__omap_dm_timer_write(self->timer, OMAP_TIMER_MATCH_REG,
			match, self->timer->posted);
		__omap_dm_timer_write(self->timer, OMAP_TIMER_LOAD_REG,
			self->state.load[0], self->timer->posted);
		self->state.new_overflow = true;
	}

	if (irq_status & OMAP_TIMER_INT_CAPTURE) {
		self->state.capture = __omap_dm_timer_read(
			self->timer, OMAP_TIMER_CAPTURE_REG,
			self->timer->posted);
		self->state.new_capture = true;
	}

	// schedule work to act on interrupt values
	schedule_work(&self->work);

	return IRQ_HANDLED;
}

static void dmtimer_ptp_work(struct work_struct *work)
{
	u64 timestamp, seconds, ts_next;
	u32 ns_remainder;
	struct dmtimer_ptp *self = container_of(work, struct dmtimer_ptp,
		work);
	
	struct ptp_clock_event extts_event = {
		.type = PTP_CLOCK_EXTTS,
		.index = 0
	};
	
	struct ptp_clock_event pps_event = {
		.type = PTP_CLOCK_PPS,
		.index = 0,
		.pps_times = self->state.pps_time
	};

	if (self->state.new_capture) {
		mutex_lock(&self->mutex);
		extts_event.timestamp = timecounter_cyc2time(&self->tc,
			self->state.capture);
		mutex_unlock(&self->mutex);
		self->state.new_capture = false;

		dev_info(self->dev, "capture: %llu\n", extts_event.timestamp);
		
		ptp_clock_event(self->ptp, &extts_event);
	}

	if (self->state.new_overflow) {
		if (self->enable_pps)
			ptp_clock_event(self->ptp, &pps_event);

		// generate the next load value
		mutex_lock(&self->mutex);
		self->state.counter += 0u - self->state.load[2];
		timestamp = timecounter_cyc2time(&self->tc,
			self->state.counter + (0u - self->state.load[0]));
		seconds = div_u64_rem(timestamp, 1000000000, &ns_remainder);
		ts_next = (seconds + 1) * 1000000000;
		// protect against missing the next match
		if (ns_remainder > 750000000)
			ts_next += 1000000000;
		
		self->state.load[2] = self->state.load[1];
		self->state.load[1] = self->state.load[0];
		self->state.load[0] = 0u - dmtimer_ptp_timecounter_ns2cyc(
			&self->tc, ts_next - timestamp);
		self->state.new_overflow = false;
		mutex_unlock(&self->mutex);
	}
}

static struct ptp_pin_desc dmtimer_ptp_pins[1] = {
	{
		.name = "HW_PIN",
		.index = 0,
		.func = PTP_PF_NONE,
		.chan = 0
	}
};

static struct ptp_clock_info dmtimer_ptp_info = {
	.owner		= THIS_MODULE,
	.name		= "dmtimer clock",
	.max_adj	= 100000000,
	.n_alarm    	= 0,
	.n_ext_ts	= 1,
	.n_per_out	= 1,
	.n_pins		= 1,
	.pps		= 1,
	.pin_config 	= dmtimer_ptp_pins,
	.adjfreq	= dmtimer_ptp_adjfreq,
	.adjtime	= dmtimer_ptp_adjtime,
	.gettime64	= dmtimer_ptp_gettime,
	.settime64	= dmtimer_ptp_settime,
	.enable		= dmtimer_ptp_enable,
	.verify     	= dmtimer_ptp_verify,
};

static void dmtimer_ptp_stop(struct dmtimer_ptp *self)
{
	mutex_lock(&self->mutex);
	omap_dm_timer_set_int_disable(self->timer,
			OMAP_TIMER_INT_CAPTURE |
			OMAP_TIMER_INT_OVERFLOW | OMAP_TIMER_INT_MATCH);
	mutex_unlock(&self->mutex);

	cancel_work_sync(&self->work);

	omap_dm_timer_enable(self->timer);
	omap_dm_timer_stop(self->timer);
}

static int dmtimer_ptp_start(struct dmtimer_ptp *self)
{
	u32 ctrl;

	dmtimer_ptp_stop(self);

	// setup the time counter for the 24 MHz clock
	self->cc.read = dmtimer_ptp_read;
	self->cc.mask = CLOCKSOURCE_MASK(32);
	self->cc_mult = 0xA6AAAAAA;
	self->cc.mult = self->cc_mult;
	self->cc.shift = 26;
	self->state.clock_freq = 24000000;

	// setup the timer to use the 24 MHz system clock
	omap_dm_timer_set_source(self->timer, OMAP_TIMER_SRC_SYS_CLK);

	// setup the timer to input, and enable auto reload
	ctrl = OMAP_TIMER_CTRL_GPOCFG | OMAP_TIMER_CTRL_AR | 
		OMAP_TIMER_CTRL_PT | OMAP_TIMER_CTRL_CE |
		OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE << 10;
	__omap_dm_timer_write(self->timer, OMAP_TIMER_CTRL_REG, ctrl,
			self->timer->posted);
	self->timer->context.tclr = ctrl;

	omap_dm_timer_set_int_enable(self->timer, OMAP_TIMER_INT_CAPTURE |
		OMAP_TIMER_INT_OVERFLOW);
	
	self->state.counter = 0;
	self->state.load[0] = 0u - 24000000;
	self->state.load[1] = self->state.load[0];
	self->state.load[2] = self->state.load[0];
	omap_dm_timer_set_load_start(self->timer, 1, self->state.load[0]);
	timecounter_init(&self->tc, &self->cc, ktime_to_ns(ktime_get_real()));

	return 0;
}

static int dmtimer_ptp_probe_dt(struct dmtimer_ptp *self,
		struct platform_device *pdev)
{
	const __be32 *phand;
	const char *tmp;
	struct device_node* timer_node;

	// get the input device
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

	of_property_read_string_index(timer_node,
		"ti,hwmods", 0, &tmp);
	if(!tmp) {
		dev_err(&pdev->dev, "Timer is not a dmtimer\n");
		return -EINVAL;
	}

	self->timer = omap_dm_timer_request_by_node(timer_node);
	if(!self->timer) {
		dev_err(&pdev->dev, "Request timer failed\n");
		return -EINVAL;
	}

	return 0;
}

static int dmtimer_ptp_probe(struct platform_device *pdev)
{
	int err;
	struct dmtimer_ptp *self = NULL;

	self = devm_kzalloc(&pdev->dev, sizeof(struct dmtimer_ptp),
		GFP_KERNEL);
	if(!self)
		return -ENOMEM;

	memset(self, 0, sizeof(struct dmtimer_ptp));

	self->dev = &pdev->dev;

	// pull the timer from the device tree
	err = dmtimer_ptp_probe_dt(self, pdev);
	if(err)
		return err;
	
	// reserve the timer interrupt
	err = devm_request_irq(&pdev->dev,
		omap_dm_timer_get_irq(self->timer), dmtimer_ptp_interrupt, 0,
		dev_name(&pdev->dev), self);
	if(err) {
		omap_dm_timer_free(self->timer);
		self->timer = NULL;
		dev_err(&pdev->dev, "IRQ request failed (%d)\n", err);
		return err;
	}
	
	// register the ptp clock
	self->info = dmtimer_ptp_info;
	self->ptp = ptp_clock_register(&self->info, &pdev->dev);
	if (IS_ERR(self->ptp)) {
		err = PTR_ERR(self->ptp);
		devm_free_irq(&pdev->dev, omap_dm_timer_get_irq(self->timer),
			self);
		omap_dm_timer_free(self->timer);
		self->timer = NULL;
		dev_err(&pdev->dev, "PTP clock register failed (%d)\n", err);
		return err;
	}

	mutex_init(&self->mutex);
	INIT_WORK(&self->work, dmtimer_ptp_work);

	platform_set_drvdata(pdev, self);

	dmtimer_ptp_start(self);

	return 0;
}

static int dmtimer_ptp_remove(struct platform_device *pdev)
{
	struct dmtimer_ptp *self = platform_get_drvdata(pdev);

	if(self->timer) {
		devm_free_irq(&pdev->dev, omap_dm_timer_get_irq(self->timer),
			self);
		dmtimer_ptp_stop(self);
		omap_dm_timer_free(self->timer);
		self->timer = NULL;
	}

	if (!IS_ERR(self->ptp)) {
		ptp_clock_unregister(self->ptp);
	}

	return 0;
}

static const struct of_device_id dmtimer_ptp_dt_ids[] = {
	{ .compatible = "dmtimer-ptp" },
	{},
};
MODULE_DEVICE_TABLE(of, dmtimer_ptp_dt_ids);

static struct platform_driver dmtimer_ptp_driver = {
	.probe	= dmtimer_ptp_probe,
	.remove	= dmtimer_ptp_remove,
	.driver	= {
		.name		= DMTIMER_PTP_NAME,
		.of_match_table	= dmtimer_ptp_dt_ids,
	},
};

module_platform_driver(dmtimer_ptp_driver);
MODULE_AUTHOR("Jonathan Herbst <jonathan_herbst@lord.com>");
MODULE_DESCRIPTION("Use dmtimer as a PTP clock");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.1");