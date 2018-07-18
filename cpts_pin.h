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
#ifndef _TI_CPTS_PIN_H_
#define _TI_CPTS_PIN_H_

#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/timecounter.h>

#include <arch/arm/plat-omap/include/plat/dmtimer.h>

#define CPTS_NUM_PINS 4
#define CPTS_AVERAGE_LEN 4  // needs to be power of 2

struct cpts;

struct cpts_extts_state {
	volatile u32 capture;
	bool new_capture;
	u32 last_capture;
	bool last_capture_valid;
	u32 period;
	unsigned long index;
	u32 load[CPTS_AVERAGE_LEN];
	s32 deficit[CPTS_AVERAGE_LEN];
};

struct cpts_perout_state {
	u64 capture;
	bool capture_valid;
	u64 last_capture;
	bool last_capture_valid;
	u32 period;
	unsigned long index;
	u32 load[CPTS_AVERAGE_LEN];
	s32 deficit[CPTS_AVERAGE_LEN];
	bool deficit_valid;
};

struct cpts_pin {
	struct device_node *timerNode;
	struct omap_dm_timer *timer;
	struct ptp_pin_desc *ptp_pin;
	struct ptp_clock_request state;
	struct work_struct capture_work;
	struct work_struct overflow_work;
	union {
		struct cpts_extts_state extts_state;
		struct cpts_perout_state perout_state;
	};
};

struct cpts_pin_info {
	struct ptp_clock_info ptp_info;
	struct ptp_clock *ptp_clock;
	struct cyclecounter cc;
	struct timecounter tc;
	int phc_index;
	struct cpts_pin pins[CPTS_NUM_PINS];
};

extern struct ptp_pin_desc cpts_pins[CPTS_NUM_PINS];

int cpts_pin_ptp_enable(struct ptp_clock_info *ptp,
		struct ptp_clock_request *rq, int on);
int cpts_pin_ptp_verify(struct ptp_clock_info *ptp, unsigned int pin,
		enum ptp_pin_function func, unsigned int chan);
int cpts_pin_register(struct cpts *cpts);
void cpts_pin_unregister(struct cpts *cpts);

#endif
