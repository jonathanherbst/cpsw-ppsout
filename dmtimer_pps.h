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
#include <linux/mutex.h>

#include <arch/arm/plat-omap/include/plat/dmtimer.h>

#define CPTS_AVERAGE_LEN 4  // needs to be power of 2

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
	int supported_pps_mode;
	struct pps_device *pps;
	struct pps_source_info info;
	struct omap_dm_timer *timer;
	struct mutex timer_mutex;
	struct work_struct capture_work;
	struct work_struct overflow_work;
	struct dmtimer_pps_settings settings;
	union {
		struct dmtimer_pps_input_state input_state;
		struct dmtimer_pps_output_state output_state;
	};
};

#endif
