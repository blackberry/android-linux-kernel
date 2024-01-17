/* Copyright (c) 2012, 2014-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM msm_low_power

#if !defined(_TRACE_MSM_LOW_POWER_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MSM_LOW_POWER_H_

#include <linux/tracepoint.h>

TRACE_EVENT(cpu_power_select,

	TP_PROTO(int index, u32 sleep_us, u32 latency, u32 next_event_us),

	TP_ARGS(index, sleep_us, latency, next_event_us),

	TP_STRUCT__entry(
		__field(int, index)
		__field(u32, sleep_us)
		__field(u32, latency)
		__field(u32, next_event_us)
	),

	TP_fast_assign(
		__entry->index = index;
		__entry->sleep_us = sleep_us;
		__entry->latency = latency;
		__entry->next_event_us = next_event_us;
	),

	TP_printk("idx:%d sleep_time:%u latency:%u next_event:%u",
		__entry->index, __entry->sleep_us, __entry->latency,
		__entry->next_event_us)
);

TRACE_EVENT(cpu_idle_enter,

	TP_PROTO(int index),

	TP_ARGS(index),

	TP_STRUCT__entry(
		__field(int, index)
	),

	TP_fast_assign(
		__entry->index = index;
	),

	TP_printk("idx:%d",
		__entry->index)
);

TRACE_EVENT(cpu_idle_exit,

	TP_PROTO(int index, bool success),

	TP_ARGS(index, success),

	TP_STRUCT__entry(
		__field(int, index)
		__field(bool, success)
	),

	TP_fast_assign(
		__entry->index = index;
		__entry->success = success;
	),

	TP_printk("idx:%d success:%d",
		__entry->index,
		__entry->success)
);

DECLARE_EVENT_CLASS(cpu_freq,

	TP_PROTO(int cpu, unsigned long cpu_clk, unsigned long l2_clk),

	TP_ARGS(cpu, cpu_clk, l2_clk),

	TP_STRUCT__entry(
		__field(int, cpu)
		__field(unsigned long, cpu_clk)
		__field(unsigned long, l2_clk)
	),

	TP_fast_assign(
		__entry->cpu = cpu;
		__entry->cpu_clk = cpu_clk;
		__entry->l2_clk = l2_clk;
	),

	TP_printk("cpu:%d cpu_clk:%luHZ l2_clk:%luHZ",
		__entry->cpu, __entry->cpu_clk, __entry->l2_clk)
);

DEFINE_EVENT(cpu_freq, cpu_idle_enter_cpu_freq,
	TP_PROTO(int cpu, unsigned long cpu_clk, unsigned long l2_clk),
	TP_ARGS(cpu, cpu_clk, l2_clk)
);

DEFINE_EVENT(cpu_freq, cpu_idle_exit_cpu_freq,
	TP_PROTO(int cpu, unsigned long cpu_clk, unsigned long l2_clk),
	TP_ARGS(cpu, cpu_clk, l2_clk)
);

DECLARE_EVENT_CLASS(cluster,

	TP_PROTO(const char *name, int index, unsigned long sync_cpus,
		unsigned long child_cpus, bool from_idle),

	TP_ARGS(name, index, sync_cpus, child_cpus, from_idle),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(int, index)
		__field(unsigned long, sync_cpus)
		__field(unsigned long, child_cpus)
		__field(bool, from_idle)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->index = index;
		__entry->sync_cpus = sync_cpus;
		__entry->child_cpus = child_cpus;
		__entry->from_idle = from_idle;
	),

	TP_printk("cluster_name:%s idx:%d sync:0x%lx child:0x%lx idle:%d",
		__entry->name,
		__entry->index,
		__entry->sync_cpus,
		__entry->child_cpus,
		__entry->from_idle)
);
DEFINE_EVENT(cluster, cluster_enter,
	TP_PROTO(const char *name, int index, unsigned long sync_cpus,
		unsigned long child_cpus, bool from_idle),
	TP_ARGS(name, index, sync_cpus, child_cpus, from_idle)
);

DEFINE_EVENT(cluster, cluster_exit,
	TP_PROTO(const char *name, int index, unsigned long sync_cpus,
		unsigned long child_cpus, bool from_idle),
	TP_ARGS(name, index, sync_cpus, child_cpus, from_idle)
);

TRACE_EVENT(pre_pc_cb,

	TP_PROTO(int tzflag),

	TP_ARGS(tzflag),

	TP_STRUCT__entry(
		__field(int, tzflag)
	),

	TP_fast_assign(
		__entry->tzflag = tzflag;
	),

	TP_printk("tzflag:%d",
		__entry->tzflag
	)
);
#endif
#define TRACE_INCLUDE_FILE trace_msm_low_power
#include <trace/define_trace.h>
