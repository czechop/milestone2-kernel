/*
 * Copyright (c) 2013, Francisco Franco. All rights reserved.
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

#define pr_fmt(fmt) "cpu-boost: " fmt

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/time.h>

#define MIM_TIME_INTERVAL_US (100 * USEC_PER_MSEC)

/* 
 * This variable comes from interactive governor. If you want to implement this
 * driver you have to also implement this variable on your governor of choice 
 */
extern int input_boost_freq;

/*
 * Use this variable in your governor of choice to calculate when the cpufreq
 * core is allowed to ramp the cpu down after an input event. That logic is done
 * by you, this var only outputs the last time in us an event was captured
 */
u64 last_input_time;

int boost_freq_buf;

static struct workqueue_struct *input_boost_wq;
static struct work_struct input_boost_work;
static struct delayed_work rem_input_boost;

/*
 * The CPUFREQ_ADJUST notifier is used to override the current policy min to
 * make sure policy min >= boost_min. The cpufreq framework then does the job
 * of enforcing the new policy.
 */
static int boost_adjust_notify(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int cpu = policy->cpu;

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	/* just in case someone underclocks below input_boost_freq */
	if (boost_freq_buf > policy->max)
		boost_freq_buf = policy->max;

	pr_debug("CPU%u policy min before boost: %u kHz\n",
		 cpu, policy->min);
	pr_debug("CPU%u boost min: %u kHz\n", cpu, boost_freq_buf);

	cpufreq_verify_within_limits(policy, boost_freq_buf, UINT_MAX);

	pr_debug("CPU%u policy min after boost: %u kHz\n",
		 cpu, policy->min);

	return NOTIFY_OK;
}

static struct notifier_block boost_adjust_nb = {
	.notifier_call = boost_adjust_notify,
};

static void do_rem_input_boost(struct work_struct *work)
{
	unsigned int i;
	boost_freq_buf = 0;
	/* Force policy re-evaluation to trigger adjust notifier. */
	for_each_online_cpu(i)
		cpufreq_update_policy(i);
}

static void do_input_boost(struct work_struct *work)
{
	unsigned int i, ret;
	struct cpufreq_policy policy;

	/* 
	 * to avoid concurrency issues we cancel rem_input_boost
	 * and wait for it to finish the work
	 */
	cancel_delayed_work_sync(&rem_input_boost);

	boost_freq_buf = input_boost_freq;

	for_each_online_cpu(i)
	{
		ret = cpufreq_get_policy(&policy, i);
		if (ret)
			continue;
		
		if (policy.cur >= input_boost_freq)
			continue;

		cpufreq_update_policy(i);
	}
	queue_delayed_work_on(0, input_boost_wq, &rem_input_boost, msecs_to_jiffies(40));
}

static void boost_input_event(struct input_handle *handle,
                unsigned int type, unsigned int code, int value)
{
	u64 now;

	now = ktime_to_us(ktime_get());

	if (now - last_input_time < MIM_TIME_INTERVAL_US)
		return;

	if (work_pending(&input_boost_work))
		return;

	queue_work_on(0, input_boost_wq, &input_boost_work);
	last_input_time = ktime_to_us(ktime_get());
}

static int boost_input_connect(struct input_handler *handler,
                struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void boost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id boost_ids[] = {
	/* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
				INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
				BIT_MASK(ABS_MT_POSITION_X) |
				BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler boost_input_handler = {
	.event          = boost_input_event,
	.connect        = boost_input_connect,
	.disconnect     = boost_input_disconnect,
	.name           = "input-boost",
	.id_table       = boost_ids,
};

static int init(void)
{
	int ret;
	input_boost_wq = create_singlethread_workqueue("input_boost_wq");

	if (!input_boost_wq)
		return -ENOMEM;

	INIT_WORK(&input_boost_work, do_input_boost);
	INIT_DELAYED_WORK(&rem_input_boost, do_rem_input_boost);

	ret = input_register_handler(&boost_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, error: %d", ret);
		return ret;
	}

	cpufreq_register_notifier(&boost_adjust_nb, CPUFREQ_POLICY_NOTIFIER);

	return 0;
}
late_initcall(init);
