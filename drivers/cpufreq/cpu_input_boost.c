/*
 * Copyright (C) 2018, Sultan Alsawaf <sultanxda@gmail.com>
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

#define pr_fmt(fmt) "cpu_input_boost: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/msm_drm_notify.h>
#include <linux/input.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/version.h>

static unsigned int input_boost_freq_lp = CONFIG_INPUT_BOOST_FREQ_LP;
static unsigned int input_boost_freq_hp = CONFIG_INPUT_BOOST_FREQ_PERF;
static unsigned short input_boost_duration = CONFIG_INPUT_BOOST_DURATION_MS;
static unsigned short dynamic_stune_boost_duration = CONFIG_INPUT_BOOST_DURATION_MS;
static unsigned int remove_input_boost_freq_lp = CONFIG_REMOVE_INPUT_BOOST_FREQ_LP;
static unsigned int remove_input_boost_freq_perf = CONFIG_REMOVE_INPUT_BOOST_FREQ_PERF;

#ifdef CONFIG_DYNAMIC_STUNE_BOOST
static bool stune_boost_active;
static int boost_slot;
static unsigned short dynamic_stune_boost;
module_param(dynamic_stune_boost, short, 0644);
module_param(dynamic_stune_boost_duration, short, 0644);
#endif

module_param(input_boost_freq_lp, uint, 0644);
module_param(input_boost_freq_hp, uint, 0644);
module_param(input_boost_duration, short, 0644);
module_param(remove_input_boost_freq_lp, uint, 0644);
module_param(remove_input_boost_freq_perf, uint, 0644);

/* The sched_param struct is located elsewhere in newer kernels */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
#include <uapi/linux/sched/types.h>
#endif

/* Available bits for boost state */
#define SCREEN_OFF		BIT(0)
#define INPUT_BOOST		BIT(1)
#define MAX_BOOST		BIT(2)

struct boost_drv {
	struct workqueue_struct *wq;
	struct delayed_work input_unboost;
	struct delayed_work dynamic_stune_unboost;
	struct delayed_work max_unboost;
	struct notifier_block cpu_notif;
	struct notifier_block msm_drm_notif;
	wait_queue_head_t boost_waitq;
	atomic64_t max_boost_expires;
	atomic_t state;
};

static struct boost_drv *boost_drv_g;

static u32 get_input_boost_freq(struct cpufreq_policy *policy)
{
	u32 freq;

	if (cpumask_test_cpu(policy->cpu, cpu_lp_mask))
		freq = input_boost_freq_lp;
	else
		freq = input_boost_freq_hp;

	return min(freq, policy->max);
}

static u32 get_max_boost_freq(struct cpufreq_policy *policy)
{
	u32 freq;

	if (cpumask_test_cpu(policy->cpu, cpu_lp_mask))
		freq = CONFIG_MAX_BOOST_FREQ_LP;
	else
		freq = CONFIG_MAX_BOOST_FREQ_PERF;

	return min(freq, policy->max);
}

static u32 get_min_freq(struct boost_drv *b, u32 cpu)
{
	if (cpumask_test_cpu(cpu, cpu_lp_mask))
		return remove_input_boost_freq_lp;

	return remove_input_boost_freq_perf;
}

static u32 get_boost_state(struct boost_drv *b)
{
	return atomic_read(&b->state);
}

static void set_boost_bit(struct boost_drv *b, u32 state)
{
	atomic_or(state, &b->state);
}

static void clear_boost_bit(struct boost_drv *b, u32 state)
{
	atomic_andnot(state, &b->state);
}

static void update_online_cpu_policy(void)
{
	u32 cpu;

	/* Only one CPU from each cluster needs to be updated */
	get_online_cpus();
	cpu = cpumask_first_and(cpu_lp_mask, cpu_online_mask);
	cpufreq_update_policy(cpu);
	cpu = cpumask_first_and(cpu_perf_mask, cpu_online_mask);
	cpufreq_update_policy(cpu);
	put_online_cpus();
}

static void __cpu_input_boost_kick(struct boost_drv *b)
{
	if (get_boost_state(b) & SCREEN_OFF)
		return;

	set_boost_bit(b, INPUT_BOOST);
	wake_up(&b->boost_waitq);
	mod_delayed_work(system_unbound_wq, &b->input_unboost,
			 msecs_to_jiffies(CONFIG_INPUT_BOOST_DURATION_MS));
}

void cpu_input_boost_kick(void)
{
	struct boost_drv *b = boost_drv_g;

	if (!b)
		return;

	__cpu_input_boost_kick(b);
}

static void __cpu_input_boost_kick_max(struct boost_drv *b,
	unsigned int duration_ms)
{
	unsigned long boost_jiffies = msecs_to_jiffies(duration_ms);
	unsigned long curr_expires, new_expires;

	if (get_boost_state(b) & SCREEN_OFF)
		return;

	do {
		curr_expires = atomic64_read(&b->max_boost_expires);
		new_expires = jiffies + boost_jiffies;

		/* Skip this boost if there's a longer boost in effect */
		if (time_after(curr_expires, new_expires))
			return;
	} while (atomic64_cmpxchg(&b->max_boost_expires, curr_expires,
				  new_expires) != curr_expires);

	set_boost_bit(b, MAX_BOOST);
	wake_up(&b->boost_waitq);
	mod_delayed_work(system_unbound_wq, &b->max_unboost, boost_jiffies);
}

void cpu_input_boost_kick_max(unsigned int duration_ms)
{
	struct boost_drv *b = boost_drv_g;

	if (!b)
		return;

#ifdef CONFIG_DYNAMIC_STUNE_BOOST
	if (!do_stune_boost("top-app", dynamic_stune_boost, &boost_slot))
		stune_boost_active = true;
#endif

	__cpu_input_boost_kick_max(b, duration_ms);
}

static void input_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b =
		container_of(to_delayed_work(work), typeof(*b), input_unboost);

	clear_boost_bit(b, INPUT_BOOST);
	wake_up(&b->boost_waitq);
}

#ifdef CONFIG_DYNAMIC_STUNE_BOOST
static void dynamic_stune_unboost_worker(struct work_struct *work)
{
	if (stune_boost_active) {
		reset_stune_boost("top-app", boost_slot);
		stune_boost_active = false;
	}
}
#endif

static void max_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(to_delayed_work(work),
					   typeof(*b), max_unboost);

#ifdef CONFIG_DYNAMIC_STUNE_BOOST
	if (stune_boost_active) {
		reset_stune_boost("top-app", boost_slot);
		stune_boost_active = false;
	}
#endif

	clear_boost_bit(b, MAX_BOOST);
	wake_up(&b->boost_waitq);
}

static int cpu_boost_thread(void *data)
{
	static const struct sched_param sched_max_rt_prio = {
		.sched_priority = MAX_RT_PRIO - 1
	};
	struct boost_drv *b = data;
	u32 old_state = 0;

#ifdef CONFIG_DYNAMIC_STUNE_BOOST
	if (!do_stune_boost("top-app", dynamic_stune_boost, &boost_slot))
		stune_boost_active = true;
#endif

	sched_setscheduler_nocheck(current, SCHED_FIFO, &sched_max_rt_prio);

	while (!kthread_should_stop()) {
		u32 curr_state;

		wait_event_interruptible(b->boost_waitq,
			(curr_state = get_boost_state(b)) != old_state ||
			kthread_should_stop());

		old_state = curr_state;
		update_online_cpu_policy();
	}

#ifdef CONFIG_DYNAMIC_STUNE_BOOST
	queue_delayed_work(b->wq, &b->dynamic_stune_unboost,
		msecs_to_jiffies(dynamic_stune_boost_duration));
#endif

	return 0;
}

static int cpu_notifier_cb(struct notifier_block *nb,
	unsigned long action, void *data)
{
	struct boost_drv *b = container_of(nb, typeof(*b), cpu_notif);
	struct cpufreq_policy *policy = data;
	u32 min_freq, state;

	if (action != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	state = get_boost_state(b);

	/* Unboost when the screen is off */
	if (state & SCREEN_OFF) {
		policy->min = policy->cpuinfo.min_freq;
		return NOTIFY_OK;
	}

	/* Boost CPU to max frequency for max boost */
	if (state & MAX_BOOST) {
		policy->min = get_max_boost_freq(policy);
		return NOTIFY_OK;
	}

	/*
	 * Boost to policy->max if the boost frequency is higher. When
	 * unboosting, set policy->min to the absolute min freq for the CPU.
	 */
	if (state & INPUT_BOOST)
		policy->min = get_input_boost_freq(policy);
	else {
		min_freq = get_min_freq(b, policy->cpu);
		policy->min = max(policy->cpuinfo.min_freq, min_freq);
	}

	return NOTIFY_OK;
}

static int msm_drm_notifier_cb(struct notifier_block *nb,
			  unsigned long action, void *data)
{
	struct boost_drv *b = container_of(nb, typeof(*b), msm_drm_notif);
	struct msm_drm_notifier *evdata = data;
	int *blank = evdata->data;

	/* Parse framebuffer blank events as soon as they occur */
	if (action != MSM_DRM_EARLY_EVENT_BLANK)
		return NOTIFY_OK;

	/* Boost when the screen turns on and unboost when it turns off */
	if (*blank == MSM_DRM_BLANK_UNBLANK) {
		clear_boost_bit(b, SCREEN_OFF);
		__cpu_input_boost_kick_max(b, CONFIG_WAKE_BOOST_DURATION_MS);
	} else {
		set_boost_bit(b, SCREEN_OFF);
		wake_up(&b->boost_waitq);
	}

	return NOTIFY_OK;
}

static void cpu_input_boost_input_event(struct input_handle *handle,
	unsigned int type, unsigned int code, int value)
{
	struct boost_drv *b = handle->handler->private;

	__cpu_input_boost_kick(b);
}

static int cpu_input_boost_input_connect(struct input_handler *handler,
	struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpu_input_boost_handle";

	ret = input_register_handle(handle);
	if (ret)
		goto free_handle;

	ret = input_open_device(handle);
	if (ret)
		goto unregister_handle;

	return 0;

unregister_handle:
	input_unregister_handle(handle);
free_handle:
	kfree(handle);
	return ret;
}

static void cpu_input_boost_input_disconnect(struct input_handle *handle)
{
#ifdef CONFIG_DYNAMIC_STUNE_BOOST
	if (stune_boost_active) {
		reset_stune_boost("top-app", boost_slot);
		stune_boost_active = false;
	}
#endif
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpu_input_boost_ids[] = {
	/* Multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* Touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ }
};

static struct input_handler cpu_input_boost_input_handler = {
	.event		= cpu_input_boost_input_event,
	.connect	= cpu_input_boost_input_connect,
	.disconnect	= cpu_input_boost_input_disconnect,
	.name		= "cpu_input_boost_handler",
	.id_table	= cpu_input_boost_ids
};

static int __init cpu_input_boost_init(void)
{
	struct task_struct *boost_thread;
	struct boost_drv *b;
	int ret;

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	INIT_DELAYED_WORK(&b->input_unboost, input_unboost_worker);
	INIT_DELAYED_WORK(&b->dynamic_stune_unboost, dynamic_stune_unboost_worker);
	INIT_DELAYED_WORK(&b->max_unboost, max_unboost_worker);
	init_waitqueue_head(&b->boost_waitq);
	atomic64_set(&b->max_boost_expires, 0);
	atomic_set(&b->state, 0);

	b->cpu_notif.notifier_call = cpu_notifier_cb;
	ret = cpufreq_register_notifier(&b->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		pr_err("Failed to register cpufreq notifier, err: %d\n", ret);
		goto free_b;
	}

	cpu_input_boost_input_handler.private = b;
	ret = input_register_handler(&cpu_input_boost_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, err: %d\n", ret);
		goto unregister_cpu_notif;
	}

	b->msm_drm_notif.notifier_call = msm_drm_notifier_cb;
	b->msm_drm_notif.priority = INT_MAX;
	ret = msm_drm_register_client(&b->msm_drm_notif);
	if (ret) {
		pr_err("Failed to register dsi_panel_notifier, err: %d\n", ret);
		goto unregister_handler;
	}

	boost_thread = kthread_run(cpu_boost_thread, b, "cpu_boostd");
	if (IS_ERR(boost_thread)) {
		pr_err("Failed to start CPU boost thread, err: %ld\n",
		       PTR_ERR(boost_thread));
		goto unregister_drm_notif;
	}

	boost_drv_g = b;

	return 0;

unregister_drm_notif:
	msm_drm_unregister_client(&b->msm_drm_notif);
unregister_handler:
	input_unregister_handler(&cpu_input_boost_input_handler);
unregister_cpu_notif:
	cpufreq_unregister_notifier(&b->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
free_b:
	kfree(b);
	return ret;
}
late_initcall(cpu_input_boost_init);