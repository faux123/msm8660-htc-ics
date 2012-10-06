/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/cpuidle.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/pm.h>
#include <linux/pm_qos_params.h>
#include <linux/proc_fs.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/tick.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/seq_file.h>
#include <linux/console.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>
#include <mach/gpio.h>
#include <mach/board_htc.h>
#include <mach/msm_watchdog.h>
#include <mach/rpm.h>
#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#ifdef CONFIG_ARCH_MSM8X60_LTE
#include <mach/board_htc.h>
#include <mach/htc_util.h>
#endif
#ifdef CONFIG_VFP
#include <asm/vfp.h>
#endif

#include "acpuclock.h"
#include "clock.h"
#include "avs.h"
#include "cpuidle.h"
#include "idle.h"
#include "pm.h"
#include "rpm_resources.h"
#include "scm-boot.h"
#include "spm.h"
#include "timer.h"
#include "qdss.h"
#include "pm-boot.h"

/******************************************************************************
 * Debug Definitions
 *****************************************************************************/

enum {
	MSM_PM_DEBUG_SUSPEND = BIT(0),
	MSM_PM_DEBUG_POWER_COLLAPSE = BIT(1),
	MSM_PM_DEBUG_SUSPEND_LIMITS = BIT(2),
	MSM_PM_DEBUG_CLOCK = BIT(3),
	MSM_PM_DEBUG_RESET_VECTOR = BIT(4),
	MSM_PM_DEBUG_IDLE_CLK = BIT(5),
	MSM_PM_DEBUG_IDLE = BIT(6),
	MSM_PM_DEBUG_IDLE_LIMITS = BIT(7),
	MSM_PM_DEBUG_HOTPLUG = BIT(8),
	MSM_PM_DEBUG_GPIO = BIT(9),
	MSM_PM_DEBUG_RPM_TIMESTAMP = BIT(10),
	MSM_PM_DEBUG_IDLE_CLOCK = BIT(11),
	MSM_PM_DEBUG_RPM_STAT = BIT(12),
};

static int msm_pm_debug_mask = MSM_PM_DEBUG_SUSPEND | MSM_PM_DEBUG_SUSPEND_LIMITS
	| MSM_PM_DEBUG_RPM_TIMESTAMP | MSM_PM_DEBUG_RPM_STAT|MSM_PM_DEBUG_CLOCK;

module_param_named(
	debug_mask, msm_pm_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

int board_mfg_mode(void);
/******************************************************************************
 * Sleep Modes and Parameters
 *****************************************************************************/

static struct msm_pm_platform_data *msm_pm_modes;
static int rpm_cpu0_wakeup_irq;

void __init msm_pm_set_platform_data(
	struct msm_pm_platform_data *data, int count)
{
	BUG_ON(MSM_PM_SLEEP_MODE_NR * num_possible_cpus() > count);
	msm_pm_modes = data;
}

void __init msm_pm_set_rpm_wakeup_irq(unsigned int irq)
{
	rpm_cpu0_wakeup_irq = irq;
}

enum {
	MSM_PM_MODE_ATTR_SUSPEND,
	MSM_PM_MODE_ATTR_IDLE,
	MSM_PM_MODE_ATTR_NR,
};

static char *msm_pm_mode_attr_labels[MSM_PM_MODE_ATTR_NR] = {
	[MSM_PM_MODE_ATTR_SUSPEND] = "suspend_enabled",
	[MSM_PM_MODE_ATTR_IDLE] = "idle_enabled",
};

struct msm_pm_kobj_attribute {
	unsigned int cpu;
	struct kobj_attribute ka;
};

#define GET_CPU_OF_ATTR(attr) \
	(container_of(attr, struct msm_pm_kobj_attribute, ka)->cpu)

struct msm_pm_sysfs_sleep_mode {
	struct kobject *kobj;
	struct attribute_group attr_group;
	struct attribute *attrs[MSM_PM_MODE_ATTR_NR + 1];
	struct msm_pm_kobj_attribute kas[MSM_PM_MODE_ATTR_NR];
};

static char *msm_pm_sleep_mode_labels[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = "power_collapse",
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = "wfi",
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE] =
		"standalone_power_collapse",
};

/*
 * Write out the attribute.
 */
static ssize_t msm_pm_mode_attr_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = -EINVAL;
	int i;

	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		struct kernel_param kp;
		unsigned int cpu;
		struct msm_pm_platform_data *mode;

		if (msm_pm_sleep_mode_labels[i] == NULL)
			continue;

		if (strcmp(kobj->name, msm_pm_sleep_mode_labels[i]))
			continue;

		cpu = GET_CPU_OF_ATTR(attr);
		mode = &msm_pm_modes[MSM_PM_MODE(cpu, i)];

		if (!strcmp(attr->attr.name,
			msm_pm_mode_attr_labels[MSM_PM_MODE_ATTR_SUSPEND])) {
			u32 arg = mode->suspend_enabled;
			kp.arg = &arg;
			ret = param_get_ulong(buf, &kp);
		} else if (!strcmp(attr->attr.name,
			msm_pm_mode_attr_labels[MSM_PM_MODE_ATTR_IDLE])) {
			u32 arg = mode->idle_enabled;
			kp.arg = &arg;
			ret = param_get_ulong(buf, &kp);
		}

		break;
	}

	if (ret > 0) {
		strlcat(buf, "\n", PAGE_SIZE);
		ret++;
	}

	return ret;
}

/*
 * Read in the new attribute value.
 */
static ssize_t msm_pm_mode_attr_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = -EINVAL;
	int i;

	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		struct kernel_param kp;
		unsigned int cpu;
		struct msm_pm_platform_data *mode;

		if (msm_pm_sleep_mode_labels[i] == NULL)
			continue;

		if (strcmp(kobj->name, msm_pm_sleep_mode_labels[i]))
			continue;

		cpu = GET_CPU_OF_ATTR(attr);
		mode = &msm_pm_modes[MSM_PM_MODE(cpu, i)];

		if (!strcmp(attr->attr.name,
			msm_pm_mode_attr_labels[MSM_PM_MODE_ATTR_SUSPEND])) {
			kp.arg = &mode->suspend_enabled;
			ret = param_set_byte(buf, &kp);
		} else if (!strcmp(attr->attr.name,
			msm_pm_mode_attr_labels[MSM_PM_MODE_ATTR_IDLE])) {
			kp.arg = &mode->idle_enabled;
			ret = param_set_byte(buf, &kp);
		}

		break;
	}

	return ret ? ret : count;
}

/*
 * Add sysfs entries for one cpu.
 */
static int __init msm_pm_mode_sysfs_add_cpu(
	unsigned int cpu, struct kobject *modes_kobj)
{
	char cpu_name[8];
	struct kobject *cpu_kobj;
	struct msm_pm_sysfs_sleep_mode *mode = NULL;
	int i, j, k;
	int ret;

	snprintf(cpu_name, sizeof(cpu_name), "cpu%u", cpu);
	cpu_kobj = kobject_create_and_add(cpu_name, modes_kobj);
	if (!cpu_kobj) {
		pr_err("%s: cannot create %s kobject\n", __func__, cpu_name);
		ret = -ENOMEM;
		goto mode_sysfs_add_cpu_exit;
	}

	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		int idx = MSM_PM_MODE(cpu, i);

		if ((!msm_pm_modes[idx].suspend_supported)
			&& (!msm_pm_modes[idx].idle_supported))
			continue;

		mode = kzalloc(sizeof(*mode), GFP_KERNEL);
		if (!mode) {
			pr_err("%s: cannot allocate memory for attributes\n",
				__func__);
			ret = -ENOMEM;
			goto mode_sysfs_add_cpu_exit;
		}

		mode->kobj = kobject_create_and_add(
				msm_pm_sleep_mode_labels[i], cpu_kobj);
		if (!mode->kobj) {
			pr_err("%s: cannot create kobject\n", __func__);
			ret = -ENOMEM;
			goto mode_sysfs_add_cpu_exit;
		}

		for (k = 0, j = 0; k < MSM_PM_MODE_ATTR_NR; k++) {
			if ((k == MSM_PM_MODE_ATTR_IDLE) &&
				!msm_pm_modes[idx].idle_supported)
				continue;
			if ((k == MSM_PM_MODE_ATTR_SUSPEND) &&
			     !msm_pm_modes[idx].suspend_supported)
				continue;
			mode->kas[j].cpu = cpu;
			mode->kas[j].ka.attr.mode = 0644;
			mode->kas[j].ka.show = msm_pm_mode_attr_show;
			mode->kas[j].ka.store = msm_pm_mode_attr_store;
			mode->kas[j].ka.attr.name = msm_pm_mode_attr_labels[k];
			mode->attrs[j] = &mode->kas[j].ka.attr;
			j++;
		}
		mode->attrs[j] = NULL;

		mode->attr_group.attrs = mode->attrs;
		ret = sysfs_create_group(mode->kobj, &mode->attr_group);
		if (ret) {
			pr_err("%s: cannot create kobject attribute group\n",
				__func__);
			goto mode_sysfs_add_cpu_exit;
		}
	}

	ret = 0;

mode_sysfs_add_cpu_exit:
	if (ret) {
		if (mode && mode->kobj)
			kobject_del(mode->kobj);
		kfree(mode);
	}

	return ret;
}

/*
 * Add sysfs entries for the sleep modes.
 */
static int __init msm_pm_mode_sysfs_add(void)
{
	struct kobject *module_kobj;
	struct kobject *modes_kobj;
	unsigned int cpu;
	int ret;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("%s: cannot find kobject for module %s\n",
			__func__, KBUILD_MODNAME);
		ret = -ENOENT;
		goto mode_sysfs_add_exit;
	}

	modes_kobj = kobject_create_and_add("modes", module_kobj);
	if (!modes_kobj) {
		pr_err("%s: cannot create modes kobject\n", __func__);
		ret = -ENOMEM;
		goto mode_sysfs_add_exit;
	}

	for_each_possible_cpu(cpu) {
		ret = msm_pm_mode_sysfs_add_cpu(cpu, modes_kobj);
		if (ret)
			goto mode_sysfs_add_exit;
	}

	ret = 0;

mode_sysfs_add_exit:
	return ret;
}

/******************************************************************************
 * CONFIG_MSM_IDLE_STATS
 *****************************************************************************/

#ifdef CONFIG_MSM_IDLE_STATS
enum msm_pm_time_stats_id {
	MSM_PM_STAT_REQUESTED_IDLE,
	MSM_PM_STAT_IDLE_WFI,
	MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE,
	MSM_PM_STAT_IDLE_POWER_COLLAPSE,
	MSM_PM_STAT_SUSPEND,
	MSM_PM_STAT_COUNT
};

struct msm_pm_time_stats {
	const char *name;
	int64_t first_bucket_time;
	int bucket[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t min_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t max_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int count;
	int64_t total_time;
};

struct msm_pm_cpu_time_stats {
	struct msm_pm_time_stats stats[MSM_PM_STAT_COUNT];
};

static DEFINE_SPINLOCK(msm_pm_stats_lock);
static DEFINE_PER_CPU_SHARED_ALIGNED(
	struct msm_pm_cpu_time_stats, msm_pm_stats);

/*
 * Add the given time data to the statistics collection.
 */
static void msm_pm_add_stat(enum msm_pm_time_stats_id id, int64_t t)
{
	unsigned long flags;
	struct msm_pm_time_stats *stats;
	int64_t bt;
	int i;

	spin_lock_irqsave(&msm_pm_stats_lock, flags);
	stats = __get_cpu_var(msm_pm_stats).stats;

	stats[id].total_time += t;
	stats[id].count++;

	bt = t;
	do_div(bt, stats[id].first_bucket_time);

	if (bt < 1ULL << (CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT *
				(CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1)))
		i = DIV_ROUND_UP(fls((uint32_t)bt),
					CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT);
	else
		i = CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1;

	if (i >= CONFIG_MSM_IDLE_STATS_BUCKET_COUNT)
		i = CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1;

	stats[id].bucket[i]++;

	if (t < stats[id].min_time[i] || !stats[id].max_time[i])
		stats[id].min_time[i] = t;
	if (t > stats[id].max_time[i])
		stats[id].max_time[i] = t;

	spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
}

/*
 * Helper function of snprintf where buf is auto-incremented, size is auto-
 * decremented, and there is no return value.
 *
 * NOTE: buf and size must be l-values (e.g. variables)
 */
#define SNPRINTF(buf, size, format, ...) \
	do { \
		if (size > 0) { \
			int ret; \
			ret = snprintf(buf, size, format, ## __VA_ARGS__); \
			if (ret > size) { \
				buf += size; \
				size = 0; \
			} else { \
				buf += ret; \
				size -= ret; \
			} \
		} \
	} while (0)

/*
 * Write out the power management statistics.
 */
static int msm_pm_read_proc
	(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	unsigned int cpu = off / MSM_PM_STAT_COUNT;
	int id = off % MSM_PM_STAT_COUNT;
	char *p = page;

	if (count < 1024) {
		*start = (char *) 0;
		*eof = 0;
		return 0;
	}

	if (cpu < num_possible_cpus()) {
		unsigned long flags;
		struct msm_pm_time_stats *stats;
		int i;
		int64_t bucket_time;
		int64_t s;
		uint32_t ns;

		spin_lock_irqsave(&msm_pm_stats_lock, flags);
		stats = per_cpu(msm_pm_stats, cpu).stats;

		s = stats[id].total_time;
		ns = do_div(s, NSEC_PER_SEC);
		SNPRINTF(p, count,
			"[cpu %u] %s:\n"
			"  count: %7d\n"
			"  total_time: %lld.%09u\n",
			cpu, stats[id].name,
			stats[id].count,
			s, ns);

		bucket_time = stats[id].first_bucket_time;
		for (i = 0; i < CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1; i++) {
			s = bucket_time;
			ns = do_div(s, NSEC_PER_SEC);
			SNPRINTF(p, count,
				"   <%6lld.%09u: %7d (%lld-%lld)\n",
				s, ns, stats[id].bucket[i],
				stats[id].min_time[i],
				stats[id].max_time[i]);

			bucket_time <<= CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT;
		}

		SNPRINTF(p, count, "  >=%6lld.%09u: %7d (%lld-%lld)\n",
			s, ns, stats[id].bucket[i],
			stats[id].min_time[i],
			stats[id].max_time[i]);

		*start = (char *) 1;
		*eof = (off + 1 >= MSM_PM_STAT_COUNT * num_possible_cpus());

		spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
	}

	return p - page;
}
#undef SNPRINTF

#define MSM_PM_STATS_RESET "reset"

/*
 * Reset the power management statistics values.
 */
static int msm_pm_write_proc(struct file *file, const char __user *buffer,
	unsigned long count, void *data)
{
	char buf[sizeof(MSM_PM_STATS_RESET)];
	int ret;
	unsigned long flags;
	unsigned int cpu;

	if (count < strlen(MSM_PM_STATS_RESET)) {
		ret = -EINVAL;
		goto write_proc_failed;
	}

	if (copy_from_user(buf, buffer, strlen(MSM_PM_STATS_RESET))) {
		ret = -EFAULT;
		goto write_proc_failed;
	}

	if (memcmp(buf, MSM_PM_STATS_RESET, strlen(MSM_PM_STATS_RESET))) {
		ret = -EINVAL;
		goto write_proc_failed;
	}

	spin_lock_irqsave(&msm_pm_stats_lock, flags);
	for_each_possible_cpu(cpu) {
		struct msm_pm_time_stats *stats;
		int i;

		stats = per_cpu(msm_pm_stats, cpu).stats;
		for (i = 0; i < MSM_PM_STAT_COUNT; i++) {
			memset(stats[i].bucket,
				0, sizeof(stats[i].bucket));
			memset(stats[i].min_time,
				0, sizeof(stats[i].min_time));
			memset(stats[i].max_time,
				0, sizeof(stats[i].max_time));
			stats[i].count = 0;
			stats[i].total_time = 0;
		}
	}

	spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
	return count;

write_proc_failed:
	return ret;
}
#undef MSM_PM_STATS_RESET
#endif /* CONFIG_MSM_IDLE_STATS */


/******************************************************************************
 * Configure Hardware before/after Low Power Mode
 *****************************************************************************/

/*
 * Configure hardware registers in preparation for Apps power down.
 */
static void msm_pm_config_hw_before_power_down(void)
{
	return;
}

/*
 * Clear hardware registers after Apps powers up.
 */
static void msm_pm_config_hw_after_power_up(void)
{
	return;
}

/*
 * Configure hardware registers in preparation for SWFI.
 */
static void msm_pm_config_hw_before_swfi(void)
{
	return;
}


/******************************************************************************
 * Suspend Max Sleep Time
 *****************************************************************************/

#ifdef CONFIG_MSM_SLEEP_TIME_OVERRIDE
static int msm_pm_sleep_time_override;
module_param_named(sleep_time_override,
	msm_pm_sleep_time_override, int, S_IRUGO | S_IWUSR | S_IWGRP);
#endif

#define SCLK_HZ (32768)
#define MSM_PM_SLEEP_TICK_LIMIT (0x6DDD000)

static uint32_t msm_pm_max_sleep_time;

/*
 * Convert time from nanoseconds to slow clock ticks, then cap it to the
 * specified limit
 */
static int64_t msm_pm_convert_and_cap_time(int64_t time_ns, int64_t limit)
{
	do_div(time_ns, NSEC_PER_SEC / SCLK_HZ);
	return (time_ns > limit) ? limit : time_ns;
}

/*
 * Set the sleep time for suspend.  0 means infinite sleep time.
 */
void msm_pm_set_max_sleep_time(int64_t max_sleep_time_ns)
{
	if (max_sleep_time_ns == 0) {
		msm_pm_max_sleep_time = 0;
	} else {
		msm_pm_max_sleep_time = (uint32_t)msm_pm_convert_and_cap_time(
			max_sleep_time_ns, MSM_PM_SLEEP_TICK_LIMIT);

		if (msm_pm_max_sleep_time == 0)
			msm_pm_max_sleep_time = 1;
	}

	if (msm_pm_debug_mask & MSM_PM_DEBUG_SUSPEND)
		pr_info("%s: Requested %lld ns Giving %u sclk ticks\n",
			__func__, max_sleep_time_ns, msm_pm_max_sleep_time);
}
EXPORT_SYMBOL(msm_pm_set_max_sleep_time);


/******************************************************************************
 *
 *****************************************************************************/

static struct msm_rpmrs_limits *msm_pm_idle_rs_limits;

static void msm_pm_swfi(void)
{
	msm_pm_config_hw_before_swfi();
	msm_arch_idle();
}

static bool msm_pm_spm_power_collapse(
	unsigned int cpu, bool from_idle, bool notify_rpm)
{
	void *entry;
	bool collapsed = 0;
	int ret;

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: notify_rpm %d\n",
			cpu, __func__, (int) notify_rpm);

	ret = msm_spm_set_low_power_mode(
			MSM_SPM_MODE_POWER_COLLAPSE, notify_rpm);
	WARN_ON(ret);

	entry = (!cpu || from_idle) ?
		msm_pm_collapse_exit : msm_secondary_startup;
	msm_pm_boot_config_before_pc(cpu, virt_to_phys(entry));

	if (MSM_PM_DEBUG_RESET_VECTOR & msm_pm_debug_mask)
		pr_info("CPU%u: %s: program vector to %p\n",
			cpu, __func__, entry);

#ifdef CONFIG_VFP
	vfp_flush_context();
#endif

	if (!from_idle && smp_processor_id() == 0) {
		if (suspend_console_deferred)
			suspend_console();

		msm_rpm_set_suspend_flag(true);

#ifdef CONFIG_MSM_WATCHDOG
		msm_watchdog_suspend(NULL);
#endif

		printk(KERN_INFO "[R] suspend end\n");
	}
	collapsed = msm_pm_collapse();

	msm_pm_boot_config_after_pc(cpu);

	if (!from_idle && smp_processor_id() == 0) {
		printk(KERN_INFO "[R] resume start\n");

#ifdef CONFIG_MSM_WATCHDOG
		msm_watchdog_resume(NULL);
#endif

		msm_rpm_set_suspend_flag(false);

		if (suspend_console_deferred)
			resume_console();
	}

	if (collapsed) {
#ifdef CONFIG_VFP
		vfp_reinit();
#endif
		cpu_init();
		writel(0xF0, MSM_QGIC_CPU_BASE + GIC_CPU_PRIMASK);
		writel(1, MSM_QGIC_CPU_BASE + GIC_CPU_CTRL);
		local_fiq_enable();
	}

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: msm_pm_collapse returned, collapsed %d\n",
			cpu, __func__, collapsed);

	if (MSM_PM_DEBUG_RPM_TIMESTAMP & msm_pm_debug_mask && !from_idle)
		msm_rpm_print_sleep_tick();

	ret = msm_spm_set_low_power_mode(MSM_SPM_MODE_CLOCK_GATING, false);
	WARN_ON(ret);
	return collapsed;
}

static bool msm_pm_power_collapse_standalone(bool from_idle)
{
	unsigned int cpu = smp_processor_id();
	unsigned int avsdscr_setting;
	bool collapsed;

	avsdscr_setting = avs_get_avsdscr();
	avs_disable();
	collapsed = msm_pm_spm_power_collapse(cpu, from_idle, false);
	avs_reset_delays(avsdscr_setting);
	return collapsed;
}

static bool msm_pm_power_collapse(bool from_idle)
{
	unsigned int cpu = smp_processor_id();
	unsigned long saved_acpuclk_rate;
	unsigned int avsdscr_setting;
	bool collapsed;
	struct clk *clk, *parent_clk;
	int blk_xo_vddmin_count = 0;

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: idle %d\n",
			cpu, __func__, (int)from_idle);

	if (smp_processor_id() == 0) {
		if (((!from_idle) && (MSM_PM_DEBUG_CLOCK & msm_pm_debug_mask)) ||
			((from_idle) && (MSM_PM_DEBUG_IDLE_CLOCK & msm_pm_debug_mask))) {
			spin_lock(&clk_enable_list_lock);
			list_for_each_entry(clk, &clk_enable_list, enable_list) {
				if (clk->ops->is_local && clk->ops->is_local(clk))
					if (is_xo_src(clk)) {
						blk_xo_vddmin_count++;
						parent_clk = clk->ops->get_parent?clk->ops->get_parent(clk):NULL;
						if (clk->vdd_class)
							pr_info("%s not off block xo vdig level %lu, parent clk: %s\n",
									clk->dbg_name, clk->vdd_class->cur_level,
									parent_clk?parent_clk->dbg_name:"none");
						else
							pr_info("%s not off block xo vdig level (none), parent clk: %s\n",
									clk->dbg_name, parent_clk?parent_clk->dbg_name:"none");
					}
			}
			spin_unlock(&clk_enable_list_lock);
			if (blk_xo_vddmin_count)
				pr_info("%d clks are on that block xo or vddmin\n", blk_xo_vddmin_count);
		}
		if ((!from_idle) && (MSM_PM_DEBUG_RPM_STAT & msm_pm_debug_mask))
			msm_rpm_dump_stat();
	}

	msm_pm_config_hw_before_power_down();
	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: pre power down\n", cpu, __func__);

	avsdscr_setting = avs_get_avsdscr();
	avs_disable();

	if (cpu_online(cpu))
		saved_acpuclk_rate = acpuclk_power_collapse();
	else
		saved_acpuclk_rate = 0;

	if ((!from_idle) && (MSM_PM_DEBUG_CLOCK & msm_pm_debug_mask))
		pr_info("CPU%u: %s: change clock rate (old rate = %lu)\n",
			cpu, __func__, saved_acpuclk_rate);

	collapsed = msm_pm_spm_power_collapse(cpu, from_idle, true);

	if ((!from_idle) && (MSM_PM_DEBUG_RPM_STAT & msm_pm_debug_mask))
			msm_rpm_dump_stat();

	if ((!from_idle) && (MSM_PM_DEBUG_CLOCK & msm_pm_debug_mask))
		pr_info("CPU%u: %s: restore clock rate to %lu\n",
			cpu, __func__, saved_acpuclk_rate);
	if (acpuclk_set_rate(cpu, saved_acpuclk_rate, SETRATE_PC) < 0)
		pr_err("CPU%u: %s: failed to restore clock rate(%lu)\n",
			cpu, __func__, saved_acpuclk_rate);

	avs_reset_delays(avsdscr_setting);
	msm_pm_config_hw_after_power_up();
	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: post power up\n", cpu, __func__);

	if (MSM_PM_DEBUG_POWER_COLLAPSE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: return\n", cpu, __func__);
	return collapsed;
}

static irqreturn_t msm_pm_rpm_wakeup_interrupt(int irq, void *dev_id)
{
	if (dev_id != &msm_pm_rpm_wakeup_interrupt)
		return IRQ_NONE;

	return IRQ_HANDLED;
}


/******************************************************************************
 * External Idle/Suspend Functions
 *****************************************************************************/

void arch_idle(void)
{
	return;
}

int msm_pm_idle_prepare(struct cpuidle_device *dev)
{
	uint32_t latency_us;
	uint32_t sleep_us;
	int i;

	latency_us = (uint32_t) pm_qos_request(PM_QOS_CPU_DMA_LATENCY);
	sleep_us = (uint32_t) ktime_to_ns(tick_nohz_get_sleep_length());
	sleep_us = DIV_ROUND_UP(sleep_us, 1000);

	for (i = 0; i < dev->state_count; i++) {
		struct cpuidle_state *state = &dev->states[i];
		enum msm_pm_sleep_mode mode;
		bool allow;
		struct msm_rpmrs_limits *rs_limits = NULL;
		int idx;

		mode = (enum msm_pm_sleep_mode) state->driver_data;
		idx = MSM_PM_MODE(dev->cpu, mode);

		allow = msm_pm_modes[idx].idle_enabled &&
				msm_pm_modes[idx].idle_supported;

		switch (mode) {
		case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
			if (!allow)
				break;

			if (num_online_cpus() > 1) {
				allow = false;
				break;
			}
#ifdef CONFIG_HAS_WAKELOCK
			if (has_wake_lock(WAKE_LOCK_IDLE)) {
				allow = false;
				break;
			}
#endif
			/* fall through */

		case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE:
			if (!allow)
				break;

			if (!dev->cpu &&
				msm_rpm_local_request_is_outstanding()) {
				allow = false;
				break;
			}
			/* fall through */

		case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
			if (!allow)
				break;

			rs_limits = msm_rpmrs_lowest_limits(true,
						mode, latency_us, sleep_us);

			if (MSM_PM_DEBUG_IDLE & msm_pm_debug_mask)
				pr_info("CPU%u: %s: %s, latency %uus, "
					"sleep %uus, limit %p\n",
					dev->cpu, __func__, state->desc,
					latency_us, sleep_us, rs_limits);

			if ((MSM_PM_DEBUG_IDLE_LIMITS & msm_pm_debug_mask) &&
					rs_limits)
				pr_info("CPU%u: %s: limit %p: "
					"pxo %d, l2_cache %d, "
					"vdd_mem %d, vdd_dig %d\n",
					dev->cpu, __func__, rs_limits,
					rs_limits->pxo,
					rs_limits->l2_cache,
					rs_limits->vdd_mem,
					rs_limits->vdd_dig);

			if (!rs_limits)
				allow = false;
			break;

		default:
			allow = false;
			break;
		}

		if (MSM_PM_DEBUG_IDLE & msm_pm_debug_mask)
			pr_info("CPU%u: %s: allow %s: %d\n",
				dev->cpu, __func__, state->desc, (int)allow);

		if (allow) {
			state->flags &= ~CPUIDLE_FLAG_IGNORE;
			state->target_residency = 0;
			state->exit_latency = 0;
			state->power_usage = rs_limits->power[dev->cpu];

			if (MSM_PM_SLEEP_MODE_POWER_COLLAPSE == mode)
				msm_pm_idle_rs_limits = rs_limits;
		} else {
			state->flags |= CPUIDLE_FLAG_IGNORE;
		}
	}

	return 0;
}
static char *gpio_sleep_status_info;

int print_gpio_buffer(struct seq_file *m)
{
	if (gpio_sleep_status_info)
		seq_printf(m, gpio_sleep_status_info);
	else
		seq_printf(m, "Device haven't suspended yet!\n");

	return 0;
}
EXPORT_SYMBOL(print_gpio_buffer);

int free_gpio_buffer()
{
	kfree(gpio_sleep_status_info);
	gpio_sleep_status_info = NULL;

	return 0;
}
EXPORT_SYMBOL(free_gpio_buffer);
int msm_pm_idle_enter(enum msm_pm_sleep_mode sleep_mode)
{
	int64_t time;
#ifdef CONFIG_MSM_IDLE_STATS
	int exit_stat;
#endif

	if (MSM_PM_DEBUG_IDLE & msm_pm_debug_mask)
		pr_info("CPU%u: %s: mode %d\n",
			smp_processor_id(), __func__, sleep_mode);

	time = ktime_to_ns(ktime_get());

	switch (sleep_mode) {
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		msm_pm_swfi();
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_WFI;
#endif
		break;

	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE:
		msm_pm_power_collapse_standalone(true);
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE;
#endif
		break;

	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE: {
		int64_t timer_expiration = msm_timer_enter_idle();
		bool timer_halted = false;
		uint32_t sleep_delay;
		int ret;
		int notify_rpm =
			(sleep_mode == MSM_PM_SLEEP_MODE_POWER_COLLAPSE);
		int collapsed;

		sleep_delay = (uint32_t) msm_pm_convert_and_cap_time(
			timer_expiration, MSM_PM_SLEEP_TICK_LIMIT);
		if (sleep_delay == 0) /* 0 would mean infinite time */
			sleep_delay = 1;

		if (MSM_PM_DEBUG_IDLE_CLK & msm_pm_debug_mask)
			clock_debug_print_enabled();

		ret = msm_rpmrs_enter_sleep(
			sleep_delay, msm_pm_idle_rs_limits, true, notify_rpm);
		if (!ret) {
			collapsed = msm_pm_power_collapse(true);
			timer_halted = true;

			msm_rpmrs_exit_sleep(msm_pm_idle_rs_limits, true,
					notify_rpm, collapsed);
		}

		msm_timer_exit_idle((int) timer_halted);
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_POWER_COLLAPSE;
#endif
		break;
	}

	default:
		__WARN();
		goto cpuidle_enter_bail;
	}

	time = ktime_to_ns(ktime_get()) - time;
#ifdef CONFIG_MSM_IDLE_STATS
	msm_pm_add_stat(exit_stat, time);
#ifdef CONFIG_ARCH_MSM8X60_LTE
	htc_idle_stat_add(sleep_mode, (u32)time/1000);
#endif
#endif

	do_div(time, 1000);
	return (int) time;

cpuidle_enter_bail:
	return 0;
}

void msm_pm_cpu_enter_lowpower(unsigned int cpu)
{
	int i;
	bool allow[MSM_PM_SLEEP_MODE_NR];

	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		struct msm_pm_platform_data *mode;

		mode = &msm_pm_modes[MSM_PM_MODE(cpu, i)];
		allow[i] = mode->suspend_supported && mode->suspend_enabled;
	}

	if (MSM_PM_DEBUG_HOTPLUG & msm_pm_debug_mask)
		pr_notice("CPU%u: %s: shutting down cpu\n", cpu, __func__);

	if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE])
		msm_pm_power_collapse(false);
	else if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE])
		msm_pm_power_collapse_standalone(false);
	else if (allow[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT])
		msm_pm_swfi();


}

static int msm_pm_enter(suspend_state_t state)
{
	bool allow[MSM_PM_SLEEP_MODE_NR];
	int i;
	int curr_len = 0;
#ifdef CONFIG_MSM_IDLE_STATS
	int64_t period = 0;
	int64_t time = msm_timer_get_sclk_time(&period);
#endif

	if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
		pr_info("%s\n", __func__);

	if (MSM_PM_DEBUG_GPIO & msm_pm_debug_mask) {
		if (gpio_sleep_status_info) {
			memset(gpio_sleep_status_info, 0,
				sizeof(gpio_sleep_status_info));
		} else {
			gpio_sleep_status_info = kmalloc(25000, GFP_ATOMIC);
			if (!gpio_sleep_status_info) {
				pr_err("[PM] kmalloc memory failed in %s\n",
					__func__);

			}
		}

		curr_len = msm_dump_gpios(NULL, curr_len,
						gpio_sleep_status_info);
		curr_len = pm8xxx_dump_gpios(NULL, curr_len,
						gpio_sleep_status_info);
		curr_len = pm8xxx_dump_mpp(NULL, curr_len,
						gpio_sleep_status_info);

	}

	if (smp_processor_id()) {
		__WARN();
		goto enter_exit;
	}


	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		struct msm_pm_platform_data *mode;

		mode = &msm_pm_modes[MSM_PM_MODE(0, i)];
		allow[i] = mode->suspend_supported && mode->suspend_enabled;
	}

	if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE]) {
		struct msm_rpmrs_limits *rs_limits;
		int ret;

		if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
			pr_info("%s: power collapse\n", __func__);

/*		clock_debug_print_enabled();	*/

#ifdef CONFIG_MSM_SLEEP_TIME_OVERRIDE
		if (msm_pm_sleep_time_override > 0) {
			int64_t ns = NSEC_PER_SEC *
				(int64_t) msm_pm_sleep_time_override;
			msm_pm_set_max_sleep_time(ns);
			msm_pm_sleep_time_override = 0;
		}
#endif /* CONFIG_MSM_SLEEP_TIME_OVERRIDE */

		if (MSM_PM_DEBUG_SUSPEND_LIMITS & msm_pm_debug_mask)
			msm_rpmrs_show_resources();

		rs_limits = msm_rpmrs_lowest_limits(false,
				MSM_PM_SLEEP_MODE_POWER_COLLAPSE, -1, -1);

		if ((MSM_PM_DEBUG_SUSPEND_LIMITS & msm_pm_debug_mask) &&
				rs_limits)
			pr_info("%s: limit %p: pxo %d, l2_cache %d, "
				"vdd_mem %d, vdd_dig %d\n",
				__func__, rs_limits,
				rs_limits->pxo, rs_limits->l2_cache,
				rs_limits->vdd_mem, rs_limits->vdd_dig);

		if (rs_limits) {
			ret = msm_rpmrs_enter_sleep(
				msm_pm_max_sleep_time, rs_limits, false, true);
			if (!ret) {
				int collapsed = msm_pm_power_collapse(false);
				msm_rpmrs_exit_sleep(rs_limits, false, true,
						collapsed);
			}
		} else {
			pr_err("%s: cannot find the lowest power limit\n",
				__func__);
		}

#ifdef CONFIG_MSM_IDLE_STATS
		if (time != 0) {
			int64_t end_time = msm_timer_get_sclk_time(NULL);
			if (end_time != 0) {
				time = end_time - time;
				if (time < 0)
					time += period;
			} else
				time = 0;
		}

		msm_pm_add_stat(MSM_PM_STAT_SUSPEND, time);
#endif /* CONFIG_MSM_IDLE_STATS */
	} else if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE]) {
		if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
			pr_info("%s: standalone power collapse\n", __func__);
		msm_pm_power_collapse_standalone(false);
	} else if (allow[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT]) {
		if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
			pr_info("%s: swfi\n", __func__);

		if (suspend_console_deferred)
			suspend_console();

#ifdef CONFIG_MSM_WATCHDOG
		msm_watchdog_suspend(NULL);
#endif

		printk(KERN_INFO "[R] suspend end\n");
		msm_pm_swfi();
		printk(KERN_INFO "[R] resume start\n");

#ifdef CONFIG_MSM_WATCHDOG
		msm_watchdog_resume(NULL);
#endif

		if (suspend_console_deferred)
			resume_console();

		if (MSM_PM_DEBUG_RPM_TIMESTAMP & msm_pm_debug_mask)
			msm_rpm_print_sleep_tick();
	}


enter_exit:
	if (MSM_PM_DEBUG_SUSPEND & msm_pm_debug_mask)
		pr_info("%s: return\n", __func__);

	return 0;
}

static struct platform_suspend_ops msm_pm_ops = {
	.enter = msm_pm_enter,
	.valid = suspend_valid_only_mem,
};


/******************************************************************************
 * Initialization routine
 *****************************************************************************/
/*
 * For speeding up boot time:
 * During booting up, disable entering arch_idle() by disable_hlt()
 * Enable it after booting up BOOT_LOCK_TIMEOUT sec.
*/

#define BOOT_LOCK_TIMEOUT_NORMAL      (60 * HZ)
#define BOOT_LOCK_TIMEOUT_SHORT      (10 * HZ)
static void do_expire_boot_lock(struct work_struct *work)
{
	enable_hlt();
	pr_info("Release 'boot-time' no_halt_lock\n");
}
static DECLARE_DELAYED_WORK(work_expire_boot_lock, do_expire_boot_lock);

static void __init boot_lock_nohalt(void)
{
	int nohalt_timeout;

	/* normal/factory2/recovery */
	switch (board_mfg_mode()) {
	case 0:  /* normal */
	case 1:  /* factory2 */
	case 2:  /* recovery */
		nohalt_timeout = BOOT_LOCK_TIMEOUT_NORMAL;
		break;
	case 3:  /* charge */
	case 4:  /* power_test */
	case 5:  /* offmode_charge */
	default:
		nohalt_timeout = BOOT_LOCK_TIMEOUT_SHORT;
		break;
	}
	disable_hlt();
	schedule_delayed_work(&work_expire_boot_lock, nohalt_timeout);
	pr_info("Acquire 'boot-time' no_halt_lock %ds\n", nohalt_timeout / HZ);
}

static int __init msm_pm_init(void)
{
	pgd_t *pc_pgd;
	pmd_t *pmd;
	unsigned long pmdval;
	unsigned int cpu;
#ifdef CONFIG_MSM_IDLE_STATS
	struct proc_dir_entry *d_entry;
#endif
	int ret;

	/* Page table for cores to come back up safely. */
	pc_pgd = pgd_alloc(&init_mm);
	if (!pc_pgd)
		return -ENOMEM;

	pmd = pmd_offset(pc_pgd +
			 pgd_index(virt_to_phys(msm_pm_collapse_exit)),
			 virt_to_phys(msm_pm_collapse_exit));
	pmdval = (virt_to_phys(msm_pm_collapse_exit) & PGDIR_MASK) |
		     PMD_TYPE_SECT | PMD_SECT_AP_WRITE;
	pmd[0] = __pmd(pmdval);
	pmd[1] = __pmd(pmdval + (1 << (PGDIR_SHIFT - 1)));

	/* It is remotely possible that the code in msm_pm_collapse_exit()
	 * which turns on the MMU with this mapping is in the
	 * next even-numbered megabyte beyond the
	 * start of msm_pm_collapse_exit().
	 * Map this megabyte in as well.
	 */
	pmd[2] = __pmd(pmdval + (2 << (PGDIR_SHIFT - 1)));
	flush_pmd_entry(pmd);
	msm_pm_pc_pgd = virt_to_phys(pc_pgd);

	ret = request_irq(rpm_cpu0_wakeup_irq,
			msm_pm_rpm_wakeup_interrupt, IRQF_TRIGGER_RISING,
			"pm_drv", msm_pm_rpm_wakeup_interrupt);
	if (ret) {
		pr_err("%s: failed to request irq %u: %d\n",
			__func__, rpm_cpu0_wakeup_irq, ret);
		return ret;
	}

	ret = irq_set_irq_wake(rpm_cpu0_wakeup_irq, 1);
	if (ret) {
		pr_err("%s: failed to set wakeup irq %u: %d\n",
			__func__, rpm_cpu0_wakeup_irq, ret);
		return ret;
	}

#ifdef CONFIG_MSM_IDLE_STATS
	for_each_possible_cpu(cpu) {
		struct msm_pm_time_stats *stats =
			per_cpu(msm_pm_stats, cpu).stats;

		stats[MSM_PM_STAT_REQUESTED_IDLE].name = "idle-request";
		stats[MSM_PM_STAT_REQUESTED_IDLE].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_WFI].name = "idle-wfi";
		stats[MSM_PM_STAT_IDLE_WFI].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE].name =
			"idle-standalone-power-collapse";
		stats[MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE].
			first_bucket_time = CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_POWER_COLLAPSE].name =
			"idle-power-collapse";
		stats[MSM_PM_STAT_IDLE_POWER_COLLAPSE].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_SUSPEND].name = "suspend";
		stats[MSM_PM_STAT_SUSPEND].first_bucket_time =
			CONFIG_MSM_SUSPEND_STATS_FIRST_BUCKET;
	}

	d_entry = create_proc_entry("msm_pm_stats",
			S_IRUGO | S_IWUSR | S_IWGRP, NULL);
	if (d_entry) {
		d_entry->read_proc = msm_pm_read_proc;
		d_entry->write_proc = msm_pm_write_proc;
		d_entry->data = NULL;
	}
#endif  /* CONFIG_MSM_IDLE_STATS */

	msm_pm_mode_sysfs_add();

	suspend_set_ops(&msm_pm_ops);
	boot_lock_nohalt();
	msm_cpuidle_init();

	if (get_kernel_flag() & KERNEL_FLAG_GPIO_DUMP) {
		suspend_console_deferred = 1;
		console_suspend_enabled = 0;
		msm_pm_debug_mask |= MSM_PM_DEBUG_GPIO;
	}

	return 0;
}

late_initcall(msm_pm_init);
