/*
 * Copyright (C) 2018.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/pm_info.h>
#include <trace/events/power.h>

static void pm_suspend_marker(char *annotation)
{
	struct timespec64 ts;
	struct rtc_time tm;

	ktime_get_real_ts64(&ts);
	rtc_time64_to_tm(ts.tv_sec, &tm);

	pr_info("Suspend %s %d-%02d-%02d %02d:%02d:%02d.%09u UTC\n",
		annotation, tm.tm_year + 1900, tm.tm_mon +1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
}

static void suspend_core_trace_fun(void *unused,
					const char *action, int val, bool start)
{
	/* Print suspend enter and exit marker log */
	if (start && val > 0 && !strcmp("suspend_enter", action)) {
		pm_suspend_marker("entry");
#ifdef CONFIG_HSUSPEND_SYS_SYNC
		his_suspend_sys_sync_queue();
#endif /*CONFIG_HSUSPEND_SYS_SYNC*/
	}

	if (!start && val == 0 && !strcmp("thaw_processes", action))
		pm_suspend_marker("exit");
}

static int __init pminfo_init(void)
{
	int ret = 0;

	ret = gpio_status_proc_init();
	if(ret)
		pr_err("%s: gpio status procfs creat failed\n", __func__);

	ret = register_trace_suspend_resume(
				suspend_core_trace_fun, NULL);
	if(ret) {
		pr_err("%s: register suspend trace failed\n", __func__);
		return ret;
	}

#ifdef CONFIG_HSUSPEND_SYS_SYNC
	ret = sys_sync_queue_init();
	if(ret) {
		pr_err("%s: suspend sync init failed\n", __func__);
		return ret;
	}
#endif /*CONFIG_HSUSPEND_SYS_SYNC*/

	return ret;
}

module_init(pminfo_init);
MODULE_DESCRIPTION("H-pm info");
MODULE_LICENSE("GPL v2");
