/*
 * Copyright (C) 2018 , Inc.
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
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/pm_wakeup.h>
#include <linux/syscalls.h>
#include <linux/suspend.h>
#include <trace/events/power.h>
#include <linux/his_debug_base.h>
#include <linux/of.h>

static int suspend_debug_mask = 1;
#define MSM_PM_DEBUG_GPIO_SUSPEND    BIT(9)

extern void clock_suspend_debug_print(void);
extern void regulator_suspend_debug_print(void);
extern void gpio_status_sleep_show(void);
#if IS_ENABLED(CONFIG_HQPNP_REG_DEBUG)
extern void vreg_status_sleep_show(void);
#endif /*CONFIG_HQPNP_REG_DEBUG*/

static void suspend_debug_trace_fun(void *unused,
					const char *action, int val, bool start)
{
	/* Print Clk\GPIO\LDO status in suspend */
	if ((MSM_PM_DEBUG_GPIO_SUSPEND & suspend_debug_mask) &&
		start && val > 0 && !strcmp("machine_suspend", action)) {
#if IS_ENABLED(CONFIG_HQPNP_REG_DEBUG)
		vreg_status_sleep_show();
#endif /*CONFIG_HQPNP_REG_DEBUG*/
		gpio_status_sleep_show();
		clock_suspend_debug_print();
		regulator_suspend_debug_print();
	}
}

static int suspend_debug_pasrse_cmdline(void)
{
	struct device_node *cmdline_node;
	const char *cmdline;
	char *cmd_str = "suspend_debug.debug_mask=";
	char *cmd_final = "512";
	char cmd_target[128];
	int ret = 0;

	cmdline_node = of_find_node_by_path("/chosen");
	ret = of_property_read_string(cmdline_node, "bootargs", &cmdline);

	if (ret) {
		pr_err("Can't not parse bootargs, ret=%d\n", ret);
		return ret;
	}

	snprintf(cmd_target, 128, "%s%s", cmd_str, cmd_final);
	if(strstr(cmdline, cmd_target)) {
		suspend_debug_mask= 512;
	}

	pr_err("%s: suspend debug mask=%d\n", __func__, suspend_debug_mask);

	return ret;
}

static int __init suspend_debug_init(void)
{
	int ret = 0;

	ret = suspend_debug_pasrse_cmdline();
	if(ret) {
		pr_err("suspend_debug_init: parse cmdline failed\n");
		return ret;
	}

	ret = register_trace_suspend_resume(
				suspend_debug_trace_fun, NULL);
	if(ret) {
		pr_err("suspend_debug_init: register suspend trace failed\n");
		return ret;
	}

	return 0;
}

module_init(suspend_debug_init);
static void __exit suspend_debug_exit(void)
{
	unregister_trace_suspend_resume(
				suspend_debug_trace_fun, NULL);
}
module_exit(suspend_debug_exit);

MODULE_DESCRIPTION("suspend debug");
MODULE_LICENSE("GPL v2");
