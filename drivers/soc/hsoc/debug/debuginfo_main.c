/*
 * Copyright (C) 2005-2016, Inc.
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

#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/his_debug_base.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/module.h>

static uint32_t boot_debug_flag;

typedef struct {
	int bit_set;
	char *cmdline;
} bootflag_type;

static bootflag_type debugflag_sets[] = {
	{DEBUG_ENABLE_BIT,   "enable_debug"},
	{PRINT_WAKELOCK_BIT, "print_active_ws"},
	{SERIAL_ENABLE_BIT,  "serial_enable"},
	{AUDIO_DEBUG_BIT,    "audio_debug"},
	{FS_DEBUG_BIT, 		 "debug_fs_switch"},
};

static int his_parse_debugflag_cmdline(void)
{
	static const char *cmdline;
	struct device_node *cmdline_node;
	int ret = 0, i = 0, cnt = 0;

	cnt = sizeof(debugflag_sets)/sizeof(bootflag_type);

	cmdline_node = of_find_node_by_path("/chosen");
	ret = of_property_read_string(cmdline_node, "bootargs", &cmdline);

	if (ret)
		pr_err("Can't not parse bootargs\n");

	for(i = 0; i < cnt; i++) {
		if(strstr(cmdline, debugflag_sets[i].cmdline)) {
			boot_debug_flag |= debugflag_sets[i].bit_set; 
			pr_buf_info("Set %s\n", debugflag_sets[i].cmdline);
		}
	}

	return 0;
}

void set_debug_flag_bit(int set_bit)
{
	boot_debug_flag |= set_bit;
}

void clear_debug_flag_bit(int set_bit)
{
	boot_debug_flag &= ~set_bit;
}

bool get_debug_flag_bit(int get_bit)
{
	if (boot_debug_flag & get_bit)
		return true;
	else
		return false;
}
EXPORT_SYMBOL_GPL(get_debug_flag_bit);

static int __init debuginfo_init(void)
{
	his_parse_debugflag_cmdline();

#ifndef CONFIG_RELEASE_VERSION
	set_debug_flag_bit(DEBUG_ENABLE_BIT);
#endif /* CONFIG_RELEASE_VERSION */

	pr_buf_info("Debug flag %s\n",
			get_debug_flag_bit(DEBUG_ENABLE_BIT)? "enable":"disable");

	return 0;
}

module_init(debuginfo_init);

MODULE_DESCRIPTION("H-debug info");
MODULE_LICENSE("GPL v2");
