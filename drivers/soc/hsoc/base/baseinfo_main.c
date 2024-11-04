/*
 * Copyright (C) 2008-2014, Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/reboot.h>
#include <asm/setup.h>

#include <linux/productinfo.h>
#include <linux/his_debug_base.h>

#define DEV_MAX_KEY_NUM     8
#define SYS_BOOT_INFO       "kbootinfo"
#define CMDLINE_STR_MAX_LEN	128

struct device_keymap {
	int key_num;

	struct keymap {
		int code;
		const char *name;
	} keymap[DEV_MAX_KEY_NUM];
};

#define BOOTINFO_ATTR(_name) \
static struct kobj_attribute _name##_attr = { \
	.attr   = {	                              \
		.name = __stringify(_name),	          \
		.mode = 0444,                         \
	},                                        \
	.show   = _name##_show,	                  \
	.store  = NULL,	                          \
}

struct device_bootinfo dev_bi;
EXPORT_SYMBOL_GPL(dev_bi);

static struct device_keymap dev_kmap;
static const char *cmdline;

static int parse_matched_cmdline(const char* source, char* dest, char* result)
{
	char *f_str = NULL;
	int i = 0, size = 0;

	f_str = strstr(source, dest);
	if(f_str) {
		f_str += strlen(dest);
		for (i = 0; *(f_str + i) != '\0'; i++) {
			if (*(f_str + i) == ' ') {
				size = i;
				break;
			}

			if(*(f_str + i + 1) == '\0') {
				pr_buf_info("This is the last cmd\n");
				size = i + 1;
				break;
			}
		}

		strncpy(result, f_str, size);
		return 0;
	}

	return -EINVAL;
}

static void parse_boot_mode_cmdline(void)
{
	int i = 0, ret = 0, len = 0;
	char boot_mode[32] = " ";
	char *cmd_str = "oemboot.mode=";
	char *cmd_sec_str = "sprdboot.mode=";
	char *cmd_final[7] = {"normal","charger","factory2","recovery",\
			"cali","silence","tfupdate"};

	len = sizeof(cmd_final)/sizeof(char *);

	ret = parse_matched_cmdline(cmdline, cmd_str, boot_mode);
	if(ret < 0)
		ret = parse_matched_cmdline(cmdline, cmd_sec_str, boot_mode);

	if(!ret) {
		for(i = 0; i < len; i++) {
			if(strstr(boot_mode, cmd_final[i])) {
				dev_bi.bootmode = i;
				break;
			}
		}
	}
}

static void parse_power_reason_cmdline(void)
{
	int ret = 0;
	char reason_buf[32] = " ";
	char *UpReason_str = "powerup_reason=";
	char *OffReason_str = "poweroff_reason=";

	ret = parse_matched_cmdline(cmdline, UpReason_str, reason_buf);
	if(!ret)
		strncpy(dev_bi.power_on_reason, reason_buf, sizeof(reason_buf));
	else
		strcpy(dev_bi.power_on_reason, "UNKNOWN");

	ret = parse_matched_cmdline(cmdline, OffReason_str, reason_buf);
	if(!ret)
		strncpy((char *)dev_bi.power_off_reason, reason_buf, sizeof(reason_buf));
	else
		strcpy(dev_bi.power_off_reason, "UNKNOWN");
}

static void parse_fuse_status_cmdline(void)
{
	char cmd_target[CMDLINE_STR_MAX_LEN];
	char *cmd_str = "fuse_status=";
	char *cmd_final[2] = {"OFF","ON"};
	int i = 0;

	dev_bi.fused = 0;
	for(i=0; i<sizeof(cmd_final)/sizeof(char *); i++) {
		snprintf(cmd_target, CMDLINE_STR_MAX_LEN, "%s%s", cmd_str, cmd_final[i]);
		if(strstr(cmdline, cmd_target)) {
			dev_bi.fused = i;
			break;
		}
	}

	pr_buf_info("%s: fuse_status=%d\n", __func__, dev_bi.fused);
}

static void parse_lcd_id_cmdline(void)
{
	char cmd_target[CMDLINE_STR_MAX_LEN];
	char *cmd_str = "lcd_id=";
	char *cmd_final[4] = {"ID8725","ID8722","ID9881","ID8394"};/*FT8725,FT8722*/
	int i = 0;

	for(i=0; i<sizeof(cmd_final)/sizeof(char *); i++) {
		snprintf(cmd_target, CMDLINE_STR_MAX_LEN, "%s%s", cmd_str, cmd_final[i]);
		if(strstr(cmdline, cmd_target)) {
			dev_bi.lcd_id = i;
			break;
		}
	}

	pr_buf_info("%s: lcd_id=%d\n", __func__, dev_bi.lcd_id);
}

static void pasrse_board_id_cmdline(void)
{
	int ret = 0;
	char *cmd_str = "oemboot.boardid=";
	char board_id[32] = " ";

	ret = parse_matched_cmdline(cmdline, cmd_str, board_id);
	if(!ret) {
		sscanf(board_id, "%d", &dev_bi.board_id);
		productinfo_register(PRODUCTINFO_HW_VERSION_ID, board_id, NULL);
	}
}

static ssize_t board_id_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += snprintf(s, 4, "%d", dev_bi.board_id);

	return s-buf;
}
BOOTINFO_ATTR(board_id);

static struct attribute *bootattr[] = {
	&board_id_attr.attr,
	NULL,
};

static struct attribute_group bootattr_group = {
	.attrs = bootattr,
};

static void of_get_device_keymap(struct device_node *np)
{
	int i = 0;
	int size = 0;

	size = of_property_count_strings(np, "dev,keymap-names");
	if ((size <= 0) || (size > DEV_MAX_KEY_NUM)) {
		pr_buf_err("device keymap size is error\n");
		return;
	}

	pr_buf_info("found Key num: %d\n", size);
	dev_kmap.key_num = size;
	for (i = 0; i < dev_kmap.key_num; i++) {
		of_property_read_string_index(np, "dev,keymap-names",
				i, &dev_kmap.keymap[i].name);

		of_property_read_u32_index(np, "dev,keymap-values",
				i, &dev_kmap.keymap[i].code);
	}
}

static void of_parse_device_info(void)
{
	struct device_node *np = NULL;

	np = of_find_node_by_path("/soc/his_devinfo");
	if (!np) {
		np = of_find_node_by_path("/his_devinfo");
		if (!np) {
			pr_buf_err("Can not find his_devinfo node\n");
			return;
		}
	}

	of_get_device_keymap(np);

	his_of_get_u32_array(np, "dev,prot-gpios", dev_bi.prot_gpios,
			&dev_bi.prot_num, PROTECTED_GPIO_NUM);
}

void input_print_keyevent_log(u32 type, u32 code, int value)
{
	int i = 0;

	if (type != EV_KEY)
		return;

	if (dev_kmap.key_num == 0) {
		pr_buf_info("%d key %s\n", code,
				value ? "pressed" : "released");
	} else {
		for (i = 0; i < dev_kmap.key_num; i++) {
			if (dev_kmap.keymap[i].code == code) {
				pr_buf_info("%s key %s\n", dev_kmap.keymap[i].name,
						value ? "pressed" : "released");
				break;
			}
		}
	}
}

inline void his_set_curr_backlight_state(u32 bl_level)
{
	static int last_level;

	/* print lcd backlight control log when turn on or off */
	if ((last_level == 0) || (bl_level == 0)) {
		pr_buf_info("Backlight: set lcd backlight from %d to %d\n",
				last_level, bl_level);
		last_level = bl_level;

		if (bl_level == 0) {
			dev_bi.backlight_on = 0;
		} else {
			dev_bi.backlight_on = 1;
		}
	}
}
EXPORT_SYMBOL(his_set_curr_backlight_state);

static void print_device_bootinfo(void)
{
	pr_err("===========================\n");
	pr_err("Boot mode:   	   %d\n", dev_bi.bootmode);
	pr_err("Board id:   	   %d\n", dev_bi.board_id);
	pr_err("Power-On reason:   %s\n", dev_bi.power_on_reason);
	pr_err("Power-Off reason:  %s\n", dev_bi.power_off_reason);
	pr_err("===========================\n");
}

static int his_parse_bootinfo_cmdline(void)
{
	struct device_node *cmdline_node;
	int ret;

	cmdline_node = of_find_node_by_path("/chosen");
	ret = of_property_read_string(cmdline_node, "bootargs", &cmdline);

	if (ret) {
		pr_err("Can't not parse bootargs, ret=%d\n", ret);
		return ret;
	}

	pasrse_board_id_cmdline();
	parse_boot_mode_cmdline();
	parse_lcd_id_cmdline();
	parse_fuse_status_cmdline();
	parse_power_reason_cmdline();

	return 0;
}

static int his_key_monitior_connect(struct input_handler *handler,
			 struct input_dev *dev,
			 const struct input_device_id *id)
{
	struct input_handle *key_monitior_handle;
	int error;

	key_monitior_handle = (struct input_handle *)kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!key_monitior_handle)
		return -ENOMEM;

	key_monitior_handle->dev = dev;
	key_monitior_handle->handler = handler;
	key_monitior_handle->name = "his_key_monitior";

	error = input_register_handle(key_monitior_handle);
	if (error) {
		pr_err("Failed to register input key monitior handler, error %d\n",
			error);
		goto err_free;
	}

	error = input_open_device(key_monitior_handle);
	if (error) {
		pr_err("Failed to open input device, error %d\n", error);
		goto err_unregister;
	}

	return 0;

 err_unregister:
	input_unregister_handle(key_monitior_handle);
 err_free:
	kfree(key_monitior_handle);
	return error;
}

static void his_key_monitior_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
}

static void his_key_monitior_event(struct input_handle *handle,
	unsigned int type, unsigned int code, int value)
{
	input_print_keyevent_log(type, code, value);
}

static const struct input_device_id his_key_monitior_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{},
};

static struct input_handler his_key_monitior_handler = {
	.event = his_key_monitior_event,
	.connect	= his_key_monitior_connect,
	.disconnect	= his_key_monitior_disconnect,
	.name = "his_key_monitior",
	.id_table	= his_key_monitior_ids,
};

static int __init baseinfo_init(void)
{
	int ret = -ENOMEM;
	struct kobject *bootinfo_kobj = NULL;

	memset(&dev_bi, 0, sizeof(struct device_bootinfo));

	/* Set default boot mode 255 */
	dev_bi.bootmode = 255;

	of_parse_device_info();
	bootinfo_kobj = kobject_create_and_add(SYS_BOOT_INFO, NULL);
	if (bootinfo_kobj == NULL) {
		pr_err("%s: bootinfo kobject create failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_group(bootinfo_kobj, &bootattr_group);
	if (ret) {
		pr_err("%s: bootattr group create failed\n", __func__);
		goto sys_fail;
	}

	ret = proc_productinfo_init();
	if(ret)
		pr_err("%s: productinfo init failed\n", __func__);

	ret = proc_memtotal_init();
	if (ret)
	    pr_err("%s: memtotal init failed\n", __func__);

	ret = input_register_handler(&his_key_monitior_handler);
	if (ret)
	    pr_err("%s: his key monitior handler init failed\n", __func__);

	ret = his_parse_bootinfo_cmdline();
	if(ret)
	    pr_err("%s: parse cmdline failed\n", __func__);

	print_device_bootinfo();

	return ret;

sys_fail:
	kobject_put(bootinfo_kobj);
	return ret;
}

module_init(baseinfo_init);

MODULE_DESCRIPTION("H-base info");
MODULE_LICENSE("GPL v2");
