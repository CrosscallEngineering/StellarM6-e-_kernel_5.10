/*
 * Copyright (C) 2016-2018, Inc.
 *
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
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/his_debug_base.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/input.h>

struct sim_switch_driver_data {
	struct device *dev;
	struct input_dev *input;

	int sim_switch_pin;
	int sim_num;
	int sim_switch_val;
	int sim_tray_pin;
	int irq;
};

static struct sim_switch_driver_data *sim_switch_pdata;

static ssize_t hw_sim_tray_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(sim_switch_pdata->sim_tray_pin));
}

static struct kobj_attribute hw_sim_tray_attr = __ATTR(hw_sim_tray, 0644, hw_sim_tray_show, NULL);

static struct attribute *hw_sim_tray_attrs[] = {
	&hw_sim_tray_attr.attr,
	NULL
};

static const struct attribute_group hw_sim_tray_group = {
	.attrs = hw_sim_tray_attrs,
};

static ssize_t sim_switch_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sim_switch_pdata->sim_switch_val);
}

static ssize_t sim_switch_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	int ret = 0;
	long value;

	ret = kstrtol(buf, 10, &value);
	if (ret) {
		pr_err("%s: sscanf is wrong!\n", __func__);
		return -EINVAL;
	}

	pr_info("%s: sim switch %ld\n", __func__, value);

	if (!!value) {
		if (gpio_is_valid(sim_switch_pdata->sim_switch_pin)) {
			gpio_set_value(sim_switch_pdata->sim_switch_pin, 1);
			sim_switch_pdata->sim_switch_val = 1;
		}
	} else {
		if (gpio_is_valid(sim_switch_pdata->sim_switch_pin)) {
			gpio_set_value(sim_switch_pdata->sim_switch_pin, 0);
			sim_switch_pdata->sim_switch_val = 0;
		}
	}

	return len;
}

static struct kobj_attribute sim_switch_attr = __ATTR_RW(sim_switch);

static struct attribute *sim_switch_attrs[] = {
	&sim_switch_attr.attr,
	NULL
};

static const struct attribute_group sim_switch_group = {
	.attrs = sim_switch_attrs,
};

#ifdef CONFIG_SIM_PLUG_REPORT
#define KEY_SIM_IN  750
#define KEY_SIM_OUT 751

void sim_plugout_report(void)
{
	if (!sim_switch_pdata || !sim_switch_pdata->input) {
		pr_err("%s: dev is NULL\n", __func__);
		return;
	}

	pr_info( "%s >>\n", __func__);
	/*here the event must be first 1 then 0 */
	/*because when deep suspend the input core will clear all the event*/
	/*then the plug out code 0 will be ingore*/
	/*or we should add the pm suspend resume function to report the plug event again when resume just like the gpio_keys dose*/
	input_report_key(sim_switch_pdata->input, KEY_SIM_OUT, 1);
	input_sync(sim_switch_pdata->input);
	input_report_key(sim_switch_pdata->input, KEY_SIM_OUT, 0);
	input_sync(sim_switch_pdata->input);
}

void sim_plugin_report(void)
{
	if (!sim_switch_pdata || !sim_switch_pdata->input) {
		pr_err("%s: dev is NULL\n", __func__);
		return;
	}

	pr_info( "%s >>\n", __func__);
	/*here the event must be first 1 then 0 */
	/*because when deep suspend the input core will clear all the event*/
	/*then the plug out code 0 will be ingore*/
	/*or we should add the pm suspend resume function to report the plug event again when resume just like the gpio_keys dose*/
	input_report_key(sim_switch_pdata->input, KEY_SIM_IN, 1);
	input_sync(sim_switch_pdata->input);
	input_report_key(sim_switch_pdata->input, KEY_SIM_IN, 0);
	input_sync(sim_switch_pdata->input);
}

static int sim_input_register(struct sim_switch_driver_data *pdata)
{
	int ret = 0;

	/*input device subsystem */
	pdata->input = devm_input_allocate_device(pdata->dev);
	if (pdata->input == NULL) {
		printk("%s, failed to allocate input device\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	input_set_capability(pdata->input, EV_KEY, KEY_SIM_IN);
	input_set_capability(pdata->input, EV_KEY, KEY_SIM_OUT);

	pdata->input->name = "Esim Input Dev";
	ret = input_register_device(pdata->input);
	if (ret) {
		pr_err("%s, failed to register input device\n", __func__);
		input_free_device(pdata->input);
		pdata->input = NULL;
	}

	return ret;
}

static irqreturn_t sim_plug_det_irq(int irq, void *data)
{
	int value = 0;
	static int prev_value = -1;

	value = !!gpio_get_value(sim_switch_pdata->sim_tray_pin);

	if(prev_value!=value && 1==value)
		sim_plugin_report();
	else if(prev_value!=value && 0==value)
		sim_plugout_report();

	prev_value = value;

	return IRQ_HANDLED;
}
#endif /*CONFIG_SIM_PLUG_REPORT*/

static int sim_switch_probe(struct platform_device *pdev)
{
    struct gpio_desc *gpiod;
	struct sim_switch_driver_data *pdata;
	struct device_node *np = pdev->dev.of_node;
	struct kobject *root_kobjs;
	enum of_gpio_flags gpio_flags;
	int err = 0, tray_pin_val = 0;

	pr_info("%s: enter\n", __func__);

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->dev = &pdev->dev;

	/* confirgure sim switch pin */
	pdata->sim_switch_pin = of_get_named_gpio_flags(np, "sim-switch-pin", 0, &gpio_flags);
	if (gpio_is_valid(pdata->sim_switch_pin)) {
		err = gpio_request_one(pdata->sim_switch_pin, GPIOF_DIR_OUT, "sim_switch");
		if (err) {
			pr_err("%s: sim_switch_pin gpio request failed %d\n", __func__, err);
			goto error_switch_pin;
		}

		pr_info("%s sim default val =%d \n", __func__, pdata->sim_switch_val);

		if (pdata->sim_switch_val == 1)
		     gpio_set_value(pdata->sim_switch_pin, 1);
		else 
		     gpio_set_value(pdata->sim_switch_pin, 0); 
	} else {
		pr_err("%s, sim switch pin is invalid\n", __func__);
		err = -EINVAL;
		goto error_switch_pin;
	}

	/* confirgure sim tray pin */
	pdata->sim_tray_pin = of_get_named_gpio_flags(np, "sim-tray-pin", 0, &gpio_flags);
	if (gpio_is_valid(pdata->sim_tray_pin)) {
		err = gpio_request_one(pdata->sim_tray_pin, GPIOF_DIR_IN, "sim_tray");
		if (err) {
			pr_err("%s: sim_tray_pin gpio request failed %d\n", __func__, err);
			goto error_tray_pin;
		} else {
			gpiod = gpio_to_desc(pdata->sim_tray_pin);
			gpiod_set_debounce(gpiod, 50 * 1000); //set 50ms debounce time

			tray_pin_val = gpio_get_value(pdata->sim_tray_pin);
			pr_info("%s sim_tray_pin %d default val =%d \n", __func__, pdata->sim_tray_pin, tray_pin_val);
		}
#ifdef CONFIG_SIM_PLUG_REPORT
		pdata->irq = gpio_to_irq(pdata->sim_tray_pin);

		err = request_threaded_irq(pdata->irq, NULL,
						sim_plug_det_irq,
						IRQF_TRIGGER_FALLING |
						IRQF_TRIGGER_RISING |
						IRQF_ONESHOT,
						"sim_plug_det", &pdev->dev);
		if(err) {
			pr_err("%s, request irq failed, err:%d\n", __func__, err);
			goto error_irq_req;
		}

		disable_irq(pdata->irq);

		err = sim_input_register(pdata);
		if(err) {
			pr_err("%s, input dev register failed, err:%d\n", __func__, err);
			goto error_input_register;
		}
#endif /*CONFIG_SIM_PLUG_REPORT*/
	} else {
		pr_err("%s, sim tary pin is invalid\n", __func__);
		err = -EINVAL;
		goto error_tray_pin;
	}

	/* create sim switch ctrl node */
	root_kobjs = kobject_create_and_add("sim_switch", NULL);

	err = sysfs_create_group(root_kobjs, &sim_switch_group);
	if (err < 0) {
		pr_err("Error create sim_switch sysfs node %d\n", err);
		goto error_sysfs_switch;
	}

	err = sysfs_create_group(root_kobjs, &hw_sim_tray_group);
	if (err < 0) {
		pr_err("Error create hw_sim_tray sysfs node %d\n", err);
		goto error_sysfs_tray;
	}

	sim_switch_pdata = pdata;

#ifdef CONFIG_SIM_PLUG_REPORT
	enable_irq(pdata->irq);
	enable_irq_wake(pdata->irq);

	if (tray_pin_val == 1)
		sim_plugin_report();
	else
		sim_plugout_report();
#endif /*CONFIG_SIM_PLUG_REPORT*/

	pr_info("%s, success\n", __func__);

	return 0;

error_sysfs_tray:
	sysfs_remove_group(root_kobjs, &sim_switch_group);
error_sysfs_switch:
#ifdef CONFIG_SIM_PLUG_REPORT
	input_unregister_device(pdata->input);
error_input_register:
	free_irq(pdata->irq, NULL);
error_irq_req:
#endif /*CONFIG_SIM_PLUG_REPORT*/
	gpio_free(pdata->sim_tray_pin);
error_tray_pin:
	gpio_free(pdata->sim_switch_pin);
error_switch_pin:
	devm_kfree(&pdev->dev, pdata);
	return err;
}

static int sim_switch_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sim_switch_of_match[] = {
	{ .compatible = "sim-switch", },
	{},
};
MODULE_DEVICE_TABLE(of, sim_switch_of_match);
#endif

static struct platform_driver sim_switch_driver = {
	.probe = sim_switch_probe,
	.remove = sim_switch_remove,
	.driver = {
		.name = "sim_switch_driver",
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(sim_switch_of_match),
#endif
	}
};

static int __init sim_switch_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&sim_switch_driver);

	return ret;
}

static void __exit sim_switch_exit(void)
{
	platform_driver_unregister(&sim_switch_driver);

	return;
}


module_init(sim_switch_init);
module_exit(sim_switch_exit);
MODULE_DESCRIPTION("Esim Switch Ctrl.");
MODULE_LICENSE("GPL v2");

