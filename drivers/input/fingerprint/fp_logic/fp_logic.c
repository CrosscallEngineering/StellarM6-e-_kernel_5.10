/*
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

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/export.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#define FP_VTG_MIN_UV               3300000
#define FP_VTG_MAX_UV               3300000

struct fp_logic_proc_data {
	struct device *dev;
    struct platform_device *pdev;
	int logic_gpio_number;
	int vdd_gpio;
	int logic_irq_num;
	int b_power_on;
	struct work_struct work_queue_logic;
	bool b_use_regulator;
	bool b_use_gpio;
    struct regulator *fp_vdd;
};

#ifdef CONFIG_FP_LOGIC_PWR_CTRL_HIGH
#define LOGIC_PWR_CTRL 1
#else
#define LOGIC_PWR_CTRL 0
#endif

struct fp_logic_proc_data *g_fp_logic_proc;

static int fp_power_on(struct fp_logic_proc_data *fp_logic_proc, bool on)
{
    int ret = 0;

    pr_err("'%s' enter.", __func__);
    if (!fp_logic_proc) {
        pr_err("fp_logic_proc is invalid");
        return -EINVAL;
    }

    pr_err("set power:%d->%d,use_regulator:%d", fp_logic_proc->b_power_on, on, fp_logic_proc->b_use_regulator);
    if (fp_logic_proc->b_power_on ^ on) {
		if (fp_logic_proc->b_use_regulator && fp_logic_proc->fp_vdd) {
			pr_err("use regulator to control power");
			if (on) 
				ret = regulator_enable(fp_logic_proc->fp_vdd);
			else 
				ret = regulator_disable(fp_logic_proc->fp_vdd);
		} else if (fp_logic_proc->b_use_gpio && gpio_is_valid(fp_logic_proc->vdd_gpio)) {
			pr_err("use gpio(%d) to control power", fp_logic_proc->vdd_gpio);
            ret = gpio_direction_output(fp_logic_proc->vdd_gpio, on);
		} else {
			pr_err("fp_logic: no method to control power");
		}
	}

	if (ret < 0) 
		pr_err("set power to %d failed,ret=%d", on, ret);
	else
		fp_logic_proc->b_power_on = on;
    pr_err("'%s' leave.", __func__);
    return ret;
}

static int fp_get_logic_status(void)
{
    struct fp_logic_proc_data *fp_logic_proc = g_fp_logic_proc;
	int logic_value1 = 0, logic_value2 = 0;

	pr_err("%s(..) enter.\n", __func__);
	logic_value1 = gpio_get_value(fp_logic_proc->logic_gpio_number);
	msleep(300);
	logic_value2 = gpio_get_value(fp_logic_proc->logic_gpio_number);
	if (logic_value1 == logic_value2 && logic_value1 == LOGIC_PWR_CTRL) {
		pr_err("%s logic GPIO keep %d for 300ms\n", __func__, logic_value1);
		return 1;
	} else {
		pr_err("%s logic GPIO start is %d, end is %d\n", __func__, logic_value1, logic_value2);
		return 0;		
	}
}

static void fp_ctl_device_event_logic(struct work_struct *ws)
{
	int logic_value1 = 0, logic_value2 = 0;
	
	struct fp_logic_proc_data *fp_logic_proc = g_fp_logic_proc;

	pr_err("%s(..) enter.\n", __func__);
	logic_value1 = gpio_get_value(fp_logic_proc->logic_gpio_number);
	msleep(300);
	logic_value2 = gpio_get_value(fp_logic_proc->logic_gpio_number);
	if (logic_value1 == logic_value2 && logic_value1 == LOGIC_PWR_CTRL) {
		disable_irq_nosync(fp_logic_proc->logic_irq_num);
		pr_err("%s logic GPIO keep %d for 300ms\n", __func__,logic_value1);
        fp_power_on(fp_logic_proc, 0);
	} else
		pr_err("%s logic GPIO start is %d, end is %d\n", __func__, logic_value1, logic_value2);	
	
}

static irqreturn_t fp_ctl_device_logic_irq(int irq, void *dev_id)
{
	struct fp_logic_proc_data *fp_logic_proc = g_fp_logic_proc;

	disable_irq_nosync(irq);
//	pr_err("%s(irq = %d, ..) toggled.\n", __func__, irq);
	schedule_work(&fp_logic_proc->work_queue_logic);
//	fp_set_logic_irq_type(IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
	enable_irq(irq);
	return IRQ_HANDLED;
}

static int fp_register_logic_irq(struct fp_logic_proc_data *fp_logic_proc)
{
    int ret = 0;
    unsigned long irqflags = 0;

    pr_err("'%s' enter.", __func__);
    /*get irq num*/
    fp_logic_proc->logic_irq_num = gpio_to_irq(fp_logic_proc->logic_gpio_number);
    pr_err("logic_gpio_number:%d,logic_irq_num:%d", fp_logic_proc->logic_gpio_number, fp_logic_proc->logic_irq_num);

    if (fp_logic_proc->logic_irq_num > 0) {
        pr_err("logic_irq_num:%d", fp_logic_proc->logic_irq_num);
        irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
        ret = request_irq(fp_logic_proc->logic_irq_num, fp_ctl_device_logic_irq, irqflags, "fp_logic_irq", fp_logic_proc);
        //fp_logic_proc->b_irq_enabled = true;
        enable_irq_wake(fp_logic_proc->logic_irq_num);
    } else {
        pr_err("logic_irq_num:%d is invalid", fp_logic_proc->logic_irq_num);
        ret = fp_logic_proc->logic_irq_num;
    }

    pr_err("'%s' leave.", __func__);
    return ret;
}



static int fp_init_power(struct fp_logic_proc_data *fp_logic_proc)
{
    int ret = 0;

    pr_err("'%s' enter.", __func__);
    
	if (!fp_logic_proc) {
        pr_err("fp_logic_proc is invalid");
        return -EINVAL;
    }

    if (fp_logic_proc->b_use_regulator) {
        /*get regulator*/
        fp_logic_proc->fp_vdd = regulator_get(&fp_logic_proc->pdev->dev, "fp_vdd");
        if (IS_ERR_OR_NULL(fp_logic_proc->fp_vdd)) {
            ret = PTR_ERR(fp_logic_proc->fp_vdd);
            pr_err("get fp vdd regulator failed,ret=%d", ret);
            return ret;
        }

        if (regulator_count_voltages(fp_logic_proc->fp_vdd) > 0) {
            ret = regulator_set_voltage(fp_logic_proc->fp_vdd, FP_VTG_MIN_UV, FP_VTG_MAX_UV);
            if (ret) {
                pr_err("vdd regulator set_voltage failed ret=%d", ret);
                regulator_put(fp_logic_proc->fp_vdd);
                return ret;
            }
        }
        pr_err("get power regulator success");
    } else if (fp_logic_proc->b_use_gpio) {
	    if (gpio_is_valid(fp_logic_proc->vdd_gpio)) {
            ret = gpio_request(fp_logic_proc->vdd_gpio, "fp_vdd_gpio");
            if (ret < 0) {
                pr_err("vdd gpio request failed");
                return ret;
            }

            ret = gpio_direction_output(fp_logic_proc->vdd_gpio, 0);
            if (ret < 0) {
                pr_err("set_direction for vdd gpio failed");
                gpio_free(fp_logic_proc->vdd_gpio);
                return ret;
            }
			pr_err("get power gpio(%d) success", fp_logic_proc->vdd_gpio);
		}
    } else {
		pr_err("fp_logic:no need to user logic to control power");
	}

    pr_err("'%s' leave.", __func__);
    return 0;
}


static int fp_logic_proc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;
	int rc = 0;
	struct device_node *np = dev->of_node;
	struct fp_logic_proc_data *fp_logic_proc = devm_kzalloc(dev, sizeof(*fp_logic_proc), GFP_KERNEL);
	dev_info(dev, "%s: enter\n", __func__);
	if (!fp_logic_proc) {
		pr_err("failed to allocate memory for struct fp_logic_proc_data\n");
		rc = -ENOMEM;
		goto exit;
	}

    fp_logic_proc->pdev = pdev;
	fp_logic_proc->dev = dev;
	platform_set_drvdata(pdev, fp_logic_proc);

	if (!np) {
		pr_err("%s: no of node found\n",__func__);
		rc = -EINVAL;
		goto exit;
	}
	
    fp_logic_proc->b_use_regulator = of_property_read_bool(np, "fp,use-regulator");

	pr_err("%s: user_regulator:%d\n",__func__,fp_logic_proc->b_use_regulator);

    fp_logic_proc->b_use_gpio = of_property_read_bool(np, "fp,use-gpio");

	pr_err("%s: user_gpio:%d\n",__func__,fp_logic_proc->b_use_gpio);
	
	fp_logic_proc->logic_gpio_number = of_get_named_gpio(np, "fp,gpio_logic", 0);
    if (!gpio_is_valid(fp_logic_proc->logic_gpio_number))
        pr_err("unable to get fp logic irq gpio");

	fp_logic_proc->vdd_gpio = of_get_named_gpio(np, "fp,vdd-gpio", 0);
    if (!gpio_is_valid(fp_logic_proc->vdd_gpio))
        pr_err("unable to get fp vdd gpio");

		/* request logic irq gpio */
    if (gpio_is_valid(fp_logic_proc->logic_gpio_number)) {
        pr_err("logic init irq gpio:%d", fp_logic_proc->logic_gpio_number);
        ret = gpio_request(fp_logic_proc->logic_gpio_number, "fp_logic_irq_gpio");
        if (ret < 0) {
            pr_err("logic irq gpio request failed");
            return ret;
        }

        ret = gpio_direction_input(fp_logic_proc->logic_gpio_number);
        if (ret < 0) {
            pr_err("set_direction for logic irq gpio failed");
            gpio_free(fp_logic_proc->logic_gpio_number);
            return ret;
        }
    }
	
	fp_logic_proc->b_power_on = false;
	INIT_WORK(&fp_logic_proc->work_queue_logic, fp_ctl_device_event_logic);
    g_fp_logic_proc = fp_logic_proc;

	ret = fp_init_power(fp_logic_proc);
	if (ret < 0) {
        pr_err("init power fails,ret=%d", ret);
        gpio_free(fp_logic_proc->logic_gpio_number);
		return ret;
    }

	fp_power_on(fp_logic_proc, 1);	

	ret = fp_get_logic_status();
	if (ret == 1) {
		fp_power_on(fp_logic_proc, 0);	
		pr_err("%s: should be power off\n", __func__);
	}
	ret = fp_register_logic_irq(fp_logic_proc);
    if (ret < 0) {
        pr_err("register fp logic irq fails,ret=%d", ret);
        gpio_free(fp_logic_proc->logic_gpio_number);
        return ret;
    }


	pr_err("%s: ok\n", __func__);

	return 0;

exit:
	return rc;
}

static int fp_logic_proc_remove(struct platform_device *pdev)
{

	struct fp_logic_proc_data *fp_logic_proc = platform_get_drvdata(pdev);
	if (fp_logic_proc) {
		if (fp_logic_proc->logic_irq_num > 0) {
			disable_irq_wake(fp_logic_proc->logic_irq_num);
			free_irq(fp_logic_proc->logic_irq_num, fp_logic_proc);
			fp_logic_proc->logic_irq_num = -1;
		}
		if (fp_logic_proc->b_use_regulator && fp_logic_proc->fp_vdd)
            regulator_put(fp_logic_proc->fp_vdd);
        if (fp_logic_proc->b_use_gpio && gpio_is_valid(fp_logic_proc->vdd_gpio))
			gpio_free(fp_logic_proc->vdd_gpio);
		if (gpio_is_valid(fp_logic_proc->logic_gpio_number))
			gpio_free(fp_logic_proc->logic_gpio_number);
		cancel_work_sync(&fp_logic_proc->work_queue_logic);
    
		platform_set_drvdata(pdev, NULL);
		kfree(fp_logic_proc);
		fp_logic_proc = NULL;
	}

	pr_err("%s: ok\n", __func__);
	return 0;
}

static struct of_device_id fp_logic_proc_of_match[] = {
	{ .compatible = "fingerprint,fp_logic", },
	{}
};

static struct platform_driver fp_logic_proc_driver = {
	.driver = {
		.name	= "fp_logic",
		.owner	= THIS_MODULE,
		.of_match_table = fp_logic_proc_of_match,
	},
	.probe	= fp_logic_proc_probe,
	.remove	= fp_logic_proc_remove,
};

static int __init fp_logic_proc_init(void)
{

	int rc;
	printk("%s(): enter\n", __func__);

	rc = platform_driver_register(&fp_logic_proc_driver);
	
	if (!rc)
		pr_info("%s OK\n", __func__);
	else
		pr_err("%s %d\n", __func__, rc);
	
	return rc;
}

static void __exit fp_logic_proc_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&fp_logic_proc_driver);
}

module_init(fp_logic_proc_init);
module_exit(fp_logic_proc_exit);

MODULE_AUTHOR("Hsoc Department");
MODULE_DESCRIPTION("Fingerprint logic configuration.");
MODULE_LICENSE("GPL v2");


