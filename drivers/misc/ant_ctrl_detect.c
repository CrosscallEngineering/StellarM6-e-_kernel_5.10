
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm_wakeup.h>

#define DETECT_DELAY_TIMEOUT (HZ / 5); /* 0.2s */
#define MAX_ANT_CTRL_GPIO    16

struct detect_timer {
	struct timer_list timer;
	int idx;
};

struct AntData{
	int ctrl_pin[MAX_ANT_CTRL_GPIO];
	int ctrl_irq[MAX_ANT_CTRL_GPIO];
	int ctrl_sts[MAX_ANT_CTRL_GPIO];
	int gpio_count;
	struct detect_timer det_timers[MAX_ANT_CTRL_GPIO];
	struct device *dev;
};

static struct AntData *G_AntDev;

static void ant_ctrl_det_timer(struct timer_list *timer)
{
	struct detect_timer *det_timer = container_of(timer,
				struct detect_timer, timer);
	struct AntData *ant_dev = G_AntDev;

	int idx = det_timer->idx;
	int cur_sts = 0;
	char *ant_change[2] = {"ANT_STATE=CHANGED", NULL};

	if (!ant_dev || idx >= ant_dev->gpio_count) {
		pr_err("%s, Null dev or idx invalid\n", __func__);
		return;
	}

	cur_sts = !!gpio_get_value(ant_dev->ctrl_pin[idx]);

	if(cur_sts != ant_dev->ctrl_sts[idx])
		kobject_uevent_env(&ant_dev->dev->kobj, KOBJ_CHANGE, ant_change);

	ant_dev->ctrl_sts[idx] = cur_sts;

	pr_err("%s, gpio(%d) state:%d\n", __func__,
				ant_dev->ctrl_pin[idx], ant_dev->ctrl_sts[idx]);
}

static irqreturn_t ant_ctrl_detect_irq(int irq, void *data)
{
	struct timer_list *cur_timer = data;
	struct AntData *ant_dev = G_AntDev;

	if (!ant_dev)
		return IRQ_HANDLED;

	del_timer(cur_timer);
	cur_timer->expires = jiffies + DETECT_DELAY_TIMEOUT;
	add_timer(cur_timer);

	pm_wakeup_dev_event(ant_dev->dev, 500, true); /* 500ms */

	return IRQ_HANDLED;
}

static int ant_ctrl_proc_show(struct seq_file *m, void *v)
{
	int i = 0;
	struct AntData *ant_dev = G_AntDev;

	if (!ant_dev)
		return -EINVAL;

	seq_printf(m, "ANT cable insert state:\n");

	for (i = 0; i < ant_dev->gpio_count; i++)
		seq_printf(m, "GPIO[%d] state %d\n", ant_dev->ctrl_pin[i],
			gpio_get_value(ant_dev->ctrl_pin[i]));

	return 0;
}
static int ant_ctrl_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ant_ctrl_proc_show, NULL);
}

static const struct proc_ops ant_ctrl_proc_fops = {
	.proc_open     = ant_ctrl_proc_open,
	.proc_read     = seq_read,
	.proc_lseek    = seq_lseek,
	.proc_release  = single_release,
};

static int ant_ctrl_detect_probe(struct platform_device *pdev)
{
	struct AntData *ant_dev;
	struct device *dev = &pdev->dev;
	struct device_node *of_node = NULL;

	int ret = 0, i = 0, irq = 0;

	ant_dev = devm_kzalloc(&pdev->dev, sizeof(*ant_dev), GFP_KERNEL);
	if (!ant_dev)
		return -ENOMEM;

	if (!pdev->dev.of_node) {
		pr_err("the of node is NULL\n");
		return -EINVAL;
	}

	of_node = pdev->dev.of_node;

	ret = of_gpio_count(of_node);
	if (ret <= 0 || ret > MAX_ANT_CTRL_GPIO) {
		pr_err("Ant gpio count invalid\n");
		return -EINVAL;
	}
	ant_dev->gpio_count = ret;
	ant_dev->dev = dev;

	for (i = 0; i < ant_dev->gpio_count; i++) {
		ant_dev->ctrl_pin[i] = of_get_gpio(of_node, i);
		irq = gpio_to_irq(ant_dev->ctrl_pin[i]);
		pr_err("Ant gpio array[%d]:%d\n", i, ant_dev->ctrl_pin[i]);

		/* add timer for delay to detect state */
		ant_dev->det_timers[i].idx = i;
		timer_setup(&ant_dev->det_timers[i].timer, ant_ctrl_det_timer, 0);

		ret = devm_request_threaded_irq(dev, irq, NULL, ant_ctrl_detect_irq,
				IRQF_ONESHOT|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
				"ANT_CTRL", &ant_dev->det_timers[i].timer);
		if (ret)
			pr_err("Ant irq(gpio:%d) request failed, ret:%d\n", ant_dev->ctrl_pin[i], ret);

		ant_dev->ctrl_irq[i] = irq;
		ant_dev->ctrl_sts[i] = gpio_get_value(ant_dev->ctrl_pin[i]);
		enable_irq_wake(ant_dev->ctrl_irq[i]);
	}

	device_init_wakeup(ant_dev->dev, true);

	G_AntDev = ant_dev;
	proc_create("ant_state", 0, NULL, &ant_ctrl_proc_fops);
 
	return 0;
}

static int ant_ctrl_detect_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ant_ctrl_detect_match[] = {
	{ .compatible = "ant_ctrl_detect" },
	{ }
};
MODULE_DEVICE_TABLE(of, ant_ctrl_detect_match);

static struct platform_driver ant_ctrl_detect_driver = {
	.probe = ant_ctrl_detect_probe,
	.remove = ant_ctrl_detect_remove,

	.driver = {
		.name = "ant_ctrl_detect",
		.of_match_table = ant_ctrl_detect_match,
	},
};

module_platform_driver(ant_ctrl_detect_driver);

MODULE_DESCRIPTION("SAR ANT CABLE Detect Driver");
MODULE_LICENSE("GPL v2");
