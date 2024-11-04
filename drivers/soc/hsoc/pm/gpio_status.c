#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/his_debug_base.h>

extern int print_all_gpio_status(struct seq_file *m, void *v);

static int gpio_status_proc_show(struct seq_file *m, void *v)
{
	return print_all_gpio_status(m, v);
}

void gpio_status_sleep_show(void)
{
	print_all_gpio_status(NULL, NULL);
}
EXPORT_SYMBOL(gpio_status_sleep_show);

static int gpio_status_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpio_status_proc_show, NULL);
}

static const struct proc_ops gpio_status_proc_fops = {
	.proc_open     = gpio_status_proc_open,
	.proc_read     = seq_read,
	.proc_lseek    = seq_lseek,
	.proc_release  = single_release,
};

int gpio_status_proc_init(void)
{
	int ret = 0;

	ret = his_create_procfs_file("gpio_status", 0, &gpio_status_proc_fops);

	return ret;
}
