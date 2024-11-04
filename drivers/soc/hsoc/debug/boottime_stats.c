
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/his_debug_base.h>

static uint32_t bl_start;
static uint32_t bl_end;
static uint32_t bl_display;
static uint32_t bl_load_kernel;
static uint32_t btimestamp;
static uint32_t counter_freq;
unsigned long long rcinit_time;

void boot_time_set_bl_data(u32 *bootloader_start, u32 *bootloader_end,
		u32 *display_time, u32 *load_kernel,
		void *counter_base, u32 freq)
{
	bl_start   = readl_relaxed(bootloader_start);
	bl_end     = readl_relaxed(bootloader_end);
	bl_display = readl_relaxed(display_time);
	bl_load_kernel = readl_relaxed(load_kernel);
	btimestamp = readl_relaxed(counter_base);
	counter_freq = freq;
}

static ssize_t boot_time_stats_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int size = 0;
	unsigned long long alltime_to_rcinit;

	alltime_to_rcinit =
		(bl_end + bl_load_kernel)*1000/counter_freq + (rcinit_time/1000);

	size += snprintf(buf+size, PAGE_SIZE, "First Bootloader cost   : %ums\n",
			bl_start*1000/counter_freq);
	size += snprintf(buf+size, PAGE_SIZE, "Second Bootloader cost  : %ums\n",
			(bl_end - bl_start)*1000/counter_freq);
	size += snprintf(buf+size, PAGE_SIZE, "LCD Display : %ums\n",
			bl_display*1000/counter_freq);
	size += snprintf(buf+size, PAGE_SIZE, "Load kernel : %ums\n",
			bl_load_kernel*1000/counter_freq);
	size += snprintf(buf+size, PAGE_SIZE, "Timestamp   : %ums\n",
			btimestamp*1000/counter_freq);
	size += snprintf(buf+size, PAGE_SIZE, "Start Init  : %lldms\n",
			rcinit_time/1000);
	size += snprintf(buf+size, PAGE_SIZE, "All_to_Init : %lldms\n",
			alltime_to_rcinit);

	return size;
}
static struct kobj_attribute boottime_stats_attr = __ATTR_RO(boot_time_stats);

static int __init boot_stats_proc_init(void)
{
	pr_err("%s(%d) --**\n", __func__, __LINE__);
	his_register_sysfs_attr(&boottime_stats_attr.attr);

	return 0;
}
module_init(boot_stats_proc_init);
