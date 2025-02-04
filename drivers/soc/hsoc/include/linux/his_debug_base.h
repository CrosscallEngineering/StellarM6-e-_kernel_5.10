#ifndef __HIS_DEBUG_CONTROL_NODE_H__
#define __HIS_DEBUG_CONTROL_NODE_H__

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>

#define PROTECTED_GPIO_NUM   12
#define POWER_REASON_LEN     32

enum {
	BOOT_NORMAL_MODE,
	BOOT_CHARGER_MODE,
	BOOT_FACTORY_MODE,
	BOOT_RECOVERY_MODE,
	BOOT_CALI_MODE,
	BOOT_SILENCE_MODE,
	BOOT_TFUPDATE
};

struct device_bootinfo {
	u8 bootmode;
	u8 board_id;
	u8 backlight_on;
	u8 fused;
	u32 lcd_id;
	u32 sector_size;
	u64 sectors_num;
	u64 ddr_size;
	/* tz protected gpios num and array */
	u32 prot_num;
	u32 prot_gpios[PROTECTED_GPIO_NUM];
	char power_on_reason[POWER_REASON_LEN];
	char power_off_reason[POWER_REASON_LEN];
};
extern struct device_bootinfo dev_bi;

#define PRINT_OUT(m, x...) \
	do { \
		if (m) \
			seq_printf(m, x); \
		else \
			pr_err(x); \
	} while (0)

#if IS_ENABLED(CONFIG_PRODUCT_BASEINFO)
/* sysfs node interface */
extern int his_register_sysfs_attr(struct attribute *attr);
extern int his_register_sysfs_attr_group(struct attribute_group *attr_group);
extern struct kobject *his_register_sysfs_dir(const char *name);

/* debugfs node interface */
extern int his_register_debugfs_file(const char *name, umode_t mode, void *data,
		const struct file_operations *fops);
extern struct dentry *his_register_debugfs_dir(const char *name);

/* procfs node interface */
extern int his_create_procfs_file(const char *name, umode_t mode,
		const struct proc_ops *fops);
extern struct proc_dir_entry *his_create_procfs_dir(const char *name);

/* get current time */
extern void his_get_current_time(char *ptime, int size);
#endif /* CONFIG_PRODUCT_BASEINFO */

#ifdef CONFIG_BSP_DEBUG_CTRL
/* debug flag defined */
enum {
	DEBUG_ENABLE_BIT       = 1U << 1,
	PRINT_WAKELOCK_BIT     = 1U << 2,
	SERIAL_ENABLE_BIT      = 1U << 3,
	AUDIO_DEBUG_BIT        = 1U << 4,
	FS_DEBUG_BIT           = 1U << 5,
};

void set_debug_flag_bit(int set_bit);
void clear_debug_flag_bit(int set_bit);
bool get_debug_flag_bit(int get_bit);

/* memtotal and productinfo node init */
extern int proc_memtotal_init(void);
extern int proc_productinfo_init(void);

/* reboot war node init */
extern int sys_reboot_war_init(void);
extern void do_sys_war_action(void);

/* device tree get u32 arrry */
extern int his_of_get_u32_array(struct device_node *np, const char *prop_name,
		u32 *out, int *len, u32 max_size);

/* key and backlight log */
extern void input_print_keyevent_log(u32 type, u32 code, int value);
extern void his_set_curr_backlight_state(u32 bl_level);

/* bootloader time debug */
void boot_time_set_bl_data(u32 *bootloader_start, u32 *bootloader_end,
		u32 *display_time, u32 *load_kernel,
		void *counter_base, u32 freq);

/* memory info */
extern unsigned long get_hs_total_ram(void);

/* create debug flag sysfs attr */
extern void debug_flag_control_init(void);

#endif	/* CONFIG_BSP_DEBUG_CTRL */

#endif	/* __HIS_DEBUG_CONTROL_NODE_H__ */
