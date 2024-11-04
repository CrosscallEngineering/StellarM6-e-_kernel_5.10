/*
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
#define pr_fmt(fmt) "subsys_report: " fmt

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/compiler.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/his_debug_base.h>
#include <asm/page.h>
#include <linux/kmsg_dump.h>

#include "subsys_dbg_internal.h"

static char *pstore_info_addr;

void subsys_get_q6v5_msg(const char *subsys_name, char *msg)
{
	struct subsys_trap_info *pinfo = NULL;
	pinfo = (struct subsys_trap_info *)pstore_info_addr;

	snprintf(pinfo->subsys_msg, KMSG_LEN, "Subsys Crash:(%s):%s\r\n",
				 subsys_name, msg);
}

EXPORT_SYMBOL(subsys_get_q6v5_msg);

static void subsys_kmsg_dump(struct kmsg_dumper *dumper,
			 enum kmsg_dump_reason reason)
{
	size_t bytes_written;
	char time_str[32] = {0};

	bool search_done = false;
	bool kmsg_done = false, trace_done = false;
	bool trace_start = false;
	char *line = NULL, *kmsg_buf = NULL;	

	struct task_struct *tsk = get_current();
	struct subsys_trap_info *pinfo = NULL;

	pr_info("%s:Enter\n", __func__);

	pinfo = (struct subsys_trap_info *)pstore_info_addr;

	pinfo->saved = KDEBUG_SAVE_MAGIC;

	his_get_current_time(time_str, sizeof(time_str));

	snprintf(pinfo->header, KHEADER_LEN, "Crash Time:%s, Current Task:%s(PID:%d)\r\n",
			 time_str, tsk->comm, task_pid_nr(tsk));

	kmsg_buf = kzalloc(KDUMPER_LEN, GFP_KERNEL);

	kmsg_dump_get_buffer(dumper, false,
		kmsg_buf, KDUMPER_LEN, &bytes_written);

	while (!search_done && (line = strsep(&kmsg_buf, "\n"))!= NULL) {
		/* Search Kernel-panic */
	    if (!kmsg_done && strstr(line, PANIC_HEADER_INDEX) != NULL) {
			snprintf(pinfo->kmsg, KMSG_LEN, "Crash Summary:%s\r\n", line);
			pinfo->saved |= KMSG_MAGIC_MASK;
			kmsg_done = true;
	    }

		/* Search first Call-trace */
		if (!trace_done && strstr(line, TRACE_HEADER_INDEX) != NULL) {
			snprintf(pinfo->trace, TRACE_LEN, "%s\r\n", line);
			trace_start = true;
		} else if (trace_start) {
			if(strstr(line, "+") != NULL) {
				strcat(pinfo->trace, line);				
				strcat(pinfo->trace, "\r\n");
			} else {
				pinfo->saved |= TRACE_MAGIC_MASK;
				trace_start = false;
				trace_done = true;
			}
		}
		search_done = kmsg_done && trace_done;
    }

	pr_err("Saved magic: 0x%x\n", pinfo->saved);

	kfree(kmsg_buf);
}
			 
static struct kmsg_dumper subsys_kmsg_dumper = {
	.dump = subsys_kmsg_dump,
};

static int subsys_err_parse_dt(struct device *dev, u32 *data)
{
	struct device_node *pnode;
	const u32 *addr;
	u64 size;

	pnode = of_parse_phandle(dev->of_node, "linux,contiguous-region", 0);
	if (pnode == NULL) {
		pr_err("mem reserve for subsys-err not present\n");
		return -EINVAL;
	}

	addr = of_get_address(pnode, 0, &size, NULL);
	if (!addr) {
		pr_err("failed to parse the reserve memory address\n");
		of_node_put(pnode);
		return -EINVAL;
	}

	data[0] = (u32) of_read_ulong(addr, 2);
	data[1] = (u32) size;

	return 0;
}

static int subsys_err_report_probe(struct platform_device *pdev)
{
	int ret = 0;
	u32 offsets[2];
	struct subsys_trap_info *pinfo = NULL;

	if (!pdev->dev.of_node) {
		pr_err("can not find of_node of device\n");
		ret = -EINVAL;
		goto out;
	}

	ret = subsys_err_parse_dt(&pdev->dev, offsets);
	if (ret < 0) {
		pr_err("subsys err report parse dts failed %d\n", ret);
		ret = -EINVAL;
		goto out;
	}

	pstore_info_addr = ioremap_wc(offsets[0], offsets[1]);
	if (pstore_info_addr == NULL) {
		pr_err("pstore addr init failed\n");
		ret = -ENOMEM;
		goto out;
	}

	pr_info("The phys_base=0x%x, size=0x%x vaddr=0x%llx\n",
			offsets[0], offsets[1], (u64)pstore_info_addr);

	pinfo = (struct subsys_trap_info *)pstore_info_addr;

	memset(pinfo, 0, sizeof(struct subsys_trap_info));

	ret = kmsg_dump_register(&subsys_kmsg_dumper);
	if (ret)
		pr_err("kmsg dump register failed\n");

out:
	return ret;
}

static int subsys_err_report_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id subsys_err_match_table[] = {
	{
		.compatible = "subsys-err-report"
	},
	{},
};

static struct platform_driver subsys_err_report_driver = {
	.probe = subsys_err_report_probe,
	.remove = subsys_err_report_remove,
	.driver = {
		.name = "subsys-err-report",
		.owner	= THIS_MODULE,
		.of_match_table = subsys_err_match_table,
	},
};

static int __init subsys_err_report_init(void)
{
	int ret;

	ret = platform_driver_register(&subsys_err_report_driver);
	if (ret) {
		pr_err("%s: register driver error\n", __func__);
		return ret;
	}

	return ret;
}

static void __exit subsys_err_report_exit(void)
{
	platform_driver_unregister(&subsys_err_report_driver);
}

module_init(subsys_err_report_init);
module_exit(subsys_err_report_exit);

MODULE_DESCRIPTION("Subsys Error Report Driver");
MODULE_LICENSE("GPL v2");

