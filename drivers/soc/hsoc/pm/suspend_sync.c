/*
 * Copyright (C) 2018.
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
#include <linux/his_debug_base.h>
#include <linux/vmstat.h>
#include <linux/pm_wakeup.h>


#define DEEP_SYNC_WAKE_THRESHOLD  4096

static DEFINE_SPINLOCK(suspend_sys_sync_lock);

static void suspend_sys_sync(struct work_struct *work);

static struct workqueue_struct *suspend_sys_sync_workqueue;
static struct wakeup_source *sync_wake;

static bool sys_sync_done = true;

static void suspend_sys_sync(struct work_struct *work)
{
	unsigned long dirty;

	dirty = global_node_page_state(NR_FILE_DIRTY);
	pr_info("PM: Syncing filesystems,syncing dirty(%lu kB) ... ", dirty);

	if(dirty > DEEP_SYNC_WAKE_THRESHOLD) {
		__pm_stay_awake(sync_wake);

		ksys_sync_helper();

		__pm_relax(sync_wake);
	} else {
		ksys_sync_helper();
	}

	spin_lock(&suspend_sys_sync_lock);
	sys_sync_done = true;
	spin_unlock(&suspend_sys_sync_lock);
}
static DECLARE_WORK(suspend_sys_sync_work, suspend_sys_sync);

void his_suspend_sys_sync_queue(void)
{
	int ret = 0;

	spin_lock(&suspend_sys_sync_lock);
	if(sys_sync_done) {
		ret = queue_work(suspend_sys_sync_workqueue, &suspend_sys_sync_work);
		if (ret)
			sys_sync_done = false;
	}
	spin_unlock(&suspend_sys_sync_lock);

}
EXPORT_SYMBOL(his_suspend_sys_sync_queue);

int sys_sync_queue_init(void)
{
	int ret = 0;

	suspend_sys_sync_workqueue =
			create_singlethread_workqueue("suspend_sys_sync");
	if (suspend_sys_sync_workqueue == NULL)
		ret = -ENOMEM;

	sync_wake = wakeup_source_register(NULL, "suspend-syncwake");

	return ret;
}
EXPORT_SYMBOL(sys_sync_queue_init);

