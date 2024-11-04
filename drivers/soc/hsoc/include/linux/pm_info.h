/*
 * Copyright (C) 2013-2014, Inc.
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

#ifndef _PM_INFO_H_
#define _PM_INFO_H_

extern int gpio_status_proc_init(void);

#ifdef CONFIG_HSUSPEND_SYS_SYNC
extern int sys_sync_queue_init(void);
extern void his_suspend_sys_sync_queue(void);
#endif /*CONFIG_HSUSPEND_SYS_SYNC*/

#endif
