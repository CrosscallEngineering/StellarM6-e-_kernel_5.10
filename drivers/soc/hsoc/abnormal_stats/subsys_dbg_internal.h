
#ifndef __SUBSYS_DBG_INTERNAL_H__
#define __SUBSYS_DBG_INTERNAL_H__

#define KHEADER_LEN      	 2048
#define KMSG_LEN        	 2048
#define TRACE_LEN        	 10240

#define KDUMPER_LEN			 25600

#define PANIC_HEADER_INDEX		"Kernel panic"
#define TRACE_HEADER_INDEX		"Call trace:"

#define KDEBUG_SAVE_MAGIC       0x5353ACAC

#define KMSG_MAGIC_MASK 	0xF0
#define TRACE_MAGIC_MASK 	0x0F

struct subsys_trap_info {
	char header[KHEADER_LEN];
	char kmsg[KMSG_LEN];
	char subsys_msg[KMSG_LEN];
	char trace[TRACE_LEN];
	int saved;
};

#endif /* __SUBSYS_DBG_INTERNAL_H__ */

