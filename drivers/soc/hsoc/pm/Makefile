obj-$(CONFIG_HPM_DEBUG)            += hs_pminfo.o
hs_pminfo-y := pminfo_main.o gpio_status.o

ifdef CONFIG_HSUSPEND_SYS_SYNC
hs_pminfo-y += suspend_sync.o
endif

obj-$(CONFIG_HSUSPEND_DEBUG)    += suspend_debug.o