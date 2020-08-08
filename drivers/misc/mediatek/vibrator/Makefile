obj-$(CONFIG_MTK_VIBRATOR) := vibrator_drv.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/vibrator/$(MTK_PLATFORM)

ifneq ($(CONFIG_C8_NRF_CONTROL),y)
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif
