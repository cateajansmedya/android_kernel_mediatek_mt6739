#
# Makefile for MMC/SD card drivers
#
#ccflags-y += -Wno-error=unused-variable
#ccflags-y += -Wno-error=unused-function

obj-$(CONFIG_MMC_BLOCK)		+= mmc_block.o
mmc_block-objs			:= block.o queue.o
obj-$(CONFIG_MMC_TEST)		+= mmc_test.o

obj-$(CONFIG_SDIO_UART)		+= sdio_uart.o
obj-$(CONFIG_MMC_FFU)		+= ffu.o
obj-$(CONFIG_MMC_BLOCK_IO_LOG)	+= mtk_mmc_block.o
