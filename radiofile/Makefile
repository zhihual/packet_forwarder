obj-m := loradio.o
#example-objs := main.o
loradio-objs := main.o common.o hal.o sx1276.o async.o
PWD = $(shell pwd)
KDIR := /home/share/lora/openwrt/dev/openwrt_barri_origin/build_dir/target-mips_34kc_uClibc-0.9.33.2/linux-ar71xx_generic/linux-3.10.49/


all:
	make -C $(KDIR) ARCH="mips" CROSS_COMPILE="mips-openwrt-linux-uclibc-" M=$(PWD) modules
clean:
	rm -rf *.o *.ko


