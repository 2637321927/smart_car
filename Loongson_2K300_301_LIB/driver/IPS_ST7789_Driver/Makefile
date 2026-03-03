# 本文件中只需要修改 KDIR 的路径，指向一个内核根目录的路径
KDIR:=/home/linux/LQ_Project/Loongson/linux-4.19
ARCH=loongarch
CROSS_COMPILE=loongarch64-linux-gnu-
PWD?=$(shell pwd)

BUILD_DIR=build
INCLUDE=inc
KO_DIR=ko

EXTRA_CFLAGS += -I$(PWD)/$(INCLUDE)/

obj-m += IPS_ST7789_dev.o
obj-m += IPS_ST7789_dri.o

all: clean prepare compile organize

prepare:
	@mkdir -p $(KO_DIR) 2>/dev/null || true
	@mkdir -p $(BUILD_DIR) 2>/dev/null || true

compile:
	make -C $(KDIR) M=$(PWD) modules

organize:
	@find . -path ./$(BUILD_DIR) -prune                 \
	        -o -type f ! -name '*.c' ! -name '*.h'      \
		    ! -name 'Makefile' ! -name '*.ko' -exec rm -f {} \;
	@find . -path ./$(BUILD_DIR) -prune -o -type f -name '*.mod.c' -exec mv {} $(BUILD_DIR) \;
	@find . -path ./$(KO_DIR) -prune -o -type f -name '*.ko' -exec mv {} $(KO_DIR) \;
	@rm -f *.o *.mod.o .*.cmd
clean:
	rm $(BUILD_DIR)/* $(BUILD_DIR)/.*.cmd $(BUILD_DIR)/.LQ* $(KO_DIR)/* .tmp* -rf
