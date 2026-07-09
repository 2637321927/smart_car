KDIR?=/home/linux/Desktop/linux-4.19-loongson-embeded
ARCH:=loongarch
CROSS_COMPILE:=loongarch64-linux-gnu-
PWD?=$(shell pwd)

BUILD_DIR:=build
KO_DIR:=ko

# 无外部头文件就用这行
EXTRA_CFLAGS += -I$(PWD)/

obj-m += lq_i2c_all_dev.o
lq_i2c_all_dev-objs := lq_i2c_all_device.o

all: prepare compile move_files
	@echo "✅ \033[32m编译完成! ko模块在 $(KO_DIR) 目录, 中间文件在 $(BUILD_DIR) 目录\033[0m"

prepare:
	@mkdir -p $(BUILD_DIR) $(KO_DIR)
	@echo "📂 \033[32m已创建目录\033[0m"

compile:
	make -C $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules

move_files:
	@find . -type f \
		-not -path "./$(BUILD_DIR)/*" -not -path "./$(KO_DIR)/*" \
		\( -name '*.o' -o -name '*.mod.o' -o -name '*.mod.c' -o -name '.*.cmd' -o -name 'modules.order' -o -name 'Module.symvers' \) \
		! -name '*.ko' -exec mv -t $(BUILD_DIR)/ {} +
	@find . -type f \
		-not -path "./$(BUILD_DIR)/*" -not -path "./$(KO_DIR)/*" \
		-name '*.ko' -exec mv -t $(KO_DIR)/ {} +
	@rm -rf .tmp_versions

clean:
	@rm -rf $(BUILD_DIR) .tmp_versions
	@rm -f *.o *.mod.o *.mod.c *.ko modules.order Module.symvers .*.cmd
	@echo "已清理所有编译产物"