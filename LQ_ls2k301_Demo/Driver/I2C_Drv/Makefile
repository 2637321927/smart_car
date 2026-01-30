# 本文件中只需要修改 KDIR 的路径，指定一下内核根目录的路径
KDIR:=/home/linux/LQ_Project/Loongson/linux-4.19
ARCH:=loongarch
CROSS_COMPILE:=loongarch64-linux-gnu-
PWD?=$(shell pwd)

BUILD_DIR:=build
INCLUDE:=inc
KO_DIR:=ko
SRC_DIR:=src

EXTRA_CFLAGS += -I$(PWD)/$(INCLUDE)/

# 设备端主程序
obj-m += lq_i2c_dev.o
lq_i2c_dev-objs := LQ_I2C_ALL_DEV.o

# 驱动端主程序
obj-m += lq_i2c_drv.o
lq_i2c_drv-objs := LQ_I2C_ALL_DRV.o \
				   $(SRC_DIR)/LQ_MPU6050_drv.o \
				   $(SRC_DIR)/LQ_VL53_drv.o \
				   $(SRC_DIR)/LQ_I2C_RDWR_drv.o \
				   $(SRC_DIR)/LQ_util_drv.o \
				   $(SRC_DIR)/LQ_List_drv.o \
				   $(SRC_DIR)/LQ_ICM42605_drv.o

all: prepare compile move_files
	@echo "编译完成! ko模块在 $(KD_DIR) 目录, 中间文件在 $(BUILD_DIR) 目录"

# 步骤 1: 提前创建目录
prepare:
	@mkdir -p $(BUILD_DIR) $(KO_DIR)
	@echo "已创建目录"

# 步骤2: 编译模块
compile:
	make -C $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules

# 步骤3: 移动文件
move_files:
# 移动中间文件（.o/.mod.o/.mod.c/.cmd等）到build目录
	@find . -type f \
		-not -path "./$(BUILD_DIR)/*" -not -path "./$(KO_DIR)/*" \
		\( -name '*.o' -o -name '*.mod.o' -o -name '*.mod.c' -o -name '.*.cmd' -o -name 'modules.order' -o -name 'Module.symvers' \) \
		! -name '*.ko' -exec mv -t $(BUILD_DIR)/ {} +
# 移动 ko 模块到 ko 目录
	@find . -type f \
		-not -path "./$(BUILD_DIR)/*" -not -path "./$(KO_DIR)/*" \
		-name '*.ko' -exec mv -t $(KO_DIR)/ {} +
# 清理残留的临时文件
	@rm -rf .tmp_versions $(SRC_DIR)/.tmp_versions

# 清理目标
clean:
	@rm -rf $(BUILD_DIR) $(KO_DIR) .tmp_versions $(SRC_DIR)/.tmp_versions
	@rm -f *.o *.mod.o *.mod.c *.ko modules.order Module.symvers .*.cmd
	@echo "已清理所有编译产物"
