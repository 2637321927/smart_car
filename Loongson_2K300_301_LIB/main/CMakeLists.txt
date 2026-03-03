cmake_minimum_required(VERSION 3.16)    # 限制最低CMake版本

# 设置交叉编译工具路径 --> (需根据自己实际路径进行修改)
set(CROSS_COMPILER_DIR /opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1)

# 指定交叉编译工具所使用的C语言编译器
set(CMAKE_C_COMPILER ${CROSS_COMPILER_DIR}/bin/loongarch64-linux-gnu-gcc)
# 指定交叉编译工具所使用的C++编译器
set(CMAKE_CXX_COMPILER ${CROSS_COMPILER_DIR}/bin/loongarch64-linux-gnu-g++)

# 指定OpenCV所用头文件路径 --> (需根据自己实际路径进行修改)
include_directories(/home/lq/Desktop/opencv/install/include/opencv4/)
# 找到opencv所需的库文件并保存到变量中 --> (需根据自己实际路径进行修改)
set(LQ_OPENCV_LIB_PATH /home/lq/Desktop/opencv/install/lib/)

# 指定NCNN安装路径 --> (需根据自己实际路径进行修改)
set(NCNN_ROOT /home/lq/Desktop/ncnn/ncnn/build-loongarch/install)
include_directories(${NCNN_ROOT}/include)
find_library(NCNN_LIB ncnn ${NCNN_ROOT}/ncnn-lib)

# 项目名称
project(main)

# 指定所用的源文件路径(与 file(GLOB LIB ${XXX}) 功能相同，但可将所有参数保存到一个变量中)
aux_source_directory(${PROJECT_SOURCE_DIR}/ SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/app/ SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/app/gyro SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/app/display SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/common/src SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/drv/src SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../example/src SRC)


# 指定所用的头文件路径
include_directories(${PROJECT_SOURCE_DIR}/)
include_directories(${PROJECT_SOURCE_DIR}/../libraries/app/)
include_directories(${PROJECT_SOURCE_DIR}/../libraries/app/gyro)
include_directories(${PROJECT_SOURCE_DIR}/../libraries/app/display)
include_directories(${PROJECT_SOURCE_DIR}/../libraries/common/inc)
include_directories(${PROJECT_SOURCE_DIR}/../libraries/drv/inc)
include_directories(${PROJECT_SOURCE_DIR}/../example/inc)


# 指定OpenCV所用头文件路径
include_directories(${LQ_OPENCV_INC_PATH})
# 找到opencv所需的库文件并保存到变量中(原始库名 + 存储变量名)
set(OPENCV_LIB_NAMES
    opencv_core      
    opencv_imgproc
    opencv_highgui
    opencv_videoio
    opencv_imgcodecs
    opencv_dnn
)
set(OPENCV_VAR_NAMES
    OPENCV_CORE
    OPENCV_IMGPROC
    OPENCV_HIGHGUI
    OPENCV_VIDEOIO
    OPENCV_IMGCODECS
    OPENCV_DNN
)
# 查找库 + 结果检查
foreach(LIB_NAME VAR_NAME IN ZIP_LISTS OPENCV_LIB_NAMES OPENCV_VAR_NAMES)
    # 查找库
    find_library(${VAR_NAME} ${LIB_NAME} ${LQ_OPENCV_LIB_PATH})
    # 检查结果: 找不到报警告, 找到则打印状态
    if (NOT ${VAR_NAME})
        message(WARNING "cannot find OpenCV lib ${LIB_NAME} (path : ${LQ_OPENCV_LIB_PATH})")
    else()
        message(STATUS "LQ [info ] find OpenCV lib ${LIB_NAME}\t (path : ${LQ_OPENCV_LIB_PATH})")
    endif()
endforeach()

# 判断是否全部找到
set(ALL_OPENCV_FOUND ON)
foreach(LIB_NAME VAR_NAME IN ZIP_LISTS OPENCV_LIB_NAMES OPENCV_VAR_NAMES)
    if (NOT ${VAR_NAME})
        set(ALL_OPENCV_FOUND OFF)
        break() # 只要有一个没找到，直接跳出循环
    endif()
endforeach()
if (ALL_OPENCV_FOUND)
    add_definitions(-DLQ_HAVE_OPENCV)
    message(STATUS "LQ [info ] all OpenCV libs found, added LQ_HAVE_OPENCV compilation definitions")
    set(ALL_OPENCV_FOUND ON)
else()
    message(STATUS "LQ [info ] some OpenCV libs not found, skipped LQ_HAVE_OPENCV compilation definitions")
    set(ALL_OPENCV_FOUND OFF)
endif()

# 若需使用静态库，可带上下面两行程序，具体可参考龙邱B站视频教程
# link_directories(${PROJECT_SOURCE_DIR}/a)
# link_libraries(ss)

# 由源文件生成一个可执行程序
add_executable(main ${SRC})

# 连接OpenCV动态库
if (ALL_OPENCV_FOUND)
    target_link_libraries(main PUBLIC ${OPENCV_CORE} ${OPENCV_IMGPROC} ${OPENCV_HIGHGUI} ${OPENCV_VIDEOIO} ${OPENCV_IMGCODECS} ${OPENCV_DNN})
else()
    message(STATUS "LQ [info ] skipped OpenCV libraries")
endif()

# 连接NCNN库
target_link_libraries(main PUBLIC ${NCNN_LIB})

# 连接多线程pthread库
target_link_libraries(main PUBLIC pthread)
