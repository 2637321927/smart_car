#!/bin/bash

# 该指令表示任意指令执行失败，立即终止脚本
set -e

echo "=== 删除旧的 build 目录 ==="
rm -rf build

echo -e "\n=== 构建 build目录 ==="
cmake -B build

echo -e "\n=== 构建项目 ==="
cmake --build build -j7
