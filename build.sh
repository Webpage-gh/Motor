#!/data/data/com.termux/files/usr/bin/sh
# 编译（Termux 自带 clang）
set -e
cd "$(dirname "$0")"
clang -O2 -o motor motor.c -lm
chmod 755 motor
echo "已生成: $(pwd)/motor"
