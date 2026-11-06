#!/data/data/com.termux/files/usr/bin/sh
# 编译（Termux 自带 clang）
set -e
cd "$(dirname "$0")"
clang -O2 -o motor motor.c -lm
chmod 755 motor
echo "已生成: $(pwd)/motor"

# 主旋律提取器（extract_lead.py 的 C 移植，见 tools/extract.c）
clang -O2 -std=c11 -D_GNU_SOURCE -o tools/extract tools/extract.c -lm
chmod 755 tools/extract
echo "已生成: $(pwd)/tools/extract"
