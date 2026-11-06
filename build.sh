#!/bin/sh
set -e
cd "$(dirname "$0")"

# 马达调用器
clang -O2 -o motor src/motor.c -lm
echo "已生成: $(pwd)/motor"

# 主旋律提取器
clang -O2 -std=c11 -D_GNU_SOURCE -o extract src/extract.c -lm
echo "已生成: $(pwd)/extract"
