#!/usr/bin/env bash
set -e

echo "==> [AGV STM32] Dang bien dich du an..."
if [ ! -d "build/Debug" ]; then
    echo "==> [AGV STM32] Khoi tao thu muc build/Debug voi CMake & Ninja..."
    cmake -B build/Debug -G Ninja
fi

cmake --build build/Debug
echo "==> [AGV STM32] Bien dich hoan tat thanh cong!"

