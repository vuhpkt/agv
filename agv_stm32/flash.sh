#!/usr/bin/env bash
set -e

echo "==> [AGV STM32] Dang kiem tra build va nap firmware qua ST-Link..."
cmake --build build/Debug -t flash
echo "==> [AGV STM32] Nap code va Reset chip thanh cong!"

