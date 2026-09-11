#!/usr/bin/env bash

echo "==> [AGV STM32] Khoi dong OpenOCD RTT Server tren cong 9090..."
echo "==> [AGV STM32] Sau khi OpenOCD chay, mo mot tab Git Bash khac va go: nc localhost 9090"
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
    -c "init" \
    -c "rtt setup 0x20000000 0x5000 \"SEGGER RTT\"" \
    -c "rtt polling_interval 20" \
    -c "rtt start" \
    -c "rtt server start 9090 0"

