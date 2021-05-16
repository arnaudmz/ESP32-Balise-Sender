#!/usr/bin/env bash

OPENOCD_PATH=~/.espressif/tools/openocd-esp32/v0.10.0-esp32-20201202/openocd-esp32/

$OPENOCD_PATH/bin/openocd -f openocd.cfg -f target/esp32.cfg -c "program_esp 0x1000 bootloader/bootloader.bin verify exit"
