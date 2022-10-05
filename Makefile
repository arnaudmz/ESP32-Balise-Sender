#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := ESP32-Balise-Sender

include $(IDF_PATH)/make/project.mk

.PHONY: configs configs_clean erase_nvs
configs:	nvs_configs/template.csv nvs_configs/values.csv
	@$(IDF_PATH)/tools/mass_mfg/mfg_gen.py generate --fileid name $^ Config 0x3000

configs_clean:
	@rm -rf bin csv

erase_nvs:
	esptool.py --before default_reset --after hard_reset --chip esp32 erase_region 0x9000 12288


# to flash config: esptool.py --before default_reset --after hard_reset --chip esp32  write_flash 0x9000 bin/Config-DEFAULT-0000.bin
# to flash bootloader + partition table + application: esptool.py --chip esp32 -p /dev/ttyUSB0 -b 921600 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 4MB 0x8000 partition_table/partition-table.bin 0x1000 bootloader/bootloader.bin 0x10000 ESP32-Balise-Sender.bin
# to flash bootloader + partition table + application + NVS: esptool.py --chip esp32 -p /dev/ttyUSB0 -b 921600 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 4MB 0x8000 partition-table-v0.17.bin 0x1000 bootloader-v0.17.bin 0x10000 ESP32-Balise-Sender-v0.17.bin 0x9000 bin/Config-MOCK-0000.bin
# to build a NVS-only DFU stuff: $IDF_PATH/tools/mkdfu.py write -o plop 0x9000 bin/Config-MOCK-1111.bin --pid 2
# to dfu a single DFU stuff (built previously): dfu-util -d 303a:2 -D plop
