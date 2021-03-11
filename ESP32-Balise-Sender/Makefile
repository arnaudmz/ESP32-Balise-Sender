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
