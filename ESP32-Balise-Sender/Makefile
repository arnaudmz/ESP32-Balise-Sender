#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := ESP32-Balise-Sender

include $(IDF_PATH)/make/project.mk

.PHONY: configs configs_clean

configs:	nvs_configs/template.csv nvs_configs/values.csv
	@$(IDF_PATH)/tools/mass_mfg/mfg_gen.py generate --fileid name $^ Config 0x3000

configs_clean:
	@rm -rf bin csv