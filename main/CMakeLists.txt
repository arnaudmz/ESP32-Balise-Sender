#idf_component_register(SRCS "main.cpp" "Config.cpp" "Beacon.cpp" "Switches.cpp" "TinyGPS++.cpp" "GPSCnx.cpp" "LED.cpp" "BLEConfig.cpp" "SPort.cpp" "Jeti.cpp" "JetiScreen.cpp"
#                    INCLUDE_DIRS "."
#                    )

set(COMPONENT_ADD_INCLUDEDIRS "")
set(COMPONENT_SRCS "main.cpp" "Config.cpp" "Beacon.cpp" "Switches.cpp" "TinyGPS++.cpp" "GPSCnx.cpp" "LED.cpp" "SPort.cpp" "Jeti.cpp" "JetiScreen.cpp")
#set(COMPONENT_REQUIRES soc nvs_flash ulp driver app_update)
#set(COMPONENT_REQUIRES freertos soc nvs_flash ulp driver app_update)
#set(EXCLUDE_COMPONENTS wpa_supplicant coap asio lwip)

if(CONFIG_IDF_TARGET_ESP32)
  list(APPEND COMPONENT_SRCS "BLEConfig.cpp")
#  list(APPEND COMPONENT_REQUIRES bluetooth)
  set(ulp_sources ../ulp/led.S)
elseif(CONFIG_IDF_TARGET_ESP32S2)
  set(ulp_sources ../ulp_riscv/main.c)
#  list(APPEND COMPONENT_REQUIRES tinyusb)
  if(CONFIG_USB_MSC_ENABLED)
    list(APPEND COMPONENT_SRCS "msc_disk.cpp")
  endif()
endif()

register_component()
if(NOT CONFIG_IDF_TARGET_ESP32C3)
set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_exp_dep_srcs "LED.cpp")

ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
endif()
