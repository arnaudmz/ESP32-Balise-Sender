set(COMPONENT_ADD_INCLUDEDIRS "../ulp_riscv")
set(COMPONENT_SRCS "main.cpp" "Config.cpp" "Beacon.cpp" "BLEConfig.cpp" "Switches.cpp" "TinyGPS++.cpp" "GPSCnx.cpp" "LED.cpp" "ULP.cpp" "Jeti.cpp" "JetiScreen.cpp")

if(CONFIG_IDF_TARGET_ESP32)
  set(ulp_sources ../ulp/ulp.S)
  list(APPEND COMPONENT_SRCS "SPort.cpp")
elseif(CONFIG_IDF_TARGET_ESP32S3)
  list(APPEND COMPONENT_SRCS "SPortULP.cpp")
  set(ulp_sources "../ulp_riscv/main.c" "../ulp_riscv/dbg.c" "../ulp_riscv/uart.c" "../ulp_riscv/led.c" "../ulp_riscv/sport.c")
endif()

register_component()
set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_exp_dep_srcs "ULP.cpp" "LED.cpp")
if(CONFIG_IDF_TARGET_ESP32S3)
	list(APPEND ulp_exp_dep_srcs "SPortULP.cpp" "LED.cpp")
endif()

ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
