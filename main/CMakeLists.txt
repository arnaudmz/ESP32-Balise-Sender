idf_component_register(SRCS "main.cpp" "Config.cpp" "Beacon.cpp" "Switches.cpp" "TinyGPS++.cpp" "GPSCnx.cpp" "LED.cpp" "BLEConfig.cpp" "SPort.cpp" "Jeti.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES soc ulp app_update nvs_flash bt)

set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_s_sources ../ulp/led.S)
set(ulp_exp_dep_srcs "LED.cpp")

ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
