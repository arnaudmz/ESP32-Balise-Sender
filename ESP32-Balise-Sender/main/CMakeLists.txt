idf_component_register(SRCS "Beacon.cpp" "Switches.cpp" "main.cpp" "TinyGPS++.cpp" "GPSCnx.cpp" "LED.cpp" "Config.cpp" "Switches.cpp" "Beacon.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES soc ulp app_update nvs_flash)

set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_s_sources ulp/led.S)
set(ulp_exp_dep_srcs "LED.cpp")

ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
