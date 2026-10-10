# ESP-IDF only. stm-forge never reads this file: it compiles every .c under
# src/, chip folders included, puts src/ on the include path of this library's
# sources and adds include/ -- which is why each port file is empty on the
# other platform. Here the chip files are listed one by one, and
# PRIV_INCLUDE_DIRS "src" lets a chip folder reach baro_priv.h by plain name.
#
# The core is always built, even with CONFIG_BARO off; each chip driver only
# when its Kconfig option is on. Kconfig values are available here during the
# build phase, and only SRCS depends on them -- never REQUIRES, which IDF
# resolves before sdkconfig. That is also why BARO off still builds the core:
# an empty SRCS would need a second, differently-shaped register call, and an
# unreferenced core costs nothing once --gc-sections has run.
set(srcs "src/baro.c" "src/baro_log.c" "src/baro_port_esp.c")

if(CONFIG_BARO AND NOT (CONFIG_BARO_BMP280 OR CONFIG_BARO_BMP388 OR
                        CONFIG_BARO_DPS310 OR CONFIG_BARO_MS5611))
    message(WARNING "baro: CONFIG_BARO is on but no chip driver is "
                    "selected; set CONFIG_BARO_<CHIP>=y "
                    "(esp-forge: BARO_<CHIP> = y)")
endif()

if(CONFIG_BARO_BMP280)
    list(APPEND srcs "src/bmp280/bmp280.c")
endif()
if(CONFIG_BARO_BMP388)
    list(APPEND srcs "src/bmp388/bmp388.c")
endif()
if(CONFIG_BARO_DPS310)
    list(APPEND srcs "src/dps310/dps310.c")
endif()
if(CONFIG_BARO_MS5611)
    list(APPEND srcs "src/ms5611/ms5611.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src"
                       REQUIRES driver
                       PRIV_REQUIRES esp_timer)
