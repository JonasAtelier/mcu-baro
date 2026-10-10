CC ?= cc
# gnu11, not c11: the baro_* log macros use `, ##__VA_ARGS__`.
CFLAGS += -std=gnu11 -Wall -Wextra -Werror -pedantic
# src/ for the common private header, and every chip folder, because the
# tests include a chip's private headers by plain name.
CPPFLAGS += -I../include -I../src $(addprefix -I,$(wildcard ../src/*/))

# Two full suites, one per platform: the same chip drivers through
# src/baro_port_esp.c against idf_stubs/ (esp/), and through
# src/baro_port_stm.c against hal_stubs/ (stm/). The four maths tests touch
# no bus or platform API and were the same in both original repos, so they
# run once; three include a public header, which wants a family header, so
# they build with the HAL stub's flags too.

# What ESP-IDF passes: ESP_PLATFORM, and every chip on.
ESP_CPPFLAGS := -Iidf_stubs -DESP_PLATFORM \
		-DCONFIG_BARO_BMP280=1 -DCONFIG_BARO_BMP388=1 \
		-DCONFIG_BARO_DPS310=1 -DCONFIG_BARO_MS5611=1
# What stm-forge passes: the family header the code includes, the HAL modules
# an app's hal_conf would enable, and every chip option on (BARO_<CHIP> = y).
STM_CPPFLAGS := -Ihal_stubs \
		-DCMSIS_device_header='"stm32_hal_stub.h"' \
		-DHAL_I2C_MODULE_ENABLED -DHAL_SPI_MODULE_ENABLED \
		-DHAL_UART_MODULE_ENABLED \
		-DCONFIG_BARO_BMP280=1 -DCONFIG_BARO_BMP388=1 \
		-DCONFIG_BARO_DPS310=1 -DCONFIG_BARO_MS5611=1 \
		-DCONFIG_BARO_DEBUG=1

# The core and the stubs every binary of a suite links.
CORE     := ../src/baro.c ../src/baro_log.c
ESP_CORE := $(CORE) ../src/baro_port_esp.c idf_stubs/idf_stubs.c
STM_CORE := $(CORE) ../src/baro_port_stm.c hal_stubs/hal_stubs.c

MATH_TESTS := test_bmp280_math test_bmp388_math test_dps310_math \
	      test_ms5611_math
SUITE      := test_core test_bmp280 test_bmp388 test_dps310 test_ms5611
# Plus one port test each, for the SPI and UART ops no chip uses yet.
ESP_TESTS  := $(addprefix esp/,$(SUITE) test_port_esp)
STM_TESTS  := $(addprefix stm/,$(SUITE) test_port_stm)
TESTS      := $(MATH_TESTS) $(ESP_TESTS) $(STM_TESTS)

.PHONY: all clean test

all: $(TESTS)

test_bmp280_math: test_bmp280_math.c
test_bmp388_math: test_bmp388_math.c
test_dps310_math: test_dps310_math.c
test_ms5611_math: test_ms5611_math.c

esp/test_core: esp/test_core.c $(ESP_CORE)
esp/test_bmp280: esp/test_bmp280.c ../src/bmp280/bmp280.c $(ESP_CORE)
esp/test_bmp388: esp/test_bmp388.c ../src/bmp388/bmp388.c $(ESP_CORE)
esp/test_dps310: esp/test_dps310.c ../src/dps310/dps310.c $(ESP_CORE)
esp/test_ms5611: esp/test_ms5611.c ../src/ms5611/ms5611.c $(ESP_CORE)
esp/test_port_esp: esp/test_port_esp.c $(ESP_CORE)

stm/test_core: stm/test_core.c $(STM_CORE)
stm/test_bmp280: stm/test_bmp280.c ../src/bmp280/bmp280.c $(STM_CORE)
stm/test_bmp388: stm/test_bmp388.c ../src/bmp388/bmp388.c $(STM_CORE)
stm/test_dps310: stm/test_dps310.c ../src/dps310/dps310.c $(STM_CORE)
stm/test_ms5611: stm/test_ms5611.c ../src/ms5611/ms5611.c $(STM_CORE)
stm/test_port_stm: stm/test_port_stm.c $(STM_CORE)

$(ESP_TESTS): CPPFLAGS += $(ESP_CPPFLAGS)
$(MATH_TESTS) $(STM_TESTS): CPPFLAGS += $(STM_CPPFLAGS)

# Lets the driver tests count calloc() calls, to prove a refused probe
# allocates nothing -- and on the HAL free() too, that a failed one leaks
# nothing. GNU ld; only the drivers' own references are wrapped.
esp/test_bmp280 esp/test_bmp388 esp/test_dps310 esp/test_ms5611: \
	LDFLAGS += -Wl,--wrap=calloc
# Every printf() in a binary that links the HAL stubs also lands in
# stub_log_last, so a test can check what a driver logged.
$(STM_TESTS): LDFLAGS += -Wl,--wrap=printf
stm/test_bmp280 stm/test_bmp388 stm/test_dps310 stm/test_ms5611: \
	LDFLAGS += -Wl,--wrap=calloc -Wl,--wrap=free

$(TESTS):
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) $(filter %.c,$^) -o $@ -lm

test: $(TESTS)
	@set -e; for t in $(TESTS); do echo "./$$t"; ./$$t; done

clean:
	rm -f $(TESTS)

# Headers are prerequisites so that editing one rebuilds. Without this, `make
# test` reruns a stale binary and reports green for code it never compiled.
# Recipes use $(filter %.c,$^) so the headers never reach the compiler.
HDRS  := $(wildcard ../include/*.h ../src/*.h ../src/*/*.h)
STUBS := $(wildcard idf_stubs/*.h idf_stubs/*/*.h hal_stubs/*.h)
$(TESTS): $(HDRS) $(STUBS)
