# Installation

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

The repository is `mcu-baro`. The existing forge package key, C API and
configuration options retain the `baro` name. The clone command below
also preserves `baro` as the ESP-IDF component directory.

## With forge

stm-forge must be at `ee921c3` or later ("build: libraries may keep sources
in src/ subfolders"); older ones only compile `src/*.c`, so the chip drivers
in `src/<chip>/` are never built and the link fails with undefined
references to the chip probes. esp-forge is unaffected: the component's
CMakeLists lists its sources.

With [esp-forge](https://github.com/JonasAtelier/esp-forge) or
[stm-forge](https://github.com/JonasAtelier/stm-forge). The catalog line in
`profiles/pkgversion` is the same for both:

```make
baro_GIT=https://github.com/JonasAtelier/mcu-baro@stable
```

and a target turns the library and its chips on in `configs/<target>_defconfig`:

```make
PKG_baro            = y
BARO_BMP280         = y
# BARO_BMP388 is not set
# BARO_DPS310 is not set
# BARO_MS5611 is not set
```

or with `make menu`. esp-forge turns each line into `CONFIG_BARO_<CHIP>=y` in
`sdkconfig.defaults`; stm-forge passes it as `-DCONFIG_BARO_<CHIP>=1`.
`BARO_DEBUG = y` turns on `baro_dbg()` lines on the HAL; on ESP-IDF the log
level decides.

## ESP-IDF

Clone into your project's `components/` directory. A local
component is named after its directory, and that name is what you
`REQUIRES`:

```sh
git clone https://github.com/JonasAtelier/mcu-baro components/baro
```

```cmake
idf_component_register(SRCS "main.c" REQUIRES baro driver)
```

Enable the chips under `idf.py menuconfig` → **Component config → Barometer
subsystem**, or put `CONFIG_BARO_<CHIP>=y` in your `sdkconfig.defaults` as
each example in `examples/esp/` does. With the subsystem on and no chip
picked, the build warns. ESP-IDF 5.2 or newer is required for the
`i2c_master` API.

## STM32 HAL

Copy `include/` and `src/` into an STM32 project, compile every `.c` under
`src/`, chip folders included, add `include/` and `src/` to the include path
(`src/` only needs to reach this library's own sources), and define what the
forge would: `CMSIS_device_header` as your family header (`"stm32f4xx.h"` or
`"stm32f7xx.h"`), `USE_HAL_DRIVER`, and `CONFIG_BARO_<CHIP>=1` for each chip
you want.

**No chip driver is built by default.** Like a kernel subsystem, the core is
always built and each driver is opt-in: a chip whose option is absent or 0
compiles to nothing. With the library on and no chip picked, the ESP-IDF
CMakeLists warns and `src/baro_port_stm.c` emits a `#warning`.

**The HAL module is yours to enable.** Every driver calls the HAL's I2C, so
your app's `stm32f4xx_hal_conf.h` (or `stm32f7xx_hal_conf.h`) must define it,
and its `.c` must be in the build:

| Option | Needs in hal_conf | HAL source |
|---|---|---|
| `BARO_BMP280`, `BARO_BMP388`, `BARO_DPS310`, `BARO_MS5611` | `HAL_I2C_MODULE_ENABLED` | `stm32f{4,7}xx_hal_i2c.c` |

The bus layer under the drivers also has SPI and UART code, which no chip
uses yet; it sits under `HAL_SPI_MODULE_ENABLED` and
`HAL_UART_MODULE_ENABLED`, so an app without those modules builds as it is.
stm-forge's `mcu/stm32f4.cmake` and `mcu/stm32f7.cmake` already list the I2C
source, plus `_hal_i2c_ex.c` (for the `HAL_I2CEx_ConfigAnalogFilter()` that
CubeMX-style init calls); each compiles to nothing until hal_conf enables the
module. A chip whose module is off stops the build with an `#error` that says
which define is missing.

**printf() is the log.** Lines go out through `printf()`, so retarget
newlib's `_write()` to a UART. stm-forge links newlib-nano
(`--specs=nano.specs`), whose `printf()` drops `%f` unless the app links with
`-u _printf_float`: add `target_link_options(firmware PRIVATE -u _printf_float)`
to your app's `CMakeLists.txt`, or `baro_log_sample()` prints blanks where the
numbers go.

**`baro_altitude_m()` needs libm.** It calls `powf()`. ESP-IDF and stm-forge
link the maths library for every firmware, so there is nothing to do there;
any other build that calls it needs `-lm`, or the link stops at
`undefined reference to 'powf'`.
