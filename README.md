# mcu-baro

<p align="center">
  <img alt="platform: ESP-IDF | STM32 HAL" src="https://img.shields.io/badge/platform-ESP--IDF%20%7C%20STM32%20HAL-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

A small C barometer library for **ESP-IDF 5.2+** and **STM32 HAL (F4/F7)**.
Four chip drivers share one API for timestamped samples of pressure in Pa and
temperature in °C, and `baro_altitude_m()` turns a pressure into height above
a reference. This library replaces `esp-baro` and `stm-baro`.

The repository is named `mcu-baro`; the C API, `CONFIG_BARO_*` options, and
forge package key remain `baro`. The clone command below keeps the ESP-IDF
component directory named `baro` as well.

> **Hardware unverified.** Host tests use simulated devices and platform stubs.
> Nothing here has run on a physical chip.

[Supported hardware](#supported-hardware) · [Drivers](#drivers) ·
[Install](#install) ·
[Quick start](#quick-start) · [Examples](#examples) ·
[Documentation](#documentation) · [Tests](#tests)

## Supported hardware

All four drivers support both platforms. Enable only the drivers you need.

| Chip | Bus | Notes |
|---|---|---|
| **BMP280** | I2C | Bosch pressure and temperature sensor |
| **BMP388** | I2C | Bosch pressure and temperature sensor |
| **DPS310** | I2C | Infineon pressure and temperature sensor |
| **MS5611** | I2C | TE Connectivity pressure and temperature sensor |

All four parts can also do SPI; no driver here does. Wiring for every chip is
in the [wiring guide](docs/hardware.md).

## Drivers

<details><summary><b>BMP280</b> — I2C</summary>

Address `0x76` or `0x77` by SDO; forced mode; a BME280 is refused by ID.

- Examples: ESP-IDF [bmp280](examples/esp/bmp280/) · STM32 HAL
  [bmp280](examples/stm/bmp280/main.c)
- Chip reference: [BMP280](docs/chips.md#bmp280)

</details>

<details><summary><b>BMP388</b> — I2C</summary>

Address `0x76` or `0x77` by SDO; forced mode; a BMP390 is refused by ID.

- Examples: ESP-IDF [bmp388](examples/esp/bmp388/) · STM32 HAL
  [bmp388](examples/stm/bmp388/main.c)
- Chip reference: [BMP388](docs/chips.md#bmp388)

</details>

<details><summary><b>DPS310</b> — I2C</summary>

Address `0x77` or `0x76` by SDO; background mode, with data ready and raw
access.

- Examples: ESP-IDF [dps310](examples/esp/dps310/) · STM32 HAL
  [dps310](examples/stm/dps310/main.c)
- Chip reference: [DPS310](docs/chips.md#dps310)

</details>

<details><summary><b>MS5611</b> — I2C</summary>

Address `0x77` or `0x76` by CSB; commands, not registers; no data-ready flag.

- Examples: ESP-IDF [ms5611](examples/esp/ms5611/) · STM32 HAL
  [ms5611](examples/stm/ms5611/main.c)
- Chip reference: [MS5611](docs/chips.md#ms5611)

</details>

## Install

### ESP-IDF

Clone into your project's components directory:

```sh
git clone https://github.com/JonasAtelier/mcu-baro components/baro
```

Enable your chips in `idf.py menuconfig` → **Component config → Barometer
subsystem**, or add this to `sdkconfig.defaults` for the BMP280:

```ini
CONFIG_BARO=y
CONFIG_BARO_BMP280=y
```

The `main` component sees the library automatically. Other components declare
`REQUIRES baro driver` in `idf_component_register()`.

### esp-forge / stm-forge

stm-forge must be at `ee921c3` or later, which builds the chip folders in
`src/`; older ones fail at link with undefined chip probes.

Add to `profiles/pkgversion`:

```make
baro_GIT=https://github.com/JonasAtelier/mcu-baro@stable
```

Enable the package and your chip in `configs/<target>_defconfig`:

```make
PKG_baro            = y
BARO_BMP280         = y
```

### STM32 HAL without forge

Compile every `.c` under `src/`, chip folders included, add `include/` and
`src/` to the include path, and define
`CMSIS_device_header` as your family header (`"stm32f4xx.h"` or
`"stm32f7xx.h"`), `USE_HAL_DRIVER`, and `CONFIG_BARO_<CHIP>=1` for each
selected chip. Enable and compile the HAL's I2C module.

**No chip driver is enabled by default.** See the
[installation guide](docs/installation.md) for all chip options, the HAL
module requirement, float logging and libm.

## Quick start

Initialise the bus in your application, then probe the sensor and read it.
This STM32 helper assumes the passed I2C handle is already initialised. The
BMP280 needs no data-ready poll: `baro_read()` triggers one measurement and
waits it out.

```c
#include "baro.h"
#include "baro_log.h"
#include "bmp280.h"

int read_once(I2C_HandleTypeDef *bus)
{
    struct baro_dev baro = { 0 }; /* Must start zeroed. */
    struct baro_sample sample;
    int ret = bmp280_probe(&baro, bus, NULL); /* 0x76, defaults */
    if (ret)
        return ret;

    ret = baro_read(&baro, &sample);
    if (!ret)
        baro_log_sample(&baro, &sample);

    bmp280_remove(&baro);
    return ret;
}
```

Call `read_once(&hi2c1)` from thread context. On ESP-IDF, pass an
`i2c_master_bus_handle_t` instead; the
[BMP280 example](examples/esp/bmp280/main/main.c) shows the bus setup.
For continuous sampling, probe once, read in your loop, and remove when
finished.

- `baro_read()` returns pressure in **Pa** and temperature in **°C**, plus
  `t_us`.
- Integer-returning API calls return **0 or a negative errno**. Use a sample
  only on success; errors leave it unchanged.
- `baro_altitude_m(p, reference)` gives height above the level where the
  pressure is `reference`. Take the reference from a reading of your own:
  against `BARO_SEA_LEVEL_PA` the answer drifts with the weather.

See [API usage](docs/usage.md) for device lifetime, bus handles, timestamps,
delays, altitude, logging, and error codes.

## Examples

Each driver's examples are listed under [Drivers](#drivers).

Every example streams pressure, temperature and height above where it was
switched on. ESP-IDF examples already locate this library through
`EXTRA_COMPONENT_DIRS`. With ESP-IDF activated, build from an example
directory:

```sh
cd examples/esp/bmp280
idf.py set-target esp32 # Replace with your board's target.
idf.py build
```

STM32 examples are snippets
to integrate into your application, with clock setup and logging retargeting
provided by you. Check the example's pin definitions against your board and
read the [wiring guide](docs/hardware.md).

## Scope and limitations

The library provides a shared sample format and synchronous chip drivers.
Your application owns bus setup, scheduling, and locking.

- **I2C only:** all four parts can do SPI; no driver here does.
- **Blocking reads:** no FIFO, interrupts or DMA. The BMP280, BMP388 and
  MS5611 wait out the datasheet maximum inside `baro_read()` — sleeping the
  task on ESP-IDF, busy-waiting in `HAL_Delay()` on the HAL; the DPS310 reads
  its latest background result.
- **No unified oversampling or rate API:** the schemes do not line up —
  forced-mode presets, a background rate and oversampling pair, one OSR for
  two conversions — so a common one would have to round.
- **Concurrency:** use one thread per device and bus handle, or supply a mutex.
  The HAL's own lock answers a clash with `HAL_BUSY`, which comes back as
  `-EAGAIN`.
- **No bus setup:** pins, clocks and speed are the app's.
- **STM32 timestamps:** the default clock has millisecond resolution and wraps
  after 49.7 days. Override `baro_time_us()` if needed.

## Documentation

| Guide | Contents |
|---|---|
| [Installation](docs/installation.md) | Forge, ESP-IDF, STM32 HAL, build options, logging setup and libm |
| [Hardware](docs/hardware.md) | Wiring and pins for each chip, and the shared addresses |
| [API usage](docs/usage.md) | Lifecycle, bus handles, timing, altitude and errors |
| [Chip reference](docs/chips.md) | Modes, waits, compensation, provenance and datasheet assumptions |
| [Development](docs/development.md) | Platform ports, bus abstraction, adding a driver and test coverage |

Public interfaces live in [include/](include/); chip drivers and platform ports
live in [src/](src/). The [repository layout](docs/development.md#repository-layout)
maps every directory and the file-naming rule for chip folders.

## Tests

```sh
make -C test test
```

Host-only tests use stubbed HAL and ESP-IDF APIs; no embedded toolchain or
physical sensor is needed. See the [test reference](docs/development.md#tests)
for compiler requirements and coverage. Error logs during the suite are
expected: tests deliberately exercise failure paths.

## License

[MIT](LICENSE)
