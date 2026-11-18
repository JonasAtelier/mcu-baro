# mcu-baro

Read air pressure in pascals and temperature in °C from a BMP280, BMP388, DPS310 or MS5611 over I2C. Pressure changes can also tell you how far a sensor has climbed or descended.

**Platforms** · ESP-IDF 5.2+ and STM32 HAL (F4/F7). Hardware testing is pending. The host tests use simulated devices.

## Contents

- [Quick start](#quick-start)
- [Drivers and examples](#drivers-and-examples)
- [Using the library in your project](#using-the-library-in-your-project)
- [License](#license)

## Quick start

Connect a BMP280 over I2C, pull CSB high and use address 0x76 for this example.

In a terminal with ESP-IDF activated, get the example and select the target for an ESP32. Change the target if your board uses another ESP32 chip.

```sh
git clone https://github.com/JonasAtelier/mcu-baro.git baro
cd baro/examples/esp/bmp280
idf.py set-target esp32
```

Check the [wiring guide](docs/hardware.md) and edit the settings in [main.c](examples/esp/bmp280/main/main.c) to match your board. Connect the board by USB, then build, upload and open the serial monitor.

```sh
idf.py build flash monitor
```

The monitor prints pressure, temperature and height relative to the first reading. For altitude above sea level, supply a reference pressure for your location.

<details>
<summary>STM32 HAL example for an initialised bus</summary>

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

</details>

## Drivers and examples

All four I2C examples use `baro_read()` for compensated pressure and temperature. The altitude helper takes a reference pressure in pascals. [Shared API](include/baro.h)

| Chip | Driver header | ESP-IDF project | STM32 HAL example |
|---|---|---|---|
| [BMP280](docs/chips.md#bmp280) | [bmp280.h](include/bmp280.h) | [Forced measurement](examples/esp/bmp280/) | [Forced measurement](examples/stm/bmp280/main.c) |
| [BMP388](docs/chips.md#bmp388) | [bmp388.h](include/bmp388.h) | [Forced measurement](examples/esp/bmp388/) | [Forced measurement](examples/stm/bmp388/main.c) |
| [DPS310](docs/chips.md#dps310) | [dps310.h](include/dps310.h) | [Pressure and relative height](examples/esp/dps310/) | [Pressure and relative height](examples/stm/dps310/main.c) |
| [MS5611](docs/chips.md#ms5611) | [ms5611.h](include/ms5611.h) | [Pressure and relative height](examples/esp/ms5611/) | [Pressure and relative height](examples/stm/ms5611/main.c) |

## Using the library in your project

- Add the component and enable the driver for your hardware. [Installation and build options](docs/installation.md)
- Open the ESP-IDF project for your hardware, or copy the STM32 example into a project with board setup already in place. [All ESP-IDF examples](examples/esp/) · [All STM32 examples](examples/stm/)
- Look up calls and chip-specific settings. [API usage](docs/usage.md) · [Hardware reference](docs/chips.md)

To run the simulated tests on your computer, run `make -C test test` from the library root. [Test requirements and coverage](docs/development.md)

## License

[MIT](LICENSE)

[Browse the workshop catalog · ESP32 and STM32 drivers](https://github.com/JonasAtelier/jonas-atelier-libraries#esp32-and-stm32-drivers)
