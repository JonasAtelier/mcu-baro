# mcu-baro

Read air pressure in pascals and temperature in °C from a BMP280, BMP388, DPS310 or MS5611 over I2C. Pressure changes can also tell you how far a sensor has climbed or descended.

**Platforms** · ESP-IDF 5.2+ and STM32 HAL (F4/F7). Hardware testing is pending. The host tests use simulated devices.

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

## Using the library in your project

- Add the component and enable the driver for your hardware. [Installation and build options](docs/installation.md)
- Start from an example for your chip. The STM32 examples fit into a project with board setup already in place. [ESP-IDF examples](examples/esp/) · [STM32 examples](examples/stm/)
- Look up calls and chip-specific settings. [API usage](docs/usage.md) · [Hardware reference](docs/chips.md)

To run the simulated tests on your computer, run `make -C test test` from the library root. [Test requirements and coverage](docs/development.md)

## License

[MIT](LICENSE)
