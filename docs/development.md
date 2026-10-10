# Architecture, driver development and tests

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Repository layout

```
include/                    public API, the only directory apps include
  baro.h                    struct baro_dev, baro_read(), baro_time_us(), baro_altitude_m()
  baro_log.h                baro_err/warn/info/dbg, baro_log_sample()
  bmp280.h bmp388.h dps310.h ms5611.h
                            one header per chip: probe, remove, extras
src/                        private to the library
  baro.c                    the core: timestamp, ops.read, altitude
  baro_log.c                baro_log_sample()
  baro_priv.h               bus ops, attach functions, waits; shared by all chips
  baro_port_esp.c           that header on ESP-IDF; empty elsewhere
  baro_port_stm.c           that header on the STM32 HAL; empty under ESP-IDF
  bmp280/                   bmp280.c  bmp280_regs.h  bmp280_math.h  bmp280_priv.h
  bmp388/                   bmp388.c  bmp388_regs.h  bmp388_math.h  bmp388_priv.h
  dps310/                   dps310.c  dps310_regs.h  dps310_math.h  dps310_priv.h
  ms5611/                   ms5611.c  ms5611_proto.h ms5611_math.h  ms5611_priv.h
test/                       host tests, `make -C test test`
  test_<chip>_math.c        maths tests: no bus and no platform API
  esp/                      suites on idf_stubs/, through baro_port_esp.c
  stm/                      suites on hal_stubs/, through baro_port_stm.c
  idf_stubs/  hal_stubs/    the slice of ESP-IDF and of the HAL the code calls
examples/
  esp/<chip>/               ESP-IDF projects: bmp280, bmp388, dps310, ms5611
  stm/<chip>/               STM32 snippets: bmp280, bmp388, dps310, ms5611
docs/                       the five guides; datasheet PDFs here are gitignored
assets/                     chip-reference figures
CMakeLists.txt              ESP-IDF only; stm-forge globs src/**/*.c instead
Kconfig                     BARO and BARO_<CHIP>, for ESP-IDF and both forges' menus
idf_component.yml           ESP-IDF component manifest
```

Naming inside `src/<chip>/`, the same in every category library:

| File | Holds |
|---|---|
| `<chip>.c` | the driver: probe, remove, the `ops.read`, the chip's extras; may split into `<chip>_<part>.c` |
| `<chip>_regs.h` | register map: addresses, bit fields, values, each with its datasheet citation |
| `<chip>_proto.h` | serial protocol: command bytes, frames, checksums, for parts without registers |
| `<chip>_math.h` | pure conversions, compiled and asserted on the host by `test/<chip>_math` tests |
| `<chip>_priv.h` | driver state, only if needed |

A chip has `_regs.h` or `_proto.h`, never both; `_math.h` and `_priv.h` only
when it needs them. Everything in `src/` is private: apps see `include/` only.

## Platform ports

It replaces esp-baro and stm-baro. The chip
drivers in `src/` are the same C on both; only the bus, the clock and the
waits under them differ, in `src/baro_port_esp.c` (ESP-IDF `i2c_master`,
`spi_master`, UART driver, `esp_timer`, FreeRTOS) and `src/baro_port_stm.c`
(HAL). Each port is empty on the other platform, chosen by `ESP_PLATFORM`,
which ESP-IDF defines for every component. Probes and configs take the
platform's own handles, exactly as the two old libraries did, so apps of
either build unchanged: on ESP-IDF an `i2c_master_bus_handle_t` and a clock
the driver sets (`scl_hz` in each config), on the HAL an
`I2C_HandleTypeDef *` with the clock the handle's.

Polls keep each old library's clock: `baro_tick_now()`, `baro_expired()` and
`baro_poll_wait()` count FreeRTOS ticks on ESP-IDF and `HAL_GetTick()`
milliseconds on the HAL, so the BMP388's `cmd_rdy` and the DPS310's ready
waits behave on each platform as they did before.

## Bus abstraction

Under the chips sits a regmap-style ops table, `struct
baro_bus_ops` in `src/baro_priv.h`: each chip holds one `struct baro_bus` and
talks to it only through `transfer(cmd, tx, rx)`, mostly by way of
`baro_reg_read()` and `baro_reg_write()`. The kind — I2C, SPI or UART — is
picked at run time by which attach the probe calls. Every baro chip today is
on I2C; the SPI and UART kinds are there because every category library
offers all three, and `test/{esp,stm}/test_port_*` check them.

On I2C a `transfer()` with nothing to send or read is a bare one-byte write.
I2C and UART also have `receive(rx, n, drop_stale)`: read n bytes with
nothing sent first. On I2C that is a read transaction of its own, with its
own START and STOP — the MS5611 sends a command with a bare `transfer()` and
reads the answer with `baro_bus_receive()`, a STOP between them, on both
platforms; `drop_stale` must be false there. On a UART it waits for a part
that streams, dropping whatever was already waiting first if asked. SPI
leaves it NULL and `baro_bus_receive()` says `-EOPNOTSUPP`.

`baro_spi_attach()` takes an SPI host the app has `spi_bus_initialize()`d, a
CS GPIO, a clock and a mode on ESP-IDF, or an `SPI_HandleTypeDef *` and a
CS port and pin on the HAL. `baro_uart_attach()` takes a `uart_port_t` whose
driver the app has installed, or a `UART_HandleTypeDef *` it has initialised.
On the HAL the SPI and UART code sits under `HAL_SPI_MODULE_ENABLED` and
`HAL_UART_MODULE_ENABLED`, so an app without those modules builds as before.

## Adding a chip

A driver is a probe, a remove and one read function:

1. `include/foo.h`: `int foo_probe(struct baro_dev *baro, <bus>, const struct foo_config *cfg);`
   and `void foo_remove(struct baro_dev *baro);`, with the platform's handle
   types behind `#ifdef ESP_PLATFORM`, as `bmp280.h` does.
2. `src/foo/foo.c`, in a folder of its own with its private headers
   (`foo_regs.h`, `foo_priv.h`, ...); common files and the platform ports
   stay at the top of `src/`. Include siblings and `baro_priv.h` by plain
   name, never with `../`: siblings resolve from the chip's own folder, and
   `baro_priv.h` through `src/`, which the ESP-IDF CMakeLists has as
   `PRIV_INCLUDE_DIRS` and stm-forge's `libs.cmake` puts on this library's
   include path while it compiles every `.c` under `src/`. Wrap the whole
   driver in
   `#if CONFIG_BARO_FOO` (never `#ifdef`); the HAL module check goes in
   `src/baro_priv.h`. Keep a `struct baro_bus` in your state and talk to the
   chip only through `baro_reg_read()`/`baro_reg_write()`,
   `bus.ops->transfer()` or `baro_bus_receive()`. Give it a
   `static const struct baro_ops foo_ops = { .read = foo_read };`, where
   `foo_read()` fills `pressure_pa` and `temp_c`. It must not touch `t_us`;
   `baro_read()` does that.
3. In probe: return `-EBUSY` if `baro->ops` or `baro->priv` is already set,
   before touching anything. Then set `baro->name` so failures log with it,
   refuse a bad config, check presence with `baro_i2c_probe()` on I2C before
   allocating, allocate your state, `baro_i2c_attach()` its bus (with
   `baro_config_scl_hz(cfg)`, which is the config's `scl_hz` on ESP-IDF and
   0 on the HAL), put it into `baro->priv`, set `baro->ops = &foo_ops`, and on
   failure `baro_bus_detach()` and put both back to NULL; remove detaches
   too. Wait with `baro_sleep_ms()`, poll with `baro_tick_now()`,
   `baro_expired()` and `baro_poll_wait()`, and use `baro_err()` for anything
   the user needs to see.
4. Guard your chip-specific calls with `baro->ops == &foo_ops`.
5. Add `CONFIG_BARO_FOO` to the no-chip checks in `CMakeLists.txt`,
   `src/baro_port_stm.c` and the HAL module `#error` in `src/baro_priv.h`, a
   `src/foo/foo.c` line to `CMakeLists.txt`, a `config BARO_FOO` bool to
   `Kconfig`, `# BARO_FOO is not set` to both forges' defconfigs, and
   `test/esp/test_foo.c` and `test/stm/test_foo.c` (and
   `test/test_foo_math.c` for its maths header) targets to `test/Makefile`.

## Tests

```sh
make -C test test
```

Host-only; no toolchain, HAL or ESP-IDF needed. Every test of both old
libraries runs, against its own stubs, in the layout of
[Repository layout](#repository-layout): `test/esp/` through
`src/baro_port_esp.c` and `test/idf_stubs/`, `test/stm/` through
`src/baro_port_stm.c` and `test/hal_stubs/`. The maths tests in `test/` touch
no bus or platform API and run once; three include a public header, so they
build with the HAL stub's flags. Nothing is checked against real silicon.
The table describes the `test/stm/` binaries; each `test/esp/` one checks
the same chip through ESP-IDF's calls instead (`i2c_master_*`,
`vTaskDelay()`, `xTaskGetTickCount()`), with its clock in ticks at 100 Hz
unless a case moves it.

| Binary | Checks |
|---|---|
| `esp/test_port_esp`, `stm/test_port_stm` | the SPI and UART ops of each port, which no chip uses yet. SPI: attach (on ESP-IDF the device added in the mode, clock and CS asked for, a refused add leaving nothing to detach; on the HAL chip select left high); one full-duplex transaction of cmd, tx and zeros with rx taken after them; a write without a read; tx and rx together bounded; failures mapped to errno with rx untouched; on the HAL chip select high again after a failure; no `receive()`. UART: an ESP-IDF port without an installed driver refused; stale input dropped before the send (on the HAL: overrun cleared, data register read empty); command and arguments in one send; no read for a command without an answer, none after a failed send; a short answer as `-ETIMEDOUT`; the read wait in ticks rounded up; driver and HAL failures mapped to errno; too much to send refused before the line is touched; `receive()` sends nothing, drops stale input only when asked, a short count is `-ETIMEDOUT`. I2C: `receive()` refuses `drop_stale` and an empty read before the bus is touched |
| `{esp,stm}/test_core` | `baro_read()` stamps before the read, passes the errno through and leaves the sample untouched on error; the default clock is `HAL_GetTick()` in µs; `baro_altitude_m()` against the ICAO standard atmosphere at 1, 5 and 11 km, zero at the reference, never NaN; log prefix and `baro_log_sample()` format, including 64-bit timestamps without `%lld` |
| `test_bmp280_math` | the integer compensation against the datasheet's double one over a sweep; pinned self-generated vectors; the divide-by-zero and wraparound guards; trim parsing; ctrl_meas presets and measurement times |
| `{esp,stm}/test_bmp280` | a fake BMP280 with a slow NVM copy and forced measurements charged at the datasheet maximum: ID check naming a BME280, bad trim, never-measured and out-of-range reads, waits that never round down, the 7-bit address shifted for the HAL, `-ENXIO`/`-EAGAIN`/`-ETIMEDOUT`/`-EIO` mapping, `-EBUSY`, another driver's `baro_dev` refused, a refused config allocates nothing and a failed probe leaks nothing |
| `test_bmp388_math` | single precision against a double transcription of §9 over a sweep; pinned self-generated vectors; §9.1 scaling exact to the bit; trim parsing against Table 23's addresses and signedness; `OSR` presets; waits against Table 22 |
| `{esp,stm}/test_bmp388` | a fake BMP388 with a slow soft reset, a stale `por_detected` and forced measurements charged at Table 22's maximum: ID check naming a BMP390, `cmd_rdy` late and never (and the full 10 ms waited), a reset that never finishes, `ODR` written and a `conf_err` from a short one refused, bad trim, sticky and clear-on-read `ERR_REG` bits, either `drdy` flag clear, out-of-range reads, waits that never round down, errno mapping, `-EBUSY`, another driver's `baro_dev` refused, a refused config allocates nothing |
| `test_dps310_math` | sign extension, coefficient unpacking against hand-worked patterns, the non-monotonic scale table, the busy-time budget, compensation expanded by hand |
| `{esp,stm}/test_dps310` | a fake DPS310: ready sequencing including never-ready, the `HAL_GetTick()`-timed ready wait straddling the tick counter's wrap (ready at 293 ms passes, 294 ms times out, the last poll past 290 ms; on ESP-IDF tick-counted at 100 Hz, ready at 310 ms passes, 320 ms times out), the 990 ms budget either side, the erratum fix only when needed, `COEF_SRCE` into `TMP_CFG`, shift bits against literals, errno mapping, `-EBUSY`, foreign devices refused by every extra |
| `test_ms5611_math` | the datasheet's worked example, both cold branches worked by hand, the AN520 CRC vector, OSR command and wait tables |
| `{esp,stm}/test_ms5611` | a fake MS5611 with no data-ready flag, serving commands by a one-byte write and answers by a separate read (`Master_Transmit` and `Master_Receive`; `i2c_master_transmit()` and `i2c_master_receive()`, a repeated start aborting): D2 before D1, waits at every OSR that never round down, ADC 0 is an error, every single-bit PROM flip fails the CRC, errno mapping, `-EBUSY`, a refused config allocates nothing |

All build under `-std=gnu11 -Wall -Wextra -Werror -pedantic`, with
`-Wl,--wrap` (GNU ld) to count allocations and capture `printf()`.
