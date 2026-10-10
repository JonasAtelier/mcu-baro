# API usage

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Device lifecycle

Initialise the bus, probe once, then read. The
[README quick start](../README.md#quick-start) shows a complete STM32
helper with cleanup. The [ESP-IDF example](../examples/esp/bmp280/main/main.c)
shows the corresponding bus setup.

One `baro_log_sample()` line looks like this (values illustrative):

```
baro: bmp280: t 1234000 us   100653.2 Pa   25.08 C
```

On ESP-IDF the same line carries the log's own prefix, as
`I (1234) baro: bmp280: ...`.

The other chips are the same after the probe:
`bmp388_probe(&baro, bus, NULL)`, `dps310_probe(&baro, bus, &cfg)` or
`ms5611_probe(&baro, bus, NULL)`. Tear down with the matching
`*_remove(&baro)`; the bus is left alone.

**Bus handles on ESP-IDF.** Every chip takes an `i2c_master_bus_handle_t`
the app has created, and the driver adds its own device at the config's
`scl_hz` (0 for 400 kHz); `*_remove()` takes it off again.

**Bus handles on the HAL.** Every chip takes an `I2C_HandleTypeDef *` the app
has `HAL_I2C_Init()`ed, plus a **7-bit** address in its config (`0x76`, not
the HAL's shifted `0xEC`; the driver shifts it). Bus speed is the handle's
business: 400 kHz or less suits all four.

**A `struct baro_dev` must start zeroed** — static storage or `= { 0 }`.
Probe on one that is already bound (non-NULL `ops` or `priv`) returns
`-EBUSY` and touches nothing, rather than leak the first binding. `*_remove()`
and a failed probe zero it again, so it can be probed afresh.

**Timestamps.** `s.t_us` comes from `baro_time_us()`, taken just before the
bus transfer. On ESP-IDF that is `esp_timer_get_time()`. On the HAL the
library's default is weak and only `HAL_GetTick() * 1000`:
whole milliseconds, wrapping after 49.7 days. **Define your own
`int64_t baro_time_us(void)` for real microseconds** and it replaces the
default at link time — the `DWT->CYCCNT` cycle counter (enable
`CoreDebug->DEMCR` TRCENA and `DWT->CTRL` CYCCNTENA, then divide by the core
clock in MHz and extend the 32-bit count to 64; an F7 may also need
`DWT->LAR` unlocked first), or a free-running timer at 1 MHz.

**Delays.** On ESP-IDF, waits use `vTaskDelay()`, rounded up to whole ticks
and plus one, so never shorter than asked; at the default 100 Hz tick even a
2 ms wait takes 10–20 ms, which is why a lower oversampling only buys speed
with `CONFIG_FREERTOS_HZ=1000`. Polls (the BMP388's `cmd_rdy`, the DPS310's
ready bits) count ticks, at least one apart. On the HAL they wait with
`HAL_Delay()`, which already adds one tick to what it is asked, so no wait is
ever shorter than the chip needs; the HAL has no microsecond delay, so the
MS5611's sub-millisecond conversions round up to 1 ms. Those waits busy-wait
on SysTick: probe and read from thread context, never from an interrupt at or
above SysTick's priority. A probe takes up to about 300 ms (the DPS310's reset
and ready wait, when the part is slow to come up); a read on a forced-mode
chip blocks for its whole measurement. If your app replaces the weak
`HAL_Delay()` (an RTOS often does), the "never short" guarantee is that
replacement's. The per-chip waits are in the [chip reference](chips.md).

**Logging.** On ESP-IDF everything goes under the log tag `baro`, prefixed
with the chip name, so one `esp_log_level_set("baro", ESP_LOG_WARN)` quiets
every driver, and `baro_dbg()` is `ESP_LOGD`, also limited by
`CONFIG_LOG_MAXIMUM_LEVEL`. On the HAL every line goes through `printf()` as
`baro: <chip>: ...`. Your own code can use the same `baro_err()`,
`baro_warn()`, `baro_info()` and `baro_dbg()` from `baro_log.h`. On the HAL
`baro_dbg()` is compiled out unless `CONFIG_BARO_DEBUG` is 1 (stm-forge:
`BARO_DEBUG = y`). There are no levels beyond that: printf() is the one sink.

**Chip extras** take the same `struct baro_dev *` and return `-EINVAL` if
another driver probed it. Only the DPS310 has any: raw reads, data ready, its
factory coefficients and raw registers (`dps310.h`). The BMP280, BMP388 and
MS5611 drivers are probe, remove and `baro_read()`.

### Altitude is relative, or it is weather

```c
float h = baro_altitude_m(s.pressure_pa, reference_pa);
```

The international barometric formula, `44330 · (1 − (p / p₀)^(1/5.255))`,
metres above the level where the pressure is `p₀`. Pass `BARO_SEA_LEVEL_PA`
(101325) and you get *pressure altitude*, which drifts by tens of metres as
the weather moves — a passing depression moves it by 100 m without you leaving
the room. That is a barometer doing its job, not a fault.

For height above a fixed point, take a reading there and use it as the
reference. Then the sensor is doing what it is good at, which is measuring
**change**. Every example does exactly this, zeroing on the first good sample.
A pressure or reference that is not positive gives 0, never NaN. It needs
libm; see [installation](installation.md).

## Errors

| errno | from | means |
|---|---|---|
| `-EINVAL` | any | NULL argument; address not 0x76/0x77; an enum out of range; a DPS310 rate/oversampling pair that needs ≥ 990 ms of every second; a `baro_dev` another driver probed |
| `-EBUSY` | probe | that `baro_dev` is already bound. Nothing was touched |
| `-ENXIO` | probe | nothing ACKed at that address, checked with `i2c_master_probe()` or `HAL_I2C_IsDeviceReady()` before anything is allocated or written |
| `-ENODEV` | probe | something answered but it is not the chip: BMP280 ID not 0x58 (0x60 is a BME280, and the log says so), BMP388 ID not 0x50 (0x60 is a BMP390, and the log says so), DPS310 product ID not 0x10, MS5611 PROM fails its CRC or reads all zeros |
| `-EAGAIN` | any | HAL only: the HAL answered `HAL_BUSY` — the bus or the handle is in use, or the handle was never `HAL_I2C_Init()`ed. On the F4, also a bus with no pull-ups or SDA stuck low during the probe's presence check — see below |
| `-ETIMEDOUT` | any | a bus transfer timed out (`ESP_ERR_TIMEOUT`, `HAL_TIMEOUT`), the DPS310 never reported sensor and coefficients ready, or the BMP388 never reported `cmd_rdy` |
| `-EIO` | probe | BMP280 `dig_T1` or `dig_P1` read as 0: a dead bus, or trim read too early. BMP388 `NVM_PAR_T1` or `NVM_PAR_P5` read as 0, or the soft reset never reported complete |
| `-EIO` | read | BMP280 both raw values still 0x80000 (nothing measured); BMP388 `ERR_REG` non-zero, or a data-ready flag clear; MS5611 ADC returned 0 (read before the conversion finished); any other bus failure, a NACK in the middle of a transfer included |
| `-ERANGE` | read | BMP280 or BMP388 pressure compensates to 0 or below (BMP388: or to NaN/inf): a raw value far outside the part's range |
| `-ENOMEM` | probe | out of heap |

On ESP-IDF bus failures map from `esp_err_t`: `ESP_ERR_TIMEOUT` is
`-ETIMEDOUT`, `ESP_ERR_INVALID_ARG` `-EINVAL`, `ESP_ERR_INVALID_SIZE`
`-EMSGSIZE`, `ESP_ERR_NO_MEM` `-ENOMEM` and the rest, a mid-transfer NACK
included, `-EIO`. On the HAL, a chip that NACKs its address is `HAL_ERROR`
from `HAL_I2C_IsDeviceReady()`, `-ENXIO`, on the F4 and the F7. On the F4, a
bus with no pull-ups or SDA stuck low sets the peripheral's BUSY flag, so the
probe gives up after 25 ms with `-EAGAIN`; `-ETIMEDOUT` from the presence
check means a START never completed. Check the wiring for either. `-EAGAIN` is
never `-EBUSY`, which a probe keeps for "already bound".

`strerror(-ret)` names any of them. **Do not use a sample when `baro_read()`
returns an error**: it writes `*s` only on success, so the old values are still
there.
