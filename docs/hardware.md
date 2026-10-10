# Hardware wiring

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Pins

On ESP-IDF any GPIO works; every example in `examples/esp/` puts SDA on
GPIO 21 and SCL on GPIO 22, set at the top of `main.c`. On STM32, pins are
the ones the examples in `examples/stm/` use, on an F4: I2C1 on PB8 (SCL)
and PB9 (SDA), AF4. Any pair your chip routes to an I2C peripheral works;
check the alternate-function table in its datasheet. The bus init — pins,
clocks, speed — is the app's; each example shows one. None of the four chips
needs an interrupt or data-ready line.

In the tables below, the SDA and SCL rows give the STM32 pin; on ESP-IDF they
are GPIO 21 and GPIO 22, or any GPIO you choose.

**Two of them share an address.** The DPS310 and the MS5611 both default to
**0x77**, and the BMP280 and BMP388 both default to 0x76. To put a DPS310 and
an MS5611 on one bus, move one of them: DPS310 SDO low gives 0x76, MS5611 CSB
high gives 0x76. All four only answer at 0x76/0x77, so one bus holds at most
two of them; a third needs a second I2C bus.

### BMP280 — `examples/*/bmp280`

| pin | goes to | note |
|---|---|---|
| VDD | 3.3 V | the bare part runs on 1.7–3.6 V. Most breakouts carry a regulator and take 5 V, so check yours |
| VDDIO | 3.3 V | the digital supply; breakouts usually tie it to VDD |
| GND | GND | shared ground, always |
| SDA | PB9 | I2C1_SDA, AF4, open drain. `SDI` on the bare chip |
| SCL | PB8 | I2C1_SCL, AF4, open drain. `SCK` on the bare chip |
| SDO | GND | the address strap: low is 0x76, which the driver assumes; high is 0x77 (`.address = 0x77`). Do not leave it floating |
| CSB | 3.3 V | has to be high for I2C. Pulled low the part talks SPI, which this driver does not do |

### BMP388 — `examples/*/bmp388`

| pin | goes to | note |
|---|---|---|
| VDD | 3.3 V | the bare part runs on 1.65–3.6 V (Table 1). Breakouts with a regulator take 5 V, so check yours |
| VDDIO | 3.3 V | the interface supply, 1.2–3.6 V; breakouts usually tie it to VDD |
| GND | GND | `VSS`, three pins on the bare chip |
| SDA | PB9 | I2C1_SDA, AF4, open drain. `SDI` on the bare chip |
| SCL | PB8 | I2C1_SCL, AF4, open drain. `SCK` on the bare chip |
| SDO | GND | the address strap: low is 0x76, which the driver assumes; high is 0x77 (`.address = 0x77`). Must not float (§5.2) |
| CSB | 3.3 V | has to be high for I2C (§5.1). Pulled low even once, the part locks into SPI until the next power cycle |
| INT | — | not used; leave open |

### DPS310 — `examples/*/dps310`

| pin | goes to | note |
|---|---|---|
| VDD | 3.3 V | the analog supply. A 1.7–3.6 V part, so nothing near 5 V |
| VDDIO | 3.3 V | the digital supply; breakouts usually tie it to VDD |
| GND | GND | shared ground, always |
| SDA | PB9 | I2C1_SDA, AF4, open drain. `SDI` on the bare chip |
| SCL | PB8 | I2C1_SCL, AF4, open drain. `SCK` on the bare chip |
| SDO | 3.3 V | the address strap: high is 0x77, which the driver assumes; low is 0x76 (`.address = 0x76`) |
| CSB | 3.3 V | has to be high for I2C. Pulled low the part talks SPI, which this driver does not |

The Adafruit breakout pictured in the [chip reference](chips.md#dps310)
labels the bus pins SCK and SDI. `dps310_probe()` leaves the interrupt bits in
`CFG_REG` clear, and `dps310_data_ready()` polls `MEAS_CFG`.

### MS5611 — `examples/*/ms5611`

| pin | goes to | note |
|---|---|---|
| VDD | 3.3 V | the bare part runs on 1.8–3.6 V. Some breakouts carry a regulator and take 5 V, so check yours |
| GND | GND | shared ground, always |
| SDA | PB9 | I2C1_SDA, AF4, open drain. `SDI/SDA` on the bare chip |
| SCLK | PB8 | I2C1_SCL, AF4, open drain. `SCL` on most breakouts |
| PS | 3.3 V | protocol select: high is I2C. Low is SPI, which this driver does not do |
| CSB | GND | the address strap: low is 0x77, which the driver assumes; high is 0x76 (`.address = 0x76`) |
| SDO | — | SPI data out. Not used by this driver; leave open |

The BMP388's pin names are from its datasheet, which is kept locally at
`docs/BMP388.pdf` (gitignored, so not in the repository). For the other three
the pin names are the chips' usual ones, not checked against a datasheet.
Breakouts label them in different ways, and many tie CSB/PS and the address
strap for you. The examples only turn on the microcontroller's weak internal
pull-ups; most breakouts carry their own, and a bare chip needs external ones
on SDA and SCL.
