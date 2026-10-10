/*
 * ms5611 -- TE Connectivity MS5611-01BA03 barometer, as a baro chip
 * driver.
 *
 * A piezo-resistive pressure die behind a 24-bit ADC, with six factory
 * calibration words in PROM. At the highest oversampling it resolves about
 * 10 cm of altitude, which is why it ends up on flight controllers.
 * Temperature comes with it, because the pressure compensation needs it.
 *
 * I2C only. The part also does SPI (PS pin low); that is not implemented here.
 *
 * Samples come out of baro_read(). Constants cite the datasheet by section
 * heading; src/ms5611/ms5611_proto.h says which sheet checked what. Nothing
 * here has been run against a physical chip.
 *
 * Every read is a command, a STOP, then a read of the answer, as the
 * MS5607-02BA03 sheet's I2C INTERFACE figures 11-15 draw it, on both
 * platforms. ESP-IDF used to send a repeated start instead, from memory; that
 * form was dropped for the cited one.
 */
#ifndef MS5611_H
#define MS5611_H

#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/i2c_master.h"
#else
#include CMSIS_device_header
#endif

#include "baro.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Oversampling, for pressure and temperature alike. 0 is the best and the
 * slowest. The times are the datasheet maximum per conversion; a read does
 * two.
 */
enum ms5611_osr {
	MS5611_OSR_4096	= 0,	/* 9.04 ms */
	MS5611_OSR_2048	= 1,	/* 4.54 ms */
	MS5611_OSR_1024	= 2,	/* 2.28 ms */
	MS5611_OSR_512	= 3,	/* 1.17 ms */
	MS5611_OSR_256	= 4,	/* 0.60 ms */
};

/* NULL cfg = all defaults. */
struct ms5611_config {
#ifdef ESP_PLATFORM
	uint8_t address;		/* 0 selects 0x77 (CSB low); 0x76 when CSB high */
	uint32_t scl_hz;		/* 0 selects 400 kHz */
#else
	uint8_t address;		/* 7-bit; 0 selects 0x77, CSB low */
#endif
	enum ms5611_osr osr;		/* 0 selects 4096 */
};

/*
 * bus is, on ESP-IDF, an i2c_master bus the app has already created; the
 * driver adds its device at cfg->scl_hz. On the STM32 HAL it is an I2C handle
 * the app has already set up with HAL_I2C_Init(), at 400 kHz or less; the bus
 * speed is the handle's, not this driver's.
 *
 * Reset the chip, read the six calibration words from PROM, and check them
 * against the 4-bit CRC stored beside them. There is no ID register, so that
 * CRC is the only evidence the part at this address is an MS5611 at all.
 * Fills baro->name, ops and priv.
 *
 * baro_read() then converts temperature (D2), then pressure (D1), and
 * compensates both, first and second order. The chip has no data-ready
 * flag, so both conversions are waited out. On ESP-IDF that blocks the
 * calling task: 20-40 ms at the default 100 Hz FreeRTOS tick whatever the
 * osr, because the wait is rounded up to whole ticks; one task per baro_dev.
 * On the STM32 HAL it busy-waits in HAL_Delay(): 20-22 ms at OSR 4096 and the
 * HAL's default 1 kHz tick. The HAL has no microsecond delay, so every wait
 * is whole milliseconds, rounded up: at OSR 256 the 0.60 ms conversion costs
 * 1-2 ms, and a read 2-4 ms; one caller per baro_dev. It returns -EIO if
 * the ADC hands back 0, which is the chip's answer to a read before the
 * conversion finished.
 *
 * -EBUSY if baro is already bound (see struct baro_dev); nothing is touched.
 * -ENXIO if nothing ACKs at the address, decided before anything is allocated.
 * -ENODEV if the PROM fails its CRC, or reads as all zeros.
 * -EINVAL for a NULL bus, an address other than 0x76/0x77, or an osr outside
 * the enum. Bus failures come back as the errno of the failed transfer.
 */
int ms5611_probe(struct baro_dev *baro,
#ifdef ESP_PLATFORM
		 i2c_master_bus_handle_t bus,
#else
		 I2C_HandleTypeDef *bus,
#endif
		 const struct ms5611_config *cfg);

/* Releases the device. Safe on an unprobed baro_dev. */
void ms5611_remove(struct baro_dev *baro);

#ifdef __cplusplus
}
#endif

#endif /* MS5611_H */
