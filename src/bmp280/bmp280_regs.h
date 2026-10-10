/*
 * BMP280 register map.
 *
 * A real register file: write the register address, then read or write bytes
 * from it, with auto-increment on reads.
 *
 * Every address, bit position and timing below is from memory of the BMP280
 * datasheet. There is no PDF in docs/ to check them against.
 */
#ifndef BMP280_REGS_H
#define BMP280_REGS_H

/* The address is 111011x, where x is the SDO pin. */
#define BMP280_ADDR_SDO_LOW	0x76	/* the default */
#define BMP280_ADDR_SDO_HIGH	0x77

/* Registers. */
#define BMP280_REG_CALIB	0x88	/* 0x88..0x9F, 24 bytes, little-endian */
#define BMP280_CALIB_LEN	24
#define BMP280_REG_ID		0xD0
#define BMP280_REG_RESET	0xE0
#define BMP280_REG_STATUS	0xF3
#define BMP280_REG_CTRL_MEAS	0xF4
#define BMP280_REG_CONFIG	0xF5
#define BMP280_REG_PRESS_MSB	0xF7	/* 0xF7..0xFC, press[3] then temp[3] */
#define BMP280_RESULT_LEN	6

/*
 * id: production parts read 0x58; pre-production samples are reported to read
 * 0x56 or 0x57, which this driver does not accept.
 *
 * A BME280 reads 0x60. It carries the same dig_T1..T3 and dig_P1..P9 at
 * 0x88..0x9F and the same pressure and temperature compensation -- it only
 * adds humidity on top: dig_H1 at 0xA1, dig_H2..H6 at 0xE1..0xE7, ctrl_hum at
 * 0xF2 and the result at 0xFD..0xFE. So it would read correct pressure and
 * temperature here. It is rejected because this driver only claims the
 * BMP280, and nothing about the BME280 has been checked.
 */
#define BMP280_CHIP_ID		0x58
#define BME280_CHIP_ID		0x60

/* reset: the only value that does anything; anything else is ignored. */
#define BMP280_RESET_WORD	0xB6
#define BMP280_RESET_WAIT_MS	2	/* start-up time, NVM copied to trim */

/* status: bit 3 measuring, bit 0 im_update (NVM being copied). */
#define BMP280_STATUS_MEASURING	0x08
#define BMP280_STATUS_IM_UPDATE	0x01

/* ctrl_meas: osrs_t[7:5] | osrs_p[4:2] | mode[1:0]. */
#define BMP280_MODE_SLEEP	0x00
#define BMP280_MODE_FORCED	0x01	/* 0x02 is forced as well */
#define BMP280_MODE_NORMAL	0x03

/*
 * config: t_sb[7:5] | filter[4:2] | spi3w_en[0]. A reset leaves it at 0x00 --
 * 0.5 ms standby, IIR off, 4-wire SPI -- which is what this driver wants, so it
 * never writes the register. The filter needs consecutive samples anyway, and
 * forced mode takes one at a time.
 */

/*
 * Both result registers reset to 0x80000 (0x80 0x00 0x00). One of them
 * landing there is an ordinary reading; both at once is a chip that has never
 * measured.
 */
#define BMP280_RAW_RESET	0x80000

#endif /* BMP280_REGS_H */
