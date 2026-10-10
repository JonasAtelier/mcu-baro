/*
 * DPS310 register addresses and bit fields.
 *
 * Everything but the CFG_REG bits appears in two independent drivers that
 * agree: the Linux kernel's drivers/iio/pressure/dps310.c and Zephyr's
 * drivers/sensor/infineon/dps310/.
 * CFG_REG follows Infineon's arduino-xensiv-dps3xx (src/util/dps3xx_config.h:
 * PRS_SE 0x04, TEMP_SE 0x08, FIFO_EN 0x02, INT_SEL 0x70); Adafruit_DPS310
 * agrees. Linux's PRS_SHIFT_EN BIT(4) and FIFO_EN BIT(5) fall inside INT_SEL
 * and are wrong.
 */
#ifndef DPS310_REGS_H
#define DPS310_REGS_H

/* Results are 24-bit, two's complement, MOST significant byte first. */
#define DPS310_PRS_B2		0x00	/* pressure, 3 bytes */
#define DPS310_TMP_B2		0x03	/* temperature, 3 bytes */
#define DPS310_RESULT_LEN	3

#define DPS310_PRS_CFG		0x06
#define DPS310_TMP_CFG		0x07
#define DPS310_MEAS_CFG		0x08
#define DPS310_CFG_REG		0x09
#define DPS310_INT_STS		0x0A
#define DPS310_FIFO_STS		0x0B
#define DPS310_RESET		0x0C
#define DPS310_PRODUCT_ID	0x0D
#define DPS310_COEF_BASE	0x10	/* 18 bytes of calibration */
#define DPS310_COEF_LEN		18
#define DPS310_COEF_SRCE	0x28

#define DPS310_PRODUCT_ID_VALUE	0x10

#define DPS310_ADDR_LOW		0x76	/* SDO low */
#define DPS310_ADDR_HIGH	0x77	/* SDO high, the usual breakout default */

/* ---- PRS_CFG (0x06) / TMP_CFG (0x07) ---- */
#define DPS310_RATE_SHIFT	4	/* bits 6:4, measurements per second */
#define DPS310_PRC_SHIFT	0	/* bits 3:0, oversampling */
#define DPS310_TMP_EXT		0x80	/* which temperature sensor; COEF_SRCE */

/* ---- MEAS_CFG (0x08) ---- */
#define DPS310_MEAS_PRS_EN	0x01
#define DPS310_MEAS_TMP_EN	0x02
#define DPS310_MEAS_BACKGROUND	0x04
#define DPS310_MEAS_CONTINUOUS	(DPS310_MEAS_PRS_EN | DPS310_MEAS_TMP_EN | \
				 DPS310_MEAS_BACKGROUND)
#define DPS310_PRS_RDY		0x10
#define DPS310_TMP_RDY		0x20
#define DPS310_SENSOR_RDY	0x40
#define DPS310_COEF_RDY		0x80

/* ---- CFG_REG (0x09) ---- */
#define DPS310_TMP_SHIFT_EN	0x08	/* required when temp oversampling > 8 */
#define DPS310_PRS_SHIFT_EN	0x04	/* required when pressure oversampling > 8 */
#define DPS310_FIFO_EN		0x02

/*
 * ---- COEF_SRCE (0x28) ----
 * Bit 7 says which of the two internal temperature sensors the factory
 * calibration was taken against. TMP_CFG must be told to use the same one or
 * every temperature, and through it every pressure, is wrong.
 */
#define DPS310_COEF_SRCE_MASK	0x80

/* ---- RESET (0x0C) ---- */
#define DPS310_RESET_MAGIC	0x09

/*
 * Undocumented but manufacturer-recommended erratum fix. Some parts read
 * around 60 C when the real temperature is around 20 C. Read 0x32: if bit 1
 * is clear, play this sequence. Both reference drivers carry it.
 */
#define DPS310_ERRATA_PROBE	0x32
#define DPS310_ERRATA_OK_BIT	0x02

#endif /* DPS310_REGS_H */
