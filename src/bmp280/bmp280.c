#include <errno.h>
#include <stdlib.h>

#include "baro_priv.h"

#if CONFIG_BARO_BMP280

#include "baro_log.h"
#include "bmp280.h"
#include "bmp280_math.h"
#include "bmp280_priv.h"
#include "bmp280_regs.h"

#define I2C_TIMEOUT_MS		100

static const struct baro_ops bmp280_ops;

/* Our state, or NULL if this baro_dev was not probed by this driver. */
static struct bmp280 *to_bmp280(struct baro_dev *baro)
{
	if (!baro || baro->ops != &bmp280_ops)
		return NULL;
	return baro->priv;
}

/* ---------------------------------------------------------------- I2C ---- */

static int bmp280_reg_write(struct bmp280 *d, uint8_t reg, uint8_t val)
{
	return baro_reg_write(&d->bus, reg, val);
}

/*
 * Reads auto-increment from reg, so the trim block and the six result bytes
 * each come back in one transfer: register address, repeated start, burst.
 */
static int bmp280_reg_read(struct bmp280 *d, uint8_t reg, uint8_t *buf,
			   size_t len)
{
	return baro_reg_read(&d->bus, reg, buf, len);
}

/* -------------------------------------------------------------- read ----- */

/* baro_ops.read: one forced measurement, waited out and compensated. */
static int bmp280_read(struct baro_dev *baro, struct baro_sample *s)
{
	struct bmp280 *d = baro->priv;
	uint8_t b[BMP280_RESULT_LEN];
	int32_t adc_p, adc_t, t_fine, temp;
	uint32_t press;
	int ret;

	/*
	 * Forced mode: one measurement, then the chip drops back to sleep by
	 * itself. Writing ctrl_meas is what starts it.
	 */
	ret = bmp280_reg_write(d, BMP280_REG_CTRL_MEAS,
			       bmp280_ctrl_meas(d->osr, BMP280_MODE_FORCED));
	if (ret)
		return ret;
	baro_sleep_ms((uint32_t)bmp280_meas_ms(d->osr));

	/*
	 * One burst, so pressure and temperature cannot come from two different
	 * measurements.
	 */
	ret = bmp280_reg_read(d, BMP280_REG_PRESS_MSB, b, sizeof(b));
	if (ret)
		return ret;
	adc_p = bmp280_raw20(&b[0]);
	adc_t = bmp280_raw20(&b[3]);

	/*
	 * Either one alone at the reset value is an ordinary reading; both at
	 * once is a chip that has not measured -- reset, asleep, or never
	 * triggered.
	 */
	if (adc_p == BMP280_RAW_RESET && adc_t == BMP280_RAW_RESET) {
		baro_err(baro, "raw registers still at their reset value: "
			 "no measurement");
		return -EIO;
	}

	temp = bmp280_compensate_T_int32(&d->calib, adc_t, &t_fine);
	press = bmp280_compensate_P_int64(&d->calib, adc_p, t_fine);
	if (press == 0) {
		/*
		 * The compensation's guard fired: either a trim set that would
		 * divide by zero, which probe already refuses, or a raw value so
		 * far outside the part's range that the pressure came out
		 * negative.
		 */
		baro_err(baro, "pressure compensates to 0 from raw p=%ld t=%ld",
			 (long)adc_p, (long)adc_t);
		return -ERANGE;
	}

	s->temp_c = (float)temp / 100.0f;
	s->pressure_pa = (float)press / 256.0f;		/* Q24.8 */
	return 0;
}

static const struct baro_ops bmp280_ops = {
	.read = bmp280_read,
};

/* ------------------------------------------------------- probe/remove ---- */

int bmp280_probe(struct baro_dev *baro, baro_i2c_bus bus,
		 const struct bmp280_config *cfg)
{
	static const struct bmp280_config defaults;	/* all zero: defaults */
	uint8_t raw[BMP280_CALIB_LEN], id = 0;
	struct bmp280 *d;
	uint8_t addr;
	int ret;

	if (!baro || !bus)
		return -EINVAL;
	/* Bound already: a second probe would leak the first binding. */
	if (baro->ops || baro->priv)
		return -EBUSY;
	if (!cfg)
		cfg = &defaults;

	baro->name = "bmp280";

	addr = cfg->address ? cfg->address : BMP280_ADDR_SDO_LOW;
	if (addr != BMP280_ADDR_SDO_LOW && addr != BMP280_ADDR_SDO_HIGH) {
		baro_err(baro, "address must be 0x76 or 0x77, not 0x%02x", addr);
		return -EINVAL;
	}
	if (bmp280_meas_ms(cfg->osr) == 0)
		return -EINVAL;

	/* Anything there? baro_i2c_probe() says why not the first read. */
	ret = baro_i2c_probe(bus, addr, I2C_TIMEOUT_MS);
	if (ret) {
		baro_err(baro, "nothing answered at 0x%02x (%d)", addr, ret);
		return ret;
	}

	d = calloc(1, sizeof(*d));
	if (!d)
		return -ENOMEM;
	ret = baro_i2c_attach(&d->bus, bus, addr, baro_config_scl_hz(cfg),
			      I2C_TIMEOUT_MS);
	if (ret) {
		baro_err(baro, "cannot add 0x%02x to the bus (%d)", addr, ret);
		free(d);
		return ret;
	}
	d->osr = cfg->osr;

	baro->ops = &bmp280_ops;
	baro->priv = d;

	ret = bmp280_reg_read(d, BMP280_REG_ID, &id, 1);
	if (ret) {
		baro_err(baro, "first transfer to 0x%02x failed (%d)", addr,
			 ret);
		goto fail;
	}
	if (id != BMP280_CHIP_ID) {
		/*
		 * A BME280 (0x60) has the same trim words and the same pressure
		 * and temperature maths, and would read correctly. It is refused
		 * because this driver only claims the BMP280, not because it
		 * would be wrong.
		 */
		baro_err(baro, "id at 0x%02x is 0x%02x, not 0x%02x%s", addr, id,
			 BMP280_CHIP_ID,
			 id == BME280_CHIP_ID ? " -- that is a BME280" : "");
		ret = -ENODEV;
		goto fail;
	}

	/*
	 * Soft reset, then the start-up time. The trim image is copied out of
	 * NVM during it -- status.im_update is set while that runs -- so a trim
	 * read that beats it cannot be trusted.
	 */
	ret = bmp280_reg_write(d, BMP280_REG_RESET, BMP280_RESET_WORD);
	if (ret)
		goto fail;
	baro_sleep_ms(BMP280_RESET_WAIT_MS);

	ret = bmp280_reg_read(d, BMP280_REG_CALIB, raw, sizeof(raw));
	if (ret)
		goto fail;
	bmp280_parse_calib(raw, &d->calib);
	if (!bmp280_calib_ok(&d->calib)) {
		baro_err(baro, "trim at 0x%02x reads dig_T1=%u dig_P1=%u", addr,
			 d->calib.t1, d->calib.p1);
		ret = -EIO;
		goto fail;
	}

	/*
	 * config is left alone: the reset above is a full power-on reset, so
	 * standby, IIR filter and spi3w_en already read the 0x00 this driver
	 * wants.
	 */

	baro_info(baro, "at 0x%02x, ctrl_meas 0x%02x, %d ms per read", addr,
		  bmp280_ctrl_meas(d->osr, BMP280_MODE_FORCED),
		  bmp280_meas_ms(d->osr));
	return 0;

fail:
	baro->ops = NULL;
	baro->priv = NULL;
	baro_bus_detach(&d->bus);
	free(d);
	return ret;
}

void bmp280_remove(struct baro_dev *baro)
{
	struct bmp280 *d = to_bmp280(baro);

	if (!d)
		return;

	baro_bus_detach(&d->bus);
	free(d);
	baro->ops = NULL;
	baro->priv = NULL;
}

#endif /* CONFIG_BARO_BMP280 */
