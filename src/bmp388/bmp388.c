#include <errno.h>
#include <math.h>
#include <stdlib.h>

#include "baro_priv.h"

#if CONFIG_BARO_BMP388

#include "baro_log.h"
#include "bmp388.h"
#include "bmp388_math.h"
#include "bmp388_priv.h"
#include "bmp388_regs.h"

#define I2C_TIMEOUT_MS		100
/*
 * How long probe waits for STATUS.cmd_rdy before the soft reset. The
 * datasheet gives no figure; nothing has been sent yet, so on a healthy part
 * the first read already says ready, and this only bounds a broken one.
 */
#define CMD_RDY_TIMEOUT_MS	10

static const struct baro_ops bmp388_ops;

/* Our state, or NULL if this baro_dev was not probed by this driver. */
static struct bmp388 *to_bmp388(struct baro_dev *baro)
{
	if (!baro || baro->ops != &bmp388_ops)
		return NULL;
	return baro->priv;
}

/* ---------------------------------------------------------------- I2C ---- */

static int bmp388_reg_write(struct bmp388 *d, uint8_t reg, uint8_t val)
{
	return baro_reg_write(&d->bus, reg, val);
}

/*
 * Reads auto-increment from reg (§5.2.2), so the trim block and the
 * flags-plus-results burst each come back in one transfer: register address,
 * repeated start, burst.
 */
static int bmp388_reg_read(struct bmp388 *d, uint8_t reg, uint8_t *buf,
			   size_t len)
{
	return baro_reg_read(&d->bus, reg, buf, len);
}

/*
 * Polls STATUS until cmd_rdy (Table 27) or CMD_RDY_TIMEOUT_MS has passed by
 * baro_expired(), baro_poll_wait() between polls. The last read is at or
 * after the deadline.
 */
static int bmp388_wait_cmd_rdy(struct bmp388 *d)
{
	const uint32_t start = baro_tick_now();
	int ret;

	for (;;) {
		uint8_t status = 0;

		ret = bmp388_reg_read(d, BMP388_REG_STATUS, &status, 1);
		if (ret)
			return ret;
		if (status & BMP388_STATUS_CMD_RDY)
			return 0;
		if (baro_expired(start, CMD_RDY_TIMEOUT_MS))
			return -ETIMEDOUT;
		baro_poll_wait(1);
	}
}

/* -------------------------------------------------------------- read ----- */

/* baro_ops.read: one forced measurement, waited out and compensated. */
static int bmp388_read(struct baro_dev *baro, struct baro_sample *s)
{
	struct bmp388 *d = baro->priv;
	uint8_t b[BMP388_BURST_LEN];
	uint32_t adc_p, adc_t;
	float temp, press;
	int ret;

	/*
	 * Forced mode (§3.3.2): one measurement, then the chip drops back to
	 * sleep by itself. Writing PWR_CTRL is what starts it, and it has to be
	 * written again for the next one.
	 */
	ret = bmp388_reg_write(d, BMP388_REG_PWR_CTRL, BMP388_PWR_FORCED);
	if (ret)
		return ret;
	baro_sleep_ms((uint32_t)bmp388_meas_ms(d->osr));

	/*
	 * One burst from ERR_REG to the last temperature byte, so the flags and
	 * both results describe the same measurement.
	 */
	ret = bmp388_reg_read(d, BMP388_REG_ERR, b, sizeof(b));
	if (ret)
		return ret;

	if (b[0] & BMP388_ERR_MASK) {
		baro_err(baro, "ERR_REG 0x%02x (fatal/cmd/conf = bits 0/1/2)",
			 b[0]);
		return -EIO;
	}
	/*
	 * Both data-ready flags or no sample: a clear one means the registers
	 * still hold the previous measurement, or the reset value.
	 */
	if ((b[1] & (BMP388_STATUS_DRDY_PRESS | BMP388_STATUS_DRDY_TEMP)) !=
	    (BMP388_STATUS_DRDY_PRESS | BMP388_STATUS_DRDY_TEMP)) {
		baro_err(baro, "STATUS 0x%02x: no new measurement", b[1]);
		return -EIO;
	}

	adc_p = bmp388_raw24(&b[2]);
	adc_t = bmp388_raw24(&b[5]);
	temp = bmp388_compensate_temperature(&d->par, adc_t);
	press = bmp388_compensate_pressure(&d->par, adc_p, temp);
	if (!(press > 0.0f) || !isfinite(press) || !isfinite(temp)) {
		/*
		 * A raw value far outside the part's 300-1250 hPa range, or a
		 * trim image that is not what it should be.
		 */
		baro_err(baro, "pressure compensates to %.1f from raw p=%lu "
			 "t=%lu", (double)press, (unsigned long)adc_p,
			 (unsigned long)adc_t);
		return -ERANGE;
	}

	s->temp_c = temp;
	s->pressure_pa = press;
	return 0;
}

static const struct baro_ops bmp388_ops = {
	.read = bmp388_read,
};

/* ------------------------------------------------------- probe/remove ---- */

int bmp388_probe(struct baro_dev *baro, baro_i2c_bus bus,
		 const struct bmp388_config *cfg)
{
	static const struct bmp388_config defaults;	/* all zero: defaults */
	uint8_t raw[BMP388_CALIB_LEN], id = 0, event = 0;
	struct bmp388_calib calib;
	struct bmp388 *d;
	uint8_t addr;
	int ret;

	if (!baro || !bus)
		return -EINVAL;
	/* Bound already: a second probe would leak the first binding. */
	if (baro->ops || baro->priv)
		return -EBUSY;
	if (!cfg)
		cfg = &defaults;

	baro->name = "bmp388";

	addr = cfg->address ? cfg->address : BMP388_ADDR_SDO_LOW;
	if (addr != BMP388_ADDR_SDO_LOW && addr != BMP388_ADDR_SDO_HIGH) {
		baro_err(baro, "address must be 0x76 or 0x77, not 0x%02x", addr);
		return -EINVAL;
	}
	if (bmp388_meas_ms(cfg->osr) == 0)
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

	baro->ops = &bmp388_ops;
	baro->priv = d;

	ret = bmp388_reg_read(d, BMP388_REG_CHIP_ID, &id, 1);
	if (ret) {
		baro_err(baro, "first transfer to 0x%02x failed (%d)", addr,
			 ret);
		goto fail;
	}
	if (id != BMP388_CHIP_ID) {
		/*
		 * A BMP390 (0x60) is described as sharing this register map.
		 * It is refused because this driver only claims the BMP388 and
		 * nothing about the BMP390 has been checked.
		 */
		baro_err(baro, "id at 0x%02x is 0x%02x, not 0x%02x%s", addr, id,
			 BMP388_CHIP_ID,
			 id == BMP390_CHIP_ID ? " -- that is a BMP390" : "");
		ret = -ENODEV;
		goto fail;
	}

	/*
	 * Soft reset, then the start-up time. EVENT.por_detected is set when a
	 * reset completes and cleared by reading it (§3.2, Table 31), so read
	 * it once beforehand -- it may still be set from power-up -- and the read
	 * afterwards says whether this reset actually finished. The command
	 * decoder has to be ready to take the reset first.
	 */
	ret = bmp388_wait_cmd_rdy(d);
	if (ret) {
		if (ret == -ETIMEDOUT)
			baro_err(baro, "STATUS at 0x%02x never reports cmd_rdy",
				 addr);
		goto fail;
	}
	ret = bmp388_reg_read(d, BMP388_REG_EVENT, &event, 1);
	if (ret)
		goto fail;
	ret = bmp388_reg_write(d, BMP388_REG_CMD, BMP388_CMD_SOFTRESET);
	if (ret)
		goto fail;
	baro_sleep_ms(BMP388_RESET_WAIT_MS);
	ret = bmp388_reg_read(d, BMP388_REG_EVENT, &event, 1);
	if (ret)
		goto fail;
	if (!(event & BMP388_EVENT_POR)) {
		baro_err(baro, "soft reset at 0x%02x not reported complete",
			 addr);
		ret = -EIO;
		goto fail;
	}

	ret = bmp388_reg_read(d, BMP388_REG_CALIB, raw, sizeof(raw));
	if (ret)
		goto fail;
	bmp388_parse_calib(raw, &calib);
	if (!bmp388_calib_ok(&calib)) {
		baro_err(baro, "trim at 0x%02x reads NVM_PAR_T1=%u "
			 "NVM_PAR_P5=%u", addr, calib.t1, calib.p5);
		ret = -EIO;
		goto fail;
	}
	bmp388_scale_calib(&calib, &d->par);

	/*
	 * OSR resets to x4/x1, so it is always written. ODR is unused in
	 * forced mode, but its 5 ms reset period is shorter than most
	 * presets; it is set past the slowest one so that no OSR can be a
	 * configuration error (see BMP388_ODR_80MS). ODR goes first, so that
	 * a chip checking the pair as each is written never sees a slow OSR
	 * against the 5 ms period. PWR_CTRL and CONFIG are left as the reset
	 * gives them: asleep, IIR filter bypassed.
	 */
	ret = bmp388_reg_write(d, BMP388_REG_ODR, BMP388_ODR_80MS);
	if (ret)
		goto fail;
	ret = bmp388_reg_write(d, BMP388_REG_OSR, bmp388_osr_reg(d->osr));
	if (ret)
		goto fail;

	baro_info(baro, "at 0x%02x, OSR 0x%02x, %d ms per read", addr,
		  bmp388_osr_reg(d->osr), bmp388_meas_ms(d->osr));
	return 0;

fail:
	baro->ops = NULL;
	baro->priv = NULL;
	baro_bus_detach(&d->bus);
	free(d);
	return ret;
}

void bmp388_remove(struct baro_dev *baro)
{
	struct bmp388 *d = to_bmp388(baro);

	if (!d)
		return;

	baro_bus_detach(&d->bus);
	free(d);
	baro->ops = NULL;
	baro->priv = NULL;
}

#endif /* CONFIG_BARO_BMP388 */
