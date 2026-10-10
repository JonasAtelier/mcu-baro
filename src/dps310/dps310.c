#include <errno.h>
#include <stdlib.h>

#include "baro_priv.h"

#if CONFIG_BARO_DPS310

#include "baro_log.h"
#include "dps310.h"
#include "dps310_math.h"
#include "dps310_priv.h"
#include "dps310_regs.h"

#define I2C_TIMEOUT_MS		100
#define RESET_WAIT_MS		40	/* the datasheet allows 40 ms to first ready */
#define READY_TIMEOUT_MS	250
#define READY_POLL_MS		5

static const struct baro_ops dps310_ops;

/* Our state, or NULL if this baro_dev was not probed by this driver. */
static struct dps310 *to_dps310(struct baro_dev *baro)
{
	if (!baro || baro->ops != &dps310_ops)
		return NULL;
	return baro->priv;
}

/* ---------------------------------------------------------------- I2C ---- */

int dps310_reg_read(struct baro_dev *baro, uint8_t reg, void *buf, size_t len)
{
	struct dps310 *d = to_dps310(baro);

	if (!d || !buf || len == 0)
		return -EINVAL;

	/* Register address, repeated start, then the burst. */
	return baro_reg_read(&d->bus, reg, buf, len);
}

int dps310_reg_write(struct baro_dev *baro, uint8_t reg, uint8_t value)
{
	struct dps310 *d = to_dps310(baro);

	if (!d)
		return -EINVAL;

	return baro_reg_write(&d->bus, reg, value);
}

/*
 * Polls until READY_TIMEOUT_MS has passed by baro_expired(), baro_poll_wait()
 * between polls. The last read is at or after the deadline.
 */
static int dps310_wait_flags(struct baro_dev *baro, uint8_t mask)
{
	const uint32_t start = baro_tick_now();
	int ret;

	for (;;) {
		uint8_t m = 0;

		ret = dps310_reg_read(baro, DPS310_MEAS_CFG, &m, 1);
		if (ret)
			return ret;
		if ((m & mask) == mask)
			return 0;
		if (baro_expired(start, READY_TIMEOUT_MS))
			return -ETIMEDOUT;
		baro_poll_wait(READY_POLL_MS);
	}
}

/*
 * Some parts read around 60 C when it is really around 20 C. This is the
 * manufacturer's recommended fix; the registers are undocumented and both
 * reference drivers carry the same sequence. Bit 1 of 0x32 says whether this
 * particular part already has it.
 */
static int dps310_apply_temperature_errata(struct baro_dev *baro)
{
	static const uint8_t seq[5][2] = {
		{ 0x0E, 0xA5 }, { 0x0F, 0x96 }, { 0x62, 0x02 },
		{ 0x0E, 0x00 }, { 0x0F, 0x00 },
	};
	uint8_t probe = 0;
	int ret, i;

	ret = dps310_reg_read(baro, DPS310_ERRATA_PROBE, &probe, 1);
	if (ret)
		return ret;
	if (probe & DPS310_ERRATA_OK_BIT)
		return 0;

	baro_info(baro, "applying the temperature erratum fix");
	for (i = 0; i < 5; i++) {
		ret = dps310_reg_write(baro, seq[i][0], seq[i][1]);
		if (ret)
			return ret;
	}
	return 0;
}

/* -------------------------------------------------------------- read ----- */

int dps310_read_raw(struct baro_dev *baro, int32_t *pressure,
		    int32_t *temperature)
{
	uint8_t b[2 * DPS310_RESULT_LEN];
	int ret;

	if (!to_dps310(baro) || !pressure || !temperature)
		return -EINVAL;

	/*
	 * PRS and TMP are adjacent, so one burst keeps them from different
	 * measurement cycles.
	 */
	ret = dps310_reg_read(baro, DPS310_PRS_B2, b, sizeof(b));
	if (ret)
		return ret;
	*pressure = dps310_raw24(&b[0]);
	*temperature = dps310_raw24(&b[DPS310_RESULT_LEN]);
	return 0;
}

/* baro_ops.read: the latest background result, compensated. */
static int dps310_read(struct baro_dev *baro, struct baro_sample *s)
{
	struct dps310 *d = baro->priv;
	int32_t praw, traw;
	float traw_sc, praw_sc;
	int ret;

	ret = dps310_read_raw(baro, &praw, &traw);
	if (ret)
		return ret;

	traw_sc = dps310_traw_sc(traw, d->kt);
	praw_sc = d->kp > 0.0f ? (float)praw / d->kp : 0.0f;

	s->temp_c = dps310_temperature_c(&d->coefs, traw_sc);
	s->pressure_pa = dps310_pressure_pa(&d->coefs, praw_sc, traw_sc);
	return 0;
}

int dps310_data_ready(struct baro_dev *baro, bool *ready)
{
	uint8_t m = 0;
	int ret;

	if (!to_dps310(baro) || !ready)
		return -EINVAL;

	ret = dps310_reg_read(baro, DPS310_MEAS_CFG, &m, 1);
	if (ret)
		return ret;
	*ready = (m & (DPS310_PRS_RDY | DPS310_TMP_RDY)) ==
		 (DPS310_PRS_RDY | DPS310_TMP_RDY);
	return 0;
}

int dps310_coefficients(struct baro_dev *baro, int32_t out[9])
{
	struct dps310 *d = to_dps310(baro);

	if (!d || !out)
		return -EINVAL;

	out[0] = d->coefs.c0;
	out[1] = d->coefs.c1;
	out[2] = d->coefs.c00;
	out[3] = d->coefs.c10;
	out[4] = d->coefs.c01;
	out[5] = d->coefs.c11;
	out[6] = d->coefs.c20;
	out[7] = d->coefs.c21;
	out[8] = d->coefs.c30;
	return 0;
}

static const struct baro_ops dps310_ops = {
	.read = dps310_read,
};

/* ------------------------------------------------------- probe/remove ---- */

int dps310_probe(struct baro_dev *baro, baro_i2c_bus bus,
		 const struct dps310_config *cfg)
{
	uint8_t addr, id = 0, coef[DPS310_COEF_LEN], srce = 0, shift = 0;
	struct dps310 *d;
	int busy, ret;

	if (!baro || !bus || !cfg)
		return -EINVAL;
	/* Bound already: a second probe would leak the first binding. */
	if (baro->ops || baro->priv)
		return -EBUSY;

	baro->name = "dps310";

	addr = cfg->address ? cfg->address : DPS310_ADDR_HIGH;
	if (addr != DPS310_ADDR_LOW && addr != DPS310_ADDR_HIGH) {
		baro_err(baro, "address must be 0x76 or 0x77, not 0x%02x", addr);
		return -EINVAL;
	}
	if (dps310_scale_factor(cfg->pressure_osr) <= 0.0f ||
	    dps310_scale_factor(cfg->temperature_osr) <= 0.0f ||
	    dps310_rate_hz(cfg->pressure_rate) == 0 ||
	    dps310_rate_hz(cfg->temperature_rate) == 0)
		return -EINVAL;

	busy = dps310_busy_time(cfg->pressure_rate, cfg->pressure_osr) +
	       dps310_busy_time(cfg->temperature_rate, cfg->temperature_osr);
	if (busy >= DPS310_BUSY_LIMIT) {
		baro_err(baro, "pressure %dx at %d Hz and temperature %dx at %d Hz "
			 "need %d.%d ms of every second; the limit is %d ms",
			 dps310_oversampling(cfg->pressure_osr),
			 dps310_rate_hz(cfg->pressure_rate),
			 dps310_oversampling(cfg->temperature_osr),
			 dps310_rate_hz(cfg->temperature_rate),
			 busy / 10, busy % 10, DPS310_BUSY_LIMIT / 10);
		return -EINVAL;
	}

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
	d->prs_osr = cfg->pressure_osr;
	d->tmp_osr = cfg->temperature_osr;
	d->kp = dps310_scale_factor(cfg->pressure_osr);
	d->kt = dps310_scale_factor(cfg->temperature_osr);

	/* From here on the chip helpers above work on this baro_dev. */
	baro->ops = &dps310_ops;
	baro->priv = d;

	ret = dps310_reg_read(baro, DPS310_PRODUCT_ID, &id, 1);
	if (ret) {
		baro_err(baro, "first transfer to 0x%02x failed (%d)", addr,
			 ret);
		goto fail;
	}
	if (id != DPS310_PRODUCT_ID_VALUE) {
		baro_err(baro, "no DPS310 at 0x%02x: product ID 0x%02x, "
			 "expected 0x%02x", addr, id, DPS310_PRODUCT_ID_VALUE);
		ret = -ENODEV;
		goto fail;
	}

	ret = dps310_reg_write(baro, DPS310_RESET, DPS310_RESET_MAGIC);
	if (ret)
		goto fail;
	baro_sleep_ms(RESET_WAIT_MS);

	/*
	 * Both the sensor and its factory calibration have to be up before the
	 * coefficients mean anything. Reading them early gives zeros.
	 */
	ret = dps310_wait_flags(baro, DPS310_SENSOR_RDY | DPS310_COEF_RDY);
	if (ret) {
		baro_err(baro, "sensor or coefficients never became ready (%d)",
			 ret);
		goto fail;
	}

	ret = dps310_apply_temperature_errata(baro);
	if (ret)
		goto fail;

	ret = dps310_reg_read(baro, DPS310_COEF_BASE, coef, sizeof(coef));
	if (ret)
		goto fail;
	dps310_unpack_coefs(coef, &d->coefs);

	/*
	 * The calibration was taken against one of two internal temperature
	 * sensors, and COEF_SRCE says which. Measure with the other one and
	 * every temperature is wrong -- and because pressure compensation uses
	 * the temperature term, every pressure is wrong with it.
	 */
	ret = dps310_reg_read(baro, DPS310_COEF_SRCE, &srce, 1);
	if (ret)
		goto fail;
	srce &= DPS310_COEF_SRCE_MASK;

	ret = dps310_reg_write(baro, DPS310_PRS_CFG,
			       (uint8_t)((cfg->pressure_rate << DPS310_RATE_SHIFT) |
					 (cfg->pressure_osr << DPS310_PRC_SHIFT)));
	if (!ret)
		ret = dps310_reg_write(baro, DPS310_TMP_CFG,
				       (uint8_t)(srce |
						 (cfg->temperature_rate << DPS310_RATE_SHIFT) |
						 (cfg->temperature_osr << DPS310_PRC_SHIFT)));
	if (ret)
		goto fail;

	/* Above 8x the result comes back shifted, and CFG_REG has to know. */
	if (dps310_needs_shift(cfg->pressure_osr))
		shift |= DPS310_PRS_SHIFT_EN;
	if (dps310_needs_shift(cfg->temperature_osr))
		shift |= DPS310_TMP_SHIFT_EN;
	ret = dps310_reg_write(baro, DPS310_CFG_REG, shift);
	if (!ret)
		ret = dps310_reg_write(baro, DPS310_MEAS_CFG,
				       DPS310_MEAS_CONTINUOUS);
	if (ret)
		goto fail;

	baro_info(baro, "at 0x%02x: pressure %dx at %d Hz, temperature %dx at %d Hz",
		  addr, dps310_oversampling(d->prs_osr),
		  dps310_rate_hz(cfg->pressure_rate),
		  dps310_oversampling(d->tmp_osr),
		  dps310_rate_hz(cfg->temperature_rate));
	return 0;

fail:
	baro->ops = NULL;
	baro->priv = NULL;
	baro_bus_detach(&d->bus);
	free(d);
	return ret;
}

void dps310_remove(struct baro_dev *baro)
{
	struct dps310 *d = to_dps310(baro);

	if (!d)
		return;

	dps310_reg_write(baro, DPS310_MEAS_CFG, 0);	/* idle */
	baro_bus_detach(&d->bus);
	free(d);
	baro->ops = NULL;
	baro->priv = NULL;
}

#endif /* CONFIG_BARO_DPS310 */
