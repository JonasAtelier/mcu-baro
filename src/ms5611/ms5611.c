#include <errno.h>
#include <stdlib.h>

#include "baro_priv.h"

#if CONFIG_BARO_MS5611

#include "baro_log.h"
#include "ms5611.h"
#include "ms5611_math.h"
#include "ms5611_priv.h"
#include "ms5611_proto.h"

#define I2C_TIMEOUT_MS		100

static const struct baro_ops ms5611_ops;

/* Our state, or NULL if this baro_dev was not probed by this driver. */
static struct ms5611 *to_ms5611(struct baro_dev *baro)
{
	if (!baro || baro->ops != &ms5611_ops)
		return NULL;
	return baro->priv;
}

/* ---------------------------------------------------------------- I2C ---- */

/*
 * The chip has no register file: every transaction is a bare command byte
 * (COMMANDS), sent with its own START and STOP.
 */
static int ms5611_command(struct ms5611 *d, uint8_t cmd)
{
	return d->bus.ops->transfer(&d->bus, cmd, NULL, 0, NULL, 0);
}

/*
 * A command byte, then len bytes back. I2C INTERFACE (MS5607-02BA03 sheet,
 * Figs 11-15; see ms5611_proto.h) draws these as two transactions -- the
 * command closed by a STOP, then a read of the answer -- so that is what is
 * sent, rather than a repeated start, on both platforms. ESP-IDF used to send
 * one i2c_master_transmit_receive() here, a repeated start whose source was
 * from memory; it gave way to these figures.
 */
static int ms5611_query(struct ms5611 *d, uint8_t cmd, uint8_t *buf,
			size_t len)
{
	int ret;

	ret = ms5611_command(d, cmd);
	if (ret)
		return ret;
	return baro_bus_receive(&d->bus, buf, len, false);
}

/*
 * Start a conversion, wait it out, read the 24-bit result. There is no
 * data-ready flag; a read before the conversion finishes returns 0.
 */
static int ms5611_convert(struct baro_dev *baro, uint8_t cmd, uint32_t *out)
{
	struct ms5611 *d = baro->priv;
	uint8_t b[3];
	int ret;

	ret = ms5611_command(d, (uint8_t)(cmd + ms5611_osr_cmd(d->osr)));
	if (ret)
		return ret;
	/*
	 * Whole milliseconds, rounded up in ms5611_conv_ms(): the HAL has no
	 * microsecond delay, so OSR 256's 0.60 ms costs 1-2 ms.
	 */
	baro_sleep_ms((uint32_t)ms5611_conv_ms(d->osr));
	ret = ms5611_query(d, MS5611_CMD_ADC_READ, b, sizeof(b));
	if (ret)
		return ret;

	*out = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
	if (*out == 0) {
		baro_err(baro, "ADC read 0: the conversion had not finished");
		return -EIO;
	}
	return 0;
}

/* -------------------------------------------------------------- read ----- */

/* baro_ops.read: D2 then D1, each waited out, then compensated. */
static int ms5611_read(struct baro_dev *baro, struct baro_sample *s)
{
	struct ms5611 *d = baro->priv;
	uint32_t d1, d2;
	int32_t temp, pressure;
	int ret;

	/* CONVERSION SEQUENCE: D2 (temperature), then D1 (pressure). */
	ret = ms5611_convert(baro, MS5611_CMD_CONV_D2, &d2);
	if (ret)
		return ret;
	ret = ms5611_convert(baro, MS5611_CMD_CONV_D1, &d1);
	if (ret)
		return ret;

	ms5611_compensate(d->prom, d1, d2, &temp, &pressure);
	s->temp_c = (float)temp / 100.0f;
	s->pressure_pa = (float)pressure;
	return 0;
}

static const struct baro_ops ms5611_ops = {
	.read = ms5611_read,
};

/* ------------------------------------------------------- probe/remove ---- */

int ms5611_probe(struct baro_dev *baro, baro_i2c_bus bus,
		 const struct ms5611_config *cfg)
{
	static const struct ms5611_config defaults;	/* all zero: defaults */
	struct ms5611 *d;
	uint8_t addr;
	int ret, i;

	if (!baro || !bus)
		return -EINVAL;
	/* Bound already: a second probe would leak the first binding. */
	if (baro->ops || baro->priv)
		return -EBUSY;
	if (!cfg)
		cfg = &defaults;

	baro->name = "ms5611";

	addr = cfg->address ? cfg->address : MS5611_ADDR_CSB_LOW;
	if (addr != MS5611_ADDR_CSB_LOW && addr != MS5611_ADDR_CSB_HIGH) {
		baro_err(baro, "address must be 0x76 or 0x77, not 0x%02x", addr);
		return -EINVAL;
	}
	if (ms5611_conv_ms(cfg->osr) == 0)
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

	baro->ops = &ms5611_ops;
	baro->priv = d;

	/* RESET SEQUENCE: once after power-up, and the PROM is reloaded. */
	ret = ms5611_command(d, MS5611_CMD_RESET);
	if (ret) {
		baro_err(baro, "first transfer to 0x%02x failed (%d)", addr,
			 ret);
		goto fail;
	}
	baro_sleep_ms(MS5611_RESET_WAIT_MS);

	for (i = 0; i < MS5611_PROM_WORDS; i++) {
		uint8_t b[2];

		ret = ms5611_query(d, (uint8_t)(MS5611_CMD_PROM_READ + 2 * i),
				   b, sizeof(b));
		if (ret)
			goto fail;
		d->prom[i] = (uint16_t)((b[0] << 8) | b[1]);
	}

	/* No ID register: this CRC is the only proof it is an MS5611 at all. */
	if (!ms5611_prom_ok(d->prom)) {
		baro_err(baro, "PROM at 0x%02x fails its CRC", addr);
		ret = -ENODEV;
		goto fail;
	}

	baro_info(baro, "at 0x%02x, OSR %d", addr, 4096 >> d->osr);
	return 0;

fail:
	baro->ops = NULL;
	baro->priv = NULL;
	baro_bus_detach(&d->bus);
	free(d);
	return ret;
}

void ms5611_remove(struct baro_dev *baro)
{
	struct ms5611 *d = to_ms5611(baro);

	if (!d)
		return;

	baro_bus_detach(&d->bus);
	free(d);
	baro->ops = NULL;
	baro->priv = NULL;
}

#endif /* CONFIG_BARO_MS5611 */
