/*
 * baro -- a small Linux-IIO-style barometer subsystem for ESP-IDF and the
 * STM32 HAL.
 *
 * One struct baro_dev per sensor. A chip driver's probe fills in its name,
 * its ops and its private state; baro_read() does the rest: timestamp, then
 * the chip's own read. Code above this header sees one shape of sample
 * whichever chip produced it.
 *
 * A barometer is a scalar sensor, so unlike mcu-imu there is no mount matrix.
 *
 * Every int-returning function returns 0 or a negative errno.
 */
#ifndef BARO_H
#define BARO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct baro_sample {
	float pressure_pa;
	float temp_c;
	int64_t t_us;		/* baro_time_us() at read */
};

struct baro_dev;

struct baro_ops {
	/* Pa and degrees C; must not touch s->t_us */
	int (*read)(struct baro_dev *baro, struct baro_sample *s);
};

/*
 * Must start zeroed -- static storage or `= { 0 }` -- because probe treats a
 * non-NULL ops or priv as "already bound" and returns -EBUSY rather than
 * leak the first binding. remove, or a failed probe, zeroes them again, so
 * the same baro_dev can be probed afresh.
 */
struct baro_dev {
	const char *name;		/* "bmp280"; log prefix */
	const struct baro_ops *ops;
	void *priv;			/* chip state, owned by probe/remove */
};

/* Timestamp, then ops->read. *s is written only on success. */
int baro_read(struct baro_dev *baro, struct baro_sample *s);

/*
 * The clock baro_read() stamps samples with, in microseconds. On ESP-IDF it
 * is esp_timer_get_time(). On the STM32 HAL the default is weak and only
 * HAL_GetTick() * 1000: millisecond steps that wrap after 49.7 days. Define
 * your own baro_time_us() for real microseconds -- the DWT->CYCCNT cycle
 * counter divided by the core clock in MHz, or a free running 32-bit timer
 * at 1 MHz extended to 64 bits.
 */
int64_t baro_time_us(void);

/*
 * Altitude in metres above the level where the pressure is sea_level_pa, from
 * the international barometric formula.
 *
 * Pass BARO_SEA_LEVEL_PA and you get pressure altitude, not your height --
 * the number wanders by tens of metres as the weather changes. For height
 * above a fixed point, read the pressure there first and use that as the
 * reference; then the sensor is doing what it is good at, which is measuring
 * *change*.
 *
 * 0 for a pressure or reference that is not positive, never NaN.
 */
#define BARO_SEA_LEVEL_PA	101325.0f
float baro_altitude_m(float pressure_pa, float sea_level_pa);

#ifdef __cplusplus
}
#endif

#endif /* BARO_H */
