#include <errno.h>
#include <math.h>

#include "baro.h"

int baro_read(struct baro_dev *baro, struct baro_sample *s)
{
	struct baro_sample tmp;
	int64_t t;
	int ret;

	if (!baro || !baro->ops || !baro->ops->read || !s)
		return -EINVAL;

	/* Stamp before the bus transfer: that is closer to the sample. */
	t = baro_time_us();
	ret = baro->ops->read(baro, &tmp);
	if (ret)
		return ret;
	tmp.t_us = t;

	*s = tmp;
	return 0;
}

/*
 * sea_level_pa is the pressure at sea level *right now*, which is a weather
 * figure, not a constant -- feed it 101325 and a passing depression will move
 * your "altitude" by 100 m without you leaving the room.
 */
float baro_altitude_m(float pressure_pa, float sea_level_pa)
{
	if (!(pressure_pa > 0.0f) || !(sea_level_pa > 0.0f))
		return 0.0f;
	return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa,
				       1.0f / 5.255f));
}
