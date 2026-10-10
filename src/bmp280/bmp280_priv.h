/* Internals of the BMP280 driver. Not public API. */
#ifndef BMP280_PRIV_H
#define BMP280_PRIV_H

#include <stdint.h>

#include "baro_priv.h"

#include "bmp280_math.h"

struct bmp280 {
	struct baro_bus bus;
	struct bmp280_calib calib;
	int osr;		/* enum bmp280_osr, as an index for the tables */
};

#endif /* BMP280_PRIV_H */
