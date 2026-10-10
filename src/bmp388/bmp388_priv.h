/* Internals of the BMP388 driver. Not public API. */
#ifndef BMP388_PRIV_H
#define BMP388_PRIV_H

#include <stdint.h>

#include "baro_priv.h"

#include "bmp388_math.h"

struct bmp388 {
	struct baro_bus bus;
	struct bmp388_par par;
	int osr;		/* enum bmp388_osr, as an index for the tables */
};

#endif /* BMP388_PRIV_H */
