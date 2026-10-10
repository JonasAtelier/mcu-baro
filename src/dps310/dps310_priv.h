/* Internals of the DPS310 driver. Not public API. */
#ifndef DPS310_PRIV_H
#define DPS310_PRIV_H

#include <stdint.h>

#include "baro_priv.h"

#include "dps310_math.h"

struct dps310 {
	struct baro_bus bus;
	struct dps310_coefs coefs;
	float kp;	/* scaling divisor for the configured pressure oversampling */
	float kt;	/* and for temperature */
	int prs_osr;
	int tmp_osr;
};

#endif /* DPS310_PRIV_H */
