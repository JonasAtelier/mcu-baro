/* Internals of the MS5611 driver. Not public API. */
#ifndef MS5611_PRIV_H
#define MS5611_PRIV_H

#include <stdint.h>

#include "baro_priv.h"

#include "ms5611_proto.h"

struct ms5611 {
	struct baro_bus bus;
	uint16_t prom[MS5611_PROM_WORDS];	/* C1..C6 in [1]..[6] */
	int osr;		/* enum ms5611_osr, as an index for the tables */
};

#endif /* MS5611_PRIV_H */
