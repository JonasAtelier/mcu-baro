#include <stdint.h>

#include "baro_log.h"

/*
 * Decimal text of v, written backwards from end. newlib-nano's printf, the
 * one stm-forge links, has no %lld, so the timestamp cannot go through PRId64.
 */
static const char *baro_format_i64(char *end, int64_t v)
{
	uint64_t u = v < 0 ? 0 - (uint64_t)v : (uint64_t)v;

	*--end = '\0';
	do {
		*--end = (char)('0' + u % 10);
		u /= 10;
	} while (u);
	if (v < 0)
		*--end = '-';
	return end;
}

void baro_log_sample(const struct baro_dev *baro, const struct baro_sample *s)
{
	char t[21];	/* "-9223372036854775808" and its NUL */

	baro_info(baro, "t %s us  %9.1f Pa  %6.2f C",
		  baro_format_i64(t + sizeof(t), s->t_us),
		  (double)s->pressure_pa, (double)s->temp_c);
}
