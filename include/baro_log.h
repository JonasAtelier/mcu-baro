/*
 * Kernel dev_err()-style logging for barometer drivers and their users.
 *
 * ESP-IDF: everything goes out under the one ESP-IDF tag "baro", prefixed
 * with the device's name, so one esp_log_level_set("baro", ...) controls
 * every chip and a line still says which chip it came from. There is
 * deliberately no level machinery of our own; baro_dbg() is also subject to
 * CONFIG_LOG_MAXIMUM_LEVEL, and CONFIG_BARO_DEBUG is not used.
 *
 * STM32 HAL: every line goes out through printf() as "baro: <name>: ...", so
 * a line still says which chip it came from. Where printf() ends up is the
 * app's business: with newlib, retarget _write() to a UART. There is
 * deliberately no level machinery; baro_dbg() alone is compiled out unless
 * CONFIG_BARO_DEBUG is 1 (stm-forge: BARO_DEBUG = y).
 */
#ifndef BARO_LOG_H
#define BARO_LOG_H

#ifdef ESP_PLATFORM
#include "esp_log.h"
#else
#include <stdio.h>
#endif

#include "baro.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ESP_PLATFORM
#define BARO_LOG_TAG	"baro"

#define baro_err(baro, fmt, ...) \
	ESP_LOGE(BARO_LOG_TAG, "%s: " fmt, (baro)->name, ##__VA_ARGS__)
#define baro_warn(baro, fmt, ...) \
	ESP_LOGW(BARO_LOG_TAG, "%s: " fmt, (baro)->name, ##__VA_ARGS__)
#define baro_info(baro, fmt, ...) \
	ESP_LOGI(BARO_LOG_TAG, "%s: " fmt, (baro)->name, ##__VA_ARGS__)
#define baro_dbg(baro, fmt, ...) \
	ESP_LOGD(BARO_LOG_TAG, "%s: " fmt, (baro)->name, ##__VA_ARGS__)

#else /* STM32 HAL */
#define baro_err(baro, fmt, ...) \
	printf("baro: %s: " fmt "\n", (baro)->name, ##__VA_ARGS__)
#define baro_warn(baro, fmt, ...) \
	printf("baro: %s: " fmt "\n", (baro)->name, ##__VA_ARGS__)
#define baro_info(baro, fmt, ...) \
	printf("baro: %s: " fmt "\n", (baro)->name, ##__VA_ARGS__)

#if CONFIG_BARO_DEBUG
#define baro_dbg(baro, fmt, ...) \
	printf("baro: %s: " fmt "\n", (baro)->name, ##__VA_ARGS__)
#else
/* Still type-checked, so a debug line cannot rot while it is off. */
#define baro_dbg(baro, fmt, ...) do { \
	if (0) \
		printf("baro: %s: " fmt "\n", (baro)->name, ##__VA_ARGS__); \
} while (0)
#endif
#endif /* ESP_PLATFORM */

/* One info line: timestamp, pressure, temperature. */
void baro_log_sample(const struct baro_dev *baro, const struct baro_sample *s);

#ifdef __cplusplus
}
#endif

#endif /* BARO_LOG_H */
