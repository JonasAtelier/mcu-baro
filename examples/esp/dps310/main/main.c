/*
 * DPS310: pressure, temperature, and height above where you switched on,
 * through the baro core.
 *
 * The altitude printed here is RELATIVE. It takes the first good pressure
 * reading as its zero, which is what you want for "how far has this moved" --
 * absolute altitude would need today's sea-level pressure from a weather
 * service, and would wander tens of metres as the weather changed.
 *
 * Wiring: any I2C bus, SDO high for 0x77, CSB high for I2C. See the README.
 */
#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "baro.h"
#include "baro_log.h"
#include "dps310.h"

#define PIN_SDA 21
#define PIN_SCL 22

void app_main(void)
{
	i2c_master_bus_config_t buscfg = {
		.i2c_port = -1,
		.sda_io_num = PIN_SDA,
		.scl_io_num = PIN_SCL,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	/*
	 * High pressure oversampling is where the altitude resolution comes
	 * from; temperature barely moves, so it does not need much. 953.6 ms of
	 * every second, just inside the 990 ms budget.
	 */
	struct dps310_config cfg = {
		.pressure_rate = DPS310_RATE_8,
		.pressure_osr = DPS310_OSR_64,
		.temperature_rate = DPS310_RATE_8,
		.temperature_osr = DPS310_OSR_8,
	};
	struct baro_dev baro = { 0 };
	i2c_master_bus_handle_t bus;
	float reference_pa = 0.0f;

	ESP_ERROR_CHECK(i2c_new_master_bus(&buscfg, &bus));

	if (dps310_probe(&baro, bus, &cfg))
		return;		/* probe already logged why, under "baro" */

	for (;;) {
		struct baro_sample s;
		bool ready = false;

		if (dps310_data_ready(&baro, &ready) || !ready) {
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}
		if (baro_read(&baro, &s))
			continue;

		if (reference_pa == 0.0f) {
			reference_pa = s.pressure_pa;
			baro_info(&baro, "zeroed at %.1f Pa",
				  (double)reference_pa);
		}
		baro_log_sample(&baro, &s);
		baro_info(&baro, "%+7.2f m",
			  (double)baro_altitude_m(s.pressure_pa, reference_pa));

		vTaskDelay(pdMS_TO_TICKS(200));
	}
}
