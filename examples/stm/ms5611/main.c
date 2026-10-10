/*
 * MS5611: pressure, temperature, and height above where you switched on,
 * through the baro core.
 *
 * The altitude printed here is RELATIVE. It takes the first good pressure
 * reading as its zero, which is what you want for "how far has this moved" --
 * absolute altitude would need today's sea-level pressure from a weather
 * service, and would wander tens of metres as the weather changed.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the bus
 * are the app's. Pins are examples -- I2C1 on PB8 (SCL) and PB9 (SDA), AF4 --
 * any pair your chip routes to an I2C peripheral works. Written for the F4,
 * whose I2C takes Init.ClockSpeed; an F7 sets Init.Timing instead, as CubeMX
 * computes it. Needs HAL_I2C_MODULE_ENABLED and BARO_MS5611 = y; printing the
 * floats needs -u _printf_float. baro_altitude_m() needs libm: stm-forge
 * links it, other builds add -lm.
 *
 * Wiring: I2C1, CSB low for 0x77, PS high for I2C. See the README.
 */
#include CMSIS_device_header

#include "baro.h"
#include "baro_log.h"
#include "ms5611.h"

static I2C_HandleTypeDef i2c1;

/* The app's half: pins, peripheral clock and bus speed. */
static void i2c1_init(void)
{
	GPIO_InitTypeDef pins = {
		.Pin = GPIO_PIN_8 | GPIO_PIN_9,
		.Mode = GPIO_MODE_AF_OD,
		.Pull = GPIO_PULLUP,	/* weak; fit real 4.7k pull-ups too */
		.Speed = GPIO_SPEED_FREQ_HIGH,
		.Alternate = GPIO_AF4_I2C1,
	};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();
	HAL_GPIO_Init(GPIOB, &pins);

	i2c1.Instance = I2C1;
	i2c1.Init.ClockSpeed = 400000;
	i2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	i2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	HAL_I2C_Init(&i2c1);
}

int main(void)
{
	static struct baro_dev baro;	/* zeroed */
	float reference_pa = 0.0f;

	HAL_Init();
	/* SystemClock_Config() and the UART behind printf() go here. */
	i2c1_init();

	/* NULL: address 0x77 (CSB low), OSR 4096. */
	if (ms5611_probe(&baro, &i2c1, NULL))
		for (;;)
			;	/* probe already logged why, as "baro: ms5611: ..." */

	for (;;) {
		struct baro_sample s;

		/* two conversions per read, so it blocks 20-22 ms */
		if (!baro_read(&baro, &s)) {
			if (reference_pa == 0.0f) {
				reference_pa = s.pressure_pa;
				baro_info(&baro, "zeroed at %.1f Pa",
					  (double)reference_pa);
			}
			baro_log_sample(&baro, &s);
			baro_info(&baro, "%+7.2f m",
				  (double)baro_altitude_m(s.pressure_pa,
							  reference_pa));
		}
		HAL_Delay(200);
	}
}
