/*
 * baro_priv.h on the STM32 HAL. stm-forge compiles every .c in src/, so
 * under ESP-IDF this file is empty; the ESP-IDF CMakeLists never lists it.
 */
#ifndef ESP_PLATFORM

#include <string.h>

#include "baro.h"
#include "baro_priv.h"

/*
 * The core is always built; each chip driver only under its own option. With
 * the library on and no chip picked, say so -- stm-forge compiles this file
 * only when PKG_baro = y, so this is the CMakeLists' "BARO on, no chip"
 * warning on the ESP-IDF side.
 */
#if !(CONFIG_BARO_BMP280 || CONFIG_BARO_BMP388 || CONFIG_BARO_DPS310 || \
      CONFIG_BARO_MS5611)
#warning "baro: no chip driver is selected; set BARO_<CHIP> = y under PKG_baro = y (or -DCONFIG_BARO_<CHIP>=1)"
#endif

/* Milliseconds dressed as microseconds; see baro.h for a real clock. */
__attribute__((weak)) int64_t baro_time_us(void)
{
	return (int64_t)HAL_GetTick() * 1000;
}

/*
 * HAL_Delay() already adds one tick to what it is asked, because its first
 * tick may be about to fire, so ms is passed straight through: 10 ms waits
 * 10-11 ms at the HAL's default 1 kHz tick, and at a slower tick still never
 * less than 10. Long, never short -- as long as the app has not replaced the
 * weak HAL_Delay() with one that drops that tick. It busy-waits on SysTick,
 * so never call it from an interrupt at or above SysTick's priority.
 */
void baro_sleep_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

/*
 * Polls count HAL_GetTick() milliseconds, as the STM32 drivers always did;
 * baro_expired() below says how the first, partial one is handled.
 */
uint32_t baro_tick_now(void)
{
	return HAL_GetTick();
}

/*
 * Has more than limit_ms passed since start, a baro_tick_now() reading? The
 * first millisecond after start may be partial, so it takes strictly more
 * than limit_ms counted to be sure limit_ms really went by. Unsigned
 * subtraction, so it holds across the counter's wrap after 49.7 days.
 */
bool baro_expired(uint32_t start, uint32_t limit_ms)
{
	return (uint32_t)(HAL_GetTick() - start) > limit_ms;
}

/* There is no scheduler to yield to: sleep instead. */
void baro_poll_wait(uint32_t ms)
{
	baro_sleep_ms(ms);
}

#if defined(HAL_I2C_MODULE_ENABLED) || defined(HAL_SPI_MODULE_ENABLED) || \
    defined(HAL_UART_MODULE_ENABLED)
/*
 * HAL_StatusTypeDef -> negative errno, at the bus boundary and nowhere else.
 * HAL_BUSY -- the bus or the handle in use, or a handle never initialised --
 * is -EAGAIN, because -EBUSY from a probe already means "this baro_dev is
 * bound". A NACK in the middle of a transfer is HAL_ERROR like any other
 * fault, so -EIO.
 */
static int baro_errno(HAL_StatusTypeDef status)
{
	switch (status) {
	case HAL_OK:
		return 0;
	case HAL_TIMEOUT:
		return -ETIMEDOUT;
	case HAL_BUSY:
		return -EAGAIN;
	default:
		return -EIO;
	}
}

/* Nothing to undo: the handle is the app's, and nothing was added to it. */
static void baro_hal_detach(struct baro_bus *bus)
{
	(void)bus;
}
#endif

#ifdef HAL_I2C_MODULE_ENABLED
/*
 * HAL_I2C_IsDeviceReady() returns HAL_ERROR when no trial was ACKed, which is
 * -ENXIO. A busy or stuck bus keeps its own errno. On the F4, no pull-ups or
 * SDA held low sets the peripheral's BUSY flag, so the HAL gives up after
 * 25 ms with HAL_BUSY, -EAGAIN; HAL_TIMEOUT, -ETIMEDOUT, means a START never
 * completed.
 */
int baro_i2c_probe(baro_i2c_bus handle, uint8_t address, uint32_t timeout_ms)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_IsDeviceReady(handle, (uint16_t)(address << 1), 1,
				       timeout_ms);
	return status == HAL_ERROR ? -ENXIO : baro_errno(status);
}

/* HAL_I2C_Mem_Read(): cmd, repeated start, burst. Mem_Write(): cmd, data. */
static int baro_i2c_transfer(struct baro_bus *bus, uint8_t cmd,
			     const uint8_t *tx, size_t tx_len, uint8_t *rx,
			     size_t rx_len)
{
	uint16_t address = (uint16_t)(bus->i2c.address << 1);

	if ((tx_len && rx_len) || tx_len > BARO_BUS_MAX_LEN ||
	    rx_len > UINT16_MAX)
		return -EINVAL;
	if (rx_len)
		return baro_errno(HAL_I2C_Mem_Read(bus->i2c.handle, address,
						   cmd, I2C_MEMADD_SIZE_8BIT,
						   rx, (uint16_t)rx_len,
						   bus->timeout_ms));
	if (tx_len)
		return baro_errno(HAL_I2C_Mem_Write(bus->i2c.handle, address,
						    cmd, I2C_MEMADD_SIZE_8BIT,
						    (uint8_t *)tx,
						    (uint16_t)tx_len,
						    bus->timeout_ms));
	/* A bare command, for a part with no registers: its own START, STOP. */
	return baro_errno(HAL_I2C_Master_Transmit(bus->i2c.handle, address,
						  &cmd, 1, bus->timeout_ms));
}

/* A read transaction of its own: START, address, rx_len bytes, STOP. */
static int baro_i2c_receive(struct baro_bus *bus, uint8_t *rx, size_t rx_len,
			    bool drop_stale)
{
	uint16_t address = (uint16_t)(bus->i2c.address << 1);

	if (!rx_len || drop_stale || rx_len > UINT16_MAX)
		return -EINVAL;
	return baro_errno(HAL_I2C_Master_Receive(bus->i2c.handle, address, rx,
						 (uint16_t)rx_len,
						 bus->timeout_ms));
}

static const struct baro_bus_ops baro_i2c_ops = {
	.kind = BARO_BUS_I2C,
	.transfer = baro_i2c_transfer,
	.receive = baro_i2c_receive,
	.detach = baro_hal_detach,
};

int baro_i2c_attach(struct baro_bus *bus, baro_i2c_bus handle, uint8_t address,
		    uint32_t scl_hz, uint32_t timeout_ms)
{
	(void)scl_hz;
	bus->ops = &baro_i2c_ops;
	bus->timeout_ms = timeout_ms;
	bus->i2c.handle = handle;
	bus->i2c.address = address;
	return 0;
}
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
/*
 * One transaction under a software chip select: CS low, cmd and tx out --
 * full duplex with zeros after them when there is something to read -- CS
 * high. CS goes high even when the transfer failed; see baro_bus_ops.
 */
static int baro_spi_transfer(struct baro_bus *bus, uint8_t cmd,
			     const uint8_t *tx, size_t tx_len, uint8_t *rx,
			     size_t rx_len)
{
	uint8_t out[1 + BARO_BUS_MAX_LEN] = { 0 }, in[1 + BARO_BUS_MAX_LEN];
	uint16_t n = (uint16_t)(1 + tx_len + rx_len);
	HAL_StatusTypeDef status;

	if (tx_len + rx_len > BARO_BUS_MAX_LEN)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	HAL_GPIO_WritePin(bus->spi.cs_port, bus->spi.cs_pin, GPIO_PIN_RESET);
	if (rx_len)
		status = HAL_SPI_TransmitReceive(bus->spi.handle, out, in, n,
						 bus->timeout_ms);
	else
		status = HAL_SPI_Transmit(bus->spi.handle, out, n,
					  bus->timeout_ms);
	HAL_GPIO_WritePin(bus->spi.cs_port, bus->spi.cs_pin, GPIO_PIN_SET);
	if (status == HAL_OK && rx_len)
		memcpy(rx, in + 1 + tx_len, rx_len);
	return baro_errno(status);
}

static const struct baro_bus_ops baro_spi_ops = {
	.kind = BARO_BUS_SPI,
	.transfer = baro_spi_transfer,
	.detach = baro_hal_detach,
};

int baro_spi_attach(struct baro_bus *bus, SPI_HandleTypeDef *handle,
		    GPIO_TypeDef *cs_port, uint16_t cs_pin,
		    uint32_t timeout_ms)
{
	bus->ops = &baro_spi_ops;
	bus->timeout_ms = timeout_ms;
	bus->spi.handle = handle;
	bus->spi.cs_port = cs_port;
	bus->spi.cs_pin = cs_pin;
	/* Deselected first, so the first transfer starts on a falling edge. */
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
	return 0;
}
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
/*
 * Blocking HAL_UART_Receive() hands back whatever already sits in the data
 * register, so a late byte from an earlier exchange would be read as the
 * start of this reply. Drop it first: clear an overrun, then read the data
 * register empty (on the F7 that macro is a flush request). Bounded by the
 * timeout, in case the line never goes quiet.
 */
static void baro_uart_drop_input(struct baro_bus *bus)
{
	UART_HandleTypeDef *handle = bus->uart;
	uint32_t start = HAL_GetTick();

	__HAL_UART_CLEAR_OREFLAG(handle);
	while (__HAL_UART_GET_FLAG(handle, UART_FLAG_RXNE) &&
	       (uint32_t)(HAL_GetTick() - start) <= bus->timeout_ms)
		__HAL_UART_FLUSH_DRREGISTER(handle);
}

static int baro_uart_transfer(struct baro_bus *bus, uint8_t cmd,
			      const uint8_t *tx, size_t tx_len, uint8_t *rx,
			      size_t rx_len)
{
	uint8_t out[1 + BARO_BUS_MAX_LEN];
	int ret;

	if (tx_len > BARO_BUS_MAX_LEN || rx_len > UINT16_MAX)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	baro_uart_drop_input(bus);
	ret = baro_errno(HAL_UART_Transmit(bus->uart, out,
					   (uint16_t)(1 + tx_len),
					   bus->timeout_ms));
	if (ret || !rx_len)
		return ret;
	/* A reply that stops short runs into the timeout: HAL_TIMEOUT. */
	return baro_errno(HAL_UART_Receive(bus->uart, rx, (uint16_t)rx_len,
					   bus->timeout_ms));
}

/* Nothing sent: the part streams. A short read runs into HAL_TIMEOUT. */
static int baro_uart_receive(struct baro_bus *bus, uint8_t *rx, size_t rx_len,
			     bool drop_stale)
{
	if (!rx_len || rx_len > UINT16_MAX)
		return -EINVAL;
	if (drop_stale)
		baro_uart_drop_input(bus);
	return baro_errno(HAL_UART_Receive(bus->uart, rx, (uint16_t)rx_len,
					   bus->timeout_ms));
}

static const struct baro_bus_ops baro_uart_ops = {
	.kind = BARO_BUS_UART,
	.transfer = baro_uart_transfer,
	.receive = baro_uart_receive,
	.detach = baro_hal_detach,
};

int baro_uart_attach(struct baro_bus *bus, UART_HandleTypeDef *handle,
		     uint32_t timeout_ms)
{
	bus->ops = &baro_uart_ops;
	bus->timeout_ms = timeout_ms;
	bus->uart = handle;
	return 0;
}
#endif /* HAL_UART_MODULE_ENABLED */

#endif /* !ESP_PLATFORM */
