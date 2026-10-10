/*
 * Shared by the chip drivers inside this library. Not public API.
 *
 * Two layers sit under the chip drivers, the way regmap sits under a kernel
 * driver:
 *
 *   - The bus kind is chosen at run time. A chip holds a struct baro_bus and
 *     reaches it only through bus->ops->transfer() and receive(), so a driver
 *     can sit on I2C, SPI or a UART. Every category library offers all three
 *     kinds, used or not; every baro chip today is on I2C.
 *   - The platform is chosen at link time. baro_port_esp.c and
 *     baro_port_stm.c each define the ops tables, the attach functions, the
 *     clock and the waits below for their platform; the other one compiles
 *     to nothing.
 *
 * ESP_PLATFORM, which ESP-IDF defines for every component, and the HAL's own
 * module switches are defined or not, never 0 or 1, so they are the one place
 * #ifdef is right; this library's CONFIG_BARO_* options are always tested with
 * #if.
 */
#ifndef BARO_PRIV_H
#define BARO_PRIV_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The most bytes after cmd: tx on I2C and UART, tx and rx together on SPI. */
#define BARO_BUS_MAX_LEN	31

struct baro_bus;

enum baro_bus_kind {
	BARO_BUS_I2C,
	BARO_BUS_SPI,
	BARO_BUS_UART,
};

struct baro_bus_ops {
	enum baro_bus_kind kind;
	/*
	 * One transaction: cmd, then tx_len bytes of tx, then rx_len bytes
	 * into rx. cmd is the register address -- or, for a part with no
	 * register file, its command byte -- with any SPI read bit already set
	 * by the chip, and a command byte on a UART.
	 *
	 *   I2C:  rx_len ? cmd, repeated start, read : cmd and tx, one write;
	 *         a bare cmd is a one-byte write. Sending tx and then reading
	 *         is -EINVAL.
	 *   SPI:  CS low, cmd, tx, rx, CS high -- high again even when the
	 *         transfer failed.
	 *   UART: drop stale input, send cmd and tx, then wait for rx_len
	 *         bytes; fewer within the timeout is -ETIMEDOUT.
	 *
	 * On SPI more than BARO_BUS_MAX_LEN bytes after cmd is -EINVAL; on I2C
	 * and UART only tx is bounded so, and rx on the HAL by its 16-bit
	 * count. 0 or a negative errno.
	 */
	int (*transfer)(struct baro_bus *bus, uint8_t cmd, const uint8_t *tx,
			size_t tx_len, uint8_t *rx, size_t rx_len);
	/*
	 * Read rx_len bytes with nothing sent first. On I2C that is a read
	 * transaction of its own, with its own START and STOP -- the second
	 * half of a part that wants its command and its answer as two
	 * transactions; drop_stale must be false there. On a UART it waits
	 * for a part that streams on its own; drop_stale first drops whatever
	 * is already waiting. NULL on SPI. Fewer than rx_len within the
	 * timeout is -ETIMEDOUT; rx is then undefined.
	 */
	int (*receive)(struct baro_bus *bus, uint8_t *rx, size_t rx_len,
		       bool drop_stale);
	/* Undo the attach. The bus itself is the app's and is left alone. */
	void (*detach)(struct baro_bus *bus);
};

#ifdef ESP_PLATFORM
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

/* The I2C argument of every I2C probe on this platform. */
typedef i2c_master_bus_handle_t baro_i2c_bus;

/*
 * One device on one bus, as its driver holds it. BARO_HAVE_BUS says the
 * platform has at least one bus kind, so the struct exists.
 */
#define BARO_HAVE_BUS
struct baro_bus {
	const struct baro_bus_ops *ops;
	uint32_t timeout_ms;
	union {
		i2c_master_dev_handle_t i2c;
		spi_device_handle_t spi;
		uart_port_t uart;
	};
};

#define baro_config_scl_hz(cfg)	((cfg)->scl_hz)

#else /* STM32 HAL */
#include CMSIS_device_header

#if !defined(HAL_I2C_MODULE_ENABLED) && \
    (CONFIG_BARO_BMP280 || CONFIG_BARO_BMP388 || CONFIG_BARO_DPS310 || \
     CONFIG_BARO_MS5611)
#error "the baro chip drivers need the HAL I2C module: #define HAL_I2C_MODULE_ENABLED in the app's stm32f{4,7}xx_hal_conf.h, with stm32f{4,7}xx_hal_i2c.c in the build"
#endif

#ifdef HAL_I2C_MODULE_ENABLED
typedef I2C_HandleTypeDef *baro_i2c_bus;
#endif

#if defined(HAL_I2C_MODULE_ENABLED) || defined(HAL_SPI_MODULE_ENABLED) || \
    defined(HAL_UART_MODULE_ENABLED)
#define BARO_HAVE_BUS
struct baro_bus {
	const struct baro_bus_ops *ops;
	uint32_t timeout_ms;
	union {
#ifdef HAL_I2C_MODULE_ENABLED
		struct {
			I2C_HandleTypeDef *handle;
			uint8_t address;	/* 7-bit */
		} i2c;
#endif
#ifdef HAL_SPI_MODULE_ENABLED
		struct {
			SPI_HandleTypeDef *handle;
			GPIO_TypeDef *cs_port;	/* app's output, idle high */
			uint16_t cs_pin;
		} spi;
#endif
#ifdef HAL_UART_MODULE_ENABLED
		UART_HandleTypeDef *uart;
#endif
	};
};
#endif

/* The configs have no scl_hz here: the bus speed is the handle's. */
#define baro_config_scl_hz(cfg)	0

#endif /* STM32 HAL */

#ifdef BARO_HAVE_BUS
/* Register helpers on top of transfer(). */
static inline int baro_reg_read(struct baro_bus *bus, uint8_t reg, void *buf,
				size_t len)
{
	return bus->ops->transfer(bus, reg, NULL, 0, buf, len);
}

static inline int baro_reg_write(struct baro_bus *bus, uint8_t reg,
				 uint8_t value)
{
	return bus->ops->transfer(bus, reg, &value, 1, NULL, 0);
}

/* bus->ops->receive(), or -EOPNOTSUPP on a kind that has none. */
static inline int baro_bus_receive(struct baro_bus *bus, uint8_t *rx,
				   size_t rx_len, bool drop_stale)
{
	if (!bus->ops->receive)
		return -EOPNOTSUPP;
	return bus->ops->receive(bus, rx, rx_len, drop_stale);
}

static inline void baro_bus_detach(struct baro_bus *bus)
{
	bus->ops->detach(bus);
}
#endif

#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
/*
 * Is anything at this 7-bit address? -ENXIO when nothing ACKs -- nothing at
 * that address -- not -ENODEV, which means something answered but it is the
 * wrong chip. A NACK in the middle of a transfer is a plain -EIO on both
 * platforms, which is why every probe decides presence here, never by its
 * first transfer.
 */
int baro_i2c_probe(baro_i2c_bus handle, uint8_t address, uint32_t timeout_ms);

/*
 * Bind bus to the I2C device at address. On ESP-IDF the driver adds the
 * device at scl_hz, 0 for 400 kHz; on the HAL the clock is the handle's and
 * scl_hz is ignored. Undo with baro_bus_detach(). Every attach leaves
 * bus->ops NULL when it fails, so a bus with ops is one with something to
 * detach.
 */
int baro_i2c_attach(struct baro_bus *bus, baro_i2c_bus handle,
		    uint8_t address, uint32_t scl_hz, uint32_t timeout_ms);
#endif

#ifdef ESP_PLATFORM
/*
 * Add the SPI device on host's cs_pin, in SPI mode 0-3 at clock_hz. The
 * host is the app's, already spi_bus_initialize()d.
 */
int baro_spi_attach(struct baro_bus *bus, spi_host_device_t host, int cs_pin,
		    uint32_t clock_hz, uint8_t mode);

/* port's driver is the app's, already installed and configured. */
int baro_uart_attach(struct baro_bus *bus, uart_port_t port,
		     uint32_t timeout_ms);
#else
#ifdef HAL_SPI_MODULE_ENABLED
/*
 * Bind bus to the SPI device behind cs_port/cs_pin, and deselect it, so the
 * first transfer starts on a falling edge. Mode, clock and frame are the
 * handle's; the chip select is an output the app has set up.
 */
int baro_spi_attach(struct baro_bus *bus, SPI_HandleTypeDef *handle,
		    GPIO_TypeDef *cs_port, uint16_t cs_pin,
		    uint32_t timeout_ms);
#endif
#ifdef HAL_UART_MODULE_ENABLED
/* handle is the app's, already HAL_UART_Init()ed. */
int baro_uart_attach(struct baro_bus *bus, UART_HandleTypeDef *handle,
		     uint32_t timeout_ms);
#endif
#endif

/* Block for at least ms milliseconds: long, never short. */
void baro_sleep_ms(uint32_t ms);

/*
 * Polling a status bit until a deadline, the way each original port did it:
 * baro_tick_now() starts the clock, baro_expired() says whether limit_ms has
 * surely passed, and baro_poll_wait() pauses between two polls.
 */
uint32_t baro_tick_now(void);
bool baro_expired(uint32_t start, uint32_t limit_ms);
void baro_poll_wait(uint32_t ms);

#endif /* BARO_PRIV_H */
