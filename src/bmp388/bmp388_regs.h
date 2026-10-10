/*
 * BMP388 register map.
 *
 * A real register file: write the register address, then read or write bytes
 * from it, with auto-increment on reads (§5.2.2).
 *
 * Every address, bit and timing below is from the Bosch BMP388 datasheet,
 * BST-BMP388-DS001-07, revision 1.7, November 2020 -- docs/BMP388.pdf, which
 * is gitignored, so fetch it from Bosch Sensortec if it is missing. Section
 * and table numbers are that revision's.
 */
#ifndef BMP388_REGS_H
#define BMP388_REGS_H

/* §5.2: the address is 111011x, where x is the SDO pin. */
#define BMP388_ADDR_SDO_LOW	0x76	/* the default */
#define BMP388_ADDR_SDO_HIGH	0x77

/* Registers, Table 24. */
#define BMP388_REG_CHIP_ID	0x00
#define BMP388_REG_ERR		0x02	/* ERR_REG */
#define BMP388_REG_STATUS	0x03
#define BMP388_REG_DATA		0x04	/* DATA_0..5: press[3] then temp[3] */
#define BMP388_REG_EVENT	0x10
#define BMP388_REG_PWR_CTRL	0x1B
#define BMP388_REG_OSR		0x1C
#define BMP388_REG_ODR		0x1D
#define BMP388_REG_CONFIG	0x1F
#define BMP388_REG_CALIB	0x31	/* NVM_PAR_T1..P11, Table 23 */
#define BMP388_CALIB_LEN	21	/* 0x31..0x45 */
#define BMP388_REG_CMD		0x7E

/*
 * The read burst: ERR_REG, STATUS and the six result bytes, 0x02..0x09, so the
 * flags and the results come from the same moment. §3.10 asks for one burst
 * over the results so that they cannot come from two measurements.
 */
#define BMP388_BURST_LEN	8

/*
 * CHIP_ID, Table 24: 0x50.
 *
 * A BMP390 reads 0x60 and is described as sharing this register map. It is
 * rejected because this driver only claims the BMP388 and there is no BMP390
 * datasheet here to check that against.
 */
#define BMP388_CHIP_ID		0x50
#define BMP390_CHIP_ID		0x60

/* CMD, Table 46: softreset. Every user setting goes back to its default. */
#define BMP388_CMD_SOFTRESET	0xB6
/*
 * Table 2, t_startup: 2 ms to first communication after power-up. The
 * datasheet gives no separate soft-reset time, so the same 2 ms is used, and
 * EVENT.por_detected is checked afterwards to confirm the reset finished.
 */
#define BMP388_RESET_WAIT_MS	2

/* ERR_REG, Table 26: cmd_err and conf_err clear on read. */
#define BMP388_ERR_FATAL	0x01
#define BMP388_ERR_CMD		0x02
#define BMP388_ERR_CONF		0x04
#define BMP388_ERR_MASK		0x07

/*
 * STATUS, Table 27. Each drdy bit clears when one of its own data registers
 * is read, so reading STATUS first in the burst sees them still set.
 */
#define BMP388_STATUS_CMD_RDY	0x10
#define BMP388_STATUS_DRDY_PRESS 0x20
#define BMP388_STATUS_DRDY_TEMP	0x40

/* EVENT, Table 31: 1 after power-up or soft reset; clears on read. */
#define BMP388_EVENT_POR	0x01

/*
 * PWR_CTRL, Table 40: press_en[0] | temp_en[1] | mode[5:4], where mode is
 * 00 sleep, 01 or 10 forced, 11 normal (Table 5).
 */
#define BMP388_PWR_PRESS_EN	0x01
#define BMP388_PWR_TEMP_EN	0x02
#define BMP388_MODE_SLEEP	0x00
#define BMP388_MODE_FORCED	0x10
#define BMP388_MODE_NORMAL	0x30
#define BMP388_PWR_FORCED	(BMP388_PWR_PRESS_EN | BMP388_PWR_TEMP_EN | \
				 BMP388_MODE_FORCED)

/*
 * OSR, Table 41: osr_t[5:3] | osr_p[2:0], 0..5 for x1..x32. Resets to 0x02,
 * pressure x4 and temperature x1, so probe writes it.
 *
 * CONFIG (Table 44) resets to 0x00, IIR filter bypassed, which is what this
 * driver wants -- the filter needs a stream of samples (§3.3.3). It is never
 * written.
 */
#define BMP388_OSR_RESET	0x02

/*
 * ODR, §4.3.18 / Table 42: odr_sel[4:0], a sampling period of 5 ms * 2^odr_sel
 * (Table 43). It only paces normal mode. It resets to 0x00, 5 ms, which is
 * shorter than every preset but ultra low (Table 22), and the datasheet does
 * not say whether forced mode checks the two against each other -- conf_err
 * is "sensor configuration error detected" and no more (Table 26). So probe
 * writes 0x04, 80 ms: the smallest period above the slowest preset's
 * 78.09 ms maximum, which no preset can then conflict with.
 */
#define BMP388_ODR_80MS		0x04

/*
 * Both results reset to 0x800000 (DATA_2 and DATA_5 at 0x80, Table 24). The
 * driver relies on STATUS drdy rather than on this, but the fake chip in the
 * tests serves it.
 */
#define BMP388_RAW_RESET	0x800000

#endif /* BMP388_REGS_H */
