/*
 * MS5611 commands and addresses.
 *
 * The chip has no register map: every transaction is a one-byte command.
 * The addresses are checked against the MS5611-01BA03 datasheet (06/2017):
 * https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5611-01BA03&DocType=Data+Sheet&DocLang=English
 * The CRC in ms5611_math.h against AN520 rev 004:
 * https://www.amsys-sensor.com/downloads/notes/MS5XXX-C-code-example-for-MS56xx-MS57xx-MS58xx-AMSYS-an520e.pdf
 * The MS5611 sheet itself was unreachable (TE: Access Denied), so the command
 * bytes, the 2.8 ms PROM reload and the I2C INTERFACE sequences (a STOP
 * between a command and its answer, Figs 11-15) are checked against the
 * sibling MS5607-02BA03 datasheet, DA5607-02BA03_018. The other section
 * headings cited here are not checked against either sheet.
 */
#ifndef MS5611_PROTO_H
#define MS5611_PROTO_H

/* I2C INTERFACE: the address is 111011C, where C is the complement of CSB. */
#define MS5611_ADDR_CSB_HIGH	0x76
#define MS5611_ADDR_CSB_LOW	0x77	/* the default */

/* COMMANDS */
#define MS5611_CMD_RESET	0x1E
#define MS5611_CMD_CONV_D1	0x40	/* pressure at OSR 256; +2 per step, 0x48 at 4096 */
#define MS5611_CMD_CONV_D2	0x50	/* temperature, likewise 0x50..0x58 */
#define MS5611_CMD_ADC_READ	0x00	/* 24 bits, MSB first; 0 if no conversion finished */
#define MS5611_CMD_PROM_READ	0xA0	/* +2*n for word n; 16 bits, MSB first */
#define MS5611_PROM_WORDS	8	/* 0 factory, 1..6 = C1..C6, 7 serial + CRC */

/* RESET SEQUENCE: the PROM reloads after a reset, which takes 2.8 ms. */
#define MS5611_RESET_WAIT_MS	3

#endif /* MS5611_PROTO_H */
