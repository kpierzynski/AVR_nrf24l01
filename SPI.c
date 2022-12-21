#include "SPI.h"

void SPI_init()
{
	DDR_SCK |= SCK;
	DDR_MOSI |= MOSI;
	DDR_MISO &= ~MISO;

	DDR_SS |= SS;

	PORT_SCK |= SCK;
	PORT_MOSI |= MOSI;
	PORT_SS &= ~SS;

	SPCR = (0 << SPIE) | (1 << SPE) | (0 << DORD) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA) | (0 << SPR0) | (1 << SPR1);
	SPSR |= (1 << SPI2X);
}

void SPI_ss_set(uint8_t state)
{
	if (state)
		PORT_SS |= SS;
	else
		PORT_SS &= ~(SS);
}

void SPI_w_byte(uint8_t byte)
{
	SPDR = byte;
	while (!(SPSR & (1 << SPIF)))
		;
}

uint8_t SPI_rw_byte(uint8_t byte)
{
	SPDR = byte;
	while (!(SPSR & (1 << SPIF)))
		;
	return SPDR;
}

void SPI_rw_buffer(uint8_t *rbuf, uint8_t *wbuf, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++)
		rbuf[i] = SPI_rw_byte(wbuf[i]);
}