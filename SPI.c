#include "spi.h"

void spi_init(void)
{
  DDR_SCK |= (1 << SCK);
  DDR_MOSI |= (1 << MOSI);
  DDR_MISO &= ~(1 << MISO);
  DDR_SS |= (1 << SS);

  PORT_SS |= (1 << SS);

  SPCR = (1 << SPE) | (1 << MSTR);
}

void spi_write_byte(uint8_t data)
{
  SPDR = data;
  while (!(SPSR & (1 << SPIF)))
    ;
}

uint8_t spi_read_byte(void)
{
  while (!(SPSR & (1 << SPIF)))
    ;
  return SPDR;
}

uint8_t spi_read_write_byte(uint8_t data)
{
  SPDR = data;
  while (!(SPSR & (1 << SPIF)))
    ;
  return SPDR;
}

void spi_write_read_buf(uint8_t *write_buf, uint8_t *read_buf, uint8_t len)
{
  uint8_t i;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    for (i = 0; i < len; i++)
    {
      SPDR = write_buf[i];
      while (!(SPSR & (1 << SPIF)))
        ;
      read_buf[i] = SPDR;
    }
  }
}
