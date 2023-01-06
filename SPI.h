// spi.h
#ifndef SPI_H
#define SPI_H

#include <avr/io.h>
#include <util/atomic.h>

#define SCK PB5
#define MISO PB4
#define MOSI PB3
#define SS PB1

#define DDR_MOSI DDRB
#define PORT_MOSI PORTB
#define PIN_MOSI PINB

#define DDR_MISO DDRB
#define PORT_MISO PORTB
#define PIN_MISO PINB

#define DDR_SCK DDRB
#define PORT_SCK PORTB
#define PIN_SCK PINB

#define DDR_SS DDRB
#define PORT_SS PORTB
#define PIN_SS PINB

void spi_init(void);
void spi_write_byte(uint8_t data);
uint8_t spi_read_byte(void);
uint8_t spi_read_write_byte(uint8_t data);
void spi_write_read_buf(uint8_t *write_buf, uint8_t *read_buf, uint8_t len);

#endif
