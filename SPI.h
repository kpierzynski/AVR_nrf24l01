#ifndef __SPI_H_
#define __SPI_H_

#include <avr/io.h>

#define SCK (1 << PB5)
#define DDR_SCK DDRB
#define PORT_SCK PORTB

#define MOSI (1 << PB3)
#define DDR_MOSI DDRB
#define PORT_MOSI PORTB

#define MISO (1 << PB4)
#define DDR_MISO DDRB
#define PORT_MISO PORTB

#define SS (1 << PB1)
#define DDR_SS DDRB
#define PORT_SS PORTB

void spi_init();

void spi_ss_set(uint8_t state);
void spi_w_byte(uint8_t byte);
uint8_t spi_rw_byte(uint8_t byte);

#endif
