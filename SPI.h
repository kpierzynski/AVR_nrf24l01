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

void SPI_init();

void SPI_ss_set(uint8_t state);
void SPI_w_byte(uint8_t byte);
uint8_t SPI_rw_byte(uint8_t byte);
void SPI_rw_buffer(uint8_t *rbuf, uint8_t *wbuf, uint8_t len);

#endif
