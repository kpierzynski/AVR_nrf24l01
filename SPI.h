#ifndef __SPI_H_
#define __SPI_H_

#include <avr/io.h>

#define SCK		( 1 << PB5 )
#define DDR_SCK		DDRB
#define PORT_SCK	PORTB

#define MOSI		( 1 << PB3 )
#define DDR_MOSI	DDRB
#define PORT_MOSI	PORTB

#define MISO		( 1 << PB4 )
#define DDR_MISO	DDRB
#define PORT_MISO	PORTB

#define SS		( 1 << PB2 )
#define DDR_SS		DDRB
#define PORT_SS		PORTB

void spi_init();
void spi_tx( uint8_t byte );

#endif
