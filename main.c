#include <avr/io.h>

#include "NRF24L01.h"

#define RX 0 // INTEGRATED
#define TX 1 // LEONARDO

#define MODE TX

#if MODE == RX
#include "uart.h"
#endif

void nrf24l01_receive_handler(void *buf, uint8_t len)
{
#if MODE == RX
	uart_putbuf(buf, len, "HANDLER");
#endif
}

int main(void)
{
#if MODE == TX
	DDRC |= (1 << PC7);
	PORTC &= ~(1 << PC7);

#elif MODE == RX
	char buf[128];
	uart_init();
	sei();

	_delay_ms(3000);
	uart_puts("STARTED...\r\n");
#endif

	SPI_init();

	register_nrf_rx_event_callback(nrf24l01_receive_handler);
	NRF_init();

	while (1)
	{

#if MODE == TX
		_delay_ms(1000);
		NRF_puts("HELLO", 5);
		PORTC ^= (1 << PC7);
#elif MODE == RX
		NRF_rx_event(buf);
#endif
	}
}