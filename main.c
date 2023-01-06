#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include "uart.h"
#include "spi.h"
#include "nrf24l01.h"

void rx(void *buf, uint8_t len)
{
	char *data = buf;

	uart_puts("len: ");
	uart_putd(len);
	uart_puts(" : ");
	for (uint8_t i = 0; i < len; i++)
		uart_putc(data[i]);
	uart_puts("\r\n");
}

int main(void)
{
	_delay_ms(5000);

	uart_init();
	sei();
	uart_puts("starting....\r\n");

	// register_nrf_rx_event_callback(rx);
	nrf24l01_init();
	// nrf24l01_rx_up();

	uart_puts("running...\r\n");
	uint8_t buf[5] = {'c', 'n', 't', ':', '0'};
	uint8_t len = 1;
	while (1)
	{
		// nrf24l01_event();
		_delay_ms(1000);
		nrf24l01_puts(buf, len++);

		buf[4] = (((buf[4] - '0') + 1) % 10) + '0';
		if (len > 5)
			len = 1;
	}

	return 0;
}