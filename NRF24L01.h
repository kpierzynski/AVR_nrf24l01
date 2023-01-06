#ifndef NRF24L01_H
#define NRF24L01_H

#include <avr/io.h>
#include <string.h>

#include "nrf24l01_map.h"

#define CSN SS
#define DDR_CSN DDR_SS
#define PORT_CSN PORT_SS
#define PIN_CSN PIN_SS

#define CE PB2
#define DDR_CE DDRB
#define PORT_CE PORTB
#define PIN_CE PINB

#define CSN_LOW PORT_CSN &= ~(1 << CSN)
#define CSN_HIGH PORT_CSN |= (1 << CSN)

#define CE_LOW PORT_CE &= ~(1 << CE)
#define CE_HIGH PORT_CE |= (1 << CE)

void nrf24l01_init(void);

void nrf24l01_set_channel(uint8_t channel);
void nrf24l01_set_crc_length(uint8_t len);
void nrf24l01_set_retr_and_delay(uint8_t retr, uint8_t delay);
void nrf24l01_set_speed_and_power(uint8_t speed, uint8_t power);
void nrf24l01_set_address_length(uint8_t len);

void nrf24l01_set_tx_addr(char *address);
void nrf24l01_set_main_rx_addr(char *address);

void nrf24l01_enable_pipe(uint8_t pipe);
void nrf24l01_enable_dynamic_payload_length(uint8_t pipe);

void nrf24l01_rx_up();
void nrf24l01_tx_up();
void nrf24l01_standby();

void nrf24l01_flush_rx();
void nrf24l01_flush_tx();

void nrf24l01_puts(uint8_t *buf, uint8_t len);
void nrf24l01_event();

void register_nrf_rx_event_callback(void (*callback)(void *buf, uint8_t len));

#endif
