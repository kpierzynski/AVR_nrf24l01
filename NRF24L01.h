/*
 * NRF24L01.h
 *
 *  Created on: 4 sie 2019
 *      Author: kpier
 */

#ifndef NRF24L01_NRF24L01_H_
#define NRF24L01_NRF24L01_H_

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "NRF24L01_register_map_table.h"

#define CE_DDR DDRB
#define CE_PORT PORTB
#define CE_PIN PINB
#define CE PB1

#define CE_LOW CE_PORT &= ~(1<<CE);
#define CE_HIGH CE_PORT |= (1<<CE);

#define CRC_1_BYTE 0
#define CRC_2_BYTES 1

#define ACK_DISABLE 0
#define ACK_ENABLE 1

#define PIPE_DISABLE 0
#define PIPE_ENABLE 1

#define DYNPL_DISABLE 0
#define DYNPL_ENABLE 1

void register_nrf_rx_event_callback( void (*callback)( void * buf, uint8_t len ) );

void NRF_init();
void NRF_puts( char * buf, uint8_t len );
void NRF_rx_event( char * buf );

void NRF_set_tx_address( char * address );
void NRF_set_rx_address( char * address[] );

void NRF_set_crc_bytes( uint8_t amount );
void NRF_set_channel( uint8_t channel );
void NRF_set_datapipe( uint8_t datapipe, uint8_t pipe_state, uint8_t ack_state );
void NRF_set_retr( uint8_t time, uint8_t amount );
void NRF_set_speed_and_power( uint8_t speed, uint8_t power );
void NRF_set_dyn_payload_on_pipe( uint8_t datapipe, uint8_t state );
void NRF_set_dyn_payload( uint8_t state );

#endif /* NRF24L01_NRF24L01_H_ */
