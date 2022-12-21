/*
 * NRF24L01_register_map_table.h
 *
 *  Created on: 4 sie 2019
 *      Author: kpier
 */

#ifndef NRF24L01_NRF24L01_REGISTER_MAP_TABLE_H_
#define NRF24L01_NRF24L01_REGISTER_MAP_TABLE_H_

//REGISTERS
//-----------------------------------------
#define REG_CONFIG 0x00

#define MASK_RX_DR 6
#define MASK_TX_DS 5
#define MASK_MAX_RT 4
#define EN_CRC 3
#define CRCO 2
#define PWR_UP 1
#define PRIM_RX 0
//-----------------------------------------

//-----------------------------------------
#define REG_EN_AA 0x01

#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0
//-----------------------------------------

//-----------------------------------------
#define REG_EN_RXADDR 0x02

#define ERX_P5 5
#define ERX_P4 4
#define ERX_P3 3
#define ERX_P2 2
#define ERX_P1 1
#define ERX_P0 0
//-----------------------------------------

//-----------------------------------------
#define REG_SETUP_AW 0x03

#define AW 0
//-----------------------------------------

//-----------------------------------------
#define REG_SETUP_RETR 0x04

#define ARD 4
#define ARC 0

#define RETR_TIME_250uS		0x00
#define RETR_TIME_500uS		0x01
#define RETR_TIME_750uS		0x02
#define RETR_TIME_1000uS	0x03
#define RETR_TIME_1250uS	0x04
#define RETR_TIME_1500uS	0x05
#define RETR_TIME_1750uS	0x06
#define RETR_TIME_2000uS	0x07
#define RETR_TIME_2250uS	0x08
#define RETR_TIME_2500uS	0x09
#define RETR_TIME_2750uS	0x0A
#define RETR_TIME_3000uS	0x0B
#define RETR_TIME_3250uS	0x0C
#define RETR_TIME_3500uS	0x0D
#define RETR_TIME_3750uS	0x0E
#define RETR_TIME_4000uS	0x0F

#define RETR_COUNT_DISABLED	0x00
#define RETR_COUNT_1		0x01
#define RETR_COUNT_2		0x02
#define RETR_COUNT_3		0x03
#define RETR_COUNT_4		0x04
#define RETR_COUNT_5		0x05
#define RETR_COUNT_6		0x06
#define RETR_COUNT_7		0x07
#define RETR_COUNT_8		0x08
#define RETR_COUNT_9		0x09
#define RETR_COUNT_10		0x0A
#define RETR_COUNT_11		0x0B
#define RETR_COUNT_12		0x0C
#define RETR_COUNT_13		0x0D
#define RETR_COUNT_14		0x0E
#define RETR_COUNT_15		0x0F
//-----------------------------------------

//-----------------------------------------
#define REG_RF_CH 0x05

#define RF_CH 0
//-----------------------------------------

//-----------------------------------------
#define REG_RF_SETUP 0x06

#define CONT_WAVE 7
#define RF_DR_LOW 5
#define PLL_LOCK 4
#define RF_DR_HIGH 3
#define RF_PWR 1

#define POWER_neg18dBm	0b00
#define POWER_neg12dBm	0b01
#define POWER_neg6dBm	0b10
#define POWER_0dBm		0b11

#define SPEED_1mbps		0b00
#define SPEED_2mbps		0b01
#define SPEED_250kbps	0b10

//-----------------------------------------

//-----------------------------------------
#define REG_STATUS 0x07

#define RX_DR 6
#define TX_DS 5
#define MAX_RT 4
#define RX_P_NO 1
#define TX_FULL_STATUS 0
//-----------------------------------------

//-----------------------------------------
#define REG_OBSERVE_TX 0x08

#define PLOS_CNT 4
#define ARC_CNT 0
//-----------------------------------------

//-----------------------------------------
#define REG_RPD 0x09

#define RPD 0
//-----------------------------------------

//-----------------------------------------
#define REG_RX_ADDR_P0 0x0A
//-----------------------------------------

//-----------------------------------------
#define REG_RX_ADDR_P1 0x0B
//-----------------------------------------

//-----------------------------------------
#define REG_RX_ADDR_P2 0x0C
//-----------------------------------------

//-----------------------------------------
#define REG_RX_ADDR_P3 0x0D
//-----------------------------------------

//-----------------------------------------
#define REG_RX_ADDR_P4 0x0E
//-----------------------------------------

//-----------------------------------------
#define REG_RX_ADDR_P5 0x0F
//-----------------------------------------

//-----------------------------------------
#define REG_TX_ADDR 0x10
//-----------------------------------------

//-----------------------------------------
#define REG_RX_PW_P0 0x11

#define RX_PW_P0 0
//-----------------------------------------

//-----------------------------------------
#define REG_RX_PW_P1 0x12

#define RX_PW_P1 0
//-----------------------------------------

//-----------------------------------------
#define REG_RX_PW_P2 0x13

#define RX_PW_P2 0
//-----------------------------------------

//-----------------------------------------
#define REG_RX_PW_P3 0x14

#define RX_PW_P3 0
//-----------------------------------------

//-----------------------------------------
#define REG_RX_PW_P4 0x15

#define RX_PW_P4 0
//-----------------------------------------

//-----------------------------------------
#define REG_RX_PW_P5 0x16

#define RX_PW_P5 0
//-----------------------------------------

//-----------------------------------------
#define REG_FIFO_STATUS 0x17

#define TX_REUSE 6
#define TX_FULL 5
#define TX_EMPTY 4
#define RX_FULL 1
#define RX_EMPTY 0
//-----------------------------------------

//-----------------------------------------
#define REG_DYNPD 0x1C

#define DPL_P5 5
#define DPL_P4 4
#define DPL_P3 3
#define DPL_P2 2
#define DPL_P1 1
#define DPL_P0 0
//-----------------------------------------

//-----------------------------------------
#define REG_FEATURE 0x1D

#define EN_DPL 2
#define EN_ACK_PAY 1
#define EN_DYN_ACK 0
//-----------------------------------------

//COMANDS
//-----------------------------------------
#define CMD_R_REG				0b00000000
#define CMD_W_REG				0b00100000
#define CMD_R_RX_PAYLOAD		0b01100001
#define CMD_W_TX_PAYLOAD		0b10100000
#define CMD_FLUSH_TX			0b11100001
#define CMD_FLUSH_RX			0b11100010
#define CMD_REUSE_TX_PL			0b11100011
#define CMD_R_RX_PL_WID			0b01100000
#define CMD_W_ACK_PAYLOAD		0b10101000
#define CMD_W_TX_PAYLOAD_NO_ACK	0b10110000
#define CMD_NOP					0b11111111
//-----------------------------------------

#endif /* NRF24L01_NRF24L01_REGISTER_MAP_TABLE_H_ */
