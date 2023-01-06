#include <util/delay.h>

#include "nrf24l01.h"
#include "nrf24l01_map.h"
#include "spi.h"

#include "uart.h"

static void (*nrf_rx_event_callback)(void *buf, uint8_t len);

void register_nrf_rx_event_callback(void (*callback)(void *buf, uint8_t len))
{
  nrf_rx_event_callback = callback;
}

static uint8_t nrf24l01_read_reg(uint8_t reg)
{
  uint8_t cmd = CMD_R_REGISTER | (reg & CMD_REGISTER_MASK);
  CSN_LOW;
  spi_read_write_byte(cmd);
  uint8_t value = spi_read_write_byte(CMD_NOP);
  CSN_HIGH;
  return value;
}

static void nrf24l01_write_reg(uint8_t reg, uint8_t value)
{
  CSN_LOW;
  spi_read_write_byte(CMD_W_REGISTER | (reg & CMD_REGISTER_MASK));
  spi_read_write_byte(value);
  CSN_HIGH;
}

void nrf24l01_set_channel(uint8_t channel)
{
  // Channel can be only values between 0 - 127.
  channel &= 0b01111111;

  nrf24l01_write_reg(REG_RF_CH, channel);
}

void nrf24l01_set_crc_length(uint8_t len)
{
  uint8_t config = nrf24l01_read_reg(REG_CONFIG);
  config &= ~(1 << CRCO);
  config |= (len << CRCO);

  nrf24l01_write_reg(REG_CONFIG, config);
}

void nrf24l01_set_retr_and_delay(uint8_t retr, uint8_t delay)
{
  nrf24l01_write_reg(REG_SETUP_RETR, (delay << ARD) | (retr << ARC));
}

void nrf24l01_set_speed_and_power(uint8_t speed, uint8_t power)
{
  nrf24l01_write_reg(REG_RF_SETUP, (speed << RF_DR_HIGH) | (power << RF_PWR));
}

static void nrf24l01_flush(uint8_t cmd)
{
  CSN_LOW;
  spi_read_write_byte(cmd);
  CSN_HIGH;
}

void nrf24l01_flush_rx()
{
  nrf24l01_flush(CMD_FLUSH_RX);
}

void nrf24l01_flush_tx()
{
  nrf24l01_flush(CMD_FLUSH_TX);
}

static void nrf24l01_clear_status()
{
  nrf24l01_write_reg(REG_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
}

void nrf24l01_set_address_length(uint8_t len)
{
  nrf24l01_write_reg(REG_SETUP_AW, (len << AW));
}

static void nrf24l01_write_long_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
  CSN_LOW;
  spi_read_write_byte(CMD_W_REGISTER | (reg & CMD_REGISTER_MASK));
  while (len--)
    spi_read_write_byte(*buf++);
  CSN_HIGH;
}

void nrf24l01_set_tx_addr(char *address)
{
  uint8_t len = strlen(address);
  nrf24l01_write_long_reg(REG_TX_ADDR, (uint8_t *)address, len > 5 ? 5 : len);
}

void nrf24l01_set_main_rx_addr(char *address)
{
  uint8_t len = strlen(address);
  nrf24l01_write_long_reg(REG_RX_ADDR_P0, (uint8_t *)address, len > 5 ? 5 : len);
}

void nrf24l01_rx_up()
{
  CE_LOW;
  uint8_t config = nrf24l01_read_reg(REG_CONFIG);
  config |= (1 << PWR_UP);
  config |= (1 << PRIM_RX);
  nrf24l01_write_reg(REG_CONFIG, config);
  CE_HIGH;
  _delay_us(140);
}

void nrf24l01_tx_up()
{
  CE_LOW;
  uint8_t config = nrf24l01_read_reg(REG_CONFIG);
  config |= (1 << PWR_UP);
  config &= ~(1 << PRIM_RX);

  nrf24l01_write_reg(REG_CONFIG, config);
  _delay_us(140);
}

void nrf24l01_standby()
{
  uint8_t config = nrf24l01_read_reg(REG_CONFIG);
  config |= (1 << PWR_UP);
  nrf24l01_write_reg(REG_CONFIG, config);
}

void nrf24l01_enable_pipe(uint8_t pipe)
{
  uint8_t en_rxaddr = nrf24l01_read_reg(REG_EN_RXADDR);
  en_rxaddr |= (1 << pipe);

  nrf24l01_write_reg(REG_EN_RXADDR, en_rxaddr);
}

void nrf24l01_enable_dynamic_payload_length(uint8_t pipe)
{
  uint8_t feature = nrf24l01_read_reg(REG_FEATURE);
  feature |= (1 << EN_DPL);
  nrf24l01_write_reg(REG_FEATURE, feature);

  uint8_t dynpd = nrf24l01_read_reg(REG_DYNPD);
  dynpd |= (1 << pipe);
  nrf24l01_write_reg(REG_DYNPD, dynpd);

  uint8_t en_aa = nrf24l01_read_reg(REG_EN_AA);
  en_aa |= (1 << pipe);
  nrf24l01_write_reg(REG_EN_AA, en_aa);
}

void nrf24l01_init(void)
{
  _delay_ms(5);

  DDR_CSN |= (1 << CSN);
  DDR_CE |= (1 << CE);
  CE_LOW;
  CSN_HIGH;

  spi_init();

  nrf24l01_set_channel(10);
  nrf24l01_set_crc_length(CRC_1_BYTE);
  nrf24l01_set_retr_and_delay(RETR_COUNT_10, RETR_TIME_2500uS);
  nrf24l01_set_speed_and_power(SPEED_1mbps, POWER_neg6dBm);
  nrf24l01_set_address_length(ADDRESS_LENGTH_5_BYTES);

  nrf24l01_set_tx_addr("banan");
  nrf24l01_set_main_rx_addr("banan");

  nrf24l01_enable_pipe(ERX_P0);
  nrf24l01_enable_dynamic_payload_length(ERX_P0);

  nrf24l01_standby();
  nrf24l01_clear_status();
  nrf24l01_flush_rx();
  nrf24l01_flush_tx();
}

static void nrf24l01_send_tx_payload(uint8_t *buf, uint8_t len)
{
  CSN_LOW;
  spi_write_byte(CMD_W_TX_PAYLOAD);
  while (len--)
    spi_write_byte(*buf++);
  CSN_HIGH;
}

void nrf24l01_puts(uint8_t *buf, uint8_t len)
{
  nrf24l01_tx_up();

  nrf24l01_send_tx_payload(buf, len);

  CE_HIGH;
  uint8_t status;
  do
  {
    status = nrf24l01_read_reg(REG_STATUS);
  } while (!(status & ((1 << TX_DS) | (1 << MAX_RT))));
  CE_LOW;

  if (status & (1 << MAX_RT))
    nrf24l01_flush_tx();

  nrf24l01_clear_status();
}

static uint8_t nrf24l01_get_payload_length()
{
  CSN_LOW;
  spi_write_byte(CMD_R_RX_PL_WID);
  uint8_t len = spi_read_write_byte(CMD_NOP);
  CSN_HIGH;

  return len;
}

static void nrf24l01_get_payload(uint8_t *buf, uint8_t len)
{
  CSN_LOW;
  spi_write_byte(CMD_R_RX_PAYLOAD);
  while (len--)
    *buf++ = spi_read_write_byte(CMD_NOP);
  CSN_HIGH;
}

void nrf24l01_event()
{
  if (nrf24l01_read_reg(REG_STATUS) & (1 << RX_DR))
  {
    CE_LOW;

    uint8_t buf[32];
    uint8_t fifo_status;

    do
    {
      uint8_t len = nrf24l01_get_payload_length();
      nrf24l01_get_payload(buf, len);

      if (len && nrf_rx_event_callback)
        (*nrf_rx_event_callback)(buf, len);

      fifo_status = nrf24l01_read_reg(REG_FIFO_STATUS);
    } while (!(fifo_status & (1 << RX_EMPTY)));

    nrf24l01_clear_status();
    CE_HIGH;
  }
}