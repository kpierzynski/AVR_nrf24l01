#include "NRF24L01.h"

static void (*NRF_rx_event_callback)(void *buf, uint8_t len);

void register_nrf_rx_event_callback(void (*callback)(void *buf, uint8_t len))
{
	NRF_rx_event_callback = callback;
}

void NRF_init()
{
	_delay_ms(5);
	CE_DDR |= CE;
	CE_LOW;

	NRF_set_tx_address("addr5");
	char *addr[] = {"addr5", "konra", "b", "c", "d", "e"};
	NRF_set_rx_address(addr);

	uint8_t config = NRF_read_reg(REG_CONFIG);
	NRF_write_reg(REG_CONFIG, config | (1 << MASK_RX_DR) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));

	NRF_set_crc_bytes(CRC_1_BYTE);
	NRF_set_channel(13);
	NRF_set_datapipe(ERX_P0, PIPE_ENABLE, ACK_ENABLE);
	NRF_set_retr(RETR_TIME_4000uS, RETR_COUNT_15);
	NRF_set_speed_and_power(SPEED_250kbps, POWER_0dBm);
	NRF_set_dyn_payload(DYNPL_ENABLE);
	NRF_set_dyn_payload_on_pipe(DPL_P0, DYNPL_ENABLE);
	NRF_rx_flush();
	NRF_tx_flush();

	NRF_write_reg(REG_STATUS, (1 << MAX_RT) | (1 << TX_DS) | (1 << RX_DR));

	NRF_rx_up();
	_delay_ms(5);
}

void NRF_puts(char *buf, uint8_t len)
{

	uint8_t i;
	char *send = buf;

	NRF_tx_up();

	SPI_ss_set(SPI_SS_LOW);
	SPI_w_byte(CMD_W_TX_PAYLOAD);
	for (i = 0; i < len; i++)
	{
		SPI_w_byte(*send++);
	}
	SPI_w_byte(0);
	SPI_ss_set(SPI_SS_HIGH);

	CE_HIGH;

	uint8_t status;

	do
	{
		status = NRF_read_reg(REG_STATUS);
	} while (!((status & (1 << TX_DS)) || (status & (1 << MAX_RT))));

	CE_LOW;

	if (status & (1 << MAX_RT))
	{
		NRF_write_reg(REG_STATUS, status | (1 << MAX_RT) | (1 << TX_DS));
		NRF_tx_flush();
	}

	if (status & (1 << TX_DS))
	{
		NRF_write_reg(REG_STATUS, status | (1 << TX_DS));
	}

	NRF_rx_up();
}

void NRF_rx_event(char *buf)
{

	if (NRF_read_reg(REG_STATUS) & (1 << RX_DR))
	{

		uint8_t len, i;
		uint8_t *pnt;
		uint8_t fifo_status;

		CE_LOW;

		do
		{
			SPI_ss_set(SPI_SS_LOW);
			SPI_w_byte(CMD_R_RX_PL_WID);
			len = SPI_rw_byte(CMD_NOP);
			i = len;
			SPI_ss_set(SPI_SS_HIGH);

			pnt = (uint8_t *)buf;

			SPI_ss_set(SPI_SS_LOW);
			SPI_w_byte(CMD_R_RX_PAYLOAD);
			while (i--)
			{
				*pnt++ = SPI_rw_byte(CMD_NOP);
			}
			SPI_ss_set(SPI_SS_HIGH);

			if (len && NRF_rx_event_callback)
				(*NRF_rx_event_callback)(buf, len);

			fifo_status = NRF_read_reg(REG_FIFO_STATUS);
		} while (!(fifo_status & (1 << RX_EMPTY)));

		uint8_t status = NRF_read_reg(REG_STATUS);
		NRF_write_reg(REG_STATUS, status | (1 << RX_DR));

		CE_HIGH;
	}
}

void NRF_tx_up()
{

	CE_LOW;

	uint8_t config = NRF_read_reg(REG_CONFIG);
	config &= ~(1 << PRIM_RX);
	config |= (1 << PWR_UP);

	NRF_write_reg(REG_CONFIG, config);

	_delay_us(140);
}

void NRF_rx_up()
{

	CE_LOW;

	uint8_t config = NRF_read_reg(REG_CONFIG);

	NRF_write_reg(REG_CONFIG, config | (1 << PWR_UP) | (1 << PRIM_RX));

	CE_HIGH;

	_delay_us(140);
}

uint8_t NRF_read_reg(uint8_t reg)
{

	uint8_t byte;

	SPI_ss_set(SPI_SS_LOW);
	SPI_w_byte(CMD_R_REG | reg);
	byte = SPI_rw_byte(CMD_NOP);
	SPI_ss_set(SPI_SS_HIGH);

	return byte;
}

void NRF_read_reg_to_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{

	SPI_ss_set(SPI_SS_LOW);
	SPI_w_byte(CMD_R_REG | reg);
	SPI_rw_buffer(buf, buf, len);
	SPI_ss_set(SPI_SS_HIGH);
}

void NRF_write_reg(uint8_t reg, uint8_t data)
{

	SPI_ss_set(SPI_SS_LOW);
	SPI_w_byte(CMD_W_REG | reg);
	SPI_w_byte(data);
	SPI_ss_set(SPI_SS_HIGH);
}

void NRF_write_reg_from_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{

	SPI_ss_set(SPI_SS_LOW);
	SPI_w_byte(CMD_W_REG | reg);
	SPI_rw_buffer(buf, buf, len);
	SPI_ss_set(SPI_SS_HIGH);
}

void NRF_set_tx_address(char *address)
{

	NRF_write_reg(REG_SETUP_AW, ((0b11) << AW));

	uint8_t addr[5];
	memcpy(addr, address, 5);

	NRF_write_reg_from_buf(REG_TX_ADDR, addr, 5);
}

void NRF_set_rx_address(char *address[])
{

	NRF_write_reg(REG_SETUP_AW, ((0b11) << AW));

	NRF_write_reg_from_buf(REG_RX_ADDR_P0, (uint8_t *)address[0], 5);
	NRF_write_reg_from_buf(REG_RX_ADDR_P1, (uint8_t *)address[1], 5);
	NRF_write_reg(REG_RX_ADDR_P2, (uint8_t)address[2][0]);
	NRF_write_reg(REG_RX_ADDR_P3, (uint8_t)address[3][0]);
	NRF_write_reg(REG_RX_ADDR_P4, (uint8_t)address[4][0]);
	NRF_write_reg(REG_RX_ADDR_P5, (uint8_t)address[5][0]);
}

void NRF_tx_flush()
{

	SPI_ss_set(SPI_SS_LOW);
	SPI_w_byte(CMD_FLUSH_TX);
	SPI_ss_set(SPI_SS_HIGH);
}

void NRF_rx_flush()
{

	SPI_ss_set(SPI_SS_LOW);
	SPI_w_byte(CMD_FLUSH_RX);
	SPI_ss_set(SPI_SS_HIGH);
}

void NRF_set_crc_bytes(uint8_t amount)
{

	uint8_t config = NRF_read_reg(REG_CONFIG);
	NRF_write_reg(REG_CONFIG, (config & ~(1 << CRCO)) | (amount << CRCO));
}

void NRF_set_channel(uint8_t channel)
{
	NRF_write_reg(REG_RF_CH, 0b01111111 & channel);
}

void NRF_set_datapipe(uint8_t datapipe, uint8_t pipe_state, uint8_t ack_state)
{

	uint8_t en_aa = NRF_read_reg(REG_EN_AA);
	uint8_t en_rxaddr = NRF_read_reg(REG_EN_RXADDR);

	NRF_write_reg(REG_EN_RXADDR, (en_rxaddr & ~(1 << datapipe)) | (pipe_state << datapipe));
	NRF_write_reg(REG_EN_AA, (en_aa & ~(1 << datapipe)) | (ack_state << datapipe));
}

void NRF_set_retr(uint8_t time, uint8_t amount)
{
	NRF_write_reg(REG_SETUP_RETR, (time << ARD) | (amount << ARC));
}

void NRF_set_speed_and_power(uint8_t speed, uint8_t power)
{

	uint8_t rf_setup = NRF_read_reg(REG_RF_SETUP);

	rf_setup = (rf_setup & ~(0b11 << RF_PWR)) | (power << RF_PWR);
	rf_setup = (rf_setup & ~(1 << RF_DR_LOW)) | ((speed >> 1) << RF_DR_LOW);
	rf_setup = (rf_setup & ~(1 << RF_DR_HIGH)) | ((speed & 0b1) << RF_DR_HIGH);

	NRF_write_reg(REG_RF_SETUP, rf_setup);
}

void NRF_set_dyn_payload_on_pipe(uint8_t datapipe, uint8_t state)
{

	uint8_t dynpd = NRF_read_reg(REG_DYNPD);

	NRF_write_reg(REG_DYNPD, (dynpd & ~(1 << datapipe)) | (state << datapipe));
}

void NRF_set_dyn_payload(uint8_t state)
{

	uint8_t feature = NRF_read_reg(REG_FEATURE);

	NRF_write_reg(REG_FEATURE, (feature & ~(1 << EN_DPL)) | (state << EN_DPL));
}