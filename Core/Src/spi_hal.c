#include "spi_hal.h"
#include "stdbool.h"

void spi_transmit(SPI_TypeDef *SPIx, const uint8_t *ab_tx_buff, uint8_t b_size)
{
	uint8_t tx_cnt = b_size;
	uint8_t initial_txcnt = b_size;
	
	if(initial_txcnt == 1U)
	{
		SPIx->DR = *ab_tx_buff;
		ab_tx_buff++;
		tx_cnt--;
	}
	while(tx_cnt > 0U)
	{
		if((READ_BIT(SPI1->SR, SPI_SR_TXE) == (SPI_SR_TXE)))
		{
			SPIx->DR = *ab_tx_buff;
			ab_tx_buff++;
			tx_cnt--;
		}
	}
	
	while(READ_BIT(SPIx->SR, SPI_SR_BSY) == (SPI_SR_BSY))
	{
	};
	
	LL_SPI_ClearFlag_OVR(SPIx);
}

void spi_transmit_receive(SPI_TypeDef *SPIx, const uint8_t *ab_tx_buff, uint8_t *ab_rx_buff, uint8_t b_size)
{	
	bool o_tx_allowed = true;
	uint8_t rx_cnt = b_size;
	uint8_t tx_cnt = b_size;
	uint8_t initial_txcnt = b_size;
	
	if(initial_txcnt == 1U)
	{
		SPIx->DR = *ab_tx_buff;
		ab_tx_buff++;
		tx_cnt--;
	}
		
	while ((tx_cnt > 0U) || (rx_cnt > 0U))
	{
		
		if ((READ_BIT(SPI1->SR, SPI_SR_TXE) == (SPI_SR_TXE)) && (tx_cnt > 0U) && (o_tx_allowed == true))
		{
			SPIx->DR = *ab_tx_buff;
			ab_tx_buff++;
			tx_cnt--;
			o_tx_allowed = false;
		}
		if ((READ_BIT(SPIx->SR, SPI_SR_RXNE) == (SPI_SR_RXNE)) && (rx_cnt > 0U))
		{
			*ab_rx_buff = SPIx->DR;
			ab_rx_buff++;
      rx_cnt--;
			o_tx_allowed = true;
		}
	}
	
	while(READ_BIT(SPIx->SR, SPI_SR_BSY) == (SPI_SR_BSY))
	{
	};
	
	LL_SPI_ClearFlag_OVR(SPIx);
	
}