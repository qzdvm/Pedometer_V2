#ifndef SPI_HAL_H
#define SPI_HAL_H

#include "stm32l0xx_ll_spi.h"

void spi_transmit(SPI_TypeDef *SPIx, const uint8_t *ab_tx_buff, uint8_t b_size);
void spi_transmit_receive(SPI_TypeDef *SPIx, const uint8_t *ab_tx_buff, uint8_t *ab_rx_buff, uint8_t b_size);

#endif
