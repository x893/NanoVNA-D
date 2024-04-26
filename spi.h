/*
 * Copyright (c) 2019-2020, Dmitry (DiSlord) dislordlive@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#ifndef __SPI_H__
#define __SPI_H__

#include "ch.h"
#include "hal.h"
#include "nanovna.h"
#include "chprintf.h"

#define LCD_CS_LOW palClearPad(GPIOB, GPIOB_LCD_CS)
#define LCD_CS_HIGH palSetPad(GPIOB, GPIOB_LCD_CS)
// Set SPI bus speed for LCD
#define LCD_SPI_SPEED SPI_BR_DIV2
// SPI bus for LCD
#define LCD_SPI SPI1

#ifdef __USE_DISPLAY_DMA__
// DMA channels for used in LCD SPI bus
#define LCD_DMA_RX DMA1_Channel2 // DMA1 channel 2 use for SPI1 rx
#define LCD_DMA_TX DMA1_Channel3 // DMA1 channel 3 use for SPI1 tx

#define txdmamode (STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY) | STM32_DMA_CR_DIR_M2P)
#define rxdmamode (STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY) | STM32_DMA_CR_DIR_P2M)
#endif

//*****************************************************************************
//********************************** SPI1 bus *********************************
//*****************************************************************************
// STM32 SPI transfer mode:
// in 8 bit mode:
// if you write *(uint8_t*)(&SPI1->DR)  = (uint8_t) data, then data send as << data
// if you write *(uint16_t*)(&SPI1->DR) =(uint16_t) data, then data send as << dataLoByte, after send dataHiByte
// in 16 bit mode
// if you write *(uint16_t*)(&SPI1->DR) =(uint16_t) data, then data send as << data

// SPI init in 8 bit mode
#define SPI_CR2_8BIT 0x0700
#define SPI_CR2_16BIT 0x0F00

//*****************************************************
// SPI bus baud rate (PPL/BR_DIV)
//*****************************************************
#define SPI_BR_DIV2 (0x00000000U)
#define SPI_BR_DIV4 (SPI_CR1_BR_0)
#define SPI_BR_DIV8 (SPI_CR1_BR_1)
#define SPI_BR_DIV16 (SPI_CR1_BR_1 | SPI_CR1_BR_0)
#define SPI_BR_DIV32 (SPI_CR1_BR_2)
#define SPI_BR_DIV64 (SPI_CR1_BR_2 | SPI_CR1_BR_0)
#define SPI_BR_DIV128 (SPI_CR1_BR_2 | SPI_CR1_BR_1)
#define SPI_BR_DIV256 (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)

#define SPI_BR_SET(spi, br) (spi->CR1 = (spi->CR1 & ~(SPI_CR1_BR)) | br)

//*****************************************************
// SPI bus activity macros
//*****************************************************
// The RXNE flag is set depending on the FRXTH bit value in the SPIx_CR2 register:
// • If FRXTH is set, RXNE goes high and stays high until the RXFIFO level is greater or equal to 1/4 (8-bit).
#define SPI_RX_IS_NOT_EMPTY(spi) (spi->SR & SPI_SR_RXNE)
#define SPI_RX_IS_EMPTY(spi) (!(spi->SR & SPI_SR_RXNE))

// The TXE flag is set when transmission TXFIFO has enough space to store data to send.
// 0: Tx buffer not empty, bit is cleared automatically when the TXFIFO level becomes greater than 1/2
// 1: Tx buffer empty, flag goes high and stays high until the TXFIFO level is lower or equal to 1/2 of the FIFO depth
#define SPI_TX_IS_NOT_EMPTY(spi) (!(spi->SR & SPI_SR_TXE))
#define SPI_TX_IS_EMPTY(spi) (spi->SR & SPI_SR_TXE)

// When BSY is set, it indicates that a data transfer is in progress on the SPI (the SPI bus is busy).
#define SPI_IS_BUSY(spi) (spi->SR & SPI_SR_BSY)

// Tx or Rx in process
#define SPI_IN_TX_RX(spi) ((spi->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || SPI_IS_BUSY(spi))

//*****************************************************
// SPI send data macros
//*****************************************************
#define SPI_WRITE_8BIT(spi, data) *(__IO uint8_t *)(&spi->DR) = (uint8_t)data
#define SPI_WRITE_16BIT(spi, data) *(__IO uint16_t *)(&spi->DR) = (uint16_t)data

//*****************************************************
// SPI read data macros
//*****************************************************
#define SPI_READ_8BIT(spi) *(__IO uint8_t *)(&spi->DR)
#define SPI_READ_16BIT(spi) *(__IO uint16_t *)(&spi->DR)

void spi_RxBuffer(uint8_t *buffer, uint16_t len);

void spi_DropRx(void);
uint8_t spi_RxByte(void);
void spi_TxByte(const uint8_t data);
void spi_TxBuffer(const uint8_t *buffer, uint16_t len);

void dmaChannelWaitCompletionRxTx(void);

// SPI transmit byte buffer use DMA (65535 bytes limit)
static inline void spi_DMATxBuffer(const uint8_t *buffer, uint16_t len, bool wait)
{
	dmaChannelSetMemory(LCD_DMA_TX, buffer);
	dmaChannelSetTransactionSize(LCD_DMA_TX, len);
	dmaChannelSetMode(LCD_DMA_TX, txdmamode | STM32_DMA_CR_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_EN);
	if (wait)
		dmaChannelWaitCompletion(LCD_DMA_TX);
}

#endif
