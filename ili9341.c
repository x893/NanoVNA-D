/*
 * Copyright (c) 2019-2023, Dmitry (DiSlord) dislordlive@gmail.com
 * Based on TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
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
#include "spi.h"

// Pin macros for LCD
#define LCD_RESET_ASSERT palClearPad(GPIOA, GPIOA_LCD_RESET)
#define LCD_RESET_NEGATE palSetPad(GPIOA, GPIOA_LCD_RESET)
#define LCD_DC_CMD palClearPad(GPIOB, GPIOB_LCD_CD)
#define LCD_DC_DATA palSetPad(GPIOB, GPIOB_LCD_CD)

// Custom display definition
#ifdef LCD_DRIVER_ILI9341
// Set SPI bus speed for LCD
#define LCD_SPI_SPEED SPI_BR_DIV2
// Read speed, need more slow, not define if need use some as Tx speed
// #define LCD_SPI_RX_SPEED SPI_BR_DIV4
// Allow enable DMA for read display data (can not stable on full speed, on less speed slower)
#define __USE_DISPLAY_DMA_RX__
#endif

#ifdef LCD_DRIVER_ST7796S
// Read speed, need more slow, not define if need use some as Tx speed
#define LCD_SPI_RX_SPEED SPI_BR_DIV4
// Allow enable DMA for read display data
#define __USE_DISPLAY_DMA_RX__
#endif

// Disable DMA rx on disabled DMA tx
#ifndef __USE_DISPLAY_DMA__
#undef __USE_DISPLAY_DMA_RX__
#endif

// LCD display buffer
pixel_t spi_buffer[SPI_BUFFER_SIZE];
// Default foreground & background colors
pixel_t foreground_color = 0;
pixel_t background_color = 0;

//*****************************************************
// SPI functions, settings and data
//*****************************************************
void spi_TxByte(const uint8_t data)
{
	while (SPI_TX_IS_NOT_EMPTY(LCD_SPI))
		;
	SPI_WRITE_8BIT(LCD_SPI, data);
}
// Transmit buffer to SPI bus  (len should be > 0)
void spi_TxBuffer(const uint8_t *buffer, uint16_t len)
{
	while (len--)
	{
		while (SPI_TX_IS_NOT_EMPTY(LCD_SPI))
			;
		SPI_WRITE_8BIT(LCD_SPI, *buffer++);
	}
}

// Receive byte from SPI bus
uint8_t spi_RxByte(void)
{
	// Start RX clock (by sending data)
	SPI_WRITE_8BIT(LCD_SPI, 0xFF);
	while (SPI_RX_IS_EMPTY(LCD_SPI))
		;
	return SPI_READ_8BIT(LCD_SPI);
}

// Receive buffer from SPI bus (len should be > 0)
void spi_RxBuffer(uint8_t *buffer, uint16_t len)
{
	do
	{
		SPI_WRITE_8BIT(LCD_SPI, 0xFF);
		while (SPI_RX_IS_EMPTY(LCD_SPI))
			;
		*buffer++ = SPI_READ_8BIT(LCD_SPI);
	} while (--len);
}

void spi_DropRx(void)
{
	// Drop Rx buffer after tx and wait tx complete
#if 1
	while (SPI_RX_IS_NOT_EMPTY(LCD_SPI) || SPI_IS_BUSY(LCD_SPI))
		(void)SPI_READ_8BIT(LCD_SPI);
	(void)SPI_READ_8BIT(LCD_SPI);
#else
	while (SPI_IS_BUSY(LCD_SPI))
		;
	(void)SPI_READ_16BIT(LCD_SPI);
	(void)SPI_READ_16BIT(LCD_SPI);
#endif
}

//*****************************************************
// SPI DMA settings and data
//*****************************************************
#ifdef __USE_DISPLAY_DMA__

// Wait DMA Rx completion
void dmaChannelWaitCompletionRxTx(void)
{
	dmaChannelWaitCompletion(LCD_DMA_TX);
	dmaChannelWaitCompletion(LCD_DMA_RX);
	//  while (SPI_IS_BUSY(LCD_SPI));   // Wait SPI tx/rx
}

// SPI receive byte buffer use DMA
static const uint16_t dummy_tx = 0xFFFF;
static inline void spi_DMARxBuffer(uint8_t *buffer, uint16_t len, bool wait)
{
	// Init Rx DMA buffer, size, mode (spi and mem data size is 8 bit), and start
	dmaChannelSetMemory(LCD_DMA_RX, buffer);
	dmaChannelSetTransactionSize(LCD_DMA_RX, len);
	dmaChannelSetMode(LCD_DMA_RX, rxdmamode | STM32_DMA_CR_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_EN);
	// Init dummy Tx DMA (for rx clock), size, mode (spi and mem data size is 8 bit), and start
	dmaChannelSetMemory(LCD_DMA_TX, &dummy_tx);
	dmaChannelSetTransactionSize(LCD_DMA_TX, len);
	dmaChannelSetMode(LCD_DMA_TX, txdmamode | STM32_DMA_CR_BYTE | STM32_DMA_CR_EN);
	if (wait)
		dmaChannelWaitCompletionRxTx();
}
#else
// Replace DMA function vs no DMA
#define dmaChannelWaitCompletionRxTx() \
	{                                  \
	}
#define spi_DMATxBuffer(buffer, len) spi_TxBuffer(buffer, len)
#define spi_DMARxBuffer(buffer, len) spi_RxBuffer(buffer, len)
#endif // __USE_DISPLAY_DMA__

static void spi_init(void)
{
	rccEnableSPI1(FALSE);
	LCD_SPI->CR1 = 0;
	LCD_SPI->CR1 = SPI_CR1_MSTR	   // SPI is MASTER
				   | SPI_CR1_SSM   // Software slave management (The external NSS pin is free for other application uses)
				   | SPI_CR1_SSI   // Internal slave select (This bit has an effect only when the SSM bit is set. Allow use NSS pin as I/O)
				   | LCD_SPI_SPEED // Baud rate control
				   | SPI_CR1_CPHA  // Clock Phase
				   | SPI_CR1_CPOL  // Clock Polarity
		;
	LCD_SPI->CR2 = SPI_CR2_8BIT	   // SPI data size, set to 8 bit
				   | SPI_CR2_FRXTH // SPI_SR_RXNE generated every 8 bit data
//            	   | SPI_CR2_SSOE      //
#ifdef __USE_DISPLAY_DMA__
				   | SPI_CR2_TXDMAEN // Tx DMA enable
#ifdef __USE_DISPLAY_DMA_RX__
				   | SPI_CR2_RXDMAEN // Rx DMA enable
#endif
#endif
		;
// Init SPI DMA Peripheral
#ifdef __USE_DISPLAY_DMA__
	dmaChannelSetPeripheral(LCD_DMA_TX, &LCD_SPI->DR); // DMA Peripheral Tx
#ifdef __USE_DISPLAY_DMA_RX__
	dmaChannelSetPeripheral(LCD_DMA_RX, &LCD_SPI->DR); // DMA Peripheral Rx
#endif
#endif
	// Enable DMA on SPI
	LCD_SPI->CR1 |= SPI_CR1_SPE; // SPI enable
}

//*****************************************************
// Display driver functions
//*****************************************************
// Display commands list
#define ILI9341_NOP 0x00
#define ILI9341_SOFTWARE_RESET 0x01
#define ILI9341_READ_IDENTIFICATION 0x04
#define ILI9341_READ_STATUS 0x09
#define ILI9341_READ_POWER_MODE 0x0A
#define ILI9341_READ_MADCTL 0x0B
#define ILI9341_READ_PIXEL_FORMAT 0x0C
#define ILI9341_READ_IMAGE_FORMAT 0x0D
#define ILI9341_READ_SIGNAL_MODE 0x0E
#define ILI9341_READ_SELF_DIAGNOSTIC 0x0F
#define ILI9341_SLEEP_IN 0x10
#define ILI9341_SLEEP_OUT 0x11
#define ILI9341_PARTIAL_MODE_ON 0x12
#define ILI9341_NORMAL_DISPLAY_MODE_ON 0x13
#define ILI9341_INVERSION_OFF 0x20
#define ILI9341_INVERSION_ON 0x21
#define ILI9341_GAMMA_SET 0x26
#define ILI9341_DISPLAY_OFF 0x28
#define ILI9341_DISPLAY_ON 0x29
#define ILI9341_COLUMN_ADDRESS_SET 0x2A
#define ILI9341_PAGE_ADDRESS_SET 0x2B
#define ILI9341_MEMORY_WRITE 0x2C
#define ILI9341_COLOR_SET 0x2D
#define ILI9341_MEMORY_READ 0x2E
#define ILI9341_PARTIAL_AREA 0x30
#define ILI9341_VERTICAL_SCROLLING_DEF 0x33
#define ILI9341_TEARING_LINE_OFF 0x34
#define ILI9341_TEARING_LINE_ON 0x35
#define ILI9341_MEMORY_ACCESS_CONTROL 0x36
#define ILI9341_VERTICAL_SCROLLING 0x37
#define ILI9341_IDLE_MODE_OFF 0x38
#define ILI9341_IDLE_MODE_ON 0x39
#define ILI9341_PIXEL_FORMAT_SET 0x3A
#define ILI9341_WRITE_MEMORY_CONTINUE 0x3C
#define ILI9341_READ_MEMORY_CONTINUE 0x3E
#define ILI9341_SET_TEAR_SCANLINE 0x44
#define ILI9341_GET_SCANLINE 0x45
#define ILI9341_WRITE_BRIGHTNESS 0x51
#define ILI9341_READ_BRIGHTNESS 0x52
#define ILI9341_WRITE_CTRL_DISPLAY 0x53
#define ILI9341_READ_CTRL_DISPLAY 0x54
#define ILI9341_WRITE_CA_BRIGHTNESS 0x55
#define ILI9341_READ_CA_BRIGHTNESS 0x56
#define ILI9341_WRITE_CA_MIN_BRIGHTNESS 0x5E
#define ILI9341_READ_CA_MIN_BRIGHTNESS 0x5F
#define ILI9341_READ_ID1 0xDA
#define ILI9341_READ_ID2 0xDB
#define ILI9341_READ_ID3 0xDC
#define ILI9341_RGB_INTERFACE_CONTROL 0xB0
#define ILI9341_FRAME_RATE_CONTROL_1 0xB1
#define ILI9341_FRAME_RATE_CONTROL_2 0xB2
#define ILI9341_FRAME_RATE_CONTROL_3 0xB3
#define ILI9341_DISPLAY_INVERSION_CONTROL 0xB4
#define ILI9341_BLANKING_PORCH_CONTROL 0xB5
#define ILI9341_DISPLAY_FUNCTION_CONTROL 0xB6
#define ILI9341_ENTRY_MODE_SET 0xB7
#define ILI9341_BACKLIGHT_CONTROL_1 0xB8
#define ILI9341_BACKLIGHT_CONTROL_2 0xB9
#define ILI9341_BACKLIGHT_CONTROL_3 0xBA
#define ILI9341_BACKLIGHT_CONTROL_4 0xBB
#define ILI9341_BACKLIGHT_CONTROL_5 0xBC
#define ILI9341_BACKLIGHT_CONTROL_7 0xBE
#define ILI9341_BACKLIGHT_CONTROL_8 0xBF
#define ILI9341_POWER_CONTROL_1 0xC0
#define ILI9341_POWER_CONTROL_2 0xC1
#define ILI9341_VCOM_CONTROL_1 0xC5
#define ILI9341_VCOM_CONTROL_2 0xC7
#define ILI9341_POWERA 0xCB
#define ILI9341_POWERB 0xCF
#define ILI9341_NV_MEMORY_WRITE 0xD0
#define ILI9341_NV_PROTECTION_KEY 0xD1
#define ILI9341_NV_STATUS_READ 0xD2
#define ILI9341_READ_ID4 0xD3
#define ILI9341_POSITIVE_GAMMA_CORRECTION 0xE0
#define ILI9341_NEGATIVE_GAMMA_CORRECTION 0xE1
#define ILI9341_DIGITAL_GAMMA_CONTROL_1 0xE2
#define ILI9341_DIGITAL_GAMMA_CONTROL_2 0xE3
#define ILI9341_DTCA 0xE8
#define ILI9341_DTCB 0xEA
#define ILI9341_POWER_SEQ 0xED
#define ILI9341_3GAMMA_EN 0xF2
#define ILI9341_INTERFACE_CONTROL 0xF6
#define ILI9341_PUMP_RATIO_CONTROL 0xF7

//
// ILI9341_MEMORY_ACCESS_CONTROL registers
//
#define ILI9341_MADCTL_MY 0x80
#define ILI9341_MADCTL_MX 0x40
#define ILI9341_MADCTL_MV 0x20
#define ILI9341_MADCTL_ML 0x10
#define ILI9341_MADCTL_BGR 0x08
#define ILI9341_MADCTL_MH 0x04
#define ILI9341_MADCTL_RGB 0x00

#define DISPLAY_ROTATION_270 (ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR)
#define DISPLAY_ROTATION_90 (ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR)
#define DISPLAY_ROTATION_0 (ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR)
#define DISPLAY_ROTATION_180 (ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR)

// Disable inline for this function
static void ili9341_send_command(uint8_t cmd, uint16_t len, const uint8_t *data)
{
	// Uncomment on low speed SPI (possible get here before previous tx complete)
	while (SPI_IS_BUSY(LCD_SPI))
		;
	LCD_CS_LOW;
	LCD_DC_CMD;
	SPI_WRITE_8BIT(LCD_SPI, cmd);
	// Need wait transfer complete and set data bit
	while (SPI_IS_BUSY(LCD_SPI))
		;
	LCD_DC_DATA;
	spi_TxBuffer(data, len);
	//  while (SPI_IN_TX_RX(LCD_SPI));
	// LCD_CS_HIGH;
}

// Disable inline for this function
uint32_t lcd_send_command(uint8_t cmd, uint8_t len, const uint8_t *data)
{
	lcd_bulk_finish();
	// Set read speed (if need different)
	SPI_BR_SET(LCD_SPI, SPI_BR_DIV256);
	// Send
	ili9341_send_command(cmd, len, data);

	// Skip data from rx buffer
	spi_DropRx();
	uint32_t ret;
	ret = spi_RxByte();
	ret <<= 8;
	ret |= spi_RxByte();
	ret <<= 8;
	ret |= spi_RxByte();
	ret <<= 8;
	ret |= spi_RxByte();
	LCD_CS_HIGH;
	SPI_BR_SET(LCD_SPI, LCD_SPI_SPEED);
	return ret;
}

#ifdef LCD_DRIVER_ILI9341
static const uint8_t ili9341_init_seq[] = {
	// cmd, len, data...,
	// SW reset
	ILI9341_SOFTWARE_RESET, 0,
	// display off
	ILI9341_DISPLAY_OFF, 0,
	// Power control B
	ILI9341_POWERB, 3, 0x00, 0xC1, 0x30,
	// Power on sequence control
	ILI9341_POWER_SEQ, 4, 0x64, 0x03, 0x12, 0x81,
	// Driver timing control A
	ILI9341_DTCA, 3, 0x85, 0x00, 0x78,
	// Power control A
	ILI9341_POWERA, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
	// Pump ratio control
	ILI9341_PUMP_RATIO_CONTROL, 1, 0x20,
	// Driver timing control B
	ILI9341_DTCB, 2, 0x00, 0x00,
	// POWER_CONTROL_1
	ILI9341_POWER_CONTROL_1, 1, 0x23,
	// POWER_CONTROL_2
	ILI9341_POWER_CONTROL_2, 1, 0x10,
	// VCOM_CONTROL_1
	ILI9341_VCOM_CONTROL_1, 2, 0x3e, 0x28,
	// VCOM_CONTROL_2
	ILI9341_VCOM_CONTROL_2, 1, 0xBE,
	// MEMORY_ACCESS_CONTROL
	// ILI9341_MEMORY_ACCESS_CONTROL, 1, 0x48, // portlait
	ILI9341_MEMORY_ACCESS_CONTROL, 1, DISPLAY_ROTATION_0, // landscape
	// COLMOD_PIXEL_FORMAT_SET : 16 bit pixel
	ILI9341_PIXEL_FORMAT_SET, 1, 0x55,
	// Frame Rate
	ILI9341_FRAME_RATE_CONTROL_1, 2, 0x00, 0x18,
	// Gamma Function Disable
	ILI9341_3GAMMA_EN, 1, 0x00,
	// gamma set for curve 01/2/04/08
	ILI9341_GAMMA_SET, 1, 0x01,
	// positive gamma correction
	ILI9341_POSITIVE_GAMMA_CORRECTION, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
	// negativ gamma correction
	ILI9341_NEGATIVE_GAMMA_CORRECTION, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
	// Column Address Set
	//  ILI9341_COLUMN_ADDRESS_SET, 4, 0x00, 0x00, 0x01, 0x3f, // width 320
	// Page Address Set
	//  ILI9341_PAGE_ADDRESS_SET, 4, 0x00, 0x00, 0x00, 0xef,   // height 240
	// entry mode
	ILI9341_ENTRY_MODE_SET, 1, 0x06,
	// display function control
	ILI9341_DISPLAY_FUNCTION_CONTROL, 3, 0x08, 0x82, 0x27,
	// Interface Control (set WEMODE=0)
	ILI9341_INTERFACE_CONTROL, 3, 0x00, 0x00, 0x00,
	// sleep out
	ILI9341_SLEEP_OUT, 0,
	// display on
	ILI9341_DISPLAY_ON, 0,
	0 // sentinel
};
#define LCD_INIT ili9341_init_seq
#endif

#ifdef LCD_DRIVER_ST7796S
static const uint8_t ST7796S_init_seq[] = {
	// SW reset
	ILI9341_SOFTWARE_RESET, 0,
	// display off
	ILI9341_DISPLAY_OFF, 0,
	// Interface Mode Control
	ILI9341_RGB_INTERFACE_CONTROL, 1, 0x00,
	// Frame Rate
	ILI9341_FRAME_RATE_CONTROL_1, 1, 0xA,
	// Display Inversion Control , 2 Dot
	ILI9341_DISPLAY_INVERSION_CONTROL, 1, 0x02,
	// RGB/MCU Interface Control
	ILI9341_DISPLAY_FUNCTION_CONTROL, 3, 0x02, 0x02, 0x3B,
	// EntryMode
	ILI9341_ENTRY_MODE_SET, 1, 0xC6,
	// Power Control 1
	ILI9341_POWER_CONTROL_1, 2, 0x17, 0x15,
	// Power Control 2
	ILI9341_POWER_CONTROL_2, 1, 0x41,
	// VCOM Control
	// ILI9341_VCOM_CONTROL_1, 3, 0x00, 0x4D, 0x90,
	ILI9341_VCOM_CONTROL_1, 3, 0x00, 0x12, 0x80,
	// Memory Access
	ILI9341_MEMORY_ACCESS_CONTROL, 1, 0x28, // landscape, BGR
											//  ILI9341_MEMORY_ACCESS_CONTROL, 1, 0x20,  // landscape, RGB
	// Interface Pixel Format,	16bpp DPI and DBI and
	ILI9341_PIXEL_FORMAT_SET, 1, 0x55,
	// P-Gamma
	//  ILI9341_POSITIVE_GAMMA_CORRECTION, 15, 0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F,
	// N-Gamma
	//  ILI9341_NEGATIVE_GAMMA_CORRECTION, 15, 0x00, 0X16, 0X19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F,
	// Set Image Func
	//  0xE9, 1, 0x00,
	// Set Brightness to Max
	ILI9341_WRITE_BRIGHTNESS, 1, 0xFF,
	// Adjust Control
	ILI9341_PUMP_RATIO_CONTROL, 4, 0xA9, 0x51, 0x2C, 0x82,
	// Exit Sleep
	ILI9341_SLEEP_OUT, 0x00,
	// display on
	ILI9341_DISPLAY_ON, 0,
	0 // sentinel
};
#define LCD_INIT ST7796S_init_seq
#endif

void lcd_init(void)
{
	spi_init();
	LCD_RESET_ASSERT;
	chThdSleepMilliseconds(10);
	LCD_RESET_NEGATE;
	const uint8_t *p;
	for (p = LCD_INIT; *p;)
	{
		ili9341_send_command(p[0], p[1], &p[2]);
		p += 2 + p[1];
		chThdSleepMilliseconds(2);
	}
	lcd_clear_screen();
}

static void ili9341_setWindow(int x, int y, int w, int h, uint16_t cmd)
{
	// Any LCD exchange start from this
	dmaChannelWaitCompletionRxTx();
	// uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
	// uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
	uint32_t xx = __REV16(x | ((x + w - 1) << 16));
	uint32_t yy = __REV16(y | ((y + h - 1) << 16));
	ili9341_send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
	ili9341_send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t *)&yy);
	ili9341_send_command(cmd, 0, NULL);
}

#ifndef __USE_DISPLAY_DMA__
void lcd_fill(int x, int y, int w, int h)
{
	ili9341_setWindow(x, y, w, h, ILI9341_MEMORY_WRITE);
	uint32_t len = w * h;
	do
	{
		while (SPI_TX_IS_NOT_EMPTY(LCD_SPI))
			;
#if LCD_PIXEL_SIZE == 2
		SPI_WRITE_16BIT(LCD_SPI, background_color);
#else
		SPI_WRITE_8BIT(LCD_SPI, background_color);
#endif
	} while (--len);
#ifdef __REMOTE_DESKTOP__
	if (sweep_mode & SWEEP_REMOTE)
	{
		remote_region_t rd = {"fill\r\n", x, y, w, h};
		send_region(&rd, (uint8_t *)&background_color, sizeof(pixel_t));
	}
#endif
}

void lcd_bulk(int x, int y, int w, int h)
{
	ili9341_setWindow(x, y, w, h, ILI9341_MEMORY_WRITE);
	spi_TxBuffer((uint8_t *)spi_buffer, w * h * sizeof(pixel_t));
#ifdef __REMOTE_DESKTOP__
	if (sweep_mode & SWEEP_REMOTE)
	{
		remote_region_t rd = {"bulk\r\n", x, y, w, h};
		send_region(&rd, (uint8_t *)spi_buffer, w * h * sizeof(pixel_t));
	}
#endif
}

#else
//
// Use DMA for send data
//
#define LCD_DMA_MODE (LCD_PIXEL_SIZE == 2 ? STM32_DMA_CR_HWORD : STM32_DMA_CR_BYTE)
// Fill region by some color
void lcd_fill(int x, int y, int w, int h)
{
	ili9341_setWindow(x, y, w, h, ILI9341_MEMORY_WRITE);
	dmaChannelSetMemory(LCD_DMA_TX, &background_color);
	uint32_t len = w * h, delta;
	while (len)
	{
		delta = len > 0xFFFF ? 0xFFFF : len; // DMA can send only 65535 data in one run
		dmaChannelSetTransactionSize(LCD_DMA_TX, delta);
		dmaChannelSetMode(LCD_DMA_TX, txdmamode | LCD_DMA_MODE | STM32_DMA_CR_EN);
		dmaChannelWaitCompletion(LCD_DMA_TX);
		len -= delta;
	}
#ifdef __REMOTE_DESKTOP__
	if (sweep_mode & SWEEP_REMOTE)
	{
		remote_region_t rd = {"fill\r\n", x, y, w, h};
		send_region(&rd, (uint8_t *)&background_color, sizeof(pixel_t));
	}
#endif
}

static void ili9341_DMA_bulk(int x, int y, int w, int h, pixel_t *buffer)
{
	ili9341_setWindow(x, y, w, h, ILI9341_MEMORY_WRITE);
	dmaChannelSetMemory(LCD_DMA_TX, buffer);
	dmaChannelSetTransactionSize(LCD_DMA_TX, w * h);
	dmaChannelSetMode(LCD_DMA_TX, txdmamode | LCD_DMA_MODE | STM32_DMA_CR_MINC | STM32_DMA_CR_EN);
#ifdef __REMOTE_DESKTOP__
	if (sweep_mode & SWEEP_REMOTE)
	{
		remote_region_t rd = {"bulk\r\n", x, y, w, h};
		send_region(&rd, (uint8_t *)buffer, w * h * sizeof(pixel_t));
	}
#endif
}

// Copy spi_buffer to region, wait completion after
void lcd_bulk(int x, int y, int w, int h)
{
	ili9341_DMA_bulk(x, y, w, h, spi_buffer); // Send data
	dmaChannelWaitCompletion(LCD_DMA_TX);	  // Wait
}

// Used only in double buffer mode
#ifndef lcd_get_cell_buffer
#define LCD_BUFFER_1 0x01
#define LCD_DMA_RUN 0x02
static uint8_t LCD_dma_status = 0;
// Return free buffer for render
pixel_t *lcd_get_cell_buffer(void)
{
	return &spi_buffer[(LCD_dma_status & LCD_BUFFER_1) ? SPI_BUFFER_SIZE / 2 : 0];
}
#endif

// Wait completion before next data send
#ifndef lcd_bulk_finish
void lcd_bulk_finish(void)
{
	dmaChannelWaitCompletion(LCD_DMA_TX); // Wait DMA
										  // while (SPI_IN_TX_RX(LCD_SPI));         // Wait tx
}
#endif

// Copy part of spi_buffer to region, no wait completion after if buffer count !=1
#ifndef lcd_bulk_continue
void lcd_bulk_continue(int x, int y, int w, int h)
{
	lcd_bulk_finish();									 // Wait DMA
	ili9341_DMA_bulk(x, y, w, h, lcd_get_cell_buffer()); // Send new cell data
	LCD_dma_status ^= LCD_BUFFER_1;						 // Switch buffer
}
#endif
#endif

#ifdef LCD_DRIVER_ILI9341
// ILI9341 send data in RGB888 format, need parse it
// Copy ILI9341 screen data to buffer
void lcd_read_memory(int x, int y, int w, int h, uint16_t *out)
{
	uint16_t len = w * h;
	ili9341_setWindow(x, y, w, h, ILI9341_MEMORY_READ);
	// Skip data from rx buffer
	spi_DropRx();
	// Set read speed (if need different)
#ifdef LCD_SPI_RX_SPEED
	SPI_BR_SET(LCD_SPI, LCD_SPI_RX_SPEED);
#endif
	// require 8bit dummy clock
	spi_RxByte();
	// receive pixel data to buffer
#ifndef __USE_DISPLAY_DMA_RX__
	spi_RxBuffer((uint8_t *)out, len * LCD_RX_PIXEL_SIZE);
	// Parse received data to RGB565 format
	uint8_t *rgbbuf = (uint8_t *)out;
	do
	{
		uint8_t r, g, b;
		// read data is always 18bit
		r = rgbbuf[0];
		g = rgbbuf[1];
		b = rgbbuf[2];
		*out++ = RGB565(r, g, b);
		rgbbuf += LCD_RX_PIXEL_SIZE;
	} while (--len);
#else
	// Set data size for DMA read
	len *= LCD_RX_PIXEL_SIZE;
	// Start DMA read, and not wait completion
	spi_DMARxBuffer((uint8_t *)out, len, false);
	// Parse received data to RGB565 format while data receive by DMA
	uint8_t *rgbbuf = (uint8_t *)out;
	do
	{
		uint16_t left = dmaChannelGetTransactionSize(LCD_DMA_RX) + LCD_RX_PIXEL_SIZE; // Get DMA data left
		if (left > len)
			continue; // Next pixel RGB data not ready
		do
		{						   // Process completed by DMA data
			uint8_t r = rgbbuf[0]; // read data is always 18bit in RGB888 format
			uint8_t g = rgbbuf[1];
			uint8_t b = rgbbuf[2];
			*out++ = RGB565(r, g, b);
			rgbbuf += LCD_RX_PIXEL_SIZE;
			len -= LCD_RX_PIXEL_SIZE;
		} while (left < len);
	} while (len);
	dmaChannelWaitCompletionRxTx(); // Wait DMA completion and stop it
#endif
	// restore speed if need
#ifdef LCD_SPI_RX_SPEED
	SPI_BR_SET(LCD_SPI, LCD_SPI_SPEED);
#endif
	LCD_CS_HIGH;
}
#endif

#ifdef LCD_DRIVER_ST7796S
// ST7796S send data in RGB565 format, not need parse it
// Copy ST7796S screen data to buffer
void lcd_read_memory(int x, int y, int w, int h, uint16_t *out)
{
	uint16_t len = w * h;
	ili9341_setWindow(x, y, w, h, ILI9341_MEMORY_READ);
	// Skip data from rx buffer
	spi_DropRx();
	// Set read speed (if need different)
#ifdef LCD_SPI_RX_SPEED
	SPI_BR_SET(LCD_SPI, LCD_SPI_RX_SPEED);
#endif
	// require 8bit dummy clock
	spi_RxByte();
	// receive pixel data to buffer
#ifndef __USE_DISPLAY_DMA_RX__
	spi_RxBuffer((uint8_t *)out, len * 2);
#else
	spi_DMARxBuffer((uint8_t *)out, len * 2, true);
#endif
	// restore speed if need
#ifdef LCD_SPI_RX_SPEED
	SPI_BR_SET(LCD_SPI, LCD_SPI_SPEED);
#endif
	LCD_CS_HIGH;
}
#endif

#if 0
static void lcd_pixel(int x, int y, uint16_t color) {
  ili9341_setWindow(x0, y0, 1, 1, ILI9341_MEMORY_WRITE);
  while (SPI_TX_IS_NOT_EMPTY(LCD_SPI));
  SPI_WRITE_16BIT(LCD_SPI, color);
}
#endif

void lcd_line(int x0, int y0, int x1, int y1)
{
	// Modified Bresenham's line algorithm
	if (x1 < x0)
	{
		SWAP(int, x0, x1);
		SWAP(int, y0, y1);
	} // Need draw from left to right
	int dx = -(x1 - x0), sx = 1;
	int dy = (y1 - y0), sy = 1;
	if (dy < 0)
	{
		dy = -dy;
		sy = -1;
	}
	int err = -((dx + dy) < 0 ? dx : dy) / 2;
	while (1)
	{
		ili9341_setWindow(x0, y0, LCD_WIDTH - x0, 1, ILI9341_MEMORY_WRITE); // prepare send Horizontal line
		while (1)
		{
			while (SPI_TX_IS_NOT_EMPTY(LCD_SPI))
				;
			SPI_WRITE_16BIT(LCD_SPI, foreground_color); // Send color
			if (x0 == x1 && y0 == y1)
				return;
			int e2 = err;
			if (e2 > dx)
			{
				err -= dy;
				x0 += sx;
			}
			if (e2 < dy)
			{
				err -= dx;
				y0 += sy;
				break;
			} // Y coordinate change, next horizontal line
		}
	}
}

void lcd_clear_screen(void)
{
	lcd_fill(0, 0, LCD_WIDTH, LCD_HEIGHT);
}

void lcd_set_foreground(uint16_t fg_idx)
{
	foreground_color = GET_PALTETTE_COLOR(fg_idx);
}

void lcd_set_background(uint16_t bg_idx)
{
	background_color = GET_PALTETTE_COLOR(bg_idx);
}

void lcd_set_colors(uint16_t fg_idx, uint16_t bg_idx)
{
	foreground_color = GET_PALTETTE_COLOR(fg_idx);
	background_color = GET_PALTETTE_COLOR(bg_idx);
}

void lcd_set_flip(bool flip)
{
	dmaChannelWaitCompletionRxTx();
	uint8_t memAcc = flip ? DISPLAY_ROTATION_180 : DISPLAY_ROTATION_0;
	lcd_send_command(ILI9341_MEMORY_ACCESS_CONTROL, 1, &memAcc);
}

void ili9341_set_rotation(uint8_t r)
{
	//  static const uint8_t rotation_const[]={DISPLAY_ROTATION_0, DISPLAY_ROTATION_90,
	//  DISPLAY_ROTATION_180, DISPLAY_ROTATION_270};
	ili9341_send_command(ILI9341_MEMORY_ACCESS_CONTROL, 1, &r);
}

void lcd_blitBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *b)
{
#if 1 // Use this for remote desctop (in this case bulk operation send to remote)
	pixel_t *buf = spi_buffer;
	uint8_t bits = 0;
	for (uint32_t c = 0; c < height; c++)
	{
		for (uint32_t r = 0; r < width; r++)
		{
			if ((r & 7) == 0)
				bits = *b++;
			*buf++ = (0x80 & bits) ? foreground_color : background_color;
			bits <<= 1;
		}
	}
	lcd_bulk(x, y, width, height);
#else
	uint8_t bits = 0;
	ili9341_setWindow(x, y, width, height, ILI9341_MEMORY_WRITE);
	for (uint32_t c = 0; c < height; c++)
	{
		for (uint32_t r = 0; r < width; r++)
		{
			if ((r & 7) == 0)
				bits = *b++;
			while (SPI_TX_IS_NOT_EMPTY(LCD_SPI))
				;
			SPI_WRITE_16BIT(LCD_SPI, (0x80 & bits) ? foreground_color : background_color);
			bits <<= 1;
		}
	}
#endif
}

void lcd_drawchar(uint8_t ch, int x, int y)
{
	lcd_blitBitmap(x, y, FONT_GET_WIDTH(ch), FONT_GET_HEIGHT, FONT_GET_DATA(ch));
}

#ifndef lcd_drawstring
void lcd_drawstring(int16_t x, int16_t y, const char *str)
{
	int x_pos = x;
	while (*str)
	{
		uint8_t ch = *str++;
		if (ch == '\n')
		{
			x = x_pos;
			y += FONT_STR_HEIGHT;
			continue;
		}
		const uint8_t *char_buf = FONT_GET_DATA(ch);
		uint16_t w = FONT_GET_WIDTH(ch);
		lcd_blitBitmap(x, y, w, FONT_GET_HEIGHT, char_buf);
		x += w;
	}
}
#endif

typedef struct
{
	const void *vmt;
	int16_t start_x;
	int16_t start_y;
	int16_t x;
	int16_t y;
	uint16_t state;
} lcdPrintStream;

static void put_normal(lcdPrintStream *ps, uint8_t ch)
{
	if (ch == '\n')
	{
		ps->x = ps->start_x;
		ps->y += FONT_STR_HEIGHT;
		return;
	}
	uint16_t w = FONT_GET_WIDTH(ch);
#if _USE_FONT_ < 3
	lcd_blitBitmap(ps->x, ps->y, w, FONT_GET_HEIGHT, FONT_GET_DATA(ch));
#else
	lcd_blitBitmap(ps->x, ps->y, w < 9 ? 9 : w, FONT_GET_HEIGHT, FONT_GET_DATA(ch));
#endif
	ps->x += w;
}

#if _USE_FONT_ != _USE_SMALL_FONT_
typedef void (*font_put_t)(lcdPrintStream *ps, uint8_t ch);
static font_put_t put_char = put_normal;
static void put_small(lcdPrintStream *ps, uint8_t ch)
{
	if (ch == '\n')
	{
		ps->x = ps->start_x;
		ps->y += sFONT_STR_HEIGHT;
		return;
	}
	uint16_t w = sFONT_GET_WIDTH(ch);
#if _USE_SMALL_FONT_ < 3
	lcd_blitBitmap(ps->x, ps->y, w, sFONT_GET_HEIGHT, sFONT_GET_DATA(ch));
#else
	lcd_blitBitmap(ps->x, ps->y, w < 9 ? 9 : w, sFONT_GET_HEIGHT, sFONT_GET_DATA(ch));
#endif
	ps->x += w;
}
void lcd_set_font(int type) { put_char = type == FONT_SMALL ? put_small : put_normal; }

#else
#define put_char put_normal
#endif

static msg_t lcd_put(void *ip, uint8_t ch)
{
	lcdPrintStream *ps = ip;
	if (ps->state)
	{
		if (ps->state == R_BGCOLOR[0])
			lcd_set_background(ch);
		else if (ps->state == R_FGCOLOR[0])
			lcd_set_foreground(ch);
		ps->state = 0;
		return MSG_OK;
	}
	else if (ch < 0x09)
	{
		ps->state = ch;
		return MSG_OK;
	}
	put_char(ps, ch);
	return MSG_OK;
}

// Simple print in buffer function
int lcd_printf(int16_t x, int16_t y, const char *fmt, ...)
{
	// Init small lcd print stream
	struct lcd_printStreamVMT
	{
		_base_sequential_stream_methods
	} lcd_vmt = {NULL, NULL, lcd_put, NULL};
	lcdPrintStream ps = {&lcd_vmt, x, y, x, y, 0};
	// Performing the print operation using the common code.
	va_list ap;
	va_start(ap, fmt);
	int retval = chvprintf((BaseSequentialStream *)(void *)&ps, fmt, ap);
	va_end(ap);
	// Return number of bytes that would have been written.
	return retval;
}

int lcd_printfV(int16_t x, int16_t y, const char *fmt, ...)
{
	// Init small lcd print stream
	struct lcd_printStreamVMT
	{
		_base_sequential_stream_methods
	} lcd_vmt = {NULL, NULL, lcd_put, NULL};
	lcdPrintStream ps = {&lcd_vmt, x, y, x, y, 0};
	lcd_set_foreground(LCD_FG_COLOR);
	lcd_set_background(LCD_BG_COLOR);
	ili9341_set_rotation(DISPLAY_ROTATION_270);
	// Performing the print operation using the common code.
	va_list ap;
	va_start(ap, fmt);
	int retval = chvprintf((BaseSequentialStream *)(void *)&ps, fmt, ap);
	va_end(ap);
	ili9341_set_rotation(DISPLAY_ROTATION_0);
	// Return number of bytes that would have been written.
	return retval;
}

void lcd_blitBitmapScale(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t size, const uint8_t *b)
{
	ili9341_setWindow(x, y, w * size, h * size, ILI9341_MEMORY_WRITE);
	for (int c = 0; c < h; c++)
	{
		const uint8_t *ptr = b;
		uint8_t bits = 0;
		for (int i = 0; i < size; i++)
		{
			ptr = b;
			for (int r = 0; r < w; r++, bits <<= 1)
			{
				if ((r & 7) == 0)
					bits = *ptr++;
				for (int j = 0; j < size; j++)
				{
					while (SPI_TX_IS_NOT_EMPTY(LCD_SPI))
						;
					SPI_WRITE_16BIT(LCD_SPI, (0x80 & bits) ? foreground_color : background_color);
				}
			}
		}
		b = ptr;
	}
}

int lcd_drawchar_size(uint8_t ch, int x, int y, uint8_t size)
{
	const uint8_t *char_buf = FONT_GET_DATA(ch);
	uint16_t w = FONT_GET_WIDTH(ch);
#if 1
	pixel_t *buf = spi_buffer;
	for (uint32_t c = 0; c < FONT_GET_HEIGHT; c++, char_buf++)
	{
		for (uint32_t i = 0; i < size; i++)
		{
			uint8_t bits = *char_buf;
			for (uint32_t r = 0; r < w; r++, bits <<= 1)
				for (uint32_t j = 0; j < size; j++)
					*buf++ = (0x80 & bits) ? foreground_color : background_color;
		}
	}
	lcd_bulk(x, y, w * size, FONT_GET_HEIGHT * size);
#else
	ili9341_setWindow(x, y, w * size, FONT_GET_HEIGHT * size, ILI9341_MEMORY_WRITE);
	for (int c = 0; c < FONT_GET_HEIGHT; c++, char_buf++)
	{
		for (int i = 0; i < size; i++)
		{
			uint8_t bits = *char_buf;
			for (int r = 0; r < w; r++, bits <<= 1)
				for (int j = 0; j < size; j++)
				{
					while (SPI_TX_IS_NOT_EMPTY(LCD_SPI))
						;
					SPI_WRITE_16BIT(LCD_SPI, (0x80 & bits) ? foreground_color : background_color);
				}
		}
	}
#endif
	return w * size;
}

void lcd_drawfont(uint8_t ch, int x, int y)
{
	lcd_blitBitmap(x, y, NUM_FONT_GET_WIDTH, NUM_FONT_GET_HEIGHT, NUM_FONT_GET_DATA(ch));
}

void lcd_drawstring_size(const char *str, int x, int y, uint8_t size)
{
	while (*str)
		x += lcd_drawchar_size(*str++, x, y, size);
}

void lcd_vector_draw(int x, int y, const vector_data *v)
{
	while (v->shift_x || v->shift_y)
	{
		int x1 = x + (int)v->shift_x;
		int y1 = y + (int)v->shift_y;
		if (!v->transparent)
			lcd_line(x, y, x1, y1);
		x = x1;
		y = y1;
		v++;
	}
}

#if 0
static const uint16_t colormap[] = {
  RGBHEX(0x00ff00), RGBHEX(0x0000ff), RGBHEX(0xff0000),
  RGBHEX(0x00ffff), RGBHEX(0xff00ff), RGBHEX(0xffff00)
};

void ili9341_test(int mode) {
  int x, y;
  int i;
  switch (mode) {
    default:
#if 1
    lcd_fill(0, 0, LCD_WIDTH, LCD_HEIGHT, 0);
    for (y = 0; y < LCD_HEIGHT; y++) {
      lcd_fill(0, y, LCD_WIDTH, 1, RGB(LCD_HEIGHT-y, y, (y + 120) % 256));
    }
    break;
    case 1:
      lcd_fill(0, 0, LCD_WIDTH, LCD_HEIGHT, 0);
      for (y = 0; y < LCD_HEIGHT; y++) {
        for (x = 0; x < LCD_WIDTH; x++) {
          ili9341_pixel(x, y, (y<<8)|x);
        }
      }
      break;
    case 2:
      //ili9341_send_command(0x55, 0xff00);
      ili9341_pixel(64, 64, 0xaa55);
    break;
#endif
#if 1
    case 3:
      for (i = 0; i < 10; i++)
        lcd_drawfont(i, i*20, 120);
    break;
#endif
#if 0
    case 4:
      draw_grid(10, 8, 29, 29, 15, 0, 0xffff, 0);
    break;
#endif
    case 4:
      lcd_line(0, 0, 15, 100);
      lcd_line(0, 0, 100, 100);
      lcd_line(0, 15, 100, 0);
      lcd_line(0, 100, 100, 0);
    break;
  }
}
#endif
