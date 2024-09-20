/*
 * Copyright (c) 2019-2020, Dmitry (DiSlord) dislordlive@gmail.com
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
#include "hal.h"
#include "nanovna.h"

#define AIC3204_ADDR 0x18

#define wait_ms(ms)     chThdSleepMilliseconds(ms)

// Register - 0x01 / 0x34 (P1_R52): Left MICPGA Positive Terminal Input Routing Configuration
#define REG_34_IN1L_TO_LEFT_P_NO    (0<<6)
#define REG_34_IN1L_TO_LEFT_P_10k   (1<<6)
#define REG_34_IN1L_TO_LEFT_P_20k   (2<<6)
#define REG_34_IN1L_TO_LEFT_P_40k   (3<<6)

#define REG_34_IN2L_TO_LEFT_P_NO    (0<<4)
#define REG_34_IN2L_TO_LEFT_P_10k   (1<<4)
#define REG_34_IN2L_TO_LEFT_P_20k   (2<<4)
#define REG_34_IN2L_TO_LEFT_P_40k   (3<<4)

#define REG_34_IN3L_TO_LEFT_P_NO    (0<<2)
#define REG_34_IN3L_TO_LEFT_P_10k   (1<<2)
#define REG_34_IN3L_TO_LEFT_P_20k   (2<<2)
#define REG_34_IN3L_TO_LEFT_P_40k   (3<<2)

#define REG_34_IN1R_TO_LEFT_P_NO    (0<<0)
#define REG_34_IN1R_TO_LEFT_P_10k   (1<<0)
#define REG_34_IN1R_TO_LEFT_P_20k   (2<<0)
#define REG_34_IN1R_TO_LEFT_P_40k   (3<<0)

// Register - 0x01 / 0x36 (P1_R54): Left MICPGA Negative Terminal Input Routing Configuration
#define REG_36_CM1L_TO_LEFT_N_NO    (0<<6)
#define REG_36_CM1L_TO_LEFT_N_10k   (1<<6)
#define REG_36_CM1L_TO_LEFT_N_20k   (2<<6)
#define REG_36_CM1L_TO_LEFT_N_40k   (3<<6)

#define REG_36_IN2R_TO_LEFT_N_NO    (0<<4)
#define REG_36_IN2R_TO_LEFT_N_10k   (1<<4)
#define REG_36_IN2R_TO_LEFT_N_20k   (2<<4)
#define REG_36_IN2R_TO_LEFT_N_40k   (3<<4)

#define REG_36_IN3R_TO_LEFT_N_NO    (0<<2)
#define REG_36_IN3R_TO_LEFT_N_10k   (1<<2)
#define REG_36_IN3R_TO_LEFT_N_20k   (2<<2)
#define REG_36_IN3R_TO_LEFT_N_40k   (3<<2)

#define REG_36_CM2L_TO_LEFT_N_NO    (0<<0)
#define REG_36_CM2L_TO_LEFT_N_10k   (1<<0)
#define REG_36_CM2L_TO_LEFT_N_20k   (2<<0)
#define REG_36_CM2L_TO_LEFT_N_40k   (3<<0)

// Register - 0x01 / 0x37 (P1_R55): Right MICPGA Positive Terminal Input Routing Configuration
#define REG_37_IN1R_TO_RIGHT_P_NO    (0<<6)
#define REG_37_IN1R_TO_RIGHT_P_10k   (1<<6)
#define REG_37_IN1R_TO_RIGHT_P_20k   (2<<6)
#define REG_37_IN1R_TO_RIGHT_P_40k   (3<<6)

#define REG_37_IN2R_TO_RIGHT_P_NO    (0<<4)
#define REG_37_IN2R_TO_RIGHT_P_10k   (1<<4)
#define REG_37_IN2R_TO_RIGHT_P_20k   (2<<4)
#define REG_37_IN2R_TO_RIGHT_P_40k   (3<<4)

#define REG_37_IN3R_TO_RIGHT_P_NO    (0<<2)
#define REG_37_IN3R_TO_RIGHT_P_10k   (1<<2)
#define REG_37_IN3R_TO_RIGHT_P_20k   (2<<2)
#define REG_37_IN3R_TO_RIGHT_P_40k   (3<<2)

#define REG_37_IN2L_TO_RIGHT_P_NO    (0<<0)
#define REG_37_IN2L_TO_RIGHT_P_10k   (1<<0)
#define REG_37_IN2L_TO_RIGHT_P_20k   (2<<0)
#define REG_37_IN2L_TO_RIGHT_P_40k   (3<<0)

// Register - 0x01 / 0x39 (P1_R57): Right MICPGA Negative Terminal Input Routing Configuration
#define REG_39_CM1R_TO_RIGHT_N_NO    (0<<6)
#define REG_39_CM1R_TO_RIGHT_N_10k   (1<<6)
#define REG_39_CM1R_TO_RIGHT_N_20k   (2<<6)
#define REG_39_CM1R_TO_RIGHT_N_40k   (3<<6)

#define REG_39_IN1L_TO_RIGHT_N_NO    (0<<4)
#define REG_39_IN1L_TO_RIGHT_N_10k   (1<<4)
#define REG_39_IN1L_TO_RIGHT_N_20k   (2<<4)
#define REG_39_IN1L_TO_RIGHT_N_40k   (3<<4)

#define REG_39_IN3L_TO_RIGHT_N_NO    (0<<2)
#define REG_39_IN3L_TO_RIGHT_N_10k   (1<<2)
#define REG_39_IN3L_TO_RIGHT_N_20k   (2<<2)
#define REG_39_IN3L_TO_RIGHT_N_40k   (3<<2)

#define REG_39_CM2R_TO_RIGHT_N_NO    (0<<0)
#define REG_39_CM2R_TO_RIGHT_N_10k   (1<<0)
#define REG_39_CM2R_TO_RIGHT_N_20k   (2<<0)
#define REG_39_CM2R_TO_RIGHT_N_40k   (3<<0)

#define REG_27_WCLK_IN               (0<<2)
#define REG_27_WCLK_OUT              (1<<2)
#define REG_27_BCLK_IN               (0<<3)
#define REG_27_BCLK_OUT              (1<<3)
#define REG_27_DATA_16               (0<<4)
#define REG_27_DATA_20               (1<<4)
#define REG_27_DATA_24               (2<<4)
#define REG_27_DATA_32               (3<<4)
#define REG_27_INTERFACE_I2S         (0<<6)
#define REG_27_INTERFACE_DSP         (1<<6)
#define REG_27_INTERFACE_RJF         (2<<6)
#define REG_27_INTERFACE_LJF         (3<<6)

// Set the interface mode: 16 bit, BCLK, WCLK as output, DSP mode
#define REG_27     (REG_27_DATA_16 | REG_27_INTERFACE_DSP | REG_27_WCLK_OUT | REG_27_BCLK_OUT)
#define REG_30(n)  (0x80 + ((n)*sizeof(int16_t)/sizeof(audio_sample_t)))

static const uint8_t conf_data[] = {
// reg, data,     // PLL clock config
  0x00, 0x00,     // Initialize to Page 0
  0x01, 0x01,     // Initialize the device through software reset
//=======================================================
// Configure PLL clock
//            PLL_CLKIN * R * J.D
// PLL_CLK = ---------------------
//                     P
#if AUDIO_CLOCK_REF == 8000000U
  // MCLK = 8.000MHz * 12.2880 = 98.304MHz,
  0x04, 0x43,           // PLL Clock High (92MHz - 137MHz), MCLK pin is input to PLL, PLL as CODEC_CLKIN
  0x05, 0x91,           // Power up PLL, P=1,R=1
  0x06, 12,             // J=12
  0x07, (2880>>8)&0xFF, // D=2880
  0x08, (2880>>0)&0xFF,
#elif AUDIO_CLOCK_REF == 12288000U
  // MCLK = 12.288MHz�* 4 * 2.0 / 1 = 98.304MHz
  0x04, 0x03,     // PLL Clock Low (80MHz - 137MHz),MCLK pin is input to PLL, PLL as CODEC_CLKIN
  0x05, 0x94,     // Power up PLL, P=1,R=4
  0x06, 0x02,     // J=2
  0x07, 0x00,     // D=0
  0x08, 0x00,
#elif AUDIO_CLOCK_REF == 98304000U
  // MCLK = 98.304MHz
  0x04, 0x00,     // MCLK as CODEC_CLKIN
  0x05, 0x00,     // Power down PLL
  0x06, 0x00,     // J=0
  0x07, 0x00,     // D=0
  0x08, 0x00,
#else
#error "Need set correct CODEC_CLKIN for aic3204"
#endif
// Configure ADC clock
//                CODEC_CLKIN
// ADC_fs  = --------------------
//            NADC * MADC * AOSR
#if AUDIO_ADC_FREQ == 12000
  // Clock config, default fs=12kHz
  // from PLL 98.304MHz/(8*8*128) = 12kHz
  0x0b, 0x88,     // Power up the NDAC divider with value 8
  0x0c, 0x88,     // Power up the MDAC divider with value 8
  0x0d, 0x00,     // DAC OSR Setting Register 1 (MSB)  Program the OSR of DAC to 128
  0x0e, 0x80,     // DAC OSR Setting Register 2 (LSB)
  0x3c, 0x01,     // Set the DAC Mode to PRB_P1
// ADC output sample rate depend from DAC, but internal use this settings
  0x12, 0x81,     // Power up the NADC divider with value 1
  0x13, 0x88,     // Power up the MADC divider with value 7
  0x14, 0x80,     // ADC Oversampling (AOSR) Program the OSR of ADC to 128
  0x3d, 0x01,     // Select ADC PRB_R1

  0x1b, REG_27,   // Set the interface mode
  0x1e, REG_30(32),// Enable the BCLKN divider with value 32 (I2S clock = 98.304MHz/(NDAC*32) = 48kHz * (16+16)
#elif AUDIO_ADC_FREQ == 24000
  // Clock config, default fs=24kHz
  // from PLL 98.304MHz/(4*8*128) = 24kHz
  0x0b, 0x84,     // Power up the NDAC divider with value 4
  0x0c, 0x88,     // Power up the MDAC divider with value 8
  0x0d, 0x00,     // DAC OSR Setting Register 1 (MSB)  Program the OSR of DAC to 128
  0x0e, 0x80,     // DAC OSR Setting Register 2 (LSB)
  0x3c, 0x01,     // Set the DAC Mode to PRB_P1
// ADC output sample rate depend from DAC, but internal use this settings
  0x12, 0x81,     // Power up the NADC divider with value 1
  0x13, 0x88,     // Power up the MADC divider with value 7
  0x14, 0x80,     // ADC Oversampling (AOSR) Program the OSR of ADC to 128
  0x3d, 0x01,     // Select ADC PRB_R1

  0x1b, REG_27,   // Set the interface mode
  0x1e, REG_30(32),// Enable the BCLKN divider with value 32 (I2S clock = 98.304MHz/(NDAC*32) = 48kHz * (16+16)
#elif AUDIO_ADC_FREQ == 48000
  // Clock config, default fs=48kHz
  // from PLL 98.304MHz/(2*8*128) = 48kHz
  0x0b, 0x82,     // Power up the NDAC divider with value 2
  0x0c, 0x88,     // Power up the MDAC divider with value 8
  0x0d, 0x00,     // DAC OSR Setting Register 1 (MSB)  Program the OSR of DAC to 128
  0x0e, 0x80,     // DAC OSR Setting Register 2 (LSB)
  0x3c, 0x01,     // Set the DAC Mode to PRB_P1
// ADC output sample rate depend from DAC, but internal use this settings
  0x12, 0x81,     // Power up the NADC divider with value 1
  0x13, 0x88,     // Power up the MADC divider with value 8
  0x14, 0x80,     // ADC Oversampling (AOSR) Program the OSR of ADC to 128
  0x3d, 0x01,     // Select ADC PRB_R1

  0x1b, REG_27,   // Set the interface mode
  0x1e, REG_30(32),// Enable the BCLKN divider with value 32 (I2S clock = 98.304MHz/(NDAC*32) = 48kHz * (16+16)
#elif AUDIO_ADC_FREQ == 96000
  // Clock config, default fs=96kHz
  // from PLL 98.304MHz/(2*8*64) = 96kHz
  0x0b, 0x82,     // Power up the NDAC divider with value 2
  0x0c, 0x88,     // Power up the MDAC divider with value 8
  0x0d, 0x00,     // DAC OSR Setting Register 1 (MSB)  Program the OSR of DAC to 64
  0x0e, 0x40,     // DAC OSR Setting Register 2 (LSB)
  0x3c, 0x01,     // Set the DAC Mode to PRB_P1
// ADC output sample rate depend from DAC, but internal use this settings
  0x12, 0x82,     // Power up the NADC divider with value 2
  0x13, 0x88,     // Power up the MADC divider with value 8
  0x14, 0x80,     // ADC Oversampling (AOSR) set OSR of ADC to 128
  0x3d, 0x01,     // Select ADC PRB_R1 (AOSR = 64 (Use with PRB_R1 to PRB_R12, ADC Filter Type A or B))

  0x1b, REG_27,   // Set the interface mode
  0x1e, REG_30(16),// Enable the BCLKN divider with value 16 (I2S clock = 98.304MHz/(NDAC*16) = 96kHz * (16+16)
#elif AUDIO_ADC_FREQ == 192000
// Clock config, default fs=192kHz
// from PLL 98.304MHz/(2*4*64) = 192kHz
// DAC setting, need only for set ADC sample rate output
  0x0b, 0x82,     // Power up the NDAC divider with value 2
  0x0c, 0x84,     // Power up the MDAC divider with value 4
  0x0d, 0x00,     // DAC OSR Setting Register 1 (MSB)  Program the OSR of DAC to 64
  0x0e, 0x40,     // DAC OSR Setting Register 2 (LSB)
  0x3c,   17,     // Set the DAC Mode to PRB_P17
// ADC output sample rate depend from DAC, but internal use this settings
  0x12, 0x82,     // Power up the NADC divider with value 2
  0x13, 0x84,     // Power up the MADC divider with value 4
  0x14, 0x40,     // ADC Oversampling (AOSR) set OSR of ADC to 64
  0x3d,    7,     // Select ADC PRB_R7

  0x1b, REG_27,   // Set the interface mode
  0x1e, REG_30(8),// Enable the BCLKN divider with value 8 (I2S clock = 98.304MHz/(NDAC*8) = 192kHz * (16+16)
#elif AUDIO_ADC_FREQ == 384000
// Clock config, default fs=384kHz
// from PLL 98.304MHz/(2*4*32) = 384kHz
// DAC setting, need only for set ADC sample rate output
  0x0b, 0x82,     // Power up the NDAC divider with value 2
  0x0c, 0x84,     // Power up the MDAC divider with value 4
  0x0d, 0x00,     // DAC OSR Setting Register 1 (MSB)  Program the OSR of DAC to 32
  0x0e, 0x20,     // DAC OSR Setting Register 2 (LSB)
  0x3c,   17,     // Set the DAC Mode to PRB_P17
// ADC output sample rate depend from DAC, but internal use this settings
  0x12, 0x82,     // Power up the NADC divider with value 2
  0x13, 0x82,     // Power up the MADC divider with value 2
  0x14, 0x40,     // ADC Oversampling (AOSR) set OSR of ADC to 64
  0x3d,    7,     // Select ADC PRB_R7

  0x1b, REG_27,   // Set the interface mode
  0x1e, REG_30(4),// Enable the BCLKN divider with value 4 (I2S clock = 98.304MHz/(NDAC*4) = 384kHz * (16+16)
#elif AUDIO_ADC_FREQ == 768000
// Clock config, default fs=768kHz
// from PLL 98.304MHz/(2*4*16) = 768kHz
// DAC setting, need only for set ADC sample rate output
  0x0b, 0x82,     // Power up the NDAC divider with value 2
  0x0c, 0x82,     // Power up the MDAC divider with value 2
  0x0d, 0x00,     // DAC OSR Setting Register 1 (MSB)  Program the OSR of DAC to 16
  0x0e, 0x20,     // DAC OSR Setting Register 2 (LSB)
  0x3c,   17,     // Set the DAC Mode to PRB_P17
// ADC output sample rate depend from DAC, but internal use this settings
  0x12, 0x82,     // Power up the NADC divider with value 2
  0x13, 0x82,     // Power up the MADC divider with value 2
  0x14, 0x40,     // ADC Oversampling (AOSR) set OSR of ADC to 64
  0x3d,    7,     // Select ADC PRB_R7

  0x1b, REG_27,   // Set the interface mode
  0x1e, REG_30(2),// Enable the BCLKN divider with value 2 (I2S clock = 98.304MHz/(NDAC*2) = 768kHz * (16+16)
#else
#error "Need set correct ADC clock for aic3204"
#endif

// Data routing
  0x00, 0x01,     // Select Page 1
  0x01, 0x08,     // Disable Internal Crude AVdd in presence of external AVdd supply or before powering up internal AVdd LDO
  0x02, 0x01,     // Enable Master Analog Power Control
//  0x14, 0x25,     // HP soft stepping settings for optimal pop performance at power up Rpop used is 6k with N = 6 and soft step = 20usec. This should work with 47uF coupling capacitor. Can try N=5,6 or 7 time constants as well. Trade-off delay vs “pop” sound.
  0x0a, 0x33,     // Set the Input Common Mode to 0.9V and Output Common Mode for Headphone to 1.65V
//  0x0a, 0x40,     // Set the Input Common Mode to 0.75V and Output Common Mode for Headphone to 1.65V

  0x3d, 0x00,     // Select ADC PTM_R4
//  0x3d, 0x64,     // Select ADC PTM_R3
//  0x3d, 0xB6,     // Select ADC PTM_R2
//  0x3d, 0xFF,     // Select ADC PTM_R1

  0x47, 0x32,     // Set MicPGA startup delay to 6.4ms
  0x7b, 0x01,     // Set the REF charging time to 40ms
  0x34, REG_34_IN2L_TO_LEFT_P_10k, // Route IN2L to LEFT_P with 10K
  0x36, REG_36_IN2R_TO_LEFT_N_10k, // Route IN2R to LEFT_N with 10K
//0x37, 0x04,     // Route IN3R to RIGHT_P with 10K
//0x39, 0x04,     // Route IN3L to RIGHT_N with 10K
//0x3b, 0x00,     // Unmute Left MICPGA, Gain selection of 32dB to make channel gain 0dB
//0x3c, 0x00,     // Unmute Right MICPGA, Gain selection of 32dB to make channel gain 0dB
};

static const uint8_t conf_data_unmute[] = {
// reg, data,
  0x00, 0x00,     // Select Page 0
  0x51, 0xc2,     // Power up Left and Right ADC Channels, ADC Volume Control Soft-Stepping disabled
  0x52, 0x00,     // Unmute Left and Right ADC Digital Volume Control
  0x00, 0x01,     // Select Page 1 (should be set as default)
};

static const uint8_t conf_data_ch3_select[] = {
// reg, data,
//0x00,   0x01,                       // Select Page 1 (should be set as default)
  0x37,   REG_37_IN3R_TO_RIGHT_P_10k, // Route IN3R to RIGHT_P with input impedance of 10K
/*0x38,*/ 0x00,                       // Reserved
/*0x39,*/ REG_39_IN3L_TO_RIGHT_N_10k, // Route IN3L to RIGHT_N with input impedance of 10K
/*0x3A,*/ 0b11000000,                 // IN1 is weakly driven to common mode
};

static const uint8_t conf_data_ch1_select[] = {
// reg, data,
//0x00,   0x01,                       // Select Page 1 (should be set as default)
  0x37,   REG_37_IN1R_TO_RIGHT_P_10k, // Route IN1R to RIGHT_P with input impedance of 10K
/*0x38,*/ 0x00,                       // Reserved
/*0x39,*/ REG_39_IN1L_TO_RIGHT_N_10k, // Route IN1L to RIGHT_N with input impedance of 10K
/*0x3A,*/ 0b00001100,                 // IN3 is weakly driven to common mode
};

static void
tlv320aic3204_bulk_write(const uint8_t *buf, int len)
{
  i2c_transfer(AIC3204_ADDR, buf, len);
}

#if 0
static int
tlv320aic3204_read(uint8_t d0)
{
  int addr = AIC3204_ADDR;
  uint8_t buf[] = { d0 };
  i2c_receive(&I2CD1, addr, buf, 1, buf, 1);
  return buf[0];
}
#endif

static void
tlv320aic3204_config(const uint8_t *data, int len)
{
  for (; len--; data += 2)
    tlv320aic3204_bulk_write(data, 2);
}

void tlv320aic3204_init(void)
{
  tlv320aic3204_config(conf_data, sizeof(conf_data)/2);
//  wait_ms(40);
  tlv320aic3204_config(conf_data_unmute, sizeof(conf_data_unmute)/2);
}

void
tlv320aic3204_write_reg(uint8_t page, uint8_t reg, uint8_t data)
{
  uint8_t buf[] = {
   0x00,  page, // Select Page
   reg,   data, // write reg data
   0x00,  0x01  // Select Page 1 (should be set as default)
  };
  tlv320aic3204_config(buf, sizeof(buf)/2);
}

void tlv320aic3204_select(uint8_t channel)
{
#if 0
  // Cache current selected channel
  static uint8_t current_channel = -1;
  if (current_channel == channel)
    return;
  current_channel = channel;
#endif
  tlv320aic3204_bulk_write(channel ? conf_data_ch1_select : conf_data_ch3_select, sizeof(conf_data_ch1_select));
//  tlv320aic3204_config(channel ? conf_data_ch1_select : conf_data_ch3_select, sizeof(conf_data_ch3_select)/2);
}

void tlv320aic3204_set_gain(uint8_t lgain, uint8_t rgain)
{
  uint8_t data[] = {
//  0x00,   0x01,  // Select Page 1 (should be set as default)
    0x3b,   lgain, // Unmute Left MICPGA, set gain
  /*0x3c,*/ rgain  // Unmute Right MICPGA, set gain
  };
//  tlv320aic3204_config(data, sizeof(data)/2);
  tlv320aic3204_bulk_write(data, sizeof(data));
}
