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

#define SI5351_REG_3_OUTPUT_ENABLE_CONTROL  3
#define SI5351_CLK0_EN     (1<<0)
#define SI5351_CLK1_EN     (1<<1)
#define SI5351_CLK2_EN     (1<<2)

// Reg 16-18 CLKX_CONTROL
#define SI5351_REG_16_CLK0_CONTROL  16
#define SI5351_REG_17_CLK1_CONTROL  17
#define SI5351_REG_18_CLK2_CONTROL  18
#define SI5351_CLK_POWERDOWN                (1<<7)
#define SI5351_CLK_INTEGER_MODE             (1<<6)
#define SI5351_CLK_PLL_SELECT_A             (0<<5)
#define SI5351_CLK_PLL_SELECT_B             (1<<5)
#define SI5351_CLK_INVERT                   (1<<4)
#define SI5351_CLK_INPUT_MASK               (3<<2)
#define SI5351_CLK_INPUT_XTAL               (0<<2)
#define SI5351_CLK_INPUT_CLKIN              (1<<2)
#define SI5351_CLK_INPUT_MULTISYNTH_0_4     (2<<2)
#define SI5351_CLK_INPUT_MULTISYNTH_N       (3<<2)
#define SI5351_CLK_DRIVE_STRENGTH_MASK      (3<<0)
#define SI5351_CLK_DRIVE_STRENGTH_2MA       (0<<0)
#define SI5351_CLK_DRIVE_STRENGTH_4MA       (1<<0)
#define SI5351_CLK_DRIVE_STRENGTH_6MA       (2<<0)
#define SI5351_CLK_DRIVE_STRENGTH_8MA       (3<<0)
#define SI5351_CLK_DRIVE_STRENGTH_AUTO       0xFF

#define SI5351_REG_PLL_A            26
#define SI5351_REG_PLL_B            34

#define SI5351_REG_42_MULTISYNTH0   42
#define SI5351_REG_50_MULTISYNTH1   50
#define SI5351_REG_58_MULTISYNTH2   58
#define SI5351_DIVBY4       (3<<2)
#define SI5351_R_DIV_1      (0<<4)
#define SI5351_R_DIV_2      (1<<4)
#define SI5351_R_DIV_4      (2<<4)
#define SI5351_R_DIV_8      (3<<4)
#define SI5351_R_DIV_16     (4<<4)
#define SI5351_R_DIV_32     (5<<4)
#define SI5351_R_DIV_64     (6<<4)
#define SI5351_R_DIV_128    (7<<4)
#define SI5351_R_DIV(n)     ((n)<<4)

#define SI5351_REG_177_PLL_RESET    177
#define SI5351_PLL_RESET_B          (1<<7)
#define SI5351_PLL_RESET_A          (1<<5)

#define SI5351_REG_183_CRYSTAL_LOAD 183
#define SI5351_CRYSTAL_LOAD__PF     (0<<6)
#define SI5351_CRYSTAL_LOAD_6PF     (1<<6)
#define SI5351_CRYSTAL_LOAD_8PF     (2<<6)
#define SI5351_CRYSTAL_LOAD_10PF    (3<<6)

void si5351_init(void);
void si5351_disable_output(void);
void si5351_enable_output(void);

void si5351_set_frequency_offset(int32_t offset);
int  si5351_set_frequency(uint32_t freq, uint8_t drive_strength);
void si5351_set_power(uint8_t drive_strength);
void si5351_set_band_mode(uint16_t t);

// Defug use functions
void si5351_bulk_write(const uint8_t *buf, int len);
void si5351_set_timing(int i, int v);
void si5351_update_band_config(int idx, uint32_t pidx, uint32_t v);
void si5351_set_tcxo(uint32_t xtal);

// Get info functions
uint32_t si5351_get_frequency(void);
uint32_t si5351_get_harmonic_lvl(uint32_t f);
