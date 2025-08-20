/***************************************************************************//**
* \file cy_gpif_header_lvds.h
* \version 1.0
*
* GPIF tool generated header file
*
*******************************************************************************
* \copyright
* (c) (2021-2024), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef _CY_GPIF_HEADER_LVDS_H_
#define _CY_GPIF_HEADER_LVDS_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include "cy_lvds.h"

/* Summary
   Initial value of early outputs from the state machine */
#define ALPHA_START 0x00
#define START       0x00

const cy_stc_lvds_gpif_wavedata_t cy_lvds_gpif0_wavedata[] = {
    {{0x2E4F4701,0x0040010A,0x80000000,0x00000005},{0x00000000,0x00000000,0x00000000,0x00000000}},
    {{0x3E734702,0x20400000,0x80000000,0x00000001},{0x1E739E03,0x00400008,0x80000000,0x00000000}},
    {{0x2E4F4701,0x0040010A,0x80000000,0x00000005},{0x1E739E04,0x00000000,0x80000100,0x00000000}}
};

uint16_t cy_lvds_gpif0_transition[] = {0x0000, 0x5555, 0x8888, 0x7777, 0xAAAA, 0xF0F0};

uint8_t cy_lvds_gpif0_wavedata_position[] = {0, 1, 0, 2, 0};


const cy_stc_lvds_gpif_reg_data_t cy_lvds_gpif0_reg_data[] = {
    {0x00000000, 0x00030000}, /* GPIF_CONFIG */
    {0x00000004, 0x00000000}, /* GPIF_BUS_CONFIG */
    {0x00000008, 0x00000000}, /* GPIF_BUS_CONFIG2 */
    {0x0000000C, 0x00000005}, /* GPIF_AD_CONFIG */
    {0x00000010, 0x03020100}, /* GPIF_CTL_FUNC0 */
    {0x00000014, 0x07060504}, /* GPIF_CTL_FUNC1 */
    {0x00000018, 0x0B0A0908}, /* GPIF_CTL_FUNC2 */
    {0x0000001C, 0x0F0E0D0C}, /* GPIF_CTL_FUNC3 */
    {0x00000020, 0x13121110}, /* GPIF_CTL_FUNC4 */
    {0x00000030, 0xFFFFFFFF}, /* GPIF_INTR */
    {0x00000034, 0x00000002}, /* GPIF_INTR_MASK */
    {0x00000038, 0x00000000}, /* GPIF_INTR_MASKED */
    {0x0000003C, 0x00000000}, /* GPIF_INTR_SET */
    {0x00000050, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_0 */
    {0x00000058, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_2 */
    {0x0000005C, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_3 */
    {0x00000060, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_4 */
    {0x00000064, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_5 */
    {0x00000068, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_6 */
    {0x0000006C, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_7 */
    {0x00000070, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_8 */
    {0x00000074, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_9 */
    {0x00000078, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_10 */
    {0x0000007C, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_11 */
    {0x00000080, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_12 */
    {0x00000084, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_13 */
    {0x00000088, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_14 */
    {0x0000008C, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_15 */
    {0x00000090, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_16 */
    {0x00000094, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_17 */
    {0x00000098, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_18 */
    {0x0000009C, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_19 */
    {0x000000B0, 0x00000000}, /* GPIF_CTRL_BUS_DEFAULT */
    {0x000000B8, 0x00000000}, /* GPIF_CTRL_BUS_TOGGLE */
    {0x00000100, 0x00000000}, /* GPIF_CTRL_BUS_SELECT0 */
    {0x00000108, 0x00000000}, /* GPIF_CTRL_BUS_SELECT2 */

    {0x000000B4, 0x00000020}, /* GPIF_CTRL_BUS_POLARITY */ 
    {0x00000104, 0x00000000}, /* GPIF_CTRL_BUS_SELECT1 */
    {0x00000114, 0x00000018}, /* GPIF_CTRL_BUS_SELECT5 */ 
    {0x00000054, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_1 */
    {0x00000064, 0x00000002}, /* GPIF_CTRL_BUS_DIRECTION_5 */
    {0x0000010C, 0x00000000}, /* GPIF_CTRL_BUS_SELECT3 */
    {0x00000110, 0x00000000}, /* GPIF_CTRL_BUS_SELECT4 */
    {0x00000118, 0x00000000}, /* GPIF_CTRL_BUS_SELECT6 */
    {0x0000011C, 0x00000000}, /* GPIF_CTRL_BUS_SELECT7 */
    {0x00000120, 0x00000000}, /* GPIF_CTRL_BUS_SELECT8 */
    {0x00000124, 0x00000000}, /* GPIF_CTRL_BUS_SELECT9 */
    {0x00000128, 0x00000000}, /* GPIF_CTRL_BUS_SELECT10 */
    {0x0000012C, 0x00000000}, /* GPIF_CTRL_BUS_SELECT11 */
    {0x00000130, 0x00000000}, /* GPIF_CTRL_BUS_SELECT12 */
    {0x00000134, 0x00000000}, /* GPIF_CTRL_BUS_SELECT13 */
    {0x00000138, 0x00000000}, /* GPIF_CTRL_BUS_SELECT14 */
    {0x0000013C, 0x00000000}, /* GPIF_CTRL_BUS_SELECT15 */
    {0x00000140, 0x00000000}, /* GPIF_CTRL_BUS_SELECT16 */
    {0x00000144, 0x00000000}, /* GPIF_CTRL_BUS_SELECT17 */
    {0x00000148, 0x00000000}, /* GPIF_CTRL_BUS_SELECT18 */
    {0x0000014C, 0x00000000}, /* GPIF_CTRL_BUS_SELECT19 */
    {0x00000160, 0x00000000}, /* GPIF_CTRL_COUNT_CONFIG */
    {0x00000164, 0x00000000}, /* GPIF_CTRL_COUNT_RESET */
    {0x00000168, 0x00000000}, /* GPIF_CTRL_COUNT_LIMIT */
    {0x00000170, 0x00000000}, /* GPIF_ADDR_COUNT_CONFIG */
    {0x00000174, 0x00000000}, /* GPIF_ADDR_COUNT_RESET */
    {0x00000178, 0x00000000}, /* GPIF_ADDR_COUNT_LIMIT */
    {0x00000180, 0x00000000}, /* GPIF_STATE_COUNT_CONFIG */
    {0x00000184, 0x00000000}, /* GPIF_STATE_COUNT_LIMIT */
    {0x00000190, 0x00000000}, /* GPIF_DATA_COUNT_CONFIG */
    {0x00000194, 0x00000000}, /* GPIF_DATA_COUNT_RESET_LSB */
    {0x0000019C, 0x00000000}, /* GPIF_DATA_COUNT_RESET_MSB */
    {0x000001A0, 0x00000000}, /* GPIF_DATA_COUNT_LIMIT_LSB */
    {0x000001A4, 0x00000000}, /* GPIF_DATA_COUNT_LIMIT_MSB */
    {0x000001A8, 0x00000000}, /* GPIF_CTRL_COMP_VALUE */
    {0x000001AC, 0x00000000}, /* GPIF_CTRL_COMP_MASK */
    {0x000001B0, 0x00000000}, /* GPIF_DATA_COMP_VALUE_WORD0 */
    {0x000001B4, 0x00000000}, /* GPIF_DATA_COMP_VALUE_WORD1 */
    {0x000001B8, 0x00000000}, /* GPIF_DATA_COMP_VALUE_WORD2 */
    {0x000001BC, 0x00000000}, /* GPIF_DATA_COMP_VALUE_WORD3 */
    {0x000001D0, 0x00000000}, /* GPIF_DATA_COMP_MASK_WORD0 */
    {0x000001D4, 0x00000000}, /* GPIF_DATA_COMP_MASK_WORD1 */
    {0x000001D8, 0x00000000}, /* GPIF_DATA_COMP_MASK_WORD2 */
    {0x000001DC, 0x00000000}, /* GPIF_DATA_COMP_MASK_WORD3 */
    {0x000001E0, 0x00000000}, /* GPIF_ADDR_COMP_VALUE */
    {0x000001E4, 0x00000000}, /* GPIF_ADDR_COMP_MASK */
    {0x00000200, 0x00000000}, /* GPIF_WAVEFORM_CTRL_STAT */
    {0x00000204, 0x00000000}, /* GPIF_WAVEFORM_SWITCH */
    {0x00000208, 0x00000000}, /* GPIF_WAVEFORM_SWITCH_TIMEOUT */
    {0x00000210, 0x00000000}, /* GPIF_CRC_CONFIG */
    {0x00000218, 0xFFFFFFFF}, /* GPIF_BETA_DEASSERT */
    {0x000002B0, 0x00000000}, /* LINK_IDLE_CONFIG */
};

/* Pushing data into Thread 0 Socket */
const cy_stc_lvds_gpif_config_t cy_lvds_gpif0_config = {
    (uint16_t)(sizeof(cy_lvds_gpif0_wavedata_position)/sizeof(uint8_t)),
    (cy_stc_lvds_gpif_wavedata_t *) cy_lvds_gpif0_wavedata,
    cy_lvds_gpif0_wavedata_position,
    (uint16_t)(sizeof(cy_lvds_gpif0_transition)/sizeof(uint16_t)),
    cy_lvds_gpif0_transition,
    (uint16_t)(sizeof(cy_lvds_gpif0_reg_data)/sizeof(cy_stc_lvds_gpif_reg_data_t)),
    (cy_stc_lvds_gpif_reg_data_t *) cy_lvds_gpif0_reg_data
};

/* LVDS PHY configuration output generated by Device Configurator */
cy_stc_lvds_phy_config_t cy_lvds_phy0_config = {
    .phyTrainingPattern  = PHY_TRAINING_PATTERN_BYTE,
#if !LVDS_LB_EN
    .wideLink            = 1u,
    .dataBusWidth        = CY_LVDS_PHY_LVDS_MODE_NUM_LANE_16,
#else
    .wideLink            = 0u,
    .dataBusWidth        = CY_LVDS_PHY_LVDS_MODE_NUM_LANE_8,
#endif /* !LVDS_LB_EN */
    .modeSelect          = CY_LVDS_PHY_MODE_LVDS,
    .gearingRatio        = CY_LVDS_PHY_GEAR_RATIO_8_1,
    .linkTrainingPattern = LINK_TRAINING_PATTERN_BYTE,
#if LVDS_LB_EN
    .clkSrc              = CY_LVDS_GPIF_CLOCK_USB2,
    .clkDivider          = CY_LVDS_GPIF_CLOCK_DIV_4,
    .loopbackModeEn      = true,
    .isPutLoopbackMode   = true,
    .interfaceClock      = CY_LVDS_PHY_INTERFACE_CLK_625_MHZ,
    .interfaceClock_kHz  = 625000UL,
#else
    .clkSrc              = CY_LVDS_GPIF_CLOCK_USB2,
    .clkDivider          = CY_LVDS_GPIF_CLOCK_DIV_2,
    .loopbackModeEn      = false,
    .isPutLoopbackMode   = false,
    .interfaceClock      = CY_LVDS_PHY_INTERFACE_CLK_148_5_MHZ,
    .interfaceClock_kHz  = 148500UL,
#endif /* LVDS_LB_EN */
    .slaveFifoMode       = CY_LVDS_NORMAL_MODE,
    .ctrlBusBitMap       = 0x00000000,
    .dataBusDirection    = CY_LVDS_PHY_AD_BUS_DIR_INPUT,
    .lvcmosClkMode       = CY_LVDS_LVCMOS_CLK_SLAVE,

};

cy_stc_lvds_config_t cy_lvds0_config =
{
    .phyConfig  = (cy_stc_lvds_phy_config_t *)&cy_lvds_phy0_config,
    .gpifConfig = &cy_lvds_gpif0_config
};


#if LVDS_LB_EN

/* GPIF1 instance is used in TX mode to push the colorbar data into GPIF0. */
const cy_stc_lvds_gpif_wavedata_t cy_lvds_gpif1_wavedata[] = {
    {{0x1E73DA01,0x0C0000C0,0x80000000,0x00000000},{0x00000000,0x00000000,0x00000000,0x00000000}},
    {{0x2E739A02,0x0C000000,0x80000000,0x00000000},{0x00000000,0x00000000,0x00000000,0x00000000}}
};

uint16_t cy_lvds_gpif1_transition[] = {0x0000, 0x8888, 0x5555};

uint8_t cy_lvds_gpif1_wavedata_position[] = {0, 1, 0};

const cy_stc_lvds_gpif_reg_data_t cy_lvds_gpif1_reg_data[] = {
    {0x00000000,    0x00008080},    /* GPIF_CONFIG */
    {0x00000004,    0x00000000},    /* GPIF_BUS_CONFIG */
    {0x00000008,    0x00000000},    /* GPIF_BUS_CONFIG2 */
    {0x0000000C,    0x00000005},    /* GPIF_AD_CONFIG */
    {0x00000010,    0x03020100},    /* GPIF_CTL_FUNC0 */
    {0x00000014,    0x07060504},    /* GPIF_CTL_FUNC1 */
    {0x00000018,    0x0B0A0908},    /* GPIF_CTL_FUNC2 */
    {0x0000001C,    0x0F0E0D0C},    /* GPIF_CTL_FUNC3 */
    {0x00000020,    0x13121110},    /* GPIF_CTL_FUNC4 */
    {0x00000030,    0x00000000},    /* GPIF_INTR */
    {0x00000034,    0x00000000},    /* GPIF_INTR_MASK */
    {0x00000038,    0x00000000},    /* GPIF_INTR_MASKED */
    {0x0000003C,    0x00000000},    /* GPIF_INTR_SET */
    {0x00000050,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_0 */
    {0x00000054,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_1 */
    {0x00000058,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_2 */
    {0x0000005C,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_3 */
    {0x00000060,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_4 */
    {0x00000064,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_5 */
    {0x00000068,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_6 */
    {0x0000006C,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_7 */
    {0x00000070,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_8 */
    {0x00000074,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_9 */
    {0x00000078,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_10 */
    {0x0000007C,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_11 */
    {0x00000080,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_12 */
    {0x00000084,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_13 */
    {0x00000088,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_14 */
    {0x0000008C,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_15 */
    {0x00000090,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_16 */
    {0x00000094,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_17 */
    {0x00000098,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_18 */
    {0x0000009C,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_19 */
    {0x000000B0,    0x00000000},    /* GPIF_CTRL_BUS_DEFAULT */
    {0x000000B4,    0x00000000},    /* GPIF_CTRL_BUS_POLARITY */
    {0x000000B8,    0x00000000},    /* GPIF_CTRL_BUS_TOGGLE */
    {0x00000100,    0x0000000B},    /* GPIF_CTRL_BUS_SELECT_0 */
    {0x00000104,    0x00000010},    /* GPIF_CTRL_BUS_SELECT_1 */
    {0x00000108,    0x00000011},    /* GPIF_CTRL_BUS_SELECT_2 */
    {0x0000010C,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_3 */
    {0x00000110,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_4 */
    {0x00000114,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_5 */
    {0x00000118,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_6 */
    {0x0000011C,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_7 */
    {0x00000120,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_8 */
    {0x00000124,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_9 */
    {0x00000128,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_10 */
    {0x0000012C,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_11 */
    {0x00000130,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_12 */
    {0x00000134,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_13 */
    {0x00000138,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_14 */
    {0x0000013C,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_15 */
    {0x00000140,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_16 */
    {0x00000144,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_17 */
    {0x00000148,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_18 */
    {0x0000014C,    0x00000000},    /* GPIF_CTRL_BUS_SELECT_19 */
    {0x00000160,    0x00000000},    /* GPIF_CTRL_COUNT_CONFIG */
    {0x00000164,    0x00000000},    /* GPIF_CTRL_COUNT_RESET */
    {0x00000168,    0x00000000},    /* GPIF_CTRL_COUNT_LIMIT */
    {0x00000170,    0x00000000},    /* GPIF_ADDR_COUNT_CONFIG */
    {0x00000174,    0x00000000},    /* GPIF_ADDR_COUNT_RESET */
    {0x00000178,    0x00000000},    /* GPIF_ADDR_COUNT_LIMIT */
    {0x00000180,    0x00000000},    /* GPIF_STATE_COUNT_CONFIG */
    {0x00000184,    0x00000000},    /* GPIF_STATE_COUNT_LIMIT */
    {0x00000190,    0x00000000},    /* GPIF_DATA_COUNT_CONFIG */
    {0x00000194,    0x00000000},    /* GPIF_DATA_COUNT_RESET_LSB */
    {0x0000019C,    0x00000000},    /* GPIF_DATA_COUNT_RESET_MSB */
    {0x000001A0,    0x00000000},    /* GPIF_DATA_COUNT_LIMIT_LSB */
    {0x000001A4,    0x00000000},    /* GPIF_DATA_COUNT_LIMIT_MSB */
    {0x000001A8,    0x00000000},    /* GPIF_CTRL_COMP_VALUE */
    {0x000001AC,    0x00000000},    /* GPIF_CTRL_COMP_MASK */
    {0x000001B0,    0x00000000},    /* GPIF_DATA_COMP_VALUE_WORD0 */
    {0x000001B4,    0x00000000},    /* GPIF_DATA_COMP_VALUE_WORD1 */
    {0x000001B8,    0x00000000},    /* GPIF_DATA_COMP_VALUE_WORD2 */
    {0x000001BC,    0x00000000},    /* GPIF_DATA_COMP_VALUE_WORD3 */
    {0x000001D0,    0x00000000},    /* GPIF_DATA_COMP_MASK_WORD0 */
    {0x000001D4,    0x00000000},    /* GPIF_DATA_COMP_MASK_WORD1 */
    {0x000001D8,    0x00000000},    /* GPIF_DATA_COMP_MASK_WORD2 */
    {0x000001DC,    0x00000000},    /* GPIF_DATA_COMP_MASK_WORD3 */
    {0x000001E0,    0x00000000},    /* GPIF_ADDR_COMP_VALUE */
    {0x000001E4,    0x00000000},    /* GPIF_ADDR_COMP_MASK */
    {0x00000200,    0x00000000},    /* GPIF_WAVEFORM_CTRL_STAT */
    {0x00000204,    0x00000000},    /* GPIF_WAVEFORM_SWITCH */
    {0x00000208,    0x00000000},    /* GPIF_WAVEFORM_SWITCH_TIMEOUT */
    {0x00000210,    0x00000000},    /* GPIF_CRC_CONFIG */
    {0x00000218,    0xFFFFFFFF},    /* GPIF_BETA_DEASSERT */
    {0x000002B0,    0x00000000},    /* LINK_IDLE_CONFIG */
};

/* Pushing data into Thread 0 Socket */
const cy_stc_lvds_gpif_config_t cy_lvds_gpif1_config = {
    (uint16_t)(sizeof(cy_lvds_gpif1_wavedata_position)/sizeof(uint8_t)),
    (cy_stc_lvds_gpif_wavedata_t *) cy_lvds_gpif1_wavedata,
    cy_lvds_gpif1_wavedata_position,
    (uint16_t)(sizeof(cy_lvds_gpif1_transition)/sizeof(uint16_t)),
    cy_lvds_gpif1_transition,
    (uint16_t)(sizeof(cy_lvds_gpif1_reg_data)/sizeof(cy_stc_lvds_gpif_reg_data_t)),
    (cy_stc_lvds_gpif_reg_data_t *) cy_lvds_gpif1_reg_data
};

/* LVDS configuration output generated by Device Configurator */
cy_stc_lvds_phy_config_t cy_lvds_phy1_config =
{
    .loopbackModeEn      = true,
    .isPutLoopbackMode   = false,
    .wideLink            = 0u,
    .modeSelect          = CY_LVDS_PHY_MODE_LVCMOS,
    .dataBusWidth        = CY_LVDS_PHY_LVCMOS_MODE_NUM_LANE_16,
    .gearingRatio        = CY_LVDS_PHY_GEAR_RATIO_1_1,
    .clkSrc              = CY_LVDS_GPIF_CLOCK_USB2,
    .clkDivider          = CY_LVDS_GPIF_CLOCK_DIV_4,
    .interfaceClock      = CY_LVDS_PHY_INTERFACE_CLK_160_MHZ,
    .phyTrainingPattern  = 0x00,
    .linkTrainingPattern = 0x00000000,
    .ctrlBusBitMap       = 0x00000000,
    .slaveFifoMode       = CY_LVDS_NORMAL_MODE,
    .dataBusDirection    = CY_LVDS_PHY_AD_BUS_DIR_INPUT,
    .lvcmosClkMode       = CY_LVDS_LVCMOS_CLK_SLAVE
};

cy_stc_lvds_config_t cy_lvds1_config =
{
    .phyConfig  = (cy_stc_lvds_phy_config_t *)&cy_lvds_phy1_config,
    .gpifConfig = &cy_lvds_gpif1_config
};

#endif /* LVDS_LB_EN */

#if defined(__cplusplus)
}
#endif

#endif /* _CY_GPIF_HEADER_LVDS_H_ */

/*[]*/

