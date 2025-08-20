/***************************************************************************//**
* \file cy_u3v_device_config.h
* \version 1.0
*
* Define macros and arrays for the U3V XML
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

#ifndef _CY_U3V_DEVICE_CONFIG_H_
#define _CY_U3V_DEVICE_CONFIG_H_

#include "COMPONENT_U3VHEADER/cy_u3v_interface.h"
#include "cy_u3v_xml.h"

#define CY_U3V_RESOLUTION_4K_WIDTH              (3840)
#define CY_U3V_RESOLUTION_4K_HEIGHT             (2160)
#define CY_U3V_RESOLUTION_1080P_WIDTH           (1920)
#define CY_U3V_RESOLUTION_1080P_HEIGHT          (1080)
#define CY_U3V_RESOLUTION_720P_WIDTH            (1280)
#define CY_U3V_RESOLUTION_720P_HEIGHT           (720)
#define CY_U3V_RESOLUTION_VGA_WIDTH             (640)
#define CY_U3V_RESOLUTION_VGA_HEIGHT            (480)

#if FPGA_ADDS_HEADER
#define U3V_LEADER_SIZE                         (56)
#define U3V_TRAILER_SIZE                        (40)
#else
#define U3V_LEADER_SIZE                         (52)
#define U3V_TRAILER_SIZE                        (32)
#endif /* FPGA_ADDS_HEADER */

/* Default image resolution */
#define U3V_HRES                                (CY_U3V_RESOLUTION_4K_WIDTH)    
#define U3V_VRES                                (CY_U3V_RESOLUTION_4K_HEIGHT)  

#define U3V_HRES_MAX                            (CY_U3V_RESOLUTION_4K_WIDTH)
#define U3V_VRES_MAX                            (CY_U3V_RESOLUTION_4K_HEIGHT)

#define U3V_FPS                                 (30)
#define USB_CURRENT_SPEED                       (0x08)

#if FPGA_ENABLE
#define STREAM_DMA_BUFFER_SIZE                  (63*1024)
#else
#define STREAM_DMA_BUFFER_SIZE                  (45*1024)
#endif /* FPGA_ENABLE */

#define FPGA_DMA_BUFFER_SIZE                    (STREAM_DMA_BUFFER_SIZE)
#define IMAGE_FORMAT                            (YUV422_8)
#define BITS_PER_PIXEL                          (16)

#define STREAM_DMA_BUFFER_COUNT                 (3)
#define U3V_FRAME_SIZE                          ((U3V_HRES * U3V_VRES * BITS_PER_PIXEL)/8)
#define U3V_FINAL_TRANSFER_1_SIZE 	            ((U3V_FRAME_SIZE)%(STREAM_DMA_BUFFER_SIZE))
#define U3V_NUM_FULL_BUFFERS	  	            ((U3V_FRAME_SIZE)/(STREAM_DMA_BUFFER_SIZE))

/* This val should be less than U3V_ABRM_MAX_RESPONSE_TIME by 10 ms atleast */
#define U3V_ABRM_MAX_RESPONSE_TIME              (300)
#define U3V_WAIT_FOR_STOP_TIMEOUT_MS            (U3V_ABRM_MAX_RESPONSE_TIME - 10) 
#define NUM_OF_STREAM_EP_SUPPORTED              (1)

#if (STREAM_DMA_BUFFER_SIZE % 16 != 0)
#error "Buffer size should be multiple of 16"
#endif

#if (((U3V_HRES * U3V_VRES * BITS_PER_PIXEL) % 8)!= 0)
#warning "Frame size is not divisible by 8"
#endif

extern cy_stc_u3v_brm_cfg_t bootstrapConfig;
extern cy_stc_u3v_reg_base_t regBaseConfig;
extern cy_stc_u3v_dev_context_t u3vDevCtxt;
#endif /* _CY_U3V_DEVICE_CONFIG_H_ */

