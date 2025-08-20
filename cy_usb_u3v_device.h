/***************************************************************************//**
* \file cy_usb_u3v_device.h
* \version 1.0
*
* Provides definitions for endpoints and features used in FX10 USB3Vision
* application.
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

#ifndef _CY_USB_U3V_DEVICE_H_
#define _CY_USB_U3V_DEVICE_H_

#include "cy_usb_common.h"
#include "cy_usb_usbd.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define USB3_DESC_ATTRIBUTES __attribute__ ((section(".descSection"), used)) __attribute__ ((aligned (32)))
#define HBDMA_BUF_ATTRIBUTES __attribute__ ((section(".hbBufSection"), used)) __attribute__ ((aligned (32)))

#define CY_USB_U3V_DEVICE_MSG_QUEUE_SIZE        20
#define CY_USB_U3V_DEVICE_MSG_SIZE              (sizeof (cy_stc_usbd_app_msg_t))

#define CY_USB_U3V_CMD_EVT_FLAG                 (0x01)   /* U3V Command Received by device Flag */
#define CY_USB_U3V_SETCONF_EVT_FLAG             (0x02)   /* Set Configuation Flag */
#define CY_USB_U3V_DMA_RST_EVT_FLAG             (0x03)   /* DMA Reset Event Flag */
#define CY_USB_U3V_RESP_SENT_EVT_FLAG           (0x05)   /* U3V Response sent Event Flag */
#define CY_USB_U3V_EPHALT_DCI_RSP_EVT_FLAG      (0x06)   /* U3V DCI Response EP Halt/Clear Event Flag */
#define CY_USB_U3V_EPHALT_DCI_CMD_EVT_FLAG      (0x07)   /* U3V DCI Command EP Halt/Clear Event Flag */
#define CY_USB_U3V_EPHALT_DSI_EVT_FLAG          (0x08)   /* U3V DSI EP Halt/Clear Event Flag */

#define CY_USB_U3V_LEADER_RXD_FLAG              (0x0A)   /* U3V Leader Received Flag */
#define CY_USB_U3V_TRAILER_RXD_FLAG             (0x0B)   /* U3V Trailer Received Flag */
#define CY_USB_U3V_FRAME_MISSING_FLAG           (0x0C)   /* U3V Frame Missing Flag */
#define CY_USB_U3V_DMA_BLOCKED_FLAG             (0x0D)   /* DMA Blocked Flag */
#define CY_USB_U3V_ZLP_DISCARDED                (0x0F)   /* U3V ZLP Discard */
#define CY_USB_U3V_NO_LEADER_RECVD              (0x10)   /* U3V no Leader Received */
#define CY_USB_U3V_VBUS_CHANGE_INTR             (0x14)   /* VBus change interrupt received. */
#define CY_USB_U3V_VBUS_CHANGE_DEBOUNCED        (0x15)   /* VBus change status has been debounced. */
#define CY_USB_U3V_FPS_POLL_EVENT               (0x1F)   /* U3V FPS Poll event */
#define CY_USB_U3V_EPCLR_DCI_RSP_EVT_FLAG       (0x20)   /* U3V DCI Response EP Clear Event Flag */
#define CY_USB_U3V_EPCLR_DCI_CMD_EVT_FLAG       (0x21)   /* U3V DCI Command EP Clear Event Flag */
#define CY_USB_U3V_EPCLR_DSI_EVT_FLAG           (0x22)   /* U3V DSI EP Clear Event Flag */

#define CY_USB_USB2_EXTN_CAPB_TYPE              (0x02)   /* USB 2.0 extension descriptor. */
#define CY_USB_SS_CAPB_TYPE                     (0x03)   /* Super speed USB specific device level capabilities. */
#define CY_USB_EP_BULK                          (0x02)   /* Bulk Endpoint Type */
#define CY_U3V_STREAM_EP_BURST                  (16)     /* USB stream endpoint burst*/
#define CY_U3V_EP_DCI_CMD                       (0x01)   /* U3V Command (OUT) endpoint index. */
#define CY_U3V_EP_DCI_RSP                       (0x01)   /* U3V Response (IN) endpoint index. */
#define CY_U3V_EP_DSI_STREAM                    (0x03)   /* U3V Streaming (IN) endpoint index. */

#define CY_U3V_EP_DCI_CMD_ADDR                  (CY_U3V_EP_DCI_CMD)
#define CY_U3V_EP_DCI_RSP_ADDR                  (0x80 | CY_U3V_EP_DCI_RSP)
#define CY_U3V_EP_DSI_STREAM_ADDR               (0x80 | CY_U3V_EP_DSI_STREAM)

#define GENCP_VERSION                           (0x010001) 
#define U3V_VERSION                             (0x01) 

/* U3V video streaming params */
typedef struct
{
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t bitsPerPixel;
    uint32_t totalFrameSize;
    uint32_t imageSize;
    uint32_t payloadTransferCount;
    uint32_t payloadTransferSize;
    uint32_t finalTransfer1Size;
    uint32_t pixelFormat;
}cy_stc_u3v_img_params_t;

/* U3V flags */
typedef struct
{
    volatile uint32_t frameOngoing;
    volatile uint32_t lastFrameProduced;
    volatile uint32_t lastFrameConsumed;
    volatile uint32_t acqStopReceived;
    volatile uint32_t blockFrame;
}cy_stc_u3v_flags_t;

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_U3V_DEVICE_H_ */

/* End of File */

