/***************************************************************************/
/**
 * \file cy_u3v_interface.h
 * \version 1.0
 *
 * Defines USB3 Vision related events, structures and functions
 *
 *******************************************************************************
 * \copyright
 * (c) (2026), Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef __CY_U3V_INTERFACE_H__
#define __CY_U3V_INTERFACE_H__

#if defined(__cplusplus)
extern "C"
{
#endif

#define U3V_REG_SECTION __attribute__((section(".data.u3vRegSection"), used))

    /* Supported events from U3V interface; Its User's responsibility to handle events*/
    /* U3V events for handling stream/parameters configuration: External FPGA or device */
    typedef enum cy_en_u3v_dev_event_t
    {
        U3V_EVT_STREAM_ENABLE = 0x00,    /* U3V stream enable event */
        U3V_EVT_STREAM_DISABLE,          /* U3V stream disable event */
        U3V_EVT_IMG_FORMAT_HEIGHT,       /* U3V image height change event */
        U3V_EVT_IMG_FORMAT_WIDTH,        /* U3V image width change event */
        U3V_EVT_IMG_FORMAT_PXL_FORMAT,   /* U3V image format event */
        U3V_EVT_ACQUISITION_START,       /* U3V acquisition start event */
        U3V_EVT_ACQUISITION_STOP,        /* U3V acquisition stop event */
        U3V_EVT_ACQ_MODE_SINGLE,         /* U3V single frame mode event */
        U3V_EVT_ACQ_MODE_MULTI,          /* U3V multi frame mode event */
        U3V_EVT_ACQ_MODE_CONTINUOUS,     /* U3V continuous frame mode event */
        U3V_EVT_ACQUISITION_FRAME_COUNT, /* U3V frame count change event */
        U3V_EVT_ACQUISITION_SET_FPS,     /* U3V frame rate change event */
        /* CY_U3V_TRIGGER_ACQ_CTRL */
        U3V_EVT_TRIGGER_MODE_TRG_ON,     /* U3V stream trigger mode enable event */
        U3V_EVT_TRIGGER_MODE_TRG_OFF,    /* U3V stream trigger mode disable event */
        U3V_EVT_TRIGGER_MODE_SEL_ACQ,    /* U3V stream acquisition trigger selection event */
        U3V_EVT_TRIGGER_MODE_SEL_FRAME,  /* U3V stream frame trigger selection event */
        U3V_EVT_TRIGGER_MODE_SEL_NA,     /* U3V stream no trigger selection event */
        U3V_EVT_TRIGGER_ACTV_FALL_EDGE,  /* U3V stream trigger at falling edge event */
        U3V_EVT_TRIGGER_ACTV_FALL_RISE,  /* U3V stream trigger at rising edge event */
        U3V_EVT_TRIGGER_SOFTWARE_RCVD,   /* U3V stream software trigger received event */
        U3V_EVT_TRIGGER_SRC_SOFTWARE,    /* U3V stream trigger source software selection event */
        U3V_EVT_TRIGGER_SRC_HARDWARE_L0, /* U3V stream trigger source hardware line 0 selection event */
        U3V_EVT_TRIGGER_SRC_HARDWARE_L1, /* U3V stream trigger source hardware line 1 selection event */
        U3V_EVT_TRIGGER_SRC_HARDWARE_L2, /* U3V stream trigger source hardware line 2 selection event */
        U3V_EVT_TRIGGER_SRC_HARDWARE_L3, /* U3V stream trigger source hardware line 3 selection event */
        /* CY_U3V_CHUNK_DATA_CTRL */
        U3V_EVT_CHUNK_MODE_ENABLE,       /* U3V stream chunk mode enable event*/
        U3V_EVT_CHUNK_MODE_DISABLE,      /* U3V stream chunk mode disable event*/
        U3V_EVT_CHUNK_SEL_FRAME_ID,      /* U3V stream chunk mode frame id selection event*/
        U3V_EVT_CHUNK_SEL_TIMESTAMP,     /* U3V stream chunk mode time stamp selection event*/
        U3V_EVT_CHUNK_SEL_NA,            /* U3V stream chunk mode no selection event*/
        U3V_EVT_CHUNK_FRAME_ID_ENABLE,   /* U3V stream frame id chunk data enable event*/
        U3V_EVT_CHUNK_FRAME_ID_DISABLE,  /* U3V stream frame id chunk data disable event*/
        U3V_EVT_CHUNK_TIMESTAMP_ENABLE,  /* U3V stream time stamp chunk data enable event*/
        U3V_EVT_CHUNK_TIMESTAMP_DISABLE, /* U3V stream time stamp chunk data disable event*/
        U3V_EVT_CHUNK_FRAMEID_TIMESTAMP_ENABLE,  /* U3V stream frame id and time stamp chunk data enable event*/
        U3V_EVT_CHUNK_FRAMEID_TIMESTAMP_DISABLE, /* U3V stream frame id and time stamp chunk data disable event*/
        U3V_EVT_UNSUPPORTED_WR_ADDR,     /* U3V unsupported write address */
        U3V_EVT_NOT_INCLUDED_IN_LIB,     /* U3V event not supported by the Library*/
    } cy_en_u3v_dev_event_t;

    /* U3V return code */
    typedef enum cy_en_u3v_return_status_t
    {
        U3V_RET_STATUS_SUCCESS = 0x00,          /* Success */
        U3V_RET_STATUS_BUFF_NULL,               /* Null buffer */
        U3V_RET_STATUS_WAITING,                 /* Waiting/Busy  */
        U3V_RET_STATUS_DBG_REG_FAILED,          /* Invalid debug log handler function  */
        U3V_RET_STATUS_EVT_REG_FAILED,          /* Invalid device event handler function  */
        U3V_RET_STATUS_SEND_RESP_REG_FAILED,    /* Invalid send response handler function  */
        U3V_RET_STATUS_BOOTSTRAP_REG_FAILED,    /* Invalid bootstrap configuration  */
        U3V_RET_STATUS_INVALID_REG_ADDR_FAILED, /* Invalid base register address configuration  */
        U3V_RET_STATUS_INVALID_CHANNEL_NUM,     /* Invalid channel number  */
        U3V_RET_STATUS_DCI_BUSY,                /* DCI status: busy state*/
        U3V_RET_STATUS_DCI_PRCSS_CMD,           /* DCI status: command process state */
    } cy_en_u3v_return_status_t;

    /* U3V Regsiter configuration */
    typedef struct cy_stc_u3v_brm_cfg_t
    {
        const char *manufacturerNameStr;     /* Manufacturer name string */
        const char *modelNameStr;            /* Model name string */
        const char *familyNameStr;           /* Family name string */
        const char *deviceVerStr;            /* Device version string */
        const char *manufacturerInfoStr;     /* Manufacturer info string */
        const char *serialNumberStr;         /* Serial number string */
        const char *usrDefinedNameStr;       /* User defined name string */
        const char *deviceSwVer;             /* Device software version string */
        unsigned short leaderSize;           /* U3V leader size */
        unsigned short trailerSize;          /* U3V trailer size */
        unsigned short payloadType;          /* U3V payload type */
        unsigned long siInfo;                /* Stream interface info */
        unsigned long maxResponseTime;       /* Max response time in ms */
        unsigned long streamChannels;        /* U3V stream channel count */
        unsigned long usbCurrentSpeed;       /* Current USB speed supported */
        unsigned long oneXferSize;           /* Unit transfer size */
        unsigned long pxlFormat;             /* Image format */
        unsigned long maxWidth;              /* Max image width */
        unsigned long maxHeight;             /* Max image height */
        unsigned long width;                 /* Image width */
        unsigned long height;                /* Image height */
        unsigned long fps;                   /* Image frame rate */
        unsigned long bitsPerPixel;          /* Image format - bits/pixel */
        unsigned long long DeviceCapability; /* Device capability */
        unsigned long long DeviceConfigBits; /* Device config bits */

        unsigned long xmlFileLen;         /* XML File size (in bytes) */
        const unsigned char *xmlFileSHA1; /* XML SHA */
        const unsigned char *xmlFileReg;  /* Pointer to XML */
    } cy_stc_u3v_brm_cfg_t;

    /* U3V Register base address configuration*/
    typedef struct cy_stc_u3v_reg_base_t
    {
        unsigned long sbrmBaseAddr;          /* SBRM reg base address */
        unsigned long sirmBaseAddr;          /* SIRM reg base address */
        unsigned long acqModeCtrlBaseAddr;   /* Acquisition mode control reg base address */
        unsigned long imgCtrlBaseAddr;       /* Image format control reg base address */
        unsigned long transportCtrlBaseAddr; /* Transport control reg base address */
        unsigned long chunkDataCtrlBaseAddr; /* Transport control reg base address */
        unsigned long devCtrlBaseAddr;       /* Device control reg base address */
        unsigned long catInqBaseAddr;        /* Category inquiry reg base address */
        unsigned long manifestTableBaseAddr; /* Manifest table reg base address */
        unsigned long long xmlFileBaseAddr;  /* XML file reg base address */
    } cy_stc_u3v_reg_base_t;

    /* GENCP image formats */
    typedef enum cy_en_u3v_img_format_t {
        MONO8           = 0x01080001,                    /* Monochrome 8-bit: 8-bit */
        MONO10          = 0x01100003,                    /* Monochrome 10-bit unpacked: 10-bit */
        MONO12          = 0x01100005,                    /* Monochrome 12-bit unpacked: 12 -bit*/
        MONO16          = 0x01100007,                    /* Monochrome 16-bit unpacked: 16-bit*/
        RGB8            = 0x02180014,                    /* Red-Green-Blue 8-bit: 24-bit */
        BGR8            = 0x02180015,                    /* Blue-Green-Red 8-bit: 24-bit */
        RGBa8           = 0x02200016,                    /* Red-Green-Blue-alpha 8-bit: 32-bit */
        BGRa8           = 0x02200017,                    /* Red-Green-Blue-alpha 8-bit: 32-bit */
        YUV422_8_UYVY   = 0x0210001F,                    /* YUV 4:2:2 8-bit: 16-bit */
        YUV8_UYV        = 0x02180020,                    /* YUV 4:4:4 8-bit: 24-bit*/
        RGB8_PLANAR     = 0x02180021,                    /* Red-Green-Blue 8-bit planar: 24-bit*/  
        YUV422_8        = 0x02100032,                    /* YUV 4:2:2 8-bit: 16-bit */
    }cy_en_u3v_img_format_t;

    /* U3V Payload type */
    typedef enum cy_en_u3v_payload_type_t
    {
        PAYLOAD_IMAGE = 0x0001,               /* Payload image */
        PAYLOAD_CHUNK = 0x4000,               /* Payload chunk data */
        PAYLOAD_IMAGE_EXTENDED_CHUNK = 0x4001 /* Payload image extended chunk data */
    } cy_en_u3v_payload_type_t;

    typedef struct cy_stc_u3v_frame_params_t
    {
        unsigned long width;                /* frame width: number of pixels in 1 line */
        unsigned long height;               /* frame height: number of lines in frame */
        unsigned long imageSize;            /* frame size in bytes */
        unsigned long chunkSize;            /* chunk data size in bytes */
        unsigned long oneXferSize;          /* buffer size of single USB transfer */
        unsigned long bitsPerPixel;         /* bits in 1 pixel */
        unsigned long fullBufCnt;           /* number of full dma buffers */
        unsigned long totalFrameSize;       /* frame size: image + chunk */
        unsigned long finalXfer1Size;       /* final transfer size 1 */
        unsigned long finalXfer2Size;       /* final transfer size 2 */
        unsigned long multiModeFrameCount;  /* number of frames selected in case of multi-mode stream */
        unsigned long long blockID;         /* frame ID (block ID) */
        unsigned char streamStatus;         /* current streaming status: running/stopped */
        unsigned short payloadType;         /* u3v payload type: image, extended ... */
        unsigned short chunkID;             /* chunk id of current frame */
        unsigned short leaderSize;          /* u3v leader size */
        unsigned short trailerSize;         /* u3v trailer size */
    } cy_stc_u3v_frame_params_t;

    /* U3V interface configuration */
    typedef struct cy_stc_u3v_interface_config_t
    {
        unsigned char u3vChannelNum;                                                                                                           /* U3V channel number */
        cy_stc_u3v_brm_cfg_t *BRM_Config;                                                                                                      /* Pointer to U3V register configuration */
        cy_stc_u3v_reg_base_t *regBaseAddr;                                                                                                    /* Pointer to U3V register base address configuration */
        void (*CY_U3V_DbgLog)(unsigned char level, char *message, ...);                                                                        /* Pointer to U3V debug log handler */
        void (*CY_U3V_SendResponse)(unsigned char *rspBuffer, unsigned short len, void *userCtx, unsigned char chNum);                         /* Pointer to U3V send response handler */
        void (*Cy_U3V_DevEvent)(unsigned long ragAddr, unsigned long ragVal, cy_en_u3v_dev_event_t event, void *userCtx, unsigned char chNum); /* Pointer to U3V device event handler */
        void *userCtx;                                                                                                                         /* Pointer to user context */
    } cy_stc_u3v_interface_config_t;

    /* U3V device context structure.
     * All fields for the context structure are internal. Firmware never reads or
     * writes these values. Firmware allocates the structure and provides the
     * address of the structure to the driver in function calls. Firmware must
     * ensure that the defined instance of this structure remains in scope
     * while the device is in use.
     */
    typedef struct cy_stc_u3v_dev_context_t
    {
        unsigned char isPendingAckSent;                    /* status of pending acknowledge */
        unsigned char cmdHandlerBusy;                      /* status of u3v command decoder function  */
        unsigned char gl_u3vState;                         /* status of u3v dci state */
        unsigned char gl_u3vStrmState;                     /* status of u3v dsi state */
        cy_stc_u3v_frame_params_t glU3VFrame;              /* data structure of u3v frame  */
        cy_stc_u3v_interface_config_t glU3VDevInterface;   /* data structure of u3v device  */
    } cy_stc_u3v_dev_context_t;

    /*****************************************************************************
     * Function Name: Cy_U3V_UpdateDCIStateIdle_N
     ******************************************************************************
     * Summary:
     * Function to set DCI state to Waiting
     *
     * Parameters:
     * \param u3vDevCtxt
     *  pointer to u3v device context
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_UpdateDCIStateIdle_N(cy_stc_u3v_dev_context_t *u3vDevCtxt);

    /*****************************************************************************
     * Function Name: Cy_U3V_UpdateDciStateAfterRspSent
     ******************************************************************************
     * Summary:
     * Function to set DCI state after command response is sent
     *
     * Parameters:
     * \param u3vDevCtxt
     * pointer to U3V device contex
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_UpdateDciStateAfterRspSent(cy_stc_u3v_dev_context_t *u3vDevCtxt);

    /*****************************************************************************
     * Function Name: Cy_U3V_DsiEphHandler
     ******************************************************************************
     * Summary:
     * DSI Endpoint halt handler
     *
     * Parameters:
     * \param u3vDevCtxt
     * pointer to U3V device context
     *
     * \param isStall
     *  endpoint stall
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_DsiEphHandler(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                   unsigned char isStall);

    /*****************************************************************************
     * Function Name: Cy_U3V_DciEphHandler
     ******************************************************************************
     * Summary:
     * DCI Endpoint halt handler
     *
     * Parameters:
     * \param u3vDevCtxt
     * pointer to U3V device context
     *
     * \param isStall
     *  endpoint stall
     *
     * \param eph
     *  command/response endpoint
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_DciEphHandler(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                   unsigned char isStall,
                                                   unsigned char eph);

    /*****************************************************************************
     * Function Name: Cy_U3V_IsDCIStateIdle
     ******************************************************************************
     * Summary:
     * Function to check DCI idle state
     *
     * Parameters:
     * \param u3vDevCtxt
     *  pointer to U3V device context
     *
     * \param isBusy
     *  DCI status: 1u - DCI state is idle
     *              0u - DCI state is not idle
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_IsDCIStateIdle(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                    unsigned char *isStateIdle);

    /*****************************************************************************
     * Function Name: Cy_U3V_IsDCIStateIdle
     ******************************************************************************
     * Summary:
     * Function to check DCI busy state
     * Parameters:
     * \param u3vDevCtxt
     *  pointer to U3V device context
     *
     * \param isBusy
     *  DCI status: 1u - DCI handler busy
     *              0u - DCI handler idle
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_IsDCIHandlerBusy(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                      unsigned char *isBusy);

    /*****************************************************************************
     * Function Name: Cy_U3V_IsSentRspPendingAck
     ******************************************************************************
     * Summary:
     * Function to check pending ack response
     *
     * Parameters:
     * \param u3vDevCtxt
     * pointer to U3V device context
     *
     * \param isRspSent
     *  response sent status
     *  1u: pending ack sent
     *  0u: ack pending
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_IsSentRspPendingAck(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                         unsigned char *isRspSent);

    /*****************************************************************************
     * Function Name: Cy_U3V_IsFullFrameCompleted
     ******************************************************************************
     * Summary:
     * Function to check if full frame is completed
     *
     * Parameters:
     * \param u3vDevCtxt
     * pointer to U3V device context
     *
     * \param xfredFrameSize
     *  frame size transfered
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_IsFullFrameCompleted(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                          unsigned long xfredFrameSize,
                                                          unsigned char *isFrameComplete);

    /*****************************************************************************
     * Function Name: Cy_U3V_GetUpdatedTrailerData
     ******************************************************************************
     * Summary:
     * Function to get the leader data
     *
     * Parameters:
     * \param u3vDevCtxt
     *  pointer to u3v device context
     *
     * \param pBuffer
     *  pointer to the buffer
     *
     * \param leaderSize
     *  size of leader
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_GetUpdatedTrailerData(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                           unsigned char *pBuffer,
                                                           unsigned short *trailerSize);

    /*****************************************************************************
     * Function Name: Cy_U3V_GetUpdatedLeaderData
     ******************************************************************************
     * Summary:
     * Function to get the leader data
     *
     * Parameters:
     * \param u3vDevCtxt
     *  pointer to u3v device context
     *
     * \param ts
     *  timestamp
     *
     * \param pBuffer
     *  pointer to the buffer
     *
     * \param leaderSize
     *  size of leader
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_GetUpdatedLeaderData(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                          unsigned long long ts, unsigned char *pBuffer,
                                                          unsigned short *leaderSize);

    /*****************************************************************************
     * Function Name: Cy_U3V_SendPendingAckResponse
     ******************************************************************************
     * Summary:
     * Function to mark pending ack response
     *
     * Parameters:
     * \param u3vDevCtxt
     * pointer to U3V device context
     *
     * \param rspBuffer
     *  pointer to response buffer
     *
     * \param responseLen
     *  response length
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_SendPendingAckResponse(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                            unsigned char *rspBuffer,
                                                            unsigned short *responseLen);

    /*****************************************************************************
     * Function Name: Cy_U3V_GetlibVersion
     ******************************************************************************
     * Summary:
     * Function to get U3V library version
     *
     * Parameters:
     * None
     *
     * Return:
     * \return unsigned long
     * u3v library version
     *****************************************************************************/
    unsigned long Cy_U3V_GetlibVersion(void);

    /*****************************************************************************
     * Function Name: Cy_U3V_InterfaceConfig
     ******************************************************************************
     * Summary:
     * Function to configure U3V interface
     *
     * Parameters:
     * \param interface
     * pointer to interface configurations
     *
     * \param u3vDevCtxt
     * pointer to U3V device context
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_InterfaceConfig(cy_stc_u3v_interface_config_t *interface,
                                                     cy_stc_u3v_dev_context_t *u3vDevCtxt);

    /*****************************************************************************
     * Function Name: Cy_U3V_DciCmdHandler
     ******************************************************************************
     * Summary:
     * DCI command handler function
     *
     * Parameters:
     * \param u3vDevCtxt
     * pointer to U3V device context
     *
     * \param cmdBuffer
     * pointer to command buffer
     *
     * \param rspBuffer
     * pointer to response buffer
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_DciCmdHandler(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                   unsigned char *cmdBuffer,
                                                   unsigned char *rspBuffer);

    /*****************************************************************************
     * Function Name: Cy_U3V_GetBlockID
     ******************************************************************************
     * Summary:
     * Function to get updated block id
     *
     * Parameters:
     * \param u3vDevCtxt
     *  pointer to u3v device context
     *
     * \param payloadType
     *  U3V payload type
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_GetBlockID(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                unsigned long long *blockID);

    /*****************************************************************************
     * Function Name: Cy_U3V_GetPayloadType
     ******************************************************************************
     * Summary:
     * Function to get payload type
     *
     * Parameters:
     * \param u3vDevCtxt
     *  pointer to u3v device context
     *
     * \param payloadType
     *  U3V payload type
     *
     * Return:
     * \return cy_en_u3v_return_status_t
     *****************************************************************************/
    cy_en_u3v_return_status_t Cy_U3V_GetPayloadType(cy_stc_u3v_dev_context_t *u3vDevCtxt,
                                                    cy_en_u3v_payload_type_t *payloadType);

#if defined(__cplusplus)
}
#endif

#endif /* __CY_U3V_INTERFACE_H__ */
