/***************************************************************************//**
* \file cy_usb_app.h
* \version 1.0
*
* Defines the interfaces used in the FX20 USB3Vision application.
*
*******************************************************************************
* \copyright
* (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef _CY_USB_APP_H_
#define _CY_USB_APP_H_

#include "cy_pdl.h"
#include "cy_debug.h"
#include "cy_usbhs_dw_wrapper.h"
#include "cy_lvds.h"
#include "cy_usb_u3v_device.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define RED                                                        "\033[0;31m"
#define CYAN                                                       "\033[0;36m"
#define COLOR_RESET                                                "\033[0m"

#define LOG_COLOR(...)                                             Cy_Debug_AddToLog(1,CYAN);\
                                                                   Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                                                   Cy_Debug_AddToLog(1,COLOR_RESET);

#define LOG_ERROR(...)                                             Cy_Debug_AddToLog(1,RED);\
                                                                   Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                                                   Cy_Debug_AddToLog(1,COLOR_RESET);

#define LOG_CLR(CLR, ...)                                          Cy_Debug_AddToLog(1,CLR);\
                                                                   Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                                                   Cy_Debug_AddToLog(1,COLOR_RESET);

#define DELAY_MICRO(us)                                            Cy_SysLib_DelayUs(us)
#define PHY_TRAINING_PATTERN_BYTE                                  (0x9C)
#define LINK_TRAINING_PATTERN_BYTE                                 (0x125A4B78)
#define SET_BIT(byte, mask)                                        (byte) |= (mask)
#define CLR_BIT(byte, mask)                                        (byte) &= ~(mask)
#define CHK_BIT(byte, mask)                                        (byte) & (mask)

#if (LVCMOS_EN)
#define LVCMOS_DDR_EN                                               (1)
#endif /* LVCMOS_EN  */

#define LINK_TRAINING                                               (1)
#define LINK_READY_CTL_PORT                                         (0)
#define LINK_READY_CTL_PIN                                          (cy_en_lvds_phy_gpio_index_t)(16+6)

#if ((FPGA_ENABLE && FPGA_ADDS_HEADER) || INMD_EN)
#define AUTO_DMA_EN                                                  (1)
#else
#define AUTO_DMA_EN                                                  (0)
#endif /*(FPGA_ENABLE && FPGA_ADDS_HEADER) */


#if ACTIVE_SERIAL
#define FPGA_CONFIG_MODE                                            (ACTIVE_SERIAL_MODE)
#else 
#define FPGA_CONFIG_MODE                                            (PASSIVE_SERIAL_MODE)
#endif /* ACTIVE_SERIAL */

#if (LVDS_LB_EN && FPGA_ENABLE)
#error LOG_COLOR(RED, "INVALID: LVDS_LB_EN WITH FPGA_ENABLE");
#endif /* LVDS_LB_EN && FPGA_ENABLE */

#if (FPGA_ADDS_HEADER && LVDS_LB_EN)
#error LOG_COLOR(RED, "INVALID: LOOPBACK WITH UVC_HEADER_BY_FPGA");
#endif /* FPGA_ADDS_HEADER && LVDS_LB_EN */

#if (LVDS_LB_EN && INMD_EN)
#error LOG_COLOR(RED, "INVALID: LVDS_LB_EN WITH INMD ");
#endif /* LVDS_LB_EN && INMD_EN */

#if (FPGA_ADDS_HEADER && INMD_EN)
#error LOG_COLOR(RED, "INVALID: FPGA_ADDS_HEADER WITH INMD_EN");
#endif /* LVDS_LB_EN && INMD_EN */

#if (LVCMOS_EN && INMD_EN)
#error LOG_COLOR(RED, "INVALID: INMD WITH LVCMOS_EN");
#endif /* LVCMOS_EN && INMD_EN*/

#if ((!LVCMOS_EN) && LVCMOS_DDR_EN)
#error LOG_COLOR(RED, "INVALID: LVCMOS_DDR_EN WITH LVDS");
#endif /* LVCMOS_EN && INMD_EN*/

#if ((LVDS_LB_EN) && (U3V_INMEM_EN))
#error LOG_COLOR(RED, "INVALID: INMEM WITH LVDS_LB_EN");
#endif /* LVCMOS_EN && INMD_EN*/

#if ((FPGA_EN) && (U3V_INMEM_EN))
#error LOG_COLOR(RED, "INVALID: INMEM WITH FPGA_EN");
#endif /* LVCMOS_EN && INMD_EN*/


/* GPIO port pins*/
#define TI180_INIT_RESET_GPIO                                       (P4_3_GPIO)
#define TI180_INIT_RESET_PORT                                       (P4_3_PORT)
#define TI180_INIT_RESET_PIN                                        (P4_3_PIN)

#define TI180_CDONE_PIN                                             (P4_4_PIN)
#define TI180_CDONE_PORT                                            (P4_4_PORT)

#define CDONE_WAIT_TIMEOUT                                          (1000)

#define CMD_BUFF_SIZE                                               (0x400)
#define RSP_BUFF_SIZE                                               (0x2000)

/* I2C Related macro */
#define FPGA_I2C_ADDRESS_WIDTH                                      (2)
#define FPGA_I2C_DATA_WIDTH                                         (1)

#define ASSERT(condition, value)                                    Cy_CheckStatus(__func__, __LINE__, condition, value, true);
#define ASSERT_NON_BLOCK(condition, value)                          Cy_CheckStatus(__func__, __LINE__, condition, value, false);
#define ASSERT_AND_HANDLE(condition, value, failureHandler)         Cy_CheckStatusAndHandleFailure(__func__, __LINE__, condition, value, false, Cy_FailHandler);

/* Loopback program color bands*/
#define BAND1_COLOR_YUYV                                            0x80ff80ff    /* White */
#define BAND2_COLOR_YUYV                                            0x94ff00ff    /* Yellow */
#define BAND3_COLOR_YUYV                                            0x1ac8bfc8    /* Blue */
#define BAND4_COLOR_YUYV                                            0x4aca55ca    /* Green */
#define BAND5_COLOR_YUYV                                            0xf3969f96    /* Pink */
#define BAND6_COLOR_YUYV                                            0xff4c544c    /* Red */
#define BAND7_COLOR_YUYV                                            0x9e40d340    /* Violet */
#define BAND8_COLOR_YUYV                                            0x80008000    /* Black */

#if U3V_INMEM_EN
#define COLORBAR_BAND_COUNT_4K                                      60
#define COLORBAR_BAND_COUNT_1080P                                   30
#define COLORBAR_BAND_COUNT_720P                                    20
#define COLORBAR_BAND_COUNT_480P                                    10
#else
#define COLORBAR_BAND_COUNT_4K                                      120
#define COLORBAR_BAND_COUNT_1080P                                   60
#define COLORBAR_BAND_COUNT_720P                                    40
#define COLORBAR_BAND_COUNT_480P                                    20
#endif /* U3V_INMEM_EN */

#define LOOPBACK_MEM_BUF_SIZE                                       (STREAM_DMA_BUFFER_SIZE + 0xC0)
#define CY_USB_U3V_BUFFER_NO_3480_2160                              (362)
#define CY_USB_U3V_LEADER_BUFFER_NO                                 (1)

/* P4.0 is used for VBus detect functionality. */
#define VBUS_DETECT_GPIO_PORT                                       (P4_0_PORT)
#define VBUS_DETECT_GPIO_PIN                                        (P4_0_PIN)
#define VBUS_DETECT_GPIO_INTR                                       (ioss_interrupts_gpio_dpslp_4_IRQn)
#define VBUS_DETECT_STATE                                           (0u)

#define LVCMOS_GPIF_CTRLBUS_BITMAP_WL                               (0x000C008F)

typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;

/* USBD layer return code shared between USBD layer and Application layer. */
typedef enum cy_en_usb_app_ret_code_ 
{
    CY_USB_APP_STATUS_SUCCESS=0,
    CY_USB_APP_STATUS_FAILURE,
}cy_en_usb_app_ret_code_t;

/* Get the LS byte from a 16-bit number */
#define CY_GET_LSB(w)                                               ((uint8_t)((w)&UINT8_MAX))

/* Get the MS byte from a 16-bit number */
#define CY_GET_MSB(w)                                               ((uint8_t)((w) >> 8))

/* Retrieves byte 0 from a 32 bit number */
#define CY_DWORD_GET_BYTE0(d)                                       ((uint8_t)((d) & 0xFF))

/* Retrieves byte 1 from a 32 bit number */
#define CY_DWORD_GET_BYTE1(d)                                       ((uint8_t)(((d) >>  8) & 0xFF))

/* Retrieves byte 2 from a 32 bit number */
#define CY_DWORD_GET_BYTE2(d)                                       ((uint8_t)(((d) >> 16) & 0xFF))

/* Retrieves byte 3 from a 32 bit number */
#define CY_DWORD_GET_BYTE3(d)                                       ((uint8_t)(((d) >> 24) & 0xFF))

/* Retrieves LS 2 bytes from a 32 bit number */
#define CY_GET_LS_HWORD(d)                                          ((uint16_t)(((d) >> 0) & 0xFFFF))

/* Retrieves MS 2 bytes from a 32 bit number */
#define CY_GET_MS_HWORD(d)                                          ((uint16_t)(((d) >> 16) & 0xFFFF))

#define FPGASLAVE_ADDR                                              (0x0D)


/*
 * USB application data structure which is bridge between USB system and device
 * functionality.
 * It maintains some usb system information which comes from USBD and it also
 * maintains info about functionality.
 */
struct cy_stc_usb_app_ctxt_
{
    uint8_t firstInitDone;
    cy_en_usb_device_state_t devState;
    cy_en_usb_device_state_t prevDevState;
    cy_en_usb_speed_t devSpeed;
    uint8_t devAddr;
    uint8_t activeCfgNum;
    cy_en_usb_enum_method_t enumMethod;
    uint8_t prevAltSetting;
	cy_en_usb_speed_t desiredSpeed;
    bool  usbConnectDone;
    
    cy_stc_app_endp_dma_set_t endpInDma[CY_USB_MAX_ENDP_NUMBER];
    cy_stc_app_endp_dma_set_t endpOutDma[CY_USB_MAX_ENDP_NUMBER];
    DMAC_Type *pCpuDmacBase;
    DW_Type *pCpuDw0Base;
    DW_Type *pCpuDw1Base;

    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;
    cy_stc_hbdma_mgr_context_t *pHbDmaMgr; 

    /* Global Task handles */
    TaskHandle_t u3vDevicetaskHandle;
    QueueHandle_t u3vMessageQueue;

    /* Timer functionality for DCI */
    TimerHandle_t dciTimerHandle;
    uint32_t dcitimerExpiry;

    /* U3V streaming related flags. */
    uint8_t u3vPendingBufCnt;
    bool    u3vFlowCtrlFlag;
    bool dmaInterruptDisabled;

    uint32_t *pUsbEvtLogBuf;
    TimerHandle_t evtLogTimer;                  /** Timer to print eventLog. */

    bool vbusChangeIntr;                        /** VBus change interrupt received flag. */
    bool vbusPresent;                           /** VBus presence indicator flag. */
    bool isLpmEnabled;                          /** Whether LPM transitions are enabled. */
    uint32_t lpmEnableTime;                     /** Timestamp at which LPM should be re-enabled. */
    TimerHandle_t vbusDebounceTimer;            /** VBus change debounce timer handle. */
    TimerHandle_t userTimer;                    /** User timer handle. */
    uint8_t fpgaVersion;

    volatile uint32_t fpsCount;
    volatile uint32_t fpsPrint;
    uint32_t glProd;
    uint32_t glCons;
    uint32_t prodCount;
    uint32_t consCount;
    uint32_t frameSizeTransferred;
    uint32_t frameSize;

    bool isPartialBuf;
    uint32_t fullBufCount;
    uint32_t partialBufSize;
};

/* LVDS loopback configuration */
typedef struct
{
    uint8_t *pBuffer;
    uint8_t start;
    uint8_t end;
    uint8_t dataMode;
    uint8_t dataSrc;
    uint8_t ctrlByte;
    uint16_t repeatCount;
    uint32_t dataL;
    uint32_t dataH;
    uint32_t ctrlBusVal;
    uint32_t lbPgmCount;
} cy_stc_lvds_loopback_config_t;

/* LVDS loopback memory structure */
typedef struct
{
    uint32_t dataWord0;
    uint32_t dataWord1;
    uint32_t dataWord2;
    uint32_t dataWord3;
} cy_stc_lvds_loopback_mem_t;

/* U3V endpoint halt state */
typedef enum 
{
    EPH_CLEAR,
    EPH_SET
} cy_en_u3v_eph_state_t;

/* U3V endpoint type */
typedef enum 
{
    EP_OUT,
    EP_IN
} cy_en_u3v_eph_type_t;

/* FPGA register map */
typedef enum cy_en_fpgaRegMap_t
{
    /*Common Register Info*/
    FPGA_MAJOR_VERSION_ADDRESS = 0x00,

    FPGA_UVC_U3V_SELECTION_ADDRESS         = 0x01,
    FPGA_UVC_ENABLE                        = 1,
    FPGA_U3V_ENABLE                        = 0,

    FPGA_U3V_HEADER_CTRL_ADDRESS = 0x02,
    FPGA_U3V_HEADER_DISABLE = 0x00,
    FPGA_U3V_HEADER_ENABLE_PAYLOAD = 0x01,
    FPGA_U3V_HEADER_ENABLE_METADATA = 0x02,

    FPGA_LVDS_PHY_TRAINING_ADDRESS = 0x03,
    FPGA_LVDS_PHY_TRAINING_DATA = 0x53,

    FPGA_LVDS_LINK_TRAINING_BLK_P0_ADDRESS = 0x04,
    FPGA_LVDS_LINK_TRAINING_BLK_P1_ADDRESS = 0x05,
    FPGA_LVDS_LINK_TRAINING_BLK_P2_ADDRESS = 0x06,
    FPGA_LVDS_LINK_TRAINING_BLK_P3_ADDRESS = 0x07,

    FPGA_ACTIVE_DEVICE_MASK_ADDRESS = 0x08,
    FPGA_LOW_PWR_MODE_ADDRESS = 0x09,
    VAL_LPM_SUPPORT = 1,

    FPGA_PHY_LINK_CONTROL_ADDRESS = 0x0A,
    FPGA_TRAINING_DISABLE = 0x00,
    FPGA_PHY_CONTROL = 0x01,
    FPGA_LINK_CONTROL = 0x02,
    PORT0_LINK_TRAINING_DONE = 0x42,
    PORT1_LINK_TRAINING_DONE = 0x82,

    ADDR_FPGA_EXT_CTRLR_STS_INFO = 0x0B,
    VAL_DMA_RDY_FLAG_STS = 0,
    VAL_DDR_CONFIG_STS = 1,
    VAL_DDR_CTRLR_BUSY_STS = 2,
    VAL_CMD_Q_FULL_STS = 3,
    VAL_DATAPATH_IDLE_STS = 4,

    FPGA_DEV0_STREAM_ENABLE_ADDRESS = 0x20,
    CAMERA_APP_DISABLE = 0x0,
    DMA_CH_RESET = 0x01,
    CAMERA_APP_ENABLE = 0x02,

    APP_STOP_NOTIFICATION = 0x06,

    FPGA_DEV0_STREAM_MODE_ADDRESS = 0x21,
    NO_CONVERSION = 0,
    INTERLEAVED_MODE = 0x01,
    STILL_CAPTURE = 0x02,
    MONO_8_CONVERSION = 0x04,
    YUV422_420_CONVERSION = 0x08,


    DEV0_IMAGE_HEIGHT_LSB_ADDRESS = 0x22,
    DEV0_IMAGE_HEIGHT_MSB_ADDRESS =  0x23,
    DEV0_IMAGE_WIDTH_LSB_ADDRESS =  0x24,
    DEV0_IMAGE_WIDTH_MSB_ADDRESS =  0x25,
    
    DEV0_FPS_ADDRESS =  0x26,


    DEV0_PIXEL_WIDTH_ADDRESS = 0x27,
    PIXEL_SIZE_8 = 8,
    PIXEL_SIZE_12 = 12,
    PIXEL_SIZE_16 = 16,
    PIXEL_SIZE_24 = 24,
    PIXEL_SIZE_32 = 32,

    DEV0_SOURCE_TYPE_ADDRESS = 0x28,
    INTERNAL_COLORBAR = 0x00,
    HDMI_SOURCE = 0x01,
    MIPI_SOURCE = 0x02,

    VAL_OTHER_SRC = 3,

    DEV0_FLAG_STATUS_ADDRESS = 0x29,
    SLAVE_FIFO_ALMOST_EMPTY = 0,
    INTERMEDIATE_FIFO_EMPTY = 1,
    INTERMEDIATE_FIFO_FULL = 2,
    DDR_WR_FLAG_STATUS = 4,
    DDR_RD_FLAG_STATUS = 5,

    DEV0_MIPI_STATUS_ADDRESS_L = 0x2A,

    DEV0_HDMI_SOURCE_INFO_ADDRESS = 0x2B,
    HDMI_CONNECT_STATUS_POS = 0x00,
    HDMI_CONNECT = 0x01,
    HDMI_DISCONNECT = 0x00,
    HDMI_CHANNEL_CONFIG_POS = 0x01,
    HDMI_CHANNEL_CONFIG_SINGLE = 0x00,
    HDMI_CHANNEL_CONFIG_DUAL = 0x02, 

    VAL_MIPI_ISP_EN = 0x10,
    VAL_MIPI_CROP_ALGO = 0x20,

    DEV0_U3V_STREAM_MODE_ADDRESS = 0x2C,
    DEV0_U3V_STREAM_MODE_CONTINUOUS = 0x01,
    DEV0_U3V_STREAM_MODE_SINGLE = 0x02,
    DEV0_U3V_STREAM_MODE_MULTI = 0x04,
    DEV0_U3V_MULTI_DEFAULT_CNT = 0x01,
    DEV0_U3V_MULTI_SHIFT = 0x03,
    DEV_U3V_MAX_MULTI_COUNT = 0x1F, 

    DEV0_ACTIVE_THREAD_INFO_ADDRESS = 0x2F,
    DEV0_ACTIVE_THREAD_INFO_SINGLE_THREAD = 0x01,
    DEV0_ACTIVE_THREAD_INFO_DUAL_THREAD = 0x02,
    DEV0_THREAD1_INFO_ADDRESS = 0x30,
    DEV0_THREAD2_INFO_ADDRESS = 0x31,
    DEV0_THREAD1_SOCKET_INFO_ADDRESS = 0x32,
    DEV0_THREAD2_SOCKET_INFO_ADDRESS = 0x33,
    DEV0_FLAG_INFO_ADDRESS = 0x34,
    DEV0_COUNTER_CRC_INFO_ADDRESS = 0x35,
    DEV0_BUFFER_SIZE_LSB_ADDRESS = 0x36,
    DEV0_BUFFER_SIZE_MSB_ADDRESS = 0x37,

    THREAD_0_SOCKET_0 = 0,
    THREAD_1_SOCKET_0 = 0,

} cy_en_fpgaRegMap_t;

/* FPGA Configuration mode selection*/
typedef enum cy_en_fpgaConfigMode_t
{
    ACTIVE_SERIAL_MODE,
    PASSIVE_SERIAL_MODE
}cy_en_fpgaConfigMode_t;

/* FPGA data stream control*/
typedef enum cy_en_streamControl_t
{
    STOP,
    START
}cy_en_streamControl_t;

extern cy_stc_hbdma_channel_t lvdsLbPgmChannel;
extern uint8_t glPhyLinkTrainControl;

extern cy_stc_lvds_md_config_t mdArray0_U3V_LEADER[16];
extern cy_stc_lvds_md_config_t mdArray1_U3V_LEADER[10];
extern cy_stc_lvds_md_config_t mdArray0_U3V_Trailer[16];

extern uint32_t glU3VBufCounter;
extern uint32_t glU3VFrameBuffer;
extern uint16_t glU3VColorbarSize;
extern cy_stc_u3v_flags_t  glFlags;
extern volatile bool glU3VIsApplnActive;


/*****************************************************************************
* Function Name: Cy_U3V_AppGpifIntr
******************************************************************************
* Summary:
* GPIF error handler
*
* Parameters:
* \param pApp
* application layer context pointer
*
* Return:
* void
*****************************************************************************/
void Cy_U3V_AppGpifIntr(void *pApp);

/*****************************************************************************
* Function Name: Cy_LVDS_InitLbPgm
******************************************************************************
* Summary:
* Function to initialize Link Looback 
*
* \param buffStat
* HBDMA buffer status
*
* \param lbPgmConfig
* loopback config
*
* \return
* None
*
 *******************************************************************************/
void Cy_LVDS_InitLbPgm(cy_stc_hbdma_buff_status_t *buffStat, cy_stc_lvds_loopback_config_t *lbPgmConfig);

/*****************************************************************************
* Function Name: Cy_HbDma_LoopbackCb
******************************************************************************
* Summary:
* HBDMA callback function to commit data to loopback channel
*
* \param handle
* HBDMA channel handle
* 
* \param cy_en_hbdma_cb_type_t
* HBDMA channel type
*
* \param pbufStat
* fHBDMA buffer status
*
* \param userCtx
* user context
*
* \return
* None
*
 *******************************************************************************/
void Cy_HbDma_LoopbackCb(cy_stc_hbdma_channel_t *handle, cy_en_hbdma_cb_type_t type, 
                        cy_stc_hbdma_buff_status_t *pbufStat, void *userCtx);

/*****************************************************************************
* Function Name: Cy_U3V_AppHandleFormatConversion
******************************************************************************
* Summary:
* Function to handle U3V Format change request
*
* Parameters:
* \param imageFormat
* image format
* 
* Return:
* void
*****************************************************************************/
void Cy_U3V_AppHandleFormatConversion(uint32_t imageFormat);

/*****************************************************************************************
* Function Name: Cy_USB_AppInit
*****************************************************************************************
* Summary:
* This function Initializes application related data structures, register callback
* creates task for device function.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD layer Context pointer
*
* \param pCpuDmacBase
* DMAC base address
*
* \param pCpuDw0Base
* DataWire 0 base address
*
* \param pCpuDw1Base
* DataWire 1 base address
*
* \param pHbDmaMgrCtxt
* HBDMA Manager Context
*
* \return
* None
*
 ************************************************************************************ */
void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, 
                    DMAC_Type *pCpuDmacBase, DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base, 
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt);

/*****************************************************************************
* Function Name: Cy_USB_AppRegisterCallback
******************************************************************************
* Summary:
*  This function will register all calback with USBD layer.
*
* \param pAppCtxt
* application layer context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);

/****************************************************************************
* Function Name: Cy_USB_AppSetCfgCallback
******************************************************************************
* Summary:
* Callback function will be invoked by USBD when set configuration is received
*
* \param pUserCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* USB Message
*
* \return
* None
*
 *******************************************************************************/
void Cy_USB_AppSetCfgCallback(void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/****************************************************************************
* Function Name: Cy_USB_AppBusResetCallback
******************************************************************************
* Summary:
* Callback function will be invoked by USBD when bus detects RESET
*
* \param pUserCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD layer context pointer
*
* \param pMsg
* USB Message
*
* \return
* None
*
 *******************************************************************************/
void Cy_USB_AppBusResetCallback(void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/****************************************************************************
* Function Name: Cy_USB_AppBusResetDoneCallback
******************************************************************************
* Summary:
* Callback function will be invoked by USBD when RESET is completed
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD layer context pointer
*
* \param pMsg
* USB Message
*
* \return
* None
*
 *******************************************************************************/
void Cy_USB_AppBusResetDoneCallback(void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/****************************************************************************
* Function Name: Cy_USB_AppBusSpeedCallback
******************************************************************************
* Summary:
* Callback function will be invoked by USBD when speed is identified or
* speed change is detected
*
* \param pUserCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD context
*
* \param pMsg
* USB Message
*
* \return
* None
*
 *******************************************************************************/
void Cy_USB_AppBusSpeedCallback(void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/****************************************************************************
* Function Name: Cy_USB_AppSetupCallback
******************************************************************************
* Summary:
* Callback function will be invoked by USBD when SETUP packet is received
*
* \param pUserCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD context
*
* \param pMsg
* USB Message
*
* \return
* None
*
 *******************************************************************************/
void Cy_USB_AppSetupCallback(void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USB_AppSuspendCallback
****************************************************************************//**
*
* Callback function will be invoked by USBD when Suspend signal/message is detected
*
* \param pUserCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD context
*
* \param pMsg
* USB Message
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppSuspendCallback(void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USB_AppResumeCallback
****************************************************************************//**
*
* Callback function will be invoked by USBD when Resume signal/message is detected
*
* \param pUserCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD context
*
* \param pMsg
* USB Message
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppResumeCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/****************************************************************************
* Function Name: Cy_USB_AppSetIntfCallback
******************************************************************************
* Summary:
* Callback function will be invoked by USBD when SET Interface is called
*
* \param pUserCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD context
*
* \param pMsg
* USB Message
*
* \return
* None
*
 *******************************************************************************/
void Cy_USB_AppSetIntfCallback(void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USB_AppL1SleepCallback
****************************************************************************//**
*
* This Function will be called by USBD layer when L1 Sleep message comes.
*
* \param pUserCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD context
*
* \param pMsg
* USB Message
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppL1SleepCallback(void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USB_AppL1ResumeCallback
****************************************************************************//**
*
* This Function will be called by USBD layer when L1 Resume message comes.
*
* \param pUserCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD context
*
* \param pMsg
* USB Message
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppL1ResumeCallback(void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USB_AppZlpCallback
****************************************************************************//**
*
* This Function will be called by USBD layer when ZLP message comes
*
* \param pUserCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD context
*
* \param pMsg
* USB Message
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppZlpCallback(void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USB_AppSlpCallback
****************************************************************************//**
*
* This Function will be called by USBD layer when SLP message comes.
*
* \param pUserCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD context
*
* \param pMsg
* USB Message
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppSlpCallback(void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: CyApp_RegisterUsbDescriptors
****************************************************************************//**
*
* This function register USB descriptor with USBD layer
*
* \param pAppCtxt
* application layer context pointer.
*
* \param usbSpeed
* USBD speed
*
* \return
* None
*
********************************************************************************/
void CyApp_RegisterUsbDescriptors(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_en_usb_speed_t usbSpeed);

/*******************************************************************************
* Function name: Cy_USB_AppQueueWrite
****************************************************************************//**
*
* Queue USBHS Write on the USB endpoint
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNumber
* Endpoint number
*
* \param pBuffer
* Data Buffer Pointer
*
* \param dataSize
* DataSize to send on USB bus
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppQueueWrite(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,  
                         uint8_t *pBuffer, uint16_t dataSize);

/*******************************************************************************
* Function name: Cy_USB_AppQueueRead
****************************************************************************//**
*
* Function to queue read operation on an OUT endpoint.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNumber
* USB endpoint number
*
* \param pBuffer
* data buffer pointer
*
* \param dataSize
* data size
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppQueueRead(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize);

/*******************************************************************************
* Function name: Cy_USB_AppReadShortPacket
****************************************************************************//**
*
* Function to modify an ongoing DMA read operation to take care of a short packet.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNumber
* USB endpoint number
*
* \param pktSize
* data packet size
*
*
* \return
* Total size of data in the DMA buffer including data which was already read by the channel
*
********************************************************************************/ 
uint16_t Cy_USB_AppReadShortPacket(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint16_t pktSize);

/*****************************************************************************
* Function Name: Cy_USB_AppDisableEndpDma
******************************************************************************
* Summary:
* This function de-inits all active USB DMA channels as part of USB disconnect process.
*
* Parameters:
* \param pAppCtxt
* application layer context pointer
*
* Return:
* void
*****************************************************************************/
void Cy_USB_AppDisableEndpDma(cy_stc_usb_app_ctxt_t *pAppCtxt);

/*****************************************************************************
* Function Name: Cy_U3V_AppHandleSendCompletion
******************************************************************************
* Summary:
* Function that handles DMA transfer completion on the USB-HS BULK-IN
* endpoint. This is equivalent to the receipt of a consume event in the USB-SS use
* case and we can discard the active data buffer on the LVDS side.
*
* \param pAppCtxt
* application layer context pointer.
*
* \return
* None
*
 *******************************************************************************/
void Cy_U3V_AppHandleSendCompletion(cy_stc_usb_app_ctxt_t *pAppCtxt);

/*****************************************************************************
* Function Name: Cy_U3V_AppResponseSendCompletion
******************************************************************************
* Summary:
* Function that handles DMA transfer completion on the USB-HS BULK-IN
* endpoint. This is equivalent to the receipt of a cosnume event in the USB-SS use
* case.
*
* \param pAppCtxt
* application layer context pointer.
*
* \return
* None
*
 *******************************************************************************/
void Cy_U3V_AppResponseSendCompletion(cy_stc_usb_app_ctxt_t *pAppCtxt);

/*****************************************************************************
* Function Name: Cy_U3V_AppCommandRecvCompletion
******************************************************************************
* Summary:
* Function that handles DMA transfer completion on the USB-HS BULK-OUT
* endpoint used to receive U3V control commands.
*
* \param pAppCtxt
* application layer context pointer.
*
* \return
* None
*
 *******************************************************************************/
void Cy_U3V_AppCommandRecvCompletion(cy_stc_usb_app_ctxt_t *pAppCtxt);

/*******************************************************************************
* Function name: Cy_USB_AppInitDmaIntr
****************************************************************************//**
*
* Function to register an ISR for the DMA channel associated with an endpoint
*
* \param endpNumber
* USB endpoint number
*
* \param endpDirection
* Endpoint direction
*
* \param userIsr
*  ISR function pointer. Can be NULL if interrupt is to be disabled.
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppInitDmaIntr(uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection, cy_israddress userIsr);

/*******************************************************************************
* Function name: Cy_USB_AppClearDmaInterrupt
****************************************************************************//**
*
* Clear DMA Interrupt
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNumber
* Endpoint number
*
* \param endpDirection
* Endpoint direction
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppClearDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt, uint32_t endpNumber, 
                                    cy_en_usb_endp_dir_t endpDirection);

/*****************************************************************************
 * Function Name: Cy_USB_SSConnectionEnable
 *****************************************************************************
 * Summary
 *  PSVP specific USB connect function.
 *
 * Parameters:
 *  \param pAppCtxt
 *  Pointer to application context structure.
 *
 * Return:
 *  void
 ****************************************************************************/
bool Cy_USB_SSConnectionEnable(cy_stc_usb_app_ctxt_t *pAppCtxt);

/*****************************************************************************
 * Function Name: Cy_USB_SSConnectionDisable
 *****************************************************************************
 * Summary
 *  PSVP specific USB disconnect function.
 *
 * Parameters: 
 *  \param pAppCtxt
 *  Pointer to application context structure.
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_USB_SSConnectionDisable(cy_stc_usb_app_ctxt_t *pAppCtxt);

/*****************************************************************************
 * Function Name: Cy_USB_DisableUsbBlock
 ******************************************************************************
 * Summary:
 *  Function to disable the USB32DEV IP block after terminating current
 *  connection.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_DisableUsbBlock (void);

/*****************************************************************************
 * Function Name: Cy_USB_EnableUsbBlock
 ******************************************************************************
 * Summary:
 *  Function to enable the USB32DEV IP block before enabling a new USB
 *  connection.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_EnableUsbBlock(void);

/*****************************************************************************
 * Function Name: Cy_LVDS_LVCMOS_Init
 *****************************************************************************
 * Summary
 *  Initialize the LVDS interface. Currently, only the SIP #0 is being initialized
 *  and configured to allow transfers into the HBW SRAM through DMA.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_LVDS_LVCMOS_Init(void);

/*****************************************************************************
 * Function Name: Cy_U3V_CommandChannel_ISR
 *****************************************************************************
 * 
 * Handler for interrupts from the DataWire channel used to receive U3V commands
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_U3V_CommandChannel_ISR(void);

/*****************************************************************************
 * Function Name: Cy_U3V_ResponseChannel_ISR
 *****************************************************************************
 * 
 * Handler for interrupts from the DataWire channel used to send U3V responses
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_U3V_ResponseChannel_ISR(void);

/*****************************************************************************
 * Function Name: Cy_U3V_StreamChannel_ISR
 *****************************************************************************
 * Summary
 * Handler for interrupts from the DataWire channel used to send U3V video stream
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_U3V_StreamChannel_ISR(void);

/*****************************************************************************
* Function Name: Cy_USB_AppPrintUsbEventLog
******************************************************************************
* Summary:
*  Function to print out the USB event log buffer content.
*
* Parameters:
*  \param pAppCtxt
*  Pointer to application context data structure.
*  \param pSSCal
*  Pointer to SSCAL context data structure.
*
* Return:
*  void
*****************************************************************************/
void Cy_USB_AppPrintUsbEventLog(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usbss_cal_ctxt_t *pSSCal);

/*****************************************************************************
* Function Name: Cy_FPGAPhyLinkTraining
******************************************************************************
*
*  I2C wRites to FPGA to set up Phy & Link trianing patterns
*
* Parameters:
* 
*
* Return:
*  0 for read success, error code for unsuccess.
*****************************************************************************/
cy_en_scb_i2c_status_t Cy_FPGAPhyLinkTraining(void);

/*****************************************************************************
* Function Name: Cy_U3V_AppDSIStop
******************************************************************************
* Summary:
* This function stops the DSI stream channel
*
* Parameters:
* \param pAppCtxt
* application layer context pointer
*
* \param pUsbdCtxt
* USBD layer context pointer
*
* Return:
* void
*****************************************************************************/
void Cy_U3V_AppDSIStop (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*****************************************************************************
* Function Name: Cy_U3V_AppDSIStart
******************************************************************************
* Summary:
* This function starts the DSI channel for streaming
*
* Parameters:
* \param pAppCtxt
* application layer context pointer
*
* \param pUsbdCtxt
* USBD layer context pointer
*
* Return:
* void
*****************************************************************************/
void Cy_U3V_AppDSIStart (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);


/*****************************************************************************
* Function Name: Cy_U3V_AppFPGAParamsUpdate
******************************************************************************
* Summary:
*  FPGA Register Writes. FPGA is updated with video resolution height, width & fps
*
* Parameters:
* \param width
* Video resolution width
*
* \param height
* Video resolution height
*
* \param fps
* Video frame rate
* 
*
* Return:
*  0 for read success, error code for unsuccess.
*****************************************************************************/
cy_en_scb_i2c_status_t Cy_U3V_AppFPGAParamsUpdate(uint32_t width, uint32_t height,uint8_t fps);

/*****************************************************************************
 * Function Name: Cy_Update_LvdsLinkClock
 *****************************************************************************
 * Summary
 *  This function updates the clock which the LVCMOS link layer is using while
 *  operating in link loopback mode. If the active USB connection is USB-HS,
 *  the clock needs to be slowed down so that there is no overflow happening
 *  on the interface.
 *
 * Parameters:
 *  isHs: Whether active USB connection is USB-HS.
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_Update_LvdsLinkClock (bool isHs);

/*****************************************************************************
 * Function Name: Cy_Update_Metadata
 *****************************************************************************
 * Summary
 *  This function updates the metadata
 *
 * Parameters:
 * \param height
 *  Image height
 * 
 *  \param width
 *  Image width
 * 
 *  \param imageSize
 *  image size (in bytes)
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_Update_Metadata(uint32_t height, uint32_t width, uint32_t imageSize);

/****************************************************************************
* Function Name: Cy_U3V_AppHandleProduceEvent
******************************************************************************
* Summary:
*  Function that handles a produce event indicating receipt of data through
*  the LVDS ingress socket.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pChHandle
* Pointer to DMA channel structure.
*
* \return
* None
*
 *******************************************************************************/
void
Cy_U3V_AppHandleProduceEvent (cy_stc_usb_app_ctxt_t  *pAppCtxt, cy_stc_hbdma_channel_t *pChHandle);

/*******************************************************************************
* Function name: Cy_CheckStatus
****************************************************************************//**
*
* Description: Function that handles prints error log
*
* \param function
* Pointer to function
*
* \param line
* Line number where error is seen
*
* \param condition
*  condition of failure
*
* \param value
*  error code
*
* \param isBlocking
*  blocking function
*
* \return
* None
*
********************************************************************************/
void Cy_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking);

/*******************************************************************************
* Function name: Cy_USB_FailHandler
****************************************************************************//**
*
* Description: Error Handler
*
*
* \return
* None
*
********************************************************************************/
void Cy_FailHandler(void);

/*******************************************************************************
* Function name: Cy_CheckStatusHandleFailure
****************************************************************************//**
*
* Description: Function that handles prints error log
*
* \param function
* Pointer to function
*
* \param line
* LineNumber where error is seen
*
* \param condition
* Line number where error is seen
*
* \param value
*  error code
*
* \param isBlocking
*  blocking function
*
* \param failureHandler
*  failure handler function
*
* \return
* None
*
********************************************************************************/
void Cy_CheckStatusAndHandleFailure(const char *function, uint32_t line, uint8_t condition,
                             uint32_t value, uint8_t isBlocking, void (*failureHandler)());



#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_H_ */

/* End of File */

