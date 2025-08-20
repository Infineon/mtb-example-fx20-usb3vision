/***************************************************************************//**
* \file cy_usb_app.c
* \version 1.0
*
* Implements the USB data handling part of the USB3 Vision application.
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

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usbss_cal_drv.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_usbd.h"
#include "cy_usbhs_dw_wrapper.h"
#include "cy_usb_u3v_device.h"
#include "cy_usb_app.h"
#include "cy_debug.h"
#include "cy_lvds.h"
#include "cy_u3v_xml.h"
#include "cy_usb_i2c.h"
#include "cy_usb_qspi.h"
#include "cy_u3v_device_config.h"
#include "cy_video_inmem.h"

extern uint8_t CyFxUSB20DeviceDscr[];
extern cy_stc_usb_app_ctxt_t appCtxt;

#if FPGA_ENABLE
extern cy_stc_hbdma_buf_mgr_t HBW_BufMgr;
bool glIsFPGAConfigComplete = false;
bool glIsFPGARegConfigured = false;
#endif /* FPGA_ENABLE */

static volatile bool glU3VDevConfigured = false;
volatile bool glU3VIsApplnActive = false;

uint32_t glU3VBufCounter = CY_USB_U3V_LEADER_BUFFER_NO;
uint32_t glU3VFrameBuffer = CY_USB_U3V_BUFFER_NO_3480_2160;
uint16_t glU3VColorbarSize  = COLORBAR_BAND_COUNT_4K;

static uint8_t  *glU3vCmdBuffer = NULL;             /* U3V Command Buffer */
static uint8_t  *glU3vResponseBuffer = NULL;        /* U3V Response Buffer */

cy_stc_u3v_flags_t   glFlags;
cy_stc_u3v_img_params_t glFrameParams;

/* Global HBWSS DMA channel handle */
static cy_stc_hbdma_channel_t  *glChHandleDSI;      /* Stream Channel pointer. */
static cy_stc_hbdma_channel_t  *glChHandleDCIRsp;   /* U3V Response Channel pointer. */
static cy_stc_hbdma_channel_t  *glChHandleDCICmd;   /* U3V Command Channel pointer. */

#if ((!LVDS_LB_EN) && (!FPGA_ADDS_HEADER))
static bool glIsLeader   = true;                    /* Leader Block Tracker */
static bool glIsTrailer  = false;                   /* Trailer Block Tracker */
#endif

static uint32_t __attribute__ ((section(".hbBufSection"), used)) __attribute__ ((aligned (32))) Ep0TestBuffer[32U];
static uint32_t __attribute__ ((section(".hbBufSection"), used)) __attribute__ ((aligned (32))) SetSelDataBuffer[8U];

/*****************************************************************************
* Function Name: Cy_USB_AppSendMsgToTask
******************************************************************************
* Summary:
* Function that sends a message to the USB application task.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pMsg
* message pointer to be sent to the task.
*
* \return
* None
*
 *******************************************************************************/
static void Cy_USB_AppSendMsgToTask (
        cy_stc_usb_app_ctxt_t *pAppCtxt,
        cy_stc_usbd_app_msg_t *pMsg)
{
    BaseType_t status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((pAppCtxt != NULL) && (pMsg != NULL)) {

#if (FAST_DMA_ISR)
        /* If we are doing U3V streaming using manual channel in Gen2x2 connection, block
         * LVDS ingress DMA interrupt here so that other tasks have time to run.
         */
        if (pAppCtxt->devSpeed == CY_USBD_USB_DEV_SS_GEN2X2) {
            pAppCtxt->dmaInterruptDisabled = true;
            NVIC_DisableIRQ(lvds2usb32ss_lvds_dma_adap0_int_o_IRQn);
            NVIC_DisableIRQ(lvds2usb32ss_lvds_dma_adap1_int_o_IRQn);
        }
#endif /* ((FAST_DMA_ISR)) */

        /* Send the message to the task. */
        status = xQueueSendFromISR(pAppCtxt->u3vMessageQueue, pMsg, &xHigherPriorityTaskWoken);
        (void)status;
        (void)xHigherPriorityTaskWoken;
    }
}

/*****************************************************************************
* Function Name: Cy_U3V_AppUpdateStreamingParams
******************************************************************************
* Summary:
* Update local streaming parameters
*
* Parameters:
*
*
* Return:
*  void
*****************************************************************************/
static void
Cy_U3V_AppUpdateStreamingParams(cy_stc_usb_app_ctxt_t *pAppCtxt, uint32_t height, uint32_t width, uint32_t fps, uint32_t pixelSize)
{
    glFrameParams.bitsPerPixel = pixelSize;
    glFrameParams.height = height;
    glFrameParams.width = width;
    glFrameParams.fps = fps;

    glFrameParams.imageSize = (glFrameParams.width * glFrameParams.height * glFrameParams.bitsPerPixel)/8;
    glFrameParams.totalFrameSize = glFrameParams.imageSize;
    glFrameParams.finalTransfer1Size = (glFrameParams.imageSize % STREAM_DMA_BUFFER_SIZE);
    pAppCtxt->isPartialBuf = 0;

    if(glFrameParams.finalTransfer1Size)
    {
        pAppCtxt->isPartialBuf = 1;
        pAppCtxt->partialBufSize = glFrameParams.finalTransfer1Size;
    }

    glU3VBufCounter = CY_USB_U3V_LEADER_BUFFER_NO;

    glU3VFrameBuffer =  (glFrameParams.imageSize/STREAM_DMA_BUFFER_SIZE)  + pAppCtxt->isPartialBuf + (2*CY_USB_U3V_LEADER_BUFFER_NO);

    if(height == CY_U3V_RESOLUTION_4K_HEIGHT &&  width == CY_U3V_RESOLUTION_4K_WIDTH)
    {
        glU3VColorbarSize = COLORBAR_BAND_COUNT_4K;
    }
    else if (height == CY_U3V_RESOLUTION_1080P_HEIGHT &&  width == CY_U3V_RESOLUTION_1080P_WIDTH)
    {
        glU3VColorbarSize = COLORBAR_BAND_COUNT_1080P;
    }
    else if(height == CY_U3V_RESOLUTION_720P_HEIGHT &&  width == CY_U3V_RESOLUTION_720P_WIDTH)
    {
        glU3VColorbarSize = COLORBAR_BAND_COUNT_720P;
    }
    else if(height == CY_U3V_RESOLUTION_VGA_HEIGHT &&  width == CY_U3V_RESOLUTION_VGA_WIDTH)
    {
        glU3VColorbarSize = COLORBAR_BAND_COUNT_480P;
    }
    else
    {
        LOG_ERROR("Resolution not supported: Width %d Height %d \n\r", width, height);
    }

#if INMD_EN
    Cy_Update_Metadata(glFrameParams.height, glFrameParams.width,  glFrameParams.imageSize);
#endif

    LOG_COLOR("Stream: Width %d Height %d FPS %d Image Size %d \n\r", glFrameParams.width, glFrameParams.height, glFrameParams.fps, glFrameParams.imageSize);
    DBG_APP_INFO("Total buffer count %d partial buf size %d \n\r",glU3VFrameBuffer, glFrameParams.finalTransfer1Size);
}

#if LVDS_LB_EN
volatile uint32_t lvdsConsCount = 0;
volatile bool lvdsLpbkBlocked = false;
cy_stc_hbdma_channel_t lvdsLbPgmChannel;

/*****************************************************************************
* Function Name: Cy_LVDS_InitLbPgm
******************************************************************************
* Summary:
* Funtion to initialize loopback pattern generator

* Parameters:
* \param buffStat
* pointer to buffer
*
* \param lbPgmConfig
* pointer to loopback configuration
*
* Return:
* none
*****************************************************************************/
void Cy_LVDS_InitLbPgm(cy_stc_hbdma_buff_status_t *buffStat, cy_stc_lvds_loopback_config_t *lbPgmConfig)
{
    cy_stc_lvds_loopback_mem_t lbPgm;
    lbPgm.dataWord0 =   ((0x00000001) |
                        (lbPgmConfig->start << 1) |
                        (lbPgmConfig->end << 2) |
                        (lbPgmConfig->dataMode << 4) |
                        (lbPgmConfig->repeatCount << 8) |
                        (lbPgmConfig->dataSrc << 20));
    lbPgm.dataWord1 =   ((lbPgmConfig->ctrlByte) |
                        (lbPgmConfig->ctrlBusVal << 12));
    lbPgm.dataWord2 = lbPgmConfig->dataL;
    lbPgm.dataWord3 = lbPgmConfig->dataH;
    lbPgmConfig->pBuffer = buffStat->pBuffer + lbPgmConfig->lbPgmCount * 16;
    memcpy(lbPgmConfig->pBuffer, &lbPgm, 16);
    lbPgmConfig->lbPgmCount += 1;
}

/*****************************************************************************
* Function Name: Cy_LVDS_CommitColorbarData(void)
******************************************************************************
* Summary:
* Funtion to commit colorbar data to receiver channel

* Parameters:
* \param none
*
* Return:
* none
*****************************************************************************/
void Cy_LVDS_CommitColorbarData(void)
{
    uint32_t loop = 0, lineCount = 0;
    cy_stc_hbdma_buff_status_t buffStat;
    cy_stc_lvds_loopback_config_t lbPgmConfig =
    {
        .lbPgmCount = 0,
        .pBuffer = NULL,
        .start = 0,
        .end = 0,
        .dataMode = 0x00,
        .dataSrc = 0x00,
        .ctrlByte = 0x01,
        .repeatCount = 0x0001,
        .dataL = 0x00000000,
        .dataH = 0xDEADBEEF,
        .ctrlBusVal = 0x00000000
    };

    if (Cy_HBDma_Channel_GetBuffer(&lvdsLbPgmChannel, &buffStat) != CY_HBDMA_MGR_SUCCESS) {
        DBG_APP_ERR("GetLpbkBuf 1 failed\r\n");
        return;
    }

    lbPgmConfig.pBuffer = buffStat.pBuffer;

    /* Start command */
    lbPgmConfig.start = 1;
    Cy_LVDS_InitLbPgm(&buffStat, &lbPgmConfig);

    /* Few cycles of IDLE */
    lbPgmConfig.start = 0;
    for(loop = 0; loop < 10; loop++)
    {
        Cy_LVDS_InitLbPgm(&buffStat, &lbPgmConfig);
    }

    /* DATA control byte*/
    lbPgmConfig.ctrlByte = 0x80;
    for (lineCount = 0; lineCount < (360 / glU3VColorbarSize); lineCount++)
    {
        /* BAND1 of colorbar */
        lbPgmConfig.dataL = BAND1_COLOR_YUYV;
        lbPgmConfig.dataH = BAND1_COLOR_YUYV;
        for(loop = 0; loop < glU3VColorbarSize; loop++)
        {
            Cy_LVDS_InitLbPgm(&buffStat, &lbPgmConfig);
        }

        /* BAND2 of colorbar */
        lbPgmConfig.dataL = BAND2_COLOR_YUYV;
        lbPgmConfig.dataH = BAND2_COLOR_YUYV;
        for(loop = 0; loop < glU3VColorbarSize; loop++)
        {
            Cy_LVDS_InitLbPgm(&buffStat, &lbPgmConfig);
        }

        /* BAND3 of colorbar */
        lbPgmConfig.dataL = BAND3_COLOR_YUYV;
        lbPgmConfig.dataH = BAND3_COLOR_YUYV;
        for(loop = 0; loop < glU3VColorbarSize; loop++)
        {
            Cy_LVDS_InitLbPgm(&buffStat, &lbPgmConfig);
        }

        /* BAND4 of colorbar */
        lbPgmConfig.dataL = BAND4_COLOR_YUYV;
        lbPgmConfig.dataH = BAND4_COLOR_YUYV;
        for(loop = 0; loop < glU3VColorbarSize; loop++)
        {
            Cy_LVDS_InitLbPgm(&buffStat, &lbPgmConfig);
        }

        /* BAND5 of colorbar */
        lbPgmConfig.dataL = BAND5_COLOR_YUYV;
        lbPgmConfig.dataH = BAND5_COLOR_YUYV;
        for(loop = 0; loop < glU3VColorbarSize; loop++)
        {
            Cy_LVDS_InitLbPgm(&buffStat, &lbPgmConfig);
        }

        /* BAND6 of colorbar */
        lbPgmConfig.dataL = BAND6_COLOR_YUYV;
        lbPgmConfig.dataH = BAND6_COLOR_YUYV;
        for(loop = 0; loop < glU3VColorbarSize; loop++)
        {
            Cy_LVDS_InitLbPgm(&buffStat, &lbPgmConfig);
        }

        /* BAND7 of colorbar */
        lbPgmConfig.dataL = BAND7_COLOR_YUYV;
        lbPgmConfig.dataH = BAND7_COLOR_YUYV;
        for(loop = 0; loop < glU3VColorbarSize; loop++)
        {
            Cy_LVDS_InitLbPgm(&buffStat, &lbPgmConfig);
        }

        /* BAND8 of colorbar */
        lbPgmConfig.dataL = BAND8_COLOR_YUYV;
        lbPgmConfig.dataH = BAND8_COLOR_YUYV;
        for(loop = 0; loop < glU3VColorbarSize; loop++)
        {
            Cy_LVDS_InitLbPgm(&buffStat, &lbPgmConfig);
        }
    }

    /* End of program command */
    lbPgmConfig.end = 0x1;
    lbPgmConfig.ctrlByte = 0x01;
    lbPgmConfig.repeatCount = 0x00000000;
    Cy_LVDS_InitLbPgm(&buffStat, &lbPgmConfig);

    /* Commit the data into buffer */
    buffStat.count = 16 * (lbPgmConfig.lbPgmCount);
    Cy_HBDma_Channel_CommitBuffer(&lvdsLbPgmChannel, &buffStat);
}
#endif /* LVDS_LB_EN */

/*****************************************************************************
* Function Name: Cy_U3V_AppFlushAndResetDma
******************************************************************************
* Summary:
* Function to flush & reset DMA channel
*
* Parameters:
* \param pAppCtxt
* application layer context pointer
*
* \param dmaChannel
* pointert to DMA channle handle
*
* \param endpoint
* endpoint number
*
* \param dir
* endpoint direction
*
* \param flag
*
* Return:
* void
*****************************************************************************/
static void Cy_U3V_AppFlushAndResetDma(
    cy_stc_usb_app_ctxt_t *pAppCtxt,
    cy_stc_hbdma_channel_t *dmaChannel,
    uint8_t endpoint,
    cy_en_usb_endp_dir_t dir,
    bool flag
)
{
    /* Enable clock control across EP reset operation to ensure that the EP is properly flushed. */
    Cy_USBSS_Cal_ClkStopOnEpRstEnable(pAppCtxt->pUsbdCtxt->pSsCalCtxt, true);
    Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpoint, dir, true);
    Cy_SysLib_DelayUs(100);
    /* Reset the DMA channel: HBW or DataWire. */
    Cy_HBDma_Channel_Reset(dmaChannel);
    if (Cy_USBD_GetDeviceSpeed(pAppCtxt->pUsbdCtxt) < CY_USBD_USB_DEV_SS_GEN1) {
        Cy_USBHS_App_ResetEpDma(&(pAppCtxt->endpInDma[endpoint]));
    }
    Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt, endpoint, dir);
    Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt, endpoint, dir, flag);
    Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpoint, dir, false);
    Cy_SysLib_Delay(1);
    Cy_USBSS_Cal_ClkStopOnEpRstEnable(pAppCtxt->pUsbdCtxt->pSsCalCtxt, false);
    DBG_APP_TRACE("EP [0x%x]: DMA Channel (0x%x) flushed & reset\r\n", dir?0x80|endpoint:endpoint, *dmaChannel);
}

/*****************************************************************************
* Function Name: Cy_USB_App_KeepLinkActive
******************************************************************************
* Summary:
* Function to keep USB link active
*
* Parameters:
* \param pAppCtxt
* application layer context pointer
*
* Return:
* void
*****************************************************************************/
static void Cy_USB_App_KeepLinkActive (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
#if USB3_LPM_ENABLE
    if (pAppCtxt->isLpmEnabled) {
        /* Disable LPM for 100 ms after any DMA transfers have been completed. */
        DBG_APP_INFO("Disabling LPM\r\n");
        Cy_USBD_LpmDisable(pAppCtxt->pUsbdCtxt);
        pAppCtxt->isLpmEnabled  = false;
    }

    pAppCtxt->lpmEnableTime = Cy_USBD_GetTimerTick() + 100;
#endif /* USB3_LPM_ENABLE */
}

/*****************************************************************************
* Function Name: Cy_U3V_WaitForLastFrameConsume
******************************************************************************
* Summary:
* Function to wait for last U3V to be consumed
*
* Parameters:
* \param
* None
*
* Return:
* void
*****************************************************************************/
void Cy_U3V_WaitForLastFrameConsume(void)
{
    uint16_t timeout = 0;
    while(!glFlags.lastFrameConsumed){
        timeout++;
        vTaskDelay(pdMS_TO_TICKS(1));
        if(timeout > U3V_WAIT_FOR_STOP_TIMEOUT_MS) {
            DBG_APP_TRACE("Last frame timed out\r\n");
            glFlags.lastFrameConsumed = true;
            break;
        }
    }
    if(timeout < U3V_WAIT_FOR_STOP_TIMEOUT_MS){
        DBG_APP_TRACE("Last packet consumption completed in [%d] ms after STOP Notify\r\n", timeout);
    }
}

/*****************************************************************************
* Function Name: Cy_U3V_AppSendResponse
******************************************************************************
* Summary:
* Function to send response on DCI channel
*
* Parameters:
* \param rspBuffer
*  pointer to buffer
*
* \param length
*  size of response
*
* Return:
* void
*****************************************************************************/
void
Cy_U3V_AppSendResponse (
        uint8_t* rspBuffer,
        uint16_t length,
        void *userCtxt,
        uint8_t u3vChNum)
{

    cy_stc_usb_app_ctxt_t* pAppCtxt = (cy_stc_usb_app_ctxt_t*)userCtxt;
    cy_en_hbdma_mgr_status_t  hbdma_stat = CY_HBDMA_MGR_SUCCESS;

    if(pAppCtxt == NULL)
    {
        DBG_APP_ERR("Cy_U3V_AppSendResponse: NULL user context \n\r");
        return;
    }

    if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1){
        hbdma_stat = Cy_HBDma_Channel_SendData(glChHandleDCIRsp, 0, rspBuffer, length);
    } else{
        /* Wait until ongoing DSI buffer transfer is finished. */
        if (glU3VIsApplnActive) {
            Cy_SysLib_Delay(10);
        }
        Cy_USB_AppQueueWrite(pAppCtxt, CY_U3V_EP_DCI_RSP, rspBuffer, length);
        Cy_SysLib_Delay(1);

        /* We can re-enable streaming endpoint interrupt once DCI command has been handled. */
        NVIC_EnableIRQ(cpuss_interrupts_dw1_0_IRQn + CY_U3V_EP_DSI_STREAM);
    }

    if (hbdma_stat != CY_HBDMA_MGR_SUCCESS) {
        DBG_APP_ERR("DCIRsp SendData HBDMA Err:%x\r\n",hbdma_stat);
        Cy_U3V_AppFlushAndResetDma(pAppCtxt,glChHandleDCIRsp, CY_U3V_EP_DCI_RSP, CY_USB_ENDP_DIR_IN, true);
    }
    return;
}

/*****************************************************************************
* Function Name: CY_U3V_AppDevEvent
******************************************************************************
* Summary:
* Function to handle events from U3V library
*
* Parameters:
* \param regAddr
*  register address
*
* \param regVal
*  register value
*
* \param event
*  U3V event
*
* Return:
* void
*****************************************************************************/
static void CY_U3V_AppDevEvent (unsigned long regAddr, unsigned long regVal, cy_en_u3v_dev_event_t event,
                                void *userCtxt, uint8_t u3vChNum)
{
    /* Create event using regAddr if not supported in lib*/
    static bool heightChanged = false;
    static bool widthChanged = false;
    cy_stc_usb_app_ctxt_t* pAppCtxt = (cy_stc_usb_app_ctxt_t*)userCtxt;
#if FPGA_ENABLE
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
#endif /* FPGA_ENABLE */

    if(pAppCtxt == NULL)
    {
        DBG_APP_ERR("CY_U3V_AppDevEvent: NULL user context\r\n");
        return;
    }

    DBG_APP_TRACE("U3V Events: regAddr: 0x%x, regVal: 0x%x, event: 0x%x\r\n", regAddr, regVal,event);
    switch (event)
    {
    case U3V_EVT_STREAM_ENABLE:
        /* Enable streaming */
        DBG_APP_INFO("Stream Enabled\r\n");
        break;

    case U3V_EVT_STREAM_DISABLE:
        /* Disable streaming */
        Cy_U3V_WaitForLastFrameConsume();
        Cy_U3V_AppDSIStop(pAppCtxt, pAppCtxt->pUsbdCtxt);
        DBG_APP_INFO("Stream Disabled\r\n");
        break;

    case U3V_EVT_IMG_FORMAT_HEIGHT:
        /* Set height */
        glFrameParams.height = regVal;
        heightChanged = true;
        if(widthChanged && heightChanged){
            heightChanged = false;
            widthChanged = false;
            Cy_U3V_AppUpdateStreamingParams(pAppCtxt, glFrameParams.height, glFrameParams.width, glFrameParams.fps, glFrameParams.bitsPerPixel );
        }

        break;
    case U3V_EVT_IMG_FORMAT_WIDTH:
        /* Set Width */
        glFrameParams.width = regVal;
        widthChanged = true;
        if(widthChanged && heightChanged){
            heightChanged = false;
            widthChanged = false;
            Cy_U3V_AppUpdateStreamingParams(pAppCtxt, glFrameParams.height, glFrameParams.width, glFrameParams.fps, glFrameParams.bitsPerPixel);
        }
        break;

    case U3V_EVT_IMG_FORMAT_PXL_FORMAT:
        /* Set Pixel format */
        glFrameParams.pixelFormat = regVal;
        Cy_U3V_AppHandleFormatConversion(glFrameParams.pixelFormat);
        Cy_U3V_AppUpdateStreamingParams(pAppCtxt, glFrameParams.height, glFrameParams.width, glFrameParams.fps, glFrameParams.bitsPerPixel );
        break;

    case U3V_EVT_ACQUISITION_START:
        /* Acquition Started */
        Cy_U3V_AppDSIStart(pAppCtxt, pAppCtxt->pUsbdCtxt);
        break;

    case U3V_EVT_ACQUISITION_STOP:
        /* Acquition Stopped - Send fpga to stop notification*/
#if FPGA_ENABLE
        status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEV0_STREAM_ENABLE_ADDRESS, APP_STOP_NOTIFICATION ,FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
#endif /* FPGA_ENABLE */
        break;

    case U3V_EVT_ACQ_MODE_CONTINUOUS:
        /* Continuous frame acquition */
#if FPGA_ENABLE
        status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_U3V_STREAM_MODE_ADDRESS, DEV0_U3V_STREAM_MODE_CONTINUOUS ,FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
#endif /* FPGA_ENABLE */
        break;

    case U3V_EVT_ACQUISITION_SET_FPS:
        /* Set FPS */
        glFrameParams.fps = (regVal & 0x00FFF)/10;
        Cy_U3V_AppUpdateStreamingParams(pAppCtxt, glFrameParams.height, glFrameParams.width, glFrameParams.fps, glFrameParams.bitsPerPixel );
        break;

    default:

        break;
    }
}

/*****************************************************************************
* Function Name: Cy_U3vAppDbgLog
******************************************************************************
* Summary:
* Function to prints U3V library messaged
*
* Parameters:
* \param level
*  debug level
*
* \param message
*  pointer to message
*
* Return:
* void
*****************************************************************************/
static void Cy_U3vAppDbgLog(unsigned char level, char *message, ...)
{
    Cy_Debug_AddToLog(level,message);
}

/*****************************************************************************
* Function Name: Cy_U3V_InitializeInterface
******************************************************************************
* Summary:
* Function to initialize U3V
*
* Parameters:
* \param
* None
*
* Return:
* void
*****************************************************************************/
static void Cy_U3V_InitializeInterface(void)
{
    cy_stc_u3v_interface_config_t u3vInterface;
    cy_en_u3v_return_status_t ret;

    u3vInterface.u3vChannelNum = 0;
    u3vInterface.BRM_Config = &bootstrapConfig;
    u3vInterface.regBaseAddr = &regBaseConfig;
    u3vInterface.CY_U3V_DbgLog = Cy_U3vAppDbgLog;
    u3vInterface.Cy_U3V_DevEvent = CY_U3V_AppDevEvent;
    u3vInterface.CY_U3V_SendResponse = Cy_U3V_AppSendResponse;
    u3vInterface.userCtx = (void *)(&appCtxt);

    ret = Cy_U3V_InterfaceConfig(&u3vInterface, &u3vDevCtxt);
    if(ret){
        DBG_APP_ERR("U3V Interface init failed, Register all func pointers, Return Status: 0x%x \n\r", ret);
    }

    DBG_APP_INFO("U3V Interface init passed, Return Status: 0x%x \n\r", ret);
}

/*****************************************************************************
* Function Name: Cy_U3V_AppGpifIntr(void *pApp)
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
void Cy_U3V_AppGpifIntr(void *pApp)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    pAppCtxt = (cy_stc_usb_app_ctxt_t*)pApp;

#if LVDS_LB_EN
    pAppCtxt->u3vFlowCtrlFlag = false;

    /*
     * We need to stop the loopback source channel first so that data does not accumulate on the
     * ingress thread interface after the streaming channel is reset. We want to ensure that the
     * loopback source is stopped cleanly at the end of a buffer. Since the transfer of each
     * loopback DMA buffer takes the order of 68 us, we set a flag indicating that no more data
     * should be committed and then wait for 150 us (more than 2 * 68 us). By this time, it
     * is guaranteed that the socket will reach an idle state.
     */
    lvdsLpbkBlocked = true;
    Cy_SysLib_DelayUs(150);
    if (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
        Cy_SysLib_DelayUs(500);
    }

    Cy_HBDma_Channel_Reset(&lvdsLbPgmChannel);
    lvdsConsCount = 0;
#endif /* LVDS_LB_EN */

    DBG_APP_INFO("GPIFSM Interrupt\r\n");

    glFlags.blockFrame = false;
    Cy_U3V_AppFlushAndResetDma(pAppCtxt, glChHandleDSI, CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN, true);

    if (glU3VIsApplnActive)
    {
        glU3VIsApplnActive = false;
    }

    return;
}

#if FPGA_ENABLE
#if !FPGA_CONFIG_EN
/*****************************************************************************
* Function Name: Cy_IsFPGAConfigured(void)
******************************************************************************
* Summary:
* Function to check for CDONE status
*
* Parameters:
* None
*
* Return:
*  0 if FPGA not configured, 1u if FPGA configured.
*****************************************************************************/
static bool Cy_IsFPGAConfigured(void)
{
    bool cdoneVal = false;
    uint32_t maxWait = CDONE_WAIT_TIMEOUT;

    while (cdoneVal == false)
    {
        /*Check if CDONE is HIGH or FPGA is configured */
        cdoneVal = Cy_GPIO_Read(TI180_CDONE_PORT, TI180_CDONE_PIN);
        Cy_SysLib_Delay(1);
        maxWait--;
        if (!maxWait)
        {
            break;
        }
    }

    if((maxWait == 0) && (cdoneVal == false))
    {
        LOG_ERROR("FPGA not configured \r\n");
        return false;
    }
    else
    {
        DBG_APP_INFO("FPGA is configured \r\n");
    }

    return true;
}
#endif /* !FPGA_CONFIG_EN */

/*****************************************************************************
* Function Name: Cy_ConfigFpgaRegister(void)
******************************************************************************
* Summary:
*  FPGA Register Writes. FPGA is configured to send internally generated
*  colorbar data over FX2G3's SlaveFIFO Interface
*
* Parameters:
* None
*
*
* Return:
*  0 for read success, error code for unsuccess.
*****************************************************************************/
static cy_en_scb_i2c_status_t Cy_ConfigFpgaRegister(void)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
    int width = 3840;
    int height = 2160;

    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEV0_STREAM_ENABLE_ADDRESS, CAMERA_APP_DISABLE,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Write FPGA register to enable UVC */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_UVC_U3V_SELECTION_ADDRESS, FPGA_U3V_ENABLE,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if FPGA_ADDS_HEADER
    /* Enable pre-addition of U3V leader and trailer by FPGA. */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_U3V_HEADER_CTRL_ADDRESS, FPGA_U3V_HEADER_ENABLE_PAYLOAD,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    LOG_COLOR("FPGA added Leader Trailer\r\n");
#elif INMD_EN
    /* Configure FPGA to send Insert MetaData (INMD) command for U3V leader and trailer addition. */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_U3V_HEADER_CTRL_ADDRESS, FPGA_U3V_HEADER_ENABLE_METADATA,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    LOG_COLOR("INMD Leader Trailer\r\n");
#else
    /* Disable adding U3V header by FPGA. U3V leader and trailer will be added by FX firmware. */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_U3V_HEADER_CTRL_ADDRESS, FPGA_U3V_HEADER_DISABLE,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    LOG_COLOR("FX added Leader Trailer\r\n");
#endif /* FPGA_ADDS_HEADER */
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_ACTIVE_DEVICE_MASK_ADDRESS, 0x01,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);


    /* write FPGA register to Disable format conversion */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEV0_STREAM_MODE_ADDRESS, NO_CONVERSION,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_SOURCE_TYPE_ADDRESS, INTERNAL_COLORBAR,
                            FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if INTERLEAVE_EN
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_ACTIVE_THREAD_INFO_ADDRESS, DEV0_ACTIVE_THREAD_INFO_DUAL_THREAD,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    LOG_COLOR("Thread Interleave Enable\r\n");
#else
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_ACTIVE_THREAD_INFO_ADDRESS, DEV0_ACTIVE_THREAD_INFO_SINGLE_THREAD,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
#endif /* INTERLEAVE_EN */
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_THREAD1_INFO_ADDRESS, CY_LVDS_GPIF_THREAD_0,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if INTERLEAVE_EN
    /*A device can be connected to maximum 2 threads at once. Device 0 is connected to Thread 0 and Thread 1 */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_THREAD2_INFO_ADDRESS, CY_LVDS_GPIF_THREAD_1,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
#else
    /*A device can be connected to maximum 2 threads at once. Device 0 is connected to Thread 0 only*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_THREAD2_INFO_ADDRESS, CY_LVDS_GPIF_THREAD_0,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
#endif /*INTERLEAVE_EN */
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_THREAD1_SOCKET_INFO_ADDRESS, THREAD_0_SOCKET_0,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if INTERLEAVE_EN
/*
 * In Wide link mode, actual socket number each thread is connected to is derived from SSAD[kkk] control byte sent by FPGA.
 * Actual socket number =  (3'bkkk x 2) + tt[0] on adapter tt[1], where tt is the thread number sent in STAD[tt]
 * if FPGA sets kkk as 0 for Thread 0, actual socket number connected to Thread 0 is 0
 * if FPGA sets kkk as 0 for Thread 1, actual socket number connected to Thread 1 is 1
 */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_THREAD2_SOCKET_INFO_ADDRESS, THREAD_1_SOCKET_0,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
#else
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_THREAD2_SOCKET_INFO_ADDRESS, THREAD_0_SOCKET_0,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
#endif /* INTERLEAVE_EN*/
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Clear FPGA register during power up, this will get update when firmware detects HDMI */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_HDMI_SOURCE_INFO_ADDRESS, HDMI_DISCONNECT,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    LOG_COLOR("DMA Buffer Size = %d\r\n",FPGA_DMA_BUFFER_SIZE);
    /* Update DMA buffer size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_BUFFER_SIZE_MSB_ADDRESS, CY_GET_MSB(FPGA_DMA_BUFFER_SIZE),
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_BUFFER_SIZE_LSB_ADDRESS, CY_GET_LSB(FPGA_DMA_BUFFER_SIZE),
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_IMAGE_WIDTH_MSB_ADDRESS, CY_GET_MSB(width),
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_IMAGE_WIDTH_LSB_ADDRESS, CY_GET_LSB(width),
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_IMAGE_HEIGHT_MSB_ADDRESS, CY_GET_MSB(height),
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_IMAGE_HEIGHT_LSB_ADDRESS, CY_GET_LSB(height),
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEV0_FPS_ADDRESS, U3V_FPS,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    if (status != CY_SCB_I2C_SUCCESS){
        DBG_APP_ERR("FPGA register config failed\n\r\r\n");
    }
    else
    {
        LOG_COLOR("FPGA register config done\r\n");
    }
    return status;
}

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
cy_en_scb_i2c_status_t Cy_U3V_AppFPGAParamsUpdate(uint32_t width, uint32_t height,uint8_t fps)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;

    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_UVC_U3V_SELECTION_ADDRESS,FPGA_U3V_ENABLE,FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs (1000);
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEV0_IMAGE_WIDTH_LSB_ADDRESS,CY_GET_LSB(width),FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs (1000);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEV0_IMAGE_WIDTH_MSB_ADDRESS,CY_GET_MSB(width),FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    Cy_SysLib_DelayUs (1000);
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEV0_IMAGE_HEIGHT_LSB_ADDRESS,CY_GET_LSB(height),FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs (1000);
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEV0_IMAGE_HEIGHT_MSB_ADDRESS,CY_GET_MSB(height),FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs (1000);
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEV0_FPS_ADDRESS,CY_GET_LSB(fps),FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs (1000);
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEV0_U3V_STREAM_MODE_ADDRESS,DEV0_U3V_STREAM_MODE_CONTINUOUS,FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs (1000);
    /*Do I2C writes for format conversion if needed*/
    Cy_U3V_AppHandleFormatConversion(IMAGE_FORMAT);

    LOG_COLOR("Video (Default): [%d x %d] @ %d FPS\r\n",width, height, fps);

    if (status != CY_SCB_I2C_SUCCESS){
        DBG_APP_ERR("Cy_U3V_AppFPGAParamsUpdate: failed with err:%x\n\r",status);
    }

    return status;
}

/*****************************************************************************
* Function Name: Cy_FPGAPhyLinkTraining(void)
******************************************************************************
* Summary:
*  I2C wRites to FPGA to set up Phy & Link trianing patterns
*
* Parameters:
*
*
* Return:
*  0 for read success, error code for unsuccess.
*****************************************************************************/
cy_en_scb_i2c_status_t Cy_FPGAPhyLinkTraining(void)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;

#if LINK_TRAINING
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEV0_STREAM_ENABLE_ADDRESS, CAMERA_APP_DISABLE,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if LVCMOS_DDR_EN
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_PHY_LINK_CONTROL_ADDRESS, FPGA_LINK_CONTROL,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
#else
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_PHY_LINK_CONTROL_ADDRESS, FPGA_LINK_CONTROL | FPGA_PHY_CONTROL,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
#endif /*LVCMOS_DDR_EN*/
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(1000);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_LVDS_PHY_TRAINING_ADDRESS, PHY_TRAINING_PATTERN_BYTE,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(1000);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_LVDS_LINK_TRAINING_BLK_P0_ADDRESS, CY_DWORD_GET_BYTE0(LINK_TRAINING_PATTERN_BYTE),
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(1000);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_LVDS_LINK_TRAINING_BLK_P1_ADDRESS, CY_DWORD_GET_BYTE1(LINK_TRAINING_PATTERN_BYTE),
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(1000);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_LVDS_LINK_TRAINING_BLK_P2_ADDRESS, CY_DWORD_GET_BYTE2(LINK_TRAINING_PATTERN_BYTE),
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(1000);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_LVDS_LINK_TRAINING_BLK_P3_ADDRESS, CY_DWORD_GET_BYTE3(LINK_TRAINING_PATTERN_BYTE),
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(1000);
#endif /*LINK_TRAINING*/
    return status;
}

#endif /* FPGA_ENABLE */

/*****************************************************************************
* Function Name: Cy_U3V_AppTimerCb
******************************************************************************
* Summary:
* Timer used to handle prints in every 1 seconds.
*
* Parameters:
* \param xTimer
* timer handle
*
* Return:
* void
*****************************************************************************/
void
Cy_U3V_AppTimerCb (TimerHandle_t xTimer)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t status;
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    /* retrieve pAppCtxt */
    pAppCtxt = ( cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);
#if AUTO_DMA_EN
    uint32_t sckCount = 0;

    if (glU3VIsApplnActive == true)
    {
        /* Read the current socket count and then clear it. */
        sckCount = LVDSSS_LVDS->ADAPTER_DMA[0].SCK[0].SCK_COUNT;
        LVDSSS_LVDS->ADAPTER_DMA[0].SCK[0].SCK_COUNT = 0;

        /* Socket count represents number of bytes transferred.
            * Divide by frame size to calculate the number of frames
            * transferred.
        */
        pAppCtxt->fpsCount = (sckCount / glFrameParams.imageSize);
        xMsg.type = CY_USB_U3V_FPS_POLL_EVENT;
        xMsg.data[0]= pAppCtxt->fpsCount;
        pAppCtxt->frameSize = glFrameParams.imageSize;
        status = xQueueSendFromISR(appCtxt.u3vMessageQueue, &(xMsg), &(xHigherPriorityTaskWoken));
        ASSERT_NON_BLOCK(pdTRUE == status,status);;

        pAppCtxt->fpsCount = 0;
    }
#else
    if (glU3VIsApplnActive == true)
    {
        pAppCtxt->fpsPrint = pAppCtxt->fpsCount;
        if (pAppCtxt->fpsPrint){

            xMsg.type = CY_USB_U3V_FPS_POLL_EVENT;
            xMsg.data[0]= pAppCtxt->fpsPrint;
            status = xQueueSendFromISR(appCtxt.u3vMessageQueue, &(xMsg), &(xHigherPriorityTaskWoken));
            ASSERT_NON_BLOCK(pdTRUE == status,status);
        }
        pAppCtxt->fpsCount = 0;
    }
#endif
}   /* end of function  */

/*****************************************************************************
* Function Name: Cy_U3V_AppDSIHaltEventHandler
******************************************************************************
* Summary:
* Data streaming Interface (DSI) Halt Event handler
*
* Parameters:
* \param pAppCtxt
* application layer context pointer
*
* \param pUsbdCtxt
* USBD layer context pointer
*
* \param setClear
* Set endpoint HALT if true elase clear endpoint HALT condition
*
* Return:
* void
*****************************************************************************/
bool
Cy_U3V_AppDSIHaltEventHandler (
        cy_stc_usb_app_ctxt_t *pAppCtxt,
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
        bool setClear )
{
    bool isStall = false;

    if (setClear)   /* SET FEATURE */
    {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN, true);
    }
    else  /* CLEAR FEATURE */
    {
        Cy_U3V_AppFlushAndResetDma(pAppCtxt, glChHandleDSI, CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN, false);
        Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN, false);

#if LVDS_LB_EN
        pAppCtxt->u3vFlowCtrlFlag = false;

        /*
         * We need to stop the loopback source channel first so that data does not accumulate on the
         * ingress thread interface after the streaming channel is reset. We want to ensure that the
         * loopback source is stopped cleanly at the end of a buffer. Since the transfer of each
         * loopback DMA buffer takes the order of 68 us, we set a flag indicating that no more data
         * should be committed and then wait for 150 us (more than 2 * 68 us). By this time, it
         * is guaranteed that the socket will reach an idle state.
         */
        lvdsLpbkBlocked = true;
        Cy_SysLib_DelayUs(150);
        if (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
            Cy_SysLib_DelayUs(500);
        }

        Cy_HBDma_Channel_Reset(&lvdsLbPgmChannel);
        lvdsConsCount = 0;
#endif /* LVDS_LB_EN */
    }

    isStall = Cy_USBD_EndpIsStallSet(pUsbdCtxt, CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN);
    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);

    return isStall;
}

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
void
Cy_U3V_AppHandleFormatConversion(uint32_t imageFormat)
{
#if FPGA_ENABLE
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
#endif /* FPGA_ENABLE */
    switch (imageFormat){
        case MONO8:
#if FPGA_ENABLE
            status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_DEV0_STREAM_MODE_ADDRESS,
                      MONO_8_CONVERSION,
                      FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
            if (status == CY_SCB_I2C_SUCCESS){
                DBG_APP_INFO("Enabled YUV422 -> Mono Conversion\r\n");
            }
#endif /* FPGA_ENABLE */
            glFrameParams.bitsPerPixel = 8;
            break;

        case MONO12: /* Kept for testing 4K. Will result in incorrect image, but can test the frame size */

#if FPGA_ENABLE
            status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_DEV0_STREAM_MODE_ADDRESS,
                      YUV422_420_CONVERSION,
                      FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
            if (status == CY_SCB_I2C_SUCCESS){
                DBG_APP_INFO("Enabled YUV422 -> YUV420 Conversion\r\n");
            }
#endif
            glFrameParams.bitsPerPixel = 12;
            break;

        default:
#if FPGA_ENABLE
            status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_DEV0_STREAM_MODE_ADDRESS,
                      NO_CONVERSION,
                      FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
            if (status == CY_SCB_I2C_SUCCESS){
                DBG_APP_INFO("NO_CONVERSION\r\n");
            }
#endif
            glFrameParams.bitsPerPixel = 16;
            break;
    }
    glFrameParams.pixelFormat = imageFormat;
}

/*****************************************************************************
* Function Name: Cy_U3V_AppPendingAckTimerCb(TimerHandle_t xTimer)
******************************************************************************
* Summary:
* This Function will be called when timer expires. This function Send Pending Ack to Host.
*
* Parameters:
* \param xTimer
* Timer handle
*
* Return:
* void
*****************************************************************************/
void
Cy_U3V_AppPendingAckTimerCb(TimerHandle_t xTimer)
{
    uint16_t len = 0;
    uint8_t isBusy = 0;

    if(Cy_U3V_SendPendingAckResponse(&u3vDevCtxt,glU3vResponseBuffer, &len))
    {
        DBG_APP_ERR("Cy_U3V_SendPendingAckResponse failed with status\r\n");
    }

    if(Cy_U3V_IsDCIHandlerBusy(&u3vDevCtxt,&isBusy) == U3V_RET_STATUS_SUCCESS)
    {
        /* Restart timer if Command not processed */
        if (isBusy){
            xTimerReset(xTimer,0);
        }
    }
}

/***********************************************************************************************
* Function Name: Cy_USB_PrintEvtLogTimerCb(TimerHandle_t xTimer)
**************************************************************************************************
* Summary:
* This Function will be called when timer expires.This function print event log.
*
* Parameters:
* \param xTimer
* timer handle
*
* Return:
* void
************************************************************************************************/
void
Cy_USB_PrintEvtLogTimerCb(TimerHandle_t xTimer)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    /* retrieve pAppCtxt */
    pAppCtxt = ( cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);
    if (pAppCtxt->devState == CY_USB_DEVICE_STATE_CONFIGURED) {
        /*
         * Print out the contents of the USB event log buffer which has
         * logged event related to enumeration and line training.
         */
        Cy_USB_AppPrintUsbEventLog(pAppCtxt, pAppCtxt->pUsbdCtxt->pSsCalCtxt);
    }
}   /* end of function() */

/*****************************************************************************
* Function Name: Cy_USB_VbusDebounceTimerCallback(TimerHandle_t xTimer)
******************************************************************************
* Summary:
* Timer used to do debounce on VBus changed interrupt notification.
*
* \param xTimer
* Timer Handle
*
* \return
* None
*
 *******************************************************************************/
void
Cy_USB_VbusDebounceTimerCallback (TimerHandle_t xTimer)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);
    cy_stc_usbd_app_msg_t xMsg;

    if (pAppCtxt->vbusChangeIntr) {
        /* Notify the VCOM task that VBus debounce is complete. */
        xMsg.type = CY_USB_U3V_VBUS_CHANGE_DEBOUNCED;
        Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);

        /* Clear and re-enable the interrupt. */
        pAppCtxt->vbusChangeIntr = false;
        Cy_GPIO_ClearInterrupt(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN);
        Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 1);
    }
}   /* end of function  */

/*****************************************************************************
* Function Name: Cy_U3V_AppDCIStartStop
******************************************************************************
* Summary:
* This function starts Start/Stop DCI Interface application
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
void
Cy_U3V_AppDCIStartStop (
        cy_stc_usb_app_ctxt_t *pAppCtxt,
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    Cy_U3V_AppFlushAndResetDma(pAppCtxt, glChHandleDCICmd, CY_U3V_EP_DCI_CMD, CY_USB_ENDP_DIR_OUT, false);
    Cy_U3V_AppFlushAndResetDma(pAppCtxt, glChHandleDCIRsp, CY_U3V_EP_DCI_RSP, CY_USB_ENDP_DIR_IN, false);

    return;
}

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
void
Cy_U3V_AppDSIStop (
        cy_stc_usb_app_ctxt_t *pAppCtxt,
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
#if FPGA_ENABLE
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
#endif /* FPGA_ENABLE */

    DBG_APP_TRACE("AppDSIStop\r\n");
    if(glU3VIsApplnActive)
    {
#if FPGA_ENABLE
        status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEV0_STREAM_ENABLE_ADDRESS, CAMERA_APP_DISABLE,FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
#endif /* FPGA_ENABLE */

        /* Update the flag so that the application thread is notified of this. */
        glU3VIsApplnActive = false;

#if ((!LVDS_LB_EN) && (!FPGA_ADDS_HEADER))
        /* Next frame should start with Leader */
        glIsLeader  = true;
        glIsTrailer = false;
#endif

        Cy_SysLib_DelayUs(150);
        Cy_U3V_AppFlushAndResetDma(pAppCtxt, glChHandleDSI, CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN, true);

#if LVDS_LB_EN
        /* Next frame should start with Leader */
        glU3VBufCounter = CY_USB_U3V_LEADER_BUFFER_NO;
        pAppCtxt->u3vFlowCtrlFlag = false;

        /*
         * We need to stop the loopback source channel first so that data does not accumulate on the
         * ingress thread interface after the streaming channel is reset. We want to ensure that the
         * loopback source is stopped cleanly at the end of a buffer. Since the transfer of each
         * loopback DMA buffer takes the order of 68 us, we set a flag indicating that no more data
         * should be committed and then wait for 150 us (more than 2 * 68 us). By this time, it
         * is guaranteed that the socket will reach an idle state.
         */
        lvdsLpbkBlocked = true;
        Cy_SysLib_DelayUs(150);
        if (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
            Cy_SysLib_DelayUs(500);
        }

        Cy_HBDma_Channel_Reset(&lvdsLbPgmChannel);
        lvdsConsCount = 0;
#endif /* LVDS_LB_EN */

        DBG_APP_INFO("DSI Stop Done\r\n");
    }
    else
    {
        DBG_APP_INFO("DSI App already stopped\r\n");
    }
}

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
void
Cy_U3V_AppDSIStart (
        cy_stc_usb_app_ctxt_t *pAppCtxt,
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_en_hbdma_mgr_status_t mgrStatus = CY_HBDMA_MGR_SUCCESS;
#if FPGA_ENABLE
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
#endif /* FPGA_ENABLE */

    DBG_APP_INFO("AppDSIStart\r\n");

    if(!glU3VIsApplnActive)
    {
        pAppCtxt->glProd = 0;
        pAppCtxt->glCons = 0;
        pAppCtxt->fpsCount = 0;
        pAppCtxt->fpsPrint = 0;
        pAppCtxt->frameSizeTransferred = 0;
        pAppCtxt->frameSize = 0;
        glU3VIsApplnActive = true;

#if U3V_INMEM_EN
        if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            /* Allow data from multiple DMA buffers to be combined into one burst. */
            Cy_USBD_SetEpBurstMode(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN, true);
        }

        /* Place the EP in NAK mode before committing all the data. */
        Cy_USB_USBD_EndpSetClearNakNrdy(pUsbdCtxt, CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN, true);
        Cy_SysLib_DelayUs(125);

        /* Fill the buffers with the data to be sent. */
        Cy_U3V_MemFillBuffers();

#endif /*U3V_INMEM_EN*/

        /* Enable the DMA channel for data transfer. */
        pAppCtxt->u3vPendingBufCnt = 0;
        mgrStatus = Cy_HBDma_Channel_Enable(glChHandleDSI, 0);
        if (mgrStatus != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_INFO ("DSIStart:ChEnable Err = 0x%x\n\r", mgrStatus);
            return;
        }

#if U3V_INMEM_EN
        Cy_U3V_MemCommitFrameBuffers(pAppCtxt, glChHandleDSI);

        Cy_USB_USBD_EndpSetClearNakNrdy(pUsbdCtxt, CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN, false);
#endif /* U3V_INMEM_EN */

#if INMD_EN
        /* Make sure the DMA callback does not drop data any more and allow the tester to re-start
         * the video stream.
         */
        glFlags.blockFrame = false;
        Cy_LVDS_GpifClearFwTrig(LVDSSS_LVDS, 0);
#endif /* INMD_EN */
#if LVDS_LB_EN
        if (lvdsConsCount == 0)
        {

            Cy_Update_LvdsLinkClock(pAppCtxt->devSpeed == CY_USBD_USB_DEV_HS);
            DBG_APP_INFO("Start loopback\r\n");
            pAppCtxt->u3vFlowCtrlFlag = false;
            Cy_HBDma_Channel_Reset(&lvdsLbPgmChannel);
            Cy_HBDma_Channel_Enable(&lvdsLbPgmChannel, 0);
            lvdsLpbkBlocked = false;
            Cy_LVDS_CommitColorbarData();
            Cy_LVDS_CommitColorbarData();
            LVDSSS_LVDS->GPIF[1].GPIF_WAVEFORM_CTRL_STAT |= LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_CPU_LAMBDA_Msk;
        }
#endif /* LVDS_LB_EN */

#if FPGA_ENABLE
        DBG_APP_INFO("Started Streaming @ %d x %d, %d fps\r\n",
            glFrameParams.width,
            glFrameParams.height,
            glFrameParams.fps);

        Cy_U3V_AppFPGAParamsUpdate(glFrameParams.width,glFrameParams.height,glFrameParams.fps);
        status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEV0_STREAM_ENABLE_ADDRESS, CAMERA_APP_ENABLE,FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
#endif
    } else{
        DBG_APP_INFO("DSI App already started..\r\n");
    }

    return;
}

/*****************************************************************************
* Function Name: Cy_U3V_AppDCICmdHaltEventHandler
******************************************************************************
* Summary:
* This function handles DCI Cmd endpoint Halt Evt
*
* Parameters:
* \param pAppCtxt
* application layer context pointer
*
* \param pUsbdCtxt
* USBD layer context pointer
*
* \param setClear
* Set endpoint HALT if true else clear endpoint HALT condition
*
* Return:
* void
****************************************************************************/
void
Cy_U3V_AppDCICmdHaltEventHandler (
        cy_stc_usb_app_ctxt_t *pAppCtxt,
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
        bool setClear)
{
    bool isStall = false;

    if (setClear)   /* SET FEATURE */
    {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_U3V_EP_DCI_CMD, CY_USB_ENDP_DIR_OUT, true);
    }
    else   /* CLEAR FEATURE */
    {

        Cy_U3V_AppFlushAndResetDma(pAppCtxt, glChHandleDCICmd, CY_U3V_EP_DCI_CMD, CY_USB_ENDP_DIR_OUT, false);
        Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DCI_CMD, CY_USB_ENDP_DIR_OUT, false);

    }

    /* Check EP Status : Stall/Clear */
    isStall = Cy_USBD_EndpIsStallSet(pUsbdCtxt, CY_U3V_EP_DCI_CMD, CY_USB_ENDP_DIR_OUT);
    DBG_APP_TRACE("Cy_U3V_AppDCICmdHaltEventHandler: isStall : %d \b\r",isStall);
    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
    return;
}

/*****************************************************************************
* Function Name: Cy_U3V_AppDCIRspHaltEventHandler
******************************************************************************
* Summary:
* This function handles DCI response endpoint event
*
* Parameters:
* \param pAppCtxt
* application layer context pointer
*
* \param pUsbdCtxt
* USBD layer context pointer
*
* \param setClear
* Set endpoint HALT if true elase clear endpoint HALT condition
*
* Return:
* void
****************************************************************************/
void
Cy_U3V_AppDCIRspHaltEventHandler (
        cy_stc_usb_app_ctxt_t *pAppCtxt,
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
        bool setClear)
{
    bool isStall = false;


    if (setClear) /* SET FEATURE */
    {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_U3V_EP_DCI_RSP, CY_USB_ENDP_DIR_IN, true);
    }
    else /* CLEAR FEATURE */
    {
        xTimerStop(pAppCtxt->dciTimerHandle,0);
        Cy_U3V_AppFlushAndResetDma(pAppCtxt, glChHandleDCIRsp, CY_U3V_EP_DCI_RSP, CY_USB_ENDP_DIR_IN, false);
        Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DCI_RSP, CY_USB_ENDP_DIR_IN, false);

    }

    isStall = Cy_USBD_EndpIsStallSet(pUsbdCtxt,CY_U3V_EP_DCI_RSP,CY_USB_ENDP_DIR_IN);
    DBG_APP_TRACE("Cy_U3V_AppDCICmdHaltEventHandler: isStall : %d \b\r",isStall);
    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);

    return;
}

#if LVDS_LB_EN
/*****************************************************************************
* Function Name: Cy_U3V_AppCommitColorbarData
******************************************************************************
* Summary:
*  Function that commits loopback data patterns that inject the colorbar
* data pattern into the LVDS ingress socket..
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
static void
Cy_U3V_AppCommitColorbarData (
        cy_stc_usb_app_ctxt_t  *pUsbApp,
        cy_stc_hbdma_channel_t *pChHandle)
{
    cy_stc_hbdma_buff_status_t lbBuffStat;

    if (lvdsLpbkBlocked) {
        DBG_APP_INFO("Skip data commit due to streaming stop\r\n");
        return;
    }

    /* Two buffers of the producer socket are required to fill one buffer of the streaming socket. */
    if (Cy_HBDma_Channel_GetBuffer(pChHandle, &lbBuffStat) != CY_HBDMA_MGR_SUCCESS) {
        DBG_APP_ERR("GetLpbkBuf 2 failed\r\n");
        return;
    }

    lbBuffStat.count = LOOPBACK_MEM_BUF_SIZE;
    Cy_HBDma_Channel_CommitBuffer(pChHandle, &lbBuffStat);

    if (Cy_HBDma_Channel_GetBuffer(pChHandle, &lbBuffStat) != CY_HBDMA_MGR_SUCCESS) {
        DBG_APP_ERR("GetLpbkBuf 3 failed\r\n");
        return;
    }

    lbBuffStat.count = LOOPBACK_MEM_BUF_SIZE;
    Cy_HBDma_Channel_CommitBuffer(pChHandle, &lbBuffStat);
}

#endif /* LVDS_LB_EN */

/*****************************************************************************
* Function Name: Cy_U3V_AppCtrlDmaCb
******************************************************************************
* Summary:
* This function is HBDMA callback funxtion for DCI channels
*
* Parameters:
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
* Return:
* void
****************************************************************************/
void Cy_U3V_AppCtrlDmaCb (
        cy_stc_hbdma_channel_t *handle,
        cy_en_hbdma_cb_type_t type,
        cy_stc_hbdma_buff_status_t* pbufStat,
        void *userCtx)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    pAppCtxt = (cy_stc_usb_app_ctxt_t *) userCtx;
    cy_stc_usbd_app_msg_t xMsg;

    if(type == CY_HBDMA_CB_PROD_EVENT)
    {
        xMsg.type = CY_USB_U3V_CMD_EVT_FLAG;
    }
    else if(type == CY_HBDMA_CB_XFER_CPLT)
    {
        xMsg.type = CY_USB_U3V_RESP_SENT_EVT_FLAG;
    }
    else
    {
        DBG_APP_ERR("AppDmaCallback Error type:%x:\r\n",type);
        return;
    }

    Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
    Cy_USB_App_KeepLinkActive(pAppCtxt);
    return;
}  /* end of function  */


/*****************************************************************************
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
Cy_U3V_AppHandleProduceEvent (
        cy_stc_usb_app_ctxt_t  *pAppCtxt,
        cy_stc_hbdma_channel_t *pChHandle)
{
    cy_en_hbdma_mgr_status_t   status;
    cy_stc_hbdma_buff_status_t buffStat;
    cy_stc_usbd_app_msg_t      xMsg;

#if (!U3V_INMEM_EN)
    uint8_t isFrameComplete = 0;
#endif /* (!U3V_INMEM_EN) */

    /* Wait for a free buffer. */
    status = Cy_HBDma_Channel_GetBuffer(pChHandle, &buffStat);
    if (status != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_APP_ERR("HB-DMA GetBuffer Error\r\n");
        return;
    }

#if LVDS_LB_EN
    if (glU3VBufCounter == CY_USB_U3V_LEADER_BUFFER_NO) {
        glFlags.frameOngoing = true;
        glFlags.lastFrameConsumed = false;
        glFlags.lastFrameProduced = false;
        Cy_U3V_GetUpdatedLeaderData(&u3vDevCtxt,0, buffStat.pBuffer,(uint16_t *)&(buffStat.count));
        glU3VBufCounter++;

        xMsg.type = CY_USB_U3V_LEADER_RXD_FLAG;
        Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
    } else {
        if ((glU3VBufCounter > CY_USB_U3V_LEADER_BUFFER_NO) && (glU3VBufCounter < glU3VFrameBuffer - 1)) {
            pAppCtxt->glProd++;
            buffStat.count = STREAM_DMA_BUFFER_SIZE;
            pAppCtxt->frameSizeTransferred += (buffStat.count);
            glU3VBufCounter++;
        } else {
            if (glU3VBufCounter == glU3VFrameBuffer - 1) {
                pAppCtxt->glProd++;
                if(pAppCtxt->isPartialBuf == 1u)
                {
                    buffStat.count = pAppCtxt->partialBufSize;
                }
                else
                {
                    buffStat.count = STREAM_DMA_BUFFER_SIZE;
                }
                pAppCtxt->frameSizeTransferred += buffStat.count;
                pAppCtxt->frameSize =  pAppCtxt->frameSizeTransferred;
                glU3VBufCounter++;
                pAppCtxt->consCount = pAppCtxt->glCons;
                pAppCtxt->prodCount = pAppCtxt->glProd;
                /*Frame has ended. Clear the producer and consumer counts*/
                pAppCtxt->glProd = 0;
                pAppCtxt->glCons = 0;
            }
            else {
                if (glU3VBufCounter == glU3VFrameBuffer) {

                    Cy_U3V_IsFullFrameCompleted(&u3vDevCtxt, pAppCtxt->frameSizeTransferred, &isFrameComplete);
                    if(!isFrameComplete){
                        xMsg.type = CY_USB_U3V_FRAME_MISSING_FLAG;
                        xMsg.data[0] = pAppCtxt->frameSizeTransferred;
                        Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
                    }
                    pAppCtxt->frameSize =  pAppCtxt->frameSizeTransferred;
                    pAppCtxt->frameSizeTransferred = 0;
                    pAppCtxt->fpsCount++;
                    Cy_U3V_GetUpdatedTrailerData(&u3vDevCtxt, buffStat.pBuffer,(uint16_t *)&(buffStat.count));
                    glU3VBufCounter = CY_USB_U3V_LEADER_BUFFER_NO;
                    glFlags.lastFrameProduced = true;
                    glFlags.frameOngoing = false;
                    xMsg.type = CY_USB_U3V_TRAILER_RXD_FLAG;
                    Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
                }
                else {
                    DBG_APP_ERR("DSI Wrong count:%x\r\n", glU3VBufCounter);
                    Cy_HBDma_Channel_DiscardBuffer(pChHandle, &buffStat);
                    return;
                }
            }
        }
    }
#else
    if (glFlags.blockFrame){
        /* App stop already received from HOST APP. Ignore leader of next frame from LVDS. */
        if (pAppCtxt->devSpeed < CY_USBD_USB_DEV_SS_GEN1){
            if (pAppCtxt->u3vPendingBufCnt){
                pAppCtxt->u3vPendingBufCnt--;
            }
        }
        xMsg.type = CY_USB_U3V_DMA_BLOCKED_FLAG;
        Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
        Cy_HBDma_Channel_DiscardBuffer(pChHandle, &buffStat);
        DBG_APP_TRACE("BlockFrame: %d, n_HBDMABytes: %d.\r\n", glFlags.blockFrame, buffStat.count);
        return;
    }
#if ((FPGA_ENABLE && FPGA_ADDS_HEADER) || (INMD_EN))
    else if(buffStat.count == (U3V_LEADER_SIZE))
    {
        glFlags.frameOngoing = true;
        glFlags.lastFrameConsumed = false;
        glFlags.lastFrameProduced = false;

        xMsg.type = CY_USB_U3V_LEADER_RXD_FLAG;
        Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
    }
    else if(buffStat.count == (U3V_TRAILER_SIZE))
    {
        pAppCtxt->fpsCount++;
        glFlags.lastFrameProduced = true;
        glFlags.frameOngoing = false;

        xMsg.type = CY_USB_U3V_TRAILER_RXD_FLAG;
        Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);

        Cy_U3V_IsFullFrameCompleted(&u3vDevCtxt, pAppCtxt->frameSizeTransferred, &isFrameComplete);
        if(!isFrameComplete){
            xMsg.type = CY_USB_U3V_FRAME_MISSING_FLAG;
            xMsg.data[0] = pAppCtxt->frameSizeTransferred;
            Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
        }
        pAppCtxt->frameSize =  pAppCtxt->frameSizeTransferred;
        pAppCtxt->frameSizeTransferred = 0;
    }
#elif (FPGA_ENABLE && (!FPGA_ADDS_HEADER))
    else if(buffStat.count == 0x00)
    {
        if(glIsLeader)
        {
            glFlags.frameOngoing = true;
            glFlags.lastFrameConsumed = false;
            glFlags.lastFrameProduced = false;

            Cy_U3V_GetUpdatedLeaderData(&u3vDevCtxt,0, buffStat.pBuffer, (uint16_t *)&(buffStat.count));

            glIsLeader = false;
            xMsg.type = CY_USB_U3V_LEADER_RXD_FLAG;
            Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
        }
        else if(glIsTrailer)
        {
            pAppCtxt->fpsCount++;

            Cy_U3V_GetUpdatedTrailerData(&u3vDevCtxt, buffStat.pBuffer,(uint16_t *)&(buffStat.count));

            glIsLeader  = true;
            glIsTrailer = false;
            glFlags.lastFrameProduced = true;
            glFlags.frameOngoing = false;
            xMsg.type = CY_USB_U3V_TRAILER_RXD_FLAG;
            Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);

            Cy_U3V_IsFullFrameCompleted(&u3vDevCtxt, pAppCtxt->frameSizeTransferred, &isFrameComplete);
            if(!isFrameComplete){
                xMsg.type = CY_USB_U3V_FRAME_MISSING_FLAG;
                xMsg.data[0] = pAppCtxt->frameSizeTransferred;
                Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
            }
            pAppCtxt->frameSize =  pAppCtxt->frameSizeTransferred;
            pAppCtxt->frameSizeTransferred = 0;

        }else{
            xMsg.type = CY_USB_U3V_ZLP_DISCARDED;
            Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
            Cy_HBDma_Channel_DiscardBuffer(pChHandle, &buffStat);
            return;
        }
    }
#endif /* FPGA_ADDS_HEADER */
    else
    {
        pAppCtxt->glProd++;
        pAppCtxt->frameSizeTransferred += (buffStat.count);

        /* If leader has not been received before payload */
        if(!glFlags.frameOngoing)
        {
            xMsg.type = CY_USB_U3V_NO_LEADER_RECVD;
            Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
        }

        if (buffStat.count != STREAM_DMA_BUFFER_SIZE)
        {
            pAppCtxt->consCount = pAppCtxt->glCons;
            pAppCtxt->prodCount = pAppCtxt->glProd;
            /*Frame has ended. Clear the producer and consumer counts*/
            pAppCtxt->glProd = 0;
            pAppCtxt->glCons = 0;

#if !FPGA_ADDS_HEADER
            glIsTrailer = true; /*Received Partial buffer - next is trailer*/
#endif
        }
    }
#endif /* LVDS_LB_EN */

    /* Send the data to the USB endpoint. */
    if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1)
    {
        status = Cy_HBDma_Channel_CommitBuffer(pChHandle, &buffStat);
        if (status != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("HB-DMA DSI CommitBuffer Error: %x\r\n", status);
            return;
        }
    }
    else
    {
        /* For USB 2.0, we need to write the data to the USB endpoint*/
        Cy_USB_AppQueueWrite(pAppCtxt, CY_U3V_EP_DSI_STREAM, buffStat.pBuffer, buffStat.count);
    }
}

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
void
Cy_U3V_AppCommandRecvCompletion (
        cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usbd_app_msg_t xMsg;

    xMsg.type = CY_USB_U3V_CMD_EVT_FLAG;
    Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);

    /* Mask out ISR for the streaming endpoint until command has been handled. */
    NVIC_DisableIRQ(cpuss_interrupts_dw1_0_IRQn + CY_U3V_EP_DSI_STREAM);
}

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
void
Cy_U3V_AppResponseSendCompletion (
        cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usbd_app_msg_t xMsg;

    xMsg.type = CY_USB_U3V_RESP_SENT_EVT_FLAG;
    Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
}

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

void
Cy_U3V_AppHandleSendCompletion (
        cy_stc_usb_app_ctxt_t *pAppCtxt)
{
#if !U3V_INMEM_EN
    cy_stc_hbdma_buff_status_t buffStat;
    cy_en_hbdma_mgr_status_t   dmaStat;

    /* At least one buffer must be pending. */
    if (pAppCtxt->u3vPendingBufCnt == 0)
    {
        DBG_APP_ERR("PendingBufCnt=0 on SendComplete\r\n");
        return;
    }

    /* The buffer which has been sent to the USB host can be discarded. */
    dmaStat = Cy_HBDma_Channel_DiscardBuffer(glChHandleDSI, &buffStat);
    if (dmaStat != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_APP_ERR("DiscardBuffer failed with status=%x\r\n", dmaStat);
        return;
    }

#if LVDS_LB_EN
    /* If colorbar data pattern was blocked, enable it to fill one more data buffer. */
    if (pAppCtxt->u3vFlowCtrlFlag)
    {
        pAppCtxt->u3vFlowCtrlFlag = false;
        Cy_U3V_AppCommitColorbarData(pAppCtxt, &lvdsLbPgmChannel);
    }
#endif /* LVDS_LB_EN */

    /* If another DMA buffer has already been filled by the producer, go
     * on and send it to the host controller.
     */
    pAppCtxt->u3vPendingBufCnt--;
    if (pAppCtxt->u3vPendingBufCnt > 0)
    {
        Cy_U3V_AppHandleProduceEvent(pAppCtxt, glChHandleDSI);
    }
#else
    /* For in-memory streaming, we need to commit the next buffer to the USB endpoint. */
    Cy_U3V_MemCommitBuffer(pAppCtxt, glChHandleDSI, glU3VBufCounter);
    if(glU3VBufCounter == glU3VFrameBuffer)
    {
        glU3VBufCounter = CY_USB_U3V_LEADER_BUFFER_NO;
    }
    else
    {
        glU3VBufCounter++;
    }
#endif /* !U3V_INMEM_EN */
}

/*****************************************************************************
* Function Name: Cy_U3V_AppStreamDmaCallback
******************************************************************************
* Summary:
* HBDMA callback function to add the UVC header to the frame/buffer coming
* from LVDS and commits to USB.
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
void
Cy_U3V_AppStreamDmaCallback (
        cy_stc_hbdma_channel_t *handle,
        cy_en_hbdma_cb_type_t type,
        cy_stc_hbdma_buff_status_t *pbufStat,
        void *userCtx)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;

    if (type == CY_HBDMA_CB_PROD_EVENT) {
        if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            Cy_U3V_AppHandleProduceEvent(pAppCtxt, handle);
        } else {
            pAppCtxt->u3vPendingBufCnt++;
            if ((pAppCtxt->u3vPendingBufCnt == 1) || (glFlags.blockFrame)) {
                Cy_U3V_AppHandleProduceEvent(pAppCtxt, handle);
            }
        }
    } else {
        if (type == CY_HBDMA_CB_CONS_EVENT) {
            if (glFlags.lastFrameProduced)
            {
                glFlags.lastFrameProduced = false;
                glFlags.lastFrameConsumed = true;
            }

            pAppCtxt->glCons++;

#if LVDS_LB_EN
            /* If the loopback TX socket is in flow control state, commit two more buffers. */
            if (pAppCtxt->u3vFlowCtrlFlag)
            {
                pAppCtxt->u3vFlowCtrlFlag = false;
                Cy_U3V_AppCommitColorbarData(pAppCtxt, &lvdsLbPgmChannel);
            }
#endif /* LVDS_LB_EN */
        }
    }

    Cy_USB_App_KeepLinkActive(pAppCtxt);

}  /* end of function  */

#if LVDS_LB_EN
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
void Cy_HbDma_LoopbackCb (cy_stc_hbdma_channel_t *handle,
                        cy_en_hbdma_cb_type_t type,
                        cy_stc_hbdma_buff_status_t *pbufStat,
                        void *userCtx)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;

    if ((type == CY_HBDMA_CB_CONS_EVENT) && (!lvdsLpbkBlocked))
    {
        lvdsConsCount++;

        /*
         * If the LVDS producer socket is active, commit the next two buffers.
         * Otherwise, set a flag indicating flow control state.
         */
        if ((lvdsConsCount & 0x01) == 0) {
            if((_FLD2VAL(LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_STATE, \
                            (LVDSSS_LVDS->ADAPTER_DMA[0].SCK[0].SCK_STATUS))) == 0x2) {
                Cy_U3V_AppCommitColorbarData(pAppCtxt, &lvdsLbPgmChannel);
            } else {
                pAppCtxt->u3vFlowCtrlFlag = true;
            }
        }
    }
}
#endif /* LVDS_LB_EN */

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
void
Cy_USB_AppDisableEndpDma (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    uint8_t i;

    /* On USB 2.x connections, make sure the DataWire channels are disabled and reset. */
    if (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
        for (i = 1; i < CY_USB_MAX_ENDP_NUMBER; i++) {
            if (pAppCtxt->endpInDma[i].valid) {
                /* DeInit the DMA channel and disconnect the triggers. */
                Cy_USBHS_App_DisableEpDmaSet(&(pAppCtxt->endpInDma[i]));
            }

            if (pAppCtxt->endpOutDma[i].valid) {
                /* DeInit the DMA channel and disconnect the triggers. */
                Cy_USBHS_App_DisableEpDmaSet(&(pAppCtxt->endpOutDma[i]));
            }
        }
    }

    /* Disable and destroy the High BandWidth DMA channels. */
    if (glChHandleDSI != NULL) {
#if U3V_INMEM_EN
        Cy_U3V_MemClearBufPointers(pAppCtxt, glChHandleDSI);
#endif /* U3V_INMEM_EN */
        Cy_HBDma_Channel_Disable(glChHandleDSI);
        Cy_HBDma_Channel_Destroy(glChHandleDSI);
        glChHandleDSI = NULL;

        /* Flush and reset the EPM in case of USB 3.x connection. */
        if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN);
            Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN, false);
        }
    }

    if (glChHandleDCIRsp != NULL) {
        Cy_HBDma_Channel_Disable(glChHandleDCIRsp);
        Cy_HBDma_Channel_Destroy(glChHandleDCIRsp);
        glChHandleDCIRsp = NULL;

        /* Flush and reset the EPM in case of USB 3.x connection. */
        if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DCI_RSP, CY_USB_ENDP_DIR_IN);
            Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DCI_RSP, CY_USB_ENDP_DIR_IN, false);
        }
    }

    if (glChHandleDCICmd != NULL) {
        Cy_HBDma_Channel_Disable(glChHandleDCICmd);
        Cy_HBDma_Channel_Destroy(glChHandleDCICmd);
        glChHandleDCICmd = NULL;

        /* Flush and reset the EPM in case of USB 3.x connection. */
        if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DCI_CMD, CY_USB_ENDP_DIR_OUT);
            Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DCI_CMD, CY_USB_ENDP_DIR_OUT, false);
        }
    }

#if LVDS_LB_EN
    pAppCtxt->u3vFlowCtrlFlag = false;
    lvdsConsCount = 0;
    Cy_HBDma_Channel_Reset(&lvdsLbPgmChannel);
#endif /* LVDS_LB_EN */
}

/***********************************************************************************************
* Function Name: Cy_U3V_AppDeviceTaskHandler
**************************************************************************************************
* Summary:
* Application task handler
*
* Parameters:
* \param pTaskParam
* task param
*
* Return:
* void
************************************************************************************************/
void
Cy_U3V_AppDeviceTaskHandler (void *pTaskParam)
{
    cy_stc_usbd_app_msg_t queueMsg;
    cy_en_hbdma_mgr_status_t  mgrStatus;
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;
    cy_stc_hbdma_buff_status_t buffStat;
    BaseType_t xStatus;
    cy_en_usbss_lnk_power_mode_t curLinkState;
    uint32_t lpEntryTime = 0;
    bool isStall = false;
    uint8_t isRespSent = 0;
    uint8_t isDciIdle = 0;

    vTaskDelay(pdMS_TO_TICKS(500));
    DBG_APP_INFO("U3vDeviceThreadCreated\r\n");

#if FPGA_ENABLE

#if (LINK_TRAINING)
    Cy_LVDS_PhyGpioModeEnable(LVDSSS_LVDS, LINK_READY_CTL_PORT,LINK_READY_CTL_PIN,
        CY_LVDS_PHY_GPIO_OUTPUT, CY_LVDS_PHY_GPIO_NO_INTERRUPT);
    Cy_LVDS_PhyGpioClr(LVDSSS_LVDS, LINK_READY_CTL_PORT, LINK_READY_CTL_PIN);
#endif /*LINK_TRAINING*/
#if FPGA_CONFIG_EN
    Cy_FPGAConfigPins(pAppCtxt,FPGA_CONFIG_MODE);
    Cy_QSPI_Start(pAppCtxt,&HBW_BufMgr);
    Cy_SPI_FlashInit(SPI_FLASH_0, true, false);

    if(Cy_FPGAConfigure(pAppCtxt,FPGA_CONFIG_MODE) == true)
    {
        DBG_APP_INFO("FPGA configuration complete \r\n");
        glIsFPGAConfigComplete = true;
    }
    else
    {
        LOG_ERROR("Failed to configure FPGA \r\n");
    }
#else
    if(true == Cy_IsFPGAConfigured())
    {
        glIsFPGAConfigComplete = true;
    }
#endif /* FPGA_CONFIG_EN */

    if((glIsFPGARegConfigured == false) && (glIsFPGAConfigComplete == true))
    {
        Cy_FPGAPhyLinkTraining();
        Cy_FPGAGetVersion(pAppCtxt);

        if (Cy_ConfigFpgaRegister() != 0){
            DBG_APP_ERR("Failed to configure FPGA via I2C \r\n");
        }else{
            glIsFPGARegConfigured = true;
            DBG_APP_TRACE("Successfuly configured  FPGA via I2C \n\r");
        }
    }

#endif /* FPGA_ENABLE*/

#if !U3V_INMEM_EN
    vTaskDelay(pdMS_TO_TICKS(100));
    Cy_LVDS_LVCMOS_Init();
    vTaskDelay(pdMS_TO_TICKS(100));
#endif /* !U3V_INMEM_EN */

    /* If VBus is present, enable the USB connection. */
    pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);
    if (pAppCtxt->vbusPresent) {
        Cy_USB_SSConnectionEnable(pAppCtxt);
    }
    else {
        DBG_APP_ERR("vbus is not present, USBSS not connected\r\n");
    }

#if USB3_LPM_ENABLE
        if ((pAppCtxt->isLpmEnabled == false) && (pAppCtxt->lpmEnableTime != 0)) {
            Cy_USBSS_Cal_GetLinkPowerState(pAppCtxt->pUsbdCtxt->pSsCalCtxt, &curLinkState);
            if (
                    (Cy_USBD_GetTimerTick() >= pAppCtxt->lpmEnableTime) &&
                    (curLinkState != CY_USBSS_LPM_U3)
               ) {
                DBG_APP_INFO("LPM re-enable\r\n");
                pAppCtxt->isLpmEnabled  = true;
                pAppCtxt->lpmEnableTime = 0;
                Cy_USBD_LpmEnable(pAppCtxt->pUsbdCtxt);
            }
        }
#endif /* USB3_LPM_ENABLE */
    vTaskDelay(pdMS_TO_TICKS(250));
    Cy_U3V_AppUpdateStreamingParams(pAppCtxt, U3V_VRES, U3V_HRES, U3V_FPS, BITS_PER_PIXEL);

#if FPGA_ENABLE
    vTaskDelay(pdMS_TO_TICKS(100));
    Cy_U3V_AppFPGAParamsUpdate(glFrameParams.width,glFrameParams.height,glFrameParams.fps);
    vTaskDelay(pdMS_TO_TICKS(100));
#endif /* FPGA_ENABLE*/

    Cy_U3V_InitializeInterface();

    do {
#if WATCHDOG_RESET_EN
        /* Kick The WDT to prevent RESET */
        KickWDT();
#endif /* WATCHDOG_RESET_EN */
        /*
         * Wait until some data is received from the queue.
         * Timeout after 20 ms.
         */

        xStatus = xQueueReceive(pAppCtxt->u3vMessageQueue, &queueMsg,20);
        if (xStatus != pdPASS) {
            continue;
        }

        switch (queueMsg.type) {
            case CY_USB_U3V_DMA_BLOCKED_FLAG:
                DBG_APP_INFO("CASE:App has already stopped. Blocked the next packet\r\n");
                break;

            case CY_USB_U3V_ZLP_DISCARDED:
                DBG_APP_ERR("CASE:Received a ZLP in-between payload.Discarded\r\n");
                break;

            case CY_USB_U3V_NO_LEADER_RECVD:
                DBG_APP_ERR("CASE:No leader received before payload\r\n");
                break;

            case CY_USB_U3V_TRAILER_RXD_FLAG:
                DBG_APP_TRACE("CASE:Trailer received\r\n");
                break;

            case CY_USB_U3V_LEADER_RXD_FLAG:
                DBG_APP_TRACE("CASE:Leader received\r\n");
                break;

            case CY_USB_U3V_FRAME_MISSING_FLAG:
                DBG_APP_ERR("CASE:Incomplete frame sent to USB\r\n");
                break;

            /* DCI Response EP Halt */
            case CY_USB_U3V_EPHALT_DCI_RSP_EVT_FLAG:
                Cy_U3V_AppDCIRspHaltEventHandler(pAppCtxt, pAppCtxt->pUsbdCtxt, true);
                Cy_U3V_DciEphHandler(&u3vDevCtxt,EPH_SET, EP_IN);
                break;

            /* DCI Response EP Clear */
            case CY_USB_U3V_EPCLR_DCI_RSP_EVT_FLAG:
                Cy_U3V_AppDCIRspHaltEventHandler(pAppCtxt, pAppCtxt->pUsbdCtxt, false);
                Cy_U3V_DciEphHandler(&u3vDevCtxt,EPH_CLEAR, EP_IN);
                break;

            /* Device Command Interface (DCI) Command EP Halt */
            case CY_USB_U3V_EPHALT_DCI_CMD_EVT_FLAG:
                Cy_U3V_AppDCICmdHaltEventHandler(pAppCtxt, pAppCtxt->pUsbdCtxt, true);
                Cy_U3V_DciEphHandler(&u3vDevCtxt,EPH_SET, EP_OUT);
                break;

            /* Device Command Interface (DCI) Command EP Clear */
            case CY_USB_U3V_EPCLR_DCI_CMD_EVT_FLAG:
                Cy_U3V_AppDCICmdHaltEventHandler(pAppCtxt, pAppCtxt->pUsbdCtxt, false);
                Cy_U3V_DciEphHandler(&u3vDevCtxt,EPH_CLEAR, EP_OUT);
                break;

            /* DSI EP Halt */
            case CY_USB_U3V_EPHALT_DSI_EVT_FLAG:
                isStall = Cy_U3V_AppDSIHaltEventHandler(pAppCtxt, pAppCtxt->pUsbdCtxt, true);
                Cy_U3V_DsiEphHandler(&u3vDevCtxt,isStall);
                break;

            /* DSI EP Clear */
            case CY_USB_U3V_EPCLR_DSI_EVT_FLAG:
                Cy_U3V_AppDSIStop(pAppCtxt, pAppCtxt->pUsbdCtxt);
                isStall = Cy_U3V_AppDSIHaltEventHandler(pAppCtxt, pAppCtxt->pUsbdCtxt, false);
                Cy_U3V_DsiEphHandler(&u3vDevCtxt,isStall);

                break;
            case CY_USB_U3V_SETCONF_EVT_FLAG:

                Cy_U3V_DciEphHandler(&u3vDevCtxt,EPH_CLEAR, EP_OUT);
                Cy_U3V_DciEphHandler(&u3vDevCtxt,EPH_CLEAR, EP_IN);
                Cy_U3V_DsiEphHandler(&u3vDevCtxt,EPH_CLEAR);
                xTimerStop(pAppCtxt->dciTimerHandle,0);
                break;

            /* Command send success.Cleanup buffer and restart fetching U3V Commands */
            case CY_USB_U3V_RESP_SENT_EVT_FLAG:
                Cy_U3V_IsSentRspPendingAck(&u3vDevCtxt,&isRespSent);
                /* Command send success.Cleanup buffer and restart fetching U3V Commands */
                if(isRespSent == 1u)
                {
                    DBG_APP_INFO("Pending ACK Xfer Done\r\n");
                }
                else
                {
                    Cy_U3V_UpdateDciStateAfterRspSent(&u3vDevCtxt);
                    Cy_U3V_IsDCIStateIdle(&u3vDevCtxt, &isDciIdle);
                    if(isDciIdle == 1u){
                        xTimerStop(pAppCtxt->dciTimerHandle,0);
                        memset(glU3vResponseBuffer, 0, CMD_BUFF_SIZE);
                    }
                }
                break;

            /* U3V Command Received by device */
            case CY_USB_U3V_CMD_EVT_FLAG:
                /* Move to Process Command state - U3V Spec: Device Control Interface State Diagram */

                if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1)
                {
                    mgrStatus = Cy_HBDma_Channel_GetBuffer(glChHandleDCICmd,&buffStat);
                    if (mgrStatus != CY_HBDMA_MGR_SUCCESS)
                    {
                        DBG_APP_ERR("DCIcmd GetBuffer Error:%x\r\n",mgrStatus);
                        break;
                    }

                    memcpy(glU3vCmdBuffer, (uint8_t *)buffStat.pBuffer, buffStat.count);
                    Cy_HBDma_Channel_DiscardBuffer(glChHandleDCICmd,&buffStat);
                }

                xTimerReset(pAppCtxt->dciTimerHandle,0);
                Cy_U3V_DciCmdHandler(&u3vDevCtxt, glU3vCmdBuffer, glU3vResponseBuffer);

                /* Queue read to fetch the next command. */
                if (pAppCtxt->devSpeed < CY_USBD_USB_DEV_SS_GEN1) {
                    Cy_USB_AppQueueRead(pAppCtxt, CY_U3V_EP_DCI_CMD, glU3vCmdBuffer, CMD_BUFF_SIZE);
                }
                break;

            case CY_USB_U3V_VBUS_CHANGE_INTR:
                /* Start the debounce timer. */
                xTimerStart(pAppCtxt->vbusDebounceTimer, 0);
                break;

            case CY_USB_U3V_VBUS_CHANGE_DEBOUNCED:
                /* Check whether VBus state has changed. */
                pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);
                if (pAppCtxt->vbusPresent) {
                    if (!pAppCtxt->usbConnectDone) {
                        DBG_APP_INFO("Enabling USB connection due to VBus detect\r\n");
                        Cy_USB_SSConnectionEnable(pAppCtxt);
                    }
                } else {
                    if (pAppCtxt->usbConnectDone) {
                        Cy_USB_AppDisableEndpDma(pAppCtxt);
                        DBG_APP_INFO("Disabling USB connection due to VBus removal\r\n");
                        Cy_USB_SSConnectionDisable(pAppCtxt);
                    }
                }
                break;

            case CY_USB_U3V_FPS_POLL_EVENT:
                DBG_APP_INFO("U3V FPS : %d\n\r",(uint32_t)queueMsg.data[0]);
                DBG_APP_INFO("U3V Bytes Transferred : %d\n\r",pAppCtxt->frameSize);
                pAppCtxt->frameSize = 0;
                xTimerReset(pAppCtxt->userTimer,0);
                break;
            default:
                DBG_APP_INFO("CASE:U3VDefault %d\r\n", queueMsg.type);
                break;

        } /* end of switch() */

        curLinkState = CY_USBSS_LPM_UNKNOWN;
        if ((glU3VDevConfigured) && (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1)) {
            Cy_USBSS_Cal_GetLinkPowerState(pAppCtxt->pUsbdCtxt->pSsCalCtxt, &curLinkState);
        }

        /*
         * If the link has been in USB2-L1 or in USB3-U2 for more than 0.5 second, initiate LPM exit so that
         * transfers do not get delayed significantly.
         */
        if (
                ((curLinkState == CY_USBSS_LPM_U2) && (!(pAppCtxt->pUsbdCtxt->pSsCalCtxt->forceLPMAccept))) ||
                (
                 (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) &&
                 ((MXS40USBHSDEV_USBHSDEV->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L1_SLEEP) != 0)
                )
           ) {
            if ((Cy_USBD_GetTimerTick() - lpEntryTime) > 500UL) {
                lpEntryTime = Cy_USBD_GetTimerTick();
                Cy_USBD_GetUSBLinkActive(pAppCtxt->pUsbdCtxt);
            }
        } else {
            lpEntryTime = Cy_USBD_GetTimerTick();
        }


        /* Enable to get next DCI CMD DMA channel if DCI state is IDLE */
        Cy_U3V_IsDCIStateIdle(&u3vDevCtxt, &isDciIdle);
        if (isDciIdle == 1u){
            mgrStatus = CY_HBDMA_MGR_SUCCESS;
            if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1){
                mgrStatus = Cy_HBDma_Channel_Enable(glChHandleDCICmd, 0);
            }else {
                Cy_USB_AppQueueRead(pAppCtxt, CY_U3V_EP_DCI_CMD, glU3vCmdBuffer, CMD_BUFF_SIZE);
            }
            if ((mgrStatus == CY_HBDMA_MGR_SUCCESS) || (mgrStatus == CY_HBDMA_MGR_SEQUENCE_ERROR)){
                Cy_U3V_UpdateDCIStateIdle_N(&u3vDevCtxt);
            }else{
                DBG_APP_ERR("DCIcmd Channel_Enable Error:%x\r\n",mgrStatus);
                Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DCI_CMD, CY_USB_ENDP_DIR_OUT, true);
                Cy_SysLib_DelayUs(100);
                Cy_HBDma_Channel_Reset(glChHandleDCICmd);
                DBG_APP_INFO("DMA Channel reset: cmd\r\n");
                Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, CY_U3V_EP_DCI_CMD, CY_USB_ENDP_DIR_OUT, false);
            }
        }

        if (pAppCtxt->dmaInterruptDisabled) {
            pAppCtxt->dmaInterruptDisabled = false;
            NVIC_EnableIRQ(lvds2usb32ss_lvds_dma_adap1_int_o_IRQn);
            NVIC_EnableIRQ(lvds2usb32ss_lvds_dma_adap0_int_o_IRQn);
        }
    } while (1);
} /* End of function  */

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
void
Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt,
                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, DMAC_Type *pCpuDmacBase,
                    DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt)
{
    uint32_t index;
    BaseType_t status = pdFALSE;
    cy_stc_app_endp_dma_set_t *pEndpInDma;
    cy_stc_app_endp_dma_set_t *pEndpOutDma;

    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pAppCtxt->devAddr = 0x00;
    pAppCtxt->activeCfgNum = 0x00;
    pAppCtxt->prevAltSetting = 0x00;
    pAppCtxt->enumMethod = CY_USB_ENUM_METHOD_FAST;
    pAppCtxt->pHbDmaMgrCtxt = pHbDmaMgrCtxt;
    pAppCtxt->dmaInterruptDisabled = false;

    for (index = 0x00; index < CY_USB_MAX_ENDP_NUMBER; index++)
    {
        pEndpInDma = &(pAppCtxt->endpInDma[index]);
        memset((void *)pEndpInDma, 0, sizeof(cy_stc_app_endp_dma_set_t));

        pEndpOutDma = &(pAppCtxt->endpOutDma[index]);
        memset((void *)pEndpOutDma, 0, sizeof(cy_stc_app_endp_dma_set_t));
    }

    pAppCtxt->pCpuDmacBase = pCpuDmacBase;
    pAppCtxt->pCpuDw0Base = pCpuDw0Base;
    pAppCtxt->pCpuDw1Base = pCpuDw1Base;
    pAppCtxt->pUsbdCtxt = pUsbdCtxt;
    pAppCtxt->u3vPendingBufCnt = 0;

    /*
     * Callbacks registered with USBD layer. These callbacks will be called
     * based on appropriate event.
     */
    Cy_USB_AppRegisterCallback(pAppCtxt);

    if (!(pAppCtxt->firstInitDone)) {
        pAppCtxt->vbusChangeIntr = false;
        pAppCtxt->vbusPresent = false;
        pAppCtxt->usbConnectDone = false;

        /* create queue and register it to kernel. */
        pAppCtxt->u3vMessageQueue = xQueueCreate(CY_USB_U3V_DEVICE_MSG_QUEUE_SIZE,
                                        CY_USB_U3V_DEVICE_MSG_SIZE);

        if (pAppCtxt->u3vMessageQueue == NULL){
            DBG_APP_ERR("QueuecreateFail\r\n");
            return;
        }
        DBG_APP_INFO("createdU3VQueue\r\n");
        vQueueAddToRegistry(pAppCtxt->u3vMessageQueue, "U3VDeviceMsgQueue");

        /* Create task and check status to confirm task created properly. */
        status = xTaskCreate(Cy_U3V_AppDeviceTaskHandler, "U3vDeviceTask", 2048,
                        (void *)pAppCtxt, 11, &(pAppCtxt->u3vDevicetaskHandle));
        if (status != pdPASS) {
            DBG_APP_ERR("Task create Fail\r\n");
            return;
        }

        pAppCtxt->dcitimerExpiry = U3V_ABRM_MAX_RESPONSE_TIME;
        pAppCtxt->dciTimerHandle = xTimerCreate("DCICMDTimer", pAppCtxt->dcitimerExpiry, pdFALSE,
                       ( void * ) pAppCtxt, Cy_U3V_AppPendingAckTimerCb);
        pAppCtxt->vbusDebounceTimer = xTimerCreate("VbusDebounceTimer", 200, pdFALSE,
                (void *)pAppCtxt, Cy_USB_VbusDebounceTimerCallback);

        pAppCtxt->userTimer = xTimerCreate("UserTimer", 1000, pdTRUE,
                                                 (void *)pAppCtxt, Cy_U3V_AppTimerCb);

        if ((pAppCtxt->dciTimerHandle == NULL) || (pAppCtxt->vbusDebounceTimer == NULL) || (pAppCtxt->userTimer == NULL))
        {
            DBG_APP_ERR("Timer Create Fail\r\n");
            return;
        }

        xTimerStart(pAppCtxt->userTimer, 0);


        pAppCtxt->evtLogTimer = xTimerCreate("EventLogTimer", 10000, pdTRUE,
                                             (void *)pAppCtxt,
                                             Cy_USB_PrintEvtLogTimerCb);
        if (pAppCtxt->evtLogTimer == NULL) {
            DBG_APP_ERR("evtLogTimer Create Fail\r\n");
            return;
        } else {
            /* Start the debounce timer. */
            DBG_APP_TRACE("evtLogTimer Start\r\n");
            xTimerStart(pAppCtxt->evtLogTimer, 0);
        }
        DBG_APP_INFO("xTimer Created\r\n");

        /* Allocate DMA Memory to CMD and Response Buffer for DCI */
        /* Allocate DMA Memory to CMD and Response Buffer for DCI */
        glU3vCmdBuffer = (uint8_t *) Cy_HBDma_BufMgr_Alloc(pHbDmaMgrCtxt->pBufMgr, CMD_BUFF_SIZE);
        glU3vResponseBuffer = (uint8_t *) Cy_HBDma_BufMgr_Alloc(pHbDmaMgrCtxt->pBufMgr, RSP_BUFF_SIZE);
        DBG_APP_INFO("Alloc DMA Mem %x %x\r\n", glU3vCmdBuffer, glU3vResponseBuffer);

        pAppCtxt->firstInitDone = 0x01;
    }

    /* Zero out the EP0 test buffer. */
    memset ((uint8_t *)Ep0TestBuffer, 0, sizeof(Ep0TestBuffer));

    return;
} /* end of function. */

/*****************************************************************************
* Function Name: Cy_USB_AppSetAddressCallback
******************************************************************************
* Summary:
*  This Function will be called by USBD layer when  a USB address has been assigned to the device.
*
* \param pUsbApp
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
void
Cy_USB_AppSetAddressCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    /* Update the state variables. */
    pAppCtxt->devState     = CY_USB_DEVICE_STATE_ADDRESS;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DEFAULT;
    pAppCtxt->devAddr      = pUsbdCtxt->devAddr;
    pAppCtxt->devSpeed     = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);

    /* Check the type of USB connection and register appropriate descriptors. */
    CyApp_RegisterUsbDescriptors(pAppCtxt, pAppCtxt->devSpeed);
}

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
void
Cy_USB_AppRegisterCallback (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET, Cy_USB_AppBusResetCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET_DONE, Cy_USB_AppBusResetDoneCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_BUS_SPEED, Cy_USB_AppBusSpeedCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETUP, Cy_USB_AppSetupCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SUSPEND, Cy_USB_AppSuspendCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESUME, Cy_USB_AppResumeCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_CONFIG, Cy_USB_AppSetCfgCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_INTF, Cy_USB_AppSetIntfCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_SLEEP, Cy_USB_AppL1SleepCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_RESUME, Cy_USB_AppL1ResumeCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_ZLP, Cy_USB_AppZlpCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SLP, Cy_USB_AppSlpCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETADDR, Cy_USB_AppSetAddressCallback);
    return;
}   /* end of function. */

/****************************************************************************
* Function Name: Cy_USB_AppSetupEndpDmaParamsHs
******************************************************************************
* Summary:
*  Configure and enable HBW DMA channels.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pEndpDscr
* Endpoint descriptor pointer
*
* \return
* None
*
 *******************************************************************************/
static void
Cy_USB_AppSetupEndpDmaParamsHs (cy_stc_usb_app_ctxt_t *pAppCtxt,
                              uint8_t *pEndpDscr)
{
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    DW_Type *pDW;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize = 0x00;
    cy_en_usb_endp_dir_t endpDirection;
    bool stat;

    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);

    if (dir != 0) {
        endpDirection = CY_USB_ENDP_DIR_IN;
        pEndpDmaSet = &(pAppCtxt->endpInDma[endpNumber]);
        pDW = pAppCtxt->pCpuDw1Base;
    } else {
        endpDirection = CY_USB_ENDP_DIR_OUT;
        pEndpDmaSet = &(pAppCtxt->endpOutDma[endpNumber]);
        pDW = pAppCtxt->pCpuDw0Base;
    }

    stat = Cy_USBHS_App_EnableEpDmaSet(pEndpDmaSet, pDW, endpNumber, endpNumber, endpDirection, maxPktSize);
    DBG_APP_INFO("Enable EPDmaSet: endp=%x dir=%x stat=%x\r\n", endpNumber, endpDirection, stat);
}   /* end of function  */

/*****************************************************************************
 * Function Name: Cy_USB_AppSetupEndpDmaParamsSs
 ******************************************************************************
 * Summary:
 *  Perform high bandwidth DMA initialization associated with a USB endpoint.
 *
 * Parameters:
 *  \param pAppCtxt
 *  Pointer to application context structure.
 *
 *  \param pEndpDscr
 * Endpoint descriptor
 *
 * Return:
 *  void
 *****************************************************************************/
static void
Cy_USB_AppSetupEndpDmaParamsSs (cy_stc_usb_app_ctxt_t *pAppCtxt,
                                uint8_t *pEndpDscr)
{
    cy_stc_hbdma_chn_config_t dmaConfig;
    cy_en_hbdma_mgr_status_t  mgrStatus = CY_HBDMA_MGR_SUCCESS;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;

    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);

    DBG_APP_INFO("AppSetupEndpDmaParamsSs: endpNum:0x%x maxPktSize:0x%x dir:0x%x\r\n",
                 endpNumber, maxPktSize, dir);

    dmaConfig.count          = 1;                          /* DMA Buffer Count */
    dmaConfig.bufferMode     = false;                      /* DMA buffer mode disabled */
    dmaConfig.prodHdrSize    = 0;                          /* No header to be added within the buffer. */
    dmaConfig.prodSckCount   = 1;                          /* No. of producer sockets */
    dmaConfig.consSckCount   = 1;                          /* No. of consumer Sockets */
    dmaConfig.prodSck[1]     = (cy_hbdma_socket_id_t)0;    /* Producer Socket ID: None */
    dmaConfig.consSck[1]     = (cy_hbdma_socket_id_t)0;    /* Consumer Socket ID: None */
    dmaConfig.eventEnable    = 0;                          /* No events to be sent. */
    dmaConfig.userCtx        = (void *)(pAppCtxt);          /* Pass the application context as user context. */

    if (dir) {
        /* Its IN endpoint which means DATA device->Host */
        pEndpDmaSet = &(pAppCtxt->endpInDma[endpNumber]);

        /* EP_DCI_RSP */
        /* Create channel which will move data from SRAM to USB endpoint. */
        if (endpNumber == CY_U3V_EP_DCI_RSP)
        {
            glChHandleDCIRsp = &(pEndpDmaSet->hbDmaChannel);

            dmaConfig.size           = CMD_BUFF_SIZE;  /* DMA Buffer size in bytes */
            dmaConfig.prodBufSize    = CMD_BUFF_SIZE;
            dmaConfig.chType         = CY_HBDMA_TYPE_MEM_TO_IP;  /* DMA Channel type: from HB-RAM to USB3-IP */
            dmaConfig.prodSck[0]     = CY_HBDMA_VIRT_SOCKET_WR;
            dmaConfig.consSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBEG_SOCKET_00 + endpNumber);
            dmaConfig.cb             = Cy_U3V_AppCtrlDmaCb; /* HB-DMA callback */
            dmaConfig.intrEnable     = 0x82;  /* Enable for Consume event */
            mgrStatus = Cy_HBDma_Channel_Create(pAppCtxt->pUsbdCtxt->pHBDmaMgr,
                                          glChHandleDCIRsp, &dmaConfig);
        }
        else if (endpNumber == CY_U3V_EP_DSI_STREAM)
        {
            /* If channel had already been created, make sure to disable and destroy it. */
            if (glChHandleDSI != NULL) {
                DBG_APP_INFO("Destroying previously created streaming channel\r\n");
#if U3V_INMEM_EN
                Cy_U3V_MemClearBufPointers(pAppCtxt, glChHandleDSI);
#endif /*U3V_INMEM_EN*/
                Cy_HBDma_Channel_Disable(glChHandleDSI);
                Cy_HBDma_Channel_Destroy(glChHandleDSI);
                glChHandleDSI = NULL;
            }

            glChHandleDSI = &(pEndpDmaSet->hbDmaChannel);
            pAppCtxt->u3vPendingBufCnt = 0;

#if U3V_INMEM_EN
            dmaConfig.size           = 256;                      /* DMA Buffer size in bytes */
            dmaConfig.count          = U3V_INMEM_BUFFER_COUNT;   /* DMA Buffer Count */
            dmaConfig.prodBufSize    = 256;
            dmaConfig.chType         = CY_HBDMA_TYPE_MEM_TO_IP;  /* DMA Channel type: from memory to USB3-IP */
            dmaConfig.prodSck[0]     = CY_HBDMA_VIRT_SOCKET_WR;
            dmaConfig.consSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBEG_SOCKET_00 + endpNumber);
            dmaConfig.cb             = Cy_U3V_MemStreamCb;       /* HB-DMA callback */
            dmaConfig.intrEnable     = LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk |
                                       LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;
            dmaConfig.eventEnable    = 0x0;
#else
            /* Create channel which will move data from LVDS to USB endpoint. */
            dmaConfig.size           = STREAM_DMA_BUFFER_SIZE;  /* DMA Buffer size in bytes */
            dmaConfig.count          = STREAM_DMA_BUFFER_COUNT; /* DMA Buffer Count */
            dmaConfig.prodBufSize    = STREAM_DMA_BUFFER_SIZE;
            dmaConfig.chType         = CY_HBDMA_TYPE_IP_TO_IP;  /* DMA Channel type: from LVDS to USB3-IP */
            dmaConfig.consSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBEG_SOCKET_00 + endpNumber);
            dmaConfig.prodSck[0]     = CY_HBDMA_LVDS_SOCKET_00;
#if INTERLEAVE_EN
            dmaConfig.prodSckCount   = 2;
            dmaConfig.prodSck[1]     = CY_HBDMA_LVDS_SOCKET_01;
#endif /* INTERLEAVE_EN */

#if ((INMD_EN) || ((FPGA_ADDS_HEADER) && (AUTO_DMA_EN)))
            dmaConfig.eventEnable    = 0x01;
            dmaConfig.intrEnable     = 0x00;
            dmaConfig.cb             = NULL;  /* HB-DMA callback */
#else /* FW adds U3V leader/trailer. */
            dmaConfig.eventEnable    = 0x00;
            dmaConfig.intrEnable     = LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk |
                                       LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;
            dmaConfig.cb             = Cy_U3V_AppStreamDmaCallback;  /* HB-DMA callback */
#endif /* ((INMD_EN) || ((FPGA_ADDS_HEADER) && (AUTO_DMA_EN))) */

            if (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
                dmaConfig.chType      = CY_HBDMA_TYPE_IP_TO_MEM;
                dmaConfig.consSck[0]  = (cy_hbdma_socket_id_t)0;
                dmaConfig.count       = 1;     /* Use a single buffer in USB-HS use case. */
                dmaConfig.eventEnable = 0;
                dmaConfig.intrEnable  = LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk |
                                        LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;
                dmaConfig.cb          = Cy_U3V_AppStreamDmaCallback;  /* HB-DMA callback */
            }
#endif /* U3V_INMEM_EN */

            DBG_APP_INFO(" DMA Channel Config - Manual Channel enable: %d U3V Header Addition by FPGA: %d\n\r", (!AUTO_DMA_EN),(FPGA_ADDS_HEADER));
            DBG_APP_INFO(" DMA Channel Config - DMA buffer Size: %d, Producer buffer count: %d \n\r",dmaConfig.size,dmaConfig.count);
            /* Create an IP to MEM channel to receive data from the SIP in the case of USB 2.x connection. */
            mgrStatus = Cy_HBDma_Channel_Create(pAppCtxt->pUsbdCtxt->pHBDmaMgr,
                                        glChHandleDSI, &dmaConfig);
            if (mgrStatus == CY_HBDMA_MGR_SUCCESS) {
#if LVDS_LB_EN
                /* Use default DMA adapter settings when Link Loopback is being used. */
                Cy_HBDma_Mgr_SetUsbEgressAdapterDelay(pAppCtxt->pUsbdCtxt->pHBDmaMgr, 0);
#endif /* LVDS_LB_EN */

#if U3V_INMEM_EN
                Cy_U3V_MemAllocateBuffers(pAppCtxt, glChHandleDSI);
#endif /* U3V_INMEM_EN*/
            }
        }

        if(mgrStatus == CY_HBDMA_MGR_SUCCESS)
        {
            pEndpDmaSet->valid = 1;
        }
        else
        {
            DBG_APP_ERR("BulkIn endpNumber:%x channel create failed 0x%x\r\n", endpNumber, mgrStatus);
            return;
        }
    } else {
        /*  EP_DCI_CMD  */
        /* Its OUT endpoint which means data Host->Device. */
        pEndpDmaSet = &(pAppCtxt->endpOutDma[endpNumber]);
        glChHandleDCICmd = &(pEndpDmaSet->hbDmaChannel);

        /* Create channel which moves data from USB ingress endpoint into HBW SRAM. */
        dmaConfig.size           = CMD_BUFF_SIZE;  /* DMA Buffer size in bytes */
        dmaConfig.bufferMode     = true;   /* DMA buffer mode  */
        dmaConfig.prodBufSize    = CMD_BUFF_SIZE;
        dmaConfig.chType         = CY_HBDMA_TYPE_IP_TO_MEM;
        dmaConfig.prodSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBIN_SOCKET_00 + endpNumber);
        dmaConfig.consSck[0]     = CY_HBDMA_VIRT_SOCKET_RD;
        dmaConfig.cb             = Cy_U3V_AppCtrlDmaCb;    /* HB-DMA callback */
        dmaConfig.intrEnable     = 0x1;  /* Enable for Producer event */
        mgrStatus = Cy_HBDma_Channel_Create(pAppCtxt->pUsbdCtxt->pHBDmaMgr,
                                          glChHandleDCICmd,
                                          &dmaConfig);

        if (mgrStatus != CY_HBDMA_MGR_SUCCESS) {
            DBG_APP_ERR("BulkOut endpNumber:%x channel create failed 0x%x\r\n",endpNumber,mgrStatus);
            return;
        } else {
            DBG_APP_INFO("HBDMA BulkOut endpNumber:%x ChnCreate status: %x\r\n",endpNumber,mgrStatus);
            pEndpDmaSet->valid = 1;
        }
    }

    return;
} /* end of function  */

/****************************************************************************
* Function Name: Cy_USB_AppSetupEndpDmaParams
******************************************************************************
* Summary:
*  Configure and enable DMA channels.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pEndpDscr
* Endpoint descriptor pointer
*
* \return
* None
*
 *******************************************************************************/

void
Cy_USB_AppSetupEndpDmaParams (cy_stc_usb_app_ctxt_t *pAppCtxt,
                              uint8_t *pEndpDscr)
{
    /* Create the HBW DMA channels in all cases. */
    Cy_USB_AppSetupEndpDmaParamsSs (pAppCtxt, pEndpDscr);

    /* In the case of USB 2.0 connection, additionally configure the DataWire channels. */
    if (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
        Cy_USB_AppSetupEndpDmaParamsHs (pAppCtxt, pEndpDscr);
    }
}

/****************************************************************************
* Function Name: Cy_USB_AppConfigureEndp
******************************************************************************
* Summary:
*  Configure all endpoints used by application (except EP0)
*
* \param pUsbdCtxt
* USBD layer context pointer
*
* \param pEndpDscr
* Endpoint descriptor pointer
*
* \return
* None
*
 *******************************************************************************/

void
Cy_USB_AppConfigureEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pEndpDscr)
{
    cy_stc_usb_endp_config_t endpConfig;
    cy_en_usb_endp_dir_t endpDirection;
    bool valid;
    uint32_t endpType;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;
    uint32_t isoPkts = 0x00;
    uint8_t burstSize = 0x00;
    uint8_t maxStream = 0x00;
    uint8_t *pCompDscr = NULL;
    cy_en_usbd_ret_code_t usbdRetCode;


    /* If it is not endpoint descriptor then return */
    if (!Cy_USBD_EndpDscrValid(pEndpDscr)) {
        return;
    }
    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);

    if (dir) {
        endpDirection = CY_USB_ENDP_DIR_IN;
    } else {
        endpDirection = CY_USB_ENDP_DIR_OUT;
    }
    Cy_USBD_GetEndpType(pEndpDscr, &endpType);

    if ((CY_USB_ENDP_TYPE_ISO == endpType) || (CY_USB_ENDP_TYPE_INTR == endpType)) {
        /* The ISOINPKS setting in the USBHS register is the actual packets per microframe value. */
        isoPkts = (
                (*((uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) & CY_USB_ENDP_ADDL_XN_MASK)
                >> CY_USB_ENDP_ADDL_XN_POS) + 1;
    }

    valid = 0x01;
    if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        /* Get companion descriptor and from there get burstSize. */
        pCompDscr = Cy_USBD_GetSsEndpCompDscr(pUsbdCtxt, pEndpDscr);
        Cy_USBD_GetEndpCompnMaxburst(pCompDscr, &burstSize);
        Cy_USBD_GetEndpCompnMaxStream(pCompDscr, &maxStream);
    }

    /* Prepare endpointConfig parameter. */
    endpConfig.endpType = (cy_en_usb_endp_type_t)endpType;
    endpConfig.endpDirection = endpDirection;
    endpConfig.valid = valid;
    endpConfig.endpNumber = endpNumber;
    endpConfig.maxPktSize = (uint32_t)maxPktSize;
    endpConfig.isoPkts = isoPkts;
    endpConfig.burstSize = burstSize;
    endpConfig.streamID = maxStream;
    endpConfig.allowNakTillDmaRdy = false;
    endpConfig.interval = 0;
    usbdRetCode = Cy_USB_USBD_EndpConfig(pUsbdCtxt, endpConfig);

    /* Print status of the endpoint configuration to help debug. */
    DBG_APP_INFO("#ENDPCFG: %d, %d\r\n", endpNumber, usbdRetCode);

    return;
}   /* end of function */

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
void
Cy_USB_AppSetCfgCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_stc_usb_cal_msg_t *pMsg)
{

    cy_stc_usb_app_ctxt_t *pAppCtxt;
    uint8_t *pActiveCfg, *pIntfDscr, *pEndpDscr;
    uint8_t index, numOfIntf, numOfEndp;
    cy_en_usb_speed_t devSpeed;
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_INFO("AppSetCfgCbStart Speed=%d\r\n", pUsbdCtxt->devSpeed);

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUserCtxt;
    pAppCtxt->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    devSpeed = pAppCtxt->devSpeed;
    /* Save the configuration number. */
    pAppCtxt->activeCfgNum = pUsbdCtxt->activeCfgNum;

    /* Destroy any active DMA channels. */
    Cy_USB_AppDisableEndpDma(pAppCtxt);
    /*
     * Based on type of application as well as how data flows,
     * data wire can be used so initialize datawire.
     */
    Cy_DMA_Enable(pAppCtxt->pCpuDw0Base);
    Cy_DMA_Enable(pAppCtxt->pCpuDw1Base);

    pActiveCfg = Cy_USB_USBD_GetActiveCfgDscr(pUsbdCtxt);
    if (!pActiveCfg) {
        /* Set config should be called when active config value > 0x00. */
        DBG_APP_ERR("pActiveCfg:%d  return\r\n",pActiveCfg);
        return;
    }
    numOfIntf = Cy_USBD_FindNumOfIntf(pActiveCfg);
    if (numOfIntf == 0x00) {
        DBG_APP_ERR("numOfIntf:%d  return\r\n",numOfIntf);
        return;
    }

    for (index = 0x00; index < numOfIntf; index++) {
        /* During Set Config command always altSetting 0 will be active. */
        pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, index, 0x00);
        if (pIntfDscr == NULL) {
            DBG_APP_ERR("pIntfDscrNull\r\n");
            return;
        }

        numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
        if (numOfEndp == 0x00) {
            DBG_APP_INFO("numOfEndp 0\r\n");
            continue;
        }

        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00) {
            Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParams(pAppCtxt, pEndpDscr);
            numOfEndp--;
            if(devSpeed > CY_USBD_USB_DEV_HS) {
                pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)) + 6);
            } else {
                pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
            }
        }
    }

    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_CONFIGURED;
    pAppCtxt->devState = CY_USB_DEVICE_STATE_CONFIGURED;

    glU3VDevConfigured = true;

    /* In USB 2.0 connections, register ISRs for and enable the DataWire channel interrupts. */
    if (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
        Cy_USB_AppInitDmaIntr(CY_U3V_EP_DCI_CMD, CY_USB_ENDP_DIR_OUT, Cy_U3V_CommandChannel_ISR);
        Cy_USB_AppInitDmaIntr(CY_U3V_EP_DCI_RSP, CY_USB_ENDP_DIR_IN, Cy_U3V_ResponseChannel_ISR);
        Cy_USB_AppInitDmaIntr(CY_U3V_EP_DSI_STREAM, CY_USB_ENDP_DIR_IN, Cy_U3V_StreamChannel_ISR);
    }

    xMsg.type = CY_USB_U3V_SETCONF_EVT_FLAG;
    Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);

    /* Zero out the command and response buffers. */
    memset(glU3vCmdBuffer,0,CMD_BUFF_SIZE);
    memset(glU3vResponseBuffer,0,RSP_BUFF_SIZE);

#if USB3_LPM_ENABLE
    /* Schedule LPM enable after 1 second. */
    DBG_APP_INFO("Schedule LPM enable\r\n");
    pAppCtxt->isLpmEnabled = false;
    pAppCtxt->lpmEnableTime = Cy_USBD_GetTimerTick() + 1000;
#endif /* USB3_LPM_ENABLE */

    DBG_APP_INFO("AppSetCfgCbEnd Done\r\n");
    return;
}   /* end of function */

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
void Cy_USB_AppBusResetCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    uint8_t i;

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUserCtxt;

    DBG_APP_INFO("AppBusResetCallback\r\n");

    for (i = 0; i < CY_USB_MAX_ENDP_NUMBER; i++) {
        if (pAppCtxt->endpInDma[i].valid) {
            DBG_APP_INFO("HBDMA destroy EP%d-In\r\n", i);
            Cy_HBDma_Channel_Disable(&(pAppCtxt->endpInDma[i].hbDmaChannel));
            Cy_HBDma_Channel_Destroy(&(pAppCtxt->endpInDma[i].hbDmaChannel));
            pAppCtxt->endpInDma[i].valid = false;
        }

        if (pAppCtxt->endpOutDma[i].valid) {
            DBG_APP_INFO("HBDMA destroy EP%d-Out\r\n", i);
            Cy_HBDma_Channel_Disable(&(pAppCtxt->endpOutDma[i].hbDmaChannel));
            Cy_HBDma_Channel_Destroy(&(pAppCtxt->endpOutDma[i].hbDmaChannel));
            pAppCtxt->endpOutDma[i].valid = false;
        }
    }

    /*
     * USBD layer takes care of reseting its own data structure as well as
     * takes care of calling CAL reset APIs. Application needs to take care
     * of reseting its own data structure as well as "device function".
     */
    Cy_USB_AppInit(pAppCtxt, pUsbdCtxt, pAppCtxt->pCpuDmacBase,
                   pAppCtxt->pCpuDw0Base, pAppCtxt->pCpuDw1Base,
                   pAppCtxt->pHbDmaMgrCtxt);
    pAppCtxt->devState = CY_USB_DEVICE_STATE_RESET;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_RESET;
    pAppCtxt->activeCfgNum = 0;

    if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
#if USB3_LPM_ENABLE
        DBG_APP_INFO("Disabling LPM\r\n");
        Cy_USBSS_Cal_LPMDisable(pUsbdCtxt->pSsCalCtxt);
#endif /* USB3_LPM_ENABLE */
    }

    glU3VDevConfigured = false;
    glU3VIsApplnActive = false;
    return;
} /* end of function. */

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
void Cy_USB_AppBusResetDoneCallback (void *pUserCtxt,
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    DBG_APP_INFO("ppBusResetDoneCallback\r\n");

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUserCtxt;
    pAppCtxt->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pAppCtxt->prevDevState = pAppCtxt->devState;
    return;
}   /* end of function. */

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

void Cy_USB_AppBusSpeedCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUserCtxt;
    pAppCtxt->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pAppCtxt->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    return;
}   /* end of function. */

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
void
Cy_USB_AppSetupCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUserCtxt;
    uint8_t  bRequest, bReqType;
    uint8_t  bType;
    uint8_t bTarget;
    uint16_t wValue,wIndex,wLength;
    bool isReqHandled = false;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_INFO("AppSetupCallback\r\n");

    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function. */

    /* Decode the fields from the setup request. */
    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bTarget  = (bReqType & CY_USB_CTRL_REQ_RECIPENT_OTHERS);
    bRequest = pUsbdCtxt->setupReq.bRequest;
    bType    = ((bReqType & CY_USB_CTRL_REQ_TYPE_MASK) >> CY_USB_CTRL_REQ_TYPE_POS);
    wValue   = pUsbdCtxt->setupReq.wValue;
    wIndex   = pUsbdCtxt->setupReq.wIndex;
    wLength  = pUsbdCtxt->setupReq.wLength;

    DBG_APP_TRACE("AppSetupCallback: bType = 0x%x, bTarget= 0x%x\r\n", bType, bTarget);

    if (bType == CY_USB_CTRL_REQ_STD)
    {
        if((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (bRequest == CY_USB_SC_CLEAR_FEATURE))
        {
            /* Clear Feature(Endpoint_Halt) received on Endpoint */
            DBG_APP_INFO("EP:0x%x Clear Feature 0x%x \r\n", wIndex, wValue);
            if (wValue == CY_USB_FEATURE_ENDP_HALT)
            {
                if (wIndex == CY_U3V_EP_DSI_STREAM_ADDR)
                {
                    xMsg.type = CY_USB_U3V_EPCLR_DSI_EVT_FLAG;
                }
                else if (wIndex == CY_U3V_EP_DCI_CMD_ADDR)
                {
                    xMsg.type = CY_USB_U3V_EPCLR_DCI_CMD_EVT_FLAG;
                }
                else if (wIndex == CY_U3V_EP_DCI_RSP_ADDR)
                {
                    xMsg.type = CY_USB_U3V_EPCLR_DCI_RSP_EVT_FLAG;
                }

                Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
                isReqHandled = true;
            }
        }
        else if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (bRequest == CY_USB_SC_SET_FEATURE))
        {
            /* SET Feature(Endpoint_Halt) received on Endpoint */
            DBG_APP_INFO("EP:0x%x Set Feature 0x%x \r\n", wIndex, wValue);
            if (wValue == CY_USB_FEATURE_ENDP_HALT)
            {
                if (wIndex == CY_U3V_EP_DCI_CMD_ADDR){
                    xMsg.type = CY_USB_U3V_EPHALT_DCI_CMD_EVT_FLAG;
                }
                else if (wIndex == CY_U3V_EP_DCI_RSP_ADDR){
                    xMsg.type = CY_USB_U3V_EPHALT_DCI_RSP_EVT_FLAG;
                }
                else if (wIndex == CY_U3V_EP_DSI_STREAM_ADDR){
                    xMsg.type = CY_USB_U3V_EPHALT_DSI_EVT_FLAG;
                }

                Cy_USB_AppSendMsgToTask(pAppCtxt, &xMsg);
                isReqHandled = true;
            }
        }
        else if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && ((bRequest == CY_USB_SC_SET_FEATURE)
                    || (bRequest == CY_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (glU3VDevConfigured)
            {
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            }
            else
            {
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,CY_USB_ENDP_DIR_IN, TRUE);
            }
            isReqHandled = true;
        }
        else if ((bRequest == CY_USB_SC_GET_STATUS) && (bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF))
        {
            /* We support only interface 0 */
            if (wIndex == 0)
            {
                Ep0TestBuffer[0] = 0;
                Ep0TestBuffer[1] = 0;
                Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)Ep0TestBuffer,0x02);
            }
            else
            {
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,CY_USB_ENDP_DIR_IN, TRUE);
            }
            isReqHandled = true;
        }
        else if ((bRequest == CY_USB_SC_SET_SEL) && (wLength == 6)){
            /* SET_SEL request is only received in USBSS case and the Cy_USB_USBD_RecvEndp0Data is blocking. */
            retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, (uint8_t *)SetSelDataBuffer, wLength);
            DBG_APP_INFO("SET_SEL: EP0 recv stat = %d, Data=%x:%x\r\n",
                    retStatus, SetSelDataBuffer[0], SetSelDataBuffer[1]);
            isReqHandled = true;
        }
        /* Supported to facilitate Chapter 9 compliance tests. */
        else if ((bRequest == CY_USB_SC_SET_FEATURE) &&  (bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                ((wValue == CY_USB_FEATURE_U1_ENABLE) || (wValue == CY_USB_FEATURE_U2_ENABLE))) {

#if USB3_LPM_ENABLE
            if (pAppCtxt->isLpmEnabled == false) {
                DBG_APP_INFO("Enabling LPM\r\n");
                pAppCtxt->isLpmEnabled = true;
                Cy_USBD_LpmEnable(pUsbdCtxt);
            }
#endif /* USB3_LPM_ENABLE */

            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            isReqHandled = true;
        }
        else if ((bRequest == CY_USB_SC_CLEAR_FEATURE) &&  (bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                ((wValue == CY_USB_FEATURE_U1_ENABLE) || (wValue == CY_USB_FEATURE_U2_ENABLE))) {

#if USB3_LPM_ENABLE
            DBG_APP_INFO("Disabling LPM\r\n");
            pAppCtxt->isLpmEnabled  = false;
            pAppCtxt->lpmEnableTime = 0;
            Cy_USBD_LpmDisable(pUsbdCtxt);
#endif /* USB3_LPM_ENABLE */

            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            isReqHandled = true;
        }
    }

    /*
     * If Request is not handled by the callback, Stall the command.
     */
    if(!isReqHandled) {
        DBG_APP_ERR("SetupRequest is not handled\r\n");
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,
                                      CY_USB_ENDP_DIR_IN, TRUE);
    }

}   /* end of function. */

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

void
Cy_USB_AppSuspendCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUserCtxt;
    pAppCtxt->prevDevState = pAppCtxt->devState;
    pAppCtxt->devState = CY_USB_DEVICE_STATE_SUSPEND;
}   /* end of function. */

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
void Cy_USB_AppResumeCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    cy_en_usb_device_state_t tempState;

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUserCtxt;

    tempState =  pAppCtxt->devState;
    pAppCtxt->devState = pAppCtxt->prevDevState;
    pAppCtxt->prevDevState = tempState;
    return;
}   /* end of function. */

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
void Cy_USB_AppSetIntfCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_setup_req_t *pSetupReq;
    uint8_t intfNum, altSetting;
    int8_t numOfEndp;
    uint8_t *pIntfDscr, *pEndpDscr;
    uint32_t endpNumber;
    cy_en_usb_endp_dir_t endpDirection;
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUserCtxt;

    DBG_APP_INFO("AppSetIntfCallback Start\r\n");
    pSetupReq = &(pUsbdCtxt->setupReq);
    /*
     * Get interface and alt setting info. If new setting same as previous
     * then return.
     * If new alt setting came then first Unconfigure previous settings
     * and then configure new settings.
     */
    intfNum = pSetupReq->wIndex;
    altSetting = pSetupReq->wValue;

    if (altSetting == pAppCtxt->prevAltSetting) {
        /* Nothing needs to be done. */
        DBG_APP_INFO("SameAltSetting\r\n");
        return;
    }

    /* New altSetting is different than previous one so unconfigure previous. */
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, pAppCtxt->prevAltSetting);
    DBG_APP_INFO("unconfigPrevAltSet\r\n");
    if (pIntfDscr == NULL) {
        DBG_APP_INFO("pIntfDscrNull\r\n");
        return;
    }
    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    if (numOfEndp == 0x00) {
        DBG_APP_INFO("SetIntf:prevNumEp 0\r\n");
    } else {
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00) {
            if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80) {
                endpDirection = CY_USB_ENDP_DIR_IN;
            } else {
                endpDirection = CY_USB_ENDP_DIR_OUT;
            }
            endpNumber =
                (uint32_t)((*(pEndpDscr+CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);

            /* with FALSE, unconfgure previous settings. */
            Cy_USBD_EnableEndp(pUsbdCtxt, endpNumber, endpDirection, FALSE);

            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
        }
    }

    /* Now take care of different config with new alt setting. */
    pAppCtxt->prevAltSetting = altSetting;
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, altSetting);
    if (pIntfDscr == NULL) {
        DBG_APP_INFO("pIntfDscrNull\r\n");
        return;
    }

    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    if (numOfEndp == 0x00) {
        DBG_APP_INFO("SetIntf:numEp 0\r\n");
    } else {
        pAppCtxt->prevAltSetting = altSetting;
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00) {
            Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParams(pAppCtxt, pEndpDscr);
            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
        }
    }

    DBG_APP_INFO("AppSetIntfCallback done\r\n");
    return;
}   /* end of function. */



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
void Cy_USB_AppZlpCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppZlpCb\r\n");

    return;
}   /* end of function. */

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
void Cy_USB_AppL1SleepCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppL1SleepCb\r\n");
    return;
}   /* end of function. */


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

void Cy_USB_AppL1ResumeCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppL1ResumeCb\r\n");
    return;
} /* end of function. */

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
void Cy_USB_AppSlpCallback (void *pUserCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUserCtxt;
    uint16_t rcvdLen = 0;
    uint8_t  endpNumber = (uint8_t)pMsg->data[0];

    if (endpNumber == CY_U3V_EP_DCI_CMD)
    {
        /* Prepare to read the short packet of data out from EPM into the DMA buffer. */
        rcvdLen = Cy_USB_AppReadShortPacket(pAppCtxt, CY_U3V_EP_DCI_CMD, (uint16_t)pMsg->data[1]);

        /* Send a trigger to the DMA channel after it has been configured. */
        Cy_TrigMux_SwTrigger(TRIG_IN_MUX_0_USBHSDEV_TR_OUT0 + CY_U3V_EP_DCI_CMD,
                CY_TRIGGER_TWO_CYCLES);

        (void)rcvdLen;
    }
}   /* end of function. */

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
void
Cy_USB_AppQueueRead (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize)
{
    cy_stc_app_endp_dma_set_t      *dmaset_p;

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
            (pAppCtxt->pCpuDw0Base == NULL) || (pBuffer == NULL) || (dataSize == 0))
    {
        DBG_APP_ERR("QueueRead: BadParam\r\n");
        return;
    }

    /* Verify that the selected endpoint is valid and the dataSize is non-zero. */
    dmaset_p  = &(pAppCtxt->endpOutDma[endpNumber]);
    if (dmaset_p->valid == 0)
    {
        DBG_APP_ERR("QueueRead:BadParam\r\n");
        return;
    }

    Cy_USBHS_App_QueueRead(dmaset_p, pBuffer, dataSize);

} /* end of function */


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
uint16_t
Cy_USB_AppReadShortPacket (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint16_t pktSize)
{
    cy_stc_app_endp_dma_set_t *dmaset_p;
    uint16_t dataSize = 0;

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) || (pAppCtxt->pCpuDw0Base == NULL))
    {
        DBG_APP_ERR("ReadSLP:NULL\r\n");
        return 0;
    }

    /* Verify that the selected endpoint is valid. */
    dmaset_p = &(pAppCtxt->endpOutDma[endpNumber]);
    if (dmaset_p->valid == 0)
    {
        DBG_APP_ERR("ReadSLP:BadParam\r\n");
        return 0;
    }

    /* The code assumes that the channel is active. */
    dataSize = Cy_USBHS_App_ReadShortPacket(dmaset_p, pktSize);
    return dataSize;
} /* end of function */

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
                          uint8_t *pBuffer, uint16_t dataSize)
{
    cy_stc_app_endp_dma_set_t *dmaset_p;

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
            (pAppCtxt->pCpuDw1Base == NULL) || (pBuffer == NULL)) {
        DBG_APP_ERR("QueueWrite Err0\r\n");
        return;
    }

    /*
     * Verify that the selected endpoint is valid and the dataSize
     * is non-zero.
     */
    dmaset_p = &(pAppCtxt->endpInDma[endpNumber]);
    if ((dmaset_p->valid == 0) || (dataSize == 0)) {
        DBG_APP_ERR("QueueWrite Err1\r\n");
        return;
    }

    Cy_USBHS_App_QueueWrite(dmaset_p, pBuffer, dataSize);
} /* end of function */

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
void Cy_USB_AppInitDmaIntr(uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection,
                           cy_israddress userIsr)
{
    cy_stc_sysint_t intrCfg;
    if ((endpNumber > 0) && (endpNumber < CY_USB_MAX_ENDP_NUMBER))
    {
        intrCfg.intrPriority = 5;
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            /* DW1 channels 0 onwards are used for IN endpoints. */
            intrCfg.intrSrc =
                (IRQn_Type)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
        else
        {
            /* DW0 channels 0 onwards are used for OUT endpoints. */
            intrCfg.intrSrc =
                (IRQn_Type)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
        }

        if (userIsr != NULL)
        {
            /* If an ISR is provided, register it and enable the interrupt. */
            Cy_SysInt_Init(&intrCfg, userIsr);
            NVIC_EnableIRQ(intrCfg.intrSrc);
        }
        else
        {
            /* ISR is NULL. Disable the interrupt. */
            NVIC_DisableIRQ(intrCfg.intrSrc);
        }
    }
} /* end of function. */

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
void Cy_USB_AppClearDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                 uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection)
{
    if ((pAppCtxt != NULL) && (endpNumber > 0) &&
            (endpNumber < CY_USB_MAX_ENDP_NUMBER)) {
        if (endpDirection == CY_USB_ENDP_DIR_IN) {
            Cy_USBHS_App_ClearDmaInterrupt(&(pAppCtxt->endpInDma[endpNumber]));
        } else  {
            Cy_USBHS_App_ClearDmaInterrupt(&(pAppCtxt->endpOutDma[endpNumber]));
        }
    }
} /* end of function. */




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
void Cy_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking)
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, RED);
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        Cy_Debug_AddToLog(1, COLOR_RESET);
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

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
void Cy_CheckStatusAndHandleFailure(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking, void (*failureHandler)(void))
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, RED);
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        Cy_Debug_AddToLog(1, COLOR_RESET);

        if(failureHandler != NULL)
        {
            (*failureHandler)();
        }
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

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
void Cy_FailHandler(void)
{
    DBG_APP_ERR("Fail Handler\r\n");
}

/*[]*/
