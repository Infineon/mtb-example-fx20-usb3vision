/***************************************************************************//**
* \file cy_video_inmem.c
* \version 1.0
*
* Implements code to handle streaming of video data from internal RAM buffers.
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
#include "cy_usb_i2c.h"
#include "cy_usb_qspi.h"
#include "cy_video_inmem.h"
#include "cy_u3v_device_config.h"

typedef struct {
    uint32_t *pU3vLeaderBuf;             /* U3V leader buffer */
    uint32_t *pU3vTrailerBuf;            /* U3V trailer buffer */
    uint8_t *pU3vStrmBuf1;               /* U3V payload buffer */
    uint8_t *pU3vStrmBuf2;               /* U3V payload buffer */
} cy_u3v_app_frame_info_t;

#if U3V_INMEM_EN
/* Structure holding all In-MEM streaming info. */
static cy_u3v_app_frame_info_t glInMemInfo =
{
    .pU3vLeaderBuf = NULL,
    .pU3vTrailerBuf = NULL,
    .pU3vStrmBuf1 = NULL,
    .pU3vStrmBuf2 = NULL
};

const uint32_t colorInfo[8] = {
    BAND1_COLOR_YUYV,
    BAND2_COLOR_YUYV,
    BAND3_COLOR_YUYV,
    BAND4_COLOR_YUYV,
    BAND5_COLOR_YUYV,
    BAND6_COLOR_YUYV,
    BAND7_COLOR_YUYV,
    BAND8_COLOR_YUYV,
};


/*****************************************************************************
* Function Name: Cy_U3V_MemStreamCb
******************************************************************************
* Summary:
* Function that handles DMA transfers on the USB-SS BULK-IN
* endpoint used to send in-ememory U3V video data
*
* \param pAppCtxt
* application layer context pointer.
*
* \param handle 
* HBDMA channel handle
*
* \param type
* HBDMA channel type                            
*
* \param pbufStat       
* HBDMA buffer status
*
* \param userCtx
* user context
*
* \return
* None
*
 *******************************************************************************/
void
Cy_U3V_MemStreamCb (
        cy_stc_hbdma_channel_t *handle,
        cy_en_hbdma_cb_type_t type,
        cy_stc_hbdma_buff_status_t* pbufStat,
        void *userCtx)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;

    if (type == CY_HBDMA_CB_CONS_EVENT) 
    {
        if (glFlags.lastFrameProduced)
        {
            glFlags.lastFrameProduced = false;
            glFlags.lastFrameConsumed = true;
        }

        if(glU3VIsApplnActive == true)
        {
            Cy_U3V_MemCommitBuffer( pAppCtxt, handle, glU3VBufCounter);

            if(glU3VBufCounter == glU3VFrameBuffer)
            {
                glU3VBufCounter = CY_USB_U3V_LEADER_BUFFER_NO;
            }
            else
            {
                glU3VBufCounter++;
            }
        }
    }
}  /* end of function  */  

/*****************************************************************************
* Function Name: Cy_U3V_MemFillBuffers
******************************************************************************
* Summary:
* Function to allocate buffers for in-memory transfers.
*
* \param
* None
*
* \return
* true if buffers are filled else false
*
 *******************************************************************************/
bool Cy_U3V_MemFillBuffers(void)
{
    uint32_t ln, band, pxl;
    uint32_t *p1, *p2;

    if(glInMemInfo.pU3vStrmBuf1 == NULL || glInMemInfo.pU3vStrmBuf2 == NULL)
    {
        return false;
    }

    p1 = (uint32_t *)glInMemInfo.pU3vStrmBuf1;
    p2 = (uint32_t *)glInMemInfo.pU3vStrmBuf2;

    DBG_APP_INFO("Fill in coloarbar \r\n");

    for (ln = 0; ln < (360 / glU3VColorbarSize); ln++) {
        for (band = 0; band < 8; band++) {
            for (pxl = 0; pxl < (glU3VColorbarSize * 4); pxl++) {
                *p1++ = colorInfo[band];
                *p2++ = colorInfo[band];
            }
        }
    }
    return true;
}

/*****************************************************************************
* Function Name: Cy_U3V_MemAllocateBuffers
******************************************************************************
* Summary:
* Function to allocate buffers for in-memory transfers.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pChannel
* pointer to the DMA channel handle
*
* \return
* None
*
 *******************************************************************************/
bool Cy_U3V_MemAllocateBuffers(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_hbdma_channel_t *pChannel)
{
    cy_stc_hbdma_desc_t dscr;
    uint16_t dscrIndex, i;
    uint16_t leaderSize;
    cy_stc_hbdma_buf_mgr_t *pBufMgr;

    if ((pAppCtxt == NULL) || (pChannel == NULL)) {
        return false;
    }

    pBufMgr = pAppCtxt->pHbDmaMgrCtxt->pBufMgr;
    if (
        (glInMemInfo.pU3vLeaderBuf == NULL) ||
        (glInMemInfo.pU3vTrailerBuf == NULL) ||
        (glInMemInfo.pU3vStrmBuf1 == NULL) ||
        (glInMemInfo.pU3vStrmBuf2  == NULL)
    )
    {
        glInMemInfo.pU3vLeaderBuf = (uint32_t *)Cy_HBDma_BufMgr_Alloc(pBufMgr,  U3V_INMEM_LEADER_SIZE);
        glInMemInfo.pU3vTrailerBuf = (uint32_t *)Cy_HBDma_BufMgr_Alloc(pBufMgr, U3V_INMEM_TRAILER_SIZE);

        glInMemInfo.pU3vStrmBuf1 = (uint8_t *)Cy_HBDma_BufMgr_Alloc(pBufMgr, U3V_INMEM_BUFFER_SIZE);
        glInMemInfo.pU3vStrmBuf2  = (uint8_t *)Cy_HBDma_BufMgr_Alloc(pBufMgr, U3V_INMEM_BUFFER_SIZE);
    }  

    if (
            (glInMemInfo.pU3vLeaderBuf == NULL) ||
            (glInMemInfo.pU3vTrailerBuf == NULL) ||
            (glInMemInfo.pU3vStrmBuf1 == NULL) ||
            (glInMemInfo.pU3vStrmBuf2  == NULL)
       ) {
        DBG_APP_ERR("Failed to allocate buffers to hold video data\r\n");
        if (glInMemInfo.pU3vLeaderBuf != NULL)
            Cy_HBDma_BufMgr_Free(pBufMgr, glInMemInfo.pU3vLeaderBuf);
        if (glInMemInfo.pU3vTrailerBuf != NULL)
            Cy_HBDma_BufMgr_Free(pBufMgr, glInMemInfo.pU3vTrailerBuf);
        if (glInMemInfo.pU3vStrmBuf1 != NULL)
            Cy_HBDma_BufMgr_Free(pBufMgr, glInMemInfo.pU3vStrmBuf1);
        if (glInMemInfo.pU3vStrmBuf2 != NULL)
            Cy_HBDma_BufMgr_Free(pBufMgr, glInMemInfo.pU3vStrmBuf2);
        return false;
    }

    /* Fill the buffers with the data to be sent. */
    Cy_U3V_MemFillBuffers();

    dscrIndex = pChannel->firstProdDscrIndex[0];
    Cy_HBDma_GetDescriptor(dscrIndex, &dscr);
    if (dscr.pBuffer != NULL) {
        Cy_HBDma_BufMgr_Free(pChannel->pContext->pBufMgr, dscr.pBuffer);
    }
    dscr.pBuffer = (uint8_t *)glInMemInfo.pU3vLeaderBuf;
    Cy_HBDma_SetDescriptor(dscrIndex, &dscr);

    Cy_U3V_GetUpdatedLeaderData(&u3vDevCtxt,0, (uint8_t *)glInMemInfo.pU3vLeaderBuf,&leaderSize);
    DBG_APP_INFO("Leader Size = %d\r\n", leaderSize);

    for (i = 1; i < U3V_INMEM_BUFFER_COUNT; i++) {
        dscrIndex = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscr.chain);
        Cy_HBDma_GetDescriptor(dscrIndex, &dscr);
        if (dscr.pBuffer != NULL) {
            Cy_HBDma_BufMgr_Free(pChannel->pContext->pBufMgr, dscr.pBuffer);
        }
        dscr.pBuffer = glInMemInfo.pU3vStrmBuf1;
        Cy_HBDma_SetDescriptor(dscrIndex, &dscr);

        dscrIndex = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscr.chain);
        Cy_HBDma_GetDescriptor(dscrIndex, &dscr);
        if (dscr.pBuffer != NULL) {
            Cy_HBDma_BufMgr_Free(pChannel->pContext->pBufMgr, dscr.pBuffer);
        }
        dscr.pBuffer = glInMemInfo.pU3vStrmBuf2;
        Cy_HBDma_SetDescriptor(dscrIndex, &dscr);
    }

    return true;
}

/*****************************************************************************
* Function Name: Cy_U3V_MemCommitBuffer
******************************************************************************
* Summary:
* Function to commit buffers for in-memory transfers.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pChannel
* pointer to the DMA channel handle
*
* \param bufferCounter
* buffer counter to identify the buffer type (leader, trailer, or stream).
*
* \return
* None
*
 *******************************************************************************/
void Cy_U3V_MemCommitBuffer (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_hbdma_channel_t *pChannel, uint32_t bufferCounter)
{
    cy_en_hbdma_mgr_status_t   dmaStat;
    cy_stc_hbdma_buff_status_t bufStat;


    if(pChannel==NULL || pAppCtxt == NULL)
    {
        DBG_APP_ERR("Buffer status or channel or app context is NULL\r\n");
        return;
    }

    dmaStat = Cy_HBDma_Channel_GetBuffer(pChannel, &bufStat);
    if (dmaStat != CY_HBDMA_MGR_SUCCESS) {
        DBG_APP_ERR("GetBuf fail %x, buf count %d\r\n", dmaStat, bufferCounter);
        return;
    }

    if (bufferCounter == CY_USB_U3V_LEADER_BUFFER_NO) {
        glFlags.frameOngoing = true;
        glFlags.lastFrameConsumed = false;
        glFlags.lastFrameProduced = false;
        bufStat.pBuffer = (uint8_t *)glInMemInfo.pU3vLeaderBuf;
        Cy_U3V_GetUpdatedLeaderData(&u3vDevCtxt,0, bufStat.pBuffer,(uint16_t *)&(bufStat.count));
    } else {
        if (bufferCounter == (glU3VFrameBuffer)) {
            bufStat.pBuffer = (uint8_t *)glInMemInfo.pU3vTrailerBuf;
            Cy_U3V_GetUpdatedTrailerData(&u3vDevCtxt, bufStat.pBuffer,(uint16_t *)&(bufStat.count));
            pAppCtxt->frameSize =  pAppCtxt->frameSizeTransferred;
            pAppCtxt->frameSizeTransferred = 0;
            glFlags.lastFrameProduced = true;
            glFlags.frameOngoing = false;
            pAppCtxt->fpsCount++;
        } 
        else if (bufferCounter == (glU3VFrameBuffer - 1))
        {
            bufStat.pBuffer = (bufferCounter & 1u)?(uint8_t *)glInMemInfo.pU3vStrmBuf2:(uint8_t *)glInMemInfo.pU3vStrmBuf1;

            /* This is the last buffer in the frame, which may be a partial buffer */
            if(pAppCtxt->isPartialBuf == 1u)
            {
                bufStat.count = pAppCtxt->partialBufSize;
            }
            else
            {
                bufStat.count = U3V_INMEM_BUFFER_SIZE;
            }
        
            pAppCtxt->frameSizeTransferred += bufStat.count;
        }
        else
        {
            bufStat.pBuffer = (bufferCounter & 1u)?(uint8_t *)glInMemInfo.pU3vStrmBuf2:(uint8_t *)glInMemInfo.pU3vStrmBuf1;
            bufStat.count = U3V_INMEM_BUFFER_SIZE; 
            pAppCtxt->frameSizeTransferred += bufStat.count;
        }
    }

    if(pAppCtxt->devSpeed > CY_USBD_USB_DEV_HS)
    {
        dmaStat = Cy_HBDma_Channel_CommitBuffer(pChannel,&bufStat);
        if (dmaStat != CY_HBDMA_MGR_SUCCESS) {
            DBG_APP_ERR("CommitBuf fail %x\r\n",dmaStat);
            return;
        }
    }
    else
    {
        Cy_USB_AppQueueWrite(pAppCtxt, CY_U3V_EP_DSI_STREAM, bufStat.pBuffer, bufStat.count);
    }
    
}

/*****************************************************************************
* Function Name: Cy_U3V_MemCommitFrameBuffers
******************************************************************************
* Summary:
* Function to commit buffers for in-memory transfers.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pChannel
* pointer to the DMA channel handle
*
* \return
* None
*
 *******************************************************************************/
void Cy_U3V_MemCommitFrameBuffers (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_hbdma_channel_t *pChannel)
{
    uint32_t i;
    uint32_t intrState;

    intrState = Cy_SysLib_EnterCriticalSection();

    glU3VBufCounter = CY_USB_U3V_LEADER_BUFFER_NO;
    
    if(pAppCtxt->devSpeed > CY_USBD_USB_DEV_HS)
    {
        for (i = CY_USB_U3V_LEADER_BUFFER_NO; i <= U3V_INMEM_BUFFER_COUNT; i++) {
            /* Commit the data buffer to USB */
            Cy_U3V_MemCommitBuffer (pAppCtxt, pChannel, i);
        }
    }
    else 
    {
        Cy_U3V_MemCommitBuffer (pAppCtxt, pChannel, CY_USB_U3V_LEADER_BUFFER_NO);
        i = 2;
    }
    
    glU3VBufCounter = i;
    Cy_SysLib_ExitCriticalSection(intrState);

}

/******************************************************************************
 * Function Name: Cy_U3V_MemClearBufPointers
 ******************************************************************************
 *
 * Clear the buffer pointers in all descriptors associated with the U3V streaming
 * channel before the channel is destroyed. This ensures that the DMA buffers
 * are not freed when the channel gets destroyed.
 *
 * \param pAppCtxt
 * Pointer to U3V application context structure.
 *
 * \param pChannel
 * Handle to the U3V video streaming DMA channel.
 *
 * \return
 * true if the operation is successful, false otherwise.
 *****************************************************************************/
bool
Cy_U3V_MemClearBufPointers(
        cy_stc_usb_app_ctxt_t  *pAppCtxt,
        cy_stc_hbdma_channel_t *pChannel)
{
    cy_stc_hbdma_desc_t dscr;
    uint16_t dscrIndex, i;

    if ((pAppCtxt == NULL) || (pChannel == NULL)) {
        DBG_APP_ERR("Unable to clear buffer pointers: pAppCtxt=%x pChannel=%x\r\n", pAppCtxt, pChannel);
        return false;
    }

    /* Mark the buffer pointer for all DMA descriptors as NULL so that channel destroy does not free them. */
    i         = 0;
    dscrIndex = pChannel->firstProdDscrIndex[0];
    do {
        Cy_HBDma_GetDescriptor(dscrIndex, &dscr);
        dscr.pBuffer = NULL;
        Cy_HBDma_SetDescriptor(dscrIndex, &dscr);
        dscrIndex = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscr.chain);
        i++;
    } while (i < U3V_INMEM_BUFFER_COUNT);

    return true;
}
#endif /* U3V_INMEM_EN */

