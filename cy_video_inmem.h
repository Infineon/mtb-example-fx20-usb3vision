/***************************************************************************//**
* \file cy_video_inmem.h
* \version 1.0
*
* Defines the interfaces used in streaming of video data from internal RAM buffers.
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


#ifndef _CY_VIDEO_INMEM_H_
#define _CY_VIDEO_INMEM_H_

#if defined(__cplusplus)
extern "C" {
#endif

#define U3V_INMEM_LEADER_SIZE                              (U3V_LEADER_SIZE )
#define U3V_INMEM_TRAILER_SIZE                             (U3V_TRAILER_SIZE)

#define U3V_INMEM_BUFFER_SIZE                              (STREAM_DMA_BUFFER_SIZE)
#define U3V_INMEM_BUFFER_COUNT                             (8)

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
bool Cy_U3V_MemAllocateBuffers(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_hbdma_channel_t *pChannel);

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
Cy_U3V_MemStreamCb ( cy_stc_hbdma_channel_t *handle, cy_en_hbdma_cb_type_t type, 
                   cy_stc_hbdma_buff_status_t* pbufStat, void *userCtx);

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
void Cy_U3V_MemCommitFrameBuffers (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_hbdma_channel_t *pChannel);

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
void Cy_U3V_MemCommitBuffer (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_hbdma_channel_t *pChannel, uint32_t bufferCounter);

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
bool Cy_U3V_MemFillBuffers(void);

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
Cy_U3V_MemClearBufPointers( cy_stc_usb_app_ctxt_t  *pAppCtxt, cy_stc_hbdma_channel_t *pChannel);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_VIDEO_INMEM_H_ */
