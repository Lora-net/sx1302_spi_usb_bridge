/*!
 * \file      com_task.c
 *
 * \brief
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2019. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH S.A. BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* -------------------------------------------------------------------------- */
/* --- DEPENDENCIES --------------------------------------------------------- */
#include "com_task.h"
#include "Project/global.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "Project/RTOS/FreeRTOS_misc.h"
#include "Project/COM/com_usb.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

/* -------------------------------------------------------------------------- */
/* --- CONSTANTS, TYPES, ENUMS & STRUCTS ------------------------------------ */
#define MAX_SIZE_BUFFER_PROCESS ( 64 )
#define DELAY_TIME_OUT_COM ( 100 )

static SemaphoreHandle_t sema_com_tx;  // semaphore to protect the USB tx access
static StaticSemaphore_t mutex_buffer_com_tx;
static TaskHandle_t      task_com_tx_handle;
static uint8_t           tx_buffer[MAX_SIZE_COMMAND];

/* -------------------------------------------------------------------------- */
/* --- MACROS --------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- VARIABLES ------------------------------------------------------------ */

/* -------------------------------------------------------------------------- */
/* ---  FUNCTIONS DECLARATION ----------------------------------------------- */

/**
 * @brief Function implementing the com_task thread.
 * @param argument: Not used
 * @retval None
 */
void com_rx__start_task( void* argument )
{
    UNUSED( argument );

    // Init USB init
    MX_USB_DEVICE_Init( );

    uint8_t    buffer[MAX_SIZE_BUFFER_PROCESS];
    TickType_t read_wait;
    uint16_t   size_rx;
    bool       is_new_frame = true;

    for( ;; )
    {
        // Inifinite wait if no frame is pending, wait 100ms for timeout if frame is pending
        read_wait = ( is_new_frame == false ) ? pdMS_TO_TICKS( DELAY_TIME_OUT_COM ) : portMAX_DELAY;

        size_rx =
            ( uint16_t ) xStreamBufferReceive( g_streambuffer_com_rx, buffer, MAX_SIZE_BUFFER_PROCESS, read_wait );

        if( size_rx == 0 )
        {
            // Time out on read car
            is_new_frame = true;
        }
        else
        {
            if( com_usb__process_buffer( is_new_frame, buffer, size_rx ) == true )
            {
                is_new_frame = true;
            }
            else
            {
                // incomplete frame
                is_new_frame = false;
            }
        }
    }
}

/**
 * @brief Start TX task: manage TX packet over USB
 *
 * @param argument
 */
void com_tx__start_task( void* argument )
{
    UNUSED( argument );  // Task withtout argument - avoid warning);

    task_com_tx_handle = xTaskGetCurrentTaskHandle( );
    sema_com_tx        = xSemaphoreCreateMutexStatic( &mutex_buffer_com_tx );

    uint16_t   size_packet;
    uint16_t   size_buffer;
    TickType_t delay;

    for( ;; )
    {
        delay       = portMAX_DELAY;  // start and infinite wait
        size_buffer = 0;              // and clear buffer size

        do
        {
            size_packet = ( uint16_t ) xMessageBufferReceive( g_msgbuffer_com_tx, tx_buffer + size_buffer,
                                                              MAX_SIZE_COMMAND - size_buffer, delay );
            // Read as much data as possible without delay (better perf on big packet !)
            delay = 0;
            size_buffer += size_packet;
        } while( size_packet != 0 );

        if( size_buffer != 0 )
        {
            // Transmit USB buffer
            if( CDC_Transmit_FS( tx_buffer, size_buffer ) != USBD_OK )
            {
                // USB com tx should be free by design
                Error_Handler( );
            }

            // Wait notif from USB complete interrupt (max block is < 1ms)
            // The delay is set to 250ms because the host may be busy.
            // It is not an issue because the host will not send an other command before reading this answer.
            if( ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS( 250 ) ) != pdTRUE )
            {
                // USB com tx should not take as much time!
                Error_Handler( );
            }
        }
    }
}

/**
 * @brief Enqueue buffer to be send on USB to host
 *
 * @param buffer buffer to send
 * @param buffer_len buffer length
 * @return true buffer have been send to the tx task and will be process
 * @return false fail to copy buffer
 */
void com_tx__post_message( const uint8_t* buffer, const uint16_t buffer_len )
{
	if ( xSemaphoreTake( sema_com_tx, pdMS_TO_TICKS( 10 ) ) != pdPASS )
    {
        Error_Handler( );
    }

    if( xMessageBufferSend( g_msgbuffer_com_tx, buffer, buffer_len, pdMS_TO_TICKS( 10 ) ) != buffer_len )
    {
        Error_Handler( );
    }

    if( xSemaphoreGive( sema_com_tx ) != pdPASS )
    {
        Error_Handler( );
    }
}

/**
 * @brief Function to call when tx usb transfer is done to wake up TX usb task
 *
 * Note: Need to be called in USBD_CDC_DataIn
 *
 */
void com_usb_it_transfer_complete( void )
{
    // Call from ISR - USB transfert complete
    BaseType_t yeld_task = pdFALSE;
    vTaskNotifyGiveFromISR( task_com_tx_handle, &yeld_task );
    portYIELD_FROM_ISR( yeld_task );
}

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* --- EOF ------------------------------------------------------------------ */
