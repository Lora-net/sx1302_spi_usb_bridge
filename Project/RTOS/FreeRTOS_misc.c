/*!
 * \file      FreeRTOS_misc.c
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
#include "FreeRTOS_misc.h"
#include "Project/global.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Project/RTOS/Task/com_task.h"
#include "Project/RTOS/Task/temperature_task.h"
#include "Project/COM/com_usb_msg_def.h"

/* -------------------------------------------------------------------------- */
/* --- CONSTANTS, TYPES, ENUMS & STRUCTS ------------------------------------ */
#define TASK_PRIORITY__COM_TX ( 6 )
#define TASK_PRIORITY__COM_RX ( 4 )
#define TASK_PRIORITY__TEMPERATURE ( 2 )

// Stack size in word (32bits)
#define STACK_SIZE__COM_RX ( 256 )
#define STACK_SIZE__COM_TX ( 256 )
#define STACK_SIZE__GENERIC ( configMINIMAL_STACK_SIZE )
#define STACK_SIZE__TEMPERATURE ( 256 )

#define STREAMBUFFER_SIZE__COM_RX ( MAX_SIZE_COMMAND )  // in bytes
#define MSGBUFFER_SIZE__COM_TX ( MAX_SIZE_COMMAND )

/* -------------------------------------------------------------------------- */
/* --- MACROS --------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- VARIABLES ------------------------------------------------------------ */
StreamBufferHandle_t  g_streambuffer_com_rx;
MessageBufferHandle_t g_msgbuffer_com_tx;

static StaticStreamBuffer_t  streambuffer_mgt_com_rx;
static StaticMessageBuffer_t msgbuffer_mgt_com_tx;

static uint8_t streambuffer_buffer_com_rx[STREAMBUFFER_SIZE__COM_RX];
static uint8_t msgbuffer_buffer_com_tx[MSGBUFFER_SIZE__COM_TX];

// Task buffer
static StaticTask_t task_buffer_radio_com_rx;
static StaticTask_t task_buffer_radio_com_tx;
static StaticTask_t task_buffer_idle;
static StaticTask_t task_buffer_temperature;

static StackType_t stack_radio_com_rx[STACK_SIZE__COM_RX];
static StackType_t stack_radio_com_tx[STACK_SIZE__COM_TX];
static StackType_t stack_idle[configMINIMAL_STACK_SIZE];
static StackType_t stack_temperature[STACK_SIZE__TEMPERATURE];

/* -------------------------------------------------------------------------- */
/* ---  FUNCTIONS DECLARATION ----------------------------------------------- */

/**
 * @brief Init the freeRTOS and buffer
 *
 * Note: the function never exit
 *
 */
void freertos__init( void )
{
    // Create buffer message
    g_streambuffer_com_rx =
        xStreamBufferCreateStatic( STREAMBUFFER_SIZE__COM_RX, 1, streambuffer_buffer_com_rx, &streambuffer_mgt_com_rx );

    g_msgbuffer_com_tx =
        xMessageBufferCreateStatic( MSGBUFFER_SIZE__COM_TX, msgbuffer_buffer_com_tx, &msgbuffer_mgt_com_tx );

    // Create tasks
    TaskHandle_t task_status;

    task_status = xTaskCreateStatic( com_rx__start_task, "COM_RX", STACK_SIZE__COM_RX, NULL, TASK_PRIORITY__COM_RX,
                                     stack_radio_com_rx, &task_buffer_radio_com_rx );

    assert_param( task_status != NULL );

    task_status = xTaskCreateStatic( com_tx__start_task, "COM_TX", STACK_SIZE__COM_TX, NULL, TASK_PRIORITY__COM_TX,
                                     stack_radio_com_tx, &task_buffer_radio_com_tx );

    assert_param( task_status != NULL );

    task_status = xTaskCreateStatic( temperature_task__start, "TEMPERATURE", STACK_SIZE__TEMPERATURE, NULL,
                                     TASK_PRIORITY__TEMPERATURE, stack_temperature, &task_buffer_temperature );

    assert_param( task_status != NULL );

    vTaskStartScheduler( );
}

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS( );
    for( ;; )
        ;
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char* pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS( );
    for( ;; )
        ;
}

/*
  vApplicationGetIdleTaskMemory gets called when configSUPPORT_STATIC_ALLOCATION
  equals to 1 and is required for static memory allocation support.
*/
void vApplicationGetIdleTaskMemory( StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer,
                                    uint32_t* pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer   = &task_buffer_idle;
    *ppxIdleTaskStackBuffer = &stack_idle[0];
    *pulIdleTaskStackSize   = ( uint32_t ) configMINIMAL_STACK_SIZE;
}

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* --- EOF ------------------------------------------------------------------ */
