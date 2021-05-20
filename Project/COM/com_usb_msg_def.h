/*!
 * \file      com_usb_msg_def.h
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

#ifndef __COM_COM_USB_MSG_DEF_H__
#define __COM_COM_USB_MSG_DEF_H__

#ifdef __cplusplus
extern "C" {
#endif
/* -------------------------------------------------------------------------- */
/* --- DEPENDENCIES --------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS & PREPROCESSOR CONSTANTS ------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC ENUMERATIONS, STRUCTURES & TYPES ------------------------------ */

typedef enum order_id_e
{
    ORDER_ID__REQ_PING            = 0x00,
    ORDER_ID__REQ_GET_STATUS      = 0x01,
    ORDER_ID__REQ_BOOTLOADER_MODE = 0x02,
    ORDER_ID__REQ_RESET           = 0x03,
    ORDER_ID__REQ_WRITE_GPIO      = 0x04,
    ORDER_ID__REQ_MULTIPLE_SPI    = 0x05,

    ORDER_ID__ACK_PING            = 0x40,
    ORDER_ID__ACK_GET_STATUS      = 0x41,
    ORDER_ID__ACK_BOOTLOADER_MODE = 0x42,
    ORDER_ID__ACK_RESET           = 0x43,
    ORDER_ID__ACK_WRITE_GPIO      = 0x44,
    ORDER_ID__ACK_MULTIPLE_SPI    = 0x45,

    ORDER_ID__CMD_ERROR = 0xFF
} order_id_t;

typedef enum cmd_order_offset_e
{
    CMD_OFFSET__ID,
    CMD_OFFSET__SIZE_MSB,
    CMD_OFFSET__SIZE_LSB,
    CMD_OFFSET__CMD,
    CMD_OFFSET__DATA
} cmd_order_offset_t;

// ------------------------------------------------------------------------------------------------------

#define MAX_SIZE_COMMAND ( 4200 )
#define MAX_SPI_COMMAND ( MAX_SIZE_COMMAND - CMD_OFFSET__DATA - 1 )

// Command without payload
#define ACK_BOOTLOADER_MODE_SIZE ( 0 )
#define REQ_BOOTLOADER_MODE_SIZE ( 0 )
#define REQ_GET_STATUS_SIZE ( 0 )
#define REQ_PING_SIZE ( 0 )
#define CMD_ERROR_SIZE ( 0 )

typedef enum cmd_offset_req_reset_e
{
    REQ_RESET__TYPE,
    REQ_RESET_SIZE
} cmd_offset_req_reset_t;

typedef enum cmd_offset_req_write_gpio_e
{
    REQ_WRITE_GPIO__PORT,
    REQ_WRITE_GPIO__PIN,
    REQ_WRITE_GPIO__STATE,
    REQ_WRITE_GPIO_SIZE
} cmd_offset_req_write_gpio_t;

typedef enum cmd_offset_ack_ping_e
{
    ACK_PING__UNIQUE_ID_0,
    ACK_PING__UNIQUE_ID_1,
    ACK_PING__UNIQUE_ID_2,
    ACK_PING__UNIQUE_ID_3,
    ACK_PING__UNIQUE_ID_4,
    ACK_PING__UNIQUE_ID_5,
    ACK_PING__UNIQUE_ID_6,
    ACK_PING__UNIQUE_ID_7,
    ACK_PING__UNIQUE_ID_8,
    ACK_PING__UNIQUE_ID_9,
    ACK_PING__UNIQUE_ID_10,
    ACK_PING__UNIQUE_ID_11,
    ACK_PING__VERSION_0,
    ACK_PING__VERSION_1,
    ACK_PING__VERSION_2,
    ACK_PING__VERSION_3,
    ACK_PING__VERSION_4,
    ACK_PING__VERSION_5,
    ACK_PING__VERSION_6,
    ACK_PING__VERSION_7,
    ACK_PING__VERSION_8,
    ACK_PING_SIZE,
} cmd_offset_ack_ping_t;

typedef enum cmd_offset_ack_get_status_e
{
    ACK_GET_STATUS__SYSTEM_TIME_31_24,
    ACK_GET_STATUS__SYSTEM_TIME_23_16,
    ACK_GET_STATUS__SYSTEM_TIME_15_8,
    ACK_GET_STATUS__SYSTEM_TIME_7_0,
    ACK_GET_STATUS__TEMPERATURE_15_8,
    ACK_GET_STATUS__TEMPERATURE_7_0,
    ACK_GET_STATUS_SIZE
} cmd_offset_ack_get_status_t;

typedef enum cmd_offset_ack_gpio_write_e
{
    ACK_GPIO_WRITE__STATUS,
    ACK_GPIO_WRITE_SIZE
} cmd_offset_ack_gpio_write_t;

typedef enum cmd_offset_ack_reset_e
{
    ACK_RESET__STATUS,
    ACK_RESET_SIZE
} cmd_offset_ack_reset_t;

typedef enum reset_type_e
{
    RESET_TYPE__GTW,
} reset_type_t;

/* -------------------------------------------------------------------------- */
/* --- DECLARATION OF PUBLIC VARIABLES  ------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- DECLARATION OF PUBLIC FUNCTIONS -------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __COM_COM_USB_MSG_DEF_H__ */

//===== END OF FILE =====================================================================
