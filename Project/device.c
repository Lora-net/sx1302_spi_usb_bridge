/*!
 * \file      device.c
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
#include "device.h"
#include "Project/global.h"
#include "Project/COM/com_usb.h"
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "spi.h"

/* -------------------------------------------------------------------------- */
/* --- CONSTANTS, TYPES, ENUMS & STRUCTS ------------------------------------ */
typedef enum spi_cs_idx_e
{
    SPI_IDX__SX1302 = 0,
    SPI_IDX__SX1261 = 1,
    SPI_IDX_SIZE
} spi_cs_idx_t;

typedef enum spi_order_e
{
    SPI_ORDER__READ_WRITE = 1,
    SPI_ORDER__RMW        = 2,
} spi_order_t;

typedef enum spi_return_value_e
{
    SPI_STATUS__OK          = 0,
    SPI_STATUS__FAIL        = 1,
    SPI_STATUS__WRONG_PARAM = 2,
    SPI_STATUS__TIMEOUT     = 3,
} spi_return_value_t;

#define SX1302_READ_ACCESS ( 0x00 )
#define SX1302_WRITE_ACCESS ( 0x80 )

#define RMW_ACK_DATA_RMW_LEN ( 2 )
/* -------------------------------------------------------------------------- */
/* --- MACROS --------------------------------------------------------------- */

#if !defined( END_SRAM_ADDR )
// See memory map of you device to find the where the ram is located
#error "END_SRAM_ADDR is not define for this chip, please edit project settings "
#endif

#define GO_TO_BOOT_VALUE ( 0xDEADBEEF )

/* -------------------------------------------------------------------------- */
/* --- VARIABLES ------------------------------------------------------------ */

/* -------------------------------------------------------------------------- */
/* ---  FUNCTIONS DECLARATION ----------------------------------------------- */
static spi_return_value_t read_modify_write_sx1302( const uint16_t addr, const uint8_t mask, const uint8_t value,
                                                    uint8_t* data_out );
static spi_return_value_t read_write_spi( const spi_cs_idx_t target, uint8_t* buff_in, uint8_t* buff_out,
                                          const uint16_t buff_len );

/**
 * @brief Init all device related variable and tools
 */
void device__init( void )
{
    HAL_GPIO_WritePin( SX1302_CS_GPIO_Port, SX1302_CS_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( SX1261_CS_GPIO_Port, SX1261_CS_Pin, GPIO_PIN_SET );

    HAL_GPIO_WritePin( SX1302_RESET_MCU_GPIO_Port, SX1302_RESET_MCU_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( SX1261_RESET_GPIO_Port, SX1261_RESET_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( POWER_EN_MCU_GPIO_Port, POWER_EN_MCU_Pin, GPIO_PIN_RESET );
}

/**
 * @brief Check boot flag and jump if required in bootloader
 *
 * This function needs to be called first thing in the function Reset_Handler
 * in file startup_stm32xxxx.s
 *
 */
void __attribute__( ( optimize( "O0" ) ) ) device__check_bootloader( void )
{
    // Read flag value at the end of sram
    uint32_t boot_value = *( ( unsigned long* ) END_SRAM_ADDR );

    // Check if it's required to jump to bootloader
    if( boot_value == GO_TO_BOOT_VALUE )
    {
        // Clear flag init bootloader mode
        *( ( unsigned long* ) END_SRAM_ADDR ) = 0;
        device__jump_bootloader( );
    }
}

/**
 * @brief Set boot flag at the end of sram and force reset
 *
 */
void __attribute__( ( optimize( "O0" ) ) ) device__set_bootloader( void )
{
    // Linux command to flash a device and leave the DFU mode
    // sudo dfu-util -a 0 -s 0x08000000:leave -t 0 -D fwm_XXXX.bin

    // Write GO_TO_BOOT_VALUE at the end of SRAM and force reset.
    // Then the Reset_Handler (startup_stm32xxxx.s) will call
    // device__check_bootloader and jump in the boot section in USB DFU mode (if required)

    // ------------------------------------------------------------------------------------
    // !! Check that device__check_bootloader is called first thing in the Reset_Handler !!
    // ------------------------------------------------------------------------------------

    *( ( unsigned long* ) END_SRAM_ADDR ) = GO_TO_BOOT_VALUE;  // End of RAM
    NVIC_SystemReset( );
}

/**
 * @brief Jump to bootloader
 *
 * Note: if an os is running, a reset is required before and it need to be called right away.
 */
void __attribute__( ( optimize( "O0" ) ) ) device__jump_bootloader( void )
{
    uint32_t jump_addr = 0;

#if defined( STM32L4 )
    jump_addr = 0x1FFF0000;
#else
#error "jump addr is not define for this chip"
#endif

    typedef void ( *pFunction )( void );
    pFunction JumpToApplication;
    HAL_RCC_DeInit( );
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // Step: Disable all interrupts
    __disable_irq( );
    // ARM Cortex-M Programming Guide to Memory Barrier Instructions.
    __DSB( );
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH( );
    // Remap is not visible at once. Execute some unrelated command!
    __DSB( );
    __ISB( );
    JumpToApplication = ( void ( * )( void ) )( *( ( uint32_t* ) ( jump_addr + 4 ) ) );

    // Initialize user application's Stack Pointer
    __set_MSP( *( __IO uint32_t* ) jump_addr );
    JumpToApplication( );
}

/**
 * @brief Start a multiple SPI transaction
 *
 *  This command allow you to perform several SPI operation in one shot to reduce the overal delay due to the USB
 *  communication. Format of the command is :
 *
 * Request 1 ID / Request 1 Type / Request 1 Payload / Request 2 ID / Request 2 Type / Request 2 Payload / ....
 *
 * Request ID is re-used in the response to match the response of each request.
 *
 *   Request type could be
 *    - SPI_ORDER__RMW to perform a Read Modify Write of a register on SX1302
 *    - SPI_ORDER__READ_WRITE to perform a standard SPI read/write on SX1302 or SX1261
 *
 * Return value format is :
 *  Request 1 ID / request 1 return value / Request 2 ID / request 2 return value / ...
 *
 * @param buffer        multiple SPI frame
 * @param buffer_len    length of the request. Max value is MAX_SPI_COMMAND
 * @return uint16_t     buffer_out length
 */

uint16_t device__spi_start_multiple( uint8_t* buffer_in, uint8_t* buffer_out, const uint16_t buffer_len )
{
    uint8_t*           buffer_in_ptr  = buffer_in;
    uint8_t*           buffer_out_ptr = buffer_out;
    spi_order_t        spi_order;
    spi_return_value_t ret_spi;

    if( ( buffer_len == 0 ) || ( buffer_len >= MAX_SPI_COMMAND ) )
    {
        return 0;  // Invalid length
    }

    while( ( buffer_in_ptr - buffer_in ) < buffer_len )
    {
        // Copy request id
        *buffer_out_ptr = *buffer_in_ptr;
        buffer_out_ptr++;
        buffer_in_ptr++;

        // Copy request type
        spi_order       = *buffer_in_ptr;
        *buffer_out_ptr = spi_order;
        buffer_in_ptr++;
        buffer_out_ptr++;

        if( spi_order == SPI_ORDER__RMW )
        {
            // Payload format of SPI_ORDER__RMW is
            //  - 2 bytes : register addr
            //  - 1 byte  : clear mask value
            //  - 1 byte  : set value
            //
            // Operation performed is ((register_value & ~clear_mask) | set_value)
            //
            // Return values has the following format
            //  - 1 byte : return value (OK, FAIL, WRONG_PARAM or TIMEOUT) see spi_return_value_t
            //  - 1 byte : register value before RMV
            //  - 1 byte : register value after RMV
            uint16_t addr = ( *buffer_in_ptr ) << 8;
            buffer_in_ptr++;

            addr += *buffer_in_ptr;
            buffer_in_ptr++;

            uint8_t mask = *buffer_in_ptr;
            buffer_in_ptr++;

            uint8_t value = *buffer_in_ptr;
            buffer_in_ptr++;

            // +1 --> 1 byte for return status
            ret_spi = read_modify_write_sx1302( addr, mask, value, buffer_out_ptr + 1 );

            *buffer_out_ptr = ret_spi;
            buffer_out_ptr++;

            if( ret_spi == SPI_STATUS__OK )
            {
                // If SPI is a success, update pointer value (register value before and after RMW have been added in
                // ouput buffer by function read_modify_write_sx1302)
                buffer_out_ptr += RMW_ACK_DATA_RMW_LEN;
            }
        }
        else if( spi_order == SPI_ORDER__READ_WRITE )
        {
            // Payload format of SPI_ORDER__READ_WRITE is
            //  - 1 bytes : SPI target (SX1302 or SX1261)
            //  - 2 bytes : payload length
            //  - X bytes : payload to write on SPI
            //
            // Return values has the following format
            //  - 1 byte  : return value (OK, FAIL, WRONG_PARAM or TIMEOUT) see spi_return_value_t
            //  - 2 bytes : payload length
            //  - X bytes : payload read on SPI
            spi_cs_idx_t tgt = ( spi_cs_idx_t ) *buffer_in_ptr;
            buffer_in_ptr++;

            uint16_t len = ( *buffer_in_ptr ) << 8;
            buffer_in_ptr++;

            len += *buffer_in_ptr;
            buffer_in_ptr++;

            // +3 --> 1 byte for return status, 2 bytes for size and data after
            ret_spi = read_write_spi( tgt, buffer_in_ptr, buffer_out_ptr + 3, len );
            buffer_in_ptr += len;  // Update buffer_in pointer

            if( ret_spi != SPI_STATUS__OK )
            {
                // If a problem appears during SPI transfert, don't add the data
                len = 0;
            }

            *buffer_out_ptr = ret_spi;
            buffer_out_ptr++;

            *buffer_out_ptr = ( uint8_t )( len >> 8 );
            buffer_out_ptr++;

            *buffer_out_ptr = ( uint8_t )( len & 0x00FF );
            buffer_out_ptr++;

            buffer_out_ptr += len;  // data has been added in the fct read_write_spi
        }
        else
        {
            *buffer_out_ptr = SPI_STATUS__WRONG_PARAM;
            buffer_out_ptr++;
        }
    }

    return ( uint16_t )( buffer_out_ptr - buffer_out );
}

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static spi_return_value_t read_modify_write_sx1302( const uint16_t addr, const uint8_t mask, const uint8_t value,
                                                    uint8_t* data_out )
{
    HAL_StatusTypeDef ret_spi;
    uint8_t           buffer[5];

    buffer[0] = 0;  // TGT is 1302
    buffer[1] = SX1302_READ_ACCESS | ( ( addr >> 8 ) & 0x7F );
    buffer[2] = ( ( addr >> 0 ) & 0xFF );
    buffer[3] = 0x00;  // Dummy byte
    buffer[4] = 0x00;  // Read value
    HAL_GPIO_WritePin( SX1302_CS_GPIO_Port, SX1302_CS_Pin, GPIO_PIN_RESET );
    ret_spi = HAL_SPI_TransmitReceive( &hspi1, buffer, buffer, 5, 10 );
    HAL_GPIO_WritePin( SX1302_CS_GPIO_Port, SX1302_CS_Pin, GPIO_PIN_SET );

    *data_out = buffer[4];  // Data before RMW

    if( ret_spi == HAL_OK )
    {
        buffer[0] = 0;  // TGT is 1302
        buffer[1] = SX1302_WRITE_ACCESS | ( ( addr >> 8 ) & 0x7F );
        buffer[2] = ( ( addr >> 0 ) & 0xFF );

        buffer[3] = *data_out;
        buffer[3] &= ~mask;
        buffer[3] |= value;

        *( data_out + 1 ) = buffer[3];  // Data after RMW

        HAL_GPIO_WritePin( SX1302_CS_GPIO_Port, SX1302_CS_Pin, GPIO_PIN_RESET );
        ret_spi = HAL_SPI_TransmitReceive( &hspi1, buffer, buffer, 4, 10 );
        HAL_GPIO_WritePin( SX1302_CS_GPIO_Port, SX1302_CS_Pin, GPIO_PIN_SET );
    }

    return ( ret_spi == HAL_OK ) ? SPI_STATUS__OK : SPI_STATUS__FAIL;
}

static spi_return_value_t read_write_spi( const spi_cs_idx_t target, uint8_t* buff_in, uint8_t* buff_out,
                                          const uint16_t buff_len )
{
    if( target == SPI_IDX__SX1302 )
    {
        HAL_GPIO_WritePin( SX1302_CS_GPIO_Port, SX1302_CS_Pin, GPIO_PIN_RESET );
    }
    else if( target == SPI_IDX__SX1261 )
    {
        if( HAL_GPIO_ReadPin( SX1261_BUSY_GPIO_Port, SX1261_BUSY_Pin ) == GPIO_PIN_SET )
        {
            // Chip is busy, wait (max 4ms)
            uint32_t timeout_tgt = ( uint32_t ) xTaskGetTickCount( ) + 4;

            while( ( HAL_GPIO_ReadPin( SX1261_BUSY_GPIO_Port, SX1261_BUSY_Pin ) == GPIO_PIN_SET ) &&
                   ( ( int32_t )( timeout_tgt - xTaskGetTickCount( ) ) > 0 ) )
            {
                // Do nothing
            }

            if( HAL_GPIO_ReadPin( SX1261_BUSY_GPIO_Port, SX1261_BUSY_Pin ) == GPIO_PIN_SET )
            {
                return SPI_STATUS__TIMEOUT;
            }
        }

        HAL_GPIO_WritePin( SX1261_CS_GPIO_Port, SX1261_CS_Pin, GPIO_PIN_RESET );
    }
    else
    {
        return SPI_STATUS__WRONG_PARAM;
    }

    HAL_StatusTypeDef ret_spi = HAL_SPI_TransmitReceive( &hspi1, buff_in, buff_out, buff_len, 100 );

    HAL_GPIO_WritePin( SX1302_CS_GPIO_Port, SX1302_CS_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( SX1261_CS_GPIO_Port, SX1261_CS_Pin, GPIO_PIN_SET );

    switch( ret_spi )
    {
    case HAL_OK:
        return SPI_STATUS__OK;

    case HAL_TIMEOUT:
        return SPI_STATUS__TIMEOUT;

    case HAL_ERROR:
    case HAL_BUSY:
        return SPI_STATUS__FAIL;

    default:
        Reset( );
    }
}
/* --- EOF ------------------------------------------------------------------ */
