/*!
 * \file      temperature_task.c
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
#include "temperature_task.h"

#include "global.h"
#include "FreeRTOS.h"
#include "task.h"

#include "stts751.h"
#include "custom_mems_conf.h"

/* -------------------------------------------------------------------------- */
/* --- CONSTANTS, TYPES, ENUMS & STRUCTS ------------------------------------ */
#define TEMPERATURE_GET_PERIOD_MS ( 30000 )
#define ST_TS751_ADDR1 ( 0x72 )
#define ST_TS751_ADDR2 ( 0x76 )

/* -------------------------------------------------------------------------- */
/* --- MACROS --------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- VARIABLES ------------------------------------------------------------ */
static STTS751_Object_t stts751;
static float            temperature        = 0.0;
static bool             temperature_status = false;

/* -------------------------------------------------------------------------- */
/* ---  FUNCTIONS DECLARATION ----------------------------------------------- */

/**
 * @brief Temperature task, manage init and periodic read of ST TS751 temperature sensor
 *
 * @param argument not use
 */
void temperature_task__start( void* argument )
{
    UNUSED( argument );  // Task withtout argument - avoid warning

    uint8_t count_fail = 0;

    STTS751_IO_t cfg_stts751 = {
        .Init     = CUSTOM_STTS751_0_I2C_Init,
        .DeInit   = CUSTOM_STTS751_0_I2C_DeInit,
        .BusType  = 0,  // I2C
        .Address  = ST_TS751_ADDR1,
        .WriteReg = CUSTOM_STTS751_0_I2C_WriteReg,
        .ReadReg  = CUSTOM_STTS751_0_I2C_ReadReg,
        .GetTick  = NULL,
    };

    temperature_status = false;
    for( ;; )
    {
        if( temperature_status == false )
        {
            //  Try the 2 temperature sensor reference
            if( cfg_stts751.Address == ST_TS751_ADDR2 )
            {
                cfg_stts751.Address = ST_TS751_ADDR1;
            }
            else
            {
                cfg_stts751.Address = ST_TS751_ADDR2;
            }

            if( STTS751_RegisterBusIO( &stts751, &cfg_stts751 ) != STTS751_OK )
            {
                Error_Handler( );
            }
            bool    status = true;
            uint8_t id     = 0;

            if( ( status == true ) && ( STTS751_Init( &stts751 ) != STTS751_OK ) )
            {
                status = false;
            }

            if( ( status == true ) && ( STTS751_ReadID( &stts751, &id ) != STTS751_OK ) )
            {
                status = false;
            }

            if( ( status == true ) && ( STTS751_TEMP_Enable( &stts751 ) != STTS751_OK ) )
            {
                status = false;
            }

            if( ( status == true ) && ( STTS751_TEMP_GetTemperature( &stts751, &temperature ) != STTS751_OK ) )
            {
                status = false;
            }

            temperature_status = status;

            if( status == false )
            {
                count_fail += 1;

                if( count_fail > 10 )
                {
                    // Infinite wait ! temperature sensor is HS
                    vTaskDelay( portMAX_DELAY );
                }
                else
                {
                    vTaskDelay( pdMS_TO_TICKS( 100 ) );
                }
            }
            else
            {
                // succed to read temperature
                count_fail = 0;
                vTaskDelay( pdMS_TO_TICKS( TEMPERATURE_GET_PERIOD_MS ) );
            }
        }
        else
        {
            // init is ok, periodic read
            if( STTS751_TEMP_GetTemperature( &stts751, &temperature ) != STTS751_OK )
            {
                temperature_status = false;
            }

            vTaskDelay( pdMS_TO_TICKS( TEMPERATURE_GET_PERIOD_MS ) );
        }
    }
}

/**
 * @brief return temperature value
 *
 * @param value temperature in celcius
 * @return true valid temperature in param value
 * @return false not valid temperature available
 */
bool temperature_task__get_temperature_celsius( float* value )
{
    if( temperature_status == true )
    {
        *value = temperature;
    }

    return temperature_status;
}
/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* --- EOF ------------------------------------------------------------------ */
