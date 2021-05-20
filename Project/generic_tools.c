/*!
 * \file      generic_tools.c
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
#include "generic_tools.h"
#include "global.h"

/* -------------------------------------------------------------------------- */
/* --- CONSTANTS, TYPES, ENUMS & STRUCTS ------------------------------------ */

/* -------------------------------------------------------------------------- */
/* --- MACROS --------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- VARIABLES ------------------------------------------------------------ */

/* -------------------------------------------------------------------------- */
/* ---  FUNCTIONS DECLARATION ----------------------------------------------- */

/**
 * @brief Read uint32 from a uint8_t buffer
 *
 * @param buffer data input
 * @return uint32_t value
 */
uint32_t read_uint32_msb_from_buffer( const uint8_t* buffer )
{
    uint32_t val = ( buffer[0] << 24 ) + ( buffer[1] << 16 ) + ( buffer[2] << 8 ) + ( buffer[3] );

    return val;
}

/**
 * @brief Read uint16 from a uint8_t buffer
 *
 * @param buffer data input
 * @return uint16_t value
 */
uint16_t read_uint16_msb_from_buffer( const uint8_t* buffer )
{
    uint16_t val = ( buffer[0] << 8 ) + ( buffer[1] );

    return val;
}

/**
 * @brief Write uint32 in a uint8_t buffer
 *
 * @param data_to_write uint32 value to write
 * @param buffer        output buffer
 */
void write_uint32_msb_to_buffer( uint32_t val, uint8_t* buffer )
{
    buffer[0] = ( uint8_t )( val >> 24 );
    buffer[1] = ( uint8_t )( val >> 16 );
    buffer[2] = ( uint8_t )( val >> 8 );
    buffer[3] = ( uint8_t )( val );
}

/**
 * @brief Write uint16 in a uint8_t buffer
 *
 * @param data_to_write uint16 value to write
 * @param buffer        output buffer
 */
void write_uint16_msb_to_buffer( uint16_t data_to_write, uint8_t* buffer )
{
    buffer[0] = ( uint8_t )( data_to_write >> 8 );
    buffer[1] = ( uint8_t )( data_to_write );
}

/**
 * @brief Write int16 in a uint8_t buffer
 *
 * @param data_to_write int16 value to write
 * @param buffer        output buffer
 */
void write_int16_msb_to_buffer( int16_t data_to_write, uint8_t* buffer )
{
    buffer[0] = ( uint8_t )( data_to_write >> 8 );
    buffer[1] = ( uint8_t )( data_to_write );
}

/**
 * @brief Compute A2 complement of the value
 *
 * @param val value to compute
 * @param nb_bit position of the sign bit
 * @return int32_t complement value
 */
int32_t tools_complement_a2( uint32_t val, uint8_t nb_bit )
{
    int32_t ret = ( int32_t ) val;

    if( val >= ( uint32_t )( 2 << ( nb_bit - 2 ) ) )
    {
        ret -= 2 << ( nb_bit - 1 );
    }
    return ret;
}

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* --- EOF ------------------------------------------------------------------ */
