/*!
 * \file      GLOBAL.c
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

#ifndef __PROJECT_GLOBAL__
#define __PROJECT_GLOBAL__

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* --- DEPENDENCIES --------------------------------------------------------- */
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "device.h"
#include "main.h"

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS, TYPES, ENUMS & STRUCTS ----------------------------- */
#pragma GCC diagnostic error "-Wincompatible-pointer-types"

//===== PROJECT CONFIGURATION ==================================================
// Add here all global project "DEFINE"

//* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS --------------------------------------------------------- */

#ifdef DEBUG
#define Reset( )   \
    {              \
        while( 1 ) \
            ;      \
    }
#else
#define Reset( )             \
    {                        \
        NVIC_SystemReset( ); \
    }
#endif

#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

#ifndef MAX
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#endif

#ifndef UNUSED
#define UNUSED( X ) ( void ) X /* To avoid gcc/g++ warnings */
#endif

#define ARRAY_SIZE( arr ) ( sizeof( arr ) / sizeof( ( arr )[0] ) )

/* -------------------------------------------------------------------------- */
/* --- GLOBAL PUBLIC VARIABLES DECLARATION ---------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DECLARATION ----------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /*__PROJECT_GLOBAL__*/

//===== END OF FILE =====================================================================
