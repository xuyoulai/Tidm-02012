//#############################################################################
// $Copyright:
// Copyright (C) 2017-2024 Texas Instruments Incorporated - http://www.ti.com/
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

#ifndef SYS_SETTINGS_H
#define SYS_SETTINGS_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup SYS SETTINGS
//! @{
//
//*****************************************************************************

// modules
#include "libraries/math/include/math.h"
#include <math.h>

//*****************************************************************************
//defines
//*****************************************************************************

//=============================================================================
// Incremental Build options for System check-out
//=============================================================================
#define DMC_LEVEL_1     1   // 50% duty, offset calibration and verify phase shift
#define DMC_LEVEL_2     2   // Open loop control to check sensing signals
#define DMC_LEVEL_3     3   // Closed current loop to check the hardware settings
#define DMC_LEVEL_4     4   // Parameters identification and run with InstaSPIN-FOC

#define DMC_BUILDLEVEL  DMC_LEVEL_4

#define MOTOR_IDENT     0   // 0 = Disables identification, 1 = Enables

// Select set of data to log via FSI
//#define LOG_ANGLE_CURRENT_VOLTAGE
#define LOG_CURRENT
//#define LOG_CURRENT_DQ
//#define LOG_IQ_VDQ
//#define LOG_IQ_UQ_UIQ
//#define LOG_IQ_UQ_VDC
//#define LOG_IU_IV_SECTOR

// Add more definition if needed


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of SYS_SETTINGS_H definition
