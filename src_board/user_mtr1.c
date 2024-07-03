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

#include "user.h"

#include "hal.h"

// the globals
USER_Params userParams_M1;
#pragma DATA_SECTION(userParams_M1,"user_data");
//*****************************************************************************
//
// USER_setParams, set control parameters for motor 1
//
//*****************************************************************************
void USER_setMotor1Params(userParams_Handle handle)
{
    USER_Params *objUser = (USER_Params *)handle;

    objUser->numIsrTicksPerCtrlTick = 1;
    objUser->numIsrTicksPerEstTick = 1;
    objUser->numIsrTicksPerTrajTick = 1;

    objUser->numCtrlTicksPerCurrentTick = 1;
    objUser->numCtrlTicksPerSpeedTick = USER_M1_NUM_ISR_TICKS_PER_SPEED_TICK;

    objUser->numCurrentSensors = USER_M1_NUM_CURRENT_SENSORS;
    objUser->numVoltageSensors = USER_M1_NUM_VOLTAGE_SENSORS;

#if defined(_FULL_FAST_LIB)
    objUser->motor_type = USER_MOTOR1_TYPE;
#endif // _FULL_FAST_LIB

    objUser->motor_numPolePairs = USER_MOTOR1_NUM_POLE_PAIRS;
    objUser->motor_numEncSlots = USER_MOTOR1_NUM_ENC_SLOTS;

    objUser->dcBus_nominal_V = USER_M1_NOMINAL_DC_BUS_VOLTAGE_V;

    objUser->systemFreq_MHz = USER_SYSTEM_FREQ_MHz;

    objUser->pwmPeriod_usec = USER_M1_PWM_PERIOD_usec;

    objUser->voltage_sf = USER_M1_VOLTAGE_SF;

    objUser->current_sf = USER_M1_CURRENT_SF;

    objUser->dcBusPole_rps = USER_M1_DCBUS_POLE_rps;


    objUser->speedPole_rps = USER_M1_SPEED_POLE_rps;

    objUser->voltageFilterPole_rps = USER_M1_VOLTAGE_FILTER_POLE_rps;

    objUser->maxVsMag_pu = USER_M1_MAX_VS_MAG_PU;

    objUser->motor_ratedFlux_Wb = USER_MOTOR1_RATED_FLUX_VpHz / MATH_TWO_PI;

#if defined(_FULL_FAST_LIB)
    objUser->motor_Rr_Ohm = USER_MOTOR1_Rr_Ohm;
    objUser->motor_Rs_Ohm = USER_MOTOR1_Rs_Ohm;
#else // FAST_LIB
#error Not select a right FAST library
#endif // FAST_LIB

    objUser->motor_Ls_d_H = USER_MOTOR1_Ls_d_H;
    objUser->motor_Ls_q_H = USER_MOTOR1_Ls_q_H;

    objUser->maxCurrent_A = USER_MOTOR1_MAX_CURRENT_A;

    objUser->Vd_sf = USER_M1_VD_SF;
    objUser->maxVsMag_V = USER_MOTOR1_RATED_VOLTAGE_V;

    objUser->angleDelayed_sf_sec = (float32_t)0.5f * USER_M1_CTRL_PERIOD_sec;

    objUser->IdRated_A = USER_MOTOR1_MAGNETIZING_CURRENT_A;
    objUser->fluxExcFreq_Hz = USER_MOTOR1_FLUX_EXC_FREQ_Hz;

    if(objUser->flag_bypassMotorId == true)
    {
        objUser->BWc_rps = MATH_TWO_PI * (float32_t)200.0f;
        objUser->BWdelta = (float32_t)20.0f;

        objUser->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                   objUser->motor_numPolePairs *
                                   objUser->motor_ratedFlux_Wb /
                                   (float32_t) (2.0f * USER_MOTOR1_INERTIA_Kgm2);
    }
    else
    {
        objUser->BWc_rps = MATH_TWO_PI * (float32_t)200.0f;
        objUser->BWdelta = (float32_t)20.0f;
        objUser->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                       objUser->motor_numPolePairs *
                                       (float32_t)(0.001f) /
                                       (float32_t)(2.0f * 0.000001f);
    }

    objUser->estWaitTime[EST_STATE_ERROR] = 0;
    objUser->estWaitTime[EST_STATE_IDLE] = 0;
    objUser->estWaitTime[EST_STATE_ROVERL] = (int_least32_t)(5.0 * USER_M1_ISR_FREQ_Hz);
    objUser->estWaitTime[EST_STATE_RS] = 0;
    objUser->estWaitTime[EST_STATE_RAMPUP] = (int_least32_t)((USER_MOTOR1_FLUX_EXC_FREQ_Hz /
            USER_M1_MAX_ACCEL_Hzps + (float32_t)1.0) * USER_M1_ISR_FREQ_Hz);

    objUser->estWaitTime[EST_STATE_CONSTSPEED] = (int_least32_t)(1.0 * USER_M1_ISR_FREQ_Hz);
    objUser->estWaitTime[EST_STATE_IDRATED] = (int_least32_t)(20.0 * USER_M1_ISR_FREQ_Hz);
    objUser->estWaitTime[EST_STATE_RATEDFLUX_OL] = (int_least32_t)(1.0 * USER_M1_ISR_FREQ_Hz);
    objUser->estWaitTime[EST_STATE_RATEDFLUX] = 0;
    objUser->estWaitTime[EST_STATE_RAMPDOWN] = (int_least32_t)(0.0 * USER_M1_ISR_FREQ_Hz);
    objUser->estWaitTime[EST_STATE_LOCKROTOR] = 0;
    objUser->estWaitTime[EST_STATE_LS] = 0;
    objUser->estWaitTime[EST_STATE_RR] = (int_least32_t)(5.0 * USER_M1_ISR_FREQ_Hz);
    objUser->estWaitTime[EST_STATE_MOTORIDENTIFIED] = 0;
    objUser->estWaitTime[EST_STATE_ONLINE] = 0;

    objUser->RsWaitTime[EST_RS_STATE_ERROR] = 0;
    objUser->RsWaitTime[EST_RS_STATE_IDLE] = 0;
    objUser->RsWaitTime[EST_RS_STATE_RAMPUP] = (int_least32_t)(1.0 * USER_M1_ISR_FREQ_Hz);
    objUser->RsWaitTime[EST_RS_STATE_COARSE] = (int_least32_t)(2.0 * USER_M1_ISR_FREQ_Hz);
    objUser->RsWaitTime[EST_RS_STATE_FINE] = (int_least32_t)(10.0 * USER_M1_ISR_FREQ_Hz);
    objUser->RsWaitTime[EST_RS_STATE_DONE] = 0;

#if defined(_FULL_FAST_LIB)
    objUser->RrWaitTime[EST_RR_STATE_ERROR] = 0;
    objUser->RrWaitTime[EST_RR_STATE_IDLE] = 0;
    objUser->RrWaitTime[EST_RR_STATE_RAMPUP] = (int_least32_t)(1.0 * USER_M1_ISR_FREQ_Hz);
    objUser->RrWaitTime[EST_RR_STATE_COARSE] = (int_least32_t)(10.0 * USER_M1_ISR_FREQ_Hz);
    objUser->RrWaitTime[EST_RR_STATE_FINE] = (int_least32_t)(30.0 * USER_M1_ISR_FREQ_Hz);
    objUser->RrWaitTime[EST_RR_STATE_DONE] = 0;
#endif // _FULL_FAST_LIB

    objUser->FluxWaitTime[EST_FLUX_STATE_ERROR] = 0;
    objUser->FluxWaitTime[EST_FLUX_STATE_IDLE] = 0;
    objUser->FluxWaitTime[EST_FLUX_STATE_CL1] = (int_least32_t)(10.0 * USER_M1_ISR_FREQ_Hz);
    objUser->FluxWaitTime[EST_FLUX_STATE_CL2] = (int_least32_t)(0.2 * USER_M1_ISR_FREQ_Hz);
    objUser->FluxWaitTime[EST_FLUX_STATE_FINE] = (int_least32_t)(20.0 * USER_M1_ISR_FREQ_Hz);
    objUser->FluxWaitTime[EST_FLUX_STATE_DONE] = 0;

    objUser->LsWaitTime[EST_LS_STATE_ERROR] = 0;
    objUser->LsWaitTime[EST_LS_STATE_IDLE] = 0;
    objUser->LsWaitTime[EST_LS_STATE_RAMPUP] = (int_least32_t)(10.0 * USER_M1_ISR_FREQ_Hz);
    objUser->LsWaitTime[EST_LS_STATE_COARSE] = (int_least32_t)(30.0 * USER_M1_ISR_FREQ_Hz);
    objUser->LsWaitTime[EST_LS_STATE_FINE] = (int_least32_t)(30.0 * USER_M1_ISR_FREQ_Hz);
    objUser->LsWaitTime[EST_LS_STATE_DONE] = 0;

    objUser->RoverL_excFreq_Hz = USER_M1_R_OVER_L_EXC_FREQ_Hz;
    objUser->maxCurrent_indEst_A = USER_MOTOR1_IND_EST_CURRENT_A;

    objUser->ctrlFreq_Hz = USER_M1_ISR_FREQ_Hz;
    objUser->estFreq_Hz = USER_M1_ISR_FREQ_Hz;

    objUser->trajFreq_Hz = USER_M1_ISR_FREQ_Hz;
    objUser->ctrlPeriod_sec = USER_M1_CTRL_PERIOD_sec;

    objUser->maxAccel_Hzps = USER_M1_MAX_ACCEL_Hzps;

    objUser->maxCurrent_resEst_A = USER_MOTOR1_RES_EST_CURRENT_A;
    objUser->maxCurrentDelta_A = USER_M1_MAX_CURRENT_DELTA_A;

#if defined(_FULL_FAST_LIB)
    objUser->maxCurrentDelta_pw_A = USER_M1_MAX_CURRENT_DELTA_PW_A;
    objUser->IdRated_delta_A = USER_M1_IDRATED_DELTA_A;
#endif // _FULL_FAST_LIB

    objUser->forceAngleFreq_Hz = USER_M1_FORCE_ANGLE_FREQ_Hz;

#if defined(_FULL_FAST_LIB)
    objUser->forceAngleAccel_Hzps = USER_M1_FORCE_ANGLE_ACCEL_Hzps;
#endif  // _FULL_FAST_LIB

    objUser->indEst_speedMaxFraction = USER_M1_SPEEDMAX_FRACTION_FOR_L_IDENT;

    objUser->Kp_min_VpA = (float32_t)0.001f;
    objUser->Kp_max_VpA = (float32_t)1000.0f;

    objUser->RoverL_Kp_sf = USER_M1_R_OVER_L_KP_SF;
    objUser->RoverL_min_rps = MATH_TWO_PI * (float32_t)5.0f;
    objUser->RoverL_max_rps = MATH_TWO_PI * (float32_t)5000.0f;

    objUser->oneOverDcBus_min_invV = (float32_t)1.0f / (float32_t)400.0f;
    objUser->oneOverDcBus_max_invV = (float32_t)1.0f / (float32_t)10.0f;

    objUser->Rs_Ohm = (float32_t)0.0;
    objUser->Rs_coarseDelta_Ohm = (float32_t)0.01f;
    objUser->Rs_fineDelta_Ohm = (float32_t)0.00001f;
    objUser->Rs_min_Ohm = (float32_t)0.001f;
    objUser->Rs_max_Ohm = (float32_t)1000.0f;

#if defined(_FULL_FAST_LIB)
    objUser->IdRatedFraction_indEst = USER_M1_IDRATED_FRACTION_FOR_L_IDENT;
    objUser->pwGain = USER_M1_PW_GAIN;

    objUser->Rr_Ohm = (float32_t)0.0;
    objUser->Rr_coarseDelta_Ohm = (float32_t)0.0001f;
    objUser->Rr_fineDelta_Ohm = (float32_t)0.00001f;
    objUser->Rr_min_Ohm = (float32_t)0.0f;
    objUser->Rr_max_Ohm = (float32_t)1000.0f;
#endif // _FULL_FAST_LIB

    objUser->Ls_d_H = (float32_t)1.0e-6;
    objUser->Ls_q_H = (float32_t)1.0e-6;
    objUser->Ls_coarseDelta_H = (float32_t)0.0000001f;
    objUser->Ls_fineDelta_H = (float32_t)0.00000001f;
    objUser->Ls_min_H = (float32_t)0.000001f;
    objUser->Ls_max_H = (float32_t)100.0f;

    objUser->RsOnLine_DeltaInc_Ohm = (float32_t)0.00002f;
    objUser->RsOnLine_DeltaDec_Ohm = (float32_t)0.00002f;
    objUser->RsOnLine_min_Ohm = (float32_t)0.001f;
    objUser->RsOnLine_max_Ohm = (float32_t)1000.0f;

    objUser->RsOnLine_angleDelta_rad = (float32_t)0.0005f;          // unit=rad
    objUser->RsOnLine_pole_rps = MATH_TWO_PI * (float32_t)0.1f;     // 0.1Hz


    objUser->freqNearZeroSpeedLimit_Hz = USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz;

    objUser->maxFrequency_Hz = USER_MOTOR1_FREQ_MAX_HZ;

    return;
} // end of USER_setParams() function
