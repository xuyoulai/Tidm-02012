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

//
// include the related header files
//
#include "sys_settings.h"
#include "sys_main.h"
#include "motor1_drive.h"

#pragma CODE_SECTION(motor1CtrlISR, ".TI.ramfunc");

// the globals

//!< the hardware abstraction layer object to motor control
volatile MOTOR_Handle motorHandle_M1;
#pragma DATA_SECTION(motorHandle_M1,"foc_data");

volatile MOTOR_Vars_t motorVars_M1;
#pragma DATA_SECTION(motorVars_M1, "foc_data");

MOTOR_SetVars_t motorSetVars_M1;
#pragma DATA_SECTION(motorSetVars_M1, "foc_data");

HAL_MTR_Obj    halMtr_M1;
#pragma DATA_SECTION(halMtr_M1, "foc_data");

//!< the voltage Clarke transform object
CLARKE_Obj    clarke_V_M1;
#pragma DATA_SECTION(clarke_V_M1, "foc_data");

//!< the current Clarke transform object
CLARKE_Obj    clarke_I_M1;
#pragma DATA_SECTION(clarke_I_M1, "foc_data");

//!< the inverse Park transform object
IPARK_Obj     ipark_V_M1;
#pragma DATA_SECTION(ipark_V_M1, "foc_data");

//!< the Park transform object
PARK_Obj      park_I_M1;
#pragma DATA_SECTION(park_I_M1, "foc_data");

//!< the Park transform object
PARK_Obj      park_V_M1;
#pragma DATA_SECTION(park_V_M1, "foc_data");

//!< the Id PI controller object
PI_Obj        pi_Id_M1;
#pragma DATA_SECTION(pi_Id_M1, "foc_data");

//!< the Iq PI controller object
PI_Obj        pi_Iq_M1;
#pragma DATA_SECTION(pi_Iq_M1, "foc_data");

//!< the speed PI controller object
PI_Obj        pi_spd_M1;
#pragma DATA_SECTION(pi_spd_M1, "foc_data");

//!< the space vector generator object
SVGEN_Obj     svgen_M1;
#pragma DATA_SECTION(svgen_M1, "foc_data");

#if defined(MOTOR1_OVM)
//!< the handle for the space vector generator current
SVGENCURRENT_Obj svgencurrent_M1;
#pragma DATA_SECTION(svgencurrent_M1, "foc_data");
#endif  // MOTOR1_OVM

//!< the speed reference trajectory object
TRAJ_Obj     traj_spd_M1;
#pragma DATA_SECTION(traj_spd_M1, "foc_data");

#if defined(MOTOR1_FWC)
//!< the fwc PI controller object
PI_Obj       pi_fwc_M1;
#pragma DATA_SECTION(pi_fwc_M1, "foc_data");
#endif  // MOTOR1_FWC

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3)
//!< the Angle Generate onject for open loop control
ANGLE_GEN_Obj    angleGen_M1;
#pragma DATA_SECTION(angleGen_M1, "foc_data");
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
//!< the Vs per Freq object for open loop control
VS_FREQ_Obj    VsFreq_M1;
#pragma DATA_SECTION(VsFreq_M1, "foc_data");
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(MOTOR1_MTPA)
//!< the Maximum torque per ampere (MTPA) object
MTPA_Obj     mtpa_M1;
#pragma DATA_SECTION(mtpa_M1, "foc_data");
#endif  // MOTOR1_MTPA

#if defined(MOTOR1_DCLINKSS)
//!< the single-shunt current reconstruction object
DCLINK_SS_Obj    dclink_M1;
#pragma DATA_SECTION(dclink_M1, "foc_data");
#endif // MOTOR1_DCLINKSS

#if defined(MOTOR1_VIBCOMPA) || defined(MOTOR1_VIBCOMPT)
//!< the vibration compensation object
VIB_COMP_Obj    vibComp_M1;
#pragma DATA_SECTION(vibComp_M1, "vibc_data");
#endif  // MOTOR1_VIBCOMPA || MOTOR1_VIBCOMPT

// the control handles for motor 1
void initMotor1Handles(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    obj->motorNum = MTR_1;

    // initialize the driver
    obj->halMtrHandle = HAL_MTR1_init(&halMtr_M1, sizeof(halMtr_M1));

    obj->motorSetsHandle = &motorSetVars_M1;
    obj->userParamsHandle = &userParams_M1;

    return;
}

// initialize control parameters for motor 1
void initMotor1CtrlParameters(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

    // initialize the user parameters
    USER_setParams_priv(obj->userParamsHandle);

    // initialize the user parameters
    USER_setMotor1Params(obj->userParamsHandle);

    // set the driver parameters
    HAL_MTR_setParams(obj->halMtrHandle, obj->userParamsHandle);

    objSets->Kp_spd = 0.05f;
    objSets->Ki_spd = 0.005f;

    objSets->Kp_fwc = USER_M1_FWC_KP;
    objSets->Ki_fwc = USER_M1_FWC_KI;

    objSets->angleFWCMax_rad = USER_M1_FWC_MAX_ANGLE_RAD;
    objSets->overModulation = USER_M1_MAX_VS_MAG_PU;

    objSets->RsOnLineCurrent_A = 0.1f * USER_MOTOR1_MAX_CURRENT_A;

    objSets->lostPhaseSet_A = USER_M1_LOST_PHASE_CURRENT_A;
    objSets->unbalanceRatioSet = USER_M1_UNBALANCE_RATIO;
    objSets->overLoadSet_W = USER_M1_OVER_LOAD_POWER_W;

    objSets->toqueFailMinSet_Nm = USER_M1_TORQUE_FAILED_SET;
    objSets->speedFailMaxSet_Hz = USER_M1_FAIL_SPEED_MAX_HZ;
    objSets->speedFailMinSet_Hz = USER_M1_FAIL_SPEED_MIN_HZ;

    objSets->stallCurrentSet_A = USER_M1_STALL_CURRENT_A;
    objSets->IsFailedCheckSet_A = USER_M1_FAULT_CHECK_CURRENT_A;

    objSets->maxPeakCurrent_A = USER_M1_ADC_FULL_SCALE_CURRENT_A * 0.45f;
    objSets->overCurrent_A = USER_MOTOR1_OVER_CURRENT_A;
    objSets->currentInv_sf = USER_M1_CURRENT_INV_SF;

    objSets->overVoltageFault_V = USER_M1_OVER_VOLTAGE_FAULT_V;
    objSets->overVoltageNorm_V = USER_M1_OVER_VOLTAGE_NORM_V;
    objSets->underVoltageFault_V = USER_M1_UNDER_VOLTAGE_FAULT_V;
    objSets->underVoltageNorm_V = USER_M1_UNDER_VOLTAGE_NORM_V;

    objSets->overCurrentTimesSet = USER_M1_OVER_CURRENT_TIMES_SET;
    objSets->voltageFaultTimeSet = USER_M1_VOLTAGE_FAULT_TIME_SET;
    objSets->motorStallTimeSet = USER_M1_STALL_TIME_SET;
    objSets->startupFailTimeSet = USER_M1_STARTUP_FAIL_TIME_SET;

    objSets->overSpeedTimeSet = USER_M1_OVER_SPEED_TIME_SET;
    objSets->overLoadTimeSet = USER_M1_OVER_LOAD_TIME_SET;
    objSets->unbalanceTimeSet = USER_M1_UNBALANCE_TIME_SET;
    objSets->lostPhaseTimeSet = USER_M1_LOST_PHASE_TIME_SET;

    objSets->stopWaitTimeSet = USER_M1_STOP_WAIT_TIME_SET;
    objSets->restartWaitTimeSet = USER_M1_RESTART_WAIT_TIME_SET;
    objSets->restartTimesSet = USER_M1_START_TIMES_SET;

    objSets->dacCMPValH = 2048U + 1024U;    // set default positive peak value
    objSets->dacCMPValL = 2048U - 1024U;    // set default negative peak value

    obj->adcData.voltage_sf = objUser->voltage_sf;
    obj->adcData.dcBusvoltage_sf = objUser->voltage_sf;

#if defined(MOTOR1_DCLINKSS)
    obj->adcData.current_sf = -objUser->current_sf;
#else   // !(MOTOR1_DCLINKSS)
    obj->adcData.current_sf = objUser->current_sf;
#endif  // MOTOR1_DCLINKSS
    obj->speedStart_Hz = USER_MOTOR1_SPEED_START_Hz;
    obj->speedForce_Hz = USER_MOTOR1_SPEED_FORCE_Hz;
    obj->speedFlyingStart_Hz = USER_MOTOR1_SPEED_FS_Hz;

    obj->accelerationMax_Hzps = USER_MOTOR1_ACCEL_MAX_Hzps;
    obj->accelerationStart_Hzps = USER_MOTOR1_ACCEL_START_Hzps;

    obj->VsRef_pu = 0.98f * USER_M1_MAX_VS_MAG_PU;
    obj->VsRef_V =
            0.98f * USER_M1_MAX_VS_MAG_PU * USER_M1_NOMINAL_DC_BUS_VOLTAGE_V;

    obj->IsSet_A = USER_MOTOR1_TORQUE_CURRENT_A;

    obj->fluxCurrent_A = USER_MOTOR1_FLUX_CURRENT_A;
    obj->alignCurrent_A = USER_MOTOR1_ALIGN_CURRENT_A;
    obj->startCurrent_A = USER_MOTOR1_STARTUP_CURRENT_A;
    obj->maxCurrent_A = USER_MOTOR1_MAX_CURRENT_A;

    obj->angleDelayed_sf = 0.5f * MATH_TWO_PI * USER_M1_CTRL_PERIOD_sec;

    obj->power_sf = MATH_TWO_PI / USER_MOTOR1_NUM_POLE_PAIRS;
    obj->VIrmsIsrScale = objUser->ctrlFreq_Hz;

    obj->stopWaitTimeCnt = 0;
    obj->flagEnableRestart = false;

    obj->faultMtrMask.all = MTR1_FAULT_MASK_SET;

    obj->flyingStartTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.5f); // 0.5s
    obj->flyingStartMode = FLYINGSTART_MODE_HALT;

    if(objUser->flag_bypassMotorId == true)
    {
#if defined(MOTOR1_DCLINKSS)
        obj->svmMode = SVM_COM_C;
#else  // !(MOTOR1_DCLINKSS)
        obj->svmMode = SVM_MIN_C;
#endif  // !(MOTOR1_DCLINKSS)
        obj->flagEnableFWC = true;
    }
    else
    {
        obj->svmMode = SVM_COM_C;
        obj->flagEnableFWC = false;
    }

    obj->flagEnableForceAngle = true;
    obj->flagEnableFlyingStart = false;
    obj->flagEnableIPD = false;

    obj->flagEnableSpeedCtrl = true;
    obj->flagEnableCurrentCtrl = true;

    obj->IsSet_A = 0.0f;

    obj->estState = EST_STATE_IDLE;
    obj->trajState = EST_TRAJ_STATE_IDLE;

    obj->flagEnableAlignment = true;
    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);      // 0.1s
    obj->forceRunTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);   // 1.0s

    obj->fwcTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);        // 2.0s

#if defined(MOTOR1_FWC)
    obj->piHandle_fwc = PI_init(&pi_fwc_M1, sizeof(pi_fwc_M1));

    // set the FWC controller
    PI_setGains(obj->piHandle_fwc, USER_M1_FWC_KP, USER_M1_FWC_KI);
    PI_setUi(obj->piHandle_fwc, 0.0);
    PI_setMinMax(obj->piHandle_fwc, USER_M1_FWC_MAX_ANGLE_RAD,
                 USER_M1_FWC_MIN_ANGLE_RAD);
#endif  // MOTOR1_FWC

#ifdef MOTOR1_MTPA
    // initialize the Maximum torque per ampere (MTPA)
    obj->mtpaHandle = MTPA_init(&mtpa_M1, sizeof(mtpa_M1));

    // compute the motor constant for MTPA
    MTPA_computeParameters(obj->mtpaHandle,
                           objUser->motor_Ls_d_H,
                           objUser->motor_Ls_q_H,
                           objUser->motor_ratedFlux_Wb);
#endif  // MOTOR1_MTPA

#if defined(MOTOR1_VIBCOMPA)
    obj->vibCompAlpha = USER_MOTOR1_VIBCOMPA_ALPHA;
    obj->vibCompGain  = USER_MOTOR1_VIBCOMPA_GAIN;
    obj->vibCompIndexDelta = USER_MOTOR1_VIBCOMPA_INDEX_DELTA;

    obj->vibCompFlagEnable = false;
    obj->vibCompFlagReset = true;

    // Initialize the handle for vibration compensation
    obj->vibCompHandle = VIB_COMP_init(&vibComp_M1, sizeof(vibComp_M1));

    VIB_COMPA_setParams(obj->vibCompHandle, obj->vibCompAlpha, obj->vibCompGain,
                        obj->vibCompIndexDelta, USER_MOTOR1_NUM_POLE_PAIRS);

    VIB_COMP_reset(obj->vibCompHandle);
#elif defined(MOTOR1_VIBCOMPT)
    obj->vibCompAngle_rad = 0.0f;
    obj->vibCompAlpha0 = 0.0f;
    obj->vibCompAlpha120 = 0.0f;
    obj->vibCompAlpha240 = 0.0f;

    obj->vibCompHandle = VIB_COMP_init(&vibComp_M1, sizeof(vibComp_M1));

    VIB_COMPT_setParams(obj->vibCompHandle, USER_MOTOR1_NUM_POLE_PAIRS);

    VIB_COMP_reset(obj->vibCompHandle);
#endif  // MOTOR1_VIBCOMPA || MOTOR1_VIBCOMPT

#if defined(MOTOR1_DCLINKSS)
    obj->dclinkHandle = DCLINK_SS_init(&dclink_M1, sizeof(dclink_M1));
    DCLINK_SS_setInitialConditions(obj->dclinkHandle,
                                   HAL_getTimeBasePeriod(obj->halMtrHandle), 0.5f);

    //disable full sampling
    DCLINK_SS_setFlag_enableFullSampling(obj->dclinkHandle, false);

    //enable sequence control
    DCLINK_SS_setFlag_enableSequenceControl(obj->dclinkHandle, true);

    // configure timing
    DCLINK_SS_setMinAVDuration(obj->dclinkHandle, USER_M1_DCLINKSS_MIN_DURATION);
    DCLINK_SS_setSampleDelay(obj->dclinkHandle, USER_M1_DCLINKSS_SAMPLE_DELAY);
#endif  // MOTOR1_DCLINKSS

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3)
    // initialize the angle generate module
    obj->angleGenHandle = ANGLE_GEN_init(&angleGen_M1, sizeof(angleGen_M1));

    ANGLE_GEN_setParams(obj->angleGenHandle, objUser->ctrlPeriod_sec);
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->Idq_set_A.value[0] = 0.0f;
    obj->Idq_set_A.value[1] = obj->startCurrent_A;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    // initialize the Vs per Freq module
    obj->VsFreqHandle = VS_FREQ_init(&VsFreq_M1, sizeof(VsFreq_M1));

    VS_FREQ_setVsMagPu(obj->VsFreqHandle, objUser->maxVsMag_pu);

    VS_FREQ_setMaxFreq(obj->VsFreqHandle, USER_MOTOR1_FREQ_MAX_HZ);

    VS_FREQ_setProfile(obj->VsFreqHandle,
                       USER_MOTOR1_FREQ_LOW_HZ, USER_MOTOR1_FREQ_HIGH_HZ,
                       USER_MOTOR1_VOLT_MIN_V, USER_MOTOR1_VOLT_MAX_V);
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

    // initialize the Clarke modules
    obj->clarkeHandle_V = CLARKE_init(&clarke_V_M1, sizeof(clarke_V_M1));

    // set the Clarke parameters
    setupClarke_V(obj->clarkeHandle_V, objUser->numVoltageSensors);

    // initialize the Clarke modules
    obj->clarkeHandle_I = CLARKE_init(&clarke_I_M1, sizeof(clarke_I_M1));

    // set the Clarke parameters
    setupClarke_I(obj->clarkeHandle_I, objUser->numCurrentSensors);

    // initialize the inverse Park module
    obj->iparkHandle_V = IPARK_init(&ipark_V_M1, sizeof(ipark_V_M1));

    // initialize the Park module
    obj->parkHandle_I = PARK_init(&park_I_M1, sizeof(park_I_M1));

    // initialize the Park module
    obj->parkHandle_V = PARK_init(&park_V_M1, sizeof(park_V_M1));

    // initialize the PI controllers
    obj->piHandle_Id  = PI_init(&pi_Id_M1, sizeof(pi_Id_M1));
    obj->piHandle_Iq  = PI_init(&pi_Iq_M1, sizeof(pi_Iq_M1));
    obj->piHandle_spd = PI_init(&pi_spd_M1, sizeof(pi_spd_M1));

    // initialize the speed reference trajectory
    obj->trajHandle_spd = TRAJ_init(&traj_spd_M1, sizeof(traj_spd_M1));

    // configure the speed reference trajectory (Hz)
    TRAJ_setTargetValue(obj->trajHandle_spd, 0.0f);
    TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
    TRAJ_setMinValue(obj->trajHandle_spd, -objUser->maxFrequency_Hz);
    TRAJ_setMaxValue(obj->trajHandle_spd, objUser->maxFrequency_Hz);
    TRAJ_setMaxDelta(obj->trajHandle_spd, (objUser->maxAccel_Hzps * objUser->ctrlPeriod_sec));

    // initialize the space vector generator module
    obj->svgenHandle = SVGEN_init(&svgen_M1, sizeof(svgen_M1));

    SVGEN_setMode(obj->svgenHandle, SVM_COM_C);

    HAL_setTriggerPrams(&obj->pwmData, USER_SYSTEM_FREQ_MHz, 0.2f, 0.1f);

#if defined(MOTOR1_OVM)
    // Initialize and setup the 100% SVM generator
    obj->svgenCurrentHandle =
            SVGENCURRENT_init(&svgencurrent_M1, sizeof(svgencurrent_M1));

    SVGENCURRENT_setup(obj->svgenCurrentHandle, 1.0f,
                       USER_M1_PWM_FREQ_kHz, USER_SYSTEM_FREQ_MHz);
#endif  // MOTOR1_OVM

    HAL_enableGate(obj->halMtrHandle);

    // initialize the estimator
    obj->estHandle = EST_initEst(MTR_1);

    // set the default estimator parameters
    EST_setParams(obj->estHandle, obj->userParamsHandle);
    EST_setFlag_enableForceAngle(obj->estHandle, obj->flagEnableForceAngle);
    EST_setFlag_enableRsRecalc(obj->estHandle, obj->flagEnableRsRecalc);

    // set the scale factor for high frequency motor
    EST_setOneOverFluxGain_sf(obj->estHandle,
                              obj->userParamsHandle, USER_M1_EST_FLUX_HF_SF);
    EST_setFreqLFP_sf(obj->estHandle,
                      obj->userParamsHandle, USER_M1_EST_FREQ_HF_SF);
    EST_setBemf_sf(obj->estHandle,
                   obj->userParamsHandle, USER_M1_EST_BEMF_HF_SF);

    objSets->Ls_d_comp_H = EST_getLs_d_H(obj->estHandle);
    objSets->Ls_q_comp_H = EST_getLs_q_H(obj->estHandle);

    objSets->Ls_d_Icomp_coef = USER_MOTOR1_Ls_d_COMP_COEF / obj->maxCurrent_A;
    objSets->Ls_q_Icomp_coef = USER_MOTOR1_Ls_q_COMP_COEF / obj->maxCurrent_A;

    objSets->Ls_min_H = objSets->Ls_d_comp_H * USER_MOTOR1_Ls_MIN_NUM_COEF;

    obj->flagEnableLsUpdate = false;

    // for Rs re-calculation
    obj->flagEnableRsRecalc = false;

    // for Rs online calibration
    obj->flagRsOnLineContinue = false;
    obj->flagStartRsOnLine = false;

    objSets->RsOnlineWaitTimeSet = USER_MOTOR1_RSONLINE_WAIT_TIME;
    objSets->RsOnlineWorkTimeSet = USER_MOTOR1_RSONLINE_WORK_TIME;

    // setup the controllers, speed, d/q-axis current pid regulator
    setupControllers(handle);

    // disable the PWM
    HAL_disablePWM(obj->halMtrHandle);

    return;
}   // end of initMotor1CtrlParameters() function

void runMotor1OffsetsCalculation(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    // Calculate motor protection value
    calcMotorOverCurrentThreshold(handle);

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

#if defined(MOTOR1_DCLINKSS)
    HAL_MTR_Obj *objHal = (HAL_MTR_Obj *)(obj->halMtrHandle);

    EPWM_setCounterCompareValue(objHal->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C, 5);
    EPWM_setCounterCompareValue(objHal->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D, 5);

    EPWM_setCounterCompareValue(objHal->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C, 5);
    EPWM_setCounterCompareValue(objHal->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D, 5);

    // Offsets in phase current sensing
    ADC_setPPBReferenceOffset(MTR1_IDC1_SOC_ADC_BASE, MTR1_IDC1_PPB,
                              MTR1_IDC1_PPB);

    ADC_setPPBReferenceOffset(MTR1_IDC2_SOC_ADC_BASE, MTR1_IDC2_PPB,
                              MTR1_IDC2_PPB);

    ADC_setPPBReferenceOffset(MTR1_IDC3_SOC_ADC_BASE, MTR1_IDC3_PPB,
                              MTR1_IDC3_PPB);

    ADC_setPPBReferenceOffset(MTR1_IDC4_SOC_ADC_BASE, MTR1_IDC4_PPB,
                              MTR1_IDC4_PPB);

    obj->adcData.offset_Idc_ad = USER_M1_IDC_OFFSET_AD;
#else // MOTOR1_DCLINKSS
    ADC_setPPBReferenceOffset(MTR1_IU_SOC_ADC_BASE, MTR1_IU_PPB,
                              USER_M1_IA_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IV_SOC_ADC_BASE, MTR1_IV_PPB,
                              USER_M1_IB_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IW_SOC_ADC_BASE, MTR1_IW_PPB,
                              USER_M1_IC_OFFSET_AD);

    obj->adcData.offset_I_ad.value[0]  = USER_M1_IA_OFFSET_AD;
    obj->adcData.offset_I_ad.value[1]  = USER_M1_IB_OFFSET_AD;
    obj->adcData.offset_I_ad.value[2]  = USER_M1_IC_OFFSET_AD;
#endif // MOTOR1_DCLINKSS

    // Offsets in phase voltage sensing
    obj->adcData.offset_V_sf.value[0]  = USER_M1_VA_OFFSET_SF;
    obj->adcData.offset_V_sf.value[1]  = USER_M1_VB_OFFSET_SF;
    obj->adcData.offset_V_sf.value[2]  = USER_M1_VC_OFFSET_SF;

    if(obj->flagEnableOffsetCalc == true)
    {
        float32_t offsetK1 = 0.998001f;  // Offset filter coefficient K1: 0.05/(T+0.05);
        float32_t offsetK2 = 0.001999f;  // Offset filter coefficient K2: T/(T+0.05);
        float32_t invCurrentSf = 1.0f / obj->adcData.current_sf;

        float32_t invVdcbus;

        uint16_t offsetCnt;

        DEVICE_DELAY_US(2.0f);      // delay 2us

#if defined(MOTOR1_DCLINKSS)
        HAL_setOffsetTrigger(obj->halMtrHandle);

        ADC_setPPBReferenceOffset(MTR1_IDC1_SOC_ADC_BASE, MTR1_IDC1_PPB, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC2_SOC_ADC_BASE, MTR1_IDC2_PPB, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC3_SOC_ADC_BASE, MTR1_IDC3_PPB, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC4_SOC_ADC_BASE, MTR1_IDC4_PPB, 0);

        obj->adcData.offset_Idc_ad  = USER_M1_IDC_OFFSET_AD * USER_M1_CURRENT_SF;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

        // enable the PWM
        HAL_enablePWM(obj->halMtrHandle);
#else  // !MOTOR1_DCLINKSS
        ADC_setPPBReferenceOffset(MTR1_IU_SOC_ADC_BASE, MTR1_IU_PPB, 0);
        ADC_setPPBReferenceOffset(MTR1_IV_SOC_ADC_BASE, MTR1_IV_PPB, 0);
        ADC_setPPBReferenceOffset(MTR1_IW_SOC_ADC_BASE, MTR1_IW_PPB, 0);

        obj->adcData.offset_I_ad.value[0] =
                 obj->adcData.offset_I_ad.value[0] * obj->adcData.current_sf;
        obj->adcData.offset_I_ad.value[1] =
                 obj->adcData.offset_I_ad.value[1] * obj->adcData.current_sf;
        obj->adcData.offset_I_ad.value[2] =
                 obj->adcData.offset_I_ad.value[2] * obj->adcData.current_sf;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

        // enable the PWM
        HAL_enablePWM(obj->halMtrHandle);
#endif // !MOTOR1_DCLINKSS

        for(offsetCnt = 0; offsetCnt < 32000; offsetCnt++)
        {
            // clear the ADC interrupt flag
            ADC_clearInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

            while(ADC_getInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM) == false);

            HAL_readMtr1ADCData(&obj->adcData);

            if(offsetCnt >= 2000)       // Ignore the first 2000 times
            {
                // Offsets in phase current sensing
#if defined(MOTOR1_DCLINKSS)
                obj->adcData.offset_Idc_ad = offsetK1 * obj->adcData.offset_Idc_ad +
                               0.25f * offsetK2 *(obj->adcData.Idc1_A.value[0] +
                                                  obj->adcData.Idc1_A.value[1] +
                                                  obj->adcData.Idc2_A.value[0] +
                                                  obj->adcData.Idc2_A.value[1]);
#else // !MOTOR1_DCLINKSS
                obj->adcData.offset_I_ad.value[0] =
                        offsetK1 * obj->adcData.offset_I_ad.value[0] +
                        obj->adcData.I_A.value[0] * offsetK2;

                obj->adcData.offset_I_ad.value[1] =
                        offsetK1 * obj->adcData.offset_I_ad.value[1] +
                        obj->adcData.I_A.value[1] * offsetK2;

                obj->adcData.offset_I_ad.value[2] =
                        offsetK1 * obj->adcData.offset_I_ad.value[2] +
                        obj->adcData.I_A.value[2] * offsetK2;
#endif // !MOTOR1_DCLINKSS

                invVdcbus = 1.0f / obj->adcData.VdcBus_V;

                // Offsets in phase voltage sensing
                obj->adcData.offset_V_sf.value[0] =
                         offsetK1 * obj->adcData.offset_V_sf.value[0] +
                         (invVdcbus * obj->adcData.V_V.value[0]) * offsetK2;

                obj->adcData.offset_V_sf.value[1] =
                         offsetK1 * obj->adcData.offset_V_sf.value[1] +
                         (invVdcbus * obj->adcData.V_V.value[1]) * offsetK2;

                obj->adcData.offset_V_sf.value[2] =
                         offsetK1 * obj->adcData.offset_V_sf.value[2] +
                         (invVdcbus * obj->adcData.V_V.value[2]) * offsetK2;
            }
            else if(offsetCnt <= 1000)
            {
                // enable the PWM
                HAL_enablePWM(obj->halMtrHandle);
            }
        } // for()

        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);

#if defined(MOTOR1_DCLINKSS)
        obj->adcData.offset_Idc_ad = obj->adcData.offset_Idc_ad * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IDC1_SOC_ADC_BASE, MTR1_IDC1_PPB,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC2_SOC_ADC_BASE, MTR1_IDC2_PPB,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC3_SOC_ADC_BASE, MTR1_IDC3_PPB,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC4_SOC_ADC_BASE, MTR1_IDC4_PPB,
                                  (uint16_t)obj->adcData.offset_Idc_ad);
#else // !MOTOR1_DCLINKSS
        obj->adcData.offset_I_ad.value[0] =
                 obj->adcData.offset_I_ad.value[0] * invCurrentSf;
        obj->adcData.offset_I_ad.value[1] =
                 obj->adcData.offset_I_ad.value[1] * invCurrentSf;
        obj->adcData.offset_I_ad.value[2] =
                 obj->adcData.offset_I_ad.value[2] * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IU_SOC_ADC_BASE, MTR1_IU_PPB,
                                  (uint16_t)obj->adcData.offset_I_ad.value[0]);

        ADC_setPPBReferenceOffset(MTR1_IV_SOC_ADC_BASE, MTR1_IV_PPB,
                                  (uint16_t)obj->adcData.offset_I_ad.value[1]);

        ADC_setPPBReferenceOffset(MTR1_IW_SOC_ADC_BASE, MTR1_IW_PPB,
                                  (uint16_t)obj->adcData.offset_I_ad.value[2]);
#endif // MOTOR1_DCLINKSS
    }   // flagEnableOffsetCalc == true

#if defined(MOTOR1_DCLINKSS)
    // Check current and voltage offset
    if( (obj->adcData.offset_Idc_ad > USER_M1_IDC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_Idc_ad < USER_M1_IDC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }
#else // !MOTOR1_DCLINKSS
    // Check current and voltage offset
    if( (obj->adcData.offset_I_ad.value[0] > USER_M1_IA_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[0] < USER_M1_IA_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->adcData.offset_I_ad.value[1] > USER_M1_IB_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[1] < USER_M1_IB_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->adcData.offset_I_ad.value[2] > USER_M1_IC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[2] < USER_M1_IC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }
#endif // MOTOR1_DCLINKSS

    if( (obj->adcData.offset_V_sf.value[0] > USER_M1_VA_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[0] < USER_M1_VA_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }

    if( (obj->adcData.offset_V_sf.value[1] > USER_M1_VB_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[1] < USER_M1_VB_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }

    if( (obj->adcData.offset_V_sf.value[2] > USER_M1_VC_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[2] < USER_M1_VC_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }

    if((obj->faultMtrNow.bit.voltageOffset == 0) &&
       (obj->faultMtrNow.bit.currentOffset == 0))
    {
        obj->flagEnableOffsetCalc = false;
    }

    return;
} // end of runMotor1OffsetsCalculation() function

void runMotor1Control(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(obj->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(obj->userParamsHandle);

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == true)
    {
        if(HAL_getMtrTripFaults(obj->halMtrHandle) != 0)
        {
            obj->faultMtrNow.bit.moduleOverCurrent = 1;
        }
    }

    obj->faultMtrPrev.all |= obj->faultMtrNow.all;
    obj->faultMtrUse.all = obj->faultMtrNow.all & obj->faultMtrMask.all;

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

    if(obj->flagClearFaults == true)
    {
        HAL_clearMtrFaultStatus(obj->halMtrHandle);

        obj->faultMtrNow.all &= MTR_FAULT_CLEAR;
        obj->flagClearFaults = false;
    }

    if(obj->flagEnableRunAndIdentify == true)
    {
        // Had some faults to stop the motor
        if(obj->faultMtrUse.all != 0)
        {
            if(obj->flagRunIdentAndOnLine == true)
            {
                obj->flagRunIdentAndOnLine = false;
                obj->motorState = MOTOR_FAULT_STOP;

                obj->stopWaitTimeCnt = objSets->restartWaitTimeSet;
                obj->restartTimesCnt++;

                if(obj->flagEnableRestart == false)
                {
                    obj->flagEnableRunAndIdentify = false;
                    obj->stopWaitTimeCnt = 0;
                }
            }
            else if(obj->stopWaitTimeCnt == 0)
            {
                if(obj->restartTimesCnt < objSets->restartTimesSet)
                {
                    obj->flagClearFaults = 1;
                }
                else
                {
                    obj->flagEnableRunAndIdentify = false;
                }
            }
        }
        // Restart
        else if((obj->flagRunIdentAndOnLine == false) &&
                (obj->stopWaitTimeCnt == 0))
        {
            restartMotorControl(handle);
        }
    }
    else if(obj->flagRunIdentAndOnLine == true)
    {
        stopMotorControl(handle);

        if(obj->flagEnableFlyingStart == false)
        {
            obj->stopWaitTimeCnt = objSets->stopWaitTimeSet;
        }
        else
        {
            obj->stopWaitTimeCnt = 0;
        }

#if defined(MOTOR1_VIBCOMPA)
        VIB_COMP_setFlag_enableOutput(obj->vibCompHandle, false);
#endif  // MOTOR1_VIBCOMPA
    }
    else
    {
        // No action
    }

    if(obj->flagRunIdentAndOnLine == true)
    {
        if(HAL_getPwmEnableStatus(obj->halMtrHandle) == false)
        {
            // enable the estimator
            EST_enable(obj->estHandle);

            // enable the trajectory generator
            EST_enableTraj(obj->estHandle);

            // enable the PWM
            HAL_enablePWM(obj->halMtrHandle);
        }

        if(obj->flagMotorIdentified == true)
        {
            if(obj->speedRef_Hz > 0.0f)
            {
                obj->direction = 1.0f;
            }
            else
            {
                obj->direction = -1.0f;
            }

            // enable or disable force angle
            EST_setFlag_enableForceAngle(obj->estHandle,
                                         obj->flagEnableForceAngle);

            EST_setFlag_enableRsRecalc(obj->estHandle,
                                       obj->flagEnableRsRecalc);

            TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);

            if((fabsf(obj->speed_Hz) > obj->speedStart_Hz) ||
               (obj->motorState == MOTOR_CTRL_RUN))
            {
                TRAJ_setMaxDelta(obj->trajHandle_spd,
                    (obj->accelerationMax_Hzps * objUser->ctrlPeriod_sec));

                if(obj->flagEnableLsUpdate ==  true)
                {
                    // Calculate the Ld and Lq which reduce with current
                    objSets->Ls_d_comp_H = objUser->motor_Ls_d_H * (1.0f - obj->Is_A * objSets->Ls_d_Icomp_coef);
                    objSets->Ls_q_comp_H = objUser->motor_Ls_q_H * (1.0f - obj->Is_A * objSets->Ls_q_Icomp_coef);

                    if(objSets->Ls_d_comp_H < objSets->Ls_min_H)
                    {
                        objSets->Ls_d_comp_H = objSets->Ls_min_H;
                    }

                    if(objSets->Ls_q_comp_H < objSets->Ls_min_H)
                    {
                        objSets->Ls_q_comp_H = objSets->Ls_min_H;
                    }

                    // Update the Ld and Lq for motor control
                    EST_setLs_d_H(obj->estHandle, objSets->Ls_d_comp_H);
                    EST_setLs_q_H(obj->estHandle, objSets->Ls_q_comp_H);
                }
                PI_setMinMax(obj->piHandle_spd, -obj->maxCurrent_A, obj->maxCurrent_A);

                SVGEN_setMode(obj->svgenHandle, obj->svmMode);

#if defined(MOTOR1_VIBCOMPA)
                VIB_COMP_setFlag_enableOutput(obj->vibCompHandle, obj->vibCompFlagEnable);
#endif  // MOTOR1_VIBCOMPA

                if(obj->motorState == MOTOR_CL_RUNNING)
                {
                    obj->stateRunTimeCnt++;

                    if(obj->stateRunTimeCnt == obj->fwcTimeDelay)
                    {
                        obj->Idq_out_A.value[0] = 0.0f;
                        obj->motorState = MOTOR_CTRL_RUN;

#if defined(MOTOR1_VIBCOMPA) || defined(MOTOR1_VIBCOMPT)
                        VIB_COMP_setAngleMechPoles(obj->vibCompHandle, obj->angleFOC_rad);
#endif  // MOTOR1_VIBCOMPA || MOTOR1_VIBCOMPT
                    }
                }
            }
            else
            {
                TRAJ_setMaxDelta(obj->trajHandle_spd,
                    (obj->accelerationStart_Hzps * objUser->ctrlPeriod_sec));

                if(obj->speed_int_Hz >= 0.0f)
                {
                    PI_setMinMax(obj->piHandle_spd, 0.0f, obj->startCurrent_A);
                }
                else
                {
                    PI_setMinMax(obj->piHandle_spd, -obj->startCurrent_A, 0.0f);
                }
            }
        }

        // Identification
#if(DMC_BUILDLEVEL == DMC_LEVEL_3)
        obj->Idq_out_A.value[0] = obj->Idq_set_A.value[0];
        obj->Idq_out_A.value[1] = obj->Idq_set_A.value[1] * obj->direction;

#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)
    }
    else
    {
        // reset motor control parameters
        resetMotorControl(handle);
    }

    // check the trajectory generator
    if(EST_isTrajError(obj->estHandle) == true)
    {
        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);
    }
    else
    {
        // update the trajectory generator state
        EST_updateTrajState(obj->estHandle);
    }

    // check the estimator
    if(EST_isError(obj->estHandle) == true)
    {
        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);
    }
    else // No estimator error
    {
        bool flagEstStateChanged = false;

        float32_t Id_target_A = EST_getIntValue_Id_A(obj->estHandle);

        if(obj->flagMotorIdentified == true)
        {
            flagEstStateChanged = EST_updateState(obj->estHandle, 0.0f);
        }
        else
        {
            flagEstStateChanged = EST_updateState(obj->estHandle, Id_target_A);
        }

        if(flagEstStateChanged == true)
        {
            // configure the trajectory generator
            EST_configureTraj(obj->estHandle);

            if(obj->flagMotorIdentified == false)
            {
                // configure the controllers
                EST_configureTrajState(obj->estHandle, obj->userParamsHandle,
                                       obj->piHandle_spd,
                                       obj->piHandle_Id, obj->piHandle_Iq);
            }

            if(objUser->flag_bypassMotorId == false)
            {
                if((EST_isLockRotor(obj->estHandle) == true) ||
                        ((EST_isMotorIdentified(obj->estHandle) == true) &&
                                    (EST_isIdle(obj->estHandle) == true)))
                {
                    if(EST_isMotorIdentified(obj->estHandle) == true)
                    {
                        obj->flagMotorIdentified = true;

                        // clear the flag
                        obj->flagRunIdentAndOnLine = false;
                        obj->flagEnableRunAndIdentify = false;

                        // disable the estimator
                        EST_disable(obj->estHandle);

                        // enable the trajectory generator
                        EST_disableTraj(obj->estHandle);
                    }
                }
            }   // objUser->flag_bypassMotorId == false
        }
    }

    obj->flagMotorIdentified = EST_isMotorIdentified(obj->estHandle);

    if(obj->flagMotorIdentified == true)
    {
        if(obj->flagSetupController == true)
        {
            // update the controller
            updateControllers(handle);
        }
        else
        {
            obj->flagSetupController = true;

            setupControllers(handle);
        }

#if defined(MOTOR1_VIBCOMPA)
        VIB_COMP_setAlphaGain(obj->vibCompHandle, obj->vibCompAlpha, obj->vibCompGain);

        VIB_COMP_setAdvIndexDelta(obj->vibCompHandle, obj->vibCompIndexDelta);

        if(obj->vibCompFlagReset == true)
        {
            obj->vibCompFlagReset = false;
            VIB_COMP_reset(obj->vibCompHandle);
        }
#endif // MOTOR1_VIBCOMPA
    }

    // run Rs online
    runRsOnLine(handle);

    // update the global variables
    updateGlobalVariables(handle);

    return;
}   // end of the runMotor1Control() function


__interrupt void motor1CtrlISR(void)
{
    motorVars_M1.isrCount++;

    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)motorHandle_M1;
    USER_Params *objUser = (USER_Params *)(obj->userParamsHandle);

    // acknowledge the ADC interrupt
    HAL_ackMtr1ADCInt();

    // read the ADC data with offsets
    HAL_readMtr1ADCData(&obj->adcData);

#if defined(MOTOR1_DCLINKSS)
    // run single-shunt current reconstruction
    DCLINK_SS_runCurrentReconstruction(obj->dclinkHandle,
                                     &obj->adcData.Idc1_A, &obj->adcData.Idc2_A);

    obj->dclinkSector = DCLINK_SS_getSector1(obj->dclinkHandle);

    obj->adcData.I_A.value[0] = DCLINK_SS_getIa(obj->dclinkHandle);
    obj->adcData.I_A.value[1] = DCLINK_SS_getIb(obj->dclinkHandle);
    obj->adcData.I_A.value[2] = DCLINK_SS_getIc(obj->dclinkHandle);
#else // !(MOTOR1_DCLINKSS)
#if defined(MOTOR1_OVM)
    // Over Modulation Supporting, run the current reconstruction algorithm
    SVGENCURRENT_RunRegenCurrent(obj->svgenCurrentHandle,
                                 &obj->adcData.I_A, &obj->adcDataPrev);
#endif  // MOTOR1_OVM
#endif // !(MOTOR1_DCLINKSS)

    // sensorless-FOC
    MATH_Vec2 phasor;

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3)
    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);
#endif  // (DMC_BUILDLEVEL <= DMC_LEVEL_3)

    // remove offsets
    obj->adcData.V_V.value[0] -=
            obj->adcData.offset_V_sf.value[0] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[1] -=
            obj->adcData.offset_V_sf.value[1] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[2] -=
            obj->adcData.offset_V_sf.value[2] * obj->adcData.VdcBus_V;

    // run Clarke transform on voltage
    CLARKE_run(obj->clarkeHandle_V,
               &obj->adcData.V_V, &obj->estInputData.Vab_V);

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->estInputData.Iab_A);

    // store the input data into a buffer
    obj->estInputData.dcBus_V = obj->adcData.VdcBus_V;

    // configure the trajectory generator
    EST_run(obj->estHandle, &obj->estInputData, &obj->estOutputData);

    // compute angle with delay compensation
    obj->angleDelta_rad =
            objUser->angleDelayed_sf_sec * obj->estOutputData.fm_lp_rps;

    obj->angleEST_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleDelta_rad);

    obj->speedEST_Hz = EST_getFm_lp_Hz(obj->estHandle);
    obj->speed_Hz = obj->speedEST_Hz;

    if(((EST_isMotorIdentified(obj->estHandle) == false) ||
            (EST_getState(obj->estHandle) == EST_STATE_RS)) &&
            (EST_isEnabled(obj->estHandle) == true))
    {
        obj->Idq_out_A.value[0] = 0.0f;
        obj->motorState = MOTOR_CTRL_RUN;

        // run identification or Rs Recalibration
        // setup the trajectory generator
        EST_setupTrajState(obj->estHandle,
                           obj->Idq_out_A.value[1],
                           obj->speedRef_Hz,
                           0.0);

        // run the trajectories
        EST_runTraj(obj->estHandle);

        obj->IdRated_A = EST_getIntValue_Id_A(obj->estHandle);

        // store the input data into a buffer
        obj->estInputData.speed_ref_Hz = EST_getIntValue_spd_Hz(obj->estHandle);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

        obj->enableSpeedCtrl = EST_doSpeedCtrl(obj->estHandle);
        obj->enableCurrentCtrl = EST_doCurrentCtrl(obj->estHandle);
    }
    else if(obj->flagMotorIdentified == true)   // Normal Running
    {
        if(obj->flagRunIdentAndOnLine == true)
        {
            obj->counterTrajSpeed++;

            if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
            {
                // clear counter
                obj->counterTrajSpeed = 0;

                // run a trajectory for speed reference,
                // so the reference changes with a ramp instead of a step
                TRAJ_run(obj->trajHandle_spd);
            }

            obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
            obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;

            // get Id reference for Rs OnLine
            obj->IdRated_A = EST_getIdRated_A(obj->estHandle);
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
        }

        obj->estInputData.speed_ref_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;
    }

    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    obj->oneOverDcBus_invV = obj->estOutputData.oneOverDcBus_invV;

    // Running state
    obj->stateRunTimeCnt++;

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleEST_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleEST_rad;
        obj->motorState = MOTOR_CL_RUNNING;
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->stateRunTimeCnt = 0;
            obj->motorState = MOTOR_OL_START;
            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = false;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        obj->angleFOC_rad = obj->angleEST_rad;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;
            }
            else
            {
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->estInputData.Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

//---------- Speed and Current Loop -------------------
#if(DMC_BUILDLEVEL >= DMC_LEVEL_4)

    // run the speed controller
    obj->counterSpeed++;

    if(obj->counterSpeed >= objUser->numCtrlTicksPerSpeedTick)
    {
        obj->counterSpeed = 0;

        if(obj->enableSpeedCtrl == true)
        {
            PI_run(obj->piHandle_spd,
                   obj->speed_int_Hz,
                   obj->speed_Hz,
                   (float32_t *)&obj->IsRef_A);
        }
        else if((obj->motorState >= MOTOR_CL_RUNNING) &&
                (obj->flagMotorIdentified == true))
        {
            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->IsSet_A;
            }
            else
            {
                obj->IsRef_A = -obj->IsSet_A;
            }

            // for switching back speed closed-loop control
            PI_setUi(obj->piHandle_spd, obj->IsRef_A);
        }
    }
#if defined(MOTOR1_FWC) && defined(MOTOR1_MTPA)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad =
                (obj->angleFWC_rad > obj->angleMTPA_rad) ?
                        obj->angleFWC_rad : obj->angleMTPA_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if((obj->flagEnableFWC == true) || (obj->flagEnableMTPA == true))
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                __sqrt((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = obj->VsRef_pu * obj->adcData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == true)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
    else if(obj->counterSpeed == 4)   // MTPA
    {
        if(obj->flagEnableMTPA == true)
        {
            obj->angleMTPA_rad =
                    MTPA_computeCurrentAngle(obj->mtpaHandle, obj->IsRef_A);
        }
        else
        {
            obj->angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_FWC)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad = obj->angleFWC_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if(obj->flagEnableFWC == true)
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                __sqrt((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = obj->VsRef_pu * obj->adcData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == true)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_MTPA)
    else if(counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        angleCurrentM1_rad = angleMTPA_rad;

        fwcPhasor.value[0] = __cos(angleCurrentM1_rad);
        fwcPhasor.value[1] = __sin(angleCurrentM1_rad);

        if(flagEnableMTPAM1 == true)
        {
            Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(counterSpeed == 4)   // MTPA
    {
        if(flagEnableMTPAM1 == true)
        {
            angleMTPA_rad = MTPA_computeCurrentAngle(mtpaHandle, obj->IsRef_A);
        }
        else
        {
            angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#else   // !MOTOR1_MTPA && !MOTOR1_FWC
    obj->Idq_out_A.value[1] = obj->IsRef_A;
#endif  // !MOTOR1_MTPA && !MOTOR1_FWC/

    obj->IdqRef_A.value[0] = obj->Idq_out_A.value[0] + obj->IdRated_A;

    // update Id reference for Rs OnLine
    EST_updateId_ref_A(obj->estHandle, &obj->IdqRef_A.value[0]);

#if defined(MOTOR1_VIBCOMPA)
    // get the Iq reference value plus vibration compensation
    obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1] +
            VIB_COMP_run(obj->vibCompHandle, obj->angleFOC_rad, obj->Idq_in_A.value[1]);
#elif defined(MOTOR1_VIBCOMPT)
    // This algorithm reduces speed ripple induced vibration by adjusting Iq
    // depending on compressor angle. Compressor angle is in radians and
    // compensation values are defined by vibCompAlpha0, vibCompAlpha120,
    // vibCompAlpha240.
    //
    // Fine tune the angle range for compensation depending on compressor's
    // torque vs angle profile
    //
    // Note that compressor may not align at 0 Mech degree at startup and hence
    // based on speed ripple, angle may need to be updated
    obj->vibCompAngle_rad = VIB_COMP_calcMechangle(obj->vibCompHandle, obj->angleFOC_rad);

    // adjust the Iq current for angle lower than 1.57rad by value of vibCompAlpha0
    if(obj->vibCompAngle_rad < 1.57f)
    {
        obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1] - obj->vibCompAlpha0;
    }
    else if(obj->vibCompAngle_rad >= 1.57f && obj->vibCompAngle_rad <= 4.2f )
    {
        obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1] +
                ((obj->vibCompAngle_rad - 1.57f) * obj->vibCompAlpha120);
    }
    else if(obj->vibCompAngle_rad > 4.2f)
    {
        obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1] -
                ((obj->vibCompAngle_rad - 4.2f) * obj->vibCompAlpha240);
    }
#else   // !MOTOR1_VIBCOMPA && !MOTOR1_VIBCOMPT
    obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1];
#endif  // !MOTOR1_VIBCOMPA && !MOTOR1_VIBCOMPT

#elif(DMC_BUILDLEVEL == DMC_LEVEL_3)
    obj->IdqRef_A.value[0] = obj->Idq_set_A.value[0];
    obj->IdqRef_A.value[1] = obj->Idq_set_A.value[1];
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

    if(obj->enableCurrentCtrl == true)
    {
        // Maximum voltage output
        objUser->maxVsMag_V =
                objUser->maxVsMag_pu * obj->adcData.VdcBus_V;

        PI_setMinMax(obj->piHandle_Id,
                     -objUser->maxVsMag_V, objUser->maxVsMag_V);

        // run the Id controller
        // run the Id controller
        PI_run(obj->piHandle_Id, obj->IdqRef_A.value[0],
               obj->Idq_in_A.value[0], (float32_t*)&obj->Vdq_out_V.value[0]);

        // calculate Iq controller limits
        float32_t outMax_V = __sqrt((objUser->maxVsMag_V * objUser->maxVsMag_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        // run the Iq controller
        PI_run(obj->piHandle_Iq, obj->IdqRef_A.value[1],
               obj->Idq_in_A.value[1], (float32_t*)&obj->Vdq_out_V.value[1]);



        // set the Id reference value in the estimator
        EST_setId_ref_A(obj->estHandle, obj->IdqRef_A.value[0]);
        EST_setIq_ref_A(obj->estHandle, obj->IdqRef_A.value[1]);
    }

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    VS_FREQ_run(obj->VsFreqHandle, obj->speed_int_Hz);
    obj->Vdq_out_V.value[0] = VS_FREQ_getVd_out(obj->VsFreqHandle);
    obj->Vdq_out_V.value[1] = VS_FREQ_getVq_out(obj->VsFreqHandle);
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

    // set the phasor in the inverse Park transform
    IPARK_setPhasor(obj->iparkHandle_V, &phasor);

    // run the inverse Park module
    IPARK_run(obj->iparkHandle_V,
              &obj->Vdq_out_V, &obj->Vab_out_V);

    // setup the space vector generator (SVGEN) module
    SVGEN_setup(obj->svgenHandle,
                obj->oneOverDcBus_invV);

    // run the space vector generator (SVGEN) module
    SVGEN_run(obj->svgenHandle,
              &obj->Vab_out_V, &(obj->pwmData.Vabc_pu));

#if(DMC_BUILDLEVEL == DMC_LEVEL_1)
    // output 50%
    obj->pwmData.Vabc_pu.value[0] = 0.0f;
    obj->pwmData.Vabc_pu.value[1] = 0.0f;
    obj->pwmData.Vabc_pu.value[2] = 0.0f;
#endif

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == false)
    {
        // clear PWM data
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;
    }

#if defined(MOTOR1_DCLINKSS)
    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

    // revise PWM compare(CMPA/B) values for shifting switching pattern
    // and, update SOC trigger point
    HAL_runSingleShuntCompensation(obj->halMtrHandle, obj->dclinkHandle,
                         &obj->Vab_out_V, &obj->pwmData, obj->adcData.VdcBus_V);
#else   // !(MOTOR1_DCLINKSS)
#if defined(MOTOR1_OVM)
    else
    {
        // run the PWM compensation and current ignore algorithm
        SVGENCURRENT_compPWMData(obj->svgenCurrentHandle,
                                 &obj->pwmData.Vabc_pu, &obj->pwmDataPrev);
    }

    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

    obj->ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(obj->svgenCurrentHandle);
    obj->midVolShunt = SVGENCURRENT_getVmid(obj->svgenCurrentHandle);

    // Set trigger point in the middle of the low side pulse
    HAL_setTrigger(obj->halMtrHandle,
                   &obj->pwmData, obj->ignoreShuntNextCycle, obj->midVolShunt);
#else   // !MOTOR1_OVM
    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);
#endif  // !MOTOR1_OVM
#endif // !(MOTOR1_DCLINKSS)

    // Collect current and voltage data to calculate the RMS value
    collectRMSData(motorHandle_M1);

#if defined(DATALOGF2_EN)
    if(DATALOGIF_enable(datalogHandle) == true)
    {
        DATALOGIF_updateWithDMA(datalogHandle);

        // Force trig DMA channel to save the data
        HAL_trigDMAforDLOG(halHandle, 0);
        HAL_trigDMAforDLOG(halHandle, 1);
    }
#endif  // DATALOGF2_EN

#if defined(DAC128S_ENABLE)
    // Write the variables data value to DAC128S085
    DAC128S_writeData(dac128sHandle);
#endif  // DAC128S_ENABLE

#ifdef FSI_LOG_EN
    // Write log data to FSI Tx buffer and start transmission
    HAL_writeFSIData(halHandle);
#endif  // FSI_LOG_EN

#ifdef SPEED_MONITOR_EN
    if(obj->flagClearRecord == 1)
    {
        obj->speedMax_Hz = 0.0f;
        obj->speedMin_Hz = 1000.0f;
        obj->flagClearRecord = 0;
    }
    else
    {
        if(obj->speed_Hz > obj->speedMax_Hz)
        {
            obj->speedMax_Hz = obj->speed_Hz;
        }

        if(obj->speed_Hz < obj->speedMin_Hz)
        {
            obj->speedMin_Hz = obj->speed_Hz;
        }

        obj->speedDelta_Hz = obj->speedMax_Hz - obj->speedMin_Hz;
    }
#endif  // SPEED_MONITOR_EN

    return;
} // end of motor1CtrlISR() function

//
//-- end of this file ----------------------------------------------------------
//
