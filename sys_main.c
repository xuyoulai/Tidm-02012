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
#include "user.h"
#include "sys_settings.h"
#include "sys_main.h"

volatile SYSTEM_Vars_t systemVars;
#pragma DATA_SECTION(systemVars,"sys_data");

#if defined(DAC128S_ENABLE)
DAC128S_Handle   dac128sHandle;        //!< the DAC128S interface handle
DAC128S_Obj      dac128s;              //!< the DAC128S interface object
#pragma DATA_SECTION(dac128sHandle,"sys_data");
#pragma DATA_SECTION(dac128s,"sys_data");
#endif  // DAC128S_ENABLE

#ifdef FSI_LOG_EN
FSILOG_Obj fsiLog;
#endif  // FSI_LOG_EN

// **************************************************************************
// the functions
// !!! Please make sure that you have gone through the design guide and followed the
// !!! guide to set up the kit and load the right code
void main(void)
{
    //Clear memory for system and controller
    // The variables must be assigned to these sector if need to be cleared to zero
    HAL_clearDataRAM((void *)ctrlVarsLoadStart,  (uint16_t)ctrlVarsLoadSize);
    HAL_clearDataRAM((void *)motorVarsLoadStart, (uint16_t)motorVarsLoadSize);
    HAL_clearDataRAM((void *)extVarsLoadStart,   (uint16_t)extVarsLoadSize);

#if defined(MOTOR1_OVM) && defined(MOTOR1_DCLINKSS)
#error Don't enable OVM if single shunt is enabled
#endif  // MOTOR1_OVM & MOTOR1_DCLINKSS

#if defined(MOTOR1_VIBCOMPT) && defined(MOTOR1_VIBCOMPA)
#error Don't enable both VIBCOMP methods at the same time
#endif  // MOTOR1_VIBCOMPT & MOTOR1_VIBCOMPA

    // initialize the driver
    halHandle = HAL_init(&hal, sizeof(hal));

    // set the driver parameters
    HAL_setParams(halHandle);

    // set the control parameters for motor 1
    motorHandle_M1 = (MOTOR_Handle)(&motorVars_M1);

    // set the reference speed, this can be replaced or removed
    motorVars_M1.speedRef_Hz = 40.0f;

    // false - enables identification, true - disables identification
// #if MOTOR_IDENT
#if defined(MOTOR1_IDENT)
    userParams_M1.flag_bypassMotorId = false;
#else
    userParams_M1.flag_bypassMotorId = true;
#endif

    initMotor1Handles(motorHandle_M1);

    // initialize motor control parameters
    initMotor1CtrlParameters(motorHandle_M1);

#if defined(DATALOGF2_EN)
    //
    // Note that some devices may not support datalog because of lack of
    // DMA and memory size
    //
#if defined(_F280015x)
#error F280015x can't support the DATALOGF2_EN option
#endif  // _F280015x

    // Initialize Datalog
    datalogHandle = DATALOGIF_init(&datalog, sizeof(datalog));
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;

    HAL_setupDMAforDLOG(halHandle, 0, &datalogBuff1[0], &datalogBuff1[1]);
    HAL_setupDMAforDLOG(halHandle, 1, &datalogBuff2[0], &datalogBuff2[1]);

#if (DMC_BUILDLEVEL <= DMC_LEVEL_2)
    // set datalog parameters
    datalogObj->iptr[0] = &motorVars_M1.adcData.I_A.value[0];
    datalogObj->iptr[1] = &motorVars_M1.adcData.I_A.value[1];
#elif (DMC_BUILDLEVEL == DMC_LEVEL_3)
    datalogObj->iptr[0] = &motorVars_M1.adcData.V_V.value[0];
    datalogObj->iptr[1] = &motorVars_M1.adcData.V_V.value[1];
#elif (DMC_BUILDLEVEL == DMC_LEVEL_4)
    datalogObj->iptr[0] = &motorVars_M1.angleFOC_rad;
    datalogObj->iptr[1] = &motorVars_M1.adcData.I_A.value[0];
#endif  // DMC_BUILDLEVEL = DMC_LEVEL_1/2/3/4
#endif  // DATALOGF2_EN

#if defined(DAC128S_ENABLE)
    //
    // Initialize the DAC128S
    // Note that if you want to change SPI instances (default is SPI A), you'll
    // need to change the DAC128S_SPIx predefined symbol
    //
    dac128sHandle = DAC128S_init(&dac128s);
    DAC128S_setupSPI(dac128sHandle);

    //
    // Comment/uncomment the values you want to send to the DAC
    //
    //************************************************************************
    // dac128s.ptrData[0] = &motorVars_M1.angleFOC_rad;                // CH_A
    // dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    // dac128s.ptrData[2] = &motorVars_M1.adcIs_A.value[0];            // CH_C
    // dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    // dac128s.gain[0] = 4096.0f / MATH_TWO_PI;
    // dac128s.gain[1] = 2.0f * 4096.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    // dac128s.gain[2] = 2.0f * 4096.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    // dac128s.gain[3] = 2.0f * 4096.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;

    // dac128s.offset[0] = (uint16_t)(0.5f * 4096.0f);
    // dac128s.offset[1] = (uint16_t)(0.5f * 4096.0f);
    // dac128s.offset[2] = (uint16_t)(0.5f * 4096.0f);
    // dac128s.offset[3] = (uint16_t)(0.5f * 4096.0f);

    //************************************************************************
    dac128s.ptrData[0] = &motorVars_M1.angleFOC_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[1];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[2];        // CH_D

    dac128s.gain[0] = 4096.0f / MATH_TWO_PI;
    dac128s.gain[1] = 2.0f * 4096.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = 2.0f * 4096.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 2.0f * 4096.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[1] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[2] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[3] = (uint16_t)(0.5f * 4096.0f);

    //************************************************************************
    // dac128s.ptrData[0] = &motorVars_M1.angleFOC_rad;                // CH_A
    // dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    // dac128s.ptrData[2] = &motorVars_M1.adcData.V_V.value[0];        // CH_C
    // dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    // dac128s.gain[0] = 4096.0f / MATH_TWO_PI;
    // dac128s.gain[1] = 4096.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    // dac128s.gain[2] = 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    // dac128s.gain[3] = 4096.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;

    // dac128s.offset[0] = (uint16_t)(0.5f * 4096.0f);
    // dac128s.offset[1] = (uint16_t)(0.5f * 4096.0f);
    // dac128s.offset[2] = (uint16_t)(0.5f * 4096.0f);
    // dac128s.offset[3] = (uint16_t)(0.5f * 4096.0f);

    //************************************************************************
    // dac128s.ptrData[0] = &motorVars_M1.angleGen_rad;                // CH_A
    // dac128s.ptrData[1] = &motorVars_M1.angleEST_rad;                // CH_B
    // dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[0];        // CH_C
    // dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    // dac128s.gain[0] = 4096.0f / MATH_TWO_PI;
    // dac128s.gain[1] = 4096.0f / MATH_TWO_PI;
    // dac128s.gain[2] = 4096.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    // dac128s.gain[3] = 4096.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;

    // dac128s.offset[0] = (uint16_t)(0.5f * 4096.0f);
    // dac128s.offset[1] = (uint16_t)(0.5f * 4096.0f);
    // dac128s.offset[2] = (uint16_t)(0.5f * 4096.0f);
    // dac128s.offset[3] = (uint16_t)(0.5f * 4096.0f);

    DAC128S_writeCommand(dac128sHandle);
#endif  // DAC128S_ENABLE

    //
    // Note that some devices may not support FSI logging because of lack of
    // FSI and memory size
    //
#if defined(_F280015x) && defined(FSI_LOG_EN)
#error F280015x can't support the FSI_LOG_EN option
#endif  // _F280015x & FSI_LOG_EN
#if defined(FSI_LOG_EN) && defined(LOG_ANGLE_CURRENT_VOLTAGE)
    fsiLog.ptrData[0] = &motorHandle_M1->angleFOC_rad;          // CH_A
    fsiLog.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];     // CH_B
    fsiLog.ptrData[2] = &motorVars_M1.adcData.V_V.value[0];     // CH_C

    fsiLog.gain[0] = 4096.0f / MATH_TWO_PI;
    fsiLog.gain[1] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    fsiLog.gain[2] = 4096.0f * 1.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    fsiLog.offset[0] = 0.5f * 4096.0f;
    fsiLog.offset[1] = 0.5f * 4096.0f;
    fsiLog.offset[2] = 0.5f * 4096.0f;
#elif defined(FSI_LOG_EN) && defined(LOG_CURRENT)
    fsiLog.ptrData[0] = &motorVars_M1.adcData.I_A.value[0];     // CH_A
    fsiLog.ptrData[1] = &motorVars_M1.adcData.I_A.value[1];     // CH_B
    fsiLog.ptrData[2] = &motorVars_M1.adcData.I_A.value[2];     // CH_C

    fsiLog.gain[0] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    fsiLog.gain[1] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    fsiLog.gain[2] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;

    fsiLog.offset[0] = 0.5f * 4096.0f;
    fsiLog.offset[1] = 0.5f * 4096.0f;
    fsiLog.offset[2] = 0.5f * 4096.0f;
#elif defined(FSI_LOG_EN) && defined(LOG_CURRENT_DQ)
    fsiLog.ptrData[0] = &motorVars_M1.adcData.I_A.value[0];     // CH_A
    fsiLog.ptrData[1] = &motorVars_M1.Idq_in_A.value[0];     // CH_B
    fsiLog.ptrData[2] = &motorVars_M1.Idq_in_A.value[1];     // CH_C

    fsiLog.gain[0] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    fsiLog.gain[1] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    fsiLog.gain[2] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;

    fsiLog.offset[0] = 0.5f * 4096.0f;
    fsiLog.offset[1] = 0.5f * 4096.0f;
    fsiLog.offset[2] = 0.5f * 4096.0f;
#elif defined(FSI_LOG_EN) && defined(LOG_IQ_VDQ)
    fsiLog.ptrData[0] = &motorVars_M1.Idq_in_A.value[1];     // CH_A
    fsiLog.ptrData[1] = &motorVars_M1.Vdq_out_V.value[0];     // CH_B
    fsiLog.ptrData[2] = &motorVars_M1.Vdq_out_V.value[1];     // CH_C

    fsiLog.gain[0] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    fsiLog.gain[1] = 4096.0f * 1.0f / 25.0f; //USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    fsiLog.gain[2] = 4096.0f * 1.0f / 25.0f; //USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    fsiLog.offset[0] = 0.5f * 4096.0f;
    fsiLog.offset[1] = 0.5f * 4096.0f;
    fsiLog.offset[2] = 0.5f * 4096.0f;
#elif defined(FSI_LOG_EN) && defined(LOG_IQ_UQ_UIQ)
    fsiLog.ptrData[0] = &motorVars_M1.Idq_in_A.value[1];     // CH_A
    fsiLog.ptrData[1] = &motorVars_M1.Vdq_out_V.value[1];     // CH_B
    fsiLog.ptrData[2] = &motorVars_M1.piHandle_Iq->Ui;     // CH_C

    fsiLog.gain[0] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    fsiLog.gain[1] = 4096.0f * 1.0f / 25.0f; //USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    fsiLog.gain[2] = 4096.0f * 1.0f / 25.0f; //USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    fsiLog.offset[0] = 0.5f * 4096.0f;
    fsiLog.offset[1] = 0.5f * 4096.0f;
    fsiLog.offset[2] = 0.5f * 4096.0f;
#elif defined(FSI_LOG_EN) && defined(LOG_IQ_UQ_VDC)
    fsiLog.ptrData[0] = &motorVars_M1.Idq_in_A.value[1];     // CH_A
    fsiLog.ptrData[1] = &motorVars_M1.Vdq_out_V.value[1];     // CH_B
    fsiLog.ptrData[2] = &motorVars_M1.adcData.VdcBus_V;     // CH_C

    fsiLog.gain[0] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    fsiLog.gain[1] = 4096.0f * 1.0f / 50.0f; //USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    fsiLog.gain[2] = 4096.0f * 1.0f / 50.0f; //USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    fsiLog.offset[0] = 0.5f * 4096.0f;
    fsiLog.offset[1] = 0.5f * 4096.0f;
    fsiLog.offset[2] = 0.0f;
#elif defined(FSI_LOG_EN) && defined(LOG_IU_IV_SECTOR)
    fsiLog.ptrData[0] = &motorVars_M1.adcData.I_A.value[0];     // CH_A
    fsiLog.ptrData[1] = &motorVars_M1.adcData.I_A.value[1];     // CH_B
//    fsiLog.ptrData[2] = &motorVars_M1.dclinkHandle->sector;     // CH_C

    fsiLog.gain[0] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    fsiLog.gain[1] = 4096.0f * 1.0f / USER_M1_DAC_FULL_SCALE_CURRENT_A;
    fsiLog.gain[2] = 4096.0f * 1.0f / 10.0f;;

    fsiLog.offset[0] = 0.5f * 4096.0f;
    fsiLog.offset[1] = 0.5f * 4096.0f;
    fsiLog.offset[2] = 0.0f;
#endif  // FSI_LOG_EN

    systemVars.flagEnableSystem = true;

#if defined(CMD_CAN_EN)
    // setup the CAN interrupt
    CANCOM_enableCANInts(halHandle);

    // initialize the CANCOM
    CANCOM_init();

    motorVars_M1.cmdCAN.speedSet_Hz = 40.0f;

    motorVars_M1.cmdCAN.flagEnableCmd = true;
    motorVars_M1.cmdCAN.flagEnableSyncLead = false;
#endif // CMD_CAN_EN

    motorVars_M1.flagEnableOffsetCalc = true;

    // run offset calibration for motor 1
    runMotor1OffsetsCalculation(motorHandle_M1);

    // enable global interrupts
    HAL_enableGlobalInts(halHandle);

    // enable debug interrupts
    HAL_enableDebugInt(halHandle);

    systemVars.powerRelayWaitTime_ms = POWER_RELAY_WAIT_TIME_ms;

    // Waiting for enable system flag to be set
    while(systemVars.flagEnableSystem == false)
    {
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER_TIMEBASE))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER_TIMEBASE);

            systemVars.timerBase_1ms++;

            if(systemVars.timerBase_1ms > systemVars.powerRelayWaitTime_ms)
            {
                systemVars.flagEnableSystem = true;
                systemVars.timerBase_1ms = 0;
            }
        }
    }

    motorVars_M1.flagInitializeDone = true;

    while(systemVars.flagEnableSystem == true)
    {
        // loop while the enable system flag is true
        systemVars.mainLoopCnt++;

        // 1ms time base
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER_TIMEBASE))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER_TIMEBASE);

            // toggle status LED
            systemVars.counterLED++;

            if(systemVars.counterLED > (uint16_t)(LED_BLINK_FREQ_Hz * 1000))
            {
                HAL_toggleLED(halHandle, LED2_GPIO);

                systemVars.counterLED = 0;
            }

            systemVars.timerBase_1ms++;

            switch(systemVars.timerBase_1ms)
            {
                case 1:     // motor 1 protection check
                    runMotorMonitor(motorHandle_M1);
                    break;
                case 2:
                    calculateRMSData(motorHandle_M1);
                    break;
                case 3:
                    break;
                case 4:     // calculate motor protection value
                    calcMotorOverCurrentThreshold(motorHandle_M1);
                    break;
                case 5:     // system control
                    systemVars.timerBase_1ms = 0;
                    systemVars.timerCnt_5ms++;
                    break;
                default:
                    break;
            }

#if defined(CMD_CAN_EN)
            CANCOM_updateCANCmdFreq(motorHandle_M1);

            if((motorVars_M1.cmdCAN.flagEnableCmd == true) && (motorVars_M1.faultMtrUse.all == 0))
            {
                canComVars.flagCmdTxRun = motorVars_M1.cmdCAN.flagCmdRun;
                canComVars.speedSet_Hz = motorVars_M1.cmdCAN.speedSet_Hz;

                if(motorVars_M1.cmdCAN.flagEnableSyncLead == true)
                {
                    motorVars_M1.flagEnableRunAndIdentify = motorVars_M1.cmdCAN.flagCmdRun;
                    motorVars_M1.speedRef_Hz = motorVars_M1.cmdCAN.speedSet_Hz;
                }
                else
                {
                    motorVars_M1.flagEnableRunAndIdentify = canComVars.flagCmdRxRun;
                    motorVars_M1.speedRef_Hz = canComVars.speedRef_Hz;
                }
            }
#endif // CMD_CAN_EN
        }       // 1ms Timer

        // run control for motor 1
        runMotor1Control(motorHandle_M1);

    } // end of while() loop

    // disable the PWM
    HAL_disablePWM(motorHandle_M1->halMtrHandle);

} // end of main() function

//
//-- end of this file ----------------------------------------------------------
//
