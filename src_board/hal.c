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
// the includes
//
#include "user.h"

//
// drivers
//

// modules

// platforms
#include "board.h"
#include "hal.h"
#include "hal_obj.h"

// libraries
#include "datalogIF.h"

#ifdef _FLASH
#pragma CODE_SECTION(Flash_initModule, ".TI.ramfunc");
#endif

// **************************************************************************
// the defines

// **************************************************************************
// the globals
HAL_Handle    halHandle;      //!< the handle for the hardware abstraction layer
HAL_Obj       hal;            //!< the hardware abstraction layer object
#pragma DATA_SECTION(halHandle, "user_data");
#pragma DATA_SECTION(hal, "user_data");

// **************************************************************************
// the functions

void HAL_disableGlobalInts(HAL_Handle handle)
{
    // disable global interrupts
    Interrupt_disableMaster();

    return;
} // end of HAL_disableGlobalInts() function

void HAL_disableWdog(HAL_Handle halHandle)
{
    // disable watchdog
    SysCtl_disableWatchdog();

    return;
} // end of HAL_disableWdog() function

void HAL_enableCtrlInts(HAL_Handle handle)
{
    // Acknowledge interrupt from PIE group
    Interrupt_clearACKGroup(MTR1_INT_ACK_GROUP);

    // enable the PIE interrupts associated with the ADC interrupts
    Interrupt_enable(MTR1_PIE_INT_NUM);    // motor_1

    // enable the ADC interrupts for motor_1
    ADC_enableInterrupt(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

    return;
} // end of HAL_enableCtrlInts() function

void HAL_enableDebugInt(HAL_Handle handle)
{
    // enable debug events
    ERTM;

    return;
} // end of HAL_enableDebugInt() function

void HAL_enableGlobalInts(HAL_Handle handle)
{
    // enable global interrupts
    Interrupt_enableMaster();

    return;
} // end of HAL_enableGlobalInts() function

HAL_Handle HAL_init(void *pMemory, const size_t numBytes)
{
    HAL_Handle handle;
    HAL_Obj *obj;

    if(numBytes < sizeof(HAL_Obj))
    {
        return((HAL_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_Handle)pMemory;

    // assign the object
    obj = (HAL_Obj *)handle;

    // disable watchdog
    SysCtl_disableWatchdog();

    // Three ADC modules in this device
    // initialize the ADC handles
    obj->adcHandle[0] = ADCA_BASE;
    obj->adcHandle[1] = ADCC_BASE;

    // initialize the ADC results
    obj->adcResult[0] = ADCARESULT_BASE;
    obj->adcResult[1] = ADCCRESULT_BASE;

    // initialize SCI handle
    obj->sciHandle = SCIA_BASE;

    // initialize LIN handle
    obj->linHandle = LINA_BASE;

    // initialize MCAN handle
    obj->mcanHandle = MCAN_COM_BASE;

    // initialize timer handles
    obj->timerHandle[0] = CPUTIMER0_BASE;
    obj->timerHandle[1] = CPUTIMER1_BASE;
    obj->timerHandle[2] = CPUTIMER2_BASE;

    return(handle);
} // end of HAL_init() function

HAL_MTR_Handle HAL_MTR1_init(void *pMemory, const size_t numBytes)
{
    HAL_MTR_Handle handle;
    HAL_MTR_Obj *obj;

    if(numBytes < sizeof(HAL_MTR_Obj))
    {
        return((HAL_MTR_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_MTR_Handle)pMemory;

    // assign the object
    obj = (HAL_MTR_Obj *)handle;

    // initialize PWM handles for Motor 1
    obj->pwmHandle[0] = MTR1_PWM_U_BASE;        //!< the PWM handle
    obj->pwmHandle[1] = MTR1_PWM_V_BASE;        //!< the PWM handle
    obj->pwmHandle[2] = MTR1_PWM_W_BASE;        //!< the PWM handle

    // initialize CMPSS handle
#if defined(MOTOR1_DCLINKSS)
    obj->cmpssHandle[0] = MTR1_CMPSS_IDC_BASE;  //!< the CMPSS handle
#else
    obj->cmpssHandle[0] = MTR1_CMPSS_IU_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[1] = MTR1_CMPSS_IV_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[2] = MTR1_CMPSS_IW_BASE;    //!< the CMPSS handle
#endif  // MOTOR1_DCLINKSS

    // Assign gateEnableGPIO
    obj->gateEnableGPIO = GPIO_SYS_DIS_FET_SUPPLY;

    obj->motorNum = MTR_1;

    return(handle);
} // end of HAL_MTR1_init() function

void HAL_setParams(HAL_Handle handle)
{
    // disable global interrupts
    Interrupt_disableMaster();

    // Disable the watchdog
    SysCtl_disableWatchdog();

#ifdef _FLASH
    //
    // Copy time critical code and flash setup code to RAM. This includes the
    // following functions: InitFlash();
    //
    // The RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart symbols
    // are created by the linker. Refer to the device .cmd file.
    //
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    memcpy(&ctrlfuncsRunStart, &ctrlfuncsLoadStart, (size_t)&ctrlfuncsLoadSize);

    // Call Flash Initialization to setup flash waitstates. This function must
    // reside in RAM.
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, DEVICE_FLASH_WAITSTATES);
#endif

    // initialize the interrupt controller
    Interrupt_initModule();

    // init vector table
    Interrupt_initVectorTable();

    // Set up PLL control and clock dividers
    SysCtl_setClock(DEVICE_SETCLOCK_CFG);

    // Configure the LSPCLK divider
    SysCtl_setLowSpeedClock(HAL_LSPCLK_PRESCALE);

    // Configure the divisor for the MCAN bit-clock
    SysCtl_setMCANClk(SYSCTL_MCANCLK_DIV_3);

    // These asserts will check that the #defines for the clock rates in
    // device.h match the actual rates that have been configured. If they do
    // not match, check that the calculations of DEVICE_SYSCLK_FREQ and
    // DEVICE_LSPCLK_FREQ are accurate. Some examples will not perform as
    // expected if these are not correct.
    ASSERT(SysCtl_getClock(DEVICE_OSCSRC_FREQ) == DEVICE_SYSCLK_FREQ);
    ASSERT(SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ) == HAL_LSPCLK_FREQ);

#ifndef _FLASH
    //
    // Call Device_cal function when run using debugger
    // This function is called as part of the Boot code. The function is called
    // in the Device_init function since during debug time resets, the boot code
    // will not be executed and the gel script will reinitialize all the
    // registers and the calibrated values will be lost.
    // Sysctl_deviceCal is a wrapper function for Device_Cal
    //
    SysCtl_deviceCal();
#endif

    // call the SysConfig-generated init function to setup peripherals
    Board_init();

#if defined(MOTOR1_DCLINKSS)
    // SysConfig doesn't currently support enabling COMPL DAC output but we want
    // to use it to supply a bias voltage for current feedback
    ASysCtl_enableCMPSSExternalDAC(ASYSCTL_CMPSS1_DACL);

#endif  //MOTOR1_DCLINKSS
    return;
} // end of HAL_setParams() function

void HAL_MTR_setParams(HAL_MTR_Handle handle, USER_Params *pUserParams)
{
    HAL_setNumCurrentSensors(handle, pUserParams->numCurrentSensors);
    HAL_setNumVoltageSensors(handle, pUserParams->numVoltageSensors);

    // Finish the PWM setup
    HAL_initPWMCounts(handle);

    return;
} // end of HAL_MTR_setParams() function

// HAL_enableGate
void HAL_enableGate(HAL_MTR_Handle handle)
{
   HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

   // Set pin high to enable the FET supply
   GPIO_writePin(obj->gateEnableGPIO, 1);

    return;
} // HAL_enableGate() function

void HAL_initPWMCounts(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    uint16_t pwmPeriodCycles = (uint16_t)(USER_M1_PWM_TBPRD_NUM);
    uint16_t numPWMTicksPerISRTick = USER_M1_NUM_PWM_TICKS_PER_ISR_TICK;

    // setup the Event Trigger Prescale Register (ETPS)
    if(numPWMTicksPerISRTick > 15)
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], 15);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A, 15);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_B, 15);

#if defined(MOTOR1_DCLINKSS)
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_A, 15);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_B, 15);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_A, 15);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_B, 15);
#endif  //MOTOR1_DCLINKSS
    }
    else if(numPWMTicksPerISRTick < 1)
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], 1);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A, 1);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_B, 1);

#if defined(MOTOR1_DCLINKSS)
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_A, 1);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_B, 1);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_A, 1);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_B, 1);
#endif  //MOTOR1_DCLINKSS
    }
    else
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], numPWMTicksPerISRTick);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A,
                                        numPWMTicksPerISRTick);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_B,
                                        numPWMTicksPerISRTick);

#if defined(MOTOR1_DCLINKSS)
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_A,
                                        numPWMTicksPerISRTick);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_B,
                                        numPWMTicksPerISRTick);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_A,
                                        numPWMTicksPerISRTick);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_B,
                                        numPWMTicksPerISRTick);
#endif  //MOTOR1_DCLINKSS
    }

    // setup the Event Trigger Clear Register (ETCLR)
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_B);

    // since the PWM is configured as an up/down counter, the period register is
    // set to one-half of the desired PWM period
    EPWM_setTimeBasePeriod(obj->pwmHandle[0], pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], pwmPeriodCycles);

    // write the PWM data value  for ADC trigger
    EPWM_setCounterCompareValue(obj->pwmHandle[0], EPWM_COUNTER_COMPARE_C, 10);

#if defined(MOTOR1_DCLINKSS)
    EPWM_clearADCTriggerFlag(obj->pwmHandle[1], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[1], EPWM_SOC_B);

    EPWM_clearADCTriggerFlag(obj->pwmHandle[2], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[2], EPWM_SOC_B);

    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C, pwmPeriodCycles >> 1);
    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D, pwmPeriodCycles >> 1);

    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C, pwmPeriodCycles >> 1);
    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D, pwmPeriodCycles >> 1);
#endif  //MOTOR1_DCLINKSS

    // setup the Dead-Band Rising Edge Delay Register (DBRED)
    EPWM_setRisingEdgeDelayCount(obj->pwmHandle[0], MTR1_PWM_DBRED_CNT);
    EPWM_setRisingEdgeDelayCount(obj->pwmHandle[1], MTR1_PWM_DBRED_CNT);
    EPWM_setRisingEdgeDelayCount(obj->pwmHandle[2], MTR1_PWM_DBRED_CNT);

    // setup the Dead-Band Falling Edge Delay Register (DBFED)
    EPWM_setFallingEdgeDelayCount(obj->pwmHandle[0], MTR1_PWM_DBFED_CNT);
    EPWM_setFallingEdgeDelayCount(obj->pwmHandle[1], MTR1_PWM_DBFED_CNT);
    EPWM_setFallingEdgeDelayCount(obj->pwmHandle[2], MTR1_PWM_DBFED_CNT);

    // enable the ePWM module time base clock sync signal
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    return;
}  // end of HAL_setupPWMs() function


void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory)
{
    uint16_t *pMemoryStart;
    uint16_t loopCount, loopLength;

    pMemoryStart = pMemory;
    loopLength = lengthMemory;

    for(loopCount = 0; loopCount < loopLength; loopCount++)
    {
        *(pMemoryStart + loopCount) = 0x0000;
    }
}   //end of HAL_clearDataRAM() function

void HAL_setMtrCMPSSDACValue(HAL_MTR_Handle handle,
                             const uint16_t dacValH, const uint16_t dacValL)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(MOTOR1_DCLINKSS)
    // Use the low value since we have the high comparator output inverted for
    // single-shunt configuration, and we're using the low comparator as a DAC
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValL);
#else
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[0], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[1], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[1], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[2], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[2], dacValL);
#endif // MOTOR1_DCLINKSS

    return;
}

void HAL_setTriggerPrams(HAL_PWMData_t *pPWMData, const float32_t systemFreq_MHz,
                   const float32_t deadband_us, const float32_t noiseWindow_us)
{
    uint16_t deadband =  (uint16_t)(deadband_us * systemFreq_MHz);
    uint16_t noiseWindow =  (uint16_t)(noiseWindow_us * systemFreq_MHz);

    pPWMData->deadband = deadband;
    pPWMData->noiseWindow = noiseWindow;

    pPWMData->minCMPValue = deadband + noiseWindow + 33U;

    return;
}

// end of file
