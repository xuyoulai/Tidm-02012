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

#ifndef HAL_H
#define HAL_H

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
//! \addtogroup HAL HAL
//! @{
//
//*****************************************************************************

// the includes
#include "userParams.h"

// platforms
#include "board.h"
#include "hal_obj.h"

#include "svgen_current.h"

#if defined(MOTOR1_DCLINKSS)
#include "dclink_ss.h"
#endif // MOTOR1_DCLINKSS


// the globals
extern HAL_Handle    halHandle;
extern HAL_Obj       hal;

#ifdef _FLASH
extern uint16_t ctrlfuncsLoadStart;
extern uint16_t ctrlfuncsLoadEnd;
extern uint16_t ctrlfuncsLoadSize;
extern uint16_t ctrlfuncsRunStart;
extern uint16_t ctrlfuncsRunEnd;
extern uint16_t ctrlfuncsRunSize;
#endif

// To clear the RAM for initialization
extern uint16_t ctrlVarsLoadStart;
extern uint16_t ctrlVarsLoadSize;
extern uint16_t ctrlVarsLoadEnd;

extern uint16_t motorVarsLoadStart;
extern uint16_t motorVarsLoadSize;
extern uint16_t motorVarsLoadEnd;

extern uint16_t extVarsLoadStart;
extern uint16_t extVarsLoadSize;
extern uint16_t extVarsLoadEnd;

// **************************************************************************
// the defines
//
// LSPCLK frequency based on the DEVICE_SYSCLK_FREQ
#define HAL_LSPCLK_PRESCALE        SYSCTL_LSPCLK_PRESCALE_2
#define HAL_LSPCLK_FREQ            (DEVICE_SYSCLK_FREQ / 2)

//! Trip Zones all interrupt
//!
#define HAL_TZFLAG_INTERRUPT_ALL    (EPWM_TZ_INTERRUPT_DCBEVT2 |                \
                                     EPWM_TZ_INTERRUPT_DCBEVT1 |                \
                                     EPWM_TZ_INTERRUPT_DCAEVT2 |                \
                                     EPWM_TZ_INTERRUPT_DCAEVT1 |                \
                                     EPWM_TZ_INTERRUPT_OST |                    \
                                     EPWM_TZ_INTERRUPT_CBC)

#define HAL_TZSEL_SIGNALS_ALL       (EPWM_TZ_SIGNAL_CBC1 |                      \
                                     EPWM_TZ_SIGNAL_CBC2 |                      \
                                     EPWM_TZ_SIGNAL_CBC3 |                      \
                                     EPWM_TZ_SIGNAL_CBC4 |                      \
                                     EPWM_TZ_SIGNAL_CBC5 |                      \
                                     EPWM_TZ_SIGNAL_CBC6 |                      \
                                     EPWM_TZ_SIGNAL_DCAEVT2 |                   \
                                     EPWM_TZ_SIGNAL_DCBEVT2 |                   \
                                     EPWM_TZ_SIGNAL_OSHT1 |                     \
                                     EPWM_TZ_SIGNAL_OSHT2 |                     \
                                     EPWM_TZ_SIGNAL_OSHT3 |                     \
                                     EPWM_TZ_SIGNAL_OSHT4 |                     \
                                     EPWM_TZ_SIGNAL_OSHT5 |                     \
                                     EPWM_TZ_SIGNAL_OSHT6 |                     \
                                     EPWM_TZ_SIGNAL_DCAEVT1 |                   \
                                     EPWM_TZ_SIGNAL_DCBEVT1)

//------------------------------------------------------------------------------
// Motor 1

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      86

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      86

//------------------------------------------------------------------------------
// interrupt
#define MTR1_PWM_INT_BASE       MTR1_PWM_U_BASE

#define MTR1_ADC_INT_BASE       MTR1_VDC_SOC_ADC_BASE   // ADCA-A8*/C11
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCA_INT1
#define MTR1_ADC_INT_SOC        MTR1_VDC_SOC            // ADCA_INT1

#define MTR1_PIE_INT_NUM        INT_ADCA1               // ADCA_INT1
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCA_INT1 -CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCA_INT1 -CPU_INT1

//------------------------------------------------------------------------------
//! \brief Defines the function to turn LEDs off
//!
#define HAL_turnLEDOff              HAL_setGPIOHigh

//! \brief Defines the function to turn LEDs on
//!
#define HAL_turnLEDOn               HAL_setGPIOLow

//! \brief Defines the function to toggle LEDs
//!
#define HAL_toggleLED               HAL_toggleGPIO

// **************************************************************************
// the typedefs
//! \brief Enumeration for the CPU Timer
//!
typedef enum
{
    HAL_CPU_TIMER_TIMEBASE = 0,  //!< Select CPU Timer0
    HAL_CPU_TIMER1 = 1,  //!< Select CPU Timer1
    HAL_CPU_TIMER2 = 2   //!< Select CPU Timer2
} HAL_CPUTimerNum_e;

// **************************************************************************
// the globals
extern __interrupt void motor1CtrlISR(void);

extern volatile uint16_t mtrPIEIER;
extern volatile uint16_t mtrIER;

extern HAL_Handle halHandle;

// **************************************************************************
// the function prototypes

//! \brief     Acknowledges an interrupt from the ADC so that another ADC
//!            interrupt can happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
static inline void HAL_ackMtr1ADCInt(void)
{
#if defined(MOTOR1_DCLINKSS)
    // clear the PWM interrupt flag
    EPWM_clearEventTriggerInterruptFlag(MTR1_PWM_INT_BASE);
#endif // MOTOR1_DCLINKSS

    // clear the ADC interrupt flag
    ADC_clearInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

    // Acknowledge interrupt from PIE group
    Interrupt_clearACKGroup(MTR1_INT_ACK_GROUP);

    return;
} // end of HAL_ackADCInt() function

//! \brief      Disables global interrupts
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_disableGlobalInts(HAL_Handle handle);

//! \brief      Enables the ADC interrupts
//! \details    Enables the ADC interrupt in the PIE, and CPU.  Enables the
//!             interrupt to be sent from the ADC peripheral.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableCtrlInts(HAL_Handle handle);

//! \brief      Enables the debug interrupt
//! \details    The debug interrupt is used for the real-time debugger.  It is
//!             not needed if the real-time debugger is not used.  Clears
//!             bit 1 of ST1.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDebugInt(HAL_Handle handle);

//! \brief     Enables global interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableGlobalInts(HAL_Handle handle);

//! \brief     Gets the PWM duty cycle times
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] pDutyCycles  A pointer to memory for the duty cycle durations
static inline void
HAL_getDutyCycles(HAL_MTR_Handle handle, uint16_t *pDutyCycles)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    pDutyCycles[0] = EPWM_getCounterCompareValue(obj->pwmHandle[0],
                                                 EPWM_COUNTER_COMPARE_A);
    pDutyCycles[1] = EPWM_getCounterCompareValue(obj->pwmHandle[1],
                                                 EPWM_COUNTER_COMPARE_A);
    pDutyCycles[2] = EPWM_getCounterCompareValue(obj->pwmHandle[2],
                                                 EPWM_COUNTER_COMPARE_A);
    return;
} // end of HAL_getDutyCycles() function

//! \brief     Gets the number of current sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of current sensors
static inline uint16_t HAL_getNumCurrentSensors(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    return(obj->numCurrentSensors);
} // end of HAL_getNumCurrentSensors() function

//! \brief     Gets the number of voltage sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of voltage sensors
static inline uint16_t HAL_getNumVoltageSensors(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    return(obj->numVoltageSensors);
} // end of HAL_getNumVoltageSensors() function

//! \brief     Gets the pwm enable status
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
//! \return    The pwm enable
static inline bool HAL_getPwmEnableStatus(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    return(obj->flagEnablePWM);
} // end of HAL_getPwmStatus() function

//! \brief     Get the period of EPWM time-base module
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The periode of EPWM time-base module
static inline uint16_t
HAL_getTimeBasePeriod(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    return(EPWM_getTimeBasePeriod(obj->pwmHandle[0]));
} // end of HAL_getTimeBasePeriod() function

//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL object.
//! \param[in]  pMemory   A pointer to the memory for the hardware abstraction layer object
//! \param[in]  numBytes  The number of bytes allocated for the hardware abstraction layer object, bytes
//! \return     The hardware abstraction layer (HAL) object handle
extern HAL_Handle HAL_init(void *pMemory, const size_t numBytes);

//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL_MTR object.
//! \param[in]  pMemory   A pointer to the memory for the hardware abstraction layer object
//! \param[in]  numBytes  The number of bytes allocated for the hardware abstraction layer object, bytes
//! \return     The hardware abstraction layer (HAL_MTR) object handle
extern HAL_MTR_Handle HAL_MTR1_init(void *pMemory, const size_t numBytes);

//! \brief      Initializes the interrupt vector table
//! \details    Points the ISR to the function mainISR.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_initIntVectorTable(HAL_Handle handle)
{
    // Motor_1->ADCA_INT1
    Interrupt_register(MTR1_PIE_INT_NUM, &motor1CtrlISR);

    return;
} // end of HAL_initIntVectorTable() function

//! \brief      Reads the ADC data with offset
//! \details    Reads in the ADC result registers and scales the values
//!             according to the settings in user_m1.h or user_m2.h.
//!             The structure gAdcData holds three phase voltages,
//!             three line currents, and one DC bus voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pADCData  A pointer to the ADC data buffer
static inline void
HAL_readMtr1ADCData(HAL_ADCData_t *pADCData)
{
    float32_t value;

#if defined(MOTOR1_DCLINKSS)
    // convert dc-link current 1
    value = (float32_t)ADC_readPPBResult(MTR1_IDC1_SOC_RESULT_BASE, MTR1_IDC1_PPB);
    pADCData->Idc1_A.value[0] = value * pADCData->current_sf;

    // convert dc-link current 2
    value = (float32_t)ADC_readPPBResult(MTR1_IDC2_SOC_RESULT_BASE, MTR1_IDC2_PPB);
    pADCData->Idc1_A.value[1] = value * pADCData->current_sf;

    // convert dc-link current 3
    value = (float32_t)ADC_readPPBResult(MTR1_IDC3_SOC_RESULT_BASE, MTR1_IDC3_PPB);
    pADCData->Idc2_A.value[0] = value * pADCData->current_sf;

    // convert dc-link current 4th
    value = (float32_t)ADC_readPPBResult(MTR1_IDC4_SOC_RESULT_BASE, MTR1_IDC4_PPB);
    pADCData->Idc2_A.value[1] = value * pADCData->current_sf;
#else   // !(MOTOR1_DCLINKSS)
    // convert phase A current
    value = (float32_t) ADC_readPPBResult(MTR1_IU_SOC_RESULT_BASE, MTR1_IU_PPB);
    pADCData->I_A.value[0] = value * pADCData->current_sf;

    // convert phase B current
    value = (float32_t)ADC_readPPBResult(MTR1_IV_SOC_RESULT_BASE, MTR1_IV_PPB);
    pADCData->I_A.value[1] = value * pADCData->current_sf;

    // convert phase C current
    value = (float32_t)ADC_readPPBResult(MTR1_IW_SOC_RESULT_BASE, MTR1_IW_PPB);
    pADCData->I_A.value[2] = value * pADCData->current_sf;
#endif  // !(MOTOR1_DCLINKSS)

    // convert phase A voltage
    value = (float32_t)ADC_readResult(MTR1_VU_SOC_RESULT_BASE, MTR1_VU_SOC);
    pADCData->V_V.value[0] = value * pADCData->voltage_sf;

    // convert phase B voltage
    value = (float32_t)ADC_readResult(MTR1_VV_SOC_RESULT_BASE, MTR1_VV_SOC);
    pADCData->V_V.value[1] = value * pADCData->voltage_sf;

    // convert phase C voltage
    value = (float32_t)ADC_readResult(MTR1_VW_SOC_RESULT_BASE, MTR1_VW_SOC);
    pADCData->V_V.value[2] = value * pADCData->voltage_sf;

    // convert dc bus voltage
    value = (float32_t)ADC_readResult(MTR1_VDC_SOC_RESULT_BASE, MTR1_VDC_SOC);
    pADCData->VdcBus_V = value * pADCData->dcBusvoltage_sf;

    return;
} // end of HAL_readMtr1ADCData() functions

//! \brief     Reads the timer count
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0, 1 or 2
//! \return    The timer count
static inline uint32_t
HAL_readCPUTimerCnt(HAL_Handle handle, const HAL_CPUTimerNum_e timerNumber)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint32_t timerCnt = CPUTimer_getTimerCount(obj->timerHandle[timerNumber]);

    return(timerCnt);
} // end of HAL_readCPUTimerCnt() function

//! \brief     Sets the GPIO pin high
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_setGPIOHigh(HAL_Handle handle, const uint32_t gpioNumber)
{
    // set GPIO high
    GPIO_writePin(gpioNumber, 1);

    return;
} // end of HAL_setGPIOHigh() function

//! \brief     Read the GPIO pin
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
//! \return    The GPIO pin
static inline uint32_t
HAL_readGPIOData(HAL_Handle handle, const uint32_t gpioNumber)
{
    uint32_t gpioPinData;

    // set GPIO high
    gpioPinData = GPIO_readPin(gpioNumber);

    return(gpioPinData);
} // end of HAL_readGPIOData() function

//! \brief     Sets the GPIO pin low
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_setGPIOLow(HAL_Handle handle, const uint32_t gpioNumber)
{
    // set GPIO low
    GPIO_writePin(gpioNumber, 0);

    return;
} // end of HAL_setGPIOLow() function

//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
static inline void
HAL_setNumVoltageSensors(HAL_MTR_Handle handle, const uint16_t numVoltageSensors)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    obj->numVoltageSensors = numVoltageSensors;

    return;
} // end of HAL_setNumVoltageSensors() function

//! \brief     Sets the number of current sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numCurrentSensors  The number of current sensors
static inline void
HAL_setNumCurrentSensors(HAL_MTR_Handle handle, const uint16_t numCurrentSensors)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    obj->numCurrentSensors = numCurrentSensors;

    return;
} // end of HAL_setNumCurrentSensors() function

//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the scale
//!             factors for the ADC voltage and current conversions.  Sets the initial
//!             offset values for voltage and current measurements.
//! \param[in]  handle       The hardware abstraction layer (HAL) handle
extern void HAL_setParams(HAL_Handle handle);

//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the scale
//!             factors for the ADC voltage and current conversions.  Sets the initial
//!             offset values for voltage and current measurements.
//! \param[in]  handle       The hardware abstraction layer (HAL) handle
extern void HAL_MTR_setParams(HAL_MTR_Handle handle, USER_Params *pUserParams);

//! \brief     Sets up the peripheral clocks
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPeripheralClks(HAL_Handle handle);

//! \brief     Sets up the PIE (Peripheral Interrupt Expansion)
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPIE(HAL_Handle handle);

//! \brief      Enables the high voltage kit device
//! \details    Provides the correct timing to enable the high voltage kit
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableGate(HAL_MTR_Handle handle);

//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The CPU timer number
static inline void
HAL_clearCPUTimerFlag(HAL_Handle halHandle, const HAL_CPUTimerNum_e timerNumber)
{
    HAL_Obj   *obj = (HAL_Obj *)halHandle;

    CPUTimer_clearOverflowFlag(obj->timerHandle[timerNumber]);

    return;
}   // end of HAL_clearCPUTimerFlag() function

//! \brief     Gets CPU Timer status
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The CPU timer number
static inline bool
HAL_getCPUTimerStatus(HAL_Handle halHandle, const HAL_CPUTimerNum_e timerNumber)
{
    HAL_Obj   *obj = (HAL_Obj *)halHandle;

    return (CPUTimer_getTimerOverflowStatus(obj->timerHandle[timerNumber]));
}


//! \brief     Toggles the GPIO pin
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_toggleGPIO(HAL_Handle handle, const uint32_t gpioNumber)
{

    // set GPIO high
    GPIO_togglePin(gpioNumber);

    return;
} // end of HAL_toggleGPIO() function

//! \brief Clears a range of RAM memory
//! \param[in] pMemory Pointer to the start of the range to clear
//! \param[in] lengthMemory Size of the memory range to clear in 16-bit words
void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory);

//! \brief     Reads PWM period register
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM period value
static inline uint16_t
HAL_readPWMPeriod(HAL_MTR_Handle handle, const uint16_t pwmNumber)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // the period value to be returned
    uint16_t pwmPeriodValue;

    pwmPeriodValue = EPWM_getTimeBasePeriod(obj->pwmHandle[pwmNumber]);

    return(pwmPeriodValue);
} // end of HAL_readPWMPeriod() function

//! \brief     Writes PWM data to the PWM comparators for motor control
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMData  The pointer to the PWM data
static inline void
HAL_writePWMData(HAL_MTR_Handle handle, HAL_PWMData_t *pPWMData)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    float32_t period = (float32_t)(EPWM_getTimeBasePeriod(obj->pwmHandle[0]));

    uint16_t pwmCnt;

    for(pwmCnt=0; pwmCnt<3; pwmCnt++)
    {
      // compute the value
        float32_t V_pu = -pPWMData->Vabc_pu.value[pwmCnt];      // Negative
        float32_t V_sat_pu = __fsat(V_pu, 0.5, -0.5);           // -0.5~0.5
        float32_t V_sat_dc_pu = V_sat_pu + 0.5;                 // 0~1.0
        pPWMData->cmpValue[pwmCnt]  = (int16_t)(V_sat_dc_pu * period);  //

        if(pPWMData->cmpValue[pwmCnt] < pPWMData->minCMPValue)
        {
            pPWMData->cmpValue[pwmCnt] = pPWMData->minCMPValue;
        }

        // write the PWM data value
        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_A,
                                    pPWMData->cmpValue[pwmCnt]);

        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_B,
                                    pPWMData->cmpValue[pwmCnt]);
    }

    return;
} // end of HAL_writePWMData() function

//! \brief      Enables the PWM devices for motor control
//! \details    Turns on the outputs of the EPWM peripheral which will allow
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enablePWM(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(MOTOR1_DCLINKSS)
    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);
#else
    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);
#endif // MOTOR1_DCLINKSS

    obj->flagEnablePWM = true;

    return;
} // end of HAL_enablePWM() function

//! \brief      clear fault status of motor control
//! \details
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_clearMtrFaultStatus(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(MOTOR1_DCLINKSS)
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
#else
    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);
#endif // MOTOR1_DCLINKSS

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);

    return;
} // end of HAL_clearMtrFaultStatus() function

//! \brief      Disables the PWM device for motor control
//! \details    Turns off the outputs of the EPWM peripherals which will put
//!             the power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_disablePWM(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

    obj->flagEnablePWM = false;

    return;
} // end of HAL_disablePWM() function

//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
extern void HAL_setupPWMs(HAL_MTR_Handle handle);

//! \brief     Initializes the PWM count values
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//!
//! These values are calculated from parameters in user_mtr1.h and so need to
//! be initialized outside of the SysConfig generated code.
extern void HAL_initPWMCounts(HAL_MTR_Handle handle);

//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
static inline uint16_t HAL_getMtrTripFaults(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t tripFault = 0;

    tripFault = (EPWM_getTripZoneFlagStatus(obj->pwmHandle[0]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2)) |
                    (EPWM_getTripZoneFlagStatus(obj->pwmHandle[1]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2)) |
                    (EPWM_getTripZoneFlagStatus(obj->pwmHandle[2]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2));

    return(tripFault);
}

//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
extern void HAL_setMtrCMPSSDACValue(HAL_MTR_Handle handle,
                               const uint16_t dacValH, const uint16_t dacValL);

//! \brief     Set trigger point in the middle of the low side pulse
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] ignoreShunt  The low side shunt that should be ignored
//! \param[in] midVolShunt  The middle length of output voltage
static inline void HAL_setTrigger(HAL_MTR_Handle handle, HAL_PWMData_t *pPWMData,
                                  const SVGENCURRENT_IgnoreShunt_e ignoreShunt,
                                  const SVGENCURRENT_VmidShunt_e midVolShunt)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    int16_t pwmNum = midVolShunt;
    int16_t pwmCMPA = EPWM_getCounterCompareValue(obj->pwmHandle[pwmNum],
                                                   EPWM_COUNTER_COMPARE_A);

    int16_t pwmSOCCMP = 5;

    if(ignoreShunt == SVGENCURRENT_USE_ALL)
    {
        // Set up event source for ADC trigger
        EPWM_setADCTriggerSource(obj->pwmHandle[0],
                                 EPWM_SOC_A,
                                 EPWM_SOC_TBCTR_D_CMPC);
    }
    else
    {
        pwmSOCCMP = pwmCMPA - pPWMData->deadband - pPWMData->noiseWindow;

        if(pwmSOCCMP <= 0)
        {
            pwmSOCCMP = 5;

            // Set up event source for ADC trigger
            EPWM_setADCTriggerSource(obj->pwmHandle[0],
                                     EPWM_SOC_A,
                                     EPWM_SOC_TBCTR_U_CMPC);
        }
        else
        {
            pwmSOCCMP = 5;

            // Set up event source for ADC trigger
            EPWM_setADCTriggerSource(obj->pwmHandle[0],
                                     EPWM_SOC_A,
                                     EPWM_SOC_TBCTR_D_CMPC);
        }

    }

    //
    pPWMData->socCMP = pwmSOCCMP;

    // write the PWM data value  for ADC trigger
    EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                EPWM_COUNTER_COMPARE_C,
                                pwmSOCCMP);
    return;
} // end of HAL_setTrigger() function

#if defined(MOTOR1_DCLINKSS)
//! \brief     PWM phase shift and ADC SOC timing for dc_link current sensing
//! \details   PWM phase shift compensation based on each phase duty and min duration.
//!            dc_link current sampling point is calcuated with shifted PWM compare value.
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] dclinkHandle The dclink handle
//! \param[in] pVab_out     The pointer to the Vab voltage
//! \param[in] pPWMData     The pointer to the PWM data
static inline void
HAL_runSingleShuntCompensation(HAL_MTR_Handle handle,
                               DCLINK_SS_Handle dclinkHandle,
                               const MATH_vec2 *pVab_out,
                               HAL_PWMData_t *pPWMData, const float32_t Vdc_V)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    MATH_ui_vec2 upSoc, downSoc;
    MATH_ui_vec3 pwmCMPA, pwmCMPB;

    // get EPWMx CMPA/CMPB values
    pwmCMPA.value[0] = pwmCMPB.value[0] = pPWMData->cmpValue[0];
    pwmCMPA.value[1] = pwmCMPB.value[1] = pPWMData->cmpValue[1];
    pwmCMPA.value[2] = pwmCMPB.value[2] = pPWMData->cmpValue[2];

    // run PWM compensation for single-shunt
    DCLINK_SS_runPWMCompensation(dclinkHandle, pVab_out, Vdc_V,
                                 &pwmCMPA, &pwmCMPB, &upSoc, &downSoc);

#if(DMC_BUILDLEVEL >= DMC_LEVEL_2)
    uint16_t pwmCnt;

    for(pwmCnt=0; pwmCnt<3; pwmCnt++)
    {
        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_A,
                                    pwmCMPA.value[pwmCnt]);

        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_B,
                                    pwmCMPB.value[pwmCnt]);
    }
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

    // set the SOC trigger point for UP count
    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C,
                                upSoc.value[0]);

    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D,
                                upSoc.value[1]);

    // set the SOC trigger point for DOWN count
    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C,
                                downSoc.value[0]);

    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D,
                                downSoc.value[1]);

    return;
} // end of HAL_runSingleShuntCompensation() function

//! \brief     Set trigger point near period for dc-link current offset
//! \param[in] handle       The hardware abstraction layer (HAL) handle
static inline void HAL_setOffsetTrigger(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t pwmPRD = HAL_getTimeBasePeriod(handle);
    uint16_t offsetUpSoc = pwmPRD * (3.0f / 4.0f);
    uint16_t offsetDownSoc = pwmPRD * (1.0f / 4.0f);

    //
    // set the SOC trigger point for UP count
    //
    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C,
                                offsetUpSoc);

    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D,
                                offsetUpSoc);

    //
    // set the SOC trigger point for DOWN count
    //
    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C,
                                offsetDownSoc);

    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D,
                                offsetDownSoc);

    return;
} // end of HAL_setOffsetTrigger() function
#endif // MOTOR1_DCLINKSS

//! \brief     Set trigger point in the middle of the low side pulse
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] deadband     The setting deadband for mosfet gate driver
//! \param[in] noisewindow  The noise window
extern void HAL_setTriggerPrams(HAL_PWMData_t *pPWMData, const float32_t systemFreq_MHz,
                   const float32_t deadband_us, const float32_t noiseWindow_us);

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

#endif // end of HAL_H definition

