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
#include "sys_main.h"
#include "communication.h"

#if defined(CMD_CAN_EN)

#pragma CODE_SECTION(mcanISR, ".TI.ramfunc");

// **************************************************************************
// the globals

volatile CANCOM_Obj canComVars;

// **************************************************************************
// the functions

void CANCOM_enableCANInts(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    // This registers the interrupt handler in PIE vector table.
    //
    Interrupt_register(INT_MCANA_0, &mcanISR);

    // Enable transmit buffer interrupt
    MCAN_txBufTransIntrEnable(obj->mcanHandle, CANCOM_BUFFER_NUM, 1U);

    // enable the PIE interrupts associated with the CAN interrupts
    Interrupt_enable(INT_MCANA_0);

    return;
} // end of HAL_enableCANInts() function

//! \brief      Initializes CAN
//! \param[in]  N/A
void CANCOM_init(void)
{
    canComVars.speedConv_sf = 0.1f;          // uint16->float(unit=0.1Hz)
    canComVars.speedInv_sf = 10.0f;          // float->uint16(unit=0.1Hz)
    canComVars.currentConv_sf = 0.01f;       // uint16->float(unit=0.01A)
    canComVars.currentInv_sf = 100.0f;       // float->uint16(unit=0.01A)

    canComVars.txMsgCount = 0;                // for debug
    canComVars.rxMsgCount = 0;                // for debug

    canComVars.waitTimeCnt = 1000;            // 1s/1000ms
    canComVars.waitTimeDelay = 2;             // 2ms

    canComVars.flagTxDone = true;             // To enable CAN
    canComVars.flagRxDone = false;

    canComVars.speedRef_Hz = 0.0f;            // 0Hz
    canComVars.speedSet_Hz = 40.0f;           // 40Hz

    canComVars.flagCmdTxRun = false;
    canComVars.flagCmdRxRun = false;

    //
    // Initialize transmit message element
    //
    canComVars.txMsg.id = ((uint32_t)(CANCOM_TX_MSG_OBJ_ID) << 18);
    canComVars.txMsg.rtr = 0U;
    canComVars.txMsg.xtd = 0U;  // Standard 11-bit identifier
    canComVars.txMsg.esi = 0U;
    canComVars.txMsg.dlc = 8U;  // 8 bytes
    canComVars.txMsg.brs = 1U;  // Bit-rate switching enabled
    canComVars.txMsg.fdf = 1U;  // Frame transmitted in CAN FD format
    canComVars.txMsg.efc = 1U;
    canComVars.txMsg.mm = 0xAAU;

    //
    // Initialize receive message element
    //
    canComVars.rxMsg.id = 0U;
    canComVars.rxMsg.rtr = 0U;
    canComVars.rxMsg.xtd = 0U;
    canComVars.rxMsg.esi = 0U;
    canComVars.rxMsg.rxts = 0U; // Rx Timestamp
    canComVars.rxMsg.dlc = 0U;
    canComVars.rxMsg.brs = 0U;
    canComVars.rxMsg.fdf = 0U;
    canComVars.rxMsg.fidx = 0U; // Filter Index
                                // (of matching Rx acceptance filter element)
    canComVars.rxMsg.anmf = 0U; // Accepted Non-matching Frame

    return;
} // end of HAL_initCANInt() function

void CANCOM_updateCANCmdFreq(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;
    uint16_t canData = 0;

    if(canComVars.flagRxDone == true)
    {
        canComVars.flagCmdRxRun = (bool)(canComVars.rxMsg.data[0]);
        canComVars.motorStateRx = (MOTOR_Status_e)(canComVars.rxMsg.data[1]);

        canComVars.speedRef_Hz = ((float32_t)((canComVars.rxMsg.data[2] << 8) +
                canComVars.rxMsg.data[3])) * canComVars.speedConv_sf;

        canComVars.speedRx_Hz = ((float32_t)((canComVars.rxMsg.data[4] << 8) +
                canComVars.rxMsg.data[5])) * canComVars.speedConv_sf;

        canComVars.IqRx_A = ((float32_t)((canComVars.rxMsg.data[6] << 8) +
                canComVars.rxMsg.data[7])) * canComVars.currentConv_sf;

        canComVars.flagTxDone = true;
        canComVars.flagRxDone = false;
    }

    if((canComVars.flagTxDone == true) && (canComVars.waitTimeCnt == 0))
    {
        canComVars.txMsg.data[0] = (uint16_t)(canComVars.flagCmdTxRun);
        canComVars.txMsg.data[1] = (uint16_t)(objMtr->motorState);

        canData = (uint16_t)(canComVars.speedSet_Hz * canComVars.speedInv_sf);
        canComVars.txMsg.data[2] = (canData >> 8) & 0x00FF;
        canComVars.txMsg.data[3] = canData & 0x00FF;

        canData = (uint16_t)(objMtr->speedAbs_Hz * canComVars.speedInv_sf);
        canComVars.txMsg.data[4] = (canData >> 8) & 0x00FF;
        canComVars.txMsg.data[5] = canData & 0x00FF;

        canData = (uint16_t)(objMtr->Idq_in_A.value[1] * canComVars.currentInv_sf);

        canComVars.txMsg.data[6] = (canData >> 8) & 0x00FF;
        canComVars.txMsg.data[7] = canData & 0x00FF;

        //
        // Write message to Message RAM.
        //
        MCAN_writeMsgRam(halHandle->mcanHandle, MCAN_MEM_TYPE_BUF,
             CANCOM_BUFFER_NUM, (const MCAN_TxBufElement *)&canComVars.txMsg);

        //
        // Add transmission request for Tx buffer 0
        //
        MCAN_txBufAddReq(halHandle->mcanHandle, CANCOM_BUFFER_NUM);

        canComVars.waitTimeCnt = canComVars.waitTimeDelay;
        canComVars.flagTxDone = false;
    }

    if(canComVars.waitTimeCnt > 0)
    {
        canComVars.waitTimeCnt--;
    }

    return;
}

__interrupt void mcanISR(void)
{
    uint32_t status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    status = MCAN_getIntrStatus(halHandle->mcanHandle);

    //
    // Interrupt was caused by completed transmission
    //
    if((status & MCAN_INTR_SRC_TRANS_COMPLETE) != 0U)
    {
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        canComVars.txMsgCount++;

        // Since the message was sent, clear any error flags.
        canComVars.errorFlag = 0;
    }
    //
    // Interrupt was caused by a new message in the Rx FIFO
    //
    else if((status & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) != 0U)
    {
        // Get the received message
        MCAN_readMsgRam(halHandle->mcanHandle, MCAN_MEM_TYPE_FIFO, 0U,
                        MCAN_RX_FIFO_NUM_0, (MCAN_RxBufElement *)&canComVars.rxMsg);
        MCAN_writeRxFIFOAck(halHandle->mcanHandle, MCAN_RX_FIFO_NUM_0, 0);

        canComVars.rxMsgCount++;
        canComVars.flagRxDone = true;

        // Since the message was received, clear any error flags.
        canComVars.errorFlag = 0;
    }
    //
    // If something unexpected caused the interrupt, this would handle it.
    //
    else
    {
        // Spurious interrupt handling can go here.
        canComVars.errorFlag++;
    }

    //
    // Clear the interrupt flags for the MCAN interrupt line
    //
    MCAN_clearIntrStatus(halHandle->mcanHandle, status);
    MCAN_extTSWriteEOI(halHandle->mcanHandle);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

    return;
}

#endif // CMD_CAN_EN
//
//-- end of this file ----------------------------------------------------------
//
