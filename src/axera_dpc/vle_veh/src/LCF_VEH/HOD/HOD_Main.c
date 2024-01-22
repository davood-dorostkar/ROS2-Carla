// Copyright [2021] <Copyright Owner Senseauto>
#include "HOD_Ext.h"
#include "HOD.h"

real32_T HOD_TrqActMan;      /* '<Root>/HOD_TrqActMan' */
real32_T HOD_VelXVeh;        /* '<Root>/HOD_VelXVeh' */
boolean_T HOD_EnaActTorque;  /* '<Root>/HOD_EnaActTorque' */
uint8_T HOD_BtfControlFct;   /* '<Root>/HOD_BtfControlFct' */
uint8_T HOD_StActFctLevel;   /* '<Root>/HOD_StActFctLevel' */
uint8_T HOD_BtfAvailableFct; /* '<Root>/HOD_BtfAvailableFun' */
real32_T HOD_CycleTime;      /* '<Root>/HOD_CycleTime' */

void LCF_HOD_Reset(void) { HOD_initialize(); }

void LCF_HOD_Exec(const sHODInput_t* pHODInput,
                  const sHODParam_t* pHODParam,
                  sHODOutput_t* pHODOutput,
                  sHODDebug_t* pHODDebug) {
    // printf("--------------------HOD--------------------\n");
    /**************************************HOD
     * input***********************************************************/
    HOD_TrqActMan = pHODInput->HOD_TrqActMan;
    HOD_VelXVeh = pHODInput->HOD_VelXVeh;
    HOD_EnaActTorque = pHODInput->HOD_EnaActTorque;
    HOD_BtfControlFct = pHODInput->HOD_BtfControlFct;
    HOD_StActFctLevel = pHODInput->HOD_StActFctLevel;
    HOD_BtfAvailableFct = pHODInput->HOD_BtfAvailableFct;
    HOD_CycleTime = pHODInput->HOD_CycleTime_Sec;
    /************************************HOD
     * function*************************************************************/
    HOD_step();
    /************************************HOD
     * output*************************************************************/
    pHODOutput->HOD_EnaHandsOffCnfm = HOD_EnaHandsOffCnfm;
    pHODOutput->HOD_HandsOffAbuseWarn = HOD_HandsOffAbuseWarn;
    pHODOutput->HOD_RatDrvAttention = HOD_RatDrvAttention;
    pHODOutput->HOD_StEstHandTrq = HOD_StEstHandTrq;
    pHODOutput->HOD_StSysWarning = HOD_StSysWarning;
    /************************************HOD
     * debug*************************************************************/
    pHODDebug->HOD_CntEstHandTrq = HOD_CntEstHandTrq;
    pHODDebug->HOD_CntHandsOffLim = HOD_CntHandsOffLim;
    pHODDebug->HOD_EnaLeftInt = HOD_EnaLeftInt;
    pHODDebug->HOD_EnaLeftOff = HOD_EnaLeftOff;
    pHODDebug->HOD_EnaLeftOn = HOD_EnaLeftOn;
    pHODDebug->HOD_EnaRightInt = HOD_EnaRightInt;
    pHODDebug->HOD_EnaRightOff = HOD_EnaRightOff;
    pHODDebug->HOD_EnaRightOn = HOD_EnaRightOn;
    pHODDebug->HOD_TrqEstHandFlt = HOD_TrqEstHandFlt;
    pHODDebug->HOD_TrqEstHandStp = HOD_TrqEstHandStp;
    pHODDebug->HOD_TrqHandsOffLim = HOD_TrqHandsOffLim;
}
