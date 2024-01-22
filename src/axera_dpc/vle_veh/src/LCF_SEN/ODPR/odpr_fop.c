/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "odpr_fop_ext.h"
#include "tue_common_libs.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

// Calibration variable
const volatile boolean ODPR_Kb_FOPObjRelToRadar_bool =
    FALSE;  // Determines if the objects are expressed in the radar coordinate
            // system
const volatile boolean ODPR_Kb_FOPRearAxleCoord_bool =
    FALSE;  // Flag if the object data should be transformed to the rear axle
            // coordinate system
const volatile float32 ODPR_Kf_FOPMaxPosXDev_met =
    4.f;  // Threshold for object switch detection
const volatile float32 ODPR_Kf_FOPMaxPosYDev_met =
    1.2f;  // Threshold for PosY0 object switch detection
const volatile float32 ODPR_Kf_FOPPosStepDebounce_sec =
    0.6f;  // Debounce time after step has been detected for both PosX and PosY
           // input signal
const volatile uint8 ODPR_Kf_FOPMinObjQuality_perc =
    50u;  // Specify min obejct quality //P_ODPFOP_MinObjQualitiy_perc
const volatile float32 ODPR_Kf_FOPMinWdhObjHyst_met =
    0.2f;  // Hysteresis for minimum target object width
const volatile float32 ODPR_Kf_FOPMinWdhObj_met =
    1.4f;  // Minimum target object width //P_ODPFOP_MinWidthTgtObj_met
const volatile float32 ODPR_Kf_FOPMaxLongVel_mps =
    15.f;  // Maximum longitudinal object velocity
const volatile float32 ODPR_Kf_FOPMaxLongVelHyst_mps =
    5.f;  // Maximum longitudinal object velocity hysteresis
const volatile float32 ODPR_Kf_FOPMaxLatVelHyst_mps =
    0.5f;  // Maximum lateral object velocity hysteresis
const volatile float32 ODPR_Kf_FOPMaxLatVel_mps =
    2.f;  // Maximum lateral object velocity
const volatile float32 ODPR_Kf_FOPMaxLongAclHyst_mps2 =
    1.f;  // Maximum object longitudinal acceleration hysteresis
const volatile float32 ODPR_Kf_FOPMaxLongAcl_mps2 =
    10.f;  // Maximum valid longitudinal object acceleration
const volatile boolean ODPR_Kb_EnableLatMovFreeze_bool =
    FALSE;  // TRUE enables freeze of lateral object data (position,
            // velocity,acceleration)
const volatile float32 ODPR_Kf_FOPStandstillVelX_mps =
    0.5f;  // Below this standstill velocity threshold both Ego and Object
           // moving state is assumed
const volatile float32 ODPR_Kf_FOPDriveOffDelay_sec =
    0.5f;  // Time until moving state of both Objects and Ego vehicle is delayed
           // at transition
const volatile float32 ODPR_Kf_FOPDriveOffTolTime_sec =
    2.5f;  // Time after driveoff until new lateral object position should be
           // inside tolerance
const volatile float32 ODPR_Kf_FOPDriveOffVelReset_mps =
    5.5f;  // Velocity threshold to reset lateral movement freeze flag after
           // standstill
const volatile float32 ODPR_Kf_FOPObjSyncDelay_sec =
    0.25f;  // Latency time between ACC target object data and ego VDY input
            // data
const volatile boolean ODPR_Kb_FOPUseEstiPosY4Yaw_bool =
    FALSE;  // If TRUE the internally estimated PosY is used with respect to
            // RelVelY and a constant
const volatile float32 ODPR_Kf_FOPObjPosYPT1Time4Yaw_sec =
    0.1f;  // PT1 time constant for PosY which is only used for object yaw angle
           // calculation

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
STATIc float32
    FOP_fPreCycAccObjTimeStamp_sec;  // Acc object timestamp in previous cycle
STATIc boolean FOP_bPreCycObjDetected_bool;  // Check if the object is detected
                                             // in previous cycle
STATIc float32 FOP_fPreCycAccObjPosX_met;  // Acc object PosX in previous cycle
STATIc float32 FOP_fPreCycObjPosY_met;     // Acc object PosY in previous cycle,
                                           // used in PosY step check
STATIc sint8 FOP_uiPreCycObjID_nu;         // Acc object ID in previous cycle
STATIc boolean FOP_bPreCycAccObjDetected_bool;  // Check if the Acc object is
                                                // detected in previous cycle
STATIc float32 FOP_fPosXTurnOffDelay_sec;       // Global time for PosX turn off
                                                // delay
STATIc float32 FOP_fPosYTurnOffDelay_sec;       // Global time for PosY turn off
                                                // delay
STATIc boolean
    FOP_bPreCycPosStep_bool;  // PosX or PosY step detected in previous cycle
STATIc boolean
    FOP_bPreCycNoObjIDSwitch_bool;  // No object ID switch in previous cycle
STATIc float32
    FOP_fObjTypeTurnOffDelay_sec;  // Global time for object type turn off delay
STATIc float32 FOP_fObjDetTurnOffDelay_sec;  // Global time for object detected
                                             // turn off delay
STATIc boolean FOP_bPreCycObjLatMovInvalid_bool;  // Flag that object latral
                                                  // moving invalid in previous
                                                  // cycle
STATIc float32 FOP_fPreCycAccObjPosY_met;  // Acc object PosY in previous cycle,
                                           // used in LatMovInvalid check
STATIc float32
    FOP_fObjStopTurnOffDelay_sec;  // Global time for object stop turn off delay
STATIc float32
    FOP_fEgoStopTurnOffDelay_sec;  // Global time for ego stop turn off delay
STATIc float32 FOP_fBothStopTurnOffDelay_sec;  // Global time for object and ego
                                               // stop turn off delay
STATIc boolean FOP_bPreCycPosXHyst_bool;  // Hysteresis output of PosY check
STATIc boolean
    FOP_bPreCycEnableSample_bool;  // Enable sample falg in previous cycle
STATIc float32 FOP_fPreCycYawAngLowPass_rad;  // Yaw angle lowpass filter output
                                              // in previous cycle
STATIc float32 FOP_fPosXIntegrator_met;       // PosX integrator output
STATIc float32 FOP_fPosYLowPass_met;          // PosY lowpass filter out
STATIc float32 FOP_fVehYawIntegrator_rad;     // Veh Yaw angle integrator output
STATIc float32 FOP_fEnableCalcTimer_sec;      // Enable calc timer
STATIc float32 FOP_fPreCycPosY_met;           // PosY in previous cycle
STATIc float32 FOP_fYawAngCalc_rad;           // Calcualted Yaw angle
STATIc float32
    FOP_fFusionTurnOnDelay_sec;  // Global time for fusion state turn on delay
STATIc boolean FOP_bPreCycFusionRS_bool;    // fusion state RS flip-flop
STATIc float32 FOP_fMeasStTurnOnDelay_sec;  // Global time for measurement state
                                            // turn on delay
STATIc boolean FOP_bPreCycMeasStRS_bool;    // measurement state RS flip-flop
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* Lookup table */
// {14.f, 0.f, 2.8f, 5.6f, 8.3f, 11.1f, 13.9f, 16.7f, 19.4f, 22.2f, 25.f, 27.8f,
// 30.6f, 33.3f, 36.1f}                    //X_ODPFOP_LongDistMaxVel_mps
// {20.f, 25.f, 40.f, 45.f, 48.f, 50.f, 50.f, 50.f, 50.f, 50.f, 50.f, 50.f, 50.f,
// 50.f}                                  //Y_ODPFOP_LongDistMax_met
STATIc TUE_CML_Vector2D_t FOP_sLongDistMax[14] =  // Maximum longitudinal
                                                  // distance for object
                                                  // validation
    {{0.f, 20.f},   {2.8f, 25.f},  {5.6f, 40.f},  {8.3f, 45.f},  {11.1f, 48.f},
     {13.9f, 50.f}, {16.7f, 50.f}, {19.4f, 50.f}, {22.2f, 50.f}, {25.f, 50.f},
     {27.8f, 50.f}, {30.6f, 50.f}, {33.3f, 50.f}, {36.1f, 50.f}};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname:    FOPReset                                        */ /*!

                                @brief           Reset function of FOP

                                @description     All global variables related to
                              FOP are
                              reset in
                              this function
                                                 when FOP executes for the first
                              time, or
                              system
                              exception needs
                                                 to be reset

                                @param[in]       none

                                @return          none
                              *****************************************************************************/
void ODPR_FOP_Reset(void) {
    FOP_fPreCycAccObjTimeStamp_sec = 0.f;
    FOP_bPreCycObjDetected_bool = FALSE;
    FOP_fPreCycAccObjPosX_met = 0.f;
    FOP_fPreCycObjPosY_met = 0.f;
    FOP_uiPreCycObjID_nu = -1;
    FOP_bPreCycAccObjDetected_bool = FALSE;
    FOP_fPosXTurnOffDelay_sec = 0.f;
    FOP_fPosYTurnOffDelay_sec = 0.f;
    FOP_bPreCycPosStep_bool = TRUE;
    FOP_bPreCycNoObjIDSwitch_bool = FALSE;
    FOP_fObjTypeTurnOffDelay_sec = 0.f;
    FOP_fObjDetTurnOffDelay_sec = 0.f;
    FOP_bPreCycObjLatMovInvalid_bool = FALSE;
    FOP_fPreCycAccObjPosY_met = 0.f;
    FOP_fObjStopTurnOffDelay_sec = 0.f;
    FOP_fEgoStopTurnOffDelay_sec = 0.f;
    FOP_fBothStopTurnOffDelay_sec = 0.f;
    FOP_bPreCycPosXHyst_bool = FALSE;
    FOP_bPreCycEnableSample_bool = FALSE;
    FOP_fPreCycYawAngLowPass_rad = 0.f;
    FOP_fPosXIntegrator_met = 0.f;
    FOP_fPosYLowPass_met = 0.f;
    FOP_fVehYawIntegrator_rad = 0.f;
    FOP_fEnableCalcTimer_sec = 0.f;
    FOP_fPreCycPosY_met = 0.f;
    FOP_fYawAngCalc_rad = 0.f;
    FOP_fFusionTurnOnDelay_sec = 0.f;
    FOP_bPreCycFusionRS_bool = FALSE;
    FOP_fMeasStTurnOnDelay_sec = 0.f;
    FOP_bPreCycMeasStRS_bool = FALSE;
}

/*****************************************************************************
  Functionname:    FOPTimeConversion */ /*!

@brief           Timestamp check and conversion

@description     Convert timestamp's unit to second from microsecond, meanwhile
    check if the input timestamp skips some cycles and makes sure it
    shall increase continuously. This function is corresponding to
    TimestampConversion module in MBD.

@param[in]       pACCObjectData  Information of OOI_NEXT obejct from ACC module
@param[in]       pSystemPara     System parameter
@param[in,out]   pFOPOutput      Validity check results output by FOP

@return          none

@uml
@startuml
start
:Initialize FOP_fPreCycAccObjTimeStamp_sec;
note:Define a global variable to store fAccObjTimeStamp_sec\n in previous cycle,
which initial value is set to 0
if (time lag between two cycles > 0.99*SystemCycleTime?) then (yes)
:Output timestamp of ACC object\n(After unit conversion);
note:Normally output
else (no)
:Output the sum of timestamp of ACC object in previous cycle\n and system
cycle time;
note right:Make sure timestamp increases continously
endif
:Reassign FOP_fPreCycAccObjTimeStamp_sec;
note:Which is used in next cycle
stop
@enduml
*****************************************************************************/
STATIc void FOPTimeConversion(const ODPRInAccFRObj_t* pACCObjectData,
                              const ODPRInSystemPara_t* pSystemPara,
                              ODPRFOPOut_t* pFOPOutput) {
    float32 fTempAccObjTimeStamp_sec;

    fTempAccObjTimeStamp_sec = pACCObjectData->uiObjTimeStamp_usec * 1.0e-06f;

    /* Check if the input timestamp skips some cycles */
    if (fTempAccObjTimeStamp_sec - FOP_fPreCycAccObjTimeStamp_sec >
        pSystemPara->fSystemCylceTime_sec * 0.99f) {
        pFOPOutput->fAccObjTimeStamp_sec = fTempAccObjTimeStamp_sec;
    } else {
        pFOPOutput->fAccObjTimeStamp_sec =
            pSystemPara->fSystemCylceTime_sec + FOP_fPreCycAccObjTimeStamp_sec;
    }

    /* Reassign value of FOP_fPreCycAccObjTimeStamp_sec, which is used in next
     * cycle*/
    FOP_fPreCycAccObjTimeStamp_sec = pFOPOutput->fAccObjTimeStamp_sec;
}

/*****************************************************************************
  Functionname:    FOPTransObjXPos */ /*!

@brief           Transfrom object X position

@description     Check to which coordinate system the object X position should
          be transformed. This function is corresponding to
          TransformObjectXposiiton module in MBD.

@param[in]       pACCObjectData  Information of OOI_NEXT obejct from ACC module
@param[in,out]   pFOPOutput      Validity check results output by FOP

@return          none

@uml
@startuml
start
if (ODPR_Kb_FOPObjRelToRadar_bool is true) then (yes)
if (ODPR_Kb_FOPRearAxleCoord_bool is true) then (yes)
   :Output the sum of fObjPosX_met,\n Wheelbase and Overhang;
   note: Object is expressed in radar coordinate\n and the center of rear
axle is vehicle\n coordinate system origin
else (no)
   :Output the sum of fObjPosX_met\n and Overhang;
   note right: Object is expressed in radar coordinate\n and the center of
front axle is vehicle\n coordinate system origin
endif
else (no)
:Output fObjPosX_met;
note right: Object is expressed in\n vehicle coordinate
endif

stop
@enduml
*****************************************************************************/
STATIc void FOPTransObjXPos(const ODPRInAccFRObj_t* pACCObjectData,
                            const ODPRParam_t* pParam,
                            ODPRFOPOut_t* pFOPOutput) {
    float32 fVehCoordCompensation;

    /* Vehicle coordinate compensation*/
    if (ODPR_Kb_FOPObjRelToRadar_bool)  // Check if the object is expressed in
                                        // the radar coordinate system
    {
        fVehCoordCompensation =
            pParam->fVehOverhangFront_met +
            (ODPR_Kb_FOPRearAxleCoord_bool ? pParam->fVehWheelBase_met : 0.f);
        pFOPOutput->fAccObjPosX_met =
            pACCObjectData->fObjPosX_met + fVehCoordCompensation;
    } else {
        pFOPOutput->fAccObjPosX_met = pACCObjectData->fObjPosX_met;
    }
}

/*****************************************************************************
  Functionname:    FOPChkPossHysteresis */ /*!

@brief           Check for possible hysteresis

@description     Check whether the ID, X position and Y position of object have
jumped.
This function is corresponding to CheckForPossibleHysteresis module
in MBD.

@param[in]       pACCObjectData  Information of OOI_NEXT obejct from ACC module
@param[in]       pSystemPara     System parameter
@param[in]       pFOPOutput      Validity check results output by FOP
@param[in,out]   pFOPDebug       Debug output of FOP

@return          sChkPossiHys                         A structure contains check
flags
bNoObjIDSwitchDetected_bool      Flag that no object ID switch is
detected
bObjHysteresisAllowed_bool       Flag that object hysteresis is
allowed
bObjPosXStepDetected_bool        Flag that objcet position X step
is detected
bObjPosYStepDetected_bool        Flag that objcet position Y step
is detected
bResetObjValidity_bool           Flag that reset object validity

@uml
@startuml
start
partition NoObjIDSwitchDetected {
if (Object ID is same as previous cycle) then (yes)
:Flag of NoObjIDSwitchDetected \nis TRUE;
else (no)
:Flag of NoObjIDSwitchDetected \nis FALSE;
endif
note: Output flag that no object \nID switch is detected;
}
partition ObjPosXStepDetected {
if (PosX variation is greater than threshold) then (yes)
:Flag of ObjPosXStepDetected \nis TRUE;
else (no)
if (Duration reaches the threshold) then (yes)
:Flag of ObjPosXStepDetected \nis FALSE;
else (no)
:Flag of ObjPosXStepDetected \nis TRUE;
endif
endif
note:Output flag that object \nposition X step is detected;
}
partition ObjPosYStepDetected {
if (PosY variation is greater than threshold) then (yes)
:Flag of ObjPosYStepDetected \nis TRUE;
else (no)
if (Duration reaches the threshold) then (yes)
:Flag of ObjPosYStepDetected \nis FALSE;
else (no)
:Flag of ObjPosYStepDetected \nis TRUE;
endif
endif
note:Output flag that object \nposition Y step is detected;
}
partition ObjHysteresisAllowed {
if (Object is detected in previous &\nNo ID switch is detected &\nAt most one
position step is detected) then (yes)
:Flag of ObjHysteresisAllowed \nis TRUE;
else (no)
:Flag of ObjHysteresisAllowed \nis FALSE;
endif
note:Output flag that object \nhysteresis is allowed;
}
partition ResetObjValidity {
if (Position X step is detected || \nPosition Y step is detected ||\nObject ID
switch is detected) then (yes)
:Flag of ResetObjValidity \nis TRUE;
else (no)
:Flag of ResetObjValidity \nis FALSE;
endif
note:Output flag that \nreset object validity;
}
stop
@enduml
*****************************************************************************/
STATIc FOPChkPossiHys_t
FOPChkPossHysteresis(const ODPRInAccFRObj_t* pACCObjectData,
                     const ODPRInSystemPara_t* pSystemPara,
                     const ODPRFOPOut_t* pFOPOutput,
                     ODPRFOPDebug_t* pFOPDebug) {
    FOPChkPossiHys_t sChkPossiHys = {0};  // result value
    boolean bObjPosXChk_bool;
    boolean bObjPosYChk_bool;
    boolean bNoIDSwitch_bool;

    /* Calculate the position difference between current cycle and pre cycle */
    float32 fTempAccObjPosX_met =
        pFOPOutput->fAccObjPosX_met - FOP_fPreCycAccObjPosX_met;
    float32 fTempObjPosY_met =
        pACCObjectData->fObjPosY_met - FOP_fPreCycObjPosY_met;

    /* ID and X/Y position check */
    bObjPosXChk_bool =
        (TUE_CML_Abs(fTempAccObjPosX_met) > ODPR_Kf_FOPMaxPosXDev_met);
    bObjPosYChk_bool =
        (TUE_CML_Abs(fTempObjPosY_met) > ODPR_Kf_FOPMaxPosYDev_met);
    bNoIDSwitch_bool = (pACCObjectData->uiObjID_nu == FOP_uiPreCycObjID_nu);

    /* Output flag that no object ID switch is detected */
    sChkPossiHys.bNoObjIDSwitchDetected_bool = bNoIDSwitch_bool;

    /* Output flag that object hysteresis is allowed */
    sChkPossiHys.bObjHysteresisAllowed_bool =
        (!(bObjPosXChk_bool && bObjPosYChk_bool)) &&
        FOP_bPreCycObjDetected_bool && bNoIDSwitch_bool;

    /* Output flag that objcet position X step is detected */
    sChkPossiHys.bObjPosXStepDetected_bool = BASICTurnOffDelay(
        (bObjPosXChk_bool && bNoIDSwitch_bool), ODPR_Kf_FOPPosStepDebounce_sec,
        pSystemPara->fSystemCylceTime_sec, &FOP_fPosXTurnOffDelay_sec);

    /* Output flag that objcet position Y step is detected */
    sChkPossiHys.bObjPosYStepDetected_bool = BASICTurnOffDelay(
        (bObjPosYChk_bool && bNoIDSwitch_bool), ODPR_Kf_FOPPosStepDebounce_sec,
        pSystemPara->fSystemCylceTime_sec, &FOP_fPosYTurnOffDelay_sec);

    /* Output flag that reset object validity */
    sChkPossiHys.bResetObjValidity_bool =
        TUE_CML_RisingEdgeSwitch((sChkPossiHys.bObjPosXStepDetected_bool ||
                                  sChkPossiHys.bObjPosYStepDetected_bool),
                                 &FOP_bPreCycPosStep_bool) ||
        TUE_CML_FallingEdgeSwitch(bNoIDSwitch_bool,
                                  &FOP_bPreCycNoObjIDSwitch_bool);

    /* Reassign value, which is used in next cycle*/
    FOP_fPreCycAccObjPosX_met = pFOPOutput->fAccObjPosX_met;
    FOP_fPreCycObjPosY_met = pACCObjectData->fObjPosY_met;
    FOP_uiPreCycObjID_nu = pACCObjectData->uiObjID_nu;

    /* Debug output */
    pFOPDebug->bNoObjIDSwitchDetected_bool =
        sChkPossiHys.bNoObjIDSwitchDetected_bool;
    pFOPDebug->bObjHysteresisAllowed_bool =
        sChkPossiHys.bObjHysteresisAllowed_bool;
    pFOPDebug->bObjPosXStepDetected_bool =
        sChkPossiHys.bObjPosXStepDetected_bool;
    pFOPDebug->bObjPosYStepDetected_bool =
        sChkPossiHys.bObjPosYStepDetected_bool;
    pFOPDebug->bResetObjValidity_bool = sChkPossiHys.bResetObjValidity_bool;
    pFOPDebug->fTempAccObjPosX_met = fTempAccObjPosX_met;
    pFOPDebug->fTempObjPosY_met = fTempObjPosY_met;
    pFOPDebug->bObjPosXChk_bool = bObjPosXChk_bool;
    pFOPDebug->bObjPosYChk_bool = bObjPosYChk_bool;
    pFOPDebug->bNoIDSwitch_bool = bNoIDSwitch_bool;

    return sChkPossiHys;
}

/*****************************************************************************
  Functionname:    FOPValidateACCObject */ /*!

@brief           Validate ACC object

@description     Validate ACC obejct attributes, including obejct quality,
position,velocity,width,acceleration type,state,etc.
This function is corresponding to ValidateACCObject module
in MBD.

@param[in]       pACCObjectData  Information of OOI_NEXT obejct from ACC module
@param[in]       pChkPossiHys    Object ID switch and position step check flags
@param[in]       pSystemPara     System parameter
@param[in]       pEgoVEDData     Dynamic data of ego vehicle  from VED module
@param[in,out]   pFOPOutput      Validity check results output by FOP
@param[in,out]   pFOPDebug       Debug output of FOP

@return          sValidACCObj                          A structure contains
check flags
bACCObjDetected_bool             Flag that ACC obejct is
detected
bObjQualityValid_bool            Flag that object quality is
valid
bObjMeasStateValid_bool          Flag that object measurement
state is valid
bObjPosYValid_bool               Flag that object position Y is
valid
bObjPosXValid_bool               Flag that object position X is
valid
bObjWidthValid_bool              Flag that obejct width is valid
bObjRelVelXValid_bool            Flag that X-axis relative
velocity is valid
bObjRelVelYValid_bool            Flag that Y-axis relative
velocity is valid
bObjRelAclXValid_bool            Flag that X-axis relative
acceleration is valid
bObjTypeValid_bool               Flag that object type is valid
bObjFusionStateValid_bool        Flag that object fusion state
is valid
bObjLatMovInvalid_bool           Flag that object latent moving
is invalid

@uml
@startuml
start
partition FOPValidateACCObject {
fork
:Validate object\nquality;
fork again
:Validate object\nposition Y;
fork again
:Validate object\nposition X;
fork again
:Validate object\nwidth;
fork again
:Validate X-axis\nrelative velocity;
fork again
:Validate Y-axis\nrelative velocity;
fork again
:Validate X-axis\nrelative acceleration;
fork again
:Validate object\ntype;
fork again
:Validate object\nfusion state;
fork again
:Validate object\ndetection;
fork again
:Validate object\nmeasurement state;
fork again
:Validate object\nlatent moving state;
end fork
}
stop
@enduml
*****************************************************************************/
STATIc FOPValidACCObj_t
FOPValidateACCObject(const ODPRInAccFRObj_t* pACCObjectData,
                     const FOPChkPossiHys_t* pChkPossiHys,
                     const ODPRInSystemPara_t* pSystemPara,
                     const ODPRInVEDVehDyn_t* pEgoVEDData,
                     const ODPRParam_t* pParam,
                     ODPRFOPOut_t* pFOPOutput,
                     ODPRFOPDebug_t* pFOPDebug) {
    /* VARIABLES */
    FOPValidACCObj_t sValidACCObj = {0};  // result value
    float32 fAbsObjPosY_met;
    float32 fSqrAccObjPosX_met2;
    float32 fOffsetObjPosY_met;
    float32 fMinObjPosY_met = 0.f;
    float32 fMinCurvature_1pm;
    float32 fDistYOffset_met;
    boolean bDistYOffsetChk_bool = FALSE;
    float32 fMinWidthTgtObj_met;
    float32 fMaxLongVel_met;
    float32 fMaxLatVel_met;
    float32 fMaxLongAcl_met;
    boolean bTempObjTypeChk_bool;
    boolean bTempObjTypeChkWithHyst_bool;
    boolean bTempFusionCamChk_bool;
    boolean bTempFusionRadChk_bool;
    boolean bTempObjAndEgoStop_bool;
    float32 fTempObjVelX_mps;
    boolean bTempObjStandstill_bool;
    boolean bTempEgoStandstill_bool;
    boolean bTempLatMovReset_bool;
    boolean bTempBothStop_bool;
    float32 fTempObjPosX_met;
    float32 fTempLongDistMax_met;
    boolean bTempPosXHyst_bool;
    boolean bTempFusionTurnOn_bool;
    boolean bTempFusionReset_bool;
    boolean bTempMeasStMeas_bool;
    boolean bTempMeasStPred_bool;
    boolean bTempMeasStTurnOn_bool;
    boolean bTempMeasStReset_bool;

    /* Validate object quality */
    sValidACCObj.bObjQualityValid_bool =
        (pACCObjectData->uiObjQuality_perc >= ODPR_Kf_FOPMinObjQuality_perc);

    /* Validate object position Y */
    if (pChkPossiHys->bObjPosYStepDetected_bool) {
        sValidACCObj.bObjPosYValid_bool = FALSE;
    } else {
        fAbsObjPosY_met = TUE_CML_Abs(pACCObjectData->fObjPosY_met);
        fSqrAccObjPosX_met2 = TUE_CML_Sqr(pFOPOutput->fAccObjPosX_met);
        fOffsetObjPosY_met =
            0.5f * pEgoVEDData->fEgoCurve_1pm * fSqrAccObjPosX_met2 -
            pACCObjectData->fObjPosY_met;
        fMinObjPosY_met =
            TUE_CML_Min(TUE_CML_Abs(fOffsetObjPosY_met), fAbsObjPosY_met);

        if (pChkPossiHys->bObjHysteresisAllowed_bool) {
            fMinCurvature_1pm = TUE_CML_Min(
                FOP_MAX_STEER_ANGLE_RAD / SafeDiv(pParam->fVehWheelBase_met),
                FOP_MAX_LAT_ACL_MPS2 /
                    SafeDiv(TUE_CML_Sqr(pEgoVEDData->fEgoVelX_mps)));
            fDistYOffset_met = 0.5f * fMinCurvature_1pm * fSqrAccObjPosX_met2 *
                                   FOP_FAC_MAX_DIST_Y_ACTIVE_NU +
                               FOP_DIST_Y_MAX_CRV_OFFSET_MET;
            bDistYOffsetChk_bool = (fAbsObjPosY_met < fDistYOffset_met);

            sValidACCObj.bObjPosYValid_bool =
                bDistYOffsetChk_bool ||
                ((fMinObjPosY_met <
                  (FOP_MAX_DIST_Y_HYST_MET + FOP_MAX_DIST_Y_MET)));
        } else {
            sValidACCObj.bObjPosYValid_bool =
                (fMinObjPosY_met < FOP_MAX_DIST_Y_MET);
        }

        sValidACCObj.bObjPosYValid_bool =
            TRUE;  // WJTest: set 'bObjPosYValid_bool' to true forcibly
    }

    /* Validate object position X */
    if (ODPR_Kb_FOPObjRelToRadar_bool)  // Check if the object is expressed in
                                        // the radar coordinate system
    {
        fTempObjPosX_met =
            pParam->fVehOverhangFront_met +
            (ODPR_Kb_FOPRearAxleCoord_bool ? pParam->fVehWheelBase_met : 0.f);
    } else {
        fTempObjPosX_met = 0.f;
    }
    fTempObjPosX_met = pFOPOutput->fAccObjPosX_met - fTempObjPosX_met;
    fTempLongDistMax_met =
        TUE_CML_CalculatePolygonValue2D(14, FOP_sLongDistMax,
                                        pEgoVEDData->fEgoVelX_mps) +
        50.f;
    bTempPosXHyst_bool = TUE_CML_HysteresisFloat(
        fTempObjPosX_met, fTempLongDistMax_met,
        (fTempLongDistMax_met - FOP_LONG_DIST_MAX_HYST_MET),
        &FOP_bPreCycPosXHyst_bool);

    sValidACCObj.bObjPosXValid_bool =
        ((!(pChkPossiHys->bObjPosXStepDetected_bool || bTempPosXHyst_bool)) &&
         (fTempObjPosX_met > 0.1f));

    /* Validate object width */
    if (pChkPossiHys->bObjHysteresisAllowed_bool) {
        fMinWidthTgtObj_met =
            ODPR_Kf_FOPMinWdhObj_met - ODPR_Kf_FOPMinWdhObjHyst_met;
    } else {
        fMinWidthTgtObj_met = ODPR_Kf_FOPMinWdhObj_met;
    }
    sValidACCObj.bObjWidthValid_bool =
        (pACCObjectData->fObjWidth_met > fMinWidthTgtObj_met);

    /* Validate X-axis relative velocity */
    if (pChkPossiHys->bObjHysteresisAllowed_bool) {
        fMaxLongVel_met =
            ODPR_Kf_FOPMaxLongVel_mps + ODPR_Kf_FOPMaxLongVelHyst_mps;
    } else {
        fMaxLongVel_met = ODPR_Kf_FOPMaxLongVel_mps;
    }
    sValidACCObj.bObjRelVelXValid_bool =
        (TUE_CML_Abs(pACCObjectData->fObjRelVelX_mps) < fMaxLongVel_met);

    /* Validate Y-axis relative velocity */
    if (pChkPossiHys->bObjHysteresisAllowed_bool) {
        fMaxLatVel_met =
            ODPR_Kf_FOPMaxLatVel_mps + ODPR_Kf_FOPMaxLatVelHyst_mps;
    } else {
        fMaxLatVel_met = ODPR_Kf_FOPMaxLatVel_mps;
    }
    sValidACCObj.bObjRelVelYValid_bool =
        (TUE_CML_Abs(pACCObjectData->fObjRelVelY_mps) < fMaxLatVel_met);

    /* Validate X-axis relative acceleration */
    if (pChkPossiHys->bObjHysteresisAllowed_bool) {
        fMaxLongAcl_met =
            ODPR_Kf_FOPMaxLongAcl_mps2 + ODPR_Kf_FOPMaxLongAclHyst_mps2;
    } else {
        fMaxLongAcl_met = ODPR_Kf_FOPMaxLongAcl_mps2;
    }
    sValidACCObj.bObjRelAclXValid_bool =
        (TUE_CML_Abs(pACCObjectData->fObjRelAclX_mps2) < fMaxLongAcl_met);

    /* Validate object type */
    bTempObjTypeChk_bool =
        ((pACCObjectData->uiObjClassType_nu == (uint8)ACC_OC_CAR) ||
         (pACCObjectData->uiObjClassType_nu == (uint8)ACC_OC_TRUCK));
    bTempObjTypeChkWithHyst_bool = BASICTurnOffDelay(
        bTempObjTypeChk_bool, FOP_OBJ_TYPE_DEBOUNCE_TIME_SEC,
        pSystemPara->fSystemCylceTime_sec, &FOP_fObjTypeTurnOffDelay_sec);

    sValidACCObj.bObjTypeValid_bool = pChkPossiHys->bObjHysteresisAllowed_bool
                                          ? bTempObjTypeChkWithHyst_bool
                                          : bTempObjTypeChk_bool;

    /* Validate object fusion state */
    bTempFusionCamChk_bool = ((pACCObjectData->uiObjSensorSource_btf &
                               FOP_FUSION_M_CAM_FRONT_NU) > 0u);
    bTempFusionRadChk_bool = ((pACCObjectData->uiObjSensorSource_btf &
                               FOP_FUSION_RADAR_FRONT_NU) > 0u);
    bTempFusionTurnOn_bool = BASICTurnOnDelay(
        (bTempFusionCamChk_bool && bTempFusionRadChk_bool),
        FOP_MIN_DUR_CAM_FUS_BEF_BRID_SEC, pSystemPara->fSystemCylceTime_sec,
        &FOP_fFusionTurnOnDelay_sec);

    bTempFusionReset_bool =
        pChkPossiHys->bResetObjValidity_bool ||
        (!(bTempFusionCamChk_bool || bTempFusionRadChk_bool));

    sValidACCObj.bObjFusionStateValid_bool =
        TUE_CML_RSFlipFlop(bTempFusionTurnOn_bool, bTempFusionReset_bool,
                           &FOP_bPreCycFusionRS_bool);

    /* Validate object detection */
    sValidACCObj.bACCObjDetected_bool = BASICTurnOffDelay(
        pACCObjectData->bObjectDetected_bool, FOP_OBJ_LOSS_DEBOUNCE_T_SEC,
        pSystemPara->fSystemCylceTime_sec, &FOP_fObjDetTurnOffDelay_sec);

    /* Validate object measurement state */
    bTempMeasStMeas_bool =
        (pACCObjectData->uiObjMeasState_nu == (uint8)STATE_MEASURED);
    bTempMeasStPred_bool =
        (pACCObjectData->uiObjMeasState_nu == (uint8)STATE_PREDICTED);
    bTempMeasStTurnOn_bool = BASICTurnOnDelay(
        bTempMeasStMeas_bool, FOP_MIN_MEAS_STATE_FOR_PRED_SEC,
        pSystemPara->fSystemCylceTime_sec, &FOP_fMeasStTurnOnDelay_sec);

    bTempMeasStReset_bool = pChkPossiHys->bResetObjValidity_bool ||
                            (!(bTempMeasStMeas_bool || bTempMeasStPred_bool));

    sValidACCObj.bObjMeasStateValid_bool =
        TUE_CML_RSFlipFlop(bTempMeasStTurnOn_bool, bTempMeasStReset_bool,
                           &FOP_bPreCycMeasStRS_bool);

    /* Validate object latent moving state*/
    if (ODPR_Kb_EnableLatMovFreeze_bool) {
        fTempObjVelX_mps =
            pEgoVEDData->fEgoVelX_mps + pACCObjectData->fObjRelVelX_mps;
        bTempObjStandstill_bool =
            (fTempObjVelX_mps < ODPR_Kf_FOPStandstillVelX_mps);
        bTempEgoStandstill_bool =
            (pEgoVEDData->fEgoVelX_mps < ODPR_Kf_FOPStandstillVelX_mps);
        bTempObjAndEgoStop_bool =
            BASICTurnOffDelay(bTempObjStandstill_bool,
                              ODPR_Kf_FOPDriveOffDelay_sec,
                              pSystemPara->fSystemCylceTime_sec,
                              &FOP_fObjStopTurnOffDelay_sec) &&
            BASICTurnOffDelay(bTempEgoStandstill_bool,
                              ODPR_Kf_FOPDriveOffDelay_sec,
                              pSystemPara->fSystemCylceTime_sec,
                              &FOP_fEgoStopTurnOffDelay_sec);

        bTempBothStop_bool = BASICTurnOffDelay(
            bTempObjAndEgoStop_bool, ODPR_Kf_FOPDriveOffTolTime_sec,
            pSystemPara->fSystemCylceTime_sec, &FOP_fBothStopTurnOffDelay_sec);

        bTempLatMovReset_bool =
            ((!pACCObjectData->bObjectDetected_bool) ||
             (!pChkPossiHys->bNoObjIDSwitchDetected_bool) ||
             (bTempBothStop_bool && (TUE_CML_Abs(pACCObjectData->fObjPosY_met -
                                                 FOP_fPreCycAccObjPosY_met) <
                                     ODPR_Kf_FOPMaxPosYDev_met)) ||
             ((pEgoVEDData->fEgoVelX_mps > ODPR_Kf_FOPDriveOffVelReset_mps) &&
              (fTempObjVelX_mps > ODPR_Kf_FOPDriveOffVelReset_mps)));

        if (bTempLatMovReset_bool) {
            sValidACCObj.bObjLatMovInvalid_bool = FALSE;
        } else {
            sValidACCObj.bObjLatMovInvalid_bool =
                bTempObjAndEgoStop_bool || FOP_bPreCycObjLatMovInvalid_bool;
        }
    } else {
        sValidACCObj.bObjLatMovInvalid_bool = FALSE;
    }

    /* Reassign value, which is used in next cycle*/
    FOP_bPreCycObjLatMovInvalid_bool = sValidACCObj.bObjLatMovInvalid_bool;

    /* Debug output */
    pFOPDebug->bACCObjDetected_bool = sValidACCObj.bACCObjDetected_bool;
    pFOPDebug->bObjQualityValid_bool = sValidACCObj.bObjQualityValid_bool;
    pFOPDebug->bObjMeasStateValid_bool = sValidACCObj.bObjMeasStateValid_bool;
    pFOPDebug->bObjPosYValid_bool = sValidACCObj.bObjPosYValid_bool;
    pFOPDebug->bObjPosXValid_bool = sValidACCObj.bObjPosXValid_bool;
    pFOPDebug->bObjWidthValid_bool = sValidACCObj.bObjWidthValid_bool;
    pFOPDebug->bObjRelVelXValid_bool = sValidACCObj.bObjRelVelXValid_bool;
    pFOPDebug->bObjRelVelYValid_bool = sValidACCObj.bObjRelVelYValid_bool;
    pFOPDebug->bObjRelAclXValid_bool = sValidACCObj.bObjRelAclXValid_bool;
    pFOPDebug->bObjTypeValid_bool = sValidACCObj.bObjTypeValid_bool;
    pFOPDebug->bObjFusionStateValid_bool =
        sValidACCObj.bObjFusionStateValid_bool;
    pFOPDebug->bObjLatMovInvalid_bool = sValidACCObj.bObjLatMovInvalid_bool;

    pFOPDebug->bDistYOffsetChk_bool = bDistYOffsetChk_bool;
    pFOPDebug->fMinObjPosY_met = fMinObjPosY_met;
    pFOPDebug->bFusionReset_bool = bTempFusionReset_bool;
    pFOPDebug->bMeasStReset_bool = bTempMeasStReset_bool;
    pFOPDebug->bObjAndEgoStop_bool = bTempObjAndEgoStop_bool;
    pFOPDebug->bBothStop_bool = bTempBothStop_bool;
    pFOPDebug->bLatMovReset_bool = bTempLatMovReset_bool;
    pFOPDebug->fOffsetObjPosY_met = fOffsetObjPosY_met;
    pFOPDebug->fMinCurvature_1pm = fMinCurvature_1pm;
    pFOPDebug->fDistYOffset_met = fDistYOffset_met;
    pFOPDebug->fTempObjPosX_met = fTempObjPosX_met;
    pFOPDebug->fTempLongDistMax_met = fTempLongDistMax_met;
    pFOPDebug->bTempPosXHyst_bool = bTempPosXHyst_bool;
    pFOPDebug->fMinWidthTgtObj_met = fMinWidthTgtObj_met;
    pFOPDebug->fMaxLongVel_met = fMaxLongVel_met;
    pFOPDebug->fMaxLatVel_met = fMaxLatVel_met;
    pFOPDebug->fMaxLongAcl_met = fMaxLongAcl_met;
    pFOPDebug->bTempObjTypeChk_bool = bTempObjTypeChk_bool;
    pFOPDebug->bTempObjTypeChkWithHyst_bool = bTempObjTypeChkWithHyst_bool;
    pFOPDebug->bTempFusionTurnOn_bool = bTempFusionTurnOn_bool;
    pFOPDebug->bTempMeasStTurnOn_bool = bTempMeasStTurnOn_bool;
    pFOPDebug->fTempObjVelX_mps = fTempObjVelX_mps;
    pFOPDebug->bTempObjStandstill_bool = bTempObjStandstill_bool;
    pFOPDebug->bTempEgoStandstill_bool = bTempEgoStandstill_bool;

    return sValidACCObj;
}

/*****************************************************************************
  Functionname:    FOPValidACCObjPostProcess                               */ /*!

        @brief           Validate ACC object post process

        @description     After all ACC object attributes are validated, this
      function
                         is used to integrate all check flags.

        @param[in]       pValidACCObj    Information of check flags from
      Validate
      ACC object module
        @param[in]       pChkPossiHys    Object ID switch and position step
      check
      flags
        @param[in,out]   pFOPOutput      Validity check results output by FOP
        @param[in,out]   pFOPDebug       Debug output of FOP

        @return          none
      *****************************************************************************/
STATIc void FOPValidACCObjPostProcess(const FOPValidACCObj_t* pValidACCObj,
                                      const FOPChkPossiHys_t* pChkPossiHys,
                                      ODPRFOPOut_t* pFOPOutput,
                                      ODPRFOPDebug_t* pFOPDebug) {
    /* Update flag that object is detected in previous cycle */
    FOP_bPreCycObjDetected_bool =
        pValidACCObj->bACCObjDetected_bool &
        pValidACCObj->bObjQualityValid_bool &
        pValidACCObj->bObjMeasStateValid_bool &
        pValidACCObj->bObjPosYValid_bool & pValidACCObj->bObjPosXValid_bool &
        pValidACCObj->bObjWidthValid_bool &
        pValidACCObj->bObjRelVelXValid_bool &
        pValidACCObj->bObjRelVelYValid_bool &
        pValidACCObj->bObjRelAclXValid_bool & pValidACCObj->bObjTypeValid_bool &
        pValidACCObj->bObjFusionStateValid_bool;

    /* Write bits that reveal acc object check results*/
    pFOPOutput->uiAccObjInvalidCheck_btf =
        ((!pValidACCObj->bACCObjDetected_bool) << 0) |
        ((!pValidACCObj->bObjQualityValid_bool) << 1) |
        ((!pValidACCObj->bObjMeasStateValid_bool) << 2) |
        ((!pValidACCObj->bObjPosYValid_bool) << 3) |
        ((!pValidACCObj->bObjPosXValid_bool) << 4) |
        ((!pValidACCObj->bObjWidthValid_bool) << 5) |
        ((!pValidACCObj->bObjRelVelXValid_bool) << 6) |
        ((!pValidACCObj->bObjRelVelYValid_bool) << 7) |
        ((!pValidACCObj->bObjRelAclXValid_bool) << 8) |
        ((!pValidACCObj->bObjTypeValid_bool) << 9) |
        ((!pValidACCObj->bObjFusionStateValid_bool) << 10) |
        ((!pChkPossiHys->bNoObjIDSwitchDetected_bool) << 11) |
        ((pValidACCObj->bObjLatMovInvalid_bool) << 12);

    /* Debug output */
    pFOPDebug->bPreCycObjDetected_bool = FOP_bPreCycObjDetected_bool;
}

/*****************************************************************************
  Functionname:    FOPFreezeLateralKinematic                               */ /*!

        @brief           Freeze lateral kinematics

        @description     Freeze lateral kinematics.This function is
      corresponding
      to
                         FreezeLateralKinematics module in MBD

        @param[in]       pACCObjectData  Information of OOI_NEXT obejct from ACC
      module
        @param[in]       pValidACCObj    Information of check flags from
      Validate
      ACC object module
        @param[in,out]   pFOPOutput      Validity check results output by FOP

        @return          none

        @uml
        @startuml
          start
          if (Lateral moving state is invalid ) then (yes)
              :PosY Keeps the output of previous cycle,\nRelVelY and RelAclY is
      set
      to 0;;
          else (no)
              :Output PosY, RelVelY and RelAclY directly;
              note right:Normally output
          endif
          stop
        @enduml
      *****************************************************************************/
STATIc void FOPFreezeLateralKinematic(const ODPRInAccFRObj_t* pACCObjectData,
                                      const FOPValidACCObj_t* pValidACCObj,
                                      ODPRFOPOut_t* pFOPOutput) {
    if (pValidACCObj->bObjLatMovInvalid_bool) {
        /* Position Y */
        pFOPOutput->fAccObjPosY_met = FOP_fPreCycAccObjPosY_met;
        /* Relative velocity Y */
        pFOPOutput->fAccObjRelVelY_mps = 0.f;
        /* Relative acceleration Y */
        pFOPOutput->fAccObjRelAclY_mps2 = 0.f;
    } else {
        /* Position Y */
        pFOPOutput->fAccObjPosY_met = pACCObjectData->fObjPosY_met;
        /* Relative velocity Y */
        pFOPOutput->fAccObjRelVelY_mps = pACCObjectData->fObjRelVelY_mps;
        /* Relative acceleration Y */
        pFOPOutput->fAccObjRelAclY_mps2 = pACCObjectData->fObjRelAclY_mps2;
    }

    /* Reassign value, which is used in next cycle*/
    FOP_fPreCycAccObjPosY_met = pFOPOutput->fAccObjPosY_met;
}

/*****************************************************************************
  Functionname:    FOPPositionEstimation                               */ /*!

                    @brief           Position estimation

                    @description     Position estimation.This function is
                  corresponding to
                                     PositionEstimation module in MBD

                    @param[in,out]   pFOPOutput      Validity check results
                  output by
                  FOP

                    @return          none

                    @uml
                    @startuml
                      start
                      :PosX estimation;
                      note:Use PosX, RelVelX and RelAclX to
                  compensate\nEstimatedPosX
                  = PosX
                  + RelVelX*T + 0.5*RelAclX* T^2;
                      :PosY estimation;
                      note:Use PosY, RelVelY to compensate\nEstimatedPosY = PosY
                  +
                  RelVelY*T;
                      stop
                    @enduml
                  *****************************************************************************/
STATIc void FOPPositionEstimation(ODPRFOPOut_t* pFOPOutput) {
    float32 fObjPosXRelAccelXDelay_met =
        0.5f * pFOPOutput->fAccObjRelAclX_mps2 *
        TUE_CML_Sqr(ODPR_Kf_FOPObjSyncDelay_sec);

    /* Estimate Obejct position */
    pFOPOutput->fEstimateObjPosX_met =
        pFOPOutput->fAccObjPosX_met + fObjPosXRelAccelXDelay_met +
        pFOPOutput->fAccObjRelVelX_mps * ODPR_Kf_FOPObjSyncDelay_sec;
    pFOPOutput->fEstimateObjPosY_met =
        pFOPOutput->fAccObjPosY_met +
        pFOPOutput->fAccObjRelVelY_mps * ODPR_Kf_FOPObjSyncDelay_sec;
}

/*****************************************************************************
  Functionname:    FOPObjYawAngEstimation                               */ /*!

                 @brief           Obejct yaw angle estimation

                 @description     Obejct yaw angle estimation.This function is
               corresponding to
                                  ObjYawAngEstimation module in MBD

                 @param[in]       pACCObjectData  Information of OOI_NEXT obejct
               from
               ACC
               module
                 @param[in]       pSystemPara     System parameter
                 @param[in]       pEgoVEDData     Dynamic data of ego vehicle
               from
               VED
               module
                 @param[in]       pChkPossiHys    Object ID switch and position
               step
               check
               flags
                 @param[in,out]   pFOPOutput      Validity check results output
               by
               FOP
                 @param[in,out]   pFOPDebug       Debug output of FOP

                 @return          none

                 @uml
                 @startuml
                   start
                   if (Use own calculated object yaw angle) then (yes)
                       :Calculate integration of object VelX to obtain object
               PosX;
                       note right:Use integrator, which output is equal to
               output of
               previous cycle plus DeltaVelX;
                       :Calculate lowpass value of PosY;
                       note right:Select raw ACC object PosY or EstimatedPosY;
                       :Calculate DeltaYaw of ego vehicle;
                       note right:Use integrator to yawrate of ego vehicle;
                       :Calculate EnableSample flag;
                       note right:If EnableSample is TRUE, then use
               PosX/PosY/DeltaYaw to
               calculate Yaw angle,\notherwise Yaw angle keeps value of previous
               cycle;
                   else (no)
                       :Output Yaw angle directly;
                   endif
                   stop
                 @enduml
               *****************************************************************************/
STATIc void FOPObjYawAngEstimation(const ODPRInAccFRObj_t* pACCObjectData,
                                   const ODPRInSystemPara_t* pSystemPara,
                                   const ODPRInVEDVehDyn_t* pEgoVEDData,
                                   const FOPChkPossiHys_t* pChkPossiHys,
                                   ODPRFOPOut_t* pFOPOutput,
                                   ODPRFOPDebug_t* pFOPDebug) {
    boolean bTempResetObjChng_bool;
    boolean bTempEnableCalc_bool;
    boolean bTempEnableSample_bool;
    boolean bTempReset_bool;
    float32 fTempYawAngle_rad;
    float32 fTempPosX_met;
    float32 fTempPosYIn_met;
    float32 fTempVehYawAng_rad;
    float32 fTempYawAngleCalc_rad = 0.f;

    bTempResetObjChng_bool =
        (!pChkPossiHys->bNoObjIDSwitchDetected_bool) ||
        (pACCObjectData->bObjectDetected_bool - FOP_bPreCycAccObjDetected_bool);
    bTempEnableCalc_bool = pACCObjectData->bObjectDetected_bool &&
                           pChkPossiHys->bNoObjIDSwitchDetected_bool;
    bTempReset_bool = (FOP_bPreCycEnableSample_bool || bTempResetObjChng_bool);

    /* Calculate Own calculated PosX */
    if (bTempReset_bool) {
        fTempPosX_met = 0.f;
    } else {
        if (bTempEnableCalc_bool) {
            fTempPosX_met = FOP_fPosXIntegrator_met +
                            pSystemPara->fSystemCylceTime_sec *
                                (pEgoVEDData->fEgoVelX_mps +
                                 pACCObjectData->fObjRelVelX_mps) /
                                1.f;
        } else {
            fTempPosX_met = FOP_fPosXIntegrator_met;
        }
    }
    fTempPosX_met = TUE_CML_MinMax(0.f, 300.f, fTempPosX_met);

    /* Calculate Own calculated PosY */
    fTempPosYIn_met = ODPR_Kb_FOPUseEstiPosY4Yaw_bool
                          ? pFOPOutput->fEstimateObjPosY_met
                          : pACCObjectData->fObjPosY_met;
    if (bTempResetObjChng_bool) {
        FOP_fPosYLowPass_met = 0.f;
    } else {
        if (bTempEnableCalc_bool) {
            TUE_CML_LowPassFilter(&FOP_fPosYLowPass_met, fTempPosYIn_met,
                                  (pSystemPara->fSystemCylceTime_sec /
                                   SafeDiv(ODPR_Kf_FOPObjPosYPT1Time4Yaw_sec)));
        } else {
        }
    }

    /* Calcualte Own calculated Veh YawAng */
    if (bTempReset_bool) {
        fTempVehYawAng_rad = 0.f;
    } else {
        if (bTempEnableCalc_bool) {
            fTempVehYawAng_rad = FOP_fVehYawIntegrator_rad +
                                 pSystemPara->fSystemCylceTime_sec *
                                     pEgoVEDData->fEgoYawRate_rps / 1.f;
        } else {
            fTempVehYawAng_rad = FOP_fVehYawIntegrator_rad;
        }
    }
    fTempVehYawAng_rad = TUE_CML_MinMax(-3.1415f, 3.1415f, fTempVehYawAng_rad);

    /* Calculate EnableSample flag */
    if (bTempReset_bool) {
        FOP_fEnableCalcTimer_sec = 0.f;
    } else {
        FOP_fEnableCalcTimer_sec =
            bTempEnableCalc_bool
                ? (FOP_fEnableCalcTimer_sec + pSystemPara->fSystemCylceTime_sec)
                : FOP_fEnableCalcTimer_sec;
    }

    bTempEnableSample_bool =
        ((FOP_fEnableCalcTimer_sec > FOP_OBJ_YAW_SAMPLE_TIME_SEC) &&
         (fTempPosX_met > FOP_OBJ_YAW_SAMPLE_DIST_MET));

    /* Calcualte Own calculated Obj YawAng */
    if (bTempEnableSample_bool) {
        fTempYawAngleCalc_rad =
            (FOP_fPosYLowPass_met -
             (FOP_fPreCycPosY_met * TUE_CML_GDBcos_52(fTempVehYawAng_rad))) /
            SafeDiv(fTempPosX_met - (FOP_fPreCycPosY_met *
                                     TUE_CML_GDBsin_52(fTempVehYawAng_rad)));

        fTempYawAngleCalc_rad =
            TUE_CML_MinMax(-1.f, 1.f, fTempYawAngleCalc_rad);

        FOP_fYawAngCalc_rad =
            fTempYawAngleCalc_rad /
            SafeDiv(0.28086f * TUE_CML_Sqr(fTempYawAngleCalc_rad) + 1.f);
    } else {
    }

    /* Yaw Angle Lowpass filter */
    fTempYawAngle_rad = FOP_USE_OWN_OBJ_YAW_BOOL
                            ? FOP_fYawAngCalc_rad
                            : pACCObjectData->fObjRelHeadingAngle_rad;
    if (bTempResetObjChng_bool) {
        FOP_fPreCycYawAngLowPass_rad = 0.f;
    } else {
        if (bTempEnableCalc_bool) {
            TUE_CML_LowPassFilter(&FOP_fPreCycYawAngLowPass_rad,
                                  fTempYawAngle_rad,
                                  (pSystemPara->fSystemCylceTime_sec /
                                   SafeDiv(FOP_OBJ_YAW_PT1_TIME_SEC)));
        } else {
        }
    }
    pFOPOutput->fAccObjRelHeadingAngle_rad = FOP_fPreCycYawAngLowPass_rad;

    /* Reassign value, which is used in next cycle*/
    FOP_bPreCycAccObjDetected_bool = pACCObjectData->bObjectDetected_bool;
    FOP_bPreCycEnableSample_bool = bTempEnableSample_bool;
    FOP_fPosXIntegrator_met = fTempPosX_met;
    FOP_fVehYawIntegrator_rad = fTempVehYawAng_rad;
    FOP_fPreCycPosY_met = FOP_fPosYLowPass_met;

    /* Debug output */
    pFOPDebug->fTempPosX_met = fTempPosX_met;
    pFOPDebug->bTempReset_bool = bTempReset_bool;
    pFOPDebug->bTempEnableCalc_bool = bTempEnableCalc_bool;
    pFOPDebug->fPosYLowPass_met = FOP_fPosYLowPass_met;
    pFOPDebug->fTempVehYawAng_rad = fTempVehYawAng_rad;
    pFOPDebug->bYawAngEnableSample_bool = bTempEnableSample_bool;
    pFOPDebug->fTempYawAngleCalc_rad = fTempYawAngleCalc_rad;
    pFOPDebug->fOwnCalculatedYawAng_rad = FOP_fYawAngCalc_rad;
    pFOPDebug->bTempResetObjChng_bool = bTempResetObjChng_bool;
    pFOPDebug->fTempPosYIn_met = fTempPosYIn_met;
    pFOPDebug->fTempYawAngle_rad = fTempYawAngle_rad;
}

/*****************************************************************************
  Functionname:    BASICTurnOnDelay                                   */ /*!

                       @brief           Turn on delay

                       @description     Turn on delay.This function is a basic
                     function.

                       @param[in]       bInput           Input to be checked
                       @param[in]       fDelayTime       Delay time
                       @param[in]       fCycleTime       Cycle time
                       @param[in]       fGlobalTime      Define a global
                     variable and
                     input
                     its address.
                                                                                     Note:
                     The
                     initial value should be 0.f.

                       @return          bTurnOnFlg_bool  Turn on flag
                     *****************************************************************************/
boolean BASICTurnOnDelay(const boolean bInput,
                         const float32 fDelayTime,
                         const float32 fCycleTime,
                         float32* fGlobalTime) {
    boolean bTurnOnFlg_bool;

    if (bInput) {
        *fGlobalTime = TUE_CML_Max(*fGlobalTime, -fCycleTime) - fCycleTime;
        bTurnOnFlg_bool = (*fGlobalTime <= (-fCycleTime));
    } else {
        *fGlobalTime = fDelayTime;
        bTurnOnFlg_bool = FALSE;
    }

    return bTurnOnFlg_bool;
}

/*****************************************************************************
  Functionname:    BASICTurnOffDelay                                   */ /*!

                    @brief           Turn off delay

                    @description     Turn off delay.This function is a basic
                  function.

                    @param[in]       bInput           Input to be checked
                    @param[in]       fDelayTime       Delay time
                    @param[in]       fCycleTime       Cycle time
                    @param[in]       fGlobalTime      Define a global variable
                  and
                  input its
                  address.
                                                      Note: The initial value
                  should
                  be 0.f.

                    @return          bTurnOffFlg_bool  Turn off flag
                  *****************************************************************************/
boolean BASICTurnOffDelay(const boolean bInput,
                          const float32 fDelayTime,
                          const float32 fCycleTime,
                          float32* fGlobalTime) {
    boolean bTurnOffFlg_bool;

    if (!bInput) {
        *fGlobalTime = TUE_CML_Max(*fGlobalTime, -fCycleTime) - fCycleTime;
        bTurnOffFlg_bool = (*fGlobalTime > (-fCycleTime));
    } else {
        *fGlobalTime = fDelayTime;
        bTurnOffFlg_bool = TRUE;
    }

    return bTurnOffFlg_bool;
}

/*****************************************************************************
  Functionname:    BASICDivDflt                                   */ /*!

                                   @brief           Division with default value

                                   @description     The division value is
                                 Numerator/Denominator if
                                 Denominator is not
                                                    zero, otherwise is default
                                 value

                                   @param[in]       fNumerator       Numerator
                                   @param[in]       fDenominator     Denominator
                                   @param[in]       fDflt            Defalut
                                 value

                                   @return          fDivValue        Division
                                 value
                                 *****************************************************************************/
float32 BASICDivDflt(const float32 fNumerator,
                     const float32 fDenominator,
                     const float32 fDflt) {
    float32 fDivValue;

    if (TUE_CML_Abs(fDenominator) < TUE_C_F32_DELTA) {
        fDivValue = fDflt;
    } else {
        fDivValue = fNumerator / fDenominator;
    }

    return fDivValue;
}

/*****************************************************************************
  Functionname:    ODPR_FOP_Exec                                            */ /*!

     @brief           Main function of Front Object Process

     @description     Peform preprocessing and validity check of OOI_NEXT obejct
                      that outputs from ACC module, including quality check,
   type
                      check, velocity check, position check, measurement state
   check,
                      etc.

     @param[in]       reqPorts             Input structure of FOP function
                           pACCObjectData  Information of OOI_NEXT obejct from
   ACC
   module
                           pSystemPara     System parameter
                           pEgoVEDData     Dynamic data of ego vehicle  from VED
   module
     @param[in]       param                Param structure of FOP function
                           pFOPParam       FOP parameters
     @param[in,out]   proPorts             Output structure of FOP function
                           pFOPOutData     Validity check results output by FOP
     @param[in,out]   debugInfo            Debug info of FOP function
                           pFOPDebug       Debug signal of FOP

     @return          none
   *****************************************************************************/
void ODPR_FOP_Exec(const FOPInReq_t* reqPorts,
                   const ODPRParam_t* param,
                   FOPOutPro_t* proPorts,
                   FOPDebug_t* debugInfo) {
    /* Input and output wrapper */
    const ODPRInAccFRObj_t* pACCObjectData = reqPorts->pAccObject;
    const ODPRInSystemPara_t* pSystemPara = reqPorts->pSystemPara;
    const ODPRInVEDVehDyn_t* pEgoVEDData = reqPorts->pEgoVehSig;
    const ODPRParam_t* pParam = param;
    ODPRFOPOut_t* pFOPOutput = proPorts->pFOPOutData;
    ODPRFOPDebug_t* pFOPDebug = debugInfo->pFOPDebug;

    /* VARIABLES */
    FOPChkPossiHys_t sChkPossiHys;
    FOPValidACCObj_t sValidACCObj;

    /* Output directly */
    pFOPOutput->fAccObjPosXStdDev_met = pACCObjectData->fObjPosXStdDev_met;
    pFOPOutput->fAccObjPosYStdDev_met = pACCObjectData->fObjPosYStdDev_met;
    pFOPOutput->fAccObjRelVelX_mps = pACCObjectData->fObjRelVelX_mps;
    pFOPOutput->fAccObjRelAclX_mps2 = pACCObjectData->fObjRelAclX_mps2;

    /* Timestamp check and conversion */
    FOPTimeConversion(pACCObjectData, pSystemPara, pFOPOutput);

    /* Transfrom object X position */
    FOPTransObjXPos(pACCObjectData, pParam, pFOPOutput);

    /* Check for possible hysteresis */
    sChkPossiHys = FOPChkPossHysteresis(pACCObjectData, pSystemPara, pFOPOutput,
                                        pFOPDebug);

    /* Validate ACC object */
    sValidACCObj =
        FOPValidateACCObject(pACCObjectData, &sChkPossiHys, pSystemPara,
                             pEgoVEDData, pParam, pFOPOutput, pFOPDebug);

    /* Validate ACC object post process */
    FOPValidACCObjPostProcess(&sValidACCObj, &sChkPossiHys, pFOPOutput,
                              pFOPDebug);

    /* Freeze lateral kinematics */
    FOPFreezeLateralKinematic(pACCObjectData, &sValidACCObj, pFOPOutput);

    /* Position estimation */
    FOPPositionEstimation(pFOPOutput);

    /* Yaw angle estimation */
    FOPObjYawAngEstimation(pACCObjectData, pSystemPara, pEgoVEDData,
                           &sChkPossiHys, pFOPOutput, pFOPDebug);
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */