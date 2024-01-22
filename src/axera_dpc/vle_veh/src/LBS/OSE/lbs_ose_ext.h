#ifndef LBS_OSE_EXT_H
#define LBS_OSE_EXT_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
        INCLUDES
*****************************************************************************/
#include "TM_Global_Types.h"
#include "lbs_external.h"
#include "string.h"
/*****************************************************************************
        TYPEDEFS : INPUT   PARAMETER    OUTPUT    DEBUG
*****************************************************************************/
// #define LBS_INPUT_OBJECT_NUMBER (80u)
#define OSE_LBS_NUM_OF_WARN_LEVELS (3u)
#define OSE_LBS_NUM_OF_BREAK_LINES (2u)
#define OSE_LBS_COMMON_WARN_LOCK_TIME (5.0f)

typedef struct {
    boolean bOSEFunctionActive; /* OSE function active flag */
    boolean bOSEPowermode3min;  // DOW power mode counter 3s
} OSEFunctionSwitchInReq_t;

typedef struct {
    uint32 uiTimeStamp_ms;
    uint16 uiMeasurementCounter_nu;
    uint16 uiCycleCounter_nu;
    uint8 eSigStatus_nu;
} OSESignalHeaderInReq_t;

typedef struct {
    boolean bRightSensor; /*Flag whether the object is detected by the right
                             sensor*/
    float32 fDistX_met;   /*Object's longitudinal relative distance*/
    float32 fDistXStd_met;
    float32 fDistY_met; /*Object's lateral relative distance*/
    float32 fDistYStd_met;
    float32 fVrelX_mps;       /*Object's longitudinal relative velocity*/
    float32 fVrelY_mps;       /*Object's lateral relative velocity*/
    float32 fVabsX_mps;       // Object's longitudinal velocity over ground
    float32 fVabsY_mps;       // Object's lateral velocity over ground
    uint16 uiLifeCycles_nu;   /*Object lifetime in cycles,unit: null*/
    float32 fWidth_met;       /*Object's overall width*/
    float32 fWidthLeft_met;   /*Object's width left of the track position(left
                                 sensor view)*/
    float32 fWidthRight_met;  /*Object's width right of the track position(left
                                 sensor view)*/
    float32 fLengthFront_met; /*Object's length ahead of the track position(left
                                 sensor view)*/
    float32 fLengthRear_met;  /*Object's length behind the track position(left
                                 sensor view)*/
    float32 fAbsOrientation_rad; /*Object moving direction,based on VX and VY in
                                    AUTOSAR(left sensor view)*/
    float32
        fAbsOrientationStd_rad; /*Standard deviation object moving direction*/
    float32
        fFirstDetectX_met; /* X position where the object was created, unit:m*/
    float32
        fProbabilityOfExistence_per; /* Probability that the object represents a
                                        real object,unit:0.0-1.0f*/
    uint8 uiMeasuredTargetFrequency_nu; /*Bitfield to indicate if the object was
                                           measured in the last 8 cycles*/
    float32 fRCS;                       // RCS
    float32 fMirrorProb_per;  // The probability that the object is mirror
    uint8 eMaintenanceState;
} OSEGenObjectInReq_t;
typedef OSEGenObjectInReq_t OSEGenObjArrayInReq[LBS_INPUT_OBJECT_NUMBER];

typedef struct {
    sint8 iNumOfUsedObjects_nu;
    sint8 iSortedObjectArray[40];
    uint8 eObjListSource;
} OSEGenObjListHeaderInReq_t;

typedef struct {
    uint32 uiVersionNumber;
    OSESignalHeaderInReq_t sSigHeader;
    OSEGenObjListHeaderInReq_t HeaderObjList;
    OSEGenObjArrayInReq aObject;
} OSEEMGenObjListInReq_t;

typedef struct {
    float32 fXmin_met; /*The object x min position,unit:m*/
    float32 fXmax_met; /*The object x max position,unit:m*/
    float32 fYmin_met; /*The object y min position,unit:m*/
    float32 fYmax_met; /*The object y max position,unit:m*/
} OSELBSObjBorders_t;

typedef struct {
    OSELBSObjBorders_t ObjBorders; /*The object border information*/
    float32 fUpdateRate_nu;     /*The object measurement update rate,unit:NULL*/
    float32 fXMovement_met;     /*The object total moving distance in the x
                                   direction,unit:m*/
    float32 fYMovement_met;     /*The object total moving distance in the y
                                   direction,unit:m*/
    float32 fAssocProbFiltered; /*Highest cluster association probability of the
                                   object filter result*/
} OSELBSObjInfoArrayInReq_t;
typedef OSELBSObjInfoArrayInReq_t
    OSELBSObjInfoArrayInReq[LBS_INPUT_OBJECT_NUMBER];

typedef struct {
    OSELBSObjInfoArrayInReq OSELBSObjInfoArray;
} OSELBSGlobalInReq_t;

typedef struct {
    float32 fCycletime_s; /* Current task cycle time from EMGlobalOutput*/
    OSEEMGenObjListInReq_t EMGenObjList;  //
    OSEFunctionSwitchInReq_t OSEFunctionSwitch;
    OSELBSGlobalInReq_t OSELBSGlobalInReq;
} OSEInReq_t;

typedef float32 fTTCThreshold_arrayParam_t[3];
typedef float32 fYMinBreakthrough_arrayParam_t[2];
typedef float32 fYMaxBreakthrough_arrayParam_t[2];
typedef float32 fYMinBreakthroughMargin_arrayParam_t[2];
typedef float32 fYMaxBreakthroughMargin_arrayParam_t[2];
typedef float32 fXBreakthroughLine_arrayParam_t[2];
typedef float32 fTargetRangeMax_arrayParam_t[3];
typedef float32 fMinTime_arrayParam_t[3];
typedef float32 fMaxTime_arrayParam_t[3];

typedef struct {
    float32 fVehicleWidth_met;   // the width of ego vehicle
    float32 fVehicleLength_met;  // the length of ego vehicle
    float32 fOverhangFront_met;  // the length of front overhang
} OSEVehParAddParam_t;

typedef struct {
    float32 fLatPos_met;
    float32 fLongPos_met;
} OSE_SensorMounting_t;

typedef struct {
    OSE_SensorMounting_t SensorLeft;
    OSE_SensorMounting_t SensorRight;
} OSE_SensorMounting_st;

typedef struct {
    fTTCThreshold_arrayParam_t
        fTTCThreshold_s;  // TTC thresholds of three levels; Value:
                          // {2.5f, 1.5f, 1.2f}
    fYMinBreakthrough_arrayParam_t
        fYMinBreakthrough_met;  // Two Y-axis min breakthrough; Value: {0.0f,
                                // 0.0f}
    fYMaxBreakthrough_arrayParam_t
        fYMaxBreakthrough_met;  // Two Y-axis max breakthrough; Value:
                                // {1.0f, 1.0f}
    fYMinBreakthroughMargin_arrayParam_t
        fYMinBreakthroughMargin_met;  // Two Y-axis min breakthrough margin;
                                      // Value: {0.5f, 0.5f}
    fYMaxBreakthroughMargin_arrayParam_t
        fYMaxBreakthroughMargin_met;  // Two Y-axis max breakthrough margin;
                                      // Value: {0.5f, 0.5f}
    fXBreakthroughLine_arrayParam_t
        fXBreakthroughLine_met;  // Distance between vehicle rear edge and two
                                 // X-axis breakthrough lines; Value:
                                 // {1.0f, 3.0f}
    float32 fVEgoMax_mps;        // The max ego vehicle speed of OSE is actived;
                                 // Value: 0.1f m/s
    float32 fVEgoMin_mps;        // The min ego vehicle speed of OSE is actived;
                                 // Value: -0.1f m/s
    float32 fVTargetMin_mps;  // The min target vehicle speed of OSE is actived;
                              // Value: 3.0f m/s
    float32 fVTargetMax_mps;  // The max target vehicle speed of OSE is actived;
                              // Value: 40.0f m/s
    fMinTime_arrayParam_t
        fMinTime_s;  // The min time thresholds of three levels OSE warning;
                     // Value: {1.0f, 1.0f, 1.0f}
    fMaxTime_arrayParam_t
        fMaxTime_s;  // The max time thresholds of three levels OSE warning;
                     // Value: {1.5f, 1.5f, 1.5f}
    float32 fMaxHeadingAngle;  // The max heading threshold; Value: 60.0f
    float32 fMinHeadingAngle;  // The min heading threshold; Value: -70.0f
    fTargetRangeMax_arrayParam_t
        fTargetRangeMax_met;  // The max range thresholds of three levels OSE
                              // warning; Value: {42.0f, 30.0f, 20.0f}
    boolean bEnableObjAdaptiveBreakthrough;  // Value: TRUE
    boolean bActive;  // Flag of BSW parameters(driver); Value: TRUE
    OSEVehParAddParam_t VehParAdd;
    OSE_SensorMounting_st SensorMounting;

} OSEParam_t;

typedef struct {
    float32 fBTHitHystTimer;
    uint8 uBreakthroughHitConfi[OSE_LBS_NUM_OF_BREAK_LINES];  // Breakthrough
                                                              // hit confidence
    boolean bBreakthroughHit[OSE_LBS_NUM_OF_BREAK_LINES];  // Flag whether the
                                                           // object hits
                                                           // breakthrough
    boolean bWarning;           // Flag whether the OSE warning is actived
    boolean bWarningLastCycle;  // Flag whether the OSE warning was actived in
                                // the last cycle
    boolean bObjectInRange;  // Flag whether the object is in the range of three
                             // different level
    boolean bTTCBelowThresh[OSE_LBS_NUM_OF_BREAK_LINES];  // Flag whether the
                                                          // object's TTC is
                                                          // below TTC Threshold
    boolean bBTHitHystActive;  // Flag whether the warning hysteresis
} OSEObjInfoLevelOutPro_t;

typedef struct {
    uint16 uCounters[3];
    float32 fValue_met;  // The object width in the algorithm
} OSEWidthEstimOutPro_t;

typedef struct {
    OSEObjInfoLevelOutPro_t
        InfoLevel[OSE_LBS_NUM_OF_WARN_LEVELS];  // DOW warning information
                                                // structure
    float32
        fYBreakthrough[OSE_LBS_NUM_OF_BREAK_LINES];  // The y-axis breakthrough
                                                     // of the object
    float32 fYBreakthroughStd[OSE_LBS_NUM_OF_BREAK_LINES];  // The y-axis
                                                            // breakthrough
                                                            // standard
    // deviation of the object
    float32 fTTC_s[OSE_LBS_NUM_OF_BREAK_LINES];          // TTC of the object
    float32 fTTCFiltered_s[OSE_LBS_NUM_OF_BREAK_LINES];  // Filtered TTC of the
                                                         // object
    float32 fDistToCrossingLine_met[OSE_LBS_NUM_OF_BREAK_LINES];  // Distance
                                                                  // between
                                                                  // object
                                                                  // front edge
                                                                  // and
    // ego vehicle rear edge
    float32 fSideTrackProb;  // The probability of the side object
    boolean bRelevant;  // Flag whether the object is relevant for a OSE warning
    boolean bMirror;    // Flag whether the object is mirror
    boolean bSideTrack;
    boolean bObjectFromRear;  // Flag whether the object is approaching from the
                              // rear in the current cycle or first time
    boolean
        bValidApproachAngle;   // Flag whether the object heading angle is valid
    boolean bObjectAtEdgeFoV;  // Flag whether the object is at the edge of FOV
    boolean bShortTTC;
    boolean bUpdatedRecently;  // Flag whether the object is updated recently
    float32 fQuality;  // The quality probability of the DOW warning object
    OSEWidthEstimOutPro_t fEstWidth;
} OSEObjInfoArrayOutPro_t;

typedef struct {
    // OSEObjInfoArrayOutPro_t OSEObjInfoArray[LBS_INPUT_OBJECT_NUMBER];
    float32 fCriticalTTC;
    float32 fCriticalObjDistX;
    uint8 uCriticalObjID;
    boolean bWarningInterrupt;
    boolean bDOWPowerModeDone;
    boolean bOSEWarnActive[OSE_LBS_NUM_OF_WARN_LEVELS];
} OSEOutPro_t;

typedef struct {
    boolean LCA_WarnDecide_bWarning;
    boolean LCA_WarnDecide_bObjectInRange;
    uint8 LCA_WarnDecide_uBreakthroughHitConfi[2];
    boolean LCA_WarnDecide_bTTCBelowThresh[2];
    boolean LCA_WarnDecide_bBTHitHystActive;
    float32 LCA_WarnDecide_fTTCThreshold_s;
} OSE_WarnDecide_InfoLevel_t;

typedef OSE_WarnDecide_InfoLevel_t
    OSE_WarnDecide_InfoLevelArray_t[OSE_LBS_NUM_OF_WARN_LEVELS];
typedef struct {
    uint8 OBJID;
    OSE_WarnDecide_InfoLevelArray_t OSE_WarnDecide_InfoLevel;
    boolean LCA_WarnDecide_bRelevant;
    boolean LCA_WarnDecide_Non_bMirror;
    boolean LCA_WarnDecide_Non_bShortTTC;
    boolean LCA_WarnDecide_Non_bSideTrack;
    boolean LCA_WarnDecide_bValidApproachAngle;
    boolean LCA_WarnDecide_bUpdatedRecently;
    float32 LCA_WarnDecide_fQuality;
    float32 LCA_WarnDecide_fRCS;
    float32 LCA_WarnDecide_fTTC_s[2];
    float32 LCA_WarnDecide_fTTCFiltered_s[2];
} OSE_Warn_Decide_Debug_t;

typedef OSE_Warn_Decide_Debug_t
    OSE_Warn_Decide_Debug_Array[LBS_INPUT_OBJECT_NUMBER];

typedef struct {
    uint32 uiVersionNum_nu;  // uint32 value example
    OSE_Warn_Decide_Debug_Array OSEWarnDecideList;
} OSEDebug_t;
/*****************************************************************************
        VARIABLES
*****************************************************************************/
extern OSEInReq_t OSEReqPorts;
extern OSEParam_t OSEParams;
extern OSEOutPro_t OSEProPorts;
extern OSEDebug_t OSEDebugInfo;
/*****************************************************************************
        FUNCTION PROTOTYPES
*****************************************************************************/

void LBS_OSE_Reset();
void OSEInitGlobals();
void OSEInitObjects();
void LBS_OSE_Exec(const OSEInReq_t* reqPorts,
                  const OSEParam_t* paras,
                  OSEOutPro_t* proPorts,
                  OSEDebug_t* debug);

#ifdef __cplusplus
}
#endif
#endif