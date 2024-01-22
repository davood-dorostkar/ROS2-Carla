#ifndef LBS_EXTERNAL_H
#define LBS_EXTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include "Rte_Type.h"
#include "TM_Global_Types.h"

#define LBS_INPUT_OBJECT_NUMBER (100u)
#define LBS_OSE_NUM_OF_WARN_LEVELS (3u)
#define LBS_RCW_MAX_NOF_CORR_OBJS (4u)
// if debug
#define LBS_OSE_LBS_NUM_OF_WARN_LEVELS (3u)
#define LBS_OSE_LBS_NUM_OF_BREAK_LINES (2u)
#define LBS_OSE_NUM_OF_BREAK_LINES (2u)

// end
/*****************************************************************************
                TYPEDEFS : INPUT
*****************************************************************************/
#ifndef Rte_TypeDef_LBS_GenObjKinematics_t
#define Rte_TypeDef_LBS_GenObjKinematics_t
typedef struct {
    float32 fDistX_met; /*Object's longitudinal relative distance*/
    float32 fDistXStd_met;
    float32 fDistY_met; /*Object's lateral relative distance*/
    float32 fDistYStd_met;
    float32 fVrelX_mps; /*Object's longitudinal relative velocity*/
    float32 fVrelXStd_mps;
    float32 fVrelY_mps; /*Object's lateral relative velocity*/
    float32 fVrelYStd_mps;
    float32 fArelX_mpss;
    float32 fArelXStd_mpss;
    float32 fArelY_mpss;
    float32 fArelYStd_mpss;
    float32 fVabsX_mps;
    float32 fVabsXStd_mps;
    float32 fVabsY_mps;
    float32 fVabsYStd_mps;
    float32 fAabsX_mpss;
    float32 fAabsXStd_mpss;
    float32 fAabsY_mpss;
    float32 fAabsYStd_mpss;
} LBS_GenObjKinematics_t;
#endif

#ifndef Rte_TypeDef_LBS_GenObjGeometry_t
#define Rte_TypeDef_LBS_GenObjGeometry_t
typedef struct {
    float32 fWidth_met;       /*Object's overall width*/
    float32 fWidthStd_met;    /**/
    float32 fWidthLeft_met;   /*Object's width left of the track position(left
                                 sensor view)*/
    float32 fWidthRight_met;  /*Object's width right of the track position(left
                                 sensor view)*/
    float32 fLength_met;      /*Object's overall length*/
    float32 fLengthStd_met;   /**/
    float32 fLengthFront_met; /*Object's length ahead of the track position(left
                                 sensor view)*/
    float32 fLengthRear_met;  /*Object's length behind the track position(left
                                 sensor view)*/
    float32 fAbsOrientation_rad; /*Object moving direction,based on VX and VY in
                                    AUTOSAR(left sensor view)*/
    float32 fAbsOrientationStd_rad;
    // float32 fRelHeading_rad; /*Heading Angle*/
    // float32 fRelHeadingStd_rad;
    // float32 fClosestPointX_met; /*X position of closet point*/
    // float32 fClosestPointY_met; /*Y position of closet point*/
} LBS_GenObjGeometry_t;
#endif

#ifndef Rte_TypeDef_LBS_SRRObjHistory_t
#define Rte_TypeDef_LBS_SRRObjHistory_t
typedef struct {
    float32
        fFirstDetectX_met; /* X position where the object was created, unit:m*/
    float32
        fFirstDetectY_met; /* Y position where the object was created, unit:m*/
    float32 fMaxRange_met; /* Max range over lifetime of this object, unit:m*/
} LBS_SRRObjHistory_t;
#endif

#ifndef Rte_TypeDef_LBS_SRRObjQualifiers_t
#define Rte_TypeDef_LBS_SRRObjQualifiers_t
typedef struct {
    float32
        fProbabilityOfExistence_per; /* Probability that the object represents a
                                        real object,unit:0.0-1.0f*/
    uint8 uiHighestAssocProb_per;    /*Highest association probability of all
                                        associated clusters in this cycle*/
    uint8 uiMeasuredTargetFrequency_nu; /*Bitfield to indicate if the object was
                                           measured in the last 8 cycles*/
    // uint8 eFusionStatus_nu;             /*Enumeration of fused objects
    // status*/
    boolean bObjStable; /*Flag that object is stable*/
} LBS_SRRObjQualifiers_t;
#endif

#ifndef Rte_TypeDef_LBS_SRRObjRoadRelation_t
#define Rte_TypeDef_LBS_SRRObjRoadRelation_t
typedef struct {
    // float32 fProbInEgoLane_per; /* TODO */ /*undetermined */
    float32 fDist2Course_met; /* Object distance to course, unit:m*/
    float32 fDist2Border_met; /* Object distance to border, unit:m*/
    // float32 fGRDTrkProbability_per; /* TODO */ /*undetermined */
    boolean bDist2BorderValid; /* Valid flag if the value in Dist2Border can be
                                  used */
} LBS_SRRObjRoadRelation_t;
#endif

#ifndef Rte_TypeDef_LBS_SRRObjSensorSpecific_t
#define Rte_TypeDef_LBS_SRRObjSensorSpecific_t
typedef struct {
    float32 fRCS;             // RCS
    float32 fMirrorProb_per;  // The probability that the object is mirror
} LBS_SRRObjSensorSpecific_t;
#endif

#ifndef Rte_TypeDef_LBS_GenObjGenerals_t
#define Rte_TypeDef_LBS_GenObjGenerals_t
typedef struct {
    float32 fLifeTime_s;    /*Object lifetime in second,unit: s*/
    uint16 uiLifeCycles_nu; /*Object lifetime in cycles,unit: null*/
    // uint32 uiLastMeasuredTimeStamp_ms;
    // uint16 uiLastMeasuredCycle_nu;
    uint8 uiMaintenanceState_nu; /*Maintenance state if
                                    object(measured,predicted)*/
    uint16 uiID_nu;
    sint32 iRawFusionID_nu;
} LBS_GenObjGenerals_t;
#endif

#ifndef Rte_TypeDef_LBS_GenObjAttributes_t
#define Rte_TypeDef_LBS_GenObjAttributes_t
typedef struct {
    uint8 eDynamicProperty_nu;   /*Object dynamic property,stationary,moving or
                                    oncoming*/
    uint8 uiDynConfidence_per;   /*General confidence of dynamic
                                    property(moving,crossing,oncoming)*/
    uint32 eClassification_nu;   /*Object classification*/
    uint8 uiClassConfidence_per; /*Confidences for all classification*/
    // uint8 eObjctOcclusion_nu;
} LBS_GenObjAttributes_t;
#endif
#ifndef Rte_TypeDef_LBS_GenObject_st
#define Rte_TypeDef_LBS_GenObject_st
typedef struct {
    // uint8 ObjID;
    LBS_GenObjKinematics_t Kinemactic;
    LBS_GenObjGeometry_t Geometry;
    LBS_GenObjGenerals_t General;
    LBS_GenObjAttributes_t Attributes;
    boolean bRightSensor;
} LBS_GenObject_st;
#endif

typedef LBS_GenObject_st LBS_GenObjectArray[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBS_SignalHeader_t
#define Rte_TypeDef_LBS_SignalHeader_t
typedef struct {
    uint32 uiTimeStamp_ms;
    uint16 uiMeasurementCounter_nu;
    uint16 uiCycleCounter_nu;
    uint8 eSigStatus_nu;
    uint8 a_reserve;
} LBS_SignalHeader_t;
#endif

#ifndef Rte_TypeDef_EMGenObjList_st
#define Rte_TypeDef_EMGenObjList_st
typedef struct {
    uint32 uiVersionNumber;
    uint8 uNumOfUsedObjects;
    LBS_SignalHeader_t sSigHeader;
    LBS_GenObjectArray aObject;
} EMGenObjList_st;
#endif

#ifndef Rte_TypeDef_LBS_SRRObject_st
#define Rte_TypeDef_LBS_SRRObject_st
typedef struct {
    // uint8 ObjID;
    // LBS_SRRObjHistory_t History;
    LBS_SRRObjQualifiers_t Qualifiers;
    // LBS_SRRObjRoadRelation_t RoadRelation;
    LBS_SRRObjSensorSpecific_t SensorSpecific;
} LBS_SRRObject_st;
#endif

typedef LBS_SRRObject_st LBS_SRRObjectArray[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_EMSRRObjList_st
#define Rte_TypeDef_EMSRRObjList_st
typedef struct {
    uint32 uiVersionNumber;
    LBS_SignalHeader_t sSigHeader;
    LBS_SRRObjectArray aObject;
} EMSRRObjList_st;
#endif

#ifndef Rte_TypeDef_AllDoorSwitchInReq_t
#define Rte_TypeDef_AllDoorSwitchInReq_t
typedef struct {
    uint8 uLeftFrontDoorSwitch;   // the switch signal of left front side door
    uint8 uLeftRearDoorSwitch;    // the switch signal of left rear side door
    uint8 uRightFronDoorSwitch;   // the switch signal of right front side door
    uint8 uRighttRearDoorSwitch;  // the switch signal of right rear side door
    uint8 uLeftFrontDoorHalfSwitch;   // the half-open and half-closed switch
                                      // signal of left front side door
    uint8 uLeftRearDoorHalfSwitch;    // the half-open and half-closed switch
                                      // signal of left rear side door
    uint8 uRightFronDoorHalfSwitch;   // the half-open and half-closed switch
                                      // signal of right front side door
    uint8 uRighttRearDoorHalfSwitch;  // the half-open and half-closed switch
                                      // signal of right rear side door
} AllDoorSwitchInReq_t;
#endif

#ifndef Rte_TypeDef_EgoVehicleInfo_t
#define Rte_TypeDef_EgoVehicleInfo_t
typedef struct {
    LBS_SignalHeader_t sSigHeader;
    float32
        fegoVelocity_mps; /*the ego vehicle longitudinal velocity ,unit:m/s*/
    float32 fVaregoVelocity_mps;   /*the ego vehicle longitudinal velocity
                                      variance,unit:m/s*/
    float32 fegoAcceleration_mps2; /*the ego vehicle longitudinal acceleration
                                      ,unit:m/s^2*/
    float32 fVaregoAcceleration_mps2; /*the ego vehicle longitudinal
                                         acceleration variance,unit:m/s^2*/
    float32 fYawRate_radps;           /*the ego vehicle yaw rate ,unit:rad/s*/
    // float32 fCurve_1pm;               /*the ego vehicle driver curve
    // ,unit:1/m*/ float32 DrvIntCurve_1pm;          /*the ego vehicle driver
    // curve ,unit:1/m*/
    float32 fLatAccel_mps2; /*the ego vehicle Lateral Acceleration,unit:m/s^2*/
    // float32 fSlipAngle_rad; /*the ego vehicle slip angle ,unit:rad*/
    float32 fSelfSteering_rad; /*the ego vehicle steering angle ,unit:rad*/
    // AllDoorSwitchInReq_t uAllDoorSwitch;  // the switch signals of all doors
} EgoVehicleInfo_t;
#endif

#ifndef Rte_TypeDef_LBS_RoadType_t
#define Rte_TypeDef_LBS_RoadType_t
typedef struct {
    float32 fRoadTypeConf;
    uint8 uiRoadType;
} LBS_RoadType_t;
#endif

#ifndef Rte_TypeDef_EMRoad_t
#define Rte_TypeDef_EMRoad_t
typedef struct {
    /*CurveInfo_t */
    float32 fCurveRadius_met; /*The curvature radius of the current driver road
                                 ,unit:m*/
    float32
        fDrivenCurveRadius_met; /*The curvature radius of the ego driver curve*/

    /*FusedEgoCourse_t */
    float32 fC0FusedEgo_1pm;       /*undetermined */
    float32 fC1FusedEgo_1pm2;      /*undetermined */
    float32 fYawAngleFusedEgo_rad; /*undetermined */
    float32 fConfEgo_per;          /*undetermined */

    // Scale cofficients
    // f = (1/2) * C0 * x^2 + (1/6) * C1 * x^3 + Offset
    /*FusedRoadBorder_t */
    float32 fC0Fused_1pm;       /*Polynomial coefficient that index is two  */
    float32 fC1Fused_1pm2;      /*Polynomial coefficient that index is three */
    float32 fYOffsetFused_met;  /*Polynomial coefficient that index is zero ->
                                   the offset of the Y Coordinate */
    float32 fYawAngleFused_rad; /*undetermined */
    float32 fConfCurvature_per; /*undetermined */
    float32 fConfYOffset_per;   /*The confidence of polynomial coefficient that
                                   index is zero*/
    float32 fYOffsetFusedOppBorder_met; /*Opposite polynomial coefficient that
                                           index is zero -> the Opposite offset
                                           of the Y Coordinate*/
    float32 fConfYOppOffset_per;        /*The confidence of opposite polynomial
                                           coefficient that index is zero*/

    /*LaneEstimation_t */
    float32 fConfAdjacentLanes_per; /*undetermined */
    float32 fConfOppositeLanes_per; /*undetermined */
    float32 fLaneWidth_met;         /*The ego lane width ,unit:m*/
    sint8 iNumOfAdjacentLanes_nu;   /*undetermined */
    sint8 iNumOfOppositeLanes_nu;   /*undetermined */

    /*GridBorderData_t */
    float32 BorderEstmGridData_fC0_1pm;       /*undetermined */
    float32 BorderEstmGridData_fC1_1pm2;      /*undetermined */
    float32 BorderEstmGridData_fYawAngle_rad; /*undetermined */
    float32 BorderEstmGridData_fYDist_met;    /*undetermined */
    float32 BorderEstmGridData_fMaxX_met;     /*undetermined */
    float32 BorderEstmGridData_fConf_per;     /*undetermined */

    // sint8 iGuardrailTrkRestID; /*deleted*/
    LBS_RoadType_t RoadType;
} EMRoad_t;
#endif

#ifndef Rte_TypeDef_LBSSystemParam_t
#define Rte_TypeDef_LBSSystemParam_t
typedef struct {
    boolean bBSDFunctionActive;       /* BSD function active flag */
    boolean bBSDFunctionOutputActive; /* BSD function output flag */
    boolean bLCAFunctionActive;       /* LCA function active flag */
    boolean bLCAFunctionOutputActive; /* LCA function output flag */
    boolean bOSEFunctionActive;       /* OSE function active flag */
    boolean bOSEFunctionOutputActive; /* OSE function output flag */
    boolean bRCWFunctionActive;       /* RCW function active flag */
    boolean bRCWFunctionOutputActive; /* RCW function output flag */
    float32 fCycletime_s;             /* Current task cycle time,unit: s*/
} LBSSystemParam_t;
#endif

#ifndef Rte_TypeDef_LBSNVRAM_t
#define Rte_TypeDef_LBSNVRAM_t
typedef struct {
    boolean LBS_Nb_BSDPowerOffSwitchState_nu;  // the BSD function switch state
                                               // when power is off
    boolean LBS_Nb_LCAPowerOffSwitchState_nu;  // the LCA function switch state
                                               // when power is off
    boolean LBS_Nb_DOWPowerOffSwitchState_nu;  // the DOW function switch state
                                               // when power is off
    boolean LBS_Nb_RCWPowerOffSwitchState_nu;  // the RCW function switch state
                                               // when power is off
} LBSNVRAM_t;
#endif

#ifndef Rte_TypeDef_LBSSupportInfoInput_t
#define Rte_TypeDef_LBSSupportInfoInput_t
typedef struct {
    // RCW
    boolean LBSRCWFailure;
    boolean LBSLeftTurnLightOpen;
    boolean LBSRightTurnLightOpen;
    boolean LBSGearInReverse;
    // BSD
    boolean LBSBSDFailure;
    // LCA
    boolean LCAFailure;
    boolean GearInReverseAndParking;
    boolean VehicleSpdDisplayValid;
    boolean ActGearValid;
    uint8 uTurnLightReqSt;
    uint8 CDCS11_VoiceMode;
    // DOW
    boolean DOWFailure;
    boolean bDoor_FR;
    boolean bDoor_FL;
    boolean bDoor_RR;
    boolean bDoor_RL;
    boolean bDOW_PowerMode_3Mins;
} LBSSupportInfoInput_t;
#endif

#ifndef Rte_TypeDef_LBSInReq_st
#define Rte_TypeDef_LBSInReq_st
typedef struct {
    EMGenObjList_st GenObjList;      /* General Radar Object information */
    EMSRRObjList_st SRRObjList;      /* SRR Radar Object information */
    EgoVehicleInfo_t EgoVehInfo;     /* Vehicle dynamic information */
    EMRoad_t Road;                   /* Road information */
    LBSSystemParam_t LBSSystemParam; /* LBS System Parameter */
    LBSNVRAM_t LBS_Ns_NVRAM_nu;
    LBSSupportInfoInput_t LBSSupportInfo;
} LBSInReq_st;
#endif

/*****************************************************************************
                TYPEDEFS : PARAMETER
*****************************************************************************/
typedef struct {
    float32 LBS_Kf_LatPos_met;  /*the radar sensor mounting Y position ,unit:m*/
    float32 LBS_Kf_LongPos_met; /*the radar sensor mounting X position ,unit:m*/
    float32 LBS_Kf_VertPos_met; /*the radar sensor mounting Z position ,unit:m*/
    float32 LBS_Kf_LongPosToCoG_met; /*the radar sensor mounting position to COG
                                        longitudinal distance ,unit:m*/
    float32 LBS_Kf_PitchAngle_rad;   /*the radar sensor mounting Pitch angle
                                        ,unit:rad*/
    float32 LBS_Kf_Orientation_rad;  /*TODO*/
    float32
        LBS_Kf_RollAngle_rad; /*the radar sensor mounting Roll angle ,unit:rad*/
    float32 LBS_Kf_YawAngle_rad; /*the radar sensor mounting Yaw angle
                                    ,unit:rad,range[-pi,+pi]*/
} LBS_SensorMounting_t;

typedef struct {
    LBS_SensorMounting_t LBS_Kf_SensorLeft_nu;
    LBS_SensorMounting_t LBS_Kf_SensorRight_nu;
} LBS_SensorMounting_st;

typedef struct {
    float32
        LBS_Kf_WheelBase_met; /*The distance between the center of the front
                                 wheel and the center of the rear wheel,unit:m*/
    float32 LBS_Kf_VehicleWidth_met;        /*the vehicle body width,unit:m*/
    float32 LBS_Kf_VehicleLength_met;       /*the vehicle body length,unit:m*/
    float32 LBS_Kf_VehCenter2FrontAxis_met; /*the vehicle center to front axis
                                               center distance*/
    float32 LBS_Kf_OverhangFront_met; /*the length of vehicle front overhang*/
} LBS_VehParameter_t;

typedef struct {
    boolean LBS_Kb_NCAPActive_nu;        /*TODO*/
    float32 LBS_Kf_SoTDelayThresh_s;     /*the SOT object delay time max
                                            threshold,unit:s */
    float32 LBS_Kf_SoTMinWarnDuration_s; /*the short warning duration time
                                            threshold,unit:s */
    float32 LBS_Kf_SoTCutoffSpeed_mps;   /*the object cutoff speed threshold of
                                            SOT check ,unit:m/s */
} LBSParameter_t;

typedef struct {
    boolean LBS_Kb_BSDNCAPActive_nu;        /*TODO*/
    float32 LBS_Kf_BSDSoTDelayThresh_s;     /*the SOT object delay time max
                                               threshold,unit:s */
    float32 LBS_Kf_BSDSoTMinWarnDuration_s; /*the short warning duration time
                                               threshold,unit:s */
    float32 LBS_Kf_BSDSoTCutoffSpeed_mps; /*the object cutoff speed threshold of
                                             SOT check ,unit:m/s */

    float32 LBS_Kf_BSDVelMinWarnDisable_mps; /*BSD function min disable velocity
                                                threshold,unit:m/s*/
    float32 LBS_Kf_BSDVelMinWarnEnable_mps;  /*BSD function min enable velocity
                                                threshold,unit:m/s*/
    float32 LBS_Kf_fTimerThresholdForSOTBlocking;
} LBSBSDWarningParameter_t;

typedef struct {
    float32 LBS_Kf_BSDXmin_met; /*the BSD Warning Zone X direction min position
                                   unit:m*/
    float32 LBS_Kf_BSDXmax_met; /*the BSD Warning Zone X direction max position
                                   unit:m*/
    float32 LBS_Kf_BSDYmin_met; /*the BSD Warning Zone Y direction min position
                                   unit:m*/
    float32 LBS_Kf_BSDYmax_met; /*the BSD Warning Zone Y direction max position
                                   unit:m*/
    float32 LBS_Kf_BSDHysteresisX_met;    /*the BSD Warning Zone X direction
                                             hysteresis value unit:m*/
    float32 LBS_Kf_BSDHysteresisYmin_met; /*the BSD Warning Zone Y direction
                                             hysteresis value unit:m*/
    float32 LBS_Kf_BSDHysteresisYmax_met; /*the BSD Warning Zone Y direction
                                             hysteresis value unit:m*/
} LBSBsdZone_t;

typedef struct {
    LBSBSDWarningParameter_t LBS_Ks_BSDWarnParameter_nu;
    LBSBsdZone_t LBS_Ks_BSDZoneParameter_nu;
} LBSBSDParameter_t;

typedef struct {
    float32 LBS_Kf_LCAZoneXMid_met; /*the LCA Warning Zone X direction middle
                                       position unit:m*/
    float32 LBS_Kf_LCAZoneXMin_met; /*the LCA Warning Zone X direction max
                                       position unit:m*/
    float32 LBS_Kf_LCAZoneYMinNear_met; /*the LCA Warning Zone Y direction min
                                           near position unit:m*/
    float32 LBS_Kf_LCAZoneYMinFar_met;  /*the LCA Warning Zone Y direction min
                                           far position unit:m*/
    float32 LBS_Kf_LCAZoneYMaxNear_met; /*the LCA Warning Zone Y direction max
                                           near position unit:m*/
    float32 LBS_Kf_LCAZoneYMaxFar_met;  /*the LCA Warning Zone Y direction max
                                           far position unit:m*/
} LBSLCAZone_t;

typedef struct {
    float32 LBS_Kf_LCAVehicleWidth_met;       /*the vehicle body width,unit:m*/
    float32 LBS_Kf_LCAVoWnMinWarnEnable_mps;  /*the min enable velocity of LCA
                                                 warning*/
    float32 LBS_Kf_LCAVownMinWarnDisable_mps; /*the max enable velocity of LCA
                                                 warning*/
    float32
        LBS_Kf_LCATTCThreshold_s; /*the single TTC Threshold is applied when the
                                     warning configuration is set to custom*/
    float32 LBS_Kf_LCATTCThreshLowRelSpeed_s; /*the low TTC threshold is applied
                                                 when the warning configuration
                                                 is set to default*/
    float32 LBS_Kf_LCATTCThreshMidRelSpeed_s; /*the mid TTC threshold is applied
                                                 when the warning configuration
                                                 is set to default*/
    float32
        LBS_Kf_LCATTCThreshHighRelSpeed_s; /*the high TTC threshold is applied
                                              when the warning configuration is
                                              set to default*/
    float32 LBS_Kf_LCAMinTTCHysteresis_s;  /*the compensation TTC threshold is
                                              applied when the LCA warning was
                                              activated in last cycle*/
    float32
        LBS_Kf_LCABridgeWarningTime_s; /*add the additional BridgeWarningTime
                                          which bridges the warning for
                                          consecutive objects*/
    float32 LBS_Kf_LCAMaxLCARange_met; /*the max LCA range threshold*/
    float32
        LBS_Kf_LCAMaxLCACurveRadius_met; /*the max LCA curve radius threshold*/
    uint8 LBS_Ku_LCAWarningDuration_nu;  /*the LCA warning configuration: off,
                                            custom and default*/
    LBSLCAZone_t LBS_Ks_LCAZone_nu;      /*LCA zone*/
} LBSLCAParam_st;

typedef float32 LBSOSETTCThresholdParam_t[3];
typedef float32 LBSOSEYMinBreakthroughParam_t[2];
typedef float32 LBSOSEYMaxBreakthroughParam_t[2];
typedef float32 LBSOSEYMinBreakthroughMarginParam_t[2];
typedef float32 LBSOSEYMaxBreakthroughMarginParam_t[2];
typedef float32 LBSOSEXBreakthroughLineParam_t[2];
typedef float32 LBSOSETargetRangeMaxParam_t[3];
typedef float32 LBSOSEMinTimeParam_t[3];
typedef float32 LBSOSEMaxTimeParam_t[3];

typedef struct {
    LBSOSETTCThresholdParam_t
        LBS_Ka_OSETTCThreshold_s;  // TTC thresholds of three levels; Value:
                                   // {2.5f, 1.5f, 1.2f}
    LBSOSEYMinBreakthroughParam_t
        LBS_Ka_OSEYMinBreakthrough_met;  // Two Y-axis min breakthrough; Value:
                                         // {0.0f, 0.0f}
    LBSOSEYMaxBreakthroughParam_t
        LBS_Ka_OSEYMaxBreakthrough_met;  // Two Y-axis max breakthrough; Value:
                                         // {1.0f, 1.0f}
    LBSOSEYMinBreakthroughMarginParam_t
        LBS_Ka_OSEYMinBreakthroughMargin_met;  // Two Y-axis min breakthrough
                                               // margin; Value: {0.5f, 0.5f}
    LBSOSEYMaxBreakthroughMarginParam_t
        LBS_Ka_OSEYMaxBreakthroughMargin_met;  // Two Y-axis max breakthrough
                                               // margin; Value: {0.5f, 0.5f}
    LBSOSEXBreakthroughLineParam_t
        LBS_Ka_OSEXBreakthroughLine_met;  // Distance between vehicle rear edge
                                          // and two X-axis breakthrough lines;
                                          // Value: {1.0f, 3.0f}
    float32 LBS_Kf_OSEVEgoMax_mps;        // The max ego vehicle speed of OSE is
                                          // actived; Value: 0.1f m/s
    float32 LBS_Kf_OSEVEgoMin_mps;        // The min ego vehicle speed of OSE is
                                          // actived; Value: -0.1f m/s
    float32 LBS_Kf_OSEVTargetMin_mps;  // The min target vehicle speed of OSE is
                                       // actived; Value: 3.0f m/s
    float32 LBS_Kf_OSEVTargetMax_mps;  // The max target vehicle speed of OSE is
                                       // actived; Value: 40.0f m/s
    LBSOSEMinTimeParam_t
        LBS_Ka_OSEMinTime_s;  // The min time thresholds of three levels OSE
                              // warning; Value: {1.0f, 1.0f, 1.0f}
    LBSOSEMaxTimeParam_t
        LBS_Ka_OSEMaxTime_s;  // The max time thresholds of three levels OSE
                              // warning; Value: {1.5f, 1.5f, 1.5f}
    float32 LBS_Kf_OSEMaxHeadingAngle_deg;  // The max heading threshold;
                                            // Value: 60.0f
    float32 LBS_Kf_OSEMinHeadingAngle_deg;  // The min heading threshold; Value:
                                            // -70.0f
    LBSOSETargetRangeMaxParam_t
        LBS_Ka_OSETargetRangeMax_met;  // The max range thresholds of three
                                       // levels OSE warning; Value:
                                       // {42.0f, 30.0f, 20.0f}
    boolean LBS_Kb_OSEEnableObjAdaptiveBreakthrough_nu;  // Value: TRUE
    boolean LBS_Kb_OSEActive_nu;  // Flag of BSW parameters(driver); Value: TRUE
} LBSOSEParameter_t;

typedef struct {
    // stationary obj checking threshold
    float32 fEgoVelStationaryCheck;  // 0.1f
    // RCW Warning Level2 TTCThreshold
    float32 fTTCThresholdLevel2;  // 0.f
    // suppression condition
    float32 fSuppressEgoLongVelMin;    // 0 m/s
    float32 fSuppressEgoLongVelMax;    // 38.88 m/s
    float32 fSuppressEgoLatVelMax;     // - m/s
    float32 fSuppressEgoLatAcceMax;    // - m/s2
    float32 fSuppressRCWBlockingTime;  // 2000 ms
    // corridor blocked
    float32 fCorrBlockedActiveOccMin;    // 0.15f
    float32 fCorrBlockedInActiveOccMin;  // 0.30f
    // choose occupancy
    float32 fOppoOccuAddAdjust;  // 0.35f
    // object warning decision
    float32 fWarnDecisionVrel2TrajMax;          // 1.f
    float32 fWarnDecisionAssocProbMin;          // 0.8f
    float32 fWarnDecisionTTCAdjustThreshold;    // 1.4f
    float32 fWarnDecisionArelXMaxTTCAdjust;     // -0.5f
    float32 fWarnDecisionArelXMinTTCAdjust;     // -1.5f
    float32 fWarnDecisionArelXTTCReduceFactor;  // 0.1f
    // object relevent check para
    float32 fReleventDeactCorrOccuThreshold;  // 0.2f
    float32 fReleventDist2CourseMax;          // -0.1f
    float32 fReleventDist2CourseMin;          // -0.5f
    float32 fReleventMaxXOppSideRelevent;     // -5.f
    float32 fReleventMinXOppSideRelevent;     // -15.f
    uint16 fReleventLifeCnt;                  // 50u
    float32 fReleventMaxDist;                 // -5.f
    float32 fReleventMinXMovement;            // 5.f
    // Corridor check para
    float32 fCorridorMinTimeInCorridorThresh;  // 0.5f
    uint8 uCorridorHitCntThresh;               // 15U
    float32 fCycletime_s;                      // Current task cycle time
    float32 fMinTimeInCorridorThresh;          // 0.5f
    float32 fCorrHitCntSmallObjWidth;          // 2.5f
    // RCW warning range
    float32 fRCWCorrMinSDISIX;  // -50.f
    float32 fRCWCorrMaxSDISIX;  // 0.f
    float32 fRCWCorridorXMid;   // -20.f;
    float32 fRCWCorridorXMin;   // -40.f;
    // General
    float32 fVrelTTCMin;           // 15 * 0.27777
    float32 fVrelTTCMax;           // 30 * 0.27777
    float32 fTTCThreshVrelMin;     // 0.f
    float32 fTTCThreshVrelMax;     // 1.4f
    float32 fMinHeadingAngle;      // 2.5f
    float32 fMaxHeadingAngle;      // 6.0f
    float32 fCorrOccWarnEnable;    // 0.3f
    float32 fCorrOccDropThresh;    // 0.1f
    float32 fCorrOccPickupThresh;  // 0.28f
    float32 fCorrOccHitThresh;     // 0.2f
} LBSRCWParameter_t;

typedef struct {
    LBS_SensorMounting_st LBS_Ks_SensorMounting_nu;
    LBS_VehParameter_t LBS_Ks_VehParameter_nu;
    LBSParameter_t LBS_Ks_LBSParameter_nu;
    LBSBSDParameter_t LBS_Ks_BSDParameter_nu;
    LBSLCAParam_st LBS_Ks_LCAParameter_nu;
    LBSOSEParameter_t LBS_Ks_OSEParameter_nu;
    LBSRCWParameter_t LBS_Ks_RCWParameter_nu;
} LBSParam_st;
/*****************************************************************************
                TYPEDEFS : OUTPUT
*****************************************************************************/
#ifndef Rte_TypeDef_bWarnLevel_t
#define Rte_TypeDef_bWarnLevel_t
typedef struct {
    boolean bPreWarnActive;
    boolean bAcuteWarnActive;
    boolean bDoorLockingActive;
} bWarnLevel_t;
#endif

#ifndef Rte_TypeDef_LBSFunState_t
#define Rte_TypeDef_LBSFunState_t
typedef struct {
    boolean bBSDWarning;
    boolean bBSDWarningLeft;
    boolean bBSDWarningRight;
    boolean bLCAWarning;
    boolean bLCAWarningLeft;
    boolean bLCAWarningRight;
    bWarnLevel_t bOSEWarning;
    bWarnLevel_t bOSEWarningLeft;
    bWarnLevel_t bOSEWarningRight;
    uint8 uRCWWarning;
    float32 fTTC_s;
} LBSFunState_t;
#endif

#ifndef Rte_TypeDef_LBSState_t
#define Rte_TypeDef_LBSState_t
typedef enum {
    LBS_INIT,
    LBS_OK,
} LBSState_t;
#endif

#ifndef Rte_TypeDef_LBS_LastCycleStates_t
#define Rte_TypeDef_LBS_LastCycleStates_t
typedef struct {
    uint8 eRoadType; /* current road type*/
    // LBSState_t LBSState;
    boolean bBSDFunctionActive;
    boolean bLCAFunctionActive;
    boolean bOSEFunctionAcitve;
    boolean bEgoSpeedConditionBSD;
    boolean bEgoSpeedConditionLCA;
    boolean bEgoSpeedConditionRPCS;
    boolean bEgoSpeedConditionOSE;
} LBS_LastCycleStates_t;
#endif

#ifndef Rte_TypeDef_ParameterObjSel_t
#define Rte_TypeDef_ParameterObjSel_t
typedef struct {
    float32 fVegoMin;
    float32 fVegoMax;
    float32 fVTargetMin;
    float32 fTargetRangeMax;
    float32 fXMinBreakThrough;
    float32 fXMaxBreakthrough;
} ParameterObjSel_t;
#endif

#ifndef Rte_TypeDef_LBS_Globals_t
#define Rte_TypeDef_LBS_Globals_t
typedef struct {
    ParameterObjSel_t ParameterObjSel;
    LBS_LastCycleStates_t LastCycleStates;
    float32 fSensorOffsetToRear_met;
    float32 fSensorOffetToSide_met;
    float32 fMaxSpeedOverGround_mps;
    boolean bInnerSensorDriven;   /* boolean value to distinguish between inner
                                     and outer sensor*/
    boolean bInnerSensorSteering; /* boolean value to distinguish between inner
                                     and outer sensor*/
    boolean bFCTObjOutput;
    boolean bBSDFunctionOutput;
    boolean bLCAFunctionOutput;
    boolean bRPCSFunctionOutput;
    boolean bOSEFunctionOutput[LBS_OSE_NUM_OF_WARN_LEVELS];
} LBS_Globals_t;
#endif

typedef struct {
    // DOW
    boolean bDOWPowerModeDone;
    uint8 uDOWRightActivation;
    uint8 uDOWLeftActivation;
    uint8 uDOWState;
    uint8 uDOWSystemFaultStatus;
    uint8 uDOWObjIndex;
    sint32 siDOWLeftHighlightID_nu;
    sint32 siDOWRightHighlightID_nu;

    // RCW
    uint8 uRCWwarningReq;
    uint8 uRCWWarnState;
    uint8 uRCWSystemFaultStatus;
    boolean bRCWState;
    uint8 uRCWObjIndex;
    sint32 siRCWHighlightID_nu;

    // LCABSD
    uint8 uLCABSDOnOffSet;
    uint8 uLCASystemFaultStatus;
    uint8 uLCALeftWarnSt;
    uint8 uLCARightWarnSt;
    uint8 uLCABSDObjIndex;
    sint32 siLCALeftHighlightID_nu;
    sint32 siLCARightHighlightID_nu;
} LBS_CAN_output;

#ifndef Rte_TypeDef_LBSOutPro_t
#define Rte_TypeDef_LBSOutPro_t
typedef struct {
    LBSFunState_t LBSFunState;
    LBS_Globals_t LBSGlobals;
    LBSNVRAM_t LBS_Ns_NVRAM_nu;
    LBS_CAN_output LBSCANOutputs;
} LBSOutPro_t;
#endif

/*****************************************************************************
                TYPEDEFS : DEBUG
*****************************************************************************/
#ifndef Rte_TypeDef_LBSBSDObjDirectionDebug_t
#define Rte_TypeDef_LBSBSDObjDirectionDebug_t
typedef enum {
    LBSBSDLeftSensorObj = 1,
    LBSBSDRightSensorObj = 2
} LBSBSDObjDirectionDebug_t;
#endif

#ifndef Rte_TypeDef_LBSBSDInfoDebug_t
#define Rte_TypeDef_LBSBSDInfoDebug_t
typedef struct {
    float32 fSoTDelayTime_s; /* current SOT object delay time,unit:s*/
    float32
        fRearConf_nu; /* The object classify to rear confidence,unit:percent*/
    float32
        fBSDZoneObjXmin_met; /* BSD Zone length per object using NHTSA formula*/
    uint8 ubAppearance_nu;   /* Zone appearance of current object :front ,side,
                                rear,unit:NULL*/
    uint8 ubHitsInFront_nu;  /* The ratio of object appearance from the front
                                direction,unit:NULL*/
    uint8 ubHitsInSide_nu;   /* The ratio of object appearance from the side
                                direction,unit:NULL*/
    uint8 ubHitsInRear_nu;   /* The ratio of object appearance from the rear
                                direction,unit:NULL*/
    uint8 ubGrdHitCounter_nu; /* Counter which increase if the object hits the
                                 guardrail next to the vehicle,unit:NULL*/
    uint8 ubBehindGrdCounter_nu; /* Counter which increase if the object is
                                    behind the guardrail,unit:NULL*/
    uint8
        ubClass_nu; /* The object classification:vehicle, static...,unit:NULL*/
    uint8 ubOwnLaneCounter_nu;  /* Counter which is increased when an object is
                                   assumed to be on the own lane,unit:NULL*/
    boolean bInBSDZone;         /*The object in BSD zone indicator flag */
    boolean bInSOTZone;         /*The object in SOT zone indicator flag */
    boolean bInSOTZonePrevious; /*The last cycle object in SOT zone indicator
                                   flag */
    boolean bObjectAndZoneOverlap; /*whether Overlap ratio of object and BSD
                                      zone is enough flag*/
    boolean bBSDRelevant;         /*The object relevance flag of BSD function */
    boolean bBSDWarning;          /*The object warning flag */
    boolean bUpdatedRecently;     /*Whether the current object is sufficiently
                                     updated(measured) flag in the last two cycle*/
    boolean bUpdatedRecentlyWeak; /*Whether the current object is sufficiently
                                     updated(measured) flag at least one of last
                                     two cycle*/
    boolean bLivedLongEnough;     /*Flag which shows that this object lived long
                                     enough */
    boolean
        bQualityEnough; /*Flag which shows that this object quality enough */
    boolean bObjectOnOwnlane; /*Object is on own lane flag*/
    boolean bCreateBehindGRD; /*if the object is relevance with the guardrail in
                                 the first cycle time*/
    boolean bObjectBehindGRD; /* Flag whether object is behind guardrail*/
    boolean bSoTDelayActive;  /*SoT delay flag of the object*/
    boolean bShortWarn;       /*ShortWarning flag of the object*/
    boolean bIsSoT;           /*SoT flag of the object*/
    boolean bFastSoT;         /*Fast SoT flag of the object*/
    boolean bPlausibility;    /*Plausibility check flag of the object*/
    boolean bPossibleWrappedObj; /*Possible wrapped flag of the object*/
    LBSBSDObjDirectionDebug_t eObjDirection; /*The object on the right side or
                                                left side flag by sensor */
} LBSBSDInfoDebug_t;
#endif

typedef LBSBSDInfoDebug_t LBSBSDInfoArrayDebug[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBS_BSDZoneParameter_t
#define Rte_TypeDef_LBS_BSDZoneParameter_t
typedef struct {
    float32 fBSDZoneXmin_met;         /* the BSD Warning Zone X direction min
                                         position,unit:m*/
    float32 fBSDZoneXmax_met;         /* the BSD Warning Zone X direction max
                                         position,unit:m*/
    float32 fBSDZoneYmin_met;         /* the BSD Warning Zone Y direction min
                                         position,unit:m*/
    float32 fBSDZoneYmax_met;         /* the BSD Warning Zone Y direction max
                                         position,unit:m*/
    float32 fBSDZoneXminWithHyst_met; /* the BSD Warning Zone X direction min
                                         position with hysteresis,unit:m*/
    float32 fBSDZoneXmaxWithHyst_met; /* the BSD Warning Zone X direction max
                                         position with hysteresis,unit:m*/
    float32 fBSDZoneYminWithHyst_met; /* the BSD Warning Zone Y direction min
                                         position with hysteresis,unit:m*/
    float32 fBSDZoneYmaxWithHyst_met; /* the BSD Warning Zone Y direction max
                                         position with hysteresis,unit:m*/
} LBS_BSDZoneParameter_t;
#endif

#ifndef Rte_TypeDef_LBS_BSDGlobal_BSDZone_st
#define Rte_TypeDef_LBS_BSDGlobal_BSDZone_st
typedef struct {
    LBS_BSDZoneParameter_t BSDZoneParameterLeft;
    LBS_BSDZoneParameter_t BSDZoneParameterRight;
} LBS_BSDGlobal_BSDZone_st;
#endif

#ifndef Rte_TypeDef_LBS_BSDScenarioObserver_t
#define Rte_TypeDef_LBS_BSDScenarioObserver_t
typedef struct {
    uint8 uNumberSoTObjs_nu;          /*current cycle SoT Object*/
    uint8 uNumberSoTObjsLastCycle_nu; /*last cycle SoT Object*/
} LBS_BSDScenarioObserver_t;
#endif

#ifndef Rte_TypeDef_LBS_BSD_Globals_t
#define Rte_TypeDef_LBS_BSD_Globals_t
typedef struct {
    LBS_BSDScenarioObserver_t ScenarioObserver;
    float32 fVxThreshMovStat_mps; /* real Vx speed threshold for SoTs,unit:m/s*/
    float32 fAngleFrontSector_deg;    /* Angle of Front Sector Cut which is
                                         adjusted by turning angle,unit:degree*/
    float32 fBSDZoneXminStatic_met;   /* Actual Xmin (bottom edge) of the BSD
                                         Zone,unit:m*/
    float32 fBSDZoneXmin_met;         /* the BSD Warning Zone X direction min
                                         position,unit:m*/
    float32 fBSDZoneXmax_met;         /* the BSD Warning Zone X direction max
                                         position,unit:m*/
    float32 fBSDZoneYmin_met;         /* the BSD Warning Zone Y direction min
                                         position,unit:m*/
    float32 fBSDZoneYmax_met;         /* the BSD Warning Zone Y direction max
                                         position,unit:m*/
    float32 fBSDZoneXminWithHyst_met; /* the BSD Warning Zone X direction min
                                         position with hysteresis,unit:m*/
    float32 fBSDZoneXmaxWithHyst_met; /* the BSD Warning Zone X direction max
                                         position with hysteresis,unit:m*/
    float32 fBSDZoneYminWithHyst_met; /* the BSD Warning Zone Y direction min
                                         position with hysteresis,unit:m*/
    float32 fBSDZoneYmaxWithHyst_met; /* the BSD Warning Zone Y direction max
                                         position with hysteresis,unit:m*/
    float32 fMinAssocProbFront_nu;    /* Quality check association threshold of
                                         object appear from front*/
    float32 fMinAssocProbSideRear_nu; /* Quality check association threshold of
                                         object appear from side and rear*/
    float32 fMinXmoved_met; /* the object movement distance min threshold  of
                               Plausibility check ,unit:m*/
    float32 fBSDWarnActiveLeftDistX_met;  /* the left side BSD warning most
                                             closest object distance x,unit:m  */
    float32 fBSDWarnActiveRightDistX_met; /* the right side BSD warning most
                                             closest object distance x,unit:m */
    boolean bBSDWarnActive;     /* boolean value to distinguish whether BSD side
                                   warning is active*/
    boolean bBSDWarnActiveLeft; /* boolean value to distinguish whether BSD left
                                   side warning is active*/
    boolean bBSDWarnActiveRight; /* boolean value to distinguish whether BSD
                                    right side warning is active*/
    boolean bBSDWarnActiveLeftLastCycle; /* boolean value to distinguish whether
                                            BSD left side warning is active in
                                            last cycle*/
    boolean bBSDWarnActiveRightLastCycle; /* boolean value to distinguish
                                             whether BSD right side warning is
                                             active in last cycle*/
    uint8 bBSDWarnActiveLeftID_nu;  /* the left side BSD warning most closest
                                       object id */
    uint8 bBSDWarnActiveRightID_nu; /* the right side BSD warning object id */
    boolean bInnerSensorDriven;   /* boolean value to distinguish between inner
                                     and outer sensor*/
    boolean bInnerSensorSteering; /* boolean value to distinguish between inner
                                     and outer sensor*/
    boolean bMultiObjectSuppression;    /*more than two objects active BSD, then
                                           suppression this case*/
    uint8 uActivationObjectCounterLeft; /*total obj activation counter in 3s*/
    uint8 uActivationObjectCounterLeftID
        [LBS_INPUT_OBJECT_NUMBER]; /*the ID info from the activation obj*/
    uint8 uActivationObjectCounterLeftIDLastCycle
        [LBS_INPUT_OBJECT_NUMBER]; /*the ID info from the activation obj in last
                                      cycle*/
    boolean
        bFirstAppearObjStartDurationLeft; /*3s start add counter for multi obj*/
    boolean bBsdEnterStandbyformultiObjLeft; /*5s suppression state machine*/
    // right zone
    uint8 uActivationObjectCounterRight; /*total obj activation counter in 3s*/
    uint8 uActivationObjectCounterRightID
        [LBS_INPUT_OBJECT_NUMBER]; /*the ID info from the activation obj*/
    uint8 uActivationObjectCounterRightIDLastCycle
        [LBS_INPUT_OBJECT_NUMBER]; /*the ID info from the activation obj in last
                                      cycle*/
    boolean bFirstAppearObjStartDurationRight; /*3s start add counter for multi
                                                  obj*/
    boolean bBsdEnterStandbyformultiObjRight;  /*5s suppression state machine*/
} LBS_BSD_Globals_t;
#endif

#ifndef Rte_TypeDef_LBS_BSDZone_ObjPar
#define Rte_TypeDef_LBS_BSDZone_ObjPar
typedef struct {
    float32 fZoneXmin_met; /* The current object BSDZone X min position,unit:m*/
    float32 fZoneXmax_met; /* The current object BSDZone X min position,unit:m*/
    float32 fZoneYmin_met; /* The current object BSDZone Y max position,unit:m*/
    float32 fZoneYmax_met; /* The current object BSDZone Y max position,unit:m*/
} LBS_BSDZone_ObjPar;
#endif

typedef LBS_BSDZone_ObjPar LBS_BSDZone_ObjPar_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBS_BSD_Warn_Decide_Debug_t
#define Rte_TypeDef_LBS_BSD_Warn_Decide_Debug_t
typedef struct {
    uint8 BSD_WarnDecide_sObjID;
    boolean BSD_WarnDecide_ObjStable;
    boolean BSD_WarnDecide_LCAMirrorObject;
    boolean BSD_WarnDecide_LCAMirrorFrontObject;
    boolean BSD_WarnDecide_InBSDZone;
    boolean BSD_WarnDecide_bBSDRelevant;
    boolean BSD_WarnDecide_bLivedLongEnough;
    boolean BSD_WarnDecide_bObjectOnOwnlane;
    boolean BSD_WarnDecide_bObjectBehindGRD;
    boolean BSD_WarnDecide_bPlausibility;
    boolean BSD_WarnDecide_bUpdatedRecently;
    uint8 BSD_WarnDecide_ubGrdHitCounter_nu;
    boolean BSD_WarnDecide_bQualityEnough;
    boolean BSD_WarnDecide_bSoTDelayActive;
    boolean BSD_WarnDecide_bShortWarn;
    boolean BSD_WarnDecide_bFastSoT;
    boolean BSD_WarnDecide_bObjectAndZoneOverlap;
} LBS_BSD_Warn_Decide_Debug_t;
#endif

typedef LBS_BSD_Warn_Decide_Debug_t
    LBS_BSD_Warn_Decide_Debug_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBSBSDDebug_t
#define Rte_TypeDef_LBSBSDDebug_t
typedef struct {
    uint32 uiVersionNumber;
    // LBSBSDInfoArrayDebug BSDObjInfo;
    // LBS_BSDGlobal_BSDZone_st BSDGlobalsZones;
    // LBS_BSD_Globals_t BSD_Globals;
    // LBS_BSDZone_ObjPar_Array BSDZoneObjParList;
    // LBS_BSD_Warn_Decide_Debug_Array LBS_BSDWarnDecideList_Debug;
} LBSBSDDebug_t;
#endif

#ifndef Rte_TypeDef_LBS_LCAWarnInfo_t
#define Rte_TypeDef_LBS_LCAWarnInfo_t
typedef struct {
    float32 fCriticalTTC_s;
    float32 fXObjectWarning_met;  // Object's longitudinal relative distance of
                                  // LCA warning condition
    uint8 uLCAWarningID_nu;       // LCA warning object ID
    boolean bLCAWarnActive;  // LCA warning flag for LCA function, on if any
                             // object is warning
    boolean bLCAWarningLastCycle;  // LCA warning status from last cycle
    boolean bLCAWarnActiveLeft;    // LCA left warning flag for LCA function, on
                                   // if any
                                   // object is warning
    boolean bLCAWarnActiveRight;  // LCA right warning flag for LCA function, on
                                  // if any
                                  // object is warning
} LBS_LCAWarnInfo_t;
#endif

#ifndef Rte_TypeDef_LBS_LCAConfig_t
#define Rte_TypeDef_LBS_LCAConfig_t
typedef struct {
    float32 fTTCThreshVrelLow_s;
    float32 fTTCThreshVrelMid_s;
    float32 fTTCThreshVrelHigh_s;
    float32 fTTCHysteresis_s;
    float32 fLCARangeMax_met;     // Maximum LCA range parameter
    float32 fLCACurveRadMax_met;  // Maximum LCA curve radius parameter
    uint8 uLCAWarningDurationCfg;
} LBS_LCAConfig_t;
#endif

#ifndef Rte_TypeDef_LBS_LCAFrontMirror_t
#define Rte_TypeDef_LBS_LCAFrontMirror_t
typedef struct {
    float32 fFMObjRate;
    float32 LCA_Vf_VxThreshAdd_mps;
    float32 LCA_Vf_VxThreshOwnLaneMin_mps;
    float32 LCA_Vf_VxThreshOwnLaneMax_mps;
    float32 LCA_Vf_VxThreshAdjLaneMin_mps;
    float32 LCA_Vf_VxThreshAdjLaneMax_mps;
    float32 fRCSStableObjOwnLane;  // RCS of the stable object in own lane
    float32 fRCSStableObjAdjLane;  // RCS of the stable object in adjacent lane
    uint8 uClosetStableObjIDOwnLane;  // the closet stable object ID in own lane
    uint8 uClosetStableObjIDAdjLane;  // the closet stable object ID in adjacent
                                      // lane
    uint8 uNofFMObjects;              // the number of front mirror objects
} LBS_LCAFrontMirror_t;
#endif

#ifndef Rte_TypeDef_LBSLCAObjInfoDebug_t
#define Rte_TypeDef_LBSLCAObjInfoDebug_t
typedef struct {
    float32 fTTCThreshold;       // TTC threshold
    float32 fBehindGrdProb_per;  // the object behind the guardrail
                                 // probability,range:0.0 - 1.0,unit:percent
    uint8 uFrontMirrorCnt;
    boolean bUpdateRecently;  // Flag whether the current object is sufficiently
                              // updated(measured) in the last two cycle
    boolean bInLCARange;      //
    boolean bLCAMirrorObject;
    boolean bLCAMirrorFrontObject;
    boolean bLCAObjPathInvalid;  // Flag whether path is accessible
    boolean bLCAQuality;         // Flag whether the object quality is enough
    boolean bLCALaneConditions;
    boolean bLCARelevant;
    boolean bLCAWarningConditions;  // Flag of meeting all LCA Warning
                                    // conditions
    boolean bLCAWarning;            // Flag whether current object is warning
} LBSLCAObjInfoDebug_t;
#endif

typedef LBSLCAObjInfoDebug_t LBSLCAObjInfoArrayDebug[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBS_LCA_Warn_Decide_Debug_t
#define Rte_TypeDef_LBS_LCA_Warn_Decide_Debug_t
typedef struct {
    boolean LCA_WarnDecide_bObjStable;
    boolean LCA_WarnDecide_bLCAQuality;
    boolean LCA_WarnDecide_bLCARelevant;
    boolean LCA_WarnDecide_bLCAMirrorObject;
    boolean LCA_WarnDecide_bLCAMirrorFrontObject;
    boolean LCA_WarnDecide_bLCALaneConditions;
    boolean LCA_WarnDecide_bLCAObjPathInvalid;
    float32 LCA_WarnDecide_fBehindGrdProb_per;
    boolean LCA_WarnDecide_bLCAWarnActive;
    boolean LCA_WarnDecide_bLCAWarningConditions;
    float32 LCA_WarnDecide_fTTC_s;
    float32 LCA_WarnDecide_fTTCThreshold;
    float32 LCA_WarnDecide_fDistX_met;
    float32 LCA_WarnDecide_fTTCAccel_mps2;
    float32 LCA_WarnDecide_fVrelX_mps;
    boolean LCA_WarnDecide_bInLCARange;
    boolean LCA_WarnDecide_bUpdateRecently;
    uint8 LCA_WarnDecide_uiHighestAssocProb_per;
    boolean LCA_WarnDecide_bLCAWarnActiveRight;
    boolean LCA_WarnDecide_bLCAWarnActiveLeft;
} LBS_LCA_Warn_Decide_Debug_t;
#endif

typedef LBS_LCA_Warn_Decide_Debug_t
    LBS_LCA_Warn_Decide_Debug_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBSLCADebug_t
#define Rte_TypeDef_LBSLCADebug_t
typedef struct {
    uint32 uiVersionNumber;
    uint32 uLCAWarningID_nu;
    // LCAGlobals
    // LBS_LCAWarnInfo_t LCAWarnInfo;
    // LBS_LCAConfig_t LCAConfig;
    // LBS_LCAFrontMirror_t LCAFrontMirror;
    float32 fLCARange;
    uint8 uCntLCAPathBlockedLeft;   // Counter of left LCA path blocked
    uint8 uCntLCAPathBlockedRight;  // Counter of right LCA path blocked
    boolean bLCAPathBlockedLeft;   // Flag whether left adjacent lane is no more
    boolean bLCAPathBlockedRight;  // Flag whether Right adjacent lane is no
                                   // more
    // LBSLCAObjInfoArrayDebug LCAObjOutputList;
    // LBS_LCA_Warn_Decide_Debug_Array LBS_LCAWarnDecideList_Debug;
} LBSLCADebug_t;
#endif

#ifndef Rte_TypeDef_LBSOSEObjInfoLevelDebug_t
#define Rte_TypeDef_LBSOSEObjInfoLevelDebug_t
typedef struct {
    float32 fBTHitHystTimer;
    uint8
        uBreakthroughHitConfi[LBS_OSE_LBS_NUM_OF_BREAK_LINES];  // Breakthrough
                                                                // hit
                                                                // confidence
    boolean bBreakthroughHit[LBS_OSE_LBS_NUM_OF_BREAK_LINES];   // Flag whether
    // the object hits
    // breakthrough
    boolean bWarning;           // Flag whether the OSE warning is actived
    boolean bWarningLastCycle;  // Flag whether the OSE warning was actived in
                                // the last cycle
    boolean bObjectInRange;  // Flag whether the object is in the range of three
                             // different level
    boolean
        bTTCBelowThresh[LBS_OSE_LBS_NUM_OF_BREAK_LINES];  // Flag whether the
                                                          // object's TTC is
                                                          // below TTC Threshold
    boolean bBTHitHystActive;  // Flag whether the warning hysteresis
} LBSOSEObjInfoLevelDebug_t;
#endif

#ifndef Rte_TypeDef_LBSOSEWidthEstimDebug_t
#define Rte_TypeDef_LBSOSEWidthEstimDebug_t
typedef struct {
    uint16 uCounters[3];
    float32 fValue_met;  // The object width in the algorithm
} LBSOSEWidthEstimDebug_t;
#endif

#ifndef Rte_TypeDef_LBSOSEObjInfoArrayDebug_t
#define Rte_TypeDef_LBSOSEObjInfoArrayDebug_t
typedef struct {
    LBSOSEObjInfoLevelDebug_t
        InfoLevel[LBS_OSE_LBS_NUM_OF_WARN_LEVELS];  // DOW warning information
                                                    // structure
    float32 fYBreakthrough[LBS_OSE_LBS_NUM_OF_BREAK_LINES];  // The y-axis
                                                             // breakthrough of
                                                             // the object
    float32 fYBreakthroughStd[LBS_OSE_LBS_NUM_OF_BREAK_LINES];  // The y-axis
                                                                // breakthrough
                                                                // standard
    // deviation of the object
    float32 fTTC_s[LBS_OSE_LBS_NUM_OF_BREAK_LINES];  // TTC of the object
    float32 fTTCFiltered_s[LBS_OSE_LBS_NUM_OF_BREAK_LINES];  // Filtered TTC of
                                                             // the object
    float32 fDistToCrossingLine_met
        [LBS_OSE_LBS_NUM_OF_BREAK_LINES];  // Distance between object front edge
                                           // and ego vehicle rear edge
    float32 fSideTrackProb;                // The probability of the side object
    // float32 fObjBreakthroughMargin[OSE_NUM_OF_BREAK_LINES];
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
    LBSOSEWidthEstimDebug_t fEstWidth;
} LBSOSEObjInfoArrayDebug_t;
#endif

#ifndef Rte_TypeDef_LBS_OSE_WarnDecide_InfoLevel_t
#define Rte_TypeDef_LBS_OSE_WarnDecide_InfoLevel_t
typedef struct {
    boolean LCA_WarnDecide_bWarning;
    boolean LCA_WarnDecide_bObjectInRange;
    uint8 LCA_WarnDecide_uBreakthroughHitConfi[2];
    boolean LCA_WarnDecide_bTTCBelowThresh[2];
    boolean LCA_WarnDecide_bBTHitHystActive;
    float32 LCA_WarnDecide_fTTCThreshold_s;
} LBS_OSE_WarnDecide_InfoLevel_t;
#endif

typedef LBS_OSE_WarnDecide_InfoLevel_t
    LBS_OSE_WarnDecide_InfoLevelArray_t[LBS_OSE_LBS_NUM_OF_WARN_LEVELS];
#ifndef Rte_TypeDef_LBS_OSE_Warn_Decide_Debug_t
#define Rte_TypeDef_LBS_OSE_Warn_Decide_Debug_t
typedef struct {
    uint8 OBJID;
    LBS_OSE_WarnDecide_InfoLevelArray_t OSE_WarnDecide_InfoLevel;
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
} LBS_OSE_Warn_Decide_Debug_t;
#endif

typedef LBS_OSE_Warn_Decide_Debug_t
    LBS_OSE_Warn_Decide_Debug_Array[LBS_INPUT_OBJECT_NUMBER];

typedef LBSOSEObjInfoArrayDebug_t
    LBSOSEObjInfoArrayDebug_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBSOSEDebug_t
#define Rte_TypeDef_LBSOSEDebug_t
typedef struct {
    uint32 uiVersionNum_nu;  // uint32 value example
    // LBSOSEObjInfoArrayDebug_Array OSEObjInfoArray;
    // LBS_OSE_Warn_Decide_Debug_Array OSEWarnDecideDebug;
} LBSOSEDebug_t;
#endif

typedef enum {
    LBSRCWState_Init = 0u,
    LBSRCWState_passive,
    LBSRCWState_StandBy,
    LBSRCWState_Active,
    LBSRCWState_Failure,
    LBSRCWState_Off
} LBSRCWStateMachine_t;

#ifndef Rte_TypeDef_LBSRCWStatusSubcondition
#define Rte_TypeDef_LBSRCWStatusSubcondition
typedef struct {
    boolean LBSdebugActive_bRCWWarningActive;
    boolean LBSdebugActive_bCorridorBlocked;
    boolean LBSdebugPassive_VelocityOutOfRange;
    boolean LBSdebugPassive_LatAcceloutofRange;
    boolean LBSdebugPassive_LeftTurnLight;
    boolean LBSdebugPassive_RightTurnLight;
    boolean LBSdebugPassive_GearPosition;
    boolean LBSdebugPassive_BlockingtimeActive;
    boolean LBSdebugHmiOff_bRCWHmiOpen;
    boolean LBSdebugFailure_bRCWFailureCondition;
} LBSRCWStatusSubcondition;
#endif

#ifndef Rte_TypeDef_LBSRCWWarningInfo_t
#define Rte_TypeDef_LBSRCWWarningInfo_t
typedef struct {
    float32 LBSfTTC;
    float32 LBSfXObjectWarning;
    uint8 LBSuRCWWarningID;
    boolean LBSbRCWWarningActive;
    boolean LBSbRCWWarningActiveLastCycle;
} LBSRCWWarningInfo_t;
#endif

#ifndef Rte_TypeDef_LBSRCW_Info_t
#define Rte_TypeDef_LBSRCW_Info_t
typedef struct {
    uint8 uObjID;
    float32 LBSfTTCThreshold;
    float32 LBSfCorridorOverlap;
    float32 LBSfCorridorOccupancy;
    float32 LBSfObjectOccupancy;
    float32 LBSfCorridorOccThreshold;
    float32 LBSfInCOrridorTime;
    float32 LBSfYBreakThrough;
    float32 LBSfHeadingFiltered;
    uint8 LBSuCorridorHitCnt;
    uint8 LBSuMultiPathCnt;
    boolean LBSbRCWQuality;
    boolean LBSbUpdateRecently;
    boolean LBSbRCWRelevant;
    boolean LBSbInRCWCorridor;
    boolean LBSbHeadingAngleInRange;
    boolean LBSbObjCorridorBlocked;
    boolean LBSbMultiPathObj;
    boolean LBSbRCWWarningConditions;
    boolean LBSbRCWWarning;
    boolean LBSbOppositeOverlap;
} LBSRCW_Info_t;
#endif
typedef LBSRCW_Info_t LBSRCW_Info_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBSRCWCorridorObserver_t
#define Rte_TypeDef_LBSRCWCorridorObserver_t
typedef struct {
    float32 LBSfXDist;
    float32 LBSfCorridorOccupancy;
    float32 LBSfXMin;
    float32 LBSfXMax;
    float32 LBSfInCorridorTime;
    uint8 LBSuObjID;
} LBSRCWCorridorObserver_t;
#endif
typedef LBSRCWCorridorObserver_t
    LBSRCWCorridorObserver_Arry[LBS_RCW_MAX_NOF_CORR_OBJS];

#ifndef Rte_TypeDef_LBS_RCW_Warn_Decide_Debug_t
#define Rte_TypeDef_LBS_RCW_Warn_Decide_Debug_t
typedef struct {
    uint8 OBJID;
    boolean RCW_WarnDecide_bEnableRCWWarning;
    float32 RCW_WarnDecide_fTTCRelevant;
    float32 RCW_WarnDecide_fTTCThreshold;
    float32 RCW_WarnDecide_fAxObj;
    boolean RCW_WarnDecide_bUpdateRecently;
    boolean RCW_WarnDecide_bObjCorridorBlocked;
    float32 RCW_WarnDecide_fCorridorOccupancy;
    float32 RCW_WarnDecide_fCorridorOccThreshold;
    float32 RCW_WarnDecide_fVrelToTraj_mps;
    boolean RCW_WarnDecide_fWarnDecisionVrel2TrajMaxThs;
    float32 RCW_WarnDecide_fAssocProbFiltered;
    float32 RCW_WarnDecide_fWarnDecisionAssocProbMin;
    boolean RCW_WarnDecide_bTTCthresholdjudgecondition;
    boolean RCW_WarnDecide_bfAxObjBelowTTCAdjust;
    float32 RCW_WarnDecide_fTTCThresholdAx;
} LBS_RCW_Warn_Decide_Debug_t;
#endif

typedef LBS_RCW_Warn_Decide_Debug_t
    LBS_RCW_Warn_Decide_Debug_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBSRCWDebug_t
#define Rte_TypeDef_LBSRCWDebug_t
typedef struct {
    LBSRCWStateMachine_t LBSDebug_RCWstatemachine;  // state machine
    // LBSRCWStatusSubcondition LBSDebug_RCWDebugSubConditions;
    // LBSRCWWarningInfo_t LBSDebug_RCWWarningInfo;
    // LBSRCW_Info_Array LBSDebug_RCWObjInfo;
    // LBSRCW_Info_Array LBSDebug_RCWObjInfo_Lastcycle;
    // LBSRCWCorridorObserver_Arry LBSDebug_RCWCorridorObjs;
    // LBS_RCW_Warn_Decide_Debug_Array LBSRCWWarnDecideDebug;
} LBSRCWDebug_t;
#endif

typedef LBS_SRRObjRoadRelation_t
    LBS_SRRObjRoadRelation_Array[LBS_INPUT_OBJECT_NUMBER];

typedef uint16 LBS_LastCycleObjID_Array[LBS_INPUT_OBJECT_NUMBER];

typedef LBS_SRRObjHistory_t LBS_ObjHistory_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBS_RunState_t
#define Rte_TypeDef_LBS_RunState_t
typedef struct {
    boolean bLBSFunctionActionLastCycle;
    // LBSState_t eLBState;
} LBS_RunState_t;
#endif

#ifndef Rte_TypeDef_LBS_ObjBorders_t
#define Rte_TypeDef_LBS_ObjBorders_t
typedef struct {
    float32 fXmin_met; /*The object x min position,unit:m*/
    float32 fXmax_met; /*The object x max position,unit:m*/
    float32 fYmin_met; /*The object y min position,unit:m*/
    float32 fYmax_met; /*The object y max position,unit:m*/
} LBS_ObjBorders_t;
#endif

#ifndef Rte_TypeDef_LBS_ObjectSelect_t
#define Rte_TypeDef_LBS_ObjectSelect_t
typedef struct {
    boolean bObjectFastEnough;
    boolean bBreakthroughHit;
    boolean bObjInRange;
} LBS_ObjectSelect_t;
#endif

#ifndef Rte_TypeDef_LBS_ObjInfo_st
#define Rte_TypeDef_LBS_ObjInfo_st
typedef struct {
    LBS_ObjBorders_t ObjBorders;         /*The object border information*/
    LBS_ObjBorders_t ObjMovementBorders; /*TODO*/
    LBS_ObjectSelect_t ObjSel;
    float32 fTTC_s;         /*The object TTC time to ego vehicle,unit:s*/
    float32 fTTCAccel_mps2; /*TODO*/
    float32 fTTCFiltered_s; /*The object filter TTC time to ego vehicle,unit:s*/
    float32 fTTCRadial_s;
    float32 fVabs_mpss;
    float32 fRangeRadial;
    float32 fXLastCycle_met;
    float32 fYLastCycle_met;
    float32 fVxPosBased;
    float32 fVyPosBased;
    float32 fSpeedFiltered_mpss;
    float32 fCycletimeSum_s; /*The object exist life time,unit:s*/
    float32 fUpdateRate_nu;  /*The object measurement update rate,unit:NULL*/
    float32 fXMovement_met;  /*The object total moving distance in the x
                                direction,unit:m*/
    float32 fYMovement_met;  /*The object total moving distance in the y
                                direction,unit:m*/
    float32 fAngle_deg; /*The object current position angle(0-360),unit:degree*/
    float32 fAssocProbFiltered; /*Highest cluster association probability of the
                                   object filter result*/
    float32 fObjWidthMax;
    float32 fObjLengthMax;
    uint16 uUniqueID;
    uint8 uLastMergedObjID;
    boolean bLowTTCAtStart;
    boolean bCreateAdjStableObj;
    uint8 firstStableDynProp;
} LBS_ObjInfo_st;
#endif
typedef LBS_ObjInfo_st LBS_ObjInfo_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBS_SICalculate_t
#define Rte_TypeDef_LBS_SICalculate_t
typedef struct {
    uint32 uiVersionNum_nu;  // uint32 value example
    //...
} LBS_SICalculate_t;
#endif

#ifndef Rte_TypeDef_LBS_SIObjInfo_t
#define Rte_TypeDef_LBS_SIObjInfo_t
typedef struct {
    uint8 eAssociatedLane;           // Lane Association
    float32 fVrelToTraj_mps;         // Object's relative velocity to trajectory
    float32 fDistToTraj_met;         // Distance to trajectory
    float32 fTraceBracketLeft_met;   // The trace bracket left side coordinate
    float32 fTraceBracketRight_met;  // The trace bracket right side coordinate
    float32 fObjBracketOverlap_met;  // Overlap between object and trace bracket

} LBS_SIObjInfo_t;
#endif
typedef LBS_SIObjInfo_t LBS_SIObjInfo_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBS_BSDCalculate_t
#define Rte_TypeDef_LBS_BSDCalculate_t
typedef struct {
    uint32 uiVersionNum_nu;     // uint32 value example
    boolean bBSDWarnActiveLeft; /* boolean value to distinguish whether BSD left
                                   side warning is active*/
    boolean bBSDWarnActiveRight; /* boolean value to distinguish whether BSD
                                    right side warning is active*/
                                 //...
    uint8 uBSDWarnActiveLeftID;
    uint8 uBSDWarnActiveRightID;
} LBS_BSDCalculate_t;
#endif

#ifndef Rte_TypeDef_LBS_BSDObjInfo_t
#define Rte_TypeDef_LBS_BSDObjInfo_t
typedef struct {
    float32 fBSDZoneXMin_met;  // BSD zone xmin from BSD global
} LBS_BSDObjInfo_t;
#endif

typedef LBS_BSDObjInfo_t LBS_BSDObjInfo_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBS_LCACalculate_t
#define Rte_TypeDef_LBS_LCACalculate_t
typedef struct {
    uint32 uiVersionNum_nu;  // uint32 value example
    boolean bLCAWarnActive;
    uint8 uLCAWarningID_nu;
    float32 fXObjectWarning_met;
    float32 fCriticalTTC_s;
    float32 fFMObjRate;

} LBS_LCACalculate_t;
#endif

#ifndef Rte_TypeDef_LBS_LCAObjInfo_t
#define Rte_TypeDef_LBS_LCAObjInfo_t
typedef struct {
    float32 fTTCThreshold;       // TTC threshold
    float32 fBehindGrdProb_per;  // the object behind the guardrail
                                 // probability,range:0.0 - 1.0,unit:percent
    uint8 uFrontMirrorCnt;
    boolean bUpdateRecently;  // Flag whether the current object is sufficiently
                              // updated(measured) in the last two cycle
    boolean bInLCARange;      //
    boolean bLCAMirrorObject;
    boolean bLCAMirrorFrontObject;
    boolean bLCAObjPathInvalid;  // Flag whether path is accessible
    boolean bLCAQuality;         // Flag whether the object quality is enough
    boolean bLCALaneConditions;
    boolean bLCARelevant;
    boolean bLCAWarningConditions;  // Flag of meeting all LCA Warning
                                    // conditions
    boolean bLCAWarning;            // Flag whether current object is warning
} LBS_LCAObjInfo_t;
#endif

typedef LBS_LCAObjInfo_t LBS_LCAObjInfo_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBS_OSEObjInfoLevelOutPro_t
#define Rte_TypeDef_LBS_OSEObjInfoLevelOutPro_t
typedef struct {
    float32 fBTHitHystTimer;
    uint8 uBreakthroughHitConfi[LBS_OSE_NUM_OF_BREAK_LINES];  // Breakthrough
                                                              // hit confidence
    boolean bBreakthroughHit[LBS_OSE_NUM_OF_BREAK_LINES];  // Flag whether the
                                                           // object hits
                                                           // breakthrough
    boolean bWarning;           // Flag whether the OSE warning is actived
    boolean bWarningLastCycle;  // Flag whether the OSE warning was actived in
                                // the last cycle
    boolean bObjectInRange;  // Flag whether the object is in the range of three
                             // different level
    boolean bTTCBelowThresh[LBS_OSE_NUM_OF_BREAK_LINES];  // Flag whether the
                                                          // object's TTC is
                                                          // below TTC Threshold
    boolean bBTHitHystActive;  // Flag whether the warning hysteresis
} LBS_OSEObjInfoLevelOutPro_t;
#endif

#ifndef Rte_TypeDef_LBS_OSEWidthEstimOutPro_t
#define Rte_TypeDef_LBS_OSEWidthEstimOutPro_t
typedef struct {
    uint16 uCounters[3];
    float32 fValue_met;  // The object width in the algorithm
} LBS_OSEWidthEstimOutPro_t;
#endif

#ifndef Rte_TypeDef_LBS_OSEObjInfo_t
#define Rte_TypeDef_LBS_OSEObjInfo_t
typedef struct {
    LBS_OSEObjInfoLevelOutPro_t
        InfoLevel[LBS_OSE_NUM_OF_WARN_LEVELS];  // DOW warning information
                                                // structure
    float32
        fYBreakthrough[LBS_OSE_NUM_OF_BREAK_LINES];  // The y-axis breakthrough
                                                     // of the object
    float32 fYBreakthroughStd
        [LBS_OSE_NUM_OF_BREAK_LINES];  // The y-axis breakthrough standard
                                       // deviation of the object
    float32 fTTC_s[LBS_OSE_NUM_OF_BREAK_LINES];          // TTC of the object
    float32 fTTCFiltered_s[LBS_OSE_NUM_OF_BREAK_LINES];  // Filtered TTC of the
                                                         // object
    float32 fDistToCrossingLine_met
        [LBS_OSE_NUM_OF_BREAK_LINES];  // Distance between object front edge and
                                       // ego vehicle rear edge
    float32 fSideTrackProb;            // The probability of the side object
    // float32 fObjBreakthroughMargin[OSE_NUM_OF_BREAK_LINES];
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
    LBS_OSEWidthEstimOutPro_t fEstWidth;
} LBS_OSEObjInfo_t;
#endif
typedef LBS_OSEObjInfo_t LBS_OSEObjInfo_Array[LBS_INPUT_OBJECT_NUMBER];

#ifndef Rte_TypeDef_LBS_WarningLastCycle_t
#define Rte_TypeDef_LBS_WarningLastCycle_t
typedef struct {
    uint32 uiVersionNum_nu;        // uint32 value example
    boolean bBSDWarningLastCycle;  // Flag whether BSD is warning in last cycle
    boolean
        bBSDWarningLeftLastCycle;  // Flag whether BSD is warning in last cycle
    boolean
        bBSDWarningRightLastCycle;  // Flag whether BSD is warning in last cycle
    boolean bLCAWarningLastCycle;   // Flag whether LCA is warning in last cycle
    boolean bOSEWarningLastCycle[LBS_OSE_NUM_OF_WARN_LEVELS];  // Flag whether
                                                               // OSE is warning
                                                               // in last cycle
    //...
} LBS_WarningLastCycle_t;
#endif

// #ifndef Rte_TypeDef_LBSCalculate_t
// #define Rte_TypeDef_LBSCalculate_t
// typedef struct {
//     // LBS_SRRObjRoadRelation_Array RoadRelation;
//     // LBS_LastCycleObjID_Array LastObjIDList;
//     // LBS_ObjHistory_Array LBSObjHistoryList;
//     // LBS_RunState_t LBSRunState;

//     // LBS_Globals_t LBS_Globals;
//     // LBS_ObjInfo_Array LBSObjInfoList;
//     // LBS_SICalculate_t LBSSICalc;
//     // LBS_SIObjInfo_Array SIObjInfoList;
//     // LBS_BSDCalculate_t LBSBSDCalc;
//     // LBS_BSDObjInfo_Array BSDObjInfoList;
//     // LBS_LCACalculate_t LBSLCACalc;
//     // LBS_LCAObjInfo_Array LCAObjInfoList;
//     // LBS_OSECalculate_t LBSOSECalc;
//     // LBS_OSEObjInfo_Array OSEObjInfoList;
//     // LBS_WarningLastCycle_t LBSWarnLastCycle;
// } LBSCalculate_t;
// #endif

#ifndef Rte_TypeDef_LBSDebug_t
#define Rte_TypeDef_LBSDebug_t
typedef struct {
    uint32 uiVersionNumber;
    LBSBSDDebug_t BSDDebug;
    LBSLCADebug_t LCADebug;
    LBSOSEDebug_t OSEDebug;
    LBSRCWDebug_t RCWDebug;
    // LBSCalculate_t LBSCalcuDebug;
} LBSDebug_t;
#endif

void LBS_Exec(const LBSInReq_st* reqPorts,
              const LBSParam_st* params,
              LBSOutPro_t* proPorts,
              LBSDebug_t* debugInfo);
void LBS_Reset();

#ifdef __cplusplus
}
#endif

#endif