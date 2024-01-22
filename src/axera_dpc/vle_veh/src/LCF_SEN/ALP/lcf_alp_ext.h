/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ALP_LCF_ALP_EXT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ALP_LCF_ALP_EXT_H_
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "TM_Global_Types.h"
#include "lcf_alp_par.h"
/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  INPUT
*****************************************************************************/
typedef enum {
    eLeftLaneIndex,
    eRightLaneIndex,
    eLaneNumber,
} eALPLaneIndex;

typedef enum {
    ALP_INIT,
    ALP_OK,
} ALPState_t;

typedef struct {
    uint16 uEgoQuality_btf;  // In15,
    // S_ABPLBP_LeLnInvalidQu_btf,S_ABPLBP_RiLnInvalidQu_btf，Quality
    // bitfield of a lane boundary track, (0, 0~100, -)
    float32 fPosY0_met;  // In17, S_ABPLBP_AnyBndDistance_met,Lateral distance
                         // at X = 0.0 m, (0, -15~15, m)
    float32 fHeadingAngle_rad;  // In18,
    // S_ABPLBP_LeLnClthHeading_rad,S_ABPLBP_RiLnClthHeading_rad
    // Heading angle (Yaw angle) of a lane boundary
    // track, (0, -0.7854~0.7854, rad)
    float32 fCurvature_1pm;  // In19,
    // S_ABPLBP_LeLnClthCrv_1pm,S_ABPLBP_RiLnClthCrv_1pmCurvature
    // of a lane boundary track, (0, -0.1~0.1, 1/m)
    float32 fCurvatureRate_1pm2;  // In20, S_ABPLBP_LeLnClthCrvChng_1pm2
                                  // ,S_ABPLBP_RiLnClthCrvChng_1pm2 Curvature
                                  // rate of a lane boundary track, (0,
                                  // -0.001~0.001, 1/m^2)
    float32 fValidLength_met;     // In21,
    // S_ABPLBP_LeLnClthLength_met,S_ABPLBP_RiLnClthLength_met
    // Visibility range of a lane boundary track, (0,
    // 0~300, m)
} sALPEgoLaneInfo_st;

typedef struct {
    float32 fPosY0_met;  // In4, S_LCFRCV_AnyBndDistance_met,Lateral distance at
                         // X = 0.0 m, (0, -15~15, m)
    float32 fHeadingAngle_rad;  // In5, S_LCFRCV_AnyBndYawAngle_rad, Heading
                                // angle (Yaw angle) of a lane boundary track,
                                // (0, -0.7854~0.7854, rad)
    float32 fCurvature_1pm;  // In6, S_LCFRCV_AnyBndCurvature_1pm,Curvature of a
                             // lane boundary track, (0, -0.1~0.1, 1/m) */
    float32 fCurvatureRate_1pm2;  // In7, S_LCFRCV_AnyBndCrvRate_1pm2,Curvature
                                  // rate of a lane boundary track, (0,
                                  // -0.001~0.001, 1/m^2) */
    float32 fValidLength_met;  // In8, S_LCFRCV_AnyBndValidLength_met,Visibility
                               // range of a lane boundary track, (0, 0~300, m)
    uint8 uQuality_nu;     // In9, S_LCFRCV_AnyBndQuality_nu,Quality of a lane
                           // boundary track, (0, 0~100, -)
    uint8 uMarkerType_nu;  // In10, S_LCFRCV_AnyBndMarkerType_nu,Describes the
                           // type of a lane marker (e.g.road edge,…) ,(0,
                           // 0~100, -)
    boolean bAvailable_bool;  // In14, S_LCFRCV_AnyBndAvailable_bool,Defines
                              // whether a lane boundary track is available or
                              // not, (0, 0~1, -) */
} sALPAdjLaneInfo_st;

typedef sALPAdjLaneInfo_st sALPAdjLaneInfo_st_Array[eLaneNumber];
typedef sALPEgoLaneInfo_st sALPEgoLaneInfo_st_Array[eLaneNumber];

typedef struct {
    float32 fEgoVelX_mps;  // S_LCFRCV_VehVelX_mps, float32, ego vehicle VelX,
                           // [-20, 150]
    boolean bLaneChangeDetected_nu;  // S_ABPLBP_LaneChangeDetected,The Flag of
                                     // lane changed detected active
    float32 fLaneWidth_met;          // S_ABPLBP_LaneWidth_met
    float32 fCycleTime_sec;  // S_LCFRCV_SysCycleTimeSen_sec,Cycle time for
                             // LCF_SEN components ,(unit, s)

    sALPEgoLaneInfo_st_Array aALPEgoLanes;  // Ego lane data input
    sALPAdjLaneInfo_st_Array aALPAdjLanes;  // Adjacent lane data input
} sALPInReq_st;

typedef struct {
    float32 fEgoVehLength_met;  // P_VEH_Length_met

    float32 fLatPosStepDtcLimit_met;   // P_ABPALP_LatPosStepDtctLimit_met
    float32 fDebounceTLatPosStep_sec;  // P_ABPALP_DebounceTLatPosStep_sec
    float32 fAliveCounterTime_sec;     // P_ABPALP_AliveCounterTime_sec
    uint8 uiMinAliveCounter_perc;      // P_ABPALP_MinAliveCount_perc
    boolean bAllowBridging_bool;       // P_ABPALP_AllowBridging_bool
    float32 LatPosPT1TConst_sec;       // P_ABPALP_LatPosPT1TConst_sec
} sALPParam_st;

typedef struct {
    boolean bLeftAdjLnStepDeounced_bool;
    boolean bRightAdjLnStepDeounced_bool;
    boolean bLeftAdjLnBridging_bool;
    boolean bRightAdjLnBridging_bool;
    boolean bLeftAdjLnStepOnBridging_bool;
    boolean bRightAdjLnStepOnBridging_bool;

    // 0x01:AnyBndAailable, 0:valid,                                1:invalid;
    // 0x02:StepDeboundced, 0:not in deboundced state,              1:debounced
    // step;
    // 0x04:Bridging,       0:not in bridging state,                1:bridging
    // state;
    // 0x08:StepOnBridging, 0:step check isn't pass after bridging, 1:step check
    // pass after bridging
    uint16 uiLeftAdjLnInvalidQu_btf;     // S_ABPALP_LeAdjLnInvalidQu_btf
    uint16 uiRightAdjLnInvalidQu_btf;    // S_ABPALP_RiAdjLnInvalidQu_btf
    float32 uiLeftAdjLnAliveCount_per;   // S_ABPALP_LeAdjAliveCount_perc
    float32 uiRightAdjLnAliveCount_per;  // S_ABPALP_RiAdjAliveCount_perc

    float32 fLeftAdjLnDistance_met;     // S_ABPALP_LeAdjLnClthPosY0_met
    float32 fLeftAdjLnYawAngle_rad;     // S_ABPALP_LeAdjLnClthHeading_rad
    float32 fLeftAdjLnCurvature_1pm;    // S_ABPALP_LeAdjLnClthCrv_1pm
    float32 fLeftAdjLnCrvRate_1pm2;     // S_ABPALP_LeAdjLnClthCrvChng_1pm2
    float32 fLeftAdjLnValidLength_met;  // S_ABPALP_LeAdjLnClthLength_met

    float32 fRightAdjLnDistance_met;     // S_ABPALP_RiAdjLnClthPosY0_met
    float32 fRightAdjLnYawAngle_rad;     // S_ABPALP_RiAdjLnClthHeading_rad
    float32 fRightAdjLnCurvature_1pm;    // S_ABPALP_RiAdjLnClthCrv_1pm
    float32 fRightAdjLnCrvRate_1pm2;     // S_ABPALP_RiAdjLnClthCrvChng_1pm2
    float32 fRightAdjLnValidLength_met;  // S_ABPALP_RiAdjLnClthLength_met
} sALPOutput_st;

#ifndef Rte_TypeDef_sALPDebug_st
#define Rte_TypeDef_sALPDebug_st
    typedef struct
    {
        uint32 uiVersionNum_nu; // uint32 value example
    } sALPDebug_st;
#endif

extern void LCF_ALP_Init_Reset(void);
extern void LCF_ALP_Exec(const sALPInReq_st* reqPorts,
                         const sALPParam_st* params,
                         sALPOutput_st* proPorts,
                         sALPDebug_st* debugInfo);

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ALP_LCF_ALP_EXT_H_
