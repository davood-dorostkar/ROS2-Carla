#ifndef LCCRA_EXT_H
#define LCCRA_EXT_H

#ifdef __cplusplus
extern "C" {
#endif
#include "LCCRA.h"
typedef struct {
    real32_T LBP_LeLnClthPosY0_met;  // the position Y0 value of left lane from
                                     // LBP module
    real32_T LBP_LeLnClthHeading_rad;  // the heading value of left lane from
                                       // LBP module
    real32_T LBP_LeLnClthCrv_1pm;  // the curvature value of left lane from LBP
                                   // module
    real32_T LBP_LeLnClthCrvChng_1pm2;  // the curvature change value of left
                                        // lane from LBP module
    boolean_T LBP_LeLnValid_bool;    // the lane validation flag value of left
                                     // lane from LBP module
    real32_T LBP_RiLnClthPosY0_met;  // the position Y0 value of right lane from
                                     // LBP module
    real32_T LBP_RiLnClthHeading_rad;  // the heading value of right lane from
                                       // LBP module
    real32_T LBP_RiLnClthCrv_1pm;  // the curvature value of right lane from LBP
                                   // module
    real32_T LBP_RiLnClthCrvChng_1pm2;  // the curvature change value of right
                                        // lane from LBP module
    boolean_T LBP_RiLnValid_bool;    // the lane validation flag value of right
                                     // lane from LBP module
    real32_T ALP_LeLnClthPosY0_met;  // the position Y0 value of adjacent left
                                     // lane from LBP module
    real32_T ALP_LeLnClthHeading_rad;  // the heading value of adjacent left
                                       // lane from LBP module
    real32_T ALP_LeLnClthCrv_1pm;  // the curvature value of adjacent left lane
                                   // from LBP module
    real32_T ALP_LeLnClthCrvChng_1pm2;  // the curvature change value of
                                        // adjacent left lane from LBP module
    boolean_T ALP_LeLnValid_bool;  // the lane validation flag value of adjacent
                                   // left lane from LBP module
    real32_T ALP_RiLnClthPosY0_met;  // the position Y0 value of adjacent right
                                     // lane from LBP module
    real32_T ALP_RiLnClthHeading_rad;  // the heading value of adjacent right
                                       // lane from LBP module
    real32_T ALP_RiLnClthCrv_1pm;  // the curvature value of adjacent right lane
                                   // from LBP module
    real32_T ALP_RiLnClthCrvChng_1pm2;  // the curvature change value of
                                        // adjacent right lane from LBP module
    boolean_T ALP_RiLnValid_bool;  // the lane validation flag value of adjacent
                                   // right lane from LBP module
    BusObject* Fusion_TargetObject_str;  // the objects from fusion module
    real32_T VED_EgoVelocity_mps;        // ego velocity
    real32_T VED_EgoYawRate_rps;         // ego yawrate
    real32_T VED_EgoClthCrv_1pm;         // ego curvature
    real32_T VED_EgoClthCrvChng_1pm2;    // ego curvature change
    real32_T ABPR_LnWidth_met;  // the ego lane width value from LBP module
    boolean_T ABPR_LnWidthValid_bool;  // the ego lane width validation flag
                                       // value from LBP module
} sLCCRAInReq_t;

typedef struct {
    real32_T
        LCFRCV_SysCycleTimeSen_sec;  // the cycle time of this module be invoked
} sLCCRAParam_t;

typedef struct {
    boolean_T LCCRA_LeftSafeFlag_bool;   // left lane change safety flag. 1:
                                         // safe, 0: not safe;
    boolean_T LCCRA_RightSafeFlag_bool;  // right lane change safety flag. 1:
                                         // safe, 0: not safe;
    boolean_T LCCRA_LeftFrontSafeFlag_bool;  // left front safety flag. 1: safe,
                                             // 0: not safe;
    boolean_T LCCRA_LeftRearSafeFlag_bool;   // left rear change safety flag. 1:
                                             // safe, 0: not safe;
    boolean_T LCCRA_RightFrontSafeFlag_bool;  // right front change safety flag.
                                              // 1: safe, 0: not safe;
    boolean_T LCCRA_RightRearSafeFlag_bool;   // rgiht rear change safety flag.
                                              // 1: safe, 0: not safe;
    int32_T LCCRA_LeftHighLightID_nu;
    int32_T LCCRA_RightHighLightID_nu;
    boolean_T LCCRA_FrontSafeFlag_bool;
    int32_T LCCRA_FrontDangerObjID_nu;
} sLCCRAOutPro_t;

typedef struct {
    BusDelayTime LCCRA_DebugDelaytime_str; /* '<S1>/LCCRA_FindMIOs' */
    BusDebugMIOs LCCRA_DebugMIOs_str;      /* '<S1>/LCCRA_FindMIOs' */
    BusWeight LCCRA_DebugWeight_str;       /* '<S1>/LCCRA_FindMIOs' */
    // BusVehicle LCCRA_TargetVehicle_str;    /* '<S1>/LCCRA_ObjectPreProcess'
    // */
    boolean_T LCCRA_bLeftFrontTTCSafe;   /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bLeftFrontTGSafe;    /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bLeftFrontRDSafe;    /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bNextLeftFrontSafe;  /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bLeftRearTTCSafe;    /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bLeftRearTGSafe;     /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bLeftRearRDSafe;     /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bNextLeftRearSafe;   /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bRightFrontTTCSafe;  /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bRightFrontTGSafe;   /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bRightFrontRDSafe;   /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bNextRightFrontSafe; /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bRightRearTTCSafe;   /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bRightRearTGSafe;    /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bRightRearRDSafe;    /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bNextRightRearSafe;  /* '<S1>/LCCRA_SafeFlag' */
    boolean_T LCCRA_bFrontTTCSafe;       /* '<S1>/LCCRA_SafeFlag' */
} sLCCRADebug_t;

void LCCRA_Reset(void);
void LCCRA_Exec(const sLCCRAInReq_t* reqPorts,
                const sLCCRAParam_t* param,
                sLCCRAOutPro_t* proPorts,
                sLCCRADebug_t* debug);
#ifdef __cplusplus
}
#endif
#endif