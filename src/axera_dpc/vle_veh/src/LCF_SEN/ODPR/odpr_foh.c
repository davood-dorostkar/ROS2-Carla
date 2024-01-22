/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
#include "odpr_foh_ext.h"
#include "odpr_fop.h"
#include "tue_common_libs.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h"

// Calibration variable
const volatile float32 ODPR_Kf_FOHLowSpdMaxVel_kph =
    13.f;  // Below this maximum Ego velocity the Low Speed Mode is valid
           // (incl.Hysteresis)
const volatile float32 ODPR_Kf_FOHLowSpdHystVel_kph =
    5.f;  // Hysteresis for maximum Ego velocity for Low Speed Mode
const volatile float32 ODPR_Kf_FOHObjHistTime_sec =
    1.f;  // Maximum time for new object history sample
const volatile float32 ODPR_Kf_FOHObjHistDistMax_met =
    10.f;  // Maximum distance between new and old object history sample point
const volatile float32 ODPR_Kf_FOHObjHistDistMin_met =
    0.8f;  // Maximum distance between new and old object history sample point
const volatile float32 ODPR_Kf_FOHFeatMinPosXTime_sec =
    0.75f;  // Minimum time to longitudinal position x (behind front axle) for
            // history point to be valid
const volatile float32 ODPR_Kf_FOHFeatMinPosX_met =
    -5.5f;  // Min longitudinal position x (with respect to front axle) for
            // history point to be valid
const volatile uint8 ODPR_Ku_FOHMinValidEntries_nu =
    0x06;  // Minimum required valid history points
const volatile float32 ODPR_Kf_FOHMinHistLengthTime_sec =
    1.f;  // Minimum history length in seconds to be treated as valid
const volatile float32 ODPR_Kf_FOHMinHistLength_met =
    6.f;  // Minimum required history length for validity
const volatile uint8 ODPR_Ku_FOHYawRateDelayCycles_nu =
    0x01;  // Specify yaw rate delay cycles
const volatile uint16 ODPR_Ku_FOHACCObjKfUpdate_btm =
    0x03FB;  // Bitmask for check of Acc object validity for KF measurement
             // update
const volatile float32 ODPR_Kf_FOHMinObjValidTime_sec =
    1.f;  // Minimum object validity time for starting measurement update
const volatile uint16 ODPR_Ku_FOHACCObjFreeze_btm =
    0x1000;  // Bitmask for S_ODPFOP_AccObjInvBitfield_btf to check if ACC obj
             // position is frozen
const volatile uint16 ODPR_Ku_FOHACCObjChange_btm =
    0x0800;  // Bitmask for S_ODPFOP_AccObjInvBitfield_btf to observe ACC object
             // changes
const volatile boolean ODPR_Kb_FOHPT1PosY1Enable_bool =
    TRUE;  // TRUE means low pass filter for lateral position is enabled
const volatile boolean ODPR_Kb_FOHPT1YawRateEnable_bool =
    TRUE;  // TRUE means low pass filter for input yaw rate is enabled
const volatile float32 ODPR_Kf_FOHMaxHistStartX0_met =
    25.f;  // Specify max history start x0
const volatile float32 ODPR_Kf_FOHPredHistMaxPosX0_sec =
    3.f;  // Maximum prediction (extrapolation) time from history X0 to ego
          // vehicle front axle
const volatile float32 ODPR_Kf_FOHWeightLastPolyfit_nu =
    0.5f;  // Specify weight last polyfit
const volatile boolean ODPR_Kb_FOHUseStraightEstim_bool =
    TRUE;  // TRUE:own straight estimation calculation will be used for
           // weighting between 1st and 3rd

const volatile float32 ODPR_PosY0StdDev_met = 0.6f;
const volatile float32 ODPR_HeadingStdDev_met = 0.02f;
const volatile float32 ODPR_CurvatureStdDev_met = 0.000001f;
const volatile float32 ODPR_CurvatureRateStdDev_met = 0.000001f;

const volatile boolean ODPR_ENABLE_KALMANFILTER = TRUE;
const volatile float32 ODPR_MaxDeltaSampleDist_met = 1.5f;
const volatile float32 ODPR_NumSamplesAccObjX = 6.0f;

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
STATIc uint16 FOH_uiPreCycAccObjBitwise_btm;
STATIc float32 FOH_fPreCycAccObjPosX_met;
STATIc boolean FOH_bPreCycAccObjValid_bool;
STATIc boolean FOH_bPreCycCutInTranOng_bool;
STATIc boolean FOH_bPreCycCutOutTranOng_bool;
STATIc boolean FOH_bPreCycFreezeStopTranOng_bool;
STATIc boolean FOH_bPreCycObjValidTranOng_bool;
STATIc boolean FOH_bPreCycObjTransCtrlEn_bool;  // Used in
// TransitionFilterPosY_PF/TransitionFilterPosY/TransitionFilterYaw
// module
STATIc float32 FOH_fPreCycPosYPF_met;  // Used in TransitionFilterPosY_PF module
STATIc float32
    FOH_fPreCycPosYPFFreeze_met;  // Used in TransitionFilterPosY_PF module
STATIc float32
    FOH_fPreCycPosYPFCorr_met;  // Used in TransitionFilterPosY_PF module
STATIc float32 FOH_fPreCycPosYLSM_met;  // Used in TransitionFilterPosY module
STATIc float32
    FOH_fPreCycPosYLSMFreeze_met;  // Used in TransitionFilterPosY module
STATIc float32
    FOH_fPreCycPosYLSMCorr_met;        // Used in TransitionFilterPosY module
STATIc float32 FOH_fPreCycYawAng_rad;  // Used in TransitionFilterYaw module
STATIc float32
    FOH_fPreCycYawAngFreeze_rad;           // Used in TransitionFilterYaw module
STATIc float32 FOH_fPreCycYawAngCorr_rad;  // Used in TransitionFilterYaw module
STATIc float32 FOH_fPreCycEgoVelX_mps;     // Used in meanVelX module
STATIc boolean
    FOH_bPreCycModeTransCtrlEn_bool;  // Used in
                                      // CosineTransitionPosY0/CosineTransitionHead/CosineTransitionLength/CosineTransitionCrv
                                      // module
STATIc float32 FOH_fPreCycPosY0_met;  // Used in CosineTransitionPosY0 module
STATIc float32
    FOH_fPreCycPosY0Freeze_met;  // Used in CosineTransitionPosY0 module
STATIc float32 FOH_fPreCycPosY0Corr_met;  // Used in CosineTransitionPosY0
                                          // module
STATIc float32 FOH_fPreCycHead_rad;       // Used in CosineTransitionHead module
STATIc float32 FOH_fPreCycHeadFreeze_rad;  // Used in CosineTransitionHead
                                           // module
STATIc float32 FOH_fPreCycHeadCorr_rad;  // Used in CosineTransitionHead module
STATIc float32
    FOH_fPreCycTrajLength_met;  // Used in CosineTransitionLength module
STATIc float32
    FOH_fPreCycTrajLengthFreeze_met;  // Used in CosineTransitionLength module
STATIc float32
    FOH_fPreCycTrajLengthCorr_met;  // Used in CosineTransitionLength module
STATIc float32 FOH_fPreCycCrv_1pm;  // Used in CosineTransitionCrv module
STATIc float32 FOH_fPreCycCrvFreeze_1pm;  // Used in CosineTransitionCrv module
STATIc float32 FOH_fPreCycCrvCorr_1pm;    // Used in CosineTransitionCrv module
STATIc boolean
    FOH_bPreCycLSMTransOngoing_bool;  // Used in DefineModeTransitionQualifier
                                      // module

STATIc float32
    FOH_fObjValidTurnOnDelay_sec;  // Global time for object valid turn on delay
STATIc boolean
    FOH_bPreCycFreezeState_bool;  // Object freeze state in previous cycle
STATIc float32 FOH_fTrajValidTurnOnDelay_sec;  // Global time for trajectory
                                               // valid turn on delay
STATIc boolean FOH_bPreCycMaxObjDistLSM_bool;  // ObjDistLSM hysteresis output
                                               // in previous cycle
STATIc boolean FOH_bPreCycObjValid_bool;       // Object valid state in previous
                                               // cycle
STATIc boolean
    FOH_bPreCycTrajValid_bool;  // Trajectory valid state in previous cycle
STATIc boolean
    FOH_bPreCycObjValid2_bool;  // Object valid state in previous cycle
STATIc float32 FOH_fTransEnableTurnOffDelay_sec;  // Global time for transition
                                                  // enable turn off delay
STATIc float32 FOH_fFactorATimer_sec;     // Time output of factor A timer
STATIc boolean FOH_bPreCycPosYPFRS_bool;  // PosY_PF RS flip-flop
STATIc boolean FOH_bPreCycPosYRS_bool;    // PosY RS flip-flop
STATIc boolean FOH_bPreCycYawRS_bool;     // Yaw RS flip-flop
STATIc float32 FOH_fPreCycYawRate_rps;    // Yaw rate in previous cycle
STATIc float32
    FOH_fPreCycYawRateLowPass_rps;  // Yaw rate lowpass value in previous cycle
STATIc boolean
    FOH_bPreCycTrajInvalid_bool;  // Traj invalid state in previous cycle
STATIc boolean FOH_bPreCycLSMState_bool;  // LSM invalid state in previous cycle
STATIc float32
    FOH_fLSMChngTurnOffDelay_sec;  // Global time for LSM change turn off delay
STATIc float32 FOH_fLSMFactorATimer_sec;     // Time output of factor A timer
STATIc float32 FOP_fPreCycPosY0LowPass_met;  // PosY0 lowpass filter output in
                                             // previous cycle
STATIc float32 FOP_fPreCycHeadLowPass_met;   // Heading lowpass filter output in
                                             // previous cycle
STATIc float32 FOP_fPreCycCurveLowPass_met;  // Curve lowpass filter output in
                                             // previous cycle
STATIc float32 FOP_fPreCycCrvChngLowPass_met;  // Curve change lowpass filter
                                               // output in previous cycle
STATIc float32 FOH_fPreCycPosYPT1LowPass_met;  // PosY PT1 PT lowpass filter
                                               // output in previous cycle
STATIc float32 FOH_fPreCycPosYLSMLowPass_met;  // PosY PT1 LSM lowpass filter
                                               // output in previous cycle
STATIc boolean FOH_bPreCycCosPosY0_bool;       // Cosine transition PosY0 RS
                                               // flip-flop
STATIc boolean FOH_bPreCycCosHead_bool;  // Cosine transition Head RS flip-flop
STATIc boolean
    FOH_bPreCycCosLength_bool;          // Cosine transition Length RS flip-flop
STATIc boolean FOH_bPreCycCosCrv_bool;  // Cosine transition Crv RS flip-flop
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*state of the KF*/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static UINT8_T valid_laneKF = 0u;
/*status_laneKF of the KF: 0: invalid, 1: valid, full update, 2: valid,
 * degraded update, 3: valid, prediction, 4: valid, init, 5: invalid, reset*/
static UINT8_T status_laneKF = 0u;
/*measurement weight*/
static REAL32_T measWeight_laneKF = 0.0f;
/*internal quality*/
static REAL32_T internalQuality_laneKF = 0.0f;
/*variable for the yaw rate standard deviation - TODO: really needed?*/
static REAL32_T vehYawRateStdDev_laneKF = 0.0f;
/*variable for the geometric model error*/
static REAL32_T kappa2diff_sigma_laneKF = 0.0f;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

#define STATE_LENGTH_LANEKF (4u)

/*****************************************************************************
  VARIABLES
*****************************************************************************/
/* Array */
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
STATIc float32 FOH_fYawRateDelay[6];  // Ego vehicle yaw rate delay

/* Structure */
STATIc FOHPreCycHistValid_t
    FOH_sPreCycHistValid;  // Feedback of trajectory polyfit info
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Lookup table */
// {14.f, 0.f, 2.8f, 5.6f, 8.3f, 11.1f, 13.9f, 16.7f, 19.4f, 22.2f, 25.f, 27.8f,
// 30.6f, 33.3f, 36.1f}      //X_ODPFOH_VehVelX_mps
// {20.f, 25.f, 28.f, 33.f, 35.f, 37.f, 32.f, 50.f, 50.f, 50.f, 50.f, 50.f, 50.f,
// 50.f}                    //Y_ODPFOH_MaxObjDistLSM_met
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
STATIc TUE_CML_Vector2D_t FOH_sMaxObjDistLSM[14] =  // Lookup table to define
                                                    // speed dependent maximum
                                                    // object distance for Low
                                                    // Speed Mode
    {{0.f, 20.f},   {2.8f, 25.f},  {5.6f, 28.f},  {8.3f, 33.f},  {11.1f, 35.f},
     {13.9f, 37.f}, {16.7f, 32.f}, {19.4f, 50.f}, {22.2f, 50.f}, {25.f, 50.f},
     {27.8f, 50.f}, {30.6f, 50.f}, {33.3f, 50.f}, {36.1f, 50.f}};

// {14.f, 0.f, 10.f, 20.f, 30.f, 40.f, 50.f, 60.f, 70.f, 80.f, 90.f, 100.f,
// 110.f, 120.f, 130.f}           //X_ODPFOH_VehVelXPT1_kph
// {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.3f, 0.3f, 0.3f, 0.3f,
// 0.3f, 0.3f}                    //Y_ODPFOH_PT1TimeConstPosY_sec
STATIc TUE_CML_Vector2D_t FOH_sPT1TimeConstPosY[14] =  // Lookup table of speed
                                                       // dependent PT1 time
                                                       // constant values for
                                                       // lateral position
    {{0.f, 0.06f},
     {10.f, 0.06f},
     {20.f, 0.06f},
     {30.f, 0.06f},
     {40.f, 0.06f},
     {50.f, 0.06f},
     {60.f, 0.06f},
     {70.f, 0.1f},
     {80.f, 0.3f},
     {90.f, 0.3f},
     {
         100.f,
         0.3f,
     },
     {110.f, 0.3f},
     {120.f, 0.3f},
     {130.f, 0.3f}};

// {14.f, 0.f, 10.f, 20.f, 30.f, 40.f, 50.f, 60.f, 70.f, 80.f, 90.f, 100.f,
// 110.f, 120.f, 130.f}           //X_ODPFOH_VehVelXPT1_kph
// {1.2f, 1.2f, 1.2f, 1.2f, 1.2f, 1.2f, 1.2f, 1.2f, 1.2f, 1.2f, 1.2f, 1.2f, 1.2f,
// 1.2f}                    //Y_ODPFOH_LSMPT1TimePosY_sec
STATIc TUE_CML_Vector2D_t FOH_sLSMPT1TimePosY[14] =  // Lookup table of speed
                                                     // dependent PT1 time
                                                     // constant values for
                                                     // lateral position in Low
                                                     // Speed Mode
    {{0.f, 1.2f},   {10.f, 1.2f},  {20.f, 1.2f},  {30.f, 1.2f}, {40.f, 1.2f},
     {50.f, 1.2f},  {60.f, 1.2f},  {70.f, 1.2f},  {80.f, 1.2f}, {90.f, 1.2f},
     {100.f, 1.2f}, {110.f, 1.2f}, {120.f, 1.2f}, {130.f, 1.2f}};

// {14.f, 0.f, 2.8f, 5.6f, 8.3f, 11.1f, 13.9f, 16.7f, 19.4f, 22.2f, 25.f, 27.8f,
// 30.6f, 33.3f, 36.1f}      //X_ODPFOH_VehVelX_mps
// {0.5f, 0.5f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.5f, 0.5f, 0.5f, 0.5f,
// 0.5f, 0.5f}                    //Y_ODPFOH_PT1TimeConstYawRate_sec
STATIc TUE_CML_Vector2D_t FOH_sPT1TimeConstYawRate[14] =  // Lookup table of
                                                          // speed dependent PT1
                                                          // time constant
                                                          // values for input
                                                          // yaw rate
    {{0.f, 0.5f},   {2.8f, 0.5f},  {5.6f, 0.3f},  {8.3f, 0.3f},  {11.1f, 0.3f},
     {13.9f, 0.3f}, {16.7f, 0.3f}, {19.4f, 0.3f}, {22.2f, 0.5f}, {25.f, 0.5f},
     {27.8f, 0.5f}, {30.6f, 0.5f}, {33.3f, 0.5f}, {36.1f, 0.5f}};

// {14.f, 0.f, 2.8f, 5.6f, 8.3f, 11.1f, 13.9f, 16.7f, 19.4f, 22.2f, 25.f, 27.8f,
// 30.6f, 33.3f, 36.1f}      //X_ODPFOH_VehVelX_mps
// {30.f, 10.f, 8.f, 6.f, 4.f, 4.f, 4.f, 4.f, 4.f, 4.f, 3.f, 3.f, 3.f, 2.f}
// Y_ODPFOH_MaxSampleAge_sec
STATIc TUE_CML_Vector2D_t FOH_sMaxSampleAge[14] =  // Maximum age for a saved
                                                   // object data sample point
                                                   // to be treated as valid
    {{0.f, 35.f},  {2.8f, 25.f}, {5.6f, 20.f}, {8.3f, 15.f}, {11.1f, 10.f},
     {13.9f, 9.f}, {16.7f, 8.f}, {19.4f, 7.f}, {22.2f, 6.f}, {25.f, 4.f},
     {27.8f, 3.f}, {30.6f, 3.f}, {33.3f, 3.f}, {36.1f, 2.f}};

// {14.f, 0.f, 2.8f, 5.6f, 8.3f, 11.1f, 13.9f, 16.7f, 19.4f, 22.2f, 25.f, 27.8f,
// 30.6f, 33.3f, 36.1f}      //X_ODPFOH_VehVelX_mps
// {0.f, 0.f, 0.f, 0.1f, 0.25f, 0.4f, 0.55f, 0.7f, 0.85f, 1.f, 1.f, 1.f, 1.f,
// 1.f}                         //Y_ODPFOH_WeightCrvPolyfit_nu;
STATIc TUE_CML_Vector2D_t FOH_sWeightCrvPolyfit[14] =  // Lookup table of speed
                                                       // dependent weighting
                                                       // factor for polyfit
                                                       // coefficients
    {{0.f, 0.f},   {2.8f, 0.2f}, {5.6f, 0.4f}, {8.3f, 0.6f}, {11.1f, 0.8f},
     {13.9f, 1.f}, {16.7f, 1.f}, {19.4f, 1.f}, {22.2f, 1.f}, {25.f, 1.f},
     {27.8f, 1.f}, {30.6f, 1.f}, {33.3f, 1.f}, {36.1f, 1.f}};

// {14.f, 0.f, 10.f, 20.f, 30.f, 40.f, 50.f, 60.f, 70.f, 80.f, 90.f, 100.f,
// 110.f, 120.f, 130.f}           //X_ODPFOH_VehVelXPT1_kph
// {0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f,
// 0.06f, 0.06f, 0.06f}      //Y_ODPFOH_PT1TimeConstPosY0_sec
STATIc TUE_CML_Vector2D_t FOH_sPT1TimeConstPosY0[14] =  // Lookup table of speed
                                                        // dependent PT1 time
                                                        // constant values for
                                                        // lateral position
    {{0.f, 0.06f},   {10.f, 0.06f}, {20.f, 0.06f},  {30.f, 0.06f},
     {40.f, 0.06f},  {50.f, 0.06f}, {60.f, 0.06f},  {70.f, 0.06f},
     {80.f, 0.06f},  {90.f, 0.06f}, {100.f, 0.06f}, {110.f, 0.06f},
     {120.f, 0.06f}, {130.f, 0.06f}};

// {14.f, 0.f, 10.f, 20.f, 30.f, 40.f, 50.f, 60.f, 70.f, 80.f, 90.f, 100.f,
// 110.f, 120.f, 130.f}           //X_ODPFOH_VehVelXPT1_kph
// {0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f,
// 0.06f, 0.06f, 0.06f}      //Y_ODPFOH_PT1TimeConstHead_sec
STATIc TUE_CML_Vector2D_t FOH_sPT1TimeConstHead[14] =  // Lookup table of speed
                                                       // dependent PT1 time
                                                       // constant values for
                                                       // heading
    {{0.f, 0.06f},   {10.f, 0.06f}, {20.f, 0.06f},  {30.f, 0.06f},
     {40.f, 0.06f},  {50.f, 0.06f}, {60.f, 0.06f},  {70.f, 0.06f},
     {80.f, 0.06f},  {90.f, 0.06f}, {100.f, 0.06f}, {110.f, 0.06f},
     {120.f, 0.06f}, {130.f, 0.06f}};

// {14.f, 0.f, 10.f, 20.f, 30.f, 40.f, 50.f, 60.f, 70.f, 80.f, 90.f, 100.f,
// 110.f, 120.f, 130.f}           //X_ODPFOH_VehVelXPT1_kph
// {0.5f, 0.5f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.5f, 0.5f, 0.5f, 0.5f,
// 0.5f, 0.5f}                    //Y_ODPFOH_PT1TimeConstCrv_sec
STATIc TUE_CML_Vector2D_t FOH_sPT1TimeConstCrv[14] =  // Lookup table of speed
                                                      // dependent PT1 time
                                                      // constant values for
                                                      // curvature
    {{0.f, 0.5f},   {10.f, 0.5f},  {20.f, 0.3f},  {30.f, 0.3f}, {40.f, 0.3f},
     {50.f, 0.3f},  {60.f, 0.3f},  {70.f, 0.3f},  {80.f, 0.5f}, {90.f, 0.5f},
     {100.f, 0.5f}, {110.f, 0.5f}, {120.f, 0.5f}, {130.f, 0.5f}};

// {14.f, 0.f, 10.f, 20.f, 30.f, 40.f, 50.f, 60.f, 70.f, 80.f, 90.f, 100.f,
// 110.f, 120.f, 130.f}           //X_ODPFOH_VehVelXPT1_kph
// {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
// 0.1f, 0.1f}                    //Y_ODPFOH_PT1TimeConstCrvChng_sec
STATIc TUE_CML_Vector2D_t FOH_sPT1TimeConstCrvChng[14] = {
    {0.f, 0.1f},   {10.f, 0.1f},  {20.f, 0.1f},  {30.f, 0.1f}, {40.f, 0.1f},
    {50.f, 0.1f},  {60.f, 0.1f},  {70.f, 0.1f},  {80.f, 0.1f}, {90.f, 0.1f},
    {100.f, 0.1f}, {110.f, 0.1f}, {120.f, 0.1f}, {130.f, 0.1f}};

// {14.f, 0.f, 2.8f, 5.6f, 8.3f, 11.1f, 13.9f, 16.7f, 19.4f, 22.2f, 25.f, 27.8f,
// 30.6f, 33.3f, 36.1f}      //X_ODPFOH_VehVelX_mps
// {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
// //Y_ODPFOH_MinTrajectoryLength_met
STATIc TUE_CML_Vector2D_t FOH_sMinTrajectoryLength[14] =  // Lookup table to
                                                          // define speed
                                                          // dependent minimum
                                                          // trajectory length
                                                          // for Kalman Filter
                                                          // input
    {{0.f, 0.f},   {2.8f, 0.f},  {5.6f, 0.f},  {8.3f, 0.f},  {11.1f, 0.f},
     {13.9f, 0.f}, {16.7f, 0.f}, {19.4f, 0.f}, {22.2f, 0.f}, {25.f, 0.f},
     {27.8f, 0.f}, {30.6f, 0.f}, {33.3f, 0.f}, {36.1f, 0.f}};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

/*****************************************************************************
  Functionname:    FOHReset                                        */ /*!

                                @brief           Reset function of FOH

                                @description     All global variables related to
                              FOH are
                              reset in
                              this function
                                                 when FOH executes for the first
                              time, or
                              system
                              exception needs
                                                 to be reset

                                @param[in]       none

                                @return          none
                              *****************************************************************************/
void ODPR_FOH_Reset(void) {
    FOH_uiPreCycAccObjBitwise_btm = 0u;
    FOH_fPreCycAccObjPosX_met = 0.f;
    FOH_bPreCycAccObjValid_bool = FALSE;
    FOH_bPreCycCutInTranOng_bool = FALSE;
    FOH_bPreCycCutOutTranOng_bool = FALSE;
    FOH_bPreCycFreezeStopTranOng_bool = FALSE;
    FOH_bPreCycObjValidTranOng_bool = FALSE;
    FOH_bPreCycObjTransCtrlEn_bool = FALSE;
    FOH_fPreCycPosYPF_met = 0.f;
    FOH_fPreCycPosYPFFreeze_met = 0.f;
    FOH_fPreCycPosYPFCorr_met = 0.f;
    FOH_fPreCycPosYLSM_met = 0.f;
    FOH_fPreCycPosYLSMFreeze_met = 0.f;
    FOH_fPreCycPosYLSMCorr_met = 0.f;
    FOH_fPreCycYawAng_rad = 0.f;
    FOH_fPreCycYawAngFreeze_rad = 0.f;
    FOH_fPreCycYawAngCorr_rad = 0.f;
    FOH_fPreCycEgoVelX_mps = 0.f;
    FOH_bPreCycModeTransCtrlEn_bool = FALSE;
    FOH_fPreCycPosY0_met = 0.f;
    FOH_fPreCycPosY0Freeze_met = 0.f;
    FOH_fPreCycPosY0Corr_met = 0.f;
    FOH_fPreCycHead_rad = 0.f;
    FOH_fPreCycHeadFreeze_rad = 0.f;
    FOH_fPreCycHeadCorr_rad = 0.f;
    FOH_fPreCycTrajLength_met = 0.f;
    FOH_fPreCycTrajLengthFreeze_met = 0.f;
    FOH_fPreCycTrajLengthCorr_met = 0.f;
    FOH_fPreCycCrv_1pm = 0.f;
    FOH_fPreCycCrvFreeze_1pm = 0.f;
    FOH_fPreCycCrvCorr_1pm = 0.f;
    FOH_bPreCycLSMTransOngoing_bool = FALSE;

    FOH_fObjValidTurnOnDelay_sec = 0.f;
    FOH_bPreCycFreezeState_bool = FALSE;
    FOH_fTrajValidTurnOnDelay_sec = 0.f;
    FOH_bPreCycMaxObjDistLSM_bool = FALSE;
    FOH_bPreCycObjValid_bool = TRUE;
    FOH_bPreCycTrajValid_bool = TRUE;
    FOH_bPreCycObjValid2_bool = TRUE;
    FOH_fTransEnableTurnOffDelay_sec = 0.f;
    FOH_fFactorATimer_sec = 0.f;
    FOH_bPreCycPosYPFRS_bool = FALSE;
    FOH_bPreCycPosYRS_bool = FALSE;
    FOH_bPreCycYawRS_bool = FALSE;
    FOH_fPreCycYawRate_rps = 0.f;
    FOH_fPreCycYawRateLowPass_rps = 0.f;
    FOH_bPreCycTrajInvalid_bool = FALSE;
    FOH_bPreCycLSMState_bool = FALSE;
    FOH_fLSMChngTurnOffDelay_sec = 0.f;
    FOH_fLSMFactorATimer_sec = 0.f;
    FOP_fPreCycPosY0LowPass_met = 0.f;
    FOP_fPreCycHeadLowPass_met = 0.f;
    FOP_fPreCycCurveLowPass_met = 0.f;
    FOP_fPreCycCrvChngLowPass_met = 0.f;
    FOH_fPreCycPosYPT1LowPass_met = 0.f;
    FOH_fPreCycPosYLSMLowPass_met = 0.f;
    FOH_bPreCycCosPosY0_bool = FALSE;
    FOH_bPreCycCosHead_bool = FALSE;
    FOH_bPreCycCosLength_bool = FALSE;
    FOH_bPreCycCosCrv_bool = FALSE;
    /*state of the KF*/
    valid_laneKF = 0u;
    /*status_laneKF of the KF: 0: invalid, 1: valid, full update, 2: valid,
     * degraded update, 3: valid, prediction, 4: valid, init, 5: invalid,
     * reset*/
    status_laneKF = 0u;
    /*measurement weight*/
    measWeight_laneKF = 0.0f;
    /*internal quality*/
    internalQuality_laneKF = 0.0f;
    /*variable for the yaw rate standard deviation - TODO: really needed?*/
    vehYawRateStdDev_laneKF = 0.0f;
    /*variable for the geometric model error*/
    kappa2diff_sigma_laneKF = 0.0f;

    /* Reset array */
    memset(FOH_fYawRateDelay, 0, sizeof(FOH_fYawRateDelay));

    /* Reset struct */
    memset(&FOH_sPreCycHistValid, 0, sizeof(FOH_sPreCycHistValid));
}

/*****************************************************************************
  Functionname:    FOHEgoMotionCalc                                           */ /*!

@brief           Ego vehicle motion calculation

@description     Outputs ego vehicle motion information, yaw rate is delayed
                several cycles according to different config. This function
                is corresponding to YawRateObjSync module in MBD.

@param[in]       pEgoVEDData     Dynamic data of ego vehicle  from VED module
@param[in,out]   pFOHDebug       Debug output of FOH

@return          sEgoMotion
                     fYawRateObjSync_rps  //Yaw rate after syncronization
                     fYawRateRaw_rps      //Raw yaw rate
                     fEgoCrv_lpm          //Ego curve
                     fEgoVelX_mps         //Ego velocity X

*****************************************************************************/
STATIc FOHEgoMotion_t FOHEgoMotionCalc(const ODPRInVEDVehDyn_t* pEgoVEDData,
                                       ODPRFOHDebug_t* pFOHDebug) {
    FOHEgoMotion_t sEgoMotion = {0};  // result value
    uint8 uiYawRatDelayNum_nu =
        TUE_CML_MinMax(1u, 6u, ODPR_Ku_FOHYawRateDelayCycles_nu) - 1u;

    /* Yaw rate delay outputs */
    if (ODPR_Ku_FOHYawRateDelayCycles_nu) {
        sEgoMotion.fYawRateObjSync_rps = FOH_fYawRateDelay[uiYawRatDelayNum_nu];
    } else {
        sEgoMotion.fYawRateObjSync_rps = pEgoVEDData->fEgoYawRate_rps;
    }

    /* Other ego motion information outputs */
    sEgoMotion.fYawRateRaw_rps = pEgoVEDData->fEgoYawRate_rps;
    sEgoMotion.fEgoCrv_1pm = pEgoVEDData->fEgoCurve_1pm;
    sEgoMotion.fEgoVelX_mps = pEgoVEDData->fEgoVelX_mps;

    /* Updates yaw rate delay array */
    for (uint8 i = 5; i > 0; i--) {
        FOH_fYawRateDelay[i] = FOH_fYawRateDelay[i - 1];
    }
    FOH_fYawRateDelay[0] = pEgoVEDData->fEgoYawRate_rps;

    /* Debug output */
    pFOHDebug->fYawRateObjSync_rps = sEgoMotion.fYawRateObjSync_rps;

    return sEgoMotion;
}

/*****************************************************************************
  Functionname:    FOHAccObjStatusChk */ /*!

@brief           Acc object status check

@description     Acc object status check, including object validity, freeze
stop,
          cut-in/cut-out,low speed mode, and trajectory invalid,etc.
          This function is corresponding to AccObjStatusCheck module in
MBD.

@param[in]       pEgoVEDData     Dynamic data of ego vehicle  from VED module
@param[in]       pSystemPara     System parameter
@param[in]       pFOPOutput      Validity check results output by FOP
@param[in,out]   pFOHDebug       Debug output of FOH

@return          sAccObjChk
               bAccObjValid_bool       //Flag that acc object is valid
               bAccObjFreezeStop_bool  //Flag that acc object freeze stop
               bObjCutOut_bool         //Flag that acc object cut out
               bObjCutIn_bool          //Flag that acc object cut in
               bLSMInactive_bool       //Flag that low speed mode inactive
               bTrajInvalid_bool       //Flag that trajectory invalid
@uml
@startuml
start
partition AccObjValid {
if (All check items specified by ODPR_Ku_FOHACCObjKfUpdate_btm are met) then (yes)
:The result of ACC object validity check is TRUE;
else (no)
:The result of ACC object validity check is FALSE;
endif
}
partition AccObjFreezeStop {
if (FallingEdge of LateralMovingInvalid is detected) then (yes)
:The result of Freeze Stop Check is TRUE;
else (no)
:The result of Freeze Stop Check is FALSE;
endif
}
partition AccObjChng {
fork
if (object ID changed &\n PosX is closer than previous cycle) then (yes)
:Flag that object is\ncutting in is TRUE;
else (no)
:Flag that object is\ncutting in is FALSE;
endif
fork again
if (object ID changed &\n PosX is greater than previous cycle &\nObject is
detected) then (yes)
:Flag that object is\ncutting out is TRUE;
else (no)
:Flag that object is\ncutting out is FALSE;
endif
end fork
}
partition LSMInactive {
if (Ego velocity is less than LSM Max threshold) then (yes)
:Flag that LSMInactive is TRUE;
elseif (Ego velocity is greater than sum of \nLSM Max threshold plus hysteresis
velocity ) then (yes)
:Flag that LSMInactive is FALSE;
else (no)
if (Trajectory polyfit is valid and keeps for a while \nor DistX is greater
than threshold) then (yes)
   :Flag that LSMInactive is FALSE;
else (no)
   :Flag that LSMInactive is TRUE;
endif
endif
}
partition TrajInvalid {
if (1st or 3rd trajectory polyfit is invalid) then (yes)
:Flag that TrajInvalid is TRUE;
else (no)
:Flag that TrajInvalid is FALSE;
endif
}
stop
@enduml
*****************************************************************************/
STATIc FOHAccObjStatusChk_t
FOHAccObjStatusChk(const ODPRInVEDVehDyn_t* pEgoVEDData,
                   const ODPRInSystemPara_t* pSystemPara,
                   const ODPRFOPOut_t* pFOPOutput,
                   ODPRFOHDebug_t* pFOHDebug) {
    FOHAccObjStatusChk_t sAccObjChk = {0};  // result value
    boolean bTempBitwiseChk_bool;
    uint16 uiTempAccObjDtctBitwise_btm;
    boolean bTempAccObjDtctChk_bool;
    boolean bTempPreHistValidChk_bool;
    float32 fTempMaxObjDistLSM_met;
    boolean bTempPreHistValidTurnOnDelay_bool;
    boolean bTempAccObjPosXHyst_bool;

    /* Acc object validity for object update */
    bTempBitwiseChk_bool = ((pFOPOutput->uiAccObjInvalidCheck_btf &
                             ODPR_Ku_FOHACCObjKfUpdate_btm) == 0u);
    sAccObjChk.bAccObjValid_bool = BASICTurnOnDelay(
        bTempBitwiseChk_bool, ODPR_Kf_FOHMinObjValidTime_sec,
        pSystemPara->fSystemCylceTime_sec, &FOH_fObjValidTurnOnDelay_sec);

    /* Acc object freeze standstill */
    bTempBitwiseChk_bool = ((pFOPOutput->uiAccObjInvalidCheck_btf &
                             ODPR_Ku_FOHACCObjFreeze_btm) != 0u);
    sAccObjChk.bAccObjFreezeStop_bool = TUE_CML_FallingEdgeSwitch(
        bTempBitwiseChk_bool, &FOH_bPreCycFreezeState_bool);

    /* Acc object change check */
    uiTempAccObjDtctBitwise_btm =
        pFOPOutput->uiAccObjInvalidCheck_btf & FOH_ACC_OBJ_DTCT_BTM;
    bTempAccObjDtctChk_bool = !((uiTempAccObjDtctBitwise_btm == 0u) &&
                                (FOH_uiPreCycAccObjBitwise_btm != 0u));
    bTempBitwiseChk_bool = ((pFOPOutput->uiAccObjInvalidCheck_btf &
                             ODPR_Ku_FOHACCObjChange_btm) != 0u);

    sAccObjChk.bObjCutOut_bool =
        bTempAccObjDtctChk_bool && bTempBitwiseChk_bool &&
        (pFOPOutput->fAccObjPosX_met > FOH_fPreCycAccObjPosX_met);
    sAccObjChk.bObjCutIn_bool =
        bTempBitwiseChk_bool &&
        (pFOPOutput->fAccObjPosX_met < FOH_fPreCycAccObjPosX_met);

    /* Low speed mode invalid */
    bTempPreHistValidChk_bool =
        FOH_sPreCycHistValid.bPreCycTrajInvalid_1st_bool ||
        FOH_sPreCycHistValid.bPreCycTrajInvalid_3rd_bool;
    bTempPreHistValidTurnOnDelay_bool = BASICTurnOnDelay(
        !bTempPreHistValidChk_bool, FOH_MIN_TRAJ_VALID_TIME_SEC,
        pSystemPara->fSystemCylceTime_sec, &FOH_fTrajValidTurnOnDelay_sec);

    fTempMaxObjDistLSM_met = TUE_CML_CalculatePolygonValue2D(
        14, FOH_sMaxObjDistLSM, pEgoVEDData->fEgoVelX_mps);
    bTempAccObjPosXHyst_bool = TUE_CML_HysteresisFloat(
        pFOPOutput->fAccObjPosX_met, fTempMaxObjDistLSM_met,
        (float32)(fTempMaxObjDistLSM_met - FOH_MAX_OBJ_DIST_HYST_LSM_MET),
        &FOH_bPreCycMaxObjDistLSM_bool);

    if ((3.6f * pEgoVEDData->fEgoVelX_mps) < ODPR_Kf_FOHLowSpdMaxVel_kph) {
        sAccObjChk.bLSMInactive_bool = FALSE;
    } else if ((3.6f * pEgoVEDData->fEgoVelX_mps) >
               (ODPR_Kf_FOHLowSpdMaxVel_kph + ODPR_Kf_FOHLowSpdHystVel_kph)) {
        sAccObjChk.bLSMInactive_bool = TRUE;
    } else {
        sAccObjChk.bLSMInactive_bool =
            (bTempPreHistValidTurnOnDelay_bool || bTempAccObjPosXHyst_bool);
    }

    /* polyfit invalid */
    sAccObjChk.bTrajInvalid_bool = bTempPreHistValidChk_bool;

    /* Debug output */
    pFOHDebug->bAccObjFreezeStop_bool = sAccObjChk.bAccObjFreezeStop_bool;
    pFOHDebug->bAccObjValid_bool = sAccObjChk.bAccObjValid_bool;
    pFOHDebug->bLSMInactive_bool = sAccObjChk.bLSMInactive_bool;
    pFOHDebug->bObjCutIn_bool = sAccObjChk.bObjCutIn_bool;
    pFOHDebug->bObjCutOut_bool = sAccObjChk.bObjCutOut_bool;
    pFOHDebug->bTrajInvalid_bool = sAccObjChk.bTrajInvalid_bool;
    pFOHDebug->bTempAccObjDtctChk_bool = bTempAccObjDtctChk_bool;
    pFOHDebug->bTempPreHistValidChk_bool = bTempPreHistValidChk_bool;
    pFOHDebug->bTempPreHistValidTurnOnDelay_bool =
        bTempPreHistValidTurnOnDelay_bool;
    pFOHDebug->fTempMaxObjDistLSM_met = fTempMaxObjDistLSM_met;
    pFOHDebug->bTempAccObjPosXHyst_bool = bTempAccObjPosXHyst_bool;

    /* Reassign value, which is used in next cycle*/
    FOH_uiPreCycAccObjBitwise_btm = uiTempAccObjDtctBitwise_btm;
    FOH_fPreCycAccObjPosX_met = pFOPOutput->fAccObjPosX_met;

    return sAccObjChk;
}

/*****************************************************************************
  Functionname:    FOHHistoryControl */ /*!

@brief           History control

@description     HistoryControl module outputs flags that save new entry,
             reset history, and enable history. This function is
             corresponding to HistoryControl module in MBD.

@param[in]       pEgoVEDData     Dynamic data of ego vehicle  from VED module
@param[in]       pFOPOutput      Validity check results output by FOP
@param[in]       pAccObjChk      Flags that acc object status check
@param[in,out]   pFOHOutput      Acc object trajectory curve output by FOH
@param[in,out]   pFOHDebug       Debug output of FOH

@return          sHistoryControl
                  bSaveNewEntry_bool       //Flag that save new netry
                  bResetHistory_bool       //Flag that reset history
                  bEnableHistory_bool      //Flag that enable history
@uml
@startuml
start
:Enable history check;
note:Set to TRUE directly
partition ResetHistory {
if (Flag that TrajInvalid becomes to TRUE from FALSE \n or Flag that
TrajInvalid is TRUE & Object becomes to Valid from invalid) then (yes)
  :The flag of ResetHistory is TRUE;
else (no)
  :The flag of ResetHistory is FALSE;
endif
}
note:If ResetHistory is TRUE,\n the matrix uesed in polyfit and parameters\n
of trajectory need to be reset
partition SaveNewEntry {
if (Acc object is valid & \n ResetHistory is FALSE &\n PosX meets threshold)
then (yes)
  :The flag of SaveNewEntry is TRUE;
else (no)
  :The flag of SaveNewEntry is FALSE;
endif
}
note:If SaveNewEntry is TRUE, then the acc \n object can be used in trajectory
polyfit
stop
@enduml
*****************************************************************************/
STATIc FOHHistoryControl_t
FOHHistoryControl(const ODPRInVEDVehDyn_t* pEgoVEDData,
                  const ODPRFOPOut_t* pFOPOutput,
                  const FOHAccObjStatusChk_t* pAccObjChk,
                  ODPRFOHOut_t* pFOHOutput,
                  ODPRFOHDebug_t* pFOHDebug) {
    FOHHistoryControl_t sHistoryControl = {0};  // result value
    float32 fTempPosXDiff_met = 0.f;
    float32 fTempPosXFromPos_met = 0.f;
    float32 fTempPosXFromVel_met = 0.f;
    boolean bTempPosXFromPosChk_bool = FALSE;
    boolean bTempPosXFromVelChk_bool = FALSE;
    boolean bTempPosXChk_bool = FALSE;

    /* Output directly */
    sHistoryControl.bEnableHistory_bool = TRUE;

    /* Check history reset conditions */
    sHistoryControl.bResetHistory_bool =
        TUE_CML_RisingEdgeSwitch(pAccObjChk->bTrajInvalid_bool,
                                 &FOH_bPreCycTrajValid_bool) ||
        (pAccObjChk->bTrajInvalid_bool &&
         TUE_CML_RisingEdgeSwitch(pAccObjChk->bAccObjValid_bool,
                                  &FOH_bPreCycObjValid_bool));

    /* Check new entry conditions */
    if (FOH_sPreCycHistValid.uiPreCycNumOfValidSamples_nu > 0u) {
        fTempPosXDiff_met = pFOPOutput->fAccObjPosX_met -
                            FOH_sPreCycHistValid.fPreCycLastStoredPointX_met;
        fTempPosXFromVel_met =
            TUE_CML_Max(pEgoVEDData->fEgoVelX_mps * ODPR_Kf_FOHObjHistTime_sec,
                        ODPR_Kf_FOHObjHistDistMax_met);
        fTempPosXFromPos_met =
            TUE_CML_Min(TUE_CML_Max(BASICDivDflt(pFOPOutput->fAccObjPosX_met,
                                                 ODPR_NumSamplesAccObjX,
                                                 ODPR_MaxDeltaSampleDist_met),
                                    ODPR_Kf_FOHObjHistDistMin_met),
                        ODPR_MaxDeltaSampleDist_met);
        ;
        bTempPosXFromVelChk_bool = (fTempPosXDiff_met < fTempPosXFromVel_met);
        bTempPosXFromPosChk_bool = (fTempPosXDiff_met > fTempPosXFromPos_met);

        bTempPosXChk_bool =
            bTempPosXFromVelChk_bool && bTempPosXFromPosChk_bool;
    } else {
        bTempPosXChk_bool = TRUE;
    }
    sHistoryControl.bSaveNewEntry_bool = bTempPosXChk_bool &&
                                         pAccObjChk->bAccObjValid_bool &&
                                         (!sHistoryControl.bResetHistory_bool);

    pFOHOutput->bAddNewSample_bool = sHistoryControl.bSaveNewEntry_bool;

    /* Debug output */
    pFOHDebug->bResetHistory_bool = sHistoryControl.bResetHistory_bool;
    pFOHDebug->bSaveNewEntry_bool = sHistoryControl.bSaveNewEntry_bool;
    pFOHDebug->bEnableHistory_bool = sHistoryControl.bEnableHistory_bool;
    pFOHDebug->fTempPosXDiff_met = fTempPosXDiff_met;
    pFOHDebug->fTempPosXFromPos_met = fTempPosXFromPos_met;
    pFOHDebug->fTempPosXFromVel_met = fTempPosXFromVel_met;
    pFOHDebug->bTempPosXFromPosChk_bool = bTempPosXFromPosChk_bool;
    pFOHDebug->bTempPosXFromVelChk_bool = bTempPosXFromVelChk_bool;
    pFOHDebug->bTempPosXChk_bool = bTempPosXChk_bool;

    return sHistoryControl;
}

/*****************************************************************************
  Functionname:    FOHAccObjPreProcessing */ /*!

@brief           Acc object data preprocessing

@description     Acc object data preprocessing. Acc object information output
by FOP is not used to trojectoty polyfit directly, it needs
preprocessing, including TransitionControl, Position Y filter
and Transition switch. This function is corresponding
to AccObDatajPreProcessing module in MBD.

@param[in]       pEgoMotion           Ego vehicle motion calculation
@param[in]       pAccObjChk           Flags that acc object status check
@param[in]       pSystemPara          System parameter
@param[in]       pFOPOutput           Validity check results output by FOP
@param[in,out]   pAccObjPreProcess    Result outputs by acc object preprocessing
@param[in,out]   pFOHDebug            Debug output of FOH

@return          none

@uml
@startuml
start
partition TransitionControl {

partition TransitionReset {
if (Object CutIn or Object CutOut or\nFreeze Stop or\nLSM & object becomes valid
from invalid) then (yes)
:The flag of TransitionReset is TRUE;
else (no)
:The flag of TransitionReset is FALSE;
endif
}
:Transition Enable;
note:If TransitionReset is TRUE,then TransitionEnable is TRUE;\nIf
TransitionReset becomes to FALSE and keeps for a while,\nthen TransitionEnable
is FALSE;
:Factor A calculation;
note:Calc time T that TransitionEnable keeps in TRUE when
TransitionReset\nbecomes to FALSE; Factor A = 0.5*(cos(T/time threshold)+1) ;
partition CutInOnging {
if (TransitionEnable is FALSE) then (yes)
:CutInOnging flag = CutIn flag;
elseif (TransitionReset is TRUE) then (yes)
:CutInOnging flag = CutIn flag;
else (no)
:CutInOnging flag Keeps \nvalue of previous cycle;
endif

}
:CutOutOnging;
note:Similar to check process of CutInOnging flag;
:FreezeStopOnging;
note:Similar to check process of CutInOnging flag;
:ObjValidEnableOnging;
note:Similar to check process of CutInOnging flag;
}
partition PosYLosPassFilter {
if (Use EstimatedPosY as filter input) then (yes)
:Calc DeltaPosY;
note:DeltaPosY = PosX * YawRate * CycleTime
:Calc PosY(Lowpass filter input);
note:PosY = EstimatedPosY - DeltaPosY;
else (no)
:Calc DeltaPosY;
note:DeltaPosY = PosX * YawRateSync * CycleTime
:Calc PosY(Lowpass filter input);
note:PosY = RawPosY + RelVelY*CycleTime - DeltaPosY
endif
}
partition TransitionSwitch {
:PosY and YawAngle fading;
note:Determine if the PosY(LSM mode and not LSM mode) can be used to trajectory
polyfit directly or\n\
need to be processed again after PosYLosPassFilter based on result of
TransitionControl.
}
stop
@enduml
*****************************************************************************/
STATIc void FOHAccObjPreProcessing(const FOHEgoMotion_t* pEgoMotion,
                                   const FOHAccObjStatusChk_t* pAccObjChk,
                                   const ODPRInSystemPara_t* pSystemPara,
                                   const ODPRFOPOut_t* pFOPOutput,
                                   FOHAccObjPreProcessing_t* pAccObjPreProcess,
                                   ODPRFOHDebug_t* pFOHDebug) {
    boolean bCutInDetected_bool;
    boolean bCutOutDetected_bool;
    boolean bAccObjFreezeStop_bool;
    boolean bObjValidity_bool;
    boolean bAccObjValidRisingChk_bool;
    float32 fTempPosYCorr_met;
    float32 fTempPosYCorrBef_met;
    float32 fTempPosYCorrAft_met;
    float32 fTempPosYPT1PF_met;
    float32 fTempPosYPT1LSM_met;
    boolean bTempTransAll_bool;
    boolean bTempTransPF_bool;
    float32 fTempFactorA_fct;
    float32 fTempTransPosYPT1PF_met;
    float32 fTempTransPosYPT1LSM_met;
    float32 fTempTransYawAng_met;
    float32 fTempPosYPT1LowpassT_sec;
    float32 fTempPosYLSMLowpassT_sec;

    /* Transition control preprocessing */
    bCutInDetected_bool =
        FOH_bPreCycAccObjValid_bool && pAccObjChk->bObjCutIn_bool;
    bCutOutDetected_bool =
        FOH_bPreCycAccObjValid_bool && pAccObjChk->bObjCutOut_bool;
    bAccObjFreezeStop_bool =
        FOH_bPreCycAccObjValid_bool && pAccObjChk->bAccObjFreezeStop_bool;
    bAccObjValidRisingChk_bool = TUE_CML_RisingEdgeSwitch(
        pAccObjChk->bAccObjValid_bool, &FOH_bPreCycObjValid2_bool);

    bObjValidity_bool = bAccObjValidRisingChk_bool &&
                        (!pAccObjChk->bLSMInactive_bool) &&
                        FOH_ENABLE_TRANS_AT_START_BOOL;

    /* Set object transition time */
    pAccObjPreProcess->pObjTranControl->bTransitionReset_bool =
        bCutInDetected_bool || bCutOutDetected_bool || bAccObjFreezeStop_bool ||
        bObjValidity_bool;
    pAccObjPreProcess->pObjTranControl->bTransitionEnable_bool =
        BASICTurnOffDelay(
            pAccObjPreProcess->pObjTranControl->bTransitionReset_bool,
            FOH_ACC_OBJ_CHNG_DURATION_SEC, pSystemPara->fSystemCylceTime_sec,
            &FOH_fTransEnableTurnOffDelay_sec);

    /* Define object transition qualifier */
    if (pAccObjPreProcess->pObjTranControl->bTransitionEnable_bool &&
        (!pAccObjPreProcess->pObjTranControl->bTransitionReset_bool)) {
        pAccObjPreProcess->pObjTranControl->bCutInOngoing_bool =
            FOH_bPreCycCutInTranOng_bool;
        pAccObjPreProcess->pObjTranControl->bCutOutOngoing_bool =
            FOH_bPreCycCutOutTranOng_bool;
        pAccObjPreProcess->pObjTranControl->bFreezeStopOnging_bool =
            FOH_bPreCycFreezeStopTranOng_bool;
        pAccObjPreProcess->pObjTranControl->bObjValidOngoing_bool =
            FOH_bPreCycObjValidTranOng_bool;
    } else {
        pAccObjPreProcess->pObjTranControl->bCutInOngoing_bool =
            bCutInDetected_bool;
        pAccObjPreProcess->pObjTranControl->bCutOutOngoing_bool =
            bCutOutDetected_bool;
        pAccObjPreProcess->pObjTranControl->bFreezeStopOnging_bool =
            bAccObjFreezeStop_bool;
        pAccObjPreProcess->pObjTranControl->bObjValidOngoing_bool =
            bObjValidity_bool;
    }

    /* Calculate transition factor A */
    if (pAccObjPreProcess->pObjTranControl->bTransitionReset_bool) {
        FOH_fFactorATimer_sec = 0.f;
    } else {
        FOH_fFactorATimer_sec =
            pAccObjPreProcess->pObjTranControl->bTransitionEnable_bool
                ? (FOH_fFactorATimer_sec + pSystemPara->fSystemCylceTime_sec)
                : FOH_fFactorATimer_sec;
    }
    pAccObjPreProcess->pTgtObjData->fTransitionFactorA_fac =
        0.5f * (TUE_CML_GDBcos_52(FOH_fFactorATimer_sec /
                                  SafeDiv(FOH_ACC_OBJ_CHNG_DURATION_SEC) *
                                  TUE_CML_Pi) +
                1.f);
    fTempFactorA_fct = pAccObjPreProcess->pTgtObjData->fTransitionFactorA_fac;

    /* Position Y Preprocessing */
    if (FOH_USE_POS_Y_CORR_BOOL) {
        if (FOH_USE_ESTIM_POS_Y_BOOL) {
            fTempPosYCorr_met = pFOPOutput->fEstimateObjPosY_met -
                                pFOPOutput->fAccObjPosX_met *
                                    pEgoMotion->fYawRateRaw_rps *
                                    pSystemPara->fSystemCylceTime_sec;
        } else {
            fTempPosYCorrBef_met = pFOPOutput->fAccObjPosX_met *
                                   pEgoMotion->fYawRateObjSync_rps *
                                   pSystemPara->fSystemCylceTime_sec;
            fTempPosYCorr_met = pFOPOutput->fAccObjPosY_met +
                                pFOPOutput->fAccObjRelVelY_mps *
                                    pSystemPara->fSystemCylceTime_sec -
                                fTempPosYCorrBef_met;
        }
    } else {
        fTempPosYCorr_met = pFOPOutput->fAccObjPosY_met;
    }

    if (FOH_USE_POS_Y_REDUCTION_BOOL &&
        (TUE_CML_Abs(fTempPosYCorr_met) < 1.f)) {
        fTempPosYCorrAft_met =
            TUE_CML_Sqr(fTempPosYCorr_met) * TUE_CML_Sign(fTempPosYCorr_met);
    } else {
        fTempPosYCorrAft_met = fTempPosYCorr_met;
    }

    /* Position Y filter */
    if (ODPR_Kb_FOHPT1PosY1Enable_bool) {
        /* PosY PT1 PF Lowpass filter */
        if (pAccObjPreProcess->pObjTranControl->bTransitionReset_bool) {
            FOH_fPreCycPosYPT1LowPass_met = fTempPosYCorrAft_met;
        } else {
            fTempPosYPT1LowpassT_sec = TUE_CML_CalculatePolygonValue2D(
                14, FOH_sPT1TimeConstPosY, (3.6f * pEgoMotion->fEgoVelX_mps));
            TUE_CML_LowPassFilter(&FOH_fPreCycPosYPT1LowPass_met,
                                  fTempPosYCorrAft_met,
                                  (pSystemPara->fSystemCylceTime_sec /
                                   SafeDiv(fTempPosYPT1LowpassT_sec)));
        }
        fTempPosYPT1PF_met = FOH_fPreCycPosYPT1LowPass_met;

        /* PosY PT1 LSM Lowpass filter */
        if (pAccObjPreProcess->pObjTranControl->bTransitionReset_bool) {
            FOH_fPreCycPosYLSMLowPass_met = fTempPosYCorrAft_met;
        } else {
            fTempPosYLSMLowpassT_sec = TUE_CML_CalculatePolygonValue2D(
                14, FOH_sLSMPT1TimePosY, (3.6f * pEgoMotion->fEgoVelX_mps));
            TUE_CML_LowPassFilter(&FOH_fPreCycPosYLSMLowPass_met,
                                  fTempPosYCorrAft_met,
                                  (pSystemPara->fSystemCylceTime_sec /
                                   SafeDiv(fTempPosYLSMLowpassT_sec)));
        }
        fTempPosYPT1LSM_met = FOH_fPreCycPosYLSMLowPass_met;
    } else {
        fTempPosYPT1PF_met = fTempPosYCorrAft_met;
        fTempPosYPT1LSM_met = fTempPosYCorrAft_met;
    }

    /* Transition switch */
    pAccObjPreProcess->pTgtObjData->fAccObjPosX_met =
        pFOPOutput->fAccObjPosX_met;
    pAccObjPreProcess->pTgtObjData->fAccObjPosXStdDev_met =
        pFOPOutput->fAccObjPosXStdDev_met;
    pAccObjPreProcess->pTgtObjData->fAccObjPosYStdDev_met =
        pFOPOutput->fAccObjPosYStdDev_met;

    bTempTransPF_bool =
        pAccObjPreProcess->pObjTranControl->bCutInOngoing_bool ||
        pAccObjPreProcess->pObjTranControl->bCutOutOngoing_bool;
    bTempTransAll_bool =
        bTempTransPF_bool ||
        pAccObjPreProcess->pObjTranControl->bFreezeStopOnging_bool ||
        pAccObjPreProcess->pObjTranControl->bObjValidOngoing_bool;

    fTempTransPosYPT1PF_met =  // This fucntion should be outside the 'if'
                               // judgement, because the delay in which should
                               // be updated every cycle regardless of whether
                               // the judgment passed
        FOHTransFilter(fTempPosYPT1PF_met, pAccObjPreProcess->pObjTranControl,
                       fTempFactorA_fct, &FOH_fPreCycPosYPF_met,
                       &FOH_fPreCycPosYPFFreeze_met, &FOH_fPreCycPosYPFCorr_met,
                       &FOH_bPreCycPosYPFRS_bool);
    if (bTempTransPF_bool) {
        pAccObjPreProcess->pTgtObjData->fAccObjPosYTransPF_met =
            fTempTransPosYPT1PF_met;

    } else {
        pAccObjPreProcess->pTgtObjData->fAccObjPosYTransPF_met =
            fTempPosYPT1PF_met;
    }

    fTempTransPosYPT1LSM_met =
        FOHTransFilter(fTempPosYPT1LSM_met, pAccObjPreProcess->pObjTranControl,
                       fTempFactorA_fct, &FOH_fPreCycPosYLSM_met,
                       &FOH_fPreCycPosYLSMFreeze_met,
                       &FOH_fPreCycPosYLSMCorr_met, &FOH_bPreCycPosYRS_bool);
    fTempTransYawAng_met =
        FOHTransFilter(pFOPOutput->fAccObjRelHeadingAngle_rad,
                       pAccObjPreProcess->pObjTranControl, fTempFactorA_fct,
                       &FOH_fPreCycYawAng_rad, &FOH_fPreCycYawAngFreeze_rad,
                       &FOH_fPreCycYawAngCorr_rad, &FOH_bPreCycYawRS_bool);
    if (bTempTransAll_bool) {
        pAccObjPreProcess->pTgtObjData->fAccObjPosYTrans_met =
            fTempTransPosYPT1LSM_met;
        pAccObjPreProcess->pTgtObjData->fAccObjRelYawAngTrans_rad =
            fTempTransYawAng_met;
    } else {
        pAccObjPreProcess->pTgtObjData->fAccObjPosYTrans_met =
            fTempPosYPT1LSM_met;
        pAccObjPreProcess->pTgtObjData->fAccObjRelYawAngTrans_rad =
            pFOPOutput->fAccObjRelHeadingAngle_rad;
    }

    /* Debug output */
    pFOHDebug->bTransitionEnable_bool =
        pAccObjPreProcess->pObjTranControl->bTransitionEnable_bool;
    pFOHDebug->bTransitionReset_bool =
        pAccObjPreProcess->pObjTranControl->bTransitionReset_bool;
    pFOHDebug->bCutInOngoing_bool =
        pAccObjPreProcess->pObjTranControl->bCutInOngoing_bool;
    pFOHDebug->bCutOutOngoing_bool =
        pAccObjPreProcess->pObjTranControl->bCutOutOngoing_bool;
    pFOHDebug->bFreezeStopOnging_bool =
        pAccObjPreProcess->pObjTranControl->bFreezeStopOnging_bool;
    pFOHDebug->bObjValidOngoing_bool =
        pAccObjPreProcess->pObjTranControl->bObjValidOngoing_bool;
    pFOHDebug->fTransitionFactorA_fac =
        pAccObjPreProcess->pTgtObjData->fTransitionFactorA_fac;
    pFOHDebug->fAccObjPosYTransPF_met =
        pAccObjPreProcess->pTgtObjData->fAccObjPosYTransPF_met;
    pFOHDebug->fAccObjPosYTrans_met =
        pAccObjPreProcess->pTgtObjData->fAccObjPosYTrans_met;
    pFOHDebug->fAccObjRelYawAngTrans_rad =
        pAccObjPreProcess->pTgtObjData->fAccObjRelYawAngTrans_rad;
    pFOHDebug->bCutInDetected_bool = bCutInDetected_bool;
    pFOHDebug->bCutOutDetected_bool = bCutOutDetected_bool;
    pFOHDebug->bAccObjFreezeStop2_bool = bAccObjFreezeStop_bool;
    pFOHDebug->bObjValidity_bool = bObjValidity_bool;
    pFOHDebug->bAccObjValidRisingChk_bool = bAccObjValidRisingChk_bool;
    pFOHDebug->fTempPosYCorr_met = fTempPosYCorr_met;
    pFOHDebug->fTempPosYCorrBef_met = fTempPosYCorrBef_met;
    pFOHDebug->fTempPosYCorrAft_met = fTempPosYCorrAft_met;
    pFOHDebug->fTempPosYPT1PF_met = fTempPosYPT1PF_met;
    pFOHDebug->fTempPosYPT1LSM_met = fTempPosYPT1LSM_met;
    pFOHDebug->bTempTransAll_bool = bTempTransAll_bool;
    pFOHDebug->bTempTransPF_bool = bTempTransPF_bool;
    pFOHDebug->fTempFactorA_fct = fTempFactorA_fct;
    pFOHDebug->fTempTransPosYPT1PF_met = fTempTransPosYPT1PF_met;
    pFOHDebug->fTempTransPosYPT1LSM_met = fTempTransPosYPT1LSM_met;
    pFOHDebug->fTempTransYawAng_met = fTempTransYawAng_met;
    pFOHDebug->fTempPosYPT1LowpassT_sec = fTempPosYPT1LowpassT_sec;
    pFOHDebug->fTempPosYLSMLowpassT_sec = fTempPosYLSMLowpassT_sec;

    /* Reassign value, which is used in next cycle*/
    FOH_bPreCycCutInTranOng_bool =
        pAccObjPreProcess->pObjTranControl->bCutInOngoing_bool;
    FOH_bPreCycCutOutTranOng_bool =
        pAccObjPreProcess->pObjTranControl->bCutOutOngoing_bool;
    FOH_bPreCycFreezeStopTranOng_bool =
        pAccObjPreProcess->pObjTranControl->bFreezeStopOnging_bool;
    FOH_bPreCycObjValidTranOng_bool =
        pAccObjPreProcess->pObjTranControl->bObjValidOngoing_bool;
}

/*****************************************************************************
  Functionname:    FOHTransFilter                                           */ /*!

     @brief           Transition filter

     @description     Transition filter. This function is corresponding to
                      TransitionFilterPosY_PF/TransitionFilterPosY/TransitionFilterYaw
                      module in MBD.

     @param[in]       fInputValue          Input values that need to be filtered
     @param[in]       pObjTranControl      Pointer that points to object
   transition control information
     @param[in]       fTransFactorA        Transition factor A
     @param[in]       fPreCycInput         Pointer that points to input value
   that
   in previous cycle
     @param[in]       fPreCycInputFreeze   Pointer that points to input freeze
   that in previous cycle
     @param[in]       fPreCycInputCorr     Pointer that points to input
   correction
   that in previous cycle

     @return          fFilterOut           Filter output

   *****************************************************************************/
STATIc float32 FOHTransFilter(const float32 fInputValue,
                              const FOHObjTranControl_t* pObjTranControl,
                              float32 fTransFactorA,
                              float32* fPreCycInput,
                              float32* fPreCycInputFreeze,
                              float32* fPreCycInputCorr,
                              boolean* bPreCycRSFlipFlop) {
    float32 fFilterOut;  // result value
    float32 fTempFreezeOut;
    float32 fTempGradLimited;
    float32 fTempInputCorr;
    boolean bTempCompFlag1_bool;
    boolean bTempCompFlag2_bool;
    boolean bTempResetDisable_bool;
    boolean bTempEnableFilter_bool;

    /* Freeze */
    if (pObjTranControl->bTransitionEnable_bool &&
        (!pObjTranControl->bTransitionReset_bool)) {
        fTempFreezeOut = *fPreCycInputFreeze;
    } else {
        fTempFreezeOut = *fPreCycInput;
    }

    /* Factor A filter */
    fTempGradLimited =
        fTempFreezeOut * fTransFactorA + fInputValue * (1.f - fTransFactorA);

    /* Enable filter  conditions*/
    fTempInputCorr = fInputValue - ((!pObjTranControl->bTransitionEnable_bool)
                                        ? fInputValue
                                        : fTempGradLimited);
    bTempCompFlag1_bool =
        ((fTempInputCorr > 0.001f) && (*fPreCycInputCorr < -0.001f));
    bTempCompFlag2_bool =
        ((*fPreCycInputCorr > 0.001f) && (fTempInputCorr < -0.001f));
    bTempResetDisable_bool = (bTempCompFlag1_bool || bTempCompFlag2_bool) &&
                             (FOH_bPreCycObjTransCtrlEn_bool);
    bTempEnableFilter_bool =
        TUE_CML_RSFlipFlop(pObjTranControl->bTransitionReset_bool,
                           bTempResetDisable_bool, bPreCycRSFlipFlop);

    fFilterOut = bTempEnableFilter_bool ? fTempGradLimited : fInputValue;

    /* Reassign value, which is used in next cycle*/
    *fPreCycInput = fInputValue;
    *fPreCycInputFreeze = fTempFreezeOut;
    *fPreCycInputCorr = fTempInputCorr;
    FOH_bPreCycObjTransCtrlEn_bool = pObjTranControl->bTransitionEnable_bool;

    return fFilterOut;
}

/*****************************************************************************
  Functionname:    FOHTgtObjCtdGeneration */ /*!

@brief           Target object clothoid generation

@description     Target object clothoid generation calls
FOHPolyfitTgtObjClothoid
function to performs trajectory polyfit. This function is
corresponding
to TargetObjectClothoidGeneration module in MBD.

@param[in]       pHistoryControl          History control info. output by
FOHHistoryControl
@param[in]       pEgoMotion               Ego vehicle motion calculation
@param[in]       pTgtObjData              Target object data output by
FOHAccObjPreProcessing
@param[in]       pSystemPara              System parameter
@param[in,out]   pFOHOutput               Acc object trajectory curve output by
FOH
@param[in,out]   pTgtObjCtdInfoAndCoeff   Target object clothoid info. and
coeffcient
@param[in,out]   pFOHDebug                Debug output of FOH

@return          sTgtObjPFOutput          Target object info. after polyfit

@uml
@startuml
start
: Filter mean ego yaw rate;
note:Perform Lowpass filter to YawRate before trajectory polyfit
: Mean velocity X ;
note:Perform Mean filter to Ego velocity before trajectory polyfit
partition FOHPolyfitTgtObjClothoid{
fork
:Motion Compensation;
fork again
:Object validation;
fork again
:Add New SamplePoint;
fork again
:Polyfit validity check;
fork again
:Calculate 1st Polyfit;
fork again
:Calculate 3rd Polyfit;
end fork
note:Call FOHPolyfitTgtObjClothoid\nfunction
}
:Kalman filter output;
note:Trajectory polyfit information and coefficient output
stop
@enduml
*****************************************************************************/
STATIc FOHTgtObjPFOutput_t
FOHTgtObjCtdGeneration(const FOHHistoryControl_t* pHistoryControl,
                       const FOHEgoMotion_t* pEgoMotion,
                       const FOHTgtObjData_t* pTgtObjData,
                       const ODPRInSystemPara_t* pSystemPara,
                       ODPRFOHOut_t* pFOHOutput,
                       FOHTgtObjCtdInfoAndCoeff_t* pTgtObjCtdInfoAndCoeff,
                       ODPRFOHDebug_t* pFOHDebug) {
    FOHTgtObjPFInput_t sTgtObjPFInput = {0};
    FOHTgtObjPFOutput_t sTgtObjPFOutput = {0};  // result value

    float32 fTempFeatPtsMinPosX_met;
    float32 fTempAlpha_fct;
    float32 fTempYawRate_rps;
    float32 fTempYawRateOut_rps;

    /* Polyfit target object clothoid input structure */
    sTgtObjPFInput.bEnable_bool = pHistoryControl->bEnableHistory_bool;
    sTgtObjPFInput.bAddNewSample_bool = pHistoryControl->bSaveNewEntry_bool;
    sTgtObjPFInput.bReset_bool = pHistoryControl->bResetHistory_bool;

    sTgtObjPFInput.fObjXPos_met = pTgtObjData->fAccObjPosX_met;
    sTgtObjPFInput.fObjYPos_met = pTgtObjData->fAccObjPosYTransPF_met;
    sTgtObjPFInput.fObjXPosStdDev_met = pTgtObjData->fAccObjPosXStdDev_met;
    sTgtObjPFInput.fObjYPosStdDev_met = pTgtObjData->fAccObjPosYStdDev_met;
    sTgtObjPFInput.fTimeSinceLastCall_sec = pSystemPara->fSystemCylceTime_sec;

    sTgtObjPFInput.fEgoCrv_1pm = pEgoMotion->fEgoCrv_1pm;

    /* Filter mean ego yaw rate */
    fTempYawRate_rps = FOH_USE_ESTIM_POS_Y_BOOL
                           ? pEgoMotion->fYawRateRaw_rps
                           : pEgoMotion->fYawRateObjSync_rps;
    fTempYawRateOut_rps = (fTempYawRate_rps + FOH_fPreCycYawRate_rps) / 2.f;

    if (ODPR_Kb_FOHPT1YawRateEnable_bool) {
        fTempAlpha_fct =
            pSystemPara->fSystemCylceTime_sec /
            SafeDiv(TUE_CML_CalculatePolygonValue2D(
                14, FOH_sPT1TimeConstYawRate, pEgoMotion->fEgoVelX_mps));
        TUE_CML_LowPassFilter(&FOH_fPreCycYawRateLowPass_rps,
                              fTempYawRateOut_rps, fTempAlpha_fct);
        sTgtObjPFInput.fEgoYawRate_rps = FOH_fPreCycYawRateLowPass_rps;
    } else {
        sTgtObjPFInput.fEgoYawRate_rps = fTempYawRateOut_rps;
    }

    /* Mean velocity X */
    sTgtObjPFInput.fEgoVelX_mps =
        (pEgoMotion->fEgoVelX_mps + FOH_fPreCycEgoVelX_mps) / 2.f;

    /* Currently not used */
    sTgtObjPFInput.fPredYawRtVar_r2ps2 = 0.001f;
    sTgtObjPFInput.fPredVelXVar_m2 = 0.001f;
    sTgtObjPFInput.fModelPosYVar_m2 = 0.001f;
    sTgtObjPFInput.fModelYawVar_rad2 = 0.001f;
    sTgtObjPFInput.fModelCrvVar_1pm2 = 0.001f;
    sTgtObjPFInput.fModelCrvChngVar_1pm4 = 0.001f;
    sTgtObjPFInput.fObjMeasPosXVar_m2 = 0.001f;
    sTgtObjPFInput.fObjMeasPosYVar_m2 = 0.001f;
    sTgtObjPFInput.fCrvDecay_nu = FOH_PF_CRV_DECAY_NU;
    sTgtObjPFInput.fCrvChngDecay_nu = FOH_PF_CRV_CHNG_DECAY_NU;

    /* Polyfit target object clothoid input structure */
    fTempFeatPtsMinPosX_met =
        -1.f * ODPR_Kf_FOHFeatMinPosXTime_sec * pEgoMotion->fEgoVelX_mps;
    sTgtObjPFInput.fMinHistStartPosX_met =
        TUE_CML_Min(fTempFeatPtsMinPosX_met, ODPR_Kf_FOHFeatMinPosX_met);

    sTgtObjPFInput.fMaxSampleAge_sec = TUE_CML_CalculatePolygonValue2D(
        14, FOH_sMaxSampleAge, pEgoMotion->fEgoVelX_mps);

    sTgtObjPFInput.uMinNumValidSamples_nu = ODPR_Ku_FOHMinValidEntries_nu;
    sTgtObjPFInput.fMinHistLength_met =
        TUE_CML_Max(pEgoMotion->fEgoVelX_mps * ODPR_Kf_FOHMinHistLengthTime_sec,
                    ODPR_Kf_FOHMinHistLength_met);
    sTgtObjPFInput.fMaxGapEgoToHist_met =
        TUE_CML_Min(ODPR_Kf_FOHMaxHistStartX0_met,
                    pEgoMotion->fEgoVelX_mps * ODPR_Kf_FOHPredHistMaxPosX0_sec);
    sTgtObjPFInput.pWeightLastFit_nu = ODPR_Kf_FOHWeightLastPolyfit_nu;

    /* Polyfit target object clothoid  */
    sTgtObjPFOutput.fPosX0_met =
        0.f;  // Prevent prompt that using uninitialized structure
    FOHPolyfitTgtObjClothoid(&sTgtObjPFInput, &sTgtObjPFOutput, pFOHDebug);

    /* Kalman filter output check */
    pTgtObjCtdInfoAndCoeff->fDetaMeanDevToTraj_met =
        sTgtObjPFOutput.fMeanDevToTraj_1st_met -
        sTgtObjPFOutput.fMeanDevToTraj_3rd_met;
    pTgtObjCtdInfoAndCoeff->fHistLength_met =
        sTgtObjPFOutput.fTrajLength_met - sTgtObjPFOutput.fPosX0_met;
    pTgtObjCtdInfoAndCoeff->fFirstStoredPntX_met = sTgtObjPFOutput.fPosX0_met;
    pTgtObjCtdInfoAndCoeff->fPosX0_met = 0.f;

    /* FOH outputs */
    pFOHOutput->fMinHistoryStartPosX_met = sTgtObjPFInput.fMinHistStartPosX_met;
    pFOHOutput->fMinHistoryLength_met = sTgtObjPFInput.fMinHistLength_met;
    pFOHOutput->fMaxGapEgoToHistory_met = sTgtObjPFInput.fMaxGapEgoToHist_met;

    pFOHOutput->fLastStoredPointX_met = sTgtObjPFOutput.fLastStoredPntX_met;
    pFOHOutput->fLastStoredPointY_met = sTgtObjPFOutput.fLastStoredPntY_met;
    pFOHOutput->fLastStoredPointAge_sec = sTgtObjPFOutput.fLastStoredAge_sec;
    pFOHOutput->uiNumOfValidSamples_nu = sTgtObjPFOutput.uNumValidSamples_nu;
    pFOHOutput->fMeanDevToTraj_1st_met = sTgtObjPFOutput.fMeanDevToTraj_1st_met;
    pFOHOutput->fMeanDevToTraj_3rd_met = sTgtObjPFOutput.fMeanDevToTraj_3rd_met;
    pFOHOutput->fFirstStoredPointAge_sec = sTgtObjPFOutput.fFirstStoredAge_sec;
    pFOHOutput->fMeanStoredPointAge_sec = sTgtObjPFOutput.fMeanSampleAge_sec;

    pFOHOutput->fFirstStoredPointX_met = sTgtObjPFOutput.fPosX0_met;

    /* Debug output */
    pFOHDebug->bTrajInvalid1st_bool = sTgtObjPFOutput.bTrajInvalid1st_bool;
    pFOHDebug->bTrajInvalid3rd_bool = sTgtObjPFOutput.bTrajInvalid3rd_bool;
    pFOHDebug->fPosY0_1st_met = sTgtObjPFOutput.fPosY0_1st_met;
    pFOHDebug->fPosY0_3rd_met = sTgtObjPFOutput.fPosY0_3rd_met;
    pFOHDebug->fHeading_1st_rad = sTgtObjPFOutput.fHeading_1st_rad;
    pFOHDebug->fHeading_3rd_rad = sTgtObjPFOutput.fHeading_3rd_rad;
    pFOHDebug->fCrv_1pm = sTgtObjPFOutput.fCrv_1pm;
    pFOHDebug->fTempFeatPtsMinPosX_met = fTempFeatPtsMinPosX_met;
    pFOHDebug->fTempAlpha_fct = fTempAlpha_fct;
    pFOHDebug->fTempYawRate_rps = fTempYawRate_rps;
    pFOHDebug->fTempYawRateOut_rps = fTempYawRateOut_rps;
    pFOHDebug->fMaxSampleAge_sec = sTgtObjPFInput.fMaxSampleAge_sec;

    /* Reassign value, which is used in next cycle */
    FOH_fPreCycEgoVelX_mps = pEgoMotion->fEgoVelX_mps;
    FOH_sPreCycHistValid.bPreCycTrajInvalid_1st_bool =
        sTgtObjPFOutput.bTrajInvalid1st_bool;  // Used in FOHHistoryControl and
                                               // FOHAccObjStatusChk
    FOH_sPreCycHistValid.bPreCycTrajInvalid_3rd_bool =
        sTgtObjPFOutput.bTrajInvalid3rd_bool;
    FOH_sPreCycHistValid.fPreCycLastStoredPointX_met =
        sTgtObjPFOutput.fLastStoredPntX_met;
    FOH_sPreCycHistValid.uiPreCycNumOfValidSamples_nu =
        sTgtObjPFOutput.uNumValidSamples_nu;
    FOH_fPreCycYawRate_rps = fTempYawRate_rps;

    return sTgtObjPFOutput;
}

/*****************************************************************************
  Functionname:    FOHPolyfitSelection */ /*!

@brief           Polyfit selection

@description     Polyfit selection performs PosY0, Heading and Curve fading
using
       1st and 3rd polyfit coefficient. This function is corresponding
       to PolyfitSelection module in MBD.

@param[in]       pEgoVEDData       Dynamic data of ggo vehicle  from VED module
@param[in]       pTgtObjPFOutput   Target object info. after polyfit
@param[in,out]   pFOHDebug         Debug output of FOH

@return          sPolyfitSelec     Polyfit info. after selection

@uml
@startuml
start
partition FOHPolyfitSelection {
fork
:Weight position Y;
note:PosY0_3rd = PosY0_1st * (1-Weight)\n+ PosY0_3rd *Weight
fork again
:Weight heading;
note:Heading_3rd = Heading_1st * (1-Weight)\n+ Heading_3rd *Weight
fork again
:Weight curve ;
note:Curve = Curve * (1-Weight)
end fork
}
stop
@enduml
*****************************************************************************/
STATIc FOHPolyfitSelec_t
FOHPolyfitSelection(const ODPRInVEDVehDyn_t* pEgoVEDData,
                    const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
                    ODPRFOHDebug_t* pFOHDebug) {
    FOHPolyfitSelec_t sPolyfitSelec = {0};  // result value
    float32 fTempWeight_fac;

    fTempWeight_fac = TUE_CML_MinMax(
        0.f, 1.f,
        TUE_CML_CalculatePolygonValue2D(14, FOH_sWeightCrvPolyfit,
                                        pEgoVEDData->fEgoVelX_mps));

    /* Weight position Y */
    sPolyfitSelec.fPosY0_3rd_met =
        pTgtObjPFOutput->fPosY0_1st_met * (1.f - fTempWeight_fac) +
        pTgtObjPFOutput->fPosY0_3rd_met * fTempWeight_fac;

    /* Weight heading */
    sPolyfitSelec.fHeading_3rd_rad =
        pTgtObjPFOutput->fHeading_1st_rad * (1.f - fTempWeight_fac) +
        pTgtObjPFOutput->fHeading_3rd_rad * fTempWeight_fac;

    /* Weight curve */
    sPolyfitSelec.fCrv_1pm = 0.f * (1.f - fTempWeight_fac) +
                             pTgtObjPFOutput->fCrv_1pm * fTempWeight_fac;

    /* Debug output */
    pFOHDebug->fPolySelecWeight_fac = fTempWeight_fac;
    pFOHDebug->fPolySelecPosY0_3rd_met = sPolyfitSelec.fPosY0_3rd_met;
    pFOHDebug->fPolySelecHeading_3rd_rad = sPolyfitSelec.fHeading_3rd_rad;
    pFOHDebug->fPolySelecCrv_1pm = sPolyfitSelec.fCrv_1pm;

    return sPolyfitSelec;
}

/*****************************************************************************
  Functionname:    FOHTrajAttributes */ /*!

@brief           Trajectory attributes calculation

@description     Trajectory attributes calculation, including Straight
probability
             and Trace quality. This function is corresponding to
             TrajAttributes module in MBD.

@param[in]       pAccObjChk              Flags that acc object status check
@param[in]       pTgtObjPFOutput         Target object info. after polyfit
@param[in]       pEgoVEDData             Dynamic data of ggo vehicle  from VED
module
@param[in]       pPolyfitSelec           Polyfit info. after selection
@param[in]       pTgtObjCtdInfoAndCoeff  Target object clothoid info. and
coeffcient
@param[in,out]   pFOHOutput              Acc object trajectory curve output by
FOH
@param[in,out]   pFOHDebug               Debug output of FOH

@return          none

@uml
@startuml
start
partition FOHTrajAttributes {
partition TraceStraightProb {
fork
:MeanDevToTraj_1st;
note:Calc StraightProb based on\n1st trajectory polyfit mean deviation\n35 *
(1 - MeanDevToTraj_1st / 0.2)
fork again
:Curve;
note:Calc StraightProb based on\ncurve after PolyfitSelection\n25 * (1 -
Curve/ 0.00125)
fork again
:PosYOffset;
note:Calc StraightProb based on\nPosYOffset for a certain time\n40 * (1 -
PosYOffset / 0.6)
end fork
}
partition ObjTraceQuality {
fork
:MeanObjAge;
note:Calc TraceQuality based\non MeanObjAge\n30*(1–MeanObjAge/3)
fork again
:MeanDevToTraj_3rd;
note:Calc TraceQuality based \non
MeanDevToTraj_3rd\n15*(1–MeanDevToTraj_3rd/0.2)
fork again
:ValidSample;
note:Calc TraceQuality based \nonValidSample\n15 * ValidSample / 20
fork again
:HistLength;
note:Calc TraceQuality based \non HistLength\n20*HistLength/VelX/2.5
fork again
:TrajLength;
note:Calc TraceQuality based \nonTrajLength\n20 * TrajLength/VelX/1.5
end fork
}
}
stop
@enduml
*****************************************************************************/
STATIc void FOHTrajAttributes(
    const FOHAccObjStatusChk_t* pAccObjChk,
    const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
    const ODPRInVEDVehDyn_t* pEgoVEDData,
    const FOHPolyfitSelec_t* pPolyfitSelec,
    const FOHTgtObjCtdInfoAndCoeff_t* pTgtObjCtdInfoAndCoeff,
    ODPRFOHOut_t* pFOHOutput,
    ODPRFOHDebug_t* pFOHDebug) {
    float32 fTempEgoPosX_met;
    float32 fTempPosYCrv_met;
    float32 fTempPosYAbs_met;
    float32 fTempStrightProb1_perc = 0.f;
    float32 fTempStrightProb2_perc = 0.f;
    float32 fTempStrightProb3_perc = 0.f;
    uint8 uiTempStrightProb_perc;
    float32 fTempMeanSampleAge_sec;
    float32 fTempMeanDevToTraj_3rd_met;
    float32 fTempNumValidSamples_nu;
    float32 fTempVelXMax_mps;
    float32 fTempHistLength_met;
    float32 fTempTrajLength_met;
    float32 fTempTraceQual1_perc = 0.f;
    float32 fTempTraceQual2_perc = 0.f;
    float32 fTempTraceQual3_perc = 0.f;
    float32 fTempTraceQual4_perc = 0.f;
    float32 fTempTraceQual5_perc = 0.f;
    uint8 uiTempTraceQual_perc;

    if (pAccObjChk->bLSMInactive_bool) {
        /* Straight estimation */
        fTempEgoPosX_met = 0.7f * pEgoVEDData->fEgoVelX_mps;
        fTempPosYCrv_met =
            TUE_CML_Sqr(fTempEgoPosX_met) * (pPolyfitSelec->fCrv_1pm / 2.f);
        fTempPosYAbs_met =
            TUE_CML_Abs(fTempPosYCrv_met + pPolyfitSelec->fPosY0_3rd_met +
                        fTempEgoPosX_met * pPolyfitSelec->fHeading_3rd_rad);
        fTempStrightProb1_perc =
            40.f * (1.f - TUE_CML_MinMax(0.f, 1.f, (fTempPosYAbs_met / 0.6f)));
        fTempStrightProb2_perc =
            25.f * (1.f - TUE_CML_MinMax(0.f, 1.f,
                                         (TUE_CML_Abs(pPolyfitSelec->fCrv_1pm) /
                                          0.00125f)));
        fTempStrightProb3_perc =
            35.f *
            (1.f - TUE_CML_MinMax(
                       0.f, 1.f,
                       (TUE_CML_Abs(pTgtObjPFOutput->fMeanDevToTraj_1st_met) /
                        0.2f)));
        uiTempStrightProb_perc = (uint8)fTempStrightProb1_perc +
                                 (uint8)fTempStrightProb2_perc +
                                 (uint8)fTempStrightProb3_perc;

        pFOHOutput->uiObjTraceStraightProb_perc =
            TUE_CML_MinMax(0u, 100u, uiTempStrightProb_perc);

        /* Trace quality */
        if (pTgtObjPFOutput->bTrajInvalid1st_bool ||
            pTgtObjPFOutput->bTrajInvalid3rd_bool) {
            pFOHOutput->uiAccObjTraceQuality_perc = 0u;
        } else {
            fTempMeanSampleAge_sec = pTgtObjPFOutput->fMeanSampleAge_sec / 3.f;
            fTempMeanDevToTraj_3rd_met =
                pTgtObjPFOutput->fMeanDevToTraj_3rd_met / 0.2f;
            fTempNumValidSamples_nu =
                pTgtObjPFOutput->uNumValidSamples_nu / 20.f;
            fTempVelXMax_mps = TUE_CML_Max(pEgoVEDData->fEgoVelX_mps, 3.f);
            fTempHistLength_met = (pTgtObjCtdInfoAndCoeff->fHistLength_met /
                                   SafeDiv(fTempVelXMax_mps)) /
                                  2.5f;
            fTempTrajLength_met =
                (pTgtObjPFOutput->fTrajLength_met / SafeDiv(fTempVelXMax_mps)) /
                1.5f;
            fTempTraceQual1_perc =
                30.f * (1.f - ((fTempMeanSampleAge_sec > 1.f)
                                   ? 1.f
                                   : fTempMeanSampleAge_sec));
            fTempTraceQual2_perc =
                15.f * (1.f - ((fTempMeanDevToTraj_3rd_met > 1.f)
                                   ? 1.f
                                   : fTempMeanDevToTraj_3rd_met));
            fTempTraceQual3_perc = 15.f * ((fTempNumValidSamples_nu > 1.f)
                                               ? 1.f
                                               : fTempNumValidSamples_nu);
            fTempTraceQual4_perc =
                20.f *
                ((fTempHistLength_met > 1.f) ? 1.f : fTempHistLength_met);
            fTempTraceQual5_perc =
                20.f *
                ((fTempTrajLength_met > 1.f) ? 1.f : fTempTrajLength_met);

            uiTempTraceQual_perc =
                TUE_CML_MinMax(0u, 30u, (uint8)fTempTraceQual1_perc) +
                TUE_CML_MinMax(0u, 15u, (uint8)fTempTraceQual2_perc) +
                TUE_CML_MinMax(0u, 15u, (uint8)fTempTraceQual3_perc) +
                TUE_CML_MinMax(0u, 20u, (uint8)fTempTraceQual4_perc) +
                TUE_CML_MinMax(0u, 20u, (uint8)fTempTraceQual5_perc);

            pFOHOutput->uiAccObjTraceQuality_perc =
                TUE_CML_MinMax(0u, 100u, uiTempTraceQual_perc);
        }
    } else {
        pFOHOutput->uiObjTraceStraightProb_perc = 75u;
        pFOHOutput->uiAccObjTraceQuality_perc = 75u;
    }

    /* Debug output */
    pFOHDebug->fStrightProb1_perc = fTempStrightProb1_perc;
    pFOHDebug->fStrightProb2_perc = fTempStrightProb2_perc;
    pFOHDebug->fStrightProb3_perc = fTempStrightProb3_perc;
    pFOHDebug->fTraceQual1_perc = fTempTraceQual1_perc;
    pFOHDebug->fTraceQual2_perc = fTempTraceQual2_perc;
    pFOHDebug->fTraceQual3_perc = fTempTraceQual3_perc;
    pFOHDebug->fTraceQual4_perc = fTempTraceQual4_perc;
    pFOHDebug->fTraceQual5_perc = fTempTraceQual5_perc;
    pFOHDebug->fTempEgoPosX_met = fTempEgoPosX_met;
    pFOHDebug->fTempPosYCrv_met = fTempPosYCrv_met;
    pFOHDebug->fTempPosYAbs_met = fTempPosYAbs_met;
    pFOHDebug->uiTempStrightProb_perc = uiTempStrightProb_perc;
    pFOHDebug->fTempMeanSampleAge_sec = fTempMeanSampleAge_sec;
    pFOHDebug->fTempMeanDevToTraj_3rd_met = fTempMeanDevToTraj_3rd_met;
    pFOHDebug->fTempNumValidSamples_nu = fTempNumValidSamples_nu;
    pFOHDebug->fTempVelXMax_mps = fTempVelXMax_mps;
    pFOHDebug->fTempHistLength_met = fTempHistLength_met;
    pFOHDebug->fTempTrajLength_met = fTempTrajLength_met;
    pFOHDebug->uiTempTraceQual_perc = uiTempTraceQual_perc;
}

/*****************************************************************************
  Functionname:    FOHBasicLimitUint8 */ /*!

@brief           Basic function that value limit.(Not used, use TUE_CML_MinMax
instead)

@description     Basic function that value limit.

@param[in]       uiInput         Input value that needs to limit
@param[in]       uiLowLimit      Low limit
@param[in]       uiHighLimit     High limit

@return          uiOutput        Output value after limit

*****************************************************************************/
// STATIc uint8 FOHBasicLimitUint8(uint8 uiInput, uint8 uiLowLimit, uint8
// uiHighLimit)
//{
//    uint8 uiOutput; //result value
//
//    if (uiInput < uiLowLimit)
//    {
//        uiOutput = uiLowLimit;
//    }
//    else if (uiInput > uiHighLimit)
//    {
//        uiOutput = uiHighLimit;
//    }
//    else
//    {
//        uiOutput = uiInput;
//    }
//    return uiOutput;
//
//}

/*****************************************************************************
  Functionname:    FOHStrgtEstimFadingCrvLimit */ /*!

@brief           Straight estimation fading cruve limiter

@description     Straight estimation fading cruve limiter. This function
performs
          polyfit coefficient weighting use straight probability. it is
          corresponding to StraightEstimationFading module in MBD.

@param[in]       pTgtObjPFOutput      Target object info. after polyfit
@param[in]       pPolyfitSelec        Polyfit info. after selection
@param[in,out]   pFOHOutput           Acc object trajectory curve output by FOH
@param[in,out]   pFOHDebug            Debug output of FOH

@return          sStrgtEstFadCrvLimit Object info. after fading curvature limit

@uml
@startuml
start
partition FOHStrgtEstimFadingCrvLimit {
if (Use straight estimation directly) then (yes)
:Output coefficient after polyfitSelection;
note:PosY0=PosY0_3rd\nCurvature=Crv\nHeading=Heading_3rd
else (no)
fork
:Curve fading;
note:Curvature=Crv*(1-TraceStraightProb)
fork again
:PosY0 fading;
note:PosY0=PosY0_3rd*(1-TraceStraightProb)\n+PosY0_1st*TraceStraightProb
fork again
:Heading fading;
note:Heading=Heading_3rd*(1-TraceStraightProb)\n+Heading_1st*TraceStraightProb
end fork
endif
}
stop
@enduml
*****************************************************************************/
STATIc FOHStrgtEstFadCrvLimit_t
FOHStrgtEstimFadingCrvLimit(const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
                            const FOHPolyfitSelec_t* pPolyfitSelec,
                            const ODPRFOHOut_t* pFOHOutput,
                            ODPRFOHDebug_t* pFOHDebug) {
    FOHStrgtEstFadCrvLimit_t sStrgtEstFadCrvLimit = {0};  // result value
    float32 fTempFading_fac = 0.f;

    if (ODPR_Kb_FOHUseStraightEstim_bool) {
        fTempFading_fac =
            (float32)pFOHOutput->uiObjTraceStraightProb_perc / 100.f;

        sStrgtEstFadCrvLimit.fCurvature_1pm =
            pPolyfitSelec->fCrv_1pm * (1.f - fTempFading_fac);
        sStrgtEstFadCrvLimit.fPosY0_met =
            pPolyfitSelec->fPosY0_3rd_met * (1.f - fTempFading_fac) +
            pTgtObjPFOutput->fPosY0_1st_met * fTempFading_fac;
        sStrgtEstFadCrvLimit.fHeading_rad =
            pPolyfitSelec->fHeading_3rd_rad * (1.f - fTempFading_fac) +
            pTgtObjPFOutput->fHeading_1st_rad * fTempFading_fac;
    } else {
        sStrgtEstFadCrvLimit.fCurvature_1pm = pPolyfitSelec->fCrv_1pm;
        sStrgtEstFadCrvLimit.fPosY0_met = pPolyfitSelec->fPosY0_3rd_met;
        sStrgtEstFadCrvLimit.fHeading_rad = pPolyfitSelec->fHeading_3rd_rad;
    }

    /* Debug output */
    pFOHDebug->fStrgtFadCurvature_1pm = sStrgtEstFadCrvLimit.fCurvature_1pm;
    pFOHDebug->fStrgtFadPosY0_met = sStrgtEstFadCrvLimit.fPosY0_met;
    pFOHDebug->fStrgtFadHeading_rad = sStrgtEstFadCrvLimit.fHeading_rad;
    pFOHDebug->fTempFading_fac = fTempFading_fac;

    return sStrgtEstFadCrvLimit;
}

static void predict_laneKF(REAL32_T dT_laneKFLe,
                           REAL32_T dX_laneKFLe,
                           REAL32_T vehYawRate,
                           TUE_CML_sMatrix_t* x_laneKFLe,
                           TUE_CML_sMatrix_t* P_laneKFLe,
                           REAL32_T P_ABPLBP_LaneKFDynDistYFact_nu,
                           REAL32_T P_ABPLBP_LaneKFDynYawFactor_nu) {
    /*initialize local matrices and variables*/
    TUE_CML_CreateMatrix_M(A_laneKFLe, STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(Q_laneKFLe, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(B_laneKFLe, STATE_LENGTH_LANEKF, 1)
                TUE_CML_CreateMatrix_M(AX_laneKFLe, STATE_LENGTH_LANEKF, 1)
                    TUE_CML_CreateMatrix_M(A_trans_laneKFLe,
                                           STATE_LENGTH_LANEKF,
                                           STATE_LENGTH_LANEKF)
                        TUE_CML_CreateMatrix_M(PA_trans_laneKFLe,
                                               STATE_LENGTH_LANEKF,
                                               STATE_LENGTH_LANEKF)
                            TUE_CML_CreateMatrix_M(APA_trans_laneKFLe,
                                                   STATE_LENGTH_LANEKF,
                                                   STATE_LENGTH_LANEKF)

                                REAL32_T dXPow2_laneKFLe;
    REAL32_T dXPow3_laneKFLe;
    REAL32_T dXPow4_laneKFLe;
    REAL32_T dXPow5_laneKFLe;
    REAL32_T dXPow6_laneKFLe;
    REAL32_T dXPow7_laneKFLe;
    REAL32_T dXPow8_laneKFLe;
    REAL32_T Q00_laneKFLe;
    REAL32_T Q01_laneKFLe;
    REAL32_T Q02_laneKFLe;
    REAL32_T Q03_laneKFLe;
    REAL32_T Q11_laneKFLe;
    REAL32_T Q12_laneKFLe;
    REAL32_T Q13_laneKFLe;
    REAL32_T Q22_laneKFLe;
    REAL32_T Q23_laneKFLe;
    REAL32_T Q33_laneKFLe;
    REAL32_T sigmaSqr_laneKFLe;

    if (valid_laneKF) {
        /*initialize system matrix A_laneKFLe*/
        TUE_CML_InitMatrix_M(A_laneKFLe, STATE_LENGTH_LANEKF,
                             STATE_LENGTH_LANEKF, 0.0f);
        /*calculate dX_laneKFLe to the power of 2*/
        dXPow2_laneKFLe = dX_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 3*/
        dXPow3_laneKFLe = dXPow2_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 4*/
        dXPow4_laneKFLe = dXPow3_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 5*/
        dXPow5_laneKFLe = dXPow4_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 6*/
        dXPow6_laneKFLe = dXPow5_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 7*/
        dXPow7_laneKFLe = dXPow6_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 8*/
        dXPow8_laneKFLe = dXPow7_laneKFLe * dX_laneKFLe;

        /*row 0: 1 dX_laneKFLe 1/2*dX_laneKFLe^2 1/6*dX_laneKFLe^3*/
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 0, 0) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 0, 1) = dX_laneKFLe;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 0, 2) = 0.5f * dXPow2_laneKFLe;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 0, 3) =
            1.0f / 6.0f * dXPow3_laneKFLe;
        /*row 1: 0 1 -dX_laneKFLe - dX_laneKFLe^2*/
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 1, 1) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 1, 2) = dX_laneKFLe;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 1, 3) = 0.5f * dXPow2_laneKFLe;
        /*row 2: 0 0 1 dX_laneKFLe*/
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 2, 2) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 2, 3) = dX_laneKFLe;
        /*row 3: 0 0 0 1*/
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 3, 3) = 1.0f;

        /*calculate sigma squared*/
        sigmaSqr_laneKFLe = kappa2diff_sigma_laneKF * kappa2diff_sigma_laneKF;

        /*calculate process noise covariance matrix Q_laneKFLe elements*/
        Q00_laneKFLe = dXPow8_laneKFLe * sigmaSqr_laneKFLe / 576.0f;
        Q01_laneKFLe = dXPow7_laneKFLe * sigmaSqr_laneKFLe / 48.0f;
        Q02_laneKFLe = dXPow6_laneKFLe * sigmaSqr_laneKFLe / 48.0f;
        Q03_laneKFLe = dXPow5_laneKFLe * sigmaSqr_laneKFLe / 24.0f;

        Q11_laneKFLe = dXPow6_laneKFLe * sigmaSqr_laneKFLe / 4.0f;
        Q12_laneKFLe = dXPow5_laneKFLe * sigmaSqr_laneKFLe / 4.0f;
        Q13_laneKFLe = dXPow4_laneKFLe * sigmaSqr_laneKFLe / 2.0f;

        Q22_laneKFLe = dXPow4_laneKFLe * sigmaSqr_laneKFLe / 4.0f;
        Q23_laneKFLe = dXPow3_laneKFLe * sigmaSqr_laneKFLe / 2.0f;

        Q33_laneKFLe = dXPow2_laneKFLe * sigmaSqr_laneKFLe;

        /*set process noise covariance matrix Q_laneKFLe row 0*/
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 0, 0) =
            P_ABPLBP_LaneKFDynDistYFact_nu *
            Q00_laneKFLe;  // Factor needed to increase the dynamic of DistY
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 0, 1) = Q01_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 0, 2) = Q02_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 0, 3) = Q03_laneKFLe;
        /*set process noise covariance matrix Q_laneKFLe row 1*/
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 1, 0) = Q01_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 1, 1) =
            P_ABPLBP_LaneKFDynYawFactor_nu * Q11_laneKFLe;  // Factor needed to
                                                            // increase the
                                                            // dynamic of
                                                            // HeadingAngle
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 1, 2) = Q12_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 1, 3) = Q13_laneKFLe;
        /*set process noise covariance matrix Q_laneKFLe row 2*/
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 2, 0) = Q02_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 2, 1) = Q12_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 2, 2) =
            100.0f *
            Q22_laneKFLe;  // Factor needed to increase the dynamic of Kappa
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 2, 3) = Q23_laneKFLe;
        /*set process noise covariance matrix Q_laneKFLe row 3*/
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 3, 0) = Q03_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 3, 1) = Q13_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 3, 2) = Q23_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 3, 3) = 2.0f * Q33_laneKFLe;

        /*additional noise caused by vehicle movement*/
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 0, 0) +=
            (dT_laneKFLe * dX_laneKFLe * vehYawRateStdDev_laneKF) *
            (dT_laneKFLe * dX_laneKFLe * vehYawRateStdDev_laneKF);
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 1, 1) +=
            (dT_laneKFLe * vehYawRateStdDev_laneKF) *
            (dT_laneKFLe * vehYawRateStdDev_laneKF);

        /*initialize steering matrix*/

        TUE_CML_InitMatrix_M(B_laneKFLe, STATE_LENGTH_LANEKF, 1, 0.0f);
        TUE_CML_GetMatrixElement_M(B_laneKFLe, 0, 0) =
            -dT_laneKFLe * dX_laneKFLe;
        TUE_CML_GetMatrixElement_M(B_laneKFLe, 1, 0) = -dT_laneKFLe;
        /*AX_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(AX_laneKFLe, A_laneKFLe, x_laneKFLe);
        /*Bu*/
        TUE_CML_ScaleMatrix_M(B_laneKFLe, vehYawRate);
        /*x_laneKFLe = AX_laneKFLe + Bu*/
        TUE_CML_AddMatrices_M(x_laneKFLe, AX_laneKFLe, B_laneKFLe);
        /*A_laneKFLe'*/
        TUE_CML_TransposeMatrix_M(A_trans_laneKFLe, A_laneKFLe);
        /*PA'*/
        TUE_CML_MutiplyMatrices_M(PA_trans_laneKFLe, P_laneKFLe,
                                  A_trans_laneKFLe);
        /*APA'*/
        TUE_CML_MutiplyMatrices_M(APA_trans_laneKFLe, A_laneKFLe,
                                  PA_trans_laneKFLe);
        /*P_laneKFLe=APA'+Q_laneKFLe*/
        TUE_CML_AddMatrices_M(P_laneKFLe, APA_trans_laneKFLe, Q_laneKFLe);

        /*Measurement Weight*/
        measWeight_laneKF = 0.0f;
        status_laneKF = 3u;
    }
}

static void init_laneKF(const TUE_CML_sMatrix_t* z_laneKFLe,
                        TUE_CML_sMatrix_t* R_laneKFLe,
                        REAL32_T quality,
                        TUE_CML_sMatrix_t* x_laneKFLe,
                        TUE_CML_sMatrix_t* P_laneKFLe,
                        UINT8_T P_ABPLBP_LaneKFMnInitQual_perc,
                        REAL32_T P_ABPLBP_LaneKFInitRFactor_nu) {
    TUE_CML_CreateMatrix_M(H_trans_laneKFLe, STATE_LENGTH_LANEKF,
                           STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(H_laneKFLe, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(tmp_laneKFLe, STATE_LENGTH_LANEKF,
                                   STATE_LENGTH_LANEKF)
                TUE_CML_CreateMatrix_M(R_scaled_laneKFLe, STATE_LENGTH_LANEKF,
                                       STATE_LENGTH_LANEKF)

                    if (quality > P_ABPLBP_LaneKFMnInitQual_perc) {
        /*scale R_laneKFLe*/
        TUE_CML_InitMatrix_M(R_scaled_laneKFLe, STATE_LENGTH_LANEKF,
                             STATE_LENGTH_LANEKF, 0.0f);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFLe, 0, 0) =
            TUE_CML_GetMatrixElement_M(R_laneKFLe, 0, 0);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFLe, 1, 1) =
            TUE_CML_GetMatrixElement_M(R_laneKFLe, 1, 1);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFLe, 2, 2) =
            TUE_CML_GetMatrixElement_M(R_laneKFLe, 2, 2);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFLe, 3, 3) =
            TUE_CML_GetMatrixElement_M(R_laneKFLe, 3, 3);
        TUE_CML_ScaleMatrix_M(R_scaled_laneKFLe, P_ABPLBP_LaneKFInitRFactor_nu);

        /*initialize matrix H_laneKFLe*/
        TUE_CML_CreateIdentityMatrix_M(H_laneKFLe, STATE_LENGTH_LANEKF);
        /*initialize matrix H_laneKFLe'*/
        TUE_CML_TransposeMatrix_M(H_trans_laneKFLe, H_laneKFLe);
        /*tmp_laneKFLe = H_laneKFLe'R_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(tmp_laneKFLe, H_trans_laneKFLe,
                                  R_scaled_laneKFLe);
        /*[H_laneKFLe'R_laneKFLe]H_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(P_laneKFLe, tmp_laneKFLe, H_laneKFLe);
        /*fill x_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(x_laneKFLe, H_trans_laneKFLe, z_laneKFLe);
        /*fill other states*/
        valid_laneKF = 1U;
        internalQuality_laneKF = 0.0f;
        measWeight_laneKF = 0.0f;
        status_laneKF = 4u;
    }
}

static void update_laneKF(const TUE_CML_sMatrix_t* z_laneKFLe,
                          const TUE_CML_sMatrix_t* R_laneKFLe,
                          REAL32_T weight,
                          TUE_CML_sMatrix_t* x_laneKFLe,
                          TUE_CML_sMatrix_t* P_laneKFLe,
                          REAL32_T P_ABPLBP_LaneKFKGainFac_nu) {
    /*temporary matrices*/
    TUE_CML_CreateMatrix_M(
        I_laneKFLe, STATE_LENGTH_LANEKF,
        STATE_LENGTH_LANEKF) TUE_CML_CreateMatrix_M(H_laneKFLe,
                                                    STATE_LENGTH_LANEKF,
                                                    STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(
            H_trans_laneKFLe, STATE_LENGTH_LANEKF,
            STATE_LENGTH_LANEKF) TUE_CML_CreateMatrix_M(K_laneKFLe,
                                                        STATE_LENGTH_LANEKF,
                                                        STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(
                PH_trans_laneKFLe, STATE_LENGTH_LANEKF,
                STATE_LENGTH_LANEKF) TUE_CML_CreateMatrix_M(HPH_trans_laneKFLe,
                                                            STATE_LENGTH_LANEKF,
                                                            STATE_LENGTH_LANEKF)
                TUE_CML_CreateMatrix_M(HPH_transPlusR_laneKFLe,
                                       STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF)
                    TUE_CML_CreateMatrix_M(HPH_transPlusR_inv_laneKFLe,
                                           STATE_LENGTH_LANEKF,
                                           STATE_LENGTH_LANEKF)
                        TUE_CML_CreateMatrix_M(
                            HX_laneKFLe, STATE_LENGTH_LANEKF,
                            1) TUE_CML_CreateMatrix_M(ZSubHX_laneKFLe,
                                                      STATE_LENGTH_LANEKF, 1)
                            TUE_CML_CreateMatrix_M(KZSubHX_laneKFLe,
                                                   STATE_LENGTH_LANEKF, 1)
                                TUE_CML_CreateMatrix_M(KH_laneKFLe,
                                                       STATE_LENGTH_LANEKF,
                                                       STATE_LENGTH_LANEKF)
                                    TUE_CML_CreateMatrix_M(ISubKH_laneKFLe,
                                                           STATE_LENGTH_LANEKF,
                                                           STATE_LENGTH_LANEKF)
                                        TUE_CML_CreateMatrix_M(
                                            P_tmp_laneKFLe, STATE_LENGTH_LANEKF,
                                            STATE_LENGTH_LANEKF)

        /*update_laneKF state with measurement*/
        TUE_CML_CreateIdentityMatrix_M(I_laneKFLe, STATE_LENGTH_LANEKF);
    TUE_CML_CreateIdentityMatrix_M(H_laneKFLe, STATE_LENGTH_LANEKF);
    TUE_CML_TransposeMatrix_M(H_trans_laneKFLe, H_laneKFLe);
    /*PH'*/
    TUE_CML_MutiplyMatrices_M(PH_trans_laneKFLe, P_laneKFLe, H_trans_laneKFLe);
    /*H_laneKFLe[PH']*/
    TUE_CML_MutiplyMatrices_M(HPH_trans_laneKFLe, H_laneKFLe,
                              PH_trans_laneKFLe);
    /*[HPH']+R_laneKFLe*/
    TUE_CML_AddMatrices_M(HPH_transPlusR_laneKFLe, HPH_trans_laneKFLe,
                          R_laneKFLe);
    /*inv([HPH'+R_laneKFLe]*/
    // CML_v_InvertMatrix(HPH_transPlusR_inv_laneKFLe, HPH_transPlusR_laneKFLe);
    TUE_CML_InvertMatrix_M(HPH_transPlusR_inv_laneKFLe,
                           HPH_transPlusR_laneKFLe);
    /*if inversion fails num. of cols and rows are set to 0*/
    if (HPH_transPlusR_inv_laneKFLe->Desc.col > 0) {
        /*K_laneKFLe=[PH'][inv(HPH'+R_laneKFLe)]*/
        TUE_CML_MutiplyMatrices_M(K_laneKFLe, PH_trans_laneKFLe,
                                  HPH_transPlusR_inv_laneKFLe);
        /*K_laneKFLe=K_laneKFLe*weight*/
        TUE_CML_ScaleMatrix_M(K_laneKFLe, weight);

        /* if degradedUpdate == true -> Set first row of K_laneKFLe to
         * FACTOR*K_laneKFLe -> weight of Disty = FACTOR*weight */
        if (weight < 1.0f) {
            /*set process noise covariance matrix Q_laneKFLe row 0*/
            TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 0) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 0);
            TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 1) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 1);
            TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 2) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 2);
            TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 3) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 3);
        }

        /*HX_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(HX_laneKFLe, H_laneKFLe, x_laneKFLe);
        /*z_laneKFLe-[HX_laneKFLe]*/
        TUE_CML_SubtractMatrices_M(ZSubHX_laneKFLe, z_laneKFLe, HX_laneKFLe);
        /*K_laneKFLe[(z_laneKFLe-HX_laneKFLe)]*/
        TUE_CML_MutiplyMatrices_M(KZSubHX_laneKFLe, K_laneKFLe,
                                  ZSubHX_laneKFLe);
        /*x_laneKFLe=x_laneKFLe+K_laneKFLe(z_laneKFLe-HX_laneKFLe)*/
        TUE_CML_AddMatrices_M(x_laneKFLe, x_laneKFLe, KZSubHX_laneKFLe);
        /*KH_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(KH_laneKFLe, K_laneKFLe, H_laneKFLe);
        /*I_laneKFLe-[KH_laneKFLe]*/
        TUE_CML_SubtractMatrices_M(ISubKH_laneKFLe, I_laneKFLe, KH_laneKFLe);
        /*P_laneKFLe=[(I_laneKFLe-K_laneKFLe*H_laneKFLe)]P_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(P_tmp_laneKFLe, ISubKH_laneKFLe, P_laneKFLe);
        TUE_CML_CopyMatrix_M(P_laneKFLe, P_tmp_laneKFLe);

        measWeight_laneKF = weight;
    } else {
        reset_laneKF(x_laneKFLe, P_laneKFLe);
    }
}

static void maintenance_laneKF(TUE_CML_sMatrix_t* x_laneKFLe,
                               TUE_CML_sMatrix_t* P_laneKFLe,
                               REAL32_T P_ABPLBP_LaneKFIncQual_1ps,
                               REAL32_T P_ABPLBP_LaneKFDecQualDeg_1ps,
                               REAL32_T P_ABPLBP_LaneKFDecQualPred_1ps,
                               REAL32_T deltaT_sec) {
    if (valid_laneKF) {
        /*Full Update, Degraded Update or No Update (predict only)? */
        if (measWeight_laneKF > 0.9f) {
            /* Full Update */
            status_laneKF = 1u;
            /* Increase internal quality by P_ABPLBP_LaneKFIncQual_1ps per
             * second*/
            internalQuality_laneKF += P_ABPLBP_LaneKFIncQual_1ps * deltaT_sec;
        } else if (measWeight_laneKF > 0.0f) {
            /* Degraded Update */
            status_laneKF = 2u;
            /* Decrease internal quality by P_ABPLBP_LaneKFDecQualDeg_1ps per
             * seconds */
            internalQuality_laneKF -=
                P_ABPLBP_LaneKFDecQualDeg_1ps * deltaT_sec;
        } else {
            /* No Update - Predict Only */
            status_laneKF = 3u;
            /* Decrease internal quality by P_ABPLBP_LaneKFDecQualPred_1ps per
             * seconds */
            internalQuality_laneKF -=
                P_ABPLBP_LaneKFDecQualPred_1ps * deltaT_sec;
        }

        /* Boundary check: 0 <= internalQuality <= 100 */
        if (internalQuality_laneKF < 0.0f) {
            internalQuality_laneKF = 0.0f;
            reset_laneKF(x_laneKFLe, P_laneKFLe);
        } else if (internalQuality_laneKF > 100.0f) {
            internalQuality_laneKF = 100.0f;
        }
    } else {
        status_laneKF = 0u;
    }
}

static void reset_laneKF(TUE_CML_sMatrix_t* x_laneKFLe,
                         TUE_CML_sMatrix_t* P_laneKFLe) {
    /*reset_laneKF parameters*/
    TUE_CML_InitMatrix_M(x_laneKFLe, STATE_LENGTH_LANEKF, 1, 0.0f);
    TUE_CML_InitMatrix_M(P_laneKFLe, STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF,
                         0.0f);
    valid_laneKF = FALSE;
    internalQuality_laneKF = 0.0f;
    measWeight_laneKF = 0.0f;
    status_laneKF = 5u;
}

void laneKalmanFilter(const laneKFInTypeV3* inputs, laneKFOutType* outputs) {
    /*state Matrix x_laneKFLe*/
    TUE_CML_CreateMatrix_M(x_laneKFLe, STATE_LENGTH_LANEKF, 1)
        /*covariance matrix P_laneKFLe*/
        TUE_CML_CreateMatrix_M(P_laneKFLe, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
        /*measurement matrix z_laneKFLe*/
        TUE_CML_CreateMatrix_M(z_laneKFLe, STATE_LENGTH_LANEKF, 1)
        /*measurement variance matrix R_laneKFLe*/
        TUE_CML_CreateMatrix_M(R_laneKFLe, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)

            REAL32_T dX_laneKFLe;
    // REAL32_T PosY0Diff_laneKFLe;
    /*calculate dX_laneKFLe*/
    dX_laneKFLe = inputs->sf_DeltaT_sec * inputs->sf_VehVelX_mps;
    vehYawRateStdDev_laneKF = inputs->sf_VehYawRateStdDev_radps;
    /*geometric dependent model error*/
    kappa2diff_sigma_laneKF =
        1 / inputs->sf_LaneKFErrCoeff1_met /
        (inputs->sf_LaneKFErrCoeff2_mps * inputs->sf_DeltaT_sec) /
        (inputs->sf_LaneKFErrCoeff2_mps * inputs->sf_DeltaT_sec);

    /*In case of an lane change allow the PosY0 position to jump in the
     * corresponding cycle - only if kalman filter has been valid*/
    if (status_laneKF > 0 && status_laneKF < 4) {
        /*check lane change detection*/
        if (inputs->sf_LaneChange_bool) {
            /*a lateral position jump is forced in case of a lane change*/
            TUE_CML_GetMatrixElement_M(x_laneKFLe, 0, 0) = inputs->sf_PosY0_met;
        }
    }

    /*predict_laneKF -  will only be executed if valid_laneKF state is set*/
    predict_laneKF(inputs->sf_DeltaT_sec, dX_laneKFLe,
                   inputs->sf_VehYawRate_radps, x_laneKFLe, P_laneKFLe,
                   inputs->sf_LaneKFDynDistYFact_nu,
                   inputs->sf_LaneKFDynYawFactor_nu);

    /*initialize measurement vector z_laneKFLe*/
    TUE_CML_GetMatrixElement_M(z_laneKFLe, 0, 0) = inputs->sf_PosY0_met;
    TUE_CML_GetMatrixElement_M(z_laneKFLe, 1, 0) = inputs->sf_HeadingAngle_rad;
    TUE_CML_GetMatrixElement_M(z_laneKFLe, 2, 0) = inputs->sf_Crv_1pm;
    TUE_CML_GetMatrixElement_M(z_laneKFLe, 3, 0) = inputs->sf_CrvChng_1pm2;

    /*initialize R_laneKFLe matrix*/
    TUE_CML_GetMatrixElement_M(R_laneKFLe, 0, 0) =
        inputs->sf_PosY0StdDev_met * inputs->sf_PosY0StdDev_met;
    TUE_CML_GetMatrixElement_M(R_laneKFLe, 1, 1) =
        inputs->sf_HeadingAngleStdDev_rad * inputs->sf_HeadingAngleStdDev_rad;
    TUE_CML_GetMatrixElement_M(R_laneKFLe, 2, 2) =
        inputs->sf_CrvStdDev_1pm * inputs->sf_CrvStdDev_1pm;
    TUE_CML_GetMatrixElement_M(R_laneKFLe, 3, 3) =
        inputs->sf_CrvChngStdDev_1pm2 * inputs->sf_CrvChngStdDev_1pm2;

    /*Init or Update*/
    if (!valid_laneKF && !(inputs->sf_DegradedUpdate_bool)) {
        /*initialization - for A_laneKFLe better initialization the R_laneKFLe
         * matrix is multiplied with A_laneKFLe constant (>1)*/
        init_laneKF(z_laneKFLe, R_laneKFLe,
                    inputs->sf_OverallMeasurementQuality_perc, x_laneKFLe,
                    P_laneKFLe, inputs->sf_LaneKFMnInitQual_perc,
                    inputs->sf_LaneKFInitRFactor_nu);
    }
    /*Update only if valid AND measurement quality >
       sf_LaneKFMnUpdateQual_perc*/
    else if (valid_laneKF && (inputs->sf_OverallMeasurementQuality_perc >=
                              (inputs->sf_LaneKFMnUpdateQual_perc))) {
        if (inputs->sf_DegradedUpdate_bool) {
            /*degraded update_laneKF - quality and weight are the 2
             * parameters*/
            update_laneKF(z_laneKFLe, R_laneKFLe,
                          inputs->sf_LaneKFDegradeWeight_nu, x_laneKFLe,
                          P_laneKFLe, inputs->sf_LaneKFKGainFac_nu);
        } else {
            /*full update_laneKF*/
            update_laneKF(z_laneKFLe, R_laneKFLe, 1.0f, x_laneKFLe, P_laneKFLe,
                          inputs->sf_LaneKFKGainFac_nu);
        }
    }
    /*maintenance_laneKF*/
    if (!(status_laneKF == 4)) {
        maintenance_laneKF(x_laneKFLe, P_laneKFLe, inputs->sf_LaneKFIncQual_1ps,
                           inputs->sf_LaneKFDecQualDeg_1ps,
                           inputs->sf_LaneKFDecQualPred_1ps,
                           inputs->sf_DeltaT_sec);
    }
    /*Reset filter if basic lane data are not valid*/
    if (!inputs->sf_LaneDataValid_bool) reset_laneKF(x_laneKFLe, P_laneKFLe);
    /*Set outputs*/
    outputs->sf_PosY0_met = TUE_CML_GetMatrixElement_M(x_laneKFLe, 0, 0);
    outputs->sf_HeadingAngle_rad = TUE_CML_GetMatrixElement_M(x_laneKFLe, 1, 0);
    outputs->sf_Crv_1pm = TUE_CML_GetMatrixElement_M(x_laneKFLe, 2, 0);
    outputs->sf_CrvChng_1pm2 = TUE_CML_GetMatrixElement_M(x_laneKFLe, 3, 0);
    outputs->sf_KFStatus_btf = status_laneKF;
    outputs->sf_QualityMeasure_perc = (UINT8_T)(internalQuality_laneKF);
}

/*****************************************************************************
  Functionname:    FOHLowPassFilter                                    */ /*!

                    @brief           Polyfit coefficient lowpass filter

                    @description     Polyfit coefficient lowpass filter. This
                  function
                  performs
                                     lowpass filter using information from
                  FOHStrgtEstimFadingCrvLimit.
                                     It is corresponding to LowPassFilter module
                  in
                  MBD.

                    @param[in]       pEgoVEDData          Dynamic data of ego
                  vehicle
                  from
                  VED module
                    @param[in]       pStrgtEstFadCrvLimit Object info. after
                  fading
                  curvature limit
                    @param[in]       pTgtObjPFOutput      Target object info.
                  after
                  polyfit
                    @param[in]       pSystemPara          System parameter
                    @param[in,out]   pFOHDebug            Debug output of FOH

                    @return          sLowPassFilter       Object info. after
                  lowpass
                  filter

                    @uml
                    @startuml
                      start
                      partition FOHLowPassFilter {
                      fork
                      :PosY0 Lowpass filter;
                      fork again
                      :Heading Lowpass filter;
                      fork again
                      :Curve Lowpass filter;
                      fork again
                      :Change of Curve Lowpass filter;
                      end fork
                      note:Lowpass parameter comes from ego velocity look-up
                  table;\nSet
                  Reset and Enable input of lowpass filter based on \nvalidity
                  of 1st
                  and
                  3rd trajectory polyfit;
                      }
                      stop
                    @enduml
                  *****************************************************************************/
STATIc FOHLowPassFilter_t
FOHLowPassFilter(const ODPRInVEDVehDyn_t* pEgoVEDData,
                 const FOHStrgtEstFadCrvLimit_t* pStrgtEstFadCrvLimit,
                 const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
                 const ODPRInSystemPara_t* pSystemPara,
                 ODPRFOHDebug_t* pFOHDebug) {
    FOHLowPassFilter_t sLowPassFilter = {0};  // result value
    boolean bTempTrajInvalidChk_bool;
    boolean bTempEnableFlg_bool;
    boolean bTempResetFlg_bool;
    float32 fTempVehVelX_kph;
    float32 fTempPosYLowpassT_sec;
    float32 fTempHeadingLowpassT_sec;
    float32 fTempCurveLowpassT_sec;
    float32 fTempCrvChngLowpassT_sec;

    bTempTrajInvalidChk_bool = pTgtObjPFOutput->bTrajInvalid1st_bool ||
                               pTgtObjPFOutput->bTrajInvalid3rd_bool;
    bTempEnableFlg_bool = !bTempTrajInvalidChk_bool;
    bTempResetFlg_bool =
        (bTempTrajInvalidChk_bool != FOH_bPreCycTrajInvalid_bool);

    fTempVehVelX_kph = 3.6f * pEgoVEDData->fEgoVelX_mps;

    /* PosY0 Lowpass filter */
    if (bTempResetFlg_bool) {
        FOP_fPreCycPosY0LowPass_met = pStrgtEstFadCrvLimit->fPosY0_met;
    } else {
        if (bTempEnableFlg_bool && FOH_PT1_POSY0_ENABLE_BOOL) {
            fTempPosYLowpassT_sec = TUE_CML_CalculatePolygonValue2D(
                14, FOH_sPT1TimeConstPosY0, fTempVehVelX_kph);
            TUE_CML_LowPassFilter(&FOP_fPreCycPosY0LowPass_met,
                                  pStrgtEstFadCrvLimit->fPosY0_met,
                                  (pSystemPara->fSystemCylceTime_sec /
                                   SafeDiv(fTempPosYLowpassT_sec)));
        } else {
        }
    }

    /* Heading Lowpass filter */
    if (bTempResetFlg_bool) {
        FOP_fPreCycHeadLowPass_met = pStrgtEstFadCrvLimit->fHeading_rad;
    } else {
        if (bTempEnableFlg_bool && FOH_PT1_HEAD_ENABLE_BOOL) {
            fTempHeadingLowpassT_sec =
                TUE_CML_Max(TUE_CML_CalculatePolygonValue2D(
                                14, FOH_sPT1TimeConstHead, fTempVehVelX_kph),
                            pSystemPara->fSystemCylceTime_sec);
            TUE_CML_LowPassFilter(&FOP_fPreCycHeadLowPass_met,
                                  pStrgtEstFadCrvLimit->fHeading_rad,
                                  (pSystemPara->fSystemCylceTime_sec /
                                   SafeDiv(fTempHeadingLowpassT_sec)));
        } else {
        }
    }

    /* Curve Lowpass filter  */
    if (bTempResetFlg_bool) {
        FOP_fPreCycCurveLowPass_met = pStrgtEstFadCrvLimit->fCurvature_1pm;
    } else {
        if (bTempEnableFlg_bool && FOH_PT1_CRV_ENABLE_BOOL) {
            fTempCurveLowpassT_sec =
                TUE_CML_Max(TUE_CML_CalculatePolygonValue2D(
                                14, FOH_sPT1TimeConstCrv, fTempVehVelX_kph),
                            pSystemPara->fSystemCylceTime_sec);
            TUE_CML_LowPassFilter(&FOP_fPreCycCurveLowPass_met,
                                  pStrgtEstFadCrvLimit->fCurvature_1pm,
                                  (pSystemPara->fSystemCylceTime_sec /
                                   SafeDiv(fTempCurveLowpassT_sec)));
        } else {
        }
    }

    /* Change of Curve Lowpass filter  */
    if (bTempResetFlg_bool) {
        FOP_fPreCycCrvChngLowPass_met = pTgtObjPFOutput->fChngOfCrv_1pm2;
    } else {
        if (bTempEnableFlg_bool && FOH_PT1_CRV_CHNG_ENABLE_BOOL) {
            fTempCrvChngLowpassT_sec =
                TUE_CML_Max(TUE_CML_CalculatePolygonValue2D(
                                14, FOH_sPT1TimeConstCrvChng, fTempVehVelX_kph),
                            pSystemPara->fSystemCylceTime_sec);
            TUE_CML_LowPassFilter(&FOP_fPreCycCrvChngLowPass_met,
                                  pTgtObjPFOutput->fChngOfCrv_1pm2,
                                  (pSystemPara->fSystemCylceTime_sec /
                                   SafeDiv(fTempCrvChngLowpassT_sec)));
        } else {
        }
    }

    /* Left lane kalman filter */
    laneKFInTypeV3 sLKFInput = {0};
    laneKFOutType sLKFOutput = {0};

    sLKFInput.sf_PosY0_met = pStrgtEstFadCrvLimit->fPosY0_met;
    sLKFInput.sf_HeadingAngle_rad = pStrgtEstFadCrvLimit->fHeading_rad;
    sLKFInput.sf_Crv_1pm = pStrgtEstFadCrvLimit->fCurvature_1pm;
    sLKFInput.sf_CrvChng_1pm2 = pTgtObjPFOutput->fChngOfCrv_1pm2;
    sLKFInput.sf_Length_met = 0.0F;
    sLKFInput.sf_PosY0StdDev_met = ODPR_PosY0StdDev_met;
    sLKFInput.sf_HeadingAngleStdDev_rad = ODPR_HeadingStdDev_met;
    sLKFInput.sf_CrvStdDev_1pm = ODPR_CurvatureStdDev_met;
    sLKFInput.sf_CrvChngStdDev_1pm2 = ODPR_CurvatureRateStdDev_met;
    sLKFInput.sf_VehYawRateStdDev_radps = 0.01f;

    sLKFInput.sf_VehVelX_mps = TUE_CML_Max_M(pEgoVEDData->fEgoVelX_mps, 0.01f);
    sLKFInput.sf_VehYawRate_radps = pEgoVEDData->fEgoYawRate_rps;
    sLKFInput.sf_DeltaT_sec = pSystemPara->fSystemCylceTime_sec;
    sLKFInput.sf_LaneDataValid_bool = bTempEnableFlg_bool;
    sLKFInput.sf_DegradedUpdate_bool = FALSE;
    sLKFInput.sf_OverallMeasurementQuality_perc = 100u;
    sLKFInput.sf_LaneChange_bool = bTempResetFlg_bool;
    sLKFInput.sf_LaneKFErrCoeff1_met = 650.0F;
    sLKFInput.sf_LaneKFErrCoeff2_mps = 35.0F;
    sLKFInput.sf_LaneKFInitRFactor_nu = 3.0F;
    sLKFInput.sf_LaneKFDegradeWeight_nu = 0.2F;
    sLKFInput.sf_LaneKFMnUpdateQual_perc = 30U;
    sLKFInput.sf_LaneKFMnInitQual_perc = 35U;
    sLKFInput.sf_LaneKFIncQual_1ps = 33.F;
    sLKFInput.sf_LaneKFDecQualDeg_1ps = 33.F;
    sLKFInput.sf_LaneKFDecQualPred_1ps = 300.F;
    sLKFInput.sf_LaneKFKGainFac_nu = 1.F;
    sLKFInput.sf_LaneKFDynYawFactor_nu = 2.F;
    sLKFInput.sf_LaneKFDynDistYFact_nu = 2500.F;
    sLKFInput.sf_LaneKFDynCrvFact_nu = 100.F;
    sLKFInput.sf_LaneKFDynCrvRateFact_nu = 2.F;

    /*  */
    laneKalmanFilter(&sLKFInput, &sLKFOutput);

    sLowPassFilter.fPosY0_met =
        (bTempEnableFlg_bool && FOH_PT1_POSY0_ENABLE_BOOL)
            ? FOP_fPreCycPosY0LowPass_met
            : pStrgtEstFadCrvLimit->fPosY0_met;
    sLowPassFilter.fHeading_rad =
        (bTempEnableFlg_bool && FOH_PT1_HEAD_ENABLE_BOOL)
            ? FOP_fPreCycHeadLowPass_met
            : pStrgtEstFadCrvLimit->fHeading_rad;
    sLowPassFilter.fCurvature_1pm =
        (bTempEnableFlg_bool && FOH_PT1_CRV_ENABLE_BOOL)
            ? FOP_fPreCycCurveLowPass_met
            : pStrgtEstFadCrvLimit->fCurvature_1pm;
    sLowPassFilter.fChngOfCrv_1pm2 =
        (bTempEnableFlg_bool && FOH_PT1_CRV_CHNG_ENABLE_BOOL)
            ? FOP_fPreCycCrvChngLowPass_met
            : pTgtObjPFOutput->fChngOfCrv_1pm2;

    sLowPassFilter.fPosY0_met = ODPR_ENABLE_KALMANFILTER
                                    ? sLKFOutput.sf_PosY0_met
                                    : sLowPassFilter.fPosY0_met;
    sLowPassFilter.fHeading_rad = ODPR_ENABLE_KALMANFILTER
                                      ? sLKFOutput.sf_HeadingAngle_rad
                                      : sLowPassFilter.fHeading_rad;
    sLowPassFilter.fCurvature_1pm = ODPR_ENABLE_KALMANFILTER
                                        ? sLKFOutput.sf_Crv_1pm
                                        : sLowPassFilter.fCurvature_1pm;
    sLowPassFilter.fChngOfCrv_1pm2 = ODPR_ENABLE_KALMANFILTER
                                         ? sLKFOutput.sf_CrvChng_1pm2
                                         : sLowPassFilter.fChngOfCrv_1pm2;

    /* Debug output */
    pFOHDebug->bLowPassResetFlg_bool = bTempResetFlg_bool;
    pFOHDebug->bLowPassEnableFlg_bool = bTempEnableFlg_bool;
    pFOHDebug->fLowPassPosY0_met = sLowPassFilter.fPosY0_met;
    pFOHDebug->fLowPassHeading_rad = sLowPassFilter.fHeading_rad;
    pFOHDebug->fLowPassCurvature_1pm = sLowPassFilter.fCurvature_1pm;
    pFOHDebug->fLowPassChngOfCrv_1pm2 = sLowPassFilter.fChngOfCrv_1pm2;
    pFOHDebug->bTempTrajInvalidChk_bool = bTempTrajInvalidChk_bool;
    pFOHDebug->fTempVehVelX_kph = fTempVehVelX_kph;
    pFOHDebug->fTempPosYLowpassT_sec = fTempPosYLowpassT_sec;
    pFOHDebug->fTempHeadingLowpassT_sec = fTempHeadingLowpassT_sec;
    pFOHDebug->fTempCurveLowpassT_sec = fTempCurveLowpassT_sec;
    pFOHDebug->fTempCrvChngLowpassT_sec = fTempCrvChngLowpassT_sec;

    /* Reassign value, which is used in next cycle*/
    FOH_bPreCycTrajInvalid_bool = bTempTrajInvalidChk_bool;

    return sLowPassFilter;
}

/*****************************************************************************
  Functionname:    FOHLSMTgtTrajProcess */ /*!

@brief           LSM target trajectory processing

@description     LSM target trajectory processing. This function calculate
trajectory
    coefficient directly without polyfit when LSM mode is active. It
    is corresponding to LSM_TgtTraj_Processing  module in MBD.

@param[in]       pEgoMotion           Ego vehicle motion calculation
@param[in]       pTgtObjData          Target object data output by
FOHAccObjPreProcessing
@param[in,out]   pFOHDebug            Debug output of FOH

@return          sLSMTgtTrajProcess   Object info. after LSM processing

@uml
@startuml
start
partition FOHLSMTgtTrajProcess {
if (Use curve only) then (yes)
:weight curve;
note:PosY0LSM and HeadingLSM are set to 0
else (no)
:Set PosY(LSM) and heading after \nFOHAccObjPreProcessing as LSM coeff
output;
note:CurveLSM is set to 0
endif
}
stop
@enduml
*****************************************************************************/
STATIc FOHLSMTgtTrajProcess_t
FOHLSMTgtTrajProcess(const FOHEgoMotion_t* pEgoMotion,
                     const FOHTgtObjData_t* pTgtObjData,
                     ODPRFOHDebug_t* pFOHDebug) {
    FOHLSMTgtTrajProcess_t sLSMTgtTrajProcess = {0};  // result value
    float32 fTempC0Calculate_lpm = 0.f;
    float32 fTempMaxCrv_lpm = 0.f;

    /* Output directly */
    sLSMTgtTrajProcess.fLengthLSM_met = pTgtObjData->fAccObjPosX_met;

    if (FOH_USE_CRV_ONLY_LSM_BOOL) {
        /* Position Y */
        sLSMTgtTrajProcess.fPosY0LSM_met = 0.f;

        /* Heading */
        sLSMTgtTrajProcess.fHeadLSM_rad = 0.f;

        /* Weight curve */
        fTempC0Calculate_lpm =
            2.f * pTgtObjData->fAccObjPosYTrans_met /
            SafeDiv(TUE_CML_Sqr(pTgtObjData->fAccObjPosX_met));
        fTempMaxCrv_lpm = TUE_CML_Max(
            fTempC0Calculate_lpm,
            pEgoMotion->fEgoCrv_1pm - FOH_MAX_DELTA_EGO_CRV_LSM_LPM);
        sLSMTgtTrajProcess.fCrvLSM_1pm =
            TUE_CML_Min(fTempMaxCrv_lpm, pEgoMotion->fEgoCrv_1pm +
                                             FOH_MAX_DELTA_EGO_CRV_LSM_LPM);
    } else {
        sLSMTgtTrajProcess.fPosY0LSM_met = pTgtObjData->fAccObjPosYTrans_met;
        sLSMTgtTrajProcess.fHeadLSM_rad =
            pTgtObjData->fAccObjRelYawAngTrans_rad;
        sLSMTgtTrajProcess.fCrvLSM_1pm = 0.f;
    }

    /* Debug output */
    pFOHDebug->fLengthLSM_met = sLSMTgtTrajProcess.fLengthLSM_met;
    pFOHDebug->fPosY0LSM_met = sLSMTgtTrajProcess.fPosY0LSM_met;
    pFOHDebug->fHeadLSM_rad = sLSMTgtTrajProcess.fHeadLSM_rad;
    pFOHDebug->fCrvLSM_1pm = sLSMTgtTrajProcess.fCrvLSM_1pm;
    pFOHDebug->fTempC0Calculate_lpm = fTempC0Calculate_lpm;
    pFOHDebug->fTempMaxCrv_lpm = fTempMaxCrv_lpm;

    return sLSMTgtTrajProcess;
}

/*****************************************************************************
  Functionname:    FOHTgtTrajTransition                                  */ /*!

              @brief           Target trajectory transition

              @description     Target trajectory transition. This function
            Performs
            trajectory
                               coefficient decision output depending on LSM mode
            is
            active or
                               not. It is corresponding to TgtTrajTransition
            module
            in
            MBD.

              @param[in]       pLowPassFilter       Object info. after lowpass
            filter
              @param[in]       pTgtObjPFOutput      Target object info. after
            polyfit
              @param[in]       pLSMTgtTrajProcess   Object info. after LSM
            processing
              @param[in]       pAccObjChk           Flags that acc object status
            check
              @param[in,out]   pTrajLength_met      Trajectory length
              @param[in,out]   pFOHOutput           Acc object trajectory curve
            output
            by FOH
              @param[in,out]   pFOHDebug            Debug output of FOH

              @return          sModeTranControl     Object info. after
            trajectory
            transition

              @uml
              @startuml
                start
                partition FOHTgtTrajTransition {
                if (LSM is valid) then (yes)
                    :Use LSM coefficient;
                    note:LSM coefficient comes from \nFOHLSMTgtTrajProcess
                else (no)
                    :Use polyfit cofficient;
                    note:polyfit cofficient comes from \nFOHLowPassFilter,which
            filters
            \noutputs of TrajectoryPolyfit
                endif
                :Trajectory polyfit coefficient filter;
                note:Filter methods and steps are similar to \nTransitionSwitch
            of
            FOHAccObjPreProcessing
                }
                stop
              @enduml
            *****************************************************************************/
STATIc FOHModeTranControl_t
FOHTgtTrajTransition(const ODPRInSystemPara_t* pSystemPara,
                     const FOHLowPassFilter_t* pLowPassFilter,
                     const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
                     const FOHLSMTgtTrajProcess_t* pLSMTgtTrajProcess,
                     const FOHAccObjStatusChk_t* pAccObjChk,
                     ODPRFOHOut_t* pFOHOutput,
                     ODPRFOHDebug_t* pFOHDebug) {
    FOHModeTranControl_t sModeTranControl;  // result value
    float32 fTempPosY0_met;
    float32 fTempHeading_rad;
    float32 fTempTrajLength_met;
    float32 fTempCrv_lpm;
    boolean bTempLSMChangeFlag_bool;
    boolean bTempLSMIvdEdge_bool;

    bTempLSMIvdEdge_bool =
        (pAccObjChk->bLSMInactive_bool != FOH_bPreCycLSMState_bool);
    bTempLSMChangeFlag_bool =
        FOH_bPreCycAccObjValid_bool && bTempLSMIvdEdge_bool;

    sModeTranControl.bTransitionEnable_bool = BASICTurnOffDelay(
        bTempLSMChangeFlag_bool, FOH_ACC_OBJ_CHNG_DURATION_SEC,
        pSystemPara->fSystemCylceTime_sec, &FOH_fLSMChngTurnOffDelay_sec);
    sModeTranControl.bTransitionReset_bool = bTempLSMChangeFlag_bool;

    /* Calculate transition factor A */
    if (sModeTranControl.bTransitionReset_bool) {
        FOH_fLSMFactorATimer_sec = 0.f;
    } else {
        FOH_fLSMFactorATimer_sec =
            sModeTranControl.bTransitionEnable_bool
                ? (FOH_fLSMFactorATimer_sec + pSystemPara->fSystemCylceTime_sec)
                : FOH_fLSMFactorATimer_sec;
    }
    sModeTranControl.fTransitionValueA_fac =
        0.5f * (TUE_CML_GDBcos_52(FOH_fLSMFactorATimer_sec /
                                  SafeDiv(FOH_ACC_OBJ_CHNG_DURATION_SEC) *
                                  TUE_CML_Pi) +
                1.f);

    /* LSM transition ongoing */
    if (sModeTranControl.bTransitionEnable_bool &&
        (!sModeTranControl.bTransitionReset_bool)) {
        sModeTranControl.bLSMTransOngoing_bool =
            FOH_bPreCycLSMTransOngoing_bool;
    } else {
        sModeTranControl.bLSMTransOngoing_bool = bTempLSMChangeFlag_bool;
    }

    /* Cosine transition */
    fTempPosY0_met = (pAccObjChk->bLSMInactive_bool)
                         ? pLowPassFilter->fPosY0_met
                         : pLSMTgtTrajProcess->fPosY0LSM_met;
    fTempHeading_rad = (pAccObjChk->bLSMInactive_bool)
                           ? pLowPassFilter->fHeading_rad
                           : pLSMTgtTrajProcess->fHeadLSM_rad;
    fTempTrajLength_met = (pAccObjChk->bLSMInactive_bool)
                              ? pTgtObjPFOutput->fTrajLength_met
                              : pLSMTgtTrajProcess->fLengthLSM_met;
    fTempCrv_lpm = (pAccObjChk->bLSMInactive_bool)
                       ? pLowPassFilter->fCurvature_1pm
                       : pLSMTgtTrajProcess->fCrvLSM_1pm;

    pFOHOutput->fTgtObjPosX0_met = 0.f;
    pFOHOutput->fTgtObjPosY0_met = FOHCosineTransition(
        fTempPosY0_met, &sModeTranControl, &FOH_fPreCycPosY0_met,
        &FOH_fPreCycPosY0Freeze_met, &FOH_fPreCycPosY0Corr_met,
        &FOH_bPreCycCosPosY0_bool);
    pFOHOutput->fTgtObjHeading_rad =
        FOHCosineTransition(fTempHeading_rad, &sModeTranControl,
                            &FOH_fPreCycHead_rad, &FOH_fPreCycHeadFreeze_rad,
                            &FOH_fPreCycHeadCorr_rad, &FOH_bPreCycCosHead_bool);
    pFOHOutput->fTgtObjCurve_1pm =
        FOHCosineTransition(fTempCrv_lpm, &sModeTranControl,
                            &FOH_fPreCycCrv_1pm, &FOH_fPreCycCrvFreeze_1pm,
                            &FOH_fPreCycCrvCorr_1pm, &FOH_bPreCycCosCrv_bool);
    pFOHOutput->fTgtObjCurveDer_1pm2 = 0.f;

    pFOHOutput->fTgtObjLength_met = FOHCosineTransition(
        fTempTrajLength_met, &sModeTranControl, &FOH_fPreCycTrajLength_met,
        &FOH_fPreCycTrajLengthFreeze_met, &FOH_fPreCycTrajLengthCorr_met,
        &FOH_bPreCycCosLength_bool);

    /* Debug output */
    pFOHDebug->bModeTransitionEnable_bool =
        sModeTranControl.bTransitionEnable_bool;
    pFOHDebug->bModeTransitionReset_bool =
        sModeTranControl.bTransitionReset_bool;
    pFOHDebug->bModeLSMTransOngoing_bool =
        sModeTranControl.bLSMTransOngoing_bool;
    pFOHDebug->fModeTransitionValueA_fac =
        sModeTranControl.fTransitionValueA_fac;
    pFOHDebug->fTempPosY0_met = fTempPosY0_met;
    pFOHDebug->fTempHeading_rad = fTempHeading_rad;
    pFOHDebug->fTempTrajLength2_met = fTempTrajLength_met;
    pFOHDebug->fTempCrv_lpm = fTempCrv_lpm;
    pFOHDebug->bTempLSMChangeFlag_bool = bTempLSMChangeFlag_bool;
    pFOHDebug->bTempLSMIvdEdge_bool = bTempLSMIvdEdge_bool;

    /* Reassign value, which is used in next cycle*/
    FOH_bPreCycAccObjValid_bool = pAccObjChk->bAccObjValid_bool;
    FOH_bPreCycLSMTransOngoing_bool = sModeTranControl.bLSMTransOngoing_bool;
    FOH_bPreCycLSMState_bool = pAccObjChk->bLSMInactive_bool;

    return sModeTranControl;
}

/*****************************************************************************
  Functionname:    FOHCosineTransition                                        */ /*!

@brief           Cosine transition

@description     Cosine transition. This function is corresponding to
                CosineTransitionPosY0/CosineTransitionHead/CosineTransitionLength/CosineTransitionCrv
                module in MBD.

@param[in]       fInputValue          Input values that need to be filtered
@param[in]       pModeTranControl     Pointer that points to object transition
control information
@param[in]       fPreCycInput         Pointer that points to input value that
in previous cycle
@param[in]       fPreCycInputFreeze   Pointer that points to input freeze that
in previous cycle
@param[in]       fPreCycInputCorr     Pointer that points to input correction
that in previous cycle
@param[in,out]   pFOHDebug            Debug output of FOH

@return          fTransOut            Transition output

*****************************************************************************/
STATIc float32 FOHCosineTransition(const float32 fInputValue,
                                   const FOHModeTranControl_t* pModeTranControl,
                                   float32* fPreCycInput,
                                   float32* fPreCycInputFreeze,
                                   float32* fPreCycInputCorr,
                                   boolean* bPreCycRSFlipFlop) {
    float32 fTransOut;  // result value
    float32 fTempFreezeOut;
    float32 fTempGradLimited;
    float32 fTempInputCorr;
    boolean bTempCompFlag1_bool;
    boolean bTempCompFlag2_bool;
    boolean bTempResetDisable_bool;
    boolean bTempEnableFilter_bool;

    /* Freeze */
    if (pModeTranControl->bTransitionEnable_bool &&
        (!pModeTranControl->bTransitionReset_bool)) {
        fTempFreezeOut = *fPreCycInputFreeze;
    } else {
        fTempFreezeOut = *fPreCycInput;
    }

    /* Factor A filter */
    if (pModeTranControl->bTransitionEnable_bool) {
        fTempGradLimited =
            fTempFreezeOut * pModeTranControl->fTransitionValueA_fac +
            fInputValue * (1.f - pModeTranControl->fTransitionValueA_fac);
    } else {
        fTempGradLimited = fInputValue;
    }

    /* Enable filter conditions*/
    fTempInputCorr = fInputValue - ((!pModeTranControl->bTransitionEnable_bool)
                                        ? fInputValue
                                        : fTempGradLimited);
    bTempCompFlag1_bool =
        ((fTempInputCorr > 0.001f) && (*fPreCycInputCorr < -0.001f));
    bTempCompFlag2_bool =
        ((*fPreCycInputCorr > 0.001f) && (fTempInputCorr < -0.001f));
    bTempResetDisable_bool = (bTempCompFlag1_bool || bTempCompFlag2_bool) &&
                             (FOH_bPreCycModeTransCtrlEn_bool) &&
                             (!pModeTranControl->bTransitionReset_bool);
    bTempEnableFilter_bool =
        TUE_CML_RSFlipFlop(pModeTranControl->bTransitionReset_bool,
                           bTempResetDisable_bool, bPreCycRSFlipFlop);

    fTransOut = bTempEnableFilter_bool ? fTempGradLimited : fInputValue;

    /* Reassign value, which is used in next cycle*/
    *fPreCycInput = fTransOut;
    *fPreCycInputFreeze = fTempFreezeOut;
    *fPreCycInputCorr = fTempInputCorr;
    FOH_bPreCycModeTransCtrlEn_bool = pModeTranControl->bTransitionEnable_bool;

    return fTransOut;
}

/*****************************************************************************
  Functionname:    FOHMinObjPosX                                           */ /*!

        @brief           Min object position X

        @description     Min object position X. This function outputs trajectory
      length.
                         It is corresponding to MinObjPosX module in MBD.

        @param[in]       pTrajLength_met   Trajectory length
        @param[in,out]   pFOHOutput        Acc object trajectory curve output by
      FOH

        @return          none

      *****************************************************************************/
STATIc void FOHMinObjPosX(const ODPRInVEDVehDyn_t* pEgoVEDData,
                          ODPRFOHOut_t* pFOHOutput) {
    float32 fTempMinTrajLength_met;

    if (FOH_MIN_POSX_VELX_ON_BOOL) {
        fTempMinTrajLength_met = TUE_CML_CalculatePolygonValue2D(
            14, FOH_sMinTrajectoryLength, pEgoVEDData->fEgoVelX_mps);
        pFOHOutput->fTgtObjLength_met =
            TUE_CML_Max(pFOHOutput->fTgtObjLength_met, fTempMinTrajLength_met);
    } else {
    }
}

/*****************************************************************************
  Functionname:    FOHInvalidBitField */ /*!

@brief           Invalid bitfield output

@description     Invalid bitfield output. This function is corresponding to
          InvalidBitField module in MBD.

@param[in]       pAccObjChk        Flags that acc object status check
@param[in]       pObjTranControl   Pointer that points to object transition
control information
@param[in]       pTgtObjPFOutput   Target object info. after polyfit
@param[in]       pHistoryControl   History control info. output by
FOHHistoryControl
@param[in]       pModeTranControl  Object info. after trajectory transition
@param[in,out]   pFOHOutput        Acc object trajectory curve output by FOH

@return          none

*****************************************************************************/
STATIc void FOHInvalidBitField(const FOHAccObjStatusChk_t* pAccObjChk,
                               const FOHObjTranControl_t* pObjTranControl,
                               const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
                               const FOHHistoryControl_t* pHistoryControl,
                               const FOHModeTranControl_t* pModeTranControl,
                               ODPRFOHOut_t* pFOHOutput) {
    boolean bTempTrajInvalid_bool;
    boolean bTempObjTrajInvalid_bool;

    bTempTrajInvalid_bool = (pTgtObjPFOutput->bTrajInvalid1st_bool ||
                             pTgtObjPFOutput->bTrajInvalid3rd_bool);
    bTempObjTrajInvalid_bool =  // This is the main Bit which TJAOBF checks: OBF
                                // should be invalid if either Object history is
                                // invalid in history mode or ODPFOP is invalid
                                // in Low speed mode
        ((bTempTrajInvalid_bool && pAccObjChk->bLSMInactive_bool) ||
         ((!pAccObjChk->bAccObjValid_bool) &&
          (!pAccObjChk->bLSMInactive_bool)));

    pFOHOutput->fTgtObjClothoidInvalidCheck_btf =
        ((!pAccObjChk->bAccObjValid_bool) << 0) |
        (pAccObjChk->bAccObjFreezeStop_bool << 1) |
        (pObjTranControl->bObjValidOngoing_bool << 2) |
        (pObjTranControl->bCutInOngoing_bool << 3) |
        (pObjTranControl->bCutOutOngoing_bool << 4) |
        (pObjTranControl->bFreezeStopOnging_bool << 5) |
        ((!pTgtObjPFOutput->bTrajUpdate_bool) << 6) |
        (bTempTrajInvalid_bool << 7) | ((!pAccObjChk->bLSMInactive_bool) << 8) |
        (pAccObjChk->bLSMInactive_bool << 9) |
        (pModeTranControl->bLSMTransOngoing_bool << 10) | (FALSE << 11) |
        ((!pHistoryControl->bEnableHistory_bool) << 12) |
        (pHistoryControl->bResetHistory_bool << 13) |
        (bTempObjTrajInvalid_bool << 14);

    if (!pAccObjChk->bLSMInactive_bool) {
        pFOHOutput->fTgtObjClothoidInvalidCheck_btf =
            pFOHOutput->fTgtObjClothoidInvalidCheck_btf &
            0xFF3F;  // clear bit 6 & 7
    }
}

/*****************************************************************************
  Functionname:    ODPR_FOH_Exec                                              */ /*!

@brief           Main function of Front Object History

@description     Front Object History that polyfit acc object history
trajectory.
                The Fitted trajectory is used in lateral funtion that object
                following in TJA.

@param[in]       reqPorts             Input structure of FOH function
                     pFOPOutput      Validity check results output by FOP
                     pSystemPara     System parameter
                     pEgoVEDData     Dynamic data of ggo vehicle  from VED
module
@param[in]       param                Param structure of FOH function
                     pFOHParam       FOH parameters
@param[in,out]   proPorts             Output structure of FOH function
                     pFOHOutput      Acc object trajectory curve output by FOH
@param[in,out]   debugInfo            Debug info of FOH function
                     pFOHDebug       Debug signal of FOH

@return          none
*****************************************************************************/
void ODPR_FOH_Exec(const FOHInReq_t* reqPorts,
                   const ODPRParam_t* param,
                   FOHOutPro_t* proPorts,
                   FOHDebug_t* debugInfo) {
    /* Input and output wrapper */
    const ODPRFOPOut_t* pFOPOutput = reqPorts->pFOPOutData;
    const ODPRInSystemPara_t* pSystemPara = reqPorts->pSystemPara;
    const ODPRInVEDVehDyn_t* pEgoVEDData = reqPorts->pEgoVehSig;
    ODPRFOHOut_t* pFOHOutput = proPorts->pFOHOutData;
    ODPRFOHDebug_t* pFOHDebug = debugInfo->pFOHDebug;

    /* VARIABLES */
    FOHEgoMotion_t sEgoMotion = {0};
    FOHAccObjStatusChk_t sAccObjChk = {0};
    FOHHistoryControl_t sHistoryControl = {0};
    FOHObjTranControl_t sObjTranControl = {0};
    FOHTgtObjData_t sTgtObjData = {0};
    FOHAccObjPreProcessing_t sAccObjPreProcess = {&sObjTranControl,
                                                  &sTgtObjData};
    FOHTgtObjPFOutput_t sTgtObjPFOutput = {0};
    FOHTgtObjCtdInfoAndCoeff_t sTgtObjCtdInfoAndCoeff = {0};
    FOHPolyfitSelec_t sPolyfitSelec = {0};
    FOHLSMTgtTrajProcess_t sLSMTgtTrajProcess = {0};
    FOHStrgtEstFadCrvLimit_t sStrgtEstFadCrvLimit = {0};
    FOHLowPassFilter_t sLowPassFilter = {0};
    FOHModeTranControl_t sModeTranControl = {0};

    /* Output directly */
    pFOHOutput->uiAccObjTraceCurveQuality_perc = 0u;
    pFOHOutput->fAccObjTraceCurve_1pm = 0.f;

    /* Ego vehicle motion calculation */
    sEgoMotion = FOHEgoMotionCalc(pEgoVEDData, pFOHDebug);

    /* Acc object status check */
    sAccObjChk =
        FOHAccObjStatusChk(pEgoVEDData, pSystemPara, pFOPOutput, pFOHDebug);

    /* History control */
    sHistoryControl = FOHHistoryControl(pEgoVEDData, pFOPOutput, &sAccObjChk,
                                        pFOHOutput, pFOHDebug);

    /* Acc object data preprocessing */
    FOHAccObjPreProcessing(&sEgoMotion, &sAccObjChk, pSystemPara, pFOPOutput,
                           &sAccObjPreProcess, pFOHDebug);

    /* Target object clothoid generation */
    sTgtObjPFOutput = FOHTgtObjCtdGeneration(
        &sHistoryControl, &sEgoMotion, &sTgtObjData, pSystemPara, pFOHOutput,
        &sTgtObjCtdInfoAndCoeff, pFOHDebug);

    /* Polyfit selection */
    sPolyfitSelec =
        FOHPolyfitSelection(pEgoVEDData, &sTgtObjPFOutput, pFOHDebug);

    /* Trajectory attributes calculation */
    FOHTrajAttributes(&sAccObjChk, &sTgtObjPFOutput, pEgoVEDData,
                      &sPolyfitSelec, &sTgtObjCtdInfoAndCoeff, pFOHOutput,
                      pFOHDebug);

    /* Straight estimation fading cruve limiter */
    sStrgtEstFadCrvLimit = FOHStrgtEstimFadingCrvLimit(
        &sTgtObjPFOutput, &sPolyfitSelec, pFOHOutput, pFOHDebug);

    /* Straight estimation fading cruve limiter */
    sLowPassFilter = FOHLowPassFilter(pEgoVEDData, &sStrgtEstFadCrvLimit,
                                      &sTgtObjPFOutput, pSystemPara, pFOHDebug);

    /* LSM target trajectory processing */
    sLSMTgtTrajProcess =
        FOHLSMTgtTrajProcess(&sEgoMotion, &sTgtObjData, pFOHDebug);

    /* Target trajectory transition */
    sModeTranControl = FOHTgtTrajTransition(
        pSystemPara, &sLowPassFilter, &sTgtObjPFOutput, &sLSMTgtTrajProcess,
        &sAccObjChk, pFOHOutput, pFOHDebug);

    /* Min object position X */
    FOHMinObjPosX(pEgoVEDData, pFOHOutput);

    /* Invalid bitfield output */
    FOHInvalidBitField(&sAccObjChk, &sObjTranControl, &sTgtObjPFOutput,
                       &sHistoryControl, &sModeTranControl, pFOHOutput);
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */