/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
/*! \file *********************************************************************

  COMPONENT:              VED (Vehicle Dynamics Observer)

  MODULENAME:             ved_.h

  @brief                  This file contains all types, definitions and
prototypes needed inside of this component


*****************************************************************************/

#ifndef VED_H_INCLUDED
#define VED_H_INCLUDED
/*** START OF SINGLE INCLUDE SECTION ****************************************/

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "ved_ext.h"

#ifndef GLOB_MEMSEC_H
#ifndef _ALGO_MEMSEC_H_
/* Empty macros for if memsec is not included */
#define SET_MEMSEC_VAR_A(v)
#define SET_MEMSEC_VAR(v)
#define SET_MEMSEC_CONST_A(v)
#define SET_MEMSEC_CONST(v)
#endif
#endif

#include "ved_par.h"
#include "ved_internal_types.h"
#include "tue_common_libs.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/* Minimum velocity for curvature estimation */
#define VED__CURVE_V_MIN ((float32)(3.0F / C_KMH_MS))

/* Filter time constant for quality of single curvatures */
#define VED__QUALITY_FT ((float32)1.0F)

/* Minimum and maximum output curvature as radius */
#define VED__MAX_RADIUS (float32)(30000.F)
#define VED__MIN_RADIUS (float32)(4.F)

/* Definitionen fuer Stillstandabgleich */
#define STANDST_STDABW_MAX                                          \
    (DEG2RAD(0.20F)) /* Maximale Standardabweichung fuer Uebernahme \
                        Stillstandabgleich */

#define WHS_SPEEED_RANGE_VOLUME \
    (sint32)4 /* Anzahl der Geschwindigkeitsbereiche des RDZ Offset */

#define SWA_OFFS_HIST_NO_BINS \
    (29uL) /* Number of histogram bins for steering wheel angle calibration */
#define SWA_OFFS_HIST_MAX_IDX \
    (SWA_OFFS_HIST_NO_BINS -  \
     1uL) /* Maximum bin index for steering wheel angle histogram */

#define AY_OFFS_HIST_NO_BINS                                           \
    (41uL) /* Number of histogram bins for lateral acceleration sensor \
              calibration */
#define AY_OFFS_HIST_MAX_IDX \
    (AY_OFFS_HIST_NO_BINS -  \
     1uL) /* Maximum bin index for lateral acceleration angle histograms */

/*! Start and final confidence during standstill calibration */
#define VED__YWR_OFF_STST_CALC_TIME_Q_MIN                                     \
    (float32)(0.6F) /*!< Minimum offset confidence for standstill calibration \
                       start */
#define VED__YWR_OFF_STST_CALC_TIME_Q_MAX                                     \
    (float32)(1.0F) /*!< Maximum offset confidence for standstill calibration \
                       end   */

/*! Time constraints for yaw rate offset calibration */
#define YWR_ECU_ELPSD_TIME_MAX        \
    (float32)(10.0F * 24.0F * 60.0F * \
              60.0F) /*!< Maximum ECU running time 10 days     */
#define YWR_OFFS_ELPSD_TIME_MAX \
    (float32)(60.0F * 60.0F) /*!< Maximum time since last calibration */

/* Number of velocity ranges for velocity correction */
#define VED__CORR_VEL_RANGES (3U)

/* Generated velo variance is calculate by this parabolic function v_var(a) =
 * m*a^2 +b */
/* Min Generated velo variance, b of var(a) */
#define VED__GEN_VELO_VAR_B (0.03030F)
/* m of parabolic function v_var(a) */
#define VED__GEN_VELO_VAR_M (0.06970F)

/* Use optimized functions dependent on IEEE754 float format */
#define VED__SQRT(x_) SQRT(x_)
#define VED__EXP(x_) VED_Exp(x_)

#if (((!defined(CFG_VED__DIS_YWR_OFFSET_COMP)) ||           \
      (!CFG_VED__DIS_YWR_OFFSET_COMP)) ||                   \
     ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
      (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)))
/* yawrate offset types for dynamic offset compensation in autocode model */
#define VED__YAWRATE_STATE_KEEP_TYPE (0U) /* do not take over use last type */
#define VED__YAWRATE_STATE_STANDSTILL \
    (1U)                            /* dynamic offset is stand still offset */
#define VED__YAWRATE_STATE_NVM (2U) /* dynamic offset taken from NVM*/
#define VED__YAWRATE_STATE_NOT_ESTIMATED \
    (3U) /* yaw rate stand still offset never ever estimated */

#define VED__YAWRATE_DYN_MAX_DIFF \
    (0.00395F) /* max difference between old dyn offset and new offset */
#endif

/*****************************************************************************
  MACROS
*****************************************************************************/

/* Simple low pass filter, first order  */
#define VED__IIR_FILTER(NEW_, OLD_, FT_, ST_) \
    (((((FT_) / (ST_)) * (OLD_)) + (NEW_)) / (1.0F + ((FT_) / (ST_))))

/* Simple low pass filter differentiator, first order  */
#define VED__IIR_DIFF(NIN_, OIN_, OOUT_, FT_, ST_) \
    ((1.F / ((ST_) + (FT_))) * (((NIN_) - (OIN_)) + ((FT_) * (OOUT_))))

/* IIR filter second order */
#define VED__IIR_FILTER2(IN_, OUT_, C_, M_)                                \
    do {                                                                   \
        (OUT_) =                                                           \
            (((C_)[0].num * (IN_)) + ((C_)[1].num * (M_)[0].fin) +         \
             ((C_)[2].num * (M_)[1].fin)) -                                \
            (((C_)[1].den * (M_)[0].fout) + ((C_)[2].den * (M_)[1].fout)); \
        (M_)[1].fin = (M_)[0].fin;                                         \
        (M_)[0].fin = (IN_);                                               \
        (M_)[1].fout = (M_)[0].fout;                                       \
        (M_)[0].fout = (OUT_);                                             \
    } while (0) /* (no trailing ; ) */

/* */
#define VED__CTRL_SET_STATE(mask_, val_) ((val_) |= (mask_))
#define VED__CTRL_CLR_STATE(mask_, val_) ((val_) &= ~(mask_))
#define VED__CTRL_GET_STATE(mask_, val_) ((boolean)((val_) & (mask_)) != FALSE)

#define VED_SET_IO_STATE_VAL(name_, val_) ((name_) = (val_))
#define VED_GET_IO_STATE_VAL(name_) (name_)

/*----- Data acquisition interface definition, only active if configured and new
 * MTS setup is used */

#define VED__PROF_START(fct_, u_IdLocal)
#define VED__PROF_STOP(fct_, u_IdLocal)
#define VED__PROF_MARKER(fct_, u_IdLocal)

/* macros for sequence */
#define VED__SEQUENCE_INIT_MODULE_INIT (1U)
#define VED__SEQUENCE_INIT_MODULE_NOT_INIT (0U)

#define VED__SET_SEQUENCE_STATE_TO_INIT(module) \
    ((module) = (uint32)VED__SEQUENCE_INIT_MODULE_INIT)
#define VED__RESET_SEQUENCE_STATES(state) ((state).states = 0U)
#define VED__SET_ALL_SEQUENCE_STATES_TO_INIT(InitStates) \
    ((InitStates).states = 0xFFFFFFFFU)
#define VED__IS_MODULE_SEQU_INITALIZED(module) \
    (((module) == (uint32)VED__SEQUENCE_INIT_MODULE_INIT) ? (TRUE) : (FALSE))

BML_INLINE uint32 BML_u_Round2UintGen(float32 x);
BML_INLINE uint32 BML_u_Round2UintGen(float32 x) {
    BML_ASSERT(x >= 0.f); /* PRQA S 3112 */
    return (uint32)(x + 0.5f);
}
#define ROUND_TO_UINT(x) BML_u_Round2UintGen(x)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/* bitfield definition to have commen definition in different projects and
 * setups */
typedef unsigned int ved__ubit32_t;

/*! bit field Configuration of the VED Component */
typedef struct {
    uint32 Version;
    ved__ubit32_t
        cfg_ved__yw_dyn_avg : 1; /*! Enable dynamic gyro offst compensation */
    ved__ubit32_t cfg_ved__ex_ywr_nvm : 1; /*! Enable offset storage  in
                                              nonvolatile Memory */
    ved__ubit32_t
        cfg_ved__int_gyro : 1; /*! Enable internal yaw rate sensor processing */
    ved__ubit32_t cfg_ved__fpm_754 : 1; /*! Enable optimized math functio
                                           approximation  */
    ved__ubit32_t
        cfg_ved__use_ex_long_accel : 1; /*! Use external provided longitudinal
                                           acceleration signal  */
    ved__ubit32_t
        cfg_ved__use_ex_long_velo : 1;    /*! Use external provided longitudinal
                                             velocity signal  */
    ved__ubit32_t cfg_ved__mot_state : 1; /*!  Enable motion state processing */
    ved__ubit32_t
        cfg_ved__do_velocity_corr : 1; /*!  Enables the velocity correction   */
    ved__ubit32_t vel_corr_aln : 1;
    ved__ubit32_t vel_corr_hist_stationary_targets : 1;
    ved__ubit32_t cfg_ved__use_correct_velo_corr_var : 1;
    ved__ubit32_t cfg_ved__rollbench_detection : 1; /*!  Enables the roll bench
                                                       detection   */
    ved__ubit32_t
        cfg_ved__use_ext_proc_curvature : 1; /*! Enables usage of external curve
                                                as ved_ output curve  */
    ved__ubit32_t
        cfg_ved__use_ext_proc_yaw_rate : 1; /*! Enables usage of external yaw
                                               rate as ved_ output yaw rate  */
    ved__ubit32_t
        cfg_ved__use_ext_proc_side_slip_angle : 1; /*! Enables usage of external
                                                      side slip angle as ved_
                                                      ouput side slip angle  */
    ved__ubit32_t
        cfg_ved__dis_swa_offset_comp : 1; /*! disable steering wheel sensor
                                             offset compensation*/
    ved__ubit32_t cfg_ved__dis_ywr_offset_comp : 1; /*! disable yaw rate sensor
                                                       offset compensation */
    ved__ubit32_t
        cfg_ved__dis_whs_offset_comp : 1; /*! disable wheel speed sensor offset
                                             compenpensation */
    ved__ubit32_t
        cfg_ved__dis_lat_offset_comp : 1; /*! disable lateral accleration sensor
                                             offset compensation */
    ved__ubit32_t
        cfg_ved__use_ext_proc_understeer_grad : 1; /*! Enables usage of external
                                                      understeer gradient as
                                                      ved_ input parameter  */
    ved__ubit32_t
        ved__use_learned_understeer_grad : 1; /*! If the learned understeer
                                                 gradiend should be used */
    ved__ubit32_t ved__use_est_wld_dep : 1; /*! If the estimated wheel laod dep
                                               should be used */
    ved__ubit32_t cfg_ved__use_velo_monitor : 1; /*! disable wheel speed sensor
                                                    pre processing */
    ved__ubit32_t cfg_ved__ywr_offset_monitor : 1; /*! disable wheel speed
                                                      sensor pre processing */
    ved__ubit32_t
        cfg_ved__dis_wheel_pre_processing : 1; /*! disable wheel speed sensor
                                                  pre processing */
    ved__ubit32_t
        cfg_ved__dis_yaw_sensor_pre_processing : 1; /*! disable yaw rate sensor
                                                       pre processing */
    ved__ubit32_t
        cfg_ved__dis_yaw_sensor_offs_pre_filtering : 1; /*! disable yaw rate
                                                           sensor offset pre
                                                           filtering */
    ved__ubit32_t cfg_ved__dis_yaw_sensor_output : 1;   /*! disable yaw rate
                                                           sensor output */
    ved__ubit32_t
        cfg_ved__dis_lat_accel_sensor_pre_processing : 1; /*! disable lateral
                                                             acceleration sensor
                                                             pre processing */
    ved__ubit32_t
        cfg_ved__dis_stw_angle_sensor_pre_processing : 1; /*! disable steering
                                                             wheel angle pre
                                                             processing */
    ved__ubit32_t
        cfg_ved__dis_side_slip_angle_estimation : 1; /*! disable the internal
                                                        side slip angle
                                                        estimation */
    ved__ubit32_t
        cfg_ved__gen_velocity_variance : 1; /*! Generated a velocity variance by
                                               deviation of the acceleration */
    ved__ubit32_t
        cfg_ved__alignment_offset_monitor : 1; /*! Do yaw rate offset monitoring
                                                  with alignment offset
                                                  estimation input */
    ved__ubit32_t cfg_ved__dis_functional_safety_mon : 1;
    ved__ubit32_t
        cfg_ved__calc_ved__timing : 1; /*! If the timing should be calculated */
    ved__ubit32_t cfg_ved__64bit_timestamp_interv : 1;
    ved__ubit32_t cfg_ved__dis_curve_output : 1;
    ved__ubit32_t cfg_ved__fs_velo_corr_mon : 1;
    ved__ubit32_t cfg_ved__mon_output_peaks : 1;
    ved__ubit32_t ved__profiling_enabled : 1;
    ved__ubit32_t cfg_ved__use_algocompstate : 1;
    ved__ubit32_t cfg_ved__set_dem_events : 1;
} VED_Config_t;

/*! Statistic measurement interval */
typedef struct {
    float32 Sum;    /*!< Sum of samples */
    float32 SqSum;  /*!< Square sum of samples */
    float32 Volume; /*!< Accumlated weights of samples */
    float32 Mean;   /*!< Mean of samples */
    float32 Dev;    /*!< Standard deviation of samples */
} VED_StatInterval_t;

/*! Statistic histogram database */
typedef struct {
    uint32 Size;         /*!< Number of bins       */
    float32 InvBinWidth; /*!< Inverse of bin width */
    float32 Mean;        /*!< Arithmetic mean      */
    float32 Dev;         /*!< Standard deviation   */
    float32 Sum;         /*!< Sum of bin volumes   */
    float32 *Range;      /*!< Bin ranges @NAME: PRange */
    float32 *Volume;     /*!< Bin volumes @NAME: PVolume */
} VED_Histogram_t;

/*! Curvature signal status */
typedef enum {
    VED__CRV_NOTOK,    /*!< Curvature outside the limits */
    VED__CRV_OK,       /*!< Curvature inside the limits */
    VED__CRV_DONT_KNOW /*!< Signal invalid or uncalibrated */
} eVED_CrvStatus_t;

typedef uint8 VED_CrvStatus_t; /*!< using 8bit type for the real enum (32bit
                                  type) eVED_CrvStatus_t to save memory*/

/*! Curvature direction status */
typedef enum {
    CRV_DIR_STRAIGHT, /*!< Curvature points straight ahead */
    CRV_DIR_LEFT,     /*!< Curvature points to the left side */
    CRV_DIR_RIGHT,    /*!< Curvature points to the right side */
    CRV_DIR_DONT_KNOW /*!< Curvature directions can not be determined */
} eVED_CrvDirStatus_t;

typedef uint8 VED_CrvDirStatus_t; /*!< using 8bit type for the real enum (32bit
                                     type) eVED_CrvDirStatus_t to save memory*/
/* IIR filter structure 2nd order */
typedef struct {
    float32 fin;  /*!< delayed input samples */
    float32 fout; /*!< delayed ouput samples */
} VED__IIR2Buff_t[2];

/* IIR Koeffizienten fuer 2.Ordnung */
typedef struct {
    float32 num; /* numerator coefficients */
    float32 den; /* denomintor coefficients first is assumed equal 1 */
} VED__IIR2Coeff_t[3];

/*! Longitudinal motion variables */
typedef struct {
    float32 VehVelocityCorr;
    float32 VelCorrFact;
    float32 VehVelo;
    float32 VehVeloVar;
    float32 VehAccel;
    float32 VehAccelVar;
    float32 FLWhlVelo;
    float32 FRWhlVelo;
    float32 RLWhlVelo;
    float32 RRWhlVelo;
    VEDMotionStateVehDyn_t MotState;
} VED_VelAccOut_t;

/*! Component internal intra module interface */
typedef struct {
    boolean FirstCycleDone; /*!< */
    float32 CycleTime;
    ved__swa_offset_t SwaOffset;
    ved__yaw_offset_t YwrOffset;
    ved__swa_offset_t AyOffset;
    VED_VelAccOut_t LongMot;
    VEDLatAccelVehDyn_t LatAcc;
    VEDCurveVehDyn_t Curve;
    VEDYawRateVehDyn_t YawRate;
    VEDSideSlipVehDyn_t SideSlipAngle;
} VED_ModIf_t;

/*! Some last valid Input Signals */
typedef struct {
    uint8 State[32];         /*!< io states of the last input signals         */
    float32 YawRate;         /*!< Yaw rate signal         */
    float32 StWheelAngle;    /*!< Steering wheel angle signal    */
    float32 LatAccel;        /*!< Lateral acceleration sensor signal        */
    float32 VehLongAccelExt; /*!< Longitudinal acceleration sensor signal */
    float32 VehVelocityExt;  /*!< Vehicle velocity signal  */
    float32 WhlVelFrLeft;  /*!< Wheel circumferential velocity front left    */
    float32 WhlVelFrRight; /*!< Wheel circumferential velocity front right   */
    float32 WhlVelReLeft;  /*!< Wheel circumferential velocity rear left    */
    float32 WhlVelReRight; /*!< Wheel circumferential velocity rear right   */
    float32 RearWhlAngle;  /*!< Rear Steering wheel angle signal    */
} VED_LastInSig_t;

#if !(defined(CFG_VED__DIS_FUNCTIONAL_SAFETY_MON) && \
      (CFG_VED__DIS_FUNCTIONAL_SAFETY_MON))
/*! Functional safety yaw rate check data */
typedef struct {
    float32 fYawWheelDelta;       /*!< gye-wye       */
    float32 fYawLatAccelDelta;    /*!< gye-aye    */
    float32 fYawSteerDelta;       /*!< gye-sye       */
    float32 fYawWheelOutDist;     /*!< fabs(gye-wye) > xx out of bounce driven
                                     distance       */
    float32 fYawLatAccelOutDist;  /*!< fabs(gye-aye) > xx out of bounce driven
                                     distance       */
    float32 fYawSteerOutDist;     /*!< fabs(gye-sye) > xx out of bounce driven
                                     distance       */
    uint16 nWheelErrorCounter;    /*!< total error counter    */
    uint16 nLatAccelErrorCounter; /*!< total error counter */
    uint16 nSwaErrorCounter;      /*!< total error counter      */
} VED_FSYawRateCheck_t;

/*! Functional safety curve check  data */
typedef struct {
    VED_StatInterval_t
        VED_FSWyeGyeStat; /*!< Statistic between the wheel yaw rate and the gier
                             yaw rate signale           */
    VED_StatInterval_t
        VED_FSAyeGyeStat; /*!< Statistic between the lateral accel yaw rate and
                             the gier yaw rate signale   */
    VED_StatInterval_t
        VED_FSSyeGyeStat; /*!< Statistic between the steering wheel yaw rate and
                             the gier yaw rate signale  */
    VED_StatInterval_t
        VED_FSAllOverIntervall; /*!< The mean of the three above */
    float32 fDeltaY; /*!< The actual Delta y for the yaw rate error from
                        VED_FSAllOverIntervall         */
    uint16 nCurveErrorCounter; /*!< total error counter */
    uint8 nDeltaYCounter; /*!< If the actual Delta y is above a threshold higher
                             the counter                */
} VED_FSCurveCheck_t;

/*! Functional safety fault states */
typedef struct {
    uint8 FSIntExtVeloCheck; /*<! Functional safety internal external veloc
                                plausibilty check error         */
    uint8 FSCorrVeloCheck; /*<! Functional safety corrected velocity plausibilty
                              check error              */
    uint8 FSMaxLatDisplacementError; /*<! Functional safety lateral displacement
                                        error check                        */
} VED_FSErrorStates_t;

/*! Functional safety internal data */
typedef struct {
    VED_FSYawRateCheck_t
        VED_FSYawRateCheck; /*!< Functional safety yaw rate check datas */
    VED_FSCurveCheck_t
        VED_FSCurveCheck; /*!< Functional safety curve check datas */
    VED_FSErrorStates_t
        FSMonErrorStates; /*!< The functional safety error states */
} VED_FSData_t;
#endif

typedef union {
    uint32 states; /*!< all states */
    struct {
        ved__ubit32_t ved__wpp_sequ_init : 1; /*!< 1 if the ved_ wpp module is
                                                 sequence initialized */
        ved__ubit32_t ved__ve_sequ_init : 1;  /*!< 1 if the ved_ ve module is
                                                 sequence initialized */
        ved__ubit32_t ved__wye_sequ_init : 1; /*!< 1 if the ved_ wye module is
                                                 sequence initialized */
        ved__ubit32_t ved__gye_sequ_init : 1; /*!< 1 if the ved_ gye module is
                                                 sequence initialized */
        ved__ubit32_t ved__aye_sequ_init : 1; /*!< 1 if the ved_ aye module is
                                                 sequence initialized */
        ved__ubit32_t ved__sye_sequ_init : 1; /*!< 1 if the ved_ sye module is
                                                 sequence initialized */
        ved__ubit32_t ved__ye_sequ_init : 1;  /*!< 1 if the ved_ ye module is
                                                 sequence initialized */
        ved__ubit32_t ved__sae_sequ_init : 1; /*!< 1 if the ved_ sae module is
                                                 sequence initialized */
        ved__ubit32_t
            ved__mot_state_sequ_init : 1; /*!< 1 if the ved_ mot state module is
                                             sequence initialized */
        ved__ubit32_t Pad : 23;
    } initStates;
} VED_SequenceInitStates_t;

typedef struct {
    float32
        StrgAngle; /*!< Current measured steering wheel angle @NAME: Angle */
    float32 StrgAngleOld; /*!< Steering wheel angle measured last cycle @NAME:
                             AngleOld */
    float32 Gradient;     /*!< Steering angle gradient non-filtered */
    float32
        StrgGradAbsOld;    /*!< Absolute value of swa gradient @NAME: GradAbs */
    float32 StrgDeltaDist; /*!< Driven distance with constant swa @NAME:
                              DeltaDist */
    boolean Valid;         /*!< Steering wheel angle signal validity */
} VED_SwaSenData_t;

typedef struct {
    float32 Offset; /*!< Steering wheel offset */
    sint32 State;   /*!< Steering wheel offset state @NAME: Status */
    float32 Dev;    /*!< Steering wheel offset standard deviation */
} VED_SwaOffset_t;

typedef struct {
    VED_SwaOffset_t
        EEPromStrgOffs;   /*!< NVM steering wheel angle offset @NAME: SwaOffs */
    boolean EEPromReadOk; /*!< NVM swa offset access state @NAME: ReadOK*/
    boolean
        EEPromWriteRequ; /*!< NVM swa offset access request @NAME: WriteReq */
} VED_SwaOffsEEprom_t;

/* Steering wheel angle offset calibration histogram bins */
typedef struct {
    float32 Range[SWA_OFFS_HIST_NO_BINS];  /*!< Range  */
    float32 Volume[SWA_OFFS_HIST_NO_BINS]; /*!< Counts */
} VED_SwaHistBin_t;

/* Steering wheel angle offset calibration */
typedef struct {
    VED_Histogram_t Hist; /*!< Histogram */
    VED_SwaHistBin_t Bin; /*!< Histogram bins */
    float32 Offs;         /*!< Estimated offset value @NAME: Offset */
    float32 ThrldDev;  /*!< Maximum deviation for new offset estimation @NAME:
                          ThrhdDev */
    float32 ThrldDist; /*!< Mininum distance for new offset estimation @NAME:
                          ThrhdDist */
    float32 Conf;      /*!< Confidence of new estimation @NAME: Confidence*/
} VED_SwaOffsEst_t;

typedef struct {
    boolean StrgOffsReadOk; /*!< Flag signaling correction read of swa offset
                               @NAME: ReadOK */
    float32 StrgOffset; /*!< Active steering wheel angle offset @NAME: Offset */
    float32 StrgOffsetNorm; /*!< Current steering wheel offset normalized @NAME:
                               OffsetNorm */
    sint32 OffsState; /*!< Calibration state steering wheel angle offset @NAME:
                         Status */
    float32 Dev;      /*!< Standard deviation                                 */
    uint32 OvrTakeCntr; /*!< Number of taken offset estimation during ignitions
                           cycle */
    float32
        WhlFrDiffFilt; /*!< Filtered wheel velocity differences front axle */
    float32 WhlReDiffFilt; /*!< Filtered wheel velocity differences rear axle
                              @NAME: WhlReDiffFlt*/
    float32 LatAccel;      /*!< Integrated lateral acceleration */
    boolean OffsInterimOk; /*!< Offset zwischengespeichert in der Hochlaufphase
                              @NAME: StatOffsIntOK */
    VED_SwaOffsEst_t Est;  /*!< Offset estimation section @NAME: SwaOffsEst */
    boolean ErrStrgOffsOutOfRange; /*!< Error indication swa offset out of range
                                      @NAME: ErrOffsRg*/
    uint8 ReInitCntr;              /*!< Repeat reinit counter of Histogramm */
} VED_SwaOffsData_t;

typedef struct {
    float32 f_WhlVelFrDiffFilt;
    float32 f_WhlVelReDiffFilt;
    float32 f_CurStOffset;
    float32 f_Sum;
    float32 f_Mean;
    float32 f_Offset;
    float32 f_OffsetSumLt;
    float32 f_OffsetMeanLt;
    uint16 u_Number;
    uint16 u_OffsetNumberLt;
} s_VED_FastSwaOffset_t;

typedef struct {
    uint32 Version;
    uint16 CycleCnt;  /*!< ved_ cycle counts                 */
    uint16 CycleTime; /*!< component cycle time             */
    uint8 CaliMode;   /*!< Calibration inhibition states    */
    uint8 CtrlMode;   /*!< Operating modes of ved_ component */
} VED_InputFrame_t;

/*! Input of the Vehicle Dynamics Module */
typedef struct {
    VED_InputFrame_t Frame;       /*!< Control frame                      */
    reqVEDParams_t Parameter;     /*!< Vehicle parameters @NAME: Para     */
    V1_7_VEDVehSigMain_t Signals; /*!< Vehicle sensor signals @NAME: Sig  */
} VED_InputData_t; /*!< @VADDR: 0x20010000 @VNAME: VED_In @ALLOW: ved__pub
                      @cycleid: ved__cycle_id  */

typedef struct {
    const VED_InputData_t *in;
    VED_ModIf_t *mif;
    uint8 *errOffsRg;
    uint8 *NVMerrOffsRg;
} VED_SwaIo_t;

/* Steering wheel angle processing database */
typedef struct {
    VED_SwaSenData_t Sensor;     /*!< Sensor signal processing @NAME: Sen */
    VED_SwaOffsData_t Offset;    /*!< Offset estimation @NAME: Offs       */
    VED_SwaOffsEEprom_t NVValue; /*!< Nonvolatile memory data @NAME: Nv   */
    s_VED_FastSwaOffset_t FastSwaOffset; /*!< Fast offset estimation data */
    VED_SwaIo_t Io; /*!< Input/Output interface              */
} VED_SwaData_t;

/*! Lateral acceleration sensor data */
typedef struct {
    float32 Ay;           /*!< Current lateral acceleration             */
    float32 AyOld;        /*!< Last cycle lateral acceleration          */
    float32 Gradient;     /*!< Raw acceleration gradient                */
    float32 AyGradAbsOld; /*!< Last cycle absolute gradient value       */
    boolean Valid;        /*!< Lateral Acceleration  valid (TRUE|FALSE) */
} VED_AySenData_t;

/* Lateral acceleration offset calibration histogram bins */
typedef struct {
    float32 Range[AY_OFFS_HIST_NO_BINS];  /*!< Range  */
    float32 Volume[AY_OFFS_HIST_NO_BINS]; /*!< Counts */
} VED_AyHistBin_t;

/* Lateral acceleration offset estimation */
typedef struct {
    VED_Histogram_t Hist; /*!< Histogram */
    VED_AyHistBin_t Bin;  /*!< Histogram bins */
    float32 Offs;         /*!< Estimated offset value */
    float32 ThrldDev;     /*!< Maximum deviation for new offset estimation  */
    float32 ThrldDist;    /*!< Mininum distance for new offset estimation   */
    float32 Conf;         /*!< Confidence of new estimation */
} VED_AyOffsEst_t;

/* Lateral acceleration offset calibration */
typedef struct {
    float32 AyOffset;   /*!< Current lateral acceleration sensor offset    */
    sint32 OffsState;   /*!< Calibration state lateral acceleration offset    */
    float32 Dev;        /*!< Standard deviation         */
    uint32 OvrTakeCntr; /*!< Number of taken offset estimation during ignitions
                           cycle */
    VED_AyOffsEst_t Est; /*!< Offset estimation section */
    boolean
        AyOffsReadOk; /*!< Singnalize that nonvolatile offset has been read */
    boolean Interims; /*!< Interims offst active */
} VED_AyOffsData_t;

typedef struct {
    const VED_InputData_t *in;
    VED_ModIf_t *mif;
    const VEDNvIoDatas_t *nv_read;
    VEDNvIoDatas_t *nv_write;
    uint8 *errOffsRg;
    uint8 *NVMerrOffsRg;
} VED_AyIo_t;

typedef struct {
    VED_AySenData_t Sensor;  /*!< Sensor signal processing          */
    VED_AyOffsData_t Offset; /*!< Offset estimation                 */
    VED_AyIo_t Io;           /*!< Input/Output interface            */
} VED_AyData_t;

/*! Yaw rate zero point offset types */
typedef enum {
    OFFS_NON = 0 /*!< No offset type present */
    ,
    OFFS_STANDST = 1 /*!< Offset acquired from standstill */
    ,
    OFFS_DYN_APPRX = 3 /*!< Offset merged standstill and temp table */
    ,
    OFFS_TEMPER_TABLE = 4 /*!< Offset acquired from temperature table */
#if (CFG_VED__EX_YWR_NVM)
    ,
    OFFS_STANDST_EEPROM = 5 /*!< Offset acquired from non-volailte memory */
#endif
    ,
    OFFS_DYN_AVG = 6 /*!< Offset estimated by averaging */
    ,
    OFFS_DYN_INTER = 7
} eVED_YwrOffsType_t;

typedef uint8 VED_YwrOffsType_t; /*!< using 8bit type for the real enum (32bit
                                    type) eVED_YwrOffsType_t to save memory*/

/*! Yaw rate standstill offset compensation */
typedef struct {
    float32 YawRateOffset; /*!< Current standstill offset @NAME: StOffset */
    float32 MaxQuality;    /*!< Maximum confidence reached at calibration @NAME:
                              StMaxQual */
    float32
        AdjustTime; /*!< Time elapsed since last calibration @NAME: AdjTime */
    VED_StatInterval_t
        SampleInterval_1; /*!< Sample gathering interval @NAME: SplInt_1 */
    VED_StatInterval_t
        SampleInterval_2; /*!< Sample delay  interval @NAME: SplInt_2 */
    VED_StatInterval_t
        SampleInterval; /*!< Sample offset evaluation interval @NAME: SplInt */
    boolean
        StandstillOK;   /*!< Standstill for offset calibration  @NAME: StStOK */
    boolean Observable; /*!< Observation of yaw rate offset possible */
} VED_YwrStandStillOffs_t;

typedef struct {
    float32 Offset1; /* Aktueller StillstandsOffset1 */
    uint8 Status1;   /* Anzahl Lernwerte entspricht einer Guete des
                        abgespeicherten Offsets1 */
    float32 Offset2; /* Aktueller StillstandsOffset2 */
    uint8 Status2;   /* Anzahl Lernwerte entspricht einer Guete des
                        abgespeicherten Offsets2 */
} VED_YwrEepromOffset_t;

typedef struct {
    VED_YwrEepromOffset_t YawRateOffset; /* Offset aus EEPROM */
    float32 TimeLastWrittenEepromOffset; /*!< Zeit seit letztem Stillstandoffset
                                            @NAME: TimeLstWr */
    float32 MaxQuality; /*!< Qualitaet des Offsets, nimmt mit zunehmender Anzahl
                           Messwerte zu @NAME: MaxQual */
    uint8 NumOfWrittenOffsets; /*!< Anzahl der in diesem Zuendungszyklus
                                  geschriebenen Offsets @NAME: NoWrOffs */
} VED_YwrStandStillEepromOffs_t;

/* Dynamischer Drehraten-Interims-Offset fuer Initialzustand */
typedef struct {
    float32 YawRateOffset;     /*!< Geschaetzter Offset @NAME: Offset */
    VED__IIR2Buff_t YwFiltInt; /* Filter Elemente */
    float32 MaxQuality;        /* Maximale Guete bei neuem Offset */
    float32 DistThrd;
} VED_YwrDynOffsInter_t;

/* Dynamischer Drehraten Abgleichdaten ueber gemittelte Gierrate */
typedef struct {
    float32 YawRateOffset; /*!< @NAME: DynOffset */
    float32
        MaxQuality;    /*!< Maximale Guete bei neuem Offset @NAME: DynMaxQual */
    float32 accYwDist; /* Akkumulierte Distanz bei Vorfilterung */
    float32 YwDist;    /* Wegstrecke die in Mittelung eingeht */
    float32 StDrv;     /* Wegstrecke fuer Lenkwinkel Streckenpruefung */
    float32 YwDrvDistThr;   /*!< Zaehler @NAME: DisThrd */
    VED__IIR2Buff_t YwFilt; /*!< @NAME: AvgFilt */
    uint32 cntCycle;        /* Zykluszaehler (Einschwingen FIR-Fitler) */
    float32 LastEcuTime;    /* Momentane ECU-Time */
    float32 DrvDistRed; /*!< Gefahrene Distanz zum reduzieren der Asymmertrie
                           @NAME: DrvDstRed */
    float32 YawRateDet; /*!< Detektionsfilter-Wert @NAME: DetOffs */
    VED__IIR2Buff_t YwFiltDet; /*!< @NAME: DetFilt */
    float32 QReduce;           /* Quality Reduction */
    boolean FlStwAStat; /*!< Steering Angle Offset Sufficient Quality  Flag
                           @NAME: FlStwStat */
    VED_YwrDynOffsInter_t IntOffs; /*!< @NAME: IntFilt */
} VED_YwrDynOffsAvg_t;

/* Zwischenspeicher fuer Offset gemittelten Offset aus verschiedenen dynamischen
 * OffsetTypen */
typedef struct {
    float32 YawRateOffset; /* Gemittelter Offset */
    float32 Quality;       /* Qualitaet des Offsets */
} VED_YwrDynOffsCache_t;

typedef struct {
    ved__yaw_offset_t OffsData; /*!< The actual offset */
    boolean IsDynamic; /*!< If it is an old dynamic offset (mean filter) */
} ToAutocode_t;

typedef struct {
    float32 YawRateOffset; /*!< Aktueller Offset @NAME: Offset */
    float32 Quality;       /*!< Aktuelle  Guete des Offsets */
    float32 MaxQuality;    /*!< Urspruengliche Guete bei Offsetberechnung
                              @NAME:MaxQual */
    float32 OffsElpsdTime; /*!< Zeit die seit dem letzten Abgleich in s @NAME:
                              OffsElpTime */
    float32 Temperature;   /* Temperatur beim Ermitteln des Offsets */
    VED_YwrStandStillOffs_t
        StandStillOffset; /*!< Daten fuer Stillstandsoffset @NAME: StandStill */
    VED_YwrStandStillEepromOffs_t
        StandStillEepromOffset;     /*!< Daten fuer EEPROM-Offset @NAME:
                                       StandStillNvm */
    VED_YwrDynOffsAvg_t DynOffsAvg; /*!< @NAME: Dynamic */
    float32 QualNoRed;              /*!< @NAME: QNoRed */
    VED_YwrDynOffsCache_t Cache;
    float32 EcuElpsdTime; /*!< Zeit seit ECU Start in s @NAME: ElpTime */
    boolean OffsCompOK;   /*!< Kompensation erfolgt */
    boolean TemperOK; /*!< Getemperte Kompensation erfolgt @NAME: TempCompOK*/
    VED_YwrOffsType_t OffsType; /*!< Alg. ueber den der akt Offset ermittlet
                                   wurde @NAME:Type */
    ToAutocode_t ToAutocode;    /*!< Interface to autocode */
} VED_YwrOffsData_t;

/* Noch nicht benoetigt */
/* Vorgesehen fuer internen Drehratensensor */
typedef enum { YWR_INTERN, YWR_EXTERN } eVED_YwrSensorType_t;

typedef uint8
    VED_YwrSensorType_t; /*!< using 8bit type for the real enum (32bit type)
                            eVED_YwrSensorType_t to save memory*/

/*! Yaw rate sensor data */
typedef struct {
    float32 YawRate;    /*!< Measured yaw rate */
    float32 YawRateOld; /*!< Yaw rate previous cycle @NAME: YawRtOld */
    float32
        YawRateCurveFilt1; /*!< Drehrate Kuemmung Filter @NAME: YawRtCrvFlt1 */
    float32
        YawRateCurveFilt2; /*!< Drehrate Kuemmung Filter @NAME: YawRtCrvFlt2 */
    float32
        YawRateCurveFilt3; /*!< Drehrate Kuemmung Filter @NAME: YawRtCrvFlt3 */
    float32 YwCurveOld;    /*!< Kruemmung letzter Zyklus fuer Gradientberechnung
                              @NAME: YawRtCrvOld*/
    float32 Gradient;      /* Drehraten Gradient ungefiltert */
    float32 GradientAbsOld;   /* Drehraten Gradien vorhergehender Zyklus */
    float32 DeltaDist;        /* Weg mit const Drehrate 0 ... 150 m */
    boolean Valid;            /* Gueltigkeitsbit fuer Drehrate */
    boolean YwFirstCycleDone; /*!< Zyklus nach nach Init durchgefuehrt @NAME:
                                 CycleDone */
    float32 FilterTime;       /*!< Filterzeit der Drehratenspur @NAME: FTime*/
} VED_YwrSenData_t;

typedef struct {
    const VED_InputData_t *in;
    VED_ModIf_t *mif;
    const VEDNvIoDatas_t *nv_read;
    VEDNvIoDatas_t *nv_write;
} VED_YwrIo_t;

/* Yaw rate sensor data processing */
typedef struct {
    VED_YwrSenData_t Sensor;      /*!< Yaw rate sensor data  @NAME: Sen */
    VED_YwrOffsData_t Offset;     /*!< Zero point offset @NAME: Offs */
    VEDCurveVehDyn_t CrsYwRt;     /*!< Drehraten Spur @NAME: Crv */
    VEDCurveVehDyn_t CrsYwRtFast; /*!< Schnelle Drehraten Spur @NAME: CrvFast */
    VED_YwrIo_t Io;
} VED_YwrData_t;

typedef struct {
    float32 YawRate;      /*!< Current yaw rate    */
    float32 YawRateOld;   /*!< Yaw rate last cycle */
    float32 Gradient;     /*!< Yaw rate gradient   */
    float32 Tempr;        /*!< Current yaw rate sensor temperature */
    float32 TemprOld;     /*!< Yaw rate sensor temperature last cycle */
    float32 TempGrad;     /*!< Yaw rate temperature gradient */
    boolean Valid;        /*!< Validity  of yaw rate signal */
    boolean StandstillOK; /*!< Vehicle standstill flag */
    boolean obsOffs;      /*!< Yaw rate offset during standstill observable */
    boolean YwFirstCycleDone; /*!< First non-init cycle completed */
} VED_YwrtSenData_t;

/*! Yaw rate offset table node */
typedef struct {
    float32 Offset; /*!< Learned offset minimum value         */
    float32 Conf;   /*!< Confidence of learned offset values  */
    uint8 No;       /*!< Node number (=index) of offset table */
} VED_YwrtLearnNode_t;

/*! Yaw rate table offset data */
typedef struct {
    VED_YwrtLearnNode_t
        Left; /*!< Nearest left (lower) table node at current temperature */
    VED_YwrtLearnNode_t
        Right; /*!< Nearest right (higher) table node at current temperature */
    VED_StatInterval_t SmpYwRt; /*!< Sample interval internal gyro yaw rate  */
    VED_StatInterval_t
        SmpTemp;     /*!< Sample interval internal gyro temperature */
    float32 Offset;  /*!< Current yaw rate offset based on correction table */
    float32 Conf;    /*!< Current confidence of yaw rate offset */
    uint16 LearnCnt; /*!< Learn progress counter during standstill */
    uint16 LearnAgainCnt; /*!< Relearn (max. confidence has been reached)
                             counter */
    float32 LastOffsTemp; /*!< Temperature of last offset learn iteration  */
    boolean bWrLeft;      /*!< Left table node write non-volatile memory flag */
    boolean bWrRight; /*!< Right table node write non-volatile memory flag */
    boolean Valid;    /*!< Validity for yaw rate offset */
    boolean Checked;  /*!< Flag to indicate that table interpolation values has
                         been checked */
} VED_YwrtOffsTblData_t;

/*! Yaw rate offset data */
typedef struct {
    VED_YwrtOffsTblData_t Tbl;    /*!< Offset temperature table */
    VED_YwrStandStillOffs_t StSt; /*!< Standstill offset data */
    VED_YwrOffsType_t OffsType;   /*!< Active final offset */
    float32 YawRateOffset;        /*!< Current yaw rate offset */
    float32 MaxQuality; /*!< Confidence of last standstill calibration */
    float32 Quality;    /*!< Current confidence of offset */
    float32
        OffsElpsdTime;   /*!< Time elapsed since last standstill calibration */
    float32 Temperature; /*!< Temperature during last standstill calibration */
    float32 confTbl;     /*!< Current confidence of table offset */
    float32 confStSt;    /*!< Current confidence of standstill offset */
    float32 YwRtTblOffsStSt; /*!< Table offset during standstill calibration */
    ToAutocode_t ToAutocode; /*!< Interface to autocode */
} VED_YwrtOffsData_t;

typedef struct {
    VED_ModIf_t *mif; /* internal interface between modules */
} VED_YwrtIo_t;

typedef struct {
    VED_YwrtSenData_t Sensor;  /* module internal yawrate data */
    VED_YwrtOffsData_t Offset; /* offset Data                */
    VED_YwrtIo_t Io;
} VED_YwrtData_t;

/*! Yaw rate offset monitoring database during vehicle halt */
typedef struct {
    float32 timeAboveThrhd; /*!< Elapsed time with yaw rate above allowed
                               thershold */
} VED_YwrMonVehHalt_t;

/*! Yaw rate offset monitoring database before vehicle drive-off */
typedef struct {
    float32 timeAboveThrhd; /*!< Elapsed time with yaw rate above allowed
                               thershold */
} VED_YwrMonVehDriveOff_t;

/*! Yaw rate offset monitoring state after vehicle-halt and before drive-off */
typedef enum {
    VED__YWR_MON_STATE_HALTING =
        0UL, /*!< Acquire data for offset after vehicle halt */
    VED__YWR_MON_STATE_STANDING =
        1UL, /*!< Acquire data for first offset during vehicle halt */
    VED__YWR_MON_STATE_WAITING =
        2UL /*!< Acquire data for future vehicle drive-off */
} eVED_YwrMonVehHaltDrvState_t;

typedef uint8
    VED_YwrMonVehHaltDrvState_t; /*!< using 8bit type for the real enum (32bit
                                    type) eVED_YwrMonVehHaltDrvState_t to save
                                    memory*/

/*! Yaw rate offset monitoring database after vehicle-halt and before drive-off
 */
typedef struct {
    VED_StatInterval_t ivSample; /*!< Sample window to get offset values */
    VED_YwrMonVehHaltDrvState_t State; /*!< Monitor state */
    float32 ywrStStOn;                 /*!< Yaw rate offset after standstill */
    float32 ywrStStOff;                /*!< Yaw rate offset befor drive-off */
} VED_YwrMonVehHaltDrv_t;

/*! Yaw rate offset calibration monitoring state during vehicle halt */
typedef uint8 VED_YwrMonVehHaltCalState_t;

#define VED__YWR_MON_CAL_STATE_MUTE ((VED_YwrMonVehHaltCalState_t)0UL)
#define VED__YWR_MON_CAL_STATE_ARMED ((VED_YwrMonVehHaltCalState_t)1UL)

/*! Yaw rate offset calibration monitoring database during vehicle halt */
typedef struct {
    VED_StatInterval_t ivSample; /*!< Sample window to get offset values */
    float32 timeStStToCalib;     /*!< Time elpased since standstill to first
                                    calibration */
    uint8 cntCalCycle;
    VED_YwrMonVehHaltCalState_t State;
} VED_YwrMonVehHaltCal_t;

/*! Yaw rate offset monitoring database */
typedef struct {
    float32 timeVehHalt;         /*!< Elapsed time since standstill start */
    float32 ywRateOld;           /*!< Yaw rate value from last cycle */
    float32 Gradient;            /*!< Yaw rate gradient */
    VED_YwrMonVehHalt_t VehHalt; /*!< Monitor during vehicle-halt */
    VED_YwrMonVehDriveOff_t
        VehDriveOff; /*!< Monitor before vehicle-drive-off */
    VED_YwrMonVehHaltDrv_t
        VehHaltDrv; /*!< Monitor at standstill start and end */
    VED_YwrMonVehHaltCal_t
        VehHaltCal; /*!< Monitor calibration during vehicle-halt  */
} VED_YwrMonData_t;

/*! Wheel speed axle raw data */
typedef struct {
    float32 WspLeft;       /*!< Wheel velocity left @NAME: Left */
    float32 WspRight;      /*!< Wheel velocity right @NAME: Right */
    boolean WspValid;      /*!< Wheel velocity on axle valid @NAME: Valid */
    float32 Ratio;         /*!< Wheel velocity ration left / right */
    float32 WspLeftFilt1;  /*!< Wheel velocity left filter stage 1 @NAME:
                              LeftFilt1 */
    float32 WspRightFilt1; /*!< Wheel velocity right filter stage 1 @NAME:
                              RightFilt1 */
    float32 WspLeftFilt2;  /*!< Wheel velocity left filter stage 2 @NAME:
                              LeftFilt2  */
    float32 WspRightFilt2; /*!< Wheel velocity left filter stage 2 @NAME:
                              RightFilt2 */
} VED_WhsAxisData_t;

/*! Wheel velocities left and right per axle */
typedef struct {
    float32 WspLeft;  /*!< Wheel velocity left @NAME: Left */
    float32 WspRight; /*!< Wheel velocity right @NAME: Right */
} VED_WhsSpeedsAxle_t;

/*! Wheel speed sensor data */
typedef struct {
    VED_WhsSpeedsAxle_t
        WspFrontRaw; /*!< Wheel speed front axle raw @NAME: FrontRaw */
    VED_WhsSpeedsAxle_t
        WspRearRaw;             /*!< Wheel speed rear axle raw @NAME: RearRaw */
    float32 WspRawRL;           /*!< Wheel speed raw rear left @NAME: RawRL */
    float32 WspRawRR;           /*!< Wheel speed raw rear right @NAME: RawRR */
    VED_WhsAxisData_t WspFornt; /*!< Wheel speed front axle @NAME: Front */
    VED_WhsAxisData_t WspRear;  /*!< Wheel speed rear axle @NAME: Rear */
    float32 CurveOld; /*!< Ungefilterte Kruemmung letzter Zyklus @NAME: CrvOld*/
    float32 VehSpeedOld;     /*!< Ego velodity of last cycle */
    float32 VehAcceleration; /*!< Vehicle acceleration @NAME: VehAccel*/
    boolean Aquaplaning;     /*!< Hydroplaning detected @NAME: Hydroplan */
} VED_WhsSenData_t;

/*! Wheel velocity offset axle data */
typedef struct {
    VED_StatInterval_t
        Interval[WHS_SPEEED_RANGE_VOLUME]; /*!< Averaging velocity ranges @NAME:
                                              Intv*/
    VED_StatInterval_t SampleInterval; /*!< Sample interval  @NAME: IntvSpl */
    float32 SampleIntFiltMean; /*!< Mean filtered mean of sample Intervall */
    float32 Factor; /*!< Tune factor, to rise sample interval weight */
    sint32
        IntervalRangeVolume; /*!< Reference to velocity range @NAME: Volume */
} VED_WhsAxleOffs_t;

/*! Wheel velocity offset data */
typedef struct {
    VED_WhsAxleOffs_t OffsFront; /*!< Offset front axle @NAME: Front */
    VED_WhsAxleOffs_t OffsRear;  /*!< Offset rear axle @NAME: Rear */
    VED_CrvStatus_t ExWspStatus; /*!< Gesammtspur ohne RDZ Geradeausfahrt
                                    (OK|NOTOK|DONT_KNOW) */
    float32 WspDeltaDist;  /*!< Weg mit const RDZ-Kruemmung @NAME: DeltaDist */
    float32 GradAbsOld;    /*!< Peak filtered absolute value of gradient */
    sint32 SpeedRange;     /*!< Velocity range */
    sint32 LastSpeedRange; /*!< Velocity range last cycle */
} VED_WhsOffsData_t;

typedef struct {
    const VED_InputData_t *in;
    VED_ModIf_t *mif;
} VED_WhsIo_t;

/* Wheel speed processing database */
typedef struct {
    VED_WhsSenData_t Sensor;  /*!< Wheel speed sensor data @NAME: Sen */
    VED_WhsOffsData_t Offset; /*!< Wheel speed offset data @NAME: Offs */
    VED_WhsIo_t Io;
} VED_WhsData_t;

typedef struct {
    float32 Yw;
    float32 Wsp;
    float32 Strg;
    float32 StrgLog;
} VED_CrvWghtStd_t;

typedef struct {
    float32 Yw;
    float32 Strg;
} VED_CrvWghtFast_t;

typedef struct {
    float32 CurveStandardOld; /* Gesamtkruemmung letzter Zyklus Standard-Spur */
    float32 CurveFastOld; /* Gesamtkruemmung letzter Zyklus Schnelle-Spur     */
    float32 CurveGradQuality;     /* Spur Gradient Guete */
    VED_CrvWghtStd_t Weight;      /* Gewichte fuer Standard-Spur  */
    VED_CrvWghtFast_t WeightFast; /* Gewichte fuer Schnelle-Spur */
    float32 VehSpeedOld; /* Eigengeschwindigkeit letzter Zyklus               */
    float32 VehAcceleration; /* Fzg. Beschleunigung */
    float32 WspWeightFact;   /* Situationsabhaengiger Gewichtungsfaktor WSP   */
    boolean ErrCanNotCalcTrack; /* Status Fehler
                                   "ERR_LP_TRACKMERGE_CANT_CALC_TRACK" */
} VED_CrvMergeData_t;

typedef struct {
    const VED_InputData_t *in;
    VED_ModIf_t *mif;
    uint8 *errNotCalc;
} VED_CrvIo_t;

#if ((defined(CFG_VED__DO_VELOCITY_CORR)) && (CFG_VED__DO_VELOCITY_CORR))
typedef enum {
    VEL_CORR_INIT = 0,
    VEL_CORR_STARTUP = 1,
    VEL_CORR_BACKUP = 2,
    VEL_CORR_READY = 3
} eVED_CorrState_t;

typedef uint8 VED_CorrState_t; /*!< using 8bit type for the real enum (32bit
                                  type) eVED_CorrState_t to save memory*/

typedef struct {
    float32 binVelRatio[REF_SPEED_NO_BINS]; /*!< Occurences of velocity ratios
                                               within bounds      */
    uint16
        binVelRatOutLo; /*!< Occurrences of velocity ratios below lower bound */
    uint16
        binVelRatOutHi; /*!< Occurrences of velocity ratios below upper bound */
    uint8 cntSample;    /*!< Sample counter of input value                    */
    float32 Median;     /*!< Arithmetic mean                                  */
    float32 Dev;        /*!< Standard deviation                               */
    float32 Sum;        /*!< Sum of bin volumes                               */
    VED_StatInterval_t EgoVel; /*!< Referenced, uncorrected ego velocity */
} VED_RefSpeed_t;

typedef struct {
    float32 CorrFact;
    float32 CorrDev;
    float32 CorrVel;
    VED_CorrState_t State;
    float32 LastNvmWrt;
} VED_VelCorrNode_t;

typedef struct {
    uint32 Timer;       /*!< Timer for non-observable ego velocity  */
    float32 minVelMeas; /*!< Minimum reference velocity for measured
                           distributions */
    float32 maxVelMeas; /*!< Maximum reference velocity for measured
                           distributions */
    float32 lastEgoVel; /*!< Ego velocity from previous cycle */
    float32 EcuTime;    /*!< Runtime since ignition start */
    uint32 cntMeasAmb; /*!< Counter for measured ambigous velocity distributions
                        */
    uint32 cntMeasRng; /*!< Counter for measured velocity outside permitted
                          range */
} VED_VelCorrAux_t;

typedef struct {
    float32 P[4]; /*!< Error covariance matrix */
    float32 H[2]; /*!< Observation matrix      */
    float32 X[2]; /*!< State matrix            */
} VED_VelCorrEst_t;

typedef struct {
    VED_ModIf_t *mif;
    uint8 *errFactRg;
    uint8 *errWin;
} VedVelCorrIo_t;

typedef struct {
    VED_RefSpeed_t Hist[VED__CORR_VEL_RANGES];
    VED_VelCorrNode_t Node[VED__CORR_VEL_RANGES];
    VED_VelCorrEst_t Est;
    VED_VelCorrAux_t Aux;
    VedVelCorrIo_t Io;
} VED_VelCorr_t;

/* FS monitoring of velocity based on ALN calculated velocity of stationary
 * targets */
typedef struct {
    float32 f_velDiff;     /* difference of input velocities */
    float32 f_varRange;    /* max. tolerance based on variances */
    float32 f_threshold;   /* threshold for accepting external velocity */
    float32 f_curVelCorr;  /* velocity correction factor of current sample */
    float32 f_VelConfDiff; /* velocity confirmation check difference of current
                              sample */
    uint16 u_counter;      /* number of failed checks */
    uint16 u_ConfCounter;  /* number of failed checks of confirmation check*/
    uint8 fault;           /* fault state */
    uint8 confFault;       /* fault state of confirmation check */
} VED_FSVelCorrMon_t;

#endif

typedef struct {
    uint32 cntOutSide;
} VED_VelMon_t;

/*! Yaw rate offset types */
typedef enum {
    OUT_YWR_OFFS_NON = 0 /*!< No offset type present                   */
    ,
    OUT_YWR_OFFS_STANDST_SHORT = 1 /*!< Offset short acquired from standstill */
    ,
    OUT_YWR_OFFS_STANDST_FULL = 2 /*!< Offset full acquired from standstill */
    ,
    OUT_YWR_OFFS_STANDST_EEPROM =
        3 /*!< Offset acquired from non-volailte memory */
    ,
    OUT_YWR_OFFS_DYN = 4 /*!< Dynamic estimated yaw rate offset        */
} eVED_OutYwrOffsType_t;

typedef uint8
    VED_OutYwrOffsType_t; /*!< using 8bit type for the real enum (32bit type)
                             eVED_CorrState_t to save memory*/

/*! Estimated curvature of vehicle trajectory */
typedef struct {
    float32 Curve;    /*!< Course curvature in 1/Radius, 1/R > 0 -> left turn */
    float32 Variance; /*!< Variance of estimation */
    float32 Gradient; /*!< Course curvature gradient d/dt (1/R) */
} VED_OutCurve_t;

typedef struct {
    VEDSignalHeader_t sSigHeader;
    VEDSignalHeader_t BSW_s_VED_CtrlData;
    // VEDSignalHeader_t VehPar;
    VEDSignalHeader_t VehSig;
    VEDSignalHeader_t VED_NVMRead;
#if ((defined(CFG_VED__DO_VELOCITY_CORR)) && (CFG_VED__DO_VELOCITY_CORR))

#if ((defined(VEL_CORR_ALN)) && (VEL_CORR_ALN))
    VEDSignalHeader_t ALN_Monitoring; /* Velocity information from ALN    */
#endif
#endif

} VED__SM_t_SyncRef;

typedef struct {
    VEDNvSlfStGradCalc_t
        SlfstGrad;      /*!< Understeer / Oversteer gradient estimation */
    VEDNvWldCalc_t Wld; /*!< Wheel load dependency  */
    uint32 State;       /*!< Read status respective write request */

} VED_NvAutoIoData_t;

/*! Non Volatile Data Interface of the Vehicle Dynamics Module */
typedef struct {
    VED_NvAutoIoData_t
        Read; /*!< Non volatile memory read interface @NAME: In */
    VED_NvAutoIoData_t
        Write;  /*!< Non volatile memory write interface @NAME: Out */
} VED_NvData_t; /*!< @VADDR: 0x20012300 @VNAME: VED_NvDataAuto @ALLOW: ved__pub
                   @cycleid: ved__cycle_id*/

/*****************************************************************************
  GLOBAL VARIABLES (COMPONENT INTERNAL SCOPE)
*****************************************************************************/
extern VED_InternalData_t
    ved__internal_data; /*!< VNAME: VED_IntData VADDR: 0x2001E000 */
extern ved__bayes_mot_states_t
    ved__bayes_mot_states; /*!< VNAME: VED_BayesOut VADDR: 0x2001E800 */

/*****************************************************************************
  FUNCTION PROTOTYPES (COMPONENT INTERNAL SCOPE)
*****************************************************************************/

extern void VED_YwrExec(const reqVEDPrtList_t *reqPorts,
                        const VED_InputData_t *input,
                        VED_ModIf_t *mif,
                        const proVEDPrtList_t *proPorts);
extern void VED_AySwaExec(const reqVEDPrtList_t *reqPorts,
                          const VED_InputData_t *input,
                          VED_ModIf_t *mif,
                          const proVEDPrtList_t *proPorts);
extern void VED_WhsExec(const VED_InputData_t *input, VED_ModIf_t *mif);
extern void VED_VelCorrExec(const reqVEDPrtList_t *reqPorts,
                            const VED_InputData_t *input,
                            VED_ModIf_t *mif,
                            const proVEDPrtList_t *proPorts,
                            boolean b_RTBDetection);

extern void VED_YwrInit(const reqVEDPrtList_t *reqPorts,
                        const proVEDPrtList_t *proPorts);
extern void VED_AySwaInit(const reqVEDPrtList_t *reqPorts,
                          const proVEDPrtList_t *proPorts);
extern void VED_WhsInit(void);
extern void VED_VelCorrInit(const proVEDPrtList_t *proPorts);

/* Functional safety monitoring functions */
#if !(defined(CFG_VED__DIS_FUNCTIONAL_SAFETY_MON) && \
      (CFG_VED__DIS_FUNCTIONAL_SAFETY_MON))
extern void VED_FSMonitor(const VED_InternalData_t *IntData,
                          const VED_ModIf_t *VED_ModIf,
                          const proVEDPrtList_t *proPorts);
extern void VED_FSInit(void);
#endif

extern boolean VED_IsFirstCycleDone(void);
extern float32 VED_GetCycleTime(void);

extern void VED_HistInit(VED_Histogram_t *hist,
                         float32 *range,
                         float32 *volume,
                         uint32 size,
                         float32 minValue,
                         float32 maxValue);
extern void VED_HistAdd(VED_Histogram_t *hist, float32 value, float32 weight);
extern uint32 VED_HistGetMaxBin(const VED_Histogram_t *hist);
extern void VED_HistCalcMeanDev(VED_Histogram_t *hist);
extern float32 VED_HistCalcSum(const VED_Histogram_t *hist);
extern float32 VED_HistCalcMeanCenter(const VED_Histogram_t *hist,
                                      uint32 idxCenter,
                                      uint32 width,
                                      uint32 pwr);
extern void VED_HistReInit(VED_Histogram_t *hist);
extern void VED_HistReduce(VED_Histogram_t *hist, float32 weight);
extern float32 VED_HistCalcMedian(const VED_Histogram_t *hist);
extern float32 VED_HistGetVolume(const VED_Histogram_t *hist, float32 value);

#if (CFG_VED__FPM_754)
extern float32 VED_Sqrt(float32 x);
extern float32 VED_InvSqrt(float32 x);
extern float32 VED_Exp(float32 x);
#endif

extern void VED_StatIntervalInit(VED_StatInterval_t *StatInterval);
extern void VED_StatIntervalInitInput(VED_StatInterval_t *StatInterval);
extern void VED_StatIntervalAdd(VED_StatInterval_t *StatInterval,
                                float32 Value,
                                float32 Weight);
extern void VED_StatIntervalReduce(VED_StatInterval_t *StatInterval,
                                   float32 Factor);
extern void VED_StatIntervalMerge(VED_StatInterval_t *StatInterval_1,
                                  const VED_StatInterval_t *StatInterval_2);
extern void VED_StatIntervalMeanDev(VED_StatInterval_t *StatInterval);
extern void VED_StatIntervalPrealloc(VED_StatInterval_t *StatInterval,
                                     float32 Mean,
                                     float32 Volume,
                                     float32 Dev);

extern void VED_HpSort(uint32 no, float32 ra[]);
extern void VED_HpSortInd(uint32 no, const float32 ra[], uint32 idx[]);
extern void VED_HpSortIndU16(uint32 no, const uint16 ra[], uint8 idx[]);
extern void VED_HpSortIndF32(uint32 no, const float32 ra[], uint8 idx[]);
extern float32 VED_Discretize(float32 value, float32 interval);

extern float32 VED_LFunction(float32 Input,
                             float32 InputMin,
                             float32 InputMax,
                             float32 OutputMin,
                             float32 OutputMax);
extern float32 VED_CalcGradient(float32 NewValue, float32 OldValue);
extern VED_CrvStatus_t VED_CheckCurve(const VED_OutCurve_t *Course,
                                      float32 MaxCurve,
                                      float32 MinQuality);
extern VED_CrvDirStatus_t VED_GetCurveDir(const VED_OutCurve_t *Course,
                                          float32 ThrdCurve,
                                          float32 MinQuality);
extern void VED_InitCurve(VED_OutCurve_t *Course);
extern void VED_CalcDistStblGrad(float32 GradMax,
                                 float32 Grad,
                                 float32 *GradAbsOld,
                                 float32 *DeltaDist,
                                 float32 VehSpeed);
extern float32 VED_CalcCycleDistance(float32 VehSpeed);
extern float32 VED_FilterCycleTime(float32 New, float32 Old, float32 TimeConst);
extern void VED_FormatCurve(float32 *Curve);
extern void VED_CurveToRadius(float32 Curve, float32 *Radius);
extern float32 VED_LinInterp1(const float32 *xn,
                              const float32 *yn,
                              const uint32 num,
                              float32 xi);
extern float32 VED_FilterOffset(float32 newin, float32 oldflt, float32 fconst);
extern float32 VED_DifferentiateCycleTime(float32 newIn,
                                          float32 oldIn,
                                          float32 oldOut,
                                          float32 TimeConst);

/*! Prototpyes of steering wheel angle */
extern boolean VED_SwaIsValid(void);
extern const VED_OutCurve_t *VED_SwaGetCurveADTR(void);
extern const VED_OutCurve_t *VED_SwaGetCurve(void);
extern const VED_OutCurve_t *VED_SwaGetCurveLog(void);
extern void VED_CalcFastSwaOffset(const VED_InputData_t *p_InputData);

/*! Prototpyes of yaw rate sensor processing  */
extern void VED_YwrEstStandstillOffset(float32 YwRate,
                                       boolean valYwr,
                                       VED_YwrStandStillOffs_t *pStStOff);
extern void VED_YwrInitStandStillOffset(VED_YwrStandStillOffs_t *StandStOffs);
extern boolean VED_YwrIsValid(void);
extern float32 VED_YwrGetDynReduced(void);
extern void VED_YwrGetYawRate(VEDYawRateVehDyn_t *yawrate);

/*! Prototpyes of wheel speed sensor processing  */
extern boolean VED_WhsIsValid(void);
extern boolean VED_WhsGetWspAquaplaning(void);
extern boolean VED_SwaCheckOffsetGoodEnough(void);

/*! Prototpyes of lateral acceleration sensor processing  */

/*! Prototpyes of curve estimation  */
extern float32 VED_CrvGradQuality(void);
extern const VED_OutCurve_t *VED_CrvGetStandard(void);
extern const VED_OutCurve_t *VED_CrvGetExWsp(void);
extern const VED_OutCurve_t *VED_CrvGetFast(void);

/*! Prototpyes of velocity and acceleration estimation  */
extern float32 VED_VelAccGetLongAcc(void);

/* Prototypes for mat probabilty tool box*/
extern uint8 VED__MAT_PROB_BAYES2(uint8 ProbabilityA,
                                  uint8 ProbabilityB,
                                  const uint8 CPT[4]);
extern uint8 VED__MAT_PROB_BAYES3(uint8 ProbabilityA,
                                  uint8 ProbabilityB,
                                  uint8 ProbabilityC,
                                  const uint8 CPT[8]);
extern uint8 VED__MAT_PROB_BAYES4(uint8 ProbabilityA,
                                  uint8 ProbabilityB,
                                  uint8 ProbabilityC,
                                  uint8 ProbabilityD,
                                  const uint8 CPT[16]);

extern const VED_SwaOffsData_t *VED_SwaGetOffsData(void);
extern const VED_YwrOffsData_t *VED_YwrGetOffsData(void);
extern const VED_WhsOffsData_t *VED_WhsGetOffsData(void);
extern ved__whs_offset_t VED_WhsOffset(void);
extern const VED_AyOffsData_t *VED_AyGetOffsData(void);

extern const VED_YwrtOffsData_t *VED_YwrtGetOffsData(void);
extern void VED_YwrtGetYawRateOffset(float32 *offset, float32 *conf);

/* Prototpyes of Monitoring */
extern void VED_InitMon(void);

extern void VED_InitForCheckSignalPeakErrors(const VED_InputData_t *pInput);
extern void VED_MonitorInput(VED_InputData_t *input, VED_Errors_t *VED_Errors);

extern void VED_MonitorOutput(const VED_InputData_t *input,
                              const proVEDPrtList_t *proPorts,
                              const reqVEDPrtList_t *reqPorts);

extern void VED_MonitorDynYwrOffset(const ToAutocode_t *Input,
                                    VED_InternalData_t *IntData);

extern void VED_VelMonExec(const VED_InputData_t *input,
                           const ved__ve_out_t *ve,
                           VED_Errors_t *pErr);
extern void VED_VelMonInit(VED_Errors_t *pErr);

extern VED_VelCorr_t *VED_VelCorrGetPrivateData(void);
extern VED_FSData_t *VED_FSMonGetPrivateData(void);
extern VED_FSVelCorrMon_t *VED_FSVelCorrMonGetPrivateData(void);
extern VED_YwrData_t *VED_YwrGetPrivateData(void);
extern VED_WhsData_t *VED_WhsGetPrivateData(void);

extern VED_SwaData_t *VED_SwaGetPrivateData(void);
extern VED_AyData_t *VED_AyGetPrivateData(void);
#ifdef __cplusplus
};
#endif

/*** END OF SINLGE INCLUDE SECTION ******************************************/
#endif
