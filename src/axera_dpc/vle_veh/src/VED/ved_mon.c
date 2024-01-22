/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "ved_consts.h"
#define VED__DACQ_IF 1L /* Activate interface to internal data */
#include "ved.h"

/* Use for NaN check */
#if ((!defined(__MATH_H)) && (defined(__M32R__)))
#include <math.h>
#define ISNAN(_x) isnan(_x)
#else
#if (!defined(__MATH_H))
#define ISNAN(_x) FALSE
#else
#define ISNAN(_x) isnan(_x)
#endif
#endif

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/
#if !(defined(CFG_VED__DIS_FUNCTIONAL_SAFETY_MON) && \
      (CFG_VED__DIS_FUNCTIONAL_SAFETY_MON))

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
/* if the yaw rates are above this threshold the distance is exeeded */
#define VED__FS_WHEEL_YAW_RATE_THRESHOLD (0.04F)
/* threshold velocities, above yaw rate error monitoring is active */
#define MIN_WHEEL_YAW_RATE_VELOCITY (15.0F)
#define MAX_WHEEL_YAW_RATE_VARIANCE (99.0F)
#endif

#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
/* if the yaw rates are above this threshold the distance is exeeded */
#define VED__FS_LAT_ACCEL_YAW_RATE_THRESHOLD (0.01F)
/* threshold velocities, above yaw rate error monitoring is active */
#define MIN_LAT_ACCEL_YAW_RATE_VELOCITY (10.0F)
#endif

#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
/* if the yaw rates are above this threshold the distance is exeeded */
#define VED__FS_SWA_YAW_RATE_THRESHOLD (0.045F)
/* threshold velocities, above yaw rate error monitoring is active */
#define MIN_SWA_YAW_RATE_VELOCITY (10.0F)
#endif

/* after this distance the yaw rate out of bounce error is released */
#define VED__FS_YAW_RATE_DIST_THRESHOLD (150.0F)

/* curve threshold, only below this the yaw rate is monitored */
#define MAX_CURVE_GRAD (0.0005F)
/* the distance after that the mean of the deltas is calculated */
#define VED__FS_MEAN_DRIVE_DISTANCE (50.0F)
/* reduce the mean after the driven distance with factor */
#define VED__FS_DISTANCE_REDUCE_FACTOR (0.5F)

/* Plausibility check of course prediction parameters */
#define VED__FS_PRED_HORZ_TIME (3.0F)
#define VED__FS_MAX_LAT_DIST_ERROR (1.5F)
#define VED__FS_MAX_LAT_DIST_COUNTER (15U)

/* Weights of wheel curve, lataccel curve and steering curve for course
 * prediction monitoring */
#define VED__WYE_CURVECHECK_WEIGHT (3.0F)
#define VED__AYE_CURVECHECK_WEIGHT (4.0F)
#define VED__SYE_CURVECHECK_WEIGHT (4.0F)

#endif

/* signal ranges */
#ifndef MAX_VELOCITY
#define MAX_VELOCITY (115.0F) /* highest possible output velocity in m/s */
#endif
#ifndef MAX_ACCELERATION
#define MAX_ACCELERATION                                                     \
    (15.0F) /* highest possible longitudinal and lateral output acceleration \
               in m/s2*/
#endif
#ifndef MAX_YAWRATE
#define MAX_YAWRATE \
    (BML_Deg2Rad(150.0F)) /* highest possible output yaw rate in deg/s */
#endif
#ifndef MAX_STEERING_ANGLE
#define MAX_STEERING_ANGLE                                                     \
    (BML_Deg2Rad(90.0F)) /* highest possible steering angle of the road wheels \
                            in deg */
#endif

/* parameter ranges: minimum and maximum allowed range, default value if outside
 */
/* vehicle (curb) weight */
#ifndef MINIMUM_VEH_WEIGHT
#define MINIMUM_VEH_WEIGHT (500.0F)
#endif
#ifndef MAXIMUM_VEH_WEIGHT
#define MAXIMUM_VEH_WEIGHT (7500.0F)
#endif
#ifndef DEFAULT_VEH_WEIGHT
#define DEFAULT_VEH_WEIGHT (1800.0F)
#endif

/* track width front and rear axle */
#ifndef MINIMUM_TRACK_WIDTH
#define MINIMUM_TRACK_WIDTH (1.0F)
#endif
#ifndef MAXIMUM_TRACK_WIDTH
#define MAXIMUM_TRACK_WIDTH (2.0F)
#endif
#ifndef DEFAULT_TRACK_WIDTH
#define DEFAULT_TRACK_WIDTH (2.0F)
#endif

/* wheel base */
#ifndef MINIMUM_WHEEL_BASE
#define MINIMUM_WHEEL_BASE (2.0F)
#endif
#ifndef MAXIMUM_WHEEL_BASE
#define MAXIMUM_WHEEL_BASE (4.5F)
#endif
#ifndef DEFAULT_WHEEL_BASE
#define DEFAULT_WHEEL_BASE (3.0F)
#endif

/* steering angle ratio */
#ifndef MINIMUM_STEERING_RATIO
#define MINIMUM_STEERING_RATIO (1.0F)
#endif
#ifndef MAXIMUM_STEERING_RATIO
#define MAXIMUM_STEERING_RATIO (500.0F)
#endif
#ifndef DEFAULT_STEERING_RATIO
#define DEFAULT_STEERING_RATIO (15.0F)
#endif

/* self steering gradient */
#ifndef MINIMUM_SELF_STEERING_GRAD
#define MINIMUM_SELF_STEERING_GRAD (BML_Deg2Rad(0.1F))
#endif
#ifndef MAXIMUM_SELF_STEERING_GRAD
#define MAXIMUM_SELF_STEERING_GRAD (BML_Deg2Rad(0.4F))
#endif
#ifndef DEFAULT_SELF_STEERING_GRAD
#define DEFAULT_SELF_STEERING_GRAD (BML_Deg2Rad(0.2F))
#endif

/* wheel pulses per revolution */
#ifndef MINIMUM_WHEEL_TICKS_REV
#define MINIMUM_WHEEL_TICKS_REV (1U)
#endif
#ifndef MAXIMUM_WHEEL_TICKS_REV
#define MAXIMUM_WHEEL_TICKS_REV (250U)
#endif
#ifndef DEFAULT_WHEEL_TICKS_REV
#define DEFAULT_WHEEL_TICKS_REV (96U)
#endif

/* center of gravity heigth */
#ifndef MINIMUM_COGH
#define MINIMUM_COGH (0.02F)
#endif
#ifndef MAXIMUM_COGH
#define MAXIMUM_COGH (1.0F)
#endif
#ifndef DEFAULT_COGH
#define DEFAULT_COGH (0.8F)
#endif

/* wheel cirumference */
#ifndef MINIMUM_WHEEL_CIRCUM
#define MINIMUM_WHEEL_CIRCUM (1.5F)
#endif
#ifndef MAXIMUM_WHEEL_CIRCUM
#define MAXIMUM_WHEEL_CIRCUM (2.5F)
#endif
#ifndef DEFAULT_WHEEL_CIRCUM
#define DEFAULT_WHEEL_CIRCUM (2.0F)
#endif

/* axle load distribution */
#ifndef MINIMUM_AXLE_LOAD_DISTR
#define MINIMUM_AXLE_LOAD_DISTR (0.2F)
#endif
#ifndef MAXIMUM_AXLE_LOAD_DISTR
#define MAXIMUM_AXLE_LOAD_DISTR (0.8F)
#endif
#ifndef DEFAULT_AXLE_LOAD_DISTR
#define DEFAULT_AXLE_LOAD_DISTR (0.5F)
#endif

/* wheel load distribution */
#ifndef MINIMUM_WHEEL_LOAD_DEP
#define MINIMUM_WHEEL_LOAD_DEP (0.0F)
#endif
#ifndef MAXIMUM_WHEEL_LOAD_DEP
#define MAXIMUM_WHEEL_LOAD_DEP (3.0F)
#endif
#ifndef DEFAULT_WHEEL_LOAD_DEP
#define DEFAULT_WHEEL_LOAD_DEP (2.0F)
#endif

/* acceleration threshold for increased allowed velocity variance */
#ifndef VED__ACC_THRESHOLD_INC_VELOCITY_VAR
#define VED__ACC_THRESHOLD_INC_VELOCITY_VAR (0.5F)
#endif

#if ((defined(CFG_VED__DEBOUNCE_OUTPUTS)) && (CFG_VED__DEBOUNCE_OUTPUTS))
/* maximum debouncing of faulty outputs */
#ifndef VED__MAX_OUTPUT_DEBOUNCING
#define VED__MAX_OUTPUT_DEBOUNCING (0.0F)
#endif
#endif

/*maximum ved_ cycles to deactivate the output Velocity monitor when abs is
 * active */
#ifndef OUTPUT_VEL_MON_DEACTIVATION_ABS
#define OUTPUT_VEL_MON_DEACTIVATION_ABS (0.0F)
#endif

/*maximum ved_ cycles to deactivate the output Velocity monitor when TSC is
 * active */
#ifndef OUTPUT_VEL_MON_DEACTIVATION_TSC
#define OUTPUT_VEL_MON_DEACTIVATION_TSC (0.0F)
#endif

#if (CFG_VED__YWR_OFFSET_MONITOR)
/* Velocity threshold for setting vehicle halt in case of vehicle standstill */
#define VED__YWR_MON_VEL_STST_THRHD ((float32)(10.0F / C_KMH_MS))

/* Yaw rate differentiator time constant for offset monitor */
#define VED__YWR_MON_DIFF_FILT ((float32)1.5F)
/* Maximum yaw acceleration during standstill for calibration */
#define VED__YWR_MON_YWR_GRAD_MAX (float32) DEG2RAD(0.50F)
/* Maximum monitor standstill time for monitoring */
#define VED__YWR_MON_TIME_MAX ((float32)3600.F)

/* Yaw rate threshold during vehicle-halt */
#define VED__YWR_MON_VHALT_YWR_MAX ((float32)DEG2RAD(20.F))
/* Maximum allowed time span for yaw rate above threshold at standstill */
#define VED__YWR_MON_VHALT_TIME_MAX ((float32)0.2F)

/* Yaw rate threshold before drive-off */
#define VED__YWR_MON_VDOFF_YWR_MAX ((float32)DEG2RAD(10.F))
/* Maximum allowed time span for yaw rate above threshold before drive-off */
#define VED__YWR_MON_VDOFF_TIME_MAX ((float32)4.5F)

/* Maximum allowed difference of yaw rate during vehicle-halt and drive-off */
#define VED__YWR_MON_VHTDR_DIFF_MAX DEG2RAD(13.0F)
/* Window length for yaw rate sample evaluation during vehicle-halt and
 * drive-off */
#define VED__YWR_MON_VHTDR_WIN_LEN 20.F
/* Maximum permitted standard deviation to take window-mean as offset */
#define VED__YWR_MON_VHTDR_WIN_DEV DEG2RAD(0.5F)

/* Maximum permitted vehicle halt time without standstill calibration */
#define VED__YWR_MON_CAL_MAX_TIME ((float32)3.0F)

/* Threshold (hysteresis) to enable active error indication */
#define VED__YWR_MON_CAL_VEL_START_THRHD ((float32)(15.0F / C_KMH_MS))

#endif

#if ((defined(CFG_VED__ALIGNMENT_OFFSET_MONITOR)) && \
     (CFG_VED__ALIGNMENT_OFFSET_MONITOR))
#define VED__ALN_OFFSET_MON_NO_DATA \
    (1E30F) /* used by ALN if no yawrate could have been calculated */
#endif

/*****************************************************************************
  MACROS
*****************************************************************************/
#if (CFG_VED__USE_VELO_MONITOR)
#define VMON_GET_ME (&VED_VelMonData)
#endif

#if (CFG_VED__YWR_OFFSET_MONITOR)
#define YWR_GET_MON_DATA (&VED_YwrMonGlobData)
#endif

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
#if (CFG_VED__YWR_OFFSET_MONITOR)
/*! Yaw rate monitoring common data */
typedef struct {
    float32 timeVehHalt;
    float32 ywRate;
    float32 cycTime;
    boolean StSt;
    boolean VehStSt;
    float32 Velocity;
} VED_YwrMonDataLoc_t;
#endif

/*****************************************************************************
  CONSTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
#if (CFG_VED__USE_VELO_MONITOR)
SET_MEMSEC_VAR(VED_VelMonData)
static VED_VelMon_t VED_VelMonData;
#endif

#if (CFG_VED__YWR_OFFSET_MONITOR)
/* Yaw rate monitor database */
SET_MEMSEC_VAR(VED_YwrMonGlobData)
static VED_YwrMonData_t
    VED_YwrMonGlobData; /*!< @VADDR: 0x20017700 @VNAME: VED_YwrMon @ALLOW:
                           ved__priv @cycleid: ved__cycle_id*/
#endif

#if ((defined(CFG_VED__ALIGNMENT_OFFSET_MONITOR)) && \
     (CFG_VED__ALIGNMENT_OFFSET_MONITOR))
SET_MEMSEC_VAR(ACCThldTime)
static uint32 ACCThldTime;
SET_MEMSEC_VAR(CGThldTime)
static uint32 CGThldTime;
#endif

SET_MEMSEC_VAR(LastInputSignals)
static VED_LastInSig_t LastInputSignals;

#if ((defined(CFG_VED__MON_OUTPUT_PEAKS)) && (CFG_VED__MON_OUTPUT_PEAKS))
SET_MEMSEC_VAR(LastOutputSignals)
static VED_LastOutSig_t LastOutputSignals;
#endif

#if !(defined(CFG_VED__DIS_FUNCTIONAL_SAFETY_MON) && \
      (CFG_VED__DIS_FUNCTIONAL_SAFETY_MON))
SET_MEMSEC_VAR(gVED_FSData)
static VED_FSData_t gVED_FSData; /*!< @VADDR: 0x20016500uL @VNAME: VED_FsMon
                                    @ALLOW: ved__priv @cycleid: ved__cycle_id*/
#endif

#if ((!defined(CFG_VED__DIS_YWR_OFFSET_COMP)) || \
     (!CFG_VED__DIS_YWR_OFFSET_COMP))
SET_MEMSEC_VAR(oldOffset)
static float32 oldOffset;
SET_MEMSEC_VAR(StandStillOffset)
static float32 StandStillOffset;
#endif

static uint16 u_velMonitorOFFCounter_ABS;

static uint16 u_velMonitorOFFCounter_TSC;

#if ((defined(CFG_VED__DEBOUNCE_OUTPUTS)) && (CFG_VED__DEBOUNCE_OUTPUTS))
static uint8 u_Debouce_VED_VEH_VEL_NOT_AVAILABLE;
#endif
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  LOCAL FUNCTION PROTOTYPES
*****************************************************************************/

#if !(defined(CFG_VED__DIS_FUNCTIONAL_SAFETY_MON) && \
      (CFG_VED__DIS_FUNCTIONAL_SAFETY_MON))
static void VED_FSVelocityMonitoring(const proVEDPrtList_t *proPorts,
                                     VED_FSData_t *VED_FSData);
static void VED_FSYawRateMonitoring(const VED_InternalData_t *IntData,
                                    const proVEDPrtList_t *proPorts,
                                    const VED_ModIf_t *VED_ModIf,
                                    VED_FSData_t *VED_FSData);
static void VED_FSCalcDeltaYawRate(VED_FSData_t *VED_FSData,
                                   const VED_InternalData_t *IntData);
static void VED_FSCurvePredictionMonitoring(const VED_InternalData_t *IntData,
                                            VED_FSData_t *VED_FSData);
#endif

#if (((!(defined(CFG_VED__DIS_FUNCTIONAL_SAFETY_MON) && \
         (CFG_VED__DIS_FUNCTIONAL_SAFETY_MON))) &&      \
      ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
       (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))) &&        \
     (CFG_VED__FS_YR_VS_WSP_ENABLE))
static void VED_FSMonitorWheelYawRate(const VED_InternalData_t *pIntData,
                                      const proVEDPrtList_t *proPorts,
                                      const VED_ModIf_t *pVED_ModIf,
                                      VED_FSData_t *pVED_FSData);
#endif
#if (CFG_VED__USE_VELO_MONITOR)
static void VED_VelMon(const V1_7_VEDVehSigMain_t *in,
                       const ved__ve_out_t *ve,
                       VED_Errors_t *pErr);
#endif

static void VED_MonitorInputSignals(VED_InputData_t *input,
                                    VED_Errors_t *VED_Errors);
static void VED_MonitorInputParameters(VED_InputData_t *input,
                                       VED_Errors_t *VED_Errors);
#if (VED_VEH_DYN_INTFVER <= 5)
static uint8 VED_CheckSignalPeakError(float32 *pInputSignal,
                                      float32 *pLastInputSignal,
                                      uint32 const *pInputSignalState,
                                      uint32 *pLastInputSignalState,
                                      uint32 Signal,
                                      float32 Threshold);
#else
static uint8 VED_CheckSignalPeakError(float32 *pInputSignal,
                                      float32 *pLastInputSignal,
                                      uint8 pInputSignalState[],
                                      uint8 pLastInputSignalState[],
                                      uint32 Signal,
                                      float32 Threshold);
#endif
static void VED_CheckForSignalPeakErrors(VED_InputData_t *pInput,
                                         VED_Errors_t *pVED_Errors);

static void VED_MonitorVelocityOutput(const proVEDPrtList_t *proPorts,
                                      const VED_InputData_t *input);

static void VED_MonitorYawrateOutput(const proVEDPrtList_t *proPorts);
static void VED_MonitorOutputRanges(const proVEDPrtList_t *proPorts);

#if ((defined(CFG_VED__MON_OUTPUT_PEAKS)) && (CFG_VED__MON_OUTPUT_PEAKS))
static void VED_MonitorOutputPeaks(const VED_InputData_t *pInput,
                                   const proVEDPrtList_t *proPorts);
#endif

#if (CFG_VED__YWR_OFFSET_MONITOR)
static uint8 VED_YwrMonVehHalt(const VED_YwrMonDataLoc_t *pMon,
                               VED_YwrMonVehHalt_t *pVehHalt);
static uint8 VED_YwrMonVehHaltDrv(const VED_YwrMonDataLoc_t *pMon,
                                  VED_YwrMonVehHaltDrv_t *pVehHtDrv);
static uint8 VED_YwrMonVehHaltCal(const VED_YwrMonDataLoc_t *pMon,
                                  VED_YwrMonVehHaltCal_t *pVehHaltCal);
static uint8 VED_YwrMonVehDriveOff(const VED_YwrMonDataLoc_t *pMon,
                                   VED_YwrMonVehDriveOff_t *pDrvOff);
#endif

#if !(defined(CFG_VED__DIS_FUNCTIONAL_SAFETY_MON) && \
      (CFG_VED__DIS_FUNCTIONAL_SAFETY_MON))
/* **********************************************************************
  @fn               VED_FSVelocityMonitoring  */ /*!
  @brief            Functional safety velocity monitoring

  @description      Sets FuSi related output faults if velocity faults were detected

  @param[in]        provided ports
  @param[out]       FS data
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_FSVelocityMonitoring(const proVEDPrtList_t *proPorts,
                                     VED_FSData_t *VED_FSData) {
    /* this error was determined by the velo internal external mon function
     * (VED_VelMonExec) */
#if (defined(CFG_VED__USE_VELO_MONITOR) && (CFG_VED__USE_VELO_MONITOR))
    VED_FSData->FSMonErrorStates.FSIntExtVeloCheck =
        proPorts->pVED_Errors->OutPutErrors.VelMonLt;
#else
    VED_FSData->FSMonErrorStates.FSIntExtVeloCheck = VED_ERR_STATE_UNKNOWN;
#endif

#if (defined(CFG_VED__DO_VELOCITY_CORR) && (CFG_VED__DO_VELOCITY_CORR))
    /* this error was determined by the velo corr module */
    VED_FSData->FSMonErrorStates.FSCorrVeloCheck =
        proPorts->pVED_Errors->OutPutErrors.VelCorrRg;
#else
    VED_FSData->FSMonErrorStates.FSCorrVeloCheck = VED_ERR_STATE_UNKNOWN;
#ifdef _WIN32
#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): Can not check for functional safety corr velo errors, please switch on CFG_VED__DO_VELOCITY_CORR")

#endif
#endif

#if ((defined(CFG_VED__FS_VELO_CORR_MON)) && (CFG_VED__FS_VELO_CORR_MON))
    /* If the fast velocity monitor detects a fault, raise correction factor
     * variance to inhibit CMS reactions */
    if (proPorts->pVED_Errors->OutPutErrors.VED_FS_VEH_CORR_MON ==
        VED_ERR_STATE_ACTIVE) {
        proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrVar =
            100.0F;
    }
#endif
}

/* **********************************************************************
  @fn               VED_FSYawRateMonitoring */ /*!
  @brief            Functional safety yaw rate monitoring

  @description      While driving straight the difference between the internal yaw rates
                    are checked
                    Sets FuSi related output faults if differences were detected
                    over a defined distance

  @param[in]        internal data
  @param[in]        internal module interface
  @param[in]        FS data
  @param[out]       provided ports
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_FSYawRateMonitoring(const VED_InternalData_t *IntData,
                                    const proVEDPrtList_t *proPorts,
                                    const VED_ModIf_t *VED_ModIf,
                                    VED_FSData_t *VED_FSData) {
#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
    float32 fDeltaDist = IntData->ved__ve_out.veh_velo * VED_ModIf->CycleTime;
#endif

#if (((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
      (!CFG_VED__DIS_WHEEL_PRE_PROCESSING)) &&         \
     (CFG_VED__FS_YR_VS_WSP_ENABLE))
    /* threshold velocities, above yaw rate error monitoring is active */
    /* curve threshold, only below this the yaw rate is monitored */
    VED_FSMonitorWheelYawRate(IntData, proPorts, VED_ModIf, VED_FSData);
#else
    proPorts->pVED_Errors->OutPutErrors.VED__FS_YR_VS_WSP =
        VED_ERR_STATE_UNKNOWN;
#ifdef _WIN32
#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): Can not check for functional safety wheel yaw rate to yaw rate, please switch on CFG_VED__DIS_WHEEL_PRE_PROCESSING and CFG_VED__FS_YR_VS_WSP_ENABLE")

#endif
#endif

#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
    /* Is the vehicle velocity above the yaw rate velocity */
    /* Is the curve threshold below this the yaw rate */
    if ((fABS(IntData->ved__ye_out.veh_merge_curve_grad) <= MAX_CURVE_GRAD) &&
        (IntData->ved__ve_out.veh_velo >= MIN_LAT_ACCEL_YAW_RATE_VELOCITY)) {
        VED_FSData->VED_FSYawRateCheck.fYawLatAccelDelta =
            IntData->ved__gye_out.gier_yaw_rate -
            IntData->ved__aye_out.ay_yaw_rate;
        /* add the delta value to the statistic data */
        VED_StatIntervalAdd(&(VED_FSData->VED_FSCurveCheck.VED_FSAyeGyeStat),
                            VED_FSData->VED_FSYawRateCheck.fYawLatAccelDelta,
                            fDeltaDist);
        /* Check if above threshold */
        if (fABS(VED_FSData->VED_FSYawRateCheck.fYawLatAccelDelta) >
            VED__FS_LAT_ACCEL_YAW_RATE_THRESHOLD) {
            VED_FSData->VED_FSYawRateCheck.fYawLatAccelOutDist += fDeltaDist;
        } else {
            VED_FSData->VED_FSYawRateCheck.fYawLatAccelOutDist = 0.0F;
        }
    } else {
        VED_FSData->VED_FSYawRateCheck.fYawLatAccelOutDist = 0.0F;
    }

    /* set the error state */
    if (VED_FSData->VED_FSYawRateCheck.fYawLatAccelOutDist >
        VED__FS_YAW_RATE_DIST_THRESHOLD) {
        if ((proPorts->pVED_Errors->OutPutErrors.VED_FS_YR_VS_AY ==
             (uint8)VED_ERR_STATE_INACTIVE) ||
            (proPorts->pVED_Errors->OutPutErrors.VED_FS_YR_VS_AY ==
             (uint8)VED_ERR_STATE_UNKNOWN)) {
            /* increase occurence counter, only used for fault analysis, overrun
             * used to continue counting */
            VED_FSData->VED_FSYawRateCheck.nLatAccelErrorCounter++;
        }
        proPorts->pVED_Errors->OutPutErrors.VED_FS_YR_VS_AY =
            VED_ERR_STATE_ACTIVE;
    } else {
        proPorts->pVED_Errors->OutPutErrors.VED_FS_YR_VS_AY =
            VED_ERR_STATE_INACTIVE;
    }
#else
    proPorts->pVED_Errors->OutPutErrors.VED_FS_YR_VS_AY = VED_ERR_STATE_UNKNOWN;
#endif

#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
    if ((fABS(IntData->ved__ye_out.veh_merge_curve_grad) <= MAX_CURVE_GRAD) &&
        (IntData->ved__ve_out.veh_velo >= MIN_SWA_YAW_RATE_VELOCITY)) {
        VED_FSData->VED_FSYawRateCheck.fYawSteerDelta =
            IntData->ved__gye_out.gier_yaw_rate -
            IntData->ved__sye_out.stw_yaw_rate;
        /* add the delta value to the statistic data */
        VED_StatIntervalAdd(&(VED_FSData->VED_FSCurveCheck.VED_FSSyeGyeStat),
                            VED_FSData->VED_FSYawRateCheck.fYawSteerDelta,
                            fDeltaDist);
        /* Check if above threshold */
        if (fABS(VED_FSData->VED_FSYawRateCheck.fYawSteerDelta) >
            VED__FS_SWA_YAW_RATE_THRESHOLD) {
            VED_FSData->VED_FSYawRateCheck.fYawSteerOutDist += fDeltaDist;
        } else {
            VED_FSData->VED_FSYawRateCheck.fYawSteerOutDist = 0.0F;
        }
    } else {
        VED_FSData->VED_FSYawRateCheck.fYawSteerOutDist = 0.0F;
    }

    /* set the error state */
    if (VED_FSData->VED_FSYawRateCheck.fYawSteerOutDist >
        VED__FS_YAW_RATE_DIST_THRESHOLD) {
        if ((proPorts->pVED_Errors->OutPutErrors.VED__FS_YR_VS_SWA ==
             (uint8)VED_ERR_STATE_INACTIVE) ||
            (proPorts->pVED_Errors->OutPutErrors.VED__FS_YR_VS_SWA ==
             (uint8)VED_ERR_STATE_UNKNOWN)) {
            VED_FSData->VED_FSYawRateCheck.nSwaErrorCounter++;
        }
        proPorts->pVED_Errors->OutPutErrors.VED__FS_YR_VS_SWA =
            VED_ERR_STATE_ACTIVE;
    } else {
        proPorts->pVED_Errors->OutPutErrors.VED__FS_YR_VS_SWA =
            VED_ERR_STATE_INACTIVE;
    }
#else
    proPorts->pVED_Errors->OutPutErrors.VED__FS_YR_VS_SWA =
        VED_ERR_STATE_UNKNOWN;
#endif
}

#if (((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
      (!CFG_VED__DIS_WHEEL_PRE_PROCESSING)) &&         \
     (CFG_VED__FS_YR_VS_WSP_ENABLE))
/* **********************************************************************
  @fn               VED_FSMonitorWheelYawRate */ /*!
  @brief            Monitors wheel yaw rate distance

  @description      If the wheel yaw rate become greater than the threashold,
	                  the yaw rate out of bounce error becomes active

  @param[in]        pIntData : Pointer to the VED Internal Data
  @param[in]        proPorts : Pointer to the VED provide port data
  @param[in]        pVED_ModIf : Pointer to Component internal intra module interface
  @param[in,out]    pVED_FSData : Pointer to Functional safety internal data
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_FSMonitorWheelYawRate(const VED_InternalData_t *pIntData,
                                      const proVEDPrtList_t *proPorts,
                                      const VED_ModIf_t *pVED_ModIf,
                                      VED_FSData_t *pVED_FSData) {
    float32 fDeltaDist = pIntData->ved__ve_out.veh_velo * pVED_ModIf->CycleTime;
    /* threshold velocities, equal or above yaw rate error monitoring is active
     */
    /* curve threshold, only equal or below this the yaw rate is monitored */
    /* wheel yaw rate variance, only if wheel yaw rate is available the fault
     * monitoring is active */
    if ((fABS(pIntData->ved__ye_out.veh_merge_curve_grad) <= MAX_CURVE_GRAD) &&
        (pIntData->ved__ve_out.veh_velo >= MIN_WHEEL_YAW_RATE_VELOCITY) &&
        (pIntData->ved__wye_out.whl_yaw_rate_var <
         MAX_WHEEL_YAW_RATE_VARIANCE)) {
        pVED_FSData->VED_FSYawRateCheck.fYawWheelDelta =
            pIntData->ved__gye_out.gier_yaw_rate -
            pIntData->ved__wye_out.whl_yaw_rate;
        /* add the delta value to the statistic data */
        VED_StatIntervalAdd(&(pVED_FSData->VED_FSCurveCheck.VED_FSWyeGyeStat),
                            pVED_FSData->VED_FSYawRateCheck.fYawWheelDelta,
                            fDeltaDist);
        /* Check if above threshold */
        if (fABS(pVED_FSData->VED_FSYawRateCheck.fYawWheelDelta) >
            VED__FS_WHEEL_YAW_RATE_THRESHOLD) {
            pVED_FSData->VED_FSYawRateCheck.fYawWheelOutDist += fDeltaDist;
        } else {
            pVED_FSData->VED_FSYawRateCheck.fYawWheelOutDist = 0.0F;
        }
    } else {
        pVED_FSData->VED_FSYawRateCheck.fYawWheelOutDist = 0.0F;
    }

    /* set the error state */
    if (pVED_FSData->VED_FSYawRateCheck.fYawWheelOutDist >
        VED__FS_YAW_RATE_DIST_THRESHOLD) {
        if ((proPorts->pVED_Errors->OutPutErrors.VED__FS_YR_VS_WSP ==
             (uint8)VED_ERR_STATE_INACTIVE) ||
            (proPorts->pVED_Errors->OutPutErrors.VED__FS_YR_VS_WSP ==
             (uint8)VED_ERR_STATE_UNKNOWN)) {
            /* increase occurence counter, only used for fault analysis, overrun
             * used to continue counting */
            pVED_FSData->VED_FSYawRateCheck.nWheelErrorCounter++;
        }
        proPorts->pVED_Errors->OutPutErrors.VED__FS_YR_VS_WSP =
            VED_ERR_STATE_ACTIVE;
    } else {
        proPorts->pVED_Errors->OutPutErrors.VED__FS_YR_VS_WSP =
            VED_ERR_STATE_INACTIVE;
    }
}
#endif

/* **********************************************************************
  @fn               VED_FSCalcDeltaYawRate */ /*!
  @brief            Functional safety, calculate the delta yaw rate need by the curve prediction monitoring

  @description      Updates yaw rate data for lateral acceleration, steering wheel and wheels
                    If one yaw rate was collected over a defined distance, the overall statistic data
                    is calculated and the curve prediction fault is calculated
                    If the difference is above a fault threshold, a delta counter is increased

  @param[in]        IntData    statistc fault data
  @param[in,out]    VED_FSData  yaw rates
  @param[out]       VED_FSData  fault counter
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_FSCalcDeltaYawRate(VED_FSData_t *VED_FSData,
                                   const VED_InternalData_t *IntData) {
    boolean bCalcOverAll = FALSE;

    /* check if the WyeGye intervall is above volume threshold to calc the
     * mean/std and reduce the intervall */
    if (VED_FSData->VED_FSCurveCheck.VED_FSWyeGyeStat.Volume >=
        VED__FS_MEAN_DRIVE_DISTANCE) {
        VED_StatIntervalMeanDev(
            &(VED_FSData->VED_FSCurveCheck.VED_FSWyeGyeStat));
        VED_StatIntervalReduce(&(VED_FSData->VED_FSCurveCheck.VED_FSWyeGyeStat),
                               VED__FS_DISTANCE_REDUCE_FACTOR);
        bCalcOverAll = TRUE;
    }

    /* check if the AyeGye intervall is above volume threshold to calc the
     * mean/std and reduce the intervall */
    if (VED_FSData->VED_FSCurveCheck.VED_FSAyeGyeStat.Volume >=
        VED__FS_MEAN_DRIVE_DISTANCE) {
        VED_StatIntervalMeanDev(
            &(VED_FSData->VED_FSCurveCheck.VED_FSAyeGyeStat));
        VED_StatIntervalReduce(&(VED_FSData->VED_FSCurveCheck.VED_FSAyeGyeStat),
                               VED__FS_DISTANCE_REDUCE_FACTOR);
        bCalcOverAll = TRUE;
    }

    /* check if the SyeGye intervall is above volume threshold to calc the
     * mean/std and reduce the intervall */
    if (VED_FSData->VED_FSCurveCheck.VED_FSSyeGyeStat.Volume >=
        VED__FS_MEAN_DRIVE_DISTANCE) {
        VED_StatIntervalMeanDev(
            &(VED_FSData->VED_FSCurveCheck.VED_FSSyeGyeStat));
        VED_StatIntervalReduce(&(VED_FSData->VED_FSCurveCheck.VED_FSSyeGyeStat),
                               VED__FS_DISTANCE_REDUCE_FACTOR);
        bCalcOverAll = TRUE;
    }

    /* calculate the overall mean and std */
    if (bCalcOverAll == TRUE) {
        VED_StatIntervalInit(
            &(VED_FSData->VED_FSCurveCheck.VED_FSAllOverIntervall));
        VED_StatIntervalAdd(
            &(VED_FSData->VED_FSCurveCheck.VED_FSAllOverIntervall),
            VED_FSData->VED_FSCurveCheck.VED_FSWyeGyeStat.Mean,
            VED__WYE_CURVECHECK_WEIGHT);
        VED_StatIntervalAdd(
            &(VED_FSData->VED_FSCurveCheck.VED_FSAllOverIntervall),
            VED_FSData->VED_FSCurveCheck.VED_FSAyeGyeStat.Mean,
            VED__AYE_CURVECHECK_WEIGHT);
        VED_StatIntervalAdd(
            &(VED_FSData->VED_FSCurveCheck.VED_FSAllOverIntervall),
            VED_FSData->VED_FSCurveCheck.VED_FSSyeGyeStat.Mean,
            VED__SYE_CURVECHECK_WEIGHT);
        VED_StatIntervalMeanDev(
            &(VED_FSData->VED_FSCurveCheck.VED_FSAllOverIntervall));

        /* Calculate the curve prediction error */
        VED_FSData->VED_FSCurveCheck.fDeltaY =
            0.5F * IntData->ved__ve_out.veh_velo * VED__FS_PRED_HORZ_TIME *
            VED__FS_PRED_HORZ_TIME *
            VED_FSData->VED_FSCurveCheck.VED_FSAllOverIntervall.Mean;

        /* Check if DeltaY is above threshold and exeed counters */
        if ((VED_FSData->VED_FSCurveCheck.fDeltaY >=
             VED__FS_MAX_LAT_DIST_ERROR) &&
            (VED_FSData->VED_FSCurveCheck.nDeltaYCounter <=
             (uint8)(VED__FS_MAX_LAT_DIST_COUNTER))) {
            VED_FSData->VED_FSCurveCheck.nDeltaYCounter++;
        } else {
            VED_FSData->VED_FSCurveCheck.nDeltaYCounter = 0U;
        }
    }
}

/* **********************************************************************
  @fn               VED_FSCurvePredictionMonitoring */ /*!
  @brief            Functional safety curve prediction monitoring

  @description      Sets fault if curve prediction delta counter is above theshold
                    see VED_FSCalcDeltaYawRate() for delta counter calculation

  @param[in]        IntData    statistc fault data
  @param[in,out]    VED_FSData  yaw rates
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_FSCurvePredictionMonitoring(const VED_InternalData_t *IntData,
                                            VED_FSData_t *VED_FSData) {
    /* Caculate the yaw rate delta */
    VED_FSCalcDeltaYawRate(VED_FSData, IntData);

    /* Check if error condition is valid */
    if (VED_FSData->VED_FSCurveCheck.nDeltaYCounter >=
        (uint8)(VED__FS_MAX_LAT_DIST_COUNTER)) {
        if ((VED_FSData->FSMonErrorStates.FSMaxLatDisplacementError ==
             (uint8)VED_ERR_STATE_INACTIVE) ||
            (VED_FSData->FSMonErrorStates.FSMaxLatDisplacementError ==
             (uint8)VED_ERR_STATE_UNKNOWN)) {
            VED_FSData->VED_FSCurveCheck.nCurveErrorCounter++;
        }
        VED_FSData->FSMonErrorStates.FSMaxLatDisplacementError =
            VED_ERR_STATE_ACTIVE;
    } else {
        VED_FSData->FSMonErrorStates.FSMaxLatDisplacementError =
            VED_ERR_STATE_INACTIVE;
    }
}

/* **********************************************************************
  @fn               VED_FSMonitor */ /*!
  @brief            Functional safety monitoring

  @description      Calls functional safety fault monitors (yaw rate, velocity and curve)

  @param[in]        IntData
  @param[in]        VED_ModIf
  @param[in]        proPorts
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_FSMonitor(const VED_InternalData_t *IntData,
                   const VED_ModIf_t *VED_ModIf,
                   const proVEDPrtList_t *proPorts) {
    /* Do the functional safety velocity monitoring */
    VED_FSVelocityMonitoring(proPorts, &gVED_FSData);

    /* Do the functional safety yawrate monitoring */
    VED_FSYawRateMonitoring(IntData, proPorts, VED_ModIf, &gVED_FSData);

    /* Do the functional safety curve prediction monitoring */
    VED_FSCurvePredictionMonitoring(IntData, &gVED_FSData);
}

/* **********************************************************************
  @fn               VED_FSInit */ /*!
  @brief            Initalize the functional safety monitoring

  @description      Initializes the FuSi related fault monitor data
                    (yaw rate, velocity and curve)

  @param[in]        -
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_FSInit(void) {
    /* Init the statistic data buffers */
    VED_StatIntervalInit(&(gVED_FSData.VED_FSCurveCheck.VED_FSWyeGyeStat));
    VED_StatIntervalInit(&(gVED_FSData.VED_FSCurveCheck.VED_FSAyeGyeStat));
    VED_StatIntervalInit(&(gVED_FSData.VED_FSCurveCheck.VED_FSSyeGyeStat));
    gVED_FSData.VED_FSCurveCheck.nCurveErrorCounter = 0U;

    /* Init yaw rate monitoring */
    gVED_FSData.VED_FSYawRateCheck.fYawWheelDelta = 0.0f;
    gVED_FSData.VED_FSYawRateCheck.fYawWheelOutDist = 0.0f;
    gVED_FSData.VED_FSYawRateCheck.fYawLatAccelDelta = 0.0f;
    gVED_FSData.VED_FSYawRateCheck.fYawLatAccelOutDist = 0.0f;
    gVED_FSData.VED_FSYawRateCheck.fYawSteerDelta = 0.0f;
    gVED_FSData.VED_FSYawRateCheck.fYawSteerOutDist = 0.0f;
    gVED_FSData.VED_FSYawRateCheck.nWheelErrorCounter = 0U;
    gVED_FSData.VED_FSYawRateCheck.nLatAccelErrorCounter = 0U;
    gVED_FSData.VED_FSYawRateCheck.nSwaErrorCounter = 0U;

    /* Init all error states to unkown */
    gVED_FSData.FSMonErrorStates.FSIntExtVeloCheck = VED_ERR_STATE_UNKNOWN;
    gVED_FSData.FSMonErrorStates.FSCorrVeloCheck = VED_ERR_STATE_UNKNOWN;

    gVED_FSData.FSMonErrorStates.FSMaxLatDisplacementError =
        VED_ERR_STATE_UNKNOWN;
}

/* **********************************************************************
  @fn                     VED_FSMonGetPrivateData */ /*!
  @brief                  Get the private date of the functional safety monitoring

  @description            Returns pointer to FuSi monitor data to allow MTS output

  @param[in]              -
  @param[out]             -
  @return                 *VED_FSData_t
  
  @pre                    -
  @post                   -
**************************************************************************** */
VED_FSData_t *VED_FSMonGetPrivateData(void) { return (&gVED_FSData); }
#endif

#if defined(VED__SIMU)
/* **********************************************************************
  @fn               VED_GetMonitorInputData */ /*!
  @brief            Get the ved_ Monitor input signals

  @description      Returns pointer to the last input signals monitor data
                    to allow access to this data in the simulation

  @param[in]        -
  @param[out]       -
  @return           Pointer to input signals of last cycle

  @pre              -
  @post             -
**************************************************************************** */
VED_LastInSig_t *VED_GetMonitorInput(void) { return &LastInputSignals; }

#if ((defined(CFG_VED__MON_OUTPUT_PEAKS)) && (CFG_VED__MON_OUTPUT_PEAKS))
/* **********************************************************************
  @fn               VED_GetMonitorOutputPeaksData */ /*!
  @brief            Get the ved_ Monitor output peaks signals and parameters

  @description      Returns pointer to the output signals peak monitor data
                    to allow access to this data in the simulation

  @param[in]        - 
  @param[out]       -
  @return           LastOutputSignals

  @pre              -
  @post             -
**************************************************************************** */
VED_LastOutSig_t *VED_GetMonitorOutputPeaksData(void) {
    return &LastOutputSignals;
}
#endif
#endif

/* **********************************************************************
  @fn               VED_MonitorInput */ /*!
  @brief            Monitor input signals and parameters

  @description      Calls signal and parameter monitoring functions
                    Calls signal peak monitoring function

  @param[in]        input signals
  @param[out]       VED_Errors signal fault state
  @return           void

  @pre              VED must be running (CtrlMode VED_CTRL_STATE_STARTUP)
  @post             -
**************************************************************************** */
void VED_MonitorInput(VED_InputData_t *input, VED_Errors_t *VED_Errors) {
    /* check for invalid parameter values or undefined parameters */
    VED_MonitorInputParameters(input, VED_Errors);

    /* check for invalid signal values */
    VED_MonitorInputSignals(input, VED_Errors);

    /* check for signal peaks between the actual and the last cycle */
    VED_CheckForSignalPeakErrors(input, VED_Errors);
}

/* **********************************************************************
  @fn               VED_MonitorInputParameters */ /*!
  @brief            Monitor input parameters

  @description      Checks that parameters are within specifiaction and that they are
                    set as valid
                    If a parameter is outside of the specified range, it
                    will be set to a default to avoid any calculation issues
                    and the parameter state is set to invalid
                    If any parameters is either invalid or out-of-range, 
                    it triggers a parameter fault

  @param[in]        input input signals
  @param[out]       VED_Errors parameter fault state
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_MonitorInputParameters(VED_InputData_t *input,
                                       VED_Errors_t *VED_Errors) {
    uint8 ParameterError = VED__STAT_INACTIVE;

    /* Check for invalid parameter if parameter is used */
#if (((!defined(CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION)) || \
      (!CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION)) ||         \
     ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) ||       \
      (!CFG_VED__DIS_WHEEL_PRE_PROCESSING)))
    /* Check that vehicle weight parameter is valid (might be on a default if
     * not valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_VEHWGT, input->Parameter.State) !=
    // VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_VEHWGT, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that vehicle weight parameter is within range */
    if ((input->Parameter.VED_Kf_VehWeight_kg < MINIMUM_VEH_WEIGHT) ||
        (input->Parameter.VED_Kf_VehWeight_kg > MAXIMUM_VEH_WEIGHT)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.VED_Kf_VehWeight_kg = DEFAULT_VEH_WEIGHT;
        // VED_SET_IO_STATE(VED_PAR_POS_VEHWGT, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }
#endif

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* Check that track width front parameter is valid (might be on a default if
     * not valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_TWDFR, input->Parameter.State) !=
    // VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_TWDFR, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that track width front parameter is within range */
    if ((input->Parameter.VED_Kf_TrackWidthFront_met < MINIMUM_TRACK_WIDTH) ||
        (input->Parameter.VED_Kf_TrackWidthFront_met > MAXIMUM_TRACK_WIDTH)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.VED_Kf_TrackWidthFront_met = DEFAULT_TRACK_WIDTH;
        // VED_SET_IO_STATE(VED_PAR_POS_TWDFR, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }

    /* Check that track width rear parameter is valid (might be on a default if
     * not valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_TWDRE, input->Parameter.State) !=
    // VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_TWDRE, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that track width parameter rear is within range */
    if ((input->Parameter.VED_Kf_TrackWidthRear_met < MINIMUM_TRACK_WIDTH) ||
        (input->Parameter.VED_Kf_TrackWidthRear_met > MAXIMUM_TRACK_WIDTH)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.VED_Kf_TrackWidthRear_met = DEFAULT_TRACK_WIDTH;
        // VED_SET_IO_STATE(VED_PAR_POS_TWDRE, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }
#endif

    /* Check that wheel base parameter is valid (might be on a default if not
     * valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_WBASE, input->Parameter.State) !=
    // VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_WBASE, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that wheel base parameter is within range */
    if ((input->Parameter.VED_Kf_WheelBase_met < MINIMUM_WHEEL_BASE) ||
        (input->Parameter.VED_Kf_WheelBase_met > MAXIMUM_WHEEL_BASE)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.VED_Kf_WheelBase_met = DEFAULT_WHEEL_BASE;
        // VED_SET_IO_STATE(VED_PAR_POS_WBASE, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }

    /* Check that steering ratio parameter is valid (might be on a default if
     * not valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_SWARAT, input->Parameter.State) !=
    // VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_SWARAT, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that steering ratio parameter is within range */
    if ((input->Parameter.SteeringRatio.swa.rat[1] < MINIMUM_STEERING_RATIO) ||
        (input->Parameter.SteeringRatio.swa.rat[1] > MAXIMUM_STEERING_RATIO)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.SteeringRatio.swa.rat[1] = DEFAULT_STEERING_RATIO;
        // VED_SET_IO_STATE(VED_PAR_POS_SWARAT, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }

    /* Check that self steering gradient parameter is valid (might be on a
     * default if not valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_SSG, input->Parameter.State) !=
    // VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_SSG, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that steering gradient parameter is within range */
    if ((input->Parameter.VED_Kf_SelfSteerGrad_nu <
         MINIMUM_SELF_STEERING_GRAD) ||
        (input->Parameter.VED_Kf_SelfSteerGrad_nu >
         MAXIMUM_SELF_STEERING_GRAD)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.VED_Kf_SelfSteerGrad_nu = DEFAULT_SELF_STEERING_GRAD;
        // VED_SET_IO_STATE(VED_PAR_POS_SSG, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* Check that wheelticks parameter is valid (might be on a default if not
     * valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_WTCKSREV, input->Parameter.State)
    // != VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_WTCKSREV, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that wheelticks parameter is within range */
    if ((input->Parameter.VED_Ku_WhlTcksPerRev_nu < MINIMUM_WHEEL_TICKS_REV) ||
        (input->Parameter.VED_Ku_WhlTcksPerRev_nu > MAXIMUM_WHEEL_TICKS_REV)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.VED_Ku_WhlTcksPerRev_nu = DEFAULT_WHEEL_TICKS_REV;
        // VED_SET_IO_STATE(VED_PAR_POS_WTCKSREV, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }
#endif

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* Check that center of gravity height parameter is valid (might be on a
     * default if not valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_COGH, input->Parameter.State) !=
    // VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_COGH, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that center of gravity height parameter is within range */
    if ((input->Parameter.VED_Kf_CntrOfGravHeight_met < MINIMUM_COGH) ||
        (input->Parameter.VED_Kf_CntrOfGravHeight_met > MAXIMUM_COGH)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.VED_Kf_CntrOfGravHeight_met = DEFAULT_COGH;
        // VED_SET_IO_STATE(VED_PAR_POS_COGH, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }
#endif

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* Check that wheel circumference parameter is valid (might be on a default
     * if not valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_WHLCIR, input->Parameter.State) !=
    // VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_WHLCIR, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that wheel circumference parameter is within range */
    if ((input->Parameter.VED_Kf_WhlCircumference_met < MINIMUM_WHEEL_CIRCUM) ||
        (input->Parameter.VED_Kf_WhlCircumference_met > MAXIMUM_WHEEL_CIRCUM)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.VED_Kf_WhlCircumference_met = DEFAULT_WHEEL_CIRCUM;
        // VED_SET_IO_STATE(VED_PAR_POS_WHLCIR, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }
#endif

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* Check that axle load distribution parameter is valid (might be on a
     * default if not valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_AXLD, input->Parameter.State) !=
    // VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_AXLD, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that axle load distribution parameter is within range */
    if ((input->Parameter.VED_Kf_AxisLoadDistr_per < MINIMUM_AXLE_LOAD_DISTR) ||
        (input->Parameter.VED_Kf_AxisLoadDistr_per > MAXIMUM_AXLE_LOAD_DISTR)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.VED_Kf_AxisLoadDistr_per = DEFAULT_AXLE_LOAD_DISTR;
        // VED_SET_IO_STATE(VED_PAR_POS_AXLD, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }
#endif

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* Check that wheel load dependency front parameter is valid (might be on a
     * default if not valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_WHLDFR, input->Parameter.State) !=
    // VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_WHLDFR, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that wheel load dependency front parameter is within range */
    if ((input->Parameter.VED_Kf_WhlLoadDepFrontAxle_nu <
         MINIMUM_WHEEL_LOAD_DEP) ||
        (input->Parameter.VED_Kf_WhlLoadDepFrontAxle_nu >
         MAXIMUM_WHEEL_LOAD_DEP)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.VED_Kf_WhlLoadDepFrontAxle_nu = DEFAULT_WHEEL_LOAD_DEP;
        // VED_SET_IO_STATE(VED_PAR_POS_WHLDFR, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }
#endif

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* Check that wheel load dependency rear parameter is valid (might be on a
     * default if not valid) */
    // if (    (VED_GET_IO_STATE(VED_PAR_POS_WHLDRE, input->Parameter.State) !=
    // VED_IO_STATE_VALID)
    //     && (VED_GET_IO_STATE(VED_PAR_POS_WHLDRE, input->Parameter.State) !=
    //     VED_IO_STATE_SUBSTITUE)  )
    //{
    //  ParameterError = VED__STAT_ACTIVE;
    //}
    /* Check that wheel load dependency rear parameter is within range */
    if ((input->Parameter.VED_Kf_WhlLoadDepRearAxle_nu <
         MINIMUM_WHEEL_LOAD_DEP) ||
        (input->Parameter.VED_Kf_WhlLoadDepRearAxle_nu >
         MAXIMUM_WHEEL_LOAD_DEP)) {
        ParameterError = VED__STAT_ACTIVE;
        input->Parameter.VED_Kf_WhlLoadDepRearAxle_nu = DEFAULT_WHEEL_LOAD_DEP;
        // VED_SET_IO_STATE(VED_PAR_POS_WHLDRE, VED_IO_STATE_SUBSTITUE,
        // input->Parameter.State);
    }
#endif

    /* Set parameter error in input parameter struct */
    if (ParameterError == VED__STAT_ACTIVE) {
        VED_Errors->ParInputErrors.InputParameterError = VED_ERR_STATE_ACTIVE;
    } else {
        VED_Errors->ParInputErrors.InputParameterError = VED_ERR_STATE_INACTIVE;
    }
}

/* **********************************************************************
  @fn               VED_MonitorInputSignals */ /*!
  @brief            Monitor input signals and parameters

  @description      Does plausibility check of wheel speeds and vehicle velocity
                    (they might be set to a negative value, so the absolute values are used)
                    Checks the ranges of these input signals:
                      yaw rate
                      steering wheel angle
                      all 4 wheel velocities
                      lateral acceleration
                      longitudinal acceleration
                      vehicle velocity
                    If there is a range violation, the signal is limited
                    to the maximum allowed value, a SignalFault is set and
                    the corresponding signals set to degraded state

  @param[in]        input input signals
  @param[out]       VED_Errors signal fault state
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_MonitorInputSignals(VED_InputData_t *input,
                                    VED_Errors_t *VED_Errors) {
    uint8 SignalError = VED__STAT_INACTIVE;

    /* Check yaw rate */
    if (VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        /* Check if allowed maximum value is exceeded */
        if (input->Signals.YawRate > MAX_YAWRATE) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.YawRate = MAX_YAWRATE;
            VED_SET_IO_STATE(VED_SIN_POS_YWR, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        } else if (input->Signals.YawRate < -MAX_YAWRATE) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.YawRate = -MAX_YAWRATE;
            VED_SET_IO_STATE(VED_SIN_POS_YWR, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        } else {
            /* nothing to do */
        }
    }

#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
    /* Check steering angle */
    if (VED_GET_IO_STATE(VED_SIN_POS_SWA, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        /* Check if allowed maximum value is exceeded */
        if (input->Signals.StWheelAngle >
            (MAX_STEERING_ANGLE * input->Parameter.SteeringRatio.swa.rat[1])) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.StWheelAngle =
                (MAX_STEERING_ANGLE *
                 input->Parameter.SteeringRatio.swa.rat[1]);
            VED_SET_IO_STATE(VED_SIN_POS_SWA, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        } else if (input->Signals.StWheelAngle <
                   -(MAX_STEERING_ANGLE *
                     input->Parameter.SteeringRatio.swa.rat[1])) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.StWheelAngle = -(
                MAX_STEERING_ANGLE * input->Parameter.SteeringRatio.swa.rat[1]);
            VED_SET_IO_STATE(VED_SIN_POS_SWA, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        } else {
            /* nothing to do */
        }
    }
#endif

    /* Check Rear steering angle */
    if (VED_GET_IO_STATE(VED_SIN_POS_RSTA, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        /* Check if allowed maximum value is exceeded */
        if (input->Signals.RearWhlAngle >
            (MAX_STEERING_ANGLE * input->Parameter.SteeringRatio.swa.rat[1])) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.RearWhlAngle =
                (MAX_STEERING_ANGLE *
                 input->Parameter.SteeringRatio.swa.rat[1]);
            VED_SET_IO_STATE(VED_SIN_POS_RSTA, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        } else if (input->Signals.RearWhlAngle <
                   -(MAX_STEERING_ANGLE *
                     input->Parameter.SteeringRatio.swa.rat[1])) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.RearWhlAngle = -(
                MAX_STEERING_ANGLE * input->Parameter.SteeringRatio.swa.rat[1]);
            VED_SET_IO_STATE(VED_SIN_POS_RSTA, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        } else {
            /* nothing to do */
        }
    }

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* Check wheel velocity front left */
    input->Signals.WhlVelFrLeft = fABS(input->Signals.WhlVelFrLeft);
    if (VED_GET_IO_STATE(VED_SIN_POS_WVEL_FL, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        if (input->Signals.WhlVelFrLeft > MAX_VELOCITY) {
            input->Signals.WhlVelFrLeft = MAX_VELOCITY;
            SignalError = VED__STAT_ACTIVE;
            VED_SET_IO_STATE(VED_SIN_POS_WVEL_FL, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        }
    }

    /* Check wheel velocity front right */
    input->Signals.WhlVelFrRight = fABS(input->Signals.WhlVelFrRight);
    if (VED_GET_IO_STATE(VED_SIN_POS_WVEL_FR, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        if (input->Signals.WhlVelFrRight > MAX_VELOCITY) {
            input->Signals.WhlVelFrRight = MAX_VELOCITY;
            SignalError = VED__STAT_ACTIVE;
            VED_SET_IO_STATE(VED_SIN_POS_WVEL_FR, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        }
    }

    /* Check wheel velocity rear left */
    input->Signals.WhlVelReLeft = fABS(input->Signals.WhlVelReLeft);
    if (VED_GET_IO_STATE(VED_SIN_POS_WVEL_RL, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        if (input->Signals.WhlVelReLeft > MAX_VELOCITY) {
            input->Signals.WhlVelReLeft = MAX_VELOCITY;
            SignalError = VED__STAT_ACTIVE;
            VED_SET_IO_STATE(VED_SIN_POS_WVEL_RL, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        }
    }

    /* Check wheel velocity rear right */
    input->Signals.WhlVelReRight = fABS(input->Signals.WhlVelReRight);
    if (VED_GET_IO_STATE(VED_SIN_POS_WVEL_RR, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        if (input->Signals.WhlVelReRight > MAX_VELOCITY) {
            input->Signals.WhlVelReRight = MAX_VELOCITY;
            SignalError = VED__STAT_ACTIVE;
            VED_SET_IO_STATE(VED_SIN_POS_WVEL_RR, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        }
    }
#endif

#if ((defined(CFG_VED__USE_EX_LONG_VELO)) && (CFG_VED__USE_EX_LONG_VELO))
    /* Check vehicle velocity */
    input->Signals.VehVelocityExt = fABS(input->Signals.VehVelocityExt);
    if (VED_GET_IO_STATE(VED_SIN_POS_VEHVEL_EXT, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        /* Check if allowed maximum value is exceeded */
        if (input->Signals.VehVelocityExt > MAX_VELOCITY) {
            input->Signals.VehVelocityExt = MAX_VELOCITY;
            SignalError = VED__STAT_ACTIVE;
            VED_SET_IO_STATE(VED_SIN_POS_VEHVEL_EXT, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        }
    }
#endif

#if ((defined(CFG_VED__USE_EX_LONG_ACCEL)) && (CFG_VED__USE_EX_LONG_ACCEL))
    /* Check longitudinal acceleration */
    if (VED_GET_IO_STATE(VED_SIN_POS_VEHACL_EXT, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        /* Check if allowed maximum value is exceeded */
        if (input->Signals.VehLongAccelExt > MAX_ACCELERATION) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.VehLongAccelExt = MAX_ACCELERATION;
            VED_SET_IO_STATE(VED_SIN_POS_VEHACL_EXT, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        }
        if (input->Signals.VehLongAccelExt < -MAX_ACCELERATION) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.VehLongAccelExt = -MAX_ACCELERATION;
            VED_SET_IO_STATE(VED_SIN_POS_VEHACL_EXT, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        }
    }
#endif

#if ((defined(CFG_VED__BMW_LONG_ACCEL_MODEL)) && \
     (CFG_VED__BMW_LONG_ACCEL_MODEL))
    /* Check longitudinal acceleration */
    if (VED_GET_IO_STATE(VED_SIN_POS_LONGA, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        /* Check if allowed maximum value is exceeded */
        if (input->Signals.LongAccel > MAX_ACCELERATION) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.LongAccel = MAX_ACCELERATION;
            VED_SET_IO_STATE(VED_SIN_POS_LONGA, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        }
        if (input->Signals.LongAccel < -MAX_ACCELERATION) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.LongAccel = -MAX_ACCELERATION;
            VED_SET_IO_STATE(VED_SIN_POS_LONGA, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        }
    }
#endif

#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
    /* Check lateral acceleration */
    if (VED_GET_IO_STATE(VED_SIN_POS_LATA, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        /* Check if allowed maximum value is exceeded */
        if (input->Signals.LatAccel > MAX_ACCELERATION) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.LatAccel = MAX_ACCELERATION;
            VED_SET_IO_STATE(VED_SIN_POS_LATA, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        }
        if (input->Signals.LatAccel < -MAX_ACCELERATION) {
            SignalError = VED__STAT_ACTIVE;
            input->Signals.LatAccel = -MAX_ACCELERATION;
            VED_SET_IO_STATE(VED_SIN_POS_LATA, VED_IO_STATE_SUBSTITUE,
                             input->Signals.State);
        }
    }
#endif

    /* Set signal error in input signal struct */
    if (SignalError == VED__STAT_ACTIVE) {
        VED_Errors->SignalInputErrors.InputSignalError = VED_ERR_STATE_ACTIVE;
    } else {
        VED_Errors->SignalInputErrors.InputSignalError = VED_ERR_STATE_INACTIVE;
    }
}

/* **********************************************************************
  @fn               VED_CheckSignalPeakError */ /*!
  @brief            Monitors input signal for peak errors

  @description      All input signals are analysed and if it is
                    considered active, the Input Signal Peak Error
                    is set to active, the signal state is set to decreased
                    and the signal is limited to the value
                    with the maximum allowed gradient

  @param[in/out]    pInputSignal : Input Signal of the Vehicle Dynamics Module
  @param[in/out]    pLastInputSignal : Input Signal of prev cycle the Vehicle Dynamics Module
  @param[in]        InputSignalState : Input Signal state of the Vehicle Dynamics Module
  @param[in/out]    pLastInputSignalState : Input Signal state of prev cycle the Vehicle Dynamics Module
  @param[in]        Signal : Signal index used for reading from the state matrix
  @param[in]        Threshold : Peak check threshold for this input signal
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
#if (VED_VEH_DYN_INTFVER <= 5)
static uint8 VED_CheckSignalPeakError(float32 *pInputSignal,
                                      float32 *pLastInputSignal,
                                      uint32 const *pInputSignalState,
                                      uint32 *pLastInputSignalState,
                                      uint32 Signal,
                                      float32 Threshold)
#else
static uint8 VED_CheckSignalPeakError(float32 *pInputSignal,
                                      float32 *pLastInputSignal,
                                      uint8 pInputSignalState[],
                                      uint8 pLastInputSignalState[],
                                      uint32 Signal,
                                      float32 Threshold)
#endif
{
    uint8 SignalPeakError = VED__STAT_INACTIVE;
    float32 fDiff;

    /* check for signal peaks between the current and the last cycle */
    fDiff = *pInputSignal - *pLastInputSignal;
    if ((fABS(fDiff) > Threshold) &&
        (VED_GET_IO_STATE(Signal, pInputSignalState) == VED_IO_STATE_VALID) &&
        (VED_GET_IO_STATE(Signal, pLastInputSignalState) ==
         VED_IO_STATE_VALID)) {
        /* Limit the signal gain to the threshold */
        if (fDiff > 0.0F) {
            *pLastInputSignal = *pLastInputSignal + Threshold;
        } else {
            *pLastInputSignal = *pLastInputSignal - Threshold;
        }
        *pInputSignal = *pLastInputSignal;
        SignalPeakError = VED__STAT_ACTIVE;
        VED_SET_IO_STATE(Signal, VED_IO_STATE_DECREASED, pInputSignalState);
    } else {
        *pLastInputSignal = *pInputSignal;
    }
    /* store the signal state of the previous cycle signal value */
    VED_SET_IO_STATE(Signal, VED_GET_IO_STATE(Signal, pInputSignalState),
                     pLastInputSignalState);

    return SignalPeakError;
}

/* **********************************************************************
  @fn               VED_InitForCheckSignalPeakErrors */ /*!
  @brief            Monitors input signals for peak errors

  @description      All input signals are analysed and if any of them
                    are considered active the Input Signal Peak Error
                    is set to active

  @param[in]        pInput : Input of the Vehicle Dynamics Module
  @param[out]       pVED_Errors : Pointer to Peak errors in input signal struct
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_InitForCheckSignalPeakErrors(const VED_InputData_t *pInput) {
    LastInputSignals.YawRate = pInput->Signals.YawRate;
    VED_SET_IO_STATE(VED_SIN_POS_YWR,
                     VED_GET_IO_STATE(VED_SIN_POS_YWR, pInput->Signals.State),
                     LastInputSignals.State);
    LastInputSignals.StWheelAngle = pInput->Signals.StWheelAngle;
    VED_SET_IO_STATE(VED_SIN_POS_SWA,
                     VED_GET_IO_STATE(VED_SIN_POS_SWA, pInput->Signals.State),
                     LastInputSignals.State);
    LastInputSignals.VehVelocityExt = pInput->Signals.VehVelocityExt;
    VED_SET_IO_STATE(
        VED_SIN_POS_VEHVEL_EXT,
        VED_GET_IO_STATE(VED_SIN_POS_VEHVEL_EXT, pInput->Signals.State),
        LastInputSignals.State);
    LastInputSignals.LatAccel = pInput->Signals.LatAccel;
    VED_SET_IO_STATE(VED_SIN_POS_LATA,
                     VED_GET_IO_STATE(VED_SIN_POS_LATA, pInput->Signals.State),
                     LastInputSignals.State);
    LastInputSignals.WhlVelFrLeft = pInput->Signals.WhlVelFrLeft;
    VED_SET_IO_STATE(
        VED_SIN_POS_WVEL_FL,
        VED_GET_IO_STATE(VED_SIN_POS_WVEL_FL, pInput->Signals.State),
        LastInputSignals.State);
    LastInputSignals.WhlVelFrRight = pInput->Signals.WhlVelFrRight;
    VED_SET_IO_STATE(
        VED_SIN_POS_WVEL_FR,
        VED_GET_IO_STATE(VED_SIN_POS_WVEL_FR, pInput->Signals.State),
        LastInputSignals.State);
    LastInputSignals.WhlVelReLeft = pInput->Signals.WhlVelReLeft;
    VED_SET_IO_STATE(
        VED_SIN_POS_WVEL_RL,
        VED_GET_IO_STATE(VED_SIN_POS_WVEL_RL, pInput->Signals.State),
        LastInputSignals.State);
    LastInputSignals.WhlVelReRight = pInput->Signals.WhlVelReRight;
    VED_SET_IO_STATE(
        VED_SIN_POS_WVEL_RR,
        VED_GET_IO_STATE(VED_SIN_POS_WVEL_RR, pInput->Signals.State),
        LastInputSignals.State);
    LastInputSignals.VehLongAccelExt = pInput->Signals.VehLongAccelExt;
    VED_SET_IO_STATE(
        VED_SIN_POS_VEHACL_EXT,
        VED_GET_IO_STATE(VED_SIN_POS_VEHACL_EXT, pInput->Signals.State),
        LastInputSignals.State);
    LastInputSignals.RearWhlAngle = pInput->Signals.RearWhlAngle;
    VED_SET_IO_STATE(VED_SIN_POS_RSTA,
                     VED_GET_IO_STATE(VED_SIN_POS_RSTA, pInput->Signals.State),
                     LastInputSignals.State);
#if ((defined(CFG_VED__BMW_LONG_ACCEL_MODEL)) && \
     (CFG_VED__BMW_LONG_ACCEL_MODEL))
    LastInputSignals.LongAccel = pInput->Signals.LongAccel;
    VED_SET_IO_STATE(VED_SIN_POS_LONGA,
                     VED_GET_IO_STATE(VED_SIN_POS_LONGA, pInput->Signals.State),
                     LastInputSignals.State);
#endif
}

/* **********************************************************************
  @fn               VED_CheckForSignalPeakErrors */ /*!
  @brief            Monitors input signals for peak errors

  @description      All input signals are analysed and if any of them
                    are considered active the Input Signal Peak Error
                    is set to active

  @param[in]        pInput : Input of the Vehicle Dynamics Module
  @param[out]       pVED_Errors : Pointer to Peak errors in input signal struct
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_CheckForSignalPeakErrors(VED_InputData_t *pInput,
                                         VED_Errors_t *pVED_Errors) {
    uint8 SignalPeakError = VED__STAT_INACTIVE;

    /* check for signal peaks between the actual and the last cycle */
    /* yaw rate */
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
    if (VED_CheckSignalPeakError(
            &pInput->Signals.YawRate, &LastInputSignals.YawRate,
            pInput->Signals.State, LastInputSignals.State, VED_SIN_POS_YWR,
            VED__PAR_YAW_RATE_INPUT_PEAK) == VED__STAT_ACTIVE) {
        SignalPeakError = VED__STAT_ACTIVE;
    } else {
        // nothing to do
    }
#endif

    /* steering wheel angle */
#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
    if (VED_CheckSignalPeakError(
            &pInput->Signals.StWheelAngle, &LastInputSignals.StWheelAngle,
            pInput->Signals.State, LastInputSignals.State, VED_SIN_POS_SWA,
            VED__PAR_SWA_INPUT_PEAK) == VED__STAT_ACTIVE) {
        SignalPeakError = VED__STAT_ACTIVE;
    } else {
        // nothing to do
    }
#endif

    /* Rear steering wheel angle */
    if (VED_CheckSignalPeakError(
            &pInput->Signals.RearWhlAngle, &LastInputSignals.RearWhlAngle,
            pInput->Signals.State, LastInputSignals.State, VED_SIN_POS_RSTA,
            VED__PAR_SWA_INPUT_PEAK) == VED__STAT_ACTIVE) {
        SignalPeakError = VED__STAT_ACTIVE;
    } else {
        // nothing to do
    }

    /* vehicle velocity */
#if ((defined(CFG_VED__USE_EX_LONG_VELO)) && (CFG_VED__USE_EX_LONG_VELO))
    if (VED_CheckSignalPeakError(
            &pInput->Signals.VehVelocityExt, &LastInputSignals.VehVelocityExt,
            pInput->Signals.State, LastInputSignals.State,
            VED_SIN_POS_VEHVEL_EXT,
            VED__PAR_WHEEL_VELO_INPUT_PEAK) == VED__STAT_ACTIVE) {
        SignalPeakError = VED__STAT_ACTIVE;
    } else {
        // nothing to do
    }
#endif

    /* lateral acceleration */
#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
    if (VED_CheckSignalPeakError(
            &pInput->Signals.LatAccel, &LastInputSignals.LatAccel,
            pInput->Signals.State, LastInputSignals.State, VED_SIN_POS_LATA,
            VED__PAR_LAT_ACCEL_INPUT_PEAK) == VED__STAT_ACTIVE) {
        SignalPeakError = VED__STAT_ACTIVE;
    } else {
        // nothing to do
    }
#endif

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* WhlVelFrLeft */
    if (VED_CheckSignalPeakError(
            &pInput->Signals.WhlVelFrLeft, &LastInputSignals.WhlVelFrLeft,
            pInput->Signals.State, LastInputSignals.State, VED_SIN_POS_WVEL_FL,
            VED__PAR_WHEEL_VELO_INPUT_PEAK) == VED__STAT_ACTIVE) {
        SignalPeakError = VED__STAT_ACTIVE;
    } else {
        // nothing to do
    }

    /* WhlVelFrRight */
    if (VED_CheckSignalPeakError(
            &pInput->Signals.WhlVelFrRight, &LastInputSignals.WhlVelFrRight,
            pInput->Signals.State, LastInputSignals.State, VED_SIN_POS_WVEL_FR,
            VED__PAR_WHEEL_VELO_INPUT_PEAK) == VED__STAT_ACTIVE) {
        SignalPeakError = VED__STAT_ACTIVE;
    } else {
        // nothing to do
    }

    /* WhlVelReLeft */
    if (VED_CheckSignalPeakError(
            &pInput->Signals.WhlVelReLeft, &LastInputSignals.WhlVelReLeft,
            pInput->Signals.State, LastInputSignals.State, VED_SIN_POS_WVEL_RL,
            VED__PAR_WHEEL_VELO_INPUT_PEAK) == VED__STAT_ACTIVE) {
        SignalPeakError = VED__STAT_ACTIVE;
    } else {
        // nothing to do
    }

    /* WhlVelReRight */
    if (VED_CheckSignalPeakError(
            &pInput->Signals.WhlVelReRight, &LastInputSignals.WhlVelReRight,
            pInput->Signals.State, LastInputSignals.State, VED_SIN_POS_WVEL_RR,
            VED__PAR_WHEEL_VELO_INPUT_PEAK) == VED__STAT_ACTIVE) {
        SignalPeakError = VED__STAT_ACTIVE;
    } else {
        // nothing to do
    }
#endif

#if ((defined(CFG_VED__USE_EX_LONG_ACCEL)) && (CFG_VED__USE_EX_LONG_ACCEL))
    if (VED_CheckSignalPeakError(
            &pInput->Signals.VehLongAccelExt, &LastInputSignals.VehLongAccelExt,
            pInput->Signals.State, LastInputSignals.State,
            VED_SIN_POS_VEHACL_EXT,
            VED__PAR_LONG_ACCEL_INPUT_PEAK) == VED__STAT_ACTIVE) {
        SignalPeakError = VED__STAT_ACTIVE;
    } else {
        // nothing to do
    }
#endif

#if ((defined(CFG_VED__BMW_LONG_ACCEL_MODEL)) && \
     (CFG_VED__BMW_LONG_ACCEL_MODEL))
    if (VED_CheckSignalPeakError(
            &pInput->Signals.LongAccel, &LastInputSignals.LongAccel,
            pInput->Signals.State, LastInputSignals.State, VED_SIN_POS_LONGA,
            VED__PAR_LONG_ACCEL_INPUT_PEAK) == VED__STAT_ACTIVE) {
        SignalPeakError = VED__STAT_ACTIVE;
    } else {
        // nothing to do
    }
#endif

    /* Set signal Peak error in the input signal struct */
    if (SignalPeakError == VED__STAT_ACTIVE) {
        pVED_Errors->SignalInputErrors.InputSignalPeakError =
            VED_ERR_STATE_ACTIVE;
    } else {
        pVED_Errors->SignalInputErrors.InputSignalPeakError =
            VED_ERR_STATE_INACTIVE;
    }
}

/* **********************************************************************
  @fn               VED_MonitorVelocityOutput */ /*!

  @brief            Check the ved_ velocity output data

  @description      Sets fault if vehicle velocity could not be calculated

  @param[in, out]   proPorts
  @return           void

  @pre              Precondition:  none

  @post             Postcondition: none
**************************************************************************** */
static void VED_MonitorVelocityOutput(const proVEDPrtList_t *proPorts,
                                      const VED_InputData_t *input) {
    if (VED_GET_IO_STATE(VED_SOUT_POS_VEL,
                         proPorts->pVehicleDynamicSignals->State) !=
        VED_IO_STATE_VALID) {
        proPorts->pVED_Errors->OutPutErrors.VED_VEH_VEL_NOT_AVAILABLE =
            VED_ERR_STATE_ACTIVE;
    }
#if ((defined(CFG_VED__VELO_VARIANCE_CHECK)) && (CFG_VED__VELO_VARIANCE_CHECK))
    else {
        if ((fABS(input->Signals.VehLongAccelExt) >
             VED__ACC_THRESHOLD_INC_VELOCITY_VAR) &&
            (proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr
                 .corrVeloVar > VED__VELOCITY_MAX_VARIANCE_DYNAMIC)) {
#if ((defined(CFG_VED__DEBOUNCE_OUTPUTS)) && (CFG_VED__DEBOUNCE_OUTPUTS))
            if (u_Debouce_VED_VEH_VEL_NOT_AVAILABLE >=
                VED__MAX_OUTPUT_DEBOUNCING) {
                proPorts->pVED_Errors->OutPutErrors.VED_VEH_VEL_NOT_AVAILABLE =
                    VED_ERR_STATE_ACTIVE;
            } else {
                (u_Debouce_VED_VEH_VEL_NOT_AVAILABLE)++;
            }
#else
            proPorts->pVED_Errors->OutPutErrors.VED_VEH_VEL_NOT_AVAILABLE =
                VED_ERR_STATE_ACTIVE;
#endif
        }

        else if ((fABS(input->Signals.VehLongAccelExt) <=
                  VED__ACC_THRESHOLD_INC_VELOCITY_VAR) &&
                 (proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr
                      .corrVeloVar > VED__VELOCITY_MAX_VARIANCE)) {
#if ((defined(CFG_VED__DEBOUNCE_OUTPUTS)) && (CFG_VED__DEBOUNCE_OUTPUTS))
            if (u_Debouce_VED_VEH_VEL_NOT_AVAILABLE >=
                VED__MAX_OUTPUT_DEBOUNCING) {
                proPorts->pVED_Errors->OutPutErrors.VED_VEH_VEL_NOT_AVAILABLE =
                    VED_ERR_STATE_ACTIVE;
            } else {
                (u_Debouce_VED_VEH_VEL_NOT_AVAILABLE)++;
            }
#else
            proPorts->pVED_Errors->OutPutErrors.VED_VEH_VEL_NOT_AVAILABLE =
                VED_ERR_STATE_ACTIVE;
#endif
        }

        else {
            proPorts->pVED_Errors->OutPutErrors.VED_VEH_VEL_NOT_AVAILABLE =
                VED_ERR_STATE_INACTIVE;

#if ((defined(CFG_VED__DEBOUNCE_OUTPUTS)) && (CFG_VED__DEBOUNCE_OUTPUTS))
            u_Debouce_VED_VEH_VEL_NOT_AVAILABLE = 0;
#endif
        }
    }
#else

    else {
        proPorts->pVED_Errors->OutPutErrors.VED_VEH_VEL_NOT_AVAILABLE =
            VED_ERR_STATE_INACTIVE;
    }

#endif
}

/* **********************************************************************
  @fn               VED_MonitorYawrateOutput */ /*!

  @brief            Check the ved_ yawrate output data

  @description      Sets fault if yaw rate could not be calculated while
                    vehicle was moving
                    Current fault state will not be changed but kept while vehicle
                    is stationary

  @param[in, out]   proPorts
  @return           void

  @pre              Precondition:  none

  @post             Postcondition: none
**************************************************************************** */
static void VED_MonitorYawrateOutput(const proVEDPrtList_t *proPorts) {
    float32 fVelocity;
    /*check velocity*/
    fVelocity =
        proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrVelo;
    if ((proPorts->pVehicleDynamicSignals->MotionState.MotState ==
         (uint8)VED_LONG_MOT_STATE_MOVE_RWD) &&
        (fVelocity > 0.0F)) {
        fVelocity = -fVelocity;
    }

    /*check yaw rate*/

    if (VED_GET_IO_STATE(VED_SOUT_POS_YWR,
                         proPorts->pVehicleDynamicSignals->State) !=
        VED_IO_STATE_VALID) {
        if (fVelocity > VED__YAWRATE_MIN_VELOCITY) {
            proPorts->pVED_Errors->OutPutErrors.VED__VEH_YWR_NOT_AVAILABLE =
                VED_ERR_STATE_ACTIVE;
        } else {
            /* stay in current mode */
        }
    }
#if ((defined(CFG_VED__YAWRATE_VARIANCE_CHECK)) && \
     (CFG_VED__YAWRATE_VARIANCE_CHECK))
    else {
        if (proPorts->pVehicleDynamicSignals->Lateral.YawRate.Variance >
            VED__YAWRATE_MAX_VARIANCE) {
            if (fVelocity > VED__YAWRATE_MIN_VELOCITY) {
                proPorts->pVED_Errors->OutPutErrors.VED__VEH_YWR_NOT_AVAILABLE =
                    VED_ERR_STATE_ACTIVE;
            } else {
                /* stay in current mode */
            }
        } else {
            proPorts->pVED_Errors->OutPutErrors.VED__VEH_YWR_NOT_AVAILABLE =
                VED_ERR_STATE_INACTIVE;
        }
    }
#else

    else {
        proPorts->pVED_Errors->OutPutErrors.VED__VEH_YWR_NOT_AVAILABLE =
            VED_ERR_STATE_INACTIVE;
    }

#endif
}

#if ((defined(CFG_VED__MON_OUTPUT_PEAKS)) && (CFG_VED__MON_OUTPUT_PEAKS))
/* **********************************************************************
  @fn               VED_MonitorOutputPeaks */ /*!

  @brief            Check and limit curve output value

  @description      Change in curve output is limited to avoid output jumps in signal
                    when switching to yawrate based curve (steering input becomes invalid)
                    Limitation value is based on max. input signal peak

  @param            pInput
  @param            proPorts
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_MonitorOutputPeaks(const VED_InputData_t *pInput,
                                   const proVEDPrtList_t *proPorts) {
    float32 fDiff;
    float32 Threshold;
    uint8 CurrentInputStwState =
        (uint8)VED_GET_IO_STATE(VED_SIN_POS_SWA, pInput->Signals.State);
    uint8 CurrentOutputCurveState = (uint8)VED_GET_IO_STATE(
        VED_SOUT_POS_CURVE, proPorts->pVehicleDynamicSignals->State);

    /* Check if steering wheel input got invalid or decreased this cycle */
    if ((((CurrentInputStwState == VED_IO_STATE_INVALID) ||
          (CurrentInputStwState == VED_IO_STATE_DECREASED)) &&
         (LastOutputSignals.InputStwState == VED_IO_STATE_VALID)) ||
        ((CurrentInputStwState == VED_IO_STATE_VALID) &&
         ((LastOutputSignals.InputStwState == VED_IO_STATE_INVALID) ||
          (LastOutputSignals.InputStwState == VED_IO_STATE_DECREASED)))) {
        LastOutputSignals.LimitCurve = TRUE;
    }

    /* only limit curve if vehicle is moving and the steering wheel input just
     * got invalid */
    if ((proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrVelo >
         C_F32_DELTA) &&
        (LastOutputSignals.LimitCurve == TRUE)) {
        /* steering ratio and wheelbase are both not 0 or a parameter fault
         * would be active, the values would have been set to safe defaults and
         * VED disabled */
        /* but check anyway to be safe */
        if (TUE_CML_IsNonZero(pInput->Parameter.SteeringRatio.swa.rat[1]) &&
            TUE_CML_IsNonZero(pInput->Parameter.VED_Kf_WheelBase_met)) {
            Threshold = VED__PAR_SWA_INPUT_PEAK /
                        (pInput->Parameter.SteeringRatio.swa.rat[1] *
                         pInput->Parameter.VED_Kf_WheelBase_met);

            /* check for signal peaks between the actual and the last cycle */
            fDiff = LastOutputSignals.Curve -
                    proPorts->pVehicleDynamicSignals->Lateral.Curve.Curve;
            if ((fABS(fDiff) > Threshold) &&
                (LastOutputSignals.CurveState == VED_IO_STATE_VALID) &&
                (CurrentOutputCurveState == VED_IO_STATE_VALID)) {
                /* Limit the signal gain to the threshold */
                if (fDiff > 0.0F) {
                    LastOutputSignals.Curve =
                        LastOutputSignals.Curve + Threshold;
                } else {
                    LastOutputSignals.Curve =
                        LastOutputSignals.Curve - Threshold;
                }
                proPorts->pVehicleDynamicSignals->Lateral.Curve.Curve =
                    LastOutputSignals.Curve;
                LastOutputSignals.OutputPeakError = VED__STAT_ACTIVE;
            } else {
                /* Within limits, take value, stop limiting the signal */
                LastOutputSignals.Curve =
                    proPorts->pVehicleDynamicSignals->Lateral.Curve.Curve;
                LastOutputSignals.OutputPeakError = VED__STAT_INACTIVE;
                LastOutputSignals.LimitCurve = FALSE;
            }
        } else {
            /* Store values if vehicle is not moving, no peak check necessary */
            LastOutputSignals.Curve =
                proPorts->pVehicleDynamicSignals->Lateral.Curve.Curve;
            LastOutputSignals.OutputPeakError = VED__STAT_INACTIVE;
        }

        /* store the signal state of the previous cycle */
        LastOutputSignals.CurveState = CurrentOutputCurveState;
        LastOutputSignals.InputStwState = CurrentInputStwState;
    } else {
        /* do nothing as parameters are corrupt */
    }
}
#endif

/* **********************************************************************
  @fn               VED_MonitorOutputRanges */ /*!

  @brief            Check the ved_ output ranges of the yaw rate and velocity data

  @description      Checks calculated output values of yaw rate, velocity and
                    longitudinal acceleration against absolut limits
                    (sanity check)
                    Fault is set if above limit
 
  @param            proPorts
  @return           void

  @pre              Precondition:  none

  @post             Postcondition: none
**************************************************************************** */
static void VED_MonitorOutputRanges(const proVEDPrtList_t *proPorts) {
    /* absolute output values */
    float32 f_VelocityAbs =
        fABS(proPorts->pVehicleDynamicSignals->Longitudinal.MotVar.Velocity);
    float32 f_LongAccelAbs =
        fABS(proPorts->pVehicleDynamicSignals->Longitudinal.MotVar.Accel);
    float32 f_YawRateAbs =
        fABS(proPorts->pVehicleDynamicSignals->Lateral.YawRate.YawRate);
    float32 f_LatAccelAbs =
        fABS(proPorts->pVehicleDynamicSignals->Lateral.Accel.LatAccel);

    /* check longitudinal motion */
    /* if vehicle velocity is in range 0 - +400 kph and vehicle acceleration is
     * below 15 m/s^2 and velocity is not NaN */
    if ((f_VelocityAbs < MAX_VELOCITY) && (f_LongAccelAbs < MAX_ACCELERATION) &&
        (ISNAN(
             proPorts->pVehicleDynamicSignals->Longitudinal.MotVar.Velocity) ==
         FALSE)) {
        proPorts->pVED_Errors->OutPutErrors.VelocityErr =
            VED_ERR_STATE_INACTIVE;

        /* check estimated yaw rate */
        /* Invalid yaw rate input sensor signal during standstill must not lead
         * to invalid output yaw rate */
        if ((VED_GET_IO_STATE(VED_SOUT_POS_YWR,
                              proPorts->pVehicleDynamicSignals->State) !=
             VED_IO_STATE_VALID) &&
            (VED_GET_IO_STATE(VED_SOUT_POS_VEL,
                              proPorts->pVehicleDynamicSignals->State) ==
             VED_IO_STATE_VALID) &&
            (proPorts->pVehicleDynamicSignals->Longitudinal.MotVar.Velocity <
             1.0F)) {
            VED_SET_IO_STATE(VED_SOUT_POS_YWR, VED_IO_STATE_VALID,
                             proPorts->pVehicleDynamicSignals->State);
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
            proPorts->pALNYawRate->bSenYawRateState = TRUE;
#endif
        } else {
            /* nothing to do */
        }
    } else {
        /* Velocity value is out of range or NaN*/
        VED_SET_IO_STATE(VED_SOUT_POS_VEL, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
        VED_SET_IO_STATE(VED_SOUT_POS_ACCEL, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
        proPorts->pVED_Errors->OutPutErrors.VelocityErr = VED_ERR_STATE_ACTIVE;
    }

    /* if yaw rate is in range -150 /s - +150 /s and lateral acceleration is
     * below 15 m /s^2*/
    if ((f_YawRateAbs < MAX_YAWRATE) && (f_LatAccelAbs < MAX_ACCELERATION)) {
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
        proPorts->pALNYawRate->bSenYawRateState = TRUE;
#endif
        proPorts->pVED_Errors->OutPutErrors.YawRateErr = VED_ERR_STATE_INACTIVE;
    } else {
        VED_SET_IO_STATE(VED_SOUT_POS_YWR, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
        VED_SET_IO_STATE(VED_SOUT_POS_LATACC, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
        proPorts->pVED_Errors->OutPutErrors.YawRateErr = VED_ERR_STATE_ACTIVE;
    }
}

/* **********************************************************************
  @fn               VED_MonitorOutput */ /*!

  @brief            Check the ved_ output data if values are in physical range

  @description      Calls the output peak and range check function
                    Calls the yawrate and velocity monitoring function

  @param            input
  @param            proPorts
  @return           void

  @pre              Precondition:  none

  @post             Postcondition: none
**************************************************************************** */
void VED_MonitorOutput(const VED_InputData_t *input,
                       const proVEDPrtList_t *proPorts,
                       const reqVEDPrtList_t *reqPorts) {
#if ((defined(CFG_VED__MON_OUTPUT_PEAKS)) && (CFG_VED__MON_OUTPUT_PEAKS))
    /* monitoring of the curve output gradient */
    VED_MonitorOutputPeaks(input, proPorts);
#else
    (void)input; /* remove compiler warning, input data is not used in this
                    configuration */
#endif
    /* monitoring of the yaw rate/lat accel and velocity/long accel output value
     * range */
    VED_MonitorOutputRanges(proPorts);

    if ((VED_GET_IO_STATE(VEH_SIG_BRAKE_ABS,
                          reqPorts->pVehicleInputSignals->Brake.State) ==
         VED_IO_STATE_VALID) &&
        (reqPorts->pVehicleInputSignals->Brake.ABSCtrl == FALSE)) {
        u_velMonitorOFFCounter_ABS = 0U;
    }
    if ((VED_GET_IO_STATE(VEH_SIG_BRAKE_TSC,
                          reqPorts->pVehicleInputSignals->Brake.State) ==
         VED_IO_STATE_VALID) &&
        (reqPorts->pVehicleInputSignals->Brake.TCSCtrl == FALSE)) {
        u_velMonitorOFFCounter_TSC = 0U;
    }
    /* global monitoring of the velocity output, if the velocity is not
     * sufficient for the other components */
    /* sensor op mode should be changed (temporary not available) */

    /* When ABS / TSC is active set the Vehicle velocity not available error as
     * Inactive for 20 sec */

    if (((VED_GET_IO_STATE(VEH_SIG_BRAKE_ABS,
                           reqPorts->pVehicleInputSignals->Brake.State) ==
          VED_IO_STATE_VALID) &&
         (reqPorts->pVehicleInputSignals->Brake.ABSCtrl == TRUE) &&
         (u_velMonitorOFFCounter_ABS < OUTPUT_VEL_MON_DEACTIVATION_ABS)) ||

        ((VED_GET_IO_STATE(VEH_SIG_BRAKE_TSC,
                           reqPorts->pVehicleInputSignals->Brake.State) ==
          VED_IO_STATE_VALID) &&
         (reqPorts->pVehicleInputSignals->Brake.TCSCtrl == TRUE) &&
         (u_velMonitorOFFCounter_TSC < OUTPUT_VEL_MON_DEACTIVATION_TSC))) {
        proPorts->pVED_Errors->OutPutErrors.VED_VEH_VEL_NOT_AVAILABLE =
            VED_ERR_STATE_INACTIVE;

        if ((VED_GET_IO_STATE(VEH_SIG_BRAKE_ABS,
                              reqPorts->pVehicleInputSignals->Brake.State) ==
             VED_IO_STATE_VALID) &&
            (reqPorts->pVehicleInputSignals->Brake.ABSCtrl == TRUE)) {
            u_velMonitorOFFCounter_ABS++;
        }

        if ((VED_GET_IO_STATE(VEH_SIG_BRAKE_TSC,
                              reqPorts->pVehicleInputSignals->Brake.State) ==
             VED_IO_STATE_VALID) &&
            (reqPorts->pVehicleInputSignals->Brake.TCSCtrl == TRUE)) {
            u_velMonitorOFFCounter_TSC++;
        }

    }

    else {
        VED_MonitorVelocityOutput(proPorts, input);
    }

    /* global monitoring of the yaw rate output, if the yaw rate is not
     * sufficient for the other components */
    /* sensor op mode should be changed (temporary not available) */
    VED_MonitorYawrateOutput(proPorts);
}

#if ((!defined(CFG_VED__DIS_YWR_OFFSET_COMP)) || \
     (!CFG_VED__DIS_YWR_OFFSET_COMP))
/* **********************************************************************
  @fn               VED_MonitorDynYwrOffset */ /*!

  @brief            Monitor the dynamic yaw rate offset, check with the old dynamic offset

  @description      Checks that yawrate offset is not changed more than a threshold value
                    by the dynamic offset correction, keeps old offset value otherwise

  @param            Input
  @param            IntData
  @return           void

  @pre              Precondition:  none

  @post             Postcondition: none
**************************************************************************** */
void VED_MonitorDynYwrOffset(const ToAutocode_t *Input,
                             VED_InternalData_t *IntData) {
    if (((fABS(IntData->ved__offsets_in.ved__yaw_offset.offset -
               Input->OffsData.offset) > DEG2RAD(0.1F)) &&
         (Input->OffsData.state ==
          (uint8)(VED__YAWRATE_STATE_NVM)) /* eeprom offset */) ||
        (Input->OffsData.state ==
         (uint8)(VED__YAWRATE_STATE_STANDSTILL)) /* stand still offset */) {
        (void)memcpy(&IntData->ved__offsets_in.ved__yaw_offset,
                     &Input->OffsData, sizeof(ved__yaw_offset_t));
        /* set old offset to stand still offset */
        oldOffset = Input->OffsData.offset;
        /* store stand still offset */
        StandStillOffset = oldOffset;
    } else {
        (void)memcpy(&IntData->ved__offsets_in.ved__yaw_offset,
                     &Input->OffsData, sizeof(ved__yaw_offset_t));
    }

    if ((fABS(IntData->ved__wye_out.raw_est_yaw_offset -
              IntData->ved__offsets_in.ved__yaw_offset.offset) >
         (float32)(VED__YAWRATE_DYN_MAX_DIFF)) &&
        ((IntData->ved__offsets_in.ved__yaw_offset.state ==
          (uint8)(VED__YAWRATE_STATE_KEEP_TYPE)) &&
         (Input->IsDynamic == FALSE))) {
        /* Set internal data input yaw rate offset temporarily to the old offset
         * (one step byside the stand still offset) */
        IntData->ved__offsets_in.ved__yaw_offset.offset = oldOffset;
        /* reinit the dynamic yaw rate offset state with the oldOffset */
        IntData->ved__offsets_in.ved__yaw_offset.state =
            VED__YAWRATE_STATE_STANDSTILL;
    } else {
        /* Reset internal data input yaw rate offset to original stand still
         * offset */
        IntData->ved__offsets_in.ved__yaw_offset.offset = StandStillOffset;
        /* Set oldoffset to actual estimated dynamic offset */
        oldOffset = IntData->ved__wye_out.gier_yaw_rate_offset;
    }
}
#endif

#if defined(VED__SIMU)
#if ((!defined(CFG_VED__DIS_YWR_OFFSET_COMP)) || \
     (!CFG_VED__DIS_YWR_OFFSET_COMP))
/* **********************************************************************
  @fn               VED_InitDynYwrOffsetMonitor */ /*!

  @brief            Init the dynamic yaw rate offset Monitor

  @description      Initalizes the dynamic montoring data in the simulation

  @param            YwrOffset
  @return           void

  @pre              Precondition:  none

  @post             Postcondition: none
**************************************************************************** */
void VED_InitDynYwrOffsetMonitor(float32 YwrOffset) {
    oldOffset = YwrOffset;
    StandStillOffset = YwrOffset;
}
#endif
#endif

#if (CFG_VED__USE_VELO_MONITOR)
/* **********************************************************************
  @fn                     VED_VelMonGetPrivateData */ /*!
  @brief                  Get access to internal velocity monitoring data

  @description            Returns pointer to velocity monitor data to allow MTS output

  @return                 *VED_YwrSenData_t
  @param[in]              -
  @param[out]             -
  @return           void     

  @pre                    -
  @post                   -
**************************************************************************** */
VED_VelMon_t *VED_VelMonGetPrivateData(void) { return (VMON_GET_ME); }

/* **********************************************************************
  @fn               VED_VelMon */ /*!
  @brief            Monitors velocity for faults

  @description      Compares externally provided velocity against velocity
                    calculated internally from wheel speeds
                    Checks result of fast velocity monitoring
                    Sets output fault states correspondingly

  @param            in
  @param            ve
  @param            pErr
  @return           void        

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelMon(const V1_7_VEDVehSigMain_t *in,
                       const ved__ve_out_t *ve,
                       VED_Errors_t *pErr) {
    fUncertainty_t uncVel;
    VED_VelMon_t *pVelMon = VMON_GET_ME;

    /* Default output error states */
    pErr->OutPutErrors.VelMon = VED_ERR_STATE_UNKNOWN;
    pErr->OutPutErrors.VelMonLt = VED_ERR_STATE_UNKNOWN;

    /* Monitoring only active when signal is valid */
    if (VED_GET_IO_STATE(VED_SIN_POS_VEHVEL_EXT, in->State) ==
        VED_IO_STATE_VALID) {
        static float32 lastVeloVar = 0.0F;
        const float32 ratThrhd_c = 0.01F; /* Ratio to determine threshold by
                                             means of absolute velocity */
        float32 velDiff;
        /* Calculate difference and uncertainty of estimated velocity and check
         * for negative variance */
        velDiff = in->VehVelocityExt - ve->veh_velo;
        if (ve->veh_velo_var < 0.0F) {
            uncVel = 4.0F * VED__SQRT(lastVeloVar);
        } else {
            uncVel = 4.0F * VED__SQRT(ve->veh_velo_var);
            lastVeloVar = ve->veh_velo_var;
        }

        /* if external velocity is outside of estimated velocity range,
         * increment error count */
        if ((fABS(velDiff) - uncVel) > (ve->veh_velo * ratThrhd_c)) {
            const uint32 incErrCnt_c = 1UL; /* Increment step size */
            pVelMon->cntOutSide += incErrCnt_c;
        } else {
            const uint32 decErrCnt_c = 1UL; /* Decrement step size */
            /* decrement error count */
            pVelMon->cntOutSide = (pVelMon->cntOutSide > decErrCnt_c)
                                      ? (pVelMon->cntOutSide - decErrCnt_c)
                                      : 0U;
        }

        /* test counter thresholds */
        if (pVelMon->cntOutSide >= VED__PAR_VMON_CYCLE_OUT_LT) {
            /* Counter above long-term limit */
            pVelMon->cntOutSide =
                MIN(pVelMon->cntOutSide, VED__PAR_VMON_CYCLE_OUT_LT + 2U);
            pErr->OutPutErrors.VelMon = VED_ERR_STATE_INACTIVE;
            pErr->OutPutErrors.VelMonLt = VED_ERR_STATE_ACTIVE;
        } else if (pVelMon->cntOutSide >= VED__PAR_VMON_CYCLE_OUT) {
            /* Counter above short limit */
            pErr->OutPutErrors.VelMon = VED_ERR_STATE_ACTIVE;
            pErr->OutPutErrors.VelMonLt = VED_ERR_STATE_INACTIVE;
        } else {
            pErr->OutPutErrors.VelMon = VED_ERR_STATE_INACTIVE;
            pErr->OutPutErrors.VelMonLt = VED_ERR_STATE_INACTIVE;
        }
    }
    return;
}

/* **********************************************************************
  @fn               VED_VelMonInit */ /*!
  @brief            Initializes Velocity Monitoring

  @description      Sets fault states to unknown and fault counter to 0
    
  @param[out]       pErr output fault structure
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_VelMonInit(VED_Errors_t *pErr) {
    VED_VelMon_t *pVelMon = VMON_GET_ME;

    pErr->OutPutErrors.VelMon = VED_ERR_STATE_UNKNOWN;
    pErr->OutPutErrors.VelMonLt = VED_ERR_STATE_UNKNOWN;

    pVelMon->cntOutSide = 0UL;

    return;
}

/* **********************************************************************
  @fn               VED_VelMonExec */ /*!
  @brief            Calls velocity monitoring routines

  @description      Main function to execute velocity monitoring
                    Inits monitoring data if VED mode is init, otherwise
                    executes monitoring
    
  @param[in]        Input signals 
  @param[in]        ve internal velocity data
  @param[out]       pErr fault outputs
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_VelMonExec(const VED_InputData_t *input,
                    const ved__ve_out_t *ve,
                    VED_Errors_t *pErr) {
#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* if ved_ control mode is init */
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
        VED_VelMonInit(pErr);
    } else {
        VED_VelMon(&input->Signals, ve, pErr);
    }
#endif

#if ((defined(CFG_VED__FS_VELO_CORR_MON)) && (CFG_VED__FS_VELO_CORR_MON))
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
        pErr->OutPutErrors.VED_FS_VEH_CORR_MON = VED_ERR_STATE_UNKNOWN;
    } else {
        VED_FSVelCorrMon_t *pCorrMonData = VED_FSVelCorrMonGetPrivateData();

        /* check if FS velocity monitor detected a fault */
        if (pCorrMonData->fault == VED_ERR_STATE_ACTIVE) {
            pErr->OutPutErrors.VED_FS_VEH_CORR_MON = VED_ERR_STATE_ACTIVE;
        } else {
            pErr->OutPutErrors.VED_FS_VEH_CORR_MON = VED_ERR_STATE_INACTIVE;
        }
    }
#endif
}

#endif

#if (CFG_VED__YWR_OFFSET_MONITOR)
/* **********************************************************************
  @fn                 VED_YwrMonVehHalt */ /*!
  @brief              Monitor amount of the yaw rate at vehicle-halt

  @description        A yaw rate sensor offset failure is detected, if the
                      absolute value of the yaw rate sensor signal at vehicle
                      standstill is higher than a limit for longer time 

  @param[in]          pMon
  @param[in]          pVehHalt
  @param[out]         -
  @return             void

  @pre                -
  @post               -
**************************************************************************** */
static uint8 VED_YwrMonVehHalt(const VED_YwrMonDataLoc_t *pMon,
                               VED_YwrMonVehHalt_t *pVehHalt) {
    uint8 ret = VED_ERR_STATE_UNKNOWN;

    if (pMon->StSt != FALSE) {
        /* Vehice is halted */
        if (fABS(pMon->ywRate) > VED__YWR_MON_VHALT_YWR_MAX) {
            /* Count time for yaw rate at standstill above limit */
            pVehHalt->timeAboveThrhd += pMon->cycTime;
        } else {
            /* Reset counted time */
            pVehHalt->timeAboveThrhd = 0.F;
        }

        /* Vehicle must be standstill for certain time to monitor the yaw rate
         * ofset */
        if (pMon->timeVehHalt > VED__YWR_MON_VHALT_TIME_MAX) {
            if (pVehHalt->timeAboveThrhd > VED__YWR_MON_VHALT_TIME_MAX) {
                /* Period of maximum permitted yaw rate has been elapsed */
                ret = VED_ERR_STATE_ACTIVE;
            } else {
                /* Period of maximum permitted yaw rate has not been elapsed */
                ret = VED_ERR_STATE_INACTIVE;
            }
        }
    } else {
        /* Vehicle is moving, reset monitor state variables */
        pVehHalt->timeAboveThrhd = 0.F;
    }

    /* Keep timer values inside defined limits */
    pVehHalt->timeAboveThrhd = MIN(pVehHalt->timeAboveThrhd,
                                   VED__YWR_MON_VHALT_TIME_MAX + C_F32_DELTA);

    return ret;
}

/* **********************************************************************
  @fn                 VED_YwrMonVehDriveOff */ /*!
  @brief              Monitor amount of the vehicle drive-off

  @description        A yaw rate sensor offset failure is detected, if the
                      absolute value of the yaw rate sensor signal at vehicle
                      standstill was higher than a limit for a minimum time 
                      before the vehicle is driving off

  @param[in]          pMon
  @param[in]          pDrvOff
  @param[out]         -
  @return             void

  @pre                -
  @post               -
**************************************************************************** */
static uint8 VED_YwrMonVehDriveOff(const VED_YwrMonDataLoc_t *pMon,
                                   VED_YwrMonVehDriveOff_t *pDrvOff) {
    uint8 ret = VED_ERR_STATE_UNKNOWN;

    if (pMon->StSt != FALSE) {
        /* Vehice is standstill */
        if (fABS(pMon->ywRate) > VED__YWR_MON_VDOFF_YWR_MAX) {
            /* Count time above max. permitted threshold */
            pDrvOff->timeAboveThrhd += pMon->cycTime;
        } else {
            /* Reset counted time */
            pDrvOff->timeAboveThrhd = 0.F;
        }
    } else {
        /* Vehicle is moving, monitored offset is */
        if (pMon->timeVehHalt > VED__YWR_MON_VDOFF_TIME_MAX) {
            if (pDrvOff->timeAboveThrhd > VED__YWR_MON_VDOFF_TIME_MAX) {
                /* Period of maximum permitted yaw rate has been elapsed */
                ret = VED_ERR_STATE_ACTIVE;
            } else {
                /* Period of maximum permitted yaw rate has not been elapsed */
                ret = VED_ERR_STATE_INACTIVE;
            }
        }

        /* reset monitor state variables */
        pDrvOff->timeAboveThrhd = 0.F;
    }

    /* Keep timer values inside defined limits */
    pDrvOff->timeAboveThrhd =
        MIN(pDrvOff->timeAboveThrhd, VED__YWR_MON_VDOFF_TIME_MAX + C_F32_DELTA);

    return ret;
}

/* **********************************************************************
  @fn                 VED_YwrMonVehHaltDrv */ /*!
  @brief              Comparison of the yaw rate at the start and at the
                      end of the vehicle standstill

  @description        At vehicle-halt the current is intermediate-stored, 
                      when the yaw rate sensor signal gets constant. At the
                      driving away the measured yaw rate is compared with 
                      the previously stored value. A yaw rate sensor offset
                      failure is stored, if the difference between both value
                      is greater than limit

  @param[in]          -
  @param[out]         -
  @return             void

  @pre                -
  @post               -
**************************************************************************** */
static uint8 VED_YwrMonVehHaltDrv(const VED_YwrMonDataLoc_t *pMon,
                                  VED_YwrMonVehHaltDrv_t *pVehHtDrv) {
    uint8 ret = VED_ERR_STATE_UNKNOWN;

    if (pMon->StSt != FALSE) {
        /* Vehicle is standing still */

        if ((VED_YwrMonVehHaltDrvState_t)pVehHtDrv->State ==
            (VED_YwrMonVehHaltDrvState_t)VED__YWR_MON_STATE_HALTING) {
            /* Acquiring offset value at standstill begin */
            if (pVehHtDrv->ivSample.Volume < VED__YWR_MON_VHTDR_WIN_LEN) {
                VED_StatIntervalAdd(&pVehHtDrv->ivSample, pMon->ywRate, 1.0F);
            } else {
                /* Sampling intervall is full, evaluate the data */
                VED_StatIntervalMeanDev(&pVehHtDrv->ivSample);

                /* Use the mean as offset, if the sampled data has no big
                 * variance */
                if (pVehHtDrv->ivSample.Dev < VED__YWR_MON_VHTDR_WIN_DEV) {
                    pVehHtDrv->ywrStStOn = pVehHtDrv->ivSample.Mean;
                    pVehHtDrv->State = (VED_YwrMonVehHaltDrvState_t)
                        VED__YWR_MON_STATE_STANDING;
                }
                /* Clear grabbed data */
                VED_StatIntervalInit(&pVehHtDrv->ivSample);
            }
        } else {
            /* Acquiring offset value for coming standstill end */

            if (pVehHtDrv->ivSample.Volume < VED__YWR_MON_VHTDR_WIN_LEN) {
                VED_StatIntervalAdd(&pVehHtDrv->ivSample, pMon->ywRate, 1.0F);
            } else {
                /* Sampling intervall is full, evaluate the data */
                VED_StatIntervalMeanDev(&pVehHtDrv->ivSample);

                /* Use the mean as offset, if the sampled data has no big
                 * variance */
                if (pVehHtDrv->ivSample.Dev < VED__YWR_MON_VHTDR_WIN_DEV) {
                    pVehHtDrv->ywrStStOff = pVehHtDrv->ivSample.Mean;
                    pVehHtDrv->State =
                        (VED_YwrMonVehHaltDrvState_t)VED__YWR_MON_STATE_WAITING;
                }
                /* Clear grabbed data */
                VED_StatIntervalInit(&pVehHtDrv->ivSample);
            }
        }
    } else {
        /* Monitoring only possible if values at vehicle-halt and vehicle
         * drive-off are available */
        if ((uint8)pVehHtDrv->State == (uint8)VED__YWR_MON_STATE_WAITING) {
            /* Test if difference exceeds allowed value */
            if (fABS(pVehHtDrv->ywrStStOn - pVehHtDrv->ywrStStOff) >
                VED__YWR_MON_VHTDR_DIFF_MAX) {
                ret = VED_ERR_STATE_ACTIVE;
            } else {
                ret = VED_ERR_STATE_INACTIVE;
            }
        }

        /* reset monitor state variables */
        VED_StatIntervalInit(&pVehHtDrv->ivSample);
        pVehHtDrv->State =
            (VED_YwrMonVehHaltDrvState_t)VED__YWR_MON_STATE_HALTING;
    }
    return ret;
}

/* **********************************************************************
  @fn                 VED_YwrMonVehHaltCal */ /*!
  @brief              Monitor yaw rate calibration during vehicle standstill

  @description        Monitor during vehicle standstill if it possible to 
                      determine zero point yaw rate offset in given time.

  @param[in]          pMon
  @param[in]          pVehHaltCal
  @param[out]         -
  @return             void

  @pre                -
  @post               -
**************************************************************************** */
static uint8 VED_YwrMonVehHaltCal(const VED_YwrMonDataLoc_t *pMon,
                                  VED_YwrMonVehHaltCal_t *pVehHaltCal) {
    const float32 CycTime_c = 0.020F;
    const uint8 maxCalCyc_c = (uint8)2UL;

    uint8 ret = VED_ERR_STATE_UNKNOWN;

    /* Enable active error reporting once, if velocity has exceeded threshold */
    if (pVehHaltCal->State == VED__YWR_MON_CAL_STATE_MUTE) {
        if (pMon->Velocity > VED__YWR_MON_CAL_VEL_START_THRHD) {
            pVehHaltCal->State = VED__YWR_MON_CAL_STATE_ARMED;
        }
    }

    /* Test for active vehicle halt */
    if ((pMon->VehStSt != FALSE) &&
        (pMon->Velocity < VED__YWR_MON_VEL_STST_THRHD)) {
        /* Increase time since reported vehicle standstill */
        pVehHaltCal->timeStStToCalib += pMon->cycTime;

        /* Test for standstill appropriate for calibration */
        if (pMon->StSt != FALSE) {
            /* Sample yaw rate values */
            VED_StatIntervalAdd(&pVehHaltCal->ivSample, pMon->ywRate, 1.0F);

            /* Evaluate sampled yaw rate if enough samples have been acquired */
            if (pVehHaltCal->ivSample.Volume >=
                (VED__PAR_YWR_STST_CAL_TIME_MIN / CycTime_c)) {
                /* Calculte mean and standard deviation of observed interval */
                VED_StatIntervalMeanDev(&pVehHaltCal->ivSample);

                /* Increase number of successfull executed calibration cycles,
                 * if deviation is small */
                if ((pVehHaltCal->ivSample.Dev < STANDST_STDABW_MAX) &&
                    (pVehHaltCal->cntCalCycle < (uint8)maxCalCyc_c)) {
                    pVehHaltCal->cntCalCycle++;
                }
                /* Init sample interval */
                VED_StatIntervalInit(&pVehHaltCal->ivSample);
            }
        }
    } else {
        /* Reset time since vehicle standstill */
        pVehHaltCal->timeStStToCalib = 0.F;

        /* In case of moving vehicle init collected data */
        VED_StatIntervalInit(&pVehHaltCal->ivSample);
        pVehHaltCal->cntCalCycle = (uint8)0UL;
    }

    /* After certain time in vehicle halt, successfull calibration is expected
     */
    if (pVehHaltCal->timeStStToCalib > VED__YWR_MON_CAL_MAX_TIME) {
        if (pVehHaltCal->cntCalCycle < (uint8)maxCalCyc_c) {
            /* Calibration was not completed */

            /* Report error active error only, after first time exceeding
             * velocity threshold */
            if (pVehHaltCal->State == VED__YWR_MON_CAL_STATE_ARMED) {
                ret = VED_ERR_STATE_ACTIVE;
            }
        } else {
            /* Calibration was successfull */
            ret = VED_ERR_STATE_INACTIVE;
        }

        /* Disable active error reporting, wait for exceeding velocity threshold
         */
        pVehHaltCal->State = VED__YWR_MON_CAL_STATE_MUTE;

        /* Limit maximum value and keep it above thershold */
        pVehHaltCal->timeStStToCalib = VED__YWR_MON_CAL_MAX_TIME + CycTime_c;
    }

    return ret;
}

/* **********************************************************************
  @fn                 VED_YwrMonInit */ /*!
  @brief              Yaw rate zero point monitoring initialization

  @description        A yaw rate sensor offset failure is detected, if the
                      zero point offset of the yaw rate exceeds during 
                      standstill

  @param[in]          input  module interface and fault pointer
  @param[in]          mif
  @param[in]          VED_Errors
  @param[out]         -
  @return             void

  @pre                -
  @post               -
**************************************************************************** */
void VED_YwrMonInit(const VED_InputData_t *input,
                    const VED_ModIf_t *mif,
                    VED_Errors_t *VED_Errors) {
    VED_YwrMonData_t *pMonData = YWR_GET_MON_DATA;

    (void)*input;
    (void)*mif;

    pMonData->ywRateOld = 0.F;
    pMonData->Gradient = 0.F;
    pMonData->timeVehHalt = 0.F;

    pMonData->VehDriveOff.timeAboveThrhd = 0.F;
    pMonData->VehHalt.timeAboveThrhd = 0.F;

    pMonData->VehHaltDrv.ywrStStOff = 0.F;
    pMonData->VehHaltDrv.State =
        (VED_YwrMonVehHaltDrvState_t)VED__YWR_MON_STATE_HALTING;
    pMonData->VehHaltDrv.ywrStStOff = 0.F;
    pMonData->VehHaltDrv.ywrStStOn = 0.F;
    VED_StatIntervalInit(&pMonData->VehHaltDrv.ivSample);

    pMonData->VehHaltCal.State = VED__YWR_MON_CAL_STATE_MUTE;
    pMonData->VehHaltCal.timeStStToCalib = 0.F;
    pMonData->VehHaltCal.cntCalCycle = (uint8)0UL;
    VED_StatIntervalInit(&pMonData->VehHaltCal.ivSample);

    /* Initialize error output states */
    VED_Errors->OutPutErrors.YwrMonVehHalt = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.YwrMonVehDOff = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.YwrMonAdmWdrwl = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.YwrMonVehHaltCal = VED_ERR_STATE_UNKNOWN;

    return;
}

/* **********************************************************************
  @fn                 VED_YwrMonExec */ /*!
  @brief              Yaw rate zero point monitoring

  @description        A yaw rate sensor offset failure is detected, if the
                      zero point offset of the yaw rate exceeds during 
                      standstill

  @param[in]          input
  @param[in]          mif
  @param[in]          proPorts
  @param[out]         -
  @return             void

  @pre                -
  @post               -
**************************************************************************** */
void VED_YwrMonExec(const VED_InputData_t *input,
                    const VED_ModIf_t *mif,
                    const proVEDPrtList_t *proPorts) {
    VED_YwrMonDataLoc_t Mon;

    VED_YwrMonData_t *pMonData = YWR_GET_MON_DATA;

    /* Initialize error output states */
    proPorts->pVED_Errors->OutPutErrors.YwrMonVehHalt = VED_ERR_STATE_UNKNOWN;
    proPorts->pVED_Errors->OutPutErrors.YwrMonVehDOff = VED_ERR_STATE_UNKNOWN;
    proPorts->pVED_Errors->OutPutErrors.YwrMonAdmWdrwl = VED_ERR_STATE_UNKNOWN;
    proPorts->pVED_Errors->OutPutErrors.YwrMonVehHaltCal =
        VED_ERR_STATE_UNKNOWN;

#if (CFG_VED__INT_GYRO)
    /* Monitoring of internal yaw rate */
    Mon.ywRate = input->Signals.YawRateInt;
#else
    /* Monitoring of external yaw rate */
    Mon.ywRate = input->Signals.YawRate;
#endif

    /* Get vehicle velocity */
    if (VED_GET_IO_STATE(VED_SOUT_POS_VEL,
                         proPorts->pVehicleDynamicSignals->State) ==
        VED_IO_STATE_VALID) {
        Mon.Velocity = mif->LongMot.VehVelocityCorr;
    } else {
        Mon.Velocity = 0.F;
    }

    if (mif->LongMot.MotState.MotState == VED_LONG_MOT_STATE_STDST) {
        Mon.VehStSt = TRUE;
    } else {
        Mon.VehStSt = FALSE;
    }

    /* Set monitor cycle time */
    Mon.cycTime = VED_GetCycleTime();
    Mon.timeVehHalt = pMonData->timeVehHalt;

    if (VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State) !=
        VED_IO_STATE_VALID) {
        /* Calculate gradient of yaw rate signal */
        pMonData->Gradient =
            VED__IIR_DIFF(Mon.ywRate, pMonData->ywRateOld, pMonData->Gradient,
                          VED__YWR_MON_DIFF_FILT, Mon.cycTime);

        /* Enable monitoring if standstill is detected and yaw rate signal is
         * stable */
        if ((mif->LongMot.MotState.MotState == VED_LONG_MOT_STATE_STDST) &&
            (mif->LongMot.MotState.Confidence > VED__PAR_YWR_STST_CONF_MIN) &&
            (fABS(pMonData->Gradient) < VED__YWR_MON_YWR_GRAD_MAX)) {
            /* Activate standstill flag and count time */
            Mon.StSt = TRUE;
            pMonData->timeVehHalt += Mon.cycTime;
        } else {
            /* reset standstill flag and count time */
            Mon.StSt = FALSE;
            pMonData->timeVehHalt = 0.0F;
        }

        /* Run several yaw rate offset monitors */
        proPorts->pVED_Errors->OutPutErrors.YwrMonVehHalt =
            VED_YwrMonVehHalt(&Mon, &pMonData->VehHalt);
        proPorts->pVED_Errors->OutPutErrors.YwrMonVehDOff =
            VED_YwrMonVehDriveOff(&Mon, &pMonData->VehDriveOff);
        proPorts->pVED_Errors->OutPutErrors.YwrMonAdmWdrwl =
            VED_YwrMonVehHaltDrv(&Mon, &pMonData->VehHaltDrv);
        proPorts->pVED_Errors->OutPutErrors.YwrMonVehHaltCal =
            VED_YwrMonVehHaltCal(&Mon, &pMonData->VehHaltCal);

        /* Keep timer values inside defined limits */
        pMonData->timeVehHalt =
            MIN(pMonData->timeVehHalt, VED__YWR_MON_TIME_MAX);

        /* Save current yaw rate for next cycle */
        pMonData->ywRateOld = Mon.ywRate;
    }
    return;
}

/* **********************************************************************
  @fn                 VED_YwrMonGetPrivateData */ /*!
  @brief              Get data of yaw rate monitoring

  @description        Returns pointer to internal yaw rate monitoring data
                      to allow MTS output of the data

  @param[in]          -
  @param[out]         -
  @return             pointer to data

  @pre                -
  @post               -
**************************************************************************** */
VED_YwrMonData_t *VED_YwrMonGetPrivateData(void) { return (YWR_GET_MON_DATA); }
#endif

#if ((defined(CFG_VED__ALIGNMENT_OFFSET_MONITOR)) && \
     (CFG_VED__ALIGNMENT_OFFSET_MONITOR))
/* **********************************************************************
  @fn               VED_AlignmentYawOffsMonitor */ /*!
  @brief            Monitor the alignment monitoring yaw rate error

  @description      Compares yaw rate offset calculated by VED and ALN
                    Fault is set if difference is above threshold
                    Note: This functionality was never activated or tested
                    in ARS301 based projects
                    Functionality will be removed as obsolete in ARS400

  @param            reqPorts 
  @param            input 
  @param            IntData 
  @param            VED_Errors
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_AlignmentYawOffsMonitor(const reqVEDPrtList_t *reqPorts,
                                 const VED_InputData_t *input,
                                 const VED_InternalData_t *IntData,
                                 VED_Errors_t *VED_Errors) {
    float32 CorrYawRateError = 0.0F;
    float32 DiffYawRateOffset = 0.0F;

    /* check if alignment yaw rate offset contains data */
    if (reqPorts->pAlignEstYawRateOffset->f_YROEStd <
        (float32)(VED__ALN_OFFSET_MON_NO_DATA)) {
        /* Correct filtered ved_ yaw rate offset with alignment yaw rate error*/
        CorrYawRateError =
            IntData->ved__gye_out_filt.raw_est_yaw_offset_filt -
            reqPorts->pAlignEstYawRateOffset->f_YawRateOffsetError;
        /* Difference between corrected filterd yaw rate offset and not filterd
         * yaw rate offset*/
        DiffYawRateOffset =
            fABS(CorrYawRateError - IntData->ved__wye_out.gier_yaw_rate_offset);

        /* if this difference is above the acc threshold set the acc error event
         */
        if (DiffYawRateOffset > DEG2RAD(VED__PAR_ACC_YAW_RATE_OFFS_ERROR)) {
            /* Absolute threshold time */
            ACCThldTime = ACCThldTime + input->Frame.CycleTime;

            /* if threshold time is above parameter set ACC offset error event
             */
            if (ACCThldTime > VED__PAR_ACC_YAW_RATE_OFFS_THLD_TIME) {
                VED_Errors->OutPutErrors.YwrACCMonAlignm = VED_ERR_STATE_ACTIVE;
            } else {
                VED_Errors->OutPutErrors.YwrACCMonAlignm =
                    VED_ERR_STATE_INACTIVE;
            }
        } else {
            ACCThldTime = 0U;
            if (DiffYawRateOffset <=
                DEG2RAD(VED__PAR_ACC_YAW_RATE_OFFS_ERROR_OFF)) {
                VED_Errors->OutPutErrors.YwrACCMonAlignm =
                    VED_ERR_STATE_INACTIVE;
            } else {
                /* use old error value, VED_Out stuct is static */
            }
        }

        /* if this difference is above the cg threshold set the cg error event
         */
        if (DiffYawRateOffset > DEG2RAD(VED__PAR_CG_YAW_RATE_OFFS_ERROR)) {
            /* Absolute threshold time */
            CGThldTime = CGThldTime + input->Frame.CycleTime;

            /* if threshold time is above parameter set CG offset error event */
            if (CGThldTime > VED__PAR_CG_YAW_RATE_OFFS_THLD_TIME) {
                VED_Errors->OutPutErrors.YwrCGMonAlignm = VED_ERR_STATE_ACTIVE;
            } else {
                VED_Errors->OutPutErrors.YwrCGMonAlignm =
                    VED_ERR_STATE_INACTIVE;
            }
        } else {
            CGThldTime = 0U;
            if (DiffYawRateOffset <=
                DEG2RAD(VED__PAR_CG_YAW_RATE_OFFS_ERROR_OFF)) {
                VED_Errors->OutPutErrors.YwrCGMonAlignm =
                    VED_ERR_STATE_INACTIVE;
            } else {
                /* use old error value, VED_Out stuct is static */
            }
        }

    } else {
        /* use old error values, VED_Out stuct is static */
    }
}
#endif

/* **********************************************************************
  @fn               VED_InitMon */ /*!
  @brief            Init Monitoring

  @description      Sets output and internal values to 0
                    Sets fault states to unknown
                    Sets outputs to invalid

  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_InitMon(void) {
    /* Init alignment monitoring offsets */
#if ((defined(CFG_VED__ALIGNMENT_OFFSET_MONITOR)) && \
     (CFG_VED__ALIGNMENT_OFFSET_MONITOR))
    ACCThldTime = 0U;
    CGThldTime = 0U;
#endif

#if ((defined(CFG_VED__MON_OUTPUT_PEAKS)) && (CFG_VED__MON_OUTPUT_PEAKS))
    /* Init the lastoutput signals to safe defaults
       setting CurveState to INVALID will deactivate peak monitoring in the very
       first cycle */
    LastOutputSignals.Curve = 0.0F;
    LastOutputSignals.CurveState = (uint8)VED_IO_STATE_INVALID;
    LastOutputSignals.OutputPeakError = VED__STAT_INACTIVE;
    LastOutputSignals.InputStwState = (uint8)VED_IO_STATE_VALID;
    LastOutputSignals.LimitCurve = FALSE;
#endif

#if ((!defined(CFG_VED__DIS_YWR_OFFSET_COMP)) || \
     (!CFG_VED__DIS_YWR_OFFSET_COMP))
    oldOffset = 0.0F;
    StandStillOffset = 0.0F;
#endif

    u_velMonitorOFFCounter_ABS = 0U;

    u_velMonitorOFFCounter_TSC = 0U;

#if ((defined(CFG_VED__DEBOUNCE_OUTPUTS)) && (CFG_VED__DEBOUNCE_OUTPUTS))
    u_Debouce_VED_VEH_VEL_NOT_AVAILABLE = 0U;
#endif
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */