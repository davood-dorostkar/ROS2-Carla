/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "ved_consts.h"
#include "tue_common_libs.h"
#define VED__DACQ_IF 1L /* Activate interface to internal data */
#include "ved.h"

#if ((defined(CFG_VED__DO_VELOCITY_CORR)) && (CFG_VED__DO_VELOCITY_CORR))

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/* Define velocity ranges and bounds  */
#define VED__CORR_VEL_RANGE_0_MIN (float32)(10.F / C_KMH_MS)
#define VED__CORR_VEL_RANGE_0_MAX (float32)(60.F / C_KMH_MS)
#define VED__CORR_VEL_RANGE_0_MID \
    (float32)((VED__CORR_VEL_RANGE_0_MIN + VED__CORR_VEL_RANGE_0_MAX) * 0.5F)

#define VED__CORR_VEL_RANGE_1_MIN VED__CORR_VEL_RANGE_0_MAX
#define VED__CORR_VEL_RANGE_1_MAX (float32)(140.F / C_KMH_MS)
#define VED__CORR_VEL_RANGE_1_MID \
    (float32)((VED__CORR_VEL_RANGE_1_MIN + VED__CORR_VEL_RANGE_1_MAX) * 0.5F)

#define VED__CORR_VEL_RANGE_2_MIN VED__CORR_VEL_RANGE_1_MAX
#define VED__CORR_VEL_RANGE_2_MAX (float32)(240.F / C_KMH_MS)
#define VED__CORR_VEL_RANGE_2_MID \
    (float32)((VED__CORR_VEL_RANGE_2_MIN + VED__CORR_VEL_RANGE_2_MAX) * 0.5F)

/* Per mill scale factor, use used to scale velocity thresholds befor checking,
 * (ECU-SIL) */
#define VED__CORR_PER_MILL_SCALE (1000.0F)

/* Histogram bin width for velocity ratios ... BINS_CORR_FACTOR_STEP Macro is
 * used in VED instead of VED__CORR_VEL_BIN_WIDTH */
/* #define VED__CORR_VEL_BIN_WIDTH          ((REF_SPEED_MAX_RATIO -
 * REF_SPEED_MIN_RATIO)/((float32) REF_SPEED_NO_BINS-1.F))  */

/* Minium ego velocity range for velocity dependency observability */
#define VED__CORR_OBS_MIN_VEL_RG (float32)(20.F / C_KMH_MS)
/* Standard deviation at non-observeable velocity dependence */
#define VED__CORR_OBS_STD_DEV (0.03F / 3.F)

/* Required number of samples for correction estimation */
#if (VEL_CORR_HIST_STATIONARY_TARGETS)
#define VED__CORR_SPL_INIT (4000UL)   /* Number of samples at init state   */
#define VED__CORR_SPL_NORMAL (8000UL) /* Number of samples at other states */
#elif (VEL_CORR_ALN)
#define VED__CORR_SPL_INIT (200UL)   /* Number of samples at init state   */
#define VED__CORR_SPL_NORMAL (400UL) /* Number of samples at other states */
#endif
#define VED__CORR_SPL_OUTSIDE_LIMITS                                        \
    (0.3F) /* Percentage of samples allowed outside of histogram borders to \
              still evaluate histogram */

/* Maximum allowed absolute correction factor deviation  */
#define VED__CORR_ABS_DEV VED__PAR_VCOR_ABS_DEV
#define VED__CORR_ABS_DEV_START (0.45F)

/* Maximum allowed standard deviation of correction factor */
#define VED__CORR_STD_DEV (0.02F)

/* Correction factor for velocity variance */
#define VED__CORR_VAR_CORRECT_FACTOR (0.1F)

/* Reduction factors for collected  */
#define VED__CORR_RED_FAC_INIT (0.2F)   /* Reudction value at init state */
#define VED__CORR_RED_FAC_NORMAL (0.4F) /* Reduction value at other states */

/* Required deviation to store new value in nv-memory */
#define VED__CORR_NVM_FAC_DEV (0.01F)

/* Init values for correction nodes */
#define VED__CORR_INIT_STD_DEV_START \
    (VED__CORR_ABS_DEV_START / 3.F) /* node standard deviation */
#ifndef VED__CORR_INIT_STD_DEV
#define VED__CORR_INIT_STD_DEV \
    (VED__CORR_ABS_DEV / 3.F) /* node standard deviation */
#endif
#define VED__CORR_INIT_VEL_NODE \
    ((float32)(60.F / C_KMH_MS)) /* node average velocity   */

/* Minimum and maximum allowed state values */
/* Zero velocity intercept limit values */
#define VED__CORR_ABS_ICT_MIN_VAL (1.0F - VED__CORR_ABS_DEV)
#define VED__CORR_ABS_ICT_MAX_VAL (1.0F + VED__CORR_ABS_DEV)

/* Velocity dependency constraints values */
#define VED__CORR_ABS_SLP_MAX_VAL VED__PAR_VCOR_VEL_DEP_MAX
#define VED__CORR_ABS_SLP_MIN_VAL VED__PAR_VCOR_VEL_DEP_MIN

/* Maximum allowed magnitude of long. acceleration  for observability */
#define VED__CORR_OBS_ACCEL_MAX ((float32)(1.0F))

/* Maximum allowed magnitude of long. acceleration  for observability */
#define VED__CORR_OBS_CURVE_MAX ((float32)((1.F / 200.F)))

/* Delay time for re-enter observability of ego velocity */
#define VED__CORR_OBS_DELAY_TIME (1000UL)

/* Correction factor estimator parameters */
#define VCOR_EST_CYCLE_TIME (0.02F)   /*!< Expected estimator cycle time */
#define VCOR_EST_K0_STD (0.1F / 3.0F) /*!< Correction factor noise */
#define VCOR_EST_K1_STD \
    (0.000001F / 3.0F) /*!< Correction factor velocity gradient noise */

#define VCOR_EST_Q_DYN \
    (VCOR_EST_CYCLE_TIME / 1E6F) /*!< Correction factor model dynamic */
#define VCOR_EST_P_INIT_QFACT                                                \
    (4E9F * VED__PAR_VCOR_ABS_DEV) /*!< Estimation coveriance initial factor \
                                      value P_INIT = Q * FACT */
#define R_STD_DEV_FACT                                                   \
    (0.1F / 1.0F) /*!< Measurement noise reduction factor for accumlated \
                     histograms  */

#define VCOR_ECU_ELPSD_TIME_MAX    \
    (float32)(10.0 * 24.0 * 60.0 * \
              60.0) /*!< Maximum ECU running time 10 days         */
#define VCOR_NVM_WRITE_TIME_SPAN \
    (float32)(120.0) /*!< Minimum time span between two nvm writes */

#if (CFG_VED__FS_VELO_CORR_MON)
/* fast velocity correction factor fault monitor options */
/* configuration options */
#if ((defined(VEL_CORR_HIST_STATIONARY_TARGETS)) && \
     (VEL_CORR_HIST_STATIONARY_TARGETS))
#define VEL_CORR_FS_MON_MAX_DEV                                                \
    (0.15F) /* max. deviation of velocity factor (defined in functional safety \
               specification) */
#define VEL_CORR_FS_MON_MAX_COUNTER \
    (5u) /* number of EM histograms used before reduced again (e.g. 5) */
#define VEL_CORR_FS_MON_REDUCTION                                            \
    (0.2F) /* reduction factor, depends on VEL_CORR_FS_MON_MAX_COUNTER (e.g. \
              0.2) */
#define VEL_CORR_FS_MON_MIN_NUMBER \
    (5.0F) /* minimum number of samples to allow evaluation */
#define VEL_CORR_FS_MON_MIN_SA                                              \
    (0.1F) /* maximum allowed standard deviation of summed up histograms to \
              allow evaluation */
#define VEL_CORR_FS_HIST_MAX_WIDTH                                         \
    (19) /* maximum width of input histogram (difference between first and \
            last bin with at least one sample) */
#define VEL_CORR_FS_HIST_MIN_NUMBER \
    (3u) /* minimum number of samples in input histogram */
#elif ((defined(VEL_CORR_ALN)) && (VEL_CORR_ALN))
#define VEL_CORR_FS_MON_MAX_COUNTER \
    (5u) /* number of ALN inputs used before reduced again (e.g. 5) */
#define VED__FS_VEL_MON_SIGMA \
    (4.0F) /* measurement sigma for variance of velocity*/
#define VED__FS_VEL_MON_MAX_VEL_DIFF                                   \
    (2.0F) /* maximum difference of radar target velocity and external \
              velocity in m/s */

#define VED__FS_VEL_CONF_MAX_VEL_DIFF                                         \
    (1.0F) /* maximum difference of radar confirmation velocity and corrected \
              velocity in m/s */
#endif
#endif

#if ((defined(CFG_VED__TRUCK_CORRFACT)) && (CFG_VED__TRUCK_CORRFACT))
#define VED__FS_VEL_MON_RAT_THRHD                                             \
    (0.15F) /* Ratio to determine threshold by means of absolute velocity for \
               TRUCKS*/

#elif ((defined(CFG_VED__CORRFACT_ARS4D2)) && (CFG_VED__CORRFACT_ARS4D2))
#define VED__FS_VEL_MON_RAT_THRHD (0.3F)

#else
#define VED__FS_VEL_MON_RAT_THRHD \
    (0.1F) /* Ratio to determine threshold by means of absolute velocity */

#endif

/*maximum ved_ cycles to deactivate the output Velocity monitor when abs is
 * active */
#ifndef OUTPUT_VEL_VAR_LIMITATION
#define OUTPUT_VEL_VAR_LIMITATION (0.0F)
#endif

#if ((CFG_VED__FS_VELO_CORR_MON) || (VEL_CORR_ALN))
/* non-configurable definitions */
#define BINS_CORR_FACTOR_STEP (0.01F) /* resolution of input histograms */
#define BINS_CORR_FACTOR_START_VALUE                                 \
    (float32)(1.0F - ((((float32)REF_SPEED_NO_BINS - 1.0f) / 2.0f) * \
                      BINS_CORR_FACTOR_STEP))
#endif

/* Dimension of noise covariance matrix for estimator */
#define VED__VCOR_DIM_COVAR_MATRIX (4U)

#if ((defined(VEL_CORR_ALN)) && (VEL_CORR_ALN))
#define VED__VCOR_NO_ALN_DATA_STD_DEV                                          \
    (1E30F) /* set by ALN as standard deviation if no velocity could have been \
               calculated */
#endif

#if ((defined(CFG_VED__VELO_MONITOR_MIN_MAX)) && \
     (CFG_VED__VELO_MONITOR_MIN_MAX))
/*values used for Interpolation to deactivate the VEL_MON and VEL_MONLT DEM */
#define VED__INTERPOL_VEL_RANGE_MIN \
    (float32)(0.F) /* minimum velocity  input for interpolation */
#define VED__INTERPOL_VEL_RANGE_MAX                                          \
    (float32)(100.F / C_KMH_MS) /* Maximum velocity  input for interpolation \
                                 */
#define VED__INTERPOL_ACCEL_RANGE_MIN \
    ((float32)(4.2F)) /* minimum Acceleration  input for interpolation */
#define VED__INTERPOL_ACCEL_RANGE_MAX \
    ((float32)(2.8F)) /* Maximum Acceleration  input for interpolation */
#define VED__DECELERATION_RANGE_MAX \
    ((float32)(-5.0F)) /* Maximum Acceleration  input for interpolation */
#endif

/*****************************************************************************
  MACROS
*****************************************************************************/
#define COR_GET_ME() (&VED_VelCorr)
#define COR_GET_HISTS() (&VED_VelCorr.Hist[0])
#define COR_GET_NODES() (&VED_VelCorr.Node[0])
#define COR_GET_EST() (&VED_VelCorr.Est)
#define COR_SET_ERR_FACT_RG(state_) (*VED_VelCorr.Io.errFactRg = (state_))
#define COR_SET_ERR_WIN(state_) (*VED_VelCorr.Io.errWin = (state_))

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
/*! Map to assign ego velocity ranges */
typedef struct {
    float32 min; /*!< Lower bound velocity of range */
    float32 mid; /*!< Mean velocity of range */
    float32 max; /*!< Upper bound velocity of range */
} VED_VelCorrRgMap_t;

/*****************************************************************************
  CONSTS
*****************************************************************************/

static const VED_VelCorrRgMap_t VED_VelCorrRgMap_c[VED__CORR_VEL_RANGES] = {
    {VED__CORR_VEL_RANGE_0_MIN, VED__CORR_VEL_RANGE_0_MID,
     VED__CORR_VEL_RANGE_0_MAX},
    {VED__CORR_VEL_RANGE_1_MIN, VED__CORR_VEL_RANGE_1_MID,
     VED__CORR_VEL_RANGE_1_MAX},
    {VED__CORR_VEL_RANGE_2_MIN, VED__CORR_VEL_RANGE_2_MID,
     VED__CORR_VEL_RANGE_2_MAX}};

/* Estimator process noise covariance matrix */
static const float32 Q_c[VED__VCOR_DIM_COVAR_MATRIX] = {
    SQR(VCOR_EST_K0_STD) * VCOR_EST_Q_DYN, /*!< K0 variance       */
    0.F,                                   /*!< K0,K1 covariance  */
    0.F,                                   /*!< K1,K0 covariance  */
    SQR(VCOR_EST_K1_STD) * VCOR_EST_Q_DYN  /*!< K1 variance       */
};

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
#if (CFG_VED__DO_VELOCITY_CORR)
SET_MEMSEC_VAR(VED_VelCorr)
static VED_VelCorr_t VED_VelCorr; /*!< @VADDR: 0x2001C000 @VNAME: VED_VelCorr
                                     @ALLOW: ved__priv @cycleid: ved__cycle_id*/
#if (CFG_VED__FS_VELO_CORR_MON)
SET_MEMSEC_VAR(VED_FSVelCorrMon)
static VED_FSVelCorrMon_t
    VED_FSVelCorrMon; /*!< @VADDR: 0x2001C500 @VNAME: VED_FSVelCorrMon @ALLOW:
                         ved__priv @cycleid: ved__cycle_id*/
#endif
#endif
static uint8 cntSampleOld;
static boolean b_init_CorrFlag;

#if ((defined(CFG_VED__VELO_MONITOR_MIN_MAX)) && \
     (CFG_VED__VELO_MONITOR_MIN_MAX))
static uint32 u_debounceCounter_set;
static uint32 u_debounceCounter_Reset;
#endif

#if (ALN_MONITORING_VERSION > 3U) && (CFG_VED__FS_VELO_CORR_MON) && \
    (CFG_VED__FS_VELO_CONF_MON_FAULT)
static uint8 u_ConfCounterOld;
#endif

static uint16 u_velVarianceCounter_ABS;
static uint16 u_velVarianceCounter_TSC;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  LOCAL FUNCTION PROTOTYPES
*****************************************************************************/
static void VED_VelCorrExecEstimator(const VED_VelCorrNode_t *node,
                                     VED_VelCorrEst_t *est,
                                     boolean mupdt);
static void VED_VelCorrCalcResult(const VED_VelCorrEst_t *est,
                                  VED_ModIf_t *mif,
                                  VEDVeloCorrVehDyn_t *corrout,
                                  const reqVEDPrtList_t *reqPorts);
static void VED_VelCorrInitEstimator(VED_VelCorrEst_t *est);
static void VED_VelCorrInitCovEstimator(VED_VelCorrEst_t *est);
static boolean VED_VelCorrOverTake(VED_RefSpeed_t *ref,
                                   VED_VelCorrNode_t *node,
                                   VED_VelCorrAux_t *pAux);
static void VED_VelCorrHistInit(VED_RefSpeed_t ref[]);
static void VED_VelCorrNodeInit(VED_VelCorrNode_t node[]);
static boolean VED_VelCorrCalc(VED_RefSpeed_t *ref, uint32 start, uint32 end);
static boolean VED_VelCorrEvalHist(VED_RefSpeed_t *ref);

#if ((defined(CFG_VED__VELO_MONITOR_MIN_MAX)) && \
     (CFG_VED__VELO_MONITOR_MIN_MAX))
static void VED_CorrectedVeloMonitoring(const VED_InputData_t *input,
                                        const proVEDPrtList_t *proPorts,
                                        VED_ModIf_t *mif);
#endif

static uint32 VED_VelCorrGetVelocityIndex(float32 Velocity);
#if (VEL_CORR_HIST_STATIONARY_TARGETS)
static void VED_VelCorrAdd(const RefSpeed_t *refin, VED_RefSpeed_t *ref);
#elif (VEL_CORR_ALN)
static void VED_VelCorrAdd(const VEDALN_Monitoring_t *refin,
                           VED_RefSpeed_t *ref);
#endif
static void VED_VelCorrSaveNvm(VED_VelCorrNode_t node[],
                               float32 EcuTime,
                               const VEDNvIoDatas_t *nv_read,
                               VEDNvIoDatas_t *nv_write);
static void VED_VelCorrReduce(VED_RefSpeed_t *ref, float32 reduce);
static boolean VED_VelCorrReadNvm(VED_VelCorrNode_t node[],
                                  const VEDNvIoDatas_t *nv_read);
static void VED_VelCorrAuxInit(VED_VelCorrAux_t *aux);
static boolean VED_VelCorrIsObservable(const VED_ModIf_t *mif,
                                       VED_VelCorrAux_t *pAux);

static boolean VED_VelCorrStateInsideLimits(const VED_VelCorrEst_t *est);
static void VED_VelCorrGetNodesAside(uint32 idx, uint32 nidx[]);

#if (CFG_VED__FS_VELO_CORR_MON)
#if (VEL_CORR_HIST_STATIONARY_TARGETS)
static void VED_FSCheckVelCorrMonitoring(const RefSpeed_t *histogram);
#elif (VEL_CORR_ALN)
static void VED_FSCheckVelCorrMonitoring(float32 f_UncorrectedVelocity,
                                         float32 f_UncorrectedVelocityVar,
                                         float32 f_ALNVelocity,
                                         float32 f_ALNVelocityDev,
                                         boolean b_EMProbRTBRecog);
#if (ALN_MONITORING_VERSION > 3U) && (CFG_VED__FS_VELO_CONF_MON_FAULT)
static void VED_FSConfirmVelocity(float32 f_CorrectedVelocity,
                                  float32 f_CorrectedVelocityVar,
                                  float32 f_ALNVelocity,
                                  boolean b_EMProbRTBRecog);
#endif
#endif
#elif (!CFG_VED__FS_VELO_CORR_MON)
static void VED_UncorrVelMonitoring(float32 f_UncorrectedVelocity,
                                    float32 f_UncorrectedVelocityVar,
                                    float32 f_ALNVelocity,
                                    float32 f_ALNVelocityDev);
#endif

#if (CFG_VED__FS_VELO_CORR_MON)
/* **********************************************************************
  @fn                     VED_FSVelCorrMonGetPrivateData */ /*!
  @brief                  Get access to internal velocity correction monitor data

  @description            Returns pointer to data to allow MTS output

  @param[in]              -
  @param[out]             -
  @return                 *VED_FSVelCorrMon_t

  @pre                    -
  @post                   -
**************************************************************************** */
VED_FSVelCorrMon_t *VED_FSVelCorrMonGetPrivateData(void) {
    return (&VED_FSVelCorrMon);
}
#endif

/* **********************************************************************
  @fn                     VED_VelCorrGetPrivateData */ /*!
  @brief                  Get access to internal velocity correction data

  @description            Returns pointer to data to allow MTS output

  @return                 *VED_VelCorr_t
  @param[in]              -
  @param[out]             -

  @pre                    -
  @post                   -
**************************************************************************** */
VED_VelCorr_t *VED_VelCorrGetPrivateData(void) { return (COR_GET_ME()); }

/* **********************************************************************
  @fn               VED_VelCorrGetSpeedIndex */ /*!
  @brief            Get array index for specified velocity

  @description      

  @param[in]        Velocity
  @param[out]       -
  @return           index

  @pre              -
  @post             -
**************************************************************************** */
static uint32 VED_VelCorrGetVelocityIndex(float32 Velocity) {
    uint32 idx;
    float32 fScaleVelo = Velocity * VED__CORR_PER_MILL_SCALE;
    uint32 uiScaleVelocity = (uint32)(fScaleVelo);

    if (uiScaleVelocity <
        (uint32)(VED__CORR_VEL_RANGE_0_MAX * VED__CORR_PER_MILL_SCALE)) {
        idx = 0UL;
    } else if (uiScaleVelocity <
               (uint32)(VED__CORR_VEL_RANGE_1_MAX * VED__CORR_PER_MILL_SCALE)) {
        idx = 1UL;
    } else {
        idx = 2UL;
    }
    return idx;
}

/* **********************************************************************
  @fn               VED_VelCorrEvalHist */ /*!
  @brief            Evaluate velocity correction histogram

  @description      Sorts histogram
                    highest bin index contains max number, is always > 0
                    if histogram is not empty (must be ensured by caller)
                    If difference of highest to second highest and highest to third highest
                    bin values are within limits and the histogram can be evaluated

  @param[in]        Velocity
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static boolean VED_VelCorrEvalHist(VED_RefSpeed_t *ref) {
    const uint32 minIdxRat_c =
        10UL; /* Minimum histogram index location for maximum = 0.7 */
    const uint32 maxIdxRat_c =
        (uint32)(REF_SPEED_NO_BINS)-10U; /* Maximum histogram index loaction for
                                            maximum = 1.3 */
    const uint32 winWidth_c =
        10UL; /* Window width for distribution evaluation */

    /* Parameters to test against normal distribution */
    const float32 rat2ndPeak_c =
        0.7F; /* Ratio between second highest and hightest bin occurrences */
    const float32 rat3rdPeak_c =
        0.5F; /* Ratio between third highest and highest bin occurences */
    const sint32 span2ndPeak_c =
        2L; /* Distance between second highest and hightest value */
    const sint32 span3rdPeak_c =
        4L; /* Distance between third highest and hightest value */

    boolean ret = FALSE;
    uint8 sortIdx[REF_SPEED_NO_BINS];
    uint32 idxMax, idxMax_1, idxMax_2;
    boolean Condition1, Condition2;

#if (VEL_CORR_HIST_STATIONARY_TARGETS)
    VED_HpSortIndU16((uint32)(REF_SPEED_NO_BINS), ref->binVelRatio, sortIdx);
#elif (VEL_CORR_ALN)
    VED_HpSortIndF32((uint32)(REF_SPEED_NO_BINS), ref->binVelRatio, sortIdx);
#endif

    idxMax = sortIdx[(uint32)(REF_SPEED_NO_BINS)-1U];
    idxMax_1 = sortIdx[(uint32)(REF_SPEED_NO_BINS)-2U];
    idxMax_2 = sortIdx[(uint32)(REF_SPEED_NO_BINS)-3U];

    /* check first max to max */
    if (TUE_CML_IsNonZero(ref->binVelRatio[idxMax])) {
        if (((float32)ref->binVelRatio[idxMax_1] /
             (float32)ref->binVelRatio[idxMax]) > rat2ndPeak_c) {
            if (fABS((sint32)idxMax - (sint32)idxMax_1) < span2ndPeak_c) {
                Condition1 = TRUE;
            } else {
                Condition1 = FALSE;
            }
        } else {
            Condition1 = TRUE;
        }

        /* check second max to max */
        if (((float32)ref->binVelRatio[idxMax_2] /
             (float32)ref->binVelRatio[idxMax]) > rat3rdPeak_c) {
            if (fABS((sint32)idxMax - (sint32)idxMax_2) < span3rdPeak_c) {
                Condition2 = TRUE;
            } else {
                Condition2 = FALSE;
            }
        } else {
            Condition2 = TRUE;
        }
    } else /* if (TUE_CML_IsNonZero(ref->binVelRatio[idxMax])) */
    {
        Condition1 = FALSE;
        Condition2 = FALSE;
    }

    if ((Condition1 == TRUE) && (Condition2 == TRUE)) {
        /* significant peaks are close together */
        if ((idxMax >= minIdxRat_c) && (idxMax <= maxIdxRat_c)) {
            ret =
                VED_VelCorrCalc(ref, idxMax - winWidth_c, idxMax + winWidth_c);
        }
    }

    return ret;
}

/* **********************************************************************
  @fn               VED_VelCorrCalc */ /*!
  @brief            Calculate correction value from accumulated histogram data

  @description      Finds the median of the quartiles (1 to 3) and calculates
                    the median of the quartiles
                    The bin value of the median bin can not be 0 unless the histogram
                    is empty (must be ensured by caller).
					Bin values can have very small values, but negative values are not possible.

  @param[in]        ref speed histogram
  @param[in]        start
  @param[in]        end
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static boolean VED_VelCorrCalc(VED_RefSpeed_t *ref, uint32 start, uint32 end) {
    const float32 bwdth_c = BINS_CORR_FACTOR_STEP;
    boolean bSuccess;

    float32 volSum = 0.F;
    float32 ivSum = 0.F;
    uint32 idx;
    float32 rank;

    float32 val_q1; /* quartile values */

    boolean checkagain;

    /* First bin exceeding accumulated sum is first quartile */
    for (idx = start; idx <= end; idx++) {
        ivSum += (float32)ref->binVelRatio[idx];
    }

    /* Calculate first quartile */
    rank = 0.25F * ivSum;

    /* First bin exceeding accumulated sum is first quartile */

    checkagain = (boolean)(volSum < rank);
    for (idx = start; (idx <= end) && (checkagain == TRUE); idx++) {
        volSum += (float32)ref->binVelRatio[idx];
        checkagain = (boolean)(volSum < rank);
    }

    /* Get first quartile range */
    idx--;
    val_q1 = ((float32)(idx)*bwdth_c) + BINS_CORR_FACTOR_START_VALUE;

    /* Improve value by linear interpolation */
    if (TUE_CML_IsNonZero(ref->binVelRatio[idx])) {
        val_q1 +=
            (-0.5F * bwdth_c) +
            ((((rank - (volSum - (float32)ref->binVelRatio[idx])) - 0.5F) *
              bwdth_c) /
             (float32)ref->binVelRatio[idx]);

        /* Calculate second quartile -> median */

        /* First bin exceeding accumulated sum is median */
        rank = 0.5F * ivSum;

        checkagain = (boolean)(volSum < rank);
        for (idx++; (idx <= end) && (checkagain == TRUE); idx++) {
            volSum += (float32)ref->binVelRatio[idx];
            checkagain = (boolean)(volSum < rank);
        }

        /* Get median range value */
        idx--;
        float32 val_q2;
        val_q2 = ((float32)(idx)*bwdth_c) + BINS_CORR_FACTOR_START_VALUE;

        /* Improve value by linear interpolation */
        if (TUE_CML_IsNonZero(ref->binVelRatio[idx])) {
            val_q2 +=
                (-0.5F * bwdth_c) +
                ((((rank - (volSum - (float32)ref->binVelRatio[idx])) - 0.5F) *
                  bwdth_c) /
                 (float32)ref->binVelRatio[idx]);

            /* Calculate third quartile */
            rank = 0.75F * ivSum;

            checkagain = (boolean)(volSum < rank);
            for (idx++; (idx <= end) && (checkagain == TRUE); idx++) {
                volSum += (float32)ref->binVelRatio[idx];
                checkagain = (boolean)(volSum < rank);
            }

            /* Get third quartile range value */
            idx--;
            float32 val_q3;
            val_q3 = ((float32)(idx)*bwdth_c) + BINS_CORR_FACTOR_START_VALUE;

            if (TUE_CML_IsNonZero(ref->binVelRatio[idx])) {
                /* Improve value by linear interpolation */
                val_q3 +=
                    (-0.5F * bwdth_c) +
                    ((((rank - (volSum - (float32)ref->binVelRatio[idx])) -
                       0.5F) *
                      bwdth_c) /
                     (float32)ref->binVelRatio[idx]);

                /* Assign median value */
                ref->Median = val_q2;

                /* Assign pseude standard deviation */
                ref->Dev = 0.7413F * (val_q3 - val_q1);

                /* Calculate average ego velocity */
                VED_StatIntervalMeanDev(&ref->EgoVel);
                bSuccess = TRUE;
            } else {
                bSuccess = FALSE;
            }
        } else {
            bSuccess = FALSE;
        }
    } else {
        bSuccess = FALSE;
    }

    return bSuccess;
}

#if (VEL_CORR_HIST_STATIONARY_TARGETS)
/* **********************************************************************
  @fn               VED_VelCorrHistAdd */ /*!
  @brief            Insert new histogram into long-term histogram

  @description       

  @param[in]        refin one shot histogram address
  @param[out]       ref   long term histogram address

  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrAdd(const RefSpeed_t *refin, VED_RefSpeed_t *ref) {
    uint32 idx;
    float32 locSum = 0.F;

    /* Step through all bins and add bin und total occurences */
    for (idx = 0UL; idx < (uint32)(REF_SPEED_NO_BINS); idx++) {
        ref->binVelRatio[idx] =
            (uint16)(ref->binVelRatio[idx] + refin->binVelRatio[idx]);
        locSum += (float32)refin->binVelRatio[idx];
    }

    /* Accumulate total target counts */
    ref->Sum += locSum;

    /* Accumulate target occurences outside observation interval */
    ref->binVelRatOutLo = (uint16)(refin->binVelRatOutLo + ref->binVelRatOutLo);
    ref->binVelRatOutHi = (uint16)(refin->binVelRatOutHi + ref->binVelRatOutHi);

    /* Add reference ego velocity to statistical observation */
    VED_StatIntervalAdd(&ref->EgoVel, refin->refEgoVelo, locSum);

    /* Store sample counter to identify used input histogram */
    ref->cntSample = refin->cntSample;

    return;
}

#elif (VEL_CORR_ALN)
/* **********************************************************************
  @fn               VED_VelCorrHistAdd */ /*!
  @brief            Insert new histogram into long-term histogram

  @description       

  @param[in]        refin  : estimated ego speed from stationary targets of current radar cycle
  @param[in]        ref    : estimated standard deviation of estimated ego speed
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrAdd(const VEDALN_Monitoring_t *refin,
                           VED_RefSpeed_t *ref) {
    sint32 idx; /* histogram index (bin) of the correction factor */
    float32 locSum =
        1.0F; /* number of samples, currently just on sample is provided (number
                 of targets could be used, but must be provided by ALN) */
    float32 f_VelCorr; /* correction factor of this cycle */

    /* possible division by zero, a check is implemented even if velocity must
     * be higher than a minimum threshold to get here */
    if (TUE_CML_IsNonZero(ref->EgoVel.Mean)) {
        f_VelCorr = refin->f_EgoSpeed / ref->EgoVel.Mean;

        /* calculate index */
        idx = (sint32)(((f_VelCorr - BINS_CORR_FACTOR_START_VALUE) /
                        BINS_CORR_FACTOR_STEP) +
                       0.5f);

        /* inside histogram borders? */
        if (idx >= 0) {
            if (idx < REF_SPEED_NO_BINS) {
                /* Add value to bin */
                ref->binVelRatio[idx] +=
                    (float32)(1.0); /* possible to use
                                       refin->f_EgoSpeedStandardDeviation
                                       as weight */
            } else {
                /* Accumulate occurences outside observation interval */
                ref->binVelRatOutHi++;
            }
        } else {
            /* Accumulate occurences outside observation interval */
            ref->binVelRatOutLo++;
        }

        /* Accumulate total count */
        ref->Sum++; /* possible to use refin->f_EgoSpeedStandardDeviation as
                       weight */

        /* Add reference ego velocity to statistical observation */
        VED_StatIntervalAdd(&ref->EgoVel, refin->f_EgoSpeed, locSum);

        /* Store sample counter to identify used input histogram */
        ref->cntSample = refin->u_UpdateCounter;
    }
    return;
}
#else
#ifdef _WIN32

#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): no velocity correction method defined")
#endif
#endif

/* **********************************************************************
  @fn               VED_VelCorrReduce */ /*!
  @brief            Reduce the observation database to avoid overflow

  @description       
  @param[in]        reduce reduction factor (0..1)
  @param[out]       ref long term histogram address
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrReduce(VED_RefSpeed_t *ref, float32 reduce) {
    uint32 binNo;

    /* Step through all bins and reduce occurences */
    for (binNo = 0UL; binNo < (uint32)(REF_SPEED_NO_BINS); binNo++) {
        uint32 RoundVelRatio;
        RoundVelRatio =
            ROUND_TO_UINT(reduce * (float32)ref->binVelRatio[binNo]);
        ref->binVelRatio[binNo] = (uint16)RoundVelRatio;
    }

    /* Reduce total occurences */
    ref->Sum *= reduce;
    ref->binVelRatOutHi = (uint16)0UL;
    ref->binVelRatOutLo = (uint16)0UL;

    /* Reduce ego velocity observation */
    VED_StatIntervalReduce(&ref->EgoVel, reduce);

    return;
}

/* **********************************************************************
  @fn               VED_VelCorrSaveNvm */ /*!
  @brief            Save learned values to non-volatile memory if necessary

  @description       

  @param[in]        node
  @param[in]        EcuTime
  @param[in]        nv_read
  @param[in]        nv_write
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrSaveNvm(VED_VelCorrNode_t node[],
                               float32 EcuTime,
                               const VEDNvIoDatas_t *nv_read,
                               VEDNvIoDatas_t *nv_write) {
    uint32 ii;

    /* Invalidate non volatile correction data */
    VED_SET_NVM_IO_STATE(VED_NVM_POS_VELCORR, VED_IO_STATE_INVALID,
                         &nv_write->State);

    /* Step through all velocity ranges */
    for (ii = 0U; ii < VED__CORR_VEL_RANGES; ii++) {
        /* Only use internal node values, if it has been learned since ignition
         * start */
        if (node[ii].CorrDev < VED__CORR_INIT_STD_DEV) {
            nv_write->VelCorr[ii].CorrFact = node[ii].CorrFact;
            nv_write->VelCorr[ii].Dev = node[ii].CorrDev;
            nv_write->VelCorr[ii].Velo = node[ii].CorrVel;
        } else {
            /* Node has not been learned, keep stored values */
            nv_write->VelCorr[ii] = nv_read->VelCorr[ii];
        }

        /* Store new correction node value if
              deviation of new value is significant compared to stored values
           or standard deviation of new value is better than stored one
           or velocity range has not been learned before
           and minimum time gap has been elapsed since last write cycle */
        if (((fABS(node[ii].CorrFact - nv_read->VelCorr[ii].CorrFact) >
              VED__CORR_NVM_FAC_DEV) ||
             (node[ii].CorrDev < nv_read->VelCorr[ii].Dev) ||
             ((node[ii].CorrDev < VED__CORR_INIT_STD_DEV) &&
              ((nv_read->VelCorr[ii].Velo < VED_VelCorrRgMap_c[ii].min) ||
               (nv_read->VelCorr[ii].Velo > VED_VelCorrRgMap_c[ii].max)) &&
              ((node[ii].CorrVel > VED_VelCorrRgMap_c[ii].min) &&
               (node[ii].CorrVel < VED_VelCorrRgMap_c[ii].max)))) &&
            ((EcuTime - node[ii].LastNvmWrt) > VCOR_NVM_WRITE_TIME_SPAN)) {
            /* Validate output nvm signal */

            VED_SET_NVM_IO_STATE(VED_NVM_POS_VELCORR, VED_IO_STATE_VALID,
                                 &nv_write->State);

            /* update time since last write access */
            node[ii].LastNvmWrt = EcuTime;
        }
    }

    return;
}

/* **********************************************************************
  @fn               VED_VelCorrReadNvm */ /*!
  @brief            Read correction factor from non-volatile memory

  @description       

  @param[in]        node Correction node address
  @param[out]       nv_read Non-volatile data address
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static boolean VED_VelCorrReadNvm(VED_VelCorrNode_t node[],
                                  const VEDNvIoDatas_t *nv_read) {
    boolean ret;
    boolean taken = FALSE; /* Correction value taken from nv memory */
    boolean learned =
        FALSE; /* New learn value at this ignition cycle available */

    if (VED__GET_NVM_IO_STATE(VED_NVM_POS_VELCORR, &nv_read->State) ==
        VED_IO_STATE_VALID) {
        uint32 ii;
        for (ii = 0UL; ii < VED__CORR_VEL_RANGES; ii++) {
            if ((node[ii].State == (VED_CorrState_t)VEL_CORR_INIT) &&
                (nv_read->VelCorr[ii].Dev < node[ii].CorrDev) &&
                (nv_read->VelCorr[ii].Velo > 0.0F)) {
                /*  Correction nodes have init-state and stored deviation is
                   smaller than current used deviation and it is not on NVM
                   defaults */

                /* Take over non volalatile correction values */
                node[ii].State = (VED_CorrState_t)VEL_CORR_BACKUP;
                node[ii].CorrDev = nv_read->VelCorr[ii].Dev;

                /* Test whether the stored ego velocity is in expected range.
                   During init all three nodes are filled with same values. This
                   prevents using this on than more time */
                if ((nv_read->VelCorr[ii].Velo > VED_VelCorrRgMap_c[ii].min) &&
                    (nv_read->VelCorr[ii].Velo < VED_VelCorrRgMap_c[ii].max)) {
                    node[ii].CorrVel = nv_read->VelCorr[ii].Velo;
                } else {
                    /* Velocity lies not inside the range, invalidate correction
                       node by initalization of standard deviation */
                    node[ii].CorrVel = VED_VelCorrRgMap_c[ii].mid;
                    node[ii].CorrDev = VED__CORR_INIT_STD_DEV;
                }

                /* Test whether to stored correction factor is in expected range
                 */
                if ((nv_read->VelCorr[ii].CorrFact >=
                     (1.F - VED__CORR_ABS_DEV)) &&
                    (nv_read->VelCorr[ii].CorrFact <=
                     (1.F + VED__CORR_ABS_DEV))) {
                    node[ii].CorrFact = nv_read->VelCorr[ii].CorrFact;
                } else {
                    /* Factor lies not inside the range, invalidate correction
                       node by initalization of standard deviation */
                    node[ii].CorrFact = 1.0F;
                    node[ii].CorrDev = VED__CORR_INIT_STD_DEV;
                }
                /* Signal that offset is taken from nvm memory */
                taken = TRUE;
            }
            /* Check if at least one new learn value has already been learned */
            if ((node[ii].State == (VED_CorrState_t)VEL_CORR_READY) ||
                (node[ii].State == (VED_CorrState_t)VEL_CORR_STARTUP)) {
                learned = TRUE;
            }
        }
    }

    /* Only init estimator if one nvm node was taken and no new learn has been
     * learned so far */
    if ((taken != FALSE) && (learned == FALSE)) {
        ret = TRUE;
    } else {
        ret = FALSE;
    }

    return ret;
}

/* **********************************************************************
  @fn               VED_VelCorrOverTake */ /*!
  @brief            Take over new correction value

  @description       

  @param[in]        ref
  @param[in]        node
  @param[in]        pAux
  @param[out]       --
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static boolean VED_VelCorrOverTake(VED_RefSpeed_t *ref,
                                   VED_VelCorrNode_t *node,
                                   VED_VelCorrAux_t *pAux) {
    float32 volReq;
    float32 redFac;
    boolean success = FALSE;

    /* Required samples are dependent on current node state */
    if (node->State == (VED_CorrState_t)VEL_CORR_INIT) {
        volReq = (float32)(VED__CORR_SPL_INIT);
        redFac = VED__CORR_RED_FAC_INIT;
    } else {
        volReq = (float32)(VED__CORR_SPL_NORMAL);
        redFac = VED__CORR_RED_FAC_NORMAL;
    }

    /* If the majority of targets are outside the window */
    if ((((float32)ref->binVelRatOutHi + (float32)ref->binVelRatOutLo) >
         (VED__CORR_SPL_OUTSIDE_LIMITS * volReq)) &&
        (ref->Sum < (0.5F * volReq))) {
        /* Correction value is out of expected range */
        pAux->cntMeasAmb++;

        /* Reduce amount of collected data */
        VED_VelCorrReduce(ref, 0.F);
    } else {
        /* If enough data is acquired to start evaluation */
        if (ref->Sum > volReq) {
            /* Calculate potential correction value */
            if ((VED_VelCorrEvalHist(ref) != FALSE) &&
                (ref->Dev < VED__CORR_STD_DEV)) {
                /* Histogram provides clear distribution */
                /* Reset ambiguous meas error counter and indicate error
                 * inactive */
                if (pAux->cntMeasAmb > 0U) {
                    pAux->cntMeasAmb--;
                }
                COR_SET_ERR_WIN(VED_ERR_STATE_INACTIVE);

                if (fABS(ref->Median - 1.F) < VED__CORR_ABS_DEV) {
                    /* Estimation is good, take over learned values */
                    node->CorrFact = ref->Median;
                    node->CorrDev = ref->Dev;
                    node->CorrVel = ref->EgoVel.Mean;

                    /* Promote learning state */
                    if (node->State == (VED_CorrState_t)VEL_CORR_INIT) {
                        node->State = (VED_CorrState_t)VEL_CORR_STARTUP;
                    } else {
                        node->State = (VED_CorrState_t)VEL_CORR_READY;
                    }

                    /* signal accepted overtake */
                    success = TRUE;
                    b_init_CorrFlag = TRUE;

#if (defined(CFG_VED__CF_ESTIMATED_IN_IGNITION) && \
     (CFG_VED__CF_ESTIMATED_IN_IGNITION))
                    /* signal accepted overtake and the new CF is estimated in
                     * the ignition cycle*/
                    VED_FSVelCorrMon.b_CorrFact_EstFlag = TRUE;
#endif

                    /* Correction factor is inside expectation range */
                    if (pAux->cntMeasRng > 0U) {
                        pAux->cntMeasRng--;
                    }
                    COR_SET_ERR_FACT_RG(VED_ERR_STATE_INACTIVE);
                } else {
                    /* Measured correction factor ouf range */
                    pAux->cntMeasRng++;
                }
            } else {
                /* Distribution inside windows was discarded */
                pAux->cntMeasAmb++;
            }

            /* If distribution provides no useable result, discard it */
            if (success == FALSE) {
                redFac = 0.F;
            }

            /* Reduce amount of collected data */
            VED_VelCorrReduce(ref, redFac);
        }
    }

    /* If number of discarded histograms above threshold indicate error */
    if (pAux->cntMeasAmb >= VED__PAR_VCOR_THRHD_CNT_MEAS_ERR) {
        COR_SET_ERR_WIN(VED_ERR_STATE_ACTIVE);
        pAux->cntMeasAmb =
            MIN(pAux->cntMeasAmb, VED__PAR_VCOR_THRHD_CNT_MEAS_ERR + 2U);
    }

    /* If number of measured correction factors out of range indicate error */
    if (pAux->cntMeasRng >= VED__PAR_VCOR_THRHD_CNT_RANGE_ERR) {
        COR_SET_ERR_FACT_RG(VED_ERR_STATE_ACTIVE);
        pAux->cntMeasAmb =
            MIN(pAux->cntMeasAmb, (VED__PAR_VCOR_THRHD_CNT_RANGE_ERR + 1U));
    }

    return success;
}

/* **********************************************************************
  @fn               VED_VelCorrHistInit */ /*!
  @brief            Initialize histogram data

  @description       

  @param[in]        ref histogram address
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrHistInit(VED_RefSpeed_t ref[]) {
    uint32 rgVel;
    uint32 rgBin;

    /* Step through all velocity ranges */
    for (rgVel = 0UL; rgVel < VED__CORR_VEL_RANGES; rgVel++) {
        /* Init histogram statistics */
        ref[rgVel].Dev = 0.F;
        ref[rgVel].Median = 0.F;
        ref[rgVel].Sum = 0.F;

        /* Init ego velocity statistics */
        VED_StatIntervalInit(&ref[rgVel].EgoVel);

        /* Initialize histogram bins */
        for (rgBin = 0UL; rgBin < (uint32)(REF_SPEED_NO_BINS); rgBin++) {
            ref[rgVel].binVelRatio[rgBin] = (uint16)0;
        }

        /* Initialize outside bins */
        ref[rgVel].binVelRatOutLo = (uint16)0;
        ref[rgVel].binVelRatOutHi = (uint16)0;

        /* Initialize sample counter of input histogram */
        ref[rgVel].cntSample = (uint8)0;
    }
    return;
}

/* **********************************************************************
  @fn               VED_VelCorrNodeInit */ /*!
  @brief            Initialize correction node values

  @description       

  @param[in]        node correction node address
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrNodeInit(VED_VelCorrNode_t node[]) {
    uint32 rgVel;

    /* Step through all velocity ranges */
    for (rgVel = 0UL; rgVel < VED__CORR_VEL_RANGES; rgVel++) {
        node[rgVel].CorrFact = 1.F;
        node[rgVel].CorrDev = VED__CORR_INIT_STD_DEV;
        node[rgVel].CorrVel = VED__CORR_INIT_VEL_NODE;
        node[rgVel].State = (VED_CorrState_t)VEL_CORR_INIT;
        node[rgVel].LastNvmWrt = -VCOR_NVM_WRITE_TIME_SPAN;
    }

    return;
}

/* **********************************************************************
  @fn               VED_VelCorrNodeInit */ /*!
  @brief            Initialize correction node values

  @description       

  @param[in]        aux correction node address
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrAuxInit(VED_VelCorrAux_t *aux) {
    aux->Timer = 0UL;
    aux->maxVelMeas = 0.F;
    aux->minVelMeas = 0.F;
    aux->lastEgoVel = 0.F;
    aux->EcuTime = 0.F;
    aux->cntMeasAmb = 0UL;
    aux->cntMeasRng = 0UL;

    return;
}

/* **********************************************************************
  @fn               VED_VelCorrInit */ /*!
  @brief            Initialize velocity correction module

  @description       

  @param[in]        mif
  @param[in]        proPorts
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_VelCorrInit(const proVEDPrtList_t *proPorts) {
    /* Initialize velocity ratio histograms */
    VED_VelCorrHistInit(COR_GET_HISTS());

    /* Initialize nodes for each velocity range  */
    VED_VelCorrNodeInit(COR_GET_NODES());

    /* Initialize estimator */
    VED_VelCorrInitEstimator(COR_GET_EST());

    /* Initialize auxillary data */
    VED_VelCorrAuxInit(&(COR_GET_ME()->Aux));

    /* Initialize output data */
    proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrFact = 1.0F;
    proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrVar =
        SQR(VED__CORR_INIT_STD_DEV_START);

    /* Init velo correction quality */
    proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrQual =
        VED_VELO_CORR_QUAL_SNA;

#if (CFG_VED__FS_VELO_CORR_MON)
    /* Init fast velocity FS monitor */
    {
        VED_FSVelCorrMon_t *pCorrMonData = VED_FSVelCorrMonGetPrivateData();
        (void)memset((void *)pCorrMonData, 0U, sizeof(VED_FSVelCorrMon_t));
    }
#endif

    /* reset static variables */
    cntSampleOld = 0U;
    /* b_init_CorrFlag to detect whether sufficient number of stationary targets
     * are detected (sets TRUE when first time correction factor kalman filter
     * updation happens)*/
    b_init_CorrFlag = FALSE;

#if ((defined(CFG_VED__VELO_MONITOR_MIN_MAX)) && \
     (CFG_VED__VELO_MONITOR_MIN_MAX))
    /* reset the corrected velocity monitor DEM setting counter  */
    u_debounceCounter_set = 0U;
    u_debounceCounter_Reset = 0U;
#endif

#if (ALN_MONITORING_VERSION > 3U) && (CFG_VED__FS_VELO_CORR_MON) && \
    (CFG_VED__FS_VELO_CONF_MON_FAULT)
    u_ConfCounterOld = 0U;
#endif

    u_velVarianceCounter_ABS = 0U;
    u_velVarianceCounter_TSC = 0U;

    return;
}

/* **********************************************************************
  @fn               VED_VelCorrIsObservable */ /*!
  @brief            Determine if ego velocity is observable by stationary target
                    velocity
  @description       

  @param[in]        mif component input data, module interface data 
  @param[in]        pAux component input data, module interface data 
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static boolean VED_VelCorrIsObservable(const VED_ModIf_t *mif,
                                       VED_VelCorrAux_t *pAux) {
    float32 maxAccelMs = 10.0F * 0.001F;
    boolean observable = TRUE;
    uint32 cyclTimeMS;
    float32 diffVelo;
    float32 maxDiffVelo;
    float32 scaledCycleTime;

    float32 f_vehLongAccelAbs;
    float32 f_diffVeloAbs;

    /* Retrieve cycle time in MS */
    scaledCycleTime = (VED_GetCycleTime() * 1000.F);
    cyclTimeMS = (uint32)scaledCycleTime;

    maxDiffVelo = (float32)((float32)(cyclTimeMS)*maxAccelMs);
    diffVelo = mif->LongMot.VehVelo - pAux->lastEgoVel;

    /* Ego velocity outside of observable range */
    if ((mif->LongMot.MotState.MotState ==
         (uint8)VED_LONG_MOT_STATE_MOVE_RWD) ||
        (mif->LongMot.VehVelo < VED__CORR_VEL_RANGE_0_MIN) ||
        (mif->LongMot.VehVelo > VED__CORR_VEL_RANGE_2_MAX)) {
        observable = FALSE;
    }

    /* calculate absolute values to compare against thresholds */
    f_vehLongAccelAbs = fABS(mif->LongMot.VehAccel);
    f_diffVeloAbs = fABS(diffVelo);

    /* Ego acceleration above observable range */
    if ((f_vehLongAccelAbs > VED__CORR_OBS_ACCEL_MAX) ||
        (f_diffVeloAbs > maxDiffVelo)) {
        pAux->Timer = VED__CORR_OBS_DELAY_TIME;
    }

    /* Ego curvature above observable range */
    if (fABS(mif->Curve.Curve) > VED__CORR_OBS_CURVE_MAX) {
        pAux->Timer = VED__CORR_OBS_DELAY_TIME;
    }

    /* Decrement timer if it is non-zero */
    if (pAux->Timer > cyclTimeMS) {
        pAux->Timer -= cyclTimeMS;
        observable = FALSE;
    } else {
        pAux->Timer = 0UL;
    }

    /* save velocity for next cycle */
    pAux->lastEgoVel = mif->LongMot.VehVelo;

    return observable;
}

/* **********************************************************************
  @fn               VED_VelCorrStateInsideLimits */ /*!
  @brief            Determine if state value are inside allowed deviations

  @description       

  @param[in]        est component input data, module interface data 
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static boolean VED_VelCorrStateInsideLimits(const VED_VelCorrEst_t *est) {
    boolean ret = FALSE;
    float32 corr_vmax;

    /* Calculate speed correction at maximum speed */
    corr_vmax = est->X[0] + (est->X[1] * SQR(VED__CORR_VEL_RANGE_2_MAX));

    if (((est->X[0] >= VED__CORR_ABS_ICT_MIN_VAL) &&
         (est->X[0] <= VED__CORR_ABS_ICT_MAX_VAL)) &&
        ((est->X[1] >= VED__CORR_ABS_SLP_MIN_VAL) &&
         (est->X[1] <= VED__CORR_ABS_SLP_MAX_VAL)) &&
        ((corr_vmax >= VED__CORR_ABS_ICT_MIN_VAL) &&
         (corr_vmax <= VED__CORR_ABS_ICT_MAX_VAL))) {
        ret = TRUE;
    }

    return ret;
}

/* **********************************************************************
  @fn               VED_VelCorrGetNodesAside */ /*!
  @brief            Determine all node indices but one passed 

  @description      Fills the nidx array with histograms numbers of the
                    histograms which are not the one given number in idx
                    Three histograms are defined for velocity correction
                    Example: index 1 leaves index 0 and inde 2 remaining

  @param[in]        idx
  @param[in]        nidx
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrGetNodesAside(uint32 idx, uint32 nidx[]) {
    switch (idx) {
        case 0:
            nidx[0] = 1UL;
            nidx[1] = 2UL;
            break;
        case 1:
            nidx[0] = 0UL;
            nidx[1] = 2UL;
            break;
        case 2:
            nidx[0] = 0UL;
            nidx[1] = 1UL;
            break;
        default:
            break;
    }
    return;
}

/* ***********************************************************************
  @fn               VED_VelCorrExec */ /*!
  @brief            Determine operating sequence for vehicle dynamics observer

  @description       

  @param[in]        reqPorts
  @param[in]        input
  @param[in]        mif
  @param[in]        proPorts
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_VelCorrExec(const reqVEDPrtList_t *reqPorts,
                     const VED_InputData_t *input,
                     VED_ModIf_t *mif,
                     const proVEDPrtList_t *proPorts,
                     boolean b_RTBDetection) {
    VED_VelCorr_t *pVelCorr = COR_GET_ME();

    float32 CycleTime;
#if ((defined(CFG_VED__VELO_MONITOR_MIN_MAX)) && \
     (CFG_VED__VELO_MONITOR_MIN_MAX))
    float32 Out_Accel_Interpolation = 0.F;
    float32 Input_velo_Interpolation = 0.F;
#endif
    pVelCorr->Io.mif = mif;
    pVelCorr->Io.errFactRg = &proPorts->pVED_Errors->OutPutErrors.VelCorrRg;
    pVelCorr->Io.errWin = &proPorts->pVED_Errors->OutPutErrors.VelCorrWin;

    /* Determine cycle time */
    CycleTime = VED_GetCycleTime();

    /* Update ecu runtime */
    if (pVelCorr->Aux.EcuTime >= VCOR_ECU_ELPSD_TIME_MAX) {
        pVelCorr->Aux.EcuTime = VCOR_ECU_ELPSD_TIME_MAX;
    } else {
        pVelCorr->Aux.EcuTime += CycleTime;
    }

    /* Initialize error velocity correction factor range as not observable */
    COR_SET_ERR_FACT_RG(VED_ERR_STATE_UNKNOWN);

    /* Initialize error velocity correction measurement window distribution as
     * not observable */
    COR_SET_ERR_WIN(VED_ERR_STATE_UNKNOWN);

    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
        proPorts->pVED_Errors->OutPutErrors.VED_FS_VEH_CORR_MON =
            VED_ERR_STATE_UNKNOWN;
    }

    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_RUNNING,
                            input->Frame.CtrlMode)) {
        boolean nvmCorrTaken;
        /* Read non-volatile correction data */
        nvmCorrTaken = VED_VelCorrReadNvm(pVelCorr->Node, reqPorts->pNVMRead);

        if (nvmCorrTaken != FALSE) {
            uint32 rgVelo;

            /* Initialize estimator with usable nodes from non-volatile memory
             */
            for (rgVelo = 0UL; rgVelo < VED__CORR_VEL_RANGES; rgVelo++) {
                if (pVelCorr->Node[rgVelo].CorrDev < VED__CORR_INIT_STD_DEV) {
                    VED_VelCorrExecEstimator(&pVelCorr->Node[rgVelo],
                                             &pVelCorr->Est, TRUE);
                }
            }

            /* Check if the stored measurements results in reasonable
             * estimations */
            if (VED_VelCorrStateInsideLimits(&pVelCorr->Est) == FALSE) {
                /* Discard nvm init values */
                VED_VelCorrInitEstimator(&pVelCorr->Est);
            } else {
                /* The measurement values are from last ignition cylce, init
                 * estimation variances */
                VED_VelCorrInitCovEstimator(&pVelCorr->Est);
            }
        }
        uint32 idxVel;
#if (VEL_CORR_HIST_STATIONARY_TARGETS)
        /* Find velocity range index */
        idxVel = VED_VelCorrGetVelocityIndex(
            reqPorts->pVelStatObj->RefSpeed.refEgoVelo);

        if ((reqPorts->pVelStatObj->state == (SigState_t)SIGNAL_VALID) &&
            (reqPorts->pVelStatObj->RefSpeed.cntSample != cntSampleOld)) {
            // uint32 idxAside[2];

            /* Update sync counter */
            cntSampleOld = reqPorts->pVelStatObj->RefSpeed.cntSample;
#elif (VEL_CORR_ALN)
        /* Find velocity range index */
        idxVel =
            VED_VelCorrGetVelocityIndex(reqPorts->pAln_Monitoring->f_EgoSpeed);
        boolean newNodeVal = FALSE;
        /* Check if new ALN data is available and Roller test bench is not
         * detected*/
        if ((reqPorts->pAln_Monitoring->u_UpdateCounter != cntSampleOld) &&
            (reqPorts->pAln_Monitoring->f_EgoSpeed > 0.0F) &&
            (reqPorts->pAln_Monitoring->f_EgoSpeedStandardDeviation <
             VED__VCOR_NO_ALN_DATA_STD_DEV) &&
            (b_RTBDetection != TRUE)) {
            /* Update sync counter */
            cntSampleOld = reqPorts->pAln_Monitoring->u_UpdateCounter;
#endif

#if (CFG_VED__FS_VELO_CORR_MON)
            /* check velocity by short term correction calculation */
#if (VEL_CORR_HIST_STATIONARY_TARGETS)
            VED_FSCheckVelCorrMonitoring(&reqPorts->pVelStatObj->RefSpeed);
#elif (VEL_CORR_ALN)
            VED_FSCheckVelCorrMonitoring(
                mif->LongMot.VehVelo, mif->LongMot.VehVeloVar,
                reqPorts->pAln_Monitoring->f_EgoSpeed,
                reqPorts->pAln_Monitoring->f_EgoSpeedStandardDeviation,
                b_RTBDetection);
#endif

#elif (!CFG_VED__FS_VELO_CORR_MON)
            VED_UncorrVelMonitoring(
                mif->LongMot.VehVelo, mif->LongMot.VehVeloVar,
                reqPorts->pAln_Monitoring->f_EgoSpeed,
                reqPorts->pAln_Monitoring->f_EgoSpeedStandardDeviation);
#endif

#if ((CFG_VED__FS_VELO_CORR_MON) && (!CFG_VED__USE_VELO_MONITOR) && \
     (CFG_VED__FS_VELO_CORR_MON_ERROR))

            /* check if FS velocity monitor detected a fault */
            if (VED_FSVelCorrMon.fault == VED_ERR_STATE_ACTIVE) {
                proPorts->pVED_Errors->OutPutErrors.VED_FS_VEH_CORR_MON =
                    VED_ERR_STATE_ACTIVE;
            } else {
                proPorts->pVED_Errors->OutPutErrors.VED_FS_VEH_CORR_MON =
                    VED_ERR_STATE_INACTIVE;
            }

#endif

            boolean bIsVelCorrObservable;

            /* Observability of ego velocity for correction */
            bIsVelCorrObservable = VED_VelCorrIsObservable(mif, &pVelCorr->Aux);
            if (bIsVelCorrObservable == TRUE) {
                uint32 idxAside[2];
                /* During init state fill all velocity ranges */
                if (VED_VelCorr.Node[0].State ==
                    (VED_CorrState_t)VEL_CORR_INIT) {
#if (VEL_CORR_HIST_STATIONARY_TARGETS)
                    /* Add data of all three histograms to long-term histograms
                     */
                    VED_VelCorrAdd(&reqPorts->pVelStatObj->RefSpeed,
                                   &pVelCorr->Hist[0]);
                    VED_VelCorrAdd(&reqPorts->pVelStatObj->RefSpeed,
                                   &pVelCorr->Hist[1]);
                    VED_VelCorrAdd(&reqPorts->pVelStatObj->RefSpeed,
                                   &pVelCorr->Hist[2]);
#elif (VEL_CORR_ALN)
                    /* Store current uncorrected speed in all three histograms
                     */
                    pVelCorr->Hist[0].EgoVel.Mean = mif->LongMot.VehVelo;
                    pVelCorr->Hist[1].EgoVel.Mean = mif->LongMot.VehVelo;
                    pVelCorr->Hist[2].EgoVel.Mean = mif->LongMot.VehVelo;

                    /* Add data of all three histograms to long-term histograms
                     */
                    VED_VelCorrAdd(reqPorts->pAln_Monitoring,
                                   &pVelCorr->Hist[0]);
                    VED_VelCorrAdd(reqPorts->pAln_Monitoring,
                                   &pVelCorr->Hist[1]);
                    VED_VelCorrAdd(reqPorts->pAln_Monitoring,
                                   &pVelCorr->Hist[2]);
#endif
                    /* Test for take over of new measurment nodes */
                    newNodeVal = VED_VelCorrOverTake(
                        &pVelCorr->Hist[0], &pVelCorr->Node[0], &pVelCorr->Aux);
                    newNodeVal |= VED_VelCorrOverTake(
                        &pVelCorr->Hist[1], &pVelCorr->Node[1], &pVelCorr->Aux);
                    newNodeVal |= VED_VelCorrOverTake(
                        &pVelCorr->Hist[2], &pVelCorr->Node[2], &pVelCorr->Aux);

                    if (newNodeVal != FALSE) {
                        /* Determine velocity range learned right now */
                        idxVel = VED_VelCorrGetVelocityIndex(
                            pVelCorr->Hist[0].EgoVel.Mean);

                        /* Clear sampled data at non-affected nodes */
                        VED_VelCorrGetNodesAside(idxVel, idxAside);
                        VED_VelCorrReduce(&pVelCorr->Hist[idxAside[0]], 0.F);
                        VED_VelCorrReduce(&pVelCorr->Hist[idxAside[1]], 0.F);
                    }
                } else {
                    boolean waitForFirstMeas;

                    /* Non-Init state, histograms are filled velocity selective
                     */
                    if ((pVelCorr->Node[0].State ==
                         (VED_CorrState_t)VEL_CORR_BACKUP) &&
                        (pVelCorr->Node[1].State ==
                         (VED_CorrState_t)VEL_CORR_BACKUP) &&
                        (pVelCorr->Node[2].State ==
                         (VED_CorrState_t)VEL_CORR_BACKUP)) {
                        waitForFirstMeas = TRUE;
                    } else {
                        waitForFirstMeas = FALSE;
                    }

                    /* Add data to specific velocity range */
#if (VEL_CORR_HIST_STATIONARY_TARGETS)
                    VED_VelCorrAdd(&reqPorts->pVelStatObj->RefSpeed,
                                   &pVelCorr->Hist[idxVel]);
#elif (VEL_CORR_ALN)
                    pVelCorr->Hist[idxVel].EgoVel.Mean =
                        mif->LongMot.VehVelo; /* Store uncorrected speed */
                    VED_VelCorrAdd(reqPorts->pAln_Monitoring,
                                   &pVelCorr->Hist[idxVel]);
#endif
                    /* Test for takeover */
                    newNodeVal = VED_VelCorrOverTake(&VED_VelCorr.Hist[idxVel],
                                                     &VED_VelCorr.Node[idxVel],
                                                     &pVelCorr->Aux);

                    if (newNodeVal != FALSE) {
                        /* difference between factor in current node and factor
                         * in node 0 and 1  */
                        float32 f_corrFactDiffNode0Abs;
                        float32 f_corrFactDiffNode1Abs;

                        /* Ascertain nodes which have not been updated */
                        VED_VelCorrGetNodesAside(idxVel, idxAside);

                        f_corrFactDiffNode0Abs =
                            fABS(pVelCorr->Node[idxVel].CorrFact -
                                 pVelCorr->Node[idxAside[0]].CorrFact);
                        f_corrFactDiffNode1Abs =
                            fABS(pVelCorr->Node[idxVel].CorrFact -
                                 pVelCorr->Node[idxAside[1]].CorrFact);

                        /* Compare first measurement at current ignition cycle
                         * with stored value from NV memory */
                        if ((waitForFirstMeas != FALSE) &&
                            ((f_corrFactDiffNode0Abs > VED__CORR_NVM_FAC_DEV) ||
                             (f_corrFactDiffNode1Abs >
                              VED__CORR_NVM_FAC_DEV))) {
                            /* Discard nvm init values due to large deviation to
                             * new value */
                            VED_VelCorrInitEstimator(&pVelCorr->Est);
                        }
                    }
                }

                /* if new estimation has been taken over, update non-volatile
                 * memory */
                if (newNodeVal != FALSE) {
                    VED_VelCorrSaveNvm(pVelCorr->Node, pVelCorr->Aux.EcuTime,
                                       reqPorts->pNVMRead, proPorts->pNVMWrite);
                }
            }
        }

        /* Update miniumum and maximum ego velocity where correction factor has
         * been observed */
        if (newNodeVal != FALSE) {
            /* Update maximum ego velocity used for velocity correction */
            if (pVelCorr->Aux.maxVelMeas > 0.F) {
                pVelCorr->Aux.maxVelMeas =
                    MAX(pVelCorr->Hist[idxVel].EgoVel.Mean,
                        pVelCorr->Aux.maxVelMeas);
            } else {
                pVelCorr->Aux.maxVelMeas = pVelCorr->Hist[idxVel].EgoVel.Mean;
            }
            /* Update minimum ego velocity used for velocity correction */
            if (pVelCorr->Aux.minVelMeas > 0.F) {
                pVelCorr->Aux.minVelMeas =
                    MIN(pVelCorr->Hist[idxVel].EgoVel.Mean,
                        pVelCorr->Aux.minVelMeas);
            } else {
                pVelCorr->Aux.minVelMeas = pVelCorr->Hist[idxVel].EgoVel.Mean;
            }
        }

        /* Run estimator */
        /* start filter after one node was updated */
        VED_VelCorrExecEstimator(&pVelCorr->Node[idxVel], &VED_VelCorr.Est,
                                 newNodeVal);

        /* Calculate output data */
        VED_VelCorrCalcResult(
            &VED_VelCorr.Est, mif,
            &proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr, reqPorts);

#if ((defined(CFG_VED__VELO_MONITOR_MIN_MAX)) && \
     (CFG_VED__VELO_MONITOR_MIN_MAX))
        /* Linear interpolation to deactiavte the vel_mon and vel_monLT DEM  */
        if (mif->LongMot.VehVelocityCorr > VED__INTERPOL_VEL_RANGE_MAX) {
            Input_velo_Interpolation = VED__INTERPOL_VEL_RANGE_MAX;
        } else {
            Input_velo_Interpolation = mif->LongMot.VehVelocityCorr;
        }

        Out_Accel_Interpolation = BML_f_LinearInterpolation(
            VED__INTERPOL_VEL_RANGE_MIN, VED__INTERPOL_ACCEL_RANGE_MIN,
            VED__INTERPOL_VEL_RANGE_MAX, VED__INTERPOL_ACCEL_RANGE_MAX,
            Input_velo_Interpolation);

        if ((mif->LongMot.VehAccel < VED__DECELERATION_RANGE_MAX) ||
            (mif->LongMot.VehAccel > Out_Accel_Interpolation)) {
            VED_SET_IO_STATE(VED_SOUT_POS_VEL, VED_IO_STATE_VALID,
                             proPorts->pVehicleDynamicSignals->State);
            proPorts->pVED_Errors->OutPutErrors.VelMon = VED_ERR_STATE_INACTIVE;
            proPorts->pVED_Errors->OutPutErrors.VelMonLt =
                VED_ERR_STATE_INACTIVE;
            u_debounceCounter_set = 0U;
            u_debounceCounter_Reset = VED__PAR_VMON_CYCLE_IN;
        } else {
            /* Monitoring the VED output velocity against the MIN and MAX
             * velocity */
            VED_CorrectedVeloMonitoring(input, proPorts, mif);
        }
#endif
#if (ALN_MONITORING_VERSION > 3U) && (CFG_VED__FS_VELO_CORR_MON) && \
    (CFG_VED__FS_VELO_CONF_MON_FAULT)

        /* Check if new ALN data is available */
        if ((reqPorts->pAln_Monitoring->u_ConfirmationUpdateCounter !=
             u_ConfCounterOld) &&
            (reqPorts->pAln_Monitoring->f_ConfirmationEgoSpeed > 0.0F)) {
            u_ConfCounterOld =
                reqPorts->pAln_Monitoring
                    ->u_ConfirmationUpdateCounter;  //  update the counter
            /* check velocity */
            VED_FSConfirmVelocity(
                mif->LongMot.VehVelocityCorr,
                proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr
                    .corrVeloVar,
                reqPorts->pAln_Monitoring->f_ConfirmationEgoSpeed,
                b_RTBDetection);

            /* only temporary, set VelMon fault if FS confirmation monitor
             * detected a fault */
            if (VED_FSVelCorrMon.confFault == VED_ERR_STATE_ACTIVE) {
                proPorts->pVED_Errors->OutPutErrors.VelMon =
                    VED_ERR_STATE_ACTIVE;
            } else {
                proPorts->pVED_Errors->OutPutErrors.VelMon =
                    VED_ERR_STATE_INACTIVE;
            }
        }
#endif

        /* set velocity correction quality flag if the node was updated or NVM
         * correction factor was confirmed by first new samples */
        /* if correction factor is available, either from NVM or even from new
         * histogram */
#if (CFG_VED__FS_VELO_CORR_MON)
        if (VED_FSVelCorrMon.fault == VED_ERR_STATE_INACTIVE) {
            /* velocity check successful, confirm qualifier */
            proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrQual =
                VED_VELO_CORR_QUAL_RANGE_VERIFIED;
        } else
#endif
        {
            if (pVelCorr->Node[idxVel].State ==
                (VED_CorrState_t)VEL_CORR_INIT) {
                /* NVM is either not available or no correction factor is stored
                 */
                proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr
                    .corrQual = VED_VELO_CORR_QUAL_EEPROM;
            } else {
                /* velocity check either not completed yet or set a fault, no
                 * corfirmation of qualifier */
                proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr
                    .corrQual = VED_VELO_CORR_QUAL_RANGE_NVERIFIED;
            }
        }
    }  // if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_RUNNING,
       // input->Frame.CtrlMode))

    return;
}

/* ***********************************************************************
  @fn               VED_VelCorrExecEstimator */ /*!
  @brief            Execute correction factor estimator

  @description       

  @param[in]        node  :  measurement points
  @param[in]        mupdt :  if true perform measurement update
  @param[out]       est   :  new estimation
  @return           void
  
  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrExecEstimator(const VED_VelCorrNode_t *node,
                                     VED_VelCorrEst_t *est,
                                     boolean mupdt) {
    /* Prediction  xhat = F * xhat*/

    /* Predicted estimate covariance  P = F * P * F' + Q */
    if (est->P[0] < (VCOR_EST_P_INIT_QFACT * Q_c[0])) {
        est->P[0] += Q_c[0];
        est->P[1] += Q_c[1];
        est->P[2] += Q_c[2];
        est->P[3] += Q_c[3];
    }

    if (mupdt != FALSE) {
        /* New measurement is available, update estimator */
        float32 y;
        float32 R;
        float32 S;
        float32 K[2];

        /* Build observation matrix */
        est->H[1] = SQR(node->CorrVel);

        /*  Calculate measurement variance */
        R = node->CorrDev * R_STD_DEV_FACT;
        R *= R;

        /* Innovation residual y = z - H * xhat */
        y = node->CorrFact -
            ((est->H[0] * est->X[0]) + (est->H[1] * est->X[1]));

        /* Innovation covariance  S = H * P * H' + R */
        S = (((est->H[0] * est->P[0]) + (est->H[1] * est->P[2])) * est->H[0]) +
            (((est->H[0] * est->P[1]) + (est->H[1] * est->P[3])) * est->H[1]) +
            R;

        /* Optimal Kalman gain   K = P * H' * inv(S) */
        {
            static const float32 minBnd_c = 1E-30F;
            float32 invS;

            /* Avoid division by zero */
            if (fABS(S) > minBnd_c) {
                invS = 1.0F / S;
            } else {
                if (S > 0.0F) {
                    invS = (1.0F / minBnd_c);
                } else {
                    invS = -(1.0F / minBnd_c);
                }
            }

            K[0] = ((est->P[0] * est->H[0]) + (est->P[1] * est->H[1])) * invS;
            K[1] = ((est->P[2] * est->H[0]) + (est->P[3] * est->H[1])) * invS;
        }

        /* Update state estimate xhat = xhat + K * y */
        est->X[0] += K[0] * y;
        est->X[1] += K[1] * y;

        /* Applying state constraints if necessary */
        {
            float32 XC_1;

            XC_1 = TUE_CML_MinMax(VED__CORR_ABS_SLP_MIN_VAL,
                                  VED__CORR_ABS_SLP_MAX_VAL, est->X[1]);

            est->X[0] = est->X[0] + (est->H[1] * (est->X[1] - XC_1));
            est->X[1] = XC_1;
        }

        /* Compute the covariance of the estimation error  P = (eye(2) - K * H)
         * * P */
        {
            float32 Pn[4];

            /* Copy new values first to seperate buffer to keep P matrix
             * consistent during update */
            Pn[0] = ((1.0F - (K[0] * est->H[0])) * est->P[0]) -
                    (K[0] * est->H[1] * est->P[2]);
            Pn[1] = ((1.0F - (K[0] * est->H[0])) * est->P[1]) -
                    (K[0] * est->H[1] * est->P[3]);
            Pn[2] = (-K[1] * est->H[0] * est->P[0]) +
                    ((1.0F - (K[1] * est->H[1])) * est->P[2]);
            Pn[3] = (-K[1] * est->H[0] * est->P[1]) +
                    ((1.0F - (K[1] * est->H[1])) * est->P[3]);

            /* Update covariance matrix */
            est->P[0] = Pn[0];
            est->P[1] = Pn[1];
            est->P[2] = Pn[2];
            est->P[3] = Pn[3];
        }
    }

    return;
}

/* ***********************************************************************
  @fn               VED_VelCorrInitCovEstimator */ /*!
  @brief            Init correction factor coveriance matrix

  @description      

  @param[in]        est
  @return           initialized coveriance matrix

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrInitCovEstimator(VED_VelCorrEst_t *est) {
    est->P[0] = VCOR_EST_P_INIT_QFACT * Q_c[0];
    est->P[1] = VCOR_EST_P_INIT_QFACT * Q_c[1];
    est->P[2] = VCOR_EST_P_INIT_QFACT * Q_c[2];
    est->P[3] = VCOR_EST_P_INIT_QFACT * Q_c[3];

    return;
}

/* ***********************************************************************
  @fn               VED_VelCorrInitEstimator */ /*!
  @brief            Init internal correction factor estimation data

  @description       
    
  @param[in]        
  @return           est : initialized estimator data base

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrInitEstimator(VED_VelCorrEst_t *est) {
    /* Init covariance matrix */
    VED_VelCorrInitCovEstimator(est);

    est->H[0] = 1.0F;
    est->H[1] = 0.F;

    est->X[0] = 1.0F;
    est->X[1] = 0.F;

    return;
}

/* **********************************************************************
  @fn               VED_VelCorrCalcResult */ /*!
  @brief            Calculate final result derived from estimated states

  @description       
   
  @param[in]        est
  @param[in]        mif
  @param[out]       correction factor, corrected speed with variances

  @pre              -
  @post             -
**************************************************************************** */
static void VED_VelCorrCalcResult(const VED_VelCorrEst_t *est,
                                  VED_ModIf_t *mif,
                                  VEDVeloCorrVehDyn_t *corrout,
                                  const reqVEDPrtList_t *reqPorts) {
    VED_VelCorrAux_t *pVCorrAux = &(COR_GET_ME()->Aux);
    float32 velo = mif->LongMot.VehVelo;
    float32 dev;
    float32 X1_constr = est->X[1];
    static float32 lastVeloVar = 0.0F;

    /* Calculate correction factor */
    {
        float32 vel2 = SQR(velo);
        corrout->corrFact = est->X[0] + (vel2 * X1_constr);
        corrout->corrVar = est->P[0] + (SQR(vel2) * est->P[3]) +
                           (vel2 * (est->P[1] + est->P[2]));
    }

#if ((defined(CFG_VED__USE_CORRECT_VELO_CORR_VAR)) && \
     (CFG_VED__USE_CORRECT_VELO_CORR_VAR))
    /* The variance of the corr velo should be higher if the velo hist is not
       learned or not validated This change has negative effects on the grid
       object if meas files without learned or validated histo are used The
       variance is higher and so the grid object position might be different
       This patch was developed for the ARS32x Project for IL_1998 */
    if ((pVCorrAux->maxVelMeas + pVCorrAux->minVelMeas) > 0.F) {
        static const float32 dVarDelta_c =
            1E-4F; /* Lower max. variance if one measurement update is available
                    */

        /* Limit output correction factor variance to max. reasonable value, if
           at least one measurement has been available */
        if (corrout->corrVar > (SQR(VED__CORR_INIT_STD_DEV) - dVarDelta_c)) {
            corrout->corrVar = SQR(VED__CORR_INIT_STD_DEV) - dVarDelta_c;
        }

        /* If velocity dependency is not observable keep variance above minimum
         * value */
        if ((pVCorrAux->maxVelMeas - pVCorrAux->minVelMeas) <
            VED__CORR_OBS_MIN_VEL_RG) {
            corrout->corrVar =
                MAX(corrout->corrVar, SQR(VED__CORR_OBS_STD_DEV));
        }
    } else {
        /* Limit output correction factor variance to max. reasonable value, if
         * no measurement is available */
        if (corrout->corrVar > SQR(VED__CORR_INIT_STD_DEV)) {
            corrout->corrVar = SQR(VED__CORR_INIT_STD_DEV);
        }
    }
#else
    /* Limit output correction factor variance to max. reasonable value */
    if (corrout->corrVar > SQR(VED__CORR_INIT_STD_DEV)) {
        corrout->corrVar = SQR(VED__CORR_INIT_STD_DEV);
    } else {
        /* If velocity dependency is not observable keep variance above minimum
         * value */
        if ((pVCorrAux->maxVelMeas - pVCorrAux->minVelMeas) <
            VED__CORR_OBS_MIN_VEL_RG) {
            corrout->corrVar = SQR(VED__CORR_OBS_STD_DEV);
        }
    }
#endif

    /* Calculate corrected velocity for interface to handcode parts */
    /* Final corrected output velocity must be calculated later as velocity is
     * calculated later in this cycle */
    mif->LongMot.VelCorrFact = corrout->corrFact;
    mif->LongMot.VehVelocityCorr = corrout->corrFact * mif->LongMot.VehVelo;

    /* Are sufficient number of stationary targets detected? (b_init_CorrFlag
     * sets TRUE when first time correction factor test was passed) */
    if (b_init_CorrFlag == TRUE) {
        /* Calculate variance of corrected velocity (variance derived from
         * product of two normal distributed RV) */
        corrout->corrVeloVar =
            (SQR(mif->LongMot.VehVelo) * corrout->corrVar) +
            (SQR(corrout->corrFact) * mif->LongMot.VehVeloVar);

        /* correction for velocity variance as correction factor is not constant
         * over velocity */
        corrout->corrVeloVar *=
            (VED__CORR_VAR_CORRECT_FACTOR * mif->LongMot.VehVelo);
    } else {
        /* Set output correction factor variance to default value */
        corrout->corrVar = TUE_CML_Sqr(VED__CORR_INIT_STD_DEV_START);

        /* If no correction factor is available or not validated with a radar
           target, the variance values might become unrealistic as the
           mathetical model of a Gaussian distribution does not fit any more.
           The variance is assumed to be of a uniform distribution of the
           correction factor with a maximum deviation of 10%*/
        corrout->corrVeloVar =
            (TUE_CML_Sqr(2.0F *
                         (mif->LongMot.VehVelo * VED__FS_VEL_MON_RAT_THRHD))) /
            12.0F;
    }
    /*During high Deceleration condition when ABS /TSC  signal is active  ,
     * Variance of velocity is limited to the maximum */
    if ((VED_GET_IO_STATE(VEH_SIG_BRAKE_ABS,
                          reqPorts->pVehicleInputSignals->Brake.State) ==
         VED_IO_STATE_VALID) &&
        (reqPorts->pVehicleInputSignals->Brake.ABSCtrl == FALSE)) {
        u_velVarianceCounter_ABS = 0U;
    }

    if ((VED_GET_IO_STATE(VEH_SIG_BRAKE_TSC,
                          reqPorts->pVehicleInputSignals->Brake.State) ==
         VED_IO_STATE_VALID) &&
        (reqPorts->pVehicleInputSignals->Brake.TCSCtrl == FALSE)) {
        u_velVarianceCounter_TSC = 0U;
    }

    /*During high Deceleration condition when ABS /TSC  signal is active  and
     * variance is greater than threshold, Variance of velocity is limited to
     * the maximum */

    if (((VED_GET_IO_STATE(VEH_SIG_BRAKE_ABS,
                           reqPorts->pVehicleInputSignals->Brake.State) ==
          VED_IO_STATE_VALID) &&
         (reqPorts->pVehicleInputSignals->Brake.ABSCtrl == TRUE) &&
         (u_velVarianceCounter_ABS < OUTPUT_VEL_VAR_LIMITATION)) ||

        ((VED_GET_IO_STATE(VEH_SIG_BRAKE_TSC,
                           reqPorts->pVehicleInputSignals->Brake.State) ==
          VED_IO_STATE_VALID) &&
         (reqPorts->pVehicleInputSignals->Brake.TCSCtrl == TRUE) &&
         (u_velVarianceCounter_TSC < OUTPUT_VEL_VAR_LIMITATION))) {
        if (corrout->corrVeloVar > VED__VELOCITY_MAX_VARIANCE) {
            corrout->corrVeloVar = VED__VELOCITY_MAX_VARIANCE;
        }

        if ((VED_GET_IO_STATE(VEH_SIG_BRAKE_ABS,
                              reqPorts->pVehicleInputSignals->Brake.State) ==
             VED_IO_STATE_VALID) &&
            (reqPorts->pVehicleInputSignals->Brake.ABSCtrl == TRUE)) {
            u_velVarianceCounter_ABS++;
        }

        if ((VED_GET_IO_STATE(VEH_SIG_BRAKE_TSC,
                              reqPorts->pVehicleInputSignals->Brake.State) ==
             VED_IO_STATE_VALID) &&
            (reqPorts->pVehicleInputSignals->Brake.TCSCtrl == TRUE)) {
            u_velVarianceCounter_TSC++;
        }
    }
    /*Making the corrected velocity variance as Non-Zero when calculated
     * corrected velocity variance is Zero */
    corrout->corrVeloVar = MAX(TUE_CML_AlmostZero, corrout->corrVeloVar);

    /* Calculate min and max value derived from 3 sigma range and check for
     * negative variance */
    if ((TUE_CML_IsNonZero(corrout->corrVeloVar)) &&
        (corrout->corrVeloVar > 0.0F)) {
        dev = 3.F * VED__SQRT(corrout->corrVeloVar);
        lastVeloVar = corrout->corrVeloVar;
    } else {
        dev = 3.F * VED__SQRT(lastVeloVar);
    }
    corrout->minVelo = corrout->corrVelo - dev;
    corrout->maxVelo = corrout->corrVelo + dev;

    return;
}
#endif

#if ((defined(CFG_VED__VELO_MONITOR_MIN_MAX)) && \
     (CFG_VED__VELO_MONITOR_MIN_MAX))
/* ***********************************************************************
  @fn               VED_CorrectedVeloMonitoring*/ /*!
  @brief            Monitor the corrected velocity 

  @description      Monitor the corrected velocity based on the Minimum and Maximum velocity range and set the DEM respectively
                    
   
  @param[in]        input minimum velocity range
  @param[in]        input maximum velocity range
  @param[out]       VED_Error vel_mon , vel_monLT
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_CorrectedVeloMonitoring(const VED_InputData_t *input,
                                        const proVEDPrtList_t *proPorts,
                                        VED_ModIf_t *mif) {
    const uint32 incErrCnt_c = 1UL; /* Increment step size */

    // if both the min and max velocities are VALID use both for monitoring the
    // corrected velocity
    if ((VED_GET_IO_STATE(VED__SIN_POS_VEHVEL_MIN, input->Signals.State) ==
         VED_IO_STATE_VALID) &&
        (VED_GET_IO_STATE(VED__SIN_POS_VEHVEL_MAX, input->Signals.State) ==
         VED_IO_STATE_VALID)) {
        if ((mif->LongMot.VehVelocityCorr < input->Signals.VehVelocityExtMin) ||
            (mif->LongMot.VehVelocityCorr > input->Signals.VehVelocityExtMax)) {
            u_debounceCounter_set += incErrCnt_c;
            u_debounceCounter_Reset = 0U;
        }

        else {
            /* decrement error count */
            u_debounceCounter_set = 0U;
            u_debounceCounter_Reset += incErrCnt_c;
        }
    }

    else if ((VED_GET_IO_STATE(VED__SIN_POS_VEHVEL_MIN, input->Signals.State) ==
              VED_IO_STATE_VALID) &&
             (VED_GET_IO_STATE(VED__SIN_POS_VEHVEL_MAX, input->Signals.State) !=
              VED_IO_STATE_VALID))  // if both the MAX velocity is Not VALID use
                                    // only MIN velo for monitoring
    {
        if (mif->LongMot.VehVelocityCorr < input->Signals.VehVelocityExtMin)

        {
            u_debounceCounter_set += incErrCnt_c;
            u_debounceCounter_Reset = 0U;
        }

        else {
            /* decrement error count */
            u_debounceCounter_set = 0U;
            u_debounceCounter_Reset += incErrCnt_c;
        }

    }

    else if ((VED_GET_IO_STATE(VED__SIN_POS_VEHVEL_MIN, input->Signals.State) !=
              VED_IO_STATE_VALID) &&
             (VED_GET_IO_STATE(VED__SIN_POS_VEHVEL_MAX, input->Signals.State) ==
              VED_IO_STATE_VALID))  // if both the MIN velocity is Not VALID use
                                    // only MAX velo for monitoring
    {
        if (mif->LongMot.VehVelocityCorr > input->Signals.VehVelocityExtMax) {
            u_debounceCounter_set += incErrCnt_c;
            u_debounceCounter_Reset = 0U;
        }

        else {
            /* decrement error count */
            u_debounceCounter_set = 0U;
            u_debounceCounter_Reset += incErrCnt_c;
        }

    }

    else {
        /* decrement error count */
        u_debounceCounter_set = 0U;
        u_debounceCounter_Reset = VED__PAR_VMON_CYCLE_IN;
    }

    /* test counter thresholds */
    if (u_debounceCounter_set >= VED__PAR_VMON_CYCLE_OUT_LT) {
        /* Counter above long-term limit and Velocity value is out of range*/
        u_debounceCounter_set =
            MIN(u_debounceCounter_set, VED__PAR_VMON_CYCLE_OUT_LT + 2);

        VED_SET_IO_STATE(VED_SOUT_POS_VEL, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
        proPorts->pVED_Errors->OutPutErrors.VelMon = VED_ERR_STATE_INACTIVE;
        proPorts->pVED_Errors->OutPutErrors.VelMonLt = VED_ERR_STATE_ACTIVE;
    }

    else if (u_debounceCounter_set >= VED__PAR_VMON_CYCLE_OUT) {
        /* Velocity value is out of range and Counter above short limit */
        VED_SET_IO_STATE(VED_SOUT_POS_VEL, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
        proPorts->pVED_Errors->OutPutErrors.VelMon = VED_ERR_STATE_ACTIVE;
        proPorts->pVED_Errors->OutPutErrors.VelMonLt = VED_ERR_STATE_INACTIVE;
    }

    else {
        // Do Nothing
    }

    if (u_debounceCounter_Reset >= VED__PAR_VMON_CYCLE_IN) {
        /* Velocity value is within the range */

        u_debounceCounter_Reset =
            MIN(u_debounceCounter_Reset, VED__PAR_VMON_CYCLE_IN + 2);

        VED_SET_IO_STATE(VED_SOUT_POS_VEL, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
        proPorts->pVED_Errors->OutPutErrors.VelMon = VED_ERR_STATE_INACTIVE;
        proPorts->pVED_Errors->OutPutErrors.VelMonLt = VED_ERR_STATE_INACTIVE;
    }
}

#endif

#if (CFG_VED__FS_VELO_CORR_MON)
#if (VEL_CORR_HIST_STATIONARY_TARGETS)
/* ***********************************************************************
  @fn               VED_FSCheckVelCorrMonitoring */ /*!
  @brief            Monitor velocity based on correction factor

  @description      FS monitor to check if velocity is outside of functional
                    safety accepted tolerance by calculation a correction 
                    factor based on the last 5 static object histograms

                    running standard deviation:
                      sqrt( ( ( sqsum - ((sum*sum)/n) ) / (n-1) )

                    square mean y:
                      y(k+1)=((n-1)*y(k)+x(k+1)*x(k+1))/n
   
  @param[in]        histogram of static objects of last radar cycle
  @return           fault status

  @pre              -
  @post             -
**************************************************************************** */
static void VED_FSCheckVelCorrMonitoring(const RefSpeed_t *histogram) {
    // float32 f32_Local;        /* local buffer for check if square root
    // argument is positive */
    int16 li = 0;       /* index of lowest bin with targets */
    int16 hi = 0;       /* index of highest bin with targets */
    uint16 number = 0U; /* amount of targets in bins */

    /* calculate scattering width (Streuungsbreite) and number of targets */
    for (uint8 i = 0u; i < (uint8)REF_SPEED_NO_BINS; i++) {
        if ((histogram->binVelRatio[i] > 0U) && (li == 0)) {
            li = (int16)i;
        }
        if (histogram->binVelRatio[i] > 0U) {
            hi = (int16)i;
        }
        number = number + histogram->binVelRatio[i];
    }

    /* only add histogramm if not too much scattered */
    if (((hi - li) <= VEL_CORR_FS_HIST_MAX_WIDTH) &&
        (number >= VEL_CORR_FS_HIST_MIN_NUMBER)) {
        float32 fBinsCorrFactor; /* correction factor of histogram column */
        /* add histogram data to fast fault detection */
        fBinsCorrFactor = BINS_CORR_FACTOR_START_VALUE;

        /* add histogram data to fast fault detection */
        for (uint8 i = 0u; i < (uint8)REF_SPEED_NO_BINS; i++) {
            /* add all elements of objects of this velocity deviation */
            for (uint8 j = 0u; j < histogram->binVelRatio[i]; j++) {
                VED_FSVelCorrMon.number += 1.0f;
                VED_FSVelCorrMon.sum += fBinsCorrFactor;
                VED_FSVelCorrMon.sqsum += fBinsCorrFactor * fBinsCorrFactor;
                VED_FSVelCorrMon.sqmean =
                    (((VED_FSVelCorrMon.number - 1.0f) *
                      VED_FSVelCorrMon.sqmean) +
                     (fBinsCorrFactor * fBinsCorrFactor)) /
                    VED_FSVelCorrMon.number;

                /* only calculate standard deviation if more than one sample is
                 * available */
                if (VED_FSVelCorrMon.number > 1.0f) {
                    float32 f32_Local =
                        (VED_FSVelCorrMon.sqsum -
                         ((VED_FSVelCorrMon.sum * VED_FSVelCorrMon.sum) /
                          VED_FSVelCorrMon.number)) /
                        (VED_FSVelCorrMon.number - 1.0f);

                    /* calculate standard deviation if possible */
                    if (TUE_CML_IsNonZero(f32_Local)) {
                        VED_FSVelCorrMon.sa = VED__SQRT(f32_Local);
                    } else {
                        VED_FSVelCorrMon.sa = 100.0F;
                    }
                }
            }
            fBinsCorrFactor += BINS_CORR_FACTOR_STEP;
        }

        /* only calculate mean if at least one sample is available */
        if (fABS(VED_FSVelCorrMon.number) > C_F32_DELTA) {
            VED_FSVelCorrMon.mean =
                VED_FSVelCorrMon.sum / VED_FSVelCorrMon.number;
        } else {
            /* Default value is 1.0 */
            VED_FSVelCorrMon.mean = 1.0f;
        }
    }

    /* check if enough samples are available to check for fault and standard
     * deviation is acceptable */
    if ((VED_FSVelCorrMon.number > VEL_CORR_FS_MON_MIN_NUMBER) &&
        (VED_FSVelCorrMon.sa <= VEL_CORR_FS_MON_MIN_SA)) {
        /* check if short term correction factor is outside of limits and set
         * fault status */
        if ((VED_FSVelCorrMon.mean - 1.0f) < VEL_CORR_FS_MON_MAX_DEV) {
            VED_FSVelCorrMon.fault = VED_ERR_STATE_INACTIVE;
        } else {
            VED_FSVelCorrMon.fault = VED_ERR_STATE_ACTIVE;
        }
    }

    VED_FSVelCorrMon.counter++;

    /* reduce amount of data used for fault detection */
    if (VED_FSVelCorrMon.counter > VEL_CORR_FS_MON_MAX_COUNTER) {
        VED_FSVelCorrMon.number =
            (VED_FSVelCorrMon.number * (float32)VEL_CORR_FS_MON_REDUCTION);
        VED_FSVelCorrMon.sum =
            (VED_FSVelCorrMon.sum * (float32)VEL_CORR_FS_MON_REDUCTION);
        VED_FSVelCorrMon.sqsum =
            (VED_FSVelCorrMon.sqsum * (float32)VEL_CORR_FS_MON_REDUCTION);
        VED_FSVelCorrMon.counter = 1u;
    }
}

#elif (VEL_CORR_ALN)
/* ***********************************************************************
  @fn               VED_FSCheckVelCorrMonitoring*/ /*!
  @brief            Monitor velocity based on correction factor

  @description      FS monitor to check if velocity is outside of functional
                    safety accepted tolerance for 5 cycles
   
  @param[in]        fUncorrectedVelocity input velocity
  @param[in]        fUncorrectedVelocityVar input velocity variance
  @param[in]        fALNVelocity velocity from ALN
  @param[in]        fALNVelocityDev velocity standard deviation from ALN
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_FSCheckVelCorrMonitoring(float32 f_UncorrectedVelocity,
                                         float32 f_UncorrectedVelocityVar,
                                         float32 f_ALNVelocity,
                                         float32 f_ALNVelocityDev,
                                         boolean b_EMProbRTBRecog) {
    /* ALN will only provide a velocity above 20kph, but check if we have a
     * vehicle velocity at all */
    if (f_UncorrectedVelocity > 0.0F) {
        /* Calculate difference and uncertainty of estimated velocities and
         * check for negative variance */
        VED_FSVelCorrMon.f_velDiff = (f_ALNVelocity - f_UncorrectedVelocity);
        VED_FSVelCorrMon.f_threshold =
            f_UncorrectedVelocity * VED__FS_VEL_MON_RAT_THRHD;
        VED_FSVelCorrMon.f_curVelCorr = f_ALNVelocity / f_UncorrectedVelocity;

        if (f_UncorrectedVelocityVar >= 0.0F) {
            float32 f_velRangeDiff;
            float32 f_velDiffAbs = fABS(VED_FSVelCorrMon.f_velDiff);

            VED_FSVelCorrMon.f_varRange =
                VED__FS_VEL_MON_SIGMA *
                (VED__SQRT(f_UncorrectedVelocityVar) + f_ALNVelocityDev);
            f_velRangeDiff =
                fABS(VED_FSVelCorrMon.f_velDiff) - VED_FSVelCorrMon.f_varRange;

#if ((defined(CFG_VED_FS_VEH_CORR_MON_TOGGLING)) && \
     (CFG_VED_FS_VEH_CORR_MON_TOGGLING))
            /* if velocity difference is inside of allowed velocity range,
             * decrement fault count, otherwise increment fault count */
            if ((f_velRangeDiff < VED_FSVelCorrMon.f_threshold) &&
                (f_velDiffAbs < VED__FS_VEL_MON_MAX_VEL_DIFF))
#else
            /* if velocity difference is inside of allowed velocity range,
             * decrement fault count, otherwise increment fault count */
            if ((f_velRangeDiff < VED_FSVelCorrMon.f_threshold) ||
                (f_velDiffAbs < VED__FS_VEL_MON_MAX_VEL_DIFF))
#endif
            {
                if (VED_FSVelCorrMon.u_counter > 0) {
                    VED_FSVelCorrMon.u_counter--;
                }

                VED_FSVelCorrMon.fault = VED_ERR_STATE_INACTIVE;
            }
#if ((defined(CFG_VED_FS_VEH_CORR_MON_TOGGLING)) && \
     (CFG_VED_FS_VEH_CORR_MON_TOGGLING))
            else if ((f_velRangeDiff > VED_FSVelCorrMon.f_threshold) &&
                     (f_velDiffAbs > VED__FS_VEL_MON_MAX_VEL_DIFF))
#else
            else
#endif
            {
                if (VED_FSVelCorrMon.u_counter < VEL_CORR_FS_MON_MAX_COUNTER) {
                    VED_FSVelCorrMon.u_counter++;
                }

                if (VED_FSVelCorrMon.u_counter >= VEL_CORR_FS_MON_MAX_COUNTER) {
                    VED_FSVelCorrMon.fault = VED_ERR_STATE_ACTIVE;
                } else {
                    VED_FSVelCorrMon.fault = VED_ERR_STATE_INACTIVE;
                }
                if (b_EMProbRTBRecog) {
#if (VED_VEH_DYN_INTFVER >= 8U)
                    {
                        if (CORRECTED_EGO_SPEED_IS_ZERO_ON_ROLLER_BENCH != 1) {
                            VED_FSVelCorrMon.fault = VED_ERR_STATE_INACTIVE;
                        }
                    }
#endif
                }
            }
#if ((defined(CFG_VED_FS_VEH_CORR_MON_TOGGLING)) && \
     (CFG_VED_FS_VEH_CORR_MON_TOGGLING))
            else {
                /* Do Nothing */
            }
#endif
        }
    }
}

#if (ALN_MONITORING_VERSION > 3U) && (CFG_VED__FS_VELO_CONF_MON_FAULT)
/* ***********************************************************************
  @fn               VED_FSConfirmVelocity */ /*!
  @brief            Monitor velocity based on confirmation velocity from ALN

  @description      FS monitor to check if velocity is outside of functional
                    safety accepted tolerance for 5 cycles
   
  @param[in]        fCorrectedVelocity input velocity
  @param[in]        fCorrectedVelocityVar input velocity variance
  @param[in]        fALNVelocity confirmation velocity from ALN
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_FSConfirmVelocity(float32 f_CorrectedVelocity,
                                  float32 f_CorrectedVelocityVar,
                                  float32 f_ALNVelocity,
                                  boolean b_EMProbRTBRecog) {
    /* ALN will only provide a velocity above 20kph, but check if we have a
     * vehicle velocity at all */
    if (f_CorrectedVelocity > 0.0F) {
        /* Calculate difference of estimated velocities if variance is not
         * negative */
        if (f_CorrectedVelocityVar >= 0.0F) {
            /* calculate velocity difference but consider variance as velocity
             * gets more inacurate during braking or acceleration */
            VED_FSVelCorrMon.f_VelConfDiff =
                fABS(f_CorrectedVelocity - f_ALNVelocity);

            /* if velocity difference is inside of allowed velocity range,
             * decrement fault count, otherwise increment fault count */
            if (VED_FSVelCorrMon.f_VelConfDiff <
                (VED__FS_VEL_CONF_MAX_VEL_DIFF +
                 (VED__SQRT(f_CorrectedVelocityVar)))) {
                if (VED_FSVelCorrMon.u_ConfCounter > 0) {
                    VED_FSVelCorrMon.u_ConfCounter--;
                }

                VED_FSVelCorrMon.confFault = VED_ERR_STATE_INACTIVE;
            } else {
                if (VED_FSVelCorrMon.u_ConfCounter <
                    VEL_CORR_FS_MON_MAX_COUNTER) {
                    VED_FSVelCorrMon.u_ConfCounter++;
                }

                if (VED_FSVelCorrMon.u_ConfCounter >=
                    VEL_CORR_FS_MON_MAX_COUNTER) {
                    VED_FSVelCorrMon.confFault = VED_ERR_STATE_ACTIVE;
                } else {
                    VED_FSVelCorrMon.confFault = VED_ERR_STATE_INACTIVE;
                }
                if (b_EMProbRTBRecog) {
#if (VED_VEH_DYN_INTFVER >= 8U)
                    {
                        if (CORRECTED_EGO_SPEED_IS_ZERO_ON_ROLLER_BENCH != 1) {
                            VED_FSVelCorrMon.confFault = VED_ERR_STATE_INACTIVE;
                        }
                    }
#endif
                }
            }
        }
    }
}
#endif
#endif
#elif ((!CFG_VED__FS_VELO_CORR_MON) && (CFG_VED__DO_VELOCITY_CORR))
/* ***********************************************************************
  @fn               VED_UncorrVelMonitoring*/ /*!
  @brief            Monitor velocity 

  @description      Flag to check if ALN velocity and VED velocity are alligned for the first time after startup
                    based on which the correction factor variance will be switched from 0.0225 to the computed value
   
  @param[in]        fUncorrectedVelocity input velocity
  @param[in]        fUncorrectedVelocityVar input velocity variance
  @param[in]        fALNVelocity velocity from ALN
  @param[in]        fALNVelocityDev velocity standard deviation from ALN
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_UncorrVelMonitoring(float32 f_UncorrectedVelocity,
                                    float32 f_UncorrectedVelocityVar,
                                    float32 f_ALNVelocity,
                                    float32 f_ALNVelocityDev) {
    float32 f_velDiff;
    float32 f_threshold;
    float32 f_varRange;

    /* ALN will only provide a velocity above 20kph, but check if we have a
     * vehicle velocity at all */
    if (f_UncorrectedVelocity > 0.0F) {
        /* Calculate difference and uncertainty of estimated velocities and
         * check for negative variance */
        f_velDiff = (f_ALNVelocity - f_UncorrectedVelocity);
        f_threshold = f_UncorrectedVelocity * VED__FS_VEL_MON_RAT_THRHD;

        if (f_UncorrectedVelocityVar >= 0.0F) {
            float32 f_velRangeDiff;
            float32 f_velDiffAbs = fABS(f_velDiff);

            f_varRange =
                (float32)(4.0 * (float32)(VED__SQRT(f_UncorrectedVelocityVar) +
                                          f_ALNVelocityDev));
            f_velRangeDiff = fABS(f_velDiff) - f_varRange;

            /* if velocity difference is inside of allowed velocity range,
             * decrement fault count, otherwise increment fault count */
            if ((f_velRangeDiff < f_threshold) || (f_velDiffAbs < 2.0)) {
                b_init_CorrFlag = TRUE;
            }
        }
    }
}
#endif
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */