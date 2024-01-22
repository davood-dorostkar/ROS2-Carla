

#ifndef _CP_H_INCLUDED
#define _CP_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "vlc_sen.h"
#include "vlc_glob_ext.h"

#include "cp_cfg.h"
#include "cp_ext.h"

#include "fip_ext.h"
#include "TM_Global_Types.h"
#include "vlcSen_consts.h"

/*****************************************************************************
  SYMBOLISCHE KONSTANTEN (KOMPONENTENINTERN)
*****************************************************************************/

/*****************************************************************************
  MACROS (KOMPONENTENINTERN)
*****************************************************************************/
/*! @brief  Utility define for the number of traces in the system */
#define CP_NUM_TRACES (TraceID_t) FIP_STATIC_TRACE_NO_OF_TRACES

/*! @brief Minimum length of trace to used in CP */
#define CP_NUM_TRACE_MIN_NUMBER_OF_POINTS (3u)

/*! @brief sTRAJ_C0 */
#define sTRAJ_C0 (0u)
/*! @brief sTRAJ_C1 */
#define sTRAJ_C1 (1u)
/*! @brief CP_INFINITE_DIST */
#define CP_INFINITE_DIST (999.f)
/*! @brief CP_INVALID_INLAP */
#define CP_INVALID_INLAP (-999.f)
/*! @brief CP_INVALID_INLAP_VAR */
#define CP_INVALID_INLAP_VAR (1.0E10f)

/*! @brief Maximum time for velocities close to zero */
#define CP_TRAJ_VAR_TIMEGAP_MAX (10.0f)
/*! @brief Maximum time for velocities close to zero */
#define CP_TRAJ_VAR_OFFSET (0.00003f)

/*! @brief Time-gap limit for the approximation of the road, trace or VDY
 * variance by the first segment */
#define CP_TRAJ_VAR_APPROX_SEGMENT_TIMEGAP_LOW (1.0f)
/*! @brief Time-gap limit for the approximation of the road, trace or VDY
 * variance by the second segment */
#define CP_TRAJ_VAR_APPROX_SEGMENT_TIMEGAP_MID (4.0f)

/*! @brief Coefficient for the approximation of the VDY course variance by a
 * power function for first two segments */
#define CP_VDY_COURSE_VAR_COEFF_LOW (0.02f)

/*! @brief Maximum ego velocity for country road specific approximation of the
 * trace trajectory course variance */
#define CP_TRACE_TRAJ_VEGO_MAX_COUNTRY (120.f / C_KMH_MS)
/*! @brief Maximum curvature for country road specific approximation of the
 * trace trajectory course variance */
#define CP_TRACE_TRAJ_CURVE_MAX_HIGHWAY (1.f / 600.f)
/*! @brief Coefficient for the approximation of the trace trajectory course
 * variance on highways */
#define CP_TRACE_TRAJ_COURSE_VAR_COEFF_HIGHWAY (0.1f)
/*! @brief Coefficient for the approximation of the trace trajectory course
 * variance for first two segments on country roads */
#define CP_TRACE_TRAJ_COURSE_VAR_COEFF_COUNTRY_LOW (0.045f)
/*! @brief Coefficient for the approximation of the trace trajectory course
 * variance for third segment on country roads */
#define CP_TRACE_TRAJ_COURSE_VAR_COEFF_COUNTRY_HIGH (0.0056f)

/*! @brief Coefficient for the approximation of the road course variance by a
 * power function for first two segments */
#define CP_ROAD_COURSE_VAR_COEFF_LOW (0.1f)
/*! @brief Coefficient for the approximation of the road course variance by a
 * power function for third segment */
#define CP_ROAD_COURSE_VAR_COEFF_HIGH (0.025f)

/*! @brief TRACE_NUM_BINS */
#define TRACE_NUM_BINS MAX_NB_TRAJ_SAMPLES
/*! @brief SAMPLE_RESOLUTION_X */
#define SAMPLE_RESOLUTION_X (CP_SAMPLEDIST_MAX / (float32)TRACE_NUM_BINS)

/*! @brief       Total number of trace samples
    @general     Total number of trace samples given by [number of traces *
   number of sample point per trace]
    @conseq      @incp  Either the number of traces has increased or the number
   of sample points per trace has witnessed an increase
                 @decp  Either the number of traces has decreased or the number
   of sample points per trace has witnessed an decrease
    @attention   Any changes will have to checked for all functions using traces
    @typical     200   @unit No-unit     @min 0   @max 200   */
#define CP_NUM_TRACE_SAMPLES \
    (MAX_NB_TRAJ_SAMPLES * FIP_STATIC_TRACE_NO_OF_TRACES)
/*! @brief CP_KALMAN_POSAMPLES_NOISE_OFFSET */
#define CP_KALMAN_POSAMPLES_NOISE_OFFSET 0.75f

/*****************************************************************************
  TYPEDEFS (KOMPONENTENINTERN)
*****************************************************************************/
/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief eCPSampleSlopQuality_t enum

    @general None

    @conseq None

    @attention None

    */
typedef enum {
    CP_SSQ_INIT,
    CP_SSQ_LOW_QUALITY,
    CP_SSQ_HIGH_QUALITY
} eCPSampleSlopQuality_t;

/* ****************************************************************
    TYPEDEF STRUCT CPTrace2CurveParallelism_t
    **************************************************************** */
/*! @brief CPTrace2CurveParallelism_t structure

    @general CPRESTRUCTED: Renamed typedef struct MOTrace2CurveParallelism_t ->
   CPTrace2CurveParallelism_t

    @conseq None

    @attention None

    */

/*! Structure used to store trace additional data */
typedef struct CPTraceScoreAddData {
    ObjNumber_t iObjNr;    /*!< Object number of the trace */
    uint8 uNumberOfPoints; /*!< Number of trace points we have already seen */
    uint8 uApproxUseLen;   /*!< Number of indices to use for approximation */
    /*uint8             uTraceLaneIdx;*/ /*!< Trace lane index assigned */
    float32 fApproxCloseDist; /*!< Close distance from where polynomial approx
                                 valid */
    float32
        fApproxFarDist; /*!< Far distance till where polynomial approx valid */
    float32
        fCurSqDist; /*!< summed squared distance of the sample points to the
                       approximation polynomial (the residual) */
    CPTracePolyL2_t ApproxPoly; /*!< Trace approximation polynomial */
    boolean
        isValid; /*!< Flag to indicate whether the trace polynomial is valid */
    boolean
        bUseTraceForFusion; /*!< Flag to indicate whether the trace is used
                               within the trajectory fusion */
} CPTraceAddData_t;

/*! Structure used to store data which reflects the relationship of traces with
 * road, VDY course and overall trace quality thereby */
typedef struct {
    CPTrace2CurveParallelism_t Trace_EGOPara[FIP_STATIC_TRACE_NO_OF_TRACES];
    uint8 Trace_QualityValid[FIP_STATIC_TRACE_NO_OF_TRACES];
} CP_t_InputSourceParams;

/*! Structure used to store position samples from different input sources */
typedef struct {
    CPPosSamples_t
        EGO_PosSamples; /*!< Ego motion trajectory samples, formerly global
                           TPSIMU_EGO_PosSamples ->
                           CPTrajFusionDebugInfo.EGO_PosSamples */
    CPPosSamples_t TRACE_PosSamples; /*!> Trace estimation trajectory samples */

} CP_t_TrajectoryPosSamples;

/*****************************************************************************
  KONSTANTEN (KOMPONENTENINTERN)
*****************************************************************************/

/*! SI_TRACE_QUAL_MASK_TRACE_VALID */
#define SI_TRACE_QUAL_MASK_TRACE_VALID 0x01
/*! SI_TRACE_QUAL_MASK_EGO_PARALLEL */
#define SI_TRACE_QUAL_MASK_EGO_PARALLEL 0x02
/*! SI_TRACE_QUAL_MASK_ROAD_PARALLEL */
#define SI_TRACE_QUAL_MASK_ROAD_PARALLEL 0x04

/*****************************************************************************
  VARIABLEN (KOMPONENTENINTERN)
*****************************************************************************/

/*! Additional data stored for traces (one entry per trace) */
extern MEMSEC_REF CPTraceAddData_t CPTraceAdd[CP_NUM_TRACES];
/* VLC_CFG_OBJECT_TRACE_PREPROCESSSING */
/*****************************************************************************
  FUNKTIONEN (KOMPONENTENINTERN)
*****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/* cp_trajectory.c */
extern float32 CPGetTraceTrajVariance(const float32 f_DistX,
                                      const float32 f_EgoSpeedCorrected);
extern float32 CPGetCourseVariance(const float32 f_DistX,
                                   const float32 f_EgoSpeedCorrected);
extern float32 CPGetRoadVariance(const float32 f_DistX,
                                 const float32 f_EgoSpeedCorrected);
extern float32 CPGetPlausibleC1AtSpeed(const fVelocity_t EgoSpeedCorrected);
extern float32 CPGetCourseGradUpdateSTDC1AtSpeed(
    const fVelocity_t EgoSpeedCorrected);
extern void CPCalculateObjectReference(
    ObjNumber_t iObj,
    CPTrajectoryData_t const *pTrajData,
    CPTrajRefPoint_t *pTrajRefPoint,
    CPTrajRefPoint_t *pTrajLastCycRefPoint,
    CPTrajRefPoint_t *pTrajRefPointLastCycle);
/* CPRESTRUCTED: Added function declarations for custom functions that are used
 * by CD and SI */
extern float32 CPGetCurvature(CPCourseData_t const *const pTrajectory,
                              const float32 fXPosition);

extern void CPSampleGradFromCourse(const CPCourseData_t *pCourseData,
                                   CPGradSamples_t *pSamples);
extern CPTrajRefPoint_t CPCalculateDistancePoint2Clothoid(const float32 fX,
                                                          const float32 fY,
                                                          const float32 fC0,
                                                          const float32 fC1);
extern void CPSamplePosClothApprox(const CPCourseData_t *pCPCourseData,
                                   const eCPClothApproxType_t CPClothApproxType,
                                   const float32 fSampleDistMax,
                                   CPPosSamples_t *pCPPosSamples);

extern void CPMoveSamplesFromCoGToSensor(CPPosSamples_t *pCPPosSamples);
extern void CPLimitSamplesXDist(const float32 fXDistMax,
                                CPPosSamples_t *pCPPosSamples);

extern CPCourseData_t CPGetTraceEstimationAsCourseData(
    const CPTraceTrajectory_t *pTraceTrajectory);
extern void CPFusionEvalTraceSamples(
    const CPTraceTrajectory_t *pTraceTrajectory,
    CPPosSamples_t *pTRACE_PosSamples,
    CPTrajectoryData_t *pTrajData);
extern void CPCallVLCFreezeforTraceTraject(
    CPTraceTrajectory_t *p_CPTraceTrajectoryInfo);

extern void CPTrajSampleFusionMain(
    CPTrajectoryData_t *pTrajData,
    CP_t_TrajectoryPosSamples *pTrajectoryPosSamples);

/* cp_lineseg.c */
extern fDistance_t CPCalculatePoint2SamplesDist(const CPPosSamples_t *pSamples,
                                                fDistance_t fPointX,
                                                fDistance_t fPointY,
                                                boolean bExtendSampleEnds);

/* cp_fusion_trace.c */
extern void CPFusionTraceMain(const CPCourseData_t *pCourseData,
                              const uint8 *pTrace_QualityValid,
                              CPTrajectoryData_t *pTrajData);
extern void CPFusionTracesEvalTraces(const CPCourseData_t *pCourseData,
                                     CP_t_InputSourceParams *pTrace_Params);

extern void CPCalculateCombinedTraceTrajectory(
    CPTraceTrajectory_t *pTraceTrajectory,
    const CPCourseData_t *pCourseData,
    const CPTrajectoryData_t *pTrajData);
extern boolean CPFusionTraceIsFusionSituation(CPTrajectoryState_t *pTrajState);
extern void CPCalculateTraceSamples(const CPTraceTrajectory_t *pTraceTrajectory,
                                    CPPosSamples_t *pTRACE_PosSamples);
extern CPTraceTrajectory_t CPTraceTrajectory;

/* cp_fusion_cam.c */
extern void CPFusionCamMain(CPTrajectoryData_t *pTrajData);

#ifdef __cplusplus
};
#endif

/* Ende der bedingten Einbindung */
#else

#endif
