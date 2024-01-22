
/*!
   @defgroup cp CP (Course Prediction)
   @ingroup vlc_sen


@{ */

#ifndef _CP_EXT_INCLUDED
#define _CP_EXT_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/

/* Note: the below file is a dependency of this header, but currently through
include hierarchy considerations, this file is only included after that file
has already been pre-processed */
#include "cp_cfg.h"

// add
//#include "TM_Global_Types.h"
#include "TM_Global_Const.h"
#include "vlcSen_consts.h"
#include "vlcSen_common_utils.h"
#include "TM_Math_Cal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/
#define CP_TRAJ_INVALID_VALUE 999.f

/*! @brief CP_SW_VERSION_NUMBER */
#define CP_SW_VERSION_NUMBER 0x000203u
/*! @brief NB_TRAJ_STATE */
#define NB_TRAJ_STATE (uint8)(2)
/*! @brief NB_TRAJ_STATE2 */
#define NB_TRAJ_STATE2 (uint8)(2u * NB_TRAJ_STATE)
/*! @brief MAX_NB_TRAJ_SAMPLES */
#define MAX_NB_TRAJ_SAMPLES (20u)

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
/*! @brief distance and deviation of object trace to vdy or road curve */
typedef struct {
    float32 fDistMean;   /*!< fDistMean */
    float32 fDistStdDev; /*!< fDistStdDev */
} CPTrace2CurveParallelism_t;

/*! @brief operating modes of sub-component */
typedef enum {

    CP_INIT, /*!< initialize all */
    CP_OK    /*!< normal processing */
} CPState_t;

/*! @brief eCPClothApproxType_t */
typedef enum {
    CPClothApproxType_Automatic = 0,
    CPClothApproxType_PolynomialOnly = 1,
    CPClothApproxType_CircleOnly = 2
} eCPClothApproxType_t;

/*! @brief CPNoiseModelLinear_t */
typedef struct {
    float32 fNoiseOffset;   /*!< fNoiseOffset */
    float32 fNoiseGradient; /*!< fNoiseGradient */
} CPNoiseModelLinear_t;

/*! @brief GDBTrajectoryData_t */
typedef struct {
    float32 fTrajC0;    /*!< fTrajC0 */
    float32 fTrajC1;    /*!< fTrajC1 */
    float32 fTrajAngle; /*!< fTrajAngle */
} GDBTrajectoryData_t;

/*! @brief CPTrajRefPoint_t */
typedef struct {
    float32 fX;          /*!< Trajectory reference point X coordinate */
    float32 fY;          /*!< Trajectory reference point Y coordinate */
    float32 fDistToTraj; /*!< Distance to trajectory @unit:m */
    float32 fDistOnTraj; /*!< Distance from vehicle center of gravity to object
                            on trajectory @unit:m */
} CPTrajRefPoint_t;

/*! @brief Holds the recursive values of the Distance Kalman filter */
typedef struct {
    GDBVector2_t X;    /*!< kalman states (distance velocity) */
    GDBSymMatrix2_t P; /*!< kalman estimation covariance matrix */
} CPTrajDistKalmanData_t;

/*! @brief CPTrajDistKalmanMeas_t */
typedef struct {
    GDBVector1_t Y;    /*!< GDBVector1_t */
    GDBSymMatrix1_t R; /*!< GDBSymMatrix1_t */
} CPTrajDistKalmanMeas_t;

/*! @brief CPObjDist2Traj_t */
typedef struct {
    CPTrajDistKalmanData_t TrajDistFilt; /*!< TrajDistFilt */
    CPTrajDistKalmanMeas_t TrajDistMeas; /*!< TrajDistMeas */
    float32 ObjDistOnTraj; /*!< Distance from vehicle center of gravity to
                              object on trajectory @unit:m */
} CPObjDist2Traj_t;

/*! @brief CPObjDist2TrajMeas_t */
typedef CPTrajDistKalmanMeas_t CPObjDist2TrajMeas_t;

/*! @brief CPCourseData_t */
typedef struct CPCourseData {
    fCurve_t fCurve;               /*!< fCurve */
    float32 fCurveVar;             /*!< fCurveVar */
    fGradient_t fCurveGradient;    /*!< fCurveGradient */
    float32 HeadingAngle;          /*!< HeadingAngle */
    float32 SideSlipAngle;         /*!< SideSlipAngle */
    float32 SideSlipAngleVariance; /*!< SideSlipAngleVariance */
} CPCourseData_t;

/*! @brief CPTraceTrajectory_t */
typedef struct {
    float32 fCurve;            /*!< fCurve */
    float32 fMaxDist;          /*!< fMaxDist */
    float32 fMeanSquaredError; /*!< fMeanSquaredError */
    float32 fWeight;           /*!< fWeight */
} CPTraceTrajectory_t;

/*! @brief CPTrajKafiData_t */
typedef struct CPTrajKafiData {
    float32 AMatrix[NB_TRAJ_STATE *
                    NB_TRAJ_STATE]; /*!< full matrix, transition matrix */
    float32 QMatrix[NB_TRAJ_STATE]; /*!< diagonal matrix, process noise
                                       covariance     */
    float32 XVector[NB_TRAJ_STATE]; /*!< Vector, estimated state vector */

    float32 UDMatrix[(NB_TRAJ_STATE * (NB_TRAJ_STATE + 1)) /
                     2]; /*!< UDUT factorized matrix, covariance            */
    float32 XsVector[NB_TRAJ_STATE];    /*!< Vector, predicted state vector */
    float32 PDiagMatrix[NB_TRAJ_STATE]; /*!< PDiagMatrix              */
} CPTrajKafiData_t;

/*! @brief CPTrajectoryConfig_t */
typedef struct {
    ubit8_t UseRoadEstim : 1; /*!< Bit indicating if road estimation shall be
                                 used for fusion */
    ubit8_t UseObjTraces : 1; /*!< Bit indicating if object traces shall be used
                                 for fusion */
    ubit8_t UseCamLane : 1; /*!< Bit indicating if camera lane shall be used for
                               trajectory fusion */
} CPTrajectoryConfig_t;

/*! @brief CPTrajectoryInputParameter_t */
typedef struct {
    boolean bSuppressFusion; /*!< Fusion will be suppressed */
} CPTrajectoryInputParameter_t;

/*! @brief CPTrajectoryState_t */
typedef struct {
    /* !!!ATTENTION!!! Do not change order of bitfield entries because of
     * initialization downward compatibility */
    ubit16_t EgoCourseOnly : 1; /*!< Bit indicating that only ego course used
                                   for trajectory (fusion off) */
    ubit16_t FusionRoadstimation : 1; /*!< Bit indicating that RE is used for
                                         trajectory fusion */
    ubit16_t FusionTraces : 1; /*!< Bit indicating that Traces are used for
                                  trajectory fusion */
    ubit16_t FusionTraceHystEgoSpeed : 1;  /*!< Bit indicating that ego speed is
                                              sufficient for Traces fusion */
    ubit16_t FusionHystLRoadRangeConf : 1; /*!< Bit indicating that features
                                              from left side of road estimation
                                              fulfill fusion criteria */
    ubit16_t FusionHystRRoadRangeConf : 1; /*!< Bit indicating that features
                                              from right side of road estimation
                                              fulfill fusion criteria */
    ubit16_t FusionHystEgoSpeed : 1;  /*!< Bit indicating that ego speed fusion
                                         criteria is fulfilled */
    ubit16_t FusionHystEgoRadius : 1; /*!< Bit indicating that ego radius fusion
                                         criteria is fulfilled */
    ubit16_t FusTraceCrit : 1; /*!< Bit set when if CPMOTraceFusionCriterion
                                  returns TRUE */
    ubit16_t FusionPlaus : 1;  /*!< Bit to non-zero if CPFusionPlausibilityCheck
                                  returns TRUE */
    ubit16_t CamLaneQualityHigh : 1; /*!< Bit indicating if camera lane
                                        foresight/quality criteria met */
    ubit16_t CamLaneFusion : 1;  /*!< Bit when camera lane fusion is active */
    ubit16_t NaviPathFusion : 1; /*!< Bit when navi path fusion is active */
} CPTrajectoryState_t;

/*! @brief CPTrajectoryData_t */
typedef struct CPTrajectoryData {
    CPTrajectoryConfig_t Config; /*!< Config */
    CPTrajectoryState_t State;   /*!< State */
    GDBTrajectoryData_t Current; /*!< Trajectory data from the current cycle */
    GDBTrajectoryData_t LastCycle;   /*!< Trajectory data from the last cycle */
    float32 fDistXUnplausibleSCurve; /*!< fDistXUnplausibleSCurve */
    float32 fRoadEstFusRange;        /*!< fRoadEstFusRange */
    float32 fMovObjFusRange;         /*!< fMovObjFusRange */
    float32 fMSETrace;               /*!< fMSETrace */
    boolean bTraceFusionActiveLastCycle; /*!< bTraceFusionActiveLastCycle */
    float32 fCamLaneFusionTimer; /*!< Down counting camera lane fusion timer */
    float32 fEgo2CamDistX; /*!< Ego curve to camera lane blending X distance */
    float32 fBaseY;        /*!< Cam lane fusion Y displacement base */
    float32 fFilteredDIC;  /*!< Filtered driver intended curvature */
    GDBBaseKafi_t KafiEst; /*!< The Kalman filter base data (matrix
                              representation information) */
    CPTrajKafiData_t
        KafiData; /*!< The Kalman filter data (the matrices/vectors themselves)
                   */
} CPTrajectoryData_t;

/*! @brief CPDistanceWidth_t */
typedef struct CPDistanceWidth {
    float32 fDistance;           /*!< fDistance */
    float32 fDistanceVar;        /*!< fDistanceVar */
    float32 fTrajectoryWidth;    /*!< fTrajectoryWidth */
    float32 fTrajectoryWidthVar; /*!< fTrajectoryWidthVar */
    float32 fObjectWidth;        /*!< fObjectWidth */
    float32 fObjectWidthVar;     /*!< fObjectWidthVar */
    float32
        fObjectCorridorWidth;  // considering the object direction, calculate
                               // the projection on the corridor of ego vehicle
    float32 fObjectCorridorWidthVar;
} CPDistanceWidth_t;

/*! @brief CPTrajInlap_t */
typedef struct CPTrajInlap {
    float32 fInlap;    /*!<fInlap */
    float32 fInlapVar; /*!<fInlapVar */
} CPTrajInlap_t;

/*! @brief CPTrajOccupancy_t */
typedef struct CPTrajOccupancy {
    float32 fOverlap;                /*!<fOverlap */
    float32 fOverlapVar;             /*!<fOverlapVar */
    float32 fObjectOccupancy;        /*!<fObjectOccupancy */
    float32 fObjectOccupancyVar;     /*!<fObjectOccupancyVar */
    float32 fTrajectoryOccupancy;    /*!<fTrajectoryOccupancy */
    float32 fTrajectoryOccupancyVar; /*!<fTrajectoryOccupancyVar */
} CPTrajOccupancy_t;

/*! @brief CPTrajectoryMeas_t */
typedef struct CPTrajectoryMeas {
    float32 fx;                  /*!<fx */
    float32 fy;                  /*!<fy */
    float32 fphi;                /*!<fphi */
    float32 fc0;                 /*!<fc0 */
    float32 fc1;                 /*!<fc1 */
    CPTrajectoryConfig_t Config; /*!<Config */
    CPTrajectoryState_t State;   /*!<State */
} CPTrajectoryMeas_t;

/*! @brief Structure used to freeze course data for a given component */
typedef struct CPTrajMeasInfo {
    CPTrajectoryMeas_t Trajectory; /*!<Trajectory */
    CPTrajectoryMeas_t FiltCourse; /*!<FiltCourse */
    float32 LaneWidth;             /*!<LaneWidth */
} CPTrajMeasInfo_t;

/*! @brief  CPTraj2MOTraceParallelism */
typedef struct CPTraj2MOTraceParallelism {
    boolean relevantTrace; /*!<relevantTrace */
    float32 fSADistMean;   /*!<fSADistMean */
    float32 fSADistStdDev; /*!<fSADistStdDev */
} CPTraj2MOTraceParallelism_t;

/*! @brief Structure storing the second degree polynomial approximation of a
trace,
using y = f(x) = fC0 + fC1 * x + fC2 * x^2 */
typedef struct CPTracePolyL2 {
    float32 fC0;     /*!<fC0 */
    float32 fC1;     /*!<fC1 */
    float32 fC2;     /*!<fC2 */
    boolean isValid; /*!<isValid */
} CPTracePolyL2_t;

/*! @brief Structure for the third order polynomial parameter mean values,
variances and residual variance
using y = f(x) = fC0 + fC1 * x + fC2 * x^2 + fC3 * x^3 */
typedef struct {
    float32 fC0;
    float32 fC1;
    float32 fC2;
    float32 fC3;
    float32 fC0Var;
    float32 fC1Var;
    float32 fC2Var;
    float32 fC3Var;
    float32 fXmin;
    float32 fXmax;
    boolean bValid;
} CPTracePolyL3_t;

/*! @brief Structure storing predicted course information per object */
typedef struct VLCCoursePredTag {
    CPObjDist2Traj_t
        TrajDist; /*!< Information about objects relation to estimated course */
} VLCCoursePred_t;

/*! @brief Structure storing position samples of trajectories */
typedef struct CPSamplesTag {
    float32 fx[MAX_NB_TRAJ_SAMPLES];   /*!< x-Position */
    float32 fy[MAX_NB_TRAJ_SAMPLES];   /*!< y-position */
    float32 fVar[MAX_NB_TRAJ_SAMPLES]; /*!< variance information */
    uint8 nb_samples;                  /*!< Number of samples */
} CPPosSamples_t;

/*! @brief Structure storing gradient samples of trajectories */
typedef struct {
    float32 fx[MAX_NB_TRAJ_SAMPLES];             /*!< x-position */
    float32 fdydx[MAX_NB_TRAJ_SAMPLES];          /*!< Gradient at x-position */
    float32 fdydxMinStdDev[MAX_NB_TRAJ_SAMPLES]; /*!< Standard deviation */
    boolean valid[MAX_NB_TRAJ_SAMPLES]; /*!< Boolean to describe if sample if
                                           valid */
} CPGradSamples_t;

/*****************************************************************************
  CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
/*! @cond Doxygen_Suppress */
extern MEMSEC_REF CPState_t CPState;

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

extern CPTrajRefPoint_t CPCalculateDistancePoint2Circle(float32 fX,
                                                        float32 fY,
                                                        float32 fC0);

/*
  cp_trajectory.c
*/
extern void CPTrajectoryInit(const CPCourseData_t *pCourseData,
                             boolean bUseRoadEstim,
                             boolean bUseObjTraces,
                             boolean bUseCamLaneMarker,
                             boolean bUseNaviPath,
                             CPTrajectoryData_t *pTrajData);
extern void CPCalculateTrajectory(
    const CPCourseData_t *pCourseData,
    const CPTrajectoryInputParameter_t *pTrajInputPara,
    CPTrajectoryData_t *pTrajData);
extern void CPPrepareTrajectoryData(CPTrajectoryData_t *pTrajData);

/*
  cp_traj2objrelation.c
*/

extern void CPCalculateOverlap(const CPDistanceWidth_t *pDistanceWidth,
                               CPTrajOccupancy_t *pOccupancy);
extern void CPCalculateInlap(const CPDistanceWidth_t *pDistanceWidth,
                             CPTrajInlap_t *pInlap);

extern void CPCalculateDistance2Traj(const float32 fX,
                                     const float32 fY,
                                     const boolean bUseEgoCourseOnly,
                                     const GDBTrajectoryData_t *pTrajData,
                                     CPTrajRefPoint_t *pDist2Traj);
extern void CPInitObjDist2Traj(const CPObjDist2TrajMeas_t *pObjDist2TrajMeas,
                               CPObjDist2Traj_t *pObjDist2TrajDist);
extern void CPUpdateObjDist2Traj(const ObjNumber_t iObj,
                                 const float32 fMaxAccelDist2Traj,
                                 CPTrajectoryData_t const *pTrajData,
                                 CPObjDist2Traj_t *pObjDist2TrajDist);
extern void CPCopyTraj2Meas(const CPTrajectoryData_t *pTrajectoryData,
                            CPTrajectoryMeas_t *pMeasTrajectory);
extern void CPCopyCourse2Meas(const CPCourseData_t *pCourseData,
                              CPTrajectoryMeas_t *pMeasTrajectory);
extern void CPGetCourseDataEgo(
    CPCourseData_t *pCourseData,
    const boolean bUseSlipAngle /*, const VED_VehDyn_t * pInputSignals */);
extern void CPGetTracePoly(CPTracePolyL2_t *pTracePoly, const TraceID_t iTr);
extern float32 CPTrajGetObjToRefDistMeas(
    const CPObjDist2Traj_t *pObjDist2TrajDist);
extern float32 CPTrajGetObjToRefDistMeasVar(
    const CPObjDist2Traj_t *pObjDist2TrajDist);
extern float32 CPTrajGetObjToRefDistFilt(
    const CPObjDist2Traj_t *pObjDist2TrajDist);
extern float32 CPTrajGetObjToRefDistFiltVar(
    const CPObjDist2Traj_t *pObjDist2TrajDist);
extern float32 CPTrajGetObjToRefDistGradFilt(
    const CPObjDist2Traj_t *pObjDist2TrajDist);
extern float32 CPTrajGetObjToRefDistGradFiltVar(
    const CPObjDist2Traj_t *pObjDist2TrajDist);
extern float32 CPTrajGetObjDistOnTraj(
    const CPObjDist2Traj_t *pObjDist2TrajDist);
extern void CPDeleteObjDist2Traj(CPObjDist2Traj_t *pObjDist2TrajDist);

extern CPTraj2MOTraceParallelism_t CPTraj2MOTraceParallelismCheck(
    TraceID_t iTr, const CPTrajectoryData_t *pTrajectoryData);
extern void CPCDProcessTrajectoriesMeas(CPTrajMeasInfo_t *pTrajectoriesMeas);

extern void CPInitPosSamples(CPPosSamples_t *pSamples);
extern void CPInitGradSamples(CPGradSamples_t *pSamples);

/*
  cp_main.c
*/
extern void VLCCPProcess(void);

/*
  cp_lineseg.c
*/
extern fDistance_t CPCalculatePoint2LineSegListDist(
    const GDBVector2_t pSamples[],
    uint8 NumSamples,
    fDistance_t fPointX,
    fDistance_t fPointY,
    boolean bExtendEndLineSegs);
extern fDistance_t CPCalculatePoint2LineSegCoordArr(const float32 *pSamplesX,
                                                    const float32 *pSamplesY,
                                                    uint8 NumSamples,
                                                    fDistance_t fPointX,
                                                    fDistance_t fPointY,
                                                    boolean bExtendEndLineSegs);
extern void CPCalculateRadii(const float32 pSamplesX[],
                             const float32 pSamplesY[],
                             uint8 NumSamples,
                             fDistance_t *pfRadius);

extern void CPCalcPointApproxPolyL2(CPTracePolyL2_t *pPoly,
                                    const float32 pafX[],
                                    const float32 pafY[],
                                    uint8 uNumPts);
extern void CPCalcPointApproxPolyL2Ext(CPTracePolyL2_t *pPoly,
                                       const float32 pafX[],
                                       const float32 pafY[],
                                       const float32 *pafYStdDev,
                                       uint8 uNumPts);

extern float32 CPCalcDistToPolyL2Y(const CPTracePolyL2_t *const pPoly,
                                   const float32 fX,
                                   const float32 fY);
extern float32 CPCalcSumSqDistance(const CPTracePolyL2_t *const pPoly,
                                   const float32 *pfPointX,
                                   const float32 *pfPointY,
                                   uint8 uNumPoints);

/* --- cp_fusion_navi.c --- */

/* --- cp_fusion_trace.c-- */
extern CPTrace2CurveParallelism_t CPFusionTraceCalcTraceCircleParallelism(
    uint8 iTr, float32 fC0, const GDBTrafoMatrix2D_t *MOT2CIR);
extern CPTrace2CurveParallelism_t CPFusionTraceCalcTraceClothoidParallelism(
    uint8 iTr, float32 fC0, float32 fC1, const GDBTrafoMatrix2D_t *MOT2CIR);

#ifdef __cplusplus
};
#endif

/*! @endcond Doxygen_Suppress */

#endif
/** @} end defgroup */
