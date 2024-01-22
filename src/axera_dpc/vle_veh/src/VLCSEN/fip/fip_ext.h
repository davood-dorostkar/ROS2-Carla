
/** @defgroup fip FIP (VLC Input Preprocessing)
   @ingroup sim_swc_vlc

@{ */

#ifndef _FIP_EXT_INCLUDED
#define _FIP_EXT_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "fip_ver.h"
#include "fip_cfg.h"
#include "fip_par.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  SYMBOLIC CONSTANTS (COMPONENT EXTERN)
*****************************************************************************/

/*****************************************************************************
  MACROS (COMPONENT EXTERN)
*****************************************************************************/
/* New  traces calculated for the VLC */
/* Number of traces that are processed in VLC */
#define FIP_STATIC_TRACE_NO_OF_TRACES (10u)
/* Maximum number of sample trace points in each trace */
#define FIP_STATIC_TRACE_NO_OF_POINTS (25)

/*!  */
#define FIP_u_TRACE_INVALID_ID (255u)
/*! for check, whether trace is occupied but without obj */
#define FIP_u_TRACE_VALID_NO_OBJ_ID (249u)  // FIP_MOT_VALID_NO_OBJ_ID
/*! for check, whether trace is occupied     with    obj */
#define FIP_u_TRACE_VALID_OBJ_ID (250u)  // EOMOT_VALID_OBJ_ID

/*****************************************************************************
  TYPEDEFS (COMPONENT EXTERN)
*****************************************************************************/

/*! Operating modes of sub-component */
typedef enum {
    FIP_INIT, /*!< Initialize all */
    FIP_OK    /*!< Normal processing */
} FIPState_t;

/* All these structures are VLC local but equivalent of definitions found in
   RTE/EM.
   Irrespective of whether we are using traces computed in EM or VLC, we would
   need these structures */
/* VLC Moving object traces*/
/* Legacy moving object traces structure*/
typedef struct {
    float32 YPredictedIntersec;
    float32 YPredictedIntersecVar;
    boolean TraceReachEgoVeh;
    boolean ValidForLaneMatrix;
} FIP_LegacyMOT_t;

typedef float32
    f_FIPStaticTracePoint_array_t
        [FIP_STATIC_TRACE_NO_OF_POINTS]; /* Array for trace points - X, Y,
                                            Standard deviation etc. */

typedef struct {
    sint8 iNumberOfPoints; /*! number of trace points in a traces.. max is 25 */
    uint8 uiReferenceToVLCObject; /*! Getting VLC Object ID */
    uint8 uiReferenceToEMObject;  /*! Getting EM Object ID */
    float32 fYIntersection; /*! Y intersection of the trace w.r.t X-axis */
    float32 fYIntersectionGradFilt; /*! Y intersection of the trace */
    f_FIPStaticTracePoint_array_t
        fTracePointX; /*! X coordinate point in Trace */
    f_FIPStaticTracePoint_array_t
        fTracePointY; /*! Y coordinate point in Trace */
    f_FIPStaticTracePoint_array_t fTracePointYStdDev; /*! Standard deviation of
                                                    trace points (note: Y not
                                                    completely correct, since
                                                    through ego vehicle
                                                    rotation,
                                                    this becomes a position
                                                    uncertainty. Only when
                                                    driving straight is it the Y
                                                    uncertainty) */
    FIP_LegacyMOT_t Legacy; /*! Legacy structure of the trace */
} FIP_t_ObjectStaticTrace;

typedef struct {
    FIP_t_ObjectStaticTrace VLCObjectStaticTrace
        [FIP_STATIC_TRACE_NO_OF_TRACES]; /*!Object static trace array */
} FIP_t_MovingObjectStaticTraces;

/*Road Works Information - fip_road_type.c*/
typedef enum FIPRoadWorks {
    FIP_NOINFO_ROADWORKS,
    FIP_ROADWORKS_DETECTED
} FIP_t_RoadWorks;

/*Types of Road - fip_road_type.c*/
typedef enum FIPRoadType {
    FIP_ROAD_TYPE_UNKNOWN,
    FIP_ROAD_TYPE_CITY,
    FIP_ROAD_TYPE_COUNTRY,
    FIP_ROAD_TYPE_HIGHWAY
} FIP_t_FusedRoadType;

/* Lane Width Class - fip_lane_matrix_plg.c */
typedef enum FIPLaneWidthClass {
    FIP_LANE_WIDTH_CLASS_UNKNOWN = -1,
    FIP_LANE_WIDTH_CLASS_NORMAL = 0,
    FIP_LANE_WIDTH_CLASS_NARROW = 1,
    FIP_LANE_WIDTH_CLASS_MORE_NARROW = 2
} FIP_t_LaneWidthClass;

/* Lane Width Source - fip_lane_matrix_plg.c */
typedef enum FIPLaneWidthSource {
    FIP_SOURCE_0,
    FIP_SOURCE_1
} FIP_t_LaneWidthSource;

typedef struct FIPRoadEstimationData {
    float32 fC0;       /*!< Road Curvature*/
    float32 fC1;       /*!< Road Curvature Gradient*/
    float32 fYawAngle; /*!< Road YawAngle*/
    float32
        fRangeMaxRight;    /*!< Estimated maximum range of road on right side */
    float32 fRangeMaxLeft; /*!< Estimated maximum range of road on left side */
    float32 fYLeft;        /*!< Estimated Distance to left side road boundary */
    float32 fYRight; /*!< Estimated Distance to right side road boundary */
    uint8 uiTrackingStatus;      /*!< Road tracking status */
    uint8 uiTrackingStatusLeft;  /*!< Road left side tracking status */
    uint8 uiTrackingStatusRight; /*!< Road right side tracking status */
    uint8 uiConfidence;          /*!< Confidence on Road estimation */
    float32 fminXLeft;  /*!< Minimal left side distance X from current cycle for
                           road estimation */
    float32 fminXRight; /*!< Minimal right side distance X from current cycle
                           for road estimation */
    float32
        fmaxXLeftCompensated; /*!< Estimated Confident maximum range of road on
                                 left side */
    float32 fmaxXRightCompensated; /*!< Estimated Confident maximum range of
                                      road on right side */
    float32 fLatStdDevFiltered; /*!< Road filtered lateral standard deviation */
    float32 fConfidenceLeft; /*!< Confidence of left side estimation of road */
    float32
        fConfidenceRight; /*!< Confidence of right side estimation of road */
} FIP_t_RoadEstimation;

typedef struct FIPRoadFusedBorderData {
    float32
        fDistRight; /*!< Estimated Distance to right side fused road boundary */
    float32 fDistRightStd; /*!< Standard Deviation of Estimated Distance to
                              right side fused road boundary */
    float32
        fDistLeft; /*!< Estimated Distance to left side fused road boundary */
    float32 fDistLeftStd; /*!< Standard Deviation of Estimated Distance to left
                             side fused road boundary */
    uint8 bStatusRight;   /*!< Confidence of Right Border estimation */
    uint8 bStatusLeft;    /*!< Confidence of Left Border estimation */
} FIP_t_RoadFusedBorder;

/*****************************************************************************
  MACROS (COMPONENT EXTERN) - Contd.
*****************************************************************************/
/*****************************************************************************

  Object trace----- OLD OBJECT TRACES

  In order to access any information from the object traces your component
  has to define the access macro(s):
    FIP_GET_MOV_OBJ_STATIC_TRACE_DATA_PTR  - It shall return the pointer to the
moving
                                      object traces buffer to use
                                      type:VLCObjectTrace_t
*****************************************************************************/
/* Irrespective of where the traces are computed, they are stored in VLC local
 * structures and referenced from there */
#define FIP_GET_MOV_OBJ_STATIC_TRACE_DATA_PTR FIP_p_MovingObjectStaticTraces
#define FIP_MOVING_OBJ_STATIC_TRACES_TYPE FIP_t_ObjectStaticTrace
#define FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTrace) \
    (&FIP_GET_MOV_OBJ_STATIC_TRACE_DATA_PTR->VLCObjectStaticTrace[iTrace])
/*! To get the X coordinates of the trace points*/
#define FIP_STATIC_TRACE_GET_X(iTrace) \
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTrace)->fTracePointX
/*! To get the Y coordinates of the trace points*/
#define FIP_STATIC_TRACE_GET_Y(iTrace) \
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTrace)->fTracePointY
/*!Gradient of the Y intersection*/
#define FIP_STATIC_TRACE_GET_Y_INTERSEC_GRAD_FILT(iTrace) \
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTrace)->fYIntersectionGradFilt
/*prediction of intersection of Y axis of the trace to the X-axis*/
#define FIP_STATIC_TRACE_GET_YAXIS_PRED_INTERSECTION(iTrace) \
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTrace)->Legacy.YPredictedIntersec
/*variance of prediction of intersection of Y axis of the trace to the X-axis*/
#define FIP_STATIC_TRACE_GET_YAXIS_PRED_INTERSECTION_VAR(iTrace) \
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTrace)->Legacy.YPredictedIntersecVar

/*! To get the VLC objects for the trace computation in VLC.. Max is
 * Envm_N_OBJECTS */
#define FIP_STATIC_TRACE_GET_VLC_ID(iTrace) \
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTrace)->uiReferenceToVLCObject
/*! To get the VLC objects for the trace computation in VLC.. Max is 100 */
#define FIP_STATIC_TRACE_GET_EM_ID(iTrace) \
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTrace)->uiReferenceToEMObject
/*! To get standard deviation of the Y- intersection */
#define FIP_STATIC_TRACE_GET_Y_STD_DEV(iTr) \
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTr)->fTracePointYStdDev
#define FIP_STATIC_TRACE_GET_Y_INTERSEC(iTr) \
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTr)->fYIntersection
/*! To get the number of trace points in trace max..25 */
#define FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr) \
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTr)->iNumberOfPoints
/*To check the trace tail is reached ego vehicle head*/
#define FIP_STATIC_TRACE_REACHED_EGO_VEH(iTr) \
    FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iTr)->Legacy.TraceReachEgoVeh
/*! To get the trace id for the computed traces */
#define OBJ_GET_STATIC_TRACE_ID(iObj) GET_VLC_OBJ(iObj).u_ReferenceToTrace

/*! Curvature type for each segment of the NAVI path */
typedef enum FIPNaviPathCurvatureType {
    NAVI_PATH_CURVY,     /*!< Segment is curvy (fABS(curvature) > 0) */
    NAVI_PATH_STRAIGHT,  /*!< Segment is straight (curvature = 0) */
    NAVI_PATH_POLYNOMIAL /*< Segment is clothoid -> use polynomial approximation
                            of clothoid */
} FIPNaviPathCurvatureType_t;

/* Macros to access FIP ROAD Information */
#define FIP_GET_ROAD_ESTIMATION_DATA_PTR                                       \
    FIP_p_RoadEstimation /*!< Pointer to access FIP_t_RoadEstimation Structure \
                            members */
#define FIP_GET_ROAD_BORDER_DATA_PTR                                  \
    FIP_p_RoadFusedBorder /*!< Pointer to acces FIP_t_RoadFusedBorder \
                             Structure members */

/*!< Pointer to access FIP_t_RoadEstimation Structure members */
#define FIP_ROAD_GET_CR_CURVATURE \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR->fC0 /*! Get Road Curvature */
#define FIP_ROAD_GET_CR_CURVATURE_GRADIENT \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR->fC1 /*! Get Road Curvature Gradient */
#define FIP_ROAD_GET_CR_YAWANGLE \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR->fYawAngle /*! Get Road YawAngle */
#define FIP_ROAD_GET_CR_MAX_X_RIGHT  \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR \
        ->fRangeMaxRight /*! Get max estimated right side distance to road */
#define FIP_ROAD_GET_CR_MAX_X_LEFT   \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR \
        ->fRangeMaxLeft /*! Get max estimated left side distance to road */
#define FIP_ROAD_GET_CR_OFFSET_LEFT  \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR \
        ->fYLeft /*! Get estimated Distance to left side road boundary */
#define FIP_ROAD_GET_CR_OFFSET_RIGHT \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR \
        ->fYRight /*! Get estimated Distance to right side road boundary */
#define FIP_ROAD_GET_CR_TRACKSTAT    \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR \
        ->uiTrackingStatus /*! Get trakcing status of road */
#define FIP_ROAD_GET_CR_TRACKSTAT_LEFT \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR   \
        ->uiTrackingStatusLeft /*! Get trakcing status of left side of road */
#define FIP_ROAD_GET_CR_TRACKSTAT_RIGHT                                       \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR                                          \
        ->uiTrackingStatusRight /*! Get trakcing status of right side of road \
                                   */
#define FIP_ROAD_GET_CR_CONFIDENCE   \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR \
        ->uiConfidence /*! Get confidence of road estimation */
#define FIP_ROAD_GET_CR_MIN_X_LEFT   \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR \
        ->fminXLeft /*! Get minimum left side distance x to road boundary */
#define FIP_ROAD_GET_CR_MIN_X_RIGHT  \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR \
        ->fminXRight /*! Get minimum right side distance x to road boundary */
#define FIP_ROAD_GET_CR_MAX_X_LEFT_COMPENSTATED                               \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR                                          \
        ->fmaxXLeftCompensated /*! Get confident maximum left side distance x \
                                  to road boundary */
#define FIP_ROAD_GET_CR_MAX_X_RIGHT_COMPENSTATED                              \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR                                          \
        ->fmaxXRightCompensated /*! Get confident maximum right side distance \
                                   x to road boundary */
#define FIP_ROAD_GET_CR_EST_STDEV                                        \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR                                     \
        ->fLatStdDevFiltered /*! Get lateral standard deviation for road \
                                estimation */
#define FIP_ROAD_GET_CR_CONFIDENCE_LEFT                                     \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR->fConfidenceLeft /*! Get minimum left  \
                                                         side distance x to \
                                                         road boundary */
#define FIP_ROAD_GET_CR_CONFIDENCE_RIGHT                                     \
    FIP_GET_ROAD_ESTIMATION_DATA_PTR->fConfidenceRight /*! Get minimum left  \
                                                          side distance x to \
                                                          road boundary */

/*!< Pointer to access FIP_t_RoadFusedBorder Structure members */
#define FIP_ROAD_GET_BORDER_RIGHT                                        \
    FIP_GET_ROAD_BORDER_DATA_PTR                                         \
        ->fDistRight /*! Get estimated Distance to right side fused road \
                        boundary */
#define FIP_ROAD_GET_BORDER_RIGHT_STD                                       \
    FIP_GET_ROAD_BORDER_DATA_PTR                                            \
        ->fDistRightStd /*! Get standard deviation of estimated Distance to \
                           right side fused road boundary */
#define FIP_ROAD_GET_BORDER_LEFT                                       \
    FIP_GET_ROAD_BORDER_DATA_PTR                                       \
        ->fDistLeft /*! Get estimated Distance to left side fused road \
                       boundary */
#define FIP_ROAD_GET_BORDER_LEFT_STD                                         \
    FIP_GET_ROAD_BORDER_DATA_PTR->fDistLeftStd /*! Get standard deviation of \
                                                  estimated Distance to left \
                                                  side fused road boundary */
#define FIP_ROAD_GET_BORDER_STAT_RIGHT                               \
    FIP_GET_ROAD_BORDER_DATA_PTR                                     \
        ->bStatusRight /*! Get confidence of right fused road border \
                          estimation */
#define FIP_ROAD_GET_BORDER_STAT_LEFT                                         \
    FIP_GET_ROAD_BORDER_DATA_PTR                                              \
        ->bStatusLeft /*! Get confidence of left fused road border estimation \
                         */

/*****************************************************************************
  CONSTANTS (COMPONENT EXTERN)
*****************************************************************************/

#define FIP_CAM_LANE_POE_LEVEL                                               \
    (                                                                        \
        70u) /*! Threshold for the existence probability of the camera lane; \
                only above this value, the camera lane is cosidered as valid \
                */
#define FIP_NAVI_MAX_APPROX_EGO_DIFF                                         \
    (3.f) /*!< Maximum difference between the approximated NAVI path and EGO \
             curve @unit:m */
#define FIP_NAVI_MAX_YCOMP_AT_ORIG                                         \
    (5.f) /*!< Threshold for maximum difference between EGO path estimated \
             NAVI path at X = 0 */

/*****************************************************************************
  VARIABLES (COMPONENT EXTERN)
*****************************************************************************/

extern FIPState_t FIPState;

/* Moving object trace structure*/
extern FIP_t_MovingObjectStaticTraces *FIP_p_MovingObjectStaticTraces;

/*! Pointer to access FIP_t_RoadEstimation Structure members */
extern const FIP_t_RoadEstimation *FIP_p_RoadEstimation;
/*! Pointer to access FIP_t_RoadFusedBorder Structure members */
extern const FIP_t_RoadFusedBorder *FIP_p_RoadFusedBorder;

/*****************************************************************************
  FUNCTIONS (COMPONENT EXTERN)
*****************************************************************************/

/*! fip_main.c */
extern void FIPProcess(void);

/*! fip_navi_data_plg.c and fip_navi_data_process.c (used in CP and SI) */

/*! fip_camera_lane_plg.c */
extern float32 FIP_f_GetCurveCamLane(void);
extern float32 FIP_f_GetCurvatureChangeCamLane(void);
extern float32 FIP_f_GetVisibilityDistCamLane(void);
extern float32 FIP_f_GetHeadingAngleCamLane(void);
extern float32 FIP_f_GetWidthCamLane(void);
extern boolean FIP_b_GetIsCamLaneRoadEdge(
    const t_CamLaneMarkerEnum CamLaneMarkerEnum);
extern boolean FIP_b_GetCamRoadworks(void);
extern boolean FIP_b_GetCamLaneObjAssocValidity(void);

/*! fip_lane_matrix_plg.c */
extern sint8 FIP_s_GetLMLeftNumLane(void);
extern sint8 FIP_s_GetLMRightNumLane(void);
extern FIP_t_LaneWidthClass FIP_t_Get_LaneWidthClass(void);
extern FIP_t_LaneWidthSource FIP_t_Get_LaneWidthSource(void);
extern float32 FIP_f_Get_LaneWidth(void);

/*! fip_traffic_orientation.c */
extern eTrafficOrientation_t FIP_t_GetTrafficOrientation(void);

/*! fip_road_type.c */
extern FIP_t_FusedRoadType FIP_t_Get_FuseRoadType(
    void); /*! Function to get FIP fused road type */
extern FIP_t_RoadWorks FIP_t_Get_RoadWorks(
    void); /*! Function to get FIP Road Works value */

/*fip_object_traces.c*/
/* to get Y axis intersection for the trace */
extern boolean FIP_b_GetStaticTraceYAxisIntersection(TraceID_t iTrace,
                                                     float32 *pfY,
                                                     float32 *pfYVar);

#ifdef __cplusplus
};
#endif
/* End of conditional inclusion */
#else
#endif /*!< _FIP_EXT_INCLUDED */

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/** @} end defgroup */
