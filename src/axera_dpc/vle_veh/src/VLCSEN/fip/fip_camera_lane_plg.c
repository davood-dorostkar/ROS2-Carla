/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "stddef.h"
#include "TM_Global_Types.h"

#include "TM_Global_Const.h"
#include "vlcSen_consts.h"
#include "vlcSen_common_utils.h"
#include "vlcSen_consts.h"
#include "fip_ext.h"

/*****************************************************************************
  GLOBAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  LOCAL SYMOBLIC CONSTANTS
*****************************************************************************/

#define FIP_CAM_MIN_VISIB_DIST \
    (7.f) /*!< Minimal visibility distance for a valid camera lane */
#define FIP_CAM_MAX_DIFF_HEADING_ANGLE                                    \
    (DEG2RAD(1.f)) /*!< Threshold for the difference of the heading angle \
                        between the left and right lane marker.           \
                        Below this threshold the lane markers are trusted */
#define FIP_CAM_MAX_LANE_WIDTH_ROADWORKS                                \
    (2.5f) /*!< Maximum camera lane width for assuming a roadworks on a \
              highway */
#define FIP_CAM_MIN_LANE_WIDTH_RANGE \
    (2.0f) /*!< Minimum valid camera lane width */
#define FIP_CAM_MAX_LANE_WIDTH_RANGE \
    (5.0f) /*!< Maximum valid camera lane width */

/*****************************************************************************
  LOCAL MACROS
*****************************************************************************/

/*****************************************************************************
  LOCAL TYPES
*****************************************************************************/

/*****************************************************************************
  LOCAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  LOCAL TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  LOCAL VARIABLES
*****************************************************************************/

/*! Evaluated camera lane data which is used in other files */
SET_MEMSEC_VAR(t_FIPCamDataEvaluated)
static struct t_FIPCamDataEvaluated {
    boolean b_Roadworks; /*!< Info if camera lane indicates a roadworks (on a
                            highway) */
    boolean
        b_CamLaneAssocValid;   /*!< Info if camera lane association is valid */
    float32 f_Curve;           /*!< Curvature of the camera lane */
    float32 f_CurvatureChange; /*!< Curvature change of the camera lane*/
    float32 f_VisibilityDist;  /*!< Visibility distance of the camera lane */
    float32 f_HeadingAngle;    /*!< Heading angle relative to the camera lane */
    float32 f_LaneWidth;       /*!< Width of the ego lane based on the camera
                                  information */
    boolean ab_CamLaneIsRoadEdge[VLC_CAM_LANE_NUM_LANES]; /*!< Info if the
                                                             camera lane is a
                                                             road edge */
} FIPCamDataEvaluated;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

static void FIP_v_CalcCurveCamLane(const boolean b_MarkerToUse,
                                   const t_CamLaneMarkerEnum MarkerToUse);
static void FIP_v_CalcCurvatureChangeCamLane(
    const boolean b_MarkerToUse, const t_CamLaneMarkerEnum MarkerToUse);
static void FIP_v_CalcVisibilityDistCamLane(
    const boolean b_MarkerToUse, const t_CamLaneMarkerEnum MarkerToUse);
static void FIP_v_CalcHeadingAngleCamLane(
    const boolean b_MarkerToUse, const t_CamLaneMarkerEnum MarkerToUse);
static void FIP_v_CalcWidthCamLane(const boolean b_MarkerToUse);
static void FIP_v_SetCamLaneMarkerToUseFusion(
    boolean* pb_MarkerToUse, t_CamLaneMarkerEnum* pt_MarkerToUse);
static void FIP_v_SetCamLaneIsRoadEdge(const boolean b_MarkerToUse);
static void FIP_v_SetCamLaneObjAssociationValidity(const boolean b_MarkerToUse);
static void FIP_v_SetRoadworksCam(const boolean b_MarkerToUse);

/*************************************************************************************************************************
  Functionname:    FIP_v_CamLaneDataProcess */
void FIP_v_CamLaneDataProcess(void) {
    boolean b_MarkerToUse; /*!< Boolean if the info in t_MarkerToUse should be
                              considered */
    t_CamLaneMarkerEnum MarkerToUse; /*!< Lane marker to be used for determing
                                        the curvature, visibility distance or
                                        heading angle */

    /*! Determine which lane marker should be considered for determing the
     * curvature, visibility distance or heading angle */
    FIP_v_SetCamLaneMarkerToUseFusion(&b_MarkerToUse, &MarkerToUse);
    /*! Determine the heading angle of the camera lane */
    FIP_v_CalcHeadingAngleCamLane(b_MarkerToUse, MarkerToUse);
    /*! Determine the curvature of the camera lane */
    FIP_v_CalcCurveCamLane(b_MarkerToUse, MarkerToUse);
    /*! Determine the curvature change of the camera lane */
    FIP_v_CalcCurvatureChangeCamLane(b_MarkerToUse, MarkerToUse);
    /*! Determine the visibility range of the camera lane */
    FIP_v_CalcVisibilityDistCamLane(b_MarkerToUse, MarkerToUse);
    /*! Determine the width of the camera lane */
    FIP_v_CalcWidthCamLane(b_MarkerToUse);
    /*! Determine if the camera lane is a road edge */
    FIP_v_SetCamLaneIsRoadEdge(b_MarkerToUse);
    /*! Determine if the camera lane object association is valid */
    FIP_v_SetCamLaneObjAssociationValidity(b_MarkerToUse);
    /*! Determine if the camera lane indicates a roadworks */
    FIP_v_SetRoadworksCam(b_MarkerToUse);
}

/*************************************************************************************************************************
  Functionname:    FIP_v_SetCamLaneMarkerToUseFusion */
static void FIP_v_SetCamLaneMarkerToUseFusion(
    boolean* pb_MarkerToUse, t_CamLaneMarkerEnum* pt_MarkerToUse) {
    /*! Set local variable for visibility distance */
    const float32 f_VisibDistLeft =
        VLCSEN_pCamLaneData->CourseInfo[CL_CAM_LANE_MK_LEFT]
            .CourseInfoSegNear.f_Length;
    const float32 f_VisibDistRight =
        VLCSEN_pCamLaneData->CourseInfo[CL_CAM_LANE_MK_RIGHT]
            .CourseInfoSegNear.f_Length;

    /*! Set boolean if Marker to be used at all */
    (*pb_MarkerToUse) = TRUE;

    /*! If both markers are valid based on existence probability and visibility
     * range */
    if ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
        (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
        (f_VisibDistLeft > FIP_CAM_MIN_VISIB_DIST) &&
        (f_VisibDistRight > FIP_CAM_MIN_VISIB_DIST) &&
        (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK)) {
        /*! Only trust the camera lane if the heading angle is similar for both
         * sides */
        if (fABS(
                VLCSEN_pCamLaneData->CourseInfo[CL_CAM_LANE_MK_LEFT].f_Angle -
                VLCSEN_pCamLaneData->CourseInfo[CL_CAM_LANE_MK_RIGHT].f_Angle) <
            FIP_CAM_MAX_DIFF_HEADING_ANGLE) {
            if (f_VisibDistLeft > f_VisibDistRight) {
                /*! If the left visibility range is higher, use the left side */
                (*pt_MarkerToUse) = CL_CAM_LANE_MK_LEFT;
            } else {
                /*! If the right visibility range is higher, use the right side
                 */
                (*pt_MarkerToUse) = CL_CAM_LANE_MK_RIGHT;
            }
        } else {
            /*! If the heading angle is different for both sides, set default */
            (*pt_MarkerToUse) = CL_CAM_LANE_MK_LEFT; /*!< Any valid value is
                                                        set. The info in
                                                        pt_MarkerToUse is only
                                                        considered in the
                                                        following, if
                                                        pb_MarkerToUse == TRUE.
                                                        */
            (*pb_MarkerToUse) = FALSE;
        }
    } else if ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                    .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
               (f_VisibDistLeft > FIP_CAM_MIN_VISIB_DIST) &&
               (VLCSEN_pCamLaneData->sSigHeader.eSigStatus ==
                AL_SIG_STATE_OK)) {
        /*! If only the left marker is valid, use the left side */
        (*pt_MarkerToUse) = CL_CAM_LANE_MK_LEFT;
    } else if ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                    .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
               (f_VisibDistRight > FIP_CAM_MIN_VISIB_DIST) &&
               (VLCSEN_pCamLaneData->sSigHeader.eSigStatus ==
                AL_SIG_STATE_OK)) {
        /*! If only the right marker is valid, use the right side */
        (*pt_MarkerToUse) = CL_CAM_LANE_MK_RIGHT;
    } else {
        /*! If no valid lane marker, set default */
        (*pt_MarkerToUse) =
            CL_CAM_LANE_MK_LEFT; /*!< Any valid value is set. The info in
                                    pt_MarkerToUse is only considered in the
                                    following, if pb_MarkerToUse == TRUE. */
        (*pb_MarkerToUse) = FALSE;
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_CalcCurveCamLane */
static void FIP_v_CalcCurveCamLane(const boolean b_MarkerToUse,
                                   const t_CamLaneMarkerEnum MarkerToUse) {
    /*! If the info in MarkerToUse is valid, use the lane marker in MarkerToUse;
     * otherwise set the default value */
    if (b_MarkerToUse == TRUE) {
        FIPCamDataEvaluated.f_Curve =
            VLCSEN_pCamLaneData->CourseInfo[MarkerToUse].CourseInfoSegNear.f_C0;
    } else {
        FIPCamDataEvaluated.f_Curve = 0.f;
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_CalcCurvatureChangeCamLane */
static void FIP_v_CalcCurvatureChangeCamLane(
    const boolean b_MarkerToUse, const t_CamLaneMarkerEnum MarkerToUse) {
    /*! If the info in MarkerToUse is valid, use the lane marker in MarkerToUse;
     * otherwise set the default value */
    if (b_MarkerToUse == TRUE) {
        FIPCamDataEvaluated.f_CurvatureChange =
            VLCSEN_pCamLaneData->CourseInfo[MarkerToUse].CourseInfoSegNear.f_C1;
    } else {
        FIPCamDataEvaluated.f_CurvatureChange = 0.f;
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_CalcVisibilityDistCamLane */
static void FIP_v_CalcVisibilityDistCamLane(
    const boolean b_MarkerToUse, const t_CamLaneMarkerEnum MarkerToUse) {
    /*! If the info in MarkerToUse is valid, use the lane marker in MarkerToUse;
     * otherwise set the default value */
    if (b_MarkerToUse == TRUE) {
        FIPCamDataEvaluated.f_VisibilityDist =
            VLCSEN_pCamLaneData->CourseInfo[MarkerToUse]
                .CourseInfoSegNear.f_Length;
    } else {
        FIPCamDataEvaluated.f_VisibilityDist = 0.f;
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_CalcWidthCamLane */
static void FIP_v_CalcWidthCamLane(const boolean b_MarkerToUse) {
    /*! Obtain lateral position of left and right lane marker */
    const float32 f_LeftMarkerDist =
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT].f_MarkerDist;
    const float32 f_RightMarkerDist =
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT].f_MarkerDist;

    /*! Check if lane markers are trustworthy */
    if ((b_MarkerToUse == TRUE) &&
        (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
        (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
        (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK) &&
        (VLCSEN_pCamLaneData->CourseInfo[CL_CAM_LANE_MK_LEFT]
             .CourseInfoSegNear.f_Length > FIP_CAM_MIN_VISIB_DIST) &&
        (VLCSEN_pCamLaneData->CourseInfo[CL_CAM_LANE_MK_RIGHT]
             .CourseInfoSegNear.f_Length > FIP_CAM_MIN_VISIB_DIST) &&
        (f_LeftMarkerDist > f_RightMarkerDist)) {
        /*! Calculate lane width */
        FIPCamDataEvaluated.f_LaneWidth = f_LeftMarkerDist - f_RightMarkerDist;
        /*! Range Check*/
        if ((FIPCamDataEvaluated.f_LaneWidth < FIP_CAM_MIN_LANE_WIDTH_RANGE) ||
            (FIPCamDataEvaluated.f_LaneWidth > FIP_CAM_MAX_LANE_WIDTH_RANGE)) {
            /*! Reset invalid cam lane width*/
            FIPCamDataEvaluated.f_LaneWidth = 0.f;
        } else {
            /* do nothing */
        }
    } else {
        /*! Not possible to provide reasonable lane width */
        FIPCamDataEvaluated.f_LaneWidth = 0.f;
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_CalcHeadingAngleCamLane */
static void FIP_v_CalcHeadingAngleCamLane(
    const boolean b_MarkerToUse, const t_CamLaneMarkerEnum MarkerToUse) {
    /*! If the info in MarkerToUse is valid, use the lane marker in MarkerToUse;
     * otherwise set the default value */
    if (b_MarkerToUse == TRUE) {
        FIPCamDataEvaluated.f_HeadingAngle =
            VLCSEN_pCamLaneData->CourseInfo[MarkerToUse].f_Angle;
    } else {
        /*! Set default if camera lane not valid */
        FIPCamDataEvaluated.f_HeadingAngle = 0.f;
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_SetCamLaneIsRoadEdge */
static void FIP_v_SetCamLaneIsRoadEdge(const boolean b_MarkerToUse) {
    uint8 i;

    for (i = 0u; i < (uint8)VLC_CAM_LANE_NUM_LANES; i++) {
        /*! Default: No road edge */
        FIPCamDataEvaluated.ab_CamLaneIsRoadEdge[i] = FALSE;

        /*! Possible lane marker types for a road edge */
        if ((b_MarkerToUse == TRUE) &&
            (VLCSEN_pCamLaneData->LaneMarkerInfo[i].u_ExistanceProbability >
             0u) &&
            ((VLCSEN_pCamLaneData->LaneMarkerInfo[i].MarkerType ==
              CL_MARKER_TYPE_CONTINUOUS) ||
             (VLCSEN_pCamLaneData->LaneMarkerInfo[i].MarkerType ==
              CL_MARKER_TYPE_LOWCURB) ||
             (VLCSEN_pCamLaneData->LaneMarkerInfo[i].MarkerType ==
              CL_MARKER_TYPE_HIGHCURB) ||
             (VLCSEN_pCamLaneData->LaneMarkerInfo[i].MarkerType ==
              CL_MARKER_TYPE_CRASHBARRIER) ||
             (VLCSEN_pCamLaneData->LaneMarkerInfo[i].MarkerType ==
              CL_MARKER_TYPE_WALL) ||
             (VLCSEN_pCamLaneData->LaneMarkerInfo[i].MarkerType ==
              CL_MARKER_TYPE_ROADSHOULDER))) {
            FIPCamDataEvaluated.ab_CamLaneIsRoadEdge[i] = TRUE;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_SetRoadworksCam */
static void FIP_v_SetRoadworksCam(const boolean b_MarkerToUse) {
    /*! Initialize detected roadworks to false */
    FIPCamDataEvaluated.b_Roadworks = FALSE;

    /*! If the camera lane width is lower than a threshold or if the
       ConstructionSite structure indicates a roadworks,
        set the detected roadworks information to true */
    if ((VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK) &&
        (b_MarkerToUse == TRUE) &&
        (((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
               .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
          (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
               .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
          ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                .f_MarkerDist -
            VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                .f_MarkerDist) < FIP_CAM_MAX_LANE_WIDTH_ROADWORKS)) ||
         (VLCSEN_pCamLaneData->ConstructionSite.b_CrossingMarker == TRUE) ||
         (VLCSEN_pCamLaneData->ConstructionSite.b_Hold == TRUE) ||
         (VLCSEN_pCamLaneData->ConstructionSite.b_InhibitSingleLane == TRUE) ||
         (VLCSEN_pCamLaneData->ConstructionSite.b_LeftBarrier == TRUE) ||
         (VLCSEN_pCamLaneData->ConstructionSite.b_RightBarrier == TRUE) ||
         (VLCSEN_pCamLaneData->ConstructionSite.b_MultipleMarker == TRUE))) {
        FIPCamDataEvaluated.b_Roadworks = TRUE;
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_SetCamLaneObjAssociationValidity */
static void FIP_v_SetCamLaneObjAssociationValidity(
    const boolean b_MarkerToUse) {
    /*! Initialize camera lane object association to false */
    FIPCamDataEvaluated.b_CamLaneAssocValid = FALSE;

    if ((VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK) &&
        (b_MarkerToUse == TRUE) &&
        (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
        (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL)) {
        FIPCamDataEvaluated.b_CamLaneAssocValid = TRUE;
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_f_GetHeadingAngleCamLane */
float32 FIP_f_GetHeadingAngleCamLane(void) {
    return FIPCamDataEvaluated.f_HeadingAngle;
}

/*************************************************************************************************************************
  Functionname:    FIP_f_GetVisibilityDistCamLane */
float32 FIP_f_GetVisibilityDistCamLane(void) {
    return FIPCamDataEvaluated.f_VisibilityDist;
}

/*************************************************************************************************************************
  Functionname:    FIP_f_GetWidthCamLane */
float32 FIP_f_GetWidthCamLane(void) { return FIPCamDataEvaluated.f_LaneWidth; }

/*************************************************************************************************************************
  Functionname:    FIP_f_GetCurveCamLane */
float32 FIP_f_GetCurveCamLane(void) { return FIPCamDataEvaluated.f_Curve; }

/*************************************************************************************************************************
  Functionname:    FIP_f_GetCurvatureChangeCamLane */
float32 FIP_f_GetCurvatureChangeCamLane(void) {
    return FIPCamDataEvaluated.f_CurvatureChange;
}

/*************************************************************************************************************************
  Functionname:    FIP_b_GetIsCamLaneRoadEdge */
boolean FIP_b_GetIsCamLaneRoadEdge(
    const t_CamLaneMarkerEnum CamLaneMarkerEnum) {
    return FIPCamDataEvaluated.ab_CamLaneIsRoadEdge[CamLaneMarkerEnum];
}

/*************************************************************************************************************************
  Functionname:    FIP_b_GetCamRoadworks */
boolean FIP_b_GetCamRoadworks(void) { return FIPCamDataEvaluated.b_Roadworks; }

/*************************************************************************************************************************
  Functionname:    FIP_b_GetCamLaneObjAssocValidity */
boolean FIP_b_GetCamLaneObjAssocValidity(void) {
    return FIPCamDataEvaluated.b_CamLaneAssocValid;
}

/*************************************************************************************************************************
  Functionname:    FIP_v_InitGlobalCamLaneData */
void FIP_v_InitGlobalCamLaneData(void) {
    uint8 i;

    /*! Set default values */
    FIPCamDataEvaluated.b_Roadworks =
        FALSE; /*!< Info if camera lane indicates a roadworks (on a highway) */
    FIPCamDataEvaluated.b_CamLaneAssocValid =
        FALSE; /*! Info if camera lane association is valid */
    FIPCamDataEvaluated.f_Curve = 0.f; /*!< Curvature of the camera lane */
    FIPCamDataEvaluated.f_CurvatureChange =
        0.f; /*!< Curvature change of the camera lane*/
    FIPCamDataEvaluated.f_VisibilityDist =
        0.f; /*!< Visibility distance of the camera lane */
    FIPCamDataEvaluated.f_HeadingAngle =
        0.f; /*!< Heading angle relative to the camera lane */
    FIPCamDataEvaluated.f_LaneWidth = 0.f; /*!< Width of the camera lane */
    for (i = 0u; i < (uint8)VLC_CAM_LANE_NUM_LANES; i++) {
        FIPCamDataEvaluated.ab_CamLaneIsRoadEdge[i] =
            FALSE; /*!< Info if the camera lane is a road edge */
    }
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */