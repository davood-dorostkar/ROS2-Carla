
/*!
   @defgroup si SI (Fct Situation Interpretation)
   @ingroup vlc_sen


@{ */

#ifndef _SI_EXT_H_INCLUDED
#define _SI_EXT_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "cp_ext.h"
#include "si_cfg.h"

#include "si_ver.h"

#if (defined(_MSC_VER))
#pragma COMPILEMSG( \
    "Remove use of corridor types in SI struct to allow removal of si_corridor_crit.h from si.he!")
#endif
#include "si_corridor_crit.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  SYMBOLISCHE KONSTANTEN (KOMPONENTENINTERN)
*****************************************************************************/

/*****************************************************************************
  SYMBOLISCHE KONSTANTEN (KOMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  MACROS (KOMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS (KOMPONENTENEXTERN)
*****************************************************************************/

/*! operating modes of sub-component */
typedef enum SIStateTag {
    SI_INIT,    /*!< initialize all */
    SI_OK,      /*!< normal processing */
    SI_RED_QUAL /*!< reduced quality check mode: don't check
                   NearRangeDisturbance, ObstacleProb., GridConfirmation and RCS
                   (for EOL Test) */
} SIState_t;

/*! 6 object interface object enumeration, aliasing RTE type */
typedef eObjOOI_t SIRelObjEnum_t;
/*! SILaneState_t */
typedef enum SILaneStateTag {
    OBJ_STATE_INLANE = 0, /*!< Object is inlane */
    OBJ_STATE_OUTLANE = 1 /*!< Object is not in ego lane */
} SILaneState_t;

typedef struct SIObjLaneStateTag {
    SILaneState_t
        SIInlaneState; /*!< Lane assignment (confirmed with all criteria) */
    SILaneState_t SIActLaneState; /*!< Lane assignment without timer and
                                     distance criteria (unconfirmed) */
    uint8 In2OutlaneTransition;
    fTime_t fCorridorRelevantTime;
    fDistance_t fCorridorRelevantDist;

} SIObjLaneState_t;

/*! bit by bit history type (used for whole lifetime valid informations )*/
typedef struct SIBoolTag {
    ubit32_t Moving : 1;          /*!< Object was Moving at some time */
    ubit32_t Oncoming : 1;        /*!< Object was Oncoming at some time */
    ubit32_t Stationary : 1;      /*!< Object was Stationary at some time */
    ubit32_t OccludedByTrace : 1; /*!< Object is marked as occluded by trace
                                     analysis */
    ubit32_t SelectedAsOOI : 1;   /*!< Object is candidate for the OOI list */
    ubit32_t AlreadyOOI : 1;      /*!< Object is already in OOI list */
    ubit32_t
        SelectedByPathDecision : 1; /*!< Object was selected by blocked path */
    ubit32_t
        FctPreselTG : 1; /*!< Object is within preselection timegap distance */

    ubit32_t InLOccValue : 1; /*!< Inlane occupancy flag (either object / lane
                                 occupancy) */
    ubit32_t InLCustomValue : 1;  /*!< Inlane custom flag (predicted inlane) */
    ubit32_t InLQualityValue : 1; /*!< Inlane quality value (unused) */
    ubit32_t InLObjOccValue : 1;  /*!< Inlane object occupancy flag */
    ubit32_t InLLaneOccValue : 1; /*!< Inlane lane occupancy flag */
    ubit32_t InLTimeValue : 1;    /*!< Inlane timer satisfied */
    ubit32_t
        OutLOccValue : 1; /*!< Outlane occupancy flag (either object / lane
                             occupancy) */
    ubit32_t
        OutLCustomValue : 1; /*!< Outlane custom flag (predicted outlane) */

    ubit32_t OutLObjOccValue : 1;  /*!< Outlane object occupancy flag */
    ubit32_t OutLLaneOccValue : 1; /*!< Outlane lane occupancy flag */
    ubit32_t
        StatObjWasOncoming : 1; /*!< bool if stationary object was treated as
                                   oncoming */
    ubit32_t : 0;
} SIBool_t;

typedef struct SIBlockedPathDecisionTag {
    uint8 PathSelectionTimer;
} SIBlockedPathDecision_t;

/*! Structure to store information for special handling of stationary objects
 * which were only seen stationary or oncoming */
typedef struct SIStatObjWasOncoming {
    uint8 uiOncomingCounter; /*!< No. of cycles which an object was seen
                                oncoming */
} SIStatObjWasOncoming_t;

typedef struct SITag {
    SIBool_t Bool;
    fDistance_t fPredictedLatDispl;    /*!< Predicted lateral displacement
                                          @min:-100 @max:100 @unit:m */
    SIObjLaneState_t ObjLaneAccStatus; /*!< The object lane assignment state */
    SIBlockedPathDecision_t BlockedPathDecision;
    SIStatObjWasOncoming_t StatObjWasOncoming;
    SIObjCorridor_t ObjCor;
} SI_t;

typedef struct SIRelObjectTag {
    boolean ObjValid;     /*!< Boolean set to TRUE if object is valid */
    boolean StatObj;      /*!< Boolean set to TRUE if object is stationary */
    boolean MovingToStat; /*!< Boolean set to TRUE if moving object turned
                             stationary */

    fDistance_t DistX; /*!< Distance of object in X direction (longitudinal) */
    fVelocity_t RelSpeedX; /*!< Relative speed of object in X direction */
    fAccel_t RelAcclX; /*!< Relative acceleration of object in X direction */

    fVelocity_t ObjSpeed; /*!< Absolute object speed */
    fAccel_t ObjAccl;     /*!< Absolute object acceleration */

    fAngle_t ObjAngle;     /*!< Object angle */
    fDistance_t ObjDistY;  /*!< Distance of object in Y direction (lateral) */
    fVelocity_t RelSpeedY; /*!< Relative speed of object in Y direction */

    fDistance_t LatDisplRoadBordL; /*!< Lateral displacement of lane border
                                      (bracket) left */
    fDistance_t LatDisplRoadBordR; /*!< Lateral displacement of lane border
                                      (bracket) right */

    fDistance_t
        RelPickupDist; /*!< Pickup (first selection X distance), formerly
                          dAbst_RelevantNeu on each object, now only here */

    /* Additional information for deducing object loss reason */

    ObjNumber_t ObjectNr; /*!< Object index (or OBJ_INDEX_NO_OBJECT if none) */

    eRelObjLossReason_t LossReason; /*!< Relevant object loss reason */

} SIRelObject_t;

/*! Enum to describe the camera lane marker crossing state */
typedef enum SILCStateCamLaneMarkerCrossed_t {
    UNKNOWN_CROSS_CAMLANE, /*!< Unknown crossing of the camera lane marker */
    LEFT_ONE_LANE_CONF_CROSS_CAMLANE,  /*!< Crossing of the left camera lane
                                          marker based on the information of the
                                          left camera lane marker */
    RIGHT_ONE_LANE_CONF_CROSS_CAMLANE, /*!< Crossing of the right camera lane
                                          marker based on the information of the
                                          right camera lane marker */
    LEFT_TWO_LANE_CONF_CROSS_CAMLANE,  /*!< Crossing of the left camera lane
                                          marker based on the information of the
                                          left and right camera lane marker */
    RIGHT_TWO_LANE_CONF_CROSS_CAMLANE  /*!< Crossing of the right camera lane
                                          marker based on the information of the
                                          left and right camera lane marker */
} t_SILCStateCamLaneMarkerCrossed;

/*! lane change states */
typedef enum SI_LC_LaneChangeState {
    LC_RIGHT = -1, /*!< Lane change right */
    LC_FOLLOW = 0, /*!< No lane change, i.e. follow mode */
    LC_LEFT = 1    /*!< Lane change left */
} SI_LC_t_LaneChangeState;

/*! Lane change Traffic orientation */
typedef enum SI_LC_TrafficOrientation {
    LC_TRAFFIC_ORIENT_RIGHT = 1,
    LC_TRAFFIC_ORIENT_LEFT = 2,
    LC_TRAFFIC_ORIENT_UNKNOWN = 0
} SI_LC_t_TrafficOrientation;

/*!< Information for each lane change phase */
typedef struct SI_LC_LaneChangePhase {
    float32 f_LCPhaseProb;                  /*!< Lane change probability */
    SI_LC_t_LaneChangeState t_LCPhaseState; /*!< Lane change state */
    SI_LC_t_TrafficOrientation
        t_LCTrafficOrientation; /*!< Traffic orientation */
} SI_LC_t_LaneChangePhaseInfo;

/*! Enum to describe if the ego vehicle moves to the new lane in the first stage
   of the lane change,
    i.e. before entering the new lane */
typedef enum SILaneChaneCamPreMove {
    LANE_CHANGE_CAM_PRE_MOVE_NO = 0,
    LANE_CHANGE_CAM_PRE_MOVE_LEFT = 1,
    LANE_CHANGE_CAM_PRE_MOVE_RIGHT = -1
} t_SILaneChangeCamPreMove;

/*****************************************************************************
  KONSTANTEN (KOMPONENTENEXTERN)
*****************************************************************************/
/*! defines the number of objects of interest*/
#define SiAnzOOI 6

/*****************************************************************************
  VARIABLEN (KOMPONENTENEXTERN)
*****************************************************************************/

/*! sub-module state */
extern MEMSEC_REF SIState_t SIState;

/*****************************************************************************
  FUNKTIONEN (KOMPONENTENEXTERN)
*****************************************************************************/
/*-- si_main.c --*/
extern void SIInit(void);
extern void VLCSIProcess(void);
extern void SIMergeObjects(ObjNumber_t iObjectToKeep,
                           ObjNumber_t iObjectToDelete);
extern CPCourseData_t *SIGetCourseData(void);
extern CPTrajectoryData_t *SIGetTrajectoryData(void);
extern void SIProcessTrajectoriesMeas(CPTrajMeasInfo_t *pTrajectoriesMeas);
extern void SITrajGetObjToRefDistance(ObjNumber_t ObjId,
                                      float32 *fDist,
                                      float32 *fDistVar);
extern void SITrajGetObjToRefDistanceGradient(ObjNumber_t ObjId,
                                              float32 *fDistGrad,
                                              float32 *fDistGradVar);
extern void SIDeleteObject(ObjNumber_t ObjId);
extern void SIMergeDeleteObjectSameVLCID(ObjNumber_t ObjNr);

/*-- si_objloss.c --*/
extern void SIObReObGetRelObjDisappearedFlag(boolean *bRelObjDisappearedFlag);
extern eRelObjLossReason_t SIObOOIGetOOILossReason(SIRelObjEnum_t SiOOINr);

/*---si_objattributes----*/
extern float32 SIGetObjectLength(const ObjNumber_t uiObject);

/*-- SISeReOb.c --*/
extern ObjNumber_t SISeReObGetRelTrckObjNumber(void);

/*-- si_output.c --*/
extern const SIRelObject_t *SIReSiDaGetRelevantObject(void);

/*---- si_ouput.c -----*/
extern void SIFreezeData(void);

/*-- si_lanechange.c --*/
extern SI_LC_t_LaneChangePhaseInfo SIGetLaneChangeTimeGap(void);

extern sint16 SILCGetLaneChangeProbability(void);
extern t_SILCStateCamLaneMarkerCrossed SI_t_GetCamLaneMarkerCrossed(void);

extern t_SILaneChangeCamPreMove SI_t_GetLaneChangeMovePre(void);

extern fDistance_t SIGetMovingObjPickupRange(void);
extern fDistance_t SIGetMovingObjBasePickupRange(void);

/*--- si_laneassociation.c ---*/

#ifdef __cplusplus
};
#endif

/* End of conditional inclusion */
#else
#endif
/** @} end defgroup */
