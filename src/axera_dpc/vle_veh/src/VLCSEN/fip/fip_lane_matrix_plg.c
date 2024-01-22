/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "fip.h"
#include "stddef.h"
#include "TM_Global_Types.h"
#include "fip_cfg.h"

#include "TM_Global_Const.h"
#include "vlcSen_consts.h"
#include "vlcSen_common_utils.h"
#include "emp_ext.h"
#include "si_ext.h"
#include "fip_ext.h"
#include "frame_sen_custom_types.h"
#include "vlc_sen.h"

/*****************************************************************************
  GLOBAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  LOCAL SYMOBLIC CONSTANTS
*****************************************************************************/

/*! Virtual address for FIP lane matrix meas freeze information */
#ifndef FIP_LANE_MATRIX_MEAS_FREEZE_VADDR
#define FIP_LANE_MATRIX_MEAS_FREEZE_VADDR (0x202703F0u)
#endif

/*****************************************************************************
  LOCAL MACROS
*****************************************************************************/

/*****************************************************************************
  LOCAL TYPES
*****************************************************************************/

/*****************************************************************************
  LOCAL CONSTANTS
*****************************************************************************/

#define FIP_LM_INVALID_NUM_LANE (-1) /*!< Invalid/default number of lanes */

#define FIP_LM_DEFAULT_CAM_LANE_WIDTH (-1.f) /*!< Default camera lane width */
#define FIP_LM_MIN_ADJ_CAM_LANE_WIDTH \
    (1.5f) /*!< Minimal camera lane width of the adjacent lane */

#define FIP_LM_TIMER_INVAILD_CURRENT_NUM_LANE                                 \
    (1.f) /*!< Only above this threshold, the filtered number of lanes is set \
             to invalid */
#define FIP_LM_TIMER_INVAILD_FILT_NUM_LANE                                     \
    (1.f) /*!< Only above this threshold, the invalid filtered number of lanes \
             is set to the new current value */
#define FIP_LM_FILTER_CONSTANT \
    (40.f) /*!< Filter constant for filtering the number of lanes */
#define FIP_LM_MAX_ROAD_BORDER_DIST_FOR_CAM_FUSION \
    (6.f) /*!< Maximal road border distance for camera lane fusion */

/*****************************************************************************
  LOCAL TYPEDEFS
*****************************************************************************/
/*! Structure to store global data for determining the FIP lane matrix */
typedef struct FIPLaneMatrixGlobalData_t {
    float32 f_LeftTimerUnknownCurrentNumLane; /*!< Timer for the unknown current
                                                 number of lanes left */
    float32
        f_RightTimerUnknownCurrentNumLane;  /*!< Timer for the unknown current
                                               number of lanes right */
    float32 f_LeftTimerUnknownFiltNumLane;  /*!< Timer for the unknown filtered
                                               number of lanes left */
    float32 f_RightTimerUnknownFiltNumLane; /*!< Timer for the unknown filtered
                                               number of lanes right */
    float32 f_FusedNumLaneLeftFilt;  /*!< Filtered fused (EM, NAVI, camera)
                                        number of lanes left */
    float32 f_FusedNumLaneRightFilt; /*!< Filtered fused (EM, NAVI, camera)
                                        number of lanes right */
} t_FIPLaneMatrixGlobalData;

/*****************************************************************************
  LOCAL VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(FIPLaneMatrixGlobalData)
static t_FIPLaneMatrixGlobalData
    FIPLaneMatrixGlobalData; /*!< Structure to store global data for determining
                                the FIP lane matrix */

/*! Lane Width Class */
SET_MEMSEC_VAR(t_FIPLaneWidthClass)
static FIP_t_LaneWidthClass t_FIPLaneWidthClass;

/*! Lane Width */
SET_MEMSEC_VAR(f_LaneWidth)
static float32 f_FIPLaneWidth;

/*! Lane Width Source */
SET_MEMSEC_VAR(t_FIPLaneWidthClass)
static FIP_t_LaneWidthSource t_FIPLaneWidthSource;

SET_MEMSEC_VAR(FIPLaneMatrix)
/*! Debug information @vaddr: FIP_LANE_MATRIX_MEAS_FREEZE_VADDR @vname:
 * VLCFIPLaneMatrix @cycleid:VLC_ENV */
static LaneMatrix_t
    FIPLaneMatrix; /*!< Lane matrix which covers the information of EM Road,
                      NAVI and camera based on the VLC input preprocessing */
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! Meas freeze structure */
static const MEASInfo_t FIPLaneMatrixMeasInfo =

    {
        FIP_LANE_MATRIX_MEAS_FREEZE_VADDR, /*!<.VirtualAddress */
        sizeof(FIPLaneMatrix),             /*!<.Length */
        VLC_MEAS_FUNC_ID,                  /*!<.FuncID */
        VLC_MEAS_FUNC_CHAN_ID              /*!<.FuncChannelID */
};

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

static void FIP_v_SetLaneMatrix(LaneMatrix_t* p_LaneMatrix);
static void FIP_v_SetLaneWidthInfo(void);

static void FIP_v_SetLaneMatrixCam(sint8* ps_NumLaneLeft,
                                   sint8* ps_NumLaneRight);
static void FIP_v_SetLaneMatrixCamOneSide(
    sint8* ps_NumLane,
    t_CamLaneMarkerEnum const DirectLaneMarker,
    t_CamLaneMarkerEnum const AdjLaneMarker);
static void FIP_v_LaneMatrixLaneChange(LaneMatrix_t* p_LaneMatrix);
static void FIP_v_LaneMatrixFusion(LaneMatrix_t* p_LaneMatrix,
                                   boolean* pb_MultipleInputData,
                                   const sint8 s_EMNumLaneLeft,
                                   const sint8 s_EMNumLaneRight,
                                   const sint8 s_NaviNumLane,
                                   const sint8 s_CamNumLaneLeft,
                                   const sint8 s_CamNumLaneRight);
static void FIP_v_LaneMatrixFiltering(LaneMatrix_t* p_LaneMatrix,
                                      const boolean b_MultipleInputData,
                                      const sint8 s_EMNumLaneLeft,
                                      const sint8 s_EMNumLaneRight,
                                      const sint8 s_NaviNumLane,
                                      const sint8 s_CamNumLaneLeft,
                                      const sint8 s_CamNumLaneRight);
static void FIP_v_IsLMInfoIdentical(boolean* pb_InfoIsIdenticalLeft,
                                    boolean* pb_InfoIsIdenticalRight,
                                    const sint8 s_NumLaneLeft,
                                    const sint8 s_NumLaneRight,
                                    const sint8 s_EMNumLaneLeft,
                                    const sint8 s_EMNumLaneRight,
                                    const sint8 s_NaviNumLane,
                                    const sint8 s_CamNumLaneLeft,
                                    const sint8 s_CamNumLaneRight);
static void FIP_v_LaneMatrixFilteringOneSide(
    float32* pf_NumLaneFilt,
    float32* pf_TimerUnknownCurrentNumLane,
    float32* pf_TimerUnknownFiltNumLane,
    sint8 const s_NumLaneCurrent,
    boolean const b_MultipleInputData,
    t_SILCStateCamLaneMarkerCrossed const CamLaneMarkerCrossed,
    boolean const b_IsInfoIdentical);

/*************************************************************************************************************************
  Functionname:    FIP_v_LaneMatrixProcess */
void FIP_v_LaneMatrixProcess(void) {
    /*! Set the lane matrix for function input preprocessing */
    FIP_v_SetLaneMatrix(&FIPLaneMatrix);

    /*! Set Lane Width */
    FIP_v_SetLaneWidthInfo();

    /*! Freeze the meas info */
    // VLC_FREEZE_DATA(&FIPLaneMatrixMeasInfo, &FIPLaneMatrix, NULL);
}

/*************************************************************************************************************************
  Functionname:    FIP_v_SetLaneMatrix */
static void FIP_v_SetLaneMatrix(LaneMatrix_t* p_LaneMatrix) {
    sint8 s_NumLaneLeftEM,
        s_NumLaneRightEM; /*!< Number of lanes left/right based on EM Road */
    sint8 s_NumLaneNavi;  /*!< Number of lanes (left + right + ego) based on the
                             NAVI data */
    sint8 s_NumLaneLeftCam, s_NumLaneRightCam; /*!< Number of lanes left/right
                                                  based on the camera */
    boolean b_MultipleInputData; /*!< Info if different input sources are
                                    available */

    /*! Default values of local variables */
    s_NumLaneLeftEM = FIP_LM_INVALID_NUM_LANE;
    s_NumLaneRightEM = FIP_LM_INVALID_NUM_LANE;
    s_NumLaneNavi = FIP_LM_INVALID_NUM_LANE;
    s_NumLaneLeftCam = FIP_LM_INVALID_NUM_LANE;
    s_NumLaneRightCam = FIP_LM_INVALID_NUM_LANE;
    b_MultipleInputData = FALSE;

    /*! Default: EM-Road */
    p_LaneMatrix->iNumOfLaneLeft = FIP_LM_INVALID_NUM_LANE;
    p_LaneMatrix->iNumOfLaneRight = FIP_LM_INVALID_NUM_LANE;

    /*! Set number of lanes based on the camera */
    FIP_v_SetLaneMatrixCam(&s_NumLaneLeftCam, &s_NumLaneRightCam);

    /*! Fusion of the three input sources for determining the FIP lane matrix:
     * EM Road, NAVI, camera */
    FIP_v_LaneMatrixFusion(p_LaneMatrix, &b_MultipleInputData, s_NumLaneLeftEM,
                           s_NumLaneRightEM, s_NumLaneNavi, s_NumLaneLeftCam,
                           s_NumLaneRightCam);

    /*! Determine the filtered fused lane matrix */
    FIP_v_LaneMatrixFiltering(p_LaneMatrix, b_MultipleInputData,
                              s_NumLaneLeftEM, s_NumLaneRightEM, s_NumLaneNavi,
                              s_NumLaneLeftCam, s_NumLaneRightCam);

    /*! Consider an ego lane change in FIP lane matrix */
    FIP_v_LaneMatrixLaneChange(p_LaneMatrix);
}

/*************************************************************************************************************************
  Functionname:    FIP_v_SetLaneWidthInfo */
static void FIP_v_SetLaneWidthInfo(void) {
    t_FIPLaneWidthClass = FIP_LANE_WIDTH_CLASS_UNKNOWN;
    f_FIPLaneWidth = 0.f;
    t_FIPLaneWidthSource = FIP_SOURCE_0;
}

/*************************************************************************************************************************
  Functionname:    FIP_v_LaneMatrixFusion */
static void FIP_v_LaneMatrixFusion(LaneMatrix_t* p_LaneMatrix,
                                   boolean* pb_MultipleInputData,
                                   const sint8 s_EMNumLaneLeft,
                                   const sint8 s_EMNumLaneRight,
                                   const sint8 s_NaviNumLane,
                                   const sint8 s_CamNumLaneLeft,
                                   const sint8 s_CamNumLaneRight) {
    boolean b_InputDataFused;

    /*! Determine if different input sources available are */
    *pb_MultipleInputData = FALSE;
    if ((s_CamNumLaneLeft != FIP_LM_INVALID_NUM_LANE) ||
        (s_CamNumLaneRight != FIP_LM_INVALID_NUM_LANE) || (s_NaviNumLane > 0)) {
        *pb_MultipleInputData = TRUE;
    }

    /*! General remarks for combing the different input sources:
        NAVI: If the NAVI data is plausible, the number of lanes can be trusted
       (but the NAVI cannot distinguish between left and right lanes) Camera:
       The camera is only good in detecting no adjacent lanes (continuous camera
       lane marker) EM Road: If the EM Road data reports two valid lanes, more
       trust is placed in the lower number of lanes */
    b_InputDataFused = FALSE; /*!< Info if input data is fused */
    /*! If the NAVI reports a valid number of lanes */
    if (s_NaviNumLane > 0) {
        /*! Combine the information from the NAVI and the camera (camera
         * information is only considered if it tells that there is no adjacent
         * lane) */
        if (s_NaviNumLane > 1) {
            /*! If the NAVI reports more than one lane and if the camera reports
             * at only one side no adjacent lane, combine the information */
            if ((s_CamNumLaneLeft == 0) && (s_CamNumLaneRight != 0)) {
                p_LaneMatrix->iNumOfLaneLeft = 0;
                p_LaneMatrix->iNumOfLaneRight = s_NaviNumLane - 1;
                b_InputDataFused = TRUE;
            } else if ((s_CamNumLaneRight == 0) && (s_CamNumLaneLeft != 0)) {
                p_LaneMatrix->iNumOfLaneRight = 0;
                p_LaneMatrix->iNumOfLaneLeft = s_NaviNumLane - 1;
                b_InputDataFused = TRUE;
            } else {
                /*! Nothing */
            }
        } else if ((s_CamNumLaneLeft == 0) && (s_CamNumLaneRight == 0)) {
            /*! If the NAVI and the camera both report only one lane, use this
             * info */
            p_LaneMatrix->iNumOfLaneLeft = 0;
            p_LaneMatrix->iNumOfLaneRight = 0;
            b_InputDataFused = TRUE;
        } else {
            /*! Nothing */
        }

        /*! If NAVI and camera data is not fused, consider EM and NAVI data */
        if (b_InputDataFused == FALSE) {
            /*! If EM road data is invalid for only one lane, the NAVI data and
             * the EM road data based on the valid EM road info is combined */
            /*! If EM road data is valid for both sides, more trust is placed in
             * the lower number of lanes and the info from this side is combined
             * with the NAVI info */
            if (((s_EMNumLaneLeft != FIP_LM_INVALID_NUM_LANE) &&
                 (s_EMNumLaneRight == FIP_LM_INVALID_NUM_LANE)) ||
                ((s_EMNumLaneLeft != FIP_LM_INVALID_NUM_LANE) &&
                 (s_EMNumLaneRight != FIP_LM_INVALID_NUM_LANE) &&
                 (s_EMNumLaneLeft < s_EMNumLaneRight))) {
                p_LaneMatrix->iNumOfLaneLeft = s_EMNumLaneLeft;
                p_LaneMatrix->iNumOfLaneRight =
                    MAX(0, (s_NaviNumLane - s_EMNumLaneLeft) - 1);
            } else if (((s_EMNumLaneLeft == FIP_LM_INVALID_NUM_LANE) &&
                        (s_EMNumLaneRight != FIP_LM_INVALID_NUM_LANE)) ||
                       ((s_EMNumLaneLeft != FIP_LM_INVALID_NUM_LANE) &&
                        (s_EMNumLaneRight != FIP_LM_INVALID_NUM_LANE) &&
                        (s_EMNumLaneRight < s_EMNumLaneLeft))) {
                p_LaneMatrix->iNumOfLaneRight = s_EMNumLaneRight;
                p_LaneMatrix->iNumOfLaneLeft =
                    MAX(0, (s_NaviNumLane - s_EMNumLaneRight) - 1);
            } else {
                /*! Nothing */
            }
        }
    } else {
        /*! If the NAVI doesn't report a valid number of lanes, combine EM road
           data and camera data. -These input sources are only combined, if the
           camera tell that there is no adjacent lane and EM reports no valid
           lane.
            -If the distance to the road border is lower than a threshold, use
           the camera data */
        if (((s_EMNumLaneLeft == FIP_LM_INVALID_NUM_LANE) &&
             (s_CamNumLaneLeft == 0))) {
            p_LaneMatrix->iNumOfLaneLeft = 0;
        }
        if (((s_EMNumLaneRight == FIP_LM_INVALID_NUM_LANE) &&
             (s_CamNumLaneRight == 0)) ||
            ((s_CamNumLaneRight == 0))) {
            p_LaneMatrix->iNumOfLaneRight = 0;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_LaneMatrixFiltering */
static void FIP_v_LaneMatrixFiltering(LaneMatrix_t* p_LaneMatrix,
                                      const boolean b_MultipleInputData,
                                      const sint8 s_EMNumLaneLeft,
                                      const sint8 s_EMNumLaneRight,
                                      const sint8 s_NaviNumLane,
                                      const sint8 s_CamNumLaneLeft,
                                      const sint8 s_CamNumLaneRight) {
    boolean b_IsInfoIdenticalLeft,
        b_IsInfoIdenticalRight; /*!< Boolean if the different input sources give
                                   the same information */
    t_SILCStateCamLaneMarkerCrossed
        CamLaneMarkerCrossed; /*!< Info if camera lane markers are crossed */

    /*! Check if the camera lane markers are crossed;
        Remark: CamLaneMarkerCrossed from last cycle due to invocation order
       (FIP before si_lanechange.c) */
    CamLaneMarkerCrossed = SI_t_GetCamLaneMarkerCrossed();

    /*! Check if the different input sources give the same information -> then
     * this information can be trusted */
    FIP_v_IsLMInfoIdentical(&b_IsInfoIdenticalLeft, &b_IsInfoIdenticalRight,
                            p_LaneMatrix->iNumOfLaneLeft,
                            p_LaneMatrix->iNumOfLaneRight, s_EMNumLaneLeft,
                            s_EMNumLaneRight, s_NaviNumLane, s_CamNumLaneLeft,
                            s_CamNumLaneRight);

    /*! Filtering of the numbers of lanes left */
    FIP_v_LaneMatrixFilteringOneSide(
        &(FIPLaneMatrixGlobalData.f_FusedNumLaneLeftFilt),
        &(FIPLaneMatrixGlobalData.f_LeftTimerUnknownCurrentNumLane),
        &(FIPLaneMatrixGlobalData.f_LeftTimerUnknownFiltNumLane),
        p_LaneMatrix->iNumOfLaneLeft, b_MultipleInputData, CamLaneMarkerCrossed,
        b_IsInfoIdenticalLeft);
    /*! Filtering of the numbers of lanes right */
    FIP_v_LaneMatrixFilteringOneSide(
        &(FIPLaneMatrixGlobalData.f_FusedNumLaneRightFilt),
        &(FIPLaneMatrixGlobalData.f_RightTimerUnknownCurrentNumLane),
        &(FIPLaneMatrixGlobalData.f_RightTimerUnknownFiltNumLane),
        p_LaneMatrix->iNumOfLaneRight, b_MultipleInputData,
        CamLaneMarkerCrossed, b_IsInfoIdenticalRight);

    /*! Set number of lanes based on filtered number of lanes values */
    p_LaneMatrix->iNumOfLaneLeft =
        (sint8)ROUND_TO_INT(FIPLaneMatrixGlobalData.f_FusedNumLaneLeftFilt);
    p_LaneMatrix->iNumOfLaneRight =
        (sint8)ROUND_TO_INT(FIPLaneMatrixGlobalData.f_FusedNumLaneRightFilt);
}

/*************************************************************************************************************************
  Functionname:    FIP_v_LaneMatrixFilteringOneSide */
static void FIP_v_LaneMatrixFilteringOneSide(
    float32* pf_NumLaneFilt,
    float32* pf_TimerUnknownCurrentNumLane,
    float32* pf_TimerUnknownFiltNumLane,
    sint8 const s_NumLaneCurrent,
    boolean const b_MultipleInputData,
    t_SILCStateCamLaneMarkerCrossed const CamLaneMarkerCrossed,
    boolean const b_IsInfoIdentical) {
    sint8 s_RoundNumLaneFilt;

    /*! Conditions under which the filtering is ignored and the filtered number
       of lanes value is directly set to the current number of lanes: the camera
       lane markers are crossed -> filtering would cause the number of lanes to
       be too "slow" only EM road as input available (b_MultipleInputData ==
       FALSE) -> no second filtering necessary, use EM number of lanes (default)
       directly the information from the different input sources give the same
       info -> info can be trusted and the "correct" number of lanes is set
       directly invalid filtered number of lanes or the current number -> no
       filtering since a wrong value may occure by filtering, either leave the
       old value (hysteresis) or set the invalid value directly
                (filtering from e.g. 1 with the value -1 may result in the wrong
       number of lanes 0) */
    if ((CamLaneMarkerCrossed != UNKNOWN_CROSS_CAMLANE) ||
        (b_MultipleInputData == FALSE) || (b_IsInfoIdentical == TRUE) ||
        (s_NumLaneCurrent == FIP_LM_INVALID_NUM_LANE) ||
        (*pf_NumLaneFilt < ((float32)FIP_LM_INVALID_NUM_LANE + C_F32_DELTA))) {
        /*! If the current number of lanes is invalid, change the filtered
         * number of lanes to invalid after a hysteresis */
        if ((s_NumLaneCurrent == FIP_LM_INVALID_NUM_LANE) &&
            (*pf_TimerUnknownCurrentNumLane <
             FIP_LM_TIMER_INVAILD_CURRENT_NUM_LANE) &&
            (b_MultipleInputData == TRUE) && (b_IsInfoIdentical == FALSE)) {
            *pf_TimerUnknownCurrentNumLane += VLC_CYCLE_TIME;
        } else if ((s_NumLaneCurrent != FIP_LM_INVALID_NUM_LANE) &&
                   (*pf_NumLaneFilt <
                    ((float32)FIP_LM_INVALID_NUM_LANE + C_F32_DELTA)) &&
                   (*pf_TimerUnknownFiltNumLane <
                    FIP_LM_TIMER_INVAILD_FILT_NUM_LANE) &&
                   (b_MultipleInputData == TRUE) &&
                   (b_IsInfoIdentical == FALSE)) {
            /*! If only the filtered number of lanes is invalid, change the
             * filtered number of lanes to the new value after a hysteresis */
            *pf_TimerUnknownFiltNumLane += VLC_CYCLE_TIME;
        } else {
            /*! Set the filtered number of lanes to the current number of lanes
             * directly */
            *pf_NumLaneFilt = (float32)s_NumLaneCurrent;
        }
    } else {
        /*! Filtering of the number of lanes */
        *pf_NumLaneFilt = GDB_FILTER((float32)s_NumLaneCurrent, *pf_NumLaneFilt,
                                     FIP_LM_FILTER_CONSTANT);
    }

    /*! Reset the timer for the unknown current number of lanes if the current
     * number of lanes is valid */
    if (s_NumLaneCurrent != FIP_LM_INVALID_NUM_LANE) {
        *pf_TimerUnknownCurrentNumLane = 0.f;
    }

    /*! Reset the timer for the unknown number of filtered lanes if the current
     * number of lanes is invalid or the filtered value is valid */
    s_RoundNumLaneFilt = (sint8)ROUND_TO_INT(*pf_NumLaneFilt);
    if ((s_NumLaneCurrent == (sint8)FIP_LM_INVALID_NUM_LANE) ||
        (s_RoundNumLaneFilt > (sint8)FIP_LM_INVALID_NUM_LANE)) {
        *pf_TimerUnknownFiltNumLane = 0.f;
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_IsLMInfoIdentical */
static void FIP_v_IsLMInfoIdentical(boolean* pb_InfoIsIdenticalLeft,
                                    boolean* pb_InfoIsIdenticalRight,
                                    const sint8 s_NumLaneLeft,
                                    const sint8 s_NumLaneRight,
                                    const sint8 s_EMNumLaneLeft,
                                    const sint8 s_EMNumLaneRight,
                                    const sint8 s_NaviNumLane,
                                    const sint8 s_CamNumLaneLeft,
                                    const sint8 s_CamNumLaneRight) {
    /*! Set default: Different input sources give not the same information */
    *pb_InfoIsIdenticalLeft = FALSE;
    *pb_InfoIsIdenticalRight = FALSE;

    /*! Check if the different input sources give the same information -> then
     * this information can be trusted */
    if (((s_EMNumLaneLeft == FIP_LM_INVALID_NUM_LANE) ||
         (s_CamNumLaneLeft == FIP_LM_INVALID_NUM_LANE) ||
         ((s_EMNumLaneLeft == s_CamNumLaneLeft) &&
          (s_NumLaneLeft ==
           s_EMNumLaneLeft))) && /*!< EM Road left and camera left identical +
                                    EM Road left and fused lane identical */
        ((s_EMNumLaneLeft == FIP_LM_INVALID_NUM_LANE) ||
         (s_EMNumLaneRight == FIP_LM_INVALID_NUM_LANE) ||
         (s_NaviNumLane == FIP_LM_INVALID_NUM_LANE) ||
         ((s_NaviNumLane == (s_EMNumLaneLeft + s_EMNumLaneRight + 1)) &&
          (s_NumLaneLeft == s_EMNumLaneLeft) &&
          (s_NumLaneRight ==
           s_EMNumLaneRight))) && /*!< NAVI and EM Road identical + EM Road and
                                     fused lane identical */
        ((s_CamNumLaneLeft == FIP_LM_INVALID_NUM_LANE) ||
         (s_CamNumLaneRight == FIP_LM_INVALID_NUM_LANE) ||
         (s_NaviNumLane == FIP_LM_INVALID_NUM_LANE) ||
         ((s_NaviNumLane == (s_CamNumLaneLeft + s_CamNumLaneRight + 1)) &&
          (s_NumLaneLeft == s_CamNumLaneLeft) &&
          (s_NumLaneRight ==
           s_CamNumLaneRight))) && /*!< NAVI and camera identical + Camera and
                                      fused lane identical */
        (((s_EMNumLaneLeft != FIP_LM_INVALID_NUM_LANE) &&
          (s_CamNumLaneLeft != FIP_LM_INVALID_NUM_LANE)) ||
         ((s_EMNumLaneLeft != FIP_LM_INVALID_NUM_LANE) &&
          (s_EMNumLaneRight != FIP_LM_INVALID_NUM_LANE) &&
          (s_NaviNumLane != FIP_LM_INVALID_NUM_LANE)) ||
         ((s_CamNumLaneLeft != FIP_LM_INVALID_NUM_LANE) &&
          (s_CamNumLaneRight != FIP_LM_INVALID_NUM_LANE) &&
          (s_NaviNumLane !=
           FIP_LM_INVALID_NUM_LANE))) /*!< Ensure that at least one of the three
                                         conditions above is fulfilled based on
                                         valid data */
        ) {
        *pb_InfoIsIdenticalLeft = TRUE;
    }

    if (((s_EMNumLaneRight == FIP_LM_INVALID_NUM_LANE) ||
         (s_CamNumLaneRight == FIP_LM_INVALID_NUM_LANE) ||
         ((s_EMNumLaneRight == s_CamNumLaneRight) &&
          (s_NumLaneRight ==
           s_EMNumLaneRight))) && /*!< EM Road right and camera right identical
                                     + EM Road right and fused lane identical */
        ((s_EMNumLaneLeft == FIP_LM_INVALID_NUM_LANE) ||
         (s_EMNumLaneRight == FIP_LM_INVALID_NUM_LANE) ||
         (s_NaviNumLane == FIP_LM_INVALID_NUM_LANE) ||
         ((s_NaviNumLane == (s_EMNumLaneLeft + s_EMNumLaneRight + 1)) &&
          (s_NumLaneLeft == s_EMNumLaneLeft) &&
          (s_NumLaneRight ==
           s_EMNumLaneRight))) && /*!< NAVI and EM Road identical + EM Road and
                                     fused lane identical */
        ((s_CamNumLaneLeft == FIP_LM_INVALID_NUM_LANE) ||
         (s_CamNumLaneRight == FIP_LM_INVALID_NUM_LANE) ||
         (s_NaviNumLane == FIP_LM_INVALID_NUM_LANE) ||
         ((s_NaviNumLane == (s_CamNumLaneLeft + s_CamNumLaneRight + 1)) &&
          (s_NumLaneLeft == s_CamNumLaneLeft) &&
          (s_NumLaneRight ==
           s_CamNumLaneRight))) && /*!< NAVI and camera identical + Camera and
                                      fused lane identical */
        (((s_EMNumLaneRight != FIP_LM_INVALID_NUM_LANE) &&
          (s_CamNumLaneRight != FIP_LM_INVALID_NUM_LANE)) ||
         ((s_EMNumLaneLeft != FIP_LM_INVALID_NUM_LANE) &&
          (s_EMNumLaneRight != FIP_LM_INVALID_NUM_LANE) &&
          (s_NaviNumLane != FIP_LM_INVALID_NUM_LANE)) ||
         ((s_CamNumLaneLeft != FIP_LM_INVALID_NUM_LANE) &&
          (s_CamNumLaneRight != FIP_LM_INVALID_NUM_LANE) &&
          (s_NaviNumLane !=
           FIP_LM_INVALID_NUM_LANE))) /*!< Ensure that at least one of the three
                                         conditions above is fulfilled based on
                                         valid data */
        ) {
        *pb_InfoIsIdenticalRight = TRUE;
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_LaneMatrixLaneChange */
static void FIP_v_LaneMatrixLaneChange(LaneMatrix_t* p_LaneMatrix) {
    sint8 s_ChangeLeftLaneLC, s_ChangeRightLaneLC, s_NumLane, s_NumLeftLaneTemp,
        s_NumRightLaneTemp;
    t_SILaneChangeCamPreMove t_LaneChangeMovePreState;

    /*! Get the lane change direction. Remark: Information from last cycle */
    t_LaneChangeMovePreState = SI_t_GetLaneChangeMovePre();

    /*! In case of a lane change adapt the number of lanes in advance */
    if (t_LaneChangeMovePreState == LANE_CHANGE_CAM_PRE_MOVE_LEFT) {
        s_ChangeLeftLaneLC = -1; /*!< Lane change left: Decrease the left number
                                    of lanes by one */
        s_ChangeRightLaneLC = 1; /*!< Lane change left: Increase the right
                                    number of lanes by one */
    } else if (t_LaneChangeMovePreState == LANE_CHANGE_CAM_PRE_MOVE_RIGHT) {
        s_ChangeLeftLaneLC = 1; /*!< Lane change right: Increase the left number
                                   of lanes by one */
        s_ChangeRightLaneLC = -1; /*!< Lane change right: Decrease the right
                                     number of lanes by one */
    } else {
        /*! No lane change: No adaption of number of lanes */
        s_ChangeLeftLaneLC = 0;
        s_ChangeRightLaneLC = 0;
    }

    /*! Change the number of lanes based on the ego lane change only if the
       number of lanes is not invalid. Further, the total number of lanes must
       not be changed by the lane change consideration */

    /*! First, get the total number of lanes based on the filtered number of
       lanes for the left and right side
        -> can only be determined if the number of lanes is available for both
       sides */
    s_NumLane = FIP_LM_INVALID_NUM_LANE;
    if ((p_LaneMatrix->iNumOfLaneLeft > FIP_LM_INVALID_NUM_LANE) &&
        (p_LaneMatrix->iNumOfLaneRight > FIP_LM_INVALID_NUM_LANE)) {
        s_NumLane =
            p_LaneMatrix->iNumOfLaneLeft + p_LaneMatrix->iNumOfLaneRight + 1;
    }
    /*! Second, determine the number of lanes under consideration of the lane
     * change (only if the number of lanes is valid)*/
    s_NumLeftLaneTemp = FIP_LM_INVALID_NUM_LANE;
    s_NumRightLaneTemp = FIP_LM_INVALID_NUM_LANE;
    if (p_LaneMatrix->iNumOfLaneLeft != FIP_LM_INVALID_NUM_LANE) {
        s_NumLeftLaneTemp =
            MAX(0, p_LaneMatrix->iNumOfLaneLeft + s_ChangeLeftLaneLC);
    }
    if (p_LaneMatrix->iNumOfLaneRight != FIP_LM_INVALID_NUM_LANE) {
        s_NumRightLaneTemp =
            MAX(0, p_LaneMatrix->iNumOfLaneRight + s_ChangeRightLaneLC);
    }

    /*! Third, set the output value, only if the total number of lanes doesn't
       change by the lane change consideration. If the total number of lanes
       (s_NumLane) cannot be determined due to one invalid number of lane for
       one side, set the new number of lane under consideration of the lane
       change only for the valid side. */
    if (((s_NumLeftLaneTemp > FIP_LM_INVALID_NUM_LANE) &&
         (s_NumRightLaneTemp > FIP_LM_INVALID_NUM_LANE) &&
         ((s_NumLeftLaneTemp + s_NumRightLaneTemp + 1) == s_NumLane)) ||
        ((p_LaneMatrix->iNumOfLaneLeft == FIP_LM_INVALID_NUM_LANE) &&
         (p_LaneMatrix->iNumOfLaneRight != FIP_LM_INVALID_NUM_LANE)) ||
        ((p_LaneMatrix->iNumOfLaneLeft != FIP_LM_INVALID_NUM_LANE) &&
         (p_LaneMatrix->iNumOfLaneRight == FIP_LM_INVALID_NUM_LANE))) {
        if (p_LaneMatrix->iNumOfLaneLeft != FIP_LM_INVALID_NUM_LANE) {
            p_LaneMatrix->iNumOfLaneLeft = s_NumLeftLaneTemp;
        }
        if (p_LaneMatrix->iNumOfLaneRight != FIP_LM_INVALID_NUM_LANE) {
            p_LaneMatrix->iNumOfLaneRight = s_NumRightLaneTemp;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_SetLaneMatrixCam */
static void FIP_v_SetLaneMatrixCam(sint8* ps_NumLaneLeft,
                                   sint8* ps_NumLaneRight) {
    float32 const f_CamLaneVisibilityDist =
        FIP_f_GetVisibilityDistCamLane(); /*!< Visibility distance of camera
                                             lane */

    /*! Set default values */
    *ps_NumLaneLeft = FIP_LM_INVALID_NUM_LANE;
    *ps_NumLaneRight = FIP_LM_INVALID_NUM_LANE;

    /*! Consider camera information only if visibility higher than a threshold
     */
    if (f_CamLaneVisibilityDist > C_F32_DELTA) {
        /*! Set the number of lanes for the left side */
        FIP_v_SetLaneMatrixCamOneSide(ps_NumLaneLeft, CL_CAM_LANE_MK_LEFT,
                                      CL_CAM_LANE_MK_ADJ_LEFT);
        /*! Set the number of lanes for the right side */
        FIP_v_SetLaneMatrixCamOneSide(ps_NumLaneRight, CL_CAM_LANE_MK_RIGHT,
                                      CL_CAM_LANE_MK_ADJ_RIGHT);
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_SetLaneMatrixCamOneSide */
static void FIP_v_SetLaneMatrixCamOneSide(
    sint8* ps_NumLane,
    t_CamLaneMarkerEnum const DirectLaneMarker,
    t_CamLaneMarkerEnum const AdjLaneMarker) {
    float32 fLaneWidthAdj;
    boolean const b_CamLaneIsRoadEdge = FIP_b_GetIsCamLaneRoadEdge(
        DirectLaneMarker); /*!< Info if direct camera lane is road edge */

    /*! Remark: The number of lanes reported by the camera can only be trusted
     * if it equals zero -> Only the value zero is set or unknown/invalid */
    /*! Set the number of lanes to zeros for a continuous line */
    if (b_CamLaneIsRoadEdge == TRUE) {
        (*ps_NumLane) = 0;
    } else if ((VLCSEN_pCamLaneData->LaneMarkerInfo[DirectLaneMarker]
                    .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
               (VLCSEN_pCamLaneData->LaneMarkerInfo[AdjLaneMarker]
                    .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
               (VLCSEN_pCamLaneData->sSigHeader.eSigStatus ==
                AL_SIG_STATE_OK)) {
        /*! Determine the lane width of the adjacent lane */
        switch (DirectLaneMarker) {
            case CL_CAM_LANE_MK_LEFT:
                fLaneWidthAdj =
                    VLCSEN_pCamLaneData->LaneMarkerInfo[AdjLaneMarker]
                        .f_MarkerDist -
                    VLCSEN_pCamLaneData->LaneMarkerInfo[DirectLaneMarker]
                        .f_MarkerDist;
                break;
            case CL_CAM_LANE_MK_RIGHT:
                fLaneWidthAdj =
                    VLCSEN_pCamLaneData->LaneMarkerInfo[DirectLaneMarker]
                        .f_MarkerDist -
                    VLCSEN_pCamLaneData->LaneMarkerInfo[AdjLaneMarker]
                        .f_MarkerDist;
                break;
            default:
                fLaneWidthAdj = FIP_LM_DEFAULT_CAM_LANE_WIDTH; /*!< Invalid lane
                                                                  width value */
                break;
        }

        /*! If the lane width of the adjacent lane is smaller than a threshold
         * consider it as no adjacent lane on this side */
        if ((fLaneWidthAdj > 0.f) &&
            (fLaneWidthAdj < FIP_LM_MIN_ADJ_CAM_LANE_WIDTH)) {
            (*ps_NumLane) = 0;
        }
    } else {
        /*! Nothing */
    }
}

/*************************************************************************************************************************
  Functionname:    FIP_v_InitGlobalLaneMatrixData */
void FIP_v_InitGlobalLaneMatrixData(void) {
    FIPLaneMatrixGlobalData.f_LeftTimerUnknownCurrentNumLane = 0.f;
    FIPLaneMatrixGlobalData.f_RightTimerUnknownCurrentNumLane = 0.f;
    FIPLaneMatrixGlobalData.f_LeftTimerUnknownFiltNumLane = 0.f;
    FIPLaneMatrixGlobalData.f_RightTimerUnknownFiltNumLane = 0.f;
    FIPLaneMatrixGlobalData.f_FusedNumLaneLeftFilt = -1.f;
    FIPLaneMatrixGlobalData.f_FusedNumLaneRightFilt = -1.f;

    FIPLaneMatrix.iNumOfLaneLeft = -1;
    FIPLaneMatrix.iNumOfLaneRight = -1;
}

/*************************************************************************************************************************
  Functionname:    FIP_v_InitLaneWidthInfo */
void FIP_v_InitLaneWidthInfo(void) {
    t_FIPLaneWidthClass = FIP_LANE_WIDTH_CLASS_UNKNOWN;
    f_FIPLaneWidth = 0.f;
    t_FIPLaneWidthSource = FIP_SOURCE_0;
}

/*************************************************************************************************************************
  Functionname:    FIP_s_GetLMLeftNumLane */
sint8 FIP_s_GetLMLeftNumLane(void) { return FIPLaneMatrix.iNumOfLaneLeft; }

/*************************************************************************************************************************
  Functionname:    FIP_s_GetLMLeftNumLane */
sint8 FIP_s_GetLMRightNumLane(void) { return FIPLaneMatrix.iNumOfLaneRight; }

/*************************************************************************************************************************
  Functionname:    FIP_t_Get_LaneWidthClass */
FIP_t_LaneWidthClass FIP_t_Get_LaneWidthClass(void) {
    return (t_FIPLaneWidthClass);
}

/*************************************************************************************************************************
  Functionname:    FIP_t_Get_LaneWidthSource */
FIP_t_LaneWidthSource FIP_t_Get_LaneWidthSource(void) {
    return (t_FIPLaneWidthSource);
}

/*************************************************************************************************************************
  Functionname:    FIP_f_Get_LaneWidth */
float32 FIP_f_Get_LaneWidth(void) { return (f_FIPLaneWidth); }

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */