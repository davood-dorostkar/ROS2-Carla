/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "si.h"
#include "cp_si.h"
#include "si_par.h"
//#include "vlc_par.h"
#include "si_corridor_cam.h"

/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/*! The lane change specific variable: need to be known beyond one cycle */
SET_MEMSEC_VAR(SILaneChangeBracket)
static struct {
    float32 fMarkerDistLeftHistory
        [SI_LANE_CHANGE_HISTORY_BUFFER]; /*!< Buffer: distance to the lane
                                            marker on the left side */
    float32 fMarkerDistRightHistory
        [SI_LANE_CHANGE_HISTORY_BUFFER]; /*!< Buffer: distance to the lane
                                            marker on the right side */
    SILaneChangeTraceBracketState_t
        LaneChgTBState;     /*!< SILaneChangeTraceBracketState_t: Info about the
                               last lane change state */
    boolean bMarkerCrossed; /*!< Information if lane marker was crossed */
    boolean bLaneChangeIsActive; /*!< Information if a lane change is detected
                                    (with hystersis) */
    boolean bBlinkerFeatureTriggered; /*!< Information if trace brackets are
                                         adapted based on blinker feature */
} SILaneChangeBracket;

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(SILaneChangeState)
SIScaleBracketState_t SILaneChangeState;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
/*! Functions related to trace bracket adaption based on the ego position in
 * lane */

static void SI_v_ResetAdaptTBForLCBasedOnLCProb(
    sint16 const i_SILaneChangeProb,
    SILaneChangeTraceBracketState_t const LaneChangeTBState);
static void SI_v_FillCamLaneMarkerDistHistoryBuffer(void);

static void SI_v_AdaptTBForLCBasedOnLCProb(
    float32 *pf_DistMarkerLeft,
    float32 *pf_DistMarkerRight,
    SIScaleBracketState_t *p_StateScaleBracket,
    sint16 const i_SILaneChangeProb,
    SILaneChangeTraceBracketState_t const LaneChangeTBState);
static void SI_v_AdaptTBIfNoLC(float32 *const pf_DistMarkerLeft,
                               float32 *const pf_DistMarkerRight,
                               SIScaleBracketState_t *const p_StateScaleBracket,
                               boolean const b_LCProbValid,
                               sint16 const i_SILaneChangeProb);
static void SI_v_AdaptTBForLCBasedOnBlinkerFeature(
    float32 *const pf_DistMarkerLeft,
    float32 *const pf_DistMarkerRight,
    SIScaleBracketState_t *const p_StateScaleBracket,
    sint16 const i_SILaneChangeProb,
    boolean const b_ValidLCProb);
static void SI_v_FillCamLaneMarkerDistHistoryBuffer(void);

/*************************************************************************************************************************
  Functionname:    SIInitCorridorCamParameter */
void SIInitCorridorCamParameter(void) {
    uint8 i;
    for (i = 0u; i < SI_LANE_CHANGE_HISTORY_BUFFER; i++) {
        SILaneChangeBracket.fMarkerDistLeftHistory[i] =
            SI_PAR_INVALIDE_LANE_MARKER_DIST;
        SILaneChangeBracket.fMarkerDistRightHistory[i] =
            SI_PAR_INVALIDE_LANE_MARKER_DIST;
    }
    SILaneChangeBracket.LaneChgTBState = SI_NO_TB_LANE_CHANGE;
    SILaneChangeBracket.bMarkerCrossed = FALSE;
    SILaneChangeBracket.bLaneChangeIsActive = FALSE;
    SILaneChangeBracket.bBlinkerFeatureTriggered = FALSE;
}

/*************************************************************************************************************************
  Functionname:    SIRelTraInitCriteriaOutputScale */
void SIRelTraInitCriteriaOutputScale(SIScaleBracketOutput_t *const pOutput) {
    pOutput->fScaleBracketLeft = INITVALUE_BRACKETPOSITION;
    pOutput->fScaleBracketRight = INITVALUE_BRACKETPOSITION;
    pOutput->StateScaleBracket = UNKNOWN;
}

/*************************************************************************************************************************
  Functionname:    SIRelTraRatioEgoPositionInLaneCam */
void SIRelTraRatioEgoPositionInLaneCam(
    SIScaleBracketOutput_t *const pScaleBracketOut) {
    /*! Declaration of local variables */
    boolean b_LCProbValid;
    sint16 iSILaneChangeProb;
    float32 fDistMarkerLeft, fDistMarkerRight, fLaneWidthHalfCam;
    float32 f_CamLaneVisibilityDist;
    SILaneChangeTraceBracketState_t eLaneChangeHistState;

    /*! Initialize distance to camera lane markers */
    fDistMarkerLeft = INITVALUE_BRACKETPOSITION;
    fDistMarkerRight = INITVALUE_BRACKETPOSITION;

    /*! Initialize of reason for trace bracket adaption */
    pScaleBracketOut->StateScaleBracket = UNKNOWN;

    /*! If no camera lane information available during the course of maneuver,
     * the trace bracket adaption is reset (former information might have been
     * wrong) */
    f_CamLaneVisibilityDist =
        FIP_f_GetVisibilityDistCamLane(); /*!< Visibility distance of camera
                                             lane */
    if ((f_CamLaneVisibilityDist <= C_F32_DELTA) &&
        (SILaneChangeBracket.LaneChgTBState != SI_NO_TB_LANE_CHANGE)) {
        SILaneChangeBracket.LaneChgTBState = SI_NO_TB_LANE_CHANGE;
        SILaneChangeBracket.bMarkerCrossed = FALSE;
        SILaneChangeBracket.bLaneChangeIsActive = FALSE;
    }

    /*! Fill the history buffer of the ego-distance to the camera lane markers
     */
    SI_v_FillCamLaneMarkerDistHistoryBuffer();

    /*! Set lane change probability and validity flag of lane change probability
     */

    /*! Default values */
    iSILaneChangeProb = 0;
    b_LCProbValid = FALSE;
    /*! Information about lane change probability */
    if (f_CamLaneVisibilityDist > SI_MIN_CAM_COURSE_DIST) {
        iSILaneChangeProb = SILCGetLaneChangeProbability();
        b_LCProbValid = TRUE;
        /*! If lane change is already active, consider hysteresis */
        if (SILaneChangeBracket.bLaneChangeIsActive == TRUE) {
            /*! LC to the left */
            if (iSILaneChangeProb > (sint16)0) {
                iSILaneChangeProb += (sint16)SI_LC_PROB_LEVEL_HYSTERESIS;
            } else /*! LC to the right */
            {
                iSILaneChangeProb -= (sint16)SI_LC_PROB_LEVEL_HYSTERESIS;
            }
        }
    }

    /*! Evaluation of LC-probability: determine if lane change is detected
     * (hysteresis) */
    if ((b_LCProbValid == TRUE) &&
        (((EGO_SPEED_X_OBJ_SYNC <= SI_LEVEL_LOW_SPEED) &&
          (ABS(iSILaneChangeProb) > (sint16)SI_LC_PROB_LEVEL_LOW_SPEED)) ||
         ((EGO_SPEED_X_OBJ_SYNC > SI_LEVEL_LOW_SPEED) &&
          (ABS(iSILaneChangeProb) > (sint16)SI_LC_PROB_LEVEL_HIGH_SPEED)))) {
        /*! LC to the left */
        if (iSILaneChangeProb > (sint16)0) {
            eLaneChangeHistState = SI_TB_LANE_CHANGE_LEFT;
            SILaneChangeBracket.LaneChgTBState = SI_TB_LANE_CHANGE_LEFT;
            SILaneChangeBracket.bLaneChangeIsActive = TRUE;
        } else /*! LC to the right */
        {
            eLaneChangeHistState = SI_TB_LANE_CHANGE_RIGHT;
            SILaneChangeBracket.LaneChgTBState = SI_TB_LANE_CHANGE_RIGHT;
            SILaneChangeBracket.bLaneChangeIsActive = TRUE;
        }
    } else /*! No LC in this cycle */
    {
        eLaneChangeHistState = SI_NO_TB_LANE_CHANGE;
        SILaneChangeBracket.bLaneChangeIsActive = FALSE;
    }

    /*! Check reset conditions for history info and if conditions fulfilled,
     * reset history info */
    SI_v_ResetAdaptTBForLCBasedOnLCProb(iSILaneChangeProb,
                                        eLaneChangeHistState);

    SI_v_AdaptTBForLCBasedOnLCProb(&fDistMarkerLeft, &fDistMarkerRight,
                                   &(pScaleBracketOut->StateScaleBracket),
                                   iSILaneChangeProb, eLaneChangeHistState);

    SI_v_AdaptTBIfNoLC(&fDistMarkerLeft, &fDistMarkerRight,
                       &(pScaleBracketOut->StateScaleBracket), b_LCProbValid,
                       iSILaneChangeProb);

    SI_v_AdaptTBForLCBasedOnBlinkerFeature(
        &fDistMarkerLeft, &fDistMarkerRight,
        &(pScaleBracketOut->StateScaleBracket), iSILaneChangeProb,
        b_LCProbValid);

    /*! Determine the ratio (relative to middle position) of the ego-position in
       the lane for altering the trace brackets.
        In case of a tunnel, the distance to the camera lane cannot be trusted,
       hence, ignore the trace bracket adaption */
    if ((fDistMarkerLeft < SI_BRACKETPOS_VALID_VAL_COMPARE) &&
        (fDistMarkerRight < SI_BRACKETPOS_VALID_VAL_COMPARE) &&
        (TUNNEL_PROBABILITY < SI_TUNNEL_PROB_THRES)) {
        fLaneWidthHalfCam = (fDistMarkerLeft + fDistMarkerRight) * 0.5f;

        if (fLaneWidthHalfCam > C_F32_DELTA) {
            pScaleBracketOut->fScaleBracketLeft =
                fDistMarkerLeft / fLaneWidthHalfCam;
            pScaleBracketOut->fScaleBracketRight =
                fDistMarkerRight / fLaneWidthHalfCam;

            /*! In case of a low speed scenario (city-traffic), the lane
              brackets are altered less
              (cars in other lanes are closer to the ego-lane, lane association
              difficult) */
            if (EGO_SPEED_X_OBJ_SYNC <= SI_LEVEL_LOW_SPEED) {
                /*! Left trace bracket becomes larger than before */
                if (pScaleBracketOut->fScaleBracketLeft > 1.f) {
                    pScaleBracketOut->fScaleBracketLeft =
                        1.f +
                        ((pScaleBracketOut->fScaleBracketLeft - 1.f) * 0.5f);
                    pScaleBracketOut->fScaleBracketRight =
                        2.f - pScaleBracketOut->fScaleBracketLeft;
                } else /*! Right trace bracket becomes larger than before */
                {
                    pScaleBracketOut->fScaleBracketRight =
                        1.f +
                        ((pScaleBracketOut->fScaleBracketRight - 1.f) * 0.5f);
                    pScaleBracketOut->fScaleBracketLeft =
                        2.f - pScaleBracketOut->fScaleBracketRight;
                }
            }
        } else /*! No adaption of trace brackets */
        {
            pScaleBracketOut->fScaleBracketLeft = INITVALUE_BRACKETPOSITION;
            pScaleBracketOut->fScaleBracketRight = INITVALUE_BRACKETPOSITION;
        }
    } else /*! No adaption of trace brackets */
    {
        pScaleBracketOut->fScaleBracketLeft = INITVALUE_BRACKETPOSITION;
        pScaleBracketOut->fScaleBracketRight = INITVALUE_BRACKETPOSITION;
    }

    /*! Set the global variable SILaneChangeState to exchange it with other
     * modules */
    SILaneChangeState = pScaleBracketOut->StateScaleBracket;
}

/*************************************************************************************************************************
  Functionname:    SI_v_FillCamLaneMarkerDistHistoryBuffer */
static void SI_v_FillCamLaneMarkerDistHistoryBuffer(void) {
    uint8 i;
    /*! Fill the history buffer of the ego-distance to the camera lane markers
     * only if the distance changed (different cycle times); order: latest value
     * first */
    /*! Left camera lane marker */
    /*! If the camera lane marker is valid */
    if ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
        (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK)) {
        const float32 fDeltaDist =
            (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                 .f_MarkerDist -
             SILaneChangeBracket.fMarkerDistLeftHistory[0u]);
        /*! Only if the distance changed */
        if (fABS(fDeltaDist) > C_F32_DELTA) {
            /*! Move entries in array */
            for (i = SI_LANE_CHANGE_HISTORY_BUFFER - 1u; i >= 1u; i--) {
                SILaneChangeBracket.fMarkerDistLeftHistory[i] =
                    SILaneChangeBracket.fMarkerDistLeftHistory[i - 1u];
            }
            /*! Set new value */
            SILaneChangeBracket.fMarkerDistLeftHistory[0u] =
                VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                    .f_MarkerDist;
        }
    } else {
        /*! If the camera lane marker is not valid, set default invalid value */
        /*! Move entries in array */
        for (i = SI_LANE_CHANGE_HISTORY_BUFFER - 1u; i >= 1u; i--) {
            SILaneChangeBracket.fMarkerDistLeftHistory[i] =
                SILaneChangeBracket.fMarkerDistLeftHistory[i - 1u];
        }
        /*! Set default invalid value */
        SILaneChangeBracket.fMarkerDistLeftHistory[0u] =
            SI_PAR_INVALIDE_LANE_MARKER_DIST;
    }

    /*! Right camera lane marker */
    /*! If the camera lane marker is valid */
    if ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
        (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK)) {
        const float32 fDeltaDist =
            VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                .f_MarkerDist -
            SILaneChangeBracket.fMarkerDistRightHistory[0u];
        /*! Only if the distance changed */
        if (fABS(fDeltaDist) > C_F32_DELTA) {
            /*! Move entries in array */
            for (i = SI_LANE_CHANGE_HISTORY_BUFFER - 1u; i >= 1u; i--) {
                SILaneChangeBracket.fMarkerDistRightHistory[i] =
                    SILaneChangeBracket.fMarkerDistRightHistory[i - 1u];
            }
            /*! Set new value */
            SILaneChangeBracket.fMarkerDistRightHistory[0u] =
                VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                    .f_MarkerDist;
        }
    } else {
        /*! If the camera lane marker is not valid, set default invalid value */
        /*! Move entries in array */
        for (i = SI_LANE_CHANGE_HISTORY_BUFFER - 1u; i >= 1u; i--) {
            SILaneChangeBracket.fMarkerDistRightHistory[i] =
                SILaneChangeBracket.fMarkerDistRightHistory[i - 1u];
        }
        /*! Set default invalid value */
        SILaneChangeBracket.fMarkerDistRightHistory[0u] =
            SI_PAR_INVALIDE_LANE_MARKER_DIST;
    }
}

/*************************************************************************************************************************
  Functionname:    SI_v_ResetAdaptTBForLCBasedOnLCProb */
static void SI_v_ResetAdaptTBForLCBasedOnLCProb(
    sint16 const i_SILaneChangeProb,
    SILaneChangeTraceBracketState_t const LaneChangeTBState) {
    /*! Check reset conditions for history info and if conditions fulfilled,
     * reset history info */
    /*! Local variables */
    uint8 i;
    boolean b_ResetDiffMarkerLCLeft, b_ResetDiffMarkerLCRight,
        b_RestHistoryInfo;
    float32 f_DistMarkerDiffLeftHistory, f_DistMarkerDiffRightHistory;
    float32 fSISeekLaneWidth = SIGetBaseSeekLaneWidth();

    /*! Set default: no reset of history info */
    b_RestHistoryInfo = FALSE;

    /*! Reset history info if lance change probability rises in the opposite
     * direction to the lane change history */
    if (((SILaneChangeBracket.LaneChgTBState == SI_TB_LANE_CHANGE_RIGHT) &&
         (i_SILaneChangeProb > SI_TB_RIGHT_LC_PROB_LEVEL_RESET_HIST)) ||
        ((SILaneChangeBracket.LaneChgTBState == SI_TB_LANE_CHANGE_LEFT) &&
         (i_SILaneChangeProb < SI_TB_LEFT_LC_PROB_LEVEL_RESET_HIST))) {
        b_RestHistoryInfo = TRUE;
    }

    /*! Evaluate the distance to the lane markers directly after the LC
       probability was high (lane markers just crossed)
        for resetting the adaption of trace brackets
        -> if the vehicle starts moving to the opposite direction compared to
       the lane change, the adaption of the
           trace brackets stops
       (SILaneChangeBracket.fMarkerDistLeft/RightHistory is reset)
        -> if the position in the middle of the lane is reached after a lane
       change, the adaption of the trace brackets
           is stopped (SILaneChangeBracket.fMarkerDistLeft/RightHistory is
       reset) */
    else if ((LaneChangeTBState == SI_NO_TB_LANE_CHANGE) &&
             (SILaneChangeBracket.LaneChgTBState != SI_NO_TB_LANE_CHANGE)) {
        /*! Determine if vehicle starts moving to the opposite direction
         * compared to the lane change: Evaluate the difference in the distance
         * to the lane markers within the history buffer */
        /*! Initialize decision according to difference in distance to lane
         * markers */
        b_ResetDiffMarkerLCLeft = FALSE;
        b_ResetDiffMarkerLCRight = FALSE;
        for (i = 0u; i < (SI_LANE_CHANGE_HISTORY_BUFFER - 1u); i++) {
            /*! Determine the difference in the distance to the lane markers
             * within the history buffer for both sides */
            f_DistMarkerDiffLeftHistory =
                SILaneChangeBracket.fMarkerDistLeftHistory[i] -
                SILaneChangeBracket.fMarkerDistLeftHistory[i + 1u];
            f_DistMarkerDiffRightHistory =
                SILaneChangeBracket.fMarkerDistRightHistory[i] -
                SILaneChangeBracket.fMarkerDistRightHistory[i + 1u];
            /*! If the lane change was to the left: the distance to the lane
             * markers decreases when driving to the left (when the marker is
             * crossed, the distance "jumps" to a higher value) */
            if (SILaneChangeBracket.LaneChgTBState == SI_TB_LANE_CHANGE_LEFT) {
                if ((f_DistMarkerDiffLeftHistory >
                     SI_LC_LEFT_DIFF_DIST_MARKER_HIST_LEVEL) &&
                    (f_DistMarkerDiffLeftHistory < (fSISeekLaneWidth * 0.5f))) {
                    b_ResetDiffMarkerLCLeft = TRUE;
                }
                if ((f_DistMarkerDiffRightHistory >
                     SI_LC_LEFT_DIFF_DIST_MARKER_HIST_LEVEL) &&
                    (f_DistMarkerDiffRightHistory <
                     (fSISeekLaneWidth * 0.5f))) {
                    b_ResetDiffMarkerLCRight = TRUE;
                }
            }
            /*! If the lane change was to the right: the distance to the lane
             * markers increases when driving to the right (when the marker is
             * crossed, the distance "jumps" to a lower value) */
            if (SILaneChangeBracket.LaneChgTBState == SI_TB_LANE_CHANGE_RIGHT) {
                if ((f_DistMarkerDiffLeftHistory <
                     SI_LC_RIGHT_DIFF_DIST_MARKER_HIST_LEVEL) &&
                    (f_DistMarkerDiffLeftHistory >
                     (-fSISeekLaneWidth * 0.5f))) {
                    b_ResetDiffMarkerLCLeft = TRUE;
                }
                if ((f_DistMarkerDiffRightHistory <
                     SI_LC_RIGHT_DIFF_DIST_MARKER_HIST_LEVEL) &&
                    (f_DistMarkerDiffRightHistory >
                     (-fSISeekLaneWidth * 0.5f))) {
                    b_ResetDiffMarkerLCRight = TRUE;
                }
            }
        }

        /*! Set reset condition:
          -> if the vehicle starts moving to the opposite direction compared to
          the lane change (indicated by distance to both lane markers)
          -> if the position in the middle of the lane is reached after a lane
          change */
        if ((b_ResetDiffMarkerLCLeft == TRUE) &&
            (b_ResetDiffMarkerLCRight == TRUE)) {
            b_RestHistoryInfo = TRUE;
        }
        /*! If the position in the middle of the lane is reached after a lane
           change, the adaption of the trace brackets is stopped */
        else if ((SILaneChangeBracket.bMarkerCrossed == TRUE) &&
                 (((SILaneChangeBracket.LaneChgTBState ==
                    SI_TB_LANE_CHANGE_LEFT) &&
                   (fABS(
                        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                            .f_MarkerDist) <
                    fABS(VLCSEN_pCamLaneData
                             ->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                             .f_MarkerDist))) ||
                  ((SILaneChangeBracket.LaneChgTBState ==
                    SI_TB_LANE_CHANGE_RIGHT) &&
                   (fABS(VLCSEN_pCamLaneData
                             ->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                             .f_MarkerDist) <
                    fABS(
                        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                            .f_MarkerDist))))) {
            b_RestHistoryInfo = TRUE;
        } else {
            /*! No reset */
        }
    } else {
        /*! No reset */
    }

    /*! Reset history info */
    if (b_RestHistoryInfo == TRUE) {
        SILaneChangeBracket.LaneChgTBState = SI_NO_TB_LANE_CHANGE;
        SILaneChangeBracket.bMarkerCrossed = FALSE;
    }
}

/*************************************************************************************************************************
  Functionname:    SI_v_AdaptTBForLCBasedOnLCProb */
static void SI_v_AdaptTBForLCBasedOnLCProb(
    float32 *pf_DistMarkerLeft,
    float32 *pf_DistMarkerRight,
    SIScaleBracketState_t *p_StateScaleBracket,
    sint16 const i_SILaneChangeProb,
    SILaneChangeTraceBracketState_t const LaneChangeTBState) {
    float32 fAbsDistCamLaneMarkerLeft, fAbsDistCamLaneMarkerRight;
    float32 fSISeekLaneWidth = SIGetBaseSeekLaneWidth();

    fAbsDistCamLaneMarkerLeft = fABS(
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT].f_MarkerDist);
    fAbsDistCamLaneMarkerRight = fABS(
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT].f_MarkerDist);

    /* QAC Fix */
    _PARAM_UNUSED(i_SILaneChangeProb);

    /*! Determining the distance of the ego-position to the lane marker in case
     * of a lane change to the left*/
    if ((LaneChangeTBState == SI_TB_LANE_CHANGE_LEFT) ||
        (SILaneChangeBracket.LaneChgTBState == SI_TB_LANE_CHANGE_LEFT)) {
        /*! Evaluate lane marker(s) to our left */
        if ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                 .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
            (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK)) {
            /*! Distinguish if the lane marker was already crossed */
            /*! Not crossed yet */
            if ((SILaneChangeBracket.bMarkerCrossed == FALSE) &&
                (((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                       .u_ExistanceProbability < FIP_CAM_LANE_POE_LEVEL) &&
                  (fAbsDistCamLaneMarkerLeft < SI_CP_PAR_LC_MAX_MARKER_DIST)) ||
                 ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                       .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
                  (fAbsDistCamLaneMarkerLeft < fAbsDistCamLaneMarkerRight)))) {
                *pf_DistMarkerRight = 0.f; /*!< Set to zero since lane change
                                              detected, i.e. new lane although
                                              camera says something different */
                /*! Second condition: plausibility */
                if ((VLCSEN_pCamLaneData
                         ->LaneMarkerInfo[CL_CAM_LANE_MK_ADJ_LEFT]
                         .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
                    (fABS(VLCSEN_pCamLaneData
                              ->LaneMarkerInfo[CL_CAM_LANE_MK_ADJ_LEFT]
                              .f_MarkerDist) > fAbsDistCamLaneMarkerLeft)) {
                    *pf_DistMarkerLeft =
                        fABS(VLCSEN_pCamLaneData
                                 ->LaneMarkerInfo[CL_CAM_LANE_MK_ADJ_LEFT]
                                 .f_MarkerDist);
                } else {
                    *pf_DistMarkerLeft = fSISeekLaneWidth;
                }
            } else /*! Marker was crossed */
            {
                *pf_DistMarkerLeft = fAbsDistCamLaneMarkerLeft;
                SILaneChangeBracket.bMarkerCrossed = TRUE;
            }
        }
        /*! Evaluate lane marker(s) to our right (only if lane marker was
         * crossed) */
        if ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                 .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
            (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK) &&
            (*pf_DistMarkerRight > SI_BRACKETPOS_VALID_VAL_COMPARE)) {
            *pf_DistMarkerRight = fAbsDistCamLaneMarkerRight;
            if (*pf_DistMarkerLeft > SI_BRACKETPOS_VALID_VAL_COMPARE) {
                *pf_DistMarkerLeft = fSISeekLaneWidth;
            }
            SILaneChangeBracket.bMarkerCrossed = TRUE;
        }
    }

    /*! Determining the distance of the ego-position to the lane marker in case
     * of a lane change to the right*/
    if ((LaneChangeTBState == SI_TB_LANE_CHANGE_RIGHT) ||
        (SILaneChangeBracket.LaneChgTBState == SI_TB_LANE_CHANGE_RIGHT)) {
        /*! Evaluate lane marker(s) to our right */
        if ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                 .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
            (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK)) {
            /*! Distinguish if the lane markers were already crossed */
            /*! Not crossed yet */
            if ((SILaneChangeBracket.bMarkerCrossed == FALSE) &&
                (((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                       .u_ExistanceProbability < FIP_CAM_LANE_POE_LEVEL) &&
                  (fAbsDistCamLaneMarkerRight <
                   SI_CP_PAR_LC_MAX_MARKER_DIST)) ||
                 ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                       .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
                  (fAbsDistCamLaneMarkerRight < fAbsDistCamLaneMarkerLeft)))) {
                *pf_DistMarkerLeft = 0.f; /*!< Set to zero since lane change
                                             detected, i.e. new lane although
                                             camera says something different */
                /*! Second condition: plausibility */
                if ((VLCSEN_pCamLaneData
                         ->LaneMarkerInfo[CL_CAM_LANE_MK_ADJ_RIGHT]
                         .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
                    (fABS(VLCSEN_pCamLaneData
                              ->LaneMarkerInfo[CL_CAM_LANE_MK_ADJ_RIGHT]
                              .f_MarkerDist) > fAbsDistCamLaneMarkerRight)) {
                    *pf_DistMarkerRight =
                        fABS(VLCSEN_pCamLaneData
                                 ->LaneMarkerInfo[CL_CAM_LANE_MK_ADJ_RIGHT]
                                 .f_MarkerDist);
                } else {
                    *pf_DistMarkerRight = fSISeekLaneWidth;
                }
            } else /*! Marker was crossed */
            {
                *pf_DistMarkerRight = fAbsDistCamLaneMarkerRight;
                SILaneChangeBracket.bMarkerCrossed = TRUE;
            }
        }
        /*! Evaluate lane marker(s) to our left (only if lane marker was
         * crossed) */
        if ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                 .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
            (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK) &&
            (*pf_DistMarkerLeft > SI_BRACKETPOS_VALID_VAL_COMPARE)) {
            *pf_DistMarkerLeft = fAbsDistCamLaneMarkerLeft;
            if (*pf_DistMarkerRight > SI_BRACKETPOS_VALID_VAL_COMPARE) {
                *pf_DistMarkerRight = fSISeekLaneWidth;
            }
            SILaneChangeBracket.bMarkerCrossed = TRUE;
        }
    }

    /* ! Set reason for trace bracket adaption */
    if ((*pf_DistMarkerLeft < SI_BRACKETPOS_VALID_VAL_COMPARE) &&
        (*pf_DistMarkerRight < SI_BRACKETPOS_VALID_VAL_COMPARE)) {
        if (*pf_DistMarkerLeft >= *pf_DistMarkerRight) {
            *p_StateScaleBracket = POST_LANE_CHANGE_LEFT;
        } else {
            *p_StateScaleBracket = POST_LANE_CHANGE_RIGHT;
        }

        *pf_DistMarkerRight = INITVALUE_BRACKETPOSITION;
        *pf_DistMarkerLeft = INITVALUE_BRACKETPOSITION;
    }

    _PARAM_UNUSED(i_SILaneChangeProb);
}

/*************************************************************************************************************************
  Functionname:    SI_v_AdaptTBIfNoLC */
static void SI_v_AdaptTBIfNoLC(float32 *const pf_DistMarkerLeft,
                               float32 *const pf_DistMarkerRight,
                               SIScaleBracketState_t *const p_StateScaleBracket,
                               boolean const b_LCProbValid,
                               sint16 const i_SILaneChangeProb) {
    float32 fAbsDistCamLaneMarkerLeft, fAbsDistCamLaneMarkerRight;

    fAbsDistCamLaneMarkerLeft = fABS(
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT].f_MarkerDist);
    fAbsDistCamLaneMarkerRight = fABS(
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT].f_MarkerDist);

    if ((b_LCProbValid == TRUE) &&
        (SILaneChangeBracket.LaneChgTBState == SI_NO_TB_LANE_CHANGE) &&
        (ABS(i_SILaneChangeProb) < (sint16)SI_LC_PROB_LEVEL_LOW) &&
        (EGO_SPEED_X_OBJ_SYNC > SI_LEVEL_LOW_SPEED) &&
        (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
        (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
        (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK) &&
        ((fAbsDistCamLaneMarkerLeft + fAbsDistCamLaneMarkerRight) >
         SI_TB_RATIO_MIN_CAM_LANE_WIDTH) &&
        ((fAbsDistCamLaneMarkerLeft + fAbsDistCamLaneMarkerRight) <
         SI_TB_RATIO_MAX_CAM_LANE_WIDTH) &&
        (fAbsDistCamLaneMarkerLeft >
         MIN_FLOAT(
             SI_MIN_DIST_MARKER_INLANE,
             SI_MIN_FACTOR_DIST_MARKER_INLANE *
                 (fAbsDistCamLaneMarkerLeft + fAbsDistCamLaneMarkerRight))) &&
        (fAbsDistCamLaneMarkerRight >
         MIN_FLOAT(
             SI_MIN_DIST_MARKER_INLANE,
             SI_MIN_FACTOR_DIST_MARKER_INLANE *
                 (fAbsDistCamLaneMarkerLeft + fAbsDistCamLaneMarkerRight)))) {
        /*! If bmw blinker feature is on or if the blinker is switched on, no
         * adaption */
        if (VLCSEN_pCustomInput->eTurnIndicator == eTurnIndicator_Off) {
            *pf_DistMarkerLeft = fAbsDistCamLaneMarkerLeft;
            *pf_DistMarkerRight = fAbsDistCamLaneMarkerRight;

            /* ! Set reason for trace bracket adaption */
            *p_StateScaleBracket = NO_LANE_CHANGE;
        } else {
            /*! nothing */
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SI_v_AdaptTBForLCBasedOnBlinkerFeature */
static void SI_v_AdaptTBForLCBasedOnBlinkerFeature(
    float32 *const pf_DistMarkerLeft,
    float32 *const pf_DistMarkerRight,
    SIScaleBracketState_t *const p_StateScaleBracket,
    sint16 const i_SILaneChangeProb,
    boolean const b_ValidLCProb) {
    boolean b_ResetBlinkerFeatureTriggered;
    uint8 i;
    float32 f_DistMarkerDiffBlinkerFeatureHistory;
    float32 fAbsDistCamLaneMarkerLeft, fAbsDistCamLaneMarkerRight;
    float32 fSISeekLaneWidth = SIGetBaseSeekLaneWidth();

    fAbsDistCamLaneMarkerLeft = fABS(
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT].f_MarkerDist);
    fAbsDistCamLaneMarkerRight = fABS(
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT].f_MarkerDist);

    /*! Default: not reset of trace bracket adaption based on the blinker
     * feature */
    b_ResetBlinkerFeatureTriggered = FALSE;

    if ((b_ValidLCProb == TRUE) && (*p_StateScaleBracket == UNKNOWN) &&
        (EGO_SPEED_X_OBJ_SYNC > SI_LEVEL_LOW_SPEED)) {
        /*! Adapt trace bracket based on blinker feature,
            if bmw blinker feature is active or if the turn indicator is active
           */

        if ((VLCSEN_pCustomInput->eTurnIndicator == eTurnIndicator_Left) ||
            (VLCSEN_pCustomInput->eTurnIndicator == eTurnIndicator_Right)) {
            SILaneChangeBracket.bBlinkerFeatureTriggered = TRUE;
        } else {
            /*! Abort trace bracket adaption based on blinker feature (after the
               blinker feature is not active anymore)
                if the ego vehicle starts driving in the opposite direction to
               the lane change */
            if (SILaneChangeBracket.bBlinkerFeatureTriggered) {
                for (i = 0u; (i < (SI_LANE_CHANGE_HISTORY_BUFFER - 1u)) &&
                             (b_ResetBlinkerFeatureTriggered == FALSE);
                     i++) {
                    if (i_SILaneChangeProb > 0) {
                        f_DistMarkerDiffBlinkerFeatureHistory =
                            SILaneChangeBracket.fMarkerDistLeftHistory[i] -
                            SILaneChangeBracket.fMarkerDistLeftHistory[i + 1u];
                        if ((f_DistMarkerDiffBlinkerFeatureHistory >
                             SI_LC_LEFT_DIFF_DIST_MARKER_HIST_LEVEL) &&
                            (f_DistMarkerDiffBlinkerFeatureHistory <
                             (fSISeekLaneWidth * 0.5f))) {
                            b_ResetBlinkerFeatureTriggered = TRUE;
                        }
                    } else {
                        f_DistMarkerDiffBlinkerFeatureHistory =
                            SILaneChangeBracket.fMarkerDistRightHistory[i] -
                            SILaneChangeBracket.fMarkerDistRightHistory[i + 1u];
                        if ((f_DistMarkerDiffBlinkerFeatureHistory <
                             SI_LC_RIGHT_DIFF_DIST_MARKER_HIST_LEVEL) &&
                            (f_DistMarkerDiffBlinkerFeatureHistory >
                             (-fSISeekLaneWidth * 0.5f))) {
                            b_ResetBlinkerFeatureTriggered = TRUE;
                        }
                    }
                }
            }
        }

        /*! Reset conditions independed current blinker feature status (blinker
         * feature may/may not be active in this cycle) */
        if (SILaneChangeBracket.bBlinkerFeatureTriggered) {
            if ((((EGO_SPEED_X_OBJ_SYNC <= SI_LEVEL_LOW_SPEED) &&
                  (ABS(i_SILaneChangeProb) >
                   (sint16)SI_LC_PROB_LEVEL_LOW_SPEED)) ||
                 ((EGO_SPEED_X_OBJ_SYNC > SI_LEVEL_LOW_SPEED) &&
                  (ABS(i_SILaneChangeProb) >
                   (sint16)SI_LC_PROB_LEVEL_HIGH_SPEED)))) {
                b_ResetBlinkerFeatureTriggered = TRUE;
            }
            /* Abort trace bracket adaption based on blinker feature if lane
               change probability is too low */
            else if (ABS(i_SILaneChangeProb) <
                     SI_TB_ABORT_BLINKER_FEATURE_LC_PROB_LEVEL) {
                b_ResetBlinkerFeatureTriggered = TRUE;
            } else {
                /*! No reset */
            }

            /*! Reset blinker feature trigger */
            if (b_ResetBlinkerFeatureTriggered) {
                SILaneChangeBracket.bBlinkerFeatureTriggered = FALSE;
            }
        }

        /* Adapt trace brackets if blinker feature is active and the camera lane
         * marker state is valid */
        if ((SILaneChangeBracket.bBlinkerFeatureTriggered) &&
            (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                 .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
            (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                 .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
            (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK) &&
            ((fAbsDistCamLaneMarkerLeft + fAbsDistCamLaneMarkerRight) >
             SI_TB_RATIO_MIN_CAM_LANE_WIDTH) &&
            ((fAbsDistCamLaneMarkerLeft + fAbsDistCamLaneMarkerRight) <
             SI_TB_RATIO_MAX_CAM_LANE_WIDTH)) {
            if (i_SILaneChangeProb >
                0) /*!< In case of a lane change to the left */
            {
                /* Set distance to lane marker in order to adapt the trace
                   brackets;
                   move trace brackets in the direction of the lane change */
                *pf_DistMarkerLeft =
                    (fAbsDistCamLaneMarkerLeft + fAbsDistCamLaneMarkerRight) *
                    SI_TB_INCREASE_FACTOR_BLINKER_FEATURE;
                *pf_DistMarkerRight =
                    (fAbsDistCamLaneMarkerLeft + fAbsDistCamLaneMarkerRight) *
                    SI_TB_DECREASE_FACTOR_BLINKER_FEATURE;

                /*! Set reason for trace bracket adaption */
                *p_StateScaleBracket = PRE_LANE_CHANGE_LEFT;
            } else /*!< In case of a lane change to the right */
            {
                /* Set distance to lane marker in order to adapt the trace
                   brackets;
                   move trace brackets in the direction of the lane change */
                *pf_DistMarkerRight =
                    (fAbsDistCamLaneMarkerLeft + fAbsDistCamLaneMarkerRight) *
                    SI_TB_INCREASE_FACTOR_BLINKER_FEATURE;
                *pf_DistMarkerLeft =
                    (fAbsDistCamLaneMarkerLeft + fAbsDistCamLaneMarkerRight) *
                    SI_TB_DECREASE_FACTOR_BLINKER_FEATURE;
                /*! Set reason for trace bracket adaption */
                *p_StateScaleBracket = PRE_LANE_CHANGE_RIGHT;
            }
        }
    } else {
        if ((b_ValidLCProb == FALSE) ||
            (EGO_SPEED_X_OBJ_SYNC < SI_LEVEL_LOW_SPEED) ||
            (*p_StateScaleBracket == POST_LANE_CHANGE_LEFT) ||
            (*p_StateScaleBracket == POST_LANE_CHANGE_RIGHT)) {
            SILaneChangeBracket.bBlinkerFeatureTriggered = FALSE;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIReturnStateScaleBracket */
SIScaleBracketState_t SIReturnStateScaleBracket(void) {
    return SILaneChangeState;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
