/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "si.h"
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

/* ExtensionCamLaneObjAssoc: */
#define SI_MAX_VREL_CAM_LANE_OBJ_ASSOC_EXT (-5.f)
#define SI_MAX_EGO_VEL_CAM_LANE_OBJ_ASSOC_EXT (70.f / C_KMH_MS)
#define SI_EXT_TRACK_CAM_LANE_OBJ_ASSOC_EXT (1.f)
#define SI_EXT_SEEK_CAM_LANE_OBJ_ASSOC_EXT (0.5f)

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

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
/*! Functions related to trace bracket adaption based on the ego position in
 * lane */

static void SI_v_AdaptTBBasedOnPerformedLC(
    RelTraObjInput_t const *pObj,
    AssTraEnvironment_t const *pEnvironment,
    const SIScaleBracketOutput_t *pRatioEgoPosInLine,
    float32 fDistY,
    float32 fDist2Ref,
    boolean *pReturnValue);

/*************************************************************************************************************************
  Functionname:    SIFindObjRelevantForTraceBracketAdaptPosInLaneCam */
boolean SIFindObjRelevantForTraceBracketAdaptPosInLaneCam(
    RelTraObjInput_t const *pObj,
    AssTraEnvironment_t const *pEnvironment,
    const SIScaleBracketOutput_t *pRatioEgoPosInLine) {
    /*! Declaration of variables */
    boolean returnValue;
    boolean b_CamLaneLeftIsRoadEdge, b_CamLaneRightIsRoadEdge;
    sint32 iNumLanes;
    float32 fCamLaneSin, fObjPolarDist, fCamLaneCOS, fX, yCompensateCam,
        fDistYCurve, yCompensateSITraj, fDistY, fCompensateEgoPos,
        fDistYCompensate, fDist2Ref, fDist2RefCompensate;
    float32 f_CamLaneVisibilityDist, f_CamLaneC0, f_CamLaneAngle;
    SIInLaneDecision_t dec_lane;
    float32 fSISeekLaneWidth = SIGetBaseSeekLaneWidth();

    /*! Set default factor value */
    dec_lane = UNKNOWN_LANE; /*!< Initialization of the variable for assigning
                                the object to the ego-lane */
    iNumLanes = 0;           /*!< Initialization of the number of lanes */
    returnValue = FALSE;     /*!< Initialization of the return value */

    /*! If the factors for the trace bracket adaption are determined, the object
    is assigned elementarily the the ego-/left/right lane.
    An adaption of the trace brackets is performed depending on this assignment.
    */
    f_CamLaneVisibilityDist =
        FIP_f_GetVisibilityDistCamLane();  /*!< Visibility distance of camera
                                              lane */
    f_CamLaneC0 = FIP_f_GetCurveCamLane(); /*!< Curvature of camera lane */
    f_CamLaneAngle =
        FIP_f_GetHeadingAngleCamLane(); /*!< Heading angle of camera lane */
    if ((pRatioEgoPosInLine->fScaleBracketRight <
         SI_BRACKETPOS_VALID_VAL_COMPARE) &&
        (pRatioEgoPosInLine->fScaleBracketLeft <
         SI_BRACKETPOS_VALID_VAL_COMPARE)) {
        /*! Calculation of lateral distance to the object; thereby, the lateral
         * offset due to the curvature of the lane is considered */
        fCamLaneSin = SIN_(-f_CamLaneAngle + DEG2RAD(pObj->fOrientation));
        fObjPolarDist =
            SQRT(SQR(pObj->fDistY) +
                 SQR(pObj->fDistX)); /*!< polar distance to the object */
        fCamLaneCOS = COS_(-f_CamLaneAngle + DEG2RAD(pObj->fOrientation));
        fX = fCamLaneCOS *
             fObjPolarDist; /*!< longitudinal distance to the object */
        yCompensateCam =
            0.5f * f_CamLaneC0 *
            SQR(fX); /*!< lateral offset due to camera lane curvature */

        /*! If the visibility of the camera lane is lower than the longitudinal
        distance to the object,
        the mean of the offset of the lateral displacement based on the camera
        lane and the ACC-trajectory is used */
        if (f_CamLaneVisibilityDist > pObj->fDistX) {
            /*! Only the curvature of the camera lane is considered */
            yCompensateSITraj = yCompensateCam;
        } else {
            /*! Determining the offset of the lateral displacement of the object
             * based on the ACC-trajectory */
            yCompensateSITraj =
                (C_SIXTH * SITrajectoryData.Current.fTrajC1 *
                 SQR(pObj->fDistX) * pObj->fDistX) +
                (0.5f * SITrajectoryData.Current.fTrajC0 * SQR(pObj->fDistX));
        }
        /*! Calculation of the mean */
        fDistYCurve = 0.5f * (yCompensateCam + yCompensateSITraj);

        /*! Determing the lateral distance to the object; thereby the curvature
         * of the lane is considered */
        fDistY = (fCamLaneSin * fObjPolarDist) - fDistYCurve;

        /*! Compensate lateral distance to object based on the ego-position in
         * lane */
        fCompensateEgoPos =
            fSISeekLaneWidth * (pRatioEgoPosInLine->fScaleBracketRight - 1.f);
        fDistYCompensate = fDistY + fCompensateEgoPos;

        /*! Determine the distance of the object to the ACC-trajectory and
         * compensate this distance based on the ego-position in lane */
        fDist2Ref = OBJ_GET_OBJ_TO_REF_DISTANCE(pObj->iObjNr);
        fDist2RefCompensate = fDist2Ref + fCompensateEgoPos;

        /* If the trace brackets are supposed to be adapted due to driving in
        the ego lane (no lane change);
        condition for adaption of the trace bracket: the difference between the
        ego curve and the curvature
        of the camera lane is not allowed to be too high (otherwise the camera
        lane cannot be trusted) */

        if ((pRatioEgoPosInLine->StateScaleBracket == NO_LANE_CHANGE) &&
            (fABS(f_CamLaneC0 - (float32)EGO_CURVE_RAW) <
             SI_LEVEL_CURVE_DIFF_CAM_EGO)) {
            /*! A elementary assignment of the object to the ego/left/right lane
             * is performed */

            /*! Distinction if we are driving on a curvy road or not */
            /*! Driving on a curvy lane (or the difference between the lane
             * curvature and the ego-course is high) */
            if ((fABS(f_CamLaneC0) > SI_TB_LANE_ASSO_CAM_COURSE_LEVEL) ||
                (fABS(f_CamLaneC0 - (float32)EGO_CURVE_RAW) >
                 SI_TB_LANE_ASSO_CAM_COURSE_EGO_COURSE_DIFF_LEVEL)) {
                /*! Determining the number of lanes in the direction if the
                 * considered object */
                /*! Object is to the left */
                if (fDistY > 0.f) {
                    iNumLanes = pEnvironment->iNumberLanesLeft;
                } else /*! Object is to the right */
                {
                    iNumLanes = pEnvironment->iNumberLanesRight;
                }

                /*! Distinguish for lane association if there is no other lane
                 * in the direction of the object or not */
                /*! No other lane: If there is no other lane in the direction of
                the object
                   (i.e. there is a continuous lane in the direction of the
                object but no continuous lanes on both sides of the ego
                vehicle),
                it is very likely that the object is on the ego lane (or the
                lateral distance to the object is very high and it might be a
                mirror object) */
                /*! Get info if camera lane markers is road border */
                b_CamLaneLeftIsRoadEdge =
                    FIP_b_GetIsCamLaneRoadEdge(CL_CAM_LANE_MK_LEFT);
                b_CamLaneRightIsRoadEdge =
                    FIP_b_GetIsCamLaneRoadEdge(CL_CAM_LANE_MK_RIGHT);
                if ((iNumLanes < 2) && /*!< iNumLanes = 1 is considered as "no
                                          lane" since there is often the
                                          emergency lane on the highway */
                    ((b_CamLaneRightIsRoadEdge == FALSE) ||
                     (b_CamLaneLeftIsRoadEdge == FALSE)) &&
                    (((fDistY < 0.f) &&
                      (fDistY >
                       -(2.f * fABS(VLCSEN_pCamLaneData
                                        ->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                                        .f_MarkerDist))) &&
                      (b_CamLaneRightIsRoadEdge == TRUE)) ||
                     ((fDistY > 0.f) &&
                      (fDistY <
                       (2.f * fABS(VLCSEN_pCamLaneData
                                       ->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                                       .f_MarkerDist))) &&
                      (b_CamLaneLeftIsRoadEdge == TRUE)))) {
                    dec_lane = EGO_LANE;
                } else /*! Other parameters for lane association if there is at
                          least one next lane in the direction of the object */
                {
                    if ((fDistYCompensate <
                         -(fSISeekLaneWidth *
                           SI_TB_ASSO_LAT_DIST_COMP_PARA_MULTI_LANE)) ||
                        (fDistY < -(fSISeekLaneWidth *
                                    SI_TB_ASSO_LAT_DIST_PARA_MULTI_LANE)) ||
                        (fDist2RefCompensate < -fSISeekLaneWidth)) {
                        dec_lane = RIGHT_LANE;
                    } else {
                        if ((fDistYCompensate >
                             (fSISeekLaneWidth *
                              SI_TB_ASSO_LAT_DIST_COMP_PARA_MULTI_LANE)) ||
                            (fDistY > (fSISeekLaneWidth *
                                       SI_TB_ASSO_LAT_DIST_PARA_MULTI_LANE)) ||
                            (fDist2RefCompensate > fSISeekLaneWidth)) {
                            dec_lane = LEFT_LANE;
                        } else {
                            dec_lane = EGO_LANE;
                        }
                    }
                }
            } else /*! Driving on a straight lane: other parameters for lane
                      association */
            {
                if ((fDistYCompensate <
                     -(fSISeekLaneWidth *
                       SI_TB_ASSO_LAT_DIST_COMP_PARA_STRAIGHT_LANE)) ||
                    (fDistY < -(fSISeekLaneWidth *
                                SI_TB_ASSO_LAT_DIST_PARA_STRAIGHT_LANE)) ||
                    (fDist2RefCompensate < -fSISeekLaneWidth)) {
                    dec_lane = RIGHT_LANE;
                } else {
                    if ((fDistYCompensate >
                         (fSISeekLaneWidth *
                          SI_TB_ASSO_LAT_DIST_COMP_PARA_STRAIGHT_LANE)) ||
                        (fDistY > (fSISeekLaneWidth *
                                   SI_TB_ASSO_LAT_DIST_PARA_STRAIGHT_LANE)) ||
                        (fDist2RefCompensate > fSISeekLaneWidth)) {
                        dec_lane = LEFT_LANE;
                    } else {
                        dec_lane = EGO_LANE;
                    }
                }
            }

            /*! object on ego-lane -> adapt trace brackets only if the object
             * can be pick-uped easier, object on the left or right side of ego
             * vehicle */
            if (dec_lane == EGO_LANE &&
                ((fDist2Ref < 0.f) &&
                     (pRatioEgoPosInLine->fScaleBracketRight > 1.f) ||
                 (fDist2Ref > 0.f) &&
                     (pRatioEgoPosInLine->fScaleBracketRight < 1.f))) {
                returnValue = TRUE;
            }

            /*! object on the left or right lane of ego vehicle -> adapt trace
             * brackets only if it becomes more difficult to pick-up the object
             */
            if ((dec_lane == RIGHT_LANE) &&
                    (pRatioEgoPosInLine->fScaleBracketRight < 1.f) ||
                (dec_lane == LEFT_LANE) &&
                    (pRatioEgoPosInLine->fScaleBracketRight > 1.f)) {
                returnValue = TRUE;
            }
        }

        /*! If the trace brackets are supposed to be adapted due to a performed
        lane change:
        Check if the object is on the second next lane (than no adaption of the
        trace brackets is performed);
        Check if the object performs a lane change at the same time as the
        ego-vehicle (than no adaption of the trace brackets is performed)
        Check if the object is inlane but deselected due to the instable yaw
        rate in a lane change situation (than no adaption of the trace brackets
        is performed)*/
        SI_v_AdaptTBBasedOnPerformedLC(pObj, pEnvironment, pRatioEgoPosInLine,
                                       fDistY, fDist2Ref, &returnValue);
    }

    /*! If the object was classified as oncoming, no adaption should be
     * performed */
    if (OBJ_GET_SI(pObj->iObjNr).Bool.Oncoming == 1u) {
        returnValue = FALSE;
    }

    return returnValue;
}

/*************************************************************************************************************************
  Functionname:    SI_v_AdaptTBBasedOnPerformedLC */
static void SI_v_AdaptTBBasedOnPerformedLC(
    RelTraObjInput_t const *pObj,
    AssTraEnvironment_t const *pEnvironment,
    const SIScaleBracketOutput_t *pRatioEgoPosInLine,
    float32 fDistY,
    float32 fDist2Ref,
    boolean *pReturnValue) {
    sint32 iNumLanes;
    float32 fSISeekLaneWidth = SIGetBaseSeekLaneWidth();

    if ((pRatioEgoPosInLine->StateScaleBracket == POST_LANE_CHANGE_LEFT) ||
        (pRatioEgoPosInLine->StateScaleBracket == POST_LANE_CHANGE_RIGHT) ||
        (pRatioEgoPosInLine->StateScaleBracket == PRE_LANE_CHANGE_LEFT) ||
        (pRatioEgoPosInLine->StateScaleBracket == PRE_LANE_CHANGE_RIGHT)) {
        float32 fObjLatVel, fObjLatDisp;

        /*! Lane change to the left */
        if ((pRatioEgoPosInLine->StateScaleBracket == POST_LANE_CHANGE_LEFT) ||
            (pRatioEgoPosInLine->StateScaleBracket == PRE_LANE_CHANGE_LEFT)) {
            iNumLanes = pEnvironment->iNumberLanesLeft;
            fObjLatVel = OBJ_LAT_VREL(pObj->iObjNr);
            fObjLatDisp = OBJ_LAT_DISPLACEMENT(pObj->iObjNr);
        } else /*! Lane change to the right */
        {
            iNumLanes = pEnvironment->iNumberLanesRight;
            fDistY = -fDistY;
            fDist2Ref = -fDist2Ref;
            fObjLatVel = -OBJ_LAT_VREL(pObj->iObjNr);
            fObjLatDisp = -OBJ_LAT_DISPLACEMENT(pObj->iObjNr);
        }

        /*! Object is not on the second next lane */
        if ((iNumLanes < 2) ||
            (fDistY <=
             (fSISeekLaneWidth * SI_FAC_LANEWIDTH_OBJECT_SELECTION))) {
            if (((fObjLatVel > SI_TB_ASSO_LC_LAT_VREL_PARA) &&
                 (OBJ_LONG_VREL(pObj->iObjNr) > SI_TB_ASSO_LC_LONG_VREL_PARA) &&
                 ((fObjLatDisp > SI_TB_ASSO_LC_LAT_DIST_MIN_PARA) ||
                  (fDistY > SI_TB_ASSO_LC_COURSE_LAT_DIST_MIN_PARA)) &&
                 (fDist2Ref < SI_TB_ASSO_LC_LAT_DIST_MAX_AVLC_LANE_PARA) &&
                 (fDist2Ref > SI_TB_ASSO_LC_LAT_DIST_MIN_AVLC_LANE_PARA) &&
                 (OBJ_LONG_DISPLACEMENT(pObj->iObjNr) <
                  SI_TB_ASSO_LC_LONG_DIST_MAX_PARALL) &&
                 ((pRatioEgoPosInLine->StateScaleBracket ==
                   POST_LANE_CHANGE_LEFT) ||
                  (pRatioEgoPosInLine->StateScaleBracket ==
                   POST_LANE_CHANGE_RIGHT))) /*!< Parallel lane change of
                                                ego-vehicle and object */
                || (((pRatioEgoPosInLine->StateScaleBracket ==
                      POST_LANE_CHANGE_LEFT) ||
                     (pRatioEgoPosInLine->StateScaleBracket ==
                      POST_LANE_CHANGE_RIGHT)) &&
                    (OBJ_GET_SI(pObj->iObjNr).ObjCor.fRelevantTime >
                     SI_TB_ASSO_POST_LC_TIME_RELEVANT_PARA_MIN) &&
                    (OBJ_GET_SI(pObj->iObjNr).ObjCor.fRelevantTime <
                     SI_TB_ASSO_LC_TIME_RELEVANT_PARA_MAX) &&
                    (fDistY > SI_TB_ASSO_POST_LC_LAT_DIST_CAM_LANE_PARA) &&
                    (fDist2Ref < SI_TB_ASSO_LC_LAT_DIST_MAX_AVLC_LANE_PARA)) ||
                (((pRatioEgoPosInLine->StateScaleBracket ==
                   PRE_LANE_CHANGE_LEFT) ||
                  (pRatioEgoPosInLine->StateScaleBracket ==
                   PRE_LANE_CHANGE_RIGHT)) &&
                 (OBJ_GET_SI(pObj->iObjNr).ObjCor.fRelevantTime <
                  SI_TB_ASSO_LC_TIME_RELEVANT_PARA_MAX) &&
                 (((OBJ_GET_SI(pObj->iObjNr).ObjCor.fRelevantTime >
                    SI_TB_ASSO_PRE_LC_MIN_TIME_RELEVANT_PARA) &&
                   (fDistY > SI_TB_ASSO_PRE_LC_LAT_DIST_CAM_LANE_PARA)) ||
                  ((OBJ_GET_SI(pObj->iObjNr).ObjCor.fRelevantTime >
                    SI_TB_ASSO_PRE_LC_MIN_MIN_TIME_RELEVANT_PARA) &&
                   (fDistY > SI_TB_ASSO_PRE_LC_LAT_DIST_CAM_LANE_PARA) &&
                   (fObjLatDisp > SI_TB_ASSO_PRE_LC_LAT_DIST_PARA))) &&
                 (fDist2Ref < SI_TB_ASSO_LC_LAT_DIST_MAX_AVLC_LANE_PARA))) {
                /* do nothing */
            } else {
                *pReturnValue = TRUE;
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraSetTrackWidthScale */
void SIRelTraSetTrackWidthScale(const float32 fYPosCenterBracket,
                                float32 *pTrackWidthLeft,
                                float32 *pTrackWidthRight,
                                const SIScaleBracketOutput_t *pScaleBracket) {
    float32 fTrackWidthRightCenter, fTrackWidthLeftCenter;
    float32 fTrackWidthRightScale, fTrackWidthLeftScale;
    float32 fBrackedWidth;

    /*Adjusting the trace brackets only if the TrackWidth
      (pTrackWidthRight,pTrackWidthLeft) and
      the ratio for moving the center (BracketPositionLeft,
      BracketPositionRight) were already determined */
    if ((pScaleBracket->fScaleBracketLeft < SI_BRACKETPOS_VALID_VAL_COMPARE) &&
        (pScaleBracket->fScaleBracketRight < SI_BRACKETPOS_VALID_VAL_COMPARE) &&
        (fABS(*pTrackWidthRight) < SI_BRACKETPOS_VALID_VAL_COMPARE) &&
        (fABS(*pTrackWidthLeft) < SI_BRACKETPOS_VALID_VAL_COMPARE) &&
        ((pScaleBracket->fScaleBracketLeft > C_F32_DELTA) ||
         (pScaleBracket->fScaleBracketRight > C_F32_DELTA))) {
        /* Moving the center of the trace bracket to y = 0*/
        fTrackWidthRightCenter = (*pTrackWidthRight) - fYPosCenterBracket;
        fTrackWidthLeftCenter = (*pTrackWidthLeft) - fYPosCenterBracket;

        /* Adjusting the trace brackets according to factor and moving to former
         * lateral position */
        fBrackedWidth =
            fABS(fTrackWidthRightCenter) + fABS(fTrackWidthLeftCenter);
        /* If the left trace bracket is longer */
        if (fABS(fTrackWidthRightCenter) <= fABS(fTrackWidthLeftCenter)) {
            fTrackWidthRightScale =
                (fTrackWidthRightCenter * pScaleBracket->fScaleBracketRight) +
                fYPosCenterBracket;
            fTrackWidthLeftScale =
                (fBrackedWidth - fABS(fTrackWidthRightCenter *
                                      pScaleBracket->fScaleBracketRight)) +
                fYPosCenterBracket;
        } else /* If the right trace bracket is longer */
        {
            fTrackWidthLeftScale =
                (fTrackWidthLeftCenter * pScaleBracket->fScaleBracketLeft) +
                fYPosCenterBracket;
            fTrackWidthRightScale =
                -(fBrackedWidth - fABS(fTrackWidthLeftCenter *
                                       pScaleBracket->fScaleBracketLeft)) +
                fYPosCenterBracket;
        }

        /* Setting the new trace brackets to variable */
        *pTrackWidthRight = fTrackWidthRightScale;
        *pTrackWidthLeft = fTrackWidthLeftScale;
    }
}

/*************************************************************************************************************************
  Functionname:    SIAdaptRatioEgoPosInLaneCamToObj */
SIScaleBracketOutput_t SIAdaptRatioEgoPosInLaneCamToObj(
    RelTraObjInput_t const *pObjInput,
    SIScaleBracketOutput_t const ScaleBracketIn,
    AssTraEnvironment_t const *pEnvironment) {
    float32 fXMax, fRatio;
    SIScaleBracketOutput_t ScaleBracketOut;

    ScaleBracketOut.fScaleBracketLeft = ScaleBracketIn.fScaleBracketLeft;
    ScaleBracketOut.fScaleBracketRight = ScaleBracketIn.fScaleBracketRight;
    ScaleBracketOut.StateScaleBracket = ScaleBracketIn.StateScaleBracket;

    /*! Reduce the amount of moving the trace brackets only in case of no lane
     * change */
    if ((ScaleBracketIn.fScaleBracketLeft < SI_BRACKETPOS_VALID_VAL_COMPARE) &&
        (ScaleBracketIn.fScaleBracketRight < SI_BRACKETPOS_VALID_VAL_COMPARE) &&
        (ScaleBracketIn.StateScaleBracket == NO_LANE_CHANGE)) {
        /*! Don't reduce the amount of moving the trace brackets if there is no
        other lane in the direction of moving the trace brackets,
        hence, the risk of a drop-in is very small and the drop-out/pick-up
        performance increases */
        if (((ScaleBracketIn.fScaleBracketLeft >
              ScaleBracketIn.fScaleBracketRight) &&
             (pEnvironment->iNumberLanesLeft <= 0)) ||
            ((ScaleBracketIn.fScaleBracketRight >
              ScaleBracketIn.fScaleBracketLeft) &&
             (pEnvironment->iNumberLanesRight <= 0))) {
            /*! nothing */
        } else {
            /*! Reduce the amount of moving the trace brackets only if the
            longitudinal distance to the object
            is higher than SI_TB_ADAPT_FACTOR_LONG_DIST */
            if (OBJ_LONG_DISPLACEMENT(pObjInput->iObjNr) >
                SI_TB_ADAPT_FACTOR_LONG_DIST) {
                /* Above the maximum longitudinal distance, the trace brackets
                are not moved at all (factor is 1);
                the maximum longitudinal distance equals the BMW bick-up
                distance (2.2s * V_EGO + 40m) */
                fXMax = SIGetMovingObjBasePickupRange();
                if (OBJ_LONG_DISPLACEMENT(pObjInput->iObjNr) < fXMax) {
                    if ((fXMax - SI_TB_ADAPT_FACTOR_LONG_DIST) > 0.f) {
                        fRatio = (OBJ_LONG_DISPLACEMENT(pObjInput->iObjNr) -
                                  SI_TB_ADAPT_FACTOR_LONG_DIST) /
                                 (fXMax - SI_TB_ADAPT_FACTOR_LONG_DIST);

                        ScaleBracketOut.fScaleBracketLeft +=
                            fRatio * (1.f - ScaleBracketIn.fScaleBracketLeft);
                        ScaleBracketOut.fScaleBracketRight +=
                            fRatio * (1.f - ScaleBracketIn.fScaleBracketRight);
                    }
                } else {
                    ScaleBracketOut.fScaleBracketLeft = 1.f;
                    ScaleBracketOut.fScaleBracketRight = 1.f;
                }
            }
        }
    }

    return ScaleBracketOut;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */