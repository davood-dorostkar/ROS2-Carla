/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
/* EMP Includes */
#include "TM_Global_Types.h"
#include "TM_Math_Cal.h"
#include "tue_common_libs.h"
#include "emp_calculation.h"

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! @brief Definition of float max
          @attention  TODO : Check if not defined elsewhere
 */
#define FLOAT_MAX (3.402823466e+38f)

/*! @brief Definition of Log based 10
 */
#define LOG_10_OF_E (0.4342944819032f)

/*****************************************************************************
  MACROS
*****************************************************************************/
#ifndef _PARAM_UNUSED
#if (defined(_MSC_VER))
#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): warning TODO: remove local macro definition if include strategy in emp is cleaned up.")
#endif
#define _PARAM_UNUSED(x) (void)(x)
#endif

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  LOCAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  LOCAL FUNCTIONS
*****************************************************************************/
static float32 EMPEvalPolyDeg2(const EMPPolyDeg2_t *pInPolynom,
                               float32 fSamplePoint);
static float32 EMPCalcTrajOrientationAtTime(float32 fTime,
                                            const EMPTrajPred_t *pInTraj);
static boolean EMPCalcVariance2D(float32 fTime,
                                 const EMPTrajPred_t *pInTraj,
                                 EMPVar2D_t *pOut_Var2D);
static float32 EMPCalcStdGaussianCDF2D(float32 fValueX,
                                       float32 fValueY,
                                       float32 fAver,
                                       float32 fSigmaX,
                                       float32 fSigmaY);
static float32 EMPCalcCubeRoot(float32 fX);

/* **************************************************************************** 

  Functionname:     EMPCalcMinDistTime                                   */ /*!

       @brief          Calculate time of minimal distance between the two
     objects
       @description    Calculate time of minimal distance between the two
     objects

       @param[in]      pIn_Obj1 Pointer of first object's tragectory
       @param[in]      pIn_Obj2 Pointer of second object's tragectory
       @param[out]     pOut_MinDistTime time of minimal distance between the two
     objects

       @return         True if no error occurred


     ****************************************************************************
     */
boolean EMPCalcMinDistTime(const EMPTrajPred_t *pIn_Obj1,
                           const EMPTrajPred_t *pIn_Obj2,
                           float32 *pOut_MinDistTime) {
    boolean bResultOK = FALSE;

    const float32 fC2Limit = 0.1f;
    const float32 fC1Limit = 0.1f;

    float32 fC2RelX = pIn_Obj1->XofT.fC2 - pIn_Obj2->XofT.fC2;
    float32 fC1RelX = pIn_Obj1->XofT.fC1 - pIn_Obj2->XofT.fC1;
    float32 fC0RelX = pIn_Obj1->XofT.fC0 - pIn_Obj2->XofT.fC0;

    float32 fC2RelY = pIn_Obj1->YofT.fC2 - pIn_Obj2->YofT.fC2;
    float32 fC1RelY = pIn_Obj1->YofT.fC1 - pIn_Obj2->YofT.fC1;
    float32 fC0RelY = pIn_Obj1->YofT.fC0 - pIn_Obj2->YofT.fC0;

    float32 fC2RelXSq = fC2RelX * fC2RelX;
    float32 fC2RelYSq = fC2RelY * fC2RelY;
    float32 fC1RelXSq = fC1RelX * fC1RelX;
    float32 fC1RelYSq = fC1RelY * fC1RelY;

    const float32 fAbsC2RelX = fABS(fC2RelX);
    const float32 fAbsC2RelY = fABS(fC2RelY);
    const float32 fAbsC1RelX = fABS(fC1RelX);
    const float32 fAbsC1RelY = fABS(fC1RelY);

    *pOut_MinDistTime = 0;

    if (fAbsC2RelX < fC2Limit) {
        fC2RelX = 0;
        fC2RelXSq = 0;
    }
    if (fAbsC1RelX < fC1Limit) {
        fC1RelX = 0;
        fC1RelXSq = 0;
    }
    if (fAbsC2RelY < fC2Limit) {
        fC2RelY = 0;
        fC2RelYSq = 0;
    }
    if (fAbsC1RelY < fC1Limit) {
        fC1RelY = 0;
        fC1RelYSq = 0;
    }

    if ((fAbsC2RelX < fC2Limit) && (fAbsC2RelY < fC2Limit)) {
        /* Both Object have the same Acceleration */
        if ((fAbsC1RelX < fC1Limit) && (fAbsC1RelY < fC1Limit)) {
            /* Both Objects have the same Velocity */
            *pOut_MinDistTime = 0;
            bResultOK = TRUE;
        } else {
            const float32 fPC1 = 2.0f * (fC1RelXSq + fC1RelYSq);
            const float32 fPC0 =
                2.0f * ((fC1RelX * fC0RelX) + (fC1RelY * fC0RelY));

            *pOut_MinDistTime = (-1.0f) * (fPC0 / fPC1);
            bResultOK = TRUE;
        }
    } else {
        const float32 fPC3 = 4.0f * (fC2RelXSq + fC2RelYSq);
        const float32 fPC2 = 6.0f * ((fC2RelX * fC1RelX) + (fC2RelY * fC1RelY));
        const float32 fPC1 =
            2.0f * ((2.0f * fC2RelX * fC0RelX) + (2.0f * fC2RelY * fC0RelY) +
                    fC1RelXSq + fC1RelYSq);
        const float32 fPC0 = 2.0f * ((fC1RelX * fC0RelX) + (fC1RelY * fC0RelY));

        const float32 fPC3Sq = fPC3 * fPC3;
        const float32 fPC2Sq = fPC2 * fPC2;
        const float32 fPC3Cu = fPC3Sq * fPC3;
        const float32 fPC2Cu = fPC2Sq * fPC2;

        const float32 fQ = ((3.0f * fPC3 * fPC1) - fPC2Sq) / (9.0f * fPC3Sq);
        const float32 fR = ((9.0f * fPC3 * fPC2 * fPC1) -
                            (27.0f * fPC3Sq * fPC0) - (2 * fPC2Cu)) /
                           (54.0f * fPC3Cu);
        const float32 fQCu = fQ * fQ * fQ;
        const float32 fRSQ = fR * fR;
        const float32 fD = fQCu + fRSQ;

        if (fD > BML_f_AlmostZero) {
            const float32 fSqrtD = SQRT(fD);
            const float32 fS = EMPCalcCubeRoot(fR + fSqrtD);
            const float32 fT = EMPCalcCubeRoot(fR - fSqrtD);
            *pOut_MinDistTime = (fS + fT) - (fPC2 / (3.0f * fPC3));
            bResultOK = TRUE;
        } else if ((fD < -BML_f_AlmostZero) &&
                   (fABS(fQCu) > BML_f_AlmostZero)) /* catch potential div/0 */
        {
            int iCurrMinTimeID;
            int iMinTimeGlobalID = -1;
            float32 fMinDistGlobal = 999.f;
            const float32 fSqrtQCuNeg = SQRT((-1.0f) * fQCu);
            const float32 fPhi = ACOS_(fR / fSqrtQCuNeg);

            float32 fMinTimes[3] = {0, 0, 0};
            const float32 fSqrtQNeg = SQRT((-1.0f) * fQ);

            fMinTimes[0] = (2.0f * fSqrtQNeg * COS_(fPhi / 3.0f)) -
                           ((fPC2) / (3.0f * fPC3));
            fMinTimes[1] = (2.0f * fSqrtQNeg *
                            COS_((fPhi / 3.0f) + ((2.0f * C_PI) / (3.0f)))) -
                           ((fPC2) / (3.0f * fPC3));
            fMinTimes[2] = (2.0f * fSqrtQNeg *
                            COS_((fPhi / 3.0f) + ((4.0f * C_PI) / (3.0f)))) -
                           ((fPC2) / (3.0f * fPC3));

            for (iCurrMinTimeID = 0; iCurrMinTimeID < 3; iCurrMinTimeID++) {
                if (fMinTimes[iCurrMinTimeID] > 0) {
                    float32 fCurrDist = EMPCalcObjObjDistAtTime(
                        fMinTimes[iCurrMinTimeID], pIn_Obj1, pIn_Obj2);
                    if (fCurrDist < fMinDistGlobal) {
                        fMinDistGlobal = fCurrDist;
                        iMinTimeGlobalID = iCurrMinTimeID;
                    }
                }
            }

            if (iMinTimeGlobalID >= 0) {
                *pOut_MinDistTime = fMinTimes[iMinTimeGlobalID];
                bResultOK = TRUE;
            } else {
                /* No Positive Number -> min dist lies in the past */
                *pOut_MinDistTime = 0;
                bResultOK = TRUE;
            }
        } else {
            /* This should not happen */
            bResultOK = FALSE;
        }
    }

    if (*pOut_MinDistTime < 0) {
        /* Time of minimum distance lies in the past */
        *pOut_MinDistTime = 0;
    }

    return bResultOK;
}

/* **************************************************************************** 

  Functionname:     EMPCalcObjExDesc                                     */ /*!

       @brief        Calculate object's Description
       @description    Calculate object's description as in its center, geometry
     and orientation

       @param[in]    fTime
       @param[in]    pIn_Obj
       @param[in]    pOut_ExistenceDesc

       @return       True if no error occurred


     ****************************************************************************
     */
boolean EMPCalcObjExDesc(float32 fTime,
                         const EMPObjPred_t *pIn_Obj,
                         EMPObjExDesc_t *pOut_ExistenceDesc) {
    boolean bResultOk = TRUE;

    pOut_ExistenceDesc->sCenter.fX =
        EMPEvalPolyDeg2(&pIn_Obj->pTrajPred->XofT, fTime);
    pOut_ExistenceDesc->sCenter.fY =
        EMPEvalPolyDeg2(&pIn_Obj->pTrajPred->YofT, fTime);
    pOut_ExistenceDesc->sGeometry = *pIn_Obj->pGeometry;
    pOut_ExistenceDesc->fOrientation =
        EMPCalcTrajOrientationAtTime(fTime, pIn_Obj->pTrajPred);

    bResultOk &=
        EMPCalcVariance2D(fTime, pIn_Obj->pTrajPred, &pOut_ExistenceDesc->sVar);

    return bResultOk;
}

/* ****************************************************************************

  Functionname:     EMPCalcCollProbExDesc                                */
boolean EMPCalcCollProbExDesc(const EMPObjExDesc_t *pIn_ExDescObj1,
                              const EMPObjExDesc_t *pIn_ExDescObj2,
                              float32 *pOut_CollisionProb) {
    EMPPos2D_t virtObj2Center;
    EMPVar2D_t virtObjVar;
    EMPSize2D_t virtObjSize;

    EMPPos2D_t intAreaFL, intAreaFR, intAreaRL, intAreaRR;

    /* Obj1 is in the center. Create virtual Obj2 relative to new position */
    virtObj2Center.fX = pIn_ExDescObj2->sCenter.fX - pIn_ExDescObj1->sCenter.fX;
    virtObj2Center.fY = pIn_ExDescObj2->sCenter.fY - pIn_ExDescObj1->sCenter.fY;

    /* Combine variances */
    virtObjVar.fX = pIn_ExDescObj1->sVar.fX + pIn_ExDescObj2->sVar.fX;
    virtObjVar.fY = pIn_ExDescObj1->sVar.fY + pIn_ExDescObj2->sVar.fY;

    /* Combine vehicle sizes into one big integration size */
    virtObjSize.fLength =
        pIn_ExDescObj1->sGeometry.fLength + pIn_ExDescObj2->sGeometry.fLength;
    virtObjSize.fWidth =
        pIn_ExDescObj1->sGeometry.fWidth + pIn_ExDescObj2->sGeometry.fWidth;

    /* Define Integration Area */
    intAreaFL.fX = virtObj2Center.fX + (virtObjSize.fLength / 2.0f);
    intAreaFL.fY = virtObj2Center.fY + (virtObjSize.fWidth / 2.0f);

    intAreaFR.fX = virtObj2Center.fX + (virtObjSize.fLength / 2.0f);
    intAreaFR.fY = virtObj2Center.fY - (virtObjSize.fWidth / 2.0f);

    intAreaRL.fX = virtObj2Center.fX - (virtObjSize.fLength / 2.0f);
    intAreaRL.fY = virtObj2Center.fY + (virtObjSize.fWidth / 2.0f);

    intAreaRR.fX = virtObj2Center.fX - (virtObjSize.fLength / 2.0f);
    intAreaRR.fY = virtObj2Center.fY - (virtObjSize.fWidth / 2.0f);

    /* Integrate using 2D CDF */
    {
        float32 cdfFL = EMPCalcStdGaussianCDF2D(intAreaFL.fX, intAreaFL.fY, 0,
                                                virtObjVar.fX, virtObjVar.fY);
        float32 cdfFR = EMPCalcStdGaussianCDF2D(intAreaFR.fX, intAreaFR.fY, 0,
                                                virtObjVar.fX, virtObjVar.fY);
        float32 cdfRL = EMPCalcStdGaussianCDF2D(intAreaRL.fX, intAreaRL.fY, 0,
                                                virtObjVar.fX, virtObjVar.fY);
        float32 cdfRR = EMPCalcStdGaussianCDF2D(intAreaRR.fX, intAreaRR.fY, 0,
                                                virtObjVar.fX, virtObjVar.fY);

        *pOut_CollisionProb = ((cdfFL - cdfFR) - cdfRL) + cdfRR;
    }

    return TRUE;
}

/* **************************************************************************** 

  Functionname:     EMPEvalPolyDeg2                                      */ /*!

       @brief            Calculate value of second degree Polynomial

       @param[in]        pInPolynom
       @param[in]        fSamplePoint

       @return           Value of Polynomial


     ****************************************************************************
     */
static float32 EMPEvalPolyDeg2(const EMPPolyDeg2_t *pInPolynom,
                               float32 fSamplePoint) {
    return (pInPolynom->fC2 * fSamplePoint * fSamplePoint) +
           (pInPolynom->fC1 * fSamplePoint) + pInPolynom->fC0;
}

/* **************************************************************************** 

  Functionname:     EMPCalcTrajOrientationAtTime                         */ /*!

       @brief        Calculate Orientation of a given EMPTrajPred_t

       @param[in]    fTime
       @param[in]    pInTraj

       @return       Orientation of Trajectory at given time from -pi to pi
     (radiant)
       @attention    TODO: Check what happens if x_gradient == 0.0f


     ****************************************************************************
     */
static float32 EMPCalcTrajOrientationAtTime(float32 fTime,
                                            const EMPTrajPred_t *pInTraj) {
    float32 fGradX = EMPEvalPolyDeg2(&(pInTraj->XofT), fTime);
    float32 fGradY = EMPEvalPolyDeg2(&(pInTraj->YofT), fTime);

    return ATAN2_(fGradY, fGradX);
}

/* **************************************************************************** 

  Functionname:     EMPCalcVariance2D                                     */ /*!

      @brief        Calculate 2-Dimensional Variance based on Trajectory
    Prediction

      @param[in]    fTime
      @param[in]    pInTraj
      @param[in]    pOut_Var2D

      @return       True (if no error occurred)


    ****************************************************************************
    */
static boolean EMPCalcVariance2D(float32 fTime,
                                 const EMPTrajPred_t *pInTraj,
                                 EMPVar2D_t *pOut_Var2D) {
    pOut_Var2D->fX = EMPEvalPolyDeg2(&pInTraj->VarXofT, fTime);
    pOut_Var2D->fY = EMPEvalPolyDeg2(&pInTraj->VarYofT, fTime);

    return TRUE;
}

/* **************************************************************************** 

  Functionname:     EMPCalcStdGaussianCDF2D                              */ /*!

       @brief        Calculate 2-Dimensional CDF without rho (=0)

       @param[in]    fValueX
       @param[in]    fValueY
       @param[in]    fAver
       @param[in]    fSigmaX
       @param[in]    fSigmaY


       @return       2D CDF Value

       @attention    TODO:Check what happens, when fSigma ~ 0


     ****************************************************************************
     */
static float32 EMPCalcStdGaussianCDF2D(float32 fValueX,
                                       float32 fValueY,
                                       float32 fAver,
                                       float32 fSigmaX,
                                       float32 fSigmaY) {
    return BML_f_CalcStdGaussianCDF(fValueX, fAver, fSigmaX) *
           BML_f_CalcStdGaussianCDF(fValueY, fAver, fSigmaY);
}

/* **************************************************************************** 

  Functionname:     EMPCalcCubeRoot                                      */ /*!

       @brief        Calculate cube root of given value

       @param[in]    fX

       @return       Cube root of fX


     ****************************************************************************
     */
static float32 EMPCalcCubeRoot(float32 fX) {
    float32 fResult = 0;
    boolean bInputZero = (boolean)((fX > -C_F32_DELTA) && (fX < C_F32_DELTA));

    if (bInputZero != FALSE) {
        fResult = 0;
    } else {
        boolean bInputNegative = (boolean)(fX < 0);
        float32 fXAbs = fABS(fX);
        float32 fIterResult = SQRT(fXAbs);  // fX / 2.f;
        float32 fIterResultOld = 0;
        const uint8 uiMaxIterations = 20u;
        uint8 uiCurrID = 0u;
        float32 fDiffAbortCrit = fIterResult;
        boolean bAbortCritMet = FALSE;

        for (uiCurrID = 0;
             (uiCurrID < uiMaxIterations) && (bAbortCritMet != TRUE);
             ++uiCurrID) {
            float32 fIterResultCube = fIterResult * fIterResult * fIterResult;
            fIterResult = fIterResult * ((fIterResultCube + (2 * fXAbs)) /
                                         ((2 * fIterResultCube) + fXAbs));
            fDiffAbortCrit = fABS(fIterResult - fIterResultOld);
            bAbortCritMet = (fDiffAbortCrit > BML_f_Delta) ? FALSE : TRUE;
            fIterResultOld = fIterResult;
        }

        if (bInputNegative != FALSE) {
            fResult = -fIterResult;
        } else {
            fResult = fIterResult;
        }
    }
    return fResult;
}

/*****************************************************************************
                                 EMP LEGACY
*****************************************************************************/

/* **************************************************************************** 

  Functionname:     EMPLEGCalcCorridorOccupancy                          */ /*!

       @brief        Calculate Occupancy of Corridor by Object

       @param[in]    fObjDistToCourse
       @param[in]    fObjWidth
       @param[in]    fTrajWidth
       @param[out]    pOut_CorridorOccupancy

       @return       NONE


     ****************************************************************************
     */
void EMPLEGCalcCorridorOccupancy(float32 fObjDistToCourse,
                                 float32 fObjWidth,
                                 float32 fTrajWidth,
                                 float32 *pOut_CorridorOccupancy) {
    float32 fOverlapZoneEdgeLow;
    float32 fOverlapZoneEdgeHigh;
    float32 fTrajectoryEdgeLow;
    float32 fTrajectoryEdgeHigh;
    float32 fOverlapZoneWidth;

    fObjDistToCourse = fABS(fObjDistToCourse);

    /* Calculate Overlap Zone Width */
    fOverlapZoneEdgeLow = fObjDistToCourse - (fObjWidth / 2.0f);
    fOverlapZoneEdgeHigh = fObjDistToCourse + (fObjWidth / 2.0f);
    fTrajectoryEdgeLow = -(fTrajWidth / 2.0f);
    fTrajectoryEdgeHigh = (fTrajWidth / 2.0f);

    /* Restrain Overlap Zone to Trajectory Zone */
    fOverlapZoneEdgeLow =
        MINMAX(fTrajectoryEdgeLow, fTrajectoryEdgeHigh, fOverlapZoneEdgeLow);
    fOverlapZoneEdgeHigh =
        MINMAX(fTrajectoryEdgeLow, fTrajectoryEdgeHigh, fOverlapZoneEdgeHigh);

    fOverlapZoneWidth = fOverlapZoneEdgeHigh - fOverlapZoneEdgeLow;

    /* Calculate Occupancy */
    *pOut_CorridorOccupancy = fOverlapZoneWidth / fTrajWidth;
}

/* **************************************************************************** 

  Functionname:     EMPLEGCalcObjOccupancy                               */ /*!

       @brief        Calculate Occupancy of Obj by Corridor

       @param[in]    fObjDistToCourse
       @param[in]    fObjWidth
       @param[in]    fTrajWidth
       @param[out]   pOut_ObjOccupancy

       @return       NONE


     ****************************************************************************
     */
void EMPLEGCalcObjOccupancy(float32 fObjDistToCourse,
                            float32 fObjWidth,
                            float32 fTrajWidth,
                            float32 *pOut_ObjOccupancy) {
    float32 fOverlapZoneEdgeLow;
    float32 fOverlapZoneEdgeHigh;
    float32 fTrajectoryEdgeLow;
    float32 fTrajectoryEdgeHigh;
    float32 fOverlapZoneWidth;

    fObjDistToCourse = fABS(fObjDistToCourse);

    /* Calculate Overlap Zone Width */
    fOverlapZoneEdgeLow = fObjDistToCourse - (fObjWidth / 2.0f);
    fOverlapZoneEdgeHigh = fObjDistToCourse + (fObjWidth / 2.0f);
    fTrajectoryEdgeLow = -(fTrajWidth / 2.0f);
    fTrajectoryEdgeHigh = (fTrajWidth / 2.0f);

    /* Restrain Overlap Zone to Trajectory Zone */
    fOverlapZoneEdgeLow =
        MINMAX(fTrajectoryEdgeLow, fTrajectoryEdgeHigh, fOverlapZoneEdgeLow);
    fOverlapZoneEdgeHigh =
        MINMAX(fTrajectoryEdgeLow, fTrajectoryEdgeHigh, fOverlapZoneEdgeHigh);

    fOverlapZoneWidth = fOverlapZoneEdgeHigh - fOverlapZoneEdgeLow;

    /* Calculate Occupancy of Obj */
    *pOut_ObjOccupancy = fOverlapZoneWidth / fObjWidth;
}

/* ***********************************************************************
  @fn             EMPCPCalculateOverlap                           */
void EMPCPCalculateOverlap(EMPDistanceWidth_t const *const pDistanceWidth,
                           EMPTrajOccupancy_t *const pOccupancy) {
    float32 A, A2, B, B2, C, AVar, BVar, CVar, ABCVar, AL_BL_n, AL_BL_y,
        AR_BR_n, AR_BR_y;
    float32 AL_BL_n_x_AR_BR_n, AL_BL_y_x_AR_BR_n, AL_BL_n_x_AR_BR_y,
        AL_BL_y_x_AR_BR_y;

    const float32 fAbsTrajectoryWidth = fABS(pDistanceWidth->fTrajectoryWidth);
    const float32 fAbsObjectWidth = fABS(pDistanceWidth->fObjectWidth);

    if ((fAbsTrajectoryWidth < C_F32_DELTA) ||
        (fAbsObjectWidth < C_F32_DELTA)) {
        pOccupancy->fOverlap = 0;
        pOccupancy->fOverlapVar = 0;
        pOccupancy->fTrajectoryOccupancy = 0;
        pOccupancy->fTrajectoryOccupancyVar = 0;
        pOccupancy->fObjectOccupancy = 0;
        pOccupancy->fObjectOccupancyVar = 0;
    } else {
        A = pDistanceWidth->fTrajectoryWidth;
        A2 = 0.5f * A;
        AVar = pDistanceWidth->fTrajectoryWidthVar;
        B = pDistanceWidth->fObjectWidth;
        B2 = 0.5f * B;
        BVar = pDistanceWidth->fObjectWidthVar;
        C = pDistanceWidth->fDistance;
        CVar = pDistanceWidth->fDistanceVar;

        ABCVar = (0.25f * AVar) + (0.25f * BVar) + CVar;
        AL_BL_n = GDBCalcCumulatDistrFunction(0.0f, (A2 - B2) + C, ABCVar);
        AL_BL_y = 1.0f - AL_BL_n;
        AR_BR_n = GDBCalcCumulatDistrFunction(0.0f, -A2 + B2 + C, ABCVar);
        AR_BR_y = 1.0f - AR_BR_n;

        AL_BL_n_x_AR_BR_n = AL_BL_n * AR_BR_n;
        AL_BL_y_x_AR_BR_n = AL_BL_y * AR_BR_n;
        AL_BL_n_x_AR_BR_y = AL_BL_n * AR_BR_y;
        AL_BL_y_x_AR_BR_y = AL_BL_y * AR_BR_y;

        /* absolute ?berdeckung gewichtete Summe entsprechend der
         * Entscheidungsws.*/
        pOccupancy->fOverlap =
            (AL_BL_n_x_AR_BR_n * (A2 + B2 + C)) + (AL_BL_n_x_AR_BR_y * A) +
            (AL_BL_y_x_AR_BR_n * B) + (AL_BL_y_x_AR_BR_y * ((A2 + B2) - C));
        pOccupancy->fOverlapVar =
            (AL_BL_n_x_AR_BR_n * ABCVar) + (AL_BL_n_x_AR_BR_y * AVar) +
            (AL_BL_y_x_AR_BR_n * BVar) + (AL_BL_y_x_AR_BR_y * ABCVar);

        /* relative ?berdeckung bezgl. A*/
        pOccupancy->fTrajectoryOccupancy = (pOccupancy->fOverlap / A);
        pOccupancy->fTrajectoryOccupancyVar =
            (pOccupancy->fOverlapVar / SQR(A));

        /* relative ?berdeckung bezgl. B*/
        pOccupancy->fObjectOccupancy = (pOccupancy->fOverlap / B);
        pOccupancy->fObjectOccupancyVar = (pOccupancy->fOverlapVar / SQR(B));
    }
}

/* **************************************************************************** 

  Functionname:     EMPCalcObjObjDistAtTime                               */ /*!

      @brief            Calculate distance b/w two objects at any given time

      @param[in]    fTime            time
      @param[in]    pIn_Obj1         pointer to the trajectory of first object
      @param[in]    pIn_Obj2         pointer to the trajectory of second object

      @return       Dist b/w two objects


    ****************************************************************************
    */
float32 EMPCalcObjObjDistAtTime(float32 fTime,
                                const EMPTrajPred_t *pIn_Obj1,
                                const EMPTrajPred_t *pIn_Obj2) {
    EMPPos2D_t PosObj1;
    EMPPos2D_t PosObj2;
    float32 fOutDist = 0;

    EMPCalcPositionAtTime(fTime, pIn_Obj1, &PosObj1);
    EMPCalcPositionAtTime(fTime, pIn_Obj2, &PosObj2);

    fOutDist =
        SQRT(SQR(PosObj1.fX - PosObj2.fX) + SQR(PosObj1.fY - PosObj2.fY));

    return fOutDist;
}

/* **************************************************************************** 

  Functionname:     EMPCalcPositionAtTime                               */ /*!

        @brief            Calc Position of an object At any given Time and
      trajectory

        @param[in]    fTime       time
        @param[in]    pIn_Obj     pointer to the trajectory of the object
        @param[out]    pOut_Position  position of the object

        @return       Object Occupancy


      ****************************************************************************
      */
void EMPCalcPositionAtTime(float32 fTime,
                           const EMPTrajPred_t *pIn_Obj,
                           EMPPos2D_t *pOut_Position) {
    pOut_Position->fX = EMPEvalPolyDeg2(&(pIn_Obj->XofT), fTime);
    pOut_Position->fY = EMPEvalPolyDeg2(&(pIn_Obj->YofT), fTime);
}

/* **************************************************************************** 

  Functionname:     EMPCalcVariance                               */ /*!

              @brief        Calculate Variance of the given numbers stored in an
            array

              @param[in]    afValues                 Array values
              @param[in]    uiArraySize              Array size

              @return       variance of given numbers


            ****************************************************************************
            */
float32 EMPCalcVariance(const float32 afValues[], uint16 uiArraySize) {
    float32 fSumforAverage = 0;
    float32 fAverage = 0;
    float32 fSumforVariance = 0;
    float32 fVariance = 0;
    uint16 uiCurrID = 0u;

    for (uiCurrID = 0u; uiCurrID < uiArraySize; ++uiCurrID) {
        fSumforAverage += afValues[uiCurrID];
    }

    fAverage = fSumforAverage / (float32)uiArraySize;

    for (uiCurrID = 0u; uiCurrID < uiArraySize; ++uiCurrID) {
        fSumforVariance += SQR(afValues[uiCurrID] - fAverage);
    }

    fVariance = fSumforVariance / (uiArraySize - 1);

    return fVariance;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */