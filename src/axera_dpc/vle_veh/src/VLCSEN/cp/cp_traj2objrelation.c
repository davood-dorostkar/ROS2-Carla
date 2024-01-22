/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cp.h"
#include "cp_kalman.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#define FIX_WIDTH_MODE 0
#define CORNER_POINT_MODE 1
const volatile uint8 CAL_OBJ_WIDTH_MODE = CORNER_POINT_MODE;

#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

// #define LMURAM0_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

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
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    CPCalculateOverlap */
void CPCalculateOverlap(const CPDistanceWidth_t *pDistanceWidth,
                        CPTrajOccupancy_t *pOccupancy) {
    float32 A, A2, B, B2, C, AVar, BVar, CVar, ABCVar, AL_BL_n, AL_BL_y,
        AR_BR_n, AR_BR_y;
    float32 AL_BL_n_x_AR_BR_n, AL_BL_y_x_AR_BR_n, AL_BL_n_x_AR_BR_y,
        AL_BL_y_x_AR_BR_y;

    const float32 fAbsTrajectoryWidth = fABS(pDistanceWidth->fTrajectoryWidth);
    float32 fAbsObjectWidth;

    if (CAL_OBJ_WIDTH_MODE == FIX_WIDTH_MODE) {
        fAbsObjectWidth = fABS(pDistanceWidth->fObjectWidth);
    } else {
        fAbsObjectWidth = fABS(pDistanceWidth->fObjectCorridorWidth);
    }

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
        B = pDistanceWidth->fObjectCorridorWidth;
        B2 = 0.5f * B;
        BVar = pDistanceWidth->fObjectCorridorWidthVar;
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

/*************************************************************************************************************************
  Functionname:    CPCalculateInlap */
void CPCalculateInlap(const CPDistanceWidth_t *pDistanceWidth,
                      CPTrajInlap_t *pInlap) {
    const float32 fObjectWidth = pDistanceWidth->fObjectWidth;
    const float32 fTrajWidth = pDistanceWidth->fTrajectoryWidth;
    const float32 fDistance = pDistanceWidth->fDistance;

    const float32 fAbsObjectWidth = fABS(pDistanceWidth->fObjectWidth);
    const float32 fAbsTrajWidth = fABS(pDistanceWidth->fTrajectoryWidth);
    const float32 fABSDistance = fABS(fDistance);

    const float32 fObjectWidthVar = pDistanceWidth->fObjectWidthVar;
    const float32 fTrajWidthVar = pDistanceWidth->fTrajectoryWidthVar;
    const float32 fDistanceVar = pDistanceWidth->fDistanceVar;

    if ((fAbsTrajWidth < C_F32_DELTA) || (fAbsObjectWidth < C_F32_DELTA)) {
        pInlap->fInlap = CP_INVALID_INLAP;
        pInlap->fInlapVar = CP_INVALID_INLAP_VAR;
    } else {
        pInlap->fInlap =
            (0.5f * fObjectWidth) + (0.5f * fTrajWidth) - fABSDistance;

        pInlap->fInlapVar =
            (0.25f * fObjectWidthVar) + (0.25f * fTrajWidthVar) + fDistanceVar;
    }
}

/*************************************************************************************************************************
  Functionname:    CPInitObjDist2Traj */
void CPInitObjDist2Traj(const CPObjDist2TrajMeas_t *pObjDist2TrajMeas,
                        CPObjDist2Traj_t *pObjDist2TrajDist) {
    CPKalmanInit(pObjDist2TrajMeas, &pObjDist2TrajDist->TrajDistFilt);

    /* Initialization of the below fields was not present in ARS300, and
    strictly speaking
    is not necessary (since these fields will be automatically filled with new
    measurement
    value when an object realy exists) */

    /* Initialization of the dist on trajectory field also not necessary in
    theory, but it's
    better to have a well-defined initial state */
    pObjDist2TrajDist->ObjDistOnTraj = 0.0f;
}

/*************************************************************************************************************************
  Functionname:    CPUpdateObjDist2Traj */
void CPUpdateObjDist2Traj(const ObjNumber_t iObj,
                          const float32 fMaxAccelDist2Traj,
                          CPTrajectoryData_t const *pTrajData,
                          CPObjDist2Traj_t *pObjDist2TrajDist) {
    GDBSymMatrix2_t Q;
    CPTrajDistKalmanMeas_t ObjDistMeas;
    float32 fCycleTime;
    float32 fCycleTimeInv;
    CPTrajRefPoint_t TrajRefPoint, TrajLastCycRefPoint, TrajRefPointLastCycle;
    float32 RefPosVar;
    float32 RefSpeedVar;
    fCycleTime = CP_CYCLE_TIME;
    fCycleTimeInv = (1.f / fCycleTime);

    CPCalculateObjectReference(iObj, pTrajData, &TrajRefPoint,
                               &TrajLastCycRefPoint, &TrajRefPointLastCycle);

    /*determine model error matrix*/
    Q.f11 = SQR(fCycleTime) * SQR(fMaxAccelDist2Traj * (1.0F / 3.0F));
    Q.f00 = Q.f11 * fCycleTime * 0.5F;
    Q.f01 = 0.0F;
    /* control of Q Matrix if RefPoint moved since last cycle */
    RefPosVar = SQR(TrajRefPoint.fX - TrajLastCycRefPoint.fX) +
                SQR(TrajRefPoint.fY - TrajLastCycRefPoint.fY);
    RefSpeedVar = RefPosVar * SQR(fCycleTimeInv);
    Q.f00 += RefPosVar;
    Q.f11 += RefSpeedVar;

    ObjDistMeas.Y.f0 = TrajRefPoint.fDistToTraj;

    /* @todo should be Sum of Variances of RefPoint and Object in Normal
     * Direction */
    ObjDistMeas.R.f00 = OBJ_LAT_DISPLACEMENT_VAR(iObj);

    /*determine filtered distance to trajectory at object*/
    CPKalmanFiltering(iObj, &ObjDistMeas, &Q, fCycleTime,
                      &pObjDist2TrajDist->TrajDistFilt);

    /*store measured distance to trajectory at object*/
    pObjDist2TrajDist->TrajDistMeas.Y.f0 = ObjDistMeas.Y.f0;
    pObjDist2TrajDist->TrajDistMeas.R.f00 = ObjDistMeas.R.f00;
    /*ObjDistOnTraj and fDistOnTraj refer to center of gravity of ego
     * vehicle!!!*/
    pObjDist2TrajDist->ObjDistOnTraj = TrajRefPoint.fDistOnTraj;
}

/*************************************************************************************************************************
  Functionname:    CPDeleteObjDist2Traj */
void CPDeleteObjDist2Traj(CPObjDist2Traj_t *pObjDist2TrajDist) {
    pObjDist2TrajDist->TrajDistFilt.X.f0 = 0.0F;
    pObjDist2TrajDist->TrajDistFilt.X.f1 = 0.0F;
    pObjDist2TrajDist->TrajDistFilt.P.f00 = 0.0F;
    pObjDist2TrajDist->TrajDistFilt.P.f01 = 0.0F;
    pObjDist2TrajDist->TrajDistFilt.P.f11 = 0.0F;
}

/*************************************************************************************************************************
  Functionname:    CPCalculateObjectReference */
void CPCalculateObjectReference(ObjNumber_t iObj,
                                CPTrajectoryData_t const *pTrajData,
                                CPTrajRefPoint_t *pTrajRefPoint,
                                CPTrajRefPoint_t *pTrajLastCycRefPoint,
                                CPTrajRefPoint_t *pTrajRefPointLastCycle) {
    /* CAUTION!! this function is AutoSAR !!! */
    /*--- (SYMBOLIC) CONSTANTS ---*/

    /*--- VARIABLES ---*/
    GDBTrajectoryData_t CurTraj;
    /*COM2COMlc*/
    const GDBTrafoMatrix2D_t *pBackMat = VLCGetTrafoMatrix2DCOFBackward();
    /*COF2COG Trafo*/
    float32 fObjCenterX = 0;
    float32 fObjCenterY = 0;
    for (int i = 0; i < 4; i++) {
        fObjCenterX += OBJ_GEOMETRY(iObj).aShapePointCoordinates[i].fPosX / 4;
        fObjCenterY += OBJ_GEOMETRY(iObj).aShapePointCoordinates[i].fPosY / 4;
    }
    float32 fObjDistX = OBJ_LONG_DISPLACEMENT(iObj) + fObjCenterX;
    float32 fObjDistY = OBJ_LAT_DISPLACEMENT(iObj) + fObjCenterY;

    pTrajRefPointLastCycle->fDistToTraj = 0.0f;
    pTrajRefPointLastCycle->fDistOnTraj = 0.0f;
    pTrajRefPointLastCycle->fX = 0.0f;
    pTrajRefPointLastCycle->fY = 0.0f;

    CurTraj.fTrajC0 = pTrajData->KafiEst.XVec.pdData[sTRAJ_C0];
    CurTraj.fTrajC1 = pTrajData->KafiEst.XVec.pdData[sTRAJ_C1];
    CurTraj.fTrajAngle = 0.0f;

    CPCalculateDistance2Traj(fObjDistX, fObjDistY,
                             (boolean)pTrajData->State.EgoCourseOnly, &CurTraj,
                             pTrajRefPoint);

    /*COG2COGlc Trafo*/
    GDBmathTrafoPos2D(pBackMat, &pTrajRefPoint->fX, &pTrajRefPoint->fY);
    GDBmathTrafoPos2D(pBackMat, &fObjDistX, &fObjDistY);

    CPCalculateDistance2Traj(fObjDistX, fObjDistY,
                             (boolean)pTrajData->State.EgoCourseOnly,
                             &pTrajData->LastCycle, pTrajLastCycRefPoint);
}

/*************************************************************************************************************************
  Functionname:    CPTrajGetObjToRefDistMeas */
float32 CPTrajGetObjToRefDistMeas(const CPObjDist2Traj_t *pObjDist2TrajDist) {
    return pObjDist2TrajDist->TrajDistMeas.Y.f0;
}

/*************************************************************************************************************************
  Functionname:    CPTrajGetObjToRefDistMeasVar */
float32 CPTrajGetObjToRefDistMeasVar(
    const CPObjDist2Traj_t *pObjDist2TrajDist) {
    return pObjDist2TrajDist->TrajDistMeas.R.f00;
}
/*************************************************************************************************************************
  Functionname:    CPTrajGetObjToRefDistFilt */
float32 CPTrajGetObjToRefDistFilt(const CPObjDist2Traj_t *pObjDist2TrajDist) {
    return pObjDist2TrajDist->TrajDistFilt.X.f0;
}

/*************************************************************************************************************************
  Functionname:    CPTrajGetObjToRefDistFiltVar */
float32 CPTrajGetObjToRefDistFiltVar(
    const CPObjDist2Traj_t *pObjDist2TrajDist) {
    return pObjDist2TrajDist->TrajDistFilt.P.f00;
}

/*************************************************************************************************************************
  Functionname:    CPTrajGetObjToRefDistGradFilt */
float32 CPTrajGetObjToRefDistGradFilt(
    const CPObjDist2Traj_t *pObjDist2TrajDist) {
    return pObjDist2TrajDist->TrajDistFilt.X.f1;
}

/*************************************************************************************************************************
  Functionname:    CPTrajGetObjToRefDistGradFiltVar */
float32 CPTrajGetObjToRefDistGradFiltVar(
    const CPObjDist2Traj_t *pObjDist2TrajDist) {
    return pObjDist2TrajDist->TrajDistFilt.P.f11;
}

/*************************************************************************************************************************
  Functionname:    CPTrajGetObjDistOnTraj */
float32 CPTrajGetObjDistOnTraj(const CPObjDist2Traj_t *pObjDist2TrajDist) {
    return pObjDist2TrajDist->ObjDistOnTraj;
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM0_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */