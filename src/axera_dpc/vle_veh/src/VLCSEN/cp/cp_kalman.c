
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "cp.h"
#include "cp_kalman.h"
#include "cp_par.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! @brief   these parameters determine the dynamic limits of the filter */
#define SA_MAX_VREL (11.0F / C_KMH_MS)

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

static void CPKalmanSetTrajModelMat(const CPTrajectoryData_t *pTrajData);

/*************************************************************************************************************************
  Functionname:    CPKalmanInit */
void CPKalmanInit(const CPTrajDistKalmanMeas_t *pMeasurement,
                  CPTrajDistKalmanData_t *pTrajDist) {
    /* Initialize X Vector */
    /* The estimation at startup is given by the initial measurement */
    pTrajDist->X.f0 = pMeasurement->Y.f0;
    pTrajDist->X.f1 = 0.0F;

    /* Initialize P Matrix */
    /* The estimation accuracy at startup is given by the measurement accuracy
       of the
       initial measurement */

    pTrajDist->P.f00 = pMeasurement->R.f00;
    pTrajDist->P.f01 = 0.0F;
    pTrajDist->P.f11 = SQR(SA_MAX_VREL); /* max expected lateral speed */
}

/*************************************************************************************************************************
  Functionname:    CPKalmanPredict */
void CPKalmanPredict(CPTrajDistKalmanData_t *pTrajDist,
                     const GDBSymMatrix2_t *pQ,
                     float32 fCycleTime) {
    /*--- (SYMBOLIC) CONSTANTS ---*/

    /*--- VARIABLES ---*/

    float32 fTemp;
    GDBSymMatrix2_t *pP;

    pTrajDist->X.f0 += fCycleTime * pTrajDist->X.f1;

    /* get pointer to P matrix */
    pP = &(pTrajDist->P);

    /* P = Q + A * P * A' */
    /* temp variable for run time optimization */
    fTemp = BML_f_MultAdd(fCycleTime, pP->f11, pP->f01);

    pP->f00 += (BML_f_MultAdd(fCycleTime, (pP->f01 + fTemp), pQ->f00));
    pP->f01 = pQ->f01 + fTemp;
    pP->f11 += pQ->f11;

    /*--- UNDEF SYMBOLIC CONSTANTS ---*/
}

/*************************************************************************************************************************
  Functionname:    CPKalmanUpdate */
void CPKalmanUpdate(const CPTrajDistKalmanMeas_t *pMeasurement,
                    CPTrajDistKalmanData_t *pTrajDist) {
    /*--- (SYMBOLIC) CONSTANTS ---*/

    /*--- VARIABLES ---*/
    float32 fTemp;
    float32 fK00, fK10;
    GDBSymMatrix2_t *pP;
    const GDBSymMatrix1_t *pR;

    /* get pointer to P matrix */
    pP = &(pTrajDist->P);

    pR = &(pMeasurement->R);

    fTemp = pP->f00 + pR->f00;

    fK00 = pP->f00 / fTemp;
    fK10 = pP->f01 / fTemp;

    /* compute innovation */
    fTemp = pMeasurement->Y.f0 - pTrajDist->X.f0;

    /* estimation */
    pTrajDist->X.f0 += (fK00 * fTemp);
    pTrajDist->X.f1 += (fK10 * fTemp);

    fTemp = 1.0F - fK00;

    pP->f11 -= (fK10 * pP->f01);
    pP->f00 *= fTemp;
    pP->f01 *= fTemp;

    /*--- UNDEF SYMBOLIC CONSTANTS ---*/
}

/*************************************************************************************************************************
  Functionname:    CPKalmanFiltering */
void CPKalmanFiltering(ObjNumber_t iObj,
                       const CPTrajDistKalmanMeas_t *pMeasurement,
                       const GDBSymMatrix2_t *pQ,
                       float32 fCycleTime,
                       CPTrajDistKalmanData_t *pTrajDist) {
    /*--- (SYMBOLIC) CONSTANTS ---*/

    /*--- VARIABLES ---*/

    /* initialization and filtering */
    if (OBJ_IS_NEW(iObj)) {
        /* initialize filter in the first cycle (after OT lifetime is 1 in first
         * cycle) */
        CPKalmanInit(pMeasurement, pTrajDist);
    } else {
        /* calc KALMAN prediction */
        CPKalmanPredict(pTrajDist, pQ, fCycleTime);

        /* compute system noise covariance for handling target maneuvers */
        /* SADistanceKalmanSysNoise(iObj); */
        /* distance  filtering */
        CPKalmanUpdate(pMeasurement, pTrajDist);
    }
    /*--- UNDEF SYMBOLIC CONSTANTS ---*/
}

/*************************************************************************************************************************
  Functionname:    CPKalmanUpdatePos */
void CPKalmanUpdatePos(const CPPosSamples_t *pFUSED_Samples,
                       CPNoiseModelLinear_t sNoiseModel,
                       const CPTrajectoryData_t *pTrajData) {
    uint16 i;
    float32 fxTgt, fxTgt2, fYNoise;

    GDBBaseMatrix_t KMat; /* kalman gain */
    GDBBaseMatrix_t FVec;
    GDBBaseMatrix_t CMat;
    float32 KMatrix[NB_TRAJ_STATE];
    float32 FVector[NB_TRAJ_STATE];
    float32 CMatrix[NB_TRAJ_STATE];

    /* Create intermediate Matrixes for the Measurement Update */
    GDBKalmanCreateMat(&KMat, KMatrix, MATTYPE_VECTOR, NB_TRAJ_STATE, (uint8)1);
    GDBKalmanCreateMat(&FVec, FVector, MATTYPE_VECTOR, NB_TRAJ_STATE, (uint8)1);
    GDBKalmanCreateMat(&CMat, CMatrix, MATTYPE_VECTOR, (uint8)1, NB_TRAJ_STATE);

    /* sequential measurement updates for all relevant Targets */
    for (i = 0u; i < pFUSED_Samples->nb_samples; i++) {
        fxTgt = pFUSED_Samples->fx[i] + VLC_fBumperToCoG;
        fxTgt2 = fxTgt * fxTgt;
        /* Complete  Measurement Matrix C */
        CMat.pdData[VECT_MAT_INDEX(sTRAJ_C0)] = fxTgt2 * (1.0f / 2.0f);

        CMat.pdData[VECT_MAT_INDEX(sTRAJ_C1)] = fxTgt2 * fxTgt * C_SIXTH;

        fYNoise =
            sNoiseModel.fNoiseOffset + (sNoiseModel.fNoiseGradient * fxTgt);
        fYNoise = SQR(fYNoise);

        /* Innovation */
        GDBKalmanMeasUpdate(&pTrajData->KafiEst, KMat, FVec, CMat,
                            pFUSED_Samples->fy[i], fYNoise);
    }
    GDBKalmanUpdatePDiag(&pTrajData->KafiEst);
}

/*************************************************************************************************************************
  Functionname:    CPKalmanUpdateGrad */
void CPKalmanUpdateGrad(const CPGradSamples_t *pFUSED_GradSamples,
                        const CPTrajectoryData_t *pTrajData) {
    uint16 i;
    float32 fxTgt, fxTgt2, fYNoise;

    GDBBaseMatrix_t KMat; /* kalman gain */
    GDBBaseMatrix_t FVec;
    GDBBaseMatrix_t CMat;
    float32 KMatrix[NB_TRAJ_STATE];
    float32 FVector[NB_TRAJ_STATE];
    float32 CMatrix[NB_TRAJ_STATE];

    /* Create intermediate Matrixes for the Measurement Update */
    GDBKalmanCreateMat(&KMat, KMatrix, MATTYPE_VECTOR, NB_TRAJ_STATE, (uint8)1);
    GDBKalmanCreateMat(&FVec, FVector, MATTYPE_VECTOR, NB_TRAJ_STATE, (uint8)1);
    GDBKalmanCreateMat(&CMat, CMatrix, MATTYPE_VECTOR, (uint8)1, NB_TRAJ_STATE);

    /* sequential measurement updates for all relevant Targets */
    for (i = 0u; i < MAX_NB_TRAJ_SAMPLES; i++) {
        if ((pFUSED_GradSamples->valid[i]) != FALSE) {
            fxTgt = pFUSED_GradSamples->fx[i] + VLC_fBumperToCoG;
            fxTgt2 = fxTgt * fxTgt;

            /* Complete  Measurement Matrix C */
            CMat.pdData[VECT_MAT_INDEX(sTRAJ_C0)] = fxTgt;

            CMat.pdData[VECT_MAT_INDEX(sTRAJ_C1)] = fxTgt2 * 0.5f;

            fYNoise = MAX_FLOAT(CP_GRADUPDATE_STD_MIN,
                                pFUSED_GradSamples->fdydxMinStdDev[i]);
            fYNoise = SQR(fYNoise);

            /* Innovation */
            GDBKalmanMeasUpdate(&pTrajData->KafiEst, KMat, FVec, CMat,
                                pFUSED_GradSamples->fdydx[i], fYNoise);
        }
    }
}

/*************************************************************************************************************************
  Functionname:    CPKalmanPredictTrajectory */
void CPKalmanPredictTrajectory(const CPTrajectoryData_t *pTrajData) {
    GDBBaseMatrix_t WMat;
    GDBBaseMatrix_t DAMat;
    GDBBaseMatrix_t DAWVec;
    float32 WMatrix[NB_TRAJ_STATE * NB_TRAJ_STATE2];
    float32 DAMatrix[NB_TRAJ_STATE2];
    float32 DAWVector[NB_TRAJ_STATE2];

    /* Update A & Q Matrixes */
    CPKalmanSetTrajModelMat(pTrajData);

    /* Create intermediate Matrixes used for the time Update */
    GDBKalmanCreateMat(&WMat, WMatrix, MATTYPE_FULL, NB_TRAJ_STATE,
                       NB_TRAJ_STATE2);
    GDBKalmanCreateMat(&DAMat, DAMatrix, MATTYPE_DIAGONAL, NB_TRAJ_STATE2,
                       NB_TRAJ_STATE2);
    GDBKalmanCreateMat(&DAWVec, DAWVector, MATTYPE_VECTOR, NB_TRAJ_STATE2,
                       (uint8)1);

    /* Predict the actual state from the old estimated states */
    GDBKalmanTimeUpdate(&pTrajData->KafiEst, WMat, DAMat, DAWVec);
}

/*************************************************************************************************************************
  Functionname:    CPKalmanSetTrajModelMat */
static void CPKalmanSetTrajModelMat(const CPTrajectoryData_t *pTrajData) {
    /* Get Cycle Time */
    const float32 fTc = CP_CYCLE_TIME; /* in s   */

    float32 dx = fTc * EGO_SPEED_X_OBJ_SYNC;
    float32 dx2half = (dx * dx) / 2.0f;
    /*  float32 dx3sixth   = dx * dx2half / 3.0f;*/
    /*float32 dx4fourth  = dx2half * dx2half;*/
    /*float32 Tc2        = fTc * fTc;*/
    /*float32 Tc3        = fTc * Tc2;*/
    float32 temp;

    /* Update System Matrix A */

    /* [ 1  0   0.5.*dx2   dx3./6    dx  -0.5.*dx.*Tc ] */
    /* [ 0  1   0.5.*dx2   dx3./6    dx  -0.5.*dx.*Tc ] */
    /* [ 0  0   1          dx        0    0           ] */
    /* [ 0  0   0          1         0    0           ] */
    /* [ 0  0   dx         0.5.*dx2  1   -Tc          ] */
    /* [ 0  0   0          0         0    1           ] */

    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sY0L, sY0L, 1.0f             );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sY0L, sTRAJ_C0,  dx2half );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sY0L, sTRAJ_C1,  dx3sixth );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sY0L, sPSI, dx               );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sY0L, sYRf, -0.5f * dx * fTc );*/

    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sY0R, sY0R, 1.0f             );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sY0R, sTRAJ_C0,  dx2half );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sY0R, sTRAJ_C1,  dx3sixth );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sY0R, sPSI, dx               );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sY0R, sYRf, -0.5f * dx * fTc );*/

    GDBKalmanSetMat(&pTrajData->KafiEst.AMat, sTRAJ_C0, sTRAJ_C0, 1.0f);
    GDBKalmanSetMat(&pTrajData->KafiEst.AMat, sTRAJ_C0, sTRAJ_C1, dx);

    GDBKalmanSetMat(&pTrajData->KafiEst.AMat, sTRAJ_C1, sTRAJ_C1, 1.0f);

    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sPSI, sTRAJ_C0,  dx );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sPSI, sTRAJ_C1,  dx2half );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sPSI, sPSI, 1.0f             );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sPSI, sYRf, -fTc             );*/

    /*TPKalmanSetMat(&pTrajData->KafiEst.AMat, sYRf, sYRf, 1.0f             );*/

    /* Update System Noise Matrix Q */
    /*temp  = dx2half*NOISE_YRf_SQR*Tc3/6.0f;
    temp += ( NOISE_Y0_SQR + (dx3sixth * ( (2.0f*NOISE_Y0*NOISE_C1) +
    (dx3sixth*NOISE_C1_SQR) )) )*fTc;
    temp -= (NOISE_Y0+(dx3sixth*NOISE_C1))*NOISE_YRf*Tc2*dx*0.5f;*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.QMat, sY0L, sY0L, temp);*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.QMat, sY0R, sY0R, temp);*/
    temp = CP_TRAJ_NOISE_C1_SQR * fTc;
    GDBKalmanSetMat(&pTrajData->KafiEst.QMat, sTRAJ_C0, sTRAJ_C0,
                    2.0f * dx2half * temp);
    GDBKalmanSetMat(&pTrajData->KafiEst.QMat, sTRAJ_C1, sTRAJ_C1, temp);
    /*TPKalmanSetMat(&pTrajData->KafiEst.QMat, sPSI, sPSI,
     * (NOISE_YRf_SQR*Tc3/3.0f) + (dx4fourth*NOISE_C1_SQR*fTc) -
     * (dx2half*NOISE_C1*NOISE_YRf*Tc2) );*/
    /*TPKalmanSetMat(&pTrajData->KafiEst.QMat, sYRf, sYRf, fTc*NOISE_YRf_SQR);*/
}

/*************************************************************************************************************************
  Functionname:    CPKalmanGetTrajEstState */
float32 CPKalmanGetTrajEstState(const CPTrajectoryData_t *pTrajData,
                                uint8 uiVarState) {
    float32 fValue =
        GDBKalmanGetMat(&pTrajData->KafiEst.XVec, uiVarState, (uint8)0);

    return fValue;
}

/*************************************************************************************************************************
  Functionname:    CPKalmanGetTrajErrorCov */
float32 CPKalmanGetTrajErrorCov(const CPTrajectoryData_t *pTrajData,
                                uint8 uiVarState) {
    float32 fValue =
        GDBKalmanGetMat(&pTrajData->KafiEst.PDiagMat, uiVarState, uiVarState);

    return fValue;
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */