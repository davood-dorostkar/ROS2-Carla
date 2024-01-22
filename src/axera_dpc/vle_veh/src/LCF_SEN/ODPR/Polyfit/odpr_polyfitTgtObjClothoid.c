/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "TM_Global_Types.h"
#include "odpr_polyfitTgtObjClothoid.h"
#include "odpr_cml_mtrx.h"
#include "odpr_cml_adapted.h"  //include customized cml functions for matrix inversion with lower F32_ZERO constant
#include "tue_common_libs.h"
#include "odpr_cfg.h"
#include <string.h>
/*****************************************************************************
  MACROS
*****************************************************************************/
#ifndef FALSE
#define FALSE (0u)
#endif
#ifndef TRUE
#define TRUE (1u)
#endif

/* order+1 (number of coefficients) of fitted polynomial */
// #define POLY_ORDER_3RD (3u) // test without c1, only c0
// #define POLY_ORDER_1ST (2u)
// /* number of used data samples for polyfit */
// #define SAMPLE_POINTS (32u)

/*****************************************************************************
  LOCAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
STATIc boolean calculatePolyfit1st(ODPR_CML_t_Matrix* Weight,
                                   ODPR_CML_t_Matrix* mtrxX,
                                   ODPR_CML_t_Matrix* vecY,
                                   ODPR_CML_t_Matrix* vecPolyCoeff1st,
                                   float32 pWeightLastFit_nu,
                                   uint8 bLastTrajInvalid1st_bool);
STATIc boolean calculatePolyfit3rd(ODPR_CML_t_Matrix* Weight,
                                   ODPR_CML_t_Matrix* mtrxX,
                                   ODPR_CML_t_Matrix* vecY,
                                   ODPR_CML_t_Matrix* vecPolyCoeff3rd,
                                   float32 pWeightLastFit_nu,
                                   uint8 bLastTrajInvalid3rd_bool);
STATIc void createEquationMatrix1st(ODPR_CML_t_Matrix* mtrxX,
                                    ODPR_CML_t_Matrix* vecX);
STATIc void createEquationMatrix2nd(ODPR_CML_t_Matrix* mtrxX,
                                    ODPR_CML_t_Matrix* vecX,
                                    float32 pCrvDecay_nu);
STATIc void createEquationMatrix3rd(ODPR_CML_t_Matrix* mtrxX,
                                    ODPR_CML_t_Matrix* vecX,
                                    float32 pCrvDecay_nu,
                                    float32 pCrvChngDecay_nu);
STATIc void addNewSamplePoint(float32 fObjXPos_met,
                              float32 fObjYPos_met,
                              ODPR_CML_t_Matrix* vecX,
                              ODPR_CML_t_Matrix* vecY,
                              ODPR_CML_t_Matrix* Weight,
                              uint8* uNumValidEntries,
                              ODPR_CML_t_Matrix* vecAge);
STATIc void validatePoints(float32 fMinPosX_met,
                           float32 fMaxSampleAge_sec,
                           ODPR_CML_t_Matrix* vecX,
                           ODPR_CML_t_Matrix* Weight,
                           uint8* uNumValidEntries,
                           ODPR_CML_t_Matrix* vecAge);
STATIc void egoMotionCompensation(float32 fDeltaX_met,
                                  float32 fDeltaY_met,
                                  float32 fDeltaYawAng_rad,
                                  float32 fCycleTime_sec,
                                  ODPR_CML_t_Matrix* vecX,
                                  ODPR_CML_t_Matrix* vecY,
                                  ODPR_CML_t_Matrix* vecAge,
                                  ODPR_CML_t_Matrix* Weight);
STATIc void polyfitTransformation1st(float32 fDeltaY_met,
                                     float32 fDeltaYawAng_rad,
                                     ODPR_CML_t_Matrix* vecPolyCoeff1st);
STATIc void polyfitTransformation3rd(float32 fDeltaY_met,
                                     float32 fDeltaYawAng_rad,
                                     ODPR_CML_t_Matrix* vecPolyCoeff3rd);
STATIc float32 getMeanSampleAge(ODPR_CML_t_Matrix* vecAge,
                                ODPR_CML_t_Matrix* Weight,
                                uint8 uNumValidEntries);
STATIc float32 getMaxSampleAge(ODPR_CML_t_Matrix* vecAge,
                               ODPR_CML_t_Matrix* Weight);
STATIc float32 getMeanDevToTraj(ODPR_CML_t_Matrix* mtrxX,
                                ODPR_CML_t_Matrix* vecY,
                                ODPR_CML_t_Matrix* vecPolyCoeff,
                                ODPR_CML_t_Matrix* Weight,
                                uint8 uNumValidEntries);
STATIc float32 getMinSampleDist(ODPR_CML_t_Matrix* vecX,
                                ODPR_CML_t_Matrix* Weight);
STATIc float32 getMaxSampleDist(ODPR_CML_t_Matrix* vecX,
                                ODPR_CML_t_Matrix* Weight);
STATIc float32 getArcTanSmallAng(float32 x);

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/* ****************************************************************************
Functionname:    calculatePolyfit1st

@brief           calculates a polynomial fit 1st order using least square method

@description     the measured x-/y-object positions are stored and ego motion
compensated
                 using the driven delta ego distance and rotated delta ego yaw
angle
                 y = X * a --> a = (X^T * (W*X))^-1 * (X^T * (W*y))

@param[in]       Weight : weighting factor vector (diagonal matrix elements)

@param[in]       mtrxX : model equation matrix X

@param[in]       vecY : y sample points

@return          bMtrxInvFailed : FALSE if matrix inversion was successful

**************************************************************************** */
STATIc boolean calculatePolyfit1st(ODPR_CML_t_Matrix* Weight,
                                   ODPR_CML_t_Matrix* mtrxX,
                                   ODPR_CML_t_Matrix* vecY,
                                   ODPR_CML_t_Matrix* vecPolyCoeff1st,
                                   float32 pWeightLastFit_nu,
                                   uint8 bLastTrajInvalid1st_bool) {
    ODPR_CML_CreateMatrixLocal(
        mtrxX_T, POLY_ORDER_1ST,
        SAMPLE_POINTS)  // transposed X^T matrix using local memory allocation
        ODPR_CML_CreateMatrixLocal(mtrxX_I, POLY_ORDER_1ST,
                                   POLY_ORDER_1ST)  // inverted X^-1 matrix
                                                    // using local memory
                                                    // allocation
        ODPR_CML_CreateMatrixLocal(
            tmpMtrxWX, SAMPLE_POINTS,
            POLY_ORDER_1ST);  // temp (W*X) matrix using local memory allocation
    ODPR_CML_CreateMatrixLocal(tmpMtrxXTWX, POLY_ORDER_1ST,
                               POLY_ORDER_1ST)  // temp (X^T)*(W*X) matrix using
                                                // local memory allocation
        ODPR_CML_CreateMatrixLocal(
            tmpMtrxWy, SAMPLE_POINTS,
            1)  // temp (W*y) vector using local memory allocation
        ODPR_CML_CreateMatrixLocal(
            tmpMtrxXTWy, POLY_ORDER_1ST,
            1)  // temp (X^T)*(W*y) vector using local memory allocation
        ODPR_CML_CreateMatrixLocal(
            vecLastPolyCoeff, POLY_ORDER_1ST,
            1)  // temp vector used for weighting using local memory allocation

        uint8 row,
        col;
    boolean bMtrxInvFailed = FALSE;

    if (bLastTrajInvalid1st_bool == FALSE) {
        ODPR_CML_v_copyMatrix(vecLastPolyCoeff, vecPolyCoeff1st);
    }

    ODPR_CML_v_TransposeMatrix(mtrxX_T, mtrxX);

    for (row = 0; row < SAMPLE_POINTS; row++) {
        for (col = 0; col < POLY_ORDER_1ST; col++) {
            ODPR_CML_GetMatrixElement(tmpMtrxWX, row, col) =
                ODPR_CML_GetMatrixElement(Weight, row, 0) *
                ODPR_CML_GetMatrixElement(mtrxX, row, col);
        }
    }
    ODPR_CML_v_MultiplyMatrices(tmpMtrxXTWX, mtrxX_T, tmpMtrxWX);
    LCF_CML_v_InvertMatrix(mtrxX_I, tmpMtrxXTWX);  // from cml_adapted.c -->
                                                   // tmpMtrxXTWX gets
                                                   // overwritten with identity
                                                   // matrix

    if ((mtrxX_I->Desc.row == 0u) || (mtrxX_I->Desc.col == 0u)) {
        bMtrxInvFailed = TRUE;
    } else {
        for (row = 0; row < SAMPLE_POINTS; row++) {
            ODPR_CML_GetMatrixElement(tmpMtrxWy, row, 0) =
                ODPR_CML_GetMatrixElement(Weight, row, 0) *
                ODPR_CML_GetMatrixElement(vecY, row, 0);
        }
        ODPR_CML_v_MultiplyMatrices(tmpMtrxXTWy, mtrxX_T, tmpMtrxWy);

        ODPR_CML_v_MultiplyMatrices(vecPolyCoeff1st, mtrxX_I, tmpMtrxXTWy);

        /* apply weighting between new and old fit */
        if (bLastTrajInvalid1st_bool == FALSE) {
            /* row treated as col since it is a vec */
            for (row = 0; row < POLY_ORDER_1ST; row++) {
                ODPR_CML_GetMatrixElement(vecPolyCoeff1st, row, 0) =
                    (pWeightLastFit_nu *
                     ODPR_CML_GetMatrixElement(vecLastPolyCoeff, row, 0)) +
                    ((1.0f - pWeightLastFit_nu) *
                     ODPR_CML_GetMatrixElement(vecPolyCoeff1st, row, 0));
            }
        }
        bMtrxInvFailed = FALSE;
    }
    return bMtrxInvFailed;
}

/* ****************************************************************************
Functionname:    calculatePolyfit3rd

@brief           calculates a polynomial fit using least square method

@description     the measured x-/y-object positions are stored and ego motion
compensated
                 using the driven delta ego distance and rotated delta ego yaw
angle
                 y = X * a --> a = (X^T * (W*X))^-1 * (X^T * (W*y))

@param[in]       Weight : weighting factor vector (diagonal matrix elements)

@param[in]       mtrxX : model equation matrix X

@param[in]       vecY : y sample points

@return          bMtrxInvFailed : FALSE if matrix inversion was successful

**************************************************************************** */
STATIc boolean calculatePolyfit3rd(ODPR_CML_t_Matrix* Weight,
                                   ODPR_CML_t_Matrix* mtrxX,
                                   ODPR_CML_t_Matrix* vecY,
                                   ODPR_CML_t_Matrix* vecPolyCoeff3rd,
                                   float32 pWeightLastFit_nu,
                                   uint8 bLastTrajInvalid3rd_bool) {
    ODPR_CML_CreateMatrixLocal(
        mtrxX_T, POLY_ORDER_3RD,
        SAMPLE_POINTS)  // transposed X^T matrix using local memory allocation
        ODPR_CML_CreateMatrixLocal(mtrxX_I, POLY_ORDER_3RD,
                                   POLY_ORDER_3RD)  // inverted X^-1 matrix
                                                    // using local memory
                                                    // allocation
        ODPR_CML_CreateMatrixLocal(
            tmpMtrxWX, SAMPLE_POINTS,
            POLY_ORDER_3RD);  // temp (W*X) matrix using local memory allocation
    ODPR_CML_CreateMatrixLocal(tmpMtrxXTWX, POLY_ORDER_3RD,
                               POLY_ORDER_3RD)  // temp (X^T)*(W*X) matrix using
                                                // local memory allocation
        ODPR_CML_CreateMatrixLocal(
            tmpMtrxWy, SAMPLE_POINTS,
            1)  // temp (W*y) vector using local memory allocation
        ODPR_CML_CreateMatrixLocal(
            tmpMtrxXTWy, POLY_ORDER_3RD,
            1)  // temp (X^T)*(W*y) vector using local memory allocation
        ODPR_CML_CreateMatrixLocal(
            vecLastPolyCoeff, POLY_ORDER_3RD,
            1)  // temp vector used for weighting using local memory allocation

        uint8 row,
        col;
    boolean bMtrxInvFailed = FALSE;

    if (bLastTrajInvalid3rd_bool == FALSE) {
        ODPR_CML_v_copyMatrix(vecLastPolyCoeff, vecPolyCoeff3rd);
    }

    ODPR_CML_v_TransposeMatrix(mtrxX_T, mtrxX);

    for (row = 0; row < SAMPLE_POINTS; row++) {
        for (col = 0; col < POLY_ORDER_3RD; col++) {
            ODPR_CML_GetMatrixElement(tmpMtrxWX, row, col) =
                ODPR_CML_GetMatrixElement(Weight, row, 0) *
                ODPR_CML_GetMatrixElement(mtrxX, row, col);
        }
    }
    ODPR_CML_v_MultiplyMatrices(tmpMtrxXTWX, mtrxX_T, tmpMtrxWX);
    LCF_CML_v_InvertMatrix(mtrxX_I, tmpMtrxXTWX);  // from cml_adapted.c -->
                                                   // tmpMtrxXTWX gets
                                                   // overwritten with identity
                                                   // matrix

    if ((mtrxX_I->Desc.row == 0u) || (mtrxX_I->Desc.col == 0u)) {
        bMtrxInvFailed = TRUE;
    } else {
        for (row = 0; row < SAMPLE_POINTS; row++) {
            ODPR_CML_GetMatrixElement(tmpMtrxWy, row, 0) =
                ODPR_CML_GetMatrixElement(Weight, row, 0) *
                ODPR_CML_GetMatrixElement(vecY, row, 0);
        }
        ODPR_CML_v_MultiplyMatrices(tmpMtrxXTWy, mtrxX_T, tmpMtrxWy);

        ODPR_CML_v_MultiplyMatrices(vecPolyCoeff3rd, mtrxX_I, tmpMtrxXTWy);

        /* apply weighting between new and old fit */
        if (bLastTrajInvalid3rd_bool == FALSE) {
            /* row treated as col since it is a vec */
            for (row = 0; row < POLY_ORDER_3RD; row++) {
                ODPR_CML_GetMatrixElement(vecPolyCoeff3rd, row, 0) =
                    (pWeightLastFit_nu *
                     ODPR_CML_GetMatrixElement(vecLastPolyCoeff, row, 0)) +
                    ((1.0f - pWeightLastFit_nu) *
                     ODPR_CML_GetMatrixElement(vecPolyCoeff3rd, row, 0));
            }
        }
        bMtrxInvFailed = FALSE;
    }
    return bMtrxInvFailed;
}

/* ****************************************************************************
Functionname:    createEquationMatrix1st

@brief           create equation matrix X with model equations

@description     create equation matrix X with model equations for 3rd order

@param[in/out]   mtrxX : model equation matrix X (out --> updated)

@param[in]       vecX : x sample points

**************************************************************************** */
STATIc void createEquationMatrix1st(ODPR_CML_t_Matrix* mtrxX,
                                    ODPR_CML_t_Matrix* vecX) {
    // uint8 row, col;
    uint8 row;

    /*fill equation matrix X */
    for (row = 0; row < SAMPLE_POINTS; row++) {
        /*for(col = 0; col < POLY_ORDER; col++){
        }*/
        /* alternatively without second for-loop for columns to avoid CML
        dependency (CML_f_expPower and factorial) */
        ODPR_CML_GetMatrixElement(mtrxX, row, 0) = 1.0f;
        ODPR_CML_GetMatrixElement(mtrxX, row, 1) =
            ODPR_CML_GetMatrixElement(vecX, row, 0);
    }
}

/* ****************************************************************************
Functionname:    createEquationMatrix2nd

@brief           create equation matrix X with model equations

@description     create equation matrix X with model equations for 2nd order

@param[in/out]   mtrxX : model equation matrix X (out --> updated)

@param[in]       vecX : x sample points

**************************************************************************** */
STATIc void createEquationMatrix2nd(ODPR_CML_t_Matrix* mtrxX,
                                    ODPR_CML_t_Matrix* vecX,
                                    float32 pCrvDecay_nu) {
    // uint8 row, col;
    uint8 row;

    /*fill equation matrix X */
    for (row = 0; row < SAMPLE_POINTS; row++) {
        /*for(col = 0; col < POLY_ORDER; col++){
        }*/
        /* alternatively without second for-loop for columns to avoid CML
        dependency (CML_f_expPower and factorial) */
        ODPR_CML_GetMatrixElement(mtrxX, row, 0) = 1.0f;
        ODPR_CML_GetMatrixElement(mtrxX, row, 1) =
            ODPR_CML_GetMatrixElement(vecX, row, 0);
        /* use pCrvDecay_nu for damping of curvature */
        if (pCrvDecay_nu > 0.0001f) {
            ODPR_CML_GetMatrixElement(mtrxX, row, 2) =
                ODPR_CML_GetMatrixElement(vecX, row, 0) *
                ODPR_CML_GetMatrixElement(vecX, row, 0) * 0.5f / pCrvDecay_nu;
        } else {
            ODPR_CML_GetMatrixElement(mtrxX, row, 2) =
                ODPR_CML_GetMatrixElement(vecX, row, 0) *
                ODPR_CML_GetMatrixElement(vecX, row, 0) * 0.5f;
        }
    }
}

/* ****************************************************************************
Functionname:    createEquationMatrix3rd

@brief           create equation matrix X with model equations for 3rd order

@description     create equation matrix X with model equations for 3rd order

@param[in/out]   mtrxX : model equation matrix X (out --> updated)

@param[in]       vecX : x sample points

**************************************************************************** */
STATIc void createEquationMatrix3rd(ODPR_CML_t_Matrix* mtrxX,
                                    ODPR_CML_t_Matrix* vecX,
                                    float32 pCrvDecay_nu,
                                    float32 pCrvChngDecay_nu) {
    // uint8 row, col;
    uint8 row;

    /*fill equation matrix X */
    for (row = 0; row < SAMPLE_POINTS; row++) {
        /*for(col = 0; col < POLY_ORDER; col++){
        }*/
        /* alternatively without second for-loop for columns to avoid CML
        dependency (CML_f_expPower and factorial) */
        ODPR_CML_GetMatrixElement(mtrxX, row, 0) = 1.0f;
        ODPR_CML_GetMatrixElement(mtrxX, row, 1) =
            ODPR_CML_GetMatrixElement(vecX, row, 0);

        /* use pCrvDecay_nu for damping of curvature */
        if (pCrvDecay_nu > 0.0001f) {
            ODPR_CML_GetMatrixElement(mtrxX, row, 2) =
                ODPR_CML_GetMatrixElement(vecX, row, 0) *
                ODPR_CML_GetMatrixElement(vecX, row, 0) * 0.5f / pCrvDecay_nu;
        } else {
            ODPR_CML_GetMatrixElement(mtrxX, row, 2) =
                ODPR_CML_GetMatrixElement(vecX, row, 0) *
                ODPR_CML_GetMatrixElement(vecX, row, 0) * 0.5f;
        }
        /* use pCrvChngDecay_nu for damping of curvature change
        if(pCrvChngDecay_nu > 0.0001f){
            ODPR_CML_GetMatrixElement(mtrxX,row,3) =
        ODPR_CML_GetMatrixElement(vecX,row,0) *
        ODPR_CML_GetMatrixElement(vecX,row,0) *
        ODPR_CML_GetMatrixElement(vecX,row,0) / 6.0f / pCrvChngDecay_nu;
        }else{
            ODPR_CML_GetMatrixElement(mtrxX,row,3) =
        ODPR_CML_GetMatrixElement(vecX,row,0) *
        ODPR_CML_GetMatrixElement(vecX,row,0) *
        ODPR_CML_GetMatrixElement(vecX,row,0) / 6.0f;
        }*/
    }
}

/* ****************************************************************************
Functionname:    addNewSamplePoint

@brief           new sample points are added

@description     new sample points are added to the global ringbuffers[0],
                 previous values are shifted by 1 index

@param[in]       fObjXPos_met : ego yaw rate

@param[in]       fObjYPos_met : ego velocity

@param[in/out]   vecX : x sample points (out --> updated)

@param[in/out]   vecY : y sample points (out --> updated)

@param[in/out]   Weight : weighting factor vector (diagonal matrix elements)
(out --> updated)

@param[in/out]   vecAge : vector for stored sample point age (out --> updated)

**************************************************************************** */
STATIc void addNewSamplePoint(float32 fObjXPos_met,
                              float32 fObjYPos_met,
                              ODPR_CML_t_Matrix* vecX,
                              ODPR_CML_t_Matrix* vecY,
                              ODPR_CML_t_Matrix* Weight,
                              uint8* uNumValidEntries,
                              ODPR_CML_t_Matrix* vecAge) {
    float32 fDeltaYawAng_rad = 0.0;
    float32 fDeltaX_met = 0.0;
    float32 fDeltaY_met = 0.0;
    uint8 i;

    /* ringbuffer: shift all values forwards by 1 index, start from behind */
    for (i = SAMPLE_POINTS - 1; i > 0; i--) {
        ODPR_CML_GetMatrixElement(vecX, i, 0) =
            ODPR_CML_GetMatrixElement(vecX, i - 1, 0);
        ODPR_CML_GetMatrixElement(vecY, i, 0) =
            ODPR_CML_GetMatrixElement(vecY, i - 1, 0);
        ODPR_CML_GetMatrixElement(Weight, i, 0) =
            ODPR_CML_GetMatrixElement(Weight, i - 1, 0);
        ODPR_CML_GetMatrixElement(vecAge, i, 0) =
            ODPR_CML_GetMatrixElement(vecAge, i - 1, 0);
    }

    /* fill in new sample point at empty index 0 */
    ODPR_CML_GetMatrixElement(vecX, 0, 0) = fObjXPos_met;
    ODPR_CML_GetMatrixElement(vecY, 0, 0) = fObjYPos_met;
    ODPR_CML_GetMatrixElement(Weight, 0, 0) = 1.0f;
    ODPR_CML_GetMatrixElement(vecAge, 0, 0) = 0.0f;

    if (*uNumValidEntries < SAMPLE_POINTS) {
        *uNumValidEntries += 1;
    }
}

/* ****************************************************************************
Functionname:    validatePoints

@brief           new sample points are added

@description     new sample points are added to the global ringbuffers[0],
                 previous values are shifted by 1 index

@param[in]       fMinPosX_met : min object x-position

@param[in]       vecAge : vector for stored sample point age (out --> updated)

@param[in/out]   vecX : x sample points (out --> updated)

@param[in/out]   Weight : weighting factor vector (diagonal matrix elements)
(out --> updated)

**************************************************************************** */
STATIc void validatePoints(float32 fMinHistStartPosX_met,
                           float32 fMaxSampleAge_sec,
                           ODPR_CML_t_Matrix* vecX,
                           ODPR_CML_t_Matrix* Weight,
                           uint8* uNumValidEntries,
                           ODPR_CML_t_Matrix* vecAge) {
    uint8 i;

    /* check if stored points moved outside of valid area and set them to
     * invalid */
    for (i = 0; i < SAMPLE_POINTS; i++) {
        if ((ODPR_CML_GetMatrixElement(Weight, i, 0) > 0.0001f) &&
            ((ODPR_CML_GetMatrixElement(vecX, i, 0) < fMinHistStartPosX_met) ||
             (ODPR_CML_GetMatrixElement(vecAge, i, 0) > fMaxSampleAge_sec))) {
            ODPR_CML_GetMatrixElement(Weight, i, 0) = 0.0f;
            if (*uNumValidEntries > 0u) {
                *uNumValidEntries -= 1;
            }
        }
    }
}

/* ****************************************************************************
Functionname:    egoMotionCompensation

@brief           compensate ego motion between 2 cycles

@description     first fDeltaX_met and fDeltaY_met are subtracted,
                 then both points are rotated with fDeltaYawAng_rad

@param[in]       fEgoYawRate_rps : ego yaw rate

@param[in]       fEgoVelX_mps : ego velocity

@param[in]       fCycleTime_sec : cycle time

@param[in/out]   vecX : global x sample points (out --> updated)

@param[in/out]   vecY : global y sample points (out --> updated)

@param[in/out]   vecAge : vector for stored sample point age (out --> updated)

**************************************************************************** */
STATIc void egoMotionCompensation(float32 fDeltaX_met,
                                  float32 fDeltaY_met,
                                  float32 fDeltaYawAng_rad,
                                  float32 fCycleTime_sec,
                                  ODPR_CML_t_Matrix* vecX,
                                  ODPR_CML_t_Matrix* vecY,
                                  ODPR_CML_t_Matrix* vecAge,
                                  ODPR_CML_t_Matrix* Weight) {
    // ODPR_CML_CreateMatrixLocal(mRot, 2, 2) /*inverted X^-1 matrix using local
    // memory allocation*/
    // ODPR_CML_CreateMatrixLocal(mtrxXY, SAMPLE_POINTS, 2) /*create X matrix
    // with sample point equations using static memory allocation*/
    uint8 i;

    /* compute the inverse 2D rotation followed by translation */
    for (i = 0; i < SAMPLE_POINTS; i++) {
        if (ODPR_CML_GetMatrixElement(Weight, i, 0) > 0.0001f) {
            ODPR_CML_GetMatrixElement(vecX, i, 0) =
                (ODPR_CML_GetMatrixElement(vecX, i, 0) *
                 TUE_CML_GDBcos_52(fDeltaYawAng_rad)) +
                (ODPR_CML_GetMatrixElement(vecY, i, 0) *
                 TUE_CML_GDBsin_52(fDeltaYawAng_rad)) -
                fDeltaX_met;
            ODPR_CML_GetMatrixElement(vecY, i, 0) =
                (ODPR_CML_GetMatrixElement(vecY, i, 0) *
                 TUE_CML_GDBcos_52(fDeltaYawAng_rad)) -
                (ODPR_CML_GetMatrixElement(vecX, i, 0) *
                 TUE_CML_GDBsin_52(fDeltaYawAng_rad)) -
                fDeltaY_met;
            ODPR_CML_GetMatrixElement(vecAge, i, 0) += fCycleTime_sec;
        }
    }

    // alternatively to for-loop matrix multiplication could be used but
    // therefore additional XY matrix needed
    /*ODPR_CML_GetMatrixElement(mRot,0,0) = TUE_CML_GDBcos_52(fDeltaYawAng_rad);
    ODPR_CML_GetMatrixElement(mRot,0,1) = TUE_CML_GDBsin_52(fDeltaYawAng_rad) *
    -1.0;
    ODPR_CML_GetMatrixElement(mRot,1,0) = TUE_CML_GDBsin_52(fDeltaYawAng_rad);
    ODPR_CML_GetMatrixElement(mRot,1,1) = TUE_CML_GDBcos_52(fDeltaYawAng_rad);*/
}

/* ****************************************************************************
Functionname:    polyfitTransformation1st

@brief           compensate ego motion between 2 cycles

@description     first fDeltaX_met and fDeltaY_met are subtracted,
                 then both points are rotated with fDeltaYawAng_rad

@param[in]       fDeltaY_met : y-portion of ego motion

@param[in]       fDeltaYawAng_rad : yaw portion of ego motion

@param[in/out]   vecPolyCoeff1st : polyFit coefficients (out --> updated)

**************************************************************************** */
STATIc void polyfitTransformation1st(float32 fDeltaY_met,
                                     float32 fDeltaYawAng_rad,
                                     ODPR_CML_t_Matrix* vecPolyCoeff1st) {
    /* y' = y(t-1) - deltaY
       yaw' = yaw(t-1) - deltaYaw */
    ODPR_CML_GetMatrixElement(vecPolyCoeff1st, 0, 0) -= fDeltaY_met;
    ODPR_CML_GetMatrixElement(vecPolyCoeff1st, 1, 0) -= fDeltaYawAng_rad;
}

/* ****************************************************************************
Functionname:    polyfitTransformation3rd

@brief           compensate ego motion between 2 cycles

@description     first fDeltaX_met and fDeltaY_met are subtracted,
                 then both points are rotated with fDeltaYawAng_rad

@param[in]       fDeltaY_met : y-portion of ego motion

@param[in]       fDeltaYawAng_rad : yaw portion of ego motion

@param[in/out]   vecPolyCoeff1st : polyFit coefficients (out --> updated)

**************************************************************************** */
STATIc void polyfitTransformation3rd(float32 fDeltaY_met,
                                     float32 fDeltaYawAng_rad,
                                     ODPR_CML_t_Matrix* vecPolyCoeff3rd) {
    /* y' = y(t-1) - deltaY
       yaw' = yaw(t-1) - deltaYaw
       c0' = c0 (assumption due to robustness)
       c1' = c1 (assumption due to robustness) */
    ODPR_CML_GetMatrixElement(vecPolyCoeff3rd, 0, 0) -= fDeltaY_met;
    ODPR_CML_GetMatrixElement(vecPolyCoeff3rd, 1, 0) -= fDeltaYawAng_rad;
}

/* ****************************************************************************
Functionname:    getMeanSampleAge

@brief           compensate ego motion between 2 cycles

@description     first fDeltaX_met and fDeltaY_met are subtracted,
                 then both points are rotated with fDeltaYawAng_rad

@param[in]       fEgoYawRate_rps : ego yaw rate

@param[in]       fEgoVelX_mps : ego velocity

@param[in]       fCycleTime_sec : cycle time

@param[in/out]   vecX : global x sample points (out --> updated)

@param[in/out]   vecY : global y sample points (out --> updated)

**************************************************************************** */
STATIc float32 getMeanSampleAge(ODPR_CML_t_Matrix* vecAge,
                                ODPR_CML_t_Matrix* Weight,
                                uint8 uNumValidEntries) {
    float32 fMeanSampleAge = 0.0;
    uint8 i;

    if (uNumValidEntries > 0u) {
        for (i = 0; i < SAMPLE_POINTS; i++) {
            if (ODPR_CML_GetMatrixElement(Weight, i, 0) > 0.0001f) {
                fMeanSampleAge += ODPR_CML_GetMatrixElement(vecAge, i, 0);
            }
        }
        fMeanSampleAge /= uNumValidEntries;
    }
    return fMeanSampleAge;
}

/* ****************************************************************************
Functionname:    getMaxSampleAge

@brief           compensate ego motion between 2 cycles

@description     first fDeltaX_met and fDeltaY_met are subtracted,
                 then both points are rotated with fDeltaYawAng_rad

@param[in]       fEgoYawRate_rps : ego yaw rate

@param[in]       fEgoVelX_mps : ego velocity

@param[in]       fCycleTime_sec : cycle time

@param[in/out]   vecX : global x sample points (out --> updated)

@param[in/out]   vecY : global y sample points (out --> updated)

**************************************************************************** */
STATIc float32 getMaxSampleAge(ODPR_CML_t_Matrix* vecAge,
                               ODPR_CML_t_Matrix* Weight) {
    float32 fFirstSampleAge = 0.0f;
    uint8 i;

    for (i = 0; i < SAMPLE_POINTS; i++) {
        if (ODPR_CML_GetMatrixElement(Weight, i, 0) > 0.0001f) {
            if (ODPR_CML_GetMatrixElement(vecAge, i, 0) > fFirstSampleAge) {
                fFirstSampleAge = ODPR_CML_GetMatrixElement(vecAge, i, 0);
            }
        }
    }
    return fFirstSampleAge;
}

/* ****************************************************************************
Functionname:    getMeanDevToTraj

@brief           compensate ego motion between 2 cycles

@description     first fDeltaX_met and fDeltaY_met are subtracted,
                 then both points are rotated with fDeltaYawAng_rad

@param[in]       fEgoYawRate_rps : ego yaw rate

@param[in]       fEgoVelX_mps : ego velocity

@param[in]       fCycleTime_sec : cycle time

@param[in/out]   vecX : global x sample points (out --> updated)

@param[in/out]   vecY : global y sample points (out --> updated)

**************************************************************************** */
STATIc float32 getMeanDevToTraj(ODPR_CML_t_Matrix* mtrxX,
                                ODPR_CML_t_Matrix* vecY,
                                ODPR_CML_t_Matrix* vecPolyCoeff,
                                ODPR_CML_t_Matrix* Weight,
                                uint8 uNumValidEntries) {
    ODPR_CML_CreateMatrixLocal(
        tmpVecXa, SAMPLE_POINTS,
        1) /*temp (X*a) Vec using local memory allocation*/
        float32 fMeanDevTraj_met = 0.0;
    uint8 i;

    if (uNumValidEntries > 0u) {
        ODPR_CML_v_MultiplyMatrices(tmpVecXa, mtrxX, vecPolyCoeff);

        for (i = 0; i < SAMPLE_POINTS; i++) {
            if (ODPR_CML_GetMatrixElement(Weight, i, 0) > 0.0001f) {
                fMeanDevTraj_met +=
                    TUE_CML_Abs(ODPR_CML_GetMatrixElement(tmpVecXa, i, 0) -
                                ODPR_CML_GetMatrixElement(vecY, i, 0));
            }
        }
        fMeanDevTraj_met /= uNumValidEntries;
    }
    return fMeanDevTraj_met;
}

/* ****************************************************************************
Functionname:    getMinSampleDist

@brief           compensate ego motion between 2 cycles

@description     first fDeltaX_met and fDeltaY_met are subtracted,
                 then both points are rotated with fDeltaYawAng_rad

@param[in]       fEgoYawRate_rps : ego yaw rate

@param[in]       fEgoVelX_mps : ego velocity

@param[in]       fCycleTime_sec : cycle time

@param[in/out]   vecX : global x sample points (out --> updated)

@param[in/out]   vecY : global y sample points (out --> updated)

**************************************************************************** */
STATIc float32 getMinSampleDist(ODPR_CML_t_Matrix* vecX,
                                ODPR_CML_t_Matrix* Weight) {
    float32 fMinSampleX = 150.0f;
    uint8 i;

    for (i = 0; i < SAMPLE_POINTS; i++) {
        if (ODPR_CML_GetMatrixElement(Weight, i, 0) > 0.0001f) {
            if (ODPR_CML_GetMatrixElement(vecX, i, 0) < fMinSampleX) {
                fMinSampleX = ODPR_CML_GetMatrixElement(vecX, i, 0);
            }
        }
    }
    return fMinSampleX;
}

/* ****************************************************************************
Functionname:    getMaxSampleDist

@brief           compensate ego motion between 2 cycles

@description     first fDeltaX_met and fDeltaY_met are subtracted,
                 then both points are rotated with fDeltaYawAng_rad

@param[in]       fEgoYawRate_rps : ego yaw rate

@param[in]       fEgoVelX_mps : ego velocity

@param[in]       fCycleTime_sec : cycle time

@param[in/out]   vecX : global x sample points (out --> updated)

@param[in/out]   vecY : global y sample points (out --> updated)

**************************************************************************** */
STATIc float32 getMaxSampleDist(ODPR_CML_t_Matrix* vecX,
                                ODPR_CML_t_Matrix* Weight) {
    float32 fMaxSampleX = 0.0f;
    uint8 i;

    for (i = 0; i < SAMPLE_POINTS; i++) {
        if (ODPR_CML_GetMatrixElement(Weight, i, 0) > 0.0001f) {
            if (ODPR_CML_GetMatrixElement(vecX, i, 0) > fMaxSampleX) {
                fMaxSampleX = ODPR_CML_GetMatrixElement(vecX, i, 0);
            }
        }
    }
    return fMaxSampleX;
}

/* ****************************************************************************
Functionname:    getArcTanSmallAng
                 arctan approximation using equation 10 of
                 "Efficient Approximations for the Arctangent Function" with
                 smaller errors around 0 degree

**************************************************************************** */
STATIc float32 getArcTanSmallAng(float32 x) {
    float32 fArcTan = 0.0f;
    /* - 1 ≤ x ≤ 1 */
    if (x > 1.0f) {
        x = 1.0f;
    } else if (x < -1.0f) {
        x = -1.0f;
    }
    fArcTan = x / (1 + (0.28086f * x * x));
    return fArcTan;
}

/* ****************************************************************************
Functionname:    FOHPolyfitTgtObjClothoid */ /*!

@brief           Polyfit target object clothoid,calculates a polynomial fit
using
              least square method

@description     Polyfit target object clothoid. This function is corresponding
to
             sfun_polyfitTgtObjectClothoid module in MBD.

@param[in]       pTgtObjPFInput   Target object info. that input to perform
polyfit

@return          sTgtObjPFOutput  Target object info. after polyfit

**************************************************************************** */
void FOHPolyfitTgtObjClothoid(const FOHTgtObjPFInput_t* pInput,
                              FOHTgtObjPFOutput_t* pOutput,
                              ODPRFOHDebug_t* pFOHDebug) {
    ODPR_CML_CreateMatrixLocal(mtrxX1st, SAMPLE_POINTS,
                               POLY_ORDER_1ST)  // create X matrix with sample
                                                // point equations using static
                                                // memory allocation
        ODPR_CML_CreateMatrixLocal(mtrxX3rd, SAMPLE_POINTS,
                                   POLY_ORDER_3RD)     // create X matrix with
                                                       // sample point equations
                                                       // using static memory
                                                       // allocation
        ODPR_CML_CreateMatrix(vecY, SAMPLE_POINTS, 1)  // vector for object
                                                       // y-position storage
                                                       // which gets filled from
                                                       // input struct and
                                                       // transformed each cycle
        ODPR_CML_CreateMatrix(vecX, SAMPLE_POINTS, 1)  // vector for object
                                                       // x-position storage
                                                       // which gets filled from
                                                       // input struct and
                                                       // transformed each cycle
        ODPR_CML_CreateMatrix(vecPolyCoeff1st, POLY_ORDER_1ST,
                              1)  // calculated polynomial coefficients which
                                  // are written to the output struct
        ODPR_CML_CreateMatrix(vecPolyCoeff3rd, POLY_ORDER_3RD,
                              1)  // calculated polynomial coefficients which
                                  // are written to the output struct
        ODPR_CML_CreateMatrix(Weight, SAMPLE_POINTS,
                              1)  // weighting factors --> diagonal elements of
                                  // matrix stored as vector for element-wise
                                  // calculations
        ODPR_CML_CreateMatrix(vecAge, SAMPLE_POINTS,
                              1)  // vector for stored sample point age which
                                  // gets accumulated each cycle

        static uint8 uNumValidEntries = 0u;
    static uint8 bLastTrajInvalid1st_bool = TRUE;
    static uint8 bLastTrajInvalid3rd_bool = TRUE;

    float32 fDeltaYawAng_rad;
    float32 fDeltaX_met;
    float32 fDeltaY_met;
    boolean bPolyfitEn_bool = false;
    boolean bStartPointValid_bool = false;
    boolean bLengthValid_bool = false;

    /* small angle approximations: sin(ang) = ang | tan(ang) = ang | cos(ang) =
     * 1-(ang^2 / 2) */
    fDeltaYawAng_rad = pInput->fEgoYawRate_rps * pInput->fTimeSinceLastCall_sec;
    fDeltaX_met = pInput->fEgoVelX_mps * pInput->fTimeSinceLastCall_sec;
    fDeltaY_met = fDeltaX_met * fDeltaYawAng_rad;

    /* reset the global and output data */
    if (pInput->bReset_bool) {
        ODPR_CML_v_InitMatrix(mtrxX1st, SAMPLE_POINTS, POLY_ORDER_1ST, 0.0f);
        ODPR_CML_v_InitMatrix(mtrxX3rd, SAMPLE_POINTS, POLY_ORDER_3RD, 0.0f);
        ODPR_CML_v_InitMatrix(vecY, SAMPLE_POINTS, 1, 0.0f);
        ODPR_CML_v_InitMatrix(vecX, SAMPLE_POINTS, 1, 0.0f);
        ODPR_CML_v_InitMatrix(vecPolyCoeff1st, POLY_ORDER_1ST, 1, 0.0f);
        ODPR_CML_v_InitMatrix(vecPolyCoeff3rd, POLY_ORDER_3RD, 1, 0.0f);
        ODPR_CML_v_InitMatrix(Weight, SAMPLE_POINTS, 1, 0.0f);
        ODPR_CML_v_InitMatrix(vecAge, SAMPLE_POINTS, 1, 0.0f);

        uNumValidEntries = 0u;
        bLastTrajInvalid1st_bool = TRUE;
        bLastTrajInvalid3rd_bool = TRUE;

        pOutput->uNumValidSamples_nu = 0u;
        pOutput->fPosX0_met = 0.0f;
        pOutput->fPosY0_1st_met = 0.0f;
        pOutput->fPosY0_3rd_met = 0.0f;
        pOutput->fHeading_1st_rad = 0.0f;
        pOutput->fHeading_3rd_rad = 0.0f;
        pOutput->fCrv_1pm = 0.0f;
        pOutput->fChngOfCrv_1pm2 = 0.0f;
        pOutput->fTrajLength_met = 0.0f;
        pOutput->bTrajUpdate_bool = FALSE;
        pOutput->bTrajInvalid1st_bool = TRUE;
        pOutput->bTrajInvalid3rd_bool = TRUE;
        pOutput->fMeanSampleAge_sec = 0.0f;
        pOutput->fFirstStoredAge_sec = 0.0f;
        pOutput->fMeanDevToTraj_1st_met = 0.0f;
        pOutput->fMeanDevToTraj_3rd_met = 0.0f;
        pOutput->fLastStoredPntX_met = 0.0f;
        pOutput->fLastStoredPntY_met = 0.0f;
        pOutput->fLastStoredAge_sec = 0.0f;
    } else {
        if (uNumValidEntries > 0u) {
            egoMotionCompensation(fDeltaX_met, fDeltaY_met, fDeltaYawAng_rad,
                                  pInput->fTimeSinceLastCall_sec, vecX, vecY,
                                  vecAge, Weight);
            validatePoints(pInput->fMinHistStartPosX_met,
                           pInput->fMaxSampleAge_sec, vecX, Weight,
                           &uNumValidEntries, vecAge);
        }
        if (pInput->bAddNewSample_bool) {
            addNewSamplePoint(pInput->fObjXPos_met, pInput->fObjYPos_met, vecX,
                              vecY, Weight, &uNumValidEntries, vecAge);
        }
        if (bLastTrajInvalid1st_bool == FALSE) {
            polyfitTransformation1st(fDeltaY_met, fDeltaYawAng_rad,
                                     vecPolyCoeff1st);
        }
        if (bLastTrajInvalid3rd_bool == FALSE) {
            polyfitTransformation3rd(fDeltaY_met, fDeltaYawAng_rad,
                                     vecPolyCoeff3rd);
        }
        if (uNumValidEntries > 0u) {
            pOutput->fTrajLength_met = getMaxSampleDist(vecX, Weight);
            pOutput->fPosX0_met = getMinSampleDist(vecX, Weight);
        } else {
            pOutput->fTrajLength_met = 0.0f;
            pOutput->fPosX0_met = 0.0f;
        }

        pOutput->uNumValidSamples_nu = uNumValidEntries;
        pOutput->fLastStoredPntX_met = ODPR_CML_GetMatrixElement(vecX, 0, 0);
        pOutput->fLastStoredPntY_met = ODPR_CML_GetMatrixElement(vecY, 0, 0);
        pOutput->fLastStoredAge_sec = ODPR_CML_GetMatrixElement(vecAge, 0, 0);
        pOutput->fMeanSampleAge_sec =
            getMeanSampleAge(vecAge, Weight, uNumValidEntries);
        pOutput->fFirstStoredAge_sec = getMaxSampleAge(vecAge, Weight);

        bLengthValid_bool = (pOutput->fTrajLength_met - pOutput->fPosX0_met) >
                            pInput->fMinHistLength_met;
        bStartPointValid_bool =
            pOutput->fPosX0_met < pInput->fMaxGapEgoToHist_met;

        bPolyfitEn_bool =
            (uNumValidEntries >= pInput->uMinNumValidSamples_nu) &&
            (bStartPointValid_bool) && (bLengthValid_bool) &&
            (pInput->bEnable_bool);

        if (bPolyfitEn_bool) {
            pOutput->bTrajUpdate_bool = TRUE;

            /* calculate 1st order fit first */
            createEquationMatrix1st(mtrxX1st, vecX);
            bLastTrajInvalid1st_bool = calculatePolyfit1st(
                Weight, mtrxX1st, vecY, vecPolyCoeff1st,
                pInput->pWeightLastFit_nu, bLastTrajInvalid1st_bool);
            pOutput->bTrajInvalid1st_bool = bLastTrajInvalid1st_bool;
            if (bLastTrajInvalid1st_bool) {
                pOutput->fPosY0_1st_met = 0.0f;
                pOutput->fHeading_1st_rad = 0.0f;
                pOutput->fMeanDevToTraj_1st_met = 0.0f;
            } else {
                pOutput->fPosY0_1st_met =
                    ODPR_CML_GetMatrixElement(vecPolyCoeff1st, 0, 0);
                pOutput->fHeading_1st_rad = getArcTanSmallAng(
                    ODPR_CML_GetMatrixElement(vecPolyCoeff1st, 1, 0));
                pOutput->fMeanDevToTraj_1st_met = getMeanDevToTraj(
                    mtrxX1st, vecY, vecPolyCoeff1st, Weight, uNumValidEntries);
            }

            /* now calculate 3rd order fit*/
            createEquationMatrix3rd(mtrxX3rd, vecX, pInput->fCrvDecay_nu,
                                    pInput->fCrvChngDecay_nu);
            bLastTrajInvalid3rd_bool = calculatePolyfit3rd(
                Weight, mtrxX3rd, vecY, vecPolyCoeff3rd,
                pInput->pWeightLastFit_nu, bLastTrajInvalid3rd_bool);
            pOutput->bTrajInvalid3rd_bool = bLastTrajInvalid3rd_bool;
            if (bLastTrajInvalid3rd_bool) {
                pOutput->fPosY0_3rd_met = 0.0f;
                pOutput->fHeading_3rd_rad = 0.0f;
                pOutput->fCrv_1pm = 0.0f;
                pOutput->fChngOfCrv_1pm2 = 0.0f;
                // output->fChngOfCrv_1pm2 =
                // ODPR_CML_GetMatrixElement(vecPolyCoeff3rd,3,0);
                pOutput->fMeanDevToTraj_3rd_met = 0.0f;
            } else {
                pOutput->fPosY0_3rd_met =
                    ODPR_CML_GetMatrixElement(vecPolyCoeff3rd, 0, 0);
                pOutput->fHeading_3rd_rad = getArcTanSmallAng(
                    ODPR_CML_GetMatrixElement(vecPolyCoeff3rd, 1, 0));
                pOutput->fCrv_1pm =
                    ODPR_CML_GetMatrixElement(vecPolyCoeff3rd, 2, 0);
                pOutput->fChngOfCrv_1pm2 = 0.0f;
                // output->fChngOfCrv_1pm2 =
                // ODPR_CML_GetMatrixElement(vecPolyCoeff3rd,3,0);
                pOutput->fMeanDevToTraj_3rd_met = getMeanDevToTraj(
                    mtrxX3rd, vecY, vecPolyCoeff3rd, Weight, uNumValidEntries);
            }
        } else {
            pOutput->fPosY0_1st_met = 0.0f;
            pOutput->fPosY0_3rd_met = 0.0f;
            pOutput->fHeading_1st_rad = 0.0f;
            pOutput->fHeading_3rd_rad = 0.0f;
            pOutput->fCrv_1pm = 0.0f;
            pOutput->fChngOfCrv_1pm2 = 0.0f;
            pOutput->bTrajUpdate_bool = FALSE;
            pOutput->bTrajInvalid1st_bool = TRUE;
            pOutput->bTrajInvalid3rd_bool = TRUE;
            pOutput->fMeanDevToTraj_1st_met = 0.0f;
            pOutput->fMeanDevToTraj_3rd_met = 0.0f;
        }
    }

    /* Debug Output */
    memcpy(pFOHDebug->vecX, fMtrxDatavecX, sizeof(fMtrxDatavecX));
    memcpy(pFOHDebug->vecY, fMtrxDatavecY, sizeof(fMtrxDatavecY));
    memcpy(pFOHDebug->Weight, fMtrxDataWeight, sizeof(fMtrxDataWeight));
    memcpy(pFOHDebug->vecAge, fMtrxDatavecAge, sizeof(fMtrxDatavecAge));

    // printf("WJ:pFOHDebug->vecX[0] = %f\n",pFOHDebug->vecX[0]);

    pFOHDebug->bPolyfitEn_bool = bPolyfitEn_bool;
    pFOHDebug->bStartPointValid_bool = bStartPointValid_bool;
    pFOHDebug->bLengthValid_bool = bLengthValid_bool;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */