

/*****************************************************************************
  INCLUDES
*****************************************************************************/
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" 

#include "TM_Global_Types.h"
#include "TM_CML_Types.h"
#include "TM_Base_Bayes.h"
#include "TM_Base_Cfg.h"
#include "TM_Base_Complex.h"
#include "TM_Base_Const.h"
#include "TM_Base_Emul.h"
#include "TM_Base_Fourier.h"
#include "TM_Base_Kafi.h"
#include "TM_Base_Mapping.h"
#include "TM_Base_Mat.h"
#include "TM_Base_Misc.h"
#include "TM_Base_Mtrx.h"
#include "TM_Base_Stat.h"
#include "TM_Base_Trigo.h"
#include "TM_Base_Vector.h"
#include "TM_Base_Interpol.h"
#include "TM_Base_Ver.h"
#include "TM_Base_Ext.h"

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

void BML_v_KTU_CaldUDdW(const float32 f_Tol,
                        const uint8 u_NbOfState2,
                        float32 *f_Sum,
                        sint8 j,
                        uint32 u_Index1,
                        float32 *p_dUD,
                        float32 *p_dDAW,
                        float32 *p_dW) {
    uint8 k, l;
    uint32 u_Index2;
    float32 f_DsInv;
    if (BML_f_Abs(*f_Sum) < f_Tol) {
        /* Us(j,k) = 0 ................................................
         */
        for (k = 0u; k < (uint8)j; k++) {
            p_dUD[BML_UpriMatIndex(k, (uint8)j)] = 0.0F;
        }
    } else {
        f_DsInv = 1.0F / *f_Sum;
        u_Index2 = 0u; /* row index for W, start with 1st line */
        for (k = 0u; k < (uint8)j; k++) {
            /* Us(k,j) = (W(k,:)' * DA * W(j,:)) / Ds(j)
             * ................ */
            /*         = (W(k,:)' * DAW(j)) / Ds(j)
             * ..................... */
            *f_Sum = 0.0f;
            for (l = 0u; l < u_NbOfState2; l++) {
                *f_Sum += (p_dW[u_Index2 + (uint32)l] * p_dDAW[l]);
            }
            *f_Sum *= f_DsInv;
            p_dUD[BML_UpriMatIndex(k, (uint8)j)] =
                *f_Sum; /* store Us(k,j) in UDU' */

            /* W(k,:) = W(k,:) - Us(k,j) * W(j,:)
             * ....................... */
            for (l = 0u; l < u_NbOfState2; l++) {
                p_dW[u_Index2 + (uint32)l] -=
                    (*f_Sum * p_dW[u_Index1 + (uint32)l]);
            }

            u_Index2 += (uint32)u_NbOfState2; /* increment row index to begin of
                                                 next line of W */
        }
    }
}

/******************************************************************************
  Functionname:    BML_v_KalmanTimeUpdate                                */ /*!

  @brief           Update the states and the error covariance matrix to the actual time

  @description     This function does the time update part of the Kalman filter
                   It update the states and the error covariance matrix to the
                   actual time. The time update equations are responsible for 
                   projecting forward (in time) the current state and error 
                   covariance estimates to obtain the estimates for the next 
                   time step. With the transition matrix, factorized covariance 
                   it calculates the new predicted state vector and an updated
                   covariance matrix.

  @param[in]       p_GDBKafi :  pointer on the Kalman filter static structures
                                Supported values for p_GDBKafi->uiNbOfStates [0,..,127]
                                Supported values for p_GDBKafi->AMat.pdData
                                [Valid float32 pointer]
                                Supported values for p_GDBKafi->QMat.pdData
                                [Valid float32 pointer]
                                Supported values for p_GDBKafi->XVec.pdData
                                [Valid float32 pointer]
                                Supported values for p_GDBKafi->UDMat.pdData
                                [Valid float32 pointer]
                                Supported values for p_GDBKafi->XsVec.pdData
                                [Valid float32 pointer]
  @param[in]       WMat :       temporary Matrix W = [W1 W2] = [A*UD Id]
                                Supported values for WMat.pdData
                                [Valid float32 pointer]
  @param[in]       DAMat :      temporary Matrix DA = [Diag(UD, Q)]
                                Supported values for DAMat.pdData
                                [Valid float32 pointer]
  @param[in]       DAWVec :     temporary Vector DAW = W'*DA*W
                                Supported values for DAWVec.pdData
                                [Valid float32 pointer]

  @return          void


*****************************************************************************/
void BML_v_KalmanTimeUpdate(BML_t_BaseKafi const *const p_GDBKafi,
                            BML_t_KafiMatrix WMat,
                            BML_t_KafiMatrix DAMat,
                            BML_t_KafiMatrix DAWVec) {
    /*--- VARIABLES ---*/
    sint8 j;
    uint8 m, n, k, l;
    const uint8 u_NbOfState2 = (uint8)(2u * p_GDBKafi->uiNbOfStates);
    const float32 f_Tol = 1e-10F; /* tolerance */
    uint32 u_Index1, u_Index2;
    float32 f_Sum;
    float32 *p_dXs, *p_dX;
    float32 *p_dW, *p_dDA, *p_dDAW;
    float32 *p_dA, *p_dQ, *p_dUD;

    /* state prediction xs = Phi * xd */
    p_dXs = p_GDBKafi->XsVec.pdData;
    p_dX = p_GDBKafi->XVec.pdData;
    p_dA = p_GDBKafi->AMat.pdData;

    for (m = 0u; m < p_GDBKafi->uiNbOfStates; m++) /* for all lines of X */
    {
        p_dXs[m] = 0.0f;
        for (n = 0u; n < p_GDBKafi->uiNbOfStates;
             n++) /* for all columns of Phi */
        {
            p_dXs[m] += (p_dA[n] * p_dX[n]);
        }
        p_dA += p_GDBKafi->uiNbOfStates; /* proceed with next line of Phi */
    }
    for (m = 0u; m < p_GDBKafi->uiNbOfStates; m++) /* for all lines of X */
    {
        p_dX[m] = p_dXs[m];
    }

    /* VI.4, S.124ff und Appendix VI.A, S.131ff) */
    /* [in] : Ud, Dd */
    /* [out]: Us, Ds */
    /*                                                                                                      */
    /* W  = [ Phi*Ud QU ]                                (4.3)  Bierman G ist QU
     * hier ist I, da Q diagonal  */
    /* DA = Diag(Dd, QD)                                 (4.4)  Bierman Q ist QD
     * hier ist Q                 */
    /*                                                                                                      */
    /* for j=n-1,..1                                            Bierman(Fortran)
     * j+1 ist j hier da C        */
    /*    Ds(j) = W(j,:)' * DA * W(j,:)                  (4.9)  W(j,:) ist j.
     * Zeile von W                   */
    /*                                                          w' D v = w1d1v1
     * + w2d2v2 + ... + wndnvn     */
    /*    for k=0,..,j-1 */
    /*      Us(k,j) = (W(k,:)' * W(j,:)) * DA / Ds(j)    (4.10) */
    /*      W(k,:) = W(k,:) - Us(k,j) * W(j,:)           (4.11) Aenderung der k.
     * Zeile von W                */
    /*                                                                                                      */
    /* Ds(0) = W(0,:)' * DA * W(0,:)                     (4.12) */

    p_dW = WMat.pdData;
    p_dDA = DAMat.pdData;
    p_dDAW = DAWVec.pdData;
    p_dA = p_GDBKafi->AMat.pdData;
    p_dQ = p_GDBKafi->QMat.pdData;
    p_dUD = p_GDBKafi->UDMat.pdData;

    /* W = [W1 W2] = [Phi*Ud QU] ........................................ */
    for (m = 0u; m < p_GDBKafi->uiNbOfStates; m++) {
        u_Index1 = 0u; /* column index of Ud */

        /* W2 = QU = I .................................................... */
        for (k = p_GDBKafi->uiNbOfStates; k < u_NbOfState2;
             k++) /* set right half of line to 0 */
        {
            p_dW[k] = 0.0;
        }
        p_dW[p_GDBKafi->uiNbOfStates + m] =
            1.0; /* set diagonal element of right half */

        /* W1 = Phi*Ud .................................................... */
        for (k = 0u; k < p_GDBKafi->uiNbOfStates; k++) {
            u_Index1 += (uint32)k; /* goto column k in UD */
            /* Bestimme W1[j,k] ............................................. */
            f_Sum = p_dA[k]; /* Phi(j,k) * U(j,j) */
            /* while U(j,j) = 1 by definition */
            for (l = 0u; l < k; l++) {
                f_Sum += (p_dA[l] * p_dUD[u_Index1 + (uint32)l]);
            }
            p_dW[k] = f_Sum;
        }
        p_dA += p_GDBKafi->uiNbOfStates; /* goto next line of Phi */
        p_dW += u_NbOfState2;            /* goto next line of W   */
    }

    /* DA = Diag(UD, Q) ................................................ */
    u_Index2 = 0u;
    for (m = 0u; m < p_GDBKafi->uiNbOfStates; m++) {
        u_Index2 += (uint32)m;
        p_dDA[m] = p_dUD[u_Index2];
        p_dDA[p_GDBKafi->uiNbOfStates + m] = p_dQ[m];
        u_Index2++;
    }

    /* .................................................................. */
    p_dW = WMat.pdData;

    u_Index1 =
        (uint32)(p_GDBKafi->uiNbOfStates) *
        (uint32)(u_NbOfState2); /* row index for W, start with last line */
    for (j = (sint8)p_GDBKafi->uiNbOfStates - 1; j >= 0; j--) {
        u_Index1 -= (uint32)u_NbOfState2; /* decrement row index to begin of
                                             previous line of W */
        /* Ds(j) = W(j,:)' * DA * W(j,:) .................................. */
        f_Sum = 0.0F;
        for (k = 0u; k < u_NbOfState2; k++) {
            p_dDAW[k] = p_dDA[k] * p_dW[u_Index1 + (uint32)k];
            f_Sum += (p_dW[u_Index1 + (uint32)k] * p_dDAW[k]);
        }

        p_dUD[BML_UpriMatIndex((uint8)j, (uint8)j)] =
            f_Sum; /* store Ds(j) in UDU' */

        if (j != 0) {
            BML_v_KTU_CaldUDdW(f_Tol, u_NbOfState2, &f_Sum, j, u_Index1, p_dUD,
                               p_dDAW, p_dW);
        }
    } /* for j */

    /* Ds(0) = W(0,:)' * DA * W(0,:) .................................... */
    f_Sum = 0.0f;
    for (k = 0u; k < u_NbOfState2; k++) {
        f_Sum += (p_dW[k] * p_dW[k] * p_dDA[k]);
    }

    p_dUD[0] = f_Sum; /* store Ds(0,0) in UDU' */
}

/*****************************************************************************
  Functionname:    CML_v_KalmanMeasUpdate                               */ /*!

  @brief           Calculate the Kalman gain, estimate the states and
                   update the error covariance matrix

  @description     This function does the measurement update part of the 
                   Kalman filter. It calculate the Kalman gain, estimate the 
                   states and update the error covariance matrix.
                   The measurement update equations are responsible for the 
                   feedback, for incorporating a new measurement to obtain an
                   improved estimate. It takes in the measurement matrix and 
                   the prior estiate values to do the correction for the 
                   better estimate.
                   
  @param[in]       p_GDBKafi :  pointer on the Kalman filter static structures
                                Supported values for p_GDBKafi->uiNbOfStates [0,..,127]
                                Supported values for p_GDBKafi->XVec.pdData
                                [Valid float32 pointer]
                                Supported values for p_GDBKafi->UDMat.pdData
                                [Valid float32 pointer]
  @param[in]       KMat :       Kalman gain matrix (Due to sequential updates
                                is here a vector whose length = Nb of states)
                                Supported values for KMat.pdData
                                [Valid float32 pointer]
  @param[in]       FVec :       temporary vector (vector length = Nb of states)
                                Supported values for FVec.pdData
                                [Valid float32 pointer]
  @param[in]       CMat :       Measurement Matrix (Due to sequential updates
                                is here a vector whose length = Nb of states)
                                Supported values for CMat.pdData
                                [Valid float32 pointer]
  @param[in]       f_Meas :     Measure (dY)
                                Supported values [Full range of float32]
  @param[in]       f_MeasNoise :  noise associated with the Measure
                                  Supported values [Full range of float32]
                                  Overflow may occur at the extreme values of
                                  the range

  @return          void


*****************************************************************************/
void CML_v_KalmanMeasUpdate(BML_t_BaseKafi const *const p_GDBKafi,
                            BML_t_KafiMatrix KMat,
                            BML_t_KafiMatrix FVec,
                            BML_t_KafiMatrix CMat,
                            float32 f_Meas,
                            float32 f_MeasNoise) {
    uint8 i, j;
    uint32 u_Index;
    float32 f_alfa, f_beta, f_gamma1, f_delta, f_lambda, f_sum;

    /* Note from gnu man page (math.h): `gamma'  calculates  the  natural
     * logarithm of the gamma function of X. */
    float32 *p_dC, *p_dX, *p_dK;
    float32 *p_dUD, *p_dF;

    /* Assign pointers */
    p_dK = KMat.pdData;
    p_dF = FVec.pdData;
    p_dUD = p_GDBKafi->UDMat.pdData;
    p_dX = p_GDBKafi->XVec.pdData;
    p_dC = CMat.pdData;

    /* V.3, S.76ff und Appendix V.A, S.100ff */
    /* [in] : Us, Ds */
    /* [out]: Ud, Dd, dx */
    /*                                                                                                      */
    /* F = U' * c; K = D * F                                    (3.1) */
    /* alfa = r + K(0) * F(0)                                   (3.2) */
    /* gamma1 = 1 / alfa */
    /* D(0) = D(0) * r * gamma1 */
    /* for j = 1 ... n-1 */
    /*   beta = alfa */
    /*   alfa = alfa + K(j) * F(j)                              (3.3) */
    /*   lamba = -F(j) * gamma1                                 (3.5) */
    /*   gamma1 = 1 / alfa */
    /*   D(j) = D(j) * beta * gamma1                            (3.4) */
    /*   U(:,j) = U(:,j) + lamda * K */
    /*   K = K + K(j) * U(:,j)                                  (3.6) */
    /*                                                                                                      */
    /* K = K * gamma1                                           (3.7) */
    /* dx = dx + K * dy */

    /* dy = y - C * dx ......................................... */
    for (j = 0u; j < p_GDBKafi->uiNbOfStates; j++) {
        f_Meas -= (p_dC[j] * p_dX[j]);
    }

    /* K[] = D[][] U'[][] C'[] und F'[] = C[] U[][] */
    u_Index = ((((uint32)p_GDBKafi->uiNbOfStates - 1u) *
                (uint32)p_GDBKafi->uiNbOfStates) /
               2u); /* init column index of U to last column */
    for (j = (uint8)(p_GDBKafi->uiNbOfStates - 1u); j > 0u; j--) {
        f_alfa = p_dC[j];
        for (i = 0u; i < j; i++) {
            f_alfa += (p_dUD[u_Index + (uint32)i] * p_dC[i]);
        }
        p_dF[j] = f_alfa;
        p_dK[j] = p_dUD[u_Index + (uint32)j] * f_alfa;
        u_Index -= ((uint32)j);
    }

    p_dF[0] = p_dC[0];
    p_dK[0] = p_dUD[0] * p_dC[0];

    /* update U and D matrix, compute gains ............................. */
    f_alfa = BML_f_MultAdd(p_dK[0], p_dF[0], f_MeasNoise);

    f_gamma1 = (f_alfa > 0.0F) ? (1.0F / f_alfa) : 0.0F;

    if (BML_f_IsNonZero(p_dK[0])) {
        p_dUD[0] *= (f_MeasNoise * f_gamma1);
    }

    u_Index = 0u;
    for (j = 1u; j < p_GDBKafi->uiNbOfStates; j++) {
        f_beta = f_alfa;
        f_delta = p_dK[j];
        f_alfa += f_delta * p_dF[j];

        if (BML_f_IsNonZero(f_alfa)) {
            f_lambda = -p_dF[j] * f_gamma1;
            for (i = 0u; i < j; i++) {
                u_Index++;
                f_sum = p_dUD[u_Index];
                p_dUD[u_Index] = BML_f_MultAdd(f_lambda, p_dK[i], f_sum);
                p_dK[i] += (f_delta * f_sum);
            }
            u_Index++;
            f_gamma1 = 1.0F / f_alfa;
            p_dUD[u_Index] *= (f_beta * f_gamma1);
        }
    }

    for (j = 0u; j < p_GDBKafi->uiNbOfStates; j++) {
        p_dK[j] *= f_gamma1;
        p_dX[j] += (p_dK[j] * f_Meas);
    }
}

/*****************************************************************************
  Functionname:    CML_v_KalmanPDiagUpdate                              */ /*!

  @brief           Calculate the diagonal of the P Matrix

  @description     This function calculate the diagonal matrix P for covariance
                   using the UDUT factorized matrix. The values are calculated
                   as follows:

                   P(i,i) = D(i,i) + D(i+1,i+1)*U(i,i+1)^2 + 
                                     D(i+2,i+2)*U(i,i+2)^2 + ...

  @param[in]       p_GDBKafi :  pointer on the Kalman filter static structures
                                Supported values for p_GDBKafi->uiNbOfStates
                                [Full range of uint8]
                                Supported values for p_GDBKafi->PDiagMat.pdData
                                [Valid float32 pointer]
                                Supported values for p_GDBKafi->UDMat.pdData
                                [Valid float32 pointer]

  @return          void

  @pre             the UD matrix has been updated


*****************************************************************************/
void CML_v_KalmanPDiagUpdate(BML_t_BaseKafi const *const p_GDBKafi) {
    uint8 i, j;
    float32 *p_PDiag = p_GDBKafi->PDiagMat.pdData;
    float32 *p_UD = p_GDBKafi->UDMat.pdData;

    for (i = 0u; i < p_GDBKafi->uiNbOfStates; i++) {
        p_PDiag[i] = p_UD[BML_UpriMatIndex(i, i)];

        /* P(i,i) = D(i,i) + D(i+1,i+1)*U(i,i+1)^2 + D(i+2,i+2)*U(i,i+2)^2 + ...
         */
        for (j = i + 1u; j < p_GDBKafi->uiNbOfStates; j++) {
            p_PDiag[i] +=
                (p_UD[BML_UpriMatIndex(j, j)] * p_UD[BML_UpriMatIndex(i, j)] *
                 p_UD[BML_UpriMatIndex(i, j)]);
        }
    }
}

/*****************************************************************************
  Functionname:    BML_v_KalmanCreatEnvmat                                */ /*!

  @brief           Initialise the static matrices required for the Kalman Filter

  @description     This function initialise the static matrices required for 
                   the Kalman Filter. It calculates the the number of elements
                   based on the matrix type and given number of rows and columns
                   Also it assigns allocated memory to the matrix elements and 
                   initializes all matrix elements to zero. Also it assigns other 
                   given value provided the calculation for number of elements is 
                   successful.

  @param[in]       p_Data :     Pointer to already allocated data
                                [Valid float32 pointer]
  @param[in]       MatType :    Type of the Matrix (full, triangular...)
                                Supported values [0,.,4]
  @param[in]       u_Rows :     num of rows in the matrix
                                Supported values [0,..,254]
  @param[in]       u_Columns :  num of columns in the matrix
                                Supported values [0,..,255]
  @param[out]      p_matM :     Pointer to matrix structure
                                Supported values for p_matM.matType
                                Supported values [0,.,4]
                                Supported values for p_matM.uiRows
                                Supported values [0,..,255]
                                Supported values for p_matM.uiColumns
                                Supported values [0,..,255]
                                Supported values for p_matM.uiElements
                                Supported values [Full range of uint16]
                                Supported values for p_matM.pdData
                                [Valid float32 pointer]

  @return          void

  @pre             The required memory for the matrix has been allocated


*****************************************************************************/
void BML_v_KalmanCreatEnvmat(BML_t_KafiMatrix *p_matM,
                             float32 *p_Data,
                             BML_t_KafiMatrixType MatType,
                             uint8 u_Rows,
                             uint8 u_Columns) {
    boolean bError = FALSE;
    uint16 i;
    uint8 u_tmp8;

    /* calc number of elements */
    switch (MatType) {
        case CML_KafiMatrixTypeVector:
            if ((u_Rows > 1u) && (u_Columns > 1u)) {
                bError = TRUE;
            } else {
                p_matM->uiElements = (uint16)u_Rows * (uint16)u_Columns;
            }
            break;
        case BML_KafiMatrixTypeFull:
            p_matM->uiElements = (uint16)u_Rows * (uint16)u_Columns;
            break;
        case BML_KafiMatrixTypeSymmetric:
            if (u_Rows != u_Columns) {
                bError = TRUE;
            } else {
                u_tmp8 = u_Rows + 1u;
                p_matM->uiElements = (((uint16)u_Rows * (uint16)u_tmp8) / 2u);
            }
            break;
        case CML_KafiMatrixTypeUpperRight:
            if (u_Rows != u_Columns) {
                bError = TRUE;
            } else {
                u_tmp8 = u_Rows + 1u;
                p_matM->uiElements = (((uint16)u_Rows * (uint16)u_tmp8) / 2u);
            }
            break;
        case BML_KafiMatrixTypeDiagonal:
            if (u_Rows != u_Columns) {
                bError = TRUE;
            } else {
                p_matM->uiElements = (uint16)u_Rows;
            }
            break;
        default:
            bError = TRUE;
            break;
    }

    if (p_matM->uiElements < (uint16)1) {
        bError = TRUE;
    }

    if (bError == FALSE) {
        /* allocate memory for the elements of the matrix */
        p_matM->pdData = p_Data;
        p_matM->matType = MatType;
        p_matM->uiColumns = u_Columns;
        p_matM->uiRows = u_Rows;

        for (i = 0u; i < p_matM->uiElements; i++) {
            p_matM->pdData[i] = 0.0f;
        }
    }
}

/*****************************************************************************
  Functionname:    BML_v_KalmanSetMat                                   */ /*!

  @brief           Alter a static matrix used by the Kalman filter

  @description     This function alters a static matrix used by the Kalman 
                   filter. Given value is updated in the specified location in 
                   the matrix defined by the row and column depending on the 
                   type of matrix.

  @param[in,out]   p_matM :   Matrix to be modified
                              Supported values for p_matM.matType
                              Supported values [0,.,4]
                              Supported values for p_matM.uiRows
                              Supported values [0,..,255]
                              Supported values for p_matM.uiColumns
                              Supported values [0,..,255]
                              Supported values for p_matM.uiElements
                              Supported values [Full range of uint16]
                              Supported values for p_matM.pdData
                              [Valid float32 pointer]
  @param[in]       u_Row :    Position in the matrix (Row)
                              Supported values [0,..,255]
  @param[in]       u_Column : Position in the matrix (Column)
                              Supported values [0,..,255]
  @param[in]       f_Value :  Value to be set
                              Supported values [Full range of float32]

  @return          void


*****************************************************************************/
void BML_v_KalmanSetMat(const BML_t_KafiMatrix *p_matM,
                        uint8 u_Row,
                        uint8 u_Column,
                        float32 f_Value) {
    /* check range of row and column index */
    if ((u_Row < p_matM->uiRows) && (u_Column < p_matM->uiColumns)) {
        switch (p_matM->matType) {
            case CML_KafiMatrixTypeVector:
                p_matM->pdData[BML_VectMatIndex(u_Row + u_Column)] = f_Value;
                break;
            case BML_KafiMatrixTypeFull:
                p_matM->pdData[BML_FullMatIndex(u_Row, u_Column,
                                                p_matM->uiColumns)] = f_Value;
                break;
            case BML_KafiMatrixTypeSymmetric:
                if (u_Row <= u_Column) {
                    p_matM->pdData[BML_SymmMatIndex(u_Row, u_Column)] = f_Value;
                } else {
                    p_matM->pdData[BML_SymmMatIndex(u_Column, u_Row)] = f_Value;
                }
                break;
            case CML_KafiMatrixTypeUpperRight:
                if (u_Row <= u_Column) {
                    p_matM->pdData[BML_UpriMatIndex(u_Row, u_Column)] = f_Value;
                }
                break;
            case BML_KafiMatrixTypeDiagonal:
                if (u_Row == u_Column) {
                    p_matM->pdData[BML_DiagMatrIndex(u_Row)] = f_Value;
                }
                break;
            default:
                /* Nothing To do - Misra Compatibility */
                break;
        }
    }
}

/*****************************************************************************
  Functionname:    BML_f_KalmanGetMatElEnvment                            */ /*!

  @brief           Get an element of a matrix used by the Kalman filter

  @description     This function gets an element of a matrix used by the 
                   Kalman filter.Required value is taken from the specified  
                   location in the matrix defined by the row and column 
                   depending on the type of matrix.

  @param[in]       p_matM :    the Matrix to be read
                               Supported values for p_matM.matType
                               Supported values [0,.,4]
                               Supported values for p_matM.uiRows
                               Supported values [0,..,255]
                               Supported values for p_matM.uiColumns
                               Supported values [0,..,255]
                               Supported values for p_matM.uiElements
                               Supported values [Full range of uint16]
                               Supported values for p_matM.pdData
                               [Valid float32 pointer]
  @param[in]       u_Row :     Position in the matrix (row)
                               Supported values [0,..,255]
  @param[in]       u_Column :  Position in the matrix (column)
                               Supported values [0,..,255]

  @return          The requested element of the matrix


*****************************************************************************/
float32 BML_f_KalmanGetMatElEnvment(const BML_t_KafiMatrix *p_matM,
                                    uint8 u_Row,
                                    uint8 u_Column) {
    float32 f_Value = 0.0f;

    if ((u_Row < p_matM->uiRows) && (u_Column < p_matM->uiColumns)) {
        switch (p_matM->matType) {
            case CML_KafiMatrixTypeVector:
                f_Value = p_matM->pdData[BML_VectMatIndex(u_Row + u_Column)];
                break;
            case BML_KafiMatrixTypeFull:
                f_Value = p_matM->pdData[BML_FullMatIndex(u_Row, u_Column,
                                                          p_matM->uiColumns)];
                break;
            case BML_KafiMatrixTypeSymmetric:
                if (u_Row <= u_Column) {
                    f_Value = p_matM->pdData[BML_SymmMatIndex(u_Row, u_Column)];
                } else {
                    f_Value = p_matM->pdData[BML_SymmMatIndex(u_Column, u_Row)];
                }
                break;
            case CML_KafiMatrixTypeUpperRight:
                if (u_Row <= u_Column) {
                    f_Value = p_matM->pdData[BML_UpriMatIndex(u_Row, u_Column)];
                } else {
                    f_Value = 0.0f;
                }
                break;
            case BML_KafiMatrixTypeDiagonal:
                if (u_Row == u_Column) {
                    f_Value = p_matM->pdData[BML_DiagMatrIndex(u_Row)];
                } else {
                    f_Value = 0.0f;
                }
                break;
            default:
                f_Value = 0.0f;
                break;
        }
    }

    return f_Value;
}

/*****************************************************************************
  Functionname:    BML_u_SimpleKafiInit                                 */ /*!

  @brief           initializes the Kalman filter parameters for given 
                   kafi->z and kafi->R

  @description     This function initializes the Kalman filter parameters 
                   for given kafi->z and kafi->R. With the given observation
                   model, noise covariance and vector, it computes the 
                   Kalman gain, error covariance and estimate vector. All the
                   required parameters are taken and updated within the Kalman
                   filter structure.

  @param[in,out]   p_kafi :  a simple Kalman filter structure
                             Supported values for p_kafi->x_y 
                             [Full range of uint8]
                             Supported values for p_kafi->z
                             [Valid float32 pointer]
                             Supported values for p_kafi->H
                             [Valid float32 pointer]
                             Supported values for p_kafi->R
                             [Valid float32 pointer]
                             Supported values for p_kafi->P
                             [Valid float32 pointer]
                             Supported values for p_kafi->x
                             [Valid float32 pointer]
                             Supported values for p_kafi->K
                             [Valid float32 pointer]
                             Supported values for p_kafi->T
                             [Valid float32 pointer]
                             Supported values for p_kafi->T2
                             [Valid float32 pointer]

  @return          true if operation was successful


*****************************************************************************/
uint8 BML_u_SimpleKafiInit(BML_t_SimpleKafi const *p_kafi) {
    uint8 u_ret;
    u_ret = TRUE;

    /*x = inv(H)*z;*/
    u_ret &= BML_u_MatrixInversion(p_kafi->H, p_kafi->K, p_kafi->T,
                                   p_kafi->x_y); /*T = inv(H)*/
    if (u_ret == TRUE) {
        u_ret &= BML_u_MatrixMultiplication(p_kafi->T, p_kafi->z, p_kafi->x,
                                            p_kafi->x_y, p_kafi->x_y,
                                            1u); /*x = T*z*/

        /*P = inv(H)*R*inv(H'); */
        u_ret &= BML_u_MatrixMultiplication(p_kafi->T, p_kafi->R, p_kafi->P,
                                            p_kafi->x_y, p_kafi->x_y,
                                            p_kafi->x_y); /*P=T*R*/
        u_ret &= CML_u_MatrixTranspose(p_kafi->H, p_kafi->K, p_kafi->x_y,
                                       p_kafi->x_y); /*K=H'*/
        u_ret &= BML_u_MatrixInversion(p_kafi->K, p_kafi->T2, p_kafi->T,
                                       p_kafi->x_y); /*T = inv(K)*/
        u_ret &= BML_u_MatrixMultiplication(p_kafi->P, p_kafi->T, p_kafi->T2,
                                            p_kafi->x_y, p_kafi->x_y,
                                            p_kafi->x_y); /*T2 = P*T*/
        u_ret &= BML_u_MatrixCopy(p_kafi->T2, p_kafi->P, p_kafi->x_y,
                                  p_kafi->x_y); /*P=T*/
    }

    return u_ret;
}

/*****************************************************************************
  Functionname:    BML_u_SimpleKafiPrediction                           */ /*!

  @brief           predict next cycle in simple Kalman filter

  @description     This function predicts the next cycle in simple Kalman 
                   filter. It does prediction for the state vector and the
                   covariance as follows:
                   if X = state vector,
                      P = covariance,
                      A = state transition model matrix
                      B = control-input model vector
                      u = control matrix
                      Q = process noise matrix, then,

                      X = A*X + B*u
                      P = A * P * A' + Q

  @param[in,out]   p_kafi :  a simple Kalman filter structure
                             Supported values for p_kafi->x_y
                             [Full range of uint8]
                             Supported values for p_kafi->B_x
                             [Full range of uint8]
                             Supported values for p_kafi->A
                             [Valid float32 pointer]
                             Supported values for p_kafi->B
                             [Valid float32 pointer]
                             Supported values for p_kafi->u
                             [Valid float32 pointer]
                             Supported values for p_kafi->Q
                             [Valid float32 pointer]
                             Supported values for p_kafi->P
                             [Valid float32 pointer]
                             Supported values for p_kafi->x
                             [Valid float32 pointer]
                             Supported values for p_kafi->T
                             [Valid float32 pointer]
                             Supported values for p_kafi->T2
                             [Valid float32 pointer]
                             Supported values for p_kafi->tx
                             [Valid float32 pointer]

  @return          true if operation was successful


*****************************************************************************/
uint8 BML_u_SimpleKafiPrediction(BML_t_SimpleKafi const *p_kafi) {
    uint8 u_ret = TRUE;
    /*Prediction for state vector and covariance:*/
    /*x = A*x + B*u*/
    u_ret &=
        BML_u_MatrixMultiplication(p_kafi->A, p_kafi->x, p_kafi->tx,
                                   p_kafi->x_y, p_kafi->x_y, 1u); /* tx= A*x */
    u_ret &=
        BML_u_MatrixCopy(p_kafi->tx, p_kafi->x, 1u, p_kafi->x_y); /*x = tx*/
    u_ret &=
        BML_u_MatrixMultiplication(p_kafi->B, p_kafi->u, p_kafi->tx,
                                   p_kafi->B_x, p_kafi->x_y, 1u); /* tx= B*u */
    u_ret &= CML_u_MatrixAddition(p_kafi->x, p_kafi->tx, p_kafi->x, 1u,
                                  p_kafi->x_y); /* x = x + tx*/

    /*P = A * P * A' + Q*/
    u_ret &=
        BML_u_MatrixMultiplication(p_kafi->A, p_kafi->P, p_kafi->T, p_kafi->x_y,
                                   p_kafi->x_y, p_kafi->x_y); /* T=A*P */
    u_ret &= CML_u_MatrixTranspose(p_kafi->A, p_kafi->T2, p_kafi->x_y,
                                   p_kafi->x_y); /* T2=A'*/
    u_ret &= BML_u_MatrixMultiplication(p_kafi->T, p_kafi->T2, p_kafi->P,
                                        p_kafi->x_y, p_kafi->x_y,
                                        p_kafi->x_y); /* P=T*K */
    u_ret &= CML_u_MatrixAddition(p_kafi->P, p_kafi->Q, p_kafi->P, p_kafi->x_y,
                                  p_kafi->x_y); /*P=P+Q*/

    return u_ret;
}

/*****************************************************************************
  Functionname:    BML_u_SimpleKafiUpdate                               */ /*!

  @brief           update Kalman filter (with a given measurement kafi->z)

  @description     This function updates Kalman filter 
                   (with a given measurement kafi->z). It computes the 
                   Kalman gain, correction based on the observation and 
                   error covariance.
                   if X = state vector,
                      P = error covariance,
                      H = Observation model
                      H'= Transpose of H
                      Z = Observation vector
                      K = Kalman gain factor, then,

                      K = P * H' * inv(H * P * H' + R)
                      X = X + K * (Z - H * X)
                      P = P - K * H * P


  @param[in,out]   p_kafi :  a simple Kalman filter structure
                             Supported values for p_kafi->x_y
                             [Full range of uint8]
                             Supported values for p_kafi->z
                             [Valid float32 pointer]
                             Supported values for p_kafi->H
                             [Valid float32 pointer]
                             Supported values for p_kafi->R
                             [Valid float32 pointer]
                             Supported values for p_kafi->P
                             [Valid float32 pointer]
                             Supported values for p_kafi->x
                             [Valid float32 pointer]
                             Supported values for p_kafi->K
                             [Valid float32 pointer]
                             Supported values for p_kafi->T
                             [Valid float32 pointer]
                             Supported values for p_kafi->T2
                             [Valid float32 pointer]
                             Supported values for p_kafi->tx
                             [Valid float32 pointer]

  @return          true if operation was successful


*****************************************************************************/
uint8 BML_u_SimpleKafiUpdate(BML_t_SimpleKafi const *p_kafi) {
    uint8 u_ret = TRUE;

    /*Compute Kalman gain factor:*/
    /*K = P*H'*inv(H*P*H'+R)*/
    u_ret &=
        BML_u_MatrixMultiplication(p_kafi->H, p_kafi->P, p_kafi->K, p_kafi->x_y,
                                   p_kafi->x_y, p_kafi->x_y); /* K=H*P */
    u_ret &= CML_u_MatrixTranspose(p_kafi->H, p_kafi->T, p_kafi->x_y,
                                   p_kafi->x_y); /* T = H'*/
    u_ret &= BML_u_MatrixMultiplication(p_kafi->K, p_kafi->T, p_kafi->T2,
                                        p_kafi->x_y, p_kafi->x_y,
                                        p_kafi->x_y); /* T2=K*T */
    u_ret &= CML_u_MatrixAddition(p_kafi->T2, p_kafi->R, p_kafi->T, p_kafi->x_y,
                                  p_kafi->x_y); /* T=T2+R */
    u_ret &= BML_u_MatrixInversion(p_kafi->T, p_kafi->K, p_kafi->T2,
                                   p_kafi->x_y); /* T2=inv(T)*/
    u_ret &= CML_u_MatrixTranspose(p_kafi->H, p_kafi->K, p_kafi->x_y,
                                   p_kafi->x_y); /* K = H'*/
    u_ret &=
        BML_u_MatrixMultiplication(p_kafi->P, p_kafi->K, p_kafi->T, p_kafi->x_y,
                                   p_kafi->x_y, p_kafi->x_y); /* T = P*K*/
    u_ret &= BML_u_MatrixMultiplication(p_kafi->T, p_kafi->T2, p_kafi->K,
                                        p_kafi->x_y, p_kafi->x_y,
                                        p_kafi->x_y); /* K = T*T2*/

    /*Correction based on observation:*/
    /*s.x = s.x + K*(s.z-s.H*s.x);*/
    u_ret &=
        BML_u_MatrixMultiplication(p_kafi->H, p_kafi->x, p_kafi->tx,
                                   p_kafi->x_y, p_kafi->x_y, 1u); /* tx = H*x */
    u_ret &= BML_u_MatrixSubtraction(p_kafi->z, p_kafi->tx, p_kafi->tx, 1u,
                                     p_kafi->x_y); /* tx = z-tx */
    u_ret &=
        BML_u_MatrixMultiplication(p_kafi->K, p_kafi->tx, p_kafi->z,
                                   p_kafi->x_y, p_kafi->x_y, 1u); /* z = K*tx */
    u_ret &= CML_u_MatrixAddition(p_kafi->x, p_kafi->z, p_kafi->x, 1u,
                                  p_kafi->x_y); /* x = x+z */

    /*P = P - K*H*P;*/
    u_ret &=
        BML_u_MatrixMultiplication(p_kafi->K, p_kafi->H, p_kafi->T, p_kafi->x_y,
                                   p_kafi->x_y, p_kafi->x_y); /* T = K*H */
    u_ret &= BML_u_MatrixMultiplication(p_kafi->T, p_kafi->P, p_kafi->T2,
                                        p_kafi->x_y, p_kafi->x_y,
                                        p_kafi->x_y); /* T2 = T*P */
    u_ret &= BML_u_MatrixSubtraction(p_kafi->P, p_kafi->T2, p_kafi->P,
                                     p_kafi->x_y, p_kafi->x_y); /* P = P-T2 */

    return u_ret;
}

// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"