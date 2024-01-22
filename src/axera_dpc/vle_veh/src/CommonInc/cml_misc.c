

/*****************************************************************************
  INCLUDES
*****************************************************************************/
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" 

#include "TM_Global_Types.h"
#include "TM_Global_defs.h"
#include "TM_CML_Types.h"
#include "TM_Base_Cfg.h"
#include "TM_Base_Const.h"
#include "TM_Base_Emul.h"
#include "TM_Base_Mapping.h"
#include "TM_Base_Mat.h"
#include "TM_Base_Misc.h"
#include "TM_Base_Trigo.h"
#include "TM_Base_Ext.h"

/*****************************************************************************
 (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! Parameters for CML_CalculateDistancePoint2Circle */
/* minimum curvature for which use circle equation instead of parabolic
 * approximation */
#define CURVATURE_USE_CIRCLE_EQUATION (1.F / 1000.F) /* in 1/m */
/* minimum curvature for which use circle equation instead of parabolic
 * approximation */
#define RADIUS_INIT_VALUE (999999.F) /* in m */

/* Constants for logarithm calculation. */
#define FLOAT_EXP_MASK \
    0x7F800000UL /*!< mask for exponent in single precision float */
#define FLOAT_MANT_MASK \
    0x007FFFFFUL /*!< mask for mantissa in single precision float */
#define FLOAT_EXP_SHIFT \
    23UL /*!< shift value for exponent in single precision float */
#define FLOAT_EXP_OFFSET \
    127L /*!< offset of exponent in single precision float */
#define FLOAT_ZERO_HEX 0x3F800000UL /*!< hexadecimal representation of 0.0f */

#define FLOAT_NEG_INF 0xFF800000UL /* float representation for (-inf) */
#define FLOAT_NEG_NAN 0xFFC00000UL /* float representation for (-NAN) */

#define NOF_LOG2_POLY_ELEMENTS \
    8 /* polynome size for calculation of CML_f_log2() */

#define ONE_OVER_LOG2_OF_10 0.301029995663981f /*!< 1.0 / log2(10) */
#define ONE_OVER_LOG2_OF_E 0.693147180559945f  /*!< 1.0 / log2(e) */

/*****************************************************************************
 TYPES
*****************************************************************************/

typedef union {
    float32 f_val; /*!< float representation */
    uint32 u_val;  /*!< hexadecimal representation */
} t_floatAccess;   /*!< access union to manipulate float values */

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/* Calculates X value of the normal crossing from object to clothoid tangent at
 * last approximated X value */
static float32 CML_f_ApproximateRefpoint(
    float32 f_X, float32 f_Y, float32 f_C0, float32 f_C1, float32 f_RefX);

/*****************************************************************************
  Functionname:    BML_f_Mod                                            */ /*!

  @brief           Calculates modulus after division (similar to MATLAB)
                   
  @description     This function calculates modulus after division:
                   result = X - n*Y where n = floor(X/Y)
                   returns same values as CML_Rem if X and Y have same signs
                   
  @param[in]       f_value  : value
                              Supported values are [Full range of float32]
                              Overflow may occur at higher values.
  @param[in]       f_modulo : modulo
                              Supported values are [Full range of float32]
                              Overflow may occur at very small values.

  @return          f_value modulo f_modulo
                   
*****************************************************************************/
float32 BML_f_Mod(float32 f_value, float32 f_modulo) {
    float32 f_quotient, f_ret;

    /* avoid Div0 if y~0 and return 0 (MATLAB returns x) */
    if (BML_f_IsZero(f_modulo)) {
        f_ret = 0.0f;
    }

    else {
        f_quotient = f_value / f_modulo;

        /* If quotient is out of range or whole number return 0. */
        if ((BML_f_Abs(f_quotient) > (float32)C_LONG_MAX) ||
            BML_f_IsZero(BML_Round(f_quotient) - f_quotient)) {
            f_ret = 0.f;
        }

        else {
            f_ret = (f_value - (BML_f_Floor(f_quotient) * f_modulo));
        }
    }

    return f_ret;
}

/*****************************************************************************
  Functionname:    BML_f_REnvm                                            */ /*!

  @brief           Calculates the remainder after division (similar to MATLAB)
                  
  @description     This function calculates remainder after division: 
                   result = X - n*Y where n = fix(X/Y)
                   returns same values as CML_Mod if X and Y have same signs
                  
  @param[in]       f_dividend : dividend
                                Supported values are [Full range of float32]
                                Overflow may occur at higher values.
  @param[in]       f_divisor  : divisor
                                Supported values are [Full range of float32]
                                Overflow may occur at very small values.

  @return          remainder of f_dividend / f_divisor
                  
*****************************************************************************/
float32 BML_f_REnvm(float32 f_dividend, float32 f_divisor) {
    float32 f_quotient, f_ret;

    if ((f_divisor < BML_f_ModuloEps) && (f_divisor > -BML_f_ModuloEps)) {
        /* avoid Div0 if y~0 and return 0 (MATLAB returns x) */
        f_ret = 0.0f;
    } else {
        f_quotient = f_dividend / f_divisor;

        if (BML_f_Abs(f_quotient) > (float32)C_LONG_MAX) {
            f_ret = 0.f;
        } else {
            f_ret = (f_dividend - ((float32)(sint32)(f_quotient)*f_divisor));
        }
    }

    return f_ret;
}

/*****************************************************************************
  Functionname:    BML_f_ModTrig                                        */ /*!

  @brief           Calculates the remainder of x when divided by y as needed 
                    by trigonometric functions
                   
  @description     This function calculates the remainder of x when divided by y
                   Works only for y > 0
                   The function is equivalent to rem() function in Matlab.
                   
                   
  @param[in]       f_dividend : The Dividend 
                                Supported values are [Full range of float32]
                                Overflow may occur at higher values.
  @param[in]       f_divisor  : The Divisor
                                Supported values are [Full range of float32]
                                Overflow may occur at very small values.

  @return          remainder of f_dividend when divided by f_divisor

*****************************************************************************/
float32 BML_f_ModTrig(float32 f_dividend, float32 f_divisor) {
    float32 f_quotient, f_ret;
    sint32 s_quotient;

    if (f_divisor < BML_f_ModuloEps) {
        f_ret = 0.f;
    } else {
        f_quotient = f_dividend / f_divisor;

        if (BML_f_Abs(f_quotient) > (float32)C_LONG_MAX) {
            f_ret = 0.f;
        } else {
            s_quotient = (sint32)(f_quotient);
            f_ret = (f_dividend - ((float32)s_quotient * f_divisor));
        }
    }

    return f_ret;
}

/*****************************************************************************
Functionname:    BML_s_CountNrOfBitsSet                               */ /*!

@brief           Count the number of bits set in a 32 bit unsigned int

@description     This function counts and returns the number of bits set
                 in a 32 bit unsigned integer
                 Executes in constant runtime, 7 cycles amortized time on C6X

@param[in]       u_PruefBits : 32-bit unsigned integer
                               Supported values [Full range of uint32]

@return          the number of bits set


*****************************************************************************/
sint32 BML_s_CountNrOfBitsSet(uint32 u_PruefBits) {
    sint32 s_AnzahlEinsBit = 0L;

    u_PruefBits = u_PruefBits - ((u_PruefBits >> 1U) & 0x55555555);
    u_PruefBits =
        (u_PruefBits & 0x33333333) + ((u_PruefBits >> 2) & 0x33333333);
    s_AnzahlEinsBit =
        (sint32)((((u_PruefBits + (u_PruefBits >> 4U)) & 0x0F0F0F0FU) *
                  0x01010101U) >>
                 24U);

    return s_AnzahlEinsBit;
}

/*****************************************************************************
  Functionname:    BML_v_CalculateCOFEgomotionMatrices                  */ /*!

  @brief           Calculate Trasformation Matrices for Egomotion Compensation 
                   of COF(Coordinate system Of vehicle Front)
                   
  @description     This function calculate Trasformation Matrices for Egomotion 
                   Compensation of COF (Coordinate system Of vehicle Front).
                   It determines the transformation type and entries of the 
                   transfromation matrices (forward and backward).
                   
  @param[in]       f_SpeedCorrected :               The ego motion speed
                                                    Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_EgoAcceleration :              The ego acceleration
                                                    Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_YawRate_e :                    The ego yaw rate
                                                    Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_CycleTime :                    The time delta to use for calculation
                                                    Optimal values are [-F_MAX,..,F_MAX]
                                                    where F_MAX is the fourth root of max range of float32.
  @param[out]      p_TrafoMatrix2DCOFForwardRaw :   pointer to buffer for forward transformation matrix.
                                                    [Valid matrix structure pointer]
                                                    Supported values for p_TrafoMatrix2DCOFForwardRaw->TrafoType are 0,1,2.
                                                    Range for p_TrafoMatrix2DCOFForwardRaw->f00 is [0,..,1]
                                                    Range for p_TrafoMatrix2DCOFForwardRaw->f02 is [Full range of float32]
                                                    Range for p_TrafoMatrix2DCOFForwardRaw->f10 is [0,..,1]
                                                    Range for p_TrafoMatrix2DCOFForwardRaw->f12 is [Full range of float32]
  @param[out]      p_TrafoMatrix2DCOFBackwardRaw :  pointer to buffer for backward transformation matrix
                                                    [Valid matrix structure pointer]
                                                    Supported values for p_TrafoMatrix2DCOFBackwardRaw->TrafoType are 0,1,2.
                                                    Range for p_TrafoMatrix2DCOFBackwardRaw->f00 is [0,..,1]
                                                    Range for p_TrafoMatrix2DCOFBackwardRaw->f02 is [Full range of float32]
                                                    Range for p_TrafoMatrix2DCOFBackwardRaw->f10 is [0,..,1]
                                                    Range for p_TrafoMatrix2DCOFBackwardRaw->f12 is [Full range of float32]

                   
  @return          none
                   
  
*****************************************************************************/
void BML_v_CalculateCOFEgomotionMatrices(
    BML_t_TrafoMatrix2D* p_TrafoMatrix2DCOFForwardRaw,
    BML_t_TrafoMatrix2D* p_TrafoMatrix2DCOFBackwardRaw,
    fVelocity_t f_SpeedCorrected,
    fAccel_t f_EgoAcceleration,
    fYawRate_t f_YawRate_e,
    fTime_t f_CycleTime) {
    fAngle_t f_YawAngle;
    float32 f_DrivenDistance;
    float32 f_cosYawAngle;
    float32 f_sinYawAngle;
    float32 f_Translation_X;
    float32 f_Translation_Y;

    /* this section is only of equal test */
    float32 f_YawRateCurve;
    float32 f_YawRate;
    if (f_SpeedCorrected > BML_f_Delta) {
        f_YawRateCurve = f_YawRate_e / f_SpeedCorrected;
    } else {
        f_YawRateCurve = 0.f;
    }
    f_YawRate = f_SpeedCorrected * f_YawRateCurve;

    f_YawAngle = f_YawRate * f_CycleTime;

    f_DrivenDistance =
        (f_SpeedCorrected * f_CycleTime) +
        (((0.5f * f_EgoAcceleration) * f_CycleTime) * f_CycleTime);

    /* to avoid division by zero */
    if (BML_f_Abs(f_YawAngle) <= BML_f_Delta) {
        p_TrafoMatrix2DCOFForwardRaw->TrafoType = TRANSLATION_;
        p_TrafoMatrix2DCOFForwardRaw->f00 = 1.0f;
        p_TrafoMatrix2DCOFForwardRaw->f02 = -1.0f * f_DrivenDistance;
        p_TrafoMatrix2DCOFForwardRaw->f10 = 0.0f;
        p_TrafoMatrix2DCOFForwardRaw->f12 = 0.0f;

        p_TrafoMatrix2DCOFBackwardRaw->TrafoType = TRANSLATION_;
        p_TrafoMatrix2DCOFBackwardRaw->f00 = 1.0f;
        p_TrafoMatrix2DCOFBackwardRaw->f02 = f_DrivenDistance;
        p_TrafoMatrix2DCOFBackwardRaw->f10 = 0.0f;
        p_TrafoMatrix2DCOFBackwardRaw->f12 = 0.0f;
    } else {
        /* fuer kleine Winkel ist die SIN/COS-Approximation schlecht!!!  */
        /* absoluter fehler sin/cos_52: <= 6.70669e-6 */
        /* absoluter fehler sin/cos_32: <= 5.98681e-4 */
        /* nutze deshalb die Approximation  */
        /* sin(x) = x, mit abs. Fehler e-6 fuer x > 0.03rad */
        /* cos(x) = 1-0.5*x*x, mit abs. Fehler e-6 fuer x > 0.003rad */
        if (f_YawAngle < BML_f_MaxAngleSinApprox_1) {
            f_sinYawAngle = f_YawAngle;
            if (f_YawAngle < BML_f_MaxAngleSinApprox_2) {
                f_cosYawAngle = 1.0f - ((0.5f * f_YawAngle) * f_YawAngle);
            } else {
                f_cosYawAngle = COS_HD_(f_YawAngle);
            }
        } else {
            f_sinYawAngle = SIN_HD_(f_YawAngle);
            f_cosYawAngle = COS_HD_(f_YawAngle);
        }

        /* calculate Translation-Vector */

        f_Translation_X =
            -1.0f * (f_DrivenDistance / f_YawAngle) * f_sinYawAngle;
        f_Translation_Y =
            -1.0f * (f_DrivenDistance / f_YawAngle) * (1.0f - f_cosYawAngle);

        p_TrafoMatrix2DCOFForwardRaw->TrafoType = DEFAULT_;
        p_TrafoMatrix2DCOFForwardRaw->f00 = f_cosYawAngle;
        p_TrafoMatrix2DCOFForwardRaw->f02 = (f_Translation_X * f_cosYawAngle) +
                                            (f_Translation_Y * f_sinYawAngle);
        p_TrafoMatrix2DCOFForwardRaw->f10 = -1.0f * f_sinYawAngle;
        p_TrafoMatrix2DCOFForwardRaw->f12 = (f_Translation_Y * f_cosYawAngle) -
                                            (f_Translation_X * f_sinYawAngle);

        p_TrafoMatrix2DCOFBackwardRaw->TrafoType = DEFAULT_;
        p_TrafoMatrix2DCOFBackwardRaw->f00 = f_cosYawAngle;
        p_TrafoMatrix2DCOFBackwardRaw->f02 = -1.0f * f_Translation_X;
        p_TrafoMatrix2DCOFBackwardRaw->f10 = f_sinYawAngle;
        p_TrafoMatrix2DCOFBackwardRaw->f12 = -1.0f * f_Translation_Y;
    }
}

/*****************************************************************************
  Functionname:    BML_v_CalcCOFEgomotionMatrices                       */ /*!

  @brief           Calculate Transformation Matrices for Egomotion Compensation 
                   of COF (Coordinate system Of vehicle Front)
                   
  @description     This function calculate forward and jitter transformation 
                   Matrices for Egomotion Compensation of COF 
                   (Coordinate system Of vehicle Front)

  @param[in]       f_EgoSpeedXTgtSync :                The ego motion speed
                                                       Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_EgoAccelXTgtSync :                The ego acceleration
                                                       Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_YawRateTgtSync :                  The ego yaw rate
                                                       Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_YawRateVarTgtSync :               variance of the ego yaw rate
                                                       Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_YawRateMaxJitterTgtSync :         max jitter of ego yaw rate
                                                       Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_SensorXPosition :                 sensor x position
                                                       Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_CycleTime :                       sensor cycle time
                                                       Optimal values are [-F_MAX,..,F_MAX]
                                                       where F_MAX is the fourth root of max range of float32.
  @param[out]      p_TrafoMatrix2DCOFForwardTgtSync :  pointer to buffer for forward transformation matrix
                                                       [Valid matrix structure pointer]
                                                       Supported values for p_TrafoMatrix2DCOFForwardTgtSync->TrafoType are 0,1,2.
                                                       Range for p_TrafoMatrix2DCOFForwardTgtSync->f00 is [0,..,1]
                                                       Range for p_TrafoMatrix2DCOFForwardTgtSync->f02 is [Full range of float32]
                                                       Range for p_TrafoMatrix2DCOFForwardTgtSync->f10 is [0,..,1]
                                                       Range for p_TrafoMatrix2DCOFForwardTgtSync->f12 is [Full range of float32]
  @param[out]      p_TrafoMatrix2DCOFForJitTgtSync :   pointer to buffer for jitter transformation matrix
                                                       [Valid matrix structure pointer]
                                                       Supported values for p_TrafoMatrix2DCOFForJitTgtSync->TrafoType are 0,1,2.
                                                       Range for p_TrafoMatrix2DCOFForJitTgtSync->f00 is [0,..,1]
                                                       Range for p_TrafoMatrix2DCOFForJitTgtSync->f02 is [Full range of float32]
                                                       Range for p_TrafoMatrix2DCOFForJitTgtSync->f10 is [0,..,1]
                                                       Range for p_TrafoMatrix2DCOFForJitTgtSync->f12 is [Full range of float32]
  @param[in]       f_SensorYPosition                   Not in use
                                                       Supported values are [Full range of float32]
  @param[in]       f_SlipAngleTgtSync                  Not in use
                                                       Supported values are [Full range of float32]
  @param[in]       f_SlipAngleVarTgtSync               Not in use
                                                       Supported values are [Full range of float32]

  @return          none
                   

*****************************************************************************/
void BML_v_CalcCOFEgomotionMatrices(
    BML_t_TrafoMatrix2D* p_TrafoMatrix2DCOFForwardTgtSync,
    BML_t_TrafoMatrix2D* p_TrafoMatrix2DCOFForJitTgtSync,
    fVelocity_t f_EgoSpeedXTgtSync,
    fVelocity_t f_EgoAccelXTgtSync,
    fYawRate_t f_YawRateTgtSync,
    fVariance_t f_YawRateVarTgtSync,
    fYawRate_t f_YawRateMaxJitterTgtSync,
    fDistance_t f_SensorXPosition,
    fDistance_t f_SensorYPosition,
    fAngle_t f_SlipAngleTgtSync,
    fVariance_t f_SlipAngleVarTgtSync,
    fTime_t f_CycleTime) {
    float32 f_YawAngle, f_YawAngleJitter;
    float32 f_DrivenDistance;
    float32 f_sinTmp = 0.0f, /* temporary variable for sinus result */
        f_cosTmp = 0.0f;     /* temporary variable for cosinus result */

    GDBTrafoMatrix2D_t COF2COG, TMOT, RMOT, TJMOT, RJMOT, COG2COF, M_tmp;

    f_YawAngle = f_YawRateTgtSync * f_CycleTime;
    f_DrivenDistance =
        (f_EgoSpeedXTgtSync * f_CycleTime) +
        (((0.5f * f_EgoAccelXTgtSync) * f_CycleTime) * f_CycleTime);

    /*Calculate Matrices*/
    M_tmp.TrafoType = TRANSLATION_;
    M_tmp.f00 = 1.0f;
    M_tmp.f10 = 0.0f;
    M_tmp.f02 = 0.0f;
    M_tmp.f12 = 0.0f;

    COF2COG.TrafoType = TRANSLATION_;
    COF2COG.f00 = 1.0f;
    COF2COG.f10 = 0.0f;
    COF2COG.f02 = f_SensorXPosition;
    COF2COG.f12 = 0.0f;

    COG2COF.TrafoType = TRANSLATION_;
    COG2COF.f00 = 1.0f;
    COG2COF.f10 = 0.0f;
    COG2COF.f02 = -1.0f * f_SensorXPosition;
    COG2COF.f12 = 0.0f;

    RMOT.TrafoType = ROTATION_;
    GDBsincos(-1.0f * f_YawAngle, &f_sinTmp, &f_cosTmp);
    RMOT.f00 = f_cosTmp;
    RMOT.f10 = f_sinTmp;
    RMOT.f02 = 0.0f;
    RMOT.f12 = 0.0f;

    TMOT.TrafoType = TRANSLATION_;
    TMOT.f00 = 1.0f;
    TMOT.f10 = 0.0f;
    /* to avoid division by zero */
    if (BML_f_Abs(f_YawAngle) <= BML_f_Delta) {
        TMOT.f02 = -1.0f * f_DrivenDistance;
        TMOT.f12 = 0.0f;
    } else {
        TMOT.f02 = (f_DrivenDistance / f_YawAngle) * RMOT.f10;
        TMOT.f12 = (f_DrivenDistance / f_YawAngle) * (1.0f - RMOT.f00);
    }

    /*! @todo write as one line to avoid runtime because of memcopying*/
    M_tmp = CML_TrafoMatrix2DMult(COF2COG,
                                  M_tmp); /*center front to center of gravity*/
    M_tmp = CML_TrafoMatrix2DMult(TMOT, M_tmp); /*translation trough motion*/
    M_tmp = CML_TrafoMatrix2DMult(RMOT, M_tmp); /*rotation trough motion*/
    M_tmp =
        CML_TrafoMatrix2DMult(COG2COF, M_tmp); /*center of gravity to front*/
    *p_TrafoMatrix2DCOFForwardTgtSync = M_tmp;

    if (f_YawRateMaxJitterTgtSync > 0.0f) {
        f_YawAngleJitter = (f_YawRateTgtSync + f_YawRateMaxJitterTgtSync +
                            BML_f_Sqrt(f_YawRateVarTgtSync)) *
                           f_CycleTime;
    } else {
        f_YawAngleJitter = ((f_YawRateTgtSync + f_YawRateMaxJitterTgtSync) -
                            BML_f_Sqrt(f_YawRateVarTgtSync)) *
                           f_CycleTime;
    }
    RJMOT.TrafoType = ROTATION_;
    GDBsincos(-1.0f * f_YawAngleJitter, &f_sinTmp, &f_cosTmp);
    RJMOT.f00 = f_cosTmp;
    RJMOT.f10 = f_sinTmp;
    RJMOT.f02 = 0.0f;
    RJMOT.f12 = 0.0f;
    TJMOT.TrafoType = TRANSLATION_;
    TJMOT.f00 = 1.0f;
    TJMOT.f10 = 0.0f;
    /* to avoid division by zero */
    if (BML_f_Abs(f_YawAngleJitter) <= BML_f_Delta) {
        TJMOT.f02 = -1.0f * f_DrivenDistance;
        TJMOT.f12 = 0.0f;
    } else {
        TJMOT.f02 = (f_DrivenDistance / f_YawAngleJitter) * RJMOT.f10;
        TJMOT.f12 = (f_DrivenDistance / f_YawAngleJitter) * (1.0f - RJMOT.f00);
    }

    /*Calculate Matrices*/
    M_tmp.TrafoType = TRANSLATION_;
    M_tmp.f00 = 1.0f;
    M_tmp.f10 = 0.0f;
    M_tmp.f02 = 0.0f;
    M_tmp.f12 = 0.0f;
    /*! @todo write as one line to avoid runtime because of memcopying*/
    M_tmp = CML_TrafoMatrix2DMult(COF2COG,
                                  M_tmp); /*center front to center of gravity*/
    M_tmp = CML_TrafoMatrix2DMult(TJMOT, M_tmp); /*translation trough motion*/
    M_tmp = CML_TrafoMatrix2DMult(RJMOT, M_tmp); /*rotation trough motion*/
    M_tmp =
        CML_TrafoMatrix2DMult(COG2COF, M_tmp); /*center of gravity to front*/
    *p_TrafoMatrix2DCOFForJitTgtSync = M_tmp;

    BML_UNREFERENCED_FORMAL_PARAMETER(f_SensorYPosition);
    BML_UNREFERENCED_FORMAL_PARAMETER(f_SlipAngleTgtSync);
    BML_UNREFERENCED_FORMAL_PARAMETER(f_SlipAngleVarTgtSync);
}

/*****************************************************************************
  Functionname:    BML_f_GetPickupDist                                  */ /*!

  @brief           Calculate pickup distance for distance control
  
  @description     Calculate the pickup distance for distance control with given
                   ego speed and deceleration, target speed, desired gap time and 
                   latency time. The pick up distance is calculated only when Ego 
                   deceleration is greater than the minimum acceptable value (~ >0).
                   Otherwise a value of 0 is returned.
                   If PD = Pickup distance,
                      OS = Object Rel speed,
                      ES = Ego speed,
                      ED = Ego deceleration,
                      GT = Gap time
                      LT = Latency time,
                   then,
                      PD = (((ES+OS)*((GT-LT)-(OS/ED)))+(ES*(LT+(OS/ED))))-((OS*OS)/(2*ED))


  @param[in]       f_ObjRelSpeed :      actual object rel speed
                                        Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_EgoSpeed :         actual ego speed
                                        Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_EgoDeceleration :  assuemd deceleration while distance control (larger than 0)
                                        Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_GapTime :          desired time gap after distance control
                                        Optimal values are [-F_MAX,..,F_MAX]
  @param[in]       f_LatencyTime :      assumed latency of distance control
                                        Optimal values are [-F_MAX,..,F_MAX]
                                        where F_MAX is the fourth root of max range of float32.

  @return          the pickup distance in the given situation (returns 0.f if fEgoDeceleration < BML_f_Delta)
                   

*****************************************************************************/
float32 BML_f_GetPickupDist(float32 f_ObjRelSpeed,
                            float32 f_EgoSpeed,
                            float32 f_EgoDeceleration,
                            float32 f_GapTime,
                            float32 f_LatencyTime) {
    float32 f_Dist;
    float32 f_ObjSpeed = f_EgoSpeed + f_ObjRelSpeed;

    if (f_EgoDeceleration < BML_f_Delta) {
        f_Dist = 0.f;
    } else {
        f_Dist = ((f_ObjSpeed * ((f_GapTime - f_LatencyTime) -
                                 (f_ObjRelSpeed / f_EgoDeceleration))) +
                  (f_EgoSpeed *
                   (f_LatencyTime + (f_ObjRelSpeed / f_EgoDeceleration)))) -
                 (0.5F * (BML_Sqr(f_ObjRelSpeed) / f_EgoDeceleration));
    }
    return f_Dist;
}

/*****************************************************************************
  Functionname:    BML_f_ComputeClothoidLateralDistance                 */ /*!

  @brief           Compute the y-distance of a clothoid for a given x-distance
  
  @description     Compute the y-distance of a clothoid for a given x-distance.
                   The lateral distance of clothoid is calculated as follows:
                   Let, A  = the heading direction or angle,
                        X  = the length,
                        Y  = lateral displacement,
                        C0 = the initial curvature,
                        C1 = curvature change,
                   then,
                        Y  = (A*X)+((C0*X*X)/2)+((C1*X*X*X)/6)
                   
                     Limitation : Since the inputs and output datatypes are same, 
                     the function fails to store the results for higher values when 
                     the result exceeds the float range.

  @param[in]       f_Xpos :  distance (x-direction)
                             Optimal range is [-MAX_VAL,..,MAX_VAL], where 
                             MAX_VAL is fourth root of maximum value of float32.
  @param[in]       f_C0 :    Curvature
                             Optimal range is [-MAX_VAL,..,MAX_VAL], where 
                             MAX_VAL is fourth root of maximum value of float32.
  @param[in]       f_C1 :    Curvature change
                             Optimal range is [-MAX_VAL,..,MAX_VAL], where 
                             MAX_VAL is fourth root of maximum value of float32.
  @param[in]       f_Angle : Yaw Angle between car and road axis
                             Optimal range is [-MAX_VAL,..,MAX_VAL], where 
                             MAX_VAL is fourth root of maximum value of float32.

  @return          y-distance of a clothoid for a given x-distance
                   
*****************************************************************************/
float32 BML_f_ComputeClothoidLateralDistance(float32 f_Xpos,
                                             float32 f_C0,
                                             float32 f_C1,
                                             float32 f_Angle) {
    float32 f_Tmp = f_Xpos * f_Xpos * 0.5f;
    return ((f_Angle * f_Xpos) + (f_C0 * f_Tmp) +
            (f_C1 * f_Xpos * f_Tmp * (1.0f / 3.0f)));
}

/*****************************************************************************
  Functionname:    BML_f_LowPassFilter2                                 */ /*!

  @brief           simple first order lowpass filter
                 
  @description     This function is an implementation of simple first order
                   lowpass filter. This determines the output sample in terms of
                   the input sample and preceding output.
                   So if x = input, y = output and z = previous output, and 
                   a = filter coefficient, then,
                   y = (a*x) + ((1-a)*z)        
  
  @param[in,out]   f_Old :   old value (in), filtered value (out)
                             Valid float pointer.
                             Supported value for data [Full range of float32]
  @param[in]       f_New :   new value
                             Supported value [Full range of float32]
  @param[in]       f_Alpha : filter coefficient
                             Optimal Values [0,..,1]

  @return          none
                   
  
*****************************************************************************/
void BML_f_LowPassFilter2(float32* f_Old, float32 f_New, float32 f_Alpha) {
    float32 f_Dummy;

    f_Dummy = (f_Alpha * f_New) + ((1.f - f_Alpha) * (*f_Old));
    *f_Old = f_Dummy;
}

/*******************************************************************************
  Functionname:    CML_f_log2                                             */
float32 CML_f_log2(const float32 f_in) {
    float32 f_mantissa,   /* mantissa float value of input */
        f_logMant,        /* log2(mant) */
        f_powVal,         /* power value for polynomial */
        f_result;         /* return value */
    sint32 s_exp,         /* exponent of input */
        s_cnt;            /* count value */
    t_floatAccess accTmp; /* buffer for bit manipulation of input value */

    const float32 a_polyCoeff[NOF_LOG2_POLY_ELEMENTS] = {
        -3.235209154962737f, 7.085099692691060f,  -7.396145690524245f,
        5.673515407791697f,  -2.914488887785300f, 0.950740445200145f,
        -0.178109466192688f, 0.014598466017242f}; /* polynomial coefficients for
                                                     log2(mant) */

    /* Check if input is a positive value. */
    if (f_in > 0.0f) {
        /* Write input to access union. */
        accTmp.f_val = f_in;

        /* Get exponent value. */
        s_exp = (sint32)((accTmp.u_val & FLOAT_EXP_MASK) >> FLOAT_EXP_SHIFT);
        s_exp -= FLOAT_EXP_OFFSET;

        /* Get mantissa in (float). */
        accTmp.u_val = accTmp.u_val & FLOAT_MANT_MASK;
        accTmp.u_val += FLOAT_ZERO_HEX;
        f_mantissa = accTmp.f_val;

        /* Calculate logarithmus of mantissa with series expansion:
           log2(x) = p0 + p1 * x^1 + p2 * x^2... */
        f_powVal = 1.0f;
        f_logMant = a_polyCoeff[0];

        for (s_cnt = 1L; s_cnt < NOF_LOG2_POLY_ELEMENTS; s_cnt++) {
            f_powVal *= f_mantissa;
            f_logMant = BML_f_MultAdd(f_powVal, a_polyCoeff[s_cnt], f_logMant);
        } /* for(s_cnt = 1L; s_cnt < NOF_LOG2_POLY_ELEMENTS; s_cnt++) */

        /* Result is log(mantissa) + exponent. */
        f_result = f_logMant + (float32)s_exp;
    } /* if(f_in > 0.0f) */

    else if (f_in < 0.0f) {
        /* Log() is not defined for real negative inputs. Return (-NAN). */
        accTmp.u_val = FLOAT_NEG_NAN;
        f_result = accTmp.f_val;
    } /* if/else if(f_in) */

    else {
        /* Log(0) is (-inf). */
        accTmp.u_val = FLOAT_NEG_INF;
        f_result = accTmp.f_val;
    } /* if/else if/else(f_in) */

    return f_result;
} /* CML_f_log2() */

/*******************************************************************************
  Functionname:    BML_f_log10                                            */
float32 BML_f_log10(const float32 f_in) {
    return ONE_OVER_LOG2_OF_10 * CML_f_log2(f_in);
} /* BML_f_log10() */

/*******************************************************************************
  Functionname:    CML_f_ln                                            */
float32 CML_f_ln(const float32 f_in) {
    return ONE_OVER_LOG2_OF_E * CML_f_log2(f_in);
} /* CML_f_ln() */

/*****************************************************************************
  Functionname:    BML_f_fastlog10                                      */ /*!

  @brief           Calculates a fast decadic logarithm approximation

  @description     This algorithm uses an IEEE754 floating point 
                   decomposition to get the 2's exponent as coarse log2 
                   value. To get a closer approximation the log2 of the 
                   mantissa is fetched from a lookup table and added to the 
                   exponent. At last the log2 is multiplied by a scaling 
                   factor to get the according log10 value.

  @param[in]       f_value : Value to be logarithmized to the base of 10
                             Supported range [Full range of float32]

  @return          Decadic logarithm approximation of input value


*****************************************************************************/
float32 BML_f_fastlog10(float32 f_value) {
    /* float access as integer */

    typedef union {
        float32 f_entry;
        uint32 u_entry;
        sint32 s_entry;
    } t_FltAccess;

    /* bias of exponent in internal float memory */
    static const uint32 F32_EXPO_BIAS = 127u;
    /* number of mantissa bits in internal float memory */
    static const uint32 F32_MANT_BITS = 23u;

    static const uint32 F32_EXPO_MASK = 0x7F800000u;
    static const uint32 F32_MANT_MASK = 0x007FFFFFu;
    static const uint32 F32_NEG_INF = 0xFF800000u;
    static const uint32 F32_NEG_NAN = 0xFFC00000u;
    static const float32 LOG10_LOG2_FACT = 0.30102999566398f;

    /* Log2 lookup table between 1 and 2 */
    static const float32 a_log2_lut[N_LUT] = {
        0.00000000000000f, 0.04439411935845f, 0.08746284125034f,
        0.12928301694497f, 0.16992500144231f, 0.20945336562895f,
        0.24792751344359f, 0.28540221886225f, 0.32192809488736f,
        0.35755200461808f, 0.39231742277876f, 0.42626475470210f,
        0.45943161863730f, 0.49185309632967f, 0.52356195605701f,
        0.55458885167764f, 0.58496250072116f, 0.61470984411521f,
        0.64385618977472f, 0.67242534197150f, 0.70043971814109f,
        0.72792045456320f, 0.75488750216347f, 0.78135971352466f,
        0.80735492205760f, 0.83289001416474f, 0.85798099512757f,
        0.88264304936184f, 0.90689059560852f, 0.93073733756289f,
        0.95419631038688f, 0.97727992349992f};

    float32 f_lut_val;
    float32 f_lut_idx;
    uint32 u_temp;
    t_FltAccess xFltAcc;
    xFltAcc.f_entry = f_value;

    if (0.0f < xFltAcc.f_entry) {
        u_temp = xFltAcc.u_entry & F32_MANT_MASK;
        f_lut_idx = (float32)u_temp;
        f_lut_idx *= C_LUT_MAP;
        f_lut_val = a_log2_lut[(uint32)f_lut_idx];

        xFltAcc.u_entry = xFltAcc.u_entry & F32_EXPO_MASK;
        xFltAcc.u_entry = xFltAcc.u_entry >> F32_MANT_BITS;
        xFltAcc.s_entry = xFltAcc.s_entry - (sint32)F32_EXPO_BIAS;

        xFltAcc.f_entry =
            (float32)(LOG10_LOG2_FACT * ((float32)xFltAcc.s_entry + f_lut_val));
    } /* if ( 0.0f < f_value ) */
    else if (0.0f > xFltAcc.f_entry) {
        xFltAcc.u_entry = F32_NEG_NAN;
    } else {
        xFltAcc.u_entry = F32_NEG_INF;
    }

    return xFltAcc.f_entry;
}

/*****************************************************************************
  Functionname:    BML_f_fastlog                                          */ /*!
  
  @brief           Calculates a fast natural logarithm approximation

  @description     This function uses BML_f_fastlog10.
                   This function calculates the natural logarithm using             
                   ln(input) = log10(input) * ln(10)


  @param[in]       f_value: Value to be logarithmized
                            Supported range [Full range of float32]

  @return          Natural logarithm approximation of input value


*****************************************************************************/
float32 BML_f_fastlog(float32 f_value) {
    /*multiply with ln(10)*/
    return CML_LN_10 * BML_f_fastlog10(f_value);
}

/*****************************************************************************
  Functionname:    BML_f_SqrtApprox                                   */
float32 BML_f_SqrtApprox(float32 f_radicand) {
    sint32 s_expo;
    uint32 u_tmp;
    float32 f_Sample, f_SampleSquare;
    float32 f_A, f_B, f_A_plus_B;
    float32 f_ret;

    /*! union for doing bit-wise manipulation on the floating point
     * representation */

    union {
        float32 f_value; /* this is the number of interest as float */
        uint32 u_value;  /* in here we hold the same number as int on which we
                            can do bit-wise manipulations */
    } x_tmp;

    /* check for x < 0 or x = NaN */
    BML_ASSERT(f_radicand >= 0.f);

    /* copy input value to union where we can manipulate it directly */
    x_tmp.f_value = f_radicand;

    /* check for negative or zero or denormalized */
    if (((x_tmp.u_value >> 31) > 0U)      /* sign bit set? */
        || ((x_tmp.u_value >> 23) == 0u)) /* exponent is zero? */
    {
        f_ret = 0.f;
    }
    /* check for infinity */
    else if ((x_tmp.u_value >> 23) >= 0xffu) {
        x_tmp.u_value = 0x7f7fffffu;
        f_ret = x_tmp.f_value; /* return max_float */
    } else {
        /* calculate start value by dividing exponent by two */
        u_tmp = x_tmp.u_value >> BML_SqrtApprox_NumExpo;
        s_expo = (sint32)(u_tmp);
        s_expo -= BML_SqrtApprox_ExponentOffset;
        if (s_expo < 0) {
            s_expo >>= 1; /* without the following line this would be unsafe
                             because right shift of an unsigned int is machine
                             dependent (sign fill vs. zero fill) */
            s_expo |=
                (sint32)0x80000000U; /* -> fill highest bit with sign (one)
                                        after shift to make it safe */
        } else {
            s_expo >>=
                1; /* for positive expo sign = 0 -> sign fill = zero fill */
        }
        s_expo += BML_SqrtApprox_ExponentOffset;
        x_tmp.u_value &= BML_SqrtApprox_MantissaMask;
        x_tmp.u_value += ((uint32)s_expo << BML_SqrtApprox_NumExpo);
        f_Sample = x_tmp.f_value;

        /* two iterations are enough */
        /* iteration one */
        f_SampleSquare = BML_Sqr(f_Sample);
        f_A = 2.f * (f_SampleSquare + f_radicand);
        f_B = f_SampleSquare - f_radicand;
        f_A_plus_B = f_A + f_B;
        if (BML_f_Abs(f_A_plus_B) < BML_SqrtApprox_AlmostZero) {
            f_ret = 0.f;
        } else {
            f_Sample *= ((f_A - f_B) / (f_A_plus_B));
            /* iteration two */
            f_SampleSquare = BML_Sqr(f_Sample);
            f_A = 2.f * (f_SampleSquare + f_radicand);
            f_B = f_SampleSquare - f_radicand;

            f_A_plus_B = f_A + f_B;
            f_Sample *= ((f_A - f_B) / (f_A_plus_B));
            f_ret = f_Sample;
        }
    }

    return f_ret;
}

/*****************************************************************************
  Functionname     - CML_InvSqrtFast67
  @brief           - Approximates the inverse square root of x

  @description     - Maximum error is 2.07e-7. No input checks are made.
                   - The caller must guarantee that the input is valid.
                   - The method is described in Lomont(2003)

  @param[in]       - x: A positive real number.

  @return          - A precise calculation of x^(-1/2)

*****************************************************************************/
float32 BML_InvSqrt67(float32 f_x) {
    const sint32 i_MagicNumber = 0x5f375a86;
    const float32 f_MagicNumber = 1.5F;
    sint32 s_i;
    float32 f_x_Half;
    float32 f_y;
    BML_t_FloatAsSigned u;
    u.f_d = f_x;

    f_x_Half = f_x * 0.5F;
    // f_y = f_x;

    s_i = u.s_x;
    s_i = i_MagicNumber - (s_i / 2);
    u.s_x = s_i;
    f_y = u.f_d;

    f_y = f_y * (f_MagicNumber - (f_x_Half * f_y * f_y));
    f_y = f_y * (f_MagicNumber - (f_x_Half * f_y * f_y));
    f_y = f_y * (f_MagicNumber - (f_x_Half * f_y * f_y));
    return f_y;
}

/*****************************************************************************
  Functionname     - BML_SqrtFast67
  @brief           - Approximates the square root of x

  @description     - Maximum error is 2.07e-7. No input checks are made.
                   - The caller must guarantee that the input is valid.
                   - The method is described in Lomont(2003)

  @param[in]       - x: A positive real number.

  @return          - A precise calculation of x^(1/2)

*****************************************************************************/
float32 BML_Sqrt67(float32 f_x) { return f_x * BML_InvSqrt67(f_x); }

/*****************************************************************************

  Functionname:    CML_s_GetLutIndex                                    */
sint32 CML_s_GetLutIndex(const float32 f_InputValue,
                         const float32 f_LutMinInputValue,
                         const float32 f_LutRes,
                         const sint32 s_LutMinInd,
                         const sint32 s_LutMaxInd) {
    /* LUT index */
    sint32 s_Ind = s_LutMinInd;
    if (BML_f_IsNonZero(f_LutRes)) {
        const float32 f_Ind =
            ((f_InputValue - f_LutMinInputValue) / f_LutRes) + 0.5f;

        /* Add Index Offset */
        s_Ind = ((sint32)f_Ind) + s_LutMinInd;
    }

    /* check if s_Ind is in the valid range */
    s_Ind = CML_MinMax(s_LutMinInd, s_LutMaxInd, s_Ind);

    return s_Ind;
}

/*****************************************************************************

  Functionname:    BML_s_GetLutIndexBackwards                           */ /*!

  @brief           Returns the index for a lookup table value; the input
                   value is in the domain of a LUT.

  @description     The function finds and returns the look-up table index for a 
                   given value. The result will be bounded between the maximum 
                   and minimum look-up table indices permitted.
                   The index of the look-up table value, which is more closer to
                   the given value, will be returned.

  @param[in]       f_InputValue : Input value for the Lookup Table
                                  Supported range [Full range of float32]
  @param[in]       a_LUT :        Lookup table
                                  Supported range for values in a_LUT[] 
                                  [Full range of float32]
  @param[in]       s_LutMinInd :  first index of the lookup table.
                                  Supported range [Full positive range of sint32]
  @param[in]       s_LutMaxInd :  last index of the lookup table.
                                  Supported range [Full positive range of sint32]

  @return          index of the lookup table

  @pre             -
  @post            -

**************************************************************************** */
sint32 BML_s_GetLutIndexBackwards(const float32 f_InputValue,
                                  const float32 a_LUT[],
                                  const sint32 s_LutMinInd,
                                  const sint32 s_LutMaxInd) {
    sint32 s_Result = s_LutMinInd, s_Ind;
    float32 f_tmp;

    if (f_InputValue <= a_LUT[s_LutMinInd]) {
        s_Result = s_LutMinInd;
    } else if (f_InputValue >= a_LUT[s_LutMaxInd]) {
        s_Result = s_LutMaxInd;
    } else {
        for (s_Ind = s_LutMinInd; s_Ind < s_LutMaxInd; s_Ind++) {
            if ((f_InputValue >= a_LUT[s_Ind]) &&
                (f_InputValue <= a_LUT[s_Ind + 1])) {
                f_tmp = BML_f_Abs(f_InputValue - a_LUT[s_Ind]);
                if (f_tmp < BML_f_Abs(f_InputValue - a_LUT[s_Ind + 1])) {
                    s_Result = s_Ind;
                } else {
                    s_Result = s_Ind + 1;
                }
            }
        }
    }
    return s_Result;
}

/**************************************************************

  Functionname:    CML_b_IsPointInsidePolygon            */
boolean CML_b_IsPointInsidePolygon(const float32 a_Xarray[],
                                   const float32 a_Yarray[],
                                   uint32 u_size,
                                   float32 f_Xpoint,
                                   float32 f_Ypoint) {
    boolean b_isInside = FALSE;
    boolean b_temp1;
    boolean b_temp2;
    uint32 i = 0u;

    /* Loop through all the edges */
    for (i = 1u; i < u_size; i++) {
        b_temp1 = (f_Ypoint > a_Yarray[i]) ? TRUE : FALSE;
        b_temp2 = (f_Ypoint > a_Yarray[i - 1U]) ? TRUE : FALSE;
        if (b_temp1 != b_temp2) {
            /* Find the intersection point */
            b_temp1 =
                (((f_Xpoint - a_Xarray[i]) * (a_Yarray[i - 1U] - a_Yarray[i])) <
                 ((a_Xarray[i - 1U] - a_Xarray[i]) * (f_Ypoint - a_Yarray[i])))
                    ? TRUE
                    : FALSE;
            b_temp2 = (a_Yarray[i - 1U] < a_Yarray[i])
                          ? TRUE
                          : FALSE;  // Multiplying the expression with a
                                    // negative number reverses the comparison.
            if (b_temp1 != b_temp2) {
                b_isInside = (b_isInside == TRUE) ? FALSE : TRUE;
            }
        }
    }
    return b_isInside;
}

/**************************************************************

  Functionname:    BML_f_PowerOfTwo                      */
extern float32 BML_f_PowerOfTwo(float32 f_value) {
    /* LUT for 2^(0, 1/32 ... 1-1/32) */
    static const sint32 s_MAXSHIFT = 31;
    uint16 u_idxlut;
    sint32 s_Pow;
    float32 f_overflow, f_out = 0.f;
    uint32 u_BitPow = 1u;  //
    static const float32 a_pow2LUT[N_LUT] = {
        1.0000000000000f, 1.0218971486541f, 1.0442737824274f, 1.0671404006768f,
        1.0905077326653f, 1.1143867425959f, 1.1387886347567f, 1.1637248587776f,
        1.1892071150027f, 1.2152473599805f, 1.2418578120735f, 1.2690509571917f,
        1.2968395546510f, 1.3252366431597f, 1.3542555469369f, 1.3839098819638f,
        1.4142135623731f, 1.4451808069770f, 1.4768261459395f, 1.5091644275934f,
        1.5422108254079f, 1.5759808451079f, 1.6104903319493f, 1.6457554781540f,
        1.6817928305074f, 1.7186192981225f, 1.7562521603733f, 1.7947090750031f,
        1.8340080864093f, 1.8741676341103f, 1.9152065613971f, 1.9571441241754f};

    s_Pow = (sint32)f_value;

    if (s_Pow < 0) {
        s_Pow--;
    }
    f_overflow = f_value - (float32)s_Pow;  // between 0 and 1

    u_idxlut = (uint16)CML_s_GetLutIndex(f_overflow, 0.f,
                                         (1.f / (float32)N_LUT), 0, N_LUT - 1L);

    if ((s_Pow >= (-s_MAXSHIFT)) &&
        (s_Pow <= s_MAXSHIFT))  // use uint32 Bitshift
    {
        if (s_Pow >= 0) {
            u_BitPow = 1u << s_Pow;
            f_out = (float32)u_BitPow * a_pow2LUT[u_idxlut];
        } else {
            u_BitPow = 1u << (-s_Pow);
            f_out = (a_pow2LUT[u_idxlut] / ((float32)u_BitPow));
        }
    } else  // Bitshift not possible
    {
        if (s_Pow >= 0) {
            f_out = GDBexp_power(2.f, (uint32)s_Pow) * a_pow2LUT[u_idxlut];
        } else {
            f_out = a_pow2LUT[u_idxlut] / GDBexp_power(2.f, (uint32)(-s_Pow));
        }
    }
    return f_out;
}

/***********************************************************************
  Functionname:     BML_f_XPowY                                   */
extern float32 BML_f_XPowY(const float32 f_Base, const float32 f_Exponent) {
    float32 f_XPowY = 0.0f;
    float32 f_LnOfBase = 0.0f;

    // special case for fBase = 0
    if (BML_f_Abs(f_Base) <= C_F32_DELTA) {
        f_XPowY = 0.0f;
    } else {
        f_LnOfBase = CML_f_ln(f_Base);
        f_XPowY = GDBexp((f_Exponent * f_LnOfBase));
    }

    return f_XPowY;
}

/************************************************************************
  Functionname:    CML_CalculateDistancePoint2Circle               */
BML_t_TrajRefPoint CML_CalculateDistancePoint2Circle(float32 f_X,
                                                     float32 f_Y,
                                                     float32 f_C0) {
    float32 f_Radius = RADIUS_INIT_VALUE;
    float32 f_R = 0.f;
    float32 f_DistToCourse = 0.f;
    float32 f_DistOnCourse = 0.f;
    float32 f_NormVecX = 0.f;
    float32 f_NormVecY = 0.f;
    float32 f_RefCourseDistX = 0.f;
    float32 f_RefCourseDistY = 0.f;
    BML_t_TrajRefPoint ReferencePoint;

    if (fABS(f_C0) > CURVATURE_USE_CIRCLE_EQUATION) {
        f_Radius = 1.0f / (f_C0);
        /* Object Transform to Moment Pole Coordinates */
        f_Y -= f_Radius;
        f_R = SQRT_(SQR(f_X) + SQR(f_Y));

        /* Check for divison by zero */
        /* Check for divison by zero */
        if (BML_f_IsZero(f_R)) {
            /* Distances are zero */
        } else {
            /* NormVec to Course always pointing to the left side of course */
            if (f_C0 > 0.0f) {
                f_NormVecX = -f_X / f_R;
                f_NormVecY = -f_Y / f_R;
                f_DistToCourse = (fABS(f_Radius) - f_R);
                f_DistOnCourse = f_Radius * (C_HALFPI + ATAN2_(f_Y, f_X));
            } else {
                f_NormVecX = f_X / f_R;
                f_NormVecY = f_Y / f_R;
                f_DistToCourse = -(fABS(f_Radius) - f_R);
                f_DistOnCourse = f_Radius * (ATAN2_(f_Y, f_X) - C_HALFPI);
            }
            /* DistCourse (fRadius-fR) positive when object left of course;
             * negative when object right of course*/

            f_RefCourseDistX = f_X - (f_NormVecX * f_DistToCourse);
            f_RefCourseDistY = (f_Y - (f_NormVecY * f_DistToCourse)) + f_Radius;
        }
    } else {
        /* use old parabolic approximation for wide curves and distance in Y-
         * Direction*/
        f_RefCourseDistX = f_X;
        f_RefCourseDistY = SQR(f_X) * (f_C0) * (0.5f);
        f_DistToCourse = (f_Y - f_RefCourseDistY);
        /*instead of integral 0 to fX of function SQRT(1+(fC0*x)^2) dx*/
        f_DistOnCourse = f_X;
    }
    ReferencePoint.f_X = f_RefCourseDistX;
    ReferencePoint.f_Y = f_RefCourseDistY;
    ReferencePoint.f_DistToTraj = f_DistToCourse;
    ReferencePoint.f_DistOnTraj = f_DistOnCourse;

    return ReferencePoint;
}

/************************************************************************
  Functionname:    CML_ApproximateRefpoint                         */
static float32 CML_f_ApproximateRefpoint(
    float32 f_X, float32 f_Y, float32 f_C0, float32 f_C1, float32 f_RefX) {
    float32 f_Xc = f_RefX;
    float32 f_XXc = SQR(f_Xc);
    float32 f_C1XXc = f_C1 * f_XXc;
    float32 f_C0Xc = f_C0 * f_Xc;
    float32 f_Yc = (C_SIXTH * f_C1XXc * f_Xc) + (0.5f * f_C0Xc * f_Xc);
    float32 f_m = ((0.5f * f_C1XXc) + f_C0Xc);
    float32 f_m_inv;

    if (fABS(f_m) < BML_f_Delta) {
        f_RefX = f_X;
    } else {
        f_m_inv = 1.0f / f_m;
        f_RefX =
            (((f_Y - f_Yc) + (f_m * f_Xc)) + (f_m_inv * f_X)) / (f_m + f_m_inv);
    }

    return f_RefX;
}

/************************************************************************
  Functionname:    CML_CalculateDistancePoint2Clothoid             */
BML_t_TrajRefPoint CML_CalculateDistancePoint2Clothoid(float32 f_X,
                                                       float32 f_Y,
                                                       float32 f_C0,
                                                       float32 f_C1) {
    float32 f_Temp;
    float32 f_YDiff;
    BML_t_TrajRefPoint ReferencePoint;

    ReferencePoint.f_X = f_X;
    ReferencePoint.f_X =
        CML_f_ApproximateRefpoint(f_X, f_Y, f_C0, f_C1, ReferencePoint.f_X);

    f_Temp = SQR(ReferencePoint.f_X);
    ReferencePoint.f_Y = (0.5f * f_C0 * f_Temp) +
                         ((f_C1 * f_Temp * ReferencePoint.f_X) * C_SIXTH);
    f_YDiff = f_Y - ReferencePoint.f_Y;
    ReferencePoint.f_DistToTraj =
        SQRT_(SQR(f_X - ReferencePoint.f_X) + SQR(f_YDiff));
    if (f_YDiff < 0.0f) {
        ReferencePoint.f_DistToTraj *= -1.0f;
    }

    /*@todo implement arclength on clothoid approximation*/
    /*@hack use difference in x instead*/
    ReferencePoint.f_DistOnTraj = ReferencePoint.f_X;

    return ReferencePoint;
}

// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"