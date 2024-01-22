

#ifndef _DIM_MOD_DRV_FDBCK_PAR_INCLUDED
#define _DIM_MOD_DRV_FDBCK_PAR_INCLUDED
#include "dim.h"

/*! @brief       DIM Feedback Default Driver Breaking Value
    @general     DIM Feedback Default Driver Breaking Value
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     FALSE
    @unit        SI-unit

       */
#define DIM_FEEDBACK_DEFAULT_DrvBraking (FALSE)
/*! @brief       DIM Feedback Default Gas Pedal Position
    @general     This macro defines the default position of Gas pedal
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     95.0f
    @unit        SI-unit

       */
#define DIM_FEEDBACK_DEFAULT_GasPedalPos (95.0f)
/*! @brief       DIM Feedback Default Gas Pedal Gradient Value
    @general     DIM Feedback Default Gas Pedal Gradient Value
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0f
    @unit        SI-unit

       */
#define DIM_FEEDBACK_DEFAULT_GasPedalGrad (0.0f)
/*! @brief       DIM Feedback Default Gas Long Acceleration Value
    @general     Default Value for Long Acceleration
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0f
    @unit        SI-unit

       */
#define DIM_FEEDBACK_DEFAULT_LongAccel (0.0f)
/*! @brief       DIM Feedback Default Long Velocity Value
    @general     Default Value for Long Velocity
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0f
    @unit        SI-unit

       */
#define DIM_FEEDBACK_DEFAULT_LongVelocity (0.0f)

/*! @brief       DIM Feedback Strong Feedback Pedal Position
    @general     DIM Feedback Strong Feedback Pedal Position
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     100
    @unit        SI-unit

       */
#define DIM_FEEDBACK_PAR_FdbckStrongPos (100)
/*! @brief       DIM Feedback Feedback Pedal Position
    @general     DIM Feedback Feedback Pedal Position
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     66
    @unit        SI-unit

       */
#define DIM_FEEDBACK_PAR_FdbckPos (66)
/*! @brief       DIM Feedback Weak Feedback Pedal Position
    @general     DIM Feedback Weak Feedback Pedal Position
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     33
    @unit        SI-unit

       */

#define DIM_FEEDBACK_PAR_FdbckWeakPos (33)
/*! @brief       DIM Feedback No Feedback Pedal Position
    @general     DIM Feedback No Feedback Pedal Position
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     10
    @unit        SI-unit

       */

#define DIM_FEEDBACK_PAR_FdbckNoPos (10)
/*! @brief       DIM Feedback No Negative Feedback Pedal Position
    @general     DIM Feedback No Negative Feedback Pedal Position
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     -10
    @unit        SI-unit

       */

#define DIM_FEEDBACK_PAR_FdbckNoNeg (-10)
/*! @brief       DIM Feedback Weak Negative Feedback Pedal Position
    @general     DIM Feedback Weak Negative Feedback Pedal Position
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     -90
    @unit        SI-unit

       */

#define DIM_FEEDBACK_PAR_FdbckWeakNeg (-90)
/*! @brief       DIM Feedback Negative Feedback Pedal Position
    @general     DIM Feedback Negative Feedback Pedal Position
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     -100
    @unit        SI-unit

       */

#define DIM_FEEDBACK_PAR_FdbckNeg (-100)
/*! @brief       DIM Feedback Normal Confidence Parameter
    @general     DIM Feedback Normal Confidence Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     100
    @unit        SI-unit

       */

#define DIM_FEEDBACK_PAR_NormalConfidence (100)
/*! @brief       DIM Feedback Missing Confidence Delta Parameter
    @general     DIM Feedback Missing Confidence Delta Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     20
    @unit        SI-unit

       */

#define DIM_FEEDBACK_PAR_MissingConfidenceDelta (20)

/*---- other defines -----*/
/*! @brief       DIM_F32_DELTA
    @general     DIM_F32_DELTA Value
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0001f
    @unit        SI-unit

       */
#define DIM_F32_DELTA (0.0001f)
/*! @brief       DIM Feedback Weak Negative Curve Points
    @general     DIM Feedback Weak Negative Curve Points
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     2
    @unit        SI-unit

       */
#define DIM_FEEDBACK_PAR_WeakNegCurve_POINTS (2)
/*! @brief       DIM Feedback Gas Pedal Passive Value
    @general     DIM Feedback Gas Pedal Passive Value
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     5.0
    @unit        SI-unit

       */
#define DIM_FEEDBACK_PAR_GasPedal_Passive_Value 5.0F
/*! @brief       DIM Feedback Gas Pedal Kickdown Value
    @general     DIM Feedback Gas Pedal Kickdown Value
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     100.0
    @unit        SI-unit

       */
#define DIM_FEEDBACK_PAR_GasPedal_Kickdown_Value 100.0F

/*
  Config: Hypothesis specific configurations
**************************************************************************/

/*! Use weak driver feedback instead of "fall into brake pedal recognition"  */
/*! @brief       Short Description
    @general     Long and general description
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     NormalValue
    @unit        SI-unit

       */
#define VLC_CFG_DIM_NO_FALL_IN_BRAKE_DETECTION SWITCH_ON

/* ****************************************************************
    TYPEDEF STRUCT DIM_FEEDBACK_PAR_struct_t
    **************************************************************** */
/*! @brief DIM Pedal Feedback Parameter Structure

    @general Pedal Feedback Parameters for DIM

    @conseq [None]

    @attention [None]

    */
typedef struct {
    GDBVector2_t fWeakNegCurve
        [DIM_FEEDBACK_PAR_WeakNegCurve_POINTS]; /*!< Timer dependent weak
                                                   feedback range @values:
                                                   float32[DIM_FEEDBACK_PAR_WeakNegCurve_POINTS][2]*/
    float32 fHighGasPedalPos;  /*!< definition for high gas pedal position ->
                                  negative feedback*/
    float32 fHighGasPedalGrad; /*!< definition for high gas pedal gradient ->
                                  negative feedback*/
    float32
        fHighGasPedalPosWeakNegH; /*!< definition for high gas pedal position ->
                                     weak negative feedback (higher threshold)*/
    float32
        fHighGasPedalPosWeakNegL; /*!< definition for high gas pedal position ->
                                     weak negative feedback (lower threshold)*/
    float32 fHighGasPedalGradWeakNeg; /*!< definition for high gas pedal
                                         gradient -> weak negative feedback*/
    float32 fAutoBrakeJerkLimit;    /*!< typical jerk for autobraking in m/s^3*/
    float32 fAutoBrkTimeNoPosFdbck; /*!< time (s) where no feedback due to gas
                                       pedal falling is allowed after a high
                                       negative accel jerk (must be > 0.0!!!)*/
    float32 fVeryHighGasPedalPos;   /*!< definition for very high gas pedal
                                       position -> negative feedback without
                                       checking autobraking*/
    float32 fVeryHighGasPedalGrad;  /*!< definition for very high gas pedal
                                       gradient -> negative feedback without
                                       checking autobraking*/
    float32 fGasPedalUsedPos;       /*!< definition for a used gas pedal (%)*/
    float32 fGasPedelGradNegative;  /*!< very slight negative gradient (%/s)*/
    float32 fNegFdbckTime;          /*!< time for negative feedback (s)*/
    float32 fPosFdbckTime;          /*!< time for positive feedback (s)*/
} DIM_FEEDBACK_PAR_struct_t;

#endif /*_DIM_MOD_DRV_FDBCK_PAR_INCLUDED*/
