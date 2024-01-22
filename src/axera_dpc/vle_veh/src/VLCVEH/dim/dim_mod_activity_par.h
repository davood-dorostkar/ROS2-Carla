

#ifndef _DIM_MOD_DRV_ACT_PAR_INCLUDED
#define _DIM_MOD_DRV_ACT_PAR_INCLUDED
#include "dim.h"

/*---- default values ----*/

/*! @brief       DIM_ACTIVITY_DEFAULT_SteeringWheelAngle
    @typical     0.0f    */
#define DIM_ACTIVITY_DEFAULT_SteeringWheelAngle (0.0f)

/*! @brief       DIM_ACTIVITY_DEFAULT_SteeringWheelGrad
    @typical     0.0f    */
#define DIM_ACTIVITY_DEFAULT_SteeringWheelGrad (0.0f)

/*! @brief       DIM_ACTIVITY_DEFAULT_LongVelocity
    @typical     0.0f    */
#define DIM_ACTIVITY_DEFAULT_LongVelocity (0.0f)

/*---- definitions ----*/
/*! @brief       DIM_ACTIVITY_PAR_NormalConfidence
    @typical     100    */
#define DIM_ACTIVITY_PAR_NormalConfidence (100)

/*! @brief       DIM_ACTIVITY_PAR_MissingConfidenceDelta
    @typical     35    */
#define DIM_ACTIVITY_PAR_MissingConfidenceDelta (35)

/*! @brief       DIM_ACTIVITY_PAR_ActivePercentage
    @typical     30    */
#define DIM_ACTIVITY_PAR_ActivePercentage (30)

/*! @brief       DIM_ACTIVITY_PAR_VeryActivePercentage
    @typical     70    */
#define DIM_ACTIVITY_PAR_VeryActivePercentage (70)

/*! @brief       DIM_ACTIVITY_PAR_EmergenySteeringPercentage
    @typical     100    */
#define DIM_ACTIVITY_PAR_EmergenySteeringPercentage (100)

/*---- parameters ----*/

/*! @brief       DIM_ACTIVITY_PAR_SteerAngleGradThres_POINTS
    @typical     2    */
#define DIM_ACTIVITY_PAR_SteerAngleGradThres_POINTS (2)

/*! @brief       DIM_ACTIVITY_PAR_SteerAngleGradEMThres_POINTS
    @typical     2    */
#define DIM_ACTIVITY_PAR_SteerAngleGradEMThres_POINTS (2)

/*! @brief       DIM_ACTIVITY_PAR_FronSteerThres_POINTS
    @typical     2    */
#define DIM_ACTIVITY_PAR_FronSteerThres_POINTS (2)

/*! @brief       DIM_ACTIVITY_PAR_fSteeringAngleGradFilterThres_POINTS
    @typical     2    */
#define DIM_ACTIVITY_PAR_fSteeringAngleGradFilterThres_POINTS (2)

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief DIM_ACTIVITY_PAR_struct_t

    @general DIM Activity structure

    @conseq [ None ]

    @attention [ None ]

    */
typedef struct {
    /*! steering wheel angle speed threshold (?s) @values:
     * float32[DIM_ACTIVITY_PAR_SteerAngleGradThres_POINTS][2]*/
    GDBVector2_t
        fSteerAngleGradThres[DIM_ACTIVITY_PAR_SteerAngleGradThres_POINTS];
    /*! steering wheel angle speed threshold (?s) @values:
     * float32[DIM_ACTIVITY_PAR_SteerAngleGradEMThres_POINTS][2]*/
    GDBVector2_t
        fSteerAngleGradEMThres[DIM_ACTIVITY_PAR_SteerAngleGradEMThres_POINTS];
    /*! steering wheel angle threshold (? @values:
     * float32[DIM_ACTIVITY_PAR_FronSteerThres_POINTS][2]*/
    GDBVector2_t fFronSteerThres[DIM_ACTIVITY_PAR_FronSteerThres_POINTS];
    /*! steering wheel angle speed Filter threshold (?s) @values:
     * float32[DIM_ACTIVITY_PAR_fSteeringAngleGradFilterThres_POINTS][2]*/
    GDBVector2_t fSteeringAngleGradFilterThres
        [DIM_ACTIVITY_PAR_fSteeringAngleGradFilterThres_POINTS];
    float32 fFronSteerThresStraight;     /*!< steering wheel angle in straight
                                            position (?*/
    float32 fFronSteerGradThresStraight; /*!< steering wheel position constant
                                            (?s)*/
    float32 fGradShutDownTime;  /*!< hold time for the hypothesis in case of a
                                   detected driver activity due to Steering Wheel
                                   Gradient*/
    float32 fAngleShutDownTime; /*!< hold time for the hypothesis in case of a
                                   detected driver activity due to Steering
                                   Wheel Angle*/
    float32
        fGradHoldTime; /*!< hold time for the probability in case of a detected
                          driver activity due to Steering Wheel Gradient*/
    float32
        fAngleHoldTime; /*!< hold time for the probability in case of a detected
                           driver activity due to Steering Wheel Angle*/
    float32 fGradShutDownTimeEM; /*!< hold time for the hypothesis in case of a
                                    detected emergency steering situation*/
    float32
        fGradPeakTime; /*!< time for which the Steering Wheel Gradient must be
                          over the threshold to sincrease the driver activity
                          (Stabilize DIM against short gradient peak)*/
    float32
        fSteeringGradFiltHoldTime; /*!< hold time for the hypothesis in case of
                                      a detected dynamic steering anctions*/
    float32 fFilterFactorSteeringGrad; /*!< The factor of the low pass filter on
                                          the Steering Wheel Gradient */
} DIM_ACTIVITY_PAR_struct_t;

#endif /*_DIM_MOD_DRV_ACT_PAR_INCLUDED*/
