

#ifndef _DIM_MOD_SPORTSTYLE_PAR_INCLUDED
#define _DIM_MOD_SPORTSTYLE_PAR_INCLUDED
#include "dim.h"

/*---- default values ----*/
/*! @brief       DIM Sportstyle Default Steering Wheel Curvature
    @general     DIM Sportstyle Default Steering Wheel Curvature
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_DEFAULT_fSteeringWheelCurvature (0.0f)
/*! @brief       DIM Sportstyle Default Vehicle Velocity
    @general     DIM Sportstyle Default Vehicle Velocity
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_DEFAULT_fVehicleVelocity (0.0f)
/*! @brief       DIM Sportstyle Default Vehicle Acceleration
    @general     DIM Sportstyle Default Vehicle Acceleration
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_DEFAULT_fVehicleAcceleration (0.0f)
/*! @brief       DIM Sportstyle Default requested Brake Torque
    @general     DIM Sportstyle Default requested Brake Torque
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_DEFAULT_fRequestedBrakeTorque (0.0f)
/*! @brief       DIM Sportstyle Default Kickdown
    @general     DIM Sportstyle Default Kickdown
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     FALSE
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_DEFAULT_bKickdown (FALSE)
/*! @brief       DIM Sportstyle Default Gas Pedal Position
    @general     DIM Sportstyle Default Gas Pedal Position
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_DEFAULT_fGasPedalPosition (0.0f)
/*! @brief       DIM Sportstyle Default Lateral Displacement
    @general     DIM Sportstyle Default Lateral Displacement
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_DEFAULT_fLateralDisplacement (0.0f)
/*! @brief       DIM Sportstyle Default LR Active
    @general     DIM Sportstyle Default LR Active
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     FALSE
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_DEFAULT_bLRActive (FALSE)
/*! @brief       DIM Sportstyle Default Lane Switch
    @general     DIM Sportstyle Default Lane Switch
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     FALSE
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_DEFAULT_bLaneSwitch (FALSE)
/*! @brief       DIM Sportstyle Default Gear Shift Active
    @general     DIM Sportstyle Default Gear Shift Active
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     FALSE
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_DEFAULT_bGearShiftActive (FALSE)

/*---- definitions ----*/
/*! @brief       DIM Sportstyle Normal Confidence Parameter
    @general     DIM Sportstyle Normal Confidence Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     100
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_PAR_NormalConfidence (100)
/*! @brief       DIM Sportstyle Missing Confidence Delta Parameter
    @general     DIM Sportstyle Missing Confidence Delta Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     10
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_PAR_MissingConfidenceDelta (10)
/*! @brief       DIM Sportstyle Very Active Percentage Parameter
    @general     DIM Sportstyle Very Active Percentage Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     70
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_PAR_VeryActivePercentage (70)

/*---- parameters ----*/
/*! @brief       DIM Sportstyle Max Lateral Acceleration Table Points Parameter
    @general     DIM Sportstyle Max Lateral Acceleration Table Points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     4
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_PAR_MaxAccLatTable_POINTS (4)
/*! @brief       DIM Sportstyle Gas Pedal Speed Table Points Parameter
    @general     DIM Sportstyle Gas Pedal Speed Table Points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     4
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_PAR_GasPedalSpeedTable_POINTS (4)
/*! @brief       DIM Sportstyle Gas Pedal Position Table Points Parameter
    @general     DIM Sportstyle Gas Pedal Position Table Points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     3
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_PAR_GasPedalPositionTable_POINTS (3)
/*! @brief       DIM Sportstyle Gas Pedal Acceleration Position Factor Table
   Points Parameter
    @general     DIM Sportstyle Gas Pedal Acceleration Position Factor Table
   Points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     2
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_PAR_GasPedalPosAccelFactorTable_POINTS (2)
/*! @brief       DIM Sportstyle Brake Torque Table Points Parameter
    @general     DIM Sportstyle Brake Torque Table Points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     2
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_PAR_BrakeTorqueTable_POINTS (2)
/*! @brief       DIM Sportstyle Brake Torque Acceleration Factor Table Points
   Parameter
    @general     DIM Sportstyle Brake Torque Acceleration Factor Table Points
   Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     2
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_PAR_BrakeTorqueAccelFactorTable_POINTS (2)
/*! @brief       DIM Sportstyle Lateral Deviation Z Pole Active Points Parameter
    @general     DIM Sportstyle Lateral Deviation Z Pole Active Points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     3
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_PAR_LatDevZpoleActive_POINTS (3)
/*! @brief       DIM Sportstyle Maximum Time Delay Steps Parameter
    @general     Number of maximun time delays, e.g. cycle_time 20ms and 50
   gives 1.0 sec.
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     NormalValue
    @unit        SI-unit

       */
#define DIM_SPORTSTYLE_PAR_MaxTimeDelay_STEPS (50)

/* ****************************************************************
    TYPEDEF STRUCT DIMSportStyleParameter_t
    **************************************************************** */
/*! @brief DIM SporsStyle Parameters Structure

    @general DIM SporsStyle Parameters Structure

    @conseq [None]

    @attention [None]

    */
typedef struct {
    BML_t_Vector2D DIM_SPORTSTYLE_PAR_MaxAccLatTable
        [DIM_SPORTSTYLE_PAR_MaxAccLatTable_POINTS]; /*!< look up table for a_lat
                                                       threshold where driver
                                                       activity is recognized
                                                       depending on velocity,
                                                       (!)>0 ; SIM:
                                                       Die_act_max_alat_table
                                                       @values:
                                                       BML_t_Vector2D[4] */

    BML_t_Vector2D DIM_SPORTSTYLE_PAR_GasPedalSpeedTable
        [DIM_SPORTSTYLE_PAR_GasPedalSpeedTable_POINTS]; /*!< look up table for
                                                           for driver activity
                                                           percentage depending
                                                           on gas pedal
                                                           velocity; SLM:
                                                           Die_act_gas_pedal_speed_table
                                                           @values:
                                                           BML_t_Vector2D[4] */

    BML_t_Vector2D DIM_SPORTSTYLE_PAR_GasPedalPositionTable
        [DIM_SPORTSTYLE_PAR_GasPedalPositionTable_POINTS]; /*!< look up table
                                                              for for driver
                                                              activity
                                                              percentage
                                                              depending on gas
                                                              pedal position;
                                                              SLM:
                                                              Die_act_gas_pedal_position_table
                                                              @values:
                                                              BML_t_Vector2D[3]
                                                              */

    BML_t_Vector2D DIM_SPORTSTYLE_PAR_GasPedalPosAccelFactorTable
        [DIM_SPORTSTYLE_PAR_GasPedalPosAccelFactorTable_POINTS]; /*!<look up
                                                                    table for
                                                                    gas pedal
                                                                    position
                                                                    activity
                                                                    factor
                                                                    depending on
                                                                    acceleration;
                                                                    SLM:
                                                                    Die_act_gas_pedal_pos_accel_factor
                                                                    @values:
                                                                    BML_t_Vector2D[2]
                                                                    */

    BML_t_Vector2D DIM_SPORTSTYLE_PAR_BrakeTorqueTable
        [DIM_SPORTSTYLE_PAR_BrakeTorqueTable_POINTS]; /*!< look up table for
                                                         brake torque activity;
                                                         SLM:
                                                         Die_act_brake_torque_table
                                                         @values:
                                                         BML_t_Vector2D[2] */

    BML_t_Vector2D DIM_SPORTSTYLE_PAR_BrakeTorqueAccelFactorTable
        [DIM_SPORTSTYLE_PAR_BrakeTorqueAccelFactorTable_POINTS]; /*!<look up
                                                                    table for
                                                                    brake torque
                                                                    activity
                                                                    factor
                                                                    depending on
                                                                    acceleration;
                                                                    SLM:
                                                                    Die_act_brake_torque_accel_factor
                                                                    @values:
                                                                    BML_t_Vector2D[2]
                                                                    */

    BML_t_Vector2D DIM_SPORTSTYLE_PAR_LatDevZpoleActiveT
        [DIM_SPORTSTYLE_PAR_LatDevZpoleActive_POINTS]; /*!< z pole value for
                                                          filtering when system
                                                          is active depending on
                                                          lateral velocity; SLM:
                                                          Die_act_lat_dev_zpole_active
                                                          --> 1/T @values:
                                                          BML_t_Vector2D[3] */

    float32 fLrwPerSIncreaseFac;
        /*!< max percentage increase per second for lrw ( %/100 ) */ /* SLM:
                                                                        Die_act_lrw_perc_per_s
                                                                        */
    float32 fLrwSWLrwGradFacT;
        /*!< transfer function first order constant for lrw signal filter ( ) */ /* SLM: Die_act_lrw_filter --> T */
    float32 fLrwSWTimeDelay;
        /*!< time delay (s) for signal to match with filtered signal (s) */ /* SLM:
                                                                               Die_act_lrw_timedelay */
    float32 fLrwSWSigmaFacT;
        /*!< transfer function first order constant for lrw sigma filter ( ) */ /* SLM: Die_act_sigma_filter --> T */
    float32 fLrwSWSigmaGain;
        /*!< gain for the sigma to compare with signal for estimate driver activity ( ) */ /* SLM: Die_act_sigma_gain */
    float32 fLrwSWSigmaThres;
        /*!< minimum sigma threshold for steering wheel curvature velocity (0.001 = 1000 m radius) (1/m), (!)>0 */ /* SLM: Die_act_min_sigma_thres */
    float32 fGradPerSIncreaseFac;
        /*!< max percentage increase per second for lrw gradient (%/100) */ /* SLM:
                                                                               Die_act_lrw_grad_perc_per_s*/
    float32 fLatDevZpoleNotActiveT;
        /*!< z pole value for filtering when system is not active; very smooth filter () */ /* SLM: Die_lat_dev_zpole_not_activ --> T */
    float32 fLaneChangeActivity;
        /*!< activity value in case of a lane change (%) */ /* SLM:
                                                               Die_act_lane_change_activity
                                                               */
    float32 fKickDownActivity;
        /*!< activity value in case of a kickdown (%) */ /* SLM: 100 */
    float32 fGearShiftActivity;
        /*!< activity value in case of a kickdown (%) */ /* SLM: 100 */
} DIMSportStyleParameter_t;

/* ****************************************************************
    TYPEDEF STRUCT DIMSportStyleDynPar_t
   **************************************************************** */
/*! @brief DIM SportStyle Dynamic Parameters Structure

    @general DIM SportStyle Dynamic Parameters Structure

    @conseq [None]

    @attention [None]

    */
typedef struct {
    /* Dynamic values */
    float32 fTimeDelayArray[DIM_SPORTSTYLE_PAR_MaxTimeDelay_STEPS]; /*!< Time
                                                                       delay
                                                                       ring
                                                                       buffer */
    uint16 iTimeDelaySteps;   /*!< iTimeDelaySteps*/
    uint16 iTimeDelayPointer; /*!<iTimeDelayPointer */
    uint16 iSPSResetCounter;  /*!<iSPSResetCounter */
    /* One step buffer */
    float32 fLatRatioPrevious;        /*!< fLatRatioPrevious*/
    float32 fSWSamplePrevious;        /*!< fSWSamplePrevious*/
    float32 fSWGradPrevious;          /*!< fSWGradPrevious*/
    float32 fSWGradLimPrevious;       /*!< fSWGradLimPrevious*/
    float32 fSWSigmaPrevious;         /*!< fSWSigmaPrevious*/
    float32 fGradRatioPrevious;       /*!< fGradRatioPrevious*/
    float32 fPAGasPedalPosPrevious;   /*!<fPAGasPedalPosPrevious */
    float32 fPAGasPedalSpeedPrevious; /*!<fPAGasPedalSpeedPrevious */
    float32 fLDLatDispPrevious;       /*!<fLDLatDispPrevious */
    float32 fLDLatDispGradPrevious;   /*!< fLDLatDispGradPrevious*/
    float32 fLDLatDispZPrevious;      /*!<fLDLatDispZPrevious */

    /* Transformed parameter to cycle interval */
    float32 fCycleTime;            /*!< fCycleTime*/
    float32 fCycleTimeTo1;         /*!< fCycleTimeTo1*/
    float32 fCycleFrequency;       /*!< fCycleFrequency*/
    float32 fLrwPerCycleIncrease;  /*!< fLrwPerCycleIncrease*/
    float32 fGradPerCycleIncrease; /*!<fGradPerCycleIncrease */
    /* PT1 Element transformations */
    float32 fSWGradX;          /*!< fSWGradX*/
    float32 fLrwSWLrwGradFacX; /*!< fLrwSWLrwGradFacX*/
    float32 fLrwSWSigmaFacX;   /*!< fLrwSWSigmaFacX*/
    float32 fPAGasPedalX;      /*!< fPAGasPedalX*/
    float32 fLDLatDispX;       /*!< fLDLatDispX*/
    BML_t_Vector2D DIM_SPORTSTYLE_PAR_LatDevZpoleActiveX
        [DIM_SPORTSTYLE_PAR_LatDevZpoleActive_POINTS]; /*!<
                                                          DIM_SPORTSTYLE_PAR_LatDevZpoleActiveX*/
    float32 fLatDevZpoleNotActiveX; /*!< fLatDevZpoleNotActiveX*/
} DIMSportStyleDynPar_t;

#endif /*_DIM_MOD_SPORTSTYLE_PAR_INCLUDED*/
