

#ifndef _DIM_MOD_LANECHANGE_PAR_INCLUDED
#define _DIM_MOD_LANECHANGE_PAR_INCLUDED
#include "dim.h"

#ifdef _DIM_MOD_DRV_SI_LACH_PAR_INCLUDED
#error \
    "Do not include Langechange and SI-Lanchange (parameter files) in the same scope without updating either function and struct names"
#endif

/*---- default values ----*/
/*! @brief       DIM Lanechange default Steering Wheel Curvature Value
    @general     DIM Lanechange default Steering Wheel Curvature Value
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_fSteeringWheelCurvature (0.0f)
/*! @brief       DIM Lanechange default vehicle Velocity
    @general     DIM Lanechange default vehicle Velocity
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_fVehicleVelocity (0.0f)
/*! @brief       DIM Lanechange default vehicle acceleration
    @general     DIM Lanechange default vehicle acceleration
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_fVehicleAcceleration (0.0f)
/*! @brief       DIM Lanechange default overall curvature
    @general     DIM Lanechange default overall curvature
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_fLCOverallCurvature (0.0f)
/*! @brief       DIM Lanechange default streeting wheel curvature standard
   deviation
    @general     DIM Lanechange default streeting wheel curvature standard
   deviation
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_fSteeringWheelCurvStdDev (0.0f)
/*! @brief       DIM Lanechange default turn indicator status
    @general     DIM Lanechange default turn indicator status
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     OFF
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_eTurnIndicator (eTurnIndicator_Off)
/*! @brief       DIM Lanechange default turn light only mode
    @general     DIM Lanechange default turn light only mode
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     lane change mode complete
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_iLCTurnLightOnlyMode (LC_mode_complete)
/*! @brief       DIM Lanechange default camera curvature
    @general     DIM Lanechange default camera curvature
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_fCameraCurvature (0.0f)
/*! @brief       DIM Lanechange default Psi
    @general     DIM Lanechange default Psi
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_fPsi (0.0f)
/*! @brief       DIM Lanechange default lateral velocity
    @general     DIM Lanechange default lateral velocity
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_fLateralVelocity (0.0f)
/*! @brief       DIM Lanechange default lane switch status
    @general     DIM Lanechange default lane switch status
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     FALSE
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_bLaneSwitch (FALSE)
/*! @brief       DIM Lanechange default LR Active
    @general     DIM Lanechange default LR Active
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     False
    @unit        SI-unit

       */
#define DIM_LANECHANGE_DEFAULT_bLRActive (FALSE)
/*! @brief       Short Description
    @general     Long and general description
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     NormalValue
    @unit        SI-unit

       */

/*---- definitions ----*/
/*! @brief       DIM Lanechange Normal Confidence Parameter
    @general     DIM Lanechange Normal Confidence Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     100
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_NormalConfidence (100)
/*! @brief       DIM Lanechange Missing Confidence Delta Parameter
    @general     DIM Lanechange Missing Confidence Delta Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     9
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_MissingConfidenceDelta (9)

/*---- parameters ----*/
/*! @brief       DIM Lanechange History to Percentage table points Parameter
    @general     DIM Lanechange History to Percentage table points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     4
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_HistoryToPercTable_POINTS (4)
/*! @brief       DIM Lanechange Turn Light to Percentage table points Parameter
    @general     DIM Lanechange Turn Light to Percentage table points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     3
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_TurnlightTimeToPercTable_POINTS (3)
/*! @brief       DIM Lanechange VY to Percentage table points Parameter
    @general     DIM Lanechange VY to Percentage table points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     2
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_VYToPercTable_POINTS (2)
/*! @brief       DIM Lanechange A Long to Percentage table points Parameter
    @general     DIM Lanechange A Long to Percentage table points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     4
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_ALongToPercTable_POINTS (4)
/*! @brief       DIM Lanechange A Long to Percentage factor table points
   Parameter
    @general     DIM Lanechange A Long to Percentage factor table points
   Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     4
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_ALongToPercFactorTable_POINTS (4)
/*! @brief       DIM Lanechange Psi to Percentage table points Parameter
    @general     DIM Lanechange Psi to Percentage table points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     2
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_PsiToPercTable_POINTS (2)
/*! @brief       DIM Lanechange CPT all table points Parameter
    @general     DIM Lanechange CPT all table points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     4
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_CptAllTable_POINTS (4)
/*! @brief       DIM Lanechange CPT cam table points Parameter
    @general     DIM Lanechange CPT cam table points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     8
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_CptCamTable_POINTS (8)
/*! @brief       DIM Lanechange CPT current table points Parameter
    @general     DIM Lanechange CPT current table points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     8
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_CptCurrentTable_POINTS (8)
/*! @brief       DIM Lanechange CPT Vdy table points Parameter
    @general     DIM Lanechange CPT Vdy table points Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     4
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_CptVdyTable_POINTS (4)
/*! @brief       DIM Lanechange Curve distance to percentage table points
   Parameter
    @general     DIM Lanechange Curve distance to percentage table points
   Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     4
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_CurveDistToPercTable_POINTS (4)
/*! @brief       DIM Lanechange Curve std deviation Z Pole table points
   Parameter
    @general     DIM Lanechange Curve std deviation Z Pole table points
   Parameter
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     2
    @unit        SI-unit

       */
#define DIM_LANECHANGE_PAR_CrvStdDevZPoleTable_POINTS (2)

/* ****************************************************************
    TYPEDEF STRUCT DIMLaneChangeParameter_t
    **************************************************************** */
/*! @brief DIM Lane Change(LC) Parameter Structure

    @general DIM Lane Change(LC) Parameter Structure

    @conseq [None]

    @attention [None]

    */
typedef struct {
    BML_t_Vector2D DIM_LANECHANGE_PAR_HistoryToPercTable
        [DIM_LANECHANGE_PAR_HistoryToPercTable_POINTS]; /*!< transfers time
                                                           since last lane
                                                           change to percentage
                                                           that lane change to
                                                           opposite direction
                                                           will happen  */

    BML_t_Vector2D DIM_LANECHANGE_PAR_TurnlightTimeToPercTable
        [DIM_LANECHANGE_PAR_TurnlightTimeToPercTable_POINTS]; /*!< transfers
                                                                 time since last
                                                                 usage of
                                                                 turnlight
                                                                 switch to
                                                                 percentage that
                                                                 the driver will
                                                                 change the lane
                                                                 to this
                                                                 direction*/

    BML_t_Vector2D DIM_LANECHANGE_PAR_TurnlightTimeToPercTable2
        [DIM_LANECHANGE_PAR_TurnlightTimeToPercTable_POINTS]; /*!< transfers
                                                                 time since last
                                                                 usage of
                                                                 turnlight
                                                                 switch to
                                                                 percentage that
                                                                 the driver will
                                                                 change the lane
                                                                 to opposite
                                                                 direction*/

    BML_t_Vector2D
        DIM_LANECHANGE_PAR_VYToPercTable
            [DIM_LANECHANGE_PAR_VYToPercTable_POINTS]; /*!< transfers lateral
                                                          velocity to lane
                                                          markers into
                                                          probability that the
                                                          driver changes the
                                                          lane in this
                                                          direction*/

    BML_t_Vector2D
        DIM_LANECHANGE_PAR_ALongToPercTable
            [DIM_LANECHANGE_PAR_ALongToPercTable_POINTS]; /*!<transfers
                                                             longitudinal
                                                             acceleration at a
                                                             speed of 30m/s into
                                                             probability that
                                                             the driver changes
                                                             the lane */

    BML_t_Vector2D
        DIM_LANECHANGE_PAR_ALongToPercFactorTable
            [DIM_LANECHANGE_PAR_ALongToPercFactorTable_POINTS]; /*!< defines a
                                                                   speed
                                                                   dependent
                                                                   factor for
                                                                   ALongToPerc
                                                                   that is used
                                                                   to increase
                                                                   the
                                                                   probability
                                                                   at lower
                                                                   acceleration
                                                                   values in
                                                                   high speed
                                                                   situations*/

    BML_t_Vector2D DIM_LANECHANGE_PAR_PsiToPercTable
        [DIM_LANECHANGE_PAR_PsiToPercTable_POINTS]; /*!< transfers angle to lane
                                                       mark into probability
                                                       that the driver changes
                                                       the lane in this
                                                       direction*/

    uint8
        DIM_LANECHANGE_PAR_CptAllTable
            [DIM_LANECHANGE_PAR_CptAllTable_POINTS]; /*!< CPT for the
                                                        combination of current
                                                        and history
                                                        probability*/

    uint8
        DIM_LANECHANGE_PAR_CptCamTable
            [DIM_LANECHANGE_PAR_CptCamTable_POINTS]; /*!< CPT for the
                                                        combination of driver
                                                        intention using camera
                                                        inputs (course, angle,
                                                        vlat) */

    uint8 DIM_LANECHANGE_PAR_CptCurrentTable
        [DIM_LANECHANGE_PAR_CptCurrentTable_POINTS]; /*!< CPT for the
                                                        combination of all
                                                        current information
                                                        (vdy, cam, turnlight)*/

    uint8 DIM_LANECHANGE_PAR_CptVdyTable
        [DIM_LANECHANGE_PAR_CptVdyTable_POINTS]; /*!< CPT for the combination of
                                                    driver intention using VDY
                                                    curvatures and driver
                                                    intention using VDY
                                                    acceleration and gas/brake
                                                    pedal position */

    BML_t_Vector2D DIM_LANECHANGE_PAR_CurveDistToPercTable
        [DIM_LANECHANGE_PAR_CurveDistToPercTable_POINTS]; /*!<convert lateral
                                                             distance between
                                                             vdy curve and lrw
                                                             curve in 1s to
                                                             probability */

    BML_t_Vector2D DIM_LANECHANGE_PAR_CrvStdDevZPoleTableT
        [DIM_LANECHANGE_PAR_CrvStdDevZPoleTable_POINTS]; /*!<  estimate a zpole
                                                            fitlering constant
                                                            for different lrw
                                                            standard deviation
                                                            values */

    float32 fTlswitchRelTimeGain;
        /*!< time elapse gain if turnligh switch is released (s) */ /* SLM:
                                                                       Die_tlswitch_rel_time_gain
                                                                       */
    float32 fTlswitchMaxOnTime;
        /*!< this is the maximum time what a turnligth switch is considered to be enabled  */ /* SLM: Die_tlswitch_wrong_side_time_decrement */
    float32 fTurnlightProb; /*!< ? (%) */ /* SLM: Die_turnlight_prob */

} DIMLaneChangeParameter_t;

#endif
