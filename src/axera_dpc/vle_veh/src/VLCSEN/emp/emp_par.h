
#ifndef EMP_PAR_H_INCLUDED
#define EMP_PAR_H_INCLUDED

/*****************************************************************************
  MACROS
*****************************************************************************/
/*! @brief       EMP_NUM_OBJECTS
    @general     Maximum number of EMP Objects, ie: No. of EM Objects
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     Envm_N_OBJECTS
    @unit        SI-unit

       */
#define EMP_NUM_OBJECTS (Envm_N_OBJECTS)

/*! @brief       EMP_DEFAULT_EGO_VAR_X_C0
    @general     x(t) = c2*t^2 + c1*t + c0
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define EMP_DEFAULT_EGO_VAR_X_C0 (0.0f)
/*! @brief       EMP_DEFAULT_EGO_VAR_X_C1
    @general     x(t) = c2*t^2 + c1*t + c0
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.05
    @unit        SI-unit

       */
#define EMP_DEFAULT_EGO_VAR_X_C1 (0.05f)
/*! @brief       EMP_DEFAULT_EGO_VAR_X_C2
    @general     x(t) = c2*t^2 + c1*t + c0
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define EMP_DEFAULT_EGO_VAR_X_C2 (0.0f)
/*! @brief       EMP_DEFAULT_EGO_VAR_Y_C0
    @general     x(t) = c2*t^2 + c1*t + c0
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define EMP_DEFAULT_EGO_VAR_Y_C0 (0.0f)
/*! @brief       EMP_DEFAULT_EGO_VAR_Y_C1
    @general     x(t) = c2*t^2 + c1*t + c0
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.1
    @unit        SI-unit

       */
#define EMP_DEFAULT_EGO_VAR_Y_C1 (0.1f)
/*! @brief       EMP_DEFAULT_EGO_VAR_Y_C2
    @general     x(t) = c2*t^2 + c1*t + c0
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.0
    @unit        SI-unit

       */
#define EMP_DEFAULT_EGO_VAR_Y_C2 (0.0f)

/*! @brief       EMP_YAW_RATE_ABS_MAX
    @general     EMP maximum YAW rate for ABS
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.6
    @unit        SI-unit

       */
#define EMP_YAW_RATE_ABS_MAX (0.6f)
/*! @brief       EMP_PREDICTION_TIME_EGO_MIN
    @general     EMP minimum EGO prediction time in seconds
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     1.0
    @unit        Seconds

       */
#define EMP_PREDICTION_TIME_EGO_MIN (1.0f) /* in Seconds */
/*! @brief       EMP_PREDICTION_TIME_EGO_MAX
    @general     EMP maximum EGO prediction time in seconds
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     8.0
    @unit        Seconds

       */
#define EMP_PREDICTION_TIME_EGO_MAX (8.0f) /* in Seconds */
/*! @brief       EMP_PREDICTION_OBJECTS_DEFAULT
    @general     EMP prediction Objects default time in seconds
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     2.0
    @unit        Seconds

       */
#define EMP_PREDICTION_OBJECTS_DEFAULT (2.0f) /* in Seconds */
/*! @brief       EMP_MAX_ANGLE_DEGREE
    @general     EMP maximum angle degree
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     90.0
    @unit        Degrees

       */
#define EMP_MAX_ANGLE_DEGREE (90.0f) /* in degree */

/*! @brief       EMP_OBJ_DIST_CORR_VEL2TRAJ
    @general     EMP Object distance corridor velocity to trajcotory in meters
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     5.0
    @unit        Meters

       */
#define EMP_OBJ_DIST_CORR_VEL2TRAJ (5.0f) /* in Meter */
                                          /*! @brief       EMP_DIST_TO_TRAJ_MAX
                                              @general     EMP maximum distance to trajcotory in meters
                                              @conseq      @incp
                                                           @decp
                                              @attention   [None]
                                              @typical     10.0
                                              @unit        Meters
                                          
                                                 */
#define EMP_DIST_TO_TRAJ_MAX (10.0f)      /* in Meter */
                                          /*! @brief       EMP_DIST_ON_TRAJ_MAX
                                              @general     EMP maximum distance on trajcotory in meters
                                              @conseq      @incp
                                                           @decp
                                              @attention   [None]
                                              @typical     10.0
                                              @unit        Meters
                                          
                                                 */
#define EMP_DIST_ON_TRAJ_MAX (100.0f)     /* in Meter */
/*! @brief       EMP_VELOCITY_TO_TRAJ_MAX
    @general     EMP maximum velocity to trajcotory in meter/second
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     2.0
    @unit        m/s

       */
#define EMP_VELOCITY_TO_TRAJ_MAX (2.0f)

/*! @brief       EMP_CORRIDOR_PREDICTION_STEPS
    @general     EMP no. of corridor perdiction steps
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     15
    @unit        SI-unit

       */
#define EMP_CORRIDOR_PREDICTION_STEPS (15)

/*! @brief       EMP_EGO_LONG_ACCEL_COMFORT_BRAKING
    @general     EMP long acceleration comfort braking
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     -3.0
    @unit        m/s^2

       */
#define EMP_EGO_LONG_ACCEL_COMFORT_BRAKING (-3.0f) /* in m/s^2 */
/*! @brief       EMP_EGO_LONG_ACCEL_FULL_BRAKING
    @general     EMP long acceleration full braking
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     -10.0
    @unit        m/s^2

       */
#define EMP_EGO_LONG_ACCEL_FULL_BRAKING (-10.0f) /* in m/s^2 */
/*! @brief       EMP_EGO_LONG_ACCEL_AVOIDANCE_MEDIUM_BRAKING
    @general     EMP long avoidance medium braking
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     -5.0
    @unit        m/s^2

       */
#define EMP_EGO_LONG_ACCEL_AVOIDANCE_MEDIUM_BRAKING (-5.0f) /* in m/s^2 */
/*! @brief       EMP_EGO_LAT_ACCEL_AVOIDANCE_MEDIUM_BRAKING
    @general     EMP lateral acceleration avoidance medium braking
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     2.0
    @unit        m/s^2

       */
#define EMP_EGO_LAT_ACCEL_AVOIDANCE_MEDIUM_BRAKING (2.0f) /* in m/s^2 */
/*! @brief       EMP_EGO_LONG_ACCEL_AVOIDANCE_MAX_BRAKING
    @general     EMP long acceleration avoidance maximum braking
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     -7.0
    @unit        m/s^2

       */
#define EMP_EGO_LONG_ACCEL_AVOIDANCE_MAX_BRAKING (-7.0f) /* in m/s^2 */
/*! @brief       EMP_EGO_LAT_ACCEL_AVOIDANCE_MAX_BRAKING
    @general     EMP lateral acceleration avoidance maximum braking
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     4.0
    @unit        m/s^2

       */
#define EMP_EGO_LAT_ACCEL_AVOIDANCE_MAX_BRAKING (4.0f) /* in m/s^2 */
/*! @brief       EMP_AVOIDANCE_PREDICTION_TIME
    @general     EMP avoidance prediction time
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     0.5
    @unit        Seconds

       */
#define EMP_AVOIDANCE_PREDICTION_TIME (0.5f) /* in Seconds */
/*! @brief       EMP_OBJ_BRAKE_JERK_TIME
    @general     EMP object break jerk time
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     1.0
    @unit        Seconds

       */
#define EMP_OBJ_BRAKE_JERK_TIME (1.0f) /* in Seconds */
/*! @brief       EMP_OBJ_BRAKE_JERK_MEDIUM
    @general     EMP object brake jerk medium
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     (30.0f / 3.6f)
    @unit        m/s

       */
#define EMP_OBJ_BRAKE_JERK_MEDIUM (30.0f / 3.6f) /* in m/s */
/*! @brief       EMP_OBJ_BRAKE_JERK_INTENSE
    @general     EMP object brake jerk intense
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     (50.0f / 3.6f)
    @unit        m/s

       */
#define EMP_OBJ_BRAKE_JERK_INTENSE (50.0f / 3.6f) /* in m/s */
/*! @brief       EMP_OBJ_BRAKE_DECEL_COMFORT
    @general     EMP object comfort deceleration braking
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     -3.0
    @unit        m/s^2

       */
#define EMP_OBJ_BRAKE_DECEL_COMFORT (-3.0f) /* in m/s^2 */
/*! @brief       EMP_OBJ_BRAKE_DECEL_FULL
    @general     EMP object full deceleration braking
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     -10.0
    @unit        m/s^2

       */
#define EMP_OBJ_BRAKE_DECEL_FULL (-10.0f) /* in m/s^2 */

/*!  @cond Doxygen_Suppress */
/* EMPLEGACY */
#define EMPLEG_OBJ_CLASS_MIN_CONF_WIDTH_SEC (95u)

#define EMPLEG_OBJECT_CLASS_CAR (1u)
#define EMPLEG_OBJECT_CLASS_TRUCK (2u)

#define EMPLEG_CAR_WIDTH_SEC (1.5f)
#define EMPLEG_TRUCK_WIDTH_SEC (2.0f)
#define EMPLEG_OBJECT_NO_CLASS_WIDTH_SEC (0.1f)

#define EMPLEG_COLL_MAX_TIME (1.5f)
#define EMPLEG_COLL_NUM_SIM_STEPS (50u)
#define EMPLEG_COLL_EGO_MAX_ACCEL_X_LIKE (10.0f)
#define EMPLEG_COLL_EGO_MAX_ACCEL_Y_LIKE (5.0f)
#define EMPLEG_COLL_EGO_COLL_WIDTH (0.9f)
#define EMPLEG_COLL_OBJ_MAX_ACCEL_UNAV (10.0f)
#define EMPLEG_COLL_EGO_MAX_ACCEL_X_UNAV (10.0f)
#define EMPLEG_COLL_EGO_MAX_ACCEL_Y_UNAV (8.0f)

#define EMPLEG_OBJECT_PROPERTY_STATIONARY (1)

#define EMPLEG_TRKASGN_MIN_RAD_FOR_ROAD_TRAJ (700.0f)
#define EMPLEG_TRKASGN_CURVE_OVRLAP_PRED_TIME (1.5f)

#define EMPLEG_RUN_UP_TRACK_WIDTH (1.8f)

#define EMPLEG_GEN_OBJECT_MT_STATE_DELETED (0)
#define EMPLEG_GEN_OBJECT_MT_STATE_MERGE_DELETED (4)

#define EMPLEG_COMMON_MIN_OBJ_QUALITY (48u)

#define EMPLEG_TRKASGN_MIN_TRACK_OVERLAP (0.75f)
#define EMPLEG_TRKASGN_TRACK_OVERLAP_PRED_OFFSET (-0.25f)
#define EMPLEG_TRKASGN_OBJ_OVERLAP_PRED_OFFSET (-0.35f)
#define EMPLEG_TRKASGN_STRAIGHT_OVRLAP_PRED_TIME (1.15f)
#define EMPLEG_TRKASGN_MAX_AVOID_DIST (20.0f)
#define EMPLEG_TRKASGN_MAX_AVOID_TTC (3.0f)
#define EMPLEG_TRKASGN_MIN_LAT_AVOID_SPEED (0.9f)

#define EMPLEG_AVLC_LENGTH (200.0f)
#define EMPLEG_AVLC_TRACK_WIDTH (2.2f)
#define EMPLEG_AVLC_TO_RUNUP_LENGTH (20.0f)

/*! Pedestrian Collision Zone Width */
#define EMPLEG_PED_COLL_PROB_ZONE (2.8f)

/*! @endcond */
#endif /* EMP_PAR_H_INCLUDED */
