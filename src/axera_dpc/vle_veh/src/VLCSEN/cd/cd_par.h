
#ifndef _CD_PAR_H_INCLUDED
#define _CD_PAR_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*! @brief CD parameter dependent switch */
#define CD_PAR_DEPENDENT_SWITCH(bool_condition, switch_value) \
    ((bool_condition) && (switch_value))

/*! @brief CD parameter constant if VLC_CFG_ENABLE_RAM_PARAM is true */
#define CD_PAR_CONST
/*! @brief CD parameter declaration macro function if VLC_CFG_ENABLE_RAM_PARAM
 * is true */
#define CD_DECL_PARAM(type_, name_) VLC_DECL_ADJ_PARAM(type_, name_)
/*! @brief CD parameter defination macro function if VLC_CFG_ENABLE_RAM_PARAM is
 * true */
#define CD_DEF_PARAM(type_, name_, value_) \
    VLC_DEF_ADJ_PARAM(type_, name_, value_)

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/****************************************************************
 Configuration of Kinematic Inputs
 *****************************************************************/
/* Instead of using "real" functions to get the address of a object property sub
 * structure
 *  it's possible to use MACROS instead -> could be faster...
 */
/*! @brief use data Macros */
#define CD_USE_DATA_MACROS SWITCH_ON

/*! @brief Activate for COD as the camera cannot reliably detect dynamic
 * properties */
#define CD_USE_COD_EM_OBJECTS SWITCH_OFF

/*! @brief Activate calculation of additional emergency steering attributes (TTS
 * Pre/Acu Left/Right) */
#define CD_USE_STEERING_EVASION SWITCH_OFF

/*! @brief Enable when TTC2 shall be output on interface */
#define CD_USE_HYPOTHESES_TTC2 SWITCH_ON

/*! @brief Enable when TTC3 shall be output on interface */
#define CD_USE_HYPOTHESES_TTC3 SWITCH_ON

/*! @brief Enable when TTC4 shall be output on interface */
#define CD_USE_HYPOTHESES_TTC4 SWITCH_ON
/*! @brief Enable when hypothesis lifetime shall be output on interface */
#define CD_USE_HYPOTHESES_LIFETIME SWITCH_ON
/*! @brief Use emergency steering hypothesis attribute */
#define CD_USE_EMERGENCY_STEERING SWITCH_OFF

/*! @brief Use small trajectory width for at farer range for small object like
 * bicycle */
#define CD_USE_NAR_TRAJ_RUN_UP_SMALL_OBJ SWITCH_ON
/*! @brief Use small trajectory width for at farer range for small object like
 * bicycle */
#define CD_USE_BRAKE_STROKE_INHIBTION SWITCH_ON
#if PROJECT_ZT_RAM_REDUCTION_SWITCH
/*! @brief Use SI-ACC object for ACC hypothesis */
#define CD_USE_AVLC_SI_OBJECT SWITCH_OFF
#else
/*! @brief Use SI-ACC object for ACC hypothesis */
#define CD_USE_AVLC_SI_OBJECT SWITCH_ON
#endif

/*! @brief Use TTS occupy recognition to check safe evasion side */
#define CD_USE_TTS_OCCUPY_RECOGNITION SWITCH_ON
/*! @brief Enable when TTC with brake jerk assumptions shall be calculated /
 * considered */
#define CD_USE_BRAKE_JERK_TTC SWITCH_OFF
/*! @brief Enable eba object high quality life time */
#define CD_USE_HIGH_QUALITY_LIFETIME SWITCH_OFF
/*! @brief Suppress crossing objects in case they are stopped */
#define CD_SUPPRESS_CROSSING_STOPPED_OBJECTS SWITCH_ON
/*! @brief try to calc a more safe value due to variances for values */
#define CD_USE_KINEMATIC_STDDEV SWITCH_OFF
/*! @brief At stationary targets assume TTC-filtering in OD as correct and
 * adjust distance accordingly */
#define CD_USE_VREL_COMPENSATED_DISTANCE SWITCH_OFF
/*! @brief Force Stationary and Stopped Targets to V-Abs=0 and AAbs=0 */
#define CD_FORCE_STAT_TO_ZERO_VEL SWITCH_OFF
/*! @brief Use limited dimensions for pedestrian objects */
#define CD_PED_USE_LIMITED_DIMENSIONS SWITCH_ON
/*! @brief Use limited dimensions for bicycle objects */
#define CD_BICYCLE_USE_LIMITED_DIMENSIONS SWITCH_ON
/*! @brief Use the RoadEstimation to determine if a pedestrian is on the Road or
 * not */
#define CD_PED_USE_EM_ROAD_ESTIMATION SWITCH_ON
/*! @brief Use custom anec long signal */
#define CD_USE_CUSTOM_ANEC_LONG SWITCH_OFF
/*! @brief Use the free lange check in the run-up-stationary hypothesis */
#define CD_USE_HYP_RUNUPSTATIONARY_FREE_LANE_CHECK SWITCH_OFF
/*! @brief Include TTM Calculations */
#define CD_USE_TTM_LONG SWITCH_OFF

/*! @brief Simulate additional longitudinal offset */
#define CD_USE_ADD_OFFSET_SIMU SWITCH_ON

/*! @brief Filter object movement with unknown dynamics */
#define CD_FILTER_UNKNOWN_DYNAMIC SWITCH_ON

#define VLC_CFG_CD_MSA_SELECTION SWITCH_OFF

/*! @brief Use Distance to pass in TTS calculation */
#define CD_USE_DISTANCE_TO_PASS SWITCH_ON

/****************************************************************
 General Calculation Settings
 *****************************************************************/

/*! @brief assumed latency of VDY data compared to Object data*/
#define CD_LATENCY_OBJ_ACQUISITION (0.0f)
/*! @brief assumed latency form VDY data to Bus output of VLC used to model
 * prediction for ego-dynamic and measured objects*/
#define CD_LATENCY_VLC2BRAKE (0.2f)
/*! @brief assumed latency starting from sensor data aquisition up to the brake
   system;
   used to calculate the control values (anec_long; anec_lat; closing velocity )
   */
#define CD_LATENCY_SYSTEM (CD_LATENCY_VLC2BRAKE + CD_LATENCY_OBJ_ACQUISITION)

/*! @brief Model parameter used as a offset time in calculation of anec long to
 * account for the fact that actual brake response curve is not a step function
 */
#define CD_BRK_RESPONSE_TIME (0.0f)

/*! @brief Maximum possible distance */
#define CD_DIST_MAX (200.0f)
/*! @brief Maximum time value as calculation result */
#define CD_TIME_MAX (40.0f)
/*! @brief give only needed AnecLong to pass behind a vehicle */
#define CD_USE_ANECLONG_PASS SWITCH_OFF
/*! @brief minimum lateral velocity to assume Target not stays in ego corridor*/
#define CD_ANECLONG_PASS_LATVEL_THRES (0.5f)

/*! @brief Maximum necessary longitudinal deceleration as calculation result */
#define CD_NEC_LONG_DECEL_MAX (-15.0f)
/*! @brief Virtual Distance to target for stillstand braking */
#define CD_NEC_LONG_VIRTUAL_OBJ_DIST (0.02f)
/*! @brief Maximum necessary lateral deceleration as calculation result*/
#define CD_NEC_LAT_DECEL_MAX (15.0f)

/*! @brief Longitudinal safety distance */
#define CD_LONG_SAFETY_DISTANCE (0.5f)

/*! @brief Longitudinal safety distance for customer output calculated value */
#define CD_LONG_SAFETY_DIST_CUST \
    CDParLongSafetyDistCust_c /*!< CD parameters @allow:oem_bmw */
/*! @brief Longitudinal safety distance for customer output calculated default
 * value */
#define CD_LONG_SAFETY_DIST_CUST_DEFAULT (0.5f)

/*!  @cond Doxygen_Suppress */
CD_DECL_PARAM(MEMSEC_REF float32, CD_LONG_SAFETY_DIST_CUST)
/*! @endcond */

/*! @brief Lateral safety distance without overlap of ego and object */
#define CD_LAT_SAFETY_DISTANCE (0.3f)

/*! @brief Maximum time to pass to be considered in TTS calculation [s]*/
#define CD_TTS_MAX_PASSING_TIME (1.3f)

/*! @brief Minimum lateral speed of object to consider time to pass in TTS
 * calculation [m/s]*/
#define CD_TTS_MAX_YVEL_FOR_PASSING (0.5f)

#if (CD_USE_ADD_OFFSET_SIMU)
/*! @brief      CD_LONG_OFFSET_SIMU
    @general     Simulate additional longitudinal offset, used to fake shorter
   longitudinal
                 distance for the hypothesis calculation. This can be used by
   system test to
                 simulate critical situation without risking a real accident.
                 Sign convention: + increase distance, - decrease distance
                 CD parameters allow:oem_bmw
*/
#define CD_LONG_OFFSET_SIMU CDParLongOffsetSimu_c

/*! @brief Simulate additional longitudinal offset default value */
#define CD_LONG_OFFSET_SIMU_DEFAULT (0.0f)

/*!  @cond Doxygen_Suppress */
CD_DECL_PARAM(MEMSEC_REF float32, CD_LONG_OFFSET_SIMU)
/*! @endcond */

/*! @brief      CD_LAT_OFFSET_SIMU
    @general    Simulate additional lateral offset, used to move adjacent
                lane object to host vehicle driving path. This can be used
                by system test to provoke intervention on target object beside
                without risking a real accident
                Sign convention: + shift position left, - shift position right
    @conseq     -
    @attention  -
    @typical -  @unit  -   @min -   @max -   */
#define CD_LAT_OFFSET_SIMU \
    CDParLatOffsetSimu_c /*!< CD parameters @allow:oem_bmw */

/*! @brief Simulate additional lateral offset */
#define CD_LAT_OFFSET_SIMU_DEFAULT (0.0f)

/*!  @cond Doxygen_Suppress */
CD_DECL_PARAM(MEMSEC_REF float32, CD_LAT_OFFSET_SIMU)
/*! @endcond */
#endif

/*! @brief Assumed longitudinal acceleration for emergency braking */

#define CD_EMERGENCY_BRAKE_ACCEL (-9.0f)  // default value

/*! @brief Activate calculation of object properties */
#define CD_USE_OBJ_PROP_CALCULATION SWITCH_ON

/*! @brief  Activate if lateral object acceleration from interface shall be
   used. If switched off
    the lateral acceleration is set to zero. This is appropriate for sensors
   with insufficient
    lateral measurement.
 */
#define CD_USE_REAL_LAT_OBJ_ACCEL SWITCH_ON

/*! @brief CD_USE_ABS_OBJ_VAL */
#define CD_USE_ABS_OBJ_VAL SWITCH_OFF

/*! @brief Use positive ego acceleration for prediction (if no: use 0 instead)*/
#define CD_USE_POS_LONG_EGO_ACCEL SWITCH_OFF

/*! @brief Number of value pairs in ego x-acceleration table. This table
   maps the raw ego velocity to assumed max. x-acceleration for comfort braking
 */
#define CD_NUMBER_OF_COMFORT_EGO_ACCEL_X_VALUES (2L)

/*!  @cond Doxygen_Suppress */
extern MEMSEC_REF const BML_t_Vector2D
    CD_COMFORT_EGO_ACCEL_X[CD_NUMBER_OF_COMFORT_EGO_ACCEL_X_VALUES];
/*! @endcond */

/*! @brief Number of value pairs in ego y-acceleration table. This table
    maps the raw ego velocity to assumed max. y-acceleration for
    comfort steering.
 */
#define CD_NUMBER_OF_COMFORT_EGO_VEL_FACTOR_Y_VALUES (7L)
/*!  @cond Doxygen_Suppress */
extern MEMSEC_REF const BML_t_Vector2D
    CD_COMFORT_EGO_VEL_FACTOR_Y[CD_NUMBER_OF_COMFORT_EGO_VEL_FACTOR_Y_VALUES];
/*! @endcond */

/*! @brief Number of value pairs in ego y-acceleration table. This table
    maps the raw ego velocity to assumed max. y-acceleration for
    emergency steering.
 */
#define CD_NUMBER_OF_EMERGENCY_EGO_ACCEL_Y_VALUES (2L)

/*!  @cond Doxygen_Suppress */
extern MEMSEC_REF const BML_t_Vector2D
    CD_EMERGENCY_EGO_ACCEL_Y[CD_NUMBER_OF_EMERGENCY_EGO_ACCEL_Y_VALUES];
/*! @endcond */

/*parameter for TTS Pre overlap*/

/*! @brief CD lateral velocity limit  */
#define CD_V_LAT_PRE_LIMIT (2.0f)

/*! @brief CD max lateral acceleration */
#define CD_A_LAT_PRE_LIMIT (3.5f)

/*! @brief CD lower acceleration threshold*/
#define CD_A_LAT_PRE_MIN (1.0f)

/*--- Brake stroke inhibition parameters ---*/
#if (CD_USE_BRAKE_STROKE_INHIBTION)
/*! @brief Run-up braking hypothesis probability reduction factor over changed
 * vrelX */
#define CD_RUNUP_BRK_HYP_PROB_RED_OVER_VREL_NO (3L)
extern MEMSEC_REF CD_PAR_CONST BML_t_Vector2D
    CD_RUNUP_BRK_PROB_VREL_RED[CD_RUNUP_BRK_HYP_PROB_RED_OVER_VREL_NO];

/*! @brief Min. time gap (based on VrelX) to activate hypothesis probability
 * reduction */
#define CD_RUNUP_BRK_TIME_GAP CDRunUpBrkTimeGap_c

/*! @brief Min. time gap (based on VrelX) default value */
#define CD_RUNUP_BRK_TIME_GAP_DEFAULT (5.0f)

/*!  @cond Doxygen_Suppress */
CD_DECL_PARAM(MEMSEC_REF float32, CD_RUNUP_BRK_TIME_GAP)
/*! @endcond */

/*! @brief Run-up hypothesis parameter number for the time-gap/ego velocity
 * table */
#define CD_RUNUP_BRK_VEGO_TGAP_NO (3L)

/*! Min. time gap (based on Vego) to freeze VrelX */
extern MEMSEC_REF CD_PAR_CONST BML_t_Vector2D
    CD_RUNUP_BRK_VEGO_TGAP_UPPER[CD_RUNUP_BRK_VEGO_TGAP_NO];
/*! @brief Min. time gap (based on Vego) to activate hypothesis probability
 * reduction */
extern MEMSEC_REF CD_PAR_CONST BML_t_Vector2D
    CD_RUNUP_BRK_VEGO_TGAP[CD_RUNUP_BRK_VEGO_TGAP_NO];

/*! @brief Cut-in hypothesis parameter number for the time-gap/ego velocity
 * table */
#define CD_CUTIN_VEGO_TGAP_NO CD_RUNUP_BRK_VEGO_TGAP_NO

/*! @brief Cut-in hypothesis parameter for the table of time gap over ego
 * velocity */
#define CD_CUTIN_VEGO_TGAP_IN CD_RUNUP_BRK_VEGO_TGAP
#endif

#if (CD_PED_USE_EM_ROAD_ESTIMATION)
/*! @brief Minimum Road Estimation Tracking Status to consider Road Estimation
 * for Pedestrian on Road */
#define CD_PED_ROAD_EST_MIN_TRACKSTAT (5u)
/*! @brief Minimal confidence for the Road Estimation to be considered for
 * Pedestrian on Road */
#define CD_PED_ROAD_EST_MIN_CONF (60.f) /* % */
/*! @brief Minimum lateral distance to the road border to consider the
 * pedestrian on the road */
#define CD_PED_ROAD_MIN_DIST_TO_BORDER (0.7f)

/*! @brief Minimum Range of the Road Estimation to conmsider valid */
#define CD_PED_ROAD_MIN_RANGE (30.0f)

/*! @brief Min lat vel of ped outside of considered ego path
 * CD_PED_MIN_VEL_CORRIDOR_WIDTH below CD_PED_MIN_LAT_VEL_THRES*/
#define CD_PED_MIN_LAT_VEL_LOW (2.0f / 3.6f) /* m/s */

/*! @brief Min lat vel of ped outside of considered ego path
 * CD_PED_MIN_VEL_CORRIDOR_WIDTH above CD_PED_MIN_LAT_VEL_THRES*/
#define CD_PED_MIN_LAT_VEL_HIGH (3.0f / 3.6f) /* m/s */

/*! @brief Threshold long ego vel for usage of CD_PED_MIN_LAT_VEL_LOW or
 * CD_PED_MIN_LAT_VEL_HIGH*/
#define CD_PED_LAT_VEL_THRES (25.0f / 3.6f) /* m/s */

/*! @brief Ego path outside which objects with low lat vel are not considered */
#define CD_PED_MIN_VEL_CORRIDOR_WIDTH (EGO_VEHICLE_WIDTH) /* m */

#endif /* CD_PED_USE_EM_ROAD_ESTIMATION */

/****************************************************************
  Object Width Settings
 *****************************************************************/

/*! @brief      CD_USE_REAL_OBJ_WIDTH_VAR
    @general    activate if the real object width variance shall be used.
                If deactivated a constant variance is being used
    @conseq     -
    @attention  if set to 1 make internal copy of object dimension structure
    @typical -  @unit  -   @min -   @max -   */
#define CD_USE_REAL_OBJ_WIDTH_VAR SWITCH_OFF

/*! @brief      CD_CONST_OBJ_WIDTH_VAR
    @general    Constant object width variance. Shall be used only
                if CD_USE_REAL_OBJ_WIDTH_VAR == SWITCH_OFF
    @conseq     -
    @attention  -
    @typical -  @unit  -   @min -   @max -   */
#define CD_CONST_OBJ_WIDTH_VAR (0.11f)

/*! @brief      CD_OBJ_CLASS_MIN_CONF_WIDTH_SEC
    @general    Minimum object classification sureness for the usage off class
                for the detection of the object width Secure width (smaller is
                safer too avoid false detections, used for collision / collision
   unavoidable)
    @conseq     -
    @attention  -
    @typical -  @unit  -   @min -   @max -   */
#define CD_OBJ_CLASS_MIN_CONF_WIDTH_SEC (95u)

/*! @brief Maximal size of a car */
#define CD_CAR_WIDTH_SEC (1.5f)

/*! @brief Maximal size of a truck */
#define CD_TRUCK_WIDTH_SEC (2.0f)

/*! @brief Maximal size of an unclassified object */
#define CD_OBJECT_NO_CLASS_WIDTH_SEC (0.1f)

#if (CD_PED_USE_LIMITED_DIMENSIONS)
/*! @brief Maximal size of a pedestrian object-width*/
#define CD_PEDESTRIAN_WIDTH_SEC (0.6f)

/*! @brief Maximal size of a pedestrian object-length*/
#define CD_PEDESTRIAN_LENGTH_SEC (0.6f)
#endif

// wulin add 20220316
#if (CD_BICYCLE_USE_LIMITED_DIMENSIONS)
/*! @brief Maximal size of a bicycle object-width*/
#define CD_BICYCLE_WIDTH_SEC (0.6f)
/*! @brief Maximal size of a bicycle object-length*/
#define CD_BICYCLE_LENGTH_SEC (2.0f)
#endif
/****************************************************************
  Common Hypothesis Settings
 *****************************************************************/

/*! @brief threshold for object relevance based on TTC */
#define CD_COMMON_TTC_THRES \
    (8.0f)  // 4.0f)change for less deceleration situation by guotao 20191121

/*! @brief threshold for object relevance based on TTB */
#define CD_COMMON_TTB_THRES \
    (8.0f)  // 4.0f)change for less deceleration situation by guotao 20191121

/*! @brief threshold for object relevance based on necessary longitudinal
 * acceleration */
#define CD_COMMON_ANECLONG_THRES (0.0f)

/*! @brief minimum object probability */
#define CD_COMMON_MIN_OBJ_QUALITY (48u)

/*! @brief minimum object probability */
#define CD_COMMON_MIN_OBJ_QUALITY_POINT (97u)

/*! @brief minimum hypothesis probability for hypothesis to be considered */
#define CD_COMMON_MIN_HYP_PROB (0.5f)

/*! @brief minimum hypothesis probability to treat hyp. as relevant in
 * hypothesis selection*/
#define CD_COMMON_HYP_PROB_IRREL_THRES (0.5f)

/*! @brief Assumed ego width */
#define CD_COMMON_EGO_WIDTH (1.8f)

/*! @brief Plausibility ego width */
#define CD_PLAUS_EGO_WIDTH (1.2f)

/*! @brief Assumed ego width */
#define CD_COMMON_USE_REAL_EGO_DIMENSION SWITCH_ON
/*! @brief Assumed ego length */
#define CD_COMMON_EGO_LENGTH (4.5f)

/*! @brief Plausibility ego length */
#define CD_PLAUS_EGO_LENGTH (2.0f)
/*! @brief The stopped confidence threshold for CD */
#define CD_COMMON_MIN_STOPPED_CONF (50u)

/*! @brief Parameter to modify ego vehicle driving path width. This can be used
 * to get earlier function reaction */
#define CD_COMMON_TRACK_WIDTH_FACT CDComTrackWidthFact_c

/*! @brief Deault value of the parameter used to modify ego vehicle driving path
 * width.*/
#define CD_COMMON_TRACK_WIDTH_FACT_DEFAULT (1.0f)

/*! @brief  CD parameter declaration macro function :
 * CD_COMMON_TRACK_WIDTH_FACT*/

/*!  @cond Doxygen_Suppress */
CD_DECL_PARAM(MEMSEC_REF float32, CD_COMMON_TRACK_WIDTH_FACT)
/*! @endcond */

/*! @brief minimum Object DistX to create a new Hypothesis for an object */
#define CD_COMMON_MIN_DISTX (1.0f)

/****************************************************************
  Follow Hypothesis Settings
 *****************************************************************/
/*! @brief Enable Follow Hypothesis Evaluation */
#define CD_USE_FOLLOW_HYPOTHESIS SWITCH_ON

/*! @brief Min. EBA generic quality */
#define CD_FOLLOW_MIN_EBA_GEN_OBJ_QUAL (CD_COMMON_MIN_OBJ_QUALITY + 1u)

/*! @brief Min. EBA stationary quality */
#define CD_FOLLOW_MIN_EBA_STAT_OBJ_QUAL (0u)

/*! @brief Min. EBA pedestrian quality */
#define CD_FOLLOW_MIN_EBA_PED_OBJ_QUAL (0u)
/*! @brief Number of object filters for follow hypothesis */
#define CD_FOLLOW_CLASS_FILTERS_NO (1u)
/*! @brief threshold for object relevance based on distance*/
#define CD_FOLLOWING_MAX_DIST (40.0f)
/*! @brief relative velocity threshold for the detection of following situation
 * (lower limit) */
#define CD_FOLLOWING_MIN_VREL (-4.0f)
/*! @brief relative velocity threshold for the detection of following situation
 * (upper limit) */
#define CD_FOLLOWING_MAX_VREL (0.0f)
/*! @brief relative velocity threshold for the detection (keep) of following
 * situation (upper limit) */
#define CD_FOLLOWING_MAX_VREL_KEEP (2.0f)
/*! @brief minimum object assignments to track */
#define CD_FOLLOWING_MIN_TRACK_ASSIGNED (1u << 7u)
/*! @brief Threshold for pre-selection of objects for follow hypothesis */
#define CD_FOLLOW_MIN_OBJ_CLASS_CONF (80u)

/****************************************************************
  Run-up Hypothesis Settings
 *****************************************************************/
/*! @brief Enable Run-up Hypothesis Evaluation */
#define CD_USE_RUN_UP_HYPOTHESIS SWITCH_ON
/*! @brief Min. EBA generic quality */
#define CD_RUN_UP_MIN_EBA_GEN_OBJ_QUAL (CD_COMMON_MIN_OBJ_QUALITY + 1u)

/*! @brief threshold for object relevance based on necessary longitudinal
 * acceleration in runup hypothese */
#define CD_RUNUP_ANECLONG_THRES (CD_COMMON_ANECLONG_THRES)

/*! @brief relative velocity threshold in runup hypothese
    @general   -1000 disables the functionaltiy in cd_hyprunup of the velocity
   threshold   */
#define CD_RUNUP_V_REL_X_THRES (0.0f)

/*! @brief Min. EBA stationary quality */
#define CD_RUN_UP_MIN_EBA_STAT_OBJ_QUAL (0u)
/*! @brief Min. EBA pedestrian quality */
#define CD_RUN_UP_MIN_EBA_PED_OBJ_QUAL (0u)
/*! @brief Number of object filters for run-up hypothesis */
#define CD_RUN_UP_CLASS_FILTERS_NO (0u)
/*! @brief maximal distance to course for run-up hypothesis (prefiltering) */
#define CD_RUN_UP_MAX_LAT_DIST (2.0f)
/*! @brief run-up track width */
#define CD_RUN_UP_TRACK_WIDTH (CD_COMMON_EGO_WIDTH * CD_COMMON_TRACK_WIDTH_FACT)
/*! @brief maximal propagation time for RUN UP hypothesis*/
#define CD_RUN_UP_MAX_PRED_TIME (1.0f)
/*! @brief minimum object assignments to track */
#define CD_RUN_UP_MIN_TRACK_ASSIGNED (7u << 5u)
/*! @brief minimum CGEB quality for track assignment of stationary objects */
#define CD_RUN_UP_MIN_TRACK_STAT_OBJ_QUALITY (50u)
/*! @brief Threshold for pre-selection of objects for run-up hypothesis */
#define CD_RUN_UP_MIN_OBJ_CLASS_CONF (0u)

/* Overlap thresholds now and TTC */
/*! @attention CD_RUN_UP_THRES_OVLC_1_NOW and CD_RUN_UP_THRES_OVLC_0_NOW must
 * not be equal.
 * This can cause a Div-by-0 exception.
 */
/*! @brief Overlap thresholds now and TTC 0 */
#define CD_RUN_UP_THRES_OVLC_0_NOW (0.2f)
/*! @brief Overlap thresholds now and TTC 1 */
#define CD_RUN_UP_THRES_OVLC_1_NOW (0.4f)
/*! @attention CD_RUN_UP_THRES_OVLC_0_TTC and CD_RUN_UP_THRES_OVLC_1_TTC must
 * not be equal.
 * This can cause a Div-by-0 exception.
 */
/*! @brief CD_RUN_UP_THRES_OVLC_0_TTC*/
#define CD_RUN_UP_THRES_OVLC_0_TTC (0.1f)
/*! @brief CD_RUN_UP_THRES_OVLC_1_TTC*/
#define CD_RUN_UP_THRES_OVLC_1_TTC (0.3f)

/*! @brief CD_RUN_UP_THRES_DIST_X0*/
#define CD_RUN_UP_THRES_DIST_X0 (50.0f)
/*! @brief CD_RUN_UP_THRES_DIST_Y0*/
#define CD_RUN_UP_THRES_DIST_Y0 (1.5f)
/*! @brief CD_RUN_UP_THRES_DIST_X1 */
#define CD_RUN_UP_THRES_DIST_X1 (130.0f)
/*! @brief CD_RUN_UP_THRES_DIST_Y1 */
#define CD_RUN_UP_THRES_DIST_Y1 (0.5f)

/*! @brief CD_RUN_UP_THRES_SPEED_X0 */
#define CD_RUN_UP_THRES_SPEED_X0 (30.0f) /*108km/h*/
/*! @brief CD_RUN_UP_THRES_SPEED_Y0 */
#define CD_RUN_UP_THRES_SPEED_Y0 (1.5f)
/*! @brief CD_RUN_UP_THRES_SPEED_X1 */
#define CD_RUN_UP_THRES_SPEED_X1 (61.0f) /*220km/h*/
/*! @brief CD_RUN_UP_THRES_SPEED_Y1 */
#define CD_RUN_UP_THRES_SPEED_Y1 (0.5f)

/*! @brief CD_RUN_UP_THRES_ACCEL_X0 */
#define CD_RUN_UP_THRES_ACCEL_X0 (-6.0f)
/*! @brief CD_RUN_UP_THRES_ACCEL_Y0 */
#define CD_RUN_UP_THRES_ACCEL_Y0 (2.0f)
/*! @brief CD_RUN_UP_THRES_ACCEL_X1 */
#define CD_RUN_UP_THRES_ACCEL_X1 (-1.0f)
/*! @brief CD_RUN_UP_THRES_ACCEL_Y1 */
#define CD_RUN_UP_THRES_ACCEL_Y1 (1.0f)

/*! @brief deceleration of object for Run-up Hypothesis
 * (Adapted from -2.0 to -1.7mpss because of NCAP IUBraking)
 */
#define CD_RUN_UP_OBJ_DECEL_BRAKING (-1.7f)

/*! Enable/disable keep optimization for run-up scenarios
 *
 * Keep objects in narrow curves at a max. distance/TTC,
 * when they follow the curve.
 */
/*! @brief CD_RUN_UP_USE_NCURVE_KEEP */
#define CD_RUN_UP_USE_NCURVE_KEEP SWITCH_ON
/*! @brief CD_RUN_UP_NCURVE_MAX_DIST */
#define CD_RUN_UP_NCURVE_MAX_DIST (20.0F)
/*! @brief CD_RUN_UP_NCURVE_MAX_TTC */
#define CD_RUN_UP_NCURVE_MAX_TTC (3.0F)
/*! @brief CD_RUN_UP_NCURVE_C0_MIN */
#define CD_RUN_UP_NCURVE_C0_MIN (1.0f / 155.0f)
/*! @brief CD_RUN_UP_NCURVE_DIST2TRAJ2 */
#define CD_RUN_UP_NCURVE_DIST2TRAJ2 (0.5F)
/*! @brief CD_RUN_UP_NCURVE_ORIENT */
#define CD_RUN_UP_NCURVE_ORIENT (DEG2RAD(5.0F))

/*--- Pass by small objects inhibition parameters ---*/
/*! @brief Minimum required distance for narrowing trajectory width for small
 * objects */
#define CD_RUNUP_SOBJ_HYP_OVLC_VEGO_DIST_NO (3L)
extern MEMSEC_REF CD_PAR_CONST BML_t_Vector2D
    CD_RUNUP_SOBJ_OVLC_VEGO_DIST[CD_RUNUP_SOBJ_HYP_OVLC_VEGO_DIST_NO];

/*! @brief Trajectory width narrowing factors */
#define CD_RUNUP_SOBJ_HYP_OVLC_RED_OVER_DIST_NO (2L)
extern MEMSEC_REF BML_t_Vector2D
    CD_RUNUP_SOBJ_HYP_OVLC_DIST_RED[CD_RUNUP_SOBJ_HYP_OVLC_RED_OVER_DIST_NO];

/*! @brief Transition section for trajectory width reduction */
#define CD_RUNUP_SOBJ_HYP_OVLC_DIST_RED_FACT CDParRunUpSObjRedDistFact_c
VLC_DECL_FIX_PARAM(MEMSEC_REF float32, CD_RUNUP_SOBJ_HYP_OVLC_DIST_RED_FACT)

/*! Narrow track width depending on ego trajectory curvature/gradient
 *
 * @attention The logic for the calculation of the curvature/gradient is wrong
 * (see CGEB ActionList)
 */
#define CD_RUN_UP_TTC_WIDTH_NO_POINTS (2L)

/*! @brief CD_RUN_UP_TTC_WIDTH_FAC_NO_POINTS */
#define CD_RUN_UP_TTC_WIDTH_FAC_NO_POINTS (2L)

/*! @brief CD_RUN_UP_TTC_WIDTH_FAC_DEF  */
#define CD_RUN_UP_TTC_WIDTH_FAC_DEF (1.0F)
/*!  @cond Doxygen_Suppress */
extern MEMSEC_REF const BML_t_Vector2D
    CD_RUN_UP_TTC_TRACK_WIDTH[CD_RUN_UP_TTC_WIDTH_NO_POINTS];
extern MEMSEC_REF const BML_t_Vector2D
    CD_RUN_UP_TTC_TRACK_WIDTH_WIDE[CD_RUN_UP_TTC_WIDTH_FAC_NO_POINTS];
/*! @endcond */

/****************************************************************
  Pass Hypothesis Settings
 *****************************************************************/
/*! @brief Enable Pass Hypothesis Evaluation */
#define CD_USE_PASS_HYPOTHESIS SWITCH_OFF
/*! @brief Min. EBA generic quality */
#define CD_PASS_MIN_EBA_GEN_OBJ_QUAL (CD_COMMON_MIN_OBJ_QUALITY + 1u)
/*! @brief Min. EBA stationary quality */
#define CD_PASS_MIN_EBA_STAT_OBJ_QUAL (0u)
/*! @brief Min. EBA pedestrian quality */
#define CD_PASS_MIN_EBA_PED_OBJ_QUAL (0u)
/*! @brief Number of object filters for cut-in hypothesis */
#define CD_PASS_CLASS_FILTERS_NO (1u)
/*! @brief maximal distance to course for run-up hypothesis */
#define CD_PASS_MAX_LAT_DIST (7.0f)
/*! @brief PASS Track width */
#define CD_PASS_TRACK_WIDTH (9.0f)
/*! @brief PASS Track width */
#define CD_PASS_TRACK_WIDTH_TTC (12.0f)
/*! @brief maximal propagation time */
#define CD_PASS_MAX_PRED_TIME (0.5f)
/*! @brief minimal necessary deceleration*/
#define CD_PASS_ANECLONG_THRES (-3.0f)
/*! @brief minimal necessary deceleration (keep)*/
#define CD_PASS_ANECLONG_THRES_KEEP (-2.5f)
/*! @brief minimum class confidence of pass object */
#define CD_PASS_MIN_OBJ_CLASS_CONF (80u)

/* Overlap thresholds now and TTC */
/*! @attention CD_PASS_THRES_OVLC_1_NOW and CD_PASS_THRES_OVLC_0_NOW must not be
 * equal.
 * This can cause a Div-by-0 exception.
 */
/*! @brief Overlap thresholds now and TTC 0 */
#define CD_PASS_THRES_OVLC_0_NOW (0.0f)
/*! @brief Overlap thresholds now and TTC 1 */
#define CD_PASS_THRES_OVLC_1_NOW (0.2f)

/*! @attention CD_PASS_THRES_OVLC_1_TTC and CD_PASS_THRES_OVLC_0_TTC must not be
 * equal.
 * This can cause a Div-by-0 exception.
 */
/*! @brief Overlap thresholds TTC 0 */
#define CD_PASS_THRES_OVLC_0_TTC (0.0f)
/*! @brief Overlap thresholds TTC 1 */
#define CD_PASS_THRES_OVLC_1_TTC (0.2f)

/****************************************************************
  Cut-in Hypothesis Settings
 *****************************************************************/
/*! @brief Enable Pass Hypothesis Evaluation */
#define CD_USE_CUT_IN_HYPOTHESIS SWITCH_OFF
/*! @brief Min. EBA generic quality */
#define CD_CUT_IN_MIN_EBA_GEN_OBJ_QUAL (CD_COMMON_MIN_OBJ_QUALITY + 1u)
/*! @brief Min. EBA stationary quality */
#define CD_CUT_IN_MIN_EBA_STAT_OBJ_QUAL (0u)
/*! @brief Min. EBA pedestrian quality */
#define CD_CUT_IN_MIN_EBA_PED_OBJ_QUAL (0u)
/*! @brief Number of object filters for cut-in hypothesis */
#define CD_CUT_IN_CLASS_FILTERS_NO (1u)
/*! @brief Cut-in Track width for estimated position*/
#define CD_CUT_IN_TRACK_WIDTH (CD_COMMON_EGO_WIDTH * CD_COMMON_TRACK_WIDTH_FACT)
/*! @brief Cut-in Track width for estimated position*/
#define CD_CUT_IN_TRACK_WIDTH_NOW (6.0f)
/*! @brief maximal prediction time */
#define CD_CUT_IN_MAX_PRED_TIME (1.0f)
/*! @brief minimum class confidence of cut-in object */
#define CD_CUT_IN_MIN_OBJ_CLASS_CONF (80u)
/*! @brief TTC threshold for cut-in object filter */
#define CD_CUT_IN_TTC_THRES (CD_COMMON_TTC_THRES)
/*! @brief TTB threshold for cut-in object filter */
#define CD_CUT_IN_TTB_THRES (CD_COMMON_TTB_THRES)
/*! @brief ANecLong threshold for cut-in object filter */
#define CD_CUT_IN_ANECLONG_THRES (CD_COMMON_ANECLONG_THRES)

/*! @brief Check if neighbor object is running up an object ahead and thus
 * needs to cut into the ego lane.
 *
 * @attention Do not use this feature. It is not parameterized yet.
 */
#define CD_CUT_IN_CHECK_NBOR_OBJ_RUN_UP SWITCH_OFF
/*! @brief Minimum lane association probability of neighbor objects */
#define CD_CUT_IN_NBOR_OBJ_RUN_UP_MIN_LANE_ASSOC_PROB (80u)
/*! @brief Minimum lane association confidence of neighbor objects */
#define CD_CUT_IN_NBOR_OBJ_RUN_UP_MIN_LANE_ASSOC_CONF (0u)

#if (CD_CUT_IN_CHECK_NBOR_OBJ_RUN_UP)
/*!  Neighbur object run-up cut in prediction */
#define CD_CUT_IN_NBOR_OBJ_RUN_UP_NO (5L)
extern MEMSEC_REF const BML_t_Vector2D CD_CUT_IN_NBOR_OBJ_RUN_UP_FAR_TTC[];
extern MEMSEC_REF const BML_t_Vector2D CD_CUT_IN_NBOR_OBJ_RUN_UP_NEAR_TTC[];
#endif /* CD_CUT_IN_CHECK_NBOR_OBJ_RUN_UP */

/* Overlap thresholds now and TTC */
/*! @attention CD_RUN_UP_THRES_OVLC_1_NOW and CD_RUN_UP_THRES_OVLC_0_NOW must
 * not be equal.
 * This can cause a Div-by-0 exception.
 */
#define CD_CUT_IN_THRES_OVLC_0_NOW CD_RUN_UP_THRES_OVLC_0_NOW
/*! @attention CD_RUN_UP_THRES_OVLC_1_NOW and CD_RUN_UP_THRES_OVLC_0_NOW must
 * not be equal.
 * This can cause a Div-by-0 exception.
 */
#define CD_CUT_IN_THRES_OVLC_1_NOW CD_RUN_UP_THRES_OVLC_1_NOW
/*! @attention CD_RUN_UP_THRES_OVLC_0_TTC and CD_RUN_UP_THRES_OVLC_1_TTC must
 * not be equal.
 * This can cause a Div-by-0 exception.
 */
#define CD_CUT_IN_THRES_OVLC_0_TTC CD_RUN_UP_THRES_OVLC_0_TTC
/*! @attention CD_RUN_UP_THRES_OVLC_0_TTC and CD_RUN_UP_THRES_OVLC_1_TTC must
 * not be equal.
 * This can cause a Div-by-0 exception.
 */
#define CD_CUT_IN_THRES_OVLC_1_TTC CD_RUN_UP_THRES_OVLC_1_TTC

/*! @attention CD_CUT_IN_THRES_TRAJC0_1 and CD_CUT_IN_THRES_TRAJC0_0 must not be
 * equal.
 * This can cause a Div-by-0 exception.
 */
/*! @brief CD_CUT_IN_THRES_TRAJC0_0 */
#define CD_CUT_IN_THRES_TRAJC0_0 (0.002f)
/*! @brief CD_CUT_IN_THRES_TRAJC0_1 */
#define CD_CUT_IN_THRES_TRAJC0_1 (0.001f)

/****************************************************************
  ACC Hypothesis Settings
 *****************************************************************/

/*! @brief CD_USE_AVLC_HYPOTHESIS */
#define CD_USE_AVLC_HYPOTHESIS SWITCH_OFF

/*! @brief CD_AVLC_MIN_EBA_GEN_OBJ_QUAL */
#define CD_AVLC_MIN_EBA_GEN_OBJ_QUAL (CD_COMMON_MIN_OBJ_QUALITY + 1u)
/*! @brief CD_AVLC_MIN_EBA_STAT_OBJ_QUAL */
#define CD_AVLC_MIN_EBA_STAT_OBJ_QUAL (0u)
/*! @brief CD_AVLC_MIN_EBA_PED_OBJ_QUAL*/
#define CD_AVLC_MIN_EBA_PED_OBJ_QUAL (0u)
/*! @brief CD_AVLC_CLASS_FILTERS_NO */
#define CD_AVLC_CLASS_FILTERS_NO (1u)

/*! @brief ACC Track width */
#define CD_AVLC_TRACK_WIDTH (2.2f * CD_COMMON_TRACK_WIDTH_FACT)
/*! @brief ACC Track width */
#define CD_AVLC_LENGTH CD_DIST_MAX
/*! @brief ACC Track width */
#define CD_AVLC_TO_RUNUP_LENGTH (20.0f)
/*! @brief minimum object assignments to track */
#define CD_AVLC_MIN_TRACK_ASSIGNED (1u << 7u)
/*! @brief Threshold for pre-selection of objects for acc hypothesis */
#define CD_AVLC_MIN_OBJ_CLASS_CONF (80u)

/****************************************************************
  Collision / Collision Unavailable Hypothesis Settings
 *****************************************************************/
/*! @brief Enable Collision Hypothesis Evaluation */
#define CD_USE_COLL_HYPOTHESIS SWITCH_OFF
/*! @brief Min. EBA generic quality */
#define CD_COLL_MIN_EBA_GEN_OBJ_QUAL (CD_COMMON_MIN_OBJ_QUALITY + 1u)
/*! @brief Min. EBA stationary quality */
#define CD_COLL_MIN_EBA_STAT_OBJ_QUAL (0u)
/*! @brief Min. EBA pedestrian quality */
#define CD_COLL_MIN_EBA_PED_OBJ_QUAL (0u)
/*! @brief Number of object filters for follow hypothesis */
#define CD_COLL_CLASS_FILTERS_NO (1u)
/*! @brief Threshold for the pre selection of objects for the collision
 * hypothesis */
#define CD_COLL_TTB_TTS_THRES (0.5f)
/*! @brief Threshold for the pre selection of objects for the collision
 * hypothesis */
#define CD_COLL_TTC_THRES (1.0f)
/*! @brief Size of the ego vehicle collision width used for collision hypothesis
 * (half of the vehicle size) */
#define CD_COLL_EGO_COLL_WIDTH (VLC_fEgoVehicleWidth * 0.5f)
/*! @brief number of time steps, which are evaluated */
#define CD_COLL_NUM_SIM_STEPS (50u)
/*! @brief Threshold for pre-selection of objects for collision hypothesis */
#define CD_COLL_MIN_OBJ_CLASS_CONF (80u)
/*! @brief assumed object acceleration */
#define CD_COLL_OBJ_MAX_ACCEL_UNAV (10.0f)
/*! @brief assumed longitudinal ego acceleration for collision unavoidable */
#define CD_COLL_EGO_MAX_ACCEL_X_UNAV (10.0f)
/*! @brief assumed lateral ego acceleration for collision unavoidable */
#define CD_COLL_EGO_MAX_ACCEL_Y_UNAV (8.0f)
/*! @brief assumed longitudinal ego acceleration for collision */
#define CD_COLL_EGO_MAX_ACCEL_X_LIKE (10.0f)
/*! @brief assumed lateral ego acceleration for collision */
#define CD_COLL_EGO_MAX_ACCEL_Y_LIKE (5.0f)
/*! @brief maximal prediction time for collision / collision unavoidable*/
#define CD_COLL_MAX_TIME (1.5f) /*s*/
/*! @brief minimum object assignments to track */
#define CD_COLL_MIN_TRACK_ASSIGNED (1u << 7u)

/*! @brief Minimum Collision Probability to activate Collision Bit */
#define CD_COLL_PROB_THRESHOLD_MEDIUM (0.7f)
/*! @brief Minimum Collision Probability to activate Collision Unavoidable Bit
 */
#define CD_COLL_PROB_THRESHOLD_MAXIMUM (0.7f)

/****************************************************************
  Run-up Stationary Hypothesis Settings
 *****************************************************************/
/*! @brief Enable Collision Hypothesis Evaluation */
#define CD_USE_RUN_UP_STAT_HYPOTHESIS SWITCH_ON
/*! @brief Min. EBA generic quality */
#define CD_RUN_UP_STAT_MIN_EBA_GEN_OBJ_QUAL (CD_COMMON_MIN_OBJ_QUALITY + 1u)
/*! @brief Min. EBA stationary quality */
#define CD_RUN_UP_STAT_MIN_EBA_STAT_OBJ_QUAL \
    (CD_RUN_UP_MIN_TRACK_STAT_OBJ_QUALITY + 1u)
/*! @brief Min. EBA pedestrian quality */
#define CD_RUN_UP_STAT_MIN_EBA_PED_OBJ_QUAL (0u)
/*! @brief Number of object filters for track assignment */
#define CD_RUN_UP_STAT_CLASS_FILTERS_NO (0u)
/*! @brief maximum run-up prediction time */
#define CD_RUN_UP_STAT_MAX_PRED_TIME (1.5f)
/*! @brief +x-direction search area for object matching */
#define CD_RUN_UP_STAT_GATE_AHEAD (5.0f)
/*! @brief -x-direction search area for object matching */
#define CD_RUN_UP_STAT_GATE_BEHIND (5.0f)
/*! @brief maximal TTS to detect run up stationary */
#define CD_RUN_UP_STAT_MAX_TTS \
    (8.0f)  // 4.0f) change for less deceleration situation by guotao 20191121
/*! @brief maximal TTC to detect run up stationary */
#define CD_RUN_UP_STAT_MAX_TTC (4.0f)
/*! @brief Threshold for pre-selection of objects for run-up stationary
 * hypothesis */
#define CD_RUN_UP_STAT_MIN_OBJ_CLASS_CONF (0u)

/****************************************************************
  Pedestrian Collision Hypothesis Settings
 *****************************************************************/
/*! @brief Enable Pedestrian Collision Hypothesis Evaluation Needs CD_USE_EMP*/
#define CD_USE_PED_COLL_HYPOTHESIS SWITCH_ON
/*! @brief Maximum Pedestrian Speed */
#define CD_PED_COLL_PED_SPEED_MAX (2.2f)
/*! @brief Min. EBA generic quality */
#define CD_PED_COLL_MIN_EBA_GEN_OBJ_QUAL \
    (0u) /*!< @todo: was CD_COMMON_MIN_OBJ_QUALITY */
/*! @brief Min. EBA stationary quality */
#define CD_PED_COLL_MIN_EBA_STAT_OBJ_QUAL (0u)
/*! @brief Min. EBA pedestrian quality */
#define CD_PED_COLL_MIN_EBA_PED_OBJ_QUAL (0u)
/*! @brief Number of object filters for track assignment */
#define CD_PED_COLL_CLASS_FILTERS_NO (0u)
/*! @brief Threshold for pre-selection of objects for pedestrian collision
 * hypothesis */
#define CD_PED_COLL_MIN_OBJ_CLASS_CONF (0u)
/* Use EMP Calculation for Probability Calculations */
#define CD_PED_COLL_USE_EMP_PROB_CALC (SWITCH_OFF)
/*
  Defines for Corridor Approach
*/
/* Use Corridor Approach */
#define CD_PED_COLL_USE_CORR_APPROACH (SWITCH_OFF)
/*! @brief Width of On Road Zone */
#define CD_PED_COLL_ON_ROAD_ZONE_WIDTH (3.3f)
/*! @brief Width of Pre-brake Zone */
#define CD_PED_COLL_PREBRAKE_ZONE_WIDTH (2.6f)
/*! @brief Time until Ped reaches Vehicle Path */
#define CD_PED_TIME_TO_VEHICLE_PATH_S (0.36f)
/*! @brief Additional escape zone offset */
#define CD_PED_COLL_ESCAPE_ZONE_OFFSET (0.3f)
/*
  Defines for Probability of Velocities Calculation in Pedestrian Collision
  Hypothesis
*/
/*! @brief Use Probability of Velocities Calculation */
#define CD_PED_COLL_USE_PSC (CD_PAR_DEPENDENT_SWITCH(CD_USE_EMP, SWITCH_ON))
/*! @brief Overwrite ObjClass when PED hyp is triggered */
#define CD_PED_COLL_OVERWRITE_OBJ_CLASS (SWITCH_ON)
/*! @brief Overwrite ObjClass when CYCLE hyp is triggered */
#define CD_CYC_COLL_OVERWRITE_OBJ_CLASS (SWITCH_ON)
/*! @brief Use EMP Structure to make own lateral velocity prediction */
#define CD_PED_COLL_PSC_EMP_VELO_PRED (SWITCH_ON)
/*! @brief Number of y distance values of every object to observe to fit
 * velocities */
#define CD_PED_COLL_OBSERVE_Y_DIST_N (5)
/*! @brief Min Velocity to use for fitting */
#define CD_PED_COLL_PED_VELO_MIN_MPS (-4.0f)
/*! @brief  Max Velocity to use for fitting */
#define CD_PED_COLL_PED_VELO_MAX_MPS (4.0f)
/*! @brief  Number of Steps between min and max velocity to use for fitting */
#define CD_PED_COLL_PED_VELO_STEPS_N (17u)
/*! @brief  Add debug Data to CDInternalObject_t which is meas freezed */
#define CD_PED_COLL_DEBUG_DATA_CD_INT_OBJ SWITCH_ON
/*! @brief  Add a gaussian kernel (build by measured object lateral velocity and
 * deviation) to velocity probability density */
#define CD_PED_COLL_USE_MEASURED_VELO_AS_GAUSSIAN SWITCH_ON
/*! @brief  Minimal Std for Velo to traj used in Ped Hypothesis */
#define CD_PED_COLL_PED_MIN_VELO_TO_TRAJ_STD (0.01f)

/*! @brief  Downgrade Ped Coll Prob for Ped/Bicycles with dominant velocity not
 * crossing */
#define CD_PED_COLL_USE_DOMINANT_VELOCITY_DOWNGRADE (SWITCH_ON)
/*! @brief Values of maximal velo_to/velo_on -rate dependend of the object dist
 * to vehicle path
 */
#define CD_NUMBER_OF_DOMINANTDOWNGRADE_MAX_RATE_VALUES (2L)
/*!  @cond Doxygen_Suppress */
extern MEMSEC_REF const BML_t_Vector2D
    CD_PED_COLL_DOMINANT_DOWNGRADE_MAX_RATE_VEC
        [CD_NUMBER_OF_DOMINANTDOWNGRADE_MAX_RATE_VALUES];
/*! @endcond */
/*! Values of maximal velo_to/velo_on -rate dependend of the object dist to
 * vehicle path
 */
#define CD_NUMBER_OF_DOMINANTDOWNGRADE_MIN_FACTOR_VALUES (2L)
/*!  @cond Doxygen_Suppress */
/*identifier differs clearly from other identifiers */
extern MEMSEC_REF const BML_t_Vector2D
    CD_PED_COLL_DOMINANT_DOWNGRADE_MIN_FACTOR_VEC
        [CD_NUMBER_OF_DOMINANTDOWNGRADE_MIN_FACTOR_VALUES];
/*! @endcond */

/*! @brief  Max velocity on/to trajectoryrate before dowmgrade */
#define CD_PED_COLL_DOMINANT_DOWNGRADE_MAX_RATE (0.6f)
/*! @brief  Min Downgrade Factor */
#define CD_PED_COLL_DOMINANT_DOWNGRADE_MIN_FACTOR (0.5f)
/*! @brief  Min object velocity to downgrade (use square of value) */
#define CD_PED_COLL_DOMINANT_DOWNGRADE_MIN_OBJ_VELO_MPS (1.0f)

/*! @brief Number of value pairs in ego additional width velocity dependend
 * table.
 */
#define CD_NUMBER_OF_EGO_ADD_WIDTH_VELO_DEP_VALUES (2L)
/*!  @cond Doxygen_Suppress */
extern MEMSEC_REF const BML_t_Vector2D
    CD_EGO_ADD_WIDTH_VELO_DEP_M[CD_NUMBER_OF_EGO_ADD_WIDTH_VELO_DEP_VALUES];
/*! @endcond */

/*********************************************************

  Bicycle Hypothesis
************************************************/
/*! @brief Enable Bicycle Collision Hypothesis */
#define CD_USE_BICYCLE_COLL_HYPOTHESIS (SWITCH_ON)

/*! @brief Min. EBA generic quality */
#define CD_BICYCLE_COLL_MIN_EBA_GEN_OBJ_QUAL \
    (0u) /*!< @todo: was CD_COMMON_MIN_OBJ_QUALITY */
/*! @brief Min. EBA stationary quality */
#define CD_BICYCLE_COLL_MIN_EBA_STAT_OBJ_QUAL (0u)
/*! @brief Min. EBA BICYCLE quality */
#define CD_BICYCLE_COLL_MIN_EBA_PED_OBJ_QUAL (0u)
/*! @brief Number of object filters for track assignment */
#define CD_BICYCLE_COLL_CLASS_FILTERS_NO (0u)
/*! @brief Threshold for pre-selection of objects for pedestrian collision
 * hypothesis */
#define CD_BICYCLE_COLL_MIN_OBJ_CLASS_CONF (0u)

/*! @brief Use the RoadEstimation to determine if a bicylce is on the Road or
 * not */
#define CD_BICYCLE_USE_EM_ROAD_ESTIMATION SWITCH_OFF

/*! @brief Use Probability of Velocities Calculation */
#define CD_BICYCLE_COLL_USE_PSC (CD_PAR_DEPENDENT_SWITCH(CD_USE_EMP, SWITCH_ON))
/*! @brief Use EMP Structure to make own lateral velocity prediction */
#define CD_BICYCLE_COLL_PSC_EMP_VELO_PRED (SWITCH_ON)
/*! @brief  Downgrade Ped Coll Prob for Bicycles with dominant velocity not
 * crossing */
#define CD_BICYCLE_COLL_USE_DOMINANT_VELOCITY_DOWNGRADE (SWITCH_ON)

/*! @brief  Max velocity on/to trajectoryrate before dowmgrade */
#define CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MAX_RATE (0.6f)
/*! @brief  Min Downgrade Factor */
#define CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MIN_FACTOR (0.5f)
/*! @brief  Min object velocity to downgrade (use square of value) */
#define CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MIN_OBJ_VELO_MPS (1.0f)

/*! @brief  Add debug Data to CDInternalObject_t which is meas freezed */
#define CD_BICYCLE_COLL_DEBUG_DATA_CD_INT_OBJ (SWITCH_OFF)
/* Use EMP Calculation for Probability Calculations */
#define CD_BICYCLE_COLL_USE_EMP_PROB_CALC (SWITCH_OFF)

/*! @brief Number of y distance values of every object to observe to fit
 * velocities */
#define CD_BICYCLE_COLL_OBSERVE_Y_DIST_N (5)
/*! @brief Min Velocity to use for fitting */
#define CD_BICYCLE_COLL_BICYCLE_VELO_MIN_MPS (-6.0f)
/*! @brief  Max Velocity to use for fitting */
#define CD_BICYCLE_COLL_BICYCLE_VELO_MAX_MPS (6.0f)
/*! @brief  Number of Steps between min and max velocity to use for fitting */
#define CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N (13u)
/*! @brief  Add a gaussian kernel (build by measured object lateral velocity and
 * deviation) to velocity probability density */
#define CD_BICYCLE_COLL_USE_MEASURED_VELO_AS_GAUSSIAN (SWITCH_ON)

/*! @brief  Minimal Std for Velo to traj used in Bicycle Hypothesis */
#define CD_BICYCLE_COLL_BICYCLE_MIN_VELO_TO_TRAJ_STD (0.01f)

// wulin add 20220316
/*! @brief  Downgrade Bicycles Coll Prob for Bicycles with dominant velocity not
 * crossing */
#define CD_BICYCLE_COLL_USE_DOMINANT_VELOCITY_DOWNGRADE (SWITCH_ON)
/*! @brief Values of maximal velo_to/velo_on -rate dependend of the object dist
 * to vehicle path
 */
// #define CD_NUMBER_OF_DOMINANTDOWNGRADE_MAX_RATE_VALUES (2L)
/*!  @cond Doxygen_Suppress */
extern MEMSEC_REF const BML_t_Vector2D
    CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MAX_RATE_VEC
        [CD_NUMBER_OF_DOMINANTDOWNGRADE_MAX_RATE_VALUES];
/*! @endcond */
/*! Values of maximal velo_to/velo_on -rate dependend of the object dist to
 * vehicle path
 */
#define CD_NUMBER_OF_DOMINANTDOWNGRADE_MIN_FACTOR_VALUES (2L)
/*!  @cond Doxygen_Suppress */
/*identifier differs clearly from other identifiers */
extern MEMSEC_REF const BML_t_Vector2D
    CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MIN_FACTOR_VEC
        [CD_NUMBER_OF_DOMINANTDOWNGRADE_MIN_FACTOR_VALUES];
/*! @endcond */

/*! @brief  Max velocity on/to trajectoryrate before dowmgrade */
#define CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MAX_RATE (0.6f)
/*! @brief  Min Downgrade Factor */
#define CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MIN_FACTOR (0.5f)
/*! @brief  Min object velocity to downgrade (use square of value) */
#define CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MIN_OBJ_VELO_MPS (1.0f)

/****************************************************************
  Pedestrian Pass Hypothesis Settings
 *****************************************************************/
/*! @brief Enable Pedestrian Collision Hypothesis Evaluation */
#define CD_USE_PED_PASS_HYPOTHESIS SWITCH_OFF
/*! @brief Min. EBA generic quality */
#define CD_PED_PASS_MIN_EBA_GEN_OBJ_QUAL \
    (0u) /*!< @todo: was CD_COMMON_MIN_OBJ_QUALITY*/
/*! @brief Min. EBA stationary quality */
#define CD_PED_PASS_MIN_EBA_STAT_OBJ_QUAL (0u)
/*! @brief Min. EBA pedestrian quality */
#define CD_PED_PASS_MIN_EBA_PED_OBJ_QUAL (0u)
/*! @brief Number of object filters for track assignment */
#define CD_PED_PASS_CLASS_FILTERS_NO (1u)
/*! @brief Threshold for pre-selection of objects for pedestrian pass hypothesis
 */
#define CD_PED_PASS_MIN_OBJ_CLASS_CONF (50u)

/****************************************************************
  Crossing Hypothesis Settings
 *****************************************************************/
/*! @brief Enable Crossing Hypothesis Evaluation */
#define CD_USE_CROSSING_HYPOTHESIS (SWITCH_OFF)
/*! @brief Threshold for Hypothesis to enter bitHypActive */
#define CD_CROSSING_ACTIVE_THRESHOLD (0.2f)
/*! @brief Min. EBA generic quality */
#define CD_CROSSING_MIN_EBA_GEN_OBJ_QUAL (0u)
/*! @brief Min. EBA stationary quality */
#define CD_CROSSING_MIN_EBA_STAT_OBJ_QUAL (0u)
/*! @brief Min. EBA pedestrian quality */
#define CD_CROSSING_MIN_EBA_PED_OBJ_QUAL (101u)
/*! @brief Min. EBA Cross braking minimum poe */
#define CD_CROSSING_MIN_POE_QUAL (90u)
/*! @brief Number of object filters for track assignment */
#define CD_CROSSING_CLASS_FILTERS_NO (1u)
/*! @brief Threshold for pre-selection of objects for crossing hypothesis */
#define CD_CROSSING_MIN_OBJ_CLASS_CONF (50u)
/*! @brief Consider objects only with a certain velocity */
#define CD_CROSSING_MIN_OBJ_VELOCITY (10.f / BML_f_Kmh2Ms)
/*! @brief Velocity threshold of Ego to activate function (Greater Zero!) */
#define CD_CROSSING_MAX_ACCELERATION_OBJECT (3.f)
/*! @brief Maximum value of deceleration of object to mitigate out of situation
 */
#define CD_CROSSING_MAX_DECELERATION_OBJECT (-9.f)
/*! @brief Minimum value of TTM for Interpolation between 0 and 1 (here for 0)
 */
#define CD_CROSSING_MIN_TTM_INTERP (1.0f)
/*! @brief Maximum value of TTM for Interpolation between 0 and 1 (here for 1)
 */
#define CD_CROSSING_MAX_TTM_INTERP (0.0f)
/*! @brief Min Angle for objects to be considered for hypCrossing */
#define CD_CROSSING_MIN_ANGLE (DEG2RAD(27.f))
/*! @brief Min xDistance for objects to be considered for hypCrossing */
#define CD_CROSSING_MIN_DISTX (12.f)
/*! @brief Consider bikes only with a certain velocity */
#define CD_CROSSING_MIN_BIKE_VELOCITY (15.f / BML_f_Kmh2Ms)
/*! @brief Stopped Object Track Assignment */
#define CD_CROSSING_STOPPED_OBJ_TRACK_ASSIGNED (7u << 5u)
/*! @brief Ego corridor width for environment filter */
#define CD_CROSSING_FILTER_EGO_CORRIDOR_WIDTH (1.5f)
/*! @brief Object EBA Quality for environment filter */
#define CD_CROSSING_FILTER_OBJECT_OBJ_QUAL (42u)
/*! @brief Probability Of Existence for environment filter */
#define CD_CROSSING_FILTER_OBJECT_POE_QUAL (90u)
/*! @brief Number of needed oncoming confirmations */
#define CD_CROSSING_NEEDED_ONCOMMING_CONFIRMATIONS (5u)
/*! @brief Was Crossing Counter Threshold */
#define CD_CROSSING_WASCROSSING_COUNTER_THRESHOLD (5u)
/*! @brief Crossing counter increase */
#define CD_CROSSING_COUNTER_INCREASE (2u)
/*! @brief Maximum int needed for counter variables */
#define CD_CROSSING_MAX_COUNTER_INT (200u)
/*! @brief TTC Threshold for crossing objects */
#define CD_CROSSING_OBJECT_TTC_THRSHLD (2.0f)
/*! @brief Angle for crossing objects */
#define CD_CROSSING_OBJECT_ANGLE_THRSHLD (DEG2RAD(48.0f))
/*! @brief DistY Threshold for crossing objects */
#define CD_CROSSING_OBJECT_DISTY_THRSHLD (15.0f)
/*! @brief Hyp Cat Velocity Threshold for crossing objects */
#define CD_CROSSING_HYP_CAT_VELOCITY_THRESHOLD (7.0f)
/*! @brief Cam Confirmation Threshold for crossing objects */
#define CD_CROSSING_CAM_CONFIRMATION_PROB_THRSHLD (97u)
/*! @brief ego velocity camera confirmation acceleration Threshold for crossing
 * objects */
#define CD_CROSSING_EGO_VEL_CAM_ACCEL_THRSHLD (52.f / BML_f_Kmh2Ms)
/*! @brief Cam Confirmation Acceleration Threshold for crossing objects */
#define CD_CROSSING_CAM_ACCEL_THRSHLD (1.0f)

/****************************************************************
  Common Hypothesis Handler and Object Filter Settings
 *****************************************************************/

/*! @brief Total number of hypothesis handlers */
#define CD_HYP_HANDLERS_NO                                     \
    (CD_USE_RUN_UP_HYPOTHESIS + CD_USE_CUT_IN_HYPOTHESIS +     \
     CD_USE_PASS_HYPOTHESIS + CD_USE_FOLLOW_HYPOTHESIS +       \
     CD_USE_COLL_HYPOTHESIS + CD_USE_RUN_UP_STAT_HYPOTHESIS +  \
     CD_USE_AVLC_HYPOTHESIS + CD_USE_PED_COLL_HYPOTHESIS +     \
     CD_USE_PED_PASS_HYPOTHESIS + CD_USE_CROSSING_HYPOTHESIS + \
     CD_USE_BICYCLE_COLL_HYPOTHESIS)

/*! @brief Total number of hypothesis handlers for common to all hyp. types (if
 * not specialized otherwise) */
#define CD_COMMON_CLASS_FILTERS_NO (8u)

/****************************************************************
  Track Assignment Settings
 *****************************************************************/

/*! @brief minimum track overlap for track assignment */
#define CD_TRKASGN_MIN_TRACK_OVERLAP (0.75f)
/*! @brief minimum object overlap for track assignment */
#define CD_TRKASGN_MIN_OBJECT_OVERLAP (0.5f)
/*! @brief offset for predicted track overlap, i.e. allowed error */
#define CD_TRKASGN_TRACK_OVERLAP_PRED_OFFSET (-0.25f)
/*! @brief offset for predicted object overlap, i.e. allowed error */
#define CD_TRKASGN_OBJ_OVERLAP_PRED_OFFSET (-0.35f)
/*! @brief minimum lateral velocity for avoidance maneuvers */
#define CD_TRKASGN_MIN_LAT_AVOID_SPEED (0.9f)
/*! @brief maximum object distance for avoidance maneuver detection */
#define CD_TRKASGN_MAX_AVOID_DIST (20.0f)
/*! @brief maximum TTC for avoidance maneuver detection */
#define CD_TRKASGN_MAX_AVOID_TTC (3.0f)
/*! @brief minimum curve radius for rad estimation based track assignment
 * in case of smaller radii the track assign algorithms uses the
 * ego trajectory for stationary objects
 */
#define CD_TRKASGN_MIN_RAD_FOR_ROAD_TRAJ (700.0f)
/*! @brief minimum run-up prediction time for track assignment */
#define CD_TRKASGN_CURVE_OVRLAP_PRED_TIME (1.5f)
/*! @brief minimum run-up prediction time for track assignment of
 * stationary objects when driving straight ahead */
#define CD_TRKASGN_STRAIGHT_OVRLAP_PRED_TIME \
    (CD_TRKASGN_CURVE_OVRLAP_PRED_TIME - 0.35f)

/*! @brief CD_TRKASGN_PROB_THRESHOLD */
#define CD_TRKASGN_PROB_THRESHOLD (0.2f)

/*! @brief Use DistToTraj to calculate Track Assignment */
#define CD_TRKASGN_BY_DIST_TO_TRAJ (SWITCH_ON)
/*! @brief Relative Reduction of Corridor to calculate Track Assignment */
#define CD_TRKASGN_CORRIDOR_REDUCTION_PERC (0.3f)
/*! @brief CD_LIMIT_RADAR_OBJECT_WIDTH */
#define CD_LIMIT_RADAR_OBJECT_WIDTH (SWITCH_ON)
/*! @brief CD_MAX_OBJ_WIDTH_STAT */
#define CD_MAX_OBJ_WIDTH_STAT (2.0f)
/****************************************************************
 Lane Association Settings
 *****************************************************************/
/*! @brief Enable if objects shall be associated to lanes
 *
 * @attention This feature is required by some algorithms in CD.
 * To save calculation time and memory check if activation is needed.
 */
#define CD_USE_LANE_ASSOCIATION (CD_CUT_IN_CHECK_NBOR_OBJ_RUN_UP)
/*! @brief Width of the lane center area. If object is in this area it gets
 *  maximum association probability
 */
#define CD_LANE_ASSOC_LANE_CENTER_WIDTH (1.25f)
/*! @brief Normal width of the lane. This is also the distance between adjacent
 * lanes
 * into the same direction. Default value is the minium lane width in Germany
 * (2.75m).
 */
#define CD_LANE_ASSOC_NORMAL_LANE_WIDTH (2.75f)
/*! @brief Extended width of the lane in near range.
 * Default value is the maxium defined lane width in Germany (3.75m).
 */
#define CD_LANE_ASSOC_EXT_LANE_WIDTH_NEAR (3.75f)
/*! @brief Extended width of the lane in far distance range.
 *
 * Lane width is interpolated between naer and far distance.
 */
#define CD_LANE_ASSOC_EXT_LANE_WIDTH_FAR (4.50f)
/*! @brief Distance for narrow extended lane width.
 */
#define CD_LANE_ASSOC_EXT_LANE_DIST_NEAR (40.0f)
/*! @brief Distance for extended width of the lane.
 */
#define CD_LANE_ASSOC_EXT_LANE_DIST_FAR (50.0f)

/****************************************************************
  Object Merge Settings (cd_wrapper.c)
 *****************************************************************/

/*! @brief Max. delete/keep lifetime percentage */
#define CD_OBJ_MERGE_CRIT_TIME_THRES_PERC (0.3f)
/*! @brief Min. liftetime of objects to consider merge to be critical @unit:
 * seconds */
#define CD_OBJ_MERGE_CRIT_MERGE_MIN_OBJ_LFT (1.33f)
/*! @brief Lateral distance to define critical obj. merge time lower boundary */
#define CD_OBJ_MERGE_CRIT_TIME_THRES_DISTY_0 (1.0f)
/*! @brief Critical merge time lower boundary */
#define CD_OBJ_MERGE_CRIT_TIME_THRES_TIME_0 (0.0f)
/*! @brief Lateral distance to define critical obj. merge time upper boundary */
#define CD_OBJ_MERGE_CRIT_TIME_THRES_DISTY_1 (3.0f)
/*! @brief Critical merge time upper boundary */
#define CD_OBJ_MERGE_CRIT_TIME_THRES_TIME_1 (30.0f)

/****************************************************************
  Performance Degradation Settings (cd_wrapper.c, cd_customfunctions.c)
 *****************************************************************/
/*! @brief Use performance degradation interface */
#define CD_USE_PERF_DEGRADATION SWITCH_ON

/*! @brief Use performance degradation results from VLC/SPM */
#define CD_USE_PERF_DEGR_SPM SWITCH_ON

#define CD_PERF_DEG_MAX_LAT_ERR_DIST_SFTY \
    (1.5f) /*!< max lateral error (m) for safety functions at max. distance */
#define CD_PERF_DEG_MAX_LAT_ERR_DIST_PERF                               \
    (2.5f) /*!< max lateral error (m) for performance functions at max. \
              distance */
#define CD_PERF_DEG_MAX_DIST \
    CD_DIST_MAX /*!< distance in case of no lateral errors */

#define CD_PERF_DEG_HRZ_DIST_PT1_CONST \
    (0.2f) /*!< PT1 filter constant for moving horizon distance filtering */
#define CD_PERF_DEG_HRZ_DIST_FUSE_VEL_LOW \
    (5.0f) /*!< Low velocity between for moving horizon fusion */
#define CD_PERF_DEG_HRZ_DIST_FUSE_VEL_HIGH \
    (8.0f) /*!< High velocity between for moving horizon fusion */

#define CD_PERF_DEG_MAX_VDY_SIG_ERRORS \
    (5uL) /*!< CD_PERF_DEG_MAX_VDY_SIG_ERRORS */

/****************************************************************
  EPF - Evasion Possibility Front Settings
 *****************************************************************/
/*! @brief Use Evasion Possibility Front */
#define CD_USE_EPF \
    (SWITCH_OFF)  // CD_PAR_DEPENDENT_SWITCH(VLC_CFG_SEN_CAM_LANE_INTERFACE,
                  // SWITCH_ON)

#define CD_EPF_USE_EMP \
    CD_PAR_DEPENDENT_SWITCH(CD_USE_EMP_TRAJ_RELATION, SWITCH_ON)

#define CD_EPF_MAX_LAT_CORR_WIDTH                                       \
    (4.0f) /*!< Maximum Distance (m) for evasion Corridor in case of no \
              Obstacle */
#define CD_EPF_TARGET_SAFETY_DIST              \
    (0.5f) /*!< Safety Distance (m) for Target \
            */
#define CD_EPF_EGO_CORRIDOR_SAFETY_EXTENSION \
    (1.8f) /*!< Lateral safety Extension (m) of Ego evasion Corridor */
#define CD_EPF_OBJ_SAFETY_DIST \
    (0.5f) /*!< Safety Distance (m) for Target and normal Traffic Objects */
#define CD_EPF_VULNERABLE_OBJ_SAFETY_DIST                                     \
    (1.0f) /*!< Safety Distance (m) for weak Traffic Objects like Pedestrians \
            */
#define CD_EPF_OBSTACLE_MIN_POE                                              \
    (50u) /*!< Minimum Obstacle Probability of Existence to be considered by \
             EPF */
#define CD_EPF_POINT_OBSTACLE_MIN_POE                                        \
    (90u) /*!< Minimum Obstacle Probability of Existence of Point Objects to \
             be considered by EPF */
#define CD_EPF_MAX_LAT_PRED_OFFSET                                      \
    (4.0f) /*!< Maximum Distance (m) for evasion Corridor in case of no \
              Obstacle */
#define CD_EPF_MAX_LONG_DIST_FACTOR                                          \
    (3.0f) /*!< Longitudinal Distance Factor for total length of Area behind \
              Target */
#define CD_EPF_TOLERATED_LAT_DIST_BEHIND_TARGET                            \
    (0.6f) /*!< Maximum Distance (m) for Objects in front of the Target to \
              penetrate the Inside of the evasion Corridor */
#define CD_EPF_CENTER_ZONE_OVERLAP                                     \
    (0.2f) /*!< Overlap Distance in front of ego vehicle of the center \
              safety zones */
#define CD_EPF_OBJ_WIDTH_LIMIT                                         \
    (2.0f) /*!< Overlap Distance in front of ego vehicle of the center \
              safety zones */
#define CD_EPF_OBJECT_ANECLONG_THRES                                     \
    (-1.0f) /*!< ANecLong at which the probability of a free corridor is \
               multiplied by 0 */
#define CD_EPF_MAX_TTC                                                       \
    (4.0f) /*!< Maximum TTC (TimeToCollision) the relevant Object is allowed \
              to have to calculate the evasion Corridors */
#define CD_EPF_MIN_V_EGO                                                 \
    (4.0f) /*!< Minimum Ego Velocity to calculate evasion possibility in \
              meter per second */
#define CD_EPF_DIST_TO_TRAJ_YAW_RATE                                          \
    (0.02f) /*!< Maximum absolute Yaw Rate at which DistToTraj Calculation is \
               favoured */

#define CD_EPF_CORRIDOR_FREEZE (SWITCH_ON)
#define CD_EPF_CORRIDOR_FREEZE_TTB_THRES \
    (0.0f) /*!< TTB Threshold at which corridors are frozen */
#define CD_EPF_CORRIDOR_FREEZE_TTS_THRES \
    (0.0f) /*!< TTS Threshold at which corridors are frozen */

/****************************************************************
  Measurement Settings
 *****************************************************************/

/*! @brief Enable/disable freeze the internal CD state */

/*! @brief VLC_MEAS_ID_CGEB_CD_HYPOTHESES */
#define VLC_MEAS_ID_CGEB_CD_HYPOTHESES (VLC_MEAS_ID_CGEB_CD_DATA + 0x0u)

/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_STATE */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_STATE (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x0u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_INT_KIN_OBJ */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_INT_KIN_OBJ \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x100u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_INT_ATTR_OBJ*/
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_INT_ATTR_OBJ \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x200u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_INT_DIM_OBJ */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_INT_DIM_OBJ \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x300u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_QUAL_OBJ */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_QUAL_OBJ \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x400u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_ADD_QUAL_OBJ */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_ADD_QUAL_OBJ \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x500u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_OBJ_EGO_TRAJ */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_OBJ_EGO_TRAJ \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x600u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_OBJ_ROAD_TRAJ */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_OBJ_ROAD_TRAJ \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x900u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_OBJ_INT */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_OBJ_INT \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0xE00u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_DYN_RAW */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_DYN_RAW \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0xF00u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_DYN_SYNC */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_DYN_SYNC \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1000u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_TRAJ_RAW */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_TRAJ_RAW \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1100u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_ROAD_TRAJ_RAW */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_ROAD_TRAJ_RAW \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1200u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_ROAD_TRAJ_RAW */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_TRAJ_SYNC \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1300u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_DYN_INT */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_DYN_INT \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1400u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_INP_INT */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_INP_INT \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1500u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_ADJ_SAFE_INT */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_ADJ_SAFE_INT \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1600u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_PAR_INT */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_PAR_INT \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1700u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_EXT_FUN */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_EXT_FUN \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1800u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_MATH_FUN */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_MATH_FUN \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1900u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_REL_HYP_INT */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_REL_HYP_INT \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1A00u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_CRV_DATA */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_CRV_DATA \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1B00u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_CRV_PAR */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_CRV_PAR \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1B50u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_CRV_RAW */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_CRV_RAW \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1B90u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_CRV_FILT */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_CRV_FILT \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1BD0u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_OBJ_LANE */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_OBJ_LANE \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x1D00u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_COLL_MODEL_INT */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_COLL_MODEL_INT \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x7A00u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_INT_STATE */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_INT_STATE \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x7B00u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_OUT_DATA */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_OUT_DATA \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x7C00u)
/*! @brief VLC_MEAS_ID_CGEB_CD_WRAP_CD_REL_OBJ_OUT */
#define VLC_MEAS_ID_CGEB_CD_WRAP_CD_REL_OBJ_OUT \
    (VLC_MEAS_ID_CGEB_CD_WRAP_DATA + 0x7C10u)

/****************************************************************
  EMP Configuration
 *****************************************************************/
/*! @brief CD_USE_EMP_OVERLAP */
#define CD_USE_EMP (SWITCH_ON)
/*! @brief CD_USE_EMP_OVERLAP */
#define CD_USE_EMP_OVERLAP (SWITCH_OFF)
/*! @brief CD_USE_EMP_TRAJ_RELATION */
#define CD_USE_EMP_TRAJ_RELATION CD_PAR_DEPENDENT_SWITCH(CD_USE_EMP, SWITCH_ON)
/*! @brief CD_USE_EMP_TRACK_ASSIGN */
#define CD_USE_EMP_TRACK_ASSIGN \
    (SWITCH_OFF) /* Needs CD_USE_EMP_TRAJ_RELATION */
/*! @brief CD_USE_EMP_HYPCOLLISION */
#define CD_USE_EMP_HYPCOLLISION (SWITCH_OFF)

/****************************************************************
  Switch between CDAssignTrackProbability() in cd_hypacc.c and
 cd_customfunctions.c
 *****************************************************************/
/*! @brief CD_USE_CUSTOM_TRACK_ASSIGNMENT */
#define CD_USE_CUSTOM_TRACK_ASSIGNMENT (SWITCH_OFF)

/****************************************************************
  Switch between standard and custom hypothesis call
 *****************************************************************/

/*! @brief CD_USE_CUSTOM_HYPOTHESIS_RUNUP */
#define CD_USE_CUSTOM_HYPOTHESIS_RUNUP (SWITCH_OFF)
/*! @brief CD_USE_CUSTOM_HYPOTHESIS_RUNUPSTATIONARY */
#define CD_USE_CUSTOM_HYPOTHESIS_RUNUPSTATIONARY (SWITCH_OFF)
/*! @brief CD_USE_CUSTOM_HYPOTHESIS_PEDESTRIAN_COLLISION */
#define CD_USE_CUSTOM_HYPOTHESIS_PEDESTRIAN_COLLISION (SWITCH_OFF)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

#endif /*_CD_PAR_H_INCLUDED */
