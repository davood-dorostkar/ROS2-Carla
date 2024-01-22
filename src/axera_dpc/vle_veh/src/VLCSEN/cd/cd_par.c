/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "cd.h"

/*****************************************************************************
  GLOBAL VARIABLES
*****************************************************************************/
/*!  @cond Doxygen_Suppress */
/*! Table to downgrade the pedestrian hypothesis probability [0,1]*/
SET_MEMSEC_CONST(CD_PED_COLL_DOMINANT_DOWNGRADE_MAX_RATE_VEC)
const BML_t_Vector2D CD_PED_COLL_DOMINANT_DOWNGRADE_MAX_RATE_VEC
    [CD_NUMBER_OF_DOMINANTDOWNGRADE_MAX_RATE_VALUES] = {
        {0, C_F32_DELTA},
        {0.2f, 1.0f}}; /* { 0.2f, 1.0f } -> ped is >0.2m away, we downgrad the
                          ped hypothesis for objects approaching in an angle
                          sharper than 45 degree.  */

SET_MEMSEC_CONST(CD_PED_COLL_DOMINANT_DOWNGRADE_MIN_FACTOR_VEC)
const BML_t_Vector2D CD_PED_COLL_DOMINANT_DOWNGRADE_MIN_FACTOR_VEC
    [CD_NUMBER_OF_DOMINANTDOWNGRADE_MIN_FACTOR_VALUES] = {{0, 1.0f},
                                                          {1.0f, 0.0f}};

// wulin add 20220316
/*! Table to downgrade the bicycle hypothesis probability [0,1]*/
SET_MEMSEC_CONST(CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MAX_RATE_VEC)
const BML_t_Vector2D CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MAX_RATE_VEC
    [CD_NUMBER_OF_DOMINANTDOWNGRADE_MAX_RATE_VALUES] = {
        {0, C_F32_DELTA},
        {0.2f, 1.0f}}; /* { 0.2f, 1.0f } -> bicycle is >0.2m away, we downgrad
                          the bicycle hypothesis for objects approaching in an
                          angle sharper than 45 degree.  */

SET_MEMSEC_CONST(CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MIN_FACTOR_VEC)
const BML_t_Vector2D CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MIN_FACTOR_VEC
    [CD_NUMBER_OF_DOMINANTDOWNGRADE_MIN_FACTOR_VALUES] = {{0, 1.0f},
                                                          {1.0f, 0.0f}};

SET_MEMSEC_CONST(CD_EGO_ADD_WIDTH_VELO_DEP_M)
const BML_t_Vector2D
    CD_EGO_ADD_WIDTH_VELO_DEP_M[CD_NUMBER_OF_EGO_ADD_WIDTH_VELO_DEP_VALUES] = {
        {8.0f, 0}, {20.0f, 1.5f}};
#ifdef PROJECT_ZT_SPECIAL_VERSION
/*! Table that maps the raw ego velocity to assumed max. x-acceleration for
 * comfort braking */
SET_MEMSEC_CONST(CD_COMFORT_EGO_ACCEL_X)
const BML_t_Vector2D
    CD_COMFORT_EGO_ACCEL_X[CD_NUMBER_OF_COMFORT_EGO_ACCEL_X_VALUES] = {
        {10.0f, -1.0f}, {15.0f, -1.0f}};
#else
/*! Table that maps the raw ego velocity to assumed max. x-acceleration for
 * comfort braking */
SET_MEMSEC_CONST(CD_COMFORT_EGO_ACCEL_X)
const BML_t_Vector2D
    CD_COMFORT_EGO_ACCEL_X[CD_NUMBER_OF_COMFORT_EGO_ACCEL_X_VALUES] = {
#ifdef SITRAK
        {10.0f, -2.0f}, {15.0f, -2.0f}
#else
        {10.0f, -5.0f}, {15.0f, -5.0f}
#endif
};
#endif
/*! Table that maps the raw ego velocity to assumed max. y-acceleration for
 * comfort steering */
SET_MEMSEC_CONST(CD_COMFORT_EGO_VEL_FACTOR_Y)
const BML_t_Vector2D
    CD_COMFORT_EGO_VEL_FACTOR_Y[CD_NUMBER_OF_COMFORT_EGO_VEL_FACTOR_Y_VALUES] =
        {{0, 1.0f},     {10.0f, 0.9f}, {15.0f, 0.75f}, {18.0f, 0.6f},
         {22.0f, 0.5f}, {40.0f, 0.5f}, {50.0f, 0.6f}};

/*! Table that maps the raw ego velocity to assumed max. y-acceleration for
 * emergency steering */
SET_MEMSEC_CONST(CD_EMERGENCY_EGO_ACCEL_Y)
const BML_t_Vector2D
    CD_EMERGENCY_EGO_ACCEL_Y[CD_NUMBER_OF_EMERGENCY_EGO_ACCEL_Y_VALUES] = {
        {22.0f, 7.0f}, {30.0f, 4.0f}};

/*! @brief      Longitudinal safety margin for customer calculated anec long
   value
    @general
    @conseq     @incp  anec higher at given long. distance
                @decp  anec lower at given long. distance
    @attention  Activation thresholds on end function are not affected
    @typical    0.5    @unit  m   @min 0.0   @max 10.0   */
SET_MEMSEC_VAR(CD_LONG_SAFETY_DIST_CUST)
CD_DEF_PARAM(float32,
             CD_LONG_SAFETY_DIST_CUST,
             CD_LONG_SAFETY_DIST_CUST_DEFAULT)

/*! @brief      Simulate additional longitudinal offset, used to fake shorter
   distance
                for the hypothesis calculation
    @general    -
    @conseq     @incp  (+) increase long. object distance
                @decp  (-) decrease long. object distance
    @attention  Activation thresholds on end function are not affected
    @typical    0.0    @unit  m   @min -50.0   @max 50.0   */
SET_MEMSEC_VAR(CD_LONG_OFFSET_SIMU)
CD_DEF_PARAM(float32, CD_LONG_OFFSET_SIMU, CD_LONG_OFFSET_SIMU_DEFAULT)

/*! @brief      Simulate additional lateral offset, used to fake shorter
   distance
                for the hypothesis calculation
    @general    -
    @conseq     @incp  (+) shift object to left direction
                @decp  (-) shift object to right direction
    @attention  Consider field of view limitation, thus, parameter is only
   effective
                in conjunction width longitudinal offset and track width factor
    @typical    0.0    @unit  m   @min -5.0   @max 5.0   */
SET_MEMSEC_VAR(CD_LAT_OFFSET_SIMU)
CD_DEF_PARAM(float32, CD_LAT_OFFSET_SIMU, CD_LAT_OFFSET_SIMU_DEFAULT)
/*! @brief      Modify driving path width to modify target object to lane
   allocation
    @general    -
    @conseq     @incp  (>1.0) widen assumed vehicle driving path width
                @decp  (<1.0) narrow assumed vehicle driving path width
    @attention  -
    @typical    1.0    @unit  -   @min 0.5   @max 5.0   */

SET_MEMSEC_VAR(CD_COMMON_TRACK_WIDTH_FACT)
CD_DEF_PARAM(float32,
             CD_COMMON_TRACK_WIDTH_FACT,
             CD_COMMON_TRACK_WIDTH_FACT_DEFAULT)
/*! @brief Ego trajectory curvature depending track width (x --> curvature, y
 * --> track width) */
SET_MEMSEC_CONST(CD_RUN_UP_TTC_TRACK_WIDTH)
const BML_t_Vector2D CD_RUN_UP_TTC_TRACK_WIDTH[CD_RUN_UP_TTC_WIDTH_NO_POINTS] =
    {{0.0015f, 2.0f}, {0.0035f, 1.7f}};
/*! @brief Obj distance depending track width (x --> distance, y --> track
 * width) */
SET_MEMSEC_CONST(CD_RUN_UP_TTC_TRACK_WIDTH_WIDE)
const BML_t_Vector2D
    CD_RUN_UP_TTC_TRACK_WIDTH_WIDE[CD_RUN_UP_TTC_WIDTH_FAC_NO_POINTS] = {
        {3.0f, 1.0f}, {5.0f, 2.0f}};
/*! @brief      Run-up braking hypothesis probability reduction
   @general    Table with reduction factor over VrelX change
   @attention  Only reduction down to 0.5 are effective         */
/* Input:  Magnitude of increased rel. long. velocity since start of run-up
 * moving */
/* Output: Probability reduction factor, only reduction down to 0.5 are
 * effective */
SET_MEMSEC_CONST(CD_RUNUP_BRK_PROB_VREL_RED)
CD_PAR_CONST BML_t_Vector2D
    CD_RUNUP_BRK_PROB_VREL_RED[CD_RUNUP_BRK_HYP_PROB_RED_OVER_VREL_NO] = {
        {0, 0.5f}, {4.0f, 0.6f}, {8.0f, 1.0f}};

/* Input: Sensor vehicle ego velocity */
/* Output: Min. time gap (based on Vego) to freeze VrelX */
CD_PAR_CONST BML_t_Vector2D
    CD_RUNUP_BRK_VEGO_TGAP_UPPER[CD_RUNUP_BRK_VEGO_TGAP_NO] = {
        {0, 2.0f}, {80.0f / 3.6f, 2.0f}, {130.0f / 3.6f, 2.0f}};

/*! @brief      Minimum required time gap to activate hypothesis reduction
   @general    Table with time gap over ego velocity
   @attention  -                                                           */
/* Input:  Sensor vehicle ego velocity */
/* Output: Minimum required time gap to activate probability reduction */
SET_MEMSEC_CONST(CD_RUNUP_BRK_VEGO_TGAP)
CD_PAR_CONST BML_t_Vector2D CD_RUNUP_BRK_VEGO_TGAP[CD_RUNUP_BRK_VEGO_TGAP_NO] =
    {{0, 1.5f}, {160.0f / 3.6f, 1.5f}, {200.0f / 3.6f, 1.5f}};

/*! Time gap (vrel based) to activate probability reduction */
SET_MEMSEC_VAR(CD_RUNUP_BRK_TIME_GAP_DEFAULT)
CD_DEF_PARAM(float32, CD_RUNUP_BRK_TIME_GAP, CD_RUNUP_BRK_TIME_GAP_DEFAULT)

/*! @brief      Minimum required distance for narrowing trajectory width for
   small objects
   @general    Table with distance threshold over ego velocity
   @attention  - */
/* Input:  Sensor vehicle ego velocity  */
/* Output: Distance Threshold for narrowing trajectory width */
SET_MEMSEC_CONST(CD_RUNUP_SOBJ_OVLC_VEGO_DIST)
CD_PAR_CONST BML_t_Vector2D
    CD_RUNUP_SOBJ_OVLC_VEGO_DIST[CD_RUNUP_SOBJ_HYP_OVLC_VEGO_DIST_NO] = {
        {60.0f / 3.6f, 25.0f}, {100.0f / 3.6f, 50.0f}, {200.0f / 3.6f, 100.5f}};

/*! @brief      Transition section for trajectory width reduction
   @general    Defines relative part of min. distance, starting width narrowing
   @attention  - */
SET_MEMSEC_CONST(CD_RUNUP_SOBJ_HYP_OVLC_DIST_RED_FACT)
VLC_DEF_FIX_PARAM(float32, CD_RUNUP_SOBJ_HYP_OVLC_DIST_RED_FACT, 0.8F)

/*! @brief      Trajectory width narrowing
   @general    Reduction factor for trajectory width, dependent on transition
   section
   @attention  -
                                                                */
SET_MEMSEC_VAR(CD_RUNUP_SOBJ_HYP_OVLC_DIST_RED)
BML_t_Vector2D
    CD_RUNUP_SOBJ_HYP_OVLC_DIST_RED[CD_RUNUP_SOBJ_HYP_OVLC_RED_OVER_DIST_NO] = {
        {0, 1.0f}, /* Those zeros will be set dynamically in hyprunup.c */
        {0, 0.4f}};

/* ************************************************************************* */
/*   Hypothesis Handlers                                                     */
/* ************************************************************************* */

/*! Handler for 'Run-up' Hypothesis */
SET_MEMSEC_CONST(CD_RUN_UP_HANDLER)
static const CDHypoHandler_t CD_RUN_UP_HANDLER = {
    &CDHypoRunUpMain,
    NULL,
    (uint16)((1u << (uint32)CDHypothesisType_RunUp) |
             (1u << (uint32)CDHypothesisType_RunUpBraking)),
    (Envm_t_GenEbaHypCat)FPS_EBA_HYP_CAT_VCL,
    CD_RUN_UP_MIN_EBA_GEN_OBJ_QUAL,
    CD_RUN_UP_MIN_EBA_STAT_OBJ_QUAL,
    CD_RUN_UP_MIN_EBA_PED_OBJ_QUAL,
    CD_RUN_UP_MIN_OBJ_CLASS_CONF,
    FALSE,
    CD_RUN_UP_CLASS_FILTERS_NO,
    VLCSEN_RTA_CD_HYP_RUNUP,
};

/*! Handler 'Follow' Hypothesis */
SET_MEMSEC_CONST(CD_FOLLOW_HANDLER)
static const CDHypoHandler_t CD_FOLLOW_HANDLER = {
    &CDHypoStaticMain,
    NULL,
    (uint16)(1u << (uint32)CDHypothesisType_Static),
    (Envm_t_GenEbaHypCat)FPS_EBA_HYP_CAT_VCL,
    CD_FOLLOW_MIN_EBA_GEN_OBJ_QUAL,
    CD_FOLLOW_MIN_EBA_STAT_OBJ_QUAL,
    CD_FOLLOW_MIN_EBA_PED_OBJ_QUAL,
    CD_FOLLOW_MIN_OBJ_CLASS_CONF,
    TRUE,
    CD_FOLLOW_CLASS_FILTERS_NO,
    VLCSEN_RTA_CD_HYP_STATIC};

/*! Handler 'Run-up Stationary' Hypothesis */
SET_MEMSEC_CONST(CD_RUN_UP_STAT_HANDLER)
static const CDHypoHandler_t CD_RUN_UP_STAT_HANDLER = {
    &CDHypoRunUpStationaryMain,
    NULL,
    (uint16)(1u << (uint32)CDHypothesisType_RunUpStationary),
    (Envm_t_GenEbaHypCat)FPS_EBA_HYP_CAT_STAT,
    CD_RUN_UP_STAT_MIN_EBA_GEN_OBJ_QUAL,
    CD_RUN_UP_STAT_MIN_EBA_STAT_OBJ_QUAL,
    CD_RUN_UP_STAT_MIN_EBA_PED_OBJ_QUAL,
    CD_RUN_UP_STAT_MIN_OBJ_CLASS_CONF,
    FALSE,
    CD_RUN_UP_STAT_CLASS_FILTERS_NO,
    VLCSEN_RTA_CD_HYP_RUNUP_STAT};

SET_MEMSEC_CONST(CD_PED_COLL_HANDLER)
static const CDHypoHandler_t CD_PED_COLL_HANDLER = {
    &CDHypoPedCollMain,
    NULL,
    (uint16)(1u << (uint32)CDHypothesisType_PedCollision),
    (Envm_t_GenEbaHypCat)FPS_EBA_HYP_CAT_PED,
    CD_PED_COLL_MIN_EBA_GEN_OBJ_QUAL,
    CD_PED_COLL_MIN_EBA_STAT_OBJ_QUAL,
    CD_PED_COLL_MIN_EBA_PED_OBJ_QUAL,
    CD_PED_COLL_MIN_OBJ_CLASS_CONF,
    FALSE,
    CD_PED_COLL_CLASS_FILTERS_NO,
    VLCSEN_RTA_CD_HYP_PEDCOLL};

// wulin to do 20220307, add CD_BICYCLE_COLL_NANDLER
SET_MEMSEC_CONST(CD_BICYCLE_COLL_NANDLER)
static const CDHypoHandler_t CD_BICYCLE_COLL_NANDLER = {
    &CDHypoBicycleCollMain,
    NULL,
    (uint16)(1u << (uint32)CDHypothesisType_CyclColl),
    (Envm_t_GenEbaHypCat)FPS_EBA_HYP_CAT_CYCL,
    CD_BICYCLE_COLL_MIN_EBA_GEN_OBJ_QUAL,
    CD_BICYCLE_COLL_MIN_EBA_STAT_OBJ_QUAL,
    CD_BICYCLE_COLL_MIN_EBA_PED_OBJ_QUAL,
    CD_BICYCLE_COLL_MIN_OBJ_CLASS_CONF,
    FALSE,
    CD_BICYCLE_COLL_CLASS_FILTERS_NO,
    VLCSEN_RTA_CD_HYP_BICYCLECOLL};

SET_MEMSEC_CONST(CD_HYP_HANDLERS)
const CDHypoHandler_t* const CD_HYP_HANDLERS[CD_HYP_HANDLERS_NO] = {
    &CD_RUN_UP_HANDLER,   &CD_FOLLOW_HANDLER,       &CD_RUN_UP_STAT_HANDLER,
    &CD_PED_COLL_HANDLER, &CD_BICYCLE_COLL_NANDLER,
};
/*! @endcond */

/* ************************************************************************* */
/*   Copyright Tuerme                     */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */