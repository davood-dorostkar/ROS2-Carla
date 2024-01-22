#ifndef OPS_EBA_MOSA_H_INCLUDED
#define OPS_EBA_MOSA_H_INCLUDED

/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/
#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "TM_Global_Types.h"
#include "stddef.h"
#include "assert.h"
#include "ops.h"
#include "ops_par.h"
#include "envm_common_utils.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  DEFINE
*****************************************************************************/

/************************************************************************/
/* Moving safe defines                                                  */
/* No moving safe conditions for the current object are met. No functional
 * reaction is allowed */
#define OPSEBA_MOSA_PERMISSION_NONE (0u)
/* moving safe  conditions for the current object are met. All known issues are
 * fixed */
#define OPSEBA_MOSA_PERMISSION_LVL1 (1u)
/* moving safe  conditions for the current object are met. All known issues are
 * fixed and the thresholds are increased to suppress also possible new events
 */
#define OPSEBA_MOSA_PERMISSION_LVL2 (2u)

/*! Min object age to set moving safe permission */
#define OPSEBA_MOSA_MIN_LIFE_CYCLES (20u)
/*! Max time gap where objects are get moving safe permission */
#define OPSEBA_MOSA_MAX_TIME_GAP (5.0F)
/*! Above this object velocity all only moved distance check is applied */
#define OPSEBA_MOSA_MAX_VABS_FOR_CHECKS (5.0F)

/*! Factor to determine the min absolute velocity necessary. At 60kph the
 * necessary speed is 4.8kph. */
#define OPSEBA_MOSA_MIN_ABS_VELOCITY_FACTOR (0.08F)
/*! Minimum of the necessary absolute velocity regardless the ego speed */
#define OPSEBA_MOSA_MIN_ABS_VELOCITY_MIN (0.5F)
/*! Maximum of the necessary absolute velocity regardless the ego speed */
#define OPSEBA_MOSA_MIN_ABS_VELOCITY_MAX (2.0F)

/*! Min object age to set moving safe permission if the object is a pedestrian
 * candidate */
#define OPSEBA_MOSA_MIN_LIFE_CYCLES_PEDCAND (5u)
/*! Min dist y of an object to handle it as ped candidate */
#define OPSEBA_MOSA_MIN_DISTY_PEDCAND (2.0F)
/*! Absolute velocity necessary when an object is a pedestrian candidate */
#define OPSEBA_MOSA_MIN_ABS_VELOCITY_PEDCAND (0.5F)

/*! Min tunnel probability to assume tunnel.
        Do not use other values than 0.5, because a state machine is included
   here */
#define OPSEBA_MOSA_INTUNNEL_PROB_THRESH (0.5F)

/*! Min movement over ground in tunnel to have permission LVL2 */
#define OPSEBA_MOSA_MIN_MOVEMENT_INTUNNEL_PERMISSION_LVL2 (15.0F)
/*! Min movement over ground in tunnel to have permission LVL1 */
#define OPSEBA_MOSA_MIN_MOVEMENT_INTUNNEL_PERMISSION_LVL1 (9.0F)

/*! Min movement over ground to have permission LVL2 */
#define OPSEBA_MOSA_MIN_MOVEMENT_PERMISSION_LVL2 (6.0F)
/*! Min movement over ground to have permission LVL1 */
#define OPSEBA_MOSA_MIN_MOVEMENT_PERMISSION_LVL1 (4.0F)

/*! Min movement over ground for ped candidates to have permission LVL2 */
#define OPSEBA_MOSA_MIN_MOVEMENT_PERMISSION_LVL2_PED_CAND (1.0F)
/*! Min movement over ground for ped candidates to have permission LVL1 */
#define OPSEBA_MOSA_MIN_MOVEMENT_PERMISSION_LVL1_PED_CAND (0.5F)

#if (ALGO_SensorType == ALGO_CFG_CR400Entry)
/*! Min obstacle probability for stationary objects pickup */
#define OPSEBA_MOSA_MIN_OBSTACLE_PROB_STAT_PICKUP (60u)
/*! Min obstacle probability for stationary objects drop */
#define OPSEBA_MOSA_MIN_OBSTACLE_PROB_STAT_DROP (53u)
#else
/*! Min obstacle probability for stationary objects pickup */
#define OPSEBA_MOSA_MIN_OBSTACLE_PROB_STAT_PICKUP (66u)
/*! Min obstacle probability for stationary objects drop */
#define OPSEBA_MOSA_MIN_OBSTACLE_PROB_STAT_DROP (54u)
#endif

/*! Cycles to average speed and way.  @min 0   @max 8   */
#define OPSEBA_MOSA_VELO_WAY_PLAUSI_AVERAGING_CYCLES (8u)
/*! Allowed relative difference for near objects */
#define OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_NEAR (0.5F)
/*! Allowed relative difference for far objects */
#define OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_FAR (0.2F)
/*! Below this object distance the relative difference threshold for near
 * objects is used */
#define OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_NEAR_THRESH (2.0F)
/*! Below this object distance the relative difference threshold for far objects
 * is used */
#define OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_FAR_THRESH (10.0F)
/*! Tolerance of the position difference per cycle
        @attention   Value has to be greater than 0 */
#define OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_TOLERANCE (0.01F)
/*! Bit mask for the target association history of the current cycle */
#define OPSEBA_MOSA_VELO_WAY_BITMASK_CURRENT_CYCLE (128u)
/*! Max value for uint8 as float */
#define OPSEBA_MOSA_VELO_WAY_UINT8_MAX (255.0F)
/*! Factor to increase the relative difference threshold if the object is a ped
 * candidate */
#define OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_THRESH_PED_INCR (0.5F)
/*! Filter constant for the filtered relative difference */
#define OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_FILTER_CONST_UP (0.6F)
/*! Filter constant for the filtered relative difference */
#define OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_FILTER_CONST_DOWN (0.3F)
/* Transformation constant between calculation and saving value */
#define OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_TRAFO (100.0F)

/*! Threshold to switch of movement integration in curves */
#define OPSEBA_MOSA_MAX_RADIUS_FOR_MOVEMENT_INTEGRATION (50.0F)
/*! Threshold to switch of movement integration for very slow objects */
#define OPSEBA_MOSA_MIN_OBJ_VELOCITY_FOR_MOVEMENT_INTEGRATION (0.25F)

#define OPSEBA_MOSA_PCC_CONFIRM_NONE (0u)
#define OPSEBA_MOSA_PCC_CONFIRM_MICRODOPPLER (1u)
#define OPSEBA_MOSA_PCC_CONFIRM_MICRODOPPLER_TRAJ (2u)

/*! Internal defines */
#define OPSEBA_MOSA_MOVDIST_RANGE          \
    ((sizeof(iOPSEBA_MOSA_MovDist_t) == 1) \
         ? 255U                            \
         : ((sizeof(iOPSEBA_MOSA_MovDist_t) == 2) ? 65535U : 4294967295U))
#define OPSEBA_MOSA_MOVDIST_MAX ((OPSEBA_MOSA_MOVDIST_RANGE / 2U) - 1U)
#define OPSEBA_MOSA_MOVDIST_VALUE_RANGE (210.0F)
#define OPSEBA_MOSA_CONV_FACTOR \
    ((float32)OPSEBA_MOSA_MOVDIST_RANGE / OPSEBA_MOSA_MOVDIST_VALUE_RANGE)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
typedef sint16 iOPSEBA_MOSA_MovDist_t;

typedef struct {
    boolean bMovedFarEnoughLvl1;
    boolean bMovedFarEnoughLvl2;
    boolean bWasObsWhenStat;
    boolean bWasFastEnough;
    boolean bIsPedCandidate;
    boolean bIsBelowMaxDistance;
    boolean bIsOldEnough;
    boolean bVeloWayPlausible;
} OPSEBA_MOSA_ObjFlags_t;

typedef struct {
    uint8 MovObjQual : 2;
    uint8 bWasObsWhenStat : 1;
    uint8 bWasFastEnough : 1;
    uint8 bMovedFarEnoughLvl1 : 1;
    uint8 bMovedFarEnoughLvl2 : 1;
    uint8 bIsPedCandidate : 1;
    uint8 : 0;
} OPSEBA_MOSA_ObjQual_t;

typedef struct {
    uint8 auCycleTime[OPSEBA_MOSA_VELO_WAY_PLAUSI_AVERAGING_CYCLES];
    uint8 uNumOfCycles;
} OPSEBA_MOSA_CycleTime_t;

typedef struct {
    OPSEBA_MOSA_ObjQual_t asObjQual[Envm_NR_PRIVOBJECTS];
    iOPSEBA_MOSA_MovDist_t aiDistXLast[Envm_NR_PRIVOBJECTS];
    iOPSEBA_MOSA_MovDist_t aiDistXMoved[Envm_NR_PRIVOBJECTS];
    iOPSEBA_MOSA_MovDist_t aiDistYLast[Envm_NR_PRIVOBJECTS];
    iOPSEBA_MOSA_MovDist_t aiDistYMoved[Envm_NR_PRIVOBJECTS];
    OPSEBA_MOSA_CycleTime_t sCycleTime;
    uint8 auRelDiff[Envm_NR_PRIVOBJECTS];
    iOPSEBA_MOSA_MovDist_t aiDistXMovAverage[Envm_NR_PRIVOBJECTS];
    iOPSEBA_MOSA_MovDist_t aiVabsXAverage[Envm_NR_PRIVOBJECTS];
    iOPSEBA_MOSA_MovDist_t aiDistYMovAverage[Envm_NR_PRIVOBJECTS];
    iOPSEBA_MOSA_MovDist_t aiVabsYAverage[Envm_NR_PRIVOBJECTS];
} OPSEBA_MOSA_Data_t;

/************************************************************************/
/* FUNCTIONS                                                   */
/************************************************************************/
void OPSMOSAInit(void);
void OPSEBA_MOSA_InitObj(EM_t_ObjNumber iObjNumber);
static float32 OPSEBA_MOSA_GetDistX(EM_t_ObjNumber iObj);
static float32 OPSEBA_MOSA_GetVabsX(EM_t_ObjNumber iObj);
static float32 OPSEBA_MOSA_GetVrelX(EM_t_ObjNumber iObj);
static float32 OPSEBA_MOSA_GetDistY(EM_t_ObjNumber iObj);
static float32 OPSEBA_MOSA_GetVabsY(EM_t_ObjNumber iObj);
static float32 OPSEBA_MOSA_GetVrelY(EM_t_ObjNumber iObj);

static iOPSEBA_MOSA_MovDist_t OPSEBA_MOSA_TrafFloatToMovDist(float32 fMovement);
static iOPSEBA_MOSA_MovDist_t OPSEBA_MOSA_TrafFloatToMovDistX(
    float32 fMovement);
static float32 OPSEBA_MOSA_Float2MovDist(float32 fMovDist);
static float32 OPSEBA_MOSA_MovDist2Float(sint32 iMovDist);
static float32 OPSEBA_MOSA_TrafMovDistXToFloat(
    iOPSEBA_MOSA_MovDist_t iMovement);
static float32 OPSEBA_MOSA_TrafMovDistToFloat(iOPSEBA_MOSA_MovDist_t iMovement);
static uint8 OPSEBA_MOSA_GetMeasHistory(EM_t_ObjNumber iObjNumber);

void OPSEBA_MOSA_PreProcess(void);
void OPSEBA_MOSA_UpdateCycleTime(void);
void OPSEBA_MOSA_Process(EM_t_ObjNumber iObjNumber);

static uint32 OPSEBA_MOSA_GetObjLifeCycles(EM_t_ObjNumber iObjNumber);
static void OPSEBA_MOSA_IntegrateMovement(EM_t_ObjNumber iObjNumber);
static float32 OPSEBA_MOSA_CompensateEgoMovement(float32 fDist,
                                                 float32 fDistLeverArm,
                                                 float32 fYawRate,
                                                 float32 fEgoSpeed,
                                                 float32 fEgoSpeedOrtho);
static void OPSEBA_MOSA_UpdateAverageMovement(
    EM_t_ObjNumber iObjNumber,
    iOPSEBA_MOSA_MovDist_t* piAverageMovement,
    float32 fCurrentMovement);
static float32 OPSEBA_MOSA_GetSumCycleTime(EM_t_ObjNumber iObjNumber,
                                           uint32 uNumOfCycles);

static void OPSEBA_MOSA_UpdateAverageVelocity(EM_t_ObjNumber iObjNumber);

static void OPSEBA_MOSA_UpdatePedFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags);
static void OPSEBA_MOSA_UpdateStatObstacleFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags);
static void OPSEBA_MOSA_UpdateFastEnoughFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags);
static void OPSEBA_MOSA_UpdateMovementFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags);
static void OPSEBA_MOSA_UpdateIsOldEnoughFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags);
static void OPSEBA_MOSA_UpdateBelowMaxDistanceFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags);
static void OPSEBA_MOSA_UpdateVeloWayFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags);

static uint8 OPSEBA_MOSA_IsMicrodopplerConfirmed(EM_t_ObjNumber iObjNumber);
static boolean OPSEBA_MOSA_GetTunnelState(void);
static float32 OPSEBA_MOSA_CalculateRelDiffOfWayAndVelo(
    float32 fPosDistAverage, float32 fPosVeloAverage);

static void OPSEBA_MOSA_CheckConditions(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t const* const pCurrentCheckFlags);
static void OPSEBA_MOSA_SaveStaticData(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t const* const pCurrentCheckFlags);
uint8 OPSEBA_MOSA_GetObjPermission(EM_t_ObjNumber iObjNumber);

#if OPS_UNIT_TEST_SWITCH == TRUE
void SetMOSAData(OPSEBA_MOSA_Data_t* pMOSAData);
iOPSEBA_MOSA_MovDist_t UnitTest_OPSEBA_MOSA_TrafFloatToMovDist(
    float32 fMovement);
iOPSEBA_MOSA_MovDist_t UnitTest_OPSEBA_MOSA_IntegrateMovement(
    EM_t_ObjNumber iObjNumber);
boolean UnitTest_OPSEBA_MOSA_UpdateStatObstacleFlag(EM_t_ObjNumber iObjNumber);
boolean UnitTest_OPSEBA_MOSA_UpdateFastEnoughFlag(EM_t_ObjNumber iObjNumber);
OPSEBA_MOSA_ObjFlags_t* UnitTest_OPSEBA_MOSA_UpdateMovementFlag(
    EM_t_ObjNumber iObjNumber);
boolean UnitTest_OPSEBA_MOSA_UpdateIsOldEnoughFlag(EM_t_ObjNumber iObjNumber);
boolean UnitTest_OPSEBA_MOSA_UpdateBelowMaxDistanceFlag(
    EM_t_ObjNumber iObjNumber);
boolean UnitTest_OPSEBA_MOSA_UpdateVeloWayFlag(EM_t_ObjNumber iObjNumber);
uint8 UnitTest_OPSEBA_MOSA_CheckConditions(
    EM_t_ObjNumber iObjNumber, OPSEBA_MOSA_ObjFlags_t* pCurrentCheckFlag);
#endif

#endif
