/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
#include "TM_Global_Types.h"
#include "envm_ext.h"
#include "envm_consts.h"
#include "TM_Global_Types.h"
#include "stddef.h"
#include "assert.h"
#include <tue_common_libs.h>
#include "envm_io.h"
/******************************************************************************/

/* Start EM default section for code and data used for MPU and cache definition.
   These two lines need to be at the start of every EM C file, but last include,
   to identify that code in this file is EM code and uses EM memory. */

/*****************************************************************************
  MACROS
*****************************************************************************/

///*! size of 'VDY delay buffer' - determines the maximum possible latency
/// compensation */
///*! max latency = VDY_DELAY_BUFFER_SIZE * 'average cycle time' */
///*! adjust carefully as it has a large influence on the heap usage */
#define VDY_DELAY_BUFFER_SIZE (4UL)
//
///*! minimum number of 'VDY delay' samples required for leaving INIT state of
/// EM module */
///*! due to calculating differences between current and last frame, at least
/// two 'VDY delay' samples are required */
#define MIN_NUM_SAMPLES_VDY_DELAY_BUFFER (2UL)

/*! default distance from back axle to COG used for dist to rotation center
 * calculation */
#define EM_ROT_DEFAULT_DIST2AXLE (1.75f)
/*! default distance from back axle to COG used for dist to rotation center
 * calculation */
#define EM_ROT_DEFAULT_LONGPOS2COG (2.0f)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
typedef enum { /* way of interpolation between two samples */
               EM_DELAY_INTER_LINEAR = 0,  // linear interpolation
               EM_DELAY_INTER_NEAREST = 1  // nearest neighbor
} EMDelayInterpolateMode_t;

/*! buffer containing 'VDY delay' samples */
typedef VEDVehDyn_t t_a_vdyDelayBuff[VDY_DELAY_BUFFER_SIZE];

/*! buffer of corresponding timestamps of 'VDY delay' samples */
typedef uint32 t_a_vdyDelayTimestamps[VDY_DELAY_BUFFER_SIZE];

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! custom io symbolic constants */
#define NWM_MIN_WRITE_INTERVAL (900.0f) /* in seconds ( = 15 minutes) */

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/* input variables */
/**********************/
/* Delay Line Buffers */
/**********************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! buffer containing 'VDY delay' samples (organized as ring buffer) */
static t_a_vdyDelayBuff a_emEgoVdyBuff;

/*! buffer of corresponding timestamps of 'VDY delay' samples */
static t_a_vdyDelayTimestamps a_emEgoVdyTimes;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*! number of entries in 'VDY delay line' */
static uint32 u_numVdyBuffElem = 0UL;

/*! points to index in 'VDY delay line' the next 'VDY sample' should be inserted
 * into */
static uint32 u_insertPosVdyBuff = 0UL;

/*! points to index of currently inserted 'VDY sample' in 'VDY delay line' */
static uint32 u_newestSamplePosVdyBuff = 0UL;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! flag for signaling that 'VDY delay buffer' can be reset */
static boolean b_resetDelayBuff = TRUE;

#if (CFG_Envm_EGO_YAWRATE_COMPENSATION == CFG_Envm_SWITCH_ON)
/*! sensor offset to rotation center */
float32 f_GlobalLongPosToRot;
float32 f_GlobalObjSyncCosFloatAng;
float32 f_GlobalCluSyncCosFloatAng;
#endif
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

/* input functions */
static void EM_v_InterpolateVdyDelayData(uint32 u_emTimestamp);

/* output functions */
/* process the validity of internal object list status */
static void EM_v_ProcessObjectListHeader(void);

/* copy tunnel probability */
static void EM_v_CopyTunnelProbability(void);

/* Process Custom EM Output. */
static void EM_v_ProcessCustomOutput(void);

/* Handling of 'VDY delay buffer' */

/* Update 'VDY delay buffer' with current 'VDY sample' */
static boolean EM_b_UpdateVdyDelayData(const VEDVehDyn_t *p_vdyData,
                                       uint32 u_vdyTimestamp);

/* Get the delayed and interpolated signal from the 'VDY delay buffer' */
static float32 EM_f_GetDelayedVdySignal(uint32 u_emTimestamp,
                                        float32 f_signalDelay,
                                        const float32 *p_delayedSignalValue,
                                        EMDelayInterpolateMode_t interpolMode);

/* Set information of EgoCurve */
static void EM_v_SetEgoCurve(uint32 u_emTimestamp);

/* Set information of EgoDrvIntCurve */
static void EM_v_SetEgoDrvIntCurve(uint32 u_emTimestamp);

/* Set information of EgoYawRate */
static void EM_v_SetEgoYawRate(uint32 u_emTimestamp);

/* Set information of EgoSlipAngle */
static void EM_v_SetEgoSlipAngle(uint32 u_emTimestamp);

/* Set information of EgoSpeedX */
static void EM_v_SetEgoSpeedX(uint32 u_emTimestamp);

/* Set information of EgoAccelX */
static void EM_v_SetEgoAccelX(uint32 u_emTimestamp);

#if (CFG_Envm_EGO_YAWRATE_COMPENSATION == CFG_Envm_SWITCH_ON)
/* Set sensor offset to the rotation center */
static void EM_v_SetYawrateCompSignals(void);
#endif

/*****************************************************************************
  INPUT FUNCTION
*****************************************************************************/

/* ****************************************************************************

  Functionname:     EM_v_ResetVdyDelayData                                   */ /*!

  @brief            Resets 'VDY delay buffer'.

  @description      Clears VDY delay buffer memory. Resets its handling
variables
                    Marks current state as being Reset.

  @return           void

  @pre              None
  @post             No changes


**************************************************************************** */
void EM_v_ResetVdyDelayData(void) {
    /* clear memory */
    (void)memset(&a_emEgoVdyBuff, 0UL, sizeof(a_emEgoVdyBuff));
    (void)memset(&a_emEgoVdyTimes, 0UL, sizeof(a_emEgoVdyTimes));

    /* reset 'handling' of 'VDY delay buffer' */
    u_numVdyBuffElem = 0UL;
    u_insertPosVdyBuff = 0UL;
    u_newestSamplePosVdyBuff = 0UL;

    /* mark 'delay buffer' as being reset (until new samples arrive) */
    b_resetDelayBuff = FALSE;
}

/* ****************************************************************************

  Functionname:     EM_v_InitVdyDelayData                                   */ /*!

   @brief            Inits 'VDY delay buffer' with current 'VehDyn' sample and
                     updates 'VDY delay signals'.

   @description      Gets current RSP timestamp used as 'EM time'.
                     Resets 'VDY delay buffer'. Fills 'VDY delay buffer' and
 updates
                     corresponding 'VDY delay signals' if valid signals are
 available.

   @return           void

   @pre              None
   @post             No changes


 **************************************************************************** */
void EM_v_InitVdyDelayData(void) {
    // const VEDVehDyn_t *p_vehDyn = GET_EGO_RAW_DATA_PTR;
    const VEDVehDyn_t *p_vehDyn = EnvmData.pEgoDynRaw;
    uint32 u_emTimestamp = 0UL;

    /**
     * get current RSP timestamp used as 'EM time'.
     * currently, this RSP time is also used for timestamping
     * 'VDY samples'.
     */

    u_emTimestamp = EM_f_GetRSPTimeStamp(SYS_SCAN_NEAR);

    /* reset 'VDY delay buffer' (only once) */
    if (b_resetDelayBuff == TRUE) {
        EM_v_ResetVdyDelayData();
        b_resetDelayBuff = FALSE;
    }

    /**
     * fill 'VDY delay buffer' and update corresponding 'VDY delay signals'.
     * but skip processing 'DelayLine' in case of having
     * Envm_MOD_INVALID_INPUT_SIGNALS active.
     */
    if (EnvmData.pFrame->eEnvmOpMode != Envm_MOD_INVALID_INPUT_SIGNALS) {
        (void)EM_b_UpdateVdyDelayData(p_vehDyn, u_emTimestamp);
        EM_v_InterpolateVdyDelayData(u_emTimestamp);
    }
}

/* ****************************************************************************

  Functionname:     EM_b_UpdateVdyDelayData                                   */ /*!

 @brief            Inserts passed 'VehDyn' sample into 'VDY delay buffer'.

 @description      Passed 'VehDyn' sample is inserted into 'VDY delay (ring)
buffer'.
                   In case of already having a 'filled' 'VDY delay buffer'
                   inserting another sample results in dropping the oldest one.
                   Checks if VDY data is valid and if this sample is already
known.
                   The 'newest' entry always represents a 'full' timestamp and
                   the remaining ones are referred to that timestamp by
calculating differences.
                   Hence updates all the old timestamps before inserting new
buffer.
                   Also updates next 'insert position'.

 @param[in]        p_vdyData       : 'VehDyn' sample being inserted into 'VDY
delay buffer'.
 @param[in]        u_vdyTimestamp  : Corresponding timestamp of current 'VehDyn'
sample.

 @return           True if VDY sample could be inserted correctly.
                   False if VDY sample is not valid or already known,
respectively.

 @pre              None
 @post             No changes


**************************************************************************** */
static boolean EM_b_UpdateVdyDelayData(const VEDVehDyn_t *p_vdyData,
                                       uint32 u_vdyTimestamp) {
    boolean b_dataIsValid = TRUE;
    uint32 u_vdyCycleTime = 0UL, u_count = 0UL, u_bufIdx = 0UL;

    /* only use valid VDY data */
    if ((((p_vdyData->sSigHeader).eSigStatus) != AL_SIG_STATE_OK) ||
        ((EnvmData.pFrame->eEnvmOpMode) == Envm_MOD_INVALID_INPUT_SIGNALS)) {
        b_dataIsValid = FALSE;
    }

    /* check if this sample is already known (compare it with the last one) */
    if (a_emEgoVdyBuff[u_newestSamplePosVdyBuff].sSigHeader.uiCycleCounter ==
        p_vdyData->sSigHeader.uiCycleCounter) {
        b_dataIsValid = FALSE;
    }

    if (b_dataIsValid == TRUE) {
        if (u_numVdyBuffElem > 0) {
            // 'overflows in time' are handled correctly due to 'unsigned wrap
            // around'. but how to handle arrival of non-monotonous samples?
            // ('u_vdyTimestamp' < 'lastTimestamp') this would result in large
            // cycle times which can be detected and handled (-> reset?)
            // -> currently, it is assumed that monotonous timestamps are
            // ensured!
            //    (and thus, 'negative differences' cannot occur!)

            /* 'VDY cycle time' */
            u_vdyCycleTime =
                u_vdyTimestamp - a_emEgoVdyTimes[u_newestSamplePosVdyBuff];

            /**
             * update 'time differences' of buffer entries with regard to the
             * current 'VDY sample'. since entry of last sample contains a
             * 'plain' timestamp, it has to be 'cleared' and updated with the
             * current 'VDY cycle time' to represent the correct differences.
             */
            u_bufIdx = u_newestSamplePosVdyBuff;
            a_emEgoVdyTimes[u_bufIdx] = u_vdyCycleTime;

            /* start one position 'behind' last sample and stop at 'insert
             * position' */
            for (u_count = 1UL; u_count < (u_numVdyBuffElem - 1UL); ++u_count) {
                /* update index while iterating 'VDY time buffer' ('clockwise')
                 */
                u_bufIdx = (u_bufIdx == 0UL) ? (u_numVdyBuffElem - 1UL)
                                             : (u_bufIdx - 1UL);

                /* update differences by adding 'VDY cycle time' */
                a_emEgoVdyTimes[u_bufIdx] += u_vdyCycleTime;
            }

        } /* if (u_numVdyBuffElem > 0) */

        // validate 'insertPos'? (>= 0, <= 'BUF_SIZE')
        /* insert current 'VDY sample' into 'delay buffer' */
        (void)memcpy(&a_emEgoVdyBuff[u_insertPosVdyBuff], p_vdyData,
                     sizeof(VEDVehDyn_t));
        a_emEgoVdyTimes[u_insertPosVdyBuff] = u_vdyTimestamp;

        /* update index of 'newest' sample */
        u_newestSamplePosVdyBuff = u_insertPosVdyBuff;

        /**
         * remember 'direction' of longitudinal ego motion {FORWARD, BACKWARD}
         * in 'VDY delay buffer'. raw VDY signals are unsigned, the motion
         * direction is given by a specific flag. take this flag into account to
         * regard direction in 'VDY delay buffer' (and later on in 'delayed
         * structures' 'EgoDynObjSync' and 'EgoDynTgtSync').
         */
        if (p_vdyData->MotionState.MotState == VED_LONG_MOT_STATE_MOVE_RWD) {
            /* moving backward -> change sign of (raw) velocity and acceleration
             * values */
            a_emEgoVdyBuff[u_insertPosVdyBuff].Longitudinal.MotVar.Accel =
                -a_emEgoVdyBuff[u_insertPosVdyBuff].Longitudinal.MotVar.Accel;
            a_emEgoVdyBuff[u_insertPosVdyBuff].Longitudinal.VeloCorr.corrVelo =
                -a_emEgoVdyBuff[u_insertPosVdyBuff]
                     .Longitudinal.VeloCorr.corrVelo;
        }

        /* update number of buffer entries */
        if (u_numVdyBuffElem < VDY_DELAY_BUFFER_SIZE) {
            ++u_numVdyBuffElem;
        }

        /* update next 'insert position' */
        u_insertPosVdyBuff = (u_insertPosVdyBuff + 1UL) % VDY_DELAY_BUFFER_SIZE;

        /* Fallback solution to handle unexpected zero variance of
         * Longitudinal.VeloCorr.corrVeloVar              */
        /* -> In order to avoid potential floating point errors (Div0) we limit
         * the variance to BML_f_AlmostZero  */
        a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
            .Longitudinal.VeloCorr.corrVeloVar =
            MAX(TUE_CML_AlmostZero, a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
                                        .Longitudinal.VeloCorr.corrVeloVar);
    } /* if (b_dataIsValid == TRUE) */

    return b_dataIsValid;
}

/*************************************************************************************************************************
Functionname:    EM_f_GetDelayedVdySignal */
static float32 EM_f_GetDelayedVdySignal(uint32 u_emTimestamp,
                                        float32 f_signalDelay,
                                        const float32 *p_delayedSignalValue,
                                        EMDelayInterpolateMode_t interpolMode) {
    /**
     * get 'address offset' between 'beginning' of entry in 'VDY delay buffer'
     * containing the 'newest' 'VSY sample' and current 'delayedSignalValue'
     * (within this buffer entry). used for iterating samples of
     * 'delayedSignalValue' in 'VDY delay buffer'.
     */

    const uint32 u_pointerOffset =
        (uint32)((const uint8 *)p_delayedSignalValue -
                 (uint8 *)(&a_emEgoVdyBuff[u_newestSamplePosVdyBuff]));

    float32 f_cycleTime = 0.f, f_sigNewer = 0.f, f_sigOlder = 0.f,
            f_interpolatedSig = 0.f;
    uint32 u_count = 0UL, u_diffNewer = 0UL, u_diffOlder = 0UL;
    uint32 u_idxNewer = 0UL, u_idxOlder = 0UL;
    sint32 s_targetTimeDiff = 0L;

    boolean b_exitLoop = FALSE;

    /**
     * get 'target time difference' of the 'EM sample' with regard to the
     * current 'VDY sample' by taking 'signal-delay' into account. this 'target
     * difference' is used to identify the correct two samples of the 'VDY delay
     * buffer' used for interpolation.
     */
    s_targetTimeDiff = ((sint32)a_emEgoVdyTimes[u_newestSamplePosVdyBuff] -
                        (sint32)u_emTimestamp) +
                       ((sint32)(f_signalDelay * 1000.f));

    /* first look for (two) target buffer entries being close to the
     * 'timestamp-of-interest'. */
    u_idxNewer = u_newestSamplePosVdyBuff;
    u_idxOlder =
        (u_idxNewer == 0UL) ? (u_numVdyBuffElem - 1UL) : (u_idxNewer - 1UL);

    /* newest sample has no 'time difference' */
    u_diffNewer = 0UL;
    u_diffOlder = a_emEgoVdyTimes[u_idxOlder];

    for (u_count = 1UL; (u_count < u_numVdyBuffElem) && (b_exitLoop == FALSE);
         ++u_count) {
        /**
         * if 'target time difference' is between 'newer' and 'older'
         * we will have identified the affected 'VDY samples'.
         */

        if ((s_targetTimeDiff >= (sint32)u_diffNewer) &&
            (s_targetTimeDiff <= (sint32)u_diffOlder)) {
            /* get corresponding signal values */
            f_sigNewer = *((float32 *)((uint8 *)(&a_emEgoVdyBuff[u_idxNewer]) +
                                       u_pointerOffset));
            f_sigOlder = *((float32 *)((uint8 *)(&a_emEgoVdyBuff[u_idxOlder]) +
                                       u_pointerOffset));

            /* save corresponding VDY cycle time */
            f_cycleTime = (float32)(u_diffOlder - u_diffNewer);

            /* leave loop if 'samples-of-interest' are found */
            b_exitLoop = TRUE;
        }

        if (b_exitLoop == FALSE) {
            /* update indices and timestamps while iterating 'VDY delay buffer'
             * (if loop is not to be left) */
            u_idxNewer = (u_idxNewer == 0UL) ? (u_numVdyBuffElem - 1UL)
                                             : (u_idxNewer - 1UL);
            u_idxOlder = (u_idxOlder == 0UL) ? (u_numVdyBuffElem - 1UL)
                                             : (u_idxOlder - 1UL);

            u_diffNewer = a_emEgoVdyTimes[u_idxNewer];
            u_diffOlder = a_emEgoVdyTimes[u_idxOlder];
        }
    }

    /**
     * interpolate delayed signal between 'older' and 'newer'.
     * ensure that two samples have been identified.
     * in case of not having TWO 'matching' 'VDY delay samples'
     * the 'EM sample' must exceed the 'buffered' 'VDY time interval'.
     *    -> 'underflow' if 'EM sample' is older than the oldest 'VDY delay
     * sample'
     *    -> 'overflow' if 'EM sample' is newer than the newest 'VDY delay
     * sample'
     *  => in case of exceeding the buffered interval, use the 'nearest'
     *     'VDY delay sample' which is the first and the last sample in the 'VDY
     * delay buffer', respectively (depends on 'EM sample' timestamp).
     */
    if (b_exitLoop == TRUE) {
        /* found two 'VDY samples' where 'EM sample' is in-between */

        if (interpolMode == EM_DELAY_INTER_NEAREST) {
            /* get nearest neighbor */
            f_interpolatedSig =
                ((s_targetTimeDiff + (sint32)(f_cycleTime / 2.f)) >=
                 (sint32)u_diffOlder)
                    ? f_sigOlder
                    : f_sigNewer;
        } else {
            /* default: linear interpolation */

            /**
             * check if interpolation is needed.
             *  ->  only if 'EM sample' lies in-between two 'VDY samples'.
             * otherwise, the signal value matching the timestamp of the 'EM
             * sample' is taken.
             */
            if (s_targetTimeDiff == (sint32)u_diffNewer) {
                /* take newer 'VDY sample' */
                f_interpolatedSig = f_sigNewer;
            } else if (s_targetTimeDiff == (sint32)u_diffOlder) {
                /* take older 'VDY sample' */
                f_interpolatedSig = f_sigOlder;
            } else {
                /* interpolation */

                if (TUE_CML_IsNonZero(f_cycleTime)) {
                    /* timestamps are in microseconds */
                    f_interpolatedSig =
                        ((f_sigNewer - f_sigOlder) *
                         (float32)((sint32)u_diffOlder - s_targetTimeDiff)) /
                        (f_cycleTime);
                    f_interpolatedSig += f_sigOlder;
                } else {
                    /* fallback: use 'newer' sample */
                    f_interpolatedSig = f_sigNewer;
                }

            } /* else of 'if (s_targetTimeDiff == (sint32)u_diffNewer)' */

        } /* else of 'if (interpolMode == EM_DELAY_INTER_NEAREST)' */

    } /* if (b_exitLoop == TRUE) */
    else {
        /**
         * no corresponding VDY samples found in 'VDY delay buffer'.
         * check which of the 'time borders' {'left', right'} is crossed
         * and take the 'nearest' delay sample as 'sample-of-interest'.
         * the circumstance of not having 'VDY delay samples' where
         * the 'EM sample' is in-between is noticed by an 'ErrorTrap'.
         */
        if (u_numVdyBuffElem == 1UL) {
            /* one-element-buffer is handled separately */
            f_interpolatedSig = *(p_delayedSignalValue);
        } else if (u_numVdyBuffElem > 1UL) {
            /**
             * use 'newest' and 'oldest' sample as 'fallback', respectively.
             * note that 'newest' sample has a difference of zero.
             * positive differences mean 'older' in time, whereas negative ones
             * represent a 'future' point in time (with regard to 'VDY time').
             */

            if (s_targetTimeDiff < 0L) {
                /**
                 * 'EM sample' is newer than most current 'VDY delay sample' =>
                 * 'overflow'.
                 *  -> take the newest 'VDY delay sample' (which is the passed
                 * one).
                 */
                f_interpolatedSig = *(p_delayedSignalValue);

                /* notice 'overflow' */
                EMErrorTrap(__FILE__, __LINE__, EM_ERRORTRAP_TYPE_ERROR);
            } else if ((u_numVdyBuffElem <= VDY_DELAY_BUFFER_SIZE) &&
                       (s_targetTimeDiff >
                        (sint32)a_emEgoVdyTimes[u_insertPosVdyBuff])) {
                /**
                 * 'EM sample' is older than oldest 'VDY delay sample' =>
                 * 'underflow'.
                 *  -> take the oldest 'VDY delay sample'.
                 *
                 * note that the 'oldest' sample is the one being marked as
                 * 'rewritable' in the next cycle. thus, the next 'insert
                 * position' points to the sample being searched for.
                 */

                f_interpolatedSig = *(
                    (float32 *)((uint8 *)(&a_emEgoVdyBuff[u_insertPosVdyBuff]) +
                                u_pointerOffset));

                /* report error in situations were the delay line should be
                 * filled correctly */
                if (EnvmData.pFrame->uiCycleCounter >
                    (VDY_DELAY_BUFFER_SIZE + 1)) {
                    /* notice 'underflow' */
                    EMErrorTrap(__FILE__, __LINE__, EM_ERRORTRAP_TYPE_ERROR);
                }
            } else {
            }

        } /* else-if of 'if (u_numVdyBuffElem == 1UL)' */
        else {
        } /* else of 'if (u_numVdyBuffElem == 1UL)' */

    } /* else of 'if (b_exitLoop == TRUE)' */

    return f_interpolatedSig;
}

/* ****************************************************************************

  Functionname:     EM_v_SetEgoCurve                                        */ /*!

   @brief            Set information of EgoCurve.

   @description      The ego data(curve, varC0, Gradient) is delayed according
 to the target and object
                     latencies.

   @param[in]        u_emTimestamp : EM timestamp the VDY signal data should be
 interpolated to.

   @return           void

   @pre              None
   @post             No changes


 **************************************************************************** */
static void EM_v_SetEgoCurve(uint32 u_emTimestamp) {
    /* get output pointer */
    VEDVehDyn_t *p_egoDynTgtSync = EnvmData.pEgoDynTgtSync;
    VEDVehDyn_t *p_egoDynObjSync = EnvmData.pEgoDynObjSync;

    p_egoDynTgtSync->Lateral.Curve.Curve = EM_f_GetDelayedVdySignal(
        u_emTimestamp, ODDELAY_f_curveTgtLatency,
        &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.Curve.Curve,
        EM_DELAY_INTER_LINEAR);
    p_egoDynTgtSync->Lateral.Curve.varC0 = EM_f_GetDelayedVdySignal(
        u_emTimestamp, ODDELAY_f_curveTgtLatency,
        &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.Curve.varC0,
        EM_DELAY_INTER_LINEAR);
    p_egoDynTgtSync->Lateral.Curve.Gradient = EM_f_GetDelayedVdySignal(
        u_emTimestamp, ODDELAY_f_curveTgtLatency,
        &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.Curve.Gradient,
        EM_DELAY_INTER_LINEAR);

    /* in case of having same latencies for 'targets' and 'objects' use the
     * already computed values */
    if (TUE_CML_IsZero(ODDELAY_f_curveTgtLatency - ODDELAY_f_curveObjLatency)) {
        p_egoDynObjSync->Lateral.Curve.Curve =
            p_egoDynTgtSync->Lateral.Curve.Curve;
        p_egoDynObjSync->Lateral.Curve.varC0 =
            p_egoDynTgtSync->Lateral.Curve.varC0;
        p_egoDynObjSync->Lateral.Curve.Gradient =
            p_egoDynTgtSync->Lateral.Curve.Gradient;
    } else {
        p_egoDynObjSync->Lateral.Curve.Curve = EM_f_GetDelayedVdySignal(
            u_emTimestamp, ODDELAY_f_curveObjLatency,
            &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.Curve.Curve,
            EM_DELAY_INTER_LINEAR);
        p_egoDynObjSync->Lateral.Curve.varC0 = EM_f_GetDelayedVdySignal(
            u_emTimestamp, ODDELAY_f_curveObjLatency,
            &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.Curve.varC0,
            EM_DELAY_INTER_LINEAR);
        p_egoDynObjSync->Lateral.Curve.Gradient = EM_f_GetDelayedVdySignal(
            u_emTimestamp, ODDELAY_f_curveObjLatency,
            &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.Curve.Gradient,
            EM_DELAY_INTER_LINEAR);
    }
}

/* ****************************************************************************

  Functionname:     EM_v_SetEgoDrvIntCurve                                  */ /*!

   @brief            Set information of EgoDrvIntCurve

   @description      Module global variable information will be copied to caller
                     The ego data (curve, variance, Gradient) is delayed
 according to the target and object
                     latencies

   @param[in]        u_emTimestamp : EM timestamp the VDY signal data should be
 interpolated to.

   @return           void

   @pre              None
   @post             No changes


 **************************************************************************** */
static void EM_v_SetEgoDrvIntCurve(uint32 u_emTimestamp) {
    /* get output pointer */
    VEDVehDyn_t *p_egoDynTgtSync = EnvmData.pEgoDynTgtSync;
    VEDVehDyn_t *p_egoDynObjSync = EnvmData.pEgoDynObjSync;

    p_egoDynTgtSync->Lateral.DrvIntCurve.Curve = EM_f_GetDelayedVdySignal(
        u_emTimestamp, ODDELAY_f_drvIntCurveTgtLatency,
        &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.DrvIntCurve.Curve,
        EM_DELAY_INTER_LINEAR);
    p_egoDynTgtSync->Lateral.DrvIntCurve.Variance = EM_f_GetDelayedVdySignal(
        u_emTimestamp, ODDELAY_f_drvIntCurveTgtLatency,
        &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.DrvIntCurve.Variance,
        EM_DELAY_INTER_LINEAR);
    p_egoDynTgtSync->Lateral.DrvIntCurve.Gradient = EM_f_GetDelayedVdySignal(
        u_emTimestamp, ODDELAY_f_drvIntCurveTgtLatency,
        &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.DrvIntCurve.Gradient,
        EM_DELAY_INTER_LINEAR);

    /* in case of having same latencies for 'targets' and 'objects' use the
     * already computed values */
    if (TUE_CML_IsZero(ODDELAY_f_drvIntCurveTgtLatency -
                       ODDELAY_f_drvIntCurveObjLatency)) {
        p_egoDynObjSync->Lateral.DrvIntCurve.Curve =
            p_egoDynTgtSync->Lateral.DrvIntCurve.Curve;
        p_egoDynObjSync->Lateral.DrvIntCurve.Variance =
            p_egoDynTgtSync->Lateral.DrvIntCurve.Variance;
        p_egoDynObjSync->Lateral.DrvIntCurve.Gradient =
            p_egoDynTgtSync->Lateral.DrvIntCurve.Gradient;
    } else {
        p_egoDynObjSync->Lateral.DrvIntCurve.Curve = EM_f_GetDelayedVdySignal(
            u_emTimestamp, ODDELAY_f_drvIntCurveObjLatency,
            &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.DrvIntCurve.Curve,
            EM_DELAY_INTER_LINEAR);
        p_egoDynObjSync->Lateral.DrvIntCurve.Variance =
            EM_f_GetDelayedVdySignal(u_emTimestamp,
                                     ODDELAY_f_drvIntCurveObjLatency,
                                     &a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
                                          .Lateral.DrvIntCurve.Variance,
                                     EM_DELAY_INTER_LINEAR);
        p_egoDynObjSync->Lateral.DrvIntCurve.Gradient =
            EM_f_GetDelayedVdySignal(u_emTimestamp,
                                     ODDELAY_f_drvIntCurveObjLatency,
                                     &a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
                                          .Lateral.DrvIntCurve.Gradient,
                                     EM_DELAY_INTER_LINEAR);
    }
}

/* ****************************************************************************

  Functionname:     EM_v_SetEgoYawRate                                      */ /*!

   @brief            Set information of EgoYawRate

   @description      Module global variable information will be copied to caller
                     The ego data (YawRate, Variance) is delayed
                     according to the target and object latencies.
                     Checks if external latency for acceleration shall be used.
                     Compute target sync jitter for both target and object
 latencies.

   @param[in]        u_emTimestamp : EM timestamp the VDY signal data should be
 interpolated to.

   @return           void

   @pre              None
   @post             No changes


 **************************************************************************** */
static void EM_v_SetEgoYawRate(uint32 u_emTimestamp) {
    f32_t f_yawRateJitterA = 0.f, f_yawRateJitterB = 0.f,
          f_absYawRateJitterA = 0.f, f_absYawRateJitterB = 0.f;
    f32_t f_yawRateLatency = 0.f, f_egoYawRateTgtSync,
          f_egoYawRateObjSync = 0.f;
    f32_t f_yawRateObjLatency = ODDELAY_f_yawRateObjLatency;

    /* get output pointer */
    VEDVehDyn_t *p_egoDynTgtSync = EnvmData.pEgoDynTgtSync;
    VEDVehDyn_t *p_egoDynObjSync = EnvmData.pEgoDynObjSync;

    /* store old value */
    EnvmData.pPrivGlob->fYawRateLast = p_egoDynTgtSync->Lateral.YawRate.YawRate;

    f_egoYawRateTgtSync = EM_f_GetDelayedVdySignal(
        u_emTimestamp, ODDELAY_f_yawRateTgtLatency,
        &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.YawRate.YawRate,
        EM_DELAY_INTER_LINEAR);
    p_egoDynTgtSync->Lateral.YawRate.YawRate = f_egoYawRateTgtSync;
    p_egoDynTgtSync->Lateral.YawRate.Variance = EM_f_GetDelayedVdySignal(
        u_emTimestamp, ODDELAY_f_yawRateTgtLatency,
        &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.YawRate.Variance,
        EM_DELAY_INTER_LINEAR);

    /* in case of having same latencies for 'targets' and 'objects' use the
     * already computed values */
    if (TUE_CML_IsZero(ODDELAY_f_yawRateTgtLatency - f_yawRateObjLatency)) {
        f_egoYawRateObjSync = f_egoYawRateTgtSync;
        p_egoDynObjSync->Lateral.YawRate.YawRate = f_egoYawRateObjSync;
        p_egoDynObjSync->Lateral.YawRate.Variance =
            p_egoDynTgtSync->Lateral.YawRate.Variance;
    } else {
        f_egoYawRateObjSync = EM_f_GetDelayedVdySignal(
            u_emTimestamp, f_yawRateObjLatency,
            &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.YawRate.YawRate,
            EM_DELAY_INTER_LINEAR);
        p_egoDynObjSync->Lateral.YawRate.YawRate = f_egoYawRateObjSync;
        p_egoDynObjSync->Lateral.YawRate.Variance = EM_f_GetDelayedVdySignal(
            u_emTimestamp, f_yawRateObjLatency,
            &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.YawRate.Variance,
            EM_DELAY_INTER_LINEAR);
    }

    /* compute target sync jitter */
    f_yawRateLatency = MAX(
        0.f, (ODDELAY_f_yawRateTgtLatency - ODDELAY_f_yawRateLatencyJitter));

    /* 'down' */
    f_yawRateJitterA = EM_f_GetDelayedVdySignal(
        u_emTimestamp, f_yawRateLatency,
        &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.YawRate.YawRate,
        EM_DELAY_INTER_LINEAR);

    f_yawRateLatency = MAX(
        0.f, (ODDELAY_f_yawRateTgtLatency + ODDELAY_f_yawRateLatencyJitter));

    /* 'up' */
    f_yawRateJitterB = EM_f_GetDelayedVdySignal(
        u_emTimestamp, f_yawRateLatency,
        &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.YawRate.YawRate,
        EM_DELAY_INTER_LINEAR);

    f_yawRateJitterA -= f_egoYawRateTgtSync;
    f_yawRateJitterB -= f_egoYawRateTgtSync;
    f_absYawRateJitterA = fABS(f_yawRateJitterA);
    f_absYawRateJitterB = fABS(f_yawRateJitterB);

    p_egoDynTgtSync->Legacy.YawRateMaxJitter =
        (f_absYawRateJitterA > f_absYawRateJitterB) ? f_yawRateJitterA
                                                    : f_yawRateJitterB;

    /* check if the already computed values can be used */
    if (TUE_CML_IsZero(ODDELAY_f_yawRateTgtLatency - f_yawRateObjLatency)) {
        p_egoDynObjSync->Legacy.YawRateMaxJitter =
            p_egoDynTgtSync->Legacy.YawRateMaxJitter;
    } else {
        /* compute object sync jitter */
        f_yawRateLatency =
            MAX(0.f, (f_yawRateObjLatency - ODDELAY_f_yawRateLatencyJitter));

        /* 'down' */
        f_yawRateJitterA = EM_f_GetDelayedVdySignal(
            u_emTimestamp, f_yawRateLatency,
            &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.YawRate.YawRate,
            EM_DELAY_INTER_LINEAR);

        f_yawRateLatency =
            MAX(0.f, (f_yawRateObjLatency + ODDELAY_f_yawRateLatencyJitter));

        /* 'up' */
        f_yawRateJitterB = EM_f_GetDelayedVdySignal(
            u_emTimestamp, f_yawRateLatency,
            &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.YawRate.YawRate,
            EM_DELAY_INTER_LINEAR);

        f_yawRateJitterA -= f_egoYawRateObjSync;
        f_yawRateJitterB -= f_egoYawRateObjSync;
        f_absYawRateJitterA = fABS(f_yawRateJitterA);
        f_absYawRateJitterB = fABS(f_yawRateJitterB);

        p_egoDynObjSync->Legacy.YawRateMaxJitter =
            (f_absYawRateJitterA > f_absYawRateJitterB) ? f_yawRateJitterA
                                                        : f_yawRateJitterB;
    }
}

/* ****************************************************************************

  Functionname:     EM_v_SetEgoSlipAngle                                    */ /*!

   @brief            Set information of EgoSlipAngle

   @description      Module global variable information will be copied to caller
                     The ego data (slide slip angle, variance) is delayed
 according to the target and object
                     latencies

   @param[in]        u_emTimestamp : EM timestamp the VDY signal data should be
 interpolated to.

   @return           void

   @pre              None
   @post             No changes


 **************************************************************************** */
static void EM_v_SetEgoSlipAngle(uint32 u_emTimestamp) {
    /* get output pointer */
    VEDVehDyn_t *p_egoDynTgtSync = EnvmData.pEgoDynTgtSync;
    VEDVehDyn_t *p_egoDynObjSync = EnvmData.pEgoDynObjSync;

    p_egoDynTgtSync->Lateral.SlipAngle.SideSlipAngle =
        EM_f_GetDelayedVdySignal(u_emTimestamp, ODDELAY_f_slipAngleTgtLatency,
                                 &a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
                                      .Lateral.SlipAngle.SideSlipAngle,
                                 EM_DELAY_INTER_LINEAR);
    p_egoDynTgtSync->Lateral.SlipAngle.Variance = EM_f_GetDelayedVdySignal(
        u_emTimestamp, ODDELAY_f_slipAngleTgtLatency,
        &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Lateral.SlipAngle.Variance,
        EM_DELAY_INTER_LINEAR);

    /* in case of having same latencies for 'targets' and 'objects' use the
     * already computed values */
    if (TUE_CML_IsZero(ODDELAY_f_slipAngleTgtLatency -
                       ODDELAY_f_slipAngleObjLatency)) {
        p_egoDynObjSync->Lateral.SlipAngle.SideSlipAngle =
            p_egoDynTgtSync->Lateral.SlipAngle.SideSlipAngle;
        p_egoDynObjSync->Lateral.SlipAngle.Variance =
            p_egoDynTgtSync->Lateral.SlipAngle.Variance;
    } else {
        p_egoDynObjSync->Lateral.SlipAngle.SideSlipAngle =
            EM_f_GetDelayedVdySignal(u_emTimestamp,
                                     ODDELAY_f_slipAngleObjLatency,
                                     &a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
                                          .Lateral.SlipAngle.SideSlipAngle,
                                     EM_DELAY_INTER_LINEAR);
        p_egoDynObjSync->Lateral.SlipAngle.Variance = EM_f_GetDelayedVdySignal(
            u_emTimestamp, ODDELAY_f_slipAngleObjLatency,
            &a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
                 .Lateral.SlipAngle.Variance,
            EM_DELAY_INTER_LINEAR);
    }
}

/* ****************************************************************************

  Functionname:     EM_v_SetEgoSpeedX                                       */ /*!

   @brief            Set function for fTPEgoSpeedX

   @description      Module global variable information will be copied to caller
                     The ego data is delayed according to the target and object
                     latencies

   @param[in]        u_emTimestamp : EM timestamp the VDY signal data should be
 interpolated to.

   @return           void

   @pre              None
   @post             No changes


 **************************************************************************** */
static void EM_v_SetEgoSpeedX(uint32 u_emTimestamp) {
    /* get output pointer */
    VEDVehDyn_t *p_egoDynTgtSync = EnvmData.pEgoDynTgtSync;
    VEDVehDyn_t *p_egoDynObjSync = EnvmData.pEgoDynObjSync;

    p_egoDynTgtSync->Longitudinal.MotVar.Velocity =
        EnvmData.pEgoDynRaw->Longitudinal.MotVar.Velocity;
    p_egoDynTgtSync->Longitudinal.MotVar.varVelocity =
        EnvmData.pEgoDynRaw->Longitudinal.MotVar.varVelocity;
    p_egoDynObjSync->Longitudinal.MotVar.Velocity =
        EnvmData.pEgoDynRaw->Longitudinal.MotVar.Velocity;
    p_egoDynObjSync->Longitudinal.MotVar.varVelocity =
        EnvmData.pEgoDynRaw->Longitudinal.MotVar.varVelocity;

    /* moving direction is already regarded in 'VDY delay buffer' */
    // p_egoDynTgtSync->Longitudinal.MotVar.Velocity =
    //     EM_f_GetDelayedVdySignal(u_emTimestamp, ODDELAY_f_speedXTgtLatency,
    //                              &a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
    //                                   .Longitudinal.VeloCorr.corrVelo,
    //                              EM_DELAY_INTER_LINEAR);
    // p_egoDynTgtSync->Longitudinal.MotVar.varVelocity =
    //     EM_f_GetDelayedVdySignal(u_emTimestamp, ODDELAY_f_speedXTgtLatency,
    //                              &a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
    //                                   .Longitudinal.VeloCorr.corrVeloVar,
    //                              EM_DELAY_INTER_LINEAR);

    /* in case of having same latencies for 'targets' and 'objects' use the
     * already computed values */
    // if (TUE_CML_IsZero(ODDELAY_f_speedXTgtLatency -
    //                    ODDELAY_f_speedXObjLatency)) {
    //     p_egoDynObjSync->Longitudinal.MotVar.Velocity =
    //         p_egoDynTgtSync->Longitudinal.MotVar.Velocity;
    //     p_egoDynObjSync->Longitudinal.MotVar.varVelocity =
    //         p_egoDynTgtSync->Longitudinal.MotVar.varVelocity;
    // } else {
    //     p_egoDynObjSync->Longitudinal.MotVar.Velocity =
    //         EM_f_GetDelayedVdySignal(u_emTimestamp,
    //         ODDELAY_f_speedXObjLatency,
    //                                  &a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
    //                                       .Longitudinal.VeloCorr.corrVelo,
    //                                  EM_DELAY_INTER_LINEAR);
    //     p_egoDynObjSync->Longitudinal.MotVar.varVelocity =
    //         EM_f_GetDelayedVdySignal(u_emTimestamp,
    //         ODDELAY_f_speedXObjLatency,
    //                                  &a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
    //                                       .Longitudinal.VeloCorr.corrVeloVar,
    //                                  EM_DELAY_INTER_LINEAR);
    // }
}

/* ****************************************************************************

  Functionname:     EM_v_SetEgoAccelX                                       */ /*!

   @brief            Set function for fEMEgoAccelX

   @description      Module global variable information will be copied to caller
                     The ego data is delayed according to the target and object
                     latencies.
                     Checks if external latency for acceleration shall be used.

   @param[in]        u_emTimestamp : EM timestamp the VDY signal data should be
 interpolated to.

   @return           void

   @pre              None
   @post             No changes


 **************************************************************************** */
static void EM_v_SetEgoAccelX(uint32 u_emTimestamp) {
    /* get output pointer */
    VEDVehDyn_t *p_egoDynTgtSync = EnvmData.pEgoDynTgtSync;
    VEDVehDyn_t *p_egoDynObjSync = EnvmData.pEgoDynObjSync;
    // f32_t f_accelXObjLatency = ODDELAY_f_accelXObjLatency;

    p_egoDynTgtSync->Longitudinal.MotVar.Accel =
        EnvmData.pEgoDynRaw->Longitudinal.MotVar.Accel;
    p_egoDynTgtSync->Longitudinal.MotVar.varAccel =
        EnvmData.pEgoDynRaw->Longitudinal.MotVar.varAccel;
    p_egoDynObjSync->Longitudinal.MotVar.Accel =
        EnvmData.pEgoDynRaw->Longitudinal.MotVar.Accel;
    p_egoDynObjSync->Longitudinal.MotVar.varAccel =
        EnvmData.pEgoDynRaw->Longitudinal.MotVar.varAccel;

    /* store old value */
    // EnvmData.pPrivGlob->fAcellXLast =
    //     p_egoDynTgtSync->Longitudinal.MotVar.Accel;

    // /* moving direction is already regarded in 'VDY delay buffer' */
    // p_egoDynTgtSync->Longitudinal.MotVar.Accel = EM_f_GetDelayedVdySignal(
    //     u_emTimestamp, ODDELAY_f_accelXTgtLatency,
    //     &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Longitudinal.MotVar.Accel,
    //     EM_DELAY_INTER_LINEAR);
    // p_egoDynTgtSync->Longitudinal.MotVar.varAccel = EM_f_GetDelayedVdySignal(
    //     u_emTimestamp, ODDELAY_f_accelXTgtLatency,
    //     &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Longitudinal.MotVar.varAccel,
    //     EM_DELAY_INTER_LINEAR);

    // /* in case of having same latencies for 'targets' and 'objects' use the
    //  * already computed values */
    // if (TUE_CML_IsZero(ODDELAY_f_accelXTgtLatency - f_accelXObjLatency)) {
    //     p_egoDynObjSync->Longitudinal.MotVar.Accel =
    //         p_egoDynTgtSync->Longitudinal.MotVar.Accel;
    //     p_egoDynObjSync->Longitudinal.MotVar.varAccel =
    //         p_egoDynTgtSync->Longitudinal.MotVar.varAccel;
    // } else {
    //     p_egoDynObjSync->Longitudinal.MotVar.Accel =
    //     EM_f_GetDelayedVdySignal(
    //         u_emTimestamp, f_accelXObjLatency,
    //         &a_emEgoVdyBuff[u_newestSamplePosVdyBuff].Longitudinal.MotVar.Accel,
    //         EM_DELAY_INTER_LINEAR);
    //     p_egoDynObjSync->Longitudinal.MotVar.varAccel =
    //         EM_f_GetDelayedVdySignal(u_emTimestamp, f_accelXObjLatency,
    //                                  &a_emEgoVdyBuff[u_newestSamplePosVdyBuff]
    //                                       .Longitudinal.MotVar.varAccel,
    //                                  EM_DELAY_INTER_LINEAR);
    // }

    // /*replacing the corrAccel and CorrAccelVar with motvar accel and its
    //  * variance, because of removal of correction code in
    //  em_speedcorrection.c
    //  * in order to optimize RAM/ROM*/
    // p_egoDynObjSync->Longitudinal.AccelCorr.corrAccel =
    //     p_egoDynObjSync->Longitudinal.MotVar.Accel;
    // p_egoDynObjSync->Longitudinal.AccelCorr.corrAccelVar =
    //     p_egoDynObjSync->Longitudinal.MotVar.varAccel;
}

#if (CFG_Envm_EGO_YAWRATE_COMPENSATION == CFG_Envm_SWITCH_ON)
/* ****************************************************************************

  Functionname:     EM_v_SetYawrateCompSignals                            */ /*!

     @brief            Determines the sensor offset to the rotation center based
   on
                       VehPar/sensor mounting signals and the float angle of the
   ego
                       vehicle.

     @description      Determines the sensor offset to the rotation center based
   on
                       VehPar/sensor mounting signals and the float angle of the
   ego
                       vehicle.

     @return           void

     @pre              None
     @post             No changes


   ****************************************************************************
   */
static void EM_v_SetYawrateCompSignals(void) {
    float32 f_Tmp, f_FloatAng;
    /* set sensor offset to the rotation center (back axle) */
    /* TODO: once sensor parameters are set correctly in all projects the
     * defaults can be removed */
    if (TUE_CML_IsNonZero(EnvmData.pGlobEgoStatic->VehParMain.WheelBase) &&
        TUE_CML_IsNonZero(EnvmData.pGlobEgoStatic->VehParMain.AxisLoadDistr)) {
        f_Tmp = EnvmData.pGlobEgoStatic->VehParMain.WheelBase *
                (1.f - EnvmData.pGlobEgoStatic->VehParMain.AxisLoadDistr);
    } else {
        f_Tmp = EM_ROT_DEFAULT_DIST2AXLE;
    }
    if (TUE_CML_IsNonZero(
            EnvmData.pGlobEgoStatic->SensorMounting.LongPosToCoG)) {
        f_GlobalLongPosToRot =
            f_Tmp + EnvmData.pGlobEgoStatic->SensorMounting.LongPosToCoG;
    } else if (TUE_CML_IsNonZero(
                   EnvmData.pGlobEgoStatic->SensorMounting.LongPos)) {
        f_GlobalLongPosToRot =
            (2.f * f_Tmp) + EnvmData.pGlobEgoStatic->SensorMounting.LongPos;
    } else  // set default
    {
        f_GlobalLongPosToRot = f_Tmp + EM_ROT_DEFAULT_LONGPOS2COG;
    }

    /* set ego vehicle float angle */
    f_FloatAng = EnvmData.pEgoDynObjSync->Lateral.Curve.Curve *
                 EnvmData.pGlobEgoStatic->VehParMain.WheelBase;
    if (fABS(f_FloatAng) < 1.f) {
        f_GlobalObjSyncCosFloatAng = COS_(f_FloatAng);
    } else {
        f_GlobalObjSyncCosFloatAng = 1.f;
    }
    f_FloatAng = EnvmData.pEgoDynTgtSync->Lateral.Curve.Curve *
                 EnvmData.pGlobEgoStatic->VehParMain.WheelBase;
    if (fABS(f_FloatAng) < 1.f) {
        f_GlobalCluSyncCosFloatAng = COS_(f_FloatAng);
    } else {
        f_GlobalCluSyncCosFloatAng = 1.f;
    }
}
#endif

/* ****************************************************************************

  Functionname:     EnvmInputInit                                          */ /*!

    @brief            Initialization of em_input local variables

    @description      Inits 'VDY delay line' buffers, sensor offset to rotation
                      center and ego vehicle float angle and resets local
  counters

    @return           void

    @pre              None
    @post             No changes


  ****************************************************************************
  */
void EnvmInputInit(void) {}

/* ****************************************************************************

  Functionname:     EnvmProcessInput                                       */ /*!

    @brief            Process EM Input

    @description      Check basic input data for processing (stop EM if input is
  not
                      sufficient for "running") and derive internal component
  operation
                      modes based on EM global operation mode. Update 'VDY delay
  data'
                      with current information. First entry in 'VDY delay
  buffer' is
                      the most current one. Handle VDY input data from 20ms task
  =>
                      update delayed VDY data. Set sensor offset to rotation
  center based
                      on vehicle parameters and ego vehicle float angle. Sets
  object list
                      header (validity and timestamp) based on RSP input data.

    @return           void

    @pre              None
    @post             No changes


  ****************************************************************************
  */
void EnvmProcessInput(void) {
    /*--- VARIABLES ---*/
    // const VEDVehDyn_t *p_vehDyn = GET_EGO_RAW_DATA_PTR;
    const VEDVehDyn_t *p_vehDyn = EnvmData.pEgoDynRaw;
    uint32 u_emTimestamp = 0UL;

    /**
     * get current RSP timestamp used as 'EM time'.
     * currently, this RSP time is also used for timestamping
     * 'VDY samples'.
     */

    u_emTimestamp = EM_f_GetRSPTimeStamp(SYS_SCAN_NEAR);

    /* Check basic input data for processing (stop EM if input is not sufficient
       for "running") and
       derive internal component operation modes based on EM global operation
       mode */
    // levi 2018-12-07 removed  //EnvmProcessStates();

    /**
     * update 'VDY delay data' with current information.
     * first entry in 'VDY delay buffer' is the most current one.
     */
    if (p_vehDyn->sSigHeader.eSigStatus == AL_SIG_STATE_OK) {
        (void)EM_b_UpdateVdyDelayData(p_vehDyn, u_emTimestamp);

        /**
         * if the INIT step is left - this function is called then - signal
         * that the 'VDY delay buffer' is modified and can thus be reset (when
         * calling 'InitDelayBuff'). allow also 'Reset' in case of
         * Envm_MOD_INVALID_INPUT_SIGNALS.
         */
        if (((EnvmData.pFrame->eEnvmOpMode) == Envm_MOD_RUNNING) ||
            ((EnvmData.pFrame->eEnvmOpMode) ==
             Envm_MOD_INVALID_INPUT_SIGNALS)) {
            b_resetDelayBuff = TRUE;
        }
    }

    /* handle VDY input data from 20ms task => update delayed VDY data */
    EM_v_InterpolateVdyDelayData(u_emTimestamp);

#if (CFG_Envm_EGO_YAWRATE_COMPENSATION == CFG_Envm_SWITCH_ON)
    /* set sensor offset to rotation center based on vehicle parameters and ego
     * vehicle float angle */
    EM_v_SetYawrateCompSignals();
#endif

    /* Set object list header (validity and timestamp) based on RSP input data
     */
    EM_v_ProcessObjectListHeader();

    /* call checkpoint for frame software task flow monitoring */
    /* do not remove this function call, do not move it to another place! */
}

/* ****************************************************************************

  Functionname:     EM_v_InterpolateVdyDelayData */ /*!

                              @brief            Interpolate delayed VDY input
                            data

                              @description      Interpolate delayed input Ego
                            data with regard to the current 'EM time'.
                                                but interpolation is only done
                            in case of having at least one valid
                                                element in 'DelayLine' and valid
                            inputs signals are available.
                                                Sets long velocity latency
                            values, long accel latency values,
                                                the Vehicle Course Curvature
                            latency values, Driver Intended Curvature
                                                latency values, the Vehicle
                            Yawrate latency values and the Slip
                                                angle latency values

                              @return           void

                              @pre              None
                              @post             No changes


                            ****************************************************************************
                            */
static void EM_v_InterpolateVdyDelayData(uint32 u_emTimestamp) {
    /**
     * interpolate delayed input Ego data with regard to the current 'EM time'.
     * but interpolation is only done in case of having at least one valid
     * element in 'DelayLine' and valid inputs signals are available.
     */
    if ((u_numVdyBuffElem > 0UL) &&
        ((EnvmData.pFrame->eEnvmOpMode) != Envm_MOD_INVALID_INPUT_SIGNALS)) {
        /* Set long velocity latency values */
        EM_v_SetEgoSpeedX(u_emTimestamp);

        /* Set long accel latency values */
        EM_v_SetEgoAccelX(u_emTimestamp);

        /* Set the Vehicle Course Curvature latency values */
        EM_v_SetEgoCurve(u_emTimestamp);

        /* Set the Driver Intended Curvature latency values */
        EM_v_SetEgoDrvIntCurve(u_emTimestamp);

        /* Set the Vehicle Yawrate latency values. */
        EM_v_SetEgoYawRate(u_emTimestamp);

        /* Set the Slip angle latency values. */
        EM_v_SetEgoSlipAngle(u_emTimestamp);
    }
}

/*****************************************************************************
  OUTPUT FUNCTION
*****************************************************************************/

/* ****************************************************************************

  Functionname:     EM_v_ProcessObjectListHeader                              */ /*!

 @brief            process the validity of internal object list status

 @description      Based on the RSP data input the validity of the EM object
list is derived.
                   Checks if near scan and/or far scan are valid. If the invalid
RSP counter
                   is above the threshold set the valid state of the object list
to invalid.
                   Stores measurement counter of near scan to be used in object
list signal header.
                   set object list status depending of EM operation mode. add
the actual em cycle
                   counter to the public obj list. Also sets signal header of
internal object list.


 @return           void

 @pre              None
 @post             No changes


**************************************************************************** */
static void EM_v_ProcessObjectListHeader(void) {}

/* ****************************************************************************

  Functionname:     Envm_v_SetSignalHeader                                */ /*!

     @brief            Fill signal header for EM output interfaces

     @description      Fill signal header for EM output interfaces

     @param[in]        p_SigHeader : Pointer to signal header structure

     @pre              None
     @post             No changes

     @return           void


   ****************************************************************************
   */
void Envm_v_SetSignalHeader(ENVMSignalHeader_t *p_SigHeader) {
    /* copy signal header from internal object list */
    *p_SigHeader = EnvmData.pPrivObjList->sSigHeader;
}

/* ****************************************************************************

  Functionname:     EnvmProcessOutput                                      */ /*!

    @brief            Copy Output data from EM to RTE

    @description      Copies tunnel probability, calls internal custom object
  list
                      processing, Calls generic object output processing and
  reports
                      'process out' checkpoint. Also transfers internal to
  external
                      object list.

    @return           void

    @pre              None
    @post             No changes


  ****************************************************************************
  */
void EnvmProcessOutput(void) {
    /* Copy tunnel probability */
    EM_v_CopyTunnelProbability();

    /* call internal custom object list processing */
    // levi 2018-12-06 removed  //  EM_v_ProcessCustomOutput();

#if (CFG_Envm_GENERIC_OBJECT_INTERFACE_ACTIVE == 1)
    /* call generic object output processing (also transfers internal to
     * external custom object list) */
    EMProcessObjOutput();
#endif

    /* call checkpoint for frame software task flow monitoring */
    /* do not remove this function call, do not move it to another place! */
}

/* ****************************************************************************

  Functionname:     EM_v_CopyTunnelProbability                              */ /*!

   @brief            Copy tunnel probability from EO to GLOB

   @description      Copy tunnel probability from EO to GLOB

   @return           void

   @pre              None
   @post             No changes


 **************************************************************************** */
static void EM_v_CopyTunnelProbability(void) {}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */