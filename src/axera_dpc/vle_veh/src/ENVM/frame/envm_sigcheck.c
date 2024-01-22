/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "TM_Global_Types.h"
#include "stddef.h"
#include "assert.h"

/*****************************************************************************
  DEFINES
*****************************************************************************/

#ifndef EGO_CURVE_STATE
#define EGO_CURVE_STATE      \
    VED_GET_IO_STATE(        \
        VED_SOUT_POS_CURVE,  \
        GET_EGO_RAW_DATA_PTR \
            ->State) /*! To be removed when defined in algo_glob.h*/
#endif

#ifndef DEF_FLOAT_ZERO
#define DEF_FLOAT_ZERO (0.0f) /*! Float zero definition to avoid QAC warning*/
#endif
#define LOOP_NOT_ENTERED (0x0)
#define LOOP_ENTERED_X_POSITION_CoG (0x1)
#define LOOP_ENTERED_X_POSITION (0x2)
#define LOOP_ENTERED_BOTH (0x3)

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static EMSigCheckMeasFreezeMem_t
    EMSignalStatusFreeze; /*! buffer for freeze of EM input signal status */
static SignalHistory_t
    historyCorrVeloOld, /*! history of corrected velocity signal */
    historyYawRateOld;  /*! history of yaw rate signal */
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  LOCAL FUNCTIONS
*****************************************************************************/

static void v_CheckVehicleDynData(
    EMSigCheckMeasFreezeMem_t *const p_sigCheckMem,
    EMSigCheckStatusInfo_t *p_Prev_Status);
static void v_CheckVehicleStatData(
    EMSigCheckMeasFreezeMem_t *const p_sigCheckMem,
    EMSigCheckStatusInfo_t *p_Prev_Status);
static void v_CheckRSPCluListNear(
    EMSigCheckMeasFreezeMem_t *const p_sigCheckMem,
    EMSigCheckStatusInfo_t *p_Prev_Status);

static void v_CheckCtrl(EMSigCheckMeasFreezeMem_t *const p_sigCheckMem,
                        EMSigCheckStatusInfo_t *p_Prev_Status);
static void v_CheckCamObjInput(EMSigCheckMeasFreezeMem_t *const p_sigCheckMem,
                               EMSigCheckStatusInfo_t *p_Prev_Status);

/*****************************************************************************
  LOCAL INLINES
*****************************************************************************/

ALGO_INLINE void v_SetError(EMSigCheckStatusData_t *p_outSignal);
ALGO_INLINE void v_SetWarning(EMSigCheckStatusData_t *p_outSignal);

ALGO_INLINE void v_CheckRange(EMSigCheckStatusInfo_t *p_Prev_Status,
                              float32 f_input,
                              EMSigCheckStatusData_t *p_output,
                              float32 f_minValue,
                              float32 f_maxValue,
                              EMSigCheckStatusInfo_t Status_Out);
ALGO_INLINE void v_CheckValidity(EMSigCheckStatusInfo_t *p_Prev_Status,
                                 float32 f_input,
                                 EMSigCheckStatusData_t *p_output,
                                 float32 f_ref,
                                 EMSigCheckStatusInfo_t Status_Out);
ALGO_INLINE void v_CheckVariance(EMSigCheckStatusInfo_t *p_Prev_Status,
                                 float32 f_inValue,
                                 float32 f_inVariance,
                                 EMSigCheckStatusData_t *p_output,
                                 float32 f_factor,
                                 float32 f_base,
                                 EMSigCheckStatusInfo_t Status_Out);
ALGO_INLINE void v_CheckDerivation(EMSigCheckStatusInfo_t *p_Prev_Status,
                                   const SignalHistory_t *p_inNew,
                                   const SignalHistory_t *p_inOld,
                                   EMSigCheckStatusData_t *p_output,
                                   float32 f_minValue,
                                   float32 f_maxValue,
                                   EMSigCheckStatusInfo_t Status_Out);

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    v_EMSigCheckInit */
void v_EMSigCheckInit(void) {
    /* Initialize SigCheck buffer to 0, so every counter starts at 0 and each
       signal status is Envm_SIGCHECK_STATUS_OK.
    */
    (void)memset((void *)&EMSignalStatusFreeze, 0,
                 sizeof(EMSigCheckMeasFreezeMem_t));

    /* Initialize header information. */
    EMSignalStatusFreeze.u_version = EM_SIGCHECK_VERSION;
    EMSignalStatusFreeze.u_cycleID = 0U;

    /* Initialize history variables. */
    (void)memset((void *)&historyCorrVeloOld, 0, sizeof(SignalHistory_t));
    (void)memset((void *)&historyYawRateOld, 0, sizeof(SignalHistory_t));
} /* v_EMSigCheckInit() */

/*************************************************************************************************************************
  Functionname:    u_EMSigCheck */
EMSigCheckStatusInfo_t u_EMSigCheck(void) {
    EMSigCheckStatusInfo_t u_SigCheckInfo = EM_SIGCHECK_VALID;

    /* Set header data. */
    EMSignalStatusFreeze.u_version = EM_SIGCHECK_VERSION;
    EMSignalStatusFreeze.u_cycleID = EnvmData.pFrame->uiCycleCounter;

    /* Validate EM input signals. */
    v_CheckVehicleDynData(&EMSignalStatusFreeze, &u_SigCheckInfo);
    v_CheckVehicleStatData(&EMSignalStatusFreeze, &u_SigCheckInfo);
    v_CheckRSPCluListNear(&EMSignalStatusFreeze, &u_SigCheckInfo);

    v_CheckCtrl(&EMSignalStatusFreeze, &u_SigCheckInfo);
    v_CheckCamObjInput(&EMSignalStatusFreeze, &u_SigCheckInfo);

    return u_SigCheckInfo;
} /* EMSigCheck() */

/* ****************************************************************************

  Functionname:     b_CheckVehicleDynData                                  */ /*!

    @brief            Validate VehicleDynData

    @description      Check, if vehicle dynamic data are valid.
                      Clears history of velocity correlation and Yaw rate.
                      Checks range of longitudinal velocity correction,
  corrected longitudinal
                      velocity, longitudinal acceleration, lateral curve,
                      and lateral Yaw rate signals.
                      Checks Variance of corrected longitudinal velocity,
  longitudinal
                      acceleration, Lateral Yaw rate signals.
                      Checks Deviation of corrected longitudinal velocity and
  yaw rate if
                      the valid signal history is available.
                      The above checks are make only if the signal status is
  Valid, else
                      signal check error is set.
                      Corresponding error is set if any of the above checks
  fail.
                      Updates previous status with that of current one.

    @param[in]        p_sigCheckMem : is the pointer to freeze buffer
    @param[in]        p_Prev_Status : returns the max of previous- and the
  current
                      status(valid/warning/error) as a pointer

    @return           void

    @pre              None
    @post             No changes


  ****************************************************************************
  */
static void v_CheckVehicleDynData(
    EMSigCheckMeasFreezeMem_t *const p_sigCheckMem,
    EMSigCheckStatusInfo_t *p_Prev_Status) {
    EMSigCheckStatusInfo_t u_SigCheckInfo = EM_SIGCHECK_VALID;
    SignalHistory_t historyCorrVeloNew, historyYawRateNew;

    (void)memset(&historyCorrVeloNew, 0, sizeof(historyCorrVeloNew));
    (void)memset(&historyYawRateNew, 0, sizeof(historyYawRateNew));

    /* Check each Signal as desired in case its status is "VALID". */

    /* pVehicleDynData->Longitudinal.VeloCorr.corrFact */
    p_sigCheckMem->VehicleDynData_Longitudinal_VeloCorr_corrFact
        .u_currentStatus = Envm_SIGCHECK_STATUS_OK;
    if (IS_SIGNAL_STATUS_OK(EGO_SPEED_X_CORRECTED_STATE)) {
        p_sigCheckMem->VehicleDynData_Long_Vel_Corrected_Status
            .u_currentStatus = Envm_SIGCHECK_STATUS_OK;
        v_CheckRange(
            &u_SigCheckInfo,
            GET_EGO_RAW_DATA_PTR->Longitudinal.VeloCorr.corrFact,
            &p_sigCheckMem->VehicleDynData_Longitudinal_VeloCorr_corrFact,
            EM_SIGCHECK_VELOCORR_CORRFACT_MIN,
            Envm_SIGCHECK_VELOCORR_CORRFACT_MAX, EM_SIGCHECK_WARNING);
    } else {
        p_sigCheckMem->VehicleDynData_Long_Vel_Corrected_Status
            .u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetError(&p_sigCheckMem->VehicleDynData_Long_Vel_Corrected_Status);
        u_SigCheckInfo = EM_SIGCHECK_ERROR;
    }

    /* pVehicleDynData->Longitudinal.VeloCorr.corrVelo */
    p_sigCheckMem->VehicleDynData_Longitudinal_VeloCorr_corrVelo
        .u_currentStatus = Envm_SIGCHECK_STATUS_OK;
    if (IS_SIGNAL_STATUS_OK(EGO_SPEED_X_STATE)) {
        p_sigCheckMem->VehicleDynData_Long_Vel_Status.u_currentStatus =
            Envm_SIGCHECK_STATUS_OK;
        v_CheckRange(
            &u_SigCheckInfo, EGO_SPEED_X_CORRECTED,
            &p_sigCheckMem->VehicleDynData_Longitudinal_VeloCorr_corrVelo,
            EM_SIGCHECK_VELOCORR_CORRVELO_MIN,
            EM_SIGCHECK_VELOCORR_CORRVELO_MAX, EM_SIGCHECK_WARNING);

        v_CheckVariance(
            &u_SigCheckInfo, EGO_SPEED_X_CORRECTED, EGO_SPEED_X_CORRECTED_VAR,
            &p_sigCheckMem->VehicleDynData_Longitudinal_VeloCorr_corrVelo,
            EM_SIGCHECK_STD_DEV_MAX_FACTOR,
            EM_SIGCHECK_STD_DEV_MAX_BASE_EGO_SPEED_X_CORRECTED,
            EM_SIGCHECK_WARNING);

        /* Check deviation only if we have a valid signal history. */
        if (TRUE == historyCorrVeloOld.b_valid) {
            historyCorrVeloNew.f_value = EGO_SPEED_X_CORRECTED;
            historyCorrVeloNew.u_TimeStamp =
                GET_EGO_RAW_DATA_PTR->sSigHeader.uiTimeStamp;

            v_CheckDerivation(
                &u_SigCheckInfo, &historyCorrVeloNew, &historyCorrVeloOld,
                &p_sigCheckMem->VehicleDynData_Longitudinal_VeloCorr_corrVelo,
                EM_SIGCHECK_VELOCORR_CORRVELO_DERIVATION_MIN,
                EM_SIGCHECK_VELOCORR_CORRVELO_DERIVATION_MAX,
                EM_SIGCHECK_WARNING);

            historyCorrVeloOld.f_value = historyCorrVeloNew.f_value;
            historyCorrVeloOld.u_TimeStamp = historyCorrVeloNew.u_TimeStamp;
            historyCorrVeloOld.b_valid = TRUE;
        }
    } else {
        p_sigCheckMem->VehicleDynData_Long_Vel_Status.u_currentStatus |=
            EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetError(&p_sigCheckMem->VehicleDynData_Long_Vel_Status);
        u_SigCheckInfo = EM_SIGCHECK_ERROR;
    }

    /* pVehicleDynData->Longitudinal.MotVar.Accel */
    p_sigCheckMem->VehicleDynData_Longitudinal_MotVar_Accel.u_currentStatus =
        Envm_SIGCHECK_STATUS_OK;
    if (IS_SIGNAL_STATUS_OK(EGO_ACCEL_X_STATE)) {
        p_sigCheckMem->VehicleDynData_Long_Accel_Status.u_currentStatus =
            Envm_SIGCHECK_STATUS_OK;
        v_CheckRange(&u_SigCheckInfo, EGO_ACCEL_X_RAW,
                     &p_sigCheckMem->VehicleDynData_Longitudinal_MotVar_Accel,
                     EM_SIGCHECK_MOTVAR_ACCEL_MIN, EM_SIGCHECK_MOTVAR_ACCEL_MAX,
                     EM_SIGCHECK_WARNING);

        v_CheckVariance(
            &u_SigCheckInfo, EGO_ACCEL_X_RAW, EGO_ACCEL_X_VAR_RAW,
            &p_sigCheckMem->VehicleDynData_Longitudinal_MotVar_Accel,
            EM_SIGCHECK_STD_DEV_MAX_FACTOR,
            Envm_SIGCHECK_STD_DEV_MAX_BASE_EGO_ACCEL_X_RAW,
            EM_SIGCHECK_WARNING);
    } else {
        p_sigCheckMem->VehicleDynData_Long_Accel_Status.u_currentStatus |=
            EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetError(&p_sigCheckMem->VehicleDynData_Long_Accel_Status);
        u_SigCheckInfo = EM_SIGCHECK_ERROR;
    }

    /* pVehicleDynData->Lateral.Curve.Curve */
    p_sigCheckMem->VehicleDynData_Lateral_Curve_Curve.u_currentStatus =
        Envm_SIGCHECK_STATUS_OK;
    if (IS_SIGNAL_STATUS_OK(EGO_CURVE_STATE)) {
        p_sigCheckMem->VehicleDynData_Lat_Curve_Status.u_currentStatus =
            Envm_SIGCHECK_STATUS_OK;
        v_CheckRange(&u_SigCheckInfo, EGO_CURVE_RAW,
                     &p_sigCheckMem->VehicleDynData_Lateral_Curve_Curve,
                     Envm_SIGCHECK_CURVE_MIN, Envm_SIGCHECK_CURVE_MAX,
                     EM_SIGCHECK_WARNING);
    } else {
        p_sigCheckMem->VehicleDynData_Lat_Curve_Status.u_currentStatus |=
            EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetError(&p_sigCheckMem->VehicleDynData_Lat_Curve_Status);
        u_SigCheckInfo = EM_SIGCHECK_ERROR;
    }

    /* pVehicleDynData->Lateral.YawRate.YawRate */
    p_sigCheckMem->VehicleDynData_Lateral_YawRate_YawRate.u_currentStatus =
        Envm_SIGCHECK_STATUS_OK;
    if (IS_SIGNAL_STATUS_OK(EGO_YAW_RATE_STATE)) {
        p_sigCheckMem->VehicleDynData_Lat_YawRate_Status.u_currentStatus =
            Envm_SIGCHECK_STATUS_OK;
        v_CheckRange(&u_SigCheckInfo, EGO_YAW_RATE_RAW,
                     &p_sigCheckMem->VehicleDynData_Lateral_YawRate_YawRate,
                     EM_SIGCHECK_YAWRATE_MIN, EM_SIGCHECK_YAWRATE_MAX,
                     EM_SIGCHECK_WARNING);

        v_CheckVariance(&u_SigCheckInfo, EGO_YAW_RATE_RAW, EGO_YAW_RATE_VAR_RAW,
                        &p_sigCheckMem->VehicleDynData_Lateral_YawRate_YawRate,
                        EM_SIGCHECK_STD_DEV_MAX_FACTOR,
                        EM_SIGCHECK_STD_DEV_MAX_BASE_EGO_YAW_RATE_RAW,
                        EM_SIGCHECK_WARNING);

        /* Check deviation only if we have a valid signal history. */
        if (TRUE == historyYawRateOld.b_valid) {
            historyYawRateNew.f_value = EGO_YAW_RATE_RAW;
            historyYawRateNew.u_TimeStamp =
                GET_EGO_RAW_DATA_PTR->sSigHeader.uiTimeStamp;

            v_CheckDerivation(
                &u_SigCheckInfo, &historyYawRateNew, &historyYawRateOld,
                &p_sigCheckMem->VehicleDynData_Lateral_YawRate_YawRate,
                EM_SIGCHECK_YAWRATE_DERIVATION_MIN,
                Envm_SIGCHECK_YAWRATE_DERIVATION_MAX, EM_SIGCHECK_WARNING);

            historyYawRateOld.f_value = historyYawRateNew.f_value;
            historyYawRateOld.u_TimeStamp = historyYawRateNew.u_TimeStamp;
            historyYawRateOld.b_valid = TRUE;
        }
    } else {
        p_sigCheckMem->VehicleDynData_Lat_YawRate_Status.u_currentStatus |=
            EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetError(&p_sigCheckMem->VehicleDynData_Lat_YawRate_Status);
        u_SigCheckInfo = EM_SIGCHECK_ERROR;
    }

    /*  pVehicleDynData->MotionState.MotState*/
    p_sigCheckMem->VehicleDynData_MotionState_Status.u_currentStatus =
        Envm_SIGCHECK_STATUS_OK;
    if (!IS_SIGNAL_STATUS_OK(EGO_MOTION_STATE_STATE)) {
        p_sigCheckMem->VehicleDynData_MotionState_Status.u_currentStatus |=
            EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetError(&p_sigCheckMem->VehicleDynData_MotionState_Status);
        u_SigCheckInfo = EM_SIGCHECK_ERROR;
    }

    *p_Prev_Status = MAX(*p_Prev_Status, u_SigCheckInfo);
} /* v_CheckVehicleDynData() */

/* ****************************************************************************

  Functionname:     v_CheckVehicleStatData                                  */ /*!

   @brief            Validate VehicleStatData

   @description      Check, if vehicle static data are valid.
                     Check each Signal as desired in case its status is "VALID".
                     Checks range of vertical position of Sensor mounting.
                     Checks validity of Sensor mounting longitudianl position to
 centre
                     of gravity, longitudinal sensor position from centre of
 front wheel axle,
                     vehicle wheel base, vehicle track width front, vehicle axle
 load distribution and
                     sets warning accordingly. Updates previous status.

   @param[in]        p_sigCheckMem  : is the pointer to freeze buffer
   @param[in]        p_Prev_Status  : returns the max of previous- and the
 current
                     status(valid/warning/error) as a pointer

   @return           void

   @pre              None
   @post             No changes


 **************************************************************************** */
static void v_CheckVehicleStatData(
    EMSigCheckMeasFreezeMem_t *const p_sigCheckMem,
    EMSigCheckStatusInfo_t *p_Prev_Status) {
    EMSigCheckStatusInfo_t u_CheckPosXCOG_StatusOK = EM_SIGCHECK_VALID,
                           u_CheckPosX_StatusOK = EM_SIGCHECK_VALID,
                           u_SigCheckInfo = EM_SIGCHECK_VALID;
    ubit8_t u_loopCheck = LOOP_NOT_ENTERED;

    /* Check each Signal as desired in case its status is "VALID". */

    /* pVehicleStatData->SensorMounting.VertPos */
    p_sigCheckMem->VehicleStatData_SensorMounting_VertPos.u_currentStatus =
        Envm_SIGCHECK_STATUS_OK;
    if (IS_SIGNAL_STATUS_OK(VED_GET_IO_STATE(VEH_PAR_SEN_MOUNT_VERT_POS,
                                             SENSOR_MOUNTING.State))) {
        p_sigCheckMem->VehicleStatData_SensorMounting_VertPos_Status
            .u_currentStatus = Envm_SIGCHECK_STATUS_OK;
        v_CheckRange(&u_SigCheckInfo, SENSOR_Z_POSITION,
                     &p_sigCheckMem->VehicleStatData_SensorMounting_VertPos,
                     Envm_SIGCHECK_VERTPOS_MIN, Envm_SIGCHECK_VERTPOS_MAX,
                     EM_SIGCHECK_ERROR);
    } else {
        p_sigCheckMem->VehicleStatData_SensorMounting_VertPos_Status
            .u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetWarning(
            &p_sigCheckMem->VehicleStatData_SensorMounting_VertPos_Status);
        u_SigCheckInfo = EM_SIGCHECK_WARNING;
    }

    // ! Assumption that the sensor is always in front of front vehicle axle and
    // this is not valid for SRR (especially Rear ones)
    p_sigCheckMem->VehicleStatData_SensorMounting_LongPosToCoG.u_currentStatus =
        Envm_SIGCHECK_STATUS_OK;
    if (IS_SIGNAL_STATUS_OK(VED_GET_IO_STATE(VEH_PAR_SEN_MOUNT_LONGPOS_TO_COG,
                                             SENSOR_MOUNTING.State))) {
        p_sigCheckMem->VehicleStatData_SensorMounting_LongPosToCoG_Status
            .u_currentStatus = Envm_SIGCHECK_STATUS_OK;
        v_CheckValidity(
            &u_CheckPosXCOG_StatusOK, SENSOR_X_POSITION_CoG,
            &p_sigCheckMem->VehicleStatData_SensorMounting_LongPosToCoG,
            EM_SIGCHECK_INVALID_SENSOR_X_POSITION_CoG, EM_SIGCHECK_WARNING);
        u_loopCheck = LOOP_ENTERED_X_POSITION_CoG;
    }

    p_sigCheckMem->VehicleStatData_SensorMounting_LongPos.u_currentStatus =
        Envm_SIGCHECK_STATUS_OK;
    if (IS_SIGNAL_STATUS_OK(VED_GET_IO_STATE(VEH_PAR_SEN_MOUNT_LONG_POS,
                                             SENSOR_MOUNTING.State))) {
        p_sigCheckMem->VehicleStatData_SensorMounting_LongPos_Status
            .u_currentStatus = Envm_SIGCHECK_STATUS_OK;
        v_CheckValidity(&u_CheckPosX_StatusOK, SENSOR_X_POSITION,
                        &p_sigCheckMem->VehicleStatData_SensorMounting_LongPos,
                        EM_SIGCHECK_INVALID_SENSOR_X_POSITION,
                        EM_SIGCHECK_WARNING);
        u_loopCheck = u_loopCheck | LOOP_ENTERED_X_POSITION;
    }

    if (u_loopCheck == LOOP_ENTERED_BOTH) {
        if ((u_CheckPosXCOG_StatusOK != EM_SIGCHECK_VALID) &&
            (u_CheckPosX_StatusOK != EM_SIGCHECK_VALID)) {
            u_SigCheckInfo = EM_SIGCHECK_ERROR;
            v_SetError(
                &p_sigCheckMem->VehicleStatData_SensorMounting_LongPosToCoG);
            v_SetError(&p_sigCheckMem->VehicleStatData_SensorMounting_LongPos);
        } else if ((u_CheckPosXCOG_StatusOK != EM_SIGCHECK_VALID) ||
                   (u_CheckPosX_StatusOK != EM_SIGCHECK_VALID)) {
            u_SigCheckInfo = MAX(u_SigCheckInfo, EM_SIGCHECK_WARNING);
        } else {
        }
    } else if (u_loopCheck == LOOP_ENTERED_X_POSITION_CoG) {
        p_sigCheckMem->VehicleStatData_SensorMounting_LongPos_Status
            .u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetWarning(
            &p_sigCheckMem->VehicleStatData_SensorMounting_LongPos_Status);
        if (u_CheckPosXCOG_StatusOK != EM_SIGCHECK_VALID) {
            u_SigCheckInfo = EM_SIGCHECK_ERROR;
        }
    } else if (u_loopCheck == LOOP_ENTERED_X_POSITION) {
        p_sigCheckMem->VehicleStatData_SensorMounting_LongPosToCoG_Status
            .u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetWarning(
            &p_sigCheckMem->VehicleStatData_SensorMounting_LongPosToCoG_Status);
        if (u_CheckPosX_StatusOK != EM_SIGCHECK_VALID) {
            u_SigCheckInfo = EM_SIGCHECK_ERROR;
        }
    } else {
        /*if (u_loopCheck == LOOP_NOT_ENTERED)*/
        p_sigCheckMem->VehicleStatData_SensorMounting_LongPos_Status
            .u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_INVALID;
        p_sigCheckMem->VehicleStatData_SensorMounting_LongPosToCoG_Status
            .u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetWarning(
            &p_sigCheckMem->VehicleStatData_SensorMounting_LongPos_Status);
        v_SetWarning(
            &p_sigCheckMem->VehicleStatData_SensorMounting_LongPosToCoG_Status);
        u_SigCheckInfo = MAX(u_SigCheckInfo, EM_SIGCHECK_WARNING);
    }

    /*! pVehicleStatData->VehParMain.WheelBase */
    p_sigCheckMem->VehicleStatData_VehicleWheelBase.u_currentStatus =
        Envm_SIGCHECK_STATUS_OK;
    if (IS_SIGNAL_STATUS_OK(VED_GET_IO_STATE(
            VED_PAR_POS_WBASE, GET_EGO_STATIC_DATA_PTR->VehParMain.State))) {
        p_sigCheckMem->VehicleStatData_VehicleWheelBase_Status.u_currentStatus =
            Envm_SIGCHECK_STATUS_OK;
        v_CheckValidity(&u_SigCheckInfo, EGO_VEHICLE_WHEEL_BASE,
                        &p_sigCheckMem->VehicleStatData_VehicleWheelBase,
                        EM_SIGCHECK_INVALID_WHEEL_BASE, EM_SIGCHECK_WARNING);
    } else {
        p_sigCheckMem->VehicleStatData_VehicleWheelBase_Status
            .u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetWarning(&p_sigCheckMem->VehicleStatData_VehicleWheelBase_Status);
        u_SigCheckInfo = MAX(u_SigCheckInfo, EM_SIGCHECK_WARNING);
    }

    /*! pVehicleStatData->VehParMain.TrackWidthFront */
    p_sigCheckMem->VehicleStatData_VehicleTrackWidthFront.u_currentStatus =
        Envm_SIGCHECK_STATUS_OK;
    if (IS_SIGNAL_STATUS_OK(VED_GET_IO_STATE(
            VED_PAR_POS_TWDFR, GET_EGO_STATIC_DATA_PTR->VehParMain.State))) {
        p_sigCheckMem->VehicleStatData_VehicleTrackWidthFront_Status
            .u_currentStatus = Envm_SIGCHECK_STATUS_OK;
        v_CheckValidity(&u_SigCheckInfo, EGO_VEHICLE_TRACK_WIDTH_FRONT,
                        &p_sigCheckMem->VehicleStatData_VehicleTrackWidthFront,
                        Envm_SIGCHECK_INVALID_TRACK_WIDTH_FRONT,
                        EM_SIGCHECK_WARNING);
    } else {
        p_sigCheckMem->VehicleStatData_VehicleTrackWidthFront_Status
            .u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetWarning(
            &p_sigCheckMem->VehicleStatData_VehicleTrackWidthFront_Status);
        u_SigCheckInfo = MAX(u_SigCheckInfo, EM_SIGCHECK_WARNING);
    }

    /*! pVehicleStatData->VehParMain.AxisLoadDistr */
    p_sigCheckMem->VehicleStatData_VehicleAxleLoadDistribution.u_currentStatus =
        Envm_SIGCHECK_STATUS_OK;
    if (IS_SIGNAL_STATUS_OK(VED_GET_IO_STATE(
            VED_PAR_POS_AXLD, GET_EGO_STATIC_DATA_PTR->VehParMain.State))) {
        p_sigCheckMem->VehicleStatData_VehicleAxleLoadDistribution_status
            .u_currentStatus = Envm_SIGCHECK_STATUS_OK;
        v_CheckValidity(
            &u_SigCheckInfo, EGO_VEHICLE_AXLE_LOAD_DISTR,
            &p_sigCheckMem->VehicleStatData_VehicleAxleLoadDistribution,
            Envm_SIGCHECK_INVALID_AXLE_LOAD_DISTR, EM_SIGCHECK_WARNING);
    } else {
        p_sigCheckMem->VehicleStatData_VehicleAxleLoadDistribution_status
            .u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_INVALID;
        v_SetWarning(
            &p_sigCheckMem->VehicleStatData_VehicleAxleLoadDistribution_status);
        u_SigCheckInfo = MAX(u_SigCheckInfo, EM_SIGCHECK_WARNING);
    }

    *p_Prev_Status = MAX(*p_Prev_Status, u_SigCheckInfo);
} /* v_CheckVehicleStatData() */

/* ****************************************************************************

  Functionname:     v_CheckRSPCluListNear                                  */ /*!

    @brief            Validate RSPCluListNear

    @description      Check, if cluster list data of near scan are valid.
                      Check each Signal as desired in case its status is
  "VALID".
                      Checks Range gate length of RSP Cluster list near.
                      Updates previous status based on current.

    @param[in]        p_sigCheckMem : is the pointer to freeze buffer
    @param[in]        p_Prev_Status : returns the max of previous- and the
  current
                      status(valid/warning/error) as a pointer

    @return           void

    @pre              None
    @post             No changes


  ****************************************************************************
  */
static void v_CheckRSPCluListNear(
    EMSigCheckMeasFreezeMem_t *const p_sigCheckMem,
    EMSigCheckStatusInfo_t *p_Prev_Status) {
    EMSigCheckStatusInfo_t u_SigCheckInfo = EM_SIGCHECK_VALID;

    /* Check each Signal as desired in case its status is "VALID". */

    /* p_RSPCluListNear->ClustListHead.f_RangegateLength */
    p_sigCheckMem->RSPCluListNear_ClustListHead_f_RangegateLength
        .u_currentStatus = Envm_SIGCHECK_STATUS_OK;

    *p_Prev_Status = MAX(*p_Prev_Status, u_SigCheckInfo);
} /* v_CheckRSPCluListNear() */

/* ****************************************************************************

  Functionname:     v_CheckALNAzimuthCorrection */ /*!

                               @brief            Validate ALNAzimuthCorrection

                               @description      Check, if azimuth correction
                             data are valid.
                                                 Check each Signal as desired in
                             case its status is "VALID".
                                                 Checks Range of Azimuth
                             misalignment correction values for
                                                 far and near scan.
                                                 Updates previous status based
                             on current.

                               @param[in]        p_sigCheckMem : is the pointer
                             to freeze buffer
                               @param[in]        p_Prev_Status : returns the max
                             of previous- and the current
                                                 status(valid/warning/error) as
                             a pointer

                               @return           void

                               @pre              None
                               @post             No changes


                             ****************************************************************************
                             */
static void v_CheckALNAzimuthCorrection(
    EMSigCheckMeasFreezeMem_t *const p_sigCheckMem,
    EMSigCheckStatusInfo_t *p_Prev_Status) {

} /* v_CheckALNAzimuthCorrection() */

/* ****************************************************************************

  Functionname:     v_CheckCtrl                                  */ /*!

              @brief            Validate Ctrl

              @description      Check, if ctrl data are valid.

              @param[in]        p_sigCheckMem : is the pointer to freeze buffer
              @param[in]        p_Prev_Status : returns the max of previous- and
            the current
                                status(valid/warning/error) as a pointer

              @return           void

              @pre              None
              @post             No changes


            ****************************************************************************
            */
static void v_CheckCtrl(EMSigCheckMeasFreezeMem_t *const p_sigCheckMem,
                        EMSigCheckStatusInfo_t *p_Prev_Status) {
    EMSigCheckStatusInfo_t u_SigCheckInfo = EM_SIGCHECK_VALID;

    /* Check each Signal as desired in case its status is "VALID". */
    _PARAM_UNUSED(p_sigCheckMem);

    *p_Prev_Status = MAX(*p_Prev_Status, u_SigCheckInfo);
} /* v_CheckCtrl() */

/* ****************************************************************************

  Functionname:     v_CheckCamObjInput                                  */ /*!

       @brief            Validate CamObjInput

       @description      Check, if camera object data are valid.

       @param[in]        p_sigCheckMem : is the pointer to freeze buffer
       @param[in]        p_Prev_Status : returns the max of previous- and the
     current
                         status(valid/warning/error) as a pointer

       @return           void

       @pre              None
       @post             No changes


     ****************************************************************************
     */
static void v_CheckCamObjInput(EMSigCheckMeasFreezeMem_t *const p_sigCheckMem,
                               EMSigCheckStatusInfo_t *p_Prev_Status) {
    EMSigCheckStatusInfo_t u_SigCheckInfo = EM_SIGCHECK_VALID;

    /* Check each Signal as desired in case its status is "VALID". */
    _PARAM_UNUSED(p_sigCheckMem);

    *p_Prev_Status = MAX(*p_Prev_Status, u_SigCheckInfo);
} /* v_CheckCamObjInput() */

/* ****************************************************************************

  Functionname:     v_SetError                                  */ /*!

               @brief            set a error in signal status

               @description      This function sets the error flag in the status
             descriptor
                                 of a signal and increments its error count.

               @param[out]       p_outSignal : pointer to signal status
             descriptor

               @return           void

               @pre              None
               @post             No changes


             ****************************************************************************
             */
ALGO_INLINE void v_SetError(EMSigCheckStatusData_t *p_outSignal) {
    p_outSignal->u_currentStatus |= Envm_SIGCHECK_STATUS_FLAG_ERROR;
    if (p_outSignal->u_errorCnt < UInt8_UpperLimit) {
        p_outSignal->u_errorCnt++;
    }
} /* v_SetError() */

/* ****************************************************************************

  Functionname:     v_SetWarning                                  */ /*!

             @brief            set a warning in signal status

             @description      This function sets the warning flag in the status
           descriptor
                               of a signal and increments its warning count.

             @param[out]       p_outSignal : pointer to signal status descriptor

             @return           void

             @pre              None
             @post             No changes


           ****************************************************************************
           */
ALGO_INLINE void v_SetWarning(EMSigCheckStatusData_t *p_outSignal) {
    p_outSignal->u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_WARNING;
    if (p_outSignal->u_warningCnt < UInt8_UpperLimit) {
        p_outSignal->u_warningCnt++;
    }
} /* v_SetWarning() */

/*************************************************************************************************************************
Functionname:    v_CheckRange */
ALGO_INLINE void v_CheckRange(EMSigCheckStatusInfo_t *p_Prev_Status,
                              float32 f_input,
                              EMSigCheckStatusData_t *p_output,
                              float32 f_minValue,
                              float32 f_maxValue,
                              EMSigCheckStatusInfo_t Status_Out) {
    EMSigCheckStatusInfo_t u_SigCheckResult = EM_SIGCHECK_VALID;

    /* If signal is out of range, set corresponding status flags and set return
     * value "invalid". */
    if ((f_input < f_minValue) || (f_input > f_maxValue)) {
        p_output->u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_RANGE;

        if (Status_Out == EM_SIGCHECK_WARNING) {
            v_SetWarning(p_output);
        } else if (Status_Out == EM_SIGCHECK_ERROR) {
            v_SetError(p_output);
        } else {
        }
        u_SigCheckResult = Status_Out;
    }

    *p_Prev_Status = MAX(*p_Prev_Status, u_SigCheckResult);
} /* v_CheckRange() */

/*************************************************************************************************************************
Functionname:    v_CheckValidity */
ALGO_INLINE void v_CheckValidity(EMSigCheckStatusInfo_t *p_Prev_Status,
                                 float32 f_input,
                                 EMSigCheckStatusData_t *p_output,
                                 float32 f_ref,
                                 EMSigCheckStatusInfo_t Status_Out) {
    EMSigCheckStatusInfo_t u_SigCheckResult = EM_SIGCHECK_VALID;

    if (TUE_CML_IsZero(f_input -
                       f_ref))  // check if the f_input is equal to f_ref
    {
        p_output->u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_INVALID;

        if (Status_Out == EM_SIGCHECK_WARNING) {
            v_SetWarning(p_output);
        } else if (Status_Out == EM_SIGCHECK_ERROR) {
            v_SetError(p_output);
        } else {
        }
        u_SigCheckResult = Status_Out;
    }

    *p_Prev_Status = MAX(*p_Prev_Status, u_SigCheckResult);
} /* v_CheckVlidity() */

/*************************************************************************************************************************

Functionname:    v_CheckVariance */
ALGO_INLINE void v_CheckVariance(EMSigCheckStatusInfo_t *p_Prev_Status,
                                 float32 f_inValue,
                                 float32 f_inVariance,
                                 EMSigCheckStatusData_t *p_output,
                                 float32 f_factor,
                                 float32 f_base,
                                 EMSigCheckStatusInfo_t Status_Out) {
    EMSigCheckStatusInfo_t u_SigCheckResult = EM_SIGCHECK_VALID;
    float32 f_threshold = DEF_FLOAT_ZERO;

    /* Calculate threshold for standard deviation and square it for variance. */
    f_threshold = TUE_CML_MultAdd(f_factor, f_inValue, f_base);
    f_threshold = SQR(f_threshold);

    /* If Variance is too high, set corresponding flag and put out warning. */
    if (f_inVariance >= f_threshold) {
        p_output->u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_VARIANCE;

        if (Status_Out == EM_SIGCHECK_WARNING) {
            v_SetWarning(p_output);
        } else if (Status_Out == EM_SIGCHECK_ERROR) {
            v_SetError(p_output);
        } else {
        }
        u_SigCheckResult = Status_Out;
    }

    *p_Prev_Status = MAX(*p_Prev_Status, u_SigCheckResult);
} /* v_CheckVariance() */

/*************************************************************************************************************************
  Functionname:    v_CheckDerivation */
ALGO_INLINE void v_CheckDerivation(EMSigCheckStatusInfo_t *p_Prev_Status,
                                   const SignalHistory_t *p_inNew,
                                   const SignalHistory_t *p_inOld,
                                   EMSigCheckStatusData_t *p_output,
                                   float32 f_minValue,
                                   float32 f_maxValue,
                                   EMSigCheckStatusInfo_t Status_Out) {
    EMSigCheckStatusInfo_t u_SigCheckResult = EM_SIGCHECK_VALID;
    float32 f_deltaX, f_deltaT, f_diffQuot;

    /* Calculate differences. */
    f_deltaX = p_inOld->f_value - p_inNew->f_value;
    f_deltaT = (float32)(p_inOld->u_TimeStamp - p_inNew->u_TimeStamp);

    /* To avoid division by 0... */
    if (TUE_CML_IsNonZero(f_deltaT)) {
        /* Calculate differential quotient. */
        f_diffQuot = f_deltaX / f_deltaT;

        /* If differential quotient is out of range, set corresponding status
           flags and set return value "invalid".
        */
        if ((f_diffQuot < f_minValue) || (f_diffQuot > f_maxValue)) {
            p_output->u_currentStatus |= EM_SIGCHECK_STATUS_FLAG_DERIVATION;
            if (Status_Out == EM_SIGCHECK_WARNING) {
                v_SetWarning(p_output);
            } else if (Status_Out == EM_SIGCHECK_ERROR) {
                v_SetError(p_output);
            } else {
            }
            u_SigCheckResult = Status_Out;
        }
    }

    *p_Prev_Status = MAX(*p_Prev_Status, u_SigCheckResult);
} /* v_CheckDerivation() */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */