/** \addtogroup tueFusion
 *  \{
 *
 * \file TueObjFusn_ParameterInterface.c
 *
 * \brief This is the parameter Interface for Fusion
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 */

#include "TueObjFusn_ParameterInterface.h"
#include "TueObjFusn_Params.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"

#include "TueObjFusn_TrackableConstants.h"

#include "tue_prv_common_types.h"
#include "tue_prv_error_management.h"

/* Reason for waiving QAC warnings 3410:
 * This warning flags macro parameters which are not in parentheses.
 * In our case, we are using macro parameters as function or parameter names,
 * therefore we cannot put them into parentheses.
 */

/** \name Parameter Interface  global variables
 \{**/

/** memory object for ALL current fusion parameters */
//#define ObjFusn_START_SEC_VAR_UNSPECIFIED
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
LOCAL VAR(TueObjFusn_ParametersType,
          ObjFusn_VAR_ZERO_INIT) sCurrentParams = {0};
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
//#define ObjFusn_STOP_SEC_VAR_UNSPECIFIED

/** \}*/

/** \name Parameter Interface  functions
 \{**/

#define ObjFusn_START_SEC_SLOW_CODE

/* PRQA S 1532 2 */ /* External Parameter Interface */
void Fusion_set_DefaultParameters(void) {
    sCurrentParams.u32SensorMode = TUEOBJFUSN_PARAMS_U32SENSORMODE_DEFAULT;
    sCurrentParams.f32MatchGate = TUEOBJFUSN_PARAMS_F32MATCHGATE_DEFAULT;
    sCurrentParams.f32PedestrianVarianceInXForQ =
        TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINXFORQ_DEFAULT;
    sCurrentParams.f32PedestrianVarianceInYForQ =
        TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINYFORQ_DEFAULT;
    sCurrentParams.f32VehicleVarianceInXForQ =
        TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINXFORQ_DEFAULT;
    sCurrentParams.f32VehicleVarianceInYForQ =
        TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINYFORQ_DEFAULT;
    sCurrentParams.bOutputIsOverground =
        TUEOBJFUSN_PARAMS_BOUTPUTISOVERGROUND_DEFAULT;
    sCurrentParams.bUseTrackMerge = TUEOBJFUSN_PARAMS_BUSETRACKMERGE_DEFAULT;
    sCurrentParams.bUseCoasting = TUEOBJFUSN_PARAMS_BUSECOASTING_DEFAULT;
}
/** \}*/

/*==================[setter
 * functions]================================================*/

/* PRQA S 1503 2 */ /* External Parameter Interface */
uint32 Fusion_set_u32SensorMode(const uint32 u32Value) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    /* PRQA S 3316 2 */ // Parameter can be configured to be greater than zero
    if (
#if (TUEOBJFUSN_PARAMS_U32SENSORMODE_MIN > 0)
        (u32Value < TUEOBJFUSN_PARAMS_U32SENSORMODE_MIN) ||
#endif
        (u32Value > TUEOBJFUSN_PARAMS_U32SENSORMODE_MAX)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_PARAMETER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_PARAMETER,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE_SET_U32SENSORMODE);
    } else
#endif
    {
        sCurrentParams.u32SensorMode = u32Value;
    }

    return u32Success;
}

/* PRQA S 1503 2 */ /* External Parameter Interface */
uint32 Fusion_set_f32MatchGate(const float32 f32Value) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if ((f32Value < TUEOBJFUSN_PARAMS_F32MATCHGATE_MIN) ||
        (f32Value > TUEOBJFUSN_PARAMS_F32MATCHGATE_MAX)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_PARAMETER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_PARAMETER,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE_SET_F32MATCHGATE);
    } else
#endif
    {
        sCurrentParams.f32MatchGate = f32Value;
    }

    return u32Success;
}

/* PRQA S 1503 2 */ /* External Parameter Interface */
uint32 Fusion_set_f32PedestrianVarianceInXForQ(const float32 f32Value) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if ((f32Value < TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINXFORQ_MIN) ||
        (f32Value > TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINXFORQ_MAX)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_PARAMETER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_PARAMETER,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE_SET_F32PEDESTRIANVARIANCEINXFORQ);
    } else
#endif
    {
        sCurrentParams.f32PedestrianVarianceInXForQ = f32Value;
    }

    return u32Success;
}

/* PRQA S 1503 2 */ /* External Parameter Interface */
uint32 Fusion_set_f32PedestrianVarianceInYForQ(const float32 f32Value) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    /* PRQA S 3316 2 */ // Parameter can be configured to be greater than zero
    if ((f32Value < TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINYFORQ_MIN) ||
        (f32Value > TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINYFORQ_MAX)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_PARAMETER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_PARAMETER,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE_SET_F32VEHICLEVARIANCEINYFORQ);
    } else
#endif
    {
        sCurrentParams.f32PedestrianVarianceInYForQ = f32Value;
    }

    return u32Success;
}

/* PRQA S 1503 2 */ /* External Parameter Interface */
uint32 Fusion_set_f32VehicleVarianceInXForQ(const float32 f32Value) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if ((f32Value < TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINXFORQ_MIN) ||
        (f32Value > TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINXFORQ_MAX)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_PARAMETER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_PARAMETER,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE_SET_F32VEHICLEVARIANCEINXFORQ);
    } else
#endif
    {
        sCurrentParams.f32VehicleVarianceInXForQ = f32Value;
    }

    return u32Success;
}

/* PRQA S 1503 2 */ /* External Parameter Interface */
uint32 Fusion_set_f32VehicleVarianceInYForQ(const float32 f32Value) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    /* PRQA S 3316 2 */ // Parameter can be configured to be greater than zero
    if ((f32Value < TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINYFORQ_MIN) ||
        (f32Value > TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINYFORQ_MAX)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_PARAMETER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_PARAMETER,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE_SET_F32VEHICLEVARIANCEINYFORQ);
    } else
#endif
    {
        sCurrentParams.f32VehicleVarianceInYForQ = f32Value;
    }

    return u32Success;
}

/* PRQA S 1503 2 */ /* External Parameter Interface */
uint32 Fusion_set_bOutputIsOverground(const boolean bValue) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    /* PRQA S 3316 2 */ // Parameter can be configured to be greater than zero
    if (
#if (TUEOBJFUSN_PARAMS_BOUTPUTISOVERGROUND_MIN > 0)
        (bValue < TUEOBJFUSN_PARAMS_BOUTPUTISOVERGROUND_MIN) ||
#endif
        (bValue > TUEOBJFUSN_PARAMS_BOUTPUTISOVERGROUND_MAX)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_PARAMETER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_PARAMETER,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE_SET_BOUTPUTISOVERGROUND);
    } else
#endif
    {
        sCurrentParams.bOutputIsOverground = bValue;
    }

    return u32Success;
}

/* PRQA S 1503 2 */ /* External Parameter Interface */
uint32 Fusion_set_bUseTrackMerge(const boolean bValue) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    /* PRQA S 3316 2 */ // Parameter can be configured to be greater than zero
    if (
#if (TUEOBJFUSN_PARAMS_BUSETRACKMERGE_MIN > 0)
        (bValue < TUEOBJFUSN_PARAMS_BUSETRACKMERGE_MIN) ||
#endif
        (bValue > TUEOBJFUSN_PARAMS_BUSETRACKMERGE_MAX)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_PARAMETER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_PARAMETER,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE_SET_BUSETRACKMERGE);
    } else
#endif
    {
        sCurrentParams.bUseTrackMerge = bValue;
    }

    return u32Success;
}

/* PRQA S 1503 2 */ /* External Parameter Interface */
uint32 Fusion_set_bUseCoasting(const boolean bValue) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    /* PRQA S 3316 2 */ // Parameter can be configured to be greater than zero
    if (
#if (TUEOBJFUSN_PARAMS_BUSECOASTING_MIN > 0)
        (bValue < TUEOBJFUSN_PARAMS_BUSECOASTING_MIN) ||
#endif
        (bValue > TUEOBJFUSN_PARAMS_BUSECOASTING_MAX)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_PARAMETER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_PARAMETER,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE,
            TUEOBJFUSN_AAU_PARAMETER_INTERFACE_SET_BUSECOASTING);
    } else
#endif
    {
        sCurrentParams.bUseCoasting = bValue;
    }

    return u32Success;
}

/*==================[getter
 * functions]================================================*/

/* PRQA S 1532 2 */ /* External Parameter Interface */
uint32 Fusion_get_u32SensorMode(void) { return sCurrentParams.u32SensorMode; }

/* PRQA S 1532 2 */ /* External Parameter Interface */
float32 Fusion_get_f32MatchGate(void) { return sCurrentParams.f32MatchGate; }

/* PRQA S 1532 2 */ /* External Parameter Interface */
float32 Fusion_get_f32PedestrianVarianceInXForQ(void) {
    return sCurrentParams.f32PedestrianVarianceInXForQ;
}

/* PRQA S 1532 2 */ /* External Parameter Interface */
float32 Fusion_get_f32PedestrianVarianceInYForQ(void) {
    return sCurrentParams.f32PedestrianVarianceInYForQ;
}

/* PRQA S 1532 2 */ /* External Parameter Interface */
float32 Fusion_get_f32VehicleVarianceInXForQ(void) {
    return sCurrentParams.f32VehicleVarianceInXForQ;
}

/* PRQA S 1532 2 */ /* External Parameter Interface */
float32 Fusion_get_f32VehicleVarianceInYForQ(void) {
    return sCurrentParams.f32VehicleVarianceInYForQ;
}

/* PRQA S 1532 2 */ /* External Parameter Interface */
boolean Fusion_get_bOutputIsOverground(void) {
    return sCurrentParams.bOutputIsOverground;
}

/* PRQA S 1532 2 */ /* External Parameter Interface */
boolean Fusion_get_bUseTrackMerge(void) {
    return sCurrentParams.bUseTrackMerge;
}

/* PRQA S 1532 2 */ /* External Parameter Interface */
boolean Fusion_get_bUseCoasting(void) { return sCurrentParams.bUseCoasting; }

/* PRQA S 1503 2 */ /* Required for Integration test */
void Fusion_get_allParameters(CONSTP2VAR(TueObjFusn_ParametersType,
                                         AUTOMATIC,
                                         ObjFusn_VAR_NOINIT) _sParameters) {
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == _sParameters) {
    } else
#endif
    {
        _sParameters->bOutputIsOverground = sCurrentParams.bOutputIsOverground;
        _sParameters->bUseCoasting = sCurrentParams.bUseCoasting;
        _sParameters->bUseTrackMerge = sCurrentParams.bUseTrackMerge;
        _sParameters->f32MatchGate = sCurrentParams.f32MatchGate;
        _sParameters->f32PedestrianVarianceInXForQ =
            sCurrentParams.f32PedestrianVarianceInXForQ;
        _sParameters->f32PedestrianVarianceInYForQ =
            sCurrentParams.f32PedestrianVarianceInYForQ;
        _sParameters->f32VehicleVarianceInXForQ =
            sCurrentParams.f32VehicleVarianceInXForQ;
        _sParameters->f32VehicleVarianceInYForQ =
            sCurrentParams.f32VehicleVarianceInYForQ;
        _sParameters->u32SensorMode = sCurrentParams.u32SensorMode;
    }
}
#define ObjFusn_STOP_SEC_SLOW_CODE

/** /} */
