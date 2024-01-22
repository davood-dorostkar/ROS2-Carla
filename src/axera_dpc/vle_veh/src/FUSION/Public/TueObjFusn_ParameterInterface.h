/** \addtogroup tueFusion
 *  @{
 * \file        TUEOBJFUSN_ParameterInterface.h
 *
 * \brief       way for setting fusion parameters from host application
 *
 * Currently all parameters need to be set at startup time, i.e. after
 * Fusion_init has been called and before the start of the first cycle
 * in the near future additional parameters might get introduced that
 * may be changed during runtime.
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_PUBLIC_TUEOBJFUSN_PARAMETERINTERFACE_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_PUBLIC_TUEOBJFUSN_PARAMETERINTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "tue_common_libs.h"
// #include "TM_Global_Types.h"

//#include "Fusion_Std_Types.h"
#include "Std_Types.h"
#include "FusionCompiler.h"
#include "TueObjFusn_Params.h"

/*==================[setter
 * functions]================================================*/
#define ObjFusn_START_SEC_SLOW_CODE

uint32 Fusion_set_u32SensorMode(const uint32 u32Value);

uint32 Fusion_set_f32MatchGate(const float32 f32Value);

uint32 Fusion_set_f32PedestrianVarianceInXForQ(const float32 f32Value);

uint32 Fusion_set_f32PedestrianVarianceInYForQ(const float32 f32Value);

uint32 Fusion_set_f32VehicleVarianceInXForQ(const float32 f32Value);

uint32 Fusion_set_f32VehicleVarianceInYForQ(const float32 f32Value);

uint32 Fusion_set_bOutputIsOverground(const boolean bValue);

uint32 Fusion_set_bUseTrackMerge(const boolean bValue);

uint32 Fusion_set_bUseCoasting(const boolean bValue);

/*==================[getter
 * functions]================================================*/
extern uint32 Fusion_get_u32SensorMode(void);
extern float32 Fusion_get_f32MatchGate(void);
extern float32 Fusion_get_f32PedestrianVarianceInXForQ(void);
extern float32 Fusion_get_f32PedestrianVarianceInYForQ(void);
extern float32 Fusion_get_f32VehicleVarianceInXForQ(void);
extern float32 Fusion_get_f32VehicleVarianceInYForQ(void);
extern boolean Fusion_get_bOutputIsOverground(void);
extern boolean Fusion_get_bUseTrackMerge(void);
extern boolean Fusion_get_bUseCoasting(void);

/** this method sets all fusion parameters to their default values */
void Fusion_set_DefaultParameters(void);

/** this method is only exported for integration testing. It allows to read the
 * internal parameter structure. More precisely it copies the contents of the
 * internal parameter structure to the given location. */

void Fusion_get_allParameters(CONSTP2VAR(TueObjFusn_ParametersType,
                                         AUTOMATIC,
                                         ObjFusn_VAR_NOINIT) _sParameters);

#define ObjFusn_STOP_SEC_SLOW_CODE

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_PUBLIC_TUEOBJFUSN_PARAMETERINTERFACE_H_
