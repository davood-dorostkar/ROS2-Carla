/** \addtogroup tueFusion
 *  @{
 * \file       tue_prv_fusion_tools.h
 * \brief      defines a bunch of methods that are used all throughout fusion on
 * different occasions
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_FUSION_TOOLS_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_FUSION_TOOLS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "tue_prv_common_types.h"

#define ObjFusn_START_SEC_CODE

/** Count number of bits that are set in i */
uint32 getNumberOfSetBits(uint32 i);
#define ObjFusn_STOP_SEC_CODE

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_FUSION_TOOLS_H_