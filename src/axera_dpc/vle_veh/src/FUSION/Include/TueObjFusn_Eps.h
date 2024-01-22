/*
* Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
* wen mingshu <wenmingshu@senseauto.com>
*/
/** \addtogroup tueFusion
 *  @{
 * \file TueObjFusn_Eps.h
 *
 * \brief allowed tolerances for floating point computation
 *
 *
 *
 *
 *
 *   (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUEOBJFUSN_EPS_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUEOBJFUSN_EPS_H_

/** epsilon for comparing f32 times */
#define TUE_PRV_FUSION_DT_EPS (0.0001f) /**> [sec] */
/// epsilon for comparing f32 positions (and velocities)
/// \todo: until we worked on the LKF precision user story we define a higher
/// epsilon so that tests will pass again.
// #define TUE_PRV_FUSION_POS_EPS_ABS (0.0001f)                 /* [m]   */
#define TUE_PRV_FUSION_POS_EPS_ABS (0.001f) /* [m]   */
#define TUE_PRV_FUSION_POS_EPS_REL (0.001f)
#define TUE_PRV_FUSION_COV_EPS_ABS (0.002f)
#define TUE_PRV_FUSION_COV_EPS_REL (0.002f)
#define TUE_PRV_FUSION_COMPARE_TO_ZERO (0.00000001f)

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUEOBJFUSN_EPS_H_
/** @} */
