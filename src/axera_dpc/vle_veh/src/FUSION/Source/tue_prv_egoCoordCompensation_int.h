/** \addtogroup tueEgoCoordComp
 *  @{
 * \file        tue_prv_egoCoordCompensation_int.h
 *
 * \brief internal header of the ego motion compensation AAU
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_EGOCOORDCOMPENSATION_INT_H
#define TUE_PRV_EGOCOORDCOMPENSATION_INT_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "tue_prv_common_matrix.h"

/*==================[macros]================================================*/

#define TUE_PRV_EGO_COORD_COMPENSATION_MIN_STATES_COMPENSATE_VEL (4u)
#define TUE_PRV_EGO_COORD_COMPENSATION_MIN_STATES_COMPENSATE_ACCX (5u)
#define TUE_PRV_EGO_COORD_COMPENSATION_MIN_STATES_COMPENSATE_ACCY (6u)
/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

/************** Internal header******************/
/** @name Internal functions */

/**
 * @fn   LOCAL void EgoCoordCompensation_CoordRotation(f32_t * const pf32X,
 * f32_t * const pf32Y, const f32_t f32Phi);
 *
 * @brief   EgoCoordCompensation_CoordRotation rotates the X and Y position of
 * phi.
 *
 * @param [in,out] pf32X  f32_t * const, x to be rotated
 * @param [in,out] pf32Y  f32_t * const, y to be rotated
 * @param   f32Phi        const f32_t, rotation angle
 */
#define ObjFusn_START_SEC_CODE

LOCAL void EgoCoordCompensation_CoordRotation(
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32X,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Y,
    const float32 f32Sin,
    const float32 f32Cos);

/**
 * @fn   LOCAL void EgoCoordCompensation_CoordTranslation(f32_t * const pf32X,
 * f32_t * const pf32Y, const f32_t f32DX, const f32_t f32DY);
 *
 * @brief   EgoCoordCompensation_CoordTranslation shifts the X and Y position by
 * dx and dy.
 *
 * @param [in,out] pf32X  f32_t * const, x to be shifted
 * @param [in,out] pf32Y  f32_t * const, y to be shifted
 * @param   f32DX         const f32_t, shifting in x
 * @param   f32DY         const f32_t, shifting in y
 */
LOCAL void EgoCoordCompensation_CoordTranslation(
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32X,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Y,
    const float32 f32DX,
    const float32 f32DY);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // TUE_PRV_EGOCOORDCOMPENSATION_INT_H
/*==================[end of file]===========================================*/

/** @} */
/** @} */
