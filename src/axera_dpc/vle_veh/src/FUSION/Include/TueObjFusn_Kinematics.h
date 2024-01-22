/*
 * \file        TueObjFusn_Kinematics.h
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUEOBJFUSN_KINEMATICS_H
#define TUEOBJFUSN_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
/* contains egoMotion structure */
#include "TueObjFusn_EgoMotionType.h"

/* contains trackable structure */
#include "TueObjFusn_ObjectListType.h"

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/
/**
 * @brief converts from relative coordinate system to over ground coordinate
 *system
 * converts all objects within the given object list from relative coordinates
 * to relative coordinates over ground centered at the middle of the rear axle.
 * Both input coordinate system and output coordinate system's x-axis are
 *aligned to
 * the vehicle x-axis. The longitudinal displacement specifies the displacement
 *of
 * the rear axle to the relative coordinate system.
 * Stores the results back into the list.
 *
 * @param stEgoMotion denotes the ego motion information that was valid when the
 *list
 * contents were originally measured from the sensor. Note that the correctness
 * of the latencies is not checked in this method.
 *
 * @param stTrkblList denotes the trackable list that is to be converted. It is
 *both used
 * as an input and an ouput data structure (in-place computation).
 *
 * @param f32deltaX denotes how far the relative coordinate system is "in front
 *of" the
 * over-ground coordinate system. For instance in order to go from front bumper
 * system to rear axle coordinate system use approximately +4.2m.
 *
 * @return TRUE (ok) or FALSE (on failure the corresponding error code is
 *returned and stTrkblList may
 * possibly be left invalid)
 **/
#define ObjFusn_START_SEC_CODE

uint32 Kinematics_Relative2OverGround(
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        stEgoMotion,
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjList,
    const float32 f32deltaX);
#define ObjFusn_STOP_SEC_CODE

/**
 * @brief converts from over ground coordinate system to relative coordinate
 *system
 * converts all objects within the given object list from relative coordinates
 *over
 * ground centered at the middle of the rear axle to relative coordinates
 * Both input coordinate system and output coordinate system's x-axis are
 *aligned to
 * the vehicle x-axis. The longitudinal displacement specifies the displacement
 *of
 * the rear axle to the relative coordinate system.
 * Stores the results back into the list.
 *
 * @param stEgoMotion denotes the ego motion information that was valid when the
 *list
 * contents were originally measured from the sensor. Note that the correctness
 * of the latencies is not checked in this method.
 *
 * @param stTrkblList denotes the trackable list that is to be converted. It is
 *both used
 * as an input and an ouput data structure (in-place computation).
 *
 * @param f32deltaX denotes how far the relative coordinate system is "in front"
 *of the
 * over-ground coordinate system. For instance in order to go from rear axle
 * system to front bumper coordinate system use approximately +4.2m.
 *
 * @return TRUE (ok) or FALSE (on failure the corresponding error code is
 *returned and stTrkblList may
 * possibly be left invalid)
 **/
#define ObjFusn_START_SEC_CODE

uint32 Kinematics_OverGround2Relative(
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion,
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjList,
    const float32 f32deltaX);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // TUEOBJFUSN_KINEMATICS_H
/*==================[end of file]===========================================*/
