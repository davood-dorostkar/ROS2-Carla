/** \defgroup tuePrvidProvider TUE Fusion ID Provider
 * \brief       provide unique IDs for objects rsp tracks.
 *
 * This code was formerly part of tue_prv_idManagement.h and has now been moved
 * to a
 * seperate file.
 * Fusion Id provider provides IDs for tracks/objects in the range 0...60000
 * (rsp. MAX_FUSION_ID). IDs are unique, i.e. an ID is not output again until
 * it has been released.
 *
 * Typical use case:
 * Requirement: an ID may not be re-used for at least one cycle
 * Solution: (example: ID=1)
 * cylce 1: object 1 exists and is output
 * cycle 2: object 1 shall die.
 *          First, new objects are created => ID 1 still assigned to old object,
 * not used
 *          Then, object 1 is killed, ID 1 is released
 *          => output list does not contain ID 1
 * cycle 3: ID 1 may be used again
 *
 * This order (first get new IDs, then release old) is enforced with a state
 * machine.
 *
 * useage:
 * - call "idProvider_init" before any cycle operation starts.
 * - at the beginning of a fusion cycle call idProvider_startCycle
 * - call idProvider_getNewFusionId to get new fusion IDs
 * - AFTER all new objects have new IDs:
 *   call idProvider_releaseFusionId to release IDs of tracks which have died
 * - you can NOT call idProvider_getNewFusionId now
 * - at the end of a fusion cycle call idProvider_finalizeCycle
 *
 * \file        tue_prv_idProvider.h
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_IDPROVIDER_H
#define TUE_PRV_IDPROVIDER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tue_prv_common_types.h"

/**
 * @brief initialize ID Provider
 * initialize variables. call only once before running the first fusion cycle.
 */
#define ObjFusn_START_SEC_CODE

void idProvider_init(void);
#define ObjFusn_STOP_SEC_CODE

/**
 * @brief obtain new ID from ID provider
 * may be called only in state TUE_PRV_IDPROVIDER_STATE_CYCLE_STARTED
 * @return if no error occured: valid ID (0... MAX_FUSION_ID)
 *         if error occured: TUEOBJFUSN_OBJLISTINPUT_U16ID_DEFAULT
 */
#define ObjFusn_START_SEC_CODE

uint16 idProvider_getNewFusionId(void);
#define ObjFusn_STOP_SEC_CODE

/**
 * @brief set ID to be cleared at end of cycle
 * @param u16FusionId   ID to be released
 * @return TRUE (ok) or FALSE (error occured)
 */
#define ObjFusn_START_SEC_CODE

uint32 idProvider_releaseFusionId(const uint16 u16FusionId);
#define ObjFusn_STOP_SEC_CODE

/**
 * @brief release ID
 * is called once at end of cycle to set all deleted cylce as free for the next
 * cycle
 * @return
 */
#define ObjFusn_START_SEC_CODE

void idProvider_finishCycle(void);
#define ObjFusn_STOP_SEC_CODE

#ifdef __cplusplus
}
#endif

#endif /* TUE_PRV_IDPROVIDER_H */