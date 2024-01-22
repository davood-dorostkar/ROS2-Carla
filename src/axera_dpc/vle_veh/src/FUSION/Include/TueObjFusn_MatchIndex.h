/** \addtogroup MatchIndex
 *  @{
 * \file    TueObjFusn_MatchIndex.h
 * \brief  This is a structure definition file for the internal association
 * match index matrix
 *
 *
 *
 *
 * <br>=====================================================<br>
 * <b>Copyright 2014 by Tuerme.</b>
 * <br>
 * All rights reserved. Property of Tuerme.<br>
 * Restricted rights to use, duplicate or disclose of this code<br>
 * are granted through contract.
 * <br>=====================================================<br>
 */

#ifndef TUEOBJFUSN_MATCHINDEX_H_
#define TUEOBJFUSN_MATCHINDEX_H_

#include "tue_prv_common_types.h" /* standard types (uint16, float32, etc.) */
#include "TueObjFusn_ConfigConstants.h"

/*==================[macros]================================================*/

#if (TUE_PRV_FUSION_MAX_INPUT_OBJECTS < TUE_PRV_FUSION_TRACKABLE_LIST_SIZE)
#define TUE_PRV_MATCH_INDEX_ARRAY_SIZE TUE_PRV_FUSION_MAX_INPUT_OBJECTS
#else
#define TUE_PRV_MATCH_INDEX_ARRAY_SIZE TUE_PRV_FUSION_TRACKABLE_LIST_SIZE
#endif

/**
 * @struct stMatchIndex_t
 * @brief   describes a matching index of an associated object pair coming from
 * list 1 and list 2
 * @param   u16IndexCol    uint16, position of the object of list 1 associated
 * to list 2
 * @param   u16IndexRow    uint16, position of the object of list 2 associated
 * to list 1
 * @param   u16Score       uint16, distance score
 */
typedef struct stMatchIndex_t_tag {
    uint16 u16IndexCol;
    uint16 u16IndexRow;
} stMatchIndex_t;

/**
 * @struct  stMatchIndexArrayType
 * @brief   describes the matching index array
 * @param   u16NumMatches     uint16, Number of matches currently stored in the
 * array
 * @param   aMatchIndexArray  stMatchIndex_t[], match index array
 */
typedef struct stMatchIndexArrayTypeTag {
    uint16 u16NumMatches;
    stMatchIndex_t aMatchIndexArray[TUE_PRV_MATCH_INDEX_ARRAY_SIZE];
} stMatchIndexArrayType;

#endif /**@} TUEOBJFUSN_MATCHINDEX_H_ */
