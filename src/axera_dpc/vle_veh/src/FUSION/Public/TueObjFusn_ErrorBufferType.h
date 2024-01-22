/** \addtogroup tue_prv_error_management
 *  @{
 * \file TueObjFusn_ErrorBufferType.h
 *
 * \brief Defines common error buffer type and the corresponding min/max values.
 *
 *
 *
 *   (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUEOBJFUSN_ERRORBUFFERTYPE_H
#define TUEOBJFUSN_ERRORBUFFERTYPE_H
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "tue_common_libs.h"
// #include "TM_Global_Types.h"
//#include "Fusion_Std_Types.h"
#include "Std_Types.h"
#include "FusionCompiler.h"
#include "TueObjFusn_ConfigVehicle.h"
#include "TueObjFusn_ConfigAlgorithm.h"

/// @name u32ErrorCode

/****************************/
/* U32ERRORCODE - mandatory */
/****************************/
/** minimum value */
#define TUEOBJFUSN_ERRORBUFFER_U32ERRORCODE_MIN (0x00000001u)
/** maximum value */
#define TUEOBJFUSN_ERRORBUFFER_U32ERRORCODE_MAX (0x80000000u)
/** default value */
#define TUEOBJFUSN_ERRORBUFFER_U32ERRORCODE_DEFAULT (0xFFFFFFFFu)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_ERRORBUFFER_U32ERRORCODE_FILLEDBYFUSION (1)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_ERRORBUFFER_U32ERRORCODE_ISENUM (0)

///@}

/// @name u8AAU_Code

/**************************/
/* U8AAU_CODE - mandatory */
/**************************/
/** minimum value */
#define TUEOBJFUSN_ERRORBUFFER_U8AAUCODE_MIN (0x01u)
/** maximum value */
#define TUEOBJFUSN_ERRORBUFFER_U8AAUCODE_MAX (0xFEu)
/** default value */
#define TUEOBJFUSN_ERRORBUFFER_U8AAUCODE_DEFAULT (0xFFu)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_ERRORBUFFER_U8AAUCODE_FILLEDBYFUSION (1)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_ERRORBUFFER_U8AAUCODE_ISENUM (0)

///@}

/// @name u8FunctionCode

/******************************/
/* U8FUNCTIONCODE - mandatory */
/******************************/
/** minimum value */
#define TUEOBJFUSN_ERRORBUFFER_U8FUNCTIONCODE_MIN (0x01u)
/** maximum value */
#define TUEOBJFUSN_ERRORBUFFER_U8FUNCTIONCODE_MAX (0xFEu)
/** default value */
#define TUEOBJFUSN_ERRORBUFFER_U8FUNCTIONCODE_DEFAULT (0xFFu)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_ERRORBUFFER_U8FUNCTIONCODE_FILLEDBYFUSION (1)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_ERRORBUFFER_U8FUNCTIONCODE_ISENUM (0)

///@}

/// @name u16Age

/**********************/
/* U16AGE - mandatory */
/**********************/
/** minimum value */
#define TUEOBJFUSN_ERRORBUFFER_U16AGE_MIN (0x0000u)
/** maximum value */
#define TUEOBJFUSN_ERRORBUFFER_U16AGE_MAX \
    (TUE_PRV_ERROR_MANAGEMENT_MAX_ERROR_AGE)
/** default value */
#define TUEOBJFUSN_ERRORBUFFER_U16AGE_DEFAULT (0xFFFFu)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_ERRORBUFFER_U16AGE_FILLEDBYFUSION (1)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_ERRORBUFFER_U16AGE_ISENUM (0)

///@}

/// @name u16NextWriteIdx

/*******************************/
/* U16NEXTWRITEIDX - mandatory */
/*******************************/
/** minimum value */
#define TUEOBJFUSN_ERRORBUFFER_U16NEXTWRITEIDX_MIN (0x0000u)
/** maximum value */
#define TUEOBJFUSN_ERRORBUFFER_U16NEXTWRITEIDX_MAX \
    (TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE - 1u)
/** default value */
#define TUEOBJFUSN_ERRORBUFFER_U16NEXTWRITEIDX_DEFAULT (0xFFFFu)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_ERRORBUFFER_U16NEXTWRITEIDX_FILLEDBYFUSION (1)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_ERRORBUFFER_U16NEXTWRITEIDX_ISENUM (0)

///@}

/// @name u16NumOfItems

/*****************************/
/* U16NUMOFITEMS - mandatory */
/*****************************/
/** minimum value */
#define TUEOBJFUSN_ERRORBUFFER_U16NUMOFITEMS_MIN (0x0000u)
/** maximum value */
#define TUEOBJFUSN_ERRORBUFFER_U16NUMOFITEMS_MAX \
    (TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE)
/** default value */
#define TUEOBJFUSN_ERRORBUFFER_U16NUMOFITEMS_DEFAULT (0xFFFFu)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_ERRORBUFFER_U16NUMOFITEMS_FILLEDBYFUSION (1)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_ERRORBUFFER_U16NUMOFITEMS_ISENUM (0)

///@}

typedef struct TueObjFusn_ErrorTypeTag {
    uint32 u32ErrorCode;
    uint8 u8AAU_Code;
    uint8 u8FunctionCode;
    uint16 u16Age;
} TueObjFusn_ErrorType;

typedef struct TueObjFusn_ErrorBufferTypeTag {
    TueObjFusn_ErrorType
        aErrorBuffer[TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE]; /**< ringbuffer
                                                               items for an
                                                               amount of time*/
    uint16 u16NextWriteIdx; /**< index of next free item */
    uint16 u16NumOfItems;   /**< Note that u16NumOfItems does not necessarily
                               point to the last object in the buffer */
} TueObjFusn_ErrorBufferType;

#endif /* TUEOBJFUSN_ERRORBUFFERTYPE_H */