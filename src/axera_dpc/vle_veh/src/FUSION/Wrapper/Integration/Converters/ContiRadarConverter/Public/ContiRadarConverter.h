#ifndef CONTIRADARCONVERTER_H_
#define CONTIRADARCONVERTER_H_ 1

/******************************************************************************

AUTOLIV ELECTRONIC document.

-------------------------------------------------------------------------------

Copyright Tuerme Inc. All rights reserved.

*******************************************************************************
H-File Template Version:
*******************************************************************************

Overview of the interfaces:
   Conversion from TUEObjList v123 to TUEObjList v131

******************************************************************************/
/* PRQA S 0288 ++ */
/*
 * Explanation:
 *    Input adapter from Conti Radar ObjectList to TUE ObjectFusionList
 */
/*
 */
/* PRQA S 0288 -- */
/*****************************************************************************/
/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/
#include "fusion_ext.h"
#include "tue_prv_common_types.h"
#include "TueObjFusn_ObjectListType.h"
#include "Converters_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
DEFINITION OF CONSTANTS
******************************************************************************/

/******************************************************************************
DECLARATION OF TYPES
******************************************************************************/

/******************************************************************************
DECLARATION OF VARIABLES
******************************************************************************/

/******************************************************************************
DECLARATION OF CONSTANT DATA
******************************************************************************/

/******************************************************************************
DECLARATION OF FUNCTIONS
******************************************************************************/
#define ObjFusn_START_SEC_SLOW_CODE

boolean ContiRaObjListToTueObjListConverter(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pNewObjList,
    CONSTP2CONST(FusObjectList_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pOldObjList,
    const uint32 u32globalTimeStamp,
    uint8 uRadarID);

// boolean TueFusionObjListToContiRaObjListConverter(
//     CONSTP2VAR(FusObjects_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pNewObjList,
//     CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
//         pOldObjList);

#define ObjFusn_STOP_SEC_SLOW_CODE

/******************************************************************************
DECLARATION OF FUNCTION-LIKE MACROS
******************************************************************************/

#ifdef __cplusplus
}
#endif
/******************************************************************************
End Of File
*****************************************************************************/

#endif /* _TUERADAROBJLISTTOOBJFUSNOBJLISTCONVERT_H_ */
