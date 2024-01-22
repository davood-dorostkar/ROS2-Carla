#ifndef MPFMVSOBJLISTTOOBJFUSNOBJLISTCONVERT_H_
#define MPFMVSOBJLISTTOOBJFUSNOBJLISTCONVERT_H_ 1

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
 *    Input adapter from TUE MVS ObjectList to TUE ObjectFusionList
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

boolean MVSObjListToTueObjListConverter(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pNewObjList,
    CONSTP2CONST(CamObjectList, AUTOMATIC, ObjFusn_VAR_NOINIT) inputCamObjList,

    const uint32 u32globalTimeStamp);
void OtherCarLocHistoryInit();
uint8 MinNumberOfArray(float32* InputArray, uint8 ArraySize);

// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/31 changed by
// changchang 20200603 start
void PedestrianLocHistoryInit();
// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/31 changed by
// changchang 20200603 end

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

#endif /* _TUEMVSOBJLISTTOOBJFUSNOBJLISTCONVERT_H_ */
