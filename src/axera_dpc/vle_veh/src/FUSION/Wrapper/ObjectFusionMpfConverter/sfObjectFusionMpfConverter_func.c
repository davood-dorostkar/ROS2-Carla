/******************************************************************************

AUTOLIV ELECTRONIC document.

-------------------------------------

Copyright Tuerme Inc. All rights reserved.

*******************************************************************************
C-File Template Version:
******************************************************************************/
/* PRQA S 0288 ++ */
/*
 * Explanation:
 *    Source file for s-function wrapper for RaCam converter in CADS4 simulink
 */
/*
 */
/* PRQA S 0288 -- */
/*!****************************************************************************

@details
   <Describes details of this module 'Template.c' file within overall
    context of component implementation>

******************************************************************************/
/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/

#include "sfObjectFusionMpfConverter_func.h"
#include <string.h>

/******************************************************************************
MODULE DEFINES
******************************************************************************/

/******************************************************************************
MODULE TYPES
******************************************************************************/

/******************************************************************************
DECLARATION OF LOCAL FUNCTIONS
******************************************************************************/

/******************************************************************************
DEFINITION OF EXPORTED VARIABLES
******************************************************************************/

/******************************************************************************
DEFINITION OF LOCAL CONSTANT DATA
******************************************************************************/

/******************************************************************************
DEFINITION OF EXPORTED CONSTANT DATA
******************************************************************************/

/******************************************************************************
MODULE FUNCTION-LIKE MACROS
******************************************************************************/

/******************************************************************************
DEFINITION OF LOCAL FUNCTION
******************************************************************************/

/******************************************************************************
DEFINITION OF APIs
******************************************************************************/

/*!****************************************************************************
 * @details
 *     Initialize TUE raCam converter
 * @param[in]
 *     none
 * @param[out]
 *     none
 * @return
 *     none
 * @constraints
 *     <>
 * @invocation_criteria
 *     <>
 ******************************************************************************/

void sfObjectFusionMpfConverter_Init_wrapper() {
    //ObjFusnObjListToVolvoSnsrFusnConvertInit();
}

/*!****************************************************************************
 * @details
 *     Call RaCam output adapter
 * @param[in]
 *     const TueObjFusn_ObjListInputType *pFusnObjLists_Input
 * @param[in]
 *     TueObjFusn_EgoMotionType *EgoMotion
 * @param[in]
 *     TueObjFusn_EgoMotionType *EgoMotion
 * @param[out]
 *     const uint32 u32TimeStamp
 * @param[out]
 *     stRaCamObjectVcc_t *pRACAMObjListOut
 * @return
 *     none
 * @constraints
 *     <>
 * @invocation_criteria
 *     <>
 ******************************************************************************/

boolean sfObjectFusionMpfConverter_Outputs_wrapper(
    TueObjFusn_ObjectListType *const pFusnObjLists_Input,
    TueObjFusn_EgoMotionType *const pEgoMotion,
    const uint32 u32TimeStamp,
    typeRaCamObjectVcc *const pRACAMObjListOut) {
    boolean bSuccess;

    // call fusion to racam converter
    // bSuccess = ObjFusnObjListToVolvoSnsrFusnConvert(
    //     pFusnObjLists_Input, pEgoMotion, pRACAMObjListOut, u32TimeStamp);

    return bSuccess;
}

/******************************************************************************
End Of File
*****************************************************************************/
