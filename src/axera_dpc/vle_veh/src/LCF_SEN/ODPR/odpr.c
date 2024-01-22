/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*! \file **********************************************************************

  PROJECT:                   LCF_100

  CPU:                       CPU-Independent

  COMPONENT:                 ODPR (Object Detect PreProcess)

  MODULNAME:                 odpr.c

  DESCRIPTION:               Object Detect PreProcess main tasks

  AUTHOR:                    $Author: jun.wang

  CREATION DATE:             $Date: 2020/08/21

  VERSION:                   $Revision: 1.0.0


  CHANGES:
  ---*/ /*---
                                                                                                                                                                                                                                CHANGE:
                                                                                                                                                                                                                              $Log:
                                                                                                                                                                                                                              odpr.c
                                                                                                                                                                                                                                CHANGE:
                                                                                                                                                                                                                              Initial
                                                                                                                                                                                                                              version

                                                                                                                                                                                                                              ****************************************************************************
                                                                                                                                                                                                                              */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "odpr_ext.h"
#include "odpr_fop_ext.h"
#include "odpr_foh_ext.h"
#include "odpr_ile_ext.h"

/*****************************************************************************
  Functionname:    LCF_ODPR_Reset                                        */ /*!

              @brief           Reset function of ODPR

              @description     All global variables related to ODPR are reset in
            this
            function
                               when ODPR executes for the first time, or system
            exception needs
                               to be reset

              @param[in]       none

              @return          none
            *****************************************************************************/
void LCF_ODPR_Reset(void) {
    /* Reset the global variables in FOP function */
    ODPR_FOP_Reset();

    /* Reset the global variables in FOH function */
    ODPR_FOH_Reset();

    /* Reset the global variables in ILE function */
    ODPR_ILE_Reset();
}

/*****************************************************************************
  Functionname:    LCF_ODPR_Exec                                        */ /*!

                 @brief           Execution entry of ODPR function

                 @description     ODPR(Object data process) is a preprocessing
               function
               that
                                  performs FOP(Front Obejct Process), FOH(Front
               Obejct
               History),
                                  and ILE(In Lane Evaluate)

                 @param[in]       reqPorts : Pointer to structure...
                                       reqPorts->sAccObject :  Information of
               OOI_NEXT
               obejct from ACC module
                                       reqPorts->sEgoVehSig :  Dynamic data of
               ggo
               vehicle
               from VED module
                                       reqPorts->sSystemPara : System parameter
                                       reqPorts->sLaneData :   Lane information
               from
               ABPR(Any Boundary Process)
                                       reqPorts->sObjectList : Objects list
               information
               from side radars

                 @param[in]       param : Pointer to structure...
                                       param->FOPParam; FOP calibration
               parameters
                                       param->pFOHParam; FOH calibration
               parameters
                                       param->ILEParam; ILE calibration
               parameters

                 @param[in,out]   proPorts : Pointer to structure...
                                       proPorts->sFOPOutData : Output structure
               of
               FOP
               fucntion
                                       proPorts->sFOHOutData : Output structure
               of
               FOH
               fucntion
                                       proPorts->sILEOutData : Output structure
               of
               ILE
               fucntion

                 @param[in,out]   debug : Pointer to structure...
                                       debug->FOPDebug : FOP debug output
                                       debug->sFOHDebug : FOH debug output
                                       debug->ILEDebug : ILE debug output

                 @return          none
               *****************************************************************************/
void LCF_ODPR_Exec(const ODPRInReq_t* reqPorts,
                   const ODPRParam_t* param,
                   ODPROutPro_t* proPorts,
                   ODPRDebug_t* debug) {
    // printf("--------------------ODPR--------------------\n");
    FOPInReq_t reqPorts1 = {&reqPorts->sAccObject, &reqPorts->sSystemPara,
                            &reqPorts->sEgoVehSig};
    // FOPParam_t param1 = { param->pFOPParam };
    FOPOutPro_t proPorts1 = {&proPorts->sFOPOutData};
    FOPDebug_t debugInfo1 = {&debug->sFOPDebug};

    FOHInReq_t reqPorts2 = {&proPorts->sFOPOutData, &reqPorts->sSystemPara,
                            &reqPorts->sEgoVehSig};
    // FOHParam_t param2 = { param->pFOHParam };
    FOHOutPro_t proPorts2 = {&proPorts->sFOHOutData};
    FOHDebug_t debugInfo2 = {&debug->sFOHDebug};

    ILEInReq_t reqPorts3 = {&reqPorts->sLaneData, &reqPorts->sObjectList};
    // ILEParam_t param3 = { param->pILEParam };
    ILEOutPro_t proPorts3 = {&proPorts->sILEOutData};
    ILEDebug_t debugInfo3 = {&debug->sILEDebug};

    // step 1, ODPR_FOP_Exec...
    // Execution entry of FOP function
    // FOPMain(reqPorts->sAccObject, reqPorts->sSystemPara,
    // reqPorts->sEgoVehSig, proPorts->sFOPOutData);
    ODPR_FOP_Exec(&reqPorts1, param, &proPorts1, &debugInfo1);

    // step 2, ODPR_FOH_Exec...
    // Execution entry of FOH function
    // FOHMain(proPorts->sFOPOutData, reqPorts->sSystemPara,
    // reqPorts->sEgoVehSig, proPorts->sFOHOutData);
    ODPR_FOH_Exec(&reqPorts2, param, &proPorts2, &debugInfo2);

    // step 3, ODPR_ILE_Exec...
    // Execution entry of ILE function
    // ILEMain(reqPorts->sLaneData, reqPorts->sObjectList,
    // proPorts->sILEOutData);
    ODPR_ILE_Exec(&reqPorts3, param, &proPorts3, &debugInfo3);
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */