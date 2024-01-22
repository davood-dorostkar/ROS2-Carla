/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "TM_Global_Types.h"

#include "ops.h"
#include <string.h>

#include "ops_eba_quality_par.h"
#include "ops_par.h"
#include "ops_eba_quality.h"
#include "ops_eba_mosa.h"
#include "ops_eba_quality_cfct.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
FctPreselEBAObj_t FctPreselEBAObj[Envm_NR_PRIVOBJECTS];
OPSEBA_MOSA_ObjFlags_t sOutPutMOSAObjFlags[Envm_NR_PRIVOBJECTS];  // for output
                                                                  // moving safe
                                                                  // internal
                                                                  // variables
                                                                  // to csv
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

static const uint8 OPS_EBA_POE_THRES[OPS_EBA_NUMBER_OF_POE_THRES] = {
    OPS_EBA_POE_THRES_OFFSET_00, OPS_EBA_POE_THRES_OFFSET_01,
    OPS_EBA_POE_THRES_OFFSET_02, OPS_EBA_POE_THRES_OFFSET_03,
    OPS_EBA_POE_THRES_OFFSET_04, OPS_EBA_POE_THRES_OFFSET_05,
    OPS_EBA_POE_THRES_OFFSET_06, OPS_EBA_POE_THRES_OFFSET_07,
    OPS_EBA_POE_THRES_OFFSET_08, OPS_EBA_POE_THRES_OFFSET_09,
    OPS_EBA_POE_THRES_OFFSET_10, OPS_EBA_POE_THRES_OFFSET_11,
    OPS_EBA_POE_THRES_OFFSET_12, OPS_EBA_POE_THRES_OFFSET_13,
    OPS_EBA_POE_THRES_OFFSET_14, OPS_EBA_POE_THRES_OFFSET_15};

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/*****************************************************************************
  @fn           OPSEBAProcess

  @brief        Process OPS EBA

  @description  Call all functions realted to OPS EBA here

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
void OPSEBAProcess(void) {
    // step01,OPS pre-processing,update object cycletime and measured stats
    OPSEBAPreselPreProc();
    // step02,calculate object quality and add hypothesis catagroy
    OPSEBAPresel();
    // step03,OPS post-processing
    OPSEBAPreselPostProc();
}

/*****************************************************************************
  @fn           OPSAEBInit

  @brief        Initialize FPS EBA Module

  @description  Loop all Object ,call OPSInitEBAObject function to Initialize

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
void OPSAEBInit(void) {
    sint8 iObjNumber;
    for (iObjNumber = (Envm_NR_PRIVOBJECTS - 1); iObjNumber >= 0;
         iObjNumber--) {
        // Initializes the data required for the object quality calculation
        // Include Observer check result - Objsafe,object measurement state,and
        // history dynamic property
        OPSInitEBAObject(iObjNumber);
    }
}

/*****************************************************************************
  @fn           OPSInitEBAObject

  @brief        Initialize OPS EBA object related values

  @description  Initialize OPS EBA object ObjSafe,Quality,DynamicProperty
values,etc

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  note:Initialize OPS EBA object related values
  partition OPSInitEBAObject{
  :pointer to pEBAObj;
  note:create pointer to pEBAObj structure
  :init fEBAObjectQuality;
  note:reset EBA Object fEBAObjectQuality to 0
  :init ObjSafe;
  note:reset EBA Object ObjSafe to 0
  partition  Init_history_of_DynProp {
  :sDynPropHistory.bWasCrossing   = FALSE;
  :sDynPropHistory.bWasMoving     = FALSE;
  :sDynPropHistory.bWasOncoming   = FALSE;
  :sDynPropHistory.bWasStationary = FALSE;
  :sDynPropHistory.bWasStopped    = FALSE;
  :sDynPropHistory.bWasMovingSafe = FALSE;
  }
  partition  Reset_public_output {
  :EBAPresel.ucEbaMovingObjQuality = 0;
  :EBAPresel.bCrossingPedEbaPresel = 0;
  :EBAPresel.eEbaInhibitionMask = 0;
  :EBAPresel.eEbaHypCat = 0;
  }
  end
  @enduml


******************************************************************************/
void OPSInitEBAObject(sint8 iObjNumber) {
    // check iObjNumber range
    if (iObjNumber < Envm_NR_PRIVOBJECTS) {
        // pointer of FctPresel
        FctPreselEBAObj_t* const pEBAObj = &FctPreselEBAObj[iObjNumber];

        // reset EBA Object
        pEBAObj->fEBAObjectQuality = 0;
        pEBAObj->iObjSafe = 0u;

        // reset history of object dynamic property
        memset(&pEBAObj->sDynPropHistory, 0, sizeof(OPS_DynPropHistory_t));

        // rest public output
        Envm_INT_OBJ_GET_EBA_MOV_PRESEL_QUALITY(iObjNumber) = 0u;
        EM_INT_OBJ_GET_EBA_CROSSING_PED_PRESEL(iObjNumber) = FALSE;
        EM_INT_OBJ_GET_EBA_INHIBITIONMASK(iObjNumber) = FPS_EBA_INH_ALL;
        Envm_INT_OBJ_GET_EBA_HYP_CAT(iObjNumber) = 0u;
    } else {
        //__EM_ASSERT(FALSE);
    }
}

/*****************************************************************************
  @fn           OPSEBAPreselPreProc

  @brief        EBA Sensor preselection pre-processing

  @description  This function performs any once per cycle pre-processing
                necessary for EBA sensor preselection
  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
void OPSEBAPreselPreProc(void) {
    // moving safe check pre process
    OPSEBA_MOSA_PreProcess();
    // update object measurement state flag
    // move update function to ops main for ACC Quality calculate by
    // changchang20200610
    // OPSObjectMeasureStatusUpdate();
}

/*****************************************************************************
  @fn           OPSObjectMeasureStatusUpdate

  @brief        Update Object current MeasureStatus

  @description  update Object MeasureStatus to
EnvmData.pPrivObjList.Qualifiers.uMeasuredTargetFrequency

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :iObjNumber;
  note:iObjNumber = Envm_NR_PRIVOBJECTS-1
  while(iObjNumber)
        if (isn't deleted object) then(yes)
                :EM_v_ShiftCurrentTgtConfirmDensity();
                note:Shift the flag bit to record the current state bit;
                if(is measured or new from merge object) then(yes)
                        note
                        Set the status bit to 1
                        for the measured or
                        new from merge object
                        end note
                        :set uMeasuredTargetFrequencyNear;
                        :set uMeasuredTargetFrequencyFar;
                        note
                        write to
                        EnvmData.pPrivObjList.Objects.Qualifiers
                        end note
                endif
        else(no)
                :Envm_v_SetTgtConfirmDensitySingleScan();
                note:Set all the status bit to 0;
        endif
        :iObjNumber--;
  end while

  end
  @enduml
******************************************************************************/
// move update function to ops main for ACC Quality calculate by
// changchang20200610
// void OPSObjectMeasureStatusUpdate(void)
//{
//	sint8 iObjNumber;
//
//	//store measured target frequency value since we will memset all the
// value after fusion process
//	static uint8 uHistoryMeasuredTargetFrequencyNear[Envm_NR_PRIVOBJECTS];
//	static uint8 uHistoryMeasuredTargetFrequencyFar[Envm_NR_PRIVOBJECTS];
//	//loop all object for update measurestatus
//	for (iObjNumber = (Envm_NR_PRIVOBJECTS - 1); iObjNumber >= 0;
// iObjNumber--)
//	{
//		//Write previous cycle data to EnvmData
//		EnvmData.pPrivObjList->Objects[iObjNumber].Qualifiers.uMeasuredTargetFrequencyNear
//= uHistoryMeasuredTargetFrequencyNear[iObjNumber];
//		EnvmData.pPrivObjList->Objects[iObjNumber].Qualifiers.uMeasuredTargetFrequencyFar
//= uHistoryMeasuredTargetFrequencyFar[iObjNumber];
//		if (!EM_INT_OBJ_IS_DELETED(iObjNumber))
//		{
//			// Shift the flag bit to record the current state bit
//			EM_v_ShiftCurrentTgtConfirmDensity(iObjNumber);
//
//			//Set the status bit to 1 for the measured or new from
// merge object 			if
// ((EM_INT_OBJ_MAINTENANCE_STATE(iObjNumber)
// ==
// MT_STATE_MEASURED) || EM_INT_OBJ_MAINTENANCE_STATE(iObjNumber) ==
// MT_STATE_MERGE_NEW)
//			{
//				//Set the status bit to 1
//				EnvmData.pPrivObjList->Objects[iObjNumber].Qualifiers.uMeasuredTargetFrequencyNear
//|= (ui8_t)(1uL << (CONFIRM_DENSITY_SIZE - 1uL));
//				EnvmData.pPrivObjList->Objects[iObjNumber].Qualifiers.uMeasuredTargetFrequencyFar
//|= (ui8_t)(1uL << (CONFIRM_DENSITY_SIZE - 1uL));
//
//			}
//		}
//		else
//		{
//			//Set all the status bit to 0
//			Envm_v_SetTgtConfirmDensitySingleScan(iObjNumber,
// CONFIRM_DENSITY_DEAFULT, SYS_SCAN_NEAR);
//			Envm_v_SetTgtConfirmDensitySingleScan(iObjNumber,
// CONFIRM_DENSITY_DEAFULT, SYS_SCAN_FAR);
//
//		}
//		//save calculate measurement history to static value
//		uHistoryMeasuredTargetFrequencyNear[iObjNumber] =
// EnvmData.pPrivObjList->Objects[iObjNumber].Qualifiers.uMeasuredTargetFrequencyNear;
//		uHistoryMeasuredTargetFrequencyFar[iObjNumber] =
// EnvmData.pPrivObjList->Objects[iObjNumber].Qualifiers.uMeasuredTargetFrequencyFar;
//
//	}
//}

/*****************************************************************************
  @fn           OPSEBAPresel

  @brief        Calculate EBA preselection qualities

  @description  Calculate EBA object qualities

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :ObjNr = (Envm_NR_PRIVOBJECTS - 1);
  while(ObjNr >= 0)
        if (!EM_INT_OBJ_IS_DELETED(ObjNr)) then(yes)
                :OPSResetEBAInhibitionMask(ObjNr);
                note:reset EBA inhibition mask,all object set to suppress status
                :bObjIsCrossing = OPSCheckCrossingProperty(ObjNr);
                note:Check whether the object is crossing
                :OPSSetEBAHypCat( ObjNr, bObjIsCrossing);
                note:set object HypCat
                :OPSCalculateObjectQuality(ObjNr);
                note:calculate object quality
                :OPSEBA_MOSA_Process(ObjNr);
                note:object movement check
                :OPSEBAPreSelCustomFct(ObjNr);
                note:custom function
                :OPSDynPropHistoryUpdate( ObjNr, bObjIsCrossing);
                note:save current object dynamic property
        else(no)
                :EBAPresel.ucEbaMovingObjQuality = 0u;
                :EBAPresel.bCrossingPedEbaPresel = FALSE;
                :EBAPresel.eEbaHypCat            = 0u;
                :EBAPresel.eEbaInhibitionMask    = FPS_EBA_INH_ALL;
        endif
  endwhile
  end
  @enduml

******************************************************************************/
void OPSEBAPresel(void) {
    boolean bObjIsCrossing;
    sint8 iObjNr;

    for (iObjNr = (Envm_NR_PRIVOBJECTS - 1); iObjNr >= 0L; iObjNr--) {
        if (!EM_INT_OBJ_IS_DELETED(iObjNr)) {
            /* reset EBA inhibition mask,all object set to suppress status*/
            OPSResetEBAInhibitionMask(iObjNr);

            /* Check whether the object is crossing*/
            bObjIsCrossing = OPSCheckCrossingProperty(iObjNr);

            /* set EBA HypCat */
            OPSSetEBAHypCat(iObjNr, bObjIsCrossing);

            /* calculate object quality */
            OPSCalculateObjectQuality(iObjNr);

            /*object movement check*/
            OPSEBA_MOSA_Process(iObjNr);

            /*custom function*/
            /*OPSEBAPreSelCustomFct(iObjNr);*/
            // use extern CustomFct interface 20200610 by changchang
            FPSEBAPreSelCustomFct(iObjNr);

            /*save current object dynamic property*/
            OPSDynPropHistoryUpdate(iObjNr, bObjIsCrossing);

        } else {
            // Quality = 0
            Envm_p_GetPrivObject(iObjNr)->EBAPresel.ucEbaMovingObjQuality = 0;
            // bCrossingPedEbaPresel = 0
            Envm_p_GetPrivObject(iObjNr)->EBAPresel.bCrossingPedEbaPresel =
                FALSE;
            // OPSSetEBAHypCat = 0
            Envm_p_GetPrivObject(iObjNr)->EBAPresel.eEbaHypCat = 0;
            // InhibitonMask = FPS_EBA_INH_ALL
            Envm_p_GetPrivObject(iObjNr)->EBAPresel.eEbaInhibitionMask =
                FPS_EBA_INH_ALL;
        }
    }
}

/*****************************************************************************
  @fn           OPSResetEBAInhibitionMask

  @brief        Set EBA inhibition-mask

  @description  Set the EBA inhibition-mask. Allow everything in base project.
                Application project specific changes can be done in
                OPSEBAPreSelCustomFct().

  @param[in]    iObjNumber: The index of the valid (non-free) object
                the EBA inhibition-mask is set for.

  @return       -

  @pre          -

******************************************************************************/
void OPSResetEBAInhibitionMask(sint8 iObjNumber) {
    /* allow everything */
    Envm_p_GetPrivObject(iObjNumber)->EBAPresel.eEbaInhibitionMask = 0u;
}

/*****************************************************************************
  @fn           OPSCheckCrossingProperty

  @brief        Check whether object speed indicates a crossing object

  @description  Check whether object speed indicates a crossing object

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :set bIsCrossingObj;
  note: bIsCrossingObj = FALSE
  :set OrientationStdThresh;
  note: OrientationStdThresh = 45
  :set fUpperOrientationThresh;
  note: fUpperOrientationThresh = 135
  :set fLowerOrientationThresh;
  note: fLowerOrientationThresh = 45
  if (bWasCrossing == TRUE) then(yes)
        :calculate OrientationStdThresh;
        note: OrientationStdThresh = 180
        :calculate fUpperOrientationThresh;
        note: fUpperOrientationThresh += 22.5
        :calculate fLowerOrientationThresh;
        note: fLowerOrientationThresh -= 22.5
  endif
  if ((OrientationStd < fOrientationStdThresh)&&
  (Orientation)  > fLowerOrientationThresh )&&
  (Orientation) <  fUpperOrientationThresh ) ) then(yes)
        :set bIsCrossingObj;
        note: bIsCrossingObj = TRUE
  endif
  :return bIsCrossingObj;
  @enduml

******************************************************************************/
static boolean OPSCheckCrossingProperty(sint8 iObjNumber) {
    FctPreselEBAObj_t const* const pFctPreselObj = &FctPreselEBAObj[iObjNumber];
    Geometry_t const* const pEnvmObjGeom =
        &Envm_p_GetPrivObject(iObjNumber)->Geometry;
    boolean bIsCrossingObj = FALSE;
    float32 fOrientationStdThresh = DEG2RAD(45.0F);
    float32 fOrientationUpperThresh = DEG2RAD(135.0F);
    float32 fOrientationLowerThresh = DEG2RAD(45.0F);

    // Apply pervious cycle check result to modify thresh value(apply
    // hysteresis)
    if (pFctPreselObj->sDynPropHistory.bWasCrossing == TRUE) {
        fOrientationStdThresh = DEG2RAD(180.0F);
        fOrientationUpperThresh += DEG2RAD(22.5F);
        fOrientationLowerThresh -= DEG2RAD(22.5F);
    }

    // Check crossing condition
    if ((pEnvmObjGeom->fOrientationStd < fOrientationStdThresh) &&
        (fABS(pEnvmObjGeom->fOrientation) < fOrientationUpperThresh) &&
        (fABS(pEnvmObjGeom->fOrientation) > fOrientationLowerThresh)) {
        bIsCrossingObj = TRUE;
    }

    return bIsCrossingObj;
}

/*****************************************************************************
  @fn           OPSSetEBAHypCat

  @brief        Set EBA Hypothesis Category

  @description  Bitfield for collision hypothesis usage of objects

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  if (POE > MIN_POE_Thresh) then(yes)
        if ((isCrossing)||
        (stopped confidence > 80)&&
        ( WasCrossing))) then(yes)
                :set uiObjHypCat;
                note:uiObjHypCat |= OPS_EBA_HYP_CAT_XING
        endif
  endif
  if (bCrossingPedEbaPresel == TRUE) then(yes)
        :set uiObjHypCat;
        note:uiObjHypCat |= OPS_EBA_HYP_CAT_PED
        :set uiObjHypCat;
        note:uiObjHypCat |= OPS_EBA_HYP_CAT_CYCL
  endif
  :switch Attributes.eDynamicProperty;
  if(:=OBJECT_PROPERTY_STATIONARY;) then(yes)
        :set uiObjHypCat;
        note:uiObjHypCat |= OPS_EBA_HYP_CAT_STAT
        if (stopped confidence > 80) then(yes)
                if((bWasOncoming == FALSE)&&
                (bWasCrossing == FALSE) then(yes)
                :set uiObjHypCat;
                note:uiObjHypCat |= OPS_EBA_HYP_CAT_VCL
                endif
        endif
  elseif(:=OBJECT_PROPERTY_ONCOMING;)
        :set uiObjHypCat;
        note:uiObjHypCat |= OPS_EBA_HYP_CAT_ONC
  elseif(:=OBJECT_PROPERTY_MOVING;)
        :set uiObjHypCat;
        note:uiObjHypCat |= OPS_EBA_HYP_CAT_VCL
  else(no)
        :default;
  endif
  :write uiObjHypCat to eEbaHypCat;
  note:EBAPresel.eEbaHypCat = uiObjHypCat
  end
  @enduml
******************************************************************************/
static void OPSSetEBAHypCat(sint8 iObjNumber, boolean bIsObjCrossing) {
    uint8 uiObjHypCat = OPS_EBA_HYP_CAT_NONE;
    boolean bIsMove2Stat = Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(iObjNumber);
    FctPreselEBAObj_t const* const pFctPreselObj = &FctPreselEBAObj[iObjNumber];
    Objects_t const* const pEnvmObj = Envm_p_GetPrivObject(iObjNumber);

    /* Set HypCat crossing */
    /* A min probability of existence is necessary */
    if (pEnvmObj->Qualifiers.fProbabilityOfExistence >
        OPS_EBA_HYPCAT_CROSS_MIN_POE_QUAL) {
        /* Set crossing category for objects which are currently crossing.
    And for objects which are stopped and were crossing.*/
        if ((bIsObjCrossing == TRUE) ||
            ((pFctPreselObj->sDynPropHistory.bWasCrossing == TRUE) &&
             (bIsMove2Stat == TRUE))) {
            // set xing category depend on current crossing result or pervious
            // cycle dynamic property
            SET_BIT(uiObjHypCat, (uint8)OPS_EBA_HYP_CAT_XING);
        }
    }

    /* Set HypCat for EBA pedestrian */
    if (pEnvmObj->EBAPresel.bCrossingPedEbaPresel == TRUE) {
        // Temporary solution until Pedestrian and bicycle detection is
        // separated.
        SET_BIT(uiObjHypCat, (uint8)OPS_EBA_HYP_CAT_PED);
        // SET_BIT(uiObjHypCat, (uint8)OPS_EBA_HYP_CAT_CYCL);
    }

    // wulin todo 20220316, add EBA bicycle or motorcycle HypCat
    /* Set HypCat for EBA bicycle or motorcycle */
    if (pEnvmObj->EBAPresel.bCrossingBicycleEbaPresel == TRUE) {
        SET_BIT(uiObjHypCat, (uint8)OPS_EBA_HYP_CAT_CYCL);
    }

    /* Add categories dependent on object's dynamic property */
    switch (pEnvmObj->Attributes.eDynamicProperty) {
        case OBJECT_PROPERTY_STATIONARY:

            /* Handle stopped objects */
            SET_BIT(uiObjHypCat, (uint8)OPS_EBA_HYP_CAT_STAT);

            if (bIsMove2Stat == TRUE) {
                /* Objects with dynamic property "oncoming->stopped" and
                 * "crossing->stopped" should not be used in the vehicle
                 * hypothesis */
                if ((pFctPreselObj->sDynPropHistory.bWasOncoming == FALSE) &&
                    (pFctPreselObj->sDynPropHistory.bWasCrossing == FALSE)) {
                    SET_BIT(uiObjHypCat, (uint8)OPS_EBA_HYP_CAT_VCL);
                }
            }
            break;

        case OBJECT_PROPERTY_ONCOMING:
            SET_BIT(uiObjHypCat, (uint8)OPS_EBA_HYP_CAT_ONC);
            break;

        case OBJECT_PROPERTY_MOVING:
            SET_BIT(uiObjHypCat, (uint8)OPS_EBA_HYP_CAT_VCL);
            break;

        default:
            //__EM_ASSERT(FALSE);
            break;
    }

    Envm_p_GetPrivObject(iObjNumber)->EBAPresel.eEbaHypCat = uiObjHypCat;
}

/*****************************************************************************
  @fn           OPSCalculateObjectQuality

  @brief        Calculate Object quality by Object property and moving state

  @description  Calculate Object quality by Object property and moving state

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  note:calculateQuality
  :iObjSafe,iThresh,Limit;
  note:init calculate value;
  if (isn't deleted object) then(yes)
        :OPSUpdateTargetConfirmation();
        note:Confirm that the object exists;
        :OPSUpdateObservers();
        note:Update Observers to calculate objsafe;
        :OPSUpdateNonAbsObservers();
        note:Update NoneObservers to calculate iThresh;
        :calculate ObjectQuality by Thresh,Limit,ObjSafe;
        note
        AEB Quality filter
        the historical value and the current value
        determine the final quality result
        end note
  else(no)
        :ObjectQuality = 0;
  endif
  :write ObjectQuality to FctPresel structure;
  :write ObjectQuality to global envm structure;
  end
  @enduml

******************************************************************************/
static void OPSCalculateObjectQuality(sint8 iObjNumber) {
    ui8_t uiObjSafe = OPS_EBA_OBJ_MAX_SAFETY;
    uint8 iThresh = 100u;
    uint8 iLimit = 100u;
    f32_t fObjectQuality;

    if (!EM_INT_OBJ_IS_DELETED(iObjNumber)) {
        // Temporarily not to use
        // OPSUpdateTargetConfirmation(iObjNumber);

        // step01,Absolute Observer,to calculate quality filter parameter
        OPSUpdateAbsObservers(iObjNumber, &uiObjSafe);

        // step02,NonAbsolute Observer,to calculate qulaity reduce value
        OPSUpdateNonAbsObservers(iObjNumber, &iThresh);
        // save observer objsafe value
        FctPreselEBAObj[iObjNumber].iObjSafe = uiObjSafe;

        // step03,OPS Quality Filter,according to Observer result to filter
        // object quality value
        fObjectQuality = OPSTimeFilter(iObjNumber, iThresh, iLimit, uiObjSafe);
    } else {
        fObjectQuality = 0.0f;
    }

    // calculate finish
    // write to local FCT data
    FctPreselEBAObj[iObjNumber].fEBAObjectQuality = fObjectQuality;

    // write to global envm data
    Envm_p_GetPrivObject(iObjNumber)->EBAPresel.ucEbaMovingObjQuality =
        (uint8)ROUND_TO_UINT(fObjectQuality * 100.0F);
}

/*****************************************************************************
  @fn           OPSUpdateTargetConfirmation

  @brief        Update the near/far confirmation of an object

  @description  Update the near/far confirmation of an object

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
static void OPSUpdateTargetConfirmation(sint8 iObjNumber) {
    /* get 'combined' target confirmation density */
    // uint8 u_tgtDensity = Envm_u_GetTgtConfirmDensity(iObjNumber);

    // if (u_tgtDensity > OPS_EBA_OBJ_MIN_TGT_CURR_CONFIRM_FREQ)
    //{
    //	EM_INT_OBJ_LONG_DISPLACEMENT(iObjNumber);
    //}
}

/*****************************************************************************
  @fn           OPSUpdateAbsObservers

  @brief        Update Absolute Observer

  @description  Update Absolute Observer

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :calculate piOutObjSafe;
  note:piOutObjSafe -= OPS_EBA_GetOBSObserver(iObjNumber)
  :calculate piOutObjSafe;
  note:piOutObjSafe -= OPS_EBA_GetRCSObserver(iObjNumber)
  :calculate piOutObjSafe;
  note:piOutObjSafe -= OPS_EBA_GetPOEObserver(iObjNumber)
  :calculate piOutObjSafe;
  note:piOutObjSafe -= OPS_EBA_GetLFTObserver(iObjNumber)
  end
  @enduml
******************************************************************************/
static void OPSUpdateAbsObservers(sint8 iObjNumber, uint8* piOutObjSafe) {
    *piOutObjSafe -= OPS_EBA_GetOBSObserver(iObjNumber);
    *piOutObjSafe -= OPS_EBA_GetRCSObserver(iObjNumber);
    *piOutObjSafe -= OPS_EBA_GetPOEObserver(iObjNumber);
    *piOutObjSafe -= OPS_EBA_GetLFTObserver(iObjNumber);
}

/*****************************************************************************
  @fn           OPS_EBA_GetOBSObserver

  @brief        the Obstacle Observer,to reduce objsafe value,for filter some
obstacle object

  @description  the Obstacle Observer,to reduce objsafe value,for filter some
obstacle object

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :prob = 0;
  :Calculate MaxThreshold ;
  note
  Take the maximum value of the
  Obj RCS threshold and the
  OPS_EBA_MAX_RCS_THRES_MOVE2STAT
  end note
  if(is MOVE2STAT) then(stopped confidence > 80)
        :Calculate uiObjOBSThresh;
        note:uiObjOBSThresh = 53
  elseif(is uiObjOBSThresh) then(dynamic prop is OBJECT_PROPERTY_STATIONARY)
        :Calculate uiObjOBSThresh;
        note:uiObjOBSThresh = 60
        if(Quality < MIN_PROBABILITY and RCS > MaxThreshold) then(Quality >
0.9999 and RCS > RCS Thresh)
                :Calculate uiObjOBSThresh;
                note:reduce high Quality and high RCS Object
OBSThresh,uiObjOBSThresh -10
        endif
  else(no)
        :Calculate uiObjOBSThresh;
        note:uiObjOBSThresh = 60
  endif
  if (OPSCheckStatObstacleProbability) then(yes)
        :SafeReduce = 0;
  else(no)
        :SafeReduce = OPS_EBA_OBJ_SAFETY_OBS_BIT ;
        note:OPS_EBA_OBJ_SAFETY_OBS_BIT = 32
  endif
  :return SafeReduce;
  end
  @enduml

******************************************************************************/
static ui8_t OPS_EBA_GetOBSObserver(sint8 iObjNumber) {
    ui8_t iObjSafeReduceVal = 0u;
    uint8 uiObjOBSThresh;
    float32 fObjRCSThresh;

    // Get Max Obj RCS Thresh
    fObjRCSThresh = MAX(Envm_INT_OBJ_RCS_TGT_TRESHOLD_UNCOMP(iObjNumber),
                        OPS_EBA_MAX_RCS_THRES_MOVE2STAT);

    // uiStoppedConfidence > 80
    if (Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(iObjNumber)) {
        uiObjOBSThresh = OPS_EBA_OBSPROB_THRES_MOVE;
    } else if (EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber) ==
               OBJECT_PROPERTY_STATIONARY) {
        uiObjOBSThresh = OPS_EBA_OBSPROB_THRES_STAT;

        // Reduce High Quality and High RCS Object OBSThresh
        if ((FctPreselEBAObj[iObjNumber].fEBAObjectQuality >
             OPS_EBA_OBJ_MIN_PROBABILITY) &&
            (Envm_INT_OBJ_RCS(iObjNumber) > fObjRCSThresh)) {
            uiObjOBSThresh -= OPS_EBA_OBSPROB_THRES_DECR_STAT;
        }
    } else {
        // Others DynmaicProperty Object Thresh
        uiObjOBSThresh = OPS_EBA_OBSPROB_THRES_MOVE;
    }

    if (OPSCheckStatObstacleProbability(iObjNumber, uiObjOBSThresh) == TRUE) {
        iObjSafeReduceVal = 0u;
    } else {
        iObjSafeReduceVal = OPS_EBA_OBJ_SAFETY_OBS_BIT;
    }
    return iObjSafeReduceVal;
}

/*****************************************************************************
  @fn            OPSCheckStatObstacleProbability


  @brief         Helper function to check the relevant obstacle probabilities of
an object.

  @description   For moving objects the obstacle probability is checked against
the given threshold.
                                 For a priori stationary objects additional
checkss are done

                                 @li check if tunnel probability below max.
allowed probability
                                 @li check given threshold against merge
guardrail probability and obstacle probability

  @param[in]     iObjNumber:  The index of the object to check
                                 ObjProbThresh: The oobstacle probability
threshold

  @return        Returns TRUE if the object obstacle probability is below the
threshold.

  @todo          -

  @uml
  @startuml
  start
  if (is stationary objec and stoppedconfidence <= 80 )then
        :check = True;
  else(no)
        if(Obstacle Probability > Thresh) then
                :check = True;
        else(no)
                :check = False;
        endif
  endif
  :return check;
  end
  @enduml
******************************************************************************/
static boolean OPSCheckStatObstacleProbability(sint8 iObjNumber,
                                               uint8 uiObjProbThresh) {
    boolean bCheck = FALSE;

    if ((EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber) ==
         OBJECT_PROPERTY_STATIONARY) &&
        (!Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(iObjNumber))) {
        bCheck = TRUE;

        // removed TunnelDetected function and check
        /* Suppress object in tunnel */
        // if in tunnel set bCheck to FALSE,suppress object

        // removed roadside boader detection since em module have already been
        // removed
        // and if no in tunnel
        // if GuardRailMergedObstProb >= uiObjProbThresh, set bCheck to TRUE ,
        // else to FALSE

    } else {
        if (EM_INT_OBJ_OBSTACLE_PROBABILITY(iObjNumber) >= uiObjProbThresh) {
            bCheck = TRUE;
        } else {
            bCheck = FALSE;
        }
    }
    return bCheck;
}

/*****************************************************************************
  @fn           OPS_EBA_GetRCSObserver

  @brief        Object RCS Observer

  @description  check object RCS value to calculate objsafe value

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :get RCSThresh;
  note:get RCSThresh Value
  if(is Stationary) then(yes)
        if(Quality > MIN_PROBABILITY and distX < MAXDIST ) then(Quality > 0.999
and distX < 10)
                :pass;
        else(no)
                :Calculate RCSThresh;
                note:increase Object RCSThresh 10
        endif
  else(no)
        :get VAbsObj;
        note:the STAT2MOVE Object VabsY
        if(     distX <= STAT2MOVE and
         Vabs < STAT2MOVE_SPEED and
         Vx > MINSPEED and Vx < MAXSPEED) then(distX < 25, Vabs < 5, 25 < Vx <
60)
                :Calculate RCSThresh;
                note
                distX < 25m,
                Object VabsX <5m/s,
                Vx between 25-60 KM/S
                increase Object RCSThresh 10
                end note

        elseif(Quality > MIN_PROBABILITY and distX < MINDIST) then(Quality >
0.9999, distX < 10)
                :pass;
        else()
                :Calculate RCSThresh;
                note
                pervious Quality<100
                distX > 10m
                increase Object RCSThresh 5
                end note
        endif
  endif
  if(OPSCheckRCS) then(yes)
        :iObjSafeReduceVal = 0;
  else(no)
        :iObjSafeReduceVal = 64;
  endif
  note: pass the RCS check,return iObjSafeReduceVal to reduce objsafe
  :return prob;
  end
  @enduml
******************************************************************************/
static ui8_t OPS_EBA_GetRCSObserver(sint8 iObjNumber) {
    ui8_t iObjSafeReduceVal = 0;
    float32 fObjVabsX = 0;
    float32 fObjRCSThresh =
        Envm_p_GetPrivObject(iObjNumber)->Legacy.fRCSTargetThresholdUncomp;

    // Stationary Object
    if (EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber) == OBJECT_PROPERTY_STATIONARY) {
        if ((FctPreselEBAObj[iObjNumber].fEBAObjectQuality >
             OPS_EBA_OBJ_MIN_PROBABILITY) &&
            (Envm_INT_GET_Envm_OBJ(iObjNumber).Kinematic.fDistX <=
             OPS_EBA_OBJ_MAX_LOW_DIST)) {
            /* nothing to do - use low RCS-Threshold for safe Objects in low
             * distance */
        } else {
            // increase RCS Threshhold for far and low quality Object
            fObjRCSThresh += OFPS_EBA_RCS_THRES_INCR_STAT;
        }
    } else {
        fObjVabsX = EMTRAFO_f_GetObjSyncEgoMotionVx(
                        Envm_INT_GET_Envm_OBJ(iObjNumber).Kinematic.fDistY) +
                    Envm_INT_GET_Envm_OBJ(iObjNumber).Kinematic.fVrelX;

        if ((Envm_INT_GET_Envm_OBJ(iObjNumber).Kinematic.fDistX <
             OPS_EBA_CRIT_MAX_STAT2MOVE_DIST) &&
            (fObjVabsX < OPS_EBA_CRIT_MAX_STAT2MOVE_OBJ_SPEED) &&
            (EGO_SPEED_X_OBJ_SYNC > OPS_EBA_CRIT_MIN_STAT2MOVE_EGO_SPEED) &&
            (EGO_SPEED_X_OBJ_SYNC < OPS_EBA_CRIT_MAX_STAT2MOVE_EGO_SPEED)) {
            fObjRCSThresh += OPS_EBA_CRIT_MAX_STAT2MOVE_RCS_THRES_INCR;

        } else if ((FctPreselEBAObj[iObjNumber].fEBAObjectQuality >
                    OPS_EBA_OBJ_MIN_PROBABILITY) &&
                   (Envm_INT_GET_Envm_OBJ(iObjNumber).Kinematic.fDistX <=
                    OPS_EBA_OBJ_MAX_LOW_DIST)) {
            // nothing to do
        } else {
            fObjRCSThresh += OPS_EBA_RCS_THRES_INCR_MOVE;
        }
    }

    if (OPSCheckRCS(iObjNumber, fObjRCSThresh) == TRUE) {
        iObjSafeReduceVal = 0u;
    } else {
        iObjSafeReduceVal = OPS_EBA_OBJ_SAFETY_RCS_BIT;
    }
    return iObjSafeReduceVal;
}

/*****************************************************************************
  @fn           OPSCheckRCS

  @brief        Check Object RCS Thresh

  @description  Check Object RCS Thresh

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  if (Object RCS >= Thresh) then(yes)
        :check = True;
  else(no)
        :check = False;
  endif
  :return check;
  end
  @enduml
******************************************************************************/
static boolean OPSCheckRCS(sint8 iObjNumber, float32 fObjRCSThresh) {
    boolean bCheck = FALSE;

    if (Envm_INT_GET_Envm_OBJ(iObjNumber).SensorSpecific.fRCS >=
        fObjRCSThresh) {
        bCheck = TRUE;
    } else {
        bCheck = FALSE;
    }
    return bCheck;
}

/*****************************************************************************
  @fn           OPS_EBA_GetPOEObserver

  @brief        Object proability of exist Observer

  @description  The check item for object POE value

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :get POEThresh;
  note:get POEThresh;
  if(is Stationary and Move2Stat > 0) then(stationary,stopped confidence > 0)
        if(POEThresh < MINTHRESH)
                :Calculate POEThresh;
                note
                the move2stat object
                POEThresh < 15
                set POEThresh = 0
                end note
        else(no)
                :Calculate POEThresh;
                note:reduce POEThresh 15
        endif
  endif

  if(is OPSCheckPOE) then(yes)
        :iObjSafeReduceVal = 0;
  else(no)
        :iObjSafeReduceVal = 128;
  endif
  note
  use OPSCheckPOE function check
  pass return 0
  else return 128 to reduce objsafe
  end note
  :return prob;
  end
  @enduml
******************************************************************************/
static ui8_t OPS_EBA_GetPOEObserver(sint8 iObjNumber) {
    ui8_t iObjSafeReduceVal = 0;
    ui8_t uiObjPOEThresh;

    uiObjPOEThresh = OPSGetPOEThreshold(iObjNumber);

    if ((EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber) ==
         OBJECT_PROPERTY_STATIONARY) &&
        (Envm_INT_GET_Envm_OBJ(iObjNumber).Attributes.uiStoppedConfidence >
         0)) {
        if (uiObjPOEThresh < OPS_EBA_STOPPED_POE_THRES_DECR) {
            uiObjPOEThresh = 0;
        } else {
            uiObjPOEThresh -= OPS_EBA_STOPPED_POE_THRES_DECR;
        }
    }

    if (OPSCheckPOE(iObjNumber, uiObjPOEThresh) == TRUE) {
        iObjSafeReduceVal = 0;
    } else {
        iObjSafeReduceVal = OPS_EBA_OBJ_SAFETY_POE_BIT;
    }
    return iObjSafeReduceVal;
}

/*****************************************************************************
  @fn           OPSCheckPOE

  @brief        Check POE Threshold value

  @description  Check POE Threshold value

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :check = False;
  if (Object ProbabilityOfExistence >= Thresh * 0.01) then(yes)
        :check = True;
  endif
  :return check;
  end
  @enduml

******************************************************************************/
static boolean OPSCheckPOE(sint8 iObjNumber, uint8 uiObjPOEThresh) {
    boolean bCheck = FALSE;

    if (Envm_INT_GET_Envm_OBJ(iObjNumber).Qualifiers.fProbabilityOfExistence >=
        ((float32)uiObjPOEThresh * 0.01F)) {
        bCheck = TRUE;
    }

    return bCheck;
}

/*****************************************************************************
  @fn           OPSGetPOEThreshold

  @brief        Get object POE Observer check Thresh

  @description  Get object POE Observer Thresh acorrding to lifetime

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  note:return poe thresh value
  if (lifetime > 0 and <= 16) then(yes)
        :POE Thresh = POEThreshArray[lifetime-1];
        note
        difference lifetime has difference threshold value
        the threshold value will increase over time
        end note
  else if(lifetime >16) then(yes)
        :POE Thresh = POEThreshArray[15];
        note:max thresh = 99
  else(no)
        :POE Thresh = POEThreshArray[15];
        note:use high threshold value for new object
  endif
  end
  @enduml
******************************************************************************/
static uint8 OPSGetPOEThreshold(sint8 iObjNumber) {
    uint8 uiObjPOEThresh;
    uint16 uiObjLifeTime = Envm_INT_GET_Envm_OBJ(iObjNumber).Legacy.uiLifeTime;

    // choose POEThresh according as Object LifeTime
    if ((uiObjLifeTime > 0) && (uiObjLifeTime <= OPS_EBA_NUMBER_OF_POE_THRES)) {
        uiObjPOEThresh = OPS_EBA_POE_THRES[uiObjLifeTime - 1];
    } else if (uiObjLifeTime > OPS_EBA_NUMBER_OF_POE_THRES) {
        uiObjPOEThresh = OPS_EBA_POE_THRES[OPS_EBA_NUMBER_OF_POE_THRES - 1];
    } else {
        uiObjPOEThresh = OPS_EBA_POE_THRES[OPS_EBA_NUMBER_OF_POE_THRES - 1];
    }

    return uiObjPOEThresh;
}

/*****************************************************************************
  @fn           OPS_EBA_GetLFTObserver

  @brief        Object LifeTime Observer

  @description  Check Object LifeTime

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :iObjSafeReduceVal = 0;
  :bCheck = FALSE;
  if(is Moving) then()
        if (LifeTime > MINMOVE) then(LifeTime > 13)
        :bCheck = TRUE;
        note:Moving Object LifeTime > 13
        endif
  elseif( is Stationary) then()
        if (LifeTime > MINSTAT) then(LifeTime > 10)
        :bCheck = TRUE;
        note:Moving Object LifeTime > 10
        endif
  elseif( is Oncoming) then()
        if (LifeTime > MINONCOMING) then(LifeTime > 6)
        :bCheck = TRUE;
        note:Moving Object LifeTime > 6
        endif
  else()
        :bCheck = FALSE;
  endif

  if(bCheck == TRUE) then(YES)
        :iObjSafeReduceVal = 0;
  else(no)
        :iObjSafeReduceVal = OPS_EBA_OBJ_SAFETY_LFT_BIT;
        note:iObjSafeReduceVal = 16;
  endif
  end
  @enduml

******************************************************************************/
static ui8_t OPS_EBA_GetLFTObserver(sint8 iObjNumber) {
    ui8_t iObjSafeReduceVal = 0;
    boolean bCheck = FALSE;
    uint16 uiObjLifeTime = Envm_INT_GET_Envm_OBJ(iObjNumber).Legacy.uiLifeTime;

    // check Object lifetime according to Object dynmaic property
    switch (EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber)) {
        case OBJECT_PROPERTY_MOVING:
            if (uiObjLifeTime >= OPS_EBA_OBJ_MIN_LIFETIME_MOVE) {
                bCheck = TRUE;
            }
            break;

        case OBJECT_PROPERTY_STATIONARY:
            if (uiObjLifeTime >= OPS_EBA_OBJ_MIN_LIFETIME_STAT) {
                bCheck = TRUE;
            }
            break;

        case OBJECT_PROPERTY_ONCOMING:
            if (uiObjLifeTime >= OPS_EBA_OBJ_MIN_LIFETIME_SAFE) {
                bCheck = TRUE;
            }
            break;

        default:
            bCheck = FALSE;
            break;
    }

    if (bCheck == TRUE) {
        iObjSafeReduceVal = 0;
    } else {
        iObjSafeReduceVal = OPS_EBA_OBJ_SAFETY_LFT_BIT;
    }
    return iObjSafeReduceVal;
}

/*****************************************************************************
  @fn           OPSUpdateNonAbsObservers

  @brief        Update Non Absolute Observer

  @description  Update Non Absolute Observer

  @param[in]    -

  @return       -

  @pre          -
  @uml
  @startuml
  start
  :piOutThresh = OPS_EBA_UpdateNonAbsObservers(ObjNumber, piOutThresh,
OPS_EBA_GetOncomingObserver);
  note:to calculate Oncoming Object reduce Thresh
  :piOutThresh = OPS_EBA_UpdateNonAbsObservers(ObjNumber, piOutThresh,
OPS_EBA_GetWideObserver);
  note:to calculate Wide Object reduce Thresh
  stop
  @enduml
******************************************************************************/
static void OPSUpdateNonAbsObservers(sint8 iObjNumber, uint8* piOutThresh) {
    // update all NonAbsObserver value
    *piOutThresh = OPS_EBA_UpdateNonAbsObservers(iObjNumber, *piOutThresh,
                                                 OPS_EBA_GetOncomingObserver);
    *piOutThresh = OPS_EBA_UpdateNonAbsObservers(iObjNumber, *piOutThresh,
                                                 OPS_EBA_GetWideObserver);
}

/*****************************************************************************
  @fn           OPS_EBA_UpdateNonAbsObservers

  @brief        Calculate Quality Thresh and Fix Point Scale

  @description  Calculate Quality Thresh and Fix Point Scale

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  :utemp;
  :get iCurrThresh;
  note
  use pointer function
  pOPS_EBA_GetObserver_Func
  to get Observer value
  end note
  :calculate utemp;
  note:utemp = iCurrThresh * iPerviousThresh
  :round utemp value;
  note
  utemp = (utemp + 100 / 2) / 100
  to round value
  end note
  if(utemp > 100) then()
        :utemp = 100;
  endif
  note:to limit utemp value max 100
  end
  @enduml
******************************************************************************/
static uint8 OPS_EBA_UpdateNonAbsObservers(
    sint8 iObjNumber,
    uint8 iPerviousThresh,
    uint8 (*pOPS_EBA_GetObserver_Func)(sint8)) {
    uint32 utemp;
    uint8 iCurrThresh;

    // get current Observer Thresh
    iCurrThresh = pOPS_EBA_GetObserver_Func(iObjNumber);
    utemp = (uint32)iCurrThresh * (uint32)iPerviousThresh;

    // fixed point scaling,and round the calculated value
    utemp = (utemp + Percentage_max / 2u) / Percentage_max;

    if (utemp > Percentage_max) {
        utemp = Percentage_max;
    }
    return (uint8)utemp;
}
/*****************************************************************************
  @fn           OPS_EBA_GetOncomingObserver

  @brief        Oncoming object Observer

  @description  Oncoming object Observer

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :bCheck = FALSE;
  if(is Oncoming and DistYStd > REDUCE_ONCOMING_OBJ_QUAL) then(DistYStd > 1)
        :bCheck = TRUE;
        note
        Oncoming and DistYStd > 1 Object
        pass the check
        end note
  endif
  if(bCheck == TRUE)
        :iNonObserverProb = 80;
  else(no)
        :iNonObserverProb = 100;
  endif
  :return iNonObserverProb;
  end
  @enduml

******************************************************************************/
static uint8 OPS_EBA_GetOncomingObserver(sint8 iObjNumber) {
    uint8 iNonObserverProb = 0;
    boolean bCheck = FALSE;

    // Reduce Quality for oncoming and high DistYStd Object
    if ((EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber) == OBJECT_PROPERTY_ONCOMING) &&
        (Envm_INT_OBJ_LAT_DISPLACEnvmENT_VAR(iObjNumber) >
         OPS_EBA_LAT_VAR_FOR_REDUCE_ONCOMING_OBJ_QUAL)) {
        bCheck = TRUE;
    }

    if (bCheck == TRUE) {
        iNonObserverProb = OPS_EBA_OBJ_NON_OBSERVER_ONCOMING_PROB;
    } else {
        iNonObserverProb = 100;
    }
    return iNonObserverProb;
}

/*****************************************************************************
  @fn           OPS_EBA_GetWideObserver

  @brief        Wide object Observer

  @description  Wide object Observer

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :bCheck = FALSE;
  if(is Wide and is Stationary and isn't Move2Stat) then(stopped confidence ==
0)
        :bCheck  = TRUE;
        note
        is wide
        stationary
        and isn't moving to stationary object
        pass the check
        end note
  endif
  if(bCheck = TRUE)then()
        :iNonObserverProb = 0;
  else(no)
        :iNonObserverProb = 100;
  endif
  :return iNonObserverProb;
  end
  @enduml

******************************************************************************/
static uint8 OPS_EBA_GetWideObserver(sint8 iObjNumber) {
    uint8 iNonObserverProb = 0;
    boolean bCheck = FALSE;

    // Reduce Quality for Wide Class Object
    if ((EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber) ==
         OBJECT_PROPERTY_STATIONARY) &&
        (Envm_INT_GET_Envm_OBJ(iObjNumber).Attributes.eClassification ==
         OBJCLASS_WIDE) &&
        (EM_INT_OBJ_MOVING_TO_STATIONARY(iObjNumber) == FALSE)) {
        bCheck = TRUE;
    }

    if (bCheck == TRUE) {
        iNonObserverProb = OPS_EBA_OBJ_NON_OBSERVER_WIDE_PROB;
    } else {
        iNonObserverProb = 100;
    }
    return iNonObserverProb;
}

/*****************************************************************************
  @fn           OPSTimeFilter

  @brief        OPS object quality fliter

  @description  OPS object quality fliter

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  if (isn't deleted obj) then(yes)
        :calculate fThresh;
        note:fThresh = 1 - (iThresh/100)
        :calculate fLimit ;
        note:fLimit = iLimit/100 ,covert to %

        if(ObjectQuality < MinQuality) then(yes,Quilty<0.48)
                :fObjQuality = Min(1.0,(ObjSafe / MaxSafe));
                note:calculate Observer result pass percent
                :fObjQuality = fObjQuality * MinQuality;
                note:calculate fObjQuality
        else(no)
                if(ObjSafe == Max_Safe) then(yes,MaxSafe = 255)
                        if(ObjectQuality < MinProability) then(min=0.9999)
                                :fGain = ((1.0 - ObjectQuality) * 0.5);
                                note:P control to incr ObjectQuality
                        else(no)
                                :fGain = 0;
                        endif
                else(no)
                        :fGain = -1.0 * ( 1.0 - ObjectQuality);
                endif
        :fObjQuality = MAX((ObjectQuality + fGain) - fThresh, 0);
        :fObjQuality = MIN(ObjectQuality, fLimit);
        note:Limit fObjQuality to a certain range 0-1
        endif
  else(no)
        :fObjQuality = 0;
  endif
  :return fObjQuality;
  stop
  @enduml
******************************************************************************/
static float32 OPSTimeFilter(sint8 iObjNumber,
                             uint8 iThresh,
                             uint8 iLimit,
                             ui8_t uiObjSafe) {
    float32 fLimit;
    float32 fObjQuality = 0.0f;
    float32 fGain = 0.0f;
    float32 fThresh = 0.0f;

    if (!EM_INT_OBJ_IS_DELETED(iObjNumber)) {
        // calculate Thresh and Limit
        fThresh = 1.0f - (float32)iThresh / 100.0f;
        fLimit = (float32)iLimit / 100.0f;

        // Calculate quality according to the observe result
        if (FctPreselEBAObj[iObjNumber].fEBAObjectQuality <
            OPS_EBA_OBJ_MIN_QUALITY) {
            fObjQuality = MIN(
                ((float32)uiObjSafe / (float32)OPS_EBA_OBJ_MAX_SAFETY), 1.0f);
            fObjQuality = fObjQuality * OPS_EBA_OBJ_MIN_QUALITY;
        } else {
            // calculate gain according to AbsObserver result
            // increase quality value when pass all observer check
            if (uiObjSafe == OPS_EBA_OBJ_MAX_SAFETY) {
                if (FctPreselEBAObj[iObjNumber].fEBAObjectQuality <
                    OPS_EBA_OBJ_MIN_PROBABILITY) {
                    fGain = (((1.0f) -
                              FctPreselEBAObj[iObjNumber].fEBAObjectQuality) *
                             0.5f);
                } else {
                    fGain = 0.0f;
                }

            } else {
                // decrease quality value when observer doesn't pass
                fGain = (-1.0f) *
                        (1.0f - FctPreselEBAObj[iObjNumber].fEBAObjectQuality);
            }
            fObjQuality = MAX((FctPreselEBAObj[iObjNumber].fEBAObjectQuality +
                               fGain - fThresh),
                              0.0f);
            fObjQuality = MIN(fObjQuality, fLimit);
        }
    } else {
        fObjQuality = 0.0f;
    }
    return fObjQuality;
}

/*****************************************************************************
  @fn           OPSEBAPreSelCustomFct

  @brief        Modifies EBA preselection quality by custom requirements

  @description  Modifies EBA preselection quality by custom requirements

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  if(GetPermission < LV1)then(yes)
        :Set eEbaInhibitionMask Bit with L2 and L3;
        note:set all object Permission to LV2-LV3
  endif
  stop
  @enduml

******************************************************************************/
void OPSEBAPreSelCustomFct(sint8 iObjNumber) {
    if (OPSEBA_MOSA_GetObjPermission(iObjNumber) <
        OPSEBA_MOSA_PERMISSION_LVL1) {
        SET_BIT(EM_INT_OBJ_GET_EBA_INHIBITIONMASK(iObjNumber),
                (FPS_EBA_INH_BRAKE_L2 | FPS_EBA_INH_BRAKE_L3));
    }
}

/*****************************************************************************
  @fn           OPSDynPropHistoryUpdate

  @brief        Update history dynamic property

  @description  Update history dynamic property

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  if (object is stationary) then(yes)
        :bWasStationary =TRUE;
        note:update dynamci history for stationary object
        if(object is moving to stationary) then(yes)
                :bWasStopped = TRUE;
        endif

  else if (object isn't stationary) then(yes)
        if (object is moving) then(yes)
                :bWasMoving = TRUE;
                note:update dynamci history for moving object
                :bWasMovingSafe = TRUE;
                :bWasOncoming = FALSE;
        endif
        if (object is Oncoming) then(yes)
                :bWasOncoming = TRUE;
                note:update dynamci history for Oncoming object
        endif
  endif
  if(bIsObjectCrossing pass) then(yes)
        :bWasCrossing = TRUE;
        note:crossing flag
  else(no)
        :bWasCrossing = FLASE;
  endif
  end
  @enduml

******************************************************************************/
static void OPSDynPropHistoryUpdate(sint8 iObjNumber,
                                    boolean bIsObjectCrossing) {
    FctPreselEBAObj_t* const pFctPreselObj = &FctPreselEBAObj[iObjNumber];
    Objects_t* const pObjEnvmData = Envm_p_GetPrivObject(iObjNumber);

    // stationary object
    if (pObjEnvmData->Attributes.eDynamicProperty ==
        OBJECT_PROPERTY_STATIONARY) {
        pFctPreselObj->sDynPropHistory.bWasStationary = TRUE;
        if (Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(iObjNumber) == TRUE) {
            pFctPreselObj->sDynPropHistory.bWasStopped = TRUE;
        }
    }
    // oncoming or moving object
    else {
        // check whether object is moving
        if (pObjEnvmData->Attributes.eDynamicProperty ==
            OBJECT_PROPERTY_MOVING) {
            pFctPreselObj->sDynPropHistory.bWasMoving = TRUE;
            pFctPreselObj->sDynPropHistory.bWasMovingSafe = TRUE;
            // reset the moving object oncoming flag
            pFctPreselObj->sDynPropHistory.bWasOncoming = FALSE;
        }

        // check whether object is oncoming
        if (pObjEnvmData->Attributes.eDynamicProperty ==
            OBJECT_PROPERTY_ONCOMING) {
            pFctPreselObj->sDynPropHistory.bWasOncoming = TRUE;
        }
    }

    // crossing object
    if (bIsObjectCrossing == TRUE) {
        pFctPreselObj->sDynPropHistory.bWasCrossing = TRUE;
    } else {
        pFctPreselObj->sDynPropHistory.bWasCrossing = FALSE;
    }
}

/*****************************************************************************
  @fn           OPSEBAPreselPostProc

  @brief        OPS Presel Post Process

  @description  OPS Presel Post Process

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
void OPSEBAPreselPostProc(void) {
    // TODO
}

/* ************************************************************************* */
/*  OPS_UNIT_TEST                                                            */
/* ************************************************************************* */
#if OPS_UNIT_TEST_SWITCH == TRUE
uint8 UnitTest_OPS_EBA_GetRCSObserver(sint8 iObjNumber);
uint8 UnitTest_OPS_EBA_GetOBSObserver(sint8 iObjNumber);
uint8 UnitTest_OPS_EBA_GetPOEObserver(sint8 iObjNumber);
uint8 UnitTest_OPS_EBA_GetLFTObserver(sint8 iObjNumber);
uint8 UnitTest_OPS_EBA_GetOncomingObserver(sint8 iObjNumber);
uint8 UnitTest_OPS_EBA_GetWideObserver(sint8 iObjNumber);
float32 UnitTest_OPSTimeFilter(sint8 iObjNumber,
                               uint8 iThresh,
                               uint8 iLimit,
                               ui8_t uiObjSafe);

uint8 UnitTest_OPS_EBA_GetRCSObserver(sint8 iObjNumber) {
    return OPS_EBA_GetRCSObserver(iObjNumber);
}

uint8 UnitTest_OPS_EBA_GetOBSObserver(sint8 iObjNumber) {
    return OPS_EBA_GetOBSObserver(iObjNumber);
}

uint8 UnitTest_OPS_EBA_GetPOEObserver(sint8 iObjNumber) {
    return OPS_EBA_GetPOEObserver(iObjNumber);
}

uint8 UnitTest_OPS_EBA_GetLFTObserver(sint8 iObjNumber) {
    return OPS_EBA_GetLFTObserver(iObjNumber);
}

uint8 UnitTest_OPS_EBA_GetOncomingObserver(sint8 iObjNumber) {
    return OPS_EBA_GetOncomingObserver(iObjNumber);
}

uint8 UnitTest_OPS_EBA_GetWideObserver(sint8 iObjNumber) {
    return OPS_EBA_GetWideObserver(iObjNumber);
}

float32 UnitTest_OPSTimeFilter(sint8 iObjNumber,
                               uint8 iThresh,
                               uint8 iLimit,
                               ui8_t uiObjSafe) {
    return OPSTimeFilter(iObjNumber, iThresh, iLimit, uiObjSafe);
}

void testOPSSetEBAHypCat(sint8 iObjNumber, boolean bIsObjCrossing);
void testOPSSetEBAHypCat(sint8 iObjNumber, boolean bIsObjCrossing) {
    uint8 Thresh = 100;
    OPSUpdateNonAbsObservers(iObjNumber, &Thresh);
    OPSSetEBAHypCat(iObjNumber, bIsObjCrossing);
    OPS_EBA_GetOBSObserver(iObjNumber);
    OPS_EBA_GetRCSObserver(iObjNumber);
    OPS_EBA_GetPOEObserver(iObjNumber);
    OPS_EBA_GetLFTObserver(iObjNumber);
    OPSEBA_MOSA_Process(iObjNumber);
}
#endif

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */