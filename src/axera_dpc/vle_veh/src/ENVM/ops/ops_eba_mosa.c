/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "ops_eba_mosa.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
// moving safe intermediate variable
static OPSEBA_MOSA_Data_t sOPSEBA_MOSA_Data;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/************************************************************************/
/*INTERMEDIATE FUNCTIONS                                                   */
/************************************************************************/
/*************************************************************************************************************************
  Functionname:    OPSEBA_MOSA_GetDistX */
static float32 OPSEBA_MOSA_GetDistX(EM_t_ObjNumber iObj) {
    return EM_INT_OBJ_LONG_DISPLACEMENT(iObj);
}

/*************************************************************************************************************************
  Functionname:    OPSEBA_MOSA_GetVrelX */
static float32 OPSEBA_MOSA_GetVrelX(EM_t_ObjNumber iObj) {
    return EM_INT_OBJ_LONG_VREL(iObj);
}

/*************************************************************************************************************************
  Functionname:    OPSEBA_MOSA_GetVabsX */
static float32 OPSEBA_MOSA_GetVabsX(EM_t_ObjNumber iObj) {
    return (OPSEBA_MOSA_GetVrelX(iObj) +
            EMTRAFO_f_GetObjSyncEgoMotionVx(OPSEBA_MOSA_GetDistY(iObj)));
}

/*************************************************************************************************************************
  Functionname:    OPSEBA_MOSA_GetDistY */
static float32 OPSEBA_MOSA_GetDistY(EM_t_ObjNumber iObj) {
    return Envm_INT_OBJ_LAT_DISPLACEnvmENT(iObj);
}
/*************************************************************************************************************************
  Functionname:    OPSEBA_MOSA_GetVrelY */
static float32 OPSEBA_MOSA_GetVrelY(EM_t_ObjNumber iObj) {
    return Envm_INT_OBJ_LAT_VREL(iObj);
}
/*************************************************************************************************************************
  Functionname:    OPSEBA_MOSA_GetVabsY */
static float32 OPSEBA_MOSA_GetVabsY(EM_t_ObjNumber iObj) {
    return (OPSEBA_MOSA_GetVrelY(iObj) +
            EMTRAFO_f_GetObjSyncEgoMotionVy(OPSEBA_MOSA_GetDistX(iObj)));
}

/*************************************************************************************************************************
  Functionname:    OPSEBA_MOSA_TrafFloatToMovDist */
static iOPSEBA_MOSA_MovDist_t OPSEBA_MOSA_TrafFloatToMovDist(
    float32 fMovement) {
    /* Transform the float value to sint32 */
    float fTmp = OPSEBA_MOSA_Float2MovDist(fMovement);
    sint32 iIntMovement = ROUND_TO_INT(fTmp);

    /* Limit result to avoid overflow */
    iIntMovement =
        TUE_CML_MinMax(-(sint32)OPSEBA_MOSA_MOVDIST_MAX,
                       (sint32)OPSEBA_MOSA_MOVDIST_MAX, iIntMovement);

    /* transform to save format*/
    return (iOPSEBA_MOSA_MovDist_t)iIntMovement;
}

/*************************************************************************************************************************
  Functionname:    OPSEBA_MOSA_TrafFloatToMovDistX */
static iOPSEBA_MOSA_MovDist_t OPSEBA_MOSA_TrafFloatToMovDistX(
    float32 fMovement) {
    /* Transform the float value to sint32 */
    float fTmp = OPSEBA_MOSA_Float2MovDist(fMovement);
    sint32 iIntMovement = ROUND_TO_INT(fTmp);
    iIntMovement -= (sint32)OPSEBA_MOSA_MOVDIST_MAX;

    /* Limit result to avoid overflow */
    iIntMovement =
        TUE_CML_MinMax(-(sint32)OPSEBA_MOSA_MOVDIST_MAX,
                       (sint32)OPSEBA_MOSA_MOVDIST_MAX, iIntMovement);

    /* transform to save format*/
    return (iOPSEBA_MOSA_MovDist_t)iIntMovement;
}

/*************************************************************************************************************************
  Functionname:    OPSEBA_MOSA_Float2MovDist */
static float32 OPSEBA_MOSA_Float2MovDist(float32 fMovDist) {
    return (fMovDist * OPSEBA_MOSA_CONV_FACTOR);
}

/*************************************************************************************************************************
  Functionname:    OPSEBA_MOSA_MovDist2Float */

static float32 OPSEBA_MOSA_MovDist2Float(sint32 iMovDist) {
    return ((float32)(iMovDist) * (1.0F / OPSEBA_MOSA_CONV_FACTOR));
}
/*****************************************************************************
  @fn           OPSEBA_MOSA_TrafMovDistXToFloat

  @brief        transform integer distX to float distX

  @description  transform integer distX to float distX

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
static float32 OPSEBA_MOSA_TrafMovDistXToFloat(
    iOPSEBA_MOSA_MovDist_t iMovement) {
    /* transform to physical unit [m] */
    sint32 iMovementShifted =
        (sint32)iMovement + (sint32)OPSEBA_MOSA_MOVDIST_MAX;

    return OPSEBA_MOSA_MovDist2Float(iMovementShifted);
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_TrafMovDistToFloat

  @brief        transform integer dist to float dist

  @description  transform integer dist to float dist

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
static float32 OPSEBA_MOSA_TrafMovDistToFloat(
    iOPSEBA_MOSA_MovDist_t iMovement) {
    /* transform to physical unit [m] */
    return OPSEBA_MOSA_MovDist2Float(iMovement);
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_GetMeasHistory

  @brief        Get object history measurement state

  @description  Get object history measurement state

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
static uint8 OPSEBA_MOSA_GetMeasHistory(EM_t_ObjNumber iObjNumber) {
    return Envm_u_GetTgtConfirmDensity(iObjNumber);
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_IsObjStationary

  @brief        Get whether the object is stationary

  @description  Get whether the object is stationary

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
static boolean OPSEBA_MOSA_IsObjStationary(EM_t_ObjNumber iObjNumber) {
    // get the object dynamic property and check if the stationary object
    return (EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber) ==
            OBJECT_PROPERTY_STATIONARY);
}

/************************************************************************/
/* FUNCTIONS                                                   */
/************************************************************************/
/*****************************************************************************
  @fn           OPSMOSAInit

  @brief        OPS MOVING SAFE INIT

  @description  Loop all object ,init object moving safe flags and CycleTime

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  while (iObjNumber < 25)
        :OPSEBA_MOSA_InitObj();
        note:init obj moving safe intermediate variable
        :iObjNumber++;
  end while
  while (Index < 8)
        :auCycleTime[Index] = 0;
        note:Initializes the array that save the object cycle time
  end while
  :uNumOfCycles = 0;
  note:set use array number to zero
  stop
  @enduml

******************************************************************************/

void OPSMOSAInit(void) {
    EM_t_ObjNumber iObjNumber;

    for (iObjNumber = (Envm_NR_PRIVOBJECTS - 1); iObjNumber >= 0;
         iObjNumber--) {
        // Init moving safe check structure
        OPSEBA_MOSA_InitObj(iObjNumber);
    }

    for (iObjNumber = (OPSEBA_MOSA_VELO_WAY_PLAUSI_AVERAGING_CYCLES - 1);
         iObjNumber >= 0; iObjNumber--) {
        // Init CycleTime array
        sOPSEBA_MOSA_Data.sCycleTime.auCycleTime[iObjNumber] = 0u;
    }
    sOPSEBA_MOSA_Data.sCycleTime.uNumOfCycles = 0u;
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_InitObj

  @brief         reset moving safe related data

  @description   reset moving safe related data

  @param[in]    -

  @return       -

  @pre          -

  @uml

  @startuml
  start
  note:OPSEBA_MOSA_InitObj
  if ((iObjNumber >= 0)&&
  (iObjNumber < Envm_NR_PRIVOBJECTS)) then(yes)
        :pointer to sFPSEBA_MOSA_Data;
        note:create pointer to structure

  partition init_values{
        :init aiDistXLast[iObjNumber]
        note:Initializes the longitudinal position at the previous time
        :init aiDistXMoved[iObjNumber];
        note:Initializes the absolute longitudinal movement distance
        :init aiDistYLast[iObjNumber];
        note:Initializes the lateral position at the previous time
        :aiDistYMoved[iObjNumber];
        note:Initializes the absolute lateral movement distance
  }
        :init asObjQual[iObjNumber].MovObjQual;
        note:init MovObjQual to OPSEBA_MOSA_PERMISSION_NONE

  partition init_flag_to_FALSE{
        :init bIsPedCandidate;
        note:Whether to be a pedestrian candidate
        :init bMovedFarEnoughLvl1;
        note:the distance of move reach level 1
        :init bMovedFarEnoughLvl2;
        note:the distance of move reach level 2
        :init bWasFastEnough;
        note:High speed moving object
        :init bWasObsWhenStat;
        note:High quality obstacle
  }

  partition init_plausiabliztion_to_0 {
        :auRelDiff[iObj];
        note
        the distance from pos of the object and the
        distance caculated from the average velocity of the object;
        end note

        :aiDistXMovAverage[iObjNumber];
        note:Average longitudinal move distance
        :aiVabsXAverage[iObjNumber];
        note:Longitudinal average absolute velocity
        :aiDistYMovAverage[iObjNumber];
        note:Average lateral move distance
        :aiVabsYAverage[iObjNumber];
        note:lateral average absolute velocity
  }
  endif
  stop
  @enduml

******************************************************************************/
void OPSEBA_MOSA_InitObj(EM_t_ObjNumber iObjNumber) {
    OPSEBA_MOSA_Data_t* const pOPSEBA_MOSA_Data = &sOPSEBA_MOSA_Data;

    if ((iObjNumber >= 0) && (iObjNumber < Envm_NR_PRIVOBJECTS)) {
        /* Init values */
        pOPSEBA_MOSA_Data->aiDistXLast[iObjNumber] =
            OPSEBA_MOSA_TrafFloatToMovDistX(OPSEBA_MOSA_GetDistX(iObjNumber));
        pOPSEBA_MOSA_Data->aiDistXMoved[iObjNumber] =
            OPSEBA_MOSA_TrafFloatToMovDist((float32)0);
        pOPSEBA_MOSA_Data->aiDistYLast[iObjNumber] =
            OPSEBA_MOSA_TrafFloatToMovDist(OPSEBA_MOSA_GetDistY(iObjNumber));
        pOPSEBA_MOSA_Data->aiDistYMoved[iObjNumber] =
            OPSEBA_MOSA_TrafFloatToMovDist((float32)0);

        /* Init FusiObjQual for moving objects with the most restrictive state
         */
        pOPSEBA_MOSA_Data->asObjQual[iObjNumber].MovObjQual =
            OPSEBA_MOSA_PERMISSION_NONE;

        /* Initialize flags */
        pOPSEBA_MOSA_Data->asObjQual[iObjNumber].bIsPedCandidate = FALSE;
        pOPSEBA_MOSA_Data->asObjQual[iObjNumber].bMovedFarEnoughLvl1 = FALSE;
        pOPSEBA_MOSA_Data->asObjQual[iObjNumber].bMovedFarEnoughLvl2 = FALSE;
        pOPSEBA_MOSA_Data->asObjQual[iObjNumber].bWasFastEnough = FALSE;
        pOPSEBA_MOSA_Data->asObjQual[iObjNumber].bWasObsWhenStat = FALSE;

        pOPSEBA_MOSA_Data->auRelDiff[iObjNumber] = 0u;
        pOPSEBA_MOSA_Data->aiDistXMovAverage[iObjNumber] =
            OPSEBA_MOSA_TrafFloatToMovDist((float32)0);
        pOPSEBA_MOSA_Data->aiVabsXAverage[iObjNumber] =
            OPSEBA_MOSA_TrafFloatToMovDist(OPSEBA_MOSA_GetVabsX(iObjNumber));

        pOPSEBA_MOSA_Data->aiDistYMovAverage[iObjNumber] =
            OPSEBA_MOSA_TrafFloatToMovDist((float32)0);
        pOPSEBA_MOSA_Data->aiVabsYAverage[iObjNumber] =
            OPSEBA_MOSA_TrafFloatToMovDist(OPSEBA_MOSA_GetVabsY(iObjNumber));

    } else {
        //__EM_ASSERT(FALSE);
    }
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_PreProcess

  @brief        to init something MOSA variables

  @description  OPSEBA_MOSA_UpdateCycleTime

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/

void OPSEBA_MOSA_PreProcess(void) {
    // save the object cycle time for the last 8 cycle use for UpdateVeloWayFlag
    OPSEBA_MOSA_UpdateCycleTime();
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_UpdateCycleTime

  @brief        Update the array used to save cycletime

  @description  Update the array used to save cycletime base on
OPSEBA_MOSA_VELO_WAY_PLAUSI_AVERAGING_CYCLES

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :iCycle;
  note:iCycle = OPSEBA_MOSA_VELO_WAY_PLAUSI_AVERAGING_CYCLES-1
  :pointer pCycleTime;
  note:create pointer to sOPSEBA_MOSA_Data.sCycleTime
  :calculate fCycleTimeMS;
  note:get current cycle pass time by EM_f_GetCycleTime() * C_MILLISEC_SEC
  while (iCycle > 0)
        :shift array to save current cycletime;
        note
        pCycleTime->auCycleTime[iCycle] =
        = pCycleTime->auCycleTime[iCycle--]
        end note
        :iCycle--;
  end while
  :set auCycleTime[0];
  note
  write fCycleTimeMS to auCycleTime[0]
  and Limit max number to OPSEBA_MOSA_VELO_WAY_UINT8_MAX(255)
  finally need to convert (uint8)
  end note
  :update pCycleTime->uNumOfCycles;
  note
  uNumOfCycles++
  and Limit uNumOfCycles max to
  OPSEBA_MOSA_VELO_WAY_PLAUSI_AVERAGING_CYCLES(8)
  end note
  end
  @enduml

******************************************************************************/
void OPSEBA_MOSA_UpdateCycleTime(void) {
    sint32 iCycle;
    OPSEBA_MOSA_CycleTime_t* pCycleTime = &sOPSEBA_MOSA_Data.sCycleTime;
    float32 fCycleTimeMS =
        EM_f_GetCycleTime() * C_MILLISEC_SEC;  // GetCycleTime unit (S)

    // shift the cycletime array
    for (iCycle = OPSEBA_MOSA_VELO_WAY_PLAUSI_AVERAGING_CYCLES - 1; iCycle > 0;
         iCycle--) {
        pCycleTime->auCycleTime[iCycle] = pCycleTime->auCycleTime[iCycle - 1];
    }

    /* Add new value in milliseconds*/
    pCycleTime->auCycleTime[0] =
        (uint8)MIN(OPSEBA_MOSA_VELO_WAY_UINT8_MAX, fCycleTimeMS);

    /* Update number of available values */
    pCycleTime->uNumOfCycles++;
    pCycleTime->uNumOfCycles = MIN(
        pCycleTime->uNumOfCycles, OPSEBA_MOSA_VELO_WAY_PLAUSI_AVERAGING_CYCLES);
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_Process

  @brief        Calculates the object moving safe permission level

  @description  Calculates the object moving safe permission level by some check
flags

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :init current cycle object check result to FALSE;
  note:Initializes the mosa structure
  if (OPSEBA_MOSA_GetObjLifeCycles(iObjNumber) <= 1) then(yes)
        :OPSEBA_MOSA_InitObj;
        note:Initialize the moving safe structure
  else(no)
        :OPSEBA_MOSA_IntegrateMovement;
        note
        used self vehicle speed to
        compensate the movement of the object
        end note
  endif
  :FPSEBA_MOSA_UpdateAverageVelocity;
  note:Calculate the average longitudinal and lateral distance
  :FPSEBA_MOSA_UpdatePedFlag;
  note:Use the doppler effect to detect pedestrians
  :FPSEBA_MOSA_UpdateStatObstacleFlag;
  note:Check for high quality stationary obstacles
  :FPSEBA_MOSA_UpdateFastEnoughFlag;
  note:Check for high speed moving objects
  :FPSEBA_MOSA_UpdateMovementFlag;
  note:Check whether the target is moving
  :FPSEBA_MOSA_UpdateIsOldEnoughFlag;
  note:Whether the target life cycle exceeds the threshold check item
  :FPSEBA_MOSA_UpdateBelowMaxDistanceFlag;
  note:Whether the longitudinal distance of the target is within 5 seconds of
the vehicle speed
  :FPSEBA_MOSA_UpdateVeloWayFlag;
  note:Cross-check the effectiveness of the target
  :FPSEBA_MOSA_CheckConditions;
  note:Setting the object deceleration according to the flag can trigger the
deceleration level
  :FPSEBA_MOSA_SaveStaticData;
  note:write data to mosa structure
  end

  @enduml

******************************************************************************/
void OPSEBA_MOSA_Process(EM_t_ObjNumber iObjNumber) {
    OPSEBA_MOSA_ObjFlags_t sCurrCycleObjFlags;

    if ((iObjNumber >= 0) && (iObjNumber < Envm_NR_PRIVOBJECTS)) {
        // init sCurrCycleObjFlags flags
        sCurrCycleObjFlags.bIsBelowMaxDistance = FALSE;
        sCurrCycleObjFlags.bIsOldEnough = FALSE;
        sCurrCycleObjFlags.bIsPedCandidate = FALSE;
        sCurrCycleObjFlags.bMovedFarEnoughLvl1 = FALSE;
        sCurrCycleObjFlags.bMovedFarEnoughLvl2 = FALSE;
        sCurrCycleObjFlags.bVeloWayPlausible = FALSE;
        sCurrCycleObjFlags.bWasFastEnough = FALSE;
        sCurrCycleObjFlags.bWasObsWhenStat = FALSE;

        // init mosa data for new object, modify lifetime judgement conditions
        // to <= 20200507
        if (OPSEBA_MOSA_GetObjLifeCycles(iObjNumber) <= 1) {
            OPSEBA_MOSA_InitObj(iObjNumber);
        } else {
            // compensation object velocity accroding as object yawrate and
            // distX,distY
            OPSEBA_MOSA_IntegrateMovement(iObjNumber);
        }
        // update object abs velocity use for moving safe check
        OPSEBA_MOSA_UpdateAverageVelocity(iObjNumber);

        // moving safe check item base on object movement property
        OPSEBA_MOSA_UpdatePedFlag(iObjNumber, &sCurrCycleObjFlags);
        OPSEBA_MOSA_UpdateStatObstacleFlag(iObjNumber, &sCurrCycleObjFlags);
        OPSEBA_MOSA_UpdateFastEnoughFlag(iObjNumber, &sCurrCycleObjFlags);
        OPSEBA_MOSA_UpdateMovementFlag(iObjNumber, &sCurrCycleObjFlags);
        OPSEBA_MOSA_UpdateIsOldEnoughFlag(iObjNumber, &sCurrCycleObjFlags);
        OPSEBA_MOSA_UpdateBelowMaxDistanceFlag(iObjNumber, &sCurrCycleObjFlags);
        OPSEBA_MOSA_UpdateVeloWayFlag(iObjNumber, &sCurrCycleObjFlags);

        // check permission level according as check results
        OPSEBA_MOSA_CheckConditions(iObjNumber, &sCurrCycleObjFlags);
        OPSEBA_MOSA_SaveStaticData(iObjNumber, &sCurrCycleObjFlags);

        // save flag to extern var,for debug
        // OPSEBA_MOSA_OutPutData2CSV(iObjNumber, &sCurrCycleObjFlags);
    } else {
        // Out Of Obj Index Range
        //__EM_ASSERT(FALSE);
    }
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_GetObjLifeCycles

  @brief        Get Object uiLifeCycle

  @description  Get Object uiLifeCycle,return the object lifetime

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
static uint32 OPSEBA_MOSA_GetObjLifeCycles(EM_t_ObjNumber iObjNumber) {
    return Envm_INT_GET_Envm_OBJ(iObjNumber).Legacy.uiLifeTime;
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_IntegrateMovement

  @brief        Calculate the Object Movement Integrate

  @description  Calculate the Object Movement Integrate

  @param[in]    -

  @return       -

  @pre          -

  @uml

  @startuml
  start
  if (ObjAbsV> MINSPEED and EgoCurve <  1 / MAXRADIUS) then(ObjAbsV > 0.25,
EgoCurve < 1/50)
        :calculate fObjDistXCompensate;
        note
        calculate fObjDistXCompensate by OPSEBA_MOSA_CompensateEgoMovement
function
        end note
        :fObjDistXCompensate = fObjDistXCompensate - ObjLastDistX;
        note
        substract last cycle obj distX
        end note
        :Calculate fObjDistXMoveOverGround;
        note
        add the current cycle fObjDistXCompensate to update the
DistXMovOverground
        DistXMovOverground = fObjDistXCompensate + aiDistXMoved
        end note
        :write aiDistXMoved;
        note:write DistXMovOverground to aiDistXMoved

        :calculate fObjDistYCompensate;
        note
        calculate fObjDistYCompensate by OPSEBA_MOSA_CompensateEgoMovement
function
        end note
        :fObjDistYCompensate = fObjDistYCompensate - ObjLastDistY;
        note
        substract last cycle obj distY
        end note
        :Calculate fObjDistYMoveOverGround;
        note
        add the current cycle fObjDistYCompensate to update the
fObjDistYMoveOverGround
        fObjDistYMoveOverGround = fObjDistYCompensate + aiDistYMoved
        end note
        :write aiDistYMoved;
        note:write fObjDistYMoveOverGround to aiDistYMoved

  endif
  end
  @enduml

******************************************************************************/
static void OPSEBA_MOSA_IntegrateMovement(EM_t_ObjNumber iObjNumber) {
    float32 fObjVx;
    float32 fObjVy;
    float32 fObjVbsSqr;
    float32 fEgoCurve;

    fObjVx = OPSEBA_MOSA_GetVabsX(iObjNumber);
    fObjVy = OPSEBA_MOSA_GetVabsY(iObjNumber);
    fObjVbsSqr = SQR(fObjVx) + SQR(fObjVy);
    fEgoCurve = EM_f_GetEgoObjSyncCurve();

    /* Check whether we are in a curve situation.
 Here the movement over ground cannot be determined accurately. Pause the
 integration.
 If the object is very slow the probability that it is a stationary object is
 very high */
    if ((fObjVbsSqr >
         SQR(OPSEBA_MOSA_MIN_OBJ_VELOCITY_FOR_MOVEMENT_INTEGRATION)) &&
        (fABS(fEgoCurve) <
         (1.0f / OPSEBA_MOSA_MAX_RADIUS_FOR_MOVEMENT_INTEGRATION))) {
        // init Obj Value
        float32 fEgoVx;
        float32 fEgoVy;
        float32 fEgoYawrate;
        float32 fObjDistX;
        float32 fObjDistY;
        float32 fObjDistXCompensate;
        float32 fObjDistXMoveOverGround;
        float32 fObjDistYCompensate;
        float32 fObjDistYMoveOverGround;
        float32 fLongPosToRot;
        OPSEBA_MOSA_Data_t* pMOSAData = &sOPSEBA_MOSA_Data;

        fEgoVx = EM_f_GetEgoObjSyncVelX();
        fEgoVy = 0.0f;
        fEgoYawrate = EM_f_GetEgoObjSyncYawRate();
        fObjDistX = OPSEBA_MOSA_GetDistX(iObjNumber);
        fObjDistY = OPSEBA_MOSA_GetDistY(iObjNumber);
        fLongPosToRot = Envm_f_GetSensorLongPosToRot();

        // if ego speed and yawrate lower thresh
        // calculate compensate Obj DistX
        fObjDistXCompensate = OPSEBA_MOSA_CompensateEgoMovement(
            fObjDistX, fObjDistY, fEgoYawrate, fEgoVx, fEgoVy);
        // substract last cycle obj distX
        fObjDistXCompensate -=
            OPSEBA_MOSA_TrafMovDistXToFloat(pMOSAData->aiDistXLast[iObjNumber]);
        // Integrate movement
        fObjDistXMoveOverGround =
            fObjDistXCompensate +
            OPSEBA_MOSA_TrafMovDistToFloat(pMOSAData->aiDistXMoved[iObjNumber]);
        // Save the integrated movement in the static structure
        pMOSAData->aiDistXMoved[iObjNumber] =
            OPSEBA_MOSA_TrafFloatToMovDist(fObjDistXMoveOverGround);

        // Use the calculated movement over ground to update the average
        // movement
        OPSEBA_MOSA_UpdateAverageMovement(
            iObjNumber, &sOPSEBA_MOSA_Data.aiDistXMovAverage[iObjNumber],
            fObjDistXCompensate);

        // calculate compensate Obj DistY
        // The lateral range of the radar installation needs to be added
        float32 fTmpObjDistX = fObjDistX + fLongPosToRot;
        fObjDistYCompensate = OPSEBA_MOSA_CompensateEgoMovement(
            fObjDistY, fTmpObjDistX, fEgoYawrate, fEgoVy, fEgoVx);
        // substract last cycle obj distY
        fObjDistYCompensate -=
            OPSEBA_MOSA_TrafMovDistToFloat(pMOSAData->aiDistYLast[iObjNumber]);
        // Integrate movement
        fObjDistYMoveOverGround =
            fObjDistYCompensate +
            OPSEBA_MOSA_TrafMovDistToFloat(pMOSAData->aiDistYMoved[iObjNumber]);
        // Save the integrated movement in the static structure
        pMOSAData->aiDistYMoved[iObjNumber] =
            OPSEBA_MOSA_TrafFloatToMovDist(fObjDistYMoveOverGround);

        // Use the calculated movement over ground to update the average
        // movement
        OPSEBA_MOSA_UpdateAverageMovement(
            iObjNumber, &sOPSEBA_MOSA_Data.aiDistYMovAverage[iObjNumber],
            fObjDistYCompensate);
    }
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_CompensateEgoMovement

  @brief        Compensate object movement

  @description  Compensate object movement caused by yawrate

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  if (max(fEgoSpeed,fEgoSpeedOrtho) > Thresh) then(speed > 0.0001)
        :calculate fEgoSpeedCompensate;
        note
        Compensate object movement caused by yaw rate
        fEgoSpeedCompensate = fEgoSpeed + fYawRate * fDistLeverArm
        end note
  else(no)
        :fEgoSpeedCompensate = fEgoSpeed;
        note:not to Compensate movement
  :calculate fDistCompensate;
  note:fDistCompensate = fDist + (fEgoSpeedCompensate * cycletime)
  :return fDistCompensate;
  endif
  end
  @enduml

******************************************************************************/
static float32 OPSEBA_MOSA_CompensateEgoMovement(float32 fDist,
                                                 float32 fDistLeverArm,
                                                 float32 fYawRate,
                                                 float32 fEgoSpeed,
                                                 float32 fEgoSpeedOrtho) {
    float32 fEgoSpeedCompensate;
    float32 fDistCompensate;

    if (fABS(MAX(fEgoSpeed, fEgoSpeedOrtho)) > C_F32_DELTA) {
        // Calculate compensate ego speed
        fEgoSpeedCompensate = fEgoSpeed + fYawRate * fDistLeverArm;
    } else {
        // Do not compensate if ego speed very slow
        fEgoSpeedCompensate = fEgoSpeed;
    }

    // Add the movement caused by the ego movement from the given distance
    fDistCompensate = fDist + (fEgoSpeedCompensate * EM_f_GetCycleTime());
    return fDistCompensate;
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_UpdateAverageMovement

  @brief        Update Object average movement

  @description  Update Object average movement by Measure history and movement

  @param[in]    -

  @return       -

  @pre          -
  @uml
  @startuml
  :get uMeasHist;
  note:gets the measured status bit of the Object
  if(current flag == True) then(Only update in cycles where the object is
updated)
        :iObjMeasuredCycles = 1;
        :uCycle = 1;
        while (uCycle < 8)
                :shift the flag bitmask;
                note
                uBitMask =
                OPSEBA_MOSA_VELO_WAY_BITMASK_CURRENT_CYCLE >> uCycle
                end note
                :check the bit flag;
                if (bit check == True) then()
                        :iObjMeasuredCycles += 1;
                        note:to add measured cycle count
                endif
                :uCycle += 1;
        end while
        :calculate fObjMoveAverage;
        note
        use low pass filter to calculate fObjMoveAverage
        fObjMoveAverage = (iObjMeasuredCycles - 1) * fObjMoveAverage +
fCurrentMovement
        fObjMoveAverage = fObjMoveAverage / iObjMeasuredCycles
        end note
        :return fObjMoveAverage;
  @enduml

******************************************************************************/
static void OPSEBA_MOSA_UpdateAverageMovement(
    EM_t_ObjNumber iObjNumber,
    iOPSEBA_MOSA_MovDist_t* piAverageMovement,
    float32 fCurrentMovement) {
    uint32 uMeasHist = OPSEBA_MOSA_GetMeasHistory(iObjNumber);

    // Only update in cycles where the object is updated
    if (CML_GetBit(uMeasHist, OPSEBA_MOSA_VELO_WAY_BITMASK_CURRENT_CYCLE) ==
        TRUE) {
        uint32 uCycle;
        uint32 uBitMask;
        sint32 iObjMeasuredCycles = 1;
        float32 fObjMoveAverage =
            OPSEBA_MOSA_TrafMovDistToFloat(*piAverageMovement);

        for (uCycle = 1; uCycle < OPSEBA_MOSA_VELO_WAY_PLAUSI_AVERAGING_CYCLES;
             uCycle++) {
            uBitMask = OPSEBA_MOSA_VELO_WAY_BITMASK_CURRENT_CYCLE >> uCycle;
            if (CML_GetBit(uMeasHist, uBitMask) == TRUE) {
                iObjMeasuredCycles += 1;
            }
        }
        // Calculate fObjMoveAverage aroccding to before MoveAverage,seems like
        // a Filter
        // Weight the current value with the number of cycles used to calculate
        // it minus one and add the current movement. Divide by the total number
        // of cycles
        fObjMoveAverage =
            (((float32)(iObjMeasuredCycles - 1) * fObjMoveAverage) +
             (1.0f * fCurrentMovement)) /
            (float32)iObjMeasuredCycles;

        *piAverageMovement = OPSEBA_MOSA_TrafFloatToMovDist(fObjMoveAverage);
    }
}
/*****************************************************************************
  @fn           OPSEBA_MOSA_UpdateAverageVelocity

  @brief        Update Object Average Velocity

  @description  Update Object Average Velocity by Object Current Velocity

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  note:Update average longtitude and lateral speed
  :get uMeasuredHistory;
  note:get measured status bitflag
  if (is uMeasuredHistory) then (yes)
        :calculate fSumCycTime;
        note:use OPSEBA_MOSA_GetSumCycleTime to get Object cycleTime;
        if (SumCycleTime > MINCYCLE ) then(SumCycleTime > 0.0001)
                :get VAbsY;
                note:Object Current VAbsY
                :get VAbsX;
                note:Object Current VAbsX

                :get VabsAverageX;
                note:get Object VabsAverageX at the last moment
                :Calculate VabsAverageX;
                note
                update VabsAverageX
                VabsAverageX =
                ((AvgTime * VabsAverageX) +
                (currentcycletime * VAbsX)) / sumcycletime
                end note
                :set VabsXAverage;
                note:write result to data

                :get VabsAverageY;
                note:get Object VabsAverageY at the last moment
                :Calculate VabsAverageY;
                note:update VabsAverageY
                :set VabsYAverage;
                note::write result to data
         endif
  endif
  stop
  @enduml

******************************************************************************/
static void OPSEBA_MOSA_UpdateAverageVelocity(EM_t_ObjNumber iObjNumber) {
    uint32 uMeasuredHistory = OPSEBA_MOSA_GetMeasHistory(iObjNumber);

    // Only update in cycles where the object is updated
    if (CML_GetBit(uMeasuredHistory,
                   OPSEBA_MOSA_VELO_WAY_BITMASK_CURRENT_CYCLE) == TRUE) {
        float32 fCurrentCycTime = EM_f_GetCycleTime();
        float32 fSumCycTime;
        float32 fAverageCycTime;
        float32 fObjVabsX;
        float32 fObjVabsY;

        // Calculate object sum cycle time
        fSumCycTime = OPSEBA_MOSA_GetSumCycleTime(
            iObjNumber, OPSEBA_MOSA_GetObjLifeCycles(iObjNumber));

        if (fSumCycTime > C_F32_DELTA) {
            float32 fObjVabsXAverage;
            float32 fObjVabsYAverage;

            fAverageCycTime = fSumCycTime - fCurrentCycTime;
            fObjVabsX = OPSEBA_MOSA_GetVabsX(iObjNumber);
            fObjVabsY = OPSEBA_MOSA_GetVabsY(iObjNumber);

            // Get the current average value
            fObjVabsXAverage = OPSEBA_MOSA_TrafMovDistToFloat(
                sOPSEBA_MOSA_Data.aiVabsXAverage[iObjNumber]);
            // seem like a LowPassFilter for fObjVabsAverage
            // Weight the current value with the cycle time used to calculate
            // it, add the current velocity. Divide it by the sum
            fObjVabsXAverage = ((fAverageCycTime * fObjVabsXAverage) +
                                (fCurrentCycTime * fObjVabsX)) /
                               fSumCycTime;
            // Save the calculated value in the static structure
            sOPSEBA_MOSA_Data.aiVabsXAverage[iObjNumber] =
                OPSEBA_MOSA_TrafFloatToMovDist(fObjVabsXAverage);

            // Get the current average value
            fObjVabsYAverage = OPSEBA_MOSA_TrafMovDistToFloat(
                sOPSEBA_MOSA_Data.aiVabsYAverage[iObjNumber]);
            // Weight the current value with the cycle time used to calculate
            // it, add the current velocity. Divide it by the sum
            fObjVabsYAverage = ((fAverageCycTime * fObjVabsYAverage) +
                                (fCurrentCycTime * fObjVabsY)) /
                               fSumCycTime;
            // Save the calculated value in the static structure
            sOPSEBA_MOSA_Data.aiVabsYAverage[iObjNumber] =
                OPSEBA_MOSA_TrafFloatToMovDist(fObjVabsYAverage);
        }
    }
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_GetSumCycleTime

  @brief        Calculate Object Life Cycle Time

  @description  Calculate Object Life Cycle Time

  @param[in]    -

  @return       -

  @pre          -

  @uml

  @startuml
  :get uMeasuredHistory;
  note:gets the measured status bit of the Object
  :Limit uNumOfCycles;
  note:Limit the max uNumOfCycles to sCycleTime.uNumOfCycles(8)
  :iCycle = 0;
  while(iCycle < uNumOfCycles)
        :shift the bitmask;
        note:uBitMask = 128 >> iCycle
        :calculate fSumCycleTime;
        note
        add Object history cycletime
        fSumCycleTime += auCycleTime[iCycle]
        end note
        :iCycle += 1;
  end while
  :return fSumCycleTime/1000.0f;
  note:to convert time unit(ms) to unit(m)
  @enduml

******************************************************************************/
static float32 OPSEBA_MOSA_GetSumCycleTime(EM_t_ObjNumber iObjNumber,
                                           uint32 uNumOfCycles) {
    uint32 uMeasuredHistory = OPSEBA_MOSA_GetMeasHistory(iObjNumber);
    float32 fSumCycleTime = 0.0f;
    OPSEBA_MOSA_CycleTime_t const* pMOSADataCycTime =
        &sOPSEBA_MOSA_Data.sCycleTime;
    uint32 uBitMask;
    sint32 iCycle;

    // Limit the max number of cycles to evaluate to the number of stored cycles
    uNumOfCycles = MIN(uNumOfCycles - 1u, pMOSADataCycTime->uNumOfCycles);

    for (iCycle = 0; iCycle < uNumOfCycles; iCycle++) {
        // Calculate the bit mask for the current cycle
        uBitMask = OPSEBA_MOSA_VELO_WAY_BITMASK_CURRENT_CYCLE >> iCycle;

        if (CML_GetBit(uMeasuredHistory, uBitMask) == TRUE) {
            // Sum up the cycle times
            fSumCycleTime += pMOSADataCycTime->auCycleTime[iCycle];
        }
    }

    // return the calculated cycle time in seconds. Saved values are in
    // milliseconds
    return ((float32)fSumCycleTime / C_MILLISEC_SEC);
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_UpdatePedFlag

  @brief        Update the object PedCandidate flag

  @description  Update the object PedCandidate flag

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :get PCC confimation value from radar raw data;
  note
  The result of this check will return FALSE directly
  because this attribute cannot be obtained from radar
  end note
  :MinDistY = 2.0;
  if (object isPedCandidata pass in last cycle) then (yes)
  :MinDistY = 0;
  endif
  if((PCCConfirmation == 1) or (PCCConfirmation == 2 while distY < MinDistY))
then (yes)
  :bPedCandidate = TRUE;
  else (no)
  :bPedCandidate = FALSE;
  endif
  end
  @enduml
******************************************************************************/
static void OPSEBA_MOSA_UpdatePedFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags) {
    OPSEBA_MOSA_ObjQual_t const* const pObjQual =
        &sOPSEBA_MOSA_Data.asObjQual[iObjNumber];
    // uint8 const uPCCConfimation =
    //     OPSEBA_MOSA_IsMicrodopplerConfirmed(iObjNumber);
    boolean bObjPedCandidate = FALSE;
    // float32 fObjMinDistYThresh = OPSEBA_MOSA_MIN_DISTY_PEDCAND;

    // Need radar raw data to check pedcandidata
    // apply hysteresis
    // if (pObjQual->bIsPedCandidate == True)
    //{
    //	fObjMinDistYThresh = 0.0f;
    //}

    ////Reduce thresholds for microdoppler confirmed objects not in ego lane
    ////This is necessary to preserve pedestrian performanceand cover tunnel
    ///fans anyway
    // if ((uPCCConfimation == OPSEBA_MOSA_PCC_CONFIRM_MICRODOPPLER_TRAJ) ||
    //	((OPSEBA_MOSA_GetDistY(iObjNumber)) && (uPCCConfimation ==
    //OPSEBA_MOSA_PCC_CONFIRM_MICRODOPPLER)))
    //{
    //	bObjPedCandidate = TRUE;
    //}

    // return the calculated value
    pCurrentCheckFlags->bIsPedCandidate = bObjPedCandidate;
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_IsMicrodopplerConfirmed

  @brief        Confirmed the Pedestrian Object

  @description  Confirmed the Pedestrian Object

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
static uint8 OPSEBA_MOSA_IsMicrodopplerConfirmed(EM_t_ObjNumber iObjNumber) {
    // Now always return Confirm None
    uint8 uPCC_Confirmation = OPSEBA_MOSA_PCC_CONFIRM_NONE;
    return uPCC_Confirmation;
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_UpdateStatObstacleFlag

  @brief        Check Stationary Obstcale Object

  @description  Check Stationary Obstcale Object

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  if (is Stationary and isn't Move2Stat) then(stopped confidence <= 80)
        if(LifeCycle > MINLIFECYCLE) then(LifeCycle > 20)
                if (is bWasObsWhenStat) then (yes)
                        :Calculate ObstclProbThresh;
                        note:ObstclProbThresh = 53
                else(no)
                        :Calculate ObstclProbThresh;
                        note:ObstclProbThresh = 60;
                endif

                if (ObjObsProb >= ObstclProbThresh)
                        :bIsObstacleObj = TRUE;
                        note:Obstacle Check pass
                else(no)
                        :bIsObstacleObj = FALSE;
                endif
        endif
  else(no)
        :bIsObstacleObj =  bWasObsWhenStat;
        note:use check result in the last time
  endif
  :update current bWasObsWhenStat;
  note:pCurrentCheckFlags->bWasObsWhenStat = bIsObstacleObj
  end
  @enduml

******************************************************************************/
static void OPSEBA_MOSA_UpdateStatObstacleFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags) {
    OPSEBA_MOSA_ObjQual_t const* const pObjQual =
        &sOPSEBA_MOSA_Data.asObjQual[iObjNumber];
    uint32 uiObjObsThresh;
    uint32 uiObjObsProb;
    boolean bIsObstacleObj = FALSE;

    // Stationary not stopped objects, camera confirmed stopped object also need
    // to be updated
    if ((EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber) ==
             OBJECT_PROPERTY_STATIONARY &&
         !Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(iObjNumber)) ||
        (Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(iObjNumber) &&
         Envm_INT_GET_Envm_OBJ(iObjNumber).SensorSpecific.bCamConfirmed)) {
        // Check whether the flag shall be set or reseted
        if (OPSEBA_MOSA_GetObjLifeCycles(iObjNumber) >
            OPSEBA_MOSA_MIN_LIFE_CYCLES) {
            uiObjObsProb = EM_INT_OBJ_OBSTACLE_PROBABILITY(iObjNumber);

            // Reduce was obstacle object obsthresh
            if (pObjQual->bWasObsWhenStat == TRUE) {
                uiObjObsThresh = OPSEBA_MOSA_MIN_OBSTACLE_PROB_STAT_DROP;
            } else {
                uiObjObsThresh = OPSEBA_MOSA_MIN_OBSTACLE_PROB_STAT_PICKUP;
            }

            // Check the object obstacle value
            if (uiObjObsProb >= uiObjObsThresh) {
                bIsObstacleObj = TRUE;
            } else {
                bIsObstacleObj = FALSE;
            }
        }

    } else {
        // Moving and stopped objects use last cycle flag
        bIsObstacleObj = pObjQual->bWasObsWhenStat;
    }
    // Update current check flag
    pCurrentCheckFlags->bWasObsWhenStat = bIsObstacleObj;
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_UpdateFastEnoughFlag

  @brief        Check object speed FastEnough flag

  @description  Check moving or oncoming Object speed

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  if (is bWasFastEnough) then(yes)
        :bWasFastEnough = TRUE;
        note
        If the previous cycle passed the check item,
        the check is skipped directly
        end note
  else(no)
        if (isn't Stationary) then(yes)
                :Calculate fEgoSpeedX;
                note:limit EgoSpeed to Min-MAX(0.5,2)
                :Calculate MinAbsVelocitySqr;
                note
                Calculate the minimum speed threshold
                according to the ego speed multiplied by the factor
                MinAbsVelocitySqr = (EgoVel*0.08))^2
                end note
                if (is bIsPedCandidate) then(yes)
                        :Calculate MinAbsVelocitySqr;
                        note
                        if MinAbsVelocitySqr > 0.5^2:
                        MinAbsVelocitySqr = 0.5^2
                        end note
                endif
                if (VabsX^2 + VabsY^2 > MinAbsVelocitySqr )
                        :bWasFastEnough = TRUE;
                        note
                        pass the check
                        end note
                endif
        endif
  endif
  end
  @enduml

******************************************************************************/
static void OPSEBA_MOSA_UpdateFastEnoughFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags) {
    OPSEBA_MOSA_ObjQual_t* const pObjQual =
        &sOPSEBA_MOSA_Data.asObjQual[iObjNumber];

    if (pObjQual->bWasFastEnough == TRUE) {
        pCurrentCheckFlags->bWasFastEnough = TRUE;
    } else {
        // Calculate for Moving and Oncoming Object
        if (EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber) !=
            OBJECT_PROPERTY_STATIONARY) {
            float32 fObjVabsX;
            float32 fObjVabsY;
            float32 fEgoSpeedX;
            float32 fObjMinAbsSpeedSqr;

            fObjVabsX = OPSEBA_MOSA_GetVabsX(iObjNumber);
            fObjVabsY = OPSEBA_MOSA_GetVabsY(iObjNumber);
            fEgoSpeedX = EM_f_GetEgoObjSyncVelX();

            // Limit EgoSpeed and calculate Thresh
            fObjMinAbsSpeedSqr = TUE_CML_MinMax(
                OPSEBA_MOSA_MIN_ABS_VELOCITY_MIN,
                OPSEBA_MOSA_MIN_ABS_VELOCITY_MAX,
                fEgoSpeedX * OPSEBA_MOSA_MIN_ABS_VELOCITY_FACTOR);
            fObjMinAbsSpeedSqr = SQR(fObjMinAbsSpeedSqr);

            // Need radar rawdata to check is pedcandidate object
            // Lower the necessary velocity if the object seems to be a
            // pedestrian
            if (pObjQual->bIsPedCandidate == TRUE) {
                fObjMinAbsSpeedSqr =
                    MIN(fObjMinAbsSpeedSqr,
                        SQR(OPSEBA_MOSA_MIN_ABS_VELOCITY_PEDCAND));
            }

            // Check the absolute speed condition always. Minimize the impact of
            // the moving safe check on stopping cars
            if ((SQR(fObjVabsX) + SQR(fObjVabsY)) > fObjMinAbsSpeedSqr) {
                pCurrentCheckFlags->bWasFastEnough = TRUE;
            }
        }
    }
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_UpdateMovementFlag

  @brief        check object movement distance

  @description  check object movement distance whether exceed the threshold
value

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :set current movement check result by last cycle check result;
  note:set current result by last cycle time check result
  if (bMovedFarEnoughLvl2 failed last cycle and object is not stationary) then
(yes)
  :calculate object moved distance square value;
  note:calculate object square value = distX^2 + distY^2
  if (object passed isPedCandidate) then (yes)
  if (DistMovedSqr > 1) then (yes)
  :bMovedFarEnoughLvl2 = TRUE;
  else if (DistMovedSqr > 0.25) then (yes)
  :bMovedFarEnoughLvl1 = TRUE;
  endif
  else (no)
  if (DistMovedSqr > 36) then (yes)
  :bMovedFarEnoughLvl2 = TRUE;
  else if (DistMovedSqr > 16) then (yes)
  :bMovedFarEnoughLvl1 = TRUE;
  endif
  endif
  endif
  end
  @enduml

******************************************************************************/
static void OPSEBA_MOSA_UpdateMovementFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags) {
    OPSEBA_MOSA_Data_t* const pOPSEBA_MOSA_Data = &sOPSEBA_MOSA_Data;

    pCurrentCheckFlags->bMovedFarEnoughLvl1 =
        pOPSEBA_MOSA_Data->asObjQual[iObjNumber].bMovedFarEnoughLvl1;
    pCurrentCheckFlags->bMovedFarEnoughLvl2 =
        pOPSEBA_MOSA_Data->asObjQual[iObjNumber].bMovedFarEnoughLvl2;

    // Check movement if object not have full movement permissions for moving
    // and oncoming object
    if ((pCurrentCheckFlags->bMovedFarEnoughLvl2 == FALSE) &&
        (EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber) !=
         OBJECT_PROPERTY_STATIONARY)) {
        float32 fObjDistXMoved = OPSEBA_MOSA_TrafMovDistToFloat(
            pOPSEBA_MOSA_Data->aiDistXMoved[iObjNumber]);
        float32 fObjDistYMoved = OPSEBA_MOSA_TrafMovDistToFloat(
            pOPSEBA_MOSA_Data->aiDistYMoved[iObjNumber]);
        float32 fObjDistMovedSqr = SQR(fObjDistXMoved) + SQR(fObjDistYMoved);

        // Missing data to check Tunnel stat now
        // Use higher thresholds if in tunnel
        // Missing data to check if is the pedcandidate object
        // Check pedcandidate object for use lower thresh
        if (pCurrentCheckFlags->bIsPedCandidate == TRUE) {
            if (fObjDistMovedSqr >
                SQR(OPSEBA_MOSA_MIN_MOVEMENT_PERMISSION_LVL2_PED_CAND)) {
                pCurrentCheckFlags->bMovedFarEnoughLvl2 = TRUE;
            } else if (fObjDistMovedSqr >
                       SQR(OPSEBA_MOSA_MIN_MOVEMENT_PERMISSION_LVL1_PED_CAND)) {
                pCurrentCheckFlags->bMovedFarEnoughLvl1 = TRUE;
            }
        } else {
            if (fObjDistMovedSqr >
                SQR(OPSEBA_MOSA_MIN_MOVEMENT_PERMISSION_LVL2)) {
                pCurrentCheckFlags->bMovedFarEnoughLvl2 = TRUE;
            } else if (fObjDistMovedSqr >
                       SQR(OPSEBA_MOSA_MIN_MOVEMENT_PERMISSION_LVL1)) {
                pCurrentCheckFlags->bMovedFarEnoughLvl1 = TRUE;
            }
        }
    }
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_GetTunnelState

  @brief        Check if in TunnelStat

  @description  Check if in TunnelStat

  @param[in]    -

  @return       -

  @pre          -

******************************************************************************/
static boolean OPSEBA_MOSA_GetTunnelState(void) {
    boolean bInTunnel = FALSE;
    float32 fTunnelProb = 0.0F;

    // Now always return false because miss data to check tunnel stat
    if (fTunnelProb > OPSEBA_MOSA_INTUNNEL_PROB_THRESH) {
        bInTunnel = TRUE;
    }

    return bInTunnel;
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_UpdateIsOldEnoughFlag

  @brief        check the object lifetime is old enough

  @description  check the object lifetime is old enough

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  note:check the object lifetime is old enough
  :get LifeThresh;
  note:LifeThresh = 20
  if (is bIsPedCandidate) then(yes)
  :get LifeThresh;
  note:pedcandidate object use 5 LifeThresh
  else(no)
  endif
  if (LifeTime > LifeThresh) then(yes)
  :bIsOldEnough = TRUE;
  note
  check the object lifetime
  set pCurrentCheckFlags = TRUE
  end note
  else(no)
  ::bIsOldEnough = FALSE;
  endif
  end
  @enduml

******************************************************************************/
static void OPSEBA_MOSA_UpdateIsOldEnoughFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags) {
    uint32 uiObjLifeTimeThresh = OPSEBA_MOSA_MIN_LIFE_CYCLES;
    uint32 uiObjLifeTime = OPSEBA_MOSA_GetObjLifeCycles(iObjNumber);

    // Use lower thresh for pedcandidate object
    if (pCurrentCheckFlags->bIsPedCandidate == TRUE) {
        uiObjLifeTimeThresh = OPSEBA_MOSA_MIN_LIFE_CYCLES_PEDCAND;
    }

    // Check object lifetime
    if (uiObjLifeTime > uiObjLifeTimeThresh) {
        pCurrentCheckFlags->bIsOldEnough = TRUE;
    } else {
        pCurrentCheckFlags->bIsOldEnough = FALSE;
    }
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_UpdateBelowMaxDistanceFlag

  @brief        check the object if in the max distance range

  @description  check the object if in the max distance range

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :Calculate MaxDistance;
  note:MaxDistance = egoSpeedX * 5s
  if (DistX < MaxDistance) then(yes)
        :bIsBelowMaxDistance = TRUE;
        note
        check whether the objec distx below the thresh
        end note
  else(no)
        :bIsBelowMaxDistance = FALSE;
  endif
  end
  @enduml

******************************************************************************/
static void OPSEBA_MOSA_UpdateBelowMaxDistanceFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags) {
    float32 fObjMaxDistThresh =
        EM_f_GetEgoObjSyncVelX() * OPSEBA_MOSA_MAX_TIME_GAP;
    float32 fObjDistX = OPSEBA_MOSA_GetDistX(iObjNumber);

    if (fObjDistX < fObjMaxDistThresh) {
        pCurrentCheckFlags->bIsBelowMaxDistance = TRUE;
    } else {
        pCurrentCheckFlags->bIsBelowMaxDistance = FALSE;
    }
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_UpdateVeloWayFlag

  @brief        Verification of observed velocity and observed distance

  @description  Verification of observed velocity and observed distance

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :get averaged distance1 form global variable;
  :get averaged distance2 = averaged velocity * cycle time;
  note:get distance from position and  averaged velocity * cycle time
  :DifferentialVal = MAX((fABS(distance �C distance) �C 0.01), 0) / MAX(0.5 *
fABS
  (distance1 + distance2)), 0.01);
  note:Calculate the difference value
  :TotalDifferentialVal = Sqrt(DifferentialValX^2 + DifferentialValY^2);
  note:sum the longtitude and lateral DifferentialVal
  if (object is updated in current cycle) then (yes)
  if (LifeCycle > 2) then (yes)
  :fFilteredDiffVal = Tue_CML_LowPassFilter2;
  note:exceeding two cycles, filtered by low-pass filtering
  else (no)
  :fFilteredDiffVal = TotalDifferentialVal;
  endif
  :fFilteredDiffVal = TUE_CML_MinMax;
  note:Limit fFilteredDiffVal
  else (no)
  :fFilteredDiffVal = LastCycleDiffVal;
  note:use last cycle value if not to updated
  endif
  :fDistRelDiffThresh = Tue_CML_BoundedLinInterpol2;
  note:get DiffThresh by object longtitude distance
  if (isPedCandidate?) then (yes)
  :fDistRelDiffThresh *= 1.5;
  note:increase DiffThresh for isPedCandidate object
  endif
  if(fFilteredDiffVal < fDistRelDiffThresh) then (yes)
  :bVeloWayPlausible = TRUE;
  else (no)
  :bVeloWayPlausible = FALSE;
  endif
  end
  @enduml

******************************************************************************/
static void OPSEBA_MOSA_UpdateVeloWayFlag(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t* const pCurrentCheckFlags) {
    OPSEBA_MOSA_Data_t* const pMOSADATA = &sOPSEBA_MOSA_Data;
    float32 fObjDistX = OPSEBA_MOSA_GetDistX(iObjNumber);
    float32 fObjDistY = OPSEBA_MOSA_GetDistY(iObjNumber);
    float32 fObjDistSqr = SQR(fObjDistX) + SQR(fObjDistY);
    uint32 uiLifeTime = OPSEBA_MOSA_GetObjLifeCycles(iObjNumber);
    uint8 uMeasureHistory = OPSEBA_MOSA_GetMeasHistory(iObjNumber);
    float32 fObjDistXAvgFromVel = 0.0f;
    float32 fObjDistYAvgFromVel = 0.0f;
    float32 fObjDistRelDiffFilter;
    float32 fObjDistRelDiffThresh;

    // Get averaged moved from pos
    float32 fObjDistXAvgFromPos = OPSEBA_MOSA_TrafMovDistToFloat(
        pMOSADATA->aiDistXMovAverage[iObjNumber]);
    float32 fObjDistYAvgFromPos = OPSEBA_MOSA_TrafMovDistToFloat(
        pMOSADATA->aiDistYMovAverage[iObjNumber]);
    float32 fObjVabsXAvg =
        OPSEBA_MOSA_TrafMovDistToFloat(pMOSADATA->aiVabsXAverage[iObjNumber]);
    float32 fObjVabsYAvg =
        OPSEBA_MOSA_TrafMovDistToFloat(pMOSADATA->aiVabsYAverage[iObjNumber]);

    // Calculate averaged move from averaged velocity
    if (uiLifeTime > 1u) {
        fObjDistXAvgFromVel = fObjVabsXAvg * EM_f_GetCycleTime();
        fObjDistYAvgFromVel = fObjVabsYAvg * EM_f_GetCycleTime();
    }

    // Calculate relative different movement base on position and velocity
    float32 fObjDistXDiff = OPSEBA_MOSA_CalculateRelDiffOfWayAndVelo(
        fObjDistXAvgFromPos, fObjDistXAvgFromVel);
    float32 fObjDistYDiff = OPSEBA_MOSA_CalculateRelDiffOfWayAndVelo(
        fObjDistYAvgFromPos, fObjDistYAvgFromVel);
    // Sum the x and y value
    float32 fObjDistRelDiff = SQRT(SQR(fObjDistXDiff) + SQR(fObjDistYDiff));

    // Pause updating of the value if the object is not updated
    if (CML_GetBit(uMeasureHistory,
                   OPSEBA_MOSA_VELO_WAY_BITMASK_CURRENT_CYCLE) == TRUE) {
        if (uiLifeTime > 2u) {
            // Get the current value
            fObjDistRelDiffFilter =
                pMOSADATA->auRelDiff[iObjNumber] *
                (1.0F / OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_TRAFO);

            // Calculate the filtered value
            if (fObjDistRelDiff < fObjDistRelDiffFilter) {
                Tue_CML_LowPassFilter2(
                    &fObjDistRelDiffFilter, fObjDistRelDiff,
                    OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_FILTER_CONST_DOWN);
            } else {
                Tue_CML_LowPassFilter2(
                    &fObjDistRelDiffFilter, fObjDistRelDiff,
                    OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_FILTER_CONST_UP);
            }
        } else {
            // In the first cycle overtake the current value
            fObjDistRelDiffFilter = fObjDistRelDiff;
        }
        // Calculate result
        fObjDistRelDiffFilter = TUE_CML_MinMax(
            0.0f, OPSEBA_MOSA_VELO_WAY_UINT8_MAX *
                      (1.0F / OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_TRAFO),
            fObjDistRelDiffFilter);
        pMOSADATA->auRelDiff[iObjNumber] = (uint8)ROUND_TO_UINT(
            fObjDistRelDiffFilter * OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_TRAFO);
    } else {
        // Overtake last time value
        fObjDistRelDiffFilter =
            pMOSADATA->auRelDiff[iObjNumber] *
            (1.0F / OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_TRAFO);
    }
    // Get VelDiffThresh
    fObjDistRelDiffThresh = Tue_CML_BoundedLinInterpol2(
        fObjDistSqr, SQR(OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_NEAR_THRESH),
        SQR(OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_FAR_THRESH),
        OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_NEAR,
        OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_FAR);

    // Change Thresh for pedstrain
    if (pCurrentCheckFlags->bIsPedCandidate == TRUE) {
        fObjDistRelDiffThresh *=
            (1.0f + OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_THRESH_PED_INCR);
    }

    // Check Diff Thresh
    if (fObjDistRelDiffFilter < fObjDistRelDiffThresh) {
        pCurrentCheckFlags->bVeloWayPlausible = TRUE;
    }
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_CalculateRelDiffOfWayAndVelo

  @brief        Calculate the distance difference

  @description  Calculate the distance difference

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  :fDifference = MAX((distance1 - distance2) - 0.01, 0);
  note: Calculate numerator
  :fDivisor = MAX( 0.5*(distance1 + distance2) ,0.01);
  note:Calculate denominator
  :fRealDiff = fDifference / fDivisor;
  note:Calculate fRealDiff
  end
  @enduml

******************************************************************************/
static float32 OPSEBA_MOSA_CalculateRelDiffOfWayAndVelo(
    float32 fPosDistAverage, float32 fPosVeloAverage) {
    float32 fDifference;
    float32 fDivisor;
    float32 fRealDiff;

    // Calculate relative difference
    fDifference = MAX(fABS(fPosDistAverage - fPosVeloAverage) -
                          OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_TOLERANCE,
                      0.0f);
    fDivisor = MAX(0.5f * fABS(fPosDistAverage + fPosVeloAverage),
                   OPSEBA_MOSA_VELO_WAY_MAX_REL_DIFF_TOLERANCE);

    fRealDiff = fDifference / fDivisor;

    return fRealDiff;
}
/*****************************************************************************
  @fn           OPSEBA_MOSA_CheckConditions

  @brief        Check object moving safe flag

  @description  Mark the deceleration level based on moving safe check result

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  if (isn't stationary is stopped) then(stopped confidence > 80)
  :pass;
  note
  only check
  moving and oncoming objects
  and stopped objects
  end note
  if (isn't PERMISSION_LVL2) then(yes)
  :get VabsX;
  note
  Check whether the object has already full permission.
  If not check the conditions
  end note
  if (bWasObsWhenStat) then(yes)
  :set PERMISSION_LVL2;
  note
  Objects which were detected while stationary
  shall get all permissions directly
  end note
  else if (VabsX > MAXCHECK) then(VabsX > 5)
  :set PERMISSION_LVL2;
  note
  Objects which are faster than a threshold(5)
  get directly full permission
  end note
  else if ( bWasFastEnough and
  bIsOldEnough and
  bVeloWayPlausible and
  bIsBelowMaxDistance) then(yes)
  if (bMovedFarEnoughLvl2) then(yes)
  :set PERMISSION_LVL2;
  note
  if pass the
  wasFastEnough
  isOldEnough
  veloWayPlausible
  isBelowMaxDistanc
  then set moving quality level
  dependent on moved distance

  end note
  else if (bMovedFarEnoughLvl1) then(yes)
  :set PERMISSION_LVL1;
  else(no)
  endif
  else(no)
  endif
  else(no)
  endif
  else(no)
  endif
  stop
  @enduml

******************************************************************************/
static void OPSEBA_MOSA_CheckConditions(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t const* const pCurrentCheckFlags) {
    OPSEBA_MOSA_ObjQual_t* const pObjQual =
        &sOPSEBA_MOSA_Data.asObjQual[iObjNumber];

    // Check moving ,oncoming and stopped objects
    if ((EM_INT_OBJ_DYNAMIC_PROPERTY(iObjNumber) !=
         OBJECT_PROPERTY_STATIONARY) ||
        (Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(iObjNumber) == TRUE)) {
        if (pObjQual->MovObjQual != OPSEBA_MOSA_PERMISSION_LVL2) {
            // Get current Object Longtitude Vabs
            float32 fObjVabsX = OPSEBA_MOSA_GetVabsX(iObjNumber);

            if (pCurrentCheckFlags->bWasObsWhenStat == TRUE) {
                pObjQual->MovObjQual = OPSEBA_MOSA_PERMISSION_LVL2;
            } else if (fABS(fObjVabsX) > OPSEBA_MOSA_MAX_VABS_FOR_CHECKS) {
                pObjQual->MovObjQual = OPSEBA_MOSA_PERMISSION_LVL2;
            } else if ((pCurrentCheckFlags->bWasFastEnough == TRUE) &&
                       (pCurrentCheckFlags->bIsOldEnough == TRUE) &&
                       (pCurrentCheckFlags->bVeloWayPlausible == TRUE) &&
                       (pCurrentCheckFlags->bIsBelowMaxDistance == TRUE)) {
                // Set permission levle base on moved distance
                if (pCurrentCheckFlags->bMovedFarEnoughLvl2 == TRUE) {
                    pObjQual->MovObjQual = OPSEBA_MOSA_PERMISSION_LVL2;
                } else if (pCurrentCheckFlags->bMovedFarEnoughLvl1 == TRUE) {
                    pObjQual->MovObjQual = OPSEBA_MOSA_PERMISSION_LVL1;
                }
            }
        }
    }
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_SaveStaticData

  @brief        Sava current object flag and position

  @description  Sava current object flag and position

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  start
  note:write current flag to pObjQual
  :update bWasObsWhenStat;
  :update bIsPedCandidate;
  :update bWasFastEnough;
  :update bMovedFarEnoughLvl1;
  :update bMovedFarEnoughLvl2;
  note:write current Distance to sOPSEBA_MOSA_Data
  :update aiDistXLast;
  :update aiDistYLast;
  end
  @enduml

******************************************************************************/
static void OPSEBA_MOSA_SaveStaticData(
    EM_t_ObjNumber iObjNumber,
    OPSEBA_MOSA_ObjFlags_t const* const pCurrentCheckFlags) {
    OPSEBA_MOSA_Data_t* const pOPSEBA_MOSA_Data = &sOPSEBA_MOSA_Data;
    OPSEBA_MOSA_ObjQual_t* const pObjQual =
        &pOPSEBA_MOSA_Data->asObjQual[iObjNumber];

    // Update current check flag
    pObjQual->bWasObsWhenStat = pCurrentCheckFlags->bWasObsWhenStat;
    pObjQual->bIsPedCandidate = pCurrentCheckFlags->bIsPedCandidate;
    pObjQual->bWasFastEnough = pCurrentCheckFlags->bWasFastEnough;
    pObjQual->bMovedFarEnoughLvl1 = pCurrentCheckFlags->bMovedFarEnoughLvl1;
    pObjQual->bMovedFarEnoughLvl2 = pCurrentCheckFlags->bMovedFarEnoughLvl2;

    // Save current object position
    pOPSEBA_MOSA_Data->aiDistXLast[iObjNumber] =
        OPSEBA_MOSA_TrafFloatToMovDistX(OPSEBA_MOSA_GetDistX(iObjNumber));
    pOPSEBA_MOSA_Data->aiDistYLast[iObjNumber] =
        OPSEBA_MOSA_TrafFloatToMovDist(OPSEBA_MOSA_GetDistY(iObjNumber));
}

/*****************************************************************************
  @fn           OPSEBA_MOSA_GetObjPermission

  @brief        Get object permission level

  @description  Get object permission level

  @param[in]    -

  @return       -

  @pre          -

  @uml
  @startuml
  :uiMOSAPermission = LV2;
  note:set default permission to Level 2;
  if(0 <= iObjNumber < 25)then(yes)
        if (Object isn't stationary)
                :uiMOSAPermission = check result;
                note:use moving safe check result
        else(no)
                if(object is moving to stationary)then(stoppedconfidence > 80)
                        :uiMOSAPermission = check result;
                        note:use moving safe check result for stationary object
                else(no)
                        :noting to do;
                endif
        endif
  else(no)
        :uiMOSAPermission = Permission None;
  endif
  :return uiMOSAPermission;
  stop
  @enduml

******************************************************************************/
uint8 OPSEBA_MOSA_GetObjPermission(EM_t_ObjNumber iObjNumber) {
    uint8 uiMOSAPermission = OPSEBA_MOSA_PERMISSION_LVL2;

    if ((iObjNumber >= 0) && (iObjNumber < Envm_NR_PRIVOBJECTS)) {
        OPSEBA_MOSA_ObjQual_t const* const pMOSAQual =
            &sOPSEBA_MOSA_Data.asObjQual[iObjNumber];

        // Check dynmaic property
        if (OPSEBA_MOSA_IsObjStationary(iObjNumber) == FALSE) {
            // moving and oncoming object
            uiMOSAPermission = pMOSAQual->MovObjQual;
        } else {
            // stopped objects
            if (Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(iObjNumber) == TRUE) {
                uiMOSAPermission = pMOSAQual->MovObjQual;
            } else {
                // stationary object
                // noting to do,stationary obstacles now get full permission
                // always
            }
        }
    } else {
        // If ObjectNumber out of index range, do not allow any permission
        uiMOSAPermission = OPSEBA_MOSA_PERMISSION_NONE;
        // Create assertion
        //__EM_ASSERT(FALSE);
    }
    return uiMOSAPermission;
}

/* ************************************************************************* */
/*  OPS_UNIT_TEST                                                            */
/* ************************************************************************* */
#if OPS_UNIT_TEST_SWITCH == TRUE
OPSEBA_MOSA_ObjFlags_t sExternObjFlags;

void SetMOSAData(OPSEBA_MOSA_Data_t* pMOSAData) {
    int iObjectNumber = 0;
    int iIndex;

    // set interal MOSA from extern
    sOPSEBA_MOSA_Data.aiDistXLast[iObjectNumber] =
        OPSEBA_MOSA_TrafFloatToMovDistX(pMOSAData->aiDistXLast[iObjectNumber]);
    sOPSEBA_MOSA_Data.aiDistXMovAverage[iObjectNumber] =
        OPSEBA_MOSA_TrafFloatToMovDist(
            pMOSAData->aiDistXMovAverage[iObjectNumber]);
    sOPSEBA_MOSA_Data.aiDistXMoved[iObjectNumber] =
        OPSEBA_MOSA_TrafFloatToMovDist(pMOSAData->aiDistXMoved[iObjectNumber]);
    sOPSEBA_MOSA_Data.aiDistYLast[iObjectNumber] =
        OPSEBA_MOSA_TrafFloatToMovDist(pMOSAData->aiDistYLast[iObjectNumber]);
    sOPSEBA_MOSA_Data.aiDistYMovAverage[iObjectNumber] =
        OPSEBA_MOSA_TrafFloatToMovDist(
            pMOSAData->aiDistYMovAverage[iObjectNumber]);
    sOPSEBA_MOSA_Data.aiDistYMoved[iObjectNumber] =
        OPSEBA_MOSA_TrafFloatToMovDist(pMOSAData->aiDistYMoved[iObjectNumber]);
    sOPSEBA_MOSA_Data.aiVabsXAverage[iObjectNumber] =
        OPSEBA_MOSA_TrafFloatToMovDist(
            pMOSAData->aiVabsXAverage[iObjectNumber]);
    sOPSEBA_MOSA_Data.aiVabsYAverage[iObjectNumber] =
        OPSEBA_MOSA_TrafFloatToMovDist(
            pMOSAData->aiVabsYAverage[iObjectNumber]);
    sOPSEBA_MOSA_Data.asObjQual[iObjectNumber] =
        pMOSAData->asObjQual[iObjectNumber];
    sOPSEBA_MOSA_Data.auRelDiff[iObjectNumber] =
        pMOSAData->auRelDiff[iObjectNumber];
}

iOPSEBA_MOSA_MovDist_t UnitTest_OPSEBA_MOSA_TrafFloatToMovDist(
    float32 fMovement) {
    return OPSEBA_MOSA_TrafFloatToMovDist(fMovement);
}

iOPSEBA_MOSA_MovDist_t UnitTest_OPSEBA_MOSA_IntegrateMovement(
    EM_t_ObjNumber iObjNumber) {
    OPSEBA_MOSA_IntegrateMovement(iObjNumber);
    return sOPSEBA_MOSA_Data.aiDistXMoved[iObjNumber];
}

boolean UnitTest_OPSEBA_MOSA_UpdateStatObstacleFlag(EM_t_ObjNumber iObjNumber) {
    memset(&sExternObjFlags, 0, sizeof(sExternObjFlags));
    OPSEBA_MOSA_UpdateStatObstacleFlag(iObjNumber, &sExternObjFlags);
    return sExternObjFlags.bWasObsWhenStat;
}

boolean UnitTest_OPSEBA_MOSA_UpdateFastEnoughFlag(EM_t_ObjNumber iObjNumber) {
    memset(&sExternObjFlags, 0, sizeof(sExternObjFlags));
    OPSEBA_MOSA_UpdateFastEnoughFlag(iObjNumber, &sExternObjFlags);
    return sExternObjFlags.bWasFastEnough;
}

OPSEBA_MOSA_ObjFlags_t* UnitTest_OPSEBA_MOSA_UpdateMovementFlag(
    EM_t_ObjNumber iObjNumber) {
    memset(&sExternObjFlags, 0, sizeof(sExternObjFlags));
    OPSEBA_MOSA_UpdateMovementFlag(iObjNumber, &sExternObjFlags);
    return &sExternObjFlags;
}

boolean UnitTest_OPSEBA_MOSA_UpdateIsOldEnoughFlag(EM_t_ObjNumber iObjNumber) {
    memset(&sExternObjFlags, 0, sizeof(sExternObjFlags));
    OPSEBA_MOSA_UpdateIsOldEnoughFlag(iObjNumber, &sExternObjFlags);
    return sExternObjFlags.bIsOldEnough;
}

boolean UnitTest_OPSEBA_MOSA_UpdateBelowMaxDistanceFlag(
    EM_t_ObjNumber iObjNumber) {
    memset(&sExternObjFlags, 0, sizeof(sExternObjFlags));
    OPSEBA_MOSA_UpdateBelowMaxDistanceFlag(iObjNumber, &sExternObjFlags);
    return sExternObjFlags.bIsBelowMaxDistance;
}

boolean UnitTest_OPSEBA_MOSA_UpdateVeloWayFlag(EM_t_ObjNumber iObjNumber) {
    memset(&sExternObjFlags, 0, sizeof(sExternObjFlags));
    OPSEBA_MOSA_UpdateVeloWayFlag(iObjNumber, &sExternObjFlags);
    return sExternObjFlags.bVeloWayPlausible;
}

uint8 UnitTest_OPSEBA_MOSA_CheckConditions(
    EM_t_ObjNumber iObjNumber, OPSEBA_MOSA_ObjFlags_t* pCurrentCheckFlag) {
    int iObjectNumber = 0;
    memcpy(&sExternObjFlags, pCurrentCheckFlag, sizeof(sExternObjFlags));
    OPSEBA_MOSA_CheckConditions(iObjNumber, &sExternObjFlags);
    return sOPSEBA_MOSA_Data.asObjQual[iObjectNumber].MovObjQual;
}
#endif

// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */