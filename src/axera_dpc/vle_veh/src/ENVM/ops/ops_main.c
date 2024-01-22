/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "TM_Global_Types.h"
#include "stddef.h"
#include "assert.h"
#include "ops.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
StateFPS_t StateFPS;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION
*****************************************************************************/

static void TUESelectedObjPostProcessInit(void);
void EMPropertyCal_Tuerme(void);
void SetAbsMovingStateBaseDynamicProperty(sint8 ObjNr);
void SetStoppedConfidenceBaseDynamicProperty(sint8 ObjNr);
void SetDynamicSubPropertyBaseDynamicProperty(sint8 ObjNr);
void SetDynamicPropertyBaseDynamicProperty(sint8 ObjNr);
sint8 OPS_SortObjectListDistX(sint8* pSortedList);
void OPS_ObjectListHeapSort(sint8 ElemNr,
                            const float32 a_compArray[],
                            sint8 a_sortedList[]);
void ObjectRCSThresholdCal(void);
float32 const* RSPGetLookUpTable(uint8 u_ScanMode);
float32 RSPLookUpTable(float32 f_DistRad, float32 const p_LUTable[]);
void OPSObjectMeasureStatusUpdate(void);

/*************************************************************************************************************************
  Functionname:    TUESelectedObjPostProcessInit */
static void TUESelectedObjPostProcessInit(void) {
    FPSInitACC();

    OPSAEBInit();
    OPSInitObjectPrioritization();
    OPSMOSAInit();
}

/*************************************************************************************************************************
  Functionname:    TUESelectedObjPostProcess */
void TUESelectedObjPostProcess(void) {
    if (StateFPS != FPS_INIT) {
        ObjectRCSThresholdCal();
        EMPropertyCal_Tuerme();
        // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/18 changed by
        // guotao 20200507 start
        // update object measurement condensity based on eObjMaintenanceState
        // value
        OPSObjectMeasureStatusUpdate();
        // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/18 changed by
        // guotao 20200507 end
        FPSACCProcess();
#ifndef TU_POWER_SAVING_WITH_ACC
        OPSEBAProcess();
#endif

        /*! Set object prio for output generation of external object list;
         * necessary only if generic object is active */
        TUE_OPS_PrioListOutputProcess();

    } else {
        /* Do initialization */
        TUESelectedObjPostProcessInit();
        StateFPS = FPS_OK;
    }
}

/*************************************************************************************************************************
  Functionname:    FPSDeleteObject */
void FPSDeleteObject(const sint8 ObjNr) {
    FPSInitACCObject(ObjNr);
    OPSInitEBAObject(ObjNr);
#if defined(FPS_CFG_OBJECT_MOSA_CHECK) && (FPS_CFG_OBJECT_MOSA_CHECK == 1)
    /* reset moving safe related data */
    OPSEBA_MOSA_InitObj(ObjNr);
#endif
}
/* **********************************************************************
  @fn            EMPropertyCal_Tuerme */ /*!

                                   @brief         calculate object property
                                   based on radar's input value

                                   @description   add some property to
                                   EnvmData.pPrivObjList since we do not have EM
                                   module anymore

                                   @return        void

                                   @pre           [none]

                                   @post          [none]


                                   @author        guotao
                                   ****************************************************************************
                                   */
void EMPropertyCal_Tuerme(void) {
    sint8 ObjNr;
    // set iNumOfUsedObjects and iDistXSortedObjectList
    GET_Envm_INT_OBJ_DATA_PTR->HeaderObjList.iNumOfUsedObjects = 0;
    EM_INT_OBJ_NUMBER_OF_OBJ_USED =
        OPS_SortObjectListDistX(Envm_INT_OBJ_INDEX_DISTX_SORTED);

    for (ObjNr = (Envm_NR_PRIVOBJECTS - 1); ObjNr >= 0L; ObjNr--) {
        if (!EM_INT_OBJ_IS_DELETED(ObjNr)) {
            // set value of GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].Attributes
            {
                // define eAbsMovingState according to the eDynamicProperty
                /* typedef eAbsMovingState_t */
                // #define OBJECT_MOVSTATE_STATIONARY  (0U)
                // #define OBJECT_MOVSTATE_STOPPED  (1U)
                // #define OBJECT_MOVSTATE_MOVING  (2U)
                SetAbsMovingStateBaseDynamicProperty(ObjNr);

                GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                    .Attributes.uiClassConfidence = 100;

                // set uiStoppedConfidence according to the eDynamicProperty
                // value
                SetStoppedConfidenceBaseDynamicProperty(ObjNr);
            }

            // set value of GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].Legacy
            // ---- TODO
            {
                // fRCSTargetThresholdUncomp
                GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].Legacy.fMaxAccelY =
                    5.0F;  // OD_MIN_AREL_Y

                // set eDynamicSubProperty according to the eDynamicProperty
                // value
                /* typedef eDynamicSubProperty_t */
                // #define GDB_OBJECT_SUBPROP_NORMAL  (0U)
                // #define GDB_OBJECT_SUBPROP_CROSSING  (1U)
                SetDynamicSubPropertyBaseDynamicProperty(ObjNr);

// set life time according to the eObjMaintenanceState
#if 0
                // we already have life time according to the track's age in the
                // fusion logic
                switch (GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                            .General.eObjMaintenanceState) {
                    case MT_STATE_DELETED:
                        GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                            .Legacy.uiLifeTime++;
                        break;
                    case MT_STATE_NEW:
                        GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                            .Legacy.uiLifeTime = 0;
                        break;
                    case MT_STATE_MEASURED:
                    case MT_STATE_PREDICTED:
                    case MT_STATE_MERGE_DELETED:
                    case MT_STATE_MERGE_NEW:
                        GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                            .Legacy.uiLifeTime++;
                        break;
                    default:
                        GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                            .Legacy.uiLifeTime++;
                        break;
                }
#endif
            }
            // set value of GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].General
            {
                GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                    .General.cObjMergingID =
                    OBJ_INDEX_NO_OBJECT;  // set to OBJ_INDEX_NO_OBJECT?

                /* typedef uint8 */
                // #define MT_STATE_DELETED  (0U)
                // #define MT_STATE_NEW  (1U)
                // #define MT_STATE_MEASURED  (2U)
                // #define MT_STATE_PREDICTED  (3U)
                // #define MT_STATE_MERGE_DELETED  (4U)
                // #define MT_STATE_MERGE_NEW  (5U)
                // GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].General.eObjMaintenanceState;
                // get this from radar directly

                // GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].General.fLifeTime =
                // GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].Legacy.uiLifeTime *
                // TASK_CYCLE_TIME_50;//OBJ_LIFETIME_SEC

                GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].General.fTimeStamp =
                    GET_Envm_INT_OBJ_DATA_PTR->uiTimeStamp;
            }
            // set value of GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].Geometry
            {
                GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                    .Geometry.fOrientationValid = 0;
            }

            // set value of GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].Qualifiers
            {
                GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                    .Qualifiers.uMeasuredTargetFrequencyFar = 0;
                GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                    .Qualifiers.uMeasuredTargetFrequencyNear = 0;
            }

            // change eDynamicProperty from radar type to algo type
            /* typedef eDynamicProperty_t */
            // #define OBJECT_PROPERTY_MOVING  (0U)
            // #define OBJECT_PROPERTY_STATIONARY  (1U)
            // #define OBJECT_PROPERTY_ONCOMING  (2U)
            SetDynamicPropertyBaseDynamicProperty(ObjNr);
        }
    }
}
/* **********************************************************************
  @fn            SetAbsMovingStateBaseDynamicProperty */
/*!
  @brief         calculate object property based on radar's input value

  @description   add AbsMovingState property base on DynamicProperty

  @return        void

  @pre           [none]

  @post          [none]

  @author        guotao
 *************************************************************************/
void SetAbsMovingStateBaseDynamicProperty(sint8 ObjNr) {
    switch (
        GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].Attributes.eDynamicProperty) {
        case TU_OBJECT_DYNPROP_MOVING:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eAbsMovingState = OBJECT_MOVSTATE_MOVING;
            break;
        case TU_OBJECT_DYNPROP_STATIONARY:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eAbsMovingState = OBJECT_MOVSTATE_STATIONARY;
            break;
        case TU_OBJECT_DYNPROP_ONCOMING:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eAbsMovingState = OBJECT_MOVSTATE_MOVING;
            break;
        case TU_OBJECT_DYNPROP_STATIONARY_CANDITATE:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eAbsMovingState = OBJECT_MOVSTATE_STATIONARY;
            break;
        case TU_OBJECT_DYNPROP_UNKNOWN:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eAbsMovingState = OBJECT_MOVSTATE_STATIONARY;
            break;
        case TU_OBJECT_DYNPROP_CROSSING_STATIONARY:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eAbsMovingState = OBJECT_MOVSTATE_STATIONARY;
            break;
        case TU_OBJECT_DYNPROP_CROSSING_MOVING:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eAbsMovingState = OBJECT_MOVSTATE_MOVING;
            break;
        case TU_OBJECT_DYNPROP_STOPPED:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eAbsMovingState = OBJECT_MOVSTATE_STOPPED;
            break;
        default:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eAbsMovingState = OBJECT_MOVSTATE_STATIONARY;
            break;
    }
}
/* **********************************************************************
  @fn            SetStoppedConfidenceBaseDynamicProperty */
/*!
  @brief         calculate object property based on radar's input value

  @description   add StoppedConfidence property base on DynamicProperty

  @return        void

  @pre           [none]

  @post          [none]

  @author        guotao
 *************************************************************************/
void SetStoppedConfidenceBaseDynamicProperty(sint8 ObjNr) {
    switch (
        GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].Attributes.eDynamicProperty) {
        case TU_OBJECT_DYNPROP_MOVING:
        case TU_OBJECT_DYNPROP_ONCOMING:
        case TU_OBJECT_DYNPROP_STATIONARY_CANDITATE:
        case TU_OBJECT_DYNPROP_UNKNOWN:
        case TU_OBJECT_DYNPROP_CROSSING_STATIONARY:
        case TU_OBJECT_DYNPROP_CROSSING_MOVING:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.uiStoppedConfidence = 0;
            break;
        case TU_OBJECT_DYNPROP_STATIONARY:
        case TU_OBJECT_DYNPROP_STOPPED:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.uiStoppedConfidence = 100;
            break;
        default:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.uiStoppedConfidence = 0;
            break;
    }
}
/* **********************************************************************
  @fn            SetDynamicSubPropertyBaseDynamicProperty */
/*!
  @brief         calculate object property based on radar's input value

  @description   add DynamicSubProperty property base on DynamicProperty

  @return        void

  @pre           [none]

  @post          [none]

  @author        guotao
 *************************************************************************/
void SetDynamicSubPropertyBaseDynamicProperty(sint8 ObjNr) {
    switch (
        GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].Attributes.eDynamicProperty) {
        case TU_OBJECT_DYNPROP_MOVING:
        case TU_OBJECT_DYNPROP_STATIONARY:
        case TU_OBJECT_DYNPROP_ONCOMING:
        case TU_OBJECT_DYNPROP_STATIONARY_CANDITATE:
        case TU_OBJECT_DYNPROP_UNKNOWN:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Legacy.eDynamicSubProperty = GDB_OBJECT_SUBPROP_NORMAL;
            break;
        case TU_OBJECT_DYNPROP_CROSSING_STATIONARY:
        case TU_OBJECT_DYNPROP_CROSSING_MOVING:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Legacy.eDynamicSubProperty = GDB_OBJECT_SUBPROP_CROSSING;
            break;
        case TU_OBJECT_DYNPROP_STOPPED:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Legacy.eDynamicSubProperty = GDB_OBJECT_SUBPROP_NORMAL;
            break;
        default:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Legacy.eDynamicSubProperty = GDB_OBJECT_SUBPROP_NORMAL;
            break;
    }
}
/* **********************************************************************
  @fn            SetDynamicPropertyBaseDynamicProperty */
/*!
  @brief         calculate object property based on radar's input value

  @description   add DynamicProperty property base on DynamicProperty

  @return        void

  @pre           [none]

  @post          [none]

  @author        guotao
 *************************************************************************/
void SetDynamicPropertyBaseDynamicProperty(sint8 ObjNr) {
    switch (
        GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr].Attributes.eDynamicProperty) {
        case TU_OBJECT_DYNPROP_MOVING:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eDynamicProperty = OBJECT_PROPERTY_MOVING;
            break;
        case TU_OBJECT_DYNPROP_STATIONARY:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eDynamicProperty = OBJECT_PROPERTY_STATIONARY;
            break;
        case TU_OBJECT_DYNPROP_ONCOMING:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eDynamicProperty = OBJECT_PROPERTY_ONCOMING;
            break;
        case TU_OBJECT_DYNPROP_STATIONARY_CANDITATE:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eDynamicProperty = OBJECT_PROPERTY_STATIONARY;
            break;
        case TU_OBJECT_DYNPROP_UNKNOWN:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eDynamicProperty = OBJECT_PROPERTY_STATIONARY;
            break;
        case TU_OBJECT_DYNPROP_CROSSING_STATIONARY:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eDynamicProperty = OBJECT_PROPERTY_STATIONARY;
            break;
        case TU_OBJECT_DYNPROP_CROSSING_MOVING:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eDynamicProperty = OBJECT_PROPERTY_MOVING;
            break;
        case TU_OBJECT_DYNPROP_STOPPED:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eDynamicProperty = OBJECT_PROPERTY_MOVING;
            break;
        default:
            GET_Envm_INT_OBJ_DATA_PTR->Objects[ObjNr]
                .Attributes.eDynamicProperty = OBJECT_PROPERTY_STATIONARY;
            break;
    }
}
/*************************************************************************************************************************
  Functionname:    OPS_SortObjectListDistX */ /*!

                                                                               @brief           Sort object list by distance fDistX

                                                                               @description     Sort object list by distance fDistX
                                                                                                                - objects with property free get sorted to the front of
                                                                                                                      this sorted list (as fDistX == 0.0)
                                                                                                                - an array with object indices is sorted, the object list
                                                                                                                      stays unchanged

                                                                               @return          sint8 : Number of used objects

                                                                               @param[in,out]   pSortedList : Array with object list indices to sort

                                                                               @pre             None
                                                                               @post            Array with object list indices is sorted by object distance
                                                                             *************************************************************************************************************************/
sint8 OPS_SortObjectListDistX(sint8* pSortedList) {
    float32 fArrayObjDistX[Envm_NR_PRIVOBJECTS];
    sint8 iObjId;
    sint8 iUsedObjNumber = 0;

    /* copy DistX from Object List to local array ArrayObjDistX */
    for (iObjId = 0; iObjId < Envm_NR_PRIVOBJECTS; iObjId++) {
        if (!EM_INT_OBJ_IS_DELETED(iObjId)) {
            fArrayObjDistX[iObjId] = EM_INT_OBJ_LONG_DISPLACEMENT(iObjId);
            iUsedObjNumber++;
        } else {
            /* to have deleted objects at end of list, set DistX to high value
             */
            fArrayObjDistX[iObjId] = 1000000.f;
        }
    }
    OPS_ObjectListHeapSort((sint8)Envm_NR_PRIVOBJECTS, fArrayObjDistX,
                           pSortedList);
    return iUsedObjNumber;
}

/*************************************************************************************************************************
  Functionname:    OPS_ObjectListHeapSort */ /*!

                                                                                @brief           Heapsort objectList algorithm (see Numerical Recipes)

                                                                                @description     Sorts an index array referencing a numerical array
                                                                                                                 into ascending numerical order

                                                                                @return          void

                                                                                @param[in]       ElemNr : number of objects samples
                                                                                @param[in]       a_compArray[] : array of compare values
                                                                                @param[in,out]   a_sortedList[] : sorted index rearrangement

                                                                                @pre             None ?
                                                                                @post            No changes ?
                                                                              *************************************************************************************************************************/
void OPS_ObjectListHeapSort(sint8 ElemNr,
                            const float32 a_compArray[],
                            sint8 a_sortedList[]) {
    sint8 i;

    /* Initialize index array */
    for (i = (sint8)0; i < ElemNr; i++) {
        a_sortedList[i] = i;
    }

    if (ElemNr > (sint8)1) {
        sint8 l, ir;
        boolean b_Finish = FALSE;

        l = ElemNr / 2;
        ir = ElemNr - (sint8)1;

        while (b_Finish == FALSE) {
            sint8 ridx;
            if (l > 0) {
                /* Still in hiring phase */
                ridx = a_sortedList[--l];
            } else {
                /* In retirement-and-promotion phase */
                ridx = a_sortedList[ir]; /* Clear a space at end of array */
                a_sortedList[ir] =
                    a_sortedList[0]; /* Retire the top of the heap into it */
                if (--ir == (sint8)0) {
                    /* Done with the last promotion */
                    a_sortedList[0] =
                        ridx; /* The least component worker of all */
                    b_Finish = TRUE;
                }
            }
            if (b_Finish == FALSE) {
                sint8 j;
                boolean b_Abort = FALSE;

                i = l; /* Whether in the hiring phase or promotion phase, we */
                j = l + l;
                // here set up to sift down element rra to its proper level
                while ((b_Abort == FALSE) && (j <= ir)) {
                    if ((j < ir) && (a_compArray[a_sortedList[j]] <
                                     a_compArray[a_sortedList[j + (sint8)1]])) {
                        // Compare to the better underling */
                        j++;
                    }
                    if (a_compArray[ridx] <
                        a_compArray[a_sortedList[j]]) { /* Demote rra */
                        a_sortedList[i] = a_sortedList[j];
                        i = j;
                        if (j * 2 < ElemNr) {
                            j = (j > (sint8)0) ? (sint8)(j * 2) : (sint8)1;
                        } else {
                            j = ElemNr;
                        }
                    } else { /* Found rra's level. Terminate the sift-down */
                        b_Abort = TRUE;
                    }
                }
                /* Put rra into its slot */
                a_sortedList[i] = ridx;
            }
        }
    }
    return;
}

/* **********************************************************************
  Functionname:    ObjectRCSThresholdCal                           */ /*!

      @brief           Calculate the actual target RCS threshold

      @description     Calculate the actual target RCS threshold
                                       for every object

      @return          void

      @c_switch_full   None

      @pre             None
      @post            No changes


      @c_switch_full   None
    *********************************************************************** */
void ObjectRCSThresholdCal(void) {
    i32_t iObj;
    float32 f_ObjX, f_ObjY, f_ObjRangeRad;
    float32 const* p_RCSThreshTable;

    /* Get the lookUpTable for the RCS Threshold */

    p_RCSThreshTable = RSPGetLookUpTable(SYS_SCAN_NEAR);

    for (iObj = 0; iObj < Envm_NR_PRIVOBJECTS; iObj++) {
        /* get RCS threshold for every object */
        if (!EM_INT_OBJ_IS_DELETED(iObj)) {
            f_ObjX = EM_INT_OBJ_LONG_DISPLACEMENT(iObj);
            f_ObjY = Envm_INT_OBJ_LAT_DISPLACEnvmENT(iObj) -
                     Envm_f_GetSensorLatOffset();
            f_ObjRangeRad = SQRT(SQR(f_ObjX) + SQR(f_ObjY));
            /* Get normal RCS threshold */
            Envm_INT_OBJ_RCS_TGT_TRESHOLD_UNCOMP(iObj) =
                RSPLookUpTable(f_ObjRangeRad, p_RCSThreshTable);
        }
    }
}

/*************************************************************************************************************************
  Functionname:    RSPLookUpTable */ /*!

                                                                                        @brief           gets RCS threshold for given distance

                                                                                        @description     Detailed Design

                                                                                        @return          float32

                                                                                        @param[in]       f_DistRad : distance of cluster
                                                                                        @param[in]       p_LUTable : pointer to LUT

                                                                                        @pre             None ?
                                                                                        @post            No changes ?
                                                                                      *************************************************************************************************************************/
float32 RSPLookUpTable(float32 f_DistRad, float32 const p_LUTable[]) {
    float32 f_RCSThres;
    i32_t s_Node;

    /* Calculate distance index */
    s_Node = (i32_t)(f_DistRad);
    /* limit index to limits of RCS table */
    s_Node = TUE_CML_MinMax(0, (RSP_NUM_OF_RCS_TABLE_VAL - 1), s_Node);
    /* compute threshold */
    f_RCSThres = p_LUTable[s_Node];

    return f_RCSThres;
}

/* ****************************************************************************
@function          RSPGetLookUpTable                                     */ /*!
    @brief             gets RCS threshold LUT
    @description       gets RCS threshold LUT depending on scan mode and ego
    velocity

    @param[in]         u_ScanMode         scan id (near, far)
    @return            pointer to LUTable

    ****************************************************************************/
float32 const* RSPGetLookUpTable(uint8 u_ScanMode) {
    float32 f_VehSpeed;
    float32 const* p_LUTable = NULL;

    /* RCS target threshold in dBsqm for traced targets in near range (have the
     * same direction of motion as the own vehicle) */
    static const float32 RCSThreshTargetTracedNear_c[RSP_NUM_OF_RCS_TABLE_VAL] =
        {RCS_THR_TARGET_TRACED_NEAR_ARRAY};

    /* RCS target threshold in dBsqm for traced targets at medium ego velocity
     * in near range (have the same direction of motion as the own vehicle) */
    static const float32
        RCSThreshTargetTracedNearMidVel_c[RSP_NUM_OF_RCS_TABLE_VAL] = {
            RCS_THR_TARGET_TRACED_NEAR_MID_VEL_ARRAY};

    /* RCS target threshold in dBsqm for traced targets for low velocities in
     * near range (have the same direction of motion as the own vehicle) */
    static const float32
        RCSThreshTargetTracedNearLowVel_c[RSP_NUM_OF_RCS_TABLE_VAL] = {
            RCS_THR_TARGET_TRACED_NEAR_LOW_VEL_ARRAY};

    // only use Near Table to LookUp,20200310 by changchang

    _PARAM_UNUSED(u_ScanMode);

    /* get vehicle speed */
    f_VehSpeed = EM_f_GetEgoCluSyncVelX();

    if (f_VehSpeed < RCS_THR_LOW_VELOCITY) {
        p_LUTable = RCSThreshTargetTracedNearLowVel_c;
    } else if (f_VehSpeed < RCS_THR_MID_VELOCITY) {
        p_LUTable = RCSThreshTargetTracedNearMidVel_c;
    } else {
        p_LUTable = RCSThreshTargetTracedNear_c;
    }

    return p_LUTable;
}

/* Stop EM default section for code and data (back to default)
   This has to be at the very end of every EM c code file.      */
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
#define CONFIRM_DENSITY_SIZE 8U
#define CONFIRM_DENSITY_DEAFULT 0U
void OPSObjectMeasureStatusUpdate(void) {
    sint8 iObjNumber;

    // store measured target frequency value since we will memset all the value
    // after fusion process
    static uint8 uHistoryMeasuredTargetFrequencyNear[Envm_NR_PRIVOBJECTS];
    static uint8 uHistoryMeasuredTargetFrequencyFar[Envm_NR_PRIVOBJECTS];
    // loop all object for update measurestatus
    for (iObjNumber = (Envm_NR_PRIVOBJECTS - 1); iObjNumber >= 0;
         iObjNumber--) {
        // Write previous cycle data to EnvmData
        EnvmData.pPrivObjList->Objects[iObjNumber]
            .Qualifiers.uMeasuredTargetFrequencyNear =
            uHistoryMeasuredTargetFrequencyNear[iObjNumber];
        EnvmData.pPrivObjList->Objects[iObjNumber]
            .Qualifiers.uMeasuredTargetFrequencyFar =
            uHistoryMeasuredTargetFrequencyFar[iObjNumber];
        if (!EM_INT_OBJ_IS_DELETED(iObjNumber)) {
            // Shift the flag bit to record the current state bit
            EM_v_ShiftCurrentTgtConfirmDensity(iObjNumber);

            // Set the status bit to 1 for the measured or new from merge object
            if ((EM_INT_OBJ_MAINTENANCE_STATE(iObjNumber) ==
                 MT_STATE_MEASURED) ||
                EM_INT_OBJ_MAINTENANCE_STATE(iObjNumber) ==
                    MT_STATE_MERGE_NEW) {
                // Set the status bit to 1
                EnvmData.pPrivObjList->Objects[iObjNumber]
                    .Qualifiers.uMeasuredTargetFrequencyNear |=
                    (ui8_t)(1uL << (CONFIRM_DENSITY_SIZE - 1uL));
                EnvmData.pPrivObjList->Objects[iObjNumber]
                    .Qualifiers.uMeasuredTargetFrequencyFar |=
                    (ui8_t)(1uL << (CONFIRM_DENSITY_SIZE - 1uL));
            }
        } else {
            // Set all the status bit to 0
            Envm_v_SetTgtConfirmDensitySingleScan(
                iObjNumber, CONFIRM_DENSITY_DEAFULT, SYS_SCAN_NEAR);
            Envm_v_SetTgtConfirmDensitySingleScan(
                iObjNumber, CONFIRM_DENSITY_DEAFULT, SYS_SCAN_FAR);
        }
        // save calculate measurement history to static value
        uHistoryMeasuredTargetFrequencyNear[iObjNumber] =
            EnvmData.pPrivObjList->Objects[iObjNumber]
                .Qualifiers.uMeasuredTargetFrequencyNear;
        uHistoryMeasuredTargetFrequencyFar[iObjNumber] =
            EnvmData.pPrivObjList->Objects[iObjNumber]
                .Qualifiers.uMeasuredTargetFrequencyFar;
    }
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */