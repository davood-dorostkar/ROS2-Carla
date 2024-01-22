/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include <string.h>

#include "vlc_sen.h"
#include "cd.h"
#include "stddef.h"
#include "TM_Global_Types.h"

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/* ************** External variables ************** */

/* ************** Algo inputs ************** */
SET_MEMSEC_VAR(CDState)
SET_MEMSEC_VAR(bWasInInit)
SET_MEMSEC_VAR(uiVDYSigErrorCnt)
SET_MEMSEC_VAR(CDObjectsInt)
SET_MEMSEC_VAR(CDEgoDynDataRawInt)
SET_MEMSEC_VAR(CDEgoDynDataSyncInt)
SET_MEMSEC_VAR(CDEgoDynDataInt)
SET_MEMSEC_VAR(CDInputData)
SET_MEMSEC_VAR(CDAdjSafeDistanceInt)
SET_MEMSEC_VAR(CDParametersInt)
SET_MEMSEC_VAR(CDExternalFunctions)
SET_MEMSEC_VAR(CDRelevantHypothesesListInt)
SET_MEMSEC_VAR(CDPreviousHypothesesListInt)
SET_MEMSEC_VAR(CDExtObjectsInt)
SET_MEMSEC_VAR(CDStatusInt)
SET_MEMSEC_VAR(CDOutputData)

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
CDState_t CDState; /*!< CD State @VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_STATE
                      @CYCLEID: VLC_ENV */
static boolean bWasInInit =
    FALSE; /*!< TRUE if CD/CD wrapper has been initialized */

static uint16 uiVDYSigErrorCnt =
    0u; /*!< Counts consecutive VDY signal errors */

static CDObjectData_t
    CDObjectsInt; /*!< CD Object Data (combining structure)
                     @_VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_OBJ_INT
                     @_CYCLEID: VLC_ENV */

static CDEgoDynamic_t
    CDEgoDynDataRawInt; /*!< Raw (unsync'd) ego kinematics
                           @VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_DYN_RAW
                           @CYCLEID: VLC_ENV */
static CDEgoDynamic_t
    CDEgoDynDataSyncInt; /*!< Sync'd ego kinematics
                            @VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_DYN_SYNC
                            @CYCLEID: VLC_ENV */
static CDEgoDynData_t
    CDEgoDynDataInt;              /*!< Ego dynamic input (combining struct.)
                                     @_VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_EGO_DYN_INT
                                     @_CYCLEID: VLC_ENV */
static CDInputData_t CDInputData; /*!< Input data to CD algo (combining struct)
                                     @_VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_INP_INT
                                     @_CYCLEID: VLC_ENV */
static CDAdjSafeDistance_t
    CDAdjSafeDistanceInt; /*!< Adjustable safety distance
                             @VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_ADJ_SAFE_INT
                             @CYCLEID: VLC_ENV */
static CDParameters_t
    CDParametersInt; /*!< CD Parameters (combining struct)
                        @_VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_PAR_INT
                        @_CYCLEID: VLC_ENV */
static CDExternalFunctions_t
    CDExternalFunctions; /*!< Callback functions
                            @_VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_EXT_FUN
                            @_CYCLEID: VLC_ENV */
/* ************** Algo internal state ************** */

static CDIntHypothesisList_t
    CDRelevantHypothesesListInt; /*!< CD relevant hypotheses
                                    @VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_REL_HYP_INT
                                    @CYCLEID: VLC_ENV */
static CDIntHypothesisList_t
    CDPreviousHypothesesListInt; /*!< CD relevant hypotheses of previous cycle
                                  */

static CDInternalObjectList_t CDExtObjectsInt; /*!< External objects input */

static CDInternalStatus_t
    CDStatusInt; /*!< Internal CD state
                    @_VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_INT_STATE
                    @_CYCLEID: VLC_ENV */

/* ************** Algo outputs ************** */

static CDOutputData_t
    CDOutputData; /*!< Outputs of CD algo
                     @_VADDR:VLC_MEAS_D_CGEB_CD_WRAP_CD_OUT_DATA
                     @_CYCLEID: VLC_ENV */
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION
*****************************************************************************/

static void CDInitEMPData(void);
void CDProcessEMPTrajectoryMeasFreeze(EMPTrajPred_t *pEMPTrajPredEgo);

static void CDCalculateOverlapInt(const CPDistanceWidth_t *pDistWidthInt,
                                  CPTrajOccupancy_t *pOccupancy);

static void CDCheckVDYInput(void);
static void CDSetSignalStatus(void);
static void CDPreProcessing(void);
static void CDPostProcessing(void);

static void CDCustomMTSFreezeData(
    CDInternalMeasurementData_t *pCDMeasurementDataInt);
static void CDCombineIntObjects(void);

/* **********************************************************************
  @fn           CDCombineIntObject */ /*!

                                      @brief        The function combines data
                                    to the internal CD object.

                                      @description  The function combines data
                                    to the internal CD object.

                                      @return       void

                                      @pre          [none]

                                      @post         [none]

                                    ****************************************************************************
                                    */
static void CDCombineIntObjects(void) {
    /* Object combining struct. */

    CDObjectsInt.iNumberOfObjects = (ObjNumber_t)Envm_N_OBJECTS;

    CDObjectsInt.pGenObjList = VLCSEN_pEmGenObjList;
    CDObjectsInt.pARSObjList = VLCSEN_pEmARSObjList;
    CDObjectsInt.pVLCPrivObjList =
        (const VLCPrivObjectList_t
             *)&VLCObjectList; /* Added explicit cast to reduce compiler
                                  warnings in target compiler*/
    CDObjectsInt.pInternalObjectList = &CDExtObjectsInt;

    /* --- Output --- */
    CDOutputData.uiNumberOfHypotheses = (uint8)CD_NUMBER_OF_HYPOTHESES;
    CDOutputData.rgRelevantHypothesesList = &VLC_pCDHypothesesSen->Hypothesis;
    VLC_pCDHypothesesSen->Header.uiNumOfHypotheses =
        (uint8)CD_NUMBER_OF_HYPOTHESES;
    CDOutputData.pDegradation = &VLC_pCDHypothesesSen->Degradation;
}

/* **********************************************************************
  @fn            CDDeleteObject */ /*!


                                         @brief         Delete all Informations
                                       of CD if objects will be deleted

                                         @description   The function delete
                                       values from the CD component

                                         @param[in]     iObjectToDelete Object
                                       index in object list.

                                         @return        void

                                         @pre           [none]

                                         @post          [none]

                                       ****************************************************************************
                                       */
void CDDeleteObject(ObjNumber_t iObjectToDelete) {
    if (iObjectToDelete < Envm_N_OBJECTS) {
        CDDeleteInternalObject(&CDExtObjectsInt[iObjectToDelete]);
    }

    return;
}

/* **********************************************************************
  @fn            CDMergeObjects */ /*!


                                         @brief         Delete all information
                                       of CD if objects will be deleted

                                         @description   The function delete
                                       values from the CD component

                                         @param[in]     iObjectToKeep Index of
                                       the object that shall be kept after
                                       merge.
                                         @param[in]     iObjectToDelete Index of
                                       the object that shall be deleted by
                                       merge.

                                         @return        void

                                         @pre           [none]

                                         @post          [none]

                                       ****************************************************************************
                                       */
void CDMergeObjects(ObjNumber_t iObjectToKeep, ObjNumber_t iObjectToDelete) {
    if ((OBJ_LIFETIME_SEC(iObjectToDelete) >
         CD_OBJ_MERGE_CRIT_MERGE_MIN_OBJ_LFT) &&
        (OBJ_LIFETIME_SEC(iObjectToKeep) >
         CD_OBJ_MERGE_CRIT_MERGE_MIN_OBJ_LFT)) {
        float32 fPerc;
        float32 fDeltaT;
        float32 fMaxLifeTime;

        fMaxLifeTime = MAX_FLOAT(OBJ_LIFETIME_SEC(iObjectToDelete),
                                 OBJ_LIFETIME_SEC(iObjectToKeep));
        fDeltaT =
            OBJ_LIFETIME_SEC(iObjectToDelete) - OBJ_LIFETIME_SEC(iObjectToKeep);
        fPerc = fABS(fDeltaT) / fMaxLifeTime;
        if (fPerc < CD_OBJ_MERGE_CRIT_TIME_THRES_PERC) {
            float32 fLatDist;
            float32 fCycles;
            fLatDist = fABS(OBJ_LONG_DISPLACEMENT(iObjectToDelete) -
                            OBJ_LONG_DISPLACEMENT(iObjectToKeep));

            fCycles = BML_f_BoundedLinInterpol2(
                fLatDist, CD_OBJ_MERGE_CRIT_TIME_THRES_DISTY_0,
                CD_OBJ_MERGE_CRIT_TIME_THRES_DISTY_1,
                CD_OBJ_MERGE_CRIT_TIME_THRES_TIME_0,
                CD_OBJ_MERGE_CRIT_TIME_THRES_TIME_1);
            CDExtObjectsInt[iObjectToKeep].uiCriticalTimeAfterMerge =
                (uint8)fCycles;
        }
    }

    CDMergeInternalObjects(&CDExtObjectsInt[iObjectToKeep],
                           &CDExtObjectsInt[iObjectToDelete]);
}

/* **********************************************************************
  @fn            CDMergeDeleteObjectsSameVLCID */ /*!

                          @brief         CD specific merging and deleting of
                        objects if the object which is merged and
                                           the object into which the old object
                        was merged have the same VLC-ID

                          @description   CD specific merging and deleting of
                        objects if the object which is merged and
                                           the object into which the old object
                        was merged have the same VLC-ID

                          @param[in]     ObjNr : Index of the object that is
                        merged and into which the old object was merged.

                          @return        void

                          @pre           [none]

                          @post          [none]

                        ****************************************************************************
                        */
void CDMergeDeleteObjectsSameVLCID(ObjNumber_t const ObjNr) {
    /*! Take care: If new information considered here, check if the same
     * information should be considered in CDMergeObjects(...) !!! */

    /*! First: Store the information locally which is considered for merging */
    ubit16_t bCrossingLeft = CDExtObjectsInt[ObjNr].HypothesisHist.CrossingLeft;
    ubit16_t bCrossingRight =
        CDExtObjectsInt[ObjNr].HypothesisHist.CrossingRight;
    ubit16_t b_CutIn = CDExtObjectsInt[ObjNr].HypothesisHist.CutIn;
    ubit16_t b_Following = CDExtObjectsInt[ObjNr].HypothesisHist.Following;
    ubit16_t b_Pass = CDExtObjectsInt[ObjNr].HypothesisHist.Pass;
    ubit16_t b_RunUpMoving = CDExtObjectsInt[ObjNr].HypothesisHist.RunUpMoving;
    ubit16_t b_RunUpStationary =
        CDExtObjectsInt[ObjNr].HypothesisHist.RunUpStationary;
    ubit16_t b_WasOncomming =
        CDExtObjectsInt[ObjNr].HypothesisHist.WasOncomming;
    ubit16_t b_WasCrossing = CDExtObjectsInt[ObjNr].HypothesisHist.WasCrossing;
    ubit16_t b_PedColl = CDExtObjectsInt[ObjNr].HypothesisHist.PedColl;
    ubit16_t b_PedPass = CDExtObjectsInt[ObjNr].HypothesisHist.PedPass;
    /* Remark: ui_CriticalTimeAfterMerge is not considered for merge after
     * discussion with Patrick Hielscher (corresponding code in
     * CDMergeObjects(...) seems to be dead code) */

    /*! Second: Delete the VLC-object */
    CDDeleteObject(ObjNr);

    /*! Third: Copy merge-information */
    CDExtObjectsInt[ObjNr].HypothesisHist.CrossingLeft |= bCrossingLeft;
    CDExtObjectsInt[ObjNr].HypothesisHist.CrossingRight |= bCrossingRight;
    CDExtObjectsInt[ObjNr].HypothesisHist.CutIn |= b_CutIn;
    CDExtObjectsInt[ObjNr].HypothesisHist.Following |= b_Following;
    CDExtObjectsInt[ObjNr].HypothesisHist.Pass |= b_Pass;
    CDExtObjectsInt[ObjNr].HypothesisHist.RunUpMoving |= b_RunUpMoving;
    CDExtObjectsInt[ObjNr].HypothesisHist.RunUpStationary |= b_RunUpStationary;
    CDExtObjectsInt[ObjNr].HypothesisHist.WasOncomming |= b_WasOncomming;
    CDExtObjectsInt[ObjNr].HypothesisHist.WasCrossing |= b_WasCrossing;
    CDExtObjectsInt[ObjNr].HypothesisHist.PedColl |= b_PedColl;
    CDExtObjectsInt[ObjNr].HypothesisHist.PedPass |= b_PedPass;
}

/*************************************************************************************************************************
  Functionname:    CDCalculateOverlapInt */ /*!

                                                                                 @brief           return CP occupancies

                                                                                 @description     Detailed Design

                                                                                 @return          static void

                                                                                 @param[in]       pDistWidthInt : Distance widtnh structure.
                                                                                 @param[in,out]   pOccupancy : Occupancy returned.


                                                                               *************************************************************************************************************************/
static void CDCalculateOverlapInt(const CPDistanceWidth_t *pDistWidthInt,
                                  CPTrajOccupancy_t *pOccupancy) {
    CPCalculateOverlap(pDistWidthInt, pOccupancy);
}

/* **********************************************************************
  @fn            CDInit */ /*!

                                                 @brief         Initializes all
                                               CD variables at the beginning of
                                               the program run

                                                 @description   The function
                                               inituializes all values from the
                                               CD component

                                                 @return        void

                                                 @pre           [none]

                                                 @post          [none]


                                               ****************************************************************************
                                               */
/*CDInit is also called by simulation functions*/
void CDInit(void) {
    ObjNumber_t iObjectIndex;
    uint32 uiHypIndex;

    bWasInInit = TRUE;

    /* Initialize internal data structures */

    uiVDYSigErrorCnt = 0u;

    /* --- Input --- */

    /* Objects */

    CDCombineIntObjects();

    /* Ego dynamics */
    (void)memset(&CDEgoDynDataRawInt, 0, sizeof(CDEgoDynDataRawInt));

    (void)memset(&CDEgoDynDataSyncInt, 0, sizeof(CDEgoDynDataSyncInt));

    (void)memset(&CDEgoDynDataInt, 0, sizeof(CDEgoDynDataInt));
    CDEgoDynDataInt.pEgoDynRaw = &CDEgoDynDataRawInt;
    CDEgoDynDataInt.pEgoDynObjSync = &CDEgoDynDataSyncInt;

    /* Input combining struct. */
    (void)memset(&CDInputData, 0, sizeof(CDInputData));
    CDInputData.pObjectData = &CDObjectsInt;
    CDInputData.pEgoData = &CDEgoDynDataInt;

    /* --- Parameters --- */
    (void)memset(&CDAdjSafeDistanceInt, 0, sizeof(CDAdjSafeDistanceInt));

    (void)memset(&CDParametersInt, 0, sizeof(CDParametersInt));
    CDParametersInt.pAdjSafeDistance = &CDAdjSafeDistanceInt;

    /* Parameter for longitudinal acceleration*/
    CDParametersInt.pAdjSafeDistance->fLongNecRemainDist =
        CD_LONG_SAFETY_DISTANCE;
    /* Parameter for lateral acceleration*/
    CDParametersInt.pAdjSafeDistance->fLatNecRemainDist =
        CD_LAT_SAFETY_DISTANCE;

    /* --- Callbacks --- */
    (void)memset(&CDExternalFunctions, 0, sizeof(CDExternalFunctions));
    CDExternalFunctions.CPCalculateOverlap = &CDCalculateOverlapInt;

    /* --- State --- */
    (void)memset(&CDRelevantHypothesesListInt, 0,
                 sizeof(CDRelevantHypothesesListInt));
    (void)memset(&CDPreviousHypothesesListInt, 0,
                 sizeof(CDPreviousHypothesesListInt));

    for (uiHypIndex = 0u; uiHypIndex < CD_NUMBER_OF_HYPOTHESES; uiHypIndex++) {
        CDRelevantHypothesesListInt[uiHypIndex].eType = CDHypothesisType_No;
        CDPreviousHypothesesListInt[uiHypIndex].eType = CDHypothesisType_No;
    }

    (void)memset(&CDExtObjectsInt, 0, sizeof(CDExtObjectsInt));
    /* initialize internal objects */
    for (iObjectIndex = 0; iObjectIndex < Envm_N_OBJECTS; iObjectIndex++) {
        CDDeleteInternalObject(&CDExtObjectsInt[iObjectIndex]);
    }

    (void)memset(&CDStatusInt, 0, sizeof(CDStatusInt));
    CDStatusInt.rgObjInternal = &CDExtObjectsInt;
    CDStatusInt.rgIntRelevantHypothesesList = &CDRelevantHypothesesListInt;
    CDStatusInt.rgPreviousHypothesesList = &CDPreviousHypothesesListInt;

    /* --- Output --- */
    (void)memset(&CDOutputData, 0, sizeof(CDOutputData));
    CDOutputData.uiNumberOfHypotheses = (uint8)CD_NUMBER_OF_HYPOTHESES;
    CDOutputData.rgRelevantHypothesesList = &(VLC_pCDHypothesesSen->Hypothesis);
    VLC_pCDHypothesesSen->Header.uiNumOfHypotheses =
        (uint8)CD_NUMBER_OF_HYPOTHESES;

    CDOutputData.pDegradation = &VLC_pCDHypothesesSen->Degradation;
    (void)memset(CDOutputData.pDegradation, 0,
                 sizeof(*CDOutputData.pDegradation));

    CDInitEMPData();

    /* Initialize customer specific data */
    CDInitCustomerData();
}

/* ***********************************************************************
  @fn            CDInitEMPData                            */ /*!

                @brief         Initializes the EMP Data Fields

                @return        void

              ****************************************************************************
              */
static void CDInitEMPData(void) {
    /* initialize Ego Object Size */
    CDStatusInt.sEgoGeometry.fWidth = VLC_fEgoVehicleWidth;
    CDStatusInt.sEgoGeometry.fLength = VLC_fEgoVehicleLength;

    CDStatusInt.sKinEgo.fAccel = 0;
    CDStatusInt.sKinEgo.fVel = 0;
    CDStatusInt.sKinEgo.fYawRate = 0;
}

/* ***********************************************************************
  @fn            VLCCDProcess                            */ /*!


                 @brief         The function handles the APIA situation analysis

                 @description   The function is called by the global main
               routine and calls
                                all necessary functions of the CD component.

                 @return        void

                 @pre           [none]

                 @post          [none]

               ****************************************************************************
               */
void VLCCDProcess(void) {
    eGDBError_t locError = GDB_ERROR_NONE;

    CDInternalMeasurementData_t CDMeasurementDataInt;

    /*Initialize internal data*/
    (void)memset(&CDMeasurementDataInt, 0, sizeof(CDMeasurementDataInt));

    if (bWasInInit == FALSE) {
        CDState = CD_STATE_INIT;
    }

    switch (CDState) {
        case CD_STATE_OK:
            /* Post processing */
            CDPreProcessing();

            /* Run algo */
            locError = CDRun(&CDInputData, &CDStatusInt, &CDOutputData,
                             &CDParametersInt, &CDExternalFunctions,
                             &CDMeasurementDataInt);
            break;

        case CD_STATE_INIT:
        default:
            /* Initialize CD */
            CDInit();
            break;
    }

    if (locError == GDB_ERROR_NONE) {
        /* Post processing */
        CDPostProcessing();

        /* Send meas-freeze */
        CDCustomMTSFreezeData(&CDMeasurementDataInt);
    } else {
        /*reinit in case of error*/
        CDInit();
    }
}

/* ***********************************************************************
  @fn            CDCheckVDYInput                           */ /*!

               @brief         Check signal state of VDY input data.

               @description   Check signal state of VDY input data. Increases
             uiVDYSigErrorCnt
                              if input signal states are not valid, resets
             uiVDYSigErrorCnt when
                              input signal states are valid.

               @return        void

               @pre           [none]

               @post          [none]

             ****************************************************************************
             */
static void CDCheckVDYInput(void) {
    /* check for and count VDY input errors */
    if ((!IS_SIGNAL_STATUS_OK(EGO_SPEED_X_STATE)) ||
        (!IS_SIGNAL_STATUS_OK(EGO_ACCEL_X_STATE)) ||
        (!IS_SIGNAL_STATUS_OK(EGO_YAW_RATE_STATE))) {
        if (uiVDYSigErrorCnt < ((uint16)((~0uL) & BITMASK_UINT16))) {
            uiVDYSigErrorCnt = uiVDYSigErrorCnt + 1u;
        }
    } else {
        // TODO: CO: review if reasonable --> should be kept in maximum count
        uiVDYSigErrorCnt = 0u;
    }
}

/* ***********************************************************************
  @fn            CDSetSignalStatus                           */ /*!

             @brief         Sets signal status in signal header.

             @description   Sets signal status in signal header depending on
           CDState and
                            uiVDYSigErrorCnt.

             @return        void

             @pre           [none]

             @post          [none]

           ****************************************************************************
           */
static void CDSetSignalStatus(void) {
    /* Set data integrity flags (eSigStatus) */
    switch (CDState) {
        case CD_STATE_OK:
            if (uiVDYSigErrorCnt > CD_PERF_DEG_MAX_VDY_SIG_ERRORS) {
                /* keep SignalState INVALID (set in VLCSenSetProvideHeader*/
            } else {
                /* set SignalState from main require port */
                VLCSEN_pCDCustomOutput->sSigHeader.eSigStatus =
                    GET_Envm_PUB_OBJ_DATA_PTR->sSigHeader.eSigStatus;
                VLC_pCDHypothesesSen->sSigHeader.eSigStatus =
                    GET_Envm_PUB_OBJ_DATA_PTR->sSigHeader.eSigStatus;
            }
            break;

        case CD_STATE_INIT:
            VLC_pCDHypothesesSen->sSigHeader.eSigStatus =
                (AlgoSignalState_t)AL_SIG_STATE_INIT;
            VLCSEN_pCDCustomOutput->sSigHeader.eSigStatus =
                (AlgoSignalState_t)AL_SIG_STATE_INIT;

            break;

        default:
            VLC_pCDHypothesesSen->sSigHeader.eSigStatus =
                (AlgoSignalState_t)AL_SIG_STATE_INVALID;
            VLCSEN_pCDCustomOutput->sSigHeader.eSigStatus =
                (AlgoSignalState_t)AL_SIG_STATE_INVALID;

            break;
    }
}

/* ***********************************************************************
  @fn            CDPreProcessing                           */ /*!

               @brief         Processing before CD hypothesis calculation

               @description   Processing before CD hypothesis calculation

               @return        void


               @pre           [none]

               @post          [none]

             ****************************************************************************
             */
static void CDPreProcessing(void) {
    ObjNumber_t iObjectIndex;

    /* combine Objects to internal struct also in running mode because of IPC
     * ringbuffer*/
    CDCombineIntObjects();

    /* Ego dynamics */
    CDEgoDynDataRawInt.fAccelX = EGO_ACCEL_X_RAW;
    CDEgoDynDataRawInt.fAccelY = EGO_YAW_RATE_RAW * EGO_SPEED_X_RAW;
    CDEgoDynDataRawInt.fVelocityX = EGO_SPEED_X_RAW;
    CDEgoDynDataRawInt.fVelocityY = 0;

    CDEgoDynDataSyncInt.fAccelX = EGO_ACCEL_X_OBJ_SYNC;
    CDEgoDynDataSyncInt.fAccelY = EGO_YAW_RATE_OBJ_SYNC * EGO_SPEED_X_OBJ_SYNC;
    CDEgoDynDataSyncInt.fVelocityX = EGO_SPEED_X_OBJ_SYNC;
    CDEgoDynDataSyncInt.fVelocityY = 0;

    /* Gather objects, trajectories, ... */
    for (iObjectIndex = 0; iObjectIndex < Envm_N_OBJECTS; iObjectIndex++) {
        /* Object kinematics */
        if (!OBJ_IS_DELETED(iObjectIndex)) {
        } else {
            /* lane association */
        } /* if( !OBJ_IS_DELETED(iObjectIndex)) */

        /* count down critical merge time */
        if (CDExtObjectsInt[iObjectIndex].uiCriticalTimeAfterMerge > 0u) {
            CDExtObjectsInt[iObjectIndex].uiCriticalTimeAfterMerge--;
        }
    }
}

/* ***********************************************************************
  @fn            CDPostProcessing                           */ /*!

              @brief         Processing after CD hypothesis calculation

              @description   Processing after CD hypothesis calculation

              @return        void


              @pre           [none]

              @post          [none]

            ****************************************************************************
            */
static void CDPostProcessing(void) {
    CDCheckVDYInput();

    CDSetSignalStatus();

    { /* Output iBrake specific long. acceleration (collision prevention) */
        uint32 ii;
        float32 fAccel;
        CDMovement_t EgoMovement;
        CDMovement_t ObjMovement;
        CDAdjSafeDistance_t safe_dist;

        /* set Ego-Movement */
        CDFillEgoMovement(&EgoMovement, &CDInputData);

        /* set customer safety distance */
        safe_dist.fLongNecRemainDist = CD_LONG_SAFETY_DIST_CUST;
        safe_dist.fLatNecRemainDist = 0;

        for (ii = 0u; ii < CD_NUMBER_OF_HYPOTHESES; ii++) {
            if ((ii < CDOutputData.uiNumberOfHypotheses) &&
                ((*CDOutputData.rgRelevantHypothesesList)[ii].eType !=
                 (eCDHypothesisType_t)CDHypothesisType_No) &&
                ((*CDOutputData.rgRelevantHypothesesList)[ii].uiObjectRef >=
                 0)) {
                /* Select object */
                const ObjNumber_t objIdx =
                    (*CDOutputData.rgRelevantHypothesesList)[ii].uiObjectRef;

                /* set object movement */
                CDFillObjectMovement(objIdx, &ObjMovement, &CDInputData,
                                     &(CDExtObjectsInt[objIdx]));

                /* Calculate acceleration */
                (void)CDCalculateAnecLongLatency(&EgoMovement, &ObjMovement,
                                                 &safe_dist, &fAccel);
            } else {
                fAccel = 0;
            }
            /* Write output */
            VLCSEN_pCDCustomOutput->iBrkOutput.AnecLong[ii] = fAccel;
        }
        /* Make signal available for data acquisition support over vehicle
         * network
         *
         * @attention Check if the iBrkOutput.AllowedDistance distance matches
         * the max. distance configured in the PreBrake function activation
         * conditions.
         */
        VLCSEN_pCDCustomOutput->iBrkOutput.AllowedDistance =
            CDOutputData.pDegradation->Safety.fMaxDistALN;
        // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/51 changed by
        // guotao 20200615 start
        VLC_pCDHypothesesSen->Header.uiNumOfHypotheses =
            CDOutputData.uiNumberOfHypotheses;
        // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/51 changed by
        // guotao 20200615 end
    }
}

/* ***********************************************************************
  @fn            CDCustomMTSFreezeData                           */ /*!

         @brief         Freeze application specific MTS data.

         @description   Freeze application specific MTS data.

         @param[in]     pCDMeasurementDataInt Pointer to internal measurement
       data.

         @return        void

         @pre           [none]

         @post          [none]

       ****************************************************************************
       */
static void CDCustomMTSFreezeData(
    CDInternalMeasurementData_t *pCDMeasurementDataInt) {
    // static const MEASInfo_t cd_meas_header = {
    //    VLC_MEAS_ID_CGEB_CD_HYPOTHESES,      /*!< Virtual address */
    //    sizeof(CDInternalMeasurementData_t), /*!< Length in bytes */
    //    VLC_MEAS_FUNC_ID,                    /*!< Function ID */
    //    VLC_MEAS_FUNC_CHAN_ID                /*!< Function cycle ID */
    //};
    uint32 tCDState;
    if (pCDMeasurementDataInt != NULL) {
        /* Send meas-freeze */
        /* Make signal available for data acquisition support over vehicle
         * network
         *
         * @attention Check if the CDMeasurementDataInt.fAllowedBrakeDist
         * distance matches the max. distance configured in the PreBrake
         * function activation conditions.
         */

        pCDMeasurementDataInt->fAllowedBrakeDist =
            CDOutputData.pDegradation->Safety.fMaxDistALN;

        //(void) VLC_FREEZE_DATA(&cd_meas_header, (void*) pCDMeasurementDataInt,
        // NULL);
    }

    /* CD internal meas freezes */
    {
        MEASInfo_t mInfo;
        mInfo.FuncID = VLC_MEAS_FUNC_ID;
        mInfo.FuncChannelID = VLC_MEAS_FUNC_CHAN_ID;

        /* to force 4 byte alignment */
        tCDState = (uint32)CDState;
        mInfo.VirtualAddress = VLC_MEAS_ID_CGEB_CD_WRAP_CD_STATE;
        mInfo.Length = sizeof(tCDState);
        //(void) VLC_FREEZE_DATA(&mInfo, &tCDState, NULL);

        mInfo.VirtualAddress = VLC_MEAS_ID_CGEB_CD_WRAP_CD_ADJ_SAFE_INT;
        mInfo.Length = sizeof(CDAdjSafeDistanceInt);
        //(void) VLC_FREEZE_DATA(&mInfo, &CDAdjSafeDistanceInt, NULL);

        mInfo.VirtualAddress = VLC_MEAS_ID_CGEB_CD_WRAP_CD_REL_HYP_INT;
        mInfo.Length = sizeof(CDRelevantHypothesesListInt);
        //(void) VLC_FREEZE_DATA(&mInfo, &CDRelevantHypothesesListInt, NULL);

        /* Freeze internal state of hypothesis 0 object */
        //{
        //    ObjNumber_t objIdx = 0;
        //    /* Freeze internal object data for object attached to most
        //    relevant
        //     * hypothesis */
        //   mInfo.VirtualAddress = VLC_MEAS_ID_CGEB_CD_WRAP_CD_REL_OBJ_OUT;
        //   mInfo.Length = sizeof(CDInternalObject_t);
        //
        //   if (((*CDOutputData.rgRelevantHypothesesList)[0].eType !=
        //         (eCDHypothesisType_t)CDHypothesisType_No) &&
        //       ((*CDOutputData.rgRelevantHypothesesList)[0].uiObjectRef <
        //         Envm_N_OBJECTS)) {
        //       /* Select object */
        //       objIdx =
        //           (*CDOutputData.rgRelevantHypothesesList)[0].uiObjectRef;
        //   }
        //(void) VLC_FREEZE_DATA(&mInfo, &CDExtObjectsInt[objIdx], NULL);
        //}
    }
}

/* ***********************************************************************
  @fn            CDProcessEMPTrajectoryMeasFreeze                           */ /*!

@brief         The function return the pointer to the EMP Trajectory Data.

@description   The function process the EMPTrajPredEgo based on KinHist in
CDStatusInt

@param[out]    pEMPTrajPredEgo

@return        void

@pre           [none]

@post          [none]

**************************************************************************** */
void CDProcessEMPTrajectoryMeasFreeze(EMPTrajPred_t *pEMPTrajPredEgo) {
    EMPPredictEgoTraj(&(CDStatusInt.sKinEgo), EMP_MANEUVER_KinematicsUnchanged,
                      pEMPTrajPredEgo);
}

/* ***********************************************************************
  @fn            CDGetPointer_InputData                           */ /*!

        @brief         The function return the pointer to the internal CD input
      data.

        @description   The function return the pointer to the internal CD input
      data.

        @return        Returns a pointer to the CD input data.

        @pre           [none]

        @post          [none]

      ****************************************************************************
      */
CDInputData_t *CDGetPointer_InputData(void) { return &CDInputData; }

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */