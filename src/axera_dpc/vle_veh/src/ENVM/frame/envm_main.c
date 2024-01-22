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
#include "envm_ext.h"           // NOLINT
#include "envm_consts.h"        // NOLINT
#include "TM_Global_Types.h"    // NOLINT
#include "envm_common_utils.h"  // NOLINT
#include "stddef.h"             // NOLINT
#include "assert.h"             // NOLINT
#include "ops_ext.h"            // NOLINT
#include "fidm_ext.h"

// #include "TueObjFusn_Params.h"       // NOLINT
// #include "tue_Fusion.h"              // NOLINT
// #include "sfObjectFusionMpf_func.h"  // NOLINT

#include "tunnel_recognize.h"  // NOLINT
#ifdef LINUX_SYS
#include "BSW_DataLog_Server.h"  // NOLINT
#endif
// #include "ContiRadarConverter_int.h"  // NOLINT
#include "envm_io.h"  // NOLINT

// #include "tue_prv_fusion_math.h"  // NOLINT
/*****************************************************************************

*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! Default value for tunnel probability */
#define EM_DEFAULTTUNNELPROBABILITY (0.0f)

/*! Just a default value to check if static init is working correct. */
#define EM_MAIN_STATIC_INIT_CHECK (0xDB)

#define CONVERT_FOR_ACC 1
#define CONVERT_FOR_SRR 2

#define TUE_SINGAL_FUSION_OBJECT_ID_MAX 200u
#define FUSION_OBJECT_CYCLE_TIME 0.05f
#define FUSION_CONFIRM_DENSITY_SIZE 8U

/*****************************************************************************
  STRUCTURES AND TYPEDEFS
*****************************************************************************/

void FPSProcessMeasFreeze(void);

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
// this array saved the mapping relationship of FusionTrackID -> FPSObjectID.
// TrackIdToObjectIdMap[394] = 25, which meas 394th fusion track saved in the
// 25th element of EnvmData.pPrivObjList[]
// static int TrackIdToObjectIdMap[TUEOBJFUSN_IDPROVIDER_U16ID_MAX + 1];
// static TueObjFusn_ObjectListType pFusnObjLists_Output;
// static TueObjFusn_ErrorBufferType pErrorBuffer;
// static TueObjFusn_EgoMotionType EgoMotion = {0, 0, 0, 0., 0., 0., 0.};
ExtObjectList_t TEMP_pExtFuisonObjList;
ExtObjectList_t TEMP_pExtObjList;
// record object history measured frequency for different SSR sensor and
// different objects
static uint8 uaFusionHistoryMeasuredFrequency[TUE_SINGAL_FUSION_OBJECT_ID_MAX];
// record object life cycle record for different SSR sensor and different
// objects
static uint8 uaFusionObjectLifeCycle[TUE_SINGAL_FUSION_OBJECT_ID_MAX];
// record object history measured frequency for different SSR sensor and
// different objects
static uint8 uaHistoryMeasuredFrequency[TUE_SINGAL_FUSION_OBJECT_ID_MAX];
// record object life cycle record for different SSR sensor and different
// objects
static uint16 uaObjectLifeCycle[TUE_SINGAL_FUSION_OBJECT_ID_MAX];

static sint32 iFusionObjIDManageArray[EM_Fusion_GEN_OBJECT_NUM * 2];

static sint32 iFusionObjIndexManageArray[EM_Fusion_GEN_OBJECT_NUM];

// IDManage
sint32 iObjIDManageArray[Envm_NR_PRIVOBJECTS * 2] = {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

/*! em wrapper module state */
StateWRP_EM_t StateWrpEM;

/*! byte array to mark invalid AzAngle hypotheses of the RSPCluster */
a_AzAngleValidFar_array_t a_AzAngleValidFar;
a_AzAngleValidNear_array_t a_AzAngleValidNear;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static void EMInit(void);
void OPSPreprocess(float32 *Dist2TrajAbsArray,
                   uint32 *PrioClassTotalNum,
                   int *OPSPrioClass);
void BubbleSortBasedOnDist(float32 Dist2TrajAbsArray[EM_Fusion_GEN_OBJECT_NUM],
                           int *ObjIdSortedByDistance);
void CalcPrioClassObjNum(uint32 PrioClassTotalNum[TUE_OPS_PRIO_NUM_QUOTA],
                         uint32 *PrioClassSelectedNum);
void SelectObjectBaseOnPrio(int ObjIdSortedByDistance[EM_Fusion_GEN_OBJECT_NUM],
                            int OPSPrioClass[EM_Fusion_GEN_OBJECT_NUM],
                            uint32 PrioClassSelectedNum[TUE_OPS_PRIO_NUM_QUOTA],
                            int *OPSSelectedObjId);
void SenseTime_IDManage();
void TUEObjectPreSelection(void);
// uint8 TUETimeIndicatorFilter(uint8 uiInputTimeValue);
void TUESceneRecognize(void);
// void TUEObjectFusionProcess(uint32 uiSystemTimeStamp_us);
void SenseTime_ReplaceFusionAddCalculation();
// void ConvertFusionIdToEnvmDataId(TueObjFusn_ObjectListType fusionResultInput,
//                                  int* fusionToEnvmDataMap);
static void EM_CalcGenObjectRelativeKinematicData(
    const ExtObjectList_t *pExtObjList);
static void EMProcess(const reqEMPrtList_t *reqPorts,
                      proEnvmPrtList_t *proPorts);
static void EM_HMI_Output(const ST_SOA_HMI_t *reqPorts,
                          Envm_ADASHighlightID_t *proPorts);
static void EM_TrafficLightSignConvert(const TSRObjectList_t *pTSRObject,
                                       const uint32 uiSystemTimeStamp_us,
                                       Envm_TrafficLight_t *pTrafficLight,
                                       Envm_TrafficSign_t *pTrafficSign);
static void EM_FusionObjectOutput(
    const ExtObjectList_t *pExtFusionObjList,
    FusionIDManegeObjList_t *pFusionIDManageObjectList,
    const VEDVehPar_t *pGlobEgoStatic,
    const float32 fEgoVelX,
    const float32 fEgoAccelX,
    const float32 fEgoYawRate,
    const float32 fCosEgoHeadingAngle,
    const uint32 uiSystemTimeStamp_us,
    Envm_FusionObjectList_t *pFusionObjectOutputList);
static void FusionObjectIDManagement(
    const ExtObjectList_t *pExtFusionObjList,
    FusionIDManegeObjList_t *pFusionIDManageObjectList);
boolean CheckFusionObjectValidation(const Objects_t sExtObject);
static void FusionObjectKinematicProcess(const Objects_t sExtObject,
                                         const float32 fEgoVelX,
                                         const float32 fEgoAccelX,
                                         const float32 fEgoYawRate,
                                         const float32 fCosEgoHeadingAngle,
                                         EM_Fusion_GenKinematics_t *pOutputObj);
static void FusionObjectGeometryProcess(const Objects_t sExtObject,
                                        EM_Fusion_GenGeometry_t *pOutputObj);
static void FusionObjectGeneralProcess(const Objects_t sExtObject,
                                       EM_Fusion_GenGenerals_t *pOutputObj);
static void FusionObjectAttributeProcess(const Objects_t sExtObject,
                                         EM_Fusion_GenAttributes_t *pOutputObj);
static void FusionObjectQualifierProcess(const Objects_t sExtObject,
                                         EM_Fusion_GenQualifiers_t *pOutputObj);
static boolean EM_b_SetupRequestPorts(const reqEMPrtList_t *reqPorts);
static boolean EM_b_SetupProvidePorts(const proEnvmPrtList_t *proPorts);
static boolean EM_b_CheckMemory(void);
void Envm_v_CalcDeltaTime(Envm_t_TimeCheckpoint eTimeCheckpoint);
static void EM_v_FreezeDeltaTime(void);

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/
// #ifdef Envm_MEASUREnvmENT
void EM_MTS_Callback(void) {
    /* increase number of completed meas freezes */
    EnvmData.pPrivGlob->u_MeasFreezeCounterCallback++;
    return;
}
// #endif

/* ****************************************************************************

  Functionname:     EnvmGetDelayedSignal                                   */ /*!

    @brief            Gets the delayed signal from the shift buffer

    @description      Gets the delayed signal from the shift buffer based on
  cycle delay
                      and subcycle delay

    @param[in]        uiCycleDelay : number of complete cycle part the delay
    @param[in]        fSubcycleDelay : sub cycle remaining part of the Delay
    @param[in]        a_buffer :  pointer on the buffer containing the signal to
  be delayed
    @param[in]        fValue : current value of the raw signal

    @return           delayed and interpolated signal

    @pre              None
    @post             No changes


  ****************************************************************************
  */
f32_t EnvmGetDelayedSignal(ui32_t uiCycleDelay,
                           f32_t fSubcycleDelay,
                           const f32_t a_buffer[],
                           f32_t fValue) {
    f32_t fDelayedSig;

    /* get element [CycleDelay + 1] */
    fDelayedSig = fSubcycleDelay * a_buffer[uiCycleDelay];

    /* get element [CycleDelay] */
    if (uiCycleDelay > 0UL) {
        fDelayedSig += (1.0F - fSubcycleDelay) * a_buffer[uiCycleDelay - 1UL];
    } else {
        fDelayedSig += (1.0F - fSubcycleDelay) * fValue;
    }

    return fDelayedSig;
}

/* ****************************************************************************

  Functionname:     EMShiftDelayBuffer                                   */ /*!

      @brief            Gets the delayed signal from the shift buffer

      @description      Shifts delay buffer by one cycle and saves he current
    value

      @param[in]        a_buffer :  pointer on the buffer containing the signal
    to be delayed
      @param[in]        fValue : current value of the raw signal

      @return           void

      @pre              None
      @post             No changes


    ****************************************************************************
    */
void EMShiftDelayBuffer(f32_t a_buffer[], const f32_t fValue) {
    i32_t i;

    /* shift buffer one cycle */
    for (i = (i32_t)DELAY_BUFFER_SIZE - 1L; i > 0L; i--) {
        a_buffer[i] = a_buffer[i - 1L];
    }
    a_buffer[0] = fValue;
}

/* ****************************************************************************

  Functionname:     Envm_v_CalcDeltaTime                                   */ /*!

    @brief            Supports time measurement in EM

    @description      Updates time with current time, if Valid ECU Service
  functions
                      are available. Gets current time stamp. Saves timestamp at
  the
                      start of each cycle. In other checkpoints, calculates the
  delta
                      times from the previous checkpoint. Also saves current
  timestamp
                      for next delta time calculation.
                      In the last checkpoint overall EM time is calculated.

    @return           void

    @pre              None
    @post             No changes


  ****************************************************************************
  */
void Envm_v_CalcDeltaTime(Envm_t_TimeCheckpoint eTimeCheckpoint) {
    UNREFERENCED_FORMAL_PARAMETER(eTimeCheckpoint);
}

/* ****************************************************************************

  Functionname:     EM_v_FreezeDeltaTime                                   */ /*!

    @brief            Measfreeze of EM time measurment data

    @description      Measfreeze of EM time measurment data

    @return           void

    @pre              None
    @post             No changes


  ****************************************************************************
  */
static void EM_v_FreezeDeltaTime(void) {}

/* ****************************************************************************

  Functionname:     EMProcess                                            */ /*!

      @brief            EM Main processing

      @description      Calls all processing functions of EM and sub-components.
                        Calls Data Preprocessing management, object tracking,
                        Environment Observation, performance degradation,
    function
                        pre selection and EM Post processing.
                        Calculates delta times at each check point and freezes
                        measurement data.

      @param[in]        reqPorts : pointer on the required ports structure
      @param[in,out]    proPorts : pointer on the provided ports structure

      @return           void

      @pre              All states must be set
      @post             No changes


    ****************************************************************************
    */
uint32 tue_CycleTime = 0;
uint8 fristPrint = 1;
static void EMProcess(const reqEMPrtList_t *reqPorts,
                      proEnvmPrtList_t *proPorts) {
    // uint32  ui32_OldExecTime = 0;

    /*------------------------------------------------------------------*/
    /* Start Environment model Processing (EM) */
    /*------------------------------------------------------------------*/
    /* EM Main Time */
    Envm_v_CalcDeltaTime(Envm_TIME_CP_PROC);

    // changed by guotao 20200410 start
    // Tickt:http://119.3.139.124:10101/tickets/EM%2FOPS_100.git/3
    /*------------------------------------------------------------------*/
    /* Start scene recognize process, mainly about tunnel */
    /* added by guotao 20200331 */
    /*------------------------------------------------------------------*/
    TUESceneRecognize();
    // changed by guotao 20200410 end
    /* Start object pre-selection Function */
    /*------------------------------------------------------------------*/
    // object pre-select function guotao 20191121
    TUEObjectPreSelection();

    /*------------------------------------------------------------------*/
    // translate the object's data type from int32 to uint8
    SenseTime_IDManage();

    /* Start TTTA Object Fusion Function */
    /*------------------------------------------------------------------*/
    // object fusion function guotao 20190508
    // #ifndef TU_ALGO_SIMULATION_VS_CONF_200
    // #ifdef LINUX_SYS
    // ui32_OldExecTime = Norm_SYS_GetCurrentTime();
    // #else
    // ui32_OldExecTime = MPC5748G_TimeGetSinceBoot_us();
    // #endif
    // #endif
    // TUEObjectFusionProcess(reqPorts->pCtrl->uiTimeStamp_us);

    /*------------------------------------------------------------------*/
    /* replace EM Fusion additianal calculation*/
    /*------------------------------------------------------------------*/
    // Increased by shenzijian 20211012 start
    SenseTime_ReplaceFusionAddCalculation();

    // GET_Envm_INT_OBJ_DATA_PTR->eSigStatus = AL_SIG_STATE_OK;
    // #ifndef TU_ALGO_SIMULATION_VS_CONF
    // tue_CycleTime = MPC5748G_TimeGetSince_us(ui32_OldExecTime);
    // #endif
    /*------------------------------------------------------------------*/

    /* Start Function pre selection (FPS) */
    /*------------------------------------------------------------------*/
    TUESelectedObjPostProcess();
    /*------------------------------------------------------------------*/

    /*------------------------------------------------------------------*/
    /* output fusion object */
    /*------------------------------------------------------------------*/
    extern float32 f_GlobalObjSyncCosFloatAng;
    EM_FusionObjectOutput(
        EnvmData.pExtFusionObjList, EnvmData.pFusionIDManageObjectList,
        EnvmData.pGlobEgoStatic,
        GET_EGO_RAW_DATA_PTR->Longitudinal.MotVar.Velocity,
        GET_EGO_RAW_DATA_PTR->Longitudinal.MotVar.Accel,
        GET_EGO_RAW_DATA_PTR->Lateral.YawRate.YawRate,
        f_GlobalObjSyncCosFloatAng, reqPorts->pCtrl->uiTimeStamp_us,
        proPorts->pFusionObjectList);

    EM_TrafficLightSignConvert(EnvmData.pTSRObject,
                               reqPorts->pCtrl->uiTimeStamp_us,
                               proPorts->pTrafficLight, proPorts->pTrafficSign);

    Envm_v_CalcDeltaTime(EM_TIME_CP_FPS);
// EMSRRGenObjList_t *pSRRGenObjets  =
// Rte_IWriteRef_SWCEnvm_RPortEnvmSRRGenObjets_Buf(); if(fristPrint) {
// fristPrint = 0;
// Debug("EMProcess : %d, %d, %d", sizeof(uint32),
// sizeof(SignalHeader_t), sizeof(EMSRRGenObjectArray_t));
// Debug("EMProcess : %p, %p, %p, %p, %p, %p",
// &pSRRGenObjets->uiVersionNumber, &pSRRGenObjets->sSigHeader,
// &pSRRGenObjets->aFrontLeftObjects,
// &pSRRGenObjets->aFrontRightObjects,
// &pSRRGenObjets->aRearLeftObjects,
// &pSRRGenObjets->aRearRightObjects);
//  }
#ifdef LINUX_SYS
    DATALOGInfo_t Record;
    Record.StructID = Data_EMSRRGenObjList_type;
    Record.Length = sizeof(EMSRRGenObjList_t);
    Record.SocBuf = *proPorts->pSRRGenObjList;
    BSW_DataLog_FreezeData(Record);
#endif
}

/* ****************************************************************************

  Functionname:     EM_HMI_Output                                        */ /*!

      @brief            Output highlights the target ID with the warn level
      @description      EM aggregated ADAS feature highlights target ID with
                        warning level; Note that the obstacle ID here needs
                        to be consistent with the original fusion target ID
                        issued by the SOC side

      @param[in]        reqPorts : pointer on the required ports structure
      @param[in,out]    proPorts : pointer on the provided ports structure

      @return           void

      @pre              none
      @post             No changes


    ****************************************************************************
    */
static void EM_HMI_Output(const ST_SOA_HMI_t *reqPorts,
                          Envm_ADASHighlightID_t *proPorts) {
    // Init ADASHighlightID Output
    Envm_ADASHighlightID_t ADASHighlightID_Init = {
        -1, 0,  -1, 0,  -1, 0,  -1, 0,  -1, 0,  -1, 0,  -1,
        0,  -1, 0,  -1, 0,  -1, 0,  -1, 0,  -1, 0,  -1, 0};
    *proPorts = ADASHighlightID_Init;

    // proVLCVEH_SOA_HMI_t
    sint32 idFCW = reqPorts->reqVLCVEH_SOA_HMI.siFCWHighlightID_nu;
    if (idFCW == -1) {
        proPorts->siFCWHighlightID_nu = -1;
    } else {
        proPorts->siFCWHighlightID_nu = iObjIDManageArray[idFCW];
    }
    proPorts->uiFCWWarningLevel_enum =
        reqPorts->reqVLCVEH_SOA_HMI.uiFCWWarningLevel_enum;

    sint32 idAEB = reqPorts->reqVLCVEH_SOA_HMI.siAEBHighlightID_nu;
    if (idAEB == -1) {
        proPorts->siAEBHighlightID_nu = -1;
    } else {
        proPorts->siAEBHighlightID_nu = iObjIDManageArray[idAEB];
    }
    proPorts->uiAEBWarningLevel_enum =
        reqPorts->reqVLCVEH_SOA_HMI.uiAEBWarningLevel_enum;

    sint32 idACC = reqPorts->reqVLCVEH_SOA_HMI.siACCHighlightID_nu;
    if (idACC == -1) {
        proPorts->siACCHighlightID_nu = -1;
    } else {
        proPorts->siACCHighlightID_nu = iObjIDManageArray[idACC];
    }
    proPorts->uiACCWarningLevel_enum =
        reqPorts->reqVLCVEH_SOA_HMI.uiACCWarningLevel_enum;

    // proCTASEN_SOA_HMI_t
    proPorts->siFCTALeftHighlightID_nu =
        reqPorts->reqCTASEN_SOA_HMI.siFCTALeftHighlightID_nu;
    proPorts->uiFCTALeftWarningLevel_enum =
        reqPorts->reqCTASEN_SOA_HMI.uiFCTALeftWarningLevel_enum;

    proPorts->siFCTARightHighlightID_nu =
        reqPorts->reqCTASEN_SOA_HMI.siFCTARightHighlightID_nu;
    proPorts->uiFCTARightWarningLevel_enum =
        reqPorts->reqCTASEN_SOA_HMI.uiFCTARightWarningLevel_enum;

    proPorts->siRCTALeftHighlightID_nu =
        reqPorts->reqCTASEN_SOA_HMI.siRCTALeftHighlightID_nu;
    proPorts->uiRCTALeftWarningLevel_enum =
        reqPorts->reqCTASEN_SOA_HMI.uiRCTALeftWarningLevel_enum;

    proPorts->siRCTARightHighlightID_nu =
        reqPorts->reqCTASEN_SOA_HMI.siRCTARightHighlightID_nu;
    proPorts->uiRCTARightWarningLevel_enum =
        reqPorts->reqCTASEN_SOA_HMI.uiRCTARightWarningLevel_enum;
    // proLBSSEN_SOA_HMI_t
    proPorts->siLCALeftHighlightID_nu =
        reqPorts->reqLBSSEN_SOA_HMI.siLCALeftHighlightID_nu;
    proPorts->uiLCALeftWarningLevel_enum =
        reqPorts->reqLBSSEN_SOA_HMI.uiLCALeftWarningLevel_enum;

    proPorts->siLCARightHighlightID_nu =
        reqPorts->reqLBSSEN_SOA_HMI.siLCARightHighlightID_nu;
    proPorts->uiLCARightWarningLevel_enum =
        reqPorts->reqLBSSEN_SOA_HMI.uiLCARightWarningLevel_enum;

    proPorts->siDOWLeftHighlightID_nu =
        reqPorts->reqLBSSEN_SOA_HMI.siDOWLeftHighlightID_nu;
    proPorts->uiDOWLeftWarningLevel_enum =
        reqPorts->reqLBSSEN_SOA_HMI.uiDOWLeftWarningLevel_enum;

    proPorts->siDOWRightHighlightID_nu =
        reqPorts->reqLBSSEN_SOA_HMI.siDOWRightHighlightID_nu;
    proPorts->uiDOWRightWarningLevel_enum =
        reqPorts->reqLBSSEN_SOA_HMI.uiDOWRightWarningLevel_enum;

    proPorts->siRCWHighlightID_nu =
        reqPorts->reqLBSSEN_SOA_HMI.siRCWHighlightID_nu;
    proPorts->uiRCWWarningLevel_enum =
        reqPorts->reqLBSSEN_SOA_HMI.uiRCWWarningLevel_enum;
    // proLCFSEN_SOA_HMI_t
    proPorts->siALCHighlightID_nu =
        reqPorts->reqLCFSEN_SOA_HMI.siALCHighlightID_nu;
    proPorts->uiALCWarningLevel_enum =
        reqPorts->reqLCFSEN_SOA_HMI.uiALCWarningLevel_enum;
}

/* ****************************************************************************

  Functionname:     EMInitRegionOfInterestInternal                       */ /*!

      @brief            Initialization of region of interest

      @description      Initialization of component part: region of interest.
                        Shall be called once before processing starts

      @return           void

      @pre              None
      @post             All internal values and all interfaces are initialized
                        to default values


    ****************************************************************************
    */
void EMInitRegionOfInterestInternal(void) {}

/* ****************************************************************************

  Functionname:     EMInit                                               */ /*!

      @brief            Initialization of the component

      @description      Initialization of all internal data storage.
                        Shall be called once before processing starts
                        Resets RM global data, EM time data, EM error trap data,
                        initialize SigCheck functionality, complete internal
    object
                        list memory, em cycle counter, RSPInputEM, EM MeasFreeze
    and
                        EM customer data.

      @return           void

      @pre              None
      @post             All internal values and all interfaces are initialized
                        to default values


    ****************************************************************************
    */
static void EMInit(void) {
    // init object fusion function guotao 20190508
    // memset(TrackIdToObjectIdMap, -1,
    //        sizeof(int) * (TUEOBJFUSN_IDPROVIDER_U16ID_MAX + 1));

    // sfObjectFusionMpf_Init_wrapper(
    //     1026, 20.0f, 0.1f, 0.01f, 0.1f, 0.01f, TRUE, FALSE,
    //     TUEOBJFUSN_PARAMS_BOUTPUTISOVERGROUND_DEFAULT, TRUE, FALSE);
    // sfObjectFusionMpf_Init_wrapper(
    // TUEOBJFUSN_PARAMS_U32SENSORMODE_DEFAULT,
    // TUEOBJFUSN_PARAMS_F32MATCHGATE_DEFAULT,
    // TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINXFORQ_DEFAULT,
    // TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINYFORQ_DEFAULT,
    // TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINXFORQ_DEFAULT,
    // TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINYFORQ_DEFAULT,
    // TRUE,
    // FALSE,
    // TUEOBJFUSN_PARAMS_BOUTPUTISOVERGROUND_DEFAULT,
    // TUEOBJFUSN_PARAMS_BUSECOASTING_DEFAULT,
    // FALSE);//set default value for testing now

    /*--- VARIABLES ---*/

    /* explicit reset of static EM glob data */
    (void)memset(&EMGlob, 0u, sizeof(EMGlob_t));

    /* explicit reset of static EM time data */
    (void)memset(&EM_a_TimeArray, 0u, sizeof(Envm_t_TimeArray));

    /* Init EM error trap */
    EMErrorTrapInitGlobalData();

    v_EMSigCheckInit();

    // (void)memset(&(ExtSRRObjects), 0u, sizeof(ExtSRRObjList_array_t));
    // ExtSRRObjects.uiVersionNumber = 1;
    // /* global reset of complete internal object list memory */
    // (void)memset(&(EnvmExternalObj),      0u,sizeof(EnvmExternalObj));
    // EnvmExternalObj.uiVersionNumber = Envm_OBJECT_LIST_INTFVER;

    // //memory for mini-eye camera object list liuyang 20190513
    // (void)memset(&(CamObj_Sense), 0u, sizeof(CamObjectList));
    // CamObj_Sense.uiVersionNumber = Envm_TUE_CAM_OBJECT_LIST_INTFVER;

    EnvmData.pFrame->bValidInputPorts = FALSE;

    EnvmData.pECAMtCyclEnvmode->fECAMtCycleTime = TASK_CYCLE_TIME_50;

    /*! Init the em cycle counter */
    Envm_CYCLE_COUNTER = 0u;
    // levi 2019-01-22 move from dap to here.
    /* initialize em_input variables */
    EnvmInputInit();

    memset(iFusionObjIDManageArray, -1,
           sizeof(sint32) * (EM_Fusion_GEN_OBJECT_NUM * 2));
    memset(iFusionObjIndexManageArray, -1,
           sizeof(sint32) * EM_Fusion_GEN_OBJECT_NUM);
}

/* ****************************************************************************

  Functionname:     OPSPreprocess */ /*!

                                             @brief            Object
                                           Pre-selection Preprocess

                                             @description      calculate
                                           distance to ego trajectory and object
                                           priority class
                                                                                   property for each object

                                             @param            void

                                             @return           void

                                             @pre              None
                                             @post             No changes


                                           ****************************************************************************
                                           */
void OPSPreprocess(float32 *Dist2TrajAbsArray,
                   uint32 *PrioClassTotalNum,
                   int *OPSPrioClass) {
    for (uint32 ui_Index = 0u; ui_Index < EM_Fusion_GEN_OBJECT_NUM;
         ui_Index++) {
        if (GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                    .General.eObjMaintenanceState != MT_STATE_DELETED &&
            GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                    .General.eObjMaintenanceState != MT_STATE_MERGE_DELETED &&
            GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                    .Qualifiers.fProbabilityOfExistence > 0 &&
            EnvmData.pExtObjList->Objects[ui_Index].Kinematic.fDistX >= -1.f) {
            /*! Distance of the object to the VED ego predicted course */
            BML_t_TrajRefPoint DistVEYPoint2Circle =
                Tue_CML_CalculateDistancePoint2Circle(
                    GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                        .Kinematic.fDistX,
                    GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                        .Kinematic.fDistY,
                    EGO_CURVE_OBJ_SYNC);
            float32 f_Dist2TrajAbs = fABS(DistVEYPoint2Circle.f_DistToTraj);
            Dist2TrajAbsArray[ui_Index] = f_Dist2TrajAbs;
            // set object priority class
            switch (GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                        .Attributes.eDynamicProperty) {
                // moving object
                case TU_OBJECT_DYNPROP_MOVING:
                case TU_OBJECT_DYNPROP_CROSSING_MOVING:
                    if (f_Dist2TrajAbs <=
                        TUE_OPS_DISTANCE_TO_TRAJECTORY_THRESHOLD) {
                        if (GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                                .Qualifiers.fProbabilityOfExistence >=
                            TUE_OPS_POE_THRESHOLD) {
                            OPSPrioClass[ui_Index] =
                                OPS_CLASS_PRIO_DYNAMIC_POE_AND_CLOSE;
                            PrioClassTotalNum
                                [OPS_CLASS_PRIO_DYNAMIC_POE_AND_CLOSE]++;
                        } else {
                            OPSPrioClass[ui_Index] =
                                OPS_CLASS_PRIO_DYNAMIC_POE_OR_CLOSE;
                            PrioClassTotalNum
                                [OPS_CLASS_PRIO_DYNAMIC_POE_OR_CLOSE]++;
                        }
                    } else {
                        if (GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                                .Qualifiers.fProbabilityOfExistence >=
                            TUE_OPS_POE_THRESHOLD) {
                            OPSPrioClass[ui_Index] =
                                OPS_CLASS_PRIO_DYNAMIC_POE_OR_CLOSE;
                            PrioClassTotalNum
                                [OPS_CLASS_PRIO_DYNAMIC_POE_OR_CLOSE]++;
                        } else {
                            OPSPrioClass[ui_Index] = OPS_CLASS_PRIO_OTHER;
                            PrioClassTotalNum[OPS_CLASS_PRIO_OTHER]++;
                        }
                    }

                    break;
                // oncoming object
                case TU_OBJECT_DYNPROP_ONCOMING:
                    if (f_Dist2TrajAbs <=
                        TUE_OPS_DISTANCE_TO_TRAJECTORY_THRESHOLD) {
                        OPSPrioClass[ui_Index] =
                            OPS_CLASS_PRIO_ONCOMING_POE_OR_CLOSE;
                        PrioClassTotalNum
                            [OPS_CLASS_PRIO_ONCOMING_POE_OR_CLOSE]++;
                    } else if (GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                                   .Qualifiers.fProbabilityOfExistence >=
                               TUE_OPS_POE_THRESHOLD) {
                        OPSPrioClass[ui_Index] =
                            OPS_CLASS_PRIO_ONCOMING_POE_OR_CLOSE;
                        PrioClassTotalNum
                            [OPS_CLASS_PRIO_ONCOMING_POE_OR_CLOSE]++;
                    } else {
                        OPSPrioClass[ui_Index] = OPS_CLASS_PRIO_OTHER;
                        PrioClassTotalNum[OPS_CLASS_PRIO_OTHER]++;
                    }
                    break;

                // stationary object
                case TU_OBJECT_DYNPROP_STATIONARY:
                case TU_OBJECT_DYNPROP_STATIONARY_CANDITATE:
                case TU_OBJECT_DYNPROP_CROSSING_STATIONARY:
                case TU_OBJECT_DYNPROP_STOPPED:
                case TU_OBJECT_DYNPROP_UNKNOWN:
                default:
                    if (f_Dist2TrajAbs <=
                        TUE_OPS_DISTANCE_TO_TRAJECTORY_THRESHOLD) {
                        if (GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                                .Qualifiers.fProbabilityOfExistence >=
                            TUE_OPS_POE_THRESHOLD) {
                            OPSPrioClass[ui_Index] =
                                OPS_CLASS_PRIO_STATIC_POE_AND_CLOSE;
                            PrioClassTotalNum
                                [OPS_CLASS_PRIO_STATIC_POE_AND_CLOSE]++;
                        } else {
                            OPSPrioClass[ui_Index] =
                                OPS_CLASS_PRIO_STATIC_POE_OR_CLOSE;
                            PrioClassTotalNum
                                [OPS_CLASS_PRIO_STATIC_POE_OR_CLOSE]++;
                        }
                    } else {
                        if (GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                                .Qualifiers.fProbabilityOfExistence >=
                            TUE_OPS_POE_THRESHOLD) {
                            OPSPrioClass[ui_Index] =
                                OPS_CLASS_PRIO_STATIC_POE_OR_CLOSE;
                            PrioClassTotalNum
                                [OPS_CLASS_PRIO_STATIC_POE_OR_CLOSE]++;
                        } else {
                            OPSPrioClass[ui_Index] = OPS_CLASS_PRIO_OTHER;
                            PrioClassTotalNum[OPS_CLASS_PRIO_OTHER]++;
                        }
                    }
                    break;
            }
        }
    }
}
/* ****************************************************************************

  Functionname:     BubbleSortBasedOnDist */ /*!

                                     @brief            sort objects based on
                                   thier distance to ego trajectory

                                     @description      sort objects based on
                                   thier distance to ego trajectory

                                     @param            void

                                     @return           void

                                     @pre              None
                                     @post             No changes


                                   ****************************************************************************
                                   */
void BubbleSortBasedOnDist(float32 Dist2TrajAbsArray[EM_Fusion_GEN_OBJECT_NUM],
                           int *ObjIdSortedByDistance) {
    uint32 insertNum = 0;
    for (uint32 i = 0; i < EM_Fusion_GEN_OBJECT_NUM; i++) {
        float32 tempMinDist = -1;
        int arrayIndex = -1;
        for (uint32 j = 0; j < EM_Fusion_GEN_OBJECT_NUM; j++) {
            if (Dist2TrajAbsArray[j] >= 0) {
                if (tempMinDist == -1 || tempMinDist > Dist2TrajAbsArray[j]) {
                    tempMinDist = Dist2TrajAbsArray[j];
                    arrayIndex = j;
                }
            }
        }
        if (arrayIndex >= 0 && arrayIndex < EM_Fusion_GEN_OBJECT_NUM &&
            insertNum < EM_Fusion_GEN_OBJECT_NUM) {
            Dist2TrajAbsArray[arrayIndex] = -1;
            ObjIdSortedByDistance[insertNum] = arrayIndex;
            insertNum++;
        }
    }
}
/* ****************************************************************************

  Functionname:     CalcPrioClassObjNum */ /*!

    @brief            calculate object number
    in each priority class

    @description      calculate object number
    in each priority class based on thier
                                            distance
    to trajectory and priority class property

    @param            void

    @return           void

    @pre              None
    @post             No changes

    **************************************************************************/
void CalcPrioClassObjNum(uint32 PrioClassTotalNum[TUE_OPS_PRIO_NUM_QUOTA],
                         uint32 *PrioClassSelectedNum) {
    uint8 TotalNeededObjNum = CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
    for (uint8 i = 0; i < TUE_OPS_PRIO_NUM_QUOTA; i++) {
        if (tue_OpsPrioQuota[i] <= PrioClassTotalNum[i]) {
            PrioClassTotalNum[i] -= tue_OpsPrioQuota[i];
            TotalNeededObjNum -= tue_OpsPrioQuota[i];
            PrioClassSelectedNum[i] = tue_OpsPrioQuota[i];
        } else {
            TotalNeededObjNum -= PrioClassTotalNum[i];
            PrioClassSelectedNum[i] = PrioClassTotalNum[i];
            PrioClassTotalNum[i] = 0;
        }
    }
    for (uint8 i = 0; i < TUE_OPS_PRIO_NUM_QUOTA && TotalNeededObjNum > 0;
         i++) {
        uint8 addedObjNum = MIN(PrioClassTotalNum[i], TotalNeededObjNum);
        TotalNeededObjNum -= addedObjNum;
        PrioClassSelectedNum[i] += addedObjNum;
    }
}
/* ****************************************************************************

  Functionname:     SelectObjectBaseOnPrio */ /*!

                                    @brief            Object pre-selection
                                  result process

                                    @description      calculate and return
                                  object id array for OPS function based on
                                  thier
                                                                          priority
                                  class property and distance to ego trajectory

                                    @param            void

                                    @return           void

                                    @pre              None
                                    @post             No changes


                                  ****************************************************************************
                                  */
void SelectObjectBaseOnPrio(int ObjIdSortedByDistance[EM_Fusion_GEN_OBJECT_NUM],
                            int OPSPrioClass[EM_Fusion_GEN_OBJECT_NUM],
                            uint32 PrioClassSelectedNum[TUE_OPS_PRIO_NUM_QUOTA],
                            int *OPSSelectedObjId) {
    uint8 insertObjNum = 0;
    for (uint32 i = 0; i < EM_Fusion_GEN_OBJECT_NUM; i++) {
        if (ObjIdSortedByDistance[i] >= 0 &&
            OPSPrioClass[ObjIdSortedByDistance[i]] >= 0 &&
            OPSPrioClass[ObjIdSortedByDistance[i]] <
                OPS_CLASS_NUM_PRIO_CLASSES &&
            insertObjNum < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX &&
            PrioClassSelectedNum[OPSPrioClass[ObjIdSortedByDistance[i]]] > 0) {
            PrioClassSelectedNum[OPSPrioClass[ObjIdSortedByDistance[i]]]--;
            OPSSelectedObjId[insertObjNum] = ObjIdSortedByDistance[i];
            insertObjNum++;
        }
    }
}

/* ****************************************************************************

  Functionname:     TUETimeIndicatorFilter */ /*!

                                    @brief            a filter for camera time
                                  indictor input

                                    @description      a filter for camera time
                                  indictor input to reduce wrong recognize of
                                  time

                                    @param            void

                                    @return           void

                                    @pre              None
                                    @post             No changes


                                  ****************************************************************************
                                  */
// uint8 TUETimeIndicatorFilter(uint8 uiInputTimeValue)
//{
//    const uint8 uiHistorySize = 5u;
//    static uint8  uiInputValHistArray[5];
//    static uint8 uiInputValTimes = 0u;
//    uint8 uiResult = 0u;
//    if (uiInputTimeValue == TUE_MINIEYE_CAMERA_TIME_INDICATOR_DAY ||
//    uiInputTimeValue == TUE_MINIEYE_CAMERA_TIME_INDICATOR_NIGHT)
//    {
//        if (uiInputValTimes < uiHistorySize)
//        {
//            uiInputValHistArray[uiInputValTimes] = uiInputTimeValue;
//            uiInputValTimes++;
//        }
//        else
//        {
//            for (int i = 0; i < uiHistorySize - 1; i++)
//            {
//                uiInputValHistArray[i] = uiInputValHistArray[i + 1];
//            }
//            uiInputValHistArray[uiHistorySize - 1] = uiInputTimeValue;
//        }
//
//        uint8 uiTotalValue = 0;
//        for (int i = 0; i < uiInputValTimes; i++)
//        {
//            uiTotalValue += uiInputValHistArray[i];
//        }
//        uiResult = (uiTotalValue) > (uiInputValTimes / 2);
//    }
//    else
//    {
//        //intput value invalid, reset
//        uiInputValTimes = 0u;
//        memset(uiInputValHistArray, 0, sizeof(uiInputValHistArray));
//    }
//    return uiResult;
//}

/* ****************************************************************************

  Functionname:     TUESceneRecognize */ /*!

                                         @brief            special secene
                                       recognize like tunnel

                                         @description      special secene
                                       recognize like tunnel, bridge or
                                       something

                                         @param            void

                                         @return           void

                                         @pre              None
                                         @post             No changes


                                       ****************************************************************************
                                       */
void TUESceneRecognize(void) {
// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/3 changed by
// guotao 20200417 start
// EnvmData.pPrivObjList->sSceneDescription.uiFilteredTimeIndicator =
// TUETimeIndicatorFilter(EnvmData.pCamObject->sCameraAEBWarning.u8TimeIndicator);
// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/3 changed by
// guotao 20200417 end
#if ALGO_CARSIM_SWITCH
    // we only have radar project in carsim, we can only triggir deceleration in
    // night without camera confirmed
    EnvmData.pPrivObjList->sSceneDescription.uiFilteredTimeIndicator =
        TUE_MINIEYE_CAMERA_TIME_INDICATOR_NIGHT;
#endif
    // Tickt:http://119.3.139.124:10101/tickets/EM%2FOPS_100.git/3 changed by
    // guotao 20200410 start tunnel recognization
    EnvmData.pPrivObjList->sSceneDescription.uiTunnelProb = TUETunnelRecognize(
        GET_Envm_EXT_OBJ_DATA_PTR, EGO_CURVE_OBJ_SYNC, EGO_SPEED_X_CORRECTED);
    // Tickt:http://119.3.139.124:10101/tickets/EM%2FOPS_100.git/3 changed by
    // guotao 20200410 end
}

/* ****************************************************************************

  Functionname:     TUEObjectPreSelection */ /*!

                                   @brief            ID manage function from
                                   fusion to EM

                                   @description      we need to translate the
                                   object's data type from int32 to
                                   uint8.

                                   @param            void

                                   @return           void

                                   @pre              None
                                   @post             No changes


                                   ****************************************************************************
                                   */
void SenseTime_IDManage() {
    static sint32 iObjIndexManageArray[Envm_NR_PRIVOBJECTS] = {
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    boolean iCurrentCycleObjID[Envm_NR_PRIVOBJECTS * 2] = {0};

    SenseTime_Memcpy(&(EnvmData.pObjPreSeletList->sSigHeader),
                     &(EnvmData.pPrivObjList->sSigHeader),
                     sizeof(ENVMSignalHeader_t));
    EnvmData.pPrivObjList->eSigStatus = EnvmData.pObjPreSeletList->eSigStatus;
    EnvmData.pPrivObjList->uiCycleCounter =
        EnvmData.pObjPreSeletList->uiCycleCounter;
    EnvmData.pPrivObjList->uiTimeStamp = EnvmData.pObjPreSeletList->uiTimeStamp;
    EnvmData.pPrivObjList->uiVersionNumber =
        EnvmData.pObjPreSeletList->uiVersionNumber;

    if (EnvmData.pObjPreSeletList->HeaderObjList.iNumOfUsedObjects == 0) {
        memset(&EnvmData.pPrivObjList->HeaderObjList, 0u,
               sizeof(HeaderObjList_t));
        for (uint8 i = 0u; i < Envm_NR_PRIVOBJECTS; i++) {
            (void)memset(&EnvmData.pPrivObjList->Objects[i], 0u,
                         sizeof(Objects_t));
        }
    } else {
        for (uint8 i = 0u; i < Envm_NR_PRIVOBJECTS; i++) {
            boolean bFindTarget = FALSE;
            for (uint8 j = 0; j < Envm_NR_PRIVOBJECTS * 2; j++) {
                if (iObjIDManageArray[j] ==
                    EnvmData.pObjPreSeletList->Objects[i].ObjectId) {
                    bFindTarget = TRUE;
                    iCurrentCycleObjID[j] = 1;
                    break;
                }
            }
            // find vacant index for increasing ID
            if (!bFindTarget) {
                for (uint8 k = 0u; k < Envm_NR_PRIVOBJECTS * 2; k++) {
                    if (iObjIDManageArray[k] == -1) {
                        iObjIDManageArray[k] =
                            EnvmData.pObjPreSeletList->Objects[i].ObjectId;
                        iCurrentCycleObjID[k] = 1;
                        break;
                    }
                }
            }
        }
        // delet index of non-existent objects in current cycle
        for (uint8 j = 0u; j < Envm_NR_PRIVOBJECTS * 2; j++) {
            if (iCurrentCycleObjID[j] != 1) {
                iObjIDManageArray[j] = -1;
            }
        }

        for (uint8 i = 0u; i < Envm_NR_PRIVOBJECTS; i++) {
            if (iCurrentCycleObjID[iObjIndexManageArray[i]] == 0 &&
                iObjIndexManageArray[i] != -1) {
                iObjIndexManageArray[i] = -1;
            }
        }
        //
        for (uint8 i = 0u; i < Envm_NR_PRIVOBJECTS * 2; i++) {
            boolean bfindIndex = FALSE;
            for (uint8 j = 0u; j < Envm_NR_PRIVOBJECTS; j++) {
                if (i == iObjIndexManageArray[j] &&  // iObjIDManageArray[i]
                    iObjIDManageArray[i] != -1) {
                    bfindIndex = TRUE;
                    iObjIndexManageArray[j] = i;
                    break;
                }
            }
            if (!bfindIndex) {
                for (uint8 k = 0u; k < Envm_NR_PRIVOBJECTS; k++) {
                    if (iObjIDManageArray[i] != -1 &&
                        iObjIndexManageArray[k] == -1) {
                        iObjIndexManageArray[k] = i;
                        break;
                    }
                }
            }
        }

        // clear all the object data before result output
        for (uint8 i = 0u; i < Envm_NR_PRIVOBJECTS; i++) {
            (void)memset(&EnvmData.pPrivObjList->Objects[i], 0u,
                         sizeof(Objects_t));
        }
        // output to EnvmData.pPrivObjList from EnvmData.pObjPreSeletList
        for (uint8 i = 0u; i < Envm_NR_PRIVOBJECTS; i++) {
            for (uint8 j = 0u; j < Envm_NR_PRIVOBJECTS; j++) {
                if (iObjIndexManageArray[j] != -1 &&
                    iObjIDManageArray[iObjIndexManageArray[j]] ==
                        EnvmData.pObjPreSeletList->Objects[i].ObjectId) {
                    EnvmData.pPrivObjList->Objects[j].ObjectId =
                        iObjIndexManageArray[j];
                    SenseTime_Memcpy(&EnvmData.pObjPreSeletList->HeaderObjList,
                                     &EnvmData.pPrivObjList->HeaderObjList,
                                     sizeof(HeaderObjList_t));
                    SenseTime_Memcpy(
                        &EnvmData.pObjPreSeletList->Objects[i].Attributes,
                        &EnvmData.pPrivObjList->Objects[j].Attributes,
                        sizeof(Attributes_t));
                    SenseTime_Memcpy(
                        &EnvmData.pObjPreSeletList->Objects[i].EBAPresel,
                        &EnvmData.pPrivObjList->Objects[j].EBAPresel,
                        sizeof(EBAPresel_t));
                    SenseTime_Memcpy(
                        &EnvmData.pObjPreSeletList->Objects[i].General,
                        &EnvmData.pPrivObjList->Objects[j].General,
                        sizeof(General_t));
                    SenseTime_Memcpy(
                        &EnvmData.pObjPreSeletList->Objects[i].Geometry,
                        &EnvmData.pPrivObjList->Objects[j].Geometry,
                        sizeof(Geometry_t));
                    SenseTime_Memcpy(
                        &EnvmData.pObjPreSeletList->Objects[i].Kinematic,
                        &EnvmData.pPrivObjList->Objects[j].Kinematic,
                        sizeof(Kinematic_t));
                    SenseTime_Memcpy(
                        &EnvmData.pObjPreSeletList->Objects[i].Legacy,
                        &EnvmData.pPrivObjList->Objects[j].Legacy,
                        sizeof(LegacyObj_t));
                    SenseTime_Memcpy(
                        &EnvmData.pObjPreSeletList->Objects[i].Qualifiers,
                        &EnvmData.pPrivObjList->Objects[j].Qualifiers,
                        sizeof(Qualifiers_t));
                    SenseTime_Memcpy(
                        &EnvmData.pObjPreSeletList->Objects[i].SensorSpecific,
                        &EnvmData.pPrivObjList->Objects[j].SensorSpecific,
                        sizeof(SensorSpecific_t));
                    break;
                }
            }
        }
    }
}

/* ****************************************************************************

  Functionname:     TUEObjectPreSelection */ /*!

                                     @brief            object preselection main
                                   function

                                     @description      consider the cpu loading,
                                   we need to delete un-important object
                                                                            from
                                   radar.The main process is get 20 core radar
                                   objects from
                                                                           40
                                   radar object input list.

                                     @param            void

                                     @return           void

                                     @pre              None
                                     @post             No changes


                                   ****************************************************************************
                                   */

void TUEObjectPreSelection(void) {
    static int
        OPSPrioClass[EM_Fusion_GEN_OBJECT_NUM];  // store the priority class
                                                 // data for every object input
    static uint32
        PrioClassTotalNum[TUE_OPS_PRIO_NUM_QUOTA];  // store object total
                                                    // priority calss number
    static uint32
        PrioClassSelectedNum[TUE_OPS_PRIO_NUM_QUOTA];  // store selected object
                                                       // number for each
                                                       // priority class
    static int ObjIdSortedByDistance[EM_Fusion_GEN_OBJECT_NUM];  // store the
                                                                 // sorted
                                                                 // object id
                                                                 // based on
    // distace to ego trajectory
    static int OPSSelectedObjId
        [CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX];  // store selected
                                                           // object id by
                                                           // Object
                                                           // Pre-selection
                                                           // process
    static float32
        Dist2TrajAbsArray[EM_Fusion_GEN_OBJECT_NUM];  // store distance to ego
                                                      // predicted trajectory
                                                      // for each object

    // clear history data
    (void)memset(&OPSPrioClass, -1, sizeof(OPSPrioClass));
    (void)memset(&PrioClassTotalNum, 0u, sizeof(PrioClassTotalNum));
    (void)memset(&PrioClassSelectedNum, 0u, sizeof(PrioClassSelectedNum));
    (void)memset(&ObjIdSortedByDistance, -1, sizeof(ObjIdSortedByDistance));
    (void)memset(&OPSSelectedObjId, -1, sizeof(OPSSelectedObjId));
    (void)memset(&Dist2TrajAbsArray, -1, sizeof(Dist2TrajAbsArray));

    if (GET_Envm_EXT_OBJ_DATA_PTR->HeaderObjList.iNumOfUsedObjects >
        CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX) {
        // step 0: customer object in the first priority class
        // step 1: calculate distance to trajectory and priority class for each
        // object
        OPSPreprocess(Dist2TrajAbsArray, PrioClassTotalNum, OPSPrioClass);
        // step 2: sorting object list based on the distance to trajectory
        BubbleSortBasedOnDist(Dist2TrajAbsArray, ObjIdSortedByDistance);
        // step 3: calculate select object number for each priority class
        CalcPrioClassObjNum(PrioClassTotalNum, PrioClassSelectedNum);
        // step 4: get object id based on the result of step 2 and step 3
        SelectObjectBaseOnPrio(ObjIdSortedByDistance, OPSPrioClass,
                               PrioClassSelectedNum, OPSSelectedObjId);
    } else {
        // since the raw radar object number less than
        // CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX, we do not need
        // pre-selection process
        uint8 insertObjNum = 0;
        for (uint32 ui_Index = 0u; ui_Index < EM_Fusion_GEN_OBJECT_NUM;
             ui_Index++) {
            if (GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                        .General.eObjMaintenanceState != MT_STATE_DELETED &&
                GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                        .General.eObjMaintenanceState !=
                    MT_STATE_MERGE_DELETED &&
                GET_Envm_EXT_OBJ_DATA_PTR->Objects[ui_Index]
                        .Qualifiers.fProbabilityOfExistence > 0 &&
                insertObjNum < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX &&
                EnvmData.pExtObjList->Objects[ui_Index].Kinematic.fDistX >=
                    -1.f) {
                OPSSelectedObjId[insertObjNum] = ui_Index;
                insertObjNum++;
            }
        }
    }
    // step 5: memory copy object from ext_obj to priv_obj
    SenseTime_Memcpy(&(EnvmData.pExtObjList->sSigHeader),
                     &(EnvmData.pObjPreSeletList->sSigHeader),
                     sizeof(ENVMSignalHeader_t));
    EnvmData.pObjPreSeletList->eSigStatus = EnvmData.pExtObjList->eSigStatus;
    EnvmData.pObjPreSeletList->uiCycleCounter =
        EnvmData.pExtObjList->uiCycleCounter;
    EnvmData.pObjPreSeletList->uiTimeStamp = EnvmData.pExtObjList->uiTimeStamp;
    EnvmData.pObjPreSeletList->uiVersionNumber =
        EnvmData.pExtObjList->uiVersionNumber;
    for (int i = 0; i < Envm_NR_PRIVOBJECTS; i++) {
        // clear all the object data before fusion result output
        (void)memset(&EnvmData.pObjPreSeletList->Objects[i], 0u,
                     sizeof(Objects_t));
        EnvmData.pObjPreSeletList->Objects[i].ObjectId = -1;
    }

    uint8 insertObjNum = 0;
    for (uint32 ui_Index = 0u;
         ui_Index < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
         ui_Index++) {
        if (OPSSelectedObjId[ui_Index] >= 0) {
            SenseTime_Memcpy(
                &(EnvmData.pExtObjList->Objects[OPSSelectedObjId[ui_Index]]),
                &(EnvmData.pObjPreSeletList->Objects[insertObjNum]),
                sizeof(ExtObjects_t));
            insertObjNum++;
        }
    }
    EnvmData.pObjPreSeletList->HeaderObjList.iNumOfUsedObjects = insertObjNum;
}
// /*
// ****************************************************************************

//   Functionname:     TUEObjectFusionProcess */ /*!

//     @brief            object fusion main
//     function

//     @description      Set fusion function input
//     value: radar object list, camera object list,
//                     ego motion value, system
//     time stamp, fusion object list output, error
//     buffer.

//     @param            void

//     @return           void

//     @pre              None
//     @post             No changes

// ***************************************************************************/
// void TUEObjectFusionProcess(uint32 uiSystemTimeStamp_us) {
//     uint16 i, j, k;

//     // clear history data
//     (void)memset(&EgoMotion, 0u, sizeof(TueObjFusn_EgoMotionType));
//     (void)memset(&pFusnObjLists_Output, 0u,
//     sizeof(TueObjFusn_ObjectListType)); (void)memset(&pErrorBuffer, 0u,
//     sizeof(TueObjFusn_ErrorBufferType));

//     // set Fusion EgoMotion structure with VDY data
//     EgoMotion.bIsValid = GET_EGO_RAW_DATA_PTR->sSigHeader.eSigStatus;
//     EgoMotion.f32Acc =
//     GET_EGO_RAW_DATA_PTR->Longitudinal.AccelCorr.corrAccel; EgoMotion.f32Age
//     = MAX(
//         0, MIN(0.2, (float32)(uiSystemTimeStamp_us -
//                               GET_EGO_RAW_DATA_PTR->sSigHeader.uiTimeStamp) /
//                         1000000));  // us -> s
//     EgoMotion.f32Speed =
//     GET_EGO_RAW_DATA_PTR->Longitudinal.VeloCorr.corrVelo;
//     EgoMotion.f32YawRate = GET_EGO_RAW_DATA_PTR->Lateral.YawRate.YawRate;

//     // Fusion wrapper main function
//     sfObjectFusionMpf_Outputs_wrapper(
//         EnvmData.pPrivObjList, EnvmData.pCamObject, &EgoMotion,
//         uiSystemTimeStamp_us, &pFusnObjLists_Output, &pErrorBuffer);
// #ifdef TU_ALGO_SIMULATION_VS_CONF_200
//     Debug(
//         "radar object size: %d, camera object size: %d, fusion object size:
//         %d "
//         "\n",
//         EnvmData.pExtObjList->HeaderObjList.iNumOfUsedObjects, 0,
//         pFusnObjLists_Output.u16NumObjects);
// #endif
//     for (i = 0; i < Envm_NR_PRIVOBJECTS; i++) {
//         // clear all the object data before fusion result output
//         (void)memset(&EnvmData.pPrivObjList->Objects[i], 0u,
//         sizeof(Objects_t));
//     }
//     // set object list time stamp to current time since fusion finished time
//     // compensate process
//     EnvmData.pPrivObjList->sSigHeader.uiTimeStamp = uiSystemTimeStamp_us;
//     EnvmData.pPrivObjList->uiTimeStamp = uiSystemTimeStamp_us;
//     // for (i = 0; i < Envm_NR_PRIVOBJECTS; i++)

//     // copy signal header to EnvmData.pPrivObjList
//     for (i = 0; i < TUEOBJFUSN_IDPROVIDER_U16ID_MAX + 1; i++) {
//         for (j = 0; j < pFusnObjLists_Output.u16NumObjects; j++) {
//             if (i == pFusnObjLists_Output.aTrackable[j].u16ID) {
//                 break;
//             } else if (j == pFusnObjLists_Output.u16NumObjects - 1) {
//                 // clear all the object data before fusion result output
//                 if (TrackIdToObjectIdMap[i] >= 0 &&
//                     TrackIdToObjectIdMap[i] < Envm_NR_PRIVOBJECTS) {
//                     // reset OPS object quality value of invalid object
//                     FPSDeleteObject(TrackIdToObjectIdMap[i]);
//                 }
//                 TrackIdToObjectIdMap[i] = -1;
//             }
//         }
//     }

//     for (i = 0; i < pFusnObjLists_Output.u16NumObjects; i++) {
//         if (/*fusionResultInput.aTrackable[i].bUpdated != 0 && */
//             TrackIdToObjectIdMap[pFusnObjLists_Output.aTrackable[i].u16ID] ==
//             -1) {
//             for (j = 0; (j < Envm_NR_PRIVOBJECTS &&
//                          TrackIdToObjectIdMap[pFusnObjLists_Output.aTrackable[i]
//                                                   .u16ID] == -1);
//                  j++) {
//                 for (k = 0; k < TUEOBJFUSN_IDPROVIDER_U16ID_MAX + 1; k++) {
//                     if (j == TrackIdToObjectIdMap[k]) {
//                         break;
//                     } else if (k == TUEOBJFUSN_IDPROVIDER_U16ID_MAX) {
//                         TrackIdToObjectIdMap[pFusnObjLists_Output.aTrackable[i]
//                                                  .u16ID] = j;
//                         break;
//                     }
//                 }
//             }
//         }
//     }
//     // set number of object
//     EnvmData.pPrivObjList->HeaderObjList.iNumOfUsedObjects =
//         pFusnObjLists_Output.u16NumObjects;
//     // TrackIdToObjectIdMap[i]
//     for (i = 0; i < pFusnObjLists_Output.u16NumObjects; i++) {
//         // if fusion result object is only checked by camera, set PoE to 99
//         if
//         // camera's PoE is more than 90 reason: FPS PoE check threshold is
//         set
//         // according to the radar parameter. Camera result PoE more than 90
//         is
//         // enough for the FPS check
//         if (pFusnObjLists_Output.aTrackable[i].u8CyclesNoRadar != 0 &&
//             pFusnObjLists_Output.aTrackable[i].u8CyclesNoVision == 0 &&
//             pFusnObjLists_Output.aTrackable[i].f32ExistenceQuality >= 90) {
//             pFusnObjLists_Output.aTrackable[i].f32ExistenceQuality = 99;
//         }
//         // convert fusion result to EM structure which would be the input of
//         FPS if
//         (TrackIdToObjectIdMap[pFusnObjLists_Output.aTrackable[i].u16ID] >=
//                 0 &&
//             TrackIdToObjectIdMap[pFusnObjLists_Output.aTrackable[i].u16ID] <
//                 Envm_NR_PRIVOBJECTS) {
//             TueFusionObjListToContiRaObjListConverter(
//                 &(EnvmData.pPrivObjList->Objects
//                       [TrackIdToObjectIdMap[pFusnObjLists_Output.aTrackable[i]
//                                                 .u16ID]]),
//                 &(pFusnObjLists_Output.aTrackable[i]));
//         } else {
// #if TU_ALGO_SIMULATION_VS_CONF_200
//             Debug(
//                 "TrackIdToObjectIdMap wrong, TrackIdToObjectIdMap[%d]: %d
//                 \n", pFusnObjLists_Output.aTrackable[i].u16ID,
//                 TrackIdToObjectIdMap[pFusnObjLists_Output.aTrackable[i].u16ID]);
// #endif
//         }
//     }
// #ifdef TU_ALGO_SIMULATION_VS_CONF_200
// // printf fusioned objects properties: distX, distY
// // SimulationDataOut_ObjectFusionResult(EnvmData.pExtObjList,
// // EnvmData.pCamObject, EnvmData.pPrivObjList);
// #endif
// #ifdef LINUX_SYS

//     DATALOGInfo_t Record;

//     Record.StructID = Data_ExtObjList_type;
//     Record.Length = sizeof(ObjectList_t);
//     Record.SocBuf = *EnvmData.pPrivObjList;
//     BSW_DataLog_FreezeData(Record);

// #endif
// }

/* ****************************************************************************

  Functionname:     SenseTime_ReplaceFusionAddCalculation */ /*!

                     @brief            replace EM Fusion additianal calculation

                     @description      replace EM Fusion additianal calculation

                     @param            void

                     @return           void

                     @pre              None
                     @post             No changes


                   ****************************************************************************
                   */
void SenseTime_ReplaceFusionAddCalculation() {
    for (uint8 i = 0; i < Envm_NR_PRIVOBJECTS; i++) {
        EnvmData.pPrivObjList->Objects[i].Legacy.uiLifeTime =
            EnvmData.pPrivObjList->Objects[i].General.fLifeTime /
            (TASK_CYCLE_TIME_50 /* * 1000 */);  // 1000ms = 1s
        // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/34 changed by
        // guotao 20200601 start
        if (EnvmData.pPrivObjList->Objects[i].General.fLifeTime -
                EnvmData.pPrivObjList->Objects[i].Legacy.uiLifeTime *
                    TASK_CYCLE_TIME_50 * 1000 >
            TUE_CML_GaussianCDFMinSigma) {
            EnvmData.pPrivObjList->Objects[i].Legacy.uiLifeTime +=
                1;  // round up the life cycle number
        }

        // TODO(heqiushu): comment these lines for temp solution of single
        // radar, 20220921 if
        // (EnvmData.pPrivObjList->Objects[i].SensorSpecific.bCamConfirmed &&
        //     EnvmData.pPrivObjList->Objects[i].Attributes.eDynamicProperty ==
        //         TU_OBJECT_DYNPROP_STATIONARY &&
        //     (EnvmData.pPrivObjList->Objects[i].Attributes.eClassification ==
        //          Envm_GEN_OBJECT_CLASS_TRUCK ||
        //      EnvmData.pPrivObjList->Objects[i].Attributes.eClassification ==
        //          Envm_GEN_OBJECT_CLASS_CAR))
        // {
        //     // change stationary camera confirmed object to stopped for ACC
        //     OOI
        //     // lost caused by dynamic property mistake
        //     EnvmData.pPrivObjList->Objects[i].Attributes.eDynamicProperty =
        //         TU_OBJECT_DYNPROP_STOPPED;
        // }

        if (EnvmData.pPrivObjList->Objects[i].Attributes.eClassification ==
            Envm_GEN_OBJECT_CLASS_PEDESTRIAN) {
            EnvmData.pPrivObjList->Objects[i].EBAPresel.bCrossingPedEbaPresel =
                TRUE;
        }

        // wulin to do 20220316, add MOTORCYCLE and BICYCLE Presel decide
        if (EnvmData.pPrivObjList->Objects[i].Attributes.eClassification ==
                Envm_GEN_OBJECT_CLASS_MOTORCYCLE ||
            EnvmData.pPrivObjList->Objects[i].Attributes.eClassification ==
                Envm_GEN_OBJECT_CLASS_BICYCLE) {
            EnvmData.pPrivObjList->Objects[i]
                .EBAPresel.bCrossingBicycleEbaPresel = TRUE;
        }
    }
}

// /*
// ****************************************************************************

//   Functionname:     ConvertFusionIdToEnvmDataId */ /*!

//     @brief            fusioned track ID convert to
//     the EnvmData's object ID

//     @description

//     @param            void

//     @return           void

//     @pre              None
//     @post             No changes

//     **************************************************************************/
// void ConvertFusionIdToEnvmDataId(TueObjFusn_ObjectListType fusionResultInput,
//                                  int* fusionToEnvmDataMap) {
//     // clear the id which is not used in the current cylce (but used in the
//     last
//     // cycle)
//     uint16 i, j, k;
//     for (i = 0; i < TUEOBJFUSN_IDPROVIDER_U16ID_MAX + 1; i++) {
//         for (j = 0; j < fusionResultInput.u16NumObjects; j++) {
//             if (i == fusionResultInput.aTrackable[j].u16ID) {
//                 break;
//             } else if (j == fusionResultInput.u16NumObjects - 1) {
//                 fusionToEnvmDataMap[i] = -1;
//             }
//             /* else if (fusionResultInput.aTrackable[j].bUpdated == 0)
//             {
//                 fusionToEnvmDataMap[fusionResultInput.aTrackable[j].u16ID] =
//                 -1;
//             }*/
//         }
//     }

//     // set the mapping ID for the new incoming track
//     for (i = 0; i < fusionResultInput.u16NumObjects; i++) {
//         if (/*fusionResultInput.aTrackable[i].bUpdated != 0 && */
//             fusionToEnvmDataMap[fusionResultInput.aTrackable[i].u16ID] == -1)
//             {
//             // find next free space for the new incoming track
//             for (j = 0;
//                  (j < Envm_NR_PRIVOBJECTS &&
//                   fusionToEnvmDataMap[fusionResultInput.aTrackable[i].u16ID]
//                   ==
//                       -1);
//                  j++) {
//                 for (k = 0; k < TUEOBJFUSN_IDPROVIDER_U16ID_MAX + 1; k++) {
//                     if (j == fusionToEnvmDataMap[k]) {
//                         break;
//                     } else if (k == TUEOBJFUSN_IDPROVIDER_U16ID_MAX) {
//                         fusionToEnvmDataMap[fusionResultInput.aTrackable[i]
//                                                 .u16ID] = j;
//                         break;
//                     }
//                 }
//             }
//         }
//     }
// }

/* ****************************************************************************

  Functionname:    EgoCoordinateConvert */ /*!

                                       @brief            EgoCoordinateConvert

                                       @description     EgoCoordinateConvert


                                     ****************************************************************************
                                     */
void EgoCoordinateConvert(ExtObjectList_t *conv_ExtObjectList_ego,
                          float32 AddDeltX,
                          float32 AddDeltY) {
    for (int objnr = 0; objnr < EM_Fusion_GEN_OBJECT_NUM; objnr++) {
        if (conv_ExtObjectList_ego->Objects[objnr].ObjectId != -1) {
            conv_ExtObjectList_ego->Objects[objnr].Kinematic.fDistX += AddDeltX;
            conv_ExtObjectList_ego->Objects[objnr].Kinematic.fDistY += AddDeltY;
        }
    }
}

/* ****************************************************************************

  Functionname:    ObjCoordianteConvert */ /*!

  @brief            ObjCoordianteConvert

  @description     ObjCoordianteConvert

****************************************************************************/
void ObjCoordianteConvert(ExtObjectList_t *conv_ExtObjectList_obj,
                          uint8 convert_for_func) {
    float32 Orien;
    float32 lenth;
    float32 delta_x;
    float32 delta_y;
    switch (convert_for_func) {
        case CONVERT_FOR_ACC:
            for (int objnr = 0; objnr < EM_Fusion_GEN_OBJECT_NUM; objnr++) {
                if (conv_ExtObjectList_obj->Objects[objnr].ObjectId != -1) {
                    if (conv_ExtObjectList_obj->Objects[objnr]
                            .Geometry.fOrientationValid == 1) {
                        Orien = conv_ExtObjectList_obj->Objects[objnr]
                                    .Geometry.fOrientation;
                        lenth = conv_ExtObjectList_obj->Objects[objnr]
                                    .Geometry.fLength;

                        if ((0 <= Orien) &&
                            (Orien < TUE_PRV_FUSION_MATH_PI_HALF)) {
                            delta_x = -(lenth / 2 * tue_prv_em_cos(Orien));
                            delta_y = -(lenth / 2 * tue_prv_em_sin(Orien));
                        } else if ((-TUE_PRV_FUSION_MATH_PI_HALF <= Orien) &&
                                   (Orien < 0)) {
                            delta_x = -(lenth / 2 * tue_prv_em_cos(-Orien));
                            delta_y = (lenth / 2 * tue_prv_em_sin(-Orien));
                        } else if ((TUE_PRV_FUSION_MATH_PI_HALF <= Orien) &&
                                   (Orien <= TUE_PRV_FUSION_MATH_PI)) {
                            delta_x = -(lenth / 2 *
                                        tue_prv_em_cos(
                                            TUE_PRV_FUSION_MATH_PI - Orien));
                            delta_y = (lenth / 2 *
                                       tue_prv_em_sin(
                                           TUE_PRV_FUSION_MATH_PI - Orien));
                        } else {
                            delta_x = -(lenth / 2 *
                                        tue_prv_em_cos(
                                            TUE_PRV_FUSION_MATH_PI + Orien));
                            delta_y = -(lenth / 2 *
                                        tue_prv_em_sin(
                                            TUE_PRV_FUSION_MATH_PI + Orien));
                        }
                        conv_ExtObjectList_obj->Objects[objnr]
                            .Kinematic.fDistX += delta_x;
                        conv_ExtObjectList_obj->Objects[objnr]
                            .Kinematic.fDistY += delta_y;
                    }
                }
            }
            break;

        case CONVERT_FOR_SRR:
            /* code */
            break;

        default:
            break;
    }
}

/* ****************************************************************************

  Functionname:    CoordinateConvert */ /*!

                                          @brief            CoordinateConvert

                                          @description     CoordinateConvert


                                        ****************************************************************************
                                        */
void CoordinateConvert(ExtObjectList_t *conv_ExtObjectList,
                       uint8 convert_for_func,
                       VEDVehParMain_t vedVehParMain) {
    float32 AddDeltX;
    switch (convert_for_func) {
        case CONVERT_FOR_ACC:
            EgoCoordinateConvert(conv_ExtObjectList,
                                 -(vedVehParMain.WheelBase +
                                   vedVehParMain.DIST_FrontAxle_to_FontBumper),
                                 0);
            ObjCoordianteConvert(conv_ExtObjectList, CONVERT_FOR_ACC);
            break;

        case CONVERT_FOR_SRR:
            // AddDeltX = vedVehParMain.WheelBase * 0.5f;
            AddDeltX = vedVehParMain.WheelBase;
            EgoCoordinateConvert(conv_ExtObjectList, -AddDeltX, 0);
            ObjCoordianteConvert(conv_ExtObjectList, CONVERT_FOR_SRR);
            break;

        default:
            break;
    }
}
/*****************************************************************************
  Functionname:     ENVM_Exec                                              */ /*!

   @brief            Em main function with input and output parameters

   @description      Saves starting time of EM, checks static memory,
                     provider ports, requester ports and error trap data.
                     Processes EM centralized input. Calls EM main flow
                     control and processes EM centralized output.
                     Calcuates cycle time of EM.

   @param[in]        reqPorts : pointer on the required ports structure
   @param[in,out]    proPorts : pointer on the provided ports structure
   @param[in]        p_AS_t_ServiceFuncts : pointer on Frame SW service
 functions like current time info

   @return           void

   @pre              None
   @post             No changes
 *****************************************************************************/
void ENVM_Exec(const reqEMPrtList_t *reqPorts, proEnvmPrtList_t *proPorts) {
    boolean b_InputPointerOk, b_OutputPointerOk, b_MemoryOk;

    /* store start timestamp of EM */
    if (reqPorts->pCtrl != NULL) {
        EnvmData.pPrivGlob->u_StartEMTimestamp =
            reqPorts->pCtrl->uiTimeStamp_us;
    }

    /* Check of static memory */
    b_MemoryOk = EM_b_CheckMemory();

    /* Request Ports mapping */
    b_InputPointerOk = EM_b_SetupRequestPorts(reqPorts);

    /* CoordinateConvert */
    CoordinateConvert(&TEMP_pExtObjList, CONVERT_FOR_ACC,
                      reqPorts->pVehicleStatData->VehParMain);
    CoordinateConvert(&TEMP_pExtFuisonObjList, CONVERT_FOR_SRR,
                      reqPorts->pVehicleStatData->VehParMain);

    /* Provide Ports mapping */
    b_OutputPointerOk = EM_b_SetupProvidePorts(proPorts);

    /* if the pointers are not set correctly or a memory error is detected,
     * remain in mode startup */
    if ((b_MemoryOk == FALSE) ||
        ((b_InputPointerOk == FALSE) || (b_OutputPointerOk == FALSE))) {
        /* if not valid control data is given, remain in startup */
        EnvmData.pFrame->eEnvmOpModeRequest = Envm_MOD_STARTUP;
    }

    /*! At this place it is necessary to test EnvmOpMode directly to get an
     * initialized component */
    if (EnvmData.pFrame->eEnvmOpModeRequest == Envm_MOD_STARTUP) {
        /* EM internal (own) init processing to set proper default values */
        EMInit();
    } else {
        /* Increment the em cycle counter */
        EM_FRAME_DATA->uiCycleCounter++;
    }

    /* EM centralized input processing */
    EnvmProcessInput();

    /* KinematicConvert */
    EM_CalcGenObjectRelativeKinematicData(reqPorts->pExtObjList);

    /* FID detect */
    FIDMProcess(reqPorts->pFIDInport, proPorts->pFIDMOutPro);

    /* Output highlights the target ID with the warn level */
    EM_HMI_Output(reqPorts->pSOA_HMI, proPorts->pADASHighlightID);
    /* Call of EM main flow control */
    EMProcess(reqPorts, proPorts);

    /* EM centralized output processing */
    EnvmProcessOutput();
    Envm_v_CalcDeltaTime(Envm_TIME_CP_PROC_OUT);

    // printf("proPorts->pEnvmGenObjectList->aObject[0].Kinematic.fDistX =
    // %f\n",
    //        proPorts->pEnvmGenObjectList->aObject[0].Kinematic.fDistX);
    // printf(
    //     "proPorts->pEnvmGenObjectList->aObject[0].Qualifiers.uiAccObjQuality
    //     = "
    //     "%d\n",
    //     proPorts->pEnvmGenObjectList->aObject[0].Qualifiers.uiAccObjQuality);
    // printf(
    //     "proPorts->pEnvmGenObjectList->aObject[0].Qualifiers.uiEbaObjQuality
    //     = "
    //     "%d\n",
    //     proPorts->pEnvmGenObjectList->aObject[0].Qualifiers.uiEbaObjQuality);
}
/*****************************************************************************
  Functionname:     EM_TrafficLightSignConvert */
/*!

  @brief            Traffic lights and signs signal conversion

  @description      Traffic lights and signs signal conversion

  @param[in]        pTSRObject : pointer on the pTSRObject structure
  @param[out]       pTrafficLight : pointer on the pTrafficLight structure
  @param[out]       pTrafficSign  : pointer on the pTrafficSign structure

  @return           void

  @pre              None
  @post             No changes
*****************************************************************************/
static void EM_TrafficLightSignConvert(const TSRObjectList_t *pTSRObject,
                                       const uint32 uiSystemTimeStamp_us,
                                       Envm_TrafficLight_t *pTrafficLight,
                                       Envm_TrafficSign_t *pTrafficSign) {
    pTrafficLight->sSigHeader.eSigStatus = AL_SIG_STATE_OK;
    pTrafficLight->sSigHeader.uiCycleCounter++;
    pTrafficLight->sSigHeader.uiMeasurementCounter++;
    pTrafficLight->sSigHeader.uiTimeStamp = uiSystemTimeStamp_us;

    pTrafficSign->sSigHeader.eSigStatus = AL_SIG_STATE_OK;
    pTrafficSign->sSigHeader.uiCycleCounter++;
    pTrafficSign->sSigHeader.uiMeasurementCounter++;
    pTrafficSign->sSigHeader.uiTimeStamp = uiSystemTimeStamp_us;

    // Traffic lights
    pTrafficLight->uTrafficLightsize = pTSRObject->uTrafficLightsize;
    for (uint8 i = 0u; i < pTSRObject->uTrafficLightsize; i++) {
        pTrafficLight->sTrafficLights[i].id = pTSRObject->sTrafficLights[i].id;
        pTrafficLight->sTrafficLights[i].position_x =
            pTSRObject->sTrafficLights[i].position_x;
        pTrafficLight->sTrafficLights[i].position_y =
            pTSRObject->sTrafficLights[i].position_y;
        pTrafficLight->sTrafficLights[i].position_z =
            pTSRObject->sTrafficLights[i].position_z;
        pTrafficLight->sTrafficLights[i].quaternion_w =
            pTSRObject->sTrafficLights[i].quaternion_w;
        pTrafficLight->sTrafficLights[i].quaternion_x =
            pTSRObject->sTrafficLights[i].quaternion_x;
        pTrafficLight->sTrafficLights[i].quaternion_y =
            pTSRObject->sTrafficLights[i].quaternion_y;
        pTrafficLight->sTrafficLights[i].quaternion_z =
            pTSRObject->sTrafficLights[i].quaternion_z;
        pTrafficLight->sTrafficLights[i].status_label =
            pTSRObject->sTrafficLights[i].status_label;
        pTrafficLight->sTrafficLights[i].type_label =
            pTSRObject->sTrafficLights[i].type_label;
        pTrafficLight->sTrafficLights[i].width =
            pTSRObject->sTrafficLights[i].width;
        pTrafficLight->sTrafficLights[i].length =
            pTSRObject->sTrafficLights[i].length;
        pTrafficLight->sTrafficLights[i].height =
            pTSRObject->sTrafficLights[i].height;
        pTrafficLight->sTrafficLights[i].confidence =
            pTSRObject->sTrafficLights[i].confidence;
        pTrafficLight->sTrafficLights[i].detection_status =
            pTSRObject->sTrafficLights[i].detection_status;
            // RENDL
        for (int j = 0; j < 8; j++) {
            pTrafficLight->sTrafficLights[i].lane_ids_8[j] =
                pTSRObject->sTrafficLights[i].lane_ids_8[j];
        }
    }
    // traffic sign
    // upper limit sign
    pTrafficSign->CAM_SpeedUpperLimit_btf = 0U;
    pTrafficSign->uSpeedUpperLimitSize = 0U;
    pTrafficSign->uSpeedLowerLimitSize = 0U;
    pTrafficSign->uForbiddenSignSize = 0U;
    for (uint8 i = 0u; i < pTSRObject->uTrafficSignsize; i++) {
        if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_SPEED_5 &&
            (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x0001U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x0001U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x0001U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_10 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_10) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x0002U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x0002U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x0002U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_20 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_20) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x0008U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x0008U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x0008U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_30 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_30) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x0020U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x0020U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x0020U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_40 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_40) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x0080U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x0080U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x0080U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_50 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_50) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x0100U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x0100U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x0100U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_60 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_60) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x0200U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x0200U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x0200U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_70 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_70) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x0400U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x0400U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x0400U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_80 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_80) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x0800U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x0800U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x0800U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_90 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_90) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x1000U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x1000U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x1000U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_100 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_100) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x2000U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x2000U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x2000U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_110 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_110) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x4000U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x4000U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x4000U;
            pTrafficSign->uSpeedUpperLimitSize += 1;

        } else if ((pTSRObject->sTrafficSigns[i].label ==
                        TRAFFICSIGN_SPEED_120 ||
                    pTSRObject->sTrafficSigns[i].label ==
                        VARIABLE_TRFCSGN_SPEED_120) &&
                   (pTrafficSign->CAM_SpeedUpperLimit_btf & 0x8000U) == 0) {
            pTrafficSign->CAM_SpeedUpperLimit_btf += 0x8000U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedUpperLimit[pTrafficSign->uSpeedUpperLimitSize]
                .label = 0x8000U;
            pTrafficSign->uSpeedUpperLimitSize += 1;
        } else {
        }
    }

    pTrafficSign->CAM_SpeedLowerLimit_btf = 0;
    for (uint8 i = 0u; i < pTSRObject->uTrafficSignsize; i++) {
        if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_SPEEDLIMIT_50) {
            pTrafficSign->CAM_SpeedLowerLimit_btf += 0x0100U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]
                .label = 0x0100U;
            pTrafficSign->uSpeedLowerLimitSize += 1;

        } else if (pTSRObject->sTrafficSigns[i].label ==
                   TRAFFICSIGN_SPEEDLIMIT_60) {
            pTrafficSign->CAM_SpeedLowerLimit_btf += 0x0200U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]
                .label = 0x0200U;
            pTrafficSign->uSpeedLowerLimitSize += 1;

        } else if (pTSRObject->sTrafficSigns[i].label ==
                   TRAFFICSIGN_SPEEDLIMIT_70) {
            pTrafficSign->CAM_SpeedLowerLimit_btf += 0x0400U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]
                .label = 0x0400U;
            pTrafficSign->uSpeedLowerLimitSize += 1;

        } else if (pTSRObject->sTrafficSigns[i].label ==
                   TRAFFICSIGN_SPEEDLIMIT_80) {
            pTrafficSign->CAM_SpeedLowerLimit_btf += 0x0800U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]
                .label = 0x0800U;
            pTrafficSign->uSpeedLowerLimitSize += 1;

        } else if (pTSRObject->sTrafficSigns[i].label ==
                   TRAFFICSIGN_SPEEDLIMIT_90) {
            pTrafficSign->CAM_SpeedLowerLimit_btf += 0x1000U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]
                .label = 0x1000U;
            pTrafficSign->uSpeedLowerLimitSize += 1;

        } else if (pTSRObject->sTrafficSigns[i].label ==
                   TRAFFICSIGN_SPEEDLIMIT_100) {
            pTrafficSign->CAM_SpeedLowerLimit_btf += 0x2000U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]
                .label = 0x2000U;
            pTrafficSign->uSpeedLowerLimitSize += 1;

        } else if (pTSRObject->sTrafficSigns[i].label ==
                   TRAFFICSIGN_SPEEDLIMIT_110) {
            pTrafficSign->CAM_SpeedLowerLimit_btf += 0x4000U;
            memcpy(
                &(pTrafficSign
                      ->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]),
                &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sSpeedLowerLimit[pTrafficSign->uSpeedLowerLimitSize]
                .label = 0x4000U;
            pTrafficSign->uSpeedLowerLimitSize += 1;
        } else {
        }
    }

    pTrafficSign->CAM_SpeedRelUpperLimit_btf = 0;
    for (uint8 i = 0u; i < pTSRObject->uTrafficSignsize; i++) {
        if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_5 &&
            (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x0001U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x0001U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_10 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x0002U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x0002U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_20 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x0008U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x0008U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_30 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x0020U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x0020U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_40 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x0080U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x0080U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_50 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x0100U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x0100U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_60 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x0200U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x0200U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_70 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x0400U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x0400U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_80 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x0800U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x0800U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_90 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x1000U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x1000U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_100 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x2000U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x2000U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_110 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x4000U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x4000U;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_LIFT_120 &&
                   (pTrafficSign->CAM_SpeedRelUpperLimit_btf & 0x8000U) == 0) {
            pTrafficSign->CAM_SpeedRelUpperLimit_btf += 0x8000U;
        } else {
        }
    }

    pTrafficSign->CAM_SpeedRelLowerLimit_btf = 0;
    pTrafficSign->CAM_ForbiddenSign_btf[0] = 0;
    pTrafficSign->CAM_ForbiddenSign_btf[1] = 0;
    for (uint8 i = 0u; i < pTSRObject->uTrafficSignsize; i++) {
        if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_X_LEFT &&
            ((pTrafficSign->CAM_ForbiddenSign_btf[0] & 0x0002) == 0)) {
            pTrafficSign->CAM_ForbiddenSign_btf[0] += 0x0002;
            memcpy(&(pTrafficSign
                         ->sForbiddenSign[pTrafficSign->uForbiddenSignSize]),
                   &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sForbiddenSign[pTrafficSign->uForbiddenSignSize]
                .label = 0x0002;
            pTrafficSign->uForbiddenSignSize += 1;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_X_RIGHT &&
                   ((pTrafficSign->CAM_ForbiddenSign_btf[0] & 0x0004) == 0)) {
            pTrafficSign->CAM_ForbiddenSign_btf[0] += 0x0004;
            memcpy(&(pTrafficSign
                         ->sForbiddenSign[pTrafficSign->uForbiddenSignSize]),
                   &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sForbiddenSign[pTrafficSign->uForbiddenSignSize]
                .label = 0x0004;
            pTrafficSign->uForbiddenSignSize += 1;
        } else if (pTSRObject->sTrafficSigns[i].label ==
                       TRAFFICSIGN_X_STRAIGHT &&
                   ((pTrafficSign->CAM_ForbiddenSign_btf[0] & 0x0008) == 0)) {
            pTrafficSign->CAM_ForbiddenSign_btf[0] += 0x0008;
            memcpy(&(pTrafficSign
                         ->sForbiddenSign[pTrafficSign->uForbiddenSignSize]),
                   &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sForbiddenSign[pTrafficSign->uForbiddenSignSize]
                .label = 0x0008;
            pTrafficSign->uForbiddenSignSize += 1;
        } else if (pTSRObject->sTrafficSigns[i].label ==
                       TRAFFICSIGN_X_TURNING &&
                   ((pTrafficSign->CAM_ForbiddenSign_btf[0] & 0x0010) == 0)) {
            pTrafficSign->CAM_ForbiddenSign_btf[0] += 0x0010;
            memcpy(&(pTrafficSign
                         ->sForbiddenSign[pTrafficSign->uForbiddenSignSize]),
                   &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sForbiddenSign[pTrafficSign->uForbiddenSignSize]
                .label = 0x0010;
            pTrafficSign->uForbiddenSignSize += 1;
        } else if (pTSRObject->sTrafficSigns[i].label ==
                       TRAFFICSIGN_X_PARKING &&
                   ((pTrafficSign->CAM_ForbiddenSign_btf[0] & 0x0020) == 0)) {
            pTrafficSign->CAM_ForbiddenSign_btf[0] += 0x0020;
            memcpy(&(pTrafficSign
                         ->sForbiddenSign[pTrafficSign->uForbiddenSignSize]),
                   &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sForbiddenSign[pTrafficSign->uForbiddenSignSize]
                .label = 0x0020;
            pTrafficSign->uForbiddenSignSize += 1;
        } else if (pTSRObject->sTrafficSigns[i].label == TRAFFICSIGN_X_ENTER &&
                   ((pTrafficSign->CAM_ForbiddenSign_btf[0] & 0x0040) == 0)) {
            pTrafficSign->CAM_ForbiddenSign_btf[0] += 0x0040;
            memcpy(&(pTrafficSign
                         ->sForbiddenSign[pTrafficSign->uForbiddenSignSize]),
                   &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sForbiddenSign[pTrafficSign->uForbiddenSignSize]
                .label = 0x0040;
            pTrafficSign->uForbiddenSignSize += 1;
        } else if (pTSRObject->sTrafficSigns[i].label ==
                       TRAFFICSIGN_X_TURNNINGBACK &&
                   ((pTrafficSign->CAM_ForbiddenSign_btf[0] & 0x0080) == 0)) {
            pTrafficSign->CAM_ForbiddenSign_btf[0] += 0x0080;
            memcpy(&(pTrafficSign
                         ->sForbiddenSign[pTrafficSign->uForbiddenSignSize]),
                   &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sForbiddenSign[pTrafficSign->uForbiddenSignSize]
                .label = 0x0080;
            pTrafficSign->uForbiddenSignSize += 1;
        } else if (pTSRObject->sTrafficSigns[i].label ==
                       TRAFFICSIGN_X_PASSING &&
                   ((pTrafficSign->CAM_ForbiddenSign_btf[0] & 0x0100) == 0)) {
            pTrafficSign->CAM_ForbiddenSign_btf[0] += 0x0100;
            memcpy(&(pTrafficSign
                         ->sForbiddenSign[pTrafficSign->uForbiddenSignSize]),
                   &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sForbiddenSign[pTrafficSign->uForbiddenSignSize]
                .label = 0x0100;
            pTrafficSign->uForbiddenSignSize += 1;
        } else if (pTSRObject->sTrafficSigns[i].label ==
                       TRAFFICSIGN_X_AUDIBLEWARNING &&
                   ((pTrafficSign->CAM_ForbiddenSign_btf[0] & 0x0200) == 0)) {
            pTrafficSign->CAM_ForbiddenSign_btf[0] += 0x0200;
            memcpy(&(pTrafficSign
                         ->sForbiddenSign[pTrafficSign->uForbiddenSignSize]),
                   &(pTSRObject->sTrafficSigns[i]), sizeof(CamTrafficSign_t));
            pTrafficSign->sForbiddenSign[pTrafficSign->uForbiddenSignSize]
                .label = 0x0200;
            pTrafficSign->uForbiddenSignSize += 1;
        } else {
        }
    }
}
/*****************************************************************************
  Functionname:     EM_CalcGenObjectRelativeKinematicData */
/*!

  @brief            Calculate EM public object relative kinematic data to EM
general object

  @description      Calculate EM public object relative kinematic data to EM
general object

  @param[in]

  @return           void

  @pre              None
  @post             No changes
*****************************************************************************/
static void EM_CalcGenObjectRelativeKinematicData(
    const ExtObjectList_t *pExtObjList) {
    float32 fTemp;
    for (uint8 i = 0; i < EM_Fusion_GEN_OBJECT_NUM; i++) {
        // Calculate fVrelX
        TEMP_pExtObjList.Objects[i].Kinematic.fVrelX =
            pExtObjList->Objects[i].Kinematic.fVabsX -
            EMTRAFO_f_GetObjSyncEgoMotionVx(
                pExtObjList->Objects[i].Kinematic.fDistY);
        TEMP_pExtFuisonObjList.Objects[i].Kinematic.fVrelX =
            pExtObjList->Objects[i].Kinematic.fVabsX -
            EMTRAFO_f_GetObjSyncEgoMotionVx(
                pExtObjList->Objects[i].Kinematic.fDistY);
        // Calculate fVrelXStd
        fTemp = SQR(pExtObjList->Objects[i].Kinematic.fVabsXStd);
        fTemp -= EM_f_GetEgoObjSyncVelXVar();
        TEMP_pExtObjList.Objects[i].Kinematic.fVrelXStd = SQRT(fTemp);
        TEMP_pExtFuisonObjList.Objects[i].Kinematic.fVrelXStd = SQRT(fTemp);
        // Calculate fVrelY
        TEMP_pExtObjList.Objects[i].Kinematic.fVrelY =
            pExtObjList->Objects[i].Kinematic.fVabsY -
            EMTRAFO_f_GetObjSyncEgoMotionVy(
                pExtObjList->Objects[i].Kinematic.fDistX);
        TEMP_pExtFuisonObjList.Objects[i].Kinematic.fVrelY =
            pExtObjList->Objects[i].Kinematic.fVabsY -
            EMTRAFO_f_GetObjSyncEgoMotionVy(
                pExtObjList->Objects[i].Kinematic.fDistX);
        // Calculate fVrelYStd
        TEMP_pExtObjList.Objects[i].Kinematic.fVrelYStd =
            pExtObjList->Objects[i].Kinematic.fVabsXStd;
        TEMP_pExtFuisonObjList.Objects[i].Kinematic.fVrelYStd =
            pExtObjList->Objects[i].Kinematic.fVabsXStd;
        // Calculate fArelX
        TEMP_pExtObjList.Objects[i].Kinematic.fArelX =
            pExtObjList->Objects[i].Kinematic.fAabsX -
            EM_f_GetEgoObjSyncAccelX();
        TEMP_pExtFuisonObjList.Objects[i].Kinematic.fArelX =
            pExtObjList->Objects[i].Kinematic.fAabsX -
            EM_f_GetEgoObjSyncAccelX();
        // Calculate fAabsXStd
        fTemp = SQR(pExtObjList->Objects[i].Kinematic.fAabsXStd);
        fTemp -= EM_f_GetEgoObjSyncAccelXVar();
        TEMP_pExtObjList.Objects[i].Kinematic.fArelXStd = SQRT(fTemp);
        TEMP_pExtFuisonObjList.Objects[i].Kinematic.fArelXStd = SQRT(fTemp);
        // Calculate fArelY
        TEMP_pExtObjList.Objects[i].Kinematic.fArelY =
            pExtObjList->Objects[i].Kinematic.fAabsY;
        TEMP_pExtFuisonObjList.Objects[i].Kinematic.fArelY =
            pExtObjList->Objects[i].Kinematic.fAabsY;
        // Calculate fArelXStd
        TEMP_pExtObjList.Objects[i].Kinematic.fArelXStd =
            pExtObjList->Objects[i].Kinematic.fAabsYStd;
        TEMP_pExtFuisonObjList.Objects[i].Kinematic.fArelXStd =
            pExtObjList->Objects[i].Kinematic.fAabsYStd;
    }
}

/*
        author: LiuYang
        Date  ��o2019-05-03
        Des   : Freeze FPS DATA to MTS
*/
void FPSProcessMeasFreeze(void) {
    GET_Envm_INT_OBJ_DATA_PTR->sSigHeader = EnvmData.pPrivObjList->sSigHeader;
}

/* ****************************************************************************

  Functionname:     EM_b_SetupRequestPorts                                */ /*!

     @brief            Connect all EM request port pointers (input data)

     @description      Checks for valid request ports, valid control data, valid
                       production parameters and ECU service functions.
                       Collects operation mode, cycle violation for em and rsp,
   algo
                       parameters, azimuth correction, measurements for near,
   far range
                       scan, PPAR, system performance monitor, FCT object list,
   vehicle
                       dynamic and static data, customer project dependent data,
   external
                       camera object interface data, external NRR object
   interface, EM call
                       back handler and ECU service functions from the requester
   ports.


     @param[in]        reqPorts : pointer on the required ports structure
     @param[in]        p_AS_t_ServiceFuncts : pointer to service function

     @return           true is pointer is valid

     @pre              None
     @post             No changes


   ****************************************************************************
   */
static boolean EM_b_SetupRequestPorts(const reqEMPrtList_t *reqPorts) {
    boolean b_Ret = FALSE;

    /* does the external pointer exists ? */
    if (reqPorts != NULL) {
        /* set data to valid */
        b_Ret = TRUE;

        /* check for valid control data */
        if (reqPorts->pCtrl != NULL) {
            /* BSW: Specific (cycle time, counter) operation mode, cycle
             * violation for em and rsp etc */
            EnvmData.pFrame->eEnvmOpModeRequest = reqPorts->pCtrl->EnvmOpMode;
            EnvmData.pFrame->bExtRSPCycleViolation =
                reqPorts->pCtrl->RSPCycleViolation;
            EnvmData.pFrame->bExtEnvmCycleViolation =
                reqPorts->pCtrl->EnvmCycleViolation;
        } else {
            /* if not valid control data is given, remain in startup */
            EnvmData.pFrame->eEnvmOpModeRequest = Envm_MOD_STARTUP;
        }

        /* Algo parameters */
        // EM_GET_BSW_ALGO_PARAMS_PTR = reqPorts->pAlgoParameters;
        EnvmData.pAlgoParameters = reqPorts->pAlgoParameters;

        // Envm_GET_NVM_IN_PTR = reqPorts->pNvmIn;
        EnvmData.pNvmIn = reqPorts->pNvmIn;

        /* FCT object list (accessed object list) */
        GET_FCT_PUB_OBJ_DATA_PTR = reqPorts->pFctObjectList;

        /* VDY : Vehicle dynamic data, velocity, yaw rate */
        GET_EGO_RAW_DATA_PTR = reqPorts->pVehicleDynData;
        /* Vehicle stat data, track width, wight */
        GET_EGO_STATIC_DATA_PTR = reqPorts->pVehicleStatData;

        TEMP_pExtObjList = *(reqPorts->pExtObjList);
        EnvmData.pExtObjList = &(TEMP_pExtObjList);

        // Set DATA to EnvmData.pExtSRRObjList
        TEMP_pExtFuisonObjList = *(reqPorts->pExtObjList);
        EnvmData.pExtFusionObjList = &(TEMP_pExtFuisonObjList);

        // EnvmData.pCamObject = reqPorts->pCamObject;

        EnvmData.pTSRObject = reqPorts->pTSRObject;
        /* external camera object interface data */
        // GET_CAM_SENSOR_INPUT_PTR  =  reqPorts->p_CamObjInput;//changed by
        // guotao 20190909 for NULL pointer issue
    }

    /* return status */
    return b_Ret;
}

/* ****************************************************************************

  Functionname:     EM_b_SetupProvidePorts                                */
static boolean EM_b_SetupProvidePorts(const proEnvmPrtList_t *proPorts) {
    boolean b_Ret = FALSE;

    static EnvmObjListDataPointer_t sEMObjListDataPointer = {
        NULL /* em generic object list output */
        ,
        NULL /* em technology specific object list output */
    };

    /* does the external pointer exists ? */
    if (proPorts != NULL) {
        /* set data to valid */
        b_Ret = TRUE;

        /* EM Frame: EM FCT cycle mode data use GDB_CYCLE_MODE to access */
        EnvmData.pECAMtCyclEnvmode = proPorts->pECAMtCyclEnvmode;
        proPorts->pECAMtCyclEnvmode->uiVersionNumber = Envm_FCT_CYCLE_INTFVER;

        /* EM Frame : Object list output */
        EnvmData.p_ObjListPointers = &sEMObjListDataPointer;
        EnvmData.p_ObjListPointers->pEnvmGenObjectList =
            proPorts->pEnvmGenObjectList;
        proPorts->pEnvmGenObjectList->uiVersionNumber =
            (Envm_GEN_OBJECT_LIST_INTFVER | Envm_GEN_OBJECT_INTFVER);
        EnvmData.p_ObjListPointers->pEnvmTechObjectList =
            proPorts->pEnvmTechObjectList;
        proPorts->pEnvmTechObjectList->uiVersionNumber =
            Envm_CR_OBJECT_LIST_INTFVER;

        /* OD: connect to internal object list memory */
        EnvmData.pPrivObjList = &(EnvmInternalObj);
        // EnvmData.pExtObjList = &(EnvmExternalObj);
        // EnvmData.pExtSRRObjList = &(ExtSRRObjects);
        EnvmData.pObjPreSeletList = &(EMPreSeletObj);

        EnvmData.pSRRIDManageObjectList = &(SRRIDManageObjectList);
        EnvmData.pFusionIDManageObjectList = &(FusionIDManageObjectList);

        // memory for mini-eye camera object list liuyang 20190513
        // EnvmData.pCamObject = &(CamObj_Sense);

        /* Object and target sync ego dynamic data */
        EnvmData.pEgoDynObjSync = proPorts->pObjSyncEgoDynamic;
        proPorts->pObjSyncEgoDynamic->uiVersionNumber = VED_VEH_DYN_INTFVER;
        EnvmData.pEgoDynTgtSync = proPorts->pTgtSyncEgoDynamic;
        proPorts->pTgtSyncEgoDynamic->uiVersionNumber = VED_VEH_DYN_INTFVER;

        /* Specific error (DEM) events */
        EM_GET_DEM_EVENTS_PTR = proPorts->pDem;

        /* Specific nonvolatile outputs */
        EM_GET_NVM_OUT_PTR = proPorts->pNvmOut;
    }

    /* return status */
    return b_Ret;
}

/* ****************************************************************************

  Functionname:     EM_b_CheckMemory                                     */ /*!

      @brief            Check if EM static memory init is working

      @description      Validate if the initialization of static memory is
    working

      @return           true is memory is valid

      @pre              None
      @post             No changes


    ****************************************************************************
    */
static boolean EM_b_CheckMemory(void) {
    boolean b_Ret = TRUE;
    static uint32 EM_u_StaticInit_Check = EM_MAIN_STATIC_INIT_CHECK;

    /* validate if the static initialization was correctly processed
        If not, EM has missing default data and is running in an undefined state
     */
    if (EM_u_StaticInit_Check != EM_MAIN_STATIC_INIT_CHECK) {
        EMErrorTrap(__FILE__, __LINE__, Envm_ERRORTRAP_TYPE_EXCEPTION);
        b_Ret = FALSE;
    }

    /* return status */
    return b_Ret;
}
/* ****************************************************************************

  Functionname:     EM_FusionObjectOutput                                     */
/*!

      @brief            transmitting fusion object list to output

      @description      ID management and signal copy

      @return

      @pre              None
      @post             No changes

    ****************************************************************************
    */
static void EM_FusionObjectOutput(
    const ExtObjectList_t *pExtFusionObjList,
    FusionIDManegeObjList_t *pFusionIDManageObjectList,
    const VEDVehPar_t *pGlobEgoStatic,
    const float32 fEgoVelX,
    const float32 fEgoAccelX,
    const float32 fEgoYawRate,
    const float32 fCosEgoHeadingAngle,
    const uint32 uiSystemTimeStamp_us,
    Envm_FusionObjectList_t *pFusionObjectOutputList) {
    uint8 uiNumberEMFusionObject = 0u;
    // ID management
    FusionObjectIDManagement(pExtFusionObjList, pFusionIDManageObjectList);
    // step 1, clear history object data
    if (pFusionObjectOutputList != NULL) {
        memset(pFusionObjectOutputList, 0, sizeof(Envm_FusionObjectList_t));
        for (uint8 i = 0u; i < EM_Fusion_GEN_OBJECT_NUM; i++) {
            pFusionObjectOutputList->aObjects[i].sGeneral.uiID_nu = 255u;
            pFusionObjectOutputList->aObjects[i].sGeneral.iRawFusionID_nu = -1;
        }
    }
    // step 2, exception handling
    // if (pExtFusionObjList == NULL || pFusionObjectOutputList == NULL) {
    if (pFusionObjectOutputList == NULL) {
        memset(&uaHistoryMeasuredFrequency, 0,
               sizeof(uint8) * TUE_SINGAL_FUSION_OBJECT_ID_MAX);
        memset(&uaObjectLifeCycle, 0,
               sizeof(uint16) * TUE_SINGAL_FUSION_OBJECT_ID_MAX);
        return;
    }
    // step 3, fusion object evaluate and output to different structure based on
    // sensor location
    if (pFusionIDManageObjectList->sSigHeader.eSigStatus == AL_SIG_STATE_OK) {
        boolean auUpdatedObject[TUE_SINGAL_FUSION_OBJECT_ID_MAX] = {0};
        for (uint8 i = 0;
             //  i < pSRRIDManageObjList->HeaderObjList.iNumOfUsedObjects &&
             i < EM_Fusion_GEN_OBJECT_NUM; i++) {
            // step 1, object validation check
            boolean bObjValid = CheckFusionObjectValidation(
                pFusionIDManageObjectList->Objects[i]);

            if (bObjValid) {
                FusionObjectKinematicProcess(
                    pFusionIDManageObjectList->Objects[i], fEgoVelX, fEgoAccelX,
                    fEgoYawRate, fCosEgoHeadingAngle,
                    &(pFusionObjectOutputList->aObjects[i].sKinematic));
                FusionObjectGeometryProcess(
                    pFusionIDManageObjectList->Objects[i],
                    &(pFusionObjectOutputList->aObjects[i].sGeometry));
                FusionObjectGeneralProcess(
                    pFusionIDManageObjectList->Objects[i],
                    &(pFusionObjectOutputList->aObjects[i].sGeneral));
                FusionObjectAttributeProcess(
                    pFusionIDManageObjectList->Objects[i],
                    &(pFusionObjectOutputList->aObjects[i].sAttribute));
                FusionObjectQualifierProcess(
                    pFusionIDManageObjectList->Objects[i],
                    &(pFusionObjectOutputList->aObjects[i].sQualifier));
                auUpdatedObject[pFusionIDManageObjectList->Objects[i]
                                    .ObjectId] = TRUE;
                // pEMSRROutObjList->aRearLeftObjects.aObjects[uiNumberLeftObject].
                // count object number of every sensor
                uiNumberEMFusionObject++;
            }
            pFusionObjectOutputList->uNumOfUsedObjects = uiNumberEMFusionObject;
        }

        // step 7, clear undetected object's measured frequency and life cycle
        // globle array
        for (uint8 i = 0; i < TUE_SINGAL_FUSION_OBJECT_ID_MAX; i++) {
            if (!auUpdatedObject[i]) {
                uaHistoryMeasuredFrequency[i] = 0;
                uaObjectLifeCycle[i] = 0;
            }
        }
    }

    pFusionObjectOutputList->sSigHeader.eSigStatus = AL_SIG_STATE_OK;
    pFusionObjectOutputList->sSigHeader.uiCycleCounter++;
    pFusionObjectOutputList->sSigHeader.uiMeasurementCounter++;
    pFusionObjectOutputList->sSigHeader.uiTimeStamp =
        uiSystemTimeStamp_us / (1000.f);

    pFusionObjectOutputList->uiVersionNumber =
        pFusionIDManageObjectList->uiVersionNumber;
}
/* ****************************************************************************

  Functionname:     FusionObjectIDManagement */
/*!

      @brief            ID management of fusion objects

      @description      ID management of fusion objects

      @return

      @pre              None
      @post             No changes

    ****************************************************************************
    */
static void FusionObjectIDManagement(
    const ExtObjectList_t *pExtFusionObjList,
    FusionIDManegeObjList_t *pFusionIDManageObjectList) {
    boolean iFusionCurrentCycleObjIDFlag[EM_Fusion_GEN_OBJECT_NUM * 2] = {0};
    pFusionIDManageObjectList->eSigStatus = 1u;
    // if the num of input objects is 0, memset fusion outputs are 0
    if (pExtFusionObjList->HeaderObjList.iNumOfUsedObjects == 0) {
        memset(pFusionIDManageObjectList, 0u, sizeof(FusionIDManegeObjList_t));
    } else {
        // Step1 : find if object ID exits in the last cycle
        for (uint8 i = 0u; i < EM_Fusion_GEN_OBJECT_NUM; i++) {
            boolean bFindTarget = FALSE;
            for (uint8 j = 0; j < EM_Fusion_GEN_OBJECT_NUM * 2; j++) {
                if (pExtFusionObjList->Objects[i].ObjectId ==
                    iFusionObjIDManageArray[j]) {
                    bFindTarget = TRUE;
                    iFusionCurrentCycleObjIDFlag[j] = 1;
                    break;
                }
            }
            // find vacant index for increasing ID
            if (!bFindTarget) {
                for (uint8 k = 0u; k < EM_Fusion_GEN_OBJECT_NUM * 2; k++) {
                    if (iFusionObjIDManageArray[k] == -1) {
                        iFusionObjIDManageArray[k] =
                            pExtFusionObjList->Objects[i].ObjectId;
                        iFusionCurrentCycleObjIDFlag[k] = 1;
                        break;
                    }
                }
            }
        }
        // step2 : delet ID of non-existent objects in current cycle
        for (uint8 i = 0u; i < EM_Fusion_GEN_OBJECT_NUM * 2; i++) {
            if (iFusionCurrentCycleObjIDFlag[i] != 1) {
                iFusionObjIDManageArray[i] = -1;
            }
        }
        // step3 : delet index of non-existent objects in current cycle
        for (uint8 i = 0u; i < EM_Fusion_GEN_OBJECT_NUM; i++) {
            if (iFusionCurrentCycleObjIDFlag[iFusionObjIndexManageArray[i]] ==
                    0 &&
                iFusionObjIndexManageArray[i] != -1) {
                iFusionObjIndexManageArray[i] = -1;
            }
        }
        // step4 : find if object index exits in the last cycle
        for (uint8 i = 0u; i < EM_Fusion_GEN_OBJECT_NUM * 2; i++) {
            boolean bfindIndex = FALSE;
            for (uint8 j = 0u; j < EM_Fusion_GEN_OBJECT_NUM; j++) {
                if (i == iFusionObjIndexManageArray
                             [j] &&  // iObjIDManageArray[i]
                    iFusionObjIDManageArray[i] != -1) {
                    bfindIndex = TRUE;
                    // iFusionObjIndexManageArray[j] = i;
                    break;
                }
            }
            if (!bfindIndex) {
                for (uint8 k = 0u; k < EM_Fusion_GEN_OBJECT_NUM; k++) {
                    if (iFusionObjIDManageArray[i] != -1 &&
                        iFusionObjIndexManageArray[k] == -1) {
                        iFusionObjIndexManageArray[k] = i;
                        break;
                    }
                }
            }
        }

        // clear all the object data before result output
        for (uint8 i = 0u; i < EM_Fusion_GEN_OBJECT_NUM; i++) {
            memset(&pFusionIDManageObjectList->Objects[i], 0u,
                   sizeof(Objects_t));
            pFusionIDManageObjectList->Objects[i].ObjectId = 255u;
            pFusionIDManageObjectList->Objects[i].RawObjectID = -1;
        }

        // output to pFusionIDManageObjectList from pExtFusionObjList
        for (uint8 i = 0u; i < EM_Fusion_GEN_OBJECT_NUM; i++) {
            for (uint8 j = 0u; j < EM_Fusion_GEN_OBJECT_NUM; j++) {
                if (iFusionObjIndexManageArray[j] != -1 &&
                    iFusionObjIDManageArray[iFusionObjIndexManageArray[j]] ==
                        pExtFusionObjList->Objects[i].ObjectId) {
                    pFusionIDManageObjectList->Objects[j].ObjectId =
                        iFusionObjIndexManageArray[j];
                    pFusionIDManageObjectList->Objects[j].RawObjectID =
                        pExtFusionObjList->Objects[i].ObjectId;
                    SenseTime_Memcpy(&(pExtFusionObjList->sSigHeader),
                                     &(pFusionIDManageObjectList->sSigHeader),
                                     sizeof(ENVMSignalHeader_t));
                    pFusionIDManageObjectList->eSigStatus =
                        pExtFusionObjList->eSigStatus;
                    pFusionIDManageObjectList->uiCycleCounter =
                        pExtFusionObjList->uiCycleCounter;
                    pFusionIDManageObjectList->uiTimeStamp =
                        pExtFusionObjList->uiTimeStamp;
                    pFusionIDManageObjectList->uiVersionNumber =
                        pExtFusionObjList->uiVersionNumber;

                    SenseTime_Memcpy(&pExtFusionObjList->HeaderObjList,
                                     &pFusionIDManageObjectList->HeaderObjList,
                                     sizeof(HeaderObjList_t));
                    SenseTime_Memcpy(
                        &pExtFusionObjList->Objects[i].Attributes,
                        &pFusionIDManageObjectList->Objects[j].Attributes,
                        sizeof(Attributes_t));
                    SenseTime_Memcpy(
                        &pExtFusionObjList->Objects[i].EBAPresel,
                        &pFusionIDManageObjectList->Objects[j].EBAPresel,
                        sizeof(EBAPresel_t));
                    SenseTime_Memcpy(
                        &pExtFusionObjList->Objects[i].General,
                        &pFusionIDManageObjectList->Objects[j].General,
                        sizeof(General_t));
                    SenseTime_Memcpy(
                        &pExtFusionObjList->Objects[i].Geometry,
                        &pFusionIDManageObjectList->Objects[j].Geometry,
                        sizeof(Geometry_t));
                    SenseTime_Memcpy(
                        &pExtFusionObjList->Objects[i].Kinematic,
                        &pFusionIDManageObjectList->Objects[j].Kinematic,
                        sizeof(Kinematic_t));
                    SenseTime_Memcpy(
                        &pExtFusionObjList->Objects[i].Legacy,
                        &pFusionIDManageObjectList->Objects[j].Legacy,
                        sizeof(LegacyObj_t));
                    SenseTime_Memcpy(
                        &pExtFusionObjList->Objects[i].Qualifiers,
                        &pFusionIDManageObjectList->Objects[j].Qualifiers,
                        sizeof(Qualifiers_t));
                    SenseTime_Memcpy(
                        &pExtFusionObjList->Objects[i].SensorSpecific,
                        &pFusionIDManageObjectList->Objects[j].SensorSpecific,
                        sizeof(SensorSpecific_t));
                    break;
                }
            }
        }
    }
}
/*****************************************************************************
  @fn           SRRObjectValidation

  @brief        SRR obejct validation process

  @description  calculate and output SRR object property based on sensor raw
data

  @param[in]    - short range sensor raw object from one SRR object
  @param[out]   - validation check result. Pass or Failed

  @return       -

  @pre          -

******************************************************************************/
boolean CheckFusionObjectValidation(const Objects_t sExtObject) {
    if (sExtObject.General.eObjMaintenanceState == MT_STATE_DELETED ||
        sExtObject.General.eObjMaintenanceState == MT_STATE_MERGE_DELETED) {
        return FALSE;
    }

    if (sExtObject.Geometry.fWidth < 0.f ||
        sExtObject.Geometry.fWidth > 51.0f ||
        sExtObject.Geometry.fLength < 0.f ||
        sExtObject.Geometry.fLength > 51.0f) {
        return FALSE;
    }

    if (sExtObject.Qualifiers.fProbabilityOfExistence <= 0.f ||
        sExtObject.Qualifiers.fProbabilityOfExistence > 1.0f ||
        fABS(sExtObject.SensorSpecific.fRCS) > 63.0f ||
        sExtObject.ObjectId < 0 ||
        sExtObject.ObjectId > TUE_SINGAL_FUSION_OBJECT_ID_MAX) {
        return FALSE;
    }

    if (fABS(sExtObject.Kinematic.fDistX) > 500.0f ||
        fABS(sExtObject.Kinematic.fDistY) > 200.0f ||
        fABS(sExtObject.Kinematic.fVrelX) > 100.0f ||
        fABS(sExtObject.Kinematic.fVrelY) > 60.0f ||
        fABS(sExtObject.Kinematic.fArelX) > 11.0f ||
        fABS(sExtObject.Kinematic.fArelY) > 6.0f ||
        fABS(sExtObject.Geometry.fOrientation) > TUE_CML_Pi) {
        return FALSE;
    }

    return TRUE;
}
/*****************************************************************************
  @fn           FusionObjectKinematicProcess

  @brief        Fusion obejct kinamatic property data output process

  @description  calculate and output Fusion object kinamatic property based on
sensor raw data

  @param[in]    - short range sensor raw object from one Fusion object
  @param[out]   - output object's kinamatic property

  @return       -

  @pre          -

******************************************************************************/
static void FusionObjectKinematicProcess(
    const Objects_t sExtObject,
    const float32 fEgoVelX,
    const float32 fEgoAccelX,
    const float32 fEgoYawRate,
    const float32 fCosEgoHeadingAngle,
    EM_Fusion_GenKinematics_t *pOutputObj) {
    pOutputObj->fAabsXStd_mpss = sExtObject.Kinematic.fArelXStd;
    pOutputObj->fAabsX_mpss = sExtObject.Kinematic.fArelX + fEgoAccelX;

    pOutputObj->fAabsYStd_mpss = sExtObject.Kinematic.fArelYStd;
    pOutputObj->fAabsY_mpss = sExtObject.Kinematic.fArelY;

    pOutputObj->fArelXStd_mpss = sExtObject.Kinematic.fArelXStd;
    pOutputObj->fArelX_mpss = sExtObject.Kinematic.fArelX;

    pOutputObj->fArelYStd_mpss = sExtObject.Kinematic.fArelYStd;
    pOutputObj->fArelY_mpss = sExtObject.Kinematic.fArelY;

    pOutputObj->fDistXStd_met = sExtObject.Kinematic.fDistXStd;
    pOutputObj->fDistX_met =
        sExtObject.Kinematic.fDistX;  //+ sSensorPosToRot.fPosX;

    pOutputObj->fDistYStd_met = sExtObject.Kinematic.fDistYStd;
    pOutputObj->fDistY_met =
        sExtObject.Kinematic.fDistY;  //+ sSensorPosToRot.fPosY;

    pOutputObj->fVabsXStd_mps = sExtObject.Kinematic.fVrelXStd;
    pOutputObj->fVabsX_mps =
        sExtObject.Kinematic.fVrelX + fEgoVelX * fCosEgoHeadingAngle;

    pOutputObj->fVabsYStd_mps = sExtObject.Kinematic.fVrelYStd;
    pOutputObj->fVabsY_mps =
        sExtObject.Kinematic.fVrelY + fEgoYawRate * pOutputObj->fDistX_met;

    pOutputObj->fVrelXStd_mps = sExtObject.Kinematic.fVrelXStd;
    pOutputObj->fVrelX_mps = sExtObject.Kinematic.fVrelX;

    pOutputObj->fVrelYStd_mps = sExtObject.Kinematic.fVrelYStd;
    pOutputObj->fVrelY_mps = sExtObject.Kinematic.fVrelY;
}
/*****************************************************************************
  @fn           FusionObjectGeometryProcess

  @brief        Fusion obejct Geometry property data output process

  @description  calculate and output Fusion object Geometry property based on
sensor raw data

  @param[in]    - short range sensor raw object from one Fusion object
  @param[out]   - output object's Geometry property

  @return       -

  @pre          -

******************************************************************************/
static void FusionObjectGeometryProcess(const Objects_t sExtObject,
                                        EM_Fusion_GenGeometry_t *pOutputObj) {
    pOutputObj->fWidth_met = sExtObject.Geometry.fWidth;
    pOutputObj->fLength_met = sExtObject.Geometry.fLength;
    pOutputObj->fLengthStd_met = 0.f;
    pOutputObj->fWidthStd_met = 0.f;

    pOutputObj->fLengthFront_met = sExtObject.Geometry.fLength / 2.0f;
    pOutputObj->fLengthRear_met = sExtObject.Geometry.fLength / 2.0f;
    pOutputObj->fWidthLeft_met = sExtObject.Geometry.fWidth / 2.0f;
    pOutputObj->fWidthRight_met = sExtObject.Geometry.fWidth / 2.0f;

    pOutputObj->fOrientationStd_rad = sExtObject.Geometry.fOrientationStd;
    pOutputObj->fOrientation_rad = sExtObject.Geometry.fOrientation;
}
/*****************************************************************************
  @fn           FusionObjectGeneralProcess

  @brief        Fusion obejct General property data output process

  @description  calculate and output Fusion object General property based on
sensor raw data

  @param[in]    - short range sensor raw object from one Fusion object
  @param[out]   - output object's General property

  @return       -

  @pre          -

******************************************************************************/
static void FusionObjectGeneralProcess(const Objects_t sExtObject,
                                       EM_Fusion_GenGenerals_t *pOutputObj) {
    pOutputObj->uiMaintenanceState_nu = sExtObject.General.eObjMaintenanceState;
    pOutputObj->uiID_nu = sExtObject.ObjectId;
    pOutputObj->iRawFusionID_nu = sExtObject.RawObjectID;

    uaObjectLifeCycle[sExtObject.ObjectId] =
        MIN(uaObjectLifeCycle[sExtObject.ObjectId] + 1, 65534u);

    pOutputObj->uiLifeCycles_nu = uaObjectLifeCycle[sExtObject.ObjectId];
    pOutputObj->fLifeTime_s =
        pOutputObj->uiLifeCycles_nu * FUSION_OBJECT_CYCLE_TIME;
}
/*****************************************************************************
  @fn           FusionObjectAttributeProcess

  @brief        Fusion obejct Attribute property data output process

  @description  calculate and output Fusion object Attribute property based on
sensor raw data

  @param[in]    - short range sensor raw object from one Fusion object
  @param[out]   - output object's Attribute property

  @return       -

  @pre          -

******************************************************************************/
static void FusionObjectAttributeProcess(
    const Objects_t sExtObject, EM_Fusion_GenAttributes_t *pOutputObj) {
    pOutputObj->eClassification_nu = sExtObject.Attributes.eClassification;
    pOutputObj->uiClassConfidence_per = 100u;
    pOutputObj->eDynamicProperty_nu = sExtObject.Attributes.eDynamicProperty;
    pOutputObj->uiDynConfidence_per = 100u;
}
/*****************************************************************************
  @fn           FusionObjectQualifierProcess

  @brief        Fusion obejct Qualifier property data output process

  @description  calculate and output Fusion object Qualifier property based on
sensor raw data

  @param[in]    - short range sensor raw object from one Fusion object
  @param[out]   - output object's Qualifier property

  @return       -

  @pre          -

******************************************************************************/
static void FusionObjectQualifierProcess(
    const Objects_t sExtObject, EM_Fusion_GenQualifiers_t *pOutputObj) {
    uint8 tempObjMeasFreq = 0;
    pOutputObj->bObjStable = TRUE;
    pOutputObj->fProbabilityOfExistence_per =
        sExtObject.Qualifiers.fProbabilityOfExistence;

    tempObjMeasFreq = uaHistoryMeasuredFrequency[sExtObject.ObjectId];
    tempObjMeasFreq >>= 1u;  // update measured frequency in the new cycle

    if (sExtObject.General.eObjMaintenanceState == MT_STATE_MEASURED ||
        sExtObject.General.eObjMaintenanceState == MT_STATE_MERGE_NEW) {
        // insert measured status while object is measured in current cycle
        tempObjMeasFreq |= (ui8_t)(1uL << (FUSION_CONFIRM_DENSITY_SIZE - 1uL));
    }
    uaHistoryMeasuredFrequency[sExtObject.ObjectId] = tempObjMeasFreq;
    pOutputObj->uiMeasuredTargetFrequency_nu = tempObjMeasFreq;
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */