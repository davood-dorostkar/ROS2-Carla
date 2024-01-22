/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "ved_consts.h"
#include "tue_common_libs.h"
#define VED__DACQ_IF 1L /* Activate interface to internal data */
#include "ved.h"

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
#include "ved_wpp.h"
#include "ved_ve.h"
#include "ved_wye.h"
#endif

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
#include "ved_gye.h"
#endif

#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
#include "ved_aye.h"
#endif

#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
#include "ved_sye.h"
#endif

#if ((!defined(CFG_VED__USE_EXT_PROC_YAW_RATE)) || \
     (!CFG_VED__USE_EXT_PROC_YAW_RATE))
#include "ved_ye.h"
#endif

#if ((!defined(CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION)) || \
     (!CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION))
#include "ved_sae.h"
#endif

#if (CFG_VED__MOT_STATE)
#include "ved_mot_st.h"
#endif

#ifndef VED__SIMU
#define VED__SIMU 0
#endif

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

#define NVM_CLEARED (uint32)(0xFFFFFFFFU)

/*****************************************************************************
  MACROS
*****************************************************************************/
#define VED__SET_FIRST_CYCLE_DONE (VED_ModIf.FirstCycleDone = TRUE)
#define VED__RESET_FIRST_CYCLE_DONE (VED_ModIf.FirstCycleDone = FALSE)
#define VED__IS_FIRST_CYCLE_DONE (VED_ModIf.FirstCycleDone)

#define VED__SET_CYCLE_TIME(time_) \
    (VED_ModIf.CycleTime = ((float32)(time_)) * 0.001F)
#define VED__GET_CYCLE_TIME (VED_ModIf.CycleTime)

/*--- Understeer gradient scaling --- */
#define VED__NVM_USTG_INV_SCALE (25000.F) /* ram -> nvram */

#if (!CFG_VED__DIS_YWR_OFFSET_COMP)
/* Offset elapesed time threshold to set sensor yaw rate quality */
#define VED__OFFSET_ELAPSED_TIME_QUALI_THRESH (1.0F)
#endif

/* Constant cycle time if cycle time is zero */
#define VED__CONST_CYCLE_TIME ((uint16)30U)
/* Maximum cycle time if cycle time too high */
#define VED__MAX_CYCLE_TIME ((uint16)110U)

#define VED__NO_PORT_FAULT ((uint8)0x00)
#define VED__REQUEST_PORT_FAULT ((uint8)0x01)
#define VED__PROVIDE_PORT_FAULT ((uint8)0x02)
#define VED__SERVICE_PORT_FAULT ((uint8)0x04)

/* curve error and confidence definitions */
#define YWR_OFFSET_ERROR_OUT_YWR_OFFS_NON                           \
    (BML_Deg2Rad(10.0F)) /* yaw rate offset error if learn state is \
                            YWR_OFFS_NON (no offset available) */
#define YWR_OFFSET_ERROR_OUT_YWR_OFFS_STANDST_SHORT                \
    (BML_Deg2Rad(2.5F)) /* yaw rate offset error if learn state is \
                           YWR_OFFS_STANDST_SHORT */
#define YWR_OFFSET_ERROR_OUT_YWR_OFFS_STANDST_FULL                 \
    (BML_Deg2Rad(1.0F)) /* yaw rate offset error if learn state is \
                           YWR_OFFS_STANDST_FULL */
#define YWR_OFFSET_ERROR_OUT_YWR_OFFS_STANDST_EEPROM               \
    (BML_Deg2Rad(2.5F)) /* yaw rate offset error if learn state is \
                           YWR_OFFS_STANDST_EEPROM */
#define YWR_OFFSET_ERROR_OUT_YWR_OFFS_DYN \
    (0.002F) /* yaw rate offset error if learn state is YWR_OFFS_DYN */

#define MIN_VELO_YAW_RATE_ERROR                                           \
    (1.0F) /* minimum velocity for yaw rate error calculation, below this \
              value the error is set to 0 */
#define YWR_OFFS_LIMIT \
    (BML_Deg2Rad(10)) /* threshold for yaw rate offset outside limit */
#ifndef MAX_SIGNAL_COUNTER
#define MAX_SIGNAL_COUNTER \
    (100U / 20U) /* debounce time for input signals: 100ms / cycle time */
#endif
#define MAX_HOLD_TIME                                                        \
    (400U / 20U) /* hold time before going down to lower confidence: 400ms / \
                    cycle time */
#define MAX_LAT_ERROR \
    (1.75F) /* maximum allowed lateral error of curves at given distances */
#define MIN_CURVE_FOR_APPROXIMATION \
    (0.00001F) /* curvatures below this value are considered straight */
#define TIME_DIST_CONF_6 (4.0F) /* time distance for status 6 */
#define TIME_DIST_CONF_5 (3.0F) /* time distance for status 6 */
#define TIME_DIST_CONF_4 (2.0F) /* time distance for status 6 */
#define TIME_DIST_CONF_3 (1.0F) /* time distance for status 6 */
#define TIME_EEPROM_TO_DYNAMIC (50U)

#ifndef VED__MAX_INPUT_DEBOUNCING
#define VED__MAX_INPUT_DEBOUNCING (3U)
#endif

#define FOUR_WHEEL_STEERING (2U)

#define VED__VAR_FAULT_COUNTER_THRESHOLD (5U)

#define VED__CALIMODE_ROLLER_BENCH (uint8)(0xFFU)

#if ((!defined(VED__SIMU)) || (!VED__SIMU))
#define VED__FREEZE_INIT_SEQUENCE (1)
#endif

#ifdef __PDO__
#if (defined(VED__FREEZE_INIT_SEQUENCE) && (VED__FREEZE_INIT_SEQUENCE))
/* VED init sequence size */
#define VED__INIT_SEQUENCE_SIZE (1024UL)
#define VED__INIT_SEQUENCE_DATA_SIZE (VED__INIT_SEQUENCE_SIZE - 4UL)
#endif
#endif

/*! Virtual addresses or related records IDs (in case of framework lookup table)
   for data acquisition interface. The addresses or IDs values must correspond
   to addresses specified in data description file (SDL, A2L etc.) */
#define VED__MEAS_ID_INPUT 0x20010000uL      /*!< Group VED_In         */
#define VED__MEAS_ID_CONFIG 0x20010800uL     /*!< Group VED_Config     */
#define VED__MEAS_ID_NVMEM_AUTO 0x20012300uL /*!< Group VED_NvAutoData */
#define VED__MEAS_ID_SWA 0x20013000uL        /*!< Group VED_Swa        */
#define VED__MEAS_ID_INIT_SEQ 0x20013500uL   /*!< Group VED_InitSeqence*/
#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
#define VED__MEAS_ID_WHS 0x20014000uL /*!< Group VED_Whs        */
#endif
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
#define VED__MEAS_ID_YWR 0x20015000uL /*!< Group VED_Ywr        */
#endif
#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
#define VED__MEAS_ID_AY 0x20016000uL /*!< Group VED_Lata       */
#endif
#if !(defined(CFG_VED__DIS_FUNCTIONAL_SAFETY_MON) && \
      (CFG_VED__DIS_FUNCTIONAL_SAFETY_MON))
#define VED__MEAS_ID_FS_MON 0x20016500uL /*!< Group VED_FsMon      */
#endif
#define VED__MEAS_ID_YE_K 0x20016700uL       /*!< Group VED_YeK      */
#define VED__MEAS_ID_TIMESTAMPS 0x20016800uL /*!< Group VED_DeltaTimeStamp */
#if (CFG_VED__INT_GYRO)
#define VED__MEAS_ID_YWRT 0x20017000uL    /*!< Group VED_Ywrt       */
#define VED__MEAS_ID_NV_YWRT 0x20350600uL /*!< Group YwrtTempTable */
#endif
#if (CFG_VED__YWR_OFFSET_MONITOR)
#define VED__MEAS_ID_YWR_MON 0x20017700uL /*!< Group YwrMon        */
#endif
#if (CFG_VED__DO_VELOCITY_CORR)
#define VED__MEAS_ID_VELCORR 0x2001C000uL /*!< Group VED_VelCorr    */
#if (CFG_VED__FS_VELO_CORR_MON)
#define VED__MEAS_ID_FSVELCORRMON 0x2001C500uL /*!< Group VED_FSVelCorrMon */
#endif
#endif
#define VED__MEAS_ID_INTDATA 0x2001E000uL   /*!< Group VED_IntData    */
#define VED__MEAS_ID_BAYESDATA 0x2001E800uL /*!< Group VED_BayesOut   */

/*----- Data acquisition interface functions, only active if configured */
#define MEAS_FREEZE_BUFFERED(ADDR_, DATA_, SIZE_)
#define MEAS_FREEZE_DIRECT(ADDR_, DATA_, SIZE_)

#ifdef __PDO__
typedef VED_Config_t
    PDOVED_Config; /*!< @VADDR: VED__MEAS_ID_CONFIG @VNAME: VED_Config @ALLOW:
                      ved__priv @cycleid: ved__cycle_id */
#if (defined(VED__FREEZE_INIT_SEQUENCE) && (VED__FREEZE_INIT_SEQUENCE))
typedef uint8 PDOVED_InitSeqence
    [VED__INIT_SEQUENCE_SIZE]; /*!< @VADDR: VED__MEAS_ID_INIT_SEQ
                                  @VNAME:VED_InitSequence @ALLOW: ved__priv
                                  @CYCLEID: ved__cycle_id @VALUES: struct {
                                  uint32 InitPackageID; uint8
                                  rawdata[VED__INIT_SEQUENCE_DATA_SIZE]; } */
#endif
#endif

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*! Frame counter data */
typedef struct {
    uint16 ExecCntr; /*!< Execution cycle counter */
} VED_EnvFrameCounter_t;

/* Kalman gain data from YE Kalman filter */
typedef struct {
    real32_T K_yaw[8];
    real32_T K_curve[4];
    uint8_T K_yaw_fault;
    uint8_T K_curve_fault;
    uint8 u_CurveFaultCounter;
    uint8 u_YawFaultCounter;
    uint8 u_DICurveFaultCounter;
} VED_Ye_K_t;

/* Timestamp differences */
typedef struct {
    uint32 ui_DeltaTimeBetweenCalls; /* Time difference between the last 2 VED
                                        calls */
    uint32
        ui_DeltaTimeDataAndExecution; /* Time difference between VehSig input
                                         data and system time at processing */
} VED_DeltaTimeStamp_t;

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Internal interface of autocode modules */
SET_MEMSEC_VAR(ved__internal_data)
VED_InternalData_t
    ved__internal_data; /*!< @VNAME: VED_IntData @VADDR: 0x2001E000 @ALLOW:
                           ved__priv @cycleid: ved__cycle_id*/
SET_MEMSEC_VAR(ved__bayes_mot_states)
ved__bayes_mot_states_t
    ved__bayes_mot_states; /*!< @VNAME: VED_BayesOut @VADDR: 0x2001E800 @ALLOW:
                              ved__priv @cycleid: ved__cycle_id*/
SET_MEMSEC_VAR(ved__ye_k)
static VED_Ye_K_t ved__ye_k; /*!< @VNAME: VED_YeK @VADDR: 0x20016700uL @ALLOW:
                                ved__priv @cycleid: ved__cycle_id*/
static VED_DeltaTimeStamp_t
    VED_DeltaTimeStamp; /*!< @VNAME: VED_DeltaTimeStamp @VADDR: 0x20016800uL
                           @ALLOW: ved__priv @cycleid: ved__cycle_id*/

/*! Frame counter to maintance execution */
SET_MEMSEC_VAR(VED_EnvFrmCnt)
static VED_EnvFrameCounter_t VED_EnvFrmCnt;

/*! The sequence init State */
SET_MEMSEC_VAR(sSequenceInitState)
static VED_SequenceInitStates_t sSequenceInitState;

#if ((!defined(CFG_VED__DIS_YWR_OFFSET_COMP)) || \
     (!CFG_VED__DIS_YWR_OFFSET_COMP))
/*! The yaw rate offset output state */
SET_MEMSEC_VAR(VED_OutYwrOffsType)
static VED_OutYwrOffsType_t VED_OutYwrOffsType;
#endif

/*! Simulation synchronisation counter of input data */
static VED__SM_t_SyncRef
    s_SyncRef; /*!< @VNAME: VED__SyncRef @VADDR: VED_MEAS_ID_SYNC_REF @ALLOW:
                  ved__priv @cycleid: ved__cycle_id*/

static V1_7_VEDVehSigMain_t
    s_LastInputSignals; /* buffer to debounce the input signals in case of
                           invalid inputs */
static uint8 u_DebouceCounter[VED_SIN_POS_MAX]; /* counter for number of cycles
                                                   a signal was debounced */
#if ((defined(CFG_VED__REDUCE_CURVE_ERROR)) && (CFG_VED__REDUCE_CURVE_ERROR))
static uint32 u_count; /* counter for reducing the curve error at system reset
                          situations */
#endif
static boolean b_RTBDetection; /* instance for Roller test bench recognition  */

/*****************************************************************************
 MODULE LOCAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL VARIABLES
*****************************************************************************/

/* Buffer for module interface hand code auto code */
SET_MEMSEC_VAR(VED_ModIf)
static VED_ModIf_t VED_ModIf;

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
/* ved__wpp modul */
SET_MEMSEC_VAR(ved__wpp_M)
static RT_MODEL_ved__wpp ved__wpp_M; /* Real-time model */
SET_MEMSEC_VAR(ved__wpp_B)
static BlockIO_ved__wpp ved__wpp_B; /* Observable signals */
SET_MEMSEC_VAR(ved__wpp_DWork)
static D_Work_ved__wpp ved__wpp_DWork; /* Observable states */

/* ved__ve modul */
SET_MEMSEC_VAR(ved__ve_M)
static RT_MODEL_ved__ve ved__ve_M; /* Real-time model */
SET_MEMSEC_VAR(ved__ve_B)
static BlockIO_ved__ve ved__ve_B; /* Observable signals */
SET_MEMSEC_VAR(ved__ve_DWork)
static D_Work_ved__ve ved__ve_DWork; /* Observable states */

/* ved__wye modul */
SET_MEMSEC_VAR(ved__wye_M)
static RT_MODEL_ved__wye ved__wye_M; /* Real-time model */
SET_MEMSEC_VAR(ved__wye_B)
static BlockIO_ved__wye ved__wye_B; /* Observable signals */
SET_MEMSEC_VAR(ved__wye_DWork)
static D_Work_ved__wye ved__wye_DWork; /* Observable states */
#endif

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
/* ved__gye modul */
SET_MEMSEC_VAR(ved__gye_M)
static RT_MODEL_ved__gye ved__gye_M; /* Real-time model */
SET_MEMSEC_VAR(ved__gye_B)
static BlockIO_ved__gye ved__gye_B; /* Observable signals */
SET_MEMSEC_VAR(ved__gye_DWork)
static D_Work_ved__gye ved__gye_DWork; /* Observable states */

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING))
/* ved__gye modul to filter and output raw yaw rate  */
SET_MEMSEC_VAR(ved__gye_M_FILT)
static RT_MODEL_ved__gye ved__gye_M_FILT; /* Real-time model */
SET_MEMSEC_VAR(ved__gye_B_FILT)
static BlockIO_ved__gye ved__gye_B_FILT; /* Observable signals */
SET_MEMSEC_VAR(ved__gye_DWork_FILT)
static D_Work_ved__gye ved__gye_DWork_FILT; /* Observable states */
SET_MEMSEC_VAR(LastFiltered)
static float32 LastFiltered = 0.0F; /* Last filtered dyn yaw rate offset */
SET_MEMSEC_VAR(OldFiltYawOffset)
static float32 OldFiltYawOffset = 0.0F; /* Filter delay for yaw rate offset */
#endif
#endif

#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
/* ved__aye modul */
SET_MEMSEC_VAR(ved__aye_M)
static RT_MODEL_ved__aye ved__aye_M; /* Real-time model */
SET_MEMSEC_VAR(ved__aye_DWork)
static D_Work_ved__aye ved__aye_DWork; /* Observable states */
#endif

#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
/* ved__sye modul */
SET_MEMSEC_VAR(ved__sye_M)
static RT_MODEL_ved__sye ved__sye_M; /* Real-time model */
SET_MEMSEC_VAR(ved__sye_B)
static BlockIO_ved__sye ved__sye_B; /* Observable signals */
SET_MEMSEC_VAR(ved__sye_DWork)
static D_Work_ved__sye ved__sye_DWork; /* Observable states */
#endif

#if ((!defined(CFG_VED__USE_EXT_PROC_YAW_RATE)) || \
     (!CFG_VED__USE_EXT_PROC_YAW_RATE))
/* ved__ve modul */
SET_MEMSEC_VAR(ved__ye_M)
static RT_MODEL_ved__ye ved__ye_M; /* Real-time model */
SET_MEMSEC_VAR(ved__ye_B)
static BlockIO_ved__ye ved__ye_B; /* Observable signals */
SET_MEMSEC_VAR(ved__ye_DWork)
static D_Work_ved__ye ved__ye_DWork; /* Observable states */
#endif

#if ((!defined(CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION)) || \
     (!CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION))
/* ved__sae modul */
SET_MEMSEC_VAR(ved__sae_M)
static RT_MODEL_ved__sae ved__sae_M; /* Real-time model */
SET_MEMSEC_VAR(ved__sae_DWork)
static D_Work_ved__sae ved__sae_DWork; /* Observable states */
#endif

#if (CFG_VED__MOT_STATE)
/* ved__mot_st modul motion state */
SET_MEMSEC_VAR(ved__mot_st_M)
static RT_MODEL_ved__mot_st ved__mot_st_M; /* Real-time model */
SET_MEMSEC_VAR(ved__mot_st_B)
static BlockIO_ved__mot_st ved__mot_st_B; /* Observable signals */
SET_MEMSEC_VAR(ved__mot_st_DWork)
static D_Work_ved__mot_st ved__mot_st_DWork; /* Observable states */
#endif

#if (defined(VED__FREEZE_INIT_SEQUENCE) && (VED__FREEZE_INIT_SEQUENCE))
SET_MEMSEC_VAR(InitPackageID)
static uint32 InitPackageID;
#endif

#if (!(CFG_VED__USE_EX_LONG_ACCEL) && (CFG_VED__USE_EX_LONG_VELO))
/* External velocity without related acceleration is provided */
SET_MEMSEC_VAR(VED_ModIf)
static float32 oldVelocityIn;
#endif
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

/*****************************************************************************
  LOCAL FUNCTION PROTOTYPES
*****************************************************************************/
static void VED_Process(const reqVEDPrtList_t *reqPorts,
                        VED_InputData_t *VED_In,
                        proVEDPrtList_t *proPorts);
static void VED_InitProcess(const reqVEDPrtList_t *reqPorts,
                            const VED_InputData_t *input,
                            const proVEDPrtList_t *proPorts);
static void VED_Init(const reqVEDPrtList_t *reqPorts,
                     const reqVEDParams_t *reqParams,
                     VED_InputData_t *VED_In,
                     const proVEDPrtList_t *proPorts);

static void VED_GetConfig(void);

static void VED_LearnInitWithParams(const VED_InputData_t *input,
                                    VED_NvData_t *nvdata);
static void VED_PreProcess(const reqVEDPrtList_t *reqPorts,
                           const VED_InputData_t *input,
                           VED_NvData_t *VED_NvData,
                           const proVEDPrtList_t *proPorts);
static void VED_HandcodeProcess(const reqVEDPrtList_t *reqPorts,
                                const VED_InputData_t *input,
                                const proVEDPrtList_t *proPorts);
static void VED_AutocodeProcess(const reqVEDPrtList_t *reqPorts,
                                VED_InputData_t *input,
                                VED_NvData_t *nvdata,
                                VED_Errors_t *VED_Errors);

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
static void VED_ProcAutoVelocity(VED_InputData_t *input);
#endif
static void VED_ProcAutoWheelYawRate(VED_InputData_t *input,
                                     VED_NvData_t *nvdata);

#if (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)
#if (CFG_VED__INT_GYRO)
static void VED_ProcAutoSenIntYawRateOffset(void);
#endif
static void VED_ProcAutoSenExtYawRateOffset(void);
#endif
static void VED_ProcAutoSenYawRate(VED_InputData_t *input);

static void VED_ProcAutoLatAccelYawRate(VED_InputData_t *input);
static void VED_ProcAutoSwaYawRate(VED_InputData_t *input,
                                   VED_NvData_t *nvdata);
static void VED_ProcAutoVehicleYawRate(VED_InputData_t *input);
static void VED_ProcAutoSideSlip(VED_InputData_t *input);

#if (CFG_VED__MOT_STATE)
static void VED_ProcMotionState(VED_InputData_t *input,
                                VEDALN_Monitoring_t *ALNMonitoring);
#endif

static void VED_PostProcess(const reqVEDPrtList_t *reqPorts,
                            const VED_InputData_t *input,
                            const VED_NvData_t *VED_NvData,
                            proVEDPrtList_t *proPorts);
static void CopyVehParamToOutput(const reqVEDPrtList_t *reqPorts,
                                 proVEDPrtList_t *proPorts);

static void VED_EnvMeasFreezeInternal(void);

#if ((!CFG_VED__DIS_YWR_OFFSET_COMP) &&                     \
     ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
      (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)))
#if (CFG_VED__INT_GYRO)
static float32 VED_GenInternalYawRateQuality(const VED_InternalData_t *IntData,
                                             const proVEDPrtList_t *proPorts,
                                             const VED_InputData_t *input);
#endif
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING))
static float32 VED_GenExternalYawRateQuality(const VED_InternalData_t *IntData,
                                             const proVEDPrtList_t *proPorts,
                                             const VED_InputData_t *input);
#else
static float32 VED_GenExternalYawRateQuality(const proVEDPrtList_t *proPorts,
                                             const VED_InputData_t *input);
#endif
#endif
static void VED_GenYawAndCurveQualitys(const VED_InternalData_t *IntData,
                                       proVEDPrtList_t *proPorts,
                                       const VED_InputData_t *input);

#if (CFG_VED__GEN_VELOCITY_VARIANCE)
static void VED_GenVeloVar(VED_InternalData_t *IntData);
#endif

static void VED_InitVehDyn(VEDVehDyn_t *VehDyn);

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
static void VED_InitALNYawRate(ALNYawRate_t *ALNYawRate);
#endif

static void VED_InitOffsets(VED_VEDOffsets_t *VED_Offsets);
static void VED_InitErrors(VED_Errors_t *VED_Errors);
static void VED_InitModuleIf(VED_ModIf_t *mif);
static void VED_InitInternal(VED_InternalData_t *intData);
static void VED_ReinitYawRate(void);

static void VED_InterfaceHand2Auto(const VED_InputData_t *input,
                                   VED_InternalData_t *IntData,
                                   const VED_ModIf_t *mif);
#if (CFG_VED__GEN_VELOCITY_VARIANCE)
static void VED_InterfaceAuto2Hand(VED_InternalData_t *IntData,
                                   VED_ModIf_t *mif);
#else
static void VED_InterfaceAuto2Hand(const VED_InternalData_t *IntData,
                                   VED_ModIf_t *mif);
#endif

static void VED_YawRate2Curve(const VED_InternalData_t *intData,
                              const proVEDPrtList_t *proPorts);

#if (defined(VED__FREEZE_INIT_SEQUENCE) && (VED__FREEZE_INIT_SEQUENCE))
static void VED_InitSequenceMeasFreeze(void);
#endif

static void VED_SetSyncFrame(const reqVEDPrtList_t *reqPorts,
                             const reqVEDParams_t *reqParams,
                             uint16 CycleCnt,
                             uint8 eVED_SigStatus);
static uint8 VED_CheckPorts(const reqVEDPrtList_t *reqPorts,
                            const reqVEDParams_t *reqParams,
                            const proVEDPrtList_t *proPorts);

static float32 VED_CalculateCurveLatDistance(float32 f_c1,
                                             float32 f_c2,
                                             float32 f_b);
static float32 VED_CalculateCurveError(const VED_InputData_t *input,
                                       const VED_VEDOffsets_t *pVED_Offsets);
static void VED_CalculateLaneErrorConf(const reqVEDPrtList_t *reqPorts,
                                       const VED_InputData_t *input,
                                       const VED_VEDOffsets_t *pVED_Offsets,
                                       VEDVehDyn_t *pVehDyn);
static void VED_CopyInputSignals(V1_7_VEDVehSigMain_t *p_Input,
                                 const V1_7_VEDVehSigMain_t *p_VehSigMain);
static void VED__f_CheckInputSignals(float32 *p_InputSignal,
                                     float32 f_VehSigMainSignal,
                                     float32 *p_LastSignal,
                                     uint8 *p_VED_InState,
                                     uint8 e_VehSigState,
                                     uint8 *p_LastState,
                                     uint8 *p_DebouceCounter);
static void VED__u_CheckInputSignals(uint8 *p_InputSignal,
                                     uint8 u_VehSigMainSignal,
                                     uint8 *p_LastSignal,
                                     uint8 *p_VED_InState,
                                     uint8 e_VehSigState,
                                     uint8 *p_LastState,
                                     uint8 *p_DebouceCounter);
static void VED__s_CheckInputSignals(uint16 *p_InputSignal,
                                     uint16 u_VehSigMainSignal,
                                     uint16 *p_LastSignal,
                                     uint8 *p_VED_InState,
                                     uint8 e_VehSigState,
                                     uint8 *p_LastState,
                                     uint8 *p_DebouceCounter);

void VED__MTS_Callback(void) { return; }

/* **********************************************************************
  @fn               VEDExec */ /*!

  @brief            Main loop for VED, called from outside

  @description      Determine operating sequence for vehicle dynamics observer
                    Collect input signals
                    Freeze provided ports
*/
/*!
  @param[in]        proPorts requested ports
                    VED_AS_t_ServiceFuncts servicefunctions
  @param[in]        Services
  @param[out]       reqPorts provided ports
  */

/*!
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VEDExec(const reqVEDPrtList_t *reqPorts,
             const reqVEDParams_t *reqParams,
             proVEDPrtList_t *proPorts) {
    static VED_InputData_t VED_In;
    uint8 ui8PortFault;
    uint32 uiTimeStamp;
    uint32 uiSystemTimeStamp;
    static uint32 uiTimeStampLastCycle = 0U;
    uint8 eVED_SigStatus;

    ui8PortFault = VED_CheckPorts(reqPorts, reqParams, proPorts);

    if (ui8PortFault == VED__NO_PORT_FAULT) {
        /* copy cycle time, control state and calibration state to the frame
         * data */
        VED_In.Frame.CycleTime = reqPorts->pCtrl->CycleTime;
        VED_In.Frame.CtrlMode = reqPorts->pCtrl->CtrlMode;
        VED_In.Frame.CaliMode = reqPorts->pCtrl->CaliMode;

        // and by LiuYang 2019-05-25 for debug
        VED_In.Frame.CaliMode = 0;

/* Process all calibration learning on roll bench detection and driving*/
#if (VED_VEH_DYN_INTFVER >= 8U)
        if (CORRECTED_EGO_SPEED_IS_ZERO_ON_ROLLER_BENCH != 1) {
            /* Disable all offset, WLD and Selfsteering Gradient learnings when
             * Ego velocity is not set to zero on roller test bench */
            if (b_RTBDetection == TRUE) {
                VED_In.Frame.CaliMode = VED__CALIMODE_ROLLER_BENCH;
            }
        }
#endif

        /*! Avoid zero or too large cycle times, 20 ms is nominal cycle time */
        VED_In.Frame.CycleTime =
            (((VED_In.Frame.CycleTime > (uint16)0U) &&
              (VED_In.Frame.CycleTime < VED__MAX_CYCLE_TIME))
                 ? VED_In.Frame.CycleTime
                 : VED__CONST_CYCLE_TIME);

        /* Initialize output */
        VED_InitVehDyn(proPorts->pVehicleDynamicSignals);

        proPorts->pNVMWrite->uiVersionNumber = VED_NVM_IO_DATA_INTFVER;

        /* VED execution */
        if (VED_In.Frame.CtrlMode <= (uint8)VED_CTRL_STATE_RUNNING) {
            if (VED_In.Frame.CtrlMode != (uint8)VED_CTRL_STATE_STARTUP) {
                /* copy input signals */
                VED_CopyInputSignals(
                    &VED_In.Signals,
                    &reqPorts->pVehicleInputSignals->VehSigMain);
                /* copy input parameters */
                (void)memcpy(&VED_In.Parameter, reqParams,
                             sizeof(reqVEDParams_t));

                /* Process VED in case of init and running mode */
                VED_Process(reqPorts, &VED_In, proPorts);
                eVED_SigStatus = AL_SIG_STATE_OK;
            } else {
                /* Initialize component in startup mode */
                VED_Init(reqPorts, reqParams, &VED_In, proPorts);
                reqPorts->pCtrl->CtrlMode =
                    VED_CTRL_STATE_RUNNING;  // and by liuyang 2019-07-07
                eVED_SigStatus = AL_SIG_STATE_INIT;
            }
        } else {
            /* Undefined operation mode, re-init, no processing */
            VED_Init(reqPorts, reqParams, &VED_In, proPorts);
            reqPorts->pCtrl->CtrlMode =
                VED_CTRL_STATE_RUNNING;  // and by liuyang 2019-07-07
            eVED_SigStatus = AL_SIG_STATE_INVALID;
        }
    } else {
        /* Critical error, no input data available */
        eVED_SigStatus = AL_SIG_STATE_INVALID;

        /* Set cycle counter in the internal buffer to reset values */
        VED_In.Frame.CycleCnt = 0;
    }

    if ((ui8PortFault & VED__PROVIDE_PORT_FAULT) != VED__PROVIDE_PORT_FAULT) {
        /* Add the input timestamp and the ved_ cycle counter to the ved_ output
         * data */
        uiTimeStamp = reqPorts->pVehicleInputSignals->sSigHeader.uiTimeStamp;
        uiSystemTimeStamp = reqPorts->pCtrl->uiSystemTimeStamp_us;

        /* if no timestamp was delivered */
        if ((uiTimeStamp == 0U) && (uiTimeStampLastCycle == 0U)) {
            /* take the current time as timestamp for output data */
            uiTimeStamp = uiSystemTimeStamp;
        } else {
            /* save current timestamp for next cycle */
            uiTimeStampLastCycle = uiTimeStamp;
        }

        /* calculate delta times between the last 2 calls and the VehSig
         * timestamp and the current system time */
        if (uiTimeStamp > 0U) {
            VED_DeltaTimeStamp.ui_DeltaTimeBetweenCalls =
                uiTimeStamp - uiTimeStampLastCycle;
        }
        VED_DeltaTimeStamp.ui_DeltaTimeDataAndExecution =
            uiSystemTimeStamp - uiTimeStamp;

        VED_SetSyncFrame(reqPorts, reqParams, VED_In.Frame.CycleCnt,
                         eVED_SigStatus);

        /* Fill the signal headers of all output ports */
        proPorts->pVehicleDynamicSignals->sSigHeader.eSigStatus =
            eVED_SigStatus;
        proPorts->pVehicleDynamicSignals->sSigHeader.uiTimeStamp = uiTimeStamp;
        proPorts->pVehicleDynamicSignals->sSigHeader.uiMeasurementCounter =
            reqPorts->pVehicleInputSignals->sSigHeader.uiCycleCounter;
        proPorts->pVehicleDynamicSignals->sSigHeader.uiCycleCounter =
            VED_In.Frame.CycleCnt;

        proPorts->pVED_Offsets->sSigHeader.eSigStatus = eVED_SigStatus;
        proPorts->pVED_Offsets->sSigHeader.uiTimeStamp = uiTimeStamp;
        proPorts->pVED_Offsets->sSigHeader.uiMeasurementCounter =
            reqPorts->pVehicleInputSignals->sSigHeader.uiCycleCounter;
        proPorts->pVED_Offsets->sSigHeader.uiCycleCounter =
            VED_In.Frame.CycleCnt;

        proPorts->pVED_EstCurves->sSigHeader.eSigStatus = eVED_SigStatus;
        proPorts->pVED_EstCurves->sSigHeader.uiTimeStamp = uiTimeStamp;
        proPorts->pVED_EstCurves->sSigHeader.uiMeasurementCounter =
            reqPorts->pVehicleInputSignals->sSigHeader.uiCycleCounter;
        proPorts->pVED_EstCurves->sSigHeader.uiCycleCounter =
            VED_In.Frame.CycleCnt;

        proPorts->pVED_Errors->sSigHeader.eSigStatus = eVED_SigStatus;
        proPorts->pVED_Errors->sSigHeader.uiTimeStamp = uiTimeStamp;
        proPorts->pVED_Errors->sSigHeader.uiMeasurementCounter =
            reqPorts->pVehicleInputSignals->sSigHeader.uiCycleCounter;
        proPorts->pVED_Errors->sSigHeader.uiCycleCounter =
            VED_In.Frame.CycleCnt;

        proPorts->pNVMWrite->sSigHeader.eSigStatus = eVED_SigStatus;
        proPorts->pNVMWrite->sSigHeader.uiTimeStamp = uiTimeStamp;
        proPorts->pNVMWrite->sSigHeader.uiMeasurementCounter =
            reqPorts->pVehicleInputSignals->sSigHeader.uiCycleCounter;
        proPorts->pNVMWrite->sSigHeader.uiCycleCounter = VED_In.Frame.CycleCnt;

        if ((ui8PortFault & VED__SERVICE_PORT_FAULT) !=
            VED__SERVICE_PORT_FAULT) {
            /* Freeze provided ports */
            MEAS_FREEZE_DIRECT(VED_MEAS_ID_VEH_DYN,
                               proPorts->pVehicleDynamicSignals,
                               sizeof(VEDVehDyn_t));
            MEAS_FREEZE_DIRECT(VED_MEAS_ID_OFFSETS, proPorts->pVED_Offsets,
                               sizeof(VED_VEDOffsets_t));
            MEAS_FREEZE_DIRECT(VED_MEAS_ID_EST_CURVES, proPorts->pVED_EstCurves,
                               sizeof(VED_EstCurves_t));
            MEAS_FREEZE_DIRECT(VED_MEAS_ID_ERRORS, proPorts->pVED_Errors,
                               sizeof(VED_Errors_t));
            MEAS_FREEZE_DIRECT(VED_MEAS_ID_NVM_WRITE, proPorts->pNVMWrite,
                               sizeof(VEDNvIoDatas_t));
        } else {
            /* no service functions available, skip meas output */
        }
    }
}

/* ***********************************************************************
  @fn               VED_Process */

static void VED_Process(const reqVEDPrtList_t *reqPorts,
                        VED_InputData_t *VED_In,
                        proVEDPrtList_t *proPorts) {
    /* Local instance of some NVM data used to copy it from const input pointer
     * and modify them */
    VED_NvData_t VED_NvData;

    VED_In->Frame.Version = VED__SW_VERSION_NUMBER;

    /* Increment execution cyle counter */
    VED_In->Frame.CycleCnt = VED_EnvFrmCnt.ExecCntr;
    VED_EnvFrmCnt.ExecCntr++;

#if ((defined(CFG_VED__REAR_WHEEL_STEERING)) && (CFG_VED__REAR_WHEEL_STEERING))
#if (BSW_VEH_PAR_INTFVER >= 9U)
    /*manipulate the Steering wheel angle as effective steering wheel angle
    considering the rear wheel angle for 4 wheel steering ( based on steering
    wheel number parameter */
    if (reqPorts->pVehicleParameter->VehParAdd.SteeringWheelNumber ==
        FOUR_WHEEL_STEERING)
#endif
    {
        /* check the valididty of both signals and non zero condition for
         * Steering ratio*/
        if ((VED_GET_IO_STATE(VED_SIN_POS_SWA, VED_In->Signals.State) !=
             VED_IO_STATE_INVALID) &&
            (VED_GET_IO_STATE(VED_SIN_POS_SWA, VED_In->Signals.State) !=
             VED_IO_STATE_INIT) &&
            (VED_GET_IO_STATE(VED_SIN_POS_SWA, VED_In->Signals.State) !=
             VED_IO_STATE_NOTAVAIL)) {
            if ((VED_GET_IO_STATE(VED_SIN_POS_RSTA, VED_In->Signals.State) !=
                 VED_IO_STATE_INVALID) &&
                (VED_GET_IO_STATE(VED_SIN_POS_RSTA, VED_In->Signals.State) !=
                 VED_IO_STATE_INIT) &&
                (VED_GET_IO_STATE(VED_SIN_POS_RSTA, VED_In->Signals.State) !=
                 VED_IO_STATE_NOTAVAIL)) {
                if (TUE_CML_IsNonZero(
                        VED_In->Parameter.SteeringRatio.swa.rat[1])) {
                    VED_In->Signals.StWheelAngle =
                        ((VED_In->Signals.StWheelAngle /
                          VED_In->Parameter.SteeringRatio.swa.rat[1]) -
                         VED_In->Signals.RearWhlAngle) *
                        (VED_In->Parameter.SteeringRatio.swa.rat[1]);
                }
            }
        }
    }

#endif

    /* Monitor input signals and parameter only if the ved_ is in running mode
     * and INIT mode*/
    VED_MonitorInput(VED_In, proPorts->pVED_Errors);

/* Initalizing the bRollerTestBench variable*/
#if (VED_VEH_DYN_INTFVER >= 8U)
    proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.bRollerTestBench =
        false;
#endif

    /* Process velocity during roll bench detection*/
    if (b_RTBDetection == TRUE) {
#if (VED_VEH_DYN_INTFVER < 8U)
        {
            VED_In->Signals.VehVelocityExt = 0;
            VED_In->Signals.WhlVelFrLeft = 0;
            VED_In->Signals.WhlVelFrRight = 0;
            VED_In->Signals.WhlVelReLeft = 0;
            VED_In->Signals.WhlVelReRight = 0;
        }
#elif (VED_VEH_DYN_INTFVER >= 8U)
        {
            /* not setting the corrected ego speed to zero. instead set the new
             * flag.  */
            proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr
                .bRollerTestBench = true;
            if (CORRECTED_EGO_SPEED_IS_ZERO_ON_ROLLER_BENCH == 1) {
                VED_In->Signals.VehVelocityExt = 0;
                VED_In->Signals.WhlVelFrLeft = 0;
                VED_In->Signals.WhlVelFrRight = 0;
                VED_In->Signals.WhlVelReLeft = 0;
                VED_In->Signals.WhlVelReRight = 0;
            }
        }
#endif
    }

    /* Transfer component input interface data and some additional infos to data
     * acquisition */
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_INPUT, VED_In, sizeof(VED_InputData_t));

    /* VED Pre Processing setup output states in condition of input states */
    VED_PreProcess(reqPorts, VED_In, &VED_NvData, proPorts);

    /* Process VED handcode modules */
    VED_HandcodeProcess(reqPorts, VED_In, proPorts);

    /* Interface between handcode (hd) and autocde (ac) */
    VED_InterfaceHand2Auto(VED_In, &ved__internal_data, &VED_ModIf);

    /* Process VED autocode modules */
    VED_AutocodeProcess(reqPorts, VED_In, &VED_NvData, proPorts->pVED_Errors);

    /* Interface between autocde (ac) and  handcode (hd) */
    VED_InterfaceAuto2Hand(&ved__internal_data, &VED_ModIf);

    /* VED Post Processing setup output states in condition of input states */
    VED_PostProcess(reqPorts, VED_In, &VED_NvData, proPorts);

    /* Transfer component internal data to data acquisition */
    VED_EnvMeasFreezeInternal();

    /* Transfer component output interface data to data acquisition */
#if (CFG_VED__INT_GYRO)
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_NV_YWRT, proPorts->pYwrtTempTable,
                       sizeof(VED_NvYwrtLearnTable_t));

#endif
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_NVMEM_AUTO, &VED_NvData,
                       sizeof(VED_NvData_t));
}

/* ***********************************************************************
  @fn               VED_Init */ /*!

  @brief            Initialize all component data

  @description      Handles initialisation of VED

  @param[in]        reqPorts requested ports
  @param[in]        Services servicefunctions
  @param[in]        VED_In
  @param[out]       proPorts provided ports
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_Init(const reqVEDPrtList_t *reqPorts,
                     const reqVEDParams_t *reqParams,
                     VED_InputData_t *VED_In,
                     const proVEDPrtList_t *proPorts) {
    /* Init cycle counters */
    VED_EnvFrmCnt.ExecCntr = 0U;
    VED_In->Frame.CycleCnt = 0U;

    /* copy input signals */
    (void)memcpy(&VED_In->Signals, &reqPorts->pVehicleInputSignals->VehSigMain,
                 sizeof(V1_7_VEDVehSigMain_t));
    /* copy input parameters */
    (void)memcpy(&VED_In->Parameter, &reqParams, sizeof(reqVEDParams_t));
    /* copy input signals to debouncing buffer */
    (void)memcpy(&s_LastInputSignals,
                 &reqPorts->pVehicleInputSignals->VehSigMain,
                 sizeof(V1_7_VEDVehSigMain_t));
    /* clear debouncing counter */
    (void)memset(u_DebouceCounter, 0x00, VED_SIN_POS_MAX);

#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
    {
        uint32 ioStateSSG;
        uint32 ioStateWLD;
        /* SSG and WLD offset writing in init mode */
        if (reqPorts->pNVMRead->State == NVM_CLEARED) {
            (void)memset(&proPorts->pNVMWrite->SlfstGrad, 0,
                         sizeof(VEDNvSlfStGradCalc_t));
            (void)memset(&proPorts->pNVMWrite->Wld, 0, sizeof(VEDNvWldCalc_t));
        } else {
            proPorts->pNVMWrite->SlfstGrad = reqPorts->pNVMRead->SlfstGrad;
            proPorts->pNVMWrite->Wld = reqPorts->pNVMRead->Wld;
#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
            proPorts->pNVMWrite->SlfstGrad.Dummy = 0;
            proPorts->pNVMWrite->Wld.Wld_rear = 0;
            proPorts->pNVMWrite->Wld.Wld_rear_quality = 0;
#endif

            ioStateSSG = VED__GET_NVM_IO_STATE(VED_NVM_POS_SSG,
                                               &reqPorts->pNVMRead->State);
            VED_SET_NVM_IO_STATE(VED_NVM_POS_SSG, ioStateSSG,
                                 &proPorts->pNVMWrite->State);

            ioStateWLD = VED__GET_NVM_IO_STATE(VED_NVM_POS_WLD,
                                               &reqPorts->pNVMRead->State);
            VED_SET_NVM_IO_STATE(VED_NVM_POS_WLD, ioStateWLD,
                                 &proPorts->pNVMWrite->State);
        }
    }
#endif

    /* copy input signals for signal peak detection */
    // LiuYang 2019-05-05 for debug // VED_InitForCheckSignalPeakErrors(VED_In);

    /* Initialization of component */
    VED_InitProcess(reqPorts, VED_In, proPorts);

    /* Transfer component input interface data and some additional infos to data
     * acquisition */
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_INPUT, VED_In, sizeof(VED_InputData_t));

    VED_EnvMeasFreezeInternal();

/* Init sequence package counter */
#if (defined(VED__FREEZE_INIT_SEQUENCE) && (VED__FREEZE_INIT_SEQUENCE))
    VED__SET_ALL_SEQUENCE_STATES_TO_INIT(sSequenceInitState);
    InitPackageID = 0U;
#endif

/* Init acceleration velocity filter */
#if (!(CFG_VED__USE_EX_LONG_ACCEL) && (CFG_VED__USE_EX_LONG_VELO))
    /* External velocity without related acceleration is provided */
    oldVelocityIn = s_LastInputSignals.VehVelocityExt;
#endif

    ved__ye_k.u_CurveFaultCounter = 0U;
    ved__ye_k.u_YawFaultCounter = 0U;
    ved__ye_k.u_DICurveFaultCounter = 0U;
#if ((defined(CFG_VED__REDUCE_CURVE_ERROR)) && (CFG_VED__REDUCE_CURVE_ERROR))
    u_count = 0U;
#endif
    b_RTBDetection = false;

    return;
}

/* ***********************************************************************
  @fn               VED_EnvGetVED_Config */ /*!

  @brief            Get Configuration of the ved_ component

  @description      Collects the configuration of the custom part and
                    does the MTS output of the configuration
                    Output is done only every 50th cycle to reduce MTS load

  @param[in]        reqPorts requested ports
  @param[in]        Services servicefunctions
  @param[out]       -
  @return           void

  @pre              -
  @post             -
  **************************************************************************** */
static void VED_GetConfig(void) {
    static uint8 nCounter = 0U;
    VED_Config_t VED_Config; /*!< @VNAME: VED_Config @VADDR: VED__MEAS_ID_CONFIG
                                @ALLOW: ved__priv @cycleid: ved__cycle_id*/

    if (nCounter < ((uint8)50U)) {
        nCounter++;
    } else {
        VED_Config.Version = VED__CUSTOM_VERSION_NUMBER;
        VED_Config.cfg_ved__yw_dyn_avg =
            CFG_VED__YW_DYN_AVG; /*! Enable dynamic gyro offst compensation */
        VED_Config.cfg_ved__ex_ywr_nvm =
            CFG_VED__EX_YWR_NVM; /*! Enable offset storage  in nonvolatile
                                    Memory */
        VED_Config.cfg_ved__int_gyro =
            CFG_VED__INT_GYRO; /*! Enable internal yaw rate sensor processing */
        VED_Config.cfg_ved__fpm_754 =
            CFG_VED__FPM_754; /*! Enable optimized math
                                 functio approximation  */
        VED_Config.cfg_ved__use_ex_long_accel =
            CFG_VED__USE_EX_LONG_ACCEL; /*! Use external provided longitudinal
                                           acceleration signal  */
        VED_Config.cfg_ved__use_ex_long_velo =
            CFG_VED__USE_EX_LONG_VELO; /*! Use external provided longitudinal
                                          velocity signal  */
        VED_Config.cfg_ved__mot_state =
            CFG_VED__MOT_STATE; /*!  Enable motion state processing */
        VED_Config.cfg_ved__do_velocity_corr =
            CFG_VED__DO_VELOCITY_CORR; /*!  Enables the velocity correction   */
        VED_Config.vel_corr_aln =
            VEL_CORR_ALN; /*!  Enables the velocity correction   */
        VED_Config.vel_corr_hist_stationary_targets =
            VEL_CORR_HIST_STATIONARY_TARGETS; /*!  Enables the velocity
                                                 correction   */

#if ((defined(CFG_VED__USE_CORRECT_VELO_CORR_VAR)) && \
     (CFG_VED__USE_CORRECT_VELO_CORR_VAR))
        VED_Config.cfg_ved__use_correct_velo_corr_var =
            CFG_VED__USE_CORRECT_VELO_CORR_VAR;
#else
        VED_Config.cfg_ved__use_correct_velo_corr_var = 0;
#endif

#if defined(CFG_VED__ROLLBENCH_DETECTION)
        VED_Config.cfg_ved__rollbench_detection =
            CFG_VED__ROLLBENCH_DETECTION; /*!  Enables the roll bench detection
                                           */
#else
        VED_Config.cfg_ved__rollbench_detection = 0U;
#endif

        VED_Config.cfg_ved__use_ext_proc_curvature =
            CFG_VED__USE_EXT_PROC_CURVATURE; /*! Enables usage of external curve
                                                as ved_ output curve  */
        VED_Config.cfg_ved__use_ext_proc_yaw_rate =
            CFG_VED__USE_EXT_PROC_YAW_RATE; /*! Enables usage of external yaw
                                               rate as ved_ output yaw rate  */
        VED_Config.cfg_ved__use_ext_proc_side_slip_angle =
            CFG_VED__USE_EXT_PROC_SIDE_SLIP_ANGLE; /*! Enables usage of external
                                                      side slip angle as ved_
                                                      ouput side slip angle  */

#if defined(CFG_VED__DIS_SWA_OFFSET_COMP)
        VED_Config.cfg_ved__dis_swa_offset_comp = CFG_VED__DIS_SWA_OFFSET_COMP;
#else
        VED_Config.cfg_ved__dis_swa_offset_comp = 0U;
#endif

#if defined(CFG_VED__DIS_YWR_OFFSET_COMP)
        VED_Config.cfg_ved__dis_ywr_offset_comp = CFG_VED__DIS_YWR_OFFSET_COMP;
#else
        VED_Config.cfg_ved__dis_ywr_offset_comp = 0U;
#endif

#if defined(CFG_VED__DIS_WHS_OFFSET_COMP)
        VED_Config.cfg_ved__dis_whs_offset_comp = CFG_VED__DIS_WHS_OFFSET_COMP;
#else
        VED_Config.cfg_ved__dis_whs_offset_comp = 0U;
#endif

#if defined(CFG_VED__DIS_LAT_OFFSET_COMP)
        VED_Config.cfg_ved__dis_lat_offset_comp = CFG_VED__DIS_LAT_OFFSET_COMP;
#else
        VED_Config.cfg_ved__dis_lat_offset_comp = 0U;
#endif

#if defined(CFG_VED__USE_EXT_PROC_UNDERSTEER_GRAD)
        VED_Config.cfg_ved__use_ext_proc_understeer_grad =
            CFG_VED__USE_EXT_PROC_UNDERSTEER_GRAD;
#else
        VED_Config.cfg_ved__use_ext_proc_understeer_grad = 0U;
#endif

        VED_Config.ved__use_learned_understeer_grad =
            VED__USE_LEARNED_UNDERSTEER_GRAD; /*! If the learned understeer
                                                 gradiend should be used */
        VED_Config.ved__use_est_wld_dep =
            VED__USE_EST_WLD_DEP; /*! If the estimated wheel load dep should be
                                     used */

#if defined(CFG_VED__USE_VELO_MONITOR)
        VED_Config.cfg_ved__use_velo_monitor = CFG_VED__USE_VELO_MONITOR;
#else
        VED_Config.cfg_ved__use_velo_monitor = 0U;
#endif

#if defined(CFG_VED__YWR_OFFSET_MONITOR)
        VED_Config.cfg_ved__ywr_offset_monitor = CFG_VED__YWR_OFFSET_MONITOR;
#else
        VED_Config.cfg_ved__ywr_offset_monitor = 0U;
#endif

#if defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)
        VED_Config.cfg_ved__dis_wheel_pre_processing =
            CFG_VED__DIS_WHEEL_PRE_PROCESSING;
#else
        VED_Config.cfg_ved__dis_wheel_pre_processing = 0U;
#endif

#if defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)
        VED_Config.cfg_ved__dis_yaw_sensor_pre_processing =
            CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING;
#else
        VED_Config.cfg_ved__dis_yaw_sensor_pre_processing = 0U;
#endif

#if defined(CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)
        VED_Config.cfg_ved__dis_yaw_sensor_offs_pre_filtering =
            CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING;
#else
        VED_Config.cfg_ved__dis_yaw_sensor_offs_pre_filtering = 0U;
#endif

#if defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)
        VED_Config.cfg_ved__dis_yaw_sensor_output =
            CFG_VED__DIS_YAW_SENSOR_OUTPUT;
#else
        VED_Config.cfg_ved__dis_yaw_sensor_output = 0U;
#endif

#if defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)
        VED_Config.cfg_ved__dis_lat_accel_sensor_pre_processing =
            CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING;
#else
        VED_Config.cfg_ved__dis_lat_accel_sensor_pre_processing = 0U;
#endif

#if defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)
        VED_Config.cfg_ved__dis_stw_angle_sensor_pre_processing =
            CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING;
#else
        VED_Config.cfg_ved__dis_stw_angle_sensor_pre_processing = 0U;
#endif

#if defined(CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION)
        VED_Config.cfg_ved__dis_side_slip_angle_estimation =
            CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION;
#else
        VED_Config.cfg_ved__dis_side_slip_angle_estimation = 0U;
#endif

#if defined(CFG_VED__GEN_VELOCITY_VARIANCE)
        VED_Config.cfg_ved__gen_velocity_variance =
            CFG_VED__GEN_VELOCITY_VARIANCE;
#else
        VED_Config.cfg_ved__gen_velocity_variance = 0U;
#endif

#if defined(CFG_VED__ALIGNMENT_OFFSET_MONITOR)
        VED_Config.cfg_ved__alignment_offset_monitor =
            CFG_VED__ALIGNMENT_OFFSET_MONITOR;
#else
        VED_Config.cfg_ved__alignment_offset_monitor = 0U;
#endif

        VED_Config.cfg_ved__dis_functional_safety_mon =
            CFG_VED__DIS_FUNCTIONAL_SAFETY_MON;
        VED_Config.cfg_ved__64bit_timestamp_interv =
            CFG_VED__64BIT_TIMESTAMP_INTERV;
        VED_Config.cfg_ved__dis_curve_output = 0u;

#if defined(CFG_VED__CALC_VED__TIMING)
        VED_Config.cfg_ved__calc_ved__timing =
            CFG_VED__CALC_VED__TIMING; /*! If the timing should be calculated */
#else
        VED_Config.cfg_ved__calc_ved__timing = 0U;
#endif

        VED_Config.cfg_ved__fs_velo_corr_mon = CFG_VED__FS_VELO_CORR_MON;
        VED_Config.cfg_ved__mon_output_peaks = CFG_VED__MON_OUTPUT_PEAKS;
        VED_Config.ved__profiling_enabled = 0u;
        VED_Config.cfg_ved__use_algocompstate = 0u;
        VED_Config.cfg_ved__set_dem_events = 0U;

        nCounter = 0U;
        MEAS_FREEZE_DIRECT(VED__MEAS_ID_CONFIG, &VED_Config,
                           sizeof(VED_Config));
    }
}

/* ***********************************************************************
  @fn               VED_LearnInitWithParams */ /*!

  @brief            Init learn values ssg and wld with parameter values

  @description      Read the NVM learn values selfsteeringgradient and
                    wheelloaddependencies, and update the used parameter values
                    If no NVM learn values are available, the coding parameter values
                    are stored in NVM

  @param[in]        -
  @param[in,out]    input learn values from NVM
  @param[in,out]    nvdata input parameter
                    updated learn values for NVM
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_LearnInitWithParams(const VED_InputData_t *input,
                                    VED_NvData_t *nvdata) {
    /* init learn self steering gradient with the vehicle parameter if the
     * parameter is available and the nvm not yet initialized */
    if (    (nvdata->Read.SlfstGrad.CalStatus == (sint8)0)
       /*&& (VED_GET_IO_STATE(VED_PAR_POS_SSG, input->Parameter.State) == VED_IO_STATE_VALID) */ )
  {
        nvdata->Read.SlfstGrad.SlfStGrad =
            input->Parameter.VED_Kf_SelfSteerGrad_nu;
        nvdata->Read.SlfstGrad.SlfStGradMax =
            (uint8)(ROUND_TO_UINT(VED__NVM_USTG_INV_SCALE *
                                  input->Parameter.VED_Kf_SelfSteerGrad_nu) &
                    0xFF);
        nvdata->Read.SlfstGrad.SlfStGradMin =
            (uint8)(ROUND_TO_UINT(VED__NVM_USTG_INV_SCALE *
                                  input->Parameter.VED_Kf_SelfSteerGrad_nu) &
                    0xFF);

        /* Set CalStatus to 1, Self steering gradient is initialized with
         * vehicle parameter */
        nvdata->Write.SlfstGrad.CalStatus = (sint8)1;
    } else {
        nvdata->Write.SlfstGrad.CalStatus = nvdata->Read.SlfstGrad.CalStatus;
        VED_SET_NVM_IO_STATE(VED_NVM_POS_SSG, VED_IO_STATE_INVALID,
                             &nvdata->Read.State);
        VED_SET_NVM_IO_STATE(VED_NVM_POS_SSG, VED_IO_STATE_INVALID,
                             &nvdata->Write.State);
    }

    /* init learn wheel load dependency with the vehicle parameter if the
     * parameter is available and the nvm not yet initialized */
    if (    (nvdata->Read.Wld.Wld_front_quality == (uint8)0U)
       && (input->Parameter.VED_Kf_WhlLoadDepFrontAxle_nu > 0.0F)
      /* && (VED_GET_IO_STATE(VED_PAR_POS_WHLDFR, input->Parameter.State) == VED_IO_STATE_VALID)*/  )
  {
        nvdata->Read.Wld.Wld_front =
            input->Parameter.VED_Kf_WhlLoadDepFrontAxle_nu;
        nvdata->Write.Wld.Wld_front = nvdata->Read.Wld.Wld_front;

        /* Set quality to 1, wheel load dependency is initialized with vehicle
         * parameter */
        nvdata->Write.Wld.Wld_front_quality = 1U;
    } else {
        nvdata->Write.Wld.Wld_front_quality =
            nvdata->Read.Wld.Wld_front_quality;
        VED_SET_NVM_IO_STATE(VED_NVM_POS_WLD, VED_IO_STATE_INVALID,
                             &nvdata->Read.State);
        VED_SET_NVM_IO_STATE(VED_NVM_POS_WLD, VED_IO_STATE_INVALID,
                             &nvdata->Write.State);
    }
}

/* ***********************************************************************
  @fn               VED_PreProcess */ /*!

  @brief            VED pre processing setup input and output states

  @description      Inits learn values ssg and wld
                    Sets the output states depending of input state and configuraton
                    and the yaw rate state depending on internal and external yaw rate
                    Sets output motion state if probability processing is disabled
 
  @param[in]        VED_NvData learn values from NVM
  @param[in]        input external input signals
  @param[in]        reqPorts requested ports
  @param[out]       proPorts provided ports
                    
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_PreProcess(const reqVEDPrtList_t *reqPorts,
                           const VED_InputData_t *input,
                           VED_NvData_t *VED_NvData,
                           const proVEDPrtList_t *proPorts) {
    uint32 iostatYwr;

    /* Convert cycle time to float value with unit sec */
    VED__SET_CYCLE_TIME(input->Frame.CycleTime);

    /* In case of Startup state reset first cycle done flag */
    if ((!VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_RUNNING,
                              input->Frame.CtrlMode)) &&
        (!VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                              input->Frame.CtrlMode))) {
        /* Clear flag to indicate that first cycle execution has not been
         * completed */
        VED__RESET_FIRST_CYCLE_DONE;
    }

    /* copy the autocode nvm data from the request and provide port to a local
     * buffer */
    if (reqPorts->pNVMRead->State == NVM_CLEARED) {
        (void)memset(&VED_NvData->Read.Wld, 0, sizeof(VEDNvWldCalc_t));
        (void)memset(&VED_NvData->Read.SlfstGrad, 0,
                     sizeof(VEDNvSlfStGradCalc_t));
        (void)memset(&VED_NvData->Write.Wld, 0, sizeof(VEDNvWldCalc_t));
        (void)memset(&VED_NvData->Write.SlfstGrad, 0,
                     sizeof(VEDNvSlfStGradCalc_t));
    } else {
#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
        /* check for valid values read from NVM */
        if ((reqPorts->pNVMRead->SlfstGrad.SlfStGrad <=
             MAXIMUM_SELF_STEERING_GRAD) &&
            (reqPorts->pNVMRead->SlfstGrad.CalStatus >= 0) &&
            (reqPorts->pNVMRead->SlfstGrad.CalStatus <= 1)) {
#endif
            VED_NvData->Read.SlfstGrad = reqPorts->pNVMRead->SlfstGrad;
#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
            /* check for valid values read from NVM */
        } else {
            (void)memset(&VED_NvData->Read.SlfstGrad, 0,
                         sizeof(VEDNvSlfStGradCalc_t));
        }
        /* check for valid values read from NVM */
        if ((reqPorts->pNVMRead->Wld.Wld_front >= MINIMUM_WHEEL_LOAD_DEP) &&
            (reqPorts->pNVMRead->Wld.Wld_front <= MAXIMUM_WHEEL_LOAD_DEP) &&
            (reqPorts->pNVMRead->Wld.Wld_front_quality >= 0) &&
            (reqPorts->pNVMRead->Wld.Wld_front_quality <= 1)) {
#endif
            VED_NvData->Read.Wld = reqPorts->pNVMRead->Wld;
#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
            /* check for valid values read from NVM */
        } else {
            (void)memset(&VED_NvData->Read.Wld, 0, sizeof(VEDNvWldCalc_t));
        }
#endif

        VED_NvData->Read.State = reqPorts->pNVMRead->State;
        VED_NvData->Write.Wld = proPorts->pNVMWrite->Wld;
        VED_NvData->Write.SlfstGrad = proPorts->pNVMWrite->SlfstGrad;
        VED_NvData->Write.State = proPorts->pNVMWrite->State;
    }

    /* Init learn values ssg and wld with parameter values */
    VED_LearnInitWithParams(input, VED_NvData);

    /* Set output state of velocity (for internal / external velocity) */
#if (CFG_VED__USE_EX_LONG_VELO)
    /* Feed in external velocity */
    VED_ModIf.LongMot.VehVelo = input->Signals.VehVelocityExt;

    if (VED_GET_IO_STATE(VED_SIN_POS_VEHVEL_EXT, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        VED_SET_IO_STATE(VED_SOUT_POS_VEL, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
    } else {
        VED_SET_IO_STATE(VED_SOUT_POS_VEL, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
    }
#else
    if ((VED_GET_IO_STATE(VED_SIN_POS_WVEL_FL, input->Signals.State) !=
         VED_IO_STATE_VALID) &&
        (VED_GET_IO_STATE(VED_SIN_POS_WVEL_FR, input->Signals.State) !=
         VED_IO_STATE_VALID) &&
        (VED_GET_IO_STATE(VED_SIN_POS_WVEL_RL, input->Signals.State) !=
         VED_IO_STATE_VALID) &&
        (VED_GET_IO_STATE(VED_SIN_POS_WVEL_RR, input->Signals.State) !=
         VED_IO_STATE_VALID)) {
        VED_SET_IO_STATE(VED_SOUT_POS_VEL, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
    } else {
        VED_SET_IO_STATE(VED_SOUT_POS_VEL, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
    }
#endif

    /* Set output state of acceleration (for internal / external acceleration)
     */
#if (CFG_VED__USE_EX_LONG_ACCEL)
    /* Feed in external acceleration */
    VED_ModIf.LongMot.VehAccel = input->Signals.VehLongAccelExt;
    VED_ModIf.LongMot.VehAccelVar = VED__PAR_ACCEL_EXT_UNC;

    if (VED_GET_IO_STATE(VED_SIN_POS_VEHACL_EXT, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        VED_SET_IO_STATE(VED_SOUT_POS_ACCEL, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
    } else {
        VED_SET_IO_STATE(VED_SOUT_POS_ACCEL, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
    }
#else
#if (CFG_VED__USE_EX_LONG_VELO)
    {
        /* External velocity without related acceleration is provided */
        VED_ModIf.LongMot.VehAccel = VED_DifferentiateCycleTime(
            VED_ModIf.LongMot.VehVelo, oldVelocityIn,
            VED_ModIf.LongMot.VehAccel, VED__PAR_VEL_DIFF_FT);
        oldVelocityIn = VED_ModIf.LongMot.VehVelo;
        VED_ModIf.LongMot.VehAccelVar = VED__PAR_ACCEL_EXT_UNC;
    }
#endif
    if (VED_GET_IO_STATE(VED_SOUT_POS_VEL,
                         proPorts->pVehicleDynamicSignals->State) ==
        VED_IO_STATE_VALID) {
        VED_SET_IO_STATE(VED_SOUT_POS_ACCEL, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
    } else {
        VED_SET_IO_STATE(VED_SOUT_POS_ACCEL, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
    }
#endif

    /* Set output motion state and copy the external motion state to the Module
     * interface if external motion state should be used */
#if (!CFG_VED__MOT_STATE)
    /* if the external direction is valid */
    if (VED_GET_IO_STATE(VED_SIN_POS_VMOT, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        if (input->Signals.VehLongMotStateExt !=
            (VehLongMotStateExt_t)VED_VEH_MOT_STATE_ROLLING) {
            VED_ModIf.LongMot.MotState.MotState = VED_LONG_MOT_STATE_STDST;
            VED_ModIf.LongMot.MotState.Confidence = (float32)100.F;
            /* Set input state to output state */
            VED_SET_IO_STATE(
                VED_SOUT_POS_MSTAT,
                VED_GET_IO_STATE(VED_SIN_POS_VMOT, input->Signals.State),
                proPorts->pVehicleDynamicSignals->State);
        } else {
            /* if the external direction is valid */
            if (VED_GET_IO_STATE(VED_SIN_POS_VDIR, input->Signals.State) ==
                VED_IO_STATE_VALID) {
                switch (input->Signals.VehLongDirExt) {
                    case VED_LONG_DIR_FWD:
                        VED_ModIf.LongMot.MotState.MotState =
                            VED_LONG_MOT_STATE_MOVE_FWD;
                        VED_ModIf.LongMot.MotState.Confidence = (float32)100.F;
                        break;

                    case VED_LONG_DIR_RWD:
                        VED_ModIf.LongMot.MotState.MotState =
                            VED_LONG_MOT_STATE_MOVE_RWD;
                        VED_ModIf.LongMot.MotState.Confidence = (float32)100.F;
                        break;

                    case VED_LONG_DIR_VOID: /* fallthrough to default */
                    default:
                        VED_ModIf.LongMot.MotState.MotState =
                            VED_LONG_MOT_STATE_MOVE;
                        VED_ModIf.LongMot.MotState.Confidence = (float32)50.F;
                        break;
                }
                /* Set input state to output state */
                VED_SET_IO_STATE(
                    VED_SOUT_POS_MSTAT,
                    VED_GET_IO_STATE(VED_SIN_POS_VDIR, input->Signals.State),
                    proPorts->pVehicleDynamicSignals->State);
            } else {
                VED_ModIf.LongMot.MotState.MotState = VED_LONG_MOT_STATE_MOVE;
                VED_ModIf.LongMot.MotState.Confidence = (float32)50.F;
                /* Set input state to output state */
                VED_SET_IO_STATE(
                    VED_SOUT_POS_MSTAT,
                    VED_GET_IO_STATE(VED_SIN_POS_VMOT, input->Signals.State),
                    proPorts->pVehicleDynamicSignals->State);
            }
        }
    } else {
        VED_ModIf.LongMot.MotState.MotState = VED_LONG_MOT_STATE_MOVE;
        VED_ModIf.LongMot.MotState.Confidence = (float32)50.F;
        VED_SET_IO_STATE(VED_SOUT_POS_MSTAT, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
    }
#else
    VED_SET_IO_STATE(VED_SOUT_POS_MSTAT, VED_IO_STATE_VALID,
                     proPorts->pVehicleDynamicSignals->State);
#endif

    /* Set temp iostatYwr to internal or external yaw rate sensor state */
#if (CFG_VED__INT_GYRO)
    /* if external yaw rate signal is valid use the external yaw rate signal
     * otherwise the internal */
    if (VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        iostatYwr = VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State);
    } else {
        iostatYwr = VED_GET_IO_STATE(VED_SIN_POS_YWRINT, input->Signals.State);
    }
#else
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
    iostatYwr = VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State);
#else
    iostatYwr = VED_IO_STATE_VALID;
#endif
#endif

    /* Set output curvature to input curvature state or to state combination of
     * swa, velocity and external input yaw rate state */
#if (CFG_VED__USE_EXT_PROC_CURVATURE)
    VED_ModIf.Curve.Curve = input->Signals.CurveC0Ext;
    VED_ModIf.Curve.C1 = input->Signals.CurveC1Ext;
    VED_ModIf.Curve.varC0 = VED__PAR_CRV_EXT_UNC;
    VED_ModIf.Curve.varC1 = VED__PAR_CRV_EXT_UNC;

    if (VED_GET_IO_STATE(VED_SIN_POS_CRV, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        VED_SET_IO_STATE(VED_SOUT_POS_CURVE, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
    } else {
        VED_SET_IO_STATE(VED_SOUT_POS_CURVE, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
    }
#else
    if (
    /* If steering wheel angle sensor pre processing is enabled */
#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
        ((VED_GET_IO_STATE(VED_SIN_POS_SWA, input->Signals.State) ==
          VED_IO_STATE_INVALID) ||
         (VED_GET_IO_STATE(VED_SIN_POS_SWA, input->Signals.State) ==
          VED_IO_STATE_NOTAVAIL)) &&
#endif
        ((VED_GET_IO_STATE(VED_SOUT_POS_VEL,
                           proPorts->pVehicleDynamicSignals->State) ==
          VED_IO_STATE_INVALID) ||
         (iostatYwr != VED_IO_STATE_VALID))) {
        VED_SET_IO_STATE(VED_SOUT_POS_CURVE, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
    } else {
        VED_SET_IO_STATE(VED_SOUT_POS_CURVE, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
    }
#endif

    /* Set output yaw rate to input yaw rate state or to state combination of
     * swa, velocity and external input yaw rate state */
#if (CFG_VED__USE_EXT_PROC_YAW_RATE)
    VED_ModIf.YawRate.YawRate = input->Signals.YawRate;
    VED_ModIf.YawRate.Variance = VED__PAR_YWR_EXT_UNC;

    if (VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        VED_SET_IO_STATE(VED_SOUT_POS_YWR, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
        proPorts->pALNYawRate->bSenYawRateState = TRUE;
#endif
    } else {
        VED_SET_IO_STATE(VED_SOUT_POS_YWR, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
        proPorts->pALNYawRate->bSenYawRateState = FALSE;
#endif
    }
#else
    if (((iostatYwr != VED_IO_STATE_VALID) &&
         (iostatYwr != VED_IO_STATE_DECREASED)) &&
        ((VED_GET_IO_STATE(VED_SIN_POS_LATA, input->Signals.State) !=
          VED_IO_STATE_VALID) &&
         (VED_GET_IO_STATE(VED_SIN_POS_LATA, input->Signals.State) !=
          VED_IO_STATE_DECREASED))) {
        VED_SET_IO_STATE(VED_SOUT_POS_YWR, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
        proPorts->pALNYawRate->bSenYawRateState = FALSE;
#endif
    } else {
        VED_SET_IO_STATE(VED_SOUT_POS_YWR, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
        proPorts->pALNYawRate->bSenYawRateState = TRUE;
#endif
    }
#endif

    /* Set output state of side slip angle if side slip angle estimation is not
     * disabled */
#if ((!defined(CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION)) || \
     (!CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION))
#if (CFG_VED__USE_EXT_PROC_SIDE_SLIP_ANGLE)

    VED_ModIf.SideSlipAngle.SideSlipAngle = input->Signals.SideSlipAngleExt;
    VED_ModIf.SideSlipAngle.Variance = VED__PAR_SSA_EXT_UNC;

    /* Set output state of slip angle depending on slip angle input state */
    VED_SET_IO_STATE(VED_SOUT_POS_SSA,
                     VED_GET_IO_STATE(VED_SIN_POS_SSA, input->Signals.State),
                     proPorts->pVehicleDynamicSignals->State);

#else /* CFG_VED__USE_EXT_PROC_SIDE_SLIP_ANGLE */
    /* set output state of slip angle depending on inputs */
    if ((iostatYwr != VED_IO_STATE_VALID) ||
        (VED_GET_IO_STATE(VED_SOUT_POS_VEL, input->Signals.State) !=
         VED_IO_STATE_VALID) ||
        (VED_GET_IO_STATE(VED_SIN_POS_LATA, input->Signals.State) !=
         VED_IO_STATE_VALID)) {
        /* if yawrate, velocity or lataccel are invalid, output SideSlipAngle is
         * also invalid */
        VED_SET_IO_STATE(VED_SOUT_POS_SSA, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
    } else {
        /* if yawrate, velocity and lataccel are valid, output SideSlipAngle is
         * also valid */
        VED_SET_IO_STATE(VED_SOUT_POS_SSA, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
    }
#endif
#else /* CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION */
    VED_SET_IO_STATE(VED_SOUT_POS_SSA, VED_IO_STATE_INVALID,
                     proPorts->pVehicleDynamicSignals->State);
#endif

    /* set output state of lateral acceleration depending on yaw rate and
     * lateral acceleration */
    if ((iostatYwr != VED_IO_STATE_VALID) &&
        (VED_GET_IO_STATE(VED_SIN_POS_LATA, input->Signals.State) !=
         VED_IO_STATE_VALID)) {
        /* if input yawrate and lataccel are invalid, output Lataccel is also
         * invalid */
        VED_SET_IO_STATE(VED_SOUT_POS_LATACC, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
    } else {
        VED_SET_IO_STATE(VED_SOUT_POS_LATACC, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
    }

    /* Set output state of driver intended curvature depending on swa input
     * state */
    VED_SET_IO_STATE(VED_SOUT_POS_DRCRV,
                     VED_GET_IO_STATE(VED_SIN_POS_SWA, input->Signals.State),
                     proPorts->pVehicleDynamicSignals->State);
}

/* ***********************************************************************
  @fn               VED_HandcodeProcess */ /*!

  @brief            VED processing of handcode modules

  @description      Executing VED handcode modules in the order:
                    - internal gyro processing
                    - velocity correction
                    - stand still yaw rate offset processing
                    - steering angle and lateral acceleration offset processing
                    - wheel velocity offset (left-right at one axle) processing

  @param[in]        reqPorts requested ports
  @param[in]        input input signals
  @param[in]        Services
  @param[out]       proPorts provided ports
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_HandcodeProcess(const reqVEDPrtList_t *reqPorts,
                                const VED_InputData_t *input,
                                const proVEDPrtList_t *proPorts) {
#if ((!CFG_VED__INT_GYRO) && (!CFG_VED__DO_VELOCITY_CORR) && \
     (CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING) &&             \
     (CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING) &&       \
     (CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    (void)input; /* remove compiler warning, input is not used in certain
                    configurations */
#endif

    /* Execute internal gyro processing */
#if (CFG_VED__INT_GYRO)
    VED_YwrtExec(reqPorts, &VED_ModIf, proPorts);
#endif

    /* Velocity correction using velocity of stationary targets */
#if (CFG_VED__DO_VELOCITY_CORR)

    /* Execute velocity correction  */
    VED_VelCorrExec(reqPorts, input, &VED_ModIf, proPorts, b_RTBDetection);

    if (VED_GET_IO_STATE(VED_SOUT_POS_VEL,
                         proPorts->pVehicleDynamicSignals->State) ==
        VED_IO_STATE_VALID) {
        /* Validate ouput data */

        VED_SET_IO_STATE(VED_SOUT_POS_VCORR, VED_IO_STATE_VALID,
                         proPorts->pVehicleDynamicSignals->State);
    } else {
        /* Invalidate output data */
        VED_SET_IO_STATE(VED_SOUT_POS_VCORR, VED_IO_STATE_INVALID,
                         proPorts->pVehicleDynamicSignals->State);
    }

#else
    /* If no correction is active set default value */
    VED_ModIf.LongMot.VelCorrFact = 1.0F;

    /* Invalidate output data */
    VED_SET_IO_STATE(VED_SOUT_POS_VCORR, VED_IO_STATE_NOTAVAIL,
                     proPorts->pVehicleDynamicSignals->State);
#endif

    /* stand still yaw rate offset processing */
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
    /* Execute yaw rate sensor signal processing stand still offset e.g.*/
    VED_YwrExec(reqPorts, input, &VED_ModIf, proPorts);
#endif

    /* Monitoring of stand still yaw rate offset, like in the brake systems */
#if (CFG_VED__YWR_OFFSET_MONITOR)
    /* Execute yaw rate sensor offset monitoring */
    VED_YwrMonExec(input, &VED_ModIf, proPorts);
#endif

    /* Steering angle and lateral acceleration offset processing */
#if (((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
      (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) ||         \
     ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
      (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)))
    /* Execute steering wheel angle sensor signal processing */
    VED_AySwaExec(reqPorts, input, &VED_ModIf, proPorts);
#endif

    /* Wheel velocity offset (left-right at one axle) processing */
#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* Execute wheel speed sensors signal processing */
    VED_WhsExec(input, &VED_ModIf);
#endif
}

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
/* ***********************************************************************
  @fn               VED_ProcAutoVelocity */ /*!

  @brief            VED processing of velocity estimation autocode

  @description      Initialises wpp and ve module in INIT
                    Calls wheel speed preprocessing module wpp
                    Calls velocity and acceleration calculation module ve

  @param[in]        input signals
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_ProcAutoVelocity(VED_InputData_t *input) {
    /* if ved_ control mode is init  */
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
        /* Initialize ved__wpp model */
        ved__wpp_initialize((boolean_T)1, &ved__wpp_M, &ved__wpp_B,
                            &ved__wpp_DWork);
        /* Initialize ved__ve model */
        ved__ve_initialize((boolean_T)1, &ved__ve_M, &ved__ve_B,
                           &ved__ve_DWork);
    } else {
        /* if ved__wpp_module is sequence initalized execute it */
        if (VED__IS_MODULE_SEQU_INITALIZED(
                sSequenceInitState.initStates.ved__wpp_sequ_init) == TRUE) {
            /* execute wheel pre processing module */
            ved__wpp_step(&ved__wpp_B, &ved__wpp_DWork, input,
                          &ved__internal_data, &ved__internal_data);
        }
        /* if ved__ve_module is sequence initalized execute it */
        if (VED__IS_MODULE_SEQU_INITALIZED(
                sSequenceInitState.initStates.ved__ve_sequ_init) == TRUE) {
            /* execute vehicle velocity and acceleration estimation modul */
            ved__ve_step(&ved__ve_B, &ved__ve_DWork, (VED_InputData_t *)input,
                         &ved__internal_data, &ved__internal_data);
        }
    }
}
#endif

/* ***********************************************************************
  @fn               VED_ProcAutoWheelYawRate */ /*!

  @brief            VED processing of wheel velocity yaw rate estimation autocode

  @description      Initialises wye module in INIT
                    Calls wheel yaw rate module wye

  @param[in]        input signals
  @param[in]        nvdata parameter
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_ProcAutoWheelYawRate(VED_InputData_t *input,
                                     VED_NvData_t *nvdata) {
/* if wheel velocity yaw rate preprocessing is enabled */
#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* if ved_ ctrl mode is init do not delete learn values but init the rest*/
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
        /* local copy of last cycle data */
        uint8 init_nvm_wld_delay_DSTATE =
            ved__wye_DWork.init_nvm_wld_delay_DSTATE;
        float32 x_delay_wld_DSTATE = ved__wye_DWork.x_delay_wld_DSTATE[2];
        float32 last_est_wld_DSTATE = ved__wye_DWork.last_est_wld_DSTATE;

        float32 x_delay_dyn_off_DSTATE =
            ved__wye_DWork.x_delay_dyn_off_DSTATE[2];
        sint8 last_dyn_yaw_offset_sign_DSTATE =
            ved__wye_DWork.last_dyn_yaw_offset_sign_DSTATE;
        float32 dyn_yaw_off_overt_count_DSTATE =
            ved__wye_DWork.dyn_yaw_off_overt_count_DSTATE;
        float32 last_dyn_yaw_offset_DSTATE =
            ved__wye_DWork.last_dyn_yaw_offset_DSTATE;

        /* Initialize ved__wye model */
        ved__wye_initialize((boolean_T)1, &ved__wye_M, &ved__wye_B,
                            &ved__wye_DWork);
        /* copy all necessary data and old learn values to reinitialize the auto
         * code module */
        ved__wye_DWork.init_nvm_wld_delay_DSTATE = init_nvm_wld_delay_DSTATE;
        ved__wye_DWork.x_delay_wld_DSTATE[2] = x_delay_wld_DSTATE;
        ved__wye_DWork.last_est_wld_DSTATE = last_est_wld_DSTATE;

        ved__wye_DWork.x_delay_dyn_off_DSTATE[2] = x_delay_dyn_off_DSTATE;
        ved__wye_DWork.last_dyn_yaw_offset_sign_DSTATE =
            last_dyn_yaw_offset_sign_DSTATE;
        ved__wye_DWork.dyn_yaw_off_overt_count_DSTATE =
            dyn_yaw_off_overt_count_DSTATE;
        ved__wye_DWork.last_dyn_yaw_offset_DSTATE = last_dyn_yaw_offset_DSTATE;
    } else {
        /* if ved__wye_module is sequence initalized execute it */
        if (VED__IS_MODULE_SEQU_INITALIZED(
                sSequenceInitState.initStates.ved__wye_sequ_init) == TRUE) {
            /* at running mode */
            ved__wye_step(&ved__wye_B, &ved__wye_DWork, input,
                          &ved__internal_data, (VED_NvData_t *)nvdata,
                          &ved__internal_data, (VED_NvData_t *)nvdata);
        }
    }
#else
    /* if the wheel pre processing module is not used (wheel velocities are not
     * available) */
    /* remove compiler warning, parameters are not used in this configuration */
    (void)input;
    (void)nvdata;

    /* set the wheel yaw rate modul to invalid variances */
    ved__internal_data.ved__wye_out.whl_yaw_rate = 0.0F;
    ved__internal_data.ved__wye_out.whl_yaw_rate_var = 100.0F;
    ved__internal_data.ved__wye_out.gier_yaw_rate_offset =
        ved__internal_data.ved__offsets_in.ved__yaw_offset.offset;
    ved__internal_data.ved__wye_out.gier_yaw_rate_offset_var =
        ved__internal_data.ved__offsets_in.ved__yaw_offset.var;
    ved__internal_data.ved__wye_out.raw_est_yaw_offset = 0.0F;
#endif
}

#if (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)
#if (CFG_VED__INT_GYRO)
/* ***********************************************************************
  @fn               VED_ProcAutoSenIntYawRateOffset */ /*!

  @brief            VED processing of sensor internal yaw rate offset

  @description      Dynamic yaw rate handling if internal yaw rate sensor is used

  @param[in]        -
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_ProcAutoSenIntYawRateOffset(void) {
    const VED_YwrtOffsData_t *pYwrOffs = VED_YwrtGetOffsData();

    /* reset dyn yaw rate offset filter if stand still */
    if (((pYwrOffs->OffsElpsdTime < (float32)1.0F) &&
         (pYwrOffs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST)) ||
        (pYwrOffs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST_EEPROM) ||
        (VED__IS_FIRST_CYCLE_DONE == FALSE)) {
        OldFiltYawOffset = ved__internal_data.ved__wye_out.gier_yaw_rate_offset;
    } else {
        OldFiltYawOffset =
            ved__internal_data.ved__gye_out_filt.raw_est_yaw_offset_filt;
    }
    /* low pass filter raw yaw offset */
    ved__internal_data.ved__gye_out_filt.raw_est_yaw_offset_filt =
        VED_FilterCycleTime(
            ved__internal_data.ved__wye_out.gier_yaw_rate_offset,
            OldFiltYawOffset, VED__PAR_RAW_YAW_RATE_OFFSET_FT);
}
#endif

/* ***********************************************************************
  @fn               VED_ProcAutoSenExtYawRateOffset */ /*!

  @brief            VED processing of sensor internal yaw rate offset

  @description      Dynamic yaw rate handling if external yaw rate sensor is used

  @param[in]        -
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_ProcAutoSenExtYawRateOffset(void) {
    const VED_YwrOffsData_t *pYwrOffs = VED_YwrGetOffsData();

    /* reset dyn yaw rate offset filter if stand still */
    if (((pYwrOffs->OffsElpsdTime < (float32)1.0F) &&
         (pYwrOffs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST)) ||
        (pYwrOffs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST_EEPROM)) {
        OldFiltYawOffset = ved__internal_data.ved__wye_out.raw_est_yaw_offset;
    } else {
        OldFiltYawOffset =
            ved__internal_data.ved__gye_out_filt.raw_est_yaw_offset_filt;
    }
    /* low pass filter raw yaw offset */
    ved__internal_data.ved__gye_out_filt.raw_est_yaw_offset_filt =
        VED_FilterCycleTime(ved__internal_data.ved__wye_out.raw_est_yaw_offset,
                            OldFiltYawOffset, VED__PAR_RAW_YAW_RATE_OFFSET_FT);
}
#endif

/* ***********************************************************************
  @fn               VED_ProcAutoSenYawRate */ /*!

  @brief            VED processing of sensor yaw rate estimation autocode

  @description      Initialises gye module in INIT
                    Calls gyro yaw rate module gye

  @param[in]        input signals
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_ProcAutoSenYawRate(VED_InputData_t *input) {
/* if sensor yaw rate pre processing is enabled*/
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
    /* if ved_ control mode is init */
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
#if (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)
        /* Initialize ved__gye model filt */
        ved__gye_initialize((boolean_T)1, &ved__gye_M_FILT, &ved__gye_B_FILT,
                            &ved__gye_DWork_FILT);
#endif
        /* do nothing */

        /* Initialize ved__gye model */
        ved__gye_initialize((boolean_T)1, &ved__gye_M, &ved__gye_B,
                            &ved__gye_DWork);
    } else {
        /* if ved__gye_module is sequence initalized execute it */
        if (VED__IS_MODULE_SEQU_INITALIZED(
                sSequenceInitState.initStates.ved__gye_sequ_init) == TRUE) {
#if (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)
            /* if sensor yaw rate offset should be prefilterd for alignment */

            /* store not filterd yaw rate offset */
            float32 NotFiltYawOffset =
                ved__internal_data.ved__wye_out.gier_yaw_rate_offset;

            /*<----- Pass yaw rate offset  ----->*/
#if (CFG_VED__INT_GYRO)
            /* if external yaw rate signal is valid use the external yaw rate
             * signal otherwise the internal */

            if (VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State) ==
                VED_IO_STATE_VALID) {
                /* reset and low pass filter external yaw rate dynamic offset */
                VED_ProcAutoSenExtYawRateOffset();
            } else {
                /* reset and low pass filter internal yaw rate dynamic offset */
                VED_ProcAutoSenIntYawRateOffset();
            }
#else
            /* reset and low pass filter external yaw rate dynamic offset */
            VED_ProcAutoSenExtYawRateOffset();
#endif

            /* copy filterd yaw rate offset to gier_yaw_rate_offset as input for
             * alignment gier sensor yaw rate*/
            ved__internal_data.ved__wye_out.gier_yaw_rate_offset =
                ved__internal_data.ved__gye_out_filt.raw_est_yaw_offset_filt;

            /* calculate gier yaw rate with filtered raw yaw rate offset */
            ved__gye_step(&ved__gye_B_FILT, &ved__gye_DWork_FILT, input,
                          &ved__internal_data, &ved__internal_data);

            /* copy sensor yaw rate with filtered raw estimated gier yaw rate
             * offset to seperate struct */
            ved__internal_data.ved__gye_out_filt.gier_yaw_rate =
                ved__internal_data.ved__gye_out.gier_yaw_rate;
            ved__internal_data.ved__gye_out_filt.gier_yaw_rate_var =
                ved__internal_data.ved__gye_out.gier_yaw_rate_var;

            /* restore original not filtered yaw rate offset*/
            ved__internal_data.ved__wye_out.gier_yaw_rate_offset =
                NotFiltYawOffset;
#else
            /* set filtered offset to not filtered raw offset */
            ved__internal_data.ved__gye_out_filt.raw_est_yaw_offset_filt =
                ved__internal_data.ved__wye_out.raw_est_yaw_offset;
#endif

            /* calculate gier yaw rate with discrete yaw rate offset */
            ved__gye_step(&ved__gye_B, &ved__gye_DWork,
                          (VED_InputData_t *)input, &ved__internal_data,
                          &ved__internal_data);
        }

#if (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)
#else
        /* copy sensor yaw rate with NOT filtered raw estimated gier yaw rate
         * offset to seperate struct */
        ved__internal_data.ved__gye_out_filt.gier_yaw_rate =
            ved__internal_data.ved__gye_out.gier_yaw_rate;
        ved__internal_data.ved__gye_out_filt.gier_yaw_rate_var =
            ved__internal_data.ved__gye_out.gier_yaw_rate_var;
#endif
    }
#else
    (void)input; /* remove compiler warning, input is not used in this
                    configuration */

    /* set internal sensor yaw rate data to default values */
    ved__internal_data.ved__gye_out.gier_yaw_rate = 0.0F;
    ved__internal_data.ved__gye_out.gier_yaw_rate_var = 100.0F;
#endif
}

/* ***********************************************************************
  @fn               VED_ProcAutoLatAccelYawRate */ /*!

  @brief            VED processing of lateral acceleration yaw rate estimation autocode

  @description      Initialises aye module in INIT
                    Calls lat accel yaw rate module gye

  @param[in]        input signals
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_ProcAutoLatAccelYawRate(VED_InputData_t *input) {
/* If lateral acceleration sensor pre processing is enabled */
#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
    /* if ved_ control mode is init */
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
        /* Initialize ved__ay model */
        ved__aye_initialize((boolean_T)1, &ved__aye_M, &ved__aye_DWork);
    } else {
        /* if ved__aye_module is sequence initalized execute it */
        if (VED__IS_MODULE_SEQU_INITALIZED(
                sSequenceInitState.initStates.ved__aye_sequ_init) == TRUE) {
            ved__aye_step(&ved__aye_DWork, input, &ved__internal_data,
                          &ved__internal_data);
        }
    }
#else
    (void)input; /* remove compiler warning, input is not used in this
                    configuration */

    /* set internal lateral accel yaw rate data to default values */
    ved__internal_data.ved__aye_out.ay_yaw_rate = 0.0F;
    ved__internal_data.ved__aye_out.ay_yaw_rate_var = 100.0F;
#endif
}

/* ***********************************************************************
  @fn               VED_ProcAutoSwaYawRate */ /*!

  @brief            VED processing of steering wheel angle yaw rate estimation autocode

  @description      Initialises sye module in INIT
                    Calls steering wheel yaw rate module sye

  @param[in]        input signals
  @param[in]        nvdata learn values/parameters
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_ProcAutoSwaYawRate(VED_InputData_t *input,
                                   VED_NvData_t *nvdata) {
/* If steering wheel angle sensor pre processing is enabled */
#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
    /* if ved_ control mode is init */
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
        /* local copy of last cycle data */
        uint8 init_nvm_eg_delay_DSTATE =
            ved__sye_DWork.init_nvm_eg_delay_DSTATE;
        float32 x_delay_eg_DSTATE = ved__sye_DWork.x_delay_eg_DSTATE[2];
        /* Initialize ved__sye model */
        ved__sye_initialize((boolean_T)1, &ved__sye_M, &ved__sye_B,
                            &ved__sye_DWork);
        /* copy all necessary data and old learn values to reinitialize the auto
         * code module */
        ved__sye_DWork.init_nvm_eg_delay_DSTATE = init_nvm_eg_delay_DSTATE;
        ved__sye_DWork.x_delay_eg_DSTATE[2] = x_delay_eg_DSTATE;
    } else {
        /* if ved__sye_module is sequence initalized execute it */
        if (VED__IS_MODULE_SEQU_INITALIZED(
                sSequenceInitState.initStates.ved__sye_sequ_init) == TRUE) {
            ved__sye_step(&ved__sye_B, &ved__sye_DWork, input,
                          &ved__internal_data, (VED_NvData_t *)nvdata,
                          &ved__internal_data, (VED_NvData_t *)nvdata);
        }
    }
#else
    /* remove compiler warning, parameters are not used in this configuration */
    (void)nvdata;
    (void)input;

    /* set internal steering wheel angle yaw rate data to default values */
    ved__internal_data.ved__sye_out.stw_yaw_rate = 0.0F;
    ved__internal_data.ved__sye_out.stw_yaw_rate_var = 100.0F;

    ved__internal_data.ved__sye_out.stw_curve = 0.0F;
    ved__internal_data.ved__sye_out.stw_curve_var = 100.0F;
#endif
}

/* ***********************************************************************
  @fn               VED_ProcAutoVehicleYawRate */ /*!

  @brief            VED processing of vehicle angle yaw rate fusion estimation autocode

  @description      Initialises ye module in INIT
                    Calls yaw rate fusion module ye

  @param[in]        input signals
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_ProcAutoVehicleYawRate(VED_InputData_t *input) {
    /* If external yaw rate sensor processing  */
#if ((!defined(CFG_VED__USE_EXT_PROC_YAW_RATE)) || \
     (!CFG_VED__USE_EXT_PROC_YAW_RATE))
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
        /* Initialize ved__ve model */
        ved__ye_initialize((boolean_T)1, &ved__ye_M, &ved__ye_B,
                           &ved__ye_DWork);
    } else {
        /* if ved__ye_module is sequence initalized execute it */
        if (VED__IS_MODULE_SEQU_INITALIZED(
                sSequenceInitState.initStates.ved__ye_sequ_init) == TRUE) {
            ved__ye_step(&ved__ye_B, &ved__ye_DWork, input, &ved__internal_data,
                         &ved__internal_data, ved__ye_k.K_yaw,
                         &ved__ye_k.K_yaw_fault, ved__ye_k.K_curve,
                         &ved__ye_k.K_curve_fault);

            if ((ved__ye_k.K_yaw_fault == 1U) ||
                (ved__ye_k.K_curve_fault == 1U)) {
                /* Initialize ved__ye model */
                ved__ye_initialize((boolean_T)1, &ved__ye_M, &ved__ye_B,
                                   &ved__ye_DWork);
            }
        }
    }
#else
    /* calculate the curve manually, is not filtered !!!*/
    if (TUE_CML_IsZero(ved__internal_data.ved__ve_out.veh_velo)) {
        ved__internal_data.ved__ye_out.veh_merge_curve = 0.0F;
    } else {
        ved__internal_data.ved__ye_out.veh_merge_curve =
            input->Signals.YawRate / ved__internal_data.ved__ve_out.veh_velo;
    }
    ved__internal_data.ved__ye_out.veh_merge_curve_grad = 0.0F;
    ved__internal_data.ved__ye_out.veh_merge_curve_var = 0.0F;
#endif
}

/* ***********************************************************************
  @fn               VED_ProcAutoSideSlip */ /*!

  @brief            VED processing of side slip angle estimation autocode

  @description      Initialises sae module in INIT
                    Calls side slip angle module sae

  @param[in]        input signals
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_ProcAutoSideSlip(VED_InputData_t *input) {
/* If side slip angle processing is enabled */
#if ((!defined(CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION)) || \
     (!CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION))
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
        /* Initialize ved__sae model */
        ved__sae_initialize((boolean_T)1, &ved__sae_M, &ved__sae_DWork);
    } else {
        /* if ved__sae_module is sequence initalized execute it */
        if (VED__IS_MODULE_SEQU_INITALIZED(
                sSequenceInitState.initStates.ved__sae_sequ_init) == TRUE) {
            ved__sae_step(&ved__sae_DWork, (VED_InputData_t *)input,
                          &ved__internal_data, &ved__internal_data);
        }
    }
#else
    (void)input; /* remove compiler warning, input is not used in this
                    configuration */

    /* set internal side slip angle data to default values */
    ved__internal_data.ved__sae_out.est_slip_angle = 0.0F;
    ved__internal_data.ved__sae_out.est_slip_angle_var = 100.0F;
#endif
}

#if (CFG_VED__MOT_STATE)
/* ***********************************************************************
  @fn               VED_ProcMotionState */ /*!

  @brief            VED processing of motion state estimation autocode

  @description      Initialises mot_st module in INIT
                    Calls motion state determination module mot_st

  @param[in]        input signals
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_ProcMotionState(VED_InputData_t *input,
                                VEDALN_Monitoring_t *ALNMonitoring) {
    /* If motion state processing is enabled */
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
        /* Initialize ved__mot_st model */
        ved__mot_st_initialize((boolean_T)1, &ved__mot_st_M, &ved__mot_st_B,
                               &ved__mot_st_DWork);
    } else {
        /* if ved__mot_state_module is sequence initalized execute it */
        if (VED__IS_MODULE_SEQU_INITALIZED(
                sSequenceInitState.initStates.ved__mot_state_sequ_init) ==
            TRUE) {
            ved__mot_st_step(&ved__mot_st_B, &ved__mot_st_DWork, input,
                             &ved__internal_data, ALNMonitoring,
                             &ved__bayes_mot_states);
            ved__internal_data.ved__mot_st_out.fwd =
                ved__bayes_mot_states.mot_st_out.fwd;
            ved__internal_data.ved__mot_st_out.ss =
                ved__bayes_mot_states.mot_st_out.ss;
            ved__internal_data.ved__mot_st_out.rvs =
                ved__bayes_mot_states.mot_st_out.rvs;
            ved__internal_data.ved__mot_st_out.mot_state =
                ved__bayes_mot_states.mot_st_out.mot_state;
            ved__internal_data.ved__mot_st_out.mot_quality =
                ved__bayes_mot_states.mot_st_out.mot_quality;
            ved__internal_data.ved__mot_st_out.mot_counter =
                ved__bayes_mot_states.mot_st_out.mot_counter;
        }
    }
}
#endif

/* ***********************************************************************
  @fn               VED_ProcessAutocode */ /*!

  @brief            VED processing of autocode modules

  @description      Executing VED autocode modules in the order:
                    - velocity calculation
                    - velocity monitoring
                    - switches between internal or external (corrected) velocity
                    - wheel speed yaw rate processing
                    - gyro yaw rate processing
                    - lateral acceleration yaw rate processing
                    - steering wheel yaw rate processing
                    - yaw rate fusion
                    - slip angle calculation
                    - motion state calculation

  @param[in]        reqPorts requested ports
  @param[in]        input input signals
  @param[in]        Services
  @param[out]       nvdata provided ports
  @param[out]       VED_Errors fault list
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_AutocodeProcess(const reqVEDPrtList_t *reqPorts,
                                VED_InputData_t *input,
                                VED_NvData_t *nvdata,
                                VED_Errors_t *VED_Errors) {
    (void)reqPorts; /* remove compiler warning, reqPorts is not used in this
                       configuration */
                    /* Process velocity estimation */
#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    VED_ProcAutoVelocity(input);
#endif

    /* Velocity monitoring (external <-> internal velocity)*/
#if (CFG_VED__USE_VELO_MONITOR)
    VED_VelMonExec(input, &ved__internal_data.ved__ve_out, VED_Errors);
#else
    (void)VED_Errors; /* remove compiler warning, VED_Errors is not used in this
                         configuration */
#endif

    /* overwrite internal velocity data if external velocity should be used and
     * is valid */
#if (CFG_VED__USE_EX_LONG_VELO)
    if (VED_GET_IO_STATE(VED_SIN_POS_VEHVEL_EXT, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        ved__internal_data.ved__ve_out.veh_velo = VED_ModIf.LongMot.VehVelo;
    }
#else
    /* keep the uncorrected velocity */
    VED_ModIf.LongMot.VehVelo = ved__internal_data.ved__ve_out.veh_velo;
#endif
    /* set the internal autocode velocity to the corrected velocity */
    ved__internal_data.ved__ve_out.veh_velo *= VED_ModIf.LongMot.VelCorrFact;

    /* Process velocity during roll bench detection*/
    if (b_RTBDetection == TRUE) {
#if (VED_VEH_DYN_INTFVER < 8U)
        {
            ved__internal_data.ved__ve_out.veh_velo = 0;
            ved__internal_data.ved__wpp_out.wheel_velo_front_left = 0;
            ved__internal_data.ved__wpp_out.wheel_velo_front_right = 0;
            ved__internal_data.ved__wpp_out.wheel_velo_rear_left = 0;
            ved__internal_data.ved__wpp_out.wheel_velo_rear_right = 0;
        }
#elif (VED_VEH_DYN_INTFVER >= 8U)
        {
            if (CORRECTED_EGO_SPEED_IS_ZERO_ON_ROLLER_BENCH == 1) {
                ved__internal_data.ved__ve_out.veh_velo = 0;
                ved__internal_data.ved__wpp_out.wheel_velo_front_left = 0;
                ved__internal_data.ved__wpp_out.wheel_velo_front_right = 0;
                ved__internal_data.ved__wpp_out.wheel_velo_rear_left = 0;
                ved__internal_data.ved__wpp_out.wheel_velo_rear_right = 0;
            }
        }
#endif
    }

    /* ved__wye modul */
    VED_ProcAutoWheelYawRate(input, nvdata);

    /* If internal gyro is used copy/overwrite internal gyro offset in autocode
     * yaw rate offset estimation data struct */
#if (CFG_VED__INT_GYRO)
    /* if external yaw rate signal is not valid use the internal yaw rate signal
     */

    if (VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State) !=
        VED_IO_STATE_VALID) {
        ved__internal_data.ved__wye_out.gier_yaw_rate_offset =
            ved__internal_data.ved__offsets_in.ved__yaw_offset.offset;
        ved__internal_data.ved__wye_out.gier_yaw_rate_offset_var =
            ved__internal_data.ved__offsets_in.ved__yaw_offset.var;
    }
#endif

    /* ved__gye modul */
    VED_ProcAutoSenYawRate(input);

    /* ved__aye modul */
    VED_ProcAutoLatAccelYawRate(input);

    /* ved__sye modul */
    VED_ProcAutoSwaYawRate(input, nvdata);

    /* ved__ye modul */
    VED_ProcAutoVehicleYawRate(input);

    /* ved__sae modul slip angle */
    VED_ProcAutoSideSlip(input);

    /* ved__mot_st modul motion state*/
#if (CFG_VED__MOT_STATE)
    VED_ProcMotionState(input, reqPorts->pAln_Monitoring);
#endif
}

/* ***********************************************************************
  @fn               VED_PostProcess */
static void VED_PostProcess(const reqVEDPrtList_t *reqPorts,
                            const VED_InputData_t *input,
                            const VED_NvData_t *VED_NvData,
                            proVEDPrtList_t *proPorts) {
    /* Get the stand still yaw rate offset */
#if ((!defined(CFG_VED__DIS_YWR_OFFSET_COMP)) || \
     (!CFG_VED__DIS_YWR_OFFSET_COMP))
#if (CFG_VED__INT_GYRO)
    const VED_YwrtOffsData_t *pYwrOffs = VED_YwrtGetOffsData();
    const VED_YwrOffsData_t *pYwrOffsStd = VED_YwrGetOffsData();
#else
    const VED_YwrOffsData_t *pYwrOffs = VED_YwrGetOffsData();
#endif
#endif

    /* copy the autocode nvm data from the provide port */
#if ((defined(CFG_VED__NVM_LEARN_DATA_ERROR)) && \
     (CFG_VED__NVM_LEARN_DATA_ERROR))
    /* check for valid values written into NVM */
    if ((VED_NvData->Write.SlfstGrad.SlfStGrad >=
         MINIMUM_SELF_NVM_STEERING_GRAD) &&
        (VED_NvData->Write.SlfstGrad.SlfStGrad <= MAXIMUM_SELF_STEERING_GRAD) &&
        (VED_NvData->Write.SlfstGrad.CalStatus >= 0) &&
        (VED_NvData->Write.SlfstGrad.CalStatus <= 1)) {
#endif
        proPorts->pNVMWrite->SlfstGrad = VED_NvData->Write.SlfstGrad;
#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
        proPorts->pNVMWrite->SlfstGrad.Dummy = 0;
#endif
        VED_SET_NVM_IO_STATE(
            VED_NVM_POS_SSG,
            VED__GET_NVM_IO_STATE(VED_NVM_POS_SSG, &VED_NvData->Write.State),
            &proPorts->pNVMWrite->State);
#if ((defined(CFG_VED__NVM_LEARN_DATA_ERROR)) && \
     (CFG_VED__NVM_LEARN_DATA_ERROR))
        /* check for valid values written into NVM */
        proPorts->pVED_Errors->OutPutErrors.SSGNVMOffsRg =
            VED_ERR_STATE_INACTIVE;
    } else {
        /*keep the old value into the NVM */
        proPorts->pVED_Errors->OutPutErrors.SSGNVMOffsRg = VED_ERR_STATE_ACTIVE;
    }
    if ((VED_NvData->Write.Wld.Wld_front >= MINIMUM_WHEEL_LOAD_DEP) &&
        (VED_NvData->Write.Wld.Wld_front <= MAXIMUM_WHEEL_LOAD_DEP) &&
        (VED_NvData->Write.Wld.Wld_front_quality >= 0) &&
        (VED_NvData->Write.Wld.Wld_front_quality <= 1)) {
#endif
        proPorts->pNVMWrite->Wld = VED_NvData->Write.Wld;
#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
        proPorts->pNVMWrite->Wld.Wld_rear = 0;
        proPorts->pNVMWrite->Wld.Wld_rear_quality = 0;
#endif
        VED_SET_NVM_IO_STATE(
            VED_NVM_POS_WLD,
            VED__GET_NVM_IO_STATE(VED_NVM_POS_WLD, &VED_NvData->Write.State),
            &proPorts->pNVMWrite->State);
#if ((defined(CFG_VED__NVM_LEARN_DATA_ERROR)) && \
     (CFG_VED__NVM_LEARN_DATA_ERROR))
        /* check for valid values written into NVM */
        proPorts->pVED_Errors->OutPutErrors.WLDNVMOffsRg =
            VED_ERR_STATE_INACTIVE;
    } else {
        /*keep the old value into the NVM */
        proPorts->pVED_Errors->OutPutErrors.WLDNVMOffsRg = VED_ERR_STATE_ACTIVE;
    }
#endif

    /* Convert yaw rates for diag and mts purpose */
    VED_YawRate2Curve(&ved__internal_data, proPorts);
#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
#if ((!defined(CFG_VED__DIS_YWR_OFFSET_COMP)) || \
     (!CFG_VED__DIS_YWR_OFFSET_COMP))
    if (proPorts->pVED_Offsets->Ywr.StandStillState > 0)
#endif
    {
        VED_CalcFastSwaOffset(input);
    }
#endif

    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_INIT,
                            input->Frame.CtrlMode)) {
        VED_InitVehDyn(proPorts->pVehicleDynamicSignals);
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
        VED_InitALNYawRate(proPorts->pALNYawRate);
#endif
        VED_InitOffsets(proPorts->pVED_Offsets);
        VED_InitErrors(proPorts->pVED_Errors);
        VED_InitInternal(&ved__internal_data);
        VED_InitModuleIf(&VED_ModIf);
    }

    /* output curvature */
    proPorts->pVehicleDynamicSignals->Lateral.Curve = VED_ModIf.Curve;
    if (proPorts->pVehicleDynamicSignals->Lateral.Curve.varC0 < 0.0F) {
        /* if variance is negative, the ye model encountered a problem, set
         * output to invalid and restart ye model */
        proPorts->pVehicleDynamicSignals->Lateral.Curve.varC0 = 100.0f;
        if (ved__ye_k.u_CurveFaultCounter < VED__VAR_FAULT_COUNTER_THRESHOLD) {
            ved__ye_k.u_CurveFaultCounter++;
        }
        if (ved__ye_k.u_CurveFaultCounter >= VED__VAR_FAULT_COUNTER_THRESHOLD) {
            VED_SET_IO_STATE(VED_SOUT_POS_CURVE, VED_IO_STATE_INVALID,
                             proPorts->pVehicleDynamicSignals->State);
            VED_ReinitYawRate();
        }
    } else {
        ved__ye_k.u_CurveFaultCounter = 0U;
    }
    VED_CalculateLaneErrorConf(reqPorts, input, proPorts->pVED_Offsets,
                               proPorts->pVehicleDynamicSignals);

    /* output yaw rate */
    proPorts->pVehicleDynamicSignals->Lateral.YawRate = VED_ModIf.YawRate;
    if (proPorts->pVehicleDynamicSignals->Lateral.YawRate.Variance < 0.0F) {
        /* if variance is negative, the ye model encountered a problem, set
         * output to invalid and restart ye model */
        proPorts->pVehicleDynamicSignals->Lateral.YawRate.Variance =
            VED__YAWRATE_MAX_VARIANCE;
        if (ved__ye_k.u_YawFaultCounter < VED__VAR_FAULT_COUNTER_THRESHOLD) {
            ved__ye_k.u_YawFaultCounter++;
        }
        if (ved__ye_k.u_YawFaultCounter >= VED__VAR_FAULT_COUNTER_THRESHOLD) {
            VED_SET_IO_STATE(VED_SOUT_POS_YWR, VED_IO_STATE_INVALID,
                             proPorts->pVehicleDynamicSignals->State);
            VED_ReinitYawRate();
        }
    } else {
        ved__ye_k.u_YawFaultCounter = 0U;
    }

    /* output lateral acceleration */
    proPorts->pVehicleDynamicSignals->Lateral.Accel = VED_ModIf.LatAcc;

    /* output driver intended curve */
    proPorts->pVehicleDynamicSignals->Lateral.DrvIntCurve.Curve =
        ved__internal_data.ved__sye_out.stw_curve;
    proPorts->pVehicleDynamicSignals->Lateral.DrvIntCurve.Variance =
        ved__internal_data.ved__sye_out.stw_curve_var;
    if (proPorts->pVehicleDynamicSignals->Lateral.DrvIntCurve.Variance < 0.0F) {
        /* if variance is negative, the ye model encountered a problem, set
         * output to invalid and restart ye model */
        proPorts->pVehicleDynamicSignals->Lateral.DrvIntCurve.Variance = 100.0f;
        if (ved__ye_k.u_DICurveFaultCounter <
            VED__VAR_FAULT_COUNTER_THRESHOLD) {
            ved__ye_k.u_DICurveFaultCounter++;
        }
        if (ved__ye_k.u_DICurveFaultCounter >=
            VED__VAR_FAULT_COUNTER_THRESHOLD) {
            VED_SET_IO_STATE(VED_SOUT_POS_DRCRV, VED_IO_STATE_INVALID,
                             proPorts->pVehicleDynamicSignals->State);
#if (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)
            ved__sye_initialize(
                (boolean_T)1, &ved__sye_M, &ved__sye_B,
                &ved__sye_DWork); /* Re-Initialize ved__sye model */
#endif
            VED_ReinitYawRate(); /* Re-Initialize ved__ye model */
        }
    } else {
        ved__ye_k.u_DICurveFaultCounter = 0U;
    }
    proPorts->pVehicleDynamicSignals->Lateral.DrvIntCurve.Gradient =
        ved__internal_data.ved__sye_out.stw_curve_grad;

    /*Limiting the Longitudinal Acceleration to defined threshold to avoid
     * overshoot of acceleration due to software reset - Temporary solution
     * needs to be removed once ve model is refined  */
    if (VED_ModIf.LongMot.VehAccel > MAX_ACCELERATION_LIMIT) {
        VED_ModIf.LongMot.VehAccel = MAX_ACCELERATION_LIMIT;
    } else if (VED_ModIf.LongMot.VehAccel < -MAX_ACCELERATION_LIMIT) {
        VED_ModIf.LongMot.VehAccel = -MAX_ACCELERATION_LIMIT;
    } else {
        /*  do nothing */
    }

    /* output velocity and acceleration variables */
    proPorts->pVehicleDynamicSignals->Longitudinal.MotVar.Velocity =
        VED_ModIf.LongMot.VehVelo;
    proPorts->pVehicleDynamicSignals->Longitudinal.MotVar.varVelocity =
        VED_ModIf.LongMot.VehVeloVar;
    proPorts->pVehicleDynamicSignals->Longitudinal.MotVar.Accel =
        VED_ModIf.LongMot.VehAccel;
    proPorts->pVehicleDynamicSignals->Longitudinal.MotVar.varAccel =
        VED_ModIf.LongMot.VehAccelVar;

    proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrVelo =
        ved__internal_data.ved__ve_out.veh_velo;
#if (!CFG_VED__DO_VELOCITY_CORR)
    proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrFact = 1.0F;
    proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrVar = 0.0F;
    proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrVelo =
        VED_ModIf.LongMot.VehVelo;
    proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.corrVeloVar =
        VED_ModIf.LongMot.VehVeloVar;
    proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.maxVelo =
        VED_ModIf.LongMot.VehVelo + 0.5F;
    proPorts->pVehicleDynamicSignals->Longitudinal.VeloCorr.minVelo =
        VED_ModIf.LongMot.VehVelo - 0.5F;
#endif

    /* output corrected acceleration, currently the same values as uncorrected
     * acceleration */
    proPorts->pVehicleDynamicSignals->Longitudinal.AccelCorr.corrAccel =
        VED_ModIf.LongMot.VehAccel;
    proPorts->pVehicleDynamicSignals->Longitudinal.AccelCorr.corrAccelVar =
        VED_ModIf.LongMot.VehAccelVar;

    /* output vehicle motion state */
    proPorts->pVehicleDynamicSignals->MotionState.MotState =
        VED_ModIf.LongMot.MotState.MotState;
    proPorts->pVehicleDynamicSignals->MotionState.Confidence =
        VED_ModIf.LongMot.MotState.Confidence;

    /* output slip angle */
    proPorts->pVehicleDynamicSignals->Lateral.SlipAngle =
        VED_ModIf.SideSlipAngle;

#if (VED_VEH_DYN_INTFVER > 6U)
    /* offset compensated steering wheel angle  */
    proPorts->pVehicleDynamicSignals->Lateral.OffCompStWheelAngle =
        input->Signals.StWheelAngle -
        ved__internal_data.ved__offsets_in.ved__swa_offset.offset;

#endif

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
    /* sensor yaw rate output */
    proPorts->pALNYawRate->YawRate =
        ved__internal_data.ved__gye_out_filt.gier_yaw_rate;
    proPorts->pALNYawRate->Quality = 0.0F;
#endif

    /* Generate qualitys for sensor yaw rate, vehicle yaw rate and vehicle curve
     */
    VED_GenYawAndCurveQualitys(&ved__internal_data, proPorts, input);

    /* VED should write this VersionNumber _EVERY_ time in "VEDExec()" when
     * writing VED_Offsets to MDB.*/
    proPorts->pVED_Offsets->uiVersionNumber = VED_OFFSETS_INTFVER;

#if ((!defined(CFG_VED__DIS_YWR_OFFSET_COMP)) || \
     (!CFG_VED__DIS_YWR_OFFSET_COMP))
    /* Yaw rate Offset output */
    proPorts->pVED_Offsets->Ywr.StandStillOffset =
        pYwrOffs->ToAutocode.OffsData.offset;
    proPorts->pVED_Offsets->Ywr.DynOffset =
        ved__internal_data.ved__wye_out.gier_yaw_rate_offset;
    proPorts->pVED_Offsets->Ywr.DynVariance =
        ved__internal_data.ved__wye_out.gier_yaw_rate_offset_var;
    /* Wrap state to the output state */
    if (TUE_CML_IsNonZero(ved__internal_data.ved__wye_out.gier_yaw_rate_offset -
                          pYwrOffs->ToAutocode.OffsData.offset)) {
        VED_OutYwrOffsType = (VED_OutYwrOffsType_t)OUT_YWR_OFFS_DYN;
        proPorts->pVED_Offsets->Ywr.StandStillState =
            (sint32)VED_OutYwrOffsType;
    } else {
        switch (pYwrOffs->ToAutocode.OffsData.state) {
            case VED__YAWRATE_STATE_STANDSTILL: /* stand still */
                /* if the standstill quality is above 0.75 then use full stand
                   still as base output state or the Yaw Rate offset
                   StandStillState was one time OUT_YWR_OFFS_STANDST_FULL */
#if (CFG_VED__INT_GYRO)
                if ((pYwrOffsStd->QualNoRed >= 0.75F) ||
                    (proPorts->pVED_Offsets->Ywr.StandStillState ==
                     (sint32)OUT_YWR_OFFS_STANDST_FULL))
#else
                if ((pYwrOffs->QualNoRed >= 0.75F) ||
                    (proPorts->pVED_Offsets->Ywr.StandStillState ==
                     (sint32)OUT_YWR_OFFS_STANDST_FULL))
#endif
                {
                    VED_OutYwrOffsType =
                        (VED_OutYwrOffsType_t)OUT_YWR_OFFS_STANDST_FULL;
                    proPorts->pVED_Offsets->Ywr.StandStillState =
                        (sint32)VED_OutYwrOffsType;
                } else {
                    /* if standstill quality is below 0.75 the stand still yaw
                     * rate offset sample time was to short */
                    VED_OutYwrOffsType =
                        (VED_OutYwrOffsType_t)OUT_YWR_OFFS_STANDST_SHORT;
                    proPorts->pVED_Offsets->Ywr.StandStillState =
                        (sint32)VED_OutYwrOffsType;
                }
                break;

            case VED__YAWRATE_STATE_NVM: /* eeprom */
                VED_OutYwrOffsType =
                    (VED_OutYwrOffsType_t)OUT_YWR_OFFS_STANDST_EEPROM;
                proPorts->pVED_Offsets->Ywr.StandStillState =
                    (sint32)VED_OutYwrOffsType;
                break;

            case VED__YAWRATE_STATE_NOT_ESTIMATED: /* no offset */
                VED_OutYwrOffsType = (VED_OutYwrOffsType_t)OUT_YWR_OFFS_NON;
                proPorts->pVED_Offsets->Ywr.StandStillState =
                    (sint32)VED_OutYwrOffsType;
                break;

            case VED__YAWRATE_STATE_KEEP_TYPE: /* fallthrough: do not take over
                                                  use last type */
            default:
                proPorts->pVED_Offsets->Ywr.StandStillState =
                    (sint32)VED_OutYwrOffsType;
                break;
        }
    }
#else
    /* Yaw rate Offset output */
    proPorts->pVED_Offsets->Ywr.StandStillOffset = 0.0F;
    proPorts->pVED_Offsets->Ywr.DynOffset = 0.0F;
    proPorts->pVED_Offsets->Ywr.DynVariance = 0.001F;
    proPorts->pVED_Offsets->Ywr.StandStillState = (sint32)OUT_YWR_OFFS_NON;
#endif

    /* Swa Offset output */
    proPorts->pVED_Offsets->Swa.Offset =
        ved__internal_data.ved__offsets_in.ved__swa_offset.offset;
    proPorts->pVED_Offsets->Swa.State =
        (sint32)ved__internal_data.ved__offsets_in.ved__swa_offset.state;
    proPorts->pVED_Offsets->Swa.Variance =
        ved__internal_data.ved__offsets_in.ved__swa_offset.var;

    /* Ay Offset output */
    proPorts->pVED_Offsets->Ay.Offset =
        ved__internal_data.ved__offsets_in.ved__ay_offset.offset;
    proPorts->pVED_Offsets->Ay.State =
        (sint32)ved__internal_data.ved__offsets_in.ved__ay_offset.state;
    proPorts->pVED_Offsets->Ay.Variance =
        ved__internal_data.ved__offsets_in.ved__ay_offset.var;

    /* In case of normal state set first cycle done flag */
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_RUNNING,
                            input->Frame.CtrlMode)) {
        /* Set flag to indicate that first cycle execution is completed */
        VED__SET_FIRST_CYCLE_DONE;

        /* Monitor the ved_ output signals for physical range only if the ved_
         * is in running mode */
        VED_MonitorOutput(input, proPorts, reqPorts);

        /* if monitoring yaw rate with alignment offset is switched on */
#if ((defined(CFG_VED__ALIGNMENT_OFFSET_MONITOR)) && \
     (CFG_VED__ALIGNMENT_OFFSET_MONITOR))
        /* Monitor the yaw rate offset */
        VED_AlignmentYawOffsMonitor(reqPorts, VED_In, &ved__internal_data,
                                    proPorts->pVED_Errors);
#endif

        /* Functional safety monitoring functions */
#if !(defined(CFG_VED__DIS_FUNCTIONAL_SAFETY_MON) && \
      (CFG_VED__DIS_FUNCTIONAL_SAFETY_MON))
        VED_FSMonitor(&ved__internal_data, &VED_ModIf, proPorts);
#endif
    }

    CopyVehParamToOutput(reqPorts, proPorts);
}
/* ***********************************************************************
  @fn               CopyVehParamToOutput */ /*!

  @brief            Transfer internal component data to acquisition

  @description      Sends all internal meas freezes

  @param[in]        reqPorts requested ports
  @param[in]        Services meas freeze function
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void CopyVehParamToOutput(const reqVEDPrtList_t *reqPorts,
                                 proVEDPrtList_t *proPorts) {
    memcpy(proPorts->pOriginVehicleSignals, reqPorts->pVehicleInputSignals,
           sizeof(V1_7_VEDVehSig_t));
}

/* ***********************************************************************
  @fn               VED_EnvMeasFreezeInternal */ /*!

  @brief            Transfer internal component data to acquisition 

  @description      Sends all internal meas freezes

  @param[in]        reqPorts requested ports
  @param[in]        Services meas freeze function
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_EnvMeasFreezeInternal(void) {
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
    VED_YwrData_t *pYwrData = VED_YwrGetPrivateData();
#endif
#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
    VED_SwaData_t *pSwaData = VED_SwaGetPrivateData();
#endif
#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
    VED_AyData_t *pAyData = VED_AyGetPrivateData();
#endif
#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    VED_WhsData_t *pWhsData = VED_WhsGetPrivateData();
#endif
#if (CFG_VED__INT_GYRO)
    VED_YwrtData_t *pYwrtData = VED_YwrtGetPrivateData();
#endif

#if (CFG_VED__USE_VELO_MONITOR)
#ifdef _WIN32
#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): Add meas freeze for velo monitoring if needed")

#endif
    /* VED_VelMon_t     *pVelMon   = VED_VelMonGetPrivateData(); */
    /* add some meas monitoring freeze data AuxMeasData.Reserved[0] =
     * pVelMon->cntOutSide; */
#endif

    /* Fetch the ved_ configuration */
    VED_GetConfig();

    /* Freeze component internal data groups */
#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_SWA, pSwaData, sizeof(VED_SwaData_t));

#endif
#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_AY, pAyData, sizeof(VED_AyData_t));

#endif
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_YWR, pYwrData, sizeof(VED_YwrData_t));

#endif

#if (CFG_VED__INT_GYRO)
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_YWRT, pYwrtData, sizeof(VED_YwrtData_t));

#endif

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_WHS, pWhsData, sizeof(VED_WhsData_t));

#endif

#if (CFG_VED__YWR_OFFSET_MONITOR)
    {
        VED_YwrMonData_t *pYwrMonData = VED_YwrMonGetPrivateData();
        MEAS_FREEZE_DIRECT(VED__MEAS_ID_YWR_MON, pYwrMonData,
                           sizeof(VED_YwrMonData_t));
    }
#endif

#if (CFG_VED__DO_VELOCITY_CORR)
    {
        VED_VelCorr_t *pCorrData = VED_VelCorrGetPrivateData();
        MEAS_FREEZE_BUFFERED(VED__MEAS_ID_VELCORR, pCorrData,
                             sizeof(VED_VelCorr_t));
    }
#if (CFG_VED__FS_VELO_CORR_MON)
    {
        VED_FSVelCorrMon_t *pCorrMonData = VED_FSVelCorrMonGetPrivateData();
        MEAS_FREEZE_DIRECT(VED__MEAS_ID_FSVELCORRMON, pCorrMonData,
                           sizeof(VED_FSVelCorrMon_t));
    }
#endif
#endif
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_INTDATA, &ved__internal_data,
                       sizeof(ved__internal_data));

    MEAS_FREEZE_DIRECT(VED__MEAS_ID_BAYESDATA, &ved__bayes_mot_states,
                       sizeof(ved__bayes_mot_states));

#if !(defined(CFG_VED__DIS_FUNCTIONAL_SAFETY_MON) && \
      (CFG_VED__DIS_FUNCTIONAL_SAFETY_MON))
    {
        VED_FSData_t *VED_FsData = VED_FSMonGetPrivateData();
        MEAS_FREEZE_DIRECT(VED__MEAS_ID_FS_MON, VED_FsData,
                           sizeof(VED_FSData_t));
    }
#endif
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_YE_K, &ved__ye_k, sizeof(VED_Ye_K_t));
    MEAS_FREEZE_DIRECT(VED__MEAS_ID_TIMESTAMPS, &VED_DeltaTimeStamp,
                       sizeof(VED_DeltaTimeStamp_t));

#if (defined(VED__FREEZE_INIT_SEQUENCE) && (VED__FREEZE_INIT_SEQUENCE))
    VED_InitSequenceMeasFreeze();
#endif

    MEAS_FREEZE_DIRECT(VED_MEAS_ID_SYNC_REF, &(s_SyncRef),
                       sizeof(VED__SM_t_SyncRef));

    return;
}

#if (CFG_VED__GEN_VELOCITY_VARIANCE)
/* ***********************************************************************
  @fn               VED_GenVeloVar */ /*!

  @brief            Generate the velocity variance with use of the acceleration

  @description      see brief description

  @param[in,out]    IntData velocity signal
  @param[out]       IntData velocity variance
  @return           void

  @pre              Precondition:  none
  @post             Postcondition: none
**************************************************************************** */
static void VED_GenVeloVar(VED_InternalData_t *IntData) {
    /* Generated velo variance is calculate by this parabolic function v_var(a)
     * = m*a^2 +b */
    IntData->ved__ve_out.veh_velo_var =
        (IntData->ved__ve_out.veh_accel * IntData->ved__ve_out.veh_accel *
         VED__GEN_VELO_VAR_M) +
        VED__GEN_VELO_VAR_B;
}
#endif

#if ((!CFG_VED__DIS_YWR_OFFSET_COMP) &&                     \
     ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
      (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)))
#if (CFG_VED__INT_GYRO)
/* ***********************************************************************
  @fn               VED_GenInternalYawRateQuality */ /*!

  @brief            Generate qualitys for Internal yaw rate sensor,

  @description      Takes internally calculated quality and reduces the quality
                    with difference between last filtered yaw rate and stand still offset
                    Sets the aln yaw rate offset compensation stat according
                    to the yaw rate offset state

  @param[in]        IntData yaw sensor data
  @param[in]        input yaw rate signal state
  @param[out]       proPorts output quality
  @return           void

  @pre              Precondition:  none

  @post             Postcondition: none
**************************************************************************** */
static float32 VED_GenInternalYawRateQuality(const VED_InternalData_t *IntData,
                                             const proVEDPrtList_t *proPorts,
                                             const VED_InputData_t *input) {
    /* If internal yaw sensor is used */
    const VED_YwrtOffsData_t *pYwrOffs = VED_YwrtGetOffsData();

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
    proPorts->pALNYawRate->Quality = pYwrOffs->Quality;
#endif

    /* Reduce sensor yaw rate quality with difference between raw yaw rate
     * offset and filtered yaw rate offset */
    {
#if (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)
        float32 ReduceQuality =
            fABS(ved__internal_data.ved__wye_out.gier_yaw_rate_offset -
                 IntData->ved__gye_out_filt.raw_est_yaw_offset_filt) *
            50.0F;
    boolean b_ReduceQualityisNotZero = TUE_CML_IsNonZero(ReduceQuality;
    boolean b_LastFilteredisZero = TUE_CML_IsZero(LastFiltered);

    /* Reduce sensor yaw rate with difference between last filtered yaw rate and stand still offset at stand still */
    if ((b_ReduceQualityisNotZero) || (b_LastFilteredisZero))
    {
            LastFiltered = IntData->ved__gye_out_filt.raw_est_yaw_offset_filt;
    }
    else
    {
            ReduceQuality +=
                fABS(ved__internal_data.ved__wye_out.gier_yaw_rate_offset -
                     LastFiltered) *
                50.0F;
    }

    if (ReduceQuality > 0.2F)
    {
            ReduceQuality = 0.2F;
    }
#else
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
        float32 ReduceQuality = 0.0F;
#endif
#endif

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
    proPorts->pALNYawRate->Quality -= ReduceQuality;

    /* Quality should not be negative */
    if (proPorts->pALNYawRate->Quality < 0.0F)
    {
            proPorts->pALNYawRate->Quality = 0.0F;
    }

    /* set the aln yaw rate offset compensation stat according to the yaw rate offset state */
    switch (pYwrOffs->OffsType)
    {
            case OFFS_NON:
                proPorts->pALNYawRate->OffsetCompState = ALN_YAW_RATE_NO_EEPROM;
                break;

            case OFFS_STANDST: /* fallthrough: same handling */
            case OFFS_DYN_AVG: /* fallthrough: same handling */
            case OFFS_DYN_INTER:
                /* if the standstill quality is above 0.75 then use full stand
                   still as base output state or the Sensor Yaw Rate
                   OffsetCompState was one time ALN_YAW_RATE_FULL_STAND_STILL */
                if ((pYwrOffs->Quality >= 0.75F) ||
                    (proPorts->pALNYawRate->OffsetCompState ==
                     ALN_YAW_RATE_FULL_STAND_STILL)) {
                    proPorts->pALNYawRate->OffsetCompState =
                        ALN_YAW_RATE_FULL_STAND_STILL;
                } else {
                    /* if standstill quality is below 0.75 the stand still yaw
                     * rate offset sample time was to short */
                    proPorts->pALNYawRate->OffsetCompState =
                        ALN_YAW_RATE_SHORT_STAND_STILL;
                }
                break;

            case OFFS_STANDST_EEPROM:
                proPorts->pALNYawRate->OffsetCompState =
                    ALN_YAW_RATE_NO_STAND_STILL;
                break;

            case OFFS_DYN_APPRX:    /* fallthrough: same as default */
            case OFFS_TEMPER_TABLE: /* fallthrough: same as default */
            default:
                /* do nothing */
                break;

    }

    /* if offset elapsed time is below the threshold set it to const quality */
    if (pYwrOffs->OffsElpsdTime <= VED__OFFSET_ELAPSED_TIME_QUALI_THRESH)
    {
            proPorts->pALNYawRate->OffsetCompState =
                ALN_YAW_RATE_PROCESS_STAND_STILL;
    }
#endif
    }

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
    /* Set output state to internal input yaw rate state */
    if (VED_GET_IO_STATE(VED_SIN_POS_YWRINT, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        proPorts->pALNYawRate->bSenYawRateState = TRUE;
    } else {
        proPorts->pALNYawRate->bSenYawRateState = FALSE;
    }
#endif

    return (pYwrOffs->Quality * 0.45F);
}

#endif
/* ***********************************************************************
  @fn               VED_GenExternalYawRateQuality */ /*!

  @brief            Generate qualitys for External yaw rate sensor,

  @description      Takes interally calculated quality and reduces the quality
                    with difference between last filtered yaw rate and stand still offset
                    Sets the aln yaw rate offset compensation stat according
                    to the yaw rate offset state
*/
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING))
/*!
  @param[in]        IntData yaw sensor data
*/
#endif
/*!
  @param[in]        input yaw rate signal state
  @param[out]       proPorts output quality
  @return           void

  @pre              Precondition:  none
  @post             Postcondition: none
**************************************************************************** */
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING))
static float32 VED_GenExternalYawRateQuality(const VED_InternalData_t *IntData,
                                             const proVEDPrtList_t *proPorts,
                                             const VED_InputData_t *input)
#else
static float32 VED_GenExternalYawRateQuality(const proVEDPrtList_t *proPorts,
                                             const VED_InputData_t *input)
#endif
{
    /* If external yaw sensor is used */
    const VED_YwrOffsData_t *pYwrOffs = VED_YwrGetOffsData();
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
    static float32 TempSensorYawRateQuality = 0.0F;
#endif
    static float32 TempVehSensorYawRateQuality = 0.0F;

#if (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)
    /* Reduce sensor yaw rate quality with difference between raw yaw rate
     * offset and filtered yaw rate offset */
    /* if the diff is 0.01rad the reduce Quality is 0.5 -> (Quality =
     * 0.05*diff[mrad])  */
    float32 ReduceQuality =
        fABS(IntData->ved__wye_out.raw_est_yaw_offset -
             IntData->ved__gye_out_filt.raw_est_yaw_offset_filt) *
        50.0F;
    boolean b_ReduceQualityIsNonYero =
        (boolean)TUE_CML_IsNonZero(ReduceQuality);
    boolean b_LastFilteredIsZero = (boolean)TUE_CML_IsZero(LastFiltered);

    /* Reduce sensor yaw rate with difference between last filtered yaw rate and
     * stand still offset at stand still */
    if ((b_ReduceQualityIsNonYero == TRUE) || (b_LastFilteredIsZero == TRUE)) {
        LastFiltered = IntData->ved__gye_out_filt.raw_est_yaw_offset_filt;
    } else {
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
        /* The difference between the current filtered dyn offset and the last
         * filtered last dyn offset*/
        ReduceQuality +=
            fABS(IntData->ved__wye_out.raw_est_yaw_offset - LastFiltered) *
            50.0F;
#endif
    }

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
    /* limit ReduceQuality */
    if (ReduceQuality > 0.2F) {
        ReduceQuality = 0.2F;
    }
#endif
#else
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
    float32 ReduceQuality = 0.0F;
#endif
#endif

    /* Case No yaw rate sensor offset -> quality = 0.25 - Reduce Quality
    Case stand still yaw rate sensor offset + its Quality is = 1.0 -> quality
    = 1.0 - Reduce Quality Case stand still yaw rate sensor offset + its Quality
    is < 1.0 -> quality = 0.75 - Reduce Quality Case EEPROM yaw rate sensor
    offset -> quality = 0.5 - Reduce Quality */
    switch (pYwrOffs->OffsType) {
        case OFFS_NON:
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
            proPorts->pALNYawRate->OffsetCompState = ALN_YAW_RATE_NO_EEPROM;
#endif
            TempVehSensorYawRateQuality = 0.0F;
            break;

        case OFFS_STANDST:
        case OFFS_DYN_AVG:
        case OFFS_DYN_INTER:
            /* if the standstill quality is above 0.75 then use full stand still
            as base output state
            or the Sensor Yaw Rate OffsetCompState was one time
            ALN_YAW_RATE_FULL_STAND_STILL */
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
            if ((pYwrOffs->QualNoRed >= 0.75F) ||
                (proPorts->pALNYawRate->OffsetCompState ==
                 (OffsetCompState_t)ALN_YAW_RATE_FULL_STAND_STILL)) {
                proPorts->pALNYawRate->OffsetCompState =
                    ALN_YAW_RATE_FULL_STAND_STILL;
            } else {
                /* if standstill quality is below 0.75 the stand still yaw rate
                 * offset sample time was to short */
                proPorts->pALNYawRate->OffsetCompState =
                    ALN_YAW_RATE_SHORT_STAND_STILL;
            }
#endif
            TempVehSensorYawRateQuality = 0.45F;
            break;

        case OFFS_STANDST_EEPROM:
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
            proPorts->pALNYawRate->OffsetCompState =
                ALN_YAW_RATE_NO_STAND_STILL;
#endif
            TempVehSensorYawRateQuality = 0.20F;
            break;

        case OFFS_DYN_APPRX:
        case OFFS_TEMPER_TABLE:
        default:
            break;
            /* take last values because static */
    }

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
    /* Quality should only be the reduce Quality */
    TempSensorYawRateQuality = ReduceQuality;
#endif

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
    /* if offset elapesed time is below the threshold set it to const quality */
    if (pYwrOffs->OffsElpsdTime <= VED__OFFSET_ELAPSED_TIME_QUALI_THRESH) {
        proPorts->pALNYawRate->OffsetCompState =
            ALN_YAW_RATE_PROCESS_STAND_STILL;
    }

    /* copy to stuct */
    proPorts->pALNYawRate->Quality = TempSensorYawRateQuality;

    /* Set output state to external input yaw rate state */

    if (VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        proPorts->pALNYawRate->bSenYawRateState = TRUE;
    } else {
        proPorts->pALNYawRate->bSenYawRateState = FALSE;
    }
#else
    (void)input;    /* remove compiler warning, input is not used in this
                       configuration */
    (void)proPorts; /* remove compiler warning, proPorts is not used in this
                       configuration */
#endif

    return TempVehSensorYawRateQuality;
}
#endif

/* ***********************************************************************
  @fn               VED_GenYawAndCurveQualitys */ /*!

  @brief            Generate qualitys for sensor yaw rate,
                    and for vehicle curve and vehicle yaw rate.

  @description      Sets curve quality based on yaw rate quality and signal state,
                    steering wheel quality and lateral acceleration offset quality

  @param[in]        IntData curve data
  @param[in]        proPorts yaw rate signal state
  @param[out]       input output quality
  @return           void

  @pre              Precondition:  none

  @post             Postcondition: none
**************************************************************************** */
static void VED_GenYawAndCurveQualitys(const VED_InternalData_t *IntData,
                                       proVEDPrtList_t *proPorts,
                                       const VED_InputData_t *input) {
    float32 VehSensorYawQuality;
    float32 VehicleCurveYawQuality = 0.0;

#if ((CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING) &&                     \
     (CFG_VED__DIS_SWA_OFFSET_COMP) && (CFG_VED__DIS_LAT_OFFSET_COMP) && \
     (CFG_VED__DIS_WHS_OFFSET_COMP))
    (void)IntData; /* remove compiler warning, IntData is not used in this
                      configuration */
#endif

    /* Sensor yaw rate quality */
    /* If ved_ does the sensor yaw rate offset is estimation */
#if ((!CFG_VED__DIS_YWR_OFFSET_COMP) &&                     \
     ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
      (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)))
    /* If internal yaw sensor is used, the quality of the internal yaw rate
     * estimation is used */
#if (CFG_VED__INT_GYRO)
    /* if external yaw rate signal is valid use the external yaw rate signal
     * otherwise the internal */

    if (VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        /* Get quality from external yaw rate sensor processing */
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING))
        VehSensorYawQuality =
            VED_GenExternalYawRateQuality(IntData, proPorts, input);
#else
        VehSensorYawQuality = VED_GenExternalYawRateQuality(proPorts, input);
#endif
    } else {
        /* Get quality from internal yaw rate sensor processing */
        VehSensorYawQuality =
            VED_GenInternalYawRateQuality(IntData, proPorts, input);
    }
#else
    /* If external yaw sensor is used */
    /* Get quality from external yaw rate sensor processing */
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING))
    VehSensorYawQuality =
        VED_GenExternalYawRateQuality(IntData, proPorts, input);
#else
    VehSensorYawQuality = VED_GenExternalYawRateQuality(proPorts, input);
#endif
#endif
#else
    /* If yaw rate offset compensation is disabeled */
    VehSensorYawQuality = 0.45F;

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
    proPorts->pALNYawRate->Quality = 1.0F;

    /* set the offset compensation stat always to full stand still */
    proPorts->pALNYawRate->OffsetCompState = ALN_YAW_RATE_FULL_STAND_STILL;

    /* Set output state to external input yaw rate state */

    if (VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State) ==
        VED_IO_STATE_VALID) {
        proPorts->pALNYawRate->bSenYawRateState = TRUE;
    } else {
        proPorts->pALNYawRate->bSenYawRateState = FALSE;
    }
#else
    (void)input;
#endif

#endif

    /* Add vehicle sensor yaw rate quality */
    VehicleCurveYawQuality += VehSensorYawQuality;

    /* Add steering wheel angle offset quality, if available */
#if (!CFG_VED__DIS_SWA_OFFSET_COMP)
    if (IntData->ved__offsets_in.ved__swa_offset.state > (uint8)2U) {
        VehicleCurveYawQuality += 0.35F;
    }
#else
    VehicleCurveYawQuality += 0.35F;
#endif

    /* Add lateral acceleration offset quality, if available */
#if (!CFG_VED__DIS_LAT_OFFSET_COMP)
    if (IntData->ved__offsets_in.ved__ay_offset.state > (uint8)2U) {
        VehicleCurveYawQuality += 0.15F;
    }
#else
    VehicleCurveYawQuality += 0.15F;
#endif

    /* Add wheel velocity offset quality, if available */
#if (!CFG_VED__DIS_WHS_OFFSET_COMP)
    if ((IntData->ved__offsets_in.ved__whs_offset.offset_ratio_front_dev <
         1.0F) &&
        (IntData->ved__offsets_in.ved__whs_offset.offset_ratio_rear_dev <
         1.0F)) {
        VehicleCurveYawQuality += 0.05F;
    }
#else
    VehicleCurveYawQuality += 0.05F;
#endif

#if (!CFG_VED__USE_EXT_PROC_YAW_RATE)
    proPorts->pVehicleDynamicSignals->Lateral.YawRate.Quality =
        VehicleCurveYawQuality;
#else
    proPorts->pVehicleDynamicSignals->Lateral.YawRate.Quality = 1.0F;
#endif

#if (!CFG_VED__USE_EXT_PROC_CURVATURE)
    proPorts->pVehicleDynamicSignals->Lateral.Curve.Quality =
        VehicleCurveYawQuality;
#else
    proPorts->pVehicleDynamicSignals->Lateral.Curve.Quality = 1.0F;
#endif
}

/* ***********************************************************************
  @fn               VED_InitVehDyn */ /*!

  @brief            Initialize ved_ output data

  @description      Sets all data in VehDyn to default values

  @param[in]        -
  @param[out]       VehDyn output data
  @return           void

  @pre              Precondition:  none
  @post             Postcondition: none
**************************************************************************** */
static void VED_InitVehDyn(VEDVehDyn_t *VehDyn) {
    /* clear everything including alignment bytes */
    (void)memset(VehDyn, 0, sizeof(VEDVehDyn_t));

    VehDyn->uiVersionNumber = VED_VEH_DYN_INTFVER;
#if (VED_VEH_DYN_INTFVER <= 5)
    VehDyn->State[0] = VED__VEH_DYN_IO_STATE_INIT;
    VehDyn->State[1] = VED__VEH_DYN_IO_STATE_INIT;
#else
    (void)memset(VehDyn->State, VED_IO_STATE_INIT, sizeof(VehDyn->State));
#endif

    VehDyn->MotionState.MotState = VED_LONG_MOT_STATE_MOVE;

    return;
}

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
/* ***********************************************************************
  @fn               VED_InitALNYawRate */ /*!

  @brief            Initialize ved_ alignment yaw rate

  @description      Sets all data in ALNYawRate to default values

  @param[in]        -
  @param[out]       ALNYawRate aln data
  @return           void

  @pre              Precondition:  none
  @post             Postcondition: none
**************************************************************************** */
static void VED_InitALNYawRate(ALNYawRate_t *ALNYawRate) {
    ALNYawRate->uiVersionNumber = VED__ALN_YAW_RATE_INTFVER;
    ALNYawRate->YawRate = 0.F;
    ALNYawRate->Quality = 0.F;
    ALNYawRate->bSenYawRateState = FALSE;
    ALNYawRate->OffsetCompState = ALN_YAW_RATE_NO_EEPROM;
}
#endif

/* ***********************************************************************
  @fn               VED_InitOffsets */ /*!

  @brief            Initialize ved_ offset data

  @description      Sets all data in VED_Offsets to default values

  @param[in]        -
  @param[out]       VED_Offsets offset data
  @return           void

  @pre              Precondition:  none
  @post             Postcondition: none
**************************************************************************** */
static void VED_InitOffsets(VED_VEDOffsets_t *VED_Offsets) {
    VED_Offsets->uiVersionNumber = VED_OFFSETS_INTFVER;

    VED_Offsets->Ay.Offset = 0.F;
    VED_Offsets->Ay.State = 0L;
    VED_Offsets->Ay.Variance = 0.F;

    VED_Offsets->Swa.Offset = 0.F;
    VED_Offsets->Swa.State = 0L;
    VED_Offsets->Swa.Variance = 0.F;

    VED_Offsets->Ywr.DynOffset = 0.F;
    VED_Offsets->Ywr.DynVariance = 0.F;
    VED_Offsets->Ywr.StandStillOffset = 0.F;
    VED_Offsets->Ywr.StandStillState = 0L;
}

/* ***********************************************************************
  @fn               VED_InitErrors */ /*!

  @brief            Initialize ved_ fault output data

  @description      Sets all faults in VED_Errors to unknown state

  @param[in]        -
  @param[out]       VED_Errors fault data
  @return           void

  @pre              Precondition:  none
  @post             Postcondition: none
**************************************************************************** */
static void VED_InitErrors(VED_Errors_t *VED_Errors) {
    VED_Errors->uiVersionNumber = VED_ERRORS_INTFVER;

    /* Initialize input signal error states */
    VED_Errors->SignalInputErrors.InputSignalError = VED_ERR_STATE_UNKNOWN;
    VED_Errors->SignalInputErrors.InputSignalPeakError = VED_ERR_STATE_UNKNOWN;

    /* Initialize input parameter error states */
    VED_Errors->ParInputErrors.InputParameterError = VED_ERR_STATE_UNKNOWN;

    /* Initialize output error states */
    VED_Errors->OutPutErrors.YwrOffsRg = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.SwaOffsRg = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.AyOffsRg = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.VelCorrRg = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.NVMYwrOffsRg = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.NVMSwaOffsRg = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.NVMAyOffsRg = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.NVMVelCorrRg = VED_ERR_STATE_UNKNOWN;
#if ((defined(CFG_VED__NVM_LEARN_DATA_ERROR)) && \
     (CFG_VED__NVM_LEARN_DATA_ERROR))
    /* check for valid values written into NVM */
    VED_Errors->OutPutErrors.YwrNVMOffsRg = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.SwaNVMOffsRg = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.AyNVMOffsRg = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.SSGNVMOffsRg = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.WLDNVMOffsRg = VED_ERR_STATE_UNKNOWN;
#endif
    VED_Errors->OutPutErrors.VelCorrWin = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.VelMon = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.VelMonLt = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.VelocityErr = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.YawRateErr = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.YwrMonVehHalt = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.YwrMonVehDOff = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.YwrMonAdmWdrwl = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.YwrACCMonAlignm = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.YwrCGMonAlignm = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.YwrMonVehHaltCal = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.VED_FS_YR_VS_AY = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.VED__FS_YR_VS_SWA = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.VED__FS_YR_VS_WSP = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.VED__VEH_YWR_NOT_AVAILABLE = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.VED_VEH_VEL_NOT_AVAILABLE = VED_ERR_STATE_UNKNOWN;
    VED_Errors->OutPutErrors.VED_FS_VEH_CORR_MON = VED_ERR_STATE_UNKNOWN;
}

/* ***********************************************************************
  @fn               VED_InitInternal */ /*!

  @brief            Initialize ved_ internal autocode data bus

  @description      Sets all data in VED_InternalData to 0

  @param[in]        -
  @param[out]       intData data
  @return           void

  @pre              Precondition:  none
  @post             Postcondition: none
**************************************************************************** */
static void VED_InitInternal(VED_InternalData_t *intData) {
    (void)memset(intData, 0x00, sizeof(VED_InternalData_t));
}

/* ***********************************************************************
  @fn               VED_InitProcess */ /*!

  @brief            Initialize module data

  @description      Sets handcode, autocode and provived output data to default values

  @param[in]        reqPorts
  @param[in]        input
  @param[out]       proPorts VED output data
  @return           void

  @pre              Precondition:  none
  @post             Postcondition: none
**************************************************************************** */
static void VED_InitProcess(const reqVEDPrtList_t *reqPorts,
                            const VED_InputData_t *input,
                            const proVEDPrtList_t *proPorts) {
#if (!(CFG_VED__YWR_OFFSET_MONITOR))
    (void)input; /* remove compiler warning, input is not used in this
                    configuration */
#endif

#if ((defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) && \
     (CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
    (void)reqPorts; /* remove compiler warning, reqPorts is not used in this
                       configuration */
#endif

    /* Clear first cycle execution completed flag */
    VED__RESET_FIRST_CYCLE_DONE;

#if (CFG_VED__INT_GYRO)
    VED_YwrtInit(proPorts);
#endif

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* Initialize yaw rate sensor module */
    VED_WhsInit();
#endif

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
    /* Initialize yaw rate sensor module */
    VED_YwrInit(reqPorts, proPorts);
#endif

#if (CFG_VED__YWR_OFFSET_MONITOR)
    /* Initialize yaw rate sensor offset monitoring */
    VED_YwrMonInit(input, &VED_ModIf, proPorts->pVED_Errors);
#endif

#if (((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
      (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) ||         \
     ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
      (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)))
    /* Initialize steering wheel angle sensor module */
    VED_AySwaInit(reqPorts, proPorts);
#endif

#if ((defined(CFG_VED__DO_VELOCITY_CORR)) && (CFG_VED__DO_VELOCITY_CORR))
    /* Initialize velocity correction */
    VED_VelCorrInit(proPorts);
#endif

    /* Initialize velocity monitoring */
#if (CFG_VED__USE_VELO_MONITOR)
    VED_VelMonInit(proPorts->pVED_Errors);
#endif

    /* Functional savety monitoring functions */
#if !(defined(CFG_VED__DIS_FUNCTIONAL_SAFETY_MON) && \
      (CFG_VED__DIS_FUNCTIONAL_SAFETY_MON))
    VED_FSInit();
#endif

    /* Module Interface */
    VED_InitModuleIf(&VED_ModIf);
    VED_InitInternal(&ved__internal_data);

    /* Auto code */

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    /* Initialize ved__wpp model */
    ved__wpp_initialize((boolean_T)1, &ved__wpp_M, &ved__wpp_B,
                        &ved__wpp_DWork);

    /* Initialize ved__ve model */
    ved__ve_initialize((boolean_T)1, &ved__ve_M, &ved__ve_B, &ved__ve_DWork);

    /* Initialize ved__wye model */
    ved__wye_initialize((boolean_T)1, &ved__wye_M, &ved__wye_B,
                        &ved__wye_DWork);
#endif

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
    /* Initialize ved__gye model */
    ved__gye_initialize((boolean_T)1, &ved__gye_M, &ved__gye_B,
                        &ved__gye_DWork);

#if (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)
    /* Initialize ved__gye_filt model */
    ved__gye_initialize((boolean_T)1, &ved__gye_M_FILT, &ved__gye_B_FILT,
                        &ved__gye_DWork_FILT);
    LastFiltered = 0.0F;     /* Last filtered dyn yaw rate offset */
    OldFiltYawOffset = 0.0F; /* Filter delay for yaw rate offset */
#endif

#endif

#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
    /* Initialize ved__ay model */
    ved__aye_initialize((boolean_T)1, &ved__aye_M, &ved__aye_DWork);
#endif

#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
    /* Initialize ved__sye model */
    ved__sye_initialize((boolean_T)1, &ved__sye_M, &ved__sye_B,
                        &ved__sye_DWork);
#endif

#if ((!defined(CFG_VED__USE_EXT_PROC_YAW_RATE)) || \
     (!CFG_VED__USE_EXT_PROC_YAW_RATE))
    /* Initialize ved__ve model */
    ved__ye_initialize((boolean_T)1, &ved__ye_M, &ved__ye_B, &ved__ye_DWork);
#endif

#if ((!defined(CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION)) || \
     (!CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION))
    /* Initialize ved__sae model */
    ved__sae_initialize((boolean_T)1, &ved__sae_M, &ved__sae_DWork);
#endif

#if (CFG_VED__MOT_STATE)
    /* Initialize ved__mot_st model */
    ved__mot_st_initialize((boolean_T)1, &ved__mot_st_M, &ved__mot_st_B,
                           &ved__mot_st_DWork);
#endif

    ved__internal_data.ved__offsets_in.ved__swa_offset.offset = 0.0F;
    ved__internal_data.ved__offsets_in.ved__swa_offset.state = 1U;
    ved__internal_data.ved__offsets_in.ved__swa_offset.var = 0.00001F;

    ved__internal_data.ved__offsets_in.ved__yaw_offset.offset = 0.0F;
    ved__internal_data.ved__offsets_in.ved__yaw_offset.state = 1U;
    ved__internal_data.ved__offsets_in.ved__yaw_offset.var = 0.000000001F;

#if ((!defined(CFG_VED__DIS_YWR_OFFSET_COMP)) || \
     (!CFG_VED__DIS_YWR_OFFSET_COMP))
    /* init external yaw rate offset state */
    VED_OutYwrOffsType = (VED_OutYwrOffsType_t)OUT_YWR_OFFS_NON;
#endif

    ved__internal_data.ved__offsets_in.ved__ay_offset.offset = 0.0F;
    ved__internal_data.ved__offsets_in.ved__ay_offset.state = 1U;
    ved__internal_data.ved__offsets_in.ved__ay_offset.var = 0.2F;

    ved__internal_data.ved__offsets_in.ved__whs_offset.offset_ratio_front =
        1.0F;
    ved__internal_data.ved__offsets_in.ved__whs_offset.offset_ratio_front_dev =
        1.0F;
    ved__internal_data.ved__offsets_in.ved__whs_offset.offset_ratio_rear = 1.0F;
    ved__internal_data.ved__offsets_in.ved__whs_offset.offset_ratio_rear_dev =
        1.0F;
    ved__internal_data.ved__offsets_in.ved__whs_offset.SpeedRange = 0;

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
    /* Initalize alignment yaw rate */
    VED_InitALNYawRate(proPorts->pALNYawRate);
#endif

    /* Initialize offsets */
    VED_InitOffsets(proPorts->pVED_Offsets);

    /* Initialize error states */
    VED_InitErrors(proPorts->pVED_Errors);

    /* Init monitoring */
    VED_InitMon();

    return;
}

/* ***********************************************************************
  @fn               VED_ReinitYawRate */ /*!

  @brief            ReInitializes the YE model

  @description      If the YE model calculates a negative variance for
                    several cycles, a re-init of the YE model will be done.

  @param[in]        -
  @param[out]       -
  @return           -

  @pre              -
  @post             -
**************************************************************************** */
static void VED_ReinitYawRate(void) {
    ved__ye_initialize((boolean_T)1, &ved__ye_M, &ved__ye_B, &ved__ye_DWork);
    ved__ye_k.u_CurveFaultCounter = 0U;
    ved__ye_k.u_YawFaultCounter = 0U;
    ved__ye_k.u_DICurveFaultCounter = 0U;
}

/* ***********************************************************************
  @fn               VED_IsFirstCycleDone */ /*!

  @brief            Test if one complete cycle has been executed after exiting
                    initialization state

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           false: first cycle not done, true: first cycle completed

  @pre              -
  @post             -
**************************************************************************** */
boolean VED_IsFirstCycleDone(void) { return (VED__IS_FIRST_CYCLE_DONE); }

/* ***********************************************************************
  @fn               VED_GetCycleTime */ /*!

  @brief            Return component cycle time

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           Cycle time in seconds

  @pre              -
  @post             -
**************************************************************************** */
float32 VED_GetCycleTime(void) { return (VED__GET_CYCLE_TIME); }

/* ***********************************************************************
  @fn               VED_IinitModuleIf */ /*!

  @brief            Init the Module Interface

  @description      Sets all data in VED_ModIf to default values

  @param[in]        -
  @param[out]       mif interface data
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_InitModuleIf(VED_ModIf_t *mif) {
    mif->LongMot.MotState.MotState = VED_LONG_MOT_STATE_MOVE_FWD;
    mif->LongMot.MotState.Confidence = (float32)0.F;

    mif->LongMot.VehVelocityCorr = 0.F;
    mif->LongMot.VehVelo = 0.F;
    mif->LongMot.VehAccel = 0.F;
    mif->LongMot.VehAccelVar = 0.F;
    mif->LongMot.VehVeloVar = 0.F;

    mif->SideSlipAngle.SideSlipAngle = 0.F;
    mif->SideSlipAngle.Variance = 0.F;

    mif->Curve.Curve = 0.F;
    mif->Curve.C1 = 0.F;
    mif->Curve.Gradient = 0.F;
    mif->Curve.varC0 = 0.F;
    mif->Curve.varC1 = 0.F;

    mif->YawRate.YawRate = 0.F;
    mif->YawRate.Variance = 0.F;
    mif->YawRate.Quality = 0.F;

    mif->LongMot.FLWhlVelo = 1.0F;
    mif->LongMot.FRWhlVelo = 1.0F;
    mif->LongMot.RLWhlVelo = 1.0F;
    mif->LongMot.RRWhlVelo = 1.0F;
    mif->LongMot.VelCorrFact = 1.0F;

    mif->LatAcc.LatAccel = 0.F;
    mif->LatAcc.Variance = 0.F;

    mif->YwrOffset.offset = 0.F;
    mif->YwrOffset.var = 0.F;

    mif->AyOffset.offset = 0.F;
    mif->AyOffset.var = 0.F;
    mif->AyOffset.state = 0U;

    mif->SwaOffset.offset = 0.F;
    mif->SwaOffset.state = 0U;
    mif->SwaOffset.var = 0.F;

    return;
}

/* ***********************************************************************
  @fn               VED_InterfaceHand2Auto */ /*!

  @brief            Interface from handcode to autocode

  @description      Sets all data in VED_InternalData which is an interface
                    between handcode and autocode  to default values

  @param[in]        input yaw rate signal state
  @param[in]        IntData steering angle and lat accel offset
  @param[out]       mif interface data
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_InterfaceHand2Auto(const VED_InputData_t *input,
                                   VED_InternalData_t *IntData,
                                   const VED_ModIf_t *mif) {
#if ((CFG_VED__DIS_SWA_OFFSET_COMP) && (CFG_VED__DIS_LAT_OFFSET_COMP))
    (void)mif; /* remove compiler warning, mif is not used in this configuration
                */
#endif

    /*<----- Steering wheel angle offset  ----->*/
#if (CFG_VED__DIS_SWA_OFFSET_COMP)
    /* input signal is already compensated, there is no zero point offset */
    IntData->ved__offsets_in.ved__swa_offset.state = 3U;
    IntData->ved__offsets_in.ved__swa_offset.offset = 0.F;
    IntData->ved__offsets_in.ved__swa_offset.var = 0.00001F;
#else
    { /*<----- Pass steering wheel angle offset  ----->*/
        /* Transfer steering wheel offset to auto code */
        IntData->ved__offsets_in.ved__swa_offset = mif->SwaOffset;
        IntData->ved__offsets_in.ved__swa_offset.var = 0.00001F;
    }
#endif

    /*<----- Yaw rate offset  ----->*/
#if (CFG_VED__DIS_YWR_OFFSET_COMP)
    /* input signal is already compensated, there is no zero point offset */
    IntData->ved__offsets_in.ved__yaw_offset.offset = 0.F;
    IntData->ved__offsets_in.ved__yaw_offset.var = 0.000000001F;
    IntData->ved__offsets_in.ved__yaw_offset.state = 1U;
    IntData->ved__offsets_in.ved__yaw_offset.quality = 1.5F;
    (void)input;
#else
    /*<----- Pass yaw rate offset  ----->*/
    {
        const ToAutocode_t *ToAutocode;
#if (CFG_VED__INT_GYRO)
        /* if external yaw rate signal is valid use the external yaw rate signal
         * otherwise the internal */

        if (VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State) ==
            VED_IO_STATE_VALID) {
            const VED_YwrOffsData_t *pYwrOffs = VED_YwrGetOffsData();
            ToAutocode = &pYwrOffs->ToAutocode;
        } else {
            const VED_YwrtOffsData_t *pYwrOffs = VED_YwrtGetOffsData();
            ToAutocode = &pYwrOffs->ToAutocode;
        }
#else
        const VED_YwrOffsData_t *pYwrOffs = VED_YwrGetOffsData();
        (void)input; /* remove compiler warning */
        /* Copy the ToAutocode struct from the external stand still yaw rate
         * offset module */
        ToAutocode = &pYwrOffs->ToAutocode;
#endif
        /* Monitor the Dynamic yaw rate offset signal */
        VED_MonitorDynYwrOffset(ToAutocode, IntData);
    }
#endif

    /*<----- Wheel Speed offset  ----->*/
#if (CFG_VED__DIS_WHS_OFFSET_COMP)
    /* input signal has been already compensated, there is no zero point offset
     */
    IntData->ved__offsets_in.ved__whs_offset.offset_ratio_front = 1.0F;
    IntData->ved__offsets_in.ved__whs_offset.offset_ratio_front_dev = 1E-6F;

    IntData->ved__offsets_in.ved__whs_offset.offset_ratio_rear = 1.0F;
    IntData->ved__offsets_in.ved__whs_offset.offset_ratio_rear_dev = 1E-6F;

    IntData->ved__offsets_in.ved__whs_offset.SpeedRange = 0;

#else
    /*<----- Pass wheel speed offsets  ----->*/
    /* Get the wheel speed offset from the whs modul */
    IntData->ved__offsets_in.ved__whs_offset = VED_WhsOffset();
#endif

    /*<----- Lateral acceleration offset ----->*/
#if (CFG_VED__DIS_LAT_OFFSET_COMP)
    /* input signal has been already compensated, there is no zero point offset
     */
    IntData->ved__offsets_in.ved__ay_offset.state = 3U;
    IntData->ved__offsets_in.ved__ay_offset.offset = 0.F;
    IntData->ved__offsets_in.ved__ay_offset.var = 0.00001F;
#else
    /*<----- Pass lateral acceleration offset  ----->*/
    /* Transfer steering wheel offset to auto code */
    IntData->ved__offsets_in.ved__ay_offset = mif->AyOffset;
#endif

    return;
}

/* ***********************************************************************
  @fn               VED_InterfaceAuto2Hand */ /*!

  @brief            Interface from autocode to handcode

  @description      Sets all data in  VED_ModIf which is an interface
                    between autocode and handcode  to default values

  @param[in]        IntData autocode data
  @param[out]       mif interface data
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
#if (CFG_VED__GEN_VELOCITY_VARIANCE)
static void VED_InterfaceAuto2Hand(VED_InternalData_t *IntData,
                                   VED_ModIf_t *mif)
#else
static void VED_InterfaceAuto2Hand(const VED_InternalData_t *IntData,
                                   VED_ModIf_t *mif)
#endif
{

#if (CFG_VED__GEN_VELOCITY_VARIANCE)
#if (CFG_VED__USE_EX_LONG_ACCEL)
    IntData->ved__ve_out.veh_accel = mif->LongMot.VehAccel;
#endif
    VED_GenVeloVar(IntData);
#endif
    mif->LongMot.VehVeloVar = IntData->ved__ve_out.veh_velo_var;

#if (!CFG_VED__USE_EX_LONG_ACCEL && !CFG_VED__USE_EX_LONG_VELO)
    mif->LongMot.VehAccel = IntData->ved__ve_out.veh_accel;
#endif
    mif->LongMot.VehAccelVar = IntData->ved__ve_out.veh_accel_var;

#if (!CFG_VED__DO_VELOCITY_CORR)
    mif->LongMot.VehVelocityCorr = IntData->ved__ve_out.veh_velo;
#endif

#if (CFG_VED__MOT_STATE)
    mif->LongMot.MotState.MotState = IntData->ved__mot_st_out.mot_state;
    mif->LongMot.MotState.Confidence =
        (float32)IntData->ved__mot_st_out.mot_quality;
#endif

#if (!CFG_VED__USE_EXT_PROC_CURVATURE)
    /* if internal curvature estimation is used, copy output data to module
     * interface */
    mif->Curve.Curve = IntData->ved__ye_out.veh_merge_curve;
    mif->Curve.C1 = 0.F;
    mif->Curve.Gradient = IntData->ved__ye_out.veh_merge_curve_grad;
    mif->Curve.varC0 = IntData->ved__ye_out.veh_merge_curve_var;
    mif->Curve.varC1 = 0.F;
    mif->Curve.Quality = 0.F;
#endif

#if (!CFG_VED__USE_EXT_PROC_YAW_RATE)
    /* if internal yaw rate estimation is used, copy output data to module
     * interface */
    mif->YawRate.YawRate = ved__internal_data.ved__ye_out.veh_yaw_rate;
    mif->YawRate.Variance = ved__internal_data.ved__ye_out.veh_yaw_rate_var;
    mif->YawRate.Quality = 0.F;
#endif

#if (!CFG_VED__USE_EXT_PROC_SIDE_SLIP_ANGLE)
    mif->SideSlipAngle.SideSlipAngle =
        ved__internal_data.ved__sae_out.est_slip_angle;
    mif->SideSlipAngle.Variance =
        ved__internal_data.ved__sae_out.est_slip_angle_var;
#endif

    mif->LongMot.FLWhlVelo = IntData->ved__wpp_out.wheel_velo_front_left;
    mif->LongMot.FRWhlVelo = IntData->ved__wpp_out.wheel_velo_front_right;
    mif->LongMot.RLWhlVelo = IntData->ved__wpp_out.wheel_velo_rear_left;
    mif->LongMot.RRWhlVelo = IntData->ved__wpp_out.wheel_velo_rear_right;

    mif->LatAcc.LatAccel = IntData->ved__ye_out.veh_lat_accel;
    mif->LatAcc.Variance = IntData->ved__ye_out.veh_merge_curve_var +
                           (2.0F * IntData->ved__ve_out.veh_velo_var);

    /* write the the dynamic yaw rate offset to the module interface */
    mif->YwrOffset.offset = IntData->ved__wye_out.gier_yaw_rate_offset;
    mif->YwrOffset.var = IntData->ved__wye_out.gier_yaw_rate_offset_var;

#if ((CFG_VED__MOT_STATE) || (!CFG_VED__MOT_STATE))
    if ((VED_ModIf.LongMot.MotState.MotState == VED_LONG_MOT_STATE_MOVE_RWD) &&
        (VED_ModIf.LongMot.VehVelo > 0.0F)) {
        ved__internal_data.ved__wye_out.whl_yaw_rate =
            -ved__internal_data.ved__wye_out.whl_yaw_rate;
        ved__internal_data.ved__sye_out.stw_yaw_rate =
            -ved__internal_data.ved__sye_out.stw_yaw_rate;
        ved__internal_data.ved__aye_out.ay_yaw_rate =
            -ved__internal_data.ved__aye_out.ay_yaw_rate;
    }
#endif

    return;
}

/* **********************************************************************
  @fn               VED_YawRate2Curve */ /*!

  @brief            Convert yaw rates to curve and copy the data to meas buffer

  @description      Calculates the curves for all the internal yaw rates
                    These curves are used by the birdeye MO

  @param[in]        intData autocode data
  @param[out]       proPorts curve data
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YawRate2Curve(const VED_InternalData_t *intData,
                              const proVEDPrtList_t *proPorts) {
    float32 invVelo;

    if (VED_ModIf.LongMot.VehVelocityCorr > C_F32_DELTA) {
        invVelo = 1.F / VED_ModIf.LongMot.VehVelocityCorr;
    } else {
        invVelo = 0.F;
    }

    proPorts->pVED_EstCurves->uiVersionNumber = VED_EST_CURVES_INTFVER;

    /* whl speed curve */
    proPorts->pVED_EstCurves->Whl.Curve =
        intData->ved__wye_out.whl_yaw_rate * invVelo;

    /* whl speed curve */
    proPorts->pVED_EstCurves->WhlFr.Curve =
        intData->ved__wye_out.front_whl_yaw_rate_filt_wld * invVelo;

    /* whl speed curve */
    proPorts->pVED_EstCurves->WhlRe.Curve =
        intData->ved__wye_out.rear_whl_yaw_rate_filt * invVelo;

    /* whl speed curve */
    proPorts->pVED_EstCurves->Whl.Curve =
        intData->ved__wye_out.whl_yaw_rate * invVelo;

    /* yaw rate curve */
    proPorts->pVED_EstCurves->YwRate.Curve =
        intData->ved__gye_out.gier_yaw_rate * invVelo;

    /* lateral acceleration curve */
    proPorts->pVED_EstCurves->Ay.Curve =
        intData->ved__aye_out.ay_yaw_rate * invVelo;

    /* steering wheel angle curve */
    proPorts->pVED_EstCurves->Swa.Curve =
        intData->ved__sye_out.stw_yaw_rate * invVelo;

    /* vehicle yaw rate curve  */
    proPorts->pVED_EstCurves->VehYaw.Curve =
        intData->ved__ye_out.veh_yaw_rate * invVelo;

    /* driver intented curve */
    proPorts->pVED_EstCurves->DrvInt.Curve = intData->ved__sye_out.stw_curve;

    return;
}

#if defined(VED__SIMU)
#if (CFG_VED__MOT_STATE)
/* ***********************************************************************
  @fn               VED_GetVED_BayesOut */ /*!
  @brief            Get pointer to estimated curves 

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           ved__bayes_mot_states_t pointer to curve data

  @pre              -
  @post             -
**************************************************************************** */
ved__bayes_mot_states_t *VED_GetVED_BayesOut(void) {
    return (&ved__bayes_mot_states);
}
#endif

/* ***********************************************************************
  @fn               VED_GetVED_IntData */ /*!

  @brief            Get pointer to internal data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           VED_InternalData_t pointer to internal data

  @pre              -
  @post             -
**************************************************************************** */
VED_InternalData_t *VED_GetVED_IntData(void) { return (&ved__internal_data); }

#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
/* ***********************************************************************
  @fn               VED_Get_D_Work_ved__wpp */ /*!

  @brief            Get pointer to estimated ved__wpp internal data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           D_Work_ved__wpp pointer to wheel speed data

  @pre              -
  @post             -
**************************************************************************** */
D_Work_ved__wpp *VED_Get_D_Work_ved__wpp(void) { return (&ved__wpp_DWork); }

/* ***********************************************************************
  @fn               VED_Get_D_Work_ved__ve */ /*!

  @brief            Get pointer to estimated ved__ve internal data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           D_Work_ved__ve pointer to yaw rate fusion data

  @pre              -
  @post             -
**************************************************************************** */
D_Work_ved__ve *VED_Get_D_Work_ved__ve(void) { return (&ved__ve_DWork); }

/* ***********************************************************************
  @fn               VED_Get_D_Work_ddy_wye */ /*!

  @brief            Get pointer to estimated ved__wye internal data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           D_Work_ved__wye pointer to wheel speed yaw rate data

  @pre              -
  @post             -
**************************************************************************** */
D_Work_ved__wye *VED_Get_D_Work_ved__wye(void) { return (&ved__wye_DWork); }
#endif

#if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
/* ***********************************************************************
  @fn               VED_Get_D_Work_ved__gye */ /*!

  @brief            Get pointer to estimated ved__gye internal data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           D_Work_ved__gye pointer to gyro yaw rate data

  @pre              -
  @post             -
**************************************************************************** */
D_Work_ved__gye *VED_Get_D_Work_ved__gye(void) { return (&ved__gye_DWork); }
#endif

#if (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)
/* ***********************************************************************
  @fn               VED_Get_D_Work_ved__gye_filt */ /*!

  @brief            Get pointer to estimated ved__gye_filt internal data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           D_Work_ved__wpp pointer to wheel speed data

  @pre              -
  @post             -
**************************************************************************** */
D_Work_ved__gye *VED_Get_D_Work_ved__gye_filt(void) {
    return (&ved__gye_DWork_FILT);
}
#endif

#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
/* ***********************************************************************
  @fn               VED_Get_D_Work_ved__aye */ /*!

  @brief            Get pointer to estimated ved__aye internal data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           D_Work_ved__aye pointer to lar accel yaw rate data

  @pre              -
  @post             -
**************************************************************************** */
D_Work_ved__aye *VED_Get_D_Work_ved__aye(void) { return (&ved__aye_DWork); }
#endif

#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
/* ***********************************************************************
  @fn               VED_Get_D_Work_ved__sye */ /*!

  @brief            Get pointer to estimated ved__sye internal data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           D_Work_ved__sye pointer to steering wheel yaw rate data

  @pre              -
  @post             -
**************************************************************************** */
D_Work_ved__sye *VED_Get_D_Work_ved__sye(void) { return (&ved__sye_DWork); }
#endif

#if ((!defined(CFG_VED__USE_EXT_PROC_YAW_RATE)) || \
     (!CFG_VED__USE_EXT_PROC_YAW_RATE))
/* ***********************************************************************
  @fn               VED_Get_D_Work_ved__ye */ /*!

  @brief            Get pointer to estimated ved__ye internal data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           D_Work_ved__ye pointer to yaw rate and curve fusion data

  @pre              -
  @post             -
**************************************************************************** */
D_Work_ved__ye *VED_Get_D_Work_ved__ye(void) { return (&ved__ye_DWork); }
#endif

#if ((!defined(CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION)) || \
     (!CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION))
/* ***********************************************************************
  @fn               VED_Get_D_Work_ved__sae */ /*!

  @brief            Get pointer to estimated ved__sae internal data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           D_Work_ved__sae pointer to slip angle data

  @pre              -
  @post             -
**************************************************************************** */
D_Work_ved__sae *VED_Get_D_Work_ved__sae(void) { return (&ved__sae_DWork); }
#endif

#if (CFG_VED__MOT_STATE)
/* ***********************************************************************
  @fn               VED_Get_D_Work_ved__mot_st */ /*!
  @brief            Get pointer to estimated ved__sae internal data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           D_Work_ved__mot_st pointer to motion state data

  @pre              -
  @post             -
**************************************************************************** */
D_Work_ved__mot_st *VED_Get_D_Work_ved__mot_st(void) {
    return (&ved__mot_st_DWork);
}
#endif

/* ***********************************************************************
  @fn               VED_GetModulIf */ /*!

  @brief            Get pointer to interface data of the internal modules

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           VED_ModIf_t pointer to interface data

  @pre              -
  @post             -
**************************************************************************** */
VED_ModIf_t *VED_GetModulIf(void) { return (&VED_ModIf); }

/* ***********************************************************************
  @fn               VED_SetSequenceInitState */ /*!

  @brief            Set the sequence initalization state

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           VED_SequenceInitStates_t pointer to sequence init data

  @pre              -
  @post             -
**************************************************************************** */
void VED_SetSequenceInitState(VED_SequenceInitStates_t _sSequenceInitState) {
    /* Set the sequence init state to the local variable */
    sSequenceInitState = _sSequenceInitState;
}

#endif

#if (defined(VED__FREEZE_INIT_SEQUENCE) && (VED__FREEZE_INIT_SEQUENCE))
#define TempBufferSize 600U
#define HeaderSize 16U
/* ***********************************************************************
  @fn               VED_InitSequenceMeasFreeze */ /*!

  @brief            Do Init sequence meas freeze

  @description      Freezes all internal data, one module every time
                    Cycles through the modules until all data is freezed
                    This function is only active in the ECU code and 
                    used to sync the simulation to the measurement

  @param[in]        Services
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_InitSequenceMeasFreeze(void) {
    //     static uint8 VED_InitSeqence[TempBufferSize];
    //     uint32 nSize = 0U;
    //     uint32 nRawSize = 0U;
    //     uint32 nPos = 0U;

    //     //  (void)Services;  /* to remove compiler warning, service function
    //     is not
    //     //  used in all configurations */

    //     switch (InitPackageID) {
    //         case 0:
    // #if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
//      (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    //             /* the raw size of the data package without the
    //             header(id+size)*/ nRawSize = sizeof(ved__wpp_DWork); if
    //             (nRawSize < (uint32)(TempBufferSize - HeaderSize)) {
    //                 /* Copy data of sequence InitPackageID */
    //                 (void)memcpy(VED_InitSeqence, &InitPackageID,
    //                              sizeof(InitPackageID));
    //                 nPos = nPos + (uint32)sizeof(InitPackageID);
    //                 (void)memcpy(&VED_InitSeqence[nPos], &nRawSize,
    //                              sizeof(nRawSize));
    //                 nPos = nPos + (uint32)sizeof(nRawSize);
    //                 (void)memcpy(&VED_InitSeqence[nPos], &ved__wpp_DWork,
    //                              sizeof(ved__wpp_DWork));
    //                 nPos = nPos + (uint32)sizeof(ved__wpp_DWork);

    //                 // /* Set up the total size of init package */
    //                 // nSize = nPos;
    //                 // /* Freeze the sequence init package */
    //                 // MEAS_FREEZE_DIRECT(VED__MEAS_ID_INIT_SEQ,
    //                 VED_InitSeqence,
    //                 //                    nSize);

    //             } else {
    //                 /* set error */
    //             }
    // #endif
    //             InitPackageID++;
    //             break;

    //         case 1:
    // #if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
//      (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    //             /* the raw size of the data package without the
    //             header(id+size)*/ nRawSize = sizeof(ved__ve_DWork); if
    //             (nRawSize < (uint32)(TempBufferSize - HeaderSize)) {
    //                 /* Copy data of sequence InitPackageID */
    //                 (void)memcpy(VED_InitSeqence, &InitPackageID,
    //                              sizeof(InitPackageID));
    //                 nPos = nPos + (uint32)sizeof(InitPackageID);
    //                 (void)memcpy(&VED_InitSeqence[nPos], &nRawSize,
    //                              sizeof(nRawSize));
    //                 nPos = nPos + (uint32)sizeof(nRawSize);
    //                 (void)memcpy(&VED_InitSeqence[nPos], &ved__ve_DWork,
    //                              sizeof(ved__ve_DWork));
    //                 nPos = nPos + (uint32)sizeof(ved__ve_DWork);

    //                 /* Set up the total size of init package */
    //                 nSize = nPos;
    //                 /* Freeze the sequence init package */
    //                 MEAS_FREEZE_DIRECT(VED__MEAS_ID_INIT_SEQ,
    //                 VED_InitSeqence,
    //                                    nSize);

    //             } else {
    //                 /* set error */
    //             }
    // #endif
    //             InitPackageID++;
    //             break;

    //         case 2:
    // #if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
//      (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    //             /* the raw size of the data package without the
    //             header(id+size)*/ nRawSize = sizeof(ved__wye_DWork); if
    //             (nRawSize < (uint32)(TempBufferSize - HeaderSize)) {
    //                 /* Copy data of sequence InitPackageID */
    //                 (void)memcpy(VED_InitSeqence, &InitPackageID,
    //                              sizeof(InitPackageID));
    //                 nPos = nPos + (uint32)sizeof(InitPackageID);
    //                 (void)memcpy(&VED_InitSeqence[nPos], &nRawSize,
    //                              sizeof(nRawSize));
    //                 nPos = nPos + (uint32)sizeof(nRawSize);
    //                 (void)memcpy(&VED_InitSeqence[nPos], &ved__wye_DWork,
    //                              sizeof(ved__wye_DWork));
    //                 nPos = nPos + (uint32)sizeof(ved__wye_DWork);

    //                 /* Set up the total size of init package */
    //                 nSize = nPos;
    //                 /* Freeze the sequence init package */
    //                 MEAS_FREEZE_DIRECT(VED__MEAS_ID_INIT_SEQ,
    //                 VED_InitSeqence,
    //                                    nSize);

    //             } else {
    //                 /* set error */
    //             }
    // #endif
    //             InitPackageID++;
    //             break;

    //         case 3:
    // #if ((!defined(CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING)) || \
//      (!CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING))
    //             /* the raw size of the data package without the
    //             header(id+size)*/ nRawSize = (uint32)sizeof(ved__gye_DWork);
    // #if ((!defined(CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)) || \
//      (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING))
    //             nRawSize = nRawSize + (uint32)sizeof(ved__gye_DWork_FILT);
    // #endif
    //             if (nRawSize < (uint32)(TempBufferSize - HeaderSize)) {
    //                 /* Copy data of sequence InitPackageID */
    //                 (void)memcpy(VED_InitSeqence, &InitPackageID,
    //                              sizeof(InitPackageID));
    //                 nPos = nPos + (uint32)sizeof(InitPackageID);
    //                 (void)memcpy(&VED_InitSeqence[nPos], &nRawSize,
    //                              sizeof(nRawSize));
    //                 nPos = nPos + (uint32)sizeof(nRawSize);
    //                 (void)memcpy(&VED_InitSeqence[nPos], &ved__gye_DWork,
    //                              sizeof(ved__gye_DWork));
    //                 nPos = nPos + (uint32)sizeof(ved__gye_DWork);
    // #if ((!defined(CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING)) || \
//      (!CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING))
    //                 (void)memcpy(&VED_InitSeqence[nPos],
    //                 &ved__gye_DWork_FILT,
    //                              sizeof(ved__gye_DWork_FILT));
    //                 nPos = nPos + (uint32)sizeof(ved__gye_DWork_FILT);
    // #endif

    //                 /* Set up the total size of init package */
    //                 nSize = nPos;

    //                 /* Freeze the sequence init package if package size is
    //                 smaller
    //                  * than buffer*/
    //                 MEAS_FREEZE_DIRECT(VED__MEAS_ID_INIT_SEQ,
    //                 VED_InitSeqence,
    //                                    nSize);

    //             } else {
    //                 /* set error */
    //             }
    // #endif
    //             InitPackageID++;
    //             break;

    //         case 4:
    // #if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
//      (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
    // /* the raw size of the data package without the header(id+size)*/
    // nRawSize = sizeof(ved__aye_DWork);
    // if (nRawSize < (uint32)(TempBufferSize - HeaderSize)) {
    //     /* Copy data of sequence InitPackageID */
    //     (void)memcpy(VED_InitSeqence, &InitPackageID,
    //                  sizeof(InitPackageID));
    //     nPos = nPos + (uint32)sizeof(InitPackageID);
    //     (void)memcpy(&VED_InitSeqence[nPos], &nRawSize,
    //                  sizeof(nRawSize));
    //     nPos = nPos + (uint32)sizeof(nRawSize);
    //     (void)memcpy(&VED_InitSeqence[nPos], &ved__aye_DWork,
    //                  sizeof(ved__aye_DWork));
    //     nPos = nPos + (uint32)sizeof(ved__aye_DWork);

    //     /* Set up the total size of init package */
    //     nSize = nPos;
    //     /* Freeze the sequence init package */
    //     MEAS_FREEZE_DIRECT(VED__MEAS_ID_INIT_SEQ, VED_InitSeqence,
    //                        nSize);

    //             } else {
    //                 /* set error */
    //             }
    // #endif
    //             InitPackageID++;
    //             break;

    //         case 5:
    // #if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
//      (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
    // /* the raw size of the data package without the header(id+size)*/
    // nRawSize = sizeof(ved__sye_DWork);
    // if (nRawSize < (uint32)(TempBufferSize - HeaderSize)) {
    //     /* Copy data of sequence InitPackageID */
    //     (void)memcpy(VED_InitSeqence, &InitPackageID,
    //                  sizeof(InitPackageID));
    //     nPos = nPos + (uint32)sizeof(InitPackageID);
    //     (void)memcpy(&VED_InitSeqence[nPos], &nRawSize,
    //                  sizeof(nRawSize));
    //     nPos = nPos + (uint32)sizeof(nRawSize);
    //     (void)memcpy(&VED_InitSeqence[nPos], &ved__sye_DWork,
    //                  sizeof(ved__sye_DWork));
    //     nPos = nPos + (uint32)sizeof(ved__sye_DWork);

    //     /* Set up the total size of init package */
    //     nSize = nPos;
    //     /* Freeze the sequence init package */
    //     MEAS_FREEZE_DIRECT(VED__MEAS_ID_INIT_SEQ, VED_InitSeqence,
    //                        nSize);

    //             } else {
    //                 /* set error */
    //             }
    // #endif
    //             InitPackageID++;
    //             break;

    //         case 6:
    // #if ((!defined(CFG_VED__USE_EXT_PROC_YAW_RATE)) || \
//      (!CFG_VED__USE_EXT_PROC_YAW_RATE))
    // /* the raw size of the data package without the header(id+size)*/
    // nRawSize = sizeof(ved__ye_DWork);
    // if (nRawSize < (uint32)(TempBufferSize - HeaderSize)) {
    //     /* Copy data of sequence InitPackageID */
    //     (void)memcpy(VED_InitSeqence, &InitPackageID,
    //                  sizeof(InitPackageID));
    //     nPos = nPos + (uint32)sizeof(InitPackageID);
    //     (void)memcpy(&VED_InitSeqence[nPos], &nRawSize,
    //                  sizeof(nRawSize));
    //     nPos = nPos + (uint32)sizeof(nRawSize);
    //     (void)memcpy(&VED_InitSeqence[nPos], &ved__ye_DWork,
    //                  sizeof(ved__ye_DWork));
    //     nPos = nPos + (uint32)sizeof(ved__ye_DWork);

    //     /* Set up the total size of init package */
    //     nSize = nPos;
    //     /* Freeze the sequence init package */
    //     MEAS_FREEZE_DIRECT(VED__MEAS_ID_INIT_SEQ, VED_InitSeqence,
    //                        nSize);

    //             } else {
    //                 /* set error */
    //             }
    // #endif
    //             InitPackageID++;
    //             break;

    //         case 7:
    // #if ((!defined(CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION)) || \
//      (!CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION))
    // /* the raw size of the data package without the header(id+size)*/
    // nRawSize = sizeof(ved__sae_DWork);
    // if (nRawSize < (uint32)(TempBufferSize - HeaderSize)) {
    //     /* Copy data of sequence InitPackageID */
    //     (void)memcpy(VED_InitSeqence, &InitPackageID,
    //                  sizeof(InitPackageID));
    //     nPos = nPos + (uint32)sizeof(InitPackageID);
    //     (void)memcpy(&VED_InitSeqence[nPos], &nRawSize,
    //                  sizeof(nRawSize));
    //     nPos = nPos + (uint32)sizeof(nRawSize);
    //     (void)memcpy(&VED_InitSeqence[nPos], &ved__sae_DWork,
    //                  sizeof(ved__sae_DWork));
    //     nPos = nPos + (uint32)sizeof(ved__sae_DWork);

    //     /* Set up the total size of init package */
    //     nSize = nPos;
    //     /* Freeze the sequence init package */
    //     MEAS_FREEZE_DIRECT(VED__MEAS_ID_INIT_SEQ, VED_InitSeqence,
    //                        nSize);

    //             } else {
    //                 /* set error */
    //             }
    // #endif
    //             InitPackageID++;
    //             break;

    //         case 8:
    // #if (CFG_VED__MOT_STATE)
    // /* the raw size of the data package without the header(id+size)*/
    // nRawSize = (uint32)sizeof(ved__mot_st_DWork);
    // if (nRawSize < (uint32)(TempBufferSize - HeaderSize)) {
    //     /* Copy data of sequence InitPackageID */
    //     (void)memcpy(VED_InitSeqence, &InitPackageID,
    //                  sizeof(InitPackageID));
    //     nPos = nPos + (uint32)sizeof(InitPackageID);
    //     (void)memcpy(&VED_InitSeqence[nPos], &nRawSize,
    //                  sizeof(nRawSize));
    //     nPos = nPos + (uint32)sizeof(nRawSize);
    //     (void)memcpy(&VED_InitSeqence[nPos], &ved__mot_st_DWork,
    //                  sizeof(ved__mot_st_DWork));
    //     nPos = nPos + (uint32)sizeof(ved__mot_st_DWork);

    // /* Set up the total size of init package */
    // nSize = nPos;
    // /* Freeze the sequence init package */
    // MEAS_FREEZE_DIRECT(VED__MEAS_ID_INIT_SEQ, VED_InitSeqence,
    //                    nSize);

    //             } else {
    //                 /* set error */
    //             }
    // #endif
    //             InitPackageID++;
    //             break;

    //         case 14:
    //             InitPackageID = 0U;
    //             break;

    //         case 9:  /* fallthrough: same as default */
    //         case 10: /* fallthrough: same as default */
    //         case 11: /* fallthrough: same as default */
    //         case 12: /* fallthrough: same as default */
    //         case 13: /* fallthrough: same as default */
    //         default:
    //             InitPackageID++;
    //             break;
    //     }
}
#endif

/* *************************************************************************
  @fn               VED_SetSyncFrame */ /*!
  @brief            Fill VED MTS sync frame

  @description      Copies the content of the signal headers of all inputs
                    (required ports) to the sync header data structure
                    and freezes the sync frame, then freeze the SIM frame

  @param[in]        Services for VED_AS_t_ServiceFuncts meas freeze function
  @param[in]        CycleCnt
  @param[in]        eVED_SigStatus
  @param[out]       reqPorts reqVEDPrtList_t sync frame
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_SetSyncFrame(const reqVEDPrtList_t *reqPorts,
                             const reqVEDParams_t *reqParams,
                             uint16 CycleCnt,
                             uint8 eVED_SigStatus) {
    s_SyncRef.BSW_s_VED_CtrlData = reqPorts->pCtrl->sSigHeader;
    // s_SyncRef.VehPar              = reqParams->pVehicleParameter->sSigHeader;
    s_SyncRef.VehSig = reqPorts->pVehicleInputSignals->sSigHeader;
    s_SyncRef.VED_NVMRead = reqPorts->pNVMRead->sSigHeader;

#if ((defined(CFG_VED__DO_VELOCITY_CORR)) && (CFG_VED__DO_VELOCITY_CORR))
#if ((defined(VEL_CORR_HIST_STATIONARY_TARGETS)) && \
     (VEL_CORR_HIST_STATIONARY_TARGETS))
    s_SyncRef.VelStatObj;
    = reqPorts->pVelStatObj->sSigHeader; /* Reference speed histogramm       */
#endif
#if ((defined(VEL_CORR_ALN)) && (VEL_CORR_ALN))
    s_SyncRef.ALN_Monitoring =
        reqPorts->pAln_Monitoring
            ->sSigHeader; /* Velocity information from ALN    */
#endif
#endif
#if ((defined(CFG_VED__ROLLBENCH_DETECTION)) && (CFG_VED__ROLLBENCH_DETECTION))
    s_SyncRef.EM_RTBRecognition =
        reqPorts->pRTBRecognition
            ->sSigHeader; /* Roll Bench detection information    */
#endif
#if ((defined(CFG_VED__64BIT_TIMESTAMP_INTERV)) && \
     (CFG_VED__64BIT_TIMESTAMP_INTERV))
    s_SyncRef.VED_LongLongTimeStamp = reqPorts->pLongLongTimeStamp->sSigHeader;
    ;
#endif

    s_SyncRef.sSigHeader.uiTimeStamp =
        reqPorts->pVehicleInputSignals->sSigHeader.uiTimeStamp;
    s_SyncRef.sSigHeader.uiMeasurementCounter =
        reqPorts->pVehicleInputSignals->sSigHeader.uiMeasurementCounter;
    s_SyncRef.sSigHeader.eSigStatus = eVED_SigStatus;
    s_SyncRef.sSigHeader.uiCycleCounter = CycleCnt;
}

/* *************************************************************************
  @fn             VED_CheckPorts */ /*!
  @brief          Check input and output ports for valid pointer

  @description    Check all requested, provided and servive ports for NULL pointer
                  and set a fault if any one contains a NULL pointer

  @param[in]      reqPorts : the inputs passed to VED
  @param[in]      proPorts : the outputs of VED
*/
/*!
  @param[in]      Services : service functions used by VED
*/
/*!
  @param[out]     -
  @return         Fault status

  @pre            -
  @post           -

**************************************************************************** */
static uint8 VED_CheckPorts(const reqVEDPrtList_t *reqPorts,
                            const reqVEDParams_t *reqParams,
                            const proVEDPrtList_t *proPorts) {
    uint8 ui8Fault = VED__NO_PORT_FAULT;

    /* Check all ports for NULL pointer */
    /* all request ports */
    if ((reqPorts == NULL) ||
        (reqPorts->pCtrl ==
         NULL) /* operation mode, calibration mode and cyle time*/
        || (reqParams == NULL) /* VehicleParameter data            */
        || (reqPorts->pVehicleInputSignals == NULL) /* Vehicle input signals */
        || (reqPorts->pNVMRead == NULL) /* VED NVM read data                */
#if ((defined(CFG_VED__DO_VELOCITY_CORR)) && (CFG_VED__DO_VELOCITY_CORR))
#if ((defined(VEL_CORR_HIST_STATIONARY_TARGETS)) && \
     (VEL_CORR_HIST_STATIONARY_TARGETS))
        || (reqPorts->pVelStatObj == NULL) /* Reference speed histogramm */
#endif
#if ((defined(VEL_CORR_ALN)) && (VEL_CORR_ALN))
        ||
        (reqPorts->pAln_Monitoring == NULL) /* Velocity information from ALN */
#endif
#endif
#if ((defined(CFG_VED__ROLLBENCH_DETECTION)) && (CFG_VED__ROLLBENCH_DETECTION))
        || (reqPorts->pRTBRecognition == NULL) /* RTB information    */
#endif
#if ((defined(CFG_VED__ALIGNMENT_OFFSET_MONITOR)) && \
     (CFG_VED__ALIGNMENT_OFFSET_MONITOR))
        || (reqPorts->pAlignEstYawRateOffset ==
            NULL) /* Yaw rate offset estimated by the alignment */
#endif
#if ((defined(CFG_VED__64BIT_TIMESTAMP_INTERV)) && \
     (CFG_VED__64BIT_TIMESTAMP_INTERV))
        || (reqPorts->pLongLongTimeStamp == NULL)
#endif
    ) {
        ui8Fault |= VED__REQUEST_PORT_FAULT;
    } else {
        /* all request ports available, no fault */
    }

    /* all provided ports */
    if ((proPorts == NULL) ||
        (proPorts->pVehicleDynamicSignals == NULL) /* Vehicle Dynamic signals */
#if ((!defined(CFG_VED__DIS_YAW_SENSOR_OUTPUT)) || \
     (!CFG_VED__DIS_YAW_SENSOR_OUTPUT))
        || (proPorts->pALNYawRate == NULL) /* The yaw rate for alignment only
                                              yaw rate from yaw rate sensro */
#endif
#if (CFG_VED__INT_GYRO)
        || (proPorts->pYwrtTempTable == NULL) /* NvYawrate temp table */
#endif
        || (proPorts->pNVMWrite == NULL) /* VED NVM write data          */
        ||
        (proPorts->pVED_Errors ==
         NULL) /* ved_ errors, input signals/parameters and internal errors */
        || (proPorts->pVED_Offsets ==
            NULL) /* ved_ offsets, for yaw rate steering wheel angle and lateral
                     acceleration sensor */
#if ((defined(CFG_VED__64BIT_TIMESTAMP_INTERV)) && \
     (CFG_VED__64BIT_TIMESTAMP_INTERV))
        || (proPorts->pLongLongTimeStamp == NULL)
#endif
        || (proPorts->pVED_EstCurves == NULL) /* VED estimated curves       */
    ) {
        ui8Fault |= VED__PROVIDE_PORT_FAULT;
    } else {
        /* all provide ports available, no fault */
    }

    return ui8Fault;
}

/* *************************************************************************
  @fn             VED_CalculateCurveError */ /*!
  @brief          Calculate lateral error of the curve

  @description    The offset learning states of ay, swa, gye and wye are
                  transformed into a yaw rate offset error (the error gets
                  less with higher states).
                  Then these remaining yaw rate errors are calculated into a
                  seperate curvature error for each input.  These individually
                  curvature errors are then added into a single curvature
                  error using the Kalman gain of the ye model as weight.
                  The ye yaw rate error and the swa yae rate error is then
                  calculated into a curve error (again using the Kalman gain
                  of the cuve fusion as weight).

  @param[in]      ved__ye_k:          the Kalman gains of YE model
  @param[in]      ved__internal_data: velocity
  @param[in]      VED_In:             input parameters
  @param[in]      offset:            offset stati
  @param[out]     -
  @return         curve error

  @pre            -
  @post           -

**************************************************************************** */
static float32 VED_CalculateCurveError(const VED_InputData_t *input,
                                       const VED_VEDOffsets_t *pVED_Offsets) {
    /* steering angle */
    float32 swa_offset_error; /* offset error */
    float32 sye_error;        /* yaw rate error */

    /* gyro */
    float32 yaw_offset_error; /* offset error */
    float32 gye_error;        /* yaw rate error */

    /* lateral acceleration */
    float32 ay_offset_error; /* offset error */
    float32 aye_error;       /* yaw rate error */

    /* wheels */
#if ((CFG_VED__DIS_WHS_OFFSET_COMP == 0) && \
     (CFG_VED__DIS_WHEEL_PRE_PROCESSING == 0))
    float32 wye_error_front; /* yaw rate error front axle*/
    float32 wye_error_rear;  /* yaw rate error rear axle*/
#endif
    float32 wye_error; /* yaw rate error */

    /* combined yaw rate */
    float32 ye_error; /* offset error */
    float32 k_yaw_sye_part, k_yaw_gye_part, k_yaw_aye_part, k_yaw_wye_part,
        k_yaw_sum; /* percentage of input yaw rates in combined yaw rate */

    /* curve */
    float32 sye_curve_error; /* error of steering angle curve  */
    float32 ye_curve_error;  /* error of yaw rate curve  */
    float32 curve_error;     /* resulting curve error */
    float32 k_curve_sye_part, k_curve_ye_part,
        k_curve_sum; /* percentage of input curves in combined curve */

    /* get steering wheel error based on offset deviation */
#if ((CFG_VED__DIS_SWA_OFFSET_COMP == 0) && \
     (CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING == 0))
    {
        VED_SwaData_t *pSwaData = VED_SwaGetPrivateData();
        swa_offset_error = pSwaData->Offset.Dev / 3.0F;
    }
#else
    swa_offset_error = 0.0F;
#endif

    (void)pVED_Offsets;

    /* get gyro error based on offset learn state */
#if (CFG_VED__DIS_YWR_OFFSET_COMP == 0)
    /*!< No offset type present                   */
    if (pVED_Offsets->Ywr.StandStillState == OUT_YWR_OFFS_NON) {
        yaw_offset_error = YWR_OFFSET_ERROR_OUT_YWR_OFFS_NON;
    }
    /*!< Offset short acquired from stand still   */
    else if (pVED_Offsets->Ywr.StandStillState == OUT_YWR_OFFS_STANDST_SHORT) {
        yaw_offset_error = YWR_OFFSET_ERROR_OUT_YWR_OFFS_STANDST_SHORT;
    }
    /* if already in the dynamic offset phase, use dynamic offset accuracy */
    else if (pVED_Offsets->Ywr.StandStillState == OUT_YWR_OFFS_DYN) {
        yaw_offset_error = YWR_OFFSET_ERROR_OUT_YWR_OFFS_DYN;
    }
    /*!< Offset full acquired from stand still    */
    else if (pVED_Offsets->Ywr.StandStillState == OUT_YWR_OFFS_STANDST_FULL) {
        // if the wheel offsets are calculated, we are also already in the
        // dynamic yaw rate offset phase
        if ((ved__internal_data.ved__offsets_in.ved__whs_offset
                 .offset_ratio_front_dev >= 1.0F) ||
            (ved__internal_data.ved__offsets_in.ved__whs_offset
                 .offset_ratio_rear_dev >= 1.0F)) {
            yaw_offset_error = YWR_OFFSET_ERROR_OUT_YWR_OFFS_STANDST_FULL;
        } else {
            yaw_offset_error = YWR_OFFSET_ERROR_OUT_YWR_OFFS_DYN;
        }
    }
    /*!< Offset acquired from non-volatile memory */
    else if (pVED_Offsets->Ywr.StandStillState == OUT_YWR_OFFS_STANDST_EEPROM) {
#if ((defined(CFG_VED__REDUCE_CURVE_ERROR)) && (CFG_VED__REDUCE_CURVE_ERROR))
        // Change the yaw offset error from EEPROM to Dynamic to reduce the
        // curve error when the vehicle is moving and threshold time is elapsed
        if ((VED_ModIf.LongMot.MotState.MotState != VED_LONG_MOT_STATE_STDST) &&
            (u_count > TIME_EEPROM_TO_DYNAMIC)) {
            yaw_offset_error = YWR_OFFSET_ERROR_OUT_YWR_OFFS_DYN;
        } else {
            if (u_count > TIME_EEPROM_TO_DYNAMIC) {
                u_count = 0;
            }
            yaw_offset_error = YWR_OFFSET_ERROR_OUT_YWR_OFFS_STANDST_EEPROM;
            u_count++;
        }
#else
        yaw_offset_error = YWR_OFFSET_ERROR_OUT_YWR_OFFS_STANDST_EEPROM;

#endif
    }
    /* invalid state information, assume max. error */
    else {
        yaw_offset_error = YWR_OFFSET_ERROR_OUT_YWR_OFFS_NON;
    }
#else
    /* yaw rate input already compensated, assume low error  */
    yaw_offset_error = YWR_OFFSET_ERROR_OUT_YWR_OFFS_DYN;
#endif

    /* get lateral acceleration error based on offset deviation */
#if ((CFG_VED__DIS_LAT_OFFSET_COMP == 0) && \
     (CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING == 0))
    {
        VED_AyData_t *pAyData = VED_AyGetPrivateData();
        ay_offset_error = pAyData->Offset.Dev / 3.0F;
    }
#else
    ay_offset_error = 0.0F;
#endif

    /* calculate yaw errors based on variance and offset error */
    sye_error = (1.0F / input->Parameter.SteeringRatio.swa.rat[1]) *
                ((swa_offset_error /
                  (input->Parameter.VED_Kf_WheelBase_met +
                   (input->Parameter.VED_Kf_SelfSteerGrad_nu *
                    TUE_CML_Sqr(ved__internal_data.ved__ve_out.veh_velo)))) *
                 ved__internal_data.ved__ve_out.veh_velo);
    gye_error = yaw_offset_error; /* gye error is already a yaw rate error, no
                                     further calculation needed */
    if (ved__internal_data.ved__ve_out.veh_velo > MIN_VELO_YAW_RATE_ERROR) {
        aye_error = ay_offset_error / ved__internal_data.ved__ve_out.veh_velo;
    } else {
        aye_error = 0.0F;
    }

#if ((CFG_VED__DIS_WHS_OFFSET_COMP == 0) && \
     (CFG_VED__DIS_WHEEL_PRE_PROCESSING == 0))
    /* calculate wheel errors based offset error */
    if (TUE_CML_IsNonZero(ved__internal_data.ved__ve_out.veh_velo)) {
        float32 f_wye_temp;

        f_wye_temp =
            (input->Parameter.VED_Kf_TrackWidthFront_met *
             (1.0F +
              ((input->Parameter.VED_Kf_AxisLoadDistr_per) *
               ((input->Parameter.VED_Kf_WhlLoadDepFrontAxle_nu *
                 (input->Parameter.VED_Kf_CntrOfGravHeight_met *
                  (input->Parameter.VED_Kf_VehWeight_kg *
                   TUE_CML_Sqr(ved__internal_data.ved__ve_out.veh_velo)))) /
                ((1000.0F * 1000.0F) *
                 TUE_CML_Sqr(input->Parameter.VED_Kf_TrackWidthFront_met))))));

        if (f_wye_temp > 0.0F) {
            wye_error_front = (ved__internal_data.ved__offsets_in
                                   .ved__whs_offset.offset_ratio_front_dev *
                               ved__internal_data.ved__ve_out.veh_velo) /
                              f_wye_temp;
        } else {
            wye_error_front = 0.0F;
        }

        f_wye_temp =
            (input->Parameter.VED_Kf_TrackWidthRear_met *
             (1.0F +
              (((1.0F - input->Parameter.VED_Kf_AxisLoadDistr_per) *
                ((input->Parameter.VED_Kf_WhlLoadDepRearAxle_nu *
                  (input->Parameter.VED_Kf_CntrOfGravHeight_met *
                   (input->Parameter.VED_Kf_VehWeight_kg *
                    TUE_CML_Sqr(ved__internal_data.ved__ve_out.veh_velo)))) /
                 ((1000.0F * 1000.0F) *
                  TUE_CML_Sqr(input->Parameter.VED_Kf_TrackWidthRear_met)))))));

        if (f_wye_temp > 0.0F) {
            wye_error_rear = (ved__internal_data.ved__offsets_in.ved__whs_offset
                                  .offset_ratio_rear_dev *
                              ved__internal_data.ved__ve_out.veh_velo) /
                             f_wye_temp;
        } else {
            wye_error_rear = 0.0F;
        }
    } else {
        wye_error_front = 0.0F;
        wye_error_rear = 0.0F;
    }

    wye_error = wye_error_front + wye_error_rear;
#else
    /* wheel speed processing deactivated, no error added (will not be used for
     * yaw rate anyway) */
    wye_error = 0;
#endif

    /* calculate combined yaw rate (ye) error considering the Kalman gain */
    k_yaw_sum = ved__ye_k.K_yaw[1] + ved__ye_k.K_yaw[3] + ved__ye_k.K_yaw[5] +
                ved__ye_k.K_yaw[7];
    if (TUE_CML_IsNonZero(k_yaw_sum)) {
        k_yaw_gye_part = ved__ye_k.K_yaw[1] / k_yaw_sum;
        k_yaw_wye_part = ved__ye_k.K_yaw[3] / k_yaw_sum;
        k_yaw_aye_part = ved__ye_k.K_yaw[5] / k_yaw_sum;
        k_yaw_sye_part = ved__ye_k.K_yaw[7] / k_yaw_sum;
        ye_error = (sye_error * k_yaw_sye_part) + (gye_error * k_yaw_gye_part) +
                   (aye_error * k_yaw_aye_part) + (wye_error * k_yaw_wye_part);
    } else {
        ye_error = 0.0F;
    }

    /* calculate combined yaw rate (ye) curve error */
    if (ved__internal_data.ved__ve_out.veh_velo > MIN_VELO_YAW_RATE_ERROR) {
        ye_curve_error = ye_error / (ved__internal_data.ved__ve_out.veh_velo);
    } else {
        ye_curve_error = 0.0F;
    }

    /* calculate steering angle (sye) curve error */
    sye_curve_error =
        (1.0F / input->Parameter.SteeringRatio.swa.rat[1]) *
        (swa_offset_error /
         (input->Parameter.VED_Kf_WheelBase_met +
          (input->Parameter.VED_Kf_SelfSteerGrad_nu *
           TUE_CML_Sqr(ved__internal_data.ved__ve_out.veh_velo))));

    /* calculate combined curve error considering the Kalman gain */
    k_curve_sum = ved__ye_k.K_curve[2] + ved__ye_k.K_curve[0];
    if (TUE_CML_IsNonZero(k_curve_sum)) {
        k_curve_ye_part = ved__ye_k.K_curve[0] / k_curve_sum;
        k_curve_sye_part = ved__ye_k.K_curve[2] / k_curve_sum;
        curve_error = (sye_curve_error * k_curve_sye_part) +
                      (ye_curve_error * k_curve_ye_part);
    } else {
        curve_error = 0.0F;
    }
    return curve_error;
}

/* *************************************************************************
  @fn             VED_CalculateLaneErrorConf */ /*!
  @brief          Calculate lateral error and confidence of curve

  @description    The lateral error of the curve output is calculated by
                  using the function VED_CalculateCurveError.
                  The confidence is set considering the states of the input
                  signals and the lateral curve error at certain distances
                  (predicted over 1, 2, 3 and 4 seconds).
                  The lateral error must be less than 1.75m at the given
                  distances (higher distance gives a higher confidence).
                  The input states are debounced for 100ms.  Reducing
                  the confidence due to lower distance is delayed by 400ms
                  to avoid a toggling confidence.

  @param[in]      ved__ye_k the Kalman gains of YE model
  @param[in]      VED_In    input signal states
  @param[in]      offset   offset stati
  @param[out]     pCurve   curve error and confidence
  @return         void

  @pre            -
  @post           -

**************************************************************************** */
static void VED_CalculateLaneErrorConf(const reqVEDPrtList_t *reqPorts,
                                       const VED_InputData_t *input,
                                       const VED_VEDOffsets_t *pVED_Offsets,
                                       VEDVehDyn_t *pVehDyn) {
    /* counter and flags for input signals */
    static uint8 u_all_wheels_qual_not_avail_counter = 0;
    static boolean b_all_wheels_qual_not_avail = FALSE;

    static uint8 u_yaw_qual_not_avail_counter = 0;
    static boolean b_yaw_qual_not_avail = FALSE;

    static uint8 u_swa_qual_not_avail_counter = 0;
    static boolean b_swa_qual_not_avail = FALSE;

    static uint8 u_esp_qual_not_avail_counter = 0;
    static boolean b_esp_qual_not_avail = FALSE;

    static uint8 u_yaw_qual_not_def_counter = 0;
    static boolean b_yaw_qual_not_def = FALSE;

    static uint8 u_swa_qual_low_precision_counter = 0;
    static boolean b_swa_qual_low_precision = FALSE;

    static uint8 u_one_wheel_qual_reduced_counter = 0;
    static boolean b_one_wheel_qual_reduced = FALSE;

    static uint8 u_lat_acc_qual_not_avail_counter = 0;
    static boolean b_lat_acc_qual_not_avail = FALSE;

    float32 f_CurveError; /* error in curve */
    float32 f_LatError;   /* lateral error in curve */

    static uint8 u_conf_counter =
        0; /* counter for hold time (before going to lower confidence) */
    static uint8 u_CurveConf =
        0; /* curve confidence calculated in this cycle */
    static uint8 u_CurveConfOutput =
        0; /* curve confidence output (calculated value might be delayed by hold
              time) */

    /* check if all the wheel speeds are not available for 5 cycles */
    if ((VED_GET_IO_STATE(VED_SIN_POS_WVEL_FL,
                          reqPorts->pVehicleInputSignals->VehSigMain.State) ==
         VED_IO_STATE_INVALID) &&
        (VED_GET_IO_STATE(VED_SIN_POS_WVEL_FR,
                          reqPorts->pVehicleInputSignals->VehSigMain.State) ==
         VED_IO_STATE_INVALID) &&
        (VED_GET_IO_STATE(VED_SIN_POS_WVEL_RL,
                          reqPorts->pVehicleInputSignals->VehSigMain.State) ==
         VED_IO_STATE_INVALID) &&
        (VED_GET_IO_STATE(VED_SIN_POS_WVEL_RR,
                          reqPorts->pVehicleInputSignals->VehSigMain.State) ==
         VED_IO_STATE_INVALID)) {
        if (u_all_wheels_qual_not_avail_counter < MAX_SIGNAL_COUNTER) {
            u_all_wheels_qual_not_avail_counter++;
        } else {
            b_all_wheels_qual_not_avail = TRUE;
        }
    } else {
        u_all_wheels_qual_not_avail_counter = 0;
        b_all_wheels_qual_not_avail = FALSE;
    }

    /* check if just one wheel speed is not available for 5 cycles */
    if ((VED_GET_IO_STATE(VED_SIN_POS_WVEL_FL,
                          reqPorts->pVehicleInputSignals->VehSigMain.State) !=
         VED_IO_STATE_VALID) ||
        (VED_GET_IO_STATE(VED_SIN_POS_WVEL_FR,
                          reqPorts->pVehicleInputSignals->VehSigMain.State) !=
         VED_IO_STATE_VALID) ||
        (VED_GET_IO_STATE(VED_SIN_POS_WVEL_RL,
                          reqPorts->pVehicleInputSignals->VehSigMain.State) !=
         VED_IO_STATE_VALID) ||
        (VED_GET_IO_STATE(VED_SIN_POS_WVEL_RR,
                          reqPorts->pVehicleInputSignals->VehSigMain.State) !=
         VED_IO_STATE_VALID)) {
        if (u_one_wheel_qual_reduced_counter < MAX_SIGNAL_COUNTER) {
            u_one_wheel_qual_reduced_counter++;
        } else {
            b_one_wheel_qual_reduced = TRUE;
        }
    } else {
        u_one_wheel_qual_reduced_counter = 0;
        b_one_wheel_qual_reduced = FALSE;
    }

    /* check if yaw rate is not available for 5 cycles */
    if (VED_GET_IO_STATE(VED_SIN_POS_YWR,
                         reqPorts->pVehicleInputSignals->VehSigMain.State) ==
        VED_IO_STATE_INVALID) {
        if (u_yaw_qual_not_avail_counter < MAX_SIGNAL_COUNTER) {
            u_yaw_qual_not_avail_counter++;
        } else {
            b_yaw_qual_not_avail = TRUE;
        }
    } else {
        u_yaw_qual_not_avail_counter = 0;
        b_yaw_qual_not_avail = FALSE;
    }

    /* check if yaw rate is decreased for 5 cycles */
    if (VED_GET_IO_STATE(VED_SIN_POS_YWR,
                         reqPorts->pVehicleInputSignals->VehSigMain.State) ==
        VED_IO_STATE_DECREASED) {
        if (u_yaw_qual_not_def_counter < MAX_SIGNAL_COUNTER) {
            u_yaw_qual_not_def_counter++;
        } else {
            b_yaw_qual_not_def = TRUE;
        }
    } else {
        u_yaw_qual_not_def_counter = 0;
        b_yaw_qual_not_def = FALSE;
    }

    /* check if steering angle is not available for 5 cycles */
    if (VED_GET_IO_STATE(VED_SIN_POS_SWA,
                         reqPorts->pVehicleInputSignals->VehSigMain.State) ==
        VED_IO_STATE_INVALID) {
        if (u_swa_qual_not_avail_counter < MAX_SIGNAL_COUNTER) {
            u_swa_qual_not_avail_counter++;
        } else {
            b_swa_qual_not_avail = TRUE;
        }
    } else {
        u_swa_qual_not_avail_counter = 0;
        b_swa_qual_not_avail = FALSE;
    }

    /* check if steering angle is at low precision for 5 cycles */
    if (VED_GET_IO_STATE(VED_SIN_POS_SWA,
                         reqPorts->pVehicleInputSignals->VehSigMain.State) ==
        VED_IO_STATE_DECREASED) {
        if (u_swa_qual_low_precision_counter < MAX_SIGNAL_COUNTER) {
            u_swa_qual_low_precision_counter++;
        } else {
            b_swa_qual_low_precision = TRUE;
        }
    } else {
        u_swa_qual_low_precision_counter = 0;
        b_swa_qual_low_precision = FALSE;
    }

    /* check if lateral acceleration is not available for 5 cycles */
    if (VED_GET_IO_STATE(VED_SIN_POS_LATA,
                         reqPorts->pVehicleInputSignals->VehSigMain.State) !=
        VED_IO_STATE_VALID) {
        if (u_lat_acc_qual_not_avail_counter < MAX_SIGNAL_COUNTER) {
            u_lat_acc_qual_not_avail_counter++;
        } else {
            b_lat_acc_qual_not_avail = TRUE;
        }
    } else {
        u_lat_acc_qual_not_avail_counter = 0;
        b_lat_acc_qual_not_avail = FALSE;
    }

    /* check if esp is not available for 5 cycles */
    if (VED_GET_IO_STATE(VEH_SIG_BRAKE_ABS,
                         reqPorts->pVehicleInputSignals->Brake.State) !=
        VED_IO_STATE_VALID) {
        if (u_esp_qual_not_avail_counter < MAX_SIGNAL_COUNTER) {
            u_esp_qual_not_avail_counter++;
        } else {
            b_esp_qual_not_avail = TRUE;
        }
    } else {
        u_esp_qual_not_avail_counter = 0;
        b_esp_qual_not_avail = FALSE;
    }

    /* calculate the curve error and store in output */
    f_CurveError = VED_CalculateCurveError(input, pVED_Offsets);

    /* if curve output is not available */
    if (VED_GET_IO_STATE(VED_SOUT_POS_CURVE, pVehDyn->State) !=
        VED_IO_STATE_VALID) {
        u_CurveConf = 0;
    } else {
        /* if both yaw rate and steering angle offset are not available, or
         * either yaw rate, steering angle or ESP are not available or all wheel
         * inputs are not available */
        if ((((pVED_Offsets->Ywr.StandStillState == OUT_YWR_OFFS_NON) ||
              (pVED_Offsets->Ywr.StandStillState ==
               OUT_YWR_OFFS_STANDST_EEPROM)) &&
             (pVED_Offsets->Swa.State == 0)) ||
            (b_swa_qual_not_avail == TRUE) || (b_yaw_qual_not_avail == TRUE) ||
            (b_esp_qual_not_avail == TRUE) ||
            (b_all_wheels_qual_not_avail == TRUE)) {
            u_CurveConf = 1;
        }
        /* if only one offset is available, the yaw rate offset is too high,
           steering angle is at low precision, yaw rate is degraded, lat accel
           or one of the wheel inputs are not available */
        else if ((fABS(pVED_Offsets->Ywr.StandStillOffset) > YWR_OFFS_LIMIT) ||
                 ((pVED_Offsets->Ywr.StandStillState == OUT_YWR_OFFS_NON) ||
                  (pVED_Offsets->Ywr.StandStillState ==
                   OUT_YWR_OFFS_STANDST_EEPROM) ||
                  (pVED_Offsets->Swa.State == 0)) ||
                 (b_swa_qual_low_precision == TRUE) ||
                 (b_yaw_qual_not_def == TRUE) ||
                 ((b_lat_acc_qual_not_avail == TRUE) &&
                  (pVED_Offsets->Ywr.StandStillState == OUT_YWR_OFFS_DYN)) ||
                 ((b_one_wheel_qual_reduced == TRUE) &&
                  (pVED_Offsets->Ywr.StandStillState == OUT_YWR_OFFS_DYN))) {
            u_CurveConf = 2;
        } else {
            /* calculate the lateral curve error in x sec distance and set the
             * conf signal */
            f_LatError = VED_CalculateCurveLatDistance(
                0.0F, f_CurveError,
                TIME_DIST_CONF_6 * ved__internal_data.ved__ve_out
                                       .veh_velo); /* distance in 4 seconds */
            if (f_LatError <= MAX_LAT_ERROR) {
                u_CurveConf = 6;
            } else {
                f_LatError = VED_CalculateCurveLatDistance(
                    0.0F, f_CurveError,
                    TIME_DIST_CONF_5 *
                        ved__internal_data.ved__ve_out
                            .veh_velo); /* distance in 3 seconds */
                if (f_LatError <= MAX_LAT_ERROR) {
                    u_CurveConf = 5;
                } else {
                    f_LatError = VED_CalculateCurveLatDistance(
                        0.0F, f_CurveError,
                        TIME_DIST_CONF_4 *
                            ved__internal_data.ved__ve_out
                                .veh_velo); /* distance in 2 seconds */
                    if (f_LatError <= MAX_LAT_ERROR) {
                        u_CurveConf = 4;
                    } else {
                        f_LatError = VED_CalculateCurveLatDistance(
                            0.0F, f_CurveError,
                            TIME_DIST_CONF_3 *
                                ved__internal_data.ved__ve_out
                                    .veh_velo); /* distance in 1 second */
                        if (f_LatError <= MAX_LAT_ERROR) {
                            u_CurveConf = 3;
                        } else {
                            u_CurveConf = 2;
                        }
                    }
                }
            }
        }
    }

    /* new conf lower then before and higher than 2? */
    if ((u_CurveConfOutput > u_CurveConf) && (u_CurveConf > 2)) {
        /* new (lower) conf already calculates for 400ms? */
        if (u_conf_counter < MAX_HOLD_TIME) {
            /* count hold time of old higher conf */
            u_conf_counter++;
        } else {
            /* take new lower conf after hold time expired */
            u_CurveConfOutput = u_CurveConf;
        }
    } else {
        /* higher or equal conf or conf is 2 or lower, take conf value directly
         */
        u_conf_counter = 0;
        u_CurveConfOutput = u_CurveConf;
    }

    /* Store values in output port */
    pVehDyn->Lateral.Curve.CrvConf = u_CurveConfOutput;
    pVehDyn->Lateral.Curve.CrvError = f_CurveError;
}

/* **********************************************************************
  @fn             VED_CalcCurveLatDistance */ /*!
  @brief          Calculate lateral distance of 2 curves in given distance 

  @description    On 2 circles with a given radius (or curvature), the
                  euklidian distance between the points on these 2 circles with
                  the same arc length is calculated

  @param[in]      c1  curve of first circle
  @param[in]      c2  curve of second circle
  @param[in]      b   arc length, used for both cicles
  @param[out]     -
  @return         distance between the 2 points

  @pre            -
  @post           -

**************************************************************************** */
static float32 VED_CalculateCurveLatDistance(float32 f_c1,
                                             float32 f_c2,
                                             float32 f_b) {
    float32 f_xc1, f_yc1, f_xc2, f_yc2; /* coordinates of points on curves */
    float32 f_d;                        /* resulting distance */

    /* if curvature is very small, the point is assumed on the straight line,
     * otherwise calculate coordinates on circle */
    if (fABS(f_c1) < MIN_CURVE_FOR_APPROXIMATION) {
        f_xc1 = 0;
        f_yc1 = f_b;
    } else {
        f_xc1 = ((COS_HD_(f_c1 * f_b)) / f_c1) - (1 / f_c1);
        f_yc1 = (SIN_HD_(f_c1 * f_b)) / f_c1;
    }

    /* if curvature is very small, the point is assumed on the straight line,
     * otherwise calculate coordinates on circle */
    if (fABS(f_c2) < MIN_CURVE_FOR_APPROXIMATION) {
        f_xc2 = 0;
        f_yc2 = f_b;
    } else {
        f_xc2 = ((COS_HD_(f_c2 * f_b)) / f_c2) - (1 / f_c2);
        f_yc2 = (SIN_HD_(f_c2 * f_b)) / f_c2;
    }

    /* calculate distance between points */
    f_d = VED__SQRT(TUE_CML_Sqr(f_xc1 - f_xc2) + TUE_CML_Sqr(f_yc1 - f_yc2));

    return f_d;
}

/* *************************************************************************
  @fn             VED_CopyInputSignals */ /*!
  @brief          Copies input signals from VehSig into VED_In

  @description    All input signals in VehSigMain are copied into VED_In
                  for internal use.
                  Some input signals are debounced to avoid a fault
                  reaction for invalid input signals if the signal
                  is only invalid for a very brief time
                  (e.g. short bus faults or sporadic timing issues)

  @param[in]      VehSig: source of input signals
  @param[out]     VED_In:  destination of input signals
  @return         void

  @pre            -
  @post           -

**************************************************************************** */
static void VED_CopyInputSignals(V1_7_VEDVehSigMain_t *p_Input,
                                 const V1_7_VEDVehSigMain_t *p_VehSigMain) {
    /* external yaw rate and temperature */
    VED__f_CheckInputSignals(
        &p_Input->YawRate, p_VehSigMain->YawRate, &s_LastInputSignals.YawRate,
        &p_Input->State[VED_SIN_POS_YWR], p_VehSigMain->State[VED_SIN_POS_YWR],
        &s_LastInputSignals.State[VED_SIN_POS_YWR],
        &u_DebouceCounter[VED_SIN_POS_YWR]);

    VED__f_CheckInputSignals(&p_Input->YawRateTemp, p_VehSigMain->YawRateTemp,
                             &s_LastInputSignals.YawRateTemp,
                             &p_Input->State[VED_SIN_POS_YWR_TEMP],
                             p_VehSigMain->State[VED_SIN_POS_YWR_TEMP],
                             &s_LastInputSignals.State[VED_SIN_POS_YWR_TEMP],
                             &u_DebouceCounter[VED_SIN_POS_YWR_TEMP]);

    /* steering angle */
    VED__f_CheckInputSignals(&p_Input->StWheelAngle, p_VehSigMain->StWheelAngle,
                             &s_LastInputSignals.StWheelAngle,
                             &p_Input->State[VED_SIN_POS_SWA],
                             p_VehSigMain->State[VED_SIN_POS_SWA],
                             &s_LastInputSignals.State[VED_SIN_POS_SWA],
                             &u_DebouceCounter[VED_SIN_POS_SWA]);

    /* lateral acceleration */
    VED__f_CheckInputSignals(&p_Input->LatAccel, p_VehSigMain->LatAccel,
                             &s_LastInputSignals.LatAccel,
                             &p_Input->State[VED_SIN_POS_LATA],
                             p_VehSigMain->State[VED_SIN_POS_LATA],
                             &s_LastInputSignals.State[VED_SIN_POS_LATA],
                             &u_DebouceCounter[VED_SIN_POS_LATA]);

    /* wheel speeds */
    VED__f_CheckInputSignals(&p_Input->WhlVelFrLeft, p_VehSigMain->WhlVelFrLeft,
                             &s_LastInputSignals.WhlVelFrLeft,
                             &p_Input->State[VED_SIN_POS_WVEL_FL],
                             p_VehSigMain->State[VED_SIN_POS_WVEL_FL],
                             &s_LastInputSignals.State[VED_SIN_POS_WVEL_FL],
                             &u_DebouceCounter[VED_SIN_POS_WVEL_FL]);

    VED__f_CheckInputSignals(
        &p_Input->WhlVelFrRight, p_VehSigMain->WhlVelFrRight,
        &s_LastInputSignals.WhlVelFrRight, &p_Input->State[VED_SIN_POS_WVEL_FR],
        p_VehSigMain->State[VED_SIN_POS_WVEL_FR],
        &s_LastInputSignals.State[VED_SIN_POS_WVEL_FR],
        &u_DebouceCounter[VED_SIN_POS_WVEL_FR]);

    VED__f_CheckInputSignals(&p_Input->WhlVelReLeft, p_VehSigMain->WhlVelReLeft,
                             &s_LastInputSignals.WhlVelReLeft,
                             &p_Input->State[VED_SIN_POS_WVEL_RL],
                             p_VehSigMain->State[VED_SIN_POS_WVEL_RL],
                             &s_LastInputSignals.State[VED_SIN_POS_WVEL_RL],
                             &u_DebouceCounter[VED_SIN_POS_WVEL_RL]);

    VED__f_CheckInputSignals(
        &p_Input->WhlVelReRight, p_VehSigMain->WhlVelReRight,
        &s_LastInputSignals.WhlVelReRight, &p_Input->State[VED_SIN_POS_WVEL_RR],
        p_VehSigMain->State[VED_SIN_POS_WVEL_RR],
        &s_LastInputSignals.State[VED_SIN_POS_WVEL_RR],
        &u_DebouceCounter[VED_SIN_POS_WVEL_RR]);

    /* internal yaw rate and temperature */
    VED__f_CheckInputSignals(&p_Input->YawRateInt, p_VehSigMain->YawRateInt,
                             &s_LastInputSignals.YawRateInt,
                             &p_Input->State[VED_SIN_POS_YWRINT],
                             p_VehSigMain->State[VED_SIN_POS_YWRINT],
                             &s_LastInputSignals.State[VED_SIN_POS_YWRINT],
                             &u_DebouceCounter[VED_SIN_POS_YWRINT]);

    VED__f_CheckInputSignals(&p_Input->YawRateIntTemp,
                             p_VehSigMain->YawRateIntTemp,
                             &s_LastInputSignals.YawRateIntTemp,
                             &p_Input->State[VED_SIN_POS_YWRINT_TEMP],
                             p_VehSigMain->State[VED_SIN_POS_YWRINT_TEMP],
                             &s_LastInputSignals.State[VED_SIN_POS_YWRINT_TEMP],
                             &u_DebouceCounter[VED_SIN_POS_YWRINT_TEMP]);

    /* internal longitudinal acceleration */
    VED__f_CheckInputSignals(&p_Input->LongAccel, p_VehSigMain->LongAccel,
                             &s_LastInputSignals.LongAccel,
                             &p_Input->State[VED_SIN_POS_LONGA],
                             p_VehSigMain->State[VED_SIN_POS_LONGA],
                             &s_LastInputSignals.State[VED_SIN_POS_LONGA],
                             &u_DebouceCounter[VED_SIN_POS_LONGA]);

    /* rear wheel steering angle */
    VED__f_CheckInputSignals(&p_Input->RearWhlAngle, p_VehSigMain->RearWhlAngle,
                             &s_LastInputSignals.RearWhlAngle,
                             &p_Input->State[VED_SIN_POS_RSTA],
                             p_VehSigMain->State[VED_SIN_POS_RSTA],
                             &s_LastInputSignals.State[VED_SIN_POS_RSTA],
                             &u_DebouceCounter[VED_SIN_POS_RSTA]);

    /* external longitudinal velocity */
    VED__f_CheckInputSignals(&p_Input->VehVelocityExt,
                             p_VehSigMain->VehVelocityExt,
                             &s_LastInputSignals.VehVelocityExt,
                             &p_Input->State[VED_SIN_POS_VEHVEL_EXT],
                             p_VehSigMain->State[VED_SIN_POS_VEHVEL_EXT],
                             &s_LastInputSignals.State[VED_SIN_POS_VEHVEL_EXT],
                             &u_DebouceCounter[VED_SIN_POS_VEHVEL_EXT]);

    /* external longitudinal acceleration */
    VED__f_CheckInputSignals(&p_Input->VehLongAccelExt,
                             p_VehSigMain->VehLongAccelExt,
                             &s_LastInputSignals.VehLongAccelExt,
                             &p_Input->State[VED_SIN_POS_VEHACL_EXT],
                             p_VehSigMain->State[VED_SIN_POS_VEHACL_EXT],
                             &s_LastInputSignals.State[VED_SIN_POS_VEHACL_EXT],
                             &u_DebouceCounter[VED_SIN_POS_VEHACL_EXT]);

    /* wheel rolling directions */
    VED__u_CheckInputSignals(&p_Input->WhlDirFrLeft, p_VehSigMain->WhlDirFrLeft,
                             &s_LastInputSignals.WhlDirFrLeft,
                             &p_Input->State[VED_SIN_POS_WDIR_FL],
                             p_VehSigMain->State[VED_SIN_POS_WDIR_FL],
                             &s_LastInputSignals.State[VED_SIN_POS_WDIR_FL],
                             &u_DebouceCounter[VED_SIN_POS_WDIR_FL]);

    VED__u_CheckInputSignals(
        &p_Input->WhlDirFrRight, p_VehSigMain->WhlDirFrRight,
        &s_LastInputSignals.WhlDirFrRight, &p_Input->State[VED_SIN_POS_WDIR_FR],
        p_VehSigMain->State[VED_SIN_POS_WDIR_FR],
        &s_LastInputSignals.State[VED_SIN_POS_WDIR_FR],
        &u_DebouceCounter[VED_SIN_POS_WDIR_FR]);

    VED__u_CheckInputSignals(&p_Input->WhlDirReLeft, p_VehSigMain->WhlDirReLeft,
                             &s_LastInputSignals.WhlDirReLeft,
                             &p_Input->State[VED_SIN_POS_WDIR_RL],
                             p_VehSigMain->State[VED_SIN_POS_WDIR_RL],
                             &s_LastInputSignals.State[VED_SIN_POS_WDIR_RL],
                             &u_DebouceCounter[VED_SIN_POS_WDIR_RL]);

    VED__u_CheckInputSignals(
        &p_Input->WhlDirReRight, p_VehSigMain->WhlDirReRight,
        &s_LastInputSignals.WhlDirReRight, &p_Input->State[VED_SIN_POS_WDIR_RR],
        p_VehSigMain->State[VED_SIN_POS_WDIR_RR],
        &s_LastInputSignals.State[VED_SIN_POS_WDIR_RR],
        &u_DebouceCounter[VED_SIN_POS_WDIR_RR]);

    /* wheel pulses */
    VED__u_CheckInputSignals(&p_Input->WhlTicksDevFrLeft,
                             p_VehSigMain->WhlTicksDevFrLeft,
                             &s_LastInputSignals.WhlTicksDevFrLeft,
                             &p_Input->State[VED_SIN_POS_WTCKS_FL],
                             p_VehSigMain->State[VED_SIN_POS_WTCKS_FL],
                             &s_LastInputSignals.State[VED_SIN_POS_WTCKS_FL],
                             &u_DebouceCounter[VED_SIN_POS_WTCKS_FL]);

    VED__u_CheckInputSignals(&p_Input->WhlTicksDevFrRight,
                             p_VehSigMain->WhlTicksDevFrRight,
                             &s_LastInputSignals.WhlTicksDevFrRight,
                             &p_Input->State[VED_SIN_POS_WTCKS_FR],
                             p_VehSigMain->State[VED_SIN_POS_WTCKS_FR],
                             &s_LastInputSignals.State[VED_SIN_POS_WTCKS_FR],
                             &u_DebouceCounter[VED_SIN_POS_WTCKS_FR]);

    VED__u_CheckInputSignals(&p_Input->WhlTicksDevReLeft,
                             p_VehSigMain->WhlTicksDevReLeft,
                             &s_LastInputSignals.WhlTicksDevReLeft,
                             &p_Input->State[VED_SIN_POS_WTCKS_RL],
                             p_VehSigMain->State[VED_SIN_POS_WTCKS_RL],
                             &s_LastInputSignals.State[VED_SIN_POS_WTCKS_RL],
                             &u_DebouceCounter[VED_SIN_POS_WTCKS_RL]);

    VED__u_CheckInputSignals(&p_Input->WhlTicksDevReRight,
                             p_VehSigMain->WhlTicksDevReRight,
                             &s_LastInputSignals.WhlTicksDevReRight,
                             &p_Input->State[VED_SIN_POS_WTCKS_RR],
                             p_VehSigMain->State[VED_SIN_POS_WTCKS_RR],
                             &s_LastInputSignals.State[VED_SIN_POS_WTCKS_RR],
                             &u_DebouceCounter[VED_SIN_POS_WTCKS_RR]);

    /* gear position */
    VED__u_CheckInputSignals(&p_Input->ActGearPos, p_VehSigMain->ActGearPos,
                             &s_LastInputSignals.ActGearPos,
                             &p_Input->State[VED_SIN_POS_GEAR],
                             p_VehSigMain->State[VED_SIN_POS_GEAR],
                             &s_LastInputSignals.State[VED_SIN_POS_GEAR],
                             &u_DebouceCounter[VED_SIN_POS_GEAR]);

    /* brake activation level */
    p_Input->BrakeActLevel = 0U;
    VED__s_CheckInputSignals(
        &p_Input->BrakeActLevel, p_VehSigMain->BrakeActLevel,
        &s_LastInputSignals.BrakeActLevel, &p_Input->State[VED_SIN_POS_BRAKE],
        p_VehSigMain->State[VED_SIN_POS_BRAKE],
        &s_LastInputSignals.State[VED_SIN_POS_BRAKE],
        &u_DebouceCounter[VED_SIN_POS_BRAKE]);

    /* parking brake state */
    VED__u_CheckInputSignals(
        &p_Input->ParkBrakeState, p_VehSigMain->ParkBrakeState,
        &s_LastInputSignals.ParkBrakeState, &p_Input->State[VED_SIN_POS_PBRK],
        p_VehSigMain->State[VED_SIN_POS_PBRK],
        &s_LastInputSignals.State[VED_SIN_POS_PBRK],
        &u_DebouceCounter[VED_SIN_POS_PBRK]);

    /* external driving direction */
    VED__u_CheckInputSignals(
        &p_Input->VehLongDirExt, p_VehSigMain->VehLongDirExt,
        &s_LastInputSignals.VehLongDirExt, &p_Input->State[VED_SIN_POS_VDIR],
        p_VehSigMain->State[VED_SIN_POS_VDIR],
        &s_LastInputSignals.State[VED_SIN_POS_VDIR],
        &u_DebouceCounter[VED_SIN_POS_VDIR]);

    /* external motion state */
    VED__u_CheckInputSignals(&p_Input->VehLongMotStateExt,
                             p_VehSigMain->VehLongMotStateExt,
                             &s_LastInputSignals.VehLongMotStateExt,
                             &p_Input->State[VED_SIN_POS_VMOT],
                             p_VehSigMain->State[VED_SIN_POS_VMOT],
                             &s_LastInputSignals.State[VED_SIN_POS_VMOT],
                             &u_DebouceCounter[VED_SIN_POS_VMOT]);

    /* external lateral curvature */
    VED__f_CheckInputSignals(&p_Input->CurveC0Ext, p_VehSigMain->CurveC0Ext,
                             &s_LastInputSignals.CurveC0Ext,
                             &p_Input->State[VED_SIN_POS_CRV],
                             p_VehSigMain->State[VED_SIN_POS_CRV],
                             &s_LastInputSignals.State[VED_SIN_POS_CRV],
                             &u_DebouceCounter[VED_SIN_POS_CRV]);

    VED__f_CheckInputSignals(&p_Input->CurveC1Ext, p_VehSigMain->CurveC1Ext,
                             &s_LastInputSignals.CurveC1Ext,
                             &p_Input->State[VED_SIN_POS_CRV],
                             p_VehSigMain->State[VED_SIN_POS_CRV],
                             &s_LastInputSignals.State[VED_SIN_POS_CRV],
                             &u_DebouceCounter[VED_SIN_POS_CRV]);

    /* external side slip angle */
    VED__f_CheckInputSignals(
        &p_Input->SideSlipAngleExt, p_VehSigMain->SideSlipAngleExt,
        &s_LastInputSignals.SideSlipAngleExt, &p_Input->State[VED_SIN_POS_SSA],
        p_VehSigMain->State[VED_SIN_POS_SSA],
        &s_LastInputSignals.State[VED_SIN_POS_SSA],
        &u_DebouceCounter[VED_SIN_POS_SSA]);

#if (defined(CFG_VED__VELO_MONITOR_MIN_MAX) && (CFG_VED__VELO_MONITOR_MIN_MAX))

    /* external MIN velocity  */
    VED__f_CheckInputSignals(&p_Input->VehVelocityExtMin,
                             p_VehSigMain->VehVelocityExtMin,
                             &s_LastInputSignals.VehVelocityExtMin,
                             &p_Input->State[VED__SIN_POS_VEHVEL_MIN],
                             p_VehSigMain->State[VED__SIN_POS_VEHVEL_MIN],
                             &s_LastInputSignals.State[VED__SIN_POS_VEHVEL_MIN],
                             &u_DebouceCounter[VED__SIN_POS_VEHVEL_MIN]);

    /* external MAX velocity  */
    VED__f_CheckInputSignals(&p_Input->VehVelocityExtMax,
                             p_VehSigMain->VehVelocityExtMax,
                             &s_LastInputSignals.VehVelocityExtMax,
                             &p_Input->State[VED__SIN_POS_VEHVEL_MAX],
                             p_VehSigMain->State[VED__SIN_POS_VEHVEL_MAX],
                             &s_LastInputSignals.State[VED__SIN_POS_VEHVEL_MAX],
                             &u_DebouceCounter[VED__SIN_POS_VEHVEL_MAX]);

#endif
}

/* *************************************************************************
  @fn             VED__f_CheckInputSignals */ /*!
  @brief          Debounces invalid float type input signals

  @description    A signal which has the state 'invalid' or 'not available'
                  (= timeout) is kept at the last valid input signal and state
                  for VED__MAX_INPUT_DEBOUNCING cycles.

  @param[in]      current input signal
  @param[in]      current input state
  @param[in,out]  last input signal
  @param[in,out]  last input state
  @param[out]     output input signals
  @param[out]     putput input state
  @param[in,out]  debounce counter
  @return         void

**************************************************************************** */
static void VED__f_CheckInputSignals(float32 *p_InputSignal,
                                     float32 f_VehSigMainSignal,
                                     float32 *p_LastSignal,
                                     uint8 *p_VED_InState,
                                     uint8 e_VehSigState,
                                     uint8 *p_LastState,
                                     uint8 *p_DebouceCounter) {
    /* invalid and not available signals are debounced, all other states are
     * valid inputs */
    if ((e_VehSigState == VED_IO_STATE_INVALID) ||
        (e_VehSigState == VED_IO_STATE_NOTAVAIL)) {
        if (*p_DebouceCounter >= VED__MAX_INPUT_DEBOUNCING) {
            /* no debouncing any more, take input value */
            *p_InputSignal = f_VehSigMainSignal;
            *p_VED_InState = e_VehSigState;
            *p_LastSignal = f_VehSigMainSignal;
            *p_LastState = e_VehSigState;
        } else {
            /* keep old value and increase counter */
            (*p_DebouceCounter)++;
            *p_InputSignal = *p_LastSignal;
            *p_VED_InState = *p_LastState;
        }
    } else {
        /* take new value and state */
        *p_InputSignal = f_VehSigMainSignal;
        *p_VED_InState = e_VehSigState;
        /* reset debounce counter */
        *p_DebouceCounter = 0U;
        /* store current signal and state for next cycle */
        *p_LastSignal = f_VehSigMainSignal;
        *p_LastState = e_VehSigState;
    }
}

/* *************************************************************************
  @fn             VED__u_CheckInputSignals */ /*!
  @brief          Debounces invalid unsigned byte type input signals

  @description    A signal which has the state 'invalid' or 'not available'
                  (= timeout) is kept at the last valid input signal and state
                  for VED__MAX_INPUT_DEBOUNCING cycles.

  @param[in]      current input signal
  @param[in]      current input state
  @param[in,out]  last input signal
  @param[in,out]  last input state
  @param[out]     output input signals
  @param[out]     putput input state
  @param[in,out]  debounce counter
  @return         void

**************************************************************************** */
static void VED__u_CheckInputSignals(uint8 *p_InputSignal,
                                     uint8 u_VehSigMainSignal,
                                     uint8 *p_LastSignal,
                                     uint8 *p_VED_InState,
                                     uint8 e_VehSigState,
                                     uint8 *p_LastState,
                                     uint8 *p_DebouceCounter) {
    /* invalid and not available signals are debounced, all other states are
     * valid inputs */
    if ((e_VehSigState == VED_IO_STATE_INVALID) ||
        (e_VehSigState == VED_IO_STATE_NOTAVAIL)) {
        if (*p_DebouceCounter >= VED__MAX_INPUT_DEBOUNCING) {
            /* no debouncing any more, take input value */
            *p_InputSignal = u_VehSigMainSignal;
            *p_VED_InState = e_VehSigState;
            *p_LastSignal = u_VehSigMainSignal;
            *p_LastState = e_VehSigState;
        } else {
            /* keep old value and increase counter */
            (*p_DebouceCounter)++;
            *p_InputSignal = *p_LastSignal;
            *p_VED_InState = *p_LastState;
        }
    } else {
        /* take new value and state */
        *p_InputSignal = u_VehSigMainSignal;
        *p_VED_InState = e_VehSigState;
        /* reset debounce counter */
        *p_DebouceCounter = 0U;
        /* store current signal and state for next cycle */
        *p_LastSignal = u_VehSigMainSignal;
        *p_LastState = e_VehSigState;
    }
}

/* *************************************************************************
  @fn             VED__s_CheckInputSignals */ /*!
  @brief          Debounces invalid unsigned short type input signals

  @description    A signal which has the state 'invalid' or 'not available'
                  (= timeout) is kept at the last valid input signal and state
                  for VED__MAX_INPUT_DEBOUNCING cycles.

  @param[in]      current input signal
  @param[in]      current input state
  @param[in,out]  last input signal
  @param[in,out]  last input state
  @param[out]     output input signals
  @param[out]     putput input state
  @param[in,out]  debounce counter
  @return         void

**************************************************************************** */
static void VED__s_CheckInputSignals(uint16 *p_InputSignal,
                                     uint16 u_VehSigMainSignal,
                                     uint16 *p_LastSignal,
                                     uint8 *p_VED_InState,
                                     uint8 e_VehSigState,
                                     uint8 *p_LastState,
                                     uint8 *p_DebouceCounter) {
    /* invalid and not available signals are debounced, all other states are
     * valid inputs */
    if ((e_VehSigState == VED_IO_STATE_INVALID) ||
        (e_VehSigState == VED_IO_STATE_NOTAVAIL)) {
        if (*p_DebouceCounter >= VED__MAX_INPUT_DEBOUNCING) {
            /* no debouncing any more, take input value */
            *p_InputSignal = u_VehSigMainSignal;
            *p_VED_InState = e_VehSigState;
            *p_LastSignal = u_VehSigMainSignal;
            *p_LastState = e_VehSigState;
        } else {
            /* keep old value and increase counter */
            (*p_DebouceCounter)++;
            *p_InputSignal = *p_LastSignal;
            *p_VED_InState = *p_LastState;
        }
    } else {
        /* take new value and state */
        *p_InputSignal = u_VehSigMainSignal;
        *p_VED_InState = e_VehSigState;
        /* reset debounce counter */
        *p_DebouceCounter = 0U;
        /* store current signal and state for next cycle */
        *p_LastSignal = u_VehSigMainSignal;
        *p_LastState = e_VehSigState;
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */