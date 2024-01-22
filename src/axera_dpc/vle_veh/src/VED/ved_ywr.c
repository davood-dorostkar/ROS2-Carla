/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "ved_consts.h"
#include "ved.h"

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/* Definitions for yaw rate filtering */

/* Minimum velocity for calculation of curvature out of yaw rate */
#define YWR_CURVE_V_MIN C_F32_DELTA /* Min Vego for curvature calculation */

/* Definitions for stand still offset calculation */
#define VED__YWR_OFF_STST_GRAD_ABS_MAX                                        \
    DEG2RAD(1.2F) /* Max yaw rate gradient for stand still offset calculation \
                   */

#define VED__YWR_OFF_STST_SUM_MAX_TIME \
    ((float32)60.0) /* Max averaging time for complete interval */
#define VED__YWR_OFF_STST_INTERV_HOLD_TIME                                    \
    ((float32)60.0) /* Hold time after standstill for sampled yaw rate offset \
                       values  */

/* Definitions for dyn. offset calculation */
#define YWR_GRAD_DLT_DST_MAX                         \
    DEG2RAD(1.5F) /* LP Max gradient for delta check \
                   */

/* Definitions for quality */
#define YWR_ELPSD_TIME_SENSOR_COOL                                         \
    ((float32)(30.0 * 60.0)) /* Ecu time after which the offset is assumed \
                                tempered */

#define YWR_TIME_SENSOR_COOL_MIN                                               \
    ((float32)(5.0 * 60.0)) /* Zeit bis zur Guetereduzierung bei kaltem Sensor \
                               (Hochlauf)       */
#define YWR_TIME_SENSOR_COOL_MAX                                             \
    ((float32)(20.0 * 60.0)) /* Zeit bis zur max Guetereduzierung bei kaltem \
                                Sensor (Hochlauf)   */
#define YWR_Q_FACTOR_SENSOR_COOL_MAX \
    (1.0F) /* Faktor fuer maximale Guete bei kaltem Sensor */
#define YWR_Q_FACTOR_SENSOR_COOL_MIN \
    (0.75F) /* Faktor fuer minimale Guete bei kaltem Sensor */

#define YWR_TIME_SENSOR_TEMP_MIN                                         \
    ((float32)(15.0 * 60.0)) /* Zeit bis zur Guetereduzierung bei warmen \
                                Sensor (eingeschwungen) */
#define YWR_TIME_SENSOR_TEMP_MAX                                             \
    ((float32)(25.0 * 60.0)) /* Zeit bis zur max Guetereduzierung bei warmen \
                                Sensor              */
#define YWR_Q_FACTOR_SENSOR_TEMP_MAX \
    ((float32)1.0) /* Faktor fuer maximale Guete bei warmen Sensor */
#define YWR_Q_FACTOR_SENSOR_TEMP_MIN \
    ((float32)0.85) /* Faktor fuer minimale Guete bei warmen Sensor */

#if (CFG_VED__EX_YWR_NVM)
#define YWR_NVM_WRITE_MIN_ADJ_TIME                                        \
    ((float32)(3.0)) /* Minimum standstill adjust time to store offset in \
                        nonvolatile memory */
#define YWR_NVM_WRITE_NEW_GAP_TIME \
    ((float32)(30.0 * 60.0)) /* Time gap between successive write access  */
#define YWR_NVM_OFFS_STABLE_TIME_1ST                                           \
    ((float32)(25.0 * 60.0)) /* Time after gyro offset is assumed to be stable \
                                for first write */
#define YWR_NVM_OFFS_STABLE_TIME                                               \
    ((float32)(35.0 * 60.0)) /* Time after gyro offset is assumed to be stable \
                                for non-first writes */
#define YWR_NVM_WRITE_OFFS_ELAPS_TIME                                       \
    ((float32)(1.0)) /* After this Time gyro offset is written to NVM after \
                        first stand still and nvm offset state is 0 (no     \
                        offset) */

#define YWR_NVM_FLT_MAX_WGHT \
    (10UL) /* Max filter weight for filtering standstill for nvm storage */
#define YWR_NVM_OFFSET_STATUS_MAX \
    (100UL) /* Maximum value of nvm standstill offset status */
#define YWR_NVM_WRITE_IGN_MAX_NO \
    (2UL) /* Maximum number of write accesses during one ignition cycle */
#define YWR_NVM_STATUS_MAX \
    (3UL) /* Maximum status value for nvm standstill offset status */

/* Linear ramp for nvm offset quality over time */
#define YWR_TIME_Q_EEPROM_OFFSET_MIN \
    ((float32)0.0) /* Start time with minimal confidence */
#define YWR_TIME_Q_EEPROM_OFFSET_MAX \
    YWR_TIME_SENSOR_TEMP_MAX /* End time with maximal confidence */
#define YWR_Q_FACTOR_EEPROM_OFFSET_MIN    \
    ((float32)0.75) /* Minimum confidence \
                     */
#define YWR_Q_FACTOR_EEPROM_OFFSET_MAX    \
    ((float32)0.95) /* Maximum confidence \
                     */

#endif

#if (CFG_VED__YW_DYN_AVG)

#define YWR_DYNA_DIST_STEP (40.F) /* Abtastdistanz fuer Filterung ueber Weg */

#define YWR_TIME_DYNAVGOFF_MIN \
    ((float32)(5.0 * 60.0)) /* Zeit bis zur Guetereduzierung        */
#define YWR_TIME_DYNAVGOFF_MAX \
    ((float32)(15.0 * 60.0)) /* Zeit bis zur max Guetereduzierung    */
#define YWR_Q_FACTOR_DYNAVGOFF_MAX \
    (1.0F) /* Faktor fuer maximale Guete           */
#define YWR_Q_FACTOR_DYNAVGOFF_MIN \
    (0.60F) /* Faktor fuer minimale Guete           */

#define YWR_DIST_DYNAVGOFF_MIN \
    ((float32)3500.0) /* Distanz bis zur Guetereduzierung        */
#define YWR_DIST_DYNAVGOFF_MAX \
    ((float32)5000.0) /* Distanz bis zur max Guetereduzierung    */
#define YWR_QUALITY_DYNAVGOFF_MAX \
    ((float32)0.95) /* Faktor fuer maximale Guete  */
#define YWR_QUALITY_DYNAVGOFF_MIN \
    ((float32)0.50) /* Faktor fuer minimale Guete  */
#define YWR_STDIST_SYMM_RESET \
    ((float32)7000.0) /* Zeit bis zur max Guetereduzierung    */

#define YWR_OFFSET_DELTA_MIN \
    DEG2RAD(0.2F) /* Offset Abweichung bis zur Guetereduzierung            */
#define YWR_OFFSET_DELTA_MED \
    DEG2RAD(0.5F) /* Offset Abweichung bis zur mittleren Reduzierung       */
#define YWR_OFFSET_DELTA_MAX \
    DEG2RAD(0.7F) /* Offset Abweichung bis zur maximalen Reduzierung       */

#define YWR_Q_FACTOR_DELTA_MAX \
    (1.0F) /* Faktor fuer maximale Guete bei minimaler Abweichung   */
#define YWR_Q_FACTOR_DELTA_MED \
    (0.75F) /* Faktor fuer maximale Guete bei mittlerer Abweichung   */
#define YWR_Q_FACTOR_DELTA_MIN \
    (0.0F) /* Faktor fuer minimale Guete bei maximaler Abweichung   */

/* Schwellen bei Dynamsicher Average-Offset bei Start */
#define YWR_OFFSET_TIME_START_MIN \
    (0.F) /* Offsetzeit Start Guete               */
#define YWR_OFFSET_TIME_START_MAX \
    (5.F * 60.F) /* Offsetzeit End-Guete                 */
#define YWR_Q_FACTOR_DELTA_START_MIN \
    (0.4F) /* Faktor fuer minimale Guete bei Start */
#define YWR_Q_FACTOR_DELTA_START_MAX \
    (1.0F) /* Faktor fuer maximale Guete bei Ende  */

#define YWR_WSP_Q_START_MIN \
    (0.1F) /* WSP minimale Eingangs-Guete                          */
#define YWR_WSP_Q_END_MAX \
    (0.5F) /* WSP maximale Eingangs-Guete                          */
#define YWR_WSP_Q_CONF_START_MIN \
    (0.0F) /* WSP minimaler Confidence Faktor zur Guetereduzierung */
#define YWR_WSP_Q_CONF_START_MAX \
    (1.0F) /* WSP maximaler Confidence Faktor zur Guetereduzierung */

/* Streckensymmetrie-Schwellen fuer Interims-Cache Offset */
#define YWR_DYNC_STSYM_START (1000.F) /* Start Unsymmetrie Reduzierung */
#define YWR_DYNC_STSYM_END (2000.F)   /* Ende  Unsymmetrie Reduzierung        */
#define YWR_DYNC_Q_PERM_MAX (1.F)     /* Minimale Reduzierung des Einflusses  */
#define YWR_DANC_Q_PERM_MIN (0.F)     /* Maximale Reduzierung des Einflusses  */

#define YWR_DYNC_QRED_DIST_MIN \
    (3000.F) /* Mindest-Filter-Distanz zuer Guetereduzierung */
#define VED__YWR_DYN_OFFSET_MIN_DIST                                           \
    ((float32)(2000.F)) /* minimum driven distance to start dyn yawrate offset \
                           calculation */

#endif

#define VED__YWR_OFFSET_REDUCTION_FACTOR                                     \
    (0.9F) /* offset data reduction if standstill offset calculation takes a \
              long time */
#define NVM_CLEARED (uint32)(0xFFFFFFFFU)

/*****************************************************************************
  MACROS
*****************************************************************************/

#define YWR_GET_ME (&VED_YwrGlobData)
#define YWR_GET_MIF_DATA (VED_YwrGlobData.Io.mif)
#define YWR_GET_NVM_READ (VED_YwrGlobData.Io.nv_read)
#define YWR_GET_NVM_WRITE (VED_YwrGlobData.Io.nv_write)
#if ((defined(CFG_VED__NVM_LEARN_DATA_ERROR)) && \
     (CFG_VED__NVM_LEARN_DATA_ERROR))
/* check for valid values written into NVM */
#define YWR_SET_NVM_WRITE (*VED_YwrGlobData.Io.YwrNVMerrOffsRg)
#endif

#define YWR_GET_DATA (&VED_YwrGlobData.Sensor)
#define YWR_GET_OFFS (&VED_YwrGlobData.Offset)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(VED_YwrGlobData)
static VED_YwrData_t
    VED_YwrGlobData; /*!< @VADDR: 0x20015000 @VNAME: VED_Ywr @ALLOW: ved__priv
                        @cycleid: ved__cycle_id*/
#if ((defined(CFG_VED__TURNTABLE_DETECTION)) && (CFG_VED__TURNTABLE_DETECTION))
static float32 PrevYawRateOffset;
#endif

#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
SET_MEMSEC_VAR(LastYawNvmData) /*New structure for storing previous cycle yaw
                                  rate offset of NVM*/
static VED_LastNvYawCal_t LastYawNvmData;
#endif
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  LOCAL FUNCTION PROTOTYPES
*****************************************************************************/
static void VED_YwrCalcRunnigTime(void);
static void VED_YwrInitYawRateData(VED_YwrSenData_t *YawRateData);
static void VED_YwrCommonInit(void);
static void VED_YwrOffsetInit(const VEDNvIoDatas_t *nv_read);
static void VED_YwrInitOffsetData(VED_YwrOffsData_t *OffsData);
static void VED_YwrCalcOffsetQualityTime(float32 *OffsQuality,
                                         float32 MaxQuality,
                                         float32 OffsElpsdTime,
                                         boolean TemperOK);
static boolean VED_YwrTakeOffs(VED_YwrOffsData_t *OffsData,
                               float32 Offset,
                               float32 Quality,
                               VED_YwrOffsType_t Type);
static void VED_YwrCalcOffset(const VED_YwrSenData_t *YawRate,
                              VED_YwrOffsData_t *OffsData);
static void VED_YwrCalcToAutocode(VED_YwrOffsData_t *Offs);
#if ((defined(CFG_VED__NVM_LEARN_DATA_ERROR)) && \
     (CFG_VED__NVM_LEARN_DATA_ERROR))
static boolean VED_YwrOffsetRangeOk(float32 Offset);
#endif
#if (CFG_VED__YW_DYN_AVG)
static void VED_YwrInitDynOffsAvg(VED_YwrDynOffsAvg_t *DynOffsAvg);
static void VED_YwrInitDynOffsInterim(VED_YwrDynOffsInter_t *DynOffsInter);
static float32 VED_YwrCalcDynOffsAvg(const VED_YwrSenData_t *YawRateData,
                                     VED_YwrDynOffsAvg_t *DynOffset,
                                     const VED_YwrOffsData_t *Offs);
static float32 VED_YwrCalcQRedRamp(const VED_YwrOffsData_t *OffsData);
static void VED_YwrMergeOffsetDrift(VED_YwrOffsData_t *OffsData,
                                    VED_YwrDynOffsCache_t *DynOffsCache);
static void VED_YwrCalcDynOffsInterim(VED_YwrDynOffsInter_t *DynOffsInter,
                                      const VED_YwrOffsData_t *Offs,
                                      float32 dist);
#endif

#if (CFG_VED__EX_YWR_NVM)
static void VED_YwrEepromWriteYwOffset(const VED_YwrEepromOffset_t *NVValue,
                                       const VEDNvIoDatas_t *nvin,
                                       VEDNvIoDatas_t *nvout);
static void VED_YwrEepromReadYwOffset(VED_YwrEepromOffset_t *NVValue,
                                      const VEDNvIoDatas_t *nvin);
static void VED_YwrCalcEepromOffsetQuality(
    VED_YwrStandStillEepromOffs_t *pNvmOffs);
static void VED_YwrInitStandStillEepromOffset(
    VED_YwrStandStillEepromOffs_t *StandStEepromOffs);
static void VED_YwrSaveStandstillOffset(VED_YwrOffsData_t *OffsetData);
static void VED_YwrCalcEepromOffsetQualityTime(float32 *OffsQuality,
                                               float32 MaxQuality,
                                               float32 EcuElpsdTime);
#else
static void VED_YwrCalcStandstillOffset(
    const VED_YwrSenData_t *YawRate, VED_YwrStandStillOffs_t *StandStillOffset);
#endif

/* **********************************************************************
  @fn               VED_YwrExec */ /*!
  @brief            Main function to execute yaw rate offset calculation 

  @description      Inits ywr module
                    Takes yaw rate offset from NVM
                    Filters input yaw rate and calculates gradient
                    Calculates standstill offset
                    Calculates another dynamic offset for plausibilisation
                    with dynamic offset from wheel module
                    Prepares data for autocode module

  @param[in]        reqPorts  NVM offset values
  @param[in]        input     signal data, timing info and operation mode
  @param[in]        mif       vehicle velocity
  @param[out]       proPorts  updated NVM offset values
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_YwrExec(const reqVEDPrtList_t *reqPorts,
                 const VED_InputData_t *input,
                 VED_ModIf_t *mif,
                 const proVEDPrtList_t *proPorts) {
    YWR_GET_ME->Io.in = input;
    YWR_GET_ME->Io.nv_read = reqPorts->pNVMRead;
    YWR_GET_ME->Io.nv_write = proPorts->pNVMWrite;
    YWR_GET_ME->Io.mif = mif;

#if ((defined(CFG_VED__NVM_LEARN_DATA_ERROR)) && \
     (CFG_VED__NVM_LEARN_DATA_ERROR))
    /* check for valid values written into NVM */
    YWR_GET_ME->Io.YwrNVMerrOffsRg =
        &proPorts->pVED_Errors->OutPutErrors.YwrNVMOffsRg;
    proPorts->pVED_Errors->OutPutErrors.YwrNVMOffsRg = VED_ERR_STATE_UNKNOWN;
#endif

    /* Distinguish between different operating states */
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_RUNNING,
                            input->Frame.CtrlMode)) {
        /*<--- Execution path for normal operating mode --->*/
        VED_YwrSenData_t *YawRateData =
            YWR_GET_DATA;                           /* General yaw rate data */
        VED_YwrOffsData_t *OffsData = YWR_GET_OFFS; /* Offset data */

        /* Calculation of time variables which have to be calculated even if
         * offset calculation is not done because input yaw rate is invalid */
        VED_YwrCalcRunnigTime();

#if (CFG_VED__EX_YWR_NVM)
#if ((!defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) || \
     (!CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
        if (VED__GET_NVM_IO_STATE(VED_NVM_POS_YWR, &YWR_GET_NVM_READ->State) ==
            VED_IO_STATE_VALID)
#endif
        {
            VED_YwrEepromReadYwOffset(
                &OffsData->StandStillEepromOffset.YawRateOffset,
                YWR_GET_NVM_READ);
            VED_YwrCalcEepromOffsetQuality(&OffsData->StandStillEepromOffset);
        }
#endif

        if (VED_GET_IO_STATE(VED_SIN_POS_YWR, input->Signals.State) ==
            VED_IO_STATE_VALID) {
            YawRateData->YawRate = input->Signals.YawRate;
            YawRateData->Valid = TRUE;
        } else {
            YawRateData->Valid = FALSE;
        }
#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
        {
            VED_SET_NVM_IO_STATE(VED_NVM_POS_YWR, VED_IO_STATE_INVALID,
                                 &proPorts->pNVMWrite->State);
        }
#endif
        if (YawRateData->Valid == TRUE) {
            /* Calculate filtered yaw rate and yaw rate gradient */
            /* in case of first cycle, init old filt yaw rate with actual yaw
             * rate */
            if (YawRateData->YwFirstCycleDone == FALSE) {
                YawRateData->YawRateOld = YawRateData->YawRate;
            }
            float32 YawRateFilt; /* Filtered yaw rate */
            YawRateFilt = VED_FilterCycleTime(YawRateData->YawRate,
                                              YawRateData->YawRateOld,
                                              VED__PAR_YWR_YAWRATE_FT);
            YawRateData->Gradient =
                VED_CalcGradient(YawRateFilt, YawRateData->YawRateOld);
            YawRateData->YawRateOld = YawRateFilt;

            /* Check filtered yaw rate for high gradients over distance (only
             * for dynamic offset calculation) */
            VED_CalcDistStblGrad(
                YWR_GRAD_DLT_DST_MAX, fABS(YawRateData->Gradient),
                &YawRateData->GradientAbsOld, &YawRateData->DeltaDist,
                YWR_GET_MIF_DATA->LongMot.VehVelocityCorr);

            /* Offset calculation */
            VED_YwrCalcOffset(YawRateData, OffsData);

#if (CFG_VED__YW_DYN_AVG)
            /* Calculate mean value between effective and interims offset in
             * case of drift */
            VED_YwrMergeOffsetDrift(OffsData, &OffsData->Cache);
#endif

#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
            {
                VEDNvIoDatas_t *nvout = YWR_GET_NVM_WRITE;
                /* writing Yaw offset into NVM in every excution cycle */
                nvout->YwRate.ZeroRate = LastYawNvmData.ZeroRate;
                nvout->YwRate.CalStatus = LastYawNvmData.CalStatus;
                nvout->YwRate.ZeroRateMin = LastYawNvmData.ZeroRateMin;
                nvout->YwRate.ZeroRateMax = LastYawNvmData.ZeroRateMax;
            }
#endif

            /* First cycle done with valid yaw rate, filter are initialised now
             */
            YawRateData->YwFirstCycleDone = TRUE;
        } else {
            /* Yaw rate invalid */
            /* Initialise general yaw rate and standstill offset data to reset
             * filter values and distance */
            VED_YwrInitYawRateData(YawRateData);
        }
    } else {
        /*<--- Execution path for initialization mode  --->*/
        VED_YwrCalcRunnigTime();
        VED_YwrCommonInit();
#if ((defined(CFG_VED__NVM_LEARN_DATA_ERROR)) && \
     (CFG_VED__NVM_LEARN_DATA_ERROR))
        /* check for valid values written into NVM */
        YWR_GET_ME->Io.YwrNVMerrOffsRg = NULL;
#endif
    }

    /* Fill the autocode interface */
    VED_YwrCalcToAutocode(YWR_GET_OFFS);

    return;
}

/* **********************************************************************
  @fn               VED_YwrInit */ /*!
  @brief            initialize module data

  @description      see brief description

  @param            reqPorts
  @param            input
  @param            mif
  @return           void

  @pre              -
  @post             -
**************************************************************************** */

void VED_YwrInit(const reqVEDPrtList_t *reqPorts,
                 const proVEDPrtList_t *proPorts) {
    VED_YwrCommonInit();
    VED_YwrOffsetInit(reqPorts->pNVMRead);
#if ((defined(CFG_VED__TURNTABLE_DETECTION)) && (CFG_VED__TURNTABLE_DETECTION))
    PrevYawRateOffset = 0.F;
#endif

#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
    {
        uint32 ioStateYaw;
        /* YAW offset writing in init mode */
        if (reqPorts->pNVMRead->State == NVM_CLEARED) {
            LastYawNvmData.ZeroRate = 0.F;
            LastYawNvmData.CalStatus = 0U;
            LastYawNvmData.ZeroRateMax = 0.F;
            LastYawNvmData.ZeroRateMin = 0.F;

            proPorts->pNVMWrite->YwRate.ZeroRate = 0.F;
            proPorts->pNVMWrite->YwRate.CalStatus = 0U;
            proPorts->pNVMWrite->YwRate.ZeroRateMax = 0.F;
            proPorts->pNVMWrite->YwRate.ZeroRateMin = 0.F;
        } else {
            LastYawNvmData.ZeroRate = reqPorts->pNVMRead->YwRate.ZeroRate;
            LastYawNvmData.CalStatus = reqPorts->pNVMRead->YwRate.CalStatus;
            LastYawNvmData.ZeroRateMax = reqPorts->pNVMRead->YwRate.ZeroRateMax;
            LastYawNvmData.ZeroRateMin = reqPorts->pNVMRead->YwRate.ZeroRateMin;

            proPorts->pNVMWrite->YwRate.ZeroRate =
                reqPorts->pNVMRead->YwRate.ZeroRate;
            proPorts->pNVMWrite->YwRate.CalStatus =
                reqPorts->pNVMRead->YwRate.CalStatus;
            proPorts->pNVMWrite->YwRate.ZeroRateMax =
                reqPorts->pNVMRead->YwRate.ZeroRateMax;
            proPorts->pNVMWrite->YwRate.ZeroRateMin =
                reqPorts->pNVMRead->YwRate.ZeroRateMin;

            ioStateYaw = VED__GET_NVM_IO_STATE(VED_NVM_POS_YWR,
                                               &reqPorts->pNVMRead->State);
            VED_SET_NVM_IO_STATE(VED_NVM_POS_YWR, ioStateYaw,
                                 &proPorts->pNVMWrite->State);
        }
    }

#else

    (void)proPorts; /* remove compiler warning, proPorts is not used in this
                       configuration */

#endif

    return;
}

#if ((defined(CFG_VED__NVM_LEARN_DATA_ERROR)) && \
     (CFG_VED__NVM_LEARN_DATA_ERROR))
/* ***********************************************************************/ /*!
  Functionname:           VED_YwrOffsetRangeOk
  @brief                  Checks YawRate offset range

  @return                 Offset OK = TRUE, NOT_OK = FALSE
  @param[in]              Offset value
  @param[out]             -
**************************************************************************** */
static boolean VED_YwrOffsetRangeOk(float32 Offset) {
    boolean YwrOffsOk = FALSE;

    /* check the offset range */
    if (fABS(Offset) <= VED__PAR_YWR_OFFSET_LIMIT_MAX) {
        /* offset within range  */
        YwrOffsOk = TRUE;
    } else {
        /* offset outside of range */
        YwrOffsOk = FALSE;
    }

    return (YwrOffsOk);
}

#endif
/* **********************************************************************
  @fn                     VED_YwrCalcToAutocode */ /*!
  @brief                  Fill the interface to the autocode

  @description            see brief description

  @param[in]              Offs
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_YwrCalcToAutocode(VED_YwrOffsData_t *Offs) {
    Offs->ToAutocode.OffsData.quality = 1.5F;
    Offs->ToAutocode.OffsData.var = 0.000000001F;

    /* Detection of yaw rate offset changes */
#if (CFG_VED__EX_YWR_NVM)
    if (((Offs->OffsElpsdTime < (float32)1.0F) &&
         (Offs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST)) ||
        ((Offs->OffsType == (VED_YwrOffsType_t)OFFS_TEMPER_TABLE) ||
         (Offs->OffsType == (VED_YwrOffsType_t)OFFS_DYN_APPRX) ||
         (Offs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST_EEPROM)) ||
        ((YWR_GET_MIF_DATA->FirstCycleDone == FALSE) &&
         (Offs->OffsType != (VED_YwrOffsType_t)OFFS_NON)))
    /* offset is checked for learning for first cycle AT363159 */
#else
    if (((Offs->OffsElpsdTime < (float32)1.0F) &&
         (Offs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST)) ||
        ((Offs->OffsType == (VED_YwrOffsType_t)OFFS_TEMPER_TABLE) ||
         (Offs->OffsType == (VED_YwrOffsType_t)OFFS_DYN_APPRX)) ||
        ((YWR_GET_MIF_DATA->FirstCycleDone == FALSE) &&
         (Offs->OffsType != (VED_YwrOffsType_t)OFFS_NON)))
    /* offset is checked for learning for first cycle AT363159 */
#endif
    {
        /* new offset was taken */
        /* set state */
#if (CFG_VED__EX_YWR_NVM)
        if (Offs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST_EEPROM) {
            /* state 2 = EEPROM offset */
            Offs->ToAutocode.OffsData.state = VED__YAWRATE_STATE_NVM;
        } else
#endif
        {
            /* state = 1 -> set dynamic offset to stand still offset */
            Offs->ToAutocode.OffsData.state = VED__YAWRATE_STATE_STANDSTILL;
        }
        Offs->ToAutocode.OffsData.offset = Offs->YawRateOffset;
    } else {
        if (Offs->OffsType == (VED_YwrOffsType_t)OFFS_NON) {
            /* state = 3 -> yaw stand still offset never ever estimated */
            Offs->ToAutocode.OffsData.offset = 0.0F;
            Offs->ToAutocode.OffsData.state = VED__YAWRATE_STATE_NOT_ESTIMATED;
        } else {
            Offs->ToAutocode.OffsData.state = 0U;
        }
    }

    /* Check if the offset is a dynamic offset and set the dynamic flag */
    if ((Offs->OffsType != (VED_YwrOffsType_t)OFFS_DYN_INTER) &&
        (Offs->OffsType != (VED_YwrOffsType_t)OFFS_DYN_AVG)) {
        Offs->ToAutocode.IsDynamic = FALSE;
    } else {
        Offs->ToAutocode.IsDynamic = TRUE;
    }
}

#if (CFG_VED__YW_DYN_AVG)
/* **********************************************************************
  @fn                     VED_YwrCalcDynOffsAvg */ /*!
  @brief                  Offsetermittlung ueber gemittelte Gierrate
                          Annahme ist, dass auf laengere Strecke im Mittel 
                          geradeaus gefahren wird
  @description            see brief description

  @param[in]              YawRateData
  @param[out]             DynOffset
  @param[out]             Offs
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static float32 VED_YwrCalcDynOffsAvg(const VED_YwrSenData_t *YawRateData,
                                     VED_YwrDynOffsAvg_t *DynOffset,
                                     const VED_YwrOffsData_t *Offs) {
    /*--- FUNKTIONSLOKALE KONSTANTEN ---*/

    /* Mittelwert Filter-Konstanten. Filterung ueber Weg d. h. TS = 1sec
     * enspricht 40 m */

    /* IIR-Tiefpass-Filterkonstanten Mittlerwert-Filter (Butterworth) */
    static const VED__IIR2Coeff_t yw1s_c = /* FC = 0.0007 Hz, FS = 1.0 Hz */
        {
            {1.0F * 0.48211046862777198e-5F,
             1.000000000000000F}, /* num0, den0 */
            {2.0F * 0.48211046862777198e-5F,
             -1.993779983847134F}, /* num1, den1 */
            {1.0F * 0.48211046862777198e-5F,
             0.993799268265879F}, /* num2, den2 */
        };

    /* IIR-Tiefpass-Filterkonstanten Detektion-Filter (Chebyshev Type II) */
    static const VED__IIR2Coeff_t yw2s_c = /* FC = 0.07 Hz, FS = 1.0 Hz */
        {
            {1.0000000000F * 0.3262393735e-2F, 1.00000000000F}, /* num0, den0 */
            {-1.8089025526F * 0.3262393735e-2F,
             -1.96443350097F},                                  /* num1, den1 */
            {1.0000000000F * 0.3262393735e-2F, 0.96505693608F}, /* num2, den2 */
        };

    /* IIR-Tiefpass-Filterkonstanten Iterim-Filter (Chebyshev Type II) */
    static const VED__IIR2Coeff_t
        yw3s_c = /* FC = 0.05 Hz, FS = 1.0 Hz, Astop = 60 dB */
        {
            {1.00000000000F * 0.1039709422e-2F,
             1.00000000000F}, /* num0, den0 */
            {-1.80890255282F * 0.1039709422e-2F,
             -1.97997659247F}, /* num1, den1 */
            {1.00000000000F * 0.1039709422e-2F,
             0.98017527828F}, /* num2, den2 */
        };

    enum { FIR_LENGTH = 20 }; /* Laenge des FIR-Vorfilters              */
    const float32 YwRateMax_c =
        DEG2RAD(10.F); /* Max. Gierrate fuer Filterung           */
    const float32 DistMax_c =
        15000.F; /* Max. Distanz in m fuer Gueteberechnung */
    const float32 DistThrdOvrMin_c =
        3000.F; /* Mindest-Distanz ueber Threshold        */
    const float32 DistThrdMin_c = 15000.F; /* Mindest-Distanz ueber Threshold */

    const float32 VehSpeedMin_c =
        30.F / C_KMH_MS; /* Mindestgeschwindigkeit fuer Filterung  */
    const float32 CrvStPreMin_c =
        1.F / (500.F); /* Grenzradius fuer Kurvenfahrt           */
    const float32 OffsTime_c =
        0.2F; /* Offset-Zeit-Schwelle zur Detektion neuer Offset */
    const float32 OffsTimeStart_c = 1.F; /* Offset-Zeit-Schwelle zur Detektion
                                            neuer Offset waehrend Startup */
    const float32 YwOffsetMax_c = DEG2RAD(5.F); /* Max. moeglicher Offsetwert */

    /*--- FUNKTIONSLOKALE VARIABLEN ---*/

    /*---- static -----*/
    static float32 YwFiltBuf[FIR_LENGTH] = /* Buffer fuer FIR-Vorfilter */
        {
            0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F,
            0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F,
        };

    /*---- non-static ----- */
    float32 filtYwRate = 0.F; /* Vorgefilterte Gierrate (dezimation) */
    float32 CycleTime;        /* Zykluszeit                          */
    float32 distRet =
        0.F; /* Distanz die in Filterung eingeht als Rueckgabewert  */
    float32 VehicleSpeed; /* Eigengeschwindigkeit korrigiert     */
    float32 OffsTimeThrd;

    /* Lenkwinkel-Daten */
    const VED_OutCurve_t SwaCurve = {0.F, 0.F, 0.F};
    const VED_OutCurve_t WspCurve = {0.F, 0.F, 0.F};

    const VED_OutCurve_t *StrgTrack; /* Lenkwinkel-Spur        */
    const VED_OutCurve_t *WspTrack;  /* Raddrehzahlen-Spur     */

    /* Zykluszeit holen */
    CycleTime = VED_GetCycleTime();

    /* Eigengeschwindikeit holen */
    VehicleSpeed = YWR_GET_MIF_DATA->LongMot.VehVelocityCorr;

    /* Referenzen fuer Lenkwinkelspur-Daten  initialisieren */
    StrgTrack = &SwaCurve;

    /* Raddrehzahl Spur */
    WspTrack = &WspCurve;

    /* Bei erstem Zyklus FIR-Filter Zaehler und akkumulierte Distanz
     * zuruecksetzen */
    if (YawRateData->YwFirstCycleDone == FALSE) {
        DynOffset->cntCycle = 0UL;
        DynOffset->accYwDist = 0.F;
    }

    /* Solange kein Filterwert gelernt ist, Qualitaet des Lenkwinkel
     * aktualisieren */
    /* Hinweis: derzeit kein Lernen des Filterwerts, damit keine Abhaengigkeit
     * vom Lenkwinkel und Rundkurserkennung deaktiviert */
    DynOffset->FlStwAStat = FALSE;

    /* Mindestgeschwindigkeit pruefen */
    if (VehicleSpeed > VehSpeedMin_c) {
        const float32 CurvMin_c =
            1.F / (1800.F); /* Mindestkruemmung fuer Richtungsbestimmung */

        float32 drvDist; /* Gefahrende Distanz pro Zyklus */

        /* Gefahrene Wegstrecke pro Zyklus */
        drvDist = (VehicleSpeed * CycleTime);

        /*  Charakteristik mit ausreichendem LW-Status moeglich*/
        if (DynOffset->FlStwAStat != FALSE) {
            VED_CrvDirStatus_t
                WspStat; /* Richtung Links/Rechts aus RDZ-Kruemmung */
            VED_CrvDirStatus_t
                StrgStat; /* Richtung Links/Rechts aus LW-Kruemmung  */
            const float32 QMinWspCrv_c =
                0.3F; /* Mindestqualitaet WSP Kruemmung    */
            const float32 QMinStwCrv_c =
                0.4F; /* Mindestqualitaet STW Kruemmung    */

            /* Streckencharakteristik ueber Lenkwinkel oder Raddrehzahl-Spur
             * bestimmen */
            WspStat = VED_GetCurveDir(WspTrack, CurvMin_c, QMinWspCrv_c);
            StrgStat = VED_GetCurveDir(StrgTrack, CurvMin_c, QMinStwCrv_c);

            /* Kurve links */
            if ((StrgStat == (VED_CrvDirStatus_t)CRV_DIR_LEFT) ||
                (WspStat == (VED_CrvDirStatus_t)CRV_DIR_LEFT)) {
                DynOffset->StDrv -= drvDist;
            }

            /* Kurve rechts */
            if ((StrgStat == (VED_CrvDirStatus_t)CRV_DIR_RIGHT) ||
                (WspStat == (VED_CrvDirStatus_t)CRV_DIR_RIGHT)) {
                DynOffset->StDrv += drvDist;
            }

            /* Kursrichtung kann nicht bestimmt werden */
            if ((WspStat == (VED_CrvDirStatus_t)CRV_DIR_DONT_KNOW) &&
                (StrgStat == (VED_CrvDirStatus_t)CRV_DIR_DONT_KNOW)) {
                DynOffset->StDrv = 0.F;
            }
        } else {
            DynOffset->StDrv = 0.F;
        }

        /* Bei vorhandenem Lenkradwinkel-Offset ausreichender Guete Grenzradius
         * pruefen */
        if ((fABS(StrgTrack->Curve) < CrvStPreMin_c) ||
            (!(DynOffset->FlStwAStat != FALSE))) {
            /* Gleitende Mittelwertbildung der Gierrate als Vorfilterung */
            if ((fABS(YawRateData->YawRate) < YwRateMax_c) &&
                (YawRateData->Valid != FALSE)) {
                uint32 ii;
                /* Weg aufintegrieren */
                DynOffset->accYwDist += drvDist;

                /* Filterspeicherwerte nach rechts schieben */
                for (ii = ((uint32)FIR_LENGTH - 1U); ii > 0U; ii--) {
                    YwFiltBuf[ii] = YwFiltBuf[ii - 1U];
                }

                /* Eingangswert einlesen */
                YwFiltBuf[0] = YawRateData->YawRate;

                /* Gleitenden Mittelwert berechnen */
                for (ii = 0UL; ii < (uint32)FIR_LENGTH; ii++) {
                    filtYwRate += YwFiltBuf[ii];
                }
                filtYwRate *= 1.F / (float32)FIR_LENGTH;

                if (DynOffset->cntCycle < (uint32)FIR_LENGTH) {
                    DynOffset->cntCycle++;
                }
            }

            /* Alterung erst ab einer Mindestdistanz und
             * Mindestlenkwinkel-Status */
            if ((DynOffset->FlStwAStat != FALSE) &&
                (DynOffset->YwDist >= DistMax_c)) {
                const float32 StDrvAgingThrd_c =
                    4000.F; /* Schwelle ab der Streckensymmertrie gealtert wird
                             */
                const float32 DistRedMin_c =
                    10000.F; /* Mindestdistanz zur Symmertrie-Alterung */

                /* Allgemein gefahrende Wegstrecke */
                DynOffset->DrvDistRed += drvDist;

                if ((fABS(DynOffset->StDrv) > StDrvAgingThrd_c) &&
                    (DynOffset->DrvDistRed > DistRedMin_c)) {
                    const float32 StrDrvRed_c =
                        500.F; /*  Granularitaet der Reduzierung */

                    if (DynOffset->StDrv > StrDrvRed_c) {
                        DynOffset->StDrv -=
                            StrDrvRed_c; /* Oberhalb positiver Grenze  */
                    } else if (DynOffset->StDrv < -StrDrvRed_c) {
                        DynOffset->StDrv +=
                            StrDrvRed_c; /* Unterhalb negativer Grenze */
                    } else {
                        DynOffset->StDrv =
                            0.F; /* Innerhalb oberer und unterer Grenze */
                    }
                    DynOffset->DrvDistRed = 0.F; /* Bei erfolgter Reduzierung
                                                    Distanz zuruecksetzen */
                } else {
                    const float32 DistRedMax_c =
                        15000.F; /* Maximaldistanz zur Symmertrie-Alterung */

                    /* Gefahrene Distanz zum reduzieren der Asymmertrie
                     * begrenzen */
                    DynOffset->DrvDistRed =
                        MIN(DynOffset->DrvDistRed, DistRedMax_c);
                }
            } else {
                DynOffset->DrvDistRed = 0.F; /* Reduzierung erst aktiv, wenn
                                                max. Distanz erreicht */
            }
        }
    }

    /* Langzeitmittelung fuer Gierrate ueber gefahrenem Weg ausfuehren ds = 40 m
     */
    if ((DynOffset->cntCycle >= (uint32)FIR_LENGTH) &&
        (DynOffset->accYwDist > YWR_DYNA_DIST_STEP)) {
        /* Filter nur aufrufen, wenn vorgefilterte Gierrate innerhalb der
         * Grenzen */
        if (fABS(filtYwRate) < YwRateMax_c) {
            VED__IIR_FILTER2(filtYwRate, DynOffset->YawRateOffset, yw1s_c,
                             DynOffset->YwFilt); /* Abgleichfilter   */

            VED__IIR_FILTER2(filtYwRate, DynOffset->YawRateDet, yw2s_c,
                             DynOffset->YwFiltDet); /* Detektionsfilter */

            /* Interimsfilter  */

            VED__IIR_FILTER2(filtYwRate, DynOffset->IntOffs.YawRateOffset,
                             yw3s_c, DynOffset->IntOffs.YwFiltInt);

            DynOffset->YwDist += DynOffset->accYwDist;
            DynOffset->YwDist =
                MIN(DistMax_c,
                    DynOffset->YwDist); /*  Aufintegrieren mit Saettigung */
        } else {
            DynOffset->YwDist -= DynOffset->accYwDist;
            DynOffset->YwDist =
                MAX(0.0f, DynOffset->YwDist); /* Abintegrieren mit Saettigung */
        }

        /* Ueberschreiten der Schwelle zur Aktivierung von diesem Offset-Typ
           muss fuer eine bestimmte Wegstrecke erfuellt sein */
        if (DynOffset->YwDist >= DistThrdMin_c) {
            const float32 YwOffsThrFirst_c =
                DEG2RAD(0.37F); /* Schwelle fuer erstmalige Uebernahme */
            const float32 YwOffsThrCons_c =
                DEG2RAD(0.1F); /* Schwelle fuer Folge-Uebernahmen     */

            /* Wenn dynamischer Offsettyp noch nicht aktiv ist, zusaetzliche
               Threshold- Bedingung fuer erstmalige Uebernahme aktiv */
            if ((Offs->OffsType != (VED_YwrOffsType_t)OFFS_DYN_AVG) &&
                (Offs->OffsType != (VED_YwrOffsType_t)OFFS_DYN_INTER)) {
                if (fABS(Offs->YawRateOffset - DynOffset->YawRateOffset) >
                    YwOffsThrFirst_c) {
                    DynOffset->YwDrvDistThr +=
                        DynOffset->accYwDist; /* Inkrementieren  */
                } else {
                    /* Dekrementieren bis auf Null */
                    DynOffset->YwDrvDistThr = MAX(
                        0.0F, DynOffset->YwDrvDistThr - DynOffset->accYwDist);
                }
            } else {
                /* Dynamischer Average oder Interims-Offset bereits wirksam */
                const float32 DistThrdOvr_c =
                    DistThrdOvrMin_c + 1.0F; /* Wert ueber Schwelle setzen */

                /* Bei Folgeabgleich Schwellenbedingung deaktivieren */
                if (fABS(Offs->YawRateOffset - DynOffset->YawRateOffset) >=
                    YwOffsThrCons_c) {
                    DynOffset->YwDrvDistThr =
                        DistThrdOvr_c; /* Bei Folgeuebernahmen Bedingung auf
                                          TRUE */
                }
            }
        } else {
            /* Eingriffschwelle zuruecksetzen, wenn gefahrene Distanz zu gering
             */
            DynOffset->YwDrvDistThr = 0.F;
        }
        distRet = DynOffset->accYwDist; /* Distanz-Schritt als Rueckgabe-Wert
                                           speichern      */
        DynOffset->accYwDist =
            0.F; /* Distanz-Akummulation des Vorfilters zuruecksetzen */
    } else {
        distRet = 0.F;
    }

    /* Streckencharakteristik ueber Lenkwinkel plausibilisieren */
    if (fABS(DynOffset->StDrv) > YWR_STDIST_SYMM_RESET) {
        /* Wenn dynamischer Offset wirksam, Abgleichfilter nicht komplett
         * zuruecksetzen */
        if (Offs->OffsType == (VED_YwrOffsType_t)OFFS_DYN_AVG) {
            DynOffset->YwDist = MIN(DistMax_c / 2.F, DynOffset->YwDist);
            DynOffset->MaxQuality = 0.F;
            DynOffset->StDrv = 0.F;
            DynOffset->YwDrvDistThr = 0.F;
            DynOffset->DrvDistRed = 0.F;
        } else {
            /* Offsetdaten reinitialisieren */
            VED_YwrInitDynOffsAvg(DynOffset);
            VED_YwrInitDynOffsInterim(&DynOffset->IntOffs);
        }
    }

    /* Waehrend der Startup-Phase erhoehte Zeitschwelle notwendig, um
     * Offset-Uebernahme zu detektieren */
    if (Offs->EcuElpsdTime < OffsTimeStart_c) {
        OffsTimeThrd = OffsTimeStart_c;
    } else {
        /* Im Falle einer ungueltigen Drehrate ist Offsetberechnung nicht aktiv,
           daher muss die Zeitschwelle entsprechend erhoeht werden, um einen
           Offsetwechsel noch ueber die Offsetzeit noch erkennen zu koennen */
        OffsTimeThrd =
            OffsTime_c + (Offs->EcuElpsdTime - DynOffset->LastEcuTime);
    }

    /* Momentane ECU-Time speichern fuer naechsten Zyklus */
    DynOffset->LastEcuTime = Offs->EcuElpsdTime;

    /* Bei erfolgreichem Offset Abgleich anderen Typs und zu grossem Offsetwert
     * Filter neu initialisieren */

    /* Zur Erkennung wird die Offsetdauer des wirksamen Offsets oder CycleDone
     * verwendet */
    if ((fABS(DynOffset->YawRateOffset) >
         YwOffsetMax_c) /* Gelernter Offsetwert (absolut) plausibilisieren */
        || ((Offs->OffsElpsdTime < OffsTimeThrd) &&
            ((Offs->OffsType ==
              (VED_YwrOffsType_t)OFFS_STANDST) /* Ablgeich ueber Stillstand  */
#if (CFG_VED__EX_YWR_NVM)
             || (Offs->OffsType ==
                 (VED_YwrOffsType_t)OFFS_STANDST_EEPROM) /* EEPROM */
#endif
             ))) {
        /* Dyn. Average Offset reinitialisieren */
        VED_YwrInitDynOffsAvg(DynOffset);

        /* Interims-Offset initialisieren */
        VED_YwrInitDynOffsInterim(&DynOffset->IntOffs);
    }

    /* Bei Uebernahme eines wirksamen Offsets Uebernahme-Schwelle zuruecksetzen
       um sofortige Folgeuebernahmen zu vermeiden */
    if ((Offs->OffsElpsdTime < OffsTime_c) &&
        (Offs->OffsType != (VED_YwrOffsType_t)OFFS_DYN_INTER)) {
        DynOffset->YwDrvDistThr = 0.F;
        DynOffset->StDrv =
            0.F; /* Verhinderung von Ruecksetzen kurz nach Folge-Uebernahme */
    }

    /* Guete in Abhaengigkeit von gefahrene Distanz ueber Einschreitschwelle
       und Lenkwinkel-Spur Charakteristik (wenn vorhanden, ansonsten null) */
    DynOffset->MaxQuality = 0.F; /* Defaultwert initialisieren */

    if ((DynOffset->YwDist >= DistMax_c) &&
        (DynOffset->YwDrvDistThr > DistThrdOvrMin_c)) {
        /* Bei aktivem Interims-Offset nur Ableich im Normal-Zustand und kleiner
         * Abweichung  */
        if ((Offs->OffsType != (VED_YwrOffsType_t)OFFS_DYN_INTER) &&
            (DynOffset->FlStwAStat != FALSE)) {
            /* Guete bei vorhandener Streckencharakteristik (Reduzierung ueber
             * Unsymmetrie) bewerten */
            DynOffset->MaxQuality =
                VED_LFunction(fABS(DynOffset->StDrv), YWR_DIST_DYNAVGOFF_MIN,
                              YWR_DIST_DYNAVGOFF_MAX, YWR_QUALITY_DYNAVGOFF_MAX,
                              YWR_QUALITY_DYNAVGOFF_MIN);
        } else {
            if ((fABS(DynOffset->YawRateOffset -
                      DynOffset->IntOffs.YawRateOffset) < DEG2RAD(0.3F)) &&
                (Offs->OffsType == (VED_YwrOffsType_t)OFFS_DYN_INTER)) {
                /* Im Initialzustand soll durch diesen Filter die langsame
                   Drifts kompensiert werden,
                   was durch die Ungenauigkeiten des Interimsfilter nicht
                   moeglich ist */
                DynOffset->MaxQuality = 1.0F;
            }
        }
    }
    return distRet;
}

/* **********************************************************************
  @fn                     VED_YwrCalcDynOffsInterim */ /*!
  @brief                  Dynamischer Offset fuer initialisierten Lenkwinkel-
                          Status
                          
  @description            see brief description

  @param[in]              DynOffsInter
  @param[in]              Offs
  @param[in]              dist
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrCalcDynOffsInterim(VED_YwrDynOffsInter_t *DynOffsInter,
                                      const VED_YwrOffsData_t *Offs,
                                      float32 dist) {
    /* Pruefen, ob Filterung ueber zurueckgelegtem Weg aktualisiert worden ist
       d. h. Abgleich- und Detektionsfilter wurde aktualisiert */
    if (dist > 0.F) {
        /* Defaultwert initialisieren */
        DynOffsInter->MaxQuality = 0.F;

        /* Abgleich nur moeglich wenn Lenkwinkel-Offset unzureichend gelernt und
         * Mindestdistanz gefahren  */
        if ((Offs->DynOffsAvg.FlStwAStat == FALSE) &&
            (Offs->DynOffsAvg.YwDist > VED__YWR_DYN_OFFSET_MIN_DIST)) {
            const float32 diffMaxDetInter_c =
                DEG2RAD(0.7F); /* Maximal-Differenz Detektionsfilter- und
                                  Interimsfilter */
            boolean preCond;   /* Vorbedingung */
            float32 diffIntCurr;

            diffIntCurr = fABS(Offs->DynOffsAvg.IntOffs.YawRateOffset -
                               Offs->YawRateOffset);

            /* Aussummieren der Strecke mit Abweichung ueber Schwelle */
            if (diffIntCurr > DEG2RAD(0.5F)) {
                DynOffsInter->DistThrd += dist;
            } else {
                /* Dekrementieren bis auf Null */
                DynOffsInter->DistThrd =
                    MAX(0.0F, DynOffsInter->DistThrd - dist);
            }

            /* Abgleichbedingungen in Abhaengigkeit des momentan wirksamen
             * Offsets pruefen */
            if (Offs->OffsType != (VED_YwrOffsType_t)OFFS_DYN_INTER) {
                const float32 diffAvgCurr_c =
                    DEG2RAD(0.2F); /* Mindest-Differenz Mittelwert zu wirksamen
                                      Offset   */
                const float32 diffIntCurr_c =
                    DEG2RAD(0.4F); /* Mindest-Differenz Interimswert zu
                                      wirksamen Offset */

                float32 diffAvgCurr;
                boolean b_diffSignEqual;

                diffAvgCurr =
                    fABS(Offs->DynOffsAvg.YawRateOffset - Offs->YawRateOffset);
                diffIntCurr = fABS(Offs->DynOffsAvg.IntOffs.YawRateOffset -
                                   Offs->YawRateOffset);

                if (((diffAvgCurr > 0.0F) && (diffIntCurr > 0.0F)) ||
                    ((diffAvgCurr < 0.0F) && (diffIntCurr < 0.0F))) {
                    b_diffSignEqual = TRUE;
                } else {
                    b_diffSignEqual = FALSE;
                }

                preCond = (boolean)((DynOffsInter->DistThrd >
                                     VED__YWR_DYN_OFFSET_MIN_DIST) &&
                                    (diffAvgCurr > diffAvgCurr_c) &&
                                    (diffIntCurr > diffIntCurr_c) &&
                                    (b_diffSignEqual == TRUE));

            } else {
                /* Folgeuebernahmen mit nicht so strengen Kriterien um Drift
                 * besser zu folgen */
                float32 f_OffsetDiffMinInterAbs;
                float32 f_OffsetDiffAvgDetAbs;

                const float32 diffMinInter_c =
                    DEG2RAD(0.2F); /* Mindest-Differenz Interims-Filter */
                const float32 diffMinAvgDet_c =
                    DEG2RAD(0.3F); /* Mindest-Differenz zwischen Mittelwert- und
                                      Detektionsfilter fuer thermisch
                                      eingeschwungenen Zustand */
                f_OffsetDiffMinInterAbs =
                    fABS(Offs->YawRateOffset - DynOffsInter->YawRateOffset);
                f_OffsetDiffAvgDetAbs = fABS(Offs->DynOffsAvg.YawRateDet -
                                             Offs->DynOffsAvg.YawRateOffset);

                preCond =
                    (boolean)((f_OffsetDiffMinInterAbs > diffMinInter_c) &&
                              (f_OffsetDiffAvgDetAbs > diffMinAvgDet_c));
            }

            if ((fABS(Offs->DynOffsAvg.YawRateDet -
                      DynOffsInter->YawRateOffset) < diffMaxDetInter_c) &&
                (preCond == TRUE)) {
                /* Abgleich aktivieren */
                DynOffsInter->MaxQuality = 1.0F;
            }
        }
    }
    return;
}

/* **********************************************************************
  @fn                     VED_YwrCalcQRedRamp */ /*!
  @brief                  Rampe fuer Guete-Reduzierung bzw.Interims-Mittelung
                          
  @description            see brief description

  @param[in]              OffsData
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static float32 VED_YwrCalcQRedRamp(const VED_YwrOffsData_t *OffsData) {
    float32 retRamp;

    /* Differenz zur Steuerung des Grades der Guetereduzierung */
    float32 diffYw =
        fABS(OffsData->DynOffsAvg.YawRateDet - OffsData->YawRateOffset);

    /* Zweistufige Rampe zur Guetereduzierung */
    if (diffYw < YWR_OFFSET_DELTA_MED) {
        retRamp =
            VED_LFunction(diffYw, YWR_OFFSET_DELTA_MIN, YWR_OFFSET_DELTA_MED,
                          YWR_Q_FACTOR_DELTA_MAX, YWR_Q_FACTOR_DELTA_MED);
    } else {
        retRamp =
            VED_LFunction(diffYw, YWR_OFFSET_DELTA_MED, YWR_OFFSET_DELTA_MAX,
                          YWR_Q_FACTOR_DELTA_MED, YWR_Q_FACTOR_DELTA_MIN);
    }
    return retRamp;
}

/* **********************************************************************
  @fn                     VED_YwrMergeOffsetDrift */ /*!
  @brief                  Fusionieren von bisher wirksamen Offset mit Interims
                          Offset bei starker Drift, da Guetereduzierung nicht
                          ausreichend
                          
  @description            see brief description

  @param[in]              OffsData
  @param[in]              DynOffsCache
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrMergeOffsetDrift(VED_YwrOffsData_t *OffsData,
                                    VED_YwrDynOffsCache_t *DynOffsCache) {
    float32 qRed;
    float32 qRedMax;

    /* Cache mit momentan wirksamen Offset initialisieren */
    DynOffsCache->YawRateOffset = OffsData->YawRateOffset;

    /* Moegliche Guete-Reduzierung berechnen */
    qRed = VED_YwrCalcQRedRamp(OffsData);

    /* Unterscheidung zwischen gelerntem Zustand und Initialzustand */
    if ((OffsData->DynOffsAvg.QReduce >=
         1.F) /* Guete-Reduzierung ist nicht aktiv  */
        && (qRed <
            1.F) /* Guete wuerde bei ausreichend LW,WSP Qualitaet reduziert */
        && (OffsData->DynOffsAvg.YwDist >
            YWR_DYNC_QRED_DIST_MIN)) /* Staerkere Wirkung erst zulassen, wenn
                                        Filter eingeschwungen */
    {
        /* max. Wirkung des Offsets zulassen */
        qRedMax = VED__PAR_YW_Q_THRLD_SWA_UC; /* mind. Qualitaet fuer
                                                 Gesamt-Gewichtung */
    } else {
        qRedMax = VED__PAR_YW_Q_THRLD_SWA_CAL;
    }

    /* Qualitaet berechnen auf Mindestniveau halten, damit Cache-Offset wirksam
     * werden kann */
    DynOffsCache->Quality = MAX(qRed, qRedMax);

    /* Reduzierung des Einflusses des Offsets bei Verdacht auf Rundkurs */
    DynOffsCache->Quality *= VED_LFunction(
        fABS(OffsData->DynOffsAvg.StDrv), YWR_DYNC_STSYM_START,
        YWR_DYNC_STSYM_END, YWR_DYNC_Q_PERM_MAX, YWR_DANC_Q_PERM_MIN);

    /* Bedingungen fuer das teilweise Einblenden des Interims-Offset ohne
     * Abgleich */
    if ((qRed < qRedMax) &&
        (OffsData->OffsType != (VED_YwrOffsType_t)OFFS_DYN_INTER)) {
        /* Anpassen der Qualitaet des wirksamen Offset, damit Ueberlagerung
         * wirksam wird */
        OffsData->Quality = OffsData->QualNoRed * DynOffsCache->Quality;

        /* Differenz zwischen wirksamen und Interims-Offset gewichtet
         * ueberlagern */
        DynOffsCache->YawRateOffset +=
            (OffsData->DynOffsAvg.IntOffs.YawRateOffset -
             OffsData->YawRateOffset) *
            (1.0F - qRed);
    }
    return;
}
#endif /* #if (CFG_VED__YW_DYN_AVG) */

/* **********************************************************************
  @fn                     VED_YwrInitYawRateData */ /*!
  @brief                  Initialisiert Drehratendaten

  @description            see brief description

  @param[in]              YawRateData
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrInitYawRateData(VED_YwrSenData_t *YawRateData) {
    YawRateData->DeltaDist = 0.F;
    YawRateData->Gradient = 0.F;
    YawRateData->GradientAbsOld = 0.F;
    YawRateData->YawRate = 0.F;
    YawRateData->YawRateCurveFilt1 = 0.F;
    YawRateData->YawRateCurveFilt2 = 0.F;
    YawRateData->YawRateCurveFilt3 = 0.F;
    YawRateData->YawRateOld = 0.F;
    YawRateData->YwCurveOld = 0.F;
    YawRateData->Valid = FALSE;
    YawRateData->YwFirstCycleDone = FALSE;
    YawRateData->FilterTime = 0.F;

    return;
}

/* **********************************************************************
  @fn                     VED_YwrInitStandStillOffset */ /*!
  @brief                  Initialises Dyn. Offset

  @description            see brief description

  @param[in]              StandStOffs
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_YwrInitStandStillOffset(VED_YwrStandStillOffs_t *StandStOffs) {
    VED_StatIntervalInit(&StandStOffs->SampleInterval_1);
    VED_StatIntervalInit(&StandStOffs->SampleInterval_2);
    VED_StatIntervalInit(&StandStOffs->SampleInterval);

    StandStOffs->AdjustTime = 0.F;
    StandStOffs->MaxQuality = 0.F;
    StandStOffs->YawRateOffset = 0.F;
    StandStOffs->StandstillOK = FALSE;

    return;
}

#if (CFG_VED__EX_YWR_NVM)
/* **********************************************************************
  @fn                     VED_YwrInitStandStillEepromOffset */ /*!
  @brief                  Initialises Eeprom-Offset

  @description            see brief description

  @param[in]              StandStEepromOffs
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrInitStandStillEepromOffset(
    VED_YwrStandStillEepromOffs_t *StandStEepromOffs) {
    StandStEepromOffs->YawRateOffset.Offset1 = 0.F;
    StandStEepromOffs->YawRateOffset.Status1 = (uint8)0UL;
    StandStEepromOffs->YawRateOffset.Offset2 = 0.F;
    StandStEepromOffs->YawRateOffset.Status2 = (uint8)0UL;
    StandStEepromOffs->TimeLastWrittenEepromOffset = YWR_OFFS_ELPSD_TIME_MAX;
    StandStEepromOffs->MaxQuality = 0.F;
    StandStEepromOffs->NumOfWrittenOffsets = (uint8)0UL;

    return;
}
#endif

#if (CFG_VED__YW_DYN_AVG)
/* **********************************************************************
  @fn                     VED_YwrInitDynOffsAvg */ /*!
  @brief                  Initialisiert

  @description            see brief description

  @param[in]              DynOffsAvg
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_YwrInitDynOffsAvg(VED_YwrDynOffsAvg_t *DynOffsAvg) {
    const VED_YwrOffsData_t *Offs = YWR_GET_OFFS; /* Offsetdaten */

    DynOffsAvg->YwFilt[0].fout = Offs->YawRateOffset;
    DynOffsAvg->YwFilt[1].fout = Offs->YawRateOffset;
    DynOffsAvg->YwFilt[0].fin = Offs->YawRateOffset;
    DynOffsAvg->YwFilt[1].fin = Offs->YawRateOffset;
    DynOffsAvg->YawRateOffset = Offs->YawRateOffset;

    DynOffsAvg->YwDist = 0.F;
    DynOffsAvg->MaxQuality = 0.F;

    DynOffsAvg->StDrv = 0.F;
    DynOffsAvg->YwDrvDistThr = 0.F;

    DynOffsAvg->DrvDistRed = 0.F;

    DynOffsAvg->YwFiltDet[0].fout = Offs->YawRateOffset;
    DynOffsAvg->YwFiltDet[1].fout = Offs->YawRateOffset;
    DynOffsAvg->YwFiltDet[0].fin = Offs->YawRateOffset;
    DynOffsAvg->YwFiltDet[1].fin = Offs->YawRateOffset;
    DynOffsAvg->YawRateDet = Offs->YawRateOffset;

    DynOffsAvg->QReduce = 1.F;
    DynOffsAvg->FlStwAStat = FALSE;

    return;
}

/* **********************************************************************
  @fn                     VED_YwrInitDynOffsInterim */ /*!
  @brief                  Initialisiert

  @description            see brief description

  @param[in]              DynOffsInter
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_YwrInitDynOffsInterim(VED_YwrDynOffsInter_t *DynOffsInter) {
    const VED_YwrOffsData_t *Offs = YWR_GET_OFFS; /* Offsetdaten */

    DynOffsInter->YawRateOffset = Offs->YawRateOffset;

    DynOffsInter->YwFiltInt[0].fout = Offs->YawRateOffset;
    DynOffsInter->YwFiltInt[1].fout = Offs->YawRateOffset;
    DynOffsInter->YwFiltInt[0].fin = Offs->YawRateOffset;
    DynOffsInter->YwFiltInt[1].fin = Offs->YawRateOffset;

    DynOffsInter->MaxQuality = 0.F;
    DynOffsInter->DistThrd = 0.F;

    return;
}

#endif

/* **********************************************************************
  @fn                     VED_YwrInitOffsetData */ /*!
  @brief                  Initialisiert modullokale Drehratendaten

  @description            see brief description

  @param[in]              OffsData
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrInitOffsetData(VED_YwrOffsData_t *OffsData) {
    VED_YwrInitStandStillOffset(&OffsData->StandStillOffset);
#if (CFG_VED__EX_YWR_NVM)
    VED_YwrInitStandStillEepromOffset(&OffsData->StandStillEepromOffset);
#endif

#if (CFG_VED__YW_DYN_AVG)
    VED_YwrInitDynOffsAvg(&OffsData->DynOffsAvg);
    VED_YwrInitDynOffsInterim(&OffsData->DynOffsAvg.IntOffs);
#endif
    OffsData->EcuElpsdTime = 0.F;
    OffsData->MaxQuality = 0.F;
    OffsData->OffsCompOK = FALSE;
    OffsData->OffsElpsdTime = 0.F;
    OffsData->OffsType = (VED_YwrOffsType_t)OFFS_NON;
    OffsData->Quality = 0.F;
#if (CFG_VED__YW_DYN_AVG)
    OffsData->QualNoRed = OffsData->Quality;
    OffsData->Cache.YawRateOffset = 0.F;
    OffsData->Cache.Quality = 0.F;
#endif
    OffsData->Temperature = 0.F;
    OffsData->TemperOK = FALSE;
    OffsData->YawRateOffset = 0.F;

    return;
}

/* **********************************************************************
  @fn                     VED_YwrGetPrivateData */ /*!
  @brief                  Zugriffunktion auf Modulinterne DrehratenDaten

  @description            see brief description

  @param[in]              -
  @param[out]             -
  @return                 *VED_YwrSenData_t

  @pre                    -
  @post                   -
**************************************************************************** */

VED_YwrData_t *VED_YwrGetPrivateData(void) { return (YWR_GET_ME); }

#if (CFG_VED__EX_YWR_NVM)
/* **********************************************************************
  @fn                     VED_YwrEepromWriteYwOffset */ /*!
  @brief                  Schreibt Yawrate-Offset in EEprom

  @description            see brief description

  @param[in]              NVValue
  @param[in]              nvin
  @param[in]              nvout
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrEepromWriteYwOffset(const VED_YwrEepromOffset_t *NVValue,
                                       const VEDNvIoDatas_t *nvin,
                                       VEDNvIoDatas_t *nvout) {
#if ((defined(CFG_VED__NVM_LEARN_DATA_ERROR)) && \
     (CFG_VED__NVM_LEARN_DATA_ERROR))
    /* check for valid values written into NVM */
    if (((VED_YwrOffsetRangeOk(NVValue->Offset1) == TRUE)) &&
        (NVValue->Status1 < YWR_NVM_OFFSET_STATUS_MAX)) {
#endif
        nvout->YwRate.ZeroRate = NVValue->Offset1;
        nvout->YwRate.CalStatus = NVValue->Status1;

#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
        /* writing Yaw offset into NVM in every excution cycle */
        LastYawNvmData.ZeroRate = NVValue->Offset1;
        LastYawNvmData.CalStatus = NVValue->Status1;
#endif

        /* check for new nvm min max yaw rate values */
        if (nvout->YwRate.CalStatus > 1U) {
            if (nvout->YwRate.ZeroRate < nvin->YwRate.ZeroRateMin) {
                nvout->YwRate.ZeroRateMin = nvout->YwRate.ZeroRate;
#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
                /* writing Yaw offset into NVM in every excution cycle */
                LastYawNvmData.ZeroRateMin = nvout->YwRate.ZeroRate;
#endif
            }
            if (nvout->YwRate.ZeroRate > nvin->YwRate.ZeroRateMax) {
                nvout->YwRate.ZeroRateMax = nvout->YwRate.ZeroRate;
#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
                /* writing Yaw offset into NVM in every excution cycle */
                LastYawNvmData.ZeroRateMax = nvout->YwRate.ZeroRate;
#endif
            }
        } else {
            /* first time that offset is written, set this offset as both min
             * and max value */
            nvout->YwRate.ZeroRateMin = nvout->YwRate.ZeroRate;
            nvout->YwRate.ZeroRateMax = nvout->YwRate.ZeroRate;
#if ((defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) && \
     (CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
            /* writing Yaw offset into NVM in every excution cycle */
            LastYawNvmData.ZeroRateMin = nvout->YwRate.ZeroRate;
            LastYawNvmData.ZeroRateMax = nvout->YwRate.ZeroRate;
#endif
        }

        VED_SET_NVM_IO_STATE(VED_NVM_POS_YWR, VED_IO_STATE_VALID,
                             &nvout->State);
#if ((defined(CFG_VED__NVM_LEARN_DATA_ERROR)) && \
     (CFG_VED__NVM_LEARN_DATA_ERROR))
        /* check for valid values written into NVM */
        YWR_SET_NVM_WRITE = (VED_ERR_STATE_INACTIVE);
    } else {
        YWR_SET_NVM_WRITE = (VED_ERR_STATE_ACTIVE);
        /*keep the old value into the NVM */
    }
#endif

    return;
}
#endif

#if (CFG_VED__EX_YWR_NVM)
/* **********************************************************************
  @fn                     VED_YwrEepromReadYwOffset */ /*!
  @brief                  Yawrate-Offset aus EEprom lesen

  @description            see brief description

  @param[in]              NVValue
  @param[in]              nvin
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrEepromReadYwOffset(VED_YwrEepromOffset_t *NVValue,
                                      const VEDNvIoDatas_t *nvin) {
#if (!(defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) || \
     (!CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
    if (VED__GET_NVM_IO_STATE(VED_NVM_POS_YWR, &nvin->State) ==
        VED_IO_STATE_VALID)
#endif
    {
        if (nvin->State == NVM_CLEARED) {
            NVValue->Offset1 = 0.F;
            NVValue->Status1 = (uint8)0UL;
        } else {
            /* Upload data from nvmemory */
            NVValue->Offset1 = nvin->YwRate.ZeroRate;
            NVValue->Status1 =
                (uint8)(nvin->YwRate.CalStatus & (uint32)0x000000FF);
        }
    }

    return;
}
#endif

/* **********************************************************************
  @fn                     VED_YwrIsValid */ /*!
  @brief                  Liefert Gueltigkeit der Drehrate

  @description            see brief description

  @param[in]              -
  @param[out]             -
  @return                 boolean *YwValid

  @pre                    -
  @post                   -

**************************************************************************** */
boolean VED_YwrIsValid(void) { return (YWR_GET_DATA->Valid); }

/* **********************************************************************
  @fn                     VED_YwrGetOffsData */ /*!
  @brief                  Zugriffunktion auf Offsetdaten

  @description            see brief description

  @param[in]              -
  @param[out]             -
  @return                 *VED_YwrOffsData_t

  @pre                    -
  @post                   -
**************************************************************************** */

const VED_YwrOffsData_t *VED_YwrGetOffsData(void) { return (YWR_GET_OFFS); }

#if (CFG_VED__YW_DYN_AVG)
/* **********************************************************************
  @fn                     VED_YwrGetDynReduced         */ /*!
  @brief                  Rueckgabe der momentanen moeglichen Reduzierung

  @description            see brief description

  @param[in]              -
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
float32 VED_YwrGetDynReduced(void) {
    VED_YwrOffsData_t *pYwOffsData = YWR_GET_OFFS;
    float32 ret;

    ret = VED_YwrCalcQRedRamp(pYwOffsData);

    return (ret);
}
#endif

/* **********************************************************************
  @fn                     VED_YwrCommonInit */ /*!
  @brief                  Initialisiert Drehratenspur ( nicht die Offset-Daten)

  @description            see brief description

  @param[in]              -
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_YwrCommonInit(void) {
    VED_YwrSenData_t *YawRateData;

    /* Hole Modulinterne Drehratendaten */
    YawRateData = YWR_GET_DATA;

    /* Hole Drehratenspur */
    VED_YwrInitYawRateData(YawRateData);

    return;
}

#if (CFG_VED__EX_YWR_NVM)
/* **********************************************************************
  @fn                     VED_YwrCalcEepromOffsetQuality */ /*!
  @brief                  Berechnet die Guete des EEPROM-Drehratenoffsets

  @description            see brief description

  @return                 -
  @param[in]        
  @param[out]       
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrCalcEepromOffsetQuality(
    VED_YwrStandStillEepromOffs_t *pNvmOffs) {
    const float32 invNvmStatMax_c = 1.F / ((float32)YWR_NVM_STATUS_MAX);

    pNvmOffs->MaxQuality =
        ((float32)pNvmOffs->YawRateOffset.Status1 * invNvmStatMax_c);

    pNvmOffs->MaxQuality = MIN(1.F, pNvmOffs->MaxQuality);

    return;
}
#endif

/* **********************************************************************
  @fn                     VED_YwrOffsetInit */ /*!
  @brief                  Initialisiert die Offsetdaten der Drehratenspur

  @description            see brief description

  @param[in]              nv_read
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrOffsetInit(const VEDNvIoDatas_t *nv_read) {
    VED_YwrOffsData_t *OffsData;

    /* Hole Drehratenoffset */
    OffsData = YWR_GET_OFFS;

    VED_YwrInitOffsetData(OffsData);

#if (CFG_VED__EX_YWR_NVM)
    /* nur im ARS210: Drehratenoffset aus Eeprom holen */
    VED_YwrEepromReadYwOffset(&OffsData->StandStillEepromOffset.YawRateOffset,
                              nv_read);

    /* Guete des Offset-Werts im EEPROM berechnen */
    VED_YwrCalcEepromOffsetQuality(&OffsData->StandStillEepromOffset);
#endif

    return;
}

#if (CFG_VED__EX_YWR_NVM)
/* **********************************************************************
  @fn               VED_YwrSaveStandstillOffset */ /*!
  @brief            Save standstill offset to nonvolatile memory 

  @description      see brief description

  @param[in,out]    OffsetData offset data 
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrSaveStandstillOffset(VED_YwrOffsData_t *OffsetData) {
    const VED_YwrStandStillOffs_t *pStStOffs = &OffsetData->StandStillOffset;
    VED_YwrStandStillEepromOffs_t *pStStNvmOffs =
        &OffsetData->StandStillEepromOffset;

    float32 CycleTime;
    float32 YwTimeFirstWriteSec;

    /* Get cycle time */
    CycleTime = VED_GetCycleTime();

    /* Required elapsed time since ignition start in dependence of prior write
     * cycles */
    if (OffsetData->StandStillEepromOffset.YawRateOffset.Status1 > (uint8)0) {
        YwTimeFirstWriteSec = YWR_NVM_OFFS_STABLE_TIME;
    } else {
        YwTimeFirstWriteSec = YWR_NVM_OFFS_STABLE_TIME_1ST;
    }

    /*  Prerequisite for storage of standstill offset in nonvolatile memory
     *   -> Warmed-up gyro sensor (elapsed time since ignition start)
     *   -> Minimum quality of standstill offset
     *   -> Minimum standstill adjust time
     *   -> Maximum number of nvm write accesses not exceeded
     *   -> Time gap between successive write accesses
     or
     *   -> NVM offset state is 0 (no offset)
     *   -> The quality of the stand still offset should be good
     *   -> Store the offset 1 second after stand still  */
    if (((OffsetData->EcuElpsdTime > YwTimeFirstWriteSec) &&
         (pStStOffs->Observable != FALSE) &&
         (pStStOffs->MaxQuality >
          (VED__YWR_OFF_STST_CALC_TIME_Q_MAX - C_F32_DELTA)) &&
         (pStStOffs->SampleInterval.Volume >=
          (YWR_NVM_WRITE_MIN_ADJ_TIME / CycleTime)) &&
         (pStStNvmOffs->NumOfWrittenOffsets <
          (uint8)YWR_NVM_WRITE_IGN_MAX_NO) &&
         (pStStNvmOffs->TimeLastWrittenEepromOffset >
          YWR_NVM_WRITE_NEW_GAP_TIME)) ||
        ((OffsetData->StandStillEepromOffset.YawRateOffset.Status1 ==
          (uint8)(0U)) &&
         (OffsetData->QualNoRed >
          (VED__YWR_OFF_STST_CALC_TIME_Q_MAX - C_F32_DELTA)) &&
         (OffsetData->OffsElpsdTime > YWR_NVM_WRITE_OFFS_ELAPS_TIME) &&
         (OffsetData->OffsType == (VED_YwrOffsType_t)OFFS_STANDST))) {
        float32 fltGain;

        /* Determine filter gain */
        if (pStStNvmOffs->YawRateOffset.Status1 > (uint8)YWR_NVM_FLT_MAX_WGHT) {
            fltGain = 1.0F / ((float32)YWR_NVM_FLT_MAX_WGHT + 1.0F);
        } else {
            fltGain =
                1.0F / ((float32)pStStNvmOffs->YawRateOffset.Status1 + 1.0F);
        }

        /* Filter standstill offset */
        pStStNvmOffs->YawRateOffset.Offset1 =
            ((1.F - fltGain) * pStStNvmOffs->YawRateOffset.Offset1) +
            (fltGain * pStStOffs->SampleInterval.Mean);

        /* Increment nvm yaw rate status */
        if (pStStNvmOffs->YawRateOffset.Status1 <
            (uint8)YWR_NVM_OFFSET_STATUS_MAX) {
            pStStNvmOffs->YawRateOffset.Status1 += (uint8)(1U);
        }

        /* Store new values in non-volatile memory */
        VED_YwrEepromWriteYwOffset(&pStStNvmOffs->YawRateOffset,
                                   YWR_GET_NVM_READ, YWR_GET_NVM_WRITE);

        /* Update nvm standstill offset quality */
        VED_YwrCalcEepromOffsetQuality(pStStNvmOffs);

        /* Reset time since last write access */
        OffsetData->StandStillEepromOffset.TimeLastWrittenEepromOffset = 0.F;

        /* Increment number of write accesses during this ignition cycle */
        pStStNvmOffs->NumOfWrittenOffsets += (uint8)(1U);
    } else {
        /* Prerequisite to store standstill offset are not given  */
    }
    return;
}
#endif

/* **********************************************************************
  @fn               VED_YwrEstStandstillOffset */ /*!
  @brief            Estimate standstill offset

  @description      see brief description

  @param[in]        YwRate standstill offset data
  @param[in]        valYwr
  @param[in]        pStStOff
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_YwrEstStandstillOffset(float32 YwRate,
                                boolean valYwr,
                                VED_YwrStandStillOffs_t *pStStOff) {
    float32 CycleTime;

    /* Get cycle time */
    CycleTime = VED_GetCycleTime();

    /* Initialize maximum quality */
    pStStOff->MaxQuality = 0.F;

    /* Test if yaw rate offset is observable */
    if (pStStOff->Observable != FALSE) {
        float32 invCycleTime;
        /* Get inverse cycle time */
        invCycleTime = 1.0F / CycleTime;

        /* Yaw rate offset is observable */
        VED_StatIntervalAdd(&pStStOff->SampleInterval_1, YwRate, 1.0F);

        /* Test if offset samples collection time has been elapsed */
        if (pStStOff->SampleInterval_1.Volume >=
            (VED__PAR_YWR_STST_CAL_TIME_MIN * invCycleTime)) {
            /* Enough yaw rate offset samples are available */

            /* Add delay interval to evaluation interval */
            VED_StatIntervalMerge(&pStStOff->SampleInterval,
                                  &pStStOff->SampleInterval_2);
            VED_StatIntervalMeanDev(&pStStOff->SampleInterval);

            /* Reset elapsed time since last offset adjust */
            pStStOff->AdjustTime = 0.F;

            /* If deviation of sampled data is too large, discard all data */
            if (pStStOff->SampleInterval.Dev >= STANDST_STDABW_MAX) {
                VED_StatIntervalInit(&pStStOff->SampleInterval_1);
                VED_StatIntervalInit(&pStStOff->SampleInterval_2);
                VED_StatIntervalInit(&pStStOff->SampleInterval);
            }

            /* Check if enough time has been elapsed for offset calibration */
            if (pStStOff->SampleInterval.Volume >=
                (VED__PAR_YWR_STST_CAL_TIME_MIN * invCycleTime)) {
#if ((defined(CFG_VED__TURNTABLE_DETECTION)) && (CFG_VED__TURNTABLE_DETECTION))
                /* Take over new zero point offset if it the delta offset
                 * difference is under the MAX limit else take delta offset as
                 * previous offset +/-  4deg/sec*/
                if (fABS(PrevYawRateOffset - pStStOff->SampleInterval.Mean) <
                    MAX_LIMITATION_DELTA_YAW_RATE_OFFSET) {
                    pStStOff->YawRateOffset = pStStOff->SampleInterval.Mean;
                } else {
                    if (pStStOff->SampleInterval.Mean > PrevYawRateOffset) {
                        pStStOff->YawRateOffset =
                            PrevYawRateOffset +
                            MAX_LIMITATION_DELTA_YAW_RATE_OFFSET;
                    } else {
                        pStStOff->YawRateOffset =
                            PrevYawRateOffset -
                            MAX_LIMITATION_DELTA_YAW_RATE_OFFSET;
                    }
                }
#else
                /* Take over new zero point offset */
                pStStOff->YawRateOffset = pStStOff->SampleInterval.Mean;

#endif

                /* Calculate quality by means of used samples */
                pStStOff->MaxQuality =
                    VED_LFunction((pStStOff->SampleInterval.Volume * CycleTime),
                                  VED__PAR_YWR_STST_CAL_TIME_MIN,
                                  VED__PAR_YWR_STST_CAL_TIME_NORM,
                                  VED__YWR_OFF_STST_CALC_TIME_Q_MIN,
                                  VED__YWR_OFF_STST_CALC_TIME_Q_MAX);
            }

            /* If offset calibration last a long time fade memory data to
             * preserve dynamic */
            if (pStStOff->SampleInterval.Volume >=
                (VED__YWR_OFF_STST_SUM_MAX_TIME / CycleTime)) {
                VED_StatIntervalReduce(&pStStOff->SampleInterval,
                                       VED__YWR_OFFSET_REDUCTION_FACTOR);
            }

            /* Copy gathering inverval to delay interval */
            VED_StatIntervalInit(&pStStOff->SampleInterval_2);
            VED_StatIntervalMerge(&pStStOff->SampleInterval_2,
                                  &pStStOff->SampleInterval_1);

            /* Initialize gathering interval */
            VED_StatIntervalInit(&pStStOff->SampleInterval_1);
        }
    } else {
        /* Yaw rate offset is not observable */

        if ((pStStOff->StandstillOK == FALSE) || (valYwr == FALSE)) {
            /* Vehicle doesn't stand still */

            /* Initialize gathering and delay interval because of possibly wrong
             * sampled data during drive-off */
            VED_StatIntervalInit(&pStStOff->SampleInterval_1);
            VED_StatIntervalInit(&pStStOff->SampleInterval_2);
        }

        /* Preserve evaluation interval for certain time to prevent
           initialization during stop and go driving situations. This reduces
           the time to complete the learning of offset value */
        if (pStStOff->SampleInterval.Volume > 0.F) {
            pStStOff->AdjustTime += CycleTime;

            if (pStStOff->AdjustTime >= VED__YWR_OFF_STST_INTERV_HOLD_TIME) {
                VED_StatIntervalInit(&pStStOff->SampleInterval);
                pStStOff->AdjustTime = 0.F;
            }
        }
    }
    return;
}

/* **********************************************************************
  @fn                     VED_YwrCalcOffsetQualityTime */ /*!
  @brief                  Berechnet die aktuelle Guete des Drehratenoffsets ueber die abgelaufene Zeit

  @description            see brief description

  @param[in]              MaxQuality    (Guete des Offsets zum Zeitpunkt der Berechnung)
  @param[in]              OffsElpsdTime (Abgelaufene Zeit seit der Berechnung d. Offset)
  @param[in]              TemperOK      (Getemperter/Ungetemperter Abgleich)
  @param[out]             OffsQuality (Guete des Drehratenoffsets)
  @return                 -

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_YwrCalcOffsetQualityTime(float32 *OffsQuality,
                                         float32 MaxQuality,
                                         float32 OffsElpsdTime,
                                         boolean TemperOK) {
    VED_YwrOffsData_t *OffsData;

    /* Hole Drehratenoffset */
    OffsData = YWR_GET_OFFS;

#if (CFG_VED__YW_DYN_AVG)
    /* Bei grosser Abweichung zu dynamischen Offset, Guete reduzieren um Gewicht
       bei Gesamtspur zu minimieren */
    {
        const VED_OutCurve_t WhsCurve_c = {0.F, 0.F, 0.F};
        const VED_OutCurve_t SwaLCurve_c = {0.F, 0.F, 0.F};

        const VED_OutCurve_t *WspTrack = &WhsCurve_c;      /* RDZ-Spur     */
        const VED_OutCurve_t *StrgLogTrack = &SwaLCurve_c; /* Log.LW-Spur  */
        const float32 qualMinStLog_c = 0.1F; /* Mindest-Guete STA */

        /* Vorbedingung die fuer eine GueteReduzierung erfuellt sein muessen */
        if (OffsData->DynOffsAvg.YwDist > YWR_DYNC_QRED_DIST_MIN) {
            float32 wghtPermit; /* Faktor fuer zugelassene Guetereduzierung */

            /* Erlaubte Guetereduzierung in Abhaengigkeit von Lenkwinkel- und
             * Raddrehzahlgueten */
            if (StrgLogTrack->Variance > qualMinStLog_c) {
                wghtPermit =
                    1.0F; /* Maximale Reduzierung zulassen d. h. 1 -> 0 */
            } else {
                /* zugelassene Reduzierung in Abhaengigkeit der
                  Raddrehzahlspur-Qualitaet
                  d. h. dQred =  (1 - Qred) * Permit, und QRedNeu = 1-dQred */
                wghtPermit = VED_LFunction(
                    WspTrack->Variance, YWR_WSP_Q_START_MIN, YWR_WSP_Q_END_MAX,
                    YWR_WSP_Q_CONF_START_MIN, YWR_WSP_Q_CONF_START_MAX);
            }

            /* Zweistufige Rampe zur Guetereduzierung */
            OffsData->DynOffsAvg.QReduce = VED_YwrCalcQRedRamp(OffsData);

            /* Bei dynamischen Average-Offset zugelassende Reduzierung in
               Abhaengigkeit von der wirksamen Offsetzeit erhoehen, entspricht
               ansatzweise einer Gradientenbetrachtung */
            if (OffsData->OffsType == (VED_YwrOffsType_t)OFFS_DYN_AVG) {
                OffsData->DynOffsAvg.QReduce *= VED_LFunction(
                    OffsData->OffsElpsdTime, YWR_OFFSET_TIME_START_MIN,
                    YWR_OFFSET_TIME_START_MAX, YWR_Q_FACTOR_DELTA_START_MIN,
                    YWR_Q_FACTOR_DELTA_START_MAX);
            }

            /* Max. Reduzierung in Abhaengigkeit der restlichen Guetesignale
             * zulassen  */
            OffsData->DynOffsAvg.QReduce =
                1.0F - ((1.0F - OffsData->DynOffsAvg.QReduce) * wghtPermit);
        } else {
            OffsData->DynOffsAvg.QReduce = 1.F; /* Default Wert setzen */
        }
    }

#endif

    /* In Abhaengigkeit des wirksamen Offset-Typs Qualitaet altern */
    switch (OffsData->OffsType) {
#if (CFG_VED__YW_DYN_AVG)
        case OFFS_DYN_AVG:
        case OFFS_DYN_INTER:
            /* DynAverageOffset, unsicher -> deshalb Guete schnell reduzieren */
            *OffsQuality = MaxQuality *
                           VED_LFunction(OffsElpsdTime, YWR_TIME_DYNAVGOFF_MIN,
                                         YWR_TIME_DYNAVGOFF_MAX,
                                         YWR_Q_FACTOR_DYNAVGOFF_MAX,
                                         YWR_Q_FACTOR_DYNAVGOFF_MIN);
            break;
#endif

#if (CFG_VED__EX_YWR_NVM)
        case OFFS_STANDST_EEPROM:

            /* Qualitaet ueber Zeit wird in seperater Funktion speziell fuer
             * EEPROM berechnet */
            *OffsQuality = 1.F;

            break;
#endif

        case OFFS_NON:
        case OFFS_DYN_APPRX:
        case OFFS_STANDST:
        case OFFS_TEMPER_TABLE:
        default:
            if (TemperOK == TRUE) { /* Getemperter (eingeschwungener) Zustand ->
                                       Guete langsam reduzieren */
                *OffsQuality =
                    MaxQuality * VED_LFunction(OffsElpsdTime,
                                               YWR_TIME_SENSOR_TEMP_MIN,
                                               YWR_TIME_SENSOR_TEMP_MAX,
                                               YWR_Q_FACTOR_SENSOR_TEMP_MAX,
                                               YWR_Q_FACTOR_SENSOR_TEMP_MIN);
            } else { /* Nicht getemperter Zustand -> Guete schnell reduzieren */
                *OffsQuality =
                    MaxQuality * VED_LFunction(OffsElpsdTime,
                                               YWR_TIME_SENSOR_COOL_MIN,
                                               YWR_TIME_SENSOR_COOL_MAX,
                                               YWR_Q_FACTOR_SENSOR_COOL_MAX,
                                               YWR_Q_FACTOR_SENSOR_COOL_MIN);
            }
            break;
    }

#if (CFG_VED__YW_DYN_AVG)
    /* Nicht-reduzierte Guete speichern fuer Uebernahme-Test */
    OffsData->QualNoRed = *OffsQuality;

    /* zurueckgegebene Guete reduzieren fuer Gewichtung */
    *OffsQuality *= OffsData->DynOffsAvg.QReduce;
#endif
} /* END:    void VED_YwrCalcOffsetQualityTime */

#if (CFG_VED__EX_YWR_NVM)
/* **********************************************************************
  @fn                     VED_YwrCalcEepromOffsetQualityTime */ /*!
  @brief                  Berechnet die aktuelle Guete des EEPROM-Drehratenoffsets 
                          ueber die abgelaufene Zeit
  @description            see brief description

  @param[in]              MaxQuality
  @param[in]              EcuElpsdTime
  @param[in,out]          OffsQuality (Guete des Drehratenoffsets)
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrCalcEepromOffsetQualityTime(float32 *OffsQuality,
                                               float32 MaxQuality,
                                               float32 EcuElpsdTime) {
    VED_YwrOffsData_t *OffsData;

    /* Hole Drehratenoffset */
    OffsData = YWR_GET_OFFS;

    /* Erhoehen der Guete ueber Zeit */

    /* Getemperter (eingeschwungener) Zustand -> maximale Guete */
    *OffsQuality =
        MaxQuality * VED_LFunction(EcuElpsdTime, YWR_TIME_Q_EEPROM_OFFSET_MIN,
                                   YWR_TIME_Q_EEPROM_OFFSET_MAX,
                                   YWR_Q_FACTOR_EEPROM_OFFSET_MIN,
                                   YWR_Q_FACTOR_EEPROM_OFFSET_MAX);

#if (CFG_VED__YW_DYN_AVG)
    /* Nicht-reduzierte Guete speichern fuer Uebernahme-Test */
    OffsData->QualNoRed = *OffsQuality;

    /* zurueckgegebene Guete reduzieren fuer Gewichtung */
    *OffsQuality *= OffsData->DynOffsAvg.QReduce;
#endif

    return;
}
#endif

/* **********************************************************************
  @fn                     VED_YwrTakeOffs */ /*!
  @brief                  Berechnet die aktuelle Guete des Drehratenoffsets ueber die abgelaufene Zeit

  @description            see brief description

  @param[in]              OffsData (aktueller Offset)
  @param[in]              Offset   (neuer Offset)
  @param[in]              Quality  (Guete des Offsets )
  @param[in]              Type     (Offsetart)
  @return                 boolean           (Offset wurde uebernommen = TRUE)

  @pre                    -
  @post                   -

**************************************************************************** */
static boolean VED_YwrTakeOffs(VED_YwrOffsData_t *OffsData,
                               float32 Offset,
                               float32 Quality,
                               VED_YwrOffsType_t Type) {
    boolean OffsetTaken = FALSE;

    /* Pruefen ob ein neuer Offset uebernommen werden soll */
    if ((Quality >= C_F32_DELTA) /* Offsetguete muss > 0 sein */
        &&
        ((Type ==
          (VED_YwrOffsType_t)OFFS_TEMPER_TABLE) /* Bei Offset aus Tabelle alle
                                                   mit Guete > 0 uebernehmen */
#if (CFG_VED__YW_DYN_AVG)
         ||
         (Quality >= OffsData->QualNoRed) /* Bei anderen Offset nur uebernehmen
                                             wenn Guete neuer Offset groesser */
#else
         ||
         (Quality >= OffsData->Quality) /* Bei anderen Offset nur uebernehmen
                                           wenn Guete neuer Offset groesser */
#endif
         )) {
        /* Stillstandoffset uebernehmen */
        OffsData->YawRateOffset = Offset;

        /* Guete neu setzen */
        OffsData->MaxQuality = Quality;

        /* Offsetart Merken */
        OffsData->OffsType = Type;

        /* Zeit zur Gueteberechnung zuruecksetzen */
        OffsData->OffsElpsdTime = 0.F;

        if (Type != (VED_YwrOffsType_t)OFFS_TEMPER_TABLE) {
            /* Temper Flag setzen falls ECU schon laenger laeuft */
            if (OffsData->EcuElpsdTime > YWR_ELPSD_TIME_SENSOR_COOL) {
                OffsData->TemperOK = TRUE;
            }
        }

        /* Offset als uebernommen markieren */
        OffsetTaken = TRUE;
    }

    return (OffsetTaken);
}

/* **********************************************************************
  @fn                     VED_YwrCalcOffset */ /*!
  @brief                  Berechnung Drehratenoffset aus der Drehrate

  @description            see brief description

  @param[in]              YawRate  (Allg. Drehratendaten)
  @param[in,out]          OffsData (Offsetdaten Stillstand u. Dyn)
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrCalcOffset(const VED_YwrSenData_t *YawRate,
                              VED_YwrOffsData_t *OffsData) {
#if ((defined(CFG_VED__TURNTABLE_DETECTION)) && (CFG_VED__TURNTABLE_DETECTION))
    static uint8 u_turntableCounter;
    boolean b_turn_table_flag = TRUE;
    static boolean b_PrevStandstillOK = FALSE;
    static boolean b_initFlag = FALSE;
    float32 yawRateDiffFront, yawRateDiffRear;
#else
    /*will make to wait for 25 cycles once after coming out of turn table */
    boolean b_turn_table_flag = FALSE;
#endif

    VED_YwrStandStillOffs_t *pStStOffs = &OffsData->StandStillOffset;
    uint8 CaliState = YWR_GET_ME->Io.in->Frame.CaliMode;

#if (CFG_VED__EX_YWR_NVM)
    float32 VehicleSpeed = YWR_GET_MIF_DATA->LongMot.VehVelocityCorr;
#endif

#if ((defined(CFG_VED__TURNTABLE_DETECTION)) && (CFG_VED__TURNTABLE_DETECTION))

    yawRateDiffFront =
        fABS(ved__internal_data.ved__wye_out.front_whl_yaw_rate_filt_wld -
             YawRate->YawRate);
    yawRateDiffRear =
        fABS(ved__internal_data.ved__wye_out.rear_whl_yaw_rate_filt_wld -
             YawRate->YawRate);

    /* will detect the vehicle turn table and set the flag  */
    if ((yawRateDiffFront > VED__YAW_RATE_DIFF_THRESHOLD) &&
        (yawRateDiffRear > VED__YAW_RATE_DIFF_THRESHOLD)) {
        b_turn_table_flag = TRUE;

        if (u_turntableCounter < VED__TURN_TABLE_ENABLE_TIME) {
            u_turntableCounter++;
        }
    } else {
        if (u_turntableCounter > 0) {
            u_turntableCounter--;
        }
    }

    if (u_turntableCounter == 0) {
        b_turn_table_flag = FALSE;
    }

    /* During the Ignition On and the ego vehicle is in standstill mode
     * initailizing the b_PrevStandstillOK to FALSE only in the first cycle for
     * storing the PrevYawRateOffset value */
    if (b_initFlag == FALSE) {
        b_PrevStandstillOK = FALSE;
        b_initFlag = TRUE;
    } else {
        b_PrevStandstillOK = pStStOffs->StandstillOK;
    }

#endif
    /* Determine standstill for offset compensation */
    pStStOffs->StandstillOK =
        (boolean)((YWR_GET_MIF_DATA->LongMot.MotState.MotState ==
                   (uint8)VED_LONG_MOT_STATE_STDST) &&
                  (YWR_GET_MIF_DATA->LongMot.MotState.Confidence >
                   VED__PAR_YWR_STST_CONF_MIN));

    /* ##Code Review##: Since the condition check of b_turn_table_flag should
     * allow for non Turn table situations , so it is acceptable " */

    /* Determine if yaw rate offset is observable */
    if ((fABS(YawRate->Gradient) <= VED__YWR_OFF_STST_GRAD_ABS_MAX) &&
        (YawRate->Valid != FALSE) && (b_turn_table_flag == FALSE) &&
        (pStStOffs->StandstillOK != FALSE)) {
        pStStOffs->Observable = TRUE;
    } else {
        pStStOffs->Observable = FALSE;
    }

#if ((defined(CFG_VED__TURNTABLE_DETECTION)) && (CFG_VED__TURNTABLE_DETECTION))
    /*moving to standstill*/
    if ((b_PrevStandstillOK == FALSE) && (pStStOffs->StandstillOK == TRUE)) {
        PrevYawRateOffset = pStStOffs->YawRateOffset;
    }
#endif

    /* Start standstill offset calibration */
    if (!VED__CTRL_GET_STATE((uint8)VED_CAL_YWR_OFFS_STST, CaliState)) {
        VED_YwrEstStandstillOffset(YawRate->YawRate, YawRate->Valid, pStStOffs);
    }

#if (CFG_VED__EX_YWR_NVM)
    /* Save standstill offset in nonvolatile memory */
    VED_YwrSaveStandstillOffset(OffsData);
#endif

#if (CFG_VED__YW_DYN_AVG)
    {
        /* Dyn. Abgleich ueber gemittelte Geradeausfahrt */
        if (!VED__CTRL_GET_STATE((uint8)VED_CAL_YWR_OFFS_DYN, CaliState)) {
            float32 distFilt;
            distFilt =
                VED_YwrCalcDynOffsAvg(YawRate, &OffsData->DynOffsAvg, OffsData);

            /* Dynamischer Offset bei ungelerntem Lenkwinkel-Offset */
            VED_YwrCalcDynOffsInterim(&OffsData->DynOffsAvg.IntOffs, OffsData,
                                      distFilt);
        }

        /* Dynamischen Interims Offset auf Uebernahme testen */
        (void)VED_YwrTakeOffs(OffsData,
                              OffsData->DynOffsAvg.IntOffs.YawRateOffset,
                              OffsData->DynOffsAvg.IntOffs.MaxQuality,
                              (VED_YwrOffsType_t)OFFS_DYN_INTER);

        /* Dynamischen Mittlerwert-Offset Uebernehmen falls neu */
        (void)VED_YwrTakeOffs(OffsData, OffsData->DynOffsAvg.YawRateOffset,
                              OffsData->DynOffsAvg.MaxQuality,
                              (VED_YwrOffsType_t)OFFS_DYN_AVG);
    }
#endif

    /* Stillstand Offset uebernehmen falls neu */
    (void)VED_YwrTakeOffs(OffsData, OffsData->StandStillOffset.YawRateOffset,
                          OffsData->StandStillOffset.MaxQuality,
                          (VED_YwrOffsType_t)OFFS_STANDST);
    /* Guete ueber Zeit */
    VED_YwrCalcOffsetQualityTime(&OffsData->Quality, OffsData->MaxQuality,
                                 OffsData->OffsElpsdTime, OffsData->TemperOK);

#if (CFG_VED__EX_YWR_NVM)
    /* bei nicht vorhandenem Stillstandsoffset, Offset aus EEPROM verwenden */
    if ((VehicleSpeed > YWR_CURVE_V_MIN) &&
        (OffsData->OffsType == (VED_YwrOffsType_t)OFFS_NON)) {
        /* Falls noch kein Offset vorhanden, EEPROM-Offset mit vorhandener
         * MaxQuality verwenden */
        (void)VED_YwrTakeOffs(
            OffsData, OffsData->StandStillEepromOffset.YawRateOffset.Offset1,
            OffsData->StandStillEepromOffset.MaxQuality,
            (VED_YwrOffsType_t)OFFS_STANDST_EEPROM);
    } else {
        /* aktueller Stillstandoffset vorhanden */
    }

    if (OffsData->OffsType == (VED_YwrOffsType_t)OFFS_STANDST_EEPROM) {
        /* Bestimmen der Drehratenguete basierend auf EEPROM-Offset ueber die
         * Zeit */
        VED_YwrCalcEepromOffsetQualityTime(
            &OffsData->Quality, OffsData->StandStillEepromOffset.MaxQuality,
            OffsData->EcuElpsdTime);
    } else {
        /* kein EEPROM-Offset, es muss keine spezielle Guete bestimmt werden */
    }
#endif

    return;
}

/* **********************************************************************
  @fn               VED_YwrGetYawRate */ /*!

  @brief            initialize component data

  @description      see brief description

  @param            yawrate
  @return           void

  @pre              Precondition:  none
  @post             Postcondition: none
**************************************************************************** */
void VED_YwrGetYawRate(VEDYawRateVehDyn_t *yawrate) {
    VED_YwrOffsData_t *pOffs = YWR_GET_OFFS;
    VED_YwrSenData_t *pSen = YWR_GET_DATA;

    /* Berechne Drehrate offsetkorregiert */
    yawrate->YawRate = pSen->YawRate - pOffs->YawRateOffset;
    yawrate->Variance = 0.f;

    return;
}

/* **********************************************************************
  @fn                     VED_YwrCalcRunnigTime */ /*!
  @brief                  Calculate run times required for offset calibration

  @description            see brief description

  @param[in]              -
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrCalcRunnigTime(void) {
    float32 CycleTime;
    VED_YwrOffsData_t *pOffsData;

    /* Get cycle time */
    CycleTime = VED_GetCycleTime();

    /* Get reference to offset data */
    pOffsData = YWR_GET_OFFS;

    /* Update ECU runtime up to maximum permitted value */
    if (pOffsData->EcuElpsdTime >= YWR_ECU_ELPSD_TIME_MAX) {
        pOffsData->EcuElpsdTime = YWR_ECU_ELPSD_TIME_MAX;
    } else {
        pOffsData->EcuElpsdTime += CycleTime;
    }

    /* Update time since last offset calibration up to maximum permitted value
     */
    if (pOffsData->OffsElpsdTime >= YWR_OFFS_ELPSD_TIME_MAX) {
        pOffsData->OffsElpsdTime = YWR_OFFS_ELPSD_TIME_MAX;
    } else {
        pOffsData->OffsElpsdTime += CycleTime;
    }

#if (CFG_VED__EX_YWR_NVM)
    /* Update time since last storage of standstill offset to nonvolatile memory
     */
    if (pOffsData->StandStillEepromOffset.TimeLastWrittenEepromOffset >=
        YWR_OFFS_ELPSD_TIME_MAX) {
        pOffsData->StandStillEepromOffset.TimeLastWrittenEepromOffset =
            YWR_OFFS_ELPSD_TIME_MAX;
    } else {
        /* Start timing not until first value has been written */
        if (pOffsData->StandStillEepromOffset.NumOfWrittenOffsets > (uint8)0) {
            pOffsData->StandStillEepromOffset.TimeLastWrittenEepromOffset +=
                CycleTime;
        }
    }
#endif
    return;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */