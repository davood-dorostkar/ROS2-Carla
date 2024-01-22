/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
/*! \file **********************************************************************

  COMPONENT:              VED (Vehicle Dynamics Observer)

  MODULENAME:             ved__ywrt.c

  @brief                  This module contains signal processing of yaw rate
                          sensor providing its temperature


*****************************************************************************/

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

#if (CFG_VED__INT_GYRO)

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/
/* Yaw rate differentiator time constant */
#define VED__YWRT_DIFF_FILT ((float32)1.5)

/* Temperature differentiator time constant */
#define VED__YWRT_TEMP_DIFF_FILT ((float32)20.0)

/* Maximum yaw rate standard deviation during standstill observation */
#define VED__YWRT_YWR_STD_DEV_MAX DEG2RAD(0.15F)

/* Temperature table yaw rate scale factors */
#define VED__YWRT_YWR_SCALE \
    (1.F /                  \
     VED__YWRT_YWR_INV_SCALE) /* floating point  = scale * fixed point    */
#define VED__YWRT_YWR_INV_SCALE \
    (1E4F) /* fixed point = inv_scale * floating point */

/* Temperature table confidence scale factors */
#define VED__YWRT_CONF_SCALE \
    (1.F /                   \
     VED__YWRT_CONF_INV_SCALE) /* floating point  = scale * fixed point    */
#define VED__YWRT_CONF_INV_SCALE \
    (255.F) /* fixed point = inv_scale * floating point */

/* Minimum temperature of table node in degree celcius */
#define VED__YWRT_TBL_MIN_TEMP (-60L)

/* Maximum temperature of table node in degree celcius */
#define VED__YWRT_TBL_MAX_TEMP (+125L)

/* Temperature gap between table nodes */
#define VED__YWRT_TBL_GAP (+5L)

/* First node of temperature table in degree celcius */
#define VED__YWRT_FIRST_NODE_TEMP ((f32_t)VED__YWRT_TBL_MIN_TEMP)

/* Last node of temperature table in degree celcius */
#define VED__YWRT_LAST_NODE_TEMP ((f32_t)VED__YWRT_TBL_MAX_TEMP)

/* Node width of temperature table in degree celcius */
#define VED__YWRT_NODE_WIDTH ((f32_t)VED__YWRT_TBL_GAP)

/* Inverse node width of temperature table in 1/deg C */
#define VED__YWRT_NODE_INV_WIDTH (1.F / VED__YWRT_NODE_WIDTH)

/* Maximum zero point offset of yaw rate sensor */
#define VED__YWRT_YWR_MAX_OFFSET (float32) DEG2RAD(5.F)

/* Maximum yaw acceleration during standstill for calibration */
#define VED__YWRT_YWR_GRAD_MAX (float32) DEG2RAD(0.2F)

/* Confidence value to activate relearn */
#define VED__YWRT_CONF_LEARN (0.90F)

/* Confidence value to activate relearn */
#define VED__YWRT_CONF_LEARN_ADJ (0.58F)

/* Minimum weightning for new offset value */
#define VED__YWRT_ALPHA_MIN (0.2F)

/* Maximum weighning for new offset value */
#define VED__YWRT_ALPHA_MAX (0.8F)

/* Maximum weighning for new offset value */
#define VED__YWRT_ALPHA_VAL_MAX (1.0F)

/* Delta for comparing confidences */
#define VED__YWRT_ALPHA_DLT (1E-5F)

/* Reduction factor for learning speed */
#define VED__YWRT_RED_LEARN_SPEED (6.F)

/* Confidence where update gain is reduced to normal gain */
#define VED__YWRT_LEARN_SPEED_CONF_MIN (0.01F)

/* Confidence where normal update gain is used */
#define VED__YWRT_LEARN_SPEED_CONF_MAX (0.20F)

/* Maximum confidence value */
#define VED__YWRT_CONF_MAX (1.F)

/* Maximum confidence value if current temperature is around node */
#define VED__YWRT_CONF_MAX_NEAR (1.F)

/* Maximum confidence value if current temperature is far from node */
#define VED__YWRT_CONF_MAX_FAR (0.6F)

/* Threshold deviation to activate relearning */
#define VED__YWRT_LEARN_THRHD ((float32)DEG2RAD(0.15F))

/* Threshold deviation at far node to reset confidence of far node when above
 * limit */
#define VED__YWRT_LEARN_THRHD_FAR ((float32)DEG2RAD(0.5F))

/* Threshold deviation at near to reduce confidence of far node when below limit
 */
#define VED__YWRT_LEARN_THRHD_NEAR ((float32)DEG2RAD(0.5F))

/* Maximum reasonable floating point resolution for offset confidence */
#define VED__YWRT_CONF_DLTF (1E-3F)

/* Maximum possible increase (innovation) of confidence per learn step */
#define VED__YWRT_CONF_MAX_INNOV (0.10F)

/* Number of learn cycles during vehicle standstill */
#define VED__YWRT_MAX_LEARN_CNT (6UL)

/* Maximum and minimal confidence values for fixed point presentation */
#define VED__YWRT_CONF_RES_FXP ((ui8_t)1UL)  /* => 0.39 %  */
#define VED__YWRT_OFFS_THRD_FXP ((i16_t)26L) /* =>  0.15 deg/s */

/* Maximum confidence of table based offset type */
#define VED__YWRT_TEMP_OFF_MAX_QUAL (0.8F)
#define VED__YWRT_TEMP_OFF_MAX_QUAL_COR (1.0F / VED__YWRT_TEMP_OFF_MAX_QUAL)

/* Factor to make internal yaw rate quality equal to external quality */
#define VED__YWRT_TEMP_QUALITY_EQUAL_EXTERNAL (0.75F / 0.7F)

/*! Linear ramp nodes for confidency reduction of standstill offset due to
 * temperature variation */
#define VED__YWRT_TEMP_DRIFT_MAX                 \
    7.5F /*!< Maximum temperature deviation in C \
          */
#define VED__YWRT_TEMP_DRIFT_MIN                                           \
    1.5F                           /*!< Minimum temperature deviation in C \
                                    */
#define VED__YWRT_MIN_RED_FAK 1.0F /*!< Minimum confidence reduction factor */
#define VED__YWRT_MAX_RED_FAK \
    0.001F /*!< Maximum confidence reduction factor  */

/*! Maximum change rate (with respect to cycle time) of offset during offse type
 * change */
#define VED__YWRT_OFFS_THRES ((float32)DEG2RAD(0.01F))

/*! Ensure that number of table nodes match with mininum and maximum temperature
 */
/* Check not possible due to VED__YWRT_NO_NODES is no define */
/* #if ( ( (
(VED__YWRT_TBL_MAX_TEMP-VED__YWRT_TBL_MIN_TEMP)/VED__YWRT_TBL_GAP)+1L) !=
VED__YWRT_NO_NODES ) #error "Number of table nodes does not match with minium
and maximum temperature" #endif
*/

/*****************************************************************************
  MACROS
*****************************************************************************/

/* Helper macros to get access to data interfaces */
#define YWRT_GET_ME (&VED_YwrtGlobData)

#define YWRT_GET_DATA (&VED_YwrtGlobData.Sensor)
#define YWRT_GET_OFFS (&VED_YwrtGlobData.Offset)

#define VED__YWRT_NVM_YWR_FLP(x_) ((f32_t)(x_)*VED__YWRT_YWR_SCALE)

#define VED__YWRT_NVM_YWR_FXP(x_) \
    ((i16_t)(((x_)*VED__YWRT_YWR_INV_SCALE) + 0.5F))

#define VED__YWRT_NVM_CONF_FLP(x_) ((f32_t)(x_)*VED__YWRT_CONF_SCALE)

#define VED__YWRT_NVM_CONF_FXP(x_) \
    ((ui8_t)(((x_)*VED__YWRT_CONF_INV_SCALE) + 0.5F))

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
/* Offset overtake modes */
typedef enum {
    YWRT_OFFS_TAKE_NO = 0UL,
    YWRT_OFFS_TAKE_LIMIT = 1UL,
    YWRT_OFFS_TAKE_UNLIMIT = 2UL
} eYwrtTakeOvr_t;

#if (CFG_USE_COMPACT_ENUMS)
typedef ui8_t YwrtTakeOvr_t; /*!< using 8bit type for the real enum (32bit type)
                                YwrtTakeOvr_t to save memory*/
#else
typedef eYwrtTakeOvr_t YwrtTakeOvr_t;
#endif

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  LOCAL FUNCTION PROTOTYPES
*****************************************************************************/
static void VED_wrtCalcOffset(const VED_YwrtSenData_t *pSen,
                              const proVEDPrtList_t *proPorts,
                              VED_YwrtOffsData_t *pOffs);
static void VED_wrtCalcTblOffset(const VED_YwrtSenData_t *pSen,
                                 const proVEDPrtList_t *proPorts,
                                 VED_YwrtOffsTblData_t *pOffsTbl);
static void VED_YwrtMergeOffset(VED_YwrtOffsData_t *pOffs,
                                float32 offsTbl,
                                float32 confTbl);
static void VED_YwrtCalcFinalOffset(const VED_YwrtSenData_t *pSen,
                                    VED_YwrtOffsData_t *pOffs);

static void VED_YwrtCalcSensor(const reqVEDPrtList_t *reqPorts,
                               VED_YwrtSenData_t *pSen);
static bool_t VED_YwrtLearnValuePresent(
    const VED_NvYwrtLearnTable_t *pYwrTmpTbl);

static void VED_YwrtGetTableNode(const VED_NvYwrtLearnTable_t *tbl,
                                 VED_YwrtLearnNode_t *ndInt);
static void VED_YwrtSetTableNode(VED_NvYwrtLearnTable_t *tbl,
                                 const VED_YwrtLearnNode_t *ndInt);
static void VED_YwrtCompletePendingWrites(VED_NvYwrtLearnTable_t *tbl);

static void VED_YwrtInterpTable(const VED_NvYwrtLearnNode_t *node,
                                const ui8_t *xi,
                                i16_t *y_out,
                                ui32_t len);

static void VED_YwrtCommonInit(void);
static void VED_YwrtOffsetInit(const proVEDPrtList_t *proPorts);
static void VED_YwrtCalcOffsetTime(void);
static void VED_YwrtCorrectTable(VED_NvYwrtLearnTable_t *pYwrTmpTbl);

static void VED_YwrCalcToAutocode(VED_YwrtOffsData_t *Offs);

/*****************************************************************************
  VARIABLES
*****************************************************************************/
SET_MEMSEC_VAR(VED_YwrtGlobData)
static VED_YwrtData_t
    VED_YwrtGlobData; /*!< @VADDR: 0x20017000 @VNAME: VED_Ywrt @ALLOW: ved__priv
                         @cycleid: ved__cycle_id*/

/* ***********************************************************************
  @fn               VED_wrtExec*/ /*!

  @brief            Determine operating sequence for vehicle dynamics observer

  @param[in]        reqPorts
  @param[in]        mif
  @param[in]        proPorts
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_YwrtExec(const reqVEDPrtList_t *reqPorts,
                  VED_ModIf_t *mif,
                  const proVEDPrtList_t *proPorts) {
    YWRT_GET_ME->Io.mif = mif;

    /* Distinguish between different operating states */
    if (VED__CTRL_GET_STATE(VED_CTRL_STATE_RUNNING,
                            reqPorts->pCtrl->CtrlMode)) {
        /*<--- Execution path for normal operating mode --->*/
        VED_YwrtSenData_t *pSen =
            YWRT_GET_DATA; /* Yaw rate sensor related data */
        VED_YwrtOffsData_t *pOffs =
            YWRT_GET_OFFS; /* Offset learning related data */

        /* Determine standstill flag for yaw rate offset compensation */

        pSen->StandstillOK = (bool_t)((mif->LongMot.MotState.MotState ==
                                       VED_LONG_MOT_STATE_STDST) &&
                                      (mif->LongMot.MotState.Confidence >
                                       VED__PAR_YWR_STST_CONF_MIN));

        /* Start sensor signal processing */
        VED_YwrtCalcSensor(reqPorts, pSen);

        /* Start offset calculation */
        VED_wrtCalcOffset(pSen, proPorts, pOffs);
    } else {
        /*<--- Execution path for initialization mode  --->*/
        VED_YwrtCommonInit();
    }

    /* Update offset times */
    VED_YwrtCalcOffsetTime();

    /* Fill the autocode interface */
    VED_YwrCalcToAutocode(YWRT_GET_OFFS);

    return;
}

/* ***********************************************************************
  @fn               VED_YwrtInit*/ /*!

  @brief            initialize module data

  @description       
  @param[in]        proPorts
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_YwrtInit(const proVEDPrtList_t *proPorts) {
    VED_YwrtCommonInit();
    VED_YwrtOffsetInit(proPorts);

    return;
}

/* ***********************************************************************
  @fn                     VED_YwrCalcToAutocode */ /*!
  @brief                  Fill the interfact to the autocode

  @description

  @param[in]              Offs
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -

**************************************************************************** */
static void VED_YwrCalcToAutocode(VED_YwrtOffsData_t *Offs) {
    Offs->ToAutocode.OffsData.quality = Offs->Quality;
    Offs->ToAutocode.OffsData.var = 0.000000001F;

    /* Detection of yaw rate offset changes */
#if (CFG_VED__EX_YWR_NVM)
    if (((Offs->OffsElpsdTime < (float32)1.0F) &&
         (Offs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST)) ||
        ((Offs->OffsType == (VED_YwrOffsType_t)OFFS_TEMPER_TABLE) ||
         (Offs->OffsType == (VED_YwrOffsType_t)OFFS_DYN_APPRX) ||
         (Offs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST_EEPROM)) ||
        (YWRT_GET_ME->Io.mif->FirstCycleDone == FALSE)) {
#else
    if (((Offs->OffsElpsdTime < (float32)1.0F) &&
         (Offs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST)) ||
        ((Offs->OffsType == (VED_YwrOffsType_t)OFFS_TEMPER_TABLE) ||
         (Offs->OffsType == (VED_YwrOffsType_t)OFFS_DYN_APPRX)) ||
        (VED__IS_FIRST_CYCLE_DONE() == FALSE)) {
#endif
        /* new offset was taken */
        /* set state */
        if (Offs->OffsType == (VED_YwrOffsType_t)OFFS_STANDST_EEPROM) {
            /* state 2 = EEPROM offset */
            Offs->ToAutocode.OffsData.state = 2U;
        } else {
            /* state = 1 -> set dynamic offset to stand still offset */
            Offs->ToAutocode.OffsData.state = 1U;
        }
        Offs->ToAutocode.OffsData.offset = Offs->YawRateOffset;
    } else {
        if (Offs->OffsType == (VED_YwrOffsType_t)OFFS_NON) {
            /* state = 3 -> yaw stand still offset never ever estimated */
            Offs->ToAutocode.OffsData.offset = 0.0F;
            Offs->ToAutocode.OffsData.state = 3U;
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

/* ***********************************************************************
  @fn               VED_YwrInitTableOffset */ /*!
  @brief            Initialize offset data

  @description      see brief description

  @param[in]        pOffsTbl
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrInitTableOffset(VED_YwrtOffsTblData_t *pOffsTbl) {
    pOffsTbl->bWrLeft = FALSE;
    pOffsTbl->bWrRight = FALSE;

    pOffsTbl->LearnCnt = (uint16)0;
    pOffsTbl->LearnAgainCnt = (uint16)0;
    pOffsTbl->LastOffsTemp = 0.F;

    pOffsTbl->Offset = 0.F;
    pOffsTbl->Conf = 0.F;
    pOffsTbl->Valid = FALSE;
    pOffsTbl->Checked = FALSE;

    pOffsTbl->Left.Conf = 0.F;
    pOffsTbl->Left.Offset = 0.F;
    pOffsTbl->Left.No = (ui8_t)0;

    pOffsTbl->Right.Conf = 0.F;
    pOffsTbl->Right.Offset = 0.F;
    pOffsTbl->Right.No = (ui8_t)0;

    VED_StatIntervalInit(&pOffsTbl->SmpYwRt);
    VED_StatIntervalInit(&pOffsTbl->SmpTemp);

    return;
}

/* ***********************************************************************
  @fn               VED_YwrtOffsetInit */ /*!
  @brief            Initialize offset data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrtOffsetInit(const proVEDPrtList_t *proPorts) {
    VED_YwrtOffsData_t *pOffs = YWRT_GET_OFFS;
    VED_YwrtOffsTblData_t *pOffsTbl = &(pOffs->Tbl); /* Offset data   */
    VED_YwrStandStillOffs_t *pOffsStSt = &(pOffs->StSt);

    if (proPorts->pYwrtTempTable != (void *)0) {
        VED__YWRT_CLEAR_ALL(proPorts->pYwrtTempTable->statRd);

        VED__YWRT_CLEAR_ALL(proPorts->pYwrtTempTable->statWr);

        VED__YWRT_CLEAR_ALL(proPorts->pYwrtTempTable->statWrpd);
        proPorts->pYwrtTempTable->cntWr = 0x0UL;
        proPorts->pYwrtTempTable->prodOffset = VED__YWRT_PROD_OFFSET_SNA;
    }

    pOffs->MaxQuality = 0.F;
    pOffs->Quality = 0.F;
    pOffs->confStSt = 0.F;
    pOffs->confTbl = 0.F;
    pOffs->OffsType = (VED_YwrOffsType_t)OFFS_NON;
    pOffs->OffsElpsdTime = 0.F;
    pOffs->YawRateOffset = 0.F;
    pOffs->Temperature = 0.F;

    /* Initialize offset table data */
    VED_YwrInitTableOffset(pOffsTbl);

    /* Initialize standstill offset data */
    VED_YwrInitStandStillOffset(pOffsStSt);

    return;
}

/* ***********************************************************************
  @fn               VED_YwrtCommonInit */ /*!
  @brief            Initialize non offset data

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrtCommonInit(void) {
    VED_YwrtSenData_t *pSen = YWRT_GET_DATA;

    pSen->Gradient = 0.F;
    pSen->StandstillOK = FALSE;
    pSen->Valid = FALSE;
    pSen->YawRate = 0.F;
    pSen->YawRateOld = 0.F;
    pSen->YwFirstCycleDone = FALSE;
    pSen->Tempr = 0.F;
    pSen->TemprOld = 0.F;
    pSen->obsOffs = FALSE;

    return;
}

/* ***********************************************************************
  @fn                     VED_YwrtCalcOffsetTime */ /*!
  @brief                  Calculate time since last standstill calibration

  @description            see brief description

  @param[in]              -
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrtCalcOffsetTime(void) {
    float32 CycleTime;
    VED_YwrtOffsData_t *OffsData = YWRT_GET_OFFS;

    /* Get cycle time */
    CycleTime = VED_GetCycleTime();

    /* Calculate time since last offset calibration */
    if (OffsData->OffsElpsdTime >= YWR_OFFS_ELPSD_TIME_MAX) {
        OffsData->OffsElpsdTime = YWR_OFFS_ELPSD_TIME_MAX;
    } else {
        OffsData->OffsElpsdTime += CycleTime;
    }
    return;
}

/* ***********************************************************************
  @fn               VED_YwrtCalcSensor */ /*!
  @brief            Calculate yaw rate sensor signals

  @description      see brief description

  @param[in]        -
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrtCalcSensor(const reqVEDPrtList_t *reqPorts,
                               VED_YwrtSenData_t *pSen) {
    pSen->obsOffs = FALSE;

    if ((VED_GET_IO_STATE(VED_SIN_POS_YWRINT,
                          reqPorts->pVehicleInputSignals->VehSigMain.State) ==
         VED_IO_STATE_VALID) &&
        (VED_GET_IO_STATE(VED_SIN_POS_YWRINT_TEMP,
                          reqPorts->pVehicleInputSignals->VehSigMain.State) ==
         VED_IO_STATE_VALID)) {
        /* Sample input data */
        pSen->Valid = TRUE;
        pSen->YawRate = reqPorts->pVehicleInputSignals->VehSigMain.YawRateInt;
        pSen->Tempr = reqPorts->pVehicleInputSignals->VehSigMain.YawRateIntTemp;

        /* Calculate yaw rate acceleration */
        pSen->Gradient =
            VED_DifferentiateCycleTime(pSen->YawRate, pSen->YawRateOld,
                                       pSen->Gradient, VED__YWRT_DIFF_FILT);

        /* Calculate temperature gradient */
        pSen->TempGrad = VED_DifferentiateCycleTime(pSen->Tempr, pSen->TemprOld,
                                                    pSen->TempGrad,
                                                    VED__YWRT_TEMP_DIFF_FILT);

        /* Determine observabilty of yaw rate zero point offset */
        if (pSen->StandstillOK != FALSE) {
            if ((fABS(pSen->YawRate) < VED__YWRT_YWR_MAX_OFFSET) &&
                ((pSen->Tempr >= VED__YWRT_FIRST_NODE_TEMP) &&
                 (pSen->Tempr <= VED__YWRT_LAST_NODE_TEMP)) &&
                (fABS(pSen->Gradient) < VED__YWRT_YWR_GRAD_MAX)) {
                pSen->obsOffs = TRUE;
            }
        }
        /* Save values for next cycles */
        pSen->YawRateOld = pSen->YawRate;
        pSen->TemprOld = pSen->Tempr;
        pSen->YwFirstCycleDone = TRUE;
    } else {
        /* Init sensor related signal data */
        pSen->Valid = FALSE;
        pSen->YawRate = 0.F;
        pSen->YawRateOld = 0.F;
        pSen->Tempr = 0.F;
        pSen->TemprOld = 0.F;
    }
    return;
}

/* ***********************************************************************
  @fn                     VED_wrtCalcOffset */ /*!
  @brief                  Calculate yaw rate offset

  @description            see brief description

  @param[in]              Yaw rate sensor, yaw rate offset
  @param[out]             yaw rate offset
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_wrtCalcOffset(const VED_YwrtSenData_t *pSen,
                              const proVEDPrtList_t *proPorts,
                              VED_YwrtOffsData_t *pOffs) {
    pOffs->StSt.Observable = pSen->obsOffs;
    pOffs->StSt.StandstillOK = pSen->StandstillOK;

    /* Run standstill offset estimation */
    VED_YwrEstStandstillOffset(pSen->YawRate, pSen->Valid, &pOffs->StSt);

    /* Run standstill offset table learning */
    VED_wrtCalcTblOffset(pSen, proPorts, &pOffs->Tbl);

    /* Calculate output offset */
    VED_YwrtCalcFinalOffset(pSen, pOffs);

    return;
}

/* ***********************************************************************
  @fn                     VED_YwrtCalcFinalOffset */ /*!
  @brief                  Calculate final yaw rate offset

  @description            see brief description

  @param[in]              yaw rate sensor
  @param[out]             yaw rate offset
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrtCalcFinalOffset(const VED_YwrtSenData_t *pSen,
                                    VED_YwrtOffsData_t *pOffs) {
    VED_YwrtOffsTblData_t *pOffsTbl = &pOffs->Tbl;
    VED_YwrStandStillOffs_t *pOffsStSt = &pOffs->StSt;

    float32 currTemp;
    float32 absDiffTemp;

    float32 tblConf;
    float32 tblOffs;

    currTemp = pSen->Tempr;

    /* Limit maximum confidence of table offset */
    tblConf = pOffsTbl->Conf * VED__YWRT_TEMP_OFF_MAX_QUAL;
    tblOffs = pOffsTbl->Offset;

    /* If current standstill offset confidence is greater than confidence from
       previous standstill offset, take over new offset. In case of no
       standstill, max quality of standstill offset is equal zero */
    if (pOffsStSt->MaxQuality >= pOffs->confStSt) {
        /* Current standstill offset is more confident than prior one. Save
           standstill offset environmental data */

        /* Save confidence reached during standstill calibration */
        pOffs->MaxQuality = pOffsStSt->MaxQuality;

        /* Save confidence derived from table reached during standstill
         * calibration */
        pOffs->confTbl = tblConf;

        /* Save temperature during standstill calibration */
        pOffs->Temperature = currTemp;

        /* Save table offset during standstill offset take over */
        pOffs->YwRtTblOffsStSt = tblOffs;

        /* Reset offset time during last standstill offset take over */
        pOffs->OffsElpsdTime = 0.F;
    }

    /* Merge standstill offset and table correction offset */
    VED_YwrtMergeOffset(pOffs, tblOffs, tblConf);

    /* Temperature drift since last standstill calibration */
    absDiffTemp = fABS(currTemp - pOffs->Temperature);

    /* Reduce confidence in dependence of temperature drift */
    pOffs->confStSt =
        pOffs->MaxQuality * VED_LFunction(absDiffTemp, VED__YWRT_TEMP_DRIFT_MIN,
                                          VED__YWRT_TEMP_DRIFT_MAX,
                                          VED__YWRT_MIN_RED_FAK,
                                          VED__YWRT_MAX_RED_FAK);
    return;
}

/* ***********************************************************************
  @fn                     VED_YwrtTakeOffset */ /*!
  @brief                  Take new offset value

  @description            see brief description

  @param[in]              take over mode, new offset value, old offset value
  @param[out]             new offset value
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrtTakeOffset(YwrtTakeOvr_t tkOvr,
                               float32 offsNew,
                               float32 *offsOldOut) {
    switch (tkOvr) {
        case YWRT_OFFS_TAKE_LIMIT: {
            *offsOldOut =
                TUE_CML_MinMax(*offsOldOut - VED__YWRT_OFFS_THRES,
                               *offsOldOut + VED__YWRT_OFFS_THRES, offsNew);
        } break;

        case YWRT_OFFS_TAKE_UNLIMIT: {
            *offsOldOut = offsNew;
        } break;

        default:
        case YWRT_OFFS_TAKE_NO: {
        } break;
    }
    return;
}

/* ***********************************************************************
  @fn                     VED_YwrtMergeOffset */ /*!
  @brief                  Merge standstill offset and table based yaw rate offset 

  @description            see brief description

  @param[in]              Offset data
  @param[in]              Table offset data
  @param[out]             Final yaw rate offset with confidence
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_YwrtMergeOffset(VED_YwrtOffsData_t *pOffs,
                                float32 offsTbl,
                                float32 confTbl) {
    YwrtTakeOvr_t tkOvr = (YwrtTakeOvr_t)YWRT_OFFS_TAKE_NO;

    if ((pOffs->StSt.MaxQuality > 0.F) || (pOffs->confTbl > 0.F)) {
        if (pOffs->StSt.StandstillOK == FALSE) {
            /* Limit gradient of offset changes during driving */
            tkOvr = (YwrtTakeOvr_t)YWRT_OFFS_TAKE_LIMIT;
        } else {
            /* Take over with no limitation if new standstill or table offset
             * has more confidence */
            if ((pOffs->StSt.MaxQuality >= pOffs->confStSt) ||
                (pOffs->confStSt < confTbl)) {
                tkOvr = (YwrtTakeOvr_t)YWRT_OFFS_TAKE_UNLIMIT;
            }
        }
    }

    /* Offset change required */
    if (tkOvr != (YwrtTakeOvr_t)YWRT_OFFS_TAKE_NO) {
        /* Selection of offset type */
        if ((pOffs->MaxQuality >= VED__YWR_OFF_STST_CALC_TIME_Q_MAX) &&
            (pOffs->confTbl >= VED__YWRT_TEMP_OFF_MAX_QUAL) &&
            (confTbl >= VED__YWRT_TEMP_OFF_MAX_QUAL)) {
            /* Standstill, table offset learned during standstill and current
               interpolated table offset has maximum confidence */
            float32 newOffs;

            /* Update current active offset type */
            pOffs->OffsType = (VED_YwrOffsType_t)OFFS_DYN_APPRX;

            /* Set confidence to maximum value */
            pOffs->Quality = VED__YWR_OFF_STST_CALC_TIME_Q_MAX;

            /* Use offset table to adapt last standstill offset */
            newOffs =
                pOffs->StSt.YawRateOffset + (offsTbl - pOffs->YwRtTblOffsStSt);

            /* Take adapted standstill offset value */
            VED_YwrtTakeOffset(tkOvr, newOffs, &pOffs->YawRateOffset);
        } else if (pOffs->confStSt >= confTbl) {
            /* Confidence of last calibrated standstill is better than current
             * table offset */

            /* Update current active offset type */
            pOffs->OffsType = (VED_YwrOffsType_t)OFFS_STANDST;

            /* Use temperature adapted confidence of last standstill offset */
            pOffs->Quality = pOffs->confStSt;

            /* Take last standstill offset value */
            VED_YwrtTakeOffset(tkOvr, pOffs->StSt.YawRateOffset,
                               &pOffs->YawRateOffset);
        } else {
            /* Current table offset is better than last calibrated standstill
             * offset */

            /* Update current active offset type */
            pOffs->OffsType = (VED_YwrOffsType_t)OFFS_TEMPER_TABLE;

            /* Use confidence of table interpolated offset */
            pOffs->Quality = confTbl;

            /* Take table offset value */
            VED_YwrtTakeOffset(tkOvr, offsTbl, &pOffs->YawRateOffset);
        }
    }

    /* Modify the output quality, FR:32207
    Increase (internal gyro) offset confidence by factor
     ��-> 1/0.8    (Remove lowering in caes of temp table offset) and
       -> 0.75/0.7 (Adapt confidence value to equivalent to external gyro offset
    compensation) */
    pOffs->Quality = MIN(pOffs->Quality * VED__YWRT_TEMP_OFF_MAX_QUAL_COR *
                             VED__YWRT_TEMP_QUALITY_EQUAL_EXTERNAL,
                         1.0F);

    return;
}

/* ***********************************************************************
  @fn               VED_YwrtGetNodesOfInterest */ /*!
  @brief            Get working nodes according to current temperature 
                    operating point

  @description      see brief description

  @param[in]        tempr
  @param[in]        pTbl
 
  @param[out]       pOffs Offset-Data
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrtGetNodesOfInterest(float32 tempr,
                                       const VED_NvYwrtLearnTable_t *pTbl,
                                       VED_YwrtOffsTblData_t *pOffs) {
    ui8_t Left, Right;

    /* Determine number of left (lower) temperature node */

    Left =
        (ui8_t)((tempr - VED__YWRT_FIRST_NODE_TEMP) * VED__YWRT_NODE_INV_WIDTH);

    /* Determine number of right (higher) temperature node */

    Right = (ui8_t)((ui32_t)Left + 1UL);

    /* Update left node number */
    pOffs->Left.No = Left;

    /* Update right node number */
    pOffs->Right.No = Right;

    /* Copy left node offset data from learning table */
    VED_YwrtGetTableNode(pTbl, &pOffs->Left);

    /* Copy right node offset data form learning table */
    VED_YwrtGetTableNode(pTbl, &pOffs->Right);

    return;
}

/* ***********************************************************************
  @fn               VED_YwrtMeasUpdateNode */ /*!
  @brief            Update offset table node with current measurement

  @description      see brief description

  @param[in]        gain, measured offset value, measurement confidence
  @param[out]       offset table node
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrtMeasUpdateNode(VED_YwrtLearnNode_t *node,
                                   f32_t gain,
                                   f32_t meas,
                                   f32_t conf) {
    const f32_t redGain_c = (1.F / VED__YWRT_RED_LEARN_SPEED);

    f32_t gainOffs;
    f32_t gainConf = gain * redGain_c;
    f32_t dConf = conf - node->Conf;

    /* Speed transient effect during first update values */
    if (node->Conf < VED__YWRT_LEARN_SPEED_CONF_MIN) {
        /* First learn value in application field, use full weight */
        gainOffs = gain;
    } else {
        gainOffs = gain * VED_LFunction(
                              node->Conf, VED__YWRT_LEARN_SPEED_CONF_MIN,
                              VED__YWRT_LEARN_SPEED_CONF_MAX, 0.8F, redGain_c);
    }

    /* Limit minimum and maximal confidence increase per learn iteration */
    if (dConf > C_F32_DELTA) {
        /* Limit the maximum possible increase */
        if ((gainConf * (conf - node->Conf)) > VED__YWRT_CONF_MAX_INNOV) {
            gainConf = VED__YWRT_CONF_MAX_INNOV / dConf;
        }

        /* Limit the minimum possible increase to table resolution */
        if ((gainConf * (conf - node->Conf)) < VED__YWRT_CONF_SCALE) {
            gainConf = VED__YWRT_CONF_SCALE / dConf;
        }
    }

    /* Update node values */
    node->Offset = ((1.0F - gainOffs) * node->Offset) + (gainOffs * meas);
    node->Conf = ((1.0F - gainConf) * node->Conf) + (gainConf * conf);

    return;
}

/* ***********************************************************************
  @fn               VED_YwrtCalcOffsTblCurr */ /*!
  @brief            Calculate current yaw rate offset based on offset table

  @description      see brief description


  @return           void
  @param[in]        offset table, temperature
  @param[out]       offset

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrtCalcOffsTblCurr(const VED_NvYwrtLearnTable_t *nvTbl,
                                    float32 currTempr,
                                    VED_YwrtOffsTblData_t *pOffs) {
    VED_YwrtLearnNode_t CurrLeft, CurrRight;
    float32 tblTempr;
    float32 ipOffs;
    float32 ipConf;
    f32_t currAlpha;

    /* Limit current temperature to offset correction table range */
    tblTempr = TUE_CML_MinMax(VED__YWRT_FIRST_NODE_TEMP,
                              VED__YWRT_LAST_NODE_TEMP, currTempr);

    /* Determine left node number */

    CurrLeft.No = (ui8_t)((tblTempr - VED__YWRT_FIRST_NODE_TEMP) *
                          VED__YWRT_NODE_INV_WIDTH);

    /* Determine right node number */

    CurrRight.No = (ui8_t)((ui32_t)CurrLeft.No + 1UL);

    /* Select determined nodes */
    VED_YwrtGetTableNode(nvTbl, &CurrLeft);
    VED_YwrtGetTableNode(nvTbl, &CurrRight);

    /* Calculate normalized distance with respect to left node */
    currAlpha = (tblTempr - (VED__YWRT_FIRST_NODE_TEMP +
                             ((f32_t)CurrLeft.No * VED__YWRT_NODE_WIDTH))) *
                VED__YWRT_NODE_INV_WIDTH;

    /* Interpolate offset and confidence values */
    ipOffs =
        CurrLeft.Offset + (currAlpha * (CurrRight.Offset - CurrLeft.Offset));
    ipConf = CurrLeft.Conf + (currAlpha * (CurrRight.Conf - CurrLeft.Conf));

    /* Fill offset interface */
    pOffs->Offset = ipOffs;
    pOffs->Conf = ipConf;
    pOffs->Valid = VED_YwrtLearnValuePresent(nvTbl);

    return;
}

/* ***********************************************************************
  @fn               VED_YwrtInitWithFirstLearnValue */ /*!
  @brief            Initialize all nodes with first standstill offset if 
                    no learn value is saved in nvm table

  @description      see brief description

  @param[in]        offset value
  @param[in,out]    offset table
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrtInitWithFirstLearnValue(VED_NvYwrtLearnTable_t *pYwrTmpTbl,
                                            float32 offset) {
    if (VED_YwrtLearnValuePresent(pYwrTmpTbl) == FALSE) {
        /* No learn value is present */
        ui32_t idx;

        /* Set all nodes to offset value  */
        for (idx = 0UL; idx < VED__YWRT_NO_NODES; idx++) {
            pYwrTmpTbl->Node[idx].Offset = VED__YWRT_NVM_YWR_FXP(offset);
            pYwrTmpTbl->Node[idx].Conf = VED__YWRT_CONF_RES_FXP;
        }
        /* Set write flag for all table nodes */

        VED__YWRT_NODES_MASK_ALL(pYwrTmpTbl->statWr);

        /* Clear pending writes for all nodes */

        VED__YWRT_NODES_CLEAR_ALL(pYwrTmpTbl->statWrpd);
    }
    return;
}

/* ***********************************************************************
  @fn               VED_YwrtFillUnlearnedNodes */ /*!

  @brief            Fill unlearned nodes located between learned nodes by
                    interpolation
  @description      see brief description

  @param[in]        offset table
  @param[in]        temperature table
  @param[out]       offset
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrtFillUnlearnedNodes(const VED_YwrtOffsTblData_t *pOffsTbl,
                                       VED_NvYwrtLearnTable_t *pYwrTmpTbl) {
    i32_t ndLowStart;
    i32_t ndHighStart;
    VED_NvYwrtLearnNode_t *node = pYwrTmpTbl->Node;
    i32_t offsIntp;
    i32_t offsDelta;

    /* Select node to start interpolation towards lower higher temperature nodes
     */
    if ((pOffsTbl->bWrLeft != FALSE) && (pOffsTbl->bWrRight != FALSE)) {
        /* Two nodes have been updated */
        ndLowStart = (i32_t)pOffsTbl->Left.No;
        ndHighStart = (i32_t)pOffsTbl->Right.No;
    } else if ((pOffsTbl->bWrLeft != FALSE) && (pOffsTbl->bWrRight == FALSE)) {
        /* One node has been updated */
        ndLowStart = (i32_t)pOffsTbl->Left.No;
        ndHighStart = (i32_t)pOffsTbl->Left.No;
    } else if ((pOffsTbl->bWrLeft == FALSE) && (pOffsTbl->bWrRight != FALSE)) {
        /* One node has been updated */
        ndLowStart = (i32_t)pOffsTbl->Right.No;
        ndHighStart = (i32_t)pOffsTbl->Right.No;
    } else {
        /* No update has arisen */
        ndLowStart = 0L;
        ndHighStart = (i32_t)VED__YWRT_NO_NODES - 1L;
    }

    /* Update nodes towards lower temperature */
    if (ndLowStart > 0L) {
        /* There are nodes below start node */

        i32_t ndLowEnd =
            -1L; /* Nearest node in lower direction with enough confidence */
        i32_t idxNd;

        /* Step down nodes as long as node is located with enough confidence */
        for (idxNd = ndLowStart - 1L; (idxNd >= 0L) && (ndLowEnd < 0L);
             idxNd--) {
            if (node[idxNd].Conf > VED__YWRT_CONF_RES_FXP) {
                ndLowEnd = idxNd;
            }
        }

        /* If located node is not adjacent to updated node start filling */
        if (ndLowEnd < (ndLowStart - 1L)) {
            /* If no node was located with enough confidence, interpolation is
             * not possible */
            if (ndLowEnd < 0L) {
                /* Zero delta, no interpolation */
                offsDelta = 0;
            } else {
                /* Calculate delta per node derived by linear interpolation
                 * between start and end nodes */

                offsDelta =
                    ((i32_t)(node[ndLowEnd].Offset - node[ndLowStart].Offset) /
                     (ndLowStart - ndLowEnd));
            }

            /* Initialize fill with start node */
            offsIntp = node[ndLowStart].Offset;

            /* Fill nodes between start and end node */
            for (idxNd = ndLowStart - 1L; idxNd > ndLowEnd; idxNd--) {
                i32_t diff;

                /* Update interpolation value for current node */
                offsIntp += offsDelta;

                /* Calculate difference between current table node and potential
                 * new value */
                diff = offsIntp - (i32_t)node[idxNd].Offset;

                /* Only update node if there is a significant difference */

                if (ABS(diff) > VED__YWRT_OFFS_THRD_FXP) {
                    /* Update node, but leave confidence untouched */
                    node[idxNd].Offset = (i16_t)offsIntp;

                    VED__YWRT_NODE_SET_VALID(idxNd, pYwrTmpTbl->statWrpd);
                }
            }
        }
    }

    /* Update nodes towards higher temperature */

    if (ndHighStart < (i32_t)(VED__YWRT_NO_NODES - 1UL)) {
        i32_t ndHighEnd =
            (i32_t)VED__YWRT_NO_NODES; /* Nearest node in higher direction with
                                          enough confidence */
        i32_t ii;

        /* Step up nodes as long as node is located with enough confidence */

        for (ii = ndHighStart + 1L;
             (ii < (i32_t)VED__YWRT_NO_NODES) &&
             (ndHighEnd > (i32_t)(VED__YWRT_NO_NODES - 1UL));
             ii++) {
            if (node[ii].Conf > VED__YWRT_CONF_RES_FXP) {
                ndHighEnd = ii;
            }
        }

        /* If located node is not adjacent to updated node start filling */
        if (ndHighEnd > (ndHighStart + 1L)) {
            /* If no node was located with enough confidence, interpolation is
             * not possible */

            if (ndHighEnd > (i32_t)(VED__YWRT_NO_NODES - 1UL)) {
                /* Zero delta, no interpolation */
                offsDelta = 0;
            } else {
                /* Calculate delta per node derived by linear interpolation
                 * between start and end nodes */

                offsDelta = ((i32_t)(node[ndHighEnd].Offset -
                                     node[ndHighStart].Offset) /
                             (ndHighEnd - ndHighStart));
            }

            /* Initialize fill with start node */
            offsIntp = node[ndHighStart].Offset;

            /* Fill nodes between start and end node */
            for (ii = ndHighStart + 1L; ii < ndHighEnd; ii++) {
                i32_t diff;

                /* Update interpolation value for current node */
                offsIntp += offsDelta;

                /* Calculate difference between current table node and potential
                 * new value */
                diff = offsIntp - (i32_t)node[ii].Offset;

                /* Only update node if there is a significant difference */

                if (ABS(diff) > VED__YWRT_OFFS_THRD_FXP) {
                    node[ii].Offset = (i16_t)offsIntp;

                    VED__YWRT_NODE_SET_VALID(ii, pYwrTmpTbl->statWrpd);
                }
            }
        }
    }
    return;
}

/* ***********************************************************************
  @fn               VED_wrtCalcTblOffset */ /*!
  @brief            Calculate yaw rate sensor offset

  @description      see brief description

  @param[in]        sensor data
  @param[in]        offset table
  @param[out]       provided ports
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_wrtCalcTblOffset(const VED_YwrtSenData_t *pSen,
                                 const proVEDPrtList_t *proPorts,
                                 VED_YwrtOffsTblData_t *pOffsTbl) {
    const f32_t fir_len_min_max_c[2] = {
        15.F, 30.F}; /* FIR-Length nodes = Number of observed samples */
    const f32_t lrn_cnt_min_max_c[2] = {4.F, 6.F}; /* Learn counter nodes */
    f32_t alpha;
    f32_t intp_offs;
    f32_t ywRateDiff;

    /* Init temp table with production offset */
    if (proPorts->pYwrtTempTable->prodOffset != VED__YWRT_PROD_OFFSET_SNA) {
        VED_YwrtInitWithFirstLearnValue(
            proPorts->pYwrtTempTable,
            (f32_t)proPorts->pYwrtTempTable->prodOffset * VED__YWRT_PROD_SCALE);
    }

    if (pSen->obsOffs != FALSE) {
        f32_t fir_len; /* lenght of input filter */

        /* Increase filter length with rising learning iterations */
        fir_len = VED_LFunction((f32_t)pOffsTbl->LearnCnt, lrn_cnt_min_max_c[0],
                                lrn_cnt_min_max_c[1], fir_len_min_max_c[0],
                                fir_len_min_max_c[1]);

        /* Update input (moving average) filter */
        VED_StatIntervalAdd(&pOffsTbl->SmpYwRt, pSen->YawRate, 1.0F);
        VED_StatIntervalAdd(&pOffsTbl->SmpTemp, pSen->Tempr, 1.0F);

        /* Maximum number of learning iteration during current standstill
         * reached */

        if ((ui32_t)pOffsTbl->LearnCnt <= VED__YWRT_MAX_LEARN_CNT) {
            /* Input filter hold enough samples to update standstill offset */
            if (pOffsTbl->SmpYwRt.Volume > (fir_len - C_F32_DELTA)) {
                f32_t ywr_mean;
                f32_t ywr_dev;
                f32_t tmpr_mean;

                /* First learn value available from previous learning iteration
                 */
                if (pOffsTbl->LearnCnt > (uint16)0) {
                    /* Non-volaltile memory write request present from last
                     * learning iteraton */

                    /* Bitmask enabling nvm write access in dependence of write
                     * counter */

                    const ui32_t NvmWriteMask_c =
                        ((0UL << 0UL) | (1UL << 1UL) | (0UL << 2UL) |
                         (0UL << 3UL) | (0UL << 4UL) | (0UL << 5UL) |
                         (1UL << 6UL) | (0UL << 7UL) | (0UL << 8UL));

                    /* Update NVM table left node, if is required or permitted
                     */
                    if (pOffsTbl->bWrLeft != FALSE) {
                        /* Update RAM table left node with new learn value */
                        VED_YwrtSetTableNode(proPorts->pYwrtTempTable,
                                             &pOffsTbl->Left);

                        /* Trigger pending write flag for left node */

                        VED__YWRT_NODE_SET_VALID(
                            pOffsTbl->Left.No,
                            proPorts->pYwrtTempTable->statWrpd);
                    }

                    /* Update NVM table right node, if is required or permitted
                     */
                    if (pOffsTbl->bWrRight != FALSE) {
                        /* Update RAM table right node with new learn value */
                        VED_YwrtSetTableNode(proPorts->pYwrtTempTable,
                                             &pOffsTbl->Right);

                        /* Trigger pending write flag for right node */

                        VED__YWRT_NODE_SET_VALID(
                            pOffsTbl->Right.No,
                            proPorts->pYwrtTempTable->statWrpd);
                    }

                    /* Update unlearned nodes */
                    VED_YwrtFillUnlearnedNodes(pOffsTbl,
                                               proPorts->pYwrtTempTable);

                    /* Update of nvm memory required */

                    if (((pOffsTbl->bWrLeft != FALSE) ||
                         (pOffsTbl->bWrRight != FALSE)) &&
                        (((1UL << ((ui32_t)pOffsTbl->LearnCnt)) &
                          NvmWriteMask_c) != 0UL)) {
                        /* Read table write counter from nonvolatile memory */

                        if (VED__YWRT_NODE_IS_VALID(
                                VED__YWRT_BIT_POS_CNTR,
                                proPorts->pYwrtTempTable->statRd)) {
                            /* Number of write accesses to temp table */
                            proPorts->pYwrtTempTable->cntWr++;

                            VED__YWRT_NODE_SET_VALID(
                                VED__YWRT_BIT_POS_CNTR,
                                proPorts->pYwrtTempTable->statWr);
                        }
                        /* Flush pending nvm write updates */
                        VED_YwrtCompletePendingWrites(proPorts->pYwrtTempTable);
                    }

                    /* New values transferred */
                    pOffsTbl->bWrLeft = FALSE;
                    pOffsTbl->bWrRight = FALSE;
                }

                /* Calculate input filter output */
                VED_StatIntervalMeanDev(&pOffsTbl->SmpYwRt);
                VED_StatIntervalMeanDev(&pOffsTbl->SmpTemp);

                /* Save statistics before init */
                ywr_mean = pOffsTbl->SmpYwRt.Mean;
                ywr_dev = pOffsTbl->SmpYwRt.Dev;
                tmpr_mean = pOffsTbl->SmpTemp.Mean;

                /* Reset input filter data */
                VED_StatIntervalInitInput(&pOffsTbl->SmpYwRt);
                VED_StatIntervalInitInput(&pOffsTbl->SmpTemp);

                /* Save current offset temperature */
                pOffsTbl->LastOffsTemp = tmpr_mean;

                /* Standard deviaiton of observed samples must be small */
                if (ywr_dev < VED__YWRT_YWR_STD_DEV_MAX) {
                    /* Init temp table with first standstill offset */
                    VED_YwrtInitWithFirstLearnValue(proPorts->pYwrtTempTable,
                                                    ywr_mean);

                    /* Update working nodes according to temperature operating
                     * point */
                    VED_YwrtGetNodesOfInterest(
                        tmpr_mean, proPorts->pYwrtTempTable, pOffsTbl);

                    /* Get normalized distance (0..1) with respect to adjacent
                     * nodes, 0-> temp = left node,  1 -> temp = right node */
                    alpha =
                        (tmpr_mean -
                         (VED__YWRT_FIRST_NODE_TEMP +
                          ((f32_t)pOffsTbl->Left.No * VED__YWRT_NODE_WIDTH))) *
                        VED__YWRT_NODE_INV_WIDTH;

                    /* Interpolate yaw rate offset value */
                    intp_offs = pOffsTbl->Left.Offset +
                                (alpha * (pOffsTbl->Right.Offset -
                                          pOffsTbl->Left.Offset));

                    /* Calculate difference between current and interpolated yaw
                     * rate offset value*/
                    ywRateDiff = fABS(ywr_mean - intp_offs);

                    /* Discrimination of temperature location with respect to
                     * nodes  */
                    if (alpha < VED__YWRT_ALPHA_DLT) {
                        /* Measurement point is directly located at left node */

                        /* Make measurement update only if node is not
                         * completely learned or relearn step is necessary */
                        if ((pOffsTbl->Left.Conf <
                             (VED__YWRT_CONF_MAX - VED__YWRT_CONF_DLTF)) ||
                            (ywRateDiff > VED__YWRT_LEARN_THRHD)) {
                            /* If there is a significant deviation between
                               complete learned node and current measurement
                               value lower confidence of learned node to re
                               enable learning. The goal is not to relearn a
                               complete new offset value, but get an average
                               between the new and old due to hysteresis. */
                            if ((ywRateDiff > VED__YWRT_LEARN_THRHD) &&
                                (pOffsTbl->Left.Conf >
                                 (VED__YWRT_CONF_MAX_NEAR -
                                  VED__YWRT_CONF_DLTF))) {
                                pOffsTbl->Left.Conf = VED__YWRT_CONF_LEARN;
                                pOffsTbl->LearnAgainCnt++;
                            }

                            /* Only update left node */
                            VED_YwrtMeasUpdateNode(
                                &pOffsTbl->Left, VED__YWRT_ALPHA_MAX, ywr_mean,
                                VED__YWRT_CONF_MAX_NEAR);
                            pOffsTbl->bWrLeft = TRUE;
                        }
                    } else if (alpha <= VED__YWRT_ALPHA_MIN) {
                        /* Measurement point is closely located at left node */

                        /* Make measurement update only if node is not
                         * completely learned or relearn step is necessary */
                        if ((pOffsTbl->Left.Conf <
                             (VED__YWRT_CONF_MAX - VED__YWRT_CONF_DLTF)) ||
                            (ywRateDiff > VED__YWRT_LEARN_THRHD)) {
                            /* If there is a significant deviation between
                               complete learned node and current measurement
                               value lower confidence of learned and adjacent
                               node to re enable learning */
                            if ((ywRateDiff > VED__YWRT_LEARN_THRHD) &&
                                (pOffsTbl->Left.Conf >
                                 (VED__YWRT_CONF_MAX_NEAR -
                                  VED__YWRT_CONF_DLTF))) {
                                f32_t diffNewOldRight;
                                f32_t diffNewOldLeft;

                                /* Reduce confidence of near node close */
                                pOffsTbl->Left.Conf = VED__YWRT_CONF_LEARN;

                                /* If difference between new offset value and
                                   current adjacent node offset is big reset
                                   confidence. This avoids relearning due to a
                                   distorted offset value from this node. The
                                   relearn thereshold is compared against the
                                   interpolated value between left and right
                                   node */
                                diffNewOldRight =
                                    ywr_mean - pOffsTbl->Right.Offset;
                                diffNewOldLeft =
                                    ywr_mean - pOffsTbl->Left.Offset;

                                /* In case of big deviation reset confidence,
                                 * otherwise it will only be reduced */
                                if (fABS(diffNewOldRight) >
                                    VED__YWRT_LEARN_THRHD_FAR) {
                                    pOffsTbl->Right.Conf = 0.F;
                                }

                                /* Reduce confidence of adjacent node only, if
                                   deviation at current node is small. This
                                   avoids
                                   readjusting of already relearned node */
                                if ((pOffsTbl->Right.Conf >
                                     (VED__YWRT_CONF_MAX_FAR -
                                      VED__YWRT_CONF_DLTF)) &&
                                    (fABS(diffNewOldLeft) <
                                     VED__YWRT_LEARN_THRHD_NEAR)) {
                                    pOffsTbl->Right.Conf =
                                        VED__YWRT_CONF_LEARN_ADJ;
                                }

                                pOffsTbl->LearnAgainCnt++;
                            }

                            /* Update left node */
                            VED_YwrtMeasUpdateNode(
                                &pOffsTbl->Left, VED__YWRT_ALPHA_MAX, ywr_mean,
                                VED__YWRT_CONF_MAX_NEAR);
                            pOffsTbl->bWrLeft = TRUE;
                        }

                        /* Update right node */
                        if (pOffsTbl->Right.Conf <
                            (VED__YWRT_CONF_MAX_FAR - VED__YWRT_CONF_DLTF)) {
                            VED_YwrtMeasUpdateNode(
                                &pOffsTbl->Right, VED__YWRT_ALPHA_MIN, ywr_mean,
                                VED__YWRT_CONF_MAX_FAR);
                            pOffsTbl->bWrRight = TRUE;
                        }
                    } else if (alpha < VED__YWRT_ALPHA_MAX) {
                        /* Measurement point is located at the middle of left
                         * and right node */

                        /* Update left node */
                        if (pOffsTbl->Left.Conf <
                            (VED__YWRT_CONF_MAX_FAR - VED__YWRT_CONF_DLTF)) {
                            VED_YwrtMeasUpdateNode(&pOffsTbl->Left,
                                                   (1.0F - alpha), ywr_mean,
                                                   VED__YWRT_CONF_MAX_FAR);
                            pOffsTbl->bWrLeft = TRUE;
                        }

                        /* Update right node */
                        if (pOffsTbl->Right.Conf <
                            (VED__YWRT_CONF_MAX_FAR - VED__YWRT_CONF_DLTF)) {
                            VED_YwrtMeasUpdateNode(&pOffsTbl->Right, alpha,
                                                   ywr_mean,
                                                   VED__YWRT_CONF_MAX_FAR);
                            pOffsTbl->bWrRight = TRUE;
                        }
                    } else if (alpha < (VED__YWRT_ALPHA_VAL_MAX -
                                        VED__YWRT_ALPHA_DLT)) {
                        /* Measurement point is closely located at right node */

                        /* Make measurement update only if node is not
                         * completely learned or relearn step is necessary */
                        if ((pOffsTbl->Right.Conf <
                             (VED__YWRT_CONF_MAX - VED__YWRT_CONF_DLTF)) ||
                            (ywRateDiff > VED__YWRT_LEARN_THRHD)) {
                            /* If there is a significant deviation between
                               complete learned node and current measurement
                               value lower confidence of learned and adjacent
                               node to re enable learning. */
                            if ((ywRateDiff > VED__YWRT_LEARN_THRHD) &&
                                (pOffsTbl->Right.Conf >
                                 (VED__YWRT_CONF_MAX_NEAR -
                                  VED__YWRT_CONF_DLTF))) {
                                f32_t diffNewOldLeft;
                                f32_t diffNewOldRight;

                                /* Reduce confidence of current node */
                                pOffsTbl->Right.Conf = VED__YWRT_CONF_LEARN;

                                /* If difference between new offset value and
                                current adjacent node offset is big reset
                                confidence. This avoids relearning due to a
                                distorted offset value from this node. The
                                relearn thereshold is compared against the
                                interpolated value between left and right node
                              */
                                diffNewOldLeft =
                                    ywr_mean - pOffsTbl->Left.Offset;
                                diffNewOldRight =
                                    ywr_mean - pOffsTbl->Right.Offset;

                                /* In case of big deviation reset confidence,
                                 * otherwise it will be only be reduced */
                                if (fABS(diffNewOldLeft) >
                                    VED__YWRT_LEARN_THRHD_FAR) {
                                    pOffsTbl->Left.Conf = 0.F;
                                }

                                /* Reduce confidence of adjacent node only, if
                                   deviation at current node is small. This
                                   avoids
                                   readjusting of already relearned node */
                                if ((pOffsTbl->Left.Conf >
                                     (VED__YWRT_CONF_MAX_FAR -
                                      VED__YWRT_CONF_DLTF)) &&
                                    (fABS(diffNewOldRight) <
                                     VED__YWRT_LEARN_THRHD_NEAR)) {
                                    pOffsTbl->Left.Conf =
                                        VED__YWRT_CONF_LEARN_ADJ;
                                }

                                pOffsTbl->LearnAgainCnt++;
                            }

                            /* Update right node */
                            VED_YwrtMeasUpdateNode(
                                &pOffsTbl->Right, VED__YWRT_ALPHA_MAX, ywr_mean,
                                VED__YWRT_CONF_MAX_NEAR);
                            pOffsTbl->bWrRight = TRUE;
                        }

                        /* Update left node */
                        if (pOffsTbl->Left.Conf <
                            (VED__YWRT_CONF_MAX_FAR - VED__YWRT_CONF_DLTF)) {
                            VED_YwrtMeasUpdateNode(
                                &pOffsTbl->Left, VED__YWRT_ALPHA_MIN, ywr_mean,
                                VED__YWRT_CONF_MAX_FAR);
                            pOffsTbl->bWrLeft = TRUE;
                        }
                    } else {
                        /* Measurement point is directly located at right node
                         */

                        /* Make measurement update only if node is not
                         * completely learned or relearn step is necessary */
                        if ((pOffsTbl->Right.Conf <
                             (VED__YWRT_CONF_MAX - VED__YWRT_CONF_DLTF)) ||
                            (ywRateDiff > VED__YWRT_LEARN_THRHD)) {
                            /* If there is a significant deviation between
                            complete learned node and current measurement value
                            lower confidence of learned node to re enable
                            learning. The goal is not to relearn a
                            complete new offset value, but get an average
                            between the new and old due to hysteresis. */
                            if ((ywRateDiff > VED__YWRT_LEARN_THRHD) &&
                                (pOffsTbl->Right.Conf >
                                 (VED__YWRT_CONF_MAX_NEAR -
                                  VED__YWRT_CONF_DLTF))) {
                                pOffsTbl->Right.Conf = VED__YWRT_CONF_LEARN;
                                pOffsTbl->LearnAgainCnt++;
                            }

                            /* Only update right node */
                            VED_YwrtMeasUpdateNode(
                                &pOffsTbl->Right, VED__YWRT_ALPHA_MAX, ywr_mean,
                                VED__YWRT_CONF_MAX_NEAR);
                            pOffsTbl->bWrRight = TRUE;
                        }
                    } /* End of discrimination of temperature location with
                         respect to nodes  */

                    /* Update learn counter only if table update is pending */
                    if ((pOffsTbl->bWrLeft != FALSE) ||
                        ((pOffsTbl->bWrRight != FALSE))) {
                        /* Increase number of learning iteration during this
                         * standstill */
                        pOffsTbl->LearnCnt++;
                    }
                }
            } /* End of new input filter value */
        }     /* End of learn count */
        else {
            /* Maximum number of learning reached, if during the standstill
               temperature deviation occurs, re-enable learning */

            /* Discard pending learning values */
            pOffsTbl->bWrLeft = FALSE;
            pOffsTbl->bWrRight = FALSE;

            /* Flush pending nvm write updates */
            VED_YwrtCompletePendingWrites(proPorts->pYwrtTempTable);

            /* Input filter hold enough samples for evalution  */
            if (pOffsTbl->SmpYwRt.Volume > (fir_len - C_F32_DELTA)) {
                /* Calculate input filter output */
                VED_StatIntervalMeanDev(&pOffsTbl->SmpYwRt);
                VED_StatIntervalMeanDev(&pOffsTbl->SmpTemp);

                /* If temperature deviation to last learned offset value,
                 * re-enable learning*/
                if (fABS(pOffsTbl->LastOffsTemp - pOffsTbl->SmpTemp.Mean) >
                    5.0F) {
                    pOffsTbl->LearnCnt = (uint16)0;
                }

                /* Reset input filter data */
                VED_StatIntervalInit(&pOffsTbl->SmpYwRt);
                VED_StatIntervalInit(&pOffsTbl->SmpTemp);
            }
        }
    } /* End of observability */
    else {
        /* Yaw rate zero point offset is not observable */

        /* Discard pending learning values */
        pOffsTbl->bWrLeft = FALSE;
        pOffsTbl->bWrRight = FALSE;

        /* At drive-off finalize learning */
        if (pSen->StandstillOK == FALSE) {
            /* reset standstill learning counter */
            pOffsTbl->LearnCnt = (uint16)0;

            /* Flush pending nvm write updates */
            VED_YwrtCompletePendingWrites(proPorts->pYwrtTempTable);

            /* Correct missing interpolated bybass values, which could be caused
             * by power interuption */

            if ((pOffsTbl->Checked == FALSE) &&
                !VED__YWRT_NODES_ALL_VALID(proPorts->pYwrtTempTable->statRd)) {
                pOffsTbl->Checked = TRUE;
                VED_YwrtCorrectTable(proPorts->pYwrtTempTable);
            }
        }

        /* Reset input filter for next standstill */
        VED_StatIntervalInit(&pOffsTbl->SmpYwRt);
        VED_StatIntervalInit(&pOffsTbl->SmpTemp);
    }

    /* Calculate current yaw rate offset based on offset table data */
    VED_YwrtCalcOffsTblCurr(proPorts->pYwrtTempTable, pSen->Tempr, pOffsTbl);

    return;
}

/* ***********************************************************************
  @fn               VED_YwrtLearnValuePresent */ /*!
  @brief            Test if any learn value is present

  @description      see brief description

  @param[in]        yaw rate temperature table
  @param[out]       -
  @return           flag: TRUE: learn value present, FALSE: no learn value present

  @pre              -
  @post             -
**************************************************************************** */
static bool_t VED_YwrtLearnValuePresent(
    const VED_NvYwrtLearnTable_t *pYwrTmpTbl) {
    ui32_t ii;
    ui32_t Sum = 0U;

    /* peruse all learn values */
    for (ii = 0U; (ii < VED__YWRT_NO_NODES) && (Sum == 0U); ii++) {
        Sum += (ui32_t)pYwrTmpTbl->Node[ii].Conf;
    }

    /* return TRUE if at least one learn value is present */

    return ((bool_t)(Sum > 0U));
}

/* ***********************************************************************
  @fn               VED_YwrtGetTableNode */ /*!
  @brief            Read correction table node and convert to floating point

  @description      see brief description

  @param[in]        yaw rate temperature table
  @param[out]       learning node
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrtGetTableNode(const VED_NvYwrtLearnTable_t *tbl,
                                 VED_YwrtLearnNode_t *ndInt) {
    ndInt->Offset = VED__YWRT_NVM_YWR_FLP(tbl->Node[ndInt->No].Offset);
    ndInt->Conf = VED__YWRT_NVM_CONF_FLP(tbl->Node[ndInt->No].Conf);

    return;
}

/* ***********************************************************************
  @fn               VED_YwrtSetTableNode */ /*!
  @brief            Convert to fixed point and write correction table node 

  @description      see brief description

  @param[in]        learning node
  @param[out]       yaw rate temperature table
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrtSetTableNode(VED_NvYwrtLearnTable_t *tbl,
                                 const VED_YwrtLearnNode_t *ndInt) {
    tbl->Node[ndInt->No].Offset = VED__YWRT_NVM_YWR_FXP(ndInt->Offset);

    tbl->Node[ndInt->No].Conf = VED__YWRT_NVM_CONF_FXP(ndInt->Conf);

    return;
}

/* ***********************************************************************
  @fn               VED_YwrtCompletePendingWrites */ /*!
  @brief            Flush pending nvm memory writes

  @description      see brief description

  @param[in,out]    yaw rate learn table
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_YwrtCompletePendingWrites(VED_NvYwrtLearnTable_t *tbl) {
    /* Enable write access of pending update values */

    tbl->statWr[0] |= (ui32_t)(tbl->statWrpd[0] & VED__YWRT_NODES_MAKS_ALL_LO);
    tbl->statWr[1] |= (ui32_t)(tbl->statWrpd[1] & VED__YWRT_NODES_MAKS_ALL_HI);

    /* Clear pending writes */

    VED__YWRT_NODES_CLEAR_ALL(tbl->statWrpd);

    return;
}

/*****************************************************************************
  @fn             VED_YwrtInterpTable
  @brief          Calculation of linear interpolated table values

  @description    see brief description

  @param[in]      xi
  @param[in]      y_out
  @param[in]      len
  @param[out]     node
  @return         void

  @pre            -
  @post           -
*****************************************************************************/
static void VED_YwrtInterpTable(const VED_NvYwrtLearnNode_t *node,
                                const ui8_t *xi,
                                i16_t *y_out,
                                ui32_t len) {
    ui32_t ni = 0U;
    i16_t ii;

    /* run through all table nodes */
    for (ii = 0; ii < (i16_t)VED__YWRT_NO_NODES; ii++) {
        f32_t y;

        /* Look for first reference node xi above current table node */
        while ((ni < (len - 1U)) && ((i16_t)xi[ni] < ii)) {
            ni++;
        }

        /* test if current table node is inside reference node range */
        if (ni > 0U) {
            i16_t x2;
            i16_t x1;

            f32_t y2;
            f32_t y1;

            /* Take actual table position and previous table position for
             * interpolation */
            x1 = (i16_t)xi[ni - 1U];
            x2 = (i16_t)xi[ni];

            /* Get reference nodes values */
            y1 = (f32_t)node[x1].Offset;
            y2 = (f32_t)node[x2].Offset;

            /* reference must be distinct for interpolation */
            if (x1 < x2) {
                if (ii < x2) {
                    /* table node is between reference node */

                    y = (((f32_t)(ii - x1) * (y2 - y1)) / (f32_t)(x2 - x1)) +
                        y1;
                } else {
                    /* table node is on or beyond reference node */
                    y = y2;
                }
            } else {
                /* reference nodes indices are identical, interpolation not
                   possible, use first node value */
                y = y1;
            }
        } else {
            /* current table node outside reference node range, use first
               reference node as output value */
            y = (f32_t)(node[xi[0]].Offset);
        }
        /* save result as fixed-point value */

        y_out[ii] = (i16_t)ROUND_TO_INT(y);
    }
    return;
}

/*****************************************************************************
  @fn             VED_YwrtRepairTable
  @brief          Calculation of linear interpolated table values

  @description    see brief description


  @param[in]      pointer to table
  @param[out]     pointer to table
  @return         void

  @pre            -
  @post           -
*****************************************************************************/
static void VED_YwrtCorrectTable(VED_NvYwrtLearnTable_t *pYwrTmpTbl) {
    ui32_t idx;
    ui8_t lrn_nds[VED__YWRT_NO_NODES];

    ui32_t len_nds = 0UL;
    VED_NvYwrtLearnNode_t *tab = pYwrTmpTbl->Node;

    /* Look for learned table nodes indices */
    for (idx = 0UL; idx < VED__YWRT_NO_NODES; idx++) {
        lrn_nds[idx] = (ui8_t)VED__YWRT_NO_NODES;
        if (tab[idx].Conf > VED__YWRT_CONF_RES_FXP) {
            lrn_nds[len_nds++] = (ui8_t)idx;
        }
    }

    /* If at least one learned node was found, interpolate values */
    if (len_nds > 0U) {
        i16_t offs[VED__YWRT_NO_NODES];

        /* Interpolate and extrapolate learned values to whole table */
        VED_YwrtInterpTable(tab, lrn_nds, offs, len_nds);

        /* Run through all nodes and correct nodes if necessary */
        for (idx = 0UL; idx < VED__YWRT_NO_NODES; idx++) {
            i16_t diff = offs[idx] - tab[idx].Offset;

            /* Correct with low confidence and significant devation to
             * interpolated value */

            if ((tab[idx].Conf <= VED__YWRT_CONF_RES_FXP) &&
                (ABS(diff) > VED__YWRT_OFFS_THRD_FXP)) {
                /* Update offset table values */
                tab[idx].Offset = offs[idx];
                tab[idx].Conf = VED__YWRT_CONF_RES_FXP;

                VED__YWRT_NODE_SET_VALID(idx, pYwrTmpTbl->statWrpd);
            }
        }
    }
    return;
}

/* ***********************************************************************
  @fn               VED_YwrtGetYawRateOffset */ /*!
  @brief            Hand over leanred offset and confidence value

  @description      see brief description

  @param[out]       pointer to offset table
  @param[out]       pointer to confidence table
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_YwrtGetYawRateOffset(float32 *offset, float32 *conf) {
    const VED_YwrtOffsTblData_t *pOffsTbl = &(YWRT_GET_OFFS->Tbl);

    *offset = pOffsTbl->Offset;
    *conf = pOffsTbl->Conf;

    return;
}

/* ***********************************************************************
  @fn                     GetOffsData */ /*!
  @brief                  Access to offset data

  @description            see brief description

  @param[in]              -
  @param[out]             -
  @return                 *VED_YwrOffsData_t

  @pre                    -
  @post                   -
**************************************************************************** */
const VED_YwrtOffsData_t *VED_YwrtGetOffsData(void) { return (YWRT_GET_OFFS); }

/* ***********************************************************************
  @fn                     VED_YwrtGetPrivateData */ /*!
  @brief                  Get internal yaw rate sensor data

  @description            see brief description

  @param[in]              -
  @param[out]             -
  @return                 *VED_YwrtData_t

  @pre                    -
  @post                   -
**************************************************************************** */
VED_YwrtData_t *VED_YwrtGetPrivateData(void);
VED_YwrtData_t *VED_YwrtGetPrivateData(void) { return (YWRT_GET_ME); }
#endif
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */