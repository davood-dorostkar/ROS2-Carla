/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <ved_consts.h>
#include "ved.h"
#include "tue_common_libs.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/* SWA offset range values */
#define SWA_ANG_OFFS_DEFAULT \
    (float32)(0.0F) /*! Default value for uncalibrated swa offset */
#define SWA_OFFSET_HIST_WIDTH                                                 \
    (float32) DEG2RAD(1.95F) /*! Bin width for steering wheel angle histogram \
                              */
#define SWA_OFFS_STAT_DIFF_RUN_UP                                             \
    ((float32)DEG2RAD(1.5F)) /*! Maximum absolute deviation at run-up to come \
                                to next higher offset state */
#define SWA_OFFS_CHANGE_HYST \
    (float32) DEG2RAD(0.5F) /*! Hysteresis at learned offset for restoring */
#define SWA_OFFS_NORM_RES \
    (float32) DEG2RAD(0.5F) /*! Scaling of steering wheel angle offset */
#define SWA_OFFS_DEV_NORMAL                          \
    (float32)(0.0F) /*! Standard deviation NVM value \
                     */

#define SWA_INTERVAL_RDCT_FCTR \
    (float32)(0.5F) /*! Reduction factor histogram counts */
#define SWA_OFFS_DRIVEN_DIST_MAX \
    (float32)(                   \
        500 *                    \
        1000) /*! Maximum accumulated distance in swa and ay histograms */

/*! Steering wheel angle offset states */
#define SWA_STATE_DEFAULT \
    (sint32)0 /*!< SWA offset state 0 non existent                  */
#define SWA_STATE_1 \
    (sint32)1 /*!< SWA offset state 1 run-up                        */
#define SWA_STATE_2 \
    (sint32)2 /*!< SWA offset state 2 run-up                        */
#define SWA_STATE_3 \
    (sint32)3 /*!< SWA offset state 3 run-up                        */
#define SWA_STATE_4 \
    (sint32)4 /*!< SWA offset state 4 learned but not confirmed yet */
#define SWA_STATE_5 \
    (sint32)5 /*!< SWA offset state 5 learned and confirmed once    */
#define SWA_STATE_6 \
    (sint32)6 /*!< SWA offset state 6 learned and confirmed twice   */

/*! Distance thresholds for offset estimation in dependence of offset state */
#ifndef SWA_DIST_SAMP_STAT_DEFAULT
#define SWA_DIST_SAMP_STAT_DEFAULT \
    (float32)20000.F /*!< Sampled distance for estimation at state 0 */
#endif
#ifndef SWA_DIST_SAMP_STAT_1
#define SWA_DIST_SAMP_STAT_1 \
    (float32)25000.F /*!< Sampled distance for estimation at state 1 */
#endif
#ifndef SWA_DIST_SAMP_STAT_2
#define SWA_DIST_SAMP_STAT_2 \
    (float32)30000.F /*!< Sampled distance for estimation at state 2 */
#endif
#ifndef SWA_DIST_SAMP_STAT_3
#define SWA_DIST_SAMP_STAT_3 \
    (float32)35000.F /*!< Sampled distance for estimation at state 3 */
#endif
#ifndef SWA_DIST_SAMP_STAT_4
#define SWA_DIST_SAMP_STAT_4 \
    (float32)40000.F /*!< Sampled distance for estimation at state 4 */
#endif
#ifndef SWA_DIST_SAMP_STAT_5
#define SWA_DIST_SAMP_STAT_5 \
    (float32)40000.F /*!< Sampled distance for estimation at state 5 */
#endif
#ifndef SWA_DIST_SAMP_STAT_6
#define SWA_DIST_SAMP_STAT_6 \
    (float32)50000.F /*!< Sampled distance for estimation at state 6 */
#endif

/*! Steering wheel angle offset standard deviations */
#define SWA_STATE_0_DEV \
    (float32)           \
        DEG2RAD(12.0F / 3.0F) /*!< SWA offset standard deviation state 0  */
#define SWA_STATE_1_DEV \
    (float32)           \
        DEG2RAD(4.0F / 3.0F) /*!< SWA offset standard deviation state 1  */
#define SWA_STATE_2_DEV \
    (float32)           \
        DEG2RAD(3.0F / 3.0F) /*!< SWA offset standard deviation state 2  */
#define SWA_STATE_3_DEV \
    (float32)           \
        DEG2RAD(2.0F / 3.0F) /*!< SWA offset standard deviation state 3  */
#define SWA_STATE_4_DEV \
    (float32)           \
        DEG2RAD(1.0F / 3.0F) /*!< SWA offset standard deviation state 4  */
#define SWA_STATE_5_DEV \
    (float32)           \
        DEG2RAD(0.5F / 3.0F) /*!< SWA offset standard deviation state 5  */
#define SWA_STATE_6_DEV \
    (float32)           \
        DEG2RAD(0.5F / 3.0F) /*!< SWA offset standard deviation state 6  */

/*! Standard deviations for steering wheel angle offset estimation */
#define SWA_DEV_HIST_STAT_DEFAULT \
    (float32)                     \
        DEG2RAD(1.0F) /*!< Default standard deviation for offset estimation */
#define SWA_DEV_HIST_STAT_STARTUP                                         \
    (float32) DEG2RAD(1.0F) /*!< Standard deviation for offset estimation \
                               during startup phase      */
#define SWA_DEV_HIST_STAT_NORMAL                                          \
    (float32) DEG2RAD(0.5F) /*!< Standard deviation for offset estimation \
                               during confirmation phase */

/*! Confidence levels for estimated steering wheel angle offsets */
#define AY_SWA_OFFS_MAX_CONF (float32)1.0F /*!< Maximum confidence level */
#define AY_SWA_OFFS_MID_CONF (float32)0.5F /*!< Medium  confidence level */
#define AY_SWA_OFFS_LOW_CONF (float32)0.2F /*!< Low confidence level     */
#define AY_SWA_OFFS_NO_CONF (float32)0.0F  /*!< No confidence level      */

/*! Thesholds for offset takeover */
#define SWA_OFFS_DIFF_MAX_MEAN \
    (DEG2RAD(1.0F)) /*!< Range gap between maximum and mean */
#define SWA_OFFS_DIFF_MEAN_CT_MED \
    (DEG2RAD(1.5F)) /*!< Range gap between central mean for median usage */
#define SWA_OFFS_DIFF_MEAN_MED \
    (DEG2RAD(1.0F)) /*!< Range gap between mean and median for median usage */
#define SWA_OFFS_DIFF_MAX_MED \
    (DEG2RAD(                 \
        2.5F)) /*!< Range gap between maximum and median for median usage */
#define SWA_OFFS_FTHR_DIST_MAX_ADJ \
    (0.5F) /*!< Counted distance gap between maximum and neighbour bins */
#define SWA_OFFS_FTHR_DIST_MAX_OUT \
    (0.2F) /*!< Counted distance gap between maximum and secondary bin */
#define SWA_OFFS_TAKE_OVR_MAX \
    (10U) /*!< Maximum number of takeovers during one ignition cycle */
#define SWA_OFFS_FTHR_DIST_MAX_MEAN \
    (0.1F) /*!< Counted distance gap between maximum and mean for median */

/*! Limits for steering wheel angle offset estimation */
#define SWA_OFFS_VEL_MIN                                                  \
    ((float32)10.F) /*!< Minimum veloctiy for steering wheel angle offset \
                       calibration       */
#define SWA_OFFS_ANGLE_MAX                                                 \
    ((float32)DEG2RAD(30.F)) /*!< Maximum allowed steering wheel angle for \
                                offset estimation         */
#ifndef SWA_OFFS_ANGLE_FAST_OFF_MAX
#define SWA_OFFS_ANGLE_FAST_OFF_MAX                                        \
    ((float32)DEG2RAD(30.F)) /*!< Maximum allowed steering wheel angle for \
                                fast offset learning estimation     */
#endif
#define SWA_OFFS_LAT_ACC_MAX                                              \
    ((float32)1.8F) /*!< Maximum allowed averaged lateral accleration for \
                       offset estimation */
#define SWA_OFFS_LAT_ACC_FT                                                \
    ((float32)60.F) /*!< Lateral acceleration average filter time constant \
                     */
#define SWA_OFFS_WHS_DIFF_FT \
    ((float32)6.F) /*!< Wheel velocity difference filter time constant */
#define SWA_OFFS_WHS_DIFF_VEL                                                  \
    ((float32)0.3F) /*!< Maximum allowed wheel velocity differences for offset \
                       estimation   */

/*! Limits for fast steering wheel angle offset estimation */
#ifndef SWA_FAST_OFFS_WHS_DIFF_VEL
#define SWA_FAST_OFFS_WHS_DIFF_VEL                                           \
    ((float32)0.6F) /*!< Maximum allowed wheel velocity differences for fast \
                       offset estimation      */
#endif
#ifndef SWA_FAST_OFFS_LONG_ACC_MAX
#define SWA_FAST_OFFS_LONG_ACC_MAX                                          \
    ((float32)0.5F) /*!< Maximum allowed longitudinal acceleration for fast \
                       offset estimation       */
#endif
#define SWA_FAST_OFFS_VEL_MIN                                                  \
    ((float32)5.0F) /*!< Minimum veloctiy for fast steering wheel angle offset \
                       extimation           */
#define SWA_FAST_OFFS_REDUCTION_FACT \
    (10) /*!< Reduction factor for long term data */
#define SWA_FAST_OFFS_SAMPLE_LIMIT                                          \
    ((uint16)250U) /*!< Number of samples used for mean calculation in fast \
                      offset estimation      */
#define SWA_FAST_OFFS_SAMPLE_LIMIT_LT                                       \
    ((uint16)500U) /*!< Number of samples used for long term calculation in \
                      fast offset estimation */
#define SWA_FAST_OFFS_NVM_LIMIT                                              \
    ((uint16)3) /*!< Number of times the fast offset estimation is stored in \
                   NVM                */

/*! Lateral acceleration sensor offset values */
#define AY_OFFS_DEFAULT \
    ((float32)0.0F) /*! Default value for uncalibrated swa offset */
#define AY_OFFSET_LIMIT_MAX \
    ((float32)1.2F) /*! Range(+/-) where offset correction is conducted */
#define AY_SWA_LIMIT_HIST                                                      \
    ((float32)5.0F) /*! Range(+/-) where swa offset data is added to histogram \
                     */

/*! Lateral acceleration sensor offset states */
#define AY_STATE_DEFAULT \
    ((sint32)0)                /*!< Ay offset state 0 non existent       */
#define AY_STATE_1 ((sint32)1) /*!< Ay offset state 1 run-up             */
#define AY_STATE_2 ((sint32)2) /*!< Ay offset state 2 run-up             */
#define AY_STATE_3 ((sint32)3) /*!< Ay offset state 3 learn and relearn  */

/*! Distance thresholds for lateral acceleration offset estimation in depedence
 * of offset state */
#define AY_DIST_SAMP_STAT_DEFAULT \
    ((float32)20000.F) /*!< Sampled distance for estimation at state 0 */
#define AY_DIST_SAMP_STAT_1 \
    ((float32)25000.F) /*!< Sampled distance for estimation at state 1 */
#define AY_DIST_SAMP_STAT_2 \
    ((float32)30000.F) /*!< Sampled distance for estimation at state 2 */
#define AY_DIST_SAMP_STAT_3 \
    ((float32)30000.F) /*!< Sampled distance for estimation at state 3 */

/*! Lateral acceleration offset standard deviations */
#define AY_STATE_0_DEV \
    (float32)(         \
        1.0F /         \
        3.0F) /*!< Lateral accleration offset standard deviation state 0 */
#define AY_STATE_1_DEV \
    (float32)(         \
        0.3F /         \
        3.0F) /*!< Lateral accleration offset standard deviation state 1 */
#define AY_STATE_2_DEV \
    (float32)(         \
        0.1F /         \
        3.0F) /*!< Lateral accleration offset standard deviation state 2 */
#define AY_STATE_3_DEV \
    (float32)(         \
        0.07F /        \
        3.0F) /*!< Lateral accleration offset standard deviation state 3 */

/*! Standard deviations for lateral acceleration offset estimation */
#define AY_DEV_SAMP_STAT_DEFAULT \
    (float32)(0.5F) /* Default standard deviation for offset estimation */
#define AY_DEV_SAMP_STAT_STARTUP                                               \
    (float32)(0.5F) /* Standard deviation for offset estimation during startup \
                       phase      */
#define AY_DEV_SAMP_STAT_NORMAL                                        \
    (float32)(0.2F) /* Standard deviation for offset estimation during \
                       confirmation phase */

#define AY_OFFS_STAT_DIFF_RUN_UP                                         \
    ((float32)(0.7F)) /* Maximum allowed difference for increasing state \
                         during learning phase */
#define AY_OFFS_STAT_DIFF_LEARNED                                             \
    (float32)(0.5F) /* Maximum allowed difference for increasing state during \
                       confirmation phase */
#define AY_OFFS_TAKE_OVR_MAX \
    (3U) /* Maximum number of takeovers during one ignition cycle */
#define AY_OFFS_DEV_NORMAL (float32)(0.0F) /* Standard deviation NVM value */
#define AY_OFFS_DIFF_MAX_MEAN \
    (float32)(0.2F) /* Range gap between maximum and mean */
#define AY_OFFS_DIFF_MAX_MED                                                 \
    (float32)(0.3F) /* Range gap between maximum and median for median usage \
                     */
#define AY_OFFS_DIFF_MEAN_MED \
    (float32)(0.2F) /* Range gap between maximum and mean for median usage */
#define AY_OFFS_FTHR_DIST_MAX_ADJ \
    (0.5F) /* Counted distance gap between maximum and neighbour bins */
#define AY_OFFS_FTHR_DIST_MAX_OUT \
    (0.2F) /* Counted distance gap between maximum and secondary bin */

#define AY_INTERVAL_RDCT_FCTR \
    (float32)(0.4F) /* Reduzierungsfaktor Sampleinterval                    */
#define AY_OFFSET_HIST_WIDTH                                                 \
    (float32)(0.25F) /* Bin width for lateral acceleration offset estimation \
                      */
#define SWA_NVM_CLEARED (uint32)(0xFFFFFFFFU)

/*****************************************************************************
  MACROS
*****************************************************************************/
#define SWA_GET_ME (&VED_SwaGlobData)
#define SWA_GET_INP_SIGNALS (&VED_SwaGlobData.Io.in->Signals)
#define SWA_GET_MIF_DATA (VED_SwaGlobData.Io.mif)
#define SWA_GET_ERR_OFFS_RG (*VED_SwaGlobData.Io.errOffsRg)
#define SWA_GET_NVM_ERR_OFFS_RG (*VED_SwaGlobData.Io.NVMerrOffsRg)

#define SWA_GET_SEN_DATA (&VED_SwaGlobData.Sensor)
#define SWA_GET_OFF_DATA (&VED_SwaGlobData.Offset)
#define SWA_GET_NV_DATA (&VED_SwaGlobData.NVValue)

#define AY_GET_ME (&VED_AyGlobData)
#define AY_GET_INP_SIGNALS (&VED_AyGlobData.Io.in->Signals)
#define AY_GET_NV_READ_DATA (VED_AyGlobData.Io.nv_read)
#define AY_GET_NV_WRITE_DATA (VED_AyGlobData.Io.nv_write)

#define AY_GET_SEN_DATA (&VED_AyGlobData.Sensor)
#define AY_GET_OFF_DATA (&VED_AyGlobData.Offset)

#define AY_GET_ERR_OFFS_RG (*VED_AyGlobData.Io.errOffsRg)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  CONSTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(VED_SwaGlobData)
static VED_SwaData_t
    VED_SwaGlobData; /*!< @VADDR: 0x20013000 @VNAME: VED_Swa @ALLOW: ved__priv
                        @cycleid: ved__cycle_id*/

SET_MEMSEC_VAR(VED_AyGlobData)
static VED_AyData_t
    VED_AyGlobData; /*!< @VADDR: 0x20016000 @VNAME: VED_Ay @ALLOW: ved__priv
                       @cycleid: ved__cycle_id*/
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  LOCAL FUNCTION PROTOTYPES
*****************************************************************************/
static void VED_SwaNvmReadOffset(VED_SwaOffsEEprom_t *NVValue,
                                 const VEDNvStWhlAngCalc_t *nvin,
                                 uint32 status);
static uint32 VED_SwaNvmWriteOffset(VED_SwaOffsEEprom_t *NVValue,
                                    VEDNvStWhlAngCalc_t *nvout);
static void VED_SwaTakeoverOffset(VED_SwaOffsData_t *StrgOffsData,
                                  VED_SwaOffsEst_t *OffsEst);
static void VED_SwaInitVolatile(VED_SwaData_t *pSwaData);
static void VED_SwaInitOffsEepromMirror(VED_SwaOffsEEprom_t *NVValue);
static void VED_SwaInitSenData(VED_SwaSenData_t *pSwaSen);
static void VED_SwaInitOffsData(VED_SwaOffsData_t *SwaOffs);
static void VED_SwaInitOffsEst(VED_SwaOffsEst_t *OffsEst);
static float32 VED_SwaEvalThrdDist(sint32 Status);
static float32 VED_SwaEvalThrdDeviation(sint32 Status);
static void VED_SwaPutEepromOffsetData(float32 Offset,
                                       sint32 Status,
                                       float32 Dev);
static void VED_SwaInitOffset(VED_SwaOffsData_t *Offset,
                              VED_SwaOffsEEprom_t *NVValue);
static void VED_SwaGetOffsetStart(VED_SwaOffsData_t *SwaOffsData);
static boolean VED_SwaCheckConfidence(const VED_SwaOffsData_t *OffsData);
static void VED_SwaOffsCheckandTakeOver(VED_SwaOffsData_t *swaOffs);
static boolean VED_SwaGetOffset(float32 *Offset, sint32 *Status, float32 *Dev);
static void VED_SwaCalcConfidence(VED_SwaOffsData_t *StrgOffsData);
static boolean VED_SwaOffsetRangeOk(float32 Offset);
static float32 VED_SwaGetOffsetDeviation(sint32 OffsState);

static void VED_AyGetOffsetStart(VED_AyOffsData_t *pAyOffsData);
static void VED_AyInitVolatile(VED_AyData_t *pAyData);
static void VED_AyInitOffset(VED_AyOffsData_t *pAyOffset);
static void VED_AyOffsCheckandTakeOver(VED_AyOffsData_t *payOffs,
                                       const VED_SwaOffsData_t *pswaOffs,
                                       boolean clrData);
static void VED_AyTakeoverOffset(VED_AyOffsData_t *offsAy,
                                 VED_AyOffsEst_t *estOffsAy);
static void VED_AyInitSenData(VED_AySenData_t *pAySen);
static void VED_AyInitOffsData(VED_AyOffsData_t *pAyOffs);
static void VED_AyInitOffsEst(VED_AyOffsEst_t *pAyOffsEst);
static float32 VED_AyEvalThrdDist(sint32 Status);
static float32 VED_AyEvalThrdDeviation(sint32 Status);
static boolean VED_AyOffsetRangeOk(float32 Offset);
static boolean VED_AyGetNvmOffsetData(float32 *Offset,
                                      sint32 *Status,
                                      float32 *Dev);
static boolean VED_AyCheckConfidence(const VED_AyOffsData_t *ayOffs);
static void VED_AyCalcConfidence(VED_AyOffsData_t *pOffsAy);
static boolean VED_AyGetOffset(float32 *Offset, sint32 *Status, float32 *Dev);
static void VED_AyPutNvmOffsetData(float32 Offset, sint32 Status);

static void VED_AySwaCalcOffset(const VED_SwaSenData_t *swaSen,
                                const VED_AySenData_t *aySen,
                                VED_SwaOffsData_t *swaOffs,
                                VED_AyOffsData_t *ayOffs);
static void VED_AySwaAcqEstData(const VED_SwaSenData_t *swaSen,
                                const VED_AySenData_t *aySen,
                                VED_SwaOffsData_t *swaOffs,
                                VED_AyOffsData_t *ayOffs,
                                float32 cycDist);
static float32 VED_AyGetOffsetDeviation(sint32 OffsState);

VED_SwaData_t *VED_SwaGetPrivateData(void);
VED_AyData_t *VED_AyGetPrivateData(void);

/* **********************************************************************
  @fn               VED_AySwaExec */ /*!

  @brief            Determine operating sequence for vehicle dynamics observer
  @description       

  @param[in]        reqPorts
  @param[in]        input
  @param[in]        mif
  @param[in]        proPorts
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_AySwaExec(const reqVEDPrtList_t *reqPorts,
                   const VED_InputData_t *input,
                   VED_ModIf_t *mif,
                   const proVEDPrtList_t *proPorts) {
    VED_SwaSenData_t *pSwaSen = SWA_GET_SEN_DATA;
    VED_SwaOffsData_t *pSwaOffs = SWA_GET_OFF_DATA;
    VED_SwaData_t *pSwaData = SWA_GET_ME;

    VED_AySenData_t *pAySen = AY_GET_SEN_DATA;
    VED_AyOffsData_t *pAyOffs = AY_GET_OFF_DATA;
    VED_AyData_t *pAyData = AY_GET_ME;

    pSwaData->Io.in = input;
    pSwaData->Io.mif = mif;
    pSwaData->Io.NVMerrOffsRg =
        &proPorts->pVED_Errors->OutPutErrors.NVMSwaOffsRg;
    proPorts->pVED_Errors->OutPutErrors.NVMSwaOffsRg = VED_ERR_STATE_UNKNOWN;

    pSwaData->Io.errOffsRg = &proPorts->pVED_Errors->OutPutErrors.SwaOffsRg;
    proPorts->pVED_Errors->OutPutErrors.SwaOffsRg = VED_ERR_STATE_UNKNOWN;

    pAyData->Io.in = input;
    pAyData->Io.mif = mif;
    pAyData->Io.nv_read = reqPorts->pNVMRead;
    pAyData->Io.nv_write = proPorts->pNVMWrite;
    /* Note: if online out of range error should also detect for lateral
       acceleration rename the errOffsRg to NVMerrOffsRg similar to the swa
       errors */
    pAyData->Io.errOffsRg = &proPorts->pVED_Errors->OutPutErrors.NVMAyOffsRg;
    proPorts->pVED_Errors->OutPutErrors.NVMAyOffsRg = VED_ERR_STATE_UNKNOWN;

    /* Distinction between different operating states */
    if (VED__CTRL_GET_STATE((uint8)VED_CTRL_STATE_RUNNING,
                            input->Frame.CtrlMode)) {
/*<--- Execution path for normal operating mode --->*/

/* Upload offset form nv memory to internal nvm offset buffer */
#if ((!defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) || \
     (!CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
        if (VED__GET_NVM_IO_STATE(VED_NVM_POS_SWA,
                                  &reqPorts->pNVMRead->State) ==
            VED_IO_STATE_VALID)
#endif
        {
            VED_SwaNvmReadOffset(&pSwaData->NVValue,
                                 &reqPorts->pNVMRead->StWhlAng,
                                 reqPorts->pNVMRead->State);
        }

        /* Read steering wheel angle signal */
        pSwaSen->StrgAngle = SWA_GET_INP_SIGNALS->StWheelAngle;
        pSwaSen->Valid =
            (boolean)((VED_GET_IO_STATE(VED_SIN_POS_SWA,
                                        SWA_GET_INP_SIGNALS->State) ==
                       VED_IO_STATE_VALID)
                          ? TRUE
                          : FALSE);

        /* Read lateral acceleration sensor signal */
        pAySen->Ay = AY_GET_INP_SIGNALS->LatAccel;
        pAySen->Valid =
            (boolean)((VED_GET_IO_STATE(VED_SIN_POS_LATA,
                                        AY_GET_INP_SIGNALS->State) ==
                       VED_IO_STATE_VALID)
                          ? TRUE
                          : FALSE);

        /* Get non volatile start values */
        VED_SwaGetOffsetStart(pSwaOffs);
        VED_AyGetOffsetStart(pAyOffs);

        /* Check validity of lateral acceleration sensor */
        if (pAySen->Valid != FALSE) {
            /* Calculate gradient */
            pAySen->Gradient = VED_CalcGradient(pAySen->Ay, pAySen->AyOld);
            pAySen->AyOld = pAySen->Ay;
        }

        /* Check validity of steering wheel angle sensor */
        if (pSwaSen->Valid != FALSE) {
            /* Calculate gradient */
            pSwaSen->Gradient = VED_CalcGradient(
                (float32)pSwaSen->StrgAngle, (float32)pSwaSen->StrgAngleOld);
            pSwaSen->StrgAngleOld = pSwaSen->StrgAngle;

            /* Start calculation of steering wheel angle and lateral
             * acceleration sensor offset */
            VED_AySwaCalcOffset(pSwaSen, pAySen, pSwaOffs, pAyOffs);

            /* Export to module interface */
            mif->SwaOffset.offset = pSwaOffs->StrgOffset;
            mif->SwaOffset.state = (uint8)pSwaOffs->OffsState;
            mif->SwaOffset.var = SQR(pSwaOffs->Dev);

            mif->AyOffset.offset = pAyOffs->AyOffset;
            mif->AyOffset.state = (uint8)pAyOffs->OffsState;
            mif->AyOffset.var = SQR(pAyOffs->Dev);
        }

        {
            /* If necessary, write new offset to non volatile memory */
            uint32 ioState;

            /* Test if write access is required and copy data */
            ioState = VED_SwaNvmWriteOffset(&pSwaData->NVValue,
                                            &proPorts->pNVMWrite->StWhlAng);

            /* Set non-volatile */

            VED_SET_NVM_IO_STATE(VED_NVM_POS_SWA, ioState,
                                 &proPorts->pNVMWrite->State);
        }

    } else {
        /*<--- Execution path for initialization mode  --->*/

        VED_SwaInitVolatile(pSwaData);
        VED_AyInitVolatile(pAyData);
    }

    return;
}

/* **********************************************************************
  @fn               VED_AySwaInit */ /*!

  @brief            initialize module data

  @description       
  @param[in]        -
  @param[in]        -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
void VED_AySwaInit(const reqVEDPrtList_t *reqPorts,
                   const proVEDPrtList_t *proPorts)

{
    VED_SwaData_t *pSwaData = SWA_GET_ME;
    VED_AyData_t *pAyData = AY_GET_ME;

    /* Initialzation of steering wheel angle data */
    VED_SwaInitVolatile(pSwaData);
    VED_SwaInitOffset(&pSwaData->Offset, &pSwaData->NVValue);

    /* Initialzation of lateral acceleration data */
    VED_AyInitVolatile(pAyData);
    VED_AyInitOffset(&pAyData->Offset);

    (void)proPorts; /* remove compiler warning, proPorts is not used in this
                       configuration */
    (void)reqPorts; /* remove compiler warning, reqPorts is not used in this
                       configuration */

    return;
}

/* **********************************************************************
  @fn               VED_SwaGetOffsetDeviation */ /*!

  @brief            Assign offset standard deviation for offset state

  @description       
  @param[in]        OffsState
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static float32 VED_SwaGetOffsetDeviation(sint32 OffsState) {
    float32 ret;

    switch (OffsState) {
        case SWA_STATE_1:
            ret = SWA_STATE_1_DEV;
            break;

        case SWA_STATE_2:
            ret = SWA_STATE_2_DEV;
            break;

        case SWA_STATE_3:
            ret = SWA_STATE_3_DEV;
            break;

        case SWA_STATE_4:
            ret = SWA_STATE_4_DEV;
            break;

        case SWA_STATE_5:
            ret = SWA_STATE_5_DEV;
            break;

        case SWA_STATE_6:
            ret = SWA_STATE_6_DEV;
            break;

        case SWA_STATE_DEFAULT:
        default:
            ret = SWA_STATE_0_DEV;
            break;
    }
    return ret;
}

/* **********************************************************************
  @fn               VED_AyGetOffsetDeviation */ /*!

  @brief            Assign offset standard deviation for offset state

  @description       
  @param[in]        OffsState
  @param[out]       -
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static float32 VED_AyGetOffsetDeviation(sint32 OffsState) {
    float32 ret;

    switch (OffsState) {
        case AY_STATE_1:
            ret = AY_STATE_1_DEV;
            break;

        case AY_STATE_2:
            ret = AY_STATE_2_DEV;
            break;

        case AY_STATE_3:
            ret = AY_STATE_3_DEV;
            break;

        case AY_STATE_DEFAULT:
        default:
            ret = AY_STATE_0_DEV;
            break;
    }
    return ret;
}

/* **********************************************************************
  @fn                     VED_SwaGetPrivateData */ /*!
  @brief                  Access to internal data

  @description            Allows accessing the steering wheel data from outside
                          of this module

  @param[in]              - 
  @param[out]             -
  @return                 *VED_SwaSenData_t
  
  @pre                    -
  @post                   -
**************************************************************************** */
VED_SwaData_t *VED_SwaGetPrivateData(void) { return (SWA_GET_ME); }

/* **********************************************************************
  @fn                     VED_AyGetPrivateData */ /*!
  @brief                  Access to internal data

  @description            Allows accessing the lateral acceleration data from outside
                          of this module

  @param[in]              - 
  @param[out]             -
  @return                 *VED_SwaSenData_t  

  @pre                    -
  @post                   -
**************************************************************************** */
VED_AyData_t *VED_AyGetPrivateData(void) { return (AY_GET_ME); }

#if (CFG_VED__YW_DYN_AVG)
/* **********************************************************************
  @fn                     VED_SwaCheckOffsetGoodEnough */ /*!
  @brief                  checks the quality of the steering wheel offset

  @description            returns TRUE if quality of steering wheel offset
                          is sufficient

  @param[in]              - 
  @param[out]             -
  @return                 boolean quality information
  
  @pre                    -
  @post                   -
**************************************************************************** */
boolean VED_SwaCheckOffsetGoodEnough(void) {
    boolean ret;

    VED_SwaSenData_t *sen = SWA_GET_SEN_DATA;
    VED_SwaOffsData_t *offs = SWA_GET_OFF_DATA;

    if ((sen->Valid == FALSE) || (offs->OffsState > SWA_STATE_6) ||
        (offs->OffsState < SWA_STATE_2)) {
        ret = FALSE;
    } else {
        ret = TRUE;
    }
    return ret;
}
#endif /* CFG_VED__YW_DYN_AVG */

/* **********************************************************************
  @fn                     VED_SwaValid */ /*!
  @brief                  checks the quality of the lateral acceleration offset

  @description            returns TRUE if quality of steering wheel offset
                          is sufficient

  @param[in]              - 
  @param[out]             -
  @return                 boolean quality information 
 
  @pre                    -
  @post                   -
**************************************************************************** */
boolean VED_SwaIsValid(void) {
    VED_SwaSenData_t *StrgData = SWA_GET_SEN_DATA;

    return (StrgData->Valid);
}

/* **********************************************************************
  @fn                     VED_SwaGetOffsData */ /*!
  @brief                  Access to internal offset data

  @description            Allows accessing the steering wheel offset data
                          from outside of this module

  @return                 *VED_SwaOffsData_t
  @param[in]              -
  @param[out]             -
  
  @pre                    -
  @post                   -
**************************************************************************** */
const VED_SwaOffsData_t *VED_SwaGetOffsData(void) { return (SWA_GET_OFF_DATA); }

/* **********************************************************************
  @fn                     VED_SwaNvmWriteOffset */ /*!
  @brief                  Writes steering wheel offset in NVM

  @description            Updates NVM data if EEPromWriteRequest is set

  @param[in]              NVValue
  @param[out]             nvout
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static uint32 VED_SwaNvmWriteOffset(VED_SwaOffsEEprom_t *NVValue,
                                    VEDNvStWhlAngCalc_t *nvout) {
    uint32 State = VED_IO_STATE_INVALID;

    if (NVValue->EEPromWriteRequ == TRUE) {
        nvout->ZeroAngle = NVValue->EEPromStrgOffs.Offset;
        nvout->CalStatus = (uint32)NVValue->EEPromStrgOffs.State;

        State = VED_IO_STATE_VALID;
        NVValue->EEPromWriteRequ = FALSE;
    }

    return State;
}

/* **********************************************************************
  @fn                     VED_SwaNvmReadOffset */ /*!
  @brief                  Reads steering wheel offset from NVM

  @description            Updates RAM data if EEPromReadOk is not set

  @param[in]              NVValue
  @param[in]              nvin
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaNvmReadOffset(VED_SwaOffsEEprom_t *NVValue,
                                 const VEDNvStWhlAngCalc_t *nvin,
                                 uint32 status) {
    if (NVValue->EEPromReadOk == FALSE) {
        if (status == SWA_NVM_CLEARED) {
            /* Set the offset and state to Default Values since NVM is
             * crashed/Erased */
            NVValue->EEPromStrgOffs.Offset = SWA_ANG_OFFS_DEFAULT;
            NVValue->EEPromStrgOffs.State = SWA_STATE_DEFAULT;
        } else {
            /* Upload data from nvmemory */
            NVValue->EEPromStrgOffs.Offset = nvin->ZeroAngle;
            NVValue->EEPromStrgOffs.State = (sint32)nvin->CalStatus;
        }

        NVValue->EEPromStrgOffs.Dev =
            VED_SwaGetOffsetDeviation(NVValue->EEPromStrgOffs.State);
        NVValue->EEPromReadOk = TRUE;
    }
    return;
}

/* **********************************************************************
  @fn                     VED_SwaInitOffsEepromMirror */ /*!
  @brief                  Init NVM steering wheel offset mirror 

  @description            see short description

  @param[in]              NVValue
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaInitOffsEepromMirror(VED_SwaOffsEEprom_t *NVValue) {
    NVValue->EEPromStrgOffs.Offset = (float32)0.F;
    NVValue->EEPromStrgOffs.Dev = SWA_STATE_0_DEV;
    NVValue->EEPromStrgOffs.State = SWA_STATE_DEFAULT;
    NVValue->EEPromWriteRequ = FALSE;
    NVValue->EEPromReadOk = FALSE;

    return;
}

/* **********************************************************************
  @fn                     VED_SwaInitSenData */ /*!
  @brief                  Initialization of steering wheel angle signal data

  @description            see short description

  @param[in]              pSwaSen 
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaInitSenData(VED_SwaSenData_t *pSwaSen) {
    pSwaSen->StrgDeltaDist = (float32)0.0F;
    pSwaSen->Gradient = (float32)0.0F;
    pSwaSen->StrgGradAbsOld = (float32)0.0F;
    pSwaSen->StrgAngle = (float32)0.0F;
    pSwaSen->StrgAngleOld = (float32)0.0F;
    pSwaSen->Valid = FALSE;

    return;
}

/* **********************************************************************
  @fn                     VED_SwaInitOffsEst*/ /*!
  @brief                  Init data of estimated offset 

  @description            see short description

  @param[in]              OffsEst  
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaInitOffsEst(VED_SwaOffsEst_t *OffsEst) {
    const float32 maxRg_c =
        (((float32)((uint32)((float32)SWA_OFFS_HIST_NO_BINS / 2.0F))) + 0.5F) *
        SWA_OFFSET_HIST_WIDTH;

    OffsEst->Offs = 0.F;

    /* Initialize estimation results */
    OffsEst->ThrldDev = SWA_DEV_HIST_STAT_DEFAULT;
    OffsEst->ThrldDist = SWA_DIST_SAMP_STAT_DEFAULT;
    OffsEst->Conf = 0.F;

    /* Setup histogram data base */
    VED_HistInit(&OffsEst->Hist, OffsEst->Bin.Range, OffsEst->Bin.Volume,
                 SWA_OFFS_HIST_NO_BINS, -maxRg_c, maxRg_c);

    return;
}

/* ***********************************************************************
  @fn                     VED_SwaInitOffsData */ /*!
  @brief                  Init of complete offset data

  @description            see short description

  @param[in]              SwaOffs
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaInitOffsData(VED_SwaOffsData_t *SwaOffs) {
    SwaOffs->StrgOffsReadOk = FALSE;
    SwaOffs->StrgOffset = SWA_ANG_OFFS_DEFAULT;
    SwaOffs->StrgOffsetNorm = (float32)0.0F;
    SwaOffs->OffsState = SWA_STATE_DEFAULT;
    SwaOffs->Dev = SWA_STATE_0_DEV;
    SwaOffs->OvrTakeCntr = 0UL;
    SwaOffs->WhlFrDiffFilt = 0.F;
    SwaOffs->WhlReDiffFilt = 0.F;
    SwaOffs->LatAccel = 0.F;

    SwaOffs->OffsInterimOk = FALSE;
    SwaOffs->ErrStrgOffsOutOfRange = FALSE;
    SwaOffs->ReInitCntr = 0U;

    VED_SwaInitOffsEst(&SwaOffs->Est);

    return;
}

/* ***********************************************************************
  @fn                     VED_SwaInitVolatile */ /*!
  @brief                  Initialization of volatile steering wheel angle data

  @description

  @param[in]              pSwaData 
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaInitVolatile(VED_SwaData_t *pSwaData) {
    VED_SwaInitSenData(&pSwaData->Sensor);
    (void)memset(&pSwaData->FastSwaOffset, 0x00, sizeof(s_VED_FastSwaOffset_t));

    /* Init error states */
    pSwaData->Io.errOffsRg = NULL;
    pSwaData->Io.NVMerrOffsRg = NULL;

    return;
}

/* ***********************************************************************
  @fn                     VED_SwaInitOffset */ /*!
  @brief                  Init of steering wheel offset data

  @description            see short description

  @param[in]              Offset
  @param[in]              NVValue
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaInitOffset(VED_SwaOffsData_t *Offset,
                              VED_SwaOffsEEprom_t *NVValue) {
    VED_SwaInitOffsEepromMirror(NVValue);

    VED_SwaInitOffsData(Offset);

    return;
}

/* ***********************************************************************
  @fn                     VED_AyInitSenData */ /*!
  @brief                  Initialization of lateral acceleration data

  @description            see short description

  @param[in]              pAySen
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_AyInitSenData(VED_AySenData_t *pAySen) {
    pAySen->Ay = (float32)0.F;
    pAySen->AyOld = (float32)0.F;

    pAySen->AyGradAbsOld = (float32)0.F;
    pAySen->Gradient = (float32)0.F;

    pAySen->Valid = FALSE;

    return;
}

/* ***********************************************************************
  @fn                     VED_AyInitOffsEst */ /*!
  @brief                  Init of lateral acceleration offset data

  @description            see short description

  @param[in]              pAyOffsEst 
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_AyInitOffsEst(VED_AyOffsEst_t *pAyOffsEst) {
    const float32 maxRg_c =
        (((float32)((uint32)((float32)AY_OFFS_HIST_NO_BINS / 2.0F))) + 0.5F) *
        AY_OFFSET_HIST_WIDTH;

    pAyOffsEst->Offs = 0.F;

    pAyOffsEst->ThrldDev = AY_DEV_SAMP_STAT_DEFAULT;
    pAyOffsEst->ThrldDist = AY_DIST_SAMP_STAT_DEFAULT;
    pAyOffsEst->Conf = 0.F;

    VED_HistInit(&pAyOffsEst->Hist, pAyOffsEst->Bin.Range,
                 pAyOffsEst->Bin.Volume, AY_OFFS_HIST_NO_BINS, -maxRg_c,
                 maxRg_c);

    return;
}

/* ***********************************************************************
  @fn                     VED_AyInitOffsData */ /*!
  @brief                  Init of estimated lateral acceleration offset data

  @description            see short description

  @param[in]              pAyOffs
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_AyInitOffsData(VED_AyOffsData_t *pAyOffs) {
    pAyOffs->AyOffset = (float32)0.F;
    pAyOffs->Dev = AY_STATE_0_DEV;
    pAyOffs->OffsState = AY_STATE_DEFAULT;
    pAyOffs->OvrTakeCntr = 0UL;
    pAyOffs->Interims = FALSE;
    pAyOffs->AyOffsReadOk = FALSE;

    VED_AyInitOffsEst(&pAyOffs->Est);

    return;
}

/* ***********************************************************************
  @fn                     VED_AyInitVolatile */ /*!
  @brief                  Initialization of volatile lateral acceleration data

  @description            see short description

  @param[in]              pAyData 
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_AyInitVolatile(VED_AyData_t *pAyData) {
    VED_AyInitSenData(&pAyData->Sensor);

    /* Init error states */
    pAyData->Io.errOffsRg = NULL;
    pAyData->Io.NVMerrOffsRg = NULL;

    return;
}

/* ***********************************************************************
  @fn                     VED_SwaInitOffset */ /*!
  @brief                  Init of lateral offset data

  @description            see short description

  @param[in]              pAyOffset 
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_AyInitOffset(VED_AyOffsData_t *pAyOffset) {
    VED_AyInitOffsData(pAyOffset);

    return;
}

/* ***********************************************************************
  @fn                     VED_SwaEvalThrdDist */ /*!
  @brief                  Sets required distance for next learning state

  @description            Sets the distance which is needed to learn the next
                          steering wheel offset based on the current state

  @param[in]              Status   steering wheel offset learning state
  @param[out]             -
  @return                 distance
  
  @pre                    -
  @post                   -
**************************************************************************** */
static float32 VED_SwaEvalThrdDist(sint32 Status) {
    float32 reqDist; /* required distance */

    switch (Status) {
        case (SWA_STATE_1):
            /* Run-up phase */
            reqDist = SWA_DIST_SAMP_STAT_1;
            break;

        case (SWA_STATE_2):
            /* Run-up phase */
            reqDist = SWA_DIST_SAMP_STAT_2;
            break;

        case (SWA_STATE_3):
            /* Run-up phase */
            reqDist = SWA_DIST_SAMP_STAT_3;
            break;

        case (SWA_STATE_4):
            /* Relearn phase */
            reqDist = SWA_DIST_SAMP_STAT_4;
            break;

        case (SWA_STATE_5):
            /* Confirmation phase */
            reqDist = SWA_DIST_SAMP_STAT_5;
            break;

        case (SWA_STATE_6):
            /* Confirmation phase */
            reqDist = SWA_DIST_SAMP_STAT_6;
            break;

        default:
            reqDist = SWA_DIST_SAMP_STAT_DEFAULT;
            break;
    }

    return (reqDist);
}

/* ***********************************************************************
  @fn                     VED_SwaEvalThrdDeviation */ /*!
  @brief                  Sets required offset difference

  @description            The maximum deviation of the steering wheel offset angle
                          for new offset estimation is set based on current state

  @param[in]              Status   steering wheel offset learning state
  @param[out]             -
  @return                 float32  angle threshold
  
  @pre                    -
  @post                   -
**************************************************************************** */
static float32 VED_SwaEvalThrdDeviation(sint32 Status) {
    float32 Angle;

    switch (Status) {
        case (SWA_STATE_1):
        case (SWA_STATE_2):
        case (SWA_STATE_3):

            /* Run-up phase */
            Angle = SWA_DEV_HIST_STAT_STARTUP;
            break;

        case (SWA_STATE_4):
        case (SWA_STATE_5):
        case (SWA_STATE_6):

            /* Confirmation phase */
            Angle = SWA_DEV_HIST_STAT_NORMAL;
            break;

        default:
            Angle = SWA_DEV_HIST_STAT_STARTUP;
            break;
    }

    return (Angle);
}

/* ************************************************************************
  @fn                     VED_AyEvalThrdDist */ /*!
  @brief                  Sets required distance for next learning state

  @description            Sets the distance which is needed to learn the next
                          lateral acceleration offset based on the current state

  @param[in]              Status   lateral acceleration offset learning state
  @param[out]             -
  @return                 distance

  @pre                    -
  @post                   -
**************************************************************************** */
static float32 VED_AyEvalThrdDist(sint32 Status) {
    float32 reqDist; /* required distance */

    switch (Status) {
        case (AY_STATE_1):
            /* Run-up phase */
            reqDist = AY_DIST_SAMP_STAT_1;
            break;

        case (AY_STATE_2):
            /* Run-up phase */
            reqDist = AY_DIST_SAMP_STAT_2;
            break;

        case (AY_STATE_3):
            /* Learn and relearn state */
            reqDist = AY_DIST_SAMP_STAT_3;
            break;

        default:
            reqDist = AY_DIST_SAMP_STAT_DEFAULT;
            break;
    }

    return (reqDist);
}

/* ***********************************************************************
  @fn                     VED_AyEvalThrdDeviation */ /*!
  @brief                  Sets required offset difference

  @description            The maximum deviation of the lateral acceleration offset angle
                          for new offset estimation is set based on current state

  @param[in]              Status   lateral acceleration offset learning state
  @param[out]             -
  @return                 acceleration threshold

  @pre                    -
  @post                   -
**************************************************************************** */
static float32 VED_AyEvalThrdDeviation(sint32 Status) {
    float32 Accel;

    switch (Status) {
        case (AY_STATE_1):
        case (AY_STATE_2):
            /* Run-up phase */
            Accel = AY_DEV_SAMP_STAT_STARTUP;
            break;

        case (AY_STATE_3):
            /* Learn and relearn phase  */
            Accel = AY_DEV_SAMP_STAT_NORMAL;
            break;

        default:
            Accel = AY_DEV_SAMP_STAT_DEFAULT;
            break;
    }

    return (Accel);
}

/* ***********************************************************************
  @fn                     VED_SwaOffsetRangeOk */ /*!
  @brief                  Checks steering angle offset range

  @param[in]              Offset value
  @param[out]             -
  @return                 Offset OK = TRUE, NOT_OK = FALSE

  @pre                    -
  @post                   -
**************************************************************************** */
static boolean VED_SwaOffsetRangeOk(float32 Offset) {
    float32 StrgAngOffsNorm;
    boolean StrgOffsOk = FALSE;

    /* norm offset */
    StrgAngOffsNorm = VED_Discretize(Offset, SWA_OFFS_NORM_RES);

    /* check the offset range */
    if (fABS(StrgAngOffsNorm) <= VED__PAR_SWA_OFFSET_LIMIT_MAX) {
        /* offset within range  */
        StrgOffsOk = TRUE;
    } else {
        /* offset outside of range */
        StrgOffsOk = FALSE;
    }

    return (StrgOffsOk);
}

/* ***********************************************************************
  @fn                     VED_AyOffsetRangeOk */ /*!
  @brief                  Checks lateral acceleration range

  @param[in]              Offset value
  @param[out]             -
  @return                 Offset OK = TRUE, NOT_OK = FALSE

  @pre                    -
  @post                   -
**************************************************************************** */
static boolean VED_AyOffsetRangeOk(float32 Offset) {
    boolean AyOffsOk = FALSE;

    /* Verfiy that offset value is inside plausible range */
    if (fABS(Offset) <= AY_OFFSET_LIMIT_MAX) {
        /* Offset value is ok */
        AyOffsOk = TRUE;
    }

    return AyOffsOk;
}

/* **********************************************************************
  @fn                     VED_SwaPutEepromOffsetData */ /*!
  @brief                  Stores offset in NVM mirror and set write
                          request for writing data into NVM

  @description            see brief description

  @param[in]              Offset (offset value)
  @param[in]              Status (learning state)
  @param[in]              Dev    (standard deviation)
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaPutEepromOffsetData(float32 Offset,
                                       sint32 Status,
                                       float32 Dev) {
    VED_SwaOffsEEprom_t *StrgOffsEEprom =
        SWA_GET_NV_DATA; /* Zugriff auf EEProm-Mirror */

    /* LW-Offset in EEProm-Mirror kopieren */
    StrgOffsEEprom->EEPromStrgOffs.Offset = Offset;
    StrgOffsEEprom->EEPromStrgOffs.State = Status;
    StrgOffsEEprom->EEPromStrgOffs.Dev = Dev;

    /* Leseanforderung setzen, abspeichern des Eeprom-Mirrors */
    StrgOffsEEprom->EEPromWriteRequ = TRUE;

    return;
}

/* **********************************************************************
  @fn               VED_AyPutNvmOffsetData */ /*!
  @brief            Store values in nonvolatile memory

  @description      see brief description
  @param[in]        Offset (offset value)
  @param[in]        Status (learning state)
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static void VED_AyPutNvmOffsetData(float32 Offset, sint32 Status) {
    VEDNvIoDatas_t *nvm = AY_GET_NV_WRITE_DATA;

    nvm->LatAcc.ZeroAccel = Offset;
    nvm->LatAcc.CalStatus = (uint32)Status;

    VED_SET_NVM_IO_STATE(VED_NVM_POS_LATACC, VED_IO_STATE_VALID, &nvm->State);

    return;
}

/* **********************************************************************
  @fn               VED_AyGetNvmOffsetData */ /*!
  @brief            Read values in nonvolatile memory

  @description      see brief description
  @param[in]        Offset
  @param[in]        Status
  @param[in]        Dev
  @return           void

  @pre              -
  @post             -
**************************************************************************** */
static boolean VED_AyGetNvmOffsetData(float32 *Offset,
                                      sint32 *Status,
                                      float32 *Dev) {
    const VEDNvIoDatas_t *nvm = AY_GET_NV_READ_DATA;
    boolean ret = FALSE;

#if (!(defined(CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK)) || \
     (!CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK))
    if (VED__GET_NVM_IO_STATE(VED_NVM_POS_LATACC, &nvm->State) ==
        VED_IO_STATE_VALID)
#endif
    {
        /* NVM read operation successfully completed */
        if (nvm->State == SWA_NVM_CLEARED) {
            *Offset = AY_OFFS_DEFAULT;
            *Status = AY_STATE_DEFAULT;
        } else {
            *Offset = nvm->LatAcc.ZeroAccel;
            *Status = (sint32)nvm->LatAcc.CalStatus;
        }
        *Dev = VED_AyGetOffsetDeviation(*Status);

        ret = TRUE;
    }
    return ret;
}

/* **********************************************************************
  @fn                     VED_SwaGetOffset */ /*!
  @brief                  Get steering wheel offset from NVM and check for plausibility

  @description             see brief description

  @param[in]              -
  @param[out]             Offset (offset value)
  @param[out]             Status (learning state)
  @param[out]             Dev    (standard deviation)
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static boolean VED_SwaGetOffset(float32 *Offset, sint32 *Status, float32 *Dev) {
    boolean StrgOffsOk = FALSE;
    VED_SwaOffsEEprom_t *StrgOffsEEprom =
        SWA_GET_NV_DATA; /* Zugriff auf EEProm-Mirror */

    /* Set StrgOffs, StrgStatus, StrgDev to init values.  If NVM access fails,
     */
    /* these default values are used and not the NVM values */
    *Offset = SWA_ANG_OFFS_DEFAULT;
    *Status = SWA_STATE_DEFAULT;
    *Dev = SWA_STATE_0_DEV;

    if (StrgOffsEEprom->EEPromReadOk == TRUE) {
        /* Read operation successfully completed */
        if ((VED_SwaOffsetRangeOk(StrgOffsEEprom->EEPromStrgOffs.Offset) !=
             TRUE)) {
            /* Stored offset value is out of range */

            /* Indicate error state active "false value stored in nvmemory "*/
            SWA_GET_NVM_ERR_OFFS_RG = (VED_ERR_STATE_ACTIVE);

            /* Try to correct invalid values in nv memory by overwriting with
             * defaults */
            VED_SwaPutEepromOffsetData(*Offset, *Status, *Dev);
        } else if ((StrgOffsEEprom->EEPromStrgOffs.State < SWA_STATE_DEFAULT) ||
                   (StrgOffsEEprom->EEPromStrgOffs.State > SWA_STATE_6)) {
            /* Stored offset state is out of range  */

            /* @todo Indicate error state active "false state stored in nvmemory
             */
            SWA_GET_NVM_ERR_OFFS_RG = (VED_ERR_STATE_ACTIVE);
            /* (void) ERRCheckForErrorTaskingActive(TRUE,
             * ERR_LP_STRG_OFFS_READ_EEPR_UNKNOWN_STATUS); */

            /* Try to correct invalid values in nv memory by overwriting with
             * defaults */
            VED_SwaPutEepromOffsetData(*Offset, *Status, *Dev);
        } else {
            /* Stored swa nvmem values are OK! */

            /* LW-Offset aus EEProm in Applikation kopieren */
            *Offset = StrgOffsEEprom->EEPromStrgOffs.Offset;
            *Status = StrgOffsEEprom->EEPromStrgOffs.State;
            *Dev = StrgOffsEEprom->EEPromStrgOffs.Dev;

            SWA_GET_NVM_ERR_OFFS_RG = (VED_ERR_STATE_INACTIVE);
        }

        /* Data read from NVM */
        StrgOffsOk = TRUE;
    }
    return (StrgOffsOk);
}

/* **********************************************************************
 @fn                     VED_AySwaAcqEstData */ /*!
  @brief                  Estimation of steering wheel angle offset

  @description            see brief description

  @param[in]              cycDist (driven distance)
  @param[out]             swaSen (steering wheel data)
  @param[out]             aySen (lat accel data)
  @param[out]             swaOffs (steering wheel offset data)
  @param[out]             ayOffs (lat accel offset data)
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_AySwaAcqEstData(const VED_SwaSenData_t *swaSen,
                                const VED_AySenData_t *aySen,
                                VED_SwaOffsData_t *swaOffs,
                                VED_AyOffsData_t *ayOffs,
                                float32 cycDist) {
    /* local storage for absolute values */
    float32 f_StrgAngleAbs;
    float32 f_WhlFrDiffFiltAbs;
    float32 f_WhlReDiffFiltAbs;

#if (((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) ||            \
      (!CFG_VED__DIS_WHEEL_PRE_PROCESSING)) ||                    \
     ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
      (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)))
    const V1_7_VEDVehSigMain_t *in = SWA_GET_INP_SIGNALS;
#endif
    uint8 SWACaliState = SWA_GET_ME->Io.in->Frame.CaliMode;
#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
    uint8 AYCaliState = AY_GET_ME->Io.in->Frame.CaliMode;
#endif

    /* Filter front wheel velocities differences */
#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    if ((VED_GET_IO_STATE(VED_SIN_POS_WVEL_FL, in->State) ==
         VED_IO_STATE_VALID) &&
        (VED_GET_IO_STATE(VED_SIN_POS_WVEL_FR, in->State) ==
         VED_IO_STATE_VALID)) {
        swaOffs->WhlFrDiffFilt =
            VED_FilterCycleTime(in->WhlVelFrLeft - in->WhlVelFrRight,
                                swaOffs->WhlFrDiffFilt, SWA_OFFS_WHS_DIFF_FT);
    } else
#endif
    {
        swaOffs->WhlFrDiffFilt = 0.F;
    }

    /* Filter rear wheel velocities differences */
#if ((!defined(CFG_VED__DIS_WHEEL_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_WHEEL_PRE_PROCESSING))
    if ((VED_GET_IO_STATE(VED_SIN_POS_WVEL_RL, in->State) ==
         VED_IO_STATE_VALID) &&
        (VED_GET_IO_STATE(VED_SIN_POS_WVEL_RR, in->State) ==
         VED_IO_STATE_VALID)) {
        swaOffs->WhlReDiffFilt =
            VED_FilterCycleTime(in->WhlVelReLeft - in->WhlVelReRight,
                                swaOffs->WhlReDiffFilt, SWA_OFFS_WHS_DIFF_FT);
    } else
#endif
    {
        swaOffs->WhlReDiffFilt = 0.F;
    }

    f_StrgAngleAbs = fABS(swaSen->StrgAngle);
    f_WhlFrDiffFiltAbs = fABS(swaOffs->WhlFrDiffFilt);
    f_WhlReDiffFiltAbs = fABS(swaOffs->WhlReDiffFilt);

    /* Validity condition for long term compensation */
    if ((f_StrgAngleAbs < SWA_OFFS_ANGLE_MAX) &&
        ((SWA_GET_MIF_DATA->LongMot.VehVelocityCorr > SWA_OFFS_VEL_MIN) &&
         (SWA_GET_MIF_DATA->LongMot.MotState.MotState !=
          (uint8)VED_LONG_MOT_STATE_MOVE_RWD)) &&
        ((f_WhlFrDiffFiltAbs < SWA_OFFS_WHS_DIFF_VEL) &&
         (f_WhlReDiffFiltAbs < SWA_OFFS_WHS_DIFF_VEL))) {
#if ((!defined(CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING))
        /* Integrate lateral acceleration to detect circular routes */
        if (VED_GET_IO_STATE(VED_SIN_POS_LATA, in->State) ==
            VED_IO_STATE_VALID) {
            swaOffs->LatAccel = VED_FilterCycleTime(
                in->LatAccel, swaOffs->LatAccel, SWA_OFFS_LAT_ACC_FT);
        }
#endif
        /* Accumulate driven distance */
        if (swaOffs->Est.Hist.Sum <= SWA_OFFS_DRIVEN_DIST_MAX) {
            float32 currAccel;

            if (VED_GET_IO_STATE(VED_SIN_POS_LATA, in->State) ==
                VED_IO_STATE_VALID) {
                currAccel = aySen->Ay;
                /* Only if lateral acceleration calibration mode is off add
                 * samples to histgramm */
                if (!VED__CTRL_GET_STATE((uint8)VED_CAL_LTA_OFFS,
                                         AYCaliState)) {
                    /* Add lateral acceleration samples to histogram */
                    VED_HistAdd(&ayOffs->Est.Hist, aySen->Ay, cycDist);
                }
            } else

            {
                currAccel = (float32)0.F;
            }

            if (fABS(currAccel) < AY_SWA_LIMIT_HIST) {
                /* Only if steering wheel angle calibration mode is off add
                 * samples to histgramm */
                if (!VED__CTRL_GET_STATE((uint8)VED_CAL_SWA_OFFS,
                                         SWACaliState)) {
                    /* Add steering angle samples to histogram */
                    VED_HistAdd(&swaOffs->Est.Hist, swaSen->StrgAngle, cycDist);
                }
            }
        }
    }
    return;
}

/* **********************************************************************
  @fn                     VED_SwaTakeoverOffset */ /*!
  @brief                  Take estimated steering wheel offset as active offset 

  @description            Calculates the new offset value based on value difference
                          and learning state
                          Increases or decreases leraning state according to difference
                          and reduces the measurement data

  @param[in]              StrgOffsData
  @param[in]              OffsEst
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaTakeoverOffset(VED_SwaOffsData_t *StrgOffsData,
                                  VED_SwaOffsEst_t *OffsEst) {
    const float32 Weightn_c =
        0.5F;               /* Weight for new offset estimation max. 1.0 */
    float32 SwaOffsNorm;    /* Normalised steering wheel offset */
    float32 SwaOffsDiff;    /* Difference between new and old offset */
    float32 SwaOffsAbsDiff; /* Absolute difference between new and old offset */
    float32 SwaOffsNew;     /* New offset value */

    /* Differences between active and new estimated offset */
    SwaOffsDiff = OffsEst->Offs - StrgOffsData->StrgOffset;
    SwaOffsAbsDiff = fABS(SwaOffsDiff);

    /* No active offset available (the estimated offset is the first calculated
     * offset)? */
    if (StrgOffsData->OffsState == SWA_STATE_DEFAULT) {
        /* Use estimated offset as first offset, no appoximation with active
         * offset (no active offset available) */
        SwaOffsNew = OffsEst->Offs;
    } else {
        /* Calculate average between active and estimated offset for successive
         * approximation of final offset */
        SwaOffsNew = StrgOffsData->StrgOffset + (Weightn_c * SwaOffsDiff);
    }

    /* Normalize offset value @todo is normalized offset still necessary */
    SwaOffsNorm = VED_Discretize(SwaOffsNew, SWA_OFFS_NORM_RES);

    /* Acquisition strategy is dependent on current state */
    switch (StrgOffsData->OffsState) {
        case SWA_STATE_DEFAULT:
        case SWA_STATE_1:
        case SWA_STATE_2:
        case SWA_STATE_3:

            /*<<<<<<<< L E A R N   P H A S E >>>>>>>>*/

            /* When offset value is stable or it is the first offset, increment
             * state  */
            if ((SwaOffsAbsDiff <= SWA_OFFS_STAT_DIFF_RUN_UP) ||
                (StrgOffsData->OffsState == SWA_STATE_DEFAULT)) {
                StrgOffsData->OvrTakeCntr++;
                if (StrgOffsData->OvrTakeCntr < SWA_OFFS_TAKE_OVR_MAX) {
                    StrgOffsData->OffsState += 1;
                }
            }

            /* Take over offset and deviation */
            StrgOffsData->StrgOffset = SwaOffsNew;
            StrgOffsData->StrgOffsetNorm = SwaOffsNorm;
            StrgOffsData->Dev =
                VED_SwaGetOffsetDeviation(StrgOffsData->OffsState);

            /* Save offset value and status in nonvolatile memory */
            VED_SwaPutEepromOffsetData(SwaOffsNew, StrgOffsData->OffsState,
                                       SWA_OFFS_DEV_NORMAL);

            /* Initialize estimation data */
            VED_HistReInit(&OffsEst->Hist);

            break;

        case SWA_STATE_4:

            /*<<<<<<<< R E L E A R N   P H A S E >>>>>>>>*/

            /* Test offset against stability */
            if (SwaOffsAbsDiff >= SWA_OFFS_CHANGE_HYST) {
                /* Estimated offset differs significantly from active offset:
                 * restore learned offset value */

                /* Remain in this offset calibration state */
                StrgOffsData->OffsState = SWA_STATE_4;

                /* Take over offset */
                StrgOffsData->StrgOffset = SwaOffsNew;
                StrgOffsData->Dev =
                    VED_SwaGetOffsetDeviation(StrgOffsData->OffsState);
                StrgOffsData->StrgOffsetNorm = SwaOffsNorm;

                /* Initialize estimation data */
                VED_HistReInit(&OffsEst->Hist);
            } else {
                /* Estimated offset approves active and stored offset */

                /* Imcrement offset state */
                StrgOffsData->OffsState = SWA_STATE_5;

                /* Take over offset */
                StrgOffsData->StrgOffset = SwaOffsNew;
                StrgOffsData->Dev =
                    VED_SwaGetOffsetDeviation(StrgOffsData->OffsState);
                StrgOffsData->StrgOffsetNorm = SwaOffsNorm;

                /* Indicate that offset value has been confirmed during this
                   ignition cylce, i.e .one confirmation per ignition cylce i.
                   e. hold offset state in next state for this ignition cycle */
                StrgOffsData->OvrTakeCntr++;

                /* Scale back estimation data */
                VED_HistReduce(&OffsEst->Hist, SWA_INTERVAL_RDCT_FCTR);
            }

            /* Save offset value and status in nonvolatile memory */
            VED_SwaPutEepromOffsetData(StrgOffsData->StrgOffset,
                                       StrgOffsData->OffsState,
                                       SWA_OFFS_DEV_NORMAL);

            break;

        case SWA_STATE_5:
        case SWA_STATE_6:

            /*<<<<<<<< C O N F I R M A T I O N   P H A S E >>>>>>>*/

            if (SwaOffsAbsDiff >= SWA_OFFS_CHANGE_HYST) {
                /* Estimated offset differs significantly form active offset:
                   lower offset state without changing the offset value */
                StrgOffsData->OffsState -= 1;

                /* Save offset state in nonvolatile memory  */
                VED_SwaPutEepromOffsetData(StrgOffsData->StrgOffset,
                                           StrgOffsData->OffsState,
                                           SWA_OFFS_DEV_NORMAL);
            } else {
                /* Estimated offset approves active and stored offset */
                if ((StrgOffsData->OvrTakeCntr == 0u) &&
                    (StrgOffsData->OffsState < SWA_STATE_6)) {
                    /* Offset state 5 has been reached since last ignition
                     * cycle, move to state 6 */
                    StrgOffsData->OffsState = SWA_STATE_6;

                    /* Take over offset */
                    StrgOffsData->StrgOffset = SwaOffsNew;
                    StrgOffsData->Dev =
                        VED_SwaGetOffsetDeviation(StrgOffsData->OffsState);
                    StrgOffsData->StrgOffsetNorm = SwaOffsNorm;

                    /* Save offset value and status in nonvolatile memory */
                    VED_SwaPutEepromOffsetData(StrgOffsData->StrgOffset,
                                               StrgOffsData->OffsState,
                                               SWA_OFFS_DEV_NORMAL);
                }
            }

            /* Scale back estimation data */
            VED_HistReduce(&OffsEst->Hist, SWA_INTERVAL_RDCT_FCTR);

            break;

        default:

            /* This state is not regular */

            /* Set default state */
            StrgOffsData->OffsState = SWA_STATE_DEFAULT;

            /* Initialize estimation data */
            VED_HistReInit(&OffsEst->Hist);

            break;
    }
    return;
}

/* **********************************************************************
  @fn                     VED_SwaCheckConfidence */ /*!
  @brief                  Returns the confidence of the steering wheel offset

  @description            Confidence is based on learning state and range

  @param[in]              OffsData
  @param[out]             -
  @return                 boolean confidence
  
  @pre                    -
  @post                   -
**************************************************************************** */
static boolean VED_SwaCheckConfidence(const VED_SwaOffsData_t *OffsData) {
    float32 reqConf;
    boolean ret = FALSE;

    if (OffsData->OffsState > SWA_STATE_3) {
        reqConf = AY_SWA_OFFS_MID_CONF;
    } else {
        reqConf = AY_SWA_OFFS_LOW_CONF;
    }

    if ((VED_SwaOffsetRangeOk(OffsData->Est.Offs) != FALSE) &&
        (OffsData->Est.Conf >= reqConf)) {
        ret = TRUE;
    }
    return ret;
}

/* **********************************************************************
  @fn                     VED_AyCheckConfidence */ /*!
  @brief                  Returns the confidence of the lat accel offset

  @description            Confidence is based on learning state and range

  @param[in]              ayOffs
  @param[out]             -
  @return                 boolean confidence
  
  @pre                    -
  @post                   -
**************************************************************************** */
static boolean VED_AyCheckConfidence(const VED_AyOffsData_t *ayOffs) {
    float32 reqConf;
    boolean ret = FALSE;

    if (ayOffs->OffsState > AY_STATE_2) {
        reqConf = AY_SWA_OFFS_MID_CONF;
    } else {
        reqConf = AY_SWA_OFFS_LOW_CONF;
    }

    if ((VED_AyOffsetRangeOk(ayOffs->Est.Offs) != FALSE) &&
        (ayOffs->Est.Conf >= reqConf)) {
        ret = TRUE;
    }

    return ret;
}

/* **********************************************************************
  @fn                     VED_SwaGetOffsetStart */ /*!
  @brief                  Get non volatile start values of steering wheel offset

  @description            Reads and normalises data from NVM and determines
                          required distance and difference

  @param[in]              SwaOffsData
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaGetOffsetStart(VED_SwaOffsData_t *SwaOffsData) {
    /* Read offset start values if not OK */
    if (SwaOffsData->StrgOffsReadOk == FALSE) {
        /* Read non-volatile offset calibration data and verify ranges */
        SwaOffsData->StrgOffsReadOk =
            VED_SwaGetOffset(&SwaOffsData->StrgOffset, &SwaOffsData->OffsState,
                             &SwaOffsData->Dev);

        /* Normalize steering wheel angle offset */
        SwaOffsData->StrgOffsetNorm =
            VED_Discretize(SwaOffsData->StrgOffset, SWA_OFFS_NORM_RES);

        /* Determine minimum required driven distance for offset estimation in
         * depedence of state */
        SwaOffsData->Est.ThrldDist =
            VED_SwaEvalThrdDist(SwaOffsData->OffsState);

        /* Determine maximum offset deviation for estimation in dependence of
         * state */
        SwaOffsData->Est.ThrldDev =
            VED_SwaEvalThrdDeviation(SwaOffsData->OffsState);
    }
    return;
}

/* **********************************************************************
  @fn                     VED_AyGetOffsetStart */ /*!
  @brief                  Get non volatile start values of lat accel offset

  @description            Reads and normalises data from NVM and determines
                          required distance and difference

  @param[in]              pAyOffsData
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_AyGetOffsetStart(VED_AyOffsData_t *pAyOffsData) {
    /* Read offset start values if not OK */
    if (pAyOffsData->AyOffsReadOk == FALSE) {
        /* Read non-volatile offset calibration data and verify ranges */
        pAyOffsData->AyOffsReadOk = VED_AyGetOffset(
            &pAyOffsData->AyOffset, &pAyOffsData->OffsState, &pAyOffsData->Dev);

        /* Determine minimum required driven distance for offset estimation in
         * depedence of state */
        pAyOffsData->Est.ThrldDist = VED_AyEvalThrdDist(pAyOffsData->OffsState);

        /* Determine maximum offset deviation for estimation in dependence of
         * state */
        pAyOffsData->Est.ThrldDev =
            VED_AyEvalThrdDeviation(pAyOffsData->OffsState);
    }
    return;
}

/* **********************************************************************
  @fn                     VED_SwaCalcConfidence */ /*!
  @brief                  Calculate confidence of steering wheel offset data

  @description            Calculates mean and deviation of histogram
                          Checks distribution and peaks
                          Sets confidence level based on the histogram quality

  @param[in,out]          StrgOffsData Confidence value
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaCalcConfidence(VED_SwaOffsData_t *StrgOffsData) {
    /* Enough data for offset estimation acquired */
    uint32 maxIdx;
    uint32 rgSort[SWA_OFFS_HIST_NO_BINS];
    VED_Histogram_t *hist = &StrgOffsData->Est.Hist;
    float32 median;

    /* Calculate mean and standard deviation of histogram data */
    VED_HistCalcMeanDev(hist);

    /* Create an sorted index of histogram bin volumes */
    VED_HpSortInd(SWA_OFFS_HIST_NO_BINS, hist->Volume, rgSort);

    /* Calculate median of histogram data */
    median = VED_HistCalcMedian(hist);

    /* Last value of sorted index coresponds to bin with maximum count */
    maxIdx = rgSort[SWA_OFFS_HIST_MAX_IDX];

    /* Init confidence level */
    StrgOffsData->Est.Conf = AY_SWA_OFFS_NO_CONF;

    /* Compare mean with maximum count bin */
    if (fABS(hist->Mean - hist->Range[maxIdx]) < SWA_OFFS_DIFF_MAX_MEAN) {
        /* Range with maximum count is close to mean */

        /* Check if adjacent bins of maximum have enough counts, detect the case
         * with nearly constant angle */
        if ((fABS(hist->Volume[maxIdx] -
                  (hist->Volume[maxIdx - 1u] + hist->Volume[maxIdx + 1u])) <
             (SWA_OFFS_FTHR_DIST_MAX_ADJ * StrgOffsData->Est.ThrldDist)) &&
            ((maxIdx > 1u) && (maxIdx < (hist->Size - 1u)))) {
            /* Normal route characteristics, best confidence */
            StrgOffsData->Est.Conf = AY_SWA_OFFS_MAX_CONF;
        } else {
            /* Driving long section with same steering wheel angle (straight or
             * constant radius) */
            StrgOffsData->Est.Conf = AY_SWA_OFFS_MID_CONF;
        }

        /* bin counts are normally distributed, use mean as offset value */
        StrgOffsData->Est.Offs = hist->Mean;
    } else {
        /* Range with maximum count is far away from mean, counts are not
           normally distributed that means that a secondary peak exists. This
           can be caused by driving oval circuits on proving grounds */

        /* Distance difference between maximum and secondary peak */
        const float32 distMax_c = hist->Volume[rgSort[SWA_OFFS_HIST_MAX_IDX]];
        float32 diffDistMaxMean;

        /* Get distance difference between maximum and range around mean */
        diffDistMaxMean = distMax_c - VED_HistGetVolume(hist, hist->Mean);

        /* If the count difference between secondary peak and maximum peak is
           big enough, blind out secondary peak by use only center around the
           maximum peak */
        if (diffDistMaxMean >
            (SWA_OFFS_FTHR_DIST_MAX_OUT * StrgOffsData->Est.ThrldDist)) {
            /* Calculate mean around maximum peak */
            StrgOffsData->Est.Offs = VED_HistCalcMeanCenter(
                hist, rgSort[SWA_OFFS_HIST_MAX_IDX], 2uL, 1uL);

            /* Limit confidence to avoid high status offset calibration */
            StrgOffsData->Est.Conf = AY_SWA_OFFS_LOW_CONF;
        } else if (fABS(median - hist->Range[maxIdx]) < SWA_OFFS_DIFF_MAX_MED) {
            /* Maximum distance count is not not much above the mean distance
              count, but median and mode are close to together, this can occur
              in case of large offset values, where histogram data is
              unsymmetric because one-sided truncation */
            float32 diffMeanMedian =
                VED_HistCalcMeanCenter(hist, rgSort[SWA_OFFS_HIST_MAX_IDX],
                                       10uL, 1uL) -
                median;

            /* Calculate mean around maximum peak */
            if (fABS(diffMeanMedian) < SWA_OFFS_DIFF_MEAN_CT_MED) {
                /*  Use median as estimated offset */
                StrgOffsData->Est.Offs = median;

                /* Maximum value looks like a outlier, use median value instead
                 */
                StrgOffsData->Est.Conf = AY_SWA_OFFS_LOW_CONF;
            }
        } else if (fABS(median - hist->Mean) < SWA_OFFS_DIFF_MEAN_MED) {
            /* Distribution looks symmetric, but maximum peak is outside the
             * centre */

            /* Use median as estimated offset */
            StrgOffsData->Est.Offs = median;

            if (diffDistMaxMean <
                (SWA_OFFS_FTHR_DIST_MAX_MEAN * StrgOffsData->Est.ThrldDist)) {
                /* Maximum value looks like a outlier, use median value instead
                 */
                StrgOffsData->Est.Conf = AY_SWA_OFFS_MID_CONF;
            } else {
                /* Maximum value looks like a outlier, use median value instead
                 */
                StrgOffsData->Est.Conf = AY_SWA_OFFS_LOW_CONF;
            }
        } else {
            /* two peaks with rougly the same distance and ambiguous route
             * characteristics */

            /* Estimation has no confidence */
            StrgOffsData->Est.Conf = AY_SWA_OFFS_NO_CONF;
        }
    }
    return;
}

/* **********************************************************************
  @fn                     VED_AyCalcConfidence */ /*!
  @brief                  Calculate confidence of lateral acceleration offset data

  @description            Calculates mean and devistion of histogram
                          Checks distribution and peaks
                          Sets confidence level based on the histogram quality

  @param[in]              pOffsAy
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_AyCalcConfidence(VED_AyOffsData_t *pOffsAy) {
    /* Enough data for offset estimation acquired */
    uint32 maxIdx;
    uint32 rgSort[AY_OFFS_HIST_NO_BINS];
    VED_Histogram_t *hist = &pOffsAy->Est.Hist;
    float32 median;

    /* Calculate mean and standard deviation of histogram data */
    VED_HistCalcMeanDev(hist);

    /* Create an sorted index of histgram bin volumes */
    VED_HpSortInd(AY_OFFS_HIST_NO_BINS, hist->Volume, rgSort);

    /* Calculate median of histogram data */
    median = VED_HistCalcMedian(hist);

    /* Last value of sorted index coresponds to bin with maximum count */
    maxIdx = rgSort[AY_OFFS_HIST_MAX_IDX];

    /* Init confidence level */
    pOffsAy->Est.Conf = AY_SWA_OFFS_NO_CONF;

    /* Compare mean with maximum count bin */
    if (fABS(hist->Mean - hist->Range[maxIdx]) < AY_OFFS_DIFF_MAX_MEAN) {
        /* Range with maximum count is close to mean */

        /* Check if adjacent bins of maximum have enough counts, detection the
         * case with nearly constant angle */
        if ((fABS(hist->Volume[maxIdx] -
                  (hist->Volume[maxIdx - 1u] + hist->Volume[maxIdx + 1u])) <
             (AY_OFFS_FTHR_DIST_MAX_ADJ * pOffsAy->Est.ThrldDist)) &&
            ((maxIdx > 1u) && (maxIdx < (hist->Size - 1u)))) {
            /* Normal route characteristics, best confidence */
            pOffsAy->Est.Conf = AY_SWA_OFFS_MAX_CONF;
        } else {
            /* Driving long section with same steering wheel angle (straight or
             * constant radius) */
            pOffsAy->Est.Conf = AY_SWA_OFFS_MID_CONF;
        }

        /* bin counts are normally distributed, use mean as offset value */
        pOffsAy->Est.Offs = hist->Mean;
    } else {
        /* Range with maximum count is far away from mean, counts are not
           normally distributed that means that a secondary peak exists. This
           can be caused by driving oval circuits on proving grounds */

        /* Distance difference between maximum and secondary peak */
        const float32 distMax_c = hist->Volume[rgSort[AY_OFFS_HIST_MAX_IDX]];
        float32 diffDistMaxMean;

        /* Get distance difference between maximum and range around mean */
        diffDistMaxMean = distMax_c - VED_HistGetVolume(hist, hist->Mean);

        /* If the count difference between secondary peak and maximum peak is
           big enough, blind out seondary peak by use only center around the
           maximum peak */
        if (diffDistMaxMean >
            (AY_OFFS_FTHR_DIST_MAX_OUT * pOffsAy->Est.ThrldDist)) {
            /* Calculate mean around maximum peak */
            pOffsAy->Est.Offs = VED_HistCalcMeanCenter(
                hist, rgSort[AY_OFFS_HIST_MAX_IDX], 3uL, 1uL);

            /* Limit confidence to avoid high status offset calibration */
            pOffsAy->Est.Conf = AY_SWA_OFFS_LOW_CONF;
        } else if (fABS(median - hist->Range[maxIdx]) < AY_OFFS_DIFF_MAX_MED) {
            /* Maximum distance count is not not much above the mean distance
               count, but median and mode are close to together,
               this can occur in case of large offset values, where histogram
               data is unsymmetric because one-sided truncation */
            float32 meanCenter;

            meanCenter = VED_HistCalcMeanCenter(
                hist, rgSort[AY_OFFS_HIST_MAX_IDX], 10uL, 1uL);

            /* Calculate mean around maximum peak */
            if (fABS(meanCenter - median) < AY_OFFS_DIFF_MEAN_MED) {
                /*  Use median as estimated offset */
                pOffsAy->Est.Offs = median;

                /* Maximum value looks like a outlier, use median value instead
                 */
                pOffsAy->Est.Conf = AY_SWA_OFFS_LOW_CONF;
            }
        } else {
            /* two peaks with rougly the same distance and ambiguous route
             * characteristics */

            /* Estimation has no confidence */
            pOffsAy->Est.Conf = AY_SWA_OFFS_NO_CONF;
        }
    }
    return;
}

/* **********************************************************************
  @fn                     VED_SwaOffsCheckandTakeOver */ /*!
  @brief                  Take new steering wheel offset value if all conditions are met

  @description            New offset value is used if confidence is ok
                          Threshold and difference for new state are updated
                          Set fault flag if range and confidence were not good
                          enough for several attempts

  @param[in,out]          swaOffs
  @param[out]             -
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_SwaOffsCheckandTakeOver(VED_SwaOffsData_t *swaOffs) {
    /* Check confidence and range of estimation value */
    if (VED_SwaCheckConfidence(swaOffs) != FALSE) {
        /* Validate offset value and use if applicable */
        VED_SwaTakeoverOffset(swaOffs, &swaOffs->Est);

        /* Update distance threshold for offset calibration */
        swaOffs->Est.ThrldDist = VED_SwaEvalThrdDist(swaOffs->OffsState);

        /* Update deviation threshold for offset calibration */
        swaOffs->Est.ThrldDev = VED_SwaEvalThrdDeviation(swaOffs->OffsState);

        /* Reset ReInit counter */
        swaOffs->ReInitCntr = 0U;
    } else {
        /* Estimated value is not usable as offset */
        VED_HistReInit(&swaOffs->Est.Hist);

        if ((VED_SwaOffsetRangeOk(swaOffs->Est.Offs) == FALSE) &&
            (swaOffs->Est.Conf > AY_SWA_OFFS_NO_CONF)) {
            /* Inc ReInit counter */
            swaOffs->ReInitCntr++;
        }
    }

    /* if ReInit counter is above parameter value (3) set the error flag */
    if (swaOffs->ReInitCntr >= VED__PAR_ERR_SWA_REINIT) {
        SWA_GET_ERR_OFFS_RG = (VED_ERR_STATE_ACTIVE);
        /* Limit ReInit counter */
        swaOffs->ReInitCntr = VED__PAR_ERR_SWA_REINIT;
    } else {
        SWA_GET_ERR_OFFS_RG = (VED_ERR_STATE_INACTIVE);
    }
    return;
}

/* **********************************************************************
  @fn                     VED_AyTakeoverOffset */ /*!
  @brief                  Take lateral acceleration offset as active offset 

  @description            Calculates the new offset value based on value difference
                          and learning state
                          Increases or decreases leraning state according to difference
                          and reduces the measurement data

  @param[in]              estOffsAy
  @param[out]             offsAy
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_AyTakeoverOffset(VED_AyOffsData_t *offsAy,
                                 VED_AyOffsEst_t *estOffsAy) {
    const float32 Weightn_c =
        0.7F;              /* Weight for new offset estimation max. 1.0 */
    float32 AyOffsDiff;    /* Difference between new and old offset */
    float32 AyOffsAbsDiff; /* Absolute difference between new and old offset */
    float32 AyOffsNew;     /* New offset value */

    /* Differences between active and new estimated offset */
    AyOffsDiff = estOffsAy->Offs - offsAy->AyOffset;
    AyOffsAbsDiff = fABS(AyOffsDiff);

    /* No active offset available (the estimated offset is the first calculated
     * offset)? */
    if (offsAy->OffsState == AY_STATE_DEFAULT) {
        /* Use estimated offset as first offset. no appoximation with active
         * offset (there is no active offset available) */
        AyOffsNew = estOffsAy->Offs;
    } else {
        /* Calculate average between active and estimated offset for successive
         * approximation of final offset */
        AyOffsNew = offsAy->AyOffset + (Weightn_c * AyOffsDiff);
    }

    /* Acquisition strategy is dependent on current state */
    switch (offsAy->OffsState) {
        case AY_STATE_DEFAULT:
        case AY_STATE_1:
        case AY_STATE_2:

            /*<<<<<<<< L E A R N   P H A S E >>>>>>>>*/

            if (offsAy->Interims == FALSE) {
                /* When offset value is stable, increment state  */
                if (AyOffsAbsDiff <= AY_OFFS_STAT_DIFF_RUN_UP) {
                    offsAy->OvrTakeCntr++;
                    if (offsAy->OvrTakeCntr < AY_OFFS_TAKE_OVR_MAX) {
                        offsAy->OffsState += 1;
                    }
                }

                /* Save offset value and status in nonvolatile memory */
                VED_AyPutNvmOffsetData(AyOffsNew, offsAy->OffsState);

                /* Initialize estimation data */
                VED_HistReInit(&estOffsAy->Hist);
            }

            /* Take over offset and deviation */
            offsAy->AyOffset = AyOffsNew;
            offsAy->Dev = VED_AyGetOffsetDeviation(offsAy->OffsState);

            break;

        case AY_STATE_3:

            /*<<<<<<<< R E L E A R N   P H A S E >>>>>>>>*/

            /* Test offset against stability */
            if (AyOffsAbsDiff >= AY_OFFS_STAT_DIFF_LEARNED) {
                /* Estimated offset differs significantly from active offset:
                 * restore learned offset value */

                /* Remain in this offset calibration state */
                offsAy->OffsState = AY_STATE_2;

                /* Take over offset */
                offsAy->AyOffset = AyOffsNew;
                offsAy->Dev = VED_AyGetOffsetDeviation(offsAy->OffsState);

                if (offsAy->Interims == FALSE) {
                    /* Initialize estimation data */
                    VED_HistReInit(&estOffsAy->Hist);
                }
            } else {
                /* Estimated offset approves active and stored offset */

                if (offsAy->Interims == FALSE) {
                    /* Imcrement offset state */
                    offsAy->OffsState = AY_STATE_3;

                    /* Take over offset */
                    offsAy->AyOffset = AyOffsNew;
                    offsAy->Dev = VED_AyGetOffsetDeviation(offsAy->OffsState);

                    /* Indicate that offset value has been confirmed during this
                       ignition cylce, i.e .one
                       confirmation per ignition cylce */
                    offsAy->OvrTakeCntr++;

                    /* Scale back estimation data */
                    VED_HistReduce(&estOffsAy->Hist, AY_INTERVAL_RDCT_FCTR);
                }
            }

            if (offsAy->Interims == FALSE) {
                /* Save offset value and status in nonvolatile memory */
                VED_AyPutNvmOffsetData(offsAy->AyOffset, offsAy->OffsState);
            }

            break;

        default:

            /* This state is not regular */

            /* Set default state */
            offsAy->OffsState = AY_STATE_DEFAULT;
            offsAy->Dev = VED_AyGetOffsetDeviation(offsAy->OffsState);

            /* Initialize estimation data */
            VED_HistReInit(&estOffsAy->Hist);

            break;
    }

    return;
}

/* **********************************************************************
  @fn                     VED_AyOffsCheckandTakeOver */ /*!
  @brief                  Take new lat accel offset value if all conditions are met

  @description            New offset value is used if confidence is ok
                          Threshold and difference for new state are updated
                          if requested by flag

  @param[in]              payOffs
  @param[in]              clrData
  @param[out]             pswaOffs
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_AyOffsCheckandTakeOver(VED_AyOffsData_t *payOffs,
                                       const VED_SwaOffsData_t *pswaOffs,
                                       boolean clrData) {
    if ((VED_AyCheckConfidence(payOffs) != FALSE) &&
        (pswaOffs->Est.Conf > 0.F)) {
        /* Validate offset value and use if applicable */
        VED_AyTakeoverOffset(payOffs, &payOffs->Est);

        if (clrData != FALSE) {
            /* Update distance threshold for offset calibration */
            payOffs->Est.ThrldDist = VED_AyEvalThrdDist(payOffs->OffsState);

            /* Update deviation threshold for offset calibration */
            payOffs->Est.ThrldDev =
                VED_SwaEvalThrdDeviation(payOffs->OffsState);
        }
    } else {
        if (clrData != FALSE) {
            /* Estimated value is not usable as offset */
            VED_HistReInit(&payOffs->Est.Hist);
        }
    }
    return;
}

/* **********************************************************************
  @fn                     VED_AySwaCalcOffset */ /*!
  @brief                  Calculate offset

  @description            Works for both steering angle and lateral acceleration offset
                          Updates histogram with new data
                          Deletes data in case of circular driving
                          If data is accumulated over necessary distance, the histograms
                          are checked and evaluated
                          If necessary distance is not driven yet, an interim offset of the
                          lateral acceleration is calculated, the data is not cleared
                          in that case

  @param[in]              swaSen
  @param[in]              aySen
  @param[in,out]          swaOffs
  @param[in,out]          ayOffs
  @return                 void
  
  @pre                    -
  @post                   -
**************************************************************************** */
static void VED_AySwaCalcOffset(const VED_SwaSenData_t *swaSen,
                                const VED_AySenData_t *aySen,
                                VED_SwaOffsData_t *swaOffs,
                                VED_AyOffsData_t *ayOffs) {
    float32 distCycle;

    /* Calculate distance driven in this cycle */
    distCycle =
        VED_CalcCycleDistance(SWA_GET_MIF_DATA->LongMot.VehVelocityCorr);

    /* Acquire data and store in histogram for offset estimation */
    VED_AySwaAcqEstData(swaSen, aySen, swaOffs, ayOffs, distCycle);

    /* Detect constant circular driving */
    if (fABS(swaOffs->LatAccel) > SWA_OFFS_LAT_ACC_MAX) {
        /* Reset learn values due to circular driving */
        VED_HistReInit(&swaOffs->Est.Hist);
        VED_HistReInit(&ayOffs->Est.Hist);

        /* Reset integrated lateral acceleration */
        swaOffs->LatAccel = 0.F;
    }

    /* Estimation requires a driven route with minimum distance */
    if (swaOffs->Est.Hist.Sum >= swaOffs->Est.ThrldDist) {
        /* Signal no interims lateral acceleration offset calibration */
        ayOffs->Interims = FALSE;

        /* Estimate and check reliability of offset */
        VED_SwaCalcConfidence(swaOffs);
        if (ayOffs->Est.Hist.Sum >= ayOffs->Est.ThrldDist) {
            VED_AyCalcConfidence(ayOffs);
        }

        /* In case of sufficient confidence take over offset */
        VED_AyOffsCheckandTakeOver(ayOffs, swaOffs, TRUE);
        VED_SwaOffsCheckandTakeOver(swaOffs);
    } else {
        if (ayOffs->Est.Hist.Sum >= ayOffs->Est.ThrldDist) {
            const float32 incDist_c =
                10000.F; /* Additional distance after interims offset */

            /* Signal interims lateral acceleration offset */
            ayOffs->Interims = TRUE;

            /* Estimate and check reliability, of offset */
            if (swaOffs->Est.Hist.Sum >= swaOffs->Est.ThrldDist) {
                VED_SwaCalcConfidence(swaOffs);
            }
            VED_AyCalcConfidence(ayOffs);

            /* In case of sufficient confidence take over offset and don't reset
             * previous data */
            VED_AyOffsCheckandTakeOver(ayOffs, swaOffs, FALSE);

            /* Increase distance threhold */
            ayOffs->Est.ThrldDist += incDist_c;
        }
    }
    return;
}

/* **********************************************************************
  @fn                     VED_AyGetOffset */ /*!
  @brief                  Get lat accel offset from NVM and check for plausibility

  @description            see brief description

  @param[in]              -
  @param[out]             Offset (offset value)
  @param[out]             Status (learning state)
  @param[out]             Dev    (standard deviation)
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
static boolean VED_AyGetOffset(float32 *Offset, sint32 *Status, float32 *Dev) {
    boolean AyOffsOk = FALSE;

    if (VED_AyGetNvmOffsetData(Offset, Status, Dev) != FALSE) {
        /* NVM read operation successfully completed */
        if ((VED_AyOffsetRangeOk(*Offset) == FALSE) ||
            ((*Status < AY_STATE_DEFAULT) || (*Status > AY_STATE_3))) {
            /* Stored offset value is out of range */

            /* Indicate error state active "false value stored in nvmemory" */
            AY_GET_ERR_OFFS_RG = (VED_ERR_STATE_ACTIVE);

            *Offset = AY_OFFS_DEFAULT;
            *Status = AY_STATE_DEFAULT;
            *Dev = VED_AyGetOffsetDeviation(*Status);

            /* Try to correct invalid values in nv memory by overwriting with
             * defaults */
            VED_AyPutNvmOffsetData(*Offset, *Status);
        } else {
            /* Stored swa nvmem values are OK! */

            /* Indicate error state inactive "false value stored in nvmemory" */
            AY_GET_ERR_OFFS_RG = (VED_ERR_STATE_INACTIVE);
        }

        /* Daten wurden aus Eeprom-Mirror gelesen */
        AyOffsOk = TRUE;
    }
    return (AyOffsOk);
}

/* *************************************************************************
  @fn             VED_CalcFastSwaOffset */
void VED_CalcFastSwaOffset(const VED_InputData_t *p_InputData) {
    float32 f_LongAccel;
    float32 f_EgoVel;
    float32 f_SteeringAngle;

    float32 f_SelfSteerGrad;
    float32 f_WheelBase;
    float32 f_SteeringRatio;

    float32 f_WhlVelFrDiff;
    float32 f_WhlVelReDiff;
    float32 f_WhlVelFrDiffFiltAbs;
    float32 f_WhlVelReDiffFiltAbs;

    float32 f_yaw_gye;
    float32 f_yaw_sye;

    VED_SwaData_t *p_SwaData =
        SWA_GET_ME; /* get the steering angle offset data */

    /* if new estimation cycle was started, delete short term data */
    if (p_SwaData->FastSwaOffset.u_Number >= SWA_FAST_OFFS_SAMPLE_LIMIT) {
        p_SwaData->FastSwaOffset.f_Sum = 0.0F;
        p_SwaData->FastSwaOffset.u_Number = 0U;
        p_SwaData->FastSwaOffset.f_Mean = 0.0F;
    }

    /* get signals */
    f_yaw_gye = ved__internal_data.ved__gye_out.gier_yaw_rate;
    f_yaw_sye = ved__internal_data.ved__sye_out.stw_yaw_rate;
    f_EgoVel = ved__internal_data.ved__ve_out.veh_velo;
    f_LongAccel = fABS(ved__internal_data.ved__ve_out.veh_accel);
    f_SteeringAngle = fABS(p_InputData->Signals.StWheelAngle);

    /* get parameters */
    f_SelfSteerGrad = p_InputData->Parameter.VED_Kf_SelfSteerGrad_nu;
    f_WheelBase = p_InputData->Parameter.VED_Kf_WheelBase_met;
    f_SteeringRatio = p_InputData->Parameter.SteeringRatio.swa.rat[1];

    /* get wheel information */
    f_WhlVelFrDiff =
        p_InputData->Signals.WhlVelFrRight - p_InputData->Signals.WhlVelFrLeft;
    f_WhlVelReDiff =
        p_InputData->Signals.WhlVelReRight - p_InputData->Signals.WhlVelReLeft;

    /* filter wheel speed differences */
    p_SwaData->FastSwaOffset.f_WhlVelFrDiffFilt = VED_FilterCycleTime(
        f_WhlVelFrDiff, p_SwaData->FastSwaOffset.f_WhlVelFrDiffFilt,
        SWA_OFFS_WHS_DIFF_FT);
    p_SwaData->FastSwaOffset.f_WhlVelReDiffFilt = VED_FilterCycleTime(
        f_WhlVelReDiff, p_SwaData->FastSwaOffset.f_WhlVelReDiffFilt,
        SWA_OFFS_WHS_DIFF_FT);

    /* get absolute values for test locally as the signed values are needed in
     * the filter in the next cycle */
    f_WhlVelFrDiffFiltAbs = fABS(p_SwaData->FastSwaOffset.f_WhlVelFrDiffFilt);
    f_WhlVelReDiffFiltAbs = fABS(p_SwaData->FastSwaOffset.f_WhlVelReDiffFilt);

    /* only calculate offset if all conditions are met */
    if ((f_LongAccel < SWA_FAST_OFFS_LONG_ACC_MAX) &&
        (f_SteeringAngle < SWA_OFFS_ANGLE_FAST_OFF_MAX) &&
        (f_EgoVel >
         SWA_FAST_OFFS_VEL_MIN) /* input signals showing static situation? */
        && (f_WhlVelFrDiffFiltAbs < SWA_FAST_OFFS_WHS_DIFF_VEL) &&
        (f_WhlVelReDiffFiltAbs <
         SWA_FAST_OFFS_WHS_DIFF_VEL) /* wheels show straight driving? */
        && (VED_GET_IO_STATE(VED_SIN_POS_SWA, p_InputData->Signals.State) ==
            VED_IO_STATE_VALID)) /* steering wheel input signal is valid? */
    {
        float32 f_StAngle_gye;
        float32 f_StAngle_sye;
        float32 f_StAngleOffset;
        /* calculate the steering angle offset */
        float32 f_Temp =
            f_EgoVel /
            (f_SteeringRatio *
             (f_WheelBase + (f_SelfSteerGrad * (f_EgoVel * f_EgoVel))));
        f_StAngle_gye = f_yaw_gye / f_Temp;
        f_StAngle_sye = f_yaw_sye / f_Temp;
        f_StAngleOffset = f_StAngle_sye - f_StAngle_gye;

        /* collect offset data */
        p_SwaData->FastSwaOffset.f_CurStOffset = f_StAngleOffset;
        p_SwaData->FastSwaOffset.f_Sum += f_StAngleOffset;
        p_SwaData->FastSwaOffset.u_Number += 1;
        p_SwaData->FastSwaOffset.f_Mean =
            p_SwaData->FastSwaOffset.f_Sum /
            (float32)p_SwaData->FastSwaOffset.u_Number;

        /* are necessary samples collected? */
        if (p_SwaData->FastSwaOffset.u_Number >= SWA_FAST_OFFS_SAMPLE_LIMIT) {
            /* calculate long term data */
            p_SwaData->FastSwaOffset.f_Offset = p_SwaData->FastSwaOffset.f_Mean;
            p_SwaData->FastSwaOffset.f_OffsetSumLt +=
                p_SwaData->FastSwaOffset.f_Offset;
            p_SwaData->FastSwaOffset.u_OffsetNumberLt += 1U;
            p_SwaData->FastSwaOffset.f_OffsetMeanLt =
                p_SwaData->FastSwaOffset.f_OffsetSumLt /
                (float32)p_SwaData->FastSwaOffset.u_OffsetNumberLt;

            /* limit long term data to prevent overflow */
            if (p_SwaData->FastSwaOffset.u_OffsetNumberLt >
                SWA_FAST_OFFS_SAMPLE_LIMIT_LT) {
                /* reduce long term data */
                p_SwaData->FastSwaOffset.f_OffsetSumLt /=
                    (float32)SWA_FAST_OFFS_REDUCTION_FACT;
                p_SwaData->FastSwaOffset.u_OffsetNumberLt /=
                    (uint16)SWA_FAST_OFFS_REDUCTION_FACT;
            }

            /* save offset as first calculated offset - only one time if state
             * signals no offset available yet */
            if ((p_SwaData->FastSwaOffset.u_OffsetNumberLt <
                 SWA_FAST_OFFS_NVM_LIMIT) &&
                (p_SwaData->Offset.OffsState < SWA_STATE_1)) {
                p_SwaData->Offset.Est.Offs =
                    p_SwaData->FastSwaOffset.f_OffsetMeanLt;
                p_SwaData->Offset.Est.Conf = AY_SWA_OFFS_LOW_CONF;

                /* use offset and store data in nonvolatile memory */
                VED_SwaOffsCheckandTakeOver(&p_SwaData->Offset);
            }
        }
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */