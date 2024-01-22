/**********************************Model Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : LDPSA
 *
 * Model Long Name     : Lane Departure Prevention

 *

 * Model Advisor       : Not Check

 *

 * Model Version       : Ver_02

 *

 * Model Author        : WJ

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    : 50ms


 ************************************Auto Coder**********************************
 *
 * File                             : LDPSA.c
 *
 * FileType                         : Code Source File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Sun Jan 15 16:16:18 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#include "LDPSA.h"
#include "LDPSA_private.h"
#include "look1_iflf_binlxpw.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */

#define CAL_START_CODE
#include "Mem_Map.h"
/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile boolean_T LDPDT_CstruSiteLDP_C_B = 0;/* Referenced by:
                                                     * '<S12>/V_Parameter7'
                                                     * '<S13>/V_Parameter7'
                                                     */

/* Switch of consturction side */
const volatile uint8_T LDPDT_CurveInner_C_St = 1U;/* Referenced by:
                                                   * '<S11>/V_Parameter11'
                                                   * '<S11>/V_Parameter2'
                                                   * '<S78>/V_Parameter2'
                                                   * '<S78>/V_Parameter3'
                                                   */

/* Constant of inner curve lane */
const volatile uint8_T LDPDT_CurveNone_C_St = 0U;/* Referenced by:
                                                  * '<S11>/V_Parameter13'
                                                  * '<S11>/V_Parameter5'
                                                  */

/* Constant of Straight lane */
const volatile uint8_T LDPDT_CurveOuter_C_St = 2U;/* Referenced by:
                                                   * '<S11>/V_Parameter12'
                                                   * '<S11>/V_Parameter4'
                                                   * '<S78>/V_Parameter1'
                                                   * '<S78>/V_Parameter4'
                                                   */

/* Constant of Outer curve lane */
const volatile real32_T LDPDT_CurveThd_C_St = 0.0F;/* Referenced by: '<S11>/V_Parameter1' */

/* Curve threshold of lane */
const volatile uint16_T LDPDT_LnIvldCclLf_C_St = 15U;
                                      /* Referenced by: '<S12>/V_Parameter13' */

/* Invalid cancel state of left lane
   ctrl for 4095
   safety for 15 */
const volatile uint16_T LDPDT_LnIvldCclRi_C_St = 15U;/* Referenced by: '<S13>/V_Parameter2' */

/* Invalid cancel state of right lane */
const volatile uint16_T LDPDT_LnIvldLf_C_St = 15U;/* Referenced by: '<S12>/V_Parameter4' */

/* Invalid state of left lane
   ctrl for 20479
   safety for 15 */
const volatile uint16_T LDPDT_LnIvldRi_C_St = 15U;/* Referenced by: '<S13>/V_Parameter4' */

/* Invalid state of right lane */
const volatile uint8_T LDPDT_LnIvldSfLf_C_St = 15U;/* Referenced by: '<S12>/V_Parameter3' */

/* Invalid safety state of left lane */
const volatile uint8_T LDPDT_LnIvldSfRi_C_St = 15U;/* Referenced by: '<S13>/V_Parameter3' */

/* Invalid safety state of right lane */
const volatile uint8_T LDPDT_NoDgrSide_C_St = 0U;/* Referenced by: '<S8>/V_Parameter' */

/* State value of no danger of lane */
const volatile boolean_T LDPDT_SfFcLDPOn_C_B = 0;/* Referenced by:
                                                  * '<S8>/V_Parameter1'
                                                  * '<S12>/V_Parameter1'
                                                  * '<S13>/V_Parameter1'
                                                  */

/* LDP switch of safety face */
const volatile real32_T LDPDT_TLCHeadAglTrsd_C_Rad = 0.0F;/* Referenced by:
                                                           * '<S10>/V_Parameter1'
                                                           * '<S10>/V_Parameter2'
                                                           */

/* Threshold of heading angle at TLC */
const volatile real32_T LDPDT_TrsdHeadAglMn_C_Rad = -0.03F;/* Referenced by:
                                                            * '<S12>/V_Parameter11'
                                                            * '<S13>/V_Parameter11'
                                                            */

/* Minimum threshold of heading angle */
const volatile real32_T LDPDT_TrsdHeadAglMx_C_Rad = 0.15F;/* Referenced by:
                                                           * '<S12>/V_Parameter10'
                                                           * '<S12>/V_Parameter8'
                                                           * '<S13>/V_Parameter10'
                                                           * '<S13>/V_Parameter8'
                                                           */

/* Maximum threshold of heading angle */
const volatile real32_T LDPDT_TrsdHeadAglOfst_C_Rad = 0.002F;/* Referenced by:
                                                              * '<S12>/V_Parameter12'
                                                              * '<S12>/V_Parameter2'
                                                              * '<S13>/V_Parameter12'
                                                              * '<S13>/V_Parameter5'
                                                              */

/* Offset of heading angle threshold */
const volatile real32_T LDPDT_TrsdLnCltdCurvLfMx_Cr_Mps[8] = { 0.008F, 0.008F,
  0.007F, 0.007F, 0.006F, 0.006F, 0.005F, 0.005F } ;
                                   /* Referenced by: '<S12>/1-D Lookup Table' */

/* Curve of maximum threshold of left clothiod curvature */
const volatile real32_T LDPDT_TrsdLnCltdCurvLfOfst_Cr_Mps[8] = { 0.001F, 0.001F,
  0.001F, 0.001F, 0.001F, 0.001F, 0.001F, 0.001F } ;
                                  /* Referenced by: '<S12>/1-D Lookup Table1' */

/* Curve of offset threshold of left clothiod curvature */
const volatile real32_T LDPDT_TrsdLnCltdCurvRiMx_Cr_Mps[8] = { 0.008F, 0.008F,
  0.007F, 0.007F, 0.006F, 0.006F, 0.005F, 0.005F } ;
                                   /* Referenced by: '<S13>/1-D Lookup Table' */

/* Curve of maximum threshold of rifht clothiod curvature */
const volatile real32_T LDPDT_TrsdLnCltdCurvRiOfst_Cr_Mps[8] = { 0.001F, 0.001F,
  0.001F, 0.001F, 0.001F, 0.001F, 0.001F, 0.001F } ;
                                  /* Referenced by: '<S13>/1-D Lookup Table1' */

/* Curve of offset threshold of right clothiod curvature */
const volatile real32_T LDPDT_VehSpdX_BX_Mps[8] = { 0.0F, 8.33F, 16.66F, 25.0F,
  33.33F, 41.66F, 50.0F, 58.33F } ;    /* Referenced by:
                                        * '<S12>/1-D Lookup Table'
                                        * '<S12>/1-D Lookup Table1'
                                        * '<S13>/1-D Lookup Table'
                                        * '<S13>/1-D Lookup Table1'
                                        */

/* Breakpoint of vehicle speed */
const volatile uint8_T LDPSC_AbtDrvActCtrl_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter11' */

/* Abort state of active control */
const volatile uint8_T LDPSC_AbtDrvIVld_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter10' */

/* Abort state of invalid driver */
const volatile uint8_T LDPSC_AbtErrSpcLDP_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter17' */

/* Abort state of error specific */
const volatile uint8_T LDPSC_AbtFctCstm_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter14' */

/* Abort state of customer specific */
const volatile uint8_T LDPSC_AbtNoAvlbVehSys_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter13' */

/* Abort state of no availible vehicle system signals */
const volatile uint16_T LDPSC_AbtVehIvld_C_St = 0U;/* Referenced by: '<S37>/V_Parameter9' */

/* Abort state of invalid vehicle */
const volatile uint8_T LDPSC_AbtVehSysErr_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter12' */

/* Abort state of vehicle system errors */
const volatile uint8_T LDPSC_CclDrvActCtrl_C_St = 0U;
                                      /* Referenced by: '<S36>/V_Parameter42' */

/* Cancel state of active control */
const volatile uint8_T LDPSC_CclDrvIVld_C_St = 0U;
                                      /* Referenced by: '<S36>/V_Parameter41' */

/* Cancel state of invalid driver */
const volatile uint8_T LDPSC_CclErrSpcLDP_C_St = 0U;
                                      /* Referenced by: '<S36>/V_Parameter46' */

/* Cancel state of error specific */
const volatile uint8_T LDPSC_CclFctCstm_St = 0U;
                                      /* Referenced by: '<S36>/V_Parameter45' */

/* Cancel state of customer specific */
const volatile uint8_T LDPSC_CclNoAvlbVehSys_C_St = 0U;
                                      /* Referenced by: '<S36>/V_Parameter44' */

/* Cancel state of no availible vehicle system signals */
const volatile uint16_T LDPSC_CclVehIvld_C_St = 0U;
                                      /* Referenced by: '<S36>/V_Parameter47' */

/* Cancel state of invalid vehicle */
const volatile uint8_T LDPSC_CclVehSysErr_C_St = 0U;
                                      /* Referenced by: '<S36>/V_Parameter43' */

/* Cancel state of vehicle system errors */
const volatile real32_T LDPSC_ContiActiveTiFns_C_Sec = 4.0F;/* Referenced by:
                                                             * '<S37>/V_Parameter4'
                                                             * '<S37>/V_Parameter6'
                                                             */

/* Maximum time of warming state */
const volatile real32_T LDPSC_ContinWarmSupp_C_Sec = 60.0F;/* Referenced by:
                                                            * '<S79>/V_Parameter3'
                                                            * '<S80>/V_Parameter3'
                                                            */

/* the time of continous warning suppression */
const volatile real32_T LDPSC_ContinWarmTimes_C_Count = 3.0F;/* Referenced by:
                                                              * '<S79>/V_Parameter4'
                                                              * '<S80>/V_Parameter5'
                                                              */

/* The number of consecutive alarms allowed
 */
const volatile real32_T LDPSC_ContinuActiveTi_C_Sec = 2.0F;/* Referenced by:
                                                            * '<S37>/V_Parameter1'
                                                            * '<S37>/V_Parameter2'
                                                            */

/* Maximum time of warming state */
const volatile real32_T LDPSC_CrvSensiAdvance_BX_ReMi[4] = { 0.002F, 0.004F,
  0.006F, 0.008F } ;              /* Referenced by: '<S78>/1-D Lookup Table9' */

/* Breakpoint of detected right lane curvature */
const volatile real32_T LDPSC_CrvSensiAdvance_BY_Mi[4] = { 0.01F, 0.02F, 0.03F,
  0.04F } ;                       /* Referenced by: '<S78>/1-D Lookup Table9' */

/* Breakpoint of detected right lane curvature */
const volatile real32_T LDPSC_CrvSensiDecay_BX_ReMi[4] = { 0.002F, 0.004F,
  0.006F, 0.008F } ;              /* Referenced by: '<S78>/1-D Lookup Table8' */

/* Breakpoint of detected right lane curvature */
const volatile real32_T LDPSC_CrvSensiDecay_BY_Mi[4] = { 0.01F, 0.02F, 0.03F,
  0.04F } ;                       /* Referenced by: '<S78>/1-D Lookup Table8' */

/* Breakpoint of detected right lane curvature */
const volatile real32_T LDPSC_DTCFctLnWid_Cr_Fct[5] = { 0.5F, 0.6F, 1.0F, 1.0F,
  1.0F } ;                        /* Referenced by: '<S78>/1-D Lookup Table3' */

/* Lane width factor of DTL    */
const volatile real32_T LDPSC_DgrCclOfst_C_Mi = 0.8F;/* Referenced by:
                                                      * '<S38>/V_Parameter1'
                                                      * '<S38>/V_Parameter53'
                                                      */

/* Danger offset distance of cancel state */
const volatile real32_T LDPSC_DgrFnsHeadAng_C_Rad = 0.005F;/* Referenced by:
                                                            * '<S33>/V_Parameter23'
                                                            * '<S33>/V_Parameter38'
                                                            */

/* Danger of heading angle  */
const volatile real32_T LDPSC_DgrFnsOfst_C_Mi = 0.7F;/* Referenced by:
                                                      * '<S33>/V_Parameter19'
                                                      * '<S33>/V_Parameter34'
                                                      */

/* Danger offset distance of finish state */
const volatile real32_T LDPSC_DgrFnsSpdVelLat_C_Mps = 0.2F;/* Referenced by:
                                                            * '<S33>/V_Parameter25'
                                                            * '<S33>/V_Parameter40'
                                                            */

/* Danger of lateral speed */
const volatile uint8_T LDPSC_DgrSideLf_C_St = 1U;/* Referenced by:
                                                  * '<S33>/V_Parameter17'
                                                  * '<S77>/V_Parameter6'
                                                  * '<S108>/V_Parameter32'
                                                  * '<S109>/V_Parameter32'
                                                  * '<S38>/V_Parameter48'
                                                  * '<S39>/V_Parameter56'
                                                  */

/* Constant of left side danger */
const volatile uint8_T LDPSC_DgrSideRi_C_St = 2U;/* Referenced by:
                                                  * '<S33>/V_Parameter32'
                                                  * '<S77>/V_Parameter7'
                                                  * '<S108>/V_Parameter2'
                                                  * '<S109>/V_Parameter1'
                                                  * '<S38>/V_Parameter49'
                                                  * '<S39>/V_Parameter59'
                                                  */

/* Constant of right side danger */
const volatile real32_T LDPSC_DlyTiOfTiToLnMn_C_Sec = 0.2F;/* Referenced by:
                                                            * '<S79>/V_Parameter12'
                                                            * '<S80>/V_Parameter12'
                                                            */

/* Delay time of time to lane crossing */
const volatile real32_T LDPSC_DlyTiTgtFns_C_Sec = 1.5F;/* Referenced by: '<S33>/V_Parameter9' */

/* Delay time of target finish  */
const volatile uint8_T LDPSC_DrvMod2_C_St = 2U;/* Referenced by: '<S78>/V_Parameter9' */

/* Driver control mode of LDP ：2 mode */
const volatile uint8_T LDPSC_DrvMod3_C_St = 3U;
                                      /* Referenced by: '<S78>/V_Parameter10' */

/* Driver control mode of LDP ：3 mode */
const volatile real32_T LDPSC_DstcOfDiscToLnLmtMn_C_Mi = -0.6F;/* Referenced by:
                                                                * '<S79>/V_Parameter14'
                                                                * '<S79>/V_Parameter5'
                                                                * '<S80>/V_Parameter14'
                                                                * '<S80>/V_Parameter4'
                                                                */

/* Minimum distance limiting value of distance to lane crossing */
const volatile real32_T LDPSC_DstcOfTiToLnMn_C_Mi = -0.15F;/* Referenced by:
                                                            * '<S79>/V_Parameter10'
                                                            * '<S80>/V_Parameter10'
                                                            */

/* Minimum distance of distance to lane crossing */
const volatile real32_T LDPSC_DstcOfstSafeSitu_C_Mi = 0.15F;/* Referenced by:
                                                             * '<S79>/V_Parameter13'
                                                             * '<S80>/V_Parameter13'
                                                             */

/* Offset distance of safe situation */
const volatile real32_T LDPSC_DstcToLnTrsdOfstLf_Cr_Mi[17] = { 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F } ;                        /* Referenced by: '<S78>/1-D Lookup Table5' */

/* Curve table of left offset distance to lane crossing threshold      */
const volatile real32_T LDPSC_DstcToLnTrsdOfstRi_Cr_Mi[17] = { 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F } ;                        /* Referenced by: '<S78>/1-D Lookup Table4' */

/* Curve table of right offset distance to lane crossing threshold      */
const volatile real32_T LDPSC_DstcTrsdVehSpdXDTL1_Cr_Mi[9] = { 0.2F, 0.2F, 0.2F,
  0.2F, 0.2F, 0.2F, 0.2F, 0.2F, 0.2F } ;
                                   /* Referenced by: '<S78>/1-D Lookup Table' */

/* Curve table of distance to lane crossing threshold at mode 1     */
const volatile real32_T LDPSC_DstcTrsdVehSpdXDTL2_Cr_Mi[9] = { 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                                  /* Referenced by: '<S78>/1-D Lookup Table1' */

/* Curve table of distance to lane crossing threshold at mode 2     */
const volatile real32_T LDPSC_DstcTrsdVehSpdXDTL3_Cr_Mi[9] = { -0.2F, -0.2F,
  -0.2F, -0.2F, -0.2F, -0.2F, -0.2F, -0.2F, -0.2F } ;
                                  /* Referenced by: '<S78>/1-D Lookup Table2' */

/* Curve table of distance to lane crossing threshold at mode 3     */
const volatile uint8_T LDPSC_ErrCstmCclLf_C_St = 0U;
                                      /* Referenced by: '<S39>/V_Parameter58' */

/* Cancel state of left customer specific */
const volatile uint8_T LDPSC_ErrCstmCclRi_C_St = 0U;
                                      /* Referenced by: '<S39>/V_Parameter61' */

/* Cancel state of right customer specific */
const volatile uint8_T LDPSC_FnsCdsnEn_C_St = 191U;/* Referenced by:
                                                    * '<S33>/V_Parameter28'
                                                    * '<S33>/V_Parameter29'
                                                    * '<S33>/V_Parameter30'
                                                    * '<S33>/V_Parameter31'
                                                    * '<S33>/V_Parameter41'
                                                    * '<S33>/V_Parameter42'
                                                    * '<S33>/V_Parameter43'
                                                    */

/* State switch of finish condition  */
const volatile real32_T LDPSC_FnsDuraMn_C_Sec = 0.5F;
                                      /* Referenced by: '<S33>/V_Parameter27' */

/* Minimum duration of finish state */
const volatile real32_T LDPSC_HdTiTrigLf_C_Sec = 1.0F;/* Referenced by:
                                                       * '<S79>/V_Parameter15'
                                                       * '<S80>/V_Parameter15'
                                                       */

/* Holding time of left warming trigger */
const volatile real32_T LDPSC_LaneWidth_BX_Mi[5] = { 2.0F, 2.5F, 3.0F, 3.5F,
  4.0F } ;                             /* Referenced by:
                                        * '<S78>/1-D Lookup Table3'
                                        * '<S78>/1-D Lookup Table7'
                                        */

/* breakpoint of lane width */
const volatile real32_T LDPSC_LnDectCrvLf_BX_ReMi[17] = { -0.05F, -0.02F, -0.01F,
  -0.008F, -0.005F, -0.002F, -0.001F, -0.0005F, 0.0F, 0.0005F, 0.001F, 0.002F,
  0.005F, 0.008F, 0.01F, 0.02F, 0.05F } ;
                                  /* Referenced by: '<S78>/1-D Lookup Table5' */

/* Breakpoint of detected left lane curvature */
const volatile real32_T LDPSC_LnDectCrvRi_BX_ReMi[17] = { -0.05F, -0.02F, -0.01F,
  -0.008F, -0.005F, -0.002F, -0.001F, -0.0005F, 0.0F, 0.0005F, 0.001F, 0.002F,
  0.005F, 0.008F, 0.01F, 0.02F, 0.05F } ;
                                  /* Referenced by: '<S78>/1-D Lookup Table4' */

/* Breakpoint of detected right lane curvature */
const volatile real32_T LDPSC_NoDgrCclOfst_C_Mi = 1.0F;/* Referenced by:
                                                        * '<S38>/V_Parameter51'
                                                        * '<S38>/V_Parameter54'
                                                        */

/* No danger offset distance of cancel state */
const volatile real32_T LDPSC_NoDgrFnsHeadAng_C_Rad = 0.005F;/* Referenced by:
                                                              * '<S33>/V_Parameter22'
                                                              * '<S33>/V_Parameter37'
                                                              */

/*  No danger of heading angle      */
const volatile real32_T LDPSC_NoDgrFnsOfst_C_Mi = 0.15F;/* Referenced by:
                                                         * '<S33>/V_Parameter21'
                                                         * '<S33>/V_Parameter36'
                                                         */

/* No danger offset distance of finish state */
const volatile real32_T LDPSC_NoDgrFnsSpdVelLat_C_Mps = 0.2F;/* Referenced by:
                                                              * '<S33>/V_Parameter24'
                                                              * '<S33>/V_Parameter39'
                                                              */

/* No danger of lateral speed */
const volatile uint8_T LDPSC_NoDgrSide_C_St = 0U;/* Referenced by:
                                                  * '<S77>/V_Parameter4'
                                                  * '<S77>/V_Parameter5'
                                                  */

/* Constant of no danger */
const volatile uint8_T LDPSC_PrjSpecQu_C_St = 0U;/* Referenced by:
                                                  * '<S79>/V_Parameter'
                                                  * '<S80>/V_Parameter'
                                                  */
const volatile real32_T LDPSC_SafetyFuncMaxTime_sec = 0.05F;/* Referenced by:
                                                             * '<S111>/V_Parameter1'
                                                             * '<S112>/V_Parameter1'
                                                             */

/* safety function active or error maximum */
const volatile uint8_T LDPSC_SidCdtnCclLf_C_St = 1U;
                                      /* Referenced by: '<S39>/V_Parameter57' */

/* Cancel constant of left side condition */
const volatile uint8_T LDPSC_SidCdtnCclRi_C_St = 1U;
                                      /* Referenced by: '<S39>/V_Parameter60' */

/* Cancel constant of right side condition */
const volatile uint8_T LDPSC_StrgRdyDrvActCtrl_C_St = 45U;
                                      /* Referenced by: '<S37>/V_Parameter16' */

/* Strong ready state of active control */
const volatile uint8_T LDPSC_StrgRdyDrvIVld_C_St = 100U;
                                      /* Referenced by: '<S37>/V_Parameter15' */

/* Strong ready state of invalid driver */
const volatile uint8_T LDPSC_StrgRdyErrSpcLDP_C_St = 250U;
                                      /* Referenced by: '<S37>/V_Parameter21' */

/* Strong ready state of error specific */
const volatile uint8_T LDPSC_StrgRdyFctCstm_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter20' */

/* Strong ready state of customer specific */
const volatile uint8_T LDPSC_StrgRdyNoAvlbVehSys_C_St = 15U;
                                      /* Referenced by: '<S37>/V_Parameter19' */

/* Strong ready state of no availible vehicle system signals */
const volatile uint16_T LDPSC_StrgRdyVehIvld_C_St = 2047U;
                                      /* Referenced by: '<S37>/V_Parameter22' */

/* Strong ready state of invalid vehicle */
const volatile uint8_T LDPSC_StrgRdyVehSysErr_C_St = 11U;
                                      /* Referenced by: '<S37>/V_Parameter18' */

/* Strong ready state of vehicle system errors */
const volatile uint8_T LDPSC_SuppDrvActCtrl_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter24' */

/* Suppresion state of active control */
const volatile uint8_T LDPSC_SuppDrvIvld_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter23' */

/* Suppresion state of invalid driver */
const volatile uint8_T LDPSC_SuppErrSpcLDP_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter28' */

/* Suppresion state of error specific */
const volatile uint8_T LDPSC_SuppFctCstm_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter27' */

/* Suppresion state of customer specific */
const volatile uint8_T LDPSC_SuppNoAvlbVehSys_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter26' */

/* Suppresion state of no availible vehicle system signals */
const volatile uint16_T LDPSC_SuppVehIvld_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter29' */

/* Suppresion state of invalid vehicle */
const volatile uint8_T LDPSC_SuppVehSysErr_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter25' */

/* Suppresion state of vehicle system errors */
const volatile uint8_T LDPSC_SuppVehicleInvalid_C_St = 127U;
                                     /* Referenced by: '<S113>/V_Parameter28' */

/* Suppresion state of vehicle */
const volatile boolean_T LDPSC_Switch_C_B = 0;
                                      /* Referenced by: '<S25>/V_Parameter11' */

/* Value of LDP disable switch */
const volatile real32_T LDPSC_TgtTrajPstnY_C_Mi = 0.5F;/* Referenced by:
                                                        * '<S33>/V_Parameter18'
                                                        * '<S33>/V_Parameter20'
                                                        * '<S33>/V_Parameter33'
                                                        * '<S33>/V_Parameter35'
                                                        * '<S38>/V_Parameter50'
                                                        * '<S38>/V_Parameter52'
                                                        */

/* Target trajectory lateral position  */
const volatile real32_T LDPSC_TiAbtDegr_C_Sec = 0.1F;
                                      /* Referenced by: '<S32>/V_Parameter63' */

/* Degradation time of abort state */
const volatile real32_T LDPSC_TiCclDegr_C_Sec = 0.1F;/* Referenced by: '<S32>/V_Parameter3' */

/* Degradation time of cancel state */
const volatile real32_T LDPSC_TiDgrFnsDegr_C_Sec = 0.1F;/* Referenced by: '<S32>/V_Parameter2' */

/* Degradation time of finish state */
const volatile real32_T LDPSC_TiStrgRdyDegr_C_Sec = 0.1F;/* Referenced by: '<S32>/V_Parameter1' */

/* Degradation time of no strong ready state */
const volatile real32_T LDPSC_TiToLnTrsdSpd_Cr_Sec[9] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;/* Referenced by: '<S78>/1-D Lookup Table6' */

/* Map table of time to lane crossing threshold */
const volatile real32_T LDPSC_TiToLnTrsdWdh_Cr_Sec[5] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F } ;                        /* Referenced by: '<S78>/1-D Lookup Table7' */

/* Map table of time to lane crossing threshold */
const volatile uint8_T LDPSC_TrigCdtnEn_C_St = 5U;/* Referenced by:
                                                   * '<S79>/V_Parameter11'
                                                   * '<S79>/V_Parameter9'
                                                   * '<S80>/V_Parameter11'
                                                   * '<S80>/V_Parameter9'
                                                   */

/* Switch state of choose threshold  */
const volatile real32_T LDPSC_VehSpdXDTL_BX_Mps[9] = { 0.0F, 40.0F, 50.0F, 60.0F,
  70.0F, 80.0F, 100.0F, 120.0F, 150.0F } ;/* Referenced by:
                                           * '<S78>/1-D Lookup Table'
                                           * '<S78>/1-D Lookup Table1'
                                           * '<S78>/1-D Lookup Table2'
                                           */

/* DTL breakpoint of vehicle speed */
const volatile real32_T LDPSC_VehSpdXTTL_BX_Mps[9] = { 0.0F, 40.0F, 50.0F, 60.0F,
  70.0F, 80.0F, 100.0F, 120.0F, 150.0F } ;
                                  /* Referenced by: '<S78>/1-D Lookup Table6' */

/* TTL breakpoint of vehicle speed */
const volatile real32_T LDPSC_VehYawRateHyst_C_rps = 0.05F;/* Referenced by: '<S115>/Constant10' */

/* Vehicle yaw rate hysteresis */
const volatile real32_T LDPSC_VehYawRateMax_C_rps = 0.25F;/* Referenced by: '<S115>/Constant9' */

/* Vehicle yaw rate maximum */
const volatile real32_T LDPSC_WarmMxTi_C_Sec = 2.0F;/* Referenced by:
                                                     * '<S79>/V_Parameter2'
                                                     * '<S80>/V_Parameter1'
                                                     * '<S40>/V_Parameter1'
                                                     */

/* Maximum time of warming state */
const volatile uint8_T LDPSC_WkRdyDrvActCtrl_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter31' */

/* Weak ready state of active control */
const volatile uint8_T LDPSC_WkRdyDrvIVld_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter30' */

/* Weak ready state of invalid driver */
const volatile uint8_T LDPSC_WkRdyErrSpcLDP_C_St = 5U;
                                      /* Referenced by: '<S37>/V_Parameter35' */

/* Weak ready state of error specific */
const volatile uint8_T LDPSC_WkRdyFctCstm_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter34' */

/* Weak ready state of customer specific */
const volatile uint8_T LDPSC_WkRdyNoAvlbVehSys_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter33' */

/* Weak ready state of no availible vehicle system signals */
const volatile uint16_T LDPSC_WkRdyVehIvld_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter36' */

/* Weak ready state of invalid vehicle */
const volatile uint8_T LDPSC_WkRdyVehSysErr_C_St = 0U;
                                      /* Referenced by: '<S37>/V_Parameter32' */

/* Weak ready state of vehicle system errors */
const volatile uint8_T LDPTT_CurvInner_C_St = 1U;/* Referenced by:
                                                  * '<S121>/V_Parameter12'
                                                  * '<S121>/V_Parameter2'
                                                  */

/* Constant of inner curve */
const volatile uint8_T LDPTT_CurvOuter_C_St = 2U;/* Referenced by:
                                                  * '<S121>/V_Parameter13'
                                                  * '<S121>/V_Parameter3'
                                                  */

/* Constant of Outer curve */
const volatile uint8_T LDPTT_DgrSideLf_C_St = 1U;/* Referenced by:
                                                  * '<S121>/V_Parameter'
                                                  * '<S128>/V_Parameter'
                                                  */

/* Constant of left side danger */
const volatile uint8_T LDPTT_DgrSideRi_C_St = 2U;
                                      /* Referenced by: '<S121>/V_Parameter1' */

/* Constant of left side danger */
const volatile boolean_T LDPTT_EnLwFilt_C_B = 1;
                                      /* Referenced by: '<S120>/V_Parameter2' */

/* Enable flag for Low-pass filter of target track lane parameters */
const volatile real32_T LDPTT_LnBdryCurvLf_BX_ReMi[6] = { 0.0F, 0.03F, 0.06F,
  0.09F, 0.12F, 0.15F } ;              /* Referenced by:
                                        * '<S121>/Lookup Table'
                                        * '<S121>/Lookup Table2'
                                        * '<S121>/Lookup Table3'
                                        */

/* X-axis for Filtered left lane clothoid curvature */
const volatile real32_T LDPTT_LnBdryCurvRi_BX_ReMi[6] = { 0.0F, 0.03F, 0.06F,
  0.09F, 0.12F, 0.15F } ;            /* Referenced by: '<S121>/Lookup Table5' */

/* X-axis for Filtered left lane clothoid curvature */
const volatile real32_T LDPTT_LnWidCalc_BX_Mi[6] = { 2.5F, 3.0F, 3.5F, 4.0F,
  4.5F, 5.0F } ;                       /* Referenced by:
                                        * '<S121>/Lookup Table1'
                                        * '<S121>/Lookup Table4'
                                        */

/* Calculation ego lane width */
const volatile real32_T LDPTT_MxTgtLatDev_C_Mi = 0.6F;/* Referenced by:
                                                       * '<S121>/V_Parameter10'
                                                       * '<S121>/V_Parameter11'
                                                       * '<S121>/V_Parameter17'
                                                       * '<S121>/V_Parameter7'
                                                       */

/* Maximal allowed distance between the middle of the vehicle and the planned target. */
const volatile real32_T LDPTT_MxTgtLatDstc_C_Mi = 0.6F;/* Referenced by:
                                                        * '<S121>/V_Parameter14'
                                                        * '<S121>/V_Parameter4'
                                                        */

/* Maximal allowed distance between the hazardous lane marking and the planned target. */
const volatile boolean_T LDPTT_TgtCntrLnEn_C_B = 0;/* Referenced by:
                                                    * '<S121>/V_Parameter16'
                                                    * '<S121>/V_Parameter6'
                                                    */

/* Constant of the target in the center of the lane.  */
const volatile real32_T LDPTT_TgtLatDistcLf_Cr_Mi[6] = { 0.25F, 0.3F, 0.55F,
  0.8F, 0.8F, 0.8F } ;               /* Referenced by: '<S121>/Lookup Table1' */

/* Left Target Lateral distance by lane width */
const volatile real32_T LDPTT_TgtLatDistcRi_Cr_Mi[6] = { 0.25F, 0.3F, 0.55F,
  0.8F, 0.8F, 0.8F } ;               /* Referenced by: '<S121>/Lookup Table4' */

/* Left Target Lateral distance by lane width */
const volatile real32_T LDPTT_TgtOfstLfIn_Cr_Mi[6] = { 0.0F, 0.0F, 0.2F, 0.5F,
  0.5F, 0.5F } ;                       /* Referenced by:
                                        * '<S121>/Lookup Table'
                                        * '<S121>/Lookup Table3'
                                        */

/* Left Target Lateral distance Offset By inner curves */
const volatile real32_T LDPTT_TgtOfstLfOut_Cr_Mi[6] = { 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F } ;                     /* Referenced by: '<S121>/Lookup Table2' */

/* Left Target Lateral distance by outer curves */
const volatile real32_T LDPTT_TgtOfstRiOut_Cr_Mi[6] = { 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F } ;                     /* Referenced by: '<S121>/Lookup Table5' */

/* Left Target Lateral distance by outer curves */
const volatile real32_T LDPTT_TiTpLnLw_C_Sec = 0.12F;/* Referenced by:
                                                      * '<S126>/V_Parameter11'
                                                      * '<S127>/V_Parameter11'
                                                      * '<S128>/V_Parameter11'
                                                      */

/* Low-pass filter time of target track lane parameters */
const volatile real32_T LDPTV_AccXObst_C_Mps2 = 0.0F;
                                     /* Referenced by: '<S151>/V_Parameter14' */

/* X-axis acceleration of obstacle */
const volatile uint8_T LDPTV_CurvInner_C_St = 1U;/* Referenced by:
                                                  * '<S157>/V_Parameter3'
                                                  * '<S157>/V_Parameter9'
                                                  */

/* Constant of inner curve */
const volatile uint8_T LDPTV_CurvOuter_C_St = 2U;/* Referenced by:
                                                  * '<S157>/V_Parameter10'
                                                  * '<S157>/V_Parameter5'
                                                  */

/* Constant of Outer curve */
const volatile real32_T LDPTV_CurvScalInner_Cr_Fct[6] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F } ;                 /* Referenced by: '<S150>/1-D Lookup Table5' */

/* Planning Horizon Scaling Factor for the calculation in the TRJPLN Curvature dependant for an inner curve. */
const volatile real32_T LDPTV_CurvScalOuter_Cr_Fct[6] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F } ;                 /* Referenced by: '<S150>/1-D Lookup Table4' */

/* Planning Horizon Scaling Factor for the calculation in the TRJPLN Curvature dependant for an outer curve. */
const volatile real32_T LDPTV_Curv_BX_ReMi[6] = { -1.0F, -0.01F, -0.001F, 0.001F,
  0.01F, 1.0F } ;                      /* Referenced by:
                                        * '<S150>/1-D Lookup Table4'
                                        * '<S150>/1-D Lookup Table5'
                                        */

/* Filtered center lane clothoid curvature for the LDP */
const volatile real32_T LDPTV_D2TPlanHorizonScal_Fct[6] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F } ;           /* Referenced by: '<S150>/1-D Lookup Table2' */

/* Filtered center lane clothoid Y0 position for the LDP */
const volatile uint8_T LDPTV_DgrSideLf_C_St = 1U;/* Referenced by:
                                                  * '<S152>/V_Parameter2'
                                                  * '<S157>/V_Parameter2'
                                                  * '<S157>/V_Parameter4'
                                                  */

/* Constant of left side danger */
const volatile uint8_T LDPTV_DgrSideRi_C_St = 2U;/* Referenced by:
                                                  * '<S152>/V_Parameter7'
                                                  * '<S157>/V_Parameter7'
                                                  * '<S157>/V_Parameter8'
                                                  */

/* Constant of left side danger */
const volatile real32_T LDPTV_DstcXObst_C_Mi = 100.0F;
                                     /* Referenced by: '<S151>/V_Parameter16' */

/* X-axis distance of obstacle */
const volatile real32_T LDPTV_DstcYObst_C_Mi = 100.0F;
                                     /* Referenced by: '<S151>/V_Parameter17' */

/* Y-axis distance of obstacle */
const volatile real32_T LDPTV_DstcYTgtAreaLf_C_Mi = 0.05F;
                                      /* Referenced by: '<S151>/V_Parameter2' */

/* Y-axis distance of host vehicle to left target area */
const volatile real32_T LDPTV_DstcYTgtAreaRi_C_Mi = 0.05F;
                                      /* Referenced by: '<S151>/V_Parameter3' */

/* Y-axis distance of host vehicle to right target area */
const volatile real32_T LDPTV_FTireAccMn_C_Mps2 = 0.0F;
                                      /* Referenced by: '<S151>/V_Parameter1' */
const volatile real32_T LDPTV_FTireAccMx_C_Mps2 = 2.0F;
                                     /* Referenced by: '<S151>/V_Parameter23' */
const volatile real32_T LDPTV_FctTgtDistY_C_Fct = 0.1F;
                                      /* Referenced by: '<S151>/V_Parameter4' */

/* Y-axis distance of obstacle */
const volatile boolean_T LDPTV_HiStatAcc_C_B = 1;
                                      /* Referenced by: '<S148>/V_Parameter5' */

/* Switch for a high stationary accuracy in the LaDMC */
const volatile real32_T LDPTV_JerkLmtMx_C_Mps3 = 2.0F;
                                     /* Referenced by: '<S151>/V_Parameter12' */

/* Maximum Jerk Allowed in the trajectory planning */
const volatile boolean_T LDPTV_LatCpstnEn_C_B = 0;
                                      /* Referenced by: '<S151>/V_Parameter9' */

/* Switch for the latency compensation in trajectory plan */
const volatile real32_T LDPTV_LatVel_BX_Mps[6] = { -20.0F, -15.0F, -5.0F, 5.0F,
  15.0F, 20.0F } ;                     /* Referenced by:
                                        * '<S148>/1-D Lookup Table'
                                        * '<S148>/1-D Lookup Table1'
                                        * '<S148>/1-D Lookup Table2'
                                        * '<S148>/1-D Lookup Table3'
                                        * '<S150>/1-D Lookup Table1'
                                        */

/* Lateral Velocity */
const volatile real32_T LDPTV_LmtCurvGradCtrlMx_C_ReMps = 1.0F;
                                     /* Referenced by: '<S151>/V_Parameter21' */

/* Maximum limiting gradient of curvature */
const volatile real32_T LDPTV_LmtCurvGradDecMx_C_ReMps = 1.0F;
                                     /* Referenced by: '<S151>/V_Parameter20' */

/* Maximum limiting curvature */
const volatile real32_T LDPTV_LmtCurvGradIncMx_C_ReMps = 1.0F;
                                     /* Referenced by: '<S151>/V_Parameter19' */

/* Maximum limiting gradient of curvature */
const volatile real32_T LDPTV_LmtCurvMx_C_ReMi = 1.0F;
                                      /* Referenced by: '<S151>/V_Parameter6' */

/* Maximum limiting curvature */
const volatile boolean_T LDPTV_LmtEn_C_B = 0;
                                     /* Referenced by: '<S151>/V_Parameter10' */

/* Switch to limit the target curvature in trajectory controller */
const volatile real32_T LDPTV_LnBdryPstnYCent_Bx_Mi[6] = { -10.0F, -5.0F, -2.5F,
  2.5F, 5.0F, 10.0F } ;          /* Referenced by: '<S150>/1-D Lookup Table2' */

/* Filtered center lane clothoid Y0 position for the LDP */
const volatile real32_T LDPTV_MxTrqScalGradLmt_C_Fct = 100.0F;
                                      /* Referenced by: '<S148>/V_Parameter6' */

/* Maximum Torque Scaling ramp out gradien */
const volatile real32_T LDPTV_MxTrqScalInGrad_C_Res = 100.0F;
                                      /* Referenced by: '<S148>/V_Parameter7' */

/* Maximum Torque Scaling ramp in gradien */
const volatile real32_T LDPTV_MxTrqScalOutGrad_C_Res = 100.0F;
                                      /* Referenced by: '<S148>/V_Parameter2' */

/* Maximum Torque Scaling ramp out gradien */
const volatile real32_T LDPTV_PredTiAgl_C_Sec = 0.0F;
                                      /* Referenced by: '<S151>/V_Parameter8' */
const volatile real32_T LDPTV_PredTiCurv_C_Sec = 0.0F;
                                      /* Referenced by: '<S151>/V_Parameter7' */
const volatile uint8_T LDPTV_ReqFreeze_C_ST = 3U;
                                      /* Referenced by: '<S149>/V_Parameter3' */

/* Request freeze of LDP state */
const volatile uint8_T LDPTV_ReqOff_C_ST = 0U;
                                      /* Referenced by: '<S149>/V_Parameter5' */

/* Request off of LDP state */
const volatile uint8_T LDPTV_ReqOn_C_ST = 1U;
                                      /* Referenced by: '<S149>/V_Parameter4' */

/* Request on of LDP state */
const volatile real32_T LDPTV_SteWhlGradAbort_C_ReS = 1000.0F;
                                      /* Referenced by: '<S148>/V_Parameter9' */

/* Steering Wheel Stiffness Abort Ramp Out Gradient */
const volatile real32_T LDPTV_SteWhlGradLmt_C_Fct = 100.0F;
                                      /* Referenced by: '<S148>/V_Parameter4' */

/* Steering Wheel Stiffness Limiter */
const volatile real32_T LDPTV_SteWhlGrad_C_ReS = 300.0F;
                                      /* Referenced by: '<S148>/V_Parameter8' */

/* Steering Wheel Stiffness Standard Ramp Out */
const volatile real32_T LDPTV_TiLmtEnDura_C_Sec = 0.0F;
                                     /* Referenced by: '<S151>/V_Parameter11' */

/* Switch to limit the target curvature in trajectory controller */
const volatile real32_T LDPTV_TrajPlanServQu_C_Fct = 8.0F;
                                      /* Referenced by: '<S151>/V_Parameter5' */
const volatile boolean_T LDPTV_TrigReplan_C_B = 1;
                                     /* Referenced by: '<S151>/V_Parameter13' */

/* It has to be 1 for trajectory planning to calculate a trajectory */
const volatile real32_T LDPTV_TrqRampGradIn_C_ReS = 100.0F;
                                     /* Referenced by: '<S148>/V_Parameter12' */

/* Torque Standard Ramp In Gradient */
const volatile real32_T LDPTV_TrqRampOutGradAbort_C_ReS = 1000.0F;
                                     /* Referenced by: '<S148>/V_Parameter11' */

/* Torque Ramp Out Abort Gradient */
const volatile real32_T LDPTV_TrqRampOutGrad_C_ReS = 40.0F;
                                     /* Referenced by: '<S148>/V_Parameter10' */

/* Torque Standard Ramp Out Gradient */
const volatile real32_T LDPTV_VXPlanHorizonScal_Cr_Fct[8] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.5F, 1.5F, 1.5F } ;/* Referenced by: '<S150>/1-D Lookup Table' */

/* Planning Horizon Scaling Factor for the calculation in the TRJPLN.VX dependant. */
const volatile real32_T LDPTV_VYMD1DeratingLevel_Fct[6] = { 100.0F, 100.0F,
  100.0F, 100.0F, 100.0F, 100.0F } ;
                                 /* Referenced by: '<S148>/1-D Lookup Table1' */

/* DMC Derating Level of the LDP function when Driving Mode is equal to 1. Lateral Velocity dependant. */
const volatile real32_T LDPTV_VYMD2DeratingLevel_Fct[6] = { 100.0F, 100.0F,
  100.0F, 100.0F, 100.0F, 100.0F } ;
                                 /* Referenced by: '<S148>/1-D Lookup Table2' */

/* DMC Derating Level of the LDP function when Driving Mode is equal to 2. Lateral Velocity dependant. */
const volatile real32_T LDPTV_VYMD3DeratingLevel_Fct[6] = { 100.0F, 100.0F,
  100.0F, 100.0F, 100.0F, 100.0F } ;
                                 /* Referenced by: '<S148>/1-D Lookup Table3' */

/* DMC Derating Level of the LDP function when Driving Mode is equal to 3. Lateral Velocity dependant. */
const volatile real32_T LDPTV_VYPlanningHorizon_Sec[6] = { 4.0F, 4.0F, 3.5F,
  3.5F, 4.0F, 4.0F } ;           /* Referenced by: '<S150>/1-D Lookup Table1' */

/* Filtered center lane clothoid Y0 position for the LDP */
const volatile real32_T LDPTV_VYStrWhStifRIGrad_Cr_Res[6] = { 150.0F, 150.0F,
  150.0F, 200.0F, 200.0F, 200.0F } ;
                                  /* Referenced by: '<S148>/1-D Lookup Table' */

/* Standard Steering Wheel Stiffness Ramp In Gradient. Vy dependant. */
const volatile real32_T LDPTV_VehVelX_BX_Mps[8] = { 0.0F, 5.55555534F,
  11.1111107F, 16.666666F, 22.2222214F, 27.7777786F, 33.3333321F, 38.8888893F } ;/* Referenced by:
                                                                      * '<S150>/1-D Lookup Table'
                                                                      * '<S150>/1-D Lookup Table3'
                                                                      */

/* Vehicle Speed which is computed based on the wheel speeds */
const volatile real32_T LDPTV_VeloXObst_C_Mps = 0.0F;
                                     /* Referenced by: '<S151>/V_Parameter18' */

/* X-axis velocity of obstacle */
const volatile real32_T LDPTV_WheightEndTi_Cr_Fct[8] = { 0.01F, 0.01F, 0.01F,
  0.01F, 0.01F, 0.01F, 0.01F, 0.01F } ;
                                 /* Referenced by: '<S150>/1-D Lookup Table3' */

/* Weight of the end time for the calculation in the TRJPLN. Speed dependant */
const volatile real32_T LDPTV_WidObst_C_Mi = 0.0F;
                                     /* Referenced by: '<S151>/V_Parameter15' */

/* Width of obstacle */
const volatile real32_T LDPVSE_HodTiTrnSgl_C_Sec = 5.0F;/* Referenced by:
                                                         * '<S161>/V_Parameter3'
                                                         * '<S161>/V_Parameter7'
                                                         */

/* Value of turn signal holding time */
const volatile real32_T LDPVSE_LnWidTrsdMn_C_Mi = 2.5F;
                                      /* Referenced by: '<S160>/V_Parameter8' */

/* Minimum threshold of lane width */
const volatile real32_T LDPVSE_LnWidTrsdMx_C_Mi = 6.5F;/* Referenced by:
                                                        * '<S160>/V_Parameter20'
                                                        * '<S160>/V_Parameter7'
                                                        */

/* Maximum threshold of lane width */
const volatile real32_T LDPVSE_LnWidTrsdOfst_C_Mi = 0.1F;/* Referenced by:
                                                          * '<S160>/V_Parameter21'
                                                          * '<S160>/V_Parameter9'
                                                          */

/* Offset of lane width */
const volatile uint8_T LDPVSE_NoDgrSide_C_St = 0U;/* Referenced by: '<S162>/V_Parameter' */

/* State value of no danger of lane */
const volatile real32_T LDPVSE_SteAglSpdTrsdMx_C_Dgpm = 120.0F;
                                      /* Referenced by: '<S160>/V_Parameter5' */

/* Maximum threshold of steering wheel angle speed */
const volatile real32_T LDPVSE_SteAglSpdTrsdOfst_C_Dgpm = 20.0F;
                                      /* Referenced by: '<S160>/V_Parameter6' */

/* Offset of steering wheel angle speed */
const volatile real32_T LDPVSE_SteAglTrsdMx_C_Dgr = 90.0F;
                                      /* Referenced by: '<S160>/V_Parameter3' */

/* Maximum threshold of steering wheel angle */
const volatile real32_T LDPVSE_SteAglTrsdOfst_C_Dgr = 10.0F;
                                      /* Referenced by: '<S160>/V_Parameter4' */

/* Offset of steering wheel angle */
const volatile uint8_T LDPVSE_TrnSglLf_C_St = 1U;/* Referenced by:
                                                  * '<S161>/V_Parameter1'
                                                  * '<S161>/V_Parameter4'
                                                  */

/* State value of left turn signal  */
const volatile uint8_T LDPVSE_TrnSglRi_C_St = 2U;/* Referenced by:
                                                  * '<S161>/V_Parameter'
                                                  * '<S161>/V_Parameter5'
                                                  */

/* State value of right turn signal  */
const volatile boolean_T LDPVSE_TrnSglRstLfEn_C_B = 1;
                                      /* Referenced by: '<S161>/V_Parameter2' */

/* Enable signal of left turn reset */
const volatile boolean_T LDPVSE_TrnSglRstRiEn_C_B = 1;
                                      /* Referenced by: '<S161>/V_Parameter6' */

/* Enable signal of right turn reset */
const volatile real32_T LDPVSE_TrsdLnCltdCurvMx_Cr_Mps[8] = { 0.01F, 0.01F,
  0.01F, 0.008F, 0.005F, 0.004F, 0.004F, 0.004F } ;
                                      /* Referenced by: '<S160>/Lookup Table' */

/* Curve of maximum threshold of clothiod curvature  */
const volatile real32_T LDPVSE_TrsdLnCltdCurvOfst_Cr_Mps[8] = { 0.005F, 0.005F,
  0.005F, 0.005F, 0.005F, 0.005F, 0.005F, 0.005F } ;
                                     /* Referenced by: '<S160>/Lookup Table1' */

/* Curve of offset threshold of clothiod curvature  */
const volatile real32_T LDPVSE_VehAccSpdTrsdXMn_C_Npkg = -2.95F;
                                     /* Referenced by: '<S160>/V_Parameter11' */

/* Minimum threshold of longitudinal Acceleration */
const volatile real32_T LDPVSE_VehAccSpdTrsdXMx_C_Npkg = 2.95F;
                                     /* Referenced by: '<S160>/V_Parameter10' */

/* Maximum threshold of longitudinal Acceleration */
const volatile real32_T LDPVSE_VehAccSpdTrsdXOfst_C_Npkg = 0.05F;
                                     /* Referenced by: '<S160>/V_Parameter12' */

/* Offset of longitudinal Acceleration */
const volatile real32_T LDPVSE_VehAccSpdTrsdYMx_C_Npkg = 5.0F;
                                     /* Referenced by: '<S160>/V_Parameter13' */

/* Maximum threshold of lateral Acceleration */
const volatile real32_T LDPVSE_VehAccSpdTrsdYOfst_C_Npkg = 0.002F;
                                     /* Referenced by: '<S160>/V_Parameter14' */

/* Offset of lateral Acceleration */
const volatile real32_T LDPVSE_VehLatTrsdLDPMn_C_Msp = -0.2F;/* Referenced by:
                                                              * '<S177>/V_Parameter4'
                                                              * '<S180>/V_Parameter10'
                                                              */

/* Minimum threshold of Lateral vehicle speed */
const volatile real32_T LDPVSE_VehLatTrsdLDPMx_C_Msp = 1.0F;/* Referenced by:
                                                             * '<S177>/V_Parameter1'
                                                             * '<S180>/V_Parameter7'
                                                             */

/* Maximum threshold of Lateral vehicle speed */
const volatile real32_T LDPVSE_VehLatTrsdLDPOfst_C_Msp = 0.1F;/* Referenced by:
                                                               * '<S177>/V_Parameter2'
                                                               * '<S177>/V_Parameter5'
                                                               * '<S180>/V_Parameter11'
                                                               * '<S180>/V_Parameter8'
                                                               */

/* Offset of Lateral vehicle speed */
const volatile real32_T LDPVSE_VehLatTrsd_Cr_Msp[8] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F } ;          /* Referenced by: '<S162>/Lookup Table' */

/* Curve of maximum threshold of lateral velocity */
const volatile real32_T LDPVSE_VehSpdTrsdMn_C_Kmph = 45.0F;
                                      /* Referenced by: '<S171>/V_Parameter1' */

/* Minimum threshold of displayed longitudinal speed */
const volatile real32_T LDPVSE_VehSpdTrsdMx_C_Kmph = 140.0F;/* Referenced by: '<S160>/V_Parameter' */

/* Maximum threshold of displayed longitudinal speed */
const volatile real32_T LDPVSE_VehSpdTrsdOfst_C_Kmph = 3.0F;
                                      /* Referenced by: '<S160>/V_Parameter2' */

/* Offset of displayed longitudinal speed */
const volatile real32_T LDPVSE_VehSpdX_BX_Mps[8] = { 0.0F, 5.55555534F,
  11.1111107F, 16.666666F, 22.2222214F, 27.7777786F, 33.3333321F, 38.8888893F } ;/* Referenced by:
                                                                      * '<S160>/Lookup Table'
                                                                      * '<S160>/Lookup Table1'
                                                                      * '<S162>/Lookup Table'
                                                                      */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h"
/* Named constants for Chart: '<S4>/LDP_State' */
#define LDPSA_IN_LDP_ACTIVE            ((uint8_T)1U)
#define LDPSA_IN_LDP_ERROR             ((uint8_T)1U)
#define LDPSA_IN_LDP_OFF               ((uint8_T)2U)
#define LDPSA_IN_LDP_ON                ((uint8_T)3U)
#define LDPSA_IN_LDP_PASSIVE           ((uint8_T)2U)
#define LDPSA_IN_LDP_STANDBY           ((uint8_T)3U)
#define LDPSA_IN_NO_ACTIVE_CHILD       ((uint8_T)0U)

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Exported block signals */
real32_T LDPTT_LnBdryPstnXLf_Mi;       /* '<S1>/LDP'
                                        * Filtered left lane clothoid X0 position from LDP
                                        */
real32_T LDPTT_LnBdryPstnYLf_Mi;       /* '<S1>/LDP'
                                        * Filtered left lane clothoid Y0 position from LDP
                                        */
real32_T LDPTT_LnBdryHeadAglLf_Rad;    /* '<S1>/LDP'
                                        * Filtered left lane clothoid heading angle from LDP
                                        */
real32_T LDPTT_LnBdryCurvLf_ReMi;      /* '<S1>/LDP'
                                        * Filtered left lane clothoid curvature from LDP
                                        */
real32_T LDPTT_LnBdryCurvRateLf_ReMi2; /* '<S1>/LDP'
                                        * Filtered left lane clothoid change of curvature from LDP
                                        */
real32_T LDPTT_LnBdryVldLengLf_Mi;     /* '<S1>/LDP'
                                        * Filtered left lane clothoid length from LDP
                                        */
real32_T LDPTT_LnBdryPstnXRi_Mi;       /* '<S1>/LDP'
                                        * Filtered right lane clothoid X0 position from LDP
                                        */
real32_T LDPTT_LnBdryPstnYRi_Mi;       /* '<S1>/LDP'
                                        * Filtered right lane clothoid Y0 position from LDP
                                        */
real32_T LDPTT_LnBdryHeadAglRi_Rad;    /* '<S1>/LDP'
                                        * Filtered right lane clothoid heading angle from LDP
                                        */
real32_T LDPTT_LnBdryCurvRi_ReMi;      /* '<S1>/LDP'
                                        * Filtered right lane clothoid curvature from LDP
                                        */
real32_T LDPTT_LnBdryCurvRateRi_ReMi2; /* '<S1>/LDP'
                                        * Filtered right lane clothoid change of curvature from LDP
                                        */
real32_T LDPTT_LnBdryVldLengRi_Mi;     /* '<S1>/LDP'
                                        * Filtered right lane clothoid length from LDP
                                        */
real32_T LDPTT_LnBdryPstnXCent_Mi;     /* '<S1>/LDP'
                                        * Filtered center lane clothoid X0 position from LDP
                                        */
real32_T LDPTT_LnBdryPstnYCent_Mi;     /* '<S1>/LDP'
                                        * Filtered center lane clothoid Y0 position from LDP
                                        */
real32_T LDPTT_LnBdryHeadAglCent_Rad;  /* '<S1>/LDP'
                                        * Filtered center lane clothoid heading angle from LDP
                                        */
real32_T LDPTT_LnBdryCurvCent_ReMi;    /* '<S1>/LDP'
                                        * Filtered center lane clothoid curvature from LDP
                                        */
real32_T LDPTT_LnBdryCurvRateCent_ReMi2;/* '<S1>/LDP'
                                         * Filtered center lane clothoid change of curvature from LDP
                                         */
real32_T LDPTT_LnBdryVldLengCent_Mi;   /* '<S1>/LDP'
                                        * Filtered center lane clothoid length from LDP
                                        */
real32_T LDPTT_TgtPstnYLf_Mi;          /* '<S1>/LDP'
                                        * Lateral distance to the target when a dangerous situation on the left side takes place
                                        */
real32_T LDPTT_TgtPstnYRi_Mi;          /* '<S1>/LDP'
                                        * Lateral distance to the target when a dangerous situation on the right side takes place
                                        */
real32_T LDPTV_FTireAccMx_Mps2;        /* '<S1>/LDP' */
real32_T LDPTV_FTireAccMn_Mps2;        /* '<S1>/LDP' */
real32_T LDPTV_DstcYTgtAreaLf_Mi;      /* '<S1>/LDP'
                                        * Y-axis distance of host vehicle to left target area
                                        */
real32_T LDPTV_DstcYTgtAreaRi_Mi;      /* '<S1>/LDP'
                                        * Y-axis distance of host vehicle to right target area
                                        */
real32_T LDPTV_FctTgtDistY_Fct;        /* '<S1>/LDP' */
real32_T LDPTV_TrajPlanServQu_Fct;     /* '<S1>/LDP' */
real32_T LDPTV_PredTiCurv_Sec;         /* '<S1>/LDP' */
real32_T LDPTV_PredTiAgl_Sec;          /* '<S1>/LDP' */
real32_T LDPTV_TiLmtEnDura_Sec;        /* '<S1>/LDP' */
real32_T LDPTV_JerkLmtMx_Mps3;         /* '<S1>/LDP'
                                        * Maximum Jerk Allowed in the trajectory planning
                                        */
real32_T LDPTV_VeloXObst_Mps;          /* '<S1>/LDP'
                                        * X-axis velocity of obstacle
                                        */
real32_T LDPTV_AccXObst_Mps2;          /* '<S1>/LDP'
                                        * X-axis acceleration of obstacle
                                        */
real32_T LDPTV_DstcXObst_Mi;           /* '<S1>/LDP'
                                        * X-axis distance of obstacle
                                        */
real32_T LDPTV_DstcYObst_Mi;           /* '<S1>/LDP'
                                        * Y-axis distance of obstacle
                                        */
real32_T LDPTV_LmtCurvMx_ReMi;         /* '<S1>/LDP'
                                        * Maximal limiter curvature allowed.
                                        */
real32_T LDPTV_LmtCurvGradIncMx_ReMps; /* '<S1>/LDP'
                                        * Maximal limiter curvature gradient allowed.
                                        */
real32_T LDPTV_LmtCurvGradDecMx_ReMps; /* '<S1>/LDP'
                                        * Maximal limiter curvature gradient allowed.
                                        */
real32_T LDPTV_SnsTiStamp_Sec;         /* '<S1>/LDP'
                                        * Sensor time stamp in seconds
                                        */
real32_T LDPTV_SteWhlGradLmt_Fct;      /* '<S1>/LDP'
                                        * Steering Wheel Stiffness Limiter
                                        */
real32_T LDPTV_SteWhlGrad_ReS;         /* '<S1>/LDP'
                                        * Steering Wheel Stiffness Gradient
                                        */
real32_T LDPTV_TrqRampGrad_ReS;        /* '<S1>/LDP'
                                        * Torque Ramp Gradient
                                        */
real32_T LDPTV_MxTrqScalGradLmt_Fct;   /* '<S1>/LDP'
                                        * Maximum Torque Scaling Limiter (Torque saturation)
                                        */
real32_T LDPTV_MxTrqScalGrad_ReS;      /* '<S1>/LDP'
                                        * Maximum Torque Scaling Gradient
                                        */
real32_T LDPTV_WeightEndTi_Fct;        /* '<S1>/LDP'
                                        * Weight of the end time for the calculation in the TRJPLN
                                        */
real32_T LDPTV_PlanningHorizon_Sec;    /* '<S1>/LDP' */
real32_T LDPTV_DMCDeraLvl_Fct;         /* '<S1>/LDP'
                                        * DMC Derating Level of the LDP function
                                        */
real32_T LDPTV_WidObst_Mi;             /* '<S1>/LDP'
                                        * Width of obstacle
                                        */
real32_T LDPTV_LmtCurvGradCtrlMx_ReMps;/* '<S1>/LDP'
                                        * Maximal limiter curvature gradient allowed.
                                        */
real32_T LDPVSE_NVRAMVehStartupSpd_Kmph;/* '<S1>/LDP' */
real32_T LDPSC_CrvSensiDecayRi_Mi;     /* '<S78>/Switch6' */
real32_T LDPDT_CrvThdMaxRi_ReMi;       /* '<S13>/1-D Lookup Table'
                                        * Curve of maximum threshold of right clothiod curvature
                                        */
real32_T LDPDT_CrvThdHystRi_ReMi;      /* '<S13>/1-D Lookup Table1'
                                        * Curve of offset threshold of right clothiod curvature
                                        */
real32_T LDPDT_LnCltdCurvRi_ReMi;      /* '<S8>/Switch5'
                                        * Clothoid curvature of right lane
                                        */
real32_T LDPDT_LnHeadRi_Rad;           /* '<S8>/Switch1'
                                        * Heading angle of rifht lane
                                        */
real32_T LDPDT_RawLatVehSpdRi_Mps;     /* '<S10>/Product1'
                                        * Raw right lateral vehicle speed
                                        */
real32_T LDPDT_LatVehSpdRi_Mps;        /* '<S10>/Switch2'
                                        * right lateral vehicle speed
                                        */
real32_T LDPSC_DstcToLnTrsdCrvCpstnRi_Mi;/* '<S78>/1-D Lookup Table4'
                                          * Curvature compensation of threshold distance to right lane crossing
                                          */
real32_T LDPSC_DstcToLnTrsdRi_Mi;      /* '<S78>/Add3'
                                        *  Threshold distance to right lane crossing
                                        */
real32_T LDPDT_LnPstnRi_Mi;            /* '<S8>/Switch3'
                                        * Position of  of right lane
                                        */
real32_T LDPDT_RawDstcToLnRi_Mi;       /* '<S10>/Subtract1'
                                        * Raw Distance of between vehicel and right lane
                                        */
real32_T LDPDT_DstcToLnRi_Mi;          /* '<S10>/Switch5'
                                        * Distance of between vehicel and right lane
                                        */
real32_T LDPDT_TiToLnRi_Sec;           /* '<S10>/Switch4'
                                        * Time of vehicle to right lane
                                        */
real32_T LDPSC_TiToLnTrsd_Sec;         /* '<S78>/Product1'
                                        * Threshold time of time to lane crossing
                                        */
real32_T LDPVSE_MaxLatVel_Mps;         /* '<S162>/Lookup Table'
                                        * Maximum lateral velocity
                                        */
real32_T LDPDT_CrvThdMaxLf_ReMi;       /* '<S12>/1-D Lookup Table'
                                        * Curve of maximum threshold of left clothiod curvature
                                        */
real32_T LDPDT_CrvThdHystLf_ReMi;      /* '<S12>/1-D Lookup Table1'
                                        * Curve of offset threshold of left clothiod curvature
                                        */
real32_T LDPDT_LnCltdCurvLf_ReMi;      /* '<S8>/Switch4'
                                        * Clothoid curvature of left lane
                                        */
real32_T LDPDT_LnHeadLf_Rad;           /* '<S8>/Switch'
                                        * Heading angle of left lane
                                        */
real32_T LDPDT_LnPstnLf_Mi;            /* '<S8>/Switch2'
                                        * Position of  of left lane
                                        */
real32_T LDPDT_RawDstcToLnLf_Mi;       /* '<S10>/Subtract'
                                        * Raw Distance of between vehicel and left lane
                                        */
real32_T LDPDT_DstcToLnLf_Mi;          /* '<S10>/Switch'
                                        * Distance of between vehicel and left lane
                                        */
real32_T LDPSC_DstcToLnTrsdCrvCpstnLf_Mi;/* '<S78>/1-D Lookup Table5'
                                          * Curvature compensation of threshold distance to left lane crossing
                                          */
real32_T LDPDT_RawLatVehSpdLf_Mps;     /* '<S10>/Product'
                                        * Raw Left lateral vehicle speed
                                        */
real32_T LDPDT_LatVehSpdLf_Mps;        /* '<S10>/Switch1'
                                        * Left lateral vehicle speed
                                        */
real32_T LDPSC_CrvSensiDecayLe_Mi;     /* '<S78>/Switch5' */
real32_T LDPSC_DstcToLnTrsdLf_Mi;      /* '<S78>/Add2'
                                        *  Threshold distance to left lane crossing
                                        */
real32_T LDPDT_TiToLnLf_Sec;           /* '<S10>/Switch3'
                                        * Time of vehicle to left lane
                                        */
real32_T LDPVSE_MaxCrvBySpd_ReMi;      /* '<S160>/Lookup Table'
                                        * Maximum curvature for LDW invalid condition
                                        */
real32_T LDPVSE_HystCrvBySpd_ReMi;     /* '<S160>/Lookup Table1'
                                        * Curvature hysteresis for LDW invalid condition
                                        */
real32_T LDPSC_RampoutTime_Sec;        /* '<S32>/Switch'
                                        * Rampout time
                                        */
real32_T LDPSC_WRBlockTime_Sec;        /* '<S37>/Switch' */
real32_T LDPTT_RawLnBdryPstnYLf_Mi;    /* '<S121>/Switch'
                                        * Raw Filtered left lane clothoid Y0 position from LDP
                                        */
real32_T LDPTT_RawLnBdryPstnYRi_Mi;    /* '<S121>/Switch1'
                                        * Raw Filtered right lane clothoid Y0 position from LDP
                                        */
real32_T LDPTT_TgtLatDstcRi_Mi;        /* '<S121>/Switch9'
                                        * Distance between the hazardous lane marking and the planned target.
                                        */
real32_T LDPTT_TgtLatDstcLf_Mi;        /* '<S121>/Switch2'
                                        * Distance between the hazardous lane marking and the planned target.
                                        */
real32_T LDPTT_RawBdryPstnYCent_Mi;    /* '<S121>/Switch16'
                                        * Raw Filtered center lane clothoid Y0 position from LDP
                                        */
real32_T LDPTV_LatVel_Mps;             /* '<S152>/Switch'
                                        * Lateral Velocity For LDPTV
                                        */
real32_T LDPSC_DlcThdMode2_Mi;         /* '<S78>/1-D Lookup Table1'
                                        * DLC threshold at LDW mode 2
                                        */
real32_T LDPSC_DlcThdMode1_Mi;         /* '<S78>/1-D Lookup Table'
                                        * DLC threshold at LDW mode 1
                                        */
real32_T LDPSC_DlcThdMode3_Mi;         /* '<S78>/1-D Lookup Table2'
                                        * DLC threshold at LDW mode 3
                                        */
real32_T LDPSC_DstcToLnTrsd_Mi;        /* '<S78>/Product'
                                        *  Threshold distance to  lane crossing
                                        */
uint16_T LDPSC_SuppValid_Debug;        /* '<S35>/Signal Conversion8'
                                        * Suppression debug
                                        */
uint8_T LDPSC_DgrSide_St;              /* '<S1>/LDP'
                                        * State of danger side
                                        */
uint8_T LDPTV_TrajCtrlSt_St;           /* '<S1>/LDP'
                                        * Trajectory Guidance Qualifier Output
                                        */
uint8_T LDPDT_CurveTypeRi_St;          /* '<S11>/Switch2'
                                        * Curve Type of right Lane
                                        */
uint8_T LDPVSE_SidCdtnLDPRi_St;        /* '<S162>/Signal Conversion1'
                                        * State of right side at LDP
                                        */
uint8_T LDPDT_CurveTypeLe_St;          /* '<S11>/Switch'
                                        * Curve Type of left Lane
                                        */
uint8_T LDPVSE_SidCdtnLDPLf_St;        /* '<S162>/Signal Conversion'
                                        * State of left side at LDP
                                        */
uint8_T LDPVSE_IvldLDP_St;             /* '<S160>/Signal Conversion2'
                                        * Invalid state of LDP
                                        */
boolean_T LDPSC_RdyToTrig_B;           /* '<S1>/LDP'
                                        * Condition of Ready to trigger state
                                        */
boolean_T LDPTV_HighStatReq_B;         /* '<S1>/LDP'
                                        * High Stationary Accuracy required
                                        */
boolean_T LDPTV_LatCpstnEn_B;          /* '<S1>/LDP'
                                        * Switch for the latency compensation in trajectory plan
                                        */
boolean_T LDPTV_LmtEn_B;               /* '<S1>/LDP'
                                        * Switch to limit the target curvature in trajectory controller
                                        */
boolean_T LDPTV_TrigReplan_B;          /* '<S1>/LDP'
                                        * It has to be 1 for trajectory planning to calculate a trajectory
                                        */
boolean_T LDPSC_NVRAMLDPSwitch_B;      /* '<S1>/LDP' */
boolean_T LDPDT_RdyTrigLDP_B;          /* '<S8>/Equal'
                                        * Condition of ready to trigger LDP state
                                        */
boolean_T LDPDT_EnaSafety_B;           /* '<S8>/AND'
                                        * Enable flag for data from the safety interface
                                        */
boolean_T LDPDT_EnaByInVldQlfrRi_B;    /* '<S13>/Relational Operator4'
                                        * Enable flag for right lane validity by left lane invalid qualifier
                                        */
boolean_T LDPDT_EnaByInVldQlfrSfRi_B;  /* '<S13>/Relational Operator1'
                                        * Enable flag for right lane validity by left lane invalid qualifier for safety interface
                                        */
boolean_T LDPDT_LnTrigVldRi_B;         /* '<S13>/Logical Operator2'
                                        * Condition validity of right lane marker at LDP trigger
                                        */
boolean_T LDPDT_CclByInVldQlfrRi_B;    /* '<S13>/Relational Operator5'
                                        * Enable flag for right lane validity by left lane invalid qualifier for safety interface when cancel the function
                                        */
boolean_T LDPDT_LnCclVldRi_B;          /* '<S13>/Logical Operator5'
                                        * Condition validity of right lane marker at LDP cancel
                                        */
boolean_T LDPDT_LnMakVldRi_B;          /* '<S13>/Switch'
                                        * Condition validity of right lane marker
                                        */
boolean_T LDPSC_RawTrigByDlcRi_B;      /* '<S80>/Relational Operator3'
                                        * Raw trigger flag by DLC for right lane
                                        */
boolean_T LDPSC_EnaTlcTrigRi_B;        /* '<S80>/Relational Operator1'
                                        * Enable flag for Raw trigger flag by TLC for right lane
                                        */
boolean_T LDPSC_RawTrigByTlcRi_B;      /* '<S80>/Relational Operator2'
                                        * Raw trigger flag by TLC for right lane
                                        */
boolean_T LDPSC_DlyTrigByTlcRi_B;      /* '<S106>/AND'
                                        * Trigger flag by TLC for right lane
                                        */
boolean_T LDPSC_EnaLdwTrigRi_B;        /* '<S95>/AND'
                                        * Enable flag for LDW function trigger
                                        */
boolean_T LDPSC_RstLdwTrigRi_B;        /* '<S80>/OR'
                                        * Reset flag for LDW function trigger
                                        */
boolean_T LDPSC_HoldLdwTrigRi_B;       /* '<S80>/Signal Conversion'
                                        * Enable flag for LDW function trigger after time holding
                                        */
boolean_T LDPSC_ResetForSafeRi_B;      /* '<S80>/Logical Operator4'
                                        * Reset flag for the safe situation condition of right lane
                                        */
boolean_T LDPSC_SetForSafeRi_B;        /* '<S80>/Relational Operator6'
                                        * Set flag for the safe situation condition of right lane
                                        */
boolean_T LDPSC_SetForContinTrigRi_B;  /* '<S80>/Logical Operator13' */
boolean_T LDPSC_ResetForContinTrigRi_B;/* '<S80>/Logical Operator18' */
boolean_T LDPVSE_EdgeRiseTurnSglRi_B;  /* '<S174>/AND' */
boolean_T LDPVSE_TrnSglRi_B;           /* '<S161>/Signal Conversion2'
                                        * Condition of  right turn signal
                                        */
boolean_T LDPVSE_RdyTrigLDW_B;         /* '<S162>/Equal'
                                        * Ready Trigger flag for LDW
                                        */
boolean_T LDPVSE_VehLatSpdVldRi_B;     /* '<S180>/Switch'
                                        * Validity of right lateral vehicle speed
                                        */
boolean_T LDPSC_TrigBySideCondRi_B;    /* '<S80>/Relational Operator7'
                                        * LDW function trigger flag by  side condition of right lane
                                        */
boolean_T LDPSC_TrigByPrjSpecRi_B;     /* '<S80>/Equal'
                                        * LDW function trigger flag by customer projects of right lane
                                        */
boolean_T LDPSC_TrigRi_B;              /* '<S80>/Logical Operator3'
                                        * Condition of right trigger
                                        */
boolean_T LDPDT_EnaByCstruSiteLf_B;    /* '<S12>/AND'
                                        * Enable flag for left lane validity by construction site detected
                                        */
boolean_T LDPDT_EnaByInVldQlfrLf_B;    /* '<S12>/Relational Operator4'
                                        * Enable flag for left lane validity by left lane invalid qualifier
                                        */
boolean_T LDPDT_EnaByInVldQlfrSfLf_B;  /* '<S12>/Relational Operator1'
                                        * Enable flag for left lane validity by left lane invalid qualifier for safety interface
                                        */
boolean_T LDPDT_LnTrigVldLf_B;         /* '<S12>/Logical Operator2'
                                        * Condition validity of left lane marker at LDP trigger
                                        */
boolean_T LDPDT_CclByInVldQlfrLf_B;    /* '<S12>/Relational Operator5'
                                        * Enable flag for left lane validity by left lane invalid qualifier for safety interface when cancel the function
                                        */
boolean_T LDPDT_LnCclVldLf_B;          /* '<S12>/Logical Operator5'
                                        * Condition validity of left lane marker at LDP cancel
                                        */
boolean_T LDPDT_LnMakVldLf_B;          /* '<S12>/Switch'
                                        * Condition validity of left lane marker
                                        */
boolean_T LDPSC_RawTrigByDlcLf_B;      /* '<S79>/Relational Operator3'
                                        * Raw trigger flag by DLC for left lane
                                        */
boolean_T LDPSC_EnaTlcTrigLf_B;        /* '<S79>/Relational Operator1'
                                        * Enable flag for Raw trigger flag by TLC for left lane
                                        */
boolean_T LDPSC_RawTrigByTlcLf_B;      /* '<S79>/Relational Operator2'
                                        * Raw trigger flag by TLC for left lane
                                        */
boolean_T LDPSC_DlyTrigByTlcLf_B;      /* '<S94>/AND'
                                        * Trigger flag by TLC for left lane
                                        */
boolean_T LDPSC_EnaLdwTrigLf_B;        /* '<S83>/AND'
                                        * Enable flag for LDW function trigger
                                        */
boolean_T LDPSC_RstLdwTrigLf_B;        /* '<S79>/Logical Operator15'
                                        * Reset flag for LDW function trigger
                                        */
boolean_T LDPSC_HoldLdwTrigLf_B;       /* '<S79>/Signal Conversion'
                                        * Enable flag for LDW function trigger after time holding
                                        */
boolean_T LDPSC_ResetForSafeLf_B;      /* '<S79>/Logical Operator4'
                                        * Reset flag for the safe situation condition of left lane
                                        */
boolean_T LDPSC_SetForSafeLf_B;        /* '<S79>/Relational Operator6'
                                        * Set flag for the safe situation condition of left lane
                                        */
boolean_T LDPSC_ResetForContinTrigLf_B;/* '<S79>/Logical Operator9' */
boolean_T LDPSC_SetForContinTrigLf_B;  /* '<S79>/Logical Operator13' */
boolean_T LDPVSE_EdgeRiseTurnSglLf_B;  /* '<S173>/AND' */
boolean_T LDPVSE_TrnSglLf_B;           /* '<S161>/Signal Conversion'
                                        * Condition of  left turn signal
                                        */
boolean_T LDPVSE_VehLatSpdVldLf_B;     /* '<S177>/Switch'
                                        * Validity of left lateral vehicle speed
                                        */
boolean_T LDPSC_TrigBySideCondLf_B;    /* '<S79>/Relational Operator7'
                                        * LDW function trigger flag by  side condition of left lane
                                        */
boolean_T LDPSC_TrigByPrjSpecLf_B;     /* '<S79>/Equal'
                                        * LDW function trigger flag by customer projects of left lane
                                        */
boolean_T LDPSC_TrigLf_B;              /* '<S79>/Logical Operator3'
                                        * Condition of left trigger
                                        */
boolean_T LDPSC_EnaDgrSide_B;          /* '<S77>/Logical Operator6'
                                        * Enable flag for Degerous side state
                                        */
boolean_T LDPSC_FnsByDgrStLf_B;        /* '<S33>/Relational Operator7' */
boolean_T LDPSC_FnsByLatDistLf_B;      /* '<S33>/Logical Operator8' */
boolean_T LDPSC_FnsByHeadingLf_B;      /* '<S33>/Logical Operator9' */
boolean_T LDPSC_FnsByLatSpdLf_B;       /* '<S33>/Logical Operator10' */
boolean_T LDPSC_DgrFnsLf_B;            /* '<S33>/Logical Operator6' */
boolean_T LDPSC_FnsByDgrStRi_B;        /* '<S33>/Relational Operator9' */
boolean_T LDPSC_FnsByLatDistRi_B;      /* '<S33>/Logical Operator14' */
boolean_T LDPSC_FnsByHeadingRi_B;      /* '<S33>/Logical Operator15' */
boolean_T LDPSC_FnsByLatSpdRi_B;       /* '<S33>/Logical Operator13' */
boolean_T LDPSC_DgrFnsRi_B;            /* '<S33>/Logical Operator12' */
boolean_T LDPSC_MinLdwBySysSt_B;       /* '<S33>/Relational Operator8' */
boolean_T LDPSC_EdgeRiseForMinLdw_B;   /* '<S60>/AND' */
boolean_T LDPSC_HoldForMinLdw_B;       /* '<S76>/GreaterThan1' */
boolean_T LDPSC_FlagMinTimeLDW_B;      /* '<S33>/Logical Operator11' */
boolean_T LDPSC_DgrFns_B;              /* '<S69>/AND'
                                        * Condition of danger finish
                                        */
boolean_T LDPSC_CancelBySpecific_B;    /* '<S36>/Relational Operator38'
                                        * LDW cancel conditions by LDW specific bitfield
                                        */
boolean_T LDPSC_CancelByVehSt_B;       /* '<S36>/Relational Operator37'
                                        * LDW cancel conditions by vehicle state
                                        */
boolean_T LDPSC_CancelByDrvSt_B;       /* '<S36>/Relational Operator32'
                                        * LDW cancel conditions by drive state
                                        */
boolean_T LDPSC_CancelByCtrlSt_B;      /* '<S36>/Relational Operator33'
                                        * LDW cancel conditions by active control state
                                        */
boolean_T LDPSC_CancelBySysSt_B;       /* '<S36>/Relational Operator34'
                                        * LDW cancel conditions by system state
                                        */
boolean_T LDPSC_CancelByAvlSt_B;       /* '<S36>/Relational Operator35'
                                        * LDW cancel conditions by no available state
                                        */
boolean_T LDPSC_CancelByPrjSpec_B;     /* '<S36>/Relational Operator36'
                                        * LDW cancel conditions by customer projects
                                        */
boolean_T LDPSC_MaxDurationBySysSt_B;  /* '<S40>/Relational Operator47' */
boolean_T LDPSC_EdgRiseForSysSt_B;     /* '<S41>/AND' */
boolean_T LDPSC_MaxDurationByStDly_B;  /* '<S40>/Logical Operator22' */
boolean_T LDPSC_TiWarmMx_B;            /* '<S40>/Logical Operator23'
                                        * Condition of warming max time
                                        */
boolean_T LDPSC_ErrSideByTrigLf_B;     /* '<S39>/Logical Operator19' */
boolean_T LDPSC_ErrSideBySideCondLf_B; /* '<S39>/Relational Operator42' */
boolean_T LDPSC_ErrSidByPrjSpecLf_B;   /* '<S39>/Relational Operator43' */
boolean_T LDPSC_ErrSidCdtnLf_B;        /* '<S39>/Logical Operator18'
                                        * Error condition of left side
                                        */
boolean_T LDPSC_SideCondByDgrLf_B;     /* '<S39>/Relational Operator41' */
boolean_T LDPSC_CanelBySideLf_B;       /* '<S39>/Logical Operator15' */
boolean_T LDPSC_SideCondByDgrRi_B;     /* '<S39>/Relational Operator44' */
boolean_T LDPSC_ErrSideByTrigRi_B;     /* '<S39>/Logical Operator21' */
boolean_T LDPSC_ErrSideBySideCondRi_B; /* '<S39>/Relational Operator45' */
boolean_T LDPSC_ErrSidByPrjSpecRi_B;   /* '<S39>/Relational Operator46' */
boolean_T LDPSC_ErrSidCdtnRi_B;        /* '<S39>/Logical Operator2'
                                        * Error condition of right side
                                        */
boolean_T LDPSC_CanelBySideRi_B;       /* '<S39>/Logical Operator1' */
boolean_T LDPSC_ErrSidCdtn_B;          /* '<S39>/Logical Operator16'
                                        * Error condition of side
                                        */
boolean_T LDPSC_CLatDevByDlcLf_B;      /* '<S38>/Logical Operator24' */
boolean_T LDPSC_CLatDevByDgrLf_B;      /* '<S38>/Relational Operator39' */
boolean_T LDPSC_CclLatDevLf_B;         /* '<S38>/Logical Operator12'
                                        * Cancel condition of left lane deviation
                                        */
boolean_T LDPSC_CLatDevByDlcRi_B;      /* '<S38>/Relational Operator40' */
boolean_T LDPSC_CLatDevByDgrRi_B;      /* '<S38>/Logical Operator25' */
boolean_T LDPSC_CclLatDevRi_B;         /* '<S38>/Logical Operator14'
                                        * Cancel condition of right lane deviation
                                        */
boolean_T LDPSC_CclLatDev_B;           /* '<S38>/Logical Operator13'
                                        * Cancel condition of lane deviation
                                        */
boolean_T LDPSC_Cancel_B;              /* '<S36>/Logical Operator11' */
boolean_T LDPSC_AbortBySpecific_B;     /* '<S37>/Relational Operator2'
                                        * LDW abort conditions by LDW specific bitfield
                                        */
boolean_T LDPSC_AbortByVehSt_B;        /* '<S37>/Relational Operator1'
                                        * LDW abort conditions by vehicle state
                                        */
boolean_T LDPSC_AbortByDrvSt_B;        /* '<S37>/Relational Operator3'
                                        * LDW abort conditions by drive state
                                        */
boolean_T LDPSC_AbortByCtrlSt_B;       /* '<S37>/Relational Operator4'
                                        * LDW abort conditions by active control state
                                        */
boolean_T LDPSC_AbortBySysSt_B;        /* '<S37>/Relational Operator5'
                                        * LDW abort conditions by system state
                                        */
boolean_T LDPSC_AbortByAvlSt_B;        /* '<S37>/Relational Operator6'
                                        * LDW abort conditions by no available state
                                        */
boolean_T LDPSC_AbortByPrjSpec_B;      /* '<S37>/Relational Operator7'
                                        * LDW abort conditions by customer projects
                                        */
boolean_T LDPSC_Abort_B;               /* '<S37>/Logical Operator6'
                                        * Condition of LDP abort state
                                        */
boolean_T LDPSC_StrgRdy_B;             /* '<S37>/Logical Operator1'
                                        * Condition of LDP strong ready state
                                        */
boolean_T LDPSC_Degradation_B;         /* '<S32>/Logical Operator1' */
boolean_T LDPSC_DegradationEdgeRise_B; /* '<S58>/AND' */
boolean_T LDPSC_Degr_B;                /* '<S32>/Logical Operator2'
                                        * Condition of degradation
                                        */
boolean_T LDPSC_SuppBySpecific_B;      /* '<S37>/Relational Operator21'
                                        * LDW suppresion conditions by LDW specific bitfield
                                        */
boolean_T LDPSC_SuppByVehSt_B;         /* '<S37>/Relational Operator20'
                                        * LDW suppresion conditions by vehicle state
                                        */
boolean_T LDPSC_SuppByDrvSt_B;         /* '<S37>/Relational Operator15'
                                        * LDW suppresion conditions by drive state
                                        */
boolean_T LDPSC_SuppByCtrlSt_B;        /* '<S37>/Relational Operator16'
                                        * LDW suppresion conditions by active control state
                                        */
boolean_T LDPSC_SuppBySysSt_B;         /* '<S37>/Relational Operator17'
                                        * LDW suppresion conditions by system state
                                        */
boolean_T LDPSC_SuppyByAvlSt_B;        /* '<S37>/Relational Operator18'
                                        * LDW suppresion conditions by no available state
                                        */
boolean_T LDPSC_SuppPrjSpec_B;         /* '<S37>/Relational Operator19'
                                        * LDW suppresion conditions by customer projects
                                        */
boolean_T LDPSC_Suppresion_B;          /* '<S37>/Logical Operator3' */
boolean_T LDPSC_WeakRdyBySpecific_B;   /* '<S37>/Relational Operator28'
                                        * LDW weak ready conditions by LDW specific bitfield
                                        */
boolean_T LDPSC_WeakRdyByVehSt_B;      /* '<S37>/Relational Operator27'
                                        * LDW weak ready conditions by vehicle state
                                        */
boolean_T LDPSC_WeakRdyByDrvSt_B;      /* '<S37>/Relational Operator22'
                                        * LDW weak ready conditions by drive state
                                        */
boolean_T LDPSC_WeakRdyByCtrlSt_B;     /* '<S37>/Relational Operator23'
                                        * LDW strong weak conditions by active control state
                                        */
boolean_T LDPSC_WeakRdyBySysSt_B;      /* '<S37>/Relational Operator24'
                                        * LDW weak ready conditions by system state
                                        */
boolean_T LDPSC_WeakRdyByAvlSt_B;      /* '<S37>/Relational Operator25'
                                        * LDW weak weak conditions by no available state
                                        */
boolean_T LDPSC_WeakRdyByPrjSpec_B;    /* '<S37>/Relational Operator26'
                                        * LDW weak weak conditions by customer projects
                                        */
boolean_T LDPSC_WkRdy_B;               /* '<S37>/Logical Operator4'
                                        * Condition of LDP weak ready state
                                        */
boolean_T LDPSC_BlockTimeBySysOut_B;   /* '<S37>/Logical Operator9' */
boolean_T LDPSC_RawBlockTimeByRampOut_B;/* '<S44>/AND' */
boolean_T LDPSC_BlockTimeByRampOut_B;  /* '<S56>/GreaterThan1' */
boolean_T LDPSC_BlockTime_B;           /* '<S37>/Logical Operator7' */
boolean_T LDPSC_Suppression_B;         /* '<S35>/OR'
                                        * Suppression condition
                                        */
boolean_T LDPVSE_TgtCntrByLnWidth_B;   /* '<S160>/Less Than' */
boolean_T LDPVSE_TgtCntrLnEn_B;        /* '<S160>/Logical Operator11'
                                        *  Enable the target in the center of the lane.
                                        */
boolean_T LDPTV_CurvInner_B;           /* '<S157>/AND2'
                                        * The flag for the left or right lane marking is an inner curve
                                        */
boolean_T LDPTV_CurvOuter_B;           /* '<S157>/AND1'
                                        * The flag for the left or right lane marking is an outer curve
                                        */
boolean_T LDPSC_Trig_B;                /* '<S77>/Logical Operator1'
                                        * Condition of trigger
                                        */
E_LDPState_nu LDPSC_SysOut_St;         /* '<S23>/Switch1'
                                        * Actual state of LDP
                                        */

/* Exported block states */
real32_T LDPSC_DlyTiOfTiToLnRiMn_Sec;  /* '<S106>/Unit Delay'
                                        * Delay time of time to right lane crossing
                                        */
real32_T LDPSC_HdTiTrigRi_Sec;         /* '<S104>/Unit Delay'
                                        * holding time right trigger
                                        */
real32_T LDPSC_ContinWarmTimesOldRi_Count;/* '<S80>/Unit Delay2'
                                           * The number of consecutive alarms in the previous cycle
                                           */
real32_T LDPSC_SuppTimeOldRi_Sec;      /* '<S105>/Unit Delay'
                                        * Holding time of left turn signal
                                        */
real32_T LDPVSE_HodTiTrnSglRi_Sec;     /* '<S176>/Unit Delay'
                                        * Holding time of right turn signal
                                        */
real32_T LDPSC_DlyTiOfTiToLnLfMn_Sec;  /* '<S94>/Unit Delay'
                                        * Delay time of time to left lane crossing
                                        */
real32_T LDPSC_HdTiTrigLf_Sec;         /* '<S92>/Unit Delay'
                                        *  holding time left trigger
                                        */
real32_T LDPSC_ContinWarmTimesOldLf_Count;/* '<S79>/Unit Delay2'
                                           * The number of consecutive alarms in the previous cycle
                                           */
real32_T LDPSC_SuppTimeOldLf_Sec;      /* '<S93>/Unit Delay'
                                        * Holding time of left turn signal
                                        */
real32_T LDPVSE_HodTiTrnSglLf_Sec;     /* '<S175>/Unit Delay'
                                        * Holding time of left turn signal
                                        */
real32_T LDPSC_HdTiWarming_Sec;        /* '<S76>/Unit Delay'
                                        * Holding time of warming state start
                                        */
real32_T LDPSC_DlyTiTgtFns_Sec;        /* '<S69>/Unit Delay'
                                        * Delay time of LDP finish state
                                        */
real32_T LDPSC_HdTiWarmMx_Sec;         /* '<S43>/Unit Delay'
                                        * Holding time of warming state start
                                        */
real32_T LDPSC_HdTiDegr_Sec;           /* '<S59>/Unit Delay'
                                        * Holding time of degradation
                                        */
real32_T LDPSC_HdTiFns_Sec;            /* '<S56>/Unit Delay'
                                        * Holding time of finish state end
                                        */
real32_T LDPSC_ActiveStopWatch_Ri_sec; /* '<S55>/Unit Delay' */
real32_T LDPSC_HdTiFns_Ri_Sec;         /* '<S57>/Unit Delay' */
real32_T LDPTT_LwLnBdryPstnXLf_Mi;     /* '<S129>/Unit Delay'
                                        * Low filtered left lane clothoid X0 position from LDP
                                        */
real32_T LDPTT_LstLnBdryPstnXLf_Mi;    /* '<S126>/UnitDelay'
                                        * Last filtered left lane clothoid Y0 position from LDP
                                        */
real32_T LDPTT_LstLnBdryVldLengLf_Mi;  /* '<S126>/UnitDelay5'
                                        * Last filtered left lane clothoid length from LDP
                                        */
real32_T LDPTT_LwLnBdryVldLengLf_Mi;   /* '<S134>/Unit Delay'
                                        * Low filtered left lane clothoid length from LDP
                                        */
real32_T LDPTT_LstLnBdryPstnYLf_Mi;    /* '<S126>/UnitDelay1'
                                        * Last filtered left lane clothoid Y0 position from LDP
                                        */
real32_T LDPTT_LstLnWidCalc_Mi;        /* '<S121>/UnitDelay'
                                        * Calculation ego lane width
                                        */
real32_T LDPTT_LwLnBdryPstnYLf_Mi;     /* '<S130>/Unit Delay'
                                        * Low filtered left lane clothoid Y0 position from LDP
                                        */
real32_T LDPTT_LwLnBdryPstnYRi_Mi;     /* '<S136>/Unit Delay'
                                        * Low filtered right lane clothoid Y0 position from LDP
                                        */
real32_T LDPTT_LwLnBdryPstnXRi_Mi;     /* '<S135>/Unit Delay'
                                        * Low filtered right lane clothoid X0 position from LDP
                                        */
real32_T LDPTT_LwLnBdryHeadAglRi_Rad;  /* '<S137>/Unit Delay'
                                        * Low filtered right lane clothoid heading angle from LDP
                                        */
real32_T LDPTT_LwLnBdryCurvRi_ReMi;    /* '<S138>/Unit Delay'
                                        * Low filtered right lane clothoid curvature from LDP
                                        */
real32_T LDPTT_LwLnBdryCurvRateRi_ReMi2;/* '<S139>/Unit Delay'
                                         * Low filtered right lane clothoid change of curvature from LDP
                                         */
real32_T LDPTT_LwLnBdryVldLengRi_Mi;   /* '<S140>/Unit Delay'
                                        * Low filtered right lane clothoid length from LDP
                                        */
real32_T LDPTT_LstTgtLatDstcRi_Mi;     /* '<S121>/UnitDelay4'
                                        * Last distance between the hazardous lane marking and the planned target.
                                        */
real32_T LDPTT_LstMxTgtLatDevRi_Mi;    /* '<S121>/UnitDelay6'
                                        * Last maximal distance between the middle of the vehicle and the planned target.
                                        */
real32_T LDPTT_LstTgtLatDevRi_Mi;      /* '<S121>/UnitDelay7'
                                        * Last Distance between the middle of the vehicle and the planned target.
                                        */
real32_T LDPTT_LstTgtLatDevLf_Mi;      /* '<S121>/UnitDelay3'
                                        * Last Distance between the middle of the vehicle and the planned target.
                                        */
real32_T LDPTT_LstTgtLatDstcLf_Mi;     /* '<S121>/UnitDelay1'
                                        * Last distance between the hazardous lane marking and the planned target.
                                        */
real32_T LDPTT_LstMxTgtLatDevLf_Mi;    /* '<S121>/UnitDelay2'
                                        * Last maximal distance between the middle of the vehicle and the planned target.
                                        */
real32_T LDPTT_LwLnBdryPstnYCent_Mi;   /* '<S141>/Unit Delay'
                                        * Low filtered center lane clothoid Y0 position from LDP
                                        */
real32_T LDPTT_LstLnBdryPstnYCent_Mi;  /* '<S128>/UnitDelay1'
                                        * Last filtered center lane clothoid Y0 position from LDP
                                        */
real32_T LDPTT_LstLnBdryCurvCent_ReMi; /* '<S128>/UnitDelay3'
                                        * Last filtered center lane clothoid curvature from LDP
                                        */
real32_T LDPTT_LwLnBdryCurvCent_ReMi;  /* '<S143>/Unit Delay'
                                        * Low filtered center lane clothoid curvature from LDP
                                        */
real32_T LDPTV_LstPlanningHorizon_Sec; /* '<S150>/UnitDelay1'
                                        * Planning Horizon Scaling Factor for the calculation in the TRJPLN
                                        */
real32_T LDPTV_LstWeightEndTi_Fct;     /* '<S150>/UnitDelay'
                                        * Last weight of the end time for the calculation in the TRJPLN
                                        */
real32_T LDPTT_LwLnBdryHeadAglCent_Rad;/* '<S142>/Unit Delay'
                                        * Low filtered center lane clothoid heading angle from LDP
                                        */
real32_T LDPTT_LstLnBdryHeadAglCent_Rad;/* '<S128>/UnitDelay2'
                                         * Last filtered center lane clothoid heading angle from LDP
                                         */
real32_T LDPTT_LwLnBdryCurvRateCent_ReMi2;/* '<S144>/Unit Delay'
                                           * Low filtered center lane clothoid change of curvature from LDP
                                           */
real32_T LDPTT_LstLnBdryCurvRateCent_ReMi2;/* '<S128>/UnitDelay4'
                                            * Last filtered center lane clothoid change of curvature from LDP
                                            */
real32_T LDPTT_LwLnBdryPstnXCent_Mi;   /* '<S145>/Unit Delay'
                                        * Low filtered center lane clothoid X0 position from LDP
                                        */
real32_T LDPTT_LstLnBdryPstnXCent_Mi;  /* '<S128>/UnitDelay'
                                        * Last filtered center lane clothoid X0 position from LDP
                                        */
real32_T LDPTT_LwLnBdryVldLengCent_Mi; /* '<S146>/Unit Delay'
                                        * Low filtered center lane clothoid length from LDP
                                        */
real32_T LDPTT_LstLnBdryVldLengCent_Mi;/* '<S128>/UnitDelay5'
                                        * Last filtered center lane clothoid length from LDP
                                        */
real32_T LDPTV_LstSteWhlGrad_ReS;      /* '<S148>/UnitDelay1'
                                        * Last Steering Wheel Stiffness Gradient by Lateral Velocity
                                        */
real32_T LDPTV_LstDMCDeraLvl_Fct;      /* '<S148>/UnitDelay'
                                        * Last DMC Derating Level of the LDP function
                                        */
real32_T LDPTT_LstLnBdryHeadAglLf_Rad; /* '<S126>/UnitDelay2'
                                        * Last filtered left lane clothoid heading angle from LDP
                                        */
real32_T LDPTT_LwLnBdryHeadAglLf_Rad;  /* '<S131>/Unit Delay'
                                        * Low filtered left lane clothoid heading angle from LDP
                                        */
real32_T LDPTT_LstLnBdryCurvLf_ReMi;   /* '<S126>/UnitDelay3'
                                        * Last filtered left lane clothoid curvature from LDP
                                        */
real32_T LDPTT_LwLnBdryCurvLf_ReMi;    /* '<S132>/Unit Delay'
                                        * Low filtered left lane clothoid curvature from LDP
                                        */
real32_T LDPTT_LstLnBdryCurvRateLf_ReMi2;/* '<S126>/UnitDelay4'
                                          * Last filtered left lane clothoid change of curvature from LDP
                                          */
real32_T LDPTT_LwLnBdryCurvRateLf_ReMi2;/* '<S133>/Unit Delay'
                                         * Low filtered left lane clothoid change of curvature from LDP
                                         */
uint8_T LDPSC_LstPrevDgrSide_St;       /* '<S2>/Unit Delay' */
uint8_T LDPSC_DgrSideOld_St;           /* '<S77>/UnitDelay1'
                                        * Old state of danger side
                                        */
boolean_T LDPDT_UHysCltdCurvVldRi_B;   /* '<S19>/Unit Delay'
                                        * Validity of right lane Clothoid curvature
                                        */
boolean_T LDPDT_BHysHeadAglTrigVldRi_B;/* '<S18>/Unit Delay'
                                        * Valid trigger of right heading angle
                                        */
boolean_T LDPDT_UHysHeadAglCclVldRi_B; /* '<S20>/Unit Delay'
                                        * Valid cancel of right heading angle
                                        */
boolean_T LDPSC_HdTiTrigRiEn_B;        /* '<S95>/Unit Delay'
                                        * Enable condition of holding time right trigger
                                        */
boolean_T LDPSC_DisTrigRi_B;           /* '<S102>/Unit Delay'
                                        * Disable condition of right trigger
                                        */
boolean_T LDPSC_SuppFlagOldRi_B;       /* '<S80>/Unit Delay'
                                        * Suppression signal of continuous alarm in previous period
                                        */
boolean_T LDPSC_PreActiveEdgeRi;       /* '<S98>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
boolean_T LDPSC_ContinTrigRiEn_B;      /* '<S99>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
boolean_T LDPSC_DisContinTrigRi_B;     /* '<S103>/Unit Delay'
                                        * Disable condition of continue left trigger
                                        */
boolean_T LDPVSE_EdgeRisTrnSglLf_B;    /* '<S174>/Unit Delay'
                                        * Edge rise of left turn signal
                                        */
boolean_T LDPVSE_BHysLatVehSpdVldRi_B; /* '<S185>/Unit Delay'
                                        * Validity of right lateral vehicle speed before trigger state
                                        */
boolean_T LDPVSE_UHysLatVehSpdVldRi_B; /* '<S186>/Unit Delay'
                                        * Validity of right lateral vehicle speed after trigger state
                                        */
boolean_T LDPDT_UHysCltdCurvVldLf_B;   /* '<S15>/Unit Delay'
                                        * Validity of left lane Clothoid curvature
                                        */
boolean_T LDPDT_BHysHeadAglTrigVldLf_B;/* '<S14>/Unit Delay'
                                        * Valid trigger of left heading angle
                                        */
boolean_T LDPDT_UHysHeadAglCclVldLf_B; /* '<S16>/Unit Delay'
                                        * Valid cancel of left heading angle
                                        */
boolean_T LDPSC_HdTiTrigLfEn_B;        /* '<S83>/Unit Delay'
                                        * Enable condition of holding time left trigger
                                        */
boolean_T LDPSC_DisTrigLf_B;           /* '<S90>/Unit Delay'
                                        * Disable condition of left trigger
                                        */
boolean_T LDPSC_SuppFlagOldLf_B;       /* '<S79>/Unit Delay'
                                        * Suppression signal of continuous alarm in previous period
                                        */
boolean_T LDPSC_PreActiveEdgeLf;       /* '<S86>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
boolean_T LDPSC_ContinTrigLfEn_B;      /* '<S87>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
boolean_T LDPSC_DisContinTrigLf_B;     /* '<S91>/Unit Delay'
                                        * Disable condition of continue left trigger
                                        */
boolean_T LDPVSE_EdgeRisTrnSglRi_B;    /* '<S173>/Unit Delay'
                                        * Edge rise of right turn signal
                                        */
boolean_T LDPVSE_BHysLatVehSpdVldLf_B; /* '<S181>/Unit Delay'
                                        * Validity of left lateral vehicle speed before trigger state
                                        */
boolean_T LDPVSE_UHysLatVehSpdVldLf_B; /* '<S182>/Unit Delay'
                                        * Validity of left lateral vehicle speed after trigger state
                                        */
boolean_T LDPSC_EdgeRisWarming_B;      /* '<S60>/Unit Delay'
                                        * Edge rise of warming state
                                        */
boolean_T LDPVSE_BHysSpdVeh_B;         /* '<S163>/Unit Delay'
                                        * Validity of displayed longitudinal speed
                                        */
boolean_T LDPVSE_UHysSteAgl_B;         /* '<S167>/Unit Delay'
                                        * Validity of steering wheel angle
                                        */
boolean_T LDPVSE_UHysSteAglSpd_B;      /* '<S168>/Unit Delay'
                                        * Validity of steering wheel angle speed
                                        */
boolean_T LDPVSE_BHysAccVehX_B;        /* '<S164>/Unit Delay'
                                        * Validity of  longitudinal Acceleration
                                        */
boolean_T LDPVSE_BHysAccVehY_B;        /* '<S169>/Unit Delay'
                                        * Validity of  lateral Acceleration
                                        */
boolean_T LDPVSE_UHysVehCurv_B;        /* '<S170>/Unit Delay'
                                        * Validity of  vehicle curvature
                                        */
boolean_T LDPVSE_BHysLnWid_B;          /* '<S165>/Unit Delay'
                                        * Validity of lane width
                                        */
boolean_T LDPSC_EdgeRisWarmMx_B;       /* '<S41>/Unit Delay'
                                        * Edge rise of warming state
                                        */
boolean_T LDPSC_EdgeRisDegr_B;         /* '<S58>/Unit Delay'
                                        * Edge rise of degradation
                                        */
boolean_T LDPSC_DegrOld_B;             /* '<S32>/UnitDelay'
                                        * UnitDelay condition of degradation
                                        */
boolean_T LDPSC_EdgeRisFns_B;          /* '<S44>/Unit Delay'
                                        * Edge rise of fginish and cancel state
                                        */
boolean_T LDPSC_EdgeRisActive_Ri_B;    /* '<S47>/Unit Delay' */
boolean_T LDPSC_EdgeRisFns_Ri_B;       /* '<S45>/Unit Delay' */
boolean_T LDPTT_CtrlIniEn_B;           /* '<S122>/Unit Delay'
                                        * Control initenable flag for LDP
                                        */
boolean_T LDPTT_LstControl_B;          /* '<S121>/UnitDelay5'
                                        * Control Sate of LDP
                                        */
boolean_T LDPTV_LstCtrl_St;            /* '<S148>/UnitDelay4'
                                        * Last Control State for DetermineStiffnessAndStatAccu
                                        */
E_LDPState_nu LDPSC_SysOld_St;         /* '<S4>/UnitDelay'
                                        * Old state of LDP
                                        */

/* Block states (default storage) */
DW_LDPSA_T LDPSA_DW;

/* Exported data definition */

/* Definition for custom storage class: Global */
uint8_T LDPSA_SetBit_BS_Param_1[9] = { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U } ;
                                   /* Referenced by: '<S116>/ex_sfun_set_bit' */

uint8_T LDPSA_SetBit_BS_Param_2[8] = { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U } ;
                                   /* Referenced by: '<S172>/ex_sfun_set_bit' */

uint8_T LDPSA_SetBit_BS_Param_3[2] = { 0U, 1U } ;/* Referenced by:
                                                  * '<S183>/ex_sfun_set_bit'
                                                  * '<S184>/ex_sfun_set_bit'
                                                  */

real32_T LDPSC_LnLatVeh_BX_Mps[9] = { -1.5F, -1.0F, -0.5F, -0.3F, 0.0F, 0.3F,
  0.5F, 1.0F, 1.5F } ;                 /* Referenced by:
                                        * '<S78>/1-D Lookup Table10'
                                        * '<S78>/1-D Lookup Table11'
                                        */

real32_T LDPSC_LnLatVeh_Lf_Mps[9] = { 0.35F, 0.35F, 0.25F, 0.2F, 0.2F, 0.1F,
  0.1F, 0.1F, 0.1F } ;           /* Referenced by: '<S78>/1-D Lookup Table10' */

real32_T LDPSC_LnLatVeh_Ri_Mps[9] = { 0.1F, 0.1F, 0.1F, 0.1F, 0.2F, 0.2F, 0.25F,
  0.35F, 0.35F } ;               /* Referenced by: '<S78>/1-D Lookup Table11' */



/* Breakpoint of vehicle speed  */

/* Definition for custom storage class: Global */
boolean_T LDPDT_LnLengthLf_B;          /* '<S17>/Unit Delay' */
boolean_T LDPDT_LnLengthRi_B;          /* '<S21>/Unit Delay' */
real32_T LDPSC_ActiveStopWatch_sec;    /* '<S54>/Unit Delay' */
boolean_T LDPSC_EdgeRisActive_B;       /* '<S46>/Unit Delay' */
boolean_T LDPSC_PrevStandbyUnitDelay_bool;/* '<S28>/Unit Delay' */

/* Previous standby status */
boolean_T LDPSC_PrevSwitchUnitDelay_bool;/* '<S25>/Unit Delay' */

/* Previous standby status */
boolean_T LDPSC_RampTimeExpiredRSFF_bool;/* '<S29>/Unit Delay' */

/* Rampout time expired */
real32_T LDPSC_SafeFuncActiveTurnOnDelay_sec;/* '<S117>/Unit Delay' */
real32_T LDPSC_SafeFuncErrorTurnOnDelay_sec;/* '<S118>/Unit Delay' */
real32_T LDPSC_SusTimeExpiredTimerRetrigger_sec;/* '<S30>/Unit Delay' */
boolean_T LDPSC_VehYawRateHyst_bool;   /* '<S119>/Unit Delay' */

/* Yawrate hysteresis--Used in LDPSC module */
real32_T LDPTT_LstLnBdryCurvRateRi_ReMi2;/* '<S127>/UnitDelay4' */

/* Last filtered right lane clothoid change of curvature from LDP */
real32_T LDPTT_LstLnBdryCurvRi_ReMi;   /* '<S127>/UnitDelay3' */

/* Last filtered right lane clothoid curvature from LDP */
real32_T LDPTT_LstLnBdryHeadAglRi_Rad; /* '<S127>/UnitDelay2' */

/* Last filtered right lane clothoid heading angle from LDP */
real32_T LDPTT_LstLnBdryPstnXRi_Mi;    /* '<S127>/UnitDelay' */

/* Last filtered right lane clothoid X0 position from LDP */
real32_T LDPTT_LstLnBdryPstnYRi_Mi;    /* '<S127>/UnitDelay1' */

/* Last filtered right lane clothoid Y0 position from LDP */
real32_T LDPTT_LstLnBdryVldLengRi_Mi;  /* '<S127>/UnitDelay5' */

/* Last filtered right lane clothoid length from LDP  */
uint8_T LDPVSE_PrevVehStartupSpd_Kmph; /* '<S171>/Unit Delay' */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Yawrate hysteresis--Used in LDPSC module */

/* Model step function */
void LDPSA_step(void)
{
  /* local block i/o variables */
  uint32_T rtb_ex_sfun_set_bit;
  boolean_T rtb_VectorConcatenate[8];
  int32_T LDPSC_TrigByPrjSpecRi_B_tmp;
  int32_T LDPVSE_NVRAMVehStartupSpd__osjy;
  real32_T rtb_Abs;
  real32_T rtb_Divide_l2e1;
  real32_T rtb_Subtract2_pfel;
  real32_T rtb_uDLookupTable8_idx_0;
  real32_T rtb_uDLookupTable9_idx_1;
  uint8_T rtb_UnitDelay_m0jr;
  boolean_T rtb_VectorConcatenate_k1un[9];
  boolean_T rtb_VectorConcatenate_otre[2];
  boolean_T rtb_AND_b1uz;
  boolean_T rtb_AND_ddra;
  boolean_T rtb_AND_ex2t;
  boolean_T rtb_Equal1_a4qg;
  boolean_T rtb_Equal3_newf;
  boolean_T rtb_Equal4;
  boolean_T rtb_LDPDT_LnCurvVldLf_B;
  boolean_T rtb_LDPDT_LnCurvVldRi_B;
  boolean_T rtb_LDPSC_DrvStInvalid_B;
  boolean_T rtb_LDPSC_LnCurveInvalid_B;
  boolean_T rtb_LDPSC_VehStInvalid_B;
  boolean_T rtb_LDPSC_VehicleInvalid_B;
  boolean_T rtb_LDPSC_VelYInvalid_B;
  boolean_T rtb_LogicalOperator14;
  boolean_T rtb_LogicalOperator14_fahu;
  boolean_T rtb_LogicalOperator2;
  boolean_T rtb_LogicalOperator8;
  boolean_T rtb_LogicalOperator_kelu;
  boolean_T rtb_NOT_fftx;
  boolean_T rtb_OR1_m0ph;
  boolean_T rtb_RelationalOperator10;
  boolean_T rtb_RelationalOperator10_gbg2;
  boolean_T rtb_RelationalOperator1_mqkp;
  boolean_T rtb_RelationalOperator29;
  boolean_T rtb_RelationalOperator3_k31o;
  boolean_T rtb_RelationalOperator4_jya3;

  /* S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/LDP'
   */
  /* Abs: '<S78>/Abs' incorporates:
   *  Inport: '<Root>/Inport25'
   *  Inport: '<Root>/Inport27'
   */
  LDPSC_CrvSensiDecayLe_Mi = fabsf(LDPSAI_CurvSafeLf_ReMi);
  rtb_uDLookupTable9_idx_1 = fabsf(LDPSAI_CurvSafeRi_ReMi);

  /* Lookup_n-D: '<S78>/1-D Lookup Table8' */
  rtb_uDLookupTable8_idx_0 = look1_iflf_binlxpw(LDPSC_CrvSensiDecayLe_Mi, ((const
    real32_T *)&(LDPSC_CrvSensiDecay_BX_ReMi[0])), ((const real32_T *)
    &(LDPSC_CrvSensiDecay_BY_Mi[0])), 3U);

  /* Lookup_n-D: '<S78>/1-D Lookup Table9' incorporates:
   *  Lookup_n-D: '<S78>/1-D Lookup Table8'
   *  UnaryMinus: '<S78>/Unary Minus'
   */
  LDPSC_CrvSensiDecayLe_Mi = -look1_iflf_binlxpw(LDPSC_CrvSensiDecayLe_Mi, ((
    const real32_T *)&(LDPSC_CrvSensiAdvance_BX_ReMi[0])), ((const real32_T *)
    &(LDPSC_CrvSensiAdvance_BY_Mi[0])), 3U);

  /* Switch: '<S11>/Switch2' incorporates:
   *  Constant: '<S11>/V_Parameter1'
   *  Inport: '<Root>/Inport26'
   *  RelationalOperator: '<S11>/GreaterThan3'
   *  RelationalOperator: '<S11>/Less Than2'
   *  Switch: '<S11>/Switch3'
   *  UnaryMinus: '<S11>/Unary Minus2'
   */
  if (LDPSAI_CurvRi_ReMi < (-LDPDT_CurveThd_C_St)) {
    /* Switch: '<S11>/Switch2' incorporates:
     *  Constant: '<S11>/V_Parameter11'
     */
    LDPDT_CurveTypeRi_St = LDPDT_CurveInner_C_St;
  } else if (LDPSAI_CurvRi_ReMi > LDPDT_CurveThd_C_St) {
    /* Switch: '<S11>/Switch3' incorporates:
     *  Constant: '<S11>/V_Parameter12'
     *  Switch: '<S11>/Switch2'
     */
    LDPDT_CurveTypeRi_St = LDPDT_CurveOuter_C_St;
  } else {
    /* Switch: '<S11>/Switch2' incorporates:
     *  Constant: '<S11>/V_Parameter13'
     *  Switch: '<S11>/Switch3'
     */
    LDPDT_CurveTypeRi_St = LDPDT_CurveNone_C_St;
  }

  /* End of Switch: '<S11>/Switch2' */

  /* Switch: '<S78>/Switch6' incorporates:
   *  Constant: '<S78>/V_Parameter3'
   *  Constant: '<S78>/V_Parameter4'
   *  RelationalOperator: '<S78>/Relational Operator4'
   *  RelationalOperator: '<S78>/Relational Operator5'
   *  Switch: '<S78>/Switch3'
   */
  if (LDPDT_CurveTypeRi_St == LDPDT_CurveInner_C_St) {
    /* Switch: '<S78>/Switch6' incorporates:
     *  Lookup_n-D: '<S78>/1-D Lookup Table8'
     */
    LDPSC_CrvSensiDecayRi_Mi = look1_iflf_binlxpw(rtb_uDLookupTable9_idx_1, ((
      const real32_T *)&(LDPSC_CrvSensiDecay_BX_ReMi[0])), ((const real32_T *)
      &(LDPSC_CrvSensiDecay_BY_Mi[0])), 3U);
  } else if (LDPDT_CurveTypeRi_St == LDPDT_CurveOuter_C_St) {
    /* Switch: '<S78>/Switch3' incorporates:
     *  Lookup_n-D: '<S78>/1-D Lookup Table8'
     *  Lookup_n-D: '<S78>/1-D Lookup Table9'
     *  Switch: '<S78>/Switch6'
     *  UnaryMinus: '<S78>/Unary Minus'
     */
    LDPSC_CrvSensiDecayRi_Mi = -look1_iflf_binlxpw(rtb_uDLookupTable9_idx_1, ((
      const real32_T *)&(LDPSC_CrvSensiAdvance_BX_ReMi[0])), ((const real32_T *)
      &(LDPSC_CrvSensiAdvance_BY_Mi[0])), 3U);
  } else {
    /* Switch: '<S78>/Switch6' incorporates:
     *  Constant: '<S78>/V_Const3'
     *  Switch: '<S78>/Switch3'
     */
    LDPSC_CrvSensiDecayRi_Mi = 0.0F;
  }

  /* End of Switch: '<S78>/Switch6' */

  /* Lookup_n-D: '<S78>/1-D Lookup Table3' incorporates:
   *  Inport: '<Root>/Inport13'
   */
  rtb_uDLookupTable9_idx_1 = look1_iflf_binlxpw(LDPSAI_LnWidCalc_Mi, ((const
    real32_T *)&(LDPSC_LaneWidth_BX_Mi[0])), ((const real32_T *)
    &(LDPSC_DTCFctLnWid_Cr_Fct[0])), 4U);

  /* RelationalOperator: '<S8>/Equal' incorporates:
   *  Constant: '<S8>/V_Parameter'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDPDT_RdyTrigLDP_B = (LDPSC_LstPrevDgrSide_St == LDPDT_NoDgrSide_C_St);

  /* Lookup_n-D: '<S13>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPDT_CrvThdMaxRi_ReMi = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDPDT_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDPDT_TrsdLnCltdCurvRiMx_Cr_Mps[0])), 7U);

  /* Lookup_n-D: '<S13>/1-D Lookup Table1' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPDT_CrvThdHystRi_ReMi = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDPDT_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDPDT_TrsdLnCltdCurvRiOfst_Cr_Mps[0])), 7U);

  /* Logic: '<S8>/AND' incorporates:
   *  Constant: '<S8>/V_Parameter1'
   */
  LDPDT_EnaSafety_B = ((LDPDT_SfFcLDPOn_C_B) && LDPDT_RdyTrigLDP_B);

  /* Switch: '<S8>/Switch5' */
  if (LDPDT_EnaSafety_B) {
    /* Switch: '<S8>/Switch5' incorporates:
     *  Inport: '<Root>/Inport27'
     */
    LDPDT_LnCltdCurvRi_ReMi = LDPSAI_CurvSafeRi_ReMi;
  } else {
    /* Switch: '<S8>/Switch5' incorporates:
     *  Inport: '<Root>/Inport26'
     */
    LDPDT_LnCltdCurvRi_ReMi = LDPSAI_CurvRi_ReMi;
  }

  /* End of Switch: '<S8>/Switch5' */

  /* Switch: '<S19>/Switch' incorporates:
   *  Sum: '<S19>/Add'
   *  UnitDelay: '<S19>/Unit Delay'
   */
  if (LDPDT_UHysCltdCurvVldRi_B) {
    rtb_Abs = LDPDT_CrvThdHystRi_ReMi + LDPDT_CrvThdMaxRi_ReMi;
  } else {
    rtb_Abs = LDPDT_CrvThdMaxRi_ReMi;
  }

  /* End of Switch: '<S19>/Switch' */

  /* RelationalOperator: '<S19>/GreaterThan' incorporates:
   *  Abs: '<S13>/Abs'
   *  UnitDelay: '<S19>/Unit Delay'
   */
  LDPDT_UHysCltdCurvVldRi_B = (rtb_Abs > fabsf(LDPDT_LnCltdCurvRi_ReMi));

  /* Logic: '<S13>/AND1' incorporates:
   *  Constant: '<S13>/V_Parameter7'
   *  Inport: '<Root>/Inport44'
   *  Logic: '<S13>/AND'
   *  Logic: '<S13>/NOT'
   *  UnitDelay: '<S19>/Unit Delay'
   */
  rtb_LDPDT_LnCurvVldRi_B = (((!LDPSAI_DtctCstruSite_B) ||
    (!LDPDT_CstruSiteLDP_C_B)) && LDPDT_UHysCltdCurvVldRi_B);

  /* Switch: '<S8>/Switch1' */
  if (LDPDT_EnaSafety_B) {
    /* Switch: '<S8>/Switch1' incorporates:
     *  Inport: '<Root>/Inport23'
     */
    LDPDT_LnHeadRi_Rad = LDPSAI_HeadAglSafeRi_Rad;
  } else {
    /* Switch: '<S8>/Switch1' incorporates:
     *  Inport: '<Root>/Inport22'
     */
    LDPDT_LnHeadRi_Rad = LDPSAI_HeadAglRi_Rad;
  }

  /* End of Switch: '<S8>/Switch1' */

  /* Switch: '<S18>/Switch' incorporates:
   *  Constant: '<S13>/V_Parameter10'
   *  Constant: '<S13>/V_Parameter11'
   *  Constant: '<S13>/V_Parameter12'
   *  Sum: '<S18>/Add'
   *  Sum: '<S18>/Add1'
   *  Switch: '<S18>/Switch1'
   *  UnitDelay: '<S18>/Unit Delay'
   */
  if (LDPDT_BHysHeadAglTrigVldRi_B) {
    rtb_Abs = LDPDT_TrsdHeadAglMx_C_Rad + LDPDT_TrsdHeadAglOfst_C_Rad;
    rtb_Divide_l2e1 = LDPDT_TrsdHeadAglMn_C_Rad - LDPDT_TrsdHeadAglOfst_C_Rad;
  } else {
    rtb_Abs = LDPDT_TrsdHeadAglMx_C_Rad;
    rtb_Divide_l2e1 = LDPDT_TrsdHeadAglMn_C_Rad;
  }

  /* End of Switch: '<S18>/Switch' */

  /* Logic: '<S18>/AND' incorporates:
   *  RelationalOperator: '<S18>/GreaterThan'
   *  RelationalOperator: '<S18>/GreaterThan1'
   *  UnitDelay: '<S18>/Unit Delay'
   */
  LDPDT_BHysHeadAglTrigVldRi_B = ((rtb_Abs > LDPDT_LnHeadRi_Rad) &&
    (LDPDT_LnHeadRi_Rad > rtb_Divide_l2e1));

  /* RelationalOperator: '<S13>/Relational Operator4' incorporates:
   *  Constant: '<S13>/Constant2'
   *  Constant: '<S13>/V_Parameter4'
   *  Inport: '<Root>/Inport37'
   *  S-Function (sfix_bitop): '<S13>/Bitwise Operator1'
   */
  LDPDT_EnaByInVldQlfrRi_B = ((((int32_T)LDPSAI_LnIVldRi_St) & ((int32_T)
    LDPDT_LnIvldRi_C_St)) == 0);

  /* RelationalOperator: '<S13>/Relational Operator1' incorporates:
   *  Constant: '<S13>/Constant1'
   *  Constant: '<S13>/V_Parameter3'
   *  Inport: '<Root>/Inport36'
   *  S-Function (sfix_bitop): '<S13>/Bitwise Operator'
   */
  LDPDT_EnaByInVldQlfrSfRi_B = ((((int32_T)LDPSAI_IvldLnSafeRi_St) & ((int32_T)
    LDPDT_LnIvldSfRi_C_St)) == 0);

  /* Switch: '<S21>/Switch' incorporates:
   *  Constant: '<S13>/Constant'
   *  UnitDelay: '<S21>/Unit Delay'
   */
  if (LDPDT_LnLengthRi_B) {
    LDPVSE_NVRAMVehStartupSpd__osjy = 11;
  } else {
    LDPVSE_NVRAMVehStartupSpd__osjy = 10;
  }

  /* End of Switch: '<S21>/Switch' */

  /* RelationalOperator: '<S21>/GreaterThan' incorporates:
   *  Inport: '<Root>/Inport33'
   *  UnitDelay: '<S21>/Unit Delay'
   */
  LDPDT_LnLengthRi_B = (((real32_T)LDPVSE_NVRAMVehStartupSpd__osjy) >
                        LDPSAI_VldLengRi_Mi);

  /* Logic: '<S13>/NOT1' incorporates:
   *  UnitDelay: '<S21>/Unit Delay'
   */
  rtb_LDPSC_VehicleInvalid_B = !LDPDT_LnLengthRi_B;

  /* Switch: '<S13>/Switch1' incorporates:
   *  Constant: '<S13>/V_Parameter1'
   *  Logic: '<S13>/Logical Operator'
   */
  if (LDPDT_SfFcLDPOn_C_B) {
    rtb_RelationalOperator1_mqkp = (LDPDT_EnaByInVldQlfrRi_B &&
      LDPDT_EnaByInVldQlfrSfRi_B);
  } else {
    rtb_RelationalOperator1_mqkp = LDPDT_EnaByInVldQlfrRi_B;
  }

  /* End of Switch: '<S13>/Switch1' */

  /* Logic: '<S13>/Logical Operator2' incorporates:
   *  UnitDelay: '<S18>/Unit Delay'
   */
  LDPDT_LnTrigVldRi_B = (((rtb_LDPDT_LnCurvVldRi_B &&
    LDPDT_BHysHeadAglTrigVldRi_B) && rtb_RelationalOperator1_mqkp) &&
    rtb_LDPSC_VehicleInvalid_B);

  /* Switch: '<S20>/Switch' incorporates:
   *  Constant: '<S13>/V_Parameter5'
   *  Constant: '<S13>/V_Parameter8'
   *  Sum: '<S20>/Add'
   *  UnitDelay: '<S20>/Unit Delay'
   */
  if (LDPDT_UHysHeadAglCclVldRi_B) {
    rtb_Abs = LDPDT_TrsdHeadAglOfst_C_Rad + LDPDT_TrsdHeadAglMx_C_Rad;
  } else {
    rtb_Abs = LDPDT_TrsdHeadAglMx_C_Rad;
  }

  /* End of Switch: '<S20>/Switch' */

  /* RelationalOperator: '<S20>/GreaterThan' incorporates:
   *  Abs: '<S13>/Abs2'
   *  UnitDelay: '<S20>/Unit Delay'
   */
  LDPDT_UHysHeadAglCclVldRi_B = (rtb_Abs > fabsf(LDPDT_LnHeadRi_Rad));

  /* RelationalOperator: '<S13>/Relational Operator5' incorporates:
   *  Constant: '<S13>/Constant3'
   *  Constant: '<S13>/V_Parameter2'
   *  Inport: '<Root>/Inport37'
   *  S-Function (sfix_bitop): '<S13>/Bitwise Operator2'
   */
  LDPDT_CclByInVldQlfrRi_B = ((((int32_T)LDPSAI_LnIVldRi_St) & ((int32_T)
    LDPDT_LnIvldCclRi_C_St)) == 0);

  /* Logic: '<S13>/Logical Operator5' incorporates:
   *  UnitDelay: '<S20>/Unit Delay'
   */
  LDPDT_LnCclVldRi_B = (((rtb_LDPDT_LnCurvVldRi_B && LDPDT_UHysHeadAglCclVldRi_B)
    && LDPDT_CclByInVldQlfrRi_B) && rtb_LDPSC_VehicleInvalid_B);

  /* Switch: '<S13>/Switch' */
  if (LDPDT_RdyTrigLDP_B) {
    /* Switch: '<S13>/Switch' */
    LDPDT_LnMakVldRi_B = LDPDT_LnTrigVldRi_B;
  } else {
    /* Switch: '<S13>/Switch' */
    LDPDT_LnMakVldRi_B = LDPDT_LnCclVldRi_B;
  }

  /* End of Switch: '<S13>/Switch' */

  /* Product: '<S10>/Product1' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPDT_RawLatVehSpdRi_Mps = LDPDT_LnHeadRi_Rad * LDPSAI_VehSpdActu_Mps;

  /* Switch: '<S10>/Switch2' */
  if (LDPDT_LnMakVldRi_B) {
    /* Switch: '<S10>/Switch2' */
    LDPDT_LatVehSpdRi_Mps = LDPDT_RawLatVehSpdRi_Mps;
  } else {
    /* Switch: '<S10>/Switch2' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    LDPDT_LatVehSpdRi_Mps = 10.0F;
  }

  /* End of Switch: '<S10>/Switch2' */

  /* Lookup_n-D: '<S78>/1-D Lookup Table4' incorporates:
   *  Inport: '<Root>/Inport27'
   */
  LDPSC_DstcToLnTrsdCrvCpstnRi_Mi = look1_iflf_binlxpw(LDPSAI_CurvSafeRi_ReMi, ((
    const real32_T *)&(LDPSC_LnDectCrvRi_BX_ReMi[0])), ((const real32_T *)
    &(LDPSC_DstcToLnTrsdOfstRi_Cr_Mi[0])), 16U);

  /* Sum: '<S78>/Add3' incorporates:
   *  Lookup_n-D: '<S78>/1-D Lookup Table11'
   *  Product: '<S78>/Product3'
   *  Sum: '<S78>/Add1'
   *  Switch: '<S10>/Switch2'
   */
  LDPSC_DstcToLnTrsdRi_Mi = ((rtb_uDLookupTable9_idx_1 * look1_iflf_binlxpw
    (LDPDT_LatVehSpdRi_Mps, (&(LDPSC_LnLatVeh_BX_Mps[0])),
     (&(LDPSC_LnLatVeh_Ri_Mps[0])), 8U)) + LDPSC_DstcToLnTrsdCrvCpstnRi_Mi) -
    LDPSC_CrvSensiDecayRi_Mi;

  /* Switch: '<S8>/Switch3' */
  if (LDPDT_EnaSafety_B) {
    /* Switch: '<S8>/Switch3' incorporates:
     *  Inport: '<Root>/Inport19'
     */
    LDPDT_LnPstnRi_Mi = LDPSAI_PstnYSafeRi_Mi;
  } else {
    /* Switch: '<S8>/Switch3' incorporates:
     *  Inport: '<Root>/Inport18'
     */
    LDPDT_LnPstnRi_Mi = LDPSAI_PstnYRi_Mi;
  }

  /* End of Switch: '<S8>/Switch3' */

  /* Product: '<S10>/Divide' incorporates:
   *  Constant: '<S10>/Constant'
   *  Inport: '<Root>/Inport'
   */
  rtb_Subtract2_pfel = LDPSAI_VehWid_Mi / 2.0F;

  /* Sum: '<S10>/Subtract1' */
  LDPDT_RawDstcToLnRi_Mi = LDPDT_LnPstnRi_Mi + rtb_Subtract2_pfel;

  /* Switch: '<S10>/Switch5' */
  if (LDPDT_LnMakVldRi_B) {
    /* Switch: '<S10>/Switch5' */
    LDPDT_DstcToLnRi_Mi = LDPDT_RawDstcToLnRi_Mi;
  } else {
    /* Switch: '<S10>/Switch5' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    LDPDT_DstcToLnRi_Mi = -10.0F;
  }

  /* End of Switch: '<S10>/Switch5' */

  /* RelationalOperator: '<S80>/Relational Operator3' incorporates:
   *  UnaryMinus: '<S80>/Unary Minus'
   */
  LDPSC_RawTrigByDlcRi_B = (LDPDT_DstcToLnRi_Mi >= (-LDPSC_DstcToLnTrsdRi_Mi));

  /* RelationalOperator: '<S80>/Relational Operator1' incorporates:
   *  Constant: '<S80>/V_Parameter10'
   *  UnaryMinus: '<S80>/Unary Minus1'
   */
  LDPSC_EnaTlcTrigRi_B = (LDPDT_DstcToLnRi_Mi > (-LDPSC_DstcOfTiToLnMn_C_Mi));

  /* Switch: '<S10>/Switch4' incorporates:
   *  Constant: '<S10>/Constant10'
   *  Constant: '<S10>/V_Parameter2'
   *  Inport: '<Root>/Inport1'
   *  Logic: '<S10>/Logical Operator1'
   *  RelationalOperator: '<S10>/Relational Operator2'
   *  RelationalOperator: '<S10>/Relational Operator3'
   */
  if (((LDPDT_LnHeadRi_Rad > LDPDT_TLCHeadAglTrsd_C_Rad) &&
       (LDPSAI_VehSpdActu_Mps > 0.0F)) && LDPDT_LnMakVldRi_B) {
    /* Switch: '<S10>/Switch4' incorporates:
     *  Abs: '<S10>/Abs1'
     *  Constant: '<S10>/Constant11'
     *  Constant: '<S10>/Constant9'
     *  MinMax: '<S10>/MinMax3'
     *  MinMax: '<S10>/MinMax4'
     *  Product: '<S10>/Divide2'
     *  UnaryMinus: '<S10>/Unary Minus1'
     */
    LDPDT_TiToLnRi_Sec = fminf((-LDPDT_DstcToLnRi_Mi) / fmaxf(fabsf
      (LDPDT_LatVehSpdRi_Mps), 0.01F), 10.0F);
  } else {
    /* Switch: '<S10>/Switch4' incorporates:
     *  Constant: '<S10>/Constant12'
     */
    LDPDT_TiToLnRi_Sec = 10.0F;
  }

  /* End of Switch: '<S10>/Switch4' */

  /* Product: '<S78>/Product1' incorporates:
   *  Inport: '<Root>/Inport1'
   *  Inport: '<Root>/Inport13'
   *  Lookup_n-D: '<S78>/1-D Lookup Table6'
   *  Lookup_n-D: '<S78>/1-D Lookup Table7'
   */
  LDPSC_TiToLnTrsd_Sec = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDPSC_VehSpdXTTL_BX_Mps[0])), ((const real32_T *)
    &(LDPSC_TiToLnTrsdSpd_Cr_Sec[0])), 8U) * look1_iflf_binlxpw
    (LDPSAI_LnWidCalc_Mi, ((const real32_T *)&(LDPSC_LaneWidth_BX_Mi[0])), ((
       const real32_T *)&(LDPSC_TiToLnTrsdWdh_Cr_Sec[0])), 4U);

  /* RelationalOperator: '<S80>/Relational Operator2' */
  LDPSC_RawTrigByTlcRi_B = (LDPDT_TiToLnRi_Sec < LDPSC_TiToLnTrsd_Sec);

  /* Switch: '<S106>/Switch' incorporates:
   *  Constant: '<S80>/V_Parameter12'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S106>/Max'
   *  Sum: '<S106>/Subtract'
   *  Switch: '<S106>/Switch1'
   *  UnaryMinus: '<S106>/Unary Minus'
   *  UnitDelay: '<S106>/Unit Delay'
   */
  if (LDPSC_RawTrigByTlcRi_B) {
    LDPSC_DlyTiOfTiToLnRiMn_Sec = fmaxf(LDPSC_DlyTiOfTiToLnRiMn_Sec,
      -LDPSAI_CycleTime_Sec) - LDPSAI_CycleTime_Sec;
  } else {
    LDPSC_DlyTiOfTiToLnRiMn_Sec = LDPSC_DlyTiOfTiToLnMn_C_Sec;
  }

  /* End of Switch: '<S106>/Switch' */

  /* Logic: '<S106>/AND' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S106>/LessThanOrEqual'
   *  UnaryMinus: '<S106>/Unary Minus1'
   *  UnitDelay: '<S106>/Unit Delay'
   */
  LDPSC_DlyTrigByTlcRi_B = (LDPSC_RawTrigByTlcRi_B &&
    (LDPSC_DlyTiOfTiToLnRiMn_Sec <= (-LDPSAI_CycleTime_Sec)));

  /* Logic: '<S80>/Logical Operator2' incorporates:
   *  Constant: '<S80>/V_Parameter11'
   *  Constant: '<S80>/V_Parameter9'
   *  DataTypeConversion: '<S96>/Data Type Conversion'
   *  DataTypeConversion: '<S97>/Data Type Conversion'
   *  Logic: '<S80>/Logical Operator'
   *  Logic: '<S80>/Logical Operator1'
   *  S-Function (sfix_bitop): '<S96>/Bitwise AND'
   *  S-Function (sfix_bitop): '<S97>/Bitwise AND'
   */
  rtb_LogicalOperator2 = ((((((uint32_T)LDPSC_TrigCdtnEn_C_St) & 4U) != 0U) &&
    LDPSC_RawTrigByDlcRi_B) || ((((((uint32_T)LDPSC_TrigCdtnEn_C_St) & 8U) != 0U)
    && LDPSC_EnaTlcTrigRi_B) && LDPSC_DlyTrigByTlcRi_B));

  /* Logic: '<S95>/AND' incorporates:
   *  Logic: '<S95>/NOT'
   *  UnitDelay: '<S95>/Unit Delay'
   */
  LDPSC_EnaLdwTrigRi_B = (rtb_LogicalOperator2 && (!LDPSC_HdTiTrigRiEn_B));

  /* Logic: '<S80>/OR' incorporates:
   *  Constant: '<S80>/V_Parameter14'
   *  Constant: '<S80>/V_Parameter2'
   *  RelationalOperator: '<S80>/GreaterThan'
   *  RelationalOperator: '<S80>/Relational Operator4'
   *  Sum: '<S80>/Add1'
   *  UnaryMinus: '<S80>/Unary Minus2'
   *  UnaryMinus: '<S80>/Unary Minus6'
   */
  LDPSC_RstLdwTrigRi_B = ((((-LDPSC_DstcToLnTrsdRi_Mi) - 0.12F) >
    LDPDT_DstcToLnRi_Mi) || (LDPDT_DstcToLnRi_Mi >
    (-LDPSC_DstcOfDiscToLnLmtMn_C_Mi)));

  /* Switch: '<S104>/Switch4' incorporates:
   *  Switch: '<S104>/Switch3'
   */
  if (LDPSC_RstLdwTrigRi_B) {
    /* Sum: '<S104>/Subtract2' incorporates:
     *  Constant: '<S104>/Constant3'
     */
    LDPSC_HdTiTrigRi_Sec = 0.0F;
  } else {
    if (LDPSC_EnaLdwTrigRi_B) {
      /* Sum: '<S104>/Subtract2' incorporates:
       *  Constant: '<S80>/V_Parameter15'
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S104>/Subtract1'
       *  Switch: '<S104>/Switch3'
       */
      LDPSC_HdTiTrigRi_Sec = LDPSC_HdTiTrigLf_C_Sec + LDPSAI_CycleTime_Sec;
    }
  }

  /* End of Switch: '<S104>/Switch4' */

  /* SignalConversion: '<S80>/Signal Conversion' incorporates:
   *  Constant: '<S104>/Constant4'
   *  Logic: '<S104>/OR'
   *  RelationalOperator: '<S104>/GreaterThan2'
   */
  LDPSC_HoldLdwTrigRi_B = (LDPSC_EnaLdwTrigRi_B || (LDPSC_HdTiTrigRi_Sec >
    1.0E-5F));

  /* Logic: '<S80>/Logical Operator5' incorporates:
   *  Logic: '<S39>/Logical Operator21'
   */
  LDPSC_ErrSideByTrigRi_B = !LDPDT_LnMakVldRi_B;

  /* Logic: '<S80>/Logical Operator4' incorporates:
   *  Constant: '<S100>/Constant'
   *  Inport: '<Root>/Inport12'
   *  Logic: '<S80>/Logical Operator5'
   *  RelationalOperator: '<S80>/Relational Operator5'
   */
  LDPSC_ResetForSafeRi_B = ((LDPSAI_DtctLnChag_B || LDPSC_ErrSideByTrigRi_B) ||
    (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_ACTIVE));

  /* RelationalOperator: '<S80>/Relational Operator6' incorporates:
   *  Constant: '<S80>/V_Parameter13'
   *  Sum: '<S80>/Add'
   *  UnaryMinus: '<S80>/Unary Minus3'
   */
  LDPSC_SetForSafeRi_B = (LDPDT_DstcToLnRi_Mi < ((-LDPSC_DstcToLnTrsdRi_Mi) -
    LDPSC_DstcOfstSafeSitu_C_Mi));

  /* Switch: '<S102>/Switch' incorporates:
   *  Constant: '<S102>/Constant2'
   *  Switch: '<S102>/Switch1'
   *  UnitDelay: '<S102>/Unit Delay'
   */
  if (LDPSC_ResetForSafeRi_B) {
    LDPSC_DisTrigRi_B = false;
  } else {
    LDPSC_DisTrigRi_B = (LDPSC_SetForSafeRi_B || LDPSC_DisTrigRi_B);
  }

  /* End of Switch: '<S102>/Switch' */

  /* Logic: '<S80>/Logical Operator6' incorporates:
   *  UnitDelay: '<S102>/Unit Delay'
   */
  rtb_LDPSC_VehicleInvalid_B = (LDPSC_HoldLdwTrigRi_B && LDPSC_DisTrigRi_B);

  /* Logic: '<S80>/Logical Operator17' incorporates:
   *  Constant: '<S80>/V_Parameter4'
   *  RelationalOperator: '<S80>/Relational Operator11'
   *  RelationalOperator: '<S80>/Relational Operator13'
   *  UnaryMinus: '<S80>/Unary Minus4'
   *  UnaryMinus: '<S80>/Unary Minus5'
   */
  rtb_LDPSC_VelYInvalid_B = ((LDPDT_DstcToLnRi_Mi > (-LDPSC_DstcToLnTrsdRi_Mi)) &&
    (LDPDT_DstcToLnRi_Mi < (-LDPSC_DstcOfDiscToLnLmtMn_C_Mi)));

  /* RelationalOperator: '<S80>/Relational Operator10' incorporates:
   *  Constant: '<S101>/Constant'
   */
  rtb_RelationalOperator10 = (((uint32_T)LDPSC_SysOld_St) ==
    E_LDPState_nu_ACTIVE);

  /* Logic: '<S80>/Logical Operator11' incorporates:
   *  Logic: '<S80>/AND'
   */
  rtb_RelationalOperator1_mqkp = !rtb_LDPSC_VelYInvalid_B;

  /* Switch: '<S80>/Switch1' incorporates:
   *  Constant: '<S80>/Constant3'
   *  Logic: '<S80>/Logical Operator10'
   *  Logic: '<S80>/Logical Operator11'
   *  Logic: '<S98>/AND'
   *  Logic: '<S98>/NOT'
   *  Sum: '<S80>/Add2'
   *  UnitDelay: '<S80>/Unit Delay'
   *  UnitDelay: '<S80>/Unit Delay2'
   *  UnitDelay: '<S98>/Unit Delay'
   */
  if (LDPSC_SuppFlagOldRi_B || rtb_RelationalOperator1_mqkp) {
    LDPSC_ContinWarmTimesOldRi_Count = 0.0F;
  } else {
    LDPSC_ContinWarmTimesOldRi_Count += (real32_T)((rtb_RelationalOperator10 &&
      (!LDPSC_PreActiveEdgeRi)) ? 1 : 0);
  }

  /* End of Switch: '<S80>/Switch1' */

  /* RelationalOperator: '<S80>/Relational Operator12' incorporates:
   *  Constant: '<S80>/V_Parameter5'
   *  UnitDelay: '<S80>/Unit Delay'
   *  UnitDelay: '<S80>/Unit Delay2'
   */
  LDPSC_SuppFlagOldRi_B = (LDPSC_ContinWarmTimesOldRi_Count >=
    LDPSC_ContinWarmTimes_C_Count);

  /* Switch: '<S105>/Switch4' incorporates:
   *  Logic: '<S80>/Logical Operator16'
   *  Switch: '<S105>/Switch3'
   *  UnitDelay: '<S80>/Unit Delay'
   */
  if (!rtb_LDPSC_VelYInvalid_B) {
    /* Sum: '<S105>/Subtract2' incorporates:
     *  Constant: '<S105>/Constant3'
     */
    LDPSC_SuppTimeOldRi_Sec = 0.0F;
  } else {
    if (LDPSC_SuppFlagOldRi_B) {
      /* Sum: '<S105>/Subtract2' incorporates:
       *  Constant: '<S80>/V_Parameter1'
       *  Constant: '<S80>/V_Parameter3'
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S105>/Subtract1'
       *  Sum: '<S80>/Add3'
       *  Switch: '<S105>/Switch3'
       */
      LDPSC_SuppTimeOldRi_Sec = (LDPSC_ContinWarmSupp_C_Sec +
        LDPSC_WarmMxTi_C_Sec) + LDPSAI_CycleTime_Sec;
    }
  }

  /* End of Switch: '<S105>/Switch4' */

  /* Logic: '<S80>/Logical Operator14' incorporates:
   *  Constant: '<S105>/Constant4'
   *  Logic: '<S105>/OR'
   *  RelationalOperator: '<S105>/GreaterThan2'
   *  UnitDelay: '<S80>/Unit Delay'
   */
  rtb_LogicalOperator14 = ((!LDPSC_SuppFlagOldRi_B) && (LDPSC_SuppTimeOldRi_Sec <=
    1.0E-5F));

  /* Logic: '<S80>/Logical Operator13' incorporates:
   *  Logic: '<S99>/AND'
   *  Logic: '<S99>/NOT'
   *  UnitDelay: '<S99>/Unit Delay'
   */
  LDPSC_SetForContinTrigRi_B = (rtb_LDPSC_VehicleInvalid_B ||
    (rtb_LogicalOperator14 && (!LDPSC_ContinTrigRiEn_B)));

  /* Logic: '<S80>/Logical Operator18' incorporates:
   *  Logic: '<S80>/AND'
   */
  LDPSC_ResetForContinTrigRi_B = (rtb_RelationalOperator1_mqkp ||
    (!rtb_LogicalOperator14));

  /* Switch: '<S103>/Switch' incorporates:
   *  Constant: '<S103>/Constant2'
   *  Switch: '<S103>/Switch1'
   *  UnitDelay: '<S103>/Unit Delay'
   */
  if (LDPSC_ResetForContinTrigRi_B) {
    LDPSC_DisContinTrigRi_B = false;
  } else {
    LDPSC_DisContinTrigRi_B = (LDPSC_SetForContinTrigRi_B ||
      LDPSC_DisContinTrigRi_B);
  }

  /* End of Switch: '<S103>/Switch' */

  /* RelationalOperator: '<S161>/Relational Operator4' incorporates:
   *  Constant: '<S161>/V_Parameter5'
   *  Inport: '<Root>/Inport6'
   */
  rtb_RelationalOperator4_jya3 = (LDPSAI_TrnSgl_St == LDPVSE_TrnSglRi_C_St);

  /* RelationalOperator: '<S161>/Relational Operator3' incorporates:
   *  Constant: '<S161>/V_Parameter4'
   *  Inport: '<Root>/Inport6'
   */
  rtb_RelationalOperator3_k31o = (LDPSAI_TrnSgl_St == LDPVSE_TrnSglLf_C_St);

  /* Logic: '<S174>/AND' incorporates:
   *  Logic: '<S174>/NOT'
   *  UnitDelay: '<S174>/Unit Delay'
   */
  LDPVSE_EdgeRiseTurnSglRi_B = (rtb_RelationalOperator3_k31o &&
    (!LDPVSE_EdgeRisTrnSglLf_B));

  /* Switch: '<S176>/Switch4' incorporates:
   *  Constant: '<S161>/V_Parameter6'
   *  Switch: '<S161>/Switch1'
   *  Switch: '<S176>/Switch3'
   */
  if ((LDPVSE_TrnSglRstRiEn_C_B) && LDPVSE_EdgeRiseTurnSglRi_B) {
    /* Sum: '<S176>/Subtract2' incorporates:
     *  Constant: '<S176>/Constant3'
     */
    LDPVSE_HodTiTrnSglRi_Sec = 0.0F;
  } else {
    if (rtb_RelationalOperator4_jya3) {
      /* Sum: '<S176>/Subtract2' incorporates:
       *  Constant: '<S161>/V_Parameter7'
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S176>/Subtract1'
       *  Switch: '<S176>/Switch3'
       */
      LDPVSE_HodTiTrnSglRi_Sec = LDPVSE_HodTiTrnSgl_C_Sec + LDPSAI_CycleTime_Sec;
    }
  }

  /* End of Switch: '<S176>/Switch4' */

  /* SignalConversion: '<S161>/Signal Conversion2' incorporates:
   *  Constant: '<S176>/Constant4'
   *  Logic: '<S176>/OR'
   *  RelationalOperator: '<S176>/GreaterThan2'
   */
  LDPVSE_TrnSglRi_B = (rtb_RelationalOperator4_jya3 || (LDPVSE_HodTiTrnSglRi_Sec
    > 1.0E-5F));

  /* SignalConversion: '<S162>/Signal Conversion3' */
  rtb_VectorConcatenate_otre[0] = LDPVSE_TrnSglRi_B;

  /* Lookup_n-D: '<S162>/Lookup Table' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPVSE_MaxLatVel_Mps = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDPVSE_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDPVSE_VehLatTrsd_Cr_Msp[0])), 7U);

  /* Switch: '<S185>/Switch' incorporates:
   *  Constant: '<S180>/V_Parameter10'
   *  Constant: '<S180>/V_Parameter11'
   *  Sum: '<S185>/Add'
   *  Sum: '<S185>/Add1'
   *  Switch: '<S185>/Switch1'
   *  UnitDelay: '<S185>/Unit Delay'
   */
  if (LDPVSE_BHysLatVehSpdVldRi_B) {
    rtb_Abs = LDPVSE_MaxLatVel_Mps + LDPVSE_VehLatTrsdLDPOfst_C_Msp;
    rtb_Divide_l2e1 = LDPVSE_VehLatTrsdLDPMn_C_Msp -
      LDPVSE_VehLatTrsdLDPOfst_C_Msp;
  } else {
    rtb_Abs = LDPVSE_MaxLatVel_Mps;
    rtb_Divide_l2e1 = LDPVSE_VehLatTrsdLDPMn_C_Msp;
  }

  /* End of Switch: '<S185>/Switch' */

  /* Logic: '<S185>/AND' incorporates:
   *  RelationalOperator: '<S185>/GreaterThan'
   *  RelationalOperator: '<S185>/GreaterThan1'
   *  UnitDelay: '<S185>/Unit Delay'
   */
  LDPVSE_BHysLatVehSpdVldRi_B = ((rtb_Abs > LDPDT_LatVehSpdRi_Mps) &&
    (LDPDT_LatVehSpdRi_Mps > rtb_Divide_l2e1));

  /* RelationalOperator: '<S162>/Equal' incorporates:
   *  Constant: '<S162>/V_Parameter'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDPVSE_RdyTrigLDW_B = (LDPSC_LstPrevDgrSide_St == LDPVSE_NoDgrSide_C_St);

  /* Switch: '<S186>/Switch' incorporates:
   *  Constant: '<S180>/V_Parameter7'
   *  Constant: '<S180>/V_Parameter8'
   *  Sum: '<S186>/Add'
   *  UnitDelay: '<S186>/Unit Delay'
   */
  if (LDPVSE_UHysLatVehSpdVldRi_B) {
    rtb_Abs = LDPVSE_VehLatTrsdLDPOfst_C_Msp + LDPVSE_VehLatTrsdLDPMx_C_Msp;
  } else {
    rtb_Abs = LDPVSE_VehLatTrsdLDPMx_C_Msp;
  }

  /* End of Switch: '<S186>/Switch' */

  /* RelationalOperator: '<S186>/GreaterThan' incorporates:
   *  Abs: '<S180>/Abs1'
   *  UnitDelay: '<S186>/Unit Delay'
   */
  LDPVSE_UHysLatVehSpdVldRi_B = (rtb_Abs > fabsf(LDPDT_LatVehSpdRi_Mps));

  /* Switch: '<S180>/Switch' */
  if (LDPVSE_RdyTrigLDW_B) {
    /* Switch: '<S180>/Switch' incorporates:
     *  UnitDelay: '<S185>/Unit Delay'
     */
    LDPVSE_VehLatSpdVldRi_B = LDPVSE_BHysLatVehSpdVldRi_B;
  } else {
    /* Switch: '<S180>/Switch' incorporates:
     *  UnitDelay: '<S186>/Unit Delay'
     */
    LDPVSE_VehLatSpdVldRi_B = LDPVSE_UHysLatVehSpdVldRi_B;
  }

  /* End of Switch: '<S180>/Switch' */

  /* Logic: '<S162>/Logical Operator1' */
  rtb_VectorConcatenate_otre[1] = !LDPVSE_VehLatSpdVldRi_B;

  /* S-Function (ex_sfun_set_bit): '<S183>/ex_sfun_set_bit' incorporates:
   *  Constant: '<S178>/Constant'
   */
  set_bit(0U, (boolean_T*)&rtb_VectorConcatenate_otre[0], (uint8_T*)
          (&(LDPSA_SetBit_BS_Param_3[0])), ((uint8_T)2U), &rtb_ex_sfun_set_bit);

  /* SignalConversion: '<S162>/Signal Conversion1' incorporates:
   *  DataTypeConversion: '<S183>/Data Type Conversion1'
   */
  LDPVSE_SidCdtnLDPRi_St = (uint8_T)rtb_ex_sfun_set_bit;

  /* RelationalOperator: '<S80>/Relational Operator7' incorporates:
   *  Constant: '<S80>/V_Const'
   *  Constant: '<S80>/V_Const2'
   *  S-Function (sfix_bitop): '<S80>/Bitwise AND1'
   */
  LDPSC_TrigBySideCondRi_B = ((((uint32_T)LDPVSE_SidCdtnLDPRi_St) & 3U) == 0U);

  /* S-Function (sfix_bitop): '<S80>/Bitwise AND' incorporates:
   *  Constant: '<S80>/V_Parameter'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S79>/Bitwise AND'
   */
  LDPSC_TrigByPrjSpecRi_B_tmp = ((int32_T)LDPSAI_PrjSpecQu_St) & ((int32_T)
    LDPSC_PrjSpecQu_C_St);

  /* RelationalOperator: '<S80>/Equal' incorporates:
   *  Constant: '<S80>/V_Const1'
   *  S-Function (sfix_bitop): '<S80>/Bitwise AND'
   */
  LDPSC_TrigByPrjSpecRi_B = (LDPSC_TrigByPrjSpecRi_B_tmp == 0);

  /* Logic: '<S80>/Logical Operator3' incorporates:
   *  Logic: '<S80>/Logical Operator12'
   *  UnitDelay: '<S103>/Unit Delay'
   */
  LDPSC_TrigRi_B = (((rtb_LDPSC_VehicleInvalid_B || LDPSC_DisContinTrigRi_B) &&
                     LDPSC_TrigBySideCondRi_B) && LDPSC_TrigByPrjSpecRi_B);

  /* Logic: '<S12>/AND' incorporates:
   *  Constant: '<S12>/V_Parameter7'
   *  Inport: '<Root>/Inport44'
   */
  LDPDT_EnaByCstruSiteLf_B = (LDPSAI_DtctCstruSite_B && (LDPDT_CstruSiteLDP_C_B));

  /* Lookup_n-D: '<S12>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPDT_CrvThdMaxLf_ReMi = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDPDT_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDPDT_TrsdLnCltdCurvLfMx_Cr_Mps[0])), 7U);

  /* Lookup_n-D: '<S12>/1-D Lookup Table1' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPDT_CrvThdHystLf_ReMi = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDPDT_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDPDT_TrsdLnCltdCurvLfOfst_Cr_Mps[0])), 7U);

  /* Switch: '<S8>/Switch4' */
  if (LDPDT_EnaSafety_B) {
    /* Switch: '<S8>/Switch4' incorporates:
     *  Inport: '<Root>/Inport25'
     */
    LDPDT_LnCltdCurvLf_ReMi = LDPSAI_CurvSafeLf_ReMi;
  } else {
    /* Switch: '<S8>/Switch4' incorporates:
     *  Inport: '<Root>/Inport24'
     */
    LDPDT_LnCltdCurvLf_ReMi = LDPSAI_CurvLf_ReMi;
  }

  /* End of Switch: '<S8>/Switch4' */

  /* Switch: '<S15>/Switch' incorporates:
   *  Sum: '<S15>/Add'
   *  UnitDelay: '<S15>/Unit Delay'
   */
  if (LDPDT_UHysCltdCurvVldLf_B) {
    rtb_Abs = LDPDT_CrvThdHystLf_ReMi + LDPDT_CrvThdMaxLf_ReMi;
  } else {
    rtb_Abs = LDPDT_CrvThdMaxLf_ReMi;
  }

  /* End of Switch: '<S15>/Switch' */

  /* RelationalOperator: '<S15>/GreaterThan' incorporates:
   *  Abs: '<S12>/Abs'
   *  UnitDelay: '<S15>/Unit Delay'
   */
  LDPDT_UHysCltdCurvVldLf_B = (rtb_Abs > fabsf(LDPDT_LnCltdCurvLf_ReMi));

  /* Logic: '<S12>/AND1' incorporates:
   *  Logic: '<S12>/NOT'
   *  UnitDelay: '<S15>/Unit Delay'
   */
  rtb_LDPDT_LnCurvVldLf_B = ((!LDPDT_EnaByCstruSiteLf_B) &&
    LDPDT_UHysCltdCurvVldLf_B);

  /* Switch: '<S8>/Switch' */
  if (LDPDT_EnaSafety_B) {
    /* Switch: '<S8>/Switch' incorporates:
     *  Inport: '<Root>/Inport21'
     */
    LDPDT_LnHeadLf_Rad = LDPSAI_HeadAglSafeLf_Rad;
  } else {
    /* Switch: '<S8>/Switch' incorporates:
     *  Inport: '<Root>/Inport20'
     */
    LDPDT_LnHeadLf_Rad = LDPSAI_HeadAglLf_Rad;
  }

  /* End of Switch: '<S8>/Switch' */

  /* Switch: '<S14>/Switch' incorporates:
   *  Constant: '<S12>/V_Parameter10'
   *  Constant: '<S12>/V_Parameter11'
   *  Constant: '<S12>/V_Parameter12'
   *  Sum: '<S14>/Add'
   *  Sum: '<S14>/Add1'
   *  Switch: '<S14>/Switch1'
   *  UnitDelay: '<S14>/Unit Delay'
   */
  if (LDPDT_BHysHeadAglTrigVldLf_B) {
    rtb_Abs = LDPDT_TrsdHeadAglMx_C_Rad + LDPDT_TrsdHeadAglOfst_C_Rad;
    rtb_Divide_l2e1 = LDPDT_TrsdHeadAglMn_C_Rad - LDPDT_TrsdHeadAglOfst_C_Rad;
  } else {
    rtb_Abs = LDPDT_TrsdHeadAglMx_C_Rad;
    rtb_Divide_l2e1 = LDPDT_TrsdHeadAglMn_C_Rad;
  }

  /* End of Switch: '<S14>/Switch' */

  /* Logic: '<S14>/AND' incorporates:
   *  RelationalOperator: '<S14>/GreaterThan'
   *  RelationalOperator: '<S14>/GreaterThan1'
   *  UnaryMinus: '<S12>/Unary Minus'
   *  UnitDelay: '<S14>/Unit Delay'
   */
  LDPDT_BHysHeadAglTrigVldLf_B = ((rtb_Abs > (-LDPDT_LnHeadLf_Rad)) &&
    ((-LDPDT_LnHeadLf_Rad) > rtb_Divide_l2e1));

  /* RelationalOperator: '<S12>/Relational Operator4' incorporates:
   *  Constant: '<S12>/Constant2'
   *  Constant: '<S12>/V_Parameter4'
   *  Inport: '<Root>/Inport35'
   *  S-Function (sfix_bitop): '<S12>/Bitwise Operator1'
   */
  LDPDT_EnaByInVldQlfrLf_B = ((((int32_T)LDPSAI_LnIVldLf_St) & ((int32_T)
    LDPDT_LnIvldLf_C_St)) == 0);

  /* RelationalOperator: '<S12>/Relational Operator1' incorporates:
   *  Constant: '<S12>/Constant1'
   *  Constant: '<S12>/V_Parameter3'
   *  Inport: '<Root>/Inport34'
   *  S-Function (sfix_bitop): '<S12>/Bitwise Operator'
   */
  LDPDT_EnaByInVldQlfrSfLf_B = ((((int32_T)LDPSAI_IvldLnSafeLf_St) & ((int32_T)
    LDPDT_LnIvldSfLf_C_St)) == 0);

  /* Switch: '<S17>/Switch' incorporates:
   *  Constant: '<S12>/Constant'
   *  UnitDelay: '<S17>/Unit Delay'
   */
  if (LDPDT_LnLengthLf_B) {
    LDPVSE_NVRAMVehStartupSpd__osjy = 11;
  } else {
    LDPVSE_NVRAMVehStartupSpd__osjy = 10;
  }

  /* End of Switch: '<S17>/Switch' */

  /* RelationalOperator: '<S17>/GreaterThan' incorporates:
   *  Inport: '<Root>/Inport32'
   *  UnitDelay: '<S17>/Unit Delay'
   */
  LDPDT_LnLengthLf_B = (((real32_T)LDPVSE_NVRAMVehStartupSpd__osjy) >
                        LDPSAI_VldLengLf_Mi);

  /* Logic: '<S12>/NOT1' incorporates:
   *  UnitDelay: '<S17>/Unit Delay'
   */
  rtb_LogicalOperator_kelu = !LDPDT_LnLengthLf_B;

  /* Switch: '<S12>/Switch1' incorporates:
   *  Constant: '<S12>/V_Parameter1'
   *  Logic: '<S12>/Logical Operator'
   */
  if (LDPDT_SfFcLDPOn_C_B) {
    rtb_RelationalOperator1_mqkp = (LDPDT_EnaByInVldQlfrLf_B &&
      LDPDT_EnaByInVldQlfrSfLf_B);
  } else {
    rtb_RelationalOperator1_mqkp = LDPDT_EnaByInVldQlfrLf_B;
  }

  /* End of Switch: '<S12>/Switch1' */

  /* Logic: '<S12>/Logical Operator2' incorporates:
   *  UnitDelay: '<S14>/Unit Delay'
   */
  LDPDT_LnTrigVldLf_B = (((rtb_LDPDT_LnCurvVldLf_B &&
    LDPDT_BHysHeadAglTrigVldLf_B) && rtb_RelationalOperator1_mqkp) &&
    rtb_LogicalOperator_kelu);

  /* Switch: '<S16>/Switch' incorporates:
   *  Constant: '<S12>/V_Parameter2'
   *  Constant: '<S12>/V_Parameter8'
   *  Sum: '<S16>/Add'
   *  UnitDelay: '<S16>/Unit Delay'
   */
  if (LDPDT_UHysHeadAglCclVldLf_B) {
    rtb_Abs = LDPDT_TrsdHeadAglOfst_C_Rad + LDPDT_TrsdHeadAglMx_C_Rad;
  } else {
    rtb_Abs = LDPDT_TrsdHeadAglMx_C_Rad;
  }

  /* End of Switch: '<S16>/Switch' */

  /* RelationalOperator: '<S16>/GreaterThan' incorporates:
   *  Abs: '<S12>/Abs2'
   *  UnitDelay: '<S16>/Unit Delay'
   */
  LDPDT_UHysHeadAglCclVldLf_B = (rtb_Abs > fabsf(LDPDT_LnHeadLf_Rad));

  /* RelationalOperator: '<S12>/Relational Operator5' incorporates:
   *  Constant: '<S12>/Constant3'
   *  Constant: '<S12>/V_Parameter13'
   *  Inport: '<Root>/Inport35'
   *  S-Function (sfix_bitop): '<S12>/Bitwise Operator2'
   */
  LDPDT_CclByInVldQlfrLf_B = ((((int32_T)LDPSAI_LnIVldLf_St) & ((int32_T)
    LDPDT_LnIvldCclLf_C_St)) == 0);

  /* Logic: '<S12>/Logical Operator5' incorporates:
   *  UnitDelay: '<S16>/Unit Delay'
   */
  LDPDT_LnCclVldLf_B = (((rtb_LDPDT_LnCurvVldLf_B && LDPDT_UHysHeadAglCclVldLf_B)
    && LDPDT_CclByInVldQlfrLf_B) && rtb_LogicalOperator_kelu);

  /* Switch: '<S12>/Switch' */
  if (LDPDT_RdyTrigLDP_B) {
    /* Switch: '<S12>/Switch' */
    LDPDT_LnMakVldLf_B = LDPDT_LnTrigVldLf_B;
  } else {
    /* Switch: '<S12>/Switch' */
    LDPDT_LnMakVldLf_B = LDPDT_LnCclVldLf_B;
  }

  /* End of Switch: '<S12>/Switch' */

  /* Switch: '<S8>/Switch2' */
  if (LDPDT_EnaSafety_B) {
    /* Switch: '<S8>/Switch2' incorporates:
     *  Inport: '<Root>/Inport17'
     */
    LDPDT_LnPstnLf_Mi = LDPSAI_PstnYSafeLf_Mi;
  } else {
    /* Switch: '<S8>/Switch2' incorporates:
     *  Inport: '<Root>/Inport16'
     */
    LDPDT_LnPstnLf_Mi = LDPSAI_PstnYLf_Mi;
  }

  /* End of Switch: '<S8>/Switch2' */

  /* Sum: '<S10>/Subtract' */
  LDPDT_RawDstcToLnLf_Mi = LDPDT_LnPstnLf_Mi - rtb_Subtract2_pfel;

  /* Lookup_n-D: '<S78>/1-D Lookup Table5' incorporates:
   *  Inport: '<Root>/Inport25'
   */
  LDPSC_DstcToLnTrsdCrvCpstnLf_Mi = look1_iflf_binlxpw(LDPSAI_CurvSafeLf_ReMi, ((
    const real32_T *)&(LDPSC_LnDectCrvLf_BX_ReMi[0])), ((const real32_T *)
    &(LDPSC_DstcToLnTrsdOfstLf_Cr_Mi[0])), 16U);

  /* Product: '<S10>/Product' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPDT_RawLatVehSpdLf_Mps = LDPDT_LnHeadLf_Rad * LDPSAI_VehSpdActu_Mps;

  /* Switch: '<S10>/Switch' incorporates:
   *  Switch: '<S10>/Switch1'
   */
  if (LDPDT_LnMakVldLf_B) {
    /* Switch: '<S10>/Switch' */
    LDPDT_DstcToLnLf_Mi = LDPDT_RawDstcToLnLf_Mi;

    /* Switch: '<S10>/Switch1' */
    LDPDT_LatVehSpdLf_Mps = LDPDT_RawLatVehSpdLf_Mps;
  } else {
    /* Switch: '<S10>/Switch' incorporates:
     *  Constant: '<S10>/Constant1'
     */
    LDPDT_DstcToLnLf_Mi = 10.0F;

    /* Switch: '<S10>/Switch1' incorporates:
     *  Constant: '<S10>/Constant2'
     */
    LDPDT_LatVehSpdLf_Mps = 10.0F;
  }

  /* End of Switch: '<S10>/Switch' */

  /* Switch: '<S11>/Switch' incorporates:
   *  Constant: '<S11>/V_Parameter1'
   *  Inport: '<Root>/Inport24'
   *  RelationalOperator: '<S11>/GreaterThan2'
   *  RelationalOperator: '<S11>/Less Than'
   *  Switch: '<S11>/Switch1'
   *  UnaryMinus: '<S11>/Unary Minus'
   */
  if (LDPSAI_CurvLf_ReMi > LDPDT_CurveThd_C_St) {
    /* Switch: '<S11>/Switch' incorporates:
     *  Constant: '<S11>/V_Parameter2'
     */
    LDPDT_CurveTypeLe_St = LDPDT_CurveInner_C_St;
  } else if (LDPSAI_CurvLf_ReMi < (-LDPDT_CurveThd_C_St)) {
    /* Switch: '<S11>/Switch1' incorporates:
     *  Constant: '<S11>/V_Parameter4'
     *  Switch: '<S11>/Switch'
     */
    LDPDT_CurveTypeLe_St = LDPDT_CurveOuter_C_St;
  } else {
    /* Switch: '<S11>/Switch' incorporates:
     *  Constant: '<S11>/V_Parameter5'
     *  Switch: '<S11>/Switch1'
     */
    LDPDT_CurveTypeLe_St = LDPDT_CurveNone_C_St;
  }

  /* End of Switch: '<S11>/Switch' */

  /* Switch: '<S78>/Switch5' incorporates:
   *  Constant: '<S78>/V_Parameter1'
   *  Constant: '<S78>/V_Parameter2'
   *  RelationalOperator: '<S78>/Relational Operator3'
   *  RelationalOperator: '<S78>/Relational Operator6'
   *  Switch: '<S78>/Switch2'
   */
  if (LDPDT_CurveTypeLe_St == LDPDT_CurveInner_C_St) {
    /* Abs: '<S78>/Abs' incorporates:
     *  Switch: '<S78>/Switch5'
     */
    LDPSC_CrvSensiDecayLe_Mi = rtb_uDLookupTable8_idx_0;
  } else {
    if (LDPDT_CurveTypeLe_St != LDPDT_CurveOuter_C_St) {
      /* Abs: '<S78>/Abs' incorporates:
       *  Constant: '<S78>/V_Const2'
       *  Switch: '<S78>/Switch2'
       *  Switch: '<S78>/Switch5'
       */
      LDPSC_CrvSensiDecayLe_Mi = 0.0F;
    }
  }

  /* End of Switch: '<S78>/Switch5' */

  /* Sum: '<S78>/Add2' incorporates:
   *  Lookup_n-D: '<S78>/1-D Lookup Table10'
   *  Product: '<S78>/Product2'
   *  Sum: '<S78>/Add'
   *  Switch: '<S10>/Switch1'
   */
  LDPSC_DstcToLnTrsdLf_Mi = (LDPSC_DstcToLnTrsdCrvCpstnLf_Mi +
    (look1_iflf_binlxpw(LDPDT_LatVehSpdLf_Mps, (&(LDPSC_LnLatVeh_BX_Mps[0])),
                       (&(LDPSC_LnLatVeh_Lf_Mps[0])), 8U) *
     rtb_uDLookupTable9_idx_1)) - LDPSC_CrvSensiDecayLe_Mi;

  /* RelationalOperator: '<S79>/Relational Operator3' */
  LDPSC_RawTrigByDlcLf_B = (LDPDT_DstcToLnLf_Mi <= LDPSC_DstcToLnTrsdLf_Mi);

  /* RelationalOperator: '<S79>/Relational Operator1' incorporates:
   *  Constant: '<S79>/V_Parameter10'
   */
  LDPSC_EnaTlcTrigLf_B = (LDPDT_DstcToLnLf_Mi < LDPSC_DstcOfTiToLnMn_C_Mi);

  /* Switch: '<S10>/Switch3' incorporates:
   *  Constant: '<S10>/Constant4'
   *  Constant: '<S10>/V_Parameter1'
   *  Inport: '<Root>/Inport1'
   *  Logic: '<S10>/Logical Operator'
   *  RelationalOperator: '<S10>/Relational Operator1'
   *  RelationalOperator: '<S10>/Relational Operator5'
   *  UnaryMinus: '<S10>/Unary Minus'
   */
  if (((LDPDT_LnHeadLf_Rad < (-LDPDT_TLCHeadAglTrsd_C_Rad)) &&
       (LDPSAI_VehSpdActu_Mps > 0.0F)) && LDPDT_LnMakVldLf_B) {
    /* Switch: '<S10>/Switch3' incorporates:
     *  Abs: '<S10>/Abs'
     *  Constant: '<S10>/Constant3'
     *  Constant: '<S10>/Constant5'
     *  MinMax: '<S10>/MinMax1'
     *  MinMax: '<S10>/MinMax2'
     *  Product: '<S10>/Divide1'
     */
    LDPDT_TiToLnLf_Sec = fminf(LDPDT_DstcToLnLf_Mi / fmaxf(fabsf
      (LDPDT_LatVehSpdLf_Mps), 0.01F), 10.0F);
  } else {
    /* Switch: '<S10>/Switch3' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    LDPDT_TiToLnLf_Sec = 10.0F;
  }

  /* End of Switch: '<S10>/Switch3' */

  /* RelationalOperator: '<S79>/Relational Operator2' */
  LDPSC_RawTrigByTlcLf_B = (LDPDT_TiToLnLf_Sec < LDPSC_TiToLnTrsd_Sec);

  /* Switch: '<S94>/Switch' incorporates:
   *  Constant: '<S79>/V_Parameter12'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S94>/Max'
   *  Sum: '<S94>/Subtract'
   *  Switch: '<S94>/Switch1'
   *  UnaryMinus: '<S94>/Unary Minus'
   *  UnitDelay: '<S94>/Unit Delay'
   */
  if (LDPSC_RawTrigByTlcLf_B) {
    LDPSC_DlyTiOfTiToLnLfMn_Sec = fmaxf(LDPSC_DlyTiOfTiToLnLfMn_Sec,
      -LDPSAI_CycleTime_Sec) - LDPSAI_CycleTime_Sec;
  } else {
    LDPSC_DlyTiOfTiToLnLfMn_Sec = LDPSC_DlyTiOfTiToLnMn_C_Sec;
  }

  /* End of Switch: '<S94>/Switch' */

  /* Logic: '<S94>/AND' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S94>/LessThanOrEqual'
   *  UnaryMinus: '<S94>/Unary Minus1'
   *  UnitDelay: '<S94>/Unit Delay'
   */
  LDPSC_DlyTrigByTlcLf_B = (LDPSC_RawTrigByTlcLf_B &&
    (LDPSC_DlyTiOfTiToLnLfMn_Sec <= (-LDPSAI_CycleTime_Sec)));

  /* Logic: '<S79>/Logical Operator2' incorporates:
   *  Constant: '<S79>/V_Parameter11'
   *  Constant: '<S79>/V_Parameter9'
   *  DataTypeConversion: '<S84>/Data Type Conversion'
   *  DataTypeConversion: '<S85>/Data Type Conversion'
   *  Logic: '<S79>/Logical Operator'
   *  Logic: '<S79>/Logical Operator1'
   *  S-Function (sfix_bitop): '<S84>/Bitwise AND'
   *  S-Function (sfix_bitop): '<S85>/Bitwise AND'
   */
  rtb_RelationalOperator4_jya3 = ((((((uint32_T)LDPSC_TrigCdtnEn_C_St) & 1U) !=
    0U) && LDPSC_RawTrigByDlcLf_B) || ((((((uint32_T)LDPSC_TrigCdtnEn_C_St) & 2U)
    != 0U) && LDPSC_EnaTlcTrigLf_B) && LDPSC_DlyTrigByTlcLf_B));

  /* Logic: '<S83>/AND' incorporates:
   *  Logic: '<S83>/NOT'
   *  UnitDelay: '<S83>/Unit Delay'
   */
  LDPSC_EnaLdwTrigLf_B = (rtb_RelationalOperator4_jya3 && (!LDPSC_HdTiTrigLfEn_B));

  /* Logic: '<S79>/Logical Operator15' incorporates:
   *  Constant: '<S79>/V_Parameter1'
   *  Constant: '<S79>/V_Parameter14'
   *  RelationalOperator: '<S79>/Less Than'
   *  RelationalOperator: '<S79>/Relational Operator4'
   *  Sum: '<S79>/Add2'
   */
  LDPSC_RstLdwTrigLf_B = (((LDPSC_DstcToLnTrsdLf_Mi + 0.12F) <
    LDPDT_DstcToLnLf_Mi) || (LDPDT_DstcToLnLf_Mi <
    LDPSC_DstcOfDiscToLnLmtMn_C_Mi));

  /* Switch: '<S92>/Switch4' incorporates:
   *  Switch: '<S92>/Switch3'
   */
  if (LDPSC_RstLdwTrigLf_B) {
    /* Sum: '<S92>/Subtract2' incorporates:
     *  Constant: '<S92>/Constant3'
     */
    LDPSC_HdTiTrigLf_Sec = 0.0F;
  } else {
    if (LDPSC_EnaLdwTrigLf_B) {
      /* Sum: '<S92>/Subtract2' incorporates:
       *  Constant: '<S79>/V_Parameter15'
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S92>/Subtract1'
       *  Switch: '<S92>/Switch3'
       */
      LDPSC_HdTiTrigLf_Sec = LDPSC_HdTiTrigLf_C_Sec + LDPSAI_CycleTime_Sec;
    }
  }

  /* End of Switch: '<S92>/Switch4' */

  /* SignalConversion: '<S79>/Signal Conversion' incorporates:
   *  Constant: '<S92>/Constant4'
   *  Logic: '<S92>/OR'
   *  RelationalOperator: '<S92>/GreaterThan2'
   */
  LDPSC_HoldLdwTrigLf_B = (LDPSC_EnaLdwTrigLf_B || (LDPSC_HdTiTrigLf_Sec >
    1.0E-5F));

  /* Logic: '<S79>/Logical Operator5' incorporates:
   *  Logic: '<S39>/Logical Operator19'
   */
  LDPSC_ErrSideByTrigLf_B = !LDPDT_LnMakVldLf_B;

  /* Logic: '<S79>/Logical Operator4' incorporates:
   *  Constant: '<S88>/Constant'
   *  Inport: '<Root>/Inport12'
   *  Logic: '<S79>/Logical Operator5'
   *  RelationalOperator: '<S79>/Relational Operator5'
   */
  LDPSC_ResetForSafeLf_B = ((LDPSAI_DtctLnChag_B || LDPSC_ErrSideByTrigLf_B) ||
    (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_ACTIVE));

  /* RelationalOperator: '<S79>/Relational Operator6' incorporates:
   *  Constant: '<S79>/V_Parameter13'
   *  Sum: '<S79>/Add'
   */
  LDPSC_SetForSafeLf_B = (LDPDT_DstcToLnLf_Mi > (LDPSC_DstcToLnTrsdLf_Mi +
    LDPSC_DstcOfstSafeSitu_C_Mi));

  /* Switch: '<S90>/Switch' incorporates:
   *  Constant: '<S90>/Constant2'
   *  Switch: '<S90>/Switch1'
   *  UnitDelay: '<S90>/Unit Delay'
   */
  if (LDPSC_ResetForSafeLf_B) {
    LDPSC_DisTrigLf_B = false;
  } else {
    LDPSC_DisTrigLf_B = (LDPSC_SetForSafeLf_B || LDPSC_DisTrigLf_B);
  }

  /* End of Switch: '<S90>/Switch' */

  /* Logic: '<S79>/Logical Operator6' incorporates:
   *  UnitDelay: '<S90>/Unit Delay'
   */
  rtb_LogicalOperator_kelu = (LDPSC_HoldLdwTrigLf_B && LDPSC_DisTrigLf_B);

  /* Logic: '<S79>/Logical Operator8' incorporates:
   *  Constant: '<S79>/V_Parameter5'
   *  RelationalOperator: '<S79>/Relational Operator8'
   *  RelationalOperator: '<S79>/Relational Operator9'
   */
  rtb_LDPSC_VehicleInvalid_B = ((LDPDT_DstcToLnLf_Mi < LDPSC_DstcToLnTrsdLf_Mi) &&
    (LDPDT_DstcToLnLf_Mi > LDPSC_DstcOfDiscToLnLmtMn_C_Mi));

  /* RelationalOperator: '<S79>/Relational Operator10' incorporates:
   *  Constant: '<S89>/Constant'
   */
  rtb_RelationalOperator10_gbg2 = (((uint32_T)LDPSC_SysOld_St) ==
    E_LDPState_nu_ACTIVE);

  /* Logic: '<S79>/Logical Operator11' incorporates:
   *  Logic: '<S79>/AND'
   */
  rtb_RelationalOperator1_mqkp = !rtb_LDPSC_VehicleInvalid_B;

  /* Switch: '<S79>/Switch1' incorporates:
   *  Constant: '<S79>/Constant3'
   *  Logic: '<S79>/Logical Operator10'
   *  Logic: '<S79>/Logical Operator11'
   *  Logic: '<S86>/AND'
   *  Logic: '<S86>/NOT'
   *  Sum: '<S79>/Add1'
   *  UnitDelay: '<S79>/Unit Delay'
   *  UnitDelay: '<S79>/Unit Delay2'
   *  UnitDelay: '<S86>/Unit Delay'
   */
  if (LDPSC_SuppFlagOldLf_B || rtb_RelationalOperator1_mqkp) {
    LDPSC_ContinWarmTimesOldLf_Count = 0.0F;
  } else {
    LDPSC_ContinWarmTimesOldLf_Count += (real32_T)
      ((rtb_RelationalOperator10_gbg2 && (!LDPSC_PreActiveEdgeLf)) ? 1 : 0);
  }

  /* End of Switch: '<S79>/Switch1' */

  /* RelationalOperator: '<S79>/Relational Operator12' incorporates:
   *  Constant: '<S79>/V_Parameter4'
   *  UnitDelay: '<S79>/Unit Delay'
   *  UnitDelay: '<S79>/Unit Delay2'
   */
  LDPSC_SuppFlagOldLf_B = (LDPSC_ContinWarmTimesOldLf_Count >=
    LDPSC_ContinWarmTimes_C_Count);

  /* Switch: '<S93>/Switch4' incorporates:
   *  Logic: '<S79>/Logical Operator7'
   *  Switch: '<S93>/Switch3'
   *  UnitDelay: '<S79>/Unit Delay'
   */
  if (!rtb_LDPSC_VehicleInvalid_B) {
    /* Sum: '<S93>/Subtract2' incorporates:
     *  Constant: '<S93>/Constant3'
     */
    LDPSC_SuppTimeOldLf_Sec = 0.0F;
  } else {
    if (LDPSC_SuppFlagOldLf_B) {
      /* Sum: '<S93>/Subtract2' incorporates:
       *  Constant: '<S79>/V_Parameter2'
       *  Constant: '<S79>/V_Parameter3'
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S79>/Add3'
       *  Sum: '<S93>/Subtract1'
       *  Switch: '<S93>/Switch3'
       */
      LDPSC_SuppTimeOldLf_Sec = (LDPSC_ContinWarmSupp_C_Sec +
        LDPSC_WarmMxTi_C_Sec) + LDPSAI_CycleTime_Sec;
    }
  }

  /* End of Switch: '<S93>/Switch4' */

  /* Logic: '<S79>/Logical Operator14' incorporates:
   *  Constant: '<S93>/Constant4'
   *  Logic: '<S93>/OR'
   *  RelationalOperator: '<S93>/GreaterThan2'
   *  UnitDelay: '<S79>/Unit Delay'
   */
  rtb_LogicalOperator14_fahu = ((!LDPSC_SuppFlagOldLf_B) &&
    (LDPSC_SuppTimeOldLf_Sec <= 1.0E-5F));

  /* Logic: '<S79>/Logical Operator9' incorporates:
   *  Logic: '<S79>/AND'
   */
  LDPSC_ResetForContinTrigLf_B = (rtb_RelationalOperator1_mqkp ||
    (!rtb_LogicalOperator14_fahu));

  /* Logic: '<S79>/Logical Operator13' incorporates:
   *  Logic: '<S87>/AND'
   *  Logic: '<S87>/NOT'
   *  UnitDelay: '<S87>/Unit Delay'
   */
  LDPSC_SetForContinTrigLf_B = (rtb_LogicalOperator_kelu ||
    (rtb_LogicalOperator14_fahu && (!LDPSC_ContinTrigLfEn_B)));

  /* Switch: '<S91>/Switch' incorporates:
   *  Constant: '<S91>/Constant2'
   *  Switch: '<S91>/Switch1'
   *  UnitDelay: '<S91>/Unit Delay'
   */
  if (LDPSC_ResetForContinTrigLf_B) {
    LDPSC_DisContinTrigLf_B = false;
  } else {
    LDPSC_DisContinTrigLf_B = (LDPSC_SetForContinTrigLf_B ||
      LDPSC_DisContinTrigLf_B);
  }

  /* End of Switch: '<S91>/Switch' */

  /* RelationalOperator: '<S161>/Relational Operator2' incorporates:
   *  Constant: '<S161>/V_Parameter1'
   *  Inport: '<Root>/Inport6'
   */
  rtb_LDPSC_VehicleInvalid_B = (LDPSAI_TrnSgl_St == LDPVSE_TrnSglLf_C_St);

  /* RelationalOperator: '<S161>/Relational Operator1' incorporates:
   *  Constant: '<S161>/V_Parameter'
   *  Inport: '<Root>/Inport6'
   */
  rtb_RelationalOperator1_mqkp = (LDPSAI_TrnSgl_St == LDPVSE_TrnSglRi_C_St);

  /* Logic: '<S173>/AND' incorporates:
   *  Logic: '<S173>/NOT'
   *  UnitDelay: '<S173>/Unit Delay'
   */
  LDPVSE_EdgeRiseTurnSglLf_B = (rtb_RelationalOperator1_mqkp &&
    (!LDPVSE_EdgeRisTrnSglRi_B));

  /* Switch: '<S175>/Switch4' incorporates:
   *  Constant: '<S161>/V_Parameter2'
   *  Constant: '<S175>/Constant3'
   *  Switch: '<S161>/Switch'
   *  Switch: '<S175>/Switch3'
   */
  if ((LDPVSE_TrnSglRstLfEn_C_B) && LDPVSE_EdgeRiseTurnSglLf_B) {
    rtb_Subtract2_pfel = 0.0F;
  } else if (rtb_LDPSC_VehicleInvalid_B) {
    /* Switch: '<S175>/Switch3' incorporates:
     *  Constant: '<S161>/V_Parameter3'
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S175>/Subtract1'
     */
    rtb_Subtract2_pfel = LDPVSE_HodTiTrnSgl_C_Sec + LDPSAI_CycleTime_Sec;
  } else {
    /* Switch: '<S175>/Switch3' incorporates:
     *  UnitDelay: '<S175>/Unit Delay'
     */
    rtb_Subtract2_pfel = LDPVSE_HodTiTrnSglLf_Sec;
  }

  /* End of Switch: '<S175>/Switch4' */

  /* SignalConversion: '<S161>/Signal Conversion' incorporates:
   *  Constant: '<S175>/Constant4'
   *  Logic: '<S175>/OR'
   *  RelationalOperator: '<S175>/GreaterThan2'
   */
  LDPVSE_TrnSglLf_B = (rtb_LDPSC_VehicleInvalid_B || (rtb_Subtract2_pfel >
    1.0E-5F));

  /* SignalConversion: '<S162>/Signal Conversion2' */
  rtb_VectorConcatenate_otre[0] = LDPVSE_TrnSglLf_B;

  /* Switch: '<S181>/Switch' incorporates:
   *  Constant: '<S177>/V_Parameter4'
   *  Constant: '<S177>/V_Parameter5'
   *  Sum: '<S181>/Add'
   *  Sum: '<S181>/Add1'
   *  Switch: '<S181>/Switch1'
   *  UnitDelay: '<S181>/Unit Delay'
   */
  if (LDPVSE_BHysLatVehSpdVldLf_B) {
    rtb_Abs = LDPVSE_MaxLatVel_Mps + LDPVSE_VehLatTrsdLDPOfst_C_Msp;
    rtb_Divide_l2e1 = LDPVSE_VehLatTrsdLDPMn_C_Msp -
      LDPVSE_VehLatTrsdLDPOfst_C_Msp;
  } else {
    rtb_Abs = LDPVSE_MaxLatVel_Mps;
    rtb_Divide_l2e1 = LDPVSE_VehLatTrsdLDPMn_C_Msp;
  }

  /* End of Switch: '<S181>/Switch' */

  /* Logic: '<S181>/AND' incorporates:
   *  RelationalOperator: '<S181>/GreaterThan'
   *  RelationalOperator: '<S181>/GreaterThan1'
   *  UnaryMinus: '<S177>/Unary Minus'
   *  UnitDelay: '<S181>/Unit Delay'
   */
  LDPVSE_BHysLatVehSpdVldLf_B = ((rtb_Abs > (-LDPDT_LatVehSpdLf_Mps)) &&
    ((-LDPDT_LatVehSpdLf_Mps) > rtb_Divide_l2e1));

  /* Switch: '<S182>/Switch' incorporates:
   *  Constant: '<S177>/V_Parameter1'
   *  Constant: '<S177>/V_Parameter2'
   *  Sum: '<S182>/Add'
   *  UnitDelay: '<S182>/Unit Delay'
   */
  if (LDPVSE_UHysLatVehSpdVldLf_B) {
    rtb_Abs = LDPVSE_VehLatTrsdLDPOfst_C_Msp + LDPVSE_VehLatTrsdLDPMx_C_Msp;
  } else {
    rtb_Abs = LDPVSE_VehLatTrsdLDPMx_C_Msp;
  }

  /* End of Switch: '<S182>/Switch' */

  /* RelationalOperator: '<S182>/GreaterThan' incorporates:
   *  Abs: '<S177>/Abs'
   *  UnitDelay: '<S182>/Unit Delay'
   */
  LDPVSE_UHysLatVehSpdVldLf_B = (rtb_Abs > fabsf(LDPDT_LatVehSpdLf_Mps));

  /* Switch: '<S177>/Switch' */
  if (LDPVSE_RdyTrigLDW_B) {
    /* Switch: '<S177>/Switch' incorporates:
     *  UnitDelay: '<S181>/Unit Delay'
     */
    LDPVSE_VehLatSpdVldLf_B = LDPVSE_BHysLatVehSpdVldLf_B;
  } else {
    /* Switch: '<S177>/Switch' incorporates:
     *  UnitDelay: '<S182>/Unit Delay'
     */
    LDPVSE_VehLatSpdVldLf_B = LDPVSE_UHysLatVehSpdVldLf_B;
  }

  /* End of Switch: '<S177>/Switch' */

  /* Logic: '<S162>/Logical Operator2' */
  rtb_VectorConcatenate_otre[1] = !LDPVSE_VehLatSpdVldLf_B;

  /* S-Function (ex_sfun_set_bit): '<S184>/ex_sfun_set_bit' incorporates:
   *  Constant: '<S179>/Constant'
   */
  set_bit(0U, (boolean_T*)&rtb_VectorConcatenate_otre[0], (uint8_T*)
          (&(LDPSA_SetBit_BS_Param_3[0])), ((uint8_T)2U), &rtb_ex_sfun_set_bit);

  /* SignalConversion: '<S162>/Signal Conversion' incorporates:
   *  DataTypeConversion: '<S184>/Data Type Conversion1'
   */
  LDPVSE_SidCdtnLDPLf_St = (uint8_T)rtb_ex_sfun_set_bit;

  /* RelationalOperator: '<S79>/Relational Operator7' incorporates:
   *  Constant: '<S79>/V_Const'
   *  Constant: '<S79>/V_Const2'
   *  S-Function (sfix_bitop): '<S79>/Bitwise AND1'
   */
  LDPSC_TrigBySideCondLf_B = ((((uint32_T)LDPVSE_SidCdtnLDPLf_St) & 3U) == 0U);

  /* RelationalOperator: '<S79>/Equal' incorporates:
   *  Constant: '<S79>/V_Const1'
   */
  LDPSC_TrigByPrjSpecLf_B = (LDPSC_TrigByPrjSpecRi_B_tmp == 0);

  /* Logic: '<S79>/Logical Operator3' incorporates:
   *  Logic: '<S79>/Logical Operator12'
   *  UnitDelay: '<S91>/Unit Delay'
   */
  LDPSC_TrigLf_B = (((rtb_LogicalOperator_kelu || LDPSC_DisContinTrigLf_B) &&
                     LDPSC_TrigBySideCondLf_B) && LDPSC_TrigByPrjSpecLf_B);

  /* Logic: '<S77>/Logical Operator6' incorporates:
   *  Constant: '<S81>/Constant'
   *  Constant: '<S82>/Constant'
   *  RelationalOperator: '<S77>/Relational Operator8'
   *  RelationalOperator: '<S77>/Relational Operator9'
   */
  LDPSC_EnaDgrSide_B = ((((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_ACTIVE) ||
                        (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_RAMPOUT));

  /* Switch: '<S77>/Switch1' incorporates:
   *  Constant: '<S77>/V_Parameter4'
   *  Switch: '<S77>/Switch'
   *  Switch: '<S77>/Switch2'
   *  Switch: '<S77>/Switch3'
   *  UnitDelay: '<S2>/Unit Delay'
   *  UnitDelay: '<S77>/UnitDelay1'
   */
  if (LDPSC_EnaDgrSide_B) {
    LDPSC_LstPrevDgrSide_St = LDPSC_DgrSideOld_St;
  } else {
    if (LDPSC_TrigRi_B) {
      /* Switch: '<S77>/Switch2' incorporates:
       *  Constant: '<S77>/V_Parameter7'
       *  UnitDelay: '<S77>/UnitDelay1'
       */
      LDPSC_DgrSideOld_St = LDPSC_DgrSideRi_C_St;
    } else if (LDPSC_TrigLf_B) {
      /* Switch: '<S77>/Switch3' incorporates:
       *  Constant: '<S77>/V_Parameter6'
       *  Switch: '<S77>/Switch2'
       *  UnitDelay: '<S77>/UnitDelay1'
       */
      LDPSC_DgrSideOld_St = LDPSC_DgrSideLf_C_St;
    } else {
      /* UnitDelay: '<S77>/UnitDelay1' incorporates:
       *  Constant: '<S77>/V_Parameter5'
       *  Switch: '<S77>/Switch2'
       *  Switch: '<S77>/Switch3'
       */
      LDPSC_DgrSideOld_St = LDPSC_NoDgrSide_C_St;
    }

    LDPSC_LstPrevDgrSide_St = LDPSC_NoDgrSide_C_St;
  }

  /* End of Switch: '<S77>/Switch1' */

  /* RelationalOperator: '<S33>/Relational Operator7' incorporates:
   *  Constant: '<S33>/V_Parameter17'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDPSC_FnsByDgrStLf_B = (LDPSC_LstPrevDgrSide_St == LDPSC_DgrSideLf_C_St);

  /* Sum: '<S33>/Add2' incorporates:
   *  Constant: '<S33>/V_Parameter20'
   *  Constant: '<S33>/V_Parameter21'
   *  Sum: '<S33>/Add4'
   */
  rtb_Abs = LDPSC_TgtTrajPstnY_C_Mi - LDPSC_NoDgrFnsOfst_C_Mi;

  /* Sum: '<S33>/Add1' incorporates:
   *  Constant: '<S33>/V_Parameter18'
   *  Constant: '<S33>/V_Parameter19'
   *  Sum: '<S33>/Add3'
   */
  rtb_Divide_l2e1 = LDPSC_TgtTrajPstnY_C_Mi + LDPSC_DgrFnsOfst_C_Mi;

  /* Logic: '<S33>/Logical Operator8' incorporates:
   *  Constant: '<S33>/V_Parameter28'
   *  DataTypeConversion: '<S61>/Data Type Conversion'
   *  Logic: '<S33>/NOT'
   *  Logic: '<S70>/AND'
   *  RelationalOperator: '<S70>/Less Than'
   *  RelationalOperator: '<S70>/Less Than1'
   *  S-Function (sfix_bitop): '<S61>/Bitwise AND'
   *  Sum: '<S33>/Add1'
   *  Sum: '<S33>/Add2'
   */
  LDPSC_FnsByLatDistLf_B = (((rtb_Divide_l2e1 > LDPDT_DstcToLnLf_Mi) &&
    (LDPDT_DstcToLnLf_Mi > rtb_Abs)) || ((((uint32_T)LDPSC_FnsCdsnEn_C_St) & 1U)
    == 0U));

  /* Logic: '<S33>/Logical Operator9' incorporates:
   *  Constant: '<S33>/V_Parameter22'
   *  Constant: '<S33>/V_Parameter23'
   *  Constant: '<S33>/V_Parameter29'
   *  DataTypeConversion: '<S62>/Data Type Conversion'
   *  Inport: '<Root>/Inport20'
   *  Logic: '<S33>/NOT1'
   *  Logic: '<S71>/AND'
   *  RelationalOperator: '<S71>/Less Than'
   *  RelationalOperator: '<S71>/Less Than1'
   *  S-Function (sfix_bitop): '<S62>/Bitwise AND'
   *  UnaryMinus: '<S33>/Unary Minus'
   */
  LDPSC_FnsByHeadingLf_B = (((LDPSC_NoDgrFnsHeadAng_C_Rad > LDPSAI_HeadAglLf_Rad)
    && (LDPSAI_HeadAglLf_Rad > (-LDPSC_DgrFnsHeadAng_C_Rad))) || ((((uint32_T)
    LDPSC_FnsCdsnEn_C_St) & 2U) == 0U));

  /* Logic: '<S33>/Logical Operator10' incorporates:
   *  Constant: '<S33>/V_Parameter24'
   *  Constant: '<S33>/V_Parameter25'
   *  Constant: '<S33>/V_Parameter30'
   *  DataTypeConversion: '<S63>/Data Type Conversion'
   *  Logic: '<S33>/NOT2'
   *  Logic: '<S72>/AND'
   *  RelationalOperator: '<S72>/Less Than'
   *  RelationalOperator: '<S72>/Less Than1'
   *  S-Function (sfix_bitop): '<S63>/Bitwise AND'
   *  UnaryMinus: '<S33>/Unary Minus1'
   */
  LDPSC_FnsByLatSpdLf_B = (((LDPSC_NoDgrFnsSpdVelLat_C_Mps >
    LDPDT_LatVehSpdLf_Mps) && (LDPDT_LatVehSpdLf_Mps >
    (-LDPSC_DgrFnsSpdVelLat_C_Mps))) || ((((uint32_T)LDPSC_FnsCdsnEn_C_St) & 4U)
    == 0U));

  /* Logic: '<S33>/Logical Operator6' */
  LDPSC_DgrFnsLf_B = (((LDPSC_FnsByDgrStLf_B && LDPSC_FnsByLatDistLf_B) &&
                       LDPSC_FnsByHeadingLf_B) && LDPSC_FnsByLatSpdLf_B);

  /* RelationalOperator: '<S33>/Relational Operator9' incorporates:
   *  Constant: '<S33>/V_Parameter32'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDPSC_FnsByDgrStRi_B = (LDPSC_LstPrevDgrSide_St == LDPSC_DgrSideRi_C_St);

  /* Logic: '<S33>/Logical Operator14' incorporates:
   *  Constant: '<S33>/V_Parameter41'
   *  DataTypeConversion: '<S65>/Data Type Conversion'
   *  Logic: '<S33>/NOT3'
   *  Logic: '<S73>/AND'
   *  RelationalOperator: '<S73>/Less Than'
   *  RelationalOperator: '<S73>/Less Than1'
   *  S-Function (sfix_bitop): '<S65>/Bitwise AND'
   *  UnaryMinus: '<S33>/Unary Minus4'
   *  UnaryMinus: '<S33>/Unary Minus5'
   */
  LDPSC_FnsByLatDistRi_B = ((((-rtb_Abs) > LDPDT_DstcToLnRi_Mi) &&
    (LDPDT_DstcToLnRi_Mi > (-rtb_Divide_l2e1))) || ((((uint32_T)
    LDPSC_FnsCdsnEn_C_St) & 8U) == 0U));

  /* Logic: '<S33>/Logical Operator15' incorporates:
   *  Constant: '<S33>/V_Parameter37'
   *  Constant: '<S33>/V_Parameter38'
   *  Constant: '<S33>/V_Parameter42'
   *  DataTypeConversion: '<S66>/Data Type Conversion'
   *  Inport: '<Root>/Inport22'
   *  Logic: '<S33>/NOT4'
   *  Logic: '<S74>/AND'
   *  RelationalOperator: '<S74>/Less Than'
   *  RelationalOperator: '<S74>/Less Than1'
   *  S-Function (sfix_bitop): '<S66>/Bitwise AND'
   *  UnaryMinus: '<S33>/Unary Minus2'
   */
  LDPSC_FnsByHeadingRi_B = (((LDPSC_DgrFnsHeadAng_C_Rad > LDPSAI_HeadAglRi_Rad) &&
    (LDPSAI_HeadAglRi_Rad > (-LDPSC_NoDgrFnsHeadAng_C_Rad))) || ((((uint32_T)
    LDPSC_FnsCdsnEn_C_St) & 16U) == 0U));

  /* Logic: '<S33>/Logical Operator13' incorporates:
   *  Constant: '<S33>/V_Parameter39'
   *  Constant: '<S33>/V_Parameter40'
   *  Constant: '<S33>/V_Parameter43'
   *  DataTypeConversion: '<S67>/Data Type Conversion'
   *  Logic: '<S33>/NOT5'
   *  Logic: '<S75>/AND'
   *  RelationalOperator: '<S75>/Less Than'
   *  RelationalOperator: '<S75>/Less Than1'
   *  S-Function (sfix_bitop): '<S67>/Bitwise AND'
   *  UnaryMinus: '<S33>/Unary Minus3'
   */
  LDPSC_FnsByLatSpdRi_B = (((LDPSC_DgrFnsSpdVelLat_C_Mps > LDPDT_LatVehSpdRi_Mps)
    && (LDPDT_LatVehSpdRi_Mps > (-LDPSC_NoDgrFnsSpdVelLat_C_Mps))) ||
    ((((uint32_T)LDPSC_FnsCdsnEn_C_St) & 32U) == 0U));

  /* Logic: '<S33>/Logical Operator12' */
  LDPSC_DgrFnsRi_B = (((LDPSC_FnsByDgrStRi_B && LDPSC_FnsByLatDistRi_B) &&
                       LDPSC_FnsByHeadingRi_B) && LDPSC_FnsByLatSpdRi_B);

  /* RelationalOperator: '<S33>/Relational Operator8' incorporates:
   *  Constant: '<S68>/Constant'
   */
  LDPSC_MinLdwBySysSt_B = (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_ACTIVE);

  /* Logic: '<S60>/AND' incorporates:
   *  Logic: '<S60>/NOT'
   *  UnitDelay: '<S60>/Unit Delay'
   */
  LDPSC_EdgeRiseForMinLdw_B = (LDPSC_MinLdwBySysSt_B && (!LDPSC_EdgeRisWarming_B));

  /* Switch: '<S76>/Switch2' incorporates:
   *  Constant: '<S33>/V_Parameter27'
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S76>/GreaterThan'
   *  Switch: '<S76>/Switch'
   *  UnitDelay: '<S76>/Unit Delay'
   */
  if (LDPSC_EdgeRiseForMinLdw_B) {
    LDPSC_HdTiWarming_Sec = LDPSC_FnsDuraMn_C_Sec;
  } else if (LDPSC_HdTiWarming_Sec > LDPSAI_CycleTime_Sec) {
    /* Switch: '<S76>/Switch' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S76>/Subtract'
     *  UnitDelay: '<S76>/Unit Delay'
     */
    LDPSC_HdTiWarming_Sec -= LDPSAI_CycleTime_Sec;
  } else {
    /* UnitDelay: '<S76>/Unit Delay' incorporates:
     *  Constant: '<S76>/Constant1'
     *  Switch: '<S76>/Switch'
     */
    LDPSC_HdTiWarming_Sec = 0.0F;
  }

  /* End of Switch: '<S76>/Switch2' */

  /* RelationalOperator: '<S76>/GreaterThan1' incorporates:
   *  Constant: '<S76>/Constant2'
   *  UnitDelay: '<S76>/Unit Delay'
   */
  LDPSC_HoldForMinLdw_B = (LDPSC_HdTiWarming_Sec > 0.0F);

  /* Logic: '<S33>/Logical Operator11' incorporates:
   *  Constant: '<S33>/V_Parameter31'
   *  DataTypeConversion: '<S64>/Data Type Conversion'
   *  Logic: '<S33>/Logical Operator2'
   *  Logic: '<S33>/Logical Operator7'
   *  Logic: '<S33>/NOT6'
   *  S-Function (sfix_bitop): '<S64>/Bitwise AND'
   */
  LDPSC_FlagMinTimeLDW_B = (((!LDPSC_MinLdwBySysSt_B) || (!LDPSC_HoldForMinLdw_B))
    || ((((uint32_T)LDPSC_FnsCdsnEn_C_St) & 128U) == 0U));

  /* Logic: '<S33>/Logical Operator3' incorporates:
   *  Logic: '<S33>/Logical Operator1'
   */
  rtb_LogicalOperator_kelu = ((LDPSC_DgrFnsLf_B || LDPSC_DgrFnsRi_B) &&
    LDPSC_FlagMinTimeLDW_B);

  /* Switch: '<S69>/Switch' incorporates:
   *  Constant: '<S33>/V_Parameter9'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S69>/Max'
   *  Sum: '<S69>/Subtract'
   *  Switch: '<S69>/Switch1'
   *  UnaryMinus: '<S69>/Unary Minus'
   *  UnitDelay: '<S69>/Unit Delay'
   */
  if (rtb_LogicalOperator_kelu) {
    LDPSC_DlyTiTgtFns_Sec = fmaxf(LDPSC_DlyTiTgtFns_Sec, -LDPSAI_CycleTime_Sec)
      - LDPSAI_CycleTime_Sec;
  } else {
    LDPSC_DlyTiTgtFns_Sec = LDPSC_DlyTiTgtFns_C_Sec;
  }

  /* End of Switch: '<S69>/Switch' */

  /* Logic: '<S69>/AND' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S69>/LessThanOrEqual'
   *  UnaryMinus: '<S69>/Unary Minus1'
   *  UnitDelay: '<S69>/Unit Delay'
   */
  LDPSC_DgrFns_B = (rtb_LogicalOperator_kelu && (LDPSC_DlyTiTgtFns_Sec <=
    (-LDPSAI_CycleTime_Sec)));

  /* UnitDelay: '<S171>/Unit Delay' */
  rtb_UnitDelay_m0jr = LDPVSE_PrevVehStartupSpd_Kmph;

  /* DataTypeConversion: '<S171>/Data Type Conversion' incorporates:
   *  Inport: '<Root>/Inport50'
   *  UnitDelay: '<S171>/Unit Delay'
   */
  LDPVSE_PrevVehStartupSpd_Kmph = (uint8_T)LDPSAI_VehStartupSpdHMI_Kmph;

  /* MultiPortSwitch: '<S171>/Multiport Switch1' incorporates:
   *  UnitDelay: '<S171>/Unit Delay'
   */
  if (((int32_T)rtb_UnitDelay_m0jr) == 0) {
    /* MultiPortSwitch: '<S171>/Multiport Switch' incorporates:
     *  Constant: '<S171>/V_Parameter1'
     *  DataTypeConversion: '<S171>/Data Type Conversion1'
     *  DataTypeConversion: '<S171>/Data Type Conversion2'
     *  Inport: '<Root>/Inport49'
     */
    if (((int32_T)((uint8_T)NVRAM_LDPStartupSpd_Kmph)) == 0) {
      rtb_UnitDelay_m0jr = (uint8_T)LDPVSE_VehSpdTrsdMn_C_Kmph;
    } else {
      rtb_UnitDelay_m0jr = (uint8_T)NVRAM_LDPStartupSpd_Kmph;
    }

    /* End of MultiPortSwitch: '<S171>/Multiport Switch' */
  } else {
    rtb_UnitDelay_m0jr = LDPVSE_PrevVehStartupSpd_Kmph;
  }

  /* End of MultiPortSwitch: '<S171>/Multiport Switch1' */

  /* Switch: '<S171>/Switch2' incorporates:
   *  Constant: '<S171>/V_Parameter3'
   *  Constant: '<S171>/V_Parameter4'
   *  DataTypeConversion: '<S171>/Data Type Conversion3'
   *  RelationalOperator: '<S171>/GreaterThanOrEqual'
   *  RelationalOperator: '<S171>/GreaterThanOrEqual1'
   *  Switch: '<S171>/Switch3'
   */
  if (60 <= ((int32_T)rtb_UnitDelay_m0jr)) {
    /* Switch: '<S171>/Switch2' */
    LDPVSE_NVRAMVehStartupSpd__osjy = 60;
  } else if (45 >= ((int32_T)rtb_UnitDelay_m0jr)) {
    /* Switch: '<S171>/Switch3' incorporates:
     *  Constant: '<S171>/V_Parameter4'
     *  Switch: '<S171>/Switch2'
     */
    LDPVSE_NVRAMVehStartupSpd__osjy = 45;
  } else {
    /* Switch: '<S171>/Switch2' */
    LDPVSE_NVRAMVehStartupSpd__osjy = (int32_T)rtb_UnitDelay_m0jr;
  }

  /* End of Switch: '<S171>/Switch2' */

  /* Switch: '<S163>/Switch' incorporates:
   *  Constant: '<S160>/V_Parameter'
   *  Constant: '<S160>/V_Parameter2'
   *  Sum: '<S163>/Add'
   *  Sum: '<S163>/Add1'
   *  Switch: '<S163>/Switch1'
   *  UnitDelay: '<S163>/Unit Delay'
   */
  if (LDPVSE_BHysSpdVeh_B) {
    rtb_Abs = LDPVSE_VehSpdTrsdMx_C_Kmph + LDPVSE_VehSpdTrsdOfst_C_Kmph;
    rtb_Divide_l2e1 = ((real32_T)LDPVSE_NVRAMVehStartupSpd__osjy) -
      LDPVSE_VehSpdTrsdOfst_C_Kmph;
  } else {
    rtb_Abs = LDPVSE_VehSpdTrsdMx_C_Kmph;
    rtb_Divide_l2e1 = (real32_T)LDPVSE_NVRAMVehStartupSpd__osjy;
  }

  /* End of Switch: '<S163>/Switch' */

  /* Logic: '<S163>/AND' incorporates:
   *  Inport: '<Root>/Inport2'
   *  RelationalOperator: '<S163>/GreaterThan'
   *  RelationalOperator: '<S163>/GreaterThan1'
   *  UnitDelay: '<S163>/Unit Delay'
   */
  LDPVSE_BHysSpdVeh_B = ((rtb_Abs >= LDPSAI_SpdVelShow_Kmph) &&
    (LDPSAI_SpdVelShow_Kmph >= rtb_Divide_l2e1));

  /* Logic: '<S160>/Logical Operator2' incorporates:
   *  UnitDelay: '<S163>/Unit Delay'
   */
  rtb_VectorConcatenate[0] = !LDPVSE_BHysSpdVeh_B;

  /* Switch: '<S167>/Switch' incorporates:
   *  Constant: '<S160>/V_Parameter3'
   *  Constant: '<S160>/V_Parameter4'
   *  Sum: '<S167>/Add'
   *  UnitDelay: '<S167>/Unit Delay'
   */
  if (LDPVSE_UHysSteAgl_B) {
    rtb_Abs = LDPVSE_SteAglTrsdOfst_C_Dgr + LDPVSE_SteAglTrsdMx_C_Dgr;
  } else {
    rtb_Abs = LDPVSE_SteAglTrsdMx_C_Dgr;
  }

  /* End of Switch: '<S167>/Switch' */

  /* RelationalOperator: '<S167>/GreaterThan' incorporates:
   *  Abs: '<S160>/Abs1'
   *  Inport: '<Root>/Inport7'
   *  UnitDelay: '<S167>/Unit Delay'
   */
  LDPVSE_UHysSteAgl_B = (rtb_Abs >= fabsf(LDPSAI_WheSteAgl_Dgr));

  /* Logic: '<S160>/Logical Operator1' incorporates:
   *  UnitDelay: '<S167>/Unit Delay'
   */
  rtb_VectorConcatenate[1] = !LDPVSE_UHysSteAgl_B;

  /* Switch: '<S168>/Switch' incorporates:
   *  Constant: '<S160>/V_Parameter5'
   *  Constant: '<S160>/V_Parameter6'
   *  Sum: '<S168>/Add'
   *  UnitDelay: '<S168>/Unit Delay'
   */
  if (LDPVSE_UHysSteAglSpd_B) {
    rtb_Abs = LDPVSE_SteAglSpdTrsdOfst_C_Dgpm + LDPVSE_SteAglSpdTrsdMx_C_Dgpm;
  } else {
    rtb_Abs = LDPVSE_SteAglSpdTrsdMx_C_Dgpm;
  }

  /* End of Switch: '<S168>/Switch' */

  /* RelationalOperator: '<S168>/GreaterThan' incorporates:
   *  Abs: '<S160>/Abs2'
   *  Inport: '<Root>/Inport8'
   *  UnitDelay: '<S168>/Unit Delay'
   */
  LDPVSE_UHysSteAglSpd_B = (rtb_Abs >= fabsf(LDPSAI_SteAglSpd_Dgpm));

  /* Logic: '<S160>/Logical Operator3' incorporates:
   *  UnitDelay: '<S168>/Unit Delay'
   */
  rtb_VectorConcatenate[2] = !LDPVSE_UHysSteAglSpd_B;

  /* Switch: '<S164>/Switch' incorporates:
   *  Constant: '<S160>/V_Parameter10'
   *  Constant: '<S160>/V_Parameter11'
   *  Constant: '<S160>/V_Parameter12'
   *  Sum: '<S164>/Add'
   *  Sum: '<S164>/Add1'
   *  Switch: '<S164>/Switch1'
   *  UnitDelay: '<S164>/Unit Delay'
   */
  if (LDPVSE_BHysAccVehX_B) {
    rtb_Abs = LDPVSE_VehAccSpdTrsdXMx_C_Npkg + LDPVSE_VehAccSpdTrsdXOfst_C_Npkg;
    rtb_Divide_l2e1 = LDPVSE_VehAccSpdTrsdXMn_C_Npkg -
      LDPVSE_VehAccSpdTrsdXOfst_C_Npkg;
  } else {
    rtb_Abs = LDPVSE_VehAccSpdTrsdXMx_C_Npkg;
    rtb_Divide_l2e1 = LDPVSE_VehAccSpdTrsdXMn_C_Npkg;
  }

  /* End of Switch: '<S164>/Switch' */

  /* Logic: '<S164>/AND' incorporates:
   *  Inport: '<Root>/Inport3'
   *  RelationalOperator: '<S164>/GreaterThan'
   *  RelationalOperator: '<S164>/GreaterThan1'
   *  UnitDelay: '<S164>/Unit Delay'
   */
  LDPVSE_BHysAccVehX_B = ((rtb_Abs >= LDPSAI_VehAccSpdX_Npkg) &&
    (LDPSAI_VehAccSpdX_Npkg >= rtb_Divide_l2e1));

  /* Logic: '<S160>/Logical Operator5' incorporates:
   *  UnitDelay: '<S164>/Unit Delay'
   */
  rtb_VectorConcatenate[3] = !LDPVSE_BHysAccVehX_B;

  /* Switch: '<S169>/Switch' incorporates:
   *  Constant: '<S160>/V_Parameter13'
   *  Constant: '<S160>/V_Parameter14'
   *  Sum: '<S169>/Add'
   *  UnitDelay: '<S169>/Unit Delay'
   */
  if (LDPVSE_BHysAccVehY_B) {
    rtb_Abs = LDPVSE_VehAccSpdTrsdYOfst_C_Npkg + LDPVSE_VehAccSpdTrsdYMx_C_Npkg;
  } else {
    rtb_Abs = LDPVSE_VehAccSpdTrsdYMx_C_Npkg;
  }

  /* End of Switch: '<S169>/Switch' */

  /* RelationalOperator: '<S169>/GreaterThan' incorporates:
   *  Abs: '<S160>/Abs'
   *  Inport: '<Root>/Inport4'
   *  UnitDelay: '<S169>/Unit Delay'
   */
  LDPVSE_BHysAccVehY_B = (rtb_Abs >= fabsf(LDPSAI_VehAccSpdY_Npkg));

  /* Logic: '<S160>/Logical Operator6' incorporates:
   *  UnitDelay: '<S169>/Unit Delay'
   */
  rtb_VectorConcatenate[4] = !LDPVSE_BHysAccVehY_B;

  /* Lookup_n-D: '<S160>/Lookup Table' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPVSE_MaxCrvBySpd_ReMi = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDPVSE_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDPVSE_TrsdLnCltdCurvMx_Cr_Mps[0])), 7U);

  /* Lookup_n-D: '<S160>/Lookup Table1' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPVSE_HystCrvBySpd_ReMi = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDPVSE_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDPVSE_TrsdLnCltdCurvOfst_Cr_Mps[0])), 7U);

  /* Switch: '<S170>/Switch' incorporates:
   *  Sum: '<S170>/Add'
   *  UnitDelay: '<S170>/Unit Delay'
   */
  if (LDPVSE_UHysVehCurv_B) {
    rtb_Abs = LDPVSE_HystCrvBySpd_ReMi + LDPVSE_MaxCrvBySpd_ReMi;
  } else {
    rtb_Abs = LDPVSE_MaxCrvBySpd_ReMi;
  }

  /* End of Switch: '<S170>/Switch' */

  /* RelationalOperator: '<S170>/GreaterThan' incorporates:
   *  Inport: '<Root>/Inport5'
   *  UnitDelay: '<S170>/Unit Delay'
   */
  LDPVSE_UHysVehCurv_B = (rtb_Abs >= LDPSAI_VehCurv_ReMi);

  /* Logic: '<S160>/Logical Operator7' incorporates:
   *  UnitDelay: '<S170>/Unit Delay'
   */
  rtb_VectorConcatenate[5] = !LDPVSE_UHysVehCurv_B;

  /* Switch: '<S165>/Switch' incorporates:
   *  Constant: '<S160>/V_Parameter7'
   *  Constant: '<S160>/V_Parameter8'
   *  Constant: '<S160>/V_Parameter9'
   *  Sum: '<S165>/Add'
   *  Sum: '<S165>/Add1'
   *  Switch: '<S165>/Switch1'
   *  UnitDelay: '<S165>/Unit Delay'
   */
  if (LDPVSE_BHysLnWid_B) {
    rtb_Abs = LDPVSE_LnWidTrsdMx_C_Mi + LDPVSE_LnWidTrsdOfst_C_Mi;
    rtb_Divide_l2e1 = LDPVSE_LnWidTrsdMn_C_Mi - LDPVSE_LnWidTrsdOfst_C_Mi;
  } else {
    rtb_Abs = LDPVSE_LnWidTrsdMx_C_Mi;
    rtb_Divide_l2e1 = LDPVSE_LnWidTrsdMn_C_Mi;
  }

  /* End of Switch: '<S165>/Switch' */

  /* Logic: '<S165>/AND' incorporates:
   *  Inport: '<Root>/Inport13'
   *  RelationalOperator: '<S165>/GreaterThan'
   *  RelationalOperator: '<S165>/GreaterThan1'
   *  UnitDelay: '<S165>/Unit Delay'
   */
  LDPVSE_BHysLnWid_B = ((rtb_Abs >= LDPSAI_LnWidCalc_Mi) && (LDPSAI_LnWidCalc_Mi
    >= rtb_Divide_l2e1));

  /* Logic: '<S160>/Logical Operator' */
  rtb_LogicalOperator_kelu = (LDPDT_LnMakVldLf_B && LDPDT_LnMakVldRi_B);

  /* Switch: '<S160>/Switch' incorporates:
   *  Constant: '<S160>/V_Const'
   *  Logic: '<S160>/Logical Operator4'
   *  UnitDelay: '<S165>/Unit Delay'
   */
  if (rtb_LogicalOperator_kelu) {
    rtb_VectorConcatenate[6] = !LDPVSE_BHysLnWid_B;
  } else {
    rtb_VectorConcatenate[6] = false;
  }

  /* End of Switch: '<S160>/Switch' */

  /* MultiPortSwitch: '<S160>/Index Vector1' incorporates:
   *  Inport: '<Root>/Inport9'
   *  Logic: '<S160>/Logical Operator8'
   */
  rtb_VectorConcatenate[7] = !LDPSAI_LDPSwitchEn_B;

  /* S-Function (ex_sfun_set_bit): '<S172>/ex_sfun_set_bit' incorporates:
   *  Constant: '<S166>/Constant'
   */
  set_bit(0U, (boolean_T*)&rtb_VectorConcatenate[0], (uint8_T*)
          (&(LDPSA_SetBit_BS_Param_2[0])), ((uint8_T)8U), &rtb_ex_sfun_set_bit);

  /* SignalConversion: '<S160>/Signal Conversion2' incorporates:
   *  DataTypeConversion: '<S172>/Data Type Conversion1'
   */
  LDPVSE_IvldLDP_St = (uint8_T)rtb_ex_sfun_set_bit;

  /* RelationalOperator: '<S36>/Relational Operator38' incorporates:
   *  Constant: '<S36>/Constant33'
   *  Constant: '<S36>/V_Parameter46'
   *  S-Function (sfix_bitop): '<S36>/Bitwise Operator32'
   */
  LDPSC_CancelBySpecific_B = ((((int32_T)LDPVSE_IvldLDP_St) & ((int32_T)
    LDPSC_CclErrSpcLDP_C_St)) != 0);

  /* RelationalOperator: '<S36>/Relational Operator37' incorporates:
   *  Constant: '<S36>/Constant34'
   *  Constant: '<S36>/V_Parameter47'
   *  Inport: '<Root>/Inport38'
   *  S-Function (sfix_bitop): '<S36>/Bitwise Operator33'
   */
  LDPSC_CancelByVehSt_B = ((((int32_T)LDPSAI_VehStIvld_St) & ((int32_T)
    LDPSC_CclVehIvld_C_St)) != 0);

  /* RelationalOperator: '<S36>/Relational Operator32' incorporates:
   *  Constant: '<S36>/Constant35'
   *  Constant: '<S36>/V_Parameter41'
   *  Inport: '<Root>/Inport39'
   *  S-Function (sfix_bitop): '<S36>/Bitwise Operator34'
   */
  LDPSC_CancelByDrvSt_B = ((((int32_T)LDPSAI_IvldStDrv_St) & ((int32_T)
    LDPSC_CclDrvIVld_C_St)) != 0);

  /* RelationalOperator: '<S36>/Relational Operator33' incorporates:
   *  Constant: '<S36>/Constant29'
   *  Constant: '<S36>/V_Parameter42'
   *  Inport: '<Root>/Inport40'
   *  S-Function (sfix_bitop): '<S36>/Bitwise Operator28'
   */
  LDPSC_CancelByCtrlSt_B = ((((int32_T)LDPSAI_CtrlStEn_St) & ((int32_T)
    LDPSC_CclDrvActCtrl_C_St)) != 0);

  /* RelationalOperator: '<S36>/Relational Operator34' incorporates:
   *  Constant: '<S36>/Constant30'
   *  Constant: '<S36>/V_Parameter43'
   *  Inport: '<Root>/Inport41'
   *  S-Function (sfix_bitop): '<S36>/Bitwise Operator29'
   */
  LDPSC_CancelBySysSt_B = ((((int32_T)LDPSAI_StError_St) & ((int32_T)
    LDPSC_CclVehSysErr_C_St)) != 0);

  /* RelationalOperator: '<S36>/Relational Operator35' incorporates:
   *  Constant: '<S36>/Constant31'
   *  Constant: '<S36>/V_Parameter44'
   *  Inport: '<Root>/Inport42'
   *  S-Function (sfix_bitop): '<S36>/Bitwise Operator30'
   */
  LDPSC_CancelByAvlSt_B = ((((int32_T)LDPSAI_CtrlStNoAvlb_St) & ((int32_T)
    LDPSC_CclNoAvlbVehSys_C_St)) != 0);

  /* RelationalOperator: '<S36>/Relational Operator36' incorporates:
   *  Constant: '<S36>/Constant32'
   *  Constant: '<S36>/V_Parameter45'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S36>/Bitwise Operator31'
   */
  LDPSC_CancelByPrjSpec_B = ((((int32_T)LDPSAI_PrjSpecQu_St) & ((int32_T)
    LDPSC_CclFctCstm_St)) != 0);

  /* RelationalOperator: '<S40>/Relational Operator47' incorporates:
   *  Constant: '<S42>/Constant'
   */
  LDPSC_MaxDurationBySysSt_B = (((uint32_T)LDPSC_SysOld_St) ==
    E_LDPState_nu_ACTIVE);

  /* Logic: '<S41>/AND' incorporates:
   *  Logic: '<S41>/NOT'
   *  UnitDelay: '<S41>/Unit Delay'
   */
  LDPSC_EdgRiseForSysSt_B = (LDPSC_MaxDurationBySysSt_B &&
    (!LDPSC_EdgeRisWarmMx_B));

  /* Switch: '<S43>/Switch2' incorporates:
   *  Constant: '<S40>/V_Parameter1'
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S43>/GreaterThan'
   *  Switch: '<S43>/Switch'
   *  UnitDelay: '<S43>/Unit Delay'
   */
  if (LDPSC_EdgRiseForSysSt_B) {
    LDPSC_HdTiWarmMx_Sec = LDPSC_WarmMxTi_C_Sec;
  } else if (LDPSC_HdTiWarmMx_Sec > LDPSAI_CycleTime_Sec) {
    /* Switch: '<S43>/Switch' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S43>/Subtract'
     *  UnitDelay: '<S43>/Unit Delay'
     */
    LDPSC_HdTiWarmMx_Sec -= LDPSAI_CycleTime_Sec;
  } else {
    /* UnitDelay: '<S43>/Unit Delay' incorporates:
     *  Constant: '<S43>/Constant1'
     *  Switch: '<S43>/Switch'
     */
    LDPSC_HdTiWarmMx_Sec = 0.0F;
  }

  /* End of Switch: '<S43>/Switch2' */

  /* Logic: '<S40>/Logical Operator22' incorporates:
   *  Constant: '<S43>/Constant2'
   *  RelationalOperator: '<S43>/GreaterThan1'
   *  UnitDelay: '<S43>/Unit Delay'
   */
  LDPSC_MaxDurationByStDly_B = (LDPSC_HdTiWarmMx_Sec <= 0.0F);

  /* Logic: '<S40>/Logical Operator23' */
  LDPSC_TiWarmMx_B = (LDPSC_MaxDurationBySysSt_B && LDPSC_MaxDurationByStDly_B);

  /* RelationalOperator: '<S39>/Relational Operator42' incorporates:
   *  Constant: '<S39>/Constant36'
   *  Constant: '<S39>/V_Parameter57'
   *  S-Function (sfix_bitop): '<S39>/Bitwise Operator35'
   */
  LDPSC_ErrSideBySideCondLf_B = ((((int32_T)LDPVSE_SidCdtnLDPLf_St) & ((int32_T)
    LDPSC_SidCdtnCclLf_C_St)) != 0);

  /* RelationalOperator: '<S39>/Relational Operator43' incorporates:
   *  Constant: '<S39>/Constant37'
   *  Constant: '<S39>/V_Parameter58'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S39>/Bitwise Operator36'
   */
  LDPSC_ErrSidByPrjSpecLf_B = ((((int32_T)LDPSAI_PrjSpecQu_St) & ((int32_T)
    LDPSC_ErrCstmCclLf_C_St)) != 0);

  /* Logic: '<S39>/Logical Operator18' */
  LDPSC_ErrSidCdtnLf_B = ((LDPSC_ErrSideByTrigLf_B ||
    LDPSC_ErrSideBySideCondLf_B) || LDPSC_ErrSidByPrjSpecLf_B);

  /* RelationalOperator: '<S39>/Relational Operator41' incorporates:
   *  Constant: '<S39>/V_Parameter56'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDPSC_SideCondByDgrLf_B = (LDPSC_LstPrevDgrSide_St == LDPSC_DgrSideLf_C_St);

  /* Logic: '<S39>/Logical Operator15' */
  LDPSC_CanelBySideLf_B = (LDPSC_ErrSidCdtnLf_B && LDPSC_SideCondByDgrLf_B);

  /* RelationalOperator: '<S39>/Relational Operator44' incorporates:
   *  Constant: '<S39>/V_Parameter59'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDPSC_SideCondByDgrRi_B = (LDPSC_LstPrevDgrSide_St == LDPSC_DgrSideRi_C_St);

  /* RelationalOperator: '<S39>/Relational Operator45' incorporates:
   *  Constant: '<S39>/Constant38'
   *  Constant: '<S39>/V_Parameter60'
   *  S-Function (sfix_bitop): '<S39>/Bitwise Operator37'
   */
  LDPSC_ErrSideBySideCondRi_B = ((((int32_T)LDPVSE_SidCdtnLDPRi_St) & ((int32_T)
    LDPSC_SidCdtnCclRi_C_St)) != 0);

  /* RelationalOperator: '<S39>/Relational Operator46' incorporates:
   *  Constant: '<S39>/Constant39'
   *  Constant: '<S39>/V_Parameter61'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S39>/Bitwise Operator38'
   */
  LDPSC_ErrSidByPrjSpecRi_B = ((((int32_T)LDPSAI_PrjSpecQu_St) & ((int32_T)
    LDPSC_ErrCstmCclRi_C_St)) != 0);

  /* Logic: '<S39>/Logical Operator2' */
  LDPSC_ErrSidCdtnRi_B = ((LDPSC_ErrSideByTrigRi_B ||
    LDPSC_ErrSideBySideCondRi_B) || LDPSC_ErrSidByPrjSpecRi_B);

  /* Logic: '<S39>/Logical Operator1' */
  LDPSC_CanelBySideRi_B = (LDPSC_SideCondByDgrRi_B && LDPSC_ErrSidCdtnRi_B);

  /* Logic: '<S39>/Logical Operator16' */
  LDPSC_ErrSidCdtn_B = (LDPSC_CanelBySideLf_B || LDPSC_CanelBySideRi_B);

  /* Sum: '<S38>/Add3' incorporates:
   *  Constant: '<S38>/V_Parameter50'
   *  Constant: '<S38>/V_Parameter51'
   *  Sum: '<S38>/Add1'
   */
  rtb_Abs = LDPSC_TgtTrajPstnY_C_Mi + LDPSC_NoDgrCclOfst_C_Mi;

  /* Logic: '<S38>/Logical Operator24' incorporates:
   *  Constant: '<S38>/V_Parameter53'
   *  RelationalOperator: '<S38>/Relational Operator48'
   *  RelationalOperator: '<S38>/Relational Operator49'
   *  Sum: '<S38>/Add3'
   *  UnaryMinus: '<S38>/Unary Minus5'
   */
  LDPSC_CLatDevByDlcLf_B = (((-LDPSC_DgrCclOfst_C_Mi) > LDPDT_DstcToLnLf_Mi) ||
    (LDPDT_DstcToLnLf_Mi > rtb_Abs));

  /* RelationalOperator: '<S38>/Relational Operator39' incorporates:
   *  Constant: '<S38>/V_Parameter48'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDPSC_CLatDevByDgrLf_B = (LDPSC_LstPrevDgrSide_St == LDPSC_DgrSideLf_C_St);

  /* Logic: '<S38>/Logical Operator12' */
  LDPSC_CclLatDevLf_B = (LDPSC_CLatDevByDlcLf_B && LDPSC_CLatDevByDgrLf_B);

  /* RelationalOperator: '<S38>/Relational Operator40' incorporates:
   *  Constant: '<S38>/V_Parameter49'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDPSC_CLatDevByDlcRi_B = (LDPSC_LstPrevDgrSide_St == LDPSC_DgrSideRi_C_St);

  /* Logic: '<S38>/Logical Operator25' incorporates:
   *  Constant: '<S38>/V_Parameter1'
   *  RelationalOperator: '<S38>/Relational Operator50'
   *  RelationalOperator: '<S38>/Relational Operator51'
   *  UnaryMinus: '<S38>/Unary Minus1'
   */
  LDPSC_CLatDevByDgrRi_B = (((-rtb_Abs) > LDPDT_DstcToLnRi_Mi) ||
    (LDPDT_DstcToLnRi_Mi > LDPSC_DgrCclOfst_C_Mi));

  /* Logic: '<S38>/Logical Operator14' */
  LDPSC_CclLatDevRi_B = (LDPSC_CLatDevByDlcRi_B && LDPSC_CLatDevByDgrRi_B);

  /* Logic: '<S38>/Logical Operator13' */
  LDPSC_CclLatDev_B = (LDPSC_CclLatDevLf_B || LDPSC_CclLatDevRi_B);

  /* Logic: '<S36>/Logical Operator11' */
  LDPSC_Cancel_B = (((((((((LDPSC_CancelBySpecific_B || LDPSC_CancelByVehSt_B) ||
    LDPSC_CancelByDrvSt_B) || LDPSC_CancelByCtrlSt_B) || LDPSC_CancelBySysSt_B) ||
                        LDPSC_CancelByAvlSt_B) || LDPSC_CancelByPrjSpec_B) ||
                      LDPSC_TiWarmMx_B) || LDPSC_ErrSidCdtn_B) ||
                    LDPSC_CclLatDev_B);

  /* RelationalOperator: '<S37>/Relational Operator2' incorporates:
   *  Constant: '<S37>/Constant'
   *  Constant: '<S37>/V_Parameter17'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator'
   */
  LDPSC_AbortBySpecific_B = ((((int32_T)LDPVSE_IvldLDP_St) & ((int32_T)
    LDPSC_AbtErrSpcLDP_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator1' incorporates:
   *  Constant: '<S37>/Constant1'
   *  Constant: '<S37>/V_Parameter9'
   *  Inport: '<Root>/Inport38'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator1'
   */
  LDPSC_AbortByVehSt_B = ((((int32_T)LDPSAI_VehStIvld_St) & ((int32_T)
    LDPSC_AbtVehIvld_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator3' incorporates:
   *  Constant: '<S37>/Constant2'
   *  Constant: '<S37>/V_Parameter10'
   *  Inport: '<Root>/Inport39'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator2'
   */
  LDPSC_AbortByDrvSt_B = ((((int32_T)LDPSAI_IvldStDrv_St) & ((int32_T)
    LDPSC_AbtDrvIVld_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator4' incorporates:
   *  Constant: '<S37>/Constant3'
   *  Constant: '<S37>/V_Parameter11'
   *  Inport: '<Root>/Inport40'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator3'
   */
  LDPSC_AbortByCtrlSt_B = ((((int32_T)LDPSAI_CtrlStEn_St) & ((int32_T)
    LDPSC_AbtDrvActCtrl_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator5' incorporates:
   *  Constant: '<S37>/Constant4'
   *  Constant: '<S37>/V_Parameter12'
   *  Inport: '<Root>/Inport41'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator4'
   */
  LDPSC_AbortBySysSt_B = ((((int32_T)LDPSAI_StError_St) & ((int32_T)
    LDPSC_AbtVehSysErr_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator6' incorporates:
   *  Constant: '<S37>/Constant5'
   *  Constant: '<S37>/V_Parameter13'
   *  Inport: '<Root>/Inport42'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator5'
   */
  LDPSC_AbortByAvlSt_B = ((((int32_T)LDPSAI_CtrlStNoAvlb_St) & ((int32_T)
    LDPSC_AbtNoAvlbVehSys_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator7' incorporates:
   *  Constant: '<S37>/Constant6'
   *  Constant: '<S37>/V_Parameter14'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator6'
   */
  LDPSC_AbortByPrjSpec_B = ((((int32_T)LDPSAI_PrjSpecQu_St) & ((int32_T)
    LDPSC_AbtFctCstm_C_St)) != 0);

  /* Logic: '<S37>/Logical Operator6' */
  LDPSC_Abort_B = ((((((LDPSC_AbortBySpecific_B || LDPSC_AbortByVehSt_B) ||
                       LDPSC_AbortByDrvSt_B) || LDPSC_AbortByCtrlSt_B) ||
                     LDPSC_AbortBySysSt_B) || LDPSC_AbortByAvlSt_B) ||
                   LDPSC_AbortByPrjSpec_B);

  /* Logic: '<S37>/Logical Operator1' incorporates:
   *  Constant: '<S37>/Constant10'
   *  Constant: '<S37>/Constant11'
   *  Constant: '<S37>/Constant12'
   *  Constant: '<S37>/Constant13'
   *  Constant: '<S37>/Constant7'
   *  Constant: '<S37>/Constant8'
   *  Constant: '<S37>/Constant9'
   *  Constant: '<S37>/V_Parameter15'
   *  Constant: '<S37>/V_Parameter16'
   *  Constant: '<S37>/V_Parameter18'
   *  Constant: '<S37>/V_Parameter19'
   *  Constant: '<S37>/V_Parameter20'
   *  Constant: '<S37>/V_Parameter21'
   *  Constant: '<S37>/V_Parameter22'
   *  Inport: '<Root>/Inport38'
   *  Inport: '<Root>/Inport39'
   *  Inport: '<Root>/Inport40'
   *  Inport: '<Root>/Inport41'
   *  Inport: '<Root>/Inport42'
   *  Inport: '<Root>/Inport43'
   *  Logic: '<S37>/Logical Operator2'
   *  RelationalOperator: '<S37>/Relational Operator10'
   *  RelationalOperator: '<S37>/Relational Operator11'
   *  RelationalOperator: '<S37>/Relational Operator12'
   *  RelationalOperator: '<S37>/Relational Operator13'
   *  RelationalOperator: '<S37>/Relational Operator14'
   *  RelationalOperator: '<S37>/Relational Operator8'
   *  RelationalOperator: '<S37>/Relational Operator9'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator10'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator11'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator12'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator13'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator7'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator8'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator9'
   */
  LDPSC_StrgRdy_B = ((((((((!LDPSC_Abort_B) && ((((int32_T)LDPVSE_IvldLDP_St) &
    ((int32_T)LDPSC_StrgRdyErrSpcLDP_C_St)) == 0)) && ((((int32_T)
    LDPSAI_VehStIvld_St) & ((int32_T)LDPSC_StrgRdyVehIvld_C_St)) == 0)) &&
    ((((int32_T)LDPSAI_IvldStDrv_St) & ((int32_T)LDPSC_StrgRdyDrvIVld_C_St)) ==
     0)) && ((((int32_T)LDPSAI_CtrlStEn_St) & ((int32_T)
    LDPSC_StrgRdyDrvActCtrl_C_St)) == 0)) && ((((int32_T)LDPSAI_StError_St) &
    ((int32_T)LDPSC_StrgRdyVehSysErr_C_St)) == 0)) && ((((int32_T)
    LDPSAI_CtrlStNoAvlb_St) & ((int32_T)LDPSC_StrgRdyNoAvlbVehSys_C_St)) == 0)) &&
                     ((((int32_T)LDPSAI_PrjSpecQu_St) & ((int32_T)
    LDPSC_StrgRdyFctCstm_C_St)) == 0));

  /* Switch: '<S32>/Switch' incorporates:
   *  Switch: '<S32>/Switch1'
   */
  if (LDPSC_Abort_B) {
    /* Switch: '<S32>/Switch' incorporates:
     *  Constant: '<S32>/V_Parameter63'
     */
    LDPSC_RampoutTime_Sec = LDPSC_TiAbtDegr_C_Sec;
  } else if (LDPSC_StrgRdy_B) {
    /* Switch: '<S32>/Switch2' incorporates:
     *  Switch: '<S32>/Switch1'
     *  Switch: '<S32>/Switch3'
     */
    if (LDPSC_DgrFns_B) {
      /* Switch: '<S32>/Switch' incorporates:
       *  Constant: '<S32>/V_Parameter2'
       */
      LDPSC_RampoutTime_Sec = LDPSC_TiDgrFnsDegr_C_Sec;
    } else if (LDPSC_Cancel_B) {
      /* Switch: '<S32>/Switch3' incorporates:
       *  Constant: '<S32>/V_Parameter3'
       *  Switch: '<S32>/Switch'
       */
      LDPSC_RampoutTime_Sec = LDPSC_TiCclDegr_C_Sec;
    } else {
      /* Switch: '<S32>/Switch' incorporates:
       *  Constant: '<S32>/Constant40'
       *  Switch: '<S32>/Switch3'
       */
      LDPSC_RampoutTime_Sec = 0.0F;
    }

    /* End of Switch: '<S32>/Switch2' */
  } else {
    /* Switch: '<S32>/Switch' incorporates:
     *  Constant: '<S32>/V_Parameter1'
     *  Switch: '<S32>/Switch1'
     */
    LDPSC_RampoutTime_Sec = LDPSC_TiStrgRdyDegr_C_Sec;
  }

  /* End of Switch: '<S32>/Switch' */

  /* Logic: '<S32>/Logical Operator22' incorporates:
   *  Chart: '<S4>/LDP_State'
   */
  rtb_NOT_fftx = !LDPSC_StrgRdy_B;

  /* Logic: '<S32>/Logical Operator1' incorporates:
   *  Logic: '<S32>/Logical Operator22'
   */
  LDPSC_Degradation_B = (((LDPSC_Abort_B || rtb_NOT_fftx) || LDPSC_DgrFns_B) ||
    LDPSC_Cancel_B);

  /* Logic: '<S58>/AND' incorporates:
   *  Logic: '<S58>/NOT'
   *  UnitDelay: '<S58>/Unit Delay'
   */
  LDPSC_DegradationEdgeRise_B = (LDPSC_Degradation_B && (!LDPSC_EdgeRisDegr_B));

  /* Logic: '<S32>/Logical Operator2' incorporates:
   *  UnitDelay: '<S32>/UnitDelay'
   */
  LDPSC_Degr_B = LDPSC_DegrOld_B;

  /* Logic: '<S32>/Logical Operator' */
  rtb_LDPSC_VehicleInvalid_B = (LDPSC_Degr_B && LDPSC_DegradationEdgeRise_B);

  /* Switch: '<S59>/Switch3' */
  if (rtb_LDPSC_VehicleInvalid_B) {
    /* Switch: '<S78>/Switch1' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S59>/Subtract1'
     */
    LDPSC_HdTiDegr_Sec = LDPSAI_CycleTime_Sec + LDPSC_RampoutTime_Sec;
  }

  /* End of Switch: '<S59>/Switch3' */

  /* Logic: '<S32>/Logical Operator2' incorporates:
   *  Constant: '<S59>/Constant4'
   *  Logic: '<S59>/OR'
   *  RelationalOperator: '<S59>/GreaterThan2'
   */
  LDPSC_Degr_B = ((!rtb_LDPSC_VehicleInvalid_B) && (LDPSC_HdTiDegr_Sec <=
    1.0E-5F));

  /* Switch: '<S78>/Switch1' incorporates:
   *  Inport: '<Root>/Inport45'
   *  Sum: '<S59>/Subtract2'
   */
  LDPSC_HdTiDegr_Sec -= LDPSAI_CycleTime_Sec;

  /* RelationalOperator: '<S37>/Relational Operator21' incorporates:
   *  Constant: '<S37>/Constant18'
   *  Constant: '<S37>/V_Parameter28'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator18'
   */
  LDPSC_SuppBySpecific_B = ((((int32_T)LDPVSE_IvldLDP_St) & ((int32_T)
    LDPSC_SuppErrSpcLDP_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator20' incorporates:
   *  Constant: '<S37>/Constant19'
   *  Constant: '<S37>/V_Parameter29'
   *  Inport: '<Root>/Inport38'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator19'
   */
  LDPSC_SuppByVehSt_B = ((((int32_T)LDPSAI_VehStIvld_St) & ((int32_T)
    LDPSC_SuppVehIvld_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator15' incorporates:
   *  Constant: '<S37>/Constant20'
   *  Constant: '<S37>/V_Parameter23'
   *  Inport: '<Root>/Inport39'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator20'
   */
  LDPSC_SuppByDrvSt_B = ((((int32_T)LDPSAI_IvldStDrv_St) & ((int32_T)
    LDPSC_SuppDrvIvld_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator16' incorporates:
   *  Constant: '<S37>/Constant14'
   *  Constant: '<S37>/V_Parameter24'
   *  Inport: '<Root>/Inport40'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator14'
   */
  LDPSC_SuppByCtrlSt_B = ((((int32_T)LDPSAI_CtrlStEn_St) & ((int32_T)
    LDPSC_SuppDrvActCtrl_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator17' incorporates:
   *  Constant: '<S37>/Constant15'
   *  Constant: '<S37>/V_Parameter25'
   *  Inport: '<Root>/Inport41'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator15'
   */
  LDPSC_SuppBySysSt_B = ((((int32_T)LDPSAI_StError_St) & ((int32_T)
    LDPSC_SuppVehSysErr_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator18' incorporates:
   *  Constant: '<S37>/Constant16'
   *  Constant: '<S37>/V_Parameter26'
   *  Inport: '<Root>/Inport42'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator16'
   */
  LDPSC_SuppyByAvlSt_B = ((((int32_T)LDPSAI_CtrlStNoAvlb_St) & ((int32_T)
    LDPSC_SuppNoAvlbVehSys_C_St)) != 0);

  /* RelationalOperator: '<S37>/Relational Operator19' incorporates:
   *  Constant: '<S37>/Constant17'
   *  Constant: '<S37>/V_Parameter27'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator17'
   */
  LDPSC_SuppPrjSpec_B = ((((int32_T)LDPSAI_PrjSpecQu_St) & ((int32_T)
    LDPSC_SuppFctCstm_C_St)) != 0);

  /* Logic: '<S37>/Logical Operator3' */
  LDPSC_Suppresion_B = ((((((LDPSC_SuppBySpecific_B || LDPSC_SuppByVehSt_B) ||
    LDPSC_SuppByDrvSt_B) || LDPSC_SuppByCtrlSt_B) || LDPSC_SuppBySysSt_B) ||
    LDPSC_SuppyByAvlSt_B) || LDPSC_SuppPrjSpec_B);

  /* RelationalOperator: '<S37>/Relational Operator28' incorporates:
   *  Constant: '<S37>/Constant25'
   *  Constant: '<S37>/V_Parameter35'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator25'
   */
  LDPSC_WeakRdyBySpecific_B = ((((int32_T)LDPVSE_IvldLDP_St) & ((int32_T)
    LDPSC_WkRdyErrSpcLDP_C_St)) == 0);

  /* RelationalOperator: '<S37>/Relational Operator27' incorporates:
   *  Constant: '<S37>/Constant26'
   *  Constant: '<S37>/V_Parameter36'
   *  Inport: '<Root>/Inport38'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator26'
   */
  LDPSC_WeakRdyByVehSt_B = ((((int32_T)LDPSAI_VehStIvld_St) & ((int32_T)
    LDPSC_WkRdyVehIvld_C_St)) == 0);

  /* RelationalOperator: '<S37>/Relational Operator22' incorporates:
   *  Constant: '<S37>/Constant27'
   *  Constant: '<S37>/V_Parameter30'
   *  Inport: '<Root>/Inport39'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator27'
   */
  LDPSC_WeakRdyByDrvSt_B = ((((int32_T)LDPSAI_IvldStDrv_St) & ((int32_T)
    LDPSC_WkRdyDrvIVld_C_St)) == 0);

  /* RelationalOperator: '<S37>/Relational Operator23' incorporates:
   *  Constant: '<S37>/Constant21'
   *  Constant: '<S37>/V_Parameter31'
   *  Inport: '<Root>/Inport40'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator21'
   */
  LDPSC_WeakRdyByCtrlSt_B = ((((int32_T)LDPSAI_CtrlStEn_St) & ((int32_T)
    LDPSC_WkRdyDrvActCtrl_C_St)) == 0);

  /* RelationalOperator: '<S37>/Relational Operator24' incorporates:
   *  Constant: '<S37>/Constant22'
   *  Constant: '<S37>/V_Parameter32'
   *  Inport: '<Root>/Inport41'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator22'
   */
  LDPSC_WeakRdyBySysSt_B = ((((int32_T)LDPSAI_StError_St) & ((int32_T)
    LDPSC_WkRdyVehSysErr_C_St)) == 0);

  /* RelationalOperator: '<S37>/Relational Operator25' incorporates:
   *  Constant: '<S37>/Constant23'
   *  Constant: '<S37>/V_Parameter33'
   *  Inport: '<Root>/Inport42'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator23'
   */
  LDPSC_WeakRdyByAvlSt_B = ((((int32_T)LDPSAI_CtrlStNoAvlb_St) & ((int32_T)
    LDPSC_WkRdyNoAvlbVehSys_C_St)) == 0);

  /* RelationalOperator: '<S37>/Relational Operator26' incorporates:
   *  Constant: '<S37>/Constant24'
   *  Constant: '<S37>/V_Parameter34'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S37>/Bitwise Operator24'
   */
  LDPSC_WeakRdyByPrjSpec_B = ((((int32_T)LDPSAI_PrjSpecQu_St) & ((int32_T)
    LDPSC_WkRdyFctCstm_C_St)) == 0);

  /* Logic: '<S37>/Logical Operator4' incorporates:
   *  Logic: '<S37>/Logical Operator5'
   */
  LDPSC_WkRdy_B = ((((((((!LDPSC_Suppresion_B) && LDPSC_WeakRdyBySpecific_B) &&
                        LDPSC_WeakRdyByVehSt_B) && LDPSC_WeakRdyByDrvSt_B) &&
                      LDPSC_WeakRdyByCtrlSt_B) && LDPSC_WeakRdyBySysSt_B) &&
                    LDPSC_WeakRdyByAvlSt_B) && LDPSC_WeakRdyByPrjSpec_B);

  /* RelationalOperator: '<S37>/Relational Operator29' incorporates:
   *  Constant: '<S48>/Constant'
   */
  rtb_RelationalOperator29 = (((uint32_T)LDPSC_SysOld_St) ==
    E_LDPState_nu_ACTIVE);

  /* Logic: '<S37>/AND' incorporates:
   *  Constant: '<S37>/Constant30'
   *  Constant: '<S49>/Constant'
   *  RelationalOperator: '<S37>/Relational Operator30'
   *  RelationalOperator: '<S37>/Relational Operator32'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  rtb_LDPSC_VehicleInvalid_B = ((((uint32_T)LDPSC_SysOld_St) ==
    E_LDPState_nu_RAMPOUT) && (((int32_T)LDPSC_LstPrevDgrSide_St) == 1));

  /* Logic: '<S37>/Logical Operator9' */
  LDPSC_BlockTimeBySysOut_B = (rtb_RelationalOperator29 ||
    rtb_LDPSC_VehicleInvalid_B);

  /* Switch: '<S54>/Switch' incorporates:
   *  Constant: '<S54>/Constant'
   *  Logic: '<S46>/AND'
   *  Logic: '<S46>/NOT'
   *  Switch: '<S54>/Switch1'
   *  UnitDelay: '<S46>/Unit Delay'
   *  UnitDelay: '<S54>/Unit Delay'
   */
  if (rtb_RelationalOperator29 && (!LDPSC_EdgeRisActive_B)) {
    LDPSC_ActiveStopWatch_sec = 0.0F;
  } else {
    if (rtb_RelationalOperator29) {
      /* UnitDelay: '<S54>/Unit Delay' incorporates:
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S54>/Add'
       *  Switch: '<S54>/Switch1'
       */
      LDPSC_ActiveStopWatch_sec = LDPSAI_CycleTime_Sec +
        LDPSC_ActiveStopWatch_sec;
    }
  }

  /* End of Switch: '<S54>/Switch' */

  /* Switch: '<S37>/Switch' incorporates:
   *  Constant: '<S37>/V_Parameter2'
   *  RelationalOperator: '<S37>/GreaterThan'
   *  UnitDelay: '<S54>/Unit Delay'
   */
  if (LDPSC_ActiveStopWatch_sec >= LDPSC_ContinuActiveTi_C_Sec) {
    /* Switch: '<S37>/Switch' incorporates:
     *  Constant: '<S37>/V_Parameter4'
     */
    LDPSC_WRBlockTime_Sec = LDPSC_ContiActiveTiFns_C_Sec;
  } else {
    /* Switch: '<S37>/Switch' incorporates:
     *  Constant: '<S37>/Constant33'
     */
    LDPSC_WRBlockTime_Sec = 4.0F;
  }

  /* End of Switch: '<S37>/Switch' */

  /* Logic: '<S37>/Logical Operator8' incorporates:
   *  Constant: '<S50>/Constant'
   *  RelationalOperator: '<S37>/Relational Operator31'
   */
  rtb_LogicalOperator8 = (rtb_LDPSC_VehicleInvalid_B || (((uint32_T)
    LDPSC_SysOld_St) == E_LDPState_nu_PASSIVE));

  /* Logic: '<S44>/AND' incorporates:
   *  Logic: '<S44>/NOT'
   *  UnitDelay: '<S44>/Unit Delay'
   */
  LDPSC_RawBlockTimeByRampOut_B = ((!rtb_LogicalOperator8) && LDPSC_EdgeRisFns_B);

  /* Switch: '<S56>/Switch2' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S56>/GreaterThan'
   *  Switch: '<S56>/Switch'
   *  UnitDelay: '<S56>/Unit Delay'
   */
  if (LDPSC_RawBlockTimeByRampOut_B) {
    LDPSC_HdTiFns_Sec = LDPSC_WRBlockTime_Sec;
  } else if (LDPSC_HdTiFns_Sec > LDPSAI_CycleTime_Sec) {
    /* Switch: '<S56>/Switch' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S56>/Subtract'
     *  UnitDelay: '<S56>/Unit Delay'
     */
    LDPSC_HdTiFns_Sec -= LDPSAI_CycleTime_Sec;
  } else {
    /* UnitDelay: '<S56>/Unit Delay' incorporates:
     *  Constant: '<S56>/Constant1'
     *  Switch: '<S56>/Switch'
     */
    LDPSC_HdTiFns_Sec = 0.0F;
  }

  /* End of Switch: '<S56>/Switch2' */

  /* RelationalOperator: '<S56>/GreaterThan1' incorporates:
   *  Constant: '<S56>/Constant2'
   *  UnitDelay: '<S56>/Unit Delay'
   */
  LDPSC_BlockTimeByRampOut_B = (LDPSC_HdTiFns_Sec > 0.0F);

  /* Logic: '<S37>/Logical Operator7' incorporates:
   *  Logic: '<S37>/Logical Operator10'
   */
  LDPSC_BlockTime_B = ((!LDPSC_BlockTimeBySysOut_B) &&
                       (!LDPSC_BlockTimeByRampOut_B));

  /* RelationalOperator: '<S113>/Relational Operator21' incorporates:
   *  Constant: '<S113>/Constant18'
   *  Constant: '<S113>/V_Parameter28'
   *  S-Function (sfix_bitop): '<S113>/Bitwise Operator18'
   */
  rtb_LDPSC_VehicleInvalid_B = ((((int32_T)LDPVSE_IvldLDP_St) & ((int32_T)
    LDPSC_SuppVehicleInvalid_C_St)) != 0);

  /* Abs: '<S115>/Abs' incorporates:
   *  Inport: '<Root>/Inport31'
   */
  rtb_Abs = fabsf(LDPSAI_VehYawRate_rps);

  /* Switch: '<S119>/Switch' incorporates:
   *  Constant: '<S115>/Constant10'
   *  Constant: '<S115>/Constant9'
   *  Constant: '<S119>/Constant'
   *  RelationalOperator: '<S119>/Less Than'
   *  RelationalOperator: '<S119>/Less Than1'
   *  Sum: '<S115>/Add2'
   *  UnitDelay: '<S119>/Unit Delay'
   */
  if ((LDPSC_VehYawRateMax_C_rps + LDPSC_VehYawRateHyst_C_rps) < rtb_Abs) {
    LDPSC_VehYawRateHyst_bool = true;
  } else {
    LDPSC_VehYawRateHyst_bool = ((rtb_Abs >= LDPSC_VehYawRateMax_C_rps) &&
      (LDPSC_VehYawRateHyst_bool));
  }

  /* End of Switch: '<S119>/Switch' */

  /* Switch: '<S109>/Switch' incorporates:
   *  Constant: '<S109>/V_Parameter1'
   *  Constant: '<S109>/V_Parameter2'
   *  Constant: '<S109>/V_Parameter32'
   *  Constant: '<S109>/V_Parameter4'
   *  RelationalOperator: '<S109>/Relational Operator1'
   *  RelationalOperator: '<S109>/Relational Operator2'
   *  RelationalOperator: '<S109>/Relational Operator9'
   *  S-Function (sfix_bitop): '<S109>/Bitwise AND'
   *  Switch: '<S109>/Switch1'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  if (LDPSC_LstPrevDgrSide_St == LDPSC_DgrSideLf_C_St) {
    rtb_LDPSC_VelYInvalid_B = ((((uint32_T)LDPVSE_SidCdtnLDPLf_St) & 2U) != 0U);
  } else if (LDPSC_LstPrevDgrSide_St == LDPSC_DgrSideRi_C_St) {
    /* Switch: '<S109>/Switch1' incorporates:
     *  Constant: '<S109>/V_Parameter5'
     *  Constant: '<S109>/V_Parameter6'
     *  RelationalOperator: '<S109>/Relational Operator3'
     *  S-Function (sfix_bitop): '<S109>/Bitwise AND1'
     */
    rtb_LDPSC_VelYInvalid_B = ((((uint32_T)LDPVSE_SidCdtnLDPRi_St) & 2U) != 0U);
  } else {
    /* Switch: '<S109>/Switch1' incorporates:
     *  Constant: '<S109>/V_Parameter3'
     */
    rtb_LDPSC_VelYInvalid_B = false;
  }

  /* End of Switch: '<S109>/Switch' */

  /* Switch: '<S108>/Switch' incorporates:
   *  Constant: '<S108>/V_Parameter2'
   *  Constant: '<S108>/V_Parameter32'
   *  RelationalOperator: '<S108>/Relational Operator1'
   *  RelationalOperator: '<S108>/Relational Operator9'
   *  Switch: '<S108>/Switch1'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  if (LDPSC_LstPrevDgrSide_St == LDPSC_DgrSideLf_C_St) {
    /* Logic: '<S108>/NOT' */
    rtb_LDPSC_LnCurveInvalid_B = !rtb_LDPDT_LnCurvVldLf_B;
  } else if (LDPSC_LstPrevDgrSide_St == LDPSC_DgrSideRi_C_St) {
    /* Switch: '<S108>/Switch1' incorporates:
     *  Logic: '<S108>/NOT'
     */
    rtb_LDPSC_LnCurveInvalid_B = !rtb_LDPDT_LnCurvVldRi_B;
  } else {
    /* Logic: '<S108>/NOT' incorporates:
     *  Logic: '<S108>/OR'
     *  Switch: '<S108>/Switch1'
     */
    rtb_LDPSC_LnCurveInvalid_B = ((!rtb_LDPDT_LnCurvVldLf_B) &&
      (!rtb_LDPDT_LnCurvVldRi_B));
  }

  /* End of Switch: '<S108>/Switch' */

  /* RelationalOperator: '<S107>/Relational Operator21' incorporates:
   *  Constant: '<S107>/Constant18'
   *  Constant: '<S107>/V_Parameter23'
   *  Inport: '<Root>/Inport39'
   *  S-Function (sfix_bitop): '<S107>/Bitwise Operator20'
   */
  rtb_LDPSC_DrvStInvalid_B = ((((uint32_T)LDPSAI_IvldStDrv_St) & 100U) != 0U);

  /* RelationalOperator: '<S111>/Relational Operator21' incorporates:
   *  Constant: '<S111>/Constant18'
   *  Constant: '<S111>/V_Parameter23'
   *  Inport: '<Root>/Inport40'
   *  S-Function (sfix_bitop): '<S111>/Bitwise Operator20'
   */
  rtb_AND_ddra = ((((uint32_T)LDPSAI_CtrlStEn_St) & 45U) != 0U);

  /* Switch: '<S117>/Switch' incorporates:
   *  Constant: '<S111>/V_Parameter1'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S117>/Max'
   *  Sum: '<S117>/Subtract'
   *  Switch: '<S117>/Switch1'
   *  UnaryMinus: '<S117>/Unary Minus'
   *  UnitDelay: '<S117>/Unit Delay'
   */
  if (rtb_AND_ddra) {
    LDPSC_SafeFuncActiveTurnOnDelay_sec = fmaxf
      (LDPSC_SafeFuncActiveTurnOnDelay_sec, -LDPSAI_CycleTime_Sec) -
      LDPSAI_CycleTime_Sec;
  } else {
    LDPSC_SafeFuncActiveTurnOnDelay_sec = LDPSC_SafetyFuncMaxTime_sec;
  }

  /* End of Switch: '<S117>/Switch' */

  /* Logic: '<S117>/AND' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S117>/LessThanOrEqual'
   *  UnaryMinus: '<S117>/Unary Minus1'
   *  UnitDelay: '<S117>/Unit Delay'
   */
  rtb_AND_ddra = (rtb_AND_ddra && (LDPSC_SafeFuncActiveTurnOnDelay_sec <=
    (-LDPSAI_CycleTime_Sec)));

  /* RelationalOperator: '<S112>/Relational Operator21' incorporates:
   *  Constant: '<S112>/Constant18'
   *  Constant: '<S112>/V_Parameter23'
   *  Inport: '<Root>/Inport41'
   *  S-Function (sfix_bitop): '<S112>/Bitwise Operator20'
   */
  rtb_AND_ex2t = ((((uint32_T)LDPSAI_StError_St) & 11U) != 0U);

  /* Switch: '<S118>/Switch' incorporates:
   *  Constant: '<S112>/V_Parameter1'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S118>/Max'
   *  Sum: '<S118>/Subtract'
   *  Switch: '<S118>/Switch1'
   *  UnaryMinus: '<S118>/Unary Minus'
   *  UnitDelay: '<S118>/Unit Delay'
   */
  if (rtb_AND_ex2t) {
    LDPSC_SafeFuncErrorTurnOnDelay_sec = fmaxf
      (LDPSC_SafeFuncErrorTurnOnDelay_sec, -LDPSAI_CycleTime_Sec) -
      LDPSAI_CycleTime_Sec;
  } else {
    LDPSC_SafeFuncErrorTurnOnDelay_sec = LDPSC_SafetyFuncMaxTime_sec;
  }

  /* End of Switch: '<S118>/Switch' */

  /* Logic: '<S118>/AND' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S118>/LessThanOrEqual'
   *  UnaryMinus: '<S118>/Unary Minus1'
   *  UnitDelay: '<S118>/Unit Delay'
   */
  rtb_AND_ex2t = (rtb_AND_ex2t && (LDPSC_SafeFuncErrorTurnOnDelay_sec <=
    (-LDPSAI_CycleTime_Sec)));

  /* RelationalOperator: '<S114>/Relational Operator21' incorporates:
   *  Constant: '<S114>/Constant18'
   *  Constant: '<S114>/V_Parameter23'
   *  Inport: '<Root>/Inport38'
   *  S-Function (sfix_bitop): '<S114>/Bitwise Operator20'
   */
  rtb_LDPSC_VehStInvalid_B = ((((uint32_T)LDPSAI_VehStIvld_St) & 255U) != 0U);

  /* Logic: '<S35>/OR' incorporates:
   *  Inport: '<Root>/Inport47'
   *  UnitDelay: '<S119>/Unit Delay'
   */
  LDPSC_Suppression_B = ((((((((rtb_LDPSC_VehicleInvalid_B ||
    (LDPSC_VehYawRateHyst_bool)) || rtb_LDPSC_VelYInvalid_B) ||
    rtb_LDPSC_LnCurveInvalid_B) || rtb_LDPSC_DrvStInvalid_B) || rtb_AND_ddra) ||
    rtb_AND_ex2t) || rtb_LDPSC_VehStInvalid_B) || LDPSAI_AEBActive_B);

  /* RelationalOperator: '<S37>/Relational Operator33' incorporates:
   *  Constant: '<S51>/Constant'
   */
  rtb_LDPDT_LnCurvVldRi_B = (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_ACTIVE);

  /* Logic: '<S37>/AND1' incorporates:
   *  Constant: '<S37>/Constant32'
   *  Constant: '<S52>/Constant'
   *  RelationalOperator: '<S37>/Relational Operator34'
   *  RelationalOperator: '<S37>/Relational Operator36'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  rtb_AND_b1uz = ((((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_RAMPOUT) &&
                  (((int32_T)LDPSC_LstPrevDgrSide_St) == 2));

  /* Switch: '<S55>/Switch' incorporates:
   *  Constant: '<S55>/Constant'
   *  Logic: '<S47>/AND'
   *  Logic: '<S47>/NOT'
   *  Switch: '<S55>/Switch1'
   *  UnitDelay: '<S47>/Unit Delay'
   *  UnitDelay: '<S55>/Unit Delay'
   */
  if (rtb_LDPDT_LnCurvVldRi_B && (!LDPSC_EdgeRisActive_Ri_B)) {
    LDPSC_ActiveStopWatch_Ri_sec = 0.0F;
  } else {
    if (rtb_LDPDT_LnCurvVldRi_B) {
      /* UnitDelay: '<S55>/Unit Delay' incorporates:
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S55>/Add'
       *  Switch: '<S55>/Switch1'
       */
      LDPSC_ActiveStopWatch_Ri_sec += LDPSAI_CycleTime_Sec;
    }
  }

  /* End of Switch: '<S55>/Switch' */

  /* Logic: '<S37>/Logical Operator13' incorporates:
   *  Constant: '<S53>/Constant'
   *  RelationalOperator: '<S37>/Relational Operator35'
   */
  rtb_LDPDT_LnCurvVldLf_B = (rtb_AND_b1uz || (((uint32_T)LDPSC_SysOld_St) ==
    E_LDPState_nu_PASSIVE));

  /* Switch: '<S57>/Switch2' incorporates:
   *  Inport: '<Root>/Inport45'
   *  Logic: '<S45>/AND'
   *  Logic: '<S45>/NOT'
   *  RelationalOperator: '<S57>/GreaterThan'
   *  Switch: '<S57>/Switch'
   *  UnitDelay: '<S45>/Unit Delay'
   *  UnitDelay: '<S57>/Unit Delay'
   */
  if ((!rtb_LDPDT_LnCurvVldLf_B) && LDPSC_EdgeRisFns_Ri_B) {
    /* Switch: '<S37>/Switch1' incorporates:
     *  Constant: '<S37>/Constant34'
     *  Constant: '<S37>/V_Parameter1'
     *  Constant: '<S37>/V_Parameter6'
     *  RelationalOperator: '<S37>/GreaterThan1'
     *  UnitDelay: '<S55>/Unit Delay'
     *  UnitDelay: '<S57>/Unit Delay'
     */
    if (LDPSC_ActiveStopWatch_Ri_sec >= LDPSC_ContinuActiveTi_C_Sec) {
      LDPSC_HdTiFns_Ri_Sec = LDPSC_ContiActiveTiFns_C_Sec;
    } else {
      LDPSC_HdTiFns_Ri_Sec = 4.0F;
    }

    /* End of Switch: '<S37>/Switch1' */
  } else if (LDPSC_HdTiFns_Ri_Sec > LDPSAI_CycleTime_Sec) {
    /* Switch: '<S57>/Switch' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S57>/Subtract'
     *  UnitDelay: '<S57>/Unit Delay'
     */
    LDPSC_HdTiFns_Ri_Sec -= LDPSAI_CycleTime_Sec;
  } else {
    /* UnitDelay: '<S57>/Unit Delay' incorporates:
     *  Constant: '<S57>/Constant1'
     *  Switch: '<S57>/Switch'
     */
    LDPSC_HdTiFns_Ri_Sec = 0.0F;
  }

  /* End of Switch: '<S57>/Switch2' */

  /* Switch: '<S25>/Switch' incorporates:
   *  UnitDelay: '<S25>/Unit Delay'
   */
  if (LDPSC_PrevSwitchUnitDelay_bool) {
    /* Switch: '<S25>/Switch' incorporates:
     *  Inport: '<Root>/Inport9'
     */
    LDPSC_NVRAMLDPSwitch_B = LDPSAI_LDPSwitchEn_B;
  } else {
    /* Switch: '<S25>/Switch' incorporates:
     *  Constant: '<S25>/V_Parameter11'
     *  Inport: '<Root>/Inport48'
     *  Switch: '<S25>/Switch1'
     */
    LDPSC_NVRAMLDPSwitch_B = (NVRAM_LDPSwitch_B || (LDPSC_Switch_C_B));
  }

  /* End of Switch: '<S25>/Switch' */

  /* Chart: '<S4>/LDP_State' incorporates:
   *  Constant: '<S57>/Constant2'
   *  Inport: '<Root>/Inport11'
   *  Logic: '<S37>/AND2'
   *  Logic: '<S37>/AND3'
   *  Logic: '<S37>/Logical Operator11'
   *  Logic: '<S37>/Logical Operator12'
   *  Logic: '<S37>/Logical Operator14'
   *  RelationalOperator: '<S57>/GreaterThan1'
   *  UnitDelay: '<S57>/Unit Delay'
   */
  if (((uint32_T)LDPSA_DW.is_active_c2_LDPSA) == 0U) {
    LDPSA_DW.is_active_c2_LDPSA = 1U;
    if (LDPSAI_LDPErrCdtn_B) {
      LDPSA_DW.is_c2_LDPSA = LDPSA_IN_LDP_ERROR;

      /* RelationalOperator: '<S80>/Relational Operator5' */
      LDPSC_SysOld_St = E_LDPState_nu_ERROR;
    } else {
      /* [!Error] */
      LDPSA_DW.is_c2_LDPSA = LDPSA_IN_LDP_OFF;

      /* RelationalOperator: '<S80>/Relational Operator5' */
      LDPSC_SysOld_St = E_LDPState_nu_OFF;
    }
  } else {
    switch (LDPSA_DW.is_c2_LDPSA) {
     case LDPSA_IN_LDP_ERROR:
      /* RelationalOperator: '<S80>/Relational Operator5' */
      LDPSC_SysOld_St = E_LDPState_nu_ERROR;
      if (!LDPSAI_LDPErrCdtn_B) {
        LDPSA_DW.is_c2_LDPSA = LDPSA_IN_LDP_OFF;

        /* RelationalOperator: '<S80>/Relational Operator5' */
        LDPSC_SysOld_St = E_LDPState_nu_OFF;
      } else {
        if ((!LDPSAI_LDPErrCdtn_B) && LDPSC_NVRAMLDPSwitch_B) {
          LDPSA_DW.is_c2_LDPSA = LDPSA_IN_LDP_ON;
          LDPSA_DW.is_LDP_ON = LDPSA_IN_LDP_PASSIVE;

          /* RelationalOperator: '<S80>/Relational Operator5' */
          LDPSC_SysOld_St = E_LDPState_nu_PASSIVE;
        }
      }
      break;

     case LDPSA_IN_LDP_OFF:
      /* RelationalOperator: '<S80>/Relational Operator5' */
      LDPSC_SysOld_St = E_LDPState_nu_OFF;
      if (LDPSAI_LDPErrCdtn_B) {
        LDPSA_DW.is_c2_LDPSA = LDPSA_IN_LDP_ERROR;

        /* RelationalOperator: '<S80>/Relational Operator5' */
        LDPSC_SysOld_St = E_LDPState_nu_ERROR;
      } else {
        if ((!LDPSAI_LDPErrCdtn_B) && LDPSC_NVRAMLDPSwitch_B) {
          LDPSA_DW.is_c2_LDPSA = LDPSA_IN_LDP_ON;
          LDPSA_DW.is_LDP_ON = LDPSA_IN_LDP_PASSIVE;

          /* RelationalOperator: '<S80>/Relational Operator5' */
          LDPSC_SysOld_St = E_LDPState_nu_PASSIVE;
        }
      }
      break;

     default:
      /* case IN_LDP_ON: */
      if (LDPSAI_LDPErrCdtn_B) {
        LDPSA_DW.is_LDP_ON = LDPSA_IN_NO_ACTIVE_CHILD;
        LDPSA_DW.is_c2_LDPSA = LDPSA_IN_LDP_ERROR;

        /* RelationalOperator: '<S80>/Relational Operator5' */
        LDPSC_SysOld_St = E_LDPState_nu_ERROR;
      } else if (!LDPSC_NVRAMLDPSwitch_B) {
        LDPSA_DW.is_LDP_ON = LDPSA_IN_NO_ACTIVE_CHILD;
        LDPSA_DW.is_c2_LDPSA = LDPSA_IN_LDP_OFF;

        /* RelationalOperator: '<S80>/Relational Operator5' */
        LDPSC_SysOld_St = E_LDPState_nu_OFF;
      } else {
        switch (LDPSA_DW.is_LDP_ON) {
         case LDPSA_IN_LDP_ACTIVE:
          /* RelationalOperator: '<S80>/Relational Operator5' */
          LDPSC_SysOld_St = E_LDPState_nu_ACTIVE;
          if (LDPSC_Suppression_B) {
            LDPSA_DW.is_LDP_ON = LDPSA_IN_LDP_PASSIVE;

            /* RelationalOperator: '<S80>/Relational Operator5' */
            LDPSC_SysOld_St = E_LDPState_nu_PASSIVE;
          } else {
            if ((rtb_NOT_fftx || LDPSC_DgrFns_B) || LDPSC_Cancel_B) {
              LDPSA_DW.is_LDP_ON = LDPSA_IN_LDP_STANDBY;

              /* RelationalOperator: '<S80>/Relational Operator5' */
              LDPSC_SysOld_St = E_LDPState_nu_STANDBY;
            }
          }
          break;

         case LDPSA_IN_LDP_PASSIVE:
          /* RelationalOperator: '<S80>/Relational Operator5' */
          LDPSC_SysOld_St = E_LDPState_nu_PASSIVE;
          if (!LDPSC_Suppression_B) {
            LDPSA_DW.is_LDP_ON = LDPSA_IN_LDP_STANDBY;

            /* RelationalOperator: '<S80>/Relational Operator5' */
            LDPSC_SysOld_St = E_LDPState_nu_STANDBY;
          }
          break;

         default:
          /* RelationalOperator: '<S80>/Relational Operator5' */
          /* case IN_LDP_STANDBY: */
          LDPSC_SysOld_St = E_LDPState_nu_STANDBY;
          if (LDPSC_Suppression_B) {
            LDPSA_DW.is_LDP_ON = LDPSA_IN_LDP_PASSIVE;

            /* RelationalOperator: '<S80>/Relational Operator5' */
            LDPSC_SysOld_St = E_LDPState_nu_PASSIVE;
          } else {
            if (((LDPSC_TrigLf_B && LDPSC_StrgRdy_B) && (LDPSC_WkRdy_B &&
                  LDPSC_BlockTime_B)) || ((LDPSC_TrigRi_B && LDPSC_StrgRdy_B) &&
                 (LDPSC_WkRdy_B && (((!rtb_LDPDT_LnCurvVldRi_B) &&
                    (!rtb_AND_b1uz)) && (LDPSC_HdTiFns_Ri_Sec <= 0.0F))))) {
              LDPSA_DW.is_LDP_ON = LDPSA_IN_LDP_ACTIVE;

              /* RelationalOperator: '<S80>/Relational Operator5' */
              LDPSC_SysOld_St = E_LDPState_nu_ACTIVE;
            }
          }
          break;
        }
      }
      break;
    }
  }

  /* RelationalOperator: '<S23>/Equal' incorporates:
   *  Constant: '<S27>/Constant'
   */
  rtb_AND_b1uz = (((uint32_T)LDPSC_SysOld_St) != E_LDPState_nu_STANDBY);

  /* Logic: '<S23>/NOT' */
  rtb_NOT_fftx = !rtb_AND_b1uz;

  /* Switch: '<S30>/Switch2' incorporates:
   *  Inport: '<Root>/Inport45'
   *  Logic: '<S28>/AND'
   *  Logic: '<S28>/NOT'
   *  RelationalOperator: '<S30>/GreaterThan'
   *  Switch: '<S30>/Switch'
   *  UnitDelay: '<S28>/Unit Delay'
   *  UnitDelay: '<S30>/Unit Delay'
   */
  if (rtb_NOT_fftx && (!LDPSC_PrevStandbyUnitDelay_bool)) {
    LDPSC_SusTimeExpiredTimerRetrigger_sec = LDPSC_RampoutTime_Sec;
  } else if (LDPSC_SusTimeExpiredTimerRetrigger_sec > LDPSAI_CycleTime_Sec) {
    /* Switch: '<S30>/Switch' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S30>/Subtract'
     *  UnitDelay: '<S30>/Unit Delay'
     */
    LDPSC_SusTimeExpiredTimerRetrigger_sec =
      LDPSC_SusTimeExpiredTimerRetrigger_sec - LDPSAI_CycleTime_Sec;
  } else {
    /* UnitDelay: '<S30>/Unit Delay' incorporates:
     *  Constant: '<S30>/Constant1'
     *  Switch: '<S30>/Switch'
     */
    LDPSC_SusTimeExpiredTimerRetrigger_sec = 0.0F;
  }

  /* End of Switch: '<S30>/Switch2' */

  /* Switch: '<S29>/Switch' incorporates:
   *  Constant: '<S29>/Constant2'
   *  Constant: '<S30>/Constant2'
   *  Logic: '<S28>/NOT1'
   *  Logic: '<S28>/NOT2'
   *  RelationalOperator: '<S30>/GreaterThan1'
   *  UnitDelay: '<S29>/Unit Delay'
   *  UnitDelay: '<S30>/Unit Delay'
   */
  if (!rtb_NOT_fftx) {
    LDPSC_RampTimeExpiredRSFF_bool = false;
  } else {
    LDPSC_RampTimeExpiredRSFF_bool = ((LDPSC_SusTimeExpiredTimerRetrigger_sec <=
      0.0F) || (LDPSC_RampTimeExpiredRSFF_bool));
  }

  /* End of Switch: '<S29>/Switch' */

  /* Switch: '<S23>/Switch1' incorporates:
   *  Logic: '<S23>/OR'
   *  UnitDelay: '<S29>/Unit Delay'
   */
  if (rtb_AND_b1uz || (LDPSC_RampTimeExpiredRSFF_bool)) {
    /* Switch: '<S23>/Switch1' */
    LDPSC_SysOut_St = LDPSC_SysOld_St;
  } else {
    /* Switch: '<S23>/Switch1' incorporates:
     *  Constant: '<S26>/Constant'
     */
    LDPSC_SysOut_St = E_LDPState_nu_RAMPOUT;
  }

  /* End of Switch: '<S23>/Switch1' */

  /* RelationalOperator: '<S80>/Relational Operator5' incorporates:
   *  RelationalOperator: '<S120>/Equal1'
   *  RelationalOperator: '<S120>/Equal3'
   *  RelationalOperator: '<S120>/Equal4'
   *  RelationalOperator: '<S121>/Equal5'
   *  RelationalOperator: '<S148>/Equal2'
   *  RelationalOperator: '<S148>/Equal6'
   *  RelationalOperator: '<S149>/Equal1'
   *  RelationalOperator: '<S149>/Equal4'
   *  RelationalOperator: '<S150>/Equal4'
   *  RelationalOperator: '<S150>/Equal6'
   *  Switch: '<S149>/Switch'
   *  Switch: '<S23>/Switch1'
   */
  LDPSC_SysOld_St = LDPSC_SysOut_St;

  /* RelationalOperator: '<S120>/Equal4' incorporates:
   *  Constant: '<S124>/Constant'
   */
  rtb_Equal4 = (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_ACTIVE);

  /* Logic: '<S122>/AND' incorporates:
   *  Logic: '<S122>/NOT'
   *  UnitDelay: '<S122>/Unit Delay'
   */
  rtb_AND_b1uz = (rtb_Equal4 && (!LDPTT_CtrlIniEn_B));

  /* RelationalOperator: '<S120>/Equal1' incorporates:
   *  Constant: '<S123>/Constant'
   */
  rtb_Equal1_a4qg = (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_ACTIVE);

  /* Switch: '<S126>/Switch12' incorporates:
   *  Constant: '<S126>/V_Const'
   *  Inport: '<Root>/Inport14'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPSAI_PstnXLf_Mi;
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S126>/Switch12' */

  /* Switch: '<S129>/Switch1' incorporates:
   *  Constant: '<S129>/Constant1'
   *  Product: '<S129>/Product'
   *  Product: '<S129>/Product1'
   *  Sum: '<S129>/Add'
   *  Sum: '<S129>/Subtract'
   *  UnitDelay: '<S129>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryPstnXLf_Mi = rtb_Abs;
  } else {
    /* Product: '<S129>/Divide' incorporates:
     *  Constant: '<S126>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryPstnXLf_Mi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryPstnXLf_Mi);
  }

  /* End of Switch: '<S129>/Switch1' */

  /* RelationalOperator: '<S120>/Equal3' incorporates:
   *  Constant: '<S125>/Constant'
   */
  rtb_Equal3_newf = (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_RAMPOUT);

  /* Switch: '<S126>/Switch' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S126>/Switch1' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S126>/UnitDelay'
     *  UnitDelay: '<S129>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryPstnXLf_Mi = LDPTT_LwLnBdryPstnXLf_Mi;
    } else {
      LDPTT_LstLnBdryPstnXLf_Mi = rtb_Abs;
    }

    /* End of Switch: '<S126>/Switch1' */
  }

  /* End of Switch: '<S126>/Switch' */

  /* Switch: '<S126>/Switch17' incorporates:
   *  Constant: '<S126>/V_Const5'
   *  Inport: '<Root>/Inport32'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPSAI_VldLengLf_Mi;
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S126>/Switch17' */

  /* Switch: '<S134>/Switch1' incorporates:
   *  Constant: '<S134>/Constant1'
   *  Product: '<S134>/Product'
   *  Product: '<S134>/Product1'
   *  Sum: '<S134>/Add'
   *  Sum: '<S134>/Subtract'
   *  UnitDelay: '<S134>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryVldLengLf_Mi = rtb_Abs;
  } else {
    /* Product: '<S134>/Divide' incorporates:
     *  Constant: '<S126>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryVldLengLf_Mi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryVldLengLf_Mi);
  }

  /* End of Switch: '<S134>/Switch1' */

  /* Switch: '<S126>/Switch10' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S126>/Switch11' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S126>/UnitDelay5'
     *  UnitDelay: '<S134>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryVldLengLf_Mi = LDPTT_LwLnBdryVldLengLf_Mi;
    } else {
      LDPTT_LstLnBdryVldLengLf_Mi = rtb_Abs;
    }

    /* End of Switch: '<S126>/Switch11' */
  }

  /* End of Switch: '<S126>/Switch10' */

  /* RelationalOperator: '<S121>/Equal6' incorporates:
   *  Constant: '<S121>/V_Parameter'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  rtb_OR1_m0ph = (LDPTT_DgrSideLf_C_St == LDPSC_LstPrevDgrSide_St);

  /* Switch: '<S121>/Switch18' incorporates:
   *  Inport: '<Root>/Inport13'
   *  UnitDelay: '<S121>/UnitDelay'
   *  UnitDelay: '<S121>/UnitDelay5'
   */
  if (!LDPTT_LstControl_B) {
    LDPTT_LstLnWidCalc_Mi = LDPSAI_LnWidCalc_Mi;
  }

  /* End of Switch: '<S121>/Switch18' */

  /* Switch: '<S121>/Switch' */
  if (rtb_OR1_m0ph) {
    /* Switch: '<S121>/Switch' incorporates:
     *  Inport: '<Root>/Inport16'
     */
    LDPTT_RawLnBdryPstnYLf_Mi = LDPSAI_PstnYLf_Mi;
  } else {
    /* Switch: '<S121>/Switch' incorporates:
     *  Inport: '<Root>/Inport18'
     *  Sum: '<S121>/Subtract12'
     *  UnitDelay: '<S121>/UnitDelay'
     */
    LDPTT_RawLnBdryPstnYLf_Mi = LDPTT_LstLnWidCalc_Mi + LDPSAI_PstnYRi_Mi;
  }

  /* End of Switch: '<S121>/Switch' */

  /* Switch: '<S126>/Switch13' incorporates:
   *  Constant: '<S126>/V_Const1'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPTT_RawLnBdryPstnYLf_Mi;
  } else {
    rtb_Abs = 10.0F;
  }

  /* End of Switch: '<S126>/Switch13' */

  /* Switch: '<S130>/Switch1' incorporates:
   *  Constant: '<S130>/Constant1'
   *  Product: '<S130>/Product'
   *  Product: '<S130>/Product1'
   *  Sum: '<S130>/Add'
   *  Sum: '<S130>/Subtract'
   *  UnitDelay: '<S130>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryPstnYLf_Mi = rtb_Abs;
  } else {
    /* Product: '<S130>/Divide' incorporates:
     *  Constant: '<S126>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryPstnYLf_Mi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryPstnYLf_Mi);
  }

  /* End of Switch: '<S130>/Switch1' */

  /* Switch: '<S126>/Switch2' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S126>/Switch3' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S126>/UnitDelay1'
     *  UnitDelay: '<S130>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryPstnYLf_Mi = LDPTT_LwLnBdryPstnYLf_Mi;
    } else {
      LDPTT_LstLnBdryPstnYLf_Mi = rtb_Abs;
    }

    /* End of Switch: '<S126>/Switch3' */
  }

  /* End of Switch: '<S126>/Switch2' */

  /* Switch: '<S121>/Switch1' */
  if (rtb_OR1_m0ph) {
    /* Switch: '<S121>/Switch1' incorporates:
     *  Inport: '<Root>/Inport16'
     *  Sum: '<S121>/Subtract13'
     *  UnitDelay: '<S121>/UnitDelay'
     */
    LDPTT_RawLnBdryPstnYRi_Mi = LDPSAI_PstnYLf_Mi - LDPTT_LstLnWidCalc_Mi;
  } else {
    /* Switch: '<S121>/Switch1' incorporates:
     *  Inport: '<Root>/Inport18'
     */
    LDPTT_RawLnBdryPstnYRi_Mi = LDPSAI_PstnYRi_Mi;
  }

  /* End of Switch: '<S121>/Switch1' */

  /* Switch: '<S127>/Switch13' incorporates:
   *  Constant: '<S127>/V_Const1'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPTT_RawLnBdryPstnYRi_Mi;
  } else {
    rtb_Abs = -10.0F;
  }

  /* End of Switch: '<S127>/Switch13' */

  /* Switch: '<S136>/Switch1' incorporates:
   *  Constant: '<S136>/Constant1'
   *  Product: '<S136>/Product'
   *  Product: '<S136>/Product1'
   *  Sum: '<S136>/Add'
   *  Sum: '<S136>/Subtract'
   *  UnitDelay: '<S136>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryPstnYRi_Mi = rtb_Abs;
  } else {
    /* Product: '<S136>/Divide' incorporates:
     *  Constant: '<S127>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryPstnYRi_Mi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryPstnYRi_Mi);
  }

  /* End of Switch: '<S136>/Switch1' */

  /* Switch: '<S127>/Switch2' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S127>/Switch3' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S127>/UnitDelay1'
     *  UnitDelay: '<S136>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryPstnYRi_Mi = LDPTT_LwLnBdryPstnYRi_Mi;
    } else {
      LDPTT_LstLnBdryPstnYRi_Mi = rtb_Abs;
    }

    /* End of Switch: '<S127>/Switch3' */
  }

  /* End of Switch: '<S127>/Switch2' */

  /* Switch: '<S127>/Switch12' incorporates:
   *  Constant: '<S127>/V_Const'
   *  Inport: '<Root>/Inport15'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPSAI_PstnXRi_Mi;
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S127>/Switch12' */

  /* Switch: '<S135>/Switch1' incorporates:
   *  Constant: '<S135>/Constant1'
   *  Product: '<S135>/Product'
   *  Product: '<S135>/Product1'
   *  Sum: '<S135>/Add'
   *  Sum: '<S135>/Subtract'
   *  UnitDelay: '<S135>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryPstnXRi_Mi = rtb_Abs;
  } else {
    /* Product: '<S135>/Divide' incorporates:
     *  Constant: '<S127>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryPstnXRi_Mi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryPstnXRi_Mi);
  }

  /* End of Switch: '<S135>/Switch1' */

  /* Switch: '<S127>/Switch' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S127>/Switch1' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S127>/UnitDelay'
     *  UnitDelay: '<S135>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryPstnXRi_Mi = LDPTT_LwLnBdryPstnXRi_Mi;
    } else {
      LDPTT_LstLnBdryPstnXRi_Mi = rtb_Abs;
    }

    /* End of Switch: '<S127>/Switch1' */
  }

  /* End of Switch: '<S127>/Switch' */

  /* Switch: '<S127>/Switch14' incorporates:
   *  Constant: '<S127>/V_Const2'
   *  Inport: '<Root>/Inport22'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPSAI_HeadAglRi_Rad;
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S127>/Switch14' */

  /* Switch: '<S137>/Switch1' incorporates:
   *  Constant: '<S137>/Constant1'
   *  Product: '<S137>/Product'
   *  Product: '<S137>/Product1'
   *  Sum: '<S137>/Add'
   *  Sum: '<S137>/Subtract'
   *  UnitDelay: '<S137>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryHeadAglRi_Rad = rtb_Abs;
  } else {
    /* Product: '<S137>/Divide' incorporates:
     *  Constant: '<S127>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryHeadAglRi_Rad = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryHeadAglRi_Rad);
  }

  /* End of Switch: '<S137>/Switch1' */

  /* Switch: '<S127>/Switch4' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S127>/Switch5' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S127>/UnitDelay2'
     *  UnitDelay: '<S137>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryHeadAglRi_Rad = LDPTT_LwLnBdryHeadAglRi_Rad;
    } else {
      LDPTT_LstLnBdryHeadAglRi_Rad = rtb_Abs;
    }

    /* End of Switch: '<S127>/Switch5' */
  }

  /* End of Switch: '<S127>/Switch4' */

  /* Switch: '<S127>/Switch15' incorporates:
   *  Constant: '<S127>/V_Const3'
   *  Inport: '<Root>/Inport26'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPSAI_CurvRi_ReMi;
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S127>/Switch15' */

  /* Switch: '<S138>/Switch1' incorporates:
   *  Constant: '<S138>/Constant1'
   *  Product: '<S138>/Product'
   *  Product: '<S138>/Product1'
   *  Sum: '<S138>/Add'
   *  Sum: '<S138>/Subtract'
   *  UnitDelay: '<S138>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryCurvRi_ReMi = rtb_Abs;
  } else {
    /* Product: '<S138>/Divide' incorporates:
     *  Constant: '<S127>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryCurvRi_ReMi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryCurvRi_ReMi);
  }

  /* End of Switch: '<S138>/Switch1' */

  /* Switch: '<S127>/Switch6' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S127>/Switch7' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S127>/UnitDelay3'
     *  UnitDelay: '<S138>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryCurvRi_ReMi = LDPTT_LwLnBdryCurvRi_ReMi;
    } else {
      LDPTT_LstLnBdryCurvRi_ReMi = rtb_Abs;
    }

    /* End of Switch: '<S127>/Switch7' */
  }

  /* End of Switch: '<S127>/Switch6' */

  /* Switch: '<S127>/Switch16' incorporates:
   *  Constant: '<S127>/V_Const4'
   *  Inport: '<Root>/Inport29'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPSAI_CurvRateRi_ReMi2;
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S127>/Switch16' */

  /* Switch: '<S139>/Switch1' incorporates:
   *  Constant: '<S139>/Constant1'
   *  Product: '<S139>/Product'
   *  Product: '<S139>/Product1'
   *  Sum: '<S139>/Add'
   *  Sum: '<S139>/Subtract'
   *  UnitDelay: '<S139>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryCurvRateRi_ReMi2 = rtb_Abs;
  } else {
    /* Product: '<S139>/Divide' incorporates:
     *  Constant: '<S127>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryCurvRateRi_ReMi2 = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryCurvRateRi_ReMi2);
  }

  /* End of Switch: '<S139>/Switch1' */

  /* Switch: '<S127>/Switch8' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S127>/Switch9' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S127>/UnitDelay4'
     *  UnitDelay: '<S139>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryCurvRateRi_ReMi2 = LDPTT_LwLnBdryCurvRateRi_ReMi2;
    } else {
      LDPTT_LstLnBdryCurvRateRi_ReMi2 = rtb_Abs;
    }

    /* End of Switch: '<S127>/Switch9' */
  }

  /* End of Switch: '<S127>/Switch8' */

  /* Switch: '<S127>/Switch17' incorporates:
   *  Constant: '<S127>/V_Const5'
   *  Inport: '<Root>/Inport33'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPSAI_VldLengRi_Mi;
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S127>/Switch17' */

  /* Switch: '<S140>/Switch1' incorporates:
   *  Constant: '<S140>/Constant1'
   *  Product: '<S140>/Product'
   *  Product: '<S140>/Product1'
   *  Sum: '<S140>/Add'
   *  Sum: '<S140>/Subtract'
   *  UnitDelay: '<S140>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryVldLengRi_Mi = rtb_Abs;
  } else {
    /* Product: '<S140>/Divide' incorporates:
     *  Constant: '<S127>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryVldLengRi_Mi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryVldLengRi_Mi);
  }

  /* End of Switch: '<S140>/Switch1' */

  /* Switch: '<S127>/Switch10' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S127>/Switch11' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S127>/UnitDelay5'
     *  UnitDelay: '<S140>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryVldLengRi_Mi = LDPTT_LwLnBdryVldLengRi_Mi;
    } else {
      LDPTT_LstLnBdryVldLengRi_Mi = rtb_Abs;
    }

    /* End of Switch: '<S127>/Switch11' */
  }

  /* End of Switch: '<S127>/Switch10' */

  /* Switch: '<S121>/Switch9' incorporates:
   *  UnitDelay: '<S121>/UnitDelay4'
   */
  LDPTT_TgtLatDstcRi_Mi = LDPTT_LstTgtLatDstcRi_Mi;

  /* Abs: '<S121>/Abs2' incorporates:
   *  Inport: '<Root>/Inport26'
   */
  rtb_Abs = fabsf(LDPSAI_CurvRi_ReMi);

  /* Switch: '<S121>/Switch9' incorporates:
   *  UnitDelay: '<S121>/UnitDelay5'
   */
  if (!LDPTT_LstControl_B) {
    /* Switch: '<S121>/Switch10' incorporates:
     *  Abs: '<S121>/Abs2'
     *  Constant: '<S121>/V_Const4'
     *  Constant: '<S121>/V_Parameter12'
     *  Constant: '<S121>/V_Parameter13'
     *  Lookup_n-D: '<S121>/Lookup Table3'
     *  RelationalOperator: '<S121>/Equal7'
     *  RelationalOperator: '<S121>/Equal8'
     *  Switch: '<S121>/Switch11'
     *  UnaryMinus: '<S121>/Unary Minus3'
     */
    if (LDPTT_CurvInner_C_St == LDPDT_CurveTypeRi_St) {
      rtb_Abs = -look1_iflf_binlxpw(rtb_Abs, ((const real32_T *)
        &(LDPTT_LnBdryCurvLf_BX_ReMi[0])), ((const real32_T *)
        &(LDPTT_TgtOfstLfIn_Cr_Mi[0])), 5U);
    } else if (LDPDT_CurveTypeRi_St == LDPTT_CurvOuter_C_St) {
      /* Switch: '<S121>/Switch11' incorporates:
       *  Abs: '<S121>/Abs2'
       *  Lookup_n-D: '<S121>/Lookup Table5'
       */
      rtb_Abs = look1_iflf_binlxpw(rtb_Abs, ((const real32_T *)
        &(LDPTT_LnBdryCurvRi_BX_ReMi[0])), ((const real32_T *)
        &(LDPTT_TgtOfstRiOut_Cr_Mi[0])), 5U);
    } else {
      rtb_Abs = 0.0F;
    }

    /* End of Switch: '<S121>/Switch10' */

    /* Switch: '<S121>/Switch9' incorporates:
     *  Abs: '<S121>/Abs3'
     *  Constant: '<S121>/V_Const5'
     *  Constant: '<S121>/V_Parameter14'
     *  Inport: '<Root>/Inport'
     *  Inport: '<Root>/Inport13'
     *  Lookup_n-D: '<S121>/Lookup Table4'
     *  MinMax: '<S121>/Min1'
     *  Product: '<S121>/Divide1'
     *  Sum: '<S121>/Add1'
     *  Sum: '<S121>/Subtract9'
     */
    LDPTT_TgtLatDstcRi_Mi = (LDPSAI_VehWid_Mi / 2.0F) + fminf
      (LDPTT_MxTgtLatDstc_C_Mi, rtb_Abs + look1_iflf_binlxpw(fabsf
        (LDPSAI_LnWidCalc_Mi), ((const real32_T *)&(LDPTT_LnWidCalc_BX_Mi[0])),
        ((const real32_T *)&(LDPTT_TgtLatDistcRi_Cr_Mi[0])), 5U));
  }

  /* End of Switch: '<S121>/Switch9' */

  /* Switch: '<S160>/Switch1' incorporates:
   *  Constant: '<S160>/V_Parameter20'
   *  Constant: '<S160>/V_Parameter21'
   *  Sum: '<S160>/Add'
   *  UnitDelay: '<S165>/Unit Delay'
   */
  if (LDPVSE_BHysLnWid_B) {
    rtb_Abs = LDPVSE_LnWidTrsdMx_C_Mi + LDPVSE_LnWidTrsdOfst_C_Mi;
  } else {
    rtb_Abs = LDPVSE_LnWidTrsdMx_C_Mi;
  }

  /* End of Switch: '<S160>/Switch1' */

  /* RelationalOperator: '<S160>/Less Than' incorporates:
   *  Inport: '<Root>/Inport13'
   */
  LDPVSE_TgtCntrByLnWidth_B = (LDPSAI_LnWidCalc_Mi < rtb_Abs);

  /* Logic: '<S160>/Logical Operator11' incorporates:
   *  Logic: '<S160>/NOT'
   */
  LDPVSE_TgtCntrLnEn_B = (rtb_LogicalOperator_kelu && LDPVSE_TgtCntrByLnWidth_B);

  /* Switch: '<S121>/Switch15' incorporates:
   *  Constant: '<S121>/V_Parameter16'
   *  Inport: '<Root>/Inport18'
   *  Inport: '<Root>/Inport30'
   *  Logic: '<S121>/AND1'
   *  Sum: '<S121>/Subtract1'
   */
  if ((LDPTT_TgtCntrLnEn_C_B) && LDPVSE_TgtCntrLnEn_B) {
    rtb_Abs = LDPSAI_PstnYCent_Mi;
  } else {
    rtb_Abs = LDPSAI_PstnYRi_Mi + LDPTT_TgtLatDstcRi_Mi;
  }

  /* End of Switch: '<S121>/Switch15' */

  /* Switch: '<S121>/Switch13' incorporates:
   *  Inport: '<Root>/Inport18'
   *  Sum: '<S121>/Subtract10'
   *  Sum: '<S121>/Subtract11'
   *  Switch: '<S121>/Switch12'
   *  UnitDelay: '<S121>/UnitDelay5'
   *  UnitDelay: '<S121>/UnitDelay6'
   *  UnitDelay: '<S121>/UnitDelay7'
   */
  if (!LDPTT_LstControl_B) {
    /* Switch: '<S121>/Switch14' incorporates:
     *  Constant: '<S121>/V_Parameter11'
     *  RelationalOperator: '<S121>/Great Than1'
     */
    if (rtb_Abs >= LDPTT_MxTgtLatDev_C_Mi) {
      /* UnitDelay: '<S121>/UnitDelay6' incorporates:
       *  Constant: '<S121>/V_Parameter17'
       *  Sum: '<S121>/Subtract4'
       */
      LDPTT_LstMxTgtLatDevRi_Mi = rtb_Abs - LDPTT_MxTgtLatDev_C_Mi;
    } else {
      /* UnitDelay: '<S121>/UnitDelay6' incorporates:
       *  Constant: '<S121>/V_Const6'
       */
      LDPTT_LstMxTgtLatDevRi_Mi = 0.0F;
    }

    /* End of Switch: '<S121>/Switch14' */
    LDPTT_LstTgtLatDevRi_Mi = (rtb_Abs - LDPTT_LstMxTgtLatDevRi_Mi) -
      LDPSAI_PstnYRi_Mi;
  }

  /* End of Switch: '<S121>/Switch13' */

  /* Sum: '<S121>/Subtract5' incorporates:
   *  Inport: '<Root>/Inport18'
   *  UnitDelay: '<S121>/UnitDelay7'
   */
  LDPTT_TgtPstnYRi_Mi = LDPSAI_PstnYRi_Mi + LDPTT_LstTgtLatDevRi_Mi;

  /* Switch: '<S121>/Switch2' incorporates:
   *  UnitDelay: '<S121>/UnitDelay1'
   */
  LDPTT_TgtLatDstcLf_Mi = LDPTT_LstTgtLatDstcLf_Mi;

  /* Abs: '<S121>/Abs' incorporates:
   *  Inport: '<Root>/Inport24'
   */
  rtb_Abs = fabsf(LDPSAI_CurvLf_ReMi);

  /* Switch: '<S121>/Switch2' incorporates:
   *  UnitDelay: '<S121>/UnitDelay5'
   */
  if (!LDPTT_LstControl_B) {
    /* Switch: '<S121>/Switch3' incorporates:
     *  Abs: '<S121>/Abs'
     *  Constant: '<S121>/V_Const1'
     *  Constant: '<S121>/V_Parameter2'
     *  Constant: '<S121>/V_Parameter3'
     *  Lookup_n-D: '<S121>/Lookup Table'
     *  RelationalOperator: '<S121>/Equal2'
     *  RelationalOperator: '<S121>/Equal3'
     *  Switch: '<S121>/Switch4'
     *  UnaryMinus: '<S121>/Unary Minus'
     */
    if (LDPTT_CurvInner_C_St == LDPDT_CurveTypeLe_St) {
      rtb_Abs = -look1_iflf_binlxpw(rtb_Abs, ((const real32_T *)
        &(LDPTT_LnBdryCurvLf_BX_ReMi[0])), ((const real32_T *)
        &(LDPTT_TgtOfstLfIn_Cr_Mi[0])), 5U);
    } else if (LDPDT_CurveTypeLe_St == LDPTT_CurvOuter_C_St) {
      /* Switch: '<S121>/Switch4' incorporates:
       *  Abs: '<S121>/Abs'
       *  Lookup_n-D: '<S121>/Lookup Table2'
       */
      rtb_Abs = look1_iflf_binlxpw(rtb_Abs, ((const real32_T *)
        &(LDPTT_LnBdryCurvLf_BX_ReMi[0])), ((const real32_T *)
        &(LDPTT_TgtOfstLfOut_Cr_Mi[0])), 5U);
    } else {
      rtb_Abs = 0.0F;
    }

    /* End of Switch: '<S121>/Switch3' */

    /* Switch: '<S121>/Switch2' incorporates:
     *  Abs: '<S121>/Abs1'
     *  Constant: '<S121>/V_Const2'
     *  Constant: '<S121>/V_Parameter4'
     *  Inport: '<Root>/Inport'
     *  Inport: '<Root>/Inport13'
     *  Lookup_n-D: '<S121>/Lookup Table1'
     *  MinMax: '<S121>/Min'
     *  Product: '<S121>/Divide'
     *  Sum: '<S121>/Add10'
     *  Sum: '<S121>/Subtract6'
     */
    LDPTT_TgtLatDstcLf_Mi = (LDPSAI_VehWid_Mi / 2.0F) + fminf
      (LDPTT_MxTgtLatDstc_C_Mi, rtb_Abs + look1_iflf_binlxpw(fabsf
        (LDPSAI_LnWidCalc_Mi), ((const real32_T *)&(LDPTT_LnWidCalc_BX_Mi[0])),
        ((const real32_T *)&(LDPTT_TgtLatDistcLf_Cr_Mi[0])), 5U));
  }

  /* End of Switch: '<S121>/Switch2' */

  /* Switch: '<S121>/Switch8' incorporates:
   *  Constant: '<S121>/V_Parameter6'
   *  Inport: '<Root>/Inport16'
   *  Inport: '<Root>/Inport30'
   *  Logic: '<S121>/AND'
   *  Sum: '<S121>/Subtract'
   */
  if ((LDPTT_TgtCntrLnEn_C_B) && LDPVSE_TgtCntrLnEn_B) {
    rtb_Abs = LDPSAI_PstnYCent_Mi;
  } else {
    rtb_Abs = LDPSAI_PstnYLf_Mi - LDPTT_TgtLatDstcLf_Mi;
  }

  /* End of Switch: '<S121>/Switch8' */

  /* Switch: '<S121>/Switch6' incorporates:
   *  Inport: '<Root>/Inport16'
   *  Sum: '<S121>/Subtract7'
   *  Sum: '<S121>/Subtract8'
   *  Switch: '<S121>/Switch5'
   *  UnitDelay: '<S121>/UnitDelay2'
   *  UnitDelay: '<S121>/UnitDelay3'
   *  UnitDelay: '<S121>/UnitDelay5'
   */
  if (!LDPTT_LstControl_B) {
    /* Switch: '<S121>/Switch7' incorporates:
     *  Constant: '<S121>/V_Parameter10'
     *  RelationalOperator: '<S121>/Great Than'
     *  UnaryMinus: '<S121>/Unary Minus2'
     */
    if (rtb_Abs < (-LDPTT_MxTgtLatDev_C_Mi)) {
      /* UnitDelay: '<S121>/UnitDelay2' incorporates:
       *  Constant: '<S121>/V_Parameter7'
       *  Sum: '<S121>/Subtract2'
       *  UnaryMinus: '<S121>/Unary Minus1'
       */
      LDPTT_LstMxTgtLatDevLf_Mi = rtb_Abs - (-LDPTT_MxTgtLatDev_C_Mi);
    } else {
      /* UnitDelay: '<S121>/UnitDelay2' incorporates:
       *  Constant: '<S121>/V_Const3'
       */
      LDPTT_LstMxTgtLatDevLf_Mi = 0.0F;
    }

    /* End of Switch: '<S121>/Switch7' */
    LDPTT_LstTgtLatDevLf_Mi = LDPSAI_PstnYLf_Mi - (rtb_Abs -
      LDPTT_LstMxTgtLatDevLf_Mi);
  }

  /* End of Switch: '<S121>/Switch6' */

  /* Sum: '<S121>/Subtract3' incorporates:
   *  Inport: '<Root>/Inport16'
   *  UnitDelay: '<S121>/UnitDelay3'
   */
  LDPTT_TgtPstnYLf_Mi = LDPSAI_PstnYLf_Mi - LDPTT_LstTgtLatDevLf_Mi;

  /* Switch: '<S121>/Switch16' incorporates:
   *  Constant: '<S121>/V_Parameter1'
   *  RelationalOperator: '<S121>/Equal4'
   *  Switch: '<S121>/Switch17'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  if (rtb_OR1_m0ph) {
    /* Switch: '<S121>/Switch16' */
    LDPTT_RawBdryPstnYCent_Mi = LDPTT_TgtPstnYLf_Mi;
  } else if (LDPTT_DgrSideRi_C_St == LDPSC_LstPrevDgrSide_St) {
    /* Switch: '<S121>/Switch17' incorporates:
     *  Switch: '<S121>/Switch16'
     */
    LDPTT_RawBdryPstnYCent_Mi = LDPTT_TgtPstnYRi_Mi;
  } else {
    /* Switch: '<S121>/Switch16' incorporates:
     *  Constant: '<S121>/V_Const'
     *  Switch: '<S121>/Switch17'
     */
    LDPTT_RawBdryPstnYCent_Mi = 0.0F;
  }

  /* End of Switch: '<S121>/Switch16' */

  /* Switch: '<S128>/Switch14' incorporates:
   *  Constant: '<S128>/V_Const1'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPTT_RawBdryPstnYCent_Mi;
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S128>/Switch14' */

  /* Switch: '<S141>/Switch1' incorporates:
   *  Constant: '<S141>/Constant1'
   *  Product: '<S141>/Product'
   *  Product: '<S141>/Product1'
   *  Sum: '<S141>/Add'
   *  Sum: '<S141>/Subtract'
   *  UnitDelay: '<S141>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryPstnYCent_Mi = rtb_Abs;
  } else {
    /* Product: '<S141>/Divide' incorporates:
     *  Constant: '<S128>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryPstnYCent_Mi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryPstnYCent_Mi);
  }

  /* End of Switch: '<S141>/Switch1' */

  /* Switch: '<S128>/Switch2' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S128>/Switch3' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S128>/UnitDelay1'
     *  UnitDelay: '<S141>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryPstnYCent_Mi = LDPTT_LwLnBdryPstnYCent_Mi;
    } else {
      LDPTT_LstLnBdryPstnYCent_Mi = rtb_Abs;
    }

    /* End of Switch: '<S128>/Switch3' */
  }

  /* End of Switch: '<S128>/Switch2' */

  /* Switch: '<S152>/Switch' incorporates:
   *  Constant: '<S152>/V_Parameter2'
   *  Constant: '<S152>/V_Parameter7'
   *  RelationalOperator: '<S152>/Equal2'
   *  RelationalOperator: '<S152>/Equal4'
   *  Switch: '<S152>/Switch1'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  if (LDPSC_LstPrevDgrSide_St == LDPTV_DgrSideLf_C_St) {
    /* Switch: '<S152>/Switch' */
    LDPTV_LatVel_Mps = LDPDT_LatVehSpdLf_Mps;
  } else if (LDPSC_LstPrevDgrSide_St == LDPTV_DgrSideRi_C_St) {
    /* Switch: '<S152>/Switch1' incorporates:
     *  Switch: '<S152>/Switch'
     */
    LDPTV_LatVel_Mps = LDPDT_LatVehSpdRi_Mps;
  } else {
    /* Switch: '<S152>/Switch' incorporates:
     *  Constant: '<S152>/V_Const'
     *  Switch: '<S152>/Switch1'
     */
    LDPTV_LatVel_Mps = 0.0F;
  }

  /* End of Switch: '<S152>/Switch' */

  /* RelationalOperator: '<S128>/Equal' incorporates:
   *  Constant: '<S128>/V_Parameter'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  rtb_LogicalOperator_kelu = (LDPSC_LstPrevDgrSide_St == LDPTT_DgrSideLf_C_St);

  /* Switch: '<S128>/Switch17' incorporates:
   *  Constant: '<S128>/V_Const3'
   *  Inport: '<Root>/Inport26'
   *  Logic: '<S128>/NOT2'
   *  Switch: '<S128>/Switch18'
   */
  if (!rtb_Equal1_a4qg) {
    rtb_Abs = 0.0F;
  } else if (rtb_LogicalOperator_kelu) {
    /* Switch: '<S128>/Switch18' incorporates:
     *  Inport: '<Root>/Inport24'
     */
    rtb_Abs = LDPSAI_CurvLf_ReMi;
  } else {
    rtb_Abs = LDPSAI_CurvRi_ReMi;
  }

  /* End of Switch: '<S128>/Switch17' */

  /* Switch: '<S143>/Switch1' incorporates:
   *  Constant: '<S143>/Constant1'
   *  Product: '<S143>/Product'
   *  Product: '<S143>/Product1'
   *  Sum: '<S143>/Add'
   *  Sum: '<S143>/Subtract'
   *  UnitDelay: '<S143>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryCurvCent_ReMi = rtb_Abs;
  } else {
    /* Product: '<S143>/Divide' incorporates:
     *  Constant: '<S128>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryCurvCent_ReMi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryCurvCent_ReMi);
  }

  /* End of Switch: '<S143>/Switch1' */

  /* Switch: '<S128>/Switch6' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S128>/Switch7' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S128>/UnitDelay3'
     *  UnitDelay: '<S143>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryCurvCent_ReMi = LDPTT_LwLnBdryCurvCent_ReMi;
    } else {
      LDPTT_LstLnBdryCurvCent_ReMi = rtb_Abs;
    }

    /* End of Switch: '<S128>/Switch7' */
  }

  /* End of Switch: '<S128>/Switch6' */

  /* Logic: '<S157>/AND2' incorporates:
   *  Constant: '<S157>/V_Parameter2'
   *  Constant: '<S157>/V_Parameter3'
   *  Constant: '<S157>/V_Parameter8'
   *  Constant: '<S157>/V_Parameter9'
   *  Logic: '<S157>/OR1'
   *  Logic: '<S157>/OR2'
   *  RelationalOperator: '<S157>/Equal10'
   *  RelationalOperator: '<S157>/Equal3'
   *  RelationalOperator: '<S157>/Equal4'
   *  RelationalOperator: '<S157>/Equal9'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDPTV_CurvInner_B = (((LDPSC_LstPrevDgrSide_St == LDPTV_DgrSideLf_C_St) &&
                        (LDPDT_CurveTypeLe_St == LDPTV_CurvInner_C_St)) ||
                       ((LDPSC_LstPrevDgrSide_St == LDPTV_DgrSideRi_C_St) &&
                        (LDPDT_CurveTypeRi_St == LDPTV_CurvInner_C_St)));

  /* Logic: '<S157>/AND1' incorporates:
   *  Constant: '<S157>/V_Parameter10'
   *  Constant: '<S157>/V_Parameter4'
   *  Constant: '<S157>/V_Parameter5'
   *  Constant: '<S157>/V_Parameter7'
   *  Logic: '<S157>/OR3'
   *  Logic: '<S157>/OR4'
   *  RelationalOperator: '<S157>/Equal2'
   *  RelationalOperator: '<S157>/Equal5'
   *  RelationalOperator: '<S157>/Equal7'
   *  RelationalOperator: '<S157>/Equal8'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDPTV_CurvOuter_B = (((LDPSC_LstPrevDgrSide_St == LDPTV_DgrSideLf_C_St) &&
                        (LDPDT_CurveTypeLe_St == LDPTV_CurvOuter_C_St)) ||
                       ((LDPSC_LstPrevDgrSide_St == LDPTV_DgrSideRi_C_St) &&
                        (LDPDT_CurveTypeRi_St == LDPTV_CurvOuter_C_St)));

  /* Switch: '<S150>/Switch3' incorporates:
   *  Abs: '<S150>/Abs'
   *  Constant: '<S158>/Constant'
   *  Constant: '<S159>/Constant'
   *  Inport: '<Root>/Inport1'
   *  Logic: '<S150>/OR1'
   *  Lookup_n-D: '<S150>/1-D Lookup Table'
   *  Lookup_n-D: '<S150>/1-D Lookup Table1'
   *  Lookup_n-D: '<S150>/1-D Lookup Table2'
   *  Lookup_n-D: '<S150>/1-D Lookup Table3'
   *  Product: '<S150>/Product'
   *  Product: '<S150>/Product1'
   *  Product: '<S150>/Product2'
   *  RelationalOperator: '<S150>/Equal4'
   *  RelationalOperator: '<S150>/Equal6'
   *  Switch: '<S150>/Switch'
   *  Switch: '<S152>/Switch'
   *  UnitDelay: '<S128>/UnitDelay1'
   *  UnitDelay: '<S150>/UnitDelay'
   *  UnitDelay: '<S150>/UnitDelay1'
   */
  if ((((uint32_T)LDPSC_SysOld_St) != E_LDPState_nu_ACTIVE) && (((uint32_T)
        LDPSC_SysOld_St) != E_LDPState_nu_RAMPOUT)) {
    /* Switch: '<S150>/Switch1' incorporates:
     *  Abs: '<S150>/Abs2'
     *  Constant: '<S150>/V_Const'
     *  Lookup_n-D: '<S150>/1-D Lookup Table5'
     *  Switch: '<S150>/Switch2'
     *  UnitDelay: '<S128>/UnitDelay3'
     */
    if (LDPTV_CurvInner_B) {
      rtb_Abs = look1_iflf_binlxpw(fabsf(LDPTT_LstLnBdryCurvCent_ReMi), ((const
        real32_T *)&(LDPTV_Curv_BX_ReMi[0])), ((const real32_T *)
        &(LDPTV_CurvScalInner_Cr_Fct[0])), 5U);
    } else if (LDPTV_CurvOuter_B) {
      /* Switch: '<S150>/Switch2' incorporates:
       *  Lookup_n-D: '<S150>/1-D Lookup Table4'
       *  Switch: '<S128>/Switch6'
       *  UnitDelay: '<S128>/UnitDelay3'
       */
      rtb_Abs = look1_iflf_binlxpw(LDPTT_LstLnBdryCurvCent_ReMi, ((const real32_T
        *)&(LDPTV_Curv_BX_ReMi[0])), ((const real32_T *)
        &(LDPTV_CurvScalOuter_Cr_Fct[0])), 5U);
    } else {
      rtb_Abs = 1.0F;
    }

    /* End of Switch: '<S150>/Switch1' */
    LDPTV_LstPlanningHorizon_Sec = (look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((
      const real32_T *)&(LDPTV_VehVelX_BX_Mps[0])), ((const real32_T *)
      &(LDPTV_VXPlanHorizonScal_Cr_Fct[0])), 7U) * look1_iflf_binlxpw(fabsf
      (LDPTT_LstLnBdryPstnYCent_Mi), ((const real32_T *)
      &(LDPTV_LnBdryPstnYCent_Bx_Mi[0])), ((const real32_T *)
      &(LDPTV_D2TPlanHorizonScal_Fct[0])), 5U)) * (look1_iflf_binlxpw
      (LDPTV_LatVel_Mps, ((const real32_T *)&(LDPTV_LatVel_BX_Mps[0])), ((const
      real32_T *)&(LDPTV_VYPlanningHorizon_Sec[0])), 5U) * rtb_Abs);
    LDPTV_LstWeightEndTi_Fct = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
      real32_T *)&(LDPTV_VehVelX_BX_Mps[0])), ((const real32_T *)
      &(LDPTV_WheightEndTi_Cr_Fct[0])), 7U);
  }

  /* End of Switch: '<S150>/Switch3' */

  /* Switch: '<S128>/Switch15' incorporates:
   *  Constant: '<S128>/V_Const2'
   */
  if (rtb_Equal1_a4qg) {
    /* Switch: '<S128>/Switch16' incorporates:
     *  Inport: '<Root>/Inport20'
     *  Inport: '<Root>/Inport22'
     */
    if (rtb_LogicalOperator_kelu) {
      rtb_Abs = LDPSAI_HeadAglLf_Rad;
    } else {
      rtb_Abs = LDPSAI_HeadAglRi_Rad;
    }

    /* End of Switch: '<S128>/Switch16' */
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S128>/Switch15' */

  /* Switch: '<S142>/Switch1' incorporates:
   *  Constant: '<S142>/Constant1'
   *  Product: '<S142>/Product'
   *  Product: '<S142>/Product1'
   *  Sum: '<S142>/Add'
   *  Sum: '<S142>/Subtract'
   *  UnitDelay: '<S142>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryHeadAglCent_Rad = rtb_Abs;
  } else {
    /* Product: '<S142>/Divide' incorporates:
     *  Constant: '<S128>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryHeadAglCent_Rad = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryHeadAglCent_Rad);
  }

  /* End of Switch: '<S142>/Switch1' */

  /* Switch: '<S128>/Switch4' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S128>/Switch5' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S128>/UnitDelay2'
     *  UnitDelay: '<S142>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryHeadAglCent_Rad = LDPTT_LwLnBdryHeadAglCent_Rad;
    } else {
      LDPTT_LstLnBdryHeadAglCent_Rad = rtb_Abs;
    }

    /* End of Switch: '<S128>/Switch5' */
  }

  /* End of Switch: '<S128>/Switch4' */

  /* Switch: '<S128>/Switch19' incorporates:
   *  Constant: '<S128>/V_Const4'
   *  Inport: '<Root>/Inport29'
   *  Logic: '<S128>/NOT3'
   *  Switch: '<S128>/Switch20'
   */
  if (!rtb_Equal1_a4qg) {
    rtb_Abs = 0.0F;
  } else if (rtb_LogicalOperator_kelu) {
    /* Switch: '<S128>/Switch20' incorporates:
     *  Inport: '<Root>/Inport28'
     */
    rtb_Abs = LDPSAI_CurvRateLf_ReMi2;
  } else {
    rtb_Abs = LDPSAI_CurvRateRi_ReMi2;
  }

  /* End of Switch: '<S128>/Switch19' */

  /* Switch: '<S144>/Switch1' incorporates:
   *  Constant: '<S144>/Constant1'
   *  Product: '<S144>/Product'
   *  Product: '<S144>/Product1'
   *  Sum: '<S144>/Add'
   *  Sum: '<S144>/Subtract'
   *  UnitDelay: '<S144>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryCurvRateCent_ReMi2 = rtb_Abs;
  } else {
    /* Product: '<S144>/Divide' incorporates:
     *  Constant: '<S128>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryCurvRateCent_ReMi2 = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryCurvRateCent_ReMi2);
  }

  /* End of Switch: '<S144>/Switch1' */

  /* Switch: '<S128>/Switch8' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S128>/Switch9' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S128>/UnitDelay4'
     *  UnitDelay: '<S144>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryCurvRateCent_ReMi2 = LDPTT_LwLnBdryCurvRateCent_ReMi2;
    } else {
      LDPTT_LstLnBdryCurvRateCent_ReMi2 = rtb_Abs;
    }

    /* End of Switch: '<S128>/Switch9' */
  }

  /* End of Switch: '<S128>/Switch8' */

  /* Switch: '<S128>/Switch13' incorporates:
   *  Constant: '<S128>/V_Const'
   *  Inport: '<Root>/Inport15'
   *  Logic: '<S128>/NOT'
   *  Switch: '<S128>/Switch12'
   */
  if (!rtb_Equal1_a4qg) {
    rtb_Abs = 0.0F;
  } else if (rtb_LogicalOperator_kelu) {
    /* Switch: '<S128>/Switch12' incorporates:
     *  Inport: '<Root>/Inport14'
     */
    rtb_Abs = LDPSAI_PstnXLf_Mi;
  } else {
    rtb_Abs = LDPSAI_PstnXRi_Mi;
  }

  /* End of Switch: '<S128>/Switch13' */

  /* Switch: '<S145>/Switch1' incorporates:
   *  Constant: '<S145>/Constant1'
   *  Product: '<S145>/Product'
   *  Product: '<S145>/Product1'
   *  Sum: '<S145>/Add'
   *  Sum: '<S145>/Subtract'
   *  UnitDelay: '<S145>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryPstnXCent_Mi = rtb_Abs;
  } else {
    /* Product: '<S145>/Divide' incorporates:
     *  Constant: '<S128>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryPstnXCent_Mi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryPstnXCent_Mi);
  }

  /* End of Switch: '<S145>/Switch1' */

  /* Switch: '<S128>/Switch' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S128>/Switch1' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S128>/UnitDelay'
     *  UnitDelay: '<S145>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryPstnXCent_Mi = LDPTT_LwLnBdryPstnXCent_Mi;
    } else {
      LDPTT_LstLnBdryPstnXCent_Mi = rtb_Abs;
    }

    /* End of Switch: '<S128>/Switch1' */
  }

  /* End of Switch: '<S128>/Switch' */

  /* Switch: '<S128>/Switch21' incorporates:
   *  Constant: '<S128>/V_Const5'
   *  Inport: '<Root>/Inport33'
   *  Logic: '<S128>/NOT4'
   *  Switch: '<S128>/Switch22'
   */
  if (!rtb_Equal1_a4qg) {
    rtb_Abs = 0.0F;
  } else if (rtb_LogicalOperator_kelu) {
    /* Switch: '<S128>/Switch22' incorporates:
     *  Inport: '<Root>/Inport32'
     */
    rtb_Abs = LDPSAI_VldLengLf_Mi;
  } else {
    rtb_Abs = LDPSAI_VldLengRi_Mi;
  }

  /* End of Switch: '<S128>/Switch21' */

  /* Switch: '<S146>/Switch1' incorporates:
   *  Constant: '<S146>/Constant1'
   *  Product: '<S146>/Product'
   *  Product: '<S146>/Product1'
   *  Sum: '<S146>/Add'
   *  Sum: '<S146>/Subtract'
   *  UnitDelay: '<S146>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryVldLengCent_Mi = rtb_Abs;
  } else {
    /* Product: '<S146>/Divide' incorporates:
     *  Constant: '<S128>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryVldLengCent_Mi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryVldLengCent_Mi);
  }

  /* End of Switch: '<S146>/Switch1' */

  /* Switch: '<S128>/Switch10' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S128>/Switch11' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S128>/UnitDelay5'
     *  UnitDelay: '<S146>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryVldLengCent_Mi = LDPTT_LwLnBdryVldLengCent_Mi;
    } else {
      LDPTT_LstLnBdryVldLengCent_Mi = rtb_Abs;
    }

    /* End of Switch: '<S128>/Switch11' */
  }

  /* End of Switch: '<S128>/Switch10' */

  /* UnitDelay: '<S148>/UnitDelay4' */
  rtb_LogicalOperator_kelu = LDPTV_LstCtrl_St;

  /* Switch: '<S148>/Switch7' incorporates:
   *  Lookup_n-D: '<S148>/1-D Lookup Table'
   *  Switch: '<S152>/Switch'
   *  UnitDelay: '<S148>/UnitDelay1'
   *  UnitDelay: '<S148>/UnitDelay4'
   */
  if (!LDPTV_LstCtrl_St) {
    LDPTV_LstSteWhlGrad_ReS = look1_iflf_binlxpw(LDPTV_LatVel_Mps, ((const
      real32_T *)&(LDPTV_LatVel_BX_Mps[0])), ((const real32_T *)
      &(LDPTV_VYStrWhStifRIGrad_Cr_Res[0])), 5U);
  }

  /* End of Switch: '<S148>/Switch7' */

  /* RelationalOperator: '<S148>/Equal6' incorporates:
   *  Constant: '<S153>/Constant'
   *  UnitDelay: '<S148>/UnitDelay4'
   */
  LDPTV_LstCtrl_St = (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_ACTIVE);

  /* RelationalOperator: '<S148>/Equal2' incorporates:
   *  Constant: '<S154>/Constant'
   */
  rtb_OR1_m0ph = (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_RAMPOUT);

  /* Switch: '<S148>/Switch4' incorporates:
   *  Logic: '<S148>/NOT'
   *  Switch: '<S148>/Switch5'
   *  UnitDelay: '<S148>/UnitDelay4'
   */
  if (!LDPTV_LstCtrl_St) {
    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Switch: '<S148>/Switch4'
     *  UnitDelay: '<S148>/UnitDelay1'
     */
    LDPTV_SteWhlGrad_ReS = LDPTV_LstSteWhlGrad_ReS;
  } else if (rtb_OR1_m0ph) {
    /* Switch: '<S148>/Switch6' incorporates:
     *  Switch: '<S148>/Switch5'
     */
    if (LDPSC_Abort_B) {
      /* SignalConversion generated from: '<S1>/LDP' incorporates:
       *  Constant: '<S148>/V_Parameter9'
       *  Switch: '<S148>/Switch4'
       */
      LDPTV_SteWhlGrad_ReS = LDPTV_SteWhlGradAbort_C_ReS;
    } else {
      /* SignalConversion generated from: '<S1>/LDP' incorporates:
       *  Constant: '<S148>/V_Parameter8'
       *  Switch: '<S148>/Switch4'
       */
      LDPTV_SteWhlGrad_ReS = LDPTV_SteWhlGrad_C_ReS;
    }

    /* End of Switch: '<S148>/Switch6' */
  } else {
    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S148>/V_Const3'
     *  Switch: '<S148>/Switch4'
     *  Switch: '<S148>/Switch5'
     */
    LDPTV_SteWhlGrad_ReS = 0.0F;
  }

  /* End of Switch: '<S148>/Switch4' */

  /* Switch: '<S148>/Switch2' incorporates:
   *  Switch: '<S148>/Switch3'
   *  Switch: '<S148>/Switch8'
   *  Switch: '<S148>/Switch9'
   *  UnitDelay: '<S148>/UnitDelay4'
   */
  if (LDPTV_LstCtrl_St) {
    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S148>/V_Parameter7'
     *  Switch: '<S148>/Switch2'
     */
    LDPTV_MxTrqScalGrad_ReS = LDPTV_MxTrqScalInGrad_C_Res;

    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S148>/V_Parameter12'
     *  Switch: '<S148>/Switch8'
     */
    LDPTV_TrqRampGrad_ReS = LDPTV_TrqRampGradIn_C_ReS;
  } else if (rtb_OR1_m0ph) {
    /* Switch: '<S148>/Switch3' incorporates:
     *  Constant: '<S148>/V_Parameter2'
     *  SignalConversion generated from: '<S1>/LDP'
     *  Switch: '<S148>/Switch2'
     */
    LDPTV_MxTrqScalGrad_ReS = LDPTV_MxTrqScalOutGrad_C_Res;

    /* Switch: '<S148>/Switch10' incorporates:
     *  Switch: '<S148>/Switch9'
     */
    if (LDPSC_Abort_B) {
      /* SignalConversion generated from: '<S1>/LDP' incorporates:
       *  Constant: '<S148>/V_Parameter11'
       *  Switch: '<S148>/Switch8'
       */
      LDPTV_TrqRampGrad_ReS = LDPTV_TrqRampOutGradAbort_C_ReS;
    } else {
      /* SignalConversion generated from: '<S1>/LDP' incorporates:
       *  Constant: '<S148>/V_Parameter10'
       *  Switch: '<S148>/Switch8'
       */
      LDPTV_TrqRampGrad_ReS = LDPTV_TrqRampOutGrad_C_ReS;
    }

    /* End of Switch: '<S148>/Switch10' */
  } else {
    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S148>/V_Const8'
     *  Switch: '<S148>/Switch2'
     *  Switch: '<S148>/Switch3'
     */
    LDPTV_MxTrqScalGrad_ReS = 0.0F;

    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S148>/V_Const4'
     *  Switch: '<S148>/Switch8'
     *  Switch: '<S148>/Switch9'
     */
    LDPTV_TrqRampGrad_ReS = 0.0F;
  }

  /* End of Switch: '<S148>/Switch2' */

  /* Switch: '<S148>/Switch12' */
  if (!rtb_LogicalOperator_kelu) {
    /* Switch: '<S148>/Switch13' incorporates:
     *  Constant: '<S148>/V_Const5'
     *  Constant: '<S148>/V_Const6'
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S148>/Equal4'
     *  RelationalOperator: '<S148>/Equal7'
     *  Switch: '<S148>/Switch14'
     */
    if (3 == ((int32_T)LDPSAI_LDPMod_St)) {
      /* UnitDelay: '<S148>/UnitDelay' incorporates:
       *  Lookup_n-D: '<S148>/1-D Lookup Table3'
       *  Switch: '<S152>/Switch'
       */
      LDPTV_LstDMCDeraLvl_Fct = look1_iflf_binlxpw(LDPTV_LatVel_Mps, ((const
        real32_T *)&(LDPTV_LatVel_BX_Mps[0])), ((const real32_T *)
        &(LDPTV_VYMD3DeratingLevel_Fct[0])), 5U);
    } else if (2 == ((int32_T)LDPSAI_LDPMod_St)) {
      /* Switch: '<S148>/Switch14' incorporates:
       *  Lookup_n-D: '<S148>/1-D Lookup Table2'
       *  Switch: '<S152>/Switch'
       *  UnitDelay: '<S148>/UnitDelay'
       */
      LDPTV_LstDMCDeraLvl_Fct = look1_iflf_binlxpw(LDPTV_LatVel_Mps, ((const
        real32_T *)&(LDPTV_LatVel_BX_Mps[0])), ((const real32_T *)
        &(LDPTV_VYMD2DeratingLevel_Fct[0])), 5U);
    } else {
      /* UnitDelay: '<S148>/UnitDelay' incorporates:
       *  Lookup_n-D: '<S148>/1-D Lookup Table1'
       *  Switch: '<S148>/Switch14'
       *  Switch: '<S152>/Switch'
       */
      LDPTV_LstDMCDeraLvl_Fct = look1_iflf_binlxpw(LDPTV_LatVel_Mps, ((const
        real32_T *)&(LDPTV_LatVel_BX_Mps[0])), ((const real32_T *)
        &(LDPTV_VYMD1DeratingLevel_Fct[0])), 5U);
    }

    /* End of Switch: '<S148>/Switch13' */
  }

  /* End of Switch: '<S148>/Switch12' */

  /* Switch: '<S126>/Switch14' incorporates:
   *  Constant: '<S126>/V_Const2'
   *  Inport: '<Root>/Inport20'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPSAI_HeadAglLf_Rad;
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S126>/Switch14' */

  /* Switch: '<S131>/Switch1' incorporates:
   *  Constant: '<S131>/Constant1'
   *  Product: '<S131>/Product'
   *  Product: '<S131>/Product1'
   *  Sum: '<S131>/Add'
   *  Sum: '<S131>/Subtract'
   *  UnitDelay: '<S131>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryHeadAglLf_Rad = rtb_Abs;
  } else {
    /* Product: '<S131>/Divide' incorporates:
     *  Constant: '<S126>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryHeadAglLf_Rad = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryHeadAglLf_Rad);
  }

  /* End of Switch: '<S131>/Switch1' */

  /* Switch: '<S126>/Switch4' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S126>/Switch5' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S126>/UnitDelay2'
     *  UnitDelay: '<S131>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryHeadAglLf_Rad = LDPTT_LwLnBdryHeadAglLf_Rad;
    } else {
      LDPTT_LstLnBdryHeadAglLf_Rad = rtb_Abs;
    }

    /* End of Switch: '<S126>/Switch5' */
  }

  /* End of Switch: '<S126>/Switch4' */

  /* Switch: '<S126>/Switch15' incorporates:
   *  Constant: '<S126>/V_Const3'
   *  Inport: '<Root>/Inport24'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPSAI_CurvLf_ReMi;
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S126>/Switch15' */

  /* Switch: '<S132>/Switch1' incorporates:
   *  Constant: '<S132>/Constant1'
   *  Product: '<S132>/Product'
   *  Product: '<S132>/Product1'
   *  Sum: '<S132>/Add'
   *  Sum: '<S132>/Subtract'
   *  UnitDelay: '<S132>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryCurvLf_ReMi = rtb_Abs;
  } else {
    /* Product: '<S132>/Divide' incorporates:
     *  Constant: '<S126>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryCurvLf_ReMi = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryCurvLf_ReMi);
  }

  /* End of Switch: '<S132>/Switch1' */

  /* Switch: '<S126>/Switch6' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S126>/Switch7' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S126>/UnitDelay3'
     *  UnitDelay: '<S132>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryCurvLf_ReMi = LDPTT_LwLnBdryCurvLf_ReMi;
    } else {
      LDPTT_LstLnBdryCurvLf_ReMi = rtb_Abs;
    }

    /* End of Switch: '<S126>/Switch7' */
  }

  /* End of Switch: '<S126>/Switch6' */

  /* Switch: '<S126>/Switch16' incorporates:
   *  Constant: '<S126>/V_Const4'
   *  Inport: '<Root>/Inport28'
   */
  if (rtb_Equal1_a4qg) {
    rtb_Abs = LDPSAI_CurvRateLf_ReMi2;
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S126>/Switch16' */

  /* Switch: '<S133>/Switch1' incorporates:
   *  Constant: '<S133>/Constant1'
   *  Product: '<S133>/Product'
   *  Product: '<S133>/Product1'
   *  Sum: '<S133>/Add'
   *  Sum: '<S133>/Subtract'
   *  UnitDelay: '<S133>/Unit Delay'
   */
  if (rtb_AND_b1uz) {
    LDPTT_LwLnBdryCurvRateLf_ReMi2 = rtb_Abs;
  } else {
    /* Product: '<S133>/Divide' incorporates:
     *  Constant: '<S126>/V_Parameter11'
     *  Inport: '<Root>/Inport45'
     */
    rtb_Divide_l2e1 = LDPSAI_CycleTime_Sec / LDPTT_TiTpLnLw_C_Sec;
    LDPTT_LwLnBdryCurvRateLf_ReMi2 = (rtb_Abs * rtb_Divide_l2e1) + ((1.0F -
      rtb_Divide_l2e1) * LDPTT_LwLnBdryCurvRateLf_ReMi2);
  }

  /* End of Switch: '<S133>/Switch1' */

  /* Switch: '<S126>/Switch8' */
  if (!rtb_Equal3_newf) {
    /* Switch: '<S126>/Switch9' incorporates:
     *  Constant: '<S120>/V_Parameter2'
     *  UnitDelay: '<S126>/UnitDelay4'
     *  UnitDelay: '<S133>/Unit Delay'
     */
    if (LDPTT_EnLwFilt_C_B) {
      LDPTT_LstLnBdryCurvRateLf_ReMi2 = LDPTT_LwLnBdryCurvRateLf_ReMi2;
    } else {
      LDPTT_LstLnBdryCurvRateLf_ReMi2 = rtb_Abs;
    }

    /* End of Switch: '<S126>/Switch9' */
  }

  /* End of Switch: '<S126>/Switch8' */

  /* RelationalOperator: '<S121>/Equal5' incorporates:
   *  Constant: '<S147>/Constant'
   *  UnitDelay: '<S121>/UnitDelay5'
   */
  LDPTT_LstControl_B = (E_LDPState_nu_ACTIVE == ((uint32_T)LDPSC_SysOld_St));

  /* Switch: '<S149>/Switch' incorporates:
   *  Constant: '<S155>/Constant'
   *  Constant: '<S156>/Constant'
   *  RelationalOperator: '<S149>/Equal1'
   *  RelationalOperator: '<S149>/Equal4'
   *  Switch: '<S149>/Switch1'
   */
  if (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_RAMPOUT) {
    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S149>/V_Parameter3'
     *  Switch: '<S149>/Switch'
     */
    LDPTV_TrajCtrlSt_St = LDPTV_ReqFreeze_C_ST;
  } else if (((uint32_T)LDPSC_SysOld_St) == E_LDPState_nu_ACTIVE) {
    /* Switch: '<S149>/Switch1' incorporates:
     *  Constant: '<S149>/V_Parameter4'
     *  SignalConversion generated from: '<S1>/LDP'
     *  Switch: '<S149>/Switch'
     */
    LDPTV_TrajCtrlSt_St = LDPTV_ReqOn_C_ST;
  } else {
    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S149>/V_Parameter5'
     *  Switch: '<S149>/Switch'
     *  Switch: '<S149>/Switch1'
     */
    LDPTV_TrajCtrlSt_St = LDPTV_ReqOff_C_ST;
  }

  /* SignalConversion: '<S35>/Signal Conversion1' */
  rtb_VectorConcatenate_k1un[0] = rtb_LDPSC_VehicleInvalid_B;

  /* SignalConversion: '<S35>/Signal Conversion2' incorporates:
   *  UnitDelay: '<S119>/Unit Delay'
   */
  rtb_VectorConcatenate_k1un[1] = LDPSC_VehYawRateHyst_bool;

  /* SignalConversion: '<S35>/Signal Conversion' */
  rtb_VectorConcatenate_k1un[2] = rtb_LDPSC_VelYInvalid_B;

  /* SignalConversion: '<S35>/Signal Conversion3' */
  rtb_VectorConcatenate_k1un[3] = rtb_LDPSC_LnCurveInvalid_B;

  /* SignalConversion: '<S35>/Signal Conversion4' */
  rtb_VectorConcatenate_k1un[4] = rtb_LDPSC_DrvStInvalid_B;

  /* SignalConversion: '<S35>/Signal Conversion5' */
  rtb_VectorConcatenate_k1un[5] = rtb_AND_ddra;

  /* SignalConversion: '<S35>/Signal Conversion6' */
  rtb_VectorConcatenate_k1un[6] = rtb_AND_ex2t;

  /* SignalConversion: '<S35>/Signal Conversion7' */
  rtb_VectorConcatenate_k1un[7] = rtb_LDPSC_VehStInvalid_B;

  /* SignalConversion: '<S35>/Signal Conversion9' incorporates:
   *  Inport: '<Root>/Inport47'
   */
  rtb_VectorConcatenate_k1un[8] = LDPSAI_AEBActive_B;

  /* S-Function (ex_sfun_set_bit): '<S116>/ex_sfun_set_bit' incorporates:
   *  Constant: '<S110>/Constant'
   */
  set_bit(0U, (boolean_T*)&rtb_VectorConcatenate_k1un[0], (uint8_T*)
          (&(LDPSA_SetBit_BS_Param_1[0])), ((uint8_T)9U), &rtb_ex_sfun_set_bit);

  /* SignalConversion: '<S35>/Signal Conversion8' incorporates:
   *  DataTypeConversion: '<S116>/Data Type Conversion1'
   */
  LDPSC_SuppValid_Debug = (uint16_T)rtb_ex_sfun_set_bit;

  /* Logic: '<S77>/Logical Operator1' */
  LDPSC_Trig_B = (LDPSC_TrigRi_B || LDPSC_TrigLf_B);

  /* Sum: '<S175>/Subtract2' incorporates:
   *  Inport: '<Root>/Inport45'
   */
  rtb_Subtract2_pfel -= LDPSAI_CycleTime_Sec;

  /* MinMax: '<S175>/Max' incorporates:
   *  Constant: '<S175>/Constant5'
   *  UnitDelay: '<S175>/Unit Delay'
   */
  LDPVSE_HodTiTrnSglLf_Sec = fmaxf(rtb_Subtract2_pfel, 0.0F);

  /* Sum: '<S93>/Subtract2' incorporates:
   *  Constant: '<S93>/Constant5'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S93>/Max'
   */
  LDPSC_SuppTimeOldLf_Sec -= LDPSAI_CycleTime_Sec;
  LDPSC_SuppTimeOldLf_Sec = fmaxf(LDPSC_SuppTimeOldLf_Sec, 0.0F);

  /* Sum: '<S92>/Subtract2' incorporates:
   *  Constant: '<S92>/Constant5'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S92>/Max'
   */
  LDPSC_HdTiTrigLf_Sec -= LDPSAI_CycleTime_Sec;
  LDPSC_HdTiTrigLf_Sec = fmaxf(LDPSC_HdTiTrigLf_Sec, 0.0F);

  /* Sum: '<S176>/Subtract2' incorporates:
   *  Constant: '<S176>/Constant5'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S176>/Max'
   */
  LDPVSE_HodTiTrnSglRi_Sec -= LDPSAI_CycleTime_Sec;
  LDPVSE_HodTiTrnSglRi_Sec = fmaxf(LDPVSE_HodTiTrnSglRi_Sec, 0.0F);

  /* Sum: '<S105>/Subtract2' incorporates:
   *  Constant: '<S105>/Constant5'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S105>/Max'
   */
  LDPSC_SuppTimeOldRi_Sec -= LDPSAI_CycleTime_Sec;
  LDPSC_SuppTimeOldRi_Sec = fmaxf(LDPSC_SuppTimeOldRi_Sec, 0.0F);

  /* Sum: '<S104>/Subtract2' incorporates:
   *  Constant: '<S104>/Constant5'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S104>/Max'
   */
  LDPSC_HdTiTrigRi_Sec -= LDPSAI_CycleTime_Sec;
  LDPSC_HdTiTrigRi_Sec = fmaxf(LDPSC_HdTiTrigRi_Sec, 0.0F);

  /* Lookup_n-D: '<S78>/1-D Lookup Table1' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPSC_DlcThdMode2_Mi = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDPSC_VehSpdXDTL_BX_Mps[0])), ((const real32_T *)
    &(LDPSC_DstcTrsdVehSpdXDTL2_Cr_Mi[0])), 8U);

  /* Lookup_n-D: '<S78>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPSC_DlcThdMode1_Mi = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDPSC_VehSpdXDTL_BX_Mps[0])), ((const real32_T *)
    &(LDPSC_DstcTrsdVehSpdXDTL1_Cr_Mi[0])), 8U);

  /* Lookup_n-D: '<S78>/1-D Lookup Table2' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDPSC_DlcThdMode3_Mi = look1_iflf_binlxpw(LDPSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDPSC_VehSpdXDTL_BX_Mps[0])), ((const real32_T *)
    &(LDPSC_DstcTrsdVehSpdXDTL3_Cr_Mi[0])), 8U);

  /* Switch: '<S78>/Switch1' incorporates:
   *  Constant: '<S78>/V_Parameter10'
   *  Constant: '<S78>/V_Parameter9'
   *  Inport: '<Root>/Inport10'
   *  RelationalOperator: '<S78>/Relational Operator1'
   *  RelationalOperator: '<S78>/Relational Operator2'
   *  Switch: '<S78>/Switch'
   */
  if (LDPSAI_LDPMod_St == LDPSC_DrvMod2_C_St) {
    rtb_Abs = LDPSC_DlcThdMode2_Mi;
  } else if (LDPSAI_LDPMod_St == LDPSC_DrvMod3_C_St) {
    /* Switch: '<S78>/Switch' */
    rtb_Abs = LDPSC_DlcThdMode3_Mi;
  } else {
    rtb_Abs = LDPSC_DlcThdMode1_Mi;
  }

  /* End of Switch: '<S78>/Switch1' */

  /* Product: '<S78>/Product' */
  LDPSC_DstcToLnTrsd_Mi = rtb_Abs * rtb_uDLookupTable9_idx_1;

  /* Update for UnitDelay: '<S95>/Unit Delay' */
  LDPSC_HdTiTrigRiEn_B = rtb_LogicalOperator2;

  /* Update for RelationalOperator: '<S80>/Relational Operator5' incorporates:
   *  Switch: '<S23>/Switch1'
   *  UnitDelay: '<S4>/UnitDelay'
   */
  LDPSC_SysOld_St = LDPSC_SysOut_St;

  /* Update for UnitDelay: '<S98>/Unit Delay' */
  LDPSC_PreActiveEdgeRi = rtb_RelationalOperator10;

  /* Update for UnitDelay: '<S99>/Unit Delay' */
  LDPSC_ContinTrigRiEn_B = rtb_LogicalOperator14;

  /* Update for UnitDelay: '<S174>/Unit Delay' */
  LDPVSE_EdgeRisTrnSglLf_B = rtb_RelationalOperator3_k31o;

  /* Update for UnitDelay: '<S83>/Unit Delay' */
  LDPSC_HdTiTrigLfEn_B = rtb_RelationalOperator4_jya3;

  /* Update for UnitDelay: '<S86>/Unit Delay' */
  LDPSC_PreActiveEdgeLf = rtb_RelationalOperator10_gbg2;

  /* Update for UnitDelay: '<S87>/Unit Delay' */
  LDPSC_ContinTrigLfEn_B = rtb_LogicalOperator14_fahu;

  /* Update for UnitDelay: '<S173>/Unit Delay' */
  LDPVSE_EdgeRisTrnSglRi_B = rtb_RelationalOperator1_mqkp;

  /* Update for UnitDelay: '<S60>/Unit Delay' */
  LDPSC_EdgeRisWarming_B = LDPSC_MinLdwBySysSt_B;

  /* Update for UnitDelay: '<S41>/Unit Delay' */
  LDPSC_EdgeRisWarmMx_B = LDPSC_MaxDurationBySysSt_B;

  /* Update for UnitDelay: '<S58>/Unit Delay' */
  LDPSC_EdgeRisDegr_B = LDPSC_Degradation_B;

  /* Update for UnitDelay: '<S32>/UnitDelay' */
  LDPSC_DegrOld_B = LDPSC_Degr_B;

  /* Update for UnitDelay: '<S46>/Unit Delay' */
  LDPSC_EdgeRisActive_B = rtb_RelationalOperator29;

  /* Update for UnitDelay: '<S44>/Unit Delay' */
  LDPSC_EdgeRisFns_B = rtb_LogicalOperator8;

  /* Update for UnitDelay: '<S47>/Unit Delay' */
  LDPSC_EdgeRisActive_Ri_B = rtb_LDPDT_LnCurvVldRi_B;

  /* Update for UnitDelay: '<S45>/Unit Delay' */
  LDPSC_EdgeRisFns_Ri_B = rtb_LDPDT_LnCurvVldLf_B;

  /* Update for UnitDelay: '<S25>/Unit Delay' incorporates:
   *  Inport: '<Root>/Inport9'
   */
  LDPSC_PrevSwitchUnitDelay_bool = LDPSAI_LDPSwitchEn_B;

  /* Update for UnitDelay: '<S28>/Unit Delay' */
  LDPSC_PrevStandbyUnitDelay_bool = rtb_NOT_fftx;

  /* Update for UnitDelay: '<S122>/Unit Delay' */
  LDPTT_CtrlIniEn_B = rtb_Equal4;

  /* Update for UnitDelay: '<S121>/UnitDelay4' */
  LDPTT_LstTgtLatDstcRi_Mi = LDPTT_TgtLatDstcRi_Mi;

  /* Update for UnitDelay: '<S121>/UnitDelay1' */
  LDPTT_LstTgtLatDstcLf_Mi = LDPTT_TgtLatDstcLf_Mi;

  /* End of Outputs for S-Function (fcgen): '<S1>/Function-Call Generator' */

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDPSC_DgrSide_St = LDPSC_LstPrevDgrSide_St;

  /* S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/LDP'
   */
  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Logic: '<S37>/Logical Operator'
   */
  LDPSC_RdyToTrig_B = (LDPSC_StrgRdy_B && LDPSC_WkRdy_B);

  /* End of Outputs for S-Function (fcgen): '<S1>/Function-Call Generator' */

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S126>/UnitDelay'
   */
  LDPTT_LnBdryPstnXLf_Mi = LDPTT_LstLnBdryPstnXLf_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S126>/UnitDelay1'
   */
  LDPTT_LnBdryPstnYLf_Mi = LDPTT_LstLnBdryPstnYLf_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S126>/UnitDelay2'
   */
  LDPTT_LnBdryHeadAglLf_Rad = LDPTT_LstLnBdryHeadAglLf_Rad;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S126>/UnitDelay3'
   */
  LDPTT_LnBdryCurvLf_ReMi = LDPTT_LstLnBdryCurvLf_ReMi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S126>/UnitDelay4'
   */
  LDPTT_LnBdryCurvRateLf_ReMi2 = LDPTT_LstLnBdryCurvRateLf_ReMi2;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S126>/UnitDelay5'
   */
  LDPTT_LnBdryVldLengLf_Mi = LDPTT_LstLnBdryVldLengLf_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S127>/UnitDelay'
   */
  LDPTT_LnBdryPstnXRi_Mi = LDPTT_LstLnBdryPstnXRi_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S127>/UnitDelay1'
   */
  LDPTT_LnBdryPstnYRi_Mi = LDPTT_LstLnBdryPstnYRi_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S127>/UnitDelay2'
   */
  LDPTT_LnBdryHeadAglRi_Rad = LDPTT_LstLnBdryHeadAglRi_Rad;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S127>/UnitDelay3'
   */
  LDPTT_LnBdryCurvRi_ReMi = LDPTT_LstLnBdryCurvRi_ReMi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S127>/UnitDelay4'
   */
  LDPTT_LnBdryCurvRateRi_ReMi2 = LDPTT_LstLnBdryCurvRateRi_ReMi2;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S127>/UnitDelay5'
   */
  LDPTT_LnBdryVldLengRi_Mi = LDPTT_LstLnBdryVldLengRi_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S128>/UnitDelay'
   */
  LDPTT_LnBdryPstnXCent_Mi = LDPTT_LstLnBdryPstnXCent_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S128>/UnitDelay1'
   */
  LDPTT_LnBdryPstnYCent_Mi = LDPTT_LstLnBdryPstnYCent_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S128>/UnitDelay2'
   */
  LDPTT_LnBdryHeadAglCent_Rad = LDPTT_LstLnBdryHeadAglCent_Rad;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S128>/UnitDelay3'
   */
  LDPTT_LnBdryCurvCent_ReMi = LDPTT_LstLnBdryCurvCent_ReMi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S128>/UnitDelay4'
   */
  LDPTT_LnBdryCurvRateCent_ReMi2 = LDPTT_LstLnBdryCurvRateCent_ReMi2;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S128>/UnitDelay5'
   */
  LDPTT_LnBdryVldLengCent_Mi = LDPTT_LstLnBdryVldLengCent_Mi;

  /* S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/LDP'
   */
  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter23'
   */
  LDPTV_FTireAccMx_Mps2 = LDPTV_FTireAccMx_C_Mps2;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter1'
   */
  LDPTV_FTireAccMn_Mps2 = LDPTV_FTireAccMn_C_Mps2;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter2'
   */
  LDPTV_DstcYTgtAreaLf_Mi = LDPTV_DstcYTgtAreaLf_C_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter3'
   */
  LDPTV_DstcYTgtAreaRi_Mi = LDPTV_DstcYTgtAreaRi_C_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter4'
   */
  LDPTV_FctTgtDistY_Fct = LDPTV_FctTgtDistY_C_Fct;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter5'
   */
  LDPTV_TrajPlanServQu_Fct = LDPTV_TrajPlanServQu_C_Fct;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter7'
   */
  LDPTV_PredTiCurv_Sec = LDPTV_PredTiCurv_C_Sec;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter8'
   */
  LDPTV_PredTiAgl_Sec = LDPTV_PredTiAgl_C_Sec;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter11'
   */
  LDPTV_TiLmtEnDura_Sec = LDPTV_TiLmtEnDura_C_Sec;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter12'
   */
  LDPTV_JerkLmtMx_Mps3 = LDPTV_JerkLmtMx_C_Mps3;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter18'
   */
  LDPTV_VeloXObst_Mps = LDPTV_VeloXObst_C_Mps;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter14'
   */
  LDPTV_AccXObst_Mps2 = LDPTV_AccXObst_C_Mps2;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter16'
   */
  LDPTV_DstcXObst_Mi = LDPTV_DstcXObst_C_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter17'
   */
  LDPTV_DstcYObst_Mi = LDPTV_DstcYObst_C_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter6'
   */
  LDPTV_LmtCurvMx_ReMi = LDPTV_LmtCurvMx_C_ReMi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter19'
   */
  LDPTV_LmtCurvGradIncMx_ReMps = LDPTV_LmtCurvGradIncMx_C_ReMps;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter20'
   */
  LDPTV_LmtCurvGradDecMx_ReMps = LDPTV_LmtCurvGradDecMx_C_ReMps;

  /* End of Outputs for S-Function (fcgen): '<S1>/Function-Call Generator' */

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Inport: '<Root>/Inport46'
   */
  LDPTV_SnsTiStamp_Sec = LDPSAI_ABDTimeStamp_Sec;

  /* S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/LDP'
   */
  /* Switch: '<S148>/Switch' incorporates:
   *  Switch: '<S148>/Switch1'
   *  Switch: '<S148>/Switch11'
   *  UnitDelay: '<S148>/UnitDelay4'
   */
  if (LDPTV_LstCtrl_St) {
    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S148>/V_Parameter4'
     */
    LDPTV_SteWhlGradLmt_Fct = LDPTV_SteWhlGradLmt_C_Fct;

    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S148>/V_Parameter6'
     */
    LDPTV_MxTrqScalGradLmt_Fct = LDPTV_MxTrqScalGradLmt_C_Fct;

    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  UnitDelay: '<S148>/UnitDelay'
     */
    LDPTV_DMCDeraLvl_Fct = LDPTV_LstDMCDeraLvl_Fct;
  } else {
    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S148>/V_Const'
     */
    LDPTV_SteWhlGradLmt_Fct = 0.0F;

    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S148>/V_Const1'
     */
    LDPTV_MxTrqScalGradLmt_Fct = 0.0F;

    /* SignalConversion generated from: '<S1>/LDP' incorporates:
     *  Constant: '<S148>/V_Const7'
     */
    LDPTV_DMCDeraLvl_Fct = 0.0F;
  }

  /* End of Switch: '<S148>/Switch' */
  /* End of Outputs for S-Function (fcgen): '<S1>/Function-Call Generator' */

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S150>/UnitDelay'
   */
  LDPTV_WeightEndTi_Fct = LDPTV_LstWeightEndTi_Fct;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  UnitDelay: '<S150>/UnitDelay1'
   */
  LDPTV_PlanningHorizon_Sec = LDPTV_LstPlanningHorizon_Sec;

  /* S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/LDP'
   */
  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S148>/V_Parameter5'
   */
  LDPTV_HighStatReq_B = LDPTV_HiStatAcc_C_B;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter9'
   */
  LDPTV_LatCpstnEn_B = LDPTV_LatCpstnEn_C_B;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter10'
   */
  LDPTV_LmtEn_B = LDPTV_LmtEn_C_B;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter13'
   */
  LDPTV_TrigReplan_B = LDPTV_TrigReplan_C_B;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter15'
   */
  LDPTV_WidObst_Mi = LDPTV_WidObst_C_Mi;

  /* SignalConversion generated from: '<S1>/LDP' incorporates:
   *  Constant: '<S151>/V_Parameter21'
   */
  LDPTV_LmtCurvGradCtrlMx_ReMps = LDPTV_LmtCurvGradCtrlMx_C_ReMps;

  /* End of Outputs for S-Function (fcgen): '<S1>/Function-Call Generator' */

  /* SignalConversion generated from: '<S1>/LDP' */
  LDPVSE_NVRAMVehStartupSpd_Kmph = (real32_T)LDPVSE_NVRAMVehStartupSpd__osjy;
}

/* Model initialize function */
void LDPSA_initialize(void)
{
  /* Registration code */

  /* block I/O */

  /* exported global signals */
  LDPTT_LnBdryPstnXLf_Mi = 0.0F;
  LDPTT_LnBdryPstnYLf_Mi = 0.0F;
  LDPTT_LnBdryHeadAglLf_Rad = 0.0F;
  LDPTT_LnBdryCurvLf_ReMi = 0.0F;
  LDPTT_LnBdryCurvRateLf_ReMi2 = 0.0F;
  LDPTT_LnBdryVldLengLf_Mi = 0.0F;
  LDPTT_LnBdryPstnXRi_Mi = 0.0F;
  LDPTT_LnBdryPstnYRi_Mi = 0.0F;
  LDPTT_LnBdryHeadAglRi_Rad = 0.0F;
  LDPTT_LnBdryCurvRi_ReMi = 0.0F;
  LDPTT_LnBdryCurvRateRi_ReMi2 = 0.0F;
  LDPTT_LnBdryVldLengRi_Mi = 0.0F;
  LDPTT_LnBdryPstnXCent_Mi = 0.0F;
  LDPTT_LnBdryPstnYCent_Mi = 0.0F;
  LDPTT_LnBdryHeadAglCent_Rad = 0.0F;
  LDPTT_LnBdryCurvCent_ReMi = 0.0F;
  LDPTT_LnBdryCurvRateCent_ReMi2 = 0.0F;
  LDPTT_LnBdryVldLengCent_Mi = 0.0F;
  LDPTT_TgtPstnYLf_Mi = 0.0F;
  LDPTT_TgtPstnYRi_Mi = 0.0F;
  LDPTV_FTireAccMx_Mps2 = 0.0F;
  LDPTV_FTireAccMn_Mps2 = 0.0F;
  LDPTV_DstcYTgtAreaLf_Mi = 0.0F;
  LDPTV_DstcYTgtAreaRi_Mi = 0.0F;
  LDPTV_FctTgtDistY_Fct = 0.0F;
  LDPTV_TrajPlanServQu_Fct = 0.0F;
  LDPTV_PredTiCurv_Sec = 0.0F;
  LDPTV_PredTiAgl_Sec = 0.0F;
  LDPTV_TiLmtEnDura_Sec = 0.0F;
  LDPTV_JerkLmtMx_Mps3 = 0.0F;
  LDPTV_VeloXObst_Mps = 0.0F;
  LDPTV_AccXObst_Mps2 = 0.0F;
  LDPTV_DstcXObst_Mi = 0.0F;
  LDPTV_DstcYObst_Mi = 0.0F;
  LDPTV_LmtCurvMx_ReMi = 0.0F;
  LDPTV_LmtCurvGradIncMx_ReMps = 0.0F;
  LDPTV_LmtCurvGradDecMx_ReMps = 0.0F;
  LDPTV_SnsTiStamp_Sec = 0.0F;
  LDPTV_SteWhlGradLmt_Fct = 0.0F;
  LDPTV_SteWhlGrad_ReS = 0.0F;
  LDPTV_TrqRampGrad_ReS = 0.0F;
  LDPTV_MxTrqScalGradLmt_Fct = 0.0F;
  LDPTV_MxTrqScalGrad_ReS = 0.0F;
  LDPTV_WeightEndTi_Fct = 0.0F;
  LDPTV_PlanningHorizon_Sec = 0.0F;
  LDPTV_DMCDeraLvl_Fct = 0.0F;
  LDPTV_WidObst_Mi = 0.0F;
  LDPTV_LmtCurvGradCtrlMx_ReMps = 0.0F;
  LDPVSE_NVRAMVehStartupSpd_Kmph = 0.0F;
  LDPSC_CrvSensiDecayRi_Mi = 0.0F;
  LDPDT_CrvThdMaxRi_ReMi = 0.0F;
  LDPDT_CrvThdHystRi_ReMi = 0.0F;
  LDPDT_LnCltdCurvRi_ReMi = 0.0F;
  LDPDT_LnHeadRi_Rad = 0.0F;
  LDPDT_RawLatVehSpdRi_Mps = 0.0F;
  LDPDT_LatVehSpdRi_Mps = 0.0F;
  LDPSC_DstcToLnTrsdCrvCpstnRi_Mi = 0.0F;
  LDPSC_DstcToLnTrsdRi_Mi = 0.0F;
  LDPDT_LnPstnRi_Mi = 0.0F;
  LDPDT_RawDstcToLnRi_Mi = 0.0F;
  LDPDT_DstcToLnRi_Mi = 0.0F;
  LDPDT_TiToLnRi_Sec = 0.0F;
  LDPSC_TiToLnTrsd_Sec = 0.0F;
  LDPVSE_MaxLatVel_Mps = 0.0F;
  LDPDT_CrvThdMaxLf_ReMi = 0.0F;
  LDPDT_CrvThdHystLf_ReMi = 0.0F;
  LDPDT_LnCltdCurvLf_ReMi = 0.0F;
  LDPDT_LnHeadLf_Rad = 0.0F;
  LDPDT_LnPstnLf_Mi = 0.0F;
  LDPDT_RawDstcToLnLf_Mi = 0.0F;
  LDPDT_DstcToLnLf_Mi = 0.0F;
  LDPSC_DstcToLnTrsdCrvCpstnLf_Mi = 0.0F;
  LDPDT_RawLatVehSpdLf_Mps = 0.0F;
  LDPDT_LatVehSpdLf_Mps = 0.0F;
  LDPSC_CrvSensiDecayLe_Mi = 0.0F;
  LDPSC_DstcToLnTrsdLf_Mi = 0.0F;
  LDPDT_TiToLnLf_Sec = 0.0F;
  LDPVSE_MaxCrvBySpd_ReMi = 0.0F;
  LDPVSE_HystCrvBySpd_ReMi = 0.0F;
  LDPSC_RampoutTime_Sec = 0.0F;
  LDPSC_WRBlockTime_Sec = 0.0F;
  LDPTT_RawLnBdryPstnYLf_Mi = 0.0F;
  LDPTT_RawLnBdryPstnYRi_Mi = 0.0F;
  LDPTT_TgtLatDstcRi_Mi = 0.0F;
  LDPTT_TgtLatDstcLf_Mi = 0.0F;
  LDPTT_RawBdryPstnYCent_Mi = 0.0F;
  LDPTV_LatVel_Mps = 0.0F;
  LDPSC_DlcThdMode2_Mi = 0.0F;
  LDPSC_DlcThdMode1_Mi = 0.0F;
  LDPSC_DlcThdMode3_Mi = 0.0F;
  LDPSC_DstcToLnTrsd_Mi = 0.0F;
  LDPSC_SuppValid_Debug = ((uint16_T)0U);
  LDPSC_DgrSide_St = ((uint8_T)0U);
  LDPTV_TrajCtrlSt_St = ((uint8_T)0U);
  LDPDT_CurveTypeRi_St = ((uint8_T)0U);
  LDPVSE_SidCdtnLDPRi_St = ((uint8_T)0U);
  LDPDT_CurveTypeLe_St = ((uint8_T)0U);
  LDPVSE_SidCdtnLDPLf_St = ((uint8_T)0U);
  LDPVSE_IvldLDP_St = ((uint8_T)0U);
  LDPSC_RdyToTrig_B = false;
  LDPTV_HighStatReq_B = false;
  LDPTV_LatCpstnEn_B = false;
  LDPTV_LmtEn_B = false;
  LDPTV_TrigReplan_B = false;
  LDPSC_NVRAMLDPSwitch_B = false;
  LDPDT_RdyTrigLDP_B = false;
  LDPDT_EnaSafety_B = false;
  LDPDT_EnaByInVldQlfrRi_B = false;
  LDPDT_EnaByInVldQlfrSfRi_B = false;
  LDPDT_LnTrigVldRi_B = false;
  LDPDT_CclByInVldQlfrRi_B = false;
  LDPDT_LnCclVldRi_B = false;
  LDPDT_LnMakVldRi_B = false;
  LDPSC_RawTrigByDlcRi_B = false;
  LDPSC_EnaTlcTrigRi_B = false;
  LDPSC_RawTrigByTlcRi_B = false;
  LDPSC_DlyTrigByTlcRi_B = false;
  LDPSC_EnaLdwTrigRi_B = false;
  LDPSC_RstLdwTrigRi_B = false;
  LDPSC_HoldLdwTrigRi_B = false;
  LDPSC_ResetForSafeRi_B = false;
  LDPSC_SetForSafeRi_B = false;
  LDPSC_SetForContinTrigRi_B = false;
  LDPSC_ResetForContinTrigRi_B = false;
  LDPVSE_EdgeRiseTurnSglRi_B = false;
  LDPVSE_TrnSglRi_B = false;
  LDPVSE_RdyTrigLDW_B = false;
  LDPVSE_VehLatSpdVldRi_B = false;
  LDPSC_TrigBySideCondRi_B = false;
  LDPSC_TrigByPrjSpecRi_B = false;
  LDPSC_TrigRi_B = false;
  LDPDT_EnaByCstruSiteLf_B = false;
  LDPDT_EnaByInVldQlfrLf_B = false;
  LDPDT_EnaByInVldQlfrSfLf_B = false;
  LDPDT_LnTrigVldLf_B = false;
  LDPDT_CclByInVldQlfrLf_B = false;
  LDPDT_LnCclVldLf_B = false;
  LDPDT_LnMakVldLf_B = false;
  LDPSC_RawTrigByDlcLf_B = false;
  LDPSC_EnaTlcTrigLf_B = false;
  LDPSC_RawTrigByTlcLf_B = false;
  LDPSC_DlyTrigByTlcLf_B = false;
  LDPSC_EnaLdwTrigLf_B = false;
  LDPSC_RstLdwTrigLf_B = false;
  LDPSC_HoldLdwTrigLf_B = false;
  LDPSC_ResetForSafeLf_B = false;
  LDPSC_SetForSafeLf_B = false;
  LDPSC_ResetForContinTrigLf_B = false;
  LDPSC_SetForContinTrigLf_B = false;
  LDPVSE_EdgeRiseTurnSglLf_B = false;
  LDPVSE_TrnSglLf_B = false;
  LDPVSE_VehLatSpdVldLf_B = false;
  LDPSC_TrigBySideCondLf_B = false;
  LDPSC_TrigByPrjSpecLf_B = false;
  LDPSC_TrigLf_B = false;
  LDPSC_EnaDgrSide_B = false;
  LDPSC_FnsByDgrStLf_B = false;
  LDPSC_FnsByLatDistLf_B = false;
  LDPSC_FnsByHeadingLf_B = false;
  LDPSC_FnsByLatSpdLf_B = false;
  LDPSC_DgrFnsLf_B = false;
  LDPSC_FnsByDgrStRi_B = false;
  LDPSC_FnsByLatDistRi_B = false;
  LDPSC_FnsByHeadingRi_B = false;
  LDPSC_FnsByLatSpdRi_B = false;
  LDPSC_DgrFnsRi_B = false;
  LDPSC_MinLdwBySysSt_B = false;
  LDPSC_EdgeRiseForMinLdw_B = false;
  LDPSC_HoldForMinLdw_B = false;
  LDPSC_FlagMinTimeLDW_B = false;
  LDPSC_DgrFns_B = false;
  LDPSC_CancelBySpecific_B = false;
  LDPSC_CancelByVehSt_B = false;
  LDPSC_CancelByDrvSt_B = false;
  LDPSC_CancelByCtrlSt_B = false;
  LDPSC_CancelBySysSt_B = false;
  LDPSC_CancelByAvlSt_B = false;
  LDPSC_CancelByPrjSpec_B = false;
  LDPSC_MaxDurationBySysSt_B = false;
  LDPSC_EdgRiseForSysSt_B = false;
  LDPSC_MaxDurationByStDly_B = false;
  LDPSC_TiWarmMx_B = false;
  LDPSC_ErrSideByTrigLf_B = false;
  LDPSC_ErrSideBySideCondLf_B = false;
  LDPSC_ErrSidByPrjSpecLf_B = false;
  LDPSC_ErrSidCdtnLf_B = false;
  LDPSC_SideCondByDgrLf_B = false;
  LDPSC_CanelBySideLf_B = false;
  LDPSC_SideCondByDgrRi_B = false;
  LDPSC_ErrSideByTrigRi_B = false;
  LDPSC_ErrSideBySideCondRi_B = false;
  LDPSC_ErrSidByPrjSpecRi_B = false;
  LDPSC_ErrSidCdtnRi_B = false;
  LDPSC_CanelBySideRi_B = false;
  LDPSC_ErrSidCdtn_B = false;
  LDPSC_CLatDevByDlcLf_B = false;
  LDPSC_CLatDevByDgrLf_B = false;
  LDPSC_CclLatDevLf_B = false;
  LDPSC_CLatDevByDlcRi_B = false;
  LDPSC_CLatDevByDgrRi_B = false;
  LDPSC_CclLatDevRi_B = false;
  LDPSC_CclLatDev_B = false;
  LDPSC_Cancel_B = false;
  LDPSC_AbortBySpecific_B = false;
  LDPSC_AbortByVehSt_B = false;
  LDPSC_AbortByDrvSt_B = false;
  LDPSC_AbortByCtrlSt_B = false;
  LDPSC_AbortBySysSt_B = false;
  LDPSC_AbortByAvlSt_B = false;
  LDPSC_AbortByPrjSpec_B = false;
  LDPSC_Abort_B = false;
  LDPSC_StrgRdy_B = false;
  LDPSC_Degradation_B = false;
  LDPSC_DegradationEdgeRise_B = false;
  LDPSC_Degr_B = false;
  LDPSC_SuppBySpecific_B = false;
  LDPSC_SuppByVehSt_B = false;
  LDPSC_SuppByDrvSt_B = false;
  LDPSC_SuppByCtrlSt_B = false;
  LDPSC_SuppBySysSt_B = false;
  LDPSC_SuppyByAvlSt_B = false;
  LDPSC_SuppPrjSpec_B = false;
  LDPSC_Suppresion_B = false;
  LDPSC_WeakRdyBySpecific_B = false;
  LDPSC_WeakRdyByVehSt_B = false;
  LDPSC_WeakRdyByDrvSt_B = false;
  LDPSC_WeakRdyByCtrlSt_B = false;
  LDPSC_WeakRdyBySysSt_B = false;
  LDPSC_WeakRdyByAvlSt_B = false;
  LDPSC_WeakRdyByPrjSpec_B = false;
  LDPSC_WkRdy_B = false;
  LDPSC_BlockTimeBySysOut_B = false;
  LDPSC_RawBlockTimeByRampOut_B = false;
  LDPSC_BlockTimeByRampOut_B = false;
  LDPSC_BlockTime_B = false;
  LDPSC_Suppression_B = false;
  LDPVSE_TgtCntrByLnWidth_B = false;
  LDPVSE_TgtCntrLnEn_B = false;
  LDPTV_CurvInner_B = false;
  LDPTV_CurvOuter_B = false;
  LDPSC_Trig_B = false;
  LDPSC_SysOut_St = E_LDPState_nu_OFF;

  /* states (dwork) */
  (void) memset((void *)&LDPSA_DW, 0,
                sizeof(DW_LDPSA_T));

  /* exported global states */
  LDPSC_DlyTiOfTiToLnRiMn_Sec = 0.0F;
  LDPSC_HdTiTrigRi_Sec = 0.0F;
  LDPSC_ContinWarmTimesOldRi_Count = 0.0F;
  LDPSC_SuppTimeOldRi_Sec = 0.0F;
  LDPVSE_HodTiTrnSglRi_Sec = 0.0F;
  LDPSC_DlyTiOfTiToLnLfMn_Sec = 0.0F;
  LDPSC_HdTiTrigLf_Sec = 0.0F;
  LDPSC_ContinWarmTimesOldLf_Count = 0.0F;
  LDPSC_SuppTimeOldLf_Sec = 0.0F;
  LDPVSE_HodTiTrnSglLf_Sec = 0.0F;
  LDPSC_HdTiWarming_Sec = 0.0F;
  LDPSC_DlyTiTgtFns_Sec = 0.0F;
  LDPSC_HdTiWarmMx_Sec = 0.0F;
  LDPSC_HdTiDegr_Sec = 0.0F;
  LDPSC_HdTiFns_Sec = 0.0F;
  LDPSC_ActiveStopWatch_Ri_sec = 0.0F;
  LDPSC_HdTiFns_Ri_Sec = 0.0F;
  LDPTT_LwLnBdryPstnXLf_Mi = 0.0F;
  LDPTT_LstLnBdryPstnXLf_Mi = 0.0F;
  LDPTT_LstLnBdryVldLengLf_Mi = 0.0F;
  LDPTT_LwLnBdryVldLengLf_Mi = 0.0F;
  LDPTT_LstLnBdryPstnYLf_Mi = 0.0F;
  LDPTT_LstLnWidCalc_Mi = 0.0F;
  LDPTT_LwLnBdryPstnYLf_Mi = 0.0F;
  LDPTT_LwLnBdryPstnYRi_Mi = 0.0F;
  LDPTT_LwLnBdryPstnXRi_Mi = 0.0F;
  LDPTT_LwLnBdryHeadAglRi_Rad = 0.0F;
  LDPTT_LwLnBdryCurvRi_ReMi = 0.0F;
  LDPTT_LwLnBdryCurvRateRi_ReMi2 = 0.0F;
  LDPTT_LwLnBdryVldLengRi_Mi = 0.0F;
  LDPTT_LstTgtLatDstcRi_Mi = 0.0F;
  LDPTT_LstMxTgtLatDevRi_Mi = 0.0F;
  LDPTT_LstTgtLatDevRi_Mi = 0.0F;
  LDPTT_LstTgtLatDevLf_Mi = 0.0F;
  LDPTT_LstTgtLatDstcLf_Mi = 0.0F;
  LDPTT_LstMxTgtLatDevLf_Mi = 0.0F;
  LDPTT_LwLnBdryPstnYCent_Mi = 0.0F;
  LDPTT_LstLnBdryPstnYCent_Mi = 0.0F;
  LDPTT_LstLnBdryCurvCent_ReMi = 0.0F;
  LDPTT_LwLnBdryCurvCent_ReMi = 0.0F;
  LDPTV_LstPlanningHorizon_Sec = 0.0F;
  LDPTV_LstWeightEndTi_Fct = 0.0F;
  LDPTT_LwLnBdryHeadAglCent_Rad = 0.0F;
  LDPTT_LstLnBdryHeadAglCent_Rad = 0.0F;
  LDPTT_LwLnBdryCurvRateCent_ReMi2 = 0.0F;
  LDPTT_LstLnBdryCurvRateCent_ReMi2 = 0.0F;
  LDPTT_LwLnBdryPstnXCent_Mi = 0.0F;
  LDPTT_LstLnBdryPstnXCent_Mi = 0.0F;
  LDPTT_LwLnBdryVldLengCent_Mi = 0.0F;
  LDPTT_LstLnBdryVldLengCent_Mi = 0.0F;
  LDPTV_LstSteWhlGrad_ReS = 0.0F;
  LDPTV_LstDMCDeraLvl_Fct = 0.0F;
  LDPTT_LstLnBdryHeadAglLf_Rad = 0.0F;
  LDPTT_LwLnBdryHeadAglLf_Rad = 0.0F;
  LDPTT_LstLnBdryCurvLf_ReMi = 0.0F;
  LDPTT_LwLnBdryCurvLf_ReMi = 0.0F;
  LDPTT_LstLnBdryCurvRateLf_ReMi2 = 0.0F;
  LDPTT_LwLnBdryCurvRateLf_ReMi2 = 0.0F;
  LDPSC_LstPrevDgrSide_St = 0U;
  LDPSC_DgrSideOld_St = 0U;
  LDPDT_UHysCltdCurvVldRi_B = false;
  LDPDT_BHysHeadAglTrigVldRi_B = false;
  LDPDT_UHysHeadAglCclVldRi_B = false;
  LDPSC_HdTiTrigRiEn_B = false;
  LDPSC_DisTrigRi_B = false;
  LDPSC_SuppFlagOldRi_B = false;
  LDPSC_PreActiveEdgeRi = false;
  LDPSC_ContinTrigRiEn_B = false;
  LDPSC_DisContinTrigRi_B = false;
  LDPVSE_EdgeRisTrnSglLf_B = false;
  LDPVSE_BHysLatVehSpdVldRi_B = false;
  LDPVSE_UHysLatVehSpdVldRi_B = false;
  LDPDT_UHysCltdCurvVldLf_B = false;
  LDPDT_BHysHeadAglTrigVldLf_B = false;
  LDPDT_UHysHeadAglCclVldLf_B = false;
  LDPSC_HdTiTrigLfEn_B = false;
  LDPSC_DisTrigLf_B = false;
  LDPSC_SuppFlagOldLf_B = false;
  LDPSC_PreActiveEdgeLf = false;
  LDPSC_ContinTrigLfEn_B = false;
  LDPSC_DisContinTrigLf_B = false;
  LDPVSE_EdgeRisTrnSglRi_B = false;
  LDPVSE_BHysLatVehSpdVldLf_B = false;
  LDPVSE_UHysLatVehSpdVldLf_B = false;
  LDPSC_EdgeRisWarming_B = false;
  LDPVSE_BHysSpdVeh_B = false;
  LDPVSE_UHysSteAgl_B = false;
  LDPVSE_UHysSteAglSpd_B = false;
  LDPVSE_BHysAccVehX_B = false;
  LDPVSE_BHysAccVehY_B = false;
  LDPVSE_UHysVehCurv_B = false;
  LDPVSE_BHysLnWid_B = false;
  LDPSC_EdgeRisWarmMx_B = false;
  LDPSC_EdgeRisDegr_B = false;
  LDPSC_DegrOld_B = false;
  LDPSC_EdgeRisFns_B = false;
  LDPSC_EdgeRisActive_Ri_B = false;
  LDPSC_EdgeRisFns_Ri_B = false;
  LDPTT_CtrlIniEn_B = false;
  LDPTT_LstControl_B = false;
  LDPTV_LstCtrl_St = false;
  LDPSC_SysOld_St = E_LDPState_nu_OFF;

  /* custom states */
  LDPSC_ActiveStopWatch_sec = 0.0F;
  LDPSC_SafeFuncActiveTurnOnDelay_sec = 0.0F;
  LDPSC_SafeFuncErrorTurnOnDelay_sec = 0.0F;
  LDPSC_SusTimeExpiredTimerRetrigger_sec = 0.0F;
  LDPTT_LstLnBdryPstnYRi_Mi = 0.0F;
  LDPTT_LstLnBdryPstnXRi_Mi = 0.0F;
  LDPTT_LstLnBdryHeadAglRi_Rad = 0.0F;
  LDPTT_LstLnBdryCurvRi_ReMi = 0.0F;
  LDPTT_LstLnBdryCurvRateRi_ReMi2 = 0.0F;
  LDPTT_LstLnBdryVldLengRi_Mi = 0.0F;
  LDPVSE_PrevVehStartupSpd_Kmph = 0U;
  LDPDT_LnLengthRi_B = false;
  LDPDT_LnLengthLf_B = false;
  LDPSC_EdgeRisActive_B = false;
  LDPSC_VehYawRateHyst_bool = false;
  LDPSC_PrevSwitchUnitDelay_bool = false;
  LDPSC_PrevStandbyUnitDelay_bool = false;
  LDPSC_RampTimeExpiredRSFF_bool = false;

  /* SystemInitialize for S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/LDP'
   */
  /* InitializeConditions for UnitDelay: '<S95>/Unit Delay' */
  LDPSC_HdTiTrigRiEn_B = true;

  /* InitializeConditions for UnitDelay: '<S98>/Unit Delay' */
  LDPSC_PreActiveEdgeRi = true;

  /* InitializeConditions for UnitDelay: '<S99>/Unit Delay' */
  LDPSC_ContinTrigRiEn_B = true;

  /* InitializeConditions for UnitDelay: '<S174>/Unit Delay' */
  LDPVSE_EdgeRisTrnSglLf_B = true;

  /* InitializeConditions for UnitDelay: '<S83>/Unit Delay' */
  LDPSC_HdTiTrigLfEn_B = true;

  /* InitializeConditions for UnitDelay: '<S86>/Unit Delay' */
  LDPSC_PreActiveEdgeLf = true;

  /* InitializeConditions for UnitDelay: '<S87>/Unit Delay' */
  LDPSC_ContinTrigLfEn_B = true;

  /* InitializeConditions for UnitDelay: '<S173>/Unit Delay' */
  LDPVSE_EdgeRisTrnSglRi_B = true;

  /* InitializeConditions for UnitDelay: '<S60>/Unit Delay' */
  LDPSC_EdgeRisWarming_B = true;

  /* InitializeConditions for UnitDelay: '<S41>/Unit Delay' */
  LDPSC_EdgeRisWarmMx_B = true;

  /* InitializeConditions for UnitDelay: '<S58>/Unit Delay' */
  LDPSC_EdgeRisDegr_B = true;

  /* InitializeConditions for UnitDelay: '<S46>/Unit Delay' */
  LDPSC_EdgeRisActive_B = true;

  /* InitializeConditions for UnitDelay: '<S47>/Unit Delay' */
  LDPSC_EdgeRisActive_Ri_B = true;

  /* InitializeConditions for UnitDelay: '<S122>/Unit Delay' */
  LDPTT_CtrlIniEn_B = true;

  /* SystemInitialize for Chart: '<S4>/LDP_State' */
  LDPSA_DW.is_LDP_ON = LDPSA_IN_NO_ACTIVE_CHILD;
  LDPSA_DW.is_active_c2_LDPSA = 0U;
  LDPSA_DW.is_c2_LDPSA = LDPSA_IN_NO_ACTIVE_CHILD;

  /* End of SystemInitialize for S-Function (fcgen): '<S1>/Function-Call Generator' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define DLMU2_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */