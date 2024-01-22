/**********************************Model Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : LDWSA
 *
 * Model Long Name     : Lane Departure Warning

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
 * File                             : LDWSA.c
 *
 * FileType                         : Code Source File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Sun Jan 15 13:14:35 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#include "LDWSA.h"
#include "LDWSA_private.h"
#include "look1_iflf_binlxpw.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
/* Exported data definition */
#define CAL_START_CODE
#include "Mem_Map.h"
/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile boolean_T LDDT_CstruSiteLDW_C_B = 0;/* Referenced by:
                                                    * '<S10>/V_Parameter7'
                                                    * '<S11>/V_Parameter7'
                                                    */

/* Switch of consturction side */
const volatile uint8_T LDDT_CurveInner_C_St = 1U;/* Referenced by:
                                                  * '<S9>/V_Parameter11'
                                                  * '<S9>/V_Parameter2'
                                                  * '<S106>/V_Parameter2'
                                                  * '<S106>/V_Parameter3'
                                                  */

/* Constant of inner curve lane */
const volatile uint8_T LDDT_CurveNone_C_St = 0U;/* Referenced by:
                                                 * '<S9>/V_Parameter13'
                                                 * '<S9>/V_Parameter5'
                                                 */

/* Constant of Straight lane */
const volatile uint8_T LDDT_CurveOuter_C_St = 2U;/* Referenced by:
                                                  * '<S9>/V_Parameter12'
                                                  * '<S9>/V_Parameter4'
                                                  * '<S106>/V_Parameter4'
                                                  * '<S106>/V_Parameter5'
                                                  */

/* Constant of Outer curve lane */
const volatile real32_T LDDT_CurveThd_C_St = 0.0F;/* Referenced by: '<S9>/V_Parameter1' */

/* Curve threshold of lane */
const volatile uint16_T LDDT_LnIvldCclLf_C_St = 15U;
                                      /* Referenced by: '<S10>/V_Parameter13' */

/* Invalid cancel state of left lane
   ctrl for 4095
   safety for 15
 */
const volatile uint16_T LDDT_LnIvldCclRi_C_St = 15U;/* Referenced by: '<S11>/V_Parameter2' */

/* Invalid cancel state of right lane
   ctrl for 4095
   safety for 15 */
const volatile uint16_T LDDT_LnIvldLf_C_St = 15U;/* Referenced by: '<S10>/V_Parameter4' */

/* Invalid state of left lane
   ctrl for 20479
   safety for 15 */
const volatile uint16_T LDDT_LnIvldRi_C_St = 15U;/* Referenced by: '<S11>/V_Parameter4' */

/* Invalid state of right lane
   ctrl for 20479
   safety for 15 */
const volatile uint8_T LDDT_LnIvldSfLf_C_St = 15U;/* Referenced by: '<S10>/V_Parameter3' */

/* Invalid safety state of left lane */
const volatile uint8_T LDDT_LnIvldSfRi_C_St = 15U;/* Referenced by: '<S11>/V_Parameter3' */

/* Invalid safety state of right lane */
const volatile uint8_T LDDT_NoDgrSide_C_St = 0U;/* Referenced by: '<S6>/V_Parameter' */

/* State value of no danger of lane */
const volatile boolean_T LDDT_SfFcLDWOn_C_B = 0;/* Referenced by:
                                                 * '<S6>/V_Parameter1'
                                                 * '<S10>/V_Parameter1'
                                                 * '<S11>/V_Parameter1'
                                                 */

/* LDW switch of safety face */
const volatile real32_T LDDT_TLCHeadAglTrsd_C_Rad = 0.0F;/* Referenced by:
                                                          * '<S8>/V_Parameter1'
                                                          * '<S8>/V_Parameter2'
                                                          */

/* Threshold of heading angle at TLC */
const volatile real32_T LDDT_TrsdHeadAglMn_C_Rad = -0.03F;/* Referenced by:
                                                           * '<S10>/V_Parameter11'
                                                           * '<S11>/V_Parameter11'
                                                           */

/* Minimum threshold of heading angle */
const volatile real32_T LDDT_TrsdHeadAglMx_C_Rad = 0.15F;/* Referenced by:
                                                          * '<S10>/V_Parameter10'
                                                          * '<S10>/V_Parameter8'
                                                          * '<S11>/V_Parameter10'
                                                          * '<S11>/V_Parameter8'
                                                          */

/* Maximum threshold of heading angle */
const volatile real32_T LDDT_TrsdHeadAglOfst_C_Rad = 0.002F;/* Referenced by:
                                                             * '<S10>/V_Parameter12'
                                                             * '<S10>/V_Parameter2'
                                                             * '<S11>/V_Parameter12'
                                                             * '<S11>/V_Parameter5'
                                                             */

/* Offset of heading angle threshold */
const volatile real32_T LDDT_TrsdLnCltdCurvLfMx_Cr_Mps[8] = { 0.008F, 0.008F,
  0.007F, 0.007F, 0.006F, 0.006F, 0.005F, 0.005F } ;
                                   /* Referenced by: '<S10>/1-D Lookup Table' */

/* Curve of maximum threshold of left clothiod curvature */
const volatile real32_T LDDT_TrsdLnCltdCurvLfOfst_Cr_Mps[8] = { 0.001F, 0.001F,
  0.001F, 0.001F, 0.001F, 0.001F, 0.001F, 0.001F } ;
                                  /* Referenced by: '<S10>/1-D Lookup Table1' */

/* Curve of offset threshold of left clothiod curvature */
const volatile real32_T LDDT_TrsdLnCltdCurvRiMx_Cr_Mps[8] = { 0.008F, 0.008F,
  0.007F, 0.007F, 0.006F, 0.006F, 0.005F, 0.005F } ;
                                   /* Referenced by: '<S11>/1-D Lookup Table' */

/* Curve of maximum threshold of rifht clothiod curvature */
const volatile real32_T LDDT_TrsdLnCltdCurvRiOfst_Cr_Mps[8] = { 0.001F, 0.001F,
  0.001F, 0.001F, 0.001F, 0.001F, 0.001F, 0.001F } ;
                                  /* Referenced by: '<S11>/1-D Lookup Table1' */

/* Curve of offset threshold of right clothiod curvature */
const volatile real32_T LDDT_VehSpdX_BX_Mps[8] = { 0.0F, 8.33F, 16.66F, 25.0F,
  33.33F, 41.66F, 50.0F, 58.33F } ;    /* Referenced by:
                                        * '<S10>/1-D Lookup Table'
                                        * '<S10>/1-D Lookup Table1'
                                        * '<S11>/1-D Lookup Table'
                                        * '<S11>/1-D Lookup Table1'
                                        */

/* Breakpoint of vehicle speed */
const volatile real32_T LDVSE_HodTiTrnSgl_C_Sec = 5.0F;/* Referenced by:
                                                        * '<S21>/V_Parameter3'
                                                        * '<S21>/V_Parameter7'
                                                        */

/* Value of turn signal holding time */
const volatile real32_T LDVSE_LnWidTrsdMn_C_Mi = 2.5F;/* Referenced by: '<S20>/V_Parameter8' */

/* Minimum threshold of lane width */
const volatile real32_T LDVSE_LnWidTrsdMx_C_Mi = 6.5F;/* Referenced by: '<S20>/V_Parameter7' */

/* Maximum threshold of lane width */
const volatile real32_T LDVSE_LnWidTrsdOfst_C_Mi = 0.1F;/* Referenced by: '<S20>/V_Parameter9' */

/* Offset of lane width */
const volatile uint8_T LDVSE_NoDgrSide_C_St = 0U;/* Referenced by: '<S22>/V_Parameter' */

/* State value of no danger of lane */
const volatile real32_T LDVSE_SteAglSpdTrsdMx_C_Dgpm = 120.0F;/* Referenced by: '<S20>/V_Parameter5' */

/* Maximum threshold of steering wheel angle speed */
const volatile real32_T LDVSE_SteAglSpdTrsdOfst_C_Dgpm = 20.0F;/* Referenced by: '<S20>/V_Parameter6' */

/* Offset of steering wheel angle speed */
const volatile real32_T LDVSE_SteAglTrsdMx_C_Dgr = 90.0F;/* Referenced by: '<S20>/V_Parameter3' */

/* Maximum threshold of steering wheel angle */
const volatile real32_T LDVSE_SteAglTrsdOfst_C_Dgr = 10.0F;/* Referenced by: '<S20>/V_Parameter4' */

/* Offset of steering wheel angle */
const volatile uint8_T LDVSE_TrnSglLf_C_St = 1U;/* Referenced by:
                                                 * '<S21>/V_Parameter1'
                                                 * '<S21>/V_Parameter4'
                                                 */

/* State value of left turn signal  */
const volatile uint8_T LDVSE_TrnSglRi_C_St = 2U;/* Referenced by:
                                                 * '<S21>/V_Parameter'
                                                 * '<S21>/V_Parameter5'
                                                 */

/* State value of right turn signal  */
const volatile boolean_T LDVSE_TrnSglRstLfEn_C_B = 1;/* Referenced by: '<S21>/V_Parameter2' */

/* Enable signal of left turn reset */
const volatile boolean_T LDVSE_TrnSglRstRiEn_C_B = 1;/* Referenced by: '<S21>/V_Parameter6' */

/* Enable signal of right turn reset */
const volatile real32_T LDVSE_TrsdLnCltdCurvMx_Cr_Mps[8] = { 0.01F, 0.01F, 0.01F,
  0.008F, 0.005F, 0.004F, 0.004F, 0.004F } ;/* Referenced by: '<S20>/Lookup Table' */

/* Curve of maximum threshold of clothiod curvature  */
const volatile real32_T LDVSE_TrsdLnCltdCurvOfst_Cr_Mps[8] = { 0.005F, 0.005F,
  0.005F, 0.005F, 0.005F, 0.005F, 0.005F, 0.005F } ;
                                      /* Referenced by: '<S20>/Lookup Table1' */

/* Curve of offset threshold of clothiod curvature  */
const volatile real32_T LDVSE_VehAccSpdTrsdXMn_C_Npkg = -2.95F;
                                      /* Referenced by: '<S20>/V_Parameter11' */

/* Minimum threshold of longitudinal Acceleration */
const volatile real32_T LDVSE_VehAccSpdTrsdXMx_C_Npkg = 2.95F;
                                      /* Referenced by: '<S20>/V_Parameter10' */

/* Maximum threshold of longitudinal Acceleration */
const volatile real32_T LDVSE_VehAccSpdTrsdXOfst_C_Npkg = 0.05F;
                                      /* Referenced by: '<S20>/V_Parameter12' */

/* Offset of longitudinal Acceleration */
const volatile real32_T LDVSE_VehAccSpdTrsdYMx_C_Npkg = 5.0F;
                                      /* Referenced by: '<S20>/V_Parameter13' */

/* Maximum threshold of lateral Acceleration */
const volatile real32_T LDVSE_VehAccSpdTrsdYOfst_C_Npkg = 0.002F;
                                      /* Referenced by: '<S20>/V_Parameter14' */

/* Offset of lateral Acceleration */
const volatile real32_T LDVSE_VehLatTrsdLDWMn_C_Msp = -0.2F;/* Referenced by:
                                                             * '<S37>/V_Parameter4'
                                                             * '<S40>/V_Parameter10'
                                                             */

/* Minimum threshold of Lateral vehicle speed */
const volatile real32_T LDVSE_VehLatTrsdLDWMx_C_Msp = 1.0F;/* Referenced by:
                                                            * '<S37>/V_Parameter1'
                                                            * '<S40>/V_Parameter7'
                                                            */

/* Maximum threshold of Lateral vehicle speed */
const volatile real32_T LDVSE_VehLatTrsdLDWOfst_C_Msp = 0.1F;/* Referenced by:
                                                              * '<S37>/V_Parameter2'
                                                              * '<S37>/V_Parameter5'
                                                              * '<S40>/V_Parameter11'
                                                              * '<S40>/V_Parameter8'
                                                              */

/* Offset of Lateral vehicle speed */
const volatile real32_T LDVSE_VehLatTrsd_Cr_Msp[8] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F } ;           /* Referenced by: '<S22>/Lookup Table' */

/* Curve of maximum threshold of lateral velocity */
const volatile real32_T LDVSE_VehSpdTrsdMn_C_Kmph = 45.0F;/* Referenced by: '<S31>/V_Parameter1' */

/* Minimum threshold of displayed longitudinal speed */
const volatile real32_T LDVSE_VehSpdTrsdMx_C_Kmph = 140.0F;/* Referenced by: '<S20>/V_Parameter' */

/* Maximum threshold of displayed longitudinal speed */
const volatile real32_T LDVSE_VehSpdTrsdOfst_C_Kmph = 3.0F;/* Referenced by: '<S20>/V_Parameter2' */

/* Offset of displayed longitudinal speed */
const volatile real32_T LDVSE_VehSpdX_BX_Mps[8] = { 0.0F, 5.55555534F,
  11.1111107F, 16.666666F, 22.2222214F, 27.7777786F, 33.3333321F, 38.8888893F } ;/* Referenced by:
                                                                      * '<S20>/Lookup Table'
                                                                      * '<S20>/Lookup Table1'
                                                                      * '<S22>/Lookup Table'
                                                                      */

/* Breakpoint of vehicle speed  */
const volatile uint8_T LDWC_AbtDrvActCtrl_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter11' */

/* Abort state of active control */
const volatile uint8_T LDWC_AbtDrvIVld_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter10' */

/* Abort state of invalid driver */
const volatile uint8_T LDWC_AbtErrSpcLDW_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter17' */

/* Abort state of error specific */
const volatile uint8_T LDWC_AbtFctCstm_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter14' */

/* Abort state of customer specific */
const volatile uint8_T LDWC_AbtNoAvlbVehSys_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter13' */

/* Abort state of no availible vehicle system signals */
const volatile uint16_T LDWC_AbtVehIvld_C_St = 0U;/* Referenced by: '<S64>/V_Parameter9' */

/* Abort state of invalid vehicle */
const volatile uint8_T LDWC_AbtVehSysErr_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter12' */

/* Abort state of vehicle system errors */
const volatile uint8_T LDWC_CclDrvActCtrl_C_St = 0U;
                                      /* Referenced by: '<S63>/V_Parameter42' */

/* Cancel state of active control */
const volatile uint8_T LDWC_CclDrvIVld_C_St = 0U;
                                      /* Referenced by: '<S63>/V_Parameter41' */

/* Cancel state of invalid driver */
const volatile uint8_T LDWC_CclErrSpcLDW_C_St = 0U;
                                      /* Referenced by: '<S63>/V_Parameter46' */

/* Cancel state of error specific */
const volatile uint8_T LDWC_CclFctCstm_St = 0U;
                                      /* Referenced by: '<S63>/V_Parameter45' */

/* Cancel state of customer specific */
const volatile uint8_T LDWC_CclNoAvlbVehSys_C_St = 0U;
                                      /* Referenced by: '<S63>/V_Parameter44' */

/* Cancel state of no availible vehicle system signals */
const volatile uint16_T LDWC_CclVehIvld_C_St = 0U;
                                      /* Referenced by: '<S63>/V_Parameter47' */

/* Cancel state of invalid vehicle */
const volatile uint8_T LDWC_CclVehSysErr_C_St = 0U;
                                      /* Referenced by: '<S63>/V_Parameter43' */

/* Cancel state of vehicle system errors */
const volatile real32_T LDWC_ContiActiveTiFns_C_Sec = 4.0F;/* Referenced by:
                                                            * '<S64>/V_Parameter3'
                                                            * '<S64>/V_Parameter6'
                                                            */

/* Maximum time of quit state */
const volatile real32_T LDWC_ContinWarmSupp_C_Sec = 60.0F;/* Referenced by:
                                                           * '<S107>/V_Parameter3'
                                                           * '<S108>/V_Parameter2'
                                                           */

/* the time of continous warning suppression */
const volatile real32_T LDWC_ContinWarmTimes_C_Count = 3.0F;/* Referenced by:
                                                             * '<S107>/V_Parameter4'
                                                             * '<S108>/V_Parameter4'
                                                             */

/* The number of consecutive alarms allowed
 */
const volatile real32_T LDWC_ContinuActiveTi_C_Sec = 2.0F;/* Referenced by:
                                                           * '<S64>/V_Parameter1'
                                                           * '<S64>/V_Parameter4'
                                                           */
const volatile real32_T LDWC_CrvSensiAdvance_BX_ReMi[4] = { 0.002F, 0.004F,
  0.006F, 0.008F } ;             /* Referenced by: '<S106>/1-D Lookup Table9' */

/* Breakpoint of detected right lane curvature */
const volatile real32_T LDWC_CrvSensiAdvance_BY_Mi[4] = { 0.01F, 0.02F, 0.03F,
  0.04F } ;                      /* Referenced by: '<S106>/1-D Lookup Table9' */

/* Breakpoint of detected right lane curvature */
const volatile real32_T LDWC_CrvSensiDecay_BX_ReMi[4] = { 0.002F, 0.004F, 0.006F,
  0.008F } ;                     /* Referenced by: '<S106>/1-D Lookup Table8' */

/* Breakpoint of detected right lane curvature */
const volatile real32_T LDWC_CrvSensiDecay_BY_Mi[4] = { 0.01F, 0.02F, 0.03F,
  0.04F } ;                      /* Referenced by: '<S106>/1-D Lookup Table8' */

/* Breakpoint of detected right lane curvature */
const volatile real32_T LDWC_DTCFctLnWid_Cr_Fct[5] = { 0.5F, 0.6F, 1.0F, 1.0F,
  1.0F } ;                       /* Referenced by: '<S106>/1-D Lookup Table3' */

/* Lane width factor of DTL    */
const volatile real32_T LDWC_DgrCclOfst_C_Mi = 0.8F;/* Referenced by:
                                                     * '<S65>/V_Parameter1'
                                                     * '<S65>/V_Parameter53'
                                                     */

/* Danger offset distance of cancel state */
const volatile real32_T LDWC_DgrFnsHeadAng_C_Rad = 0.005F;/* Referenced by:
                                                           * '<S60>/V_Parameter23'
                                                           * '<S60>/V_Parameter38'
                                                           */

/* Danger of heading angle  */
const volatile real32_T LDWC_DgrFnsOfst_C_Mi = 0.7F;/* Referenced by:
                                                     * '<S60>/V_Parameter19'
                                                     * '<S60>/V_Parameter34'
                                                     */

/* Danger offset distance of finish state */
const volatile real32_T LDWC_DgrFnsSpdVelLat_C_Mps = 0.2F;/* Referenced by:
                                                           * '<S60>/V_Parameter25'
                                                           * '<S60>/V_Parameter40'
                                                           */

/* Danger of lateral speed */
const volatile uint8_T LDWC_DgrSideLf_C_St = 1U;/* Referenced by:
                                                 * '<S60>/V_Parameter17'
                                                 * '<S104>/V_Parameter6'
                                                 * '<S105>/V_Parameter8'
                                                 * '<S136>/V_Parameter32'
                                                 * '<S137>/V_Parameter32'
                                                 * '<S65>/V_Parameter48'
                                                 * '<S66>/V_Parameter56'
                                                 */

/* Constant of left side danger */
const volatile uint8_T LDWC_DgrSideRi_C_St = 2U;/* Referenced by:
                                                 * '<S60>/V_Parameter32'
                                                 * '<S104>/V_Parameter7'
                                                 * '<S136>/V_Parameter2'
                                                 * '<S137>/V_Parameter1'
                                                 * '<S65>/V_Parameter49'
                                                 * '<S66>/V_Parameter59'
                                                 */

/* Constant of right side danger */
const volatile real32_T LDWC_DlyTiFns_C_Sec = 4.0F;/* Referenced by:
                                                    * '<S64>/V_Parameter2'
                                                    * '<S64>/V_Parameter5'
                                                    */

/* Maximum time of quit state */
const volatile real32_T LDWC_DlyTiOfTiToLnMn_C_Sec = 0.2F;/* Referenced by:
                                                           * '<S107>/V_Parameter12'
                                                           * '<S108>/V_Parameter12'
                                                           */

/* Delay time of time to lane crossing */
const volatile real32_T LDWC_DlyTiTgtFns_C_Sec = 1.5F;/* Referenced by: '<S60>/V_Parameter9' */

/* Delay time of target finish  */
const volatile uint8_T LDWC_DrvMod2_C_St = 2U;
                                      /* Referenced by: '<S106>/V_Parameter9' */

/* Driver control mode of LDW ：2 mode */
const volatile uint8_T LDWC_DrvMod3_C_St = 3U;
                                     /* Referenced by: '<S106>/V_Parameter10' */

/* Driver control mode of LDW ：3 mode */
const volatile real32_T LDWC_DstcOfDiscToLnLmtMn_C_Mi = -0.6F;/* Referenced by:
                                                               * '<S107>/V_Parameter1'
                                                               * '<S107>/V_Parameter14'
                                                               * '<S108>/V_Parameter14'
                                                               * '<S108>/V_Parameter3'
                                                               */

/* Minimum distance limiting value of distance to lane crossing */
const volatile real32_T LDWC_DstcOfTiToLnMn_C_Mi = -0.15F;/* Referenced by:
                                                           * '<S107>/V_Parameter10'
                                                           * '<S108>/V_Parameter10'
                                                           */

/* Minimum distance of distance to lane crossing */
const volatile real32_T LDWC_DstcOfstSafeSitu_C_Mi = 0.15F;/* Referenced by:
                                                            * '<S107>/V_Parameter13'
                                                            * '<S108>/V_Parameter13'
                                                            */

/* Offset distance of safe situation */
const volatile real32_T LDWC_DstcToLnTrsdOfstLf_Cr_Mi[17] = { 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F } ;                       /* Referenced by: '<S106>/1-D Lookup Table7' */

/* Curve table of left offset distance to lane crossing threshold      */
const volatile real32_T LDWC_DstcToLnTrsdOfstRi_Cr_Mi[17] = { 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F } ;                       /* Referenced by: '<S106>/1-D Lookup Table4' */

/* Curve table of right offset distance to lane crossing threshold      */
const volatile real32_T LDWC_DstcTrsdVehSpdXDTL1_Cr_Mi[9] = { 0.2F, 0.2F, 0.2F,
  0.2F, 0.2F, 0.2F, 0.2F, 0.2F, 0.2F } ;
                                  /* Referenced by: '<S106>/1-D Lookup Table' */

/* Curve table of distance to lane crossing threshold at mode 1     */
const volatile real32_T LDWC_DstcTrsdVehSpdXDTL2_Cr_Mi[9] = { 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                                 /* Referenced by: '<S106>/1-D Lookup Table2' */

/* Curve table of distance to lane crossing threshold at mode 2     */
const volatile real32_T LDWC_DstcTrsdVehSpdXDTL3_Cr_Mi[9] = { -0.2F, -0.2F,
  -0.2F, -0.2F, -0.2F, -0.2F, -0.2F, -0.2F, -0.2F } ;
                                 /* Referenced by: '<S106>/1-D Lookup Table1' */

/* Curve table of distance to lane crossing threshold at mode 3     */
const volatile uint8_T LDWC_ErrCstmCclLf_C_St = 0U;
                                      /* Referenced by: '<S66>/V_Parameter58' */

/* Cancel state of left customer specific */
const volatile uint8_T LDWC_ErrCstmCclRi_C_St = 0U;
                                      /* Referenced by: '<S66>/V_Parameter61' */

/* Cancel state of right customer specific */
const volatile uint8_T LDWC_FnsCdsnEn_C_St = 191U;/* Referenced by:
                                                   * '<S60>/V_Parameter28'
                                                   * '<S60>/V_Parameter29'
                                                   * '<S60>/V_Parameter30'
                                                   * '<S60>/V_Parameter31'
                                                   * '<S60>/V_Parameter41'
                                                   * '<S60>/V_Parameter42'
                                                   * '<S60>/V_Parameter43'
                                                   */

/* State switch of finish condition  */
const volatile real32_T LDWC_FnsDuraMn_C_Sec = 0.5F;
                                      /* Referenced by: '<S60>/V_Parameter27' */

/* Minimum duration of finish state */
const volatile real32_T LDWC_HdTiTrigLf_C_Sec = 1.0F;/* Referenced by:
                                                      * '<S107>/V_Parameter15'
                                                      * '<S108>/V_Parameter15'
                                                      */

/* Holding time of left warming trigger */
const volatile real32_T LDWC_LDPSensiDecay_C_Mi = 0.05F;
                                      /* Referenced by: '<S106>/V_Parameter1' */

/* LDP sensitivity decay */
const volatile real32_T LDWC_LaneWidth_BX_Mi[5] = { 2.0F, 2.5F, 3.0F, 3.5F, 4.0F
} ;                                    /* Referenced by:
                                        * '<S106>/1-D Lookup Table3'
                                        * '<S106>/1-D Lookup Table6'
                                        */

/* breakpoint of lane width */
const volatile real32_T LDWC_LnDectCrvLf_BX_ReMi[17] = { -0.05F, -0.02F, -0.01F,
  -0.008F, -0.005F, -0.002F, -0.001F, -0.0005F, 0.0F, 0.0005F, 0.001F, 0.002F,
  0.005F, 0.008F, 0.01F, 0.02F, 0.05F } ;
                                 /* Referenced by: '<S106>/1-D Lookup Table7' */

/* Breakpoint of detected left lane curvature */
const volatile real32_T LDWC_LnDectCrvRi_BX_ReMi[17] = { -0.05F, -0.02F, -0.01F,
  -0.008F, -0.005F, -0.002F, -0.001F, -0.0005F, 0.0F, 0.0005F, 0.001F, 0.002F,
  0.005F, 0.008F, 0.01F, 0.02F, 0.05F } ;
                                 /* Referenced by: '<S106>/1-D Lookup Table4' */

/* Breakpoint of detected right lane curvature */
const volatile real32_T LDWC_NoDgrCclOfst_C_Mi = 1.0F;/* Referenced by:
                                                       * '<S65>/V_Parameter51'
                                                       * '<S65>/V_Parameter54'
                                                       */

/* No danger offset distance of cancel state */
const volatile real32_T LDWC_NoDgrFnsHeadAng_C_Rad = 0.005F;/* Referenced by:
                                                             * '<S60>/V_Parameter22'
                                                             * '<S60>/V_Parameter37'
                                                             */

/*  No danger of heading angle      */
const volatile real32_T LDWC_NoDgrFnsOfst_C_Mi = 0.15F;/* Referenced by:
                                                        * '<S60>/V_Parameter21'
                                                        * '<S60>/V_Parameter36'
                                                        */

/* No danger offset distance of finish state */
const volatile real32_T LDWC_NoDgrFnsSpdVelLat_C_Mps = 0.2F;/* Referenced by:
                                                             * '<S60>/V_Parameter24'
                                                             * '<S60>/V_Parameter39'
                                                             */

/* No danger of lateral speed */
const volatile uint8_T LDWC_NoDgrSide_C_St = 0U;/* Referenced by:
                                                 * '<S104>/V_Parameter4'
                                                 * '<S104>/V_Parameter5'
                                                 * '<S105>/V_Parameter5'
                                                 */

/* Constant of no danger */
const volatile uint8_T LDWC_PrjSpecQu_C_St = 0U;/* Referenced by:
                                                 * '<S107>/V_Parameter'
                                                 * '<S108>/V_Parameter'
                                                 */
const volatile real32_T LDWC_SafetyFuncMaxTime_sec = 0.05F;/* Referenced by:
                                                            * '<S139>/V_Parameter1'
                                                            * '<S140>/V_Parameter1'
                                                            */

/* safety function active or error maximum */
const volatile uint8_T LDWC_SidCdtnCclLf_C_St = 1U;
                                      /* Referenced by: '<S66>/V_Parameter57' */

/* Cancel constant of left side condition */
const volatile uint8_T LDWC_SidCdtnCclRi_C_St = 1U;
                                      /* Referenced by: '<S66>/V_Parameter60' */

/* Cancel constant of right side condition */
const volatile uint8_T LDWC_StrgRdyDrvActCtrl_C_St = 45U;
                                      /* Referenced by: '<S64>/V_Parameter16' */

/* Strong ready state of active control */
const volatile uint8_T LDWC_StrgRdyDrvIVld_C_St = 36U;
                                      /* Referenced by: '<S64>/V_Parameter15' */

/* Strong ready state of invalid driver */
const volatile uint8_T LDWC_StrgRdyErrSpcLDW_C_St = 250U;
                                      /* Referenced by: '<S64>/V_Parameter21' */

/* Strong ready state of error specific */
const volatile uint8_T LDWC_StrgRdyFctCstm_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter20' */

/* Strong ready state of customer specific */
const volatile uint8_T LDWC_StrgRdyNoAvlbVehSys_C_St = 15U;
                                      /* Referenced by: '<S64>/V_Parameter19' */

/* Strong ready state of no availible vehicle system signals */
const volatile uint16_T LDWC_StrgRdyVehIvld_C_St = 2047U;
                                      /* Referenced by: '<S64>/V_Parameter22' */

/* Strong ready state of invalid vehicle */
const volatile uint8_T LDWC_StrgRdyVehSysErr_C_St = 11U;
                                      /* Referenced by: '<S64>/V_Parameter18' */

/* Strong ready state of vehicle system errors */
const volatile uint8_T LDWC_SuppDrvActCtrl_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter24' */

/* Suppresion state of active control */
const volatile uint8_T LDWC_SuppDrvIvld_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter23' */

/* Suppresion state of invalid driver */
const volatile uint8_T LDWC_SuppErrSpcLDW_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter28' */

/* Suppresion state of error specific */
const volatile uint8_T LDWC_SuppFctCstm_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter27' */

/* Suppresion state of customer specific */
const volatile uint8_T LDWC_SuppNoAvlbVehSys_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter26' */

/* Suppresion state of no availible vehicle system signals */
const volatile uint16_T LDWC_SuppVehIvld_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter29' */

/* Suppresion state of invalid vehicle */
const volatile uint8_T LDWC_SuppVehSysErr_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter25' */

/* Suppresion state of vehicle system errors */
const volatile uint8_T LDWC_SuppVehicleInvalid_C_St = 127U;
                                     /* Referenced by: '<S141>/V_Parameter28' */

/* Suppresion state of vehicle */
const volatile boolean_T LDWC_Switch_C_B = 0;
                                      /* Referenced by: '<S50>/V_Parameter11' */

/* Value of LDW disable switch */
const volatile real32_T LDWC_TgtTrajPstnY_C_Mi = 0.5F;/* Referenced by:
                                                       * '<S60>/V_Parameter18'
                                                       * '<S60>/V_Parameter20'
                                                       * '<S60>/V_Parameter33'
                                                       * '<S60>/V_Parameter35'
                                                       * '<S65>/V_Parameter50'
                                                       * '<S65>/V_Parameter52'
                                                       */

/* Target trajectory lateral position  */
const volatile real32_T LDWC_TiAbtDegr_C_Sec = 0.1F;
                                      /* Referenced by: '<S59>/V_Parameter63' */

/* Degradation time of abort state */
const volatile real32_T LDWC_TiCclDegr_C_Sec = 0.1F;/* Referenced by: '<S59>/V_Parameter3' */

/* Degradation time of cancel state */
const volatile real32_T LDWC_TiDgrFnsDegr_C_Sec = 0.1F;/* Referenced by: '<S59>/V_Parameter2' */

/* Degradation time of finish state */
const volatile real32_T LDWC_TiStrgRdyDegr_C_Sec = 0.1F;/* Referenced by: '<S59>/V_Parameter1' */

/* Degradation time of no strong ready state */
const volatile real32_T LDWC_TiToLnTrsdSpd_Cr_Sec[9] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                 /* Referenced by: '<S106>/1-D Lookup Table5' */

/* Map table of time to lane crossing threshold */
const volatile real32_T LDWC_TiToLnTrsdWdh_Cr_Sec[5] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F } ;                       /* Referenced by: '<S106>/1-D Lookup Table6' */

/* Map table of time to lane crossing threshold */
const volatile uint8_T LDWC_TrigCdtnEn_C_St = 5U;/* Referenced by:
                                                  * '<S107>/V_Parameter11'
                                                  * '<S107>/V_Parameter9'
                                                  * '<S108>/V_Parameter11'
                                                  * '<S108>/V_Parameter9'
                                                  */

/* Switch state of choose threshold  */
const volatile real32_T LDWC_VehSpdXDTL_BX_Mps[9] = { 0.0F, 40.0F, 50.0F, 60.0F,
  70.0F, 80.0F, 100.0F, 120.0F, 150.0F } ;/* Referenced by:
                                           * '<S106>/1-D Lookup Table'
                                           * '<S106>/1-D Lookup Table1'
                                           * '<S106>/1-D Lookup Table2'
                                           */

/* DTL breakpoint of vehicle speed */
const volatile real32_T LDWC_VehSpdXTTL_BX_Mps[9] = { 0.0F, 40.0F, 50.0F, 60.0F,
  70.0F, 80.0F, 100.0F, 120.0F, 150.0F } ;
                                 /* Referenced by: '<S106>/1-D Lookup Table5' */

/* TTL breakpoint of vehicle speed */
const volatile real32_T LDWC_VehYawRateHyst_C_rps = 0.05F;/* Referenced by: '<S143>/Constant10' */

/* Vehicle yaw rate hysteresis */
const volatile real32_T LDWC_VehYawRateMax_C_rps = 0.25F;/* Referenced by: '<S143>/Constant9' */

/* Vehicle yaw rate maximum */
const volatile real32_T LDWC_WarmMxTi_C_Sec = 2.0F;/* Referenced by:
                                                    * '<S107>/V_Parameter2'
                                                    * '<S108>/V_Parameter1'
                                                    * '<S67>/V_Parameter63'
                                                    */

/* Maximum time of warming state */
const volatile uint8_T LDWC_WkRdyDrvActCtrl_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter31' */

/* Weak ready state of active control */
const volatile uint8_T LDWC_WkRdyDrvIVld_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter30' */

/* Weak ready state of invalid driver */
const volatile uint8_T LDWC_WkRdyErrSpcLDW_C_St = 5U;
                                      /* Referenced by: '<S64>/V_Parameter35' */

/* Weak ready state of error specific */
const volatile uint8_T LDWC_WkRdyFctCstm_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter34' */

/* Weak ready state of customer specific */
const volatile uint8_T LDWC_WkRdyNoAvlbVehSys_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter33' */

/* Weak ready state of no availible vehicle system signals */
const volatile uint16_T LDWC_WkRdyVehIvld_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter36' */

/* Weak ready state of invalid vehicle */
const volatile uint8_T LDWC_WkRdyVehSysErr_C_St = 0U;
                                      /* Referenced by: '<S64>/V_Parameter32' */

/* Weak ready state of vehicle system errors */
const volatile real32_T VehicleSpeedThresholdHMI_Kph = 80.0F;/* Referenced by:
                                                              * '<S105>/V_Parameter1'
                                                              * '<S105>/V_Parameter3'
                                                              */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
/* Exported data definition */
#define CAL_STOP_CODE
#include "Mem_Map.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
/* Exported data definition */
// #define DLMU2_START_CODE
// #include "Mem_Map.h"

/* Named constants for Chart: '<S5>/LDW_State' */
#define LDWSA_IN_LDW_ACTIVE            ((uint8_T)1U)
#define LDWSA_IN_LDW_ERROR             ((uint8_T)1U)
#define LDWSA_IN_LDW_OFF               ((uint8_T)2U)
#define LDWSA_IN_LDW_ON                ((uint8_T)3U)
#define LDWSA_IN_LDW_PASSIVE           ((uint8_T)2U)
#define LDWSA_IN_LDW_STANDBY           ((uint8_T)3U)
#define LDWSA_IN_NO_ACTIVE_CHILD       ((uint8_T)0U)

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Exported block signals */
real32_T LDVSE_NVRAMVehStartupSpd_kmph;/* '<S1>/LDW' */
real32_T LDWC_CrvSensiDecayRi_Mi;      /* '<S106>/Switch6' */
real32_T LDDT_CrvThdMaxRi_ReMi;        /* '<S11>/1-D Lookup Table'
                                        * Curve of maximum threshold of right clothiod curvature
                                        */
real32_T LDDT_CrvThdHystRi_ReMi;       /* '<S11>/1-D Lookup Table1'
                                        * Curve of offset threshold of right clothiod curvature
                                        */
real32_T LDDT_LnCltdCurvRi_ReMi;       /* '<S6>/Switch5'
                                        * Clothoid curvature of right lane
                                        */
real32_T LDDT_LnHeadRi_Rad;            /* '<S6>/Switch1'
                                        * Heading angle of rifht lane
                                        */
real32_T LDDT_RawLatVehSpdRi_Mps;      /* '<S8>/Product1'
                                        * Raw right lateral vehicle speed
                                        */
real32_T LDDT_LatVehSpdRi_Mps;         /* '<S8>/Switch2'
                                        * right lateral vehicle speed
                                        */
real32_T LDWC_CrrctByLnWidth_Fct;      /* '<S106>/1-D Lookup Table3'
                                        * DLC threshold corretction factor
                                        */
real32_T LDWC_DstcToLnTrsdCrvCpstnRi_Mi;/* '<S106>/1-D Lookup Table4'
                                         * Curvature compensation of threshold distance to right lane crossing
                                         */
real32_T LDWC_DstcToLnTrsdRi_Mi;       /* '<S106>/Add3'
                                        *  Threshold distance to right lane crossing
                                        */
real32_T LDDT_LnPstnRi_Mi;             /* '<S6>/Switch3'
                                        * Position of  of right lane
                                        */
real32_T LDDT_RawDstcToLnRi_Mi;        /* '<S8>/Subtract1'
                                        * Raw Distance of between vehicel and right lane
                                        */
real32_T LDDT_DstcToLnRi_Mi;           /* '<S8>/Switch4'
                                        * Distance of between vehicel and right lane
                                        */
real32_T LDVSE_MaxLatVel_Mps;          /* '<S22>/Lookup Table'
                                        * Maximum lateral velocity
                                        */
real32_T LDDT_CrvThdMaxLf_ReMi;        /* '<S10>/1-D Lookup Table'
                                        * Curve of maximum threshold of left clothiod curvature
                                        */
real32_T LDDT_CrvThdHystLf_ReMi;       /* '<S10>/1-D Lookup Table1'
                                        * Curve of offset threshold of left clothiod curvature
                                        */
real32_T LDDT_LnCltdCurvLf_ReMi;       /* '<S6>/Switch4'
                                        * Clothoid curvature of left lane
                                        */
real32_T LDDT_LnHeadLf_Rad;            /* '<S6>/Switch'
                                        * Heading angle of left lane
                                        */
real32_T LDDT_RawLatVehSpdLf_Mps;      /* '<S8>/Product'
                                        * Raw Left lateral vehicle speed
                                        */
real32_T LDDT_LatVehSpdLf_Mps;         /* '<S8>/Switch1'
                                        * Left lateral vehicle speed
                                        */
real32_T LDVSE_MaxCrvBySpd_ReMi;       /* '<S20>/Lookup Table'
                                        * Maximum curvature for LDW invalid condition
                                        */
real32_T LDVSE_HystCrvBySpd_ReMi;      /* '<S20>/Lookup Table1'
                                        * Curvature hysteresis for LDW invalid condition
                                        */
real32_T LDDT_TiToLnRi_Sec;            /* '<S8>/Switch6'
                                        * Time of vehicle to right lane
                                        */
real32_T LDWC_TiToLnTrsd_Sec;          /* '<S106>/Product1'
                                        * Threshold time of time to lane crossing
                                        */
real32_T LDDT_LnPstnLf_Mi;             /* '<S6>/Switch2'
                                        * Position of  of left lane
                                        */
real32_T LDDT_RawDstcToLnLf_Mi;        /* '<S8>/Subtract'
                                        * Raw Distance of between vehicel and left lane
                                        */
real32_T LDDT_DstcToLnLf_Mi;           /* '<S8>/Switch3'
                                        * Distance of between vehicel and left lane
                                        */
real32_T LDWC_DstcToLnTrsdCrvCpstnLf_Mi;/* '<S106>/1-D Lookup Table7'
                                         * Curvature compensation of threshold distance to left lane crossing
                                         */
real32_T LDWC_CrvSensiDecayLe_Mi;      /* '<S106>/Switch5' */
real32_T LDWC_DstcToLnTrsdLf_Mi;       /* '<S106>/Add2'
                                        *  Threshold distance to left lane crossing
                                        */
real32_T LDDT_TiToLnLf_Sec;            /* '<S8>/Switch5'
                                        * Time of vehicle to left lane
                                        */
real32_T LDWC_WRBlockTime_Sec;         /* '<S64>/Switch' */
real32_T LDWC_DlcThdMode2_Mi;          /* '<S106>/1-D Lookup Table2'
                                        * DLC threshold at LDW mode 2
                                        */
real32_T LDWC_DlcThdMode1_Mi;          /* '<S106>/1-D Lookup Table'
                                        * DLC threshold at LDW mode 1
                                        */
real32_T LDWC_DlcThdMode3_Mi;          /* '<S106>/1-D Lookup Table1'
                                        * DLC threshold at LDW mode 3
                                        */
real32_T LDWC_DstcToLnTrsd_Mi;         /* '<S106>/Product'
                                        *  Threshold distance to  lane crossing
                                        */
uint16_T LDWC_SuppValid_Debug;         /* '<S62>/Signal Conversion8' */
uint8_T LDWC_DgrSide_St;               /* '<S1>/LDW'
                                        * State of danger side
                                        */
uint8_T LDDT_CurveTypeRi_St;           /* '<S9>/Switch2'
                                        * Curve Type of right Lane
                                        */
uint8_T LDVSE_SidCdtnLDWRi_St;         /* '<S22>/Signal Conversion4'
                                        * State of right side at LDW
                                        */
uint8_T LDDT_CurveTypeLe_St;           /* '<S9>/Switch'
                                        * Curve Type of left Lane
                                        */
uint8_T LDVSE_SidCdtnLDWLf_St;         /* '<S22>/Signal Conversion3'
                                        * State of left side at LDW
                                        */
uint8_T LDVSE_IvldLDW_St;              /* '<S20>/Signal Conversion1'
                                        * Invalid state of LDW
                                        */
uint8_T ELDWTriggerDgrForHMI;          /* '<S105>/Switch6'
                                        * for HMI
                                        */
boolean_T LDWC_NVRAMLDWSwitch_B;       /* '<S1>/LDW' */
boolean_T LDWC_RdyToTrig_B;            /* '<S1>/LDW'
                                        * Condition of Ready to trigger state
                                        */
boolean_T LDDT_RdyTrigLDW_B;           /* '<S6>/Equal'
                                        * Condition of ready to trigger LDW state
                                        */
boolean_T LDDT_EnaSafety_B;            /* '<S6>/AND'
                                        * Enable flag for data from the safety interface
                                        */
boolean_T LDDT_EnaByInVldQlfrRi_B;     /* '<S11>/Relational Operator4'
                                        * Enable flag for right lane validity by left lane invalid qualifier
                                        */
boolean_T LDDT_EnaByInVldQlfrSfRi_B;   /* '<S11>/Relational Operator1'
                                        * Enable flag for right lane validity by left lane invalid qualifier for safety interface
                                        */
boolean_T LDDT_LnTrigVldRi_B;          /* '<S11>/Logical Operator2'
                                        * Condition validity of right lane marker at LDW trigger
                                        */
boolean_T LDDT_CclByInVldQlfrRi_B;     /* '<S11>/Relational Operator5'
                                        * Enable flag for right lane validity by left lane invalid qualifier for safety interface when cancel the function
                                        */
boolean_T LDDT_LnCclVldRi_B;           /* '<S11>/Logical Operator5'
                                        * Condition validity of right lane marker at LDW cancel
                                        */
boolean_T LDDT_LnMakVldRi_B;           /* '<S11>/Switch2'
                                        * Condition validity of right lane marker
                                        */
boolean_T LDWC_RawTrigByDlcRi_B;       /* '<S108>/Relational Operator3'
                                        * Raw trigger flag by DLC for right lane
                                        */
boolean_T LDVSE_RdyTrigLDW_B;          /* '<S22>/Equal'
                                        * Ready Trigger flag for LDW
                                        */
boolean_T LDDT_EnaByCstruSiteLf_B;     /* '<S10>/NOT'
                                        * Enable flag for left lane validity by construction site detected
                                        */
boolean_T LDDT_EnaByInVldQlfrLf_B;     /* '<S10>/Relational Operator4'
                                        * Enable flag for left lane validity by left lane invalid qualifier
                                        */
boolean_T LDDT_EnaByInVldQlfrSfLf_B;   /* '<S10>/Relational Operator1'
                                        * Enable flag for left lane validity by left lane invalid qualifier for safety interface
                                        */
boolean_T LDDT_LnTrigVldLf_B;          /* '<S10>/Logical Operator2'
                                        * Condition validity of left lane marker at LDW trigger
                                        */
boolean_T LDDT_CclByInVldQlfrLf_B;     /* '<S10>/Relational Operator5'
                                        * Enable flag for left lane validity by left lane invalid qualifier for safety interface when cancel the function
                                        */
boolean_T LDDT_LnCclVldLf_B;           /* '<S10>/Logical Operator5'
                                        * Condition validity of left lane marker at LDW cancel
                                        */
boolean_T LDDT_LnMakVldLf_B;           /* '<S10>/Switch'
                                        * Condition validity of left lane marker
                                        */
boolean_T LDVSE_VehLatSpdVldLf_B;      /* '<S37>/Switch1'
                                        * Validity of left lateral vehicle speed
                                        */
boolean_T LDVSE_TrnSglLf_B;            /* '<S21>/Signal Conversion3'
                                        * Condition of  left turn signal
                                        */
boolean_T LDVSE_VehLatSpdVldRi_B;      /* '<S40>/Switch1'
                                        * Validity of right lateral vehicle speed
                                        */
boolean_T LDVSE_TrnSglRi_B;            /* '<S21>/Signal Conversion4'
                                        * Condition of  right turn signal
                                        */
boolean_T LDWC_EnaTlcTrigRi_B;         /* '<S108>/Relational Operator1'
                                        * Enable flag for Raw trigger flag by TLC for right lane
                                        */
boolean_T LDWC_RawTrigByTlcRi_B;       /* '<S108>/Relational Operator2'
                                        * Raw trigger flag by TLC for right lane
                                        */
boolean_T LDWC_DlyTrigByTlcRi_B;       /* '<S134>/AND' */
boolean_T LDWC_EnaLdwTrigRi_B;         /* '<S123>/AND'
                                        * Enable flag for LDW function trigger
                                        */
boolean_T LDWC_RstTlcTrigRi_B;         /* '<S108>/Logical Operator15'
                                        * Reset flag for Raw trigger flag by TLC for right lane
                                        */
boolean_T LDWC_ResetForSafeRi_B;       /* '<S108>/Logical Operator4'
                                        * Reset flag for the safe situation condition of right lane
                                        */
boolean_T LDWC_SetForSafeRi_B;         /* '<S108>/Relational Operator6'
                                        * Set flag for the safe situation condition of right lane
                                        */
boolean_T LDWC_SetForContinTrigRi_B;   /* '<S108>/Logical Operator13' */
boolean_T LDWC_ResetForContinTrigRi_B; /* '<S108>/Logical Operator9' */
boolean_T LDWC_TrigBySideCondRi_B;     /* '<S108>/Relational Operator7'
                                        * LDW function trigger flag by  side condition of right lane
                                        */
boolean_T LDWC_TrigByPrjSpecRi_B;      /* '<S108>/Equal'
                                        * LDW function trigger flag by customer projects of right lane
                                        */
boolean_T LDWC_TrigRi_B;               /* '<S108>/Logical Operator3'
                                        * Condition of right trigger
                                        */
boolean_T LDWC_RawTrigByDlcLf_B;       /* '<S107>/Relational Operator3'
                                        * Raw trigger flag by DLC for left lane
                                        */
boolean_T LDWC_EnaTlcTrigLf_B;         /* '<S107>/Relational Operator1'
                                        * Enable flag for Raw trigger flag by TLC for left lane
                                        */
boolean_T LDWC_RawTrigByTlcLf_B;       /* '<S107>/Relational Operator2'
                                        * Raw trigger flag by TLC for left lane
                                        */
boolean_T LDWC_DlyTrigByTlcLf_B;       /* '<S122>/AND' */
boolean_T LDWC_EnaLdwTrigLf_B;         /* '<S111>/AND'
                                        * Enable flag for LDW function trigger
                                        */
boolean_T LDWC_RstTlcTrigLf_B;         /* '<S107>/OR'
                                        * Reset flag for Raw trigger flag by TLC for left lane
                                        */
boolean_T LDWC_ResetForSafeLf_B;       /* '<S107>/Logical Operator4'
                                        * Reset flag for the safe situation condition of left lane
                                        */
boolean_T LDWC_SetForSafeLf_B;         /* '<S107>/Relational Operator6'
                                        * Set flag for the safe situation condition of left lane
                                        */
boolean_T LDWC_ResetForContinTrigLf_B; /* '<S107>/Logical Operator9' */
boolean_T LDWC_SetForContinTrigLf_B;   /* '<S107>/Logical Operator13' */
boolean_T LDWC_TrigBySideCondLf_B;     /* '<S107>/Relational Operator7'
                                        * LDW function trigger flag by  side condition of left lane
                                        */
boolean_T LDWC_TrigByPrjSpecLf_B;      /* '<S107>/Equal'
                                        * LDW function trigger flag by customer projects of left lane
                                        */
boolean_T LDWC_TrigLf_B;               /* '<S107>/Logical Operator3'
                                        * Condition of left trigger
                                        */
boolean_T LDWC_EnaDgrSide_B;           /* '<S104>/Logical Operator6'
                                        * Enable flag for Degerous side state
                                        */
boolean_T LDWC_FnsByDgrStLf_B;         /* '<S60>/Relational Operator7' */
boolean_T LDWC_FnsByLatDistLf_B;       /* '<S60>/Logical Operator8' */
boolean_T LDWC_FnsByHeadingLf_B;       /* '<S60>/Logical Operator9' */
boolean_T LDWC_FnsByLatSpdLf_B;        /* '<S60>/Logical Operator10' */
boolean_T LDWC_DgrFnsLf_B;             /* '<S60>/Logical Operator6' */
boolean_T LDWC_FnsByDgrStRi_B;         /* '<S60>/Relational Operator9' */
boolean_T LDWC_FnsByLatDistRi_B;       /* '<S60>/Logical Operator14' */
boolean_T LDWC_FnsByHeadingRi_B;       /* '<S60>/Logical Operator15' */
boolean_T LDWC_FnsByLatSpdRi_B;        /* '<S60>/Logical Operator13' */
boolean_T LDWC_DgrFnsRi_B;             /* '<S60>/Logical Operator12' */
boolean_T LDWC_MinLdwBySysSt_B;        /* '<S60>/Relational Operator8' */
boolean_T LDWC_EdgeRiseForMinLdw_B;    /* '<S87>/AND' */
boolean_T LDWC_HoldForMinLdw_B;        /* '<S103>/GreaterThan1' */
boolean_T LDWC_FlagMinTimeLDW_B;       /* '<S60>/Logical Operator11' */
boolean_T LDWC_DgrFns_B;               /* '<S96>/AND'
                                        * Condition of danger finish
                                        */
boolean_T LDWC_CancelBySpecific_B;     /* '<S63>/Relational Operator38'
                                        * LDW cancel conditions by LDW specific bitfield
                                        */
boolean_T LDWC_CancelByVehSt_B;        /* '<S63>/Relational Operator37'
                                        * LDW cancel conditions by vehicle state
                                        */
boolean_T LDWC_CancelByDrvSt_B;        /* '<S63>/Relational Operator32'
                                        * LDW cancel conditions by drive state
                                        */
boolean_T LDWC_CancelByCtrlSt_B;       /* '<S63>/Relational Operator33'
                                        * LDW cancel conditions by active control state
                                        */
boolean_T LDWC_CancelBySysSt_B;        /* '<S63>/Relational Operator34'
                                        * LDW cancel conditions by system state
                                        */
boolean_T LDWC_CancelByAvlSt_B;        /* '<S63>/Relational Operator35'
                                        * LDW cancel conditions by no available state
                                        */
boolean_T LDWC_CancelByPrjSpec_B;      /* '<S63>/Relational Operator36'
                                        * LDW cancel conditions by customer projects
                                        */
boolean_T LDWC_MaxDurationBySysSt_B;   /* '<S67>/Relational Operator47' */
boolean_T LDWC_EdgRiseForSysSt_B;      /* '<S68>/AND' */
boolean_T LDWC_MaxDurationByStDly_B;   /* '<S67>/Logical Operator22' */
boolean_T LDWC_TiWarmMx_B;             /* '<S67>/Logical Operator23'
                                        * Condition of warming max time
                                        */
boolean_T LDWC_ErrSideByTrigLf_B;      /* '<S66>/Logical Operator19' */
boolean_T LDWC_ErrSideBySideCondLf_B;  /* '<S66>/Relational Operator42' */
boolean_T LDWC_ErrSidByPrjSpecLf_B;    /* '<S66>/Relational Operator43' */
boolean_T LDWC_ErrSidCdtnLf_B;         /* '<S66>/Logical Operator18'
                                        * Error condition of left side
                                        */
boolean_T LDWC_SideCondByDgrLf_B;      /* '<S66>/Relational Operator41' */
boolean_T LDWC_CanelBySideLf_B;        /* '<S66>/Logical Operator15' */
boolean_T LDWC_SideCondByDgrRi_B;      /* '<S66>/Relational Operator44' */
boolean_T LDWC_ErrSideByTrigRi_B;      /* '<S66>/Logical Operator21' */
boolean_T LDWC_ErrSideBySideCondRi_B;  /* '<S66>/Relational Operator45' */
boolean_T LDWC_ErrSidByPrjSpecRi_B;    /* '<S66>/Relational Operator46' */
boolean_T LDWC_ErrSidCdtnRi_B;         /* '<S66>/Logical Operator2'
                                        * Error condition of right side
                                        */
boolean_T LDWC_CanelBySideRi_B;        /* '<S66>/Logical Operator1' */
boolean_T LDWC_ErrSidCdtn_B;           /* '<S66>/Logical Operator16'
                                        * Error condition of side
                                        */
boolean_T LDWC_CLatDevByDlcLf_B;       /* '<S65>/Logical Operator24' */
boolean_T LDWC_CLatDevByDgrLf_B;       /* '<S65>/Relational Operator39' */
boolean_T LDWC_CclLatDevLf_B;          /* '<S65>/Logical Operator12'
                                        * Cancel condition of left lane deviation
                                        */
boolean_T LDWC_CLatDevByDlcRi_B;       /* '<S65>/Relational Operator40' */
boolean_T LDWC_CLatDevByDgrRi_B;       /* '<S65>/Logical Operator25' */
boolean_T LDWC_CclLatDevRi_B;          /* '<S65>/Logical Operator14'
                                        * Cancel condition of right lane deviation
                                        */
boolean_T LDWC_CclLatDev_B;            /* '<S65>/Logical Operator13'
                                        * Cancel condition of lane deviation
                                        */
boolean_T LDWC_Cancel_B;               /* '<S63>/Logical Operator11' */
boolean_T LDWC_AbortBySpecific_B;      /* '<S64>/Relational Operator2'
                                        * LDW abort conditions by LDW specific bitfield
                                        */
boolean_T LDWC_AbortByVehSt_B;         /* '<S64>/Relational Operator1'
                                        * LDW abort conditions by vehicle state
                                        */
boolean_T LDWC_AbortByDrvSt_B;         /* '<S64>/Relational Operator3'
                                        * LDW abort conditions by drive state
                                        */
boolean_T LDWC_AbortByCtrlSt_B;        /* '<S64>/Relational Operator4'
                                        * LDW abort conditions by active control state
                                        */
boolean_T LDWC_AbortBySysSt_B;         /* '<S64>/Relational Operator5'
                                        * LDW abort conditions by system state
                                        */
boolean_T LDWC_AbortByAvlSt_B;         /* '<S64>/Relational Operator6'
                                        * LDW abort conditions by no available state
                                        */
boolean_T LDWC_AbortByPrjSpec_B;       /* '<S64>/Relational Operator7'
                                        * LDW abort conditions by customer projects
                                        */
boolean_T LDWC_Abort_B;                /* '<S64>/Logical Operator6'
                                        * Condition of LDW abort state
                                        */
boolean_T LDWC_StrgRdyBySpecific_B;    /* '<S64>/Relational Operator9'
                                        * LDW strong ready conditions by LDW specific bitfield
                                        */
boolean_T LDWC_StrgRdyByVehSt_B;       /* '<S64>/Relational Operator8'
                                        * LDW strong ready conditions by vehicle state
                                        */
boolean_T LDWC_StrgRdyByDrvSt_B;       /* '<S64>/Relational Operator10'
                                        * LDW strong ready conditions by drive state
                                        */
boolean_T LDWC_StrgRdyByCtrlSt_B;      /* '<S64>/Relational Operator11'
                                        * LDW strong ready conditions by active control state
                                        */
boolean_T LDWC_StrgRdyBySysSt_B;       /* '<S64>/Relational Operator12'
                                        * LDW strong ready conditions by system state
                                        */
boolean_T LDWC_StrgRdyByAvlSt_B;       /* '<S64>/Relational Operator13'
                                        * LDW strong ready conditions by no available state
                                        */
boolean_T LDWC_StrgRdyByPrjSpec_B;     /* '<S64>/Relational Operator14'
                                        * LDW strong ready conditions by customer projects
                                        */
boolean_T LDWC_StrgRdy_B;              /* '<S64>/Logical Operator1'
                                        * Condition of LDW strong ready state
                                        */
boolean_T LDWC_Degradation_B;          /* '<S59>/Logical Operator1' */
boolean_T LDWC_DegradationEdgeRise_B;  /* '<S85>/AND' */
boolean_T LDWC_Degr_B;                 /* '<S59>/Logical Operator2'
                                        * Condition of degradation
                                        */
boolean_T LDWC_SuppBySpecific_B;       /* '<S64>/Relational Operator21'
                                        * LDW suppresion conditions by LDW specific bitfield
                                        */
boolean_T LDWC_SuppByVehSt_B;          /* '<S64>/Relational Operator20'
                                        * LDW suppresion conditions by vehicle state
                                        */
boolean_T LDWC_SuppByDrvSt_B;          /* '<S64>/Relational Operator15'
                                        * LDW suppresion conditions by drive state
                                        */
boolean_T LDWC_SuppByCtrlSt_B;         /* '<S64>/Relational Operator16'
                                        * LDW suppresion conditions by active control state
                                        */
boolean_T LDWC_SuppBySysSt_B;          /* '<S64>/Relational Operator17'
                                        * LDW suppresion conditions by system state
                                        */
boolean_T LDWC_SuppyByAvlSt_B;         /* '<S64>/Relational Operator18'
                                        * LDW suppresion conditions by no available state
                                        */
boolean_T LDWC_SuppPrjSpec_B;          /* '<S64>/Relational Operator19'
                                        * LDW suppresion conditions by customer projects
                                        */
boolean_T LDWC_Suppresion_B;           /* '<S64>/Logical Operator3' */
boolean_T LDWC_WeakRdyBySpecific_B;    /* '<S64>/Relational Operator28'
                                        * LDW weak ready conditions by LDW specific bitfield
                                        */
boolean_T LDWC_WeakRdyByVehSt_B;       /* '<S64>/Relational Operator27'
                                        * LDW weak ready conditions by vehicle state
                                        */
boolean_T LDWC_WeakRdyByDrvSt_B;       /* '<S64>/Relational Operator22'
                                        * LDW weak ready conditions by drive state
                                        */
boolean_T LDWC_WeakRdyByCtrlSt_B;      /* '<S64>/Relational Operator23'
                                        * LDW strong weak conditions by active control state
                                        */
boolean_T LDWC_WeakRdyBySysSt_B;       /* '<S64>/Relational Operator24'
                                        * LDW weak ready conditions by system state
                                        */
boolean_T LDWC_WeakRdyByAvlSt_B;       /* '<S64>/Relational Operator25'
                                        * LDW weak weak conditions by no available state
                                        */
boolean_T LDWC_WeakRdyByPrjSpec_B;     /* '<S64>/Relational Operator26'
                                        * LDW weak weak conditions by customer projects
                                        */
boolean_T LDWC_WkRdy_B;                /* '<S64>/Logical Operator4'
                                        * Condition of LDW weak ready state
                                        */
boolean_T LDWC_BlockTimeBySysOut_B;    /* '<S64>/Logical Operator9' */
boolean_T LDWC_RawBlockTimeByRampOut_B;/* '<S71>/AND' */
boolean_T LDWC_BlockTimeByRampOut_B;   /* '<S83>/GreaterThan1' */
boolean_T LDWC_BlockTime_B;            /* '<S64>/Logical Operator10' */
boolean_T LDWC_Suppression_B;          /* '<S62>/OR'
                                        * Suppresion condition
                                        */
boolean_T LDWC_Trig_B;                 /* '<S104>/Logical Operator1'
                                        * Condition of trigger
                                        */
E_LDWState_nu LDWC_SysOut_St;          /* '<S48>/Switch1'
                                        * Actual state of LDW
                                        */

/* Exported block states */
real32_T LDVSE_HodTiTrnSglLf_Sec;      /* '<S35>/Unit Delay'
                                        * Holding time of left turn signal
                                        */
real32_T LDVSE_HodTiTrnSglRi_Sec;      /* '<S36>/Unit Delay'
                                        * Holding time of right turn signal
                                        */
real32_T LDWC_DlyTiOfTiToLnRiMn_Sec;   /* '<S134>/Unit Delay'
                                        * Delay time of time to right lane crossing
                                        */
real32_T LDWC_HdTiTrigRi_Sec;          /* '<S132>/Unit Delay'
                                        * holding time right trigger
                                        */
real32_T LDWC_ContinWarmTimesOldRi_Count;/* '<S108>/Unit Delay2'
                                          * The number of consecutive alarms in the previous cycle
                                          */
real32_T LDWC_SuppTimeOldRi_Sec;       /* '<S133>/Unit Delay'
                                        * Holding time of left turn signal
                                        */
real32_T LDWC_DlyTiOfTiToLnLfMn_Sec;   /* '<S122>/Unit Delay'
                                        * Delay time of time to left lane crossing
                                        */
real32_T LDWC_HdTiTrigLf_Sec;          /* '<S120>/Unit Delay'
                                        *  holding time left trigger
                                        */
real32_T LDWC_ContinWarmTimesOldLf_Count;/* '<S107>/Unit Delay2'
                                          * The number of consecutive alarms in the previous cycle
                                          */
real32_T LDWC_SuppTimeOldLf_Sec;       /* '<S121>/Unit Delay'
                                        * Holding time of left turn signal
                                        */
real32_T LDWC_HdTiWarming_Sec;         /* '<S103>/Unit Delay'
                                        * Holding time of warming state start
                                        */
real32_T LDWC_DlyTiTgtFns_Sec;         /* '<S96>/Unit Delay'
                                        * Delay time of LDW finish state
                                        */
real32_T LDWC_HdTiWarmMx_Sec;          /* '<S70>/Unit Delay'
                                        * Holding time of warming state start
                                        */
real32_T LDWC_HdTiDegr_Sec;            /* '<S86>/Unit Delay'
                                        * Holding time of degradation
                                        */
real32_T LDWC_HdTiFns_Sec;             /* '<S83>/Unit Delay'
                                        * Holding time of finish state end
                                        */
real32_T LDWC_ActiveStopWatch_Ri_sec;  /* '<S82>/Unit Delay' */
real32_T LDWC_HdTiFns_Ri_Sec;          /* '<S84>/Unit Delay' */
uint8_T LDWC_DgrSideOld_St;            /* '<S104>/UnitDelay1'
                                        * Old state of danger side
                                        */
boolean_T LDDT_UHysCltdCurvVldRi_B;    /* '<S17>/Unit Delay'
                                        * Validity of right lane Clothoid curvature
                                        */
boolean_T LDDT_BHysHeadAglTrigVldRi_B; /* '<S16>/Unit Delay'
                                        * Valid trigger of right heading angle
                                        */
boolean_T LDDT_UHysHeadAglCclVldRi_B;  /* '<S18>/Unit Delay'
                                        * Valid cancel of right heading angle
                                        */
boolean_T LDVSE_BHysLatVehSpdVldLf_B;  /* '<S41>/Unit Delay'
                                        * Validity of left lateral vehicle speed before trigger state
                                        */
boolean_T LDDT_UHysCltdCurvVldLf_B;    /* '<S13>/Unit Delay'
                                        * Validity of left lane Clothoid curvature
                                        */
boolean_T LDDT_BHysHeadAglTrigVldLf_B; /* '<S12>/Unit Delay'
                                        * Valid trigger of left heading angle
                                        */
boolean_T LDDT_UHysHeadAglCclVldLf_B;  /* '<S14>/Unit Delay'
                                        * Valid cancel of left heading angle
                                        */
boolean_T LDVSE_UHysLatVehSpdVldLf_B;  /* '<S42>/Unit Delay'
                                        * Validity of left lateral vehicle speed after trigger state
                                        */
boolean_T LDVSE_EdgeRisTrnSglRi_B;     /* '<S33>/Unit Delay'
                                        * Edge rise of right turn signal
                                        */
boolean_T LDVSE_BHysLatVehSpdVldRi_B;  /* '<S45>/Unit Delay'
                                        * Validity of right lateral vehicle speed before trigger state
                                        */
boolean_T LDVSE_UHysLatVehSpdVldRi_B;  /* '<S46>/Unit Delay'
                                        * Validity of right lateral vehicle speed after trigger state
                                        */
boolean_T LDVSE_EdgeRisTrnSglLf_B;     /* '<S34>/Unit Delay'
                                        * Edge rise of left turn signal
                                        */
boolean_T LDVSE_UHysSteAgl_B;          /* '<S27>/Unit Delay'
                                        * Validity of steering wheel angle
                                        */
boolean_T LDVSE_BHysSpdVeh_B;          /* '<S23>/Unit Delay'
                                        * Validity of displayed longitudinal speed
                                        */
boolean_T LDVSE_UHysSteAglSpd_B;       /* '<S28>/Unit Delay'
                                        * Validity of steering wheel angle speed
                                        */
boolean_T LDVSE_BHysAccVehX_B;         /* '<S24>/Unit Delay'
                                        * Validity of  longitudinal Acceleration
                                        */
boolean_T LDVSE_BHysAccVehY_B;         /* '<S29>/Unit Delay'
                                        * Validity of  lateral Acceleration
                                        */
boolean_T LDVSE_UHysVehCurv_B;         /* '<S30>/Unit Delay'
                                        * Validity of  vehicle curvature
                                        */
boolean_T LDVSE_BHysLnWid_B;           /* '<S25>/Unit Delay'
                                        * Validity of lane width
                                        */
boolean_T LDWC_HdTiTrigRiEn_B;         /* '<S123>/Unit Delay'
                                        * Enable condition of holding time right trigger
                                        */
boolean_T LDWC_DisTrigRi_B;            /* '<S130>/Unit Delay'
                                        * Disable condition of right trigger
                                        */
boolean_T LDWC_SuppFlagOldRi_B;        /* '<S108>/Unit Delay'
                                        * Suppression signal of continuous alarm in previous period
                                        */
boolean_T LDWC_PreActiveEdgeRi;        /* '<S126>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
boolean_T LDWC_ContinTrigRiEn_B;       /* '<S127>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
boolean_T LDWC_DisContinTrigRi_B;      /* '<S131>/Unit Delay'
                                        * Disable condition of continue left trigger
                                        */
boolean_T LDWC_HdTiTrigLfEn_B;         /* '<S111>/Unit Delay'
                                        * Enable condition of holding time left trigger
                                        */
boolean_T LDWC_DisTrigLf_B;            /* '<S118>/Unit Delay'
                                        * Disable condition of left trigger
                                        */
boolean_T LDWC_SuppFlagOldLf_B;        /* '<S107>/Unit Delay'
                                        * Suppression signal of continuous alarm in previous period
                                        */
boolean_T LDWC_PreActiveEdgeLf;        /* '<S114>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
boolean_T LDWC_ContinTrigLfEn_B;       /* '<S115>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
boolean_T LDWC_DisContinTrigLf_B;      /* '<S119>/Unit Delay'
                                        * Disable condition of continue left trigger
                                        */
boolean_T LDWC_EdgeRisWarming_B;       /* '<S87>/Unit Delay'
                                        * Edge rise of warming state
                                        */
boolean_T LDWC_EdgeRisWarmMx_B;        /* '<S68>/Unit Delay'
                                        * Edge rise of warming state
                                        */
boolean_T LDWC_EdgeRisDegr_B;          /* '<S85>/Unit Delay'
                                        * Edge rise of degradation
                                        */
boolean_T LDWC_DegrOld_B;              /* '<S59>/UnitDelay'
                                        * UnitDelay condition of degradation
                                        */
boolean_T LDWC_EdgeRisFns_B;           /* '<S71>/Unit Delay'
                                        * Edge rise of fginish and cancel state
                                        */
boolean_T LDWC_EdgeRisActive_Ri_B;     /* '<S74>/Unit Delay' */
boolean_T LDWC_EdgeRisFns_Ri_B;        /* '<S72>/Unit Delay' */

/* Block states (default storage) */
DW_LDWSA_T LDWSA_DW;

/* Exported data definition */

/* Definition for custom storage class: Global */
real32_T LDWC_LnLatVeh_BX_Mps[9] = { -1.5F, -1.0F, -0.5F, -0.3F, 0.0F, 0.3F,
  0.5F, 1.0F, 1.5F } ;                 /* Referenced by:
                                        * '<S106>/1-D Lookup Table10'
                                        * '<S106>/1-D Lookup Table11'
                                        */

real32_T LDWC_LnLatVeh_Lf_Mps[9] = { 0.35F, 0.35F, 0.25F, 0.2F, 0.2F, 0.1F, 0.1F,
  0.1F, 0.1F } ;                /* Referenced by: '<S106>/1-D Lookup Table10' */

real32_T LDWC_LnLatVeh_Ri_Mps[9] = { 0.1F, 0.1F, 0.1F, 0.1F, 0.2F, 0.2F, 0.25F,
  0.35F, 0.35F } ;              /* Referenced by: '<S106>/1-D Lookup Table11' */

uint8_T LDWSA_SetBit_BS_Param_1[8] = { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U } ;
                                    /* Referenced by: '<S32>/ex_sfun_set_bit' */

uint8_T LDWSA_SetBit_BS_Param_2[2] = { 0U, 1U } ;/* Referenced by:
                                                  * '<S43>/ex_sfun_set_bit'
                                                  * '<S44>/ex_sfun_set_bit'
                                                  */

uint8_T LDWSA_SetBit_BS_Param_3[9] = { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U } ;
                                   /* Referenced by: '<S144>/ex_sfun_set_bit' */



/* vehicle speed threshold for HMI */

/* Definition for custom storage class: Global */
boolean_T LDDT_LnLengthLf_B;           /* '<S15>/Unit Delay' */
boolean_T LDDT_LnLengthRi_B;           /* '<S19>/Unit Delay' */
uint8_T LDVSE_PrevVehStartupSpd_Kmph;  /* '<S31>/Unit Delay' */

/* Yawrate hysteresis--Used in LDWC module */
real32_T LDWC_ActiveStopWatch_sec;     /* '<S81>/Unit Delay' */
boolean_T LDWC_EdgeRisActive_B;        /* '<S73>/Unit Delay' */
uint8_T LDWC_PrevDgrSide_St;           /* '<S2>/Unit Delay' */

/* Yawrate hysteresis--Used in LDWC module */
boolean_T LDWC_PrevSwitchUnitDelay_bool;/* '<S50>/Unit Delay' */
E_LDWState_nu LDWC_PrevSysOutIn_St;    /* '<S53>/Unit Delay' */

/* Yawrate hysteresis--Used in LDWC module */
boolean_T LDWC_RampTimeExpiredRSFF_bool;/* '<S56>/Unit Delay' */
real32_T LDWC_SafeFuncActiveTurnOnDelay_sec;/* '<S145>/Unit Delay' */
real32_T LDWC_SafeFuncErrorTurnOnDelay_sec;/* '<S146>/Unit Delay' */
real32_T LDWC_SusTimeExpiredTimerRetrigger_sec;/* '<S57>/Unit Delay' */

/* Yawrate hysteresis--Used in LDWC module */
E_LDWState_nu LDWC_SysOld_St;          /* '<S5>/UnitDelay' */

/* Yawrate hysteresis--Used in LDWC module */
boolean_T LDWC_VehYawRateHyst_bool;    /* '<S147>/Unit Delay' */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Yawrate hysteresis--Used in LDPSC module */

/* Model step function */
void LDWSA_step(void)
{
  /* local block i/o variables */
  uint32_T rtb_ex_sfun_set_bit;
  uint32_T rtb_ex_sfun_set_bit_czhm;
  uint32_T rtb_ex_sfun_set_bit_dmuc;
  boolean_T rtb_VectorConcatenate[2];
  boolean_T rtb_VectorConcatenate_grcm[2];
  boolean_T rtb_VectorConcatenate_oy1j[8];
  int32_T LDVSE_NVRAMVehStartupSpd_k_chkk;
  int32_T LDWC_TrigByPrjSpecRi_B_tmp;
  real32_T LDVSE_UHysLatVehSpdVldLf_B_tmp;
  real32_T LDVSE_UHysLatVehSpdVldRi_B_tmp;
  real32_T rtb_LDWC_RampoutTime_Sec;
  real32_T rtb_Subtract2_hwoc;
  real32_T rtb_uDLookupTable8_idx_0;
  real32_T rtb_uDLookupTable9_idx_1;
  real32_T tmp;
  E_LDWState_nu rtb_LDWState;
  uint8_T rtb_UnitDelay_btcf;
  boolean_T rtb_VectorConcatenate_oaf0[9];
  boolean_T rtb_AND_jseu;
  boolean_T rtb_AND_k3gu;
  boolean_T rtb_Equal_azab;
  boolean_T rtb_LDDT_LnCurvVldLf_B;
  boolean_T rtb_LDDT_LnCurvVldRi_B;
  boolean_T rtb_LDWC_VehStInvalid_B;
  boolean_T rtb_LDWC_VehicleInvalid_B;
  boolean_T rtb_LDWC_VelYInvalid_B;
  boolean_T rtb_LogicalOperator13;
  boolean_T rtb_LogicalOperator14;
  boolean_T rtb_LogicalOperator14_iqhs;
  boolean_T rtb_LogicalOperator14_o2nq;
  boolean_T rtb_LogicalOperator2_i4cj;
  boolean_T rtb_LogicalOperator8_m03q;
  boolean_T rtb_RelationalOperator1;
  boolean_T rtb_RelationalOperator10;
  boolean_T rtb_RelationalOperator10_f5fy;
  boolean_T rtb_RelationalOperator2;
  boolean_T rtb_RelationalOperator29;
  boolean_T rtb_RelationalOperator33;
  boolean_T rtb_RelationalOperator4;

  /* S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/LDW'
   */
  /* Abs: '<S106>/Abs' incorporates:
   *  Inport: '<Root>/Inport25'
   *  Inport: '<Root>/Inport27'
   */
  LDWC_CrvSensiDecayLe_Mi = fabsf(LDWSAI_CurvSafeLf_ReMi);
  rtb_uDLookupTable9_idx_1 = fabsf(LDWSAI_CurvSafeRi_ReMi);

  /* Lookup_n-D: '<S106>/1-D Lookup Table8' */
  rtb_uDLookupTable8_idx_0 = look1_iflf_binlxpw(LDWC_CrvSensiDecayLe_Mi, ((const
    real32_T *)&(LDWC_CrvSensiDecay_BX_ReMi[0])), ((const real32_T *)
    &(LDWC_CrvSensiDecay_BY_Mi[0])), 3U);

  /* Lookup_n-D: '<S106>/1-D Lookup Table9' incorporates:
   *  Lookup_n-D: '<S106>/1-D Lookup Table8'
   *  UnaryMinus: '<S106>/Unary Minus'
   */
  LDWC_CrvSensiDecayLe_Mi = -look1_iflf_binlxpw(LDWC_CrvSensiDecayLe_Mi, ((const
    real32_T *)&(LDWC_CrvSensiAdvance_BX_ReMi[0])), ((const real32_T *)
    &(LDWC_CrvSensiAdvance_BY_Mi[0])), 3U);

  /* Switch: '<S9>/Switch2' incorporates:
   *  Constant: '<S9>/V_Parameter1'
   *  Inport: '<Root>/Inport26'
   *  RelationalOperator: '<S9>/GreaterThan3'
   *  RelationalOperator: '<S9>/Less Than2'
   *  Switch: '<S9>/Switch3'
   *  UnaryMinus: '<S9>/Unary Minus2'
   */
  if (LDWSAI_CurvRi_ReMi < (-LDDT_CurveThd_C_St)) {
    /* Switch: '<S9>/Switch2' incorporates:
     *  Constant: '<S9>/V_Parameter11'
     */
    LDDT_CurveTypeRi_St = LDDT_CurveInner_C_St;
  } else if (LDWSAI_CurvRi_ReMi > LDDT_CurveThd_C_St) {
    /* Switch: '<S9>/Switch3' incorporates:
     *  Constant: '<S9>/V_Parameter12'
     *  Switch: '<S9>/Switch2'
     */
    LDDT_CurveTypeRi_St = LDDT_CurveOuter_C_St;
  } else {
    /* Switch: '<S9>/Switch2' incorporates:
     *  Constant: '<S9>/V_Parameter13'
     *  Switch: '<S9>/Switch3'
     */
    LDDT_CurveTypeRi_St = LDDT_CurveNone_C_St;
  }

  /* End of Switch: '<S9>/Switch2' */

  /* Switch: '<S106>/Switch6' incorporates:
   *  Constant: '<S106>/V_Parameter3'
   *  Constant: '<S106>/V_Parameter5'
   *  RelationalOperator: '<S106>/Relational Operator4'
   *  RelationalOperator: '<S106>/Relational Operator6'
   *  Switch: '<S106>/Switch8'
   */
  if (LDDT_CurveTypeRi_St == LDDT_CurveInner_C_St) {
    /* Switch: '<S106>/Switch6' incorporates:
     *  Lookup_n-D: '<S106>/1-D Lookup Table8'
     */
    LDWC_CrvSensiDecayRi_Mi = look1_iflf_binlxpw(rtb_uDLookupTable9_idx_1, ((
      const real32_T *)&(LDWC_CrvSensiDecay_BX_ReMi[0])), ((const real32_T *)
      &(LDWC_CrvSensiDecay_BY_Mi[0])), 3U);
  } else if (LDDT_CurveTypeRi_St == LDDT_CurveOuter_C_St) {
    /* Switch: '<S106>/Switch8' incorporates:
     *  Lookup_n-D: '<S106>/1-D Lookup Table8'
     *  Lookup_n-D: '<S106>/1-D Lookup Table9'
     *  Switch: '<S106>/Switch6'
     *  UnaryMinus: '<S106>/Unary Minus'
     */
    LDWC_CrvSensiDecayRi_Mi = -look1_iflf_binlxpw(rtb_uDLookupTable9_idx_1, ((
      const real32_T *)&(LDWC_CrvSensiAdvance_BX_ReMi[0])), ((const real32_T *)
      &(LDWC_CrvSensiAdvance_BY_Mi[0])), 3U);
  } else {
    /* Switch: '<S106>/Switch6' incorporates:
     *  Constant: '<S106>/V_Const2'
     *  Switch: '<S106>/Switch8'
     */
    LDWC_CrvSensiDecayRi_Mi = 0.0F;
  }

  /* End of Switch: '<S106>/Switch6' */

  /* Switch: '<S106>/Switch4' incorporates:
   *  Constant: '<S106>/V_Const3'
   *  Constant: '<S106>/V_Parameter1'
   *  Inport: '<Root>/Inport14'
   */
  if (LDWSAI_LDPSwitchOn_B) {
    rtb_uDLookupTable9_idx_1 = LDWC_LDPSensiDecay_C_Mi;
  } else {
    rtb_uDLookupTable9_idx_1 = 0.0F;
  }

  /* End of Switch: '<S106>/Switch4' */

  /* RelationalOperator: '<S6>/Equal' incorporates:
   *  Constant: '<S6>/V_Parameter'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDDT_RdyTrigLDW_B = (LDWC_PrevDgrSide_St == LDDT_NoDgrSide_C_St);

  /* Lookup_n-D: '<S11>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDDT_CrvThdMaxRi_ReMi = look1_iflf_binlxpw(LDWSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDDT_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDDT_TrsdLnCltdCurvRiMx_Cr_Mps[0])), 7U);

  /* Lookup_n-D: '<S11>/1-D Lookup Table1' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDDT_CrvThdHystRi_ReMi = look1_iflf_binlxpw(LDWSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDDT_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDDT_TrsdLnCltdCurvRiOfst_Cr_Mps[0])), 7U);

  /* Logic: '<S6>/AND' incorporates:
   *  Constant: '<S6>/V_Parameter1'
   */
  LDDT_EnaSafety_B = ((LDDT_SfFcLDWOn_C_B) && LDDT_RdyTrigLDW_B);

  /* Switch: '<S6>/Switch5' */
  if (LDDT_EnaSafety_B) {
    /* Switch: '<S6>/Switch5' incorporates:
     *  Inport: '<Root>/Inport27'
     */
    LDDT_LnCltdCurvRi_ReMi = LDWSAI_CurvSafeRi_ReMi;
  } else {
    /* Switch: '<S6>/Switch5' incorporates:
     *  Inport: '<Root>/Inport26'
     */
    LDDT_LnCltdCurvRi_ReMi = LDWSAI_CurvRi_ReMi;
  }

  /* End of Switch: '<S6>/Switch5' */

  /* Switch: '<S17>/Switch' incorporates:
   *  Sum: '<S17>/Add'
   *  UnitDelay: '<S17>/Unit Delay'
   */
  if (LDDT_UHysCltdCurvVldRi_B) {
    rtb_LDWC_RampoutTime_Sec = LDDT_CrvThdHystRi_ReMi + LDDT_CrvThdMaxRi_ReMi;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDDT_CrvThdMaxRi_ReMi;
  }

  /* End of Switch: '<S17>/Switch' */

  /* RelationalOperator: '<S17>/GreaterThan' incorporates:
   *  Abs: '<S11>/Abs'
   *  UnitDelay: '<S17>/Unit Delay'
   */
  LDDT_UHysCltdCurvVldRi_B = (rtb_LDWC_RampoutTime_Sec > fabsf
    (LDDT_LnCltdCurvRi_ReMi));

  /* Logic: '<S11>/NOT' incorporates:
   *  Constant: '<S11>/V_Parameter7'
   *  Inport: '<Root>/Inport44'
   *  Logic: '<S10>/AND'
   *  Logic: '<S10>/NOT'
   *  Logic: '<S11>/AND'
   */
  LDDT_EnaByCstruSiteLf_B = ((!LDWSAI_DtctCstruSite_B) ||
    (!LDDT_CstruSiteLDW_C_B));

  /* Logic: '<S11>/AND1' incorporates:
   *  Logic: '<S11>/AND'
   *  Logic: '<S11>/NOT'
   *  UnitDelay: '<S17>/Unit Delay'
   */
  rtb_LDDT_LnCurvVldRi_B = (LDDT_EnaByCstruSiteLf_B && LDDT_UHysCltdCurvVldRi_B);

  /* Switch: '<S6>/Switch1' */
  if (LDDT_EnaSafety_B) {
    /* Switch: '<S6>/Switch1' incorporates:
     *  Inport: '<Root>/Inport23'
     */
    LDDT_LnHeadRi_Rad = LDWSAI_HeadAglSafeRi_Rad;
  } else {
    /* Switch: '<S6>/Switch1' incorporates:
     *  Inport: '<Root>/Inport22'
     */
    LDDT_LnHeadRi_Rad = LDWSAI_HeadAglRi_Rad;
  }

  /* End of Switch: '<S6>/Switch1' */

  /* Switch: '<S16>/Switch' incorporates:
   *  Constant: '<S11>/V_Parameter10'
   *  Constant: '<S11>/V_Parameter11'
   *  Constant: '<S11>/V_Parameter12'
   *  Sum: '<S16>/Add'
   *  Sum: '<S16>/Add1'
   *  Switch: '<S16>/Switch1'
   *  UnitDelay: '<S16>/Unit Delay'
   */
  if (LDDT_BHysHeadAglTrigVldRi_B) {
    rtb_LDWC_RampoutTime_Sec = LDDT_TrsdHeadAglMx_C_Rad +
      LDDT_TrsdHeadAglOfst_C_Rad;
    tmp = LDDT_TrsdHeadAglMn_C_Rad - LDDT_TrsdHeadAglOfst_C_Rad;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDDT_TrsdHeadAglMx_C_Rad;
    tmp = LDDT_TrsdHeadAglMn_C_Rad;
  }

  /* End of Switch: '<S16>/Switch' */

  /* Logic: '<S16>/AND' incorporates:
   *  RelationalOperator: '<S16>/GreaterThan'
   *  RelationalOperator: '<S16>/GreaterThan1'
   *  UnitDelay: '<S16>/Unit Delay'
   */
  LDDT_BHysHeadAglTrigVldRi_B = ((rtb_LDWC_RampoutTime_Sec > LDDT_LnHeadRi_Rad) &&
    (LDDT_LnHeadRi_Rad > tmp));

  /* RelationalOperator: '<S11>/Relational Operator4' incorporates:
   *  Constant: '<S11>/Constant2'
   *  Constant: '<S11>/V_Parameter4'
   *  Inport: '<Root>/Inport37'
   *  S-Function (sfix_bitop): '<S11>/Bitwise Operator1'
   */
  LDDT_EnaByInVldQlfrRi_B = ((((int32_T)LDWSAI_LnIVldRi_St) & ((int32_T)
    LDDT_LnIvldRi_C_St)) == 0);

  /* RelationalOperator: '<S11>/Relational Operator1' incorporates:
   *  Constant: '<S11>/Constant1'
   *  Constant: '<S11>/V_Parameter3'
   *  Inport: '<Root>/Inport36'
   *  S-Function (sfix_bitop): '<S11>/Bitwise Operator'
   */
  LDDT_EnaByInVldQlfrSfRi_B = ((((int32_T)LDWSAI_IvldLnSafeRi_St) & ((int32_T)
    LDDT_LnIvldSfRi_C_St)) == 0);

  /* Switch: '<S19>/Switch' incorporates:
   *  Constant: '<S11>/Constant'
   *  UnitDelay: '<S19>/Unit Delay'
   */
  if (LDDT_LnLengthRi_B) {
    LDVSE_NVRAMVehStartupSpd_k_chkk = 11;
  } else {
    LDVSE_NVRAMVehStartupSpd_k_chkk = 10;
  }

  /* End of Switch: '<S19>/Switch' */

  /* RelationalOperator: '<S19>/GreaterThan' incorporates:
   *  Inport: '<Root>/Inport28'
   *  UnitDelay: '<S19>/Unit Delay'
   */
  LDDT_LnLengthRi_B = (((real32_T)LDVSE_NVRAMVehStartupSpd_k_chkk) >
                       LDWSAI_LnLengthRi_Mi);

  /* Logic: '<S11>/NOT1' incorporates:
   *  UnitDelay: '<S19>/Unit Delay'
   */
  rtb_LDWC_VehicleInvalid_B = !LDDT_LnLengthRi_B;

  /* Switch: '<S11>/Switch1' incorporates:
   *  Constant: '<S11>/V_Parameter1'
   *  Logic: '<S11>/Logical Operator'
   */
  if (LDDT_SfFcLDWOn_C_B) {
    rtb_RelationalOperator29 = (LDDT_EnaByInVldQlfrRi_B &&
      LDDT_EnaByInVldQlfrSfRi_B);
  } else {
    rtb_RelationalOperator29 = LDDT_EnaByInVldQlfrRi_B;
  }

  /* End of Switch: '<S11>/Switch1' */

  /* Logic: '<S11>/Logical Operator2' incorporates:
   *  UnitDelay: '<S16>/Unit Delay'
   */
  LDDT_LnTrigVldRi_B = (((rtb_LDDT_LnCurvVldRi_B && LDDT_BHysHeadAglTrigVldRi_B)
    && rtb_RelationalOperator29) && rtb_LDWC_VehicleInvalid_B);

  /* Switch: '<S18>/Switch' incorporates:
   *  Constant: '<S11>/V_Parameter5'
   *  Constant: '<S11>/V_Parameter8'
   *  Sum: '<S18>/Add'
   *  UnitDelay: '<S18>/Unit Delay'
   */
  if (LDDT_UHysHeadAglCclVldRi_B) {
    rtb_LDWC_RampoutTime_Sec = LDDT_TrsdHeadAglOfst_C_Rad +
      LDDT_TrsdHeadAglMx_C_Rad;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDDT_TrsdHeadAglMx_C_Rad;
  }

  /* End of Switch: '<S18>/Switch' */

  /* RelationalOperator: '<S18>/GreaterThan' incorporates:
   *  Abs: '<S11>/Abs2'
   *  UnitDelay: '<S18>/Unit Delay'
   */
  LDDT_UHysHeadAglCclVldRi_B = (rtb_LDWC_RampoutTime_Sec > fabsf
    (LDDT_LnHeadRi_Rad));

  /* RelationalOperator: '<S11>/Relational Operator5' incorporates:
   *  Constant: '<S11>/Constant3'
   *  Constant: '<S11>/V_Parameter2'
   *  Inport: '<Root>/Inport37'
   *  S-Function (sfix_bitop): '<S11>/Bitwise Operator2'
   */
  LDDT_CclByInVldQlfrRi_B = ((((int32_T)LDWSAI_LnIVldRi_St) & ((int32_T)
    LDDT_LnIvldCclRi_C_St)) == 0);

  /* Logic: '<S11>/Logical Operator5' incorporates:
   *  UnitDelay: '<S18>/Unit Delay'
   */
  LDDT_LnCclVldRi_B = (((rtb_LDDT_LnCurvVldRi_B && LDDT_UHysHeadAglCclVldRi_B) &&
                        LDDT_CclByInVldQlfrRi_B) && rtb_LDWC_VehicleInvalid_B);

  /* Switch: '<S11>/Switch2' */
  if (LDDT_RdyTrigLDW_B) {
    /* Switch: '<S11>/Switch2' */
    LDDT_LnMakVldRi_B = LDDT_LnTrigVldRi_B;
  } else {
    /* Switch: '<S11>/Switch2' */
    LDDT_LnMakVldRi_B = LDDT_LnCclVldRi_B;
  }

  /* End of Switch: '<S11>/Switch2' */

  /* Product: '<S8>/Product1' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDDT_RawLatVehSpdRi_Mps = LDDT_LnHeadRi_Rad * LDWSAI_VehSpdActu_Mps;

  /* Switch: '<S8>/Switch2' */
  if (LDDT_LnMakVldRi_B) {
    /* Switch: '<S8>/Switch2' */
    LDDT_LatVehSpdRi_Mps = LDDT_RawLatVehSpdRi_Mps;
  } else {
    /* Switch: '<S8>/Switch2' incorporates:
     *  Constant: '<S8>/Constant8'
     */
    LDDT_LatVehSpdRi_Mps = 10.0F;
  }

  /* End of Switch: '<S8>/Switch2' */

  /* Lookup_n-D: '<S106>/1-D Lookup Table3' incorporates:
   *  Inport: '<Root>/Inport13'
   */
  LDWC_CrrctByLnWidth_Fct = look1_iflf_binlxpw(LDWSAI_LnWidCalc_Mi, ((const
    real32_T *)&(LDWC_LaneWidth_BX_Mi[0])), ((const real32_T *)
    &(LDWC_DTCFctLnWid_Cr_Fct[0])), 4U);

  /* Lookup_n-D: '<S106>/1-D Lookup Table4' incorporates:
   *  Inport: '<Root>/Inport27'
   */
  LDWC_DstcToLnTrsdCrvCpstnRi_Mi = look1_iflf_binlxpw(LDWSAI_CurvSafeRi_ReMi, ((
    const real32_T *)&(LDWC_LnDectCrvRi_BX_ReMi[0])), ((const real32_T *)
    &(LDWC_DstcToLnTrsdOfstRi_Cr_Mi[0])), 16U);

  /* Sum: '<S106>/Add3' incorporates:
   *  Lookup_n-D: '<S106>/1-D Lookup Table11'
   *  Product: '<S106>/Product3'
   *  Sum: '<S106>/Add1'
   *  Sum: '<S106>/Add5'
   *  Switch: '<S8>/Switch2'
   */
  LDWC_DstcToLnTrsdRi_Mi = ((look1_iflf_binlxpw(LDDT_LatVehSpdRi_Mps,
    (&(LDWC_LnLatVeh_BX_Mps[0])), (&(LDWC_LnLatVeh_Ri_Mps[0])), 8U) *
    LDWC_CrrctByLnWidth_Fct) + LDWC_DstcToLnTrsdCrvCpstnRi_Mi) -
    (LDWC_CrvSensiDecayRi_Mi + rtb_uDLookupTable9_idx_1);

  /* Switch: '<S6>/Switch3' */
  if (LDDT_EnaSafety_B) {
    /* Switch: '<S6>/Switch3' incorporates:
     *  Inport: '<Root>/Inport19'
     */
    LDDT_LnPstnRi_Mi = LDWSAI_PstnYSafeRi_Mi;
  } else {
    /* Switch: '<S6>/Switch3' incorporates:
     *  Inport: '<Root>/Inport18'
     */
    LDDT_LnPstnRi_Mi = LDWSAI_PstnYRi_Mi;
  }

  /* End of Switch: '<S6>/Switch3' */

  /* Product: '<S8>/Divide' incorporates:
   *  Constant: '<S8>/Constant'
   *  Inport: '<Root>/Inport'
   */
  rtb_Subtract2_hwoc = LDWSAI_VehWid_Mi / 2.0F;

  /* Sum: '<S8>/Subtract1' */
  LDDT_RawDstcToLnRi_Mi = LDDT_LnPstnRi_Mi + rtb_Subtract2_hwoc;

  /* Switch: '<S8>/Switch4' */
  if (LDDT_LnMakVldRi_B) {
    /* Switch: '<S8>/Switch4' */
    LDDT_DstcToLnRi_Mi = LDDT_RawDstcToLnRi_Mi;
  } else {
    /* Switch: '<S8>/Switch4' incorporates:
     *  Constant: '<S8>/Constant7'
     */
    LDDT_DstcToLnRi_Mi = -10.0F;
  }

  /* End of Switch: '<S8>/Switch4' */

  /* RelationalOperator: '<S108>/Relational Operator3' incorporates:
   *  UnaryMinus: '<S108>/Unary Minus'
   */
  LDWC_RawTrigByDlcRi_B = (LDDT_DstcToLnRi_Mi >= (-LDWC_DstcToLnTrsdRi_Mi));

  /* RelationalOperator: '<S22>/Equal' incorporates:
   *  Constant: '<S22>/V_Parameter'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDVSE_RdyTrigLDW_B = (LDWC_PrevDgrSide_St == LDVSE_NoDgrSide_C_St);

  /* Lookup_n-D: '<S22>/Lookup Table' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDVSE_MaxLatVel_Mps = look1_iflf_binlxpw(LDWSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDVSE_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDVSE_VehLatTrsd_Cr_Msp[0])), 7U);

  /* Lookup_n-D: '<S10>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDDT_CrvThdMaxLf_ReMi = look1_iflf_binlxpw(LDWSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDDT_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDDT_TrsdLnCltdCurvLfMx_Cr_Mps[0])), 7U);

  /* Lookup_n-D: '<S10>/1-D Lookup Table1' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDDT_CrvThdHystLf_ReMi = look1_iflf_binlxpw(LDWSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDDT_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDDT_TrsdLnCltdCurvLfOfst_Cr_Mps[0])), 7U);

  /* Switch: '<S6>/Switch4' */
  if (LDDT_EnaSafety_B) {
    /* Switch: '<S6>/Switch4' incorporates:
     *  Inport: '<Root>/Inport25'
     */
    LDDT_LnCltdCurvLf_ReMi = LDWSAI_CurvSafeLf_ReMi;
  } else {
    /* Switch: '<S6>/Switch4' incorporates:
     *  Inport: '<Root>/Inport24'
     */
    LDDT_LnCltdCurvLf_ReMi = LDWSAI_CurvLf_ReMi;
  }

  /* End of Switch: '<S6>/Switch4' */

  /* Switch: '<S13>/Switch' incorporates:
   *  Sum: '<S13>/Add'
   *  UnitDelay: '<S13>/Unit Delay'
   */
  if (LDDT_UHysCltdCurvVldLf_B) {
    rtb_LDWC_RampoutTime_Sec = LDDT_CrvThdHystLf_ReMi + LDDT_CrvThdMaxLf_ReMi;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDDT_CrvThdMaxLf_ReMi;
  }

  /* End of Switch: '<S13>/Switch' */

  /* RelationalOperator: '<S13>/GreaterThan' incorporates:
   *  Abs: '<S10>/Abs'
   *  UnitDelay: '<S13>/Unit Delay'
   */
  LDDT_UHysCltdCurvVldLf_B = (rtb_LDWC_RampoutTime_Sec > fabsf
    (LDDT_LnCltdCurvLf_ReMi));

  /* Logic: '<S10>/AND1' incorporates:
   *  UnitDelay: '<S13>/Unit Delay'
   */
  rtb_LDDT_LnCurvVldLf_B = (LDDT_EnaByCstruSiteLf_B && LDDT_UHysCltdCurvVldLf_B);

  /* Switch: '<S6>/Switch' */
  if (LDDT_EnaSafety_B) {
    /* Switch: '<S6>/Switch' incorporates:
     *  Inport: '<Root>/Inport21'
     */
    LDDT_LnHeadLf_Rad = LDWSAI_HeadAglSafeLf_Rad;
  } else {
    /* Switch: '<S6>/Switch' incorporates:
     *  Inport: '<Root>/Inport20'
     */
    LDDT_LnHeadLf_Rad = LDWSAI_HeadAglLf_Rad;
  }

  /* End of Switch: '<S6>/Switch' */

  /* Switch: '<S12>/Switch' incorporates:
   *  Constant: '<S10>/V_Parameter10'
   *  Constant: '<S10>/V_Parameter11'
   *  Constant: '<S10>/V_Parameter12'
   *  Sum: '<S12>/Add'
   *  Sum: '<S12>/Add1'
   *  Switch: '<S12>/Switch1'
   *  UnitDelay: '<S12>/Unit Delay'
   */
  if (LDDT_BHysHeadAglTrigVldLf_B) {
    rtb_LDWC_RampoutTime_Sec = LDDT_TrsdHeadAglMx_C_Rad +
      LDDT_TrsdHeadAglOfst_C_Rad;
    tmp = LDDT_TrsdHeadAglMn_C_Rad - LDDT_TrsdHeadAglOfst_C_Rad;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDDT_TrsdHeadAglMx_C_Rad;
    tmp = LDDT_TrsdHeadAglMn_C_Rad;
  }

  /* End of Switch: '<S12>/Switch' */

  /* Logic: '<S12>/AND' incorporates:
   *  RelationalOperator: '<S12>/GreaterThan'
   *  RelationalOperator: '<S12>/GreaterThan1'
   *  UnaryMinus: '<S10>/Unary Minus'
   *  UnitDelay: '<S12>/Unit Delay'
   */
  LDDT_BHysHeadAglTrigVldLf_B = ((rtb_LDWC_RampoutTime_Sec > (-LDDT_LnHeadLf_Rad))
    && ((-LDDT_LnHeadLf_Rad) > tmp));

  /* RelationalOperator: '<S10>/Relational Operator4' incorporates:
   *  Constant: '<S10>/Constant2'
   *  Constant: '<S10>/V_Parameter4'
   *  Inport: '<Root>/Inport35'
   *  S-Function (sfix_bitop): '<S10>/Bitwise Operator1'
   */
  LDDT_EnaByInVldQlfrLf_B = ((((int32_T)LDWSAI_LnIVldLf_St) & ((int32_T)
    LDDT_LnIvldLf_C_St)) == 0);

  /* RelationalOperator: '<S10>/Relational Operator1' incorporates:
   *  Constant: '<S10>/Constant1'
   *  Constant: '<S10>/V_Parameter3'
   *  Inport: '<Root>/Inport34'
   *  S-Function (sfix_bitop): '<S10>/Bitwise Operator'
   */
  LDDT_EnaByInVldQlfrSfLf_B = ((((int32_T)LDWSAI_IvldLnSafeLf_St) & ((int32_T)
    LDDT_LnIvldSfLf_C_St)) == 0);

  /* Switch: '<S15>/Switch' incorporates:
   *  Constant: '<S10>/Constant'
   *  UnitDelay: '<S15>/Unit Delay'
   */
  if (LDDT_LnLengthLf_B) {
    LDVSE_NVRAMVehStartupSpd_k_chkk = 11;
  } else {
    LDVSE_NVRAMVehStartupSpd_k_chkk = 10;
  }

  /* End of Switch: '<S15>/Switch' */

  /* RelationalOperator: '<S15>/GreaterThan' incorporates:
   *  Inport: '<Root>/Inport15'
   *  UnitDelay: '<S15>/Unit Delay'
   */
  LDDT_LnLengthLf_B = (((real32_T)LDVSE_NVRAMVehStartupSpd_k_chkk) >
                       LDWSAI_LnLengthLf_Mi);

  /* Logic: '<S10>/NOT1' incorporates:
   *  UnitDelay: '<S15>/Unit Delay'
   */
  rtb_LDWC_VelYInvalid_B = !LDDT_LnLengthLf_B;

  /* Switch: '<S10>/Switch1' incorporates:
   *  Constant: '<S10>/V_Parameter1'
   *  Logic: '<S10>/Logical Operator'
   */
  if (LDDT_SfFcLDWOn_C_B) {
    rtb_RelationalOperator29 = (LDDT_EnaByInVldQlfrLf_B &&
      LDDT_EnaByInVldQlfrSfLf_B);
  } else {
    rtb_RelationalOperator29 = LDDT_EnaByInVldQlfrLf_B;
  }

  /* End of Switch: '<S10>/Switch1' */

  /* Logic: '<S10>/Logical Operator2' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   */
  LDDT_LnTrigVldLf_B = (((rtb_LDDT_LnCurvVldLf_B && LDDT_BHysHeadAglTrigVldLf_B)
    && rtb_RelationalOperator29) && rtb_LDWC_VelYInvalid_B);

  /* Switch: '<S14>/Switch' incorporates:
   *  Constant: '<S10>/V_Parameter2'
   *  Constant: '<S10>/V_Parameter8'
   *  Sum: '<S14>/Add'
   *  UnitDelay: '<S14>/Unit Delay'
   */
  if (LDDT_UHysHeadAglCclVldLf_B) {
    rtb_LDWC_RampoutTime_Sec = LDDT_TrsdHeadAglOfst_C_Rad +
      LDDT_TrsdHeadAglMx_C_Rad;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDDT_TrsdHeadAglMx_C_Rad;
  }

  /* End of Switch: '<S14>/Switch' */

  /* RelationalOperator: '<S14>/GreaterThan' incorporates:
   *  Abs: '<S10>/Abs2'
   *  UnitDelay: '<S14>/Unit Delay'
   */
  LDDT_UHysHeadAglCclVldLf_B = (rtb_LDWC_RampoutTime_Sec > fabsf
    (LDDT_LnHeadLf_Rad));

  /* RelationalOperator: '<S10>/Relational Operator5' incorporates:
   *  Constant: '<S10>/Constant3'
   *  Constant: '<S10>/V_Parameter13'
   *  Inport: '<Root>/Inport35'
   *  S-Function (sfix_bitop): '<S10>/Bitwise Operator2'
   */
  LDDT_CclByInVldQlfrLf_B = ((((int32_T)LDWSAI_LnIVldLf_St) & ((int32_T)
    LDDT_LnIvldCclLf_C_St)) == 0);

  /* Logic: '<S10>/Logical Operator5' incorporates:
   *  UnitDelay: '<S14>/Unit Delay'
   */
  LDDT_LnCclVldLf_B = (((rtb_LDDT_LnCurvVldLf_B && LDDT_UHysHeadAglCclVldLf_B) &&
                        LDDT_CclByInVldQlfrLf_B) && rtb_LDWC_VelYInvalid_B);

  /* Switch: '<S10>/Switch' */
  if (LDDT_RdyTrigLDW_B) {
    /* Switch: '<S10>/Switch' */
    LDDT_LnMakVldLf_B = LDDT_LnTrigVldLf_B;
  } else {
    /* Switch: '<S10>/Switch' */
    LDDT_LnMakVldLf_B = LDDT_LnCclVldLf_B;
  }

  /* End of Switch: '<S10>/Switch' */

  /* Product: '<S8>/Product' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDDT_RawLatVehSpdLf_Mps = LDDT_LnHeadLf_Rad * LDWSAI_VehSpdActu_Mps;

  /* Switch: '<S8>/Switch1' */
  if (LDDT_LnMakVldLf_B) {
    /* Switch: '<S8>/Switch1' */
    LDDT_LatVehSpdLf_Mps = LDDT_RawLatVehSpdLf_Mps;
  } else {
    /* Switch: '<S8>/Switch1' incorporates:
     *  Constant: '<S8>/Constant2'
     */
    LDDT_LatVehSpdLf_Mps = 10.0F;
  }

  /* End of Switch: '<S8>/Switch1' */

  /* Switch: '<S41>/Switch' incorporates:
   *  Constant: '<S37>/V_Parameter4'
   *  Constant: '<S37>/V_Parameter5'
   *  Sum: '<S41>/Add'
   *  Sum: '<S41>/Add1'
   *  Switch: '<S41>/Switch1'
   *  UnitDelay: '<S41>/Unit Delay'
   */
  if (LDVSE_BHysLatVehSpdVldLf_B) {
    rtb_LDWC_RampoutTime_Sec = LDVSE_MaxLatVel_Mps +
      LDVSE_VehLatTrsdLDWOfst_C_Msp;
    tmp = LDVSE_VehLatTrsdLDWMn_C_Msp - LDVSE_VehLatTrsdLDWOfst_C_Msp;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDVSE_MaxLatVel_Mps;
    tmp = LDVSE_VehLatTrsdLDWMn_C_Msp;
  }

  /* End of Switch: '<S41>/Switch' */

  /* Logic: '<S41>/AND' incorporates:
   *  RelationalOperator: '<S41>/GreaterThan'
   *  RelationalOperator: '<S41>/GreaterThan1'
   *  UnaryMinus: '<S37>/Unary Minus'
   *  UnitDelay: '<S41>/Unit Delay'
   */
  LDVSE_BHysLatVehSpdVldLf_B = ((rtb_LDWC_RampoutTime_Sec >
    (-LDDT_LatVehSpdLf_Mps)) && ((-LDDT_LatVehSpdLf_Mps) > tmp));

  /* Abs: '<S37>/Abs' incorporates:
   *  Abs: '<S8>/Abs'
   *  Switch: '<S8>/Switch5'
   */
  LDVSE_UHysLatVehSpdVldLf_B_tmp = fabsf(LDDT_LatVehSpdLf_Mps);

  /* Switch: '<S42>/Switch' incorporates:
   *  Constant: '<S37>/V_Parameter1'
   *  Constant: '<S37>/V_Parameter2'
   *  Sum: '<S42>/Add'
   *  UnitDelay: '<S42>/Unit Delay'
   */
  if (LDVSE_UHysLatVehSpdVldLf_B) {
    rtb_LDWC_RampoutTime_Sec = LDVSE_VehLatTrsdLDWOfst_C_Msp +
      LDVSE_VehLatTrsdLDWMx_C_Msp;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDVSE_VehLatTrsdLDWMx_C_Msp;
  }

  /* End of Switch: '<S42>/Switch' */

  /* RelationalOperator: '<S42>/GreaterThan' incorporates:
   *  Abs: '<S37>/Abs'
   *  UnitDelay: '<S42>/Unit Delay'
   */
  LDVSE_UHysLatVehSpdVldLf_B = (rtb_LDWC_RampoutTime_Sec >
    LDVSE_UHysLatVehSpdVldLf_B_tmp);

  /* Switch: '<S37>/Switch1' */
  if (LDVSE_RdyTrigLDW_B) {
    /* Switch: '<S37>/Switch1' incorporates:
     *  UnitDelay: '<S41>/Unit Delay'
     */
    LDVSE_VehLatSpdVldLf_B = LDVSE_BHysLatVehSpdVldLf_B;
  } else {
    /* Switch: '<S37>/Switch1' incorporates:
     *  UnitDelay: '<S42>/Unit Delay'
     */
    LDVSE_VehLatSpdVldLf_B = LDVSE_UHysLatVehSpdVldLf_B;
  }

  /* End of Switch: '<S37>/Switch1' */

  /* Logic: '<S22>/Logical Operator2' */
  rtb_VectorConcatenate[1] = !LDVSE_VehLatSpdVldLf_B;

  /* RelationalOperator: '<S21>/Relational Operator2' incorporates:
   *  Constant: '<S21>/V_Parameter1'
   *  Inport: '<Root>/Inport6'
   */
  rtb_RelationalOperator2 = (LDWSAI_TrnSgl_St == LDVSE_TrnSglLf_C_St);

  /* RelationalOperator: '<S21>/Relational Operator1' incorporates:
   *  Constant: '<S21>/V_Parameter'
   *  Inport: '<Root>/Inport6'
   */
  rtb_RelationalOperator1 = (LDWSAI_TrnSgl_St == LDVSE_TrnSglRi_C_St);

  /* Switch: '<S21>/Switch3' incorporates:
   *  Constant: '<S21>/V_Const'
   *  Constant: '<S21>/V_Parameter2'
   *  Logic: '<S33>/AND'
   *  Logic: '<S33>/NOT'
   *  UnitDelay: '<S33>/Unit Delay'
   */
  if (LDVSE_TrnSglRstLfEn_C_B) {
    rtb_RelationalOperator29 = (rtb_RelationalOperator1 &&
      (!LDVSE_EdgeRisTrnSglRi_B));
  } else {
    rtb_RelationalOperator29 = false;
  }

  /* End of Switch: '<S21>/Switch3' */

  /* Switch: '<S35>/Switch4' incorporates:
   *  Switch: '<S35>/Switch3'
   */
  if (rtb_RelationalOperator29) {
    /* Sum: '<S35>/Subtract2' incorporates:
     *  Constant: '<S35>/Constant3'
     */
    LDVSE_HodTiTrnSglLf_Sec = 0.0F;
  } else {
    if (rtb_RelationalOperator2) {
      /* Sum: '<S35>/Subtract2' incorporates:
       *  Constant: '<S21>/V_Parameter3'
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S35>/Subtract1'
       *  Switch: '<S35>/Switch3'
       */
      LDVSE_HodTiTrnSglLf_Sec = LDVSE_HodTiTrnSgl_C_Sec + LDWSAI_CycleTime_Sec;
    }
  }

  /* End of Switch: '<S35>/Switch4' */

  /* SignalConversion: '<S21>/Signal Conversion3' incorporates:
   *  Constant: '<S35>/Constant4'
   *  Logic: '<S35>/OR'
   *  RelationalOperator: '<S35>/GreaterThan2'
   */
  LDVSE_TrnSglLf_B = (rtb_RelationalOperator2 || (LDVSE_HodTiTrnSglLf_Sec >
    1.0E-5F));

  /* SignalConversion: '<S22>/Signal Conversion1' */
  rtb_VectorConcatenate[0] = LDVSE_TrnSglLf_B;

  /* Switch: '<S45>/Switch' incorporates:
   *  Constant: '<S40>/V_Parameter10'
   *  Constant: '<S40>/V_Parameter11'
   *  Sum: '<S45>/Add'
   *  Sum: '<S45>/Add1'
   *  Switch: '<S45>/Switch1'
   *  UnitDelay: '<S45>/Unit Delay'
   */
  if (LDVSE_BHysLatVehSpdVldRi_B) {
    rtb_LDWC_RampoutTime_Sec = LDVSE_MaxLatVel_Mps +
      LDVSE_VehLatTrsdLDWOfst_C_Msp;
    tmp = LDVSE_VehLatTrsdLDWMn_C_Msp - LDVSE_VehLatTrsdLDWOfst_C_Msp;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDVSE_MaxLatVel_Mps;
    tmp = LDVSE_VehLatTrsdLDWMn_C_Msp;
  }

  /* End of Switch: '<S45>/Switch' */

  /* Logic: '<S45>/AND' incorporates:
   *  RelationalOperator: '<S45>/GreaterThan'
   *  RelationalOperator: '<S45>/GreaterThan1'
   *  UnitDelay: '<S45>/Unit Delay'
   */
  LDVSE_BHysLatVehSpdVldRi_B = ((rtb_LDWC_RampoutTime_Sec > LDDT_LatVehSpdRi_Mps)
    && (LDDT_LatVehSpdRi_Mps > tmp));

  /* Abs: '<S40>/Abs1' incorporates:
   *  Abs: '<S8>/Abs1'
   *  Switch: '<S8>/Switch6'
   */
  LDVSE_UHysLatVehSpdVldRi_B_tmp = fabsf(LDDT_LatVehSpdRi_Mps);

  /* Switch: '<S46>/Switch' incorporates:
   *  Constant: '<S40>/V_Parameter7'
   *  Constant: '<S40>/V_Parameter8'
   *  Sum: '<S46>/Add'
   *  UnitDelay: '<S46>/Unit Delay'
   */
  if (LDVSE_UHysLatVehSpdVldRi_B) {
    rtb_LDWC_RampoutTime_Sec = LDVSE_VehLatTrsdLDWOfst_C_Msp +
      LDVSE_VehLatTrsdLDWMx_C_Msp;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDVSE_VehLatTrsdLDWMx_C_Msp;
  }

  /* End of Switch: '<S46>/Switch' */

  /* RelationalOperator: '<S46>/GreaterThan' incorporates:
   *  Abs: '<S40>/Abs1'
   *  UnitDelay: '<S46>/Unit Delay'
   */
  LDVSE_UHysLatVehSpdVldRi_B = (rtb_LDWC_RampoutTime_Sec >
    LDVSE_UHysLatVehSpdVldRi_B_tmp);

  /* Switch: '<S40>/Switch1' */
  if (LDVSE_RdyTrigLDW_B) {
    /* Switch: '<S40>/Switch1' incorporates:
     *  UnitDelay: '<S45>/Unit Delay'
     */
    LDVSE_VehLatSpdVldRi_B = LDVSE_BHysLatVehSpdVldRi_B;
  } else {
    /* Switch: '<S40>/Switch1' incorporates:
     *  UnitDelay: '<S46>/Unit Delay'
     */
    LDVSE_VehLatSpdVldRi_B = LDVSE_UHysLatVehSpdVldRi_B;
  }

  /* End of Switch: '<S40>/Switch1' */

  /* Logic: '<S22>/Logical Operator1' */
  rtb_VectorConcatenate_grcm[1] = !LDVSE_VehLatSpdVldRi_B;

  /* RelationalOperator: '<S21>/Relational Operator4' incorporates:
   *  Constant: '<S21>/V_Parameter5'
   *  Inport: '<Root>/Inport6'
   */
  rtb_RelationalOperator4 = (LDWSAI_TrnSgl_St == LDVSE_TrnSglRi_C_St);

  /* RelationalOperator: '<S21>/Relational Operator3' incorporates:
   *  Constant: '<S21>/V_Parameter4'
   *  Inport: '<Root>/Inport6'
   */
  rtb_RelationalOperator2 = (LDWSAI_TrnSgl_St == LDVSE_TrnSglLf_C_St);

  /* Switch: '<S21>/Switch1' incorporates:
   *  Constant: '<S21>/V_Const1'
   *  Constant: '<S21>/V_Parameter6'
   *  Logic: '<S34>/AND'
   *  Logic: '<S34>/NOT'
   *  UnitDelay: '<S34>/Unit Delay'
   */
  if (LDVSE_TrnSglRstRiEn_C_B) {
    rtb_RelationalOperator29 = (rtb_RelationalOperator2 &&
      (!LDVSE_EdgeRisTrnSglLf_B));
  } else {
    rtb_RelationalOperator29 = false;
  }

  /* End of Switch: '<S21>/Switch1' */

  /* Switch: '<S36>/Switch4' incorporates:
   *  Switch: '<S36>/Switch3'
   */
  if (rtb_RelationalOperator29) {
    /* Sum: '<S36>/Subtract2' incorporates:
     *  Constant: '<S36>/Constant3'
     */
    LDVSE_HodTiTrnSglRi_Sec = 0.0F;
  } else {
    if (rtb_RelationalOperator4) {
      /* Sum: '<S36>/Subtract2' incorporates:
       *  Constant: '<S21>/V_Parameter7'
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S36>/Subtract1'
       *  Switch: '<S36>/Switch3'
       */
      LDVSE_HodTiTrnSglRi_Sec = LDVSE_HodTiTrnSgl_C_Sec + LDWSAI_CycleTime_Sec;
    }
  }

  /* End of Switch: '<S36>/Switch4' */

  /* SignalConversion: '<S21>/Signal Conversion4' incorporates:
   *  Constant: '<S36>/Constant4'
   *  Logic: '<S36>/OR'
   *  RelationalOperator: '<S36>/GreaterThan2'
   */
  LDVSE_TrnSglRi_B = (rtb_RelationalOperator4 || (LDVSE_HodTiTrnSglRi_Sec >
    1.0E-5F));

  /* SignalConversion: '<S22>/Signal Conversion2' */
  rtb_VectorConcatenate_grcm[0] = LDVSE_TrnSglRi_B;

  /* Switch: '<S27>/Switch' incorporates:
   *  Constant: '<S20>/V_Parameter3'
   *  Constant: '<S20>/V_Parameter4'
   *  Sum: '<S27>/Add'
   *  UnitDelay: '<S27>/Unit Delay'
   */
  if (LDVSE_UHysSteAgl_B) {
    rtb_LDWC_RampoutTime_Sec = LDVSE_SteAglTrsdOfst_C_Dgr +
      LDVSE_SteAglTrsdMx_C_Dgr;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDVSE_SteAglTrsdMx_C_Dgr;
  }

  /* End of Switch: '<S27>/Switch' */

  /* RelationalOperator: '<S27>/GreaterThan' incorporates:
   *  Abs: '<S20>/Abs1'
   *  Inport: '<Root>/Inport7'
   *  UnitDelay: '<S27>/Unit Delay'
   */
  LDVSE_UHysSteAgl_B = (rtb_LDWC_RampoutTime_Sec >= fabsf(LDWSAI_WheSteAgl_Dgr));

  /* Logic: '<S20>/Logical Operator1' incorporates:
   *  UnitDelay: '<S27>/Unit Delay'
   */
  rtb_VectorConcatenate_oy1j[1] = !LDVSE_UHysSteAgl_B;

  /* UnitDelay: '<S31>/Unit Delay' */
  rtb_UnitDelay_btcf = LDVSE_PrevVehStartupSpd_Kmph;

  /* DataTypeConversion: '<S31>/Data Type Conversion' incorporates:
   *  Inport: '<Root>/Inport50'
   *  UnitDelay: '<S31>/Unit Delay'
   */
  LDVSE_PrevVehStartupSpd_Kmph = (uint8_T)LDWSAI_VehStartupSpdHMI_Kmph;

  /* MultiPortSwitch: '<S31>/Multiport Switch1' incorporates:
   *  UnitDelay: '<S31>/Unit Delay'
   */
  if (((int32_T)rtb_UnitDelay_btcf) == 0) {
    /* MultiPortSwitch: '<S31>/Multiport Switch2' incorporates:
     *  Constant: '<S31>/V_Parameter1'
     *  DataTypeConversion: '<S31>/Data Type Conversion1'
     *  DataTypeConversion: '<S31>/Data Type Conversion2'
     *  Inport: '<Root>/Inport49'
     */
    if (((int32_T)((uint8_T)NVRAM_LDWStartupSpd_Kmph)) == 0) {
      rtb_UnitDelay_btcf = (uint8_T)LDVSE_VehSpdTrsdMn_C_Kmph;
    } else {
      rtb_UnitDelay_btcf = (uint8_T)NVRAM_LDWStartupSpd_Kmph;
    }

    /* End of MultiPortSwitch: '<S31>/Multiport Switch2' */
  } else {
    rtb_UnitDelay_btcf = LDVSE_PrevVehStartupSpd_Kmph;
  }

  /* End of MultiPortSwitch: '<S31>/Multiport Switch1' */

  /* Switch: '<S31>/Switch2' incorporates:
   *  Constant: '<S31>/V_Parameter5'
   *  Constant: '<S31>/V_Parameter6'
   *  DataTypeConversion: '<S31>/Data Type Conversion3'
   *  RelationalOperator: '<S31>/GreaterThanOrEqual'
   *  RelationalOperator: '<S31>/GreaterThanOrEqual1'
   *  Switch: '<S31>/Switch3'
   */
  if (60 <= ((int32_T)rtb_UnitDelay_btcf)) {
    /* Switch: '<S31>/Switch2' */
    LDVSE_NVRAMVehStartupSpd_k_chkk = 60;
  } else if (45 >= ((int32_T)rtb_UnitDelay_btcf)) {
    /* Switch: '<S31>/Switch3' incorporates:
     *  Constant: '<S31>/V_Parameter6'
     *  Switch: '<S31>/Switch2'
     */
    LDVSE_NVRAMVehStartupSpd_k_chkk = 45;
  } else {
    /* Switch: '<S31>/Switch2' */
    LDVSE_NVRAMVehStartupSpd_k_chkk = (int32_T)rtb_UnitDelay_btcf;
  }

  /* End of Switch: '<S31>/Switch2' */

  /* Switch: '<S23>/Switch' incorporates:
   *  Constant: '<S20>/V_Parameter'
   *  Constant: '<S20>/V_Parameter2'
   *  Sum: '<S23>/Add'
   *  Sum: '<S23>/Add1'
   *  Switch: '<S23>/Switch1'
   *  UnitDelay: '<S23>/Unit Delay'
   */
  if (LDVSE_BHysSpdVeh_B) {
    rtb_LDWC_RampoutTime_Sec = LDVSE_VehSpdTrsdMx_C_Kmph +
      LDVSE_VehSpdTrsdOfst_C_Kmph;
    tmp = ((real32_T)LDVSE_NVRAMVehStartupSpd_k_chkk) -
      LDVSE_VehSpdTrsdOfst_C_Kmph;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDVSE_VehSpdTrsdMx_C_Kmph;
    tmp = (real32_T)LDVSE_NVRAMVehStartupSpd_k_chkk;
  }

  /* End of Switch: '<S23>/Switch' */

  /* Logic: '<S23>/AND' incorporates:
   *  Inport: '<Root>/Inport2'
   *  RelationalOperator: '<S23>/GreaterThan'
   *  RelationalOperator: '<S23>/GreaterThan1'
   *  UnitDelay: '<S23>/Unit Delay'
   */
  LDVSE_BHysSpdVeh_B = ((rtb_LDWC_RampoutTime_Sec >= LDWSAI_SpdVelShow_Kmph) &&
                        (LDWSAI_SpdVelShow_Kmph >= tmp));

  /* Logic: '<S20>/Logical Operator2' incorporates:
   *  UnitDelay: '<S23>/Unit Delay'
   */
  rtb_VectorConcatenate_oy1j[0] = !LDVSE_BHysSpdVeh_B;

  /* Switch: '<S28>/Switch' incorporates:
   *  Constant: '<S20>/V_Parameter5'
   *  Constant: '<S20>/V_Parameter6'
   *  Sum: '<S28>/Add'
   *  UnitDelay: '<S28>/Unit Delay'
   */
  if (LDVSE_UHysSteAglSpd_B) {
    rtb_LDWC_RampoutTime_Sec = LDVSE_SteAglSpdTrsdOfst_C_Dgpm +
      LDVSE_SteAglSpdTrsdMx_C_Dgpm;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDVSE_SteAglSpdTrsdMx_C_Dgpm;
  }

  /* End of Switch: '<S28>/Switch' */

  /* RelationalOperator: '<S28>/GreaterThan' incorporates:
   *  Abs: '<S20>/Abs2'
   *  Inport: '<Root>/Inport8'
   *  UnitDelay: '<S28>/Unit Delay'
   */
  LDVSE_UHysSteAglSpd_B = (rtb_LDWC_RampoutTime_Sec >= fabsf
    (LDWSAI_SteAglSpd_Dgpm));

  /* Logic: '<S20>/Logical Operator3' incorporates:
   *  UnitDelay: '<S28>/Unit Delay'
   */
  rtb_VectorConcatenate_oy1j[2] = !LDVSE_UHysSteAglSpd_B;

  /* Switch: '<S24>/Switch' incorporates:
   *  Constant: '<S20>/V_Parameter10'
   *  Constant: '<S20>/V_Parameter11'
   *  Constant: '<S20>/V_Parameter12'
   *  Sum: '<S24>/Add'
   *  Sum: '<S24>/Add1'
   *  Switch: '<S24>/Switch1'
   *  UnitDelay: '<S24>/Unit Delay'
   */
  if (LDVSE_BHysAccVehX_B) {
    rtb_LDWC_RampoutTime_Sec = LDVSE_VehAccSpdTrsdXMx_C_Npkg +
      LDVSE_VehAccSpdTrsdXOfst_C_Npkg;
    tmp = LDVSE_VehAccSpdTrsdXMn_C_Npkg - LDVSE_VehAccSpdTrsdXOfst_C_Npkg;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDVSE_VehAccSpdTrsdXMx_C_Npkg;
    tmp = LDVSE_VehAccSpdTrsdXMn_C_Npkg;
  }

  /* End of Switch: '<S24>/Switch' */

  /* Logic: '<S24>/AND' incorporates:
   *  Inport: '<Root>/Inport3'
   *  RelationalOperator: '<S24>/GreaterThan'
   *  RelationalOperator: '<S24>/GreaterThan1'
   *  UnitDelay: '<S24>/Unit Delay'
   */
  LDVSE_BHysAccVehX_B = ((rtb_LDWC_RampoutTime_Sec >= LDWSAI_VehAccSpdX_Npkg) &&
    (LDWSAI_VehAccSpdX_Npkg >= tmp));

  /* Logic: '<S20>/Logical Operator5' incorporates:
   *  UnitDelay: '<S24>/Unit Delay'
   */
  rtb_VectorConcatenate_oy1j[3] = !LDVSE_BHysAccVehX_B;

  /* Switch: '<S29>/Switch' incorporates:
   *  Constant: '<S20>/V_Parameter13'
   *  Constant: '<S20>/V_Parameter14'
   *  Sum: '<S29>/Add'
   *  UnitDelay: '<S29>/Unit Delay'
   */
  if (LDVSE_BHysAccVehY_B) {
    rtb_LDWC_RampoutTime_Sec = LDVSE_VehAccSpdTrsdYOfst_C_Npkg +
      LDVSE_VehAccSpdTrsdYMx_C_Npkg;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDVSE_VehAccSpdTrsdYMx_C_Npkg;
  }

  /* End of Switch: '<S29>/Switch' */

  /* RelationalOperator: '<S29>/GreaterThan' incorporates:
   *  Abs: '<S20>/Abs'
   *  Inport: '<Root>/Inport4'
   *  UnitDelay: '<S29>/Unit Delay'
   */
  LDVSE_BHysAccVehY_B = (rtb_LDWC_RampoutTime_Sec >= fabsf
    (LDWSAI_VehAccSpdY_Npkg));

  /* Logic: '<S20>/Logical Operator6' incorporates:
   *  UnitDelay: '<S29>/Unit Delay'
   */
  rtb_VectorConcatenate_oy1j[4] = !LDVSE_BHysAccVehY_B;

  /* Lookup_n-D: '<S20>/Lookup Table' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDVSE_MaxCrvBySpd_ReMi = look1_iflf_binlxpw(LDWSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDVSE_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDVSE_TrsdLnCltdCurvMx_Cr_Mps[0])), 7U);

  /* Lookup_n-D: '<S20>/Lookup Table1' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDVSE_HystCrvBySpd_ReMi = look1_iflf_binlxpw(LDWSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDVSE_VehSpdX_BX_Mps[0])), ((const real32_T *)
    &(LDVSE_TrsdLnCltdCurvOfst_Cr_Mps[0])), 7U);

  /* Switch: '<S30>/Switch' incorporates:
   *  Sum: '<S30>/Add'
   *  UnitDelay: '<S30>/Unit Delay'
   */
  if (LDVSE_UHysVehCurv_B) {
    rtb_LDWC_RampoutTime_Sec = LDVSE_HystCrvBySpd_ReMi + LDVSE_MaxCrvBySpd_ReMi;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDVSE_MaxCrvBySpd_ReMi;
  }

  /* End of Switch: '<S30>/Switch' */

  /* RelationalOperator: '<S30>/GreaterThan' incorporates:
   *  Inport: '<Root>/Inport5'
   *  UnitDelay: '<S30>/Unit Delay'
   */
  LDVSE_UHysVehCurv_B = (rtb_LDWC_RampoutTime_Sec >= LDWSAI_VehCurv_ReMi);

  /* Logic: '<S20>/Logical Operator7' incorporates:
   *  UnitDelay: '<S30>/Unit Delay'
   */
  rtb_VectorConcatenate_oy1j[5] = !LDVSE_UHysVehCurv_B;

  /* Switch: '<S25>/Switch' incorporates:
   *  Constant: '<S20>/V_Parameter7'
   *  Constant: '<S20>/V_Parameter8'
   *  Constant: '<S20>/V_Parameter9'
   *  Sum: '<S25>/Add'
   *  Sum: '<S25>/Add1'
   *  Switch: '<S25>/Switch1'
   *  UnitDelay: '<S25>/Unit Delay'
   */
  if (LDVSE_BHysLnWid_B) {
    rtb_LDWC_RampoutTime_Sec = LDVSE_LnWidTrsdMx_C_Mi + LDVSE_LnWidTrsdOfst_C_Mi;
    tmp = LDVSE_LnWidTrsdMn_C_Mi - LDVSE_LnWidTrsdOfst_C_Mi;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDVSE_LnWidTrsdMx_C_Mi;
    tmp = LDVSE_LnWidTrsdMn_C_Mi;
  }

  /* End of Switch: '<S25>/Switch' */

  /* Logic: '<S25>/AND' incorporates:
   *  Inport: '<Root>/Inport13'
   *  RelationalOperator: '<S25>/GreaterThan'
   *  RelationalOperator: '<S25>/GreaterThan1'
   *  UnitDelay: '<S25>/Unit Delay'
   */
  LDVSE_BHysLnWid_B = ((rtb_LDWC_RampoutTime_Sec >= LDWSAI_LnWidCalc_Mi) &&
                       (LDWSAI_LnWidCalc_Mi >= tmp));

  /* Switch: '<S20>/Switch' incorporates:
   *  Constant: '<S20>/V_Const'
   *  Logic: '<S20>/Logical Operator'
   *  Logic: '<S20>/Logical Operator4'
   *  UnitDelay: '<S25>/Unit Delay'
   */
  if (LDDT_LnMakVldLf_B && LDDT_LnMakVldRi_B) {
    rtb_VectorConcatenate_oy1j[6] = !LDVSE_BHysLnWid_B;
  } else {
    rtb_VectorConcatenate_oy1j[6] = false;
  }

  /* End of Switch: '<S20>/Switch' */

  /* Switch: '<S20>/Switch1' incorporates:
   *  Inport: '<Root>/Inport9'
   *  Logic: '<S20>/Logical Operator8'
   */
  rtb_VectorConcatenate_oy1j[7] = !LDWSAI_LDWSwitchEn_B;

  /* S-Function (ex_sfun_set_bit): '<S32>/ex_sfun_set_bit' incorporates:
   *  Constant: '<S26>/Constant'
   */
  set_bit(0U, (boolean_T*)&rtb_VectorConcatenate_oy1j[0], (uint8_T*)
          (&(LDWSA_SetBit_BS_Param_1[0])), ((uint8_T)8U), &rtb_ex_sfun_set_bit);

  /* S-Function (ex_sfun_set_bit): '<S43>/ex_sfun_set_bit' incorporates:
   *  Constant: '<S38>/Constant'
   */
  set_bit(0U, (boolean_T*)&rtb_VectorConcatenate_grcm[0], (uint8_T*)
          (&(LDWSA_SetBit_BS_Param_2[0])), ((uint8_T)2U),
          &rtb_ex_sfun_set_bit_czhm);

  /* S-Function (ex_sfun_set_bit): '<S44>/ex_sfun_set_bit' incorporates:
   *  Constant: '<S39>/Constant'
   */
  set_bit(0U, (boolean_T*)&rtb_VectorConcatenate[0], (uint8_T*)
          (&(LDWSA_SetBit_BS_Param_2[0])), ((uint8_T)2U),
          &rtb_ex_sfun_set_bit_dmuc);

  /* RelationalOperator: '<S108>/Relational Operator1' incorporates:
   *  Constant: '<S108>/V_Parameter10'
   *  UnaryMinus: '<S108>/Unary Minus1'
   */
  LDWC_EnaTlcTrigRi_B = (LDDT_DstcToLnRi_Mi > (-LDWC_DstcOfTiToLnMn_C_Mi));

  /* Switch: '<S8>/Switch6' incorporates:
   *  Constant: '<S8>/Constant10'
   *  Constant: '<S8>/V_Parameter2'
   *  Inport: '<Root>/Inport1'
   *  Logic: '<S8>/Logical Operator1'
   *  RelationalOperator: '<S8>/Relational Operator2'
   *  RelationalOperator: '<S8>/Relational Operator3'
   */
  if (((LDDT_LnHeadRi_Rad > LDDT_TLCHeadAglTrsd_C_Rad) && (LDWSAI_VehSpdActu_Mps
        > 0.0F)) && LDDT_LnMakVldRi_B) {
    /* Switch: '<S8>/Switch6' incorporates:
     *  Constant: '<S8>/Constant11'
     *  Constant: '<S8>/Constant9'
     *  MinMax: '<S8>/MinMax3'
     *  MinMax: '<S8>/MinMax4'
     *  Product: '<S8>/Divide2'
     *  UnaryMinus: '<S8>/Unary Minus1'
     */
    LDDT_TiToLnRi_Sec = fminf((-LDDT_DstcToLnRi_Mi) / fmaxf
      (LDVSE_UHysLatVehSpdVldRi_B_tmp, 0.01F), 10.0F);
  } else {
    /* Switch: '<S8>/Switch6' incorporates:
     *  Constant: '<S8>/Constant12'
     */
    LDDT_TiToLnRi_Sec = 10.0F;
  }

  /* Product: '<S106>/Product1' incorporates:
   *  Inport: '<Root>/Inport1'
   *  Inport: '<Root>/Inport13'
   *  Lookup_n-D: '<S106>/1-D Lookup Table5'
   *  Lookup_n-D: '<S106>/1-D Lookup Table6'
   */
  LDWC_TiToLnTrsd_Sec = look1_iflf_binlxpw(LDWSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDWC_VehSpdXTTL_BX_Mps[0])), ((const real32_T *)
    &(LDWC_TiToLnTrsdSpd_Cr_Sec[0])), 8U) * look1_iflf_binlxpw
    (LDWSAI_LnWidCalc_Mi, ((const real32_T *)&(LDWC_LaneWidth_BX_Mi[0])), ((
       const real32_T *)&(LDWC_TiToLnTrsdWdh_Cr_Sec[0])), 4U);

  /* RelationalOperator: '<S108>/Relational Operator2' */
  LDWC_RawTrigByTlcRi_B = (LDDT_TiToLnRi_Sec < LDWC_TiToLnTrsd_Sec);

  /* Switch: '<S134>/Switch' incorporates:
   *  Constant: '<S108>/V_Parameter12'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S134>/Max'
   *  Sum: '<S134>/Subtract'
   *  Switch: '<S134>/Switch1'
   *  UnaryMinus: '<S134>/Unary Minus'
   *  UnitDelay: '<S134>/Unit Delay'
   */
  if (LDWC_RawTrigByTlcRi_B) {
    LDWC_DlyTiOfTiToLnRiMn_Sec = fmaxf(LDWC_DlyTiOfTiToLnRiMn_Sec,
      -LDWSAI_CycleTime_Sec) - LDWSAI_CycleTime_Sec;
  } else {
    LDWC_DlyTiOfTiToLnRiMn_Sec = LDWC_DlyTiOfTiToLnMn_C_Sec;
  }

  /* End of Switch: '<S134>/Switch' */

  /* Logic: '<S134>/AND' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S134>/LessThanOrEqual'
   *  UnaryMinus: '<S134>/Unary Minus1'
   *  UnitDelay: '<S134>/Unit Delay'
   */
  LDWC_DlyTrigByTlcRi_B = (LDWC_RawTrigByTlcRi_B && (LDWC_DlyTiOfTiToLnRiMn_Sec <=
    (-LDWSAI_CycleTime_Sec)));

  /* Logic: '<S108>/Logical Operator2' incorporates:
   *  Constant: '<S108>/V_Parameter11'
   *  Constant: '<S108>/V_Parameter9'
   *  DataTypeConversion: '<S124>/Data Type Conversion'
   *  DataTypeConversion: '<S125>/Data Type Conversion'
   *  Logic: '<S108>/Logical Operator'
   *  Logic: '<S108>/Logical Operator1'
   *  S-Function (sfix_bitop): '<S124>/Bitwise AND'
   *  S-Function (sfix_bitop): '<S125>/Bitwise AND'
   */
  rtb_RelationalOperator4 = ((((((uint32_T)LDWC_TrigCdtnEn_C_St) & 4U) != 0U) &&
    LDWC_RawTrigByDlcRi_B) || ((((((uint32_T)LDWC_TrigCdtnEn_C_St) & 8U) != 0U) &&
    LDWC_EnaTlcTrigRi_B) && LDWC_DlyTrigByTlcRi_B));

  /* Logic: '<S123>/AND' incorporates:
   *  Logic: '<S123>/NOT'
   *  UnitDelay: '<S123>/Unit Delay'
   */
  LDWC_EnaLdwTrigRi_B = (rtb_RelationalOperator4 && (!LDWC_HdTiTrigRiEn_B));

  /* Logic: '<S108>/Logical Operator15' incorporates:
   *  Constant: '<S108>/V_Parameter14'
   *  Constant: '<S108>/V_Parameter5'
   *  RelationalOperator: '<S108>/Relational Operator11'
   *  RelationalOperator: '<S108>/Relational Operator4'
   *  Sum: '<S108>/Add2'
   *  UnaryMinus: '<S108>/Unary Minus2'
   *  UnaryMinus: '<S108>/Unary Minus6'
   */
  LDWC_RstTlcTrigRi_B = ((((-LDWC_DstcToLnTrsdRi_Mi) - 0.12F) >
    LDDT_DstcToLnRi_Mi) || (LDDT_DstcToLnRi_Mi > (-LDWC_DstcOfDiscToLnLmtMn_C_Mi)));

  /* Switch: '<S132>/Switch4' incorporates:
   *  Switch: '<S132>/Switch3'
   */
  if (LDWC_RstTlcTrigRi_B) {
    /* Switch: '<S106>/Switch3' incorporates:
     *  Constant: '<S132>/Constant3'
     */
    LDWC_HdTiTrigRi_Sec = 0.0F;
  } else {
    if (LDWC_EnaLdwTrigRi_B) {
      /* Switch: '<S106>/Switch3' incorporates:
       *  Constant: '<S108>/V_Parameter15'
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S132>/Subtract1'
       *  Switch: '<S132>/Switch3'
       */
      LDWC_HdTiTrigRi_Sec = LDWC_HdTiTrigLf_C_Sec + LDWSAI_CycleTime_Sec;
    }
  }

  /* End of Switch: '<S132>/Switch4' */

  /* Logic: '<S108>/Logical Operator5' incorporates:
   *  Logic: '<S66>/Logical Operator21'
   */
  LDWC_ErrSideByTrigRi_B = !LDDT_LnMakVldRi_B;

  /* Logic: '<S108>/Logical Operator4' incorporates:
   *  Constant: '<S128>/Constant'
   *  Inport: '<Root>/Inport12'
   *  Logic: '<S108>/Logical Operator5'
   *  RelationalOperator: '<S108>/Relational Operator5'
   */
  LDWC_ResetForSafeRi_B = ((LDWSAI_DtctLnChag_B || LDWC_ErrSideByTrigRi_B) ||
    (E_LDWState_nu_ACTIVE == ((uint32_T)LDWC_SysOld_St)));

  /* RelationalOperator: '<S108>/Relational Operator6' incorporates:
   *  Constant: '<S108>/V_Parameter13'
   *  Sum: '<S108>/Add'
   *  UnaryMinus: '<S108>/Unary Minus3'
   */
  LDWC_SetForSafeRi_B = (LDDT_DstcToLnRi_Mi < ((-LDWC_DstcToLnTrsdRi_Mi) -
    LDWC_DstcOfstSafeSitu_C_Mi));

  /* Switch: '<S130>/Switch' incorporates:
   *  Constant: '<S130>/Constant2'
   *  Switch: '<S130>/Switch1'
   *  UnitDelay: '<S130>/Unit Delay'
   */
  if (LDWC_ResetForSafeRi_B) {
    LDWC_DisTrigRi_B = false;
  } else {
    LDWC_DisTrigRi_B = (LDWC_SetForSafeRi_B || LDWC_DisTrigRi_B);
  }

  /* End of Switch: '<S130>/Switch' */

  /* Logic: '<S108>/Logical Operator6' incorporates:
   *  Constant: '<S132>/Constant4'
   *  Logic: '<S132>/OR'
   *  RelationalOperator: '<S132>/GreaterThan2'
   *  UnitDelay: '<S130>/Unit Delay'
   */
  rtb_LDWC_VehicleInvalid_B = ((LDWC_EnaLdwTrigRi_B || (LDWC_HdTiTrigRi_Sec >
    1.0E-5F)) && LDWC_DisTrigRi_B);

  /* Logic: '<S108>/Logical Operator8' incorporates:
   *  Constant: '<S108>/V_Parameter3'
   *  RelationalOperator: '<S108>/Relational Operator8'
   *  RelationalOperator: '<S108>/Relational Operator9'
   *  UnaryMinus: '<S108>/Unary Minus4'
   *  UnaryMinus: '<S108>/Unary Minus5'
   */
  rtb_LDWC_VelYInvalid_B = ((LDDT_DstcToLnRi_Mi > (-LDWC_DstcToLnTrsdRi_Mi)) &&
    (LDDT_DstcToLnRi_Mi < (-LDWC_DstcOfDiscToLnLmtMn_C_Mi)));

  /* RelationalOperator: '<S108>/Relational Operator10' incorporates:
   *  Constant: '<S129>/Constant'
   */
  rtb_RelationalOperator10 = (((uint32_T)LDWC_SysOld_St) == E_LDWState_nu_ACTIVE);

  /* Logic: '<S108>/Logical Operator11' incorporates:
   *  Logic: '<S108>/AND'
   */
  rtb_RelationalOperator29 = !rtb_LDWC_VelYInvalid_B;

  /* Switch: '<S108>/Switch1' incorporates:
   *  Constant: '<S108>/Constant3'
   *  Logic: '<S108>/Logical Operator10'
   *  Logic: '<S108>/Logical Operator11'
   *  Logic: '<S126>/AND'
   *  Logic: '<S126>/NOT'
   *  Sum: '<S108>/Add1'
   *  UnitDelay: '<S108>/Unit Delay'
   *  UnitDelay: '<S108>/Unit Delay2'
   *  UnitDelay: '<S126>/Unit Delay'
   */
  if (LDWC_SuppFlagOldRi_B || rtb_RelationalOperator29) {
    LDWC_ContinWarmTimesOldRi_Count = 0.0F;
  } else {
    LDWC_ContinWarmTimesOldRi_Count += (real32_T)((rtb_RelationalOperator10 && (
      !LDWC_PreActiveEdgeRi)) ? 1 : 0);
  }

  /* End of Switch: '<S108>/Switch1' */

  /* RelationalOperator: '<S108>/Relational Operator12' incorporates:
   *  Constant: '<S108>/V_Parameter4'
   *  UnitDelay: '<S108>/Unit Delay'
   *  UnitDelay: '<S108>/Unit Delay2'
   */
  LDWC_SuppFlagOldRi_B = (LDWC_ContinWarmTimesOldRi_Count >=
    LDWC_ContinWarmTimes_C_Count);

  /* Switch: '<S133>/Switch4' incorporates:
   *  Logic: '<S108>/Logical Operator7'
   *  Switch: '<S133>/Switch3'
   *  UnitDelay: '<S108>/Unit Delay'
   */
  if (!rtb_LDWC_VelYInvalid_B) {
    /* Sum: '<S133>/Subtract2' incorporates:
     *  Constant: '<S133>/Constant3'
     */
    LDWC_SuppTimeOldRi_Sec = 0.0F;
  } else {
    if (LDWC_SuppFlagOldRi_B) {
      /* Sum: '<S133>/Subtract2' incorporates:
       *  Constant: '<S108>/V_Parameter1'
       *  Constant: '<S108>/V_Parameter2'
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S108>/Add3'
       *  Sum: '<S133>/Subtract1'
       *  Switch: '<S133>/Switch3'
       */
      LDWC_SuppTimeOldRi_Sec = (LDWC_ContinWarmSupp_C_Sec + LDWC_WarmMxTi_C_Sec)
        + LDWSAI_CycleTime_Sec;
    }
  }

  /* End of Switch: '<S133>/Switch4' */

  /* Logic: '<S108>/Logical Operator14' incorporates:
   *  Constant: '<S133>/Constant4'
   *  Logic: '<S133>/OR'
   *  RelationalOperator: '<S133>/GreaterThan2'
   *  UnitDelay: '<S108>/Unit Delay'
   */
  rtb_LogicalOperator14 = ((!LDWC_SuppFlagOldRi_B) && (LDWC_SuppTimeOldRi_Sec <=
    1.0E-5F));

  /* Logic: '<S108>/Logical Operator13' incorporates:
   *  Logic: '<S127>/AND'
   *  Logic: '<S127>/NOT'
   *  UnitDelay: '<S127>/Unit Delay'
   */
  LDWC_SetForContinTrigRi_B = (rtb_LDWC_VehicleInvalid_B ||
    (rtb_LogicalOperator14 && (!LDWC_ContinTrigRiEn_B)));

  /* Logic: '<S108>/Logical Operator9' incorporates:
   *  Logic: '<S108>/AND'
   */
  LDWC_ResetForContinTrigRi_B = (rtb_RelationalOperator29 ||
    (!rtb_LogicalOperator14));

  /* Switch: '<S131>/Switch' incorporates:
   *  Constant: '<S131>/Constant2'
   *  Switch: '<S131>/Switch1'
   *  UnitDelay: '<S131>/Unit Delay'
   */
  if (LDWC_ResetForContinTrigRi_B) {
    LDWC_DisContinTrigRi_B = false;
  } else {
    LDWC_DisContinTrigRi_B = (LDWC_SetForContinTrigRi_B ||
      LDWC_DisContinTrigRi_B);
  }

  /* End of Switch: '<S131>/Switch' */

  /* SignalConversion: '<S22>/Signal Conversion4' incorporates:
   *  DataTypeConversion: '<S43>/Data Type Conversion1'
   */
  LDVSE_SidCdtnLDWRi_St = (uint8_T)rtb_ex_sfun_set_bit_czhm;

  /* RelationalOperator: '<S108>/Relational Operator7' incorporates:
   *  Constant: '<S108>/V_Const'
   *  Constant: '<S108>/V_Const2'
   *  S-Function (sfix_bitop): '<S108>/Bitwise AND1'
   */
  LDWC_TrigBySideCondRi_B = ((((uint32_T)LDVSE_SidCdtnLDWRi_St) & 3U) == 0U);

  /* S-Function (sfix_bitop): '<S108>/Bitwise AND' incorporates:
   *  Constant: '<S108>/V_Parameter'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S107>/Bitwise AND'
   */
  LDWC_TrigByPrjSpecRi_B_tmp = ((int32_T)LDWSAI_PrjSpecQu_St) & ((int32_T)
    LDWC_PrjSpecQu_C_St);

  /* RelationalOperator: '<S108>/Equal' incorporates:
   *  Constant: '<S108>/V_Const1'
   *  S-Function (sfix_bitop): '<S108>/Bitwise AND'
   */
  LDWC_TrigByPrjSpecRi_B = (LDWC_TrigByPrjSpecRi_B_tmp == 0);

  /* Logic: '<S108>/Logical Operator3' incorporates:
   *  Logic: '<S108>/Logical Operator12'
   *  UnitDelay: '<S131>/Unit Delay'
   */
  LDWC_TrigRi_B = (((rtb_LDWC_VehicleInvalid_B || LDWC_DisContinTrigRi_B) &&
                    LDWC_TrigBySideCondRi_B) && LDWC_TrigByPrjSpecRi_B);

  /* Switch: '<S6>/Switch2' */
  if (LDDT_EnaSafety_B) {
    /* Switch: '<S6>/Switch2' incorporates:
     *  Inport: '<Root>/Inport17'
     */
    LDDT_LnPstnLf_Mi = LDWSAI_PstnYSafeLf_Mi;
  } else {
    /* Switch: '<S6>/Switch2' incorporates:
     *  Inport: '<Root>/Inport16'
     */
    LDDT_LnPstnLf_Mi = LDWSAI_PstnYLf_Mi;
  }

  /* End of Switch: '<S6>/Switch2' */

  /* Sum: '<S8>/Subtract' */
  LDDT_RawDstcToLnLf_Mi = LDDT_LnPstnLf_Mi - rtb_Subtract2_hwoc;

  /* Switch: '<S8>/Switch3' */
  if (LDDT_LnMakVldLf_B) {
    /* Switch: '<S8>/Switch3' */
    LDDT_DstcToLnLf_Mi = LDDT_RawDstcToLnLf_Mi;
  } else {
    /* Switch: '<S8>/Switch3' incorporates:
     *  Constant: '<S8>/Constant1'
     */
    LDDT_DstcToLnLf_Mi = 10.0F;
  }

  /* End of Switch: '<S8>/Switch3' */

  /* Lookup_n-D: '<S106>/1-D Lookup Table7' incorporates:
   *  Inport: '<Root>/Inport25'
   */
  LDWC_DstcToLnTrsdCrvCpstnLf_Mi = look1_iflf_binlxpw(LDWSAI_CurvSafeLf_ReMi, ((
    const real32_T *)&(LDWC_LnDectCrvLf_BX_ReMi[0])), ((const real32_T *)
    &(LDWC_DstcToLnTrsdOfstLf_Cr_Mi[0])), 16U);

  /* Switch: '<S9>/Switch' incorporates:
   *  Constant: '<S9>/V_Parameter1'
   *  Inport: '<Root>/Inport24'
   *  RelationalOperator: '<S9>/GreaterThan2'
   *  RelationalOperator: '<S9>/Less Than'
   *  Switch: '<S9>/Switch1'
   *  UnaryMinus: '<S9>/Unary Minus'
   */
  if (LDWSAI_CurvLf_ReMi > LDDT_CurveThd_C_St) {
    /* Switch: '<S9>/Switch' incorporates:
     *  Constant: '<S9>/V_Parameter2'
     */
    LDDT_CurveTypeLe_St = LDDT_CurveInner_C_St;
  } else if (LDWSAI_CurvLf_ReMi < (-LDDT_CurveThd_C_St)) {
    /* Switch: '<S9>/Switch1' incorporates:
     *  Constant: '<S9>/V_Parameter4'
     *  Switch: '<S9>/Switch'
     */
    LDDT_CurveTypeLe_St = LDDT_CurveOuter_C_St;
  } else {
    /* Switch: '<S9>/Switch' incorporates:
     *  Constant: '<S9>/V_Parameter5'
     *  Switch: '<S9>/Switch1'
     */
    LDDT_CurveTypeLe_St = LDDT_CurveNone_C_St;
  }

  /* End of Switch: '<S9>/Switch' */

  /* Switch: '<S106>/Switch5' incorporates:
   *  Constant: '<S106>/V_Parameter2'
   *  Constant: '<S106>/V_Parameter4'
   *  RelationalOperator: '<S106>/Relational Operator3'
   *  RelationalOperator: '<S106>/Relational Operator5'
   *  Switch: '<S106>/Switch7'
   */
  if (LDDT_CurveTypeLe_St == LDDT_CurveInner_C_St) {
    /* Abs: '<S106>/Abs' incorporates:
     *  Switch: '<S106>/Switch5'
     */
    LDWC_CrvSensiDecayLe_Mi = rtb_uDLookupTable8_idx_0;
  } else {
    if (LDDT_CurveTypeLe_St != LDDT_CurveOuter_C_St) {
      /* Abs: '<S106>/Abs' incorporates:
       *  Constant: '<S106>/V_Const5'
       *  Switch: '<S106>/Switch5'
       *  Switch: '<S106>/Switch7'
       */
      LDWC_CrvSensiDecayLe_Mi = 0.0F;
    }
  }

  /* End of Switch: '<S106>/Switch5' */

  /* Sum: '<S106>/Add2' incorporates:
   *  Lookup_n-D: '<S106>/1-D Lookup Table10'
   *  Product: '<S106>/Product2'
   *  Sum: '<S106>/Add'
   *  Sum: '<S106>/Add4'
   *  Switch: '<S8>/Switch1'
   */
  LDWC_DstcToLnTrsdLf_Mi = (LDWC_DstcToLnTrsdCrvCpstnLf_Mi + (look1_iflf_binlxpw
    (LDDT_LatVehSpdLf_Mps, (&(LDWC_LnLatVeh_BX_Mps[0])),
     (&(LDWC_LnLatVeh_Lf_Mps[0])), 8U) * LDWC_CrrctByLnWidth_Fct)) -
    (LDWC_CrvSensiDecayLe_Mi + rtb_uDLookupTable9_idx_1);

  /* RelationalOperator: '<S107>/Relational Operator3' */
  LDWC_RawTrigByDlcLf_B = (LDDT_DstcToLnLf_Mi <= LDWC_DstcToLnTrsdLf_Mi);

  /* RelationalOperator: '<S107>/Relational Operator1' incorporates:
   *  Constant: '<S107>/V_Parameter10'
   */
  LDWC_EnaTlcTrigLf_B = (LDDT_DstcToLnLf_Mi < LDWC_DstcOfTiToLnMn_C_Mi);

  /* Switch: '<S8>/Switch5' incorporates:
   *  Constant: '<S8>/Constant4'
   *  Constant: '<S8>/V_Parameter1'
   *  Inport: '<Root>/Inport1'
   *  Logic: '<S8>/Logical Operator'
   *  RelationalOperator: '<S8>/Relational Operator1'
   *  RelationalOperator: '<S8>/Relational Operator5'
   *  UnaryMinus: '<S8>/Unary Minus'
   */
  if (((LDDT_LnHeadLf_Rad < (-LDDT_TLCHeadAglTrsd_C_Rad)) &&
       (LDWSAI_VehSpdActu_Mps > 0.0F)) && LDDT_LnMakVldLf_B) {
    /* Switch: '<S8>/Switch5' incorporates:
     *  Constant: '<S8>/Constant3'
     *  Constant: '<S8>/Constant5'
     *  MinMax: '<S8>/MinMax1'
     *  MinMax: '<S8>/MinMax2'
     *  Product: '<S8>/Divide1'
     */
    LDDT_TiToLnLf_Sec = fminf(LDDT_DstcToLnLf_Mi / fmaxf
      (LDVSE_UHysLatVehSpdVldLf_B_tmp, 0.01F), 10.0F);
  } else {
    /* Switch: '<S8>/Switch5' incorporates:
     *  Constant: '<S8>/Constant6'
     */
    LDDT_TiToLnLf_Sec = 10.0F;
  }

  /* RelationalOperator: '<S107>/Relational Operator2' */
  LDWC_RawTrigByTlcLf_B = (LDDT_TiToLnLf_Sec < LDWC_TiToLnTrsd_Sec);

  /* Switch: '<S122>/Switch' incorporates:
   *  Constant: '<S107>/V_Parameter12'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S122>/Max'
   *  Sum: '<S122>/Subtract'
   *  Switch: '<S122>/Switch1'
   *  UnaryMinus: '<S122>/Unary Minus'
   *  UnitDelay: '<S122>/Unit Delay'
   */
  if (LDWC_RawTrigByTlcLf_B) {
    LDWC_DlyTiOfTiToLnLfMn_Sec = fmaxf(LDWC_DlyTiOfTiToLnLfMn_Sec,
      -LDWSAI_CycleTime_Sec) - LDWSAI_CycleTime_Sec;
  } else {
    LDWC_DlyTiOfTiToLnLfMn_Sec = LDWC_DlyTiOfTiToLnMn_C_Sec;
  }

  /* End of Switch: '<S122>/Switch' */

  /* Logic: '<S122>/AND' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S122>/LessThanOrEqual'
   *  UnaryMinus: '<S122>/Unary Minus1'
   *  UnitDelay: '<S122>/Unit Delay'
   */
  LDWC_DlyTrigByTlcLf_B = (LDWC_RawTrigByTlcLf_B && (LDWC_DlyTiOfTiToLnLfMn_Sec <=
    (-LDWSAI_CycleTime_Sec)));

  /* Logic: '<S107>/Logical Operator2' incorporates:
   *  Constant: '<S107>/V_Parameter11'
   *  Constant: '<S107>/V_Parameter9'
   *  DataTypeConversion: '<S112>/Data Type Conversion'
   *  DataTypeConversion: '<S113>/Data Type Conversion'
   *  Logic: '<S107>/Logical Operator'
   *  Logic: '<S107>/Logical Operator1'
   *  S-Function (sfix_bitop): '<S112>/Bitwise AND'
   *  S-Function (sfix_bitop): '<S113>/Bitwise AND'
   */
  rtb_LogicalOperator2_i4cj = ((((((uint32_T)LDWC_TrigCdtnEn_C_St) & 1U) != 0U) &&
    LDWC_RawTrigByDlcLf_B) || ((((((uint32_T)LDWC_TrigCdtnEn_C_St) & 2U) != 0U) &&
    LDWC_EnaTlcTrigLf_B) && LDWC_DlyTrigByTlcLf_B));

  /* Logic: '<S111>/AND' incorporates:
   *  Logic: '<S111>/NOT'
   *  UnitDelay: '<S111>/Unit Delay'
   */
  LDWC_EnaLdwTrigLf_B = (rtb_LogicalOperator2_i4cj && (!LDWC_HdTiTrigLfEn_B));

  /* Logic: '<S107>/OR' incorporates:
   *  Constant: '<S107>/V_Parameter14'
   *  Constant: '<S107>/V_Parameter5'
   *  RelationalOperator: '<S107>/Relational Operator11'
   *  RelationalOperator: '<S107>/Relational Operator4'
   *  Sum: '<S107>/Add2'
   */
  LDWC_RstTlcTrigLf_B = (((LDWC_DstcToLnTrsdLf_Mi + 0.12F) < LDDT_DstcToLnLf_Mi)
    || (LDDT_DstcToLnLf_Mi < LDWC_DstcOfDiscToLnLmtMn_C_Mi));

  /* Switch: '<S120>/Switch4' incorporates:
   *  Constant: '<S120>/Constant3'
   *  Switch: '<S120>/Switch3'
   */
  if (LDWC_RstTlcTrigLf_B) {
    rtb_Subtract2_hwoc = 0.0F;
  } else if (LDWC_EnaLdwTrigLf_B) {
    /* Switch: '<S120>/Switch3' incorporates:
     *  Constant: '<S107>/V_Parameter15'
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S120>/Subtract1'
     */
    rtb_Subtract2_hwoc = LDWC_HdTiTrigLf_C_Sec + LDWSAI_CycleTime_Sec;
  } else {
    /* Switch: '<S120>/Switch3' incorporates:
     *  UnitDelay: '<S120>/Unit Delay'
     */
    rtb_Subtract2_hwoc = LDWC_HdTiTrigLf_Sec;
  }

  /* End of Switch: '<S120>/Switch4' */

  /* Logic: '<S107>/Logical Operator5' incorporates:
   *  Logic: '<S66>/Logical Operator19'
   */
  LDWC_ErrSideByTrigLf_B = !LDDT_LnMakVldLf_B;

  /* Logic: '<S107>/Logical Operator4' incorporates:
   *  Constant: '<S116>/Constant'
   *  Inport: '<Root>/Inport12'
   *  Logic: '<S107>/Logical Operator5'
   *  RelationalOperator: '<S107>/Relational Operator5'
   */
  LDWC_ResetForSafeLf_B = ((LDWSAI_DtctLnChag_B || LDWC_ErrSideByTrigLf_B) ||
    (((uint32_T)LDWC_SysOld_St) == E_LDWState_nu_ACTIVE));

  /* RelationalOperator: '<S107>/Relational Operator6' incorporates:
   *  Constant: '<S107>/V_Parameter13'
   *  Sum: '<S107>/Add'
   */
  LDWC_SetForSafeLf_B = (LDDT_DstcToLnLf_Mi > (LDWC_DstcToLnTrsdLf_Mi +
    LDWC_DstcOfstSafeSitu_C_Mi));

  /* Switch: '<S118>/Switch' incorporates:
   *  Constant: '<S118>/Constant2'
   *  Switch: '<S118>/Switch1'
   *  UnitDelay: '<S118>/Unit Delay'
   */
  if (LDWC_ResetForSafeLf_B) {
    LDWC_DisTrigLf_B = false;
  } else {
    LDWC_DisTrigLf_B = (LDWC_SetForSafeLf_B || LDWC_DisTrigLf_B);
  }

  /* End of Switch: '<S118>/Switch' */

  /* Logic: '<S107>/Logical Operator6' incorporates:
   *  Constant: '<S120>/Constant4'
   *  Logic: '<S120>/OR'
   *  RelationalOperator: '<S120>/GreaterThan2'
   *  UnitDelay: '<S118>/Unit Delay'
   */
  rtb_LDWC_VehicleInvalid_B = ((LDWC_EnaLdwTrigLf_B || (rtb_Subtract2_hwoc >
    1.0E-5F)) && LDWC_DisTrigLf_B);

  /* Logic: '<S107>/Logical Operator8' incorporates:
   *  Constant: '<S107>/V_Parameter1'
   *  RelationalOperator: '<S107>/Relational Operator8'
   *  RelationalOperator: '<S107>/Relational Operator9'
   */
  rtb_LDWC_VelYInvalid_B = ((LDDT_DstcToLnLf_Mi < LDWC_DstcToLnTrsdLf_Mi) &&
    (LDDT_DstcToLnLf_Mi > LDWC_DstcOfDiscToLnLmtMn_C_Mi));

  /* RelationalOperator: '<S107>/Relational Operator10' incorporates:
   *  Constant: '<S117>/Constant'
   */
  rtb_RelationalOperator10_f5fy = (((uint32_T)LDWC_SysOld_St) ==
    E_LDWState_nu_ACTIVE);

  /* Logic: '<S107>/Logical Operator11' incorporates:
   *  Logic: '<S107>/AND'
   */
  rtb_RelationalOperator29 = !rtb_LDWC_VelYInvalid_B;

  /* Switch: '<S107>/Switch1' incorporates:
   *  Constant: '<S107>/Constant3'
   *  Logic: '<S107>/Logical Operator10'
   *  Logic: '<S107>/Logical Operator11'
   *  Logic: '<S114>/AND'
   *  Logic: '<S114>/NOT'
   *  Sum: '<S107>/Add1'
   *  UnitDelay: '<S107>/Unit Delay'
   *  UnitDelay: '<S107>/Unit Delay2'
   *  UnitDelay: '<S114>/Unit Delay'
   */
  if (LDWC_SuppFlagOldLf_B || rtb_RelationalOperator29) {
    LDWC_ContinWarmTimesOldLf_Count = 0.0F;
  } else {
    LDWC_ContinWarmTimesOldLf_Count += (real32_T)((rtb_RelationalOperator10_f5fy
      && (!LDWC_PreActiveEdgeLf)) ? 1 : 0);
  }

  /* End of Switch: '<S107>/Switch1' */

  /* RelationalOperator: '<S107>/Relational Operator12' incorporates:
   *  Constant: '<S107>/V_Parameter4'
   *  UnitDelay: '<S107>/Unit Delay'
   *  UnitDelay: '<S107>/Unit Delay2'
   */
  LDWC_SuppFlagOldLf_B = (LDWC_ContinWarmTimesOldLf_Count >=
    LDWC_ContinWarmTimes_C_Count);

  /* Switch: '<S121>/Switch4' incorporates:
   *  Constant: '<S121>/Constant3'
   *  Logic: '<S107>/Logical Operator7'
   *  Switch: '<S121>/Switch3'
   *  UnitDelay: '<S107>/Unit Delay'
   */
  if (!rtb_LDWC_VelYInvalid_B) {
    rtb_uDLookupTable9_idx_1 = 0.0F;
  } else if (LDWC_SuppFlagOldLf_B) {
    /* Switch: '<S121>/Switch3' incorporates:
     *  Constant: '<S107>/V_Parameter2'
     *  Constant: '<S107>/V_Parameter3'
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S107>/Add3'
     *  Sum: '<S121>/Subtract1'
     */
    rtb_uDLookupTable9_idx_1 = (LDWC_ContinWarmSupp_C_Sec + LDWC_WarmMxTi_C_Sec)
      + LDWSAI_CycleTime_Sec;
  } else {
    /* Switch: '<S121>/Switch3' incorporates:
     *  UnitDelay: '<S121>/Unit Delay'
     */
    rtb_uDLookupTable9_idx_1 = LDWC_SuppTimeOldLf_Sec;
  }

  /* End of Switch: '<S121>/Switch4' */

  /* Logic: '<S107>/Logical Operator14' incorporates:
   *  Constant: '<S121>/Constant4'
   *  Logic: '<S121>/OR'
   *  RelationalOperator: '<S121>/GreaterThan2'
   *  UnitDelay: '<S107>/Unit Delay'
   */
  rtb_LogicalOperator14_iqhs = ((!LDWC_SuppFlagOldLf_B) &&
    (rtb_uDLookupTable9_idx_1 <= 1.0E-5F));

  /* Logic: '<S107>/Logical Operator9' incorporates:
   *  Logic: '<S107>/AND'
   */
  LDWC_ResetForContinTrigLf_B = (rtb_RelationalOperator29 ||
    (!rtb_LogicalOperator14_iqhs));

  /* Logic: '<S107>/Logical Operator13' incorporates:
   *  Logic: '<S115>/AND'
   *  Logic: '<S115>/NOT'
   *  UnitDelay: '<S115>/Unit Delay'
   */
  LDWC_SetForContinTrigLf_B = (rtb_LDWC_VehicleInvalid_B ||
    (rtb_LogicalOperator14_iqhs && (!LDWC_ContinTrigLfEn_B)));

  /* Switch: '<S119>/Switch' incorporates:
   *  Constant: '<S119>/Constant2'
   *  Switch: '<S119>/Switch1'
   *  UnitDelay: '<S119>/Unit Delay'
   */
  if (LDWC_ResetForContinTrigLf_B) {
    LDWC_DisContinTrigLf_B = false;
  } else {
    LDWC_DisContinTrigLf_B = (LDWC_SetForContinTrigLf_B ||
      LDWC_DisContinTrigLf_B);
  }

  /* End of Switch: '<S119>/Switch' */

  /* SignalConversion: '<S22>/Signal Conversion3' incorporates:
   *  DataTypeConversion: '<S44>/Data Type Conversion1'
   */
  LDVSE_SidCdtnLDWLf_St = (uint8_T)rtb_ex_sfun_set_bit_dmuc;

  /* RelationalOperator: '<S107>/Relational Operator7' incorporates:
   *  Constant: '<S107>/V_Const'
   *  Constant: '<S107>/V_Const2'
   *  S-Function (sfix_bitop): '<S107>/Bitwise AND1'
   */
  LDWC_TrigBySideCondLf_B = ((((uint32_T)LDVSE_SidCdtnLDWLf_St) & 3U) == 0U);

  /* RelationalOperator: '<S107>/Equal' incorporates:
   *  Constant: '<S107>/V_Const1'
   */
  LDWC_TrigByPrjSpecLf_B = (LDWC_TrigByPrjSpecRi_B_tmp == 0);

  /* Logic: '<S107>/Logical Operator3' incorporates:
   *  Logic: '<S107>/Logical Operator12'
   *  UnitDelay: '<S119>/Unit Delay'
   */
  LDWC_TrigLf_B = (((rtb_LDWC_VehicleInvalid_B || LDWC_DisContinTrigLf_B) &&
                    LDWC_TrigBySideCondLf_B) && LDWC_TrigByPrjSpecLf_B);

  /* Logic: '<S104>/Logical Operator6' incorporates:
   *  Constant: '<S109>/Constant'
   *  Constant: '<S110>/Constant'
   *  RelationalOperator: '<S104>/Relational Operator1'
   *  RelationalOperator: '<S104>/Relational Operator2'
   */
  LDWC_EnaDgrSide_B = ((((uint32_T)LDWC_SysOld_St) == E_LDWState_nu_ACTIVE) ||
                       (((uint32_T)LDWC_SysOld_St) == E_LDWState_nu_RAMPOUT));

  /* Switch: '<S104>/Switch2' incorporates:
   *  Constant: '<S104>/V_Parameter4'
   *  Switch: '<S104>/Switch1'
   *  Switch: '<S104>/Switch3'
   *  Switch: '<S104>/Switch4'
   *  UnitDelay: '<S104>/UnitDelay1'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  if (LDWC_EnaDgrSide_B) {
    LDWC_PrevDgrSide_St = LDWC_DgrSideOld_St;
  } else {
    if (LDWC_TrigRi_B) {
      /* Switch: '<S104>/Switch1' incorporates:
       *  Constant: '<S104>/V_Parameter7'
       *  UnitDelay: '<S104>/UnitDelay1'
       */
      LDWC_DgrSideOld_St = LDWC_DgrSideRi_C_St;
    } else if (LDWC_TrigLf_B) {
      /* Switch: '<S104>/Switch3' incorporates:
       *  Constant: '<S104>/V_Parameter6'
       *  Switch: '<S104>/Switch1'
       *  UnitDelay: '<S104>/UnitDelay1'
       */
      LDWC_DgrSideOld_St = LDWC_DgrSideLf_C_St;
    } else {
      /* UnitDelay: '<S104>/UnitDelay1' incorporates:
       *  Constant: '<S104>/V_Parameter5'
       *  Switch: '<S104>/Switch1'
       *  Switch: '<S104>/Switch3'
       */
      LDWC_DgrSideOld_St = LDWC_NoDgrSide_C_St;
    }

    LDWC_PrevDgrSide_St = LDWC_NoDgrSide_C_St;
  }

  /* End of Switch: '<S104>/Switch2' */

  /* RelationalOperator: '<S60>/Relational Operator7' incorporates:
   *  Constant: '<S60>/V_Parameter17'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDWC_FnsByDgrStLf_B = (LDWC_PrevDgrSide_St == LDWC_DgrSideLf_C_St);

  /* Sum: '<S60>/Add2' incorporates:
   *  Constant: '<S60>/V_Parameter20'
   *  Constant: '<S60>/V_Parameter21'
   *  Sum: '<S60>/Add4'
   */
  rtb_LDWC_RampoutTime_Sec = LDWC_TgtTrajPstnY_C_Mi - LDWC_NoDgrFnsOfst_C_Mi;

  /* Sum: '<S60>/Add1' incorporates:
   *  Constant: '<S60>/V_Parameter18'
   *  Constant: '<S60>/V_Parameter19'
   *  Sum: '<S60>/Add3'
   */
  rtb_uDLookupTable8_idx_0 = LDWC_TgtTrajPstnY_C_Mi + LDWC_DgrFnsOfst_C_Mi;

  /* Logic: '<S60>/Logical Operator8' incorporates:
   *  Constant: '<S60>/V_Parameter28'
   *  DataTypeConversion: '<S88>/Data Type Conversion'
   *  Logic: '<S60>/NOT'
   *  Logic: '<S97>/AND'
   *  RelationalOperator: '<S97>/Less Than'
   *  RelationalOperator: '<S97>/Less Than1'
   *  S-Function (sfix_bitop): '<S88>/Bitwise AND'
   *  Sum: '<S60>/Add1'
   *  Sum: '<S60>/Add2'
   */
  LDWC_FnsByLatDistLf_B = (((rtb_uDLookupTable8_idx_0 > LDDT_DstcToLnLf_Mi) &&
    (LDDT_DstcToLnLf_Mi > rtb_LDWC_RampoutTime_Sec)) || ((((uint32_T)
    LDWC_FnsCdsnEn_C_St) & 1U) == 0U));

  /* Logic: '<S60>/Logical Operator9' incorporates:
   *  Constant: '<S60>/V_Parameter22'
   *  Constant: '<S60>/V_Parameter23'
   *  Constant: '<S60>/V_Parameter29'
   *  DataTypeConversion: '<S89>/Data Type Conversion'
   *  Inport: '<Root>/Inport20'
   *  Logic: '<S60>/NOT1'
   *  Logic: '<S98>/AND'
   *  RelationalOperator: '<S98>/Less Than'
   *  RelationalOperator: '<S98>/Less Than1'
   *  S-Function (sfix_bitop): '<S89>/Bitwise AND'
   *  UnaryMinus: '<S60>/Unary Minus'
   */
  LDWC_FnsByHeadingLf_B = (((LDWC_NoDgrFnsHeadAng_C_Rad > LDWSAI_HeadAglLf_Rad) &&
    (LDWSAI_HeadAglLf_Rad > (-LDWC_DgrFnsHeadAng_C_Rad))) || ((((uint32_T)
    LDWC_FnsCdsnEn_C_St) & 2U) == 0U));

  /* Logic: '<S60>/Logical Operator10' incorporates:
   *  Constant: '<S60>/V_Parameter24'
   *  Constant: '<S60>/V_Parameter25'
   *  Constant: '<S60>/V_Parameter30'
   *  DataTypeConversion: '<S90>/Data Type Conversion'
   *  Logic: '<S60>/NOT2'
   *  Logic: '<S99>/AND'
   *  RelationalOperator: '<S99>/Less Than'
   *  RelationalOperator: '<S99>/Less Than1'
   *  S-Function (sfix_bitop): '<S90>/Bitwise AND'
   *  UnaryMinus: '<S60>/Unary Minus1'
   */
  LDWC_FnsByLatSpdLf_B = (((LDWC_NoDgrFnsSpdVelLat_C_Mps > LDDT_LatVehSpdLf_Mps)
    && (LDDT_LatVehSpdLf_Mps > (-LDWC_DgrFnsSpdVelLat_C_Mps))) || ((((uint32_T)
    LDWC_FnsCdsnEn_C_St) & 4U) == 0U));

  /* Logic: '<S60>/Logical Operator6' */
  LDWC_DgrFnsLf_B = (((LDWC_FnsByDgrStLf_B && LDWC_FnsByLatDistLf_B) &&
                      LDWC_FnsByHeadingLf_B) && LDWC_FnsByLatSpdLf_B);

  /* RelationalOperator: '<S60>/Relational Operator9' incorporates:
   *  Constant: '<S60>/V_Parameter32'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDWC_FnsByDgrStRi_B = (LDWC_PrevDgrSide_St == LDWC_DgrSideRi_C_St);

  /* Logic: '<S60>/Logical Operator14' incorporates:
   *  Constant: '<S60>/V_Parameter41'
   *  DataTypeConversion: '<S92>/Data Type Conversion'
   *  Logic: '<S100>/AND'
   *  Logic: '<S60>/NOT3'
   *  RelationalOperator: '<S100>/Less Than'
   *  RelationalOperator: '<S100>/Less Than1'
   *  S-Function (sfix_bitop): '<S92>/Bitwise AND'
   *  UnaryMinus: '<S60>/Unary Minus4'
   *  UnaryMinus: '<S60>/Unary Minus5'
   */
  LDWC_FnsByLatDistRi_B = ((((-rtb_LDWC_RampoutTime_Sec) > LDDT_DstcToLnRi_Mi) &&
    (LDDT_DstcToLnRi_Mi > (-rtb_uDLookupTable8_idx_0))) || ((((uint32_T)
    LDWC_FnsCdsnEn_C_St) & 8U) == 0U));

  /* Logic: '<S60>/Logical Operator15' incorporates:
   *  Constant: '<S60>/V_Parameter37'
   *  Constant: '<S60>/V_Parameter38'
   *  Constant: '<S60>/V_Parameter42'
   *  DataTypeConversion: '<S93>/Data Type Conversion'
   *  Inport: '<Root>/Inport22'
   *  Logic: '<S101>/AND'
   *  Logic: '<S60>/NOT4'
   *  RelationalOperator: '<S101>/Less Than'
   *  RelationalOperator: '<S101>/Less Than1'
   *  S-Function (sfix_bitop): '<S93>/Bitwise AND'
   *  UnaryMinus: '<S60>/Unary Minus2'
   */
  LDWC_FnsByHeadingRi_B = (((LDWC_DgrFnsHeadAng_C_Rad > LDWSAI_HeadAglRi_Rad) &&
    (LDWSAI_HeadAglRi_Rad > (-LDWC_NoDgrFnsHeadAng_C_Rad))) || ((((uint32_T)
    LDWC_FnsCdsnEn_C_St) & 16U) == 0U));

  /* Logic: '<S60>/Logical Operator13' incorporates:
   *  Constant: '<S60>/V_Parameter39'
   *  Constant: '<S60>/V_Parameter40'
   *  Constant: '<S60>/V_Parameter43'
   *  DataTypeConversion: '<S94>/Data Type Conversion'
   *  Logic: '<S102>/AND'
   *  Logic: '<S60>/NOT5'
   *  RelationalOperator: '<S102>/Less Than'
   *  RelationalOperator: '<S102>/Less Than1'
   *  S-Function (sfix_bitop): '<S94>/Bitwise AND'
   *  UnaryMinus: '<S60>/Unary Minus3'
   */
  LDWC_FnsByLatSpdRi_B = (((LDWC_DgrFnsSpdVelLat_C_Mps > LDDT_LatVehSpdRi_Mps) &&
    (LDDT_LatVehSpdRi_Mps > (-LDWC_NoDgrFnsSpdVelLat_C_Mps))) || ((((uint32_T)
    LDWC_FnsCdsnEn_C_St) & 32U) == 0U));

  /* Logic: '<S60>/Logical Operator12' */
  LDWC_DgrFnsRi_B = (((LDWC_FnsByDgrStRi_B && LDWC_FnsByLatDistRi_B) &&
                      LDWC_FnsByHeadingRi_B) && LDWC_FnsByLatSpdRi_B);

  /* RelationalOperator: '<S60>/Relational Operator8' incorporates:
   *  Constant: '<S95>/Constant'
   */
  LDWC_MinLdwBySysSt_B = (((uint32_T)LDWC_SysOld_St) == E_LDWState_nu_ACTIVE);

  /* Logic: '<S87>/AND' incorporates:
   *  Logic: '<S87>/NOT'
   *  UnitDelay: '<S87>/Unit Delay'
   */
  LDWC_EdgeRiseForMinLdw_B = (LDWC_MinLdwBySysSt_B && (!LDWC_EdgeRisWarming_B));

  /* Switch: '<S103>/Switch2' incorporates:
   *  Constant: '<S60>/V_Parameter27'
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S103>/GreaterThan'
   *  Switch: '<S103>/Switch'
   *  UnitDelay: '<S103>/Unit Delay'
   */
  if (LDWC_EdgeRiseForMinLdw_B) {
    LDWC_HdTiWarming_Sec = LDWC_FnsDuraMn_C_Sec;
  } else if (LDWC_HdTiWarming_Sec > LDWSAI_CycleTime_Sec) {
    /* Switch: '<S103>/Switch' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S103>/Subtract'
     *  UnitDelay: '<S103>/Unit Delay'
     */
    LDWC_HdTiWarming_Sec -= LDWSAI_CycleTime_Sec;
  } else {
    /* UnitDelay: '<S103>/Unit Delay' incorporates:
     *  Constant: '<S103>/Constant1'
     *  Switch: '<S103>/Switch'
     */
    LDWC_HdTiWarming_Sec = 0.0F;
  }

  /* End of Switch: '<S103>/Switch2' */

  /* RelationalOperator: '<S103>/GreaterThan1' incorporates:
   *  Constant: '<S103>/Constant2'
   *  UnitDelay: '<S103>/Unit Delay'
   */
  LDWC_HoldForMinLdw_B = (LDWC_HdTiWarming_Sec > 0.0F);

  /* Logic: '<S60>/Logical Operator11' incorporates:
   *  Constant: '<S60>/V_Parameter31'
   *  DataTypeConversion: '<S91>/Data Type Conversion'
   *  Logic: '<S60>/Logical Operator2'
   *  Logic: '<S60>/Logical Operator7'
   *  Logic: '<S60>/NOT6'
   *  S-Function (sfix_bitop): '<S91>/Bitwise AND'
   */
  LDWC_FlagMinTimeLDW_B = (((!LDWC_MinLdwBySysSt_B) || (!LDWC_HoldForMinLdw_B)) ||
    ((((uint32_T)LDWC_FnsCdsnEn_C_St) & 128U) == 0U));

  /* Logic: '<S60>/Logical Operator3' incorporates:
   *  Logic: '<S60>/Logical Operator1'
   */
  rtb_LDWC_VehicleInvalid_B = ((LDWC_DgrFnsLf_B || LDWC_DgrFnsRi_B) &&
    LDWC_FlagMinTimeLDW_B);

  /* Switch: '<S96>/Switch' incorporates:
   *  Constant: '<S60>/V_Parameter9'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S96>/Max'
   *  Sum: '<S96>/Subtract'
   *  Switch: '<S96>/Switch1'
   *  UnaryMinus: '<S96>/Unary Minus'
   *  UnitDelay: '<S96>/Unit Delay'
   */
  if (rtb_LDWC_VehicleInvalid_B) {
    LDWC_DlyTiTgtFns_Sec = fmaxf(LDWC_DlyTiTgtFns_Sec, -LDWSAI_CycleTime_Sec) -
      LDWSAI_CycleTime_Sec;
  } else {
    LDWC_DlyTiTgtFns_Sec = LDWC_DlyTiTgtFns_C_Sec;
  }

  /* End of Switch: '<S96>/Switch' */

  /* Logic: '<S96>/AND' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S96>/LessThanOrEqual'
   *  UnaryMinus: '<S96>/Unary Minus1'
   *  UnitDelay: '<S96>/Unit Delay'
   */
  LDWC_DgrFns_B = (rtb_LDWC_VehicleInvalid_B && (LDWC_DlyTiTgtFns_Sec <=
    (-LDWSAI_CycleTime_Sec)));

  /* SignalConversion: '<S20>/Signal Conversion1' incorporates:
   *  DataTypeConversion: '<S32>/Data Type Conversion1'
   */
  LDVSE_IvldLDW_St = (uint8_T)rtb_ex_sfun_set_bit;

  /* RelationalOperator: '<S63>/Relational Operator38' incorporates:
   *  Constant: '<S63>/Constant33'
   *  Constant: '<S63>/V_Parameter46'
   *  S-Function (sfix_bitop): '<S63>/Bitwise Operator32'
   */
  LDWC_CancelBySpecific_B = ((((int32_T)LDVSE_IvldLDW_St) & ((int32_T)
    LDWC_CclErrSpcLDW_C_St)) != 0);

  /* RelationalOperator: '<S63>/Relational Operator37' incorporates:
   *  Constant: '<S63>/Constant34'
   *  Constant: '<S63>/V_Parameter47'
   *  Inport: '<Root>/Inport38'
   *  S-Function (sfix_bitop): '<S63>/Bitwise Operator33'
   */
  LDWC_CancelByVehSt_B = ((((int32_T)LDWSAI_VehStIvld_St) & ((int32_T)
    LDWC_CclVehIvld_C_St)) != 0);

  /* RelationalOperator: '<S63>/Relational Operator32' incorporates:
   *  Constant: '<S63>/Constant35'
   *  Constant: '<S63>/V_Parameter41'
   *  Inport: '<Root>/Inport39'
   *  S-Function (sfix_bitop): '<S63>/Bitwise Operator34'
   */
  LDWC_CancelByDrvSt_B = ((((int32_T)LDWSAI_IvldStDrv_St) & ((int32_T)
    LDWC_CclDrvIVld_C_St)) != 0);

  /* RelationalOperator: '<S63>/Relational Operator33' incorporates:
   *  Constant: '<S63>/Constant29'
   *  Constant: '<S63>/V_Parameter42'
   *  Inport: '<Root>/Inport40'
   *  S-Function (sfix_bitop): '<S63>/Bitwise Operator28'
   */
  LDWC_CancelByCtrlSt_B = ((((int32_T)LDWSAI_CtrlStEn_St) & ((int32_T)
    LDWC_CclDrvActCtrl_C_St)) != 0);

  /* RelationalOperator: '<S63>/Relational Operator34' incorporates:
   *  Constant: '<S63>/Constant30'
   *  Constant: '<S63>/V_Parameter43'
   *  Inport: '<Root>/Inport41'
   *  S-Function (sfix_bitop): '<S63>/Bitwise Operator29'
   */
  LDWC_CancelBySysSt_B = ((((int32_T)LDWSAI_StError_St) & ((int32_T)
    LDWC_CclVehSysErr_C_St)) != 0);

  /* RelationalOperator: '<S63>/Relational Operator35' incorporates:
   *  Constant: '<S63>/Constant31'
   *  Constant: '<S63>/V_Parameter44'
   *  Inport: '<Root>/Inport42'
   *  S-Function (sfix_bitop): '<S63>/Bitwise Operator30'
   */
  LDWC_CancelByAvlSt_B = ((((int32_T)LDWSAI_CtrlStNoAvlb_St) & ((int32_T)
    LDWC_CclNoAvlbVehSys_C_St)) != 0);

  /* RelationalOperator: '<S63>/Relational Operator36' incorporates:
   *  Constant: '<S63>/Constant32'
   *  Constant: '<S63>/V_Parameter45'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S63>/Bitwise Operator31'
   */
  LDWC_CancelByPrjSpec_B = ((((int32_T)LDWSAI_PrjSpecQu_St) & ((int32_T)
    LDWC_CclFctCstm_St)) != 0);

  /* RelationalOperator: '<S67>/Relational Operator47' incorporates:
   *  Constant: '<S69>/Constant'
   */
  LDWC_MaxDurationBySysSt_B = (((uint32_T)LDWC_SysOld_St) ==
    E_LDWState_nu_ACTIVE);

  /* Logic: '<S68>/AND' incorporates:
   *  Logic: '<S68>/NOT'
   *  UnitDelay: '<S68>/Unit Delay'
   */
  LDWC_EdgRiseForSysSt_B = (LDWC_MaxDurationBySysSt_B && (!LDWC_EdgeRisWarmMx_B));

  /* Switch: '<S70>/Switch2' incorporates:
   *  Constant: '<S67>/V_Parameter63'
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S70>/GreaterThan'
   *  Switch: '<S70>/Switch'
   *  UnitDelay: '<S70>/Unit Delay'
   */
  if (LDWC_EdgRiseForSysSt_B) {
    LDWC_HdTiWarmMx_Sec = LDWC_WarmMxTi_C_Sec;
  } else if (LDWC_HdTiWarmMx_Sec > LDWSAI_CycleTime_Sec) {
    /* Switch: '<S70>/Switch' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S70>/Subtract'
     *  UnitDelay: '<S70>/Unit Delay'
     */
    LDWC_HdTiWarmMx_Sec -= LDWSAI_CycleTime_Sec;
  } else {
    /* UnitDelay: '<S70>/Unit Delay' incorporates:
     *  Constant: '<S70>/Constant1'
     *  Switch: '<S70>/Switch'
     */
    LDWC_HdTiWarmMx_Sec = 0.0F;
  }

  /* End of Switch: '<S70>/Switch2' */

  /* Logic: '<S67>/Logical Operator22' incorporates:
   *  Constant: '<S70>/Constant2'
   *  RelationalOperator: '<S70>/GreaterThan1'
   *  UnitDelay: '<S70>/Unit Delay'
   */
  LDWC_MaxDurationByStDly_B = (LDWC_HdTiWarmMx_Sec <= 0.0F);

  /* Logic: '<S67>/Logical Operator23' */
  LDWC_TiWarmMx_B = (LDWC_MaxDurationBySysSt_B && LDWC_MaxDurationByStDly_B);

  /* RelationalOperator: '<S66>/Relational Operator42' incorporates:
   *  Constant: '<S66>/Constant36'
   *  Constant: '<S66>/V_Parameter57'
   *  S-Function (sfix_bitop): '<S66>/Bitwise Operator35'
   */
  LDWC_ErrSideBySideCondLf_B = ((((int32_T)LDVSE_SidCdtnLDWLf_St) & ((int32_T)
    LDWC_SidCdtnCclLf_C_St)) != 0);

  /* RelationalOperator: '<S66>/Relational Operator43' incorporates:
   *  Constant: '<S66>/Constant37'
   *  Constant: '<S66>/V_Parameter58'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S66>/Bitwise Operator36'
   */
  LDWC_ErrSidByPrjSpecLf_B = ((((int32_T)LDWSAI_PrjSpecQu_St) & ((int32_T)
    LDWC_ErrCstmCclLf_C_St)) != 0);

  /* Logic: '<S66>/Logical Operator18' */
  LDWC_ErrSidCdtnLf_B = ((LDWC_ErrSideByTrigLf_B || LDWC_ErrSideBySideCondLf_B) ||
    LDWC_ErrSidByPrjSpecLf_B);

  /* RelationalOperator: '<S66>/Relational Operator41' incorporates:
   *  Constant: '<S66>/V_Parameter56'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDWC_SideCondByDgrLf_B = (LDWC_PrevDgrSide_St == LDWC_DgrSideLf_C_St);

  /* Logic: '<S66>/Logical Operator15' */
  LDWC_CanelBySideLf_B = (LDWC_ErrSidCdtnLf_B && LDWC_SideCondByDgrLf_B);

  /* RelationalOperator: '<S66>/Relational Operator44' incorporates:
   *  Constant: '<S66>/V_Parameter59'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDWC_SideCondByDgrRi_B = (LDWC_PrevDgrSide_St == LDWC_DgrSideRi_C_St);

  /* RelationalOperator: '<S66>/Relational Operator45' incorporates:
   *  Constant: '<S66>/Constant38'
   *  Constant: '<S66>/V_Parameter60'
   *  S-Function (sfix_bitop): '<S66>/Bitwise Operator37'
   */
  LDWC_ErrSideBySideCondRi_B = ((((int32_T)LDVSE_SidCdtnLDWRi_St) & ((int32_T)
    LDWC_SidCdtnCclRi_C_St)) != 0);

  /* RelationalOperator: '<S66>/Relational Operator46' incorporates:
   *  Constant: '<S66>/Constant39'
   *  Constant: '<S66>/V_Parameter61'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S66>/Bitwise Operator38'
   */
  LDWC_ErrSidByPrjSpecRi_B = ((((int32_T)LDWSAI_PrjSpecQu_St) & ((int32_T)
    LDWC_ErrCstmCclRi_C_St)) != 0);

  /* Logic: '<S66>/Logical Operator2' */
  LDWC_ErrSidCdtnRi_B = ((LDWC_ErrSideByTrigRi_B || LDWC_ErrSideBySideCondRi_B) ||
    LDWC_ErrSidByPrjSpecRi_B);

  /* Logic: '<S66>/Logical Operator1' */
  LDWC_CanelBySideRi_B = (LDWC_SideCondByDgrRi_B && LDWC_ErrSidCdtnRi_B);

  /* Logic: '<S66>/Logical Operator16' */
  LDWC_ErrSidCdtn_B = (LDWC_CanelBySideLf_B || LDWC_CanelBySideRi_B);

  /* Sum: '<S65>/Add3' incorporates:
   *  Constant: '<S65>/V_Parameter50'
   *  Constant: '<S65>/V_Parameter51'
   *  Sum: '<S65>/Add1'
   */
  rtb_LDWC_RampoutTime_Sec = LDWC_TgtTrajPstnY_C_Mi + LDWC_NoDgrCclOfst_C_Mi;

  /* Logic: '<S65>/Logical Operator24' incorporates:
   *  Constant: '<S65>/V_Parameter53'
   *  RelationalOperator: '<S65>/Relational Operator48'
   *  RelationalOperator: '<S65>/Relational Operator49'
   *  Sum: '<S65>/Add3'
   *  UnaryMinus: '<S65>/Unary Minus5'
   */
  LDWC_CLatDevByDlcLf_B = (((-LDWC_DgrCclOfst_C_Mi) > LDDT_DstcToLnLf_Mi) ||
    (LDDT_DstcToLnLf_Mi > rtb_LDWC_RampoutTime_Sec));

  /* RelationalOperator: '<S65>/Relational Operator39' incorporates:
   *  Constant: '<S65>/V_Parameter48'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDWC_CLatDevByDgrLf_B = (LDWC_PrevDgrSide_St == LDWC_DgrSideLf_C_St);

  /* Logic: '<S65>/Logical Operator12' */
  LDWC_CclLatDevLf_B = (LDWC_CLatDevByDlcLf_B && LDWC_CLatDevByDgrLf_B);

  /* RelationalOperator: '<S65>/Relational Operator40' incorporates:
   *  Constant: '<S65>/V_Parameter49'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDWC_CLatDevByDlcRi_B = (LDWC_PrevDgrSide_St == LDWC_DgrSideRi_C_St);

  /* Logic: '<S65>/Logical Operator25' incorporates:
   *  Constant: '<S65>/V_Parameter1'
   *  RelationalOperator: '<S65>/Relational Operator50'
   *  RelationalOperator: '<S65>/Relational Operator51'
   *  UnaryMinus: '<S65>/Unary Minus1'
   */
  LDWC_CLatDevByDgrRi_B = (((-rtb_LDWC_RampoutTime_Sec) > LDDT_DstcToLnRi_Mi) ||
    (LDDT_DstcToLnRi_Mi > LDWC_DgrCclOfst_C_Mi));

  /* Logic: '<S65>/Logical Operator14' */
  LDWC_CclLatDevRi_B = (LDWC_CLatDevByDlcRi_B && LDWC_CLatDevByDgrRi_B);

  /* Logic: '<S65>/Logical Operator13' */
  LDWC_CclLatDev_B = (LDWC_CclLatDevLf_B || LDWC_CclLatDevRi_B);

  /* Logic: '<S63>/Logical Operator11' */
  LDWC_Cancel_B = (((((((((LDWC_CancelBySpecific_B || LDWC_CancelByVehSt_B) ||
    LDWC_CancelByDrvSt_B) || LDWC_CancelByCtrlSt_B) || LDWC_CancelBySysSt_B) ||
                       LDWC_CancelByAvlSt_B) || LDWC_CancelByPrjSpec_B) ||
                     LDWC_TiWarmMx_B) || LDWC_ErrSidCdtn_B) || LDWC_CclLatDev_B);

  /* RelationalOperator: '<S64>/Relational Operator2' incorporates:
   *  Constant: '<S64>/Constant'
   *  Constant: '<S64>/V_Parameter17'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator'
   */
  LDWC_AbortBySpecific_B = ((((int32_T)LDVSE_IvldLDW_St) & ((int32_T)
    LDWC_AbtErrSpcLDW_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator1' incorporates:
   *  Constant: '<S64>/Constant1'
   *  Constant: '<S64>/V_Parameter9'
   *  Inport: '<Root>/Inport38'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator1'
   */
  LDWC_AbortByVehSt_B = ((((int32_T)LDWSAI_VehStIvld_St) & ((int32_T)
    LDWC_AbtVehIvld_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator3' incorporates:
   *  Constant: '<S64>/Constant2'
   *  Constant: '<S64>/V_Parameter10'
   *  Inport: '<Root>/Inport39'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator2'
   */
  LDWC_AbortByDrvSt_B = ((((int32_T)LDWSAI_IvldStDrv_St) & ((int32_T)
    LDWC_AbtDrvIVld_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator4' incorporates:
   *  Constant: '<S64>/Constant3'
   *  Constant: '<S64>/V_Parameter11'
   *  Inport: '<Root>/Inport40'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator3'
   */
  LDWC_AbortByCtrlSt_B = ((((int32_T)LDWSAI_CtrlStEn_St) & ((int32_T)
    LDWC_AbtDrvActCtrl_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator5' incorporates:
   *  Constant: '<S64>/Constant4'
   *  Constant: '<S64>/V_Parameter12'
   *  Inport: '<Root>/Inport41'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator4'
   */
  LDWC_AbortBySysSt_B = ((((int32_T)LDWSAI_StError_St) & ((int32_T)
    LDWC_AbtVehSysErr_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator6' incorporates:
   *  Constant: '<S64>/Constant5'
   *  Constant: '<S64>/V_Parameter13'
   *  Inport: '<Root>/Inport42'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator5'
   */
  LDWC_AbortByAvlSt_B = ((((int32_T)LDWSAI_CtrlStNoAvlb_St) & ((int32_T)
    LDWC_AbtNoAvlbVehSys_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator7' incorporates:
   *  Constant: '<S64>/Constant6'
   *  Constant: '<S64>/V_Parameter14'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator6'
   */
  LDWC_AbortByPrjSpec_B = ((((int32_T)LDWSAI_PrjSpecQu_St) & ((int32_T)
    LDWC_AbtFctCstm_C_St)) != 0);

  /* Logic: '<S64>/Logical Operator6' */
  LDWC_Abort_B = ((((((LDWC_AbortBySpecific_B || LDWC_AbortByVehSt_B) ||
                      LDWC_AbortByDrvSt_B) || LDWC_AbortByCtrlSt_B) ||
                    LDWC_AbortBySysSt_B) || LDWC_AbortByAvlSt_B) ||
                  LDWC_AbortByPrjSpec_B);

  /* RelationalOperator: '<S64>/Relational Operator9' incorporates:
   *  Constant: '<S64>/Constant7'
   *  Constant: '<S64>/V_Parameter21'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator7'
   */
  LDWC_StrgRdyBySpecific_B = ((((int32_T)LDVSE_IvldLDW_St) & ((int32_T)
    LDWC_StrgRdyErrSpcLDW_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator8' incorporates:
   *  Constant: '<S64>/Constant8'
   *  Constant: '<S64>/V_Parameter22'
   *  Inport: '<Root>/Inport38'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator8'
   */
  LDWC_StrgRdyByVehSt_B = ((((int32_T)LDWSAI_VehStIvld_St) & ((int32_T)
    LDWC_StrgRdyVehIvld_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator10' incorporates:
   *  Constant: '<S64>/Constant9'
   *  Constant: '<S64>/V_Parameter15'
   *  Inport: '<Root>/Inport39'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator9'
   */
  LDWC_StrgRdyByDrvSt_B = ((((int32_T)LDWSAI_IvldStDrv_St) & ((int32_T)
    LDWC_StrgRdyDrvIVld_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator11' incorporates:
   *  Constant: '<S64>/Constant10'
   *  Constant: '<S64>/V_Parameter16'
   *  Inport: '<Root>/Inport40'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator10'
   */
  LDWC_StrgRdyByCtrlSt_B = ((((int32_T)LDWSAI_CtrlStEn_St) & ((int32_T)
    LDWC_StrgRdyDrvActCtrl_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator12' incorporates:
   *  Constant: '<S64>/Constant11'
   *  Constant: '<S64>/V_Parameter18'
   *  Inport: '<Root>/Inport41'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator11'
   */
  LDWC_StrgRdyBySysSt_B = ((((int32_T)LDWSAI_StError_St) & ((int32_T)
    LDWC_StrgRdyVehSysErr_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator13' incorporates:
   *  Constant: '<S64>/Constant12'
   *  Constant: '<S64>/V_Parameter19'
   *  Inport: '<Root>/Inport42'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator12'
   */
  LDWC_StrgRdyByAvlSt_B = ((((int32_T)LDWSAI_CtrlStNoAvlb_St) & ((int32_T)
    LDWC_StrgRdyNoAvlbVehSys_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator14' incorporates:
   *  Constant: '<S64>/Constant13'
   *  Constant: '<S64>/V_Parameter20'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator13'
   */
  LDWC_StrgRdyByPrjSpec_B = ((((int32_T)LDWSAI_PrjSpecQu_St) & ((int32_T)
    LDWC_StrgRdyFctCstm_C_St)) == 0);

  /* Logic: '<S64>/Logical Operator1' incorporates:
   *  Logic: '<S64>/Logical Operator2'
   */
  LDWC_StrgRdy_B = ((((((((!LDWC_Abort_B) && LDWC_StrgRdyBySpecific_B) &&
    LDWC_StrgRdyByVehSt_B) && LDWC_StrgRdyByDrvSt_B) && LDWC_StrgRdyByCtrlSt_B) &&
                      LDWC_StrgRdyBySysSt_B) && LDWC_StrgRdyByAvlSt_B) &&
                    LDWC_StrgRdyByPrjSpec_B);

  /* Switch: '<S59>/Switch3' incorporates:
   *  Constant: '<S59>/V_Parameter63'
   *  Switch: '<S59>/Switch2'
   */
  if (LDWC_Abort_B) {
    rtb_LDWC_RampoutTime_Sec = LDWC_TiAbtDegr_C_Sec;
  } else if (LDWC_StrgRdy_B) {
    /* Switch: '<S59>/Switch1' incorporates:
     *  Constant: '<S59>/Constant40'
     *  Constant: '<S59>/V_Parameter2'
     *  Switch: '<S59>/Switch'
     *  Switch: '<S59>/Switch2'
     */
    if (LDWC_DgrFns_B) {
      rtb_LDWC_RampoutTime_Sec = LDWC_TiDgrFnsDegr_C_Sec;
    } else if (LDWC_Cancel_B) {
      /* Switch: '<S59>/Switch' incorporates:
       *  Constant: '<S59>/V_Parameter3'
       */
      rtb_LDWC_RampoutTime_Sec = LDWC_TiCclDegr_C_Sec;
    } else {
      rtb_LDWC_RampoutTime_Sec = 0.0F;
    }

    /* End of Switch: '<S59>/Switch1' */
  } else {
    /* Switch: '<S59>/Switch2' incorporates:
     *  Constant: '<S59>/V_Parameter1'
     */
    rtb_LDWC_RampoutTime_Sec = LDWC_TiStrgRdyDegr_C_Sec;
  }

  /* End of Switch: '<S59>/Switch3' */

  /* Logic: '<S59>/Logical Operator22' incorporates:
   *  Chart: '<S5>/LDW_State'
   */
  rtb_Equal_azab = !LDWC_StrgRdy_B;

  /* Logic: '<S59>/Logical Operator1' incorporates:
   *  Logic: '<S59>/Logical Operator22'
   */
  LDWC_Degradation_B = (((LDWC_Abort_B || rtb_Equal_azab) || LDWC_DgrFns_B) ||
                        LDWC_Cancel_B);

  /* Logic: '<S85>/AND' incorporates:
   *  Logic: '<S85>/NOT'
   *  UnitDelay: '<S85>/Unit Delay'
   */
  LDWC_DegradationEdgeRise_B = (LDWC_Degradation_B && (!LDWC_EdgeRisDegr_B));

  /* Logic: '<S59>/Logical Operator2' incorporates:
   *  UnitDelay: '<S59>/UnitDelay'
   */
  LDWC_Degr_B = LDWC_DegrOld_B;

  /* Logic: '<S59>/Logical Operator' */
  rtb_LDWC_VehicleInvalid_B = (LDWC_Degr_B && LDWC_DegradationEdgeRise_B);

  /* Switch: '<S86>/Switch3' */
  if (rtb_LDWC_VehicleInvalid_B) {
    /* Sum: '<S86>/Subtract2' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S86>/Subtract1'
     */
    LDWC_HdTiDegr_Sec = rtb_LDWC_RampoutTime_Sec + LDWSAI_CycleTime_Sec;
  }

  /* End of Switch: '<S86>/Switch3' */

  /* Logic: '<S59>/Logical Operator2' incorporates:
   *  Constant: '<S86>/Constant4'
   *  Logic: '<S86>/OR'
   *  RelationalOperator: '<S86>/GreaterThan2'
   */
  LDWC_Degr_B = ((!rtb_LDWC_VehicleInvalid_B) && (LDWC_HdTiDegr_Sec <= 1.0E-5F));

  /* Sum: '<S86>/Subtract2' incorporates:
   *  Constant: '<S86>/Constant5'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S86>/Max'
   */
  LDWC_HdTiDegr_Sec -= LDWSAI_CycleTime_Sec;
  LDWC_HdTiDegr_Sec = fmaxf(LDWC_HdTiDegr_Sec, 0.0F);

  /* RelationalOperator: '<S64>/Relational Operator21' incorporates:
   *  Constant: '<S64>/Constant18'
   *  Constant: '<S64>/V_Parameter28'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator18'
   */
  LDWC_SuppBySpecific_B = ((((int32_T)LDVSE_IvldLDW_St) & ((int32_T)
    LDWC_SuppErrSpcLDW_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator20' incorporates:
   *  Constant: '<S64>/Constant19'
   *  Constant: '<S64>/V_Parameter29'
   *  Inport: '<Root>/Inport38'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator19'
   */
  LDWC_SuppByVehSt_B = ((((int32_T)LDWSAI_VehStIvld_St) & ((int32_T)
    LDWC_SuppVehIvld_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator15' incorporates:
   *  Constant: '<S64>/Constant20'
   *  Constant: '<S64>/V_Parameter23'
   *  Inport: '<Root>/Inport39'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator20'
   */
  LDWC_SuppByDrvSt_B = ((((int32_T)LDWSAI_IvldStDrv_St) & ((int32_T)
    LDWC_SuppDrvIvld_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator16' incorporates:
   *  Constant: '<S64>/Constant14'
   *  Constant: '<S64>/V_Parameter24'
   *  Inport: '<Root>/Inport40'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator14'
   */
  LDWC_SuppByCtrlSt_B = ((((int32_T)LDWSAI_CtrlStEn_St) & ((int32_T)
    LDWC_SuppDrvActCtrl_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator17' incorporates:
   *  Constant: '<S64>/Constant15'
   *  Constant: '<S64>/V_Parameter25'
   *  Inport: '<Root>/Inport41'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator15'
   */
  LDWC_SuppBySysSt_B = ((((int32_T)LDWSAI_StError_St) & ((int32_T)
    LDWC_SuppVehSysErr_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator18' incorporates:
   *  Constant: '<S64>/Constant16'
   *  Constant: '<S64>/V_Parameter26'
   *  Inport: '<Root>/Inport42'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator16'
   */
  LDWC_SuppyByAvlSt_B = ((((int32_T)LDWSAI_CtrlStNoAvlb_St) & ((int32_T)
    LDWC_SuppNoAvlbVehSys_C_St)) != 0);

  /* RelationalOperator: '<S64>/Relational Operator19' incorporates:
   *  Constant: '<S64>/Constant17'
   *  Constant: '<S64>/V_Parameter27'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator17'
   */
  LDWC_SuppPrjSpec_B = ((((int32_T)LDWSAI_PrjSpecQu_St) & ((int32_T)
    LDWC_SuppFctCstm_C_St)) != 0);

  /* Logic: '<S64>/Logical Operator3' */
  LDWC_Suppresion_B = ((((((LDWC_SuppBySpecific_B || LDWC_SuppByVehSt_B) ||
    LDWC_SuppByDrvSt_B) || LDWC_SuppByCtrlSt_B) || LDWC_SuppBySysSt_B) ||
                        LDWC_SuppyByAvlSt_B) || LDWC_SuppPrjSpec_B);

  /* RelationalOperator: '<S64>/Relational Operator28' incorporates:
   *  Constant: '<S64>/Constant25'
   *  Constant: '<S64>/V_Parameter35'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator25'
   */
  LDWC_WeakRdyBySpecific_B = ((((int32_T)LDVSE_IvldLDW_St) & ((int32_T)
    LDWC_WkRdyErrSpcLDW_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator27' incorporates:
   *  Constant: '<S64>/Constant26'
   *  Constant: '<S64>/V_Parameter36'
   *  Inport: '<Root>/Inport38'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator26'
   */
  LDWC_WeakRdyByVehSt_B = ((((int32_T)LDWSAI_VehStIvld_St) & ((int32_T)
    LDWC_WkRdyVehIvld_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator22' incorporates:
   *  Constant: '<S64>/Constant27'
   *  Constant: '<S64>/V_Parameter30'
   *  Inport: '<Root>/Inport39'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator27'
   */
  LDWC_WeakRdyByDrvSt_B = ((((int32_T)LDWSAI_IvldStDrv_St) & ((int32_T)
    LDWC_WkRdyDrvIVld_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator23' incorporates:
   *  Constant: '<S64>/Constant21'
   *  Constant: '<S64>/V_Parameter31'
   *  Inport: '<Root>/Inport40'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator21'
   */
  LDWC_WeakRdyByCtrlSt_B = ((((int32_T)LDWSAI_CtrlStEn_St) & ((int32_T)
    LDWC_WkRdyDrvActCtrl_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator24' incorporates:
   *  Constant: '<S64>/Constant22'
   *  Constant: '<S64>/V_Parameter32'
   *  Inport: '<Root>/Inport41'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator22'
   */
  LDWC_WeakRdyBySysSt_B = ((((int32_T)LDWSAI_StError_St) & ((int32_T)
    LDWC_WkRdyVehSysErr_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator25' incorporates:
   *  Constant: '<S64>/Constant23'
   *  Constant: '<S64>/V_Parameter33'
   *  Inport: '<Root>/Inport42'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator23'
   */
  LDWC_WeakRdyByAvlSt_B = ((((int32_T)LDWSAI_CtrlStNoAvlb_St) & ((int32_T)
    LDWC_WkRdyNoAvlbVehSys_C_St)) == 0);

  /* RelationalOperator: '<S64>/Relational Operator26' incorporates:
   *  Constant: '<S64>/Constant24'
   *  Constant: '<S64>/V_Parameter34'
   *  Inport: '<Root>/Inport43'
   *  S-Function (sfix_bitop): '<S64>/Bitwise Operator24'
   */
  LDWC_WeakRdyByPrjSpec_B = ((((int32_T)LDWSAI_PrjSpecQu_St) & ((int32_T)
    LDWC_WkRdyFctCstm_C_St)) == 0);

  /* Logic: '<S64>/Logical Operator4' incorporates:
   *  Logic: '<S64>/Logical Operator5'
   */
  LDWC_WkRdy_B = ((((((((!LDWC_Suppresion_B) && LDWC_WeakRdyBySpecific_B) &&
                       LDWC_WeakRdyByVehSt_B) && LDWC_WeakRdyByDrvSt_B) &&
                     LDWC_WeakRdyByCtrlSt_B) && LDWC_WeakRdyBySysSt_B) &&
                   LDWC_WeakRdyByAvlSt_B) && LDWC_WeakRdyByPrjSpec_B);

  /* RelationalOperator: '<S64>/Relational Operator29' incorporates:
   *  Constant: '<S76>/Constant'
   */
  rtb_RelationalOperator29 = (((uint32_T)LDWC_SysOld_St) == E_LDWState_nu_ACTIVE);

  /* Logic: '<S64>/AND' incorporates:
   *  Constant: '<S64>/Constant29'
   *  Constant: '<S75>/Constant'
   *  RelationalOperator: '<S64>/Relational Operator30'
   *  RelationalOperator: '<S64>/Relational Operator32'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  rtb_LDWC_VehicleInvalid_B = ((((uint32_T)LDWC_SysOld_St) ==
    E_LDWState_nu_RAMPOUT) && (((int32_T)LDWC_PrevDgrSide_St) == 1));

  /* Logic: '<S64>/Logical Operator9' */
  LDWC_BlockTimeBySysOut_B = (rtb_RelationalOperator29 ||
    rtb_LDWC_VehicleInvalid_B);

  /* Switch: '<S81>/Switch' incorporates:
   *  Constant: '<S81>/Constant'
   *  Logic: '<S73>/AND'
   *  Logic: '<S73>/NOT'
   *  Switch: '<S81>/Switch1'
   *  UnitDelay: '<S73>/Unit Delay'
   *  UnitDelay: '<S81>/Unit Delay'
   */
  if (rtb_RelationalOperator29 && (!LDWC_EdgeRisActive_B)) {
    LDWC_ActiveStopWatch_sec = 0.0F;
  } else {
    if (rtb_RelationalOperator29) {
      /* UnitDelay: '<S81>/Unit Delay' incorporates:
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S81>/Add'
       *  Switch: '<S81>/Switch1'
       */
      LDWC_ActiveStopWatch_sec = LDWSAI_CycleTime_Sec + LDWC_ActiveStopWatch_sec;
    }
  }

  /* End of Switch: '<S81>/Switch' */

  /* Switch: '<S64>/Switch' incorporates:
   *  Constant: '<S64>/V_Parameter1'
   *  RelationalOperator: '<S64>/GreaterThan'
   *  UnitDelay: '<S81>/Unit Delay'
   */
  if (LDWC_ActiveStopWatch_sec >= LDWC_ContinuActiveTi_C_Sec) {
    /* Switch: '<S64>/Switch' incorporates:
     *  Constant: '<S64>/V_Parameter3'
     */
    LDWC_WRBlockTime_Sec = LDWC_ContiActiveTiFns_C_Sec;
  } else {
    /* Switch: '<S64>/Switch' incorporates:
     *  Constant: '<S64>/V_Parameter2'
     */
    LDWC_WRBlockTime_Sec = LDWC_DlyTiFns_C_Sec;
  }

  /* End of Switch: '<S64>/Switch' */

  /* Logic: '<S64>/Logical Operator8' incorporates:
   *  Constant: '<S77>/Constant'
   *  RelationalOperator: '<S64>/Relational Operator31'
   */
  rtb_LogicalOperator8_m03q = (rtb_LDWC_VehicleInvalid_B || (((uint32_T)
    LDWC_SysOld_St) == E_LDWState_nu_PASSIVE));

  /* Logic: '<S71>/AND' incorporates:
   *  Logic: '<S71>/NOT'
   *  UnitDelay: '<S71>/Unit Delay'
   */
  LDWC_RawBlockTimeByRampOut_B = ((!rtb_LogicalOperator8_m03q) &&
    LDWC_EdgeRisFns_B);

  /* Switch: '<S83>/Switch2' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S83>/GreaterThan'
   *  Switch: '<S83>/Switch'
   *  UnitDelay: '<S83>/Unit Delay'
   */
  if (LDWC_RawBlockTimeByRampOut_B) {
    LDWC_HdTiFns_Sec = LDWC_WRBlockTime_Sec;
  } else if (LDWC_HdTiFns_Sec > LDWSAI_CycleTime_Sec) {
    /* Switch: '<S83>/Switch' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S83>/Subtract'
     *  UnitDelay: '<S83>/Unit Delay'
     */
    LDWC_HdTiFns_Sec -= LDWSAI_CycleTime_Sec;
  } else {
    /* UnitDelay: '<S83>/Unit Delay' incorporates:
     *  Constant: '<S83>/Constant1'
     *  Switch: '<S83>/Switch'
     */
    LDWC_HdTiFns_Sec = 0.0F;
  }

  /* End of Switch: '<S83>/Switch2' */

  /* RelationalOperator: '<S83>/GreaterThan1' incorporates:
   *  Constant: '<S83>/Constant2'
   *  UnitDelay: '<S83>/Unit Delay'
   */
  LDWC_BlockTimeByRampOut_B = (LDWC_HdTiFns_Sec > 0.0F);

  /* Logic: '<S64>/Logical Operator10' */
  LDWC_BlockTime_B = (LDWC_BlockTimeBySysOut_B || LDWC_BlockTimeByRampOut_B);

  /* RelationalOperator: '<S64>/Relational Operator33' incorporates:
   *  Constant: '<S79>/Constant'
   */
  rtb_RelationalOperator33 = (((uint32_T)LDWC_SysOld_St) == E_LDWState_nu_ACTIVE);

  /* Logic: '<S64>/AND1' incorporates:
   *  Constant: '<S64>/Constant31'
   *  Constant: '<S78>/Constant'
   *  RelationalOperator: '<S64>/Relational Operator34'
   *  RelationalOperator: '<S64>/Relational Operator36'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  rtb_LDWC_VehicleInvalid_B = ((((uint32_T)LDWC_SysOld_St) ==
    E_LDWState_nu_RAMPOUT) && (((int32_T)LDWC_PrevDgrSide_St) == 2));

  /* Logic: '<S64>/Logical Operator14' */
  rtb_LogicalOperator14_o2nq = (rtb_RelationalOperator33 ||
    rtb_LDWC_VehicleInvalid_B);

  /* Switch: '<S82>/Switch' incorporates:
   *  Constant: '<S82>/Constant'
   *  Logic: '<S74>/AND'
   *  Logic: '<S74>/NOT'
   *  Switch: '<S82>/Switch1'
   *  UnitDelay: '<S74>/Unit Delay'
   *  UnitDelay: '<S82>/Unit Delay'
   */
  if (rtb_RelationalOperator33 && (!LDWC_EdgeRisActive_Ri_B)) {
    LDWC_ActiveStopWatch_Ri_sec = 0.0F;
  } else {
    if (rtb_RelationalOperator33) {
      /* UnitDelay: '<S82>/Unit Delay' incorporates:
       *  Inport: '<Root>/Inport45'
       *  Sum: '<S82>/Add'
       *  Switch: '<S82>/Switch1'
       */
      LDWC_ActiveStopWatch_Ri_sec += LDWSAI_CycleTime_Sec;
    }
  }

  /* End of Switch: '<S82>/Switch' */

  /* Logic: '<S64>/Logical Operator13' incorporates:
   *  Constant: '<S80>/Constant'
   *  RelationalOperator: '<S64>/Relational Operator35'
   */
  rtb_LogicalOperator13 = (rtb_LDWC_VehicleInvalid_B || (((uint32_T)
    LDWC_SysOld_St) == E_LDWState_nu_PASSIVE));

  /* Switch: '<S84>/Switch2' incorporates:
   *  Inport: '<Root>/Inport45'
   *  Logic: '<S72>/AND'
   *  Logic: '<S72>/NOT'
   *  RelationalOperator: '<S84>/GreaterThan'
   *  Switch: '<S84>/Switch'
   *  UnitDelay: '<S72>/Unit Delay'
   *  UnitDelay: '<S84>/Unit Delay'
   */
  if ((!rtb_LogicalOperator13) && LDWC_EdgeRisFns_Ri_B) {
    /* Switch: '<S64>/Switch1' incorporates:
     *  Constant: '<S64>/V_Parameter4'
     *  Constant: '<S64>/V_Parameter5'
     *  Constant: '<S64>/V_Parameter6'
     *  RelationalOperator: '<S64>/GreaterThan1'
     *  UnitDelay: '<S82>/Unit Delay'
     *  UnitDelay: '<S84>/Unit Delay'
     */
    if (LDWC_ActiveStopWatch_Ri_sec >= LDWC_ContinuActiveTi_C_Sec) {
      LDWC_HdTiFns_Ri_Sec = LDWC_ContiActiveTiFns_C_Sec;
    } else {
      LDWC_HdTiFns_Ri_Sec = LDWC_DlyTiFns_C_Sec;
    }

    /* End of Switch: '<S64>/Switch1' */
  } else if (LDWC_HdTiFns_Ri_Sec > LDWSAI_CycleTime_Sec) {
    /* Switch: '<S84>/Switch' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S84>/Subtract'
     *  UnitDelay: '<S84>/Unit Delay'
     */
    LDWC_HdTiFns_Ri_Sec -= LDWSAI_CycleTime_Sec;
  } else {
    /* UnitDelay: '<S84>/Unit Delay' incorporates:
     *  Constant: '<S84>/Constant1'
     *  Switch: '<S84>/Switch'
     */
    LDWC_HdTiFns_Ri_Sec = 0.0F;
  }

  /* End of Switch: '<S84>/Switch2' */

  /* RelationalOperator: '<S141>/Relational Operator21' incorporates:
   *  Constant: '<S141>/Constant18'
   *  Constant: '<S141>/V_Parameter28'
   *  S-Function (sfix_bitop): '<S141>/Bitwise Operator18'
   */
  rtb_LDWC_VehicleInvalid_B = ((((int32_T)LDVSE_IvldLDW_St) & ((int32_T)
    LDWC_SuppVehicleInvalid_C_St)) != 0);

  /* Abs: '<S143>/Abs' incorporates:
   *  Inport: '<Root>/Inport31'
   */
  rtb_uDLookupTable8_idx_0 = fabsf(LDWSAI_VehYawRate_rps);

  /* Switch: '<S147>/Switch' incorporates:
   *  Constant: '<S143>/Constant10'
   *  Constant: '<S143>/Constant9'
   *  Constant: '<S147>/Constant'
   *  RelationalOperator: '<S147>/Less Than'
   *  RelationalOperator: '<S147>/Less Than1'
   *  Sum: '<S143>/Add2'
   *  UnitDelay: '<S147>/Unit Delay'
   */
  if ((LDWC_VehYawRateMax_C_rps + LDWC_VehYawRateHyst_C_rps) <
      rtb_uDLookupTable8_idx_0) {
    LDWC_VehYawRateHyst_bool = true;
  } else {
    LDWC_VehYawRateHyst_bool = ((rtb_uDLookupTable8_idx_0 >=
      LDWC_VehYawRateMax_C_rps) && (LDWC_VehYawRateHyst_bool));
  }

  /* End of Switch: '<S147>/Switch' */

  /* Switch: '<S137>/Switch' incorporates:
   *  Constant: '<S137>/V_Parameter1'
   *  Constant: '<S137>/V_Parameter2'
   *  Constant: '<S137>/V_Parameter32'
   *  Constant: '<S137>/V_Parameter4'
   *  RelationalOperator: '<S137>/Relational Operator1'
   *  RelationalOperator: '<S137>/Relational Operator2'
   *  RelationalOperator: '<S137>/Relational Operator9'
   *  S-Function (sfix_bitop): '<S137>/Bitwise AND'
   *  Switch: '<S137>/Switch1'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  if (LDWC_PrevDgrSide_St == LDWC_DgrSideLf_C_St) {
    rtb_LDWC_VelYInvalid_B = ((((uint32_T)LDVSE_SidCdtnLDWLf_St) & 2U) != 0U);
  } else if (LDWC_PrevDgrSide_St == LDWC_DgrSideRi_C_St) {
    /* Switch: '<S137>/Switch1' incorporates:
     *  Constant: '<S137>/V_Parameter5'
     *  Constant: '<S137>/V_Parameter6'
     *  RelationalOperator: '<S137>/Relational Operator3'
     *  S-Function (sfix_bitop): '<S137>/Bitwise AND1'
     */
    rtb_LDWC_VelYInvalid_B = ((((uint32_T)LDVSE_SidCdtnLDWRi_St) & 2U) != 0U);
  } else {
    /* Switch: '<S137>/Switch1' incorporates:
     *  Constant: '<S137>/V_Parameter3'
     */
    rtb_LDWC_VelYInvalid_B = false;
  }

  /* End of Switch: '<S137>/Switch' */

  /* Switch: '<S136>/Switch' incorporates:
   *  Constant: '<S136>/V_Parameter2'
   *  Constant: '<S136>/V_Parameter32'
   *  RelationalOperator: '<S136>/Relational Operator1'
   *  RelationalOperator: '<S136>/Relational Operator9'
   *  Switch: '<S136>/Switch1'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  if (LDWC_PrevDgrSide_St == LDWC_DgrSideLf_C_St) {
    /* Logic: '<S136>/NOT' */
    rtb_LDDT_LnCurvVldRi_B = !rtb_LDDT_LnCurvVldLf_B;
  } else if (LDWC_PrevDgrSide_St == LDWC_DgrSideRi_C_St) {
    /* Switch: '<S136>/Switch1' incorporates:
     *  Logic: '<S136>/NOT'
     */
    rtb_LDDT_LnCurvVldRi_B = !rtb_LDDT_LnCurvVldRi_B;
  } else {
    /* Logic: '<S136>/NOT' incorporates:
     *  Logic: '<S136>/OR'
     *  Switch: '<S136>/Switch1'
     */
    rtb_LDDT_LnCurvVldRi_B = ((!rtb_LDDT_LnCurvVldLf_B) &&
      (!rtb_LDDT_LnCurvVldRi_B));
  }

  /* End of Switch: '<S136>/Switch' */

  /* RelationalOperator: '<S135>/Relational Operator21' incorporates:
   *  Constant: '<S135>/Constant18'
   *  Constant: '<S135>/V_Parameter23'
   *  Inport: '<Root>/Inport39'
   *  S-Function (sfix_bitop): '<S135>/Bitwise Operator20'
   */
  rtb_LDDT_LnCurvVldLf_B = ((((uint32_T)LDWSAI_IvldStDrv_St) & 36U) != 0U);

  /* RelationalOperator: '<S139>/Relational Operator21' incorporates:
   *  Constant: '<S139>/Constant18'
   *  Constant: '<S139>/V_Parameter23'
   *  Inport: '<Root>/Inport40'
   *  S-Function (sfix_bitop): '<S139>/Bitwise Operator20'
   */
  rtb_AND_jseu = ((((uint32_T)LDWSAI_CtrlStEn_St) & 45U) != 0U);

  /* Switch: '<S145>/Switch' incorporates:
   *  Constant: '<S139>/V_Parameter1'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S145>/Max'
   *  Sum: '<S145>/Subtract'
   *  Switch: '<S145>/Switch1'
   *  UnaryMinus: '<S145>/Unary Minus'
   *  UnitDelay: '<S145>/Unit Delay'
   */
  if (rtb_AND_jseu) {
    LDWC_SafeFuncActiveTurnOnDelay_sec = fmaxf
      (LDWC_SafeFuncActiveTurnOnDelay_sec, -LDWSAI_CycleTime_Sec) -
      LDWSAI_CycleTime_Sec;
  } else {
    LDWC_SafeFuncActiveTurnOnDelay_sec = LDWC_SafetyFuncMaxTime_sec;
  }

  /* End of Switch: '<S145>/Switch' */

  /* Logic: '<S145>/AND' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S145>/LessThanOrEqual'
   *  UnaryMinus: '<S145>/Unary Minus1'
   *  UnitDelay: '<S145>/Unit Delay'
   */
  rtb_AND_jseu = (rtb_AND_jseu && (LDWC_SafeFuncActiveTurnOnDelay_sec <=
    (-LDWSAI_CycleTime_Sec)));

  /* RelationalOperator: '<S140>/Relational Operator21' incorporates:
   *  Constant: '<S140>/Constant18'
   *  Constant: '<S140>/V_Parameter23'
   *  Inport: '<Root>/Inport41'
   *  S-Function (sfix_bitop): '<S140>/Bitwise Operator20'
   */
  rtb_AND_k3gu = ((((uint32_T)LDWSAI_StError_St) & 11U) != 0U);

  /* Switch: '<S146>/Switch' incorporates:
   *  Constant: '<S140>/V_Parameter1'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S146>/Max'
   *  Sum: '<S146>/Subtract'
   *  Switch: '<S146>/Switch1'
   *  UnaryMinus: '<S146>/Unary Minus'
   *  UnitDelay: '<S146>/Unit Delay'
   */
  if (rtb_AND_k3gu) {
    LDWC_SafeFuncErrorTurnOnDelay_sec = fmaxf(LDWC_SafeFuncErrorTurnOnDelay_sec,
      -LDWSAI_CycleTime_Sec) - LDWSAI_CycleTime_Sec;
  } else {
    LDWC_SafeFuncErrorTurnOnDelay_sec = LDWC_SafetyFuncMaxTime_sec;
  }

  /* End of Switch: '<S146>/Switch' */

  /* Logic: '<S146>/AND' incorporates:
   *  Inport: '<Root>/Inport45'
   *  RelationalOperator: '<S146>/LessThanOrEqual'
   *  UnaryMinus: '<S146>/Unary Minus1'
   *  UnitDelay: '<S146>/Unit Delay'
   */
  rtb_AND_k3gu = (rtb_AND_k3gu && (LDWC_SafeFuncErrorTurnOnDelay_sec <=
    (-LDWSAI_CycleTime_Sec)));

  /* RelationalOperator: '<S142>/Relational Operator21' incorporates:
   *  Constant: '<S142>/Constant18'
   *  Constant: '<S142>/V_Parameter23'
   *  Inport: '<Root>/Inport38'
   *  S-Function (sfix_bitop): '<S142>/Bitwise Operator20'
   */
  rtb_LDWC_VehStInvalid_B = ((((uint32_T)LDWSAI_VehStIvld_St) & 255U) != 0U);

  /* Logic: '<S62>/OR' incorporates:
   *  Inport: '<Root>/Inport47'
   *  UnitDelay: '<S147>/Unit Delay'
   */
  LDWC_Suppression_B = ((((((((rtb_LDWC_VehicleInvalid_B ||
    (LDWC_VehYawRateHyst_bool)) || rtb_LDWC_VelYInvalid_B) ||
    rtb_LDDT_LnCurvVldRi_B) || rtb_LDDT_LnCurvVldLf_B) || rtb_AND_jseu) ||
    rtb_AND_k3gu) || rtb_LDWC_VehStInvalid_B) || LDWSAI_AEBActive_B);

  /* Switch: '<S50>/Switch' incorporates:
   *  UnitDelay: '<S50>/Unit Delay'
   */
  if (LDWC_PrevSwitchUnitDelay_bool) {
    /* Switch: '<S50>/Switch' incorporates:
     *  Inport: '<Root>/Inport9'
     */
    LDWC_NVRAMLDWSwitch_B = LDWSAI_LDWSwitchEn_B;
  } else {
    /* Switch: '<S50>/Switch' incorporates:
     *  Constant: '<S50>/V_Parameter11'
     *  Inport: '<Root>/Inport48'
     *  Switch: '<S50>/Switch1'
     */
    LDWC_NVRAMLDWSwitch_B = (NVRAM_LDWSwitch_B || (LDWC_Switch_C_B));
  }

  /* End of Switch: '<S50>/Switch' */

  /* Chart: '<S5>/LDW_State' incorporates:
   *  Constant: '<S84>/Constant2'
   *  Inport: '<Root>/Inport11'
   *  Logic: '<S64>/AND2'
   *  Logic: '<S64>/AND3'
   *  Logic: '<S64>/Logical Operator11'
   *  Logic: '<S64>/Logical Operator12'
   *  Logic: '<S64>/Logical Operator7'
   *  RelationalOperator: '<S84>/GreaterThan1'
   *  UnitDelay: '<S84>/Unit Delay'
   */
  if (((uint32_T)LDWSA_DW.is_active_c1_LDWSA) == 0U) {
    LDWSA_DW.is_active_c1_LDWSA = 1U;
    if (LDWSAI_LDWErrCdtn_B) {
      LDWSA_DW.is_c1_LDWSA = LDWSA_IN_LDW_ERROR;
      rtb_LDWState = E_LDWState_nu_ERROR;
    } else {
      /* [!Error] */
      LDWSA_DW.is_c1_LDWSA = LDWSA_IN_LDW_OFF;
      rtb_LDWState = E_LDWState_nu_OFF;
    }
  } else {
    switch (LDWSA_DW.is_c1_LDWSA) {
     case LDWSA_IN_LDW_ERROR:
      rtb_LDWState = E_LDWState_nu_ERROR;
      if (!LDWSAI_LDWErrCdtn_B) {
        LDWSA_DW.is_c1_LDWSA = LDWSA_IN_LDW_OFF;
        rtb_LDWState = E_LDWState_nu_OFF;
      } else {
        if ((!LDWSAI_LDWErrCdtn_B) && LDWC_NVRAMLDWSwitch_B) {
          LDWSA_DW.is_c1_LDWSA = LDWSA_IN_LDW_ON;
          LDWSA_DW.is_LDW_ON = LDWSA_IN_LDW_PASSIVE;
          rtb_LDWState = E_LDWState_nu_PASSIVE;
        }
      }
      break;

     case LDWSA_IN_LDW_OFF:
      rtb_LDWState = E_LDWState_nu_OFF;
      if (LDWSAI_LDWErrCdtn_B) {
        LDWSA_DW.is_c1_LDWSA = LDWSA_IN_LDW_ERROR;
        rtb_LDWState = E_LDWState_nu_ERROR;
      } else {
        if ((!LDWSAI_LDWErrCdtn_B) && LDWC_NVRAMLDWSwitch_B) {
          LDWSA_DW.is_c1_LDWSA = LDWSA_IN_LDW_ON;
          LDWSA_DW.is_LDW_ON = LDWSA_IN_LDW_PASSIVE;
          rtb_LDWState = E_LDWState_nu_PASSIVE;
        }
      }
      break;

     default:
      /* case IN_LDW_ON: */
      if (LDWSAI_LDWErrCdtn_B) {
        LDWSA_DW.is_LDW_ON = LDWSA_IN_NO_ACTIVE_CHILD;
        LDWSA_DW.is_c1_LDWSA = LDWSA_IN_LDW_ERROR;
        rtb_LDWState = E_LDWState_nu_ERROR;
      } else if (!LDWC_NVRAMLDWSwitch_B) {
        LDWSA_DW.is_LDW_ON = LDWSA_IN_NO_ACTIVE_CHILD;
        LDWSA_DW.is_c1_LDWSA = LDWSA_IN_LDW_OFF;
        rtb_LDWState = E_LDWState_nu_OFF;
      } else {
        switch (LDWSA_DW.is_LDW_ON) {
         case LDWSA_IN_LDW_ACTIVE:
          rtb_LDWState = E_LDWState_nu_ACTIVE;
          if (LDWC_Suppression_B) {
            LDWSA_DW.is_LDW_ON = LDWSA_IN_LDW_PASSIVE;
            rtb_LDWState = E_LDWState_nu_PASSIVE;
          } else {
            if ((rtb_Equal_azab || LDWC_DgrFns_B) || LDWC_Cancel_B) {
              LDWSA_DW.is_LDW_ON = LDWSA_IN_LDW_STANDBY;
              rtb_LDWState = E_LDWState_nu_STANDBY;
            }
          }
          break;

         case LDWSA_IN_LDW_PASSIVE:
          rtb_LDWState = E_LDWState_nu_PASSIVE;
          if (!LDWC_Suppression_B) {
            LDWSA_DW.is_LDW_ON = LDWSA_IN_LDW_STANDBY;
            rtb_LDWState = E_LDWState_nu_STANDBY;
          }
          break;

         default:
          /* case IN_LDW_STANDBY: */
          rtb_LDWState = E_LDWState_nu_STANDBY;
          if (LDWC_Suppression_B) {
            LDWSA_DW.is_LDW_ON = LDWSA_IN_LDW_PASSIVE;
            rtb_LDWState = E_LDWState_nu_PASSIVE;
          } else {
            if (((LDWC_TrigLf_B && LDWC_StrgRdy_B) && (LDWC_WkRdy_B &&
                  (!LDWC_BlockTime_B))) || ((LDWC_TrigRi_B && LDWC_StrgRdy_B) &&
                 (LDWC_WkRdy_B && ((!rtb_LogicalOperator14_o2nq) &&
                                   (LDWC_HdTiFns_Ri_Sec <= 0.0F))))) {
              LDWSA_DW.is_LDW_ON = LDWSA_IN_LDW_ACTIVE;
              rtb_LDWState = E_LDWState_nu_ACTIVE;
            }
          }
          break;
        }
      }
      break;
    }
  }

  /* RelationalOperator: '<S108>/Relational Operator5' incorporates:
   *  RelationalOperator: '<S48>/Equal'
   *  RelationalOperator: '<S53>/Equal'
   *  Switch: '<S48>/Switch1'
   */
  LDWC_SysOld_St = rtb_LDWState;

  /* RelationalOperator: '<S53>/Equal' incorporates:
   *  Constant: '<S54>/Constant'
   */
  rtb_Equal_azab = (((uint32_T)LDWC_SysOld_St) == E_LDWState_nu_STANDBY);

  /* Switch: '<S57>/Switch2' incorporates:
   *  Constant: '<S55>/Constant'
   *  Inport: '<Root>/Inport45'
   *  Logic: '<S53>/AND'
   *  RelationalOperator: '<S53>/Equal1'
   *  RelationalOperator: '<S57>/GreaterThan'
   *  Switch: '<S57>/Switch'
   *  UnitDelay: '<S53>/Unit Delay'
   *  UnitDelay: '<S57>/Unit Delay'
   */
  if ((((uint32_T)LDWC_PrevSysOutIn_St) == E_LDWState_nu_ACTIVE) &&
      rtb_Equal_azab) {
    LDWC_SusTimeExpiredTimerRetrigger_sec = rtb_LDWC_RampoutTime_Sec;
  } else if (LDWC_SusTimeExpiredTimerRetrigger_sec > LDWSAI_CycleTime_Sec) {
    /* Switch: '<S57>/Switch' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S57>/Subtract'
     *  UnitDelay: '<S57>/Unit Delay'
     */
    LDWC_SusTimeExpiredTimerRetrigger_sec =
      LDWC_SusTimeExpiredTimerRetrigger_sec - LDWSAI_CycleTime_Sec;
  } else {
    /* UnitDelay: '<S57>/Unit Delay' incorporates:
     *  Constant: '<S57>/Constant1'
     *  Switch: '<S57>/Switch'
     */
    LDWC_SusTimeExpiredTimerRetrigger_sec = 0.0F;
  }

  /* End of Switch: '<S57>/Switch2' */

  /* Switch: '<S56>/Switch' incorporates:
   *  Constant: '<S56>/Constant2'
   *  Constant: '<S57>/Constant2'
   *  Logic: '<S53>/NOT1'
   *  Logic: '<S53>/NOT2'
   *  RelationalOperator: '<S57>/GreaterThan1'
   *  UnitDelay: '<S56>/Unit Delay'
   *  UnitDelay: '<S57>/Unit Delay'
   */
  if (!rtb_Equal_azab) {
    LDWC_RampTimeExpiredRSFF_bool = false;
  } else {
    LDWC_RampTimeExpiredRSFF_bool = ((LDWC_SusTimeExpiredTimerRetrigger_sec <=
      0.0F) || (LDWC_RampTimeExpiredRSFF_bool));
  }

  /* End of Switch: '<S56>/Switch' */

  /* Switch: '<S48>/Switch1' incorporates:
   *  Constant: '<S51>/Constant'
   *  Logic: '<S48>/OR'
   *  RelationalOperator: '<S48>/Equal'
   *  UnitDelay: '<S56>/Unit Delay'
   */
  if ((((uint32_T)LDWC_SysOld_St) != E_LDWState_nu_STANDBY) ||
      (LDWC_RampTimeExpiredRSFF_bool)) {
    /* Switch: '<S48>/Switch1' */
    LDWC_SysOut_St = LDWC_SysOld_St;
  } else {
    /* Switch: '<S48>/Switch1' incorporates:
     *  Constant: '<S52>/Constant'
     */
    LDWC_SysOut_St = E_LDWState_nu_RAMPOUT;
  }

  /* SignalConversion: '<S62>/Signal Conversion' */
  rtb_VectorConcatenate_oaf0[0] = rtb_LDWC_VehicleInvalid_B;

  /* SignalConversion: '<S62>/Signal Conversion1' incorporates:
   *  UnitDelay: '<S147>/Unit Delay'
   */
  rtb_VectorConcatenate_oaf0[1] = LDWC_VehYawRateHyst_bool;

  /* SignalConversion: '<S62>/Signal Conversion2' */
  rtb_VectorConcatenate_oaf0[2] = rtb_LDWC_VelYInvalid_B;

  /* SignalConversion: '<S62>/Signal Conversion3' */
  rtb_VectorConcatenate_oaf0[3] = rtb_LDDT_LnCurvVldRi_B;

  /* SignalConversion: '<S62>/Signal Conversion4' */
  rtb_VectorConcatenate_oaf0[4] = rtb_LDDT_LnCurvVldLf_B;

  /* SignalConversion: '<S62>/Signal Conversion5' */
  rtb_VectorConcatenate_oaf0[5] = rtb_AND_jseu;

  /* SignalConversion: '<S62>/Signal Conversion6' */
  rtb_VectorConcatenate_oaf0[6] = rtb_AND_k3gu;

  /* SignalConversion: '<S62>/Signal Conversion7' */
  rtb_VectorConcatenate_oaf0[7] = rtb_LDWC_VehStInvalid_B;

  /* SignalConversion: '<S62>/Signal Conversion9' incorporates:
   *  Inport: '<Root>/Inport47'
   */
  rtb_VectorConcatenate_oaf0[8] = LDWSAI_AEBActive_B;

  /* S-Function (ex_sfun_set_bit): '<S144>/ex_sfun_set_bit' incorporates:
   *  Constant: '<S138>/Constant'
   */
  set_bit(0U, (boolean_T*)&rtb_VectorConcatenate_oaf0[0], (uint8_T*)
          (&(LDWSA_SetBit_BS_Param_3[0])), ((uint8_T)9U),
          &rtb_ex_sfun_set_bit_dmuc);

  /* SignalConversion: '<S62>/Signal Conversion8' incorporates:
   *  DataTypeConversion: '<S144>/Data Type Conversion1'
   */
  LDWC_SuppValid_Debug = (uint16_T)rtb_ex_sfun_set_bit_dmuc;

  /* Switch: '<S105>/Switch6' incorporates:
   *  Constant: '<S105>/V_Parameter3'
   *  Constant: '<S105>/V_Parameter5'
   *  Constant: '<S105>/V_Parameter8'
   *  Inport: '<Root>/Inport2'
   *  RelationalOperator: '<S105>/Relational Operator4'
   *  RelationalOperator: '<S105>/Relational Operator5'
   *  RelationalOperator: '<S105>/Relational Operator6'
   *  Switch: '<S105>/Switch7'
   *  Switch: '<S105>/Switch8'
   *  UnitDelay: '<S2>/Unit Delay'
   */
  if (LDWC_PrevDgrSide_St == LDWC_NoDgrSide_C_St) {
    /* Switch: '<S105>/Switch6' incorporates:
     *  Constant: '<S105>/Constant1'
     */
    ELDWTriggerDgrForHMI = 0U;
  } else if (LDWC_PrevDgrSide_St == LDWC_DgrSideLf_C_St) {
    /* Switch: '<S105>/Switch5' incorporates:
     *  Constant: '<S105>/V_Parameter1'
     *  Inport: '<Root>/Inport2'
     *  RelationalOperator: '<S105>/Relational Operator3'
     *  Switch: '<S105>/Switch7'
     */
    if (LDWSAI_SpdVelShow_Kmph >= VehicleSpeedThresholdHMI_Kph) {
      /* Switch: '<S105>/Switch6' incorporates:
       *  Constant: '<S105>/Constant2'
       */
      ELDWTriggerDgrForHMI = 3U;
    } else {
      /* Switch: '<S105>/Switch6' incorporates:
       *  Constant: '<S105>/Constant'
       */
      ELDWTriggerDgrForHMI = 1U;
    }

    /* End of Switch: '<S105>/Switch5' */
  } else if (LDWSAI_SpdVelShow_Kmph >= VehicleSpeedThresholdHMI_Kph) {
    /* Switch: '<S105>/Switch8' incorporates:
     *  Constant: '<S105>/Constant4'
     *  Switch: '<S105>/Switch6'
     *  Switch: '<S105>/Switch7'
     */
    ELDWTriggerDgrForHMI = 4U;
  } else {
    /* Switch: '<S105>/Switch6' incorporates:
     *  Constant: '<S105>/Constant3'
     *  Switch: '<S105>/Switch7'
     *  Switch: '<S105>/Switch8'
     */
    ELDWTriggerDgrForHMI = 2U;
  }

  /* End of Switch: '<S105>/Switch6' */

  /* Logic: '<S104>/Logical Operator1' */
  LDWC_Trig_B = (LDWC_TrigRi_B || LDWC_TrigLf_B);

  /* Sum: '<S121>/Subtract2' incorporates:
   *  Inport: '<Root>/Inport45'
   */
  rtb_uDLookupTable9_idx_1 -= LDWSAI_CycleTime_Sec;

  /* MinMax: '<S121>/Max' incorporates:
   *  Constant: '<S121>/Constant5'
   *  UnitDelay: '<S121>/Unit Delay'
   */
  LDWC_SuppTimeOldLf_Sec = fmaxf(rtb_uDLookupTable9_idx_1, 0.0F);

  /* Sum: '<S120>/Subtract2' incorporates:
   *  Inport: '<Root>/Inport45'
   */
  rtb_Subtract2_hwoc -= LDWSAI_CycleTime_Sec;

  /* MinMax: '<S120>/Max' incorporates:
   *  Constant: '<S120>/Constant5'
   *  UnitDelay: '<S120>/Unit Delay'
   */
  LDWC_HdTiTrigLf_Sec = fmaxf(rtb_Subtract2_hwoc, 0.0F);

  /* Sum: '<S133>/Subtract2' incorporates:
   *  Constant: '<S133>/Constant5'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S133>/Max'
   */
  LDWC_SuppTimeOldRi_Sec -= LDWSAI_CycleTime_Sec;
  LDWC_SuppTimeOldRi_Sec = fmaxf(LDWC_SuppTimeOldRi_Sec, 0.0F);

  /* Switch: '<S106>/Switch3' incorporates:
   *  Constant: '<S132>/Constant5'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S132>/Max'
   *  Sum: '<S132>/Subtract2'
   */
  LDWC_HdTiTrigRi_Sec -= LDWSAI_CycleTime_Sec;
  LDWC_HdTiTrigRi_Sec = fmaxf(LDWC_HdTiTrigRi_Sec, 0.0F);

  /* Sum: '<S36>/Subtract2' incorporates:
   *  Constant: '<S36>/Constant5'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S36>/Max'
   */
  LDVSE_HodTiTrnSglRi_Sec -= LDWSAI_CycleTime_Sec;
  LDVSE_HodTiTrnSglRi_Sec = fmaxf(LDVSE_HodTiTrnSglRi_Sec, 0.0F);

  /* Sum: '<S35>/Subtract2' incorporates:
   *  Constant: '<S35>/Constant5'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S35>/Max'
   */
  LDVSE_HodTiTrnSglLf_Sec -= LDWSAI_CycleTime_Sec;
  LDVSE_HodTiTrnSglLf_Sec = fmaxf(LDVSE_HodTiTrnSglLf_Sec, 0.0F);

  /* Lookup_n-D: '<S106>/1-D Lookup Table2' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDWC_DlcThdMode2_Mi = look1_iflf_binlxpw(LDWSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDWC_VehSpdXDTL_BX_Mps[0])), ((const real32_T *)
    &(LDWC_DstcTrsdVehSpdXDTL2_Cr_Mi[0])), 8U);

  /* Lookup_n-D: '<S106>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDWC_DlcThdMode1_Mi = look1_iflf_binlxpw(LDWSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDWC_VehSpdXDTL_BX_Mps[0])), ((const real32_T *)
    &(LDWC_DstcTrsdVehSpdXDTL1_Cr_Mi[0])), 8U);

  /* Lookup_n-D: '<S106>/1-D Lookup Table1' incorporates:
   *  Inport: '<Root>/Inport1'
   */
  LDWC_DlcThdMode3_Mi = look1_iflf_binlxpw(LDWSAI_VehSpdActu_Mps, ((const
    real32_T *)&(LDWC_VehSpdXDTL_BX_Mps[0])), ((const real32_T *)
    &(LDWC_DstcTrsdVehSpdXDTL3_Cr_Mi[0])), 8U);

  /* Switch: '<S106>/Switch3' incorporates:
   *  Constant: '<S106>/V_Parameter10'
   *  Constant: '<S106>/V_Parameter9'
   *  Inport: '<Root>/Inport10'
   *  RelationalOperator: '<S106>/Relational Operator1'
   *  RelationalOperator: '<S106>/Relational Operator2'
   *  Switch: '<S106>/Switch2'
   */
  if (LDWSAI_LDWMod_St == LDWC_DrvMod2_C_St) {
    rtb_LDWC_RampoutTime_Sec = LDWC_DlcThdMode2_Mi;
  } else if (LDWSAI_LDWMod_St == LDWC_DrvMod3_C_St) {
    /* Switch: '<S106>/Switch2' */
    rtb_LDWC_RampoutTime_Sec = LDWC_DlcThdMode3_Mi;
  } else {
    rtb_LDWC_RampoutTime_Sec = LDWC_DlcThdMode1_Mi;
  }

  /* End of Switch: '<S106>/Switch3' */

  /* Product: '<S106>/Product' */
  LDWC_DstcToLnTrsd_Mi = rtb_LDWC_RampoutTime_Sec * LDWC_CrrctByLnWidth_Fct;

  /* Update for UnitDelay: '<S33>/Unit Delay' */
  LDVSE_EdgeRisTrnSglRi_B = rtb_RelationalOperator1;

  /* Update for UnitDelay: '<S34>/Unit Delay' */
  LDVSE_EdgeRisTrnSglLf_B = rtb_RelationalOperator2;

  /* Update for UnitDelay: '<S123>/Unit Delay' */
  LDWC_HdTiTrigRiEn_B = rtb_RelationalOperator4;

  /* Update for RelationalOperator: '<S108>/Relational Operator5' incorporates:
   *  Switch: '<S48>/Switch1'
   *  UnitDelay: '<S5>/UnitDelay'
   */
  LDWC_SysOld_St = LDWC_SysOut_St;

  /* Update for UnitDelay: '<S126>/Unit Delay' */
  LDWC_PreActiveEdgeRi = rtb_RelationalOperator10;

  /* Update for UnitDelay: '<S127>/Unit Delay' */
  LDWC_ContinTrigRiEn_B = rtb_LogicalOperator14;

  /* Update for UnitDelay: '<S111>/Unit Delay' */
  LDWC_HdTiTrigLfEn_B = rtb_LogicalOperator2_i4cj;

  /* Update for UnitDelay: '<S114>/Unit Delay' */
  LDWC_PreActiveEdgeLf = rtb_RelationalOperator10_f5fy;

  /* Update for UnitDelay: '<S115>/Unit Delay' */
  LDWC_ContinTrigLfEn_B = rtb_LogicalOperator14_iqhs;

  /* Update for UnitDelay: '<S87>/Unit Delay' */
  LDWC_EdgeRisWarming_B = LDWC_MinLdwBySysSt_B;

  /* Update for UnitDelay: '<S68>/Unit Delay' */
  LDWC_EdgeRisWarmMx_B = LDWC_MaxDurationBySysSt_B;

  /* Update for UnitDelay: '<S85>/Unit Delay' */
  LDWC_EdgeRisDegr_B = LDWC_Degradation_B;

  /* Update for UnitDelay: '<S59>/UnitDelay' */
  LDWC_DegrOld_B = LDWC_Degr_B;

  /* Update for UnitDelay: '<S73>/Unit Delay' */
  LDWC_EdgeRisActive_B = rtb_RelationalOperator29;

  /* Update for UnitDelay: '<S71>/Unit Delay' */
  LDWC_EdgeRisFns_B = rtb_LogicalOperator8_m03q;

  /* Update for UnitDelay: '<S74>/Unit Delay' */
  LDWC_EdgeRisActive_Ri_B = rtb_RelationalOperator33;

  /* Update for UnitDelay: '<S72>/Unit Delay' */
  LDWC_EdgeRisFns_Ri_B = rtb_LogicalOperator13;

  /* Update for UnitDelay: '<S50>/Unit Delay' incorporates:
   *  Inport: '<Root>/Inport9'
   */
  LDWC_PrevSwitchUnitDelay_bool = LDWSAI_LDWSwitchEn_B;

  /* Update for UnitDelay: '<S53>/Unit Delay' */
  LDWC_PrevSysOutIn_St = rtb_LDWState;

  /* End of Outputs for S-Function (fcgen): '<S1>/Function-Call Generator' */

  /* SignalConversion generated from: '<S1>/LDW' */
  LDVSE_NVRAMVehStartupSpd_kmph = (real32_T)LDVSE_NVRAMVehStartupSpd_k_chkk;

  /* S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/LDW'
   */
  /* SignalConversion generated from: '<S1>/LDW' incorporates:
   *  Logic: '<S64>/Logical Operator'
   */
  LDWC_RdyToTrig_B = (LDWC_StrgRdy_B && LDWC_WkRdy_B);

  /* End of Outputs for S-Function (fcgen): '<S1>/Function-Call Generator' */

  /* SignalConversion generated from: '<S1>/LDW' incorporates:
   *  UnitDelay: '<S2>/Unit Delay'
   */
  LDWC_DgrSide_St = LDWC_PrevDgrSide_St;
}

/* Model initialize function */
void LDWSA_initialize(void)
{
  /* Registration code */

  /* block I/O */

  /* exported global signals */
  LDVSE_NVRAMVehStartupSpd_kmph = 0.0F;
  LDWC_CrvSensiDecayRi_Mi = 0.0F;
  LDDT_CrvThdMaxRi_ReMi = 0.0F;
  LDDT_CrvThdHystRi_ReMi = 0.0F;
  LDDT_LnCltdCurvRi_ReMi = 0.0F;
  LDDT_LnHeadRi_Rad = 0.0F;
  LDDT_RawLatVehSpdRi_Mps = 0.0F;
  LDDT_LatVehSpdRi_Mps = 0.0F;
  LDWC_CrrctByLnWidth_Fct = 0.0F;
  LDWC_DstcToLnTrsdCrvCpstnRi_Mi = 0.0F;
  LDWC_DstcToLnTrsdRi_Mi = 0.0F;
  LDDT_LnPstnRi_Mi = 0.0F;
  LDDT_RawDstcToLnRi_Mi = 0.0F;
  LDDT_DstcToLnRi_Mi = 0.0F;
  LDVSE_MaxLatVel_Mps = 0.0F;
  LDDT_CrvThdMaxLf_ReMi = 0.0F;
  LDDT_CrvThdHystLf_ReMi = 0.0F;
  LDDT_LnCltdCurvLf_ReMi = 0.0F;
  LDDT_LnHeadLf_Rad = 0.0F;
  LDDT_RawLatVehSpdLf_Mps = 0.0F;
  LDDT_LatVehSpdLf_Mps = 0.0F;
  LDVSE_MaxCrvBySpd_ReMi = 0.0F;
  LDVSE_HystCrvBySpd_ReMi = 0.0F;
  LDDT_TiToLnRi_Sec = 0.0F;
  LDWC_TiToLnTrsd_Sec = 0.0F;
  LDDT_LnPstnLf_Mi = 0.0F;
  LDDT_RawDstcToLnLf_Mi = 0.0F;
  LDDT_DstcToLnLf_Mi = 0.0F;
  LDWC_DstcToLnTrsdCrvCpstnLf_Mi = 0.0F;
  LDWC_CrvSensiDecayLe_Mi = 0.0F;
  LDWC_DstcToLnTrsdLf_Mi = 0.0F;
  LDDT_TiToLnLf_Sec = 0.0F;
  LDWC_WRBlockTime_Sec = 0.0F;
  LDWC_DlcThdMode2_Mi = 0.0F;
  LDWC_DlcThdMode1_Mi = 0.0F;
  LDWC_DlcThdMode3_Mi = 0.0F;
  LDWC_DstcToLnTrsd_Mi = 0.0F;
  LDWC_SuppValid_Debug = ((uint16_T)0U);
  LDWC_DgrSide_St = ((uint8_T)0U);
  LDDT_CurveTypeRi_St = ((uint8_T)0U);
  LDVSE_SidCdtnLDWRi_St = ((uint8_T)0U);
  LDDT_CurveTypeLe_St = ((uint8_T)0U);
  LDVSE_SidCdtnLDWLf_St = ((uint8_T)0U);
  LDVSE_IvldLDW_St = ((uint8_T)0U);
  ELDWTriggerDgrForHMI = 0U;
  LDWC_NVRAMLDWSwitch_B = false;
  LDWC_RdyToTrig_B = false;
  LDDT_RdyTrigLDW_B = false;
  LDDT_EnaSafety_B = false;
  LDDT_EnaByInVldQlfrRi_B = false;
  LDDT_EnaByInVldQlfrSfRi_B = false;
  LDDT_LnTrigVldRi_B = false;
  LDDT_CclByInVldQlfrRi_B = false;
  LDDT_LnCclVldRi_B = false;
  LDDT_LnMakVldRi_B = false;
  LDWC_RawTrigByDlcRi_B = false;
  LDVSE_RdyTrigLDW_B = false;
  LDDT_EnaByCstruSiteLf_B = false;
  LDDT_EnaByInVldQlfrLf_B = false;
  LDDT_EnaByInVldQlfrSfLf_B = false;
  LDDT_LnTrigVldLf_B = false;
  LDDT_CclByInVldQlfrLf_B = false;
  LDDT_LnCclVldLf_B = false;
  LDDT_LnMakVldLf_B = false;
  LDVSE_VehLatSpdVldLf_B = false;
  LDVSE_TrnSglLf_B = false;
  LDVSE_VehLatSpdVldRi_B = false;
  LDVSE_TrnSglRi_B = false;
  LDWC_EnaTlcTrigRi_B = false;
  LDWC_RawTrigByTlcRi_B = false;
  LDWC_DlyTrigByTlcRi_B = false;
  LDWC_EnaLdwTrigRi_B = false;
  LDWC_RstTlcTrigRi_B = false;
  LDWC_ResetForSafeRi_B = false;
  LDWC_SetForSafeRi_B = false;
  LDWC_SetForContinTrigRi_B = false;
  LDWC_ResetForContinTrigRi_B = false;
  LDWC_TrigBySideCondRi_B = false;
  LDWC_TrigByPrjSpecRi_B = false;
  LDWC_TrigRi_B = false;
  LDWC_RawTrigByDlcLf_B = false;
  LDWC_EnaTlcTrigLf_B = false;
  LDWC_RawTrigByTlcLf_B = false;
  LDWC_DlyTrigByTlcLf_B = false;
  LDWC_EnaLdwTrigLf_B = false;
  LDWC_RstTlcTrigLf_B = false;
  LDWC_ResetForSafeLf_B = false;
  LDWC_SetForSafeLf_B = false;
  LDWC_ResetForContinTrigLf_B = false;
  LDWC_SetForContinTrigLf_B = false;
  LDWC_TrigBySideCondLf_B = false;
  LDWC_TrigByPrjSpecLf_B = false;
  LDWC_TrigLf_B = false;
  LDWC_EnaDgrSide_B = false;
  LDWC_FnsByDgrStLf_B = false;
  LDWC_FnsByLatDistLf_B = false;
  LDWC_FnsByHeadingLf_B = false;
  LDWC_FnsByLatSpdLf_B = false;
  LDWC_DgrFnsLf_B = false;
  LDWC_FnsByDgrStRi_B = false;
  LDWC_FnsByLatDistRi_B = false;
  LDWC_FnsByHeadingRi_B = false;
  LDWC_FnsByLatSpdRi_B = false;
  LDWC_DgrFnsRi_B = false;
  LDWC_MinLdwBySysSt_B = false;
  LDWC_EdgeRiseForMinLdw_B = false;
  LDWC_HoldForMinLdw_B = false;
  LDWC_FlagMinTimeLDW_B = false;
  LDWC_DgrFns_B = false;
  LDWC_CancelBySpecific_B = false;
  LDWC_CancelByVehSt_B = false;
  LDWC_CancelByDrvSt_B = false;
  LDWC_CancelByCtrlSt_B = false;
  LDWC_CancelBySysSt_B = false;
  LDWC_CancelByAvlSt_B = false;
  LDWC_CancelByPrjSpec_B = false;
  LDWC_MaxDurationBySysSt_B = false;
  LDWC_EdgRiseForSysSt_B = false;
  LDWC_MaxDurationByStDly_B = false;
  LDWC_TiWarmMx_B = false;
  LDWC_ErrSideByTrigLf_B = false;
  LDWC_ErrSideBySideCondLf_B = false;
  LDWC_ErrSidByPrjSpecLf_B = false;
  LDWC_ErrSidCdtnLf_B = false;
  LDWC_SideCondByDgrLf_B = false;
  LDWC_CanelBySideLf_B = false;
  LDWC_SideCondByDgrRi_B = false;
  LDWC_ErrSideByTrigRi_B = false;
  LDWC_ErrSideBySideCondRi_B = false;
  LDWC_ErrSidByPrjSpecRi_B = false;
  LDWC_ErrSidCdtnRi_B = false;
  LDWC_CanelBySideRi_B = false;
  LDWC_ErrSidCdtn_B = false;
  LDWC_CLatDevByDlcLf_B = false;
  LDWC_CLatDevByDgrLf_B = false;
  LDWC_CclLatDevLf_B = false;
  LDWC_CLatDevByDlcRi_B = false;
  LDWC_CLatDevByDgrRi_B = false;
  LDWC_CclLatDevRi_B = false;
  LDWC_CclLatDev_B = false;
  LDWC_Cancel_B = false;
  LDWC_AbortBySpecific_B = false;
  LDWC_AbortByVehSt_B = false;
  LDWC_AbortByDrvSt_B = false;
  LDWC_AbortByCtrlSt_B = false;
  LDWC_AbortBySysSt_B = false;
  LDWC_AbortByAvlSt_B = false;
  LDWC_AbortByPrjSpec_B = false;
  LDWC_Abort_B = false;
  LDWC_StrgRdyBySpecific_B = false;
  LDWC_StrgRdyByVehSt_B = false;
  LDWC_StrgRdyByDrvSt_B = false;
  LDWC_StrgRdyByCtrlSt_B = false;
  LDWC_StrgRdyBySysSt_B = false;
  LDWC_StrgRdyByAvlSt_B = false;
  LDWC_StrgRdyByPrjSpec_B = false;
  LDWC_StrgRdy_B = false;
  LDWC_Degradation_B = false;
  LDWC_DegradationEdgeRise_B = false;
  LDWC_Degr_B = false;
  LDWC_SuppBySpecific_B = false;
  LDWC_SuppByVehSt_B = false;
  LDWC_SuppByDrvSt_B = false;
  LDWC_SuppByCtrlSt_B = false;
  LDWC_SuppBySysSt_B = false;
  LDWC_SuppyByAvlSt_B = false;
  LDWC_SuppPrjSpec_B = false;
  LDWC_Suppresion_B = false;
  LDWC_WeakRdyBySpecific_B = false;
  LDWC_WeakRdyByVehSt_B = false;
  LDWC_WeakRdyByDrvSt_B = false;
  LDWC_WeakRdyByCtrlSt_B = false;
  LDWC_WeakRdyBySysSt_B = false;
  LDWC_WeakRdyByAvlSt_B = false;
  LDWC_WeakRdyByPrjSpec_B = false;
  LDWC_WkRdy_B = false;
  LDWC_BlockTimeBySysOut_B = false;
  LDWC_RawBlockTimeByRampOut_B = false;
  LDWC_BlockTimeByRampOut_B = false;
  LDWC_BlockTime_B = false;
  LDWC_Suppression_B = false;
  LDWC_Trig_B = false;
  LDWC_SysOut_St = E_LDWState_nu_OFF;

  /* states (dwork) */
  (void) memset((void *)&LDWSA_DW, 0,
                sizeof(DW_LDWSA_T));

  /* exported global states */
  LDVSE_HodTiTrnSglLf_Sec = 0.0F;
  LDVSE_HodTiTrnSglRi_Sec = 0.0F;
  LDWC_DlyTiOfTiToLnRiMn_Sec = 0.0F;
  LDWC_HdTiTrigRi_Sec = 0.0F;
  LDWC_ContinWarmTimesOldRi_Count = 0.0F;
  LDWC_SuppTimeOldRi_Sec = 0.0F;
  LDWC_DlyTiOfTiToLnLfMn_Sec = 0.0F;
  LDWC_HdTiTrigLf_Sec = 0.0F;
  LDWC_ContinWarmTimesOldLf_Count = 0.0F;
  LDWC_SuppTimeOldLf_Sec = 0.0F;
  LDWC_HdTiWarming_Sec = 0.0F;
  LDWC_DlyTiTgtFns_Sec = 0.0F;
  LDWC_HdTiWarmMx_Sec = 0.0F;
  LDWC_HdTiDegr_Sec = 0.0F;
  LDWC_HdTiFns_Sec = 0.0F;
  LDWC_ActiveStopWatch_Ri_sec = 0.0F;
  LDWC_HdTiFns_Ri_Sec = 0.0F;
  LDWC_DgrSideOld_St = 0U;
  LDDT_UHysCltdCurvVldRi_B = false;
  LDDT_BHysHeadAglTrigVldRi_B = false;
  LDDT_UHysHeadAglCclVldRi_B = false;
  LDVSE_BHysLatVehSpdVldLf_B = false;
  LDDT_UHysCltdCurvVldLf_B = false;
  LDDT_BHysHeadAglTrigVldLf_B = false;
  LDDT_UHysHeadAglCclVldLf_B = false;
  LDVSE_UHysLatVehSpdVldLf_B = false;
  LDVSE_EdgeRisTrnSglRi_B = false;
  LDVSE_BHysLatVehSpdVldRi_B = false;
  LDVSE_UHysLatVehSpdVldRi_B = false;
  LDVSE_EdgeRisTrnSglLf_B = false;
  LDVSE_UHysSteAgl_B = false;
  LDVSE_BHysSpdVeh_B = false;
  LDVSE_UHysSteAglSpd_B = false;
  LDVSE_BHysAccVehX_B = false;
  LDVSE_BHysAccVehY_B = false;
  LDVSE_UHysVehCurv_B = false;
  LDVSE_BHysLnWid_B = false;
  LDWC_HdTiTrigRiEn_B = false;
  LDWC_DisTrigRi_B = false;
  LDWC_SuppFlagOldRi_B = false;
  LDWC_PreActiveEdgeRi = false;
  LDWC_ContinTrigRiEn_B = false;
  LDWC_DisContinTrigRi_B = false;
  LDWC_HdTiTrigLfEn_B = false;
  LDWC_DisTrigLf_B = false;
  LDWC_SuppFlagOldLf_B = false;
  LDWC_PreActiveEdgeLf = false;
  LDWC_ContinTrigLfEn_B = false;
  LDWC_DisContinTrigLf_B = false;
  LDWC_EdgeRisWarming_B = false;
  LDWC_EdgeRisWarmMx_B = false;
  LDWC_EdgeRisDegr_B = false;
  LDWC_DegrOld_B = false;
  LDWC_EdgeRisFns_B = false;
  LDWC_EdgeRisActive_Ri_B = false;
  LDWC_EdgeRisFns_Ri_B = false;

  /* custom states */
  LDWC_ActiveStopWatch_sec = 0.0F;
  LDWC_SafeFuncActiveTurnOnDelay_sec = 0.0F;
  LDWC_SafeFuncErrorTurnOnDelay_sec = 0.0F;
  LDWC_SusTimeExpiredTimerRetrigger_sec = 0.0F;
  LDWC_PrevDgrSide_St = 0U;
  LDVSE_PrevVehStartupSpd_Kmph = 0U;
  LDDT_LnLengthRi_B = false;
  LDDT_LnLengthLf_B = false;
  LDWC_EdgeRisActive_B = false;
  LDWC_VehYawRateHyst_bool = false;
  LDWC_PrevSwitchUnitDelay_bool = false;
  LDWC_RampTimeExpiredRSFF_bool = false;
  LDWC_SysOld_St = E_LDWState_nu_OFF;
  LDWC_PrevSysOutIn_St = E_LDWState_nu_OFF;

  /* SystemInitialize for S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/LDW'
   */
  /* InitializeConditions for UnitDelay: '<S33>/Unit Delay' */
  LDVSE_EdgeRisTrnSglRi_B = true;

  /* InitializeConditions for UnitDelay: '<S34>/Unit Delay' */
  LDVSE_EdgeRisTrnSglLf_B = true;

  /* InitializeConditions for UnitDelay: '<S123>/Unit Delay' */
  LDWC_HdTiTrigRiEn_B = true;

  /* InitializeConditions for UnitDelay: '<S126>/Unit Delay' */
  LDWC_PreActiveEdgeRi = true;

  /* InitializeConditions for UnitDelay: '<S127>/Unit Delay' */
  LDWC_ContinTrigRiEn_B = true;

  /* InitializeConditions for UnitDelay: '<S111>/Unit Delay' */
  LDWC_HdTiTrigLfEn_B = true;

  /* InitializeConditions for UnitDelay: '<S114>/Unit Delay' */
  LDWC_PreActiveEdgeLf = true;

  /* InitializeConditions for UnitDelay: '<S115>/Unit Delay' */
  LDWC_ContinTrigLfEn_B = true;

  /* InitializeConditions for UnitDelay: '<S87>/Unit Delay' */
  LDWC_EdgeRisWarming_B = true;

  /* InitializeConditions for UnitDelay: '<S68>/Unit Delay' */
  LDWC_EdgeRisWarmMx_B = true;

  /* InitializeConditions for UnitDelay: '<S85>/Unit Delay' */
  LDWC_EdgeRisDegr_B = true;

  /* InitializeConditions for UnitDelay: '<S73>/Unit Delay' */
  LDWC_EdgeRisActive_B = true;

  /* InitializeConditions for UnitDelay: '<S74>/Unit Delay' */
  LDWC_EdgeRisActive_Ri_B = true;

  /* SystemInitialize for Chart: '<S5>/LDW_State' */
  LDWSA_DW.is_LDW_ON = LDWSA_IN_NO_ACTIVE_CHILD;
  LDWSA_DW.is_active_c1_LDWSA = 0U;
  LDWSA_DW.is_c1_LDWSA = LDWSA_IN_NO_ACTIVE_CHILD;

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