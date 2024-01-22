#ifndef TRJPLN_CONSTS_H
#define TRJPLN_CONSTS_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#ifndef P_TPLLCO_CCCENABLE_NU
#define P_TPLLCO_CCCENABLE_NU TRUE
#endif

#define TPLLCO_CCCMINTESTDISTX_MET 5.0f
#define TPLLCO_CCCTESTDISXPERC_NU 0.5f
#define TPLLCO_CCCRESETMAXDISTX_MET 60.0f
#define TPLLCO_CCCRESETMAXPERC_NU 0.7f
#define TPLLCO_CCCMINLNQUALITY_PERC 70.f
#define TPLLCO_CCCMINCRVQUALITY_PERC 60.f
#define TPLLCO_CCCMINQUCYCLE_NU 6u
#define TPLLCO_CCCTHRESHOLDMAXDEX_MET 0.2f
#define TPLLCO_CCCVALIDMAXPERC_NU 2.f
#define TPLLCO_CCCMAXCURVATURE_1PM 0.0002f
#define TPLLCO_CCCENABLECURVE_NU 0u
#define TPLLCO_CCCMINVALIDLENGTH_MET 0.f
#define TPLLCO_CCCALLOWOVERWRITE_NU 0u
#define TPLLCO_USESENSORTSTAMP_NU 0u
#define TPLLCO_SUPRESSDOUBLETRIGRPL_NU 0u
#define TPLLCO_ODOHISTLENGTH_NU 25u
#define TPLLCO_TIMEDIFFSWITCH_NU 1u
#define TPLLCO_TIMEDIFF_SEC 0.f
#define TPLLCO_ODOMAXTIME_US 0xFFFFFFFF
#define TPLLCO_PREVIEWDISTXENABLE_NU 1u
#define TPLLCO_VEHVELX2PREVIEWDISTX_TABLENUM_NU 13u  //
#define TPLLCO_VEHVELX2PREVIEWDISTX_TABLEX_MPS                                \
    {                                                                         \
        0.f, 5.f, 10.f, 15.f, 20.f, 25.f, 30.f, 35.f, 40.f, 45.f, 50.f, 55.f, \
            60.f                                                              \
    }
#define TPLLCO_VEHVELX2PREVIEWDISTX_TABLEY_MET \
    { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f }

#define TPLLCO_VEHVELX2PREVIEWDISTXOF_TABLEY_MET                            \
    {                                                                       \
        0.f, 3.5f, 4.f, 8.f, 8.f, 10.5f, 12.0f, 13.5f, 13.5f, 13.5f, 13.5f, \
            13.5f, 13.5f                                                    \
    }

#define TPLLCO_NUMITER_NU 10u
#define TPLLCO_CALCULATIONBYRAMPOUT_NU TRUE
#define TPLLCO_CALCULATIONBYREQUEST_NU TRUE
#define TPLLCO_COORDTRAFOMINLENGTH_MET 5.0f
#define TPLLCO_REPLANDEVENABLE_NU FALSE
#define TPLLCO_USEODOREPLAN_NU TRUE
#define TPLLCO_DISTYCHECKTOLERANCE_MET 0.1f
#define TPLLCO_CRVCHECKTOLERANCE_MET 0.5f
#define TPLLCO_TGTCRIDRHEADTOLCRV_TABLENUM_NU 6u  //
#define TPLLCO_CORRIDORCURVATURE_TABLEX_1PM \
    { 0.f, 0.0005f, 0.001f, 0.002f, 0.004f, 0.008f }
#define TPLLCO_TGTCRIDRHEADTOLCRV_TABLEY_RAD \
    { 0.785f, 0.785f, 0.785f, 0.785f, 0.785f, 0.785f }
#define TPLLCO_TGTCRIDRHEADTOLVELX_TABLENUM_NU 13u  //
#define TPLLCO_VEHVELX_TABLEX_MPS                                             \
    {                                                                         \
        0.f, 5.f, 10.f, 15.f, 20.f, 25.f, 30.f, 35.f, 40.f, 45.f, 50.f, 55.f, \
            60.f                                                              \
    }
#define TPLLCO_TGTCRIDRHEADTOLVELX_TABLEY_RAD                           \
    {                                                                   \
        0.785f, 0.785f, 0.785f, 0.785f, 0.785f, 0.785f, 0.785f, 0.785f, \
            0.785f, 0.785f, 0.785f, 0.785f, 0.785f                      \
    }
#define TPLLCO_ALLOWEDDEVPOSY_TABLENUM_NU 4u  //
#define TPLLCO_TESTPOINTPOSX_TABLEX_MET \
    { 0.f, 30.f, 60.f, 90.f }
#define TPLLCO_ALLOWEDDEVPOSY_TABLEY_MET \
    { 0.05f, 0.05f, 0.05f, 0.05f }

#define TPLCEN_VMINTIMEBASEDTRAJ_KPH 5.F
#define TPLCEN_TRAJPLANENABLE_NU TRUE
#define TPLCEN_LCOREPLANENABLE_NU TRUE
#define TPLCEN_ACTIVEBYRAMPOUT_NU FALSE
#define TPLCEN_ALLOWSPEPLANSTRATEGY_NU FALSE
#define TPLCEN_ENABLEFCNCHNGDETECT_NU TRUE
#define TPLCEN_REINIWRTDMCREQ_NU FALSE
#define TPLCEN_ENABLE_JUMPDETECTOBF_NU FALSE
#define TPLCEN_ENABLEJUMPDETECTOFRQ_NU TRUE
#define TPLCEN_REINIDELTADISTY_MET 0.1f
#define TPLCEN_REINIDELTAHEAD_RAD 0.01f
#define TPLCEN_REPLANBYTJA2X_NU FALSE
#define TPLCEN_USEJUMPDETECTION_NU FALSE
#define TPLCEN_ALLOWREPLANOBF_NU FALSE
#define TPLCEN_MANUALTRQMAXTIME_SEC 0.06f
#define TPLCEN_MANUALTRQMAX_NM 0.5f
#define TPLCEN_MANUALTRQMINTIME_SEC 1.f
#define TPLCEN_MANUALTRQMIN_NM 1.5f
#define TPLCEN_ERRDISTY_MET 0.5f
#define TPLCEN_ERRHEADAGLPREV_RAD 0.1f
#define TPLCEN_REPLANLARGEERRORMODE_NU 0u
#define TPLCEN_DEADTIMEVEHCTRL_TABLENUM_NU 7u
#define TPLCEN_POTVECDYNVELX_TABLEX_MPS \
    { -13.f, 0.f, 13.f, 28.f, 42.f, 55.f, 70.f }
#define TPLCEN_DEADTIMEVEHCTRL_TABLEY_SEC \
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
#define TPLCEN_DTGRADLIMITDURATION_SEC 0.5f
#define TPLCEN_MAXPREDICTIONTIME_SEC 1.f
#define TPLCEN_PTCRVGRADLIMITDUR_SEC 0.5f
#define TPLCEN_PTHEADGRADLIMITDUR_SEC 0.5f
//#define TPLCEN_CYCTIMETRAJCALC_SEC 0.18f
#define TPLCEN_CYCENABLE_NU FALSE

#define TPLFRT_USEDMCSUMCRV_NU FALSE
#define TPLFRT_CRIDRSWITCHDETECTION_NU FALSE
#define TPLFRT_CALCLEFTCRIDRENABLE_NU FALSE
#define TPLFRT_USETGTRAJX0_NU TRUE
#define TPLFRT_TGTPOINTMIN_SEC 0.2f

#define TPLTJC_ALLOWEDTIMETOCROSS_SEC 3.f
#define TPLTJC_PARAMCOSTFCTINTERNAL_NU FALSE
#define TPLTJC_WEIGHTTGTDISTY_NU 0.f
#define TPLTJC_WGHTTGTDISTYCUSTFCT_TABLENUM_NU 2u
#define TPLTJC_WGHTTGTDISTYCUSTFCT_TABLEX_NU \
    { 0.f, 1.f }
#define TPLTJC_WGHTTGTDISTYCHARACT_TABLEY_NU \
    { 0.f, 45.f }
#define TPLTJC_WEIGHTENDTIME_TABLENUM_NU 5u
#define TPLTJC_PLANNINGHORIZON_TABLEX_SEC \
    { 2.f, 3.f, 4.f, 5.f, 6.f }
#define TPLTJC_WEIGHTENDTIME_TABLEY_NU \
    { 20.f, 2.f, 0.4f, 0.2f, 0.04f }
#define TPLTJC_WGHTTGTTIMECHARACT_TABLENUM_NU 2u
#define TPLTJC_WGHTTGTTIMECUSTFCT_TABLEX_NU \
    { 0.f, 1.f }
#define TPLTJC_WGHTTGTTIMECHARACT_TABLEY_NU \
    { 0.f, 20.f }
#define TPLTJC_MAXACLY_TABLENUM_NU 6u
#define TPLTJC_VEHVELX_TABLEX_MPS \
    { 0.f, 5.f, 10.f, 20.f, 40.f, 60.f }
#define TPLTJC_MAXACLY_TABLEY_MPS2 \
    { 5.f, 5.f, 5.f, 5.f, 5.f, 5.f }
#define TPLTJC_FTIREACLMINSCALING_NU 0.f
#define TPLTJC_NUMCIRCLES_NU 4u
#define TPLTJC_WEIGHTDISTBASED_NU 0.00000099f
#define TPLTJC_MAXARCLENGTH_MET 70.f
#define TPLTJC_TMAX_SEC 6.f
#define TPLTJC_TIMETRAJENDENABLE_NU TRUE

#define TPLFBT_TRAJPLANENBL_NU FALSE
#define TPLFBT_CRVPREDICTIONENBL_NU TRUE
#define TPLFBT_CRVPT1FILTERENBL_NU FALSE
#define TPLFBT_CRVPT1TIMECONST_SEC 0.1f
#define TPLFBT_TGTCRVSWITCH_NU 2u
#define TPLFBT_USETGTCRIDRHEADING_NU FALSE
#define TPLFBT_ENABLEOBFMODE_NU FALSE
#define TPLFBT_ENABLEDIRECTSWITCH_NU TRUE
#define TPLFBT_DELTATARGETCRV_1PM 0.f
#define TPLFBT_DELTATARGETPOSY0_MET 0.f
#define TPLFBT_DELTATARGETHEADING_RAD 0.f
#define TPLFBT_DURATIONGRADLIMIT_SEC 4.f
#define TPLFBT_GRADLIMITCRVFACTOR_TABLENUM_NU 13u
#define TPLFBT_VEHVELX_MPS_TABLEX_MPS                                         \
    {                                                                         \
        0.f, 5.f, 10.f, 15.f, 20.f, 25.f, 30.f, 35.f, 40.f, 45.f, 50.f, 55.f, \
            60.f                                                              \
    }
#define TPLFBT_GRADLIMITCRVFACTOR_TABLEY_NU \
    { 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f }
#define TPLFBT_GRADLIMITCRV_1PM2 0.0001f
#define TPLFBT_GRADLIMITPOSY_MPS 20.f
#define TPLFBT_GRADLIMITPOSYHIGH_MPS 200.f
#define TPLFBT_GRADLIMITHEAD_RPS 0.01f
#define TPLFBT_GRADLIMITHEADHIGH_RPS 100.f
#define TPLFBT_MANIPTRAJGUIQU_NU TRUE
#define TPLFBT_USELONGQUSTATUS_NU FALSE
#define TPLFBT_HOLDALLBITS_NU TRUE
#define TPLFBT_MAXQUSTATUSHOLD_SEC 0.6f
#define TPLFBT_MINQUSTATUSHOLD_SEC 0.06f
#define TPLFBT_QUSTATUSHOLD_NU TRUE
#define TPLFBT_CHECKTRAJPLANSTATUS_NU 0x5E62
#define TPLFBT_QUSTATUSTRAJPLAN_NU 0x0000

enum {
    E_LCF_SYSSTATE_NOTPRESENT = 0,
    E_LCF_SYSSTATE_PASSIVE = 1,
    E_LCF_SYSSTATE_STANDBY = 2,
    E_LCF_SYSSTATE_CONTROLLING = 3,
    E_LCF_SYSSTATE_SUSPENDED = 4,
    E_LCF_SYSSTATE_RAMPOUT = 5,
    E_LCF_SYSSTATE_ERROR = 6,
    E_LCF_SYSSTATE_REQUEST = 7
};

#define P_TJATVG_TrajPlanValServQu_nu 0u      // 0x00
#define P_TJATVG_TrajPlanValSrvQuSALC_nu 16u  // 0x10,

enum {
    E_TJASTM_LATCTRLMD_PASSIVE = 0,
    E_TJASTM_LATCTRLMD_LC = 1,
    E_TJASTM_LATCTRLMD_OF = 2,
    E_TJASTM_LATCTRLMD_CMB = 3,
    E_TJASTM_LATCTRLMD_SALC = 4,
    E_TJASTM_LATCTRLMD_LC_RQ = 5,
    E_TJASTM_LATCTRLMD_OF_RQ = 6,
    E_TJASTM_LATCTRLMD_CMB_RQ = 7,
    E_TJASTM_LATCTRLMD_SALC_RQ = 8
};

enum {
    E_LCF_OFF_nu = 0,
    E_LCF_TJA_nu = 1,
    E_LCF_LDP_nu = 2,
    E_LCF_LDPOC_nu = 3,
    E_LCF_RDP_nu = 4,
    E_LCF_ALCA_nu = 5,
    E_LCF_AOLC_nu = 6,
    E_LCF_ESA_nu = 7
};

enum {
    E_LCF_TGQ_REQ_OFF = 0,
    E_LCF_TGQ_REQ = 1,
    E_LCF_TGQ_REQ_FREEZE = 3,
    E_LCF_TGQ_REQ_FFC = 4,
    E_LCF_TGQ_REQ_REFCHNG = 5,   // reference function change
    E_LCF_TGQ_REQ_CHNGFAIL = 6,  // reference function change
    E_LCF_TGQ_REQ_LANECHANGED = 7
};

#define TRAJECTORY_PLAN_VERSION_NUM 20210105u;
#define LEFT_DISTY_ARRAY_SIZE 100u    //
#define TARGET_POINTS_ARRAY_SIZE 15u  //
#define TRAJ_PARAM_ARRAY_SIZE 6u      //

#define TPLTJC_VEHVELXKT_TABLENUM_NU 8u
#define TPLTJC_VEHVELXKT_TABLEX_MPS \
    { 0.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f }
#define TPLTJC_KT_TABLEY_MPS2 \
    { 0.000099f, 0.000999f, 0.000999f, 0.009999f, 0.1f, 0.5f, 0.8f, 1.f }

#define TPLTJC_DELTATVEL_TABLENUM_NU 8u
#define TPLTJC_VEHVELXDELTAT_TABLEX_MPS \
    { 0.f, 1.f, 1.5f, 2.f, 2.5f, 3.f, 4.f, 5.f }
#define TPLTJC_DELTATVEL_TABLEY_MPS2 \
    { 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.3f, 0.1f, 0.1f }

#define TPLFBT_LANCHGPLAUSICHECK_MET 1.f
/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

#ifdef __cplusplus
}
#endif
#endif
