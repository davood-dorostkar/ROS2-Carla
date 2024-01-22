
#ifndef _OPS_H_INCLUDED
#define _OPS_H_INCLUDED

/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/
#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "TM_Global_Types.h"

#include "ops_ext.h"
#include "ops_cfg.h"              //guotao added 20190418
#include "ops_eba_quality_par.h"  //guotao added 20190418
#include "ops_eba_quality_cfct.h"
#include "ops_obj_prioritization.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
typedef sint8 EM_t_ObjNumber;

typedef struct {
    ubit8_t bWasOncoming : 1;   /*!< The object was oncoming in it lifetime */
    ubit8_t bWasCrossing : 1;   /*!< The object was crossing in it lifetime */
    ubit8_t bWasMoving : 1;     /*!< The object was moving in it lifetime */
    ubit8_t bWasStationary : 1; /*!< The object was stationary in it lifetime */
    ubit8_t bWasStopped : 1;    /*!< The object was stopped in it lifetime */
    ubit8_t
        bWasMovingSafe : 1; /*!< The object was secure moving in it lifetime */
} OPS_DynPropHistory_t;

typedef struct {
    ui8_t sCustomData;
    f32_t fEBAObjectQuality; /*!< The output EBA object quality */
    OPS_DynPropHistory_t
        sDynPropHistory; /*!< History of the object dynamic properties */
    ui8_t iObjSafe;      /*!< 8 Bits for Object Quality */

} FctPreselEBAObj_t;

/*****************************************************************************
  MACROS
*****************************************************************************/

#define FPS_STATIONARY_OBJ_MAX_LAT_DISP (1.0f)
#define FPS_STATIONARY_OBJ_MAX_LONG_DISP (2.5f)

#define FPS_MOVING_OBJ_UPDATE_QUALITY_THRES (0.9f)
#define FPS_PRESEL_OBJ_LIFETIME_THRES (65535u)
#define FPS_PRESEL_OBJ_FREQ_VAL_DEFAULT_THRES (255u)
#define FPS_PRESEL_OBJ_DENSITY_DEFAULT_THRES (36L)

#define FPS_ACC_INT_OBJ_MAX_VRELX (0.3f)
#define FPS_ACC_INT_OBJ_MAX_DISTX (1.0f)

#define FPS_ACC_HIGHEST_CLUST_VAR_THRES (14u)
#define FPS_ACC_HIGH_CLUST_VAR_THRES (10u)
#define FPS_ACC_LOW_CLUST_VAR_THRES (7u)

#if (!defined(CFG_Envm_USE_NEW_FPS_QUALTIY) || \
     (defined(CFG_Envm_USE_NEW_FPS_QUALTIY) && \
      CFG_Envm_USE_NEW_FPS_QUALTIY == CFG_Envm_SWITCH_OFF))
/* Last object distance with target */
#define FPS_EBA_LAST_OBJ_DIST_DEFAULT (200.0f)
#endif

#if ((CFG_Envm_CAMOBJ_FUSION_LEVEL >= CFG_EM_FUSION_LEVEL_2) && \
     (FPS_CFG_CAMERA_CONF_HOLD_WORKAROUND))
/*! Default value for number of cycles without camera object confirmation after
 * last camera object confirmation */
#define FPS_ACC_DEFAULT_NUM_CYCLE_NO_CAM_CONF (255u)
#endif

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*! Struct for ACC function preselection state per object */
typedef struct FctPreselAccStateTag {
    struct

    {
        /* Bits filled in by function pre-selection */
        ubit8_t FctPreselPOBS : 1; /*!< Obstacle probability check value */
        ubit8_t FctPreselLV : 1;  /*!< Long vehicle check (used to supress truck
                                     cabins) */
        ubit8_t FctPreselPED : 1; /*!< Pedestrian preselection */
        ubit8_t FctPreselCam : 1; /*!< Camera object fusion confirmation bit */
#if ((CFG_Envm_CAMOBJ_FUSION_LEVEL >= CFG_EM_FUSION_LEVEL_2) ||    \
     ((CFG_Envm_CAMOBJ_FUSION_LEVEL == CFG_Envm_FUSION_LEVEL_1) && \
      (FPS_CFG_ACC_STAT_OBJ_FUSION_LEVEL_1 == SWITCH_ON)))
#if (FPS_CFG_CAMERA_CONF_HOLD_WORKAROUND)
        ubit8_t FctPreselOnceCam : 1; /*!< Was camera confirmed at least once
                                         (sticky bit) */
#endif
#if (FPS_CFG_USE_CAM_FOR_ONCOME_STOPPED)
        ubit8_t FctSeenMoving : 1; /*!< Was seen moving */
#endif
#endif
        /* Bits filled in by sensor pre-selection */
        ubit8_t SensorPreselPOE : 1; /*!< Probability of existance criteria
                                        fulfilled */
        ubit8_t
            SensorPreselRCS : 1; /*!< Radar cross section criteria fulfilled */
        ubit8_t SensorPreselMTF : 1; /*!< Measured target frequency criteria
                                        fulfilled */
        ubit8_t
            SensorPreselLATV : 1;    /*!< Lateral variance criteria fulfilled */
        ubit8_t SensorPreselMTD : 1; /*!< Measured target density criteria
                                        fulfilled */
        ubit8_t SensorPreselLTV : 1; /*!< Life-time criteria fulfilled */
        ubit8_t
            SensorPreselNoVrelAmbigObj : 1; /*!< Obj not VrelAmbig suspicious */
        ubit8_t SensorPreselNoMirrorGhost : 1; /*!< Object is no mirror ghost */
        /* Summary bits */
        ubit8_t FctPreselSum : 1;    /*!< Function preselection summary bit */
        ubit8_t SensorPreselSum : 1; /*!< Sensor preselection summary bit */
        ubit8_t : 0; /*!< Reserved, only needed when structure not size of
                        n*8bit  */
    } Bool;

#if (CFG_Envm_DTR_OBJECT_QUALIFIER)
    struct {
        /* Bits filled in by FPSLongFuncQualProcess */
        ubit8_t LongFuncQualPOE : 1; /*!< Probability of existance criteria
                                        fulfilled */
        ubit8_t
            LongFuncQualRCS : 1; /*!< Radar cross section criteria fulfilled */
        ubit8_t LongFuncQualMTF : 1; /*!< Measured target frequency criteria
                                        fulfilled */
        ubit8_t LongFuncQualUQV : 1; /*!< Update quality criteria fulfilled */
        ubit8_t LongFuncQualMTD : 1; /*!< Measured target density criteria
                                        fulfilled */
        ubit8_t LongFuncQualLTV : 1; /*!< Life-time criteria fulfilled */
        ubit8_t LongFuncNoVrelAmbigObj : 1; /*!< Obj not VrelAmbig suspicious */
        ubit8_t LongFuncQualSum : 1;        /*!< LongFuncQual summary bit */
        /* ubit8_t                : 0;*/    /*!< Reserved, only needed when
                                               structure not size of n*8bit  */
    } LongFuncQual;

    struct {
        /* Bits filled in by FPSLatFuncQualProcess */
        ubit8_t LtrlFuncQualNST : 1; /*!< NonStationary criteria fulfilled */
        ubit8_t LtrlFuncQualOCC : 1; /*!< Occlusion criteria fulfilled */
        ubit8_t
            LtrlFuncQualGRD : 1; /*!< Guard Rail Distance criteria fulfilled */
        ubit8_t LtrlFuncQualFOV : 1;    /*!< FOV criteria fulfilled */
        ubit8_t LtrlFuncQualMTF : 1;    /*!< MTF criteria fulfilled */
        ubit8_t LtrlFuncQualImpres : 1; /*!< Object is in no situation with
                                           possible impaired resolution */
        ubit8_t LtrlFuncQualFuncSafetyFOV : 1; /*!< Functional safety FOV
                                                  criteria fulfilled */
        ubit8_t LtrlFuncQualSum : 1;           /*!< LatFuncQual summary bit */
        /* ubit8_t                  : 0; */    /*!< Reserved, only needed when
                                                  structure not size of n*8bit  */
    } LtrlFuncQual;

    struct {
        /* Bits filled in by FPSLongFusionQualProcess */
        ubit8_t LongFusionQualPOE : 1; /*!< Probability of existance criteria
                                          fulfilled */
        ubit8_t LongFusionQualRCS : 1; /*!< Radar cross section criteria
                                          fulfilled */
        ubit8_t LongFusionQualMTF : 1; /*!< Measured target frequency criteria
                                          fulfilled */
        ubit8_t LongFusionQualUQV : 1; /*!< Update quality criteria fulfilled */
        ubit8_t LongFusionQualMTD : 1; /*!< Measured target density criteria
                                          fulfilled */
        ubit8_t LongFusionQualLTV : 1; /*!< Life-time criteria fulfilled */
        ubit8_t
            LongFusionNoVrelAmbigObj : 1;   /*!< Obj not VrelAmbig suspicious */
        ubit8_t LongFusionQualSum : 1;      /*!< LongFuncQual summary bit */
        /* ubit8_t                  : 0; */ /*!< Reserved, only needed when
                                               structure not size of n*8bit  */
    } LongFusionQual;

    struct {
        /* Bits filled in by FPSCrossFusionQualProcess */
        ubit8_t CrossFusionQualPOE : 1; /*!< Probability of existance criteria
                                           fulfilled */
        ubit8_t CrossFusionQualRCS : 1; /*!< Radar cross section criteria
                                           fulfilled */
        ubit8_t CrossFusionQualMTF : 1; /*!< Measured target frequency criteria
                                           fulfilled */
        ubit8_t
            CrossFusionQualUQV : 1; /*!< Update quality criteria fulfilled */
        ubit8_t CrossFusionQualLTV : 1; /*!< Life-time criteria fulfilled */
        ubit8_t CrossFusionQualNoVrelAmbigObj : 1; /*!< Obj not VrelAmbig
                                                      suspicious */
        ubit8_t CrossFusionQualSum : 1; /*!< LongFuncQual summary bit */
        ubit8_t : 0; /*!< Reserved, only needed when structure not size of
                        n*8bit  */
    } CrossFusionQual;

    struct {
        /* Bits filled in by FPSPedestFusionQualProcess */
        ubit8_t PedestFusionQualPOE : 1; /*!< Probability of existance criteria
                                            fulfilled */
        ubit8_t PedestFusionQualRCS : 1; /*!< Radar cross section criteria
                                            fulfilled */
        ubit8_t PedestFusionQualMTF : 1; /*!< Measured target frequency criteria
                                            fulfilled */
        ubit8_t
            PedestFusionQualUQV : 1; /*!< Update quality criteria fulfilled */
        ubit8_t PedestFusionQualLTV : 1; /*!< Life-time criteria fulfilled */
        ubit8_t PedestFusionQualNoVrelAmbigObj : 1; /*!< Obj not VrelAmbig
                                                       suspicious */
        ubit8_t PedestFusionQualFuncSafetyFOV : 1;  /*!< functional safety FOV
                                                       criteria fulfilled */
        ubit8_t PedestFusionQualSum : 1; /*!< LongFuncQual summary bit */
        /* ubit8_t                       : 0; */ /*!< Reserved, only needed when
                                                    structure not size of n*8bit
                                                  */
    } PedestFusionQual;
#endif

    float fRelevantTime;          /*!< Time given object was relevant */
    uint8 uiCyclesBelowRCSThresh; /*!< Counter for number of cycles object is
                                     below RCS threshold */

#if ((CFG_Envm_CAMOBJ_FUSION_LEVEL >= CFG_EM_FUSION_LEVEL_2) ||    \
     ((CFG_Envm_CAMOBJ_FUSION_LEVEL == CFG_Envm_FUSION_LEVEL_1) && \
      (FPS_CFG_ACC_STAT_OBJ_FUSION_LEVEL_1 == SWITCH_ON)))
#if (FPS_CFG_CAMERA_CONF_HOLD_WORKAROUND)
    ui8_t CamConfObjID; /*!< The camera object ID that last confirmed this
                           object */
    ui8_t u_NumCycleNoCamConf; /*!< Number of cycles without camera object
                                  confirmation after last camera object
                                  confirmation */
    ui8_t u_NumCyclesCamConf;  /*!< Number of cycles with camera object
                                  confirmation */
#endif
#endif

} FctPreselAccState_t;

#if 1
// liuyang 2018-11-28 remove
//(!defined(CFG_Envm_USE_NEW_FPS_QUALTIY) ||
//(defined(CFG_Envm_USE_NEW_FPS_QUALTIY) && CFG_Envm_USE_NEW_FPS_QUALTIY ==
// CFG_Envm_SWITCH_OFF) )
#endif

/*! Struct for FPS EBA conditional probability settings */
#define FPS_EBA_NUMBER_OF_COND_TABLE_ENTRIES (4)
typedef struct FctPreselEBACondProbTable2Tag {
    uint8
        CPT[FPS_EBA_NUMBER_OF_COND_TABLE_ENTRIES]; /*!< Percentages for cases
                                                      (F|F,  T|F,  F|T,  T|T) */
} FctPreselEBACondProbTable2_t;

/*! Function pointer to FPS EBA Observer Update function */
typedef uint8 (*FctPreselEBAObserverUpd_fpt)(sint8 ObjNr);

/*! Length of observer name */
#define FPS_EBA_OBSNAME_LENGTH (11u)
/*! Struct to define an FPS EBA Observer */
typedef struct FctPreselEBAObserverTag {
    const ui8_t szObsName[FPS_EBA_OBSNAME_LENGTH + 1u]; /*!< Observer name */
    const ui8_t uiObserverID; /*!< Number of the observer */
    const ui8_t uiGroupID;    /*!< Group of the observer */
    const FctPreselEBACondProbTable2_t
        CPT; /*!< Pointer to the conditional probability settings */
    FctPreselEBAObserverUpd_fpt
        fptUpdate; /*!< Pointer to the actual observer update function */
} FctPreselEBAObserver_t;

/*! Struct to define the FPS sensor criteria thresholds for PoE */
typedef struct {
    f32_t MovingPickUp;
    f32_t MovingDrop;
    f32_t StationaryPickUp;
    f32_t StationaryDrop;
    f32_t DefaultPickUp;
    f32_t DefaultDrop;

} FPS_POE_Thresholds_t;

/*! Struct to define the FPS sensor criteria thresholds for MTF */
typedef struct {
    ui32_t MovingPickUp;
    ui32_t MovingDrop;
    ui32_t StoppedPickUp;
    ui32_t StoppedDrop;
    ui32_t StationaryPickUp;
    ui32_t StationaryDrop;
    ui32_t DefaultPickUp;
    ui32_t DefaultDrop;

} FPS_MTF_Thresholds_t;

/*! Struct to define the FPS sensor criteria thresholds for MTD */
typedef struct {
    i32_t MovingPickUp;
    i32_t MovingDrop;
    i32_t StationaryPickUp;
    i32_t StationaryDrop;
    i32_t DefaultPickUp;
    i32_t DefaultDrop;

} FPS_MTD_Thresholds_t;

/*! Struct to define the FPS sensor criteria thresholds for Lifetime */
typedef struct {
    ui16_t MovingPickUp;
    ui16_t MovingDrop;
    ui16_t StationaryPickUp;
    ui16_t StationaryDrop;
    ui16_t OncomingPickUp;
    ui16_t OncomingDrop;
    ui16_t DefaultPickUp;
    ui16_t DefaultDrop;

} FPS_Lifetime_Thresholds_t;

/*****************************************************************************
  VARIABLES
*****************************************************************************/
/*! The function preselection states for all objects */
extern FctPreselAccState_t FctPreselStates[40];

#if (!defined(CFG_Envm_USE_NEW_FPS_QUALTIY) || \
     (defined(CFG_Envm_USE_NEW_FPS_QUALTIY) && \
      CFG_Envm_USE_NEW_FPS_QUALTIY == CFG_Envm_SWITCH_OFF))
extern FctPreselEBAObj_t FctPreselEBAObj[Envm_NR_PRIVOBJECTS];
#endif

/*****************************************************************************
  FUNCTION
*****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************/
/* ACC Functions                                                        */
/************************************************************************/
// liuyang 2018-11-28 remove
extern void FPSACCProcess(void);
extern void OPSInitObjectPrioritization(void);
extern void FPSInitACC(void);
extern void FPSInitCustom(void);

#if (defined(CFG_Envm_USE_NEW_FPS_QUALTIY) && \
     CFG_Envm_USE_NEW_FPS_QUALTIY == CFG_Envm_SWITCH_ON)
extern void FPSEBA_CustomFctInit(void);
#endif
extern void FPSInitACCObject(sint8 ObjNr);

extern uint8 FPSACCSensorPresel(const sint8 s_ObjNr);
extern uint8 FPSACCFunctionPresel(const sint8 ObjNr);

extern void FPSACCFuncPreSelPreProc(void);
extern void FPSACCSenPreSelPreProc(void);

#if (!defined(CFG_Envm_USE_NEW_FPS_QUALTIY) || \
     (defined(CFG_Envm_USE_NEW_FPS_QUALTIY) && \
      CFG_Envm_USE_NEW_FPS_QUALTIY == CFG_Envm_SWITCH_OFF))
extern void OPSEBAProcess(void);
extern void OPSAEBInit(void);
extern void OPSInitEBAObject(sint8 ObjNr);
// changchang 20200605 remove form ops
// extern void FPSEBAPreselInit        (void);
extern void OPSEBAPreselPreProc(void);
extern void OPSEBAPreselPostProc(void);
extern void OPSEBAPresel(void);
#endif
extern void OPSEBAPreSelCustomFct(sint8 ObjNr);

extern bool_t FPSSenCritCheckPOE(const sint8 ObjNr, const f32_t threshold);
extern bool_t FPSSenCritCheckMTF(const sint8 ObjNr, const ui32_t threshold);
extern bool_t FPSSenCritCheckRCS(const sint8 ObjNr,
                                 const f32_t threshold,
                                 const bool_t HysteresisActive);
extern bool_t FPSSenCritCheckMTD(const sint8 ObjNr, const i32_t threshold);
extern bool_t FPSSenCritCheckLifetime(const sint8 ObjNr,
                                      const ui16_t threshold);
#if (FPS_CFG_SUPPRESS_GHOST_TARGETS)
extern bool_t FPSCheckGhostTarget(const sint8 s_ObjNr);
#endif

#if (FPS_CFG_SUPPRESS_MIRROR_GHOST_TARGETS)
extern bool_t FPSCheckMirrorGhostCriteria(const sint8 s_ObjNr);
#endif

#if ((CFG_Envm_CAMOBJ_FUSION_LEVEL >= CFG_EM_FUSION_LEVEL_2) ||    \
     ((CFG_Envm_CAMOBJ_FUSION_LEVEL == CFG_Envm_FUSION_LEVEL_1) && \
      (FPS_CFG_ACC_STAT_OBJ_FUSION_LEVEL_1 == SWITCH_ON)))
extern bool_t FPSCheckCamConfirmation(const sint8 ObjNr);
#endif

#if (FPS_CFG_SUPPRESS_MOV_ON_BRIDGE)
extern bool_t FPSSuppressMovOnBridge(const sint8 ObjNr);
#endif
#if (FPS_CFG_SUPPRESS_VRELAMBIG_ON_GUARDRAIL)
extern bool_t FPSSenCritSuppressVrelAmbiguityOnGuardrail(const sint8 ObjNr);
#endif

extern f32_t FPSACCSetPOEThresh(const sint8 ObjNr,
                                const bool_t HysteresisActive,
                                const FPS_POE_Thresholds_t *Thresholds);
extern ui32_t FPSACCSetMTFThresh(const sint8 ObjNr,
                                 const bool_t HysteresisActive,
                                 const FPS_MTF_Thresholds_t *Thresholds);

extern f32_t FPSSenCritSetPOEThresh(const sint8 ObjNr,
                                    const bool_t HysteresisActive,
                                    const FPS_POE_Thresholds_t *Thresholds);
extern ui32_t FPSSenCritSetMTFThresh(const sint8 ObjNr,
                                     const bool_t HysteresisActive,
                                     const FPS_MTF_Thresholds_t *Thresholds);
extern f32_t FPSSenCritSetRCSThresh(const sint8 ObjNr,
                                    const bool_t HysteresisActive);
extern i32_t FPSSenCritSetMTDThresh(const sint8 ObjNr,
                                    const bool_t HysteresisActive,
                                    const FPS_MTD_Thresholds_t *Thresholds);
extern ui16_t FPSSenCritSetLifetimeThresh(
    const sint8 ObjNr,
    const bool_t HysteresisActive,
    const FPS_Lifetime_Thresholds_t *Thresholds);

#if (CFG_Envm_GENERIC_OBJECT_INTERFACE_ACTIVE ==                          \
     1) /* Object prioritization necessary only if generic object list is \
           active */
// changchang 20200605 remove
// extern void    FPS_v_SetPrioIndexListForObjOutput(void);
/* Initialization of static variables: custom data and non custom data */
// extern void    FPS_v_InitObjPrioStaticCustomData (void);
#if (FPS_CFG_QUOTA_PRIO_VERSION)
// extern boolean FPS_b_ObjPrioByCustom             (uint32 const ui_Obj);
// extern boolean FPS_b_PrioObjRejectCustom         (uint32 const ui_Obj, const
// float32 f_DistToTrajAbs);
#else
// extern void FPS_v_ObjPrioByCustom(boolean * pb_PrioByCustom, boolean *
// pb_AllReqByCustomFulfilled, uint32 const ui_Index, ObjectList_t const *
// p_EnvmPublicObjectList);
#endif
#endif

#if (defined(CFG_Envm_USE_NEW_FPS_QUALTIY) && \
     CFG_Envm_USE_NEW_FPS_QUALTIY == CFG_Envm_SWITCH_ON)
/************************************************************************/
/* EBA Functions                                                        */
/************************************************************************/

extern void OPSEBAProcess(void);
extern void OPSAEBInit(void);
extern void OPSInitEBAObject(sint8 ObjNr);
// remove FPSEBA_ObjQualProcess from ops by cc 20200605
// extern void FPSEBA_ObjQualProcess     (const sint8 ObjNr, boolean
// b_FuncSafetyFOV);

#endif

#if defined(FPS_CFG_OBJECT_MOSA_CHECK) && (FPS_CFG_OBJECT_MOSA_CHECK == 1)
/************************************************************************/
/* Moving safe defines                                                  */
/************************************************************************/

// typedef sint8 EM_t_ObjNumber;

extern uint8 OPSEBA_MOSA_GetObjPermission(EM_t_ObjNumber iObj);
extern void OPSEBA_MOSA_PreProcess(void);
extern void OPSEBA_MOSA_Process(EM_t_ObjNumber iObj);
// changchang 20200605 remove FPSEBA_MOSA_PostProcess from ops
// extern void  FPSEBA_MOSA_PostProcess     (void);
extern void OPSEBA_MOSA_InitObj(EM_t_ObjNumber iObj);
extern void OPSMOSAInit(void);

#endif

// changchang 20200605 remove EMSORT_v_BubbleSort from ops
// void EMSORT_v_BubbleSort(uint32 u_ElemNr, const float32 a_compArray[], uint32
// a_sortedList[]);//added this function from em module by guotao 20190418
#ifdef __cplusplus
};
#endif

#endif /* FPS_H_INCLUDED */
/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
