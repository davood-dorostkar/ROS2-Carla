

/* Bedingte Einbindung */
#ifndef _VLC_SEN_H_INCLUDED
#define _VLC_SEN_H_INCLUDED

/* Additional check to assure that the VLC pars running in two different task
contexts do not try to access the same data - possibly corrupting each other's
states, leading to reproducibility issues in simulation */
#ifdef _VLC_VEH_H_INCLUDED
#error vlc_sen.h and vlc_veh.h shall not be included in the same context!
#endif

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#ifdef __cplusplus
/* @todo: later on all internal _ext headers shall contain proper C++ extern C
declarations, to allow directly including them in C++ sources. */
extern "C" {
#endif

#include "vlcSen_ext.h"
#include "vlc_glob_ext.h"
#include "cp_ext.h"
#include "emp_ext.h"
#include "si_ext.h"
//#include "cd_ext.h"
//#include "vlc_long_sen_ext.h"
#include "fip_ext.h"
#include "frame_sen_custom_types.h"
#include "vlc_ver.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*! @cond Doxygen_Suppress */
#define VLC_FREEZE_DATA(pInfo, pData, Callback) \
    MEASFreezeDataDLG((pInfo), (pData), (Callback))

#define GET_PRIV_OBJ_DATA VLCObjectList
#define GET_VLC_OBJ(iObj) VLCObjectList[iObj]

#define GET_Envm_VLC_CYCLE_MODE_PTR VLCSEN_pECAMtCyclEnvmode
#define GET_Envm_PUB_OBJ_DATA_PTR VLCSEN_pEmGenObjList
#define GET_EM_ARS_OBJ_LIST_PTR VLCSEN_pEmARSObjList

#define GET_EGO_OBJ_SYNC_DATA_PTR VLCSEN_pEgoDynObjSync
#define GET_EGO_RAW_DATA_PTR VLCSEN_pEgoDynRaw
#define GET_EGO_STATIC_DATA_PTR VLCSEN_pGlobEgoStatic
#define GET_ROAD_DATA_PTR VLCSEN_pRoadData
#define GET_VLCSEN_PARAMETERS pVLCSEN_Parameters

#define RSP_GET_KONTEXT_DATA_PTR VLCSEN_pRSPContextData
#define ALN_MON_INPUT_PTR VLCSEN_pAlignmentMonInput

#define GET_VLC_PUB_OBJ_DATA_PTR VLCSEN_pPubFctObjList
#define GET_SPM_DATA_PTR VLCSEN_pSysPerfMonStates

#define GET_RSP_PD_OUTPUT_DATA_PTR VLCSEN_pRSPOutputPD

#define OBJ_GET_CP(iObj) GET_VLC_OBJ(iObj).CP

#define OBJ_GET_SI(iObj) GET_VLC_OBJ(iObj).SI

#define GET_VLC_AVLC_OOI_DATA_PTR VLCSEN_pAccOOIData

/* component defines */
#define CP_CYCLE_TIME TASK_CYCLE_TIME   // TPGetCycleTime()
#define SPM_CYCLE_TIME TASK_CYCLE_TIME  // TPGetCycleTime()

#define VLC_BSW_ALGO_PARAM_PTR VLCSEN_pBswAlgoParameters

#define VLC_CYCLE_TIME TASK_CYCLE_TIME  // TPGetCycleTime

#define OBJ_ABS_VELO_X(iObj) \
    VLCSEN_pCustomOutput->CustObjData[iObj].AbsolutKinematics.fAbsVelocityX
#define OBJ_ABS_ACCEL_X(iObj) \
    VLCSEN_pCustomOutput->CustObjData[iObj].AbsolutKinematics.fAbsAccelerationX

/*! Macros for accessing ACC Assessed Object List per object (based on
 * GET_VLC_OBJ_PUB(iObj)) */
#define OBJ_GET_ASSOCIATED_LANE(iObj) \
    GET_VLC_OBJ_PUB(iObj)             \
        .LaneInformation.eAssociatedLane /*!< Remark: SIGetAssociateLane */
#define OBJ_GET_FUNC_LANE(iObj) \
    GET_VLC_OBJ_PUB(iObj).LaneInformation.eFuncAssociatedLane
#define OBJ_GET_OOI_POS(iObj) GET_VLC_OBJ_PUB(iObj).ObjOfInterest.eObjOOI
#define OBJ_GET_RELEVANT(iObj)                                               \
    (OBJ_GET_OOI_POS(iObj) ==                                                \
     OBJ_NEXT_OOI) /*!< Remark: ((iObj >= 0) && (iObj == EMRelObjPrevCycle)) \
                    */
#define OBJ_GET_OBJ_TO_REF_DISTANCE(iObj)                              \
    GET_VLC_OBJ_PUB(iObj)                                              \
        .Legacy.fDistToRef /*!< Remark:                                \
                              SITrajGetObjToRefDistance((ui32_t)i_Obj, \
                              &fDistLat, &fDistLatVar) */
#define OBJ_GET_CUT_IN_POTENTIAL(iObj) \
    GET_VLC_OBJ_PUB(iObj).LaneInformation.uiCutInProbability
#define OBJ_GET_CUT_OUT_POTENTIAL(iObj) \
    GET_VLC_OBJ_PUB(iObj).LaneInformation.uiCutOutProbability
#define OBJ_GET_EXTERNAL_OBJ_ID(iObj) \
    GET_VLC_OBJ_PUB(iObj).ObjOfInterest.cExternalID

/*! Macros for accessing ACC Assessed Object List object of interest array
 * (based on GET_VLC_PUB_OBJ_DATA_PTR) */
#define OBJ_GET_OOI_LIST_OBJ_IDX(iOoiPos) \
    GET_VLC_PUB_OBJ_DATA_PTR->HeaderAssessedObjList.aiOOIList[iOoiPos]
#define OBJ_GET_RELEVANT_OBJ_NR \
    OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI) /*!< Remark: iRelObjNr */

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

typedef enum {
    VLCSEN_RTA_PROCESS,                  /*!< Whole procedure runtime */
    VLCSEN_RTA_PREPRE_PROCESS,           /*!< data preparation runtime */
    VLCSEN_RTA_PRE_PROCESS,              /*!< data preprocessing runtime */
    VLCSEN_RTA_POST_PROCESS,             /*!< Post processing runtime */
    VLCSEN_RTA_PROCESSFREEZE,            /*!< Process Measfreeze runtime */
    VLCSEN_RTA_FRAMEFREEZE,              /*!< Frame Measfreeze runtime */
    VLCSEN_RTA_CD,                       /*!< Collision Detection runtime */
    VLCSEN_RTA_CD_UPDATE_EMP,            /*!< CD EMP Update runtime */
    VLCSEN_RTA_CD_CALCULATE_OBJECT_PROP, /*!< Calculate object properties (all
                                            objects) runtime */
    VLCSEN_RTA_CD_HYPO_HANDLER, /*!< Test all hypotheses for all objects runtime
                                 */
    VLCSEN_RTA_CD_HYPO_UPDATE,  /*!< Update hypotheses runtime */
    VLCSEN_RTA_CD_SORT_HYPO,    /*!< Sort hypotheses runtime */
    VLCSEN_RTA_CD_CUSTOMER_FUNCTIONS, /*!< Customer specific functions runtime
                                       */
    VLCSEN_RTA_CD_HYP_AVLC_SINGLE, /*!< Hypothesis ACC runtime (one object) */
    VLCSEN_RTA_CD_HYP_COLLISION_SINGLE, /*!< Hypothesis Collision runtime (one
                                           object) */
    VLCSEN_RTA_CD_HYP_CROSSING_SINGLE,  /*!< Hypothesis Crossing runtime (one
                                           object) */
    VLCSEN_RTA_CD_HYP_CUTIN_SINGLE, /*!< Hypothesis Cutin runtime (one object)
                                     */
    VLCSEN_RTA_CD_HYP_PASS_SINGLE,  /*!< Hypothesis Pass runtime (one object) */
    VLCSEN_RTA_CD_HYP_PEDCOLL_SINGLE, /*!< Hypothesis PedCollision runtime (one
                                         object) */
    VLCSEN_RTA_CD_HYP_PEDPASS_SINGLE, /*!< Hypothesis PedPass runtime (one
                                         object) */
    VLCSEN_RTA_CD_HYP_BICYCLECOLL_SINGLE, /*!< Hypothesis CycleColl runtime (one
                                             object) */
    VLCSEN_RTA_CD_HYP_RUNUP_SINGLE, /*!< Hypothesis RunUp runtime (one object)
                                     */
    VLCSEN_RTA_CD_HYP_RUNUP_STAT_SINGLE, /*!< Hypothesis Runup Stationary
                                            runtime (one object) */
    VLCSEN_RTA_CD_HYP_STATIC_SINGLE, /*!< Hypothesis Static runtime (one object)
                                      */
    VLCSEN_RTA_CD_HYP_ACC,       /*!< Hypothesis ACC runtime (all objects) */
    VLCSEN_RTA_CD_HYP_COLLISION, /*!< Hypothesis Collision runtime (all objects)
                                  */
    VLCSEN_RTA_CD_HYP_CROSSING,  /*!< Hypothesis Crossing runtime (all objects)
                                  */
    VLCSEN_RTA_CD_HYP_CUTIN,     /*!< Hypothesis Cutin runtime (all objects) */
    VLCSEN_RTA_CD_HYP_PASS,      /*!< Hypothesis Pass runtime (all objects) */
    VLCSEN_RTA_CD_HYP_PEDCOLL,   /*!< Hypothesis PedCollision runtime (all
                                    objects) */
    VLCSEN_RTA_CD_HYP_PEDPASS, /*!< Hypothesis PedPass runtime (all objects) */
    VLCSEN_RTA_CD_HYP_BICYCLECOLL, /*!< Hypothesis CycleColl runtime (all
                                      objects) */
    VLCSEN_RTA_CD_HYP_RUNUP,      /*!< Hypothesis RunUp runtime (all objects) */
    VLCSEN_RTA_CD_HYP_RUNUP_STAT, /*!< Hypothesis Runup Stationary runtime (all
                                     objects) */
    VLCSEN_RTA_CD_HYP_STATIC, /*!< Hypothesis Static runtime (all objects) */
    VLCSEN_RTA_CD_CALCULATE_SINGLE_OBJECT_PROP, /*!< Calculate object properties
                                                   (single object) runtime ->
                                                   show after 12*/
    VLCSEN_RTA_CD_CALCULATE_TTC,  /*!< Calculate single object TTCs */
    VLCSEN_RTA_CD_CALCULATE_LONG, /*!< Calculate single object AnecLong and TTBs
                                   */
    VLCSEN_RTA_CD_CALCULATE_LAT,  /*!< Calculate single object AnecLat and TTSs
                                   */
    VLCSEN_RTA_CD_CALCULATE_ASSIGN, /*!< Calculate single object Track
                                       assignment and rest */
    VLCSEN_RTA_CD_CALCULATE_COMMON, /*!< Calculate single object common data */
    VLCSEN_RTA_UDW,                 /*!< UDW processing runtime */
    VLCSEN_RTA_AVLC_FIP,  /*!< Runtime for FIP (Function Input Preprocessing) */
    VLCSEN_RTA_CP,        /*!< Course Prediction runtime */
    VLCSEN_RTA_AVLC_LA,   /*!< Runtime for LA (Lane Association) */
    VLCSEN_RTA_AVLC_OOI,  /*!< Runtime for OOI (Object Of Interest selection) */
    VLCSEN_RTA_AVLC_SIT,  /*!< Runtime for SIT (Situation) */
    VLCSEN_RTA_AVLC_OUT,  /*!< Runtime for OUT (Output) */
    VLCSEN_RTA_AVLC_HC,   /*!< Runtime for HC (Headway Control) */
    VLCSEN_RTA_SPM,       /*!< System Performance Monitoring runtime */
    VLCSEN_RTA_FRAME_SEN, /*!< Runtime for AVLC_FRAME_SEN */
    VLCSEN_RTA_NOF_CHECKPOINTS /*!< This line has to be the last to determine
                                  the size of the structure */
} VLCSenRtaMapping_t;

/*! @VADDR: VLC_MEAS_ID_TRAJECTORIES @VNAME: VLCTrajectories @CYCLEID: VLC_ENV
 */
typedef struct VLCTrajectoriesMeas {
    CPTrajMeasInfo_t SI; /*!< Course information for SI */

    EMPTrajPred_t EMPTrajPredEgo;

} VLCTrajectoriesMeas_t;

/*! operating modes of sub-component */
typedef enum {
    VLC_SEN_INIT,    /*!< initialize all    */
    VLC_SEN_RUN,     /*!< normal processing */
    VLC_SEN_RG_HIGH, /*!< normal processing, but set RGLength to HIGH Resolution
                      */
    VLC_SEN_SHUTDOWN, /*!< shutdown (system in error, but recovery possible) */
    VLC_SEN_ERROR     /*!< fatal error, no recovery*/
} VLCSenState_t;

typedef struct {
    VLCSwVersion_t uiCP;
    VLCSwVersion_t uiSI;
    VLCSwVersion_t uiCD;
    VLCSwVersion_t uiFIP;
    uint32 uProjectID;
    VLCSwVersion_t FctVersionNumVar;
    VLCSwVersion_t uiVLCSEN;
} VLCSenVersions_t;

/*! The function frame @VADDR:VLC_MEAS_ID_SEN_FRAME_DATA @VNAME:VLCSenFrameData
 * @CYCLEID: VLC_ENV */
typedef struct VLCSenFrame_t {
    AlgoCycleCounter_t uiCycleCounter; /*!< The VLC_SEN cycle counter */
    boolean bFirstCycleDone;  /*!< Boolean showing if first cycle was done */
    VLC_OP_MODE_t eVLCOpMode; /*!< VLC requested operation mode */
    VLCSenState_t eVLCState;  /*!< VLC current operation mode */
    VLCSenVersions_t Versions;
} VLCSenFrame_t;

/*****************************************************************************
  TYPEDEFS (KOMPONENTENEXTERN)
*****************************************************************************/

/* ****************************************************************
    TYPEDEF
    **************************************************************** */
/*! @brief VLCObject    */
typedef struct VLCObject {
    VLCCoursePred_t CP;
    SI_t SI;
    VLCCustomObjectProperties_t VLCCustomObjectProperties;
    TraceID_t u_ReferenceToTrace;
} VLCObject_t;

/*-- VLCObjectList --*/
typedef VLCObject_t VLCPrivObjectList_t[Envm_N_OBJECTS];

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! The VLC-Sen FuncID used for meas-freezes */
#define VLC_MEAS_FUNC_ID TASK_ID_ALGO_SEN_CYCLE
/*! The VLC-Sen FuncChannelID used for meas-freezes @todo: Clarify later */
#define VLC_MEAS_FUNC_CHAN_ID 60u

/*****************************************************************************
  VARIABLES
*****************************************************************************/
extern MEMSEC_REF boolean VLCSenIsInitialized;
/* vehicle width */
extern MEMSEC_REF float32 VLC_fEgoVehicleWidth;
/* vehicle length */
extern MEMSEC_REF float32 VLC_fEgoVehicleLength;
extern MEMSEC_REF float32 VLC_fBumperToCoG;

/*! sub-module state */
extern MEMSEC_REF const VLCCtrlData_t*
    VLCSEN_pSenCtrlData; /*!< Pointer to VLC control data */
extern MEMSEC_REF const ECAMtCyclEnvmode_t*
    VLCSEN_pECAMtCyclEnvmode; /*!< Global system cycle mode etc. data */
extern MEMSEC_REF const Envm_t_GenObjectList*
    VLCSEN_pEmGenObjList; /*!< The generic EM object data */
extern MEMSEC_REF const Envm_t_CRObjectList* VLCSEN_pEmARSObjList;
extern MEMSEC_REF AssessedObjList_t*
    VLCSEN_pPubFctObjList; /*!< The public VLC object data */
extern MEMSEC_REF const VED_VehDyn_t*
    VLCSEN_pEgoDynObjSync; /*!< The object list synchronized ego
                              dynamic signals */
extern MEMSEC_REF const VED_VehDyn_t*
    VLCSEN_pEgoDynRaw; /*!< The dynamic vehicle ego data raw */
extern MEMSEC_REF const VED_VehPar_t*
    VLCSEN_pGlobEgoStatic; /*!< the static vehicle ego data */

extern MEMSEC_REF const VLCCustomInput_t*
    VLCSEN_pCustomInput; /*!< VLC custom input */
extern MEMSEC_REF VLCCustomOutput_t*
    VLCSEN_pCustomOutput; /*!< VLC custom output */

extern MEMSEC_REF VLCCDOutputCustom_t*
    VLCSEN_pCDCustomOutput; /*!< CD custom output data */

extern MEMSEC_REF VLC_HypothesisIntf_t*
    VLC_pCDHypothesesSen; /*!< VLC CD hypotheses */
extern MEMSEC_REF const Com_AlgoParameters_t*
    VLCSEN_pBswAlgoParameters; /*!< Input algo parameters from BSW */
extern MEMSEC_REF const VLC_Parameters_t*
    VLCSEN_pCPAR_VLC_Parameters; /*!< VLC Coding Parameters */

extern MEMSEC_REF const t_CamLaneInputData*
    VLCSEN_pCamLaneData; /*!< Camera lane input data */
/* Provide OOI Objects from SEN to VEH */
extern MEMSEC_REF VLCSenAccOOI_t* VLCSEN_pAccOOIData;

/*! the unsorted private vlc object list */
extern MEMSEC_REF VLCPrivObjectList_t VLCObjectList;

/*! VLC_SEN frame (cycle time, cycle counter, opmode ...) */
extern MEMSEC_REF VLCSenFrame_t VLCSenFrame;

/* Meas freeze reference */
extern MEMSEC_REF VLCSen_SyncRef_t VLCSenSyncRef;

/*****************************************************************************
  FUNCTION
*****************************************************************************/
/*! set states of subcomponents according to VLC_OP_MODE */
extern void VLCSenProcessStates(VLC_OP_MODE_t eOpMode);

/*! set states of subcomponents to save state and signal Error on VLC level */
extern void VLCSenSignalErrorShutdown(const boolean isRecoveryPossible);

/*! Process VLC Input. */
extern void VLCSenProcessInput(fTime_t fCycleTime, const VED_VehDyn_t* pEgoDyn);

/*! Process VLC Meas */
extern void VLCSenProcessMeasFreeze(
    const proVLCSenPrtList_t* const pProvidePorts);
extern void VLCSenMeasCallback(void);
extern void VLCSenFrameFreeze(void);

/*! Input meas freeze of VLC_SEN input */
extern void VLCSenInputMeasFreeze(void);

/*! Initialization data freeze of VLC_SEN */
extern void VLCSenInitMeasFreeze(void);

/* vlc_input.c externals */

/* Ego motion matrix get functions */

extern const GDBTrafoMatrix2D_t* VLCGetTrafoMatrix2DCOFForward(void);
extern const GDBTrafoMatrix2D_t* VLCGetTrafoMatrix2DCOFBackward(void);

/* Function to fill error output */
extern void VLCSenFillErrorOut(DFSErrorOut_t* pDest);

void VLCSEN_SERVICE_ADD_EVENT(const AS_t_RtaEventType RtaEvtType,
                              const uint8 u_IdLocal,
                              const uint8 u_OptData);

#ifdef __cplusplus
};
#endif

/* Include has to be located at the end of vlc_sen.h
   Typedefs in this file should be moved to a seperate file and this include
   should be moved to the top.*/
#include "vlc_sen_access_func.h"

/* END of #ifndef _VLC_SEN_H_INCLUDED */
#endif
/*! @endcond Doxygen_Suppress */
/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
