/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */

#ifndef _SI_H_INCLUDED
#define _SI_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "sen_sim.h"
#include "vlc_sen.h"
#include "si_ext.h"

#include "si_cfg.h"
#include "si_custom.h"
#include "fip_ext.h"
#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  SYMBOLISCHE KONSTANTEN (KOMPONENTENINTERN)
*****************************************************************************/

/* length of path occupation array */
#define SI_PATH_OVLC_ENTRIES ((2 * Envm_N_OBJECTS) + 4)

#ifndef SI_CUSTOM_OUTPUT_DEBUG_DATA_VADDR
#define SI_CUSTOM_OUTPUT_DEBUG_DATA_VADDR 539362432U
#endif
/*****************************************************************************
  MACROS (KOMPONENTENINTERN)
*****************************************************************************/

/*! The function ID used for SI measurement data output */
#define SI_MEAS_FUNC_ID VLC_MEAS_FUNC_ID

/*! The func channel ID used for SI measurement data output */
#define SI_MEAS_FUNC_CHAN_ID VLC_MEAS_FUNC_CHAN_ID

/*! Macro to check if Object Quality is sufficient for functional use*/
#define SI_SUFFICIENT_FUN_PRESEL_QUALITY(iObject) (SIBasePreselObjList[iObject])

/*! Macro to set the selection of a given object */
#define SI_OBJ_SET_OBJ_OF_INTEREST(iObj, iOOI) \
    (GET_VLC_OBJ_PUB(iObj).ObjOfInterest.eObjOOI = (iOOI))

/*! The SI inter-call cycle time */
#define SI_CYCLE_TIME TASK_CYCLE_TIME

/*****************************************************************************
  TYPEDEFS (KOMPONENTENINTERN)
*****************************************************************************/
/*!< Information about parameters used to check if there are other objects in
 * area relative to reference object */
typedef struct {
    /* Defines area and velocity delta in relation to reference object in which
     * objects can be located */
    float32 DeltaDistXLower; /* Maximum distance delta in negative X direction
                                e.g. (-20.f) */
    float32 DeltaDistXUpper; /* Maximum distance delta in positive X direction
                                e.g. (40.f)  */
    float32 DeltaDistYLower; /* Maximum distance delta in negative Y direction
                                e.g. (-6.f)  */
    float32 DeltaDistYUpper; /* Maximum distance delta in positive Y direction
                                e.g. (6.f)   */
    float32 DeltaVelLower;   /* Velocity delta lower threshold e.g. (-999.f) */
    float32 DeltaVelUpper; /* Velocity delta upper threshold e.g. (-5.f/3.6F) */
} SIFindObjInAreaArgs_t;

/*! defined different function types for object selection*/
typedef enum SIObjSelectionTypeTag {
    SI_OBJ_SELECTION_NEXT_LONG = 1,
    SI_OBJ_SELECTION_NEXT_LATERAL = 2
} SIObjSelectionType_t;

/*! Predicted distance structure with variances */
typedef struct SIPredictedDistanceTag {
    float32 pdist;              /*!< The predicted distance */
    float32 pdist_var;          /*!< The predicted distance variance */
    float32 pdist_var_fullpred; /*!< A variance corrected predicted distance */
} SIPredictedDistance_t;

typedef boolean BasePreselObjList_t; /*!< Type storing pre-selection decision
                                        for one object */

/*! Path occupation structure, used in si_check_occupied_lane.c. Only here
because simulation
frame needs access to type. In ECU code type is only used in the mentioned C
file */
typedef struct SIPathOccupationStruct {
    float32 LengthTrajEgoFrontToObj;
    float32 DistTrajToObjEdge;
    ObjNumber_t ObjID;
    struct SIPathOccupationStruct* next;
    struct SIPathOccupationStruct* prev;
} SiPathOccupation_t;

typedef SiPathOccupation_t SIPathOccupationArray_t[SI_PATH_OVLC_ENTRIES];

/*! Path occupation result structure, used in si_check_occupied_lane.c. Only
here because
simulation frame needs access to type. In ECU code type is only used in single C
file. */
typedef struct SIPathOccResultStruct {
    float32 LengthTrajEgoFrontToObj;
    float32 PathWidth;
    ObjNumber_t ObjIDL;
    ObjNumber_t ObjIDR;
} SiPathOccResult_t;

typedef SiPathOccResult_t SIPathOccResultArray_t[SI_PATH_OVLC_ENTRIES];

typedef struct {
    ObjNumber_t ObjID;
    float32 HalbeSpurbreite;
    float32 CalculatedObjLength;
    float32 DistTrajToObjEdge; /*!< Measured distance to trajectory */
    float32 ObjDistOnTraj;     /*!< Distance from EGO center of gravity to
                                  trajectory object reference point */
} SiPathOccInsertObjData_t;

/*! Struct for CutIn-potential specific object data */
typedef struct SiCutInObjDataTag {
    uint8 PotentialLateralKinematics;
    uint8 PotentialMultiObjectAnalyse;
    uint8 dummy01;
    uint8 dummy02;
} SICutInObjData_t;

/*! Type output over measurement interface for one relevant object */
typedef struct {
    ObjNumber_t object_id;  /*!< The object-ID of the object, or
                               OBJ_INDEX_NO_OBJECT if none */
    eAssociatedLane_t lane; /*!< The lane assigned to the object */
    Envm_t_CR_Classification object_class; /*!< The object class */
    Envm_t_CR_DynamicProperty object_type; /*!< The object type */
    EM_t_ARS_DynamicSubProperty
        object_sub_prop; /*!< The objects dynamic sub-property */
    uint8 potential; /*!< The cut-in potential for outlane objects, the cut-out
                        potential for inlane ones */
    boolean seen_moving; /*!< Flag indicating if object is stopped (stationary
                            now, but seen moving) */

    float32
        pred_lat_displ; /*!< Predicted lateral displacement from ego course */
    float32 long_displacement; /*!< Distance of object in X direction */
#if (defined(_MSC_VER))
#pragma COMPILEMSG("Remove @name modifier once visualizations updated!")
#endif
    float32 lat_displacement;      /*!< Distance of object in Y direction @name:
                                      lat_displacement_to_curvature @todo: remove
                                      misleading name alias! */
    float32 rel_long_velocity;     /*!< Object's relative speed in X direction
                                      (longitudinal) */
    float32 rel_long_acceleration; /*!< Object's relative acceleration in X
                                      direction */
    float32 rel_lat_velocity;      /*!< Object's relative speed in Y direction
                                      (lateral) */

#if (defined(_MSC_VER))
#pragma COMPILEMSG("Remove @name modifier once visualizations updated!")
    /* */

#endif
    float32 fTraceBracketLeft;  /*!< The left trace bracket offset to course
                                   estimation @unit: m @name:
                                   lat_displ_road_bord_l @todo: Modify birdeye
                                   for new name, then remove renaming! */
    float32 fTraceBracketRight; /*!< The right trace bracket offset to course
                                   estimation @unit: m @name:
                                   lat_displ_road_bord_r @todo: Modify birdeye
                                   for new name, then remove renaming! */
    SICustMeasOOI_t custom;     /*!< Custom meas data for OOI objects */
} SI_Meas_t;

/*! Type of measurement output for relevant objects (6) @vname: gSI_OOI_LIST
 * @vaddr: SI_OOI_LIST_MEAS_VADDR @VALUES: struct { SI_Meas_t
 * SI_OOI_LIST[SiAnzOOI]; } @cycleid: VLC_ENV */
typedef SI_Meas_t SI_MeasObjs_t[SiAnzOOI];

/*! Structure for si custom debug data */
typedef struct {
    uint8 ui_dummy;
} SICustomOutputDebugInfo_t;
/*****************************************************************************
  KONSTANTEN (KOMPONENTENINTERN)
*****************************************************************************/

/*****************************************************************************
  VARIABLEN (KOMPONENTENINTERN)
*****************************************************************************/

extern MEMSEC_REF BasePreselObjList_t SIBasePreselObjList[Envm_N_OBJECTS];
extern float32 fRangeFac;

extern MEMSEC_REF CPTrajectoryData_t SITrajectoryData;
extern MEMSEC_REF CPCourseData_t SICourseData;
extern ObjNumber_t SILastCycleOOIObjID[SiAnzOOI];

/*! @todo: Make RelObject static in si_objloss.c, remove type from si.he, modify
   spm_main.c to not use complete struct, but only the distance to relevant
   object */
extern MEMSEC_REF SIRelObject_t SIRelObject;

/*! Declaration of si custom debug data variable */
/*! @VADDR: SI_CUSTOM_OUTPUT_DEBUG_DATA_VADDR @CYCLEID: VLC_ENV */
extern SICustomOutputDebugInfo_t SICustomOutputDebugData;

/*****************************************************************************
  FUNKTIONEN (KOMPONENTENINTERN)
*****************************************************************************/

/*---si_objselection---*/
extern void SISelectBaseObjectsOfInterest(void);
extern void SIPreselectObjects(void);

extern void SIFreezeCustomOutputDebugData(void);
extern void SIGenerateOutputData(void);
extern void SIReSiDaInit(void);
extern void SISeReObPutRelTrckObjNumber(ObjNumber_t RelTrckObjNr);

/*---si_objfeatures----*/
extern void SIObReObRelObjLossReason(const ObjNumber_t NewObjId,
                                     const SIRelObject_t* pOldObject,
                                     SIRelObjEnum_t SiOOINr);
extern void SICalcBaseFeatures(void);
extern void SICalcBasePotentials(void);

/*---si_objattributes----*/
extern void SICalcObjAttributes(void);

extern void SIReInit(void);
extern void SIMeasCallback(void);

/*---si_laneassociation-----*/
extern void SILaneAssociation(void);
extern void SIObj2TrajInit(void);
extern boolean SICheckObjOccPickupValue(const ObjNumber_t iObj,
                                        const CPTrajOccupancy_t* pOccupancy);
extern boolean SI_b_CheckObjLaneQuality(const ObjNumber_t s_Obj);
extern boolean SI_b_CheckObjAdjLaneValidity(
    const ObjNumber_t s_Obj, const eAssociatedLane_t e_BaseAssocLane);
extern boolean SI_b_CheckObjOncRollBack(
    const ObjNumber_t s_Obj, const eAssociatedLane_t e_BaseAssocLane);

extern void SICalcPredDisplToCourseStandard(
    fTime_t prediction_time,
    Envm_t_CR_Classification object_class,
    ObjNumber_t obj_id,
    SIPredictedDistance_t* pred_dist);
extern fTime_t SILimitPredictionBySpeed(fTime_t normal_prediction_time);
extern fTime_t SILimitPredictionTimeDist(ObjNumber_t obj_id);
extern void SICalcPredDisplToCutOut(fTime_t prediction_time,
                                    Envm_t_CR_Classification object_class,
                                    ObjNumber_t obj_id,
                                    SIPredictedDistance_t* pred_dist);

/*--- si_calc_cutin_pot -----*/
extern uint8 SICalcCutInNeighborObj(const ObjNumber_t ObjId);
extern void SICutInObjectDataInit(void);
extern void SICutInObjectDataFreeze(void);
float32 SIGetCrossingDistTrace(const ObjNumber_t ObjID);

/*--- si_calc_cutout_pot -----*/
extern void SiCalculateCutOutPotFiltered(const float32 p_dist,
                                         const float32 FullPotLine,
                                         const float32 ZeroPotLine,
                                         const ObjNumber_t ObjNr);

/*--- si_objloss.c -----*/
extern void SIObReObInit(void);

/*--- si_select_moving.c -----*/
extern ObjNumber_t SISelectNextObjectMoving(eAssociatedLane_t Lane,
                                            fTime_t fPredictionTime,
                                            SIObjSelectionType_t SelectionType);

/*--- si_select_stationary.c -----*/
extern void SISelectStationaryObject(ObjNumber_t* const pNewObjId,
                                     SIRelObjEnum_t eRelObjType);

extern void SIObjectPreselection(void);

/*--- si_predicted_lane_ass.c -----*/
extern boolean SICheckPredictedInlaneCriteria(
    ObjNumber_t iObj, const CPTrajOccupancy_t* pOccupancy);
extern boolean SICheckPredictedOutlaneCriteria(
    ObjNumber_t iObj, const CPTrajOccupancy_t* pOccupancy);

extern void SISelectOncomingObject(ObjNumber_t* const pNewObjId,
                                   SIRelObjEnum_t eRelObjType);

extern void SICorridorInit(void);
extern void SI_Calculate_AVLC_Corridor(CPDistanceWidth_t pDistWidth[]);
extern float32 SIGetObjWidthForCorridor(const ObjNumber_t iObj);

/*--- si_check_occupied_lane.c ----*/
extern void SISelectCorridorObjects(ObjNumber_t* const pNextObjId,
                                    ObjNumber_t* const pHiddenObjId);
extern void SIInitBlockedPath(void);

/*--- si_lanechange.c ----*/
extern void SIInitLaneChange(void);
extern void SIDetectLaneChange(void);

/*--- si_corridor_scene.c ----*/
extern void SI_v_PerformO2OLaneAssociation(void);

/*--- si_acc_function_presel.c ----*/
extern void SIInitAccFunPreselection(void);
extern void SIFindObjInArea(ObjNumber_t p_ObjInArea[],
                            const ObjNumber_t ObjA,
                            SIFindObjInAreaArgs_t const* const AreaArgs);

/*--- si_output.c ----*/
extern void SIDeleteOOIData(AccOOIGenKinematics_t* pKinematic,
                            AccOOIGenAttributes_t* pAttributes);

#ifdef __cplusplus
};
#endif

/* Ende der bedingten Einbindung */
#else
#endif
