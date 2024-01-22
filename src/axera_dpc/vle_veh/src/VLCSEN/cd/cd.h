

#ifndef _CD_INT_INCLUDED
#define _CD_INT_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "vlcSen_ext.h"
#include "TM_Global_Types.h"
#include "TM_Math_Cal.h"
#include "vlc_types.h"
#include "vlc_glob_ext.h"

#include "emp_ext.h"
#include "cd_ext.h"
#include "cp_ext.h"
#include "cd_ext.h"
#include "si_ext.h"

#include "frame_sen_custom_types.h"
#include "vlc_ver.h"

#include "cd_par.h"
#include "cp_cd.h"

/*****************************************************************************
  MACROS
*****************************************************************************/
/*! @brief       Bit Mask for u16 Variables 1111 1111 1111 1111  */
#define BITMASK_UINT16 (0xFFFFu)

#ifdef ALGO_INLINE
#undef ALGO_INLINE
#endif
#define ALGO_INLINE static inline

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDMovement_t structure

    @general Structure to describe kinematics

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    float32 fX;  /*!< position X*/
    float32 fY;  /*!< position Y*/
    float32 fVx; /*!< velocity X*/
    float32 fVy; /*!< velocity Y*/
    float32 fAx; /*!< acceleration X*/
    float32 fAy; /*!< acceleration Y*/
} CDMovement_t;

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief CDSide_t Enum

    @general Type definition for sides relative to trajectory or sensor

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef enum { CD_SIDE_LEFT = 0u, CD_SIDE_RIGHT = 1u } CDSide_t;

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief eCDDynamicProperty_t Enumerator

    @general Type definition for DynamicProperty @todo: replace with global
   dynamic property

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef enum {
    CD_DYN_PROP_NO_OBJECT = 0u, /*!< NO Object */
    CD_DYN_PROP_STANDING = 1u,  /*!< Object Standing */
    CD_DYN_PROP_STOPPED = 2u,   /*!< Object Stopped */
    CD_DYN_PROP_MOVING = 3u,    /*!< Object Moving */
    CD_DYN_PROP_ONCOMING = 4u,  /*!< Object Oncoming */
    CD_DYN_PROP_CROSSING = 5u   /*!< Object Crossing */
} eCDDynamicProperty_t;

typedef uint8 CDDynamicProperty_t; /*!< @values: eCDDynamicProperty_t */

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief eCDAssociatedLane_t Enumerator

    @general Type definition for CD associated lane

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef enum {
    CD_LANE_ADJ_LEFT = 0u,     /* adjacent left lane */
    CD_LANE_EGO = 1u,          /* ego lane */
    CD_LANE_ADJ_RIGHT = 2u,    /* adjacent right lane */
    CD_NUM_OF_ASSOC_LANES = 3u /* number of lanes in this enum */
} eCDAssociatedLane_t;

/*! @brief Typedef for eCDAssociatedLane_t*/
typedef uint8 CDAssociatedLane_t; /*!< @values: eCDAssociatedLane_t */

#if (VLC_USE_EM_GENERIC_OBJECT_LIST)
/*! @brief Typedef for VLCCustGeometry_t*/
typedef VLCCustGeometry_t CDObjGeometry_t;
/*! @brief Typedef for VLCCustDimension_t */
typedef VLCCustDimension_t CDObjDimension_t;
#else
/*! @brief Typedef for Geometry_t */
typedef Geometry_t CDObjGeometry_t;
#endif

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDLaneAssociation_t structure

    @general Lane association information

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    percentage_t
        rgAssocLaneProb[CD_NUM_OF_ASSOC_LANES]; /*!< probabilities for the
                                                   associated lanes */
    percentage_t uiAssocLaneConf; /*!< confidence of associated lane */
} CDLaneAssociation_t;

/*! Structure for storing ego kinematic information */
/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDEgoDynamic_t Structure

    @general CD Ego Vehicle X&Y Velocity & Acceleration

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    fVelocity_t fVelocityX; /*!< Velocity X */
    fAccel_t fAccelX;       /*!< Acceleration X */
    fVelocity_t fVelocityY; /*!< Velocity Y */
    fAccel_t fAccelY;       /*!< Acceleration Y */
} CDEgoDynamic_t;

/*! Structure for storing ego kinematic information */
/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDEgoDynData_t Structure

    @general Structure holding Ego Dynamics data

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    const CDEgoDynamic_t *pEgoDynRaw; /*!< Raw ego dynamics */
    const CDEgoDynamic_t
        *pEgoDynObjSync; /*!< Object synchronized ego dynamics */
} CDEgoDynData_t;

/*! Hypothesis history flag
 *
 * @remark One bit per hypothesis type (used for a 1 loop history)
 */
/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDHypothesisHist_t Structure

    @general CD Hypothesis list

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    ubit16_t RunUpMoving : 1;     /*!< Hypothesis : Run up moving */
    ubit16_t Following : 1;       /*!< Hypothesis : Following */
    ubit16_t CutIn : 1;           /*!< Hypothesis : Cut In */
    ubit16_t Pass : 1;            /*!< Hypothesis : Pass */
    ubit16_t RunUpStationary : 1; /*!< Hypothesis : Run Up Stationary */
    ubit16_t WasOncomming : 1;    /*!< Hypothesis : Was Oncoming */
    ubit16_t PedColl : 1;         /*!< Hypothesis : Pedestrian Collission */
    ubit16_t PedPass : 1;         /*!< Hypothesis : Pedestrian Passing */
    ubit16_t CrossingLeft : 1;    /*!< Hypothesis : Crossing Left */
    ubit16_t CrossingRight : 1;   /*!< Hypothesis : Crossing Right */
    ubit16_t WasCrossing : 1;     /*!< Hypothesis : Was Crossing */
    ubit16_t BicycleColl : 1;     /*!< Hypothesis : Bicycle Collision */
} CDHypothesisHist_t;

/*! ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDRunUpBrkProbRed_t Structure

    @general more description here if any,otherwise skip this

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    float32 fVRelXStartRunUp; /*!< VrelX value at start of run-up (moving or
                                 braking) hypothesis */
    boolean bActive;          /*!< Indicate if reduction is active */
    uint8 uVrelXAbsDiff; /*!< VRelX reduction since start of run-up braking */
    uint16 uTimeGapVrel; /*!< time gap based on long. rel. velocity
                            @resolution:0.01 */
    uint16 uTimeGapVego; /*!< time gap based on ego velocity @resolution:0.01 */
} CDRunUpBrkProbRed_t;

/*! @brief       iOccupancy_min Current Object Occupancy MIN value */
#define iOccupancy_min (-32768)

/*! @brief       iOccupancy_max Current Object Occupancy MAX value */
#define iOccupancy_max (32767)

/*! @brief       iOccupancy_s Value used */
#define iOccupancy_s 1000.0F

/*! @brief Integer occupancy type
    @min:-32.767
    @max:32.767
    @resolution:0.001 */
typedef sint16 iOccupancy_t;

/*! Object information for RunUp/RunUpBraking Hypotheses */
/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDInternalObjHypRunUpData_t Structure

    @general Current and predicted Object Occupancy, Variance,
             trajectory,cource quality data

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    CDRunUpBrkProbRed_t BrkPropRed; /*!< Brake Proportional Reductiom*/

} CDInternalObjHypRunUpData_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDInternalObjHypRunUpStatData_t Structure

    @general Object information for RunUp Stationary Hypothesis

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    boolean bObstacleAtLeft;  /*!< there is an obstacle at the left side of this
                                 object*/
    boolean bObstacleAtRight; /*!< there is an obstacle at the right side of
                                 this object*/
} CDInternalObjHypRunUpStatData_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDInternalObjHypCutInData_t Structure

    @general Object information for cut-in hypothesis

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    percentage_t
        uiNeighborCutInProb; /*!< Cut-in probability for neighbor object */
    ObjNumber_t iNeighborRunUpObjIndex; /*!< Neighbor Cut-in run-up object */
} CDInternalObjHypCutInData_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDIntHypothesis_t Structure

    @general Internal hypothesis type

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    float32 fHypothesisProbability;           /*!< probability of the hypothesis
                                                 classification */
    ObjNumber_t iObjectRef;                   /*!< reference to the object*/
    Envm_t_GenObjClassification eObjectClass; /*!< object classification*/
    float32 fRelevance;          /*!< relevance of the hypothesis */
    eCDHypothesisType_t eType;   /*!< type of the hypothesis */
    fTime_t fHypothesisLifetime; /*!< hypothesis lifetime */
} CDIntHypothesis_t;

/*! @brief CDIntHypothesis_t Array For looping list of CD Hypothesis*/
typedef CDIntHypothesis_t CDIntHypothesisList_t[CD_NUMBER_OF_HYPOTHESES];

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDOutputData_t Structure

    @general Output Hypotheses array

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    Hypothesis_array_t *rgRelevantHypothesesList; /*!< Array of hypotheses */
    uint8 uiNumberOfHypotheses;   /*!< Number of valid hypotheses */
    HypoIntfDegr_t *pDegradation; /*!< Performance degradation */
} CDOutputData_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDHypothesisMeasurement_t Structure

    @general Hypothesis Measurement Data

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    uint8 TrackAssigned; /*!< object assigned to ego lane */
} CDHypothesisMeasurement_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDInternalMeasurementData_t Structure

    @general Hypotheses Measurement Data

    @conseq Introduce the consequences section

    @attention Special warnings about the consequence of any change

    @VADDR:VLC_MEAS_ID_CGEB_CD_HYPOTHESES

    @VNAME:TP_CD_Hypotheses

    @CYCLEID: VLC_ENV

    */
typedef struct {
    CDHypothesisMeasurement_t
        rgHypotheses[CD_NUMBER_OF_HYPOTHESES]; /*!< array of measurement data
                                                  structs */
    float32 fAllowedBrakeDist; /*!< below this alignment dependent distance
                                  braking will be allowed @unit:m*/
} CDInternalMeasurementData_t; /*!< @VADDR:VLC_MEAS_ID_CGEB_CD_HYPOTHESES
                                  @VNAME:TP_CD_Hypotheses @CYCLEID: VLC_ENV */

/* Pedestrian Hypothesis Internal Data */
/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDHypPedCollInternalData_t Structure

    @general Pedestrian Hypothesis Internal Data

    @conseq Not Applicable

    @attention Not Applicable
    */
typedef struct {
    uint8 afVelocityProbabilitiesOnlyFitting
        [CD_PED_COLL_PED_VELO_STEPS_N]; /*!< Fitting Velocity Probability */
    uint8 afVelocityProbabilitiesPlusGaussian
        [CD_PED_COLL_PED_VELO_STEPS_N]; /*!< Velocity Probability + Gaussian */
    uint8 afVelocityProbabilitiesOnlyGaussian
        [CD_PED_COLL_PED_VELO_STEPS_N]; /*!< Velocity Probability only
                                           Gaussian */
    float32 fHypProbPlusGaussian;   /*!< Hypothesis Probability incl gaussian */
    float32 fHypProbOnlyGaussian;   /*!< Hypothesis Probability only gaussian */
    float32 fDowngradeFactor;       /*!< Downgrade factor */
    float32 fCollCorrVeloLeft_mps;  /*!< Collission corridor velocity left */
    float32 fCollCorrVeloRight_mps; /*!< Collission corridor velocity right */
    float32 fHypProbOnlyFitting;    /*!< Hypothesis Probability fitting */
} CDHypPedCollInternalData_t;

// wulin todo 20220316, add CDHypBicycleCollInternalData_t struct
/* Bicycle Hypothesis Internal Data */
/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDHypBicycleCollInternalData_t Structure

    @general Bicycle Hypothesis Internal Data

    @conseq Not Applicable

    @attention Not Applicable
    */
typedef struct {
    uint8 afVelocityProbabilitiesOnlyFitting
        [CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N];
    uint8 afVelocityProbabilitiesPlusGaussian
        [CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N];
    uint8 afVelocityProbabilitiesOnlyGaussian
        [CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N];

    float fHypProbPlusGaussian;   /*!< Hypothesis Probability incl gaussian */
    float fHypProbOnlyGaussian;   /*!< Hypothesis Probability only gaussian */
    float fDowngradeFactor;       /*!< Downgrade factor */
    float fCollCorrVeloLeft_mps;  /*!< Collission corridor velocity left */
    float fCollCorrVeloRight_mps; /*!< Collission corridor velocity right */
    float fHypProbOnlyFitting;    /*!< Hypothesis Probability fitting */
} CDHypBicycleCollInternalData_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief  CDInternalObject_t Structure

    @general Collision detection relevant data in object array

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    fVelocity_t fVAbsX; /*!< absolute object velocity*/
    fAccel_t fAAbsX;    /*!< absolute object acceleration*/
    fTime_t
        TTC; /*!< Time to collision (assuming that the vehicle is in course*/
    fTime_t
        TTC2; /*!< Time to collision 2 (assuming that the vehicle is in course*/
    fTime_t
        TTC3; /*!< Time to collision 3 (assuming that the vehicle is in course*/
    fTime_t
        TTC4; /*!< Time to collision 4 (assuming that the vehicle is in course*/
    fAccel_t LongNecAccel; /*!< Necessary longitudinal acceleration to avoid a
                              collision*/
    fAccel_t
        LatNecAccel;  /*!< Necessary lateral acceleration to avoid a collision*/
    fTime_t TTBPre;   /*!< time to brake for comfort braking */
    fTime_t TTBAcute; /*!< time to brake for emergency braking */
    fTime_t TTSPre;   /*!< time to steer for comfort steering*/
    fTime_t TTSAcute; /*!< time to steer for emergency steering*/
    fVelocity_t ClosingVelocity; /*!< Relative speed at time of collision (not
                                    AUTOSAR)*/
    fTime_t TTSPreRight;         /*!< time to steer for comfort steering right*/
    fTime_t TTSPreLeft;          /*!< time to steer for comfort steering left */
    fTime_t TTSAcuteRight;     /*!< time to steer for emergency steering right*/
    fTime_t TTSAcuteLeft;      /*!< time to steer for emergency steering left */
    fAccel_t LatNecAccelRight; /*!< Necessary lateral acceleration to right side
                                  to avoid a collision*/
    fAccel_t LatNecAccelLeft;  /*!< Necessary lateral acceleration to left side
                                  to avoid a collision */
    EMPObjDesc_t EMPObjData;   /*!< EMP Object Data*/

    uint8 uiCriticalTimeAfterMerge; /*!< is a object merge with high lateral
                                    displacement
                                    and long lifetimes for both objects happened
                                    add a timer
                                    that sets the lateral speed to 0*/
    uint8 TrackAssigned; /*!< Shifting Register for Track Assignment */
    CDHypothesisHist_t HypothesisHist; /*!< List if the object had a hypothesis
                                          in the last loop*/
    EMPObjToTrajRelation_t TrajRelation; /*!< Trajectory */
    uint16
        bitHypPresel; /*!< one bit for each hypothesis @values:
                      struct {
                      .offset(0) = uint16 Bits;
                      .offset(0).mask(1<<(CDHypothesisType_RunUp-1)) = uint16
                      bRunUp;
                      .offset(0).mask(1<<(CDHypothesisType_RunUpBraking-1)) =
                      uint16 bRunUpBraking;
                      .offset(0).mask(1<<(CDHypothesisType_RunUpStationary-1)) =
                      uint16 bRunUpStationary;
                      .offset(0).mask(1<<(CDHypothesisType_Static-1)) = uint16
                      bStatic;
                      .offset(0).mask(1<<(CDHypothesisType_ACC-1)) = uint16
                      bACC;
                      .offset(0).mask(1<<(CDHypothesisType_Pass-1)) = uint16
                      bPass;
                      .offset(0).mask(1<<(CDHypothesisType_CutIn-1)) = uint16
                      bCutIn;
                      .offset(0).mask(1<<(CDHypothesisType_Collision-1)) =
                      uint16 Collision;
                      .offset(0).mask(1<<(CDHypothesisType_CollisionUnavoidable-1))
                      = uint16 CollisionUnAv;
                      .offset(0).mask(1<<(CDHypothesisType_PedCollision-1)) =
                      uint16 PedColl;
                      .offset(0).mask(1<<(CDHypothesisType_PedPass-1)) = uint16
                      PedPass;
                      }*/
    uint16
        bitHypActive; /*!< one bit for each hypothesis @values:
                      struct {
                      .offset(0) = uint16 Bits;
                      .offset(0).mask(1<<(CDHypothesisType_RunUp-1)) = uint16
                      bRunUp;
                      .offset(0).mask(1<<(CDHypothesisType_RunUpBraking-1)) =
                      uint16 bRunUpBraking;
                      .offset(0).mask(1<<(CDHypothesisType_RunUpStationary-1)) =
                      uint16 bRunUpStationary;
                      .offset(0).mask(1<<(CDHypothesisType_Static-1)) = uint16
                      bStatic;
                      .offset(0).mask(1<<(CDHypothesisType_ACC-1)) = uint16
                      bACC;
                      .offset(0).mask(1<<(CDHypothesisType_Pass-1)) = uint16
                      bPass;
                      .offset(0).mask(1<<(CDHypothesisType_CutIn-1)) = uint16
                      bCutIn;
                      .offset(0).mask(1<<(CDHypothesisType_Collision-1)) =
                      uint16 Collision;
                      .offset(0).mask(1<<(CDHypothesisType_CollisionUnavoidable-1))
                      = uint16 CollisionUnAv;
                      .offset(0).mask(1<<(CDHypothesisType_PedCollision-1)) =
                      uint16 PedColl;
                      .offset(0).mask(1<<(CDHypothesisType_PedPass-1)) = uint16
                      PedPass;
                      }*/

    CDInternalObjHypRunUpData_t
        sHypRunUpData; /*!< run up hypothesis object data*/
    CDInternalObjHypRunUpStatData_t
        sHypRunUpStatData; /*!< run up stationary hypothesis object data*/
    CDHypPedCollInternalData_t
        sHypPedCollData; /*!< Debug Data for CDHypPedColl used to transport data
                            to MTS CDViewPlugin >*/
    // wulin to do 202203016, add sHypBicycleCollData
    CDHypBicycleCollInternalData_t
        sHypBicycleCollData; /*!< Debug Data for CDHypBicycleColl used to
                            transport data to MTS CDViewPlugin >*/
} CDInternalObject_t;

/*! @brief CD Internal object list        */
typedef CDInternalObject_t CDInternalObjectList_t[Envm_N_OBJECTS];

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Describe the Structure briefly here.

    @general Internal CD status

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    CDInternalObjectList_t *rgObjInternal; /*!< Internal object list */
    CDIntHypothesisList_t
        *rgIntRelevantHypothesesList; /*!< Relevant hypotheses */
    CDIntHypothesisList_t
        *rgPreviousHypothesesList; /*!< Relevant hypotheses of previous cycle */
    EMPSize2D_t sEgoGeometry;      /*!< S Ego geomerty */
    EMPKinEgo_t sKinEgo;           /*!< S Kinetic Ego */
    uint8 uiNofRelevantHypotheses;
    uint8 uiNofPreviousHypotheses;
} CDInternalStatus_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDObjectData_t Structure

    @general Pointers to various object attributes

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    ObjNumber_t iNumberOfObjects;            /*!< Number of objects */
    const Envm_t_GenObjectList *pGenObjList; /*!< Generic object list */
    const Envm_t_CRObjectList
        *pARSObjList; /*!< ARS-technology specific object list */
    const VLCPrivObjectList_t *pVLCPrivObjList;  /*!< VLC private object list */
    CDInternalObjectList_t *pInternalObjectList; /*!< CD Internal List */
} CDObjectData_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDInputData_t Structure

    @general Main input data structure for CD

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct CDInputData {
    const CDObjectData_t *pObjectData; /*!< Pointer to object data */
    const CDEgoDynData_t *pEgoData;    /*!< Pointer to ego data */
} CDInputData_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDAdjSafeDistance_t Structure

    @general Structure for setting min. required safety distances in
             lateral/longitudinal direction

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    float32 fLongNecRemainDist; /*!< Safety distance for calculation of
                                   Necessary long. deceleration, in m @unit:m */
    float32 fLatNecRemainDist; /*!< Safety distance for calculation of Necessary
                                  lat. acceleration, in m @unit:m */
} CDAdjSafeDistance_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDParameters_t Structure ,CGEB parameters
    @general Adjustable safety distances

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    CDAdjSafeDistance_t *pAdjSafeDistance; /*!< Safe Distance from vehicle */
} CDParameters_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDBaseObjTrajectory_t Structure

    @general Struct to describe object kinematics relative to trajectory

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    float32 fDistToTraj;     /*!< lateral object displacement to a trajectory*/
    float32 fDistToTrajVar;  /*!< variance of lateral object displacement to a
                                trajectory*/
    float32 fVelocityToTraj; /*!< lateral object velocity to a trajectory*/
    float32 fVelocityToTrajVar; /*!< variance of lateral object velocity to a
                                   trajectory*/
} CDBaseObjTrajectory_t;

/* ---- math function pointers ---- */

/* Function pointer to sqrt function */
/*! @brief Describe the Typedef briefly here.    */
typedef float32 (*CDSQRT_fpt)(float32 fArgument);

/* ---- other function pointers ---- */

/*function pointer to SAOverlapCalculation function*/
/*! @brief Describe the Typedef briefly here. */
typedef void (*CPOverlapCalculation_fpt)(const CPDistanceWidth_t *pDistWidthInt,
                                         CPTrajOccupancy_t *pOccupancy);
/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDExternalFunctions_t Structure

    @general CD External Function

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    CPOverlapCalculation_fpt CPCalculateOverlap; /*!< Member Description */
} CDExternalFunctions_t;

/*! Hypothesis handler function pointer */
typedef void (*CDHypothesisHandler_fpt)(
    ObjNumber_t iObjectIndex,
    boolean bObjFilterMatched,
    const CDInputData_t *pInputData,
    CDInternalStatus_t *pInternalStatus,
    const CDExternalFunctions_t *pExternalFunctions);

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CDHypoClassFilter_t Structure

    @general CD classification filter

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    Envm_t_GenObjClassification
        eClassification; /*!< Classification of the object */
} CDHypoClassFilter_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief  CDHypoHandler_t Structure

    @general CD hypothesis handler

    @conseq Not Applicable

    @attention Not Applicable

    */
typedef struct {
    CDHypothesisHandler_fpt fpHypHandler; /*!< The hypothesis handler */
    const CDHypoClassFilter_t *
        *rgClassFilter; /*!< Filter descriptor of relevant objects */
    uint16 uiHypTypes;  /*!< The hypothesis types which could be set */
    Envm_t_GenEbaHypCat bitHypCat; /*!< Bitfield for the hypothesis category */
    uint8 uiGenObjQuality;         /*!< The object general quality */
    uint8 uiStatObjQuality;        /*!< The object stationary quality */
    uint8 uiPedObjQuality;         /*!< The object pedestrian quality */
    uint8 uiMinClassConf; /*!< Min. classification confidence of the object */
    boolean bNegativeFilter; /*!< The filter is negative, i.e. hypotheses is
                                evaluated except for the classes in the filter
                                */
    uint8 uiClassFilterCnt;  /*!< Number of hypothesis handlers in the list */
    uint8 uRTACheckpoint;
} CDHypoHandler_t;

/*--- External declaration of hypothesis handlers ---*/
/*!  @cond Doxygen_Suppress */
extern const CDHypoHandler_t *const CD_HYP_HANDLERS[CD_HYP_HANDLERS_NO];

/*! @endcond */
/*****************************************************************************
  FUNCTIONS
*****************************************************************************/
ALGO_INLINE void CD_CHECK_NULL(eGDBError_t *error, const void *pointer);
ALGO_INLINE void CD_SET_HYP_BIT(uint16 *var, uint32 hypnr);
ALGO_INLINE void CD_CLEAR_HYP_BIT(uint16 *var, uint32 hypnr);

extern eGDBError_t CDRun(const CDInputData_t *pInputData,
                         CDInternalStatus_t *pInternalStatus,
                         CDOutputData_t *pOutputData,
                         const CDParameters_t *pParameters,
                         const CDExternalFunctions_t *pExternalFunctions,
                         CDInternalMeasurementData_t *pMeasurementData);

extern void CDPrepareCycleHypothesesSelection(
    CDInternalStatus_t *pInternalStatus);
extern void CDCalculateObjectProperties(
    const CDInputData_t *pInputData,
    const CDInternalStatus_t *pInternalStatus,
    const CDParameters_t *pParameters);
/*!  @cond Doxygen_Suppress */
extern void CDAssignTrackProbability(ObjNumber_t iObjectIndex,
                                     const CDInputData_t *pInputData,
                                     const CDInternalStatus_t *pInternalStatus);
/*! @endcond */
extern void CDSortHypotheses(const CDInternalStatus_t *pInternalStatus,
                             Hypothesis_array_t *rgSortedHypothesesList,
                             CDInternalMeasurementData_t *pMeasurementData,
                             const CDObjectData_t *pObjectData,
                             const CDInternalObjectList_t *rgObjInternal);
extern float32 CDCalcCorridorWidth(const float32 fDistX,
                                   const float32 fLength,
                                   const float32 fNarrow,
                                   const float32 fWide,
                                   const float32 fInflectionpoint);
extern void CDHypothesesSelection(CDIntHypothesis_t *pHypothesis,
                                  const CDObjectData_t *pObjectData,
                                  CDInternalStatus_t *pInternalStatus);
extern void CDUpdateEMPData(const CDInputData_t *pInputData,
                            CDInternalStatus_t *pInternalStatus);

extern void CDMergeInternalObjects(
    CDInternalObject_t *const pObjectToKeep,
    const CDInternalObject_t *const pObjectToDelete);
extern void CDDeleteInternalObject(CDInternalObject_t *const pLocalObject);

/*!  @cond Doxygen_Suppress */
extern void CDHypoRunUpMain(ObjNumber_t iObjectIndex,
                            boolean bObjFilterMatched,
                            const CDInputData_t *pInputData,
                            CDInternalStatus_t *pInternalStatus,
                            const CDExternalFunctions_t *pExternalFunctions);
extern void CDHypoAccMain(ObjNumber_t iObjectIndex,
                          boolean bObjFilterMatched,
                          const CDInputData_t *pInputData,
                          CDInternalStatus_t *pInternalStatus,
                          const CDExternalFunctions_t *pExternalFunctions);
extern void CDHypoCutInMain(ObjNumber_t iObjectIndex,
                            boolean bObjFilterMatched,
                            const CDInputData_t *pInputData,
                            CDInternalStatus_t *pInternalStatus,
                            const CDExternalFunctions_t *pExternalFunctions);
extern void CDHypoPassMain(ObjNumber_t iObjectIndex,
                           boolean bObjFilterMatched,
                           const CDInputData_t *pInputData,
                           CDInternalStatus_t *pInternalStatus,
                           const CDExternalFunctions_t *pExternalFunctions);
extern void CDHypoStaticMain(ObjNumber_t iObjectIndex,
                             boolean bObjFilterMatched,
                             const CDInputData_t *pInputData,
                             CDInternalStatus_t *pInternalStatus,
                             const CDExternalFunctions_t *pExternalFunctions);
extern void CDHypoCollisionMain(
    ObjNumber_t iObjectIndex,
    boolean bObjFilterMatched,
    const CDInputData_t *pInputData,
    CDInternalStatus_t *pInternalStatus,
    const CDExternalFunctions_t *pExternalFunctions);
extern void CDHypoRunUpStationaryMain(
    ObjNumber_t iObjectIndex,
    boolean bObjFilterMatched,
    const CDInputData_t *pInputData,
    CDInternalStatus_t *pInternalStatus,
    const CDExternalFunctions_t *pExternalFunctions);
extern void CDHypoPedCollMain(ObjNumber_t iObjectIndex,
                              boolean bObjFilterMatched,
                              const CDInputData_t *pInputData,
                              CDInternalStatus_t *pInternalStatus,
                              const CDExternalFunctions_t *pExternalFunctions);

// wulin to do 20220316, add CDHypoBicycleCollMain()
extern void CDHypoBicycleCollMain(
    ObjNumber_t iObjectIndex,
    boolean bObjFilterMatched,
    const CDInputData_t *pInputData,
    CDInternalStatus_t *pInternalStatus,
    const CDExternalFunctions_t *pExternalFunctions);

extern void CDHypoPedPassMain(ObjNumber_t iObjectIndex,
                              boolean bObjFilterMatched,
                              const CDInputData_t *pInputData,
                              CDInternalStatus_t *pInternalStatus,
                              const CDExternalFunctions_t *pExternalFunctions);
extern void CDHypoCrossingMain(ObjNumber_t iObjectIndex,
                               boolean bObjFilterMatched,
                               const CDInputData_t *pInputData,
                               CDInternalStatus_t *pInternalStatus,
                               const CDExternalFunctions_t *pExternalFunctions);

extern void CDHypoRunUpInitInternalData(
    CDInternalObjHypRunUpData_t *pRunUpData);
extern void CDHypoCutInInitInternalData(
    CDInternalObjHypCutInData_t *pCutInData);
/*! @endcond */

void CDInitInternalObjData(CDInternalObject_t *pInternalObjData);

void CDFillEgoMovement(CDMovement_t *pEgoMovement,
                       const CDInputData_t *pInputData);

void CDFillObjectMovement(ObjNumber_t iObjectIndex,
                          CDMovement_t *pObjMovement,
                          const CDInputData_t *pInputData,
                          const CDInternalObject_t *pCDInternalObject);

void CDPredictMovement(const CDMovement_t *pMovement,
                       CDMovement_t *pPredictedMovement,
                       float32 fPredictTime,
                       boolean bStopXDir,
                       boolean bStopYDir);

boolean CDCalculateClosingVelocity(const CDMovement_t *pEgoMovement,
                                   const CDMovement_t *pObjMovement,
                                   fVelocity_t *pfClosingVelocity);

boolean CDCalculateAnecLong(const CDMovement_t *pEgoMovement,
                            const CDMovement_t *pObjMovement,
                            fAccel_t *pfAnecLong);

boolean CDCalculateAnecLongLatency(
    const CDMovement_t *pEgoMovement,
    const CDMovement_t *pObjMovement,
    const CDAdjSafeDistance_t *pCDAdjSafeDistance,
    fAccel_t *pfAnecLong);

boolean CDCalculateAnecLat(const CDMovement_t *pEgoMovement,
                           const CDMovement_t *pObjMovement,
                           fDistance_t fLateralOffset,
                           fAccel_t *pfAnecLat);

boolean CDCalculateVehStopTime(fVelocity_t fVelocity,
                               fAccel_t fAcceleration,
                               fTime_t *pfStopTime);

boolean CDCalculateTTC2(const CDMovement_t *pEgoMovement,
                        const CDMovement_t *pObjMovement,
                        fTime_t *pfTTC);

boolean CDCalculateTTC3(const CDMovement_t *pEgoMovement,
                        const CDMovement_t *pObjMovement,
                        fTime_t *pfTTC);

boolean CDCalculateTTC4(const CDMovement_t *pEgoMovement,
                        const CDMovement_t *pObjMovement,
                        fTime_t *pfTTC);

boolean CDCalculateTTC(const CDMovement_t *pEgoMovement,
                       const CDMovement_t *pObjMovement,
                       fTime_t *pfTTC);

boolean CDCalculateTTBDyn(const CDMovement_t *pEgoMovement,
                          const CDMovement_t *pObjMovement,
                          fAccel_t fEgoAccelAssumed,
                          fTime_t *pfTTB);

boolean CDCalculateTTSDyn(const CDMovement_t *pEgoMovement,
                          const CDMovement_t *pObjMovement,
                          fAccel_t fEgoAccelAssumed,
                          fDistance_t fLateralOffset,
                          fDistance_t fDistanceToPass,
                          fTime_t *pfTTS,
                          const boolean bUseSteeringChange);

/*------------- custom functions----------*/
void CDInitCustomerData(void);

extern void CDCalcCustomerFunctions(
    CDInternalStatus_t *pInternalStatus,
    const CDObjectData_t *pObjectData,
    const CDExternalFunctions_t *pExternalFunctions);
extern void CDCustomerHypothesisHandler(
    const CDInputData_t *pInputData,
    CDInternalStatus_t *pInternalStatus,
    const CDExternalFunctions_t *pExternalFunctions);
extern void CDGetCustomerParameters(CDInternalStatus_t *pInternalStatus);

extern void CDCalcCustomerPerfDegradation(
    const CDInputData_t *pInputData,
    CDOutputData_t *pOutputData,
    const CDExternalFunctions_t *pExternalFunctions);

ALGO_INLINE const Envm_t_GenObjKinEnvmatics *CDGetPointer_Kinematic(
    const CDObjectData_t *const pCDObjectData, const ObjNumber_t iObjIndex);
ALGO_INLINE const Envm_t_GenObjAttributes *CDGetPointer_Attributes(
    const CDObjectData_t *const pCDObjectData, const ObjNumber_t iObjIndex);
ALGO_INLINE const CDObjDimension_t *CDGetPointer_Dimensions(
    const CDObjectData_t *const pCDObjectData, const ObjNumber_t iObjIndex);
ALGO_INLINE float32 CDGet_Dimension_Width(
    const CDObjectData_t *const pCDObjectData, const ObjNumber_t iObjIndex);
ALGO_INLINE uint8 CD_EBA_OBJ_QUALITY(const CDObjectData_t *pCDObjectData,
                                     const ObjNumber_t iObjIndex);
ALGO_INLINE float32 CD_OBJ_WIDTH_STDDEV(const CDObjectData_t *pCDObjectData,
                                        const ObjNumber_t iObjIndex);
ALGO_INLINE float32 CD_OBJ_LENGTH_STDDEV(const CDObjectData_t *pCDObjectData,
                                         const ObjNumber_t iObjIndex);
ALGO_INLINE float32 CD_OBJ_DISTX_STDDEV(const CDObjectData_t *pCDObjectData,
                                        const ObjNumber_t iObjIndex);
ALGO_INLINE float32 CD_OBJ_DISTY_STDDEV(const CDObjectData_t *pCDObjectData,
                                        const ObjNumber_t iObjIndex);
ALGO_INLINE float32 CD_GET_DIST_Y(const CDObjectData_t *pCDObjectData,
                                  const ObjNumber_t iObjIndex);
ALGO_INLINE float32 CD_GET_DIST_X(const CDObjectData_t *pCDObjectData,
                                  const ObjNumber_t iObjIndex);

/*------------- CPCD functions -----------*/
extern const CPCDObjToTrajRelation_t *CPCDGetObjToTrajRelationEgo(
    ObjNumber_t iObjId);
extern const CPCDObjToTrajRelation_t *CPCDGetObjToTrajRelationRoad(
    ObjNumber_t iObjId);

/*------------- additional data functions -----------*/
extern CDInputData_t *CDGetPointer_InputData(void);

/*------------- variables -----------*/
extern CDAdjSafeDistance_t
    CDAdjSafeDistance; /*!< CGEB safety distance parameters @allow:oem_bmw */

/*! @brief       max number of CD hypotheses types */
#define CDHypothesisType_Max (16u)

/*****************************************************************************
  INLINE FUNCTIONS
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    CD_CHECK_NULL */ /*!

      @brief           Functional Summary

      @description     Detailed Design

      @return          ALGO_INLINE void

      @param[in,out]   error :
      @param[in]       pointer :

      @pre             [ None ]
      @post            [ None ]

    *************************************************************************************************************************/
ALGO_INLINE void CD_CHECK_NULL(eGDBError_t *error, const void *pointer) {
    if ((GDB_ERROR_NONE == (*error)) && (NULL == (pointer))) {
        (*error) = GDB_ERROR_POINTER_NULL;
    }
}

/*************************************************************************************************************************
  Functionname:    CD_SET_HYP_BIT */ /*!

      @brief           Functional Summary

      @description     set a bit in a uint

      @return          ALGO_INLINE void

      @param[in,out]   *var :
      @param[in]       hypnr :

      @pre             [ None ]
      @post            [ None ]

    *************************************************************************************************************************/
ALGO_INLINE void CD_SET_HYP_BIT(uint16 *var, uint32 hypnr) {
    if ((hypnr > CDHypothesisType_No) && (hypnr <= CDHypothesisType_Max)) {
        (*var) |= (uint16)(1u << (hypnr - 1u));
    } else {
        /* do nothing */
    }
}

/*************************************************************************************************************************
  Functionname:    CD_CLEAR_HYP_BIT */ /*!

      @brief           Functional Summary

      @description     clear a bit in a uint

      @return          ALGO_INLINE void

      @param[in,out]   *var :
      @param[in]       hypnr :

      @pre             [ None ]
      @post            [ None ]
    *************************************************************************************************************************/
ALGO_INLINE void CD_CLEAR_HYP_BIT(uint16 *var, uint32 hypnr) {
    if ((hypnr > CDHypothesisType_No) && (hypnr <= CDHypothesisType_Max)) {
        (*var) &= (uint16)((~(1u << (hypnr - 1u))) & BITMASK_UINT16);
    } else {
        /* do nothing */
    }
}

/*************************************************************************************************************************
  Functionname:    CDGetPointer_Attributes */ /*!

      @brief           returns a pointer to the specified object attribute

      @description     returns a pointer to the "attributes" data of the
    uiObjIndex's object

      @return          ALGO_INLINE const Envm_t_GenObjAttributes *

      @param[in]       pCDObjectData :
      @param[in]       iObjIndex :

      @pre             [ None ]
      @post            [ None ]

    *****************************************************************************/
ALGO_INLINE const Envm_t_GenObjAttributes *CDGetPointer_Attributes(
    const CDObjectData_t *const pCDObjectData, const ObjNumber_t iObjIndex) {
    return &((*(pCDObjectData)->pVLCPrivObjList)[iObjIndex]
                 .VLCCustomObjectProperties.Attributes);
}
/* ***********************************************************************
  @fn              CDGetPointer_Kinematic  */ /*!

                               @brief           returns a pointer to the
                             specified object attribute

                               @description     returns a pointer to the
                             "kinematic" data of the uiObjIndex's object

                               @param[in]       pCDObjectData   CD internal
                             object data
                               @param[in]       iObjIndex       object number

                               @return          pointer to kinematic
                             object[iObjIndex]

                               @pre             [ None ]
                               @post            [ None ]

                             *****************************************************************************/
ALGO_INLINE const Envm_t_GenObjKinEnvmatics *CDGetPointer_Kinematic(
    const CDObjectData_t *const pCDObjectData, const ObjNumber_t iObjIndex) {
    return &(pCDObjectData->pGenObjList->aObject[iObjIndex].Kinematic);
}

/* ***********************************************************************
  @fn              CDGetPointer_Dimensions */ /*!

                               @brief           returns a pointer to the
                             specified object attribute

                               @description     returns a pointer to the
                             "dimensions" data of the uiObjIndex's object

                               @param[in]       pCDObjectData   CD internal
                             object data
                               @param[in]       iObjIndex       object number

                               @return          pointer to dimensions
                             object[iObjIndex]

                             *****************************************************************************/
ALGO_INLINE const CDObjDimension_t *CDGetPointer_Dimensions(
    const CDObjectData_t *const pCDObjectData, const ObjNumber_t iObjIndex) {
    /*use EM data directly*/
    return &(pCDObjectData->pARSObjList->aObject[iObjIndex].Geometry);
}

/*************************************************************************************************************************
  Functionname:    CDGet_Dimension_Width */ /*!

      @brief           Functional Summary

      @description     Detailed Design

      @return          ALGO_INLINE float32

      @param[in]       pCDObjectData :
      @param[in]       iObjIndex :

      @pre             [ None ]
      @post            [ None ]
    *************************************************************************************************************************/
ALGO_INLINE float32 CDGet_Dimension_Width(
    const CDObjectData_t *const pCDObjectData, const ObjNumber_t iObjIndex) {
    float32 fObjWidth =
        pCDObjectData->pARSObjList->aObject[iObjIndex].Geometry.fWidth;
    /* Limit the Width of the stationary Objects that is used to calculate the
     * Overlaps */
    if ((OBJ_DYNAMIC_PROPERTY(iObjIndex) == CR_OBJECT_PROPERTY_STATIONARY) &&
        (!OBJ_IS_MOVING_TO_STATIONARY(iObjIndex))) {
        fObjWidth = MIN_FLOAT(CD_MAX_OBJ_WIDTH_STAT, fObjWidth);
    }
    return (fObjWidth);
}

/*************************************************************************************************************************
  Functionname:    CD_GET_DIST_X */ /*!

      @brief           Functional Summary

      @description     Detailed Design

      @return          ALGO_INLINE float32

      @param[in]       pCDObjectData :
      @param[in]       iObjIndex :

      @pre             [ None ]
      @post            [ None ]
    *************************************************************************************************************************/
ALGO_INLINE float32 CD_GET_DIST_X(const CDObjectData_t *pCDObjectData,
                                  const ObjNumber_t iObjIndex) {
    return (pCDObjectData->pGenObjList->aObject[iObjIndex].Kinematic.fDistX);
}

/*************************************************************************************************************************
  Functionname:    CD_GET_DIST_Y */ /*!

      @brief           Functional Summary

      @description     Detailed Design

      @return          ALGO_INLINE float32

      @param[in]       pCDObjectData :
      @param[in]       iObjIndex :

      @pre             [ None ]
      @post            [ None ]
    *************************************************************************************************************************/
ALGO_INLINE float32 CD_GET_DIST_Y(const CDObjectData_t *pCDObjectData,
                                  const ObjNumber_t iObjIndex) {
    return (pCDObjectData->pGenObjList->aObject[iObjIndex].Kinematic.fDistY);
}

/*************************************************************************************************************************
  Functionname:    CD_EBA_OBJ_QUALITY */ /*!

      @brief           CD EBA Object Quality

      @description     CD EBA Object Quality

      @return          ALGO_INLINE uint8

      @param[in]       pCDObjectData :
      @param[in]       iObjIndex :

      @pre             [ None ]
      @post            [ None ]
    *************************************************************************************************************************/
ALGO_INLINE uint8 CD_EBA_OBJ_QUALITY(const CDObjectData_t *pCDObjectData,
                                     const ObjNumber_t iObjIndex) {
    return pCDObjectData->pGenObjList->aObject[iObjIndex]
        .Qualifiers.uiEbaObjQuality;
}

/*************************************************************************************************************************
  Functionname:    CD_OBJ_WIDTH_STDDEV */ /*!

      @brief           CD Object Width Standard Deviation

      @description     CD Object Width Standard Deviation

      @return          ALGO_INLINE float32

      @param[in]       pCDObjectData :
      @param[in]       iObjIndex :

      @pre             [None]
      @post            [None]
    *************************************************************************************************************************/
ALGO_INLINE float32 CD_OBJ_WIDTH_STDDEV(const CDObjectData_t *pCDObjectData,
                                        const ObjNumber_t iObjIndex) {
    _PARAM_UNUSED(pCDObjectData);
    _PARAM_UNUSED(iObjIndex);
    return CD_CONST_OBJ_WIDTH_VAR;
}

/*************************************************************************************************************************
  Functionname:    CD_OBJ_LENGTH_STDDEV */ /*!

      @brief           CD Object Length Standard Deviation

      @description     CD Object Length Standard Deviation

      @return          ALGO_INLINE float32

      @param[in]       pCDObjectData :
      @param[in]       iObjIndex :

      @pre             [None]
      @post            [None]
    *************************************************************************************************************************/
ALGO_INLINE float32 CD_OBJ_LENGTH_STDDEV(const CDObjectData_t *pCDObjectData,
                                         const ObjNumber_t iObjIndex) {
    _PARAM_UNUSED(pCDObjectData);
    _PARAM_UNUSED(iObjIndex);
    return CD_CONST_OBJ_WIDTH_VAR;
}

/*************************************************************************************************************************
  Functionname:    CD_OBJ_DISTX_STDDEV */ /*!

      @brief           CD Object DistanceX Standard Deviation

      @description     CD Object DistanceX Standard Deviation

      @return          ALGO_INLINE float32

      @param[in]       pCDObjectData :
      @param[in]       iObjIndex :

      @pre             [None]
      @post            [None]
    *************************************************************************************************************************/
ALGO_INLINE float32 CD_OBJ_DISTX_STDDEV(const CDObjectData_t *pCDObjectData,
                                        const ObjNumber_t iObjIndex) {
    _PARAM_UNUSED(pCDObjectData);
    _PARAM_UNUSED(iObjIndex);
    return 0;
}

/*************************************************************************************************************************
  Functionname:    CD_OBJ_DISTY_STDDEV */ /*!

      @brief           CD Object DistanceY Standard Deviation

      @description     CD Object DistanceY Standard Deviation

      @return          ALGO_INLINE float32

      @param[in]       pCDObjectData :
      @param[in]       iObjIndex :

      @pre             [None]
      @post            [None]
    *************************************************************************************************************************/
ALGO_INLINE float32 CD_OBJ_DISTY_STDDEV(const CDObjectData_t *pCDObjectData,
                                        const ObjNumber_t iObjIndex) {
    _PARAM_UNUSED(pCDObjectData);
    _PARAM_UNUSED(iObjIndex);
    return 0;
}

#endif /* _CD_INT_INCLUDED*/
