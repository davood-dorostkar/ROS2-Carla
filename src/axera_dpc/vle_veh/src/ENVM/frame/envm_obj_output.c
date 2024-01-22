/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "TM_Global_Types.h"
#include "stddef.h"
#include "assert.h"
#include "ops_ext.h"
#include "ops_cfg.h"
#include "envm_io.h"
/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

#define EM_OBJ_OUT_MAX_QUADRANT 3
#define EM_OBJ_OUT_REQUIRED_ORIENTATION_STD (TUE_DEG2RAD(10.0f))

/*! percentage threshold for fully occluded object [0..100] */
#define EM_OBJ_OUT_OCCL_FULL_PERCENTAGE ((uint8)35)
/*! percentage threshold for partially occluded object [0..100] */
#define EM_OBJ_OUT_OCCL_PARTLY_PERCENTAGE ((uint8)0)

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
#if (CFG_Envm_GENERIC_OBJECT_INTERFACE_ACTIVE == 1)
/* local memory for index list used to copy internal to external list */
static Envm_t_ObjectPrioIndexArray aObjectPrioIndex;
/* local memory for index list used to copy internal to external list */
static Envm_t_ObjectPrioIndexArray aObjectPrioIndexRangeSorted;
#endif
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

#if (CFG_Envm_GENERIC_OBJECT_INTERFACE_ACTIVE == 1)
static void EM_v_InitObjOutput(void);
static void EM_v_ObjOutSetDefaultPrio(void);

static void EM_v_CopyPublic2GenObjectData(const Objects_t* p_EnvmPublicObject,
                                          Envm_t_GenObject* p_EMGenObject);
static void EM_v_CopyPublic2GenObjectKinematicData(
    const Objects_t* p_EnvmPublicObject,
    Envm_t_GenObjKinEnvmatics* p_GenObjectData);
static void EM_v_CopyPublic2GenObjectAttributeData(
    const Objects_t* p_EnvmPublicObject, Envm_t_GenObjAttributes* pAttributes);
static void EM_v_CopyPublic2GenObjClass(const Objects_t* p_EnvmPublicObject,
                                        Envm_t_GenObjAttributes* pAttributes);

static void EM_v_CopyPublic2GenObjectGeometryData(
    const Objects_t* p_EnvmPublicObject, EM_t_GenObjGeometry* p_EMGenObject);
static void EM_v_CopyPublic2GenObjectGeneralData(
    const Objects_t* p_EnvmPublicObject, EM_t_GenObjGenerals* p_EMGenObject);
static void SetMaintenanceStateBaseObjMaintenanceStateLocal(
    uint8 ObjMaintenanceStateLocal, uint8* pMaintenanceState);
static void EM_v_CopyPublic2GenObjectQualifierData(
    const Objects_t* p_EnvmPublicObject,
    Envm_t_GenObjQualifiers* p_EMGenObject);

static void EM_v_SetDefaultGenObjectData(Envm_t_GenObject* p_EMGenObject);
static void EM_v_CalcGenObjectAbsoluteKinematicData(
    const Objects_t* p_EnvmPublicObject,
    Envm_t_GenObjKinEnvmatics* p_GenObjectData);
static void EM_v_ProcessGenericObjectList(void);

static void EM_v_CopyPublic2TechObjectData(const Objects_t* p_EnvmPublicObject,
                                           EM_t_ARSObject* p_EMTechObject);
static void EM_v_CopyPublic2TechObjClass(const Objects_t* p_EnvmPublicObject,
                                         Envm_t_CR_Attributes* pTechAttributes);

static void EM_v_CopyPublic2TechObjectMotionData(
    const Objects_t* p_EnvmPublicObject,
    Envm_t_CR_MotionAttributes* p_MotionAttributes);
static void EM_v_SetDefaultTechObjectData(EM_t_ARSObject* p_EMTechObject);

#endif /* CFG_Envm_GENERIC_OBJECT_INTERFACE_ACTIVE */

/*****************************************************************************
  INPUT FUNCTION
*****************************************************************************/

/*****************************************************************************
  OUTPUT FUNCTION
*****************************************************************************/

#if (CFG_Envm_GENERIC_OBJECT_INTERFACE_ACTIVE == 1)

/* ****************************************************************************

  Functionname:     EM_v_InitObjOutput                                   */ /*!

      @brief            Reset and copy Output data from EM to RTE

      @description      Sets standard signal header for General and tech object
    data.
                        Sets defaults custom object data and override with heder
    data
                        from internal object data. Resets object list header,
    process all
                        objects and sets default object data.

      @return           void

      @pre              None
      @post             No changes


    ****************************************************************************
    */
static void EM_v_InitObjOutput(void) {
    /* get pointer to object list interface */
    Envm_t_GenObjectList* p_EMGenObjectList = GET_Envm_GEN_OBJ_DATA_PTR;

    uint32 ui_ObjIndexOut;

    Envm_t_CRObjectList* pEMTechObjectList = GET_Envm_TECH_OBJ_DATA_PTR;

    /* set standard header */
    Envm_v_SetSignalHeader(&(p_EMGenObjectList->sSigHeader));
    /* set standard header */
    Envm_v_SetSignalHeader(&(pEMTechObjectList->sSigHeader));

    /* copy object list header */
    p_EMGenObjectList->HeaderObjList.iNumOfUsedObjects = (sint8)0;

    /* process all objects */
    for (ui_ObjIndexOut = 0u; ui_ObjIndexOut < Envm_N_OBJECTS;
         ui_ObjIndexOut++) {
        EM_t_ARSObject* p_EMTechObject;
        Envm_t_GenObject* p_EMGenObject;
        /* get output object */
        p_EMGenObject = &(p_EMGenObjectList->aObject[ui_ObjIndexOut]);
        p_EMTechObject = &(pEMTechObjectList->aObject[ui_ObjIndexOut]);

        /* reset index */
        p_EMGenObjectList->HeaderObjList.iSortedObjectList[ui_ObjIndexOut] =
            OBJ_INDEX_NO_OBJECT;

        /* set defaults */
        EM_v_SetDefaultGenObjectData(p_EMGenObject);
        /* set defaults */
        EM_v_SetDefaultTechObjectData(p_EMTechObject);
    } /* for */
}

/* ****************************************************************************

  Functionname:     EM_v_ObjOutSetDefaultPrio                        */ /*!

          @brief            Fill object prioritization index array with default
        value
                            (take first Envm_N_OBJECTS objects)

          @description      This index array will be used to map a selection of
                            internal objects to the outside interface

          @pre              None
          @post             No changes

          @return           void


        ****************************************************************************
        */
static void EM_v_ObjOutSetDefaultPrio(void) {
    uint32 uiIndex;
    for (uiIndex = 0u; uiIndex < Envm_N_OBJECTS; uiIndex++) {
        aObjectPrioIndex[uiIndex] = (uint8)uiIndex;
        aObjectPrioIndexRangeSorted[uiIndex] = (uint8)uiIndex;
    }
}

/* ****************************************************************************

  Functionname:     Envm_v_ObjOutSetPrioIndexList                         */ /*!

     @brief            Fill object prioritization index array with values

     @description      This index array will be used to map a selection of
                       internal objects to the outside interface

     @param[in]        p_ObjectPrioIndex : Index array used to copy internal to
   external data
                                           Envm_t_GenObject[i] =
   Objects_t[p_ObjectPrioIndex[i]]

     @param[in]        p_ObjectPrioIndexRangeSorted : Index array used to copy
   internal to external data
                                           p_EMGenObjectList->HeaderObjList.iSortedObjectList[i]
                                           = aObjectPrioIndexRangeSorted[i]

     @pre              None
     @post             No changes

     @return           void


   ****************************************************************************
   */
void Envm_v_ObjOutSetPrioIndexList(
    const Envm_t_ObjectPrioIndexArray p_ObjectPrioIndex,
    const Envm_t_ObjectPrioIndexArray p_ObjectPrioIndexRangeSorted) {
    /* just copy index list to internal structure */
    if ((p_ObjectPrioIndex != NULL) && (p_ObjectPrioIndexRangeSorted != NULL)) {
        uint32 uiIndex;
        for (uiIndex = 0u; uiIndex < Envm_N_OBJECTS; uiIndex++) {
            aObjectPrioIndex[uiIndex] = p_ObjectPrioIndex[uiIndex];
            aObjectPrioIndexRangeSorted[uiIndex] =
                p_ObjectPrioIndexRangeSorted[uiIndex];
        }
    } else {
        /* if no proper input is given take default */
        EM_v_ObjOutSetDefaultPrio();
    }
}

/* ****************************************************************************

  Functionname:     EM_v_SetDefaultGenObjectData                        */ /*!

       @brief            Set default EM output interfaces

       @description      Initialize EM object data and reset ID, mergeID and
     splitID of the object

       @param[in]        p_EMGenObject : Pointer to output object data signal
     header structure

       @pre              None
       @post             No changes

       @return           void

     ****************************************************************************
     */
static void EM_v_SetDefaultGenObjectData(Envm_t_GenObject* p_EMGenObject) {
    (void)memset(p_EMGenObject, 0u, sizeof(Envm_t_GenObject));
    p_EMGenObject->General.uiID = Envm_INVALID_ID_INDEX;
    p_EMGenObject->General.uiMergeID = Envm_GEN_OBJECT_SM_ID_NONE;
    p_EMGenObject->General.uiSplitID = Envm_GEN_OBJECT_SM_ID_NONE;
}

/* ****************************************************************************

  Functionname:     EM_v_CopyPublic2GenObjectKinematicData                   */ /*!

  @brief            Fill object signals of EM output interfaces

  @description      Copy EM public object kinematic data to EM general object

  @param[in]        p_EnvmPublicObject : Pointer to input object data signal
structure
  @param[in]        p_GenObjectData : Pointer to output object data signal
structure

  @pre              None
  @post             No changes

  @return           void


**************************************************************************** */
static void EM_v_CopyPublic2GenObjectKinematicData(
    const Objects_t* p_EnvmPublicObject,
    Envm_t_GenObjKinEnvmatics* p_GenObjectData) {
    p_GenObjectData->fDistX = p_EnvmPublicObject->Kinematic.fDistX;
    p_GenObjectData->fDistXStd = p_EnvmPublicObject->Kinematic.fDistXStd;

    p_GenObjectData->fDistY = p_EnvmPublicObject->Kinematic.fDistY;
    p_GenObjectData->fDistYStd = p_EnvmPublicObject->Kinematic.fDistYStd;

    p_GenObjectData->fVrelX = p_EnvmPublicObject->Kinematic.fVrelX;
    p_GenObjectData->fVrelXStd = p_EnvmPublicObject->Kinematic.fVrelXStd;

    p_GenObjectData->fVrelY = p_EnvmPublicObject->Kinematic.fVrelY;
    p_GenObjectData->fVrelYStd = p_EnvmPublicObject->Kinematic.fVrelYStd;

    p_GenObjectData->fArelX = p_EnvmPublicObject->Kinematic.fArelX;
    p_GenObjectData->fArelXStd = p_EnvmPublicObject->Kinematic.fArelXStd;

    p_GenObjectData->fArelY = p_EnvmPublicObject->Kinematic.fArelY;
    p_GenObjectData->fArelYStd = p_EnvmPublicObject->Kinematic.fArelYStd;

    // copy abs velocity and acceleration
    p_GenObjectData->fVabsX = p_EnvmPublicObject->Kinematic.fVabsX;
    p_GenObjectData->fVabsXStd = p_EnvmPublicObject->Kinematic.fVabsXStd;

    p_GenObjectData->fVabsY = p_EnvmPublicObject->Kinematic.fVabsY;
    p_GenObjectData->fVabsYStd = p_EnvmPublicObject->Kinematic.fVabsYStd;

    p_GenObjectData->fAabsX = p_EnvmPublicObject->Kinematic.fAabsX;
    p_GenObjectData->fAabsXStd = p_EnvmPublicObject->Kinematic.fAabsXStd;

    p_GenObjectData->fAabsY = p_EnvmPublicObject->Kinematic.fAabsY;
    p_GenObjectData->fAabsYStd = p_EnvmPublicObject->Kinematic.fAabsYStd;
}

/* ****************************************************************************

  Functionname:     EM_v_CopyPublic2GenObjectKinematicData                   */ /*!

  @brief            Fill object signals of EM output interfaces

  @description      Copy EM public object Absolute kinematic data to EM general
object

  @param[in]        p_EnvmPublicObject : Pointer to input object data signal
structure
  @param[in]        p_GenObjectData : Pointer to output object data signal
structure

  @pre              None
  @post             No changes

  @return           void


**************************************************************************** */
static void EM_v_CalcGenObjectAbsoluteKinematicData(
    const Objects_t* p_EnvmPublicObject,
    Envm_t_GenObjKinEnvmatics* p_GenObjectData) {
    float32 fTemp;
    p_GenObjectData->fVabsX =
        p_EnvmPublicObject->Kinematic.fVrelX +
        EMTRAFO_f_GetObjSyncEgoMotionVx(p_EnvmPublicObject->Kinematic.fDistY);
    fTemp = SQR(p_EnvmPublicObject->Kinematic.fVrelXStd);
    fTemp += EM_f_GetEgoObjSyncVelXVar();
    p_GenObjectData->fVabsXStd = SQRT(fTemp);  // to be refined with YawRate

    p_GenObjectData->fVabsY =
        p_EnvmPublicObject->Kinematic.fVrelY +
        EMTRAFO_f_GetObjSyncEgoMotionVy(p_EnvmPublicObject->Kinematic.fDistX);
    p_GenObjectData->fVabsYStd =
        p_EnvmPublicObject->Kinematic.fVrelYStd;  // to be refined with YawRate

    p_GenObjectData->fAabsX =
        p_EnvmPublicObject->Kinematic.fArelX +
        EM_f_GetEgoObjSyncAccelX();  // to be refined with YawRate
    fTemp = SQR(p_EnvmPublicObject->Kinematic.fArelXStd);
    fTemp += EM_f_GetEgoObjSyncAccelXVar();
    p_GenObjectData->fAabsXStd = SQRT(fTemp);  // to be refined with YawRate

    p_GenObjectData->fAabsY =
        p_EnvmPublicObject->Kinematic.fArelY;  // to be refined with YawRate
    p_GenObjectData->fAabsYStd =
        p_EnvmPublicObject->Kinematic.fArelYStd;  // to be refined with YawRate
}

/* ****************************************************************************

  Functionname:     EM_v_CopyPublic2GenObjectGeometryData                   */ /*!

   @brief            Fill object signals of EM output interfaces

   @description      Copy EM public object Geometry data to EM general object

   @param[in]        p_EnvmPublicObject : Pointer to input object data signal
 structure
   @param[in]        p_EMGenObject : Pointer to output object data signal
 structure

   @pre              None
   @post             No changes

   @return           void


 **************************************************************************** */
static void EM_v_CopyPublic2GenObjectGeometryData(
    const Objects_t* p_EnvmPublicObject, EM_t_GenObjGeometry* p_EMGenObject) {
    Envm_t_GenObjShapePointCoord *pLeftFrontPoint, *pLeftFarPoint,
        *pRightFarPoint, *pRightFrontPoint;
    uint8 IdxLeftFrontPoint, IdxLeftFarPoint, IdxRightFarPoint,
        IdxRightFrontPoint;
    uint8 u_Back = 0u, u_Left = 1u, u_Front = 2u;
    /*u_Right = 3u;*/
    const Geometry_t* p_GeometryInput = &(p_EnvmPublicObject->Geometry);
    float32 f_Orientation =
        EMSIZE_f_GetDeltaOrienation2RefPoint(p_GeometryInput->fOrientation, 0);

    /* calculate sinus and cosine just once */
    float32 f_SinPhi, f_CosPhi;
    if (f_Orientation > -TUE_PRV_FUSION_MATH_PI_HALF &&
        f_Orientation < TUE_PRV_FUSION_MATH_PI_HALF) {
        f_SinPhi = SIN_(f_Orientation);
        f_CosPhi = COS_(f_Orientation);
    } else {
        f_SinPhi = SIN_(TUE_PRV_FUSION_MATH_PI + f_Orientation);
        f_CosPhi = COS_(TUE_PRV_FUSION_MATH_PI + f_Orientation);
    }

    float32 f_Width_Cos_Phi = p_GeometryInput->fWidth * f_CosPhi,
            f_Length_Cos_Phi = p_GeometryInput->fLength * f_CosPhi,
            f_Width_Sin_Phi = p_GeometryInput->fWidth * f_SinPhi,
            f_Length_Sin_Phi = p_GeometryInput->fLength * f_SinPhi;

    /* define indexes of shape points */
    IdxLeftFrontPoint = 0U;
    IdxLeftFarPoint = 1U;
    IdxRightFarPoint = 2U;
    IdxRightFrontPoint = 3U;
    pLeftFrontPoint =
        &(p_EMGenObject->aShapePointCoordinates[IdxLeftFrontPoint]);
    pLeftFarPoint = &(p_EMGenObject->aShapePointCoordinates[IdxLeftFarPoint]);
    pRightFarPoint = &(p_EMGenObject->aShapePointCoordinates[IdxRightFarPoint]);
    pRightFrontPoint =
        &(p_EMGenObject->aShapePointCoordinates[IdxRightFrontPoint]);

    /*Quandrant change adaptation from refpoint Oreintation*/

    if (0 == u_Back) {
        f_Width_Sin_Phi = 0.5f * f_Width_Sin_Phi;
        f_Width_Cos_Phi = 0.5f * f_Width_Cos_Phi;

        /* get rotated rect x positions */
        pLeftFrontPoint->fPosX = -f_Width_Sin_Phi;
        pRightFrontPoint->fPosX = f_Width_Sin_Phi;
        pRightFarPoint->fPosX = f_Width_Sin_Phi + f_Length_Cos_Phi;
        pLeftFarPoint->fPosX = -f_Width_Sin_Phi + f_Length_Cos_Phi;

        /* get rotated rect y positions */
        pLeftFrontPoint->fPosY = f_Width_Cos_Phi;
        pRightFrontPoint->fPosY = -f_Width_Cos_Phi;
        pRightFarPoint->fPosY = -f_Width_Cos_Phi + f_Length_Sin_Phi;
        pLeftFarPoint->fPosY = f_Width_Cos_Phi + f_Length_Sin_Phi;
    } else if (0 == u_Left) {
        f_Length_Sin_Phi = 0.5f * f_Length_Sin_Phi;
        f_Length_Cos_Phi = 0.5f * f_Length_Cos_Phi;

        /* get rotated rect x positions */
        pLeftFrontPoint->fPosX = f_Length_Sin_Phi;
        pRightFrontPoint->fPosX = f_Length_Sin_Phi + f_Width_Cos_Phi;
        pRightFarPoint->fPosX = -f_Length_Sin_Phi + f_Width_Cos_Phi;
        pLeftFarPoint->fPosX = -f_Length_Sin_Phi;

        /* get rotated rect y positions */
        pLeftFrontPoint->fPosY = -f_Length_Cos_Phi;
        pRightFrontPoint->fPosY = -f_Length_Cos_Phi + f_Width_Sin_Phi;
        pRightFarPoint->fPosY = f_Length_Cos_Phi + f_Width_Sin_Phi;
        pLeftFarPoint->fPosY = f_Length_Cos_Phi;

    } else if (0 == u_Front) {
        f_Width_Sin_Phi = 0.5f * f_Width_Sin_Phi;
        f_Width_Cos_Phi = 0.5f * f_Width_Cos_Phi;

        /* get rotated rect x positions */
        pLeftFrontPoint->fPosX = f_Width_Sin_Phi + f_Length_Cos_Phi;
        pRightFrontPoint->fPosX = -f_Width_Sin_Phi + f_Length_Cos_Phi;
        pRightFarPoint->fPosX = -f_Width_Sin_Phi;
        pLeftFarPoint->fPosX = f_Width_Sin_Phi;

        /* get rotated rect y positions */
        pLeftFrontPoint->fPosY = -f_Width_Cos_Phi + f_Length_Sin_Phi;
        pRightFrontPoint->fPosY = f_Width_Cos_Phi + f_Length_Sin_Phi;
        pRightFarPoint->fPosY = f_Width_Cos_Phi;
        pLeftFarPoint->fPosY = -f_Width_Cos_Phi;
    } else {
        /* if(p_GeometryInput->eRefPointPos == u_Right) */
        f_Length_Sin_Phi = 0.5f * f_Length_Sin_Phi;
        f_Length_Cos_Phi = 0.5f * f_Length_Cos_Phi;

        /* get rotated rect x positions */
        pLeftFrontPoint->fPosX = -f_Length_Sin_Phi + f_Width_Cos_Phi;
        pRightFrontPoint->fPosX = -f_Length_Sin_Phi;
        pRightFarPoint->fPosX = f_Length_Sin_Phi;
        pLeftFarPoint->fPosX = f_Length_Sin_Phi + f_Width_Cos_Phi;

        /* get rotated rect y positions */
        pLeftFrontPoint->fPosY = f_Length_Cos_Phi + f_Width_Sin_Phi;
        pRightFrontPoint->fPosY = f_Length_Cos_Phi;
        pRightFarPoint->fPosY = -f_Length_Cos_Phi;
        pLeftFarPoint->fPosY = -f_Length_Cos_Phi + f_Width_Sin_Phi;
    }
}

/*************************************************************************************************************************
  Functionname:    EM_v_CopyPublic2GenObjClass */
static void EM_v_CopyPublic2GenObjClass(const Objects_t* p_EnvmPublicObject,
                                        Envm_t_GenObjAttributes* pAttributes) {
    /* map object classification */
    switch (p_EnvmPublicObject->Attributes.eClassification) {
        case OBJCLASS_POINT:
            pAttributes->eClassification = Envm_GEN_OBJECT_CLASS_POINT;
            break;
        case OBJCLASS_CAR:
            pAttributes->eClassification = Envm_GEN_OBJECT_CLASS_CAR;
            break;
        case OBJCLASS_TRUCK:
            pAttributes->eClassification = Envm_GEN_OBJECT_CLASS_TRUCK;
            break;
        case OBJCLASS_PEDESTRIAN:
            pAttributes->eClassification = Envm_GEN_OBJECT_CLASS_PEDESTRIAN;
            break;
        case OBJCLASS_MOTORCYCLE:
            pAttributes->eClassification = Envm_GEN_OBJECT_CLASS_MOTORCYCLE;
            break;
        case OBJCLASS_BICYCLE:
            pAttributes->eClassification = Envm_GEN_OBJECT_CLASS_BICYCLE;
            break;
        case OBJCLASS_WIDE:
            pAttributes->eClassification = Envm_GEN_OBJECT_CLASS_WIDE;
            break;
        case OBJCLASS_UNCLASSIFIED:
            pAttributes->eClassification = Envm_GEN_OBJECT_CLASS_UNCLASSIFIED;
            break;
        default:
            pAttributes->eClassification = Envm_GEN_OBJECT_CLASS_UNCLASSIFIED;
            break;
    }
    /* set confidence (be aware that currently two class confidences are
     * existing (one in attributes and one in legacy) */
    pAttributes->uiClassConfidence =
        p_EnvmPublicObject->Attributes.uiClassConfidence;

    /* set objects, that are classified as belonging to guardrail, to POINT
     * independent from base class */
    /* in CR3xx, this reclassification was already done in the base
     * classification algo itself (leading to changes in object tracking) */
    /* now, to avoid influencing the tracking, but to have the information
     * available in FCT, do the re-classification here */
    /* todo: consider introducing a class GUARDRAIL */

    return;
}

/* ****************************************************************************

  Functionname:     EM_v_CopyPublic2GenObjectAttributeData                   */ /*!

  @brief            Fill object signals of EM output interfaces

  @description      Copy EM public object Attribute data to EM general object.
                    Sets dynamic property of object based on its moving state
and Orientation.
                    Maps confidence of dynamic property and maps object
classification.
                    and object occlusion.

  @param[in]        p_EnvmPublicObject : Pointer to input object data signal
structure
  @param[in]        pAttributes : Pointer to output object data signal structure

  @pre              None
  @post             No changes

  @return           void


**************************************************************************** */
static void EM_v_CopyPublic2GenObjectAttributeData(
    const Objects_t* p_EnvmPublicObject, Envm_t_GenObjAttributes* pAttributes) {
    float32 f_Orientation = p_EnvmPublicObject->Geometry.fOrientation;

    switch (p_EnvmPublicObject->Attributes.eAbsMovingState) {
        case OBJECT_MOVSTATE_STATIONARY:
            pAttributes->eDynamicProperty =
                Envm_GEN_OBJECT_DYN_PROPERTY_STATIONARY;
            break;
        case OBJECT_MOVSTATE_STOPPED:
            pAttributes->eDynamicProperty =
                Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED;
            break;
        case OBJECT_MOVSTATE_MOVING: {
            float32 fOrientationDeg;
            fOrientationDeg = RAD2DEG(f_Orientation);

            if ((-45.0f < fOrientationDeg) && (fOrientationDeg <= 45.0f)) {
                /* moving (N) */
                pAttributes->eDynamicProperty =
                    Envm_GEN_OBJECT_DYN_PROPERTY_MOVING;
            } else if ((-225.0f < fOrientationDeg) &&
                       (fOrientationDeg <= -135.0f)) {
                /* oncoming (SSE) */
                pAttributes->eDynamicProperty =
                    Envm_GEN_OBJECT_DYN_PROPERTY_ONCOMING;
            } else if ((135.0f < fOrientationDeg) &&
                       (fOrientationDeg <= 225.0f)) {
                /* oncoming (SSW) */
                pAttributes->eDynamicProperty =
                    Envm_GEN_OBJECT_DYN_PROPERTY_ONCOMING;
            } else if ((45.0f < fOrientationDeg) &&
                       (fOrientationDeg <= 135.0f)) {
                /* crossing to left (W) */
                pAttributes->eDynamicProperty =
                    Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_RIGHT;
            } else if ((-135.0f < fOrientationDeg) &&
                       (fOrientationDeg <= -45.0f)) {
                /* crossing to right (E) */
                pAttributes->eDynamicProperty =
                    Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_LEFT;
            } else {
                pAttributes->eDynamicProperty =
                    Envm_GEN_OBJECT_DYN_PROPERTY_UNKNOWN;
            }
            break;
        }
        default:
            pAttributes->eDynamicProperty =
                Envm_GEN_OBJECT_DYN_PROPERTY_UNKNOWN;
            break;
    }

    /* map confidence */
    pAttributes->uiDynConfidence = (ui8_t)(100.0f);

    /*map class and class confidence atrributes*/
    EM_v_CopyPublic2GenObjClass(p_EnvmPublicObject, pAttributes);

#ifdef _MSC_VER /* Microsoft compiler -> code only for simulation */

#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): Check if this is the desired mapping for eObjectOcclusion")
#endif

    pAttributes->eObjectOcclusion = Envm_GEN_OBJECT_OCCL_NONE;
}

/* ****************************************************************************

  Functionname:     EM_v_CopyPublic2GenObjectGeneralData                   */ /*!

    @brief            Fill object signals of EM output interfaces

    @description      Copy EM public object general data to EM general object.
                      Sets maintenance state based on uiD, FCT obj ID last
  cycle.
                      Also sets merge ID based on public object merging ID, FCT
  merge ID and
                      maintenance state. Sets sensor source of general object
  depending
                      on measured sources.

    @param[in]        p_EnvmPublicObject : Pointer to input object data signal
  structure
    @param[in]        p_EMGenObject : Pointer to output object data signal
  structure

    @pre              None
    @post             No changes

    @return           void


  ****************************************************************************
  */
sint8 FPS_i_GetFCTObjIDLastCycle(uint32 const ui_Index);

static void EM_v_CopyPublic2GenObjectGeneralData(
    const Objects_t* p_EnvmPublicObject, EM_t_GenObjGenerals* p_EMGenObject) {
    sint8 s_ObjectID;

    sint8 i_FCTObjIDLastCycle;
    sint8 EMObjIDMergePointer;
    uint8 e_ObjMaintenanceStateLocal;

    p_EMGenObject->fLifeTime = p_EnvmPublicObject->General.fLifeTime;
    p_EMGenObject->uiLifeCycles = p_EnvmPublicObject->Legacy.uiLifeTime;
    if (p_EnvmPublicObject->General.fTimeStamp <= 65535u) {
    } else {
#ifdef _MSC_VER /* Microsoft compiler -> code only for simulation */

#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): Define propper handling of data uiLastMeasuredTimeStamp")
#endif
    }

    /*! Set maintenance state */
    e_ObjMaintenanceStateLocal =
        p_EnvmPublicObject->General.eObjMaintenanceState;

    i_FCTObjIDLastCycle = FPS_i_GetFCTObjIDLastCycle(
        p_EMGenObject->uiID); /*!< FCT-ID of object in the last cycle */

    /*! If object was not selected for FCT in last cycle and if it is selected
     * in this cycle -> Maintenance state becomes NEW */
    if ((GET_FCT_OBJ_ID(p_EMGenObject->uiID) != OBJ_INDEX_NO_OBJECT) &&
        (i_FCTObjIDLastCycle == OBJ_INDEX_NO_OBJECT)) {
        e_ObjMaintenanceStateLocal = MT_STATE_NEW;
        /*! Remark: handling of MT_STATE_MERGE_NEW is implicitly included: If
           the object was not selected in the last cycle for FCT, the
           maintenance shall change from MT_STATE_MERGE_NEW to MT_STATE_NEW */
    } else if ((e_ObjMaintenanceStateLocal == MT_STATE_MERGE_NEW) &&
               (GET_FCT_OBJ_ID(p_EMGenObject->uiID) !=
                OBJ_INDEX_NO_OBJECT) &&  //!< Object is selected for FCT in this
                                         // cycle
               (GET_FCT_OBJ_ID(p_EnvmPublicObject->General.cObjMergingID) ==
                OBJ_INDEX_NO_OBJECT)  //!< Object into which the object is
                                      //!< merged
                                      // (cObjMergingID) is not selected for FCT

    ) {
        /*! If the object into which a object is merged (cObjMergingID) is not
         * selected for FCT, MT_STATE_MERGE_NEW becomes MT_STATE_NEW */
        e_ObjMaintenanceStateLocal = MT_STATE_NEW;
    } else if (GET_FCT_OBJ_ID(p_EMGenObject->uiID) == OBJ_INDEX_NO_OBJECT) {
        /*! If an object is not selected for FCT, set the state to
         * MT_STATE_DELETED */
        e_ObjMaintenanceStateLocal = MT_STATE_DELETED;
    } else {
        /*! Nothing */
    }

    /*! Set uiSplitMergeID / MT_STATE_MERGE_NEW. As uiSplitMergeID, the FCT-ID
     * is used. */
    p_EMGenObject->uiMergeID =
        Envm_GEN_OBJECT_SM_ID_UNKNOWN;  //!< Set default: Envm_INVALID_ID_INDEX

    if (((e_ObjMaintenanceStateLocal == MT_STATE_MERGE_NEW) ||
         (e_ObjMaintenanceStateLocal == MT_STATE_MERGE_DELETED)) &&
        (GET_FCT_OBJ_ID(p_EnvmPublicObject->General.cObjMergingID) !=
         OBJ_INDEX_NO_OBJECT)) {
        // For the MT_STATE_MERGE_NEW and MT_STATE_MERGE_DELETED case, set the
        // uiSplitMergeID (if this object, into which the object is merged, is
        // selected for FCT)
        /***************************************************/

        /***************************************************/
        s_ObjectID = GET_FCT_OBJ_ID(p_EnvmPublicObject->General.cObjMergingID);
        p_EMGenObject->uiMergeID =
            (s_ObjectID < 0) ? 255u : (uint8)(s_ObjectID);
    } else if ((e_ObjMaintenanceStateLocal == MT_STATE_NEW)) {
        /*! Special handling of MT_STATE_NEW */
        /*! Get EMObjIDMergePointer (EM-ID): Merge pointer tells that the former
           object on this position was merged into this object. If no merge,
           EMObjIDRef = OBJ_INDEX_NO_OBJECT */
        if (p_EMGenObject->uiID < Envm_NR_PRIVOBJECTS) {
            EMObjIDMergePointer =
                FPSGetIDRefToMerge(GET_FCT_OBJ_ID(p_EMGenObject->uiID),
                                   (sint8)p_EMGenObject->uiID);
        } else {
            /* error: object ID is to big */
            EMErrorTrap(__FILE__, __LINE__, EM_ERRORTRAP_TYPE_ERROR);
            EMObjIDMergePointer = OBJ_INDEX_NO_OBJECT;
        }
        /*! If merge took place, EMObjIDRef is a valid ID. For a merge that is
         * relevant in FCT, the object into which was merged must be selected
         * for FCT */
        if ((EMObjIDMergePointer > OBJ_INDEX_NO_OBJECT) &&
            (EMObjIDMergePointer < Envm_NR_PRIVOBJECTS) &&
            (GET_FCT_OBJ_ID(EMObjIDMergePointer) > OBJ_INDEX_NO_OBJECT)) {
            /*! Set the maintenance state to MT_STATE_MERGE_NEW */
            e_ObjMaintenanceStateLocal = MT_STATE_MERGE_NEW;
            /*! Set uiSplitMergeID */
            /**********************************************************/

            /**********************************************************/
            s_ObjectID = GET_FCT_OBJ_ID(EMObjIDMergePointer);
            p_EMGenObject->uiMergeID =
                (s_ObjectID < 0) ? 255u : (uint8)(s_ObjectID);
            /*! Remark: Special handing directly in FCT if the merged object and
               the new object, into which the object was merged, have the same
               FCT-ID (based on the info uiSplitMergeID == FCT-ID) */
        }
    } else {
        /*! Nothing */
    }

    /*! Remark: uiID is set in function EM_v_ProcessGenericObjectList */

    /*! Do the mapping of the ENUM from EM internally to the generic object list
     */
    SetMaintenanceStateBaseObjMaintenanceStateLocal(
        e_ObjMaintenanceStateLocal, &(p_EMGenObject->eMaintenanceState));
}
static void SetMaintenanceStateBaseObjMaintenanceStateLocal(
    uint8 ObjMaintenanceStateLocal, uint8* pMaintenanceState) {
    switch (ObjMaintenanceStateLocal) {
        case MT_STATE_DELETED:
            *pMaintenanceState = Envm_GEN_OBJECT_MT_STATE_DELETED;
            break;
        case MT_STATE_NEW:
            *pMaintenanceState = Envm_GEN_OBJECT_MT_STATE_NEW;
            break;
        case MT_STATE_MEASURED:
            *pMaintenanceState = Envm_GEN_OBJECT_MT_STATE_MEASURED;
            break;
        case MT_STATE_PREDICTED:
            *pMaintenanceState = Envm_GEN_OBJECT_MT_STATE_PREDICTED;
            break;
        case MT_STATE_MERGE_DELETED:
            *pMaintenanceState = Envm_GEN_OBJECT_MT_STATE_DELETED;
            break;
        case MT_STATE_MERGE_NEW:
            *pMaintenanceState = Envm_GEN_OBJECT_MT_STATE_NEW;
            break;

#ifdef _MSC_VER /* Microsoft compiler -> code only for simulation */

#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): Define propper mapping for EM_GEN_OBJECT_MT_STATE_SPLIT_*")
#endif

        default:
            *pMaintenanceState = Envm_GEN_OBJECT_MT_STATE_DELETED;
            break;
    }
}

/* ****************************************************************************

  Functionname:     EM_v_CopyPublic2GenObjectQualifierData                   */ /*!

  @brief            Fill object signals of EM output interfaces

  @description      Copy EM public object Qualifier data to EM general object.

  @param[in]        p_EnvmPublicObject : Pointer to input object data signal
structure
  @param[in]        p_EMGenObject : Pointer to output object data signal
structure

  @pre              None
  @post             No changes

  @return           void

**************************************************************************** */

static void EM_v_CopyPublic2GenObjectQualifierData(
    const Objects_t* p_EnvmPublicObject,
    Envm_t_GenObjQualifiers* p_EMGenObject) {
    float32 fPoE = p_EnvmPublicObject->Qualifiers.fProbabilityOfExistence;
    const float32 fConversionFactor = 100.0F;

    if ((0 <= fPoE) && (fPoE <= 1.0f)) {
        uint32 uiPoE;
        uiPoE = ROUND_TO_UINT(fConversionFactor * fPoE);
        p_EMGenObject->uiProbabilityOfExistence = (uint8)uiPoE;
    } else {
        p_EMGenObject->uiProbabilityOfExistence = 0u;
    }

    /* copy qualities */
    p_EMGenObject->uiAccObjQuality =
        p_EnvmPublicObject->ACCPresel.ucAccObjQuality;

    p_EMGenObject->uiEbaObjQuality =
        p_EnvmPublicObject->EBAPresel.ucEbaMovingObjQuality;

    /* copy HypCat and InhibitionMask attributes */
    p_EMGenObject->eEbaHypCat = p_EnvmPublicObject->EBAPresel.eEbaHypCat;
    p_EMGenObject->eEbaInhibitionMask =
        p_EnvmPublicObject->EBAPresel.eEbaInhibitionMask;
}

/* ****************************************************************************

  Functionname:     EM_v_CopyPublic2GenObjectData                        */ /*!

      @brief            Copy EM public object data to EM general object

      @description      Copy Kinematic data, absolute kinematic values, Geometry
    data,
                        Attribute data, General Data and qualifier data of EM
    public object
                        to General object.

      @param[in]        p_EnvmPublicObject : Pointer to input object data signal
    header structure
      @param[in]        p_EMGenObject : Pointer to output object data signal
    header structure

      @pre              None
      @post             No changes

      @return           void


    ****************************************************************************
    */
static void EM_v_CopyPublic2GenObjectData(const Objects_t* p_EnvmPublicObject,
                                          Envm_t_GenObject* p_EMGenObject) {
    p_EMGenObject->General.eMaintenanceState = Envm_GEN_OBJECT_MT_STATE_NEW;

    /* Envm_t_GenObjKinEnvmatics */
    /*************************/
    EM_v_CopyPublic2GenObjectKinematicData(p_EnvmPublicObject,
                                           &(p_EMGenObject->Kinematic));

    /* calculate absolute kinematic values */
    /***************************************/
    // EM_v_CalcGenObjectAbsoluteKinematicData(p_EnvmPublicObject,
    //                                         &(p_EMGenObject->Kinematic));

    /* EM_t_GenObjGeometry */
    /***********************/
    EM_v_CopyPublic2GenObjectGeometryData(p_EnvmPublicObject,
                                          &(p_EMGenObject->Geometry));

    /* Envm_t_GenObjAttributes */
    /*************************/
    EM_v_CopyPublic2GenObjectAttributeData(p_EnvmPublicObject,
                                           &(p_EMGenObject->Attributes));

    /* EM_t_GenObjGenerals */
    /***********************/
    EM_v_CopyPublic2GenObjectGeneralData(p_EnvmPublicObject,
                                         &(p_EMGenObject->General));

    /* Envm_t_GenObjQualifiers */
    /*************************/

    EM_v_CopyPublic2GenObjectQualifierData(p_EnvmPublicObject,
                                           &(p_EMGenObject->Qualifiers));
}

/* ****************************************************************************

  Functionname:     EM_v_ProcessGenericObjectList                        */ /*!

      @brief            Fill EM generic output object data to RTE interface

      @description      Sets signal header for general, custom and tech object
    data.
                        Process all Objects by getting index for prio list,
    takes
                        sorted array of objects from interface layer. Maps only
    valid
                        internal objects and ignores others. For valid internal
    objects,
                        increases number of valid processed objects, gets input
    object,
                        converst internal object data to general object and tech
    object data.
                        If internal index is invalid sets default general,
    object and custom
                        object data.

      @pre              None
      @post             No changes

      @return           void

    ****************************************************************************
    */
static void EM_v_ProcessGenericObjectList(void) {
    sint8 s_ObjectID;

    /* get pointer to object list interface */
    ObjectList_t* p_EnvmPublicObjectList = GET_Envm_INT_OBJ_DATA_PTR;
    Envm_t_GenObjectList* p_EMGenObjectList = GET_Envm_GEN_OBJ_DATA_PTR;

    uint32 ui_ObjIndexOut;

    Envm_t_CRObjectList* pEMTechObjectList = GET_Envm_TECH_OBJ_DATA_PTR;

    /* set standard header */
    Envm_v_SetSignalHeader(&(p_EMGenObjectList->sSigHeader));
    /* set standard header */
    Envm_v_SetSignalHeader(&(pEMTechObjectList->sSigHeader));

    /* copy object list header */
    p_EMGenObjectList->HeaderObjList.iNumOfUsedObjects = (sint8)0;

    /* process all objects */
    for (ui_ObjIndexOut = 0u; ui_ObjIndexOut < Envm_N_OBJECTS;
         ui_ObjIndexOut++) {
        EM_t_ARSObject* p_EMTechObject;
        Envm_t_GenObject* p_EMGenObject;
        uint8 ui8_Index;
        uint32 ui_ObjIndexIntern;
        /* get index for prio list */
        ui_ObjIndexIntern = aObjectPrioIndex[ui_ObjIndexOut];

        // Take sorted array from interface layer (aObjectPrioIndexRangeSorted
        // contains the FCT object ID) */
        ui8_Index = aObjectPrioIndexRangeSorted[ui_ObjIndexOut];
        // aObjectPrioIndexRangeSorted[ui_ObjIndexIntern];
        if (ui8_Index < (uint8)Envm_N_OBJECTS) {
            p_EMGenObjectList->HeaderObjList.iSortedObjectList[ui_ObjIndexOut] =
                (sint8)ui8_Index;
        } else {
            p_EMGenObjectList->HeaderObjList.iSortedObjectList[ui_ObjIndexOut] =
                OBJ_INDEX_NO_OBJECT;
        }

        /* get output object */
        p_EMGenObject = &(p_EMGenObjectList->aObject[ui_ObjIndexOut]);
        p_EMTechObject = &(pEMTechObjectList->aObject[ui_ObjIndexOut]);

        /* can only map valid internal objects, others are ignored */
        if (ui_ObjIndexIntern < Envm_NR_PRIVOBJECTS) {
            if (EM_INT_OBJ_IS_DELETED(ui_ObjIndexIntern)) {
                /* set defaults */
                EM_v_SetDefaultGenObjectData(p_EMGenObject);

                // If the object contains information about merged objects,
                // this information must be taken over to FCT (only if the
                // object, into which the ui_ObjIndexIntern is merged, exists in
                // the FCT object list) */
                if ((EM_INT_OBJ_MAINTENANCE_STATE(ui_ObjIndexIntern) ==
                     MT_STATE_MERGE_DELETED) &&
                    (GET_FCT_OBJ_ID(
                         p_EnvmPublicObjectList->Objects[ui_ObjIndexIntern]
                             .General.cObjMergingID) != OBJ_INDEX_NO_OBJECT)) {
                    // p_EMGenObject->General.uiID = (uint8)ui_ObjIndexIntern;
                    p_EMGenObject->General.uiID =
                        p_EnvmPublicObjectList->Objects[ui_ObjIndexIntern]
                            .ObjectId;
                    p_EMGenObject->General.eMaintenanceState =
                        Envm_GEN_OBJECT_MT_STATE_DELETED;

                    /******************************************/

                    /******************************************/
                    s_ObjectID = GET_FCT_OBJ_ID(
                        p_EnvmPublicObjectList->Objects[ui_ObjIndexIntern]
                            .General.cObjMergingID);
                    p_EMGenObject->General.uiMergeID =
                        (s_ObjectID < 0) ? 255u : (uint8)(s_ObjectID);
                }
                /* set defaults */
                EM_v_SetDefaultTechObjectData(p_EMTechObject);

            } else {
                Objects_t* p_EnvmPublicObject;
                /* increase number of valid processed objects */
                p_EMGenObjectList->HeaderObjList.iNumOfUsedObjects++;

                /* get input object */
                p_EnvmPublicObject =
                    &(p_EnvmPublicObjectList->Objects[ui_ObjIndexIntern]);

                /* the index was used as ID in the internal list */
                // p_EMGenObject->General.uiID = (uint8)ui_ObjIndexIntern;
                p_EMGenObject->General.uiID = p_EnvmPublicObject->ObjectId;

                EM_v_CopyPublic2GenObjectData(p_EnvmPublicObject,
                                              p_EMGenObject);

                EM_v_CopyPublic2TechObjectData(p_EnvmPublicObject,
                                               p_EMTechObject);
            }

        } else {
            /* internal index is invalid */
            /* set defaults */
            EM_v_SetDefaultGenObjectData(p_EMGenObject);
            /* set defaults */
            EM_v_SetDefaultTechObjectData(p_EMTechObject);
        }
    } /* for */
}

/* ****************************************************************************

  Functionname:     EM_v_SetDefaultGenObjectData                        */ /*!

       @brief            Set default object data in tech list

       @description      Set default object data in tech list

       @param[in]        p_EMTechObject : Pointer to output object data signal
     header structure

       @pre              None
       @post             No changes

       @return           void


     ****************************************************************************
     */
static void EM_v_SetDefaultTechObjectData(EM_t_ARSObject* p_EMTechObject) {
    (void)memset(p_EMTechObject, 0u, sizeof(EM_t_ARSObject));
}

/* ****************************************************************************

  Functionname:     EM_v_CopyPublic2TechObjectMotionData */ /*!

                      @brief            Fill signal header for EM output
                    interfaces

                      @description      Fills motion attributes i.e, dynamic
                    property, stopped confidence,
                                        dynamic sub property and Absolute moving
                    state of EM output Tech Data.

                      @param[in]        p_EnvmPublicObject : Pointer to input
                    object data signal structure
                      @param[in]        p_MotionAttributes : Pointer to output
                    object data signal structure

                      @pre              None
                      @post             No changes

                      @return           void


                    ****************************************************************************
                    */
static void EM_v_CopyPublic2TechObjectMotionData(
    const Objects_t* p_EnvmPublicObject,
    Envm_t_CR_MotionAttributes* p_MotionAttributes) {
    /* Envm_t_CR_MotionAttributes */
    /*****************************/

    switch (p_EnvmPublicObject->Attributes.eDynamicProperty) {
        case OBJECT_PROPERTY_MOVING:
            p_MotionAttributes->eDynamicProperty = CR_OBJECT_PROPERTY_MOVING;
            break;
        case OBJECT_PROPERTY_STATIONARY:
            p_MotionAttributes->eDynamicProperty =
                CR_OBJECT_PROPERTY_STATIONARY;
            break;
        case OBJECT_PROPERTY_ONCOMING:
            p_MotionAttributes->eDynamicProperty = CR_OBJECT_PROPERTY_ONCOMING;
            break;
        default:
#ifdef _MSC_VER /* Microsoft compiler -> code only for simulation */

#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): TODO: Define ARS_OBJECT_PROPERTY_SNA")
#endif
            p_MotionAttributes->eDynamicProperty = CR_OBJECT_PROPERTY_MOVING;
            break;
    }

    /* copy StoppedConfidence */
    p_MotionAttributes->uiStoppedConfidence =
        p_EnvmPublicObject->Attributes.uiStoppedConfidence;

    /* map state for dynamic sub property */
    switch (p_EnvmPublicObject->Legacy.eDynamicSubProperty) {
        case GDB_OBJECT_SUBPROP_NORMAL:
            p_MotionAttributes->eDynamicSubProperty = CR_OBJECT_SUBPROP_UNIFAL;
            break;
        case GDB_OBJECT_SUBPROP_CROSSING:
            p_MotionAttributes->eDynamicSubProperty =
                CR_OBJECT_SUBPROP_CROSSING;
            break;
        default:
#ifdef _MSC_VER /* Microsoft compiler -> code only for simulation */

#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): TODO: Define ARS_OBJECT_SUBPROP_SNA")
#endif
            p_MotionAttributes->eDynamicSubProperty = CR_OBJECT_SUBPROP_UNIFAL;
            break;
    }

    /* map state for motion over ground (to be used in combination with the
     * orientation) */
    switch (p_EnvmPublicObject->Attributes.eAbsMovingState) {
        case OBJECT_MOVSTATE_STATIONARY:
            p_MotionAttributes->eAbsMovingState = CR_OBJECT_MOVSTATE_STATIONARY;
            break;
        case OBJECT_MOVSTATE_STOPPED:
            p_MotionAttributes->eAbsMovingState = CR_OBJECT_MOVSTATE_STOPPED;
            break;
        case OBJECT_MOVSTATE_MOVING:
            p_MotionAttributes->eAbsMovingState = CR_OBJECT_MOVSTATE_MOVING;
            break;
        default:
#ifdef _MSC_VER /* Microsoft compiler -> code only for simulation */

#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): TODO: Define ARS_OBJECT_MOVSTATE_SNA")
#endif
            p_MotionAttributes->eAbsMovingState = CR_OBJECT_MOVSTATE_MOVING;
            break;
    }
}

/* ****************************************************************************

Functionname:     EM_v_CopyPublic2TechObjectData                        */ /*!

         @brief            Fill signal header for EM output interfaces

         @description      Fills Tech object data from public object. Fills
       Kinematic, Geometry,
                           motion attributes, ARS Attributes, sensor specific
       and legacy members
                           of Tech object.

         @param[in]        p_EnvmPublicObject : Pointer to input object data
       signal header structure
         @param[in]        p_EMTechObject : Pointer to output object data signal
       header structure

         @pre              None
         @post             No changes

         @return           void

       ****************************************************************************
       */
// levi 2018-12-07 added here
/* Object is at the farscan beam border (left side) */
#define OD_BEAMPOS_FAR_L (1.0F)
/* Object is at the farscan beam border (right side) */
#define OD_BEAMPOS_FAR_R (-1.0F)
/* Object is at the nearscan beam border (left side) */
#define OD_BEAMPOS_NEAR_L (2.0F)
/* Object is at the nearscan beam border (right side) */
#define OD_BEAMPOS_NEAR_R (-2.0F)
// levi end

static void EM_v_CopyPublic2TechObjectData(const Objects_t* p_EnvmPublicObject,
                                           EM_t_ARSObject* p_EMTechObject) {
    // float32 fAbsAngularBeamPos;
    // static const float32 fFOVBoundaryMargin = 0.9F;
    // static const float32 fBeamPos = OD_BEAMPOS_NEAR_L;

    Envm_t_CR_MotionAttributes* p_MotionAttributes =
        &(p_EMTechObject->MotionAttributes);

    /* Envm_t_CR_KinEnvmatic */
    /**********************/
    p_EMTechObject->Kinematic.fMaxAccelY =
        p_EnvmPublicObject->Legacy.fMaxAccelY;

    /* Envm_t_CR_Geometry */
    /*********************/
    p_EMTechObject->Geometry.fLength = p_EnvmPublicObject->Geometry.fLength;
    p_EMTechObject->Geometry.fWidth = p_EnvmPublicObject->Geometry.fWidth;
    p_EMTechObject->Geometry.fOrientation =
        p_EnvmPublicObject->Geometry.fOrientation;
    p_EMTechObject->Geometry.fOrientationStd =
        p_EnvmPublicObject->Geometry.fOrientationStd;

    /* Envm_t_CR_MotionAttributes */
    /*****************************/
    EM_v_CopyPublic2TechObjectMotionData(p_EnvmPublicObject,
                                         p_MotionAttributes);

    /* Envm_t_CR_Attributes */
    /***********************/

    /*Copy the class and class confidence information*/
    EM_v_CopyPublic2TechObjClass(p_EnvmPublicObject,
                                 &(p_EMTechObject->Attributes));

    p_EMTechObject->Attributes.uiReferenceToTrace = 0;

    /* Envm_t_CR_SensorSpecific */
    /***************************/

    p_EMTechObject->SensorSpecific.fRCS =
        p_EnvmPublicObject->SensorSpecific.fRCS;

    p_EMTechObject->SensorSpecific.ucMeasuredSources = CR_MEAS_SEN_NECRCAN;
    p_EMTechObject->SensorSpecific.eObjRelationsClass =
        CR_LONGVEHICLE_TYPE_REAL;

    p_EMTechObject->Legacy.fAngle =
        RAD2DEG(p_EMTechObject->Geometry.fOrientation);
    p_EMTechObject->Legacy.fLastTargetDistX = 0.0f;
    p_EMTechObject->Legacy.fLastTargetDistY = 0.0f;

    /* Envm_t_CR_Legacy */
    /*******************/
}

/*************************************************************************************************************************
  Functionname:    EM_v_CopyPublic2TechObjClass */
static void EM_v_CopyPublic2TechObjClass(
    const Objects_t* p_EnvmPublicObject,
    Envm_t_CR_Attributes* pTechAttributes) {
    pTechAttributes->uiClassConfidence =
        p_EnvmPublicObject->Attributes.uiClassConfidence;

    switch (p_EnvmPublicObject->Attributes.eClassification) {
        case OBJCLASS_POINT:
            pTechAttributes->eClassification = CR_OBJCLASS_POINT;
            break;
        case OBJCLASS_CAR:
            pTechAttributes->eClassification = CR_OBJCLASS_CAR;
            break;
        case OBJCLASS_TRUCK:
            pTechAttributes->eClassification = CR_OBJCLASS_TRUCK;
            break;
        case OBJCLASS_PEDESTRIAN:
            pTechAttributes->eClassification = CR_OBJCLASS_PEDESTRIAN;
            break;
        case OBJCLASS_MOTORCYCLE:
            pTechAttributes->eClassification = CR_OBJCLASS_MOTORCYCLE;
            break;
        case OBJCLASS_BICYCLE:
            pTechAttributes->eClassification = CR_OBJCLASS_BICYCLE;
            break;
        case OBJCLASS_WIDE:
            pTechAttributes->eClassification = CR_OBJCLASS_WIDE;
            break;
        case OBJCLASS_UNCLASSIFIED:
            pTechAttributes->eClassification = CR_OBJCLASS_UNCLASSIFIED;
            break;
        default:
            pTechAttributes->eClassification = CR_OBJCLASS_UNCLASSIFIED;
            break;
    }

    /* set objects, that are classified as belonging to guardrail, to POINT
     * independent from base class */
    /* in CR3xx, this reclassification was already done in the base
     * classification algo itself (leading to changes in object tracking) */
    /* now, to avoid influencing the tracking, but to have the information
     * available in FCT, do the re-classification here */
    /* todo: consider introducing a class GUARDRAIL */

    return;
}

/* ****************************************************************************

  Functionname:     EMProcessObjOutput                                   */ /*!

      @brief            Copy Output data from EM to RTE

      @description      Fills EM generic output object data to RTE interface.
                        And Resets Output data from EM to RTE if signal status
    is
                        not OK.

      @return           void

      @pre              None
      @post             No changes


    ****************************************************************************
    */
void EMProcessObjOutput(void) {
    ObjectList_t* p_EnvmPublicObjectList = GET_Envm_INT_OBJ_DATA_PTR;

    if (p_EnvmPublicObjectList->eSigStatus == AL_SIG_STATE_OK) {
        /* Fill EM generic and technology specific output object data to RTE
         * interface  */
        EM_v_ProcessGenericObjectList();
    } else {
        EM_v_InitObjOutput();
    }
}

#endif /* CFG_Envm_GENERIC_OBJECT_INTERFACE_ACTIVE */

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */