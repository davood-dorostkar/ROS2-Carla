/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "frame_sen_wrapper.h"
#include "frame_sen_custom_types.h"
//#include "vlc_par.h"

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/
/*****************************************************************************
  MACROS
*****************************************************************************/

/* Macro used to test 'statusFusion' field for camera confirmation */
#define STATUS_FUSION_CAM_CONF_MASK                                           \
    (EMB0_FUSION_SCAM_FRONT | EMB0_FUSION_MCAM_FRONT | EMB0_FUSION_CAM_FEND | \
     EMB0_FUSION_CAM_FEND | EMB0_FUSION_TOP_VIEW | EMB0_FUSION_CAM_REAR)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  CONSTS
*****************************************************************************/

/* definition of classification dependent minimum, maximum and default
 * dimensions */
/* class car minimum, maximum and default dimensions */
#define VLC_SEN_CLASS_CAR_MIN_LENGTH 3.0f
#define VLC_SEN_CLASS_CAR_MAX_LENGTH 7.0f
#define VLC_SEN_CLASS_CAR_DEFAULT_LENGTH 5.0f
#define VLC_SEN_CLASS_CAR_MIN_WIDTH 1.4f
#define VLC_SEN_CLASS_CAR_MAX_WIDTH 2.2f
#define VLC_SEN_CLASS_CAR_DEFAULT_WIDTH 1.8f
/* class truck minimum, maximum and default dimensions */
#define VLC_SEN_CLASS_TRUCK_MIN_LENGTH 5.0f
#define VLC_SEN_CLASS_TRUCK_MAX_LENGTH 25.0f
#define VLC_SEN_CLASS_TRUCK_DEFAULT_LENGTH 18.0f
#define VLC_SEN_CLASS_TRUCK_MIN_WIDTH 2.0f
#define VLC_SEN_CLASS_TRUCK_MAX_WIDTH 2.8f
#define VLC_SEN_CLASS_TRUCK_DEFAULT_WIDTH 2.5f
/* class pedestrian minimum, maximum and default dimensions length and width
 * treated same */
#define VLC_SEN_CLASS_PED_MIN_DIMENSION 0.4f
#define VLC_SEN_CLASS_PED_MAX_DIMENSION 0.8f
#define VLC_SEN_CLASS_PED_DEFAULT_DIMENSION 0.6f
/* class motorcycle minimum, maximum and default dimensions */
#define VLC_SEN_CLASS_MOTORCYCLE_MIN_LENGTH 2.0f
#define VLC_SEN_CLASS_MOTORCYCLE_MAX_LENGTH 4.0f
#define VLC_SEN_CLASS_MOTORCYCLE_DEFAULT_LENGTH 2.5f
#define VLC_SEN_CLASS_MOTORCYCLE_MIN_WIDTH 0.5f
#define VLC_SEN_CLASS_MOTORCYCLE_MAX_WIDTH 1.2f
#define VLC_SEN_CLASS_MOTORCYCLE_DEFAULT_WIDTH 1.0f
/* class bicile minimum, maximum and default dimensions */
#define VLC_SEN_CLASS_BICICLE_MIN_LENGTH VLC_SEN_CLASS_MOTORCYCLE_MIN_LENGTH
#define VLC_SEN_CLASS_BICICLE_MAX_LENGTH VLC_SEN_CLASS_MOTORCYCLE_MAX_LENGTH
#define VLC_SEN_CLASS_BICICLE_DEFAULT_LENGTH \
    VLC_SEN_CLASS_MOTORCYCLE_DEFAULT_LENGTH
#define VLC_SEN_CLASS_BICICLE_MIN_WIDTH VLC_SEN_CLASS_MOTORCYCLE_MIN_WIDTH
#define VLC_SEN_CLASS_BICICLE_MAX_WIDTH VLC_SEN_CLASS_MOTORCYCLE_MAX_WIDTH
#define VLC_SEN_CLASS_BICICLE_DEFAULT_WIDTH \
    VLC_SEN_CLASS_MOTORCYCLE_DEFAULT_WIDTH
/* class point minimum, maximum and default dimensions length and width treated
 * same */
#define VLC_SEN_CLASS_POINT_MIN_DIMENSION 0.1f
#define VLC_SEN_CLASS_POINT_MAX_DIMENSION 0.5f
#define VLC_SEN_CLASS_POINT_DEFAULT_DIMENSION 0.4f
/* class unclassified minimum, maximum and default dimensions length and width
 * treated same */
#define VLC_SEN_CLASS_UNCLASSIFIED_MIN_DIMENSION 0.1f
#define VLC_SEN_CLASS_UNCLASSIFIED_MAX_DIMENSION 50.0f
#define VLC_SEN_CLASS_UNCLASSIFIED_DEFAULT_DIMENSION 50.0f

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  PROTOTYPES
*****************************************************************************/
static void VLCInitCustomObject(VLCCustomObjectProperties_t *pCustomData);

/* FUNCTION: void VLCPreProcessObjectList(VLCObject_t *p_CustomData)
**************************************************************************
* Description: Function returns geometry of an object in length, width
* and orientation in pCustomData.Geometry
**************************************************************************
* PARAMETERS:
* p_CustomData    pointer to the VLCObject struct
* implicit to ObjectList via GET_Envm_GEN_OBJ
**************************************************************************
* RETURN VALUE :
* void
* output in p_CustomData.Geometry
*************************************************************************/
/*************************************************************************************************************************
  Functionname:    VLCPreProcessObjectList */
void VLCPreProcessObjectList(VLCPrivObjectList_t *pObjectList,
                             const reqVLCSenPrtList_t *pRequirePorts) {
    if ((pObjectList != NULL) && (pRequirePorts != NULL)) {
        /* declare and initialize counter variables */
        uint8 uiObject = 0u;

        /* pointer to CustomData */
        VLCCustomObjectProperties_t *p_CustomData = NULL;
        /* pointer to Geometry on EM-generic-objects */
        const Envm_t_GenObjAttributes *p_GenObjDataAttr = NULL;
        /* loop all objects in generic object list */
        for (uiObject = 0u; uiObject < Envm_N_OBJECTS; uiObject++) {
            const EM_t_ARSObject *const p_ArsObjData =
                &VLCSEN_pEmARSObjList->aObject[uiObject];
            /* set pointer to the Custom object */
            p_CustomData = &(VLCObjectList[uiObject].VLCCustomObjectProperties);
            /* set pointer to the EM-Generic objects */
            p_GenObjDataAttr = &(GET_Envm_GEN_OBJ(uiObject).Attributes);
            p_CustomData->Attributes.eDynamicProperty =
                p_GenObjDataAttr->eDynamicProperty;
            p_CustomData->Attributes.eObjectOcclusion =
                p_GenObjDataAttr->eObjectOcclusion;
            p_CustomData->Attributes.uiDynConfidence =
                p_GenObjDataAttr->uiDynConfidence;
            p_CustomData->Attributes.eClassification =
                p_GenObjDataAttr->eClassification;
            p_CustomData->Attributes.uiClassConfidence =
                p_GenObjDataAttr->uiClassConfidence;
            /* Initialize default geometry with data from ARS
             * technology-specific list */
            p_CustomData->Geometry = p_ArsObjData->Geometry;

            /* If pedestrian hyp cat bit is set (since that has cam conf
            drop-out work-arounds,
            then keep object as pedestrian) */
            if ((OBJ_GET_EBA_HYPOTHESIS_CATEGORY(uiObject) &
                 FPS_EBA_HYP_CAT_PED) != 0) {
                p_CustomData->Attributes.eClassification =
                    Envm_GEN_OBJECT_CLASS_PEDESTRIAN;
                p_CustomData->Attributes.uiClassConfidence = 100u;
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    VLCInitCustomObject */
static void VLCInitCustomObject(VLCCustomObjectProperties_t *pCustomData)

{
    pCustomData->Attributes.eDynamicProperty =
        Envm_GEN_OBJECT_DYN_PROPERTY_UNKNOWN;
    pCustomData->Attributes.uiDynConfidence = 0u;
    pCustomData->Attributes.eClassification =
        Envm_GEN_OBJECT_CLASS_UNCLASSIFIED;
    pCustomData->Attributes.uiClassConfidence = 0u;
    pCustomData->Attributes.eObjectOcclusion = Envm_GEN_OBJECT_OCCL_NONE;
}

/*************************************************************************************************************************
  Functionname:    VLCInitCustomObjectList */
void VLCInitCustomObjectList(void) {}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */