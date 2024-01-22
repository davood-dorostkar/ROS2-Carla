#ifndef LCFOPS_EXT_H
#define LCFOPS_EXT_H

#ifdef __cplusplus
extern "C" {
#endif
#include "TM_Global_Types.h"
#include "../LCCRA/LCCRA_ext.h"
#define LCF_NUM_FUSION_OBJECTS 100u
#define LCF_OPS_OBJECTS_NUM 40u

#ifndef Rte_TypeDef_LCF_FusionObjects_t
#define Rte_TypeDef_LCF_FusionObjects_t
typedef struct {
    sint32 iID_nu;                  // Object ID
    float32 fPosition_met[2];       // Object position, including x,u,z-axis
    float32 fVelocity_mps[2];       // Object velocity, including x,u,z-axis
    float32 fAcceleration_mps2[2];  // Object acceleration, including x,u,z-axis
    float32 fLength_met;            // Object length
    float32 fWidth_met;             // Object width
    float32 fHeadingAngle_rad;      // Object heading angle
    uint8 uClassification_nu;       // Object classification
    uint32 uSensorsource_nu;        // Object sensor source
    float32 fExistence_perc;        // Object existence
} LCF_FusionObjects_t;
#endif

#ifndef Rte_TypeDef_LCF_FusionObjectsArray_t
#define Rte_TypeDef_LCF_FusionObjectsArray_t
typedef struct {
    uint8 uNumTgtObj_nu;  // Number of valid fusion objects
    LCF_FusionObjects_t
        sFusionObjects[LCF_NUM_FUSION_OBJECTS];  // Fusion objects
} LCF_FusionObjectsArray_t;
#endif

/*typedef struct {
    uint8 uNumTgtObj_nu;  // Number of valid fusion objects
    LCF_FusionObjects_t sFusionObjects[LCF_OPS_OBJECTS_NUM];  // Fusion objects
} LCF_OPSFusionObjectsArray_t;*/
typedef struct {
    const LCF_FusionObjectsArray_t* inObjects;
    float32 fEgoCurve_1pm;
} sLCFOPSInReq_t;

typedef struct {
    float32
        LCFRCV_SysCycleTimeSen_sec;  // the cycle time of this module be invoked
} sLCFOPSParam_t;

typedef struct {
    BusObject outObjects;
} sLCFOPSOutPro_t;

typedef struct {
    uint32 uiVersionNum;

} sLCFOPSDebug_t;

void LCFOPS_Reset(void);
void LCFOPS_Exec(const sLCFOPSInReq_t* reqPorts,
                 const sLCFOPSParam_t* param,
                 sLCFOPSOutPro_t* proPorts,
                 sLCFOPSDebug_t* debug);
#ifdef __cplusplus
}
#endif
#endif