#pragma once
#ifndef LCFOPS_H
#define LCFOPS_H

#ifdef __cplusplus
extern "C" {
#endif
#include "lcfops_ext.h"
#include "tue_common_libs.h"
const float32 LCF_OPS_FRONT_ZONE_POSX_THRES_1 = 3.0f;
const float32 LCF_OPS_FRONT_ZONE_POSX_THRES_2 = 15.0f;
const float32 LCF_OPS_REAR_ZONE_POSX_THRES_1 = -3.0f;
const float32 LCF_OPS_REAR_ZONE_POSX_THRES_2 = -15.0f;
const float32 LCF_OPS_LEFT_ZONE_POSY_THRES = 3.0f;
const float32 LCF_OPS_RIGHT_ZONE_POSY_THRES = -3.0f;

const uint16 LCF_OPS_FRONT_QUALITY_THRES = 65u;
const uint16 LCF_OPS_REAR_QUALITY_THRES = 50u;
const uint16 LCF_OPS_SIDE_QUALITY_THRES = 50u;
const uint16 LCF_OPS_OTHER_QUALITY_THRES = 0u;

const float32 LCF_OPS_EGO_LANE_POSY_THRES = 3.0f;
const float32 LCF_OPS_OBJ_TTC_CAL_SPEEDX_THRES = 0.1f;
const float32 LCF_OPS_OBJ_MAX_TTC = 20.0f;
const float32 LCF_OPS_EGO_CURVE_THRES = 0.001f;

const uint16 LCF_OPS_OUTPUT_OBJECT_SIZE = LCF_OPS_OBJECTS_NUM;
const uint16 LCF_OPS_FRONT_OBJECT_SIZE = 10u;
const uint16 LCF_OPS_REAR_OBJECT_SIZE = 10u;
const uint16 LCF_OPS_SIDE_OBJECT_SIZE = 8u;
const uint16 LCF_OPS_OTHER_OBJECT_SIZE = 4u;

void ObjectZoneClassification(LCF_FusionObjectsArray_t InternObjList);
void ObjectSwap(sint16* Obj1, sint16* Obj2);
void ObjectSortInFrontZone(LCF_FusionObjectsArray_t InternObjList,
                           float32 fEgoCurve_1pm);
void ObjectSortInLeftZone(LCF_FusionObjectsArray_t InternObjList);
void ObjectSortInRightZone(LCF_FusionObjectsArray_t InternObjList);
void ObjectSortInRearZone(LCF_FusionObjectsArray_t InternObjList);
void ZoneResizeAndOutput(LCF_FusionObjectsArray_t InternObjList,
                         BusObject* outObjList);
void ConvertFusionOBjToBUsObj(LCF_FusionObjects_t inObj, BusObjPara* outObj);
#ifdef __cplusplus
}
#endif
#endif