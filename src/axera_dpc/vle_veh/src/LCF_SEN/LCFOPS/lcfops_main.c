#include "lcfops.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static sint16 FrontObjList[LCF_NUM_FUSION_OBJECTS];
static sint16 LeftObjList[LCF_NUM_FUSION_OBJECTS];
static sint16 RightObjList[LCF_NUM_FUSION_OBJECTS];
static sint16 RearObjList[LCF_NUM_FUSION_OBJECTS];
static sint16 OtherObjList[LCF_NUM_FUSION_OBJECTS];
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static uint8 FrontObjListIndex = 0;
static uint8 LeftObjListIndex = 0;
static uint8 RightObjListIndex = 0;
static uint8 RearObjListIndex = 0;
static uint8 OtherObjListIndex = 0;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

/*****************************************************************************
  Functionname:     LCFOPS_Reset                                  */
/*!

                                      @brief:           LCFOPS function reset

                                      @description:     All global variables
   related to LCFOPS are reset in this function when LCFOPS executes for the
   first time, or system exception needs to be reset

                                      @param[in]:       void

                                      @return:          void
                                *****************************************************************************/
void LCFOPS_Reset(void) {
    memset(FrontObjList, -1, sizeof(uint8) * LCF_NUM_FUSION_OBJECTS);
    memset(LeftObjList, -1, sizeof(uint8) * LCF_NUM_FUSION_OBJECTS);
    memset(RightObjList, -1, sizeof(uint8) * LCF_NUM_FUSION_OBJECTS);
    memset(RearObjList, -1, sizeof(uint8) * LCF_NUM_FUSION_OBJECTS);
    memset(OtherObjList, -1, sizeof(uint8) * LCF_NUM_FUSION_OBJECTS);

    FrontObjListIndex = 0;
    LeftObjListIndex = 0;
    RightObjListIndex = 0;
    RearObjListIndex = 0;
    OtherObjListIndex = 0;
}

/*****************************************************************************
  Functionname:     LCFOPS_Exec                                  */
/*!

                                        @brief:           Execution entry of
   LCFOPS function

                                        @description:     LCFOPS main function

                                        @param[in]:       reqPorts   LCFOPS
   input params     LCFOPS parameter input proPorts   LCFOPS output debugInfo
   LCFOPS debug information

                                        @return:void
                                      *****************************************************************************/
void LCFOPS_Exec(const sLCFOPSInReq_t* reqPorts,
                 const sLCFOPSParam_t* param,
                 sLCFOPSOutPro_t* proPorts,
                 sLCFOPSDebug_t* debug) {
    memset(FrontObjList, -1, sizeof(sint16) * LCF_NUM_FUSION_OBJECTS);
    memset(LeftObjList, -1, sizeof(sint16) * LCF_NUM_FUSION_OBJECTS);
    memset(RightObjList, -1, sizeof(sint16) * LCF_NUM_FUSION_OBJECTS);
    memset(RearObjList, -1, sizeof(sint16) * LCF_NUM_FUSION_OBJECTS);
    memset(OtherObjList, -1, sizeof(sint16) * LCF_NUM_FUSION_OBJECTS);

    if (reqPorts == NULL || param == NULL || proPorts == NULL ||
        debug == NULL || reqPorts->inObjects->uNumTgtObj_nu == 0u ||
        reqPorts->inObjects->uNumTgtObj_nu >= LCF_NUM_FUSION_OBJECTS) {
        return;
    }

    memset(proPorts, 0, sizeof(sLCFOPSOutPro_t));
    memset(debug, 0, sizeof(sLCFOPSDebug_t));

    FrontObjListIndex = 0;
    LeftObjListIndex = 0;
    RightObjListIndex = 0;
    RearObjListIndex = 0;
    OtherObjListIndex = 0;
    // for (uint8 i = 0; i < LCF_NUM_FUSION_OBJECTS; i++) {
    //     reqPorts->inObjects->sFusionObjects[i].fExistence_perc =
    //         reqPorts->inObjects->sFusionObjects[i].fExistence_perc * 100.f;
    // }
    ObjectZoneClassification(*reqPorts->inObjects);
    ObjectSortInFrontZone(*reqPorts->inObjects, reqPorts->fEgoCurve_1pm);
    ObjectSortInRearZone(*reqPorts->inObjects);
    ObjectSortInLeftZone(*reqPorts->inObjects);
    ObjectSortInRightZone(*reqPorts->inObjects);
    ZoneResizeAndOutput(*reqPorts->inObjects, &proPorts->outObjects);
}

/*****************************************************************************
  Functionname:     ObjectZoneClassification                                  */
/*!

                                        @brief:           calculate the related
zone of object based on it's position

                                        @description:fto

                                        @param[in]: LCF_FusionObjectsArray_t
InternObjList

                                        @return:void
*****************************************************************************/

void ObjectZoneClassification(LCF_FusionObjectsArray_t InternObjList) {
    for (int i = 0; i < InternObjList.uNumTgtObj_nu; i++) {
        LCF_FusionObjects_t InternObj = InternObjList.sFusionObjects[i];
        if (InternObj.fPosition_met[0] > LCF_OPS_FRONT_ZONE_POSX_THRES_2) {
            // object is in front and quite far away from ego vehicle.
            if (InternObj.fExistence_perc > LCF_OPS_FRONT_QUALITY_THRES) {
                FrontObjList[FrontObjListIndex] = i;
                FrontObjListIndex =
                    FrontObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                        ? FrontObjListIndex + 1
                        : LCF_NUM_FUSION_OBJECTS - 1;
            } else {
                OtherObjList[OtherObjListIndex] = i;
                OtherObjListIndex =
                    OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                        ? OtherObjListIndex + 1
                        : LCF_NUM_FUSION_OBJECTS - 1;
            }
        } else if (InternObj.fPosition_met[0] >
                   LCF_OPS_FRONT_ZONE_POSX_THRES_1) {
            // object is in front and near to ego vehicle.
            if (InternObj.fPosition_met[1] > LCF_OPS_RIGHT_ZONE_POSY_THRES &&
                InternObj.fPosition_met[1] < LCF_OPS_LEFT_ZONE_POSY_THRES) {
                // if the lateral distance is small and quality is satisfied,
                // the object can be classified to front zone.
                if (InternObj.fExistence_perc > LCF_OPS_FRONT_QUALITY_THRES) {
                    FrontObjList[FrontObjListIndex] = i;
                    FrontObjListIndex =
                        FrontObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? FrontObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                } else {
                    OtherObjList[OtherObjListIndex] = i;
                    OtherObjListIndex =
                        OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? OtherObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                }
            } else if (InternObj.fPosition_met[1] <=
                       LCF_OPS_RIGHT_ZONE_POSY_THRES) {
                // if the lateral distance is large and negative, the object can
                // be classified to right zone.
                if (InternObj.fExistence_perc > LCF_OPS_SIDE_QUALITY_THRES) {
                    RightObjList[RightObjListIndex] = i;
                    RightObjListIndex =
                        RightObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? RightObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                } else {
                    OtherObjList[OtherObjListIndex] = i;
                    OtherObjListIndex =
                        OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? OtherObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                }
            } else {
                // if the lateral distance is large and positive, the object can
                // be classified to right zone.
                if (InternObj.fExistence_perc > LCF_OPS_SIDE_QUALITY_THRES) {
                    LeftObjList[LeftObjListIndex] = i;
                    LeftObjListIndex =
                        LeftObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? LeftObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                } else {
                    OtherObjList[OtherObjListIndex] = i;
                    OtherObjListIndex =
                        OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? OtherObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                }
            }
        } else if (InternObj.fPosition_met[0] >
                   LCF_OPS_REAR_ZONE_POSX_THRES_1) {
            // object is very close to ego vehicle, that longitudinal distance
            // is between -3m to 3m
            if (InternObj.fPosition_met[1] <= LCF_OPS_RIGHT_ZONE_POSY_THRES) {
                // if the lateral distance is large and negative, the object can
                // be classified to right zone.
                if (InternObj.fExistence_perc > LCF_OPS_SIDE_QUALITY_THRES) {
                    RightObjList[RightObjListIndex] = i;
                    RightObjListIndex =
                        RightObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? RightObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                } else {
                    OtherObjList[OtherObjListIndex] = i;
                    OtherObjListIndex =
                        OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? OtherObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                }
            } else if (InternObj.fPosition_met[1] >=
                       LCF_OPS_LEFT_ZONE_POSY_THRES) {
                // if the lateral distance is large and positive, the object can
                // be classified to left zone.
                if (InternObj.fExistence_perc > LCF_OPS_SIDE_QUALITY_THRES) {
                    LeftObjList[LeftObjListIndex] = i;
                    LeftObjListIndex =
                        LeftObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? LeftObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                } else {
                    OtherObjList[OtherObjListIndex] = i;
                    OtherObjListIndex =
                        OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? OtherObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                }
            } else {
                OtherObjList[OtherObjListIndex] = i;
                OtherObjListIndex =
                    OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                        ? OtherObjListIndex + 1
                        : LCF_NUM_FUSION_OBJECTS - 1;
            }
        } else if (InternObj.fPosition_met[0] >
                   LCF_OPS_REAR_ZONE_POSX_THRES_2) {
            // object is behind and near to ego vehicle.
            if (InternObj.fPosition_met[1] > LCF_OPS_RIGHT_ZONE_POSY_THRES &&
                InternObj.fPosition_met[1] < LCF_OPS_LEFT_ZONE_POSY_THRES) {
                // if the lateral distance is small and quality is satisfied,
                // the object can be classified to rear zone.
                if (InternObj.fExistence_perc > LCF_OPS_REAR_QUALITY_THRES) {
                    RearObjList[RearObjListIndex] = i;
                    RearObjListIndex =
                        RearObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? RearObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                } else {
                    OtherObjList[OtherObjListIndex] = i;
                    OtherObjListIndex =
                        OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? OtherObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                }
            } else if (InternObj.fPosition_met[1] <=
                       LCF_OPS_RIGHT_ZONE_POSY_THRES) {
                // if the lateral distance is large and negative, the object can
                // be classified to right zone.
                if (InternObj.fExistence_perc > LCF_OPS_SIDE_QUALITY_THRES) {
                    RightObjList[RightObjListIndex] = i;
                    RightObjListIndex =
                        RightObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? RightObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                } else {
                    OtherObjList[OtherObjListIndex] = i;
                    OtherObjListIndex =
                        OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? OtherObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                }
            } else {
                // if the lateral distance is large and positive, the object can
                // be classified to left zone.
                if (InternObj.fExistence_perc > LCF_OPS_SIDE_QUALITY_THRES) {
                    LeftObjList[LeftObjListIndex] = i;
                    LeftObjListIndex =
                        LeftObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? LeftObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                } else {
                    OtherObjList[OtherObjListIndex] = i;
                    OtherObjListIndex =
                        OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                            ? OtherObjListIndex + 1
                            : LCF_NUM_FUSION_OBJECTS - 1;
                }
            }
        } else if (InternObj.fPosition_met[0] <=
                   LCF_OPS_REAR_ZONE_POSX_THRES_2) {
            // object is behind and quite far away from ego vehicle.
            if (InternObj.fExistence_perc > LCF_OPS_REAR_QUALITY_THRES) {
                RearObjList[RearObjListIndex] = i;
                RearObjListIndex = RearObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                                       ? RearObjListIndex + 1
                                       : LCF_NUM_FUSION_OBJECTS - 1;
            } else {
                OtherObjList[OtherObjListIndex] = i;
                OtherObjListIndex =
                    OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                        ? OtherObjListIndex + 1
                        : LCF_NUM_FUSION_OBJECTS - 1;
            }
        } else {
            OtherObjList[OtherObjListIndex] = i;
            OtherObjListIndex = OtherObjListIndex < LCF_NUM_FUSION_OBJECTS - 1
                                    ? OtherObjListIndex + 1
                                    : LCF_NUM_FUSION_OBJECTS - 1;
        }
    }
}

/*****************************************************************************
  Functionname:     ObjectSwap                                  */
/*!

                                        @brief:           swap value of obj1 and
obj2

                                        @description:

                                        @param[in]:

                                        @return:void
*****************************************************************************/
void ObjectSwap(sint16* Obj1, sint16* Obj2) {
    sint16 temp;
    temp = *Obj1;
    *Obj1 = *Obj2;
    *Obj2 = temp;
}

/*****************************************************************************
  Functionname:     ObjectSortInFrontZone                                  */
/*!

                                        @brief:           sort object priority
in front zone

                                        @description:

                                        @param[in]: LCF_FusionObjectsArray_t
InternObjList float32 fEgoCurve_1pm

                                        @return:void
*****************************************************************************/
void ObjectSortInFrontZone(LCF_FusionObjectsArray_t InternObjList,
                           float32 fEgoCurve_1pm) {
    float32 afPosToEgoCourseArray[LCF_NUM_FUSION_OBJECTS] = {0};
    float32 afPosOnEgoCourseArray[LCF_NUM_FUSION_OBJECTS] = {0};

    for (uint8 i = 0; i < FrontObjListIndex; i++) {
        float32 FrontObjPosX =
            InternObjList.sFusionObjects[FrontObjList[i]].fPosition_met[0];
        float32 FrontObjPosY =
            InternObjList.sFusionObjects[FrontObjList[i]].fPosition_met[1];
        // if the curvature of ego vehicle's predicted trajectory is larger than
        // threshold, the object's x-y coordinate value will be transform to
        // polar coordinate, else the object's x-y coordinate is equal to polar
        // coordinate. The distance between object to predicted ego trajectory
        // and the distance between object and ego vehicle on predicted ego
        // trajectory are calculated.
        if (fEgoCurve_1pm > LCF_OPS_EGO_CURVE_THRES) {
            float32 EgoRadius = 1 / fEgoCurve_1pm;
            float32 FrontObjRadius =
                SQRT(TUE_CML_Sqr(FrontObjPosX) +
                     TUE_CML_Sqr(FrontObjPosY - EgoRadius));
            afPosToEgoCourseArray[i] = EgoRadius - FrontObjRadius;
            afPosOnEgoCourseArray[i] =
                fABS(ATAN2_(FrontObjPosX, fABS(FrontObjPosY - EgoRadius)) *
                     EgoRadius);
        } else if (fEgoCurve_1pm < -LCF_OPS_EGO_CURVE_THRES) {
            float32 EgoRadius = 1 / fEgoCurve_1pm;
            float32 FrontObjRadius =
                SQRT(TUE_CML_Sqr(FrontObjPosX) +
                     TUE_CML_Sqr(FrontObjPosY - EgoRadius));
            afPosToEgoCourseArray[i] = EgoRadius + FrontObjRadius;
            afPosOnEgoCourseArray[i] =
                fABS(ATAN2_(FrontObjPosX, fABS(EgoRadius + FrontObjPosY)) *
                     EgoRadius);
        } else {
            afPosToEgoCourseArray[i] = FrontObjPosY;
            afPosOnEgoCourseArray[i] = FrontObjPosX;
        }
    }

    // bubble sort.
    for (uint8 i = 0; i < FrontObjListIndex - 1; i++) {
        uint8 count = 0;
        for (uint8 j = 1; j < FrontObjListIndex - i; j++) {
            float32 FrontObjToCourse_1 = fABS(afPosToEgoCourseArray[j]);
            float32 FrontObjOnCourse_1 = afPosOnEgoCourseArray[j];
            float32 FrontObjToCourse_2 = fABS(afPosToEgoCourseArray[j - 1]);
            float32 FrontObjOnCourse_2 = afPosOnEgoCourseArray[j - 1];

            if (FrontObjToCourse_1 < LCF_OPS_EGO_LANE_POSY_THRES &&
                FrontObjToCourse_2 < LCF_OPS_EGO_LANE_POSY_THRES) {
                // if the two objects are both very close to ego predicted
                // trajectory, they will be sorted by distance on the
                // trajectory, the closer one will be in front of the vector.
                if (FrontObjOnCourse_1 < FrontObjOnCourse_2) {
                    ObjectSwap(&FrontObjList[j], &FrontObjList[j - 1]);
                    count++;
                }
            } else if (FrontObjToCourse_1 < LCF_OPS_EGO_LANE_POSY_THRES &&
                       FrontObjToCourse_2 >= LCF_OPS_EGO_LANE_POSY_THRES) {
                // if one object is very close to ego predicted trajectory, the
                // other one is not, the closer one will be sorted in front.
                ObjectSwap(&FrontObjList[j], &FrontObjList[j - 1]);
                count++;
            } else if ((FrontObjToCourse_1 >= LCF_OPS_EGO_LANE_POSY_THRES &&
                        FrontObjToCourse_2 >= LCF_OPS_EGO_LANE_POSY_THRES)) {
                // if the two objects are both far away from ego predicted
                // trajectory, they will be sorted by distance to ego predicted
                // trajecory.
                if (FrontObjToCourse_1 < FrontObjToCourse_2) {
                    ObjectSwap(&FrontObjList[j], &FrontObjList[j - 1]);
                    count++;
                }
            }
        }
        if (count == 0) {
            break;
        }
    }
}

/*****************************************************************************
  Functionname:     ObjectSortInLeftZone                                  */
/*!

                                        @brief:           sort object priority
in left zone

                                        @description:

                                        @param[in]: LCF_FusionObjectsArray_t
InternObjList

                                        @return:void
*****************************************************************************/
void ObjectSortInLeftZone(LCF_FusionObjectsArray_t InternObjList) {
    for (uint8 i = 0; i < LeftObjListIndex - 1; i++) {
        uint8 count = 0;
        // bubble sort by lateral distance, the object closer to ego vehicle
        // will be sorted in front.
        for (uint8 j = 1; j < LeftObjListIndex - i; j++) {
            if (InternObjList.sFusionObjects[LeftObjList[j]].fPosition_met[1] <
                InternObjList.sFusionObjects[LeftObjList[j - 1]]
                    .fPosition_met[1]) {
                ObjectSwap(&LeftObjList[j], &LeftObjList[j - 1]);
                count++;
            }
        }
        if (count == 0) {
            break;
        }
    }
}

/*****************************************************************************
  Functionname:     ObjectSortInRightZone                                  */
/*!

                                        @brief:           sort object priority
in right zone

                                        @description:

                                        @param[in]: LCF_FusionObjectsArray_t
InternObjList

                                        @return:void
*****************************************************************************/
void ObjectSortInRightZone(LCF_FusionObjectsArray_t InternObjList) {
    for (uint8 i = 0; i < RightObjListIndex - 1; i++) {
        uint8 count = 0;
        // bubble sort by lateral distance, the object closer to ego vehicle
        // will be sorted in front.
        for (uint8 j = 1; j < RightObjListIndex - i; j++) {
            if (InternObjList.sFusionObjects[RightObjList[j]].fPosition_met[1] >
                InternObjList.sFusionObjects[RightObjList[j - 1]]
                    .fPosition_met[1]) {
                ObjectSwap(&RightObjList[j], &RightObjList[j - 1]);
                count++;
            }
        }
        if (count == 0) {
            break;
        }
    }
}

/*****************************************************************************
  Functionname:     ObjectSortInRearZone                                  */
/*!

                                        @brief:           sort object priority
in rear zone

                                        @description:

                                        @param[in]: LCF_FusionObjectsArray_t
InternObjList

                                        @return:void
*****************************************************************************/
void ObjectSortInRearZone(LCF_FusionObjectsArray_t InternObjList) {
    float32 afRearObjTTC[LCF_NUM_FUSION_OBJECTS] = {0};
    for (uint8 i = 0; i < RearObjListIndex; i++) {
        float32 RearObjRelSpeed =
            InternObjList.sFusionObjects[RearObjList[i]].fVelocity_mps[0];
        // calculate object's time to collision to ego vehicle, max value is
        // 20s.
        if (RearObjRelSpeed > LCF_OPS_OBJ_TTC_CAL_SPEEDX_THRES &&
            InternObjList.sFusionObjects[RearObjList[i]].fPosition_met[0] <
                0.0) {
            afRearObjTTC[i] =
                -InternObjList.sFusionObjects[RearObjList[i]].fPosition_met[0] /
                RearObjRelSpeed;
            if (afRearObjTTC[i] > LCF_OPS_OBJ_MAX_TTC) {
                afRearObjTTC[i] = LCF_OPS_OBJ_MAX_TTC;
            }
        } else {
            afRearObjTTC[i] = LCF_OPS_OBJ_MAX_TTC;
        }
    }

    for (uint8 i = 0; i < RearObjListIndex - 1; i++) {
        uint8 count = 0;
        for (uint8 j = 1; j < RearObjListIndex - i; j++) {
            // bubble sort by ttc, the object ttc is smaller will be sorted in
            // front.
            if (afRearObjTTC[j] < afRearObjTTC[j - 1]) {
                ObjectSwap(&RearObjList[j], &RearObjList[j - 1]);
                count++;
            }
        }
        if (count == 0) {
            break;
        }
    }
}

/*****************************************************************************
  Functionname:     ZoneResizeAndOutput                                  */
/*!

                                        @brief:           output object with the
size protection based on the sorted objects priority

                                        @description:

                                        @param[in]: LCF_FusionObjectsArray_t
InternObjList
                                        @param[out]: LCF_OPSFusionObjectsArray_t
*outObjList
                                        @return
*****************************************************************************/
void ZoneResizeAndOutput(LCF_FusionObjectsArray_t InternObjList,
                         BusObject* outObjList) {
    // if the amount of total objects list exceeds limit, the every zone object
    // list will be checked one by one, and delete the redundant object.
    while (FrontObjListIndex + RearObjListIndex + LeftObjListIndex +
               RightObjListIndex + OtherObjListIndex >
           LCF_OPS_OUTPUT_OBJECT_SIZE) {
        if (OtherObjListIndex > LCF_OPS_OTHER_OBJECT_SIZE) {
            OtherObjListIndex--;
        }
        if (FrontObjListIndex + RearObjListIndex + LeftObjListIndex +
                RightObjListIndex + OtherObjListIndex <=
            LCF_OPS_OUTPUT_OBJECT_SIZE) {
            break;
        }

        if (RearObjListIndex > LCF_OPS_REAR_OBJECT_SIZE) {
            RearObjListIndex--;
        }
        if (FrontObjListIndex + RearObjListIndex + LeftObjListIndex +
                RightObjListIndex + OtherObjListIndex <=
            LCF_OPS_OUTPUT_OBJECT_SIZE) {
            break;
        }

        if (LeftObjListIndex > LCF_OPS_SIDE_OBJECT_SIZE) {
            LeftObjListIndex--;
        }
        if (FrontObjListIndex + RearObjListIndex + LeftObjListIndex +
                RightObjListIndex + OtherObjListIndex <=
            LCF_OPS_OUTPUT_OBJECT_SIZE) {
            break;
        }

        if (RightObjListIndex > LCF_OPS_SIDE_OBJECT_SIZE) {
            RightObjListIndex--;
        }
        if (FrontObjListIndex + RearObjListIndex + LeftObjListIndex +
                RightObjListIndex + OtherObjListIndex <=
            LCF_OPS_OUTPUT_OBJECT_SIZE) {
            break;
        }

        if (FrontObjListIndex > LCF_OPS_FRONT_OBJECT_SIZE) {
            FrontObjListIndex--;
        }
    }

    // output object list
    for (uint8 i = 0; i < FrontObjListIndex; i++) {
        // FrontObjList[i].uiObjectZonePriority_nu = i;
        ConvertFusionOBjToBUsObj(
            InternObjList.sFusionObjects[FrontObjList[i]],
            &outObjList->Objects[MIN(outObjList->NumTgtObj,
                                     LCF_OPS_OBJECTS_NUM - 1)]);
        outObjList->NumTgtObj =
            MIN(outObjList->NumTgtObj + 1, LCF_OPS_OBJECTS_NUM);
    }

    for (uint8 i = 0; i < RearObjListIndex; i++) {
        ConvertFusionOBjToBUsObj(
            InternObjList.sFusionObjects[RearObjList[i]],
            &outObjList->Objects[MIN(outObjList->NumTgtObj,
                                     LCF_OPS_OBJECTS_NUM - 1)]);

        outObjList->NumTgtObj =
            MIN(outObjList->NumTgtObj + 1, LCF_OPS_OBJECTS_NUM);
    }

    for (uint8 i = 0; i < LeftObjListIndex; i++) {
        ConvertFusionOBjToBUsObj(
            InternObjList.sFusionObjects[LeftObjList[i]],
            &outObjList->Objects[MIN(outObjList->NumTgtObj,
                                     LCF_OPS_OBJECTS_NUM - 1)]);

        outObjList->NumTgtObj =
            MIN(outObjList->NumTgtObj + 1, LCF_OPS_OBJECTS_NUM);
    }

    for (uint8 i = 0; i < RightObjListIndex; i++) {
        ConvertFusionOBjToBUsObj(
            InternObjList.sFusionObjects[RightObjList[i]],
            &outObjList->Objects[MIN(outObjList->NumTgtObj,
                                     LCF_OPS_OBJECTS_NUM - 1)]);

        outObjList->NumTgtObj =
            MIN(outObjList->NumTgtObj + 1, LCF_OPS_OBJECTS_NUM);
    }

    for (uint8 i = 0; i < OtherObjListIndex; i++) {
        ConvertFusionOBjToBUsObj(
            InternObjList.sFusionObjects[OtherObjList[i]],
            &outObjList->Objects[MIN(outObjList->NumTgtObj,
                                     LCF_OPS_OBJECTS_NUM - 1)]);

        outObjList->NumTgtObj =
            MIN(outObjList->NumTgtObj + 1, LCF_OPS_OBJECTS_NUM);
    }
}

/*****************************************************************************
  Functionname:     ConvertFusionOBjToBUsObj                                  */
/*!

                                        @brief:

                                        @description:

                                        @param[in]: LCF_FusionObjectsArray_t
InternObjList
                                        @param[out]: LCF_OPSFusionObjectsArray_t
*outObjList
                                        @return
*****************************************************************************/
void ConvertFusionOBjToBUsObj(LCF_FusionObjects_t inObj, BusObjPara* outObj) {
    outObj->Acceleration[0] = inObj.fAcceleration_mps2[0];
    outObj->Acceleration[1] = inObj.fAcceleration_mps2[1];
    // inobj.uClassification_nu
    // 0: OBJCLASS_POINT; 1:OBJCLASS_CAR; 2:OBJCLASS_TRUCK;
    // 3:OBJCLASS_PEDESTRIAN; 4:OBJCLASS_MOTORCYCLE; 5:OBJCLASS_BICYCLE;
    // 6:OBJCLASS_WIDE; 7:OBJCLASS_UNCLASSIFIED; 8:OBJCLASS_CAM_CROSSING;
    // 9:OBJCLASS_CAM_GEN
    // outobj.Classification
    // 0: vechile; 1:Bus; 2:Pedestrain; 3:Bike; 4:UnknownMovable;
    // 5:UnknownStatic
    switch (inObj.uClassification_nu) {
        case 1:
            outObj->Classification = 0;
            break;
        case 2:
            outObj->Classification = 1;
            break;
        case 3:
            outObj->Classification = 2;
            break;
        case 4:
            outObj->Classification = 3;
            break;
        case 5:
            outObj->Classification = 3;
            break;
        default:
            outObj->Classification = 5;
    }
    outObj->Existence = inObj.fExistence_perc;
    outObj->HeadingAngle = inObj.fHeadingAngle_rad;
    outObj->ID = inObj.iID_nu;
    outObj->Length = inObj.fLength_met;
    outObj->Position[0] = inObj.fPosition_met[0];
    outObj->Position[1] = inObj.fPosition_met[1];
    outObj->Sensorsource = inObj.uSensorsource_nu;
    outObj->Velocity[0] = inObj.fVelocity_mps[0];
    outObj->Velocity[1] = inObj.fVelocity_mps[1];
    outObj->Width = inObj.fWidth_met;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */