#pragma once
#if 1
#ifndef ALGO_GLOB_H_INCLUDED
#define ALGO_GLOB_H_INCLUDED

#ifdef PRQA_SIZE_T

#pragma PRQA_MACRO_MESSAGES_OFF "_PARAM_UNUSED" 3112
#endif 

#define UINT16_MAX_LIMIT 65535

#define ALN_f_AngleDeviationFar_AngleMin (BML_Deg2Rad(-22.f))

#define ALN_f_AngleDeviationNear_AngleMin (BML_Deg2Rad(-90.f))

#define ALN_f_AngleDeviationFar_AngleInc  ( - (2.f * ALN_f_AngleDeviationFar_AngleMin ) / (float32)(ALN_ANGLE_DEV_FAR_NOF_ANGLES-1) )

#define ALN_f_AngleDeviationNear_AngleInc ( - (2.f * ALN_f_AngleDeviationNear_AngleMin) / (float32)(ALN_ANGLE_DEV_NEAR_NOF_ANGLES-1) )

#define STRASSENBREITE       (3.75F)

#define HALBESTRASSENBREITE  (STRASSENBREITE/2.0F)

#define FAHRZEUGBREITE       (2.2F)

#define HALBEFAHRZEUGBREITE  (FAHRZEUGBREITE/2.F)

#define ALGO_StringInternalQuote(x)			#x

#define ALGO_StringQuote(x) 						ALGO_StringInternalQuote(x)

#define ALGO_f_ConvCyclesToLifetime(c)	(c)

#define ALGO_f_ConvTimeToLifeTime(t)		((t) * (1.f/ALGO_f_RadarCycleTime))

#define ALGO_f_ConvLifetimeToTime(lt)   ((lt) * ALGO_f_RadarCycleTime)

#define ALGO_f_ConvLifetimeToCycles(lt) (lt)

#define GET_Envm_GEN_OBJ(iObj)                      GET_Envm_PUB_OBJ_DATA_PTR->aObject[iObj]

#define OBJ_KINEMATIC(iObj)                       GET_Envm_GEN_OBJ(iObj).Kinematic

#define OBJ_GENERAL(iObj)                         GET_Envm_GEN_OBJ(iObj).General

#define OBJ_ATTRIBUTES(iObj)                      GET_Envm_GEN_OBJ(iObj).Attributes

#define OBJ_QUALIFIERS(iObj)                      GET_Envm_GEN_OBJ(iObj).Qualifiers

#define OBJ_GEOMETRY(iObj)                        GET_Envm_GEN_OBJ(iObj).Geometry

#define GET_EM_ARS_OBJ_PUB(iObj)                  GET_EM_ARS_OBJ_LIST_PTR->aObject[iObj]

#define OBJ_CR_KINEnvmATICS(iObj)                  GET_EM_ARS_OBJ_PUB(iObj).Kinematic
#define OBJ_CR_GEOMETRY(iObj)                    GET_EM_ARS_OBJ_PUB(iObj).Geometry
#define OBJ_ARS_MOTIONATTRIBUTES(iObj)            GET_EM_ARS_OBJ_PUB(iObj).MotionAttributes
#define OBJ_CR_ATTRIBUTES(iObj)                  GET_EM_ARS_OBJ_PUB(iObj).Attributes
#define OBJ_CR_SENSORSPECIFIC(iObj)              GET_EM_ARS_OBJ_PUB(iObj).SensorSpecific
#define OBJ_ARS_LEGACY(iObj)                      GET_EM_ARS_OBJ_PUB(iObj).Legacy

#define OBJ_LONG_DISPLACEMENT(iObj)               OBJ_KINEMATIC(iObj).fDistX
#define OBJ_LONG_VREL(iObj)                       OBJ_KINEMATIC(iObj).fVrelX
#define OBJ_LONG_AREL(iObj)                       OBJ_KINEMATIC(iObj).fArelX

#define OBJ_LAT_DISPLACEMENT(iObj)                OBJ_KINEMATIC(iObj).fDistY
#define OBJ_LAT_VREL(iObj)                        OBJ_KINEMATIC(iObj).fVrelY
#define OBJ_LAT_AREL(iObj)                        OBJ_KINEMATIC(iObj).fArelY

#define OBJ_LONG_DISPLACEMENT_VAR(iObj)           SQR(OBJ_KINEMATIC(iObj).fDistXStd)
#define OBJ_LONG_DISPLACEMENT_STD(iObj)           OBJ_KINEMATIC(iObj).fDistXStd
#define OBJ_LONG_VREL_VAR(iObj)                   SQR(OBJ_KINEMATIC(iObj).fVrelXStd)
#define OBJ_LONG_VREL_STD(iObj)                   OBJ_KINEMATIC(iObj).fVrelXStd
#define OBJ_LONG_AREL_VAR(iObj)                   SQR(OBJ_KINEMATIC(iObj).fArelXStd)
#define OBJ_LONG_AREL_STD(iObj)                   OBJ_KINEMATIC(iObj).fArelXStd

#define OBJ_LAT_DISPLACEMENT_VAR(iObj)            SQR(OBJ_KINEMATIC(iObj).fDistYStd)
#define OBJ_LAT_DISPLACEMENT_STD(iObj)            OBJ_KINEMATIC(iObj).fDistYStd
#define OBJ_LAT_VREL_VAR(iObj)                    SQR(OBJ_KINEMATIC(iObj).fVrelYStd)
#define OBJ_LAT_VREL_STD(iObj)                    OBJ_KINEMATIC(iObj).fVrelYStd
#define OBJ_LAT_AREL_VAR(iObj)                    SQR(OBJ_KINEMATIC(iObj).fArelYStd)
#define OBJ_LAT_AREL_STD(iObj)                    OBJ_KINEMATIC(iObj).fArelYStd

#define Envm_OBJ_INDEX_DISTX_SORTED                 GET_Envm_PUB_OBJ_DATA_PTR->HeaderObjList.iSortedObjectList

#define OBJ_GET_ID_I(iObj)                        iObj

#define OBJ_NUMBER_OF_OBJ_USED                    GET_Envm_PUB_OBJ_DATA_PTR->HeaderObjList.iNumOfUsedObjects

#define OBJ_LIFETIME_SEC(iObj)                    OBJ_GENERAL(iObj).fLifeTime
#define OBJ_LIFECYCLES(iObj)                      OBJ_GENERAL(iObj).uiLifeCycles

#define OBJ_MAINTENANCE_STATE(iObj)               (OBJ_GENERAL(iObj).eMaintenanceState)
#define OBJ_IS_DELETED(iObj)                      (OBJ_GENERAL(iObj).eMaintenanceState == Envm_GEN_OBJECT_MT_STATE_DELETED)
#define OBJ_IS_NEW(iObj)                          (OBJ_GENERAL(iObj).eMaintenanceState == Envm_GEN_OBJECT_MT_STATE_NEW)

#define OBJ_KALMAN_MAX_ACCEL_Y(iObj)              OBJ_CR_KINEnvmATICS(iObj).fMaxAccelY

#define OBJ_TGT_LAST_DISTANCE(iObj)               OBJ_ARS_LEGACY(iObj).fLastTargetDistX
#define OBJ_TGT_LAST_HORZ_DISPLACEMENT(iObj)      OBJ_ARS_LEGACY(iObj).fLastTargetDistY

#define OBJ_ANGLE(iObj)                           OBJ_ARS_LEGACY(iObj).fAngle

#define OBJ_TRACE_ID(iObj)                        OBJ_CR_ATTRIBUTES(iObj).uiReferenceToTrace

#define OBJ_DYNAMIC_PROPERTY(iObj)                OBJ_ARS_MOTIONATTRIBUTES(iObj).eDynamicProperty
#define OBJ_DYNAMIC_SUB_PROPERTY(iObj)            OBJ_ARS_MOTIONATTRIBUTES(iObj).eDynamicSubProperty
#define OBJ_CLASSIFICATION(iObj)                  OBJ_CR_ATTRIBUTES(iObj).eClassification
#define OBJ_MOVING_TO_STATIONARY(iObj)            OBJ_ARS_MOTIONATTRIBUTES(iObj).uiStoppedConfidence

#define OBJ_IS_MOVING_TO_STATIONARY(iObj)         (OBJ_ARS_MOTIONATTRIBUTES(iObj).uiStoppedConfidence > 80U)

#define OBJ_ORIENTATION(iObj)                     OBJ_CR_GEOMETRY(iObj).fOrientation
#define OBJ_ORIENTATION_STD(iObj)                 OBJ_CR_GEOMETRY(iObj).fOrientationStd
#define OT_GET_OBJ_WIDTH(iObj)                    OBJ_CR_GEOMETRY(iObj).fWidth
#define OT_GET_OBJ_LENGTH(iObj)                   OBJ_CR_GEOMETRY(iObj).fLength
#define OBJ_CLASS_CONFIDENCE_PERCENT(iObj)        OBJ_CR_ATTRIBUTES(iObj).uiClassConfidence

#define OBJ_RCS(iObj)                             OBJ_CR_SENSORSPECIFIC(iObj).fRCS

#define OBJ_LONG_VEHICLE_TYPE(iObj)               OBJ_CR_SENSORSPECIFIC(iObj).eObjRelationsClass
#define OBJ_IS_NORMAL(iObj)                       ((OBJ_LONG_VEHICLE_TYPE(iObj) == CR_LONGVEHICLE_TYPE_UNIFAL) ? TRUE : FALSE)
#define OBJ_IS_SHADOW(iObj)                       (((OBJ_LONG_VEHICLE_TYPE(iObj) == CR_LONGVEHICLE_TYPE_SHADOW) || (OBJ_LONG_VEHICLE_TYPE(iObj) == CR_LONGVEHICLE_TYPE_MIDDLE)) ? TRUE : FALSE)
#define OBJ_IS_MIDDLE(iObj)                       ((OBJ_LONG_VEHICLE_TYPE(iObj) == CR_LONGVEHICLE_TYPE_MIDDLE) ? TRUE : FALSE)
#define OBJ_IS_REAL(iObj)                         ((OBJ_LONG_VEHICLE_TYPE(iObj) == CR_LONGVEHICLE_TYPE_REAL) ? TRUE : FALSE)

#define OBJ_IS_MEASURED(iObj, ucMask)             ((OBJ_CR_SENSORSPECIFIC(iObj).ucMeasuredSources & (ucMask)) ? TRUE : FALSE)

#define OBJ_GET_FOV_OVERLAP_FAR(iObj)             OBJ_CR_SENSORSPECIFIC(iObj).eFOVOverlapFar

#define OBJ_PROBABILITY_OF_EXIST(iObj)            OBJ_QUALIFIERS(iObj).uiProbabilityOfExistence 

#define OBJ_GET_AVLC_FUN_PRESEL_QUALITY(iObj)      OBJ_QUALIFIERS(iObj).uiAccObjQuality
#define OBJ_GET_EBA_MOV_PRESEL_QUALITY(iObj)      OBJ_QUALIFIERS(iObj).uiEbaObjQuality    
#define OBJ_GET_EBA_HYPOTHESIS_CATEGORY(iObj)     OBJ_QUALIFIERS(iObj).eEbaHypCat
#define OBJ_GET_EBA_INHIBITION_MASK(iObj)         OBJ_QUALIFIERS(iObj).eEbaInhibitionMask

#define OBJ_PD_NEAR_OBJ_IN_BEAM(iObj)             OBJ_CR_SENSORSPECIFIC(iObj).bNearObjInBeam

#define GET_VLC_OBJ_PUB(iObj)                     GET_VLC_PUB_OBJ_DATA_PTR->ObjList[iObj]

#define VLC_PUB_OBJ_LIST_VERSION                  GET_VLC_PUB_OBJ_DATA_PTR->uiVersionNumber   
#define VLC_PUB_OBJ_LIST_NUM_OBJS                 GET_VLC_PUB_OBJ_DATA_PTR->HeaderAssessedObjList.iNumOfUsedObjects

#define OBJ_GET_EM_CD_OBJECT_PROBABILITY(iObj)    (( ((ObjNumber_t)OBJ_MERGE_DELETE_FLAG(iObj)) == (iObj))?(GET_VLC_OBJ_PUB(iObj).Legacy.CDObjectQuality):(0.0f)) 

#define RSP_GET_RANGE_GATE_LENGTH                 EnvmData.p_RHCAdjHwConfigFar->RampPar.f_RangeGateLength   

#define RSP_INPUT_REQ_SMALL_TGT_THRESHOLD         0.0f 

#define PD_GET_SRD_SYS_DAMP                       GET_PERF_DEG_DATA_INPUT_PTR->eSRDSysDamp             

#define RSP_OUT_PD_GET_ROADBEAM_VISIBILITY        GET_RSP_PD_OUTPUT_DATA_PTR->RSPBlockageData.RoadVisibility
#define RSP_OUT_PD_GET_ROADBEAM_VISIBILITY_QUAL   GET_RSP_PD_OUTPUT_DATA_PTR->RSPBlockageData.iQuality
#define RSP_OUT_PD_GET_ROADBEAM_SENSOR_TILTED     GET_RSP_PD_OUTPUT_DATA_PTR->RSPBlockageData.SensorTilted


#define TUNNEL_PROBABILITY                        0  

#define SPM_RESET_BLOCKAGE_CRITERIA_FLAG          GET_SPM_DATA_PTR->bResetBlockageCriteria

#define EGO_SPEED_X_STATE                         VED_GET_IO_STATE(VED_SOUT_POS_VEL, GET_EGO_RAW_DATA_PTR->State) 

#define EGO_SPEED_X_RAW                           GET_EGO_RAW_DATA_PTR->Longitudinal.VeloCorr.corrVelo
#define EGO_SPEED_X_VAR_RAW                       GET_EGO_RAW_DATA_PTR->Longitudinal.MotVar.varVelocity           

#define EGO_SPEED_X_CORRECTED_STATE               VED_GET_IO_STATE(VED_SOUT_POS_VCORR, GET_EGO_RAW_DATA_PTR->State)

#define EGO_SPEED_X_CORRECTED                     GET_EGO_RAW_DATA_PTR->Longitudinal.VeloCorr.corrVelo            
#define EGO_SPEED_X_CORRECTED_VAR                 GET_EGO_RAW_DATA_PTR->Longitudinal.VeloCorr.corrVeloVar         

#define EGO_ACCEL_X_STATE                         VED_GET_IO_STATE(VED_SOUT_POS_ACCEL, GET_EGO_RAW_DATA_PTR->State) 
#define EGO_ACCEL_X_RAW                           GET_EGO_RAW_DATA_PTR->Longitudinal.MotVar.Accel                 
#define EGO_ACCEL_X_VAR_RAW                       GET_EGO_RAW_DATA_PTR->Longitudinal.MotVar.varAccel              

#define EGO_YAW_RATE_STATE                        VED_GET_IO_STATE(VED_SOUT_POS_YWR, GET_EGO_RAW_DATA_PTR->State) 
#define EGO_YAW_RATE_RAW                          GET_EGO_RAW_DATA_PTR->Lateral.YawRate.YawRate                    
#define EGO_YAW_RATE_VAR_RAW                      GET_EGO_RAW_DATA_PTR->Lateral.YawRate.Variance                   
#define EGO_YAW_RATE_QUALITY_RAW                  GET_EGO_RAW_DATA_PTR->Lateral.YawRate.Quality

#define EGO_CURVE_RAW                             GET_EGO_RAW_DATA_PTR->Lateral.Curve.Curve         
#define EGO_CURVE_GRAD_RAW                        GET_EGO_RAW_DATA_PTR->Lateral.Curve.Gradient      
#define EGO_CURVE_VAR_RAW                         GET_EGO_RAW_DATA_PTR->Lateral.Curve.varC0         
#define EGO_CURVE_QUALITY_RAW                     GET_EGO_RAW_DATA_PTR->Lateral.Curve.Quality       
#define EGO_CURVE_ERROR_RAW                       GET_EGO_RAW_DATA_PTR->Lateral.Curve.CrvError      

#define EGO_DRV_INT_CURVE_RAW                     GET_EGO_RAW_DATA_PTR->Lateral.DrvIntCurve.Curve          
#define EGO_DRV_INT_CURVE_GRAD_RAW                GET_EGO_RAW_DATA_PTR->Lateral.DrvIntCurve.Gradient       
#define EGO_DRV_INT_CURVE_VAR_RAW                 GET_EGO_RAW_DATA_PTR->Lateral.DrvIntCurve.Variance       

#define EGO_SIDE_SLIP_RAW                         GET_EGO_RAW_DATA_PTR->Lateral.SlipAngle.SideSlipAngle    
#define EGO_SIDE_SLIP_VAR_RAW                     GET_EGO_RAW_DATA_PTR->Lateral.SlipAngle.Variance         

#define EGO_MOTION_STATE_RAW                      GET_EGO_RAW_DATA_PTR->MotionState.MotState
#define EGO_MOTION_STATE_STATE                    VED_GET_IO_STATE(VED_SOUT_POS_MSTAT, GET_EGO_RAW_DATA_PTR->State)

#define EGO_SPEED_X_OBJ_SYNC                      GET_EGO_OBJ_SYNC_DATA_PTR->Longitudinal.MotVar.Velocity         
#define EGO_SPEED_X_VAR_OBJ_SYNC                  GET_EGO_OBJ_SYNC_DATA_PTR->Longitudinal.MotVar.varVelocity      

#define EGO_ACCEL_X_OBJ_SYNC                      GET_EGO_OBJ_SYNC_DATA_PTR->Longitudinal.MotVar.Accel            
#define EGO_ACCEL_X_VAR_OBJ_SYNC                  GET_EGO_OBJ_SYNC_DATA_PTR->Longitudinal.MotVar.varAccel         

#define EGO_MERGED_EGO_ACCEL_X_VAR_OBJ_SYNC       GET_EGO_OBJ_SYNC_DATA_PTR->Longitudinal.AccelCorr.corrAccelVar  
#define EGO_MERGED_EGO_ACCEL_X_STD_OBJ_SYNC       SQRT_(EGO_MERGED_EGO_ACCEL_X_VAR_OBJ_SYNC)                      
#define EGO_MERGED_EGO_ACCEL_X_OBJ_SYNC           GET_EGO_OBJ_SYNC_DATA_PTR->Longitudinal.AccelCorr.corrAccel     

#define EGO_YAW_RATE_OBJ_SYNC                     GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.YawRate.YawRate              
#define EGO_YAW_RATE_VAR_OBJ_SYNC                 GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.YawRate.Variance             
#define EGO_YAW_RATE_MAX_JITTER_OBJ_SYNC          GET_EGO_OBJ_SYNC_DATA_PTR->Legacy.YawRateMaxJitter              

#define EGO_CURVE_OBJ_SYNC                        GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.Curve.Curve                  
#define EGO_CURVE_GRAD_OBJ_SYNC                   GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.Curve.Gradient               
#define EGO_CURVE_VAR_OBJ_SYNC                    GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.Curve.varC0                  

#define EGO_DRV_INT_CURVE_OBJ_SYNC                GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.DrvIntCurve.Curve            
#define EGO_DRV_INT_CURVE_GRAD_OBJ_SYNC           GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.DrvIntCurve.Gradient         
#define EGO_DRV_INT_CURVE_VAR_OBJ_SYNC            GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.DrvIntCurve.Variance         

#define EGO_SIDE_SLIP_OBJ_SYNC                    GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.SlipAngle.SideSlipAngle      
#define EGO_SIDE_SLIP_VAR_OBJ_SYNC                GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.SlipAngle.Variance           

#define EGO_SPEED_X_TGT_SYNC                      GET_EGO_TGT_SYNC_DATA_PTR->Longitudinal.MotVar.Velocity        
#define EGO_SPEED_X_VAR_TGT_SYNC                  GET_EGO_TGT_SYNC_DATA_PTR->Longitudinal.MotVar.varVelocity     

#define EGO_ACCEL_X_TGT_SYNC                      GET_EGO_TGT_SYNC_DATA_PTR->Longitudinal.MotVar.Accel           
#define EGO_ACCEL_X_VAR_TGT_SYNC                  GET_EGO_TGT_SYNC_DATA_PTR->Longitudinal.MotVar.varAccel        

#define EGO_YAW_RATE_TGT_SYNC                     GET_EGO_TGT_SYNC_DATA_PTR->Lateral.YawRate.YawRate             
#define EGO_YAW_RATE_VAR_TGT_SYNC                 GET_EGO_TGT_SYNC_DATA_PTR->Lateral.YawRate.Variance            
#define EGO_YAW_RATE_MAX_JITTER_TGT_SYNC          GET_EGO_TGT_SYNC_DATA_PTR->Legacy.YawRateMaxJitter             

#define EGO_CURVE_TGT_SYNC                        GET_EGO_TGT_SYNC_DATA_PTR->Lateral.Curve.Curve                 
#define EGO_CURVE_GRAD_TGT_SYNC                   GET_EGO_TGT_SYNC_DATA_PTR->Lateral.Curve.Gradient              
#define EGO_CURVE_VAR_TGT_SYNC                    GET_EGO_TGT_SYNC_DATA_PTR->Lateral.Curve.varC0                 

#define EGO_DRV_INT_CURVE_TGT_SYNC                GET_EGO_TGT_SYNC_DATA_PTR->Lateral.DrvIntCurve.Curve           
#define EGO_DRV_INT_CURVE_GRAD_TGT_SYNC           GET_EGO_TGT_SYNC_DATA_PTR->Lateral.DrvIntCurve.Gradient        
#define EGO_DRV_INT_CURVE_VAR_TGT_SYNC            GET_EGO_TGT_SYNC_DATA_PTR->Lateral.DrvIntCurve.Variance        

#define EGO_SIDE_SLIP_TGT_SYNC                    GET_EGO_TGT_SYNC_DATA_PTR->Lateral.SlipAngle.SideSlipAngle     
#define EGO_SIDE_SLIP_VAR_TGT_SYNC                GET_EGO_TGT_SYNC_DATA_PTR->Lateral.SlipAngle.Variance          

#define EGO_VEHICLE_WIDTH                         GET_EGO_STATIC_DATA_PTR->VehParAdd.VehicleWidth                
#define EGO_VEHICLE_TRACK_WIDTH_FRONT             GET_EGO_STATIC_DATA_PTR->VehParMain.TrackWidthFront             
#define EGO_VEHICLE_WHEEL_BASE                    GET_EGO_STATIC_DATA_PTR->VehParMain.WheelBase               
#define EGO_VEHICLE_AXLE_LOAD_DISTR               GET_EGO_STATIC_DATA_PTR->VehParMain.AxisLoadDistr        

#define ROAD_ESTMATION_TYPE                     RoadEstimation_t
#define GET_ROAD_ESTIMATION                     &GET_ROAD_DATA_PTR->RoadEstimation

#define ROAD_GET_TRAFFIC_ORIENTATION              GET_ROAD_DATA_PTR->Traffic.eTrafficOrientation

#define ROAD_GET_LANEWIDTH                        GET_ROAD_DATA_PTR->LaneWidth.fLaneWidth
#define ROAD_GET_LANEWIDTH_SOURCE                 GET_ROAD_DATA_PTR->LaneWidth.eLaneWidthSource
#define ROAD_GET_LANE_CLASS                       GET_ROAD_DATA_PTR->LaneWidth.eLaneWidthClass

#define ROAD_GET_LANE_MATRIX                      GET_ROAD_DATA_PTR->LaneMatrix

#define ROAD_GET_BORDER                           GET_ROAD_DATA_PTR->FusedRoadBorder
#define ROAD_GET_BORDER_RIGHT                     ROAD_GET_BORDER.fDistRight
#define ROAD_GET_BORDER_RIGHT_STD                 GET_ROAD_DATA_PTR->FusedRoadBorder.fDistRightStd
#define ROAD_GET_BORDER_LEFT                      ROAD_GET_BORDER.fDistLeft
#define ROAD_GET_BORDER_LEFT_STD                  GET_ROAD_DATA_PTR->FusedRoadBorder.fDistLeftStd
#define ROAD_GET_BORDER_STAT_RIGHT                ROAD_GET_BORDER.bStatusRight
#define ROAD_GET_BORDER_STAT_LEFT                 ROAD_GET_BORDER.bStatusLeft


#define ROAD_GET_CR_YAWANGLE                    GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.ClothoidParams.f_YawAngle
#define ROAD_GET_CR_CURVATURE                   GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.ClothoidParams.f_Curvature
#define ROAD_GET_CR_CLOTHOID_PARAM              GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.ClothoidParams.f_ClothoidParam1
#define ROAD_GET_CR_TRANSITION_POINT            GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.ClothoidParams.f_TransitionPoint
#define ROAD_GET_CR_CLOTHOID_PARAM_2            GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.ClothoidParams.f_ClothoidParam2
#define ROAD_GET_CR_CONFIDENCE_LEFT             GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasLeft.f_Confidence
#define ROAD_GET_CR_MIN_X_LEFT                  GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasLeft.f_MinX
#define ROAD_GET_CR_MAX_X_LEFT                  GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasLeft.f_MaxX
#define ROAD_GET_CR_MAX_X_LEFT_COMPENSTATED     GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasLeft.f_MaxXCompensated
#define ROAD_GET_CR_TRACKSTAT_LEFT              GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasLeft.u_TrackingStatus
#define ROAD_GET_CR_CONFIDENCE_RIGHT            GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasRight.f_Confidence
#define ROAD_GET_CR_MIN_X_RIGHT                 GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasRight.f_MinX
#define ROAD_GET_CR_MAX_X_RIGHT                 GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasRight.f_MaxX
#define ROAD_GET_CR_MAX_X_RIGHT_COMPENSTATED    GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasRight.f_MaxXCompensated
#define ROAD_GET_CR_TRACKSTAT_RIGHT             GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasRight.u_TrackingStatus
#define ROAD_GET_CR_OFFSET_LEFT                 GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.f_LateralOffsetLeft
#define ROAD_GET_CR_OFFSET_RIGHT                GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.f_LateralOffsetRight
#define ROAD_GET_CR_CONFIDENCE                  GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.u_GlobalConfidence
#define ROAD_GET_CR_TRACKSTAT                   GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.u_GlobalTrackingStatus
#define ROAD_GET_CR_EST_STDEV                   GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.f_LatStdDevFiltered

#define ROAD_GET_BL_YAWANGLE                    GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.ClothoidParams.f_YawAngle
#define ROAD_GET_BL_CURVATURE                   GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.ClothoidParams.f_Curvature
#define ROAD_GET_BL_CLOTHOID_PARAM              GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.ClothoidParams.f_ClothoidParam1
#define ROAD_GET_BL_TRANSITION_POINT            GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.ClothoidParams.f_TransitionPoint
#define ROAD_GET_BL_CLOTHOID_PARAM_2            GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.ClothoidParams.f_ClothoidParam2
#define ROAD_GET_BL_CONFIDENCE_LEFT             GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.QualityMeasLeft.f_Confidence
#define ROAD_GET_BL_MIN_X_LEFT                  GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.QualityMeasLeft.f_MinX
#define ROAD_GET_BL_MAX_X_LEFT                  GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.QualityMeasLeft.f_MaxX
#define ROAD_GET_BL_MAX_X_LEFT_COMPENSTATED     GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.QualityMeasLeft.f_MaxXCompensated
#define ROAD_GET_BL_TRACKSTAT_LEFT              GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.QualityMeasLeft.u_TrackingStatus
#define ROAD_GET_BL_CONFIDENCE_RIGHT            GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.QualityMeasLeft.f_Confidence
#define ROAD_GET_BL_MIN_X_RIGHT                 GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.QualityMeasRight.f_MinX
#define ROAD_GET_BL_MAX_X_RIGHT                 GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.QualityMeasRight.f_MaxX
#define ROAD_GET_BL_MAX_X_RIGHT_COMPENSTATED    GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.QualityMeasRight.f_MaxXCompensated
#define ROAD_GET_BL_TRACKSTAT_RIGHT             GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.QualityMeasRight.u_TrackingStatus
#define ROAD_GET_BL_OFFSET_LEFT                 GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.f_LateralOffsetLeft
#define ROAD_GET_BL_OFFSET_RIGHT                GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.f_LateralOffsetRight
#define ROAD_GET_BL_CONFIDENCE                  GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.u_GlobalConfidence
#define ROAD_GET_BL_TRACKSTAT                   GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.u_GlobalTrackingStatus
#define ROAD_GET_BL_EST_STDEV                   GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.f_LatStdDevFiltered

#define ROAD_GET_BR_YAWANGLE                    GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.ClothoidParams.f_YawAngle
#define ROAD_GET_BR_CURVATURE                   GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.ClothoidParams.f_Curvature
#define ROAD_GET_BR_CLOTHOID_PARAM              GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.ClothoidParams.f_ClothoidParam1
#define ROAD_GET_BR_TRANSITION_POINT            GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.ClothoidParams.f_TransitionPoint
#define ROAD_GET_BR_CLOTHOID_PARAM_2            GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.ClothoidParams.f_ClothoidParam2
#define ROAD_GET_BR_CONFIDENCE_LEFT             GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.QualityMeasLeft.f_Confidence
#define ROAD_GET_BR_MIN_X_LEFT                  GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.QualityMeasLeft.f_MinX
#define ROAD_GET_BR_MAX_X_LEFT                  GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.QualityMeasLeft.f_MaxX
#define ROAD_GET_BR_MAX_X_LEFT_COMPENSTATED     GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.QualityMeasLeft.f_MaxXCompensated
#define ROAD_GET_BR_TRACKSTAT_LEFT              GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.QualityMeasLeft.u_TrackingStatus
#define ROAD_GET_BR_CONFIDENCE_RIGHT            GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.QualityMeasLeft.f_Confidence
#define ROAD_GET_BR_MIN_X_RIGHT                 GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.QualityMeasRight.f_MinX
#define ROAD_GET_BR_MAX_X_RIGHT                 GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.QualityMeasRight.f_MaxX
#define ROAD_GET_BR_MAX_X_RIGHT_COMPENSTATED    GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.QualityMeasRight.f_MaxXCompensated
#define ROAD_GET_BR_TRACKSTAT_RIGHT             GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.QualityMeasRight.u_TrackingStatus
#define ROAD_GET_BR_OFFSET_LEFT                 GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.f_LateralOffsetLeft
#define ROAD_GET_BR_OFFSET_RIGHT                GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.f_LateralOffsetRight
#define ROAD_GET_BR_CONFIDENCE                  GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.u_GlobalConfidence
#define ROAD_GET_BR_TRACKSTAT                   GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.u_GlobalTrackingStatus
#define ROAD_GET_BR_EST_STDEV                   GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.f_LatStdDevFiltered

#define ROAD_GET_CR_DATA                        GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad
#define ROAD_GET_CR_PARAMETERS                  GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.ClothoidParams
#define ROAD_GET_CR_LEFT_QUALITY                GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasLeft
#define ROAD_GET_CR_RIGHT_QUALITY               GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasRight
#define ROAD_GET_BL_DATA                        GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft
#define ROAD_GET_BL_PARAMETERS                  GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.ClothoidParams
#define ROAD_GET_BL_QUALITY                     GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderLeft.QualityMeas
#define ROAD_GET_BR_DATA                        GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight
#define ROAD_GET_BR_PARAMETERS                  GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.ClothoidParams
#define ROAD_GET_BR_QUALITY                     GET_ROAD_DATA_PTR->RoadEstimation.RoadBorderRight.QualityMeas

#define ROAD_SET_CONFIDENCE_LEFT_RIGHT(left,right)  \
  GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasLeft.f_Confidence = left; \
  GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasRight.f_Confidence = right;

#define ROAD_GET_MIN_MAX_TGT_DIST(fMinRight, fMaxRight, fMinLeft, fMaxLeft) \
  *fMinRight = GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasRight.f_MinX; \
  *fMaxRight = GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasRight.f_MaxXCompensated; \
  *fMinLeft  = GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasLeft.f_MinX;  \
  *fMaxLeft  = GET_ROAD_DATA_PTR->RoadEstimation.CoupledRoad.QualityMeasLeft.f_MaxXCompensated;

#define ROAD_GET_ROADTYPE               GET_ROAD_DATA_PTR->RoadType.eRoadTypeClass
#define ROAD_GET_ROADWORKS              GET_ROAD_DATA_PTR->RoadType.eRoadWorksClass
#define ROAD_GET_ROADTYPE_CONF          GET_ROAD_DATA_PTR->RoadType.fRoadTypeClassConf
#define ROAD_GET_ROADWORKS_CONF         GET_ROAD_DATA_PTR->RoadType.fRoadWorksClassConf

#define ROAD_GET_TYPE1( iTypeLevel1, fTypeLevel1Conf)  \
  *iTypeLevel1     = GET_ROAD_DATA_PTR->RoadType.eRoadTypeClass; \
  *fTypeLevel1Conf = GET_ROAD_DATA_PTR->RoadType.fRoadTypeClassConf;

#define ROAD_GET_TYPE2( iTypeLevel2, fTypeLevel2Conf)  \
  *iTypeLevel2     = GET_ROAD_DATA_PTR->RoadType.eRoadWorksClass; \
  *fTypeLevel2Conf = GET_ROAD_DATA_PTR->RoadType.fRoadWorksClassConf

#define ROAD_GET_TYPE( iTypeLevel1, iTypeLevel2)  \
  *iTypeLevel1     = GET_ROAD_DATA_PTR->RoadType.eRoadTypeClass; \
  *iTypeLevel2     = GET_ROAD_DATA_PTR->RoadType.eRoadWorksClass

#define ROAD_GET_LANE_NO_LEFT                     GET_ROAD_DATA_PTR->LaneMatrix.iNumOfLaneLeft
#define ROAD_GET_LANE_NO_RIGHT                    GET_ROAD_DATA_PTR->LaneMatrix.iNumOfLaneRight
#define ROAD_GET_NO_LANE_MATRIX_PROB_LEFT         GET_ROAD_DATA_PTR->LaneMatrix.NoLaneProbLeft
#define ROAD_GET_NO_LANE_MATRIX_PROB_RIGHT        GET_ROAD_DATA_PTR->LaneMatrix.NoLaneProbRight


#define OBJECT_TRACES_TYPE                            ObjectTraces_t
#define GET_TRACE(iTrace)                             GET_OBJ_TRACE_LIST_DATA_PTR->ObjectTraces[iTrace]                                              
#define TRACE_GET_HEAD_VECTOR(iTrace)                 GET_TRACE(iTrace).ObjectTraceBasic.Head
#define TRACE_GET_TAIL_VECTOR(iTrace)                 GET_TRACE(iTrace).ObjectTraceBasic.Tail
#define TRACE_GET_STATE(iTrace)                       GET_TRACE(iTrace).ObjectTraceAttributes.eTraceUpdateState
#define TRACE_GET_TYPE(iTrace)                        GET_TRACE(iTrace).ObjectTraceAttributes.eTraceType

#define MOVING_OBJ_TRACES_TYPE                        ObjectTrace_t
#define GET_MOVING_OBJ_TRACE_PTR(iTrace)              (&GET_MOV_OBJ_TRACE_DATA_PTR->ObjectTrace[iTrace])
#define TRACE_GET_X(iTrace)                           GET_MOVING_OBJ_TRACE_PTR(iTrace)->fTracePointX
#define TRACE_GET_Y(iTrace)                           GET_MOVING_OBJ_TRACE_PTR(iTrace)->fTracePointY
#define TRACE_GET_Y_INTERSEC_GRAD_FILT(iTrace)        GET_MOVING_OBJ_TRACE_PTR(iTrace)->fYIntersectionGradFilt
#define TRACE_GET_YAXIS_PRED_INTERSECTION(iTrace)     GET_MOVING_OBJ_TRACE_PTR(iTrace)->Legacy.YPredictedIntersec
#define TRACE_GET_YAXIS_PRED_INTERSECTION_VAR(iTrace) GET_MOVING_OBJ_TRACE_PTR(iTrace)->Legacy.YPredictedIntersecVar


#define GET_OBJ_TRACE_LIST_PTR(iTR)               (&GET_OBJ_TRACE_LIST_DATA_PTR->ObjectTraces[iTr])
#define TRACE_GET_ID(iTr)                         GET_MOVING_OBJ_TRACE_PTR(iTr)->uiReferenceToObject
#define TRACE_GET_VLC_ID(iTr)                     GET_MOVING_OBJ_TRACE_PTR(iTr)->uiReferenceToVLCObject
#define TRACE_GET_Y_STD_DEV(iTr)                  GET_MOVING_OBJ_TRACE_PTR(iTr)->fTracePointYStdDev
#define TRACE_GET_Y_INTERSEC(iTr)                 GET_MOVING_OBJ_TRACE_PTR(iTr)->fYIntersection
#define TRACE_GET_NO_OF_POINTS(iTr)               GET_MOVING_OBJ_TRACE_PTR(iTr)->iNumberOfPoints
#define TRACE_REACHED_EGO_VEH(iTr)                GET_MOVING_OBJ_TRACE_PTR(iTr)->Legacy.TraceReachEgoVeh
#define TRACE_VALID_LANEMATRIX(iTr)               GET_MOVING_OBJ_TRACE_PTR(iTr)->Legacy.ValidForLaneMatrix


#define SENSOR_MOUNTING                           GET_EGO_STATIC_DATA_PTR->SensorMounting
#define SENSOR_X_POSITION                         SENSOR_MOUNTING.LongPos 
#define SENSOR_Y_POSITION                         SENSOR_MOUNTING.LatPos  
#define SENSOR_Z_POSITION                         SENSOR_MOUNTING.VertPos 

#define SENSOR_X_POSITION_CoG                     SENSOR_MOUNTING.LongPosToCoG  

#define LOBE_ANGLE                                GET_EGO_STATIC_DATA_PTR->Sensor.fLobeAngle       

#define COVERAGE_ANGLE                            GET_EGO_STATIC_DATA_PTR->Sensor.fCoverageAngle   

#define MAX_ANGLE                                 ((LOBE_ANGLE+(COVERAGE_ANGLE*0.5F))*0.5F)

#define NOF_SCANS                                 GET_EGO_STATIC_DATA_PTR->Sensor.uNoOfScans       

#define GDB_CYCLE_MODE                            GET_Envm_VLC_CYCLE_MODE_PTR->eCycleMode        

#define TASK_CYCLE_TIME                          GET_Envm_VLC_CYCLE_MODE_PTR->fECAMtCycleTime


#define VED_IO_STATE_BITMASK                   ((VEDIoState_t) (0x3U))
#define VED_SET_IO_STATE(pos_, state_, val_)   ( (val_)[(pos_)/32UL] = (~((VEDIoState_t) (VED_IO_STATE_BITMASK<<((pos_)%32UL))) & ((val_)[(pos_)/32UL])) | ((VED_IO_STATE_BITMASK & (state_))<<((pos_)%32UL)))
#define VED_GET_IO_STATE(pos_,val_)            ( ((val_)[(pos_)/32UL] >> ((pos_) % 32UL)) & VED_IO_STATE_BITMASK)


#define OBJ_INDEX_NO_OBJECT     ((ObjNumber_t)-1)

#ifndef STRING_QUOTE
#define STRING_QUOTE         ALGO_StringQuote
#endif

#define RSP_VALID      (RSPValidFlag_t) 1    
#define RSP_INVALID    (RSPValidFlag_t) 0    

#define RSP_DIST_MAX_FAR 200    

#define RSP_NUM_OF_REG_OF_INT_SAMP 20 

#define MEAS_ID_EGO_STATIC_PUBLIC_GLOB_DATA   0x2000FC80uL               

#define ALGO_OPTIMIZED_TARGET_DIST_FOR_ELEVATION  70.f

#ifndef _PARAM_UNUSED
#define _PARAM_UNUSED(x) (void)(x)
#endif

#endif

//common use
#ifndef Acc_sit_class_undefined
#define Acc_sit_class_undefined 0U
#endif
#ifndef Acc_sit_class_freemode
#define Acc_sit_class_freemode 1U
#endif
#ifndef Acc_sit_class_follow
#define Acc_sit_class_follow 2U
#endif
#ifndef Acc_sit_class_crawl
#define Acc_sit_class_crawl 3U
#endif
#ifndef Acc_sit_class_stop
#define Acc_sit_class_stop 4U
#endif
#ifndef Acc_sit_class_go
#define Acc_sit_class_go 5U
#endif
#ifndef Acc_sit_class_overtake
#define Acc_sit_class_overtake 6U
#endif

#ifndef OBJ_NOT_OOI
#define OBJ_NOT_OOI -1
#endif
#ifndef OBJ_NEXT_OOI
#define OBJ_NEXT_OOI 0
#endif
#ifndef OBJ_HIDDEN_NEXT_OOI
#define OBJ_HIDDEN_NEXT_OOI 1
#endif
#ifndef OBJ_NEXT_LONG_LEFT_OOI
#define OBJ_NEXT_LONG_LEFT_OOI 2
#endif
#ifndef OBJ_NEXT_LONG_RIGHT_OOI
#define OBJ_NEXT_LONG_RIGHT_OOI 3
#endif
#ifndef OBJ_NEXT_LAT_LEFT_OOI
#define OBJ_NEXT_LAT_LEFT_OOI 4
#endif
#ifndef OBJ_NEXT_LAT_RIGHT_OOI
#define OBJ_NEXT_LAT_RIGHT_OOI 5
#endif

#ifndef Obj_lane_left1
#define Obj_lane_left1 -1
#endif
#ifndef Obj_lane_same
#define Obj_lane_same 0
#endif
#ifndef Obj_lane_right1
#define Obj_lane_right1 1
#endif



#endif