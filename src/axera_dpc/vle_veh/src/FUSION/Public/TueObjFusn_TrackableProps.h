/** \addtogroup objectlist
 *  \{
 *  \file    TueObjFusn_TrackableListProps.h
 *  \brief   Defines min, max and default values for all fields of an Trackable
 * object
 *
 *
 *
 *
 * <br>=====================================================<br>
 * <b>Copyright 2014 by Tuerme.</b>
 * <br>
 * All rights reserved. Property of Tuerme.<br>
 * Restricted rights to use, duplicate or disclose of this code<br>
 * are granted through contract.
 * <br>=====================================================<br>
 */

#ifndef TUEOBJFUSN_TRACKABLEPROPS_H_
#define TUEOBJFUSN_TRACKABLEPROPS_H_

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_ConfigAlgorithm.h"
#include "TueObjFusn_TrackableConstants.h"

/*==================[macros]================================================*/

/************/
/* bUpdated */
/************/
/// \name bUpdated
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_BUPDATED_MIN (STD_OFF)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_BUPDATED_MAX (STD_ON)
/** default value */
#define TUEOBJFUSN_TRACKABLE_BUPDATED_DEFAULT (STD_OFF)
/// \}

/*********/
/* u16ID */
/*********/
/// \name u16ID
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U16ID_MIN (TUEOBJFUSN_IDPROVIDER_U16ID_MIN)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U16ID_MAX (TUEOBJFUSN_IDPROVIDER_U16ID_MAX)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT (61234u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U16ID_FILLEDBYFUSION (STD_ON)
/// \}

/**********/
/* u16Age */
/**********/
/// \name u16Age
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U16AGE_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U16AGE_MAX (TUEOBJFUSN_MAX_AGE)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U16AGE_DEFAULT (65234u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U16AGE_FILLEDBYFUSION (STD_ON)
/// \}

/***************/
/* u16Lifespan */
/***************/
/// \name u16Lifespan
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U16LIFESPAN_MIN \
    (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_FREE_SLOT)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U16LIFESPAN_MAX \
    (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_COASTED)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U16LIFESPAN_DEFAULT \
    (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_FREE_SLOT)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U16LIFESPAN_FILLEDBYFUSION (STD_ON)
/// \}

/******************/
/* u32SensorsCurr */
/******************/
/// \name u32SensorsCurr
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U32SENSORSCURR_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U32SENSORSCURR_MAX (0x7FFFFFFFu)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U32SENSORSCURR_DEFAULT (0xFFFFFFFFu)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U32SENSORSCURR_FILLEDBYFUSION (STD_ON)
/// \}

/******************/
/* u32SensorsHist */
/******************/
/// \name u32SensorsHist
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U32SENSORSHIST_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U32SENSORSHIST_MAX (0x7FFFFFFFu)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U32SENSORSHIST_DEFAULT (0xFFFFFFFFu)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U32SENSORSHIST_FILLEDBYFUSION (STD_ON)
/// \}

/********************/
/* u8CyclesNoVision */
/********************/
/// \name u32SensorsHist
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U8CYCLESNOVISION_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U8CYCLESNOVISION_MAX (200u)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U8CYCLESNOVISION_DEFAULT (255u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U8CYCLESNOVISION_FILLEDBYFUSION (STD_ON)
/// \}

/*******************/
/* u8CyclesNoRadar */
/*******************/
/// \name u32SensorsHist
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U8CYCLESNORADAR_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U8CYCLESNORADAR_MAX (200u)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U8CYCLESNORADAR_DEFAULT (255u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U8CYCLESNORADAR_FILLEDBYFUSION (STD_ON)
/// \}

/****************/
/* au16SensorID */
/****************/
/// \name au16SensorID
/** minimum value */
#define TUEOBJFUSN_DEBUGLIST_AU16SENSORID_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_DEBUGLIST_AU16SENSORID_MAX (60000u)
/** default value */
#define TUEOBJFUSN_DEBUGLIST_AU16SENSORID_DEFAULT (61234u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_AU16SENSORID_FILLEDBYFUSION (STD_ON)
/// \}

/***********/
/* f32PosX */
/***********/
/// \name f32PosX
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32POSX_MIN (-1000.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32POSX_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32POSX_DEFAULT (12345.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32POSX_FILLEDBYFUSION (STD_ON)
///\}

/***********/
/* f32PosY */
/***********/
/// \name f32PosY
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32POSY_MIN (-1000.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32POSY_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32POSY_DEFAULT (12345.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32POSY_FILLEDBYFUSION (STD_ON)
///\}

/**************/
/* f32PosVarX */
/**************/
/// \name f32PosVarX
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32POSVARX_MIN (0.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32POSVARX_MAX (2000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32POSVARX_DEFAULT (-1.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32POSVARX_FILLEDBYFUSION (STD_ON)
///\}

/**************/
/* f32PosVarY */
/**************/
/// \name f32PosVarY
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32POSVARY_MIN (0.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32POSVARY_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32POSVARY_DEFAULT (-1.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32POSVARY_FILLEDBYFUSION (STD_ON)
///\}

/***************/
/* f32PosCovXY */
/***************/
/// \name f32PosCovXY
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32POSCOVXY_MIN (-1000.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32POSCOVXY_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32POSCOVXY_DEFAULT (12345.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32POSCOVXY_FILLEDBYFUSION (STD_ON)
///\}

/***********/
/* f32VelX */
/***********/
/// \name  f32VelX
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32VELX_MIN (-1000.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32VELX_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32VELX_DEFAULT (12345.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32VELX_FILLEDBYFUSION (STD_ON)
///\}

/***********/
/* f32VelY */
/***********/
/// \name f32VelY
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32VELY_MIN (-1000.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32VELY_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32VELY_DEFAULT (12345.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32VELY_FILLEDBYFUSION (STD_ON)
///\}

/**************/
/* f32VelVarX */
/**************/
/// \name f32VelVarX
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32VELVARX_MIN (0.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32VELVARX_MAX (2000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32VELVARX_DEFAULT (-1.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32VELVARX_FILLEDBYFUSION (STD_ON)
///\}

/**************/
/* f32VelVarY */
/**************/
/// \name f32VelVarY
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32VELVARY_MIN (0.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32VELVARY_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32VELVARY_DEFAULT (-1.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32VELVARY_FILLEDBYFUSION (STD_ON)
///\}

/***************/
/** f32VelCovXY */
/***************/
/// \name f32VelCovXY
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32VELCOVXY_MIN (-1000.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32VELCOVXY_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32VELCOVXY_DEFAULT (12345.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32VELCOVXY_FILLEDBYFUSION (STD_ON)
///\}

/***********/
/* f32AccX */
/***********/
/// \name f32AccX
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32ACCX_MIN (-1000.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32ACCX_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32ACCX_DEFAULT (12345.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32ACCX_FILLEDBYFUSION (STD_ON)
///\}

/***********/
/* f32AccY */
/***********/
/// \name f32AccY
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32ACCY_MIN (-1000.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32ACCY_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32ACCY_DEFAULT (12345.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32ACCY_FILLEDBYFUSION (STD_ON)
///\}

/**************/
/* f32AccVarX */
/**************/
/// \name  f32AccVarX
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32ACCVARX_MIN (0.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32ACCVARX_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32ACCVARX_DEFAULT (-1.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32ACCVARX_FILLEDBYFUSION (STD_ON)
///\}

/**************/
/* f32AccVarY */
/**************/
/// \name f32AccVarY
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32ACCVARY_MIN (0.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32ACCVARY_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32ACCVARY_DEFAULT (-1.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32ACCVARY_FILLEDBYFUSION (STD_ON)
///\}

/***************/
/* f32AccCovXY */
/***************/
/// \name f32AccCovXY
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32ACCCOVXY_MIN (-1000.0f)
/** maximum value for */
#define TUEOBJFUSN_TRACKABLE_F32ACCCOVXY_MAX (1000.0f)
/** default value for */
#define TUEOBJFUSN_TRACKABLE_F32ACCCOVXY_DEFAULT (12345.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32ACCCOVXY_FILLEDBYFUSION (STD_ON)
///\}

/*********/
/* f32Width */
/*********/
/// \name f32Width
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32WIDTH_MIN (0.0f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32WIDTH_MAX (10.0f)
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32WIDTH_DEFAULT (2.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32WIDTH_FILLEDBYFUSION (STD_ON)
/// \}

/*************/
/* f32Length */
/*************/
/// \name f32Length
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32LENGTH_MIN (0.0f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32LENGTH_MAX (50.0f)
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32LENGTH_DEFAULT (1.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32LENGTH_FILLEDBYFUSION (STD_ON)
/// \}

/*************/
/* f32Height */
/*************/
/// \name f32Height
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32HEIGHT_MIN (0.0f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32HEIGHT_MAX (10.0f)
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32HEIGHT_DEFAULT (1.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32HEIGHT_FILLEDBYFUSION (STD_ON)
/// \}

/***************/
/* u16RefPoint */
/***************/
/// \name u16RefPoint
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U16REFPOINT_MIN \
    (TUEOBJFUSN_TRACKABLE_U16REFPOINT_POINT_OBJECT)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U16REFPOINT_MAX \
    (TUEOBJFUSN_TRACKABLE_U16REFPOINT_CENTER_OF_WIDTH)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U16REFPOINT_DEFAULT (61234u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U16REFPOINT_FILLEDBYFUSION (STD_ON)
/// \}

/*****************/
/* u8CoordSystem */
/*****************/
/// \name u8CoordSystem
///** minimum value */
#define TUEOBJFUSN_TRACKABLE_U8COORDSYSTEM_MIN \
    (TUEOBJFUSN_U8COORDSYSTEM_FRONTBUMP_RELATIVE)
///** maximum value */
#define TUEOBJFUSN_TRACKABLE_U8COORDSYSTEM_MAX (TUEOBJFUSN_U8COORDSYSTEM_GLOBAL)
///** default value */
#define TUEOBJFUSN_TRACKABLE_U8COORDSYSTEM_DEFAULT (0xFFu)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U8COORDSYSTEM_FILLEDBYFUSION (STD_OFF)
/// \}

/*****************/
/* u16MotionType */
/*****************/
/// \name u16MotionType
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MAX (512u)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_DEFAULT (61234u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_FILLEDBYFUSION (STD_ON)
/// \}

/************/
/* u16Class */
/************/
/// \name u16Class
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_MAX \
    (TUEOBJFUSN_TRACKABLE_U16CLASS_ANIMAL_SMALL)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_DEFAULT (61234u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_FILLEDBYFUSION (STD_ON)
/// \}

/****************/
/* u16ClassProb */
/****************/
/// \name u16ClassProb
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U16CLASSPROB_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U16CLASSPROB_MAX (101u)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U16CLASSPROB_DEFAULT (0u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U16CLASSPROB_FILLEDBYFUSION (STD_OFF)
/// \}

/************/
/* bMCPFlag */
/************/
#define TUEOBJFUSN_TRACKABLE_bMCPFlag_DEFAULT (FALSE)
#define TUEOBJFUSN_TRACKABLE_bMCPFlag_TRUE (TRUE)

/************/
/* bCIPVFlag */
/************/
#define TUEOBJFUSN_TRACKABLE_bCIPVFlag_DEFAULT (FALSE)
#define TUEOBJFUSN_TRACKABLE_bCIPVFlag_TRUE (TRUE)

/************/
/* uLifeCycle */
/************/
#define TUEOBJFUSN_TRACKABLE_uLifeCycle_DEFAULT (0u)
#define TUEOBJFUSN_TRACKABLE_uLifeCycle_MAX (61234u)

/******************/
/* bIsAsilVerified */
/******************/
/// \name bIsAsilVerified
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U8ISASILVERIFIED_MIN (0x00u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U8ISASILVERIFIED_MAX (0xFFu)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U8ISASILVERIFIED_DEFAULT (0x00u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U8ISASILVERIFIED_FILLEDBYFUSION (STD_OFF)
/// \}

/**************************/
/* f32ObstacleProbability */
/**************************/
/// \name f32ObstacleProbability
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32OBSTACLEPROBABILITY_MIN (0.0f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32OBSTACLEPROBABILITY_MAX (100.0f)
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32OBSTACLEPROBABILITY_DEFAULT (0.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32OBSTACLEPROBABILITY_FILLEDBYFUSION (STD_ON)
/// \}

/***************************/
/* f32ExistenceProbability */
/***************************/
/// \name f32ExistenceProbability
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32EXISTENCEQUALITY_MIN (0.f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32EXISTENCEQUALITY_MAX (100.0f)
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32EXISTENCEQUALITY_DEFAULT (0.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32EXISTENCEQUALITY_FILLEDBYFUSION (STD_ON)
// default value for mini eye pedestrian and other car object existence quality
#define TUEOBJFUSN_MINIEYE_TRACKABLE_F32EXISTENCEQUALITY_DEFAULT (95.0f)
/// \}

/*********/
/* f32Gain */
/*********/
/// \name f32Gain
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32GAIN_MIN (0.7f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32GAIN_MAX (1.3f)
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32GAIN_DEFAULT (1.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32GAIN_FILLEDBYFUSION (STD_OFF)
/// \}

/**************/
/* f32GainVar */
/**************/
/// \name f32GainVar
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32GAINVAR_MIN (0.0f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32GAINVAR_MAX \
    (0.05f) /* for 30% scaling error realistic is 1/3 * 0.3^2 = 0.03*/
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32GAINVAR_DEFAULT (-1.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32GAINVAR_FILLEDBYFUSION (STD_OFF)
/// \}

/**********************/
/* u8TrkblBinPosition */
/**********************/
/// \name u8TrkblBinPosition
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U8TRKBLBINPOSITION_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U8TRKBLBINPOSITION_MAX \
    (TUE_PRV_OBJECTSELECTION_MAX_BINS)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U8TRKBLBINPOSITION_DEFAULT (255u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U8TRKBLBINPOSITION_FILLEDBYFUSION (STD_OFF)
/// \}

/**************/
/* f32YawRate */
/**************/
/// \name f32YawRate
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32YAWRATE_MIN (-20.0f * 3.14159265f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32YAWRATE_MAX (20.0f * 3.14159265f)
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32YAWRATE_DEFAULT (0.f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32YAWRATE_FILLEDBYFUSION (STD_ON)
/// \}

/*****************/
/* f32YawRateVar */
/*****************/
/// \name f32YawRateVar
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32YAWRATEVAR_MIN (0.f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32YAWRATEVAR_MAX (10.f)
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32YAWRATEVAR_DEFAULT (1.f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32YAWRATEVAR_FILLEDBYFUSION (STD_ON)
/// \}

/**************/
/* f32Heading */
/**************/
/// \name f32Heading
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32HEADING_MIN (-2.0f * 3.14159265f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32HEADING_MAX (2.0f * 3.14159265f)
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32HEADING_DEFAULT (0.f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32HEADING_FILLEDBYFUSION (STD_ON)
/// \}

/*****************/
/* f32HeadingVar */
/*****************/
/// \name f32HeadingVar
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_MIN (0.f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_MAX (1000.f)
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_DEFAULT (10.0f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_FILLEDBYFUSION (STD_ON)
/// \}

/**************************/
/* f32CovarHeadingYawRate */
/**************************/
/// \name f32CovarHeadingYawRate
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_F32COVARHEADINGYAWRATE_MIN (0.f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_F32COVARHEADINGYAWRATE_MAX (1000.f)
/** default value */
#define TUEOBJFUSN_TRACKABLE_F32COVARHEADINGYAWRATE_DEFAULT (0.f)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_F32COVARHEADINGYAWRATE_FILLEDBYFUSION (STD_ON)
/// \}

/***************/
/* u8VisionIdx */
/***************/
/// \name u8VisionIdx
/** minimum value */
#define TUEOBJFUSN_TRACKABLE_U8VISIONIDX_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLE_U8VISIONIDX_MAX (TUE_PRV_FUSION_MAX_INPUTS - 1u)
/** default value */
#define TUEOBJFUSN_TRACKABLE_U8VISIONIDX_DEFAULT (255u)
/** flag indicating if this field will be filled by fusion module */
#define TUEOBJFUSN_TRACKABLE_U8VISIONIDX_FILLEDBYFUSION (STD_ON)
/// \}

/*****************/
/* Reserved data */
/*****************/
#define TUEOBJFUSN_TRACKABLE_RESERVED_DEFAULT (0u)

#endif /* TUEOBJFUSN_TRACKABLELISTPROPS_H_ */

/** \} */
