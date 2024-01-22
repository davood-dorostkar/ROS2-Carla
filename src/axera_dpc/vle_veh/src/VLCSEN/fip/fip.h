

#ifndef _FIP_H_INCLUDED
#define _FIP_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/

//#include "vlc_sen.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  SYMOBLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

/*---fip_camera_lane_plg.c---*/
extern void FIP_v_CamLaneDataProcess(
    void); /*!< VLC Camera Lane Data Preprocessing */
extern void FIP_v_InitGlobalCamLaneData(
    void); /*!< Initialization of global camera lane data */

/*! fip_lane_matrix_plg.c */
extern void FIP_v_LaneMatrixProcess(void); /*!< FIP lane matrix preprocessing */
extern void FIP_v_InitGlobalLaneMatrixData(
    void); /*!< Initialization of global lane matrix data */
extern void FIP_v_InitLaneWidthInfo(void); /*!< Initialization of lane width */

/*! fip_object_traces.c */
extern void FIP_v_InitGlobalObjTraceData(
    void); /*!< Initialization of global object trace data */
extern void FIP_v_ObjTraceProcess(void); /*!< FIP Object Trace Preprocessing */

/*! fip_traffic_orientation.c */
extern void FIP_v_InitGlobalLaneTrafficOrientation(void);
extern void FIP_v_TrafficOrientationProcess(void);

/*! fip_customfunctions.c */
extern void FIP_v_Set_CustomTrafficOrientation(void);
extern void FIP_v_Init_CustomTrafficOrientation(void);

/*! fip_road_type.c */
extern void FIP_v_Init_Road(void);
extern void FIP_v_Init_FuseRoadType(void);
extern void FIP_v_RoadProcess(void);
extern void FIP_v_Process_FuseRoadType(void);

#ifdef __cplusplus
};
#endif

/* End of conditional inclusion */
#endif /*!< _FIP_H_INCLUDED */

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
