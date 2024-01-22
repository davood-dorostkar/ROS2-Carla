

#ifndef _FIP_OBJECT_TRACES_H_INCLUDED
#define _FIP_OBJECT_TRACES_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "fip.h"
#include "fip_par.h"

#include "TM_Global_Const.h"
#include "vlcSen_consts.h"
#include "vlcSen_common_utils.h"
#include "fip_ext.h"
#include "TM_Math_Cal.h"
/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/* Object Macro Definition for Moving object traces*/
#define FIPMOT_INVALID_VALUE (999.9f)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
/*! Bitfield for quality criteria of moving object traces - common to both
   static and dynamic.
    In future if requirements mandates then they will have to be separated */
typedef struct {
    ubit32_t dummy_00 : 1;               /*!< bit 00 */
    ubit32_t dummy_01 : 1;               /*!< bit 01 */
    ubit32_t dummy_02 : 1;               /*!< bit 02 */
    ubit32_t dummy_03 : 1;               /*!< bit 03 */
    ubit32_t dummy_04 : 1;               /*!< bit 04 */
    ubit32_t dummy_05 : 1;               /*!< bit 05 */
    ubit32_t dummy_06 : 1;               /*!< bit 06 */
    ubit32_t dummy_07 : 1;               /*!< bit 07 */
    ubit32_t dummy_08 : 1;               /*!< bit 08 */
    ubit32_t dummy_09 : 1;               /*!< bit 09 */
    ubit32_t dummy_10 : 1;               /*!< bit 10 */
    ubit32_t dummy_11 : 1;               /*!< bit 11 */
    ubit32_t dummy_12 : 1;               /*!< bit 12 */
    ubit32_t dummy_13 : 1;               /*!< bit 13 */
    ubit32_t dummy_14 : 1;               /*!< bit 14 */
    ubit32_t dummy_15 : 1;               /*!< bit 15 */
    ubit32_t dummy_16 : 1;               /*!< bit 16 */
    ubit32_t dummy_17 : 1;               /*!< bit 17 */
    ubit32_t dummy_18 : 1;               /*!< bit 18 */
    ubit32_t dummy_19 : 1;               /*!< bit 19 */
    ubit32_t dummy_20 : 1;               /*!< bit 20 */
    ubit32_t dummy_21 : 1;               /*!< bit 21 */
    ubit32_t dummy_22 : 1;               /*!< bit 22 */
    ubit32_t valid_NoShadowObj : 1;      /*!< bit 23 */
    ubit32_t valid_YIntersecGrad : 1;    /*!< bit 24 */
    ubit32_t valid_MaxAngleGrad : 1;     /*!< bit 25 */
    ubit32_t valid_AngleGradVar : 1;     /*!< bit 26 */
    ubit32_t valid_MeanVarMOT2Curve : 1; /*!< bit 27 */
    ubit32_t valid_YIntersecDelta : 1;   /*!< bit 28 */
    ubit32_t ReachedEgoVeh : 1;          /*!< bit 29 */
    ubit32_t PointNotOnStraightLine : 1; /*!< bit 30 */
    ubit32_t dummy_31 : 1;               /*!< bit 31 */
} FIP_t_MOTQuality;

/*! Structure for the traces from moving objects */
typedef struct {
    float32
        fYVar[FIP_STATIC_TRACE_NO_OF_POINTS]; /*!< fYVar values of trace (m) */
    float32 fLength;                          /*!< length of trace (m) */
    float32 fTraceWithoutObjTime; /*!< time since object has been deleted (s) */
    float32 YIntersecVar;   /*!< intersection variance of trace with y-axis */
    float32 YIntersecDelta; /*!< distance, trace has moved on the y-axis (used
                               for quality analysis) */
    sint8 iBadCycles; /*!< Quality Number for hysteresis at trace deletion */
    FIP_t_MOTQuality Quality; /*!< quality of the trace */
} FIP_t_MovingObjStaticTrace;

/*****************************************************************************
  MACROS
*****************************************************************************/
/*! To get the Quality analysis values from the trace structure */
#define FIP_STATIC_TRACE_GET_QUALITY(iTr) \
    FIP_p_MOSTGetMovingObjectTraces()[iTr].Quality

/*****************************************************************************
  CONSTANTS
*****************************************************************************/
/*! sampling length in m */
#define FIP_MOST_SAMPLING_LENGTH (5.0f)

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/
/*! Initialize the trace parameters */
extern void FIP_v_Init_Static_Traces(void);

/* pointer to function to get the base address of Moving object trace
 * structure*/
extern FIP_t_MovingObjStaticTrace* FIP_p_MOSTGetMovingObjectTraces(void);
/* main function for to compute the VLC moving object traces*/
extern void FIP_v_CalculateMovingObjectStaticTraces(void);

/* Functions common to both static and dynamic traces */
/*get Variance of Object depending on Distance*/
extern float32 FIP_f_GetObjObservationVariance(ObjNumber_t iObj);
/* for updating ego motion attributes */
extern const GDBTrafoMatrix2D_t* FIPGetTrafoMatrix2DCOFFwdTgtSync(void);
/* for updating ego motion attributes */
extern const GDBTrafoMatrix2D_t* FIPGetTrafoMatrix2DCOFForJitTgtSync(void);

/*****************************************************************************
  CUSTOM FUNCTIONS
*****************************************************************************/

#endif /*!< _FIP_OBJECT_TRACES_H_INCLUDED*/
