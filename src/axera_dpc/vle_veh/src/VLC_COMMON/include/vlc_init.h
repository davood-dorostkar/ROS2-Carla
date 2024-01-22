

#ifndef VLC_INIT_H
#define VLC_INIT_H

/*****************************************************************************
  INCLUDES
*****************************************************************************/

/*****************************************************************************
  SYMBOLIC CONSTANTS (COMPONENT INTERN)
*****************************************************************************/

/*****************************************************************************
  SYMBOLIC CONSTANTS (COMPONENT EXTERN)
*****************************************************************************/

#define SI_CURVEOBS_INIT_NAME "SICurveObsInit"

/*****************************************************************************
  MACROS (COMPONENT EXTERN)
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS (COMPONENT EXTERN)
*****************************************************************************/

typedef struct {
    boolean bRangeFacAvail;
    float32 fRangeFac;

    boolean bDrivenDistAvail;
    float32 fDrivenDist;

    boolean bSumCurveAvail;
    float32 fSumCurve;

    boolean bNumberOfSamplesAvail;
    sint32 iNumberOfSamples;

    boolean bMeanCurveHistAvail;
    float32 fMeanCurveHist[5];

    boolean bComplexRangeFacAvail;
    float32 fComplexRangeFac;

    boolean bDistRoad2VDYAbsAvail;
    float32 f_RoadVDYDeltaFar;
    float32 f_RoadVDYDeltaNear;

} LACurveObsInitData_t;

/*****************************************************************************
  CONSTANTS (COMPONENT EXTERN)
*****************************************************************************/

/*****************************************************************************
  VARIABLES (COMPONENT EXTERN)
*****************************************************************************/

/*****************************************************************************
  FUNCTIONS (COMPONENT EXTERN)
*****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
};
#endif

/* End of conditional inclusion */
#else
#endif
