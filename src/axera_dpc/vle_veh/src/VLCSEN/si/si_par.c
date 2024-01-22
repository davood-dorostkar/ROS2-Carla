/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "si.h"
#include "si_par.h"
const volatile SIRangeReductionParams_t SIRangeReductionParams = {
    8.0f,
    (1.0f / 2000.0f),
    100.0f,
    5.0f,
};

const volatile SIOccupancyParams_t SIOccupancyParams = {
    0.35f, 0.30f,
    0.25f, /* Note: in ARS350 this was set to 0.30, while in base set to 0.35
              t.b.c*/
    0.35f, 0.30f,
};

const volatile SILowSpeedStatPedesParams_t SILowSpeedStatPedesParams = {
    5.0f, 5.0f, 5.0f, 0.2f, 0.0f,
};

const volatile GDBVector2_t
    SI_a_SamplePointDistancePara[SI_NUM_PICKUP_DIST_POINTS] = {
        {(60.f / C_KMH_MS), 100.f},  /*  0-60 kph ->  100 m */
        {(120.f / C_KMH_MS), 180.f}, /* 120-inf kph -> 180 m */
};

/***************************************************/
/*! Calculation of lane change probability */

const volatile SILaneChangeProbParameter_t SILCProbParData = {
    /*! const float32 SI_LC_PROB_TABLE_TURN_LIGHT;
        (s) -> (%) */
    {
        {0.f, 100.f},
        {38.f, 51.f},
        {50.f, 0.f},
    },

    /*! const float32 SI_LC_PROB_TABLE_LAT_VEL_CAM_LANE_MARKER;
       (m/s) -> (&) */
    {
        {0.2f, 0.f},
        {0.7f, 100.f},
    },

    /*! const uint8 SI_LC_PROB_TABLE_COMB_CAM_PROB;
        iLatDiffCamCurveProb, iVelLatCamLaneMarkerProb -> (%) */
    {
        0u,
        20u,
        85u,
        100u,
    },

    /*! const uint8 SI_LC_PROB_TABLE_COMB_ALL_LATDIFFCURVE_TURNLIGHT;
        iLatDiffFilteredCurvesProb, iLCProbTemp, iTurnLightProb -> (%) */
    {
        0u,
        5u,
        10u,
        70u,
        5u,
        70u,
        70u,
        100u,
    },

    /*! const uint8 SI_LC_PROB_TABLE_COMB_LATDIFFCURVE_TURNLIGHT;
        iLatDiffFilteredCurvesProb, iTurnLightProb -> (%) */
    {
        0u,
        50u,
        5u,
        100u,
    },

    /*! const float32 SI_LC_PROB_TABLE_DIFF_LAT_FILT_CURVES;
        (s) -> (%) */
    {
        {0.f, 0.f},
        {0.1f, 10.f},
        {0.2f, 45.f},
        {0.35f, 100.f},
    },

    /*! const float32 SI_LC_PROB_TABLE_DRIVER_INT_CURVE_FILTER_CONST;
         (),   (), */
    {
        {0.0002f, 0.00857142857f}, /* X = 0.30f, -> T = 0,008571428571428571 */
        {0.0009f, 0.38f},          /* X = 0.95f, -> T = 0,38 */
    },

    /*! SI_LC_PROB_TABLE_COMB_LATDIST_TURNLIGHT_LATDIFFCURVE;
        iDistLatProb, iLatDiffFilteredCurvesProb, iTurnLightProb -> (%) */
    {
        0u,
        20u,
        20u,
        60u,
        0u,
        80u,
        90u,
        100u,
    },

    /*! SI_LC_PROB_TABLE_COMB_LOWSPEED_TURNLIGHT;
        iLCProbTemp, iTurnLightProb -> (%) */
    {
        0u,
        98u,
        85u,
        100u,
    },

    /*! SI_LC_PROB_TABLE_COMB_LATDIST_LATDIFFCURVE;
        iLCProbTemp, iLatDiffFilteredCurvesProb, iDistLatProb -> (%) */
    {
        0u,
        60u,
        10u,
        100u,
        0u,
        90u,
        90u,
        100u,
    },

    /*! SI_LC_PROB_TABLE_COMB_MARKERCROSSING,
        iLCProbTemp, iCamLaneMarkerCrossedProb -> (%) */
    {
        0u,
        80u,
        85u,
        100u,
    },

    /*! const float32 SI_LC_PROB_TABLE_LAT_DIST;
       (m)-> (%) */
    {
        {0.3f, 0.f},
        {0.6f, 100.f},
        {1.6f, 100.f},
        {3.5f, 0.f},
    },

    /*! const float32 SI_LC_PROB_TABLE_CURVE
        (1/m) ->  (%) */
    {
        {0.002f, 0.f},
        {0.01f, 100.f},
        {0.018f, 100.f},
        {0.03f, 0.f},
    },

    /*! SI_LC_PROB_TABLE_COMB_CURVE
        iCurveProb -> (%) */
    {
        0u,
        70u,
        30u,
        100u,
    },
};

/*!< Calculation of lane change probability */
/***************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */