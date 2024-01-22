
#ifndef SI_CFG_H_INCLUDED
#define SI_CFG_H_INCLUDED

#include "vlc_config.h"

/*****************************************************************************
  Config: SI (Situation Interpretation)
*****************************************************************************/

/*! time gap for stationary obstacles
    former ACTIVATE_REGELZEITABSTANDSTAT
        *not used* */
#define CFG_SI_TIME_GAP_FOR_STATIONARY 1

/*! time gap for stationary obstacles with moving to stationary
    former ACTIVATE_RZASTAT_MOVSTAT
        *not used* */
#define CFG_SI_TIME_GAP_FOR_MOVING2STAT 1

/*! complex Curve Observer and Range Reduction Factor
    mainly based on mean curve of the last 200m */
#define CFG_SI_CURVEOBSERVER_RANGEREDUCTION 1

/*! Range reduction based on tunnel probability, acc range is reduced
   immediately
    (range factor is set to maximum) if ego-vehicle is driving a curve in a
   tunnel*/
#define CFG_SI_TUNNEL_RANGEREDUCTION 1

/*! configuration switch for enabling road border criteria, causing
 a extension of trace brackets based on the Direction of Curvature. */
#define CFG_SA_USE_ROAD_BORDER_BRACKET_CRITERIA 1

/*! Configuration switch for activating additional call to custom
SICustProcessCriteriaMatrix after all other trace bracket processing
took place. When enabled, you can use the function SICustProcessCriteriaMatrix
for customizing trace brackets
Be careful before activating, because some deleted functions in
si_customfunctions.c! */
#define SI_CFG_CUSTOM_CRITERIA_MATRIX_PROCESSING 0

/*! Configuration switch for activating object-to-object (O2O) relation based
 * ego lane association. */
/* No functional impact, computation only. Uses traces, distance to trajectory
 * (to be removed) and CP extern functions */
#define SI_CFG_O2O_EGO_LANE_ASSOC                              \
    VLC_CFG_DEPENDENT_SWITCH(                                  \
        ((VLC_CFG_OBJECT_TRACE_PREPROCESSSING) &&              \
         (VLC_CFG_COURSE_PREDICTION) && (VLC_CFG_ROAD_INPUT)), \
        SWITCH_OFF)

#define SI_CFG_O2O_EGO_LANE_ASSOC_RANGE_EXTENSION                    \
    VLC_CFG_DEPENDENT_SWITCH(                                        \
        ((SI_CFG_O2O_EGO_LANE_ASSOC) && (CFG_SI_DTR_OBJ_SELECTION)), \
        SWITCH_OFF)

#define SI_CFG_O2O_EGO_LANE_ASSOC_BRACKET_EXTENSION                  \
    VLC_CFG_DEPENDENT_SWITCH(                                        \
        ((SI_CFG_CUSTOM_CRITERIA_MATRIX_PROCESSING) &&               \
         (SI_CFG_O2O_EGO_LANE_ASSOC) && (CFG_SI_DTR_OBJ_SELECTION)), \
        SWITCH_OFF)

/*! Configuration switch to activate additional call to custom
SICustomCorridorPreProcessing , which allows to enable or disable
selected corridor bracket functions before their actual calculation */
#define SI_CFG_CUSTOM_CORRIDOR_PREPROCESSING 1

/*! Configuration switch for enabling custom output type (SICustMeasOOI_t)
output over measurement interface for each OOI object. This allows
application-project specific data to be attached to the OOI measurement
objects for validation or visualization */
#define SI_CFG_CUSTOM_OOI_MEAS_DATA 1

/*! Configuration switch for enabling custom output of the
    collective velocity */
#define SI_CFG_CUSTOM_COLECTIVE_VEL_POSTPROCESSING 0

/*! Configuration switch selecting if road estimation shall be used
for SI course */
#define SI_USE_ROADESTIM (boolean)1

/*! Configuration switch selecting if object traces shall be used for
SI course */
#define SI_USE_OBJTRACES (boolean)1

/*! Configuration switch to enable fusion of camera lane for ACC trajectory */
#define SI_USE_CAM_LANE_FUSION \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_SEN_CAM_LANE_INTERFACE, SWITCH_ON)

/*! Configuration switch for enabling modification (narrowing) of seek lane
width based on lane width class from EM */
#define SI_CFG_SEEK_LANE_WIDTH_USE_LANE_CLASS \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_ROAD_INPUT, SWITCH_ON)

/*! Configuration switch for enabling calibration of seek lane width over
XCP (creation of global variable for overriding default seek width) */
#define CFG_SI_CALIBRATE_SEEK_LANE_WIDTH 0

/*! Configuration switch for enabling modification of occupancy handling for
objects at high angles. Objects seen at great angles have a tendency to have
very high standard deviations leading to very late pickups
see SIGetCorrectedOccupancy */
#define SI_CFG_MANIPULATE_OCCUPANCY_AT_HIGH_ANGLES 1

/*! Configuration switch to enable new object scoring module */
#define SI_CFG_OBJECT_SCORING 0

/* Configuration switch to enable a bracket extension used for highspeed
approaches. For a relevant object which we approach very fast without
reliable road estimation, the brackets are extended */
#define SI_CFG_EXT_HIGHSPEED_APPROACH 1

/*! Configuration switch for enabling NBS Diagnose Test
@todo: Clarify if this is needed! */
#define SI_CFG_NBS_DIAGNOSE_TEST 0

/*! Configuration switch for enabling optimized country road hold performance
for preventing drop-outs on country roads */
#define CFG_SI_OPTIMIZED_OBJECT_HOLD_FEATURES 1

/*! Configuration switch allowing extension of track lane width based on road
border estimation. Useful when rel. objects is merged with mirror in guardrail
*/
#define SI_CFG_GUARDRAILROADBORDER_HIGHWAY 1

/*! Configuration switch allowing limitation of seek lane width based on road
lane width estimation. Useful when road lane width estimation incorporates
fusion
of camera information (for example) */
#define SI_CFG_LIMIT_SEEK_LANEWIDTH_BASED_ON_ROAD \
    (VLC_SEN_CFG_EM_CLD_INPUT || VLC_CFG_SEN_CAM_LANE_INTERFACE)

/*! Configuration switch allowing limitation of seek lane width based on tunnel
 * probability*/
#define SI_CFG_LIMIT_LANEWIDTH_IN_TUNNEL 0

/*! Configuration switch to enable old wide near range track. This was a special
work-around checked in to provide better cut-in recognition in near range. New
simulation on the original measurements no longer shows any benefits */
#define SI_CFG_USE_WIDE_NEAR_TRACEBRACKETS 0

/*! Configuration switch allowing the use of object orientation angle(OOA) in
 * overlap and occupancy calculations for objects that are in NEAR region */
#define SI_CFG_USE_OBJECT_ORIENTATION_NEAR_OBJECTS 0
/*! Configuration switch allowing the use of extended prediction time(EPT) for
 * objects that are in NEAR region */
#define SI_CFG_USE_EXTENDED_PRED_TIME_NEAR_OBJECTS 0
/* Configuration switch to enable a bracket extension used for improved
robustness
against drop-outs in tunnels, since trajectory fusion inputs may be not
available
or less stable in this environment. For a relevant object which is already held
for a
long time the brackets are extended.*/
#define SI_CFG_ADD_EXT_TUNNEL_PROB 1

/* calculate CutinPotential for all neighbor objects, not only for the OOI
 * selceted neighbor objects */
/* (default value for unchanged behaviour 0) */
#define SI_CUTINPOTENTIAL_CALC_FOR_ALL_OBJECTS 0

/*! Configuration switch to enable multi object analyse as one input for the
cutin potential
if enabled, object OOI[3] (and OOI[2]) are compared to other objects ahead to
determine a cutin potential */
#define SI_CUTINPOTENTIAL_MULTIOBJECTANALYSE 0

/*! since CutOutPotential shows lots of false positive events in follow mode on
country road
and in cities, suppress CutOutpotential in these scenarios
(quick solution, long term solution could be general optimization of calculation
of CutOutPotential */
#define SI_CUTOUTPOT_SUPRESS_ON_COUNTRYRAD_CITY 0

/*! Configuration switch, which sets selection of OOI objects 4 & 5 as next
longitudinal
instead of next predicted lateral */
#define SI_CFG_SELECT_OOI_4_AND_5_NEXT_LONG 0

/*! Configuration switch, which sets selection of bicycles and motocycles
on left and right OOI position ON, even if no lane was recognized.
see FR95432 (Zweiräder auch bei nicht vorhandenener Nebenspur als
Nebenspurobjekt [DANTE 1345])*/
#define SI_CFG_NOLANE_BIKE_SELECTION 1

/*! Configuration switch, which sets selection of cars and trucks
on left and right OOI position ON, even if no lane was recognized.
see Issue 321998 (Traktor auf schmalem Standstreifen auf zweispuriger Landstraße
erhält keine Vorselektion als rechtes Nebenspurobjekt [DANTE 5244])*/
#define SI_CFG_NOLANE_VEHICLE_SELECTION 0

/*! Configuration switch, which moves the trace brackets based on the ego
   position in lane;
    only active in cmbination with the camera lane interface */
/*! In case of a lane change: */
#define SI_CFG_ADAPT_TRACEBRACKETS_LC_BASED_ON_POS_INLANE_CAM                 \
    VLC_CFG_DEPENDENT_SWITCH(                                                 \
        ((VLC_CFG_INPUT_PREPROCESSSING) && (VLC_CFG_SEN_CAM_LANE_INTERFACE)), \
        SWITCH_ON)
/*! In case of no lane change: */
#define SI_CFG_ADAPT_TRACEBRACKETS_NOLC_BASED_ON_POS_INLANE_CAM               \
    VLC_CFG_DEPENDENT_SWITCH(                                                 \
        ((VLC_CFG_INPUT_PREPROCESSSING) && (VLC_CFG_SEN_CAM_LANE_INTERFACE)), \
        SWITCH_ON)
/*! In case of the blinker feature (VLC_CFG_CUSTOM_IO_INTERFACE need to be
 * switched on): */
#define SI_CFG_ADAPT_TRACEBRACKETS_BLINKER_FEATURE_BASED_ON_POS_INLANE_CAM     \
    VLC_CFG_DEPENDENT_SWITCH(                                                  \
        ((VLC_CFG_INPUT_PREPROCESSSING) && (VLC_CFG_SEN_CAM_LANE_INTERFACE) && \
         (VLC_CFG_CUSTOM_IO_INTERFACE)),                                       \
        SWITCH_ON)
/*! Use the BMW blinker feature
   (SI_CFG_ADAPT_TRACEBRACKETS_BLINKER_FEATURE_BASED_ON_POS_INLANE_CAM need to
   be switched on);
    otherwise the turn indicator signal is used */
#define SI_CFG_ADAPT_TRACEBRACKETS_USE_BMW_BLINKER_FEATURE                  \
    VLC_CFG_DEPENDENT_SWITCH(                                               \
        SI_CFG_ADAPT_TRACEBRACKETS_BLINKER_FEATURE_BASED_ON_POS_INLANE_CAM, \
        SWITCH_OFF)

/*! Configuration switch, which enables use of camera information for lane width
 * detection */
#define SI_CFG_ADAPT_LANEWIDTH_BASED_ON_CAM 1

/*! Configuration switch, which enables corridor extension based on camera lane
 object association
 Only to be enabled in case of camera lane information and camera object lane
 association
 -> (VLC_CFG_INPUT_PREPROCESSSING) && (VLC_CFG_SEN_CAM_LANE_INTERFACE) &&
 (VLC_CFG_CAMERA_OBJECT_DETECTION) */
#define CFG_SI_CAM_LANE_OBJ_ASSOC_EXTENSION 0

/*! Configuration switch to enable bracket extension used for situations in
which the relevant object moves in the direction of a near neighboring object.
For the relevant object the brackets are extended. Should only be active
if camera lane information is available to detect the ego lane change phases. */
#define SI_CFG_EXT_NEIGHBORHOOD_REL_OBJ                                       \
    VLC_CFG_DEPENDENT_SWITCH(                                                 \
        (SI_CFG_ADAPT_TRACEBRACKETS_LC_BASED_ON_POS_INLANE_CAM &&             \
         SI_CFG_ADAPT_TRACEBRACKETS_NOLC_BASED_ON_POS_INLANE_CAM &&           \
         SI_CFG_ADAPT_TRACEBRACKETS_BLINKER_FEATURE_BASED_ON_POS_INLANE_CAM), \
        SWITCH_OFF)

/*! Extend trace brackets for low ego velocities and missing adjacent lanes.
  Since low speeds are currently a problem for lanematrix, check with fused road
  border if a driveable lane exists */
#define SI_CFG_EXT_LOWSPEEDFUSEDBRD 1

/*! Configuration switch to enable occlusion calculation usage for inlane
 * decision */
#define SI_USE_OBJ_OCCLUSION_LANEASSOCIATION 1

/*! Configuration switch enabling increased in-lane time for objects with micro
doppler signature
Prerequisite: active micro doppler computation in EM
(CFG_Envm_PED_MICRO_DOPPLER)
and updated interface */
#define SI_CFG_MICRODOPPLER_PEDESTRIAN_HIGH_INLANE_TIME 0

/*! Configuration switch to enable cluster variance usage for inlane decision */
/*! Reduces drop-ins on difficult highways */
/* For objects with high cluster variance: */
/* 1. increased object occupany pick-up threshold */
/* 2. increased object occupany drop threshold */
/* 3. increased object inlane-time */
/* 4. no predicted lane association */
#define SI_USE_CLUSTER_VARIANCE_IN_LANEASSOCIATION \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_ROAD_INPUT, SWITCH_ON)

/*!> Configuration switch to enable situative increase of inlane timer as a
  function of relative velocity, curvature, and distance */
#define SI_SIT_VREL_INLANE_TIME_INC 1

/*!> Configuration switch to enable increase of inlane timer as a
  function of approximate time-to-collision (distance divided by relative
  velocity) */
#define SI_TTC_DEPENDANT_INLANE_TIME 0

/*! Configuration switch to enable suppression of objects at high velocities
 * when ego is on the left lane */
/*! This feature is useful for german autobahn and above 140 kph. It should be
 * made dependent on traffic direction. */
/* When an object is detected and there is no other object which could be
 * overtaken by it, */
/* increase the in-lane timer */
#define SI_HIGH_VEGO_INLANE_TIME                                       \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_LANE_MATRIX_INPUT_PREPROCESSSING, \
                             SWITCH_ON)

/*! Configuration switch to enable suppression of trucks at high distances when
 * ego is on the left lane */
/*! This feature is useful for german autobahn and above 140 kph. It should be
 * made dependent on traffic direction. */
/* When a truck is detected and there is no other object which could be
 * overtaken by it, */
/* set the pick-up distance to a minimum */
#define SI_HIGH_DIST_TRUCK_SUPPRESS                                    \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_LANE_MATRIX_INPUT_PREPROCESSSING, \
                             SWITCH_ON)

/*! Configuration switch to enable road-VDY distance/delta and road fusion
 * dependent range extension */
/* When road is fused into the trajectory it compensates the range reducing
 * influence of the range factor */
/* and thus enables a better pick-up distance on motorways */
#define SI_ROAD_VDY_DEPENDANT_RANGE_EXTENSION \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_ROAD_INPUT, SWITCH_OFF)

/*! Configuration switch to enable road-VDY distance/delta dependent range
   reduction
   Measure against rare drop-ins in far range when road has strong deviation
   from VDY
   and falsifies the trajectory*/
#define SI_ROAD_VDY_DEPENDANT_RANGE_REDUCTION \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_ROAD_INPUT, SWITCH_ON)

/*! Configuration switch to parametrize road-vdy distance/delta dependent range
 * reduction */
/* If switched on, the difference between road and vdy is measured for the near
  range,
  switched off, the difference is measured in the far range */
#define SI_ROAD_VDY_DEPENDANT_RANGE_REDUCTION_USE_NEAR_RANGE 1

/*! Configuration switch to enable a road and vrel dependent range reduction. */
/* The range is reduced whenever the road estimation is not available and/or  */
/* an object is only slowly approaching or faster, respectively.  */
#define SI_VDY_ONLY_VREL_DEPENDENT_REDUCTION 1

/*! Configuration switch to enable extended CurveInnerBorder criterion */
/* in very narrow curves extend the trace bracket on the curve inner side beyond
 * the FOV */
/* currently extension up to azimuth angle 39 degree on both sides) */
/* fixes parts of Dante1368 - dropout in traffic circle */
#define SI_EXTENDED_CURVEINNERBORDER_CRITERION 1

/* do not apply wide bracket criteria in case of steering gradient is leading
 * out of curve and there is another lane available */
/* fixes some special situation with late release of a cutout object */
/* but also causes some drop outs in curvy following / approach situation */
#define SI_EXTENDED_CURVEINNERBORDER_CRITERION_WEAKER \
    (SI_EXTENDED_CURVEINNERBORDER_CRITERION && SWITCH_OFF)

#define SI_CFG_ADD_EXTENSION_OBJ_APPROX 0

/*! Configuration switch to enable restriction based on relevant object trace */
/* This restriction is aimed at improving cutouts */
#define SI_CFG_ADD_RESTRICTION_OBJ_TRACE                               \
    VLC_CFG_DEPENDENT_SWITCH(                                          \
        (VLC_CFG_OBJECT_TRACE_PREPROCESSSING) && (VLC_CFG_ROAD_INPUT), \
        SWITCH_ON)

/*! @brief       Switch to enable restriction based on stationary objects in
   next lane
    @general     To avoid stationary objects which are on next lane being
   selected as OOI, restrictions are added to its corridor.
    @conseq      @incp  Restriction is ENABLED
                 @decp  Restriction is DISABLED
    @attention   Only applies to objects that are stationary, approach to such
   such can be negatively affected.
    @typical     0   @unit NO-UNIT     @min 0   @max 1   */
#define SI_CFG_ADD_RESTRICTION_STATOBJ_NEXTLANE 0

/*! Configuration switch to enable a restriction in the near range for country
road /city scenarios
to improve the release of objects that take a turn */
#define SI_CFG_RESTRICTION_CITY_NEAR_RANGE 1

#define SI_CFG_EXT_RELEVANT_OBJECT 1

/*! Configuration switch to enable/disable the calculation of the traffic
 * density (si_trafficdensity.c) */
#define SI_CFG_TRAFFIC_DENSITY 0

/*! Configuration switch to enable/disable the calculation of a traffic flow and
 * traffic jam probability (si_traffic_estimation.c) */
#define SI_CFG_TRAFFIC_ESTIMATION 0

/*! Calculation of the traffic jam probability given only the velocity of the
    object-of-interest OOI-0 */
#define SI_TE_CALC_TJP_OOI_ONLY 0

/*! Calculation of the traffic jam probability given the velocity of the
    object-of-interest OOI-0 as well as the averaged velocity of the
    surrounding objects */
#define SI_TE_CALC_TJP_COLLECTIVE 0

/*---------------------------------Sample point selection for pickup
 * distance-----------------------------------------------------*/
/*! Configuration switches to select the different sets of sample points for the
 * pickup distance */
#define SI_CFG_CUSTOM_IO_INTERFACE_PRESEL_DISTANCE                          \
    0 /*!< Switch to enable selecting a different parameter set during the  \
        operation                                                           \
        (i.e. the parameter set is changed by the driver when selecting the \
        ECO-Pro-Mode)*/

/* The default set of sample points has to be selected below (only one)*/
#define SI_CFG_BASE_PRESEL_DISTANCE 1
#define SI_CFG_ACCLOW_PRESEL_DISTANCE 0
#define SI_CFG_ACCHIGH_PRESEL_DISTANCE 0
#define CFG_SI_DTR_OBJ_SELECTION 0
#define CFG_SI_TRUCK_OBJ_SELECTION 0
#if ((SI_CFG_BASE_PRESEL_DISTANCE + SI_CFG_ACCLOW_PRESEL_DISTANCE + \
      SI_CFG_ACCHIGH_PRESEL_DISTANCE + CFG_SI_DTR_OBJ_SELECTION +   \
      CFG_SI_TRUCK_OBJ_SELECTION) != 1)
#error \
    "too many switches are selected for the si pickup distance parameter sets!"
#endif
/*--------------------------------------------------------------------------------------------------------------------------------*/

/*! Configuration switch for treating stationary objects as oncoming */
#define SI_CFG_CHECK_STAT_OBJ_WAS_ONCOMING 1

/*! Limit the inner curve border extension based on the curvature of the course
 * and the position of the object */
#define SI_CFG_LIMIT_CURVE_INNER_BORDER_EXT 1

/*! Configuration switch enabling additional object quality check for objects
and only allowing
them to become in-lane when the function SICheckObjInlaneAllowed() returns TRUE
*/
#define SI_CFG_ADD_CHECK_INLANE_TRANSITION 1

/*! Configuration switch enabling lane change probability calculation */
#define SI_CFG_LC_PROB_CALC 1

/*! Configuration switch enabling to use traffic orientation within the lane
  change
  probability. */
#define SI_LC_USE_TRAFFIC_ORIENTATION 1

#define SI_USE_NAVI_PATH_FUSION (boolean)0

/*! Configuration switch enabling the trace bracket adaption based on the navi
 * information */
#define SI_CFG_NAVI_COUNTRYROAD_EXTENSION 0
#define SI_CFG_NAVI_OBJ_CLOSE_TO_EXIT_RESTRICTION 0

/*! Configuration switch enabling the trace bracket extension for the blocked
 * path between an object and an obstacle */
/* Only if VLC_CFG_COURSE_PREDICTION is ON will the blocked obstacle extension
 * be usable */
#define SI_CFG_BLOCKED_OBSTACLE_EXTENSION 0

/*! SI OOI with debug TTC/TG information in meas-freeze (simplifies quick
performance evaluation) */
#define SI_CFG_OOI_TTC_TG_DEBUG_CRITERIA 1

/*! Configuration switch enabling the drive off monitor */
#define SI_CFG_DRIVE_OFF_MONITOR 0
/*! Configuration switch enabling the activation prevention */
#define SI_CFG_ACTIVATION_PREVENTION 0

/*! drops object at lanechange when it is outside the pickup distance (but
 * inside the hysteresis) */
#define SI_CFG_HYSTERESIS_DROP 0

/*! Configuration switch to use pickup border values decreasing with distance */
#define SI_CFG_DECREASING_PICKUP_BORDER 0
/*! Configuration switch to delay the predicted lane association */
#define SI_CFG_DELAY_PRED_LANE_ASSOC 0

/*! Configuration switch enabling ObjectLossReason at low distance */
#define SI_CFG_OBJLOSS_LOWDIST 0

#endif
