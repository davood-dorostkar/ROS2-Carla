

#ifndef SI_PAR_H_INCLUDED
#define SI_PAR_H_INCLUDED

#include "TM_Global_Types.h"
#include "stddef.h"

/*! Tunnel probability threshold, above which tunnel specific behaviour @min:0
 * @max: 1 */
#define SI_TUNNEL_PROB_THRES 0.50f

/*! Velocity threshold to ignore the roadtype Highway for the guardrail
 * extension*/
#define SI_GUARDRAILEXT_V_THRES (80.f / C_KMH_MS)

/*parameters for acceleration/deceleration authority*/
/*! defines max accel authority */
#define Si_auth_max_accel (4.0f)
#define Si_auth_max_decel (-6.0f)
#define Si_auth_max_decel_stationary (0.0f)
// #define Si_auth_max_decel_hidden (-1.0f)
#define Si_auth_max_decel_adjacent (-6.0f) /*decel for ttlc=0s*/
#define Si_auth_min_decel_adjacent (4.0f)  /*decel for ttlc=6s*/
#define Si_auth_adjacent_max_ttlc (6.0f)   /*max ttlc (for 4m/s?*/

/* SI specific vehicle and road parameters */
#define SI_FAHRZEUGBREITE (1.8f)
#define SI_FAHRZEUG_SAFETY_CORRIDOR (0.4f)

#define SI_STATIONARY_STRASSENBREITE (2.5f)

#define STANDARDLANEWIDTHSEEK (STRASSENBREITE)
#define STANDARDLANEWIDTHTRCK (STANDARDLANEWIDTHSEEK + 0.5F)

#define CITYLANEWIDTHSEEK (2.7F)
#define CITYLANEWIDTHTRCK (CITYLANEWIDTHSEEK + 0.75F)

#define TUNNEL_LANEWIDTHSEEK (2.7F)
#define TUNNEL_LANEWIDTHTRCK (TUNNEL_LANEWIDTHSEEK + 0.5F)

#define ROADWORKSLANEWIDTHSEEK (2.25f) /*(STRASSENBREITE - 1.5F)*/
#define ROADWORKSLANEWIDTHTRCK (ROADWORKSLANEWIDTHSEEK + 0.2F)

/* Maximum lanewidth measured by cam indicating that ego-vehicle is on
 * lane-class NARROW  */
#define SI_CAM_LANEWIDTH_NARROW_LANE_MAX (3.35)
/* Maximum lanewidth measured by cam indicating that ego-vehicle is on
 * lane-class MORE_NARROW  */
#define SI_CAM_LANEWIDTH_MORE_NARROW_LANE_MAX (2.85)
/* Maximum ego-speed, up to which the camera based reduction of seeklanewidth is
 * applied */
#define SI_CAM_LANEWIDTH_MAX_SPEED (140.0 / C_KMH_MS)

/*! Factor used for scaling seek lane width if EO lane class is narrow
@min: 0.5 @max: 1.0 */
#define SI_SEEK_LANE_WIDTH_FACTOR_NARROW_LANE (0.9f)
/*! Factor used for scaling seek lane width if EO lane class is narrower
@min: 0.5 @max: 1.0 */
#define SI_SEEK_LANE_WIDTH_FACTOR_NARROWER_LANE (0.85f)

/*! Maximum tolerance factor from Road's estimated lane width to the seek
lane width. (E.g.: if set to 1.25f, that means the used seek lane width
may be maxmium 25% greater or 25% smaller than the lane width) @min:1 @max:2 */
#define SI_PAR_SEEK_WIDTH_TO_CAM_LANE_WIDTH_TOLERANCE (1.25f)

/*! The tracked objects maximum additional lane width */
#define SI_HIGHWAYLANEWIDTH_TRCK_OFFSET_MAX (1.2f)
/*! The tracked objects minimum additional lane width */
#define SI_HIGHWAYLANEWIDTH_TRCK_OFFSET_MIN (0.9f)

/*! The seek lane width corresponding to the ego speed of
 * #SI_MAX_SPEED_HIGWAYLANEWIDTH */
#define SI_HIGHWAYLANEWIDTHSEEK_MAX (3.55f)
/*! The seek lane width corresponding to the ego speed of
 * #SI_MIN_SPEED_HIGWAYLANEWIDTH */
#define SI_HIGHWAYLANEWIDTHSEEK_MIN (3.0f)
/*! The ego speed for maximum seek lane width on highways */
#define SI_MAX_SPEED_HIGWAYLANEWIDTH (160.0f / C_KMH_MS)
/*! The ego speed for minimum seek lane width on highways */
#define SI_MIN_SPEED_HIGWAYLANEWIDTH (130.0f / C_KMH_MS)

/* Minimum confidence of road estimation (RE) */
#define MIN_GUARDRAIL_ROAD_CONFIDENCE (0.F)
/* RE range needs to reach at least x meter behind object */
#define MIN_GUARDRAIL_ROAD_RANGE_OFFSET (10.F)
/* OFfset of trace bracket behind RE */
#define MAX_GUARDRAIL_ROAD_HALF_OBJ_WIDTH (2.5F)
/* maximum offset of guardrail at ego vehicle on highway*/
#define MAX_GUARDRAIL_OFFSET_HIGHWAY (5.8F)
/* maximum offset of guardrail at ego vehicle on highway*/
#define MAX_GUARDRAIL_OFFSET_ROADWORKS (2.25F)

/*! The scaling factor used for the occupancy standard deviation value at
 * SI_STDDEV_OBJ_OVLC_ANGLE_MAX */
#define SI_STDDEV_OBJ_OVLC_FAC_MAX (1.0f)
/*! The scaling factor used for the occupancy standard deviation value at
 * SI_STDDEV_OBJ_OVLC_ANGLE_MIN */
#define SI_STDDEV_OBJ_OVLC_FAC_MIN (0.4f)
/*! The angle where the SI_STDDEV_OBJ_OVLC_FAC_MAX scaling factor is used */
#define SI_STDDEV_OBJ_OVLC_ANGLE_MAX (8.0f)
/*! The angle where the SI_STDDEV_OBJ_OVLC_FAC_MIN scaling factor is used */
#define SI_STDDEV_OBJ_OVLC_ANGLE_MIN (15.0f)

#define SI_OCCLUSION_MAX_DIST \
    (90.f) /*!< Maximal distance for occlusion consideration */
#define SI_OCCLUSION_MIN_DIST_NO_HIGHWAY \
    (30.f) /*!< Minimal distance for occlusion consideration on no highway*/
#define SI_OCCLUSION_MIN_VEL_NO_HIGHWAY                                       \
    (20.f / C_KMH_MS) /*!< Minimal velocity for occlusion consideration on no \
                         highway*/
#define SI_PAR_OVLC_HIGHWAY_INLANE_TIME_FAC                                  \
    (5.f) /*!< In-lane threshold time factor when occlusion is detected on a \
             highway */
#define SI_PAR_OVLC_NO_HIGHWAY_INLANE_TIME_FAC                                \
    (2.f) /*!< In-lane threshold time factor when occlusion is detected on no \
             highway */

#define SI_MAX_OBJ_DEV_PRED_LA                                         \
    (12u) /*!< Maximum cluster spread of object to calculate predicted \
             inlane criteria */
#define SI_MIN_DIST_OBJ_DEV_DISTX                                            \
    (50.f) /*!< Minimum longitudinal distance to consider cluster spread for \
              predicted inlane criteria */

/*****************************************************************************
  Tracebracket parameters (used in si_corridor_trackwidth.c)
*****************************************************************************/
#define SI_TB_EXT_MIN_VELOCITY (110.0f / C_KMH_MS)
#define SI_TB_EXT_MAX_CURVATURE (0.0006f)

/* ***********************************************************************/ /*!
  @brief            OOI Selection Parameters

  @description     The following section inherits all parameters necessary for
object selection as OOI.

  @todo            -

**************************************************************************** */

/*! defines the prediction time for lateral displacement prediction (using
 * lateral velocity)*/
#define SiLatDisplPredictionTime (1.5f)

/*! defines requested acceleration difference between next and hidden next
 * object where the potential for the hidden next object shall be 100%*/
#define SiMaxPotentialDifference (-2.0f)

/*! defines requested acceleration difference between next and hidden next
 * object where the potential for the hidden next object shall be 0%*/
#define SiMinPotentialDifference (0.5f)

/*! defines predicted distance of and object where the cutin potential for the
 * object shall be 100%*/
#define SiMaxPotentialPDist (0.0f)

/*! defines predicted distance of and object where the cutin potential for the
 * object shall be 0%*/
#define SiMinPotentialPDist (2.0f)

/*! defines prediction time (of lateral displacement) for cut in potential
 * estimation (s)*/
#define SiCutInPotPredTime (1.0f) /*SiLatDisplPredictionTime*/
#define SiCutInPotPredTimeLimited (0.5f)
#define SiLaneAssPredTime (1.0f)

#define SiMaxDistPredictionTime (2.1f)
/*! maximum distance for cut out potential calculation */
#define SiMaxDistCutOut (70.f)

/*! defines the maximum and minimum prediction times for cut out potential */
#define SI_CUT_OUT_PRED_MAX (1.0f)
#define SI_CUT_OUT_PRED_MIN (0.5f)

/*! defines the maximum and minimum distances (in time!!!) for maximum and
 * minimum prediction time for cut out potential */
#define SI_CUT_OUT_DIST_MAX (3.0f)
#define SI_CUT_OUT_DIST_MIN (2.0f)

#define SI_MAX_VEHICLE_SPEED_LIMIT_PREDICTION (60.f / C_KMH_MS)
#define SI_TOLERABLE_CUT_IN_ERROR (0.45f)
#define SI_TOLERABLE_CUT_OUT_ERROR (0.45f)

/*! This factor scales down the variances of Vrel to course after start of new
 * distance to course filtering in SA */
#define SI_VARIANCE_SCALE_FACTOR (0.1272727272727F)

/* Cut-In and Cut-Out potential parameters */
/*! lines for standard cut out potentials */
#define Y_DIST_MIN_CUT_OUT_POT (0.0f)
#define Y_DIST_MAX_CUT_OUT_POT HALBESTRASSENBREITE

/*! lines for cut out potentials in road works */
#define Y_DIST_MIN_CUT_OUT_POT_ROADWORKS (0.0f)
#define Y_DIST_MAX_CUT_OUT_POT_ROADWORKS HALBESTRASSENBREITE

/*! lines for cut out potentials in city traffic */
#define Y_DIST_MIN_CUT_OUT_POT_CITY (0.0f)
#define Y_DIST_MAX_CUT_OUT_POT_CITY HALBESTRASSENBREITE

/*! Old cut-in potential algo lines for standard cut in potentials (ARS310: 3.1
 * and 2.2 at default) */
#define Y_DIST_MIN_CUT_IN_POT \
    (SI_FAHRZEUGBREITE + SI_FAHRZEUG_SAFETY_CORRIDOR + 0.9f)
#define Y_DIST_MAX_CUT_IN_POT (SI_FAHRZEUGBREITE + SI_FAHRZEUG_SAFETY_CORRIDOR)

/*! Old cut-in potential algo lines for cut-in potentials in roadworks (ARS310:
 * 1.86 and 1.32 at default) */
#define Y_DIST_MIN_CUT_IN_POT_ROADWORKS \
    (Y_DIST_MIN_CUT_IN_POT * (ROADWORKSLANEWIDTHSEEK / STANDARDLANEWIDTHSEEK))
#define Y_DIST_MAX_CUT_IN_POT_ROADWORKS \
    (Y_DIST_MAX_CUT_IN_POT * (ROADWORKSLANEWIDTHSEEK / STANDARDLANEWIDTHSEEK))

/*! Old cut-in potential algo lines for cutin potentials in city traffic
 * (ARS310: 2.31 and 1.64 at default) */
#define Y_DIST_MIN_CUT_IN_POT_CITY \
    (Y_DIST_MIN_CUT_IN_POT * (CITYLANEWIDTHSEEK / STANDARDLANEWIDTHSEEK))
#define Y_DIST_MAX_CUT_IN_POT_CITY \
    (Y_DIST_MAX_CUT_IN_POT * (CITYLANEWIDTHSEEK / STANDARDLANEWIDTHSEEK))

#define SI_TOLERABLE_STDDEV_CI_CO_POT (0.20f)

/*! only calculate potential for object with absolute longitudinal velocity
 * higher CUTINPOTENTIAL_MIN_OBJ_ABSVEL */
#define CUTINPOTENTIAL_MIN_OBJ_ABSVEL (0.0f / C_KMH_MS)

/*! maximum road border standard deviation */
#define SI_MAX_ROAD_BORDER_STDDEV (3.5f)

/*! definitions for  NEAR Range track width rise */
#define SI_EGO_SPEED_MAX_FOR_WIDE_NEAR_TRACK (25.0f / C_KMH_MS)
#define SI_MIN_DIST_FOR_FOR_WIDE__NEAR_TRACK (10.0f)
#define SI_MAX_DIST_FOR_FOR_WIDE__NEAR_TRACK (15.0f)
#define SI_WIDE_NEAR_TRACK_WIDTH (3.75f)
#define SI_WIDE_NEAR_OBJECT_WIDTH (1.8f)
#define SI_EGO_SPEED_MIN_FOR_WIDE_NEAR_TRACK (15.0f / C_KMH_MS)
#define SI_WIDE_NEAR_TRACK_FACTOR_MIN (0.0f)
#define SI_WIDE_NEAR_TRACK_FACTOR_MAX (1.0f)

/*! crossing traffic only reported to 6 object interface below the followoing
 * distance */
#define SI_MAX_DIST_FOR_CROSSING_REPORT (0.0f)

/*! select bikes as neighbouring object even if there is no lane detected*/
#define SI_MOVING_CLIP_BIKE_ROAD_BORDER_ABSDIST                     \
    (2.50f) /*<! considered distance to roadborder if no roadborder \
               estimation available*/
#define SI_MOVING_CLIP_BIKE_ROAD_BORDER_MAX                           \
    (2.0f *                                                           \
     SI_MOVING_CLIP_BIKE_ROAD_BORDER_ABSDIST) /*<! maximal considered \
                                                 distance to roadborder */

/*! in tunnels select oncomming objects on adjacent lanes up to this distance to
 * trajectory */
#define SI_ONCOMMING_MIN_DIST2ROADBORDER_TUNNEL (2.F * STRASSENBREITE)

/*! parameter specifying minimum for road border stat left/right for discarding
oncomming objects that are outside of the road border estimation */
#define SI_ONCOMMING_ROAD_BORDER_CLIP_MIN_STAT (100u)

#define SI_ONCOMING_CLIP_ABSDIST (9.0f)

/*****************************************************************************
  New cut-in potential parameters
*****************************************************************************/

/*! Switch to enable use of old cut-in potential. If this switch is set, then
the new algorithm is not used */
#define SI_USE_OLD_CUT_IN_POTENTIAL_ALGO SWITCH_OFF
/*! Switch to enable use of minimum of old and new cut-in potential value. This
switch only has an effect if SI_USE_OLD_CUT_IN_POTENTIAL_ALGO is off */
#define SI_USE_MIN_OF_OLD_AND_NEW_CUT_IN_POT_ALGO SWITCH_OFF

/*! Minimal prediction time used for predicted lane association */
#define SI_MIN_LANE_ASSOC_PRED_TIME (0.1f)

/* Lower prediciton time in free-drive mode to avoid drop-ins e.g. 0.25 */
#define SI_MAX_LANE_ASSOC_PRED_TIME_LOW (0.2f)

/*! Maximal prediction time used for predicted lane association */
#define SI_MAX_LANE_ASSOC_PRED_TIME (0.5f)

/*! Minimal prediction time used for cut-in potential calculation */
#define SI_MIN_CUT_IN_PRED_TIME (0.1f)
/*! Maximal prediction time used for cut-in potential calculation */
#define SI_MAX_CUT_IN_PRED_TIME (0.5f)
/*! Factor used to decrease prediction time depending on object distance */
#define SI_PRED_TIME_TEST_FACTOR (0.75f)
/*! Relative velocity variance maximum over which prediction time is set to
 * SI_MIN_CUT_IN_PRED_TIME */
#define SI_REL_VEL_TRAJ_VAR_MAX (5.f)

/*! In cut-in calculation : (Ego vehicle width + Object Width)/2 plus this value
is taken to
be the 100% cut-in potential line. Note: due to issues with trucks, this value
is not added
for trucks */
#define SI_CUT_IN_MAX_POT_OFFSET                                              \
    (0.25f) /* Object widths plus this value are 100% potential in near range \
             */
/*! In cut-in calculation : (Ego vehilce width + Object Width)/2 plus this value
is the 0%
potential line. */
#define SI_CUT_IN_MIN_POT_OFFSET (1.5f)
/*! The cut-in ramp is modified depending on object distance (determined in time
to reach
the given point). This define specifies the time from which the lines shall not
be modified */
#define SI_DIST_RAMP_FULL_PRED_TIME (1.0f)

/*! The minimum ramping factor to use in cut-in calculation reducation based on
object
distance @see also SI_DIST_RAMP_FULL_PRED_TIME */
#define SI_MIN_CUT_IN_POT_SCALE_FACTOR (0.625f)
/*! The maximum ramping factor to use in cut-in calculation, always leave on
 * 1.0! */
#define SI_MAX_CUT_IN_POT_SCALE_FACTOR (1.0f)

/*! The radius of the current course curve from where curvature has no influence
on
cut-in potential (i.e: behaves as if road was straight). Below this curvature
the
cut-in potential is reduced */
#define SI_CUT_IN_CURV_RADIUS_MAX (2000.f)
/*! The radius of the current course curve from where curvature has maximal
influence
(factor of SI_CUT_IN_CURV_SCALE_FACTOR_MIN) on cut-in potential. */
#define SI_CUT_IN_CURV_RADIUS_MIN (700.f)
/*! The maximum scaling factor the course curvature has on the cut-in potential
 */
#define SI_CUT_IN_CURV_SCALE_FACTOR_MIN (0.75f)

#define SI_CUT_IN_CROSS_DIST_MAX (50.f)
#define SI_CUT_IN_CROSS_DIST_MIN (15.f)
#define SI_CUT_IN_CROSS_SCALE_MIN (0.77f)

/*! If an object is on the border of the far beam range, then aritifical
decrease it's
cut-in potential. This decrease takes place if the object is between the
distance
specified below, and the function OTGetObjFOVOverlapFar returns TRUE */
#define SI_NEAR_FAR_LIMIT_MIN_DIST (10.f)
#define SI_NEAR_FAR_LIMIT_MAX_DIST (75.f)
/*! The amount in meters the predicted position is shifted outwards for objects
 * near beam border */
#define SI_REFLECTION_SHIFT_WIDTH_FACTOR (0.10f)

/*****************************************************************************
  Multi Object Observer parameters
*****************************************************************************/

/*! The minimum ego-speed to use the Multi Object Observer */
#define MOR_ACTIVE_MIN_SPEED (16.6F)

/*! The maximal distance to the Object to use the Multi Object Observer */
#define MOR_ACTIVE_MAX_DIST (100.F)

/*! The maximal distance between the Objects to use the Multi Object Observer
(we can not give a reliable prediction for the curvature between both Objects)
*/
#define MOR_ACTIVE_MAX_REL_DIST (75.F)

/*!  maximal timegap for selection of object ahead */
#define MOR_TIMEGAP_SELECTION_OBJ_AHEAD (2.0f)

/*! The maximal relative velocity to the Objects to use the Multi Object
Observer
(necessary to get a symmetrical Algorithm for left and right Lane)*/
#define MOR_ACTIVE_RAMP_REL_VEL_MIN (-0.5F)
#define MOR_ACTIVE_RAMP_REL_VEL_MAX (0.F)
#define MOR_ACTIVE_RAMP_REL_VEL_LENGTH \
    (MOR_ACTIVE_RAMP_REL_VEL_MAX - MOR_ACTIVE_RAMP_REL_VEL_MIN)

/*! The Offset for the lateral Distance of the Object within the lateral
Displacement can be bigger than
the lateral Displacement of the Object ahead */
#define MOR_ACTIVE_RAMP_OFFSET_LAT_MIN (0.25F)
#define MOR_ACTIVE_RAMP_OFFSET_LAT_MAX (0.5F)
#define MOR_ACTIVE_RAMP_OFFSET_LAT_LENGTH \
    (MOR_ACTIVE_RAMP_OFFSET_LAT_MAX - MOR_ACTIVE_RAMP_OFFSET_LAT_MIN)

/*! The minimum distance necessary between the Object and Object ahead to use
 * the Multi Object Observer */
#define MOR_ACTIVE_MIN_REL_DIST (5.F)

/*! Estimated length of Cars and Trucks added to the distance to calculate the
 * TTC more precisely */
#define MOR_OFFSET_LEN_CAR (2.F)
#define MOR_OFFSET_LEN_TRUCK (5.F)

/*! Withim this distance from a Truck we assume Objects except Cars and Trucks
 * to be reflections of the drivers cab */
#define MOR_MAX_DIST_TRUCK_CAB (20.F)

/*! The minimum relative distance between 2 Objects the Enhanced TTC is merged
 * with the regular TTC */
#define MOR_ETTC_RAMP_MIN_DIST (40.F)
#define MOR_ETTC_RAMP_MAX_DIST (100.F)
#define MOR_ETTC_RAMP_LENGTH (MOR_ETTC_RAMP_MAX_DIST - MOR_ETTC_RAMP_MIN_DIST)

/*! The minimum absolute curvature to calculate the radius of the curve */
#define MOR_MIN_ABS_CURVATURE (0.00001F)

/*! The minimum and maximum TTC/ETTC value to get a Cut-In Potential
 * (MO_MIN_TTC_VALUE: 100%, MO_MAX_TTC_VALUE: 0%) */
#define MOR_MIN_TTC_VALUE (1.F)
#define MOR_MAX_TTC_VALUE (10.F)

/*! The maximal lateral distance to Object ahead and the Start of the linear
 * Ramp to decrease the Potential (Object ahead seems not to be in the next
 * lane) */
#define MOR_LAT_DIST_OBJ_AHEAD_RAMP_MIN (4.F)
#define MOR_LAT_DIST_OBJ_AHEAD_RAMP_MAX (6.F)
#define MOR_LAT_DIST_OBJ_AHEAD_RAMP_LENGTH \
    (MOR_LAT_DIST_OBJ_AHEAD_RAMP_MAX - MOR_LAT_DIST_OBJ_AHEAD_RAMP_MIN)

/*! The minimum and maximum difference of the TTC to the Object and between the
Object and Object ahead
the potential is reduced linearly */
#define MOR_TTC_OBJ_RAMP_MIN (4.F)
#define MOR_TTC_OBJ_RAMP_MAX (6.F)
#define MOR_TTC_OBJ_RAMP_LENGTH (MOR_TTC_OBJ_RAMP_MAX - MOR_TTC_OBJ_RAMP_MIN)

#define MOR_TTC_DIFF_RAMP_MIN (3.F)
#define MOR_TTC_DIFF_RAMP_MAX (4.F)
#define MOR_TTC_DIFF_RAMP_LENGTH (MOR_TTC_DIFF_RAMP_MAX - MOR_TTC_DIFF_RAMP_MIN)

/*! The Value of cut_in_pot calculated by the MOR-Observer assumed as 100% and
 * the start of the Ramp to this value */
#define MOR_FULL_POTENTIAL_RAMP_MAX (0.7F)
#define MOR_FULL_POTENTIAL_RAMP_MIN (0.5F)
#define MOR_FULL_POTENTIAL_RAMP_LENGTH \
    (MOR_FULL_POTENTIAL_RAMP_MAX - MOR_FULL_POTENTIAL_RAMP_MIN)

/* ***********************************************************************/ /*!
  @brief            Preselection Parameters

  @description     The following section inherits all parameters necessary for
object preselection.
                   These parameters are used by module si_accquality.c

  @todo            check completness of parameters, all of them customer
adjustable?
                   building groups assigned to preselection functions
                   define new quality criteria based on Probability of existence
and X,Y Variances
                   translate german comments to english

**************************************************************************** */

/*--------------- Definitions for unlearned steering angle offset
 * restrictions-----------------------*/
/*! max. distance for stationary objects, if sensor is unlearned (steering angle
 * offset) */ /*|*/
#define SI_ABSTAND_MAX_UNLEARNED_SENSOR (70.0F) /*|*/
                                                /*|*/
/*! Inlane time for stationary objects is rised by below factor in umlearned
 * situation */ /*|*/
#define SI_UNLEARNED_SENSOR_INLANE_TIME_RISE_FACTOR (1.2F) /*|*/
                                                           /*|*/
/*! Below this distance objects are safe even if steering angle is unlearned */ /*|*/
#define SI_SAFE_DIST_UNLEARNED (30.0F) /*|*/
                                       /*|*/
/* if varianvce value of steering angle is bigger than this value, sensor is
 * unlearned !!! */ /*|*/
#define SI_VDY_YAWRATE_QUAL_SENSOR_UNLEARNED (0.55F) /*|*/
                                                     /*|*/
/*--------------- Definitions for unlearned steering angle offset
 * restrictions-----------------------*/

/*--------------- Definitions for maximum range
 * criteria---------------------------------------------*/
/*! Absolute minimale Sichtweite   */          /*|*/
#define DISTANCE_MIN (80.0F) /* Distanz min */ /*|*/
/* Absolute maximale Sichtweite */
#define DISTANCE_MAX (168.33F) /* Distanz max */ /*|*/

/*! Pickup distance parameter */
/*! Number of points which define the ramp */
#define SI_NUM_PICKUP_DIST_POINTS (2)

extern const volatile GDBVector2_t
    SI_a_SamplePointDistancePara[SI_NUM_PICKUP_DIST_POINTS];

/* Suppression of objects at high velocities/distances for german autobahn when
 * ego on left lane     */
/* Defines area and velocity delta in which possible overtaken object can be
 * located in relation     */
/* to the overtaking object */
#define SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAX_LOWER (-20.F)  /*|*/
#define SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAX_UPPER (40.F)   /*|*/
#define SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAY_LOWER (-6.F)   /*|*/
#define SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAY_UPPER (0.F)    /*|*/
#define SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAV_LOWER (-999.F) /*|*/
#define SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAV_UPPER (999.F)  /*|*/

/* Distance to road border dependant thresholds for object suppression at high
 * velocities            */
#define SI_HIGH_VEGO_OBJ_SUPPRESS_BORDER_DIST_R (-8.F) /*|*/
#define SI_HIGH_VEGO_OBJ_SUPPRESS_BORDER_DIST_L (3.3F) /*|*/

/* No object suppression for high velocities if object is braking intensly */
#define SI_HIGH_DIST_OBJ_SUPPRESS_AREL_MIN (-3.F) /*|*/

/* Length of array of return objects of function SIFindObjInArea */
/* e.g. maximal number of objects to consider as possibly overtaken or passed by
 * reference object    */
#define SI_FIND_MOV_OBJ_IN_AREA_NUM_MAX (5u) /*|*/

/* Velocity and distance thresholds for truck suppression at high distances */
#define SI_HIGH_DIST_TRUCK_SUPPRESS_VEGO_MIN (140.F / C_KMH_MS) /*|*/
#define SI_HIGH_DIST_TRUCK_SUPPRESS_XDIST_MIN (80.F)            /*|*/
#define SI_HIGH_DIST_TRUCK_SUPPRESS_VREL_MAX (-65.F / C_KMH_MS) /*|*/
#define SI_HIGH_DIST_TRUCK_SUPPRESS_VOBJ_MIN (60.F / C_KMH_MS)  /*|*/

/* Allow high VW pickup range only if road-estimation is available for ACC
 * trajectory fusion */
/* and has a sufficient range, or if we approach an object with a high relative
 * velocity */
#define SI_VDY_ONLY_VREL_REDUC_MIN_VREL (-15.0f / C_KMH_MS) /*|*/
#define SI_VDY_ONLY_VREL_REDUC_RE_RANGE_MIN (150.0f)        /*|*/

/*|*/
/* curve dependent RangeReduction to avoid false pickups in curveexit
 * situations*/ /*|*/
/* and alternating curves*/ /*|*/
typedef struct SIRangeReductionParams {
    float32 RangeReduction_Vego_Delta;   /*!< maximum offset subtracted from ego
                                            speed to reduce range. @min:1e-8 @max:
                                            10 */
    float32 RangeReduction_Curve_Thresh; /*!< Curvature threshod for reducing
                                            range   @min:1e-8 @max: 1 */
    float32
        RangeFactor_On_Timeconstant; /*!< Filter constant for range reducing
                                        factor in range reduction active case.
                                        @min:1e-8 @max: 1 */
    float32 RangeFactor_Off_Timeconstant; /*!< Filter constant for range
                                             reducing factor in range reduction
                                             inactive case.@min:1e-8 @max: 1 */
} SIRangeReductionParams_t;               /*!< @allow: all_cust */

/* curve dependent RangeReduction to avoid false pickups in curveexit
 * situations*/ /*|*/
/* and alternating curves*/ /*|*/
#define RANGEREDUCTION_VEGO_DELTA                                            \
    SIRangeReductionParams.RangeReduction_Vego_Delta /* typical value : 7.0f \
                                                      */
#define RANGEREDUCTION_CURVE_THRESH \
    SIRangeReductionParams          \
        .RangeReduction_Curve_Thresh /* typical value: 1.0f/2000.0f */
#define RANGEFACTOR_ON_TIMEKONSTANT \
    SIRangeReductionParams          \
        .RangeFactor_On_Timeconstant /* typical value: 100.0f */
#define RANGEFACTOR_OFF_TIMEKONSTANT \
    SIRangeReductionParams           \
        .RangeFactor_Off_Timeconstant /* typical value: 5.0f */

/*! Maximum distance for stationary object */ /*|*/
#define SI_ABSTAND_MAX_NORELOBJ (100.0F)      /*|*/
/*--------------- Definitions for maximum range
 * criteria---------------------------------------------*/

#define SI_CUTIN_KRITERIA SWITCH_ON
/*-------------------------------------------------------------------------------------------------------------*/

/* ARS202 */                          /*|*/
/* #define SCHZI_NEBENKEULE_ACTIVE */ /*|*/
/*-------------------------------------------------------------------------------------------------------------*/

/*-- Switch: prevent that objects in distances greater than
 * OBJ_REGELZEITABSTAND_MAX (4.1s) become relevant ---*/
#define REGELZEITABSTAND_ACTIVE /*|*/
/*-------------------------------------------------------------------------------------------------------------*/

#define SI_MIN_ROAD_BORDER_STAT                                             \
    (100u) /*!< Minimum road border status for considering the road boarder \
              during trace bracket calculation */

/* -- Konstanten fuer Freischalten des Nahbereichs bei bis in den --*/ /*|*/
/* -- Stoerzeiger-Nahbereich eintauchenden Objekten               --*/ /*|*/
/* Objekte > STABI_STOER_NAHBEREICH_ABST_MAX immer verwenden */        /*|*/
#define STABI_STOER_NAHBEREICH_ABST_MAX (7.F)                          /*|*/
/* Abstand_Neu fuer Objekte die bis in den Stoerzeigerbereich eintauchen */ /*|*/
#define STABI_STOER_ABST_EINTAUCHEN (10.F) /*|*/
                                           /*|*/
/* Vrel fuer Objekte, die im Nahbereich entstehen und deutliche Vrel aufweisen
 */ /*|*/
#define STABI_STOER_VREL (2.F / C_KMH_MS)   /*|*/
                                            /*|*/
/* Zeit fuer Freischaltung */               /*|*/
#define STABI_STOERBEREICH_FREI_ZEIT (10.F) /*|*/
/* Max-Wert fuer NahbereichFrei-Zaehler */  /*|*/
#define STABI_STOERBEREICH_FREI_ZAEHLER \
    (sint32) ROUND(STABI_STOERBEREICH_FREI_ZEIT / TPGetCycleTime()) /*|*/
                                                                    /*|*/
/*-- Regelzeitabstand --*/                                          /*|*/
/* Maximaler Regelzeitabstand */                                    /*|*/
#define OBJ_REGELZEITABSTAND_MAX (4.1F)                             /*|*/
/*! Regelzeitabstand f?uer Gegenverkehr */
#define REGELZEITABSTAND_ONCOMING (2.0F) /*|*/
/* max distance for select oncoming objects */
#define MAXDISTONCOMING (180.0F) /*|*/
/* max. lateral acceleration for range reduction of oncoming traffic in curve
 * situation */
#define AYMAX (4.0F) /*|*/
/* Mindesabstand fuer Regelzeitabstand d.h. zwischen 80 und 150m wird
 * ausgeblendet */
#define STABI_ABSTAND_REGEL_MIN_STATIONARY (80.F) /*|*/
#define STABI_ABSTAND_REGEL_MIN (80.F)            /*|*/
#define SI_MAX_DIST_HYSTERESIS_MOV (20.F)         /*|*/
#define SI_MAX_DIST_HYSTERESIS (20.F)             /*|*/
#define SI_MAX_DIST_HYSTERESIS_MIN (3.F)          /*|*/
                                                  /*|*/
                                                  /*|*/
                                                  /*|*/

/*! Maximum X distance for an oncoming object to become relevant */
#define SI_PAR_ONCOMMING_KEEP_RELEV_DISTX_MAX (25.0f)

/*! Minimum speed-sum (obj_v_rel + v_ego) for an oncoming object to become
 * relevant */
#define SI_PAR_SPEED_SUM_ROLL_BACK_MIN (-15.F / C_KMH_MS)

/*! maximum registered accumulated driven distance where an object was relevant
 */
#define SI_INLANE_DIST_MAXVALUE (200.0f)

// #define ABST_GRENZ_NAHBEREICH          (11.0F)
/* min. Objektlebensdauer f?uer Absenkung der Zielschwelle */
/* auf -ZIEL_SENK_MAX */
// #define OBJLEB_ZIEL_SENK     (CONV_CYCLES_TO_LIFETIME(20))  /* OLD */

/* Maximaler Abstand f?uer sichere Spurtrennung d.h. bis dorthin */
/* Spurverbreiterung m�glich */
#define SI_MAX_VREL_REL_LANE_EXT_FACT                                       \
    (-2.5f / C_KMH_MS) /*!< Maximal relative velocity for applying relative \
                          lane extension factor */
#define SPUR_SICHERHEIT (100.F) /* OLD */
/* Minimale Zeit fuer SpurZeitErweiterungsFaktor */
#define SPURZEIT_MIN (0.5F) /* OLD */
/* Maximale Zeit fuer SpurZeitErweiterungsFaktor */
#define SPURZEIT_MAX (2.0F) /* OLD */
/* Maximale Abstandsdifferenz fuer Spurverbreiterung */
#define SPUR_ABSTANDDIFF_MAX (30.F) /* OLD */
/* Minimale Abstandsdifferenz fuer Spurverbreiterung */
#define SPUR_ABSTANDDIFF_MIN (10.F) /* OLD */

/* Zeit in s um SpurZeitErweiterungsFaktor bei Spurwechsel des */
/* relevantes Obj abzubauen */
#define SPURZEIT_DEC (0.66F) /* OLD */

/* Max. Zeit fuer Rel.Obj in s */
#define RELEVANTZEIT_MAX (20.F) /* OLD */

/* threshold for maximum stability criteria */
#define STAB_ZAHL_MAX (4L)

typedef struct SILowSpeedStatPedesParams {
    float32 Max_Speed; /*!< max. speed [km/h] to select stationary pedestrians
                          as OOI.       @min:1e-8 @max: 10 */
    float32 Max_Dist; /*!< max. distance [m] to select stationary pedestrians as
                         OOI.       @min:1e-8 @max: 10 */
    float32 Min_LTime;         /*!< min Lifetime [cycles] to select stationary
                                  pedestrians as OOI.   @min:1e-8 @max: 10 */
    float32 MinTime_In_Lane;   /*!< Time [s] in Lane to select stationary
                                  pedestrians as OOI.        @min:1e-8 @max: 1 */
    float32 RCS_Add;           /*!< RCS Offset [dB] to be added to the detection
                                  threshold.          @min:1e-8 @max: 1 */
} SILowSpeedStatPedesParams_t; /*!< @allow: all_cust */

/*-------------------------------------------------------------------------------------------------------------*/

/* ! Constant for object width */
#define SI_MIN_OBJ_WIDTH_CAR_STAT (1.8f)
#define SI_MAX_OBJ_WIDTH_CAR_STAT (3.0f)
#define SI_OBJ_WIDTH_CAR_MOVE (1.8f)
#define SI_OBJ_WIDTH_TRUCK (2.5f)
#define SI_MIN_OBJ_WIDTH_OTHER_CLASS_STAT SI_MIN_OBJ_WIDTH_CAR_STAT
#define SI_MAX_OBJ_WIDTH_OTHER_CLASS_STAT SI_MAX_OBJ_WIDTH_CAR_STAT
#define SI_OBJ_WIDTH_SMALL_OBJ (0.8f)
#define SI_OBJ_WIDTH_MOVE (2.f)
/* minimaler Abstand ab dem alle Objekte Minimumbreite annehmen */
#define SI_AVLC_MIN_DIST_OBJ_WIDTH_DIST_ADAPTED (110.f)
/* minimale Breite zur Overlapberechnung fuer Objekte in weitem Abstand */
#define SI_AVLC_MIN_OBJ_WIDTH_FAR_DIST (1.8f)
#define SI_MIN_OBJ_WIDTH_STAT (0.8f)
#define SI_MAX_OBJ_WIDTH_STAT (2.f)

#define SI_OBJ_FAR_RANGE (90.0F)
#define SI_OBJ_NEAR_RANGE (30.0F)

#define SI_OBJ_WIDTH_VAR_MOVE_FAR (1.0f)
#define SI_OBJ_WIDTH_VAR_MOVE_NEAR (0.33f)

#define SI_OBJ_WIDTH_VAR_FACTOR (0.25f)

#define SI_AVLC_TRAJECTORY_WIDTH_STAT (2.5f)
#define SI_AVLC_TRAJECTORY_NARROWWIDTH_STAT (2.0f)

#define SI_PAR_MAX_DISTX_STAT_POINT_SMALL_SEEK (50.f)

#define SI_BLOCKED_PATH_CALCULATION_ENABLE SWITCH_ON
#define SI_MIN_PATHWIDTH_PASSABLE_STAT (1.75f)
#define SI_MIN_PATHWIDTH_PASSABLE_STAT_HYST (2.1f)
#define SI_MIN_PATHWIDTH_PASSABLE_MOV_HYST_ADD (0.5f)
#define SI_BLOCKED_PATH_DECISION_TIME 5u
#define SI_HALF_LANEOCCUPATION_WIDTH (5.0f) /*! m*/
#define SI_CORR_SEL_MOVING_AS_STAT_SPEED (15.0f / C_KMH_MS)

#define SI_MIN_PATHWIDTH_PASSABLE_MOV_MIN (2.0f)
#define SI_MIN_PATHWIDTH_PASSABLE_MOV_MAX (2.5f)
#define SI_MIN_PATHWIDTH_PASSABLE_MOV_VOBJ_MIN (5.0f)
#define SI_MIN_PATHWIDTH_PASSABLE_MOV_VOBJ_MAX (15.0f)

#define SI_CORRIDOR_SEL_Y_OFFSET_MIN (0.0f)
#define SI_CORRIDOR_SEL_Y_OFFSET_MAX (0.5f)
#define SI_CORRIDOR_SEL_Y_OFFSET_DIST_MIN (20.0f)
#define SI_CORRIDOR_SEL_Y_OFFSET_DIST_MAX (40.0f)

#define SI_CORSELYOFF_Obj2ObjDistX_MIN (0.0f)
#define SI_CORSELYOFF_Obj2ObjDistX_MAX (5.0f)

#define SI_CORSELYOFF_TTCGAP_MIN (1.7f)
#define SI_CORSELYOFF_TTCGAP_MAX (3.3f)

#define SI_CORSEL_Y_HYST_MIN (0.10f)
#define SI_CORSEL_Y_HYST_MAX (1.30f)
#define SI_CORSEL_Y_HYST_TIMEGAP_MIN (1.0f)
#define SI_CORSEL_Y_HYST_TIMEGAP_MAX (2.8f)

/* maximum curvature for active blocked path selection */
#define SI_MAX_CURVE_FOR_BLOCKED_PATH_SELECTION (1.0f / 80.0f)

/*------------------ Parameters for reduced predicted drop border for large
 * angles --------------------------*/

/*! Minimal decrease of the drop border based on object angle */
#define SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MOD_MIN (0.0f)
/*! Maximal decrease of the drop border based on object angle */
#define SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MOD_MAX (0.0f)
/*! The angle corresponding to minimal decrease of drop border */
#define SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MIN (22.0f)
/*! The angle corresponding to maximal decrease of drop border */
#define SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MAX (30.0f)

/*-------------------------- Switches for pickup prediction time limitation
 * ---------------------------------*/

/*! Configuration switch to limit cut-in prediction time based no
 * distance/relative-speed */
#define SI_CFG_USE_RELATIVE_SPEED_DIST_TO_LIMIT_PRED_TIME SWITCH_OFF

/*! Configuration switch to limit cut-in prediction time based on
 * distance/ego-speed */
#define SI_CFG_USE_EGO_SPEED_DIST_TO_LIMIT_PRED_TIME SWITCH_OFF

/*--------------------------- End of customisation part
 * -----------------------------------------------------*/

/* switch for enabling ROV reset condition by time */
#define SI_ROV_RESET_BY_TIME SWITCH_OFF

/******************************************************************************
  si_laneassociation.c parameters
******************************************************************************/

typedef struct SIOccupancyParams {
    float32 ObjOccPickupThreshOutlane; /*!< Pickup threshold object occupancy
                                          when outlane @min:1e-8 @max: 1 */
    float32 ObjOccPickupThreshInlane;  /*!< Pickup threshold object occupancy
                                          when inlane  @min:1e-8 @max: 1 */
    float32 ObjOccDropTresh; /*!< Drop threshold object occupancy   @min:1e-8
                                @max: 1 */
    float32 LaneOccPickupThresh; /*!< Pickup threshold lane occupancy
                                    @min:1e-8 @max: 1 */
    float32 LaneOccDropTresh; /*!< Drop threshold lane occupancy     @min:1e-8
                                 @max: 1 */
} SIOccupancyParams_t;        /*!< @allow: all_cust */

/* Pickup and drop thresholds for lane selection of moving objects */
#define ObjectOccupancyPickupThreshOutlane \
    SIOccupancyParams.ObjOccPickupThreshOutlane /*0.55f*/
#define ObjectOccupancyPickupThreshInlane \
    SIOccupancyParams.ObjOccPickupThreshInlane                      /*0.50f*/
#define ObjectOccupancyDropThresh SIOccupancyParams.ObjOccDropTresh /*0.30f*/
#define LaneOccupancyPickupThresh \
    SIOccupancyParams.LaneOccPickupThresh                          /*0.50f*/
#define LaneOccupancyDropThresh SIOccupancyParams.LaneOccDropTresh /*0.30f*/

#define SI_OBJ_OVLC_PICKUP_THRESH_OUTLANE_DISTX_MIN                     \
    (40.f) /* Minimum longitudinal distance for higher object occupancy \
              out-lane pickup */

/*Max. Overlap variance for normal object occupancy drop border*/
#define ObjectOccupancyDropMaxOverlapVariance (0.35f)
/*object occupancy drop border for insecure overlap values     */
#define ObjectOccupancyDropThreshInsecureOverlap (0.30f)

/* Pickup and drop thresholds for lane selection of stationary and oncoming
 * objects */
#define ObjectOccupancyPickupThreshStat (0.60f)
#define ObjectOccupancyDropThreshStat (0.45f)
#define LaneOccupancyPickupThreshStat                                    \
    (0.30f) /* Lane is only 2.0m for the "point" object and 2.5m for the \
               other Objects */
#define LaneOccupancyDropThreshStat                                      \
    (0.20f) /* Lane is only 2.0m for the "point" object and 2.5m for the \
               other Objects */

/* Pickup and drop thresholds for lane selection of stationary and oncoming
 * objects in Road Works or unlearned sensor*/
#define ObjectOccupancyPickupThreshStatConservative (0.80f)
#define ObjectOccupancyDropThreshStatConservative (0.45f)
#define LaneOccupancyPickupThreshStatConservative                        \
    (0.50f) /* Lane is only 2.0m for the "point" object and 2.5m for the \
               other Objects */
#define LaneOccupancyDropThreshStatConservative                          \
    (0.45f) /* Lane is only 2.0m for the "point" object and 2.5m for the \
               other Objects */

/* Pickup and drop thresholds for predicted lane selection of moving objects */
#define SI_OBJ_OVLC_PRED_PICKUP_THRESH \
    (0.10f) /* Formerly ObjectOccupancyPredPickupThresh */
#define SI_TRUCK_OBJ_OVLC_PRED_PICKUP_THRESH \
    (0.45f) /* Formerly ObjectOccupancyPredPickupThreshTruck */
#define SI_MAX_OBJ_OVLC_PRED_PICKUP_THRESH \
    (0.65f) /* Formerly ObjectOccupancyPredPickupThreshMax */
#define SI_LOW_NEAR_DIST_OBJ_OVLC_PRED_DROP_THRESH \
    (0.25f) /* Formerly OOPredDropThreshLowNearDist */

/* time (s) for stationary objects for being accepted in lane */
#define SI_STATOBJ_INLANE_HIGHSPEED (100.0f / C_KMH_MS) /* [m/s] */
#define SI_STATOBJ_INLANE_LOWSPEED (25.0f / C_KMH_MS)   /* [m/s] */
#define SI_STATOBJ_INLANE_PARKSPEED (5.0f / C_KMH_MS)   /* [m/s] */
#define SI_STATOBJ_INLANE_NOSPEED (C_F32_DELTA)         /* [m/s] */

#define SI_STATOBJ_INLANE_DISTMAX (20.0f) /* [m] */
#define SI_STATOBJ_INLANE_DISTMIN (5.0f)  /* [m] */
#define SI_STATOBJ_INLANE_DISTSLOPE                            \
    ((SI_STATOBJ_INLANE_DISTMAX - SI_STATOBJ_INLANE_DISTMIN) / \
     (SI_STATOBJ_INLANE_HIGHSPEED - SI_STATOBJ_INLANE_LOWSPEED)) /* [s] */
#define SI_STATOBJ_INLANE_DISTOFFSET \
    (SI_STATOBJ_INLANE_DISTMAX -     \
     (SI_STATOBJ_INLANE_DISTSLOPE * SI_STATOBJ_INLANE_HIGHSPEED)) /* [m] */

#define SI_STATOBJ_INLANE_TIME_PARKSPEED \
    (SI_STATOBJ_INLANE_DISTMIN / SI_STATOBJ_INLANE_PARKSPEED) /* [s] */
#define SI_STATOBJ_INLANE_TIME_NOSPEED (0.72f)                /* [s] */

/* time (s) for stationary vehicles(Truck, Car, Motorcycle) for being accepted
 * in lane */
#define SI_STATVEH_INLANE_HIGHSPEED (100.0f / C_KMH_MS) /* [m/s] */
#define SI_STATVEH_INLANE_LOWSPEED (25.0f / C_KMH_MS)   /* [m/s] */
#define SI_STATVEH_INLANE_PARKSPEED (5.0f / C_KMH_MS)   /* [m/s] */
#define SI_STATVEH_INLANE_NOSPEED (C_F32_DELTA)         /* [m/s] */

#define SI_STATVEH_INLANE_DISTMAX (20.0f) /* [m] */
#define SI_STATVEH_INLANE_DISTMIN (5.0f)  /* [m] */
#define SI_STATVEH_INLANE_DISTSLOPE                            \
    ((SI_STATVEH_INLANE_DISTMAX - SI_STATVEH_INLANE_DISTMIN) / \
     (SI_STATVEH_INLANE_HIGHSPEED - SI_STATVEH_INLANE_LOWSPEED)) /* [s] */
#define SI_STATVEH_INLANE_DISTOFFSET \
    (SI_STATVEH_INLANE_DISTMAX -     \
     (SI_STATVEH_INLANE_DISTSLOPE * SI_STATVEH_INLANE_HIGHSPEED)) /* [m] */

#define SI_STATVEH_INLANE_TIME_PARKSPEED \
    (SI_STATVEH_INLANE_DISTMIN / SI_STATVEH_INLANE_PARKSPEED) /* [s] */
#define SI_STATVEH_INLANE_TIME_NOSPEED (0.72f)                /* [s] */

/* time (s) for moving objects for being accepted in lane */
/* time is a fuction of distance */
#define SI_MINTIME_MOVE_OBJ_IN_LANE_MIN (0.0f)
#define SI_MINTIME_MOVE_OBJ_IN_LANE_MAX (0.3f)
#define SI_DISTX_MOVE_OBJ_IN_LANE_MIN (60.0f)
#define SI_DISTX_MOVE_OBJ_IN_LANE_MAX (120.0f)

/* Parameters for situative increase of inlane timer as a function */
/* of relative velocity, curvature, and distance */
#define SI_SIT_VREL_INLANE_TIME_INC_MIN_VREL (-10.0f)
#define SI_SIT_VREL_INLANE_TIME_INC_MAX_VREL (-0.5f)
#define SI_SIT_VREL_INLANE_TIME_INC_MIN_EGO_VEL (70.0f / C_KMH_MS)
#define SI_SIT_VREL_INLANE_TIME_INC_MIN_EGO_DISTX (40.0f)
#define SI_SIT_VREL_INLANE_TIME_INC_MAX_CURVE (0.001f)

#define SI_SIT_VREL_INLANE_TIME_INC_MAX_TIME_TRUCKS (1.0f)
#define SI_SIT_VREL_INLANE_TIME_INC_MAX_TIME_STRAIGHT_CAR (0.5f)

#define SI_SIT_VREL_INLANE_TIME_INC_MAX_INCR (0.5f)
#define SI_SIT_VREL_INLANE_TIME_INC_MED_INCR (0.4f)
#define SI_SIT_VREL_INLANE_TIME_INC_MIN_INCR (0.2f)

/* Velocity and distance thresholds for object suppression at high velocities */
#define SI_HIGH_VEGO_INLANE_TIME_VEGO_MIN (120.F / C_KMH_MS) /*|*/
#define SI_HIGH_VEGO_INLANE_TIME_VOBJ_MIN (70.F / C_KMH_MS)  /*|*/
#define SI_HIGH_VEGO_INLANE_TIME_VREL_MAX (-50.F / C_KMH_MS) /*|*/
#define SI_HIGH_VEGO_INLANE_TIME_XDIST_MIN (90.F)            /*|*/
#define SI_HIGH_VEGO_INLANE_TIME_STRONG_CURVE (1.F / 1600.F) /*|*/
/* Increased in-lane time threshold values */
#define SI_HIGH_VEGO_INLANE_TIME_TRUCK (0.8F)    /*|*/
#define SI_HIGH_VEGO_INLANE_TIME_OCCLUDED (0.7F) /*|*/
#define SI_HIGH_VEGO_INLANE_TIME_CAR (0.7F)      /*|*/

/*! The in-lane threshold time factor when roadworks is detected */
#define SI_PAR_ROADWORKS_INLANE_TIME_FACTOR (3.f)

#define SI_SCALE_LANE_WIDTH_LANE_ASSOC \
    (1.5f) /*!< Scale lane width by this factor for lane association */
#define SI_SCALE_RESET_INLANE_TIMER_STAT                                 \
    (2.f) /*!< Scale cycle time by this factor to reset inlane timer for \
             stationary objects */
#define SI_MAX_REL_TIMER_DROP_OVLC_THRES                               \
    (1.f) /*!< Maximum relevant time for spetial handling of occupancy \
             theshold (not statinonary objects) */
#define SI_SUM_NONSTAT_DROP_OVLC_THRES                                  \
    (0.1f) /*!< Summand for spetial handling of occupancy theshold (not \
              statinonary objects) */
#define SI_MAX_CORR_REL_TIME_NONSTAT \
    (1.f) /*!< Maximum value of fCorridorRelevantTime for moving objects */

/******************************************************************************
  si_laneassociation.c parameters end
******************************************************************************/

/*! Parameter of how many poles to use to filter VDY course for stationary
objects.
Set to zero or 1 to disable filtering completely */
#define SI_PAR_STAT_COURSE_FIR_POLES 8u

/* Parameters for restriction in the near range for country road /city scenarios
to improve the release
of objects that take a turn */
#define SI_REST_CITY_NEAR_RANGE_MIN_RESTICT \
    (0.35f * CITYLANEWIDTHTRCK) /*!< Minimal restriction */
#define SI_REST_CITY_NEAR_RANGE_MAX_RESTICT \
    (0.5f * CITYLANEWIDTHTRCK) /*!< Maximal restriction */
#define SI_REST_CITY_NEAR_RANGE_MIN_DIST_X \
    (10.f) /*!< Longitudinal distance for minimal restriction */
#define SI_REST_CITY_NEAR_RANGE_MAX_DIST_X \
    (12.f) /*!< Longitudinal distance for maximal restriction */

#define SI_REST_CITY_NEAR_RANGE_MAX_VREL \
    (-1.f) /*!< Maximal relative velocity */
#define SI_REST_CITY_NEAR_RANGE_MAX_VELOCITY \
    (70.f / C_KMH_MS) /*!< Maximal ego velocity */
#define SI_REST_CITY_NEAR_RANGE_MAX_CURVE \
    (0.004f) /*!< Maximal ego curve; radius = 250m */

/*************************  Do not change definitions below !!!!!!!!
 * *****************************************/
/*                          Contact Base development of ADC !!!!!!!! */
/*************************************************************************************************************/

/* Definitions for ZAWSIM*/

#define STABI_CODE_WINKELGRADIENT (0x00000001)
#define STABI_CODE_ABLAGEGRADIENT (0x00000002)
#define STABI_CODE_ZIELSICHERHEIT (0x00000004)
#define STABI_CODE_OBJEKTSICHERHEIT (0x00000008)
#define STABI_CODE_LKWTYP_SCHATTEN (0x00000010)
#define STABI_CODE_ABSTAND (0x00000020)
#define STABI_CODE_NAHBEREICH (0x00000040)
#define STABI_CODE_NEBENMAX (0x00000080)
#define STABI_CODE_WINKELGUETE (0x00000100)
#define STABI_CODE_REGELZEITABSTAND (0x00000200)
#define STABI_CODE_EMVDETEKTION (0x00000400)
#define STABI_CODE_SPRUEHFAHNEN (0x00000800)
#define STABI_CODE_KLEINER_RCS (0x00001000)
#define STABI_CODE_KLEINER_RCS_ABSOLUT (0x00002000)

extern const volatile SIOccupancyParams_t SIOccupancyParams;
extern const volatile SIRangeReductionParams_t SIRangeReductionParams;
extern const volatile SILowSpeedStatPedesParams_t SILowSpeedStatPedesParams;

/***************************************************/
/*! Calculation of lane change probability */

#define SI_LC_PROB_POINTS_TURN_LIGHT_TABLE \
    (3) /*!< No. of elements in SI_LC_PROB_TABLE_TURN_LIGHT */
#define SI_LC_PROB_POINTS_LAT_VEL_CAM_LANE_MARKER_TABLE \
    (2) /*!< No. of elements in SI_LC_PROB_TABLE_LAT_VEL_CAM_LANE_MARKER */
#define SI_LC_PROB_POINTS_LAT_DIST_TABLE \
    (4) /*!< No. of elements in SI_LC_PROB_TABLE_LAT_DIST */
#define SI_LC_PROB_POINTS_CURVE_TABLE \
    (4) /*!< No. of elements in SI_LC_PROB_TABLE_CURVE */
#define SI_LC_PROB_POINTS_DIFF_LAT_FILT_CURVES_TABLE \
    (4) /*!< No. of elements in SI_LC_PROB_TABLE_DIFF_LAT_FILT_CURVES */
#define SI_LC_PROB_POINTS_DRIVER_INT_CURVE_FILTER_TABLE \
    (2) /*!< No. of elements in                         \
           SI_LC_PROB_TABLE_DRIVER_INT_CURVE_FILTER_CONST */
#define SI_LC_PROB_POINTS_COMB_CAM_PROB_TABLE \
    (4u) /*!< No. of elements in SI_LC_PROB_TABLE_COMB_CAM_PROB */
#define SI_LC_PROB_POINTS_COMB_ALL_LATDIFFCURVE_TURNLIGHT_TABLE \
    (8u) /*!< No. of elements in                                \
            SI_LC_PROB_TABLE_COMB_ALL_LATDIFFCURVE_TURNLIGHT */
#define SI_LC_PROB_POINTS_COMB_LATDIST_TURNLIGHT_LATDIFFCURVE_TABLE \
    (8u) /*!< No. of elements in                                    \
            SI_LC_PROB_TABLE_COMB_LATDIST_TURNLIGHT_LATDIFFCURVE */
#define SI_LC_PROB_POINTS_COMB_LOWSPEED_TURNLIGHT_TABLE \
    (4u) /*!< No. of elements in SI_LC_PROB_TABLE_COMB_LOWSPEED_TURNLIGHT */
#define SI_LC_PROB_POINTS_COMB_LATDIST_LATDIFFCURVE_TABLE \
    (8u) /*!< No. of elements in SI_LC_PROB_TABLE_COMB_LATDIST_LATDIFFCURVE */
#define SI_LC_PROB_POINTS_COMB_MARKERCROSSING_TABLE \
    (4u) /*!< No. of elements in SI_LC_PROB_TABLE_COMB_MARKERCROSSING */
#define SI_LC_PROB_POINTS_COMB_CURVE_TABLE \
    (4u) /*!< No. of elements in SI_LC_PROB_TABLE_COMB_CURVE */
#define SI_LC_PROB_POINTS_COMB_LATDIFFCURVE_TURNLIGHT_TABLE                   \
    (4u) /*!< No. of elements in SI_LC_PROB_TABLE_COMB_LATDIFFCURVE_TURNLIGHT \
          */

typedef struct {
    /*! Convert time since last usage of turn light switch into probability that
     * the driver changes the lane in this direction */
    GDBVector2_t
        SI_LC_PROB_TABLE_TURN_LIGHT[SI_LC_PROB_POINTS_TURN_LIGHT_TABLE];
    /*! Convert lateral velocity to lane markers into probability that the
     * driver changes the lane in this direction */
    GDBVector2_t SI_LC_PROB_TABLE_LAT_VEL_CAM_LANE_MARKER
        [SI_LC_PROB_POINTS_LAT_VEL_CAM_LANE_MARKER_TABLE];
    /*! Combination of camera based lane change probabilities to a single
     * probability */
    uint8 SI_LC_PROB_TABLE_COMB_CAM_PROB[SI_LC_PROB_POINTS_COMB_CAM_PROB_TABLE];
    /*! Combination of already accumulated LC probability (camera) with
     * iLatDiffFilteredCurvesProb and turn light based probability */
    uint8 SI_LC_PROB_TABLE_COMB_ALL_LATDIFFCURVE_TURNLIGHT
        [SI_LC_PROB_POINTS_COMB_ALL_LATDIFFCURVE_TURNLIGHT_TABLE];
    /*! Combination of different LC probabilities (difference is lateral
     * displacement based on filtered curves, turn light) to a single
     * probability */
    uint8 SI_LC_PROB_TABLE_COMB_LATDIFFCURVE_TURNLIGHT
        [SI_LC_PROB_POINTS_COMB_LATDIFFCURVE_TURNLIGHT_TABLE];
    /*! Convert difference in lateral distance between curve (ego or camera) and
     * filtered driver intended curve to probability */
    GDBVector2_t SI_LC_PROB_TABLE_DIFF_LAT_FILT_CURVES
        [SI_LC_PROB_POINTS_DIFF_LAT_FILT_CURVES_TABLE];
    /*! Filter constant for driver intended curve */
    GDBVector2_t SI_LC_PROB_TABLE_DRIVER_INT_CURVE_FILTER_CONST
        [SI_LC_PROB_POINTS_DRIVER_INT_CURVE_FILTER_TABLE];
    /*! Combination different LC probabilities (lateral displacement, difference
     * is lateral displacement based on filtered curves, turn light) to one
     * combined probability (low speed) */
    uint8 SI_LC_PROB_TABLE_COMB_LATDIST_TURNLIGHT_LATDIFFCURVE
        [SI_LC_PROB_POINTS_COMB_LATDIST_TURNLIGHT_LATDIFFCURVE_TABLE];
    /*! Combination already accumulated LC probability (camera) with the turn
     * light based probability (low speed) */
    uint8 SI_LC_PROB_TABLE_COMB_LOWSPEED_TURNLIGHT
        [SI_LC_PROB_POINTS_COMB_LOWSPEED_TURNLIGHT_TABLE];
    /*! Combination already accumulated LC probability (camera) with
     * iLatDiffFilteredCurvesProb and distance in lateral direction (low speed)
     */
    uint8 SI_LC_PROB_TABLE_COMB_LATDIST_LATDIFFCURVE
        [SI_LC_PROB_POINTS_COMB_LATDIST_LATDIFFCURVE_TABLE];
    /*! Combination already accumulated LC probability (camera) with detected
     * crossing of lane markers */
    uint8 SI_LC_PROB_TABLE_COMB_MARKERCROSSING
        [SI_LC_PROB_POINTS_COMB_MARKERCROSSING_TABLE];
    /*! Convert driven distance in lateral direction into LC probability */
    GDBVector2_t SI_LC_PROB_TABLE_LAT_DIST[SI_LC_PROB_POINTS_LAT_DIST_TABLE];
    /*! Convert curve (driver intended curve) into LC probability */
    GDBVector2_t SI_LC_PROB_TABLE_CURVE[SI_LC_PROB_POINTS_CURVE_TABLE];
    /*! Combination already accumulated LC probability with curvature based
     * probability (low speed) */
    uint8 SI_LC_PROB_TABLE_COMB_CURVE[SI_LC_PROB_POINTS_COMB_CURVE_TABLE];
} SILaneChangeProbParameter_t;

extern const volatile SILaneChangeProbParameter_t SILCProbParData;

/*!< Calculation of lane change probability */
/***************************************************/

#endif
