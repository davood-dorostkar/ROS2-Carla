
#ifndef CP_PAR_H_INCLUDED
#define CP_PAR_H_INCLUDED

/*! @brief CURVATURE_USE_CIRCLE_EQUATION */
#define CURVATURE_USE_CIRCLE_EQUATION (1.F / 1000.F) /* in 1/m */

/*! @brief FUSIONTRACESPEEDTHRESH */
#define FUSIONTRACESPEEDTHRESH (float32)(40.F / C_KMH_MS)

/*! @brief FUSIONTRACESPEEDHYSTOFFSET */
#define FUSIONTRACESPEEDHYSTOFFSET (float32)(10.F / C_KMH_MS)
/*! @brief FUSIONSPEEDHYSTOFFSET */
#define FUSIONSPEEDHYSTOFFSET (float32)(10.F / C_KMH_MS)
/*! @brief MINFUSIONSPEED */
#define MINFUSIONSPEED (float32)(125.F / C_KMH_MS) /*@todo not used delete*/
/*! @brief MINFUSIONCURVE */
#define MINFUSIONCURVE (1.F / 500.F)
/*! @brief MAXFUSIONCURVE */
#define MAXFUSIONCURVE (1.F / 300.F)

/*! @brief (uint8)(78) */
#define MINFUSIONEOROADCONFIDENCE (78.0f)
/*! @brief (uint8)(84) */
#define MAXFUSIONEOROADCONFIDENCE (84.0f)
/*! @brief MINREFUSIONEGOSPEEDDIST */
#define MINREFUSIONEGOSPEEDDIST (80.0F) /* distance min */
/*! @brief MAXREFUSIONEGOSPEEDDIST */
#define MAXREFUSIONEGOSPEEDDIST (165.0F) /* distace max */
/*! @brief MAXREFUSIONEGOSPEED */
#define MAXREFUSIONEGOSPEED (200.0F / C_KMH_MS)
/*! @brief MINREFUSIONEGOSPEED */
#define MINREFUSIONEGOSPEED (80.0F / C_KMH_MS)
/*! @brief SAREFUSIONDISTHYST */
#define SAREFUSIONDISTHYST (10.0F) /*Hysteresis for State*/
/*! @brief CURVEDIRECTIONTHRESH */
#define CURVEDIRECTIONTHRESH (1.80f) /*m*/
/*! @brief PARALLELISM_THRESH */
#define PARALLELISM_THRESH \
    1.0F /*Do not set to zero!!! causes division by zero*/
/*! @brief PARALLELISM_MIN_LENGTH_TRACE */
#define PARALLELISM_MIN_LENGTH_TRACE ((sint8)4)
/*! @brief PARALLELISM_MIN_RE_TRACKING_STAT */
#define PARALLELISM_MIN_RE_TRACKING_STAT 5u
/*! @brief PLAUSIBILITY_DEVIATION_CORRIDOR */
#define PLAUSIBILITY_DEVIATION_CORRIDOR 1.0F
/*! @brief PLAUSIBILITY_S_CURVE_CORRIDOR */
#define PLAUSIBILITY_S_CURVE_CORRIDOR 1.5F
/*! @brief EXTOBJAPPROX_PARALLELISM_MIN_LENGTH_TRACE */
#define EXTOBJAPPROX_PARALLELISM_MIN_LENGTH_TRACE ((sint8)2)

/*! @brief PLAUSIBILITY_HIGHWAY_SLIP_OFFSET */
#define PLAUSIBILITY_HIGHWAY_SLIP_OFFSET DEG2RAD(1.0F)
/*! @brief TRACE_FUSION_MIN_TRACEPOINTS */
#define TRACE_FUSION_MIN_TRACEPOINTS 10L

/*! @brief MAXFUSIONSPEED */
#define MAXFUSIONSPEED (float32)(105.F / C_KMH_MS)
/*! @brief MAXFUSIONSPEEDWITHTRACEINFO */
#define MAXFUSIONSPEEDWITHTRACEINFO (float32)(75.F / C_KMH_MS)

/*! @brief maximum acceleration of object to trajectory*/
#define SI_AVLC_MAXACCELTRAJDIST 5.0f

/*! @brief time distanse for fusion sampeling*/
#define CP_SAMPLETIMEDIST (4.F)
/*! @brief minimum distanse for fusion sampeling (set to MAX to disable
 * faeture)*/
#define CP_SAMPLEDIST_MIN (50.F)
/*! @brief maximum distanse for fusion sampeling*/
#define CP_SAMPLEDIST_MAX (200.F)

/*! @brief minimum std for gradient update*/
#define CP_GRADUPDATE_STD_MIN (0.01F)

/*! @brief RE plausibility parameters for C1 ramp (ego vs re plausibilisation)*/
#define PLAUSIBILITY_HIGHWAY_HIGHSPEED (150.F / C_KMH_MS)
/*! @brief PLAUSIBILITY_HIGHWAY_LOWSPEED */
#define PLAUSIBILITY_HIGHWAY_LOWSPEED (80.F / C_KMH_MS)
/*! @brief PLAUSIBILITY_HIGHWAY_HIGHSPEED_A */
#define PLAUSIBILITY_HIGHWAY_HIGHSPEED_A 250.0F
/*! @brief PLAUSIBILITY_HIGHWAY_LOWSPEED_A */
#define PLAUSIBILITY_HIGHWAY_LOWSPEED_A 60.0F

/*! @brief EGO STD parameters for C1 ramp (weight of ego in grad update)*/
#define COURSE_GRADUPDATE_STD_HIGHSPEED (150.F / C_KMH_MS)
/*! @brief COURSE_GRADUPDATE_STD_LOWSPEED */
#define COURSE_GRADUPDATE_STD_LOWSPEED (40.F / C_KMH_MS)
/*! @brief COURSE_GRADUPDATE_STD_HIGHSPEED_A */
#define COURSE_GRADUPDATE_STD_HIGHSPEED_A 150.0F
/*! @brief COURSE_GRADUPDATE_STD_LOWSPEED_A */
#define COURSE_GRADUPDATE_STD_LOWSPEED_A 30.0F
/*! @brief trajectory noise*/
#define CP_TRAJ_NOISE_C1 2.0e-3f
/*! @brief CP_TRAJ_NOISE_C1_SQR */
#define CP_TRAJ_NOISE_C1_SQR (CP_TRAJ_NOISE_C1 * CP_TRAJ_NOISE_C1)

/*! @brief Minimal visibility distance in meters for lane marker visibility for
 * fusion with ACC course @unit:m */
#define CP_PAR_MIN_LANE_MARKER_VISIB_DIST 20.f

/*! @brief Minimal visibility distance for lane marker visibility (Ratio
 * Visibility_Dist/AVLC_Pickup_Dist) */
#define CP_PAR_MIN_CAM_LANE_FUS_DIST_RATIO 0.3f

/*!  @brief Minimal visibility timegap in seconds for lane marker visibility for
 * fusion with ACC course @unit:s */
#define CP_PAR_MIN_LANE_MARKER_VISIB_TIMEGAP 2.f

/*!  @brief Maximum camera curvature to use for fusion */
#define CP_PAR_MAX_CAM_CURVATURE 1e-2f

/*!  @brief Maximum ego speed for camera marker fusion */
#define CP_PAR_MAX_CAM_LANE_FUS_SPEED (72.f / C_KMH_MS)

/*!  @brief Minimum ego speed for camera marker fusion */
#define CP_PAR_MIN_CAM_LANE_FUS_SPEED (10.f / C_KMH_MS)

/*!  @brief Hysteriesis value used for thresholds ACC course fusion is on
 * @min:0.01 @max:1 */
#define CP_PAR_CAM_LANE_FUSION_HYST 0.75f

/*!  @brief Maximum camera lane angle for fusion of camera lane with ACC course
 * @unit:rad */
extern const volatile float32 CP_PAR_MAX_CAM_LANE_ANGLE;

/*!  @brief The activation threshold for lane change recognition: if DIM lane
change probability exceeds
this threshold, then fusion with camera course is activated */
#define CP_PAR_DIM_LANE_CHANGE_PROB_MIN 80

/*!  @brief The activation threshold for lane change recognition in lateral
acceleration deviation between
filtered (low-pass) lateral acceleration and driver intended lateral
acceleration */
#define CP_PAR_LAT_AVLC_THRES_CAM_LANE_ACTIVE 0.1f

/*!  @brief The activation time of how long camera lane fusion is active after a
 * lane change was detected @unit:s */
#define CP_PAR_CAM_LANE_CHANGE_ACTIVATION_TIME 2.5f

/*!  @brief The default lane marker offset when lane change trajectory
 * modification is active @unit:m @min:0 @max:3 */
#define CP_PAR_DEFAULT_LC_MARKER_OFFSET 1.5f

/*! @brief  The minimum distance of the lane marker on a given side for lane
change to assume that the next lane
marker is the destination lane Example: if set to 1 m, then that means when lane
change to the left is
active and the left lane marker is in 0.2m distance, then it is assumed that the
destination lane is
between the left and beyond left lane markers. @unit:m @min:0 @max:3 */
#define CP_PAR_LC_MIN_MARKER_DIST 1.0f

/*!  @brief The maximum distance of the lane marker on a given side for lane
change to assume that the next lane
marker is the destination lane Example: if set to 1 m, then that means when lane
change to the left is
active and the left lane marker is in 0.2m distance, then it is assumed that the
destination lane is
between the left and beyond left lane markers. @unit:m
@min:#CP_PAR_LC_MIN_MARKER_DIST @max:10 */
#define CP_PAR_LC_MAX_MARKER_DIST 5.6f

/*!  @brief The minimum distance of a lane marker in non-lane-change case
 * @unit:m @min:0 @max:3 */
#define CP_PAR_NO_LC_MIN_MARKER_DIST 0.05f

/*! @brief  The maximum distance of a lane marker in non-lane change case.
(I.e.: lane change already took
place and we are in the destination lane) @unit:m
@min:#CP_PAR_NO_LC_MIN_MARKER_DIST @max:5 */
#define CP_PAR_NO_LC_MAX_MARKER_DIST 4.2f

/*! @brief  Maximum lane width for camera lane fusion in non-lane change case
 * @unit:m @min:1 @max:10 */
#define CP_PAR_NO_LC_MAX_LANEWIDTH 5.0f

/*!  @brief Tolerance in Y direction when solving second order equation to find
X coordinate where
ego dynamics based trajectory meets the camera lane center @unit:m */
#define CP_CAM_LANE_FUSION_EGO_LANE_TOLERANCE 1.0f

/*! @brief  Parameter for maximum average driver intended curvature vs trace
distance
[in m] for a given trace to be fused into the ACC trajectory. @unit:m */
#define CP_PAR_MAX_TRACE_EGO_AVG_DIFF 1.25f

/*!  @brief Parameters for increasing trace fusion's gradient update standard
deviation
depending on average error [in m] from the driver intended curvature over the
next
second when lane available on given side. The add-on standard deviation is
linearly
ramped between CP_PAR_EGO_LA_TRACE_ERR_STD_DEV_MIN ..
CP_PAR_EGO_LA_TRACE_ERR_STD_DEV_MAX,
for average errors between CP_PAR_EGO_LA_TRACE_ERR_MIN ..
CP_PAR_EGO_LA_TRACE_ERR_MAX */

/*! Parameters for increasing trace fusion's gradient update standard deviation
depending on average error [in m] from the driver intended curvature over the
next second when directly driving on trace and no lane known in given direction.
*/

/*! Parameters for increasing trace fusion's standard deviation depending
on the distance of the trace at X=0. When lanes are splitting, then this
increase in stddev leads to lower weighting of far away lane objects */
/*! @brief CP_MOT_POINT_STDDEV_THRESH */
#define CP_MOT_POINT_STDDEV_THRESH 1.5f
/*! @brief CP_MIN_PEX_FOR_VALID_TRACE */
#define CP_MIN_PEX_FOR_VALID_TRACE 96u /*0.96f*/
/*! @brief CP_ONETRACEGRADFUSIONDIST */
#define CP_ONETRACEGRADFUSIONDIST 80.0F
/*!  @brief standard deviation for gradient update of traces */
#define CP_MOT_SLOPEANGLE_STDDEV DEG2RAD(5.f)

/*!  @brief maximum lateral Acceleration when following a trace in m/s2 */
#define CP_TRACE_MAX_PLAUSIBLE_LAT_ACCEL (8.f)

/*!  @brief Lower hysteresis limit for Valid EGO speed range */
#define CP_NAVI_HYST_MIN_VALID_SPEED (90.f / C_KMH_MS)
/*!  @brief Required threshold for Valid EGO speed range */
#define CP_NAVI_REQ_VALID_SPEED (100.f / C_KMH_MS)

#endif
