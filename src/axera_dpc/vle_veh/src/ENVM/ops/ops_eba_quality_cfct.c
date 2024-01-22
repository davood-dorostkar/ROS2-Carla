/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
INCLUDES
*****************************************************************************/
#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "TM_Global_Types.h"
#include "stddef.h"
#include "assert.h"

#include "ops_par.h"
#include "ops_eba_quality_cfct.h"
#include "ops_eba_mosa.h"

/*****************************************************************************
 DEFINES
*****************************************************************************/

/*! Minimum DistX for vehicles under which missing camera confirmation will
not lead to revocation of 'bCameraConfirmed' flag and setting of
'FPS_EBA_INH_BRAKE_L2' bit in inhibition flags. Basically this means if an
object looses it's camera confirmation under this X distance, then the
'FPS_EBA_INH_BRAKE_L2' bit does not get set if the object was camera
confirmed. @unit: m */
#define FPS_EBA_PAR_VEH_MIN_DISTX_KEEP_CAM_CONF 7.f
/*! Minimum relative X speed under which missing camera confirmation will be
kept for vehicles @unit: m/s */
#define FPS_EBA_PAR_VEH_MIN_VREL_KEEP_CAM_CONF (-20.f / 3.6f)
/*! Minimum number of camera confirmation cycles for camera hold on vehicles */
#define FPS_EBA_PAR_VEH_MIN_COUNTER_CAM_CONF (10)
/*! Limit camera confirmation hold to given parameter speed */
#define FPS_EBA_PAR_VEH_MAX_EGO_SPEED_CAM_CONF (35.f / 3.6f)
/*! Limit camera confirmation hold for vehicle to curvature limit */
#define FPS_EBA_PAR_VEH_MAX_EGO_CURVE_CAM_CONF (1.f / 200.f)

/*! Maximum counter for number of cycles confirmed by camera */
#define FPS_PAR_MAX_CAM_CONF_COUNTER_VALUE 30u

#define FPSEBACFCT_MAX_OBJ_QUAL_NO_CAM CDObjEbaQual_NoCamConf /* (96u) */
#define FPSEBACFCT_MAX_OBJ_QUAL_NO_CAM_WSTAT \
    CDObjEbaQual_NoCamAvail /* (97u)         \
                             */
#define FPS_PAR_EBA_CAM_CONF_PICKUP_GATE (90u)
#define FPS_PAR_EBA_CAM_CONF_DROP_GATE (70u)

#define FPS_PAR_CONST_RADARCLASS_CYCLE_COUNT (12u) /* 720ms */

/*! Configuration switch to enable limiting maximum distY error in near
range for camera confirmation */

/*! Configuration switch to enable applying the camera VrelY for crossing
 * pedestrians */
#define FPS_EBA_CFG_CHECK_CAMERA_VREL_Y_CROSSING_PEDESTRIAN SWITCH_OFF

/*! Configuration switch to abort braking in case camera VabsY sinks below a
 * certain value */
#define FPS_EBA_CFG_CHECK_CAMERA_VABSY_TO_ABORT_BRAKING SWITCH_OFF

/*! Minimum obstacle probability of stationary non-pedestrian objects for
 * radar-only L2 activation */
#define FPS_EBA_RADAR_ONLY_MIN_L2_OBSTACLE_PROB 50u
/*! Minimum object quality for radar-only L2 activation possibility */
#define FPS_EBA_RADAR_ONLY_MIN_L2_EBA_QUAL 96u
/*! Minimum object quality for radar-only L3 activation possibility */
#define FPS_EBA_RADAR_ONLY_MIN_L3_EBA_QUAL 98u
/*! Number of linear sample points for maximum distance at which radar-only
 * activation is possible */
#define FPS_EBA_RADAR_ONLY_MAXDIST_POINTS 2u
/*! Linear interpolation table specifying maximum object distance for L3 brake
reaction radar-only.
First value of pairs is always ego speed and second value is distance */

static const GDBVector2_t
    FPSEBARadarOnlyL3MaxDist[FPS_EBA_RADAR_ONLY_MAXDIST_POINTS] = {
        {5.f, 6.f}, {12.f, 15.f}};

#if defined(EM_FPS_PAR_FULL_QUAL_OBJ)
static void FPSEBACalibration(sint8 ObjNr);
#endif

/******************************************************************************
@brief        Initialize all global variables of this module here

@description  Initialize all global variables of this module here.

@param[in]    void

@return       void
******************************************************************************/
void FPS_v_InitCustomQuality(void) {}

#if defined(EM_FPS_PAR_FULL_QUAL_OBJ)
/******************************************************************************
@brief        Modifies EBA pre selection quality by calibration

@description  Boost EBA quality in dependence of tunable modified by RAM
calibration

@param[in]    ObjNr : The index of the valid (non-free) object to check

@return       None
******************************************************************************/
static void FPSEBACalibration(sint8 ObjNr) {
    const f32_t maxAllowDist_c = 80.F;     /* max. used longitudinal distance */
    const uint8 minRequiredQuality_c = 1u; /* minimum required quality */
    const uint8 boostedQuality_c = 100u;   /* boosted output quality */

    /* Select object in dependency of dynamic object property selected by
     * calibration mask */
    if (((1uL << EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr)) &
         EM_FPS_PAR_FULL_QUAL_OBJ) > 0uL) {
        /* Test prerequisites to boost eba quality */
        if ((EM_INT_OBJ_LONG_DISPLACEMENT(ObjNr) < maxAllowDist_c) &&
            (Envm_INT_OBJ_GET_EBA_MOV_PRESEL_QUALITY(ObjNr) >
             minRequiredQuality_c)) {
            /* boost value up to max. specified value */
            Envm_INT_OBJ_GET_EBA_MOV_PRESEL_QUALITY(ObjNr) =
                MAX(boostedQuality_c,
                    Envm_INT_OBJ_GET_EBA_MOV_PRESEL_QUALITY(ObjNr));
        }
    }
}
#endif

/******************************************************************************
@brief        Modifies EBA preselection quality by custom requirements

@description  Modifies EBA preselection quality by custom requirements

@param[in]    ObjNr : The index of the valid (non-free) object to check

@return       None
******************************************************************************/
void FPSEBAPreSelCustomFct(sint8 ObjNr) {
    EMFPSEBACustFctObj_t *const pEbaCust =
        (EMFPSEBACustFctObj_t *)&FctPreselEBAObj[ObjNr].sCustomData;

    /* For new / deleted objects reset EBA camera confirmation information */
    if ((EM_INT_OBJ_MAINTENANCE_STATE(ObjNr) != MT_STATE_PREDICTED) &&
        (EM_INT_OBJ_MAINTENANCE_STATE(ObjNr) != MT_STATE_MEASURED)) {
        pEbaCust->bCameraConfirmed = FALSE;
        pEbaCust->bCamConfHold = FALSE;
        pEbaCust->bCamConfAsPed = FALSE;
        pEbaCust->uCamConfCounter = 0u;
    }

#if defined(EM_FPS_PAR_FULL_QUAL_OBJ)
    FPSEBACalibration(ObjNr);
#endif /* EM_FPS_PAR_FULL_QUAL_OBJ */

#if defined(FPS_CFG_OBJECT_MOSA_CHECK) && (FPS_CFG_OBJECT_MOSA_CHECK == 1)
    /* suppress brake reactions for all known issues */
    if (OPSEBA_MOSA_GetObjPermission(ObjNr) < OPSEBA_MOSA_PERMISSION_LVL1) {
        SET_BIT(EM_INT_OBJ_GET_EBA_INHIBITIONMASK(ObjNr),
                (FPS_EBA_INH_BRAKE_L2 | FPS_EBA_INH_BRAKE_L3));
    }
#endif

    // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/7 changed by
    // guotao 20200423 start
    // decrease aeb quality while in sharp turning road situation
    // radius of turning cycle less than 100 meters
    if (fABS(GET_EGO_RAW_DATA_PTR->Lateral.DrvIntCurve.Curve) >= 0.02 ||
        fABS(GET_EGO_RAW_DATA_PTR->Lateral.Curve.Curve) >= 0.01) {
        Envm_p_GetPrivObject(ObjNr)->EBAPresel.ucEbaMovingObjQuality =
            MIN(Envm_p_GetPrivObject(ObjNr)->EBAPresel.ucEbaMovingObjQuality,
                (uint8)(100 * OPS_EBA_UNCONFIRMED_OBJ_MIN_QUALITY));
        // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/7 changed by
        // guotao 20200423 end
    } else if ((EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) ==
                OBJECT_PROPERTY_STATIONARY) &&
               // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/12
               // changed by guotao 20200427 start
               !Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(ObjNr) &&
               // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/12
               // changed by guotao 20200427 end
               !Envm_INT_GET_Envm_OBJ(ObjNr).SensorSpecific.bCamConfirmed) {
        // Ticket: http://119.3.139.124:10101/tickets/DFA-ZT19.git/8 changed by
        // guotao 20200424 start
        // decrease stationary radar only object aeb quality to solve the false
        // triggering.
        // all the AEB triggered object need to be confirmed by camera
        Envm_p_GetPrivObject(ObjNr)->EBAPresel.ucEbaMovingObjQuality =
            MIN(Envm_p_GetPrivObject(ObjNr)->EBAPresel.ucEbaMovingObjQuality,
                (uint8)(100 * OPS_EBA_UNCONFIRMED_OBJ_MIN_QUALITY));
    }
    // Ticket: http://119.3.139.124:10101/tickets/DFA-ZT19.git/8 changed by
    // guotao 20200424 end
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */