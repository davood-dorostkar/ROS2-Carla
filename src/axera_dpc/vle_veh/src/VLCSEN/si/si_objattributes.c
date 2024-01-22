/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "si.h"
#include "si_par.h"

/*****************************************************************************
  MODULE GLOBAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULE GLOBAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL MACROS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL TYPES
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL CONSTANTS
*****************************************************************************/

#define TP_SI_VDY_A_OBJ_FILTER_TIME 200.0F
#define TP_SI_MERGED_A_OBJ_FILTER_TIME_MIN 0.0F
#define TP_SI_MERGED_A_OBJ_STD_DEV_MAX 1.0F

#define SI_OBJLENGTH_CAR (5.f)
#define SI_OBJLENGTH_TRUCK (18.f)
#define SI_OBJLENGTH_SHORT (1.5f)
#define SI_OBJLENGTH_DEFAULT (5.f)

/*****************************************************************************
  MODULE LOCAL TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL VARIABLES
*****************************************************************************/

SET_MEMSEC_VAR(SIDetermineObjectAbsSpeed)
static fVelocity_t SIDetermineObjectAbsSpeed(const ObjNumber_t ObjId);
SET_MEMSEC_VAR(SIDetermineObjectAbsAccel)
static float32 SIDetermineObjectAbsAccel(const ObjNumber_t ObjId);

/*************************************************************************************************************************
  Functionname:    SIDetermineObjectAbsSpeed */
static fVelocity_t SIDetermineObjectAbsSpeed(const ObjNumber_t ObjId) {
    fVelocity_t fAbsSpd;
    fAbsSpd = OBJ_LONG_VREL(ObjId) + EGO_SPEED_X_OBJ_SYNC;
    return fAbsSpd;
}

/*************************************************************************************************************************
  Functionname:    SIDetermineObjectAbsAccel */
static float32 SIDetermineObjectAbsAccel(const ObjNumber_t ObjId) {
    float32 fAbsAccel = 0.0F;

    if (OBJ_IS_NEW(ObjId)) {
        /*set accel always to 0*/
        fAbsAccel = 0.0F;
    } else {
        /* Object old enough, start filtering it's absolute acceleration */
        /* Get previous absolute acceleration of object for filtering */
        const float32 fPrevAbsAccel = OBJ_ABS_ACCEL_X(ObjId);
        float32 fFilterConst;
        float32 fEgoAccelXStd;

        /* Calculate raw absolute acceleration of object */
        fAbsAccel = OBJ_LONG_AREL(ObjId) + EGO_ACCEL_X_OBJ_SYNC;

        /* Get standard deviation of ego accleration in X direction */
        fEgoAccelXStd = EGO_MERGED_EGO_ACCEL_X_STD_OBJ_SYNC;

        /* Limit standard deviation to a given maximum */
        if (fEgoAccelXStd > TP_SI_MERGED_A_OBJ_STD_DEV_MAX) {
            fEgoAccelXStd = TP_SI_MERGED_A_OBJ_STD_DEV_MAX;
        }

        /* Calculate a filtering constant: the value will be between
        [TP_SI_MERGED_A_OBJ_FILTER_TIME_MIN .. TP_SI_VDY_A_OBJ_FILTER_TIME]
        depending on the ego acceleration standard deviation */

        fFilterConst = TP_SI_MERGED_A_OBJ_FILTER_TIME_MIN +
                       (fEgoAccelXStd * ((TP_SI_VDY_A_OBJ_FILTER_TIME -
                                          TP_SI_MERGED_A_OBJ_FILTER_TIME_MIN) /
                                         (TP_SI_MERGED_A_OBJ_STD_DEV_MAX)));

        /* Normalize the filter constant */
        fFilterConst /= (float32)(SI_CYCLE_TIME * 1000.0F);

        /* The final absolute acceleration is the previous abs acceleration and
        the new raw value, with the previous value weighted with the filter
        constant */
        fAbsAccel = ((fPrevAbsAccel * fFilterConst) + fAbsAccel) /
                    (fFilterConst + 1.0F);
    }

    return fAbsAccel;
}

/*************************************************************************************************************************
  Functionname:    SICalcObjAttributes */
void SICalcObjAttributes(void) {
    ObjNumber_t ObjNr;
    for (ObjNr = (ObjNumber_t)(Envm_N_OBJECTS - 1); ObjNr >= 0; ObjNr--) {
        OBJ_ABS_VELO_X(ObjNr) = SIDetermineObjectAbsSpeed(ObjNr);
        OBJ_ABS_ACCEL_X(ObjNr) = SIDetermineObjectAbsAccel(ObjNr);
    }
}

/*************************************************************************************************************************
  Functionname:    SIGetObjectLength */
float32 SIGetObjectLength(const ObjNumber_t uiObject) {
    float32 fObjectLength;
    /*! @todo : Object width should be derived from measured values of OT, if
     * available */
    if (OBJ_CLASSIFICATION(uiObject) == CR_OBJCLASS_CAR) {
        fObjectLength = SI_OBJLENGTH_CAR;
    } else if (OBJ_CLASSIFICATION(uiObject) == CR_OBJCLASS_TRUCK) {
        fObjectLength = SI_OBJLENGTH_TRUCK;
    } else if ((OBJ_CLASSIFICATION(uiObject) == CR_OBJCLASS_POINT) ||
               (OBJ_CLASSIFICATION(uiObject) == CR_OBJCLASS_MOTORCYCLE) ||
               (OBJ_CLASSIFICATION(uiObject) == CR_OBJCLASS_BICYCLE) ||
               (OBJ_CLASSIFICATION(uiObject) == CR_OBJCLASS_PEDESTRIAN)) {
        fObjectLength = SI_OBJLENGTH_SHORT;
    } else {
        if (OBJ_DYNAMIC_PROPERTY(uiObject) == CR_OBJECT_PROPERTY_STATIONARY) {
            fObjectLength = MINMAX_FLOAT(SI_OBJLENGTH_CAR, SI_OBJLENGTH_TRUCK,
                                         OT_GET_OBJ_LENGTH(uiObject));
        } else {
            fObjectLength = SI_OBJLENGTH_DEFAULT;
            /*!    OBJCLASS_UNDEFINED   = 0U,
                  CR_OBJCLASS_WIDE        = 5U,
                  OBJCLASS_BRIDGE      = 7U
            @todo: realistic for all of these types ? */
        }
    }
    return fObjectLength;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
