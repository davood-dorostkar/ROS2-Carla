
/** @defgroup vlc_obj VLC_OBJ
   @ingroup acc_long_veh

@{ */
#ifndef OBJ_HE_INCLUDED
#define OBJ_HE_INCLUDED

/* INCLUDES */
#include "vlc_glob_ext.h"
#include "vlc_long_cfg.h"

/* Currently disable content, since moved to RTE. Only needed if non-RTE
 * environment being used. */
#if ((!defined(_RTE_TYPE_H_)) && (!defined(RTE_TYPE_H)) && (0))
/* TYPE DEFINITIONS */

#define Obj_class_none (obj_class_t)(0)
#define Obj_class_passenger_car (obj_class_t)(1)
#define Obj_class_truck (obj_class_t)(2)
#define Obj_class_cycle (obj_class_t)(3)
#define Obj_class_pedestrian (obj_class_t)(4)
typedef enum_t
    obj_class_t; /*!<
                    @values:Obj_class_none,Obj_class_passenger_car,Obj_class_truck,Obj_class_cycle,Obj_class_pedestrian
                    */

/*! @brief Bit that reflect the object status */
typedef struct obj_status_t {
    ubit8_t MEASURED : 1; /*Object measured*/
    ubit8_t TRACKED : 1;  /*Object only tracked, not seen*/
    ubit8_t NEW : 1;      /*Object is new*/
    ubit8_t STANDING : 1; /*stationary object*/
    ubit8_t STOPPED : 1;  /*object was seen moving before*/
    ubit8_t MOVING : 1;   /*object is moving*/
    ubit8_t DETECTED : 1; /*object is detected*/
    ubit8_t : 1;
} obj_status_t;

/*! @brief All object features according to AUTOSAR */
typedef struct {
    velocity_t REL_LONG_SPEED;
    velocity_t REL_LAT_SPEED;
    distance_t LONG_DISPLACEMENT;
    distance_t LAT_DISPLACEMENT;
    acceleration_t REL_LONG_ACCEL;
    acceleration_t REL_LAT_ACCEL;
    confidence_t QUALITY;
#if OBJ_CLASS_IN_LONG_CTRL_NEEDED
    obj_class_t CLASS;
#endif
    obj_status_t OBJECT_STATUS;
    distance_t WIDTH;
    ObjNumber_t OBJECT_ID;
} object_t;

#else

/* nothing to do here */

#endif

#endif /* #ifndef OBJ_HE_INCLUDED */

/** @} end defgroup */