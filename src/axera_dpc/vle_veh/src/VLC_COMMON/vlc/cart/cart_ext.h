
#ifndef CART_HE
#define CART_HE

/** @defgroup vlc_long_cart VLC_LONG_CART ( CARTRONIC Interface )
 Definition of the interface between Driver Assistance System and Longitudinal
Dynamics Manager
   @ingroup acc_long_veh

  - A CARTRONIC interface specification can be found under <A
HREF="../../../cartronic_interface_spec.mht">cartronic_interface_spec.mht</A>
@{ */

/* includes */
#include "vlc_glob_ext.h"

#include "vlc_long_cfg.h" /* needed for CFG_VLC_LODM */
#include "pt_ext.h"
#include "brake_ext.h"
#include "chassis_ext.h"

/*! Shift request */
#define Cart_no_shift_request ((cart_shift_request_t)0)
#define Cart_keep_gear ((cart_shift_request_t)1)
#define Cart_shift_down ((cart_shift_request_t)2)

#define Das_Mode_Normal 0
#define Das_Mode_Eco 1
#define Das_Mode_Sport 2

typedef enum_t
    cart_shift_request_t; /*!<
                             @values:Cart_no_shift_request,Cart_keep_gear,Cart_shift_down
                             */

/*! @brief Driver assistance system status bits */
typedef struct cart_das_status_t {
    ubit8_t DAS_FAIL_REVERSABLE : 1;
    ubit8_t DAS_FAIL_IRREVERSABLE : 1;
    ubit8_t DAS_PREFILL : 1;
    ubit8_t DAS_STAND_STILL : 1;
    ubit8_t DAS_DRIVE_OFF : 1;
    ubit8_t DAS_DRIVE_OFF_LAST_CYC : 1;
    ubit8_t DAS_DRIVE_OFF_INHIBIT : 1;
    ubit8_t DAS_ENGAGED : 1;
    ubit8_t DAS_SHUTOFF_REQ : 1;
    ubit8_t DAS_OVERRIDE : 1;
    ubit8_t DAS_OFF : 1;
    ubit8_t DAS_LIM : 1;
    ubit8_t : 4;
} cart_das_status_t;

/*! @brief Longitudinal dynamics management status bits */
typedef struct cart_lodm_status_t {
    ubit8_t DAS_RESET : 1;   /*driver assistance system is resetted*/
    ubit8_t DAS_ENABLE : 1;  /*driver assistance system can be enabled*/
    ubit8_t DAS_INHIBIT : 1; /*driver assistance system can be inhibited*/
    ubit8_t DAS_MODE : 2; /*driver assistance system mode, including: normal 0,
                             eco 1, sport 2*/
    ubit8_t DC_LIM_ACCEL : 1;    /*acceleration request is limited*/
    ubit8_t DC_LIM_DECEL : 1;    /*deceleration request is limited*/
    ubit8_t DAS_SHUTOFF_ACQ : 1; /*shutoff was acknowledged by LODM*/
    ubit8_t OVERRIDE_ACCEL : 1;  /*driver override by accelerator pedal*/
    ubit8_t OVERRIDE_DECEL : 1;  /*driver override by decelerator pedal*/
    ubit8_t STANDSTILL : 1;      /*standstill detected*/

    ubit8_t BRAKE_LIGHT_REQ : 1;
    ubit8_t : 4;
} cart_lodm_status_t;

/*! @brief Data from driver assistance system to longitudinal dynamics
 * management */
typedef struct cart_das_output_data_t {
    acceleration_t MIN_REQ_ACCEL;
    acceleration_t MAX_REQ_ACCEL;
    cart_das_status_t DAS_STAT;
} cart_das_output_data_t;

/*! Result of the decision between engine and brake */
#define Cart_arbit_none (cart_arbit_mode_t)0
#define Cart_arbit_engine (cart_arbit_mode_t)1
#define Cart_arbit_brake (cart_arbit_mode_t)2
typedef enum_t
    cart_arbit_mode_t; /*!<
                          @values:Cart_arbit_none,Cart_arbit_engine,Cart_arbit_brake
                          */

/*! @brief Validity bits for data from longitudinal dynamics management to
 * driver assistance system */
typedef struct cart_lodm_data_valid_t {
    ubit8_t VEHICLE_SPEED : 1;
    ubit8_t VEHICLE_ACCEL : 1;
    ubit8_t INIT_ACCEL : 1;
    ubit8_t : 5;
} cart_lodm_data_valid_t;

/*! @brief Data from longitudinal dynamics management to driver assistance
 * system */
typedef struct cart_das_input_data_t {
    velocity_t VEHICLE_SPEED;
    acceleration_t VEHICLE_ACCEL;
    acceleration_t A_INIT;
    pt_status_t PT_STAT;
    brake_status_t BRAKE_STAT;
    chassis_status_t CHASSIS_STAT;
    cart_lodm_status_t LODM_STAT;
    cart_lodm_data_valid_t DATA_VALID;
} cart_das_input_data_t;

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */
extern void CART_INIT_DAS_OUTPUT_DATA(cart_das_output_data_t* das_output_data);
extern void CART_INIT_DAS_INPUT_DATA(cart_das_input_data_t* das_input_data);

#endif

/** @} end defgroup */
