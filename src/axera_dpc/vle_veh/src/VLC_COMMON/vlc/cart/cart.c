/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "vlc_glob_ext.h"
#include "cart_ext.h"

/******************************************************************************

  FUNCTION NAME:  CART_INIT_DAS_INPUT_DATA */ /*!

                                    Description:    Initialize DAS_INPUT_DATA

                                    @param[in]      das_input_data

                                    @param[out]     -

                                    @return         void

                                  *****************************************************************************/
void CART_INIT_DAS_INPUT_DATA(cart_das_input_data_t* das_input_data) {
    das_input_data->A_INIT = (acceleration_t)0;
    das_input_data->DATA_VALID.VEHICLE_ACCEL = FALSE;
    das_input_data->DATA_VALID.VEHICLE_SPEED = FALSE;
    das_input_data->LODM_STAT.DAS_ENABLE = TRUE;
    das_input_data->LODM_STAT.DAS_INHIBIT = FALSE;
    das_input_data->LODM_STAT.DAS_RESET = FALSE;
    das_input_data->LODM_STAT.DAS_MODE = 0;
    das_input_data->CHASSIS_STAT.ABS_ACT = FALSE;
    das_input_data->CHASSIS_STAT.ESP_ACT = FALSE;
    das_input_data->LODM_STAT.DC_LIM_ACCEL = FALSE;
    das_input_data->LODM_STAT.DC_LIM_DECEL = FALSE;
    das_input_data->CHASSIS_STAT.TCS_ACT = FALSE;
    das_input_data->CHASSIS_STAT.TCS_ESP_OFF = FALSE;
    das_input_data->CHASSIS_STAT.PB_ACT = FALSE;
    das_input_data->LODM_STAT.DAS_SHUTOFF_ACQ = FALSE;
    das_input_data->VEHICLE_ACCEL = (acceleration_t)0;
    das_input_data->VEHICLE_SPEED = (velocity_t)0;
    das_input_data->LODM_STAT.BRAKE_LIGHT_REQ = FALSE;
    das_input_data->BRAKE_STAT.PEDAL_INIT_TRAVEL = FALSE;
    das_input_data->PT_STAT.CLUTCH_OPEN = FALSE;
    das_input_data->PT_STAT.KICKDOWN = FALSE;
    das_input_data->LODM_STAT.OVERRIDE_ACCEL = FALSE;
    das_input_data->LODM_STAT.OVERRIDE_DECEL = FALSE;
    das_input_data->LODM_STAT.STANDSTILL = FALSE;
}

/******************************************************************************

  FUNCTION NAME:  CART_INIT_DAS_OUTPUT_DATA */ /*!

                                   Description:    Initialize DAS_OUTPUT_DATA

                                   @param[in]      -

                                   @param[out]     das_output_data

                                   @return         void

                                 *****************************************************************************/
void CART_INIT_DAS_OUTPUT_DATA(cart_das_output_data_t* das_output_data) {
    das_output_data->DAS_STAT.DAS_DRIVE_OFF = FALSE;
    das_output_data->DAS_STAT.DAS_ENGAGED = FALSE;
    das_output_data->DAS_STAT.DAS_FAIL_IRREVERSABLE = FALSE;
    das_output_data->DAS_STAT.DAS_FAIL_REVERSABLE = FALSE;
    das_output_data->DAS_STAT.DAS_LIM = FALSE;
    das_output_data->DAS_STAT.DAS_OVERRIDE = FALSE;
    das_output_data->DAS_STAT.DAS_PREFILL = FALSE;
    das_output_data->DAS_STAT.DAS_STAND_STILL = FALSE;
    das_output_data->MAX_REQ_ACCEL = (acceleration_t)0;
    das_output_data->MIN_REQ_ACCEL = (acceleration_t)0;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */