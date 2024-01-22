

#ifndef _DIM_MOD_DISTRACTION_PAR_INCLUDED
/*! @brief       _DIM_MOD_DISTRACTION_PAR_INCLUDED */
#define _DIM_MOD_DISTRACTION_PAR_INCLUDED
#include "dim.h"

/*! @brief       DIM_DISTRACTION_DEFAULT_RadioButtons
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_RadioButtons (FALSE)
/*! @brief       DIM_DISTRACTION_DEFAULT_SeatButtons
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_SeatButtons (FALSE)
/*! @brief       DIM_DISTRACTION_DEFAULT_WindowButtons
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_WindowButtons (FALSE)
/*! @brief       DIM_DISTRACTION_DEFAULT_MirrorButtons
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_MirrorButtons (FALSE)
/*! @brief       DIM_DISTRACTION_DEFAULT_InteriorLightsButtons
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_InteriorLightsButtons (FALSE)
/*! @brief       DIM_DISTRACTION_DEFAULT_ExteriorLightsButtons
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_ExteriorLightsButtons (FALSE)
/*! @brief       DIM_DISTRACTION_DEFAULT_SunRoofButtons
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_SunRoofButtons (FALSE)
/*! @brief       DIM_DISTRACTION_DEFAULT_SteeringWheelButtons
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_SteeringWheelButtons (FALSE)
/*! @brief       DIM_DISTRACTION_DEFAULT_DriverTired
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_DriverTired (FALSE)
/*! @brief       DIM_DISTRACTION_DEFAULT_StrColumnButtons
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_StrColumnButtons (FALSE)
/*! @brief       DIM_DISTRACTION_DEFAULT_MidConsoleButtons
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_MidConsoleButtons (FALSE)
/*! @brief       DIM_DISTRACTION_DEFAULT_OtherButtons
    @typical     False   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_DEFAULT_OtherButtons (FALSE)

/*---- definitions ----*/
/*! @brief       DIM_DISTRACTION_PAR_NormalConfidence
    @typical     100   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_PAR_NormalConfidence (100)
/*! @brief       DIM_DISTRACTION_PAR_MissingConfidenceDelta
    @typical     10   @unit -     @min -   @max -   */
#define DIM_DISTRACTION_PAR_MissingConfidenceDelta (10)

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief DIM_DISTRACTION_PAR_struct_t

    @general Struct for distraction timeout,maximum percentage and confidence
   timeout

    @conseq [ None ]

    @attention [ None ]

    */
typedef struct {
    /* distraction timeout */
    float32 DIM_DISTR_RADIO_SWITCH_TIME;  /*!<  DIM_DISTR_RADIO_SWITCH_TIME */
    float32 DIM_DISTR_SEAT_SWITCH_TIME;   /*!<  DIM_DISTR_RADIO_SWITCH_TIME */
    float32 DIM_DISTR_WINDOW_SWITCH_TIME; /*!<  DIM_DISTR_WINDOW_SWITCH_TIME*/
    float32 DIM_DISTR_MIRROR_SWITCH_TIME; /*!<  DIM_DISTR_WINDOW_SWITCH_TIME*/
    float32 DIM_DISTR_INT_LIGHTS_TIME;    /*!<  DIM_DISTR_INT_LIGHTS_TIME*/
    float32 DIM_DISTR_EXT_LIGHTS_TIME;    /*!<  DIM_DISTR_EXT_LIGHTS_TIME*/
    float32
        DIM_DISTR_SUN_ROOF_SWITCH_TIME; /*!< DIM_DISTR_SUN_ROOF_SWITCH_TIME */
    float32 DIM_DISTR_STR_WHL_SWITCH_TIME; /*!<  DIM_DISTR_STR_WHL_SWITCH_TIME*/
    float32 DIM_DISTR_TIRED_SWITCH_TIME;   /*!<  DIM_DISTR_TIRED_SWITCH_TIME*/
    float32
        DIM_DISTR_STR_COLUMN_SWITCH_TIME; /*!< DIM_DISTR_STR_COLUMN_SWITCH_TIME
                                             */
    float32
        DIM_DISTR_MID_CONSOLE_SWITCH_TIME; /*!<
                                              DIM_DISTR_MID_CONSOLE_SWITCH_TIME
                                              */
    float32 DIM_DISTR_OTHER_SWITCH_TIME;   /*!<  DIM_DISTR_OTHER_SWITCH_TIME */

    /* confidence timeout */
    float32
        DIM_DISTR_RADIO_CONFIDENCE_TIME; /*!< DIM_DISTR_RADIO_CONFIDENCE_TIME */
    float32
        DIM_DISTR_SEAT_CONFIDENCE_TIME; /*!< DIM_DISTR_SEAT_CONFIDENCE_TIME */
    float32
        DIM_DISTR_WINDOW_CONFIDENCE_TIME; /*!< DIM_DISTR_WINDOW_CONFIDENCE_TIME
                                             */
    float32
        DIM_DISTR_MIRROR_CONFIDENCE_TIME; /*!<
                                             DIM_DISTR_MIRROR_CONFIDENCE_TIME*/
    float32
        DIM_DISTR_INT_LIGHTS_CONF_TIME; /*!< DIM_DISTR_INT_LIGHTS_CONF_TIME */
    float32
        DIM_DISTR_EXT_LIGHTS_CONF_TIME;   /*!< DIM_DISTR_EXT_LIGHTS_CONF_TIME */
    float32 DIM_DISTR_SUN_ROOF_CONF_TIME; /*!<  DIM_DISTR_SUN_ROOF_CONF_TIME*/
    float32 DIM_DISTR_STR_WHL_CONF_TIME;  /*!< DIM_DISTR_STR_WHL_CONF_TIME */
    float32
        DIM_DISTR_TIRED_CONFIDENCE_TIME; /*!<  DIM_DISTR_TIRED_CONFIDENCE_TIME
                                            */
    float32
        DIM_DISTR_STR_COLUMN_CONF_TIME; /*!< DIM_DISTR_STR_COLUMN_CONF_TIME */
    float32
        DIM_DISTR_MID_CONSOLE_CONF_TIME; /*!< DIM_DISTR_MID_CONSOLE_CONF_TIME */
    float32
        DIM_DISTR_OTHER_CONFIDENCE_TIME; /*!< DIM_DISTR_OTHER_CONFIDENCE_TIME */

    /* maximum percentage */
    percentage_t
        DIM_DISTR_RADIO_SWITCH_MAX_P; /*!< DIM_DISTR_RADIO_SWITCH_MAX_P */
    percentage_t
        DIM_DISTR_SEAT_SWITCH_MAX_P; /*!< DIM_DISTR_SEAT_SWITCH_MAX_P */
    percentage_t
        DIM_DISTR_WINDOW_SWITCH_MAX_P; /*!< DIM_DISTR_WINDOW_SWITCH_MAX_P */
    percentage_t
        DIM_DISTR_MIRROR_SWITCH_MAX_P; /*!< DIM_DISTR_MIRROR_SWITCH_MAX_P */
    percentage_t DIM_DISTR_INT_LIGHTS_MAX_P; /*!< DIM_DISTR_INT_LIGHTS_MAX_P */
    percentage_t DIM_DISTR_EXT_LIGHTS_MAX_P; /*!< DIM_DISTR_EXT_LIGHTS_MAX_P */
    percentage_t
        DIM_DISTR_SUN_ROOF_SWITCH_MAX_P; /*!< DIM_DISTR_SUN_ROOF_SWITCH_MAX_P */
    percentage_t
        DIM_DISTR_STR_WHL_SWITCH_MAX_P; /*!<  DIM_DISTR_STR_WHL_SWITCH_MAX_P */
    percentage_t
        DIM_DISTR_TIRED_SWITCH_MAX_P; /*!< DIM_DISTR_TIRED_SWITCH_MAX_P */
    percentage_t
        DIM_DISTR_STR_COLUMN_SWITCH_MAX_P; /*!<
                                              DIM_DISTR_STR_COLUMN_SWITCH_MAX_P
                                              */
    percentage_t
        DIM_DISTR_MID_CONSOLE_SWITCH_MAX_P; /*!<
                                               DIM_DISTR_MID_CONSOLE_SWITCH_MAX_P
                                               */
    percentage_t
        DIM_DISTR_OTHER_SWITCH_MAX_P; /*!< DIM_DISTR_OTHER_SWITCH_MAX_P */
} DIM_DISTRACTION_PAR_struct_t;

#endif /* _DIM_MOD_DISTRACTION_PAR_INCLUDED */
