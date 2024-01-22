

#ifndef _DIM_MOD_DRV_FDBCK_INCLUDED
#define _DIM_MOD_DRV_FDBCK_INCLUDED
#include "dim.h"
#include "dim_mod_feedback_par.h"

/*! @brief       FeedBack buffer size
    @general     Dim feedback buffer size used for Accelbuffer
    @conseq      [none]
    @attention   [none]
    @typical     [none]     @unit [none]    @min 0    @max 20   */
#define DIM_FEEDBACK_DEF_ACCELBUFFER_SIZE (20u)

/*! @brief       FeedBack buffer scale
    @general     Dim feedback buffer Scale used for AccelBuffer
    @conseq      [none]
    @attention   [none]
    @typical     [none]     @unit [none]    @min 0    @max 10   */
#define DIM_FEEDBACK_ACCEL_BUFFER_SCALE (10.0f)

/*! @brief       FeedBack Filter time
    @general     Dim feedback use for fliter time defination
    @conseq      [none]
    @attention   [none]
    @typical     [none]     @unit [none]    @min 0    @max 0.25   */
#define DIM_FEEDBACK_DEF_GRAD_FILTER_TIME (0.25f)

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Dim data modfeedback

    @general All data for feedback are clubed for data feedback

    @conseq [none]

    @attention [none]

    */
typedef struct {
    sint16 iProbability;          /*!< save hypothesis probability @unit:%*/
    sint16 iConfidence;           /*!< save hypothesis confidence @unit:%*/
    float32 fNegFeedbackTime;     /*!< timer for negative feedback  @unit:s*/
    float32 fNegWeakFeedbackTime; /*!< timer for negative feedback  @unit:s*/
    float32 fPosFeedbackTime;     /*!< timer for positive feedback  @unit:s*/
    float32 fFallIntoBrakeTime; /*!< timer for not falling into brake @unit:s*/
    float32 fCurrentAccelGrad;  /*!< current  @unit:m/s^3*/
    boolean bDriverWasBraking;  /*!< driver was braking in the last n cycles
                                   @unit:bool*/
    uint8 uiAccelBufferPointer; /*!< current position in AccelBuffer*/
    sint8
        iAccelBuffer[DIM_FEEDBACK_DEF_ACCELBUFFER_SIZE]; /*!< buffer for last
                                                            accelerations
                                                            (scaled by 10)
                                                            @unit:m/s^2
                                                            @resolution:0.1*/
    boolean
        bBoolBuffer[DIM_FEEDBACK_DEF_ACCELBUFFER_SIZE]; /*!< buffer where the
                                                           information "driver
                                                           braking" @unit:bool*/
} DIMInternalDataModFeedback_t;

extern eGDBError_t DIMInitModuleFeedback(
    DIMInternalDataModFeedback_t *pInternalData);
extern eGDBError_t DIMRunModuleFeedback(
    float32 fCycleTime,
    const DIMInputDataGlobal_t *pInputData,
    GDB_DMHypothesis_t *pOutHypothesis,
    DIMInternalDataModFeedback_t *pInternalData,
    const DIM_FEEDBACK_PAR_struct_t *pDIM_feedback_par);

#endif /*_DIM_MOD_DRV_FDBCK_INCLUDED*/
