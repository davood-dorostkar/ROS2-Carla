

#ifndef _DIM_EBA_H_INCLUDED
#define _DIM_EBA_H_INCLUDED

#include "dim_mod_activity.h"
#include "dim_mod_attention.h"
#include "dim_mod_feedback.h"

#include "dim_mod_activity_par_eba.h"
#include "dim_mod_attention_par_eba.h"
#include "dim_mod_feedback_par_eba.h"

/* ****************************************************************
    TYPEDEF ENUM DimEBAHypothesisIdx_t
    **************************************************************** */
/*! @brief DIM EBA Hypthesis IDs

    @general Identifiers for different DIM EBA Hypothesis

    @conseq [None]

    @attention [None]

    */
typedef enum {
    DIM_EBA_HYP_IDX_ATTENTION =
        0u, /*!< DIM EBA Attention Hypothesis identification */
    DIM_EBA_HYP_IDX_FEEDBACK =
        1u, /*!< DIM EBA Feedback Hypothesis identification */
    DIM_EBA_HYP_IDX_ACTIVITY =
        2u /*!< DIM EBA Activity Hypothesis identification */
} DimEBAHypothesisIdx_t;

/* ****************************************************************
    TYPEDEF STRUCT DIMInteralDataEBA_t
    **************************************************************** */
/*! @brief DIM Internal EBA Data Structure

    @general Internal Data Structure holding different DIM EBA Hypothesis

    @conseq [None]

    @attention [None]

    */
typedef struct {
    DIMInternalDataModAttention_t
        Internal_Attention; /*!< DIM Internal Attention */
    DIMInternalDataModFeedback_t
        Internal_Feedback; /*!< DIM Internal Feedback */
    DIMInternalDataModActivity_t
        Internal_Activity; /*!< DIM Internal Activity */
} DIMInteralDataEBA_t;

#endif
