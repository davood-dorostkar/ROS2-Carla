

/*****************************************************************************
  MACROS
*****************************************************************************/

#ifndef VLC_VEH_ACCESS_H_INCLUDED
#define VLC_VEH_ACCESS_H_INCLUDED

#ifdef ALGO_INLINE
#undef ALGO_INLINE
#endif
#define ALGO_INLINE static inline

ALGO_INLINE VED_VehDyn_t const* VLCVeh_p_GetVDYDynRaw(void);
ALGO_INLINE float32 VLCVeh_f_GetEgoVelRaw(void);
ALGO_INLINE float32 VLCVeh_f_GetEgoAccelRaw(void);
ALGO_INLINE VED_VehPar_t const* VLCVeh_p_GetVehPar(void);
ALGO_INLINE float32 VLCVeh_f_GetCycleTime(void);
ALGO_INLINE VLCVehFrame_t* VLCVeh_p_GetVLCVehFrame(void);
ALGO_INLINE VLC_HypothesisIntf_t const* VLCVeh_p_GetHypothesisIntf(void);
ALGO_INLINE Hypothesis_t const* VLCVeh_p_GetHypothesis(sint32 iHyp);
ALGO_INLINE HypoIntfDegr_t const* VLCVeh_p_GetHypDegradation(void);
ALGO_INLINE Com_AlgoParameters_t const* VLCSen_p_GetAlgoParameters(void);
ALGO_INLINE VLC_Parameters_t const* VLCSen_p_GetCPARVLCParameters(void);
ALGO_INLINE VLC_SADOutputCustom_t* VLCVeh_p_GetHeadOutCustom(void);
ALGO_INLINE VLC_SADOutputGeneric_t* VLCVeh_p_GetHeadOutGeneric(void);
ALGO_INLINE VLC_SADInputGeneric_t const* VLCVeh_p_GetHeadInGeneric(void);
ALGO_INLINE VLC_SADInputCustom_t const* VLCVeh_p_GetHeadInCustom(void);

ALGO_INLINE VLC_DIMOutputCustom_t* VLCVeh_p_GetDIMOutCustom(void);
ALGO_INLINE VLC_DIMInputCustom_t const* VLCVeh_p_GetDIMInCustom(void);
ALGO_INLINE VLC_DIMInputGeneric_t const* VLCVeh_p_GetDIMInGeneric(void);

/*****************************************************************************
  EXTERN VARIABLES
*****************************************************************************/

/*****************************************************************************
  INLINE FUNCTIONS
*****************************************************************************/

/**************/

/************************************************************************/
/* VDY signals                                                          */
/************************************************************************/

/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetVDYDynRaw */
ALGO_INLINE VED_VehDyn_t const* VLCVeh_p_GetVDYDynRaw(void) {
    return VLCVEH_pEgoDynRaw;
}

/*************************************************************************************************************************
  Functionname:    VLCVeh_f_GetEgoVelRaw */
ALGO_INLINE float32 VLCVeh_f_GetEgoVelRaw(void) {
    return VLCVeh_p_GetVDYDynRaw()->Longitudinal.MotVar.Velocity;
}

/*************************************************************************************************************************
  Functionname:    VLCVeh_f_GetEgoAccelRaw */
ALGO_INLINE float32 VLCVeh_f_GetEgoAccelRaw(void) {
    return VLCVeh_p_GetVDYDynRaw()->Longitudinal.MotVar.Accel;
}

/************************************************************************/
/* Vehicle parameter                                                    */
/************************************************************************/
/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetVehPar */
ALGO_INLINE VED_VehPar_t const* VLCVeh_p_GetVehPar(void) {
    return VLCVEH_pGlobEgoStatic;
}

/************************************************************************/
/* Misc                                                                 */
/************************************************************************/
/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetCycleTime */
ALGO_INLINE float32 VLCVeh_f_GetCycleTime(void) {
    /* Todo: Use calculated cycle time instead */
    return (VLC_VEH_CYCLE_TIME);
}
/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetVLCVehFrame */
ALGO_INLINE VLCVehFrame_t* VLCVeh_p_GetVLCVehFrame(void) {
    return &VLCVehFrame;
}

/************************************************************************/
/* Hypothesis                                                           */
/************************************************************************/
/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetHypothesisIntf */
ALGO_INLINE VLC_HypothesisIntf_t const* VLCVeh_p_GetHypothesisIntf(void) {
    return VLC_pCDHypothesesVeh;
}

/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetHypothesis */
ALGO_INLINE Hypothesis_t const* VLCVeh_p_GetHypothesis(sint32 iHyp) {
    /* Check range */
    /* Suppress "Msg(3:3112) This statement has no side-effect - it can be
     * removed"  */
    VLC_ASSERT((0 <= iHyp) && (iHyp < MAX_NUM_OF_HYPS));

    return &(VLCVeh_p_GetHypothesisIntf()->Hypothesis[iHyp]);
}

/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetHypDegradation */
ALGO_INLINE HypoIntfDegr_t const* VLCVeh_p_GetHypDegradation(void) {
    return &(VLCVeh_p_GetHypothesisIntf()->Degradation);
}

/************************************************************************/
/* Parameter                                                            */
/************************************************************************/
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetAlgoParameters */
ALGO_INLINE Com_AlgoParameters_t const* VLCSen_p_GetAlgoParameters(void) {
    return VLCVEH_pBswAlgoParameters;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetCPARVLCParameters */
ALGO_INLINE VLC_Parameters_t const* VLCSen_p_GetCPARVLCParameters(void) {
    return VLCVEH_pCPAR_VLC_Parameters;
}

/************************************************************************/
/* HEAD                                                                 */
/************************************************************************/

/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetHeadOutCustom */
ALGO_INLINE VLC_SADOutputCustom_t* VLCVeh_p_GetHeadOutCustom(void) {
    return VLC_pHEADCustDataOut;
}

/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetHeadOutGeneric */
ALGO_INLINE VLC_SADOutputGeneric_t* VLCVeh_p_GetHeadOutGeneric(void) {
    return VLC_pHEADGenericDataOut;
}

/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetHeadInGeneric */
ALGO_INLINE VLC_SADInputGeneric_t const* VLCVeh_p_GetHeadInGeneric(void) {
    return VLC_pHEADGenericDataIn;
}

/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetHeadInCustom */
ALGO_INLINE VLC_SADInputCustom_t const* VLCVeh_p_GetHeadInCustom(void) {
    return VLC_pHEADCustDataIn;
}

/************************************************************************/
/* DIM                                                                  */
/************************************************************************/

/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetDIMOutCustom */
ALGO_INLINE VLC_DIMOutputCustom_t* VLCVeh_p_GetDIMOutCustom(void) {
    return VLC_pDIMCustDataOut;
}
/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetDIMInCustom */
ALGO_INLINE VLC_DIMInputCustom_t const* VLCVeh_p_GetDIMInCustom(void) {
    return VLC_pDIMCustDataIn;
}
/*************************************************************************************************************************
  Functionname:    VLCVeh_p_GetDIMInGeneric */
ALGO_INLINE VLC_DIMInputGeneric_t const* VLCVeh_p_GetDIMInGeneric(void) {
    return VLC_pDIMGenericDataIn;
}

#endif
