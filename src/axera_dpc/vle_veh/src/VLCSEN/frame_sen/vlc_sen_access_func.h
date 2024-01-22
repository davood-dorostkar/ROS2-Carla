

/*****************************************************************************
  MACROS
*****************************************************************************/

#ifndef VLC_SEN_ACCESS_H_INCLUDED
#define VLC_SEN_ACCESS_H_INCLUDED

/*****************************************************************************
  EXTERN VARIABLES
*****************************************************************************/

/*****************************************************************************
  INLINE FUNCTIONS
*****************************************************************************/

#ifdef ALGO_INLINE
#undef ALGO_INLINE
#endif
#define ALGO_INLINE static inline

/**************/
ALGO_INLINE AS_t_ServiceFuncts const* VLCSen_p_GetVLCServicePointer(void);
ALGO_INLINE Envm_t_GenObjectList const* VLCSen_p_GetEMGenObjList(void);
ALGO_INLINE Envm_t_GenObject const* VLCSen_p_GetEMGenObj(ObjNumber_t iObj);
ALGO_INLINE Envm_t_GenObjKinEnvmatics const* VLCSen_p_GetEMGenObjKinematics(
    ObjNumber_t iObj);
ALGO_INLINE Envm_t_GenObjQualifiers const* VLCSen_p_GetEMGenObjQualifiers(
    ObjNumber_t iObj);
ALGO_INLINE Envm_t_GenObjAttributes const* VLCSen_p_GetEMGenObjAttributes(
    ObjNumber_t iObj);
ALGO_INLINE EM_t_GenObjGenerals const* VLCSen_p_GetEMGenObjGeneral(
    ObjNumber_t iObj);
ALGO_INLINE EM_t_GenObjGeometry const* VLCSen_p_GetEMGenObjGeometry(
    ObjNumber_t iObj);
ALGO_INLINE Envm_t_CRObjectList const* VLCSen_p_GetEMARSObjList(void);
ALGO_INLINE EM_t_ARSObject const* VLCSen_p_GetEMARSObj(ObjNumber_t iObj);
ALGO_INLINE Envm_t_CR_Attributes const* VLCSen_p_GetEMARSObjAttributes(
    ObjNumber_t iObj);
ALGO_INLINE Envm_t_CR_Geometry const* VLCSen_p_GetEMARSObjGeometry(
    ObjNumber_t iObj);
ALGO_INLINE Envm_t_CR_KinEnvmatic const* VLCSen_p_GetEMARSObjKinematic(
    ObjNumber_t iObj);
ALGO_INLINE Envm_t_CR_Legacy const* VLCSen_p_GetEMARSObjLegacy(
    ObjNumber_t iObj);
ALGO_INLINE Envm_t_CR_MotionAttributes const*
VLCSen_p_GetEMARSObjMotionAttributes(ObjNumber_t iObj);
ALGO_INLINE Envm_t_CR_SensorSpecific const* VLCSen_p_GetEMARSObjSensorSpecific(
    ObjNumber_t iObj);
ALGO_INLINE VLCObject_t* VLCSen_p_GetVLCPrivObj(ObjNumber_t iObj);
ALGO_INLINE VLCCoursePred_t* VLCSen_p_GetVLCPrivObj_CP(ObjNumber_t iObj);
ALGO_INLINE SI_t* VLCSen_p_GetVLCPrivObj_SI(ObjNumber_t iObj);
ALGO_INLINE VLCCustomObjectProperties_t* VLCSen_p_GetVLCPrivObj_CustObjProps(
    ObjNumber_t iObj);
ALGO_INLINE AssessedObjList_t* VLCSen_p_GetVLCPubObjList(void);
ALGO_INLINE HeaderAssessedObjList_t* VLCSen_p_GetVLCPubObjListHeader(void);
ALGO_INLINE VLCPubObject_t* VLCSen_p_GetVLCPubObj(ObjNumber_t iObj);
ALGO_INLINE LaneInformation_t* VLCSen_p_GetVLCPubObj_LaneInfo(ObjNumber_t iObj);
ALGO_INLINE LegacyAOL_t* VLCSen_p_GetVLCPubObj_LegacyAOL(ObjNumber_t iObj);
ALGO_INLINE ObjOfInterest_t* VLCSen_p_GetVLCPubObj_OOI(ObjNumber_t iObj);
ALGO_INLINE boolean VLCSen_b_IsObjOOI(ObjNumber_t iObj, eObjOOI_t eObjOOI);
ALGO_INLINE boolean VLCSen_b_IsObjRelevant(ObjNumber_t iObj);
ALGO_INLINE ObjNumber_t VLCSen_i_GetOOIObjIndex(eObjOOI_t eObjOOI);
ALGO_INLINE ObjNumber_t VLCSen_i_GetRelObjIndex(void);
ALGO_INLINE VED_VehDyn_t const* VLCSen_p_GetVDYDynObjSync(void);
ALGO_INLINE float32 VLCSen_f_GetEgoVelObjSync(void);
ALGO_INLINE float32 VLCSen_f_GetEgoAccelObjSync(void);
ALGO_INLINE VED_VehDyn_t const* VLCSen_p_GetVDYDynRaw(void);
ALGO_INLINE float32 VLCSen_f_GetEgoVelRaw(void);
ALGO_INLINE float32 VLCSen_f_GetEgoAccelRaw(void);
ALGO_INLINE VED_VehPar_t const* VLCSen_p_GetVehPar(void);
ALGO_INLINE SensorMounting_t const* VLCSen_p_GetSensorMounting(void);
ALGO_INLINE ECAMtCyclEnvmode_t const* VLCSen_p_GetEmFctCycleMode(void);
ALGO_INLINE float32 VLCSen_f_GetCycleTime(void);

ALGO_INLINE float32 VLCSen_f_GetSICycleTime(void);
ALGO_INLINE float32 VLCSen_f_GetSPMCycleTime(void);
ALGO_INLINE float32 VLCSen_f_GetCPCycleTime(void);

ALGO_INLINE VLCCustomInput_t const* VLCSen_p_GetCustomInput(void);
ALGO_INLINE VLCCustomOutput_t* VLCSen_p_GetCustomOutput(void);
ALGO_INLINE VLCCDOutputCustom_t* VLCSen_p_GetCDCustomOutput(void);
ALGO_INLINE VLC_HypothesisIntf_t* VLCSen_p_GetHypothesisIntf(void);
ALGO_INLINE Hypothesis_t* VLCSen_p_GetHypothesis(sint32 iHyp);
ALGO_INLINE HypoIntfDegr_t* VLCSen_p_GetHypDegradation(void);
ALGO_INLINE Com_AlgoParameters_t const* VLCSen_p_GetAlgoParameters(void);
ALGO_INLINE VLCSenFrame_t* VLCSen_p_GetVLCSenFrame(void);
ALGO_INLINE VLCSen_SyncRef_t* VLCSen_p_GetVLCSenSyncRef(void);

/************************************************************************/
/* VLC Service Pointers                                                  */
/************************************************************************/

/************************************************************************/
/* EM Gen Object list                                                   */
/************************************************************************/
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMGenObjList */
ALGO_INLINE Envm_t_GenObjectList const* VLCSen_p_GetEMGenObjList(void) {
    return VLCSEN_pEmGenObjList;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMGenObj */
ALGO_INLINE Envm_t_GenObject const* VLCSen_p_GetEMGenObj(ObjNumber_t iObj) {
    /* Perform range check before accessing an object */
    /* Suppress "Msg(3:3112) This statement has no side-effect - it can be
     * removed"  */
    VLC_ASSERT((iObj >= 0) && (iObj < Envm_N_OBJECTS));

    return &(VLCSen_p_GetEMGenObjList()->aObject[iObj]);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMGenObjKinematics */
ALGO_INLINE Envm_t_GenObjKinEnvmatics const* VLCSen_p_GetEMGenObjKinematics(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetEMGenObj(iObj)->Kinematic);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMGenObjQualifiers */
ALGO_INLINE Envm_t_GenObjQualifiers const* VLCSen_p_GetEMGenObjQualifiers(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetEMGenObj(iObj)->Qualifiers);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMGenObjAttributes */
ALGO_INLINE Envm_t_GenObjAttributes const* VLCSen_p_GetEMGenObjAttributes(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetEMGenObj(iObj)->Attributes);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMGenObjGeneral */
ALGO_INLINE EM_t_GenObjGenerals const* VLCSen_p_GetEMGenObjGeneral(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetEMGenObj(iObj)->General);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMGenObjGeometry */
ALGO_INLINE EM_t_GenObjGeometry const* VLCSen_p_GetEMGenObjGeometry(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetEMGenObj(iObj)->Geometry);
}

/************************************************************************/
/* EM ARS Object list                                                   */
/************************************************************************/
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMARSObjList */
ALGO_INLINE Envm_t_CRObjectList const* VLCSen_p_GetEMARSObjList(void) {
    return VLCSEN_pEmARSObjList;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMARSObj */
ALGO_INLINE EM_t_ARSObject const* VLCSen_p_GetEMARSObj(ObjNumber_t iObj) {
    /* Perform range check before accessing an object */
    /* Suppress "Msg(3:3112) This statement has no side-effect - it can be
     * removed"  */
    VLC_ASSERT((iObj >= 0) && (iObj < Envm_N_OBJECTS));

    return &(VLCSen_p_GetEMARSObjList()->aObject[iObj]);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMARSObjAttributes */
ALGO_INLINE Envm_t_CR_Attributes const* VLCSen_p_GetEMARSObjAttributes(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetEMARSObj(iObj)->Attributes);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMARSObjGeometry */
ALGO_INLINE Envm_t_CR_Geometry const* VLCSen_p_GetEMARSObjGeometry(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetEMARSObj(iObj)->Geometry);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMARSObjKinematic */
ALGO_INLINE Envm_t_CR_KinEnvmatic const* VLCSen_p_GetEMARSObjKinematic(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetEMARSObj(iObj)->Kinematic);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMARSObjLegacy */
ALGO_INLINE Envm_t_CR_Legacy const* VLCSen_p_GetEMARSObjLegacy(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetEMARSObj(iObj)->Legacy);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMARSObjMotionAttributes */
ALGO_INLINE Envm_t_CR_MotionAttributes const*
VLCSen_p_GetEMARSObjMotionAttributes(ObjNumber_t iObj) {
    return &(VLCSen_p_GetEMARSObj(iObj)->MotionAttributes);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEMARSObjSensorSpecific */
ALGO_INLINE Envm_t_CR_SensorSpecific const* VLCSen_p_GetEMARSObjSensorSpecific(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetEMARSObj(iObj)->SensorSpecific);
}

/************************************************************************/
/* Private object list                                                  */
/************************************************************************/
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCPrivObj */
ALGO_INLINE VLCObject_t* VLCSen_p_GetVLCPrivObj(ObjNumber_t iObj) {
    /* Perform range check before accessing an object */
    /* Suppress "Msg(3:3112) This statement has no side-effect - it can be
     * removed"  */
    VLC_ASSERT((iObj >= 0) && (iObj < Envm_N_OBJECTS));

    return &(VLCObjectList[iObj]);
}
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCPrivObj_CP */
ALGO_INLINE VLCCoursePred_t* VLCSen_p_GetVLCPrivObj_CP(ObjNumber_t iObj) {
    return &(VLCSen_p_GetVLCPrivObj(iObj)->CP);
}
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCPrivObj_SI */
ALGO_INLINE SI_t* VLCSen_p_GetVLCPrivObj_SI(ObjNumber_t iObj) {
    return &(VLCSen_p_GetVLCPrivObj(iObj)->SI);
}
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCPrivObj_CustObjProps */
ALGO_INLINE VLCCustomObjectProperties_t* VLCSen_p_GetVLCPrivObj_CustObjProps(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetVLCPrivObj(iObj)->VLCCustomObjectProperties);
}
/************************************************************************/
/* Public object list                                                   */
/************************************************************************/
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCPubObjList */
ALGO_INLINE AssessedObjList_t* VLCSen_p_GetVLCPubObjList(void) {
    return VLCSEN_pPubFctObjList;
}
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCPubObjListHeader */
ALGO_INLINE HeaderAssessedObjList_t* VLCSen_p_GetVLCPubObjListHeader(void) {
    return &(VLCSen_p_GetVLCPubObjList()->HeaderAssessedObjList);
}
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCPubObj */
ALGO_INLINE VLCPubObject_t* VLCSen_p_GetVLCPubObj(ObjNumber_t iObj) {
    /* Perform range check before accessing an object */
    /* Suppress "Msg(3:3112) This statement has no side-effect - it can be
     * removed"  */
    VLC_ASSERT((iObj >= 0) && (iObj < Envm_N_OBJECTS));

    return &(VLCSen_p_GetVLCPubObjList()->ObjList[iObj]);
}
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCPubObj_LaneInfo */
ALGO_INLINE LaneInformation_t* VLCSen_p_GetVLCPubObj_LaneInfo(
    ObjNumber_t iObj) {
    return &(VLCSen_p_GetVLCPubObj(iObj)->LaneInformation);
}
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCPubObj_LegacyAOL */
ALGO_INLINE LegacyAOL_t* VLCSen_p_GetVLCPubObj_LegacyAOL(ObjNumber_t iObj) {
    return &(VLCSen_p_GetVLCPubObj(iObj)->Legacy);
}
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCPubObj_OOI */
ALGO_INLINE ObjOfInterest_t* VLCSen_p_GetVLCPubObj_OOI(ObjNumber_t iObj) {
    return &(VLCSen_p_GetVLCPubObj(iObj)->ObjOfInterest);
}
/*************************************************************************************************************************
  Functionname:    VLCSen_b_IsObjOOI */
ALGO_INLINE boolean VLCSen_b_IsObjOOI(ObjNumber_t iObj, eObjOOI_t eObjOOI) {
    return (VLCSen_p_GetVLCPubObj_OOI(iObj)->eObjOOI == eObjOOI) ? TRUE : FALSE;
}
/*************************************************************************************************************************
  Functionname:    VLCSen_b_IsObjRelevant */
ALGO_INLINE boolean VLCSen_b_IsObjRelevant(ObjNumber_t iObj) {
    return VLCSen_b_IsObjOOI(iObj, OBJ_NEXT_OOI);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_i_GetOOIObjIndex */
ALGO_INLINE ObjNumber_t VLCSen_i_GetOOIObjIndex(eObjOOI_t eObjOOI) {
    /* Suppress "Msg(3:3112) This statement has no side-effect - it can be
     * removed"  */
    VLC_ASSERT((eObjOOI >= 0) && (eObjOOI < 6));
    return VLCSen_p_GetVLCPubObjListHeader()->aiOOIList[eObjOOI];
}
/*************************************************************************************************************************
  Functionname:    VLCSen_i_GetRelObjIndex */
ALGO_INLINE ObjNumber_t VLCSen_i_GetRelObjIndex(void) {
    return VLCSen_i_GetOOIObjIndex(OBJ_NEXT_OOI);
}
/************************************************************************/
/* VDY signals                                                          */
/************************************************************************/

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVDYDynObjSync */
ALGO_INLINE VED_VehDyn_t const* VLCSen_p_GetVDYDynObjSync(void) {
    return VLCSEN_pEgoDynObjSync;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_f_GetEgoVelObjSync */
ALGO_INLINE float32 VLCSen_f_GetEgoVelObjSync(void) {
    return VLCSen_p_GetVDYDynObjSync()->Longitudinal.MotVar.Velocity;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_f_GetEgoAccelObjSync */
ALGO_INLINE float32 VLCSen_f_GetEgoAccelObjSync(void) {
    return VLCSen_p_GetVDYDynObjSync()->Longitudinal.MotVar.Accel;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVDYDynRaw */
ALGO_INLINE VED_VehDyn_t const* VLCSen_p_GetVDYDynRaw(void) {
    return VLCSEN_pEgoDynRaw;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_f_GetEgoVelRaw */
ALGO_INLINE float32 VLCSen_f_GetEgoVelRaw(void) {
    return VLCSen_p_GetVDYDynRaw()->Longitudinal.MotVar.Velocity;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_f_GetEgoAccelRaw */
ALGO_INLINE float32 VLCSen_f_GetEgoAccelRaw(void) {
    return VLCSen_p_GetVDYDynRaw()->Longitudinal.MotVar.Accel;
}

/************************************************************************/
/* Vehicle parameter                                                    */
/************************************************************************/

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVehPar */
ALGO_INLINE VED_VehPar_t const* VLCSen_p_GetVehPar(void) {
    return VLCSEN_pGlobEgoStatic;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetSensorMounting */
ALGO_INLINE SensorMounting_t const* VLCSen_p_GetSensorMounting(void) {
    return &(VLCSen_p_GetVehPar()->SensorMounting);
}

/************************************************************************/
/* Misc                                                                 */
/************************************************************************/
/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetEmFctCycleMode */
ALGO_INLINE ECAMtCyclEnvmode_t const* VLCSen_p_GetEmFctCycleMode(void) {
    return VLCSEN_pECAMtCyclEnvmode;
}
/*************************************************************************************************************************
  Functionname:    VLCSen_f_GetCycleTime */
ALGO_INLINE float32 VLCSen_f_GetCycleTime(void) {
    return VLCSen_p_GetEmFctCycleMode()->fECAMtCycleTime;
}
/*************************************************************************************************************************
  Functionname:    VLCSen_f_GetSICycleTime */
ALGO_INLINE float32 VLCSen_f_GetSICycleTime(void) {
    return VLCSen_f_GetCycleTime();
}
/*************************************************************************************************************************
  Functionname:    VLCSen_f_GetSPMCycleTime */
ALGO_INLINE float32 VLCSen_f_GetSPMCycleTime(void) {
    return VLCSen_f_GetCycleTime();
}
/*************************************************************************************************************************
  Functionname:    VLCSen_f_GetCPCycleTime */
ALGO_INLINE float32 VLCSen_f_GetCPCycleTime(void) {
    return VLCSen_f_GetCycleTime();
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetCustomInput */
ALGO_INLINE VLCCustomInput_t const* VLCSen_p_GetCustomInput(void) {
    return VLCSEN_pCustomInput;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetCustomOutput */
ALGO_INLINE VLCCustomOutput_t* VLCSen_p_GetCustomOutput(void) {
    return VLCSEN_pCustomOutput;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetCDCustomOutput */
ALGO_INLINE VLCCDOutputCustom_t* VLCSen_p_GetCDCustomOutput(void) {
    return VLCSEN_pCDCustomOutput;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetHypothesisIntf */
ALGO_INLINE VLC_HypothesisIntf_t* VLCSen_p_GetHypothesisIntf(void) {
    return VLC_pCDHypothesesSen;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetHypothesis */
ALGO_INLINE Hypothesis_t* VLCSen_p_GetHypothesis(sint32 iHyp) {
    /* Check range */
    /* Suppress "Msg(3:3112) This statement has no side-effect - it can be
     * removed"  */
    VLC_ASSERT((0 <= iHyp) && (iHyp < MAX_NUM_OF_HYPS));

    return &(VLC_pCDHypothesesSen->Hypothesis[iHyp]);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetHypDegradation */
ALGO_INLINE HypoIntfDegr_t* VLCSen_p_GetHypDegradation(void) {
    return &(VLCSen_p_GetHypothesisIntf()->Degradation);
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetAlgoParameters */
ALGO_INLINE Com_AlgoParameters_t const* VLCSen_p_GetAlgoParameters(void) {
    return VLCSEN_pBswAlgoParameters;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCSenFrame */
ALGO_INLINE VLCSenFrame_t* VLCSen_p_GetVLCSenFrame(void) {
    return &VLCSenFrame;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_p_GetVLCSenSyncRef */
ALGO_INLINE VLCSen_SyncRef_t* VLCSen_p_GetVLCSenSyncRef(void) {
    return &VLCSenSyncRef;
}

#endif
