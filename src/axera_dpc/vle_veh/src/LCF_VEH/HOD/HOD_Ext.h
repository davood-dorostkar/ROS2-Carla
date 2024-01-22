// Copyright [2021] <Copyright Owner Senseauto>
#ifndef DECISION_SRC_LCF_VEH_HOD_HOD_EXT_H_
#define DECISION_SRC_LCF_VEH_HOD_HOD_EXT_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "tue_common_libs.h"
#include "rtwtypes.h"

typedef struct {
    REAL32_T HOD_CycleTime_Sec; /* HOD cycle time */
    REAL32_T HOD_TrqActMan;     /* HOD Act hand torque input */
    REAL32_T HOD_VelXVeh;       /* Vehicle Speed */
    boolean HOD_EnaActTorque;   /* ActTorque Enable flag */
    UINT8_T HOD_BtfControlFct;  /* Control function Enable byte equel to 1 means enable */
    UINT8_T HOD_StActFctLevel;  /* Level of system warning  */
    UINT8_T HOD_BtfAvailableFct; /* Function available byte   equel to 2 means enable*/
} sHODInput_t;

typedef struct {
    REAL32_T Sys_VehWid_Mi; /* Not use in function, to make the parameter structure exist */
} sHODParam_t;

typedef struct {
    UINT8_T HOD_StEstHandTrq;      /* State of hand torque */
                                   /* 0 state HandsOnLeft */
                                   /* 1 state HandsOnRight */
                                   /* 2 SuspHandsOn  */
                                   /* 3 SuspHandsOffLeft */
                                   /* 4 SuspHandsOffRight */
                                   /* 5 HandsOff */
    UINT8_T HOD_StSysWarning;      /* Level of hands off system warning */
                                   /* 0 NoWarning */
                                   /* 1 HandsOnRequest */
                                   /* 2 TakeOverRequest */
                                   /* 3 TriggerDegradation */
    REAL32_T HOD_RatDrvAttention;  /* detected driver attention level */
    boolean HOD_HandsOffAbuseWarn; /* flag of hands off abuse warning */
    boolean HOD_EnaHandsOffCnfm;   /* flag of hands off or on */
} sHODOutput_t;

#ifndef Rte_TypeDef_sHODDebug_t
typedef struct {
    REAL32_T HOD_TrqEstHandFlt; /* hand torque after the low pass filter */
    REAL32_T HOD_TrqEstHandStp; /* hand torque change rate after the low pass filter */
    REAL32_T HOD_TrqHandsOffLim;  /* the limit value of hand torque for the hands off enable flag */
    boolean HOD_EnaRightOn;  /* right handson enable flag */
    boolean HOD_EnaRightInt; /* right hands interval enable flag */
    boolean HOD_EnaRightOff; /* right handsoff enable flag */
    boolean HOD_EnaLeftOn;   /* left handson enable flag */
    boolean HOD_EnaLeftInt;  /* left hands interval enable flag */
    boolean HOD_EnaLeftOff;  /* left handsoffenable flag */
    REAL32_T HOD_CntEstHandTrq;  /* counter for the suspicious state  */
    REAL32_T HOD_CntHandsOffLim; /* the time limite for the state from suspicious off to hansoff */
} sHODDebug_t;
#define Rte_TypeDef_sHODDebug_t
#endif

extern void LCF_HOD_Reset(void);
extern void LCF_HOD_Exec(const sHODInput_t* pHODInput,
                         const sHODParam_t* pHODParam,
                         sHODOutput_t* pHODOutput,
                         sHODDebug_t* pHODDebug);
#ifdef __cplusplus
}
#endif
#endif  // DECISION_SRC_LCF_VEH_HOD_HOD_EXT_H_
