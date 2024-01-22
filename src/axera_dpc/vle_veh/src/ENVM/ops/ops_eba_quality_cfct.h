
#ifndef EM_OPS_EBA_QUALITY_CFCT_H
#define EM_OPS_EBA_QUALITY_CFCT_H

typedef struct {
    bool_t
        bCameraConfirmed : 1; /*!< boolean flag if confirmed in current cycle */
    bool_t
        bCamConfHold : 1; /*!< Bit set if confirmed by camera at least once */
    bool_t bCamConfAsPed : 1;  /*!< Bit set if confirmed by camera as ped */
    uint8 uCamConfCounter : 5; /*!< Counter for number of confirmation cycles */
} EMFPSEBACustFctObj_t;

void FPS_v_InitCustomQuality(void);
void FPSEBAPreSelCustomFct(sint8 ObjNr);

#endif /*EM_OPS_EBA_QUALITY_CFCT_H*/
