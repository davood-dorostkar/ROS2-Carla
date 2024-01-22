/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "si.h"
#include "si_par.h"

/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/
#define ROVG_VEIGENMIN (10.F / C_KMH_MS)

#define ROVG_BREMSUNG (6.F)

#define ROVG_AUFGELOEST_VEIGENMIN (30.F / C_KMH_MS)
#define ROVG_GRENZKRUEMMUNG_AUS (0.5F / 1500.F)
#define ROVG_GRENZKRUEMMUNG_AUS_AF (0.5F / 1000.F)
#define ROVG_GRENZWINKEL (LOBE_ANGLE * 0.75F)
#define ROVG_ABSTAND_MAX (80.F)

#define ROVG_GRENZE_BX (25.F)

#define ROVG_GRENZWINKEL_BX_NEU ROVG_GRENZWINKEL /*(ERFASSUNGSWINKEL *0.75f)*/

#define SI_ROV_SET_MAX_TIME_DIST (2.5f)
#define SI_ROV_RESET_MAX_TIME_DIST (4.0f)
#define SI_ROV_MAX_DIST (100.0f)

#define SI_FAST_CURVE_SCALE (-0.5f)

#define KRUEMMUNGGRENZ (0.5F / 40.F)
/* Geschwindigkeitsschwelle fuer RCS-Ueberpruefung */
#define TRACK_ABSCHALT_VEIGEN_MIN (30.F / C_KMH_MS)

#define ROVG_FLAG_ABSTAND_MAX (60.F)
#define ROVG_FLAG_GRENZGRADIENT_AF (0.0005F)
/* TB A ARS200 SCR 159  20-12-2004 */
#define ROVG_ABSTAND_MIN \
    (10.F) /* Mindestabstand fuer die Ausgabe des OBJ_LOSS_DISAPPEARED */
           /* TB A ARS200 SCR 159  20-12-2004 */

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/
typedef struct {
    ObjNumber_t RelTrckObjNr;
    const SIRelObject_t *pRelObj;
    float32 TPVehicleSpeed;
    fCurve_t TrackStandardCurve;
    float32 TrackStandardGradient;
    fCurve_t TrackFastCurve;
    uint8 OOINr;
} ROVDataInput_t;

typedef enum SIFastTrackSwitchOff {
    NO_SWITCH_OFF,
    LEFT_CURVE_SWITCH_OFF,
    RIGHT_CURVE_SWITCH_OFF
} SIFastTrackSwitchOff_t;

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(eRelObjVerlustGrund)
static eRelObjLossReason_t eRelObjVerlustGrund[SiAnzOOI];

SET_MEMSEC_VAR(dROVGHalteDistanz)
static float32 dROVGHalteDistanz[SiAnzOOI];

SET_MEMSEC_VAR(dROVGHalteVObjekt)
static fVelocity_t dROVGHalteVObjekt[SiAnzOOI];

SET_MEMSEC_VAR(dKruemmungWaehrendROVMin)
static float32 dKruemmungWaehrendROVMin[SiAnzOOI];

SET_MEMSEC_VAR(RelObjDisappearedFlag)
static boolean RelObjDisappearedFlag[SiAnzOOI];
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static void SIReObRelObVerlustGrundInit(void);
static SIFastTrackSwitchOff_t SISeReObFastTrackAbschaltung(void);
static void SIObReObRelObjVerlRueckTimeDist(const ROVDataInput_t *ROVDataInput);
static void SIObReObRelObjVerlustSetzenTimeDist(
    const ROVDataInput_t *ROVDataInput);

/*************************************************************************************************************************
  Functionname:    SIReObRelObVerlustGrundInit */
static void SIReObRelObVerlustGrundInit(void) {
    ObjNumber_t i;
    for (i = 0; i < SiAnzOOI; i++) {
        eRelObjVerlustGrund[i] = OBJ_LOSS_NO_INFO;

        dKruemmungWaehrendROVMin[i] = 0.0f;
        dROVGHalteDistanz[i] = 0.0f;
        dROVGHalteVObjekt[i] = 0.0f;
    }
}

/*************************************************************************************************************************
  Functionname:    SIObReObInit */
void SIObReObInit(void) { SIReObRelObVerlustGrundInit(); }

/*************************************************************************************************************************
  Functionname:    SISeReObFastTrackAbschaltung */
static SIFastTrackSwitchOff_t SISeReObFastTrackAbschaltung(void) {
    SIFastTrackSwitchOff_t FastTrackSwitchOff;
    fCurve_t fCurveFast;

    /*! Set default to "No switch off" */
    FastTrackSwitchOff = NO_SWITCH_OFF;
    /* Holen der Schnellen-Spur */
    fCurveFast = SI_FAST_CURVE_SCALE * EGO_CURVE_OBJ_SYNC;

    if (EGO_SPEED_X_OBJ_SYNC > (TRACK_ABSCHALT_VEIGEN_MIN - C_F32_DELTA)) {
        if (fCurveFast > KRUEMMUNGGRENZ) {
            FastTrackSwitchOff = RIGHT_CURVE_SWITCH_OFF;
        } else if (fCurveFast < -KRUEMMUNGGRENZ) {
            FastTrackSwitchOff = LEFT_CURVE_SWITCH_OFF;
        } else {
            FastTrackSwitchOff = NO_SWITCH_OFF;
        }
    }

    return FastTrackSwitchOff;
}

/*************************************************************************************************************************
  Functionname:    SIObReObRelObjLossReason */
void SIObReObRelObjLossReason(const ObjNumber_t NewObjId,
                              const SIRelObject_t *pOldObject,
                              SIRelObjEnum_t SiOOINr) {
    /*--- VARIABLES ---*/
    fCurve_t fCurveFast;
    ROVDataInput_t ROVDataInput;

    fCurveFast = SI_FAST_CURVE_SCALE * EGO_CURVE_OBJ_SYNC;

    ROVDataInput.RelTrckObjNr = NewObjId;
    ROVDataInput.pRelObj =
        pOldObject; /*!< Remark: pOldObject points to the global variable
                       SIRelObject, which is properly initialized -> variables
                       of the structure can be always accessed */
    ROVDataInput.TPVehicleSpeed = EGO_SPEED_X_OBJ_SYNC;
    ROVDataInput.TrackStandardCurve = EGO_CURVE_OBJ_SYNC;
    ROVDataInput.TrackStandardGradient = EGO_CURVE_GRAD_OBJ_SYNC;
    ROVDataInput.TrackFastCurve = fCurveFast;
    ROVDataInput.OOINr = (uint8)SiOOINr;

    if ((NewObjId == OBJ_INDEX_NO_OBJECT) && (pOldObject->ObjValid == TRUE) &&
        (pOldObject->StatObj == FALSE) &&
        (EGO_SPEED_X_OBJ_SYNC > ROVG_VEIGENMIN)) {
        /* Setzen eines detektierten ROVG */
        SIObReObRelObjVerlustSetzenTimeDist(&ROVDataInput);
    } else {
        /* Loeschen eines detektierten ROVG */
        SIObReObRelObjVerlRueckTimeDist(&ROVDataInput);
    }

#if VLC_LONG_SEN_DEBUG == 1
    ROVDataInput_pRelObj.LatDisplRoadBordL =
        ROVDataInput.pRelObj->LatDisplRoadBordL;
    ROVDataInput_pRelObj.LatDisplRoadBordR =
        ROVDataInput.pRelObj->LatDisplRoadBordR;
    ROVDataInput_pRelObj.ObjDistY = ROVDataInput.pRelObj->ObjDistY;
#endif
}

/*************************************************************************************************************************
  Functionname:    SIObReObGetRelObjDisappearedFlag */
void SIObReObGetRelObjDisappearedFlag(boolean *bRelObjDisappearedFlag) {
    *bRelObjDisappearedFlag = RelObjDisappearedFlag[OBJ_NEXT_OOI];
}

/*************************************************************************************************************************
  Functionname:    SIObOOIGetOOILossReason */
eRelObjLossReason_t SIObOOIGetOOILossReason(SIRelObjEnum_t SiOOINr) {
    return eRelObjVerlustGrund[SiOOINr];
}

/*************************************************************************************************************************
  Functionname:    SIObReObRelObjVerlustSetzenTimeDist */
static void SIObReObRelObjVerlustSetzenTimeDist(
    const ROVDataInput_t *ROVDataInput) {
    float32 dGrenzwinkel = ROVG_GRENZWINKEL;

    float32 TimeDist;

    RelObjDisappearedFlag[ROVDataInput->OOINr] = FALSE;

    /* Anderer Grenzwinkel fuer nahe Verfolgte: */
    /*!< Remark: pRelObj points to the global variable SIRelObject, which is
     * properly initialized -> variables of the structure can be always accessed
     */
    if (ROVDataInput->pRelObj->DistX < ROVG_GRENZE_BX) {
        dGrenzwinkel = ROVG_GRENZWINKEL_BX_NEU;
    }

    if (fABS(ROVDataInput->TPVehicleSpeed) > C_F32_DELTA) {
        TimeDist = ROVDataInput->pRelObj->DistX / ROVDataInput->TPVehicleSpeed;
    } else {
        TimeDist = SI_ROV_SET_MAX_TIME_DIST;
    }

    /*  13.10.07 Definition ZVG-Kurve ab R3.3.1 (Kolb+Reichmann):

    Ausl�sen:
    Objekte_Y_Ablage ausserhalb sicherem FBR Erfassungsbe. ca. +/-6.0?
    UND Objekt_X_Abst < 100m
    UND Objekt_X_Zeitabstand < 2,5 Sek.
    UND Objektverlust aus Spur in Richtung kl. Radius (Kurveninnenseitig)

    Aufheben im gleichen Systemzyklus (�berschreibt Ausl�sen-Kritierien):
    Sichtweite im sicheren FBR Erfassungsbe. > 100m
    ODER Sichtweite im sicheren FBR Erfassungsbereich > 4.0 Sek Zeitab.*/

    if ((TimeDist < SI_ROV_SET_MAX_TIME_DIST) &&
        (ROVDataInput->pRelObj->DistX <= SI_ROV_MAX_DIST)) {
        /*Linkskurve und Objekt war links !!!!*/
        if ((ROVDataInput->TrackStandardCurve > 0.f) &&
            (ROVDataInput->pRelObj->ObjAngle > dGrenzwinkel)) {
            /* Objekt noch da, aber zum Kurvenaussenrand verschwunden ? */
            if ((SIBasePreselObjList[ROVDataInput->pRelObj->ObjectNr] ==
                 FALSE) ||
                (OBJ_IS_NEW(ROVDataInput->pRelObj->ObjectNr)) ||
                (OBJ_GET_ASSOCIATED_LANE(ROVDataInput->pRelObj->ObjectNr) !=
                 ASSOC_LANE_RIGHT)) {
                /* Objekt ist nict mehr sichtbar --> Zielverlustgrund */
                eRelObjVerlustGrund[ROVDataInput->OOINr] = OBJ_LOSS_CURVE_LEFT;
                dKruemmungWaehrendROVMin[ROVDataInput->OOINr] =
                    ROVDataInput->TrackStandardCurve;
                SIObReObRelObjVerlRueckTimeDist(ROVDataInput);
            }
        }
        /* Rechtskurve, Objekt war rechts */
        else if ((ROVDataInput->TrackStandardCurve < 0.f) &&
                 (ROVDataInput->pRelObj->ObjAngle < -dGrenzwinkel)) {
            /* Objekt noch da, aber zum Kurvenaussenrand verschwunden ? */
            if ((SIBasePreselObjList[ROVDataInput->pRelObj->ObjectNr] ==
                 FALSE) ||
                (OBJ_IS_NEW(ROVDataInput->pRelObj->ObjectNr)) ||
                (OBJ_GET_ASSOCIATED_LANE(ROVDataInput->pRelObj->ObjectNr) !=
                 ASSOC_LANE_LEFT)) {
                /* Objekt ist nict mehr sichtbar --> Zielverlustgrund */
                eRelObjVerlustGrund[ROVDataInput->OOINr] = OBJ_LOSS_CURVE_RIGHT;
                dKruemmungWaehrendROVMin[ROVDataInput->OOINr] =
                    ROVDataInput->TrackStandardCurve;
                SIObReObRelObjVerlRueckTimeDist(ROVDataInput);
            }
        } else {
            /* do nothing but satisfy lint */
        }
    }

    /* Hier Nachbearbeitung, um OBJ_LOSS_DISAPPEARED zu detektieren: */

    if (eRelObjVerlustGrund[ROVDataInput->OOINr] == OBJ_LOSS_NO_INFO) {
        /* Zunaechst nur bei Geradeausfahrt und bei mindestens
         * ROVG_AUFGELOEST_VEIGENMIN: */
        if (ROVDataInput->TPVehicleSpeed > ROVG_AUFGELOEST_VEIGENMIN) {
            /* Winkel des Objekts innerhalb entsprechender Winkelgrenzen: */
            /* dGrenzwinkel hier: ROVG_GRENZWINKEL */
            if (fABS(ROVDataInput->pRelObj->ObjAngle) < dGrenzwinkel) {
                if (((ROVDataInput->pRelObj->LatDisplRoadBordR -
                      ROVDataInput->pRelObj->ObjDistY) < 0.F)
                    // ">" was used, but LatDisplRoadBordR is always less than
                    // 0, so that if the target vehilce is in lane,
                    // the result should be smaller than 0, changed by heqiushu
                    // 20211116
                    && ((ROVDataInput->pRelObj->LatDisplRoadBordL -
                         ROVDataInput->pRelObj->ObjDistY) > 0.F))
                // the same reason to change comapration symbol, changed by
                // heqiushu 20211116
                {
                    /* Uebergang Fahrend-> Stehend beruecksichtigen: Wenn, dann
                     * Objekt nicht aufgeloest! */
                    /* Wenn nicht, dann naechste Pruefung: */
                    if (!((ROVDataInput->RelTrckObjNr != OBJ_INDEX_NO_OBJECT) &&
                          (OBJ_DYNAMIC_PROPERTY(ROVDataInput->RelTrckObjNr) ==
                           CR_OBJECT_PROPERTY_STATIONARY) &&
                          (OBJ_IS_MOVING_TO_STATIONARY(
                              ROVDataInput->RelTrckObjNr)))) {
                        /* frueher: Wenn an derselben Stelle noch das fahrende
                         * Objekt vorhanden ist, */
                        /*          dieses allerdings die Relevanz verloren hat,
                         * dann ebenfalls kein ROVG aufgel�st ! */
                        /*          Objektnummer sollte in RelObj.ObjectNr sein.
                         */
                        /* jetzt:   ROVG aufgel�st auch dann setzen, wenn
                         * relevantes Objekt wegen */
                        /*          Ziel- oder Objektsicherheit verloren ging */
                        if (OBJ_GET_AVLC_FUN_PRESEL_QUALITY(
                                ROVDataInput->pRelObj->ObjectNr) <
                            FUN_PRESEL_AVLC_MIN_INLANE_OBJ_QUAL) {
                            /* TB A ARS200 SCR 159  20-12-2004 */
                            if ((fABS(ROVDataInput->TrackStandardCurve) <
                                 ROVG_GRENZKRUEMMUNG_AUS) &&
                                (ROVDataInput->pRelObj->DistX <
                                 ROVG_ABSTAND_MAX) &&
                                (ROVDataInput->pRelObj->DistX >=
                                 ROVG_ABSTAND_MIN)) {
                                /* TB E ARS200 SCR 159  20-12-2004 */
                                eRelObjVerlustGrund[ROVDataInput->OOINr] =
                                    OBJ_LOSS_DISAPPEARED;
                                dROVGHalteDistanz[ROVDataInput->OOINr] =
                                    ROVDataInput->pRelObj->DistX;
                                dROVGHalteVObjekt[ROVDataInput->OOINr] =
                                    ROVDataInput->TPVehicleSpeed;
                            }

                            if ((fABS(ROVDataInput->TrackStandardCurve) <
                                 ROVG_GRENZKRUEMMUNG_AUS_AF) &&
                                (ROVDataInput->pRelObj->DistX <
                                 ROVG_FLAG_ABSTAND_MAX) &&
                                (ROVDataInput->TrackStandardGradient >
                                 (-ROVG_FLAG_GRENZGRADIENT_AF)) &&
                                (ROVDataInput->TrackStandardGradient <
                                 ROVG_FLAG_GRENZGRADIENT_AF)) {
                                RelObjDisappearedFlag[ROVDataInput->OOINr] =
                                    TRUE;
                            }
                        }
                    }
                }
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIObReObRelObjVerlRueckTimeDist */
static void SIObReObRelObjVerlRueckTimeDist(
    const ROVDataInput_t *ROVDataInput) {
    /*--- VARIABLES ---*/
    float32 TimeDist;
    float32 dGrenzwinkel = ROVG_GRENZWINKEL;
    float32 DistanceToFOV = 0.0f;
    SIFastTrackSwitchOff_t FastTrackSwitchOff;

    /* The reset flag of OBJ_LOSS_DISAPPEARED */
    RelObjDisappearedFlag[ROVDataInput->OOINr] = FALSE;

    /* Anderer Grenzwinkel fuer nahe Verfolgte: */
    /*!< Remark: pRelObj points to the global variable SIRelObject, which is
     * properly initialized -> variables of the structure can be always accessed
     */
    if (ROVDataInput->pRelObj->DistX < ROVG_GRENZE_BX) {
        dGrenzwinkel = ROVG_GRENZWINKEL_BX_NEU;
    }

    if (eRelObjVerlustGrund[ROVDataInput->OOINr] == OBJ_LOSS_DISAPPEARED) {
        dROVGHalteDistanz[ROVDataInput->OOINr] -=
            ROVDataInput->TPVehicleSpeed * SI_CYCLE_TIME;
        dROVGHalteDistanz[ROVDataInput->OOINr] +=
            dROVGHalteVObjekt[ROVDataInput->OOINr] * SI_CYCLE_TIME;

        if (dROVGHalteVObjekt[ROVDataInput->OOINr] > (float32)0.) {
            dROVGHalteVObjekt[ROVDataInput->OOINr] -=
                ROVG_BREMSUNG * SI_CYCLE_TIME;
        }

        if (dROVGHalteDistanz[ROVDataInput->OOINr] <= 0.f) {
            SIReObRelObVerlustGrundInit();
        }
    } else if ((eRelObjVerlustGrund[ROVDataInput->OOINr] ==
                OBJ_LOSS_CURVE_LEFT) ||
               (eRelObjVerlustGrund[ROVDataInput->OOINr] ==
                OBJ_LOSS_CURVE_RIGHT)) {
        const float32 fSinBorderAngle = GDBcos_52(DEG2RAD(dGrenzwinkel));
        const float32 fCosBorderAngle = GDBcos_52(DEG2RAD(90.f - dGrenzwinkel));
        const float32 fTrackCurvDivisor =
            MAX_FLOAT(fABS(ROVDataInput->TrackStandardCurve), C_F32_DELTA);

        DistanceToFOV =
            (2.f * fSinBorderAngle * fCosBorderAngle) / fTrackCurvDivisor;

        if (ROVDataInput->TPVehicleSpeed > C_F32_DELTA) {
            TimeDist = DistanceToFOV / ROVDataInput->TPVehicleSpeed;
        } else {
            TimeDist = SI_ROV_RESET_MAX_TIME_DIST;
        }

        /* OBJ_LOSS_CURVE_RIGHT oder OBJ_LOSS_CURVE_LEFT */
        /* ROV loeschen bei:                              */
        /* - Sichtweite bis Erfassungsrand > 2,1s oder > 100m
              && Kurvenradius hat sich gegenueber Zielverlust wieder
           verdoppelt*/
        /* - Vorzeichenwechsel  des Kurvenradius                 */
        if ((TimeDist >= SI_ROV_RESET_MAX_TIME_DIST) ||
            (DistanceToFOV > SI_ROV_MAX_DIST)) {
            SIReObRelObVerlustGrundInit();
        } else {
            const float32 fCurStoredCurvAbs =
                fABS(dKruemmungWaehrendROVMin[ROVDataInput->OOINr]);
            const float32 fNewCurvAbs = fABS(ROVDataInput->TrackStandardCurve);
            if (fCurStoredCurvAbs < fNewCurvAbs) {
                dKruemmungWaehrendROVMin[ROVDataInput->OOINr] =
                    ROVDataInput->TrackStandardCurve;
            }
        }
    } else {
        /* Keiner der folgenden ROV-Gruenden ist aktiv */
        /* OBJ_LOSS_DISAPPEARED                       */
        /* OBJ_LOSS_CURVE_LEFT                        */
        /* OBJ_LOSS_CURVE_RIGHT                       */
        /* OBJ_LOSS_CURVE_LEFT_AHEAD                  */
        /* OBJ_LOSS_CURVE_RIGHT_AHEAD                 */
    }

    if ((ROVDataInput->RelTrckObjNr != OBJ_INDEX_NO_OBJECT) &&
            (OBJ_DYNAMIC_PROPERTY(ROVDataInput->RelTrckObjNr) ==
             CR_OBJECT_PROPERTY_MOVING) ||
        ROVDataInput->TPVehicleSpeed <= ROVG_VEIGENMIN) {
        /* Relevant Objekt Verlustgrund loeschen */
        SIReObRelObVerlustGrundInit();
    }

    /* ROVG Ausgabe bei zu grosser Spurkruemmung ist unabhaengig ob gerade das
     * rel. Objekt verloren wird, */
    /* wird jedoch nur ausgegeben, wenn kein anderer Relevant Objekt
     * Verlustgrund detektiert ist.        */
    FastTrackSwitchOff = SISeReObFastTrackAbschaltung();

    if (FastTrackSwitchOff == RIGHT_CURVE_SWITCH_OFF) {
        /* Starker Rechtseinschlag, sofern kein anderer Grund detektiert: */

        if (eRelObjVerlustGrund[ROVDataInput->OOINr] == OBJ_LOSS_NO_INFO) {
            eRelObjVerlustGrund[ROVDataInput->OOINr] = OBJ_LOSS_STEER_RIGHT;
            dKruemmungWaehrendROVMin[ROVDataInput->OOINr] =
                ROVDataInput->TrackStandardCurve;
        }
    } else if (FastTrackSwitchOff == LEFT_CURVE_SWITCH_OFF) {
        /* Starker Linkseinschlag, sofern kein anderer Grund detektiert:  */

        if (eRelObjVerlustGrund[ROVDataInput->OOINr] == OBJ_LOSS_NO_INFO) {
            eRelObjVerlustGrund[ROVDataInput->OOINr] = OBJ_LOSS_STEER_LEFT;
            dKruemmungWaehrendROVMin[ROVDataInput->OOINr] =
                ROVDataInput->TrackStandardCurve;
        }
    } else {
        /* Spurkruemmung blendet keine rel. Objekte aus, deshalb diesen ROVG
         * falls gesetzt */
        /* zuruecknehmen, alle anderen jedoch beibehalten falls diese gesetzt
         * sind.       */

        if ((eRelObjVerlustGrund[ROVDataInput->OOINr] == OBJ_LOSS_STEER_LEFT) ||
            (eRelObjVerlustGrund[ROVDataInput->OOINr] ==
             OBJ_LOSS_STEER_RIGHT)) {
            SIReObRelObVerlustGrundInit();
        }
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */