/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_EGOLANEGENERATION_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_EGOLANEGENERATION_H_

#include "tue_common_libs.h"
#include "LBP_Ext.h"
void EgoLaneGenerationReset(void);

void INPUT_EgoLaneGeneration(const sLBPInput_t *pLBPInput,
                             const sLBPParam_t *pLBPParam,
                             const sULPOutput_t *pULPOutput,
                             const sCLPOutput_t *pCLPOutput,
                             const sLFPOutput_t *pLFPOutput,
                             sELGInput_t *pELGInput,
                             sELGParam_t *pELGParam);
void INPUT_EgoSafetyInterface(const sELGInput_t *pELGInput,
                              const sELGParam_t *pELGParam,
                              sESIInput_t *pESIInput,
                              sESIParam_t *pESIParam);
void ESI_CalHeading(const sESIInput_t *pESIInput,
                    const sESIParam_t *pESIParam,
                    sESIOutput_t *pESIOutput);
void ESI_CalPosY(const sESIInput_t *pESIInput,
                 const sESIParam_t *pESIParam,
                 sESIOutput_t *pESIOutput,
                 sESIDebug_t *pESIDebug);
void ESI_CheckLeftSafetyLaneData(const sESIInput_t *pESIInput,
                                 const sESIParam_t *pESIParam,
                                 sESIOutput_t *pESIOutput,
                                 sESIDebug_t *pESIDebug);
void EgoSafetyInterface(const sESIInput_t *pESIInput,
                        const sESIParam_t *pESIParam,
                        sESIOutput_t *pESIOutput,
                        sESIDebug_t *pESIDebug);
void INPUT_EgoControlInterface(const sELGInput_t *pELGInput,
                               const sELGParam_t *pELGParam,
                               sECIInput_t *pECIInput,
                               sECIParam_t *pECIParam);
UINT8_T ECI_bValidBoth(UINT8_T uLaneValidQualifier, UINT8_T uBridgePossible);
void ECI_EgoLaneBoundariesOut(UINT8_T bValidBoth,
                              UINT8_T bValidOnlyLf,
                              UINT8_T bValidOnlyRi,
                              const sECIInput_t *pECIInput,
                              const sECIParam_t *pECIParam,
                              sECIOutput_t *pECIOutput);
void ECI_RangeChecks(const sECIParam_t *pECIParam, sECIOutput_t *pECIOutput);
void ECI_DetermineDMCQualifier(const sECIInput_t *pECIInput,
                               sECIOutput_t *pECIOutput);
void ECI_DetermineVisualQualifier(UINT8_T bValidOnlyLf,
                                  UINT8_T bValidOnlyRi,
                                  const sECIInput_t *pECIInput,
                                  sECIOutput_t *pECIOutput);
void ECI_DetermineLeftLaneQualifierBit(const sECIInput_t *pECIInput,
                                       sECIOutput_t *pECIOutput);
void ECI_DetermineLeftLaneQualifier(UINT8_T bValidBoth,
                                    const sECIParam_t *pECIParam,
                                    const sECIInput_t *pECIInput,
                                    sECIOutput_t *pECIOutput);
void ECI_DetermineRightLaneQualifierBit(const sECIInput_t *pECIInput,
                                        sECIOutput_t *pECIOutput);
void ECI_DetermineRightLaneQualifier(UINT8_T bValidBoth,
                                     const sECIParam_t *pECIParam,
                                     const sECIInput_t *pECIInput,
                                     sECIOutput_t *pECIOutput);
void EgoControlInterface(const sECIInput_t *pECIInput,
                         const sECIParam_t *pECIParam,
                         sECIOutput_t *pECIOutput,
                         sECIDebug_t *pECIDebug);
void OUTPUT_EgoLaneGeneration(const sESIOutput_t *pESIOutput,
                              const sESIDebug_t *pESIDebug_t,
                              const sECIOutput_t *pECIOutput,
                              const sECIDebug_t *pECIDebug_t,
                              sELGOutput_t *pELGOutput,
                              sELGDebug_t *pELGDebug);
void EgoLaneGeneration(const sELGInput_t *pELGInput,
                       const sELGParam_t *pELGParam,
                       sELGOutput_t *pELGOutput,
                       sELGDebug_t *pELGDebug);
void LBPOutput(const sULPOutput_t *pULPOutput,
               const sULPDebug_t *pULPDebug,
               const sCLPOutput_t *pCLPOutput,
               const sCLPDebug_t *pCLPDebug,
               const sLFPOutput_t *pLFPOutput,
               const sLFPDebug_t *pLFPDebug,
               const sELGOutput_t *pELGOutput,
               const sELGDebug_t *pELGDebug,
               sLBPOutput_t *pLBPOutput,
               sLBPDebug_t *pLBPDebug);
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_EGOLANEGENERATION_H_
