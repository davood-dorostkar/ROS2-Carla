

#ifndef _CP_KALMAN_INCLUDED
#define _CP_KALMAN_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif
/*! @cond Doxygen_Suppress */
extern void CPKalmanInit(const CPTrajDistKalmanMeas_t *pMeasurement,
                         CPTrajDistKalmanData_t *pTrajDist);
extern void CPKalmanPredict(CPTrajDistKalmanData_t *pTrajDist,
                            const GDBSymMatrix2_t *pQ,
                            float32 fCycleTime);
extern void CPKalmanUpdate(const CPTrajDistKalmanMeas_t *pMeasurement,
                           CPTrajDistKalmanData_t *pTrajDist);
extern void CPKalmanFiltering(ObjNumber_t iObj,
                              const CPTrajDistKalmanMeas_t *pMeasurement,
                              const GDBSymMatrix2_t *pQ,
                              float32 fCycleTime,
                              CPTrajDistKalmanData_t *pTrajDist);

extern void CPKalmanUpdatePos(const CPPosSamples_t *pFUSED_Samples,
                              CPNoiseModelLinear_t sNoiseModel,
                              const CPTrajectoryData_t *pTrajData);
extern void CPKalmanUpdateGrad(const CPGradSamples_t *pFUSED_GradSamples,
                               const CPTrajectoryData_t *pTrajData);
extern void CPKalmanPredictTrajectory(const CPTrajectoryData_t *pTrajData);

extern float32 CPKalmanGetTrajEstState(const CPTrajectoryData_t *pTrajData,
                                       uint8 uiVarState);
extern float32 CPKalmanGetTrajErrorCov(const CPTrajectoryData_t *pTrajData,
                                       uint8 uiVarState);

#ifdef __cplusplus
};
#endif
/*! @endcond Doxygen_Suppress */
#endif
