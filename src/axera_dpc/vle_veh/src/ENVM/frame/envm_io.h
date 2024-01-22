#ifndef ENVM_IO_H
#define ENVM_IO_H
#ifdef __cplusplus
extern "C" {
#endif
extern void EMProcessObjOutput(void);
extern void EMErrorTrap(const char a_file[], i32_t s_line, ui32_t u_errorType);
extern void v_EMSigCheckInit(void);
extern void FPSDeleteObject(const sint8 ObjNr);
void EMErrorTrapInitGlobalData(void);
void EnvmProcessInput(void);
void Envm_v_SetSignalHeader(ENVMSignalHeader_t *p_SigHeader);
void EnvmInputInit(void);
void EnvmProcessOutput(void);
#ifdef __cplusplus
}
#endif
#endif
