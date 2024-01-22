         
#ifndef _BML_EXT_INCLUDED
  // #pragma message(__FILE__": Inclusion of BML_fourier.h is discouraged. It exists only for compatibility with CR3xx and might be deleted without prior notice. Include BML_ext.h instead.")
#endif 

#ifndef _BML_FOURIER_INCLUDED
#define _BML_FOURIER_INCLUDED

void CML_v_FFT(t_Complexf32 a_Elements[], const uint8 u_LogNofElements, boolean b_Inverse);

void CML_v_InverseFFT16(t_Complexf32 a_Elements[]);

#endif  

