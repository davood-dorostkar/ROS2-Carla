         
#ifndef _BML_EXT_INCLUDED
  // #pragma message(__FILE__": Inclusion of BML_complex.h is discouraged. It exists only for compatibility with CR3xx and might be deleted without prior notice. Include BML_ext.h instead.")
#endif 

#ifndef _BML_COMPLEX_INCLUDED
#define _BML_COMPLEX_INCLUDED

void CML_v_Cartesian2Polar(const t_Complexf32 cartesian, 
                           t_ComplexPolarf32 * const p_polar);

void CML_v_Polar2Cartesian(const t_ComplexPolarf32 polar, 
                           t_Complexf32 * const p_cartesian);

void CML_v_PhaseUnwrapping(t_ComplexPolarf32 a_values[], 
                           const uint32 u_NofChannels, 
                           const uint32 u_NofSubarrays);

void BML_v_MultiplyComplex(const t_Complexf32 *p_Fac1, 
                           const t_Complexf32 *p_Fac2, 
                           const boolean b_Conj, 
                           t_Complexf32 *p_Prod);

void BML_v_DivideComplex(const t_Complexf32 *p_Num, 
	                     const t_Complexf32 *p_Denom,
                         t_Complexf32 *p_Quot);

void BML_v_QuadraticEquationComplex(const t_Complexf32 *p_p, 
                                    const t_Complexf32 *p_q,
                                    t_Complexf32 *p_z1,
                                    t_Complexf32 *p_z2);

#endif  

