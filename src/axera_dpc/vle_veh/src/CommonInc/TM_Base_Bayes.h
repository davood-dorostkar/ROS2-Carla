         
#ifndef _BML_EXT_INCLUDED
  // #pragma message(__FILE__": Inclusion of BML_bayes.h is discouraged. It exists only for compatibility with CR3xx and might be deleted without prior notice. Include BML_ext.h instead.")
#endif 

#ifndef _BML_BAYES_INCLUDED
#define _BML_BAYES_INCLUDED

#define CML_BAYES2_CPT_SIZE (4)   
#define BML_BAYES3_CPT_SIZE (8)   
#define BML_BAYES4_CPT_SIZE (16)  
#define BML_BAYES5_CPT_SIZE (32)  

extern percentage_t CML_Bayes2(percentage_t u_ProbabilityA,
                               percentage_t u_ProbabilityB,
                               const percentage_t a_CPT[CML_BAYES2_CPT_SIZE]);

extern percentage_t BML_Bayes3(percentage_t u_ProbabilityA,
                               percentage_t u_ProbabilityB,
                               percentage_t u_ProbabilityC,
                               const percentage_t a_CPT[BML_BAYES3_CPT_SIZE]);

extern percentage_t CML_Bayes4(percentage_t u_ProbabilityA, 
                               percentage_t u_ProbabilityB,
                               percentage_t u_ProbabilityC,
                               percentage_t u_ProbabilityD,
                               const percentage_t a_CPT[BML_BAYES4_CPT_SIZE]);

extern percentage_t CML_Bayes5(percentage_t u_ProbabilityA, 
                               percentage_t u_ProbabilityB, 
                               percentage_t u_ProbabilityC, 
                               percentage_t u_ProbabilityD, 
                               percentage_t u_ProbabilityE, 
                               const percentage_t a_CPT[BML_BAYES5_CPT_SIZE]);

#endif 

