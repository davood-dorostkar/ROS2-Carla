         
#ifdef PRQA_SIZE_T
      
  #pragma PRQA_MACRO_MESSAGES_OFF "BML_ASSERT" 3112
#endif 

#ifndef _BML_EXT_INCLUDED
  // #pragma message(__FILE__": Inclusion of BML_cfg.h is discouraged. It exists only for compatibility with CR3xx and might be deleted without prior notice. Include BML_ext.h instead.")
#endif 

#ifndef _BML_CFG_INCLUDED
#define _BML_CFG_INCLUDED

#if (defined(WIN32) && !defined(__PDO__))
  #include <assert.h>
#endif

#define BML_MatrixBoundsCheckOn (0)

typedef uint8 BML_enum_t;

#if (defined(WIN32) && !defined(__PDO__))
  #define BML_ASSERT(cond) assert(cond)
#else
  #define BML_ASSERT(cond) ((void)0)
#endif

#endif 

