
#ifndef glob_defs_H
#define glob_defs_H

#ifdef __cplusplus
extern "C"
{
#endif

#define C_PI ((float32) 3.14159265359f)

#define C_GRAVITY ((float32) 9.80665f)

#define C_F32_DELTA ((float32) 0.0001f)

#ifndef C_KMH_MS
#define C_KMH_MS ((float32) 3.6f)
#endif

#define C_V_LIGHT ((float32) 299792458.0f)

#define C_BIT_SET (uint8)1u

#define C_BIT_RESET (uint8)0u

#define C_PIN_SET (uint8)1u

#define C_PIN_RESET (uint8)0u

#define FLAG_SET (uint8)1u

#define FLAG_RESET (uint8)0u

#ifndef SWITCH_ON
#define SWITCH_ON 1u
#endif

#ifndef SWITCH_OFF
#define SWITCH_OFF 0u
#endif

#define ARRAY_LEN(a)  \
    (sizeof( a ) / sizeof(*a))

#define BUILD_WARNING(x)  \
  message(__FILE__"("STRING_QUOTE(__LINE__)"): !! ERROR !!: " x)

#define DEG2RAD(deg_)  \
  ((deg_)*(C_PI/180.F))

#define GET_UI16_HIGHBYTE(x)  \
  (uint8)(((x) >> 8) & 0xFF)

#define GET_UI16_LOWBYTE(x)  \
  (uint8)((x) & 0xFF)

#define GET_UI32_HIGHBYTE(x)  \
  (uint8)(((x) >> 24) & 0xFF)

#define GET_UI32_HMIDBYTE(x)  \
  (uint8)(((x) >> 16) & 0xFF)

#define GET_UI32_LMIDBYTE(x)  \
  (uint8)(((x) >> 8) & 0xFF)

#define GET_UI32_LOWBYTE(x)  \
  (uint8)((x) & 0xFF)

#define LP_FILTER(New, Old, FilterT, SampleT)  \
  (((FilterT) / (SampleT) * (Old) + (New)) / (1 + (FilterT) / (SampleT)))

#ifndef MAX
#define MAX(x, y)  \
  (((x)>(y))?(x):(y))
#endif

#ifndef MIN
#define MIN(x, y)  \
  (((x)<(y))?(x):(y))
#endif

#define MINMAX(min, max, value)  \
  (MAX((MIN((value),(max))),(min)))

#define RAD2DEG(rad_)  \
  ((rad_)*(180.F/C_PI))

#define ROUND_TABLE_TO_MULTIPLE_OF_128(ELCNT, ELSIZE)  \
  error - unknown platform!

#define ROUND_TABLE_TO_MULTIPLE_OF_CACHELINE(ELCNT, ELSIZE)  \
  error - unknown platform!

#define ROUND_TO_MULTIPLE_OF_128(ELCNT)  \
  ROUND_TO_MULTIPLE_OF_N(ELCNT, 128)

#define ROUND_TO_MULTIPLE_OF_16(ELCNT)  \
  ROUND_TO_MULTIPLE_OF_N(ELCNT, 16)

#define ROUND_TO_MULTIPLE_OF_32(ELCNT)  \
  ROUND_TO_MULTIPLE_OF_N(ELCNT, 32)

#define ROUND_TO_MULTIPLE_OF_64(ELCNT)  \
  ROUND_TO_MULTIPLE_OF_N(ELCNT, 64)

#define ROUND_TO_MULTIPLE_OF_8(ELCNT)  \
  ROUND_TO_MULTIPLE_OF_N(ELCNT, 8)

#define ROUND_TO_MULTIPLE_OF_CACHELINE(ELCNT)  \
  error - unknown platform!

#define ROUND_TO_MULTIPLE_OF_CALIGN(ELCNT)  \
  error - unknown platform!

#define ROUND_TO_MULTIPLE_OF_N(ELCNT, _N)  \
  ( ( ( (ELCNT) + (_N) - 1 ) / (_N)) * (_N) )

#define SIGN(x)  \
  (((x)==(0))?(0):(((x)>(0))?(1):(-1)))

#define SQR(x)  \
  ((x)*(x))

#define STRING_INTERNAL_QUOTE(x)  \
  #x

#define STRING_QUOTE(x)  \
  STRING_INTERNAL_QUOTE(x)

#define fSIGN(x)  \
  ((F32_IS_ZERO(x))?(0):((x > 0.F)?(1):(-1)))

#ifndef ABS
#define iABS(x)  \
  (((x)<(0L))?(-(x)):(x))
#endif

#ifdef __cplusplus
}
#endif

#endif
 
