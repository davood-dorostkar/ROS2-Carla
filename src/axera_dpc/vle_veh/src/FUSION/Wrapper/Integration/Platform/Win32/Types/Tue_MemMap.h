/** \addtogroup AUTOSAR
 *  @{
 * \file        Tue_MemMap.h
 * \brief       Definition of memory map for component Tue
 *
 *
 *
 * <br>=====================================================<br>
 * <b>Copyright 2017 by Tuerme.</b>
 *
 *  All rights reserved. Property of Tuerme.<br>
 *  Restricted rights to use, duplicate or disclose of this code<br>
 *  are granted through contract.
 * <br>=====================================================<br>
 */

#if defined FUSION_START_SEC_CODE
#if defined Tue_CODE_OPEN
#error \
    "FUSION_START_SEC_CODE: Close the former segment before opening a new one."
#else
#undef FUSION_START_SEC_CODE
#define Tue_CODE_OPEN
#pragma code_seg(push, s1, "fusion_code")
#endif
#elif defined FUSION_STOP_SEC_CODE
#undef FUSION_STOP_SEC_CODE
#undef Tue_CODE_OPEN
#pragma code_seg(pop, s1)
#elif defined FUSION_START_SEC_ROM
#if defined Tue_ROM_OPEN
#error \
    "FUSION_START_SEC_ROM: Close the former segment before opening a new one."
#else
#undef FUSION_START_SEC_ROM
#define Tue_ROM_OPEN
#pragma const_seg(push, s2, "fusion_text")
#endif
#elif defined FUSION_STOP_SEC_ROM
#undef FUSION_STOP_SEC_ROM
#undef Tue_ROM_OPEN
#pragma const_seg(pop, s2)
#elif defined FUSION_START_SEC_FUSI_CODE
#if defined Tue_FUSI_CODE_OPEN
#error \
    "FUSION_START_SEC_FUSI_CODE: Close the former segment before opening a new one."
#else
#undef FUSION_START_SEC_FUSI_CODE
#define Tue_FUSI_CODE_OPEN
#pragma code_seg(push, s1, "fusion_code")
#endif
#elif defined FUSION_STOP_SEC_FUSI_CODE
#undef FUSION_STOP_SEC_FUSI_CODE
#undef Tue_FUSI_CODE_OPEN
#pragma code_seg(pop, s1)
#elif defined FUSION_START_SEC_MATH_CODE
#if defined Tue_MATH_CODE_OPEN
#error \
    "FUSION_START_SEC_MATH_CODE: Close the former segment before opening a new one."
#else
#undef FUSION_START_SEC_MATH_CODE
#define Tue_MATH_CODE_OPEN
#pragma code_seg(push, s1, "fusion_code")
#pragma auto_inline(on)
#endif
#elif defined FUSION_STOP_SEC_MATH_CODE
#undef FUSION_STOP_SEC_MATH_CODE
#undef Tue_MATH_CODE_OPEN
#pragma auto_inline(off)
#pragma code_seg(pop, s1)
#elif defined FUSION_START_SEC_VAR8
#if defined Tue_VAR8_OPEN
#error \
    "FUSION_START_SEC_VAR8: Close the former segment before opening a new one."
#else
#undef FUSION_START_SEC_VAR8
#define Tue_VAR8_OPEN
#pragma push_macro("section")
//#pragma section("fusion_data", read, write)
__declspec(allocate("fusion_data"))
#endif
#elif defined FUSION_STOP_SEC_VAR8
#undef FUSION_STOP_SEC_VAR8
#undef Tue_VAR8_OPEN
#pragma pop_macro("section")
#elif defined FUSION_START_SEC_VAR16
#if defined Tue_VAR16_OPEN
#error \
    "FUSION_START_SEC_VAR16: Close the former segment before opening a new one."
#else
#undef FUSION_START_SEC_VAR16
#define Tue_VAR16_OPEN
#pragma push_macro("section")
//#pragma section("fusion_data", read, write)
__declspec(allocate("fusion_data"))
#endif
#elif defined FUSION_STOP_SEC_VAR16
#undef FUSION_STOP_SEC_VAR16
#undef Tue_VAR16_OPEN
#pragma pop_macro("section")
#elif defined FUSION_START_SEC_VAR32
#if defined Tue_VAR32_OPEN
#error \
    "FUSION_START_SEC_VAR32: Close the former segment before opening a new one."
#else
#undef FUSION_START_SEC_VAR32
#define Tue_VAR32_OPEN
#pragma push_macro("section")
//#pragma section("fusion_data", read, write)
__declspec(allocate("fusion_data"))
#endif
#elif defined FUSION_STOP_SEC_VAR32
#undef FUSION_STOP_SEC_VAR32
#undef Tue_VAR32_OPEN
#pragma pop_macro("section")
#elif defined FUSION_START_SEC_VAR_UNSPECIFIED
#if defined Tue_VAR_UNSPECIFIED_OPEN
#error \
    "FUSION_START_SEC_VAR_UNSPECIFIED: Close the former segment before opening a new one."
#else
#undef FUSION_START_SEC_VAR_UNSPECIFIED
#define Tue_VAR_UNSPECIFIED_OPEN
#pragma push_macro("section")
//#pragma section("fusion_data", read, write)
__declspec(allocate("fusion_data"))
#endif
#elif defined FUSION_STOP_SEC_VAR_UNSPECIFIED
#undef FUSION_STOP_SEC_VAR_UNSPECIFIED
#undef Tue_VAR_UNSPECIFIED_OPEN
#pragma pop_macro("section")
#else
#error "Tue_MemMap.h, unknown section"
#endif
