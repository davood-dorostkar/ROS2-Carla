         
#ifndef _BML_EXT_INCLUDED
  // #pragma message(__FILE__": Inclusion of BML_vector.h is discouraged. It exists only for compatibility with CR3xx and might be deleted without prior notice. Include BML_ext.h instead.")
#endif 

#ifndef _BML_VECTOR_INCLUDED
#define _BML_VECTOR_INCLUDED

#define VECTOR_I32_VALUE_INVALID      0x7FFFFFFF

extern void Vector2AddP_i16(const Vector2_i16_t * p_vector1, Vector2_i16_t * p_vector2);
extern void Vector2SubP_i16(Vector2_i16_t * p_min, const Vector2_i16_t * p_sub);
extern Vector2_f32_t Vector2Mull_i16(Vector2_i16_t Vec, float32 f_Factor);
extern sint32 Vector2ScalarProd_i16(Vector2_i16_t Point1, Vector2_i16_t Point2);
extern sint32 Vector2GetSqrDist_i16(Vector2_i16_t input_vector);
extern void Vector2MathRotate2D_i16(BML_t_TrafoMatrix2D const *p_Transform_Matrix , Vector2_i16_t *p_input_vector);
extern float32 Vector2AngleToXAxis_i16(Vector2_i16_t input_vector);
extern void Vector2Init_i16(Vector2_i16_t * p_Vector);

extern Vector2_i32_t Vector2AddI16_i32(Vector2_i32_t input_vector1, Vector2_i16_t input_vector2);
extern sint32 Vector2ScalarProd_i32(Vector2_i32_t Point1, Vector2_i32_t Point2);
extern boolean Vector2YAxisIntersectionAdd_i32(Vector2_i32_t Point1, Vector2_i32_t Point2, sint32 *p_YIntersection);

extern Vector2_f32_t Vector2Add_f32(Vector2_f32_t vector1, Vector2_f32_t vector2);
extern Vector2_f32_t Vector2Sub_f32(Vector2_f32_t min, Vector2_f32_t sub);
extern boolean Vector2GetYPointOnLine_f32(Vector2_f32_t Point, Vector2_i16_t DirectionVector, Vector2_f32_t *p_InOut);
extern float32 Vector2GetSqrDist_f32(Vector2_f32_t input_vector);
extern float32 Vector2GetSqrDistI32_f32(Vector2_i32_t input_vector);
extern float32 Vector2GetSqrDistI16_f32(Vector2_i16_t input_vector);
extern float32 Vector2AngleToXAxis_f32(Vector2_f32_t input_vector);
extern float32 VectorWeightedPNorm(float32 a_vector[], float32 a_weights[], uint32 u_length, uint32 u_exponent);

#endif  

