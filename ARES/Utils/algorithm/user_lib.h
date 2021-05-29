#ifndef USER_LIB_H
#define USER_LIB_H
#include "main.h"

typedef __packed struct {
  fp32 input;        //��������
  fp32 out;          //�������
  fp32 min_value;    //�޷���Сֵ
  fp32 max_value;    //�޷����ֵ
  fp32 frame_period; //ʱ����
} ramp_function_source_t;

typedef __packed struct {
  fp32 input;        //��������
  fp32 out;          //�˲����������
  fp32 num[1];       //�˲�����
  fp32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
//���ٿ���
extern fp32 invSqrt(fp32 num);

//б��������ʼ��
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//б����������
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
//һ���˲���ʼ��
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//һ���˲�����
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//��������
extern void abs_limit(fp32 *num, fp32 Limit);
//�жϷ���λ
extern fp32 sign(fp32 value);
//��������
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26����
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//�޷�����
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//�Ƕ� ���޷� 180 ~ -180
extern fp32 theta_format(fp32 Ang);
//�������ֵ
#define ABS(x) ((x) > 0 ? (x) : -(x))
//��������֮��ľ���
extern fp32 distance_2d(fp32 x1, fp32 y1, fp32 x2, fp32 y2);

extern fp32 atan2_fast(fp32 x, fp32 y);

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.01745329251994329576923690768489f)
#endif
#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.295779513082320876798154814105f)
#endif

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
//�ж��Ƿ����ж�״̬��
extern inline int is_in_isr(void);
//ͨ����λ�ж�һ��ֵ�ķ��� ������1 ������-1
#define SIGN(x) (((signed char *)&x)[sizeof(x) - 1] >> 7 | 1)
#define SIGNBIT(x) (((signed char *)&x)[sizeof(x) - 1] >> 7)

extern fp32 maxabs4f(fp32 value0, fp32 value1, fp32 value2, fp32 value3);

#endif
