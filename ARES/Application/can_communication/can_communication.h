/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  * 
  * @file     can_communication.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note     通过固定速率RTOS任务计算PID并且完成发送
  * @version  0.1
  * @date     2020-10-12
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */

#ifndef CAN_COMMUNIATION
#define CAN_COMMUNIATION
#include "main.h"
#include "pid.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"

//电机类型
typedef enum { NONE, VESC_MOTOR, RM_MOTOR } motor_type_e;

//统一电机状态结构体
typedef struct {
  fp32    angle;               //电机角度值，弧度制
  fp32    last_angle;          //上次电机角度值
  int32_t angle_time_stamp;    //最新角度值的时间
  fp32    rpm;                 //电机转速
  uint8_t temperature;         //电机温度
  fp32    current;             //电机电流
  int32_t rpm_time_stamp;      //最新转速值的时间
  fp32    cumulative_position; //电机累积转子位置,弧度制
  int16_t cumulative_turn;     //电机累积旋转圈数
} motor_status_t;

//电机指令类型
typedef enum { ANGLE, RPM, DUTY, CURRENT } motor_instruct_e;
//电机的一些常数
#define RM_ECD2RAD 0.0007669903939428206f
#define RM_CURRENT_MAX_OUT 16384.0f
#define MOTOR_INIT_TIME 500
#define MOTOR_CONTROL_TIME 2

#ifndef PI
#define PI 3.141592653589793238462643383279f
#endif

//默认PID
#define RM_DEFAULT_KP 3.0f
#define RM_DEFAULT_KI 1.0f
#define RM_DEFAULT_KD 0.0f
#define RM_CURRENT_PID_MAX_OUT RM_CURRENT_MAX_OUT
#define RM_RPM_PID_MAX_OUT 2048.0f
#define RM_DEFAULT_MAX_IOUT 2000.0f

//统一电机指令结构体
typedef struct {
  bool_t            transmitted_flag; //是否被发送
  PidTypeDef *      motor_speed_pid;
  PidTypeDef *      motor_angle_pid;
  motor_instruct_e  instruct_type;
  fp32              set_angle;
  fp32              set_rpm;
  fp32              set_current;
  fp32              set_duty;
  SemaphoreHandle_t instruct_mutex;
} motor_instruct_t;

//统一电机结构体
typedef struct {
  motor_type_e       motor_type;
  motor_status_t     motor_status;
  motor_instruct_t   motor_instruct;
  CAN_HandleTypeDef *motor_can;
} motor_t;

//统一电机更新函数
extern void motor_set_speed_pid(uint8_t id, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout);
extern void motor_set_angle_pid(uint8_t id, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout);
extern void motor_set_rpm(uint8_t id, fp32 rpm);
extern void motor_set_angle(uint8_t id, fp32 angle);
extern void motor_set_duty(uint8_t id, fp32 duty);
extern void motor_set_current(uint8_t id, fp32 current);
extern void motor_clear_cumulative_turn(uint8_t id);

//统一电机指令函数
extern void instruct_motor(uint8_t id);

//统一电机指针获取
extern const motor_t *get_motor_id_pointer(uint8_t id);
extern const motor_t *get_motor_all_pointer(void);

extern void can_communication_task(void *pvParameters);
extern void test_throw(void *argument);
    
typedef enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT,
  CAN_PACKET_SET_CURRENT_BRAKE,
  CAN_PACKET_SET_RPM,
  CAN_PACKET_SET_POS,
  CAN_PACKET_FILL_RX_BUFFER,
  CAN_PACKET_FILL_RX_BUFFER_LONG,
  CAN_PACKET_PROCESS_RX_BUFFER,
  CAN_PACKET_PROCESS_SHORT_BUFFER,
  CAN_PACKET_STATUS,
  CAN_PACKET_SET_CURRENT_REL,
  CAN_PACKET_SET_CURRENT_BRAKE_REL,
  CAN_PACKET_SET_CURRENT_HANDBRAKE,
  CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
  CAN_PACKET_STATUS_2,
  CAN_PACKET_STATUS_3,
  CAN_PACKET_STATUS_4,
  CAN_PACKET_PING,
  CAN_PACKET_PONG,
  CAN_PACKET_DETECT_APPLY_ALL_FOC,
  CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
  CAN_PACKET_CONF_CURRENT_LIMITS,
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
  CAN_PACKET_CONF_CURRENT_LIMITS_IN,
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
  CAN_PACKET_CONF_FOC_ERPMS,
  CAN_PACKET_CONF_STORE_FOC_ERPMS,
  CAN_PACKET_STATUS_5,
  CAN_PACKET_POLL_TS5700N8501_STATUS
} VESC_CAN_COMMAND_ID;

#endif
