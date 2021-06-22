/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     quick_test_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    夹爪
  * @version  0.1
  * @date     2021-04-23
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "quick_test_task.h"
#include "RM_motor.h"
#include "feedback_task.h"

static uint32_t last_wake;
static uint8_t  zeroed   = 0;
static int32_t  timebase = 0;
static fp32     set_angle;
static fp32     set_rads;
static fp32     setrpm;
static Motor *  test_motor;
static fp32     dir = 0;

PID_ControllerParam        whip_speed_pidparam = {.general  = PIDPOS_CONTROLLER,
                                           .Int_type = NORMAL_INT,
                                           .kP       = 5.0f,
                                           .kI       = 0.0f,
                                           .kD       = 0.0f,
                                           .kB       = 0.03f,
                                           .max_Iout = 5000,
                                           .N        = 45};
PID_Controller             whip_angle_pid;
PID_ControllerParam        whip_angle_pid_param   = {.general  = PIDPOS_CONTROLLER,
                                            .Int_type = BACK_CALCULATION_INT,
                                            .kP       = 8000.0f,
                                            .kI       = 0.0f,
                                            .kD       = 0.0f,
                                            .kB       = 0.3f,
                                            .max_Iout = 5000,
                                            .N        = 70};
static fp32                whip_angle_O_Hlim      = 8000;
static fp32                whip_angle_O_Llim      = -8000;
static fp32                whip_angle_I_loop_Llim = 0;
static fp32                whip_angle_I_loop_Hlim = 0;
static ControllerConstrain whip_angle_constrain   = {.I_loop_Hlim = &whip_angle_I_loop_Hlim,
                                                   .I_loop_Llim = &whip_angle_I_loop_Llim,
                                                   .O_Hlim      = &whip_angle_O_Hlim,
                                                   .O_Llim      = &whip_angle_O_Llim};

static fp32       ag;
static fp32       rpmcom;
static fp32       rpm;
MotorInstructType doubleMotorCtrl0(Motor *motor, Controller *controller, void *param) {
  (void)param;
  ag     = (2 * PI * motor->status.cumulative_turn + motor->status.angle - motor->status.zero) / M2006_REDUCTION_RATIO;
  rpmcom = controllerUpdate(controller, &set_angle, &ag, NULL);

  ((RM_Motor *)motor)->set_current =
      controllerUpdate((Controller *)&((RM_Motor *)motor)->speed_pid, &rpmcom, &motor->status.speed, NULL);
  return INSTRUCT_CURRENT;
}

void quick_test_task() {
  vTaskDelay(1024);
  while(test_motor==NULL)
  test_motor = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 4);

  PID_ControllerInit(&whip_angle_pid, &whip_angle_constrain, &whip_angle_pid_param, 10);

  Motor_SetAltController(test_motor, (Controller *)&whip_angle_pid, NULL, doubleMotorCtrl0);
  Motor_SetSpeedPID(test_motor, &whip_speed_pidparam);
  static fp32 temp_kp;
  temp_kp = whip_speed_pidparam.kP;

  for (;;) {
    Sbus_lpf();
    last_wake = xTaskGetTickCount();
    if (zeroed) {
      if (HAL_GPIO_ReadPin(POT_PROXIMITY_GPIO_Port, POT_PROXIMITY_Pin) == GPIO_PIN_RESET) {
        set_angle = PI+0.05;
      } else {
        set_angle = 0;
      }
      Motor_AltControl(test_motor, 2);
    } else {

      whip_speed_pidparam.kP = 1.0f;
      Motor_SetSpeed(test_motor, -2000, 2);
      if (test_motor->status.speed == 0 && test_motor->status.current != 0) {
        ++timebase;
      } else {
        timebase = 0;
      }
      if (timebase > 200) {
        timebase = -1;
        Motor_Zero(test_motor);
        zeroed                 = 1;
        whip_speed_pidparam.kP = temp_kp;
      }
    }
    vTaskDelayUntil(&last_wake, 1);
  }
}
