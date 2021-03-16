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
  * @brief    
  * @version  0.1
  * @date     2021-03-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "quick_test_task.h"
#include "RM_motor.h"
#include "feedback_task.h"
static uint32_t            last_wake;
uint8_t                    zeroed   = 0;
uint32_t                   timebase = 0;
static fp32                set_angle;
Motor *                    test_motor[2];
PID_ControllerParam        whip_speed_pidparam = {.general  = PIDPOS_CONTROLLER,
                                           .Int_type = BACK_CALCULATION_INT,
                                           .kP       = 25.0f,
                                           .kI       = 0.0f,
                                           .kD       = 0.0f,
                                           .kB       = 0.3f,
                                           .max_Iout = 5000,
                                           .N        = 100};
PID_Controller             whip_angle_pid;
static PID_ControllerParam whip_angle_pid_param   = {.general  = PIDPOS_CONTROLLER,
                                                   .Int_type = BACK_CALCULATION_INT,
                                                   .kP       = 2000.0f,
                                                   .kI       = 0.0f,
                                                   .kD       = 0.0f,
                                                   .kB       = 0.3f,
                                                   .max_Iout = 5000,
                                                   .N        = 100};
static fp32                whip_angle_O_Hlim      = 8000;
static fp32                whip_angle_O_Llim      = -8000;
static fp32                whip_angle_I_loop_Llim = 0;
static fp32                whip_angle_I_loop_Hlim = 0;
static ControllerConstrain whip_angle_constrain   = {.I_loop_Hlim = &whip_angle_I_loop_Hlim,
                                                   .I_loop_Llim = &whip_angle_I_loop_Llim,
                                                   .O_Hlim      = &whip_angle_O_Hlim,
                                                   .O_Llim      = &whip_angle_O_Llim};

                                                   
static fp32 ag;
MotorInstructType doubleMotorCtrl0(Motor *motor, Controller *controller) {
  ag                       = (2 * PI * motor->status.cumulative_turn + motor->status.angle-motor->status.zero) / M3508_REDUCTION_RATIO;
  fp32 rpm                         = controllerUpdate(controller, &set_angle, &ag, NULL);
  ((RM_Motor *)motor)->set_current = controllerUpdate((Controller*)&((RM_Motor *)motor)->speed_pid, &rpm, &motor->status.speed, NULL);
  return INSTRUCT_CURRENT;
}
MotorInstructType doubleMotorCtrl1(Motor *motor, Controller *controller) {
  (void)controller;
  ((RM_Motor *)motor)->set_current = -((RM_Motor *)test_motor[0])->set_current;
  return INSTRUCT_CURRENT;
}
fp32 qangle=-100.0f;
void quick_test_task() {
  vTaskDelay(4000);

  test_motor[0] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 0);
  test_motor[1] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 1);
  feedback_register(&ag,7);
  PID_ControllerInit(&whip_angle_pid, &whip_angle_constrain, &whip_angle_pid_param, 10);

  Motor_SetAltController(test_motor[0], (Controller*)&whip_angle_pid, doubleMotorCtrl0);
  Motor_SetAltController(test_motor[1], NULL, doubleMotorCtrl1);
  Motor_SetSpeedPID(test_motor[0], &whip_speed_pidparam);

  for (;;) {
    last_wake = xTaskGetTickCount();
    if (zeroed) {
      if (SBUS_CHANNEL[9] > 0.8f&&timebase==0) {
        timebase = 2200;
      }
      if(timebase!=0){
        timebase--;
        set_angle = DEG2RAD(qangle);
      }else{
        set_angle = DEG2RAD(-2.0f);
      }
      Motor_AltControl(test_motor[0], 4);
      Motor_AltControl(test_motor[1], 4);
    } else {
      Motor_SetSpeed(test_motor[0], 2000, 2);
      Motor_SetCurrent(test_motor[1], 0, 2);
      if (test_motor[0]->status.speed == 0 && test_motor[0]->status.current != 0) {
        ++timebase;
      } else {
        timebase = 0;
      }
      if (timebase > 500) {
        timebase = 0;
        Motor_Zero(test_motor[0]);
        Motor_Zero(test_motor[1]);
        zeroed = 1;
      }
    }
    vTaskDelayUntil(&last_wake, 1);
  }
}
