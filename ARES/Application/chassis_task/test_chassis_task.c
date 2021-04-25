#include "motor.h"
#include "sbus.h"
#include "cmsis_os.h"
#include "main.h"
#include "feedback_task.h"
#include "rm_motor.h"
#include "computer_task.h"

#define SQRT2 1.4142135623730950488016887242097f
//#define FILTER_CUTOFF 1000

static Motor *chassisMotor[4];
static fp32   translationSens = 2000;
static fp32   rotationSens    = 2000;

static PID_ControllerParam chassisPid = {.general  = PIDPOS_CONTROLLER,
                                         .Int_type = BACK_CALCULATION_INT,
                                         .kB       = 0.2,
                                         .kD       = 0.3,
                                         .kI       = 0.1,
                                         .kP       = 1.0,
                                         .max_Iout = 16000,
                                         .N        = 25};

static fp32 set[4];

//static MotorInstructType chassis_motorAltCtrl(Motor *motor, Controller *controller) {
//  fp32 setCurrent =
//      controllerUpdate((Controller *)(&((RM_Motor *)motor)->speed_pid), &set[motor->info.id], &motor->status.speed, NULL);
//  fp32 q                           = 2 * PI * FILTER_CUTOFF * ((RM_Motor *)motor)->speed_pid.dt;
//  ((RM_Motor *)motor)->set_current = q * setCurrent + (1 - q) * ((RM_Motor *)motor)->set_current;
//  return INSTRUCT_CURRENT;
//}

void chassis_task1(void *argument) {
  vTaskDelay(623);

  while (chassisMotor[0] == NULL || chassisMotor[1] == NULL || chassisMotor[2] == NULL || chassisMotor[3] == NULL) {
    chassisMotor[0] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 0);
    chassisMotor[1] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 1);
    chassisMotor[2] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 2);
    chassisMotor[3] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 3);
    vTaskDelay(10);
  }

  Motor_SetSpeedPID(chassisMotor[0], &chassisPid);
  Motor_SetSpeedPID(chassisMotor[1], &chassisPid);
  Motor_SetSpeedPID(chassisMotor[2], &chassisPid);
  Motor_SetSpeedPID(chassisMotor[3], &chassisPid);

  for (;;) {
    if (sbus.channel[SBUS_SA_CHANNEL] > 0) {
      if (xTaskGetTickCount() - computerRxMsg.updateTime > 1500) {
        set[0] = (-0.5f * SQRT2 * translationSens * sbus.channel[2] - 0.5f * SQRT2 * translationSens * sbus.channel[0] +
                  rotationSens * sbus.channel[3]);
        set[1] = (0.5f * SQRT2 * translationSens * sbus.channel[2] - 0.5f * SQRT2 * translationSens * sbus.channel[0] +
                  rotationSens * sbus.channel[3]);
        set[2] = (0.5f * SQRT2 * translationSens * sbus.channel[2] + 0.5f * SQRT2 * translationSens * sbus.channel[0] +
                  rotationSens * sbus.channel[3]);
        set[3] = (-0.5f * SQRT2 * translationSens * sbus.channel[2] + 0.5f * SQRT2 * translationSens * sbus.channel[0] +
                  rotationSens * sbus.channel[3]);
      } else {
        set[0] = (-0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vx -
                  0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vy + rotationSens * computerRxMsg.instruct.wz);
        set[1] = (0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vx -
                  0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vy + rotationSens * computerRxMsg.instruct.wz);
        set[2] = (0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vx +
                  0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vy + rotationSens * computerRxMsg.instruct.wz);
        set[3] = (-0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vx +
                  0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vy + rotationSens * computerRxMsg.instruct.wz);
      }
    } else {
      set[0] = set[1] = set[2] = set[3] = 0;
    }

    Motor_SetSpeed(chassisMotor[0], set[0], 2);
    Motor_SetSpeed(chassisMotor[1], set[1], 2);
    Motor_SetSpeed(chassisMotor[2], set[2], 2);
    Motor_SetSpeed(chassisMotor[3], set[3], 2);

    osDelay(1);
  }
}
void chassis_task(void *argument) {
  vTaskDelay(623);

  while (chassisMotor[0] == NULL) {
    chassisMotor[0] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 0);
    vTaskDelay(10);
  }

  Motor_SetSpeedPID(chassisMotor[0], &chassisPid);


  for (;;) {
    if (sbus.channel[SBUS_SA_CHANNEL] > 0) {
      if (xTaskGetTickCount() - computerRxMsg.updateTime > 1500) {
        set[0] = (-0.5f * SQRT2 * translationSens * sbus.channel[2] - 0.5f * SQRT2 * translationSens * sbus.channel[0] +
                  rotationSens * sbus.channel[3]);
        set[1] = (0.5f * SQRT2 * translationSens * sbus.channel[2] - 0.5f * SQRT2 * translationSens * sbus.channel[0] +
                  rotationSens * sbus.channel[3]);
        set[2] = (0.5f * SQRT2 * translationSens * sbus.channel[2] + 0.5f * SQRT2 * translationSens * sbus.channel[0] +
                  rotationSens * sbus.channel[3]);
        set[3] = (-0.5f * SQRT2 * translationSens * sbus.channel[2] + 0.5f * SQRT2 * translationSens * sbus.channel[0] +
                  rotationSens * sbus.channel[3]);
      } else {
        set[0] = (-0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vx -
                  0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vy + rotationSens * computerRxMsg.instruct.wz);
        set[1] = (0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vx -
                  0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vy + rotationSens * computerRxMsg.instruct.wz);
        set[2] = (0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vx +
                  0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vy + rotationSens * computerRxMsg.instruct.wz);
        set[3] = (-0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vx +
                  0.5f * SQRT2 * translationSens * computerRxMsg.instruct.vy + rotationSens * computerRxMsg.instruct.wz);
      }
    } else {
      set[0] = set[1] = set[2] = set[3] = 0;
    }

    Motor_SetSpeed(chassisMotor[0], set[0], 2);


    osDelay(1);
  }
}
