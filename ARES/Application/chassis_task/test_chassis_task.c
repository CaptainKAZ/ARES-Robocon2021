#include "motor.h"
#include "sbus.h"
#include "cmsis_os.h"
#include "main.h"

#define SQRT2 1.4142135623730950488016887242097f

static Motor *chassisMotor[4];
static fp32   translationSens = 5;
static fp32   rotationSens    = 2;

static PID_ControllerParam chassisPid = {.general  = PIDPOS_CONTROLLER,
                                         .Int_type = BACK_CALCULATION_INT,
                                         .kB       = 0.2,
                                         .kD       = 0.1,
                                         .kI       = 10,
                                         .kP       = 15,
                                         .max_Iout = 16000,
                                         .N        = 30};

void chassis_task(void *argument) {
  vTaskDelay(623);
  while (chassisMotor[0] == NULL || chassisMotor[1] == NULL || chassisMotor[2] == NULL || chassisMotor[3] == NULL) {
    chassisMotor[0] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 0);
    chassisMotor[1] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 1);
    chassisMotor[2] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 2);
    chassisMotor[3] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 3);
  }
  Motor_SetSpeedPID(chassisMotor[0], &chassisPid);
  Motor_SetSpeedPID(chassisMotor[1], &chassisPid);
  Motor_SetSpeedPID(chassisMotor[2], &chassisPid);
  Motor_SetSpeedPID(chassisMotor[3], &chassisPid);
  for (;;) {
    Motor_SetSpeed(chassisMotor[0],
                   (-0.5f * SQRT2 * translationSens * sbus.channel[0] - 0.5f * SQRT2 * translationSens * sbus.channel[1] -
                    rotationSens * sbus.channel[2]),
                   2);
    Motor_SetSpeed(chassisMotor[1],
                   (0.5f * SQRT2 * translationSens * sbus.channel[0] - 0.5f * SQRT2 * translationSens * sbus.channel[1] -
                    rotationSens * sbus.channel[2]),
                   2);
    Motor_SetSpeed(chassisMotor[2],
                   (0.5f * SQRT2 * translationSens * sbus.channel[0] + 0.5f * SQRT2 * translationSens * sbus.channel[1] -
                    rotationSens * sbus.channel[2]),
                   2);
    Motor_SetSpeed(chassisMotor[3],
                   (-0.5f * SQRT2 * translationSens * sbus.channel[0] + 0.5f * SQRT2 * translationSens * sbus.channel[1] -
                    rotationSens * sbus.channel[2]),
                   2);
    osDelay(1);
  }
}
