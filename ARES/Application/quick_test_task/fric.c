#include "quick_test_task.h"
#include "motor.h"
#include "rm_motor.h"
#include "feedback_task.h"
Motor *testMotor[4];

#define HARDSOFT_THRESHOLD (600.0f)
#define FRICTIONGEAR_RADIUS (0.04f)
#define CLAMP_ANGLE (-2.0f)

uint8_t    touchFlag      = 0;
const fp32 error          = 0.51081562;
fp32       spc_ang        = 0;
fp32       friction0Speed = 0;
fp32       friction1Speed = 0;
fp32       shootSpeed     = 11.0f;

PID_ControllerParam gimbal_speed_pidparam = {.general  = PIDPOS_CONTROLLER,
                                            .Int_type = NORMAL_INT,
                                            .kP       = 250.0f,
                                            .kI       = 100.0f,
                                            .kD       = 0.0f,
                                            .kB       = 0.3f,
                                            .max_Iout = 5000,
                                            .N        = 100};

PID_ControllerParam gimbal_angle_pidparam = {.general  = PIDPOS_CONTROLLER,
                                            .Int_type = NORMAL_INT,
                                            .kP       = 900.0f,
                                            .kI       = 0.0f,
                                            .kD       = 40.0f,
                                            .kB       = 0.3f,
                                            .max_Iout = 5000,
                                            .N        = 50};


static fp32         ang;
static fp32         set;
PID_ControllerParam friction_speed_pidparam_soft = {.general  = PIDPOS_CONTROLLER,
                                                    .Int_type = NORMAL_INT,
                                                    .kP       = 12.0f,
                                                    .kI       = 3.0f,
                                                    .kD       = 0.0f,
                                                    .kB       = 0.03f,
                                                    .max_Iout = 5000,
                                                    .N        = 45};

PID_ControllerParam friction_speed_pidparam_hard = {.general  = PIDPOS_CONTROLLER,
                                                    .Int_type = NORMAL_INT,
                                                    .kP       = 30.0f,
                                                    .kI       = 1.0f,
                                                    .kD       = 0.0f,
                                                    .kB       = 0.03f,
                                                    .max_Iout = 5000,
                                                    .N        = 45};

PID_ControllerParam clamp_speed_pidparam = {.general  = PIDPOS_CONTROLLER,
                                            .Int_type = NORMAL_INT,
                                            .kP       = 6.0f,
                                            .kI       = 0.0f,
                                            .kD       = 0.0f,
                                            .kB       = 0.03f,
                                            .max_Iout = 5000,
                                            .N        = 45};

PID_ControllerParam clamp_angle_pidparam = {.general  = PIDPOS_CONTROLLER,
                                            .Int_type = BACK_CALCULATION_INT,
                                            .kP       = 10000.0f,
                                            .kI       = 0.0f,
                                            .kD       = 0.0f,
                                            .kB       = 0.3f,
                                            .max_Iout = 5000,
                                            .N        = 70};

static fp32       actualAngle;
static fp32       setAngle;
MotorInstructType clampMotorControl(Motor *motor, Controller *controller, void *param) {
  actualAngle = (2 * PI * motor->status.cumulative_turn + motor->status.angle - motor->status.zero) / M2006_REDUCTION_RATIO;
  fp32 rpm    = controllerUpdate((Controller *)&((RM_Motor *)motor)->angle_pid, &setAngle, &actualAngle, NULL);
  ((RM_Motor *)motor)->set_current =
      controllerUpdate((Controller *)&((RM_Motor *)motor)->speed_pid, &rpm, &motor->status.speed, NULL);
  return INSTRUCT_CURRENT;
}

MotorInstructType frictionGearControl(Motor *motor, Controller *controller, void *param) {
  if (touchFlag == 0) {
    if (__fabsf(motor->status.speed - *(fp32 *)param) > HARDSOFT_THRESHOLD) {
      ((RM_Motor *)motor)->speed_pid.general.param = &friction_speed_pidparam_hard;
      touchFlag                                    = 1;
    } else {
      ((RM_Motor *)motor)->speed_pid.general.param = &friction_speed_pidparam_soft;
    }
  } else {
    if (__fabsf(motor->status.speed - *(fp32 *)param) <= (HARDSOFT_THRESHOLD - 100.0f)) {
      ((RM_Motor *)motor)->speed_pid.general.param = &friction_speed_pidparam_soft;
      touchFlag                                    = 0;
    } else {
      ((RM_Motor *)motor)->speed_pid.general.param = &friction_speed_pidparam_hard;
    }
  }
  ((RM_Motor *)motor)->set_current =
      controllerUpdate((Controller *)&((RM_Motor *)motor)->speed_pid, param, &motor->status.speed, NULL);
  return INSTRUCT_CURRENT;
}

uint8_t zero     = 0;
int32_t timebase = 0;
void    quick_test_task() {
  vTaskDelay(105);
  while (testMotor[0] == NULL && testMotor[1] == NULL && testMotor[2] == NULL && testMotor[3] == NULL) {
    testMotor[0] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 4);
    testMotor[1] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 5);
    testMotor[2] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 6);
    testMotor[3] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 7);
    vTaskDelay(10);
  }
  
  feedback_register(&testMotor[0]->status.speed,6);
  feedback_register(&testMotor[1]->status.speed,7);
  
  Motor_SetSpeedPID(testMotor[3], &gimbal_speed_pidparam);
  Motor_SetAnglePID(testMotor[3], &gimbal_angle_pidparam);
  Motor_SetAnglePID(testMotor[2], &clamp_angle_pidparam);
  Motor_SetSpeedPID(testMotor[2], &clamp_speed_pidparam);
  Motor_SetAltController(testMotor[0], NULL, &friction0Speed, frictionGearControl);
  Motor_SetAltController(testMotor[1], NULL, &friction1Speed, frictionGearControl);
  Motor_SetAltController(testMotor[2], NULL, NULL, clampMotorControl);
  for (;;) {
    if (sbus.real.channel[SBUS_SA_CHANNEL] > 0.8f) {

      // 30度射箭
      if (sbus.real.channel[SBUS_SB_CHANNEL] == 1)
        spc_ang = PI / 6;
      else
        spc_ang = 0;
    

      if (zero) {
        // v1旋钮实现36度的微调
        set = -sbus.real.channel[SBUS_V1_CHANNEL] * PI / 10 + error + spc_ang;
        Motor_SetAngle(testMotor[3],set,200);
        // ang = (testMotor[3]->status.cumulative_turn * 2 * PI + testMotor[3]->status.angle - testMotor[3]->status.zero);
        // set = -sbus.real.channel[SBUS_V1_CHANNEL] * PI / 10 + ERROR;
        // fp32 speed = controllerUpdate((Controller *)&motor_angle_pid, &set, &ang, NULL);
        // Motor_SetSpeed(testMotor[3], speed, 200);

        friction0Speed = -shootSpeed * 60.0f / (2.0f * PI * FRICTIONGEAR_RADIUS);
        friction1Speed = shootSpeed * 60.0f / (2.0f * PI * FRICTIONGEAR_RADIUS);
        if (sbus.real.channel[SBUS_SD_CHANNEL] > 0.8f && timebase <= 0) {
          setAngle = CLAMP_ANGLE;
          Motor_AltControl(testMotor[2], 100);
          timebase = 10;
        } else if (timebase > 0) {
          setAngle = CLAMP_ANGLE;
          Motor_AltControl(testMotor[2], 100);
          timebase--;
        }

        else {
          setAngle = 0;
          Motor_AltControl(testMotor[2], 100);
        }
      } else {

        friction0Speed = friction1Speed = 0;
        Motor_SetCurrent(testMotor[2], 2000, 100);
        if (testMotor[2]->status.speed == 0) {
          timebase++;
          if (timebase > 10) {
            zero     = 1;
            timebase = 0;
            Motor_Zero(testMotor[2]);
          }
        }
      }

    } else {
      friction0Speed = friction1Speed = 0;
    }
    Motor_AltControl(testMotor[0], 100);
    Motor_AltControl(testMotor[1], 100);
    vTaskDelay(50);
  }
}
