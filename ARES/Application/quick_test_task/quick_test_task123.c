#include "quick_test_task.h"
#include "servo.h"
#include "RMD_motor.h"

fp32 fetchPoint[3] = {0};

Motor *FetchMotor[3];
Motor *ShootMotor[2];
Motor *GimbalMotor;

void quick_test_task(){
  vTaskDelay(2000);
  GimbalMotor=CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 7);
  ShootMotor[0] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 1);
  ShootMotor[1] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 1);
  FetchMotor[0] = CAN_Find_Motor(RM_MOTOR,INTERNAL_CAN2,1);
  FetchMotor[1] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, );
  FetchMotor[2] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, );

  


  while(1)
    vTaskDelay(2);
}
