#include "quick_test_task.h"
#include "RM_motor.h"
#include "feedback_task.h"

Motor *test_motor[6];
fp32 currentctrl=1000;


void quick_test_task() {
  vTaskDelay(1024);
  
    for(uint8_t i=0 ; i<6;i++){
    test_motor[i]  = CAN_Find_Motor(RM_MOTOR,EXTERNAL_CAN1,i);
    }

  for (;;) {
    for(uint8_t i=0 ; i<6;i++){
    Motor_SetCurrent(test_motor[i], currentctrl, 2);
    }
    vTaskDelay(1);
  }
}
