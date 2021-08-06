#include "quick_test_task.h"
#include "RM_motor.h"
#include "feedback_task.h"
#include "servo.h"


void quick_test_task() {
  vTaskDelay(2048);

  Servo_getExistNonBlocking();
  vTaskDelay(10);
  for (;;) {
    for (uint8_t i = 0; i < 10;i++){
      if(servo.exist[i]){
        Servo_setPos(i,(++servo.expectedPos[i]) % 4096);
        vTaskDelay(1);
      }
    }
    vTaskDelay(1);
  }
}
