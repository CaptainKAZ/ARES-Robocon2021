#include "quick_test_task.h"
#include "motor.h"

Motor* motor

void quick_test_task() {
  
  for (;;){
    //Mcp2515_tx(&frame);
    vTaskDelay(50);
  }
}
