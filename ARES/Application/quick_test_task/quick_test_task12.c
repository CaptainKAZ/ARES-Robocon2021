#include "controller.h"
#include "motor.h"
#include "cmsis_os.h"
#include "ADRC.h"
Motor *testMotor;
ADRC_Controller   adrc;
ADRC_ControllerParam adrcParam;

fp32 setAngle = PI;
static fp32                angle_O_Hlim      = 100;
static fp32                angle_O_Llim      = -100;
static fp32                angle_I_loop_Llim = 0;
static fp32                angle_I_loop_Hlim = 0;
static ControllerConstrain angle_constrain   = {.I_loop_Hlim = &angle_I_loop_Hlim,
                                                   .I_loop_Llim = &angle_I_loop_Llim,
                                                   .O_Hlim      = &angle_O_Hlim,
                                                   .O_Llim      = &angle_O_Llim};

MotorInstructType adrcContorl(Motor *motor, Controller *controller, void *param){
  motor->instruct.set = controllerUpdate(controller, &setAngle, &motor->status.angleOutput, NULL);
  return INSTRUCT_CURRENT;
}

void quick_test_task() { 
  vTaskDelay(2048);
  testMotor = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 7);
  ADRC_Init(&adrcParam);
  ADRC_ControllerInit(&adrc, &angle_constrain, &adrcParam,2);
  RM_Motor_setAsGm6020(testMotor);
  
  Motor_SetAltController(testMotor, (Controller*)&adrc, NULL, adrcContorl);
  for (;;){
    Motor_AltControl(testMotor, 2);
    //Motor_SetSpeed(testMotor,400,2);
    vTaskDelay(1);
  }
}
