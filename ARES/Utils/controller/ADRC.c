#include "ADRC.h"
#include "user_lib.h"
#include "math.h"

#define ADRC ((ADRC_Controller *)self)
#define ADRCPARAM ((ADRC_ControllerParam *)(((Controller *)self)->param))



const float ADRC_Unit[3][16] = {
    /*TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合*/
    /*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/
    {50  , 0.001, 3, 30 , 200 ,500, 0.001, 0.002, 4.1, 1.6   , 5, 5, 0.8, 1.5, 50, 0},
    {300000, 0.001, 3, 300, 4000, 10000, 0.001, 0.002, 2.0, 0.0010, 5, 5, 0.8, 1.5, 50, 0},
    {300000, 0.001, 3, 300, 4000, 10000, 0.001, 0.002, 1.2, 0.0005, 5, 5, 0.8, 1.5, 50, 0},
};

float Constrain_Float(float amt, float low, float high) { return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))); }

int16_t Sign_ADRC(float Input) {
  int16_t output = 0;
  if (Input > 1E-6f)
    output = 1;
  else if (Input < -1E-6)
    output = -1;
  else
    output = 0;
  return output;
}

int16_t Fsg_ADRC(float x, float d) {
  int16_t output = 0;
  output         = (Sign_ADRC(x + d) - Sign_ADRC(x - d)) / 2;
  return output;
}

void ADRC_Init(ADRC_ControllerParam *fhan_Input) {
  fhan_Input->r       = ADRC_Unit[0][0];
  fhan_Input->h       = ADRC_Unit[0][1];
  fhan_Input->N0      = (uint16_t)(ADRC_Unit[0][2]);
  fhan_Input->beta_01 = ADRC_Unit[0][3];
  fhan_Input->beta_02 = ADRC_Unit[0][4];
  fhan_Input->beta_03 = ADRC_Unit[0][5];
  fhan_Input->b0      = ADRC_Unit[0][6];
  fhan_Input->beta_0  = ADRC_Unit[0][7];
  fhan_Input->beta_1  = ADRC_Unit[0][8];
  fhan_Input->beta_2  = ADRC_Unit[0][9];
  fhan_Input->N1      = (uint16_t)(ADRC_Unit[0][10]);
  fhan_Input->c       = ADRC_Unit[0][11];

  fhan_Input->alpha1 = ADRC_Unit[0][12];
  fhan_Input->alpha2 = ADRC_Unit[0][13];
  fhan_Input->zeta   = ADRC_Unit[0][14];
  fhan_Input->b      = ADRC_Unit[0][15];
  fhan_Input->general.type = ADRC_CONTROLLER;
}

void ADRC_Clear(ADRC_Controller *self) {
  /*****安排过度过程*******/
  self->x1 = self->x2 = 0;
  self->fh            = 0;

  /*****扩张状态观测器*******/
  /******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
  self->z1 = self->z2 = self->z3 = 0;
  self->e                        = 0;
  self->y                        = 0;
  self->fe = self->fe1 = 0;

  /**********系统状态误差反馈率*********/
  self->e0 = self->e1 = self->e2 = 0;
  self->u0 = self->u = 0;
}

void ADRC_ControllerSetParam(Controller *self, ControllerParam *param) {
  if (NULL == self || param == NULL) {
    return;
  }
  if (param->type == ADRC_CONTROLLER) {
    ADRC_Clear(ADRC);
    self->type  = param->type;
    self->param = param;
  }
}

//ADRC最速跟踪微分器TD，改进的算法fhan
void Fhan_ADRC(ADRC_Controller *fhan_Input, ADRC_ControllerParam *fhan_param, float expect_ADRC) //安排ADRC过度过程
{
  float d = 0, a0 = 0, y = 0, a1 = 0, a2 = 0, a = 0;
  float x1_delta = 0;                              //ADRC状态跟踪误差项
  x1_delta       = fhan_Input->x1 - expect_ADRC;   //用x1-v(k)替代x1得到离散更新公式
  fhan_param->h0 = fhan_param->N0 * fhan_param->h; //用h0替代h，解决最速跟踪微分器速度超调问题
  d              = fhan_param->r * fhan_param->h0 * fhan_param->h0; //d=rh^2;
  a0             = fhan_param->h0 * fhan_Input->x2;                 //a0=h*x2
  y              = x1_delta + a0;                                   //y=x1+a0
  a1             = sqrt(d * (d + 8 * ABS(y)));                      //a1=sqrt(d*(d+8*ABS(y))])
  a2             = a0 + Sign_ADRC(y) * (a1 - d) / 2;                //a2=a0+sign(y)*(a1-d)/2;
  a              = (a0 + y) * Fsg_ADRC(y, d) + a2 * (1 - Fsg_ADRC(y, d));
  fhan_Input->fh = -fhan_param->r * (a / d) * Fsg_ADRC(a, d) -
                   fhan_param->r * Sign_ADRC(a) * (1 - Fsg_ADRC(a, d)); //得到最速微分加速度跟踪量
  fhan_Input->x1 += fhan_param->h * fhan_Input->x2;                     //跟新最速跟踪状态量x1
  fhan_Input->x2 += fhan_param->h * fhan_Input->fh;                     //跟新最速跟踪状态量微分x2
}

//原点附近有连线性段的连续幂次函数
float Fal_ADRC(float e, float alpha, float zeta) {
  int16_t s          = 0;
  float   fal_output = 0;
  s                  = (Sign_ADRC(e + zeta) - Sign_ADRC(e - zeta)) / 2;
  fal_output         = e * s / (powf(zeta, 1 - alpha)) + powf(ABS(e), alpha) * Sign_ADRC(e) * (1 - s);
  return fal_output;
}

/************扩张状态观测器********************/
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
void ESO_ADRC(ADRC_Controller *fhan_Input, ADRC_ControllerParam *fhan_param) {
  fhan_Input->e = fhan_Input->z1 - fhan_Input->y; //状态误差

  fhan_Input->fe  = Fal_ADRC(fhan_Input->e, 0.5, fhan_param->h); //非线性函数，提取跟踪状态与当前状态误差
  fhan_Input->fe1 = Fal_ADRC(fhan_Input->e, 0.25, fhan_param->h);

  /*************扩展状态量更新**********/
  fhan_Input->z1 += fhan_param->h * (fhan_Input->z2 - fhan_param->beta_01 * fhan_Input->e);
  fhan_Input->z2 += fhan_param->h * (fhan_Input->z3 - fhan_param->beta_02 * fhan_Input->fe + fhan_param->b * fhan_Input->u);
  //ESO估计状态加速度信号，进行扰动补偿，传统MEMS陀螺仪漂移较大，估计会产生漂移
  fhan_Input->z3 += fhan_param->h * (-fhan_param->beta_03 * fhan_Input->fe1);
}

/************非线性组合****************/
/*
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float Sy=0,Sa=0;//ADRC状态跟踪误差项

  fhan_Input->h1=fhan_Input->N1*fhan_Input->h;

  d=fhan_Input->r*fhan_Input->h1*fhan_Input->h1;
  a0=fhan_Input->h1*fhan_Input->c*fhan_Input->e2;
  y=fhan_Input->e1+a0;
  a1=sqrt(d*(d+8*ABS(y)));
  a2=a0+Sign_ADRC(y)*(a1-d)/2;

  Sy=Fsg_ADRC(y,d);
  a=(a0+y-a2)*Sy+a2;
  Sa=Fsg_ADRC(a,d);
  fhan_Input->u0=-fhan_Input->r*((a/d)-Sign_ADRC(a))*Sa-fhan_Input->r*Sign_ADRC(a);

  //a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));

  //fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
  //                -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪量
}
*/
void Nolinear_Conbination_ADRC(ADRC_Controller *fhan_Input, ADRC_ControllerParam *fhan_param) {
  float temp_e2  = 0;
  temp_e2        = Constrain_Float(fhan_Input->e2, -3000, 3000);
  fhan_Input->u0 = fhan_param->beta_1 * Fal_ADRC(fhan_Input->e1, fhan_param->alpha1, fhan_param->zeta) +
                   fhan_param->beta_2 * Fal_ADRC(temp_e2, fhan_param->alpha2, fhan_param->zeta);
}

fp32 ADRC_ControllerUpdate(Controller *self, fp32 *set, fp32 *ref, fp32 *out) {
  //void ADRC_Control(Fhan_Data *fhan_Input, float expect_ADRC, float feedback_ADRC)
  float expect_ADRC   = *set;
  float feedback_ADRC = *ref;
  /*自抗扰控制器第1步*/
  /********
          **
          **
          **
          **
          **
      ********/
  /*****
        安排过度过程，输入为期望给定，
        由TD跟踪微分器得到：
        过度期望信号x1，过度期望微分信号x2
        ******/
  Fhan_ADRC(ADRC, ADRCPARAM, expect_ADRC);

  /*自抗扰控制器第2步*/
  /********
              *
              *
        ****
      *
      *
      ********/
  /************系统输出值为反馈量，状态反馈，ESO扩张状态观测器的输入*********/
  ADRC->y = feedback_ADRC;
  /*****
        扩张状态观测器，得到反馈信号的扩张状态：
        1、状态信号z1；
        2、状态速度信号z2；
        3、状态加速度信号z3。
        其中z1、z2用于作为状态反馈与TD微分跟踪器得到的x1,x2做差后，
        经过非线性函数映射，乘以beta系数后，
        组合得到未加入状态加速度估计扰动补偿的原始控制量u
        *********/
  ESO_ADRC(ADRC, ADRCPARAM); //低成本MEMS会产生漂移，扩展出来的z3此项会漂移，目前暂时未想到办法解决，未用到z3
                             /*自抗扰控制器第3步*/
                             /********
             **
          **
        **
          **
            **
      ********/
  /********状态误差反馈率***/
  ADRC->e0 += ADRC->e1 * ADRCPARAM->h; //状态积分项
  ADRC->e1 = ADRC->x1 - ADRC->z1;      //状态偏差项
  ADRC->e2 = ADRC->x2 - ADRC->z2;      //状态微分项，
  /********线性组合*******/
  
        ADRC->u0=ADRCPARAM->beta_0*ADRC->e0
                      +ADRCPARAM->beta_1*ADRC->e1
                      +ADRCPARAM->beta_2*ADRC->e2;
      
  //Nolinear_Conbination_ADRC(ADRC, ADRCPARAM);
  /**********扰动补偿*******/
  ADRC->u=ADRC->u0
               -ADRC->z3/ADRCPARAM->b0;    
  //由于MEMS传感器漂移比较严重，当beta_03取值比较大时，长时间z3漂移比较大，目前不加入扰动补偿控制量
  //ADRC->u = Constrain_Float(ADRC->u0, -200, 200);
  return ADRC->u;
}

void ADRC_ControllerInit(ADRC_Controller *self, ControllerConstrain *constrain, ADRC_ControllerParam *param, fp32 timeout) {
  if (NULL == self) {
    return;
  }
  if (param->general.type != ADRC_CONTROLLER) {
    return;
  }
  self->general.type      = param->general.type;
  self->general.constrain = constrain;
  self->general.param     = (ControllerParam *)param;
  self->general.I_size = self->general.O_size = 1;
  self->timeout                               = timeout;
  ADRC_Clear(self);
}
