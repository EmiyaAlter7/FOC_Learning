/*
 * motor.c
 *
 *  Created on: Mar 22, 2023
 *      Author: EmiyaAlter
 */

#include "motor.h"
#include "debug.h"
#include <math.h>
#include "fastSin.h"


MOTOR m1;


/*********************************************************************
 * @fn      TIM_PWM_Init
 *
 * @brief   Initializes TIM PWM OUT.
 *
 * @param   freq
 *          duty
 *          channel
 *
 * @return  none
 */
void PWM_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOE, &GPIO_InitStructure );

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);    //初始化IO

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOE, &GPIO_InitStructure );

    TIM_TimeBaseInitStructure.TIM_Period = 7200;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter=1;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);    //配置定时器模式


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 3600;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;
    TIM_OC1Init( TIM1, &TIM_OCInitStructure );
    TIM_OC2Init( TIM1, &TIM_OCInitStructure );
    TIM_OC3Init( TIM1, &TIM_OCInitStructure );     //配置比较输出通道123


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_Pulse = 7198;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;

    TIM_OC4Init( TIM1, &TIM_OCInitStructure );

   TIM_CtrlPWMOutputs(TIM1, ENABLE );          //高级定时器使能MOE
   TIM_OC1PreloadConfig( TIM1, TIM_OCPreload_Enable );
   TIM_OC2PreloadConfig( TIM1, TIM_OCPreload_Enable );
   TIM_OC3PreloadConfig( TIM1, TIM_OCPreload_Enable );
   TIM_OC4PreloadConfig( TIM1, TIM_OCPreload_Enable );    //允许预装载


    TIM_ARRPreloadConfig( TIM1, ENABLE );           


    TIM_Cmd( TIM1, ENABLE );       //启动定时器

}


//void ADC_Init(void)
//{
//    ADC_InitTypeDef ADC_InitStructure;
//    GPIO_InitTypeDef GPIO_InitStructure;
//
//
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1 , ENABLE ); //使能GPIOA时钟和ADC
//    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
//}


void Motor_Init(void)
{
    PWM_Init();
}


const float SQRT3DIV3 = 0.5773502691896257f;




void ipark(MOTOR *motorPtr)      //��park�任
{
    motorPtr->sine    = fast_sin(motorPtr->theta);
    motorPtr->cosine  = fast_cos(motorPtr->theta);
//    motorPtr->sine    = sin(motorPtr->theta);
//    motorPtr->cosine  = cos(motorPtr->theta);
    motorPtr->u_alpha = (motorPtr->u_d * motorPtr->cosine) - (motorPtr->u_q * motorPtr->sine);
    motorPtr->u_beta  = (motorPtr->u_q * motorPtr->cosine) + (motorPtr->u_d * motorPtr->sine);
}



void clarke(MOTOR *motorPtr)   // Clarke�任
{
    motorPtr->i_alpha = motorPtr->i_a;
    motorPtr->i_beta  = (motorPtr->i_a+2*motorPtr->i_b)*SQRT3DIV3;
}

void park(MOTOR *motorPtr)   //  park�任
{
    motorPtr->sine    = sin(motorPtr->theta);
    motorPtr->cosine  = cos(motorPtr->theta);
    motorPtr->i_d = motorPtr->i_alpha * motorPtr->cosine + motorPtr->i_beta  * motorPtr->sine;
    motorPtr->i_q = motorPtr->i_beta  * motorPtr->cosine - motorPtr->i_alpha * motorPtr->sine;
}

void svpwm(MOTOR *motorPtr)
{
    float u1 = motorPtr->u_beta;
    float u2 = (-0.8660254037844386f * motorPtr->u_alpha) - (0.5f * motorPtr->u_beta);
    float u3 = (0.8660254037844386f  * motorPtr->u_alpha) - (0.5f * motorPtr->u_beta);

    uint8_t sector = (u1 > 0.0f) + ((u2 > 0.0f) << 1) + ((u3 > 0.0f) << 2);


    switch (sector)
    {
    case 1:
    {
      float t2 = -u3;
      float t6 = -u2;
      float sum = t2 + t6;
      if (sum > 1.0f) {
        float k = 1.0f / sum;
        t2 = k * t2;
        t6 = k * t6;
      }
      float t7 = (1.0f - t2 - t6) / 2;
      motorPtr->t_a = t6 + t7;
      motorPtr->t_b = t2 + t6 + t7;
      motorPtr->t_c = t7;
    }
        break;
    case 2:
    {
      float t1 = -u1;
      float t3 = -u3;
      float sum = t1 + t3;
      if (sum > 1.0f) {
        float k = 1.0f / sum;
        t1 = k * t1;
        t3 = k * t3;
      }
      float t7 = (1.0f - t1 - t3) / 2;
      motorPtr->t_a = t7;
      motorPtr->t_b = t3 + t7;
      motorPtr->t_c = t1 + t3 + t7;
    }
        break;
    case 3:
    {
      float t2 = u1;
      float t3 = u2;
      float sum = t2 + t3;
      if (sum > 1.0f) {
        float k = 1.0f / sum;
        t2 = k * t2;
        t3 = k * t3;
      }
      float t7 = (1.0f - t2 - t3) / 2;
      motorPtr->t_a = t7;
      motorPtr->t_b = t2 + t3 + t7;
      motorPtr->t_c = t3 + t7;
    }
        break;
    case 4:
    {
      float t4 = -u2;
      float t5 = -u1;
      float sum = t4 + t5;
      if (sum > 1.0f) {
        float k = 1.0f / sum;
        t4 = k * t4;
        t5 = k * t5;
      }
      float t7 = (1.0f - t4 - t5) / 2;
      motorPtr->t_a = t4 + t5 + t7;
      motorPtr->t_b = t7;
      motorPtr->t_c = t5 + t7;
    }
        break;
    case 5:
    {
      float t4 = u3;
      float t6 = u1;
      float sum = t4 + t6;
      if (sum > 1.0f) {
        float k = 1.0f / sum;
        t4 = k * t4;
        t6 = k * t6;
      }
      float t7 = (1.0f - t4 - t6) / 2;
      motorPtr->t_a = t4 + t6 + t7;
      motorPtr->t_b = t6 + t7;
      motorPtr->t_c = t7;
    }
        break;
    case 6:
    {
      float t1 = u2;
      float t5 = u3;
      float sum = t1 + t5;
      if (sum > 1.0f) {
        float k = 1.0f / sum;
        t1 = k * t1;
        t5 = k * t5;
      }
      float t7 = (1.0f - t1 - t5) / 2;
      motorPtr->t_a = t5 + t7;
      motorPtr->t_b = t7;
      motorPtr->t_c = t1 + t5 + t7;
    }
        break;

    default:
        break;
    }
}


void Motor_Test(void)
{

    //GPIOE->BCR = GPIO_Pin_6;//拉低IO
    for (float theta = 0; theta < 6.283; theta += 0.01)
    {
        GPIOE->BCR = GPIO_Pin_6;//拉低IO
        m1.u_d = 0.6;
        m1.u_q = 0;
        m1.theta = theta;
        ipark(&m1);
        svpwm(&m1);

        TIM1->CH1CVR = (uint16_t)(m1.t_a * 1000);
        TIM1->CH2CVR = (uint16_t)(m1.t_b * 1000);
        TIM1->CH3CVR = (uint16_t)(m1.t_c * 1000);
        GPIOE->BSHR = GPIO_Pin_6;//拉高IO
        printf("%f,%f,%f\r\n", m1.t_a, m1.t_b, m1.t_c);
        //printf("%f,%f\r\n", sin(theta), fast_sin(theta));

    }
//    GPIOE->BSHR = GPIO_Pin_6;//拉高IO
    //Delay_Ms(100);
}
