/*
 * motor.h
 *
 *  Created on: Mar 22, 2023
 *      Author: EmiyaAlter
 */

#ifndef USER_DEVICES_INC_MOTOR_H_
#define USER_DEVICES_INC_MOTOR_H_


typedef struct
{
    /* data */

    float u_d;
    float u_q;
    float theta;

    float u_alpha;
    float u_beta;

    float t_a;
    float t_b;
    float t_c;

    float i_a;
    float i_b;
    float i_c;

    float i_alpha;
    float i_beta;

    float i_d;
    float i_q;


    float sine;
    float cosine;

}MOTOR;

extern MOTOR m1;

extern void ipark(MOTOR *motorPtr);
extern void clarke(MOTOR *motorPtr);
extern void park(MOTOR *motorPtr);
extern void svpwm(MOTOR *motorPtr);

extern void Motor_Init(void);
extern void Motor_Test(void);



#endif /* USER_DEVICES_INC_MOTOR_H_ */
