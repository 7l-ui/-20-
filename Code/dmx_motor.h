#ifndef _DMX_MOTOR_H_
#define _DMX_MOTOR_H_

// 电机PWM引脚
#define LEFT_MOTOR_ONE 			PWMA2_P_P62
#define LEFT_MOTOR_TWO 			PWMA1_P_P60
#define RIGHT_MOTOR_ONE   	PWMA3_P_P64
#define RIGHT_MOTOR_TWO   	PWMA4_P_P66

// 电机频率
#define MOTOR_FREQ          3000
// 电机限幅
#define MOTOR_A_DUTY        10000
// 电机限幅
#define MOTOR_B_DUTY        10000
// 电机死区
#define MOTOR_DEAD        	1000

/**
*
* @brief    电机初始化
* @param
* @return   void
* @notes
* Example:  Init_Motor();
*
**/
void Init_Motor(void);

/**
*
* @brief    发送左电机占空比
* @param    duty            占空比大小,负值反转
* @return   void
* @notes
* Example:  Set_Left_Motor_Duty(1500);
*
**/
void Set_Left_Motor_Duty(int duty);

/**
*
* @brief    发送右电机占空比
* @param    duty            占空比大小,负值反转
* @return   void
* @notes
* Example:  Set_Right_Motor_Duty(1500);
*
**/
void Set_Right_Motor_Duty(int duty);

/**
*
* @brief    获取左电机速度
* @param
* @return   void
* @notes
* Example:  Get_Left_Motor_Speed();
*
**/
int Get_Left_Motor_Speed(void);

/**
*
* @brief    获取右电机速度
* @param
* @return   void
* @notes
* Example:  Get_Right_Motor_Speed();
*
**/
int Get_Right_Motor_Speed(void);

#endif