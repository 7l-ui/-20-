#include "dmx_all.h"
#include "dmx_motor.h"

/**
*
* @brief    电机初始化
* @param
* @return   void
* @notes
* Example:  Init_Motor();
*
**/
void Init_Motor(void)
{

    // 左电机控制引脚
		// 初始化左轮PWM,频率为MOTOR_FREQ,初始占空比为0%
		init_pwm(LEFT_MOTOR_ONE,MOTOR_FREQ,0);
		init_pwm(LEFT_MOTOR_TWO,MOTOR_FREQ,0);
		
    // 右电机控制引脚
		// 初始化右轮PWM,频率为MOTOR_FREQ,初始占空比为0%
		init_pwm(RIGHT_MOTOR_ONE,MOTOR_FREQ,0);
		init_pwm(RIGHT_MOTOR_TWO,MOTOR_FREQ,0);
		
		// 初始化编码器 
		// 脉冲数捕捉引脚为ENCODER3_P04,方向引脚为P53
		init_encoder(ENCODER3_P04,GPIO_P53);
		// 脉冲数捕捉引脚为ENCODER0_P34,方向引脚为P35
		init_encoder(ENCODER0_P34,GPIO_P35);
		
}

/**
*
* @brief    发送左电机占空比
* @param    duty            占空比大小,负值反转
* @return   void
* @notes
* Example:  Set_Left_Motor_Duty(1500);
*
**/
void Set_Left_Motor_Duty(int duty)
{
    if(duty >=0)
    {
			  duty = duty + MOTOR_DEAD;
			  // PWM限幅
        if(duty > MOTOR_A_DUTY)
            duty = MOTOR_A_DUTY;

				// 左轮正转
        set_duty_pwm(LEFT_MOTOR_ONE, duty);
        set_duty_pwm(LEFT_MOTOR_TWO, 0);
    }
    else
    {
			   duty = duty - MOTOR_DEAD;
			  // PWM限幅
        if(duty < -MOTOR_B_DUTY)
            duty = -MOTOR_B_DUTY;

				// 左轮反转
        set_duty_pwm(LEFT_MOTOR_ONE, 0);
        set_duty_pwm(LEFT_MOTOR_TWO, -duty);
    }
}

/**
*
* @brief    发送右电机占空比
* @param    duty            占空比大小,负值反转
* @return   void
* @notes
* Example:  Set_Right_Motor_Duty(1500);
*
**/
void Set_Right_Motor_Duty(int duty)
{
    if(duty >=0)
    {
			  duty = duty + MOTOR_DEAD;
			  // PWM限幅
        if(duty > MOTOR_A_DUTY)
            duty = MOTOR_A_DUTY;

				// 右轮正转
        set_duty_pwm(RIGHT_MOTOR_ONE, duty);
        set_duty_pwm(RIGHT_MOTOR_TWO, 0);
    }
    else
    {
			   duty = duty - MOTOR_DEAD;
			  // PWM限幅
        if(duty < -MOTOR_B_DUTY)
            duty = -MOTOR_B_DUTY;

				// 右轮正转
        set_duty_pwm(RIGHT_MOTOR_ONE, 0);
        set_duty_pwm(RIGHT_MOTOR_TWO, -duty);
    }
}

/**
*
* @brief    获取左电机速度
* @param
* @return   void
* @notes
* Example:  Get_Left_Motor_Speed();
*
**/
int Get_Left_Motor_Speed(void)
{
		int left_speed = (int)get_encoder_count(ENCODER3_P04);
		// 清除编码器计数值
		clean_encoder_count(ENCODER3_P04);
		if(!P53) left_speed = -left_speed;
    return left_speed;
}

/**
*
* @brief    获取右电机速度
* @param
* @return   void
* @notes
* Example:  Get_Right_Motor_Speed();
*
**/
int Get_Right_Motor_Speed(void)
{
		int	right_speed = (int)get_encoder_count(ENCODER0_P34);
		// 清除编码器计数值
		clean_encoder_count(ENCODER0_P34);
		if(P35) right_speed = -right_speed;
    return right_speed;
}