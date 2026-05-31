/****************************************************************************************
 *     COPYRIGHT NOTICE
 *     Copyright (C) 2023,AS DAIMXA
 *     copyright Copyright (C) 呆萌侠DAIMXA,2023
 *     All rights reserved.
 *     技术讨论QQ群：710026750
 *
 *     除注明出处外，以下所有内容版权均属呆萌侠智能科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留呆萌智能侠科技的版权声明。
 *      ____    _    ___ __  ____  __    _    
 *     |  _ \  / \  |_ _|  \/  \ \/ /   / \   
 *     | | | |/ _ \  | || |\/| |\  /   / _ \  
 *     | |_| / ___ \ | || |  | |/  \  / ___ \ 
 *     |____/_/   \_\___|_|  |_/_/\_\/_/   \_\
 *
 * @file       dmx_isr.c
 * @brief      呆萌侠STC32G12K128开源库
 * @company    合肥呆萌侠智能科技有限公司
 * @author     呆萌侠科技（QQ：2453520483）
 * @MCUcore    STC32G12K128
 * @Software   Keil5 C251
 * @version    查看说明文档内version版本说明
 * @Taobao     https://daimxa.taobao.com/
 * @Openlib    https://gitee.com/daimxa
 * @date       2023-11-10
****************************************************************************************/

#include "dmx_all.h"
#include "dmx_angle.h"
#include "dmx_motor.h"   
#include "dmx_control.h"       
#include "xunji.h"
#include "collect.h"
float car_zero_angle =0.8;
int sat_vel_time_count = 0;
int ltla=0;
int ltlb=0;
int ltlc=0;
int ltld=1;
int ltlf=0;
int ltle=0;
//int ltlg=0;
int h=1;
int ltlcc=6;
float z1_angle=0;
int ltlaa=0;
int ltlbb=0;
int ltlccc=0;
int ltldd=0;
int ltlff=0;
//int ltlee=0;
int ltlgg=0;
//int ltlhh=0;
int ltlii=3;
int dd=0;
// 串级PID计算出的PWM值                                                   
int blance_pwm = 0;   
// 左右编码器值
int left_rear_speed,right_rear_speed;
// LED闪烁标志位
int led_time_count = 0;    
// 角速度定时器标志位
int angle_vel_time_count = 0;
// 角度定时器标志位
int angle_time_count = 0;                                                                                                                                   
// 速度环定时器标志位
int speed_time_count = 0;   
// 目标角速度
float target_angle_vel = 0;
// 目标角度
float target_angle = 0;
int target_speedy=60;
int target_speed[4]={60,5,4333,45};
int target_speed2=60;
float target_yaw_rate; 
//转向最终输出
float duty_steer=0;


float turn_time_count = 0;
int avg_speed=0;
//float grayscale_error =0;

//目标转弯
float target_zhijiao =90;
//float tingche_time_count = 0;
int tingche_flag = 0;
//float clean_time_count = 0;

//int i = 0;


// 外部中断0
void INT0_ISR(void) interrupt 0
{
    if(P32)	// 上升沿触发
    {

    }
    else		// 下降沿触发
    {
			
    }
}

// 外部中断1
void INT1_ISR(void) interrupt 2
{
    if(P33)	// 上升沿触发
    {

    }
    else		// 下降沿触发
    {

    }
}

// 外部中断2
void INT2_ISR(void) interrupt 10
{
    // 下降沿触发

}

// 外部中断3
void INT3_ISR(void) interrupt 11
{
    // 下降沿触发

}

// 外部中断4
void INT4_ISR(void) interrupt 16
{
    // 下降沿触发,P30作为RXD引脚时不可使用该中断

}
// 定时器0中断
void TIME0_ISR(void) interrupt 1
{
		// 编码器P34占用
}
// 定时器1中断
void TIME1_ISR(void) interrupt 3
{
	
	
	  // 中断计时50次(50*5ms = 250ms)LED电平翻转
		if(++led_time_count > 50)
		{
			P52 = !P52;
			led_time_count = 0;
		}
		
	  // 四元数解算欧拉角
		Get_Angles_ICM();

		// 角速度环,周期为5ms * 1 = 5ms
		if( ++angle_vel_time_count >= 1)
		{
			angle_vel_time_count = 0;
			if(P33==1)
			{ltle++;}
				if(ltle>=333)
				{
				P33=0;
				ltle=0;}
			// 计算最终输出PWM
			blance_pwm =  Realize_PID(&angle_vel, -(icm42688_gyro_y * 0.060976 * 10), target_angle_vel);
			// 电机输出

target_speed2=target_speed[0];
			Set_Left_Motor_Duty(blance_pwm-duty_steer);
			Set_Right_Motor_Duty(blance_pwm+duty_steer);
			if(P73==1)
				{
		if(
			(adc_values[1]==0||adc_values[6]==0)&&
		(adc_values[2]==0||adc_values[5]==0)
		&&(adc_values[1]==0||adc_values[2]==0)
		&&(adc_values[5]==0||adc_values[6]==0)
		&&(adc_values[3]==1||adc_values[4]==1)
		&&(adc_values[0]==1||adc_values[7]==1)
//		&&!(adc_values[3]==1&&adc_values[2]==1)
)	
				ltla++;
			
			if(						
		(adc_values[1]==0||adc_values[6]==0)&&
		(adc_values[2]==0||adc_values[5]==0)
		&&(adc_values[1]==0||adc_values[2]==0)
		&&(adc_values[5]==0||adc_values[6]==0)
		&&(adc_values[3]==1||adc_values[4]==1)
		&&(adc_values[0]==1||adc_values[7]==1)
//		&&!(adc_values[3]==1&&adc_values[2]==1)
			&&ltla>=ltlcc)
			{
				
			if(ltld++<=1)
			{P33=1;
			ltlc++;}
			if(ltlc==3)
			{
				while(1)
				{
				Set_Left_Motor_Duty(0);
			Set_Right_Motor_Duty(0);
				}
			}
	
		}
			
			if(ltlc<=3&&ltld>1)
				ltlf++;
			if(ltlf>=4000)
			{
			ltld=1;
			ltlf=0;

			}
			if(ltla<ltlcc)
			ltlb++;
				if(ltlb>=ltlcc+1)
				{
				ltla=0;
				ltlb=0;
				}	
				if(ltla>=ltlcc)
					ltla=0;
			
		}
			
		
		
		// 角度环,周期为5ms * 2 = 10ms
		if( ++angle_time_count >= 2)
		{
			angle_time_count = 0;
			// 计算目标角速度 
			target_angle_vel = Realize_PID(&angle, euler_angle.pitch, car_zero_angle- target_angle);
		}
					// 速度环,周期为5ms * 20 = 100ms
		if( ++speed_time_count >= 20)
		{
			speed_time_count = 0; 
			// 速度采集,需要加滤波
			left_rear_speed = Get_Left_Motor_Speed();
			right_rear_speed = Get_Right_Motor_Speed();
			
			avg_speed = (left_rear_speed + right_rear_speed)/2;
			target_angle = Speed_PID(&speed, avg_speed, target_speed[0]);
			
		}
		
				
					//偏航角速度环，周期为5ms * 1 = 5ms
		if ( ++turn_time_count >= 1)
		  {
				
			turn_time_count = 0;

			xun_xian();
			  
			  if(P72==1)
			  {
			zhijiao_angle();
			if( zhijiao_flag == 2&&ltlaa==0&&ltlbb==0&&ltlff==0&&h==1)
			{
				ltlccc++;
			
//					xunxian_zhijiao();
				
			}
						if( zhijiao_flag == 2&&ltlccc>=ltlii &&ltlaa==0&&ltlbb==0&&ltlff==0&&h==1)
			{
				ltlaa=1;ltlbb=0;
		P33=1;
			
//	xunxian_zhijiao();
				
			}
									if( adc_values[7]==1&&adc_values[6]==1&&adc_values[5]==1&&adc_values[4]==1&&adc_values[3]==1
							&&adc_values[2]==1&&adc_values[1]==1&&adc_values[0]==1&&ltlaa==0&&ltlbb==1&&dd>=300)
{ltlccc++;}
						if( adc_values[7]==1&&adc_values[6]==1&&adc_values[5]==1&&adc_values[4]==1&&adc_values[3]==1
							&&adc_values[2]==1&&adc_values[1]==1&&adc_values[0]==1&&ltlccc>=ltlii&&ltlaa==0&&ltlbb==1&&dd>=300)
			{
				ltlaa=1;ltlbb=1;
		P33=1;
			
//	xunxian_zhijiao();
				
			}
			

			
		



	if(ltlaa==1&&ltlbb==0)
	{ 
		z1_angle=z1_angle+icm42688_gyro_z*0.003;  //积分获取真实角度
	duty_steer=-Realize_PID(&zhijiao,z1_angle,target_zhijiao);
		
	}
	if(ltlaa==0&&ltlbb==1)
		dd++;
		if(ltlaa==0&&ltlbb==0)
		dd=0;

		if(ltlaa==1&&ltlbb==1)
	{ h=0;
		z1_angle=z1_angle+icm42688_gyro_z*0.003;  //积分获取真实角度
	duty_steer=-Realize_PID(&zhijiao,z1_angle,target_zhijiao);
	}

	if((ltlaa==0&&ltlbb==1)||(ltlaa==1&&ltlbb==0))
		h--;
if(ltlaa==1&&ltlbb==0&&z1_angle<=-75)
{
	ltlaa=0;
	ltlbb=1;
	z1_angle=0;
	ltlff=1;
	ltlgg=0;
}
if(ltlaa==1&&ltlbb==1&&z1_angle<=-75)
{
	ltlaa=0;
	ltlbb=0;
	z1_angle=0;
	ltlff=2;
}
if(ltlff==2&&ltlaa==0&&ltlbb==0)
{ltlgg++;
	target_speed[0]=target_speed2;

	
}
if(ltlgg>=target_speed[2])
{ltlff=0;
	h=1;
target_speed2=target_speed[0];
	
}


				if(ltlccc<ltlii)
			ltldd++;
				if(ltldd>=ltlii+1)
				{
				ltlccc=0;
				ltldd=0;
				}	
				if(ltlccc>=ltlii)
					ltlccc=0;
		  }
//		if(ltlff==2)
//			P33=1;

	  }
//		if(P72==0)  {    steering.p = -60.2; // 大了车身连着车轮会左右抖
//      steering.i = 0;
//      steering.d = -27; // 小了车抖，大了循迹差
//      //偏航角角速度参数
//      angle_hv.p = 58;  // 提高跟踪速度
//      angle_hv.i = 1.6; // 消除稳态误差
//      angle_hv.d = 19;  // 抑制超调和振荡
//			}
	  
	  }

}


// 定时器2中断
void TIME2_ISR(void) interrupt 12
{
	// 串口1占用
}

// 定时器3中断
void TIME3_ISR(void) interrupt 19
{
	// 编码器P04占用
}

// 定时器4中断
void TIME4_ISR(void) interrupt 20
{
  // 串口2占用
}


// 串口1中断
void UART1_ISR(void) interrupt 4
{
    if(TI)		// 获取串口1发送标志
    {
        TI = 0;		// 清理串口1发送标志
        TX_BUSY[1] = 0;
    }
    if(RI)		// 获取串口1接收标志
    {
        RI = 0;		// 清理串口1接收标志

    }
}

// 串口2中断
void UART2_ISR(void) interrupt 8
{
    if(S2TI)	// 获取串口2发送标志
    {
        S2TI = 0;	// 清理串口2发送标志
        TX_BUSY[2] = 0;
    }
    if(S2RI)	// 获取串口2接收标志
    {
        S2RI = 0;	// 清理串口2接收标志

    }
}

// 串口3中断
void UART3_ISR(void) interrupt 17
{
    if(S3TI)	// 获取串口3发送标志
    {
        S3TI = 0;	// 清理串口3发送标志
        TX_BUSY[3] = 0;
    }
    if(S3RI)	// 获取串口3接收标志
    {
        S3RI = 0;	// 清理串口3接收标志

    }
}

// 串口4中断
void UART4_ISR(void) interrupt 18
{
    if(S4TI)	// 获取串口4发送标志
    {
        S4TI = 0;	// 清理串口4发送标志
        TX_BUSY[4] = 0;
    }
    if(S4RI)	// 获取串口4接收标志
    {
        S4RI = 0;	// 清理串口4接收标志

    }
}

/**
*
* @brief    中断优先级设置
* @param    isr_nvic				中断名,根据isr.h中枚举查看
* @param    priority				优先级(0~3,3级为最高级)
* @return   void
* @notes    串口发送数据时,串口中断优先级要高于定时器中断优先级
* Example:  init_nvic(TIME1_PRIORITY,3);	// 定时器1中断优先级设置为最高级别3级
*
**/
void init_nvic(ISR_nvic_enum isr_nvic, unsigned char priority)
{
    switch(isr_nvic)
    {
    // 外部中断优先级分配
    case INT0_PRIORITY:
        switch(priority)
        {
        case 0:
            PX0H = 0;
            PX0 = 0;
            break;
        case 1:
            PX0H = 0;
            PX0 = 1;
            break;
        case 2:
            PX0H = 1;
            PX0 = 0;
            break;
        case 3:
            PX0H = 1;
            PX0 = 1;
            break;
        }
        break;
    case INT1_PRIORITY:
        switch(priority)
        {
        case 0:
            PX1H = 0;
            PX1 = 0;
            break;
        case 1:
            PX1H = 0;
            PX1 = 1;
            break;
        case 2:
            PX1H = 1;
            PX1 = 0;
            break;
        case 3:
            PX1H = 1;
            PX1 = 1;
            break;
        }
        break;
    case INT4_PRIORITY:
        switch(priority)
        {
        case 0:
            PX4H = 0;
            PX4 = 0;
            break;
        case 1:
            PX4H = 0;
            PX4 = 1;
            break;
        case 2:
            PX4H = 1;
            PX4 = 0;
            break;
        case 3:
            PX4H = 1;
            PX4 = 1;
            break;
        }
        break;
    // 定时器中断优先级分配
    case TIME0_PRIORITY:
        switch(priority)
        {
        case 0:
            PT0H = 0;
            PT0 = 0;
            break;
        case 1:
            PT0H = 0;
            PT0 = 1;
            break;
        case 2:
            PT0H = 1;
            PT0 = 0;
            break;
        case 3:
            PT0H = 1;
            PT0 = 1;
            break;
        }
        break;
    case TIME1_PRIORITY:
        switch(priority)
        {
        case 0:
            PT1H = 0;
            PT1 = 0;
            break;
        case 1:
            PT1H = 0;
            PT1 = 1;
            break;
        case 2:
            PT1H = 1;
            PT1 = 0;
            break;
        case 3:
            PT1H = 1;
            PT1 = 1;
            break;
        }
        break;
    // 串口中断优先级分配
    case UART1_PRIORITY:
        switch(priority)
        {
        case 0:
            PSH = 0;
            PS = 0;
            break;
        case 1:
            PSH = 0;
            PS = 1;
            break;
        case 2:
            PSH = 1;
            PS = 0;
            break;
        case 3:
            PSH = 1;
            PS = 1;
            break;
        }
        break;
    case UART2_PRIORITY:
        switch(priority)
        {
        case 0:
            PS2H = 0;
            PS2 = 0;
            break;
        case 1:
            PS2H = 0;
            PS2 = 1;
            break;
        case 2:
            PS2H = 1;
            PS2 = 0;
            break;
        case 3:
            PS2H = 1;
            PS2 = 1;
            break;
        }
        break;
    case UART3_PRIORITY:
        switch(priority)
        {
        case 0:
            PS3H = 0;
            PS3 = 0;
            break;
        case 1:
            PS3H = 0;
            PS3 = 1;
            break;
        case 2:
            PS3H = 1;
            PS3 = 0;
            break;
        case 3:
            PS3H = 1;
            PS3 = 1;
            break;
        }
        break;
    case UART4_PRIORITY:
        switch(priority)
        {
        case 0:
            PS4H = 0;
            PS4 = 0;
            break;
        case 1:
            PS4H = 0;
            PS4 = 1;
            break;
        case 2:
            PS4H = 1;
            PS4 = 0;
            break;
        case 3:
            PS4H = 1;
            PS4 = 1;
            break;
        }
        break;

    }
}
