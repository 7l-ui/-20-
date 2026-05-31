
#include "dmx_all.h"
#include "dmx_angle.h" 
#include "dmx_motor.h"
#include "dmx_control.h"
#include "xunji.h"
#include "collect.h"
/**	！！！！！！！！
* 该开源库单片机主频为软件强制设置内部IRC主频,STC-ISP中所选无效
* 如需修改单片机主频应去dmx_board.h中修改MAIN_FOSC宏定义
！！！！！！！！**/

/** ！！！！！！！！
* 简单的串级PID平衡车直立演示程序(无速度环和转向环)
！！！！！！！！**/

#define BUZZER 	P33

// AD1
#define AD1	ADC3_P13
// AD2
#define AD2	ADC7_P17
// AD3
#define AD3	ADC6_P16
// AD4
#define AD4	ADC5_P15
// AD5
#define AD5	ADC4_P14
// AD6
#define AD6	ADC1_P11
// AD7
#define AD7	ADC0_P10
// AD8
#define AD8	ADC14_P06

#define SWITCH1     P73
#define SWITCH2     P72

#define KEY_UP      P70
#define KEY_DOWN    P71
#define KEY_LEFT    P36
#define KEY_RIGHT   P37
#define KEY_MID     P51

char buff[200];
void menu_loop()
{int i=0;
    while (1)
    {
                Show_Int_IPS(45,1*16,target_speed[0],4,BLACK,WHITE,Show8x16);
                Show_Int_IPS(45,2*16,target_speed[1],4,BLACK,WHITE,Show8x16);
                Show_Int_IPS(45,3*16,target_speed[2],4,BLACK,WHITE,Show8x16);
		                Show_Int_IPS(45,4*16,target_speed[3],4,BLACK,WHITE,Show8x16);

		                Show_Int_IPS(45,5*16,i,4,BLACK,WHITE,Show8x16);

        if (KEY_UP==0)
        {
			if(i!=2)
            target_speed[i] += 1;
			if(i==2)
				target_speed[i] += 333;
			  delay_ms(90); // 防止重复按下
        }
		

       
		 if (KEY_DOWN==0)
        {
      if(i!=2)
            target_speed[i] -= 1;
			if(i==2)
				target_speed[i] -= 333;
			  delay_ms(90); // 防止重复按下
        }
		  if (KEY_LEFT==0)
        {
            i--;
			if(i<0)
				i=3;


			  delay_ms(90); // 防止重复按下
        }
		

        if (KEY_RIGHT==0)
        {
            i++;
			if(i>3)
				i=0;
			  delay_ms(90); // 防止重复按下
        }

        if (KEY_MID==0)
        {
            Show_String_IPS(2,8*16,"0k",BLACK,WHITE,Show8x16);
            delay_ms(60); // 显示确认信息1秒
                    Clean_IPS(WHITE);
            break;
    }
}
	}
void menu_loop2()
{int q=0;
    while (1)
    {
		ad();
			Show_String_IPS(2,0*16,"AD1:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(35,0*16,get_adc(AD1, ADC_12BIT),4,BLACK,WHITE,Show8x16);
			// 灰度传感器2
			Show_String_IPS(2,1*16,"AD2:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(35,1*16,get_adc(AD2, ADC_12BIT),4,BLACK,WHITE,Show8x16);
			// 灰度传感器3
			Show_String_IPS(2,2*16,"AD3:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(35,2*16,get_adc(AD3, ADC_12BIT),4,BLACK,WHITE,Show8x16);
			// 灰度传感器4
			Show_String_IPS(2,3*16,"AD4:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(35,3*16,get_adc(AD4, ADC_12BIT),4,BLACK,WHITE,Show8x16);
			// 灰度传感器5
			Show_String_IPS(2,4*16,"AD5:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(35,4*16,get_adc(AD5, ADC_12BIT),4,BLACK,WHITE,Show8x16);
			// 灰度传感器6
			Show_String_IPS(2,5*16,"AD6:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(35,5*16,get_adc(AD6, ADC_12BIT),4,BLACK,WHITE,Show8x16);
			// 灰度传感器7
			Show_String_IPS(2,6*16,"AD7:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(35,6*16,get_adc(AD7, ADC_12BIT),4,BLACK,WHITE,Show8x16);
			// 灰度传感器8
			Show_String_IPS(2,7*16,"AD8:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(35,7*16,get_adc(AD8, ADC_12BIT),4,BLACK,WHITE,Show8x16);
			
			Show_Int_IPS(100,0*16,hd[0],4,BLACK,WHITE,Show8x16);
			// 灰度传感器2
			Show_Int_IPS(100,1*16,hd[1],4,BLACK,WHITE,Show8x16);
			// 灰度传感器3
			Show_Int_IPS(100,2*16,hd[2],4,BLACK,WHITE,Show8x16);
			// 灰度传感器4
			Show_Int_IPS(100,3*16,hd[3],4,BLACK,WHITE,Show8x16);
			// 灰度传感器5
			Show_Int_IPS(100,4*16,hd[4],4,BLACK,WHITE,Show8x16);
			// 灰度传感器6
			Show_Int_IPS(100,5*16,hd[5],4,BLACK,WHITE,Show8x16);
			// 灰度传感器7
			Show_Int_IPS(100,6*16,hd[6],4,BLACK,WHITE,Show8x16);
			// 灰度传感器8
			Show_Int_IPS(100,7*16,hd[7],4,BLACK,WHITE,Show8x16);
			
			Show_Int_IPS(140,0*16,adc_values[0],1,BLACK,WHITE,Show8x16);
			// 灰度传感器2
			Show_Int_IPS(140,1*16,adc_values[1],1,BLACK,WHITE,Show8x16);
			// 灰度传感器3
			Show_Int_IPS(140,2*16,adc_values[2],1,BLACK,WHITE,Show8x16);
			// 灰度传感器4
			Show_Int_IPS(140,3*16,adc_values[3],1,BLACK,WHITE,Show8x16);
			// 灰度传感器5
			Show_Int_IPS(140,4*16,adc_values[4],1,BLACK,WHITE,Show8x16);
			// 灰度传感器6
			Show_Int_IPS(140,5*16,adc_values[5],1,BLACK,WHITE,Show8x16);
			// 灰度传感器7
			Show_Int_IPS(140,6*16,adc_values[6],1,BLACK,WHITE,Show8x16);
			// 灰度传感器8
			Show_Int_IPS(140,7*16,adc_values[7],1,BLACK,WHITE,Show8x16);
	
			
			Show_Int_IPS(180,1*16,q+1,1,BLACK,WHITE,Show8x16);

			
        if (KEY_UP==0)
        {
            hd[q]+=100;
			  delay_ms(40); // 防止重复按下
        }
		

        if (KEY_DOWN==0)
        {
            hd[q]-=100;
			  delay_ms(60); // 防止重复按下
        }
		  if (KEY_LEFT==0)
        {
            q--;
			if(q<0)
				q=7;


			  delay_ms(60); // 防止重复按下
        }
		

        if (KEY_RIGHT==0)
        {
            q++;
			if(q>7)
				q=0;
			  delay_ms(60); // 防止重复按下
        }

        if (KEY_MID==0)
        {
            Show_String_IPS(2,8*16,"0k",BLACK,WHITE,Show8x16);
            delay_ms(300); // 显示确认信息1秒
                    Clean_IPS(WHITE);
            break;
        }
    }
}
	
void main(void)
{
		// 初始化芯片
    init_chip();	
//	write_eeprom(0x00,pid_date,41);
//	read_eeprom(0x00,pid_date,41);//按照字节读取数据
	  // 以下可放置用户代码段
//	pid_read();
	  // P33引脚作为输出则需要初始化为推挽输出
		// 初始化P33引脚蜂鸣器,推挽输出,上拉禁止,注意:此处的0不代表低电平,代表上拉禁止!
		init_gpio(GPIO_P33,OUT_PP ,0);
		
	  // 高电平,鸣叫
		BUZZER = 1;
	
	  // 初始化串口4,利用定时器4,RXD引脚为P02,TX引脚为P03,波特率为115200(插入无线转串口模块即可)
	  init_uart(UART4,PIT4,UART4_RX_P02_TX_P03,115200);
		// 发送"UART4 TEST!\n"字符串到上位机 
		send_string_uart(UART4,"UART4 TEST!\n");
		//ADC初始化
		init_adc(AD1,ADC_CLK_2);
		init_adc(AD2,ADC_CLK_2);
		init_adc(AD3,ADC_CLK_2);
		init_adc(AD4,ADC_CLK_2);
		init_adc(AD5,ADC_CLK_2);
		init_adc(AD6,ADC_CLK_2);
		init_adc(AD7,ADC_CLK_2);
		init_adc(AD8,ADC_CLK_2);
		// 初始化IPS屏幕,默认硬件SPI驱动,型号IPS114,横屏,可在dmx_ips.h文件中进行选择
		Init_IPS();

		// 初始化ICM42688陀螺仪
		// 如想使用陀螺仪原始数据,Init_ICM42688();中注释设置陀螺仪低通滤波器带宽和量程的函数
		Init_ICM42688();
	
		// 陀螺仪去零飘初始化
		Init_Gyro_Offset();
	
	  // 电机初始化
	  Init_Motor();
	
	  // PID初始化
	  Init_PID();
		
		// 低电平,不鸣叫
		BUZZER = 0;
		
	target_speedy=target_speed[0];
	menu_loop();
	        zhijiao.p=target_speed[3];
	target_speed2=target_speed[0]+target_speed[1];
	menu_loop2();
	
	// 定时器中断1初始化,中断时间为5ms
	init_pit_ms(PIT1,3);



    while(1)
    {
//			
//			zhijiao_gocome();
//			ui();
					// 灰度传感器1
			Show_String_IPS(2,3*16,"AD1:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(30,3*16,adc_values[0],4,BLACK,WHITE,Show8x16);
//			// 灰度传感器2
			Show_String_IPS(2,4*16,"AD2:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(30,4*16,adc_values[1],4,BLACK,WHITE,Show8x16);
//			// 灰度传感器3
			Show_String_IPS(2,5*16,"AD3:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(30,5*16,adc_values[2],4,BLACK,WHITE,Show8x16);
//			// 灰度传感器4
			Show_String_IPS(2,6*16,"AD4:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(30,6*16,adc_values[3],4,BLACK,WHITE,Show8x16);
			// 灰度传感器5
			Show_String_IPS(100,3*16,"AD5:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(130,3*16,adc_values[4],4,BLACK,WHITE,Show8x16);
//			// 灰度传感器6
			Show_String_IPS(100,4*16,"AD6:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(130,4*16,adc_values[5],4,BLACK,WHITE,Show8x16);
//			// 灰度传感器7
			Show_String_IPS(100,5*16,"AD7:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(130,5*16,adc_values[6],4,BLACK,WHITE,Show8x16);
//			// 灰度传感器8
			Show_String_IPS(100,6*16,"AD8:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(130,6*16,adc_values[7],4,BLACK,WHITE,Show8x16);
			Show_Int_IPS(145,7*16,ltlc,4,BLACK,WHITE,Show8x16);
			// 显示俯仰角数据
			// 角度解算在定时器中断1函数里调用
			Show_String_IPS(2,0*16,"PITCH-ANGLE:",BLACK,WHITE,Show8x16);
			Show_Float_IPS(100,0*16,euler_angle.pitch,3,2,BLACK,WHITE,Show8x16);
			
			// 显示左右轮编码器数据
			Show_String_IPS(2,1*16,"LEFT-SPEED:",BLACK,WHITE,Show8x16);
			Show_Float_IPS(100,1*16,left_rear_speed,4,0,BLACK,WHITE,Show8x16);
			Show_String_IPS(2,2*16,"RIGHT-SPEED:",BLACK,WHITE,Show8x16);
			Show_Float_IPS(100,2*16,right_rear_speed,4,0,BLACK,WHITE,Show8x16);
			//			//			// 灰度传感器1
//			Show_String_IPS(2,3*16,"AD1:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(50,3*16,get_adc(AD1, ADC_12BIT),4,BLACK,WHITE,Show8x16);
//			// 灰度传感器2
//			Show_String_IPS(2,4*16,"AD2:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(50,4*16,get_adc(AD2, ADC_12BIT),4,BLACK,WHITE,Show8x16);
//			// 灰度传感器3
//			Show_String_IPS(2,5*16,"AD3:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(50,5*16,get_adc(AD3, ADC_12BIT),4,BLACK,WHITE,Show8x16);
//			// 灰度传感器4
//			Show_String_IPS(2,6*16,"AD4:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(50,6*16,get_adc(AD4, ADC_12BIT),4,BLACK,WHITE,Show8x16);
//			// 灰度传感器5
//			Show_String_IPS(90,3*16,"AD5:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(150,3*16,get_adc(AD5, ADC_12BIT),4,BLACK,WHITE,Show8x16);
//			// 灰度传感器6
//			Show_String_IPS(90,4*16,"AD6:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(150,4*16,get_adc(AD6, ADC_12BIT),4,BLACK,WHITE,Show8x16);
//			// 灰度传感器7
//			Show_String_IPS(90,5*16,"AD7:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(150,5*16,get_adc(AD7, ADC_12BIT),4,BLACK,WHITE,Show8x16);
//			// 灰度传感器8
//			Show_String_IPS(90,6*16,"AD8:",BLACK,WHITE,Show8x16);
			Show_Int_IPS(150,6*16,get_adc(AD8, ADC_12BIT),4,BLACK,WHITE,Show8x16);
//			// 变量转字符串
//			//真实角速度，目标角速度，真实角度，目标角度，真实速度，目标速度
//			sprintf(buff,"%f,%f,%f,%f,%f,%f\n",-(icm42688_gyro_y * 0.060976 * 10),target_angle_vel, car_zero_angle - target_angle/100,euler_angle.pitch,(left_rear_speed + right_rear_speed) / 2,target_speed);
//			sprintf(buff,"%d,%f,%f\n",zhijiao_flag,z_angle,icm42688_gyro_z);
			sprintf(buff, "%f,%f,%f\n", target_yaw_rate, icm42688_gyro_z, grayscale_error);
			//			sprintf(buff,"%f,%f,%f\n",speed.integral,icm42688_gyro_z,duty_steer);
			// sprintf(buff,"%d\n",zhijiao_flag);
			//			// 串口发送该字符串
			send_string_uart(UART4,buff);
			
    }
}