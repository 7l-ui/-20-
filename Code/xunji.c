#include "dmx_all.h"
#include "dmx_motor.h"
#include "dmx_control.h"
#include "xunji.h"
#include "collect.h"
/**	！！！！！！！！
* 该开源库单片机主频为软件强制设置内部IRC主频,STC-ISP中所选无效
* 如需修改单片机主频应去dmx_board.h中修改MAIN_FOSC宏定义
！！！！！！！！**/

/** ！！！！！！！！
* IPS114显示屏显示8路灰度传感器ADC数据
！！！！！！！！**/
float z_angle = 0; //积分的角度
int zhijiao_flag = 0; //直角标志位
int hd[8]={900,900,1100,700,1100,1200,2000,2000};


int out_flag = 0; //出近道标志位
//PID_struct zhijiao; //设置直角循迹结构体重命名


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

#define BUZZER 	P33

// 灰度传感器的数值转化为0和1
unsigned int adc_values[8];
void ad()
{
    int i;
	
	adc_values[0] = get_adc(AD1, ADC_12BIT);
    adc_values[1] = get_adc(AD2, ADC_12BIT);
    adc_values[2] = get_adc(AD3, ADC_12BIT);
    adc_values[3] = get_adc(AD4, ADC_12BIT);
	adc_values[4] = get_adc(AD5, ADC_12BIT);
    adc_values[5] = get_adc(AD6, ADC_12BIT);
    adc_values[6] = get_adc(AD7, ADC_12BIT);
    adc_values[7] = get_adc(AD8, ADC_12BIT);
   if(adc_values[0]<hd[0])
			{adc_values[0]=0;}
				else
			{adc_values[0]=1;}
			if(adc_values[1]<hd[1])
			{adc_values[1]=0;}
				else
			{adc_values[1]=1;}
			if(adc_values[2]<hd[2])
			{adc_values[2]=0;}
				else
			{adc_values[2]=1;}
			if(adc_values[3]<hd[3])
			{adc_values[3]=0;}
				else
			{adc_values[3]=1;}
			if(adc_values[4]<hd[4])
			{adc_values[4]=0;}
				else
			{adc_values[4]=1;}
			if(adc_values[5]<hd[5])
			{adc_values[5]=0;}
				else
			{adc_values[5]=1;}
			if(adc_values[6]<hd[6])
			{adc_values[6]=0;}
				else
			{adc_values[6]=1;}
			if(adc_values[7]<hd[7])
			{adc_values[7]=0;}
				else
			{adc_values[7]=1;}
//for(i=0;i++;i<=7)
//   if(adc_values[i]<1500)
//   {adc_values[i]=0;}
//   	else
//			{adc_values[i]=1;}
}


int gray_state = 0x0;
int gray_status[7]={0};
float error_hv = 0;
int gray_status_worse=0;
//将8个数字传感器（如开关量）的状态合并为16进制
int Sensor_GetState(void)
{
	int i=0;
	unsigned long int State = 0X0000000;
	ad();
	for(i=0;i<8;i++)
	State |= (adc_values[7-i] << i);
	return State;
}



//权重法循迹
float Get_Grayscale_Error() 
{
	int i=0;
    int sensor_weights[8] = {-12, -8, -4, -1, 1, 4, 8,12}; // 假设传感器物理位置权重
    int active_sensors = 0;
    float weighted_sum = 0;

    gray_state = Sensor_GetState();

    // 遍历每个传感器位
    for (i = 0; i < 8; i++) {
        if ((gray_state >> i) & 0x01) 
		{ // 检查第i位是否为1（触发）
            weighted_sum += sensor_weights[i];
            active_sensors++;
        }
    }

    // 处理无传感器触发的情况
    if (active_sensors == 0) {
        gray_status_worse++;
//		error_hv = 0;
        return error_hv; // 返回上一次误差或默认值
    }

    // 计算加权平均值
    error_hv = weighted_sum / active_sensors;
    return error_hv;
}




//float imu_get_angle()//积分一个角度
//{
//	z_angle=z_angle+icm42688_gyro_z*0.002;  //积分获取真实角度
//	zhijiao.p=70;
//	zhijiao.i=0;
//	zhijiao.d=21;
//	duty_steer=Realize_PID(&zhijiao,z_angle,target_zhijiao);
//	return duty_steer;
//	
//}

//循迹识别近道标志位
void zhijiao_angle()
{
	
	gray_state=Sensor_GetState();

	switch(gray_state)
	{
//		case 0xE0:zhijiao_flag = 1;goout_flag = 1;break;	  //11100000b
//		case 0x07:zhijiao_flag = 2;goout_flag = 1;break;	  //00000111b
	
		case 0xF0:zhijiao_flag = 1;break;	  //11110000b
		case 0x0F:zhijiao_flag = 2;break;	  //00001111b
		
		case 0xF8:zhijiao_flag = 1;break;	  //11111000b
		case 0x1F:zhijiao_flag = 2;break;	  //00011111b
		
		case 0xFC:zhijiao_flag = 1;break;	  //11111100b
		case 0x3F:zhijiao_flag = 2;break;	  //00111111b

		default:
		{
			zhijiao_flag = 0;break;
		}	
	}
}

//void out_detect()
//{
//	int gray_state = 0x0;
//	gray_state=Sensor_GetState();
//	if(gray_state == 0xFF)
//	{
//		out_flag = 1;	
//	}
//}


//void left_turn()
//{

//	duty_steer=7000;

//}









////转过了80°，恢复正常循迹
//void huifu_jindao()
//{
//	if(z_angle>70 || z_angle<-70)
//    {
//		zhijiao_flag = 0;
//    }
//}

//void chu_jindao1()
//{

//		target_zhijiao = 130;
//		imu_get_angle();//只有i的控制
//		huifu_jindao();
//	
//}

//void chu_jindao2()
//{
//	gray_state=Sensor_GetState();
//	if(gray_state == 0xFF)
//	{
//		target_zhijiao = -80;
//		imu_get_angle();//只有i的控制
//		get_jindao();
//	}
//}
////扫到进度出去
//void chuqu_angle1()
//{
//	
//	gray_state=Sensor_GetState();
//	if(gray_state == 0xFF)
//	{
//		target_zhijiao = 80;
//		imu_get_angle();
//		chuqu_flag = 0;
//	}

//}

////扫到进到出去
//void chuqu_angle2()
//{
//	
//	gray_state=Sensor_GetState();
//	if(gray_state == 0xFF)
//	{
//		target_zhijiao = 80;
//		imu_get_angle();
//		chuqu_flag = 0;
//	}

//}





