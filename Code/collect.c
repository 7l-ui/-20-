#include "dmx_all.h"
#include "dmx_motor.h"
#include "dmx_control.h"
#include "xunji.h"
#include "collect.h"

float filter1 = 0;
float filter2 = 0;
float grayscale_error =0;

int count_flag = 0;//
int stop_line_count = 0;
int stop_count = 0;
float stop_flag = 1;


void xun_xian()//正常巡线，误差环串偏航角角速度环
{

	grayscale_error = Get_Grayscale_Error();
	
	target_yaw_rate=Realize_PID(&steering, grayscale_error,0);
	duty_steer=Pianhang_PID(&angle_hv,icm42688_gyro_z ,target_yaw_rate);
//	if(grayscale_error<=4&&grayscale_error>=-4)
//	{
//		stop_flag=1.3;
//	}
//	else
//	{
//		stop_flag=1;
//	}
}

void xunxian_zhijiao()//正常巡线，忽略直角
{
//	int gray_state = 0x0;
	grayscale_error = 0;
	
	target_yaw_rate=Realize_PID(&steering, grayscale_error,0);
	duty_steer=Pianhang_PID(&angle_hv,icm42688_gyro_z ,target_yaw_rate);

}

void go_zhijiao1()//判断是不是直角近道
{
	grayscale_error=0;
//	target_zhijiao = 40;
//	imu_get_angle();//积分一个角度
//	get_jindao();//是否转弯完成
//	chuqu_flag = 1;//给出去的标志位置1
}

void go_zhijiao2()//判断是不是直角近道
{
	grayscale_error=0;
//	target_zhijiao = -40;
//	imu_get_angle();//积分一个角度
//	get_jindao();//是否转弯完成
//	chuqu_flag = 2;

}



float lowpassfilter1(int input)
{
	static float pre=0;
	filter1 = 0.3*input+0.7*pre;
	pre = filter1;
	return filter1;

}

float lowpassfilter2(int input)
{
	static float pre=0;
	filter2 = 0.3*input+0.7*pre;
	pre = filter2;
	return filter2;

}

void stop_line()
{
    int detect_flag = 0;
    int gray_state = 0x00;
    static int feature_count = 0;
    static int error_count = 0;
    int count=0;
    int steer[8]={0};
    int i =0;
    int j =0;
    int k =0;
    gray_state = Sensor_GetState();
    for(i=0;i<8;i++)
    {
        if(gray_state & (1 << (7-i)))
        {
            steer[count++] = i;
        }
    
    }
    for(i=0;i<count;i++)
    {
        for(j=i+1;j<count;j++)
        {
            int state = steer[i];
            int end = steer[j];
            for(k=state;k<end;k++)
            {
                if((gray_state & (1 <<(7-k)))==0)
                {
					
					detect_flag = 1;
					goto arr;          
                }
            }
        }
    }
	arr:
	if(detect_flag)
	{
		error_count = 0;
		if(count_flag == 1)
		{
			stop_line_count++;
		}
		else
		{
			feature_count++;
			if(feature_count>=6)
			{
				count_flag = 1;
				stop_line_count=1;
				feature_count = 0;
			}
		}
	}
	else
	{
		feature_count=0;
		if(++error_count == 3)
		{
			count_flag = 0;
			error_count = 0;
			stop_line_count = 0;//连续3次没有检测到起跑线特征
		}    


	}
	if(stop_line_count >= 100)//计数器累加到500ms
	{
		 stop_line_count = 0;
		 if(++stop_count == 3)//起跑线计数
		 {
			 stop_count = 0;//起跑线计数清空
			 stop_flag = 0;//停车    
		 }
	}
}//代码没问题了，但是参数要调节

    
void zhijiao_gocome()
{
	int a,err=0;
	int count=0,dount=0;
	int gray_state = 0x00;
	gray_state = Sensor_GetState();
	for(a=0;a<4;a++)
	{
		if(gray_state & (0x80 >> a))
		{
			count++;
		}
	}
	for(a=4;a<8;a++)
	{
		if(gray_state & (0x80 >> a))
		{
			dount++;
		}
	}
	
	err=count-dount;
	if(err>3)
	{
		
			if(gray_state & (0x80 >> 0))
			{
				zhijiao_flag = 1;	
			}
			
		
		
	}
}



//void zhijiao_gocome()
//{
//	int a,err=0;
//	int count=0,dount=0;
//	int gray_state = 0x00;
//	gray_state = Sensor_GetState();
//	for(a=0;a<3;a++)
//	{
//		if(gray_state & (0x80 >> a))
//		{
//			count++;
//		}
//	}
//	for(a=7;a>4;a--)
//	{
//		if(gray_state & (0x80 >> a))
//		{
//			dount++;
//		}
//	}
//	err=count+dount;
//	if(err==6)
//	{
//		zhijiao_flag = 1;	
//	}
//}



void zhijiao_leavecome()
{
	int a,sums=0;
	int count=0,dount=0;
	int gray_state = 0x00;
	gray_state = Sensor_GetState();
	for(a=0;a<4;a++)
	{
		if(gray_state & (0x80 >> a))
		{
			count++;
		}
	}
	for(a=4;a<8;a++)
	{
		if(gray_state & (0x80 >> a))
		{
			dount++;
		}
	}
	sums=count+dount;
	if(sums>7)
	{
		out_flag = 1;
	}
}





////void chuqu1()//出近道的标志位
////{
////		gray_state = Sensor_GetState();
////		if(chuqu_flag == 1 && gray_state == 0xFF)//出去标志位为1，且八个传感器全黑，则代表扫到近道
////		{
////				
////			target_zhijiao = -90;//设置目标角度
////			imu_get_angle();//积分一个角度
////			get_jindao();//看是否转弯完成
////			chuqu_flag = 0;
////		}
////		
////		
////}

////void chuqu2()//出近道的标志位
////{
////		gray_state = Sensor_GetState();
////		if(chuqu_flag == 2 && gray_state == 0xFF)
////		{
////				
////			target_zhijiao = 90;
////			imu_get_angle();//积分一个角度
////			get_jindao();//看是否转弯完成
////			chuqu_flag = 0;
////		}		
////}
//void out_detect()
//{
//	gray_state = Sensor_GetState();
//	if(gray_state == 0xFF)//出去标志位为1，且八个传感器全黑，则代表扫到近道
//	{
//		chuqu_flag = 1;
//	}
//}






void left_turn()
{
	duty_steer = 0;
}



