#include "dmx_all.h"
#include "save.h"
unsigned char pid_date[42] = {0};//定义存储缓冲数组
unsigned char threshold_gra = 0;//定义二值化阈值
unsigned char sudu = 0;//定义二值化阈值
float lingdian = 0;//定义二值化阈值
//电机启动标志位
int start_flag = 1;

//定义角速度环共用体
pid_adjust angle_vels_kp;
pid_adjust angle_vels_ki;
//定义角度环共用体
pid_adjust angles_kp;
pid_adjust angles_kd;
//定义速度环共用体
pid_adjust speeds_kp;
pid_adjust speeds_ki;
//定义转向环共用体
pid_adjust turns_kp;
pid_adjust turns_kd;
//定义偏航角环共用体
pid_adjust angle_hvs_kp;
pid_adjust angle_hvs_ki;



/**********PID数据写入**********/
void pid_write(void)
{
	int i = 0;
	//存入二值化阈值
	pid_date[40] = threshold_gra;
	pid_date[41] = sudu;
	//存入角速度环kp
	angle_vels_kp.f = angle_vel.kp;
//	pid_date[0] = angle_vels_kp.byte[0];
//	pid_date[1] = angle_vels_kp.byte[1];
//	pid_date[2] = angle_vels_kp.byte[2];
//	pid_date[3] = angle_vels_kp.byte[3];
	for(i = 0;i < 4;i++)
	{
		pid_date[i] = angle_vels_kp.byte[i];
	}
	//存入角速度环ki
	angle_vels_ki.f = angle_vel.ki;
	for(i = 4;i < 8;i++)
	{
		pid_date[i] = angle_vels_ki.byte[i - 4];
	}
	//存入角度环kp
	angles_kp.f = angle.kp;
	for(i = 8;i < 12;i++)
	{
		pid_date[i] = angles_kp.byte[i - 8];
	}
	//存入角度环kd
  angles_kd.f = angle.kd;
	for(i = 12;i < 16;i++)
	{
		pid_date[i] = angles_kd.byte[i - 12];
	}
  //存入速度环kp
  speeds_kp.f = speed.kp;
	for(i = 16;i < 20;i++)
	{
		pid_date[i] = speeds_kp.byte[i - 16];
	}
	//存入速度环ki
	speeds_ki.f = speed.lingdian;
	for(i = 20;i < 24;i++)
	{
		pid_date[i] = speeds_ki.byte[i - 20];
	}
	//存入转向环kp
	turns_kp.f = turn.kp;
	for(i = 24;i < 28;i++)
	{
		pid_date[i] = turns_kp.byte[i - 24];
	}
	//存入转向环kd
  turns_kd.f = turn.kd;
	for(i = 28;i < 32;i++)
	{
		pid_date[i] = turns_kd.byte[i - 28];
	}
	//存入偏航角环kp
	angle_hvs_kp.f = angle_hv.kp;
	for(i = 32;i < 36;i++)
	{
		pid_date[i] = angle_hvs_kp.byte[i - 32];
	}
	//存入偏航角环ki
	angle_hvs_ki.f = angle_hv.ki;
	for(i = 36;i < 40;i++)
	{
		pid_date[i] = angle_hvs_ki.byte[i - 36];
	}
}

/**********PID参数读取**********/
void pid_read(void)
{
	int i = 0;
	//读取二值化阈值
	threshold_gra = pid_date[40];
	sudu = pid_date[41];
//	lingdian = pid_date[42];
	//读出角速度环kp
	for(i = 0;i < 4;i++)
	{
		 angle_vels_kp.byte[i] = pid_date[i];
	}
//	angle_vels_kp.byte[0] = pid_date[0];
//	angle_vels_kp.byte[1] = pid_date[1];
//	angle_vels_kp.byte[2] = pid_date[2];
//	angle_vels_kp.byte[3] = pid_date[3];
	angle_vel.kp =  angle_vels_kp.f;//重组为原浮点数
	
	//读出角速度环ki
	for(i = 4;i < 8;i++)
	{
		angle_vels_ki.byte[i - 4] = pid_date[i];
	}
	angle_vel.ki = angle_vels_ki.f;
	//读出角度环kp
	for(i = 8;i < 12;i++)
	{
		angles_kp.byte[i - 8] = pid_date[i];
	}
	angle.kp = angles_kp.f;
	
	//读出角度环kd
	for(i = 12;i < 16;i++)
	{
		angles_kd.byte[i - 12] = pid_date[i];
	}
	angle.kd = angles_kd.f;
	
	//读出速度环kp
	for(i = 16;i < 20;i++)
	{
		speeds_kp.byte[i - 16] = pid_date[i];
	}
	speed.kp = speeds_kp.f;
	
	//读出速度环ki
	for(i = 20;i < 24;i++)
	{
		speeds_ki.byte[i - 20] = pid_date[i];
	}
	speed.lingdian = speeds_ki.f;
	
	//读出转向环kp
	for(i = 24;i < 28;i++)
	{
		turns_kp.byte[i - 24] = pid_date[i];
	}
	turn.kp = turns_kp.f;
	
	//读出转向环kd
	for(i = 28;i < 32;i++)
	{
		turns_kd.byte[i - 28] = pid_date[i];
	}
	turn.kd = turns_kd.f;
	
	//读出偏航角环kp
	for(i = 32;i < 36;i++)
	{
		angle_hvs_kp.byte[i - 32] = pid_date[i];
	}
	angle_hv.kp = angle_hvs_kp.f;
	
	//读出偏航角环ki
	for(i = 36;i < 40;i++)
	{
		angle_hvs_ki.byte[i - 36] = pid_date[i];
	}
	angle_hv.ki = angle_hvs_ki.f;
}
