#include "dmx_all.h"
#include "dmx_control.h"
#include "xunji.h"
#include "collect.h"
// 定义角速度结构体
PID_struct angle_vel;
// 定义角度结构体
PID_struct angle;
PID_struct speed;
PID_struct steering;
PID_struct angle_hv;
PID_struct zhijiao; //设置直角循迹结构体重命名

void Init_PID(void)
{
//	
//      // 角速度环参数     （内环）角速度环→角度环
//      angle_vel.p = 50;
//      angle_vel.i = 4.1; // 大了车身会前后抖
//      angle_vel.d = 0;
//      // 角度环参数       （外环）
//      angle.p = 4.6; // 大了会前后摇摆磕头
//      angle.i = 0;
//      angle.d = 15.3; // d比p大
//      // 速度环参
//      speed.p = 19.1; // 小了车跑不动，大了速度过快效果差
//      speed.i = 0;    // 不给i
//      speed.d = 13;
//      // 转向误差环参数   //转向误差→(P增益)→目标偏航角速度→(角速度环)→实际偏航角速度
//      steering.p = -65.2; // 大了车身连着车轮会左右抖
//      steering.i = 0;
//      steering.d = -25; // 小了车抖，大了循迹差
//      //偏航角角速度参数
//      angle_hv.p = 65;  // 提高跟踪速度
//	  angle_hv.i = 5;// 消除稳态误差
//      angle_hv.d = 17.5;  // 抑制超调和振荡
//	  

//      // 角速度环参数     （内环）角速度环→角度环
//      angle_vel.p = 50;
//      angle_vel.i = 4.1; // 大了车身会前后抖
//      angle_vel.d = 0;
//      // 角度环参数       （外环）
//      angle.p = 4.7; // 大了会前后摇摆磕头
//      angle.i = 0;
//      angle.d = 15.3; // d比p大
//      // 速度环参
//      speed.p = 17.2; // 小了车跑不动，大了速度过快效果差
//      speed.i = 0;    // 不给i
//      speed.d = 8.5;
      // 转向误差环参数   //转向误差→(P增益)→目标偏航角速度→(角速度环)→实际偏航角速度
//      steering.p = -60.2; // 大了车身连着车轮会左右抖
//      steering.i = 0;
//      steering.d = -27; // 小了车抖，大了循迹差
      //偏航角角速度参数
//      angle_hv.p = 58;  // 提高跟踪速度
//      angle_hv.i = 1.6; // 消除稳态误差
//      angle_hv.d = 19;  // 抑制超调和振荡

//	  steering.p = -47;//大了车抖55
//      steering.i = 0;
//      steering.d = -19.5;//小了车抖，大了循迹差25
//    //偏航角角速度参数57
//        angle_hv.p = 36;
//        angle_hv.i = 0.19;  //过大循迹抖动1.2/16
//        angle_hv.d = 0;
	  steering.p = -44;//大了车抖55
	  steering.i = 0;
	  steering.d = -19.5;//小了车抖，大了循迹差25

	//偏航角角速度参数57
		angle_hv.p = 36;
		angle_hv.i = 0.19;  //过大循迹抖动1.2/16
		angle_hv.d = 0;
		
		//	//偏航角角速度参数57
//		angle_hv.p = 55;
//		angle_hv.i = 1.2;  //过大循迹抖动1.2/16
//		angle_hv.d = 16;
//		    // 转向误差环参数
//	  steering.p = -44;//大了车抖55
//	  steering.i = 0;
//	  steering.d = -19.5;//小了车抖，大了循迹差25


		
			angle_vel.p = 51;
		angle_vel.i = 4.3;
		angle_vel.d = 0.1;
						angle.p = 2.2 ;
		angle.i = 0;
		angle.d =16.3;//15.3
//				angle.p =2.2;
//		angle.i = 0;
//		angle.d =26.3;//15.3
//				angle.p = 3.7;
//		angle.i = 0;
//		angle.d =15.3;//15.3
        zhijiao.p=50;
        zhijiao.i=0;
        zhijiao.d=0;

		speed.p = 0.21;//小了车跑不动，大了速度过快效果差
		speed.i = 0;
		speed.d=0.01;
} // 误差环增大，偏航角角速度也要增大

/**
 *
 * @brief    位置式PID
 * @param    pid		     PID结构体
 * @param    rear        	 真实值
 * @param    hope            目标值
 * @return   int             PID运算结果
 * @notes
 * Example:  Realize_PID(&pid, rear, hope);
 *
 **/
int Realize_PID(PID_struct *pid, float rear, float hope)
{
    // 偏差计算
    pid->error = hope - rear;
    // 积分
    pid->integral += pid->error;
    // 积分限幅
    if (pid->integral > 1000)
        pid->integral = 1000;
    if (pid->integral < -1000)
        pid->integral = -1000;
    // PID运算结果
    pid->out = pid->p * pid->error + pid->i * pid->integral + pid->d * (pid->error - pid->last_error);
    // 更新上次偏差
    pid->last_error = pid->error;
    // 输出限幅
    if (pid->out > 7000)
        pid->out = 7000;
    if (pid->out < -7000)
        pid->out = -7000;

    return (int)(pid->out);
}

int Speed_PID(PID_struct *pid, float rear, float hope)
{
    // 偏差计算

    //	pid->error = hope - rear;
    //	pid->error +=pid->error*0.3+pid->last_error * 0.7;
    //	pid->last_error = pid->error;
    pid->last_error = hope - rear;
    pid->error *= 0.8;
    pid->error += pid->last_error * 0.2;
    // 积分
    pid->integral += pid->error;
    // 积分限幅
    if (pid->integral > 1000)
        pid->integral = 1000;
    if (pid->integral < -1000)
        pid->integral = -1000;
    // PID运算结果
    pid->out = pid->p * pid->error + pid->i * pid->integral + pid->d * (pid->error - pid->last_error);
    // 更新上次偏差
    pid->last_error = pid->error;
    // 输出限幅
    if (pid->out > 7.3)
        pid->out = 7.3;
    if (pid->out < -10)
        pid->out = -10;

    return (int)(pid->out);
}

int AV_PID(PID_struct *pid, float rear, float hope)
{
    // 偏差计算
    pid->error = hope - rear;
    // 积分
    pid->integral += pid->error;
    // 积分限幅
    if (pid->integral > 1000)
        pid->integral = 1000;
    if (pid->integral < -1000)
        pid->integral = -1000;
    // PID运算结果
    pid->out = pid->p * pid->error + pid->i * pid->integral + pid->d * (pid->error - pid->last_error);
    // 更新上次偏差
    pid->last_error = pid->error;
    // 输出限幅
    if (pid->out > 8000)
        pid->out = 8000;
    if (pid->out < -8000)
        pid->out = -8000;

    return (int)(pid->out);
}

int Pianhang_PID(PID_struct *pid, float rear, float hope)
{
    // 偏差计算
    pid->error = hope - rear;
    // 积分
    pid->integral += pid->error;
    // 积分限幅
    if (pid->integral > 3000)
        pid->integral = 3000;
    if (pid->integral < -3000)
        pid->integral = -3000;
    // PID运算结果
    pid->out = pid->p * pid->error + pid->i * pid->integral + pid->d * (pid->error - pid->last_error);
    // 更新上次偏差
    pid->last_error = pid->error;
    // 输出限幅
    if (pid->out > 8000)
        pid->out = 8000;
    if (pid->out < -8000)
        pid->out = -8000;

    return (int)(pid->out);
}

void clean_collect()
{
    angle_vel.integral = 0;
}

void clean_xunji()
{
    angle_hv.integral = 0;
}

void clean_speed()
{
    speed.integral = 0;
}
