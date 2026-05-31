#ifndef _DMX_CONTROL_H_
#define _DMX_CONTROL_H_

// 定义PID结构体
typedef struct {
    float p;
    float i;
    float d;
//    float wp;
//    float wd;
    float error;
    float integral;
    float last_error;
    float last_last_error;
    float out;
} PID_struct;

// 声明角速度结构体
extern PID_struct angle_vel;
// 声明角度结构体
extern PID_struct angle;
// 声明速度结构体
extern PID_struct speed;
//声明误差结构体
extern PID_struct steering;
// 声明偏航角速度结构体
extern PID_struct angle_hv;
extern PID_struct zhijiao; //设置直角循迹结构体重命名
//extern PID_struct zhijiao;
/**
*
* @brief    PID参数初始化
* @param
* @return   void
* @notes
* Example:  Init_PID();
*
**/
void Init_PID(void);

/**
*
* @brief    位置式PID
* @param    pid				      PID结构体
* @param    rear        		真实值
* @param    hope            目标值
* @return   int             PID运算结果
* @notes
* Example:  Realize_PID(&pid, rear, hope);
*
**/
int Realize_PID(PID_struct *pid, float rear, float hope);
int turn_PID(PID_struct *pid, float rear, float hope);
int AV_PID(PID_struct *pid, float rear, float hope);
int Pianhang_PID(PID_struct *pid, float rear, float hope);
int Speed_PID(PID_struct *pid, float rear, float hope);

void clean_collect();
void clean_xunji();
void chuqu1();
void chuqu2();
void clean_speed();
#endif