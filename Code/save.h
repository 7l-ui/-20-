#ifndef _SAVE_H_
#define _SAVE_H_

extern unsigned char pid_date[];//声明缓冲数组
extern unsigned char sudu;
extern unsigned char threshold_gra;//声明二值化阈值
extern float lingdian;
extern int start_flag;

//定义调参共用体
typedef union
{
	float f;
	unsigned char byte[4];
}pid_adjust;

//函数声明
void pid_read(void);
void pid_write(void);





#endif