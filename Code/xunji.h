#ifndef _xunji_H_
#define _xunji_H_


int hdgs(void);
void ad();
float Get_Grayscale_Error(void);
float imu_get_angle();
void chuqu_angle();
void zhijiao_angle();
void huifu_jindao();
void chuqu_angle1();
void chuqu_angle2();
void chu_jindao1();
void chu_jindao2();
int Sensor_GetState(void);
void left_turn();
void out_detect();



extern int hd[8];
extern unsigned int adc_values[8];
extern float error_hv;
extern int zhijiao_flag;
extern float z_angle;
extern int out_flag;
extern int gray_state;




#endif