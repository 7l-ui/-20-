#ifndef _UI_H_
#define _UI_H_

//函数声明
void ui(void);
void display_main(void);
int get_key_flag(void);
void display_cursor(void);
void ui_contentkey(void);
void display_angle_vel(void);
void display_angle(void);
void display_speed(void);
void display_turn(void);
void display_angle_hv(void);
void display_image_date(void);
void display_gray(void);
void display_threshold(void);
void ui_update(void);
int page_up(void);
int page_down(void);
void display_start(void);
void display_mul(void);
void ui_datekey(void);
void display_save(void);
void display_already_save(void);

//定义按键引脚
#define up P70
#define down P71
#define enter P51
#define back P36
#define right P37
#define start P73

//开关标志位
typedef enum
{
	key1_flag = 1,
	key2_flag = 2,
	key3_flag = 3,
	key4_flag = 4,
	key5_flag = 5,
};

#endif