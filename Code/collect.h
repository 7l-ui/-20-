#ifndef _COLLECT_H_
#define _COLLECT_H_
void xun_xian();
void go_zhijiao1();
void go_zhijiao2();
void xunxian_zhijiao();
float lowpassfilter1(int input);
float lowpassfilter2(int input);
void stop_line();
void zhijiao_gocome();
void zhijiao_leavecome();
//void out_detect();
void left_turn();
extern float grayscale_error;
extern float stop_flag;

extern int count_flag;
extern int stop_count;
extern int stop_line_count;

#endif