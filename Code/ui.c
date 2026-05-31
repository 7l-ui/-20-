#include "dmx_all.h"
#include "ui.h"

bit key_flag_on = 0;//拨码开关标志位

char cursor_y = 0;//光标位置
char last_cursor_y = 0;//上一页光标位置

unsigned char index_ui = 0;//菜单显示索引

float mul = 1;//调参倍率

bit adjust_flag = 0;//调参标志位

//开关状态变量
bit key1_status = 1;
bit key2_status = 1;
bit key3_status = 1;
bit key4_status = 1;
bit key5_status = 1;
bit key6_status = 1;

//上一次开关状态变量
bit key1_last_status;
bit key2_last_status;
bit key3_last_status;
bit key4_last_status;
bit key5_last_status;

/**********UI主函数**********/
void ui(void)
{
	if(!adjust_flag)
	{
		ui_update();//主菜单显示
    ui_contentkey();//接收键值并更新光标位置
	}
	else
	{
		ui_update();
		ui_datekey();//调参按键检测
	}
	display_cursor();//光标显示
}

/**********获取按键状态**********/
int get_key_flag(void)
{
	//保存按键状态
	key1_last_status = key1_status;
	key2_last_status = key2_status;
	key3_last_status = key3_status;
	key4_last_status = key4_status;
	key5_last_status = key5_status;
	
	//读取当前按键状态
	key1_status = up;
	key2_status = down;
	key3_status = enter;
	key4_status = back;
	key5_status = right;
	
	
	//按键检测
	if(key1_status && !key1_last_status) return key1_flag;
	if(key2_status && !key2_last_status) return key2_flag;
	if(key3_status && !key3_last_status) return key3_flag;
	if(key4_status && !key4_last_status) return key4_flag;
	if(key5_status && !key5_last_status) return key5_flag;
	
	return 0;
}

/**********光标显示**********/
void display_cursor(void)
{
	Show_String_IPS(2,cursor_y*16,">",RED,WHITE,Show8x16);
}

/***********接收键值并更新光标**********/
void ui_contentkey(void)
{
	char key = get_key_flag();
	
	key6_status = start;//检测拨码开关状态
	
	if(key == key1_flag)//向上
	{
		Show_String_IPS(0,cursor_y*16," ",BLACK,WHITE,Show8x16);
		cursor_y--;
	}
	if(key == key2_flag)//向下
  {
		Show_String_IPS(0,cursor_y*16," ",BLACK,WHITE,Show8x16);
		cursor_y++;
	}
	if(key == key3_flag)//确认
	{
		Clean_IPS(WHITE);
		index_ui = page_down();
		if(index_ui == 9)
		{
			last_cursor_y = index_ui - 4;
		}
		else
		{
			last_cursor_y = index_ui - 1;
		}
		cursor_y = 0;
	}
	if(key == key4_flag)//返回
	{
		Clean_IPS(WHITE);
		index_ui = page_up();
		if(index_ui == 6)
		{
			cursor_y = 0;
		}
		else
		{
			cursor_y = last_cursor_y;
		}
	}
	if(key6_status && key_flag_on)//拨码开关检测
	{
		Clean_IPS(WHITE); 
		display_main();
    start_flag = 0;	
    delay_ms(5);	
		TR1 = 0;
		left_rear_speed = 0;
		right_rear_speed = 0;
		key_flag_on = ~key_flag_on;
	}
	if(!key6_status && !key_flag_on)
	{
		Clean_IPS(WHITE);
		display_start();
		key_flag_on = ~key_flag_on;
	}
	if(index_ui >= 1 && index_ui <= 7)//调参状态检测
	{
		if(key == key5_flag)
		{
			adjust_flag = 1;
		}
	}
	//光标位置限幅
	if(cursor_y < 0)
	{
		cursor_y = 7;
	}
	if(cursor_y > 7)
	{
		cursor_y = 0;
	}
}

/**********调参键值检测**********/
void ui_datekey(void)
{
	char key = get_key_flag();//接收键值
	
	Show_String_IPS(210,cursor_y*16,"....",RED,WHITE,Show8x16);
	
	if(key == key1_flag)
	{
		switch(index_ui)//判断在哪一页
		{
			case 1:
			{
				switch(cursor_y)//判断在哪一行
				{
					case 0:angle_vel.kp += mul; break;//在第一行，调整kp
					case 1:angle_vel.ki += mul; break;//在第二行，调整ki
					case 2:sudu += mul; break;//在第二行，调整ki
					default:break;
				}
				break;
			}
			case 2:
			{
				switch(cursor_y)
				{
					case 0:angle.kp += mul; break;
					case 1:angle.kd += mul; break;
					case 2:lingdian += mul; break;
					default:break;
				}
				break;
			}
			case 3:
			{
				switch(cursor_y)
				{
					case 0:speed.kp += mul; break;
					case 1:speed.lingdian += mul; break;
					default:break;
				}
				break;
			}
			case 4:
			{
				switch(cursor_y)
				{
					case 0:turn.kp += mul; break;
					case 1:turn.kd += mul; break;
					default:break;
				}
				break;
			}
			case 5:
			{
				switch(cursor_y)
				{
					case 0:angle_hv.kp += mul; break;
					case 1:angle_hv.ki += mul; break;
					default:break;
				}
				break;
			}
			case 6:
			{
				if(cursor_y == 1)
				{
					threshold_gra += mul;
				}
				break;
			}
			case 7:
			{
				mul = mul * 10;//调整倍率
				break;
			}
		}
	}
	if(key == key2_flag)
	{
		switch(index_ui)//判断在哪一页
		{
			case 1:
			{
				switch(cursor_y)//判断在哪一行
				{
					case 0:angle_vel.kp -= mul; break;//在第一行，调整kp
					case 1:angle_vel.ki -= mul; break;//在第二行，调整ki
					case 2:sudu -= mul; break;//在第二行，调整ki
					default:break;
				}
				break;
			}
			case 2:
			{
				switch(cursor_y)
				{
					case 0:angle.kp -= mul; break;
					case 1:angle.kd -= mul; break;
					case 2:lingdian -= mul; break;//在第二行，调整ki
					default:break;
				}
				break;
			}
			case 3:
			{
				switch(cursor_y)
				{
					case 0:speed.kp -= mul; break;
					case 1:speed.lingdian -= mul; break;
					default:break;
				}
				break;
			}
			case 4:
			{
				switch(cursor_y)
				{
					case 0:turn.kp -= mul; break;
					case 1:turn.kd -= mul; break;
					default:break;
				}
				break;
			}
			case 5:
			{
				switch(cursor_y)
				{
					case 0:angle_hv.kp -= mul; break;
					case 1:angle_hv.ki -= mul; break;
					default:break;
				}
				break;
			}
			case 6:
			{
				if(cursor_y == 1)
				{
					threshold_gra -= mul;
				}
				break;
			}
			case 7:
			{
				mul = mul / 10.0;//调整倍率
				break;
			}
		}
	}
	if(key == key4_flag)//按下返回键
	{
		Clean_IPS(WHITE);
		adjust_flag = 0;//标志位归零，退出调参状态
	}
}

/**********UI更新当前界面*********/
void ui_update(void)
{
	switch(index_ui)
	{
		/**一级菜单**/
		case 0:display_main();       break;//主菜单
		/**二级菜单**/
		case 1:display_angle_vel();  break;//角速度环参数
		case 2:display_angle();      break;//角度环参数
		case 3:display_speed();      break;//速度环参数
		case 4:display_turn();       break;//转向环参数
		case 5:display_angle_hv();   break;//偏航角环参数
		case 6:display_image_date(); break;//图像数据
		case 7:display_mul();        break;//倍率调整
		case 8:display_save();       break;//是否保存
		/**三级菜单**/
		case 9:display_gray();       break;//灰度值显示
		case 10:
		{
			display_already_save();
			index_ui = 0;
			break;//保存
		}
		default:display_main();      break;
	}
}

/**********返回上一级菜单**********/
int page_up(void)
{
	switch(index_ui)
	{
		case 0:return 0; 
		case 1:return 0; 
		case 2:return 0; 
		case 3:return 0; 
		case 4:return 0; 
	  case 5:return 0; 
		case 6:return 0; 
		case 7:return 0;
		case 8:return 0;
		case 9:return 6;
		default:return 0;
	}
}

/**********进入下一级菜单**********/
int page_down(void)
{
	switch(index_ui)
	{
		case 0://在主菜单
		{
			switch(cursor_y)//判断光标在第几行
			{
				case 0:return 1;//在第一行，进入角速度环参数
				case 1:return 2;//在第二行，进入角度环参数
				case 2:return 3;//在第三行，进入速度环参数
				case 3:return 4;//在第四行，进入转向环参数
				case 4:return 5;//在第五行，进入偏航角环参数
				case 5:return 6;//在第六行，进入图像数据显示选择
				case 6:return 7;//在第七行，进入倍率调节
				case 7:return 8;//在第八行，进入保存
				default: return 0;
			}
		}break;
		case 6://在图像数据界面
		{
			switch(cursor_y)//判断光标在第几行
			{
			  case 0:return 9;//在第一行，进入灰度值显示
				case 1:return 6;//在第二行，不变
				default:return 0;
			}
		}break;
		case 1:return 1; 
		case 2:return 2; 
		case 3:return 3; 
		case 4:return 4; 
	  case 5:return 5; 
		case 8:return 10;
		default:return 0;
	}
}

/**********主菜单显示**********/
void display_main(void)
{
	Show_String_IPS(10,0*16,"1. angle_vel:",BLACK,WHITE,Show8x16);//角速度环
	Show_String_IPS(10,1*16,"2. angle:",BLACK,WHITE,Show8x16);//角度环
	Show_String_IPS(10,2*16,"3. speed:",BLACK,WHITE,Show8x16);//速度环
	Show_String_IPS(10,3*16,"4. turn:",BLACK,WHITE,Show8x16);//转向环
	Show_String_IPS(10,4*16,"5. angle_hv:",BLACK,WHITE,Show8x16);//偏航角
	Show_String_IPS(10,5*16,"6. image:",BLACK,WHITE,Show8x16);//图像
	Show_String_IPS(10,6*16,"7. mul:",BLACK,WHITE,Show8x16);//调参倍率
	Show_String_IPS(95,7*16,"L:",RED,WHITE,Show8x16);//左轮编码器数据
	Show_Float_IPS(115,7*16,left_rear_speed,4,0,BLACK,WHITE,Show8x16);
	Show_String_IPS(170,7*16,"R:",RED,WHITE,Show8x16);//右轮编码器数据
	Show_Float_IPS(190,7*16,right_rear_speed,4,0,BLACK,WHITE,Show8x16);
	Show_String_IPS(10,7*16,"8. save:",BLACK,WHITE,Show8x16);
}

/**********次级菜单---角速度环参数**********/
void display_angle_vel(void)
{
	Show_String_IPS(10,0*16,"1. angle_vel.kp:",BLACK,WHITE,Show8x16);
	Show_Float_IPS(145,0*16,angle_vel.kp,3,3,BLACK,WHITE,Show8x16);
	Show_String_IPS(10,1*16,"2. angle_vel.ki:",BLACK,WHITE,Show8x16);
	Show_Float_IPS(145,1*16,angle_vel.ki,3,3,BLACK,WHITE,Show8x16);
	Show_String_IPS(10,2*16,"3. sudu:",BLACK,WHITE,Show8x16);
	Show_Float_IPS(145,2*16,sudu,3,3,BLACK,WHITE,Show8x16);
}

/**********次级菜单---角度环参数**********/
void display_angle(void)
{
	Show_String_IPS(10,0*16,"1. angle.kp:",BLACK,WHITE,Show8x16);
	Show_Float_IPS(115,0*16,angle.kp,3,3,BLACK,WHITE,Show8x16);
	Show_String_IPS(10,1*16,"2. angle.kd:",BLACK,WHITE,Show8x16);
	Show_Float_IPS(115,1*16,angle.kd,3,3,BLACK,WHITE,Show8x16);

}

/**********次级菜单---速度环参数**********/
void display_speed(void)
{
	Show_String_IPS(10,0*16,"1. speed.kp:",BLACK,WHITE,Show8x16);
	Show_Float_IPS(115,0*16,speed.kp,3,3,BLACK,WHITE,Show8x16);
	Show_String_IPS(10,1*16,"2. speed.ld:",BLACK,WHITE,Show8x16);
	Show_Float_IPS(115,1*16,speed.lingdian,3,4,BLACK,WHITE,Show8x16);
}

/**********次级菜单---转向环参数**********/
void display_turn(void)
{
	Show_String_IPS(10,0*16,"1. turn.kp:",BLACK,WHITE,Show8x16);
	Show_Float_IPS(115,0*16,turn.kp,3,3,BLACK,WHITE,Show8x16);
	Show_String_IPS(10,1*16,"2. turn.kd:",BLACK,WHITE,Show8x16);
	Show_Float_IPS(115,1*16,turn.kd,3,3,BLACK,WHITE,Show8x16);
}

/**********次级菜单---偏航角环参数**********/
void display_angle_hv(void)
{
	Show_String_IPS(10,0*16,"1. angle_hv.kp:",BLACK,WHITE,Show8x16);
	Show_Float_IPS(145,0*16,angle_hv.kp,3,3,BLACK,WHITE,Show8x16);
	Show_String_IPS(10,1*16,"2. angle_hv.ki:",BLACK,WHITE,Show8x16);
	Show_Float_IPS(145,1*16,angle_hv.ki,3,3,BLACK,WHITE,Show8x16);
}

/**********次级菜单---图像数据*********/
void display_image_date(void)
{
	Show_String_IPS(10,0*16,"1. huiduzhi:",BLACK,WHITE,Show8x16);
	Show_String_IPS(10,1*16,"2. threshold_gra:",BLACK,WHITE,Show8x16);
	Show_Int_IPS(160,1*16,threshold_gra,4,BLACK,WHITE,Show8x16);
}

/**********次级菜单---灰度值显示**********/
void display_gray(void)
{
	// 灰度传感器1
	Show_String_IPS(2,0*16,"AD1:",BLACK,WHITE,Show8x16);
	Show_Int_IPS(45,0*16,get_adc(AD1, ADC_12BIT),4,BLACK,WHITE,Show8x16);
	// 灰度传感器2
	Show_String_IPS(2,1*16,"AD2:",BLACK,WHITE,Show8x16);
	Show_Int_IPS(45,1*16,get_adc(AD2, ADC_12BIT),4,BLACK,WHITE,Show8x16);
	// 灰度传感器3
	Show_String_IPS(2,2*16,"AD3:",BLACK,WHITE,Show8x16);
	Show_Int_IPS(45,2*16,get_adc(AD3, ADC_12BIT),4,BLACK,WHITE,Show8x16);
	// 灰度传感器4
	Show_String_IPS(2,3*16,"AD4:",BLACK,WHITE,Show8x16);
	Show_Int_IPS(45,3*16,get_adc(AD4, ADC_12BIT),4,BLACK,WHITE,Show8x16);
	// 灰度传感器5
	Show_String_IPS(2,4*16,"AD5:",BLACK,WHITE,Show8x16);
	Show_Int_IPS(45,4*16,get_adc(AD5, ADC_12BIT),4,BLACK,WHITE,Show8x16);
	//灰度传感器6
	Show_String_IPS(2,5*16,"AD6:",BLACK,WHITE,Show8x16);
	Show_Int_IPS(45,5*16,get_adc(AD6, ADC_12BIT),4,BLACK,WHITE,Show8x16);
	// 灰度传感器7
	Show_String_IPS(2,6*16,"AD7:",BLACK,WHITE,Show8x16);
	Show_Int_IPS(45,6*16,get_adc(AD7, ADC_12BIT),4,BLACK,WHITE,Show8x16);
	// 灰度传感器8
	Show_String_IPS(2,7*16,"AD8:",BLACK,WHITE,Show8x16);
	Show_Int_IPS(45,7*16,get_adc(AD8, ADC_12BIT),4,BLACK,WHITE,Show8x16);	
}

/**********次级菜单---智能车启动**********/
void display_start(void)
{
	Show_String_IPS(2,0*16,"starting......",BLACK,WHITE,Show8x16);
	delay_ms(400);
	Clean_IPS(WHITE);
	display_main();//切换回主菜单
	start_flag = 1;//电机启动
	TR1 = 1;//开启定时器
}

/**********次级菜单---倍率显示**********/
void display_mul(void)
{
	Show_Float_IPS(10,0*16,mul,5,6,BLACK,WHITE,Show8x16);
}

/**********次级菜单---是否保存**********/
void display_save(void)
{
	Show_String_IPS(10,0*16,"save this change ?",BLACK,WHITE,Show8x16);
}

/**********次级菜单---保存**********/
void display_already_save(void)
{
	erase_sector(0);//擦除EEPROM要写入区域
	pid_write();//写入数据
	write_eeprom(0x00 + 20,pid_date,42);//写入参数到eeprom
	delay_ms(40);//延时
	
	Show_String_IPS(10,0*16,"Already save !",BLACK,WHITE,Show8x16);
	delay_ms(400);//延时
	Clean_IPS(WHITE);//白色清屏
	display_main();//返回主菜单
}
