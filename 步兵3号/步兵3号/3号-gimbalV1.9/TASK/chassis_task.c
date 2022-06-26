#include "chassis_task.h"
#include "sent_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "can_receive.h"
#include "bsp_uart.h"

uint8_t chassis_speed_x,chassis_speed_y;

uint16_t move_count=0;
short int f_move_flag=0;
short int l_move_flag=0;

int yuuuu=0;
void remote_chassis(void)
{
	if(gimbal_set_mode==GIMBAL_INIT) //初始
	{
		canTX_chassis_first(rc_sent.x_speed,rc_sent.y_speed,rc_sent.r_speed,gimbal_y.given_current);
		canTX_chassis_second(1);
	}
	if(gimbal_set_mode==GIMBAL_CALI||gimbal_set_mode==GIMBAL_RELATIVE_ANGLE) //正常
	{
		canTX_chassis_first(rc_sent.x_speed,rc_sent.y_speed,rc_sent.r_speed,gimbal_y.CAN_actual_angle);
		canTX_chassis_second(0);
	}
	if(gimbal_set_mode==GIMBAL_ABSOLUTE_ANGLE) //随动
	{
//		//上位机测试视觉所用控制底盘移动
//		if(!F_move_flag==0 && move_count==0)
//		{
//			move_count=10;
//			f_move_flag=F_move_flag;
//		}
//		if(!L_move_flag==0 && move_count==0)
//		{
//			move_count=10;
//			l_move_flag=L_move_flag;
//		}
//		
//			if(!move_count==0)
//			{
//					move_count--;
//				switch(f_move_flag)
//				{	
//					case 1:
//						rc_sent.x_speed=40;
//					break;
//					case -1:
//						rc_sent.x_speed=-40;
//					break;
//					default:break;
//				}
//				switch(l_move_flag)
//				{
//					case 1:
//						rc_sent.y_speed=-40;
//					break;
//					case -1:
//						rc_sent.y_speed=40; 
//		      default:break;
//				}
//			}
//			else if(move_count==0)
//			{
//						rc_sent.y_speed=0;
//						rc_sent.x_speed=0;
//						f_move_flag=0;
//						l_move_flag=0;
//			}
//			F_move_flag=0;
//			L_move_flag=0;
		canTX_chassis_first(rc_sent.x_speed,rc_sent.y_speed,rc_sent.r_speed,gimbal_y.CAN_actual_angle);
		canTX_chassis_second(1);
	}	
	if(gimbal_set_mode==GIMBAL_TOP_ANGLE) //小陀螺
	{
		canTX_chassis_first(rc_sent.x_speed,rc_sent.y_speed,rc_sent.r_speed,gimbal_y.CAN_actual_angle);
		canTX_chassis_second(2);
	}
	if(gimbal_set_mode==GIMBAL_ZERO_FORCE) //无力
	{
		canTX_chassis_first(rc_sent.x_speed,rc_sent.y_speed,rc_sent.r_speed,gimbal_y.given_current);
		canTX_chassis_second(3);
	}
}
