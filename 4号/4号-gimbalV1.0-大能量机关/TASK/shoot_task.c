#include "shoot_task.h"
#include "remote_control.h"
#include "sent_task.h"
#include "gimbal_task.h"
uint8_t data[2]={1,0};
shoot_task_t rc_shoot;
FRIC_SPEED fricspeed=FRIC_MAX;

int shoot_times=0;
int One_Shoot_flag=0;
int Ten_Shoot_flag=0;

float TriggerSpeed[5] 	= {4.7f,			0.2f,			0.0f,		8000,			800};  //拨弹轮速度环 8000 4.8
float TriggerAngle[5] 	= {0.2f,			0.0000f,			0.00f,		1700,			0};  //拨弹轮角度环 1700

//float FricLeftSpeed[5] 	= {4.5f,      0.95f,      0.2f,    3000.0f,      500.0f};  //左摩擦轮速度环
//float FricRightSpeed[5] 	= {10.0f,      0.2f,      0.3f,    3000.0f,      500.0f};  //右摩擦轮速度环
float FricLeftSpeed[5] 	= {7.0f,      0.1f,      0.2f,    3000.0f,      500.0f};  //左摩擦轮速度环
float FricRightSpeed[5] 	= {11.0f,      0.2f,      0.3f,    3000.0f,      500.0f};  //右摩擦轮速度环
void trigger_angle_set(void);
//更新发子弹个数
static void shoot_angle_clc(void);
//放卡弹
static void prevent_jam(void);
//摩擦轮PID计算
static void fric_pid(void);
//波弹轮PID计算
static void trigger_pid(void);
void prevent_jam()
{
	if(rc_shoot.trigger.last_back_flag==0)
	{
		if(rc_ctrl.rc.s[1]==2)  
		{
			int angle;
			rc_shoot.trigger.last_back_flag=1;
			angle=((1*36*36)/360*8191);
			if(rc_shoot.trigger.rounds>=90)
			{
				rc_shoot.trigger.target_angle-=rc_shoot.trigger.rounds*8192;
				if(rc_shoot.trigger.target_angle<=0) rc_shoot.trigger.target_angle=1;
				rc_shoot.trigger.total_angle -=rc_shoot.trigger.rounds*8192 ;
					rc_shoot.trigger.begin_angle=rc_shoot.trigger.total_angle;
				rc_shoot.trigger.rounds=0;
			}
			rc_shoot.trigger.target_angle-=angle;
		}
	}
	
	if(rc_ctrl.rc.s[1]!=2) rc_shoot.trigger.last_back_flag=0;
	
}
void shoot_task(void)
{
	if(trigger_cnt_flag==1)
		trigger_cnt++;
	if(trigger_cnt>=19)
	{
		trigger_cnt_flag=0;
		trigger_cnt=0;
		rc_shoot.trigger.target_angle-=19200.0f;
	}
	shoot_angle_clc();
	fric_pid();
	trigger_pid();
	canTX_fric(rc_shoot.left_fric.set_currunt,rc_shoot.right_fric.set_currunt,rc_shoot.trigger.set_currunt);
	//canTX_gimbal(gimbal_p.given_current,gimbal_y.given_current);
	prevent_jam();
}
void shoot_init(void)
{
	
	PID_Init(&rc_shoot.trigger.speed_pid,TriggerSpeed);
	PID_Init(&rc_shoot.trigger.angle_pid,TriggerAngle);
	
  PID_Init(&rc_shoot.left_fric.speed_pid,FricLeftSpeed);
	PID_Init(&rc_shoot.right_fric.speed_pid,FricRightSpeed);
	
	rc_shoot.trigger.actual_angle = 0.0f;
	rc_shoot.trigger.actual_speed = 0.0f;
	
	rc_shoot.trigger.last_shoot_flag = 0;
	rc_shoot.trigger.last_back_flag = 0;
	rc_shoot.trigger.target_angle = 4365.0f;
	rc_shoot.trigger.target_speed = 0.0f;
	
	rc_shoot.left_fric.actual_speed = 0.0f;
	rc_shoot.left_fric.target_speed = 0;
	
	rc_shoot.right_fric.actual_speed = 0.0f;
	rc_shoot.right_fric.target_speed = 0;

}
//1987
static void shoot_angle_clc(void)
{
	if(rc_shoot.trigger.last_shoot_flag==0)
	{
		if(One_Shoot_flag==1||Ten_Shoot_flag==1)  
		{
			rc_shoot.trigger.last_shoot_flag=1;
			trigger_angle_set();
			shoot_times+=1;
			if(shoot_times>6) shoot_times=0;
		}
	}
	
	if(One_Shoot_flag!=1 && Ten_Shoot_flag!=1) rc_shoot.trigger.last_shoot_flag=0;

}

static void fric_pid(void)
{
	PID_Calc(&rc_shoot.left_fric.speed_pid,rc_shoot.left_fric.target_speed,rc_shoot.left_fric.actual_speed);
	
	rc_shoot.left_fric.set_currunt=rc_shoot.left_fric.speed_pid.out;
	
	PID_Calc(&rc_shoot.right_fric.speed_pid,rc_shoot.right_fric.target_speed,rc_shoot.right_fric.actual_speed);
	
	rc_shoot.right_fric.set_currunt=rc_shoot.right_fric.speed_pid.out;
}


static void trigger_pid(void)
{
	PID_Calc(&rc_shoot.trigger.angle_pid,rc_shoot.trigger.target_angle,rc_shoot.trigger.total_angle);
	
	rc_shoot.trigger.target_speed = rc_shoot.trigger.angle_pid.out;
	
	PID_Calc(&rc_shoot.trigger.speed_pid,rc_shoot.trigger.target_speed,rc_shoot.trigger.actual_speed);
	
	rc_shoot.trigger.set_currunt = rc_shoot.trigger.speed_pid.out;
}
int trigger_cnt=0,trigger_cnt_flag=0;
void trigger_angle_set(void)
{
	float angle_max;
//	trigger_cnt_flag=1;
//	int angle_min;
	//angle_max=((float)(1.0f*36.0f*36.0f)/360.0f*8191.0f);
	angle_max=29485.6f+19200.0f;
	trigger_cnt_flag=1;

//	angle_min=((SHOOT_NUM*36*36)/360*8191);
	
	if(rc_shoot.trigger.rounds>=90)
	{
		rc_shoot.trigger.target_angle-=rc_shoot.trigger.rounds*8192;
		if(rc_shoot.trigger.target_angle<=0) rc_shoot.trigger.target_angle=1;
		rc_shoot.trigger.total_angle -=rc_shoot.trigger.rounds*8192;
	    rc_shoot.trigger.begin_angle=rc_shoot.trigger.total_angle;
		rc_shoot.trigger.rounds=0;
	}
	
	if(One_Shoot_flag==1)
	{
		rc_shoot.trigger.target_angle+=angle_max;
	}
	else 
	{
		rc_shoot.trigger.target_angle+=((SHOOT_NUM*10*36*36)/360*8191);
	}
	
	
	
	
}

void Trigger_Motor_Callback(trigger_t *motor,uint16_t angle, int16_t speed)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->last_speed = speed;

	if(motor->record_begin_angle_status==0)
	{
		motor->begin_angle = angle;
		motor->record_begin_angle_status++;
	}
	if(motor->actual_angle - motor->last_angle > 4896)
		motor->rounds --;
	else if (motor->actual_angle - motor->last_angle < -4896)
		motor->rounds ++;
	motor->total_angle = motor->rounds * 8192 + motor->actual_angle;

	
}

