#include "chassis_move.h"
#include "fuzzy_pid.h"

static float chassis_follow(void);
static void chassis_speed_control(float speed_x, float speed_y, float speed_r);
static float chassis_power_loop(uint16_t target_power,float actual_power,float last_power);
static void chassis_move_mode(void);
static void can_send_chassis_current(void);
static void power_limitation_jugement(void);
static float chassis_buffer_loop(uint16_t buffer);
static void chassis_fly(uint16_t buffer);
static void speed_optimize(void);
static float Get_chassis_theta(void);
CHASSIS_CONTROL_ORDER_t chassis_control_order;
MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4,chassis_center;
POWER_PID_t p_pid;
BUFFER_PID_t b_pid;
REAl_CHASSIS_SPEED_t real_chassis_speed;
uint8_t fly_flag;
int8_t max_d_speed_x;
int8_t max_d_speed_y;
float vx,vy,wz;
float avx,avy,awz;
float lvx,lvy,lwz;
float last_vx,last_vy,last_wz;

STEPSTAR step_flag;
/**
  * @breif         底盘运动函数
  * @param[in]     none 
	* @param[out]    none
  * @retval        none     
  */
void chassis_move(void)
{
	//优化速度
//	speed_optimize();
	//模式选择
	chassis_move_mode();
	//功率限制
	power_limitation_jugement();
	//pid运算
	vpid_chassis_realize();
	
	
	//发送电流
	can_send_chassis_current();
}
/**
  * @breif         获取云台与底盘之间的夹角
  * @param[in]     none
	* @param[out]    云台与底盘之间的夹角(弧度制)
  * @retval        none     
  */
static float Get_chassis_theta(void)
{
	float temp,temp2,angle;
	if(chassis_center.actual_angle<GIMBAL_HEAD_ANGLE)
		temp=chassis_center.actual_angle+360.0f;
	else temp=chassis_center.actual_angle;
	temp2=temp-GIMBAL_HEAD_ANGLE;	
	angle=temp2/360.0f*2*PI;
	return angle;
}
float theta; 
void chassis_spin(float *vx,float *vy) 
{					
	   
	theta=Get_chassis_theta(); 
	*vx = (float)(avy*sin(theta) + avx*cos(theta)); 
	*vy = (float)(avy*cos(theta) - avx*sin(theta));   
}
  

/**
  * @breif         底盘功率限制
  * @param[in]     none 
	* @param[out]    输出限制后的四个电机电流值
  * @retval        none     
  */
float current_scale,BUFFER_MAX=60.0f,POWER_TOTAL_CURRENT_LIMIT=9000.0f;
float temp3,temp1,temp2,speed1,speed2,speed3,speed4,total_current_limit,total_current,power,last_power;
float power_scale,buffer_scale;
uint16_t max_power,buffer;
float yuuu=0;
 static void power_limitation_jugement(void)
{
	total_current=0;
	last_power=power;
	get_chassis_power_and_buffer_and_max(&power,&buffer,&max_power);
	
	

	power_scale=chassis_power_loop(max_power-8,power,last_power);
	buffer_scale=chassis_buffer_loop(buffer);
	temp1=chassis_motor1.pid.speed_loop.vpid.target_speed*buffer_scale*power_scale;
	chassis_motor1.pid.speed_loop.vpid.target_speed=(int16_t)temp1;
	temp1=chassis_motor2.pid.speed_loop.vpid.target_speed*buffer_scale*power_scale;
	chassis_motor2.pid.speed_loop.vpid.target_speed=(int16_t)temp1;
	temp1=chassis_motor3.pid.speed_loop.vpid.target_speed*buffer_scale*power_scale;
	chassis_motor3.pid.speed_loop.vpid.target_speed=(int16_t)temp1;
	temp1=chassis_motor4.pid.speed_loop.vpid.target_speed*buffer_scale*power_scale;
	chassis_motor4.pid.speed_loop.vpid.target_speed=(int16_t)temp1;
	if(power>max_power*2)CHASSIS_vPID_max=5000;
	else if(power<max_power*1.6)CHASSIS_vPID_max=9000;
	if(buffer<BUFFER_MAX*0.17) CHASSIS_vPID_max=2000;
	if(CHASSIS_vPID_max<=2500 && buffer>=BUFFER_MAX*0.50f ) CHASSIS_vPID_max=5000;
	if(CHASSIS_vPID_max<=5500 && buffer>=BUFFER_MAX*0.96f ) CHASSIS_vPID_max=9000;

}
/**
  * @breif         底盘功率环函数
  * @param[in]     target_power：设定的目标值
	* @param[in]     target_power：返回的真实值  
	* @param[in]     last_power：上一次返回的真实值
	* @param[out]    四个电机的输出电流
  * @retval        none     
  */
static float chassis_power_loop(uint16_t target_power,float actual_power,float last_power)
{
	float temp;
	p_pid.target_power=(float)target_power;
	p_pid.actual_power=actual_power;
	//此处进行pid运算
	power_pid_realize(&p_pid);
	//此处计算比例系数
	temp=1.07+((float)p_pid.PID_OUT/1000);
//	if(temp>1.2f)  temp-=0.2f;
//	temp=temp/2.0*0.45f+0.1f;
	temp*=0.8;

	return temp;
}

static float chassis_buffer_loop(uint16_t buffer)
{
	float temp;
	b_pid.target_buffer=50;
	b_pid.actual_buffer=buffer;
	buffer_pid_realize(&b_pid);
	temp=1.07-((float)b_pid.PID_OUT/1000.0f);
	temp*=0.83;
//	if(temp>1.2f)  temp-=0.2f;

	return temp;
}

/**
  * @breif         底盘飞坡函数，防止因飞坡后缓冲能量用完
  * @param[in]     buffer：底盘缓冲能量
	* @param[out]    四个电机的输出电流
  * @retval        none     
  */
static void chassis_fly(uint16_t buffer)
{
	if(buffer<20)
	{
		chassis_motor1.pid.speed_loop.vpid.PID_OUT*=0.5f;
		chassis_motor2.pid.speed_loop.vpid.PID_OUT*=0.5f;
		chassis_motor3.pid.speed_loop.vpid.PID_OUT*=0.5f;
		chassis_motor4.pid.speed_loop.vpid.PID_OUT*=0.5f;
	}
}

/**
  * @breif         中心速度优化，让速度变化变平稳
  * @param[in]     chassis_control_order.vx_set：x方向速度
	* @param[in]     chassis_control_order.vy_set：y方向速度
	* @param[out]    real_chassis_speed.real_vx：优化后x方向速度
	* @param[out]    real_chassis_speed.real_vy：优化后y方向速度
  * @retval        none     
  */
static void speed_optimize(void)
{
	
	static int16_t last_xspeed,last_yspeed;
	real_chassis_speed.real_vx=chassis_control_order.vx_set;
	real_chassis_speed.real_vy=chassis_control_order.vy_set;
	
	if(chassis_control_order.vx_set>last_xspeed)
		max_d_speed_x=5;
	else if(chassis_control_order.vx_set<-last_xspeed)
		max_d_speed_x=10;
	if(abs(chassis_control_order.vx_set-last_xspeed)>max_d_speed_x)
	{
		if(chassis_control_order.vx_set>last_xspeed)
			real_chassis_speed.real_vx+=max_d_speed_x;
		else if(chassis_control_order.vx_set<-last_xspeed)
			real_chassis_speed.real_vx-=max_d_speed_x;
	}

	if(chassis_control_order.vy_set>last_yspeed)
		max_d_speed_y=5;
	else if(chassis_control_order.vy_set<-last_yspeed)
		max_d_speed_y=10;	
	if(abs(chassis_control_order.vy_set-last_yspeed)>max_d_speed_y)
	{
		if(chassis_control_order.vy_set>last_yspeed)
			real_chassis_speed.real_vx+=max_d_speed_y;
		else if(chassis_control_order.vy_set<-last_yspeed)
			real_chassis_speed.real_vy-=max_d_speed_y;
	}
	
	last_xspeed=real_chassis_speed.real_vx;
	last_yspeed=real_chassis_speed.real_vy;
}


/**
  * @breif         运动学分解，将底盘中心的速度转换为四个轮子的速度
  * @param[in]     speed_x：x方向速度
	* @param[in]     speed_y：y方向速度
	* @param[in]     speed_r：自转速度
	* @param[out]    四个电机的目标速度
  * @retval        none     
  */
static void chassis_speed_control(float speed_x, float speed_y, float speed_r)
{
	int max;
		//速度换算，运动学分解
	BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
	
	max=find_max();
	if(max>MAX_MOTOR_SPEED)
	{
		chassis_motor1.target_speed=(int)(chassis_motor1.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor2.target_speed=(int)(chassis_motor2.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor3.target_speed=(int)(chassis_motor3.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor4.target_speed=(int)(chassis_motor4.target_speed*MAX_MOTOR_SPEED*1.0/max);
	}
	set_chassis_speed(chassis_motor1.target_speed, chassis_motor2.target_speed, chassis_motor3.target_speed, chassis_motor4.target_speed);
}	

/**
  * @breif         跟随模式，通过角度环将目标角度转换为目标速度
  * @param[in]     none
	* @param[out]    底盘自转速度
  * @retval        none     
  */
static float chassis_follow(void)
{
	//云台枪口对应的角度值
	chassis_center.pid.position_loop.apid.target_angle=GIMBAL_HEAD_ANGLE;
	chassis_center.pid.position_loop.apid.actual_angle=chassis_control_order.gimbal_6020_angle;
	follow_pid_realize();
	return (float)chassis_center.pid.position_loop.apid.PID_OUT;
}

float K_VX,K_VY,B_VX,B_VY; //分别代表K和B 一次函数
int step_times_x=0,step_times_y=0; //时间
float TIME_LIMIT=250; //斜坡的时间 
int STEP_VALUE=50; //差值大于step_value就用斜坡
//斜坡函数状态判断
void step_flag_judge(float VX_,float VY_,float LAST_VX_,float LAST_VY_)
{
	if(step_flag==NO_STEP)
	{
		if(abs(VX_-LAST_VX_)>STEP_VALUE&&abs(VX_)>100) step_flag=X_STEP;
	    else if(abs(VY_-LAST_VY_)>STEP_VALUE&&abs(VY_)>90) step_flag=Y_STEP;
		return;
	}
	if(step_flag==X_STEP)
	{
		if(step_times_x>TIME_LIMIT)
	    {
		    step_times_x=0;
		    step_flag=NO_STEP;
			return;
	    }
		 if(abs(VX_)<=2.0f) 
	     {
		    step_flag=NO_STEP;
		    step_times_x=0;
			return;
	     }
		if(abs(VY_-LAST_VY_)>STEP_VALUE&&abs(VY_)>90) 
		{
			step_flag=XY_STEP;
		}
		return;
	}
	
	if(step_flag==Y_STEP)
	{
		if(step_times_y>TIME_LIMIT)
	    {
		    step_times_y=0;
		    step_flag=NO_STEP;
			return;
	    }
		 if(abs(VY_)<=2.0f) 
	     {
		    step_flag=NO_STEP;
		    step_times_y=0;
			return;
	     }
		if(abs(VX_-LAST_VX_)>STEP_VALUE&&abs(VX_)>90) 
		{
			step_flag=XY_STEP;
		}
		return;
	}
	
	if(step_flag==XY_STEP)
	{
		if(step_times_y>TIME_LIMIT &&step_times_x>TIME_LIMIT)
	    {
		    step_times_y=0;
			step_times_x=0;
		    step_flag=NO_STEP;
			return;
	    }
		if(step_times_x>TIME_LIMIT)
	    {
		    step_times_x=0;
		    step_flag=Y_STEP;
			return;
	    }
		if(step_times_y>TIME_LIMIT)
	    {
		    step_times_y=0;
		    step_flag=X_STEP;
			return;
	    }
		 if(abs(VY_)<=2.0f&&abs(VX_)<=2.0f) 
	     {
		    step_flag=NO_STEP;
		    step_times_y=0;
			 step_times_x=0;
			 return;
	     }
		 if(abs(VY_)<=2.0f) 
	     {
		    step_flag=X_STEP;
		    step_times_y=0;
	     }
		 if(abs(VX_)<=2.0f) 
	     {
		    step_flag=Y_STEP;
		    step_times_x=0;
	     }
		return;
	}
	
	
}

void step_star(float *VX_,float *VY_,float LAST_VX_,float LAST_VY_)
{
	step_flag_judge(*VX_,*VY_,LAST_VX_,LAST_VY_);

	if(step_flag==NO_STEP)  return;

	
	if(step_flag==X_STEP)
	{
		step_times_x++;
		if(step_times_x<=1)
		{
			K_VX=(*VX_-LAST_VX_)/TIME_LIMIT;
			B_VX=LAST_VX_;
		}
		
		*VX_=(float)(K_VX*(float)step_times_x)+B_VX;
		
	}
	if(step_flag==Y_STEP)
	{
		step_times_y++;
		if(step_times_y<=Y_STEP)
		{
			K_VY=(*VY_-LAST_VY_)/TIME_LIMIT;
			B_VY=LAST_VY_;
		}
		
		*VY_=K_VY*(float)step_times_y+B_VY;
	}
	if(step_flag==XY_STEP)
	{
		step_times_y++;
		if(step_times_y<=Y_STEP)
		{
			K_VY=(*VY_-LAST_VY_)/TIME_LIMIT;
			B_VY=LAST_VY_;
		}
		step_times_x++;
		if(step_times_x<=1)
		{
			K_VX=(*VX_-LAST_VX_)/TIME_LIMIT;
			B_VX=LAST_VX_;
		}
		
		*VX_=(float)(K_VX*(float)step_times_x)+B_VX;
		
		*VY_=K_VY*(float)step_times_y+B_VY;
	}
}


/**
  * @breif         选择底盘运动模式
  * @param[in]     none
	* @param[out]    底盘三个方向的速度
  * @retval        none     
  */
static void chassis_move_mode(void)
{

	vx=(float)chassis_control_order.vx_set;
	vy=(float)chassis_control_order.vy_set;
	wz=(float)chassis_control_order.wz_set;
	avx=vx;
	avy=vy;
	step_star(&avx,&avy,last_vx,last_vy);
	if(step_flag!=NO_STEP)
	{
		vx=avx;
	    vy=avy;
	}
	CHASSIS_vPID_max=9300;
//	chassis_control_order.chassis_mode=CHASSIS_NORMAL;
	switch(chassis_control_order.chassis_mode)
	{
		case CHASSIS_NORMAL:
		break;
		//case CHASSIS_NO_FORCE:
		case CHASSIS_NO_FORCE:
		{   wz=0;
			vx=0;
			vy=0;
			}
		break;
		case CHASSIS_FOLLOW:
		{			
			chassis_spin(&vx,&vy);
			wz=-1.0f*chassis_follow();
		}
		break;
		case CHASSIS_SPIN:
		{
			if(vx==0&&vy==0&&wz==0)
			{
				CHASSIS_vPID_max=9000;
			}
			chassis_spin(&vx,&vy);
			wz=2.0f;
		}
		break;
		default:break;

	}
//	chassis_control_order.last_vx_set=chassis_control_order.vx_set;
//	chassis_control_order.last_vy_set=chassis_control_order.vy_set;
//	chassis_control_order.last_wz_set=chassis_control_order.wz_set;
	last_vx=(float)chassis_control_order.vx_set;
	last_vy=(float)chassis_control_order.vy_set;
	//last_wz=(float)chassis_control_order.wz_set;
	chassis_speed_control(vx,vy,wz);
}

/**
  * @breif         发送四个电机的电流
  * @param[in]     none
	* @param[out]    四个电机的电流值
  * @retval        none     
  */
static void can_send_chassis_current(void)
{
	static uint8_t cdata[8];
	cdata[0]=(chassis_motor1.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[1]=(chassis_motor1.pid.speed_loop.vpid.PID_OUT)&0xFF;
	cdata[2]=(chassis_motor2.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[3]=(chassis_motor2.pid.speed_loop.vpid.PID_OUT)&0xFF;
	cdata[4]=(chassis_motor3.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[5]=(chassis_motor3.pid.speed_loop.vpid.PID_OUT)&0xFF;
	cdata[6]=(chassis_motor4.pid.speed_loop.vpid.PID_OUT)>>8;
	cdata[7]=(chassis_motor4.pid.speed_loop.vpid.PID_OUT)&0xFF;
	
	Can_Tx_Message(&hcan1,cdata);
}


