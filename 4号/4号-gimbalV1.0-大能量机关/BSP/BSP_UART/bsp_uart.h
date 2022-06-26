#ifndef __BSP_UART_H
#define __BSP_UART_H


#define DMA_REC_LEN    10
// -10.57 335
#define R_UP_YAW -4.18f
#define R_UP_PITCH -6.11f

#define R_DOWN_YAW -4.43f
#define R_DOWN_PITCH 1.49f

#define L_UP_YAW 4.23f
#define L_UP_PITCH -5.81f

#define L_DOWN_YAW 3.93f
#define L_DOWN_PITCH 2.19f
extern int vision_t,bsp_vision_flag;
extern short int F_move_flag;
extern short int L_move_flag;
void uart_init(void);
void DMA_Send(void);
extern float VISION_SHOOT_TIME;
typedef enum
{
	CLOCKWISE=0,
	ANTICL=1,
}DIRECTION;

typedef enum
{
	ON_RESET=0,
	OFF=1,
}VISION_RESET; //重启标志位

extern VISION_RESET VISION_RESET_FLAG;

extern DIRECTION WIND_DIRECTION;

#endif

