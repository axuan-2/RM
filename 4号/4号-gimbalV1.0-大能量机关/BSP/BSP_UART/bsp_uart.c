#include "bsp_uart.h"
#include "gimbal_task.h"
#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "usart.h"
#include "vision_task.h"

//union Receive_data{int get_data[2];char char_data[8];}receive_data;
uint8_t receive_data[20];
union Send_Data{int data[12];char char_data[8];} send_data;
uint8_t start_receive_flag = 0;

short int F_move_flag=0;		// �ĸ�����ı�ʶ��
short int L_move_flag=0;
float VISION_SHOOT_TIME=0.0f;
static int i = 0;
float a,b = 0;

DIRECTION WIND_DIRECTION=CLOCKWISE;

uint8_t ch;
int int_get[2];
int iii=-1;
uint8_t dma_rx_buff[10];

void uart_init(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN); 	
}
void SHOOT_DIRECTION(void)
{
	vision_sent.yaw.target_angle=vision_yaw;
	vision_sent.pitch.target_angle=vision_pitch;
    if(F_move_flag==1) //����
	{
		vision_sent.yaw.target_angle =vision_yaw+R_UP_YAW;
		vision_sent.pitch.target_angle =vision_pitch+R_UP_PITCH;
	}
	if(F_move_flag==2) //����
	{
		vision_sent.yaw.target_angle =vision_yaw+L_UP_YAW;
		vision_sent.pitch.target_angle =vision_pitch+L_UP_PITCH;
	}
	if(F_move_flag==3) //����
	{
		vision_sent.yaw.target_angle =vision_yaw+L_DOWN_YAW;
		vision_sent.pitch.target_angle =vision_pitch+L_DOWN_PITCH;
	}
	if(F_move_flag==4) //����
	{
		vision_sent.yaw.target_angle =vision_yaw+R_DOWN_YAW;
		vision_sent.pitch.target_angle =vision_pitch+R_DOWN_PITCH;
	}
//	F_move_flag=0;
}

uint8_t length=0;

int vision_t=0,bsp_vision_flag=0;
void USART1_IRQHandler(void)
{
	length=10;
	int j;
	 if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)
	 {
		 __HAL_UART_CLEAR_IDLEFLAG(&huart1);
		 HAL_UART_DMAStop(&huart1);
		 length=DMA_REC_LEN-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
		 for(j=0;i<length;j++)
		 {
			 if(dma_rx_buff[j] == ';')//��������
				{
					if(bsp_vision_flag==0)
					{
						F_move_flag=(int16_t)(receive_data[1]<<8|receive_data[0]);
					    F_move_flag++;
					    VISION_SHOOT_TIME=(int16_t)(receive_data[3]<<8|receive_data[2]);
						bsp_vision_flag=1;
//						if(VISION_SHOOT_TIME<=40) {F_move_flag=0;VISION_SHOOT_TIME=0;bsp_vision_flag=0;}
					}
								
//					vision_sent.yaw.target_angle =(int16_t)(receive_data[5]<<8|receive_data[4])/100.0f;
//					vision_sent.pitch.target_angle =(int16_t)(receive_data[7]<<8|receive_data[6])/100.0f;
					SHOOT_DIRECTION();
					start_receive_flag = 0;
					i = 0;
					break;
				}
			 if(start_receive_flag == 1)   //��������ת��
      { 
				if(i<9)
				{
					receive_data[i]=dma_rx_buff[j];
				  i++;
				}
	    }
	    if(dma_rx_buff[j] == '*')//��ʼ����
	    {
		    start_receive_flag = 1;
	    }

		 }
		 HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN);
	 }
}

uint8_t Buffer[14],status;
VISION_RESET VISION_RESET_FLAG=OFF;
int Sent_dataA = 0;
int Sent_dataB = 0;
int Sent_dataC = 0;
int Sent_dataD = 0;
int Sent_dataE = 0;

void DMA_Send(void)
{ 
	//c=100,d=200;
//	vision_getSpeed();
	Sent_dataA=WIND_DIRECTION;
    SHOOT_DIRECTION();
	Sent_dataB=VISION_RESET_FLAG;


	
	Buffer[0] = '*';
	Buffer[2] = (Sent_dataA>>8);
	Buffer[1] =  Sent_dataA&0xff;
//	Buffer[4] = (Sent_dataB>>8);
//	Buffer[3] =  Sent_dataB&0xff;
//	Buffer[6] = (Sent_dataC>>8);
//	Buffer[5] =  Sent_dataC&0xff;
//	Buffer[8] = (Sent_dataD>>8);
//	Buffer[7] =  Sent_dataD&0xff;
//	Buffer[10] = (Sent_dataE>>8);
//	Buffer[9] =  Sent_dataE&0xff;

	  for(int i=5;i<13;i++)
	{
	  Buffer[i] = 0;
	}

//  for(int i=1;i<(7-1);i++)
//	{
//	  Buffer[i] = send_data.data[i-1];
//	}
	Buffer[13] = ';';
	status=HAL_UART_Transmit(&huart1,Buffer,14,0xff);

}
