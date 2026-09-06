#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "NRF24L01.h"
#include "PID.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

uint8_t keynum;
uint8_t RunState;

int16_t AX,AY,AZ,GX,GY,GZ;
int16_t AvePWM,DifPWM,LPWM,RPWM;

float AngleAcc,AngleGyro,Angle;
float AveSpeed,DifSpeed,LSpeed,RSpeed;

PID_t AnglePID={
	.Kp=3,
	.Ki=0.1,
	.Kd=3,
	
	.Target=0,
	
	.ErrorIntMax=500,
	.ErrorIntMin=-500,
	.OutPut_Offset=4,	//左侧电机死区为4，右侧死区为5
	
	.OutMax=600,		//限制直立环输出平均PWM最大100
	.OutMin=-600,		//限制直立环输出平均PWM最小-100
};

PID_t SpeedPID={
	.Kp=2,
	.Ki=0.05,
	.Kd=0,
	
	.Target=0,
	
	.ErrorIntMax=150,
	.ErrorIntMin=-150,
	
	.OutMax=20,		//限制速度环输出最大15°
	.OutMin=-20,	//限制速度环输出最小-15°
};

PID_t YawPID={
	.Kp=4,
	.Ki=3,
	.Kd=0,
	
	.Target=0,
	
	.ErrorIntMax=+20,
	.ErrorIntMin=-20,
	
	.OutMax=50,		//限转向环输出差分PWM最大为100
	.OutMin=-50,	//限制速度环输出差分PWM最小-100
};


int main(void)
{
	OLED_Init();
	LED_Init();
	Timer_Init();
	Key_Init();
	MPU6050_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	BlueSerial_Init();
	NRF24L01_Init();

	while (1)
	{	
		/*--------------------------按键获取--------------------------*/	
		keynum=Key_GetNum();
		if(keynum==1)
		{
			RunState=!RunState;
			
			PID_Init(&AnglePID);
			PID_Init(&SpeedPID);
			PID_Init(&YawPID);
		}
		
		if(RunState==1)
		{
			LED_ON();
		}
		else
		{
			LED_OFF();
		}
		
		/*--------------------------遥控发送与接收--------------------------*/	
		if (NRF24L01_Receive() == 1)
		{
			uint8_t ID = NRF24L01_RxPacket[0];
			
			if (ID == 0x00 || ID == 0x01)
			{
				if (ID == 0x01)
				{
					NRF24L01_TxPacket[0] = 0x02;
					NRF24L01_TxPacket[1] = (int8_t)LPWM;
					NRF24L01_TxPacket[2] = (int8_t)RPWM;
					*(float *)&NRF24L01_TxPacket[4] = Angle;		
					*(float *)&NRF24L01_TxPacket[8] = LSpeed;		
					*(float *)&NRF24L01_TxPacket[12] = RSpeed;		
					
					NRF24L01_Send();
				}
				
//				int8_t LH = NRF24L01_RxPacket[1];
				int8_t LV = NRF24L01_RxPacket[2];
				int8_t RH = NRF24L01_RxPacket[3];
//				int8_t RV = NRF24L01_RxPacket[4];
				uint8_t KEY = NRF24L01_RxPacket[5];
				
				SpeedPID.Target = LV / 25.0;
				YawPID.Target = RH / 25.0;
				
				if (KEY == 1)
				{
					if (RunState == 0)
					{
						PID_Init(&AnglePID);
						PID_Init(&SpeedPID);
						PID_Init(&YawPID);
						RunState = 1;
					}
					else
					{
						RunState = 0;
					}
				}
			}
		}

		/*--------------------------蓝牙发送与接收--------------------------*/
		if (BlueSerial_RxFlag == 1)
		{	
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (strcmp(Tag, "key") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
				
				if (strcmp(Name, "1") == 0 && strcmp(Action, "up") == 0)
				{
					RunState=!RunState;
					AnglePID.ErrorInt=0;
					
					printf("key,1,up\r\n");
				}
				else if (strcmp(Name, "2") == 0 && strcmp(Action, "down") == 0)
				{
					printf("key,2,down\r\n");
				}
			}
			else if (strcmp(Tag, "slider") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");
				
				if (strcmp(Name, "AngleKp") == 0)
				{
					AnglePID.Kp = atof(Value);
				}
				else if (strcmp(Name, "AngleKi") == 0)
				{
					AnglePID.Ki = atof(Value);
				}
				else if (strcmp(Name, "AngleKd") == 0)
				{
					AnglePID.Kd = atof(Value);
				}
				else if (strcmp(Name, "SpeedKp") == 0)
				{
					SpeedPID.Kp = atof(Value);
				}
				else if (strcmp(Name, "SpeedKi") == 0)
				{
					SpeedPID.Ki = atof(Value);	
				}
				else if (strcmp(Name, "SpeedKd") == 0)
				{
					SpeedPID.Kd = atof(Value);
				}
				else if (strcmp(Name, "YawKp") == 0)
				{
					YawPID.Kp = atof(Value);
				}
				else if (strcmp(Name, "YawKi") == 0)
				{
					YawPID.Ki = atof(Value);		
				}
				else if (strcmp(Name, "YawKd") == 0)
				{
					YawPID.Kd = atof(Value);
				}
			}	
			else if (strcmp(Tag, "joystick") == 0)
			{
				int8_t LH = atoi(strtok(NULL, ","));
				int8_t LV = atoi(strtok(NULL, ","));
				int8_t RH = atoi(strtok(NULL, ","));
				int8_t RV = atoi(strtok(NULL, ","));
				
				SpeedPID.Target=LV/25.0;		//限制小车平均速度在-4Rounds/S~4Rounds/S之间变化
				YawPID.Target=RH/25.0;				//限制小车的差分速度在-4Rounds/S~4Rounds/S之间变化
			}
			
			BlueSerial_RxFlag = 0;
		}
		
		BlueSerial_Printf("[plot,%f,%f,%f]",AnglePID.ErrorInt,SpeedPID.ErrorInt,YawPID.ErrorInt);
		
		/*-------------------------串口发送与接收--------------------------*/
		Serial_Printf("%f,%f,%f,%f,%f,%f,%f\n",AngleAcc,AngleGyro,Angle,AnglePID.Out,AnglePID.Kp,AnglePID.Ki,AnglePID.Target);	
		
		/*--------------------------OLED显示--------------------------*/
		OLED_Printf(0,0,OLED_6X8,"AX=%+05d",AX);
		OLED_Printf(0,8,OLED_6X8,"AY=%+05d",AY);
		OLED_Printf(0,16,OLED_6X8,"AZ=%+05d",AZ);
		
		OLED_Printf(24,56,OLED_6X8,"Agl=+%05.2f",Angle);
		
		OLED_Printf(30,32,OLED_6X8,"LS=%+04.2f",LSpeed);
		OLED_Printf(30,40,OLED_6X8,"RS=%+04.2f",RSpeed);
		
		OLED_Printf(60,0,OLED_6X8,"GX=%+05d",GX);
		OLED_Printf(60,8,OLED_6X8,"GY=%+05d",GY);
		OLED_Printf(60,16,OLED_6X8,"GZ=%+05d",GZ);
		
		OLED_Update();
	}
}


void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0,Count1,Count2;
	static float Alpha=0.01;
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Key_Tick();

		/*--------------------------直立环--------------------------*/	
		Count0++;
		
		if(Count0>=10)
		{
			Count0=0;
			
			MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);
			
			AX-=20;		//零漂补偿
			AngleAcc=-atan2(AX,AZ)/3.14159*180;
			
			GY-=25;		//零漂补偿
			AngleGyro=Angle+GY/32768.0*2000*0.01;
			
			Angle= Alpha*AngleAcc+(1-Alpha)*AngleGyro;
			
			AnglePID.Actual=Angle;
			
			PID_Update(&AnglePID);
			
			AvePWM=-AnglePID.Out;

			/*--------------------------电机控制--------------------------*/		
			LPWM=AvePWM+DifPWM/2;
			RPWM=AvePWM-DifPWM/2;
			
			if (LPWM > 100) {LPWM = 100;} else if (LPWM < -100) {LPWM = -100;}
			if (RPWM > 100) {RPWM = 100;} else if (RPWM < -100) {RPWM = -100;}
			
			if(Angle>=50||Angle<=-50)
			{
				RunState=0;
			}
			
			if(RunState==1)
			{
				Motor_SetPWM(LPWM,1);
				Motor_SetPWM(RPWM,2);
			}
			else
			{
				Motor_SetPWM(0,1);
				Motor_SetPWM(0,2);
			}
		}

		/*--------------------------速度环--------------------------*/		
		Count1++;
		
		if(Count1>=50)
		{
			Count1=0;
			
			LSpeed=Encoder_Get(1)/0.05/44.0/9.2766;		//单位：转/秒,最大转速为14.11转每秒	
			RSpeed=Encoder_Get(2)/0.05/44.0/9.2766;		//单位：转/秒,最大转速为14.11转每秒
			
			AveSpeed=(LSpeed+RSpeed)/2.0;		//最大为14.11转每秒，最小为0转每秒
			
			SpeedPID.Actual=AveSpeed;
			
			PID_Update(&SpeedPID);
			
			AnglePID.Target=SpeedPID.Out;
		}
		
		/*--------------------------转向环--------------------------*/		
		Count2++;
		
		if(Count2>=50)
		{
			Count2=0;
			
			DifSpeed=LSpeed-RSpeed;		//最大为25转每秒，最小为-25转每秒
			
			YawPID.Actual=DifSpeed;
			
			PID_Update(&YawPID);
			
			DifPWM=YawPID.Out;
		}
	
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

	}
}

