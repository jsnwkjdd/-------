#include "stm32f10x.h"                  // Device header

#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include <stdlib.h>
float Target, Actual, Out;		
float Kp, Ki, Kd;
float Error0=0, Error1=0, Error2=0;	
uint8_t KeyNum;
uint8_t p=0;
int main(void)
{

	OLED_Init();
	OLED_Init();
	Motor_Init();
	Motor2_Init();
	Key_Init();
	Encoder_Init();	
	Encoder_Init2();
	Serial_Init();
	Timer_Init();
	Target=0;
	Kp=5, Ki=1, Kd=3;
	OLED_Printf(0, 0, OLED_8X16, "Speed Control");
	OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Kp);	
	OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Ki);	
	OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Kd);	
	OLED_Update();
	while (1)
	{
		KeyNum = Key_GetNum();
		if(KeyNum==1)
		{
			Motor_SetSpeed(0);
			Motor2_SetSpeed(0);
			Target=0;
			Actual=0;
			Out = 0;
			Error0 = 0;
			Error1 = 0;
			Error2 = 0;
			p=1-p; 
			OLED_Clear();
			OLED_Update();
			if(p==0)
			{	
				Kp=7, Ki=1.8, Kd=2;
				OLED_Printf(0, 0, OLED_8X16, "Speed Control");
				OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Kp);	
				OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Ki);	
				OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Kd);	
				OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", Target);
				OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", Actual);
				OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", Out);	
				OLED_Update();
			}
			else{
					Kp=1.2, Ki=0.13, Kd=0.1;
					OLED_Printf(0, 0, OLED_8X16, "Location Control");
					OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Kp);	
					OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Ki);	
					OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Kd);
					OLED_Update();
			}
		}
		if(p==0)
			{	
				OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", Target);
				OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", Actual);
				OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", Out);	
				OLED_Update();
			}
		else{
			OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", Target);
			OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", Actual);
			OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", Out);	
			OLED_Update();
			}
		Serial_Printf("%f,%f,%f\r\n", Target, Actual, Out);	
		if(Serial_RxFlag == 1)
		{
			Target = (float)atof(Serial_RxPacket);
			Serial_RxFlag = 0;
		}
	}
}
void TIM1_UP_IRQHandler(void)
{

	static uint16_t Count;	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Count ++;
		Key_Tick();	
		if (Count >= 10){
			Count = 0;	
			if(p==0){
				Actual = Encoder_Get();
				Error2 = Error1;
				Error1 = Error0;	
				Error0 = Target - Actual;	
				Out += Kp * (Error0 - Error1) + Ki * Error0
					+ Kd * (Error0 - 2 * Error1 + Error2);
				if (Out > 100) {Out = 100;}	
				if (Out < -100) {Out = -100;}
				Motor_SetSpeed(Out);
			}
			else
			{	
				Target+=Encoder_Get();
				Actual+=Encoder_Get2();
				Error2 = Error1;
				Error1 = Error0;	
				Error0 = Target - Actual;	
				Out += Kp * (Error0 - Error1) + Ki * Error0
					+ Kd * (Error0 - 2 * Error1 + Error2);
				if (Out > 100) {Out = 100;}	
				if (Out < -100) {Out = -100;}
				Motor2_SetSpeed(Out);
			}
		}
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

