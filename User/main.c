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
int main(void)
{

	OLED_Init();
	OLED_Init();
	Motor_Init();	
	Encoder_Init();	
	Serial_Init();
	Timer_Init();
	OLED_Printf(0, 0, OLED_8X16, "Speed Control");
	OLED_Update();
	Target=0;
	while (1)
	{
		Kp=1.5, Ki=0.3, Kd=0.1;
		OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Kp);	
		OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Ki);	
		OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Kd);	
		
		OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", Target);
		OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", Actual);
		OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", Out);	
		
		OLED_Update();
		
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
		if (Count >= 10)	
		{
			Count = 0;	
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
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

