/*
 * SimpleParser.c
 *
 *  Created on: Aug 28, 2024
 *      Author: Szymon&Piotr
 */


#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "RingBuffer.h"
#include "stdlib.h"
#include "SimpleParser.h"
#include "Line_Follower.h"


void Parser_TakeLine(RingBuffer_t *Buf, uint8_t *ReceivedData)
{
	uint8_t Tmp;
	uint8_t i = 0;
	do
	{
		RB_Read(Buf, &Tmp);

		if(Tmp == ENDLINE)
		{
			ReceivedData[i] = 0;

		}

		else
		{
			ReceivedData[i] = Tmp;
		}
		i++;

	} while(Tmp != ENDLINE);

}

static void kp_change(LineFollower_t *LF)
{
	char *ParsePointer = strtok(NULL, ",");

	if(strlen(ParsePointer) > 0 && strlen(ParsePointer) < 32)
	{
		LF->Kp = atof(ParsePointer);
	}

}
static void kd_change(LineFollower_t *LF)
{
	char *ParsePointer = strtok(NULL, ",");

	if(strlen(ParsePointer) > 0 && strlen(ParsePointer) < 32)
	{
		LF->Kd = atof(ParsePointer);
	}
}
static void Base_speed_change(LineFollower_t *LF)
{
	char *ParsePointer = strtok(NULL, ",");

	if(strlen(ParsePointer) > 0 && strlen(ParsePointer) < 32)
	{
		LF->Base_speed_R = atof(ParsePointer);
		LF->Base_speed_L = atof(ParsePointer);
	}
}

static void Max_speed_change(LineFollower_t *LF)
{
	char *ParsePointer = strtok(NULL, ",");

	if(strlen(ParsePointer) > 0 && strlen(ParsePointer) < 32)
	{
		LF->Max_speed_R = atof(ParsePointer);
		LF->Max_speed_L = atof(ParsePointer);
	}
}
static void Sharp_bend_speed_right_change(LineFollower_t *LF)
{
	char *ParsePointer = strtok(NULL, ",");

	if(strlen(ParsePointer) > 0 && strlen(ParsePointer) < 32)
	{
	LF->Sharp_bend_speed_right = atof(ParsePointer);
	}
}
static void Sharp_bend_speed_left_change(LineFollower_t *LF)
{
	char *ParsePointer = strtok(NULL, ",");

	if(strlen(ParsePointer) > 0 && strlen(ParsePointer) < 32)
	{
	LF->Sharp_bend_speed_left = atof(ParsePointer);
	}
}
static void Bend_speed_right_change(LineFollower_t *LF)
{
	char *ParsePointer = strtok(NULL, ",");

	if(strlen(ParsePointer) > 0 && strlen(ParsePointer) < 32)
	{
	LF->Bend_speed_right = atof(ParsePointer);
	}
}
static void Bend_speed_left_change(LineFollower_t *LF)
{
	char *ParsePointer = strtok(NULL, ",");

	if(strlen(ParsePointer) > 0 && strlen(ParsePointer) < 32)
	{
	LF->Bend_speed_left = atof(ParsePointer);
	}
}
static void App_Controll(char RxData, LineFollower_t *LineFollower)
{
	/*Stop robot*/
	if(RxData == 'N')
	{
		/*Stop GRUZIK2.0 and turn off the LED*/
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);

		/*Send battery percentage*/
		//SN_UART_Send(&huart1, "%.1f \r \n" ,battery_procentage_raw);

	}
	/*Start robot*/
	if (RxData == 'Y')
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
	}

 	  //low
 	  if(RxData == 'n')
 	  {
 	    LineFollower->Base_speed_R = 116;
 	    LineFollower->Base_speed_L = 116;
 	    LineFollower->Max_speed_L = 167;
 	  	LineFollower->Max_speed_R = 167;
 	 	LineFollower->Sharp_bend_speed_right = -96;
 		LineFollower->Sharp_bend_speed_left = 185;
 		LineFollower->Bend_speed_right = -50;
 		LineFollower->Bend_speed_left = 100;
 		LineFollower->Kp = 0.02;
 		LineFollower->Kd = 350;
 	  }
 	  //medium
 	  if(RxData == 'h')
 	  {
 	    LineFollower->Base_speed_R = 123;
 	    LineFollower->Base_speed_L = 123;
 	    LineFollower->Max_speed_L = 172;
 	    LineFollower->Max_speed_R = 172;
 	 	LineFollower->Sharp_bend_speed_right = -90;
 		LineFollower->Sharp_bend_speed_left = 185;
 		LineFollower->Bend_speed_right = -50;
 		LineFollower->Bend_speed_left = 100;
 		LineFollower->Kp = 0.02;
 		LineFollower->Kd = 350;
 	  }
 	  //fast
 	  if(RxData == 'o')
 	  {
 	    LineFollower->Base_speed_R = 143;
 	    LineFollower->Base_speed_L = 143;
 	   	LineFollower->Max_speed_L = 182;
 	  	LineFollower->Max_speed_R = 182;
 	  	LineFollower->Sharp_bend_speed_right = -76;
 		LineFollower->Sharp_bend_speed_left = 90;
 		LineFollower->Bend_speed_right = -50;
 		LineFollower->Bend_speed_left = 100;
 		LineFollower->Kp = 0.02;
 		LineFollower->Kd = 350;
 	   }
 	   //rura
 	   if(RxData == 'u')
 	   {
 	     LineFollower->Base_speed_R = 153;
 	     LineFollower->Base_speed_L = 153;
 	     LineFollower->Max_speed_L = 187;
 	   	 LineFollower->Max_speed_R = 187;
 	  	 LineFollower->Sharp_bend_speed_right = -76;
 	 	 LineFollower->Sharp_bend_speed_left = 90;
 	 	 LineFollower->Bend_speed_right = -50;
 	 	 LineFollower->Bend_speed_left = 100;
 	 	 LineFollower->Kp = 0.02;
 	 	 LineFollower->Kd = 350;
 	   }
 	  //full rura
 	  if(RxData == 'p')
 	  {
 		 LineFollower->Base_speed_R = 143;
 		 LineFollower->Base_speed_L = 143;
 		 LineFollower->Max_speed_L = 182;
 		 LineFollower->Max_speed_R = 182;
 		 LineFollower->Sharp_bend_speed_right = -76;
 		 LineFollower->Sharp_bend_speed_left = 90;
 		 LineFollower->Bend_speed_right = -40;//40
 		 LineFollower->Bend_speed_left = 110;
 		 LineFollower->Kp = 0.02;
 		 LineFollower->Kd = 350;
 	  }
}

static void Mode_change(LineFollower_t *LF)
{
	char *ParsePointer = strtok(NULL, ",");

	if(strlen(ParsePointer) > 0 && strlen(ParsePointer) < 2)
	{
		App_Controll(ParsePointer[0], LF);
	}
}
void Parser_Parse(uint8_t *ReceivedData, LineFollower_t *LineFollower)
{
	char *ParsePointer = strtok((char*)ReceivedData, "=");

	if(!strcmp("Kp",ParsePointer))
	{
		kp_change(LineFollower);
	}
	else if(!strcmp("Kd",ParsePointer))
	{
		kd_change(LineFollower);
	}
	else if(!strcmp("Base_speed",ParsePointer))
	{
		Base_speed_change(LineFollower);
	}
	else if(!strcmp("Max_speed",ParsePointer))
	{
		Max_speed_change(LineFollower);
	}
	else if(!strcmp("Sharp_bend_speed_right",ParsePointer))
	{
		Sharp_bend_speed_right_change(LineFollower);
	}
	else if(!strcmp("Sharp_bend_speed_left",ParsePointer))
	{
		Sharp_bend_speed_left_change(LineFollower);
	}
	else if(!strcmp("Bend_speed_right",ParsePointer))
	{
		Bend_speed_right_change(LineFollower);
	}
	else if(!strcmp("Bend_speed_left",ParsePointer))
	{
		Bend_speed_left_change(LineFollower);
	}
	else if(!strcmp("Mode",ParsePointer))
	{
		Mode_change(LineFollower);
	}
}
