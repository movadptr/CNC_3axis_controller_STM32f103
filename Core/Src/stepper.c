/*
 * stepper.c
 *
 *  Created on: Apr 24, 2024
 *  Author: Póti Szabolcs
 */

#include "main.h"
#include "stepper.h"
#include "math.h"
#include "stdlib.h"


volatile uint8_t switches=0;
CP stepper_pos = {0};

/*
 * Plot Absolute
 */
void HPGL_PA(uint32_t xcoord, uint32_t ycoord, uint32_t zcoord, CP* currentpos)
{
	int16_t xmove = xcoord - currentpos->current_pos_x;
	int16_t ymove = ycoord - currentpos->current_pos_y;
	int16_t zmove = zcoord - currentpos->current_pos_z;
	int8_t xdir = 0, ydir = 0, zdir = 0;

	if(xmove>0)
	{
		xdir = FORWARD;
	}
	else
	{
		xdir = BACKWARD;
		xmove = abs(xmove);
	}

	if(ymove>0)
	{
		ydir = FORWARD;
	}
	else
	{
		ydir = BACKWARD;
		ymove = abs(ymove);
	}

	if(zmove>0)
	{
		zdir = BACKWARD;
	}
	else
	{
		zdir = FORWARD;
		zmove = abs(zmove);
	}

	stepxyz3D((uint32_t)xmove, xdir, (uint32_t)ymove, ydir, (uint32_t)zmove, zdir, currentpos);
}

/*
 * Plot Relative
 */
void HPGL_PR(int32_t xmove, int32_t ymove, int32_t zmove,CP* currentpos)
{
	int8_t xdir = 0, ydir = 0, zdir = 0;

	if(xmove>0)
	{
		xdir = FORWARD;
	}
	else
	{
		xdir = BACKWARD;
		xmove = abs(xmove);
	}
	if(ymove>0)
	{
		ydir = FORWARD;
	}
	else
	{
		ydir = BACKWARD;
		ymove = abs(ymove);
	}
	if(zmove>0)
	{
		zdir = FORWARD;
	}
	else
	{
		zdir = BACKWARD;
		zmove = abs(zmove);
	}

	stepxyz3D((uint32_t)xmove, xdir, (uint32_t)ymove, ydir, (uint32_t)zmove, zdir, currentpos);
}

void get_limit_sw_state(void)
{
	if(LL_GPIO_IsInputPinSet(SW1_1_GPIO_Port, SW1_1_Pin))	{ switches |= sw11bitMSK;}
	else{ switches &= ~sw11bitMSK;}

	if(LL_GPIO_IsInputPinSet(SW2_1_GPIO_Port, SW2_1_Pin))	{ switches |= sw21bitMSK;}
	else{ switches &= ~sw21bitMSK;}

	if(LL_GPIO_IsInputPinSet(SW1_3_GPIO_Port, SW1_3_Pin))	{ switches |= sw13bitMSK;}
	else{ switches &= ~sw13bitMSK;}

	if(LL_GPIO_IsInputPinSet(SW2_3_GPIO_Port, SW2_3_Pin))	{ switches |= sw23bitMSK;}
	else{ switches &= ~sw23bitMSK;}

	if(LL_GPIO_IsInputPinSet(SW1_4_GPIO_Port, SW1_4_Pin))	{ switches &= ~sw14bitMSK;}
	else{ switches |= sw14bitMSK;}

	if(LL_GPIO_IsInputPinSet(SW2_4_GPIO_Port, SW2_4_Pin))	{ switches &= ~sw24bitMSK;}
	else{ switches |= sw24bitMSK;}
}

void gotozero(CP* currentpos)
{
	uint32_t gx=0, gy=0;

	while( (switches&sw14bitMSK) != sw14bitMSK)
	{
		stepz(1, BACKWARD, currentpos);
	}
	currentpos->current_pos_z = 0;

	while( (switches&(sw11bitMSK|sw13bitMSK)) != (sw11bitMSK|sw13bitMSK) )
	{
		if((switches&sw11bitMSK)==sw11bitMSK)	{ gx = 0;}	else{ gx = 1;}
		if((switches&sw13bitMSK)==sw13bitMSK)	{ gy = 0;}	else{ gy = 1;}
		stepxyz25D(gx, BACKWARD, gy, BACKWARD, 0, BACKWARD, currentpos);
	}
	stepxyz25D(100, FORWARD, 100, FORWARD, 100, FORWARD, currentpos);//step out of the limit sw proximity
	currentpos->current_pos_x = 0;
	currentpos->current_pos_y = 0;
	currentpos->current_pos_z = ZaxisLen;
	currentpos->error_state = 0;

	uint8_t str[] = {"Home pos"};
	ext_brd_transmit_string(PrintInfo_cmd, str, sizeof(str));
	msDelay(300);

	uint8_t str2[] = {"> "};
	ext_brd_transmit_string(PrintInfo_cmd, str2, sizeof(str2));
	msDelay(100);
}

void stepz(uint32_t z, int8_t dirz, CP* currentpos)
{
	uint32_t z_steps_taken = 0;

	if(dirz==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_4_GPIO_Port, DIR_4_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_4_GPIO_Port, DIR_4_Pin);}
	get_limit_sw_state();
	usDelay(1);

	while(  (z_steps_taken<z) && IfMovingAwayFromSwitch(dirz, sw14bitMSK, sw24bitMSK) )//ha nem a bedőlt limit switch felé megy akkor csinálhat steppet
	{
		LL_GPIO_SetOutputPin(STCK_4_GPIO_Port, STCK_4_Pin);//motor steps on rising edge
		z_steps_taken++;
		//stepdelay(currentpos);
		usDelay(800);
		LL_GPIO_ResetOutputPin(STCK_4_GPIO_Port, STCK_4_Pin);
		//stepdelay(currentpos);
		usDelay(800);
	}

	currentpos->current_pos_z += (z_steps_taken*dirz);
}

void stepxyz3D(uint32_t x, int8_t dirx, uint32_t y, int8_t diry, uint32_t z, int8_t dirz, CP* currentpos)
{
	axle aX={dirx, sw11bitMSK, sw21bitMSK, x, 0, STCK_1_GPIO_Port, STCK_1_Pin};
	axle aY={diry, sw13bitMSK, sw23bitMSK, y, 0, STCK_3_GPIO_Port, STCK_3_Pin};
	axle aZ={dirz, sw14bitMSK, sw24bitMSK, z, 0, STCK_4_GPIO_Port, STCK_4_Pin};

	float slope10 = 0;
	float slope21 = 0;
	float virtual_slope10 = 0;
	float virtual_slope21 = 0;

	uint32_t swap=1;
	axle* tmp;
	axle* axt[3]={&aX, &aY, &aZ};

	while(swap)//sorting, small to big
	{
		swap=0;
		if(axt[0]->steps > axt[1]->steps)	{tmp=axt[0]; axt[0]=axt[1]; axt[1]=tmp; swap=1;} else{}
		if(axt[1]->steps > axt[2]->steps)	{tmp=axt[1]; axt[1]=axt[2]; axt[2]=tmp; swap=1;} else{}
	}

	if(dirx==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_1_GPIO_Port, DIR_1_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_1_GPIO_Port, DIR_1_Pin);}
	if(diry==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_3_GPIO_Port, DIR_3_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_3_GPIO_Port, DIR_3_Pin);}
	if(dirz==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_4_GPIO_Port, DIR_4_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_4_GPIO_Port, DIR_4_Pin);}
	get_limit_sw_state();

	if((axt[0]->steps == 0) || (axt[1]->steps == 0))	{slope10 = 0;}	else{ slope10 = fabsf((float)axt[1]->steps / (float)axt[0]->steps);}
	if((axt[1]->steps == 0) || (axt[2]->steps == 0))	{slope21 = 0;}	else{ slope21 = fabsf((float)axt[2]->steps / (float)axt[1]->steps);}

	while(  (axt[2]->stepsTaken < axt[2]->steps) && IfMovingAwayFromSwitch(axt[2]->dir, axt[2]->sw1MSK, axt[2]->sw2MSK) )//ha nem a bedőlt limit switch felé megy akkor csinálhat steppet
	{
		LL_GPIO_SetOutputPin(axt[2]->GPIOport, axt[2]->GPIOpin);//motor steps on rising edge
		axt[2]->stepsTaken++;

		if(slope21 != 0)
		{
			virtual_slope21 = ((float)axt[2]->stepsTaken/(axt[1]->stepsTaken+1));

			if( (virtual_slope21>=slope21 ) && (axt[1]->stepsTaken<axt[1]->steps) && IfMovingAwayFromSwitch(axt[1]->dir, axt[1]->sw1MSK, axt[1]->sw2MSK))
			{
				LL_GPIO_SetOutputPin(axt[1]->GPIOport, axt[1]->GPIOpin);//motor steps on rising edge
				axt[1]->stepsTaken++;

				if(slope10 != 0)
				{
					virtual_slope10 = ((float)axt[1]->stepsTaken/(axt[0]->stepsTaken+1));

					if( (virtual_slope10>=slope10 ) && (axt[0]->stepsTaken<axt[0]->steps) && IfMovingAwayFromSwitch(axt[0]->dir, axt[0]->sw1MSK, axt[0]->sw2MSK))
					{
						LL_GPIO_SetOutputPin(axt[0]->GPIOport, axt[0]->GPIOpin);//motor steps on rising edge
						axt[0]->stepsTaken++;
					}
				}
			}
		}
		stepdelay(currentpos);
		LL_GPIO_ResetOutputPin(STCK_1_GPIO_Port, STCK_1_Pin);
		LL_GPIO_ResetOutputPin(STCK_3_GPIO_Port, STCK_3_Pin);
		LL_GPIO_ResetOutputPin(STCK_4_GPIO_Port, STCK_4_Pin);
		stepdelay(currentpos);
	}

	currentpos->current_pos_x += (aX.stepsTaken * aX.dir);
	currentpos->current_pos_y += (aY.stepsTaken * aY.dir);
	currentpos->current_pos_z -= (aZ.stepsTaken * aZ.dir);
}

void stepxyz25D(uint32_t x, int8_t dirx, uint32_t y, int8_t diry, uint32_t z, int8_t dirz, CP* currentpos)
{
	float slope = 0;
	float next_slope = 0;
	uint32_t x_steps_taken = 0;
	uint32_t y_steps_taken = 0;

	if(dirx==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_1_GPIO_Port, DIR_1_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_1_GPIO_Port, DIR_1_Pin);}
	if(diry==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_3_GPIO_Port, DIR_3_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_3_GPIO_Port, DIR_3_Pin);}
	get_limit_sw_state();
	stepz(z, dirz, currentpos);

	if(x>y)
	{
		if(y==0)	{ slope=0;}
		else{ slope = fabs((float)x/y);}

		while(  (x_steps_taken<x) && IfMovingAwayFromSwitch(dirx, sw11bitMSK, sw21bitMSK) )//ha nem a bedőlt limit switch felé megy akkor csinálhat steppet
		{
			LL_GPIO_SetOutputPin(STCK_1_GPIO_Port, STCK_1_Pin);//motor steps on rising edge
			x_steps_taken++;

			if(slope != 0)
			{
				next_slope = ((float)x_steps_taken/(y_steps_taken+1));

				if( (next_slope>=slope ) && (y_steps_taken<y) && IfMovingAwayFromSwitch(diry, sw13bitMSK, sw23bitMSK))
				{
					LL_GPIO_SetOutputPin(STCK_3_GPIO_Port, STCK_3_Pin);//motor steps on rising edge
					y_steps_taken++;
				}
			}
			stepdelay(currentpos);
			LL_GPIO_ResetOutputPin(STCK_1_GPIO_Port, STCK_1_Pin);
			LL_GPIO_ResetOutputPin(STCK_3_GPIO_Port, STCK_3_Pin);
			stepdelay(currentpos);
		}
	}
	else
	{
		if(x==0)	{ slope=0;}
		else{ slope = fabs((float)y/x);}

		while( (y_steps_taken<y) && IfMovingAwayFromSwitch(diry, sw13bitMSK, sw23bitMSK) )//ha nem a bedőlt limit switch felé megy akkor csinálhat steppet
		{
			LL_GPIO_SetOutputPin(STCK_3_GPIO_Port, STCK_3_Pin);//motor steps on rising edge
			y_steps_taken++;

			if(slope != 0)
			{
				next_slope = ((float)y_steps_taken/(x_steps_taken+1));

				if( (next_slope>=slope ) && (x_steps_taken<x) && IfMovingAwayFromSwitch(dirx, sw11bitMSK, sw21bitMSK))
				{
					LL_GPIO_SetOutputPin(STCK_1_GPIO_Port, STCK_1_Pin);//motor steps on rising edge
					x_steps_taken++;
				}
			}
			stepdelay(currentpos);
			LL_GPIO_ResetOutputPin(STCK_1_GPIO_Port, STCK_1_Pin);
			LL_GPIO_ResetOutputPin(STCK_3_GPIO_Port, STCK_3_Pin);
			stepdelay(currentpos);
		}
	}

	currentpos->current_pos_x += (x_steps_taken*dirx);
	currentpos->current_pos_y += (y_steps_taken*diry);
}

void stepdelay(CP* currentpos)
{
	switch(currentpos->curr_state & (FeedMove|RapidMove))
	{
		case RapidMove:		usDelay(800);
							break;

		case FeedMove:		switch(currentpos->movespeed)
							{
								case toolspeed_0:	usDelay(3000);
													break;
								case toolspeed_1:	usDelay(2000);
													break;
								case toolspeed_2:	usDelay(1600);
													break;
								case toolspeed_3:	usDelay(1200);
													break;
								case toolspeed_4:	usDelay(1000);
													break;
								case toolspeed_5:	usDelay(800);
													break;
								case toolspeed_6:	usDelay(750);
													break;
								case toolspeed_7:	usDelay(650);
													break;
								default:			usDelay(650);
													break;
							}
							break;

		default:	usDelay(800);
					break;
	}
}


/*
#define Z_axis_lowest_pos	0
#define Z_axis_highest_pos	200
void set_servo(int32_t zval, CP* currentpos)
{
	// 0 val -> -100 means tool is the lowest position
	// 200 val -> 100 means tool is in the top position
	if((zval<Z_axis_lowest_pos) || (zval>Z_axis_highest_pos))
	{
		currentpos->error_state |= ZaxisPositionError_MSK;
		Error_Handler();
	}
	else
	{
		if(currentpos->current_toolpos != zval)
		{
			servo_pos(zval-100,TIM2);
			currentpos->current_toolpos = zval;
			msDelay((uint32_t)abs((int)currentpos->current_toolpos - (int)zval) + 50);//+50 to give time for the servo
		}else{}
	}
}
*/

void STSPIN220_power_down(void)
{
	LL_GPIO_ResetOutputPin(STBY_RESET_ALL_GPIO_Port, STBY_RESET_ALL_Pin);
	LL_GPIO_ResetOutputPin(EN_FAULT_1_GPIO_Port, EN_FAULT_1_Pin);
	LL_GPIO_ResetOutputPin(EN_FAULT_3_GPIO_Port, EN_FAULT_3_Pin);
	LL_GPIO_ResetOutputPin(EN_FAULT_4_GPIO_Port, EN_FAULT_4_Pin);
}

void STSPIN220_init(CP *currentpos, uint16_t stepsizeX, uint16_t stepsizeY, uint16_t stepsizeZ)
{

	LL_GPIO_ResetOutputPin(STBY_RESET_ALL_GPIO_Port, STBY_RESET_ALL_Pin);
	LL_GPIO_ResetOutputPin(EN_FAULT_1_GPIO_Port, EN_FAULT_1_Pin);
	LL_GPIO_ResetOutputPin(EN_FAULT_3_GPIO_Port, EN_FAULT_3_Pin);
	LL_GPIO_ResetOutputPin(EN_FAULT_4_GPIO_Port, EN_FAULT_4_Pin);

	currentpos->curr_state &= ~(stepsizeX1_0 | stepsizeX1_2 | stepsizeX1_4 | stepsizeX1_8 | stepsizeY1_0 | stepsizeY1_2 | stepsizeY1_4 | stepsizeY1_8 | stepsizeZ1_0 | stepsizeZ1_2 | stepsizeZ1_4 | stepsizeZ1_8);

	switch(stepsizeX)
	{
		case stepsizeX1_0:	//X axis full step
							LL_GPIO_ResetOutputPin(MODE1_1_GPIO_Port, MODE1_1_Pin);
							LL_GPIO_ResetOutputPin(MODE2_1_GPIO_Port, MODE2_1_Pin);
							LL_GPIO_ResetOutputPin(STCK_1_GPIO_Port, STCK_1_Pin);
							LL_GPIO_ResetOutputPin(DIR_1_GPIO_Port, DIR_1_Pin);

							currentpos->curr_state |= stepsizeX1_0;
							break;

		case stepsizeX1_2:	//X axis full step
							LL_GPIO_SetOutputPin(MODE1_1_GPIO_Port, MODE1_1_Pin);
							LL_GPIO_ResetOutputPin(MODE2_1_GPIO_Port, MODE2_1_Pin);
							LL_GPIO_SetOutputPin(STCK_1_GPIO_Port, STCK_1_Pin);
							LL_GPIO_ResetOutputPin(DIR_1_GPIO_Port, DIR_1_Pin);

							currentpos->curr_state |= stepsizeX1_2;
							break;

		case stepsizeX1_4:	//X axis 1/4 step
							LL_GPIO_ResetOutputPin(MODE1_1_GPIO_Port, MODE1_1_Pin);
							LL_GPIO_SetOutputPin(MODE2_1_GPIO_Port, MODE2_1_Pin);
							LL_GPIO_ResetOutputPin(STCK_1_GPIO_Port, STCK_1_Pin);
							LL_GPIO_SetOutputPin(DIR_1_GPIO_Port, DIR_1_Pin);

							currentpos->curr_state |= stepsizeX1_4;
							break;

		case stepsizeX1_8:	//X axis full step
							LL_GPIO_SetOutputPin(MODE1_1_GPIO_Port, MODE1_1_Pin);
							LL_GPIO_ResetOutputPin(MODE2_1_GPIO_Port, MODE2_1_Pin);
							LL_GPIO_SetOutputPin(STCK_1_GPIO_Port, STCK_1_Pin);
							LL_GPIO_SetOutputPin(DIR_1_GPIO_Port, DIR_1_Pin);

							currentpos->curr_state |= stepsizeX1_8;
							break;

		default:	break;
	}



	switch(stepsizeY)
	{
		case stepsizeY1_0:	LL_GPIO_ResetOutputPin(MODE1_3_GPIO_Port, MODE1_3_Pin);
							LL_GPIO_ResetOutputPin(MODE2_3_GPIO_Port, MODE2_3_Pin);
							LL_GPIO_ResetOutputPin(STCK_3_GPIO_Port, STCK_3_Pin);
							LL_GPIO_ResetOutputPin(DIR_3_GPIO_Port, DIR_3_Pin);

							currentpos->curr_state |= stepsizeY1_0;
							break;

		case stepsizeY1_2:	LL_GPIO_SetOutputPin(MODE1_3_GPIO_Port, MODE1_3_Pin);
							LL_GPIO_ResetOutputPin(MODE2_3_GPIO_Port, MODE2_3_Pin);
							LL_GPIO_SetOutputPin(STCK_3_GPIO_Port, STCK_3_Pin);
							LL_GPIO_ResetOutputPin(DIR_3_GPIO_Port, DIR_3_Pin);

							currentpos->curr_state |= stepsizeY1_2;
							break;

		case stepsizeY1_4:	LL_GPIO_ResetOutputPin(MODE1_3_GPIO_Port, MODE1_3_Pin);
							LL_GPIO_SetOutputPin(MODE2_3_GPIO_Port, MODE2_3_Pin);
							LL_GPIO_ResetOutputPin(STCK_3_GPIO_Port, STCK_3_Pin);
							LL_GPIO_SetOutputPin(DIR_3_GPIO_Port, DIR_3_Pin);

							currentpos->curr_state |= stepsizeY1_4;
							break;

		case stepsizeY1_8:	LL_GPIO_SetOutputPin(MODE1_3_GPIO_Port, MODE1_3_Pin);
							LL_GPIO_ResetOutputPin(MODE2_3_GPIO_Port, MODE2_3_Pin);
							LL_GPIO_SetOutputPin(STCK_3_GPIO_Port, STCK_3_Pin);
							LL_GPIO_SetOutputPin(DIR_3_GPIO_Port, DIR_3_Pin);

							currentpos->curr_state |= stepsizeY1_8;
							break;

		default:	break;
	}

	switch(stepsizeZ)
	{
		case stepsizeZ1_0:	LL_GPIO_ResetOutputPin(MODE1_4_GPIO_Port, MODE1_4_Pin);
							LL_GPIO_ResetOutputPin(MODE2_4_GPIO_Port, MODE2_4_Pin);
							LL_GPIO_ResetOutputPin(STCK_4_GPIO_Port, STCK_4_Pin);
							LL_GPIO_ResetOutputPin(DIR_4_GPIO_Port, DIR_4_Pin);

							currentpos->curr_state |= stepsizeZ1_0;
							break;

		case stepsizeZ1_2:	LL_GPIO_SetOutputPin(MODE1_4_GPIO_Port, MODE1_4_Pin);
							LL_GPIO_ResetOutputPin(MODE2_4_GPIO_Port, MODE2_4_Pin);
							LL_GPIO_SetOutputPin(STCK_4_GPIO_Port, STCK_4_Pin);
							LL_GPIO_ResetOutputPin(DIR_4_GPIO_Port, DIR_4_Pin);

							currentpos->curr_state |= stepsizeZ1_2;
							break;

		case stepsizeZ1_4:	LL_GPIO_ResetOutputPin(MODE1_4_GPIO_Port, MODE1_4_Pin);
							LL_GPIO_SetOutputPin(MODE2_4_GPIO_Port, MODE2_4_Pin);
							LL_GPIO_ResetOutputPin(STCK_4_GPIO_Port, STCK_4_Pin);
							LL_GPIO_SetOutputPin(DIR_4_GPIO_Port, DIR_4_Pin);

							currentpos->curr_state |= stepsizeZ1_4;
							break;

		case stepsizeZ1_8:	LL_GPIO_SetOutputPin(MODE1_4_GPIO_Port, MODE1_4_Pin);
							LL_GPIO_ResetOutputPin(MODE2_4_GPIO_Port, MODE2_4_Pin);
							LL_GPIO_SetOutputPin(STCK_4_GPIO_Port, STCK_4_Pin);
							LL_GPIO_SetOutputPin(DIR_4_GPIO_Port, DIR_4_Pin);

							currentpos->curr_state |= stepsizeZ1_8;
							break;

		default:	break;
	}


	LL_mDelay(1);//10
	LL_GPIO_SetOutputPin(STBY_RESET_ALL_GPIO_Port, STBY_RESET_ALL_Pin);//get out of low consumption mode
	LL_mDelay(1);//10
	LL_GPIO_SetOutputPin(EN_FAULT_1_GPIO_Port, EN_FAULT_1_Pin);//EN drivers
	LL_GPIO_SetOutputPin(EN_FAULT_3_GPIO_Port, EN_FAULT_3_Pin);//
	LL_GPIO_SetOutputPin(EN_FAULT_4_GPIO_Port, EN_FAULT_4_Pin);//

	LL_GPIO_ResetOutputPin(STCK_1_GPIO_Port, STCK_1_Pin);
	LL_GPIO_ResetOutputPin(DIR_1_GPIO_Port, DIR_1_Pin);
	LL_GPIO_ResetOutputPin(STCK_3_GPIO_Port, STCK_3_Pin);
	LL_GPIO_ResetOutputPin(DIR_3_GPIO_Port, DIR_3_Pin);
	LL_GPIO_ResetOutputPin(STCK_4_GPIO_Port, STCK_4_Pin);
	LL_GPIO_ResetOutputPin(DIR_4_GPIO_Port, DIR_4_Pin);
}


