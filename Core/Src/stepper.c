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
	if(LL_GPIO_IsInputPinSet(SW_XH_GPIO_Port, SW_XH_Pin))	{ switches |= SW_X_H;}
	else{ switches &= ~SW_X_H;}

	if(LL_GPIO_IsInputPinSet(SW_XE_GPIO_Port, SW_XE_Pin))	{ switches |= SW_X_E;}
	else{ switches &= ~SW_X_E;}

	if(LL_GPIO_IsInputPinSet(SW_YH_GPIO_Port, SW_YH_Pin))	{ switches |= SW_Y_H;}
	else{ switches &= ~SW_Y_H;}

	if(LL_GPIO_IsInputPinSet(SW_YE_GPIO_Port, SW_YE_Pin))	{ switches |= SW_Y_E;}
	else{ switches &= ~SW_Y_E;}

	if(LL_GPIO_IsInputPinSet(SW_ZH_GPIO_Port, SW_ZH_Pin))	{ switches &= ~SW_Z_H;}
	else{ switches |= SW_Z_H;}

	if(LL_GPIO_IsInputPinSet(SW_ZE_GPIO_Port, SW_ZE_Pin))	{ switches &= ~SW_Z_E;}
	else{ switches |= SW_Z_E;}
}


//TODO don't use and delete stepz() and stepxyz25D()
void gotozero(CP* currentpos)
{
	uint32_t gx=0, gy=0;

	while( (switches&SW_Z_H) != SW_Z_H)
	{
		stepz(1, BACKWARD, currentpos);
	}
	currentpos->current_pos_z = 0;

	while( (switches&(SW_X_H|SW_Y_H)) != (SW_X_H|SW_Y_H) )
	{
		if((switches&SW_X_H)==SW_X_H)	{ gx = 0;}	else{ gx = 1;}
		if((switches&SW_Y_H)==SW_Y_H)	{ gy = 0;}	else{ gy = 1;}
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

void stepxyz3D(uint32_t x, int8_t dirx, uint32_t y, int8_t diry, uint32_t z, int8_t dirz, CP* currentpos)
{
	axle aX={dirx, SW_X_H, SW_X_E, x, 0, STEP_X_GPIO_Port, STEP_X_Pin};
	axle aY={diry, SW_Y_H, SW_Y_E, y, 0, STEP_Y_GPIO_Port, STEP_Y_Pin};
	axle aZ={dirz, SW_Z_H, SW_Z_E, z, 0, STEP_Z_GPIO_Port, STEP_Z_Pin};

	float slope10 = 0;
	float slope21 = 0;
	float virtual_slope10 = 0;
	float virtual_slope21 = 0;

	uint32_t swap=1;
	axle* tmp;
	axle* axt[3]={&aX, &aY, &aZ};

	float toolpath_len = 0;

	while(swap)//sorting, small to big
	{
		swap=0;
		if(axt[0]->steps > axt[1]->steps)	{tmp=axt[0]; axt[0]=axt[1]; axt[1]=tmp; swap=1;} else{}
		if(axt[1]->steps > axt[2]->steps)	{tmp=axt[1]; axt[1]=axt[2]; axt[2]=tmp; swap=1;} else{}
	}

	//only calculate this if toolspeed not zero, otherwise a default value will be used in StepDelay()
	if((currentpos->toolspeed != 0) && ((currentpos->curr_state&(FeedMove|RapidMove))==FeedMove))
	{
		//calculate the distance between current point and the destination
		toolpath_len = sqrt((x*XSTEPLENMM*x*XSTEPLENMM)+(y*YSTEPLENMM*y*YSTEPLENMM));
		toolpath_len = sqrt((toolpath_len*toolpath_len)+(z*ZSTEPLENMM*z*ZSTEPLENMM));
		//calculate the delay to achive the required tool speed
		//the outer axis will perform a step in every iteration so we use that stepsize for the calc
		currentpos->stp_delay_us = (uint32_t)(((toolpath_len/*mm*/ / currentpos->toolspeed/*mm/s*/) / axt[2]->steps) * 1000 * 1000/*us*/);
	}

	if(dirx==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_X_GPIO_Port, DIR_X_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_X_GPIO_Port, DIR_X_Pin);}
	if(diry==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_Y_GPIO_Port, DIR_Y_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_Y_GPIO_Port, DIR_Y_Pin);}
	if(dirz==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_Z_GPIO_Port, DIR_Z_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_Z_GPIO_Port, DIR_Z_Pin);}
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
		StepDelay(currentpos);
		LL_GPIO_ResetOutputPin(STEP_X_GPIO_Port, STEP_X_Pin);
		LL_GPIO_ResetOutputPin(STEP_Y_GPIO_Port, STEP_Y_Pin);
		LL_GPIO_ResetOutputPin(STEP_Z_GPIO_Port, STEP_Z_Pin);
		StepDelay(currentpos);
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

	if(dirx==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_X_GPIO_Port, DIR_X_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_X_GPIO_Port, DIR_X_Pin);}
	if(diry==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_Y_GPIO_Port, DIR_Y_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_Y_GPIO_Port, DIR_Y_Pin);}
	get_limit_sw_state();
	stepz(z, dirz, currentpos);

	if(x>y)
	{
		if(y==0)	{ slope=0;}
		else{ slope = fabs((float)x/y);}

		while(  (x_steps_taken<x) && IfMovingAwayFromSwitch(dirx, SW_X_H, SW_X_E) )//ha nem a bedőlt limit switch felé megy akkor csinálhat steppet
		{
			LL_GPIO_SetOutputPin(STEP_X_GPIO_Port, STEP_X_Pin);//motor steps on rising edge
			x_steps_taken++;

			if(slope != 0)
			{
				next_slope = ((float)x_steps_taken/(y_steps_taken+1));

				if( (next_slope>=slope ) && (y_steps_taken<y) && IfMovingAwayFromSwitch(diry, SW_Y_H, SW_Y_E))
				{
					LL_GPIO_SetOutputPin(STEP_Y_GPIO_Port, STEP_Y_Pin);//motor steps on rising edge
					y_steps_taken++;
				}
			}
			//StepDelay(currentpos);
			usDelay(800);
			LL_GPIO_ResetOutputPin(STEP_X_GPIO_Port, STEP_X_Pin);
			LL_GPIO_ResetOutputPin(STEP_Y_GPIO_Port, STEP_Y_Pin);
			//StepDelay(currentpos);
			usDelay(800);
		}
	}
	else
	{
		if(x==0)	{ slope=0;}
		else{ slope = fabs((float)y/x);}

		while( (y_steps_taken<y) && IfMovingAwayFromSwitch(diry, SW_Y_H, SW_Y_E) )//ha nem a bedőlt limit switch felé megy akkor csinálhat steppet
		{
			LL_GPIO_SetOutputPin(STEP_Y_GPIO_Port, STEP_Y_Pin);//motor steps on rising edge
			y_steps_taken++;

			if(slope != 0)
			{
				next_slope = ((float)y_steps_taken/(x_steps_taken+1));

				if( (next_slope>=slope ) && (x_steps_taken<x) && IfMovingAwayFromSwitch(dirx, SW_X_H, SW_X_E))
				{
					LL_GPIO_SetOutputPin(STEP_X_GPIO_Port, STEP_X_Pin);//motor steps on rising edge
					x_steps_taken++;
				}
			}
			//StepDelay(currentpos);
			usDelay(800);
			LL_GPIO_ResetOutputPin(STEP_X_GPIO_Port, STEP_X_Pin);
			LL_GPIO_ResetOutputPin(STEP_Y_GPIO_Port, STEP_Y_Pin);
			//StepDelay(currentpos);
			usDelay(800);
		}
	}

	currentpos->current_pos_x += (x_steps_taken*dirx);
	currentpos->current_pos_y += (y_steps_taken*diry);
}

void stepz(uint32_t z, int8_t dirz, CP* currentpos)
{
	uint32_t z_steps_taken = 0;

	if(dirz==FORWARD)	{ LL_GPIO_SetOutputPin(DIR_Z_GPIO_Port, DIR_Z_Pin);}	else{ LL_GPIO_ResetOutputPin(DIR_Z_GPIO_Port, DIR_Z_Pin);}
	get_limit_sw_state();
	usDelay(1);

	while(  (z_steps_taken<z) && IfMovingAwayFromSwitch(dirz, SW_Z_H, SW_Z_E) )//ha nem a bedőlt limit switch felé megy akkor csinálhat steppet
	{
		LL_GPIO_SetOutputPin(STEP_Z_GPIO_Port, STEP_Z_Pin);//motor steps on rising edge
		z_steps_taken++;
		//stepdelay(currentpos);
		usDelay(800);
		LL_GPIO_ResetOutputPin(STEP_Z_GPIO_Port, STEP_Z_Pin);
		//stepdelay(currentpos);
		usDelay(800);
	}

	currentpos->current_pos_z += (z_steps_taken*dirz);
}

void StepDelay(CP* currentpos)
{
	switch(currentpos->curr_state & (FeedMove|RapidMove))
	{
		case RapidMove:		usDelay(650);
							break;

		case FeedMove:		if(currentpos->toolspeed == 0)
							{
								usDelay(650);
							}
							else
							{
								usDelay(currentpos->stp_delay_us);
							}
							break;

		default:	usDelay(800);
					break;
	}
}

void A4988_power_down(void)
{
	LL_GPIO_ResetOutputPin(_SLEEP_GPIO_Port, _SLEEP_Pin);//enable internal circuitry
	LL_GPIO_ResetOutputPin(_RST_GPIO_Port, _RST_Pin);//set translator to home state and switch off output FETs
	LL_GPIO_SetOutputPin(_EN_GPIO_Port, _EN_Pin);//switch off output FETs
}

void A4988_init(CP *currentpos, uint16_t stepsizeX, uint16_t stepsizeY, uint16_t stepsizeZ)
{
	LL_GPIO_SetOutputPin(_SLEEP_GPIO_Port, _SLEEP_Pin);//enable internal circuitry
	LL_mDelay(2);
	LL_GPIO_ResetOutputPin(_RST_GPIO_Port, _RST_Pin);//set translator to home state and switch off output FETs
	LL_GPIO_SetOutputPin(_EN_GPIO_Port, _EN_Pin);//switch off output FETs

	currentpos->curr_state &= ~(stepsize_1_0 | stepsize_1_2 | stepsize_1_4 | stepsize_1_8 | stepsize_1_16);

	switch(stepsizeX)
	{
		case stepsize_1_0:	LL_GPIO_ResetOutputPin(MS1_GPIO_Port, MS1_Pin);
							LL_GPIO_ResetOutputPin(MS2_GPIO_Port, MS2_Pin);
							LL_GPIO_ResetOutputPin(MS3_GPIO_Port, MS3_Pin);

							currentpos->curr_state |= stepsize_1_0;
							break;

		case stepsize_1_2:	LL_GPIO_SetOutputPin(MS1_GPIO_Port, MS1_Pin);
							LL_GPIO_ResetOutputPin(MS2_GPIO_Port, MS2_Pin);
							LL_GPIO_ResetOutputPin(MS3_GPIO_Port, MS3_Pin);

							currentpos->curr_state |= stepsize_1_2;
							break;

		case stepsize_1_4:	LL_GPIO_ResetOutputPin(MS1_GPIO_Port, MS1_Pin);
							LL_GPIO_SetOutputPin(MS2_GPIO_Port, MS2_Pin);
							LL_GPIO_ResetOutputPin(MS3_GPIO_Port, MS3_Pin);

							currentpos->curr_state |= stepsize_1_4;
							break;

		case stepsize_1_8:	LL_GPIO_SetOutputPin(MS1_GPIO_Port, MS1_Pin);
							LL_GPIO_SetOutputPin(MS2_GPIO_Port, MS2_Pin);
							LL_GPIO_ResetOutputPin(MS3_GPIO_Port, MS3_Pin);

							currentpos->curr_state |= stepsize_1_8;
							break;

		case stepsize_1_16:	LL_GPIO_SetOutputPin(MS1_GPIO_Port, MS1_Pin);
							LL_GPIO_SetOutputPin(MS2_GPIO_Port, MS2_Pin);
							LL_GPIO_SetOutputPin(MS3_GPIO_Port, MS3_Pin);

							currentpos->curr_state |= stepsize_1_0;
							break;

		default:	break;
	}

	LL_mDelay(1);
	LL_GPIO_SetOutputPin(_RST_GPIO_Port, _RST_Pin);//enable output FETs
	LL_GPIO_ResetOutputPin(_EN_GPIO_Port, _EN_Pin);//enable output FETs

	//step one forward and one back, the stepsize select only applies after the next step rising edge
	LL_GPIO_SetOutputPin(DIR_X_GPIO_Port, DIR_X_Pin);
	LL_GPIO_ResetOutputPin(STEP_X_GPIO_Port, STEP_X_Pin);
	LL_GPIO_SetOutputPin(STEP_X_GPIO_Port, STEP_X_Pin);
	LL_GPIO_ResetOutputPin(STEP_X_GPIO_Port, STEP_X_Pin);
	LL_GPIO_ResetOutputPin(DIR_X_GPIO_Port, DIR_X_Pin);
	LL_GPIO_SetOutputPin(STEP_X_GPIO_Port, STEP_X_Pin);
	LL_GPIO_ResetOutputPin(STEP_X_GPIO_Port, STEP_X_Pin);

	LL_GPIO_SetOutputPin(DIR_Y_GPIO_Port, DIR_Y_Pin);
	LL_GPIO_ResetOutputPin(STEP_Y_GPIO_Port, STEP_Y_Pin);
	LL_GPIO_SetOutputPin(STEP_Y_GPIO_Port, STEP_Y_Pin);
	LL_GPIO_ResetOutputPin(STEP_Y_GPIO_Port, STEP_Y_Pin);
	LL_GPIO_ResetOutputPin(DIR_Y_GPIO_Port, DIR_Y_Pin);
	LL_GPIO_SetOutputPin(STEP_Y_GPIO_Port, STEP_Y_Pin);
	LL_GPIO_ResetOutputPin(STEP_Y_GPIO_Port, STEP_Y_Pin);

	LL_GPIO_SetOutputPin(DIR_Z_GPIO_Port, DIR_Z_Pin);
	LL_GPIO_ResetOutputPin(STEP_Z_GPIO_Port, STEP_Z_Pin);
	LL_GPIO_SetOutputPin(STEP_Z_GPIO_Port, STEP_Z_Pin);
	LL_GPIO_ResetOutputPin(STEP_Z_GPIO_Port, STEP_Z_Pin);
	LL_GPIO_ResetOutputPin(DIR_Z_GPIO_Port, DIR_Z_Pin);
	LL_GPIO_SetOutputPin(STEP_Z_GPIO_Port, STEP_Z_Pin);
	LL_GPIO_ResetOutputPin(STEP_Z_GPIO_Port, STEP_Z_Pin);
}


