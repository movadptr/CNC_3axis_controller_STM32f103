/*
 * stepper.h
 *
 *  Created on: Apr 24, 2024
 *  Author: Póti Szabolcs
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "stdint.h"
#include "main.h"


#define CMD_DONE_CHAR	(';')

#define FORWARD		1
#define BACKWARD	(-1)

#define XaxisLen	9600L
#define YaxisLen	14200L
#define ZaxisLen	1900L

//limit switch defines
#define sw11bitMSK	0x01//X axis home side switch
#define sw21bitMSK	0x02//X axis axle end switch
#define sw13bitMSK	0x04//Y axis home side switch
#define sw23bitMSK	0x08//Y axis axle end switch
#define sw14bitMSK	0x10//Z axis home side switch
#define sw24bitMSK	0x20//Z axis axle end switch

#define sw11bitPOS	0U
#define sw21bitPOS	1U
#define sw13bitPOS	2U
#define sw23bitPOS	3U
#define sw14bitPOS	4U
#define sw24bitPOS	5U

/* deprecated but left here for example
#define IfMovingAwayFromSwitch_X	(((dirx==BACKWARD)&&((switches&sw11bitMSK)!=sw11bitMSK)) || ((dirx==FORWARD)&&((switches&sw21bitMSK)!=sw21bitMSK)) )
#define IfMovingAwayFromSwitch_Y	(((diry==BACKWARD)&&((switches&sw13bitMSK)!=sw13bitMSK)) || ((diry==FORWARD)&&((switches&sw23bitMSK)!=sw23bitMSK)) )
#define IfMovingAwayFromSwitch_Z	(((dirz==BACKWARD)&&((switches&sw14bitMSK)!=sw14bitMSK)) || ((dirz==FORWARD)&&((switches&sw24bitMSK)!=sw24bitMSK)) )*/
#define IfMovingAwayFromSwitch(dir, sw1bitMSK, sw2bitMSK)	(((dir==BACKWARD)&&((switches&sw1bitMSK)!=sw1bitMSK)) || ((dir==FORWARD)&&((switches&sw2bitMSK)!=sw2bitMSK)) )

typedef struct
{
	uint32_t current_pos_x;
	uint32_t current_pos_y;
	uint32_t current_pos_z;
	int32_t current_toolpos;
	uint8_t movespeed;
	uint16_t curr_state;
	uint8_t error_state;
}CP;
//bit masks for CP.movespeed

#define	toolspeed_0	0x01
#define	toolspeed_1	0x02
#define	toolspeed_2	0x04
#define	toolspeed_3	0x08
#define	toolspeed_4	0x10
#define	toolspeed_5	0x20
#define	toolspeed_6	0x40
#define	toolspeed_7	0x80

//bit masks for CP.curr_state
#define AbsoluteStep_MSK	0x01
#define RelativeStep_MSK	0x02

#define RapidMove	0x04	//used for fast moves(pen up command PU)
#define	FeedMove	0x08	//used for cutting moves(pen down command PD)

#define stepsizeX1_0		0x0010//full step
#define stepsizeX1_2		0x0020//1/2 step
#define stepsizeX1_4		0x0040//1/4 step
#define stepsizeX1_8		0x0080//1/8 step

#define stepsizeY1_0		0x0100//full step
#define stepsizeY1_2		0x0200//1/2 step
#define stepsizeY1_4		0x0400//1/4 step
#define stepsizeY1_8		0x0800//1/8 step

#define stepsizeZ1_0		0x1000//full step
#define stepsizeZ1_2		0x2000//1/2 step
#define stepsizeZ1_4		0x4000//1/4 step
#define stepsizeZ1_8		0x8000//1/8 step

//bit masks for CP.error_state
#define XaxisLowerLimitReached_MSK	0x01
#define XaxisUpperLimitReached_MSK	0x02
#define YaxisLowerLimitReached_MSK	0x04
#define YaxisUpperLimitReached_MSK	0x08
#define ZaxisPositionError_MSK		0x10


typedef struct
{
	volatile int8_t dir;
	volatile uint8_t sw1MSK;
	volatile uint8_t sw2MSK;
	volatile uint32_t steps;
	volatile int32_t stepsTaken;
	GPIO_TypeDef* GPIOport;
	uint32_t GPIOpin;
}axle;

void get_limit_sw_state(void);
void STSPIN220_init(CP *currentpos, uint16_t stepsizeX, uint16_t stepsizeY, uint16_t stepsizeZ);
void STSPIN220_power_down(void);
void stepM(uint8_t M_axis);
void stepp(uint8_t M_axis, int8_t dir, uint32_t steps);
void rampup(uint8_t M_axis);
void gotozero(CP* currentpos);
void stepdelay(CP* currentpos);

void stepz(uint32_t z, int8_t dirz, CP* currentpos);

void stepxyz25D(uint32_t x, int8_t dirx, uint32_t y, int8_t diry, uint32_t z, int8_t dirz, CP* currentpos);
void stepxyz3D(uint32_t x, int8_t dirx, uint32_t y, int8_t diry, uint32_t z, int8_t dirz, CP* currentpos);
void HPGL_PR(int32_t xmove, int32_t ymove, int32_t zmove,CP* currentpos);
void HPGL_PA(uint32_t xcoord, uint32_t ycoord, uint32_t zcoord, CP* currentpos);

#endif /* INC_STEPPER_H_ */
