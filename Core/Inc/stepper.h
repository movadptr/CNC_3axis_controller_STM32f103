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

//                  1/16 step  1/4 step
#define XaxisLen	142800L   //35700L	//356.3mm
#define YaxisLen	127200L   //31800L	//317.5mm
#define ZaxisLen	7600L     //1900L	//25mm //TODO change after Zaxis build, also consider if 200val is appropriate when stepout from limit sw proximit at gotohome()

#define XSTEPLENMM	0.002495f //0.00998f
#define YSTEPLENMM	0.002495f //0.00998f
#define ZSTEPLENMM	0.0032875f//0.01315f //TODO update after Zaxis build

#define LIMITSWSTEPOUT	800L  //200L  //how many steps to perform after init to step out of limit switch proximity

#define DEFAULTTOOLSPEED	10000L //um/s
//////////////////////////////////////////////////

#define CMD_DONE_CHAR	(';')

#define BUFFERED_CMD_TERMINATOR	('.')
#define SINGLE_CMD_TERMINATOR	(';')

#define FORWARD		1
#define BACKWARD	(-1)

//limit switch defines
#define SW_X_H	0x01//X axis home side switch
#define SW_X_E	0x02//X axis end side switch
#define SW_Y_H	0x04//Y axis home side switch
#define SW_Y_E	0x08//Y axis end side switch
#define SW_Z_H	0x10//Z axis home side switch
#define SW_Z_E	0x20//Z axis end side switch

#define SW_X_H_BITPOS	0U
#define SW_X_E_BITPOS	1U
#define SW_Y_H_BITPOS	2U
#define SW_Y_E_BITPOS	3U
#define SW_Z_H_BITPOS	4U
#define SW_Z_E_BITPOS	5U

/* deprecated but left here for example
#define IfMovingAwayFromSwitch_X	(((dirx==BACKWARD)&&((switches&sw11bitMSK)!=sw11bitMSK)) || ((dirx==FORWARD)&&((switches&sw21bitMSK)!=sw21bitMSK)) )
#define IfMovingAwayFromSwitch_Y	(((diry==BACKWARD)&&((switches&sw13bitMSK)!=sw13bitMSK)) || ((diry==FORWARD)&&((switches&sw23bitMSK)!=sw23bitMSK)) )
#define IfMovingAwayFromSwitch_Z	(((dirz==BACKWARD)&&((switches&sw14bitMSK)!=sw14bitMSK)) || ((dirz==FORWARD)&&((switches&sw24bitMSK)!=sw24bitMSK)) )*/
#define IfMovingAwayFromSwitch(dir, sw1bitMSK, sw2bitMSK)	(((dir==BACKWARD)&&((switches&sw1bitMSK)!=sw1bitMSK)) || ((dir==FORWARD)&&((switches&sw2bitMSK)!=sw2bitMSK)) )

typedef struct
{
	int32_t current_pos_x;
	int32_t current_pos_y;
	int32_t current_pos_z;
	int32_t origin_offset_x;
	int32_t origin_offset_y;
	int32_t origin_offset_z;
	int32_t toolspeed;// um/s	//if zero then a default val will be used
	uint32_t stp_delay_us;
	uint16_t curr_state;
	uint8_t error_state;
}CP;

//bit masks for CP.curr_state
#define AbsoluteStep_MSK	0x01
#define RelativeStep_MSK	0x02

#define RapidMove	0x04	//used for fast moves(pen up command PU)
#define	FeedMove	0x08	//used for cutting moves(pen down command PD)

#define stepsize_1_0		0x0010//full step
#define stepsize_1_2		0x0020//1/2 step
#define stepsize_1_4		0x0040//1/4 step
#define stepsize_1_8		0x0080//1/8 step
#define stepsize_1_16		0x0100//1/16 step

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
	volatile int32_t steps;
	volatile int32_t stepsTaken;
	GPIO_TypeDef* GPIOport;
	uint32_t GPIOpin;
}axle;

void get_limit_sw_state(void);
void A4988_init(CP *currentpos, uint16_t stepsize);
void A4988_power_down(void);
void stepM(uint8_t M_axis);
void stepp(uint8_t M_axis, int8_t dir, uint32_t steps);
void rampup(uint8_t M_axis);
void gotohome(CP* currentpos);
void StepDelay(CP* currentpos);

void stepz(uint32_t z, int8_t dirz, CP* currentpos);

void stepxyz25D(int32_t x, int8_t dirx, int32_t y, int8_t diry, int32_t z, int8_t dirz, CP* currentpos);
void stepxyz3D(int32_t x, int8_t dirx, int32_t y, int8_t diry, int32_t z, int8_t dirz, CP* currentpos);
void HPGL_PR(int32_t xmove, int32_t ymove, int32_t zmove,CP* currentpos);
void HPGL_PA(int32_t xcoord, int32_t ycoord, int32_t zcoord, CP* currentpos);

#endif /* INC_STEPPER_H_ */
