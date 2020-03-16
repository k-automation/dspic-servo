//---------------------------------------------------------------------
//	File:		commands.c
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: This set of routines deals used to implement
//          the serial command struct for configuring
//          the internal servo parameters.
//---------------------------------------------------------------------
//
// Revision History
//
// Aug 11 2006 --    first version Lawrence Glaister
// Sept 22 2006      added deadband programming
// Sept 25 2006      added programmable servo loop interval
// 
//---------------------------------------------------------------------- 
#include <xc.h>
#include "dspicservo.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

extern volatile unsigned short int timer_test;

extern unsigned short int cmd_posn;			// current posn cmd from PC
extern unsigned short int cmd_err;			// number of bogus encoder positions detected
extern unsigned short int cmd_bits;			// a 4 bit number with old and new port values

extern struct PID pid;
//extern struct COF cof;
extern char rxbuff[];		// global rx buffer for serial data
extern char *rxbuffptr;		// local input ptr for storing data
extern short int rxrdy;		// flag to indicate a line of data is available in buffer

extern int save_setup( void );

float jerk;					// global used for loop tuning

void print_tuning(void)
{

        
    printf("\rCurrent Settings(cksum=0x%04X):\r\n",pid.cksum);
	printf("servo enabled = %d\r\n",	pid.enable);
	printf("(p) = %f\r\n",		(double)pid.pgain);
	printf("(i) = %f\r\n",		(double)pid.igain);
	printf("(d) = %f\r\n",		(double)pid.dgain);
	printf("FF(0) = %f\r\n",	(double)pid.ff0gain);
	printf("FF(1) = %f\r\n",	(double)pid.ff1gain);
    printf("dead(b)and = %f\r\n",(double)pid.deadband);
	printf("(m)ax Output = %famps\r\n",(double)pid.maxoutput);
	printf("(f)ault error = %f\r\n", (double)pid.maxerror);
	printf("(x)pc cmd multiplier = %hu\r\n", pid.multiplier);
    printf("(t)icks per servo cycle= %hu => %fms\r\n",
    pid.ticksperservo, pid.ticksperservo * 0.250);
}

void process_serial_buffer()
{
	int i;

	printf("processing serial buffer\r\n");
	for ( i=0; i < 15; i++ )
	{
		if ( rxbuff[i] )
			printf("%02X ",rxbuff[i] & 0xff);
		else
		{
			printf("\r\n");
			break;
		}
	}

	switch( rxbuff[0] )
	{
	case 'k':
		 pid.command = atof(&rxbuff[1]);
		 printf("\rencoder = 0x%04X = %d\r\n",POSCNT, POSCNT & 0xffff);
		break;

	case 'b':
		if (rxbuff[1])
		{
			pid.deadband = atof(&rxbuff[1]);
			save_setup();
		}
		print_tuning();
		break;		

	case 'p':
		if (rxbuff[1])
		{
			pid.pgain = atof(&rxbuff[1]);
			save_setup();
		}
		print_tuning();
		break;		
	case 'i':
		if (rxbuff[1])
		{
			pid.igain = atof(&rxbuff[1]);
			pid.error_i = 0.0;	//reset integrator
			save_setup();
		}
		print_tuning();
		break;		
	case 'd':
		if (rxbuff[1])
		{
			pid.dgain = atof(&rxbuff[1]);
			save_setup();
		}
		print_tuning();
		break;		
	case '0':
		if (rxbuff[1])
		{
			pid.ff0gain = atof(&rxbuff[1]);
			save_setup();
		}
		print_tuning();
		break;		
	case '1':
		if (rxbuff[1])
		{
			pid.ff1gain = atof(&rxbuff[1]);
			save_setup();
		}
		print_tuning();
		break;		
	case 'm':
		if (rxbuff[1])
		{
			pid.maxoutput = atof(&rxbuff[1]);
		   save_setup();
		}
		print_tuning();
		break;		

	case 'f':
		if (rxbuff[1])
		{
			pid.maxerror = atof(&rxbuff[1]);
			save_setup();
		}
		print_tuning();
		break;		

	case 'x':
		if (rxbuff[1])
		{
			pid.multiplier = (short)atof(&rxbuff[1]);
			if ( pid.multiplier < 1 ) pid.multiplier = 1;
			if ( pid.multiplier > 22 ) pid.multiplier = 22;
			save_setup();
		}
		print_tuning();
		break;		

    case 't':
		if (rxbuff[1])
		{
			pid.ticksperservo = (short)atof(&rxbuff[1]);
			if ( pid.ticksperservo < 1 ) pid.ticksperservo = 1;		// 250us/servo calc =>4000hz
			if ( pid.ticksperservo > 100 ) pid.ticksperservo = 100; // .025sec =>40hz rate
			save_setup();
		}
		print_tuning();
		break;		

	case 'j':
		if (rxbuff[1])
		{
			jerk = atof(&rxbuff[1]);	
		}
		break;

	case 'e':
		printf("\rencoder = 0x%04X = %d\r\n",POSCNT, POSCNT & 0xffff);
		break;	

	case 'l':
		print_tuning();
		break;

	case 's':
		printf("\rServo Loop Internal Calcs:\r\n");
		printf("command: %ld\r\n",pid.command);
		printf("feedback: %ld\r\n",pid.feedback);
		printf("error: %f\r\n",(double)pid.error);
		printf("max error: %f\r\n",(double)pid.maxposerror);
		pid.maxposerror = 0.0;
		printf("error_i: %famps\r\n",(double)(pid.error_i * pid.igain));
		printf("error_d: %famps\r\n",(double)(pid.error_d * pid.dgain));
		printf("output: %famps\r\n",(double)pid.output);
		printf("limit_state: %d\r\n",(int)pid.limit_state);
		break;
 
 case 'r':
          pid.command = 0.0;
    		pid.feedback = 0.0;
    		pid.error = 0.0;
//			cof.emergncy=0; // 'e' also can reset emergency
 break;
	
default:
		printf("\r\nUSAGE:\r\n");
		printf("p x.x set proportional gain\r\n");
        printf("i x.x set integral gain\r\n"); 
		printf("d x.x set differential gain\r\n"); 
        printf("0 x.x set FF0 gain\r\n"); 
		printf("1 x.x set FF1 gain\r\n"); 
		printf("b x.x set deadband\r\n");
		printf("m x.x set max output current(amps)\r\n"); 
		printf("f x.x set max error before drive faults(counts)\r\n");
		printf("x n   set pc command multiplier (1-16)\r\n");
        printf("t n   set # of 250us ticks/per servo calc(1-100)\r\n");
		printf("e print current encoder count\r\n"); 
		printf("l print current loop tuning values\r\n"); 
        printf("s print internal loop components\r\n");
        printf("j x.x alternately posn for loop tuning\r\n");
		printf("? print this help\r\n");
	
	}

	// reset input buffer state
	rxrdy = 0;
	rxbuff[0] = 0;
	rxbuffptr = &rxbuff[0];
	putchar('>');
}