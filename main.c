//---------------------------------------------------------------------
//	File:		main.c
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: This program is used to control an single axis servo card
//			using a pic 30f2010 and a OPA549 power op am driver stage.
// 
// The following files should be included in the MPLAB project:
//
//		main.c		    -- Main source code file
//		capture.c		-- interface to pc quadrature cmd inputs using IC1 and IC2
//      timer1.c        -- timer 1 setup for 100us intervals
//      serial.c        -- interface to pc serial port for tuning - 9600n81
//      encoder.c       -- interface to quadature encoder
//		pwm.c			-- pwn ch for motor current control
//		pid.c			-- actual code for pid loop
//		save-res.c		-- routines to read/write configuration
//		p30f4012.gld	-- Linker script file
//		DataEEPROM.s	-- assembler file for read/write eeprom
//---------------------------------------------------------------------
//
// Revision History
//
// July 4 2006 -- first version 
// Aug 13 2006 -- fixed wrap around problem in servo loop
//             -- when servo is reenabled, the target posn is set to the current posn
//             -- if drive faults, it now stays faulted until pwr cycle
//				  or disable/reenable
// Aug 15 2006 -- added pulse multiplier option for high count motor encoders
// Sept 23 2006-  servo loop from 1ms to 500us (2khz rate)
// Sept 26 2006   servo loop calcs now done in pwm isr
// Nov 16 2006    added some debugging to try and find out why config gets lost sometimes
//                eeprom cksum printed on saves and on the powerup restore.
//                brownlout logic and 64ms powerup delay added to cpuconfig word
//              -- on powerup, if no config is found, status led flashes fast until serial cmd rx'd
// Nov 21 2006  -- changed osc to use a 14.318mhz packaged osc instead of a 6mhz xtal
//              -- added j x.x command for servo tuning
//              -- added error output on pin 22/pwm3l/re4
//---------------------------------------------------------------------- 
#include <xc.h>
#include <stdio.h>
#include "dspicservo.h"
#include <math.h>

//--------------------------Device Configuration------------------------       
// needs a 30mhz part
_FOSC( XT_PLL16 );  // 6mhz * PLL16  / 4 = 24mips
//_FOSC( EC_PLL8 );  // 14.318mhz osc * PLL8  / 4 = 28.636mips
_FWDT(WDT_OFF);                   // wdt off
/* enablebrownout @4.2v, 64ms powerup delay, MCLR pin active */
/* pwm pins in use, both active low to give 64ms delay on powerup */
//_FBORPOR(PBOR_ON & BORV_42 & PWRT_64 & MCLR_EN & RST_PWMPIN & PWMxL_ACT_LO );

_FBORPOR(PBOR_ON & BORV45 & MCLR_DIS & PWRT_64 );

//----------------------------------------------------------------------

extern void setup_TMR1(void);
extern void setup_encoder(void);
extern void setup_uart(void);

extern volatile unsigned short int timer_test;
extern volatile unsigned short int cmd_posn;			// current posn cmd from PC
extern volatile short int rxrdy;
extern volatile float jerk;					// global used for loop tuning

extern void setup_pwm(void);
//extern void set_pwm(float amps);
extern void setup_adc10(void);
extern void setup_capture(void);
extern int restore_setup( void );
extern int calc_cksum(int sizew, int *adr);
extern void print_tuning( void );

extern void test_pc_interface( void );
extern void test_pwm_interface( void );
					
extern struct PID _EEDATA(32) pidEE;
extern struct PID pid;
extern struct COF cof;
extern void init_pid(void);
extern void	process_serial_buffer();



void __attribute__((__interrupt__,auto_psv)) _StackError (void)
{
	PDC1 = 0;	// cutoff output
        PDC3 = 0;	// cutoff output
	printf("STACK ERROR\r\n");	
while (1)
	{	
	}
}

void __attribute__((__interrupt__,auto_psv)) _AddressError (void)
{
	PDC1 = 0;	// cutoff output
	PDC3 = 0;	// cutoff output
	printf("ADDRESS ERROR\r\n");	
while (1)
	{
	}
}

void __attribute__((__interrupt__,auto_psv)) _MathError (void)
{
	PDC1 = 0;	// cutoff output
	PDC3 = 0;	// cutoff output
	printf("MATH ERROR\r\n");		
	while (1)
	{
	}
}
/*********************************************************************
  Function:        void set-io(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Sets up io ports for in and out

  Note:            Some periperal configs may change them again
********************************************************************/
void setup_io(void)
{
	// PORT B
    ADPCFG = 0xffff;        // make sure analog doesnt grab encoder pins (all digital)
    ADPCFGbits.PCFG0 = 0;   // AN0 is analog in - spare
							// 0=output, 1=input
	_TRISB0 = 1;			// used as AN0 above
	_TRISB1 = 0;			// used to indicate when PID calc is active
	_TRISB2 = 0;			// spare
	_TRISB3 = 0;			// spare
	_TRISB4 = 1;			// used by quad encoder input ch A
	_TRISB5 = 1;			// used by quad encoder input ch B

	// PORT C				// 0=output, 1=input
	_TRISC13 = 0;			// serial port aux  tx data
	_TRISC14 = 1;			// serial port aux  rx data
	_TRISC15 = 1;			// spare (maybe future xtal)

	// PORT D				// 0=output, 1=input
	_TRISD0 = 1;			// quad signal from pc
	_TRISD1 = 1;			// quad signal from pc

	// PORT E				// 0=output, 1=input
	_TRISE0 = 0;			// used by pwm1L to drive servo(set as i/p for startup)
	_TRISE1 = 0;			// used by on board fault led
	_TRISE2 = 0;			// used to set fwd/rev on motor power stage
	_TRISE3 = 0;			// inverted version of above
	_TRISE4 = 0;			// used by pwm3L to indicate position error  
	_TRISE5 = 0;			// spare 
	_TRISE8 = 1;			// goes low on amp overtemp/fault (also state of ext sw)

	// PORT F				// 0=output, 1=input
	_TRISF2 = 1;			// PGC used by icsp
	_TRISF3 = 1;			// PGD used by icsp
}

/*********************************************************************
  Function:        int main(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        main function of the application. Peripherals are 
                   initialized.

  Note:            None.
********************************************************************/
int main(void) 
{
	int cs;
	// vars used for detection of incremental motion
	jerk = 0.0;
	setup_io();             // make all i/o pins go the right dir
	setup_uart();		// setup the serial interface to the PC
        setup_TMR1();           // set up 1ms timer
	IEC0bits.T1IE = 1;      // Enable interrupts for timer 1
   	// needed for delays in following routines
	// 1/2 seconds startup delay 
	
 
    timer_test = 5000;		
	while ( timer_test );
    printf("\r\nPowerup..i/o...uart...timer...");
    
	init_pid();
	pid.enable = 0;		// turn servo loop off for a while
	setup_pwm();		// start analog output
	//set_pwm(0.0); 
    setup_encoder();    // 16 bit quadrature encoder module setup
    setup_capture();    // 2 pins with quadrature cmd from PC
	
	// some junk for the serial channel
	printf("%s%s\n\r",CPWRT,VERSION);

	// restore config from eeprom
	// Read array named "setupEE" from DataEEPROM and place 
	// the result into array in RAM named, "setup" 
	restore_setup();
	cs = -calc_cksum(((long int)&pid.cksum - (long int)&pid)/sizeof(int),(int*)&pid);
	if ( cs != pid.cksum )
	{
		// opps, no valid setup detected
		// assume we are starting from a new box
		printf(" EEPROM ERORR 0x%04X\r\n",pid.cksum);
		init_pid();
		while (1 )
		{
			// a very fast flash to indicate no config... serial activity
			// (hopefully the user setting params gets us out of the loop)
		 	timer_test = 1000; 
			while ( timer_test )
				{				}
			if ( rxrdy ) break;
		}
	}
	else
	{
		printf("Using setup from eeprom.. ? for help\r\n");
		print_tuning();
	}

    printf("using %fms servo loop interval\r\n",pid.ticksperservo * 0.250);
	while (1)
	{
		// check for serial cmds
		if ( rxrdy )
			process_serial_buffer();

		if ( jerk > 0.0 )
		{
			while ( 1 )
			{
				// loop forever until serial active or servo gets disabled
				if ( rxrdy || !SVO_ENABLE )
				{
					jerk = 0.0;
					break;
				}
				pid.command += jerk;
			    timer_test = 5000; while ( timer_test );
				pid.command -= jerk;
			    timer_test = 5000; while ( timer_test );
			}
		}
/*    
  // check for a drive fault ( posn error > allowed )
      if (( pid.maxerror > 0.0 ) && ( fabs(pid.error) > pid.maxerror ) && SVO_ENABLE )
      {
      pid.enable = 0;    // shut down servo updates
        while (1)          // trap here until svo disabled or pwr cycle
        {
//          set_pwm( 0.0 );
          printf("ÇÞÕì ÍÏ %f\r\n",(double)pid.error);
           timer_test = 2500*4; while ( timer_test );

        if (!SVO_ENABLE) 
          break;
      }
            while ( !SVO_ENABLE );  // wait for us to be enabled
    }

*/
		// check to see if external forces are causing use to change servo status
		if (SVO_ENABLE)
		{
			if ( pid.enable == 0 )	// last loop, servo was off
			{
				pid.enable = 1;
				printf("ããßä\r\n>");
				// give the servo loop some time to get established
				timer_test = 2500; while ( timer_test );
			}
		}
		else
		{
			if ( pid.enable == 1 )	// last loop servo was active
			{
				pid.enable = 0;
				printf("ÛíÑ ããßä\r\n>");
			}
		}
	}
	// to keep compiler happy....
	return 0;
}