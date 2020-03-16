/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

//---------------------------------------------------------------------
//	File:		dspicservo.h
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: Various header info for the project
//      
// 
//---------------------------------------------------------------------
//
// Revision History
// March 11 2006 --   formatted into multi file project
// Sept 25 2006 -  6x pwm rate
//---------------------------------------------------------------------- 
// define which chip we are using (peripherals change)
#include <xc.h>
// the following version string is printed on powerup (also in every object module)
#define CPWRT  "\rAlkhaldi Automation\r\n"
#define VERSION "1.0"

// this number was tweaked by hand until serial port baud was correct
#define FCY  (6000000 * 16 / 4)       // 24 MIPS ==> 6mhz osc * 16pll  / 4
//#define FCY  (14318000 * 8 / 4)       // 28.636 MIPS ==> 14.318mhz osc * 8pll  / 4

/* define required pwm rate... dont make it too high as we loose resolution */
#define FPWM 16000		// 48000 gives approx +- 10 bit current control

// define some i/o bits for the various modules
//#define STATUS_LED 	_LATE1		
//#define SVO_DIR     _LATE2
//#define SVO_NDIR    _LATE3	//inverted 

#define SVO_ENABLE   1
//#define PWM_INTR    _LATB1

// for some reason PI may not be defined in math.h on some systems
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define	TRUE	(1)
#define	FALSE	(0)	

struct PID{
	// the first block of params must survive powerfails and cksums
    // keep params together followed by cksum so that calc_cksum() works
    float pgain;		 /* param: proportional gain                 */
    float igain;		 /* param: integral gain                     */
    float dgain;		 /* param: derivative gain                   */
    float ff0gain;		 /* param: feedforward proportional          */
    float ff1gain;		 /* param: feedforward derivative            */
    float maxoutput;	 /* param: limit for PID output              */
    float deadband;		 /* param: deadband                          */
    float maxerror;		 /* param: limit for error                   */
    float maxerror_i;	 /* param: limit for integrated error        */
    float maxerror_d;	 /* param: limit for differentiated error    */
    float maxcmd_d;		 /* param: limit for differentiated cmd      */
	short multiplier;	 /* param: pc command multiplier             */
	short ticksperservo; /* param: number of 100us ticks/servo cycle */
    short cksum;		 /* data block cksum used to verify eeprom   */
	// the following block of temp vars is related to axis servo calcs
    // but should not be cksumed
    long int command;	/* commanded value */
    long int feedback;	/* feedback value */
    long int prev_cmd;	/* previous command for differentiator */
    float error;		/* command - feedback */
    float maxposerror;  /* status: maximum position error so far */
    float error_i;		/* opt. param: integrated error */
    float prev_error;	/* previous error for differentiator */
    float error_d;		/* opt. param: differentiated error */
    float cmd_d;		/* opt. param: differentiated command */
    float output;		/* the output value */
    short enable;		/* enable input */
    short limit_state;	/* 1 if in limit, else 0 */
//	unsigned char emergncy;
};


struct COF{
unsigned char emergncy; //TESTTEST
};
