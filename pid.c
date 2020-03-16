/********************************************************************
* Description:  pid.c
*               This file, 'pid.c', is a single 
*               Proportional/Integeral/Derivative control loop used
*				for a single axis of position control. It was derived from
*				GPL2'd code by John Kasunich in the EMC2 project.
*				Copyright (C) 2003 John Kasunich
*               <jmkasunich AT users DOT sourceforge DOT net>
* Please do not bother John with support questions for this software.
* Significant changes have been made to support running on a small
* pic 30f2010 processor controlling a single axis servo driver.
* This derived work is also released under GPL 2 and is copy
*/

/*
    The three most important components are 'command', 'feedback', and
    'output'.  For a position loop, 'command' and 'feedback' are
    in position units.  For a linear axis, this could be inches,
    mm, metres, or whatever is relavent.  Likewise, for a angular
    axis, it could be degrees, radians, etc.  The units of the
    'output' pin represent the change needed to make the feedback
    match the command.  As such, for a position loop 'Output' is
    a velocity, in inches/sec, mm/sec, degrees/sec, etc.

    Each loop has several other pins as well.  'error' is equal to
    'command' minus 'feedback'.  'enable' is a bit that enables
    the loop.  If 'enable' is false, all integrators are reset,
    and the output is forced to zero.  If 'enable' is true, the
    loop operates normally.

    The PID gains, limits, and other 'tunable' features of the
    loop are implemented as parameters.  These are as follows:

    Pgain	Proportional gain
    Igain	Integral gain
    Dgain	Derivative gain
    bias	Constant offset on output
    FF0		Zeroth order Feedforward gain
    FF1		First order Feedforward gain
    deadband	Amount of error that will be ignored
    maxerror	Limit on error
    maxerrorI	Limit on error integrator
    maxerrorD	Limit on error differentiator
    maxcmdD	Limit on command differentiator
    maxoutput	Limit on output value

    All of the limits (max____) are implemented such that if the
    parameter value is zero, there is no limit.

    errorI	Integral of error
    errorD	Derivative of error
    commandD	Derivative of the command

    The PID loop calculations are as follows (see the code for
    all the nitty gritty details):

    error = command - feedback
    if ( abs(error) < deadband ) then error = 0
    limit error to +/- maxerror
    errorI += error * period
    limit errorI to +/- maxerrorI
    errorD = (error - previouserror) / period
    limit errorD to +/- paxerrorD
    commandD = (command - previouscommand) / period
    limit commandD to +/- maxcmdD
    output = bias + error * Pgain + errorI * Igain +
             errorD * Dgain + command * FF0 + commandD * FF1
    limit output to +/- maxoutput

*/


/** This program is free software; you can redistribute it and/or
    modify it under the terms of version 2.1 of the GNU General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111 USA

*/
#include <xc.h>
#include "dspicservo.h"
#include <math.h>

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* our servo loop data variable */
struct PID pid;
struct COF cof;
/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
void calc_pid(void);
void init_pid(void);

void init_pid(void)
{
    /* init all structure members */
    pid.enable = 1;//SVO_ENABLE;		// mirror state of PIN
    pid.command = 0.0;
    pid.feedback = 0.0;
	pid.maxposerror = 0.0;
    pid.error = 0.0;
    pid.output = 0.0;
    pid.deadband = 0.0;
    pid.maxerror = 1000.0;
    pid.maxerror_i = 0.0;
    pid.maxerror_d = 0.0;
    pid.maxcmd_d = 0.0;
    pid.error_i = 0.0;
    pid.prev_error = 0.0;
    pid.error_d = 0.0;
    pid.prev_cmd = 0.0;
    pid.limit_state = 0;
    pid.cmd_d = 0.0;
    pid.pgain = 0.005;
    pid.igain = 0.0;
    pid.dgain = 0.0;
    pid.ff0gain = 0.0;
    pid.ff1gain = 0.0;
    pid.maxoutput = 2000.0;		// emergency limit
	pid.multiplier = 1;
    pid.ticksperservo = 1;		// 500us/servo calc
//	unsigned char emergncy=0; //TESTTEST
}


/***********************************************************************
*                   REALTIME PID LOOP CALCULATIONS                     *
* 	this code is embedded inside a periodic ISR that defines the servo 
*	loop timing.
*
************************************************************************/
void calc_pid( void )
{
    float tmp1;
    float periodfp, periodrecip;
    long period = pid.ticksperservo * 250000;	/* thread period in ns */

    /* precalculate some timing constants */
    periodfp = period * 0.000000001;		// usually .001 sec
    periodrecip = 1.0 / periodfp;			// usually 1000.0

    /* calculate the error */
    tmp1 = (float)(pid.command - pid.feedback);
    pid.error = tmp1;

	// update a staus variable we used to check for max error during a move
	if ( fabs(pid.error) > fabs(pid.maxposerror) )
		pid.maxposerror = pid.error;


    pid.error_i += tmp1 * periodfp ;






     //calculate derivative term */
    pid.error_d = (tmp1 - pid.prev_error) * periodrecip;
    pid.prev_error = tmp1;
 

    // calculate derivative of command  ( used with ff1 tuning param ) */
    pid.cmd_d = (float)(pid.command - pid.prev_cmd) * periodrecip;
    pid.prev_cmd = pid.command;

 

	// calculate the output value */
	tmp1 = pid.pgain * tmp1 + pid.igain * pid.error_i +pid.dgain * pid.error_d;
	
	tmp1 += pid.command * pid.ff0gain + pid.cmd_d * pid.ff1gain;


	pid.output = tmp1;

if (fabs(pid.error) > pid.maxoutput)
	{
		cof.emergncy=1;  //TESTTEST
 	}

}
