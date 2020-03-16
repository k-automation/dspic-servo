//---------------------------------------------------------------------
//  File:    pwm.c
//
//  Written By:  Lawrence Glaister
//
// Purpose: This set of routines to run pwn output
//      
// 
//---------------------------------------------------------------------
//
// Revision History
//
// Nov 21 2005 --    first version Lawrence Glaister
// Mar 18 2006 --    stripped to single output for servo card
// Sept 26 2006      put pid calcs inside pwm isr
// Nov 18 2006 --    added lpf on motor output
//---------------------------------------------------------------------- 
#include <xc.h>
#include "dspicservo.h"
#include <pwm.h>
#include <stdio.h>
#include <math.h>


extern struct PID pid;
extern struct COF cof;
extern void calc_pid( void );
extern volatile unsigned short int cmd_posn;      // current posn cmd from PC

//void set_pwm(float amps);
void set_pwm_error(float posn_error);

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _PWMInterrupt(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        handles pwm interrupts 
           we get a pwm intr every 4 pwm cycles (250us)
                   - setup by PTCON
                   - try and keep code < 250us or we need to deal with reentrancy

  Note:            None.
********************************************************************/
void __attribute__((__interrupt__,auto_psv)) _PWMInterrupt(void)
{
  static short gear = 0;
  static unsigned short new_cmd,last_cmd, new_fb,last_fb = 0;
  static short last_state = 0;  // last servo cycle enable/disable state

//  PWM_INTR = 1;    // use output pin to show how long we are in here
  IFS2bits.PWMIF =0;  // clr the interrrupt
  if (++gear >= pid.ticksperservo)
  {
    gear = 0;
    // time to do servo calcs
    if ( pid.enable && (last_state == 0) )
    {
      // we just got enabled.. try to prevent jumps
      // setup servo loop internals so our current posn is the target posn
      new_cmd = last_cmd = new_fb = last_fb = 0;
      cmd_posn = POSCNT;    // make 16bit incr registers match
      pid.command = 0L;    // make 32 bit counter match
      pid.feedback = 0L;
      pid.error_i = 0.0;    // reset internal error accumulators
      pid.error_d = 0.0;
      pid.error = 0.0;
      pid.cmd_d = 0.0;
      pid.prev_cmd = 0L;
    }
    // the servo calcs are run even if we are not enabled
    // this helps debugging because the s serial command can be used
    // to look at servo calc results without the motor going bezerk  
    new_cmd = cmd_posn;     // grab current cmd from pc
    new_fb = POSCNT;        // grab current posn from encoder
    pid.command  += (long int)((short)(new_cmd - last_cmd));
    pid.feedback += (long int)((short)(new_fb  - last_fb ));
      last_cmd = new_cmd;
    last_fb = new_fb;
    calc_pid();

    //set_pwm(pid.output);
	  set_pwm_error(pid.output);  
	// set_pwm(0.0);
    // update loop position analog output
    
    last_state = pid.enable;
  }
//  PWM_INTR = 0;
}
/*********************************************************************
  Function:        void setupPWM(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        port setup with deadtime, 
                   fault input disables pwm outputs on a per cycle basis

  Note:            None.
********************************************************************/
void setup_pwm(void)
{
    /* Holds the PWM interrupt configuration value*/
    unsigned int config;
    /* Configure pwm interrupt enable/disable and set interrupt priorties */
    config = (PWM_INT_EN & PWM_FLTA_DIS_INT & PWM_INT_PR1 & PWM_FLTA_INT_PR0);
    /* clear the Interrupt flags */
    IFS2bits.PWMIF = 0;  
    IFS2bits.FLTAIF = 0;  
    /* Set priority for the period match */
    IPC9bits.PWMIP      = (0x0007 & config);
    /* Set priority for the Fault A */
    IPC10bits.FLTAIP    = (0x0070 & config)>> 4;
    /* enable /disable of interrupt Period match */
    IEC2bits.PWMIE      = (0x0008 & config) >> 3;
    /* enable /disable of interrupt Fault A.*/
    IEC2bits.FLTAIE     = (0x0080 & config) >> 7;
    /* Configure PWM to generate 0 current*/
    PWMCON2bits.UDIS = 0;
    PDC1 = (FCY/FPWM - 1);
    PDC3 = (FCY/FPWM - 1);
    PTPER = (FCY/FPWM/2 - 1);      // set the pwm period register(/2 for cnt up/dwn)
    SEVTCMP = 0x00;
    /* 1L output is independant and enabled */
    /* 3L output is independant and enabled */
    PWMCON1 = (PWM_MOD1_IND & PWM_MOD2_IND & PWM_MOD3_IND &  /* independant i/o */
        PWM_PEN1L  & PWM_PDIS1H &   /* use 1L as pwm, TAM T3DEEL LLWADH3 (ON) */
        PWM_PDIS2L & PWM_PDIS2H &           /* rest as I/O */
        PWM_PEN3L  & PWM_PDIS3H     // TAM T3DEEL LLWADH3 (ON)
        );
    /* set dead time options, scale = 1, 10*FCY (about 625ns) */
    DTCON1 = PWM_DTAPS1 & PWM_DTA10;
    /* set up the fault mode override bits and mode */
    FLTACON = PWM_FLTA_MODE_CYCLE &
              PWM_FLTA1_DIS &
              PWM_FLTA2_DIS &
              PWM_FLTA3_DIS &
              PWM_OVA1L_INACTIVE &
              PWM_OVA1H_INACTIVE &
              PWM_OVA2L_INACTIVE &
              PWM_OVA2H_INACTIVE &
              PWM_OVA3L_INACTIVE &
              PWM_OVA3H_INACTIVE ;
    /* set special event post scaler, output override sync select and pwm update enable */
    PWMCON2 = (PWM_SEVOPS1 & PWM_OSYNC_PWM & PWM_UEN);
    // we get a pwm intr every 4 pwm cycles (250us)
    PTCON   = (PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE4 & PWM_IPCLK_SCALE1 & PWM_MOD_UPDN);
}
/*********************************************************************
  Function:        void set_pwm_error(float position_error)

  PreCondition:    None.
 
  Input:           In this appication it is used to output a scaled
                   position error. 50%=0 error, 0 and 100% are 
                   - or plus errors that are equal to the fault error distance.
  Output:          None.

  Side Effects:    None.

  Overview:        This routine accepts a value of position error in counts
           and scales it for display on a scope for servo tuning purposes

  Note:            None.
********************************************************************/
void set_pwm_error(float posn_error)
{
    const long pwm_max = (FCY/FPWM) - 1;      // 100% pwm count ( gives 5v output )
    long temp;
    long temp2;
    temp = (long)(((float)pwm_max * posn_error)/pid.maxerror);
    temp2= (long)(((float)pwm_max * posn_error)/pid.maxerror);
    temp2=fabs(temp2);

   if(temp2>1499){
       temp2=1499;
    }
//	if(cof.emergncy>0) //TESTTEST
//		{
//			PDC1 = 0;
//			PDC3 = 0;
//			temp2=0;
//			temp=0;
//				while(1)
//				{
//				}

//		}

/////////////////////////////
    if (temp>0){
    PDC3 = 0;
    asm("NOP");
    asm("NOP");
    asm("NOP");
    PDC1 = temp2;
    }
    if (temp<0){
    PDC1 = 0;
    asm("NOP");
    asm("NOP");
    asm("NOP");
    PDC3 = temp2;
    }

    temp=0;
    temp2=0;
}

/////