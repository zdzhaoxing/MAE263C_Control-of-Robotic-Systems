/**********************************************************************************************
 * Arduino PID Library - Version 0.4
 * by Zhaoxing Deng <denniszhaoxing@gmail.com>
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PIDx.h>

/* PIDx(...)*************************************************************
 * Create an PIDx object.
 ******************************************************************************/
PIDx::PIDx(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd)
{
    myOutput = Output;                    // set pointer to input, output, setpoint
    myInput = Input;
    mySetpoint = Setpoint;

    ITerm = 0;                            // set initial values. clear out ITerm. set output to 0 
	lastInput = *myInput;
    *myOutput = 0;

    PIDx::SetOutputLimits(0, 255);
    PIDx::SetInputChangeLimits(-30, 30);
	
    SampleTime = 1000;

    PIDx::SetTunings(Kp, Ki, Kd);         // set Kp, Ki, Kd unit to second

    lastTime = millis();                  // give an initial time to lastTime variable
	
	lastSetpoint = *mySetpoint;           // give the initial value to lastSetpoint variable
	 
    firstTime = true;                     // set the firstTime variable equal to true, since when construct a new object,  
	                                      // the next call of Compute() must be the first time 
}

/* Compute(...)*************************************************************
 * Compute am Output in each sample time. The output is computed by 
 * calculation = kp * error + ITerm - kd * dInput.
 * If the time change between two calls are less than sample time,
 * return false and not compute an output
 ******************************************************************************/

bool PIDx::Compute()
{
   unsigned long now = millis();                       // compute the time change from last call of Compute()
   unsigned long timeChange = (now - lastTime);
   
   if(firstTime){                                      // If it's the first call, set timeChange = 0, and let lastTime equal to now
     timeChange = 0;
     firstTime  = false;
     lastTime   = now;
   }
   
   if(*mySetpoint != lastSetpoint){                    // If Setpoint is changing, which means moving to the next point
	   ITerm = 0;                                      // set all the variables in computation equal to zero
	   *myOutput = 0;                                  // and return false
	   lastSetpoint = *mySetpoint;
	   return false;
   }
   else if(timeChange >= SampleTime)                       // If the timeChange >= SampleTime
   {

      double input = *myInput;  
      double dInput = (input - lastInput);	               // calculate the error
	  
	  if((dInput > inChaMax) || (dInput < inChaMin))       // if dInput is too large, elliminate this point.
	  {
		  input = lastInput;
		  dInput= 0;
	  }

      double error = *mySetpoint - input;
      double calculation;

      ITerm += (ki * error);                                // compute Ki term
      if(ITerm > outMax) ITerm = outMax;                     // if Ki term exceeds the limits
      else if (ITerm < outMin) ITerm = outMin;              // set it to the maximum or minimum
          
      calculation = kp * error + ITerm - kd * dInput; 

	  if(calculation > outMax) calculation = outMax;        // if calculation exceeds the limits
      else if(calculation < outMin) calculation = outMin;   // set it to maximum or minimum

	  *myOutput = calculation;                              // set output equal to calculation

      lastInput = input;                                    // refresh lastInput and lastTime

      lastTime  = now;
	  
	  return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * Change P I D parameters in PID controller
 ******************************************************************************/
void PIDx::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
   double SampleTimeInSec = ((double)SampleTime)/1000;      // set Kp, Ki, Kd unit to second
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

}

/* SetSampleTime(...)*************************************************************
 * Set new SampleTime. change the Ki, Kd accordingly.
 ******************************************************************************/
void PIDx::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)*************************************************************
 * set limits to the output.
 ******************************************************************************/
void PIDx::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

}

/* SetInputChangeLimits(...)*************************************************************
 * set limits to the input change. avoid disterbance in feedback signal.
 ******************************************************************************/
void PIDx::SetInputChangeLimits(double inMin, double inMax)
{
   if(inMin >= inMax) return;
   
   inChaMin = inMin;
   inChaMax = inMax;

}