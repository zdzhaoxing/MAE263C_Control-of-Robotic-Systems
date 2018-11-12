#ifndef PIDx_h
#define PIDx_h
#define LIBRARY_VERSION 0.4

class PIDx
{


  public:
  //commonly used functions **************************************************************************


    PIDx(double*, double*, double*,        // * constructor.  construct PIDx object, pass through kp, ki, kd
        double, double, double);         



    bool Compute();                       // * performs the PID calculation.  It should be
                                          //   called every time itertively in every time interval. 
										  //   The return value is true or false.
										  //   When return true, drive an output. Ohterwise, no output.
                                          //   SetSampleTime respectively

    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, user can define the upper bound and lower bound

    void SetInputChangeLimits(double);          // * clamps input change to a specific range. -30 - 30 by default. Users can define the range by themselves.
	
    void SetTunings(double, double,       // * Most users will set the tunings when construct an object, but can also change the ki, kp, kd 
                   double);               //   during the work process
   

    void SetSampleTime(int);              // * sets the SampleTime, in Milliseconds, default is 1000

    

  private:

	double kp;                  // * (P)roportional Parameter
    double ki;                  // * (I)ntegral Parameter
    double kd;                  // * (D)erivative Parameter


    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   can easily reach the value of Input, Output, Setpoint 
    double *mySetpoint;           
                                  

	unsigned long lastTime;                      // * to record the lastTime when Compute() is called
	double ITerm, lastInput, lastSetpoint;       // * ITerm to record the acculated Ki terms. 
	                                             // * lastInput and lastSetpoint to record the input and setpoint last time when Compute() is called

	unsigned long SampleTime;                    // * define the SampleTime, minimum output and maximum output
	double outMin, outMax;
	double inChaMin, inChanMax;                  // * define the input change upper bounds and lower bounds
    bool firstTime;                              // * show whether the Compute() is called at the first time
};
#endif
