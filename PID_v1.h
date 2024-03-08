#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.1.1

class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1

  //commonly used functions **************************************************************************
    PID(volatile double*, volatile double*, volatile double*,        // * constructor.  links the PID to the Input, Output, and 
        volatile double, volatile double, volatile double, volatile int, volatile int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(volatile double*, volatile double*, volatile double*,        // * constructor.  links the PID to the Input, Output, and 
        volatile double, volatile double, volatile double, volatile int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(double, double,       // * overload for specifying proportional mode
                    double, int);         	  

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
	void SetIntergralMemory(int);	  	  // * Sets the memory duration of the integral summation variable
										  //   A negative number means infinite memory duration
	void SetTolerance(int);				  // * Tolerance is set to kill the output if the error is less than 
										  //   the absolute tolerance
	
										  
  //Display functions ****************************************************************
	double GetKp();						  // These functions query the pid for interal values.
	double GetKi();						  //  they were created mainly for the pid front-end,
	double GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //
	
	void Initialize();
	
  private:
	
	
	volatile double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	volatile double dispKi;				//   format for display purposes
	volatile double dispKd;				//
    
	volatile double kp;                  // * (P)roportional Tuning Parameter
    volatile double ki;                  // * (I)ntegral Tuning Parameter
    volatile double kd;                  // * (D)erivative Tuning Parameter

	volatile int controllerDirection;
	volatile int pOn;

    volatile double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    volatile double *myOutput;             //   This creates a hard link between the variables and the 
    volatile double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	volatile double outputSum, lastInput;

	volatile unsigned long SampleTime;
	volatile double outMin, outMax;
	volatile bool inAuto, pOnE;
	volatile int Imem,ImemCounter;
	volatile int tolerance;
};
#endif

