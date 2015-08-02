#include <Makeblock.h>
#include <EEPROM.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>

/*
	Version		1.0.1
	HISTORY
		Date			Author		Changes
		----			------		-------
		2015/June/8   - Robbo1		Fixed error with parameters being corrupted during the decoding of the cmd line  (Scara seemed to have a mind of its own)
		2015/June/8   - Robbo1		Fixed potential buffer overflow problem if newline character was ever corrupted/lost
		2015/June/8   - Robbo1		Fixed positional accuracy error.  Caused by using a test that allowed for finishing moving before all stepping complete
	
	Version		2.0.0
			Overview of changes
				- Major rewrite of the calculations of the angles and steps.  
				  Description of method used to do the calculations
						- The 2 arms can be thought to be scribing 2 circles centred at 0,0 and X,Y which
						  intersect at 2 points.  
						- 4 triangles can be created, 2 on one side of the diagonal (0,0 to X,Y) and 2 
						  mirrored on the other side of the triangle.  
									  
									  Arm1 end point
							              .
							             /| .
							            / |    .
							           /th| thb1  . 
							          / b2|          .
							         /    |             .
							     L2 /     |                . L1
							       /      | b                 .
							      /       |                      .
							     /        |                         .
							    /  tha2   |                      tha1  .
							X,Y/__________|_______________________________. 0,0
							   \    a2    |              a1            .              diagonal = a1 + a2
							    \         |                         .
								  
						  0,0 to X,Y diagaonal shown horizontal for explaination.  Its angle to the x axis is 
						  determined by atan(Y/X) and adjusted depending on which quadrant X,Y is in.
						
						- Solving the triangles provides the angles tha1 & tha2 
						  a1^2 + b^2 = L1^2   and   a2^2 + b^2 = L2^2      .... (I)
						  
						  and substracting them gives
						  a1^2 - a2^2 = L1^2 - L2^2                        .... (II)
						  
						  Taking d = a1 + a2                               ....  (III)
						  which can be rewritten as  a2 = d - a1 and combined with (II)
						  
							   a1^2 - (d - a1)^2  = L1^2 - L2^2
						  
						  ==>  a1^2 - d^2 + 2*d*a1 - a1^2 = L1^2 - L2^2
						  ==>  2*d*a1 - d^2 = L1^2 - L2^2
						  ==>  a1  =  ((L1^2 - L2^2) + d^2) / 2*d          .... (IV)
						  
						  Then a2 can be found by solving (III) using a1 found in (IV)
						  
						- Angles tha1 & tha2 can be found using acos function
						
						  tha1 = acos(a1/L1)    &   tha2 = acos(a2/L2)     .... (V)
						  
						  tha1 is the angle for Arm 1 from the diagonal
						  
						- thb1 & thb2 can be found by fact that angles in a triangle add up to PI
						  and the fact the triangles are right angle triangles
						
						  thb1 = PI/2 - tha1  &  thb2 = PI/2 - tha2

						  thb  = thb1 + thb2 = PI - tha1 - tha2           .... (VI)
						  
						- angle for arm 2 in relation to arm1 is the sum of thb1 & thb2
						
						- then to express the angles as polar coordinates orientated as normal the
						  angles will have to be adjusted.  Arm 1 angle is adjusted by the angle of the diagonal
						  and arm 2 is adjusted so that zero rads is normal (most CW related to Arm1)
						  So angle for arm 1 is angle of the diagonal +/- tha1
						  
							 angle arm 1 = atan(Y/X) + adj for quadrant +/- tha1

						  And angle for arm 2 will be adjusted for orientation CW on Arm1
						  ie            angle arm 2   = 2*PI - thb           OR   = thb
								 & can be written as  = PI + tha1 + tha2     OR   = PI - tha1 - tha2

					  
				- Major rewrite of movement to accommodate the best use of rewritten angle calculations
				- Changed arm 1  0 Rad angle from the negative direction to the positive.  This is to standardise 
				  polar coordinates with normal orinatation.  This fixing of the orientation does not change anything
				  other than angle value.
				- Introduce angle limits for arm 1 & 2
					The limits are defined in terms of limit going CW and limit going CCW
					
						                  |
						                  |
						                  |
						                  |
						                  |
							--------------+--------------
						                 /|\
						                / | \
						               /  |  \
						              /   |   \
						             /    |    \
						            /     |     \
								 Limit <------> Limit
							      CCW   out of   CW
							            bounds
										  
				- Introduce a "black listed" area for supplied X,Y point.  This area is to protect the structure from the arms.
					This prevents the pen/laser from attempting to enter an area that includes the structure.  For the standard mScara 
					it will include the structure and most of 3rd and 4th quadrants.  Obviously it cannot include the home position.
					The box specified does not prevent the arms moving through the box, just the pen/laser cannot be told to go to
					a point in the area.
					
					Typically for a stock standard setup the "black listed" area would be around X_left=-75, X_right=150, Y_top=50, Y_bottom=-120
					And obviously the drawing should not be placed in 3rd or 4th quadrant if no board exists.
					
					    -75,50                                         150,50
					       ................................................
					       .                                              .
					       .           +---------------------+            .
					       .           |                     |            .
					       .        +--+  structure          +--+         .
					       .        |                           |         .
					       .        +----------------+--------+-+         .
					       .                         |        |           .
					       .                         |        |           .
					       .                         +--------+           .
					       .                                              .
					       .                                              .
					       .                                              .
					       ................................................
					    -75,-120                                       -75,150
					
					
				- Add software stepping rate to give the user the option to reduce resolution to increase accuracy of the lines.
						- With stepper motors only full step and 1/2 stepping is electro-mechanically stable (as long as torque on shaft is less than holding torque)
						- And 1/4 step is generally accepted to be stable but with slightly less torque figures.
						- 1/8 and more so 1/16 stepping cannot be positioned accurately and exactly what position achieved depends on a 
						  number of factors such as torque applied to the shaft, how far and what speed the motor was moving.
						- As such using resolution of 1/16th causes a noticable drawing error of upto around +/- 1/2 mm which is 
						  very noticable.  By being able to reduce, by software, the resolution to say 1/4 stepping the drawn lines will
						  line up correctly (accuracy) but the accuracy is reduced by a quarter.
						- The allowed software stepping rates are 1/16, 1/8, 1/4, 1/2, FULL_STEP 
						  And the mDraw program will have values   16    8    4    2      1      to specify these stepping rates
						  The software implements this as number of 1/16th steps (no changes to H/W stepping on the controllers)
						  So 1/16 has one 1 step, 1/8 has 2 steps, 1/4 has 4 steps, 1/2 has 8 steps and FULL_STEP is 16 steps    
						       where steps are the H/W steps (1/16th stepping)
						- NOTE:  There are still the same number of actual steps per revolution, the S/W stepping rate just does multiple steps
						         to achieve the different stepping rate.
						  
				- Use variable names with more meaning
				- This requires a major rewrite.
			Expected Benefits
				- Greater efficiency when calculating the angles, even though the code is greater
				- Greater accuracy in calculations.  Float only has 6-7 decimal digits of precision.  The previous
				  calculations had factors to the 4th power losing precision, whereas the new calculations 
				  have powers of 2 only so that precision is higher.
				- Ability to use the 2nd intercept point of the 2 circles scribed by the 2 arms, which greatly increases
				  the area available for drawing.
				- angle limits and excluded area will eliminate the arms impacting the frameing.
				- Code will be more suitable for modification.
	HISTORY
		Date			Author		Changes
		----			------		-------
		2015/June/24  - Robbo1		Implementing Version 2.0.0 rewrite


	
*/

/*
	Acceleration (deceleration) table stored in program memory
	Also the number of entries is stored.
	The values are number of uSecs to delay after taking a step.
	The average time taken for a loop has to be accounted for (substract from period so
	that the delay time + loop time is the required period.
	
	Acceleration table for 400 full steps / second squared
	
	Usually only a portion of the table is used because 200uSec per 1/16th step can be too fast for the weight of the laser.
	
*/
const unsigned int motorAccel[] PROGMEM = {25000, 6250, 2589, 1987, 1675, 1476, 1334, 1227, 1142, 1073, 1015, 965, 922, 885, 851, 821, 794, 770, 748, 727, 708,
												 691,  675,  659,  645,  632,  619,  608,  596,  586,  576, 566, 557, 549, 540, 533, 525, 518, 511, 504, 498,
												 492,  486,  480,  474,  469,  464,  459,  454,  449,  445, 440, 436, 432, 428, 424, 420, 416, 413, 409, 406,
												 402,  399,  396,  393,  390,  387,  384,  381,  378,  375, 373, 370, 368, 365, 363, 360, 358, 355, 353, 351,
												 349,  347,  345,  342,  340,  338,  337,  335,  333,  331, 329, 327, 325, 324, 322, 320, 319, 317, 315, 314,
												 312,  311,  309,  308,  306,  305,  303,  302,  301,  299, 298, 296, 295, 294, 293, 291, 290, 289, 288, 286,
												 285,  284,  283,  282,  281,  279,  278,  277,  276,  275, 274, 273, 272, 271, 270, 269, 268, 267, 266, 265,
												 264,  263,  262,  261,  260,  260,  259,  258,  257,  256, 255, 254, 254, 253, 252, 251, 250, 250, 249, 248,
												 247,  246,  246,  245,  244,  243,  243,  242,  241,  241, 240, 239, 238, 238, 237, 236, 236, 235, 234, 234,
												 233,  232,  232,  231,  231,  230,  229,  229,  228,  228, 227, 226, 226, 225, 225, 224, 223, 223, 222, 222,
												 221,  221,  220,  220,  219,  218,  218,  217,  217,  216, 216, 215, 215, 214, 214, 213, 213, 212, 212, 211,
												 211,  210,  210,  210,  209,  209,  208,  208,  207,  207, 206, 206, 205, 205, 205, 204, 204, 203, 203, 202,
												 202,  202,  201,  201,  200,  200,  200 };
//#define  motorAccelEntries        247			// Number of entries in acceleration table (not counting element zero which is settling time)
#define  motorAccelEntries         40				// Number of entries in acceleration table (not counting element zero which is settling time)

// data stored in eeprom
// Robbo1 - 2015/06/24  -  Add allocation for additional values to be stored in EEPROM 
union{
    struct{
      char name[8];
      unsigned char motorADir;
      unsigned char motorBDir;
      int           arm0len;
      int           arm1len;
      int           speed;
	  float         arm1LimitCCW;
	  float         arm1LimitCW;
	  float         arm2LimitCCW;
	  float         arm2LimitCW;
	  int           excludeBoxXRight;
	  int           excludeBoxXLeft;
	  int           excludeBoxYTop;
	  int           excludeBoxYBottom;
	  int           swStepRate;
	  int           servoPenUp;
	  int           servoPenDown;
	  int	        stepsPerRevolution;
    }data;
    char buf[64];
}roboSetup;

//	Robbo1 - 2015/06/24    -   Add defines and structures
//	defines and structures for parameters for positioning at a point
#define  CW                                  0
#define  CCW                                 1
#define  numberAngleType                     2
#define  impossibleMoveDirection             3

typedef struct {
	float			thArm1,  thArm2;				// angles for the arms 1/2
	unsigned char   validAngles;					// angle for arms valid? flag
	unsigned int	stepsA,  stepsB;				// step position for motoras A/B
} positionParameters;

typedef struct {
	float				X, Y;
	positionParameters	angles[numberAngleType];
	int					angleType;
	int					mode;						
	unsigned long		pointDelay;					// time to delay at a point
} scaraMoveControl;


//	Defaults and Hard Values

#define  armALengthDefault                168
#define  armBLengthDefault                206
#define  stepsPerRevolutionDefault      16000
#define  arm1LimitCCWDefault                3.3f
#define  arm1LimitCWDefault                 0.6f
#define  arm2LimitCCWDefault                5.9f
#define  arm2LimitCWDefault                 0.4f
#define  excludeBoxXRightDefault          150 
#define  excludeBoxXLeftDefault           -90
#define  excludeBoxYTopDefault             60
#define  excludeBoxYBottomDefault        -125
#define  servoPositionPenUpDefault        130
#define  servoPositionPenDownDefault       90
#define  softSteppingDefault               16						// Software implementation of motor stepping.  Values can be 1, 2, 4, 8, 16 giving equivalent motor stepping  16==1/16, 8==1/8, 4==1/4, 2==1/2, 1==full_step
//	softStepping provides a software way of controlling the motor resolution.  Greater resolution reduces accuracy.   1/16 step has less accuracy than 1/2 step.  Experiementation
//	shows that 1/4 stepping provides good repeative accuracy.  Repeative accuracy is how accurate can a point be reach when trying to return to the point.
//	Stepper motor mechanics means that 1/8 and 1/16 step resolution is only "best effort" and often only moves a indeterminate amount causing slight errors in positioning


//	Point mode     (not fully implemented yet - do not use - risk of injury to bot and one self)
//	This feature is to allow other devices to be attached to arm and perform its function at each point rather than drawing with device
//	For instance a drill could be attached to drill holes at specified points and the point delay used to specify how long attached device is on
//	uses additional port to pulse the device and speed control is the motor port
//	
#define  drawMode                           0						// normal draw mode
#define  auxMode                            1						// Aux mode   (allows drill to be attached to drill holes, or other device

//	Return Movement indicatiors and Error codes 
#define  operationSuccess                   0

#define  errorPointTooCloseToOrigin         1
#define  errorPointTooFarForArmsToReach     2
#define  errorArmAnglesOutOfBounds          3
#define  errorImpossibleMove                4


//	Set up ports
MePort stpB(PORT_2);
MePort stpA(PORT_1);
MeDCMotor laser(M2);
MeDCMotor auxDevice(M2);
MePort servoPort(PORT_7);
int servopin =  servoPort.pin2();
Servo servoPen;


//	Robbo1 - 2015/06/24    -   Add new global variable area
//	Global Variables to use
scaraMoveControl   	newPoint;						// Positional parameters for the new point
scaraMoveControl    currPoint;						// Positional parameters for the current point 
float				L1, L2;         	            // arm1Len & arm2Len
float	 			L1SQRminusL2SQR;				// Holds the value for ArmL1^2 - armL2^2 which reamins constant for a given arm 1&2 lengths
float               L1SQR, L2SQR;					// arm1 squared   arm2 squared
float               L1L2Sum, L1L2Diff;				// addition of arm length and absolute of L1 - L2
unsigned char       arm1ZeroAngleOutOfLimit;		// flag to indicate zero rads is included in motor A angle limits
unsigned char       arm2ZeroAngleOutOfLimit;		// flag to indicate zero rads is included in motor B angle limits
int					motorAStepsLimitCW;				// motor A step limit CW
int					motorAStepsLimitCCW;			// motor A step limit CCW
int					motorBStepsLimitCW;				// motor B step limit CW
int					motorBStepsLimitCCW;			// motor B step limit CCW
int					stepsPerRevolution;				// steps per revolutions ( motor steps per rev * (big_gear/little_gear) )
float				stepsPerRevolutionFP;			// If memory space allows having the floating point as a var is quicker than type casting & and saves stack space
int					motorSteppingRate;				// 1=FULL_RATE, 2=1/2 step, 4=1/4, 8=1/8, 16=1/16 stepping  All other values are invalid and the default will be set
int					stepResolution;					// how many 1/16 steps to limit resolution to.    1=1/16 stepping, 2=2/16=1/8 stepping, 4=4/16=1/4 stepping, 8=8/16=1/2 stepping, 16=16/16 stepping 
int					servoPositionCurrent;			// Used to keep track of the current pen position
int                 servoPositionLastCommand;		// Used to remember what the last command set it to
int                 laserPowerCurrent;              // Current Laser Power Setting            (Current system where PWM controls Laser --  Very bad for the laser since the PWM is switching a switching power reg that exists before the laser)
int                 laserPowerLastCommand;          // Current Laser Power Last Command

//*********************************  DEBUG VARS TO BE REMOVED
int		cmdNo=0;	// number of move commands to lockin other debug output

//	Angle from ATAN has to be adjusted for the quadrant that the point is in.
#define  radAdjaTan(x,y)                	(((x) >= 0) ? (((y) >= 0) ? 0 : TWO_PI) : PI)

//	Constrain angle to 0-2PI
#define	 modulo2PI(thx)						(((thx)<0)?(thx)+TWO_PI:(((thx)>=TWO_PI)?(thx)-TWO_PI:(thx)))

//  convert angle to steps
//#define  thSteps(th)						((th)*(stepsPerRevolutionFP)/TWO_PI)
#define  thSteps(th)						(int)((((th)*(stepsPerRevolutionFP)/TWO_PI)/stepResolution)*stepResolution)

//	Variables used Globally




/*

	Routines to determine the step position to use for both motors
	
	- Orientation: Normal: 0 Rads is along the +ve X axis.
	  The +ve X axis is horizontal on the right side
	  
	   +----->  0 rads
	  0,0
	  
	- To prevent the arms from colliding there are two types of limits to prevent the
	  arms from moving into "forbidden areas.
	  1)	Angle limits to prevent the arms moving into areas that hit any of the structure.
	  2)	Exclusion Box to prevent any attempt to draw/engrave in the area bounded by
            the framing and non-existant space (quadrants 3 & 4 in typical setup)	  
	
	- The 2 sets of possible angles can be considered the internal angles of 2 sets of 
	  of triangles mirrored either side of the line (diagonal) joining 0,0 and X,Y
	  See desciption above for an explaination of the maths involved and how the 
	  internal angles are adjusted to orientate the angle to normal orientation.
	
*/

/*

	Function:	prepareMoveCalcs(void)
	
	Purpose:	Prepare global variables that precalculate constant values.
	
	Parameters:	none
	
	Returns:	Values set in Global variables
				L1SQR				- L1^2
				L2SQR               - L2^2
				L1SQRminusL2SQR		- L1^2 - L2^2
				
	NOTES:		Only called on startup and setting of system parameters.
				
*/
void prepareMoveCalcs(void){

    L1		 		= (float)roboSetup.data.arm0len;	
    L2 				= (float)roboSetup.data.arm1len;	
	L1SQR           = (float)(L1 * L1);
	L2SQR           = (float)(L2 * L2);
	L1SQRminusL2SQR = L1SQR - L2SQR;
	L1L2Sum         = L1 + L2;
	L1L2Diff		= ((L1 >= L2) ? L1 - L2 : L2 - L1) ;		// quicker than abs() which cannot have maths in it - see arduino reference
	stepsPerRevolution      = roboSetup.data.stepsPerRevolution;
	stepsPerRevolutionFP    = stepsPerRevolution;
	arm1ZeroAngleOutOfLimit = ( roboSetup.data.arm1LimitCW < roboSetup.data.arm1LimitCCW );		// set flag to know which way to test against limits
	arm2ZeroAngleOutOfLimit = ( roboSetup.data.arm2LimitCW < roboSetup.data.arm2LimitCCW );		// set flag to know which way to test against limits
	motorAStepsLimitCW      = (int)(thSteps(roboSetup.data.arm1LimitCW));						// limits as motor steps
	motorAStepsLimitCCW     = (int)(thSteps(roboSetup.data.arm1LimitCCW));						// limits as motor steps
	motorBStepsLimitCW      = (int)(thSteps(roboSetup.data.arm2LimitCW));						// limits as motor steps
	motorBStepsLimitCCW     = (int)(thSteps(roboSetup.data.arm2LimitCCW));						// limits as motor steps
	if ( (roboSetup.data.swStepRate == 16) || (roboSetup.data.swStepRate ==  8) || (roboSetup.data.swStepRate ==  4) || 
	     (roboSetup.data.swStepRate ==  2) || (roboSetup.data.swStepRate ==  1) ) { 
		motorSteppingRate = roboSetup.data.swStepRate;
	} else {
		motorSteppingRate = softSteppingDefault;
	}
	stepResolution    = 16 / motorSteppingRate;
	
	

	Serial.println(F(" "));
	Serial.print(F("SETUP:      L1: "));Serial.print(L1);Serial.print(F("\tL2: "));Serial.print(L2);Serial.print(F("\tL1^2: "));Serial.print(L1SQR);Serial.print(F("\tL2^2: "));Serial.print(L2SQR);
	Serial.print(F("\tL1L2Diff: "));Serial.print(L1L2Diff);Serial.print(F("\tL1L2Sum: "));Serial.println(L1L2Sum);
	
	Serial.print(F("SETUP:   mALCW: "));Serial.print(motorAStepsLimitCW);Serial.print(F("\tmALCCW: "));Serial.print(motorAStepsLimitCCW);
	Serial.print(F("\tmBLCW: "));Serial.print(motorBStepsLimitCW);Serial.print(F("\tmBLCCW: "));Serial.println(motorBStepsLimitCCW);
	
	Serial.print(F("SETUP:   L10OB: "));Serial.print(arm1ZeroAngleOutOfLimit);Serial.print(F("\tL20OB: "));Serial.print(arm2ZeroAngleOutOfLimit);Serial.print(F(" SALCW: "));Serial.print(motorAStepsLimitCW);
	Serial.print(F(" SALCCW: "));Serial.print(motorAStepsLimitCCW);Serial.print(F(" SBLCW: "));Serial.print(motorBStepsLimitCW);Serial.print(F(" SBLCCW: "));Serial.println(motorBStepsLimitCCW);
	
	Serial.print(F("SETUP:   roboSetup.data.arm1LimitCW:       "));Serial.println(roboSetup.data.arm1LimitCW);
	Serial.print(F("SETUP:   roboSetup.data.arm1LimitCCW:      "));Serial.println(roboSetup.data.arm1LimitCCW);
	Serial.print(F("SETUP:   roboSetup.data.arm2LimitCW:       "));Serial.println(roboSetup.data.arm2LimitCW);
	Serial.print(F("SETUP:   roboSetup.data.arm2LimitCCW:      "));Serial.println(roboSetup.data.arm2LimitCCW);
	Serial.print(F("SETUP:   roboSetup.data.excludeBoxXLeft:   "));Serial.println(roboSetup.data.excludeBoxXLeft);
	Serial.print(F("SETUP:   roboSetup.data.excludeBoxXRight:  "));Serial.println(roboSetup.data.excludeBoxXRight);
	Serial.print(F("SETUP:   roboSetup.data.excludeBoxYTop:    "));Serial.println(roboSetup.data.excludeBoxYTop);
	Serial.print(F("SETUP:   roboSetup.data.excludeBoxYBottom: "));Serial.println(roboSetup.data.excludeBoxYBottom);
	Serial.print(F("SETUP:   arm1ZeroAngleOutOfLimit:          "));Serial.println(arm1ZeroAngleOutOfLimit);
	Serial.print(F("SETUP:   arm2ZeroAngleOutOfLimit:          "));Serial.println(arm2ZeroAngleOutOfLimit);
	Serial.print(F("SETUP:   stepsPerRevolution:               "));Serial.println(stepsPerRevolution);
	Serial.print(F("SETUP:   stepsPerRevolutionFP:             "));Serial.println(stepsPerRevolutionFP);
	Serial.print(F("SETUP:   swStepRate:                       "));Serial.println(motorSteppingRate);
	Serial.print(F("SETUP:   stepResolution:                   "));Serial.println(stepResolution);
}





/*

	Function:	checkLimits(void);
	
	Purpose:    Check angle limits for the new point and set flags in new point data structure 
				
	Parameters:	none
	            
	Returns:	Possible to get to point, or both angle types are out of bounds
				 
	
	Notes:		
	
	Description:   	Check Limits for Arm 1 & Arm 2 to see if out of bounds for both CW and CCW
				   
*/
int checkLimits(void){
	
	int		arm1Valid, arm2Valid;
	
	newPoint.angles[CW].validAngles  = true;
	newPoint.angles[CCW].validAngles = true;

//	Check CW angles first
	if (arm1ZeroAngleOutOfLimit){ arm1Valid  = ((newPoint.angles[CW].thArm1  >= roboSetup.data.arm1LimitCW) && (newPoint.angles[CW].thArm1  <= roboSetup.data.arm1LimitCCW));
//Serial.print(F("WORKING: A1V1: "));Serial.println(arm1Valid);
	} else {                      arm1Valid  = ((newPoint.angles[CW].thArm1  >= roboSetup.data.arm1LimitCW) || (newPoint.angles[CW].thArm1  <= roboSetup.data.arm1LimitCCW));
//Serial.print(F("WORKING: A1V2: "));Serial.println(arm1Valid);
	}
	if (arm2ZeroAngleOutOfLimit) { arm2Valid = ((newPoint.angles[CW].thArm2  >= roboSetup.data.arm2LimitCW) && (newPoint.angles[CW].thArm2  <= roboSetup.data.arm2LimitCCW));
//Serial.print(F("WORKING: A2V3: "));Serial.println(arm2Valid);
	} else {                       arm2Valid = ((newPoint.angles[CW].thArm2  >= roboSetup.data.arm2LimitCW) || (newPoint.angles[CW].thArm2  <= roboSetup.data.arm2LimitCCW));
//Serial.print(F("WORKING: A2V4: "));Serial.println(arm2Valid);
	}
	newPoint.angles[CW].validAngles = (arm1Valid && arm2Valid);

//	check CCW angles 
	if (arm1ZeroAngleOutOfLimit){ arm1Valid  = ((newPoint.angles[CCW].thArm1  >= roboSetup.data.arm1LimitCW) && (newPoint.angles[CCW].thArm1  <= roboSetup.data.arm1LimitCCW));
//Serial.print(F("WORKING: A1V5: "));Serial.println(arm1Valid);
	} else {                      arm1Valid  = ((newPoint.angles[CCW].thArm1  >= roboSetup.data.arm1LimitCW) || (newPoint.angles[CCW].thArm1  <= roboSetup.data.arm1LimitCCW));
//Serial.print(F("WORKING: A1V6: "));Serial.println(arm1Valid);
	}
	if (arm2ZeroAngleOutOfLimit) { arm2Valid = ((newPoint.angles[CCW].thArm2  >= roboSetup.data.arm2LimitCW) && (newPoint.angles[CCW].thArm2  <= roboSetup.data.arm2LimitCCW));
//Serial.print(F("WORKING: A2V7: "));Serial.println(arm2Valid);
	} else {                       arm2Valid = ((newPoint.angles[CCW].thArm2  >= roboSetup.data.arm2LimitCW) || (newPoint.angles[CCW].thArm2  <= roboSetup.data.arm2LimitCCW));
//Serial.print(F("WORKING: A2V8: "));Serial.println(arm2Valid);
	}
	newPoint.angles[CCW].validAngles = (arm1Valid && arm2Valid);
	
//Serial.print(F("WORKING: AngV: "));Serial.print(newPoint.angles[CW].validAngles);Serial.print(F("/"));Serial.println(newPoint.angles[CCW].validAngles);

	if (newPoint.angles[CW].validAngles || newPoint.angles[CCW].validAngles){
		return operationSuccess;
	}else{
		return errorArmAnglesOutOfBounds;
	}

 
 }





/*

	Function:	calcAngles(X, Y);
	
	Purpose:    find the angles for each arm for new point X,Y and set New point structures
	
	Parameters:	X     		- X coordinate of the point to move to
	            Y	     	- Y coordinate of the point to move to
				
	Returns:	Success OR failure reason
				angles in the move control structures for both intercept points
				
	Notes:		calculating the 2 sets of angles is aprox 15-30% faster than the old 
				method used to calculate the one set of angles.
	
	Description:   	Solve the simultaneous equations for both triangles on the CW side 
					of the line joining 0,0 and X,Y to get angles required for motors.
				   
*/
int calcAngles(float X, float Y) {

unsigned long	t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10;

//	local variables
	float		dia, dia2;				// length of diagonal ( dia ) and its square
	float		a1, a2;					// a1 + a2 = dia
	float		tha1, tha2, thb, thd;	// angles a1, a2, b & d
	float       thtmp;
	int			returnStatus;			// 0 success >0 for failure

	float oldth1, oldth2;

	t0=micros();t2=t0;t3=t0;t4=t0;t5=t0;t6=t0;t7=t0;t1=micros();
	
	newPoint.X   = X;
	newPoint.Y   = Y;
//	calc common factors used multiple times 
	dia2 = ( X * X ) + ( Y * Y );
	dia  = sqrt(dia2);
t2=micros();
//	check if the requested X,Y point allows the circles scribed by the 2 arms intersect
//	and return error if they don't
//	Needed to ensure maths don't break down and give errors
	if (dia < L1L2Diff){                              // check can reach point when arms folded on themselves
		returnStatus = errorPointTooCloseToOrigin;
		newPoint.angles[CW].validAngles = false;
		newPoint.angles[CCW].validAngles = false;
	}
	else if (dia > L1L2Sum){                          // check outstretched arms can reach this point
		returnStatus = errorPointTooFarForArmsToReach;
		newPoint.angles[CW].validAngles  = false;
		newPoint.angles[CCW].validAngles = false;
	}
	else{
//	calc a1 & a2
		a1   = (L1SQRminusL2SQR + dia2) / ( dia + dia );
		a2   = dia - a1;
t3=micros();
//	calc angles a1 a2 & b  --- these are negated for CW
		tha1 = acos(a1/L1);
t4=micros();
		tha2 = acos(a2/L2);
t5=micros();
		thb  = PI - tha1 - tha2;
//  calc angle of diagonal (incl adjustment for quadrant)
		thd  = ( (X==0.0) ? HALF_PI : atan(Y/X) ) + radAdjaTan(X,Y);
t6=micros();
		
//	set structure for CW angles and CCW angles.  Need to check they are in range 0-2PI
		thtmp = thd - tha1;   newPoint.angles[CW].thArm1   = ((thtmp<0) ? thtmp+TWO_PI : ((thtmp>TWO_PI) ? thtmp-TWO_PI : thtmp));
		thtmp = TWO_PI - thb; newPoint.angles[CW].thArm2   = ((thtmp<0) ? thtmp+TWO_PI : ((thtmp>TWO_PI) ? thtmp-TWO_PI : thtmp));
		thtmp = thd + tha1;   newPoint.angles[CCW].thArm1  = ((thtmp<0) ? thtmp+TWO_PI : ((thtmp>TWO_PI) ? thtmp-TWO_PI : thtmp));
		newPoint.angles[CCW].thArm2  = thb;
		newPoint.angles[CW].stepsA   = (int)(thSteps(newPoint.angles[CW].thArm1));
		newPoint.angles[CW].stepsB   = (int)(thSteps(newPoint.angles[CW].thArm2));
		newPoint.angles[CCW].stepsA  = (int)(thSteps(newPoint.angles[CCW].thArm1));
		newPoint.angles[CCW].stepsB  = (int)(thSteps(newPoint.angles[CCW].thArm2));
t7=micros();
		checkLimits();
		if ((newPoint.angles[CW].validAngles) || (newPoint.angles[CCW].validAngles)) {
			returnStatus = operationSuccess;				// Success
		} else {
			returnStatus = errorArmAnglesOutOfBounds;
		}
	} 
t8=micros();


//  Serial.print(F("WORKING:  dia: "));Serial.print(dia);  Serial.print(F("\t  a1: "));  Serial.print(a1);   Serial.print(F("\ta2: ")); Serial.print(a2);
//  Serial.print(F("\ttha1: "));Serial.print(tha1);        Serial.print(F("\ttha2: "));  Serial.print(tha2); Serial.print(F("\tthb: "));Serial.print(thb);Serial.print(F("\tthd: "));Serial.println(thd);
//  Serial.print(F("WORKING:   L1: "));Serial.print(L1);   Serial.print(F("\t  L2: "));  Serial.println(L2);


//  Serial.print(F("CW:        X: "));Serial.print(newPoint.X); Serial.print(F("\tY: "));Serial.print(newPoint.Y); Serial.print(F("\tth1: "));Serial.print(newPoint.angles[CW].thArm1); 
//  Serial.print(F("\tth2: "));Serial.print(newPoint.angles[CW].thArm2);
//  Serial.print(F("  Asteps: "));Serial.print(newPoint.angles[CW].stepsA);Serial.print(F("  Bsteps: "));Serial.println(newPoint.angles[CW].stepsB);
  
//  Serial.print(F("CCW:       X: "));Serial.print(newPoint.X);Serial.print(F("\tY: "));Serial.print(newPoint.Y);Serial.print(F("\tth1: "));Serial.print(newPoint.angles[CCW].thArm1);
//  Serial.print(F("\tth2: "));Serial.print(newPoint.angles[CCW].thArm2);
//  Serial.print(F("  Asteps: "));Serial.print(newPoint.angles[CCW].stepsA);Serial.print(F("  Bsteps: "));Serial.println(newPoint.angles[CCW].stepsB);
t9=micros();
//Serial.print(F("WORKING:  t1: "));Serial.print(t1);Serial.print(F("\tt2: "));Serial.print(t2);Serial.print(F("\tt3: "));Serial.print(t3);Serial.print(F("\tt4: "));Serial.print(t4);
//Serial.print(F("\tt5: "));Serial.print(t5);Serial.print(F("\tt6: "));Serial.print(t6);Serial.print(F("\tt7: "));Serial.print(t7);Serial.print(F("\tt8: "));Serial.print(t8);
//Serial.print(F("\tt9: "));Serial.println(t9);

//Serial.print(F("WORKING:  CA time for setup calcs:      "));Serial.println(t2-t1);
//Serial.print(F("WORKING:  CA time for a1 a2 calcs:      "));Serial.print(t3-t2);Serial.print(F("  \t"));Serial.println(t3-t1);
//Serial.print(F("WORKING:  CA time for th1  acos calc:   "));Serial.print(t4-t3);Serial.print(F("  \t"));Serial.println(t4-t1);
//Serial.print(F("WORKING:  CA time for tha2 acos calc:   "));Serial.print(t5-t4);Serial.print(F("  \t"));Serial.println(t5-t1);
//Serial.print(F("WORKING:  CA time for thd  atan calc:   "));Serial.print(t6-t5);Serial.print(F("  \t"));Serial.println(t6-t1);
//Serial.print(F("WORKING:  CA time for 4 x steps calc:   "));Serial.print(t7-t6);Serial.print(F("  \t"));Serial.println(t7-t1);
//Serial.print(F("WORKING:  CA time for return status:    "));Serial.print(t8-t7);Serial.print(F("  \t"));Serial.println(t8-t1);
//Serial.print(F("WORKING:  CA time for angles debug prt: "));Serial.print(t9-t8);Serial.print(F("  \t"));Serial.println(t9-t1);
//t10=micros();Serial.println(t10);

	return returnStatus;
  
}





/*

	Function:	findStepDir(stepFrom, stepATo, stepLimitCW, stepLimitCCW);
	
	Purpose:    Find which direction to move when moving from one step position to another 
				
	Parameters:	stepFrom		- FROM this step position
				stepTo			- To this step position
				stepLimitCW		- CW step limit
				stepLimitCCW	- CCW step limit
				
	Returns:	Movement direction (CW or CCW) or impossible
				 
	
	Notes:		No difference to which motor the 'from/to' steps belong to
	
	Description:   	Using the limits determine which direction the motor should move
					In most cases thos will prevent the arms "crashing" it is still possible
					to have a crash if the from position is close to the structure and the to position
					is also close to the structure but on the other side.  This hopefully would be rare 
					since that would assume that a drawing has a "hole" in the middle of it where the
					structure is.  It would seem to be a rare requirement and could be worked around by
					doing two drawings keeping the problem areas on separate draws.
					
					The logic is to see if either limit is within the motion in a direction. If
					both directions have a limit within the motion then either the 'from' or 'to' 
					is within a forbidden angle area. And is an impossible movement situation resulting
					in no movement allowed.
				   
*/
int findStepDir(unsigned int sF, unsigned int sT, unsigned int sLCW, unsigned int sLCCW) {

	int		direction;
	int		failed;
	
	direction = impossibleMoveDirection;

//	check CW first.  If the motion passes through step 0 then it involves two tests
	if (sF < sT) { failed = ( ((sLCW <= sF) || (sLCW >= sT))  ||  ((sLCCW <= sF) || (sLCCW >= sT)) );
//Serial.print(F("WORKING:ChkD1: "));Serial.println(failed);
	} else {       failed = ( ((sLCW >= sT) && (sLCW <= sF))  ||  ((sLCCW >= sT) && (sLCCW <= sF)) );
//Serial.print(F("WORKING:ChkD2: "));Serial.println(failed);
	}
	if (failed == false) { 
		direction = CW;
	} else {
//		check CCW only if CW failed  (only one way will be valid)
		if (sF < sT) { failed = ( ((sLCW >= sF) && (sLCW <= sT)) || ((sLCCW >= sF) && (sLCCW <= sT)) );
//Serial.print(F("WORKING:ChkD3: "));Serial.println(failed);
		} else {       failed = ( ((sLCW <= sT) || (sLCW >= sF)) || ((sLCCW <= sT) || (sLCCW >= sF)) );
//Serial.print(F("WORKING:ChkD4: "));Serial.println(failed);
		}
		if (failed == false) {
			direction = CCW;
		}
	}
//Serial.print(F("WORKING:ChkD?: "));Serial.println(direction);

	
	return direction;
}


/*

	Function:	stepMotorA(int moveDir);
	Function:	stepMotorB(int moveDir);
	
	Purpose:    Pulse the motor controller with the correct direction signal
				
	Parameters:	moveDir			- FROM this step position
				
	Returns:	nothing
				 
	
	Notes:		
	
	Description:   	Use the moveDir and the configuration motor orientation to set direction bit
					and pulse the step control line.
					CW is "normal" direction which is motor wiring is Actual CW == CW
					 wiring         moveDir      Dir_line
					   CW              CW            LOW
					   CW             CCW            HIGH
					  CCW              CW            HIGH
					  CCW             CCW            LOW
					In other words if wiring == moveDir then dir_line is LOW  
					               if wiring != moveDir then dir_line is HIGH
	
*/
void stepMotorA(int moveDir){
//  Serial.print(F("stepper A: "));Serial.println(moveDir);
static long ta1=0, ta2=0;
int 	i;

ta1=micros(); if (ta2 < 100) { ta2=ta1; }

  if(moveDir == roboSetup.data.motorADir){
    stpA.dWrite1(LOW);
	i=-1;
  }else{
    stpA.dWrite1(HIGH);
	i=1;
  }
  stpA.dWrite2(HIGH);
  stpA.dWrite2(LOW);

//Serial.print(F("DEBUG: StepA:    NO: "));Serial.print(cmdNo);Serial.print(F(" \tStepA: "));Serial.print(i);Serial.print(F(" \tDelay: "));Serial.println(ta1-ta2);
ta2=micros();  
}

void stepMotorB(int moveDir){
//  Serial.print(F("stepper B: "));Serial.println(moveDir);
static long tb1=0, tb2=0;
int 	i;

tb1=micros(); if (tb2 < 100) { tb2=tb1; }

  if(moveDir == roboSetup.data.motorBDir){
    stpB.dWrite1(LOW);
	i=-1;
  }else{
    stpB.dWrite1(HIGH);
	i=1;
  }
  stpB.dWrite2(HIGH);
  stpB.dWrite2(LOW);

//Serial.print(F("DEBUG: StepB:    NO: "));Serial.print(cmdNo);Serial.print(F(" \tStepB: "));Serial.print(i);Serial.print(F(" \tDelay: "));Serial.println(tb1-tb2);
tb2=micros();  
}





/*

	Function:	moveSteps(stepA, stepB);
	
	Purpose:    Move the motors from current position to supplied steps
				
	Parameters:	Step position for motor A & B
	            
	Returns:	nothing
				 
	
	Notes:		Acceleration now uses a table and the motor with most steps has the timing applied
				since the other motor will be going proportionally slower.
	
	Description:   	Do the move to the supplied steps.
					The direction of the move is determined by the motor directions and whichever
					way that doesn't move through the angle limits.  If new point can only be 
					achieved with the other angle type then swap angle types before doing the
					move with pen up.
					Do nothing if new point has no valid angles.
					Proportional stepping is done by having counters set to the other #steps 
					and decrementing by minimum steps.  This save compensating for floating
					point inaccuracies and saves time as well (FP maths is relatively slow)
				   
*/
unsigned int stepAuxDelay=0;
unsigned long tmr1,tmr2;
int moveSteps(int stepA, int stepB){

unsigned long	t0,t1,t2,t3,t4,t5,t6,t7;


	int				dirA, dirB;
	int				retCode;
	int				numStepsA, numStepsB;
	int				xA, xB, xAcc;						// provide proportional stepping counters
	int				minSteps, maxSteps, halfMinSteps;	// need these for proportional factors
	int				stepsLeftA, stepsLeftB, accStep;	// counters for # steps left to move
	int				accelLast, decelStart, currAccel;	// step count for last accel, first decel and current accel entry 
	unsigned long	delayTime;							// current delay time
	int				i, j;
	
#define  approxStepLoopTime           44               // uSeconds for loop  (1 motor step 44uSecs and 2 motor step 80uSecs)
//stepAuxDelay=10550;
t0=micros();t2=t0;t3=t0;t4=t0;t5=t0;t6=t0;t7=t0;t1=micros();
	dirA = findStepDir(currPoint.angles[currPoint.angleType].stepsA, stepA, motorAStepsLimitCW, motorAStepsLimitCCW);
	dirB = findStepDir(currPoint.angles[currPoint.angleType].stepsB, stepB, motorBStepsLimitCW, motorBStepsLimitCCW);

	if ( (dirA == impossibleMoveDirection) || (dirB == impossibleMoveDirection) ) {
		retCode = errorImpossibleMove;
	} else {
//		work out number of steps to move & direction for each motor
		numStepsA  = currPoint.angles[currPoint.angleType].stepsA - stepA;		// get delta steps
		numStepsA  = ((dirA==CW)? numStepsA:-numStepsA);						// adjust for direction
		numStepsA += ((numStepsA<0)?stepsPerRevolution:0);						// normalise to 0 to max
		numStepsB  = currPoint.angles[currPoint.angleType].stepsB - stepB;		// get delta steps
		numStepsB  = ((dirB==CW)? numStepsB:-numStepsB);						// adjust for direction
		numStepsB += ((numStepsB<0)?stepsPerRevolution:0);						// normalise to 0 to max
//Serial.print(F("DEBUG:   NEW:    NO: "));Serial.print(cmdNo);Serial.print(F(" \tStepsA: "));Serial.print(stepA);Serial.print(F("  "));
//if (dirA==CCW) { Serial.print(F("-")); }
//Serial.print(numStepsA);Serial.print(F(" \tStepsB: "));Serial.print(stepB);Serial.print(F("  "));
//if (dirB==CCW) { Serial.print(F("-")); }
//Serial.println(numStepsB);

//		Initialise parameters for looping
		if (numStepsA > numStepsB) { minSteps=numStepsB; maxSteps=numStepsA; }				// set max/min steps 
		else                       { minSteps=numStepsA; maxSteps=numStepsB; }
		halfMinSteps  = minSteps / 2;														// To know limit for acceleration
		xA=numStepsB;         xB=numStepsA;        											// initialise proportional counters
		stepsLeftA=numStepsA; stepsLeftB=numStepsB;	accStep=0;								// initialise number of steps left to move
		accelLast  = ((motorAccelEntries>halfMinSteps)?halfMinSteps:motorAccelEntries);		// only use accel periods for upto int(maxsteps/2) or # of accel steps
		decelStart = maxSteps - ( accelLast - 1);											// first reduced rate (increased period) step
		currAccel  = 0;																		// initialise current acceleration entry
tmr1=micros();
//		Do the stepping with acceleration/deceleration
		while ((stepsLeftA>0)||(stepsLeftB>0)){									// loop until all steps done
			xA -= minSteps;
			xB -= minSteps;
			if (xA <= 0){ stepMotorA(dirA); xA += numStepsB; stepsLeftA--; }
			if (xB <= 0){ stepMotorB(dirB); xB += numStepsA; stepsLeftB--; }
			accStep++;
//			find the delay period.  the accel table is origin 1 since element 0 is settling time.  AccStep is also origin 1
			if      (accStep <= accelLast) { delayTime = (long)(pgm_read_word_near(motorAccel +  accStep)); }				// acceleration phase
			else if (accStep < decelStart) { delayTime = (long)(pgm_read_word_near(motorAccel +  accelLast)); }				// coasting phase
			else if (accStep < maxSteps )  { delayTime = (long)(pgm_read_word_near(motorAccel +  (maxSteps-accStep))); }	// deceleration phase
			else                           { delayTime = (long)(pgm_read_word_near(motorAccel +  0)); }						// last step - settling time delay
			delayTime += (long)stepAuxDelay;
//			delayTime -= (long)approxStepLoopTime;											// account for approx time spent in loop
//			Apply the speed factor to the delay to reduce speed  (speed factor is in %age)  increase delay by 100/factor is same as % of speed
			delayTime = ( delayTime * 100L ) / (long)roboSetup.data.speed;
//			delayMicroseconds has problems if the value gets too high.  8000 is a safe delay time.
			while (delayTime > 8192L) { delayMicroseconds(8000); delayTime -= 8000L; }
			i = (int)delayTime;	
			delayMicroseconds(i);
			  // set laser on for time set by power level
		}
		retCode = operationSuccess;
Serial.println(" ");
t6=micros();
	}

//Serial.print(F("WORKING:  t1: "));Serial.print(t1);Serial.print(F("\tt2: "));Serial.print(t2);Serial.print(F("\tt3: "));Serial.print(t3);Serial.print(F("\tt4: "));Serial.print(t4);
//Serial.print(F("\tt5: "));Serial.print(t5);Serial.print(F("\tt6: "));Serial.println(t6);

//Serial.print(F("WORKING:  MS time for direction  calcs: "));Serial.println(t2-t1);
//Serial.print(F("WORKING:  MS time for deltaSteps calcs: "));Serial.print(t3-t2);Serial.print(F("  \t"));Serial.println(t3-t1);
//Serial.print(F("WORKING:  MS time for prints:           "));Serial.print(t4-t3);Serial.print(F("  \t"));Serial.println(t4-t1);
//Serial.print(F("WORKING:  MS time for loop preperation: "));Serial.print(t5-t4);Serial.print(F("  \t"));Serial.println(t5-t1);
//Serial.print(F("WORKING:  MS time for loops:            "));Serial.print(t6-t5);Serial.print(F("  \t"));Serial.println(t6-t1);
//t7=micros();Serial.println(t7);

		return retCode;
}


/*

	Function:	moveToNewPoint(X, Y);
	
	Purpose:    Do the move to the new point
				
	Parameters:	none
	            
	Returns:	nothing
				 
	
	Notes:		
	
	Description:   	Do the move to the new point from the old point
					If the new point cannot be positioned using the current angle type
					then swap angle types before doing the move with pen up.
					Do nothing if new point has no valid angles.
				   
*/
int moveToNewPoint(float X, float Y) {

unsigned long	t0,t1,t2,t3,t4,t5,t6,t7;

	int					otherType;
	int					i;
	int					result;
	scaraMoveControl	tmpPoint;
	int					d;
	unsigned long 		dly;

t0=micros();t2=t0;t3=t0;t4=t0;t5=t0;t6=t0;t7=t0;t1=micros();
	
//	need to have a pointer to the other type for testing
	otherType = ( (currPoint.angleType == CW) ? CCW : CW );
t2=micros();	

//	setup newpoint structure
//	if neither valid then do nothing and leave it at the current point.
	result=calcAngles(X, Y);
t3=micros();

//		if new point has current angle type as invalid and if other angle type for the new 
//		point is valid then swap angle type before moving to new point. But if other angle
//		type for the current point is invalid then have to go home and then swap.
//******NOTE:  Home is always considered a valid point with both angle types valid
	if ( (result == operationSuccess) &&
		 ( (! newPoint.angles[currPoint.angleType].validAngles) && (newPoint.angles[otherType].validAngles)             ) ) { 
		servoPen.write(roboSetup.data.servoPenUp);                         // move the pen up
		if (currPoint.mode == drawMode) { laser.run(0); }                  // make sure laser is off (only need to consider laser power if in draw mode)
		if (!currPoint.angles[otherType].validAngles)  {                   // invalid other angle type, so pen up and go home swapping angle type
			Serial.println(F("Move Home and Swap Angle Types "));
			tmpPoint = newPoint;
			result = calcAngles(-(roboSetup.data.arm0len+roboSetup.data.arm1len), 0);
			result = moveSteps(newPoint.angles[currPoint.angleType].stepsA,newPoint.angles[currPoint.angleType].stepsB);
			if (result == operationSuccess) { currPoint = newPoint; currPoint.angleType = otherType; } else { Serial.println(F("Failed to swap angles")); }
			newPoint  = tmpPoint;

		} else {                                                           // valid other angle type, so pen up and swap sides
			Serial.println(F("Swapping Angle Types "));
			result = moveSteps(currPoint.angles[otherType].stepsA, currPoint.angles[otherType].stepsB);
			if (result == operationSuccess) { currPoint.angleType = otherType; } else { Serial.println(F("Failed to swap angles")); }
		}
		servoPen.write(servoPositionCurrent);                              // move the pen back to current position
		if (currPoint.mode == drawMode) { laser.run(laserPowerCurrent); }  // restore laser to current power
	}
t4=micros();

//		if new point has current angle type as valid then move
	if ( (result == operationSuccess) &&
		 (newPoint.angles[currPoint.angleType].validAngles) ) {
//		Serial.println(F("Moving to New Position"));
		result = moveSteps(newPoint.angles[currPoint.angleType].stepsA,newPoint.angles[currPoint.angleType].stepsB);
		if (result == operationSuccess) { 
			newPoint.angleType = currPoint.angleType;
			currPoint = newPoint;
		} else {
			Serial.println(F("Failed to move to new point"));
		}
	} else {
		result = errorImpossibleMove;
	}
t5=micros();
	if (result != operationSuccess) {
		i=0;
//		need to remember the current pen state so it can be reinstated to cmd position on end of next move
//				reinstated at end of next move because the current move should have moved to a new position but it did not
//				and the next move thinks it is moving from that new position, so want pen up otherwise get unwanted line from old position (missing this one) to next position.
		servoPen.write(roboSetup.data.servoPenUp);                         // move the pen up
		servoPositionCurrent = roboSetup.data.servoPenUp;                  // remember the pen is currently up
		laser.run(0);                                                      // turn off laser
		laserPowerCurrent    = 0;                                          // remember laser is currently off
		Serial.println(F("Cannot move to this position"));
	} else {

		if (servoPositionCurrent != servoPositionLastCommand) {
			servoPen.write(servoPositionLastCommand);                      // reinstate the commanded pen position
			servoPositionCurrent = servoPositionLastCommand;
			}

		if ( (laserPowerCurrent != laserPowerLastCommand) && (currPoint.mode == drawMode) ) {
			laser.run(laserPowerLastCommand);                              // restore laser to last commanded power
			laserPowerCurrent = laserPowerLastCommand;
			}  
		
		if (currPoint.mode == auxMode) {
//			turn on aux for delay time
			dly = currPoint.pointDelay;
			auxDevice.run(255);
			while (dly > 8192L) { delayMicroseconds(8000); dly -= 8000L; }
			d = (int)dly;	
			delayMicroseconds(d);
			auxDevice.run(0);

		}
	}
			
				
			
t6=micros();
//Serial.print(F("WORKING:  t1: "));Serial.print(t1);Serial.print(F("\tt2: "));Serial.print(t2);Serial.print(F("\tt3: "));Serial.print(t3);Serial.print(F("\tt4: "));Serial.print(t4);
//Serial.print(F("\tt5: "));Serial.print(t5);Serial.print(F("\tt6: "));Serial.println(t6);

//Serial.print(F("WORKING:  MP time for otherType  calcs: "));Serial.println(t2-t1);
//Serial.print(F("WORKING:  MP time for calcAngles calcs: "));Serial.print(t3-t2);Serial.print(F("  \t"));Serial.println(t3-t1);
//Serial.print(F("WORKING:  MP time for swap Angles:      "));Serial.print(t4-t3);Serial.print(F("  \t"));Serial.println(t4-t1);
//Serial.print(F("WORKING:  MP time for actual move:      "));Serial.print(t5-t4);Serial.print(F("  \t"));Serial.println(t5-t1);
//***Serial.print(F("WORKING:  MP Time for Calcs:  "));Serial.print(tmr1-t1); Serial.print(F("  \t"));Serial.println(t6-t1);
//Serial.print(F("WORKING:  MP total time for moveNewPt:  "));Serial.print(t6-t5);Serial.print(F("  \t"));Serial.println(t6-t1);
//t7=micros();Serial.println(t7);
}


/*

	Function:	checkNoGoArea(X, Y);
	
	Purpose:    Check if the X,Y point is outside the No Go Box
				
	Parameters:	X, Y coordinates
	            
	Returns:	success/failure   (operationSuccess/errorPointTooCloseToOrigin)
				 
	
	Notes:		
	
	Description:   	Returns failure if the X,Y point is within the Box
				   
*/
int checkNoGoArea(float X, float Y) {

	if ( (X >= roboSetup.data.excludeBoxXLeft) && (X <= roboSetup.data.excludeBoxXRight) &&
	     (Y <= roboSetup.data.excludeBoxYTop)  && (Y >= roboSetup.data.excludeBoxYBottom) ) {
		return errorPointTooCloseToOrigin;
	} else {
		return operationSuccess;
	}
}



// arduino only handle A,B step mapping
float curSpd,tarSpd; // speed profile
float curX,curY,curZ;
float tarX,tarY,tarZ; // target xyz position
float curTh1, curTh2;
float tarTh1, tarTh2; // target angle of joint
int tarA,tarB,posA,posB; // target stepper position
int8_t motorAfw,motorAbk;
int8_t motorBfw,motorBbk;

#define ARML1 168
#define ARML2 206
/************** motor movements ******************/
void stepperMoveA(int dir)
{
//  Serial.print(F("stepper A: "));Serial.println(dir);
  if(dir>0){
    stpA.dWrite1(LOW);
  }else{
    stpA.dWrite1(HIGH);
  }
  stpA.dWrite2(HIGH);
  stpA.dWrite2(LOW);
}

void stepperMoveB(int dir)
{
//  Serial.print(F("stepper B: "));Serial.println(dir);
  if(dir>0){
    stpB.dWrite1(LOW);
  }else{
    stpB.dWrite1(HIGH);
  }
  stpB.dWrite2(HIGH);
  stpB.dWrite2(LOW);
}

/************** scara inversekinect **********************/
// th1 solution
//2*atan((2*L1*y + (- L1^4 + 2*L1^2*L2^2 + 2*L1^2*x^2 + 2*L1^2*y^2 - L2^4 + 2*L2^2*x^2 + 2*L2^2*y^2 - x^4 - 2*x^2*y^2 - y^4)^(1/2))/(L1^2 - 2*L1*x - L2^2 + x^2 + y^2))
// th2 solution
//2*atan(((- L1^2 + 2*L1*L2 - L2^2 + x^2 + y^2)*(L1^2 + 2*L1*L2 + L2^2 - x^2 - y^2))^(1/2)/(L1^2 + 2*L1*L2 + L2^2 - x^2 - y^2))
float th1,th2;
void scaraInverseKinect(float x, float y)
{
  float L1 = roboSetup.data.arm0len;
  float L2 = roboSetup.data.arm1len;
  //Serial.print(F("x "));Serial.println(x);
  //Serial.print(F("y "));Serial.println(y);
  th1 = 2*atan((2*L1*y + sqrt(- L1*L1*L1*L1 + 2*L1*L1*L2*L2 + 2*L1*L1*x*x + 2*L1*L1*y*y - L2*L2*L2*L2 + 2*L2*L2*x*x + 2*L2*L2*y*y - x*x*x*x - 2*x*x*y*y - y*y*y*y))/(L1*L1 - 2*L1*x - L2*L2 + x*x + y*y));
  th2 = 2*atan(sqrt((- L1*L1 + 2*L1*L2 - L2*L2 + x*x + y*y)*(L1*L1 + 2*L1*L2 + L2*L2 - x*x - y*y))/(L1*L1 + 2*L1*L2 + L2*L2 - x*x - y*y));
  //Serial.print(F("th1 "));Serial.println(th1/PI*180);
  //Serial.print(F("th2 "));Serial.println(th2/PI*180);
//  Serial.print(F("ORIG:      X: "));Serial.print(x);Serial.print(F("\tY: "));Serial.print(y);Serial.print(F("\tth1: "));Serial.print(th1/PI*180);Serial.print(F("\tth2: "));Serial.println(th2/PI*180);
}

#define STEPS_PER_CIRCLE 16000.0f
long pos1,pos2;
void thetaToSteps(float th1, float th2)
{
  pos1 = round(th1/PI*STEPS_PER_CIRCLE/2);
  pos2 = round(th2/PI*STEPS_PER_CIRCLE/2);
}

/*
	Robbo1 8/June/2015
	
	Fixed loop test where the movement may not have actually stopped
	
	Sympton  - The movement would stop sometimes one step short because of floating point values 
	           not always the exact amount and 'N' additions of 'N' segments may not add up to 
			   the whole.  The best way is to loop until all steps have been done.
			   
	Solution - Change the loop finish test from 'i<maxD' to '(posA!=tarA)||(posB!=tarB)'
	           That is test for movement not yet finished
			   This is a miniminal change to enable the least changes to the code
			   
*/
/************** calculate movements ******************/
//#define STEPDELAY_MIN 200 // micro second
//#define STEPDELAY_MAX 1000
//int stepAuxDelay=0;
int stepdelay_min=200;
int stepdelay_max=2000;
#define ACCELERATION 2 // mm/s^2 don't get inertia exceed motor could handle
#define SEGMENT_DISTANCE 10 // 1 mm for each segment
#define SPEED_STEP 1

void doMove()
{
  int mDelay=stepdelay_max;
  int speedDiff = -SPEED_STEP;
  int dA,dB,maxD;
  float stepA,stepB,cntA=0,cntB=0;
  int d;
  dA = tarA - posA;
  dB = tarB - posB;
Serial.print(F("oldMSteps:  deltaA:"));Serial.print(dA);Serial.print(F("/"));
if (dA>=0) { Serial.print(F("CW")); } else { Serial.print(F("CCW")); }
Serial.print(F("    deltaB:"));Serial.print(dB);Serial.print(F("/"));
if (dB>=0) { Serial.print(F("CW")); } else { Serial.print(F("CCW")); }
Serial.println(F(" "));
  maxD = max(abs(dA),abs(dB));
  stepA = (float)abs(dA)/(float)maxD;
  stepB = (float)abs(dB)/(float)maxD;
  //Serial.printf("move: max:%d da:%d db:%d\n",maxD,dA,dB);
  //Serial.print(stepA);Serial.print(' ');Serial.println(stepB);
  //for(int i=0;i<=maxD;i++){
  //for(int i=0;i<maxD;i++){                                           // Robbo1 2015/6/8 Removed - kept for now to show what the loop looked like before
  for(int i=0;(posA!=tarA)||(posB!=tarB);i++){                         // Robbo1 2015/6/8 Changed - change loop terminate test to test for moving not finished rather than a preset amount of moves
    //Serial.printf("step %d A:%d B;%d\n",i,posA,posB);
    // move A
    if(posA!=tarA){
      cntA+=stepA;
      if(cntA>=1){
        d = dA>0?motorAfw:motorAbk;
        stepperMoveA(d);
        cntA-=1;
        posA+=d;
      }
    }
    // move B
    if(posB!=tarB){
      cntB+=stepB;
      if(cntB>=1){
        d = dB>0?motorBfw:motorBbk;
        stepperMoveB(d);
        cntB-=1;
        posB+=d;
      }
    }
    mDelay=constrain(mDelay+speedDiff,stepdelay_min,stepdelay_max)+stepAuxDelay;
    delayMicroseconds(mDelay);
    if((maxD-i)<((stepdelay_max-stepdelay_min)/SPEED_STEP)){
      speedDiff=SPEED_STEP;
    }
  }
  //Serial.printf("finally %d A:%d B;%d\n",maxD,posA,posB);
  posA = tarA;
  posB = tarB;
}

void prepareMove()
{
	unsigned long        time1, time2, time3;
	int                  ii;
	time1 = micros();

  int maxD;
  unsigned long t0,t1;
  float segInterval;
  float dx = tarX - curX;
  float dy = tarY - curY;
  float distance = sqrt(dx*dx+dy*dy);
  float distanceMoved=0,distanceLast=0;
  //Serial.print("distance=");Serial.println(distance);
  if (distance < 0.001) 
    return;
	if (checkNoGoArea(tarX, tarY) != operationSuccess)  {
		servoPen.write(roboSetup.data.servoPenUp);                         // move the pen up
		servoPositionCurrent = roboSetup.data.servoPenUp;
		if (currPoint.mode == drawMode) { 
			laser.run(0);                                                  // make sure laser is off (only need to consider laser power if in draw mode)
			laserPowerCurrent = 0;
		}
		Serial.println(F("Move Aborted - X,Y within Exclude Box"));
		return;			// Abort move if an attempt is made to move to a point within the Exclude Box
	}
  scaraInverseKinect(tarX,tarY);
  thetaToSteps(th1, th2);
  tarA = pos1;tarB = pos2;
  //Serial.print("theta:");Serial.print(th1/PI*180);Serial.print(' ');Serial.println(th2/PI*180);
  //Serial.printf("tar Pos %d %d\r\n",tarA,tarB);
  time2 = micros();       													// **************************************************************************************
  time3 = time2 - time1;       													// **************************************************************************************
  cmdNo++;       													// **************************************************************************************
//  Serial.print(F("Time for old angles calc: "));Serial.println(time3);
//  Serial.print(F("DEBUG:   OLD:    NO: "));Serial.print(cmdNo);Serial.print(F(" \tStepsA: "));Serial.print(tarA);Serial.print(F("  "));Serial.print(tarA-posA);Serial.print(F(" \tStepsB: "));Serial.print(tarB);Serial.print(F("  "));Serial.println(tarB-posB);
//Serial.print(F("DEBUG:   OLD:    NO: "));Serial.print(cmdNo);Serial.print(F(" \tX: "));     Serial.print(tarX);     Serial.print(F(" \tY: "));     Serial.print(tarY);
//                                                             Serial.print(F(" \tth1: "));   Serial.print(PI-th1);   Serial.print(F(" \tth2: "));   Serial.print(TWO_PI-th2);
//                                                             Serial.print(F(" \tStepsA: "));Serial.print(8000-tarA);Serial.print(F(" \tStepsB: "));Serial.print(16000-tarB);
//										                     Serial.println(F(" "));
//  doMove();       													// **************************************************************************************
  posA = tarA;       													// *******normally done in domove*******************************************************************************
  posB = tarB;       													// *******normally done in domove*******************************************************************************
  
  time1 = micros();
  moveToNewPoint(tarX, tarY);
  time2 = micros();
  time3 = time2 - time1;
//ii=currPoint.angleType;
//Serial.print(F("DEBUG:   NEW:    NO: "));Serial.print(cmdNo);Serial.print(F(" \tX: "));     Serial.print(tarX);                        Serial.print(F(" \tY: "));     Serial.print(tarY);
//                                                             Serial.print(F(" \tth1: "));   Serial.print(currPoint.angles[ii].thArm1); Serial.print(F(" \tth2: "));   Serial.print(currPoint.angles[ii].thArm2);
//                                                             Serial.print(F(" \tStepsA: "));Serial.print(currPoint.angles[ii].stepsA); Serial.print(F(" \tStepsB: "));Serial.print(currPoint.angles[ii].stepsB);
//										                     Serial.println(F(" "));
//  Serial.print(F("Time for move to new point: "));Serial.println(time3);
  
  curX = tarX;
  curY = tarY;
}

void initPosition()
{
  curX=-(roboSetup.data.arm0len+roboSetup.data.arm1len-0.01); curY=0;
  scaraInverseKinect(curX,curY);                 
  curTh1=th1;curTh2=th2;
  thetaToSteps(curTh1,curTh2);
  posA = pos1;
  posB = pos2;
	calcAngles(curX, curY);
	currPoint = newPoint;
	currPoint.angleType = CCW;
	if (currPoint.angles[CW].validAngles) { currPoint.angleType = CW; }
}

/*
	Robbo1 8/June/2015
	
	Fixed loop test where phantom parameters overwrite actual parameters and cause wild motions in scara
	
	Sympton  - The test was testing the previous loop's pointer and when last token/parameter is processed 
	           It would then loop one more time.  This meant that the loop used the NULL pointer as the 
			   string pointer.  Now if the bytes at address zero happened to start with the characters X or Y or Z or F or A
			   the loop would process that phantom string and convert the following bytes into a number and 
			   replace the actual parameters value with the phantom one.
			  
			   While this happened only in certain circumstances, it did happen for some svg files that 
			   happened to be placed in certain areas, because it seems the conversion codes would place 
			   intermediate data at addres 0 and if that data resulted in the byte at addres 0 being
			   one of the parameter labels (X Y Z F A) then the problem occurred.
			  
			   This explains why some people had wild things happen with the arms rotating all the way and 
			   crashing into the framework
			  
	Solution - Move the strtok_r to the while loop test which means that the loop test is done on the
	           current pointer.  This means that when there are no more tokens (str == NULL) the loop
			   terminates without processing it.
			   
	Other identified issues
				If the code is changed prior to calling the function, the initial strtok_r may gobble 
				up a parameter.  It is currently there because the "G" code is not removed from the 
				cmd string prior to calling the function and has to be removed prior to processing the
				parameters.
				
*/

/************** calculate movements ******************/
void parseCordinate(char * cmd)
{
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);             // Robbo1 2015/6/8 comment - this removes the G code token from the input string - potential for future errors if method of processing G codes changes
  while((str=strtok_r(0, " ", &tmp))!=NULL){  // Robbo1 2015/6/8 changed - placed strtok_r into loop test so that the pointer tested is the one used in the current loop
    //str = strtok_r(0, " ", &tmp);           // Robbo1 2015/6/8 removed - removed from here and place in the while loop test
    //Serial.printf("%s;",str);
    if(str[0]=='X'){
      tarX = atof(str+1);
    }else if(str[0]=='Y'){
      tarY = atof(str+1);
    }else if(str[0]=='Z'){
      tarZ = atof(str+1);
    }else if(str[0]=='F'){
      float speed = atof(str+1);
      tarSpd = speed/60; // mm/min -> mm/s
    }else if(str[0]=='A'){
      stepAuxDelay = atoi(str+1);
	}else if(str[0]=='D'){
	  newPoint.pointDelay = atol(str+1);
    }
  }
  prepareMove();
}


void parseServo(char * cmd)
{
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int pos = atoi(tmp);
  servoPositionLastCommand = pos;
  servoPen.write(pos);
  servoPositionCurrent     = pos;
}

void parseAuxDelay(char * cmd)
{
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  stepAuxDelay = atoi(tmp);
}

void parseLaserPower(char * cmd)
{
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int pwm = atoi(tmp);
  laserPowerLastCommand = pwm;
  laser.run(pwm);
  laserPowerCurrent = pwm;
}

void parseGcode(char * cmd)
{
  int code;
  code = atoi(cmd);
  newPoint.mode = drawMode;
  switch(code){
    case 1: // xyz move
      parseCordinate(cmd);
      break;
    case 28: // home
      stepAuxDelay = 0;
      tarX=-(roboSetup.data.arm0len+roboSetup.data.arm1len-0.01); tarY=0;
      prepareMove();
      break; 
	case 127: // Aux mode - unimplemented experiemental use at own risk!!!!!!  Destroy your bot in one easy operation :)
	  newPoint.mode = auxMode;
      parseCordinate(cmd);
      break;
  }
}

void echoArmSetup(char * cmd)
{
  Serial.print(F("M10 MSCARA "));
  Serial.print(roboSetup.data.arm0len);Serial.print(F(" "));
  Serial.print(roboSetup.data.arm1len);Serial.print(F(" "));
  Serial.print(curX);Serial.print(F(" "));
  Serial.print(curY);Serial.print(F(" "));
  Serial.print(F("A"));Serial.print((int)roboSetup.data.motorADir);
  Serial.print(F(" B"));Serial.print((int)roboSetup.data.motorBDir);
  Serial.print(F(" D"));Serial.println((int)roboSetup.data.speed);
}

void parseRobotSetup(char * cmd)
{
  char * tmp;
  char * str;
  //	set defaults in case setup does not --- This is to be removed when mDraw includes these in its configuration
  roboSetup.data.arm1LimitCCW       = arm1LimitCCWDefault;
  roboSetup.data.arm1LimitCW        = arm1LimitCWDefault;
  roboSetup.data.arm2LimitCCW       = arm2LimitCCWDefault;
  roboSetup.data.arm2LimitCW        = arm2LimitCWDefault;
  roboSetup.data.excludeBoxXLeft    = excludeBoxXLeftDefault;
  roboSetup.data.excludeBoxXRight   = excludeBoxXRightDefault;
  roboSetup.data.excludeBoxYTop     = excludeBoxYTopDefault;
  roboSetup.data.excludeBoxYBottom  = excludeBoxYBottomDefault;
  roboSetup.data.swStepRate         = softSteppingDefault;
  roboSetup.data.stepsPerRevolution = stepsPerRevolutionDefault;
  roboSetup.data.servoPenUp	        = servoPositionPenUpDefault;
  roboSetup.data.servoPenDown       = servoPositionPenDownDefault;
  str = strtok_r(cmd, " ", &tmp);
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='A'){
      roboSetup.data.motorADir = atoi(str+1);
      Serial.print(F("motorADir "));Serial.print(roboSetup.data.motorADir);
    }else if(str[0]=='B'){
      roboSetup.data.motorBDir = atoi(str+1);
      Serial.print(F("motorBDir "));Serial.print(roboSetup.data.motorBDir);
    }else if(str[0]=='M'){
      roboSetup.data.arm0len = atoi(str+1);
      Serial.print(F("ARML1 "));Serial.print(roboSetup.data.arm0len);
    }else if(str[0]=='N'){
      roboSetup.data.arm1len = atoi(str+1);
      Serial.print(F("ARML2 "));Serial.print(roboSetup.data.arm1len);
    }else if(str[0]=='D'){
      roboSetup.data.speed = atoi(str+1);
      Serial.print(F("Speed "));Serial.print(roboSetup.data.speed);
    }else if(str[0]=='P'){
      roboSetup.data.servoPenDown = atoi(str+1);
      Serial.print(F("ServoPenDown "));Serial.print(roboSetup.data.servoPenDown);
    }else if(str[0]=='Q'){
      roboSetup.data.servoPenUp = atoi(str+1);
      Serial.print(F("ServoPenUp "));Serial.print(roboSetup.data.servoPenUp);
    }
  }
  syncRobotSetup();
  prepareMoveCalcs();               // setup some common variables
}


void parseMcode(char * cmd)
{
  int code;
  code = atoi(cmd);
  switch(code){
    case 1:
      parseServo(cmd);
      break;
    case 2:
    
      break;
    case 3:
      parseAuxDelay(cmd);
      break;
    case 4:
      parseLaserPower(cmd);
      break;
    case 5:
      parseRobotSetup(cmd);
      break;
    case 10: // echo robot config
      echoArmSetup(cmd);
      break;
  }

}


void parseCmd(char * cmd)
{
  if(cmd[0]=='G'){ // gcode
    parseGcode(cmd+1);  
  }else if(cmd[0]=='M'){ // mcode
    parseMcode(cmd+1);
  }else if(cmd[0]=='P'){
    Serial.print(F("POS X"));Serial.print(curX);Serial.print(F(" Y"));Serial.println(curY);
  }
  Serial.println("OK");
}

// local data
void initRobotSetup()
{
  int i;
  //Serial.println("read eeprom");
  for(i=0;i<64;i++){
    roboSetup.buf[i] = EEPROM.read(i);
    //Serial.print(roboSetup.buf[i],16);Serial.print(' ');
  }

  //Serial.println();
  if(strncmp(roboSetup.data.name,"SCARA3",6)!=0){
    Serial.println("set to default setup");
    // set to default setup
    memset(roboSetup.buf,0,64);
    memcpy(roboSetup.data.name,"SCARA3",6);
    roboSetup.data.motorADir = 0;
    roboSetup.data.motorBDir = 0;
    roboSetup.data.arm0len = ARML1;
    roboSetup.data.arm1len = ARML2;
    roboSetup.data.speed = 80;
	roboSetup.data.arm1LimitCCW       = arm1LimitCCWDefault;
	roboSetup.data.arm1LimitCW        = arm1LimitCWDefault;
	roboSetup.data.arm2LimitCCW       = arm2LimitCCWDefault;
	roboSetup.data.arm2LimitCW        = arm2LimitCWDefault;
	roboSetup.data.excludeBoxXLeft    = excludeBoxXLeftDefault;
	roboSetup.data.excludeBoxXRight   = excludeBoxXRightDefault;
	roboSetup.data.excludeBoxYTop     = excludeBoxYTopDefault;
	roboSetup.data.excludeBoxYBottom  = excludeBoxYBottomDefault;
	roboSetup.data.swStepRate         = softSteppingDefault;
	roboSetup.data.stepsPerRevolution = stepsPerRevolutionDefault;
	roboSetup.data.servoPenUp	      = servoPositionPenUpDefault;
	roboSetup.data.servoPenDown       = servoPositionPenDownDefault;
    syncRobotSetup();
  }
  // init pen position to up and laser power to off
  servoPositionCurrent     = roboSetup.data.servoPenUp;
  servoPositionLastCommand = roboSetup.data.servoPenUp; 
  laserPowerCurrent        = 0;
  laserPowerLastCommand    = 0;
  // init motor direction
  if(roboSetup.data.motorADir==0){
    motorAfw=1;motorAbk=-1;
  }else{
    motorAfw=-1;motorAbk=1;
  }
  if(roboSetup.data.motorBDir==0){
    motorBfw=1;motorBbk=-1;
  }else{
    motorBfw=-1;motorBbk=1;
  }
  int spd = 100 - roboSetup.data.speed;
  stepdelay_min = spd*10;
  stepdelay_max = spd*100;
  prepareMoveCalcs();            // setup some common variables
  //Serial.printf("spd %d %d\n",stepdelay_min,stepdelay_max);
}

void syncRobotSetup()
{
  int i;
  for(i=0;i<64;i++){
    EEPROM.write(i,roboSetup.buf[i]);
  }
}

/************** arduino ******************/
void setup() {
  Serial.begin(115200);
  initRobotSetup();
  servoPen.attach(servopin);
  servoPen.write(servoPositionCurrent);
  laser.run(laserPowerCurrent);
  initPosition();
}

char buf[64];
char bufindex;
char buf2[64];
char bufindex2;

/*
	Robbo1 8/June/2015
	
	Fixed potential probelms from buffer overflow and first cmd string not being null terminated
	
*/

void loop() {
  if(Serial.available()){
    char c = Serial.read();
    //buf[bufindex++]=c;                 // Robbo1 2015/6/8 Removed - Do not store the \n
    if(c=='\n'){
      buf[bufindex++]='\0';              // Robbo1 2015/6/8 Add     - Null terminate the string - Essential for first use of 'buf' and good programming practice
Serial.println(buf);
      parseCmd(buf);
      memset(buf,0,64);
      bufindex = 0;
    }else if(bufindex<64){               // Robbo1 2015/6/8 Add     - Only add char to string if the string can fit it and still be null terminated 
      buf[bufindex++]=c;                 // Robbo1 2015/6/8 Moved   - Store the character here now
	}
  }

}
