//------------------------------------------------------------------------------
// TEAM 1280 - SAN RAMON VALLEY HIGH SCHOOL RAGIN' C-BISCUITS
// 2016 STRONGHOLD ROBOT CODE
//------------------------------------------------------------------------------
#include "WPILib.h" // Instruction to preprocessor to include the WPI Library
                    // header file
#include <cmath>

#include "../H/ArmLower.h"
#include "../H/ArmUpper.h"
#include "../H/CameraLights.h"
#include "../H/Loader.h"
#include "../H/Shooter.h"
#include "../H/Climber.h"

#define CONSOLE

//------------------------------------------------------------------------------
// DEFINE StrongholdRobot CLASS
//------------------------------------------------------------------------------
// Derived from IterativeRobot class in WPILib.h.  Inherits attributes and
// functions of the base IterativeRobot class.
// Extends class through:
// - Overrides to virtual methods contained in the IterativeRobot base class
// - Methods specific to the 2016 Team 1280 Stronghold robot
// - Constants used by class
// - Pointers to required objects
// - Additional local variables required
//------------------------------------------------------------------------------
class StrongholdRobot : public IterativeRobot
{
	//--------------------------------------------------------------------------
	// DECLARATION OF PUBLIC METHODS
	//--------------------------------------------------------------------------
	public:
		// Constructor and destructor methods
		StrongholdRobot();
		~StrongholdRobot();

		//----------------------------------------------------------------------
		// OVERRIDES TO IterativeRobot BASE CLASS VIRTUAL METHODS
		//----------------------------------------------------------------------
		// Robot and state initialization methods
		void   RobotInit();
		void   DisabledInit();
		void   AutonomousInit();
		void   TeleopInit();

		// Robot periodic methods performed in loops
		void   DisabledPeriodic();
		void   AutonomousPeriodic();
		void   TeleopPeriodic();

	private:
		//----------------------------------------------------------------------
		// CONSTANTS USED IN CLASS
		//----------------------------------------------------------------------
		// DRIVER STATION PORTS AND CHANNELS
		//----------------------------------------------------------------------
		// Driver Station Joystick ports
		static const uint JS_PORT_LEFT           =  0;
		static const uint JS_PORT_RIGHT			 =  1;
		static const uint CCI_PORT1        	     =  2;  // eStop Robots CCI Inputs
//		static const uint CCI_PORT2				 =  3;  // eStop Robots CCI Inputs

		// Joystick Button (right joystick)
		static const uint CAMERA_LIGHT_SW_CH     =  1;

		// Driver Station CCI1 Channels (Uses joystick button references)
		static const uint LOWER_ARM_FWD_SW_CH    =  4;  // TEMP TO MOVE ARM
		static const uint LOWER_ARM_REV_SW_CH    =  3;  // TEMP TO MOVE ARM
		static const uint UPPER_ARM_FWD_SW_CH    =  5;  // TEMP TO MOVE ARM
		static const uint UPPER_ARM_REV_SW_CH    =  2;  // TEMP TO MOVE ARM

//		static const uint PORTCULLIS_EXEC_SW_CH  =  1;
//		static const uint PORTCULLIS_SETUP_SW_CH =  2;
//		static const uint DRAWBRIDGE_EXEC_SW_CH  =  3;
//		static const uint DRAWBRIDGE_SETUP_SW_CH =  4;
//		static const uint SALLY_PORT_EXEC_SW_CH  =  5;
//		static const uint SALLY_PORT_SETUP_SW_CH =  6;
//		static const uint CHEVAL_EXEC_SW_CH      =  7;
//		static const uint CHEVAL_SETUP_SW_CH     =  8;
//		static const uint ROBOT_LIFT_EXEC_SW_CH  =  9;
//		static const uint ROBOT_LIFT_SETUP_SW_CH =  8;
		static const uint LOAD_BALL_SW_CH		 = 10;
		static const uint EJECT_BALL_SW_CH       = 11;

		// Driver Station CCI2 Channels (Uses joystick button references)
		static const uint SHOOT_BALL_SW_CH       =  1;
//		static const uint AUTO_EXEC_SW_CH		 =  2;
		static const uint ARM_POT_SW_CH			 =  8;
//		static const uint ARM_TOP_SW_CH          =  4;
//		static const uint ARM_BOTTOM_SW_CH       =  5;
 		static const uint LOWER_SW_CH			 =  7;
		static const uint CLIMB_SW_CH    		 =  6;
//		static const uint UNUSED_SW_CH           =  8;
		static const uint SHOOTER_MOTOR_SW_CH    =  9;

		//----------------------------------------------------------------------
		// ROBOT CHANNELS - INPUTS AND OUTPUTS
		//----------------------------------------------------------------------
        // ROBOT INPUTS
		//----------------------------------------------------------------------
		// roboRio GPIO Channels
		static const uint AUTONOMOUS_SW_1_CH        =  0;
		static const uint AUTONOMOUS_SW_2_CH        =  1;
		static const uint AUTONOMOUS_SW_3_CH        =  2;
		static const uint SHOOTER_RESET_SENSOR_CH   =  6;
		static const uint BOTTOM_STOP_SENSOR_CH	    =  7;  // NORMAL CLOSE
		static const uint BALL_IN_SHOOTER_SENSOR_CH =  8;  // NORMAL CLOSE
		static const uint LOADER_SENSOR_CH          =  9;  // NORMAL CLOSE

		// roboRio Analog Channels
		static const uint ARM_LOWER_POT_CH 		    =  0;
		static const uint ARM_UPPER_POT_CH 		    =  1;

		// roboRio Relay Channels
		static const uint CAMERA_LIGHT_CH           =  0;

		//----------------------------------------------------------------------
        // ROBOT OUTPUTS
		//----------------------------------------------------------------------

		// roboRio PWM Channels (PWM = Pulsed width modulation)
		static const uint RIGHT_FRONT_MOTOR_CH	  =  0;
		static const uint RIGHT_REAR_MOTOR_CH     =  1;
		static const uint LEFT_FRONT_MOTOR_CH	  =  2;
		static const uint LEFT_REAR_MOTOR_CH	  =  3;
		static const uint CLIMBER_MOTOR1_CH		  =  4;
		static const uint CLIMBER_MOTOR2_CH		  =  5;
		static const uint SHOOTER_MOTOR_CH		  =  6;
		static const uint LOADER_MOTOR_CH		  =  7;

		static const uint ARM_LOWER_MOTOR_CH      =  8;
		static const uint ARM_UPPER_MOTOR_CH	  =  9;

		// roboRio Relay Channels

		// roboRio Solenoid Channels

		//----------------------------------------------------------------------
		// CONSTANTS USED TO DETERMINE STATUS OF DRIVER STATION AND ROBOT INPUTS
		// AND TO INSTRUCT ROBOT
		// Private static constants used in multiple methods
		//----------------------------------------------------------------------
		// CONSTANTS USED IN DECLARING OBJECTS
		//----------------------------------------------------------------------

		//----------------------------------------------------------------------
		// AUTONOMOUS MODE ROBOT CONTROL CONSTANTS (OUTPUTS)
		//----------------------------------------------------------------------
        // Autonomous mode timings (Starts and Durations)
		// For durations, 50 loops is approximately 1 second.
		static const uint  AM2_S1_START            = 100;  // CONFIGURE
		static const uint  AM2_S1_DRIVE_FWD        = 200;  // CONFIGURE

		static const uint  AM3_S1_START            =   0;  // CONFIGURE
		static const uint  AM3_S1_DRIVE_FWD        = 200;  // CONFIGURE

		static const uint  AM4_S1_START            = 100;  // CONFIGURE
		static const uint  AM4_S1_DRIVE_FWD        = 225;  // CONFIGURE
		static const uint  AM4_S2_TURN_AROUND      =  50;  // CONFIGURE
		static const uint  AM4_S3_DRIVE_FWD        = 225;  // CONFIGURE

		static const uint  AM5_S1_START            = 100;  // CONFIGURE
		static const uint  AM5_S1_DRIVE            = 225;  // CONFIGURE
		static const uint  AM5_S2_TURN_RT          = 100;  // CONFIGURE
		static const uint  AM5_S3_EJECT            = 150;  // CONFIGURE

		static const uint  AM6_S1_START            = 100;  // CONFIGURE
		static const uint  AM6_S1_DRIVE            = 210;  // CONFIGURE
		static const uint  AM6_S2_TURN_RT          =  30;  // CONFIGURE
		static const uint  AM6_S3_SHOOT            = 250;  // CONFIGURE

        // Robot Drive Speeds
		const float AM_DRIVE_FWD_RIGHT_FAST_SPEED =  0.900;  // CONFIGURE
		const float AM_DRIVE_FWD_LEFT_FAST_SPEED  =  0.850;  // CONFIGURE
		const float AM_TURN_RIGHT_RIGHT_SPEED     = -1.000;  // CONFIGURE
		const float AM_TURN_RIGHT_LEFT_SPEED      =  1.000;  // CONFIGURE
		const float AM_DRIVE_STOP                 =  0.000;

        //----------------------------------------------------------------------
		// AUTONOMOUS MODE ROBOT STATE & TIMING TRACKING
		// Used to determine what robot is or should be doing in autonomous mode
		//----------------------------------------------------------------------
        // Autonomous Mode States
        enum autoModeStates {kAM1Off, kAM2DriveLowBar, kAM3DriveOnly
        				   , kAM4TurnLowBar, kAM5LowBarShootLow
						   , kAM6LowBarShootHigh, kAM7LowBarShootLow2
        				   , kAM8LowBarShootHigh2};

		//----------------------------------------------------------------------
		// POINTERS FOR REQUIRED OBJECTS
		//----------------------------------------------------------------------
		// DRIVER STATION INPUT & OUTPUT POINTERS
		//----------------------------------------------------------------------
		// Includes driver station laptop, joysticks, switches and other digital
		// and analog devices connected through the eStop Robotics CCI.
		//----------------------------------------------------------------------
		Joystick		 *pDriveStickLeft;
		Joystick		 *pDriveStickRight;
		Joystick         *pCCI1;             // CCI
//		Joystick 		 *pCCI2;

		// Joystick Buttons - Right Joystick
		JoystickButton   *pCameraLightButton;

		// eStop Robotics Custom Control Interface (CCI) #1
		JoystickButton   *pLowerArmFwdSwitch;
		JoystickButton   *pLowerArmRevSwitch;
		JoystickButton   *pUpperArmFwdSwitch;
		JoystickButton   *pUpperArmRevSwitch;

//		JoystickButton   *pSallyPortSetupSwitch;
//		JoystickButton   *pSallyPortExecSwitch;
//		JoystickButton   *pPortcullisSetupSwitch;
//		JoystickButton   *pPortcullisExecSwitch;
//		JoystickButton   *pChevalSetupSwitch;
//		JoystickButton   *pChevalExecSwitch;
//		JoystickButton   *pDrawbridgeSetupSwitch;
//		JoystickButton   *pDrawbridgeExecSwitch;
//		JoystickButton   *pRobotLiftSetupSwitch;
//		JoystickButton   *pRobotLiftExecSwitch;
		JoystickButton   *pLoadBallSwitch;
		JoystickButton   *pEjectBallSwitch;

		// eStop Robotics Custom Control Interface (CCI) #2
        JoystickButton   *pShootBallSwitch;
//      JoystickButton   *pAutoExecSwitch;
        JoystickButton   *pArmPOTSwitch;
//		JoystickButton   *pArmTopSwitch;
//		JoystickButton   *pArmBottomSwitch;
        JoystickButton   *pShooterMotorSwitch;
        JoystickButton   *pClimberSwitch;
        JoystickButton   *pLowerSwitch;
 //     JoystickButton   *pUnusedSwitch;

		//----------------------------------------------------------------------
		// ROBOT INPUT & OUTPUT POINTERS
		//----------------------------------------------------------------------
		// Robot Digital Inputs - GPIO Inputs including Encoders
		//----------------------------------------------------------------------
		// Autonomous Mode Switches
	    DigitalInput    *pAutoSwitch1;
	    DigitalInput    *pAutoSwitch2;
	    DigitalInput    *pAutoSwitch3;
		//----------------------------------------------------------------------
		// Robot Digital Outputs - Relays (Spikes)
		//----------------------------------------------------------------------

		//----------------------------------------------------------------------
		// Robot Objects
		//----------------------------------------------------------------------
		Loader 			*pBallLoader;
		RobotDrive		*pDriveTrain;
		ArmLower		*pLowerArm;
		ArmUpper		*pUpperArm;
		Shooter         *pBallShooter;
		Climber			*pClimber;
		CameraLights    *pCameraLight;

		//----------------------------------------------------------------------
		// VARIABLES USED IN CLASS
		//----------------------------------------------------------------------
		// DRIVER STATION INPUTS
		// Analog inputs from joysticks and eStop Robotics CCI.
		//----------------------------------------------------------------------

		//----------------------------------------------------------------------
		// DRIVER STATION INPUTS - Digital Inputs from eStop Robotics CCI
		//----------------------------------------------------------------------

		//----------------------------------------------------------------------
		// CLASS VARIABLES USED TO TRACK ROBOT STATUS
		//----------------------------------------------------------------------
		// General status tracking
		//----------------------------------------------------------------------
		// Class variables to track look (packet) counts
		uint   loopCount;

		//----------------------------------------------------------------------
		// Arm Positioning
		//----------------------------------------------------------------------
		// Arm Controls
		bool   lowerArmInPosition;
		bool   upperArmInPosition;
		uint   armTarget;
		//----------------------------------------------------------------------
		// Loader Controls
		//----------------------------------------------------------------------
		bool   prevLoadBallSw;
		bool   loadBall;
		bool   ballLoaded;
		bool   prevEjectBallSw;
		bool   ejectBall;
		bool   ballEjected;
		//Drive Train
		float rightDriveSpeed;
		float leftDriveSpeed;
		//----------------------------------------------------------------------
		// Shooter
		//----------------------------------------------------------------------
		bool prevShootBallSw;
		bool shootBall;
		bool shooterReset;
		//----------------------------------------------------------------------
		// Camera Light
		//----------------------------------------------------------------------
		bool prevCameraLightButton;

		//----------------------------------------------------------------------
		// Autonomous Mode Switches & variables
		//----------------------------------------------------------------------
		uint autoMode;
		uint autoModeSelected    = 0;

		//----------------------------------------------------------------------
		// Autonomous Mode Timings
		//----------------------------------------------------------------------
		uint am2S1DriveFwdStart   =   0;
		uint am2S1DriveFwdEnd     =   0;

		uint am3S1DriveFwdStart   =   0;
		uint am3S1DriveFwdEnd     =   0;

		uint am4S1DriveFwdStart   =   0;
		uint am4S1DriveFwdEnd     =   0;
		uint am4S2TurnAroundStart =   0;
		uint am4S2TurnAroundEnd   =   0;
		uint am4S3DriveFwdStart   =   0;
		uint am4S3DriveFwdEnd     =   0;

		uint am5S1DriveStart      =   0;
		uint am5S1DriveEnd        =   0;
		uint am5S2TurnRtStart     =   0;
		uint am5S2TurnRtEnd       =   0;
		uint am5S3EjectStart      =   0;
		uint am5S3EjectEnd        =   0;

		uint am6S1DriveStart      =   0;
		uint am6S1DriveEnd        =   0;
		uint am6S2TurnRtStart     =   0;
		uint am6S2TurnRtEnd       =   0;
		uint am6S3ShootStart      =   0;
		uint am6S3ShootEnd        =   0;

		//----------------------------------------------------------------------
		// CUSTOM METHODS SPECIFIC TO TEAM 1280 ROBOT
		//----------------------------------------------------------------------
		// Initialization and reset methods

		// Driver station and robot input gathering methods
		void   GetDriverStationInput();
		void   ShowDSValues();
		void   GetRobotSensorInput();
		void   ShowRobotValues();

		// Robot output methods
		void   MoveArmToPosition();
		void   MoveArmUsingPOT();
		void   MoveArmUsingSwitchPosition();
		void   MoveArmUsingMotorSwitch();
		void   CheckBallLoader();
		void   CheckLoaderSwitches();
		void   ShootBall();
		void   CheckShootBallSwitch();
		void   RunClimber();
		void   CheckCameraLightSwitch();

		// Autonomous mode methods
		void   CalcAutoModeTimings();
		void   GetAutoModeSwitches();
		void   RunAutonomousMode();
		void   ShowAMStatus();
		void   AM1Off();
		void   AM2DriveLowBar();
		void   AM3DriveOnly();
		void   AM4LowBarTwice();
		void   AM5DriveShootLow();
		void   AM6DriveShootHigh();
		void   AMDriveFwd();
		void   AMDriveStop();
		void   AMTurnRight();
		void   AMShootLow();
		void   AMShootHigh();
};
//------------------------------------------------------------------------------
// INITIALIZE STATIC CONSTANTS
//------------------------------------------------------------------------------

START_ROBOT_CLASS(StrongholdRobot);

//------------------------------------------------------------------------------
// METHOD DEFINITONS
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::StrongholdRobot
// Type:	Public default constructor for StrongholdRobot class
//------------------------------------------------------------------------------
// Defines pointers to required robot objects and initializes packet count
// variable.
//------------------------------------------------------------------------------
StrongholdRobot::StrongholdRobot()
{
	//----------------------------------------------------------------------
	// DEFINE POINTERS TO REQUIRED ROBOT OBJECTS
	//----------------------------------------------------------------------
	// DRIVER STATION INPUTS
	//----------------------------------------------------------------------

	// Define joysticks & CCI
	pDriveStickLeft		   = new Joystick(JS_PORT_LEFT);
	pDriveStickRight	   = new Joystick(JS_PORT_RIGHT);
	pCCI1                  = new Joystick(CCI_PORT1); // CCI uses joystick object
//	pCCI2				   = new Joystick(CCI_PORT2);

	// Joystick Buttons (Right Joystick)
	pCameraLightButton     = new JoystickButton(pDriveStickRight,CAMERA_LIGHT_SW_CH);

	// CCI 1 Switches
	pLowerArmFwdSwitch     = new JoystickButton(pCCI1,LOWER_ARM_FWD_SW_CH);
	pLowerArmRevSwitch     = new JoystickButton(pCCI1,LOWER_ARM_REV_SW_CH);
	pUpperArmFwdSwitch     = new JoystickButton(pCCI1,UPPER_ARM_FWD_SW_CH);
	pUpperArmRevSwitch     = new JoystickButton(pCCI1,UPPER_ARM_REV_SW_CH);

//	pSallyPortSetupSwitch  = new JoystickButton(pCCI1,SALLY_PORT_SETUP_SW_CH);
//	pSallyPortExecSwitch   = new JoystickButton(pCCI1,SALLY_PORT_EXEC_SW_CH);
//	pPortcullisSetupSwitch = new JoystickButton(pCCI1,PORTCULLIS_SETUP_SW_CH);
//	pPortcullisExecSwitch  = new JoystickButton(pCCI1,PORTCULLIS_EXEC_SW_CH);
//	pChevalSetupSwitch     = new JoystickButton(pCCI1,CHEVAL_SETUP_SW_CH);
//	pChevalExecSwitch      = new JoystickButton(pCCI1,CHEVAL_EXEC_SW_CH);
//	pDrawbridgeSetupSwitch = new JoystickButton(pCCI1,DRAWBRIDGE_SETUP_SW_CH);
//	pDrawbridgeExecSwitch  = new JoystickButton(pCCI1,DRAWBRIDGE_EXEC_SW_CH);
//	pRobotLiftSetupSwitch  = new JoystickButton(pCCI1,ROBOT_LIFT_SETUP_SW_CH);
//	pRobotLiftExecSwitch   = new JoystickButton(pCCI1,ROBOT_LIFT_EXEC_SW_CH);
	pLoadBallSwitch 	   = new JoystickButton(pCCI1,LOAD_BALL_SW_CH);
	pEjectBallSwitch	   = new JoystickButton(pCCI1,EJECT_BALL_SW_CH);

	// CCI 2 Switches
	pShootBallSwitch 	   = new JoystickButton(pCCI1,SHOOT_BALL_SW_CH);
//	pAutoExecSwitch        = new JoystickButton(pCCI2,AUTO_EXEC_SW_CH);
	pArmPOTSwitch          = new JoystickButton(pCCI1,ARM_POT_SW_CH);
//	pArmTopSwitch          = new JoystickButton(pCCI2,ARM_TOP_SW_CH);
//	pArmBottomSwitch       = new JoystickButton(pCCI2,ARM_BOTTOM_SW_CH);
	pShooterMotorSwitch    = new JoystickButton(pCCI1,SHOOTER_MOTOR_SW_CH);
	pClimberSwitch         = new JoystickButton(pCCI1,CLIMB_SW_CH);
	pLowerSwitch 		   = new JoystickButton(pCCI1,LOWER_SW_CH);
//	pUnusedSwitch          = new JoystickButton(pCCI2,UNUSED_SW_CH);

	//----------------------------------------------------------------------
	// ROBOT INPUTS
	//----------------------------------------------------------------------
	// GPIO & Spare Power Inputs
	// - Autonomous Mode Switches
	pAutoSwitch1         = new DigitalInput(AUTONOMOUS_SW_1_CH);
	pAutoSwitch2         = new DigitalInput(AUTONOMOUS_SW_2_CH);
	pAutoSwitch3         = new DigitalInput(AUTONOMOUS_SW_3_CH);

	//----------------------------------------------------------------------
	// ROBOT CONTROLS (OUTPUTS)
	//----------------------------------------------------------------------
	pDriveTrain		     = new RobotDrive(LEFT_FRONT_MOTOR_CH,LEFT_REAR_MOTOR_CH,
										  RIGHT_FRONT_MOTOR_CH, RIGHT_REAR_MOTOR_CH);

	pLowerArm			 = new ArmLower(ARM_LOWER_MOTOR_CH, ARM_LOWER_POT_CH
			                          , BOTTOM_STOP_SENSOR_CH);
	pUpperArm			 = new ArmUpper(ARM_UPPER_MOTOR_CH, ARM_UPPER_POT_CH);
	
	pBallLoader			 = new Loader(LOADER_MOTOR_CH, LOADER_SENSOR_CH, BALL_IN_SHOOTER_SENSOR_CH);

	pBallShooter		 = new Shooter(SHOOTER_MOTOR_CH, SHOOTER_RESET_SENSOR_CH, pBallLoader);

	pClimber		     = new Climber(CLIMBER_MOTOR1_CH, CLIMBER_MOTOR2_CH);

	pCameraLight         = new CameraLights(CAMERA_LIGHT_CH);

	//----------------------------------------------------------------------
	// INITIALIZE VARIABLES
	//----------------------------------------------------------------------
	// Initialize loop counter
	loopCount      = 0;

	// Initialize robot control variables
	autoMode              = kAM1Off;
	lowerArmInPosition    = true;
	upperArmInPosition    = true;
	armTarget             = 0;
	prevLoadBallSw        = false;
	loadBall              = false;
	ballLoaded            = false;
	prevEjectBallSw       = false;
	ejectBall             = false;
	ballEjected           = true;

	prevShootBallSw		  = false;
	shootBall 			  = false;
	shooterReset		  = true;
	rightDriveSpeed       = 0.0;
	leftDriveSpeed        = 0.0;

	prevCameraLightButton = false;

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::~StrongholdRobot
// Type:	Public default destructor for StrongholdRobot class
//------------------------------------------------------------------------------
StrongholdRobot::~StrongholdRobot()
{
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::RobotInit()
// Type:	Performs robot initiation functions.  Overrides RobotInit() virtual
//          method contained in WPILib.
//
//			These actions are performed once and only once when the robot is
//	        powered on.
//------------------------------------------------------------------------------
// Functions:
// - Initializes the SmartDashboard
//------------------------------------------------------------------------------
void StrongholdRobot::RobotInit()
{
#ifdef CONSOLE
	SmartDashboard::init();
#endif

	prevLoadBallSw  = false;
	loadBall        = false;
	ballLoaded      = false;

	prevEjectBallSw = false;
	ejectBall       = false;
	ballEjected     = true;

	prevShootBallSw = false;
	shootBall   	= false;
	shooterReset 	= true;

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::DisabledInit()
// Type:	Executes when the robot is first placed in Disabled mode.  Overrides
//			DisabledInit() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Resets loop counter for disabled mode
//------------------------------------------------------------------------------
void StrongholdRobot::DisabledInit()
{
	// Reset loop counter
	loopCount  = 0;

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AutonomousInit()
// Type:	Executes when the robot is first placed in Autonomous mode.
//			Overrides AutonomousInit() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Resets the loop counter for autonomous mode
// - Determines which autonomous mode we want to use
// - Optionally displays the status of the autonomous mode switches for debugging
//   purposes
//--------------------------------------------------- ---------------------------
void StrongholdRobot::AutonomousInit()
{
	// Reset loop counter
	loopCount  = 0;

	CalcAutoModeTimings();

	GetAutoModeSwitches();
	GetRobotSensorInput();
	ShowAMStatus();

	pCameraLight->TurnOff();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::TeleopInit()
// Type:	Executes when the robot is first placed in Teleoperated mode.
//			Overrides TeleopInit() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Resets the loop counters for teleoperated mode
// - Sets the ator arm in position variable to true so arm doesn't move
//------------------------------------------------------------------------------
void StrongholdRobot::TeleopInit()
{
	// Loop count initialization
	loopCount      = 0;

	lowerArmInPosition = true;
	upperArmInPosition = true;

	shootBall = false;

	pCameraLight->TurnOn();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::DisabledPeriodic()
// Type:	Executes when the robot is in disabled mode.  Overrides the
//			DisabledPeriodic() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Increment the disabled loop counter
// - Optionally displays robot inputs (autonomous mode switches and sensors)
//------------------------------------------------------------------------------
void StrongholdRobot::DisabledPeriodic()
{
    // Increment loop counter
	loopCount++;

	GetAutoModeSwitches();
	GetRobotSensorInput();
	ShowAMStatus();

#ifdef CONSOLE
	ShowRobotValues();
#endif

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AutonomousPeriodic()
// Type:	Executes when the robot is in autonomous mode.  Overrides the
//			AutonomousPeriodic() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Increments the count of loops while in autonomous mode.
// - Gets robot sensor inputs
// - Runs autonomous mode
//------------------------------------------------------------------------------
void StrongholdRobot::AutonomousPeriodic()
{
    // Increment & display loop counter
	loopCount++;

	GetAutoModeSwitches();

	GetRobotSensorInput();
	
	ShowAMStatus();
	
	RunAutonomousMode();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::TeleopPeriodic()
// Type:	Executes when the robot is in teleoperated mode each time a new
//          packet of information has been received by the Driver Station.  Any
//          code which needs new information from the Driver Station should be
//          placed here.  This method overrides the TeleopPeriodic() virtual
//          method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Increments the count of loops processed and packets received while
//   in teleoperated mode.
// - Obtains input from the driver station (joystick inputs, switches,
//   potentiometers, etc.)
// - Obtains inputs from the robot (analog and digital sensors)
// - Moves the arm ator to the target position based on driver station
//   inputs
// - Sets the drive motor values based on joystick movement
//------------------------------------------------------------------------------
void StrongholdRobot::TeleopPeriodic()
{
	ShowAMStatus();

	// Increment & display loop counter
	loopCount++;

	// Get inputs from the driver station
	GetDriverStationInput();
	
	// Get robot sensor input
	GetRobotSensorInput();

	// Drive Robot using Tank Drive
	pDriveTrain->TankDrive(leftDriveSpeed,rightDriveSpeed);

	// Move arm to position defined by driver station switches
	MoveArmToPosition();

	// Determine if ball loader should run
    CheckBallLoader();

    // Determine if shooter should run
    ShootBall();

    // Determine if climber should run
    RunClimber();

    // Determine if camera lights should be turned on or off
    CheckCameraLightSwitch();

	return;
}

//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::GetDriverStationInput()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Obtains the input from the DriverStation required for teleoperated mode.
// Includes obtaining input for the following switches:
// - Optionally displays driver station values
//------------------------------------------------------------------------------
void StrongholdRobot::GetDriverStationInput()
{
	rightDriveSpeed		= -1 * pDriveStickRight->GetY();
	leftDriveSpeed		= -1 * pDriveStickLeft->GetY();

#ifdef CONSOLE
    ShowDSValues();
#endif

	return;
}
#ifdef CONSOLE
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::ShowDSValues()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Displays desired driver station values to the SmartDashboard for display
// purposes.
//------------------------------------------------------------------------------
void StrongholdRobot::ShowDSValues()
{
	// Show the values for driver station inputs
/*	SmartDashboard::PutBoolean("DS Sally Port Setup",pSallyPortSetupSwitch->Get());
	SmartDashboard::PutBoolean("DS Sally Port Exec",pSallyPortExecSwitch->Get());
	SmartDashboard::PutBoolean("DS Portcullis Setup",pPortcullisSetupSwitch->Get());
	SmartDashboard::PutBoolean("DS Portcullis Exec",pPortcullisExecSwitch->Get());
	SmartDashboard::PutBoolean("DS Cheval Setup",pChevalSetupSwitch->Get());
	SmartDashboard::PutBoolean("DS Cheval Exec",pChevalExecSwitch->Get());
	SmartDashboard::PutBoolean("DS Drawbridge Setup",pDrawbridgeSetupSwitch->Get());
	SmartDashboard::PutBoolean("DS Drawbridge Exec",pDrawbridgeExecSwitch->Get());
*/
	SmartDashboard::PutBoolean("DS Arm Use POT",pArmPOTSwitch->Get());
//	SmartDashboard::PutBoolean("DS Robot Lift Exec",pRobotLiftExecSwitch->Get());

	SmartDashboard::PutBoolean("DS Load Ball",pLoadBallSwitch->Get());
	SmartDashboard::PutBoolean("DS Eject Ball",pEjectBallSwitch->Get());
	SmartDashboard::PutBoolean("DS Shoot Ball",pShootBallSwitch->Get());

//	SmartDashboard::PutBoolean("DS Auto Setup Exec",pAutoExecSwitch->Get());

	SmartDashboard::PutBoolean("DS Shooter Motor On",pShooterMotorSwitch->Get());

	SmartDashboard::PutBoolean("DS Raise Robot",pClimberSwitch->Get());
	SmartDashboard::PutBoolean("DS Lower Robot",pLowerSwitch->Get());

//	SmartDashboard::PutBoolean("DS Arm Use POT",pArmPOTSwitch->Get());
//	SmartDashboard::PutBoolean("DS Arm Top Position",pArmTopSwitch->Get());
//	SmartDashboard::PutBoolean("DS Arm Bottom Position",pArmBottomSwitch->Get());

//	SmartDashboard::PutBoolean("DS Unused",pUnusedSwitch->Get());

	SmartDashboard::PutBoolean("DS Lower Arm Fwd",pLowerArmFwdSwitch->Get());
	SmartDashboard::PutBoolean("DS Lower Arm Rev",pLowerArmRevSwitch->Get());
	SmartDashboard::PutBoolean("DS Upper Arm Fwd",pUpperArmFwdSwitch->Get());
	SmartDashboard::PutBoolean("DS Upper Arm Rev",pUpperArmRevSwitch->Get());

	SmartDashboard::PutNumber("DS Upper Arm POT",pCCI1->GetX());
	SmartDashboard::PutNumber("DS Lower Arm POT",pCCI1->GetY());

	SmartDashboard::PutBoolean("DS Light Button",pCameraLightButton->Get());
/*
 	SmartDashboard::PutNumber("Left JoyStick",pDriveStickLeft->GetY());
 	SmartDashboard::PutNumber("Right JoyStick",pDriveStickRight->GetY());
 	SmartDashboard::PutBoolean("Prev Eject Ball Switch",prevEjectBallSw);
 	SmartDashboard::PutBoolean("Prev Shoot Ball Switch",prevShootBallSw);
*/
	return;
}
#endif
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::GetRobotSensorInput()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Looks at data in the packet and obtains input coming from the robot to be
// used in both Autonomous and Teleoperated Modes.
// - Distance traveled by right wheels
// - Distance traveled by left wheels
//------------------------------------------------------------------------------
void StrongholdRobot::GetRobotSensorInput()
{
#ifdef CONSOLE
	ShowRobotValues();
#endif
	
	return;
}
#ifdef CONSOLE
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::ShowRobotValues()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StrongholdRobot::ShowRobotValues()
{
//	SmartDashboard::PutNumber("AM Mode",autoMode);
	SmartDashboard::PutBoolean("R AM Switch 1",pAutoSwitch1->Get());
	SmartDashboard::PutBoolean("R AM Switch 2",pAutoSwitch2->Get());
	SmartDashboard::PutBoolean("R AM Switch 3",pAutoSwitch3->Get());

	SmartDashboard::PutBoolean("R Lower Arm Limit Sw",pLowerArm->GetStopSensor());

//	SmartDashboard::PutNumber("R Lower Arm Ratio",pLowerArm->GetRatio());
//	SmartDashboard::PutNumber("R Lower Arm Constant",pLowerArm->GetConstant());
	SmartDashboard::PutNumber("R Lower Arm Target POT Input",pLowerArm->GetTargetPOTInput());
	SmartDashboard::PutNumber("R Lower Arm Target Position",pLowerArm->GetTargetPosition());
	SmartDashboard::PutNumber("R Lower Arm Target POT Output",pLowerArm->GetTargetPOTOutput());
	SmartDashboard::PutNumber("R Lower Arm Current POT",pLowerArm->GetCurrentPosition());
// 	SmartDashboard::PutNumber("R Lower Arm Target Speed",pLowerArm->GetTargetMotorSpeed());
	SmartDashboard::PutNumber("R Lower Arm Motor Speed",pLowerArm->GetMotorSpeed());

//	SmartDashboard::PutNumber("R Upper Arm Ratio",pUpperArm->GetRatio());
//	SmartDashboard::PutNumber("R Upper Arm Constant",pUpperArm->GetConstant());
	SmartDashboard::PutNumber("R Upper Arm Target POT Input",pUpperArm->GetTargetPOTInput());
	SmartDashboard::PutNumber("R Upper Arm Target Position",pUpperArm->GetTargetPosition());
	SmartDashboard::PutNumber("R Upper Arm Target POT Output",pUpperArm->GetTargetPOTOutput());
	SmartDashboard::PutNumber("R Upper Arm Current POT",pUpperArm->GetCurrentPosition());
// 	SmartDashboard::PutNumber("R Upper Arm Target Speed",pUpperArm->GetTargetMotorSpeed());
	SmartDashboard::PutNumber("R Upper Arm Motor Speed",pUpperArm->GetMotorSpeed());

	SmartDashboard::PutBoolean("R Prev Camera Button",prevCameraLightButton);
	SmartDashboard::PutBoolean("R Camera Lights",pCameraLight->GetCameraStatus());

//	SmartDashboard::PutNumber("Loader Motor Speed",pBallLoader->GetMotorSpeed());
//	SmartDashboard::PutBoolean("Load Ball Mode",loadBall);
//	SmartDashboard::PutBoolean("Ball Loaded? Limit switch",pBallLoader->GetLoadedSensor());
//	SmartDashboard::PutBoolean("Ball Loaded? Loader code",pBallLoader->GetBallLoaded());
//	SmartDashboard::PutBoolean("Ball Loaded? Robot code",ballLoaded);

//	SmartDashboard::PutBoolean("Eject Ball Mode",ejectBall);
//	SmartDashboard::PutBoolean("First Eject Loop",pBallLoader->GetFirstEjectLoop());
//	SmartDashboard::PutNumber("Eject Counter",pBallLoader->GetEjectCounter());
//	SmartDashboard::PutBoolean("Ball Ejected? Loader code",pBallLoader->GetBallEjected());
//	SmartDashboard::PutBoolean("Ball Ejected? Robot code",ballEjected);

//	SmartDashboard::PutBoolean("Shoot Ball Mode",shootBall);
//	SmartDashboard::PutBoolean("Shooter 1st loop",pBallShooter->GetShooterFirstLoop());

//	SmartDashboard::PutBoolean("Ball in shooter? Limit Switch",
//			                    pBallLoader->GetBallInShooterSensor());
//	SmartDashboard::PutBoolean("Ball in shooter? Loader code flag",
//			                    pBallLoader->GetBallInShooterFlag());
//	SmartDashboard::PutBoolean("Ball in shooter? Shooter code flag",
//			                    pBallShooter->GetBallInShooter());

//	SmartDashboard::PutBoolean("Shooter Reset-Robot code",shooterReset);
//	SmartDashboard::PutBoolean("Shooter Reset-Shooter code",pBallShooter->GetShooterReset());
//	SmartDashboard::PutBoolean("Shooter Reset prev banner",pBallShooter->GetPrevShooterReset());
//	SmartDashboard::PutBoolean("Shooter Reset-Shooter banner sensor",
//			                    pBallShooter->GetBannerSensor());
//	SmartDashboard::PutNumber("Shooter Motor Speed",pBallShooter->GetMotorSpeed());

//	SmartDashboard::PutNumber("Climber Motor 1 Speed",pClimber->GetMotor1Speed());
//	SmartDashboard::PutNumber("Climber Motor 2 Speed",pClimber->GetMotor2Speed());

	return;
}
#endif
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::MoveArmToPosition()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Determines which position to move the arm Arm based on driver station
// switch values then moves to arm to the target position.
//------------------------------------------------------------------------------
void StrongholdRobot::MoveArmToPosition()
{
	if ( pArmPOTSwitch->Get() )
	{
		MoveArmUsingPOT();
	}
	else
	{
		if ( pLowerArmFwdSwitch->Get() || pLowerArmRevSwitch->Get() ||
		     pUpperArmFwdSwitch->Get() || pUpperArmRevSwitch->Get()    )
	    {
		    MoveArmUsingMotorSwitch();
	    }
	    else
	    {
		    pLowerArm->StopArm();
	     	pUpperArm->StopArm();
	    }
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::MoveArmUsingPOT()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Determines which position to move the arm Arm based on driver station
// switch values then moves to arm to the target position.
//------------------------------------------------------------------------------
void StrongholdRobot::MoveArmUsingPOT()
{
	pLowerArm->MoveArmPOTInput(pCCI1->GetY());
	pUpperArm->MoveArmPOTInput(pCCI1->GetX());

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::MoveArmUsingSwitchInput()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Determines which position to move the arm Arm based on driver station
// switch values then moves to arm to the target position.
//------------------------------------------------------------------------------
void StrongholdRobot::MoveArmUsingSwitchPosition()
{
/*	if ( pArmTopSwitch->Get() )
	{
		armTarget            = ArmLower::kTop;
	}
	else
	{
		if ( pArmBottomSwitch->Get() )
		{
			armTarget          = ArmLower::kBottom;
		}
		else
		{
			armTarget          = ArmLower::kMiddle;
		}
	}

	pLowerArm->MoveArmPositionInput(armTarget);

	pUpperArm->MoveArmPositionInput(armTarget);
*/
	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::MoveArmUsingMotorSwitch()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Determines which position to move the arm Arm based on driver station
// switch values then moves to arm to the target position.
//------------------------------------------------------------------------------
void StrongholdRobot::MoveArmUsingMotorSwitch()
{
	if ( pLowerArmFwdSwitch->Get() )
	{
		pLowerArm->MoveArmUp();
	}
	else
	{
		if ( pLowerArmRevSwitch->Get() )
		{
			pLowerArm->MoveArmDown();
		}
		else
		{
			pLowerArm->StopArm();
		}
	}

	if ( pUpperArmFwdSwitch->Get() )
	{
		pUpperArm->MoveArmUp();
	}
	else
	{
		if ( pUpperArmRevSwitch->Get() )
		{
			pUpperArm->MoveArmDown();
		}
		else
		{
			pUpperArm->StopArm();
		}
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::MoveToPosition()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Determines which position to move the arm ator based on driver station
// switch values then moves to arm to the target position.
//------------------------------------------------------------------------------
void StrongholdRobot::CheckBallLoader()
{

	CheckLoaderSwitches();

	if ( loadBall )
	{
		if ( !ballLoaded )
		{
			ballLoaded = pBallLoader->LoadBall();
		}
		else
		{
			loadBall    = false;
			ballEjected = false;
		}
	}

	if ( ejectBall )
	{
		if ( !ballEjected )
		{
			ballEjected = pBallLoader->EjectBall();
		}
		else
		{
			ballLoaded = false;
			ejectBall  = false;
		}
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::CheckLoaderSwitches()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Determines which position to move the arm ator based on driver station
// switch values then moves to arm to the target position.
//------------------------------------------------------------------------------
void StrongholdRobot::CheckLoaderSwitches()
{

	if ( ! prevLoadBallSw           &&
		   pLoadBallSwitch->Get() )
	{
		loadBall  = true;
		ejectBall = false;
	}

	prevLoadBallSw = pLoadBallSwitch->Get();

	if ( ! prevEjectBallSw          &&
		   pEjectBallSwitch->Get() )
	{
		ejectBall = true;
		loadBall  = false;
		ballEjected = false;
	}

	prevEjectBallSw = pEjectBallSwitch->Get();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::ShootBall()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StrongholdRobot::ShootBall()
{
	CheckShootBallSwitch();

	if ( shootBall )
	{
		shooterReset = pBallShooter->ShootBall();

		if ( shooterReset )
		{
			shootBall   = false;
			ballLoaded  = false;
			ballEjected = true;
		}
	}
	else
	{
		if ( pShooterMotorSwitch->Get() )
		{
			pBallShooter->RunShooter();
		}
		else
		{
			pBallShooter->StopShooter();
		}
	}

	return;
}

//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::RunClimber()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StrongholdRobot::CheckShootBallSwitch()
{
	if ( ! prevShootBallSw           &&
		   pShootBallSwitch->Get()       )
	{
		shootBall  = true;
	}

	prevShootBallSw = pShootBallSwitch->Get();

	return;
}


//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::RunClimber()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StrongholdRobot::RunClimber()
{
	if ( pClimberSwitch->Get() )
	{
		pClimber->Climb();
	}

	else
	{
		if ( pLowerSwitch->Get() )
		{
			pClimber->Lower();
		}
		else
		{
			pClimber->StopClimber();
		}
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::CheckCameraLightSwitch()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StrongholdRobot::CheckCameraLightSwitch()
{
	if ( !prevCameraLightButton      &&
		  pCameraLightButton->Get()     )
	{
		if ( pCameraLight->GetCameraStatus() )
		{
			pCameraLight->TurnOff();
		}
		else
		{
			pCameraLight->TurnOn();
		}
	}

	prevCameraLightButton = pCameraLightButton->Get();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::CalcAutoModeTimings()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Obtains the autonomous mode switch values from the robot and determines
// which autonomous mode we want to run.
//------------------------------------------------------------------------------
void StrongholdRobot::CalcAutoModeTimings()
{
    am2S1DriveFwdStart    =  AM2_S1_START;
    am2S1DriveFwdEnd      =  am2S1DriveFwdStart   + AM2_S1_DRIVE_FWD;

    am3S1DriveFwdStart    =  AM3_S1_START;
    am3S1DriveFwdEnd      =  am3S1DriveFwdStart   + AM3_S1_DRIVE_FWD;

    am4S1DriveFwdStart    =  AM4_S1_START;
    am4S1DriveFwdEnd      =  am4S1DriveFwdStart   + AM4_S1_DRIVE_FWD;
    am4S2TurnAroundStart  =  am4S1DriveFwdEnd;
    am4S2TurnAroundEnd    =  am4S2TurnAroundStart + AM4_S2_TURN_AROUND;
    am4S3DriveFwdStart    =  am4S2TurnAroundEnd;
    am4S3DriveFwdEnd      =  am4S3DriveFwdStart   + AM4_S3_DRIVE_FWD;

    am5S1DriveStart       =  AM5_S1_START;
    am5S1DriveEnd         =  am5S1DriveStart      + AM5_S1_DRIVE;
    am5S2TurnRtStart      =  am5S1DriveEnd;
    am5S2TurnRtEnd        =  am5S2TurnRtStart     + AM5_S2_TURN_RT;
    am5S3EjectStart       =  am5S2TurnRtEnd;
    am5S3EjectEnd         =  am5S3EjectStart      + AM5_S3_EJECT;

    am6S1DriveStart       =  AM6_S1_START;
    am6S1DriveEnd         =  am6S1DriveStart      + AM6_S1_DRIVE;
    am6S2TurnRtStart      =  am6S1DriveEnd;
    am6S2TurnRtEnd        =  am6S2TurnRtStart     + AM6_S2_TURN_RT;
    am6S3ShootStart       =  am6S2TurnRtEnd;
    am6S3ShootEnd         =  am6S3ShootStart      + AM6_S3_SHOOT;

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::GetAutoModeSwitches()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Obtains the autonomous mode switch values from the robot and determines
// which autonomous mode we want to run.
//------------------------------------------------------------------------------
void StrongholdRobot::GetAutoModeSwitches()
{
	if ( pAutoSwitch1->Get() )
	{
		if ( pAutoSwitch2->Get() )
		{
			if ( pAutoSwitch3->Get() )
			{
				// Autonomous Mode 8 - On On On
				autoMode         = kAM8LowBarShootHigh2;
				autoModeSelected = 8;
			}
			else
			{
				// Autonomous Mode 6 - On On Off
				autoMode = kAM6LowBarShootHigh;
				autoModeSelected = 6;
			}
		}
		else
		{
			if ( pAutoSwitch3->Get() )
			{
				// Autonomous Mode 7 - On Off On
				autoMode = kAM7LowBarShootLow2;
				autoModeSelected  = 7;
			}
			else
			{
				// Autonomous Mode 5 - On Off Off
				autoMode = kAM5LowBarShootLow;
				autoModeSelected  = 5;
			}
		}
	}
	else
	{
		if ( pAutoSwitch2->Get() )
		{
			if ( pAutoSwitch3->Get() )
			{
				// Autonomous Mode 4 - Off On On
				autoMode = kAM4TurnLowBar;
				autoModeSelected = 4;
			}
			else
			{
				// Autonomous Mode 2 - Off On Off
				autoMode = kAM2DriveLowBar;
				autoModeSelected  = 2;
			}
		}
		else
		{
			if ( pAutoSwitch3->Get() )
			{
				// Autonomous Mode 3 - Off Off On
				autoMode = kAM3DriveOnly;
				autoModeSelected = 3;
			}
			else
			{
				// Autonomous Mode 1 - Off Off Off
				autoMode = kAM1Off;
				autoModeSelected = 1;
			}
		}
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::RunAutonomousMode()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Executes autonomous mode functions.
//------------------------------------------------------------------------------
void StrongholdRobot::RunAutonomousMode()
{
	switch ( autoMode )
	{
		case kAM1Off:
			AM1Off();
			break;

		case kAM2DriveLowBar:
			AM2DriveLowBar();
			break;

		case kAM3DriveOnly:
			 AM3DriveOnly();
			 break;

		case kAM4TurnLowBar:
			 AM4LowBarTwice();
			 break;

		case kAM5LowBarShootLow:
			 AM5DriveShootLow();
			 break;

		case kAM6LowBarShootHigh:
			 AM6DriveShootHigh();
			 break;

		case kAM7LowBarShootLow2:
			 break;

		case kAM8LowBarShootHigh2:
			 break;

		default:
		 	break;
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::ShowAMStatus()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Displays information about autonomous mode for debugging purposes.
//------------------------------------------------------------------------------
void StrongholdRobot::ShowAMStatus()
{
	SmartDashboard::PutNumber("Autonomous Mode",autoModeSelected);

	SmartDashboard::PutNumber("Loop Counter",loopCount);
/*	SmartDashboard::PutNumber("AM3S1 Start",am3S1DriveSlowStart);
	SmartDashboard::PutNumber("AM3S1 End",am3S1DriveSlowEnd);
	SmartDashboard::PutNumber("AM3S2 Start",am3S2DriveFwdStart);
	SmartDashboard::PutNumber("AM3S2 End",am3S2DriveFwdEnd);
	SmartDashboard::PutNumber("AM3S3 Start",am3S3DriveSlowStart);
	SmartDashboard::PutNumber("AM3S3 End",am3S3DriveSlowEnd);
*/
	SmartDashboard::PutBoolean("R AM Switch 1",pAutoSwitch1->Get());
	SmartDashboard::PutBoolean("R AM Switch 2",pAutoSwitch2->Get());
	SmartDashboard::PutBoolean("R AM Switch 3",pAutoSwitch3->Get());

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AM1Off()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Does nothing when autonomous mode is turned off
//------------------------------------------------------------------------------
void StrongholdRobot::AM1Off()
{
	// Do nothing
	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AM2DriveLowBar()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Lowers arm and drives robot through low bar defense
//------------------------------------------------------------------------------
void StrongholdRobot::AM2DriveLowBar()
{
	pLowerArm->MoveArmPositionInput(ArmLower::kBottom);

	if ( loopCount >= am2S1DriveFwdStart  &&
		 loopCount <  am2S1DriveFwdEnd        )
	{
		AMDriveFwd();
	}

	if ( loopCount >= am2S1DriveFwdEnd       )
	{
		AMDriveStop();
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AM3DriveOnly()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Does nothing when autonomous mode is turned off
//------------------------------------------------------------------------------
void StrongholdRobot::AM3DriveOnly()
{
	if ( loopCount >= am3S1DriveFwdStart  &&
		 loopCount <  am3S1DriveFwdEnd        )
	{
		AMDriveFwd();
	}

	if ( loopCount >= am3S1DriveFwdEnd       )
	{
		AMDriveStop();
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AM4LowBarTwice()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Does nothing when autonomous mode is turned off
//------------------------------------------------------------------------------
void StrongholdRobot::AM4LowBarTwice()
{
	pLowerArm->MoveArmPositionInput(ArmLower::kBottom);

	if ( loopCount >= am4S1DriveFwdStart  &&
		 loopCount <  am4S1DriveFwdEnd        )
	{
		AMDriveFwd();
	}

	if ( loopCount >= am4S2TurnAroundStart &&
		 loopCount <  am4S2TurnAroundEnd      )
	{
		AMTurnRight();
	}

	if ( loopCount >= am4S3DriveFwdStart  &&
		 loopCount <  am4S3DriveFwdEnd        )
	{
		AMDriveFwd();
	}

	if ( loopCount >= am4S3DriveFwdEnd        )
	{
		AMDriveStop();
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AM5DriveShootLow()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Lowers the arm, drives the robot forward, turns right and shoots the ball.
//------------------------------------------------------------------------------
void StrongholdRobot::AM5DriveShootLow()
{
	pLowerArm->MoveArmPositionInput(ArmLower::kBottom);

	if ( loopCount >= am5S1DriveStart  &&
		 loopCount <  am5S1DriveEnd        )
	{
		AMDriveFwd();
	}

	if ( loopCount >= am5S2TurnRtStart &&
		 loopCount <  am5S2TurnRtEnd      )
	{
		AMTurnRight();
	}

	if ( loopCount >= am5S3EjectStart   &&
		 loopCount <  am5S3EjectEnd       )
	{
		AMShootLow();
	}

	if ( loopCount >= am5S3EjectEnd       )
	{
		AMDriveStop();
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AM6DriveShootHigh()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Lowers the arm, drives the robot forward, turns right and shoots the ball.
//------------------------------------------------------------------------------
void StrongholdRobot::AM6DriveShootHigh()
{
	pLowerArm->MoveArmPositionInput(ArmLower::kBottom);

	if ( loopCount >= am6S1DriveStart  &&
		 loopCount <  am6S1DriveEnd        )
	{
		AMDriveFwd();
	}

	if ( loopCount >= am6S2TurnRtStart &&
		 loopCount <  am6S2TurnRtEnd      )
	{
		AMTurnRight();
	}

	if ( loopCount >= am6S3ShootStart   &&
		 loopCount <  am6S3ShootEnd       )
	{
		AMShootHigh();
	}

	if ( loopCount >= am6S3ShootEnd       )
	{
		AMDriveStop();
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AMDriveFwd()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Does nothing when autonomous mode is turned off
//------------------------------------------------------------------------------
void StrongholdRobot::AMDriveFwd()
{
	pDriveTrain->TankDrive(AM_DRIVE_FWD_LEFT_FAST_SPEED,AM_DRIVE_FWD_RIGHT_FAST_SPEED);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AMDriveStop()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Stops the robot drive
//------------------------------------------------------------------------------
void StrongholdRobot::AMDriveStop()
{
	pDriveTrain->TankDrive(AM_DRIVE_STOP,AM_DRIVE_STOP);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AMTurnRight()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Turns the robot to the right
//------------------------------------------------------------------------------
void StrongholdRobot::AMTurnRight()
{
	pDriveTrain->TankDrive(AM_TURN_RIGHT_LEFT_SPEED,AM_TURN_RIGHT_RIGHT_SPEED);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AMShootLow()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Ejects ball in low target
//------------------------------------------------------------------------------
void StrongholdRobot::AMShootLow()
{
	pBallLoader->EjectBall();

	ballLoaded  = false;
	ballEjected = true;

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::AMShootHigh()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Shoots at the high target
//------------------------------------------------------------------------------
void StrongholdRobot::AMShootHigh()
{
	if ( loopCount == am6S3ShootStart )
	{
		shootBall   = true;
		ballLoaded  = true;
		ballEjected = false;
	}

	if ( shootBall )
	{
		shooterReset = pBallShooter->ShootBall();

		if ( shooterReset )
		{
			shootBall   = false;
			ballLoaded  = false;
			ballEjected = true;
		}
	}

	return;
}
