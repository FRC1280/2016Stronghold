//------------------------------------------------------------------------------
// TEAM 1280 - SAN RAMON VALLEY HIGH SCHOOL RAGIN' C-BISCUITS
// 2016 STRONGHOLD ROBOT CODE
//------------------------------------------------------------------------------
#include "WPILib.h" // Instruction to preprocessor to include the WPI Library
                    // header file
#include <cmath>

#include "../H/Arm.h"
#include "../H/CameraLights.h"
#include "../H/Loader.h"
#include "../H/Shooter.h"
#include "../H/Climber.h"

#define CONSOLE
//#define VISION

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
		static const uint CCI_PORT2				 =  3;

		// Driver Station CCI1 Channels (Uses joystick button references)
		static const uint LOAD_BALL_SW_CH		 = 11;
		static const uint EJECT_BALL_SW_CH       = 12;

		// Driver Station CCI2 Channels (Uses joystick button references)
		static const uint AUTO_EXEC_SW_CH		 =  2;
		static const uint ARM_POT_SW_CH			 =  3;
		static const uint ARM_TOP_SW_CH          =  4;
		static const uint ARM_BOTTOM_SW_CH       =  5;
//		static const uint CAMERA_LIGHTS_SW_CH    =  4;
		static const uint LOWER_SW_CH			 =  6;
		static const uint CLIMB_SW_CH    		 =  7;
		static const uint SHOOT_BALL_SW_CH       =  8;
		static const uint SHOOTER_MOTOR_SW_CH    =  9;

		static const uint CH01                   =  1;
		static const uint CH02                   =  2;
		static const uint CH03                   =  3;
		static const uint CH04                   =  4;
		static const uint CH05                   =  5;
		static const uint CH06                   =  6;
		static const uint CH07                   =  7;
		static const uint CH08                   =  8;
		static const uint CH09                   =  9;
		static const uint CH10                   =  10;
		static const uint CH11                   =  11;
		static const uint CH12                   =  12;

		//----------------------------------------------------------------------
		// ROBOT CHANNELS - INPUTS AND OUTPUTS
		//----------------------------------------------------------------------
        // ROBOT INPUTS
		//----------------------------------------------------------------------

		// roboRio GPIO Channels
		static const uint TOP_LIMIT_SW_CH		    =  0;
		static const uint BOTTOM_LIMIT_SW_CH	    =  1;
		static const uint LOADER_SENSOR_CH          =  9;
		static const uint BALL_IN_SHOOTER_SENSOR_CH =  8;
		static const uint SHOOTER_RESET_SENSOR_CH   =  6;

		// roboRio Analog Channels
		static const uint ARM_LOWER_POT_CH 		    =  0;
		static const uint ARM_UPPER_POT_CH 		    =  1;

		// navX MXP Inertial Measurement Unit (IMU) Constants
		static const uint8_t IMU_UPDATE_RATE        = 50;

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

		static const uint ARM_MOTOR_CH		      =  8;
		static const uint ARM_LOWER_MOTOR_CH      =  8;
		static const uint ARM_UPPER_MOTOR_CH	  =  9;

		// roboRio Relay Channels
//		static const uint CAMERA_LIGHTS_CH        =  2;

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
        // Robot drive variables

        // Robot Set Drive Speeds

        //----------------------------------------------------------------------
		// AUTONOMOUS MODE ROBOT STATE & TIMING TRACKING
		// Used to determine what robot is or should be doing in autonomous mode
		//----------------------------------------------------------------------
        // Autonomous Mode States
        enum autoModeStates {kAutoModeOff};

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
		Joystick 		 *pCCI2;

		// eStop Robotics Custom Control Interface (CCI)
		JoystickButton   *pArmTopSwitch;
		JoystickButton   *pArmBottomSwitch;
//		JoystickButton 	 *pCameraLightSwitch;
		JoystickButton   *pLoadBallSwitch;
		JoystickButton   *pEjectBallSwitch;
        JoystickButton   *pShootBallSwitch;
        JoystickButton   *pShooterMotorSwitch;
        JoystickButton   *pClimberSwitch;
        JoystickButton   *pLowerSwitch;

        JoystickButton   *pCCI1Ch01;
        JoystickButton   *pCCI1Ch02;
        JoystickButton   *pCCI1Ch03;
        JoystickButton   *pCCI1Ch04;
        JoystickButton   *pCCI1Ch05;
        JoystickButton   *pCCI1Ch06;
        JoystickButton   *pCCI1Ch07;
        JoystickButton   *pCCI1Ch08;
        JoystickButton   *pCCI1Ch09;
        JoystickButton   *pCCI1Ch10;
        JoystickButton   *pCCI1Ch11;
        JoystickButton   *pCCI1Ch12;

		//----------------------------------------------------------------------
		// ROBOT INPUT & OUTPUT POINTERS
		//----------------------------------------------------------------------
		// navX MXP Inertial Measurement Unit (IMU)
		SerialPort       *pIMUPort;

		//----------------------------------------------------------------------
		// Robot Digital Inputs - GPIO Inputs including Encoders
		//----------------------------------------------------------------------
		// Autonomous Mode Switches
	
		//----------------------------------------------------------------------
		// Robot Digital Outputs - Relays (Spikes)
		//----------------------------------------------------------------------
//		CameraLights	*pCameraLights;			// Camera LED lights

		//----------------------------------------------------------------------
		// Robot Objects
		//----------------------------------------------------------------------
		Loader 			*pBallLoader;
		RobotDrive		*pDriveTrain;
		Arm 			*pArm;
		Shooter         *pBallShooter;
		Climber			*pClimber;

#ifdef VISION
		JoystickButton   *pBottomSwitch;
Vision			*pVision;
#endif
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
		bool   armInPosition;
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
		// Camera Image Processing
		//----------------------------------------------------------------------
		// Camera Switches
//		bool   lightsOn;

		//----------------------------------------------------------------------
		// ROBOT INPUTS
		//----------------------------------------------------------------------
		// Autonomous Mode Switches & variables
		//----------------------------------------------------------------------
		uint autoMode;
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
		void   MoveToPosition();
		void   CheckBallLoader();
		void   CheckLoaderSwitches();
		void   ShootBall();
		void   CheckShootBallSwitch();
		void   RunClimber();

		// Autonomous mode methods
		void   CalcAutoModeTimings();
		void   GetAutoModeSwitches();
		void   RunAutonomousMode();
		void   ShowAMStatus();
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
	pDriveStickLeft		  = new Joystick(JS_PORT_LEFT);
	pDriveStickRight	  = new Joystick(JS_PORT_RIGHT);
	pCCI1                 = new Joystick(CCI_PORT1); // CCI uses joystick object
	pCCI2				  = new Joystick(CCI_PORT2);

	// CCI1 Switches
	pLoadBallSwitch 	  = new JoystickButton(pCCI1,LOAD_BALL_SW_CH);
	pEjectBallSwitch	  = new JoystickButton(pCCI1,EJECT_BALL_SW_CH);
	pShootBallSwitch 	  = new JoystickButton(pCCI2,SHOOT_BALL_SW_CH);

	// CCI2 Switches
	pArmTopSwitch         = new JoystickButton(pCCI2,ARM_TOP_SW_CH);
	pArmBottomSwitch      = new JoystickButton(pCCI2,ARM_BOTTOM_SW_CH);
//	pCameraLightSwitch    = new JoystickButton(pCCI1,CAMERA_LIGHTS_SW_CH);
	pShooterMotorSwitch   = new JoystickButton(pCCI2,SHOOTER_MOTOR_SW_CH);
	pClimberSwitch        = new JoystickButton(pCCI2,CLIMB_SW_CH);
	pLowerSwitch 		  = new JoystickButton(pCCI2,LOWER_SW_CH);

	pCCI1Ch01   		  = new JoystickButton(pCCI1,CH01);
	pCCI1Ch02   		  = new JoystickButton(pCCI1,CH02);
	pCCI1Ch03   		  = new JoystickButton(pCCI1,CH03);
	pCCI1Ch04   		  = new JoystickButton(pCCI1,CH04);
	pCCI1Ch05   		  = new JoystickButton(pCCI1,CH05);
	pCCI1Ch06   		  = new JoystickButton(pCCI1,CH06);
	pCCI1Ch07   		  = new JoystickButton(pCCI1,CH07);
	pCCI1Ch08   		  = new JoystickButton(pCCI1,CH08);
	pCCI1Ch09   		  = new JoystickButton(pCCI1,CH09);
	pCCI1Ch10   		  = new JoystickButton(pCCI1,CH10);
	pCCI1Ch11   		  = new JoystickButton(pCCI1,CH11);
	pCCI1Ch12   		  = new JoystickButton(pCCI1,CH12);

	//----------------------------------------------------------------------
	// ROBOT INPUTS
	//----------------------------------------------------------------------
	// GPIO & Spare Power Inputs
	// - Autonomous Mode Switches
	
	// navX MXP Intertial Measurement Unit (IMU)
	pIMUPort             = new SerialPort(57600,SerialPort::kMXP);

	//----------------------------------------------------------------------
	// ROBOT CONTROLS (OUTPUTS)
	//----------------------------------------------------------------------
	// Spike Relays (Relay Connections)
	// lights
//	pCameraLights		 = new CameraLights(CAMERA_LIGHTS_CH);

	// Drive Train
	pDriveTrain		     = new RobotDrive(LEFT_FRONT_MOTOR_CH,LEFT_REAR_MOTOR_CH,
										  RIGHT_FRONT_MOTOR_CH, RIGHT_REAR_MOTOR_CH);
	pArm			     = new Arm (ARM_MOTOR_CH, ARM_LOWER_POT_CH, TOP_LIMIT_SW_CH,
										  BOTTOM_LIMIT_SW_CH);
	
	//Ball Loader
	pBallLoader			 = new Loader(LOADER_MOTOR_CH, LOADER_SENSOR_CH, BALL_IN_SHOOTER_SENSOR_CH);

	pBallShooter		 = new Shooter(SHOOTER_MOTOR_CH, SHOOTER_RESET_SENSOR_CH, pBallLoader);

	//Climber
	pClimber		     = new Climber(CLIMBER_MOTOR1_CH, CLIMBER_MOTOR2_CH);

	//----------------------------------------------------------------------
	// INITIALIZE VARIABLES
	//----------------------------------------------------------------------
	// Initialize loop counter
	loopCount      = 0;

	// Initialize robot control variables
	autoMode              = kAutoModeOff;
	armInPosition         = true;
	armTarget             = 0;
	prevLoadBallSw        = false;
	loadBall              = false;
	ballLoaded            = false;
	prevEjectBallSw       = false;
	ejectBall             = false;
	ballEjected           = true;

	prevShootBallSw		  = true;
	shootBall 			  = false;
	shooterReset		  = true;
	rightDriveSpeed       = 0.0;
	leftDriveSpeed        = 0.0;
//	lightsOn              = false;  // CONFIG

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

	prevShootBallSw = true;
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
// - Sets camera lights to desired setting for autonomous mode
// - Determines which autonomous mode we want to use
// - Optionally displays the status of the autonomous mode switches for debugging
//   purposes
//--------------------------------------------------- ---------------------------
void StrongholdRobot::AutonomousInit()
{
	// Reset loop counter
	loopCount  = 0;

	// Set Robot Components to Default Starting Positions
//	pCameraLights->TurnOff();                  // CONFIG

	GetAutoModeSwitches();
	GetRobotSensorInput();
	ShowAMStatus();

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
#ifdef VISION
	pVision = new Vision;
#endif

	// Loop count initialization
	loopCount      = 0;

	armInPosition = true;

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
// - Turns camera lights on/off
// - Sets the drive motor values based on joystick movement
//------------------------------------------------------------------------------
void StrongholdRobot::TeleopPeriodic()
{
	// Increment & display loop counter
	loopCount++;

#ifdef VISION
	pVision->processImage();
#endif

	// Get inputs from the driver station
	GetDriverStationInput();
	
	// Get robot sensor input
	GetRobotSensorInput();

	// Drive Robot using Tank Drive
	pDriveTrain->TankDrive(rightDriveSpeed,leftDriveSpeed);

	// Move arm to position defined by driver station switches
	MoveToPosition();

	// Determine if ball loader should run
    CheckBallLoader();

    // Determine if shooter should run
    ShootBall();

    // Determine if climber should run
    RunClimber();

	// Turn camera LED lights on or off based on driver station input
/*	if ( lightsOn )
		pCameraLights->TurnOn();
	else
		pCameraLights->TurnOff();
*/
	return;
}

//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::GetDriverStationInput()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Obtains the input from the DriverStation required for teleoperated mode.
// Includes obtaining input for the following switches:
// - Camera lights switch
// - Optionally displays driver station values
//------------------------------------------------------------------------------
void StrongholdRobot::GetDriverStationInput()
{
	// Obtain the position of switches on the driver station
	// Camera Switches
//    lightsOn  				 = pCameraLightSwitch->Get();

	rightDriveSpeed		= pDriveStickRight->GetY();
	leftDriveSpeed		= pDriveStickLeft->GetY();
	rightDriveSpeed		= rightDriveSpeed * -1;
	leftDriveSpeed		= leftDriveSpeed * -1;

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
//	SmartDashboard::PutBoolean("Top Switch",pTopSwitch->Get());
//	SmartDashboard::PutBoolean("Bottom Switch",pBottomSwitch->Get());
//	SmartDashboard::PutBoolean("Camera Lights Switch",lightsOn);

//	SmartDashboard::PutNumber("Left JoyStick",pDriveStickLeft->GetY());
//	SmartDashboard::PutNumber("Right JoyStick",pDriveStickRight->GetY());
 	SmartDashboard::PutBoolean("Load Ball Switch",pLoadBallSwitch->Get());
 	SmartDashboard::PutBoolean("Eject Ball Switch",pEjectBallSwitch->Get());
	SmartDashboard::PutBoolean("Prev Eject Ball Switch",prevEjectBallSw);
	SmartDashboard::PutBoolean("Prev Shoot Ball Switch",prevShootBallSw);
	SmartDashboard::PutBoolean("Shoot Ball Switch",pShootBallSwitch->Get());
	SmartDashboard::PutBoolean("Shooter Motor Switch",pShooterMotorSwitch->Get());
//	SmartDashboard::PutBoolean("Climb Switch",pClimberSwitch->Get());
//	SmartDashboard::PutBoolean("Lower Switch",pLowerSwitch->Get());

/*	SmartDashboard::PutBoolean("CCI1 CH01",pCCI1Ch01->Get());
	SmartDashboard::PutBoolean("CCI1 CH02",pCCI1Ch02->Get());
	SmartDashboard::PutBoolean("CCI1 CH03",pCCI1Ch03->Get());
	SmartDashboard::PutBoolean("CCI1 CH04",pCCI1Ch04->Get());
	SmartDashboard::PutBoolean("CCI1 CH05",pCCI1Ch05->Get());
	SmartDashboard::PutBoolean("CCI1 CH06",pCCI1Ch06->Get());
	SmartDashboard::PutBoolean("CCI1 CH07",pCCI1Ch07->Get());
	SmartDashboard::PutBoolean("CCI1 CH08",pCCI1Ch08->Get());
	SmartDashboard::PutBoolean("CCI1 CH09",pCCI1Ch09->Get());
	SmartDashboard::PutBoolean("CCI1 CH10",pCCI1Ch10->Get());
	SmartDashboard::PutBoolean("CCI1 CH11",pCCI1Ch11->Get());
	SmartDashboard::PutBoolean("CCI1 CH12",pCCI1Ch12->Get());
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
//	SmartDashboard::PutBoolean("Camera Lights",pCameraLights->GetCameraStatus());
//	SmartDashboard::PutNumber("Arm POT Current Position",pArm->GetCurrentPosition());
//	SmartDashboard::PutNumber("Arm POT Target Position",pArm->GetPositionTarget());
//	SmartDashboard::PutBoolean("Upper Limit Switch",pArm->GetUpperLimitSwitch());
//	SmartDashboard::PutBoolean("Lower Limit Switch",pArm->GetLowerLimitSwitch());
//	SmartDashboard::PutNumber("Arm Target Motor Speed",pArm->GetTargetMotorSpeed());
//	SmartDashboard::PutNumber("Arm Motor Speed",pArm->GetMotorSpeed());

//	SmartDashboard::PutNumber("Loader Motor Speed",pBallLoader->GetMotorSpeed());

	SmartDashboard::PutBoolean("Load Ball Mode",loadBall);
	SmartDashboard::PutBoolean("Ball Loaded? Limit switch",pBallLoader->GetLoadedSensor());
	SmartDashboard::PutBoolean("Ball Loaded? Loader code",pBallLoader->GetBallLoaded());
	SmartDashboard::PutBoolean("Ball Loaded? Robot code",ballLoaded);

	SmartDashboard::PutBoolean("Eject Ball Mode",ejectBall);
	SmartDashboard::PutBoolean("First Eject Loop",pBallLoader->GetFirstEjectLoop());
	SmartDashboard::PutNumber("Eject Counter",pBallLoader->GetEjectCounter());
	SmartDashboard::PutBoolean("Ball Ejected? Loader code",pBallLoader->GetBallEjected());
	SmartDashboard::PutBoolean("Ball Ejected? Robot code",ballEjected);

	SmartDashboard::PutBoolean("Shoot Ball Mode",shootBall);
	SmartDashboard::PutBoolean("Shooter 1st loop",pBallShooter->GetShooterFirstLoop());

	SmartDashboard::PutBoolean("Ball in shooter? Limit Switch",
			                    pBallLoader->GetBallInShooterSensor());
	SmartDashboard::PutBoolean("Ball in shooter? Loader code flag",
			                    pBallLoader->GetBallInShooterFlag());
	SmartDashboard::PutBoolean("Ball in shooter? Shooter code flag",
			                    pBallShooter->GetBallInShooter());

	SmartDashboard::PutBoolean("Shooter Reset-Robot code",shooterReset);
	SmartDashboard::PutBoolean("Shooter Reset-Shooter code",pBallShooter->GetShooterReset());
	SmartDashboard::PutBoolean("Shooter Reset prev banner",pBallShooter->GetPrevShooterReset());
	SmartDashboard::PutBoolean("Shooter Reset-Shooter banner sensor",
			                    pBallShooter->GetBannerSensor());
//	SmartDashboard::PutNumber("Shooter Motor Speed",pBallShooter->GetMotorSpeed());

//	SmartDashboard::PutNumber("Climber Motor 1 Speed",pClimber->GetMotor1Speed());
//	SmartDashboard::PutNumber("Climber Motor 2 Speed",pClimber->GetMotor2Speed());

#ifdef VISION
	SmartDashboard::PutBoolean("Camera sees bright", pVision->getIsBright());
#endif

	return;
}
#endif
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::MoveToPosition()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Determines which position to move the arm Arm based on driver station
// switch values then moves to arm to the target position.
//------------------------------------------------------------------------------
void StrongholdRobot::MoveToPosition()
{
	if ( pArmTopSwitch->Get() )
	{
		armInPosition    = false;
		armTarget        = Arm::kPosition3;
	}
	else
	{
		if ( pArmBottomSwitch->Get() )
		{
			armInPosition    = false;
			armTarget        = Arm::kPosition1;
		}
	}

	if ( ! armInPosition )
	{
		armInPosition = pArm->MoveArm(armTarget);
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
	if (   prevShootBallSw           &&
		 ! pShootBallSwitch->Get()       )
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
// METHOD:  StrongholdRobot::GetAutoModeSwitches()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Obtains the autonomous mode switch values from the robot and determines
// which autonomous mode we want to run.
//------------------------------------------------------------------------------
void StrongholdRobot::GetAutoModeSwitches()
{
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
	return;
}
