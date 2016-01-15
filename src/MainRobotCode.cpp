//------------------------------------------------------------------------------
// TEAM 1280 - SAN RAMON VALLEY HIGH SCHOOL RAGIN' C-BISCUITS
// 20156 Stronghold ROBOT CODE
//------------------------------------------------------------------------------
#include "WPILib.h" // Instruction to preprocessor to include the WPI Library
                    // header file
#include <cmath>

//#define CONSOLE
//#define VISION

#include "../H/CameraLights.h"

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

		//----------------------------------------------------------------------
		// CUSTOM METHODS SPECIFIC TO TEAM 1280 ROBOT
		//----------------------------------------------------------------------
		// Initialization and reset methods

		// Driver station and robot input gathering methods
		void   GetDriverStationInput();
		void   GetElevatorTarget();
		void   GetElevatorBase();
		void   GetElevatorOffset();
		void   ShowDSValues();
		void   GetRobotSensorInput();
		void   ShowRobotValues();

		// Autonomous mode methods
		void   CalcAutoModeTimings();
		void   GetAutoModeSwitches();
		uint   DeterminePiecesToSet();
		void   RunAutonomousMode();
		void   RunSetRobot();
		void   RunSetToteLeft();
		void   RunSetToteRight();
		void   RunStackTotes();
		void   AMDriveRobot(float driveX, float driveY, float driveZ);
		void   ShowAMStatus();

	private:
		//----------------------------------------------------------------------
		// CONSTANTS USED IN CLASS
		//----------------------------------------------------------------------
		// DRIVER STATION PORTS AND CHANNELS
		//----------------------------------------------------------------------
		// Driver Station Joystick ports

		static const uint JS_PORT          =  0;
		static const uint CCI_PORT         =  1;  // eStop Robots CCI Inputs

		// Driver Station CCI Channels (Uses joystick button references)
		static const uint CAMERA_LIGHTS_SW_CH       =  1;
		//----------------------------------------------------------------------
		// ROBOT CHANNELS - INPUTS AND OUTPUTS
		//----------------------------------------------------------------------
        // ROBOT INPUTS
		//----------------------------------------------------------------------

		// roboRio GPIO Channels


		// roboRio Analog Channels

		// navX MXP Inertial Measurement Unit (IMU) Constants
		static const uint8_t IMU_UPDATE_RATE       = 50;

		//----------------------------------------------------------------------
        // ROBOT OUTPUTS
		//----------------------------------------------------------------------

		// roboRio PWM Channels
		// PWM = Pulsed width modulation
		static const uint LEFT_FRONT_MOTOR_CH	   = 0;
		static const uint LEFT_REAR_MOTOR_CH	   = 1;
		static const uint RIGHT_FRONT_MOTOR_CH	   = 2;
		static const uint RIGHT_REAR_MOTOR_CH      = 3;

		// roboRio Relay Channels
		static const uint CAMERA_LIGHTS_CH         = 2;

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
        const float  AM_STOP_ROBOT_X   =  0.0;   // CONFIG
        const float  AM_STOP_ROBOT_Y   =  0.0;   // CONFIG
        const float  AM_STOP_ROBOT_Z   =  0.0;   // CONFIG

        // Robot Set Drive Speeds
        const float  AM_DRIVE_FWD_X =  0.00;   // CONFIG
        const float  AM_DRIVE_FWD_Y =  0.50;   // CONFIG
        const float  AM_DRIVE_FWD_Z =  0.00;   // CONFIG
        const float  AM_TURN_LEFT_X =  0.0;   // CONFIG
        const float  AM_TURN_LEFT_Y =  0.0;   // CONFIG
        const float  AM_TURN_LEFT_Z =  0.45;  // CONFIG

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
		Joystick		 *pDriveStick;

		// eStop Robotics Custom Control Interface (CCI)
		Joystick         *pCCI;                     // CCI
		JoystickButton 	 *pCameraLightSwitch;

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
		CameraLights	*pCameraLights;			// Camera LED lights
		//----------------------------------------------------------------------
		// Robot Objects
		//----------------------------------------------------------------------
		RobotDrive		*pDriveTrain;
#ifdef VISION
		Vision			*pVision;
#endif
		//----------------------------------------------------------------------
		// VARIABLES USED IN CLASS
		//----------------------------------------------------------------------
		// DRIVER STATION INPUTS - Analog inputs from joysticks and
		// eStop Robotics CCI.
		//----------------------------------------------------------------------
		// Joystick drive speed inputs for tank drive
		// Disk shooting aim angle position
		// - Manual input based on potentionmeter setting
		// Elevator Position
		// - Manual input based on potentiometer setting
		//----------------------------------------------------------------------
//		float  leftDriveSpeed;
//		float  rightDriveSpeed;

		//----------------------------------------------------------------------
		// DRIVER STATION INPUTS - Digital Inputs from eStop Robotics CCI
		//----------------------------------------------------------------------
		// Field Orientation Buttons
		bool   fieldOrientationOn;
		// Camera Switches
		bool   lightsOn;
		// Robot Switches

		//----------------------------------------------------------------------
		// CLASS VARIABLES USED TO TRACK ROBOT STATUS
		//----------------------------------------------------------------------
		// General status tracking
		//----------------------------------------------------------------------
		// Class variables to track look (packet) counts
		uint   loopCount;

		//----------------------------------------------------------------------
		// Camera Image Processing
		//----------------------------------------------------------------------

		//----------------------------------------------------------------------
		// Autonomous Mode Switches & variables
		//----------------------------------------------------------------------
		uint autoMode;

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
	pDriveStick			 = new Joystick(JS_PORT);
	pCCI                 = new Joystick(CCI_PORT); // CCI uses joystick object

	// CCI Switches
	pCameraLightSwitch   			 = new JoystickButton(pCCI,CAMERA_LIGHTS_SW_CH);

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
	pCameraLights		 = new CameraLights(CAMERA_LIGHTS_CH);

	// Drive Train
	pDriveTrain		     = new RobotDrive(LEFT_FRONT_MOTOR_CH,LEFT_REAR_MOTOR_CH,
									      RIGHT_FRONT_MOTOR_CH,RIGHT_REAR_MOTOR_CH);

	
	//----------------------------------------------------------------------
	// INITIALIZE VARIABLES
	//----------------------------------------------------------------------
	// Initialize loop counter
	loopCount      = 0;

	// Initialize robot control variables
	autoMode           = kAutoModeOff;
	fieldOrientationOn = false;  // CONFIG
	lightsOn           = false;  // CONFIG

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
// - Sets type of input used to determine elevator position
// - Sets target values for three preset elevator positions
// - Initializes robot status tracking settings
//------------------------------------------------------------------------------
void StrongholdRobot::RobotInit()
{
#ifdef CONSOLE
	SmartDashboard::init();
#endif

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::DisabledInit()
// Type:	Executes when the robot is placed in Disabled mode.  Overrides
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
// Type:	Executes when the robot is placed in Autonomous mode.  Overrides
//			AutonomousInit() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Resets the loop counter for autonomous mode
// - Resets the AutoState
// - Resets the encoders on the wheels that measure distance traveled.
// - Optionally prints the status of the autonomous mode switches for debugging
//   purposes
//------------------------------------------------------------------------------
void StrongholdRobot::AutonomousInit()
{
	// Reset loop counter
	loopCount  = 0;

	// Set Robot Components to Default Starting Positions
	pCameraLights->TurnOff();                  // CONFIG

	fieldOrientationOn = true;                 // CONFIG

	GetAutoModeSwitches();
	GetRobotSensorInput();
	ShowAMStatus();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::TeleopInit()
// Type:	Executes when the robot is placed in Teleoperated mode.  Overrides
//			TeleopInit() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Resets the loop counters for teleoperated mode
// - Resets the distance tracking variables
// - Resets the wheel encoder counters
// - Obtain the current position of the elevator from the robot
//------------------------------------------------------------------------------
void StrongholdRobot::TeleopInit()
{
#ifdef VISION
	pVision = new Vision;
#endif

	// Loop count initialization
	loopCount      = 0;

	fieldOrientationOn = false;  // CONFIG

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::DisabledPeriodic()
// Type:	Executes when the robot is in disabled mode.  Overrides the
//			DisabledPeriodic() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Increment the disabled loop counter
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
// - Feeds to watchdog to prevent robot shut-down
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
// - Feeds to watchdog to prevent robot shut-down
// - Obtains input from the driver station (joystick inputs, switches, arm
//   rotator potentiometer)
// - Sets the drive motor values based on joystick movement
// - Sets the ball launcher motor speed values based on on/off switch and
//   potentiometer position
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

	// Turn camera LED lights on or off based on driver station input
	if ( lightsOn )
		pCameraLights->TurnOn();
	else
		pCameraLights->TurnOff();

	return;
}

//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::GetDriverStationInput()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Obtains the input from the DriverStation required for teleoperated mode.
// Includes obtaining input for the following switches:
// -
// May also need to include reading additional switches depending on where
// functions are included on the robot or driver station.
// -
// Include also:
// - Reading joystick function
// Does not include:
// - Robot drive input from the joystick.  This is coded directly in the
//   SetDriveMotors() method.
//------------------------------------------------------------------------------
void StrongholdRobot::GetDriverStationInput()
{
	// Obtain the position of switches on the driver station
    // Field Orientation Joystick Button Values
	// Camera Switches
    lightsOn  				 = pCameraLightSwitch->Get();
    pDriveTrain->TankDrive(pDriveStick->GetX(),pDriveStick->GetTwist());

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
//
//------------------------------------------------------------------------------
void StrongholdRobot::ShowDSValues()
{
// Show the values for driver station inputs
	SmartDashboard::PutBoolean("Camera Lights Switch",lightsOn);

	SmartDashboard::PutNumber("Joystick X",pDriveStick->GetX());
	SmartDashboard::PutNumber("Joystick Y",pDriveStick->GetY());
	SmartDashboard::PutNumber("Joystick Twist",pDriveStick->GetTwist());

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
	SmartDashboard::PutNumber("Packet count",loopCount);
	SmartDashboard::PutNumber("AM Mode",autoMode);
	SmartDashboard::PutBoolean("Camera Lights",pCameraLights->GetCameraStatus());

#ifdef VISION
	SmartDashboard::PutBoolean("Camera sees bright", pVision->getIsBright());
#endif

	return;
}
#endif
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::GetAutoModeSwitches()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StrongholdRobot::GetAutoModeSwitches(){
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::RunAutonomousMode()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StrongholdRobot::RunAutonomousMode(){
}
//------------------------------------------------------------	------------------
// METHOD:  StrongholdRobot::AMDriveRobot()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StrongholdRobot::AMDriveRobot(float driveX, float driveY, float driveZ)
{
	//Set Drive Speed and drive mode with or without field orientation
	
	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::ShowAMStatus()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Shifts gear on Grabber to match input from driver station.  Gear shift
// are via a pneumatic cylinder controlled by a solenoid.
//------------------------------------------------------------------------------
void StrongholdRobot::ShowAMStatus()
{
	return;
}
