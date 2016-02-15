//------------------------------------------------------------------------------
// TEAM 1280 - SAN RAMON VALLEY HIGH SCHOOL RAGIN' C-BISCUITS
// 2016 STRONGHOLD ROBOT CODE
//------------------------------------------------------------------------------
#include "WPILib.h" // Instruction to preprocessor to include the WPI Library
                    // header file
#include <cmath>
#include "../H/CameraLights.h"
#include "../H/Elevator.h"
#include "../H/Loader.h"
#include "../H/Shooter.h"

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
		void   MoveElevToPosition();
		void   CheckBallLoader();
		void   ShootBall();

		// Autonomous mode methods
		void   CalcAutoModeTimings();
		void   GetAutoModeSwitches();
		void   RunAutonomousMode();
		void   ShowAMStatus();

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
		static const uint SHOOT_BALL_SW_CH       =  2;
		static const uint LOAD_BALL_SW_CH		 =  11;
		static const uint EJECT_BALL_SW_CH       =  12;

		// Driver Station CCI2 Channels (Uses joystick button references)
		static const uint ELEVATOR_TOP_SW_CH     =  4;
		static const uint ELEVATOR_BOTTOM_SW_CH  =  5;
//		static const uint CAMERA_LIGHTS_SW_CH    =  4;
		static const uint SHOOTER_MOTOR_SW_CH    =  9;

		//----------------------------------------------------------------------
		// ROBOT CHANNELS - INPUTS AND OUTPUTS
		//----------------------------------------------------------------------
        // ROBOT INPUTS
		//----------------------------------------------------------------------

		// roboRio GPIO Channels
		static const uint TOP_LIMIT_SW_CH		   =  0;
		static const uint BOTTOM_LIMIT_SW_CH	   =  1;
		static const uint LOADER_BANNER_SENSOR_CH  =  2;
		static const uint SHOOTER_BANNER_SENSOR_CH =  3;


		// roboRio Analog Channels
		static const uint ELEVATOR_POT_CH 		  =  0;

		// navX MXP Inertial Measurement Unit (IMU) Constants
		static const uint8_t IMU_UPDATE_RATE      = 50;

		//----------------------------------------------------------------------
        // ROBOT OUTPUTS
		//----------------------------------------------------------------------

		// roboRio PWM Channels (PWM = Pulsed width modulation)
		static const uint LEFT_FRONT_MOTOR_CH	  =  0;
		static const uint LEFT_REAR_MOTOR_CH	  =  1;
		static const uint RIGHT_FRONT_MOTOR_CH	  =  2;
		static const uint RIGHT_REAR_MOTOR_CH     =  3;
		static const uint ELEVATOR_MOTOR_CH		  =  4;
		static const uint LOADER_MOTOR_CH		  =  5;
		static const uint SHOOTER_MOTOR_CH		  =  6;

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
		JoystickButton   *pElevTopSwitch;
		JoystickButton   *pElevBottomSwitch;
//		JoystickButton 	 *pCameraLightSwitch;
		JoystickButton   *pLoadBallSwitch;
		JoystickButton   *pEjectBallSwitch;
        JoystickButton   *pShootBallSwitch;
        JoystickButton   *pShooterMotorSwitch;

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
		Elevator 		*pElevator;
		Shooter         *pBallShooter;
#ifdef VISION
		JoystickButton   *pElevBottomSwitch;
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
		// Elevator Arm Positioning
		//----------------------------------------------------------------------
		// Arm Elevator Controls
		bool   elevatorArmInPosition;
		uint   elevatorTarget;
		//----------------------------------------------------------------------
		// Loader
		//----------------------------------------------------------------------
		bool   ballLoaded;
		bool   ballEjected;
		//----------------------------------------------------------------------
		// Shooter
		//----------------------------------------------------------------------
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
	pShootBallSwitch 	  = new JoystickButton(pCCI1,SHOOT_BALL_SW_CH);

	// CCI2 Switches
	pElevTopSwitch        = new JoystickButton(pCCI2,ELEVATOR_TOP_SW_CH);
	pElevBottomSwitch     = new JoystickButton(pCCI2,ELEVATOR_BOTTOM_SW_CH);
//	pCameraLightSwitch    = new JoystickButton(pCCI1,CAMERA_LIGHTS_SW_CH);
	pShooterMotorSwitch   = new JoystickButton(pCCI2,SHOOTER_MOTOR_SW_CH);

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
	pElevator			 = new Elevator (ELEVATOR_MOTOR_CH, ELEVATOR_POT_CH, TOP_LIMIT_SW_CH,
										  BOTTOM_LIMIT_SW_CH);
	
	//Ball Loader
	pBallLoader			 = new Loader(LOADER_MOTOR_CH, LOADER_BANNER_SENSOR_CH);

	pBallShooter		 = new Shooter(SHOOTER_MOTOR_CH, SHOOTER_BANNER_SENSOR_CH, &pBallLoader);

	//----------------------------------------------------------------------
	// INITIALIZE VARIABLES
	//----------------------------------------------------------------------
	// Initialize loop counter
	loopCount      = 0;

	// Initialize robot control variables
	autoMode              = kAutoModeOff;
	elevatorArmInPosition = true;
	ballLoaded            = false;
	ballEjected           = true;
	elevatorTarget        = 0;
	shooterReset		  = false;
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
// - Sets the elevator arm in position variable to true so arm doesn't move
//------------------------------------------------------------------------------
void StrongholdRobot::TeleopInit()
{
#ifdef VISION
	pVision = new Vision;
#endif

	// Loop count initialization
	loopCount      = 0;

	ballLoaded = pBallLoader->GetBannerSensor();

	if ( ballLoaded )
	{
		ballEjected = false;
	}
	else
	{
		ballEjected = true;
	}

	elevatorArmInPosition = true;

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
// - Moves the arm elevator to the target position based on driver station
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

	// Move elevator arm to position defined by driver station switches
	MoveElevToPosition();

	// Turn camera LED lights on or off based on driver station input
/*	if ( lightsOn )
		pCameraLights->TurnOn();
	else
		pCameraLights->TurnOff();
*/
	// Drive Robot using Tank Drive
    pDriveTrain->TankDrive(pDriveStickLeft,pDriveStickRight);


    CheckBallLoader();

    ShootBall();

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
	SmartDashboard::PutBoolean("Elev Top Switch",pElevTopSwitch->Get());
	SmartDashboard::PutBoolean("Elev Bottom Switch",pElevBottomSwitch->Get());
//	SmartDashboard::PutBoolean("Camera Lights Switch",lightsOn);

	SmartDashboard::PutNumber("Left JoyStick",pDriveStickLeft->GetY());
	SmartDashboard::PutNumber("Right JoyStick",pDriveStickRight->GetY());
	SmartDashboard::PutBoolean("Load Ball Switch",pLoadBallSwitch->Get());
	SmartDashboard::PutBoolean("Eject Ball Switch",pEjectBallSwitch->Get());
	SmartDashboard::PutBoolean("Shoot Ball Switch",pShootBallSwitch->Get());
	SmartDashboard::PutBoolean("Shooter Motor Switch",pShooterMotorSwitch->Get());

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
	SmartDashboard::PutNumber("AM Mode",autoMode);
//	SmartDashboard::PutBoolean("Camera Lights",pCameraLights->GetCameraStatus());
	SmartDashboard::PutNumber("Elev POT Current Position",pElevator->GetCurrentPosition());
	SmartDashboard::PutNumber("Elev POT Target Position",pElevator->GetPositionTarget());
	SmartDashboard::PutBoolean("Upper Limit Switch",pElevator->GetUpperLimitSwitch());
	SmartDashboard::PutBoolean("Lower Limit Switch",pElevator->GetLowerLimitSwitch());
	SmartDashboard::PutNumber("Elevator Target Motor Speed",pElevator->GetTargetMotorSpeed());
	SmartDashboard::PutNumber("Elevator Motor Speed",pElevator->GetMotorSpeed());
	SmartDashboard::PutNumber("Loader Eject Counter",pBallLoader->GetEjectCounter());
	SmartDashboard::PutBoolean("Loader Banner Sensor",pBallLoader->GetBannerSensor());
	SmartDashboard::PutNumber("Loader Motor Speed",pBallLoader->GetMotorSpeed());
	SmartDashboard::PutNumber("Shooter Motor Speed",pBallShooter->GetMotorSpeed());
	SmartDashboard::PutBoolean("Shooter Banner Sensor",pBallShooter->GetBannerSensor());

#ifdef VISION
	SmartDashboard::PutBoolean("Camera sees bright", pVision->getIsBright());
#endif

	return;
}
#endif
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::MoveElevToPosition()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Determines which position to move the arm elevator based on driver station
// switch values then moves to arm to the target position.
//------------------------------------------------------------------------------
void StrongholdRobot::MoveElevToPosition()
{
	if ( pElevTopSwitch->Get() )
	{
		elevatorArmInPosition = false;
		elevatorTarget        = Elevator::kHang;
	}
	else
	{
		if ( pElevBottomSwitch->Get() )
		{
			elevatorArmInPosition = false;
			elevatorTarget        = Elevator::kGround;
		}
	}

	if ( ! elevatorArmInPosition )
	{
		elevatorArmInPosition = pElevator->MoveElevator(elevatorTarget);
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  StrongholdRobot::MoveElevToPosition()
// Type:	Public accessor for StrongholdRobot class
//------------------------------------------------------------------------------
// Determines which position to move the arm elevator based on driver station
// switch values then moves to arm to the target position.
//------------------------------------------------------------------------------
void StrongholdRobot::CheckBallLoader()
{
	if ( pLoadBallSwitch->Get() )
	{
		if ( !ballLoaded )
		{
			ballLoaded = pBallLoader->LoadBall();
		}
	}

	if ( pEjectBallSwitch->Get() )
	{
		if ( !ballEjected )
		{
			ballEjected = pBallLoader->EjectBall();
			if ( ballEjected )
			{
				ballLoaded = false;
			}
		}
	}

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
	if ( pShootBallSwitch->Get() )
	{
		if ( !shooterReset )
		{
			shooterReset = pBallShooter->ShootBall();
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
