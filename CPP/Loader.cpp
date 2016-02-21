#include "../H/Loader.h"

//------------------------------------------------------------------------------
// METHOD:  Loader::Loader()
// Type:	Public constructor method
//------------------------------------------------------------------------------
// Creates an Loader object using as input:
// - Loader motor PWM channel
// - Ball loaded sensor digital I/O channel
// - Ball in shooter sensor digital I/O channel
//------------------------------------------------------------------------------
Loader::Loader(uint loadMotorCh, uint loadedSensorCh, uint inShooterSensorCh)
{
	pLoaderMotor      = new Spark(loadMotorCh);
	pLoadedSensor     = new DigitalInput(loadedSensorCh);
	pInShooterSensor  = new DigitalInput(inShooterSensorCh);

	// Initialize class variables
	// Load Ball
	ballLoaded          = false;
	// Eject Ball
	ballEjected         = true;
	firstEjectLoop      = true;
	ejectCounter 	    = 0;
	// Load Ball in Shooter
	ballInShooter       = false;

	// Set default starting position for Loader to current position
	StopLoader();
}

//------------------------------------------------------------------------------
// METHOD:  Loader::Loader()
// Type:	Public destructor method
//------------------------------------------------------------------------------
// Destroys the Loader object
//------------------------------------------------------------------------------
Loader::~Loader()
{
}
//------------------------------------------------------------------------------
// METHOD:  Loader::MoveLoader()
// Type:	Public accessor method
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool  Loader::LoadBall()
{
	if ( pLoadedSensor->Get() )
	{
		StopLoader();
		ballLoaded  = true;
		ballEjected = false;
	}
	else
	{
		RunLoader(MOTOR_SPEED_LOAD);
		ballLoaded  = false;
	}

	return ballLoaded;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::EjectBall()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current targeted elevator motor speed.
//------------------------------------------------------------------------------
bool  Loader::EjectBall()
{
	if ( firstEjectLoop )
	{
		ejectCounter   = 0;
		ballEjected    = false;
		firstEjectLoop = false;
	}

	if ( ejectCounter >= MAX_EJECT_LOOPS )
	{
		StopLoader();
		ballEjected    = true;
		firstEjectLoop = true;
	}
	else
	{
		RunLoader(MOTOR_SPEED_EJECT);
		ejectCounter++;
	}

	return ballEjected;
}
//------------------------------------------------------------------------------
// METHOD:  Loader::LoadToShooter()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Loads Ball into Kicker.
//------------------------------------------------------------------------------
bool Loader::LoadToShooter()
{
	if ( pInShooterSensor->Get() )
	{
		StopLoader();
		ballInShooter = true;
		ballEjected   = true;
		ballLoaded    = false;
	}
	else
	{
		RunLoader(MOTOR_SPEED_SHOOT);
		ballInShooter = false;
	}

	return ballInShooter;
}
//------------------------------------------------------------------------------
// METHOD:  Loader::GetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual loader motor speed.
//------------------------------------------------------------------------------
float  Loader::GetMotorSpeed() const
{
	return pLoaderMotor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Loader::GetLoadedSensor()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Banner Sensor reading.
//------------------------------------------------------------------------------
bool Loader::GetLoadedSensor() const
{
	return pLoadedSensor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Loader::GetBallLoaded()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Banner Sensor reading.
//------------------------------------------------------------------------------
bool Loader::GetBallLoaded() const
{
	return ballLoaded;
}
//------------------------------------------------------------------------------
// METHOD:  Loader::GetFirstEjectLoop()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Banner Sensor reading.
//------------------------------------------------------------------------------
bool Loader::GetFirstEjectLoop() const
{
	return firstEjectLoop;
}
//------------------------------------------------------------------------------
// METHOD:  Loader::GetEjectCounter()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current eject loop.
//------------------------------------------------------------------------------
int Loader::GetEjectCounter() const
{
	return ejectCounter;
}
//------------------------------------------------------------------------------
// METHOD:  Loader::GetBallEjected()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current eject loop.
//------------------------------------------------------------------------------
bool Loader::GetBallEjected() const
{
	return ballEjected;
}
//------------------------------------------------------------------------------
// METHOD:  Loader::GetBallInShooter()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current eject loop.
//------------------------------------------------------------------------------
bool Loader::GetBallInShooterSensor() const
{
	return pInShooterSensor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Loader::GetBallInShooter()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current eject loop.
//------------------------------------------------------------------------------
bool Loader::GetBallInShooterFlag() const
{
	return ballInShooter;
}
//------------------------------------------------------------------------------
// METHOD:  Loader::RunLoader()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Sets motor speed.
//------------------------------------------------------------------------------
void Loader::RunLoader(float motorSpeed)
{
	pLoaderMotor->Set(motorSpeed);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  Loader::StopLoader()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Stops the Loader.
//------------------------------------------------------------------------------
void Loader::StopLoader()
{
	pLoaderMotor->Set(ALL_STOP);

	return;
}
