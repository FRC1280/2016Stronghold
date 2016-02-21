#include "../H/Shooter.h"

//------------------------------------------------------------------------------
// METHOD:  Shooter::Shooter()
// Type:	Public constructor method
//------------------------------------------------------------------------------
// Creates an Shooter object using as input:
// - Shooter motor PWM channel
// - Shooter potentiometer analog channel
// - Upper limit switch digital I/O channel
// - Lower limit switch digital I/O channel
//------------------------------------------------------------------------------
Shooter::Shooter(uint loadMotorCh, uint loadBannerCh, Loader *pRobotLoader)
{
	pShooterMotor     = new Spark(loadMotorCh);
	pBannerSensor     = new DigitalInput(loadBannerCh);

	pBallLoader       =  pRobotLoader;

	// Initialize class variables
	prevBannerValue   = true;
	shooterReset	  = true;
	firstLoop 		  = true;
	ballLoaded        = false;

	// Set default starting position for Shooter to current position
	StopShooter();
}

//------------------------------------------------------------------------------
// METHOD:  Shooter::Shooter()
// Type:	Public destructor method
//------------------------------------------------------------------------------
// Destroys the Shooter object
//------------------------------------------------------------------------------
Shooter::~Shooter()
{
}
//------------------------------------------------------------------------------
// METHOD:  Shooter::MoveShooter()
// Type:	Public accessor method
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool  Shooter::ShootBall()
{
	if ( firstLoop )
	{
		firstLoop    = false;
		shooterReset = false;
		ballLoaded   = false;
	}

	if ( ! ballLoaded )
		ballLoaded = pBallLoader->LoadToShooter();

	if ( ballLoaded )
	{
		RunShooter();
	}

	if (  prevBannerValue        &&
		! pBannerSensor->Get()   &&
		! pBallLoader->GetBallInShooterSensor() )
	{
		StopShooter();
		firstLoop    = true;
		shooterReset = true;
		ballLoaded   = false;
	}

	prevBannerValue = pBannerSensor->Get();

	return shooterReset;
}
//------------------------------------------------------------------------------
// METHOD:  Shooter::RunShooter()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Sets motor speed.
//------------------------------------------------------------------------------
void Shooter::RunShooter()
{
	pShooterMotor->Set(MOTOR_SPEED_SHOOT);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  Shooter::StopShooter()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Stops the Shooter.
//------------------------------------------------------------------------------
void Shooter::StopShooter()
{
	pShooterMotor->Set(ALL_STOP);

	return;
}//------------------------------------------------------------------------------
// METHOD:  Shooter::GetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Shooter motor speed.
//------------------------------------------------------------------------------
float  Shooter::GetMotorSpeed() const
{
	return pShooterMotor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Shooter::GetBannerSensor()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Banner Sensor reading.
//------------------------------------------------------------------------------
bool Shooter::GetBannerSensor() const
{
	return pBannerSensor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Shooter::GetBannerSensor()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Banner Sensor reading.
//------------------------------------------------------------------------------
bool Shooter::GetShooterFirstLoop() const
{
	return firstLoop;
}
//------------------------------------------------------------------------------
// METHOD:  Shooter::GetBannerSensor()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Banner Sensor reading.
//------------------------------------------------------------------------------
bool Shooter::GetPrevShooterReset() const
{
	return prevBannerValue;
}
//------------------------------------------------------------------------------
// METHOD:  Shooter::GetBannerSensor()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Banner Sensor reading.
//------------------------------------------------------------------------------
bool Shooter::GetShooterReset() const
{
	return shooterReset;
}
//------------------------------------------------------------------------------
// METHOD:  Shooter::GetBannerSensor()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Banner Sensor reading.
//------------------------------------------------------------------------------
bool Shooter::GetBallInShooter() const
{
	return ballLoaded;
}
