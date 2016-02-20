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
	prevBannerValue   = false;
	shooterReady	  = true;
	firstLoop 		  = true;
	ballLoaded        = false;

	// Set default starting position for Shooter to current position
	StopShooter();
}

//Change code - add limit switch to detect loaded ball in shooter

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
		firstLoop = false;
		shooterReady = false;
		ballLoaded = false;
	}

	if ( ! ballLoaded )
		ballLoaded = pBallLoader->LoadToShooter();

	RunShooter(MOTOR_SPEED_SHOOT);

	if( !prevBannerValue && pBannerSensor->Get() )
	{
		shooterReady = true;
		StopShooter();
		firstLoop = true;
	}

	prevBannerValue = pBannerSensor->Get();

	return shooterReady;
}
//------------------------------------------------------------------------------
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
// METHOD:  Shooter::RunShooter()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Sets motor speed.
//------------------------------------------------------------------------------
void Shooter::RunShooter()
{
	RunShooter(MOTOR_SPEED_SHOOT);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  Shooter::RunShooter()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Sets motor speed.
//------------------------------------------------------------------------------
void Shooter::RunShooter(float motorSpeed)
{
	pShooterMotor->Set(motorSpeed);

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
}


