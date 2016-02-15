#include "../H/Loader.h"

//------------------------------------------------------------------------------
// METHOD:  Loader::Loader()
// Type:	Public constructor method
//------------------------------------------------------------------------------
// Creates an Loader object using as input:
// - Loader motor PWM channel
// - Loader potentiometer analog channel
// - Upper limit switch digital I/O channel
// - Lower limit switch digital I/O channel
//------------------------------------------------------------------------------
Loader::Loader(uint loadMotorCh, uint loadBannerCh)
{
	pLoaderMotor      = new Talon(loadMotorCh);
	pBannerSensor     = new DigitalInput(loadBannerCh);

	// Initialize class variables
	prevBannerValue   = false;
	ejectCounter 	  = 0;
	shootCounter				 =    0;
	firstShootLoop			 = 	  true;

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
	bool ballLoaded = false;

	if(prevBannerValue == false	 &&	 pBannerSensor->Get() == true)
	{
		StopLoader();
		ballLoaded = true;
	}
	else
		RunLoader(MOTOR_SPEED_LOAD);

	prevBannerValue = pBannerSensor->Get();

	return ballLoaded;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetTargetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current targeted elevator motor speed.
//------------------------------------------------------------------------------
bool  Loader::EjectBall()
{
	bool ballEjected =  false;

	if(firstLoop)
	{
		ejectCounter = 0;
		firstLoop    = false;
	}

	if (ejectCounter == EJECT_LOOPS)
	{
		StopLoader();
		ballEjected  = true;
		firstLoop    = true;
	}
	else
	{
		ejectCounter++;
		RunLoader(MOTOR_SPEED_EJECT);
	}

	return ballEjected;

}//------------------------------------------------------------------------------
// METHOD:  Loader::LoadToShooter()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Loads Ball into Kicker.
//------------------------------------------------------------------------------
bool Loader::LoadToShooter()
{
	bool shooterLoaded = false;

	if ( firstShootLoop )
	{
		shootCounter 	  = 0;
		firstShootLoop 	  = false;
	}

	if ( shootCounter == SHOOT_LOOPS )
	{
		StopLoader();
		shooterLoaded  	  = true;
		firstShootLoop    = true;
	}
	else
	{
		shootCounter++;
		RunLoader(MOTOR_SPEED_SHOOT);
	}

	return shooterLoaded;
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
// METHOD:  Loader::GetBannerSensor()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Banner Sensor reading.
//------------------------------------------------------------------------------
bool Loader::GetBannerSensor() const
{
	return pBannerSensor->Get();
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

