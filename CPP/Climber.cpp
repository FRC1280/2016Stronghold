#include "../H/Climber.h"

//------------------------------------------------------------------------------
// METHOD:  Climber::Climber()
// Type:	Public constructor method
//------------------------------------------------------------------------------
// Creates an Climber object using as input:
// - Climber motor PWM channel
// - Climber potentiometer analog channel
// - Upper limit switch digital I/O channel
// - Lower limit switch digital I/O channel
//------------------------------------------------------------------------------
Climber::Climber(uint climbMotor1Ch, uint climbMotor2Ch)
{
	pClimberMotor1     = new Talon(climbMotor1Ch);
	pClimberMotor2     = new Talon(climbMotor2Ch);

	StopClimber();
}

//------------------------------------------------------------------------------
// METHOD:  Climber::Climber()
// Type:	Public destructor method
//------------------------------------------------------------------------------
// Destroys the Climber object
//------------------------------------------------------------------------------
Climber::~Climber()
{
}
//------------------------------------------------------------------------------
// METHOD:  Climber::MoveClimber()
// Type:	Public accessor method
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void  Climber::Climb()
{
	pClimberMotor1->Set(MOTOR_SPEED_CLIMB);
	pClimberMotor2->Set(MOTOR_SPEED_CLIMB);
	return;
}
//------------------------------------------------------------------------------
// METHOD:  Climber::MoveClimber()
// Type:	Public accessor method
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void  Climber::Lower()
{
	pClimberMotor1->Set(MOTOR_SPEED_LOWER);
	pClimberMotor2->Set(MOTOR_SPEED_LOWER);
	return;
}

//------------------------------------------------------------------------------
// METHOD:  Climber::GetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Climber motor speed.
//------------------------------------------------------------------------------
float  Climber::GetMotor1Speed() const
{
	return pClimberMotor1->Get();
}

//------------------------------------------------------------------------------
// METHOD:  Climber::GetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Climber motor speed.
//------------------------------------------------------------------------------
float  Climber::GetMotor2Speed() const
{
	return pClimberMotor2->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Climber::StopClimber()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Stops the Climber.
//------------------------------------------------------------------------------
void Climber::StopClimber()
{
	pClimberMotor1->Set(ALL_STOP);
	pClimberMotor2->Set(ALL_STOP);

	return;
}
