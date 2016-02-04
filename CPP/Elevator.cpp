#include "../H/Elevator.h"

Elevator::Elevator(uint elevMotorCh, uint elevPotCh, uint upperLimitSwCh, uint lowerLimitSwCh)
{
	pElevatorMotor    = new Talon(elevMotorCh);
	pElevatorPot      = new AnalogPotentiometer(elevPotCh, POT_FULL_RANGE, POT_OFFSET);
	pUpperLimitHit    = new DigitalInput(upperLimitSwCh);
	pLowerLimitHit    = new DigitalInput(lowerLimitSwCh);

	// Initialize class variables
	elevatorTarget   = 0.0;
	targetMotorSpeed = 0.0;

	// Set default starting position for elevator to current position
	MoveElevator(pElevatorPot->Get());
}

Elevator::~Elevator()
{
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::MoveElevator()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input base and offset
// positions.
//------------------------------------------------------------------------------
bool  Elevator::MoveElevator(uint inputTarget)
{
	bool   targetFound  = false;
	double baseTarget   = 0;

	targetInput    = inputTarget;

 	baseTarget     = CalcBaseTarget(inputTarget);

	elevatorTarget = baseTarget;

	targetFound = GoToPotTargetNoPID(elevatorTarget);

	return targetFound;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetTargetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current targeted elevator motor speed.
//------------------------------------------------------------------------------
float  Elevator::GetTargetMotorSpeed() const
{
	return targetMotorSpeed;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current elevator motor speed.
//------------------------------------------------------------------------------
float  Elevator::GetMotorSpeed() const
{
	return pElevatorMotor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetCurrentPosition()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current elevator position potentiometer reading.
//------------------------------------------------------------------------------
double Elevator::GetCurrentPosition() const
{
	return pElevatorPot->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetPositionTarget()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target elevator position potentiometer reading.
//------------------------------------------------------------------------------
double Elevator::GetPositionTarget() const
{
	return elevatorTarget;
}

//------------------------------------------------------------------------------
// METHOD:  Elevator::GetPositionTargetInput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target elevator input potentiometer reading.
//------------------------------------------------------------------------------
uint   Elevator::GetPositionTargetInput() const
{
 	return targetInput;
}

//------------------------------------------------------------------------------
// METHOD:  Elevator::GetUpperLimitSwitch()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the value of the upper limit switch.  Note the limit switches read
// the opposite of what we want them to read to be intuitive.
//------------------------------------------------------------------------------
bool   Elevator::GetUpperLimitSwitch() const
{
	bool upperLimit = false;

	if (pUpperLimitHit->Get() == true)
		upperLimit = false;
	else
		upperLimit = true;

	return upperLimit;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetLowerLimitSwitch()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the value of the lower limit switch.  Note the limit switches read
//  the opposite of what we want them to read to be intuitive.
//------------------------------------------------------------------------------
bool   Elevator::GetLowerLimitSwitch() const
{
	bool lowerLimit = false;

	if (pLowerLimitHit->Get() == true)
		lowerLimit = false;
	else
		lowerLimit = true;

	return lowerLimit;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::CalcBaseTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the base elevator position target based on an input position.
//------------------------------------------------------------------------------
double Elevator::CalcBaseTarget(uint basePosition)
{
	double targetPot = 0;

    switch ( basePosition )
    {
    	case kGround:
			targetPot = ELEV_POS_LOWER_LIMIT;
    		break;

    	case kHang:
			targetPot = ELEV_POS_UPPER_LIMIT;
			break;

    	default:
    		targetPot = pElevatorPot->Get();
    		break;
    }

	return targetPot;
}

//------------------------------------------------------------------------------
// METHOD:  Elevator::GoToPotTargetNoPID()
// Type:	Private method
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool  Elevator::GoToPotTargetNoPID(double inputPotValue)
{
	bool   potTargetFound  = false;
	double targetLowValue  = inputPotValue - TARGET_TOLERANCE;
	double targetHighValue = inputPotValue + TARGET_TOLERANCE;

	if ( pElevatorPot->Get() >= targetLowValue  &&
		 pElevatorPot->Get() <= targetHighValue )
	{
		pElevatorMotor->Set(ALL_STOP);
		targetMotorSpeed = ALL_STOP;
		potTargetFound = true;
	}
	else
	{
		if ( pElevatorPot->Get() > targetHighValue )  // Elevator moving down
		{
			if ( !pLowerLimitHit->Get() )  // If lower limit switch is hit, stop motors
			{
				pElevatorMotor->Set(ALL_STOP);
				targetMotorSpeed = ALL_STOP;
				potTargetFound = true;
			}
			else
			{
				pElevatorMotor->Set(MOTOR_SPEED_DOWN);
				targetMotorSpeed = MOTOR_SPEED_DOWN;
			}
		}
		else
			if ( pElevatorPot->Get() < targetLowValue )  // Elevator moving up
			{
				if ( !pUpperLimitHit->Get() )  // If upper limit switch is hit, stop motors
				{
					pElevatorMotor->Set(ALL_STOP);
					targetMotorSpeed = ALL_STOP;
					potTargetFound = true;
				}
				else
				{
					pElevatorMotor->Set(MOTOR_SPEED_UP);
					targetMotorSpeed = MOTOR_SPEED_UP;
				}
			}
	}

	return potTargetFound;
}
