#include "../H/Elevator.h"

//------------------------------------------------------------------------------
// METHOD:  Elevator::Elevator()
// Type:	Public constructor method
//------------------------------------------------------------------------------
// Creates an elevator object using as input:
// - Elevator motor PWM channel
// - Elevator potentiometer analog channel
// - Upper limit switch digital I/O channel
// - Lower limit switch digital I/O channel
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
// METHOD:  Elevator::Elevator()
// Type:	Public destructor method
//------------------------------------------------------------------------------
// Destroys the elevator object
//------------------------------------------------------------------------------
Elevator::~Elevator()
{
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::MoveElevator()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the elevator to the desired position.
//------------------------------------------------------------------------------
bool  Elevator::MoveElevator(uint inputTarget)
{
	bool   targetFound  = false;

	targetInput    = inputTarget;

	elevatorTarget = CalcPOTTarget(inputTarget);

	targetFound = GoToPotTarget(elevatorTarget);

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
// Returns the current actual elevator motor speed.
//------------------------------------------------------------------------------
float  Elevator::GetMotorSpeed() const
{
	return pElevatorMotor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetCurrentPosition()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual elevator position potentiometer reading.
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
// Returns the target elevator position input value.
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
// *** WE WANT TO CHANGE THE WIRING ON THE LIMIT SWITCHES TO READ THE OPPOSITE
// *** OF WHAT THEY DO NOW SO THE CODE WILL BE MORE INTUITIVE
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
// the opposite of what we want them to read to be intuitive.
// *** WE WANT TO CHANGE THE WIRING ON THE LIMIT SWITCHES TO READ THE OPPOSITE
// *** OF WHAT THEY DO NOW SO THE CODE WILL BE MORE INTUITIVE
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
// METHOD:  Elevator::CalcPOTTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the elevator potentiometer target based on a passed target input
// position.
//------------------------------------------------------------------------------
double Elevator::CalcPOTTarget(uint targetPosition)
{
	double targetPot = 0;

    switch ( targetPosition )
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
// METHOD:  Elevator::GoToPotTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Moves the arm elevator until it reaches the target position, or activates
// the upper or lower limit switches.
// *** NOTE THE LIMIT SWITCHES CURRENTLY READ THE OPPOSITION OF WHAT IS
// *** INTUITIVE.  WE WANT TO CHANGE THE WIRING SO THE LIMIT SWITCHES WILL
// *** READ FALSE WHEN NOT ACTIVATED AND TRUE WHEN ACTIVATED SO THE CODE CAN
// *** BE MORE INTUITIVE.
//------------------------------------------------------------------------------
bool  Elevator::GoToPotTarget(double inputPotValue)
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
