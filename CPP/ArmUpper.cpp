#include "../H/ArmUpper.h"

//------------------------------------------------------------------------------
// METHOD:  ArmUpper::ArmUpper()
// Type:	Public constructor method
//------------------------------------------------------------------------------
// Creates an Arm object using as input:
// - Arm motor PWM channel
// - Arm potentiometer analog channel
//------------------------------------------------------------------------------
ArmUpper::ArmUpper(uint armMotorCh, uint armPotCh)
{
	pArmMotor         = new Talon(armMotorCh);
	pArmPot           = new AnalogPotentiometer(armPotCh, POT_FULL_RANGE, POT_OFFSET);

	// Initialize class variables
	targetPOTInput    = 0.0;
	targetPOTCalc     = 0.0;
	targetPosition    = 0;
	targetMotorSpeed  = 0.0;

	// Set default starting position for Arm to current position
//	MoveArm(pArmPot->Get());
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::~ArmUpper()
// Type:	Public destructor method
//------------------------------------------------------------------------------
// Destroys the Arm object
//------------------------------------------------------------------------------
ArmUpper::~ArmUpper()
{
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::MoveArm()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
bool  ArmUpper::MoveArm(uint inputTarget)
{
	bool   targetFound  = false;

	targetPosition = inputTarget;

	targetPOTCalc  = CalcPOTTarget(inputTarget);

	targetFound = GoToPotTarget(targetPOTCalc);

	return targetFound;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::MoveArm()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
void  ArmUpper::MoveArm(float inputTarget)
{
	targetMotorSpeed = inputTarget;
	pArmMotor->Set(inputTarget);
	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::MoveArmUp()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
void  ArmUpper::MoveArmUp()
{
	MoveArm(MOTOR_SPEED_UP);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::MoveArmDown()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
void  ArmUpper::MoveArmDown()
{
	MoveArm(MOTOR_SPEED_DOWN);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::StopArm()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
void  ArmUpper::StopArm()
{
	MoveArm(ALL_STOP);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::GetTargetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current targeted Arm motor speed.
//------------------------------------------------------------------------------
float  ArmUpper::GetTargetMotorSpeed() const
{
	return targetMotorSpeed;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::GetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Arm motor speed.
//------------------------------------------------------------------------------
float  ArmUpper::GetMotorSpeed() const
{
	return pArmMotor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::GetCurrentPosition()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Arm position potentiometer reading.
//------------------------------------------------------------------------------
double ArmUpper::GetCurrentPosition() const
{
	return pArmPot->Get();
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::GetPositionTarget()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target Arm position potentiometer reading.
//------------------------------------------------------------------------------
double ArmUpper::GetTargetPOTInput() const
{
	return targetPOTInput;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::GetPositionTarget()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target Arm position potentiometer reading.
//------------------------------------------------------------------------------
double ArmUpper::GetTargetPOTCalc() const
{
	return targetPOTCalc;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::GetPositionTargetInput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target Arm position input value.
//------------------------------------------------------------------------------
uint   ArmUpper::GetTargetPosition() const
{
 	return targetPosition;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::CalcPOTTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the Arm potentiometer target based on a passed target input
// position.
//------------------------------------------------------------------------------
double ArmUpper::CalcPOTTarget(uint targetPosition)
{
	double targetPot = 0;

    switch ( targetPosition )
    {
    	case kTop:
			targetPot = ARM_TOP;
    		break;

    	case kMiddle:
			targetPot = ARM_MIDDLE;
			break;

    	case kBottom:
			targetPot = ARM_BOTTOM;
			break;

    	default:
    		targetPot = pArmPot->Get();
    		break;
    }

	return targetPot;
}

//------------------------------------------------------------------------------
// METHOD:  ArmUpper::GoToPotTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Moves the arm Arm until it reaches the target position, or activates
// the upper or lower limit switches.
// *** NOTE TO ALLOW THE LIMIT SWITCHES TO READ THE WAY WE WANT THEM TO IN THE
// **  CODE - WIRE THEM "NORMAL CLOSED" OR NC.
//------------------------------------------------------------------------------
bool  ArmUpper::GoToPotTarget(double inputPotValue)
{
	bool   potTargetFound  = false;
	double targetLowValue  = inputPotValue - TARGET_TOLERANCE;
	double targetHighValue = inputPotValue + TARGET_TOLERANCE;

	if ( pArmPot->Get() >= targetLowValue  &&
		 pArmPot->Get() <= targetHighValue )
	{
		pArmMotor->Set(ALL_STOP);
		targetMotorSpeed = ALL_STOP;
		potTargetFound   = true;
	}
	else
	{
		if ( pArmPot->Get() > targetHighValue )  // Arm moving down
		{
			pArmMotor->Set(MOTOR_SPEED_DOWN);
			targetMotorSpeed = MOTOR_SPEED_DOWN;
		}
		else
		{
			if ( pArmPot->Get() < targetLowValue )  // Arm moving up
			{
					pArmMotor->Set(MOTOR_SPEED_UP);
					targetMotorSpeed = MOTOR_SPEED_UP;
			}
		}
	}

	return potTargetFound;
}
