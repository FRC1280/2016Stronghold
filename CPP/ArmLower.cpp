#include "../H/ArmLower.h"

//------------------------------------------------------------------------------
// METHOD:  ArmLower::ArmLower()
// Type:	Public constructor method
//------------------------------------------------------------------------------
// Creates an Arm object using as input:
// - Arm motor PWM channel
// - Arm potentiometer analog channel
//------------------------------------------------------------------------------
ArmLower::ArmLower(uint armMotorCh, uint armPotCh)
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
// METHOD:  ArmLower::~ArmLower()
// Type:	Public destructor method
//------------------------------------------------------------------------------
// Destroys the Arm object
//------------------------------------------------------------------------------
ArmLower::~ArmLower()
{
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::MoveArm()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
void  ArmLower::MoveArm(float inputTarget)
{

	targetMotorSpeed = inputTarget;
	pArmMotor->Set(inputTarget);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::MoveArm()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
bool  ArmLower::MoveArm(uint inputTarget)
{
	bool   targetFound  = false;

	targetPosition = inputTarget;

	targetPOTCalc  = CalcPOTTarget(inputTarget);

	targetFound = GoToPotTarget(targetPOTCalc);

	return targetFound;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::MoveArmUp()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
void  ArmLower::MoveArmUp()
{
	MoveArm(MOTOR_SPEED_UP);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::MoveArmDown()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
void  ArmLower::MoveArmDown()
{
	MoveArm(MOTOR_SPEED_DOWN);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::StopArm()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
void  ArmLower::StopArm()
{
	MoveArm(ALL_STOP);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::GetTargetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current targeted Arm motor speed.
//------------------------------------------------------------------------------
float  ArmLower::GetTargetMotorSpeed() const
{
	return targetMotorSpeed;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::GetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Arm motor speed.
//------------------------------------------------------------------------------
float  ArmLower::GetMotorSpeed() const
{
	return pArmMotor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::GetCurrentPosition()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Arm position potentiometer reading.
//------------------------------------------------------------------------------
double ArmLower::GetCurrentPosition() const
{
	return pArmPot->Get();
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::GetPositionTarget()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target Arm position potentiometer reading.
//------------------------------------------------------------------------------
double ArmLower::GetTargetPOTInput() const
{
	return targetPOTInput;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::GetPositionTarget()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target Arm position potentiometer reading.
//------------------------------------------------------------------------------
double ArmLower::GetTargetPOTCalc() const
{
	return targetPOTCalc;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::GetPositionTargetInput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target Arm position input value.
//------------------------------------------------------------------------------
uint   ArmLower::GetTargetPosition() const
{
 	return targetPosition;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::CalcPOTTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the Arm potentiometer target based on a passed target input
// position.
//------------------------------------------------------------------------------
double ArmLower::CalcPOTTarget(uint targetPosition)
{
	double targetPot = 0;

    switch ( targetPosition )
    {
    	case kPosition1:
			targetPot = ARM_POSITION_1;
    		break;

    	case kPosition2:
			targetPot = ARM_POSITION_2;
			break;

    	case kPosition3:
			targetPot = ARM_POSITION_3;
			break;

    	default:
    		targetPot = pArmPot->Get();
    		break;
    }

	return targetPot;
}

//------------------------------------------------------------------------------
// METHOD:  ArmLower::GoToPotTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Moves the arm Arm until it reaches the target position, or activates
// the upper or lower limit switches.
// *** NOTE TO ALLOW THE LIMIT SWITCHES TO READ THE WAY WE WANT THEM TO IN THE
// **  CODE - WIRE THEM "NORMAL CLOSED" OR NC.
//------------------------------------------------------------------------------
bool  ArmLower::GoToPotTarget(double inputPotValue)
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
