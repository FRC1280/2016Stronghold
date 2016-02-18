#include "../H/Arm.h"

//------------------------------------------------------------------------------
// METHOD:  Arm::Arm()
// Type:	Public constructor method
//------------------------------------------------------------------------------
// Creates an Arm object using as input:
// - Arm motor PWM channel
// - Arm potentiometer analog channel
// - Upper limit switch digital I/O channel
// - Lower limit switch digital I/O channel
//------------------------------------------------------------------------------
Arm::Arm(uint armMotorCh, uint armPotCh, uint upperLimitSwCh, uint lowerLimitSwCh)
{
	pArmMotor         = new Talon(armMotorCh);
	pArmPot           = new AnalogPotentiometer(armPotCh, POT_FULL_RANGE, POT_OFFSET);
	pUpperLimitHit    = new DigitalInput(upperLimitSwCh);
	pLowerLimitHit    = new DigitalInput(lowerLimitSwCh);

	// Initialize class variables
	armTarget        = 0.0;
	targetMotorSpeed = 0.0;

	// Set default starting position for Arm to current position
	MoveArm(pArmPot->Get());
}

//------------------------------------------------------------------------------
// METHOD:  Arm::Arm()
// Type:	Public destructor method
//------------------------------------------------------------------------------
// Destroys the Arm object
//------------------------------------------------------------------------------
Arm::~Arm()
{
}
//------------------------------------------------------------------------------
// METHOD:  Arm::MoveArm()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
bool  Arm::MoveArm(uint inputTarget)
{
	bool   targetFound  = false;

	targetInput    = inputTarget;

	armTarget = CalcPOTTarget(inputTarget);

	targetFound = GoToPotTarget(armTarget);

	return targetFound;
}
//------------------------------------------------------------------------------
// METHOD:  Arm::GetTargetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current targeted Arm motor speed.
//------------------------------------------------------------------------------
float  Arm::GetTargetMotorSpeed() const
{
	return targetMotorSpeed;
}
//------------------------------------------------------------------------------
// METHOD:  Arm::GetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Arm motor speed.
//------------------------------------------------------------------------------
float  Arm::GetMotorSpeed() const
{
	return pArmMotor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Arm::GetCurrentPosition()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Arm position potentiometer reading.
//------------------------------------------------------------------------------
double Arm::GetCurrentPosition() const
{
	return pArmPot->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Arm::GetPositionTarget()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target Arm position potentiometer reading.
//------------------------------------------------------------------------------
double Arm::GetPositionTarget() const
{
	return armTarget;
}

//------------------------------------------------------------------------------
// METHOD:  Arm::GetPositionTargetInput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target Arm position input value.
//------------------------------------------------------------------------------
uint   Arm::GetPositionTargetInput() const
{
 	return targetInput;
}

//------------------------------------------------------------------------------
// METHOD:  Arm::GetUpperLimitSwitch()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the value of the upper limit switch.
//------------------------------------------------------------------------------
// *** NOTE TO ALLOW THE LIMIT SWITCHES TO READ THE WAY WE WANT THEM TO IN THE
// *** CODE - WIRE THEM "NORMAL CLOSED" OR NC.
//------------------------------------------------------------------------------
bool   Arm::GetUpperLimitSwitch() const
{
	return pUpperLimitHit->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Arm::GetLowerLimitSwitch()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the value of the lower limit switch.
//------------------------------------------------------------------------------
// *** NOTE TO ALLOW THE LIMIT SWITCHES TO READ THE WAY WE WANT THEM TO IN THE
// *** CODE - WIRE THEM "NORMAL CLOSED" OR NC.
//------------------------------------------------------------------------------
bool   Arm::GetLowerLimitSwitch() const
{
	return pLowerLimitHit->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Arm::CalcPOTTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the Arm potentiometer target based on a passed target input
// position.
//------------------------------------------------------------------------------
double Arm::CalcPOTTarget(uint targetPosition)
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
// METHOD:  Arm::GoToPotTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Moves the arm Arm until it reaches the target position, or activates
// the upper or lower limit switches.
// *** NOTE TO ALLOW THE LIMIT SWITCHES TO READ THE WAY WE WANT THEM TO IN THE
// **  CODE - WIRE THEM "NORMAL CLOSED" OR NC.
//------------------------------------------------------------------------------
bool  Arm::GoToPotTarget(double inputPotValue)
{
	bool   potTargetFound  = false;
	double targetLowValue  = inputPotValue - TARGET_TOLERANCE;
	double targetHighValue = inputPotValue + TARGET_TOLERANCE;

	if ( pArmPot->Get() >= targetLowValue  &&
		 pArmPot->Get() <= targetHighValue )
	{
		pArmMotor->Set(ALL_STOP);
		targetMotorSpeed = ALL_STOP;
		potTargetFound = true;
	}
	else
	{
		if ( pArmPot->Get() > targetHighValue )  // Arm moving down
		{
			if ( pLowerLimitHit->Get() )  // If lower limit switch is hit, stop motors
			{
				pArmMotor->Set(ALL_STOP);
				targetMotorSpeed = ALL_STOP;
				potTargetFound = true;
			}
			else
			{
				pArmMotor->Set(MOTOR_SPEED_DOWN);
				targetMotorSpeed = MOTOR_SPEED_DOWN;
			}
		}
		else
			if ( pArmPot->Get() < targetLowValue )  // Arm moving up
			{
				if ( pUpperLimitHit->Get() )  // If upper limit switch is hit, stop motors
				{
					pArmMotor->Set(ALL_STOP);
					targetMotorSpeed = ALL_STOP;
					potTargetFound = true;
				}
				else
				{
					pArmMotor->Set(MOTOR_SPEED_UP);
					targetMotorSpeed = MOTOR_SPEED_UP;
				}
			}
	}

	return potTargetFound;
}
