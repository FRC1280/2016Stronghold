#include "../H/ArmSegment.h"

//------------------------------------------------------------------------------
// METHOD:  ArmSegment::ArmSegment()
// Type:	Public constructor method
//------------------------------------------------------------------------------
// Creates an ArmSegment object using as input:
// - ArmSegment motor PWM channel
// - ArmSegment potentiometer analog channel
// - Upper limit switch digital I/O channel
// - Lower limit switch digital I/O channel
//------------------------------------------------------------------------------
ArmSegment::ArmSegment(uint armMotorCh, uint armPotCh)
{
	// Initialize class variables
	inputPOTLowerLimit  =    0.0;
	inputPOTUpperLimit  =   -1.0;

	outputPOTLowerLimit =    0.0;
	outputPOTUpperLimit =   10.0;

	targetPOTLowerLimit =    0.0;
	targetPOTUpperLimit =   10.0;

	CalcPOTRangeOffset(outputPOTLowerLimit, outputPOTUpperLimit);
	CalcTargetRatioConstant();

	pArmMotor         = new Talon(armMotorCh);
	pArmPot           = new AnalogPotentiometer(armPotCh, outputPOTFullRange, outputPOTOffset);
	pUpperLimitHit    = nullptr;
	pLowerLimitHit    = nullptr;

	// Save POT channel so can reconstruct arm POT later if range & offset change
    armPOTChannel   = armPotCh;

	armTargetInput   = 0.0;
	targetMotorSpeed = 0.0;
	upperLimitSw     = false;
	lowerLimitSw     = false;

	// Set default starting position for Arm to current position
	MoveArm(pArmPot->Get());
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::ArmSegment()
// Type:	Public destructor method
//------------------------------------------------------------------------------
// Destroys the ArmSegment object
//------------------------------------------------------------------------------
ArmSegment::~ArmSegment()
{
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::DefineLimitSwitch()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the ArmSegment to the desired position.
//------------------------------------------------------------------------------
void  ArmSegment::DefineLimitSwitch(uint limitCh, uint limitLocation)
{
    if ( limitLocation == kUpperLimit)
    {
    	pUpperLimitHit = new DigitalInput(limitCh);
    	upperLimitSw   = true;
    }
    else
    {
    	if ( limitLocation == kLowerLimit )
    	{
    		pLowerLimitHit = new DigitalInput(limitCh);
    		lowerLimitSw   = true;
    	}
    }

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::SetInputPotValue()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the ArmSegment to the desired position.
//------------------------------------------------------------------------------
void  ArmSegment::SetInputPotRange(double minPotValue, double maxPotValue)
{
	inputPOTLowerLimit = minPotValue;
	inputPOTUpperLimit = maxPotValue;

	CalcTargetRatioConstant();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::SetOutputPotValue()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the ArmSegment to the desired position.
//------------------------------------------------------------------------------
void  ArmSegment::SetOutputPotRange(double minPotValue, double maxPotValue)
{
	outputPOTUpperLimit = maxPotValue;
	outputPOTLowerLimit = minPotValue;

	CalcPOTRangeOffset(minPotValue, maxPotValue);

//	delete pArmPot;
//	pArmPot = new AnalogPotentiometer(armPOTChannel, outputPOTFullRange, outputPOTOffset);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::SetTargetPotRange()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the ArmSegment to the desired position.
//------------------------------------------------------------------------------
void  ArmSegment::SetTargetPotRange(double minPotValue, double maxPotValue)
{
	targetPOTLowerLimit = minPotValue;
	targetPOTUpperLimit = maxPotValue;

	CalcTargetRatioConstant();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::MoveArm()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the ArmSegment to the desired position.
//------------------------------------------------------------------------------
bool  ArmSegment::MoveArm(double inputTarget)
{
	bool   targetFound = false;

	armTargetInput = inputTarget;
	armTargetCalc  = CalcPOTTarget(inputTarget);

	targetFound    = GoToPotTarget(armTargetInput);

	return targetFound;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::GetTargetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current targeted ArmSegment motor speed.
//------------------------------------------------------------------------------
float  ArmSegment::GetTargetMotorSpeed() const
{
	return targetMotorSpeed;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::GetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual ArmSegment motor speed.
//------------------------------------------------------------------------------
float  ArmSegment::GetMotorSpeed() const
{
	return pArmMotor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::GetCurrentPosition()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual ArmSegment position potentiometer reading.
//------------------------------------------------------------------------------
double ArmSegment::GetCurrentPosition() const
{
	return pArmPot->Get();
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::GetPositionTarget()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target ArmSegment position potentiometer reading.
//------------------------------------------------------------------------------
double ArmSegment::GetPositionTargetInput() const
{
	return armTargetInput;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::GetPositionTarget()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target ArmSegment position potentiometer reading.
//------------------------------------------------------------------------------
double ArmSegment::GetPositionTargetCalc() const
{
	return armTargetCalc;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::GetLimitSwitch()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the value of the upper limit switch.
//------------------------------------------------------------------------------
// *** NOTE TO ALLOW THE LIMIT SWITCHES TO READ THE WAY WE WANT THEM TO IN THE
// *** CODE - WIRE THEM "NORMAL CLOSED" OR NC.
//------------------------------------------------------------------------------
bool   ArmSegment::GetLimitSwitch(uint limitLocation) const
{
	bool  limitSwitchValue = false;

	switch ( limitLocation )
	    {
	    	case kUpperLimit:
				limitSwitchValue = GetUpperLimitSwitch();
	    		break;

	    	case kLowerLimit:
				limitSwitchValue = GetLowerLimitSwitch();
				break;

	    	case kNone:
				limitSwitchValue = false;
				break;

	    	default:
	    		limitSwitchValue = false;
	    		break;
	    }

	return limitSwitchValue;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::GetPositionTarget()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target ArmSegment position potentiometer reading.
//------------------------------------------------------------------------------
double ArmSegment::GetRatio() const
{
	return targetRatio;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::GetPositionTarget()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target ArmSegment position potentiometer reading.
//------------------------------------------------------------------------------
double ArmSegment::GetConstant() const
{
	return targetConstant;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::GetUpperLimitSwitch()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the value of the upper limit switch.
//------------------------------------------------------------------------------
// *** NOTE TO ALLOW THE LIMIT SWITCHES TO READ THE WAY WE WANT THEM TO IN THE
// *** CODE - WIRE THEM "NORMAL CLOSED" OR NC.
//------------------------------------------------------------------------------
bool   ArmSegment::GetUpperLimitSwitch() const
{
	bool  limitSwitchValue = false;

	if ( upperLimitSw )
	{
		limitSwitchValue = pUpperLimitHit->Get();
	}
	else
		limitSwitchValue = false;

	return limitSwitchValue;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::GetLowerLimitSwitch()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the value of the upper limit switch.
//------------------------------------------------------------------------------
// *** NOTE TO ALLOW THE LIMIT SWITCHES TO READ THE WAY WE WANT THEM TO IN THE
// *** CODE - WIRE THEM "NORMAL CLOSED" OR NC.
//------------------------------------------------------------------------------
bool   ArmSegment::GetLowerLimitSwitch() const
{
	bool  limitSwitchValue = false;

	if ( lowerLimitSw )
	{
		limitSwitchValue = pLowerLimitHit->Get();
	}
	else
		limitSwitchValue = false;

	return limitSwitchValue;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::CalcPOTRangeOffset()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the ArmSegment to the desired position.
//------------------------------------------------------------------------------
void  ArmSegment::CalcPOTRangeOffset(double minPotValue, double maxPotValue)
{
	outputPOTFullRange  = maxPotValue - minPotValue;
	outputPOTOffset     = minPotValue;

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::CalcTargetRatioConstant()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target position
// and then moves the ArmSegment to the desired position.
//------------------------------------------------------------------------------
void  ArmSegment::CalcTargetRatioConstant()
{
	if ( inputPOTUpperLimit - inputPOTLowerLimit == 0 )
	{
		targetRatio    = 1;
		targetConstant = 0;
	}
	else
	{
		targetRatio  =  ( ( targetPOTUpperLimit - targetPOTLowerLimit )
		    	        / ( inputPOTUpperLimit  - inputPOTLowerLimit  ) );

		targetConstant = ( targetPOTLowerLimit - ( inputPOTLowerLimit * targetRatio ) );
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::CalcPOTTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the ArmSegment potentiometer target based on a passed target input
// position.
//------------------------------------------------------------------------------
double ArmSegment::CalcPOTTarget(double inputPot)
{
	double outputPot;

	outputPot = ( ( inputPot * targetRatio ) + targetConstant );

	return outputPot;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::CalcPOTTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the ArmSegment potentiometer target based on a passed target input
// position.
//------------------------------------------------------------------------------
double ArmSegment::CalcPOTTargetInput(double outputPot)
{
	double inputPot;

	inputPot = ( ( outputPot - targetConstant ) / targetRatio );

	return inputPot;
}
//------------------------------------------------------------------------------
// METHOD:  ArmSegment::GoToPotTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Moves the arm Arm until it reaches the target position, or activates
// the upper or lower limit switches.
// *** NOTE TO ALLOW THE LIMIT SWITCHES TO READ THE WAY WE WANT THEM TO IN THE
// **  CODE - WIRE THEM "NORMAL CLOSED" OR NC.
//------------------------------------------------------------------------------
bool  ArmSegment::GoToPotTarget(double inputPotValue)
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
			if ( GetLimitSwitch(kLowerLimit) )  // If lower limit switch is hit, stop motors
			{
				pArmMotor->Set(ALL_STOP);
				targetMotorSpeed = ALL_STOP;
				potTargetFound = true;
			}
			else
			{
				pArmMotor->Set(MOTOR_SPEED_CONTRACT);
				targetMotorSpeed = MOTOR_SPEED_CONTRACT;
			}
		}
		else
		{
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
					pArmMotor->Set(MOTOR_SPEED_EXTEND);
					targetMotorSpeed = MOTOR_SPEED_EXTEND;
				}
			}
		}
	}

	return potTargetFound;
}
