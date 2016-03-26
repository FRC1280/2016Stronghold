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
	targetRatio       = 0.0;
	targetConstant    = 0.0;
	targetPOTInput    = 0.0;
	targetPosition    = 0;
	targetPOTOutput   = 0.0;
	targetMotorSpeed  = 0.0;

	// Calculate Ratio and Constant
	CalcTargetRatioConstant();

	// Set default starting position for Arm to current position
	targetPOTOutput = pArmPot->Get();
	targetPOTInput  = CalcInputPOT(targetPOTOutput);
	MoveArmPOTInput(targetPOTInput);
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
// METHOD:  ArmUpper::MoveArmPOTInput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target POT from driver
// station and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
bool  ArmUpper::MoveArmPOTInput(double inputPOT)
{
	bool  targetFound = false;

	targetPOTInput = inputPOT;

	targetPosition = 0;

	targetPOTOutput = CalcOutputPOT(inputPOT);

	targetFound = GoToPOTTarget(targetPOTOutput);

	return targetFound;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::MoveArmPositionInput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target switch position
// from driver station and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
bool  ArmUpper::MoveArmPositionInput(uint inputTarget)
{
	bool   targetFound  = false;

	targetPosition  = inputTarget;

	targetPOTOutput = CalcOutputPOT(inputTarget);

	targetPOTInput  = CalcInputPOT(targetPOTOutput);

	targetFound = GoToPOTTarget(targetPOTOutput);

	return targetFound;
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
	if ( pArmPot->Get() <= ARM_BACK_STOP_POT )
		RunArmMotor(MOTOR_SPEED_UP);
	else
		StopArm();

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
	if ( pArmPot->Get() >= ARM_FWD_STOP_POT )
		RunArmMotor(MOTOR_SPEED_DOWN);
	else
		StopArm();

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
	RunArmMotor(ALL_STOP);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::GetTargetPOTInput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target arm position potentiometer reading.
//------------------------------------------------------------------------------
double ArmUpper::GetTargetPOTInput() const
{
	return targetPOTInput;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::GetTargetPosition()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target arm position input value.
//------------------------------------------------------------------------------
uint   ArmUpper::GetTargetPosition() const
{
 	return targetPosition;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::GetTargetPOTOutput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target arm position potentiometer reading.
//------------------------------------------------------------------------------
double ArmUpper::GetTargetPOTOutput() const
{
	return targetPOTOutput;
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
// METHOD:  ArmUpper::GetRatio()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the ratio used to convert the input POT to the target POT reading.
//------------------------------------------------------------------------------
double ArmUpper::GetRatio() const
{
	return targetRatio;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::GetConstant()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the ratio used to convert the input POT to the target POT reading.
//------------------------------------------------------------------------------
double ArmUpper::GetConstant() const
{
	return targetConstant;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::CalcTargetRatioConstant()
// Type:	Private method
//------------------------------------------------------------------------------
// Calculates the ratio and constant used to convert the driver station input
// POT value to the target robot POT value
//------------------------------------------------------------------------------
void  ArmUpper::CalcTargetRatioConstant()
{
	targetRatio    = ( ( OUTPUT_POT_FULL_FWD - OUTPUT_POT_FULL_BACK )
			         / ( INPUT_POT_FULL_FWD  - INPUT_POT_FULL_BACK  ) );

	targetConstant = ( OUTPUT_POT_FULL_BACK - ( INPUT_POT_FULL_BACK * targetRatio ) );

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::CalcOutputPOT()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the Arm potentiometer target based on a passed target POT
// reading.
//------------------------------------------------------------------------------
double  ArmUpper::CalcOutputPOT(double inputPOTValue)
{
	double targetPOT = 0.0;

	targetPOT  = ( ( inputPOTValue * targetRatio ) + targetConstant );

	return targetPOT;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::CalcOutputPOT()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the Arm potentiometer target based on a passed target input
// position.
//------------------------------------------------------------------------------
double ArmUpper::CalcOutputPOT(uint inputPosition)
{
	double targetPOT = 0.0;

    switch ( inputPosition )
    {
    	case kTop:
			targetPOT = ARM_TOP;
    		break;

    	case kMiddle:
			targetPOT = ARM_MIDDLE;
			break;

    	case kBottom:
			targetPOT = ARM_BOTTOM;
			break;

    	default:
    		targetPOT = pArmPot->Get();
    		break;
    }

	return targetPOT;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::CalcInputPOT()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the input Arm potentiometer reading based on the robot
// potentiometer reading.
//------------------------------------------------------------------------------
double ArmUpper::CalcInputPOT(double outputPotValue)
{
	double inputPot = 0.0;

	inputPot  = ( ( outputPotValue - targetConstant ) / targetRatio );

	return inputPot;
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
bool  ArmUpper::GoToPOTTarget(double inputPotValue)
{
	bool   potTargetFound  = false;
	double targetLowValue  = inputPotValue - TARGET_TOLERANCE;
	double targetHighValue = inputPotValue + TARGET_TOLERANCE;

	if ( pArmPot->Get() >= targetLowValue  &&
		 pArmPot->Get() <= targetHighValue )
	{
		StopArm();
		potTargetFound   = true;
	}
	else
	{
		if ( pArmPot->Get() > targetHighValue )  // Arm moving forward
		{
			MoveArmUp();
		}
		else
		{
			if ( pArmPot->Get() < targetLowValue )  // Arm moving back
			{
					MoveArmDown();
			}
		}
	}

	return potTargetFound;
}
//------------------------------------------------------------------------------
// METHOD:  ArmUpper::RunaArmMotor()
// Type:	Private method
//------------------------------------------------------------------------------
// Runs the arm motor at a given speed.
//------------------------------------------------------------------------------
void  ArmUpper::RunArmMotor(float motorSpeed)
{
	targetMotorSpeed = motorSpeed;

	pArmMotor->Set(motorSpeed);

	return;
}
