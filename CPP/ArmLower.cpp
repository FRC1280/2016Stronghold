#include "../H/ArmLower.h"

//------------------------------------------------------------------------------
// METHOD:  ArmLower::ArmLower()
// Type:	Public constructor method
//------------------------------------------------------------------------------
// Creates an Arm object using as input:
// - Arm motor PWM channel
// - Arm potentiometer analog channel
//------------------------------------------------------------------------------
ArmLower::ArmLower(uint armMotorCh, uint armPotCh, uint armSensorCh)
{
	pArmMotor         = new Talon(armMotorCh);
	pArmPot           = new AnalogPotentiometer(armPotCh, POT_FULL_RANGE, POT_OFFSET);
	pArmStopSensor    = new DigitalInput(armSensorCh);

	// Initialize class variables
	targetRatio       = 0.0;
	targetConstant    = 0.0;
	targetPOTInput    = 0.0;
	targetPOTOutput   = 0.0;
	targetPosition    = 0;
	targetMotorSpeed  = 0.0;

	// Calculate Ratio and Constant
	CalcTargetRatioConstant();

	// Set default starting position for Arm to current position
	targetPOTOutput = pArmPot->Get();
	targetPOTInput  = CalcInputPOT(targetPOTOutput);
	MoveArmPOTInput(targetPOTInput);
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
// METHOD:  ArmLower::MoveArmPOTInput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target POT from driver
// station and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
bool  ArmLower::MoveArmPOTInput(double inputPOT)
{
	bool  targetFound = false;

	targetPOTInput = inputPOT;

	targetPosition = 0;

	targetPOTOutput = CalcOutputPOT(inputPOT);

	targetFound = GoToPOTTarget(targetPOTOutput);

	return targetFound;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::MoveArmPositionInput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input target switch position
// from driver station and then moves the Arm to the desired position.
//------------------------------------------------------------------------------
bool  ArmLower::MoveArmPositionInput(uint inputTarget)
{
	bool   targetFound  = false;

	targetPosition  = inputTarget;

	targetPOTOutput = CalcOutputPOT(inputTarget);

	targetPOTInput  = CalcInputPOT(targetPOTOutput);

	targetFound = GoToPOTTarget(targetPOTOutput);

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
	if ( pArmStopSensor->Get()     ||
		pArmPot->Get() <= OUTPUT_POT_FULL_FWD )
		StopArm();
	else
		RunArmMotor(MOTOR_SPEED_UP);

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
	if ( pArmPot->Get() >= OUTPUT_POT_FULL_BACK )
		StopArm();
	else
		RunArmMotor(MOTOR_SPEED_DOWN);

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
	RunArmMotor(ALL_STOP);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::GetTargetPOTOutput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target Arm position potentiometer reading.
//------------------------------------------------------------------------------
double ArmLower::GetTargetPOTInput() const
{
	return targetPOTInput;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::GetTargetPosition()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target Arm position input value.
//------------------------------------------------------------------------------
uint   ArmLower::GetTargetPosition() const
{
 	return targetPosition;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::GetTargetPOTOutput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target Arm position potentiometer reading.
//------------------------------------------------------------------------------
double ArmLower::GetTargetPOTOutput() const
{
	return targetPOTOutput;
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
// METHOD:  ArmLower::GetStopSensor()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current actual Arm motor speed.
//------------------------------------------------------------------------------
bool   ArmLower::GetStopSensor() const
{
	return pArmStopSensor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::GetRatio()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the ratio used to convert the input POT to the target output POT.
//------------------------------------------------------------------------------
double  ArmLower::GetRatio() const
{
 	return targetRatio;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::GetConstant()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the constant used to convert the input POT to the target output POT.
//------------------------------------------------------------------------------
double  ArmLower::GetConstant() const
{
 	return targetConstant;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::CalcTargetRatioConstant()
// Type:	Private method
//------------------------------------------------------------------------------
// Calculates the ratio and constant used to convert the driver station input
// POT value to the target robot POT value
//------------------------------------------------------------------------------
void  ArmLower::CalcTargetRatioConstant()
{
	targetRatio    = ( ( OUTPUT_POT_FULL_FWD - OUTPUT_POT_FULL_BACK )
			         / ( INPUT_POT_FULL_FWD  - INPUT_POT_FULL_BACK  ) );

	targetConstant = ( OUTPUT_POT_FULL_BACK - ( INPUT_POT_FULL_BACK * targetRatio ) );

	return;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::CalcOutputPOT()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the Arm potentiometer target based on a passed target POT
// reading.
//------------------------------------------------------------------------------
double  ArmLower::CalcOutputPOT(double inputPOTValue)
{
	double targetPOT = 0.0;

	targetPOT  = ( ( inputPOTValue * targetRatio ) + targetConstant );

	return targetPOT;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::CalcOutputPOT()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the Arm potentiometer target based on a passed target input
// position.
//------------------------------------------------------------------------------
double ArmLower::CalcOutputPOT(uint inputPosition)
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
// METHOD:  ArmLower::CalcInputPOT()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the input Arm potentiometer reading based on the robot
// potentiometer reading.
//------------------------------------------------------------------------------
double ArmLower::CalcInputPOT(double outputPotValue)
{
	double inputPot = 0.0;

	inputPot  = ( ( outputPotValue - targetConstant ) / targetRatio );

	return inputPot;
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
bool  ArmLower::GoToPOTTarget(double inputPotValue)
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
		if ( pArmPot->Get() > targetHighValue )  // Arm moving down
		{
			MoveArmUp();
		}
		else
		{
			if ( pArmPot->Get() < targetLowValue )  // Arm moving up
			{
					MoveArmDown();
			}
		}
	}

	return potTargetFound;
}
//------------------------------------------------------------------------------
// METHOD:  ArmLower::RunaArmMotor()
// Type:	Private method
//------------------------------------------------------------------------------
// Runs the arm motor at a given speed.
//------------------------------------------------------------------------------
void  ArmLower::RunArmMotor(float motorSpeed)
{
	targetMotorSpeed = motorSpeed;

	pArmMotor->Set(motorSpeed);

	return;
}
