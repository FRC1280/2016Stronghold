#ifndef ELEVATOR_H
#define ELEVATOR_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE Elevator CLASS
//------------------------------------------------------------------------------
// Positions the elevator based on a target potentiometer reading.  Class
// has two primary input methods:
// 1) Driver station potentiometer reading
// 2) Driver station switch values for base position plus an off-set
//------------------------------------------------------------------------------
class Elevator
{
	public:

		enum  target {kGround, kHang};

		Elevator(uint elevMotorCh, uint elevPotCh, uint upperLimitSwCh, uint lowerLimitSwCh);
		~Elevator();

//		void   SetInputPotRange(double minPotValue, double maxPotValue);
//		bool   MoveElevator(double inputPotReading);
		bool   MoveElevator(uint inputTarget);
//		bool   GetControlType() const;
		float  GetTargetMotorSpeed() const;
		float  GetMotorSpeed() const;
		double GetCurrentPosition() const;
		double GetPositionTarget() const;
		double GetPositionTargetInput() const;
		bool   GetUpperLimitSwitch() const;
		bool   GetLowerLimitSwitch() const;
		//Needed these?
		double CalcBaseTarget(uint basePosition);
		bool GoToPotTargetNoPID(double inputPotValue);

	private:
		const float   MOTOR_SPEED_UP            =   0.5;    // CONFIGURE
		const float   MOTOR_SPEED_DOWN          =  -0.5;    // CONFIGURE
		const float   ALL_STOP                  =   0.0;


//		const double  DEFAULT_INPUT_UPPER_LIMIT =   5.0;    // CONFIGURE
//		const double  DEFAULT_INPUT_LOWER_LIMIT =   0.0;    // CONFIGURE
		const double  ELEV_POS_UPPER_LIMIT      =  75.0;    // CONFIGURE
		const double  ELEV_POS_LOWER_LIMIT      =  25.0;    // CONFIGURE
		const double  TARGET_TOLERANCE          =   1.0;    // CONFIGURE

//		const double  POSITION0_INCREMENT       =   0.75;   // CONFIGURE
//		const double  POSITION1_INCREMENT       =  13.25;   // CONFIGURE

		const double  POT_FULL_RANGE            = 100.0;    // CONFIGURE
		const double  POT_OFFSET                =   0.0;    // CONFIGURE

//		const float kP = 10; //PID controller constants
//		const float kI =  0;
//		const float kD =  0; //these will need to be set to the right values

//		void   CalcTargetRatioConstant();
		double CalcTargetPotValue(double inputPotValue);
//		double CalcBaseTarget(uint basePosition);
//		bool   GoToPotTargetNoPID(double potTarget);
		bool   GoToPotTargetPID(double potTarget);

		Talon               *pElevatorMotor;
		AnalogPotentiometer *pElevatorPot;
		DigitalInput        *pUpperLimitHit;
		DigitalInput        *pLowerLimitHit;
		PIDController       *pPIDController;

		double              inputPOTLowerLimit;
		double              inputPOTUpperLimit;
		double              targetRatio;
		double              targetConstant;
		double              targetInput;
		double              elevatorTarget;
		float               targetMotorSpeed;
		bool                usePIDController;
};

#endif
