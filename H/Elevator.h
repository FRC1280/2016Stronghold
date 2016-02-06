#ifndef ELEVATOR_H
#define ELEVATOR_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE Elevator CLASS
//------------------------------------------------------------------------------
// Positions the elevator based on a target position.  The target position
// translates into a target potentiometer reading.
//------------------------------------------------------------------------------
class Elevator
{
	public:

		enum  target {kGround, kHang};

		Elevator(uint elevMotorCh, uint elevPotCh, uint upperLimitSwCh, uint lowerLimitSwCh);
		~Elevator();

		bool   MoveElevator(uint inputTarget);
		float  GetTargetMotorSpeed() const;
		float  GetMotorSpeed() const;
		double GetCurrentPosition() const;
		double GetPositionTarget() const;
		uint   GetPositionTargetInput() const;
		bool   GetUpperLimitSwitch() const;
		bool   GetLowerLimitSwitch() const;

	private:
		const float   MOTOR_SPEED_UP           =    0.25;   // CONFIGURE
		const float   MOTOR_SPEED_DOWN         =   -0.25;   // CONFIGURE
		const float   ALL_STOP                 =    0.00;

		const double  ELEV_POS_UPPER_LIMIT     =   75.0;    // CONFIGURE
		const double  ELEV_POS_LOWER_LIMIT     =   25.0;    // CONFIGURE
		const double  TARGET_TOLERANCE         =    1.0;    // CONFIGURE

		const double  POT_FULL_RANGE           = -100.0;    // CONFIGURE
		const double  POT_OFFSET               =  100.0;    // CONFIGURE

		double CalcTargetPotValue(double inputPotValue);
 		double CalcPOTTarget(uint targetPosition);
 		bool   GoToPotTarget(double potTarget);

		Talon               *pElevatorMotor;
		AnalogPotentiometer *pElevatorPot;
		DigitalInput        *pUpperLimitHit;
		DigitalInput        *pLowerLimitHit;
		PIDController       *pPIDController;

		uint                targetInput;
		double              elevatorTarget;
		float               targetMotorSpeed;
};

#endif
