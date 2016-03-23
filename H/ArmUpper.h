#ifndef ARMUPPER_H
#define ARMUPPER_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE Arm CLASS
//------------------------------------------------------------------------------
// Positions the Arm based on a target position.  The target position
// translates into a target potentiometer reading.
//------------------------------------------------------------------------------
class ArmUpper
{
	public:

		enum  target {kPosition1, kPosition2, kPosition3};

		ArmUpper(uint armMotorCh, uint armPotCh);
		~ArmUpper();

		bool   MoveArm(uint inputTarget);
		void   MoveArm(float inputTarget);
		void   MoveArmUp();
		void   MoveArmDown();
		void   StopArm();
		float  GetTargetMotorSpeed() const;
		float  GetMotorSpeed()       const;
		double GetCurrentPosition()  const;
		double GetTargetPOTInput()   const;
		double GetTargetPOTCalc()    const;
		uint   GetTargetPosition()   const;

	private:
		const float   MOTOR_SPEED_UP           =    1.00;   // CONFIGURE
		const float   MOTOR_SPEED_DOWN         =   -1.00;   // CONFIGURE
		const float   ALL_STOP                 =    0.00;

		const double  ARM_POSITION_1           =   25.0;    // CONFIGURE
		const double  ARM_POSITION_2           =   50.0;    // CONFIGURE
		const double  ARM_POSITION_3           =   75.0;    // CONFIGURE
		const double  TARGET_TOLERANCE         =    1.0;    // CONFIGURE

		const double  POT_FULL_RANGE           =-3600.0;    // CONFIGURE
		const double  POT_OFFSET               = 3081.6;    // CONFIGURE

		double CalcTargetPotValue(double inputPotValue);
 		double CalcPOTTarget(uint targetPosition);
 		bool   GoToPotTarget(double potTarget);

		Talon               *pArmMotor;
		AnalogPotentiometer *pArmPot;

		double              targetPOTInput;
		double              targetPOTCalc;
		uint                targetPosition;
		float               targetMotorSpeed;
};

#endif
