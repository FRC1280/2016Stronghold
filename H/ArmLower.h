#ifndef ARMLOWER_H
#define ARMLOWER_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE Arm CLASS
//------------------------------------------------------------------------------
// Positions the Arm based on a target position.  The target position
// translates into a target potentiometer reading.
//------------------------------------------------------------------------------
class ArmLower
{
	public:

		enum  target {kTop, kMiddle, kBottom};

		ArmLower(uint armMotorCh, uint armPotCh);
		~ArmLower();

		void   MoveArmPOTInput(float inputTarget);
		bool   MoveArmPositionInput(uint inputTarget);
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
		const float   MOTOR_SPEED_UP           =    1.00;
		const float   MOTOR_SPEED_DOWN         =   -1.00;
		const float   ALL_STOP                 =    0.00;

		const double  ARM_TOP                  =   25.0;    // CONFIGURE ROBOT POT TARGET
		const double  ARM_MIDDLE               =   50.0;    // CONFIGURE ROBOT POT TARGET
		const double  ARM_BOTTOM               =   75.0;    // CONFIGURE ROBOT POT TARGET
		const double  TARGET_TOLERANCE         =    1.0;    // CONFIGURE ROBOT POT TOLERANCE

		const double  POT_FULL_RANGE           =  194.580;  // CONFIGURE
		const double  POT_OFFSET               =   -0.308;  // CONFIGURE

		void   RunArmMotor(float motorSpeed);
		double CalcTargetPotValue(double inputPotValue);
 		double CalcPOTTarget(uint targetPosition);
 		bool   GoToPotTarget(double potTarget);

		Talon               *pArmMotor;
		AnalogPotentiometer *pArmPot;

		double              targetPOTInput;      // Input from driver station POT
		double              targetPOTCalc;       // Calculated target robot POT reading
		uint                targetPosition;      // Input from driver station switches
		float               targetMotorSpeed;
};

#endif
