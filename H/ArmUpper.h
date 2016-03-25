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

		enum  target {kTop, kMiddle, kBottom};

		ArmUpper(uint armMotorCh, uint armPotCh);
		~ArmUpper();

		bool   MoveArmPOTInput(double inputTarget);
		bool   MoveArmPositionInput(uint inputTarget);
		void   MoveArmUp();
		void   MoveArmDown();
		void   StopArm();
		double GetTargetPOTInput()   const;
		uint   GetTargetPosition()   const;
		double GetTargetPOTOutput()  const;
		double GetCurrentPosition()  const;
		float  GetTargetMotorSpeed() const;
		float  GetMotorSpeed()       const;
		double GetRatio()            const;
		double GetConstant()         const;

	private:
		const float   MOTOR_SPEED_UP           =   -1.00;
		const float   MOTOR_SPEED_DOWN         =    1.00;
		const float   ALL_STOP                 =    0.00;

		const double  INPUT_POT_FULL_FWD       =   -1.000;   // CONFIGURE
		const double  INPUT_POT_FULL_BACK      =    1.000;   // CONFIGURE

		const double  POT_FULL_RANGE           =  3600.0;    // CONFIGURE
		const double  POT_OFFSET               = -1852.1;    // CONFIGURE

		const double  OUTPUT_POT_FULL_FWD      =    6.000;   // CONFIGURE
		const double  OUTPUT_POT_FULL_BACK     =  355.000;   // CONFIGURE

		const double  ARM_TOP                  =    25.0;    // CONFIGURE
		const double  ARM_MIDDLE               =    50.0;    // CONFIGURE
		const double  ARM_BOTTOM               =    75.0;    // CONFIGURE
		const double  TARGET_TOLERANCE         =     4.0;

		void   CalcTargetRatioConstant();
		double CalcOutputPOT(double inputPotValue);
		double CalcOutputPOT(uint inputPosition);
		double CalcInputPOT(double outputPotValue);
 		bool   GoToPOTTarget(double potTarget);
 		void   RunArmMotor(float motorSpeed);

		Talon               *pArmMotor;
		AnalogPotentiometer *pArmPot;

		double              targetRatio;    // Calculated value used to convert input POT to output POT
		double              targetConstant; // Calculated value used to convert input POT to output POT

		double              targetPOTInput;
		double              targetPOTOutput;
		uint                targetPosition;
		float               targetMotorSpeed;
};

#endif
