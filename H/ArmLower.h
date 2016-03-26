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

		ArmLower(uint armMotorCh, uint armPotCh, uint armSensorCh);
		~ArmLower();

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
		bool   GetStopSensor()       const;
		double GetRatio()            const;
		double GetConstant()         const;

	private:
		const float   MOTOR_SPEED_UP           =    1.00;
		const float   MOTOR_SPEED_DOWN         =   -1.00;
		const float   ALL_STOP                 =    0.00;

		const double  INPUT_POT_FULL_FWD       =   -1.000;  // CONFIGURE
		const double  INPUT_POT_FULL_BACK      =   -0.953;  // CONFIGURE

		const double  POT_FULL_RANGE           = -264.735;  // CONFIGURE
		const double  POT_OFFSET               =  250.376;  // CONFIGURE

		const double  OUTPUT_POT_FULL_FWD      =    0.000;  // CONFIGURE
		const double  OUTPUT_POT_FULL_BACK     =  103.000;  // CONFIGURE

		const double  ARM_TOP                  =   75.0;    // CONFIGURE ROBOT POT TARGET
		const double  ARM_MIDDLE               =   50.0;    // CONFIGURE ROBOT POT TARGET
		const double  ARM_BOTTOM               =    0.0;    // CONFIGURE ROBOT POT TARGET
		const double  TARGET_TOLERANCE         =    1.0;    // CONFIGURE ROBOT POT TOLERANCE

		void   CalcTargetRatioConstant();
		double CalcOutputPOT(double inputPotValue);
		double CalcOutputPOT(uint inputPosition);
		double CalcInputPOT(double outputPotValue);
 		bool   GoToPOTTarget(double potTarget);
		void   RunArmMotor(float motorSpeed);

		Talon               *pArmMotor;
		AnalogPotentiometer *pArmPot;
		DigitalInput        *pArmStopSensor;

		double              targetRatio;         // Calculated value used to convert input POT to output POT
		double              targetConstant;      // Calculated value used to convert input POT to output POT

		double              targetPOTInput;      // Input from driver station POT
		double              targetPOTOutput;     // Calculated target robot POT reading
		uint                targetPosition;      // Input from driver station switches
		float               targetMotorSpeed;
};

#endif
