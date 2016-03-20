#ifndef ARMSEGMENT_H
#define ARMSEGMENT_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE ArmSegment CLASS
//------------------------------------------------------------------------------
// Positions the ArmSegment based on a target position.  The target position
// translates into a target potentiometer reading.
//------------------------------------------------------------------------------
class ArmSegment
{
	public:

	    enum limitLocation {kNone, kUpperLimit, kLowerLimit};

		ArmSegment(uint armMotorCh, uint armPotCh);
		~ArmSegment();

		void   DefineLimitSwitch(uint limitCh, uint limitLocation);
		void   SetInputPotRange(double minPotValue, double maxPotValue);
		void   SetOutputPotRange(double minPotValue, double maxPotValue);
		void   SetTargetPotRange(double minPotValue, double maxPotValue);
		bool   MoveArm(double inputTarget);
		double CalcPOTTargetInput(double inputPotValue);
		float  GetTargetMotorSpeed()              const;
		float  GetMotorSpeed()                    const;
		double GetCurrentPosition()               const;
		double GetPositionTargetInput()           const;
		double GetPositionTargetCalc()            const;
		bool   GetLimitSwitch(uint limitLocation) const;
		double GetRatio()                         const;
		double GetConstant()                      const;

	private:
		const float   MOTOR_SPEED_EXTEND       =    0.20;   // CONFIGURE
		const float   MOTOR_SPEED_CONTRACT     =   -0.20;   // CONFIGURE
		const float   ALL_STOP                 =    0.00;
		const double  TARGET_TOLERANCE         =    5.00;   // CONFIGURE

		void   CalcPOTRangeOffset(double minPotValue, double maxPotValue);
		void   CalcTargetRatioConstant();
 		double CalcPOTTarget(double inputPotValue);
 		bool   GoToPotTarget(double potTarget);
 		bool   GetUpperLimitSwitch() const;
 		bool   GetLowerLimitSwitch() const;

		Talon               *pArmMotor;
		AnalogPotentiometer *pArmPot;
		DigitalInput        *pUpperLimitHit;
		DigitalInput        *pLowerLimitHit;

		uint                armPOTChannel;
		double              inputPOTLowerLimit;
		double              inputPOTUpperLimit;

		double              outputPOTLowerLimit;
		double              outputPOTUpperLimit;

		double              targetPOTLowerLimit;
		double              targetPOTUpperLimit;

		double              outputPOTFullRange;
		double              outputPOTOffset;

		double              targetRatio;
		double              targetConstant;

		bool                upperLimitSw;
		bool                lowerLimitSw;

		uint                targetInput;

		double              armTargetInput;
		double              armTargetCalc;

		float               targetMotorSpeed;
};

#endif
