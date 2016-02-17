#ifndef CLIMBER_H
#define CLIMBER_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE Climber CLASS
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

class Climber
{
	public:

		Climber(uint climbMotor1Ch, uint climbMotor2Ch);
		~Climber();

		void   Climb();
		void   Lower();
		float  GetMotor1Speed() const;
		float  GetMotor2Speed() const;
		void   StopClimber();

	private:
		const float   MOTOR_SPEED_CLIMB			 = 	  0.25;
		const float   MOTOR_SPEED_LOWER			 = 	 -0.25;
		const float   ALL_STOP                   =    0.00;

		Talon        *pClimberMotor1;
		Talon        *pClimberMotor2;
};
#endif
