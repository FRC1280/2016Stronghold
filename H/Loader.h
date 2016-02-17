#ifndef LOADER_H
#define LOADER_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE Loader CLASS
//------------------------------------------------------------------------------
// Positions the Loader based on a target position.  The target position
// translates into a target potentiometer reading.
//------------------------------------------------------------------------------
class Loader
{
	public:

		Loader(uint loadMotorCh, uint loadBannerCh);
		~Loader();

		bool   LoadBall();
		bool   EjectBall();
		bool   LoadToShooter();
		float  GetMotorSpeed() const;
		bool   GetBannerSensor() const;
		int    GetEjectCounter() const;

	private:
		const float   MOTOR_SPEED_LOAD           =    0.25;   // CONFIGURE
		const float   MOTOR_SPEED_SHOOT			 = 	  0.75;
		const float   MOTOR_SPEED_EJECT          =   -0.25;   // CONFIGURE
		const float   ALL_STOP                   =    0.00;
		const int 	  EJECT_LOOPS				 =   50;
		const int     SHOOT_LOOPS				 =   25;

		bool 		  prevBannerValue			 = 	  false;
		bool 		  firstLoop   				 =    true;
		bool 		  firstShootLoop			 = 	  true;
		int 		  ejectCounter 				 =    0;
		int 		  shootCounter				 =    0;

		void RunLoader(float motorSpeed);
		void StopLoader();

		Spark               *pLoaderMotor;
		DigitalInput        *pBannerSensor;

};

#endif
