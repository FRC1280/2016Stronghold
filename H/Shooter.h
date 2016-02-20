#include "../H/Loader.h"
#ifndef SHOOTER_H
#define SHOOTER_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE Shooter CLASS
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

class Shooter
{
	public:

		Shooter(uint loadMotorCh, uint loadBannerCh, Loader *pRobotLoader);
		~Shooter();

		bool   ShootBall();
		float  GetMotorSpeed() const;
		int    GetShooterCounter() const;
		bool   GetBannerSensor() const;
		void   RunShooter();
		void   StopShooter();

		//Change code - add limit switch to detect loaded ball in shooter

	private:
		const float   MOTOR_SPEED_SHOOT			 = 	  0.25;
		const float   ALL_STOP                   =    0.00;

		bool 		  firstLoop   				 =    true;
		bool      	  prevBannerValue			 =    false;
		bool		  shootBannerSensor			 =    false;
		bool 		  shooterReady				 =    false;
		bool          ballLoaded                 =    false;

		void   RunShooter(float motorSpeed);

		Spark               *pShooterMotor;
		DigitalInput        *pBannerSensor;
		Loader              *pBallLoader;
};

#endif
