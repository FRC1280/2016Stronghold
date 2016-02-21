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
		void   RunShooter();
		void   StopShooter();
		float  GetMotorSpeed()       const;
		bool   GetBannerSensor()     const;
		bool   GetShooterFirstLoop() const;
		bool   GetPrevShooterReset() const;
		bool   GetShooterReset()     const;
		bool   GetBallInShooter()    const;

	private:
		const float   MOTOR_SPEED_SHOOT			 = 	  1.00;
		const float   ALL_STOP                   =    0.00;

		bool 		  firstLoop   				 =    true;
		bool      	  prevBannerValue			 =    false;
		bool 		  shooterReset				 =    true;
		bool          ballInShooter          =    false;

		void   RunShooter(float motorSpeed);

		Spark               *pShooterMotor;
		DigitalInput        *pBannerSensor;
		Loader              *pBallLoader;
};

#endif
