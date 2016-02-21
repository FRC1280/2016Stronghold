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

		Loader(uint loadMotorCh, uint loadedSensorCh, uint inShooterSensorCh);
		~Loader();

		bool   LoadBall();
		bool   EjectBall();
		bool   LoadToShooter();
		float  GetMotorSpeed()        const;
		bool   GetLoadedSensor()      const;
		bool   GetBallLoaded()        const;
		bool   GetFirstEjectLoop()    const;
		int    GetEjectCounter()      const;
		bool   GetBallEjected()       const;
		bool   GetShooterSensor()     const;
		bool   GetBallInShooter()     const;

	private:
		const float   MOTOR_SPEED_LOAD           =   -0.25;   // CONFIGURE
		const float   MOTOR_SPEED_EJECT          =    0.25;   // CONFIGURE
		const float   MOTOR_SPEED_SHOOT			 = 	 -0.75;   // CONFIGURE
		const float   ALL_STOP                   =    0.00;
		const int 	  MAX_EJECT_LOOPS	    	 =   50;      // CONFIGURE

		// Load Ball
		bool  ballLoaded;
		// Eject Ball
		bool  ballEjected;
		bool  firstEjectLoop;
		int   ejectCounter;
		// Load Ball in Shooter
		bool  ballInShooter;

		void RunLoader(float motorSpeed);
		void StopLoader();

		Spark               *pLoaderMotor;
		DigitalInput        *pLoadedSensor;
		DigitalInput        *pInShooterSensor;
};

#endif
