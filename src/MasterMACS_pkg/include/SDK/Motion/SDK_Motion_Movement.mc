/**
*	@file		SDK_Motion_Movement.mc
*	@brief		Functions for the simplification of movements.
*	$Revision: 39 $
*
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Motion_Movement.mh"

/**
*	@brief 		Start a movement in continuous position mode.
*	@details	The function starts a movement with a given acceleration and speed. The axis must have been switched on before.
*				The specified velocity value will be scaled by the parameters Maximum velocity VELMAX and Velocity Resolution
*				VELRES to determine the final command velocity. See Vel for more information.
* 	@param 		axis	Axis module number
* 	@param 		vel		Velocity value (negative value for reversing), Range: 1..VELRES, Command Velocity = vel * VELMAX/VELRES
* 	@param 		acc		Scaled acceleration value. Range: 1..VELRES
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkStartContinuousMove(long axis, long vel, long acc)
{
	Cvel(axis, vel);		// Set the velocity
	Acc(axis, acc);			// Set the acceleration
	AxisCvelStart(axis);	// Start constant velocity mode

	return(1);
}

/**
*	@brief 		Stop a movement in continuous position mode.
*	@details	The function stops a constant velocity movement with a given deceleration.
* 	@param 		axis	Axis module number
* 	@param 		dec		Scaled deceleration value. Range: 1..VELRES
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkStopContinuousMove(long axis, long dec)
{
	Dec(axis, dec);			// Set the deceleration
	AxisCvelStop(axis);		// Stop constant velocity mode

	return(1);
}

