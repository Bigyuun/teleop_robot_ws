/**
*	@file		SDK_Axis_Setup.mc
*	@brief		Functions for the axis setup.
*	$Revision: 192 $
*
*	@example Maxon_EC45_flat_1ax_BC_Enc.mc
*   @example Maxon_EC45_flat_1ax_BC_Hall.mc
*   @example Maxon_ECi52_1ax_SC_SSI.mc
*   @example Maxon_EC45_flat_1ax_SC_Hall_Inc.mc
*   @example Maxon_ECi40_1ax_SC_OL_Inc.mc
*   @example Maxon_ECi40_3ax_SC_OL_Inc.mc
*   @example Maxon_RE_40_1ax_Inc.mc
*   @example Maxon_RE_40_1ax_OL.mc
*   @example Stepper_XY_CL.mc
*   @example Stepper_XY_OL.mc
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Axis_Setup.mh"

/**
*	@brief 		Set parameters for PID position control loop
*	@details	The function sets the basic factors of the position PID controller. With the function @ref sdkSetupPositionPIDControlExt()
*				the extended factors of the position controller can be set. More information are available in the Help of ApossIDE in the topic "Control Loop Design".
* 	@param 		axis			Axis module number
* 	@param 		kprop			Proportional value for PID position control loop (default 30)
* 	@param 		kint			Integral value for PID position control loop (default 0)
* 	@param 		kder			Derivative value for PID position control loop (default 0)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupPositionPIDControl(long axis, long kprop, long kint, long kder)
{
	// Parameters for position controller
	AXE_PARAM(axis, KPROP)     	= kprop;  	// position-controller  proportional factor:       1 … 65000
	AXE_PARAM(axis, KINT)      	= kint;    	// position-controller  integral factor:           0 … 65000
	AXE_PARAM(axis, KDER)      	= kder;  	// position-controller  differencial factor:       0 … 65000

	return(1);
}

/**
*	@brief 		Set extended parameters for PID position control loop
*	@details	The function sets the extended factors of the position PID controller. With the function @ref sdkSetupPositionPIDControl()
*				the basic factors of the position controller can be set. More information are available in the Help of ApossIDE in the topic "Control Loop Design".
* 	@param 		axis			Axis module number
* 	@param 		kprop			Proportional value for PID position control loop (default 30)
* 	@param 		kint			Integral value for PID position control loop (default 0)
* 	@param 		kder			Derivative value for PID position control loop (default 0)
* 	@param 		kilim			Limit value for the integral sum of the PID position control loop (default 1000)
* 	@param 		kilimtime		Time used to increase or decrease the integral limit (default 0)
* 	@param 		bandwidth		Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint (default 1000)
* 	@param 		ffvel			Velocity Feed forward (default 0)
* 	@param 		kffacc			Acceleration Feed forward (default 0)
* 	@param 		kffdec			Deceleration Feed Forward (default 0)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupPositionPIDControlExt(long axis, long kprop, long kint, long kder, long kilim, long kilimtime, long bandwidth, long ffvel, long kffacc, long kffdec)
{
	// Parameters for position controller
	AXE_PARAM(axis, KPROP)     	= kprop;  	// position-controller  proportional factor:       			1 … 65000
	AXE_PARAM(axis, KINT)      	= kint;    	// position-controller  integral factor:          			0 … 65000
	AXE_PARAM(axis, KDER)      	= kder;  	// position-controller  differencial factor:       			0 … 65000

	AXE_PARAM(axis, KILIM)     	= kilim;  	// position-controller  limits the integral sum:       		0 … 65000
	AXE_PARAM(axis, KILIMTIME) 	= kilimtime;// position-controller  Time used to for integral limit:    -10000…10000
	AXE_PARAM(axis, BANDWIDTH)	= bandwidth;// position-controller  Bandwidth of the PID filter:       	0 … 2000
	AXE_PARAM(axis, FFVEL)     	= ffvel;  	// position-controller  Velocity Feed forward:       		0…100000
	AXE_PARAM(axis, KFFACC)     = kffacc;   // position-controller  Acceleration Feed Forward:          0 … 8000
	AXE_PARAM(axis, KFFDEC)     = kffdec;   // position-controller  Deceleration Feed Forward:          0 … 8000

	return(1);
}

/**
*	@brief 		Setup to convert the resolution of the machine in user units.
*	@details	The basic resolution of the entire machine is 1 encoder quadcount. Using the above formula, this can be converted to
*				the resolution of the machine in user units. This is the highest precision to which an application may specify positions.
*				For example, if 1 uu equals 10 qc, then the command "AxisPosRelStart(0,1);" will move the motor foward by 10 qc;
*				it will not be possible for the application program to move the motor by only 5 qc. Conversely, if the motor moves forward
*				by only 1 qc, then the Apos command is likely to return the same actual position value even though the motor has moved to a
*				new position; the application program will be unable to differentiate between these two motor positions.
*				More information are available in the Help of ApossIDE: "How to Set Up Resolution and Gear Parameters".
* 	@param 		axis			Axis module number
* 	@param 		posencrev		Number of revolutions of the motor
* 	@param 		posencqc		Number of quadcounts in POSENCREV revolutions
* 	@param 		posfact_z		Number of revolutions of the input shaft
*	@param		posfact_n		Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
*	@param		feedrev 		Number of revolutions of the gear box output shaft
*	@param		feeddist 		Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/

long sdkSetupAxisUserUnits(long axis, long posencrev, long posencqc, long posfact_z, long posfact_n, long feedrev, long feeddist)
{
    AXE_PARAM(axis,POSENCREV) = posencrev;		//	Number of revolutions of the motor
    AXE_PARAM(axis,POSENCQC)  = posencqc;		//	Number of quadcounts in POSENCREV revolutions
    AXE_PARAM(axis,POSFACT_Z) = posfact_z;		//	Number of revolutions of the input shaft
    AXE_PARAM(axis,POSFACT_N) = posfact_n;		//	Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
    AXE_PARAM(axis,FEEDREV)   = feedrev;		//	Number of revolutions of the gear box output shaft
    AXE_PARAM(axis,FEEDDIST)  = feeddist;		//	Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft

    return(1);
}


/**
*	@brief 		Defines parameters which are required for the calculation of ramps and velocities.
*	@details	The function sets all basic parameters which are needed for the calculation of the ramps and velocities.
* 	@param 		axis		Axis module number
* 	@param 		velres		Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands (default 100)
* 	@param 		maxRpm		Defines the rated speed of the drive. This value is listed in RPM (default 1000)
* 	@param 		ramptype	Defines the ramptype (default 0)\n
*							@b 0 @b RAMPTYPE_TRAPEZ: Trapezoid ramp\n
*							@b 2 @b RAMPTYPE_JERKLIMITED:  Jerk limited ramp
* 	@param 		rampmin		Maximum acceleration (default 1000)
* 	@param 		jerkmin		Minimum time (ms) required before reaching the maximum acceleration (default 100)\n
*							just usefull for ramptype "RAMPTYPE_JERKLIMITED"
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupAxisMovementParam(long axis, long velres, long maxRpm, long ramptype, long rampmin, long jerkmin)
{
     AXE_PARAM(axis, VELRES)	= velres;	// Velocity resolution
     AXE_PARAM(axis, VELMAX) 	= maxRpm;	// Rated speed of drive
     AXE_PARAM(axis, RAMPTYPE) 	= ramptype;	// Ramp type
     AXE_PARAM(axis, RAMPMIN)  	= rampmin;	// Maximum acceleration
     AXE_PARAM(axis, JERKMIN) 	= jerkmin;	// Minimum time required before reaching the maximum acceleration

	return(1);
}

/**
*	@brief 		Definition of the jerk limited extended parameter.
*	@details	Function to set the extended jerk limited parameters. This topic is described in the ApossIDE Help on the page "Limited Jerk".
*				If the parameters JERKMIN2 - 4 are set to the value 0, the jerk limit is taken from the parameter JERKMIN.
* 	@param 		axis		Axis module number
* 	@param 		jerkmin1	Defines the minimum time [ms] required before reaching the maximum acceleration (default 100)
* 	@param 		jerkmin2	Defines the minimum time [ms] required to ramp the acceleration down from maximum acceleration to 0 (default 0)
* 	@param 		jerkmin3	Defines the minimum time [ms] required to ramp the deceleration up from 0 to maximum deceleration (default 0)
* 	@param 		jerkmin4	Defines the minimum time [ms] required to ramp the deceleration down from maximum deceleration to 0 (default 0)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupAxisJerkLimited(long axis, long jerkmin1, long jerkmin2, long jerkmin3, long jerkmin4)
{
     AXE_PARAM(axis, RAMPTYPE)	= RAMPTYPE_JERKLIMITED;		// Ramp type: Jerk limited
     AXE_PARAM(axis, JERKMIN) 	= jerkmin1;
     AXE_PARAM(axis, JERKMIN2) 	= jerkmin2;
     AXE_PARAM(axis, JERKMIN3)  = jerkmin3;
     AXE_PARAM(axis, JERKMIN4) 	= jerkmin4;

	return(1);
}


/**
*	@brief 		Definition of the rotation direction of the axis
*	@details	The application defines what is a forward and backward movement of the motor. With the help of this function this definition can be adjusted.
* 	@param 		axis		Axis module number
* 	@param 		posdrct		Parameter to set the rotation direction (default 0)\n
*							@b 1 @b Normal direction\n
*							@b -1 @b Invert direction
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupAxisDirection(long axis, long posdrct)
{
	// Set rotation direction
	if (posdrct==1 || posdrct==-1 )
    {
    	VIRTAMP_PARAM(axis,VIRTAMP_INVERT)=posdrct;
		VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_UUFACT_UNITNO)=posdrct;
    }
    else	// Error
    {
    	print("SDK Error: not supported rotation direction");
    	return(-1);
    }

	return(1);
}

/**
*	@brief 		Setting up a software axis limitation in user units
*	@details	For additional safety a software axis limit can be set. This function is for setting the limits and must
*				be activated with the function @ref skdEnableAxisSoftwareLimit(). The limit is set in user units.
* 	@param 		axis		Axis module number
* 	@param 		negLimitUu	Negative axis limit [Uu]\n
* 	@param 		posLimitUu	Positive axis limit [Uu]\n
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long skdSetupAxisSoftwareLimit( long axis, long negLimitUu, long posLimitUu)
{
	AXE_PARAM(axis,NEGLIMIT) 		=	negLimitUu * ((double)(AXE_PARAM(axis,FEEDREV)*AXE_PARAM(axis,POSFACT_Z)*AXE_PARAM(axis,POSENCQC))/((double)AXE_PARAM(axis,FEEDDIST)*AXE_PARAM(axis,POSFACT_N)*AXE_PARAM(axis,POSENCREV)));
	AXE_PARAM(axis,POSLIMIT) 		=	posLimitUu * ((double)(AXE_PARAM(axis,FEEDREV)*AXE_PARAM(axis,POSFACT_Z)*AXE_PARAM(axis,POSENCQC))/((double)AXE_PARAM(axis,FEEDDIST)*AXE_PARAM(axis,POSFACT_N)*AXE_PARAM(axis,POSENCREV)));
	print("Set negative software limit to ", AXE_PARAM(axis,NEGLIMIT), " qc");
	print("Set positive software limit to ", AXE_PARAM(axis,POSLIMIT), " qc");
	return(1);
}

/**
*	@brief 		Activation of the axis software limits.
*	@details	This function enables or disables the software limits set with the xy function @ref skdSetupAxisSoftwareLimit().
* 	@param 		axis		Axis module number
* 	@param 		active		Parameter to enable / disable the software limit\n
*							@b 0 @b Disable software limit\n
*							@b 1 @b Enable software limit
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/


long skdEnableAxisSoftwareLimit( long axis, long active)
{
	if(active == 0)
	{
		AXE_PARAM(axis,SWPOSLIMACT) 	=	active;
		AXE_PARAM(axis,SWNEGLIMACT) 	=   active;
		print("Disable software limit axis ", axis);
	}
	else
	{
		AXE_PARAM(axis,SWPOSLIMACT) 	=	1;
		AXE_PARAM(axis,SWNEGLIMACT) 	=   1;
		print("Enable software limit axis ", axis);
	}

	return(1);
}

