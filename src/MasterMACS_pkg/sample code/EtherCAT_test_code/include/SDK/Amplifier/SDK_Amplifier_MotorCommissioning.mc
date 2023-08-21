/**
*	@file		SDK_Amplifier_MotorCommissioning.mc
*	@brief		Functions for commissioning a motor.
*	$Revision: 171 $
*
*   @example MotorCommissioning_MaxonECi40.mc
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Amplifier_MotorCommissioning.mh"

/**
*	@brief 		Current step for setting the current controller
*	@details	This function can be used to do a current step and record it. The controller parameters can then be adjusted until t
*				he desired behaviour of the current controller is achieved. The axis must be correctly parameterised before calling the function.
*				With the help of this function, the attached example and the utility monitor file {@link ut_Control_Loop_Parameter_Slider.zbm}
*				the controller can be adjusted. More information about the controller setting can be found in the ApossIDE Help.
*	@note		To adjust the current controller, the motor does not need to move. A step which only takes several ms is enough.
* 	@param 		axis			Axis module number
* 	@param 		current			Current for the step in mA
*	@param 		time			Duration of the current step in ms
* 	@param 		recordEnable	Definition record osciloscope (Only available on MACS6)\n
*								@b 0 Record Disable\n
*								@b 1 Record Enable
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/

long sdkMotorCommCurrentStep(long axis, long current, long time, long recordEnable)
{
	if (recordEnable==1)	// Settings for the recording
		{
		RecordIndex(	HWAMP_PROCESS_INDEX(axis,PO_HWAMP_CURRENT),
						AXE_PROCESS_INDEX(axis,REG_USERREFCUR));
		RecordTimeBase(RECORD_LOOP_CUR);
		RecordDest(DYNMEM);
		RecordStart(0);
	}

	// Set the controller loop
	HWAMP_PARAM(axis, HWAMP_MODE)     = HWAMP_MODE_POS_CUR;

	// Parameters for position controller -> Make sure that the position controller is not running
	AXE_PARAM(axis, KPROP)     = 0;		// position-controller  proportional factor
	AXE_PARAM(axis, KINT)      = 0;		// position-controller  integral factor
	AXE_PARAM(axis, KDER)      = 0;		// position-controller  differencial factor
	AXE_PARAM(axis, FFVEL)     = 0;  	// position-controller  Velocity Feed forward
	AXE_PARAM(axis, KFFACC)    = 0;   	// position-controller  Acceleration Feed Forward
	AXE_PARAM(axis, KFFDEC)    = 0;   	// position-controller  Deceleration Feed Forward
    AXE_PARAM(axis, POSERR) = 2000000;	// very high to avoid errors

	AxisControl(axis, USERREFCUR);		// Turn motor control on with a user defined reference current
	print("Start current step");
	Delay(50);

	AXE_PROCESS(axis,REG_USERREFCUR)=current;	//Current Step to "current" mA
	Delay(time);								//Wait for "time" ms
	AXE_PROCESS(axis,REG_USERREFCUR)=0;			//Set current to 0 mA

	Delay(50);
	print("Stop current step");
	RecordStop(0,0);

	return(1);
}

/**
*	@brief 		Velocity step for setting the velocity controller
*	@details	This function can be used to do a velocity step and record it. The controller parameters can then be adjusted until
*				the desired behaviour of the current controller is achieved. The axis must be correctly parameterised before calling the function.
*				With the help of this function, the attached example and the utility monitor file {@link ut_Control_Loop_Parameter_Slider.zbm}
*				the controller can be adjusted. More information about the controller setting can be found in the ApossIDE Help.
*	@note		To adjust the velocity controller, the motor does need to move. The speed must be achievable in the selected time.
* 	@param 		axis			Axis module number
* 	@param 		velocity		Velocity for the step, scaling is —0x4000 to 0x4000 for -100 to 100% of VELMAX
*	@param 		time			Duration of the velocity step in ms
* 	@param 		recordEnable	Definition record osciloscope (Only available on MACS6)\n
*								@b 0 Record Disable\n
*								@b 1 Record Enable
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/


long sdkMotorCommVelStep(long axis, long velocity, long time, long recordEnable)
{
	// Settings for the recording
	if (recordEnable==1)
		{
		RecordIndex(	HWAMP_PROCESS_INDEX(axis,PO_HWAMP_VELPI_ACTUAL),
						HWAMP_PROCESS_INDEX(axis,PO_HWAMP_VELPI_REF));
		RecordTimeBase(RECORD_LOOP_VEL);
		RecordDest(DYNMEM);
		RecordStart(0);
		}

	// Set the controller loop
	HWAMP_PARAM(axis, HWAMP_MODE)     = HWAMP_MODE_POS_VEL_CUR;

	// Parameters for position controller -> Make sure that the position controller is not running
	AXE_PARAM(axis, KPROP)     = 0;		// position-controller  proportional factor
	AXE_PARAM(axis, KINT)      = 0;		// position-controller  integral factor
	AXE_PARAM(axis, KDER)      = 0;		// position-controller  differencial factor
	AXE_PARAM(axis, FFVEL)     = 0;  	// position-controller  Velocity Feed forward
	AXE_PARAM(axis, KFFACC)    = 0;   	// position-controller  Acceleration Feed Forward
	AXE_PARAM(axis, KFFDEC)    = 0;   	// position-controller  Deceleration Feed Forward
	AXE_PARAM(axis, POSERR) = 2000000;	// very high to avoid errors


	AxisControl(axis, USERREFVEL);		// Turn motor control on with a user defined reference current
	print("Start velocity step");
	Delay(50);

	AXE_PROCESS(axis,REG_USERREFVEL)=velocity;	// Velcity step to "velocity"
	Delay(time);								// Wait for "time" ms
	AXE_PROCESS(axis,REG_USERREFVEL)=0;			// Set velocity to 0

	Delay(50);
	print("Stop velocity step");
	RecordStop(0,0);

	return(1);
}

/**
*	@brief 		Position ramp for setting the position controller
*	@details	This function can be used to do a positon ramp and record it. The controller parameters can then be adjusted until
*				the desired behaviour of the position controller is achieved. The axis must be correctly parameterised before calling the function.
*				With the help of this function, the attached example and the utility monitor file {@link ut_Control_Loop_Parameter_Slider.zbm}
*				the controller can be adjusted. More information about the controller setting can be found in the ApossIDE Help.
* 	@param 		axis			Axis module number
* 	@param 		distance		Relative distance in user units between start and end position
* 	@param 		controlMode		Define control typ (default 3)\n
*								@b 0 @b HWAMP_MODE_POS_VEL_CUR: 		Pos -> Vel -> Cur -> PWM \n
*								@b 1 @b HWAMP_MODE_POS_VEL:				Pos -> Vel -> PWM \n
*								@b 2 @b HWAMP_MODE_POS_CUR: 			Pos -> Cur -> PWM \n
*								@b 3 @b HWAMP_MODE_POS: 				Pos -> PWM
* 	@param 		recordEnable	Definition record osciloscope (Only available on MACS6)\n
*								@b 0 Record Disable\n
*								@b 1 Record Enable
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/


long sdkMotorCommPositionRamp(long axis, long distance, long controlMode, long recordEnable)
{
	if (recordEnable==1)	// Settings for the recording
		{
		RecordIndex(	AXE_PROCESS_INDEX(axis,REG_COMPOS),
						AXE_PROCESS_INDEX(axis,REG_ACTPOS),
						AXE_PROCESS_INDEX(axis,REG_TRACKERR));
		RecordDest(DYNMEM);
		RecordStart(0);
	}

	// Set controller loop
	HWAMP_PARAM(axis, HWAMP_MODE)     = controlMode;
    AXE_PARAM(axis, POSERR) = 2000000;	// very high to avoid errors

	AxisControl(axis, ON);				// Turn the motor on
	print("Start position ramp");
	Delay(50);

	AxisPosRelStart(axis, distance);	// Start relative movement
	AxisWaitReached(axis);				// Wait until target ist reached

	Delay(50);
	print("Stop position ramp");
	RecordStop(0,0);

	return(1);
}