/**
*	@file		SDK_Amplifier_MotorAlignment.mc
*	@brief		Functions for aligning a motor.
*	$Revision: 192 $
*
*   @example Stepper_XY_CL.mc
*   @example Maxon_ECi40_1ax_SC_OL_Inc.mc
*   @example Maxon_ECi40_3ax_SC_OL_Inc.mc
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Amplifier_MotorAlignment.mh"

/**
*	@brief 		The purpose of this function is to find the rotor position of a brushless motor in relation to the encoder position feedback.
*	@details	The function will start a program in the amplifier, which will apply a magnetic field to the rotor.
*				The rotor will then align to this field. The field will then be varied a bit back and forth to commit the measurement.
*				To use this function the maximum current of the motor must also be specified.
*	@note		In order for this procedure to work, the motor must not be braked or be in a limit position.
*				The rotor must be able to rotate freely for about half a revolution.
* 	@param 		axis			Axis module number
* 	@param 		mode			Mode of aligment \n
*								@b 1: For brushless motors \n
*								@b 3: For stepper motors
*	@param 		maxCurrent		Maximum current of the motor
*	@param 		alignCurrent	Current for the alignment function
*	@return 	value:	Command Reference, MotorAlingStatus() \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkMotorAlignment(long axis, long mode, long maxCurrent, long alignCurrent)
{
	long retVal;

	HWAMP_PARAM((axis), HWAMP_MAXCUR)   = alignCurrent; 	// Set the maximum current for aligning the motor

	print("AxisNo ", axis, " Start position detection");
	MotorAlignStart(axis, mode); 							// Start motor aliginment

	while((retVal = MotorAlignStatus(axis)) == 0)			// Wait until the alignment is successful or failed
	{
		print("Aligning... ", retVal);
		Delay(500);
	}
	if(retVal < 0)
	{
		print("!!!! Rotor position detection FAILED !!!! retVal: ", retVal);
	}
	else if (retVal > 0)
	{
		print("Alignment completed with ", retVal);
	}
	HWAMP_PARAM((axis), HWAMP_MAXCUR)   = maxCurrent;		// Set the maximum current for the motor

	return(retVal);
}


/**
*	@brief 		The purpose of this function is to find the rotor position of a brushless motor in relation to the encoder position feedback.
*	@details	The function is the same as function @ref sdkMotorAlignment() - but several motors can be started in simultaneous. The same current is set for each axis.
*	@note		In order for this procedure to work, the motor must not be braked or be in a limit position.
*				The rotor must be able to rotate freely for about half a revolution.
* 	@param 		axis			Axis module number (lowest axis)
* 	@param 		axisNumber		Number axes. These must be lined up and increased by one for each axis
* 	@param 		mode			Mode of aligment \n
*								@b 1: For brushless motors \n
*								@b 3: For stepper motors
*	@param 		maxCurrent		Maximum current of the motor
*	@param 		alignCurrent	Current for the alignment function
*	@return 	value:	Command Reference, MotorAlingStatus() \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkMotorMultiAlignment(long axis, long axisNumber, long mode, long maxCurrent, long alignCurrent)
{
	long retVal[6]={0,0,0,0,0,0},retValCheck[6]={0,0,0,0,0,0}, retValSum=0, i;

	axisNumber=axis+axisNumber;

	for (i=axis;i<axisNumber;i++)
	{
		AxisControl(i,OFF);
		HWAMP_PARAM((i), HWAMP_MAXCUR)   = alignCurrent; 	// Set the maximum current for aligning the motor
		print("AxisNo ", i, " Start position detection");
		MotorAlignStart(i, mode);
		Delay(250);
	}
	print("AxisNumber ", axisNumber - axis);

	while(!(retValSum==(axisNumber-axis)))
	{
		retValSum=0;
		for (i=axis;i<axisNumber;i++)
		{
			retVal[i] = MotorAlignStatus(i);
			printf("Aligning Axis: %ld, Status: %ld", i, retVal[i]);
			print("");

			if(retVal[i] < 0)
			{
				print("!!!! Rotor position detection FAILED !!!! retVal: ", retVal[i]);
				print("");
				return(-1);
			}
			else if (retVal[i] > 0)
			{
				printf("Aligning Axis: %ld completed with: %ld", i, retVal[i]);
				print("");
				retValCheck[i]=1;
			}
			retValSum=retValCheck[i]+retValSum;
		}
		Delay(1000);
	}
	for (i=axis;i<=axisNumber;i++)
	{
		HWAMP_PARAM((i), HWAMP_MAXCUR)   = maxCurrent;		// Set the maximum current for the motor
	}

	return(1);
}

/**
*	@brief 		This function sets the offset of the electric field in relation to an absolute encoder.
*	@details	If an absolute encoder is used and the offset between rotor/ encoder is already known,
*				the classic motor alignment can be dispensed with. The offset and the polarity can be
*				determined with the function @ref sdkMotorAlignment() during commissioning.
* 	@param 		axis		Axis module number
* 	@param 		elPol		Encoder polarity vs. electrical polarity (default -1)\n
*							@b 1 @b HWAMP_ELPOL_REGULAR: 		Electrical polarity is equal encoder's polarity \n
*							@b -1 @b HWAMP_ELPOL_INVERS: 		Electrical polarity is inverse to encoder's polarity \n
*	@param 		poselOffset	Sets the offsets for the aligned absolute encoders.
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetMotorAlignmentOffset(long axis, long elPol, long poselOffset)
{
	HWAMP_PARAM(axis,HWAMP_ELPOL) 			= elPol;
	HWAMP_PARAM(axis, HWAMP_POSEL_OFFSET) 	= poselOffset;

	return(1);
}
