/**
*	@file		SDK_Miscellaneous_IO.mc
*	@brief		Functions with IO samples.
*	$Revision: 183 $
*
*	@example PWM_1Ax_ESCON.mc
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Miscellaneous_IO.mh"


/**
*	@brief 		Setting up a PWM output with a user parameter as input source.
*	@details	This function sets all important parameters for the use of a PWM output. As input a UserParameter is used,
*				where the input source is interpreted as unsigned 16bit value. Not all MACS controllers support PWM signals.
* 	@param 		userParam		User parameter number 1-100 which sets the PWM\n
*								Input value is an unsigned 16bit value
* 	@param 		digOutPWM		Digital output number for the PWM (0-xx zero based)
* 	@param 		frequency		Frequency of the PWM output (default 1000)
* 	@param 		cycleRange		Duty cycle range\n
*								A value of 90% means a duty cycle range of 5-95 (default 90)
* 	@param 		polarity		Polarity of the reference signal (default 0)\n
*								@b 0 @b HWPWMGEN_POLARITY_POSITIVE:		Output Signal is not inverted \n
*								@b 1 @b HWPWMGEN_POLARITY_NEGATIVE:		Output Signal is inverted
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupPwmGenerator_UserParamMode(long userParam, long digOutPWM,long frequency, long cycleRange, long polarity)
{
	// To enable PWM generation, the HW Digital Output has to be switched to this mode
	HWDIGOUT_PARAM(0,HWDIGOUT_PISRC_BIT1+digOutPWM) 		= 0x400;

	// Input value is an unsigned 16bit value
	HWPWMGEN_PARAM(digOutPWM,HWPWMGEN_MODE)				= HWPWMGEN_MODE_UNSIGNED;

	// Polarity of the reference signal
	HWPWMGEN_PARAM(digOutPWM,HWPWMGEN_POLARITY)			= polarity;

	// Frequency of the PWM output
	HWPWMGEN_PARAM(digOutPWM,HWPWMGEN_FREQUENCY)		= frequency;

	// Duty cycle range and source
	HWPWMGEN_PARAM(digOutPWM,HWPWMGEN_DUTYCYCLE_RANGE)	= cycleRange;
	HWPWMGEN_PARAM(digOutPWM,HWPWMGEN_PISRC_DUTYCYCLE)	= USER_PARAM_SRCINDEX(userParam);

	print("sdkSetupPwmGenerator_UserParamMode:");
	return(1);
}

/**
*	@brief 		Setting up a PWM output with a virtual amplifier as input source.
*	@details	This function sets all important parameters for the use of a PWM output. As input a virtual amplifier is used,
*				where the input source is interpreted as signed 16bit value. Afterwards, standard ApossC move commands such as AxisCvelStart()
*				can be used. A feedback is not implemented with this function. Not all MACS controllers support PWM signals.
* 	@param 		axisNo			Axis module number
*				digOutEnable	Enable output number which is automatically set when an axis drive command is started (0-xx zero based)
* 	@param 		digOutPWM		Digital output number for the PWM (0-xx zero based)
* 	@param 		frequency		Frequency of the PWM output (default 1000)
* 	@param 		cycleRange		Duty cycle range\n
*								A value of 90% means a duty cycle range of 5-95 (default 90)
* 	@param 		polarity		Polarity of the reference signal (default 0)\n
*								@b 0 @b HWPWMGEN_POLARITY_POSITIVE:		Output Signal is not inverted \n
*								@b 1 @b HWPWMGEN_POLARITY_NEGATIVE:		Output Signal is inverted
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupPwmGenerator_AxisModeVel(long axisNo, long digOutEnable, long digOutPWM, long frequency, long cycleRange, long polarity)
{
	// To enable PWM generation, the HW Digital Output has to be switched to this mode
	HWDIGOUT_PARAM(0,HWDIGOUT_PISRC_BIT1+digOutPWM) 		= 0x400;

	// Input value is an unsigned 16bit value
	HWPWMGEN_PARAM(digOutPWM,HWPWMGEN_MODE)				= HWPWMGEN_MODE_SIGNED;

	// Polarity of the reference signal
	HWPWMGEN_PARAM(digOutPWM,HWPWMGEN_POLARITY)			= polarity;

	// Frequency of the PWM output
	HWPWMGEN_PARAM(digOutPWM,HWPWMGEN_FREQUENCY)		= frequency;

	// Duty cycle range and source
	HWPWMGEN_PARAM(digOutPWM,HWPWMGEN_DUTYCYCLE_RANGE)	= cycleRange;
	HWPWMGEN_PARAM(digOutPWM,HWPWMGEN_PISRC_DUTYCYCLE)	= VIRTAMP_PROCESS_SRCINDEX(axisNo,PO_VIRTAMP_REFVEL);

	// Set up the virtual amplifer for axis command
	// Resolution +/- 0x8000 (signed 16 bit value)
	VIRTAMP_PARAM(axisNo,VIRTAMP_REF100PERC) 				= 0x8000;
	// Automatic setting of the enable output
	VIRTAMP_PARAM(axisNo,VIRTAMP_REFOUTP)					= digOutEnable+1;
	VIRTAMP_PARAM(axisNo,VIRTAMP_REFOUTN)					= digOutEnable+1;

	// Because no feedback is processed, the tracking error must be switched off.
	AXE_PARAM(axisNo,POSERR)	= 0x7FFFFFF;

	// Position controller is bypassed, the set speed is transferred directly
	AXE_PARAM(axisNo,KPROP)		= 0;
	AXE_PARAM(axisNo,KDER)		= 0;
	AXE_PARAM(axisNo,KINT)		= 0;
	AXE_PARAM(axisNo,FFVEL)		= 1000;

	print("sdkSetupPwmGenerator_AxisModeVel:");
	return(1);
}

/**
*	@brief 		Scaling of an analog input with user units.
*	@details	This function scales an analog input signal to user specific user units. Additionally, an offset of the analog input signal can be specified.
* 	@param 		analogInputNo	Analog input number(0-xx zero based)
* 	@param 		minVoltage		Minimum input voltage (mV)
* 	@param 		maxVoltage		Maximum input voltage (mV)
* 	@param 		offsetVoltage	Voltage offest, for example, if the voltage range is 0 to 4 V, 2 V should be interpreted as 0 (mV)
* 	@param 		minValue		Scaling value minimum value to be used in the program (Uu)
* 	@param 		maxValue		Scaling value maximal value to be used in the program (Uu)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/

long sdkScaleAnalogInput(long analogInputNo, long minVoltage, long maxVoltage,long offsetVoltage, long minValue, long maxValue)
{
	double scale;

	// Scale the maximal and minimal voltage to internal value
	maxVoltage = ((double)0x8000/10000)* maxVoltage;
	minVoltage = ((double)0x8000/10000)* minVoltage;

	// Scale
	scale = ((double)maxValue-(double)minValue)/ ((double)maxVoltage-(double)minVoltage);
	if((long)scale>1)
	{
		VIRTANIN_PARAM(analogInputNo,VIRTANIN_UUFACT_UNITNO)=1000;
		VIRTANIN_PARAM(analogInputNo,VIRTANIN_UUFACT_DIGNO)=(long)(scale*1000);
	}
	else
	{
		VIRTANIN_PARAM(analogInputNo,VIRTANIN_UUFACT_UNITNO)=(long)(scale*1000);
		VIRTANIN_PARAM(analogInputNo,VIRTANIN_UUFACT_DIGNO)=1000;
	}

	// Offset
	offsetVoltage = -((double)0x8000/10000)* offsetVoltage;
	HWANIN_PARAM(analogInputNo,HWANIN_OFFSET)=offsetVoltage;

	print("sdkScaleAnalogInput: ", analogInputNo);
	return(1);
}

