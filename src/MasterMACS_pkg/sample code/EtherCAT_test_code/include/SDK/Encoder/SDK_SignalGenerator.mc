/**
*	@file		SDK_SignalGenerator.mc
*	@brief		Functions for the operation of the signal generator.
*	$Revision: 210 $
*
*	@example	SetupSignalGeneratorAxis.mc
*	@example	SetupSignalGeneratorOpenloop.mc
*/

#pragma once

#include <SysDef.mh>
#include "SDK_SignalGenerator.mh"

#ifndef HWSIGGEN_MODE_ENABLE
	#define HWSIGGEN_MODE_ENABLE 0x0001
#endif

/**
*	@brief 		Settings for a Signal Generator
*	@details	With this function the settings can be made which are required for a signal generator.
*				This is a closed loop signal generator which can generate encoder signals by running
*               a virtual axis.
*               The signal generator can also generate index signals with a defined index distance. This signal
*               will be generated on a digital output. For signal generator 0 this will be digital output 1, for
*               signal generator 1 digital output 2 and so on. The digital output can then not be controlled by
*               normal output functions anymore.
*
*				The generated signal cannot be read back. A encoder signal connection to a different port
*               has to be made to receive the generated signals.
*
* 	@param 		axis			Axis number 0-n
* 	@param 		encPortOut		Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
* 	@param 		indexDistance	Distance in increments between two index signals.

*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSigGenAxisSetupMiniMACS6(long axis, long encPortOut, long indexDistance)
{
	long virtMasterNo = encPortOut;

	if(SYS_INFO(SYS_MAX_SIGGEN) == 0)
	{
		print("Controller does not support Signal Generator");
		return(-1);
	}

	if((encPortOut+1) > SYS_INFO(SYS_MAX_SIGGEN))
	{
		print("Port does not support Signal Generator");
		return(-2);
	}

	if(SYS_INFO(SYS_MAX_VIRTMAST) == 0)
	{
		print("Controller does not support Virtual Masters");
		return(-3);
	}

	if((encPortOut+1) > SYS_INFO(SYS_MAX_VIRTMAST))
	{
		print("Port does not support Virtual Masters");
		return(-4);
	}

	// Setup selected encoder port as signal generator
	HWENC_PARAM(encPortOut, HWENCODER_MODE) = HWENCODER_MODE_INCROUTPUT;  	// signal generator mode

	HWSIGGEN_PARAM(encPortOut, HWSIGGEN_SIGDIST) = indexDistance;			// setup index distance

	VIRTMAST_PARAM(encPortOut, VIRTMAST_UUFACT_UNITNO)	= 0xFFFFF;
	// Full reference value of REG_REFERENCE is 0xFFFFF
	// Virtual master works in 1/1000 Hz on each line (Spur)
	// This results in factor 1000/4 = 250
	// REG_REFERENCE is scaled with 65'536
	// 250 % 65.536 = 3.814697265625
	VIRTMAST_PARAM(encPortOut, VIRTMAST_UUFACT_INCNO) = AXE_PARAM(axis, VELMAXQC) * 3.814697265625;

	// set velocity source to axis cmdvel value
	VIRTMAST_PARAM(encPortOut, VIRTMAST_PISRC_CMDVEL) = AXE_PROCESS_SRCINDEX(axis, REG_REFERENCE);
	AXE_PARAM(axis, FFVEL) = 1000;
	AXE_PARAM(axis, KPROP) = 100;
	AXE_PARAM(axis, KDER) = 0;

	// scale to 1/1000 HZ (Pulses)
	VIRTMAST_PARAM(encPortOut, VIRTMAST_MODE) = VIRTMAST_MODE_VELOCITY;

	HWSIGGEN_PARAM(encPortOut, HWSIGGEN_MODE) = HWSIGGEN_MODE_ENABLE;		// enable signal generator

	return(1);
}

/**
*	@brief 		Settings for a Signal Generator
*	@details	With this function the settings can be made which are required for a signal generator.
*				This is an open loop signal generator which can generate an encoder signal with a defined
*               signal speed and and with a ramp of defined acceleration and deceleration.
*               The signal generator can also generate index signals with a defined index distance. This signal
*               will be generated on a digital output. For signal generator 0 this will be digital output 1, for
*               signal generator 1 digital output 2 and so on. The digital output can then not be controlled by
*               normal output functions anymore.
*
*				The generated signal cannot be read back. A encoder signal connection to a different port
*               can be made to receive the generated signals.
*
* 	@param 		encPort			Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
* 	@param 		indexDistance	Distance in increments between two index signals.

*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSigGenOpenloopSetupMiniMACS6(long encPort, long indexDistance)
{
	long virtMasterNo = encPort;

	if(SYS_INFO(SYS_MAX_SIGGEN) == 0)
	{
		print("Controller does not support Signal Generator");
		return(-1);
	}

	if((encPort+1) > SYS_INFO(SYS_MAX_SIGGEN))
	{
		print("Port does not support Signal Generator");
		return(-2);
	}

	if(SYS_INFO(SYS_MAX_VIRTMAST) == 0)
	{
		print("Controller does not support Virtual Masters");
		return(-3);
	}

	if((encPort+1) > SYS_INFO(SYS_MAX_VIRTMAST))
	{
		print("Port does not support Virtual Masters");
		return(-4);
	}

	// Setup selected encoder port as signal generator
	HWENC_PARAM(encPort, HWENCODER_MODE) = HWENCODER_MODE_INCROUTPUT;  	// signal generator mode

	HWSIGGEN_PARAM(encPort, HWSIGGEN_SIGDIST) = indexDistance;			// setup index distance

	VIRTMAST_PARAM(encPort, VIRTMAST_UUFACT_UNITNO) = 1;
	VIRTMAST_PARAM(encPort, VIRTMAST_UUFACT_INCNO) = 1;
	VIRTMAST_PARAM(encPort, VIRTMAST_MODE) = VIRTMAST_MODE_PROFILE;

	HWSIGGEN_PARAM(encPort, HWSIGGEN_MODE) = HWSIGGEN_MODE_ENABLE;		// enable signal generator

	return(1);
}

/**
*	@brief 		Enable/Disable Signal Generator
*	@details	Disabling and Enabling the Signal Generator will reset the index distance counter. This means, an
*               index signal will immediately generated. The next signal will be generated after index distance.
* 	@param 		encPort			Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
* 	@param 		enable			1 = enable, 0 = disable
*
*/
void sdkSigGenEnable(long encPort, long enable)
{
	if(enable)
		HWSIGGEN_PARAM(encPort, HWSIGGEN_MODE) = HWSIGGEN_MODE_ENABLE;		// enable signal generator
	else
		HWSIGGEN_PARAM(encPort, HWSIGGEN_MODE) = 0;							// disable signal generator
}

/**
*	@brief 		Reset Signal Generator
*	@details	This function will reset the index distance counter. This means, an index signal will immediately
*               generated. The next signal will be generated after index distance.
* 	@param 		encPort			Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual

*/
void sdkSigGenReset(long encPort)
{
	HWSIGGEN_PARAM(encPort, HWSIGGEN_MODE) = 0;							// disable signal generator
	HWSIGGEN_PARAM(encPort, HWSIGGEN_MODE) = HWSIGGEN_MODE_ENABLE;		// enable signal generator
}

/**
*	@brief 		Set acceleration of Signal Generator
*	@details
* 	@param 		encPort			Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
* 	@param 		acceleration	The acceleration of the generated signal [Hz/s]
*
*/
void sdkSigGenOpenloopAcceleration(long encPort, long acceleration)
{
	VIRTMAST_PARAM(encPort, VIRTMAST_ACC) = acceleration; // acc is in Hz/s
}

/**
*	@brief 		Set acceleration of Signal Generator
*	@details
* 	@param 		encPort			Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
* 	@param 		acceleration	The acceleration of the generated signal [Hz/s]
*
*/
void sdkSigGenOpenloopDeceleration(long encPort, long deceleration)
{
	VIRTMAST_PARAM(encPort, VIRTMAST_DEC) = deceleration; // dec is in Hz/s
}

/**
*	@brief 		Set velocity of Signal Generator
*	@details
* 	@param 		encPort		Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
* 	@param 		velocity	The velocity of the generated signal [Hz]
*
*/
void sdkSigGenOpenloopVelocity(long encPort, long velocity)
{
	VIRTMAST_PARAM(encPort, VIRTMAST_VEL) = velocity*1000; // vel is in 1/1000 Hz
}
