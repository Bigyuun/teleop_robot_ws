/**
*	@file		SDK_Encoder_Setup.mc
*	@brief		Functions for the general amplifier setup.
*	$Revision: 205 $
*
*	@example	SetupAbsSSIEncoder.mc
*	@example	SetupIncEncoder.mc
*	@example	SetupSinCosEncoder.mc
*	@example	Maxon_EC45_flat_1ax_BC_Hall.mc
*
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Encoder_Setup.mh"

/**
*	@brief 		Settings for an incremental encoder
*	@details	With this function the settings can be made which are required for an incremental encoder.
*				Default is the encoder input 0 of axis 0, the encoder input 1 of axis 1 etc.
*				This function is used to reassign the modules. The encoder assignment can be set up variably.
* 	@param 		axis			Axis module number
* 	@param 		encPort			Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
* 	@param 		encRes			Resolution of the encoder for position feed back in increments (quadcounts)
* 	@param 		latchType		Defines the latch type \n
*								@b 0: Default latchTyp Encoder Z signal \n
*								@b 1: A digital input is used as latch signal \n
* 	@param 		latchParam		Parameter depending on the latch type \n
*								@b latchType @b 0: Parameter can be set to 0 \n
*								@b latchType @b 1: The preferred digital input is set (Digital input 1 - 64)
* 	@param 		latchSlope		Defines the slope of the trigger signal (Default 1) \n
*								@b 0 @b HWLATCH_SLOPE_CONTINUOUS: Continuous trigger signal\n
*								@b 1 @b HWLATCH_SLOPE_RISING: Rising trigger signal\n
*								@b 2 @b HWLATCH_SLOPE_FALLING: Falling trigger signal\n
*								@b 3 @b HWLATCH_SLOPE_BOTH: Rising/ Falling trigger signal
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupIncEncoder(long axis, long encPort, long encRes, long latchType, long latchParam, long latchSlope)
{
	VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE;												// Defines how the source value is handled
	VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_PISRC_COUNTER) = HWCOUNTINC_PROCESS_SRCINDEX(encPort,PO_HWCNTINC_VALUE);		// Setup virtual counter module

	HWENC_PARAM(encPort, HWENCODER_MODE) = HWENCODER_MODE_INCREMENTAL;												// Setup encoder type
    HWAMP_PARAM(encPort, HWAMP_ENCRES) = encRes;																	// Resolution of the encoder
    HWAMP_PARAM(axis, HWAMP_PISRC_ACTPOS) = HWCOUNTINC_PROCESS_SRCINDEX(encPort,PO_HWCNTINC_VALUE);					// Source input for actual position

    // Virtuallatch module mapping
    VIRTLATCH_PARAM(axis,VIRTLATCH_MODE)=
    VIRTLATCH_MODE_HARDWARE;
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_COUNTER) =
    VIRTCOUNTIN_PROCESS_SRCINDEX(axis,PO_VIRTCNTIN_VALUE);
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_LATCHCNT)=
    HWCOUNTINC_PROCESS_SRCINDEX(encPort,PO_HWCNTINC_VALUE);
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_LATCH)=
    HWLATCH_PROCESS_SRCINDEX(encPort,PO_HWLATCH_VALUE);
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_LATCHSTAT)=
    HWLATCH_PROCESS_SRCINDEX(encPort,PO_HWLATCH_FLAG);
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_LATCHFIFO_AMOUNT)=
    HWLATCH_PROCESS_SRCINDEX(encPort,PO_HWLATCH_AMOUNT_IN_FIFO);
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_LATCHFIFO_READ)=
    HWLATCH_PROCESS_SRCINDEX(encPort,PO_HWLATCH_FIFOREAD);

    if (latchType == 0)	// Default latchtype, encoder Z signal
    {
		HWLATCH_PARAM(encPort, HWLATCH_PISRC_TRIGGER) = HWLATCH_PISRC_TRIGGER_ENCZ;
    }
    else if (latchType == 1)	// Digital input for latch signal
    {
		HWLATCH_PARAM(encPort, HWLATCH_PISRC_TRIGGER) =   HWLATCH_PISRC_TRIGGER_DINP + latchParam-1;
		HWLATCH_PARAM(encPort, HWLATCH_SLOPE) = latchSlope;
    }
   	else	// Error
    {
    	print("Not supported latchType");
    	return(-1);
    }
	return(1);
}

/**
*	@brief 		Settings for an SinCos encoder
*	@details	With this function the settings can be made which are required for an SinCos encoder.
*				Default is the encoder input 0 of axis 0, the encoder input 1 of axis 1 etc.
*				This function is used to reassign the modules. The encoder assignment can be set up variably.
* 	@param 		axis			Axis module number
* 	@param 		encPort			Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
* 	@param 		encRes			Resolution of the encoder for position feed back in increments (quadcounts)\n
*								Input must be multiplied by a factor of 256.
* 	@param 		latchType		Defines the latch type \n
*								@b 0: Default latchTyp Encoder Z signal \n
*								@b 1: A digital input is used as latch signal \n
* 	@param 		latchParam		Parameter depending on the latch type \n
*								@b latchType @b 0: Parameter can be set to 0 \n
*								@b latchType @b 1: The preferred digital input is set (Digital input 1 - 64)
* 	@param 		latchSlope		Defines the slope of the trigger signal (Default 1) \n
*								@b 0 @b HWLATCH_SLOPE_CONTINUOUS: Continuous trigger signal\n
*								@b 1 @b HWLATCH_SLOPE_RISING: Rising trigger signal\n
*								@b 2 @b HWLATCH_SLOPE_FALLING: Falling trigger signal\n
*								@b 3 @b HWLATCH_SLOPE_BOTH: Rising/ Falling trigger signal
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupSinCosEncoder(long axis, long encPort, long encRes, long latchType, long latchParam, long latchSlope)
{
	VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE;												// Defines how the source value is handled
	VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_PISRC_COUNTER) = HWCOUNTINC_PROCESS_SRCINDEX(encPort,PO_HWCNTINC_VALUE);		// Setup virtual counter module

	HWENC_PARAM(encPort, HWENCODER_MODE) = HWENCODER_MODE_SINCOS;													// Setup encoder type
    HWAMP_PARAM(encPort, HWAMP_ENCRES) = encRes;																	// Resolution of the encoder
    HWAMP_PARAM(axis, HWAMP_PISRC_ACTPOS) = HWCOUNTINC_PROCESS_SRCINDEX(encPort,PO_HWCNTINC_VALUE);					// Source input for actual position

    // Virtuallatch module mapping
    VIRTLATCH_PARAM(axis,VIRTLATCH_MODE)=
    VIRTLATCH_MODE_HARDWARE;
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_COUNTER)=
    VIRTCOUNTIN_PROCESS(encPort,PO_VIRTCNTIN_VALUE);
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_LATCHCNT)=
    HWCOUNTINC_PROCESS_SRCINDEX(encPort,PO_HWCNTINC_VALUE);
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_LATCH)=
    HWLATCH_PROCESS_SRCINDEX(encPort,PO_HWLATCH_VALUE);
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_LATCHSTAT)=
    HWLATCH_PROCESS_SRCINDEX(encPort,PO_HWLATCH_FLAG);
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_LATCHFIFO_AMOUNT)=
    HWLATCH_PROCESS_SRCINDEX(encPort,PO_HWLATCH_AMOUNT_IN_FIFO);
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_LATCHFIFO_READ)=
    HWLATCH_PROCESS_SRCINDEX(encPort,PO_HWLATCH_FIFOREAD);

    if (latchType == 0)	// Default latchtype, encoder Z signal
    {
		HWLATCH_PARAM(encPort, HWLATCH_PISRC_TRIGGER) = HWLATCH_PISRC_TRIGGER_ENCZ;
    }
    else if (latchType == 1)	// Digital input for latch signal
    {
		HWLATCH_PARAM(encPort, HWLATCH_PISRC_TRIGGER) = HWLATCH_PISRC_TRIGGER_DINP + latchParam -1;
		HWLATCH_PARAM(encPort, HWLATCH_SLOPE) = latchSlope;
    }
   	else	// Error
    {
    	print("Not supported latchType");
    	return(-1);
    }
	return(1);
}



/**
*	@brief 		Settings for a SSI encoder
*	@details	With this function the settings can be made which are required for an SSI encoder.
*				Default is the encoder input 0 of axis 0, the encoder input 1 of axis 1 etc.
*				This function is used to reassign the modules. The encoder assignment can be set up variably.\n \n
*				Example for a latching with a singelturn absSSI encoder: \n A latching is performed on an encoder with a resolution of 8192
*				with the actual value of 2192. At the next overflow (reaching 8192) this value is subtracted by 8192. The last marker value is -6000,
*				after the next turn -14 192 and so on.
* 	@param 		axis			Axis module number
* 	@param 		encPort			Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
* 	@param 		encRes			Resolution of the encoder for position feed back in increments (quadcounts)
*	@param		clockFreq		Clock frequency of the SSI encoder (Hz)
* 	@param 		fastUpdate		The fast update is only necessary, when the motor commutation is done with this encoder (PMSM mode)\n
*								@b fastUpdate @b 0: The encoder is updated with the velocity controller update rate (8 kHz on MiniMACS6) \n
*								@b fastUpdate @b 1: The encoder is updated with the current controller update rate (24 kHz on MiniMACS6)
*	@param		datlen			Databit length of endat position
*	@param		isBinary 		1 if data coding is binary otherwise it is grey coded.
* 	@param 		latchBitMask	The result from VIRTLATCH_PISRC_LATCHVALID will be ANDed with this bitmask to decide if a latch should be accepted.
*								In this function the virtual input module is set as source. This is linked to the hardware digital inputs by default.
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*	@note
*/
long sdkSetupAbsSSIEncoder(long axis, long encPort, long encRes, long clockFreq, long fastUpdate, long datlen, long isBinary, long latchBitMask)
{
	VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_MODE) =  VIRTCNTIN_MODE_ABSOLUTE_DIRECT_ENDLESS;							// Defines how the source value is handled
	VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_PISRC_COUNTER) = HWCOUNTABS_PROCESS_SRCINDEX(encPort,PO_HWCNTABS_VALUE);	// Setup virtual counter module
	VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_OVFL_VALUE) = encRes;														// Value where counter should produce an overflow

	HWENC_PARAM(encPort, HWENCODER_DATLEN) = datlen;
    HWENC_PARAM(encPort, HWENCODER_CLOCKFREQ) = clockFreq;

    HWAMP_PARAM(axis, HWAMP_PISRC_ACTPOS) = HWCOUNTABS_PROCESS_SRCINDEX(encPort,PO_HWCNTABS_VALUE);				// Source input for actual position

    HWENC_PARAM(encPort, HWENCODER_MODE) = HWENCODER_MODE_SSI_ACTIVE; 											// Setup SSI Encoder active clock

	if(fastUpdate) 																								// Setup fast update for commutation
	{
		HWENC_PARAM(encPort, HWENCODER_FAST_UPDATE) = 1;
		//HWENC_PARAM(encPort, HWENCODER_FAST_UPDATE) = HWENCODER_FAST_UPDATE_ENABLE;
	}
	else
	{
		HWENC_PARAM(encPort, HWENCODER_FAST_UPDATE) = 0;
		//HWENC_PARAM(encPort, HWENCODER_FAST_UPDATE) = HWENCODER_FAST_UPDATE_DISABLE;
	}
	if(isBinary) 																								// Setup data coding typ
	{
		HWCOUNTABS_PARAM(encPort, HWCNTABS_CODING) = HWCNTABS_CODING_NONE;
	}
	else
	{
		HWCOUNTABS_PARAM(encPort, HWCNTABS_CODING) = HWCNTABS_CODING_GREY;
	}

    // Virtuallatch module mapping
	VIRTLATCH_PARAM(axis,VIRTLATCH_MODE)=VIRTLATCH_MODE_SOFTWARE;
	// Set the source for the lathc module
    VIRTLATCH_PARAM(axis,VIRTLATCH_PISRC_COUNTER)=
    HWCOUNTABS_PROCESS_SRCINDEX(encPort,PO_HWCNTABS_VALUE);
   	VIRTLATCH_PARAM(axis, VIRTLATCH_PISRC_LATCHVALID) =
   	// Set the source for input trigger, could be an other source like user param
   	VIRTDIGIN_PROCESS_SRCINDEX(axis,PO_VIRTDIGIN_VALLONG);
   	// Bitmask for the latchvalid source
   	VIRTLATCH_PARAM(axis, VIRTLATCH_LATCHVALID_BITMASK) = latchBitMask;

	return(1);
}
/**
*	@brief 		Settings for an hall encoder
*	@details	With this function the settings can be made which are required for an hall encoder.
*				This function only valid for a MACS5 controller
* 	@param 		axis			Axis module number
* 	@param 		hallPort		Hall port number
* 	@param 		hallAligment	Hall aligment. See SDO Dictionary Index: 4000, SubIndex: 88
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupHallEncoderMACS5(long axis, long hallPort,long hallAligment)
{
	long hall_ModNo;

	// Determine Hall Number, note that the Hall-Module number has a fix assignment to the axis number
	if     (axis == 0){ hall_ModNo = 0; }
	else if (axis == 2){ hall_ModNo = 1; }
	else if (axis == 3){ hall_ModNo = 2; }
	else if (axis == 5){ hall_ModNo = 3; }
	else { // (AxisNo == 1 || AxisNo == 4)
	  print("Axis No ", axis, " cannot be used as BLDC axis");
	  return(-1);
	}

	HWAMP_PARAM((axis), HWAMP_HALL_ALIGNMENT) =  hallAligment;  	// set hall alignment

	// Parameterization Hall Sensors
	HWENC_PARAM((hallPort), HWENCODER_MODE) = HWENCODER_MODE_HALL; 	// Set encder mode
	HWHALL_PARAM((hallPort), HWHALL_MODE) = HWHALL_MODE_ENABLE;  	// enable hall decoder
	HWHALL_PARAM((hall_ModNo), HWHALL_PISRC_ENCOUT) = (hallPort);  	// use input number

	return(1);
}


/**
*	@brief 		Settings for an hall encoder
*	@details	With this function the settings can be made which are required for use hall encoder only.
* 	@param 		axis			Axis module number
* 	@param 		hallAligment	Hall aligment. See SDO Dictionary Index: 4000, SubIndex: 88
* 	@param 		hallDirection	Direction of rotation of the hall signals\n
*								@b hallDirection @b 1: Normal \n
*								@b hallDirection @b -1: Inverse (For example Maxon)
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupHallEncoder(long axis, long hallAligment, long hallDirection)
{
	VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE;												// Defines how the source value is handled
	VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_PISRC_COUNTER) = HWHALL_PROCESS_SRCINDEX(axis,PO_HWHALL_POS);

	HWAMP_PARAM(axis, HWAMP_HALL_ALIGNMENT) =  hallAligment;  	// set hall alignment
	HWHALL_PARAM(axis, HWHALL_MODE) = HWHALL_MODE_ENABLE;  		// enable hall encoder

	if(hallDirection==-1)
		VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_UUFACT_INCNO) = -1;

	return(1);
}

