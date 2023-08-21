/**
*	@file		SDK_Amplifier_MACS.mc
*	@brief		Functions for the integrated amplifier setup.
*	$Revision: 215 $
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
#include "SDK_Amplifier_MACS.mh"

/**
*	@brief 		Sets the amplifier parameters for a DC motor
*	@details	The function sets the amplifier parameters for a DC motor.
*				Additionally the controller mode can be selected. Depending on the mode,
*				the controllers must then be parameterized. \n
*				The following functions are available:
*				@ref sdkSetupPositionPIDControl() / sdkSetupPositionPIDControlExt(),
*				@ref sdkSetupCurrentPIControl(),
*				@ref sdkSetupVelocityPIControl()
* 	@param 		axis		Axis module number
* 	@param 		controlMode	Define control typ (default 3)\n
*							@b 0 @b HWAMP_MODE_POS_VEL_CUR: 		Pos -> Vel -> Cur -> PWM \n
*							@b 1 @b HWAMP_MODE_POS_VEL:				Pos -> Vel -> PWM \n
*							@b 2 @b HWAMP_MODE_POS_CUR: 			Pos -> Cur -> PWM \n
*							@b 3 @b HWAMP_MODE_POS: 				Pos -> PWM
* 	@param 		polePairs	Number of pole pairs (default 1)
* 	@param 		maxCur		Maximal current allowed in mA (default 2000)
* 	@param 		encQc		Resolution of the encoder for position feed back in increments qc (default 2000)
* 	@param 		maxRpm		Maximum velocity in RPM, Use for scale the parameters for the velocity controller (default 1500)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupAmpDcMotor(long axis, long controlMode,long polePairs, long maxCur, long encQc, long maxRpm)
{
	HWAMP_PARAM(axis, HWAMP_COMMTYPE) = HWAMP_COMMTYPE_DC;     	// Set motor type
	HWAMP_PARAM(axis, HWAMP_MODE)     = controlMode;         	// Set controller priciple
    HWAMP_PARAM(axis, HWAMP_POLES)    = polePairs;     			// Number of pole pairs
    HWAMP_PARAM(axis, HWAMP_MAXCUR)   = maxCur;  				// Max current in mA
	HWAMP_PARAM(axis, HWAMP_ENCRES)   = encQc;  				// Given in qc
	HWAMP_PARAM(axis, HWAMP_MAXRPM)   = maxRpm;  				// Given in RPM

	return(1);
}

/**
*	@brief 		Sets the amplifier parameters for a BLDC 120° motor\n
*               Deprecated: Use sdkSetupAmpBldcMotor() and HWAMP_HALL_ALIGNMENT
*	@details	The function sets the amplifier parameters for a BLDC 120° motor.
*				Additionally the controller mode can be selected. Depending on the mode,
*				the controllers must then be parameterized. \n
*				The following functions are available:
*				@ref sdkSetupPositionPIDControl() / sdkSetupPositionPIDControlExt(),
*				@ref sdkSetupCurrentPIControl(),
*				@ref sdkSetupVelocityPIControl()
* 	@param 		axis		Axis module number
* 	@param 		controlMode	Define control typ (default 3)\n
*							@b 0 @b HWAMP_MODE_POS_VEL_CUR: 		Pos -> Vel -> Cur -> PWM \n
*							@b 1 @b HWAMP_MODE_POS_VEL:				Pos -> Vel -> PWM \n
*							@b 2 @b HWAMP_MODE_POS_CUR: 			Pos -> Cur -> PWM \n
*							@b 3 @b HWAMP_MODE_POS: 				Pos -> PWM
* 	@param 		polePairs	Number of pole pairs (default 1)
* 	@param 		maxCur		Maximal current allowed in mA (default 2000)
* 	@param 		encQc		Resolution of the encoder for position feed back in increments qc (default 2000)
* 	@param 		maxRpm		Maximum velocity in RPM, Use for scale the parameters for the velocity controller (default 1500)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupAmpBldc120Motor(long axis, long controlMode,long polePairs, long maxCur, long encQc, long maxRpm)
{
	HWAMP_PARAM(axis, HWAMP_COMMTYPE) = HWAMP_COMMTYPE_BLDC_120; 	// Set motor type
	HWAMP_PARAM(axis, HWAMP_MODE)     = controlMode;         		// Set controller priciple
    HWAMP_PARAM(axis, HWAMP_POLES)    = polePairs;     				// Number of pole pairs
    HWAMP_PARAM(axis, HWAMP_MAXCUR)   = maxCur;  					// Max current in mA
	HWAMP_PARAM(axis, HWAMP_ENCRES)   = encQc;  					// Given in qc
	HWAMP_PARAM(axis, HWAMP_MAXRPM)   = maxRpm;  					// Given in RPM

	return(1);
}

/**
*	@brief 		Sets the amplifier parameters for a BLDC  motor.
*	@details	The function sets the amplifier parameters for a BLDC motor.
*				Additionally the controller mode can be selected. Depending on the mode,
*				the controllers must then be parameterized. \n
*				The following functions are available:
*				@ref sdkSetupPositionPIDControl() / sdkSetupPositionPIDControlExt(),
*				@ref sdkSetupCurrentPIControl(),
*				@ref sdkSetupVelocityPIControl()
* 	@param 		axis		Axis module number
* 	@param 		hallAligment	Hall aligment. See SDO Dictionary Index: 4000, SubIndex: 88 (default 5)
* 	@param 		controlMode	Define control typ (default 3)\n
*							@b 0 @b HWAMP_MODE_POS_VEL_CUR: 		Pos -> Vel -> Cur -> PWM \n
*							@b 1 @b HWAMP_MODE_POS_VEL:				Pos -> Vel -> PWM \n
*							@b 2 @b HWAMP_MODE_POS_CUR: 			Pos -> Cur -> PWM \n
*							@b 3 @b HWAMP_MODE_POS: 				Pos -> PWM
* 	@param 		polePairs	Number of pole pairs (default 1)
* 	@param 		maxCur		Maximal current allowed in mA (default 2000)
* 	@param 		encQc		Resolution of the encoder for position feed back in increments qc (default 2000)
* 	@param 		maxRpm		Maximum velocity in RPM, Use for scale the parameters for the velocity controller (default 1500)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupAmpBldcMotor(long axis, long hallAligment, long controlMode,long polePairs, long maxCur, long encQc, long maxRpm)
{
	HWAMP_PARAM(axis, HWAMP_COMMTYPE) = HWAMP_COMMTYPE_BLDC; 		// Set motor type
	HWAMP_PARAM(axis, HWAMP_HALL_ALIGNMENT) =  hallAligment;  	 	// set hall alignment
	HWAMP_PARAM(axis, HWAMP_MODE)     = controlMode;         		// Set controller priciple
    HWAMP_PARAM(axis, HWAMP_POLES)    = polePairs;     				// Number of pole pairs
    HWAMP_PARAM(axis, HWAMP_MAXCUR)   = maxCur;  					// Max current in mA
	HWAMP_PARAM(axis, HWAMP_ENCRES)   = encQc;  					// Given in qc
	HWAMP_PARAM(axis, HWAMP_MAXRPM)   = maxRpm;  					// Given in RPM

	return(1);
}

/**
*	@brief 		Sets the amplifier parameters for a stepper motor (closed loop)
*	@details	The function sets the amplifier parameters for a stepper motor in closed loop.
*				Additionally the controller mode can be selected. Depending on the mode,
*				the controllers must then be parameterized. \n
*				The following functions are available:
*				@ref sdkSetupPositionPIDControl() / sdkSetupPositionPIDControlExt(),
*				@ref sdkSetupCurrentPIControl(),
*				@ref sdkSetupVelocityPIControl()
* 	@param 		axis		Axis module number
* 	@param 		controlMode	Define control typ (default 3)\n
*							@b 0 @b HWAMP_MODE_POS_VEL_CUR: 		Pos -> Vel -> Cur -> PWM \n
*							@b 1 @b HWAMP_MODE_POS_VEL:				Pos -> Vel -> PWM \n
*							@b 2 @b HWAMP_MODE_POS_CUR: 			Pos -> Cur -> PWM \n
*							@b 3 @b HWAMP_MODE_POS: 				Pos -> PWM
* 	@param 		steps		Steps per revolution
* 	@param 		maxCur		Maximal current allowed in mA (default 2000)
* 	@param 		encQc		Resolution of the encoder for position feed back in increments qc (default 2000)
* 	@param 		maxRpm		Maximum velocity in RPM, Use for scale the parameters for the velocity controller (default 1500)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupAmpStepMotor_CL(long axis, long controlMode,long steps, long maxCur, long encQc, long maxRpm)
{
	HWAMP_PARAM(axis, HWAMP_COMMTYPE) = HWAMP_COMMTYPE_STEP; 		// Set motor type
	HWAMP_PARAM(axis, HWAMP_MODE)     = controlMode;         		// Set controller priciple
	HWAMP_PARAM(axis, HWAMP_POLES)    = steps/4;     				// For 2-phase stepper: Poles = Steps per revolution / 4
    HWAMP_PARAM(axis, HWAMP_MAXCUR)   = maxCur;  					// Max current in mA
	HWAMP_PARAM(axis, HWAMP_ENCRES)   = encQc;  					// Given in qc
	HWAMP_PARAM(axis, HWAMP_MAXRPM)   = maxRpm;  					// Given in RPM

	return(1);
}
/**
*	@brief 		Sets the amplifier parameters for a stepper motor (closed loop)
*	@details	The function sets the amplifier parameters for a stepper motor in closed loop.
*				Additionally the controller mode can be selected. Depending on the mode,
*				the controllers must then be parameterized. \n
*				The following functions are available:
*				@ref sdkSetupPositionPIDControl() / sdkSetupPositionPIDControlExt(),
*				@ref sdkSetupCurrentPIControl(),
*				@ref sdkSetupVelocityPIControl()
* 	@param 		axis		Axis module number
* 	@param 		controlMode	Define control typ (default 3)\n
*							@b 0 @b HWAMP_MODE_POS_VEL_CUR: 		Pos -> Vel -> Cur -> PWM \n
*							@b 1 @b HWAMP_MODE_POS_VEL:				Pos -> Vel -> PWM \n
*							@b 2 @b HWAMP_MODE_POS_CUR: 			Pos -> Cur -> PWM \n
*							@b 3 @b HWAMP_MODE_POS: 				Pos -> PWM
* 	@param 		steps		Steps per revolution
* 	@param 		maxCur		Maximal current allowed in mA (default 2000)
* 	@param 		maxRpm		Maximum velocity in RPM, Use for scale the parameters for the velocity controller (default 1500)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupAmpStepMotor_OL(long axis, long steps, long maxCur,long maxRpm)
{
	HWAMP_PARAM(axis, HWAMP_COMMTYPE) = HWAMP_COMMTYPE_STEP; 		// Set motor type
	HWAMP_PARAM(axis, HWAMP_MODE)     = HWAMP_MODE_PFG_CUR_PWM; 		// Set controller priciple
	HWAMP_PARAM(axis, HWAMP_POLES)    = steps/4;     				// For 2-phase stepper: Poles = Steps per revolution / 4
    HWAMP_PARAM(axis, HWAMP_MAXCUR)   = maxCur;  					// Max current in mA
	HWAMP_PARAM(axis, HWAMP_ENCRES)   = HWAMP_PARAM(axis, HWAMP_POLES) * HWAMP_ENCRES_MICROSTEP_RES;// Given in qc
	HWAMP_PARAM(axis, HWAMP_MAXRPM)   = maxRpm;  					// Given in RPM

	return(1);
}

/**
*	@brief 		Sets the amplifier parameters for a brushless, PMSM commuted motor (no Hall sensors are used)
*	@details	The function sets the amplifier parameters for a brushless, PMSM commuted motor.
*				Additionally the controller mode can be selected. Depending on the mode,
*				the controllers must then be parameterized. \n
*				The following functions are available:
*				@ref sdkMotorAlignment()
*				@ref sdkSetupPositionPIDControl() / sdkSetupPositionPIDControlExt(),
*				@ref sdkSetupCurrentPIControl(),
*				@ref sdkSetupVelocityPIControl()
* 	@param 		axis		Axis module number
* 	@param 		controlMode	Define control typ (default 3)\n
*							@b 0 @b HWAMP_MODE_POS_VEL_CUR: 		Pos -> Vel -> Cur -> PWM \n
*							@b 1 @b HWAMP_MODE_POS_VEL:				Pos -> Vel -> PWM \n
*							@b 2 @b HWAMP_MODE_POS_CUR: 			Pos -> Cur -> PWM \n
*							@b 3 @b HWAMP_MODE_POS: 				Pos -> PWM
* 	@param 		polePairs	Number of pole pairs (default 1)
* 	@param 		maxCur		Maximal current allowed in mA (default 2000)
* 	@param 		encQc		Resolution of the encoder for position feed back in increments qc (default 2000)
* 	@param 		maxRpm		Maximum velocity in RPM, Use for scale the parameters for the velocity controller (default 1500)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupAmpPmsmMotor(long axis, long controlMode,long polePairs, long maxCur, long encQc, long maxRpm)	//$B
{
	HWAMP_PARAM(axis, HWAMP_COMMTYPE) = HWAMP_COMMTYPE_PMSM; 		// Set motor type
	HWAMP_PARAM(axis, HWAMP_MODE)     = controlMode;         		// Set controller priciple
    HWAMP_PARAM(axis, HWAMP_POLES)    = polePairs;     				// Number of pole pairs
    HWAMP_PARAM(axis, HWAMP_MAXCUR)   = maxCur;  					// Max current in mA
	HWAMP_PARAM(axis, HWAMP_ENCRES)   = encQc;  					// Given in qc
	HWAMP_PARAM(axis, HWAMP_MAXRPM)   = maxRpm;  					// Given in RPM

	return(1);
}
/**
*	@brief 		Sets the amplifier parameters for a brushless, PMSM commuted motor (Hall sensors are used)
*	@details	The function sets the amplifier parameters for a brushless, PMSM commuted motor.
*				Additionally the controller mode can be selected. Depending on the mode,
*				the controllers must then be parameterized. \n
*				The following functions are available:
*				@ref sdkSetupPositionPIDControl() / sdkSetupPositionPIDControlExt(),
*				@ref sdkSetupCurrentPIControl(),
*				@ref sdkSetupVelocityPIControl()
* 	@param 		axis		Axis module number
* 	@param 		controlMode	Define control typ (default 3)\n
*							@b 0 @b HWAMP_MODE_POS_VEL_CUR: 		Pos -> Vel -> Cur -> PWM \n
*							@b 1 @b HWAMP_MODE_POS_VEL:				Pos -> Vel -> PWM \n
*							@b 2 @b HWAMP_MODE_POS_CUR: 			Pos -> Cur -> PWM \n
*							@b 3 @b HWAMP_MODE_POS: 				Pos -> PWM
* 	@param 		polePairs	Number of pole pairs (default 1)
* 	@param 		maxCur		Maximal current allowed in mA (default 2000)
* 	@param 		encQc		Resolution of the encoder for position feed back in increments qc (default 2000)
* 	@param 		maxRpm		Maximum velocity in RPM, Use for scale the parameters for the velocity controller (default 1500)
* 	@param 		elPol		Encoder polarity vs. electrical polarity (default -1)\n
*							@b 1 @b HWAMP_ELPOL_REGULAR: 		Electrical polarity is equal encoder's polarity \n
*							@b -1 @b HWAMP_ELPOL_INVERS: 		Electrical polarity is inverse to encoder's polarity \n
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupAmpHallPmsmMotor(long axis, long controlMode,long polePairs, long maxCur, long encQc, long maxRpm, long elPol)
{
	HWAMP_PARAM(axis, HWAMP_COMMTYPE) = HWAMP_COMMTYPE_HALL_PMSM; 	// Set motor type
	HWAMP_PARAM(axis, HWAMP_ELPOL) 	  = elPol; 						// encoder polarity vs. electrical polarity:
	HWAMP_PARAM(axis, HWAMP_MODE)     = controlMode;         		// Set controller priciple
    HWAMP_PARAM(axis, HWAMP_POLES)    = polePairs;     				// Number of pole pairs
    HWAMP_PARAM(axis, HWAMP_MAXCUR)   = maxCur;  					// Max current in mA
	HWAMP_PARAM(axis, HWAMP_ENCRES)   = encQc;  					// Given in qc
	HWAMP_PARAM(axis, HWAMP_MAXRPM)   = maxRpm;  					// Given in RPM

	return(1);
}

/**
*	@brief 		Set parameters for PI current control loop
*	@details	The function sets the factors of the current PI controller. More information are available in the Help of ApossIDE.
* 	@param 		axis		Axis module number
* 	@param 		curkprop	Proportional factor of current controller (default 200)
* 	@param 		curkint		Integral factor of current controller (default 100)
* 	@param 		curkilim	Integral limit of current controller (default 1000)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupCurrentPIControl(long axis, long curkprop, long curkint, long curkilim)
{
	// parameters for current controller
	HWAMP_PARAM((axis), HWAMP_CURKPROP) = curkprop;	// current-controller  proportional factor:   +/-32767   (CURKPROP)
	HWAMP_PARAM((axis), HWAMP_CURKINT)  = curkint;	// current-controller  integral factor:       +/-32767   (CURKINT)
	HWAMP_PARAM((axis), HWAMP_CURKILIM) = curkilim;	// current-controller Integral limit:         0 … 32767  (CURKILIM)

	return(1);
}

/**
*	@brief 		Set parameters for PI velocity control loop
*	@details	The function sets the factors of the velocity PI controller. More information are available in the Help of ApossIDE.
* 	@param 		axis		Axis module number
* 	@param 		velkprop	Proportional factor of velocity controller (default 200)
* 	@param 		velkint		Integral factor of velocity controller (default 5)
* 	@param 		velkilim	Integral limit of velocity controller (default 1000)
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupVelocityPIControl(long axis, long velkprop, long velkint, long velkilim)
{
	// parameters for velocity controller
	HWAMP_PARAM(axis, HWAMP_VELKPROP) = velkprop;	// velocity-controller  proportional factor:   +/-32767   (VELKPROP)
	HWAMP_PARAM(axis, HWAMP_VELKINT)  = velkint;	// velocity-controller  integral factor:       +/-32767   (VELKINT)
	HWAMP_PARAM(axis, HWAMP_VELKILIM) = velkilim;	// velocity-controller Integral limit:         0 … 32767  (VELKILIM)

	return(1);
}

/**
*	@brief 		Function to generate a virtual I2T protection.
*	@details	This function activates a virtual I2T protection. More information about this can be found in the ApossIDE help.
* 	@param 		axis		Axis module number
* 	@param 		nominalCur	Nominal current (max. continuous load current) in mA
* 	@param 		thermalTime	Therm. Winding time constant in ms
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupVirtualI2T(long axis, long nominalCur, long thermalTime)
{
	// parameters for I2T protection
	VIRTAMP_PARAM(axis,VIRTAMP_I2TLIMIT)	=	(nominalCur*nominalCur/1000);	// I*I*1000
	VIRTAMP_PARAM(axis,VIRTAMP_I2TTIME)	= 	thermalTime;

	return(1);
}
