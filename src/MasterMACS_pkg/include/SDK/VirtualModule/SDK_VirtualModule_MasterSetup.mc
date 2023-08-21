/**
*	@file		SDK_VirtualModule_MasterSetup.mc
*	@brief		Functions to work with a virtual master.
*	$Revision: 155 $
*
*   @example VirtualMaster_ProfileMode.mc
*
*/

#pragma once

#include "SDK_VirtualModule_MasterSetup.mh"

/**
*	@brief 		Setup a virtual master
*	@details	The virtual master module generates a virtual speed and position signal.
*				There are two modes to operate the module (see VIRTMAST_MODE):
* 	@param 		master	Master module number
* 	@param 		mode	Axis module number\n
*						@b 0 @b VIRTMAST_MODE_DISABLED: Disabled, output velocity is zero\n
*						@b 1 @b VIRTMAST_MODE_VELOCITY: Velocity mode, PO_VIRTMAST_VEL is taken directly from VIRTMAST_PISRC_CMDVEL\n
*						@b 3 @b VIRTMAST_MODE_PROFILE: Velocity profile mode, PO_VIRTMAST_VEL is generated using VIRTMAST_VEL/ACC/DEC
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupVirtualMasterMode(long master, long mode)
{
	VIRTMAST_PARAM(master,VIRTMAST_MODE)	= mode;

	return(1);
}

/**
*	@brief 		Set the virtual master as master of an axis
*	@details	Set the virtual master as master of an axis.
* 	@param 		master	Master module number
* 	@param 		axis	Axis module number
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupVirtualMasterAxisLink(long master, long axis)
{

	// Axis master module = axis module number + max axis module
	VIRTCOUNTIN_PARAM(axis + SYS_PROCESS(SYS_MAXAX), VIRTCNTIN_PISRC_COUNTER) = VIRTMAST_PROCESS_SRCINDEX(master,PO_VIRTMAST_POS);

	return(1);
}

/**
*	@brief 		Scale the input parameters of the virtual master
*	@details	This function converts user units into increments (increments * (numerator/denominator))\n
*				The parameters VIRTMAST_VEL, VIRTMAST_ACC, VIRTMAST_DEC, VIRTMAST_PISRC_CMDVEL and the output
*				PO_VIRTMAST_VEL are changed according to this scaling.
* 	@param 		master	Master module number
* 	@param 		numerator	User factor numerator to scale to increments
* 	@param 		denominator	User factor denominator to scale to increments
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupVirtualMasterScale(long master, long numerator, long denominator)
{
	VIRTMAST_PARAM(master,VIRTMAST_UUFACT_INCNO) = numerator;
	VIRTMAST_PARAM(master,VIRTMAST_UUFACT_UNITNO) = denominator;

	return(1);
}

/**
*	@brief 		Set a virtual master profile
*	@details	Only if the virtual master is in profile mode (@ref sdkSetupVirtualMasterMode()).  \n
*				Definition of the profile parameters of the master.
*				The scaling can be adjusted with the @ref sdkSetupVirtualMasterScale() function.
* 	@param 		master	Master module number
* 	@param 		acc		Acceleration to reach target velocity		[uu/ms]
* 	@param 		dec		Deceleration to reach target velocity		[uu/ms]
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetVirtualMasterProfile(long master, long acc, long dec)
{
	VIRTMAST_PARAM(master,VIRTMAST_ACC)		= acc;
	VIRTMAST_PARAM(master,VIRTMAST_DEC)		= dec;

	return(1);
}

/**
*	@brief 		Starts a virtual master in profile mode.
*	@details	Only if the virtual master is in profile mode (@ref sdkSetupVirtualMasterMode()). \n
*				Starts a virtual master in profile mode, where the speed is transferred.
*				The scaling can be adjusted with the @ref sdkSetupVirtualMasterScale() function.
* 	@param 		master	Master module number
* 	@param 		vel		Target velocity		[uu/ms]
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkStartVirtualMasterProfile(long master, long vel)
{
	VIRTMAST_PARAM(master,VIRTMAST_VEL)		= vel;

	return(1);
}
/**
*	@brief 		Stops a virtual master in profile mode.
*	@details	Only if the virtual master is in profile mode (@ref sdkSetupVirtualMasterMode()). \n
*				Stops a virtual master in profile mode.
* 	@param 		master	Master module number
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkStopVirtualMasterProfile(long master)
{
	VIRTMAST_PARAM(master,VIRTMAST_VEL)		= 0;

	return(1);
}
