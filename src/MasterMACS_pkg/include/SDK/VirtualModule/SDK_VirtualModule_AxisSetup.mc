/**
*	@file		SDK_VirtualModule_AxisSetup.mc
*	@brief		Functions to work with virtual axis modules.
*	$Revision: 39 $
*
*/

#pragma once

#include "SDK_VirtualModule_AxisSetup.mh"

/**
*	@brief 		Start a simulated axis
*	@details	Create a simulated axis with a virtual axismodule.
*				A virtual axis can be used for simulation reasons.
* 	@param 		axis	Axis module number
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupAxisSimulation(long axis)
{
	VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_PISRC_COUNTER) = AXE_PROCESS_SRCINDEX(axis, REG_COMPOS);
	VIRTCOUNTIN_PARAM(axis, VIRTCNTIN_MODE) = VIRTCNTIN_MODE_SIMULATION_SLAVE;
	VIRTLATCH_PARAM(axis, VIRTLATCH_MODE) = VIRTLATCH_MODE_SIMULATOR;
	VIRTLATCH_PARAM(axis, VIRTLATCH_PISRC_COUNTER) = AXE_PROCESS_SRCINDEX(axis, REG_ACTPOS);

	return(1);
}