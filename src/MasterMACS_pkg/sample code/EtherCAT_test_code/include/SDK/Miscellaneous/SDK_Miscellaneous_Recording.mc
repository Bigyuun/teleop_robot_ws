/**
*	@file		SDK_Miscellaneous_Recording.mc
*	@brief		Functions with recording samples.
*	$Revision: 139 $
*
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Miscellaneous_Recording.mh"


/**
*	@brief 		Starts a recording of general axis parameter of a single axis.
*	@details	This function records the nominal and actual position of the axis.
*				Additionally the trackerror, speed and current output is recorded. The recording can be stopped during a
*				desired time with the "RecordStop(0,0)" method.
* 	@param 		axis		Axis module number
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkRecordGeneralAxisParam_1Ax(long axis)
{
    RecordTime(1);				// Sampling period in milliseconds.
    RecordIndex( 	AXE_PROCESS_INDEX(axis,REG_COMPOS),
    				AXE_PROCESS_INDEX(axis,REG_ACTPOS),
    				AXE_PROCESS_INDEX(axis,REG_TRACKERR),
    				AXE_PROCESS_INDEX(axis,REG_AVEL),
    				HWAMP_PROCESS_INDEX(axis, PO_HWAMP_CURRENT)
    			);

    RecordDest (DYNMEM);        // DYNMEM for recording data into free memory.
    RecordType(1);              // 1 = cyclic recording
    RecordStart(0);				// Start recording

	return(1);
}


/**
*	@brief 		Starts a recording of general axis parameter of four axes
*	@details	This function records the nominal and actual position of four axes
*				Additionally the trackerror, speed and current output is recorded. The recording can be stopped during a
*				desired time with the "RecordStop(0,0)" method.
* 	@param 		axis1		Axis module number 1
* 	@param 		axis2		Axis module number 2
* 	@param 		axis3		Axis module number 3
* 	@param 		axis4		Axis module number 4
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkRecordGeneralAxisParam_4Ax(long axis1, long axis2, long axis3, long axis4)
{
    RecordTime(1);				// Sampling period in milliseconds.
    RecordIndex( 	AXE_PROCESS_INDEX(axis1,REG_COMPOS),
    				AXE_PROCESS_INDEX(axis1,REG_ACTPOS),
    				AXE_PROCESS_INDEX(axis1,REG_TRACKERR),
    				AXE_PROCESS_INDEX(axis1,REG_AVEL),
    				HWAMP_PROCESS_INDEX(axis1, PO_HWAMP_CURRENT),
    				AXE_PROCESS_INDEX(axis2,REG_COMPOS),
    				AXE_PROCESS_INDEX(axis2,REG_ACTPOS),
    				AXE_PROCESS_INDEX(axis2,REG_TRACKERR),
    				AXE_PROCESS_INDEX(axis2,REG_AVEL),
    				HWAMP_PROCESS_INDEX(axis2, PO_HWAMP_CURRENT),
    				AXE_PROCESS_INDEX(axis3,REG_COMPOS),
    				AXE_PROCESS_INDEX(axis3,REG_ACTPOS),
    				AXE_PROCESS_INDEX(axis3,REG_TRACKERR),
    				AXE_PROCESS_INDEX(axis3,REG_AVEL),
    				HWAMP_PROCESS_INDEX(axis3, PO_HWAMP_CURRENT),
    				AXE_PROCESS_INDEX(axis4,REG_COMPOS),
    				AXE_PROCESS_INDEX(axis4,REG_ACTPOS),
    				AXE_PROCESS_INDEX(axis4,REG_TRACKERR),
    				AXE_PROCESS_INDEX(axis4,REG_AVEL),
    				HWAMP_PROCESS_INDEX(axis4, PO_HWAMP_CURRENT)
    			);

    RecordDest (DYNMEM);        // DYNMEM for recording data into free memory.
    RecordType(1);              // 1 = cyclic recording
    RecordStart(1);				// Start recording

	return(1);
}
