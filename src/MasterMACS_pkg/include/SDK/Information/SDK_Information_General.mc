/**
*	@file		SDK_Information_General.mc
*	@brief		Functions to get general informations.
*	$Revision: 24 $
*
*	@example InformationGeneral.mc
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Information_General.mh"

/**
*	@brief 		Prints the software versions of the controller.
*	@details	Function to print the software versions of the controller.
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkInfoPrintSoftware()
{

    print("**********************************");
	print("------ Software information ------");
    print("**********************************\n");

    print("Hardware ID:\t", SYS_INFO(SYS_HARDWARE_ID));
    print("Firmware:\t\t", SYS_INFO(SYS_FW_CPU));
    print("FPGA:\t\t", SYS_INFO(SYS_FPGA_SW_VERSION));
    print("Amplifier:\t\t", SYS_INFO(SYS_FW_AMPLIFIER_VERSION));
    print("Coprocessor:\t", SYS_INFO(SYS_FW_COPROCESSOR_VERSION));
    print("Bootloader:\t", SYS_INFO(SYS_BOOTLOADER_SW_VERSION));
	print("");

	return(1);
}

/**
*	@brief 		Prints the hardware information of the controller.
*	@details	Function to print the hardware version of the controller.
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkInfoPrintHardware()
{

    print("**********************************");
	print("-------- H/W information --------");
    print("**********************************\n");

    print("Amplifier:\t\t\t", SYS_INFO(SYS_ZB_AMP_NO));
    print("Encoder:\t\t\t", SYS_INFO(SYS_MAX_ENCODER ));
    print("Incremental counters:\t", SYS_INFO(SYS_MAX_CNTINC));
    print("Absolute counters:\t\t", SYS_INFO(SYS_MAX_CNTABS));
    print("Universal counters:\t", SYS_INFO(SYS_MAX_CNTUNI));
    print("Comparators:\t\t", SYS_INFO(SYS_MAX_COMPARATORS));
    print("Latches:\t\t\t", SYS_INFO(SYS_MAX_LATCH));
    print("Signal generators:\t\t", SYS_INFO(SYS_MAX_SIGGEN));
    print("CAN Module:\t\t", SYS_INFO(SYS_MAX_CANBUS));
	print("");

	return(1);
}


/**
*	@brief 		Prints the position information of all axes.
*	@details	Function to print the actual, command and marker position of all axes.
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkInfoPrintAxesPos()
{
    long axiscnt, axis;

    axiscnt = SYS_INFO(SYS_MAX_AXES);   // Number of axes on controller

    print("Axis Positions:");
    print("    Axis",chr(9),"Actual",chr(9),"Command", chr(9),"Marker");

    for (axis = 0; axis < axiscnt; axis++)
    {
        print("    ",axis,chr(9),Apos(axis),chr(9),Cpos(axis), chr(9),Ipos(axis));
	}
	print("");
	return(1);
}


/**
*	@brief 		Prints the parameters of the position PID controller.
*	@details	Function to print the basic settings of the PID controller.
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkInfoPrintPosPID()
{
	long axisCnt, axis;

    axisCnt = SYS_INFO(SYS_MAX_AXES);   // Number of axes on controller

    print("PID control parameters:");
    print("    Axis",chr(9),"Prop",chr(9),"Int",chr(9),"Diff",chr(9),"Timer",chr(9),"I-Limit");

    for (axis = 0; axis < axisCnt; axis++)
    {
        printf("    %ld\t%ld\t%ld\t%ld\t%ld\t%ld\n",
               axis,
               AXE_PARAM(axis, KPROP),  // Proportional factor
               AXE_PARAM(axis, KINT),   // Integral factor
               AXE_PARAM(axis, KDER),   // Derivative factor
               AXE_PARAM(axis, TIMER),  // Derivative time
               AXE_PARAM(axis, KILIM)); // Integration limit
	}
	print("");
	return(1);
}
