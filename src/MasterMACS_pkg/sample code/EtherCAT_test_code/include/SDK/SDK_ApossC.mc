/**
*	@file		SDK_ApossC.mc
*	@brief		File to include the entire SDK.
*	@details	This file can be included to include the entire SDK. The folder "SDK" must be on the same level as the used program.
*				In order to keep the memory requirements of the SDK as low as possible, the compiler must be set to include only referenced files.
*				The procedure for this is shown in detail in the section @ref HowToUse.
*	@example 	HowToUse_entire_SDK.mc
*	@example 	HowToUse_individual_SDK.mc
*	$Revision: 224 $
*/

//Include all SDK amplifier files
#include "Amplifier\SDK_Amplifier_MACS.mc"
#include "Amplifier\SDK_Amplifier_MotorAlignment.mc"
#include "Amplifier\SDK_Amplifier_MotorCommissioning.mc"

// Include all SDK DS402 amplifier
#include "Amplifier\SDK_Amplifier_Epos4.mc"
#include "Amplifier\SDK_Amplifier_MiniMACS6_DS402_Slave.mc"

//Include all SDK axis files
#include "Axis\SDK_Axis_Setup.mc"

//Include all SDK bussystem files
#include "BusSystem\SDK_Bussystem_EtherCat.mc"

//Include all SDK communication files
#include "Communication\SDK_Communication_Ethernet.mc"

//Include all SDK encoder files
#include "Encoder\SDK_Encoder_Setup.mc"

//Include all SDK information files
#include "Information\SDK_Information_General.mc"

//Include all SDK kinematics files
#include "Kinematics\SDK_Kinematics_Setup.mc"

//Include all SDK miscellaneous files
#include "Miscellaneous\SDK_Miscellaneous_Recording.mc"
#include "Miscellaneous\SDK_Miscellaneous_IO.mc"

//Include all SDK motion files
#include "Motion\SDK_Motion_Movement.mc"

//Include all SDK virtual system files
#include "VirtualModule\SDK_VirtualModule_AxisSetup.mc"
#include "VirtualModule\SDK_VirtualModule_MasterSetup.mc"

