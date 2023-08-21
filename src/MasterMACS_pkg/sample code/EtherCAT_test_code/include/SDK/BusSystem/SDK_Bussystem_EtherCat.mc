/**
*	@file		SDK_Bussystem_EtherCat.mc
*	@brief		Functions to work with an EtherCat Master/ slave
*	$Revision: 228 $
*
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Bussystem_EtherCat.mh"

/**
*	@brief 		Initialization of an EtherCAT master
*	@details	Initializes an EtherCAT master and scans the EtherCat bus for slaves.
*				The slaves are now in PREOP state.
* 	@param 		-
*	@return 	value:	Number of slaves found on bus \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkEtherCATMasterInitialize(void)
{
    long slaveCount;

    ECatMasterCommand(0x1000, 0);				// disable Master
    ECatMasterCommand(0x1000, 1);				// start Master
    slaveCount = ECatMasterInfo(0x1000, 0);		// slave count

    print(" ECAT salve count: ",slaveCount);
    return(slaveCount);
}

/**
*	@brief 		Mapping of EtherCAT Master IO Image
*	@details	Scans through the slave's PDO setup and generates mapping.
*               The slaves PDO setup has to be done by SDO commands before this
*               function is called.
*				The slaves are SAFEOP state after this function.
* 	@param 		-
*	@return 	value:	Number of slaves found on bus \n
*				value 	> 0 Process successful 	\n
*				value 	< 0 Error
*/
long sdkEtherCATMasterDoMapping(void)
{
	print("map input/output");
    return ECatMasterCommand(0x1000, 2);	//  Map Input and Output buffers (go to safeop)
}


/**
*	@brief 		Sets the DC cycle for an individual slave
*	@details	DC0 clock output will be configured with the selected cycle time and
*               offset.
*				Typically, Cycle time is 1ms and offset 0.
*               This function has to be called after sdkEtherCATMasterDoMapping.
* 	@param 		slaveNo			ID of the EhterCAT slave
* 	@param 		cycleTime_ms	cycletime in milliseconds
* 	@param 		offset_us		shift offset
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkEtherCATSetupDC(long slaveNo, long cycleTime_ms, long offset_us)
{
    ECatMasterCommand(slaveNo, ( (cycleTime_ms<<8) + (offset_us<<16)));

    return(1);
}


/**
*	@brief 		Start an EtherCAT Master
*	@details	If DC is configured, this function will block until the slaves are
*               synchronized to the master. This can take some seconds.
*				After that the system waits until all slaves are in the OP state.
* 	@param -
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkEtherCATMasterStart(void)
{
	long offsetValue;
	long dcActive;

	Delay(2);	//Wait for ECatMasterInfo(0x1000, 12) value to be updated the first time

	dcActive = ECatMasterInfo(0, 30);	//is DC configured?
	print("DC Active: ", dcActive);

	if(dcActive)
	{
		print("wait for DC to lock");
		offsetValue = 100001;
		while(abs(offsetValue) > 1000)
		{
			offsetValue = ECatMasterInfo(0x1000, 12);
			print(offsetValue);
			Delay(300);
		}
	}

    print("wait for op");
    ECatMasterCommand(0x1000, 3);	//  request & wait OP state for all slaves

    return(1);
}



