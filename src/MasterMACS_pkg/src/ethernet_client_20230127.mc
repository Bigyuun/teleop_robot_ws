/*********************************************************************
**
**    This sample provides a prototype of an Aposs state machine
** program.  The state machine consists of two substates.  A timer
** is used to toggle between the substates.
*/
#pragma once
#include <SysDef.mh>
#include "..\include\SDK\SDK_ApossC.mc"
#include "..\include\user_definition.mh"
#include "..\include\EtherCAT_config.mc"

/*********************************************************************
** Application Defines
*********************************************************************/

/*********************************************************************
** State Machine Setup Parameters
*********************************************************************/

/*********************************************************************
** State Definitions
*********************************************************************/
long socketHandle, status = 0;
long receiveData[512] = {0};
wchar charArray[512] = {0};
long retVal=0;
long n=0;

/*********************************************************************
** State Machine Initialization Functions
*********************************************************************/
void interrupt_test(void)
{
	print("Up to date");
	return;
}

void EthernetHandler(void)
{
	retVal = EthernetReceiveTelegram(socketHandle, receiveData);
	//retVal = EthernetReceiveTelegram(socketHandle, charArray);
	//print("first byte: %ld, second byte: %ld", receiveData[0], receiveData[1]);


	//print(charArray); //print the whole array
	print(receiveData);
	return;
}

/*********************************************************************
** Aposs Main Program
*********************************************************************/
long main(void)
{
    /*
    ** Start the state machine.
    */
	long Sdata[512];
	wchar Swdata[512] = "Hi daeyun";
	long rrV;
	//long handle = EthernetOpenServer(PROT_TCP, 9800);
	//print("TCP return : ", handle);


//	socketHandle = EthernetOpenServer(PROT_TCP, 77777);
	socketHandle = EthernetOpenClient(PROT_TCP, 172.16.1.5 ,77777);

	InterruptSetup(ETHERNET, EthernetHandler, socketHandle);
	InterruptSetup(TIME, interrupt_test, 1000);

	if(socketHandle < 0) printf("There was an error: %ld \r\n", socketHandle);
	else printf("Success. The handle is: %ld \r\n", socketHandle);

	//InterruptEnable(ALL);
	/*
	* status of ethernet connection
	SOCK_STATUS_INIT = 0,
	SOCK_STATUS_WAITING = 1,
	SOCK_STATUS_CONNECTING = 2,
	SOCK_STATUS_READY = 3,
	SOCK_STATUS_CLOSED = 4,
	SOCK_STATUS_ERRORSENDING = -1,
	SOCK_STATUS_ERROR = -2
	*/
	status = EthernetGetConnectionStatus(socketHandle);

	print("status : ", status);



	while(1)
	{
	/*
	This command can be used to receive a telegram if there is one in the buffer.
	The socket connection previously has to be setup with EthernetOpenClient or EthernetOpenServer. The connection status has to be SOCK_STATUS_READY = 3.
	*/
	rrV++;
	if(rrV<=1000)
	{
		retVal = EthernetSendTelegram(socketHandle, Swdata, arraylen(Swdata));
	}
	else
	{
		print("done");
	}

	//Delay(1);
	}


    return(0);
}
