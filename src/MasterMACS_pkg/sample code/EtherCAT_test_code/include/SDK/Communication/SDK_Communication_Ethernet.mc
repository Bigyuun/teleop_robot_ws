/**
*	@file		SDK_Communication_Ethernet.mc
*	@brief		Functions to work with an Ethernet as communication
*	$Revision: 198 $
*
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Communication_Ethernet.mh"

/**
*	@brief 		Sends a string as ethernet telegram
*	@details	Simplified usage of the function EthernetSendTelegram() and can be used to send a string.
* 	@param 		sendArray		Array which contains the data to be sent
* 	@param 		handle			The handle returned by the EthernetOpenClient() or EthernetOpenServer() command.
*	@return 	value:	See list in Ethernet Commands or use @ref sdkEthernetPrintGeneralResult() \n
*				value 	> 0 -	\n
*				value 	= 0 No error, everything OK	\n
*				value 	< 0 Error
*	@example 	EthernetSocket_OpenServer.mc
*	@example 	EthernetSocket_OpenClient.mc
*/
long sdkEthernetSendString(wchar sendArray[], long handle)
{
	long result;

	print("sdkEthernetSendString: ", sendArray);
    result = EthernetSendTelegram(handle, sendArray, arraylen(sendArray));

    return(result);
}


/**
*	@brief 		Prints the return values of the Ethernet connection status
*	@details	Prints the return values of the Ethernet connection status from the list
*				"EthernetGetConnectionStatus" of the APossIDE help.
* 	@param -
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*	@example 	EthernetSocket_OpenServer.mc
*	@example 	EthernetSocket_OpenClient.mc
*/
long sdkEthernetPrintConnectionStatus(long status)
{
	switch(status)
	{
		case -2:	print("SOCK_STATUS_ERROR ");
					break;
		case -1:	print("SOCK_STATUS_ERRORSENDING");
					break;
		case 0:		print("SOCK_STATUS_INIT");
					break;
		case 1:		print("SOCK_STATUS_WAITING");
					break;
		case 2:		print("SOCK_STATUS_CONNECTING");
					break;
		case 3:		print("SOCK_STATUS_READY");
					break;
		case 4:		print("SOCK_STATUS_CLOSED");
					break;
		default:	print("SOCK_UNKNOW_STATUS: ",status);
					break;
	}
    return(1);
}


/**
*	@brief 		Prints the general return values of the Ethernet commands
*	@details	Prints the general return values of the Ethernet commands from the list
*				"Ethernet Commads" of the APossIDE help.
* 	@param 		result			Return value of an Ethernet command like for example EthernetSendTelegram()
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*	@example 	EthernetSocket_OpenServer.mc
*	@example 	EthernetSocket_OpenClient.mc
*/
long sdkEthernetPrintGeneralResult(long result)
{
	printf("sdkEthernetPrintGeneralResult: ");
	switch(result)
	{
		case 0:		print("No error, everything OK");
					break;
		case -1:	print("Out of memory error");
					break;
		case -2:	print("Buffer error");
					break;
		case -3:	print("Timeout");
					break;
		case -4:	print("Routing problem");
					break;
		case -5:	print("Operation in progress");
					break;
		case -6:	print("Illegal value");
					break;
		case -7:	print("Operation would block");
					break;
		case -8:	print("Address in use");
					break;
		case -9:	print("Already connected");
					break;
		case -10:	print("Connection aborted");
					break;
		case -11:	print("Connection reset");
					break;
		case -12:	print("Connection closed");
					break;
		case -13:	print("Not connected");
					break;
		case -14:	print("Illegal argument");
					break;
		case -115:	print("Low-level netif error");
					break;
		case -21:	print("Invalid handle has been passed");
					break;
		case -22:	print("Limit of socket connection exceeded");
					break;
		case -23:	print("The socket is not connected");
					break;
		case -24:	print("Too much data has been attempted to send");
					break;
		case -25:	print("Previous data not yet sent");
					break;
		case -26:	print("Previously received data has not been read and new data has arrived");
					break;
		case -27:	print("Received telegram is too long for the internal buffer");
					break;
		case -28:	print("This controller does not support the socket feature");
					break;
		case -29:	print("The requested protocol type is not supported");
					break;
		default:	print("Unknown result: ",result);
					break;
	}
    return(1);
}
