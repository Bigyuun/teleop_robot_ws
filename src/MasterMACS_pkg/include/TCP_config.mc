#pragma once
#include <SysDef.mh>
#include "..\include\SDK\SDK_ApossC.mc"
#include "user_definition.mh"
#include "TCP_definition.mh"



/*********************************************************************
** State Machine Setup Parameters
*********************************************************************/

/*********************************************************************
** State Definitions
*********************************************************************/
long TCP_get_connection_status(void);
void TCP_receiveHandler(void);
long TCP_client_open(void);
long TCP_server_open(void);
long TCP_sendmsg(long sendmsg[]);
long TCP_close(void);
/*********************************************************************
** Initialization Functions
*********************************************************************/

/** status of ethernet connection
SOCK_STATUS_INIT = 0,
SOCK_STATUS_WAITING = 1,
SOCK_STATUS_CONNECTING = 2,
SOCK_STATUS_READY = 3,
SOCK_STATUS_CLOSED = 4,
SOCK_STATUS_ERRORSENDING = -1,
SOCK_STATUS_ERROR = -2
*/
long TCP_get_connection_status(void)
{
	long i;
	status = EthernetGetConnectionStatus(socketHandle);

	switch(status){

		case SOCK_STATUS_INIT:
			print("SOCKET_STATUS_INIT");
			break;
		case SOCK_STATUS_WAITING:
			for(i=0; i<NUM_OF_MOTORS;i++) {
				target_val[i]=0;
			}
			print("SOCKET_STATUS_WAITING");
			break;
		case SOCK_STATUS_CONNECTING:
			print("SOCKET_STATUS_CONNECTING");
			break;
		case SOCK_STATUS_READY:
			print("SOCKET_STATUS_READY");
			break;
		case SOCK_STATUS_CLOSED:
			for(i=0; i<NUM_OF_MOTORS;i++) {
				target_val[i]=0;
			}
			print("SOCKET_STATUS_CLOSED");
			break;
		case SOCK_STATUS_ERRORSENDING:
			for(i=0; i<NUM_OF_MOTORS;i++) {
				target_val[i]=0;
			}
			print("SOCKET_STATUS_ERRORSENDING");
			break;
		case SOCK_STATUS_ERROR:
			for(i=0; i<NUM_OF_MOTORS;i++) {
				target_val[i]=0;
			}
			TCP_close();
			printf("Socket Error! Check the connection (Error value : %ld). Reconnecting... \n", status);
			break;
	}
	return status;
}

void TCP_receiveHandler(void)
{
	long i;

	retVal = EthernetReceiveTelegram(socketHandle, receiveData);

	/**
	** @author DY
	** @brief Just input the value when TCP is READY(connected)
	**/
	if (status == SOCK_STATUS_READY) {
		for(i=0; i<NUM_OF_MOTORS;i++) {
			target_val[i].ub0 = receiveData[BUFFER_TYPE*i+0];
			target_val[i].ub1 = receiveData[BUFFER_TYPE*i+1];
			target_val[i].ub2 = receiveData[BUFFER_TYPE*i+2];
			target_val[i].ub3 = receiveData[BUFFER_TYPE*i+3];

			target_velocity_profile[i].ub0 = receiveData[BUFFER_TYPE*NUM_OF_MOTORS + BUFFER_TYPE*i+0];
			target_velocity_profile[i].ub1 = receiveData[BUFFER_TYPE*NUM_OF_MOTORS + BUFFER_TYPE*i+1];
			target_velocity_profile[i].ub2 = receiveData[BUFFER_TYPE*NUM_OF_MOTORS + BUFFER_TYPE*i+2];
			target_velocity_profile[i].ub3 = receiveData[BUFFER_TYPE*NUM_OF_MOTORS + BUFFER_TYPE*i+3];
		}
	}
	//else {
	//	for(i=0; i<NUM_OF_MOTORS;i++){
	//		target_val[i]=0;
	//	}
	//	//print("TCP socket not READY");
	//}
	return;
}

/*********************************************************************
** Aposs Main Program
*********************************************************************/
long TCP_client_open(void)
{
	socketHandle = EthernetOpenClient(PROT_TCP, g_IP, g_PORT);

	if(socketHandle < 0) printf("SOCKET error: %ld \r\n", socketHandle);
	else printf("SOCKET OPEN Success. The handle is: %ld \r\n", socketHandle);

	return(1);
}

long TCP_server_open(void){
	socketHandle = EthernetOpenServer(PROT_TCP, g_PORT);

	if (socketHandle < 0) printf("SOCKET error: %ld \r\n", socketHandle);
	else printf("SOCKET OPEN Success. The handle is: %ld \r\n", socketHandle);

	return(1);
}

long TCP_sendmsg(long sendmsg[])
{
	retVal = EthernetSendTelegram(socketHandle, sendmsg, BUFFER_SIZE);
	return retVal;
}

long TCP_close(void)
{
	EthernetClose(socketHandle);
	print("SOCKET closed");
	return 0;
}





