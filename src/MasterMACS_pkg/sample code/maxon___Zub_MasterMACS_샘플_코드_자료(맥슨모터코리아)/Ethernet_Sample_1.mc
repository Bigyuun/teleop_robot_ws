
#include <SysDef.mh>
#include "C:\Users\mmkhych\Desktop\Zub\SDK\ApossC-SDK-V01-12\SDK\SDK_ApossC.mc"

long server_handle ; // Ethernet 통신 위한 Handle 값

dim long array[60];  // Ethernet 통신 수신시, 처음 받는 Raw 데이터 저장 공간
dim long array1[60]; // Ethenet 통신 수신시, 처음 받는 Raw 데이터를 변환하여 저장하는 공간
dim long Recv_data[60]; // Ethernet 통신수신시, 변환한 데이터를 옮겨 저장하는 공간
dim long Pattern_Send_data[32]; // Ethernet 통신 송신시, 송신할 데이터 저장하는 공간

double an1;
double Voltage1;
double an2;
double Voltage2;
double X_Voltage_Offset;
double Y_Voltage_Offset;
double AX2_Voltage_Diff;
double AX1_Voltage_Diff;
long AX1_Error;
long AX2_Error;
long errNo;
long X_Axis_Center_Offset;
long Y_Axis_Center_Offset;
long CV_2;
long CV_1;
long Status;


long length = 32;               //Ethernet 통신시 허용 데이터 길이(byte 단위)
double AX1_POS;
double AX2_POS;
long AX1_POS_LONG;
long AX2_POS_LONG;





#define MY_PORT_NUM       91

void EthernetSendData(void);

void FunctionEthernetHandler(void);
/*********************************************************************
**              State Machine Version of Program
*********************************************************************/

/*********************************************************************
** State Machine Setup Parameters
*********************************************************************/

#pragma SmConfig {    1,  // Runtime flags.
                      46, // Event pool size.
                      10, // Maximum number of timers.
                      5,  // Subscribe pool size.5
                      50, // Param pool size.
                      0,  // Position pool size.
                      2 } // System signal pool size (used for SmSystem.)



/*********************************************************************
** Event Definitions
*********************************************************************/

SmEvent Event_Standby {}
SmEvent TickPeriod_StatusMonitor_Comparison {PRM_CLOCK}
SmEvent TickPeriod_StatusMonitor_Calculation {PRM_CLOCK}
SmEvent SIG_ETH_RECEIVE {handle}
SmEvent SIG_PERIOD {}





SmState EthernetHandling {

	SIG_INIT = {
	            print("SIG_INIT   EhternetHandling");

	            SmSystem (ETHERNET, server_handle, id, SIG_ETH_RECEIVE);
	            SmPeriod(10, id, SIG_PERIOD );

	            return(SmTrans(->DO_EthernetHandling));
	           }

	SIG_ETH_RECEIVE = {

	                   FunctionEthernetHandler();

	                 }

     SIG_PERIOD = {
                       EthernetSendData();
                   }




	SmState DO_EthernetHandling   {

		SIG_ENTRY = {
					print("SIG_ENTRY   EhternetHandling");
					}


    	SIG_EXIT = {
					SmSystem(DELETE, ETHERNET, server_handle, id, SIG_ETH_RECEIVE);
		           }
    }

}

SmState Axis1_2_State  // 1,2번 모터 제어용 루프
{

	SIG_INIT =
	{
	    SmSubscribe(id, SIG_ERROR); // Error Detection 루프

		return(SmTrans(->Standing)); // 이벤트 선언 후 "Standing_AXIS1_2"라는 State로 넘어감.
	}

    SIG_ERROR =
    {



		errNo = ErrorNo();



		return(SmTrans(->Standing));
	}


	SmState Standing   // 위에서 선언한 3개의 이벤트가 발생하는 것을 대기하는 State 루프
	{



	    SIG_IDLE = {




/////////////////////////////Ethernet으로 0x0D 받는 경우//////////////////////////////////////////////////////////////
                     if(array1[0] == 0x0D)
                     {


                     }

/////////////////////////////Ethernet으로 0x0A 받는 경우//////////////////////////////////////////////////////////////
                     if(array1[0] == 0x0A)
                     {

                     }

/////////////////////////////Ethernet으로 0x0C 받는 경우//////////////////////////////////////////////////////////////
                     if(array1[0] == 0x0C)
                     {

                     }

/////////////////////////////Ethernet으로 0x0B 받는 경우//////////////////////////////////////////////////////////////
                     if(array1[0] == 0x0B)
                     {

                     }

/////////////////////////////Ethernet으로 0x05 받는 경우//////////////////////////////////////////////////////////////
                     if(array1[0] == 0x05)
                     {

                     }

/////////////////////////////Ethernet으로 0x07 받는 경우///////////////////////////////////////////////////////////
                     if(array1[0] == 0x07)
                     {

                     }

/////////////////////////////Ethernet으로 0x08 받는 경우///////////////////////////////////////////////////////////
                     if(array1[0] == 0x08)
                     {

                     }

/////////////////////////////Ethernet으로 0x09 받는 경우///////////////////////////////////////////////////////////
                     if(array1[0] == 0x09)
                     {

                     }
/////////////////////////////Ethernet으로 0x04 받는 경우///////////////////
                     if(array1[0] == 0x04)
                     {

                     }
/////////////////////////////Ethernet으로 0x0E 받는 경우///////////////////
                     if(array1[0] == 0x0E)
                     {


                     }
////////////////////////////Ethernet으로 0x1F 받는 경우//////////////////////////////////////////////
                     if(array1[0] == 0x1F )
                     {


                     }
////////////////////////////Ethernet으로 0x3F 받는 경우////////////////////////////////////////////////
                     if(array1[0] == 0x3F)
                     {

                     }
///////////////////////////Ethernet으로 0x5F 받는 경우//////////////////////////////////////
                     if(array1[0] == 0x5F)
                     {


                     }





	               }





    }

    }





/*********************************************************************
** State Machine Definitions
*********************************************************************/
SmMachine Axis1_2StateMachine {1, *, Axis1_2_State, 20, 2}
SmMachine Sm_Ethernet {2,           // State machine ID.
				   *,           // Initialization function.
				   EthernetHandling,        // Top-most state.
				   20,          // Event queue size.
				   2 }          // Private data array size.


long main(void)
{

   long res;




    server_handle = EthernetOpenServer(PROT_TCP, MY_PORT_NUM);


    SYS_PROCESS(SYS_CANOM_MASTERSTATE) = 0;
    GLB_PARAM(CANSYNCTIMER) = 1;

    Delay(500);
	ErrorClear();


   res = SmRun(Axis1_2StateMachine,Sm_Ethernet);

   return(0);
}

void EthernetSendData(void)
{
	long result , i;


    Pattern_Send_data[0] = AX1_Error.ub0;
    Pattern_Send_data[1] = AX1_Error.ub1;
    Pattern_Send_data[2] = AX1_Error.ub2;
    Pattern_Send_data[3] = AX1_Error.ub3;

    Pattern_Send_data[4] = AX2_Error.ub0;
    Pattern_Send_data[5] = AX2_Error.ub1;
    Pattern_Send_data[6] = AX2_Error.ub2;
    Pattern_Send_data[7] = AX2_Error.ub3;

    Pattern_Send_data[8] = errNo.ub0;
    Pattern_Send_data[9] = errNo.ub1;
    Pattern_Send_data[10] = errNo.ub2;
    Pattern_Send_data[11] = errNo.ub3;

    Pattern_Send_data[12] = Status.ub0;
    Pattern_Send_data[13] = Status.ub1;
    Pattern_Send_data[14] = Status.ub2;
    Pattern_Send_data[15] = Status.ub3;

    Pattern_Send_data[16] = X_Axis_Center_Offset.ub0;
    Pattern_Send_data[17] = X_Axis_Center_Offset.ub1;
    Pattern_Send_data[18] = X_Axis_Center_Offset.ub2;
    Pattern_Send_data[19] = X_Axis_Center_Offset.ub3;

    Pattern_Send_data[20] = AX1_POS_LONG.ub0;
    Pattern_Send_data[21] = AX1_POS_LONG.ub1;
    Pattern_Send_data[22] = AX1_POS_LONG.ub2;
    Pattern_Send_data[23] = AX1_POS_LONG.ub3;

    Pattern_Send_data[24] = Y_Axis_Center_Offset.ub0;
    Pattern_Send_data[25] = Y_Axis_Center_Offset.ub1;
    Pattern_Send_data[26] = Y_Axis_Center_Offset.ub2;
    Pattern_Send_data[27] = Y_Axis_Center_Offset.ub3;

    Pattern_Send_data[28] = AX2_POS_LONG.ub0;
    Pattern_Send_data[29] = AX2_POS_LONG.ub1;
    Pattern_Send_data[30] = AX2_POS_LONG.ub2;
    Pattern_Send_data[31] = AX2_POS_LONG.ub3;

    result = EthernetSendTelegram(handle, Pattern_Send_data, length);

    return;
}



////////////////////////////////////////////////////////
void FunctionEthernetHandler(void)
{
	long result , i;

    result = EthernetReceiveTelegram(handle, array);


    for (i= 0; i <= (result-1); i++)
      {
       array1[i] = array[i]&0xFF;


       Recv_data[0].ub0 = array1[0];
       Recv_data[0].ub1 = array1[1];
       Recv_data[0].ub2 = array1[2];
       Recv_data[0].ub3 = array1[3];

       Recv_data[1].ub0 = array1[4];
       Recv_data[1].ub1 = array1[5];
       Recv_data[1].ub2 = array1[6];
       Recv_data[1].ub3 = array1[7];

       Recv_data[2].ub0 = array1[8];
       Recv_data[2].ub1 = array1[9];
       Recv_data[2].ub2 = array1[10];
       Recv_data[2].ub3 = array1[11];

       Recv_data[3].ub0 = array1[12];
       Recv_data[3].ub1 = array1[13];
       Recv_data[3].ub2 = array1[14];
       Recv_data[3].ub3 = array1[15];

       Recv_data[4].ub0 = array1[16];
       Recv_data[4].ub1 = array1[17];
       Recv_data[4].ub2 = array1[18];
       Recv_data[4].ub3 = array1[19];




      }




    return;
}//$X {Status,1,1,0,27,0,184,0,(-1),1},0x2000,185,0
//$X {User Parameter (5),1,1,0,-1,0,-1,0,(-1),-1},0x2201,5,0
//$X {CV_2,1,1,0,25,0,182,0,(-1),1},0x2000,183,0
//$X {Recv_data,1,1,0,9,1,2,1,(0,60),1},0x2102,5,0
//$X {Recv_data,1,1,0,9,1,2,1,(2,60),1},0x2102,7,0
//$X {Recv_data,1,1,0,9,1,2,1,(3,60),1},0x2102,8,0
//$X {Recv_data,1,1,0,9,1,2,1,(1,60),1},0x2102,6,0
//$X {array1,1,1,0,8,1,1,1,(0,60),1},0x2101,5,0
//$X {errNo,1,1,0,22,0,179,0,(-1),1},0x2000,180,0
//$X {AX1_Error,1,1,0,20,0,177,0,(-1),1},0x2000,178,0
//$X {AX2_Error,1,1,0,21,0,178,0,(-1),1},0x2000,179,0
