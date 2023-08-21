
#include <SysDef.mh>
//#include "C:\Users\mmkhych\Desktop\Zub\SDK\ApossC-SDK-V01-12\SDK\SDK_ApossC.mc"
#include "..\include\SDK\SDK_ApossC.mc"


long server_handle ; // Ethernet 통신 위한 Handle 값

dim long array[60];  // Ethernet 통신 수신시, 처음 받는 Raw 데이터 저장 공간
dim long array1[60]; // Ethenet 통신 수신시, 처음 받는 Raw 데이터를 변환하여 저장하는 공간
dim long Recv_data[60]; // Ethernet 통신수신시, 변환한 데이터를 옮겨 저장하는 공간
dim long Pattern_Send_data[500]; // Ethernet 통신 송신시, 송신할 데이터 저장하는 공간

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


long length = 500; //200개일 때 10ms  //400개일 때 60ms             //Ethernet 통신시 허용 데이터 길이(byte 단위)
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


	           // return(SmTrans(->DO_EthernetHandling));
	           }

	SIG_ETH_RECEIVE = {

	                   FunctionEthernetHandler();

	                 }

     SIG_IDLE = {
                       EthernetSendData();
                   }




//	SmState DO_EthernetHandling   {
//
//		SIG_ENTRY = {
//					print("SIG_ENTRY   EhternetHandling");
//					}
//
//
//    	SIG_EXIT = {
//					SmSystem(DELETE, ETHERNET, server_handle, id, SIG_ETH_RECEIVE);
//		           }
//    }

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




    // server_handle = EthernetOpenServer(PROT_TCP, MY_PORT_NUM);
    server_handle = EthernetOpenClient(PROT_TCP, 172.16.1.5, 1918);
	printf("server_status = %d\n", server_handle);

    SYS_PROCESS(SYS_CANOM_MASTERSTATE) = 0;
    GLB_PARAM(CANSYNCTIMER) = 1;

    Delay(500);
	ErrorClear();

	print("EE");

   res = SmRun(Axis1_2StateMachine,Sm_Ethernet);

   return(0);
}

void EthernetSendData(void)
{
	long result , i;


    Pattern_Send_data[0] = 0x01;
    Pattern_Send_data[0] = (0xff)&(777);
    Pattern_Send_data[1] = 0x02;
    Pattern_Send_data[2] = 0x03;
    Pattern_Send_data[3] = 0x04;

    Pattern_Send_data[4] = 0x05;
    Pattern_Send_data[5] = 0x06;
    Pattern_Send_data[6] = 0x07;
    Pattern_Send_data[7] = 0x08;

    Pattern_Send_data[8] = 0x09;
    Pattern_Send_data[9] = 0x0A;
    Pattern_Send_data[10] = 0x0B;
    Pattern_Send_data[11] = 0x0C;

    Pattern_Send_data[12] = 0x0D;
    Pattern_Send_data[13] = 0x0E;
    Pattern_Send_data[14] = 0x0F;
    Pattern_Send_data[15] = 0x10;

    Pattern_Send_data[16] = 0x11;
    Pattern_Send_data[17] = 0x12;
    Pattern_Send_data[18] = 0x13;
    Pattern_Send_data[19] = 0x14;

    Pattern_Send_data[20] = 0x15;
    Pattern_Send_data[21] = 0x16;
    Pattern_Send_data[22] = 0x17;
    Pattern_Send_data[23] = 0x18;

    Pattern_Send_data[24] = 0x19;
    Pattern_Send_data[25] = 0x1A;
    Pattern_Send_data[26] = 0x1B;
    Pattern_Send_data[27] = 0x1C;

    Pattern_Send_data[28] = 0x1D;
    Pattern_Send_data[29] = 0x1E;
    Pattern_Send_data[30] = 0x1F;
    Pattern_Send_data[31] = 0x20;

    Pattern_Send_data[32] = 0x01;
    Pattern_Send_data[33] = 0x02;
    Pattern_Send_data[34] = 0x03;
    Pattern_Send_data[35] = 0x04;

    Pattern_Send_data[36] = 0x05;
    Pattern_Send_data[37] = 0x06;
    Pattern_Send_data[38] = 0x07;
    Pattern_Send_data[39] = 0x08;

    Pattern_Send_data[40] = 0x09;
    Pattern_Send_data[41] = 0x0A;
    Pattern_Send_data[42] = 0x0B;
    Pattern_Send_data[43] = 0x0C;

    Pattern_Send_data[44] = 0x0D;
    Pattern_Send_data[45] = 0x0E;
    Pattern_Send_data[46] = 0x0F;
    Pattern_Send_data[47] = 0x10;

    Pattern_Send_data[48] = 0x11;
    Pattern_Send_data[49] = 0x12;
    Pattern_Send_data[50] = 0x13;
    Pattern_Send_data[51] = 0x14;

    Pattern_Send_data[52] = 0x15;
    Pattern_Send_data[53] = 0x16;
    Pattern_Send_data[54] = 0x17;
    Pattern_Send_data[55] = 0x18;

    Pattern_Send_data[56] = 0x19;
    Pattern_Send_data[57] = 0x1A;
    Pattern_Send_data[58] = 0x1B;
    Pattern_Send_data[59] = 0x1C;

    Pattern_Send_data[60] = 0x1D;
    Pattern_Send_data[61] = 0x1E;
    Pattern_Send_data[62] = 0x1F;
    Pattern_Send_data[63] = 0x20;

    Pattern_Send_data[64] = 0x01;
    Pattern_Send_data[65] = 0x02;
    Pattern_Send_data[66] = 0x03;
    Pattern_Send_data[67] = 0x04;

    Pattern_Send_data[68] = 0x05;
    Pattern_Send_data[69] = 0x06;
    Pattern_Send_data[70] = 0x07;
    Pattern_Send_data[71] = 0x08;

    Pattern_Send_data[72] = 0x09;
    Pattern_Send_data[73] = 0x0A;
    Pattern_Send_data[74] = 0x0B;
    Pattern_Send_data[75] = 0x0C;

    Pattern_Send_data[76] = 0x0D;
    Pattern_Send_data[77] = 0x0E;
    Pattern_Send_data[78] = 0x0F;
    Pattern_Send_data[79] = 0x10;

    Pattern_Send_data[80] = 0x11;
    Pattern_Send_data[81] = 0x12;
    Pattern_Send_data[82] = 0x13;
    Pattern_Send_data[83] = 0x14;

    Pattern_Send_data[84] = 0x15;
    Pattern_Send_data[85] = 0x16;
    Pattern_Send_data[86] = 0x17;
    Pattern_Send_data[87] = 0x18;

    Pattern_Send_data[88] = 0x19;
    Pattern_Send_data[89] = 0x1A;
    Pattern_Send_data[90] = 0x1B;
    Pattern_Send_data[91] = 0x1C;

    Pattern_Send_data[92] = 0x1D;
    Pattern_Send_data[93] = 0x1E;
    Pattern_Send_data[94] = 0x1F;
    Pattern_Send_data[95] = 0x20;

    Pattern_Send_data[96] = 0x1D;
    Pattern_Send_data[97] = 0x1E;
    Pattern_Send_data[98] = 0x1F;
    Pattern_Send_data[99] = 0x20;


    //////////////////////////////////////////////////

    Pattern_Send_data[100+0] = 0x01;
    Pattern_Send_data[100+1] = 0x02;
    Pattern_Send_data[100+2] = 0x03;
    Pattern_Send_data[100+3] = 0x04;

    Pattern_Send_data[100+4] = 0x05;
    Pattern_Send_data[100+5] = 0x06;
    Pattern_Send_data[100+6] = 0x07;
    Pattern_Send_data[100+7] = 0x08;

    Pattern_Send_data[100+8] = 0x09;
    Pattern_Send_data[100+9] = 0x0A;
    Pattern_Send_data[100+10] = 0x0B;
    Pattern_Send_data[100+11] = 0x0C;

    Pattern_Send_data[100+12] = 0x0D;
    Pattern_Send_data[100+13] = 0x0E;
    Pattern_Send_data[100+14] = 0x0F;
    Pattern_Send_data[100+15] = 0x10;

    Pattern_Send_data[100+16] = 0x11;
    Pattern_Send_data[100+17] = 0x12;
    Pattern_Send_data[100+18] = 0x13;
    Pattern_Send_data[100+19] = 0x14;

    Pattern_Send_data[100+20] = 0x15;
    Pattern_Send_data[100+21] = 0x16;
    Pattern_Send_data[100+22] = 0x17;
    Pattern_Send_data[100+23] = 0x18;

    Pattern_Send_data[100+24] = 0x19;
    Pattern_Send_data[100+25] = 0x1A;
    Pattern_Send_data[100+26] = 0x1B;
    Pattern_Send_data[100+27] = 0x1C;

    Pattern_Send_data[100+28] = 0x1D;
    Pattern_Send_data[100+29] = 0x1E;
    Pattern_Send_data[100+30] = 0x1F;
    Pattern_Send_data[100+31] = 0x20;

    Pattern_Send_data[100+32] = 0x01;
    Pattern_Send_data[100+33] = 0x02;
    Pattern_Send_data[100+34] = 0x03;
    Pattern_Send_data[100+35] = 0x04;

    Pattern_Send_data[100+36] = 0x05;
    Pattern_Send_data[100+37] = 0x06;
    Pattern_Send_data[100+38] = 0x07;
    Pattern_Send_data[100+39] = 0x08;

    Pattern_Send_data[100+40] = 0x09;
    Pattern_Send_data[100+41] = 0x0A;
    Pattern_Send_data[100+42] = 0x0B;
    Pattern_Send_data[100+43] = 0x0C;

    Pattern_Send_data[100+44] = 0x0D;
    Pattern_Send_data[100+45] = 0x0E;
    Pattern_Send_data[100+46] = 0x0F;
    Pattern_Send_data[100+47] = 0x10;

    Pattern_Send_data[100+48] = 0x11;
    Pattern_Send_data[100+49] = 0x12;
    Pattern_Send_data[100+50] = 0x13;
    Pattern_Send_data[100+51] = 0x14;

    Pattern_Send_data[100+52] = 0x15;
    Pattern_Send_data[100+53] = 0x16;
    Pattern_Send_data[100+54] = 0x17;
    Pattern_Send_data[100+55] = 0x18;

    Pattern_Send_data[100+56] = 0x19;
    Pattern_Send_data[100+57] = 0x1A;
    Pattern_Send_data[100+58] = 0x1B;
    Pattern_Send_data[100+59] = 0x1C;

    Pattern_Send_data[100+60] = 0x1D;
    Pattern_Send_data[100+61] = 0x1E;
    Pattern_Send_data[100+62] = 0x1F;
    Pattern_Send_data[100+63] = 0x20;

    Pattern_Send_data[100+64] = 0x01;
    Pattern_Send_data[100+65] = 0x02;
    Pattern_Send_data[100+66] = 0x03;
    Pattern_Send_data[100+67] = 0x04;

    Pattern_Send_data[100+68] = 0x05;
    Pattern_Send_data[100+69] = 0x06;
    Pattern_Send_data[100+70] = 0x07;
    Pattern_Send_data[100+71] = 0x08;

    Pattern_Send_data[100+72] = 0x09;
    Pattern_Send_data[100+73] = 0x0A;
    Pattern_Send_data[100+74] = 0x0B;
    Pattern_Send_data[100+75] = 0x0C;

    Pattern_Send_data[100+76] = 0x0D;
    Pattern_Send_data[100+77] = 0x0E;
    Pattern_Send_data[100+78] = 0x0F;
    Pattern_Send_data[100+79] = 0x10;

    Pattern_Send_data[100+80] = 0x11;
    Pattern_Send_data[100+81] = 0x12;
    Pattern_Send_data[100+82] = 0x13;
    Pattern_Send_data[100+83] = 0x14;

    Pattern_Send_data[100+84] = 0x15;
    Pattern_Send_data[100+85] = 0x16;
    Pattern_Send_data[100+86] = 0x17;
    Pattern_Send_data[100+87] = 0x18;

    Pattern_Send_data[100+88] = 0x19;
    Pattern_Send_data[100+89] = 0x1A;
    Pattern_Send_data[100+90] = 0x1B;
    Pattern_Send_data[100+91] = 0x1C;

    Pattern_Send_data[100+92] = 0x1D;
    Pattern_Send_data[100+93] = 0x1E;
    Pattern_Send_data[100+94] = 0x1F;
    Pattern_Send_data[100+95] = 0x20;

    Pattern_Send_data[100+96] = 0x1D;
    Pattern_Send_data[100+97] = 0x1E;
    Pattern_Send_data[100+98] = 0x1F;
    Pattern_Send_data[100+99] = 0x20;

    ///////////////////////////////////////

        Pattern_Send_data[200+0] = 0x01;
    Pattern_Send_data[200+1] = 0x02;
    Pattern_Send_data[200+2] = 0x03;
    Pattern_Send_data[200+3] = 0x04;

    Pattern_Send_data[200+4] = 0x05;
    Pattern_Send_data[200+5] = 0x06;
    Pattern_Send_data[200+6] = 0x07;
    Pattern_Send_data[200+7] = 0x08;

    Pattern_Send_data[200+8] = 0x09;
    Pattern_Send_data[200+9] = 0x0A;
    Pattern_Send_data[200+10] = 0x0B;
    Pattern_Send_data[200+11] = 0x0C;

    Pattern_Send_data[200+12] = 0x0D;
    Pattern_Send_data[200+13] = 0x0E;
    Pattern_Send_data[200+14] = 0x0F;
    Pattern_Send_data[200+15] = 0x10;

    Pattern_Send_data[200+16] = 0x11;
    Pattern_Send_data[200+17] = 0x12;
    Pattern_Send_data[200+18] = 0x13;
    Pattern_Send_data[200+19] = 0x14;

    Pattern_Send_data[200+20] = 0x15;
    Pattern_Send_data[200+21] = 0x16;
    Pattern_Send_data[200+22] = 0x17;
    Pattern_Send_data[200+23] = 0x18;

    Pattern_Send_data[200+24] = 0x19;
    Pattern_Send_data[200+25] = 0x1A;
    Pattern_Send_data[200+26] = 0x1B;
    Pattern_Send_data[200+27] = 0x1C;

    Pattern_Send_data[200+28] = 0x1D;
    Pattern_Send_data[200+29] = 0x1E;
    Pattern_Send_data[200+30] = 0x1F;
    Pattern_Send_data[200+31] = 0x20;

    Pattern_Send_data[200+32] = 0x01;
    Pattern_Send_data[200+33] = 0x02;
    Pattern_Send_data[200+34] = 0x03;
    Pattern_Send_data[200+35] = 0x04;

    Pattern_Send_data[200+36] = 0x05;
    Pattern_Send_data[200+37] = 0x06;
    Pattern_Send_data[200+38] = 0x07;
    Pattern_Send_data[200+39] = 0x08;

    Pattern_Send_data[200+40] = 0x09;
    Pattern_Send_data[200+41] = 0x0A;
    Pattern_Send_data[200+42] = 0x0B;
    Pattern_Send_data[200+43] = 0x0C;

    Pattern_Send_data[200+44] = 0x0D;
    Pattern_Send_data[200+45] = 0x0E;
    Pattern_Send_data[200+46] = 0x0F;
    Pattern_Send_data[200+47] = 0x10;

    Pattern_Send_data[200+48] = 0x11;
    Pattern_Send_data[200+49] = 0x12;
    Pattern_Send_data[200+50] = 0x13;
    Pattern_Send_data[200+51] = 0x14;

    Pattern_Send_data[200+52] = 0x15;
    Pattern_Send_data[200+53] = 0x16;
    Pattern_Send_data[200+54] = 0x17;
    Pattern_Send_data[200+55] = 0x18;

    Pattern_Send_data[200+56] = 0x19;
    Pattern_Send_data[200+57] = 0x1A;
    Pattern_Send_data[200+58] = 0x1B;
    Pattern_Send_data[200+59] = 0x1C;

    Pattern_Send_data[200+60] = 0x1D;
    Pattern_Send_data[200+61] = 0x1E;
    Pattern_Send_data[200+62] = 0x1F;
    Pattern_Send_data[200+63] = 0x20;

    Pattern_Send_data[200+64] = 0x01;
    Pattern_Send_data[200+65] = 0x02;
    Pattern_Send_data[200+66] = 0x03;
    Pattern_Send_data[200+67] = 0x04;

    Pattern_Send_data[200+68] = 0x05;
    Pattern_Send_data[200+69] = 0x06;
    Pattern_Send_data[200+70] = 0x07;
    Pattern_Send_data[200+71] = 0x08;

    Pattern_Send_data[200+72] = 0x09;
    Pattern_Send_data[200+73] = 0x0A;
    Pattern_Send_data[200+74] = 0x0B;
    Pattern_Send_data[200+75] = 0x0C;

    Pattern_Send_data[200+76] = 0x0D;
    Pattern_Send_data[200+77] = 0x0E;
    Pattern_Send_data[200+78] = 0x0F;
    Pattern_Send_data[200+79] = 0x10;

    Pattern_Send_data[200+80] = 0x11;
    Pattern_Send_data[200+81] = 0x12;
    Pattern_Send_data[200+82] = 0x13;
    Pattern_Send_data[200+83] = 0x14;

    Pattern_Send_data[200+84] = 0x15;
    Pattern_Send_data[200+85] = 0x16;
    Pattern_Send_data[200+86] = 0x17;
    Pattern_Send_data[200+87] = 0x18;

    Pattern_Send_data[200+88] = 0x19;
    Pattern_Send_data[200+89] = 0x1A;
    Pattern_Send_data[200+90] = 0x1B;
    Pattern_Send_data[200+91] = 0x1C;

    Pattern_Send_data[200+92] = 0x1D;
    Pattern_Send_data[200+93] = 0x1E;
    Pattern_Send_data[200+94] = 0x1F;
    Pattern_Send_data[200+95] = 0x20;

    Pattern_Send_data[200+96] = 0x1D;
    Pattern_Send_data[200+97] = 0x1E;
    Pattern_Send_data[200+98] = 0x1F;
    Pattern_Send_data[200+99] = 0x20;

        ///////////////////////////////////////

        Pattern_Send_data[300+0] = 0x01;
    Pattern_Send_data[300+1] = 0x02;
    Pattern_Send_data[300+2] = 0x03;
    Pattern_Send_data[300+3] = 0x04;

    Pattern_Send_data[300+4] = 0x05;
    Pattern_Send_data[300+5] = 0x06;
    Pattern_Send_data[300+6] = 0x07;
    Pattern_Send_data[300+7] = 0x08;

    Pattern_Send_data[300+8] = 0x09;
    Pattern_Send_data[300+9] = 0x0A;
    Pattern_Send_data[300+10] = 0x0B;
    Pattern_Send_data[300+11] = 0x0C;

    Pattern_Send_data[300+12] = 0x0D;
    Pattern_Send_data[300+13] = 0x0E;
    Pattern_Send_data[300+14] = 0x0F;
    Pattern_Send_data[300+15] = 0x10;

    Pattern_Send_data[300+16] = 0x11;
    Pattern_Send_data[300+17] = 0x12;
    Pattern_Send_data[300+18] = 0x13;
    Pattern_Send_data[300+19] = 0x14;

    Pattern_Send_data[300+20] = 0x15;
    Pattern_Send_data[300+21] = 0x16;
    Pattern_Send_data[300+22] = 0x17;
    Pattern_Send_data[300+23] = 0x18;

    Pattern_Send_data[300+24] = 0x19;
    Pattern_Send_data[300+25] = 0x1A;
    Pattern_Send_data[300+26] = 0x1B;
    Pattern_Send_data[300+27] = 0x1C;

    Pattern_Send_data[300+28] = 0x1D;
    Pattern_Send_data[300+29] = 0x1E;
    Pattern_Send_data[300+30] = 0x1F;
    Pattern_Send_data[300+31] = 0x20;

    Pattern_Send_data[300+32] = 0x01;
    Pattern_Send_data[300+33] = 0x02;
    Pattern_Send_data[300+34] = 0x03;
    Pattern_Send_data[300+35] = 0x04;

    Pattern_Send_data[300+36] = 0x05;
    Pattern_Send_data[300+37] = 0x06;
    Pattern_Send_data[300+38] = 0x07;
    Pattern_Send_data[300+39] = 0x08;

    Pattern_Send_data[300+40] = 0x09;
    Pattern_Send_data[300+41] = 0x0A;
    Pattern_Send_data[300+42] = 0x0B;
    Pattern_Send_data[300+43] = 0x0C;

    Pattern_Send_data[300+44] = 0x0D;
    Pattern_Send_data[300+45] = 0x0E;
    Pattern_Send_data[300+46] = 0x0F;
    Pattern_Send_data[300+47] = 0x10;

    Pattern_Send_data[300+48] = 0x11;
    Pattern_Send_data[300+49] = 0x12;
    Pattern_Send_data[300+50] = 0x13;
    Pattern_Send_data[300+51] = 0x14;

    Pattern_Send_data[300+52] = 0x15;
    Pattern_Send_data[300+53] = 0x16;
    Pattern_Send_data[300+54] = 0x17;
    Pattern_Send_data[300+55] = 0x18;

    Pattern_Send_data[300+56] = 0x19;
    Pattern_Send_data[300+57] = 0x1A;
    Pattern_Send_data[300+58] = 0x1B;
    Pattern_Send_data[300+59] = 0x1C;

    Pattern_Send_data[300+60] = 0x1D;
    Pattern_Send_data[300+61] = 0x1E;
    Pattern_Send_data[300+62] = 0x1F;
    Pattern_Send_data[300+63] = 0x20;

    Pattern_Send_data[300+64] = 0x01;
    Pattern_Send_data[300+65] = 0x02;
    Pattern_Send_data[300+66] = 0x03;
    Pattern_Send_data[300+67] = 0x04;

    Pattern_Send_data[300+68] = 0x05;
    Pattern_Send_data[300+69] = 0x06;
    Pattern_Send_data[300+70] = 0x07;
    Pattern_Send_data[300+71] = 0x08;

    Pattern_Send_data[300+72] = 0x09;
    Pattern_Send_data[300+73] = 0x0A;
    Pattern_Send_data[300+74] = 0x0B;
    Pattern_Send_data[300+75] = 0x0C;

    Pattern_Send_data[300+76] = 0x0D;
    Pattern_Send_data[300+77] = 0x0E;
    Pattern_Send_data[300+78] = 0x0F;
    Pattern_Send_data[300+79] = 0x10;

    Pattern_Send_data[300+80] = 0x11;
    Pattern_Send_data[300+81] = 0x12;
    Pattern_Send_data[300+82] = 0x13;
    Pattern_Send_data[300+83] = 0x14;

    Pattern_Send_data[300+84] = 0x15;
    Pattern_Send_data[300+85] = 0x16;
    Pattern_Send_data[300+86] = 0x17;
    Pattern_Send_data[300+87] = 0x18;

    Pattern_Send_data[300+88] = 0x19;
    Pattern_Send_data[300+89] = 0x1A;
    Pattern_Send_data[300+90] = 0x1B;
    Pattern_Send_data[300+91] = 0x1C;

    Pattern_Send_data[300+92] = 0x1D;
    Pattern_Send_data[300+93] = 0x1E;
    Pattern_Send_data[300+94] = 0x1F;
    Pattern_Send_data[300+95] = 0x20;

    Pattern_Send_data[300+96] = 0x1D;
    Pattern_Send_data[300+97] = 0x1E;
    Pattern_Send_data[300+98] = 0x1F;
    Pattern_Send_data[300+99] = 0x20;

            ///////////////////////////////////////

        Pattern_Send_data[400+0] = 0x01;
    Pattern_Send_data[400+1] = 0x02;
    Pattern_Send_data[400+2] = 0x03;
    Pattern_Send_data[400+3] = 0x04;

    Pattern_Send_data[400+4] = 0x05;
    Pattern_Send_data[400+5] = 0x06;
    Pattern_Send_data[400+6] = 0x07;
    Pattern_Send_data[400+7] = 0x08;

    Pattern_Send_data[400+8] = 0x09;
    Pattern_Send_data[400+9] = 0x0A;
    Pattern_Send_data[400+10] = 0x0B;
    Pattern_Send_data[400+11] = 0x0C;

    Pattern_Send_data[400+12] = 0x0D;
    Pattern_Send_data[400+13] = 0x0E;
    Pattern_Send_data[400+14] = 0x0F;
    Pattern_Send_data[400+15] = 0x10;

    Pattern_Send_data[400+16] = 0x11;
    Pattern_Send_data[400+17] = 0x12;
    Pattern_Send_data[400+18] = 0x13;
    Pattern_Send_data[400+19] = 0x14;

    Pattern_Send_data[400+20] = 0x15;
    Pattern_Send_data[400+21] = 0x16;
    Pattern_Send_data[400+22] = 0x17;
    Pattern_Send_data[400+23] = 0x18;

    Pattern_Send_data[400+24] = 0x19;
    Pattern_Send_data[400+25] = 0x1A;
    Pattern_Send_data[400+26] = 0x1B;
    Pattern_Send_data[400+27] = 0x1C;

    Pattern_Send_data[400+28] = 0x1D;
    Pattern_Send_data[400+29] = 0x1E;
    Pattern_Send_data[400+30] = 0x1F;
    Pattern_Send_data[400+31] = 0x20;

    Pattern_Send_data[400+32] = 0x01;
    Pattern_Send_data[400+33] = 0x02;
    Pattern_Send_data[400+34] = 0x03;
    Pattern_Send_data[400+35] = 0x04;

    Pattern_Send_data[400+36] = 0x05;
    Pattern_Send_data[400+37] = 0x06;
    Pattern_Send_data[400+38] = 0x07;
    Pattern_Send_data[400+39] = 0x08;

    Pattern_Send_data[400+40] = 0x09;
    Pattern_Send_data[400+41] = 0x0A;
    Pattern_Send_data[400+42] = 0x0B;
    Pattern_Send_data[400+43] = 0x0C;

    Pattern_Send_data[400+44] = 0x0D;
    Pattern_Send_data[400+45] = 0x0E;
    Pattern_Send_data[400+46] = 0x0F;
    Pattern_Send_data[400+47] = 0x10;

    Pattern_Send_data[400+48] = 0x11;
    Pattern_Send_data[400+49] = 0x12;
    Pattern_Send_data[400+50] = 0x13;
    Pattern_Send_data[400+51] = 0x14;

    Pattern_Send_data[400+52] = 0x15;
    Pattern_Send_data[400+53] = 0x16;
    Pattern_Send_data[400+54] = 0x17;
    Pattern_Send_data[400+55] = 0x18;

    Pattern_Send_data[400+56] = 0x19;
    Pattern_Send_data[400+57] = 0x1A;
    Pattern_Send_data[400+58] = 0x1B;
    Pattern_Send_data[400+59] = 0x1C;

    Pattern_Send_data[400+60] = 0x1D;
    Pattern_Send_data[400+61] = 0x1E;
    Pattern_Send_data[400+62] = 0x1F;
    Pattern_Send_data[400+63] = 0x20;

    Pattern_Send_data[400+64] = 0x01;
    Pattern_Send_data[400+65] = 0x02;
    Pattern_Send_data[400+66] = 0x03;
    Pattern_Send_data[400+67] = 0x04;

    Pattern_Send_data[400+68] = 0x05;
    Pattern_Send_data[400+69] = 0x06;
    Pattern_Send_data[400+70] = 0x07;
    Pattern_Send_data[400+71] = 0x08;

    Pattern_Send_data[400+72] = 0x09;
    Pattern_Send_data[400+73] = 0x0A;
    Pattern_Send_data[400+74] = 0x0B;
    Pattern_Send_data[400+75] = 0x0C;

    Pattern_Send_data[400+76] = 0x0D;
    Pattern_Send_data[400+77] = 0x0E;
    Pattern_Send_data[400+78] = 0x0F;
    Pattern_Send_data[400+79] = 0x10;

    Pattern_Send_data[400+80] = 0x11;
    Pattern_Send_data[400+81] = 0x12;
    Pattern_Send_data[400+82] = 0x13;
    Pattern_Send_data[400+83] = 0x14;

    Pattern_Send_data[400+84] = 0x15;
    Pattern_Send_data[400+85] = 0x16;
    Pattern_Send_data[400+86] = 0x17;
    Pattern_Send_data[400+87] = 0x18;

    Pattern_Send_data[400+88] = 0x19;
    Pattern_Send_data[400+89] = 0x1A;
    Pattern_Send_data[400+90] = 0x1B;
    Pattern_Send_data[400+91] = 0x1C;

    Pattern_Send_data[400+92] = 0x1D;
    Pattern_Send_data[400+93] = 0x1E;
    Pattern_Send_data[400+94] = 0x1F;
    Pattern_Send_data[400+95] = 0x20;

    Pattern_Send_data[400+96] = 0x1D;
    Pattern_Send_data[400+97] = 0x1E;
    Pattern_Send_data[400+98] = 0x1F;
    Pattern_Send_data[400+99] = 0x20;

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
}//$X {Status,1,1,0,29,0,28,0,(-1),1},0x2000,29,0
//$X {User Parameter (5),1,1,0,-1,0,-1,0,(-1),-1},0x2201,5,0
//$X {CV_2,1,1,0,27,0,26,0,(-1),1},0x2000,27,0
//$X {Recv_data,1,1,0,11,1,2,1,(0,60),1},0x2102,5,0
//$X {Recv_data,1,1,0,11,1,2,1,(2,60),1},0x2102,7,0
//$X {Recv_data,1,1,0,11,1,2,1,(3,60),1},0x2102,8,0
//$X {Recv_data,1,1,0,11,1,2,1,(1,60),1},0x2102,6,0
//$X {array1,1,1,0,10,1,1,1,(0,60),1},0x2101,5,0
//$X {errNo,1,1,0,24,0,23,0,(-1),1},0x2000,24,0
//$X {AX1_Error,1,1,0,22,0,21,0,(-1),1},0x2000,22,0
//$X {AX2_Error,1,1,0,23,0,22,0,(-1),1},0x2000,23,0
