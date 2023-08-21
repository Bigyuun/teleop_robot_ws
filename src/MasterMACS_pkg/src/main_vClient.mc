/***
* @name : main.mc
* @brief : No class structures, it doesn't available in ApossIDE
***/

#pragma once
#include <SysDef.mh>
//#include "C:\Users\bigyu\Desktop\Github repositories_Bigyuun\EtherCAT_MasterMACS_ws\include\SDK\SDK_ApossC.mc"
#include "..\include\SDK\SDK_ApossC.mc"
#include "..\include\user_definition.mh"
#include "..\include\EtherCAT_config.mc"
#include "..\include\TCP_config.mc"

long i,j,k;
long slaveCount, homingState, slaveState;

// DY
long status_sm = -1;

// State Machine ID set
#define STATE_MACHINE_ID_EtherCAT 	1
#define STATE_MACHINE_ID_TCP		2

// sine wave test parameters
#define DEBUG_FLAG 					0
#define DEBUG_TCP_FLAG				0
#define SINE_WAVE_TEST_FLAG 		0
#define PI 3.1415926
#define SINE_WAVE_NORMAL_TIME		2.0 * PI			// normal
#define SINE_WAVE_TIME  			10000				// time per 1 wave (ms)
#define SINE_WAVE_AMP				10
#define SINE_WAVE_TIMER_FREQ        1000   // 1 kHz
#define SINE_WAVE_TIMER_DURATION    1/SINE_WAVE_TIMER_FREQ  // 1 ms
dim long pos[7]={0}, vel[7]={0}, acc[7]={0};
long cnt_sine_wave = 0;


#if 1
/*********************************************************************
**              State Machine Version of Program
*********************************************************************/

/*********************************************************************
** State Machine Setup Parameters
*********************************************************************/

#pragma SmConfig {    1,        // Runtime flags.
                      500,       // Event pool size.
                      6,        // Maximum number of timers.
                      100,        // Subscribe pool size.
                      100,       // Param pool size.
                      100,        // Position pool size.
                      100 }       // System signal pool size (used for SmSystem.)
/*********************************************************************
** Event Definitions
*********************************************************************/

// EtherCAT
SmEvent SIG_PLAY { }
SmEvent SIG_STOP { }
SmEvent SIG_CLEAR { }
SmEvent SIG_REINIT { }
SmEvent SIG_PREOP { }
SmEvent SIG_OP { }
SmEvent SIG_ENABLE { }
SmEvent SIG_DISABLE { }
SmEvent TICK_EtherCAT_MasterCommand{}
SmEvent TICK_EtherCAT_Callback_SlaveFeedback{}
SmEvent TICK_Configure_SineWave{}

// Ethernet TCP/IP
SmEvent TICK_TCP_ConnStatus{}
SmEvent TCP_RECEIVE_HANDLE{}
SmEvent TCP_RECONNECT{}

/*********************************************************************
** State Definitions
*********************************************************************/


SmState EtherCAT_Handler
{
                SIG_INIT = {
                            SmSubscribe(id, SIG_ERROR);
                            SmParam (0x01220105, 1, SM_PARAM_EQUAL, id, SIG_PLAY);
                            SmParam (0x01220105, 2, SM_PARAM_EQUAL, id, SIG_STOP);
                            SmParam (0x01220105, 3, SM_PARAM_EQUAL, id, SIG_CLEAR);
                            SmParam (0x01220105, 4, SM_PARAM_EQUAL, id, SIG_REINIT);
                            SmParam (0x01220105, 5, SM_PARAM_EQUAL, id, SIG_PREOP);
                            SmParam (0x01220105, 6, SM_PARAM_EQUAL, id, SIG_OP);
                            SmParam (0x01220105, 7, SM_PARAM_EQUAL, id, SIG_ENABLE);
                            SmParam (0x01220105, 8, SM_PARAM_EQUAL, id, SIG_DISABLE);
                            print("State Machine(EtherCAT) Init done");
                            for(i=0;i<NUM_OF_MOTORS;i++) {
                            	printf("#%ld Motor pos : %ld\n", i, Apos(C_AXIS1+i));
                            }

                            return(SmTrans(->Standing));
                            }

                SIG_ERROR = {
                            long errAxis, errNo;
                            errAxis = ErrorAxis();
                            errNo = ErrorNo();
                            print("errAxis : ", errAxis, " / errNo : ", errNo);
                            return(SmTrans(->Standing));
                            }

	SmState Standing
    {
				SIG_ENTRY  = 	{
                                SmPeriod(1, id, TICK_EtherCAT_MasterCommand);
                                USER_PARAM(5) = 3;
                                USER_PARAM(5) = 7;
								}

				SIG_START = 	{
								}

				SIG_IDLE = 		{
								}

                // DY - If TCP state machine receive the message, this timer will be update the value
                // @issue : if use TCP data, synchronization is not exactly same.
                TICK_EtherCAT_MasterCommand =
                			{
                			#if SINE_WAVE_TEST_FLAG
                			for(i=0;i<NUM_OF_MOTORS;i++){
                				Cvel(C_AXIS1+i, pos[0]);
                			}
  							USER_PARAM(5) = 1;		// start moving
  							#else
							for(i=0;i<NUM_OF_MOTORS;i++){
							   Cvel(C_AXIS1+i, target_val[i]);
						    }
						    Sysvar[0x01220105] = 1;		// same as USER_PARAM(5) = 1;
  							#endif



                            }

				TICK_Configure_SineWave = {

							#if SINE_WAVE_TEST_FLAG
							cnt_sine_wave ++;   // increase 1 in every 1 ms
							pos[0] = (double)SINE_WAVE_AMP * sin( SINE_WAVE_NORMAL_TIME * (1.0/(long)SINE_WAVE_TIME)*cnt_sine_wave);         // update global variable 'pos'

							if(cnt_sine_wave >= SINE_WAVE_TIME)
							{
								cnt_sine_wave = 0;
							}

							#endif
							}

				TICK_EtherCAT_Callback_SlaveFeedback = {
								for(i=0;i<NUM_OF_MOTORS;i++) {
								/**
									Apos : data from the virtual machine
									BUSMOD_PROCESS : data from the EtherCAT
									BUSMOD_PROCESS is same as Sysvar(----)
									In Sysvar, just put 0x01, Index(e.g. 4C00), and SubIndex(02)
									In Index 4th number is axis number : 4C0'axis number'
									**/
									printf("#%ld Motor pos : %ld / vel : %ld\n", i, Apos(C_AXIS1+i), Avel(C_AXIS1+i)); //Sysvar(0x014C0002);
								    printf("#%ld Motor pos : %ld / vel : %ld\n", i, BUSMOD_PROCESS(0,PO_BUSMOD_VALUE2), Avel(C_AXIS1+i)); //Sysvar(0x014C0002);
								}
							}	//$B

                SIG_PLAY	  = {
								for(i=0;i<NUM_OF_MOTORS;i++){
									AxisCvelStart(C_AXIS1+i);
								}
                                //AxisCvelStart(AXALL);
                                //Sysvar[0x01220105] = 0;
                            }
                SIG_STOP	  = {
                                print("STOP");
                                AxisStop(AXALL);
                                //Sysvar[0x01220105] = 0;
                            }
                SIG_REINIT =  {
                				return(SmTrans(Standing));
                				}

                SIG_PREOP  =  {
                				return(SmTrans(Standing));
                				}
                SIG_OP 	  =  {
                				return(SmTrans(Standing));
                				}

                SIG_CLEAR  = {
                                print("CLEAR");
                                ErrorClear();
                                AmpErrorClear(AXALL);
                                //Sysvar[0x01220105] = 0;
                            }
                SIG_ENABLE = {
                                print("Motor Enable");
                                AxisControl(AXALL,ON);
                            }
                SIG_DISABLE = {
                                print("Disable");
                                AxisControl(AXALL,OFF);
                                }
    }
}

/***********************************************************************************************/
// DY - TCP/IP Handler
// 2022.11.12 - TICK_Configure_Sinewave test for checking timer in 1 ms
SmState TCPIP_Handler{

	// DY
	SIG_INIT = {
				print("TCP/IP Handler Initializing");
				SmParam (0x01220103, 3, SM_PARAM_EQUAL, id, TCP_RECONNECT);
				return(SmTrans(->Running));
				}
	SmState Running
	{
		// send the data (pos, vel)
		SIG_START = {
					 for(i=0;i<NUM_OF_MOTORS;i++){
						 actual_pos[i]=Apos(C_AXIS1+i);
						 actual_vel[i]=Avel(C_AXIS1+i);

						 // position values
						 sendData[i*8+0]=actual_pos[i].ub0;
						 sendData[i*8+1]=actual_pos[i].ub1;
						 sendData[i*8+2]=actual_pos[i].ub2;
						 sendData[i*8+3]=actual_pos[i].ub3;
						 // velocity values
						 sendData[i*8+4]=actual_vel[i].ub0;
						 sendData[i*8+5]=actual_vel[i].ub1;
						 sendData[i*8+6]=actual_vel[i].ub2;
						 sendData[i*8+7]=actual_vel[i].ub3;

	//					 sendData[i*8+4]=test_c.ub0;
	//					 sendData[i*8+5]=test_c.ub1;
	//					 sendData[i*8+6]=test_c.ub2;
	//					 sendData[i*8+7]=test_c.ub3;

					 }
					 TCP_sendmsg(sendData);
					 //data[0]++;
					}

		SIG_IDLE = 		{
						}

		TICK_TCP_ConnStatus = {data[1] = TCP_get_connection_status();
								#if DEBUG_TCP_FLAG
								print("TCP_status = ", data[1]);
								printf("target_q = %f", target_q[0]);
								#endif
								print("TCP_status = ", data[1]);
								/*
								** @author - DY
								** @brief if connection fail(disconnect any), TCP_RECONNECT function will be operated
								*/
								if(data[1] < 0) {
									print("GO data[1]");
									Sysvar[0x01220103] = 3;		// same as USER_PARMA(3)=3
								}
							  }
		TCP_RECEIVE_HANDLE =  {TCP_receiveHandler();}

		TCP_RECONNECT = {
							print("TCP_RECONNECTION...");
							TCP_close();
							print("CLOSED");
							TCP_client_open();
							print("OPEN");
							//Sysvar[0x01220103] = 0;
						}
    }
}


/*********************************************************************
** State Machine Definitions
*********************************************************************/

//SmMachine Operation {1, *, MyMachine, 20, 2}
// DY
SmMachine SM_EtherCAT {STATE_MACHINE_ID_EtherCAT, init_sm_EtherCAT, EtherCAT_Handler, 400, 200}
SmMachine SM_TCP {STATE_MACHINE_ID_TCP, init_sm_tcp, TCPIP_Handler, 400, 100}


long main(void)
{
	long res;
	print(status_sm);

	print("DY header : ", DY);
	printf("State Machines Run...");
  	status_sm = SmRun(SM_EtherCAT, SM_TCP); // Opearation 이라는 이름의 Statemachine을 실행.

  	// DY
  	// Sm Run 이후는 실행되지 않음.
	if(status_sm==0) {printf("Done");}
	else {printf("#Error during Running State Machines!");}

    return(0);
}

// DY
long init_sm_EtherCAT(long id, long data[])
{
	data[0] = 0;
	//configure_EtherCAT();
	EtherCAT_configuration();

	#if SINE_WAVE_TEST_FLAG
	SmPeriod(1, id, TICK_Configure_SineWave);
	#endif

	#if DEBUG_FLAG
	SmPeriod(500, id, TICK_EtherCAT_Callback_SlaveFeedback);
	#endif

	return(0);
}

// DY
long init_sm_tcp(long id, long data[])
{
	TCP_client_open();
	SmSystem(ETHERNET, socketHandle, STATE_MACHINE_ID_TCP, TCP_RECEIVE_HANDLE);
	SmPeriod(100, id, TICK_TCP_ConnStatus);
	return(0);
}


