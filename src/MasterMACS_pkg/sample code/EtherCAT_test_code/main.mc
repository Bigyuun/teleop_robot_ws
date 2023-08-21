
#pragma once
#include <SysDef.mh>
//#include "C:\Users\bigyu\Desktop\Github repositories_Bigyuun\EtherCAT_MasterMACS_ws\include\SDK\SDK_ApossC.mc"
#include ".\include\SDK\SDK_ApossC.mc"
#include ".\include\EtherCAT_config.mc"

long slaveCount, i,j,k, homingState,slaveState;

// DY
long status_sm = -1;

// State Machine ID set
#define STATE_MACHINE_ID_EtherCAT 	1

// sine wave test parameters
#define DEBUG_FLAG 					1
#define SINE_WAVE_TEST_FLAG 		1
#define PI 3.1415926
#define SINE_WAVE_NORMAL_TIME		2.0 * PI			// normal
#define SINE_WAVE_TIME  			10000				// time per 1 wave (ms)
#define SINE_WAVE_AMP				10
#define SINE_WAVE_TIMER_FREQ        1000   // 1 kHz
#define SINE_WAVE_TIMER_DURATION    1/SINE_WAVE_TIMER_FREQ  // 1 ms

dim double pos[7]={0}, vel[7]={0}, acc[7]={0};
long cnt_sine_wave = 0;


#if 1
/*********************************************************************
**              State Machine Version of Program
*********************************************************************/

/*********************************************************************
** State Machine Setup Parameters
*********************************************************************/

#pragma SmConfig {    1,        // Runtime flags.
                      1000,       // Event pool size.
                      6,        // Maximum number of timers.
                      1000,        // Subscribe pool size.
                      1000,       // Param pool size.
                      1000,        // Position pool size.
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

/*********************************************************************
** State Definitions
*********************************************************************/
SmState EtherCAT_Handler
{
                SIG_INIT = {
                            //long cnt = 0;
                            //print(status_sm) 	// 전역변수 전달 필요
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
                            print("pos[0] = ", pos[0]);
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
                                print("into the Standing State ");
                                print("Default User Parameter(USER_PARAM) = ", USER_PARAM(5));
                                USER_PARAM(5) = 7;
								}

				SIG_START = 	{

								}

				SIG_IDLE = 		{
								}

                // DY - If TCP state machine receive the message, this timer will be update the value
                TICK_EtherCAT_MasterCommand =
                			{
                			#if SINE_WAVE_TEST_FLAG
                			for(i=0;i<g_NUM_OF_SLAVES;i++){
                				Cvel(C_AXIS1+i, pos[0]);
                			}
  							USER_PARAM(5) = 1;		// start moving
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
							print("pos : ", Apos(C_AXIS1));
							print("pos : ", Apos(C_AXIS2));
							}

                SIG_PLAY	  = {
								for(i=0;i<g_NUM_OF_SLAVES;i++){
									AxisCvelStart(C_AXIS1+i);
								}
                                //AxisCvelStart(AXALL);
                                Sysvar[0x01220105] = 0;
                            }
                SIG_STOP	  = {
                                print("STOP");
                                AxisStop(AXALL);
                                Sysvar[0x01220105] = 0;
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
                                Sysvar[0x01220105] = 0;
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
// DY - TCP/IP Handler -> 2 timer(SmPeriod 1ms). each timer preceed the receive and send the data
// 2022.11.12 - TICK_Configure_Sinewave test for checking timer in 1 ms

/*********************************************************************
** State Machine Definitions
*********************************************************************/

//SmMachine Operation {1, *, MyMachine, 20, 2}
// DY
SmMachine SM_EtherCAT {STATE_MACHINE_ID_EtherCAT, init_sm_EtherCAT, EtherCAT_Handler, 400, 20}

long main(void)
{
	long res;
	print(status_sm);
	printf("State Machines Run...");
  	status_sm = SmRun(SM_EtherCAT); // Opearation 이라는 이름의 Statemachine을 실행.

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

