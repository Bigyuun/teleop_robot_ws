/**
*	@brief		This is a testprogram which demonstrates the use of the EPOS4 SDK
*	@detail		The drive will run back and forth when the program is started. It will also
*				record some test data. This data can be displayed by pressing the "Oscilloscope(Record)"
*				button after the program has run.
*
*	$Revision: 30 $
*
*	@example 	ECAT_3Ax_EPOS4-Test_csp.mc
*
*/
//#include "ApossC_SDK_V01.05\SDK\SDK_ApossC.mc"
#include "Impec_210409.mh" // 참조할 헤더 파일
dim long MOTOR_Write_Array[100]; // Index 번호를 0x2100으로 가지고 있는 Array
dim long MOTOR_Read_Array[100];  // Index 번호를 0x2101로 가지고 있는 Array


#define C_AXIS1	0						// Axis Module No. 1번 축은 0. 이 모듈 번호는 셋업하는 함수나, 모션 함수에서 사용이 됨.
#define C_AXIS2 1                       // Axis Module No. 2번 축은 1. 이 모듈 번호 역시 셋업하는 함수나, 모션 함수에서 사용이 됨.

#define C_DRIVE_BUSID1 1000001			// EtherCAT 통신 시에 사용이 되는 Network ID. 1번 축부터 1000001 로 시작.
#define C_DRIVE_BUSID2 1000002          // EtherCAT 통신 시에 사용이 되는 Network ID. 2번 축은 1000002로 시작.

#define C_ENC_QC_1 512*4               // 1번 축의 엔코더 분해능 값. 현재 모터에 장착되어 있는 엔코더 의 분해능은 512 이지만, 드라이브에서 4체배를 시키기 때문에 512 * 4 인 2048이 모터 엔코더 1회전 분해능.
#define C_ENC_QC_2 512*4               // 2번 축의 엔코더도 1번 축의 제품과 동일한 사양.

#define C_EC_CYCLE_TIME	1				// Cycletime in milliseconds -> EtherCAT 통신 시에 사용이 되는 Cycle time 1ms -> 이 항목은 변경 필요 없음
#define C_EC_OFFSET		1				// Shift offset -> EtherCAT 통신 시에 사용이 되는 Offset 값.이 항목은 변경 필요 없음

#define C_PDO_NUMBER	1				// Used PDO number -> 이 항목은 변경 필요 없음
#define C_OP_MODE_CSP   1               // EPOS4의 Operation Mode를 CSP 모드로 결정하여 사용하는데 쓰는 인자.이는 아래 셋업하는 함수에서 사용하는 인자

#define C_MOTOR_MAX_RPM		6600		// Maximum velocity in RPM

#define C_AXISPOLARITY		1			// Definition of the polarity 0: Normal, 1: Inverse //

#define VEL_ACC 30 // 모션 함수에서 사용이 되는 가속도의 값. 단위는 %
#define VEL_DEC 30 // 모션 함수에서 사용이 되는 감속도의 값. 단위는 %

#define PAUSETIME 3000 // 테스트 용 PAUSE TIME 실제로는 아래 제시된 Array에 데이터를 적용하여야 함.

#define SMDATA_SAVEARRAY      0
#define POSREACHED 3
//////////////////////////////////////////////////
/////////////////Writing Array////////////////////
//////////////////////////////////////////////////
#define MOTOR1_MODE Sysvar[0x01210005]           // Motor1 의 Continuous Mode : 0 / Quantity Mode : 1
#define MOTOR1_SPEED Sysvar[0x01210006]          // Motor1 속도 입력
#define MOTOR1_SUCKBACK_DEGREE Sysvar[0x01210007]// Motor1 SuckBack 각도 입력
#define MOTOR1_PAUSETIME Sysvar[0x01210008]      // Motor1 SuckBack 전 PAUSE TIME 입력
#define MOTOR1_QUANTITY_DEGREE Sysvar[0x01210009]// Motor1 Quantity Mode 시 동작 회전 각도 입력

#define MOTOR2_MODE Sysvar[0x0121000A]           // Motor2 의 Continuous Mode : 0 / Quantity Mode : 1
#define MOTOR2_SPEED Sysvar[0x0121000B]          // Motor2 속도 입력
#define MOTOR2_SUCKBACK_DEGREE Sysvar[0x0121000C]// Motor2 SuckBack 각도 입력
#define MOTOR2_PAUSETIME Sysvar[0x0121000D]      // Motor2 SuckBack 전 PAUSE TIME 입력
#define MOTOR2_QUANTITY_DEGREE Sysvar[0x0121000E]// Motor2 Quantity Mode 시 동작 회전 각도 입력

#define Error Sysvar[0x0121000F]                 // Error Reset 1 입력 /

#define MOTOR1_SUCKBACK_LIMIT Sysvar[0x01210010] // Motor1 SuckBack 시 회전 각도 Limit 값 입력 Limit 값이 360일 때 Suckback degree에 360 이상 넣지 못함. 넣게 된다면 Limit값으로 넣어짐.
#define MOTOR2_SUCKBACK_LIMIT Sysvar[0x01210011] // Motor2 SuckBack 시 회전 각도 Limit 값 입력

#define SAVEDATAROM Sysvar[0x01210012]           // 1 입력 시 SAVE DATA

///////////////////////////////////////////////////
////////////////Reading Array//////////////////////
///////////////////////////////////////////////////
#define MOTOR1_DONE Sysvar[0x01210105].i1        // Motor1 동작 완료 Ack 신호 bit i는 0부터 시작
#define MOTOR2_DONE Sysvar[0x01210105].i2        // Motor2 동작 완료 Ack 신호
#define MOTOR1_Actual_SPEED Sysvar[0x01210106]   // Motor1의 실제 속도 모니터링
#define MOTOR2_Actual_SPEED Sysvar[0x01210107]   // Motor2의 실제 속도 모니터링
#define MOTOR1_ZUB_ERROR Sysvar[0x01210108]      // Motor1 StateMachine에서 발생하는 Zub Controller의 Error 코드
#define MOTOR1_EPOS4_ERROR Sysvar[0x01210109]    // Motor1 StateMachine에서 발생하는 EPOS4의 Error 코드
#define MOTOR2_ZUB_ERROR Sysvar[0x0121010A]      // Motor2 StateMachine에서 발생하는 Zub Controller의 Error 코드 - 위와 동일.
#define MOTOR2_EPOS4_ERROR Sysvar[0x0121010B]    // Motor2 StateMachine에서 발생하는 EPOS4의 Error 코드
#define EtherCAT_Slave_Count Sysvar[0x01210105].ub2 // Zub Controller와 연결되어 있는 EtherCAT Slave의 수 (Slave는 EPOS4)

#define ErrorReset_Mask 1                           // ErrorReset 이벤트의 Mask 값.
#define Save_Mask 1                                 // Save 이벤트의 Mask 값

double NumberOfTurn,NumberOfTurn_QUANTITY,MOTOR1_RPM,NumberOfTurn_MOTOR2,NumberOfTurn_QUANTITY_MOTOR2,MOTOR2_RPM ; // 소수점 값을 가진 변수 설정

long GearRatio,i,res,slaveCount,DI1,DI2; // 32bit integer 값을 가진 변수 설정

#pragma SmConfig {1,       // runflag : Delay에 유무에 따라서 1 혹은 0으로.
                  25,      // Event 의 갯수
                  6,       // Timer 의 갯수
                  1,
                  12,
                  0,
                  2}



SmEvent SIG_TARGET_REACHED {}                       // Motor1번의 위치 제어 후 위치결정완료 플래그 이벤트
SmEvent SIG_TARGET_REACHED_2 {}                     // Motor2번의 위치 제어 후 위치결정완료 플래그 이벤트
SmEvent TickPeriod {PRM_CLOCK}                      // Motor1 StateMachine내에서 사용하는 "TickPeriod"라는 이름을 가진 Timer  //
SmEvent TickPeriod_1 {PRM_CLOCK}                    // Motor2 StateMachine내에서 사용하는 "TickPeriod_1"이라는 이름을 가진 Timer
SmEvent TickSingle_DisCharge_Delay { PRM_CLOCK }    // Motor1 StateMachine내에서 사용하는 "TickSingle_DisCharge_Delay"라는 이름을 가진 Timer
SmEvent TickSingle_DisCharge_Delay_1 { PRM_CLOCK }  // Motor2 StateMachine내에서 사용하는 "TickSingle_DisCharge_Delay_1"라는 이름을 가진 Timer
SmEvent Input1Rising  { PRM_NUMBER, PRM_VALUE }     // Motor1 기동을 위한 Digital Input1번 신호 Rising Edge 이벤트   // PRM_NUMBER , PRM_VALUE의 출처
SmEvent Input1Falling  { PRM_NUMBER, PRM_VALUE }    // Motor1 정지를 위한 Digital Input1번 신호 Falling Edge 이벤트
SmEvent Input2Rising { PRM_NUMBER, PRM_VALUE }      // Motor2 기동을 위한 Digital Input2번 신호 Rising Edge 이벤트
SmEvent Input2Falling { PRM_NUMBER, PRM_VALUE }     // Motor2 정지를 위한 Digital Input2번 신호 Falling Edge 이벤트
SmEvent Error_Reset {}                              // Error Reset 이벤트
SmEvent SAVEARRAY {}                                // SAVE Array 이벤트

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////Motor1 State Machine//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SmState Motor1 {
               SIG_INIT = {
                            SmSubscribe(id, SIG_ERROR); // Error 관리 StateMachine

                            SmParam (0x0125007B, SM_STAT_POSREACHED, SM_PARAM_RISING, id, SIG_TARGET_REACHED); // Motor1의 위치결정완료 신호 이벤트 (이벤트 이름 : SIG_TARGET_REACHED) //SM_STAT 나온 부분
                            SmInput(1, SM_INPUT_RISING,  id, Input1Rising);                                    // Motor1의 Digital Input 1번 Rising edge 이벤트( 이벤트 이름 : Input1Rising)
                            SmInput(1, SM_INPUT_FALLING,  id, Input1Falling);                                  // Motor1의 Digital Input 1번 Falling edge 이벤트( 이벤트 이름 : Input1Falling)
                            SmParam (0x01210012, Save_Mask, SM_PARAM_EQUAL, id, SAVEARRAY);                    // Save 이벤트 : Index 0x2100 SubIndex 0x12의 데이터가 위에서 선언한 Mask값과 동일하면 이벤트 발생(이벤트 이름: SAVEARRAY)
                            SmParam (0x0121000F, ErrorReset_Mask, SM_PARAM_EQUAL, id,Error_Reset);             // Error Reset 이벤트 ; Index 0x2100 SubIndex 0x0F의 데이터가 위에서 선언한 Mask값과 동일하면 이벤트 발생(이벤트 이름: Error_Reset)
                            SmPeriod (100, id, TickPeriod);                                                    // TickPeriod라는 이름을 가진 타이머 이벤트


                            return(SmTrans(->Standing)); // 위의 이벤트 선언을 마무리 한다음에는 Standing이라는 StateMachine으로 이동
                            }
                            SIG_ERROR = {
                                          res = ErrorAxis();

                                          if (res == 0)
                                          {
                                             MOTOR1_ZUB_ERROR = ErrorNo();               // ErrorNo()라는 함수 이용하여 Zub Controller에서 발생하는 Error 를 체크하여 Motor1_ZUB_ERROR라는 Array로 옮겨 옴.
                                             MOTOR1_EPOS4_ERROR = Sysvar[0x014C0005];    // Motor1과 연결되어 있는 EPOS4에서 에러가 발생하면 Index 4C00 SubIndex 05에 저장이 되는데 이를 MOTOR1_EPOS4_ERROR라는 Array로 옮겨 옴.
                                                 if(ErrorNo() == F_POSERR)
                                                 {
                                                    print ("track error limit reached");
                                                    return(SmTrans(->Standing));
                                                 }
                                                 if(ErrorNo() == F_AMP)
                                                 {
                                                    print ("Axis Error");
                                                    return(SmTrans(->Standing));
                                                 }
                                                 if(ErrorNo() == F_ECAT_MASTER )
                                                 {
											        if(data[SMDATA_SAVEARRAY]==1)
												    {
												        print ("EtherCAT Error Save Array");	//$B
												        ErrorClear();
												        data[SMDATA_SAVEARRAY]  = 0;
                                          		    }
                                         		     else
												    {
													print ("EtherCAT Error");
													ErrorClear();
													return(SmTrans(->Standing));
												    }
                                                 }
                                          }
                                         }
                   SmState Standing

                             {
                             SIG_ENTRY = { // StateMachine 진입 스텝에 있는 StateMachine
                                            print ("Go Ready ", id);
                                            Sysvar[0x01210012] = 0;

                                         }
                                         SAVEARRAY = Save_Data; // SAVEARRAY라는 이벤트가 발생을 하면 Save_Data라는 함수로 이동
                             Error_Reset = Error__RESET;        // Error_Reset이라는 이벤트가 발생을 하면 Error__RESET 함수로 이동
                             Input1Rising = DISCHARGE_START;    // Input1Rising이라는 이벤트가 발생을 하면 DISCHARGE_START라는 함수로 이동.

                             TickPeriod = {   // 위에서 선언한 타이머 100ms Thread 개념과 동일. 바로 위에서 선언해 놓은 3줄의 이벤트가 발생하기 전까지 이 타이머대로 Loop가 발생.
                                              GearRatio = 44.0;
                                              MOTOR1_RPM = GearRatio * (MOTOR1_SPEED / (C_MOTOR_MAX_RPM / 100.0)); // 1 대신에 MOTOR1_SPEED
                                              NumberOfTurn = MOTOR1_SUCKBACK_DEGREE * (C_ENC_QC_1 / 360.0); // 100 대신에 MOTOR1_SUCKBACK_DEGREE
                                              NumberOfTurn_QUANTITY = MOTOR1_QUANTITY_DEGREE * (C_ENC_QC_1 / 360.0);// 300 대신에 MOTOR1_QUANTITY_DEGREE
                                              MOTOR1_Actual_SPEED = Sysvar[0x014C0004];
                                              if ( MOTOR1_SUCKBACK_DEGREE > MOTOR1_SUCKBACK_LIMIT)// SuckBack Degree 입력 시 SuckBack degree의 Limit수치 관계
                                              {
                                                 MOTOR1_SUCKBACK_DEGREE = MOTOR1_SUCKBACK_LIMIT;
                                              }
                                              DI1 = DigInput(1); // DI1이라는 변수의 상태는 DigInput(1)이라는 함수를 통해 취득 가능
                                          }
                              }

                   SmState DISCHARGE_PROGRESS_CONTINUOUS { //Continuous Mode 시 처음 동작 시퀀스를 나타내는 State Machine

                              SIG_ENTRY = {  print ("DISCHARGE Continuous PROGRESS");}

                              Input1Falling = DISCHARGE_DONE; // Digital Input1번의 입력이 OFF되는 이벤트가 발생을 하면 DISCHARGE_DONE이라는 함수로 이동
                                TickPeriod = { // TickPeriod라는 루프에서 DI1번 값과 모터의 실제 속도값을 계속적으로 모니터링
                                              DI1 = DigInput(1);
                                              MOTOR1_Actual_SPEED = Sysvar[0x014C0004];
                                              }
                                              }


                   SmState DISCHARGE_PROGRESS_QUANTITY {   //Quantity Mode 시 처음 동작 시퀀스를 나타내는 State Machine


                              SIG_ENTRY = {  print ("DISCHARGE Quantity PROGRESS");}

                              SIG_TARGET_REACHED = DISCHARGE_DONE; // 위치결정완료 이벤트가 발생을 하면 DISCHARGE_DONE이라는 함수로 이동

                               TickPeriod = {DI1 = DigInput(1);
                                             MOTOR1_Actual_SPEED = Sysvar[0x014C0004];
                                             }

                                                       }

                   SmState PAUSE_TIME {

                               SIG_ENTRY = {  print ("PAUSE TIME");

                                            }

                               TickPeriod = {DI1 = DigInput(1);
                                            MOTOR1_Actual_SPEED = Sysvar[0x014C0004];
                                            }
                               TickSingle_DisCharge_Delay = START_SUCKBACK; //TickSingle_DisCharge_Delay라는 이벤트가 발생을 하면 START_SUCKBACK이라는 함수로 이동

                                       }

                   SmState SUCKBACK_PROGRESS {
                                SIG_ENTRY = {  print ("SUCKBACK Done"); }

                                TickPeriod = {DI1 = DigInput(1);
                                             MOTOR1_Actual_SPEED = Sysvar[0x014C0004];
                                             }
                                SIG_TARGET_REACHED = SUCKBACK_DONE; //SuckBack 동작인 위치제어가 완료되면 SUCKBACK DONE이라는 함수로 이동
                                             }



                                    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////Motor2 State Machine//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SmState Motor2 {
               SIG_INIT = {
                            SmSubscribe(id, SIG_ERROR);


                            SmParam (0x0125017B, SM_STAT_POSREACHED, SM_PARAM_RISING, id, SIG_TARGET_REACHED_2);
                            SmInput(2, SM_INPUT_RISING,  id, Input2Rising);
                            SmInput(2, SM_INPUT_FALLING,  id, Input2Falling);
                            SmPeriod (100, id, TickPeriod_1);


                            return(SmTrans(->Standing_MOTOR2));
                            }


                             SIG_ERROR = {
                                          res = ErrorAxis();

                                          if (res == 1)
                                          {
                                             MOTOR2_ZUB_ERROR = ErrorNo();
                                             MOTOR2_EPOS4_ERROR = Sysvar[0x014C0105];
                                                 if(ErrorNo() == F_POSERR)
                                                 {
                                                    print ("track error limit reached");
                                                    return(SmTrans(->Standing_MOTOR2));
                                                 }
                                                 if(ErrorNo() == F_AMP)
                                                 {
                                                    print ("Axis Error");
                                                    return(SmTrans(->Standing_MOTOR2));
                                                 }

                                          }
                                         }




                   SmState Standing_MOTOR2

                             {
                             SIG_ENTRY = {
                                            print ("Go Ready ", id);
                                            Delay(500);
                                         }

                             Input2Rising = DISCHARGE_START_MOTOR2;
                             TickPeriod_1 = {
                                              GearRatio = 44.0;
                                              MOTOR2_RPM = GearRatio * (MOTOR2_SPEED / (C_MOTOR_MAX_RPM / 100.0)); // 1 대신에 MOTOR1_SPEED
                                              NumberOfTurn_MOTOR2 = MOTOR2_SUCKBACK_DEGREE * (C_ENC_QC_2 / 360.0); // 100 대신에 MOTOR1_SUCKBACK_DEGREE
                                              NumberOfTurn_QUANTITY_MOTOR2 = MOTOR2_QUANTITY_DEGREE * (C_ENC_QC_2 / 360.0);// 300 대신에 MOTOR1_QUANTITY_DEGREE
                                              MOTOR2_Actual_SPEED = Sysvar[0x014C0104];
                                              if ( MOTOR2_SUCKBACK_DEGREE > MOTOR2_SUCKBACK_LIMIT)
                                              {
                                                 MOTOR2_SUCKBACK_DEGREE = MOTOR2_SUCKBACK_LIMIT;
                                              }
                                              DI2 = DigInput(2);
                                          }
                              }

                   SmState DISCHARGE_PROGRESS_CONTINUOUS_MOTOR2 {

                              SIG_ENTRY = {  print ("DISCHARGE PROGRESS");}

                              Input2Falling = DISCHARGE_DONE_MOTOR2;
                                TickPeriod_1 = {DI2 = DigInput(2);
                                MOTOR2_Actual_SPEED = Sysvar[0x014C0104];
                                }
                                              }


                   SmState DISCHARGE_PROGRESS_QUANTITY_MOTOR2 {

                              SIG_ENTRY = {  print ("DISCHARGE PROGRESS");}

                              SIG_TARGET_REACHED_2 = DISCHARGE_DONE_MOTOR2;

                               TickPeriod_1 = {DI2 = DigInput(2);
                               MOTOR2_Actual_SPEED = Sysvar[0x014C0104];
                                }

                                                       }

                   SmState PAUSE_TIME_MOTOR2 {

                               SIG_ENTRY = {  print ("PAUSE TIME");

                                            }

                               TickPeriod_1 = {DI2 = DigInput(2);
                                               MOTOR2_Actual_SPEED = Sysvar[0x014C0104];
                                                }
                               TickSingle_DisCharge_Delay_1 = START_SUCKBACK_MOTOR2;

                                       }

                   SmState SUCKBACK_PROGRESS_MOTOR2 {
                                SIG_ENTRY = {  print ("SUCKBACK Done"); }

                                TickPeriod_1 = {DI1 = DigInput(1);
                                               MOTOR2_Actual_SPEED = Sysvar[0x014C0104];
                                                }
                                SIG_TARGET_REACHED_2 = SUCKBACK_DONE_MOTOR2;
                                             }



                                    }




SmMachine Operation{1, *, Motor1, 20, 2}        // 최상위에 Motor1이라는 이름을 가진 top StateMachine을 가지고 있는 SmMachine의 이름은 Operation
SmMachine Operation_MOTOR2{2, *, Motor2, 20, 2} // 최상위에 Motor2라는 이름을 가진 Top StateMachine을 가지고 있는 SmMachine의 이름은 Operation_Motor2




long DISCHARGE_START (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x01210005] == 0)) // Continuous Mode 선택 시
  {
     if(MOTOR1_RPM == 0)
     {
        print ("Motor1 RPM 0");
        return(SmTrans(Standing));
     }
     if(MOTOR1_RPM != 0)
     {
        print (" START DISCHARGE CONTINUOUS MODE ");
        Cvel (C_AXIS1, MOTOR1_RPM); // Motor1번의 속도제어
        Acc(C_AXIS1, VEL_ACC);      // Motor1번의 속도제어시 사용되는 가속도
        Dec(C_AXIS1, VEL_DEC);      // Motor1번의 속도제어시 사용되는 감속도

        AxisControl(C_AXIS1,ON);    // Motor1번의 Enable
        AxisCvelStart(C_AXIS1);     // Motor1번의 속도제어 시작
        MOTOR1_DONE = 0;            // Motor1번의 Ack 신호 0으로 만듦.
        return(SmTrans(DISCHARGE_PROGRESS_CONTINUOUS)); // 기동시킨 후 DISCHARGE_PROGRESS_CONTINUOUS StateMachine으로 이동
     }
  }

  if ((Sysvar[0x01210005] == 1)) // Quantity Mode 선택 시
  {
      if(MOTOR1_RPM == 0)
      {
         print ("Motor1 RPM 0");
         return(SmTrans(Standing));
      }
      if(MOTOR1_RPM != 0)
      {
        print (" START DISCHARGE QUANTITY MODE ");

        Vel(C_AXIS1, MOTOR1_RPM); // Motor1번의 위치제어시 사용되는 속도
        Acc(C_AXIS1, VEL_ACC);    // Motor1번의 위치제어시 사용되는 가속도
        Dec(C_AXIS1, VEL_DEC);    // Motor1번의 위치제어시 사용되는 감속도

        AxisControl(C_AXIS1,ON); // Motor1번의 Enable
        AxisPosAbsStart(C_AXIS1, Apos(C_AXIS1) + NumberOfTurn_QUANTITY); // Absolute 이동 : 현재 위치값 + 설정해 놓은 Quantity Degree값 만큼 이동.
        MOTOR1_DONE = 0;        // Motor1번의 Ack 신호 0으로 만듦
        return(SmTrans(DISCHARGE_PROGRESS_QUANTITY)); // 기동시킨 후 DISCHARGE_PROGRESS_QUANTITY StateMachine으로 이동
      }

  }


}

long DISCHARGE_DONE(long id, long signal, long event[], long data[])
{


  if (( Sysvar[0x01210005] == 0))
  {
  print (" DISCHARGE CONTINUOUS MODE DONE");

  AxisCvelStop(C_AXIS1); // 속도제어 동작 정지
  SmTimer (MOTOR1_PAUSETIME,id, TickSingle_DisCharge_Delay); // MOTOR1_PAUSETIME
  return(SmTrans(PAUSE_TIME)); //PAUSE_TIME이라는 StateMachine으로 이동

  }

  if (( Sysvar[0x01210005] == 1))
  {
  print (" DISCHARGE QUANTITY MODE DONE");
  SmTimer (MOTOR1_PAUSETIME,id, TickSingle_DisCharge_Delay); // MOTOR1_PAUSETIME
  return(SmTrans(PAUSE_TIME));//PAUSE_TIME이라는 StateMachine으로 이동

  }


}


long START_SUCKBACK (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x01210005] == 0))
  {
  print (" START SUCKBACK Continuous Mode");
  Vel (C_AXIS1,MOTOR1_RPM);
  Acc(C_AXIS1, VEL_ACC);
  Dec(C_AXIS1, VEL_DEC);
  AxisPosAbsStart(C_AXIS1, Apos(C_AXIS1) - NumberOfTurn);
  return(SmTrans(SUCKBACK_PROGRESS)); // SUCKBACK_PROGRESS라는 StateMachine으로 이동
  }

  if (( Sysvar[0x01210005] == 1))
  {
  print (" START SUCKBACK QUANTITY Mode");
  Vel (C_AXIS1,MOTOR1_RPM);
  Acc(C_AXIS1, VEL_ACC);
  Dec(C_AXIS1, VEL_DEC);
  AxisPosAbsStart(C_AXIS1, Apos(C_AXIS1) - NumberOfTurn);
  return(SmTrans(SUCKBACK_PROGRESS));// SUCKBACK_PROGRESS라는 StateMachine으로 이동
  }



}


long SUCKBACK_DONE (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x01210005] == 0))
  {
  print (" SUCKBACK Continuous Mode DONE ");
  MOTOR1_DONE = 1; // Motor1의 Ack 신호 1로 생성
  return(SmTrans(Standing)); // Standing이라는 StateMachine으로 이동
  }
  if (( Sysvar[0x01210005] == 1))
  {
  print (" SUCKBACK Quantity Mode DONE ");
  MOTOR1_DONE = 1; // Motor1의 Ack 신호 1로 생성
  return(SmTrans(Standing));// Standing이라는 StateMachine으로 이동
  }

}

////////////////////////////////////////////////////////////////////////
////////////////////////////MOTOR 2/////////////////////////////////////
////////////////////////////////////////////////////////////////////////

long DISCHARGE_START_MOTOR2 (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x0121000A] == 0))
  {
       if(MOTOR2_RPM == 0)
       {
       print ("Motor2 RPM 0");
       return(SmTrans(Standing_MOTOR2));
       }
       if(MOTOR2_RPM != 0)
       {
       print (" START DISCHARGE CONTINUOUS MODE ");
       Cvel (C_AXIS2, MOTOR2_RPM);
       Acc(C_AXIS2, VEL_ACC);
       Dec(C_AXIS2, VEL_DEC);

       AxisControl(C_AXIS2,ON);
       AxisCvelStart(C_AXIS2);
       MOTOR2_DONE = 0;
       return(SmTrans(DISCHARGE_PROGRESS_CONTINUOUS_MOTOR2));
       }
  }

  if ((Sysvar[0x0121000A] == 1))
  {
       if(MOTOR2_RPM == 0)
       {
       print ("Motor2 RPM 0");
       return(SmTrans(Standing_MOTOR2));
       }
       if(MOTOR2_RPM != 0)
       {
       print (" START DISCHARGE QUANTITY MODE ");

       Vel(C_AXIS2, MOTOR2_RPM);
       Acc(C_AXIS2, VEL_ACC);
       Dec(C_AXIS2, VEL_DEC);

       AxisControl(C_AXIS2,ON);
       AxisPosAbsStart(C_AXIS2, Apos(C_AXIS2) + NumberOfTurn_QUANTITY_MOTOR2);
       MOTOR2_DONE = 0;
       return(SmTrans(DISCHARGE_PROGRESS_QUANTITY_MOTOR2));
       }

  }


}

long DISCHARGE_DONE_MOTOR2(long id, long signal, long event[], long data[])
{


  if (( Sysvar[0x0121000A] == 0))
  {
  print (" DISCHARGE CONTINUOUS MODE DONE");

  AxisCvelStop(C_AXIS2);
  SmTimer (MOTOR2_PAUSETIME,id, TickSingle_DisCharge_Delay_1); // 1000 대신에 MOTOR1_PAUSETIME
  return(SmTrans(PAUSE_TIME_MOTOR2));

  }

  if (( Sysvar[0x0121000A] == 1))
  {
  print (" DISCHARGE QUANTITY MODE DONE");
  SmTimer (MOTOR2_PAUSETIME,id, TickSingle_DisCharge_Delay_1); // 1000 대신에 MOTOR1_PAUSETIME
  return(SmTrans(PAUSE_TIME_MOTOR2));

  }


}


long START_SUCKBACK_MOTOR2 (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x0121000A] == 0))
  {
  print (" START SUCKBACK Continuous Mode");
  Vel (C_AXIS2,MOTOR2_RPM);
  Acc(C_AXIS2, VEL_ACC);
  Dec(C_AXIS2, VEL_DEC);
  AxisPosAbsStart(C_AXIS2, Apos(C_AXIS2) - NumberOfTurn_MOTOR2);
  return(SmTrans(SUCKBACK_PROGRESS_MOTOR2));
  }

  if (( Sysvar[0x0121000A] == 1))
  {
  print (" START SUCKBACK QUANTITY Mode");
  Vel (C_AXIS2,MOTOR2_RPM);
  Acc(C_AXIS2, VEL_ACC);
  Dec(C_AXIS2, VEL_DEC);
  AxisPosAbsStart(C_AXIS2, Apos(C_AXIS2) - NumberOfTurn_MOTOR2);
  return(SmTrans(SUCKBACK_PROGRESS_MOTOR2));
  }



}


long SUCKBACK_DONE_MOTOR2 (long id, long signal, long event[], long data[])
{
  if (( Sysvar[0x0121000A] == 0))
  {
  print (" SUCKBACK Continuous Mode DONE ");
  MOTOR2_DONE = 1;
  return(SmTrans(Standing_MOTOR2));
  }
  if (( Sysvar[0x0121000A] == 1))
  {
  print (" SUCKBACK Quantity Mode DONE ");
  MOTOR2_DONE = 1;
  return(SmTrans(Standing_MOTOR2));
  }

}

long Error__RESET (long id, long signal, long event[], long data[])
{

    ErrorClear();
    AmpErrorClear(C_AXIS1);
    AmpErrorClear(C_AXIS2);
    Sysvar[0x01210012] = 0;
    return(SmTrans(Standing));

}

long Save_Data (long id, long signal, long event[], long data[])
{
print (" SAVE DATA TO ROM");
data[SMDATA_SAVEARRAY]  = 1;
Delay(1);
Save(ARRAYS);
return(SmTrans(Standing));
}







long main(void)
{




	ErrorClear();

	ECatMasterCommand(0x1000, 0);

	EtherCAT_Slave_Count = sdkEtherCATMasterInitialize();

    for ( i = 1000001; i <= 1000001+EtherCAT_Slave_Count-1; i++) {
    sdkEpos4_SetupECatSdoParam(i, C_PDO_NUMBER, C_AXISPOLARITY, C_OP_MODE_CSP );

    }

	for (i = 1; i <= EtherCAT_Slave_Count; i++) {
	   sdkEtherCATSetupDC(i, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]
        }

	sdkEtherCATMasterStart();

    for (i = 0; i <= EtherCAT_Slave_Count-1 ; i++)
    {
    sdkEpos4_SetupECatBusModule(i, i+1000001, C_PDO_NUMBER, C_OP_MODE_CSP);
    sdkEpos4_SetupECatVirtAmp(i, C_MOTOR_MAX_RPM, C_OP_MODE_CSP);
    sdkEpos4_SetupECatVirtCntin(i, C_OP_MODE_CSP);

    AXE_PARAM(i, POSENCREV) = 1;		// Number of revolutions of the motor
    AXE_PARAM(i, POSENCQC)  = 512*4;     // Number of quadcounts in POSENCREV revolutions
    AXE_PARAM(i, POSFACT_Z) = 4356;		// Number of revolutions of the input shaft
    AXE_PARAM(i, POSFACT_N) = 100;		// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
    AXE_PARAM(i, FEEDREV)   = 1;		// Number of revolutions of the gear box output shaft
    AXE_PARAM(i, FEEDDIST)  = 512*4;	// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft


    AXE_PARAM(i, VELMAX) = C_MOTOR_MAX_RPM;
    AXE_PARAM(i, RAMPMIN) = 100;	// Maximum acceleration

    AXE_PARAM(i, VELMAX) = 4000;

    AXE_PARAM(i, KPROP) = 0;
    AXE_PARAM(i, KDER) = 0;
    AXE_PARAM(i, POSERR) = 200000;
    AXE_PARAM(i, RAMPTYPE) = 0;
    AXE_PARAM(i, JERKMIN) = 100;
    AXE_PARAM(i, VELRES) = 100;
    }





	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------

	ErrorClear();
    AmpErrorClear(C_AXIS1);
    AmpErrorClear(C_AXIS2);


    res = SmRun(Operation,Operation_MOTOR2);

}
//$X {Element (1),1,1,0,-1,0,-1,0,(-1),-1},0x2100,5,0
//$X {Element (2),1,1,0,-1,0,-1,0,(-1),-1},0x2100,6,0
//$X {Element (3),1,1,0,-1,0,-1,0,(-1),-1},0x2100,7,0
//$X {Element (4),1,1,0,-1,0,-1,0,(-1),-1},0x2100,8,0
//$X {Element (5),1,1,0,-1,0,-1,0,(-1),-1},0x2100,9,0
//$X {Element (6),1,1,0,-1,0,-1,0,(-1),-1},0x2100,10,0
//$X {Element (7),1,1,0,-1,0,-1,0,(-1),-1},0x2100,11,0
//$X {Element (8),1,1,0,-1,0,-1,0,(-1),-1},0x2100,12,0
//$X {Element (9),1,1,0,-1,0,-1,0,(-1),-1},0x2100,13,0
//$X {Element (10),1,1,0,-1,0,-1,0,(-1),-1},0x2100,14,0
//$X {Element (12),1,1,0,-1,0,-1,0,(-1),-1},0x2100,16,0
//$X {Element (13),1,1,0,-1,0,-1,0,(-1),-1},0x2100,17,0
//$X {Element (14),1,1,0,-1,0,-1,0,(-1),-1},0x2100,18,0
//$X {Element (11),1,1,0,-1,0,-1,0,(-1),-1},0x2100,15,0
//$X {MOTOR_Write_Array,1,1,0,14,1,0,1,(18,100),1},0x2100,23,0
