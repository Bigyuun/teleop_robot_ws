
#include <SysDef.mh> // 기본적으로 참조해야 하는 참조파일(기본 문법 / 기본 파라미터 등이 정의가 되어 있음)
#include "ApossC_SDK_V01.09\SDK\SDK_ApossC.mc" // 쉽게 프로그램 할 수 있도록 기능 별로 SDK를 만들어 놨음

dim long Monitoring_MAXON_TO_PC[12];
long Direction;
double AnalogValue;
double dSpeed;
            long velres, maxrpm, rampmin;
long errAxis,errNo,USERVEL,ActVel,JOG_Mode;
//*** 명령어 정의
#define COMMAND_NONE            0
#define COMMAND_MOVE_VEL        1  //2번축 테스트 완료
#define COMMAND_MOVE_REL        2  //2번축 테스트 완료
#define COMMAND_MOVE_ABS        3  //2번축 테스트 완료
#define COMMAND_MOVE_HOME       4  //2번축 테스트 완료
#define COMMAND_MOVE_ANALOG     5  //
#define COMMAND_STOP            6   //2번축 테스트 완료
#define COMMAND_SET_SOFTLIMIT   7   //2번축 테스트 완료
#define COMMAND_SET_SERVO_ON    8   //2번축 테스트 완료
#define COMMAND_SET_SERVO_OFF   9   //2번축 테스트 완료
#define COMMAND_SET_POS         10  // vlfdydjqtdj qhdla.
#define ignored_value 5

// 모터 EC-i40 496661 CPT 1024
#define AXIS0 0 //모터 출력 포트 : 오른쪽에서부터 0,1,2,3
#define AXIS0_ENCPORT 0 // 엔코더 입력 포트 : 오른쪽에서부터 0,1,2,3
#define AXIS0_ENCRES 1024*4 //엔코더 분해능
#define AXIS0_ENC_LATCHTYPE 0 // 신경안써도 됨
#define AXIS0_ENC_LATCHPARAM 0//신경안써도 됨.
#define AXIS0_ENC_LATCHSLOPE HWLATCH_SLOPE_RISING //신경안써도 됨.

#define AXIS0_CONTROLMODE HWAMP_MODE_POS_VEL_CUR // 위치 / 속도 / 토크 제어기
#define AXIS0_POLEPAIRS 7 // 모터 자극 쌍의 수
#define AXIS0_MAXCUR	2720*1.25 // 모터 최대 전류  (단위 : mA)
#define AXIS0_CONCUR	2720 // 모터의 정격 전류      (단위 : mA)
#define AXIS0_THERMAL_TIME 20700 // Thermaltime constant winding 정격 전류의 2.5배에 달하는
                                 // 부하가 걸렸을 때 모터가 구동 가능한 시간 (단위 : ms)
#define AXIS0_MAX_RPM 8000 // 카탈로그의 23번 항목의 max.speed를 넘지 않는 것을 권장 드림.

// Tools - Current regulator Calculaotr 에서 인가전압 / 인덕턴스와 저항을 입력하여 계산해서 나온 결과물 기입
#define AXIS0_CURKPROP	5898 // 전류 제어기의 P 게인 - 위 메뉴에서 계산 가능.
#define AXIS0_CURKINT 402    // 전류 제어기의 I 게인 - 위 메뉴에서 계산 가능.
#define AXIS0_CURKILIM	32767 // 고정값
#define AXIS0_VELKPROP		1700 // 속도제어기의 P 게인
#define AXIS0_VELKINT		100  // 속도제어기의 I 게인
#define AXIS0_VELKILIM		1000 // 고정값

#define AXIS0_BRUSHLESS 1     // 브러쉬리스 모터를 사용할 경우에 필요한 파라미터(브러쉬가 있는 DC모터를 사용할 경우에는 필요없음)
#define AXIS0_ALIGN_CUR 1000  // Align 잡을 때 사용하는 전류 레벨

//Movemnet Param
#define AXIS0_VELRES 100  // 속도 분해능
#define AXIS0_RAMPTYPE 0  // 모션 프로파일 타입 : 0 : 사다리꼴 1 : S-curve 2: Jerk
#define AXIS0_RAMPMIN 1000 // 가속도 : 최대 가속도 (설정해놓은 최대 속도까지 올라가는 데 걸리는 시간) 단위는 ms
#define AXIS0_JERKMIN 1000 // Jerk 시 사용하는 최대 가속도

#define AXIS0_DIRECTION -1 // 모터의 구동 방향 결정

#define AXIS0_KPROP 80 // 위치제어기의 P 게인
#define AXIS0_KINT 0   // 위치제어기의 I 게인
#define AXIS0_KDER 600 // 위치제어기의 D 게인
#define AXIS0_KILIM 3000 // 고정값
#define AXIS0_KILIMTIME 0 // 고정값
#define AXIS0_BANDWIDTH 1000 // 고정값
#define AXIS0_FFVEL 1000     // 고정값
#define AXIS0_KFFAC 0        // 고정값
#define AXIS0_KFFDEC 0       // 고정값

#define AXIS0_POSENCREV	1 // 고정값
#define AXIS0_POSENCQC AXIS1_ENCRES // 사용되는 엔코더 분해능
#define AXIS0_POSFACT_Z 1 // 사용되는 기어의 기어비를 넣는 부분(전자기어비) ex. 100:1 -> 100
#define AXIS0_POSFACT_N 1 // 사용되는 기어의 기어비를 넣는 부분(전자기어비) ex. 100:1 -> 1
#define AXIS0_FEEDREV 1 // 고정값
#define AXIS0_FEEDDIST AXIS1_ENCRES // 사용되는 엔코더 분해능

// 모터 EC-i40 496661 CPT 1024
#define AXIS1 1
#define AXIS1_ENCPORT 1
#define AXIS1_ENCRES 1024*4
#define AXIS1_ENC_LATCHTYPE 0
#define AXIS1_ENC_LATCHPARAM 0
#define AXIS1_ENC_LATCHSLOPE HWLATCH_SLOPE_RISING

#define AXIS1_CONTROLMODE HWAMP_MODE_POS_VEL_CUR
#define AXIS1_POLEPAIRS 7
#define AXIS1_MAXCUR	2720*1.25
#define AXIS1_CONCUR	2720
#define AXIS1_THERMAL_TIME 20700
#define AXIS1_MAX_RPM 8000

// Tools - Current regulator Calculaotr 에서 인가전압 / 인덕턴스와 저항을 입력하여 계산해서 나온 결과물 기입
#define AXIS1_CURKPROP	5898
#define AXIS1_CURKINT 402
#define AXIS1_CURKILIM	32767
#define AXIS1_VELKPROP		1700					// Proporti onal factor of velocity controller
#define AXIS1_VELKINT		100					// Integral factor of velocity controller
#define AXIS1_VELKILIM		1000

#define AXIS1_BRUSHLESS 1
#define AXIS1_ALIGN_CUR 1000

//Movemnet Param
#define AXIS1_VELRES 100
#define AXIS1_RAMPTYPE 0
#define AXIS1_RAMPMIN 1000 //단위는 ms
#define AXIS1_JERKMIN 1000

#define AXIS1_DIRECTION -1 // 모터의 구동 방향 결정

#define AXIS1_KPROP 80
#define AXIS1_KINT 0
#define AXIS1_KDER 600
#define AXIS1_KILIM 3000
#define AXIS1_KILIMTIME 0
#define AXIS1_BANDWIDTH 1000
#define AXIS1_FFVEL 1000
#define AXIS1_KFFAC 0
#define AXIS1_KFFDEC 0

#define AXIS1_POSENCREV	1
#define AXIS1_POSENCQC AXIS2_ENCRES
#define AXIS1_POSFACT_Z 1
#define AXIS1_POSFACT_N 1
#define AXIS1_FEEDREV 1
#define AXIS1_FEEDDIST AXIS2_ENCRES


// 모터 EC-i40 496661 CPT 1024
#define AXIS2 2
#define AXIS2_ENCPORT 2
#define AXIS2_ENCRES 1024*4
#define AXIS2_ENC_LATCHTYPE 0
#define AXIS2_ENC_LATCHPARAM 0
#define AXIS2_ENC_LATCHSLOPE HWLATCH_SLOPE_RISING

#define AXIS2_CONTROLMODE HWAMP_MODE_POS_VEL_CUR
#define AXIS2_POLEPAIRS 7
#define AXIS2_MAXCUR	2720*1.25
#define AXIS2_CONCUR	2720
#define AXIS2_THERMAL_TIME 20700
#define AXIS2_MAX_RPM 8000

// Tools - Current regulator Calculaotr 에서 인가전압 / 인덕턴스와 저항을 입력하여 계산해서 나온 결과물 기입
#define AXIS2_CURKPROP	5898
#define AXIS2_CURKINT 402
#define AXIS2_CURKILIM	32767
#define AXIS2_VELKPROP		1700					// Proporti onal factor of velocity controller
#define AXIS2_VELKINT		100					// Integral factor of velocity controller
#define AXIS2_VELKILIM		1000

#define AXIS2_BRUSHLESS 1
#define AXIS2_ALIGN_CUR 1000

//Movemnet Param
#define AXIS2_VELRES 10000
#define AXIS2_RAMPTYPE 0
#define AXIS2_RAMPMIN 1000 //단위는 ms
#define AXIS2_JERKMIN 1000

#define AXIS2_DIRECTION -1 // 모터의 구동 방향 결정

#define AXIS2_KPROP 80
#define AXIS2_KINT 0
#define AXIS2_KDER 600
#define AXIS2_KILIM 3000
#define AXIS2_KILIMTIME 0
#define AXIS2_BANDWIDTH 1000
#define AXIS2_FFVEL 1000
#define AXIS2_KFFAC 0
#define AXIS2_KFFDEC 0

#define AXIS2_POSENCREV	1
#define AXIS2_POSENCQC AXIS3_ENCRES
#define AXIS2_POSFACT_Z 1
#define AXIS2_POSFACT_N 1
#define AXIS2_FEEDREV 1
#define AXIS2_FEEDDIST AXIS3_ENCRES





/// FEED b7b93a0b56c6_1 ECXTQ22M 24V 3.9:1 CPT 1024

#define AXIS3                3
#define AXIS3_ENCPORT        3
#define AXIS3_ENCRES         1024*4
#define AXIS3_ENC_LATCHTYPE  0
#define AXIS3_ENC_LATCHPARAM 0
#define AXIS3_ENC_LATCHSLOPE HWLATCH_SLOPE_RISING

#define AXIS3_CONTROLMODE    HWAMP_MODE_POS_VEL_CUR
#define AXIS3_POLEPAIRS      2
#define AXIS3_MAXCUR	     2480*1.25
#define AXIS3_CONCUR	     2480  // 정격전류
#define AXIS3_THERMAL_TIME   31200 //단위 : ms : 41.5s
#define AXIS3_MAX_RPM        8000  //기어의 Max.입력속도에 의거해서 결정.

// Tools - Current regulator Calculaotr 에서 인가전압 / 인덕턴스와 저항을 입력하여 계산해서 나온 결과물 기입
#define AXIS3_CURKPROP	     4151
#define AXIS3_CURKINT        351
#define AXIS3_CURKILIM	     32767 // 구체적ㅈ인 용도
#define AXIS3_VELKPROP		 1700  // Proportional factor of velocity controller
#define AXIS3_VELKINT		 100   // Integral factor of velocity controller
#define AXIS3_VELKILIM		 1000


//Movemnet Param
#define AXIS3_VELRES         100
#define AXIS3_RAMPTYPE       0
#define AXIS3_RAMPMIN        1000 //단위는 ms
#define AXIS3_JERKMIN        1000

#define AXIS3_DIRECTION      -1 // 모터의 구동 방향 결정

#define AXIS3_BRUSHLESS      1
#define AXIS3_ALIGN_CUR      1000

#define AXIS3_KPROP          80
#define AXIS3_KINT           0
#define AXIS3_KDER           600
#define AXIS3_KILIM          3000
#define AXIS3_KILIMTIME      0
#define AXIS3_BANDWIDTH      1000
#define AXIS3_FFVEL          1000
#define AXIS3_KFFAC          0
#define AXIS3_KFFDEC         0

#define AXIS3_POSENCREV	     1
#define AXIS3_POSENCQC       AXIS3_ENCRES
#define AXIS3_POSFACT_Z      12167
#define AXIS3_POSFACT_N      64
#define AXIS3_FEEDREV        1
#define AXIS3_FEEDDIST       AXIS3_ENCRES






/*********************************************************************
**              State Machine Version of Program
*********************************************************************/

/*********************************************************************
** State Machine Setup Parameters
*********************************************************************/

#pragma SmConfig {    1,  // Runtime flags.
                      40, // Event pool size.
                      10, // Maximum number of timers.
                      5,  // Subscribe pool size.
                      12, // Param pool size.
                      0,  // Position pool size.
                      2 } // System signal pool size (used for SmSystem.)



/*********************************************************************
** Event Definitions
*********************************************************************/
SmEvent TickPeriod1 {PRM_CLOCK}
SmEvent TickPeriod2 {PRM_CLOCK}
SmEvent TickPeriod3 {PRM_CLOCK}
SmEvent TickPeriod4 {PRM_CLODK}

//*** 2번축 Event 리스트
SmEvent Event_VelocityMove_Axis2 {}
SmEvent Event_RelativeMove_Axis2 {}
SmEvent Event_AbsoluteMove_Axis2 {}
SmEvent Event_HomeMove_Axis2 {}
SmEvent Event_AnalogMove_Axis2 {}
SmEvent Event_Stop_Axis2 {}
SmEvent Event_SoftwareLimit_Axis2 {}
SmEvent Event_ServoOn_Axis2 {}
SmEvent Event_ServoOff_Axis2 {}
SmEvent Event_SetPosition_Axis2 {}
SmEvent Event_Applied_Velocity {}

SmEvent TickPeriod_AnalogControl_Axis2 {PRM_CLOCK}
SmEvent TickPeriod_StatusMonitor_Axis2 {PRM_CLOCK}

/***
 *** 각축별 명령 Command 영역을 리셋한다
 ***/
void CommandReset(long Axis)
{
	if (Axis == AXIS0)	Sysvar[0x01220110] = 0;
	if (Axis == AXIS1)	Sysvar[0x01220120] = 0;
	if (Axis == AXIS2)	Sysvar[0x01220130] = 0;
	if (Axis == AXIS3)	Sysvar[0x01220140] = 0;
}

/***
 *** 소프트웨어 리미트를 설정한다
 ***/
void SetSoftwareLimit(long Axis, long SwEnable, double Max, double Min)
{
	if (SwEnable == 0)
	{
		//*** 소프트웨어 리미트 Disable
		AXE_PARAM(Axis, SWPOSLIMACT) = 0;
		AXE_PARAM(Axis, SWNEGLIMACT) = 0;
	}
	else
	{
		//*** 소프트웨어 리미트 Enable
		AXE_PARAM(Axis, NEGLIMIT) = Min;
		AXE_PARAM(Axis, POSLIMIT) = Max;
		AXE_PARAM(Axis, SWPOSLIMACT) = 1;
		AXE_PARAM(Axis, SWNEGLIMACT) = 1;
	}
}

/***
 *** 위치값을 기록한다
 ***/
void UpdatePosition(long Axis)
{
	if (Axis == AXIS0)
	{
		Sysvar[0x01210009] = Apos(Axis);
		Sysvar[0x0121000A] = Cpos(Axis);
	}
	if (Axis == AXIS1)
	{
		Sysvar[0x0121000B] = Apos(Axis);
		Sysvar[0x0121000C] = Cpos(Axis);
	}
	if (Axis == AXIS2)
	{
		Sysvar[0x0121000D] = Apos(Axis);
		Sysvar[0x0121000E] = Cpos(Axis);
	}
	if (Axis == AXIS3)
	{
		Sysvar[0x0121000F] = Apos(Axis);
		Sysvar[0x01210010] = Cpos(Axis);
	}
}

/***
 *** User Parameter에서 속도값을 읽는다
 ***/
double GetSpeedFromUserParameter(long Axis)
{
	if (Axis == AXIS0) { return Sysvar[0x01220111]; }
	if (Axis == AXIS1) { return Sysvar[0x01220121]; }
	if (Axis == AXIS2) { return Sysvar[0x01220131]; }
	if (Axis == AXIS3) { return Sysvar[0x01220141]; }
}

/***
 *** User Parameter에서 가속도값을 읽는다
 ***/
double GetAccelFromUserParameter(long Axis)
{
	if (Axis == AXIS0) { return Sysvar[0x01220112]; }
	if (Axis == AXIS1) { return Sysvar[0x01220122]; }
	if (Axis == AXIS2) { return Sysvar[0x01220132]; }
	if (Axis == AXIS3) { return Sysvar[0x01220142]; }
}

/***
 *** User Parameter에서 감속도값을 읽는다
 ***/
double GetDecelFromUserParameter(long Axis)
{
	if (Axis == AXIS0){return Sysvar[0x01220113]; }
	if (Axis == AXIS1){return Sysvar[0x01220123]; }
	if (Axis == AXIS2){return Sysvar[0x01220133]; }
	if (Axis == AXIS3){return Sysvar[0x01220143]; }
}

/***
 *** User Parameter에서 위치값을 읽는다
 ***/
double GetPositionFromUserParameter(long Axis)
{
	if (Axis == AXIS0) { return Sysvar[0x01220114]; }
	if (Axis == AXIS1) { return Sysvar[0x01220124]; }
	if (Axis == AXIS2) { return Sysvar[0x01220134]; }
	if (Axis == AXIS3) { return Sysvar[0x01220144]; }
}

/***
 *** User Parameter에서 Software Limit 상한값을 읽는다
 ***/
double GetSoftLimitCwFromUserParameter(long Axis)
{
	if (Axis == AXIS0) { return Sysvar[0x01220115]; }
	if (Axis == AXIS1) { return Sysvar[0x01220125]; }
	if (Axis == AXIS2) { return Sysvar[0x01220135]; }
	if (Axis == AXIS3) { return Sysvar[0x01220145]; }
}

/***
 *** User Parameter에서 Software Limit 하한값을 읽는다
 ***/
double GetSoftLimitCcwFromUserParameter(long Axis)
{
	if (Axis == AXIS0) { return Sysvar[0x01220116]; }
	if (Axis == AXIS1) { return Sysvar[0x01220126]; }
	if (Axis == AXIS2) { return Sysvar[0x01220136]; }
	if (Axis == AXIS3) { return Sysvar[0x01220146]; }
}

/*********************************************************************
** State Definitions
*********************************************************************/
SmState Axis0_State
{
}

SmState Axis1_State
{

}

SmState Axis2_State
{
	SIG_INIT =
	{
	    SmSubscribe(id, SIG_ERROR);
        SmPeriod (80, id, TickPeriod_StatusMonitor_Axis2);
        SmPeriod (1, id, TickPeriod_AnalogControl_Axis2);

        SmParam (0x01220130, COMMAND_MOVE_VEL, SM_PARAM_EQUAL, id, Event_VelocityMove_Axis2);  //TEST완결
        SmParam (0x01220130, COMMAND_MOVE_REL, SM_PARAM_EQUAL, id, Event_RelativeMove_Axis2);  //TEST완결
        SmParam (0x01220130, COMMAND_MOVE_ABS, SM_PARAM_EQUAL, id, Event_AbsoluteMove_Axis2);  //TEST완결
        SmParam (0x01220130, COMMAND_MOVE_HOME, SM_PARAM_EQUAL, id, Event_HomeMove_Axis2);
        SmParam (0x01220130, COMMAND_MOVE_ANALOG, SM_PARAM_EQUAL, id, Event_AnalogMove_Axis2);
        SmParam (0x01220130, COMMAND_STOP, SM_PARAM_EQUAL, id, Event_Stop_Axis2);              //TEST완결
        SmParam (0x01220130, COMMAND_SET_SOFTLIMIT, SM_PARAM_EQUAL, id, Event_SoftwareLimit_Axis2);
        SmParam (0x01220130, COMMAND_SET_SERVO_ON, SM_PARAM_EQUAL, id, Event_ServoOn_Axis2);   //TEST완결
        SmParam (0x01220130, COMMAND_SET_SERVO_OFF, SM_PARAM_EQUAL, id, Event_ServoOff_Axis2); //TEST완결
        SmParam (0x01220130, COMMAND_SET_POS, SM_PARAM_EQUAL, id, Event_SetPosition_Axis2);

		return(SmTrans(->Standing_AXIS2));


	}
    SIG_ERROR = {
		           errAxis = ErrorAxis();
		           errNo = ErrorNo();
		           if(ErrorNo() != 0)
		           {
		           ErrorClear();
		        //  errAxis = 0;
		        //   errNo = 0;
		           print("ErrorClear");
		           }
		           return(SmTrans(->Standing_AXIS2));
		            }
	//*** 명령수행 이벤트
	SmState Standing_AXIS2
	{
		Event_VelocityMove_Axis2 =         // TEST 완료
		{
			double Speed, Accel;

			//*** 실행명령 User Parameter 리셋
			CommandReset(AXIS0);

			Speed = GetSpeedFromUserParameter(AXIS0); // 속도값을 읽어온다
			Accel = GetAccelFromUserParameter(AXIS0); // 가속도값을 읽어온다

			Cvel(AXIS0, Speed);     // Set the velocity
			Acc(AXIS0, Accel);      // Set the acceleration

			AxisCvelStart(AXIS0);   // Start constant velocity mode
		}

		Event_RelativeMove_Axis2 =            // TEST 완료
		{
			double Speed, Accel, Decel, Pos;

			//*** 실행명령 User Parameter 리셋
			CommandReset(AXIS0);

			Speed = GetSpeedFromUserParameter(AXIS0);  // 속도값을 읽어온다
			Accel = GetAccelFromUserParameter(AXIS0);  // 가속도값을 읽어온다
			Decel = GetDecelFromUserParameter(AXIS0);  // 감속도값을 읽어온다
			Pos = GetPositionFromUserParameter(AXIS0); // 위치값을 읽어온다

			Vel(AXIS0, Speed);
			Acc(AXIS0, Accel);
			Dec(AXIS0, Decel);
			AxisPosRelStart(AXIS0, Pos);
		}

		Event_AbsoluteMove_Axis2 =    // TEST 완료
		{
			double Speed, Accel, Decel, Pos;

			//*** 실행명령 User Parameter 리셋
			CommandReset(AXIS0);

			Speed = GetSpeedFromUserParameter(AXIS0);  // 속도값을 읽어온다
			Accel = GetAccelFromUserParameter(AXIS0);  // 가속도값을 읽어온다
			Decel = GetDecelFromUserParameter(AXIS0);  // 감속도값을 읽어온다
			Pos = GetPositionFromUserParameter(AXIS0); // 위치값을 읽어온다

			Vel(AXIS0, Speed);
			Acc(AXIS0, Accel);
			Dec(AXIS0, Decel);
			AxisPosAbsStart(AXIS0, Pos);
		}

		Event_HomeMove_Axis2 =    // TEST 완료
		{
			//*** 실행명령 User Parameter 리셋
			CommandReset(AXIS0);

            velres = AXIS0_VELRES;
			maxrpm = AXIS0_MAX_RPM;
            dSpeed = 60.0 * (velres / maxrpm);


		    Cvel(AXIS0,10);
		    Acc(AXIS0,100);
		    Dec(AXIS0,100);
		    AxisCvelStart(AXIS0);
            //AxisHomeStart(AXIS2);
            print("Home Start");
		}

		Event_AnalogMove_Axis2 =
		{
			//*** 실행명령 User Parameter 리셋
			CommandReset(AXIS0);
            JOG_Mode = 1; // Analog Jog Mode
			Cvel(AXIS0, USERVEL);     // Set the velocity
			Acc(AXIS0, 500);      // Set the acceleration
            Dec(AXIS0, 500);
			AxisCvelStart(AXIS0);   // Start constant velocity mode

			//AXE_PROCESS(AXIS2, REG_USERREFVEL) = USERVEL;
		}

		Event_Stop_Axis2 =          // TEST 완료
		{
			double Decel;

			CommandReset(AXIS0);
            if(JOG_Mode == 0)
            {
			AxisStop(AXIS0);

			//Sysvar[0x0125007A]; }
			Decel = GetDecelFromUserParameter(AXIS2);  // 감속도값을 읽어온다
			Dec(AXIS0, Decel);			               // Set the deceleration
			AxisCvelStop(AXIS0);		               // Stop constant velocity mode
			}
			if(JOG_Mode == 1)
			{
			AxisCvelStop(AXIS0);
			JOG_Mode = 0;
			}

		}

		Event_SoftwareLimit_Axis2 =   // TEST 완료
		{
			double SwMax, SwMin;

			CommandReset(AXIS0);

			//*** User Parameter에서 소프트웨어 리미트 값을 읽어온다
			SwMax = GetSoftLimitCwFromUserParameter(AXIS0);
			SwMin = GetSoftLimitCcwFromUserParameter(AXIS0);

			if (SwMax == SwMin)
				SetSoftwareLimit(AXIS0, 0, 0, 0);         // 소프트웨어 리미트 Disable
			else
				SetSoftwareLimit(AXIS0, 1, SwMax, SwMin); // 소프트웨어 리미트 Enable
		}

		Event_ServoOn_Axis2 =                    // TEST 완료
		{
			CommandReset(AXIS0);

			AxisControl (AXIS0, ON); // Servo On
			print("Axis2 Enabled");
		}

		Event_ServoOff_Axis2 =                   // TEST 완료
		{
			CommandReset(AXIS0);

			AxisControl (AXIS0, OFF); // Servo Off
			print("Axis2 Disabled");
		}

		Event_SetPosition_Axis2 =                // TEST 완료 필요없어 보임.
		{
			CommandReset(AXIS0);
		}
    }

	//Command Velocity [RPM] = vel *  Maximum Velocity (VELMAX)/ Velocity Resolution (VELRES)

	//*** 타이머, Analog 입력으로 모터제어
	TickPeriod_AnalogControl_Axis2 =
	{


		//Direction = DigInput(16);                         // Get the state of input 13, 1 또는 0
		AnalogValue = AnalogInput(1);                     // 1번 아날로그 입력 전압을 읽는다

		// 1=정방향, -1=역방향
		if (Direction > 0)
			Direction = 1;
		else
			Direction = -1;

		AnalogValue = AnalogValue * Direction;            // 입력전압에 방향을 적용한다
		USERVEL = AnalogValue / 300;


		if(JOG_Mode == 1)
		{
		Cvel(AXIS2, USERVEL);
		}
	}

	//*** 상태를 업데이트 하기위한 타이머
	TickPeriod_StatusMonitor_Axis2 =
	{
		//*** Busy상태 업데이트, 1=Moving, 0=Stop
		AxisMoving(AXIS0);
		AxisMoving(AXIS1);
		AxisMoving(AXIS2);
		AxisMoving(AXIS3);

		if (AxisStatus(0) & AXSTAT_POSLIMERR)
			print("Positive software position limit reached "));
		if (AxisStatus(0) & AXSTAT_NEGLIMERR)
			print("Negative software position limit reached ");


		if(Sysvar[0x0125007B].i8 == 1)
			Sysvar[0x01210005].i0 = 1;
		else
			Sysvar[0x01210005].i0 = 0;

		//*** Motor On 상태 업데이트, 1=Off, 0=On
		if(Sysvar[0x0125007B].i7 == 0)
			Sysvar[0x01210005].i2 = 1;
		else
			Sysvar[0x01210005].i2 = 0;

		//*** 위치값 업데이트
		UpdatePosition(AXIS0);
	}
}

SmState Axis3_State
{
}

/*********************************************************************
** State Machine Definitions
*********************************************************************/
SmMachine Axis0StateMachine {1, *, Axis0_State, 20, 2}
SmMachine Axis1StateMachine {2, *, Axis1_State, 20, 2}
SmMachine Axis2StateMachine {3, *, Axis2_State, 20, 2}
SmMachine Axis3StateMachine {4, *, Axis3_State, 20, 2}

void InitAxis0()
{
     // Amplifier setup

       sdkSetupAmpPmsmMotor(AXIS0,
								AXIS0_CONTROLMODE,
								AXIS0_POLEPAIRS,
								AXIS0_MAXCUR,
								AXIS0_ENCRES,
								AXIS0_MAX_RPM

								);

	   sdkSetupIncEncoder(		AXIS0,
								AXIS0_ENCPORT,
								AXIS0_ENCRES,
								AXIS0_ENC_LATCHTYPE,
								AXIS0_ENC_LATCHPARAM,
								AXIS0_ENC_LATCHSLOPE
								);


	// Current control setup
	sdkSetupCurrentPIControl( 	AXIS0,
								AXIS0_CURKPROP,//Current Regulator Calculator에서 계산한 값들을 넣게 되어 있음.
								AXIS0_CURKINT,
								AXIS0_CURKILIM
								);
	// Velocity control setup
	sdkSetupVelocityPIControl( 	AXIS0,
								AXIS0_VELKPROP,
								AXIS0_VELKINT,
								AXIS0_VELKILIM
								);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	AXIS0,
								AXIS0_VELRES,
								AXIS0_MAX_RPM,
								AXIS0_RAMPTYPE,
								AXIS0_RAMPMIN,
								AXIS0_JERKMIN
								);
	// Set the direction of the axis
	sdkSetupAxisDirection( 		AXIS0,
								AXIS0_DIRECTION);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	AXIS0,
									AXIS0_KPROP,
									AXIS0_KINT,
									AXIS0_KDER,
									AXIS0_KILIM,
									AXIS0_KILIMTIME,
									AXIS0_BANDWIDTH,
									AXIS0_FFVEL,
									AXIS0_KFFAC,
									AXIS0_KFFDEC
									);
	// Definition of the user units
	sdkSetupAxisUserUnits(		AXIS0,
								AXIS0_POSENCREV,
								AXIS0_POSENCQC,
								AXIS0_POSFACT_Z,
								AXIS0_POSFACT_N,
								AXIS0_FEEDREV,
								AXIS0_FEEDDIST
								);
     sdkMotorAlignment(			AXIS0,
								AXIS0_BRUSHLESS,
								AXIS0_MAXCUR,
								AXIS0_ALIGN_CUR
								);
	VIRTAMP_PARAM(AXIS0,VIRTAMP_I2TLIMIT) =	(AXIS0_CONCUR/1000)*(AXIS0_CONCUR/1000)*1000;
	VIRTAMP_PARAM(AXIS0,VIRTAMP_I2TTIME)  = AXIS0_THERMAL_TIME;
	AXE_PARAM(AXIS0,POSERR)               = 2000000000;

	print("Setup finished");
}

void InitAxis1()
{
     // Amplifier setup

       sdkSetupAmpPmsmMotor(AXIS1,
								AXIS1_CONTROLMODE,
								AXIS1_POLEPAIRS,
								AXIS1_MAXCUR,
								AXIS1_ENCRES,
								AXIS1_MAX_RPM

								);

	   sdkSetupIncEncoder(		AXIS1,
								AXIS1_ENCPORT,
								AXIS1_ENCRES,
								AXIS1_ENC_LATCHTYPE,
								AXIS1_ENC_LATCHPARAM,
								AXIS1_ENC_LATCHSLOPE
								);


	// Current control setup
	sdkSetupCurrentPIControl( 	AXIS1,
								AXIS1_CURKPROP,//Current Regulator Calculator에서 계산한 값들을 넣게 되어 있음.
								AXIS1_CURKINT,
								AXIS1_CURKILIM
								);
	// Velocity control setup
	sdkSetupVelocityPIControl( 	AXIS1,
								AXIS1_VELKPROP,
								AXIS1_VELKINT,
								AXIS1_VELKILIM
								);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	AXIS1,
								AXIS1_VELRES, //100으로 설정되어 있음.
								AXIS1_MAX_RPM, // 6900RPM
								AXIS1_RAMPTYPE, // 0으로 선언 . Trapezoidal.
								AXIS1_RAMPMIN, // 1000ms
								AXIS1_JERKMIN  // 1000ms
								);
	// Set the direction of the axis
	sdkSetupAxisDirection( 		AXIS1,
								AXIS1_DIRECTION);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	AXIS1,
									AXIS1_KPROP,
									AXIS1_KINT,
									AXIS1_KDER,
									AXIS1_KILIM,
									AXIS1_KILIMTIME,
									AXIS1_BANDWIDTH,
									AXIS1_FFVEL,
									AXIS1_KFFAC,
									AXIS1_KFFDEC
									);
	// Definition of the user units
	sdkSetupAxisUserUnits(		AXIS1,
								AXIS1_POSENCREV,
								AXIS1_POSENCQC,
								AXIS1_POSFACT_Z,
								AXIS1_POSFACT_N,
								AXIS1_FEEDREV,
								AXIS1_FEEDDIST
								);
     sdkMotorAlignment(			AXIS1,
								AXIS1_BRUSHLESS,
								AXIS1_MAXCUR,
								AXIS1_ALIGN_CUR
								);
     VIRTAMP_PARAM(AXIS1,VIRTAMP_I2TLIMIT)	=	(AXIS1_CONCUR/1000)*(AXIS1_CONCUR/1000)*1000;
     VIRTAMP_PARAM(AXIS1,VIRTAMP_I2TTIME)	= 	AXIS1_THERMAL_TIME;
    AXE_PARAM(AXIS1,POSERR)				= 	2000000000;

    print("Setup finished");
}

void InitAxis2()
{
     // Amplifier setup

	sdkSetupAmpPmsmMotor(AXIS2,
	                     AXIS2_CONTROLMODE,
	                     AXIS2_POLEPAIRS,
	                     AXIS2_MAXCUR,
	                     AXIS2_ENCRES,
	                     AXIS2_MAX_RPM);

	sdkSetupIncEncoder(AXIS2,
	                   AXIS2_ENCPORT,
	                   AXIS2_ENCRES,
	                   AXIS2_ENC_LATCHTYPE,
	                   AXIS2_ENC_LATCHPARAM,
	                   AXIS2_ENC_LATCHSLOPE);


	// Current control setup
	sdkSetupCurrentPIControl(AXIS2,
	                         AXIS2_CURKPROP,  //Current Regulator Calculator에서 계산한 값들을 넣게 되어 있음.
	                         AXIS2_CURKINT,
	                         AXIS2_CURKILIM);

	// Velocity control setup
	sdkSetupVelocityPIControl(AXIS2,
	                          AXIS2_VELKPROP,
	                          AXIS2_VELKINT,
	                          AXIS2_VELKILIM);

	// Movement parameters for the axis
	sdkSetupAxisMovementParam(AXIS2,
	                          AXIS2_VELRES,   // 100으로 설정되어 있음.
	                          AXIS2_MAX_RPM,  // 6900RPM
	                          AXIS2_RAMPTYPE, // 0으로 선언 . Trapezoidal.
	                          AXIS2_RAMPMIN,  // 1000ms
	                          AXIS2_JERKMIN); // 1000ms

	// Set the direction of the axis
	sdkSetupAxisDirection(AXIS2, AXIS2_DIRECTION);

	// Position control setup
	sdkSetupPositionPIDControlExt(AXIS2,
	                              AXIS2_KPROP,
	                              AXIS2_KINT,
	                              AXIS2_KDER,
	                              AXIS2_KILIM,
	                              AXIS2_KILIMTIME,
	                              AXIS2_BANDWIDTH,
	                              AXIS2_FFVEL,
	                              AXIS2_KFFAC,
	                              AXIS2_KFFDEC);

	// Definition of the user units
	sdkSetupAxisUserUnits(AXIS2,
	                      AXIS2_POSENCREV,
	                      AXIS2_POSENCQC,
	                      AXIS2_POSFACT_Z,
	                      AXIS2_POSFACT_N,
	                      AXIS2_FEEDREV,
	                      AXIS2_FEEDDIST);

	sdkMotorAlignment(AXIS2, AXIS2_BRUSHLESS, AXIS2_MAXCUR, AXIS2_ALIGN_CUR);

	VIRTAMP_PARAM(AXIS2,VIRTAMP_I2TLIMIT) = (AXIS2_CONCUR/1000)*(AXIS2_CONCUR/1000)*1000;
	VIRTAMP_PARAM(AXIS2,VIRTAMP_I2TTIME) = AXIS2_THERMAL_TIME;
	AXE_PARAM(AXIS2,POSERR) = 2000000000;

    print("Setup finished");
}

void InitAxis3()
{
	// Amplifier setup

	sdkSetupAmpPmsmMotor(AXIS3,
                         AXIS3_CONTROLMODE,
                         AXIS3_POLEPAIRS,
                         AXIS3_MAXCUR,
                         AXIS3_ENCRES,
                         AXIS3_MAX_RPM);

	sdkSetupIncEncoder(AXIS3,
	                   AXIS3_ENCPORT,
	                   AXIS3_ENCRES,
	                   AXIS3_ENC_LATCHTYPE,
	                   AXIS3_ENC_LATCHPARAM,
	                   AXIS3_ENC_LATCHSLOPE);

	// Current control setup
	sdkSetupCurrentPIControl(AXIS3,
	                         AXIS3_CURKPROP,//Current Regulator Calculator에서 계산한 값들을 넣게 되어 있음.
	                         AXIS3_CURKINT,
	                         AXIS3_CURKILIM);

	// Velocity control setup
	sdkSetupVelocityPIControl(AXIS3,
	                          AXIS3_VELKPROP,
	                          AXIS3_VELKINT,
	                          AXIS3_VELKILIM);

	// Movement parameters for the axis
	sdkSetupAxisMovementParam(AXIS3,
	                          AXIS3_VELRES, //100으로 설정되어 있음.
	                          AXIS3_MAX_RPM, // 6900RPM
	                          AXIS3_RAMPTYPE, // 0으로 선언 . Trapezoidal.
	                          AXIS3_RAMPMIN, // 1000ms
	                          AXIS3_JERKMIN);  // 1000ms

	// Set the direction of the axis
	sdkSetupAxisDirection(AXIS3, AXIS3_DIRECTION);

	// Position control setup
	sdkSetupPositionPIDControlExt(AXIS3,
	                              AXIS3_KPROP,
	                              AXIS3_KINT,
	                              AXIS3_KDER,
	                              AXIS3_KILIM,
	                              AXIS3_KILIMTIME,
	                              AXIS3_BANDWIDTH,
	                              AXIS3_FFVEL,
	                              AXIS3_KFFAC,
	                              AXIS3_KFFDEC);

	// Definition of the user units
	sdkSetupAxisUserUnits(AXIS3,
	                      AXIS3_POSENCREV,
	                      AXIS3_POSENCQC,
	                      AXIS3_POSFACT_Z,
	                      AXIS3_POSFACT_N,
	                      AXIS3_FEEDREV,
	                      AXIS3_FEEDDIST);

	sdkMotorAlignment(AXIS3, AXIS3_BRUSHLESS, AXIS3_MAXCUR, AXIS3_ALIGN_CUR);

	VIRTAMP_PARAM(AXIS3,VIRTAMP_I2TLIMIT) = (AXIS3_CONCUR/1000)*(AXIS3_CONCUR/1000)*1000;
	VIRTAMP_PARAM(AXIS3,VIRTAMP_I2TTIME) = AXIS3_THERMAL_TIME;
    AXE_PARAM(AXIS3,POSERR) = 2000000000;

    print("Setup finished");
}

long main(void)
{
	long res;

    ErrorClear();

    print("Motor Init start");
    //*** 각축별 초기화
    InitAxis0();
   // InitAxis1();
  //  InitAxis2();
 //   InitAxis3();
    print("Motor Init Finished");

    //*** 모터 Off
    AxisControl(AXIS0, OFF);




	//*** State Machine 시작
    res = SmRun(Axis0StateMachine, Axis1StateMachine, Axis2StateMachine, Axis3StateMachine);

    return(0);
}

//$X {User Parameter (48),1,1,0,-1,0,-1,0,(-1),-1},0x2201,48,0
//$X {Element (9),1,1,0,-1,0,-1,0,(-1),-1},0x2100,13,0
//$X {User Parameter (49),1,1,0,-1,0,-1,0,(-1),-1},0x2201,49,0
//$X {User Parameter (50),1,1,0,-1,0,-1,0,(-1),-1},0x2201,50,0
//$X {User Parameter (51),1,1,0,-1,0,-1,0,(-1),-1},0x2201,51,0
//$X {User Parameter (52),1,1,0,-1,0,-1,0,(-1),-1},0x2201,52,0
//$X {User Parameter (53),1,1,0,-1,0,-1,0,(-1),-1},0x2201,53,0
//$X {User Parameter (54),1,1,0,-1,0,-1,0,(-1),-1},0x2201,54,0
//$X {errAxis,1,1,0,10,0,132,0,(-1),1},0x2000,133,0
//$X {errNo,1,1,0,10,0,133,0,(-1),1},0x2000,134,0
//$X {USERVEL,1,1,0,10,0,134,0,(-1),1},0x2000,135,0
//$X {JOG_Mode,1,1,0,10,0,136,0,(-1),1},0x2000,137,0
//$X {Direction,1,1,0,6,0,124,0,(-1),1},0x2000,125,0
//$X {AnalogValue,8,2,0,7,0,125,0,(-1),1},0x2000,126,3
//$X {dSpeed,8,2,0,8,0,127,0,(-1),1},0x2000,128,3
