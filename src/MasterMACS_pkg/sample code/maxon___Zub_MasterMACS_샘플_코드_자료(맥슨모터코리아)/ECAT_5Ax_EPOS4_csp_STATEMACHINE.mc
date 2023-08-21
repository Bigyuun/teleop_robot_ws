
#include <SysDef.mh>
#include "..\..\include\SDK\SDK_ApossC.mc"
#define C_AXIS1	0						// Axis module number
#define C_AXIS2	1						// Axis module number
#define C_AXIS3	2						// Axis module number
#define C_AXIS4 3
#define C_AXIS5 4
#define C_AXIS6 5
#define C_AXIS7 6

#define C_AXIS1_POLARITY 	0		// Definition of the polarity 0: Normal, 1: Inverse
#define C_AXIS2_POLARITY 	0		// Definition of the polarity 0: Normal, 1: Inverse
#define C_AXIS3_POLARITY 	0		// Definition of the polarity 0: Normal, 1: Inverse
#define C_AXIS4_POLARITY    0
#define C_AXIS5_POLARITY 	0		// Definition of the polarity 0: Normal, 1: Inverse
#define C_AXIS6_POLARITY    0

#define C_DRIVE_BUSID1 1000001			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus
#define C_DRIVE_BUSID2 1000002			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus
#define C_DRIVE_BUSID3 1000003
#define C_DRIVE_BUSID4 1000004
#define C_DRIVE_BUSID5 1000005
#define C_DRIVE_BUSID6 1000006
#define C_DRIVE_BUSID7 1000007


#define C_EC_CYCLE_TIME	1				// Cycletime in milliseconds
#define C_EC_OFFSET		0				// Shift offset
#define C_PDO_NUMBER	1				// Used PDO number

#define C_MOTOR_MAX_RPM		2000		// Maximum velocity in RPM



// EPOS4 operation mode
#define C_EPOS4_OP_MODE_HOMING 	6						// x6060 Operation mode ”6: Homing mode.”
#define C_EPOS4_OP_MODE_CSP		8						// x6060 Operation mode ”8: CSP.”

// Encoder settings & axis user units (MACS)
#define C_AXIS_ENCRES 			4*1024						// Resolution of the encoder for position feed back in increments (quadcounts)
#define	C_AXIS_POSENCREV		1							// Number of revolutions of the motor
#define	C_AXIS_POSENCQC			C_AXIS_ENCRES				// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS_POSFACT_Z		1							// Number of revolutions of the input shaft
#define	C_AXIS_POSFACT_N		1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS_FEEDREV			1							// Number of revolutions of the gear box output shaft
#define	C_AXIS_FEEDDIST			C_AXIS_ENCRES				// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft [mm]

// Axis Movement Parameter
#define C_AXIS_MAX_RPM			2000					// Maximum velocity in RPM
#define C_AXIS_VELRES			100						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands, default
#define C_AXIS_RAMPTYPE			0	// Defines the ramptype
#define C_AXIS_RAMPMIN			800						// Maximum acceleration
#define C_AXIS_JERKMIN			1000					// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS_TRACKERR			2000000					// There is also a following error on EPOS4, could be very high ond the MACS

// Axis MACS control loop settings
// MACS position control is not used
#define	C_AXIS_KPROP			0
#define	C_AXIS_KINT				0
#define	C_AXIS_KDER				0
#define	C_AXIS_KILIM			0
#define	C_AXIS_KILIMTIME		0
#define	C_AXIS_BANDWIDTH		1000
#define	C_AXIS_FFVEL			1000
#define	C_AXIS_KFFAC			0
#define	C_AXIS_KFFDEC			0
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
// Encoder settings & axis user units (MACS)
#define C_AXIS2_ENCRES 			42						// Resolution of the encoder for position feed back in increments (quadcounts)
#define	C_AXIS2_POSENCREV		1							// Number of revolutions of the motor
#define	C_AXIS2_POSENCQC			C_AXIS2_ENCRES				// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS2_POSFACT_Z		1							// Number of revolutions of the input shaft
#define	C_AXIS2_POSFACT_N		1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS2_FEEDREV			1							// Number of revolutions of the gear box output shaft
#define	C_AXIS2_FEEDDIST			C_AXIS2_ENCRES				// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft [mm]

// Axis Movement Parameter
#define C_AXIS2_MAX_RPM			2000					// Maximum velocity in RPM
#define C_AXIS2_VELRES			100						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands, default
#define C_AXIS2_RAMPTYPE			0	// Defines the ramptype
#define C_AXIS2_RAMPMIN			2000						// Maximum acceleration
#define C_AXIS2_JERKMIN			1000					// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS2_TRACKERR			200000000					// There is also a following error on EPOS4, could be very high ond the MACS

// Axis MACS control loop settings
// MACS position control is not used
#define	C_AXIS2_KPROP			0
#define	C_AXIS2_KINT				0
#define	C_AXIS2_KDER				0
#define	C_AXIS2_KILIM			0
#define	C_AXIS2_KILIMTIME		0
#define	C_AXIS2_BANDWIDTH		1000
#define	C_AXIS2_FFVEL			1000
#define	C_AXIS2_KFFAC			0
#define	C_AXIS2_KFFDEC			0
///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

// Encoder settings & axis user units (MACS)
#define C_AXIS3_ENCRES 			42						// Resolution of the encoder for position feed back in increments (quadcounts)
#define	C_AXIS3_POSENCREV		1							// Number of revolutions of the motor
#define	C_AXIS3_POSENCQC			C_AXIS3_ENCRES				// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS3_POSFACT_Z		1							// Number of revolutions of the input shaft
#define	C_AXIS3_POSFACT_N		1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS3_FEEDREV			1							// Number of revolutions of the gear box output shaft
#define	C_AXIS3_FEEDDIST			C_AXIS3_ENCRES				// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft [mm]

// Axis Movement Parameter
#define C_AXIS3_MAX_RPM			2000					// Maximum velocity in RPM
#define C_AXIS3_VELRES			100						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands, default
#define C_AXIS3_RAMPTYPE			0	// Defines the ramptype
#define C_AXIS3_RAMPMIN			2000						// Maximum acceleration
#define C_AXIS3_JERKMIN			1000					// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS3_TRACKERR			200000000				// There is also a following error on EPOS4, could be very high ond the MACS

// Axis MACS control loop settings
// MACS position control is not used
#define	C_AXIS3_KPROP			0
#define	C_AXIS3_KINT				0
#define	C_AXIS3_KDER				0
#define	C_AXIS3_KILIM			0
#define	C_AXIS3_KILIMTIME		0
#define	C_AXIS3_BANDWIDTH		1000
#define	C_AXIS3_FFVEL			1000
#define	C_AXIS3_KFFAC			0
#define	C_AXIS3_KFFDEC			0
///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

// Encoder settings & axis user units (MACS)
#define C_AXIS4_ENCRES 			42						// Resolution of the encoder for position feed back in increments (quadcounts)
#define	C_AXIS4_POSENCREV		1							// Number of revolutions of the motor
#define	C_AXIS4_POSENCQC			C_AXIS4_ENCRES				// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS4_POSFACT_Z		1							// Number of revolutions of the input shaft
#define	C_AXIS4_POSFACT_N		1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS4_FEEDREV			1							// Number of revolutions of the gear box output shaft
#define	C_AXIS4_FEEDDIST			C_AXIS4_ENCRES				// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft [mm]

// Axis Movement Parameter
#define C_AXIS4_MAX_RPM			2000					// Maximum velocity in RPM
#define C_AXIS4_VELRES			100						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands, default
#define C_AXIS4_RAMPTYPE			0	// Defines the ramptype
#define C_AXIS4_RAMPMIN			2000						// Maximum acceleration
#define C_AXIS4_JERKMIN			1000					// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS4_TRACKERR			200000000					// There is also a following error on EPOS4, could be very high ond the MACS

// Axis MACS control loop settings
// MACS position control is not used
#define	C_AXIS4_KPROP			0
#define	C_AXIS4_KINT				0
#define	C_AXIS4_KDER				0
#define	C_AXIS4_KILIM			0
#define	C_AXIS4_KILIMTIME		0
#define	C_AXIS4_BANDWIDTH		1000
#define	C_AXIS4_FFVEL			1000
#define	C_AXIS4_KFFAC			0
#define	C_AXIS4_KFFDEC			0
///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

// Encoder settings & axis user units (MACS)
#define C_AXIS5_ENCRES 			4*1024						// Resolution of the encoder for position feed back in increments (quadcounts)
#define	C_AXIS5_POSENCREV		1							// Number of revolutions of the motor
#define	C_AXIS5_POSENCQC			C_AXIS5_ENCRES				// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS5_POSFACT_Z		1							// Number of revolutions of the input shaft
#define	C_AXIS5_POSFACT_N		1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS5_FEEDREV			1							// Number of revolutions of the gear box output shaft
#define	C_AXIS5_FEEDDIST			C_AXIS5_ENCRES				// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft [mm]

// Axis Movement Parameter
#define C_AXIS5_MAX_RPM			2000					// Maximum velocity in RPM
#define C_AXIS5_VELRES			100						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands, default
#define C_AXIS5_RAMPTYPE			0	// Defines the ramptype
#define C_AXIS5_RAMPMIN			2000						// Maximum acceleration
#define C_AXIS5_JERKMIN			1000					// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS5_TRACKERR			200000000					// There is also a following error on EPOS4, could be very high ond the MACS

// Axis MACS control loop settings
// MACS position control is not used
#define	C_AXIS5_KPROP			0
#define	C_AXIS5_KINT				0
#define	C_AXIS5_KDER				0
#define	C_AXIS5_KILIM			0
#define	C_AXIS5_KILIMTIME		0
#define	C_AXIS5_BANDWIDTH		1000
#define	C_AXIS5_FFVEL			1000
#define	C_AXIS5_KFFAC			0
#define	C_AXIS5_KFFDEC			0

///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

// Encoder settings & axis user units (MACS)
#define C_AXIS6_ENCRES 			42						// Resolution of the encoder for position feed back in increments (quadcounts)
#define	C_AXIS6_POSENCREV		1							// Number of revolutions of the motor
#define	C_AXIS6_POSENCQC			C_AXIS6_ENCRES				// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS6_POSFACT_Z		1							// Number of revolutions of the input shaft
#define	C_AXIS6_POSFACT_N		1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS6_FEEDREV			1							// Number of revolutions of the gear box output shaft
#define	C_AXIS6_FEEDDIST			C_AXIS6_ENCRES				// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft [mm]

// Axis Movement Parameter
#define C_AXIS6_MAX_RPM			2000					// Maximum velocity in RPM
#define C_AXIS6_VELRES			100						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands, default
#define C_AXIS6_RAMPTYPE			0	// Defines the ramptype
#define C_AXIS6_RAMPMIN			2000						// Maximum acceleration
#define C_AXIS6_JERKMIN			1000					// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS6_TRACKERR			200000000					// There is also a following error on EPOS4, could be very high ond the MACS

// Axis MACS control loop settings
// MACS position control is not used
#define	C_AXIS6_KPROP			0
#define	C_AXIS6_KINT				0
#define	C_AXIS6_KDER				0
#define	C_AXIS6_KILIM			0
#define	C_AXIS6_KILIMTIME		0
#define	C_AXIS6_BANDWIDTH		1000
#define	C_AXIS6_FFVEL			1000
#define	C_AXIS6_KFFAC			0
#define	C_AXIS6_KFFDEC			0

long slaveCount, i,j,k, homingState,slaveState;


	//$B
#if 1
/*********************************************************************
**              State Machine Version of Program
*********************************************************************/

/*********************************************************************
** State Machine Setup Parameters
*********************************************************************/

#pragma SmConfig {    1,        // Runtime flags.
                      25,       // Event pool size.
                      5,        // Maximum number of timers.
                      5,        // Subscribe pool size.
                      12,        // Param pool size.
                      0,        // Position pool size.
                      2 }       // System signal pool size (used for SmSystem.)
/*********************************************************************
** Event Definitions
*********************************************************************/

SmEvent SIG_PLAY { }
SmEvent SIG_STOP { }
SmEvent SIG_CLEAR { }
SmEvent SIG_REINIT { }
SmEvent SIG_PREOP { }
SmEvent SIG_OP { }
SmEvent SIG_ENABLE { }
SmEvent SIG_DISABLE { }

/*********************************************************************
** State Definitions
*********************************************************************/


SmState MyMachine {
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

		       return(SmTrans(->Standing));
	           }
     SIG_ERROR =
    {
    	long errAxis, errNo;

		errAxis = ErrorAxis();
		errNo = ErrorNo();



		return(SmTrans(->Standing));
	}
	SmState Standing {
		               SIG_ENTRY = {
			                        print(" into the Standing State ");


		                           }

		               SIG_PLAY = { print("PLAY");
		                            AxisPosRelStart(C_AXIS1,20000,C_AXIS2,20000,C_AXIS3,20000,C_AXIS4,20000,C_AXIS5,20000,C_AXIS6,20000  );
		                            Sysvar[0x01220105] = 0;
		                          }
		               SIG_STOP = { print("STOP");
		                            AxisStop(AXALL);
		                            Sysvar[0x01220105] = 0;
		                          }
		               SIG_REINIT =  EtherCAT_REINIT ;
                       SIG_PREOP =  EtherCAT_PREOP ;
                       SIG_OP =  EtherCAT_OP ;

		               SIG_CLEAR = { print("CLEAR");
		                             ErrorClear();
		                             AmpErrorClear(AXALL);
		                             Sysvar[0x01220105] = 0;
		                           }
		               SIG_ENABLE = { print("Enable");
		                              AxisControl(AXALL,ON);
		                            }
                       SIG_DISABLE = {
                                      print("Disable");
                                      AxisControl(AXALL,OFF);
                                     }


	                  }


                  }

	//$B


/*********************************************************************
** State Machine Definitions
*********************************************************************/

SmMachine Operation {1, *, MyMachine, 20, 2}



long main(void)
{
    long slaveCount, i,retval,retval1,res,retval2;

	ErrorClear();
	AmpErrorClear(C_AXIS1); // Clear error on EPOS4
	AmpErrorClear(C_AXIS2); // Clear error on EPOS4
	AmpErrorClear(C_AXIS3); // Clear error on EPOS4
    AmpErrorClear(C_AXIS4);
    AmpErrorClear(C_AXIS5); // Clear error on EPOS4

    print(Apos(0));
    print(Apos(1));
    print(Apos(2));
    print(Apos(3));
    print(Apos(4 ));

    //AmpErrorClear(C_AXIS6);



	ECatMasterCommand(0x1000, 0);


	//----------------------------------------------------------------
	// Application Setup
	//----------------------------------------------------------------

	slaveCount = sdkEtherCATMasterInitialize();
	print("slavecount: ",slaveCount);

	// initialising maxon drives
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID1, C_PDO_NUMBER, C_AXIS1_POLARITY, EPOS4_OP_CSP );
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID2, C_PDO_NUMBER, C_AXIS2_POLARITY, EPOS4_OP_CSP );
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID3, C_PDO_NUMBER, C_AXIS3_POLARITY, EPOS4_OP_CSP );
    sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID4, C_PDO_NUMBER, C_AXIS4_POLARITY, EPOS4_OP_CSP );
    sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID5, C_PDO_NUMBER, C_AXIS5_POLARITY, EPOS4_OP_CSP );
    //sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID6, C_PDO_NUMBER, C_AXIS6_POLARITY, EPOS4_OP_CSP );
	for (i = 1; i <= 5; i++) {
	   sdkEtherCATSetupDC(i, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]
	   print(i, " pos : ", Apos(i));
    }


	// starting the ethercat
	sdkEtherCATMasterStart();
	print("sdkEtherCATMasterStart fin");
	// setup EtherCAT bus module for csp mode
	sdkEpos4_SetupECatBusModule(C_AXIS1, C_DRIVE_BUSID1, C_PDO_NUMBER, EPOS4_OP_CSP);
	sdkEpos4_SetupECatBusModule(C_AXIS2, C_DRIVE_BUSID2, C_PDO_NUMBER, EPOS4_OP_CSP);
	sdkEpos4_SetupECatBusModule(C_AXIS3, C_DRIVE_BUSID3, C_PDO_NUMBER, EPOS4_OP_CSP);
    sdkEpos4_SetupECatBusModule(C_AXIS4, C_DRIVE_BUSID4, C_PDO_NUMBER, EPOS4_OP_CSP);
    sdkEpos4_SetupECatBusModule(C_AXIS5, C_DRIVE_BUSID5, C_PDO_NUMBER, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatBusModule(C_AXIS6, C_DRIVE_BUSID6, C_PDO_NUMBER, EPOS4_OP_CSP);

	// setup virtual amplifier for csp mode
	sdkEpos4_SetupECatVirtAmp(C_AXIS1, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtAmp(C_AXIS2, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtAmp(C_AXIS3, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
    sdkEpos4_SetupECatVirtAmp(C_AXIS4, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtAmp(C_AXIS5, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatVirtAmp(C_AXIS6, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	//
	// setup irtual counter for csp mode
	sdkEpos4_SetupECatVirtCntin(C_AXIS1, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtCntin(C_AXIS2, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtCntin(C_AXIS3, EPOS4_OP_CSP);
    sdkEpos4_SetupECatVirtCntin(C_AXIS4, EPOS4_OP_CSP);
    sdkEpos4_SetupECatVirtCntin(C_AXIS5, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatVirtCntin(C_AXIS6, EPOS4_OP_CSP);
	// All axis have in this example the same parameters
	for (i = 0; i < 1; i++) {
		// Movement parameters for the axis
		sdkSetupAxisMovementParam(	i,
									C_AXIS_VELRES,
									C_AXIS_MAX_RPM,
									C_AXIS_RAMPTYPE,
									C_AXIS_RAMPMIN,
									C_AXIS_JERKMIN
									);

		// Definition of the user units
		sdkSetupAxisUserUnits(		i,
									C_AXIS_POSENCREV,
									C_AXIS_POSENCQC,
									C_AXIS_POSFACT_Z,
									C_AXIS_POSFACT_N,
									C_AXIS_FEEDREV,
									C_AXIS_FEEDDIST
									);
		// Position control setup
		sdkSetupPositionPIDControlExt( 	i,
										C_AXIS_KPROP,
										C_AXIS_KINT,
										C_AXIS_KDER,
										C_AXIS_KILIM,
										C_AXIS_KILIMTIME,
										C_AXIS_BANDWIDTH,
										C_AXIS_FFVEL,
										C_AXIS_KFFAC,
										C_AXIS_KFFDEC
										);
	}

	    sdkSetupAxisMovementParam(	1,
									C_AXIS2_VELRES,
									C_AXIS2_MAX_RPM,
									C_AXIS2_RAMPTYPE,
									C_AXIS2_RAMPMIN,
									C_AXIS2_JERKMIN
									);

		// Definition of the user units
		sdkSetupAxisUserUnits(		1,
									C_AXIS2_POSENCREV,
									C_AXIS2_POSENCQC,
									C_AXIS2_POSFACT_Z,
									C_AXIS2_POSFACT_N,
									C_AXIS2_FEEDREV,
									C_AXIS2_FEEDDIST
									);
		// Position control setup
		sdkSetupPositionPIDControlExt( 	1,
										C_AXIS2_KPROP,
										C_AXIS2_KINT,
										C_AXIS2_KDER,
										C_AXIS2_KILIM,
										C_AXIS2_KILIMTIME,
										C_AXIS2_BANDWIDTH,
										C_AXIS2_FFVEL,
										C_AXIS2_KFFAC,
										C_AXIS2_KFFDEC
										);
         sdkSetupAxisMovementParam(	2,
									C_AXIS3_VELRES,
									C_AXIS3_MAX_RPM,
									C_AXIS3_RAMPTYPE,
									C_AXIS3_RAMPMIN,
									C_AXIS3_JERKMIN
									);

		// Definition of the user units
		sdkSetupAxisUserUnits(		2,
									C_AXIS3_POSENCREV,
									C_AXIS3_POSENCQC,
									C_AXIS3_POSFACT_Z,
									C_AXIS3_POSFACT_N,
									C_AXIS3_FEEDREV,
									C_AXIS3_FEEDDIST
									);
		// Position control setup
		sdkSetupPositionPIDControlExt( 	2,
										C_AXIS3_KPROP,
										C_AXIS3_KINT,
										C_AXIS3_KDER,
										C_AXIS3_KILIM,
										C_AXIS3_KILIMTIME,
										C_AXIS3_BANDWIDTH,
										C_AXIS3_FFVEL,
										C_AXIS3_KFFAC,
										C_AXIS3_KFFDEC
										);
	        sdkSetupAxisMovementParam(	3,
									C_AXIS4_VELRES,
									C_AXIS4_MAX_RPM,
									C_AXIS4_RAMPTYPE,
									C_AXIS4_RAMPMIN,
									C_AXIS4_JERKMIN
									);

		// Definition of the user units
		sdkSetupAxisUserUnits(		3,
									C_AXIS4_POSENCREV,
									C_AXIS4_POSENCQC,
									C_AXIS4_POSFACT_Z,
									C_AXIS4_POSFACT_N,
									C_AXIS4_FEEDREV,
									C_AXIS4_FEEDDIST
									);
		// Position control setup
		sdkSetupPositionPIDControlExt( 	3,
										C_AXIS4_KPROP,
										C_AXIS4_KINT,
										C_AXIS4_KDER,
										C_AXIS4_KILIM,
										C_AXIS4_KILIMTIME,
										C_AXIS4_BANDWIDTH,
										C_AXIS4_FFVEL,
										C_AXIS4_KFFAC,
										C_AXIS4_KFFDEC
										);
        sdkSetupAxisMovementParam(	4,
									C_AXIS5_VELRES,
									C_AXIS5_MAX_RPM,
									C_AXIS5_RAMPTYPE,
									C_AXIS5_RAMPMIN,
									C_AXIS5_JERKMIN
									);

		// Definition of the user units
		sdkSetupAxisUserUnits(		4,
									C_AXIS5_POSENCREV,
									C_AXIS5_POSENCQC,
									C_AXIS5_POSFACT_Z,
									C_AXIS5_POSFACT_N,
									C_AXIS5_FEEDREV,
									C_AXIS5_FEEDDIST
									);
		// Position control setup
		sdkSetupPositionPIDControlExt( 	4,
										C_AXIS5_KPROP,
										C_AXIS5_KINT,
										C_AXIS5_KDER,
										C_AXIS5_KILIM,
										C_AXIS5_KILIMTIME,
										C_AXIS5_BANDWIDTH,
										C_AXIS5_FFVEL,
										C_AXIS5_KFFAC,
										C_AXIS5_KFFDEC
										);
//       sdkSetupAxisMovementParam(	5,
//									C_AXIS6_VELRES,
//									C_AXIS6_MAX_RPM,
//									C_AXIS6_RAMPTYPE,
//									C_AXIS6_RAMPMIN,
//									C_AXIS6_JERKMIN
//									);
//
//		// Definition of the user units
//		sdkSetupAxisUserUnits(		5,
//									C_AXIS6_POSENCREV,
//									C_AXIS6_POSENCQC,
//									C_AXIS6_POSFACT_Z,
//									C_AXIS6_POSFACT_N,
//									C_AXIS6_FEEDREV,
//									C_AXIS6_FEEDDIST
//									);
//		// Position control setup
//		sdkSetupPositionPIDControlExt( 	5,
//										C_AXIS6_KPROP,
//										C_AXIS6_KINT,
//										C_AXIS6_KDER,
//										C_AXIS6_KILIM,
//										C_AXIS6_KILIMTIME,
//										C_AXIS6_BANDWIDTH,
//										C_AXIS6_FFVEL,
//										C_AXIS6_KFFAC,
//										C_AXIS6_KFFDEC
//										);
//
	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------

	ErrorClear();

	Vel(C_AXIS4,20,C_AXIS5,20,C_AXIS6,20);   // 속도 지정
	Acc(C_AXIS4,40,C_AXIS5,40,C_AXIS6,40);   // 가속도 지정
	Dec(C_AXIS4,50,C_AXIS5,50,C_AXIS6,50);   // 감속도 지정



	//$B
  res = SmRun(Operation); // Opearation 이라는 이름의 Statemachine을 실행.

    return(0);
}

long EtherCAT_REINIT (long id, long signal, long event[], long data[]) // EtherCAT NMT 상태를 재부팅하여 Operational 상태로 만듦.
{
    long slaveCount, i,retval,res;
    print("REINIT");

    ECatMasterCommand(0x1000, 0);

	slaveCount = sdkEtherCATMasterInitialize();
    sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID1, C_PDO_NUMBER, C_AXIS1_POLARITY, EPOS4_OP_CSP );
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID2, C_PDO_NUMBER, C_AXIS2_POLARITY, EPOS4_OP_CSP );
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID3, C_PDO_NUMBER, C_AXIS3_POLARITY, EPOS4_OP_CSP );
    sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID4, C_PDO_NUMBER, C_AXIS4_POLARITY, EPOS4_OP_CSP );
    sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID5, C_PDO_NUMBER, C_AXIS5_POLARITY, EPOS4_OP_CSP );
    //sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID6, C_PDO_NUMBER, C_AXIS6_POLARITY, EPOS4_OP_CSP );
	for (i = 1; i <= 1; i++) {
	                          sdkEtherCATSetupDC(i, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]
                             }


    sdkEtherCATMasterStart();
    sdkEpos4_SetupECatBusModule(C_AXIS1, C_DRIVE_BUSID1, C_PDO_NUMBER, EPOS4_OP_CSP);
	sdkEpos4_SetupECatBusModule(C_AXIS2, C_DRIVE_BUSID2, C_PDO_NUMBER, EPOS4_OP_CSP);
	sdkEpos4_SetupECatBusModule(C_AXIS3, C_DRIVE_BUSID3, C_PDO_NUMBER, EPOS4_OP_CSP);
    sdkEpos4_SetupECatBusModule(C_AXIS4, C_DRIVE_BUSID4, C_PDO_NUMBER, EPOS4_OP_CSP);
    sdkEpos4_SetupECatBusModule(C_AXIS5, C_DRIVE_BUSID5, C_PDO_NUMBER, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatBusModule(C_AXIS6, C_DRIVE_BUSID6, C_PDO_NUMBER, EPOS4_OP_CSP);
	// setup virtual amplifier for csp mode
	sdkEpos4_SetupECatVirtAmp(C_AXIS1, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtAmp(C_AXIS2, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtAmp(C_AXIS3, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
    sdkEpos4_SetupECatVirtAmp(C_AXIS4, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtAmp(C_AXIS5, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatVirtAmp(C_AXIS6, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	//
	// setup irtual counter for csp mode
	sdkEpos4_SetupECatVirtCntin(C_AXIS1, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtCntin(C_AXIS2, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtCntin(C_AXIS3, EPOS4_OP_CSP);
    sdkEpos4_SetupECatVirtCntin(C_AXIS4, EPOS4_OP_CSP);
    sdkEpos4_SetupECatVirtCntin(C_AXIS5, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatVirtCntin(C_AXIS6, EPOS4_OP_CSP);

	ErrorClear();
    AmpErrorClear(AXALL);





    AxisControl(C_AXIS1,ON,C_AXIS2,ON,C_AXIS3,ON,C_AXIS4,ON,C_AXIS5,ON);
	Sysvar[0x01220105] = 0;
    return(SmTrans(Standing));

}	//$B


long EtherCAT_PREOP (long id, long signal, long event[], long data[])  // EtherCAT NMT State를 PRE operational 상태로 바꿈
{

ECatMasterCommand(0x1000,1);
Delay(10);
SdoWrite(1000001,0x6040,0,0x80);
SdoWrite(1000002,0x6040,0,0x80);
SdoWrite(1000003,0x6040,0,0x80);
SdoWrite(1000004,0x6040,0,0x80);
SdoWrite(1000005,0x6040,0,0x80);
//SdoWrite(1000006,0x6040,0,0x80);
Delay(10);

for (i = 1; i <= 1; i++) {
   sdkEtherCATSetupDC(i, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]
                         }
return(SmTrans(Standing));
}

long EtherCAT_OP (long id, long signal, long event[], long data[]) // EtherCAT NMT State를 Operational 상태로 바꿈
{
sdkEtherCATMasterStart();
AmpErrorClear(AXALL);


return(SmTrans(Standing));
}

//$X {User Parameter (5),1,1,0,-1,0,-1,0,(-1),-1},0x2201,5,0
