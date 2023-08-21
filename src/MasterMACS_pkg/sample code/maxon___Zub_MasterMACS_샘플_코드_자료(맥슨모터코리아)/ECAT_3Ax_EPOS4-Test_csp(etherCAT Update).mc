  /**
*	@brief		This is a testprogram which demonstrates the use of the EPOS4 SDK
*	@detail		The drive will run back and forth when the program is started. It will also
*				record some test data. This data can be displayed by pressing the "Oscilloscope(Record)"
*				button after the program has run.
*
*	$Revision: 228 $
*
*	@example 	ECAT_3Ax_EPOS4-Test_csp.mc
*
*/
#include <SysDef.mh>
#include "C:\Users\mmkhych\Desktop\Zub\SDK\ApossC-SDK-V01-13\SDK\SDK_ApossC.mc"
#define CYCLE_TIME	     		100

long ActTorque,Mode;
// Parameters for the SDK function
#define C_AXIS1	0						// Axis module number
#define C_AXIS2	1						// Axis module number
#define C_AXIS3	2						// Axis module number
#define C_AXIS4	3						// Axis module number
#define C_AXIS5	4						// Axis module number
#define C_AXIS6 5
#define C_AXIS7 6


#define C_AXIS1_POLARITY 	0		// Definition of the polarity 0: Normal, 1: Inverse
#define C_AXIS2_POLARITY 	0		// Definition of the polarity 0: Normal, 1: Inverse
#define C_AXIS3_POLARITY 	0		// Definition of the polarity 0: Normal, 1: Inverse
#define C_AXIS4_POLARITY 	0		// Definition of the polarity 0: Normal, 1: Inverse
#define C_AXIS5_POLARITY 	0		// Definition of the polarity 0: Normal, 1: Inverse
#define C_AXIS6_POLARITY    0
#define C_AXIS7_POLARITY    0

#define C_DRIVE_BUSID1 1000001			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus
#define C_DRIVE_BUSID2 1000002			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus
#define C_DRIVE_BUSID3 1000003			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus
#define C_DRIVE_BUSID4 1000004			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus
#define C_DRIVE_BUSID5 1000005			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus
#define C_DRIVE_BUSID6 1000006
#define C_DRIVE_BUSID7 1000007


#define C_EC_CYCLE_TIME	1				// Cycletime in milliseconds
#define C_EC_OFFSET		0				// Shift offset
#define C_PDO_NUMBER	1				// Used PDO number

#define	C_AXIS_KPROP			0
#define	C_AXIS_KINT				0
#define	C_AXIS_KDER				0
#define	C_AXIS_KILIM			0
#define	C_AXIS_KILIMTIME		0
#define	C_AXIS_BANDWIDTH		1000
#define	C_AXIS_FFVEL			1000
#define	C_AXIS_KFFAC			0
#define	C_AXIS_KFFDEC			0



// Encoder settings & axis user units (MACS)
#define C_AXIS_ENCRES 			48						// Resolution of the encoder for position feed back in increments (quadcounts)
#define	C_AXIS_POSENCREV		1							// Number of revolutions of the motor
#define	C_AXIS_POSENCQC			C_AXIS_ENCRES				// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS_POSFACT_Z		1							// Number of revolutions of the input shaft
#define	C_AXIS_POSFACT_N		1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS_FEEDREV			1							// Number of revolutions of the gear box output shaft
#define	C_AXIS_FEEDDIST			C_AXIS_ENCRES				// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft [mm]

// Axis Movement Parameter
#define C_AXIS_MAX_RPM			2000					// Maximum velocity in RPM
#define C_AXIS_VELRES			2000						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands, default
#define C_AXIS_RAMPTYPE			RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define C_AXIS_RAMPMIN			800 //ms						// Maximum acceleration
#define C_AXIS_JERKMIN			1000					// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS_TRACKERR			2000					// There is also a following error on EPOS4, could be very high ond the MACS

// Axis MACS control loop settings
// MACS position control is not used

void EncoderDataupdates(void);


long main(void) {


    long slaveCount, i,retval;
    long homingStateAx_0=0,homingStateAx_1=0,homingStateAx_2=0;
	print("-----------------------------------------------------------");
	print(" Test application EtherCAT Master with 3 EPOS4 drive");
	print("-----------------------------------------------------------");


	ErrorClear();
	AmpErrorClear(C_AXIS1); // Clear error on EPOS4
	AmpErrorClear(C_AXIS2); // Clear error on EPOS4
	AmpErrorClear(C_AXIS3); // Clear error on EPOS4
	AmpErrorClear(C_AXIS4); // Clear error on EPOS4
//	AmpErrorClear(C_AXIS5); // Clear error on EPOS4
//	AmpErrorClear(C_AXIS6);
//	AmpErrorClear(C_AXIS7);
	InterruptSetup(ERROR, ErrorHandler);



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
//    sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID5, C_PDO_NUMBER, C_AXIS5_POLARITY, EPOS4_OP_CSP );
//    sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID6, C_PDO_NUMBER, C_AXIS6_POLARITY, EPOS4_OP_CSP );
//    sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID7, C_PDO_NUMBER, C_AXIS7_POLARITY, EPOS4_OP_CSP );


	sdkEtherCATMasterDoMapping();

	for (i = 1; i <= 4; i++) {
	   sdkEtherCATSetupDC(i, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]
    }

	// starting the ethercat
	sdkEtherCATMasterStart();

	// setup EtherCAT bus module for csp mode
	sdkEpos4_SetupECatBusModule(C_AXIS1, C_DRIVE_BUSID1, C_PDO_NUMBER, EPOS4_OP_CSP);
	sdkEpos4_SetupECatBusModule(C_AXIS2, C_DRIVE_BUSID2, C_PDO_NUMBER, EPOS4_OP_CSP);
	sdkEpos4_SetupECatBusModule(C_AXIS3, C_DRIVE_BUSID3, C_PDO_NUMBER, EPOS4_OP_CSP);
	sdkEpos4_SetupECatBusModule(C_AXIS4, C_DRIVE_BUSID4, C_PDO_NUMBER, EPOS4_OP_CSP);
//    sdkEpos4_SetupECatBusModule(C_AXIS5, C_DRIVE_BUSID5, C_PDO_NUMBER, EPOS4_OP_CSP);
//    sdkEpos4_SetupECatBusModule(C_AXIS6, C_DRIVE_BUSID6, C_PDO_NUMBER, EPOS4_OP_CSP);
//    sdkEpos4_SetupECatBusModule(C_AXIS7, C_DRIVE_BUSID7, C_PDO_NUMBER, EPOS4_OP_CSP);


	// setup virtual amplifier for csp mode
	sdkEpos4_SetupECatVirtAmp(C_AXIS1, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtAmp(C_AXIS2, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtAmp(C_AXIS3, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtAmp(C_AXIS4, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
//	sdkEpos4_SetupECatVirtAmp(C_AXIS5, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
//	sdkEpos4_SetupECatVirtAmp(C_AXIS6, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
//	sdkEpos4_SetupECatVirtAmp(C_AXIS7, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
//

	// setup irtual counter for csp mode
	sdkEpos4_SetupECatVirtCntin(C_AXIS1, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtCntin(C_AXIS2, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtCntin(C_AXIS3, EPOS4_OP_CSP);
	sdkEpos4_SetupECatVirtCntin(C_AXIS4, EPOS4_OP_CSP);
//	sdkEpos4_SetupECatVirtCntin(C_AXIS5, EPOS4_OP_CSP);
//	sdkEpos4_SetupECatVirtCntin(C_AXIS6, EPOS4_OP_CSP);
//	sdkEpos4_SetupECatVirtCntin(C_AXIS7, EPOS4_OP_CSP);
//

	// All axis have in this example the same parameters
	for (i = 0; i < 4; i++) {
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

  //  AXE_PARAM(0,POSERR)=4000000000;
	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------

	ErrorClear();

	//----------------------------------------------------------------
	// Homing start
	//----------------------------------------------------------------

//	// Homing setup
//	for (i = 0; i < 3; i++)
//	{
//		SdoWrite( C_DRIVE_BUSID1+i, EPOS4_HOMING_METHOD, 				0,   EPOS4_HOMING_CURRENT_N_SPEED);	// 0x6098 Set homing method to “-4" : Homing Method -4 (Current Threshold Negative Speed).”
//		SdoWrite( C_DRIVE_BUSID1+i, EPOS4_HOMING_SPEEDS, 				1,   20);    						// Homing Speed / Speed for switch speed [velocity units]
//		SdoWrite( C_DRIVE_BUSID1+i, EPOS4_HOMING_SPEEDS, 				2,   20);    						// Homing Speed / Speed for zero search [velocity units]
//		SdoWrite( C_DRIVE_BUSID1+i, EPOS4_HOMING_ACCELERATION, 			0,   20);    						// Homing acceleration [acceleration units]
//		SdoWrite( C_DRIVE_BUSID1+i, EPOS4_HOME_OFFSET_MOVE_DISTANCE, 	0,   6400);   						// Home offset move distance [position units]
//		SdoWrite( C_DRIVE_BUSID1+i, EPOS4_HOME_POSITION, 				0,   0);   							// Home position [position units]
//		SdoWrite( C_DRIVE_BUSID1+i, EPOS4_CURRENT_THRESHOLD_FOR_HOMING_MODE, 0,   1500);  				// Current threshold for homing mode [mA]
//
//		// Disable MACS trackerror for homing
//		AXE_PARAM(i,POSERR)=2000000000;
//	}
//
//	// Homing statemachine
//	retval=0;
//
//	while(retval!=7)
//	{
//		retval.i[0] = 	sdkEpos4_AxisHomingStart(C_AXIS1, C_DRIVE_BUSID1, EPOS4_OP_CSP, homingStateAx_0);
//		retval.i[1] =  	sdkEpos4_AxisHomingStart(C_AXIS2, C_DRIVE_BUSID2, EPOS4_OP_CSP, homingStateAx_1);
//		retval.i[2] =  	sdkEpos4_AxisHomingStart(C_AXIS3, C_DRIVE_BUSID3, EPOS4_OP_CSP, homingStateAx_2);
//	}

	AxisControl(C_AXIS1,ON,  C_AXIS2,ON,  C_AXIS3,ON,  C_AXIS4,ON);//,  C_AXIS3,ON,  C_AXIS4,ON,  C_AXIS5,ON,  C_AXIS6,ON,  C_AXIS7,ON

    InterruptSetup(PERIOD,EncoderDataupdates,CYCLE_TIME);

	print("-----------------------------------------------------------");
	print("                Movement in CSP Mode                       ");
	print("----------------------------------------------------------- \n");

	Vel(C_AXIS1, 40, C_AXIS2, 40, C_AXIS3, 40, C_AXIS4, 40);//, C_AXIS2, 40, C_AXIS3, 40, C_AXIS4, 40, C_AXIS5, 40, C_AXIS6, 40, C_AXIS7, 40
	Acc(C_AXIS1, 30, C_AXIS2, 30, C_AXIS3, 30, C_AXIS4, 30);//, C_AXIS2, 30, C_AXIS3, 30, C_AXIS4, 30, C_AXIS5, 30, C_AXIS6, 30, C_AXIS7, 30
	Dec(C_AXIS1, 30, C_AXIS2, 30, C_AXIS3, 30, C_AXIS4, 30);//, C_AXIS2, 30, C_AXIS3, 30, C_AXIS4, 30, C_AXIS4, 30, C_AXIS6, 30, C_AXIS7, 30

	for(i=100;i>=0;i--)
	{
		print("Start, move to target position");
		AxisPosRelStart( C_AXIS1, 20000, C_AXIS2, 20000,   C_AXIS3, 20000,   C_AXIS4, 20000);//, C_AXIS2, 20000,   C_AXIS3, 20000,   C_AXIS4, 20000,   C_AXIS5, 20000,   C_AXIS6, 20000,   C_AXIS7, 20000

		AxisWaitReached(C_AXIS1,C_AXIS2,C_AXIS3,C_AXIS4);//,C_AXIS2,C_AXIS3,C_AXIS4,C_AXIS5,C_AXIS6,C_AXIS7
		print("Target position is reached \n");
		print("Start, back to start position");

		AxisPosAbsStart( C_AXIS1, 0, C_AXIS2, 0, C_AXIS3, 0, C_AXIS4, 0);//, C_AXIS2, 0, C_AXIS3, 0, C_AXIS4, 0, C_AXIS5, 0, C_AXIS6, 0, C_AXIS7, 0
		AxisWaitReached(C_AXIS1,C_AXIS2,C_AXIS3,C_AXIS4);//,C_AXIS2,C_AXIS3,C_AXIS4,C_AXIS5,C_AXIS6,C_AXIS7
		print("Start position is reached");
		print(i, " repetitions to go \n");

	}



	AxisControl(AXALL, OFF);
	RecordStop(0, 0);
	print("Program done, Axis OFF ");


    return(0);
}

void ErrorHandler(void)
{
	long axeNbr 	= ErrorAxis();
	long errNbr		= ErrorNo();
	long errInfoNbr	= ErrorInfo();

    AxisControl(AXALL,OFF);

  	switch(errNbr)
	{
		case F_AMP:		print("EPOS4 Error");
						print("Error Axis: ", axeNbr ," ErrorCode: ", radixstr(SdoRead(C_DRIVE_BUSID1,EPOS4_ERROR_CODE,0x00),16));

						AmpErrorClear(axeNbr); // Clear error on EPOS4
						break;

		case F_CANIO:	print("ErrorNo: ",errNbr," info: ",errInfoNbr);
						printf("SDO Abort Code %lX\n", SYS_PROCESS(SYS_CANOM_SDOABORT) );
						print("Check Can baudrate & Can bus id");
						break;

		default:		print("ErrorNo: ",errNbr," info: ",errInfoNbr, " AxisNo: ", axeNbr);
	}
	//ErrorClear();
	print("");	print(" There is no error handlig → Exit()");
	print("Error Axis: ", axeNbr ," ErrorCode: ", radixstr(SdoRead(C_DRIVE_BUSID1,EPOS4_ERROR_CODE,0x00),16));
	Exit(0);
}
void EncoderDataupdates(void)
{
	sdkSetupIncEncoder(0, 0, 4096, 0, 0, 0);

    Delay(1000);

    print("Position = ", Apos(0));

    return;
}

//$X {REG_ACTPOS,1,1,0,-1,0,-1,0,(-1),-1},0x2500,1,0
//$X {REG_ACTPOS,1,1,0,-1,0,-1,0,(-1),-1},0x2501,1,0
//$X {REG_ACTPOS,1,1,0,-1,0,-1,0,(-1),-1},0x2502,1,0
//$X {REG_ACTPOS,1,1,0,-1,0,-1,0,(-1),-1},0x2503,1,0
//$X {REG_ACTPOS,1,1,0,-1,0,-1,0,(-1),-1},0x2504,1,0
//$X {REG_ACTPOS,1,1,0,-1,0,-1,0,(-1),-1},0x2505,1,0
//$X {REG_ACTPOS,1,1,0,-1,0,-1,0,(-1),-1},0x2506,1,0
