#pragma once

#include <SysDef.mh>
#include "EtherCAT_definition.mh"
#include "EtherCAT_config.mh"

long EtherCAT_configuration(void)
{
	long n_slaves, i, retval, retval1, res, retval2;
	print("Start configuring EtherCAT...");

	//----------------------------------------------------------------
	// Error Clear
	//----------------------------------------------------------------
	print("Error clear & setup updating...");
	ErrorClear();

	#if g_ETHERCAT_CONFIG_INDIVIDUAL
	for(i=0;i<g_NUM_OF_SLAVES;i++)
	{
		AmpErrorClear(i);
	}
	#else
	//AmpErrorClear(AXALL); 			// Clear error on EPOS4

	for(i=0;i<g_NUM_OF_SLAVES;i++)
	{
		AmpErrorClear(C_AXIS1+i);
	}

	#endif

	ECatMasterCommand(0x1000, 0);	// The master itself


	//----------------------------------------------------------------
	// Application Setup
	//----------------------------------------------------------------
	n_slaves = sdkEtherCATMasterInitialize();
	print("Number of Slaves (from header) : ", g_NUM_OF_SLAVES);
	print("Number of Slaves (found on bus): ", n_slaves);

	//----------------------------------------------------------------
	// Initialization of MAXON Drives
	//----------------------------------------------------------------
	#if g_ETHERCAT_CONFIG_INDIVIDUAL
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID1, C_PDO_NUMBER, C_AXIS1_POLARITY, EPOS4_OP_CSV );
	//sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID2, C_PDO_NUMBER, C_AXIS2_POLARITY, EPOS4_OP_CSV );
	//sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID3, C_PDO_NUMBER, C_AXIS3_POLARITY, EPOS4_OP_CSV );
    //sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID4, C_PDO_NUMBER, C_AXIS4_POLARITY, EPOS4_OP_CSV );
    //sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID5, C_PDO_NUMBER, C_AXIS5_POLARITY, EPOS4_OP_CSV );
    //sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID6, C_PDO_NUMBER, C_AXIS6_POLARITY, EPOS4_OP_CSV );
	#else
	for(i=0;i<g_NUM_OF_SLAVES;i++)
	{
		sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID1+i, C_PDO_NUMBER, C_AXIS1_POLARITY, g_OP_MODE);
		
		/** @author DY
		*	@brief  tuning parameters
		*/
		//SdoWriten(C_DRIVE_BUSID1+i, 0x6065, 0, 4, 100000);	// Floowing Error window
		//SdoWriten(C_DRIVE_BUSID1+i, 0x607F, 0, 4, 10000);	// Profile Velocity
		SdoWriten(C_DRIVE_BUSID1+i, EPOS4_FOLLOWING_ERROR_WINDOW, 0, 4, 10000);	// Floowing Error window
		SdoWriten(C_DRIVE_BUSID1+i, EPOS4_MAX_MOTOR_SPEED, 0, 4, 12500);
		SdoWriten(C_DRIVE_BUSID1+i, EPOS4_MAX_PROFILE_VELOCITY, 0, 4, 10000);
		SdoWriten(C_DRIVE_BUSID1+i, EPOS4_PROFILE_VELOCITY, 0, 4, 10000);			// Profile Velocity
		SdoWriten(C_DRIVE_BUSID1+i, EPOS4_PROFILE_ACCELERATION, 0, 4, 6000);
		SdoWriten(C_DRIVE_BUSID1+i, EPOS4_PROFILE_DECELERATION, 0, 4, 6000);
	}
//	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID1, C_PDO_NUMBER, C_AXIS1_POLARITY, g_OP_MODE );
//	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID2, C_PDO_NUMBER, C_AXIS1_POLARITY, g_OP_MODE );
//	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID3, C_PDO_NUMBER, C_AXIS1_POLARITY, g_OP_MODE );
//	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID4, C_PDO_NUMBER, C_AXIS1_POLARITY, g_OP_MODE );
//	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID5, C_PDO_NUMBER, C_AXIS1_POLARITY, g_OP_MODE );

	#endif
	//Delay(10);
    sdkEtherCATMasterDoMapping();
    //Delay(10);
    for (i = 1; i <= g_NUM_OF_SLAVES; i++) {
    //for (i = 1; i <= g_NUM_OF_SLAVES; i++) {
	   sdkEtherCATSetupDC(i, C_EC_CYCLE_TIME, 0);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]
    }
    //Delay(10);
    //----------------------------------------------------------------
	// Start the EtherCAT
	//----------------------------------------------------------------
	sdkEtherCATMasterStart();
    //Delay(10);
	//----------------------------------------------------------------
	// Setup EtherCAT bus module (OP-MODE)
	//----------------------------------------------------------------
	#if g_ETHERCAT_CONFIG_INDIVIDUAL
	sdkEpos4_SetupECatBusModule(C_AXIS1, C_DRIVE_BUSID1, C_PDO_NUMBER, EPOS4_OP_CSV);
	//sdkEpos4_SetupECatBusModule(C_AXIS2, C_DRIVE_BUSID2, C_PDO_NUMBER, EPOS4_OP_CSV);
	//sdkEpos4_SetupECatBusModule(C_AXIS3, C_DRIVE_BUSID3, C_PDO_NUMBER, EPOS4_OP_CSV);
    //sdkEpos4_SetupECatBusModule(C_AXIS4, C_DRIVE_BUSID4, C_PDO_NUMBER, EPOS4_OP_CSV);
    //sdkEpos4_SetupECatBusModule(C_AXIS5, C_DRIVE_BUSID5, C_PDO_NUMBER, EPOS4_OP_CSV);
    //sdkEpos4_SetupECatBusModule(C_AXIS6, C_DRIVE_BUSID6, C_PDO_NUMBER, EPOS4_OP_CSV);
	#else
	for(i=0;i<g_NUM_OF_SLAVES;i++)
	{
		sdkEpos4_SetupECatBusModule(C_AXIS1+i, C_DRIVE_BUSID1+i, C_PDO_NUMBER, g_OP_MODE);
	}
	#endif
    //Delay(10);
	//----------------------------------------------------------------
	// Setup Virtual Amplifier following the OP-MODE
	//----------------------------------------------------------------
	#if g_ETHERCAT_CONFIG_INDIVIDUAL
	sdkEpos4_SetupECatVirtAmp(C_AXIS1, C_AXIS1_MAX_RPM, EPOS4_OP_CSV);
	//sdkEpos4_SetupECatVirtAmp(C_AXIS2, C_AXIS2_MAX_RPM, EPOS4_OP_CSV);
	//sdkEpos4_SetupECatVirtAmp(C_AXIS3, C_AXIS3_MAX_RPM, EPOS4_OP_CSV);
    //sdkEpos4_SetupECatVirtAmp(C_AXIS4, C_AXIS4_MAX_RPM, EPOS4_OP_CSV);
	//sdkEpos4_SetupECatVirtAmp(C_AXIS5, C_AXIS5_MAX_RPM, EPOS4_OP_CSV);
    //sdkEpos4_SetupECatVirtAmp(C_AXIS6, C_AXIS6_MAX_RPM, EPOS4_OP_CSV);
	#else
	for(i=0;i<g_NUM_OF_SLAVES;i++)
	{
		sdkEpos4_SetupECatVirtAmp(C_AXIS1+i, C_AXIS1_MAX_RPM, g_OP_MODE);
	}
	#endif
    //Delay(10);
	#if g_ETHERCAT_CONFIG_INDIVIDUAL
	//sdkEpos4_SetupECatVirtCntin(C_AXIS1, EPOS4_OP_CSV);
	//sdkEpos4_SetupECatVirtCntin(C_AXIS2, EPOS4_OP_CSV);
	//sdkEpos4_SetupECatVirtCntin(C_AXIS3, EPOS4_OP_CSV);
    //sdkEpos4_SetupECatVirtCntin(C_AXIS4, EPOS4_OP_CSV);
    //sdkEpos4_SetupECatVirtCntin(C_AXIS5, EPOS4_OP_CSV);
    //sdkEpos4_SetupECatVirtCntin(C_AXIS6, EPOS4_OP_CSV);
	#else
	for(i=0;i<g_NUM_OF_SLAVES;i++)
	{
		sdkEpos4_SetupECatVirtCntin(C_AXIS1+i, g_OP_MODE);
	}
	#endif
    //Delay(10);

	#if g_ETHERCAT_CONFIG_INDIVIDUAL
	#if g_NUM_OF_SLAVES >=1
	// All axis have in this example the same parameters
	sdkSetupAxisMovementParam(	C_AXIS1,
								C_AXIS1_VELRES,
								C_AXIS1_MAX_RPM,
								C_AXIS1_RAMPTYPE,
								C_AXIS1_RAMPMIN,
								C_AXIS1_JERKMIN
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		C_AXIS1,
								C_AXIS1_POSENCREV,
								C_AXIS1_POSENCQC,
								C_AXIS1_POSFACT_Z,
								C_AXIS1_POSFACT_N,
								C_AXIS1_FEEDREV,
								C_AXIS1_FEEDDIST
								);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	C_AXIS1,
									C_AXIS1_KPROP,
									C_AXIS1_KINT,
									C_AXIS1_KDER,
									C_AXIS1_KILIM,
									C_AXIS1_KILIMTIME,
									C_AXIS1_BANDWIDTH,
									C_AXIS1_FFVEL,
									C_AXIS1_KFFAC,
									C_AXIS1_KFFDEC
									);

	#elif g_NUM_OF_SLAVES >= 2
	// All axis have in this example the same parameters
	sdkSetupAxisMovementParam(	C_AXIS2,
								C_AXIS2_VELRES,
								C_AXIS2_MAX_RPM,
								C_AXIS2_RAMPTYPE,
								C_AXIS2_RAMPMIN,
								C_AXIS2_JERKMIN
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		C_AXIS2,
								C_AXIS2_POSENCREV,
								C_AXIS2_POSENCQC,
								C_AXIS2_POSFACT_Z,
								C_AXIS2_POSFACT_N,
								C_AXIS2_FEEDREV,
								C_AXIS2_FEEDDIST
								);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	C_AXIS2,
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

	#elif g_NUM_OF_SLAVES >=3
	// All axis have in this example the same parameters
	sdkSetupAxisMovementParam(	C_AXIS3,
								C_AXIS3_VELRES,
								C_AXIS3_MAX_RPM,
								C_AXIS3_RAMPTYPE,
								C_AXIS3_RAMPMIN,
								C_AXIS3_JERKMIN
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		C_AXIS3,
								C_AXIS3_POSENCREV,
								C_AXIS3_POSENCQC,
								C_AXIS3_POSFACT_Z,
								C_AXIS3_POSFACT_N,
								C_AXIS3_FEEDREV,
								C_AXIS3_FEEDDIST
								);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	C_AXIS3,
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
	#elif g_NUM_OF_SLAVES >=4
	// All axis have in this example the same parameters
	sdkSetupAxisMovementParam(	C_AXIS4,
								C_AXIS4_VELRES,
								C_AXIS4_MAX_RPM,
								C_AXIS4_RAMPTYPE,
								C_AXIS4_RAMPMIN,
								C_AXIS4_JERKMIN
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		C_AXIS4,
								C_AXIS4_POSENCREV,
								C_AXIS4_POSENCQC,
								C_AXIS4_POSFACT_Z,
								C_AXIS4_POSFACT_N,
								C_AXIS4_FEEDREV,
								C_AXIS4_FEEDDIST
								);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	C_AXIS4,
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
	#elif g_NUM_OF_SLAVES >=5
	// All axis have in this example the same parameters
	sdkSetupAxisMovementParam(	C_AXIS5,
								C_AXIS5_VELRES,
								C_AXIS5_MAX_RPM,
								C_AXIS5_RAMPTYPE,
								C_AXIS5_RAMPMIN,
								C_AXIS5_JERKMIN
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		C_AXIS5,
								C_AXIS5_POSENCREV,
								C_AXIS5_POSENCQC,
								C_AXIS5_POSFACT_Z,
								C_AXIS5_POSFACT_N,
								C_AXIS5_FEEDREV,
								C_AXIS5_FEEDDIST
								);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	C_AXIS5,
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
	#elif g_NUM_OF_SLAVES >=6
	// All axis have in this example the same parameters
	sdkSetupAxisMovementParam(	C_AXIS6,
								C_AXIS6_VELRES,
								C_AXIS6_MAX_RPM,
								C_AXIS6_RAMPTYPE,
								C_AXIS6_RAMPMIN,
								C_AXIS6_JERKMIN
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		C_AXIS6,
								C_AXIS6_POSENCREV,
								C_AXIS6_POSENCQC,
								C_AXIS6_POSFACT_Z,
								C_AXIS6_POSFACT_N,
								C_AXIS6_FEEDREV,
								C_AXIS6_FEEDDIST
								);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	C_AXIS6,
									C_AXIS6_KPROP,
									C_AXIS6_KINT,
									C_AXIS6_KDER,
									C_AXIS6_KILIM,
									C_AXIS6_KILIMTIME,
									C_AXIS6_BANDWIDTH,
									C_AXIS6_FFVEL,
									C_AXIS6_KFFAC,
									C_AXIS6_KFFDEC
									);

	#elif g_NUM_OF_SLAVES >=7
	// All axis have in this example the same parameters
	sdkSetupAxisMovementParam(	C_AXIS7,
								C_AXIS7_VELRES,
								C_AXIS7_MAX_RPM,
								C_AXIS7_RAMPTYPE,
								C_AXIS7_RAMPMIN,
								C_AXIS7_JERKMIN
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		C_AXIS7,
								C_AXIS7_POSENCREV,
								C_AXIS7_POSENCQC,
								C_AXIS7_POSFACT_Z,
								C_AXIS7_POSFACT_N,
								C_AXIS7_FEEDREV,
								C_AXIS7_FEEDDIST
								);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	C_AXIS7,
									C_AXIS7_KPROP,
									C_AXIS7_KINT,
									C_AXIS7_KDER,
									C_AXIS7_KILIM,
									C_AXIS7_KILIMTIME,
									C_AXIS7_BANDWIDTH,
									C_AXIS7_FFVEL,
									C_AXIS7_KFFAC,
									C_AXIS7_KFFDEC
									);
	#elif
	print("[ERROR] Check the number of slaves");
	#endif
	#else
	for(i=0;i<g_NUM_OF_SLAVES;i++)
	{
		// All axis have in this example the same parameters
		sdkSetupAxisMovementParam(	C_AXIS1+i,
									C_AXIS1_VELRES,
									C_AXIS1_MAX_RPM,
									C_AXIS1_RAMPTYPE,
									C_AXIS1_RAMPMIN,
									C_AXIS1_JERKMIN
									);
		// Definition of the user units
		sdkSetupAxisUserUnits(		C_AXIS1+i,
									C_AXIS1_POSENCREV,
									C_AXIS1_POSENCQC,
									C_AXIS1_POSFACT_Z,
									C_AXIS1_POSFACT_N,
									C_AXIS1_FEEDREV,
									C_AXIS1_FEEDDIST
									);
		// Position control setup
		sdkSetupPositionPIDControlExt( 	C_AXIS1+i,
										C_AXIS1_KPROP,
										C_AXIS1_KINT,
										C_AXIS1_KDER,
										C_AXIS1_KILIM,
										C_AXIS1_KILIMTIME,
										C_AXIS1_BANDWIDTH,
										C_AXIS1_FFVEL,
										C_AXIS1_KFFAC,
										C_AXIS1_KFFDEC
										);
	}

	#endif


	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------
	ErrorClear();

	for(i=0;i<g_NUM_OF_SLAVES;i++)
	{
		print("MPOS : ", Mapos(C_AXIS1+i));
	}

	return(0);
}


long EtherCAT_REINIT (long id, long signal, long event[], long data[]) // EtherCAT NMT 상태를 재부팅하여 Operational 상태로 만듦.
{
    long n_slaves, i,retval,res;
    print("REINIT");

    ECatMasterCommand(0x1000, 0);

	n_slaves = sdkEtherCATMasterInitialize();
    sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID1, C_PDO_NUMBER, C_AXIS1_POLARITY, EPOS4_OP_CSV );
	//sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID2, C_PDO_NUMBER, C_AXIS2_POLARITY, EPOS4_OP_CSP );
	//sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID3, C_PDO_NUMBER, C_AXIS3_POLARITY, EPOS4_OP_CSP );
    //sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID4, C_PDO_NUMBER, C_AXIS4_POLARITY, EPOS4_OP_CSP );
    //sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID5, C_PDO_NUMBER, C_AXIS5_POLARITY, EPOS4_OP_CSP );
    //sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID6, C_PDO_NUMBER, C_AXIS6_POLARITY, EPOS4_OP_CSP );

	for (i = 1; i <= 1; i++) {
	                          sdkEtherCATSetupDC(i, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]
                             }


    sdkEtherCATMasterStart();
    sdkEpos4_SetupECatBusModule(C_AXIS1, C_DRIVE_BUSID1, C_PDO_NUMBER, EPOS4_OP_CSV);
	//sdkEpos4_SetupECatBusModule(C_AXIS2, C_DRIVE_BUSID2, C_PDO_NUMBER, EPOS4_OP_CSP);
	//sdkEpos4_SetupECatBusModule(C_AXIS3, C_DRIVE_BUSID3, C_PDO_NUMBER, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatBusModule(C_AXIS4, C_DRIVE_BUSID4, C_PDO_NUMBER, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatBusModule(C_AXIS5, C_DRIVE_BUSID5, C_PDO_NUMBER, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatBusModule(C_AXIS6, C_DRIVE_BUSID6, C_PDO_NUMBER, EPOS4_OP_CSP);

	// setup virtual amplifier for csp mode
	sdkEpos4_SetupECatVirtAmp(C_AXIS1, C_AXIS1_MAX_RPM, EPOS4_OP_CSV);
	//sdkEpos4_SetupECatVirtAmp(C_AXIS2, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	//sdkEpos4_SetupECatVirtAmp(C_AXIS3, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatVirtAmp(C_AXIS4, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	//sdkEpos4_SetupECatVirtAmp(C_AXIS5, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatVirtAmp(C_AXIS6, C_AXIS_MAX_RPM, EPOS4_OP_CSP);
	//

	// setup irtual counter for csp mode
	sdkEpos4_SetupECatVirtCntin(C_AXIS1, EPOS4_OP_CSV);
	//sdkEpos4_SetupECatVirtCntin(C_AXIS2, EPOS4_OP_CSP);
	//sdkEpos4_SetupECatVirtCntin(C_AXIS3, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatVirtCntin(C_AXIS4, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatVirtCntin(C_AXIS5, EPOS4_OP_CSP);
    //sdkEpos4_SetupECatVirtCntin(C_AXIS6, EPOS4_OP_CSP);

	ErrorClear();
    AmpErrorClear(AXALL);

    //AxisControl(C_AXIS1,ON,C_AXIS2,ON,C_AXIS3,ON,C_AXIS4,ON,C_AXIS5,ON,C_AXIS6,ON);
	Sysvar[0x01220105] = 0;

 	//(SmTrans(Standing));
    return(0);

}


long EtherCAT_PREOP (long id, long signal, long event[], long data[])  // EtherCAT NMT State를 PRE operational 상태로 바꿈
{
	long i=0;
	// DY
	print("EtherCAT PREOP set...");

	ECatMasterCommand(0x1000,1);
	Delay(10);
	SdoWrite(1000001,0x6040,0,0x80);
	//SdoWrite(1000002,0x6040,0,0x80);
	//SdoWrite(1000003,0x6040,0,0x80);
	//SdoWrite(1000004,0x6040,0,0x80);
	//SdoWrite(1000005,0x6040,0,0x80);
	//SdoWrite(1000006,0x6040,0,0x80);
	Delay(10);

	for (i = 1; i <= g_NUM_OF_SLAVES; i++) {
	   sdkEtherCATSetupDC(i, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]
							 }
	//(SmTrans(Standing));
	return(0);
}

long EtherCAT_OP (long id, long signal, long event[], long data[]) // EtherCAT NMT State를 Operational 상태로 바꿈
{
	//DY
	print("EtherCAT OP set...");

	sdkEtherCATMasterStart();
	AmpErrorClear(AXALL);

	//return(SmTrans(Standing));
	return(0);
}

/*
long main()
{
	print("'EtherCAT_config.mc' is compiled.");
	return(0);
}
*/
