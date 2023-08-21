/**
*	@file		SDK_Amplifier_MiniMACS6_DS402_Slave.mc
*	@brief		This file provides all the function used to setup a MiniMACS6 DS402 slave
*	@details	The MiniMACS DS402 Slave can be operated in the different modes csp, csv and cst
*				The amplifiers can be controlled via CAN bus.
*				The motor, encoder and controller parameters are set directly on the MiniMACS6 DS402 Application.
*	$Revision: 225 $
*	@example 	CAN_4Ax_MiniMACS6_csp.mc
*	@example 	CAN_4Ax_MiniMACS6_csv.mc
*	@example 	CAN_4Ax_MiniMACS6_cst.mc
*/
#pragma once

#include <SysDef.mh>
#include "SDK_Amplifier_MiniMACS6_DS402_Slave.mh"

/**
*	@brief 		Setup the Can bus module for an MiniMACS6 DS402 slave
*	@details	This function sets up the Can bus module for an MiniMACS6 DS402 slave. It can be defined with which operation mode is used.
*				Currently there are no differences in the different operation modes regarding the Bus Module setup.
* 	@param 		axis		Axis module number
* 	@param 		busId		Bus ID of the connected slave
* 	@param 		pdoNumber 	Used PDO number
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 0x08: Cyclic synchronous position (csp) mode \n
*								@b 0x09: Cyclic synchronous velocity (csv) mode n
*								@b 0x0A: Cyclic synchronous torque (cst) mode
*	@note		The value passed from "BUSMOD_RXMAP_POVALUE3" is the current torque. The value is given in per
*				thousand of “Motor rated torque”.
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/

long sdkMiniMACS6_SetupCanBusModule(long axis, long busId, long pdoNumber, long operationMode)
{
	long busmod, signedFlag;
	signedFlag = 0x10000000;
    busmod = axis;

    print("sdkMiniMACS6_SetupCanBusModule: ", busmod);

    BUSMOD_PARAM(busmod, BUSMOD_MODE) =  BUSMOD_MODE_DEACTIVATE;			// Deactivate (objects will be deleted)
    BUSMOD_PARAM(busmod, BUSMOD_BUSTYPE) =  BUSMOD_BUSTYPE_CAN;          	// Can bus


    BUSMOD_PARAM(busmod, BUSMOD_BUSNO) =  busId / 100000;       			// Bus number (0 = master, 1 = slave CAN bus)
    BUSMOD_PARAM(busmod, BUSMOD_ID)   =  busId % 1000;        				// CAN Id for this bus module
    BUSMOD_PARAM(busmod, BUSMOD_SYNC) =  1;                     			// Sync active
    BUSMOD_PARAM(busmod, BUSMOD_GUARDTIME) =  0;               				// no guarding
	BUSMOD_PARAM(busmod, BUSMOD_MODE) =  BUSMOD_MODE_ACTIVATE;				// Create bus module

    BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT1) =  VIRTAMP_PROCESS_SRCINDEX(axis,PO_VIRTAMP_CMDWORD);   	// Select the input value sources object (send to bus): CMD Word

    if (operationMode == MINIMACS6_OP_CSP)	// cycle synchronous position (csp) mode
    {
    	BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT2) =  VIRTAMP_PROCESS_SRCINDEX(axis,PO_VIRTAMP_REFPOS);	// Select the input value sources object (send to bus): position setpoint
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode == MINIMACS6_OP_CSV)	// cycle synchronous velocity (csv) mode
    {
    	BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT2) =  VIRTAMP_PROCESS_SRCINDEX(axis,PO_VIRTAMP_REFVEL);	// Select the input value sources object (send to bus): velocity setvel
    	print("...: cycle synchronous velocity (csv) mode");
    }
    else if (operationMode == MINIMACS6_OP_CST)	// cycle synchronous torque (cst) mode
    {
    	BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT2) =  AXE_PROCESS_SRCINDEX(axis,REG_USERREFCUR);	// Select the input value sources object (send to bus): target torque
    	print("...: cycle synchronous torque (cst) mode");
    }
    else// Error
    {
    	print("...: not supported operation mode");
    	return(-1);
    }

    BUSMOD_PARAM(busmod, BUSMOD_TXMAP_INPUT1) =  pdoNumber*0x01000000 + 2*0x00010000 + 0;   // pdo ; length in bytes; bytes offset  control word

    if (operationMode != MINIMACS6_OP_CST)
    	BUSMOD_PARAM(busmod, BUSMOD_TXMAP_INPUT2) =  pdoNumber*0x01000000 + 4*0x00010000 + 2;   // pdo ; length in bytes; bytes offset  target position/ velocity
    else // cycle synchronous torque (cst) mode
    	BUSMOD_PARAM(busmod, BUSMOD_TXMAP_INPUT2) =  pdoNumber*0x01000000 + 2*0x00010000 + 2;   // pdo ; length in bytes; bytes offset  target torque

    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE1) =  pdoNumber*0x01000000 + 2*0x00010000 + 0; // pdo ; length in bytes; bytes offset   status word
    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE2) =  pdoNumber*0x01000000 + 4*0x00010000 + 2; // pdo ; length in bytes; bytes offset   position actual value
    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE3) =  signedFlag + pdoNumber*0x01000000 + 2*0x00010000 + 6; // pdo ; length in bytes; bytes offset   torque actual value

    BUSMOD_PARAM(busmod, BUSMOD_MODE) = BUSMOD_MODE_ACTIVATE_NOSTOP;                    	// Start bus module

    return(1);
}

/**
*	@brief 		Setup the virtual amplifier for an MiniMACS6 DS402 slave with Can bus
*	@details	This function sets up the virtual amplifier with Can bus for an MiniMACS6 DS402 slave. It can be defined with which operation mode is used.
* 	@param 		axis			Axis module number
*	@param		maxRpm			Definition of the maximum motor speed \[rpm\]
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 0x08: Cyclic synchronous position (csp) mode \n
*								@b 0x09: Cyclic synchronous velocity (csv) mode n
*								@b 0x0A: Cyclic synchronous torque (cst) mode
*	@note		The value passed from "PO_BUSMOD_VALUE3" to "VIRTAMP_PISRC_CURRENT" is the current torque. The value is given in per
*				thousand of “Motor rated torque”.
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/

long sdkMiniMACS6_SetupCanVirtAmp(long axis, long maxRpm, long operationMode)
{
	print("sdkMiniMACS6_SetupCanVirtAmp: ",axis );

	// virtual amplifiers have a fixed connection to axes number, axes 0 uses amplifier 0.
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_CMDWORD)    = AXE_PROCESS_SRCINDEX(axis,REG_CNTRLWORD);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_REFPOS)     = AXE_PROCESS_SRCINDEX(axis,REG_COMPOS);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_REFVEL)     = AXE_PROCESS_SRCINDEX(axis,REG_REFERENCE);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_REFACC)     = AXE_PROCESS_SRCINDEX(axis,PID_FFACCPART);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_STATUS)     = BUSMOD_PROCESS_SRCINDEX(axis,PO_BUSMOD_VALUE1);	//statusword
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_CURRENT)    = BUSMOD_PROCESS_SRCINDEX(axis, PO_BUSMOD_VALUE3); //torque

	VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWROFF)    = 0x06;
	VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWRONDIS)  = 0x06;
	VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWRONENP)  = 0x0F;
	VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWRONENN)  = 0x0F;
	VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_QUICKSTOP) = 0x02;
	VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_RESET)     = 0x80;

    VIRTAMP_PARAM(axis,VIRTAMP_STOPDELAY)        = 0x00;
   	VIRTAMP_PARAM(axis,VIRTAMP_ERROR_BITMASK)    = 0x08;
    VIRTAMP_PARAM(axis,VIRTAMP_ERROR_POLARITY)   = 1;

    // Ready bit: Ready to switch on, Switched on, Operation enabled, Voltage enabled
    VIRTAMP_PARAM(axis,VIRTAMP_READY_BITMASK)   = 0x17;
    VIRTAMP_PARAM(axis,VIRTAMP_READY_POLARITY)  = 1;

    if (operationMode == MINIMACS6_OP_CSP)	// cycle synchronous position (csp) mode
    {
    	VIRTAMP_PARAM(axis,VIRTAMP_REF100PERC)       = 0;  // not used in csp
    	VIRTAMP_PARAM(axis,VIRTAMP_REFLIMIT)         = 0;  // not used in csp
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode == MINIMACS6_OP_CSV)	// cycle synchronous velocity (csv) mode
    {
    	VIRTAMP_PARAM(axis,VIRTAMP_REF100PERC)       = maxRpm;
    	VIRTAMP_PARAM(axis,VIRTAMP_REFLIMIT)         = 2* maxRpm;
    	print("...: cycle synchronous velocity (csv) mode");
    }
    else if (operationMode == MINIMACS6_OP_CST)	// cycle synchronous torque (cst) mode
    {
    	VIRTAMP_PARAM(axis,VIRTAMP_REF100PERC)       = 0;  // not used in cst
    	VIRTAMP_PARAM(axis,VIRTAMP_REFLIMIT)         = 0;  // not used in cst
    	print("...: cycle synchronous torque (cst) mode");
    }
    else	// Error
    {
    	print("...: not supported operation mode");
    	return(-1);
    }

    VIRTAMP_PARAM(axis,VIRTAMP_MODE) = VIRTAMP_MODE_ENABLE; 	// has to be the last one because it activates all

    return(1);
}

/**
*	@brief 		Setup the virtual counter input for an MiniMACS6 DS402 slave with Can bus
*	@details	This function sets up the virtual counter input for an MiniMACS6 DS402 slave. It can be defined with which operation mode is used.
* 	@param 		axis			Axis module number
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 0x08: Cyclic synchronous position (csp) mode \n
*								@b 0x09: Cyclic synchronous velocity (csv) mode n
*								@b 0x0A: Cyclic synchronous torque (cst) mode
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/

long sdkMiniMACS6_SetupCanVirtCntin(long axis, long operationMode)
{
    print("sdkMiniMACS6_SetupCanVirtCntin: ",axis);

    VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_PISRC_COUNTER) = BUSMOD_PROCESS_SRCINDEX(axis, PO_BUSMOD_VALUE2);

    if (operationMode == MINIMACS6_OP_CSP)	// cycle synchronous position (csp) mode
    {
    	VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE_DIRECT;   // Source is absolute and is taken as it is
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode == MINIMACS6_OP_CSV)	// cycle synchronous velocity (csv) mode
    {
    	VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE;   		// Source is a position value and difference to last value is added
    	print("...: cycle synchronous velocity (csv) mode");
    }
    else if (operationMode == MINIMACS6_OP_CSP)	// cycle synchronous torque (cst) mode
    {
    	VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE_DIRECT;   // Source is absolute and is taken as it is
    	print("...: cycle synchronous torque (cst) mode");
    }
    else											// Error
    {
    	print("...: not supported operation mode");
    	return(-1);
    }

    return(1);
}

/**
*	@brief 		Setup the Sdo parameter for an MiniMACS6 DS402 slave
*	@details	This function sets up the Sdo parameter for an MiniMACS6 DS402 slave. It can be defined with which operation mode is used.
*				Each axis is controlled via its own PDO. A maximum of 4 axes can be controlled.
* 	@param 		busId			Bus ID of the connected slave
* 	@param 		pdoNumber 		Used PDO number
* 	@param 		slaveAxisNo		Axis module number of the slave (0-5)
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 0x08: Cyclic synchronous position (csp) mode \n
*								@b 0x09: Cyclic synchronous velocity (csv) mode n
*								@b 0x0A: Cyclic synchronous torque (cst) mode
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/
long sdkMiniMACS6_SetupCanSdoParam(long busId, long pdonumber, long slaveAxisNo, long operationMode)
///////////////////////////////////////////////////////////////////////////////
{
    long nodeid, last_value;
    long axOffset = 0x8000000;
    nodeid = busId % 100000;

	print("sdkMiniMACS6_SetupCanSdoParam: " ,nodeid);

	// Set ms loop sync source to CAN PDO, This feature will sync the controllers loop to a bus signal.
	SdoWrite(busId, 0x2203, 0x13, 1);

    // reset Transmittype
    SdoWrite(busId, MINIMACS6_TRANSMIT_PDO_1_PARAMETER+ pdonumber-1, 2, 255);    // TxPDO1 Transmittype

    // the pdos have to be disabled for configuring
    last_value = (SdoRead(busId, MINIMACS6_TRANSMIT_PDO_1_PARAMETER+ pdonumber-1, 1));
    SdoWrite(busId, MINIMACS6_TRANSMIT_PDO_1_PARAMETER+ pdonumber-1, 1, (last_value | 0x80000000)); // disable pdo x

    //the pdos have to be disabled for configuring
    last_value = (SdoRead(busId, MINIMACS6_RECEIVE_PDO_1_PARAMETER+pdonumber-1, 1));
    SdoWrite(busId, MINIMACS6_RECEIVE_PDO_1_PARAMETER+ pdonumber-1, 1, (last_value | 0x80000000)); // disable pdo 1

    // Now we setup the correct PDO transmission for Tx and Rx
    SdoWrite(busId, (MINIMACS6_TRANSMIT_PDO_1_PARAMETER + pdonumber-1), 2,  1);			// TxPDO Transmittype SYNC
    SdoWrite(busId, (MINIMACS6_RECEIVE_PDO_1_PARAMETER + pdonumber-1), 2,  1);			// RxPDO Transmittype SYNC

    // config RX PDO
    SdoWrite(busId, (MINIMACS6_RECEIVE_PDO_1_MAPPING + pdonumber-1), 0x00, 0);			// disable
    SdoWrite(busId, (MINIMACS6_RECEIVE_PDO_1_MAPPING + pdonumber-1), 0x01, 0x60400010+axOffset*slaveAxisNo);	// controlword

    if (operationMode == MINIMACS6_OP_CSP) {
    	SdoWrite(busId, (MINIMACS6_RECEIVE_PDO_1_MAPPING + pdonumber-1), 2, 0x607A0020+axOffset*slaveAxisNo);	// download pdo 0x1600 entry:	position set point
    	SdoWrite(busId, MINIMACS6_MODES_OF_OPERATION, 0, MINIMACS6_OP_CSP);										// mode of operation CSP
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode == MINIMACS6_OP_CSV ) {
		SdoWrite(busId, (MINIMACS6_RECEIVE_PDO_1_MAPPING + pdonumber-1), 2, 0x60FF0020+axOffset*slaveAxisNo);  	// download pdo 0x1600 entry: target velocity 32bit
		SdoWrite(busId, MINIMACS6_MODES_OF_OPERATION, 0, MINIMACS6_OP_CSV);										// mode of operation CSV
    	print("...: cycle synchronous velocity (csv) mode");
    }
   else if (operationMode == MINIMACS6_OP_CST) {
		SdoWrite(busId, (MINIMACS6_RECEIVE_PDO_1_MAPPING + pdonumber-1), 2, 0x60710010+axOffset*slaveAxisNo);  	// download pdo 0x1600 entry: Target torque 16bit
		SdoWrite(busId, MINIMACS6_MODES_OF_OPERATION, 0, MINIMACS6_OP_CST);										// mode of operation CST
    	print("...: cycle synchronous torque (cst) mode");
    }
    else {	// Error
		print("...: not supported operation mode");
    	return(-1);
    }

    SdoWrite(busId, (MINIMACS6_RECEIVE_PDO_1_MAPPING + pdonumber - 1), 0x00, 2);     		// enable

    // config TX PDO
    SdoWrite(busId, (MINIMACS6_TRANSMIT_PDO_1_MAPPING + pdonumber - 1), 0x00, 0);     		// disable
    SdoWrite(busId, (MINIMACS6_TRANSMIT_PDO_1_MAPPING + pdonumber - 1), 0x01, 0x60410010+axOffset*slaveAxisNo);	// statusword
    SdoWrite(busId, (MINIMACS6_TRANSMIT_PDO_1_MAPPING + pdonumber - 1), 0x02, 0x60640020+axOffset*slaveAxisNo);	// actpos
    SdoWrite(busId, (MINIMACS6_TRANSMIT_PDO_1_MAPPING + pdonumber - 1), 0x03, 0x60770010+axOffset*slaveAxisNo);	// Torque actual
    SdoWrite(busId, (MINIMACS6_TRANSMIT_PDO_1_MAPPING + pdonumber - 1), 0x00, 3 );			// enable

    // fill in the wanted pdo
    SdoWrite(busId, (MINIMACS6_TRANSMIT_PDO_1_PARAMETER + pdonumber - 1), 1, (0x80000180 + (pdonumber - 1)*0x100 + nodeid) );// fill in pdo
    SdoWrite(busId, (MINIMACS6_RECEIVE_PDO_1_PARAMETER + pdonumber - 1), 1, (0x80000200 + (pdonumber - 1)*0x100 + nodeid) );// fill in pdo

    // enable
    SdoWrite(busId, (MINIMACS6_TRANSMIT_PDO_1_PARAMETER + pdonumber - 1), 1, (0x00000180 + (pdonumber - 1)*0x100 + nodeid) );// enable pdo
    SdoWrite(busId, (MINIMACS6_RECEIVE_PDO_1_PARAMETER + pdonumber - 1), 1, (0x00000200 + (pdonumber - 1)*0x100 + nodeid) );// enable pdo

	return(1);
}

/**
*	@brief 		State machine function for performing a homing on an MiniMACS6 DS402 slave
*	@details	This function sets the MiniMACS6 DS402 slave in homing mode and starts the homing defined on the MiniMACS6 DS402 slave. Before calling this function,
*				all homing parameters must be configured on the MiniMACS6 DS402 slave. The function is not blocking.
*				The return parameter must be checked.
* 	@param 		axis			Axis module number
* 	@param 		busId			Bus ID of the connected slave
* 	@param 		operationMode 	Operation mod which should be set after homing.
* 	@param 		homingState		Call-by-reference variable for the iteration of the different homing states.
*								Must be initialized with 0.
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/

long sdkMiniMACS6_AxisHomingStart(long axis, long busId, long operationMode, long &homingState)
///////////////////////////////////////////////////////////////////////////////
{
    long time, displayMode;
	long axOffset = 0x800;
    time= Time()%1000;

   	switch (homingState) {
		case 0:
			print("sdkMiniMACS6_AxisHomingStart: " ,axis);
   			homingState = 1;
   			break;
		case 1:
			// x6060 Change operation mode to ”6: Homing mode.”
			SdoWriten( busId, MINIMACS6_MODES_OF_OPERATION+axOffset*axis, 0, 1, MINIMACS6_OP_HMM);
			print("...: DS402 Homing AxisNo: ",axis," - Set mode of OP: 0x06");

			Delay(1);
			print("...: DS402 Homing AxisNo: ",axis," - Enable Drive");
		   	AxisControl(axis, ON);

			homingState = 2;

		case 2:
			displayMode = SdoRead(busId, MINIMACS6_MODES_OF_OPERATION_DISPLAY+axOffset*axis, 0);
			if(displayMode== MINIMACS6_OP_HMM)
			{
				print("...: DS402 Homing AxisNo: ",axis," - Display mode of OP: ",displayMode);
				homingState = 3;
			}
			else if(time==0)
			{
				printf("...: DS402 Homing AxisNo: %ld - Waiting for OP mode display:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
				// This section  must be called in a 1 ms loop because of the print function.
				Delay(1);
			}
		   	break;

		case 3:
			// Bit 10 Target reached - Homing procedure is interrupted or not started
			if(VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS).i[10]==1)
			{
				printf("...: DS402 Homing AxisNo: %ld - Homing start signal Status:   %lx\n",axis,VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
				VIRTAMP_PARAM(axis, VIRTAMP_CNTRLW_PWRONENP) = 0x1F;
				VIRTAMP_PARAM(axis, VIRTAMP_CNTRLW_PWRONENN) = 0x1F;
				homingState = 4;
			}
			// Bit 13 Homingerror - Homing error occurred
			else if(VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS).i[13]==1)
			{
				printf("...: DS402 Homing AxisNo: %ld - Homing Error - Status:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
				return(-1);
			}
			else if(time==0)
			{
				printf("...: DS402 Homing AxisNo: %ld - Waiting for ready bit:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
				// This section  must be called in a 1 ms loop because of the print function.
				Delay(1);
			}
			break;

		case 4:
			   	// Homing procedure is completed successfully
				if (VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS).i[12]==1)
				{
					printf("...: DS402 Homing AxisNo: %ld - Homing done - Status:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));

					VIRTAMP_PARAM(axis, VIRTAMP_CNTRLW_PWRONENP) = 0x0F;
					VIRTAMP_PARAM(axis, VIRTAMP_CNTRLW_PWRONENN) = 0x0F;

					print("...: DS402 Homing AxisNo: ",axis," - Disable Drive");
		   			AxisControl(axis, OFF);

					SdoWriten(busId, MINIMACS6_MODES_OF_OPERATION+axOffset*axis, 0, 1, operationMode );        // 0x6060 Change operation mode to ”8: CSP mode.”

					homingState = 5;
					break;
			   	}
				// Bit 13 Homingerror - Homing error occurred
				else if(VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS).i[13]==1)
				{
					printf("...: DS402 Homing AxisNo: %ld - Homing Error - Status:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
					return(-1);
				}
				// Homing procedure is in progress
			   	else if(time==0)
			   	{
			   		printf("...: DS402 Homing AxisNo: %ld - Homing in progress - Status:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
			   		// This section  must be called in a 1 ms loop because of the print function.
			   		Delay(1);
			   	}
			   	break;

		case 5:
			displayMode = SdoRead(busId, MINIMACS6_MODES_OF_OPERATION_DISPLAY+axOffset*axis, 0);
			if(displayMode== operationMode)
			{
				print("...: DS402 Homing AxisNo: ",axis," - Display mode of OP: ",displayMode);
				homingState = 6;
			}
			else if(time==0)
			{
				printf("...: DS402 Homing AxisNo: %ld - Waiting for OP mode display:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
				// This section  must be called in a 1 ms loop because of the print function.
				Delay(1);
			}
		case 6:
 		   	print("...: DS402 Homing AxisNo: ",axis," - Finished");
		   	homingState = 7;
		   	return(1);
		case 7:
			return(1);
		default :
			print("...: DS402 Homing AxisNo: ",axis," - Incorrect hominState init value: ", homingState);
			return(-1);
	}

	return(0);
}
