/**
*	@file		SDK_Kinematics_Setup.mc
*	@brief		Functions for the kinematics setup.
*	@example	DeltaRobot_EPOS4_ECAT_Basic_no_SM.mc
*	$Revision: 129 $
*
*/

#pragma once

#include <SysDef.mh>
#include "SDK_Kinematics_Setup.mh"

/**
*	@brief 		Setup a Delta multi-axis kinematics model
*	@details	This function sets up a Delta kinematic. The parameters of the mechanics must be known and are transferred to the function.
*				More information can be found in the ApossIDE Help.
* 	@param 		axis_0			Axis module number
* 	@param 		axis_1			Axis module number
* 	@param 		axis_2			Axis module number
* 	@param 		prms_A			Definition of length A \[mm*10\](80.0 mm => 800)
* 	@param 		prms_B			Definition of length B \[mm*10\](80.0 mm => 800)
* 	@param 		prms_C			Definition of length C \[mm*10\](80.0 mm => 800)
* 	@param 		prms_D			Definition of length D \[mm*10\](80.0 mm => 800)
* 	@param 		prms_minMotAng	Minimum motor angle [°]
* 	@param 		prms_maxMotAng	Maximum motor angle [°]
*	@return 	value:	Kinematics handle used to reference this group of axes. \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupDeltaKinematics(long axis_0, long axis_1, long axis_2, long prms_A, long prms_B, long prms_C, long prms_D, long prms_minMotAng, long prms_maxMotAng)
{
    long   axes[3], handle;
    double prms[6];

    axes[0] = axis_0;   // Define axis 0
    axes[1] = axis_1;   // Define axis 1
    axes[2] = axis_2;   // Define axis 2

    prms[0] = prms_A;   // Length A
    prms[1] = prms_B;   // Length B
    prms[2] = prms_C;   // Length C
    prms[3] = prms_D;   // Length D

    prms[4] = rad(prms_minMotAng);   // The arm must not move higher than "prms_minMotAng" (for safety).
    prms[5] = rad(prms_maxMotAng);   // The arm must not move lower than "prms_maxMotAng" (for safety).

    handle = KinematicsSetup("Delta", axes, prms); // Get the kinematics handler

    return(handle);
}
/**
*	@brief 		Setup a 2-dimensional cartesian multi-axis kinematics model
*	@details	This function sets up a 2-dimensional cartesian kinematic.
*				More information can be found in the ApossIDE Help.
* 	@param 		axis_0			X Axis
* 	@param 		axis_1			Y Axis
*	@return 	value:	Kinematics handle used to reference this group of axes. \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupCartesian2dKinematics(long axis_0, long axis_1)
{
    long   axes[2], handle;
    double prms[1];

    axes[0] = axis_0;   // Define X axis
    axes[1] = axis_1;   // Define Y axis

    prms[0] = 2;   		// Set 2-dimensional system

    handle = KinematicsSetup("Cartesian", axes, prms); // Get the kinematics handler

    return(handle);
}

/**
*	@brief 		Setup a 3-dimensional cartesian multi-axis kinematics model
*	@details	This function sets up a 3-dimensional cartesian kinematic.
*				More information can be found in the ApossIDE Help.
* 	@param 		axis_0			X Axis
* 	@param 		axis_1			Y Axis
* 	@param 		axis_2			Z Axis
*	@return 	value:	Kinematics handle used to reference this group of axes. \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupCartesian3dKinematics(long axis_0, long axis_1, long axis_2)
{
    long   axes[3], handle;
    double prms[1];

    axes[0] = axis_0;   // Define X axis
    axes[1] = axis_1;   // Define Y axis
    axes[2] = axis_2;   // Define Z axis

    prms[0] = 3;   		// Set 3-dimensional system

    handle = KinematicsSetup("Cartesian", axes, prms); // Get the kinematics handler

    return(handle);
}

/**
*	@brief 		Setup a HBot or CoreXY multi-axis kinematics model
*	@details	This function sets up a HBot or CoreXY kinematic.
*				More information can be found in the ApossIDE Help.
* 	@param 		axis_0			Axis module number
* 	@param 		axis_1			Axis module number
*	@return 	value:	Kinematics handle used to reference this group of axes. \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupHBotCoreXYKinematics(long axis_0, long axis_1)
{
    long   axes[2], handle;
    double prms[1];

    axes[0] = axis_0;   // Define axis 0
    axes[1] = axis_1;   // Define axis 1
	prms[0] = 0;       	// Just to prevent a "Referenced but never assigned" warning.

    handle = KinematicsSetup("Hbot", axes, prms); // Get the kinematics handler

    return(handle);
}

/**
*	@brief 		Setup a Scara 2d multi-axis kinematics model
*	@details	This function sets up a Scara 2d kinematic. The parameters of the mechanics must be known and are transferred to the function.
*				More information can be found in the ApossIDE Help.
* 	@param 		axis_0			Axis module number
* 	@param 		axis_1			Axis module number
* 	@param 		prms_A			The length of the primary arm, in Machine Coordinate System units
* 	@param 		prms_B			The length of the secondary arm, in Machine Coordinate System units
* 	@param 		prms_orient		The orientation of the arms: 1 - Left arm oientation, -1 - Right arm orientation
*	@return 	value:	Kinematics handle used to reference this group of axes. \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupScara2dKinematics(long axis_0, long axis_1, long prms_A, long prms_B, long prms_orient)
{
    long   axes[2], handle;
    double prms[4];

    axes[0] = axis_0;   // Define axis 0
    axes[1] = axis_1;   // Define axis 1

    prms[0] = prms_A;   	// Length A
    prms[1] = prms_B;   	// Length B
    prms[2] = prms_orient;  // Orientation of the System "Left" / "Right"
    prms[3] = 2;   			// Set 2-dimensional system

    handle = KinematicsSetup("Scara", axes, prms); // Get the kinematics handler

    return(handle);
}

/**
*	@brief 		Setup a Scara 3d multi-axis kinematics model
*	@details	This function sets up a Scara 3d kinematic. The parameters of the mechanics must be known and are transferred to the function.
*				More information can be found in the ApossIDE Help.
* 	@param 		axis_0			Axis module number
* 	@param 		axis_1			Axis module number
* 	@param 		axis_2			Axis module number
* 	@param 		prms_A			The length of the primary arm, in Machine Coordinate System units
* 	@param 		prms_B			The length of the secondary arm, in Machine Coordinate System units
* 	@param 		prms_orient		The orientation of the arms: 1 - Left arm oientation, -1 - Right arm orientation
*	@return 	value:	Kinematics handle used to reference this group of axes. \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupScara3dKinematics(long axis_0, long axis_1, long axis_2, long prms_A, long prms_B, long prms_orient)
{
    long   axes[3], handle;
    double prms[4];

    axes[0] = axis_0;   // Define axis 0
    axes[1] = axis_1;   // Define axis 1
    axes[2] = axis_2;   // Define axis 2

    prms[0] = prms_A;   	// Length A
    prms[1] = prms_B;   	// Length B
    prms[2] = prms_orient;  // Orientation of the System "Left" / "Right"
    prms[3] = 3;   			// Set 3-dimensional system

    handle = KinematicsSetup("Scara", axes, prms); // Get the kinematics handler

    return(handle);
}

/**
*	@brief 		Setup a Dual Scara 2d multi-axis kinematics model
*	@details	This function sets up a Dual Scara 2d kinematic. The parameters of the mechanics must be known and are transferred to the function.
*				More information can be found in the ApossIDE Help.
* 	@param 		axis_0			Axis module number
* 	@param 		axis_1			Axis module number
* 	@param 		prms_A			The length of the left primary arm, in Machine Coordinate System units
* 	@param 		prms_B			The length of the left secondary arm, in Machine Coordinate System units
* 	@param 		prms_C			The length of the right primary arm, in Machine Coordinate System units
* 	@param 		prms_D			The length of the right secondary arm, in Machine Coordinate System units
* 	@param 		prms_distance	The separation distance between axis 1 and axis 2
*	@return 	value:	Kinematics handle used to reference this group of axes. \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupDualScara2dKinematics(long axis_0, long axis_1, long prms_A, long prms_B, long prms_C, long prms_D, long prms_distance)
{
    long   axes[2], handle;
    double prms[6];

    axes[0] = axis_0;   // Define axis 0
    axes[1] = axis_1;   // Define axis 1

    prms[0] = prms_A;   	// Length A
    prms[1] = prms_B;   	// Length B
    prms[2] = prms_C;   	// Length C
    prms[3] = prms_D;   	// Length D
    prms[4] = prms_distance;// Separation distance
    prms[5] = 2;   			// Set 2-dimensional system

    handle = KinematicsSetup("DualScara", axes, prms); // Get the kinematics handler

    return(handle);
}

/**
*	@brief 		Setup a Dual Scara 3d multi-axis kinematics model
*	@details	This function sets up a Dual Scara 3d kinematic. The parameters of the mechanics must be known and are transferred to the function.
*				More information can be found in the ApossIDE Help.
* 	@param 		axis_0			Axis module number
* 	@param 		axis_1			Axis module number
* 	@param 		axis_2			Axis module number
* 	@param 		prms_A			The length of the left primary arm, in Machine Coordinate System units
* 	@param 		prms_B			The length of the left secondary arm, in Machine Coordinate System units
* 	@param 		prms_C			The length of the right primary arm, in Machine Coordinate System units
* 	@param 		prms_D			The length of the right secondary arm, in Machine Coordinate System units
* 	@param 		prms_distance	The separation distance between axis 1 and axis 2
*	@return 	value:	Kinematics handle used to reference this group of axes. \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupDualScara3dKinematics(long axis_0, long axis_1, long axis_2, long prms_A, long prms_B, long prms_C, long prms_D, long prms_distance)
{
    long   axes[3], handle;
    double prms[6];

    axes[0] = axis_0;   // Define axis 0
    axes[1] = axis_1;   // Define axis 1
    axes[2] = axis_2;   // Define axis 2

    prms[0] = prms_A;   	// Length A
    prms[1] = prms_B;   	// Length B
    prms[2] = prms_C;   	// Length C
    prms[3] = prms_D;   	// Length D
    prms[4] = prms_distance;// Separation distance
    prms[5] = 3;   			// Set 3-dimensional system

    handle = KinematicsSetup("DualScara", axes, prms); // Get the kinematics handler

    return(handle);
}

/**
*	@brief 		Setup the working coordinates for a kinematics System
*	@details	Set up the Work-to-Machine transformation.  This is the transformation
*				from Work (i.e. paper) coordinates to Machine coordinates.  There are three parts:
*
*				1. Scaling:  For example. The Work coordinate system uses mm's and the Machine
*				coordinate system uses micrometers. So one Work unit (i.e. 1 mm) will be 1000 Machine units
*				(i.e. 1000 micrometers).
*
*				2. Rotation:  We would like to define the orientation. For example. The Work coordinate
*				system must be rotated CLOCKWISE by 90 degrees (i.e. -90 degrees counterclockwise) with
*				respect to the Machine coordinate system.
*
*				3. Translation:  The origin position of the coordinate system can be set in which there is a
*				shift in relation to the machine coordinate system.
* 	@param 		workToMachine		Transform variable of the coordinate system
* 	@param 		originTranslate_X	Translational movement of the coordinate system -> X [in mm depending on the scaling]
* 	@param 		originTranslate_Y	Translational movement of the coordinate system -> Y [in mm depending on the scaling]
* 	@param 		originTranslate_Z	Translational movement of the coordinate system -> Z [in mm depending on the scaling]
* 	@param 		rotation_XY			Rotation around the XY plane [°]
* 	@param 		scale				Scaling of the system [ ]
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*/
long sdkSetupWorkCoordKinematics(transform workToMachine, double originTranslate_X, double originTranslate_Y, double originTranslate_Z, double rotation_XY,double scale)
{
	TransIdent(workToMachine);
    TransScale(workToMachine, scale);
    TransRotateXY(workToMachine,rotation_XY*PI/180.0);
   	TransTranslate(workToMachine, originTranslate_X, originTranslate_Y, originTranslate_Z);

    print("Working coordinates are changed");
    return(1);
}











