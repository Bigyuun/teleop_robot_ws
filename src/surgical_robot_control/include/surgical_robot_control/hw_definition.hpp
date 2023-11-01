/**
 * @file hw_definition.hpp
 * @author daeyun (bigyun9375@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef HW_DEFINITION_HPP
#define HW_DEFINITION_HPP
//===============================
// Motor Parameters
//===============================
#define NUM_OF_MOTORS       10
#define GEAR_RATIO_44       44
#define GEAR_RATIO_3_9      3.9
#define ENCODER_CHANNEL     4
#define ENCODER_RESOLUTION  1024
#define INC_PER_ROT_44      GEAR_RATIO_44 * ENCODER_CHANNEL * ENCODER_RESOLUTION
#define INC_PER_ROT_3_9     GEAR_RATIO_3_9 * ENCODER_CHANNEL * ENCODER_RESOLUTION
#define DIRECTION_COUPLER   -1      // if not, use 1         

/** Motor control mode **/
/**
 * @brief It means that the ETA(Estimation Time Arrive) is same for all motors (arrive at same time)
 *        If not use it, just each motors move same velocity (not arrive at same time)
*/
#define MOTOR_CONTROL_SAME_DURATION 1
#define PERCENT_100 100

//===============================
// Continuum Manipulator Paremeters
//===============================
#define DOF               5     // Degree of freedom
#define NUM_OF_JOINT      5     // ea
#define SEGMENT_ARC       6.92  // mm
#define SEGMENT_DIAMETER  3.0   // mm
#define WIRE_DISTANCE     1.05  // mm
#define SLOT_LENGTH       1.05  // mm
#define SLOT_WIDTH        0.3   // mm

#define MAX_BENDING_DEGREE   90.0 // degree
#define MAX_FORCEPS_RAGNE_DEGREE 30.0  // mm
#define MAX_FORCEPS_RAGNE_MM 2.0  // mm (nor int)

#define JOINT_INTERVAL    3   // mm

//===============================
// Loadcell Parameters
//===============================
#define LOADCELL_THRESHOLD  1000.0


typedef struct {
  float gear_ratio = 44;
  int encoder_channel = 3;
  int encoder_resolution = 1024;
  int inc_per_rot = gear_ratio * encoder_channel * encoder_resolution;
} DCX22_G44;

typedef struct {
  float gear_ratio = 3.9;
  int encoder_channel = 3;
  int encoder_resolution = 1024;
  int inc_per_rot = gear_ratio * encoder_channel * encoder_resolution;
} DCX22_G3_9;

#endif