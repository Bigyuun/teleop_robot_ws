/**
 * @file hw_spec.hpp
 * @author daeyun (bigyun9375@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#define GEAR_RATIO_44       44
#define GEAR_RATIO_3_9      3.9
#define ENCODER_CHANNEL     3
#define ENCODER_RESOLUTION  1024
#define INC_PER_ROT_44      GEAR_RATIO_44 * ENCODER_CHANNEL * ENCODER_RESOLUTION
#define INC_PER_ROT_3_9     GEAR_RATIO_3_9 * ENCODER_CHANNEL * ENCODER_RESOLUTION


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
