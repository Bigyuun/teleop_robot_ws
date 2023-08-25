#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <math.h>

// DY
#define NUM_OF_JOINT      4     // ea
#define SEGMENT_ARC       6.92  // mm
#define SEGMENT_DIAMETER  3     // mm
#define WIRE_DISTANCE     1.05  // mm

#define MAX_BENDING_DEGREE   60 // degree
#define MAX_FORCEPS_RAGNE_DEGREE 30  // mm
#define MAX_FORCEPS_RAGNE_MM 3  // mm

struct structure {
  int num_joint;
  float pAngle;
  float tAngle;
  float arc;
  float diameter;
  float disWire;
};

class SurgicalTool
{
public:
  SurgicalTool();
  ~SurgicalTool();

  struct structure surgicaltool_;
  // Make wire
  void init_surgicaltool(
    int num_joint,
    float arc,
    float diameter,
    float disWire
  );

  float max_bending_deg_ = MAX_BENDING_DEGREE;
  float max_forceps_deg_ = MAX_FORCEPS_RAGNE_DEGREE;

	float wrLengthWest_, wrLengthEast_, wrLengthSouth_, wrLengthNorth_;
  float wrLengthGrip;

  void set_bending_angle(float pAngle, float tAngle);
  void set_forceps_angle(float angle);
  void get_bending_kinematic_result(float pAngle, float tAngle, float grip);
  void kinematics();
  
  float tomm();
  float todegree();

private:
  // const double PI = acos(-1);
  const double PI_ = M_PI;

  // unit : radian
  float deg_ = M_PI / 180;
	float rad_ = 180 / M_PI;
	float mm_ = 0.001;

  float pAngle_ = 0;   // East * West
	float tAngle_ = 0;   // South * North
  float target_forceps_angle_ = 0;

  float alpha_;
};