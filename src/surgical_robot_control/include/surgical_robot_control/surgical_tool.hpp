#include <iostream>
#include <cmath>
#include <chrono>
#include <math.h>

// DY
#define NUM_OF_JOINT      4     // ea
#define SEGMENT_ARC       6.92  // mm
#define SEGMENT_DIAMETER  3     // mm
#define WIRE_DISTANCE     1.05  // mm

using namespace std;

struct structure {
  double num_joint;
  double pAngle;
  double tAngle;
  double arc;
  double diameter;
  double disWire;
};

class SurgicalTool
{
public:
  SurgicalTool();
  ~SurgicalTool();

  struct structure surgicaltool_;
  void init_surgicaltool(
    double num_joint,
    double arc,
    double diameter,
    double disWire
  );
  void set_target_angle(float pAngle, float tAngle);
  void get_target_linear();
  void kinematics();

private:
  // const double PI = acos(-1);
  const double PI_ = M_PI;

  // Make wire
  // unit : radian
	double deg_ = M_PI / 180;
	double rad_ = 180 / M_PI;
	double mm_ = 0.001;

  float pAngle_ = 5 * deg_;   // East * West
	float tAngle_ = -5 * deg_;  // South * North

  double alpha_;
	double wrLengthWest_, wrLengthEast_, wrLengthSouth_, wrLengthNorth_;
};