#include <iostream>
#include <cmath>
#include <chrono>
#include <math.h>

using namespace std;

struct SurgicalTool {
  double NumJ;
  double pAngle;
  double tAngle;
  double radius;
  double width;
  double disWire;
};

class Kinematics
{
public:
  struct SurgicalTool surgicaltool_;

private:
  void init_surgicaltool(
    double NumJ,
    double pAngle,
    double tAngle,
    double radius,
    double width,
    double disWire
  );
  
  // const double PI = acos(-1);
  const double PI = M_PI;

  // Make wire
	double deg = PI / 180;
	double rad = 180 / PI;
	double mm = 0.001;

  double alpha;
	double wrLengthWest, wrLengthEast, wrLengthSouth, wrLengthNorth;
};