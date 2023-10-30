#ifndef SURGICAL_TOOL_HPP
#define SURGICAL_TOOL_HPP

#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <math.h>

#include "hw_definition.hpp"

/**
 * @authors DY, JKim
 * @brief Structure of Surgical tool (Forceps)
 * @unit Degree
 * @param pAngle : Pan angle (East(-) & West(+))
 * @param tAngle : Tilt angle (South(+) & North(-))
 * @param num_joint number of joint of continuum parts
 * @param arc       degree of arc of the segment part (mm)
 * @param diameter  diameter of the segment (mm)
 * @param disWire   distance between the center and the center of ellipse (mm)
 * @param shift     center of wire in tilt direction shifted during pan motion
 */
struct structure {
  int num_joint;
  float pAngle;
  float tAngle;
  float arc;
  float diameter;
  float disWire;
  float shift;
};

class SurgicalTool
{
public:
  /**
   * @brief Construct a new Surgical Tool object
   */
  SurgicalTool();

  /**
   * @brief Destroy the Surgical Tool object
   */
  ~SurgicalTool();

  /**
   * @brief make 1 object of the surgical tool 
   * 
   */
  struct structure surgicaltool_;
  
  /**
   * @authors DY
   * @brief initialize of surgical tool
   */
  void init_surgicaltool(
    int num_joint,
    float arc,
    float diameter,
    float disWire,
    float shift
  );

  // **************************
  // Limitation of workspace
  // **************************
  double max_bending_deg_ = MAX_BENDING_DEGREE;
  double max_forceps_deg_ = MAX_FORCEPS_RAGNE_DEGREE;

  /**w
   * @brief target length for moving wire using motor 
   * @unit mm
   */
	double wrLengthWest_, wrLengthEast_, wrLengthSouth_, wrLengthNorth_;
  double wrLengthGrip;

  /**
   * @brief Set the bending angle object
   * @unit degree
   * @param pAngle 
   * @param tAngle 
   */
  void set_bending_angle(double pAngle, double tAngle);

  /**
   * @brief Set the forceps angle object
   * @unit degree
   * @param angle 
   */
  void set_forceps_angle(double angle);

  /**
   * @brief Get the bending kinematic result object
   * @param pAngle 
   * @param tAngle 
   * @param grip 
   * @return void
   *         but the wrLength** variables has the target values
   */
  void get_bending_kinematic_result(double pAngle, double tAngle, double grip);

  /**
   * @brief calculate the kinematics
   */
  void kinematics();

  /**
   * @brief make input variable to 'mm' unit
   * @return * float 
   */
  float tomm();

  /**
   * @brief make input variable to 'rad' unit
   * @return float 
   */
  float torad();

private:
  // const double PI = acos(-1);
  const double PI_ = M_PI;

  double deg_ = M_PI / 180;
	double rad_ = 180 / M_PI;
	double mm_ = 0.001;

  double pAngle_ = 0;   // East * West
	double tAngle_ = 0;   // South * North
  double target_forceps_angle_ = 30;

  double alpha_;
};

#endif