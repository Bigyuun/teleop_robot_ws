#include "surgical_tool.hpp"


SurgicalTool::SurgicalTool() {
	init_surgicaltool(
		NUM_OF_JOINT,
		SEGMENT_ARC,
		SEGMENT_DIAMETER,
		WIRE_DISTANCE
		);
	std::cout << "Surgical tool is created" << &this->surgicaltool_ << std::endl;
}

SurgicalTool::~SurgicalTool() {

}

void SurgicalTool::init_surgicaltool(	int num_joint,
																			float arc,
																			float diameter,
																			float disWire
																		)
{
	this->surgicaltool_.num_joint = num_joint;
	this->surgicaltool_.arc 			=	arc * mm_;
	this->surgicaltool_.diameter  =	diameter * mm_;
	this->surgicaltool_.disWire   =	disWire * mm_;
	this->alpha_ = asin(this->surgicaltool_.disWire / this->surgicaltool_.arc);
}

void SurgicalTool::set_bending_angle(float pAngle, float tAngle) {
	this->pAngle_ = pAngle * torad();
	this->tAngle_ = tAngle * torad();
}

void SurgicalTool::set_forceps_angle(float angle) {	// degree
	this->target_forceps_angle_ = angle;	// non radian
}

void SurgicalTool::get_bending_kinematic_result(
	float pAngle,
	float tAngle,
	float gAngle)
{
	// 1. set angle(degree) of continuum part
	this->set_bending_angle(pAngle, tAngle);
	// 2. set angle(degree) o forceps
	this->set_forceps_angle(gAngle);
	// 3. calculate kinematics
	this->kinematics();
}

void SurgicalTool::kinematics()
{
	this->wrLengthEast_ = 2 *  surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ + pAngle_ / 2) + 1 - cos(tAngle_ / 2));
	this->wrLengthWest_ = 2 *  surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ - pAngle_ / 2) + 1 - cos(tAngle_ / 2));
	this->wrLengthSouth_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint * (cos(alpha_) - cos(alpha_ - tAngle_ / 2) + 1 - cos(pAngle_ / 2));
	this->wrLengthNorth_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint * (cos(alpha_) - cos(alpha_ + tAngle_ / 2) + 1 - cos(pAngle_ / 2));

	this->wrLengthEast_ =  (-1) * this->wrLengthEast_ / mm_;
	this->wrLengthWest_ = 			  this->wrLengthWest_ / mm_;
	this->wrLengthSouth_ = 			  this->wrLengthSouth_ / mm_;
	this->wrLengthNorth_ = (-1) * this->wrLengthNorth_ / mm_;

	// y = -x + 30
	this->wrLengthGrip = (-1) * this->target_forceps_angle_ + this->max_forceps_deg_; 
}

float SurgicalTool::tomm()
{
	return this->mm_;
}

float SurgicalTool::torad()
{
	return this->deg_;
}