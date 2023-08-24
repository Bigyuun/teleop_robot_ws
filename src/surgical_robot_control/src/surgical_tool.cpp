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

void SurgicalTool::init_surgicaltool(	double num_joint,
																			double arc,
																			double diameter,
																			double disWire
																		)
{
	this->surgicaltool_.num_joint = num_joint;
	this->surgicaltool_.arc 			=	arc * mm_;
	this->surgicaltool_.diameter  =	diameter * mm_;
	this->surgicaltool_.disWire   =	disWire * mm_;
	this->alpha_ = asin(this->surgicaltool_.disWire / this->surgicaltool_.arc);
}

void SurgicalTool::set_target_angle(float pAngle, float tAngle) {
	this->pAngle_ = pAngle;
	this->tAngle_ = tAngle;
}

void SurgicalTool::kinematics()
{
	this->wrLengthWest_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ - pAngle_ / 2) + 1 - cos(tAngle_ / 2));
	this->wrLengthEast_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ + pAngle_ / 2) + 1 - cos(tAngle_ / 2));
	this->wrLengthSouth_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint * (cos(alpha_) - cos(alpha_ - tAngle_ / 2) + 1 - cos(pAngle_ / 2));
	this->wrLengthNorth_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint * (cos(alpha_) - cos(alpha_ + tAngle_ / 2) + 1 - cos(pAngle_ / 2));
}