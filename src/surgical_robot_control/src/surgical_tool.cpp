#include "surgical_tool.hpp"

SurgicalTool::SurgicalTool() {
	init_surgicaltool(
		NUM_OF_JOINT,
		SEGMENT_ARC,
		SEGMENT_DIAMETER,
		WIRE_DISTANCE,
		SHIFT
		);
	std::cout << "Surgical tool is created" << &this->surgicaltool_ << std::endl;
}

SurgicalTool::~SurgicalTool() {

}

void SurgicalTool::init_surgicaltool(	int num_joint,
																			float arc,
																			float diameter,
																			float disWire,
																			float shift
																		)
{
	this->surgicaltool_.num_joint = num_joint;
	this->surgicaltool_.arc 			=	arc * mm_;
	this->surgicaltool_.diameter  =	diameter * mm_;
	this->surgicaltool_.disWire   =	disWire * mm_;
	this->surgicaltool_.shift			= shift * torad();
	this->alpha_ = asin(this->surgicaltool_.disWire / this->surgicaltool_.arc);
}

void SurgicalTool::set_bending_angle(double pAngle, double tAngle) {
	this->pAngle_ = pAngle * torad();
	this->tAngle_ = tAngle * torad();
}

void SurgicalTool::set_forceps_angle(double angle) {	// degree
	this->target_forceps_angle_ = angle;	// non radian
}

std::tuple<double, double, double, double, double> SurgicalTool::get_bending_kinematic_result(
	double pAngle,
	double tAngle,
	double gAngle)
{
	// 1. set angle(degree) of continuum part
	this->set_bending_angle(pAngle, tAngle);
	// 2. set angle(degree) o forceps
	this->set_forceps_angle(gAngle);
	// 3. calculate kinematics
	this->kinematics();

	return std::make_tuple(this->wrLengthEast_, this->wrLengthWest_, this->wrLengthSouth_, this->wrLengthNorth_, this->wrLengthGrip);
	// return std::tuple<double, double, double, double, double>
	// (this->wrLengthEast_, this->wrLengthWest_, this->wrLengthSouth_, this->wrLengthNorth_, this->wrLengthGrip);
}

void SurgicalTool::kinematics()
{
	// y = kx (k=SHIFT/SHIFT_THRESHOLD)
	if (pAngle_ <= SHIFT_THRESHOLD * torad() && pAngle_ >= -SHIFT_THRESHOLD * torad()) 
	{ 
		pAngle_ = (pAngle_- pAngle_ * (surgicaltool_.shift/SHIFT_THRESHOLD)) / surgicaltool_.num_joint;	
	}
	else {
		pAngle_ = (pAngle_ - surgicaltool_.shift) / surgicaltool_.num_joint;
	}

	if (tAngle_ <= SHIFT_THRESHOLD * torad() && tAngle_ >= -SHIFT_THRESHOLD * torad()) 
	{ 
		tAngle_ = (tAngle_- tAngle_ * (surgicaltool_.shift/SHIFT_THRESHOLD)) / surgicaltool_.num_joint;	
	}
	else {
		tAngle_ = (tAngle_ - surgicaltool_.shift) / surgicaltool_.num_joint;
	}

	this->wrLengthEast_  = 2 * surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ + (pAngle_ / (2*surgicaltool_.num_joint))) + 1 - cos(tAngle_ / (2*surgicaltool_.num_joint)));
	this->wrLengthWest_  = 2 * surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ - (pAngle_ / (2*surgicaltool_.num_joint))) + 1 - cos(tAngle_ / (2*surgicaltool_.num_joint)));
	this->wrLengthSouth_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ - (tAngle_ / (2*surgicaltool_.num_joint))) + 1 - cos(pAngle_ / (2*surgicaltool_.num_joint)));
	this->wrLengthNorth_ = 2 * surgicaltool_.arc * surgicaltool_.num_joint * ( cos(alpha_) - cos(alpha_ + (tAngle_ / (2*surgicaltool_.num_joint))) + 1 - cos(pAngle_ / (2*surgicaltool_.num_joint)));

	this->wrLengthEast_ =  this->wrLengthEast_ / mm_;
	this->wrLengthWest_ =  this->wrLengthWest_ / mm_;
	this->wrLengthSouth_ = this->wrLengthSouth_ / mm_;
	this->wrLengthNorth_ = this->wrLengthNorth_ / mm_;

	// y = -x + 30
	this->wrLengthGrip = ((-1) * this->target_forceps_angle_ + this->max_forceps_deg_) * ( MAX_FORCEPS_RAGNE_MM / MAX_FORCEPS_RAGNE_DEGREE ); 
}

float SurgicalTool::tomm()
{
	return this->mm_;
}

float SurgicalTool::torad()
{
	return this->deg_;
}