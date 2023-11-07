#include "surgical_tool.hpp"

SurgicalTool::SurgicalTool()
{
	init_surgicaltool(
			NUM_OF_JOINT,
			SEGMENT_ARC,
			SEGMENT_DIAMETER,
			WIRE_DISTANCE,
			SLOT_LENGTH,
			SLOT_WIDTH);
	std::cout << "Surgical tool is created" << &this->surgicaltool_ << std::endl;
}

SurgicalTool::~SurgicalTool()
{
}

void SurgicalTool::init_surgicaltool(int num_joint,
																		 float arc,
																		 float diameter,
																		 float disWire,
																		 float slotlength,
																		 float slotwidth)
{
	this->surgicaltool_.num_joint = num_joint;
	this->surgicaltool_.arc = arc * mm_;
	this->surgicaltool_.diameter = diameter * mm_;
	this->surgicaltool_.disWire = disWire * mm_;
	this->surgicaltool_.slotlength = slotlength * tomm();
	this->surgicaltool_.slotwidth = slotwidth * tomm();

	this->alpha_ = asin(this->surgicaltool_.disWire / this->surgicaltool_.arc);
	this->alphaP_ = asin((this->surgicaltool_.disWire + 0.5 * this->surgicaltool_.slotwidth) / this->surgicaltool_.arc);
	this->alphaN_ = asin((this->surgicaltool_.disWire - 0.5 * this->surgicaltool_.slotwidth) / this->surgicaltool_.arc);
	this->beta_ = asin(this->surgicaltool_.slotlength * 0.5 / this->surgicaltool_.arc);
}

void SurgicalTool::set_bending_angle(double pAngle, double tAngle)
{
	this->pAngle_ = pAngle * torad();
	this->tAngle_ = tAngle * torad();
}

void SurgicalTool::set_forceps_angle(double angle)
{																			 // degree
	this->target_forceps_angle_ = angle; // non radian
}

void SurgicalTool::get_bending_kinematic_result(
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
}

void SurgicalTool::kinematics()
{

	//===========================================================================
	//===========================================================================
	// pAngle_ = pAngle_/surgicaltool_.num_joint;
	// tAngle_ = tAngle_/surgicaltool_.num_joint;
	// float scale_factor = 1.0;
	// if (pAngle_ < 0) {
	// 	if (tAngle_ < 0) {
	// 		wrLengthWest_  = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaP_) - cos(alphaP_ - pAngle_ / 2) + scale_factor * (1 - cos(tAngle_ / 2)));
	// 		wrLengthEast_  = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaN_) - cos(alphaN_ + pAngle_ / 2) + scale_factor * (1 - cos(tAngle_ / 2)));
	// 		wrLengthSouth_ = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaP_) - cos(alphaP_ - tAngle_ / 2) + scale_factor * (1 - cos(pAngle_ / 2)));
	// 		wrLengthNorth_ = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaN_) - cos(alphaN_ + tAngle_ / 2) + scale_factor * (1 - cos(pAngle_ / 2)));
	// 	}
	// 	else {
	// 		wrLengthWest_  = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaP_) - cos(alphaP_ - pAngle_ / 2) + scale_factor * (1 - cos(tAngle_ / 2)));
	// 		wrLengthEast_  = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaN_) - cos(alphaN_ + pAngle_ / 2) + scale_factor * (1 - cos(tAngle_ / 2)));
	// 		wrLengthSouth_ = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaN_) - cos(alphaN_ - tAngle_ / 2) + scale_factor * (1 - cos(pAngle_ / 2)));
	// 		wrLengthNorth_ = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaP_) - cos(alphaP_ + tAngle_ / 2) + scale_factor * (1 - cos(pAngle_ / 2)));
	// 	}
	// }
	// else {
	// 	if (tAngle_ < 0) {
	// 		wrLengthWest_  = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaN_) - cos(alphaN_ - pAngle_ / 2) + scale_factor * (1 - cos(tAngle_ / 2)));
	// 		wrLengthEast_  = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaP_) - cos(alphaP_ + pAngle_ / 2) + scale_factor * (1 - cos(tAngle_ / 2)));
	// 		wrLengthSouth_ = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaP_) - cos(alphaP_ - tAngle_ / 2) + scale_factor * (1 - cos(pAngle_ / 2)));
	// 		wrLengthNorth_ = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaN_) - cos(alphaN_ + tAngle_ / 2) + scale_factor * (1 - cos(pAngle_ / 2)));
	// 	}
	// 	else {
	// 		wrLengthWest_  = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaN_) - cos(alphaN_ - pAngle_ / 2) + scale_factor * (1 - cos(tAngle_ / 2)));
	// 		wrLengthEast_  = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaP_) - cos(alphaP_ + pAngle_ / 2) + scale_factor * (1 - cos(tAngle_ / 2)));
	// 		wrLengthSouth_ = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaN_) - cos(alphaN_ - tAngle_ / 2) + scale_factor * (1 - cos(pAngle_ / 2)));
	// 		wrLengthNorth_ = 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alphaP_) - cos(alphaP_ + tAngle_ / 2) + scale_factor * (1 - cos(pAngle_ / 2)));
	// 	}
	// }

	// double posLimitWire = 0.107313 * tomm();
	// double negLimitWire = -0.0759263 * tomm();

	// if (pAngle_ < 1 * torad() && pAngle_ >= 0 * torad() && tAngle_ < 1 * torad() && tAngle_ >= 0 * torad()) {
	// 	wrLengthWest_ = negLimitWire / torad() * pAngle_;
	// 	wrLengthEast_ = posLimitWire / torad() * pAngle_;
	// 	wrLengthSouth_ = negLimitWire / torad() * tAngle_;
	// 	wrLengthNorth_ = posLimitWire / torad() * tAngle_;
	// }

	// if (pAngle_ < 0 * torad() && pAngle_ >= -1 * torad() && tAngle_ < 1 * torad() && tAngle_ >= 0 * torad()) {
	// 	wrLengthWest_ = - posLimitWire / torad() * pAngle_;
	// 	wrLengthEast_ = - negLimitWire / torad() * pAngle_;
	// 	wrLengthSouth_ = negLimitWire / torad() * tAngle_;
	// 	wrLengthNorth_ = posLimitWire / torad() * tAngle_;
	// }

	// if (pAngle_ < 0 * torad() && pAngle_ >= -1 * torad() && tAngle_ < 0 * torad() && tAngle_ >= -1 * torad()) {
	// 	wrLengthWest_ = - posLimitWire / torad() * pAngle_;
	// 	wrLengthEast_ = - negLimitWire / torad() * pAngle_;
	// 	wrLengthSouth_ = -posLimitWire / torad() * tAngle_;
	// 	wrLengthNorth_ = -negLimitWire / torad() * tAngle_;
	// }

	// if (pAngle_ < 1 * torad() && pAngle_ >= 0 * torad() && tAngle_ < 0 * torad() && tAngle_ >= -1 * torad()) {
	// 	wrLengthWest_ = negLimitWire / torad() * pAngle_;
	// 	wrLengthEast_ = posLimitWire / torad() * pAngle_;
	// 	wrLengthSouth_ = -posLimitWire / torad() * tAngle_;
	// 	wrLengthNorth_ = -negLimitWire / torad() * tAngle_;
	// }

	// this->wrLengthEast_ =  this->wrLengthEast_ / tomm();
	// this->wrLengthWest_ =  this->wrLengthWest_ / tomm();
	// this->wrLengthSouth_ = this->wrLengthSouth_ / tomm();
	// this->wrLengthNorth_ = this->wrLengthNorth_ / tomm();

	// std::cout << "South wire length: " << wrLengthSouth_ << std::endl;
	// std::cout << "North wire length: " << wrLengthNorth_  << std::endl;
	// std::cout << "East wire length: " << wrLengthEast_  << std::endl;
	// std::cout << "West wire length: " << wrLengthWest_  << std::endl;
	// std::cout << "==================" << std::endl;
	//===========================================================================
	//===========================================================================

	pAngle_ = pAngle_ / surgicaltool_.num_joint;
	tAngle_ = tAngle_ / surgicaltool_.num_joint;

	float scale_factor_k = 1.0;
	float scale_factor_p1 = 1.0;
	float scale_factor_p2 = 1.0;
	float scale_factor_p3 = 0.7;

	// scale
	float scale_factor_kw = 1.0;
	float scale_factor_ke = 1.0;
	float scale_factor_ks = 1.0;
	float scale_factor_kn = 1.0;
	float scale_factor_pullWire = 1.0;
	float scale_factor_pushWire = 0.8;

	if (pAngle_ >= 0 && tAngle_ >= 0)
	{
		scale_factor_kw = scale_factor_pullWire;
		scale_factor_ke = scale_factor_pushWire;
		scale_factor_ks = scale_factor_pullWire;
		scale_factor_kn = scale_factor_pushWire;
	}
	else if (pAngle_ < 0 && tAngle_ >= 0)
	{
		scale_factor_kw = scale_factor_pushWire;
		scale_factor_ke = scale_factor_pullWire;
		scale_factor_ks = scale_factor_pullWire;
		scale_factor_kn = scale_factor_pushWire;
	}
	else if (pAngle_ >= 0 && tAngle_ < 0)
	{
		scale_factor_kw = scale_factor_pullWire;
		scale_factor_ke = scale_factor_pushWire;
		scale_factor_ks = scale_factor_pushWire;
		scale_factor_kn = scale_factor_pullWire;
	}
	else
	{
		scale_factor_kw = scale_factor_pushWire;
		scale_factor_ke = scale_factor_pullWire;
		scale_factor_ks = scale_factor_pushWire;
		scale_factor_kn = scale_factor_pullWire;
	}

	wrLengthEast_ = scale_factor_ke * 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alpha_) - cos(alpha_ + pAngle_ / 2) + scale_factor_p3 * (1 - cos(tAngle_ / 2)));
	wrLengthWest_ = scale_factor_kw * 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alpha_) - cos(alpha_ - pAngle_ / 2) + scale_factor_p3 * (1 - cos(tAngle_ / 2)));
	wrLengthSouth_ = scale_factor_ks * 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alpha_) - cos(alpha_ - tAngle_ / 2) + scale_factor_p3 * (1 - cos(pAngle_ / 2)));
	wrLengthNorth_ = scale_factor_kn * 2 * this->surgicaltool_.arc * this->surgicaltool_.num_joint * (cos(alpha_) - cos(alpha_ + tAngle_ / 2) + scale_factor_p3 * (1 - cos(pAngle_ / 2)));

	this->wrLengthEast_ = this->wrLengthEast_ / tomm();
	this->wrLengthWest_ = this->wrLengthWest_ / tomm();
	this->wrLengthSouth_ = this->wrLengthSouth_ / tomm();
	this->wrLengthNorth_ = this->wrLengthNorth_ / tomm();

	// std::cout << "South wire length: " << wrLengthSouth_ << std::endl;
	// std::cout << "North wire length: " << wrLengthNorth_ << std::endl;
	// std::cout << "East wire length: " << wrLengthEast_ << std::endl;
	// std::cout << "West wire length: " << wrLengthWest_ << std::endl;
	// std::cout << "==================" << std::endl;

	// y = -x + 30
	this->wrLengthGrip = ((-1) * this->target_forceps_angle_ + this->max_forceps_deg_) * (MAX_FORCEPS_RAGNE_MM / MAX_FORCEPS_RAGNE_DEGREE);
}

float SurgicalTool::tomm()
{
	return this->mm_;
}

float SurgicalTool::torad()
{
	return this->deg_;
}