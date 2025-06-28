



#include "ChJoint.h"

namespace chrono

{

	ChJoint::ChJoint(Real min, Real max, std::shared_ptr<ChLinkMotor> motor) : m_min(min), m_max(max), m_motor(motor) {

		
		motor->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(0));
		
	}



	ChJoint::~ChJoint()
	{
	}




	//ChJointPrismatic::ChJointPrismatic(Real min, Real max, std::shared_ptr<ChLinkMotor> motor) : ChJoint(min, max, motor) {
	//}

	ChJointPrismatic::~ChJointPrismatic()
	
	{
	}

	//ChJointRevolute::ChJointRevolute(Real min, Real max, bool wraparound, std::shared_ptr<ChLinkMotor> motor) : ChJoint(min, max, motor) {
	//}


	ChJointRevolute::~ChJointRevolute()
	{
	}



}