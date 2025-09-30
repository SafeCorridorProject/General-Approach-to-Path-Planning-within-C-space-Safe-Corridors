
//#include "ChPlanner.h"
//#include "ChPlanModel.h"
//#include "ChPlanner.h"
#include <physics/ChLinkMotorLinear.h>
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "ChJoint.h"
#include "ChPlanModel.h"
#include "ChPlanner.h"  
#include "ChPlannerOMPL.h"
//#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
//#include "chrono/physics/ChLinkMotorLinearPosition.h"
//#include "chrono/physics/ChLinkMotorLinearSpeed.h"
//#include "chrono/physics/ChLinkMotorLinearForce.h"
//#include "chrono/physics/ChLinkMotorLinearDriveline.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/functions/ChFunctionSine.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include <cmath>

//#include "chrono/core/ChLog.h"
//#include "chrono/core/ChTransform.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChFrameMoving.h"
#include "chrono/core/ChTimer.h"
//#include "chrono/physics/ChLinkMotorRotationAngle.h"
//#include "chrono/physics/ChLinkMotorRotationSpeed.h"
//#include "chrono/physics/ChLinkMotorRotationTorque.h"
//#include "chrono/physics/ChLinkMotorRotationDriveline.h"
//#include "chrono/physics/ChLinkMotorLinearPosition.h"
//#include "chrono/physics/ChLinkMotorLinearSpeed.h"
//#include "chrono/physics/ChLinkMotorLinearForce.h"
//#include "chrono/physics/ChLinkMotorLinearDriveline.h"
//#include "chrono/physics/ChShaftsMotorSpeed.h"
//#include "chrono/physics/ChShaftsMotorPosition.h"
//#include "chrono/physics/ChShaftsPlanetary.h"
//#include "chrono/physics/ChShaftsGear.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/functions/ChFunctionSine.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include <cmath>

//#include "chrono/core/ChLog.h"
//#include "chrono/core/ChTransform.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChFrameMoving.h"
#include "chrono/core/ChTimer.h"
namespace chrono
{



	///////////////////// ChPlanModel /////////////////////



	ChPlanModel::~ChPlanModel()
	{
	}





	double ChPlanModel::distance(const ChVectorDynamic<Real>& q1, const ChVectorDynamic<Real>& q2) const {

		double dist = 0;
		for (auto js = 0; js < m_joints.size(); ++js) {
			dist += m_joints[js]->distance(q1[js], q2[js]);                                       /////       //distance between the two joints configurations
		}
		return dist;
	}




	ChVectorDynamic<ChPlanModel::Real> ChPlanModel::interpolate(const ChVectorDynamic<Real>& q1, const ChVectorDynamic<Real>& q2, const Real& alpha) const {

		ChVectorDynamic<Real> i_joint(getDof());
		for (auto js = 0; js < m_joints.size(); ++js) {                                                                    // the values in between represent intermpolated configurations
			i_joint[js] = m_joints[js]->interpolate(q1[js], q2[js], alpha);                                                // a vector that will be populated with the interpolated joint configurations
		}
		return i_joint;
	}

	bool ChPlanModel::isColliding()
	{
		++this->totalQueries;

		bool has_collided = collision_detector->HasCollided();

		//std::cout << "Collision?" << has_collided << std::endl;

		++this->freeQueries;
		return has_collided;
		                                                     
	}

	// due to isColliding()
	::std::size_t ChPlanModel::getBodies() const
	{
		
		//return m_system->GetBodies().size();                      //  modified
		return m_system->GetNumBodies();
	}
	
	//void ChPlanModel::setPosition(const ChVectorDynamic<Real>& q_final)
	//{
	//	ChVectorDynamic<> q_intermediate(m_joints.size());
	//	ChVectorDynamic<> q_current(m_joints.size());

	//	
	//	assert(q_final.size() == m_joints.size());
	//	for (auto fun_sel = 0; fun_sel < m_joints.size(); ++fun_sel) {
	//		q_current[fun_sel] = m_joints[fun_sel]->getFunction()->GetConstant();
	//	}

	//	//std::cout << "Current position: " << q_current << std::endl;

	//	const int numInterpolationSteps = 10;
	//	for (auto i = 0; i <= numInterpolationSteps; i++) {
	//		q_intermediate = this->interpolate(q_current, q_final, i / (double)numInterpolationSteps);

	//		//std::cout << "Intermediate position: " << q_intermediate << std::endl;

	//		for (auto fun_sel = 0; fun_sel < m_joints.size(); ++fun_sel) {
	//			m_joints[fun_sel]->getFunction()->SetConstant(q_intermediate(fun_sel));
	//		}
	//		m_system->DoAssembly(AssemblyLevel::POSITION);
	//		
	//	}
	//	m_system->ComputeCollisions();                             // TODO: check if proper position
	//}



	void ChPlanModel::setPosition(const ChVectorDynamic<Real>& q) {
		assert(q.size() == m_joints.size());
		for (auto fun_sel = 0; fun_sel < m_joints.size(); ++fun_sel) {
			m_joints[fun_sel]->getFunction()->SetConstant(q[fun_sel]);
			

		}
		m_system->DoAssembly(AssemblyLevel::POSITION);
		m_system->ComputeCollisions();		                            
	}

     
//	void
//		ChPlanModel::updateFrames(bool doUpdateModel)   //TODO: check updateFrames
//	{
//		std::cout << "Update" << std::endl;
//
//		m_system->DoAssembly(AssemblyLevel::POSITION);
//
//		if (doUpdateModel)
//		{
//			m_system->ComputeCollisions();
//		}
//
//	}




		::std::size_t ChPlanModel::getFreeQueries() const
		{
			return this->freeQueries;
		}

		::std::size_t ChPlanModel::getTotalQueries() const
		{
			return this->totalQueries;
		}

		void ChPlanModel::AddJoint(std::shared_ptr<ChJoint> new_joint) 
		{
			m_joints.push_back(new_joint);
		}



		ChVectorDynamic<ChPlanModel::Real> ChPlanModel::getMaximum() const {

			ChVectorDynamic<Real> joints_max(getDof());

			for (auto js = 0; js < m_joints.size(); ++js) {
				joints_max[js] = m_joints[js]->getMaximum();
			}
			return joints_max;
		}

		ChVectorDynamic<ChPlanModel::Real> ChPlanModel::getMinimum() const {
			ChVectorDynamic<Real> joints_min(getDof());

			for (auto js = 0; js < m_joints.size(); ++js) {
				joints_min[js] = m_joints[js]->getMinimum();
			}
			return joints_min;
		}
		//void ChPlanModel::RemoveAxis(size_t rem_axis_id) {

		//    m_functionslist.erase(m_functionslist.begin() + rem_axis_id);

		//    if (rem_axis_id >= 0 && rem_axis_id < m_function_min.size()) {
		//        // TODO: check if it works
		//        m_function_min.segment(rem_axis_id, m_function_min.size() - rem_axis_id - 1) = m_function_min.segment(rem_axis_id + 1, m_function_min.size() - rem_axis_id - 1);
		//        m_function_min.conservativeResize(m_function_min.size() - 1);

		//        m_function_max.segment(rem_axis_id, m_function_max.size() - rem_axis_id - 1) = m_function_max.segment(rem_axis_id + 1, m_function_max.size() - rem_axis_id - 1);
		//        m_function_max.conservativeResize(m_function_max.size() - 1);
		//    }
		//    else {
		//        std::cerr << "Invalid index to remove." << std::endl;
		//    }

		//}

		bool ChPlanModel::isColliding(const ChVectorDynamic<Real>& q)
		{
			collision_detector->ResetCollisionFlag();
			this->setPosition(q);
			bool has_collided = collision_detector->HasCollided();

			return has_collided;
		}

		bool ChPlanModel::isValid(const ChVectorDynamic<Real>& q) const
		{
			assert(q.size() == this->getDof());

			for (auto i = 0; i < this->m_joints.size(); ++i)
			{
				if (!this->m_joints[i]->isValid(q[i]))
				{
					return false;
				}
			}

			return true;
		}
		////////////////////////////

		ChVectorDynamic<> ChPlanModel::computeEndEffectorPosition(const ChVectorDynamic<>& system_state) const
		{
			// Assume system_state[0] and system_state[1] are the positions of the X and Y bodies
			double posX = system_state[0];
			double posY = system_state[1];

			// Return the position as a vector
			ChVectorDynamic<> endEffectorPos(2);
			endEffectorPos << posX, posY;

			return endEffectorPos;
		}


		void ChPlanModel::reset()
		{
			this->freeQueries = 0;
			this->totalQueries = 0;
		}


	
}