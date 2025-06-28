

#ifndef CH_PLAN_MODEL_H
#define CH_PLAN_MODEL_H


#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionInfo.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/ChCollisionShape.h"
#include "chrono/core/ChApiCE.h"
//#include "chrono/collision/multicore/ChCollisionSystemMulticore.h"
#include "chrono/core/ChVector3.h"
#include "ChJoint.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/physics/ChLinkMotor.h"

namespace chrono
{
        
        /// Narrow-phase manager to perform collision detection without adding contact constraints
        class CollisionDetectorOMPL : public ChCollisionSystem::NarrowphaseCallback {
        public:

            /// Constructor
            CollisionDetectorOMPL() {};

            /// Callback used to process collision pairs found by the narrow-phase collision step.
            /// Return true to generate a contact for this pair of overlapping bodies.
            virtual bool OnNarrowphase(ChCollisionInfo& contactinfo){
             
                m_has_collided = true;
                m_modelA = contactinfo.modelA;
                m_modelB = contactinfo.modelB;

                return false; // true/false: do/do not generate contact for overlapping bodies
            }

            void ResetCollisionFlag() {
                m_has_collided = false;
            }

            bool HasCollided() const {
                //std::cout << "Collision? " << m_has_collided << std::endl;

                return m_has_collided;
            }

            ChBody* GetCollidedBodyA() {
                return GetCollidedBody(m_modelA);
            }

            ChBody* GetCollidedBodyB() {
                return GetCollidedBody(m_modelB);
            }



        protected:

            ChBody* GetCollidedBody(ChCollisionModel* m_model) {

                if (!m_model)
                    return nullptr;

                ChContactable* contactable = m_model->GetContactable();

                ChBody* body_ptr = dynamic_cast<ChBody*>(contactable);

                return body_ptr;

            }

            ChCollisionModel* m_modelA = nullptr;
            ChCollisionModel* m_modelB = nullptr;

            bool m_has_collided = false;
        };







        class ChPlanModel
        {
        public:

            typedef double Real;

            ChPlanModel(ChSystem* sys) : m_system(sys), freeQueries(0), totalQueries(0) {
				collision_detector = std::make_shared<CollisionDetectorOMPL>();
				m_system->GetCollisionSystem()->RegisterNarrowphaseCallback(collision_detector);
			}

            virtual ~ChPlanModel();
            
            virtual double distance(const ChVectorDynamic<Real>& q1, const ChVectorDynamic<Real>& q2) const;          

            

            // Implicitly refers to the velocity dofs
            virtual ::std::size_t getDof() const { return m_joints.size(); }


            virtual ChVectorDynamic<Real> getMaximum() const;

            virtual ChVectorDynamic<Real> getMinimum() const;


            //virtual void interpolate(const ChVectorDynamic<Real>& q1, const ChVectorDynamic<Real>& q2, const Real& alpha) const;

            virtual ChVectorDynamic<Real>  interpolate(const ChVectorDynamic<Real>& q1, const ChVectorDynamic<Real>& q2, const Real& alpha) const;

            /// Tells if the current configuration is of collision
            virtual bool isColliding();

            /// Get number of bodies
            virtual ::std::size_t getBodies() const;

            std::shared_ptr<ChBody> ChPlanModel::getBody(const ::std::size_t& i) const { return m_system->GetBodies().at(i); }
           
            virtual void setPosition(const ChVectorDynamic<Real>& q_final);


            //virtual void updateFrames(bool doUpdateModel = true);

            void AddJoint(std::shared_ptr<ChJoint> new_joint);

            //void RemoveAxis(size_t rem_axis_id);


            ::std::size_t getFreeQueries() const;

            ::std::size_t getTotalQueries() const;

            std::vector<std::shared_ptr<ChBody>>  ChPlanModel::CreateCuboid(std::vector<ChVectorDynamic<>> new_path);
            std::vector<ChVector3d> ChPlanModel::GetCenter(std::vector<std::shared_ptr<ChBody>> cuboids);
            /// Tells if a given configuration is of collision
            virtual bool isColliding(const ChVectorDynamic<Real>& q);

            
            ChBody* getCollidingBodyA() {

                return collision_detector->GetCollidedBodyA();

            }

            
            ChBody* getCollidingBodyB() {

                return collision_detector->GetCollidedBodyB();

            }

            virtual void reset();
            virtual bool isValid(const ChVectorDynamic<Real>& q) const;


            ChVectorDynamic<> computeEndEffectorPosition(const ChVectorDynamic<>& system_state) const;         

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        protected:

            ChSystem* m_system;

            std::shared_ptr<CollisionDetectorOMPL> collision_detector;
            std::vector<std::shared_ptr<ChJoint>> m_joints;
            ChVectorDynamic<Real> m_function_min;
            ChVectorDynamic<Real> m_function_max;
            
            ChVectorDynamic<Real> q_current;
            
            /// Queries to collision system that ended in no collisions
            ::std::size_t freeQueries;

            /// Total number of queries to collision system
            ::std::size_t totalQueries;

        private:
            
        };

    
}
#endif // CH_PLAN_MODEL_H