

#ifndef CH_JOINT_H
#define CH_JOINT_H

//#include <rl/math/Units.h>
//#include <rl/math/Vector.h>
//#include "Transform.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChMatrix.h"
#include <string>
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotor.h"
#include <iostream>
#include <cmath>
#include <limits>
//#include "chrono/functions/ChFunctionBase.h"
//#include "chrono/functions/ChFunctionConst.h"
//#include "chrono/physics/ChLinkMotorLinear.h"

namespace chrono
{

	class ChJoint
	{
	public:

		typedef double Real;

		ChJoint(Real min, Real max, std::shared_ptr<ChLinkMotor> motor);

		virtual ~ChJoint();

		virtual ::std::size_t getDof() const { return 1; }

		virtual Real getMaximum() const { return m_max; }

		virtual Real getMinimum() const { return m_min; }

		virtual std::shared_ptr<ChFunctionConst> getFunction() const { return std::dynamic_pointer_cast<ChFunctionConst>(m_motor->GetMotorFunction()); }
		

		virtual double distance(Real q1, Real q2) const = 0;

		virtual double interpolate(Real q1, Real q2, Real alpha) const = 0;

		virtual bool isValid(Real q) const{
			return q >= m_min && q <= m_max;
		}

		virtual void setMaximum(Real new_max) {
			m_max = new_max;
		}


		virtual void setMinimum(Real new_min) {
			m_min = new_min;
		}



	protected:
		Real m_max;
		Real m_min;
		
		std::shared_ptr<ChLinkMotor> m_motor; 
		




	};

	class ChJointPrismatic : public ChJoint {
	public:
		ChJointPrismatic(Real min, Real max, std::shared_ptr<ChLinkMotor> motor) : ChJoint(min, max, motor) {
		}
		virtual ~ChJointPrismatic();

		virtual double distance(Real q1, Real q2) const override {
		   
			return std::abs(q2 - q1);
		}
		virtual double interpolate(Real q1, Real q2, Real alpha) const override {

			return (1 - alpha) * q1 + alpha * q2;
		}





	};

	
	class ChJointRevolute : public ChJoint {
	public:
		ChJointRevolute(Real min, Real max, bool wraparound, std::shared_ptr<ChLinkMotor> motor) : ChJoint(min, max, motor) {
		}
		virtual ~ChJointRevolute();
    // distance between two angles, q1 and q2. It calculates the absolute distance between two angles without considering the wraparound.
	// If wraparound is enabled, it calculates the distance considering the wraparound. It calculates the total range of motion of the joint
    // as the absolute difference between the maximun and minimum that the joint can reach. It returns the minimun delta distance and the distance
	// considering the wraparound

		double q_wrap(double q) const {
			if ((m_max - q) >= 0.0 && (m_max - q) < std::numeric_limits<Real>::epsilon())
				return m_max;

			if ((q - m_min) >= 0.0 && (q - m_min) < std::numeric_limits<Real>::epsilon())
				return m_min;

			return std::fmod(q - m_min, m_max - m_min) + m_min;
		}

		virtual double distance(Real q1, Real q2) const override {

			Real delta = std::abs(q2 - q1);

			if (m_wraparound)
			{
				Real range = std::abs(m_max - m_min);                      // total range of motion
				//Real range = std::abs(360 - 0);
				//Real distance = std::min(delta, std::abs(range - delta));  // return the minimum between delta and the difference between the range and delta
				//return distance;
				Real q1_wrap = q_wrap(q1);
				Real q2_wrap = q_wrap(q2);
				Real distance = std::abs(q2_wrap - q1_wrap);
				//Real through_wrap_distance = std::abs(range - delta);                  //add now
				return distance;
				//return std::min(delta, through_wrap_distance);                         //add now
			}
			else
			{
				return delta;
			}
		}
		virtual double interpolate(Real q1, Real q2, Real alpha) const override {

			Real q;

			if (m_wraparound)
			{
				Real diff = std::abs(q2 - q1);
				Real range = std::abs(m_max - m_min);

				if (std::abs(range - diff) < diff)             
				{
					if (q1 > q2)
					{
						q = (1 - alpha) * q1 + alpha * (q2 + range);
					}
					else
					{
						q = (1 - alpha) * (q1 + range) + alpha * q2;
					}

					while (q > m_max)
					{
						q -= range;
					}

					while (q < m_min)
					{
						q += range;
					}
				}
				else
				{
					q = (1 - alpha) * q1 + alpha * q2;
				}
			}
			else
			{
				q = (1 - alpha) * q1 + alpha * q2;
			}

			return q;
		}

		

	
	
	 protected:
		bool m_wraparound;
		

	};


	}


#endif // CH_JOINT_H
