/*
 *  Copyright (c) 2012-2013, MYOROBOTICS consortium
 *  Author: Steffen Wittmeier, Konstantinos Dalamagkidis, Michael Jaentsch
 *  All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification is governed by the MYOROBOTICS Non-Commercial Software
 *  License Agreement. See LICENSE file distributed with this work for
 *  additional information.
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under this license is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either expressed or
 *  implied. See the License for the specific language governing permissions
 *  and limitations under the License.
 *
 */

#ifndef _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_PULLEY_STATE_MACHINE_H_
#define _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_PULLEY_STATE_MACHINE_H_

#include <rl/math/Vector.h>
#include <sgal/Exception.h>

namespace de
{
namespace caliper
{
namespace sim
{
namespace ext
{
namespace actuators
{
namespace er
{
namespace physics
{
class PulleyStateMachine
{
public:
	enum State
	{
		NotWrapping, Positive, Negative
	};

	PulleyStateMachine(State initialState, unsigned int numHalfRevolutions);

	virtual ~PulleyStateMachine();

	State getState() const;

	unsigned int getNumHalfRevolutions() const;

	void updateState(const rl::math::Vector3& anchorA,
			const rl::math::Vector3& anchorB, const rl::math::Vector3& N,
			rl::math::Real radius);

	void updateRevolutionCounter(const rl::math::Vector3& previousForcePoint,
			const rl::math::Vector3& nextForcePoint) throw (sgal::Exception);

	double getArcAngle(const rl::math::Vector3& pointA,
			const rl::math::Vector3& pointB, rl::math::Real radius);

	static void computeTangentPoint(const rl::math::Vector3& point,
			const rl::math::Vector3& N, rl::math::Real radius,
			rl::math::Vector3& tangentPointA, rl::math::Vector3& tangentPointB)
					throw (sgal::Exception)
	{
		if (point.norm() < radius)
		{
			tangentPointA = tangentPointB = point;

			throw sgal::Exception(
					"Tangent computation error: anchor penetrates sphere");
		}

		rl::math::Vector3 K1 = point.cross(N);

		K1.normalize();

		rl::math::Real a1 = std::pow(radius, 2) / point.norm();

		rl::math::Real b1 = std::sqrt(std::pow(radius, 2) - std::pow(a1, 2));

		tangentPointA = a1 * point.normalized() - b1 * K1;

		tangentPointB = a1 * point.normalized() + b1 * K1;
	}

protected:
	State state;

	rl::math::Vector3 N;

	bool firstUpdate;

	rl::math::Real TBonTAA;

	unsigned int numHalfRevolutions;

	rl::math::Vector3 previousAnchor;
};
}
}
}
}
}
}
}

#endif
