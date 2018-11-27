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

#include "PulleyStateMachine.h"

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
PulleyStateMachine::PulleyStateMachine(State initialState,
		unsigned int numHalfRevolutions) :
		state(initialState), numHalfRevolutions(numHalfRevolutions), TBonTAA(
				0.0), firstUpdate(true)
{
	// initialize TBonTAA
	switch (state)
	{
	case NotWrapping:
		TBonTAA = 0.0;
		break;
	default:
		if (numHalfRevolutions % 2 == 0)
		{
			TBonTAA = -1.0;
		}
		else
		{
			TBonTAA = 1.0;
		}
		break;
	}
}

PulleyStateMachine::~PulleyStateMachine()
{

}

PulleyStateMachine::State PulleyStateMachine::getState() const
{
	return state;
}

unsigned int PulleyStateMachine::getNumHalfRevolutions() const
{
	return numHalfRevolutions;
}

void PulleyStateMachine::updateState(const rl::math::Vector3& anchorA,
		const rl::math::Vector3& anchorB, const rl::math::Vector3& N,
		rl::math::Real radius)
{
	// State machine
	previousAnchor = anchorA;

	rl::math::Vector3 anchorBn = anchorB.normalized();

	rl::math::Vector3 BAn = (anchorA - anchorB).normalized();

	rl::math::Real CtoAB = std::fabs(
			anchorB.norm() * std::sin(std::acos(anchorBn.dot(BAn))));

	if (firstUpdate)
	{
		this->N = N;
	}

	switch (state)
	{
	case NotWrapping:
	{
		if (CtoAB < radius && anchorA.dot(anchorB) < 0)
		{
			state = Positive;
		}

		break;
	}
	case Positive:
	{
		if (CtoAB >= radius && numHalfRevolutions == 0)
		{
			state = NotWrapping;
		}
		else if (N.dot(this->N) < 0)
		{
			state = Negative;
		}

		break;
	}
	case Negative:
	{
		if (N.dot(this->N) < 0)
		{
			state = Positive;
		}

		break;
	}
	default:
		break;
	}

	this->N = N;

	firstUpdate = false;
}

void PulleyStateMachine::updateRevolutionCounter(
		const rl::math::Vector3& previousForcePoint,
		const rl::math::Vector3& nextForcePoint) throw (sgal::Exception)
{
	switch (state)
	{
	case NotWrapping:
	{
		TBonTAA = 0;

		break;
	}
	case Positive:
	{
		double newTBonTAA = nextForcePoint.dot(
				previousAnchor - previousForcePoint);

		if (TBonTAA > 0 && newTBonTAA < 0)
		{
			numHalfRevolutions++;
		}
		else if (TBonTAA < 0 && newTBonTAA > 0)
		{
			if (numHalfRevolutions == 0)
			{
				throw sgal::Exception("Revolution counter negative");
			}

			numHalfRevolutions--;
		}

		TBonTAA = newTBonTAA;

		break;
	}
	case Negative:
	{
		double newTBonTAA = nextForcePoint.dot(
				previousAnchor - previousForcePoint);

		if (TBonTAA < 0 && newTBonTAA > 0)
		{
			numHalfRevolutions++;
		}
		else if (TBonTAA > 0 && newTBonTAA < 0)
		{
			if (numHalfRevolutions == 0)
			{
				throw sgal::Exception("Revolution counter negative");
			}

			numHalfRevolutions--;
		}

		TBonTAA = newTBonTAA;

		break;
	}
	}
}

double PulleyStateMachine::getArcAngle(const rl::math::Vector3& pointA,
		const rl::math::Vector3& pointB, rl::math::Real radius)
{
	if (state == PulleyStateMachine::NotWrapping)
	{
		return 0;
	}
	else
	{
		double arcAngle = std::ceil(numHalfRevolutions / 2.0) * 2 * M_PI;

		rl::math::Real c2 = (pointA - pointB).squaredNorm();

		if (numHalfRevolutions % 2 == 0)
		{
			arcAngle += std::acos(1 - c2 / (2 * std::pow(radius, 2)));
		}
		else
		{
			arcAngle -= std::acos(1 - c2 / (2 * std::pow(radius, 2)));
		}

		return arcAngle;
	}
}
}
}
}
}
}
}
}
