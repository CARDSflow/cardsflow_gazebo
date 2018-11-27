/*
 *  Copyright (c) 2012-2013, MYOROBOTICS consortium
 *  Author: Steffen Wittmeier
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

#ifndef _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_LINEAR_SPRING_DAMPER_H_
#define _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_LINEAR_SPRING_DAMPER_H_

#include "caliper/de.caliper.sim/ext/actuators/eccerobotMuscle/physics/ILinearSpringDamper.h"

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
class LinearSpringDamper: public ILinearSpringDamper
{
public:
	LinearSpringDamper();

	virtual ~LinearSpringDamper();

	void init(const QDomElement& element);

	void update(rl::math::Real length, rl::math::Real time);

	rl::math::Real getForce();

	rl::math::Real getLength();

	rl::math::Real getRestingLength();

	rl::math::Real getVelocity();

	void setRestingLength(rl::math::Real restingLength);

	unsigned int isInSegment();

	/**
	 * Sets the damping constant of the spring-damper
	 *
	 * @param[in] dampingConstant damping constant
	 */
	void setDampingConstant(rl::math::Real dampingConstant);

	/**
	 * Returns the damping constant.
	 *
	 * @return damping constant
	 */
	rl::math::Real getDampingConstant();

	/**
	 * Sets the spring constant
	 *
	 * @param[in] springConstant spring constant
	 */
	void setSpringConstant(rl::math::Real springConstant);

	/**
	 * Returns the spring constant
	 *
	 * @return spring constant
	 */
	rl::math::Real getSpringConstant();

	/**
	 * Turn the asymetric mode on/off.
	 * Asymetric mode means that only expansions of the spring
	 * will result in forces.
	 *
	 * @param[in] asymmetricMode true=on, false=off
	 */
	void useAsymmetricMode(bool asymmetricMode);

	/**
	 * Returns whether asymmetric is on or off.
	 */
	bool isAsymmetricMode();

protected:
	/**
	 * The resting length of the spring.
	 */
	rl::math::Real restingLength;

	/**
	 * The spring constant.
	 */
	rl::math::Real springConstant;

	/**
	 * The damping constant.
	 */
	rl::math::Real dampingConstant;

	/**
	 * The spring expansion of the previous sim step.
	 */
	rl::math::Real previousExpansion;

	/**
	 * asymetric mode on(true) or off(false)
	 */
	bool asymmetricMode;

	rl::math::Real force;

	rl::math::Real velocity;

	bool firstUpdate;

	unsigned int segment;
};
}
}
}
}
}
}
}
#endif
