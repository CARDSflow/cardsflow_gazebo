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

#ifndef _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_SPHERICALPULLEY_H_
#define _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_SPHERICALPULLEY_H_

#include <sgal/physics/Body.h>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>

#include "caliper/de.caliper.sim/ext/actuators/eccerobotMuscle/physics/ISphericalPulley.h"
#include "AttachmentPoint.h"
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
class SphericalPulley: public virtual ISphericalPulley, public AttachmentPoint
{
public:
	SphericalPulley(sgal::physics::Body* body,
			const ::rl::math::Transform &frame, IMuscle* muscle,
			::rl::math::Real radius, PulleyStateMachine::State initialState,
			unsigned int numHalfRevolutions);

	virtual ~SphericalPulley();

	IAttachmentPoint::Type getType() const;

	void updateForcePoints(rl::math::Real time) throw (sgal::Exception);

	void applyForce(rl::math::Real time,
			rl::math::Real previousSegmentKiteLineVelocity, bool beforeSEE)
					throw (sgal::Exception);

	virtual rl::math::Vector3 getPreviousForcePointInGlobal() const;

	virtual rl::math::Vector3 getNextForcePointInGlobal() const;

	rl::math::Real getPreviousSegmentLength() const;

	ISphericalPulley::State getState() const;

	const rl::math::Vector3& getN() const;

	const rl::math::Vector3& getTangentPointA1() const;

	const rl::math::Vector3& getTangentPointB1() const;

	const rl::math::Vector3& getTangentPointA2() const;

	const rl::math::Vector3& getTangentPointB2() const;

	rl::math::Real getRadius() const;

	void setRadius(rl::math::Real radius);

	void getTendonNodes(QVector< rl::math::Vector3 >& tendonNodes);

protected:
	PulleyStateMachine pulleyStateMachine;

	rl::math::Vector3 tangentPointA1;

	rl::math::Vector3 tangentPointA2;

	rl::math::Vector3 tangentPointB1;

	rl::math::Vector3 tangentPointB2;

	rl::math::Vector3 previousForcePoint;

	rl::math::Vector3 nextForcePoint;

	rl::math::Vector3 previousForcePointInGlobal;

	rl::math::Vector3 nextForcePointInGlobal;

	rl::math::Vector3 previousAttachmentPointOrigin;

	rl::math::Vector3 nextAttachmentPointOrigin;

	rl::math::Vector3 N;

	rl::math::Real radius;

	rl::math::Real arcAngle;
};
}
}
}
}
}
}
}
#endif
