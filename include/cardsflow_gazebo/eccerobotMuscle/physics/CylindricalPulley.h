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

#ifndef _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_CYLINDRICALPULLEY_H_
#define _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_CYLINDRICALPULLEY_H_

#include <sgal/physics/Body.h>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>

#include "caliper/de.caliper.sim/ext/actuators/eccerobotMuscle/physics/ICylindricalPulley.h"
#include "caliper/de.caliper.sim/ext/actuators/eccerobotMuscle/physics/IMuscle.h"
#include "PulleyStateMachine.h"
#include "AttachmentPoint.h"

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
class CylindricalPulley: public ICylindricalPulley, public AttachmentPoint
{
public:
	CylindricalPulley(sgal::physics::Body* body,
			const ::rl::math::Transform &frame, IMuscle* muscle,
			::rl::math::Real radius, rl::math::Real height,
			PulleyStateMachine::State initialState,
			unsigned int numHalfRevolutions);

	virtual ~CylindricalPulley();

	IAttachmentPoint::Type getType() const;

	void updateForcePoints(rl::math::Real time) throw (sgal::Exception);

	void applyForce(rl::math::Real time,
			rl::math::Real previousSegmentKiteLineVelocity, bool beforeSEE)
					throw (sgal::Exception);

	rl::math::Real getPreviousSegmentLength() const;

	rl::math::Vector3 getPreviousForcePointInGlobal() const;

	rl::math::Vector3 getNextForcePointInGlobal() const;

	void getTendonNodes(QVector< rl::math::Vector3 >& tendonNodes);

	ICylindricalPulley::State getState() const;

	rl::math::Real getRadius() const;

	void setRadius(rl::math::Real radius);

	rl::math::Real getHeight() const;

	void setHeight(rl::math::Real height);

	const rl::math::Vector3& getPreviousInPlane() const;

	const rl::math::Vector3& getNextInPlane() const;

	const rl::math::Vector3& getN() const;

protected:
	rl::math::Real radius;

	rl::math::Real height;

	rl::math::Real arcAngle;

	PulleyStateMachine pulleyStateMachine;

	rl::math::Vector3 N;

	rl::math::Vector3 previousForcePoint;

	rl::math::Vector3 nextForcePoint;

	rl::math::Vector3 previousForcePointInGlobal;

	rl::math::Vector3 nextForcePointInGlobal;

	rl::math::Vector3 previousAttachmentPointOrigin;

	rl::math::Vector3 nextAttachmentPointOrigin;

	rl::math::Vector3 previousAttachmentOriginInPlane;

	rl::math::Vector3 nextAttachmentOriginInPlane;

	rl::math::Vector3 previousForcePointInPlane;

	rl::math::Vector3 nextForcePointInPlane;

protected:
	double computeYCoordinate(double angle) const;
};
}
}
}
}
}
}
}
#endif
