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

#ifndef _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_ATTACHMENT_POINT_H_
#define _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_ATTACHMENT_POINT_H_

#include <rl/math/Transform.h>
#include <sgal/physics/Body.h>
#include <QVector>

#include "cardsflow_gazebo/eccerobotMuscle/physics/IAttachmentPoint.h"
#include "cardsflow_gazebo/eccerobotMuscle/physics/IMuscle.h"

class AttachmentPoint: public virtual IAttachmentPoint
{
public:
	AttachmentPoint(sgal::physics::Body* body,
			const ::rl::math::Transform &frame, IMuscle* muscle);

	virtual ~AttachmentPoint();

	IAttachmentPoint::Type getType() const;

	void updatePosition(rl::math::Real time);

	void updateForcePoints(rl::math::Real time) throw (sgal::Exception);

	void applyForce(rl::math::Real time,
			rl::math::Real previousSegmentKiteLineVelocity, bool beforeSEE)
					throw (sgal::Exception);

	rl::math::Vector3 getFrameInLocal() const;

	void setFrameInLocal(const rl::math::Vector3& frameInLocal);

	rl::math::Vector3 getFrameInGlobal() const;

	rl::math::Transform getTransformInGlobal() const;

	rl::math::Transform getTransformInLocal() const;

	sgal::physics::Body* getBody() const;

	rl::math::Real getPreviousForce() const;

	void setPreviousForce(rl::math::Real force);

	rl::math::Real getNextForce() const;

	void setNextForce(rl::math::Real force);

	void setPreviousAttachmentPoint(IAttachmentPoint* previous);

	virtual IAttachmentPoint* getPreviousAttachmentPoint() const;

	void setNextAttachmentPoint(IAttachmentPoint* next);

	virtual IAttachmentPoint* getNextAttachmentPoint() const;

	virtual rl::math::Vector3 getPreviousForcePointInGlobal() const;

	virtual rl::math::Vector3 getNextForcePointInGlobal() const;

	rl::math::Real getPreviousSegmentLength() const;

	rl::math::Real getPreviousSegmentKiteLineVelocity() const;

	void getTendonNodes(QVector< rl::math::Vector3 >& tendonNodes);

	void setUc(rl::math::Real uc);

	rl::math::Real getUc();

	void setUs(rl::math::Real us);

	rl::math::Real getUs();

	void setVs(rl::math::Real vs);

protected:
	/**
	 * The rigid body to which the attachment point belongs.
	 */
	sgal::physics::Body* body;

	/**
	 * The position of the attachment point in local coordinates.
	 */
	::rl::math::Transform frameLocal;

	/**
	 * The position of the attachment point in world coordinates.
	 */
	::rl::math::Transform frameGlobal;

	rl::math::Transform bodyFrame;

	IAttachmentPoint* previous;

	IAttachmentPoint* next;

	IMuscle* muscle;

	rl::math::Real uc;

	rl::math::Real us;

	rl::math::Real vs;

	rl::math::Real fa;

	rl::math::Real fb;

	rl::math::Vector3 Fa;

	rl::math::Vector3 Fb;

	rl::math::Vector3 A;

	rl::math::Vector3 B;

	rl::math::Real previousSegmentKiteLineVelocity;
};

#endif
