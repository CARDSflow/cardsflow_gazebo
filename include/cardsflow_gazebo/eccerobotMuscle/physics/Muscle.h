/*
 *  Copyright (c) 2012-2013, MYOROBOTICS consortium
 *  Author: Michael Jaentsch
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

#ifndef _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_MUSCLE_H_
#define _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_MUSCLE_H_

#include <QMutex>
#include "caliper/de.caliper.sim/ext/actuators/eccerobotMuscle/physics/IMuscle.h"

#ifdef _DO_PROFILE_
#include "../test/BenchmarkerData.h"
#include <rl/util/Timer.h>
#endif

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
class Muscle: public IMuscle
{
public:
	Muscle(sgal::physics::Model* model);

	~Muscle();

	void init(const QDomElement& element);

	void setRootPath(const QString& rootPath);

	void preTickCallback(rl::math::Real timeStep);

	void postTickCallback(rl::math::Real timeStep);

	void setInitialKiteLength(rl::math::Real initialKiteLength);

	rl::math::Real getInitialKiteLength();

	rl::math::Real getKiteLength();

	rl::math::Real getLength();

	rl::math::Real getVelocity();

	rl::math::Real getForce();

	rl::math::Real getAppliedForce();

	rl::math::Real getMaxSegmentForce();

	const ::std::size_t getNumAttachments() const;

	IAttachmentPoint* getAttachment(std::size_t i);

	ISeriesElasticElement* getSeriesElasticElement();

	IActuator* getActuator();

	unsigned int getForceSensorSegment();

protected:
	QVector< IAttachmentPoint* > attachmentPoints;

	bool firstTick;

	rl::math::Real kiteLength;

	rl::math::Real initialKiteLength;

	rl::math::Real length;

	rl::math::Real previousLength;

	rl::math::Real velocity;

	rl::math::Real mu;

	rl::math::Real force;

	IActuator* actuator;

	ISeriesElasticElement* see;

	unsigned int forceSensorSegment;

	QMutex mutex;

	QString rootPath;

#ifdef _DO_PROFILE_
	BenchmarkerData preTickTimes;

	rl::util::Timer preTickTimer;
#endif
};
}
}
}
}
}
}
}

#endif
