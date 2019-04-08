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

#ifndef _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_IMUSCLE_H_
#define _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_IMUSCLE_H_

#include <sgal/physics/Extension.h>
#include <QVector>

#include "IAttachmentPoint.h"
#include "IActuator.h"
#include "ISeriesElasticElement.h"

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
/**
 * \brief Physics implementation of a muscle.
 *
 * Physics implementation of a muscle as described in \cite Wittmeier2013b
 */
class IMuscle: public sgal::physics::Extension
{
public:
	/**
	 * \brief Constructor.
	 *
	 * \param model The sgal::physics::Model object to which the muscle will be added.
	 */
	IMuscle(sgal::physics::Model* model) :
			sgal::physics::Extension(model)
	{

	}

	/**
	 * \brief Destructor.
	 */
	virtual ~IMuscle()
	{

	}

	/**
	 * \brief Initializes the muscle object.
	 *
	 * \param element QDomElement representing the root of the muscle definition
	 * in the muscle XML file.
	 */
	virtual void init(const QDomElement& element) = 0;

	/**
	 * \brief Sets the root path for relative paths within the XML file.
	 *
	 * In case a relative path is used within the muscle XML file (e.g. to
	 * reference a mesh), this root path is used as a prefix to derive
	 * the absolute path.
	 *
	 * \param rootPath Root path.
	 */
	virtual void setRootPath(const QString& rootPath) = 0;

	/**
	 * \brief Pre-tick callback function.
	 */
	virtual void preTickCallback(rl::math::Real timeStep) = 0;

	/**
	 * \brief Post-tick callback function.
	 */
	virtual void postTickCallback(rl::math::Real timeStep) = 0;

	/**
	 * \brief Sets the initial kite length \f$l_{T_0}\f$ of the muscle.
	 *
	 * \param initialKiteLength Initial kite length in [m].
	 */
	virtual void setInitialKiteLength(rl::math::Real initialKiteLength) = 0;

	/**
	 * \brief Returns the initial kite length \f$l_{T_0}\f$ of the muscle in [m].
	 */
	virtual rl::math::Real getInitialKiteLength() = 0;

	/**
	 * \brief Returns the kite length \f$l_T\f$.
	 *
	 * \return Kite length \f$ l_T \f$ in [m].
	 */
	virtual rl::math::Real getKiteLength() = 0;

	/**
	 * \brief Returns the muscle length \f$l_M\f$.
	 *
	 * @return Muscle length in [m].
	 */
	virtual rl::math::Real getLength() = 0;

	/**
	 * \brief Returns the muscle velocity \f$v_M\f$.
	 *
	 * \return Muscle velocity in [m/s].
	 */
	virtual rl::math::Real getVelocity() = 0;

	/**
	 * \brief Returns the muscle force sensor value.
	 *
	 * The position of the force sensor is given by the force sensor segment.
	 *
	 * \return Muscle force in [N].
	 */
	virtual rl::math::Real getForce() = 0;

	/**
	 * \brief Returns the force of the last muscle segment before the insertion.
	 *
	 * \return Muscle force in [N].
	 */
	virtual rl::math::Real getAppliedForce() = 0;

	/**
	 * \brief Returns the highest force of the muscle.
	 *
	 * When muscle friction is modeled, the force varies with each muscle segment.
	 * This method returns the highest segment force.
	 *
	 * \return Highest muscle segment force in [N].
	 */
	virtual rl::math::Real getMaxSegmentForce() = 0;

	/**
	 * \brief Returns the number of attachments (obstacles) of the muscle.
	 *
	 * \return Number of attachments (obstacles).
	 */
	virtual const ::std::size_t getNumAttachments() const = 0;

	/**
	 * \brief Returns an attachment (obstacle).
	 *
	 * \return Attachment (obstacle).
	 */
	virtual IAttachmentPoint* getAttachment(std::size_t i) = 0;

	/**
	 * \brief Returns the series elastic element.
	 *
	 * \return Series elastic element.
	 */
	virtual ISeriesElasticElement* getSeriesElasticElement() = 0;

	/**
	 * \brief Returns the actuator (DC motor + gear + spindle).
	 *
	 * \return Actuator.
	 */
	virtual IActuator* getActuator() = 0;

	/**
	 * \brief Returns the segment index of the force sensor.
	 *
	 * \return Force sensor segment index.
	 */
	virtual unsigned int getForceSensorSegment() = 0;
};
}
}
}
}
}
}
}

#endif
