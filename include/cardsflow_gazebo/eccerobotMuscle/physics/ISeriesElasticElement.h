/*
 *  Copyright (c) 2013, MYOROBOTICS consortium
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


//#include <QDomElement>
#include <rl/math/Real.h>
#include <rl/math/Vector.h>

/**
 * \brief Base class for series elastic elements (SEEs).
 */
class ISeriesElasticElement
{
public:
	/**
	 * \brief Destructor.
	 */
	virtual ~ISeriesElasticElement()
	{

	}

	/**
	 * \brief Initializes the SEE object.
	 *
	 * \param element QDomElement representing the root of the SEE definition
	 * in the muscle XML file.
	 */
//	virtual void init(const QDomElement& element) = 0;

	/**
	 * \brief Updates the SEE force.
	 *
	 * \param[in] length Length \f$l_S\f$ of the SEE in [m].
	 * \param[in] time Simulation time step in [s].
	 */
	virtual void update(rl::math::Real length, rl::math::Real time) = 0;

	/**
	 * \brief Returns the SEE force.
	 *
	 * \return SEE force in [N].
	 */
	virtual rl::math::Real getForce() = 0;

	/**
	 * \brief Returns the SEE length.
	 *
	 * \return SEE length in [N].
	 */
	virtual rl::math::Real getLength() = 0;

	/**
	 * \brief Returns the SEE velocity.
	 *
	 * \return SEE velocity in [m/s].
	 */
	virtual rl::math::Real getVelocity() = 0;

	/**
	 * \brief Returns the resting length (at zero force) of the SEE.
	 *
	 * \return Resting length in [m].
	 */
	virtual rl::math::Real getRestingLength() = 0;

	/**
	 * \brief Sets the resting length of the SEE.
	 *
	 * \param restingLength Resting length in [m].
	 */
	virtual void setRestingLength(rl::math::Real restingLength) = 0;

	/**
	 * \brief Returns the muscle segment index of the SEE.
	 *
	 * \return Muscle segment index of the SEE.
	 */
	virtual unsigned int isInSegment() = 0;
};

