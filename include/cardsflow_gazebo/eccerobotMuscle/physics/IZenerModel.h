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

#include "ISeriesElasticElement.h"

/**
 * \brief Interface class of a Zener (or standard linear solid) model.
 *
 * \par Zener or standard linear solid model
 * \f$\dot{\sigma}_Z + \frac{E_1}{\mu}\sigma_Z = \dot{\varepsilon}_Z(E_0+E_1) +\frac{E_0E_1}{\mu}\varepsilon_Z \f$ with
 * - \f$ \sigma_Z \f$ Stress.
 * - \f$ \varepsilon_Z \f$ Engineering strain.
 * - \f$ \mu \f$ Viscocity of the dashpot.
 * - \f$ E_0 \f$ Young's modulus of the parallel spring.
 * - \f$ E_1 \f$ Young's modulus of the Maxwell spring.
 *
 * \par XML-Definition of a Zener Model
 *
 * Zener models can be instantiated for a muscle by adding the following XML
 * definition to a muscle-tag.
 *
 * \include de.caliper.sim/zener.xml
 */
class IZenerModel: public ISeriesElasticElement
{
public:
	/**
	 * \brief Destructor.
	 */
	virtual ~IZenerModel()
	{

	}

	/**
	 * \brief Sets the cross section area.
	 *
	 * \param A0 Cross section area in [m^2].
	 */
	virtual void setA0(rl::math::Real A0) = 0;

	/**
	 * \brief Returns the cross section area.
	 *
	 * \return Cross section area in [m^2].
	 */
	virtual rl::math::Real getA0() = 0;

	/**
	 * \brief Sets the free spring modulus.
	 *
	 * \param E0 Free spring modulus in [Pa].
	 */
	virtual void setE0(rl::math::Real E0) = 0;

	/**
	 * \brief Returns the free spring modulus.
	 *
	 * \return Free spring modulus in [Pa].
	 */
	virtual rl::math::Real getE0() = 0;

	/**
	 * \brief Sets the Maxwell spring modulus.
	 *
	 * \param E1 Maxwell spring modulus in [Pa].
	 */
	virtual void setE1(rl::math::Real E1) = 0;

	/**
	 * \brief Returns the Maxwell spring modulus.
	 *
	 * \return Maxwell spring modulus in [Pa].
	 */
	virtual rl::math::Real getE1() = 0;

	/**
	 * \brief Sets the Maxwell viscosity.
	 *
	 * \param mu1 Maxwell viscosity in [kg/ms].
	 */
	virtual void setMu1(rl::math::Real mu1) =0;

	/**
	 * \brief Returns the Maxwell viscosity.
	 *
	 * \return Maxwell viscosity in [kg/ms].
	 */
	virtual rl::math::Real getMu1() = 0;
};
