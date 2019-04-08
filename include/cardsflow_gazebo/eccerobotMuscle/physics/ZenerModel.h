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


#include "cardsflow_gazebo/eccerobotMuscle/physics/IZenerModel.h"

class ZenerModel: public IZenerModel
{
public:
	ZenerModel();

	virtual ~ZenerModel();

//	void init(const QDomElement& element);

	void update(rl::math::Real length, rl::math::Real time);

	unsigned int isInSegment();

	void update2(rl::math::Real strain, rl::math::Real dt);

	rl::math::Real getForce();

	rl::math::Real getLength();

	rl::math::Real getVelocity();

	rl::math::Real getRestingLength();

	void setRestingLength(rl::math::Real restingLength);

	void setA0(rl::math::Real A0);

	rl::math::Real getA0();

	void setE0(rl::math::Real E0);

	rl::math::Real getE0();

	void setE1(rl::math::Real E1);

	rl::math::Real getE1();

	void setMu1(rl::math::Real mu1);

	rl::math::Real getMu1();

protected:
	rl::math::Real L0;

	rl::math::Real A0;

	rl::math::Real E0;

	rl::math::Real E1;

	rl::math::Real mu1;

	rl::math::Real y;

	rl::math::Real strain;

	rl::math::Real previousStrain;

	rl::math::Real force;

	rl::math::Real velocity;

	rl::math::Real k1;

	rl::math::Real k2;

	rl::math::Real k3;

	rl::math::Real k4;

	rl::math::Real dt;

	rl::math::Real dtCorrected;

	unsigned int steps;

	unsigned int segment;

	bool firstUpdate;
};
