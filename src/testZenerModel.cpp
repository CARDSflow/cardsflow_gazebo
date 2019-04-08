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

#include "cardsflow_gazebo/eccerobotMuscle/physics/ZenerModel.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

int main(int argc, char** argv)
{
	if (argc == 1)
	{
		std::cout << "Usage: testZenerModel [timeStep]" << std::endl;

		return 0;
	}

	ZenerModel zenerModel;

	zenerModel.setRestingLength(0.01);
	zenerModel.setA0(0.005 * 0.005 / 4 * M_PI);
	zenerModel.setE0(0.694e6);
	zenerModel.setE1(0.2116e6);
	zenerModel.setMu1(277788.48);

	std::ofstream file;

	file.open("zenerModel.txt");

	double timeStep = atof(argv[1]);

//	rl::util::Timer timer;

	std::vector<double> times;
	std::vector<double> stress;

//	timer.start();

	double strain = 0.0;

	for (double time = 0; time < 10.0; time += timeStep)
	{
		if (time < 1.0)
		{
			strain = 0.0;
		}
		else if (time > 1.0 && time < 6.0)
		{
			strain = 0.5;
		}
		else
		{
			strain = 0.0;
		}

		zenerModel.update2(strain, timeStep);

		times.push_back(time);

		stress.push_back(zenerModel.getForce() / zenerModel.getA0());
	}

//	timer.stop();

//	std::cout << "Simulation took=" << timer.elapsed() << std::endl;

	for (int i = 0; i < times.size(); i++)
	{
		file << times[i] << "\t" << stress[i] << std::endl;
	}

	file.close();

	return 0;
}
