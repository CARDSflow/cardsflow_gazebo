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

#include <QMutexLocker>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>

#include "Muscle.h"
#include "Actuator.h"
#include "AttachmentPoint.h"
#include "SphericalPulley.h"
#include "CylindricalPulley.h"
#include "MeshPulley.h"
#include "PulleyStateMachine.h"
#include "LinearSpringDamper.h"
#include "ZenerModel.h"
#include "../ParseException.h"

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

Muscle::Muscle(sgal::physics::Model* model) :
		IMuscle(model), actuator(0), see(0), kiteLength(0.0), length(0.0), previousLength(
				0.0), firstTick(true), mu(0.0), velocity(0.0), force(0.0), forceSensorSegment(
				1), initialKiteLength(0.0)
{
}

Muscle::~Muscle()
{
	for (int i = 0; i < attachmentPoints.size(); i++)
	{
		delete attachmentPoints[i];
	}

	delete see;

	delete actuator;
}

void Muscle::init(const QDomElement& element)
{
	name = element.attribute("name").toStdString();

	QDomElement childElement;

	childElement = element.firstChildElement("attachments");

	if (!childElement.isNull())
	{
		for (QDomElement attachmentElement = childElement.firstChildElement();
				!attachmentElement.isNull(); attachmentElement =
						attachmentElement.nextSiblingElement())
		{
			QString bodyName = attachmentElement.attribute("body");

			QDomElement translateElement = attachmentElement.firstChildElement(
					"translate");

			rl::math::Transform frame;

			frame.setIdentity();

			frame.translation() = rl::math::Vector3(
					translateElement.attribute("x").toDouble(),
					translateElement.attribute("y").toDouble(),
					translateElement.attribute("z").toDouble());

			QDomElement rotationElement = attachmentElement.firstChildElement(
					"rotate");

			if (!rotationElement.isNull())
			{
				rl::math::AngleAxis angleAxis(
						rotationElement.attribute("angle").toDouble()
								* rl::math::DEG2RAD,
						rl::math::Vector3(
								rotationElement.attribute("x").toDouble(),
								rotationElement.attribute("y").toDouble(),
								rotationElement.attribute("z").toDouble()));

				frame.rotate(angleAxis);
			}

			AttachmentPoint * attachmentPoint = 0;

			for (std::size_t i = 0; i < model->getNumBodies(); i++)
			{
				if (model->getBody(i)->getName() == bodyName.toStdString())
				{
					::rl::math::Transform tmpTransform;

					dynamic_cast< sgal::physics::Body * >(model->getBody(i))->getPrincipalTransform(
							tmpTransform);

					frame = tmpTransform.inverse() * frame;

					if (attachmentElement.tagName() == QString("pulley"))
					{
						// spherical or cylindrical pulley
						if (attachmentElement.hasAttribute("radius"))
						{
							rl::math::Real radius = attachmentElement.attribute(
									"radius").toDouble();

							PulleyStateMachine::State initialState;

							QString initialStateValue =
									attachmentElement.attribute("initialState");

							if (initialStateValue == QString("NotWrapping"))
							{
								initialState = PulleyStateMachine::NotWrapping;
							}
							else if (initialStateValue == "Positive")
							{
								initialState = PulleyStateMachine::Positive;
							}
							else if (initialStateValue == "Negative")
							{
								initialState = PulleyStateMachine::Negative;
							}

							if (attachmentElement.hasAttribute("height"))
							{
								double height = attachmentElement.attribute(
										"height").toDouble();

								attachmentPoint =
										new CylindricalPulley(
												dynamic_cast< sgal::physics::Body* >(model->getBody(
														i)), frame, this,
												radius, height, initialState,
												0);
							}
							else
							{
								attachmentPoint =
										new SphericalPulley(
												dynamic_cast< sgal::physics::Body* >(model->getBody(
														i)), frame, this,
												radius, initialState, 0);
							}

							break;
						}
						// mesh pulley
						else
						{
							attachmentPoint =
									new MeshPulley(
											dynamic_cast< sgal::physics::Body* >(model->getBody(
													i)), this,
											attachmentElement, rootPath);
						}
					}
					else
					{
						attachmentPoint =
								new AttachmentPoint(
										dynamic_cast< sgal::physics::Body* >(model->getBody(
												i)), frame, this);

						if (attachmentElement.hasAttribute("uc"))
						{
							attachmentPoint->setUc(
									attachmentElement.attribute("uc").toDouble());
						}

						if (attachmentElement.hasAttribute("us"))
						{
							attachmentPoint->setUs(
									attachmentElement.attribute("us").toDouble());
						}

						if (attachmentElement.hasAttribute("vs"))
						{
							attachmentPoint->setVs(
									attachmentElement.attribute("vs").toDouble());
						}

						break;
					}
				}
			}

			if (attachmentPoint == 0)
			{
				throw ParseException("no attachment body found for body ");
			}

			attachmentPoints.push_back(attachmentPoint);

			if (attachmentPoints.size() > 1)
			{
				attachmentPoints[attachmentPoints.size() - 1]->setPreviousAttachmentPoint(
						attachmentPoints[attachmentPoints.size() - 2]);

				attachmentPoints[attachmentPoints.size() - 2]->setNextAttachmentPoint(
						attachmentPoints[attachmentPoints.size() - 1]);
			}
		}

		for (int i = 0; i < attachmentPoints.size(); i++)
		{
			attachmentPoints[i]->updatePosition(0.0);
		}

		for (int i = 0; i < attachmentPoints.size(); i++)
		{
			attachmentPoints[i]->updateForcePoints(0.0);
		}
	}
	else
	{
		throw ParseException("no attachments defined");
	}

	childElement = element.firstChildElement("kiteLine");

	if (!childElement.isNull())
	{
		QDomElement initialLengthElement = childElement.firstChildElement(
				"initialLength");

		if (!initialLengthElement.isNull())
		{
			kiteLength = initialKiteLength =
					initialLengthElement.text().toDouble();
		}
		else
		{
			throw ParseException("no initialLengthElement element");
		}
	}
	else
	{
		throw ParseException("no kiteLine element");
	}

	childElement = element.firstChildElement("forceSensor");

	if (!childElement.isNull())
	{
		forceSensorSegment = childElement.attribute("inSegment", "1").toUInt();
	}
	else
	{
		throw ParseException("no forceSensor element");
	}

	childElement = element.firstChildElement("linearSpringDamper");

	if (!childElement.isNull())
	{
		see = new LinearSpringDamper();

		see->init(childElement);
	}

	childElement = element.firstChildElement("zenerModel");

	if (!childElement.isNull())
	{
		see = new ZenerModel();

		see->init(childElement);
	}

	childElement = element.firstChildElement("actuator");

	if (!childElement.isNull())
	{
		actuator = new Actuator();

		actuator->init(childElement);
	}

	childElement = element.firstChildElement("damping");

	if (!childElement.isNull())
	{
		mu = childElement.text().toDouble();
	}

	if (see == 0)
	{
		throw ParseException("No series elastic element defined.");
	}

	if (see->isInSegment() >= attachmentPoints.size() - 1)
	{
		throw ParseException(
				"Series elastic element segment index out of range");
	}

	if (forceSensorSegment >= attachmentPoints.size() - 1)
	{
		throw ParseException("Force sensor segment index out of range");
	}
}

void Muscle::setRootPath(const QString& rootPath)
{
	this->rootPath = rootPath;
}

void Muscle::preTickCallback(rl::math::Real time)
{
#ifdef _DO_PROFILE_
	preTickTimer.start();
#endif

	// update world coordinates of attachment points.
	for (int i = 0; i < attachmentPoints.size(); i++)
	{
		attachmentPoints[i]->updatePosition(time);
	}

	// update force points of attachments
	for (int i = 0; i < attachmentPoints.size(); i++)
	{
		try
		{
			attachmentPoints[i]->updateForcePoints(time);
		} catch (sgal::Exception& ex)
		{
			std::cout << name << ", " << ex.what() << std::endl;

			throw;
		}
	}

	// compute length of muscle
	double length = 0.0;

	for (int i = 1; i < attachmentPoints.size(); i++)
	{
		length += attachmentPoints[i]->getPreviousSegmentLength();
	}

	{
		QMutexLocker locker(&mutex);

		this->length = length;
	}

	// compute velocity of muscle
	if (firstTick)
	{
		previousLength = length;

		firstTick = false;
	}

	//std::cout << name << ", length=" << length - see->getRestingLength() << std::endl;

	{
		QMutexLocker locker(&mutex);

		velocity = (length - previousLength) / time;
	}

	// compute force of spring
	if (see)
	{
		see->update(length - kiteLength, time);

		// propagate force through tendon path (friction!!)
		attachmentPoints[see->isInSegment()]->setNextForce(see->getForce());

		attachmentPoints[see->isInSegment() + 1]->setPreviousForce(
				see->getForce());

		// propagate to ref attachment (motor)
		for (int i = see->isInSegment(); i >= 0; i--)
		{
			if (actuator)
			{
				attachmentPoints[i]->applyForce(time,
						actuator->getLinearVelocity(), true);
			}
			else
			{
				attachmentPoints[i]->applyForce(time, 0, true);
			}
		}

		// propagate to attachment
		for (int i = see->isInSegment() + 1; i < attachmentPoints.size(); i++)
		{
			if (actuator)
			{
				attachmentPoints[i]->applyForce(time,
						actuator->getLinearVelocity() + see->getVelocity(),
						false);
			}
			else
			{
				attachmentPoints[i]->applyForce(time, see->getVelocity(),
						false);
			}
		}

		{
			QMutexLocker locker(&mutex);

			force = attachmentPoints[forceSensorSegment]->getNextForce();
		}

#ifdef DEBUG
		std::cout << name << ", length=" << length << ", SEE force="
		<< see->getForce() << ", Sensor force=" << force
		<< ", actuatorVel=";

		if (actuator)
		{
			std::cout << actuator->getLinearVelocity();
		}
		else
		{
			std::cout << "0.0";
		}

		std::cout << ", SEE-vel=" << see->getVelocity() << "\n";

		for (int i = 0; i < attachmentPoints.size(); i++)
		{
			if (i == 0)
			{
				std::cout << "\tSegment=" << i << "\t"
				<< attachmentPoints[i]->getNextForce() << " - ";
			}
			else if (i == attachmentPoints.size() - 1)
			{
				std::cout << attachmentPoints[i]->getPreviousForce()
				<< std::endl;
			}
			else
			{
				std::cout << attachmentPoints[i]->getPreviousForce()
				<< std::endl << "\tSegment=" << i << "\t"
				<< attachmentPoints[i]->getNextForce() << " - ";
			}
		}

		for (int i = 0; i < attachmentPoints.size(); i++)
		{
			std::cout << "\tAttachmentPointKiteLineVelocity=" << i << "\t"
			<< attachmentPoints[i]->getPreviousSegmentKiteLineVelocity()
			<< " m/s" << std::endl;
		}
#endif

		if (actuator)
		{
			actuator->setLoadTorque(
					actuator->getSpindleRadius()
							* attachmentPoints[0]->getNextForce());

			actuator->step(time);

			QMutexLocker locker(&mutex);

			kiteLength = initialKiteLength
					- (actuator->getPosition() - actuator->getInitialPosition())
							* rl::math::DEG2RAD * actuator->getSpindleRadius();
		}
	}

	previousLength = length;

#ifdef _DO_PROFILE_
	preTickTimer.stop();

	preTickTimes.addValue(preTickTimer.elapsed());
#endif
}

void Muscle::postTickCallback(rl::math::Real timeStep)
{

}

void Muscle::setInitialKiteLength(rl::math::Real initialKiteLength)
{
	QMutexLocker locker(&mutex);

	this->initialKiteLength = initialKiteLength;
}

rl::math::Real Muscle::getInitialKiteLength()
{
	QMutexLocker locker(&mutex);

	return initialKiteLength;
}

rl::math::Real Muscle::getKiteLength()
{
	QMutexLocker locker(&mutex);

	return kiteLength;
}

rl::math::Real Muscle::getLength()
{
	QMutexLocker locker(&mutex);

	return length;
}

rl::math::Real Muscle::getVelocity()
{
	QMutexLocker locker(&mutex);

	return velocity;
}

rl::math::Real Muscle::getForce()
{
	QMutexLocker locker(&mutex);

	return force;
}

rl::math::Real Muscle::getAppliedForce()
{
	return attachmentPoints[attachmentPoints.size() - 1]->getPreviousForce();
}

rl::math::Real Muscle::getMaxSegmentForce()
{
	QMutexLocker locker(&mutex);

	rl::math::Real maxSegmentForce = 0.0;

	for (int i = 1; i < attachmentPoints.size(); i++)
	{
		if (attachmentPoints[i]->getPreviousForce() > maxSegmentForce)
		{
			maxSegmentForce = attachmentPoints[i]->getPreviousForce();
		}
	}

	return maxSegmentForce;
}

const ::std::size_t Muscle::getNumAttachments() const
{
	return attachmentPoints.size();
}

IAttachmentPoint* Muscle::getAttachment(std::size_t i)
{
	if (i >= 0 && i < attachmentPoints.size())
	{
		return attachmentPoints[i];
	}

	return 0;
}

ISeriesElasticElement* Muscle::getSeriesElasticElement()
{
	return see;
}

IActuator* Muscle::getActuator()
{
	return actuator;
}

unsigned int Muscle::getForceSensorSegment()
{
	return forceSensorSegment;
}
}
}
}
}
}
}
}
