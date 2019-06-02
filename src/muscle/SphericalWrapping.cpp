#include "cardsflow_gazebo/muscle/SphericalWrapping.hpp"

namespace cardsflow_gazebo {

    SphericalWrapping::SphericalWrapping() : IViaPoints(gazebo::math::Vector3d(0, 0, 0), Type::SPHERICAL, nullptr),
                                             stateMachine(StateMachine()),
                                             radius(0), prevCoord(gazebo::math::Vector3d(0, 0, 0)),
                                             nextCoord(gazebo::math::Vector3d(0, 0, 0)),
                                             normal(gazebo::math::Vector3d(0, 0, 0)), arcAngle(0) {

    };

    SphericalWrapping::SphericalWrapping(gazebo::math::Vector3d point, physics::LinkPtr link) : SphericalWrapping() {
        localCoordinates = point;
        this->link = link;
    };

    SphericalWrapping::SphericalWrapping(gazebo::math::Vector3d point, double radius, int state, int counter,
                                         physics::LinkPtr link) : SphericalWrapping(point, link) {
        this->radius = radius;
        this->stateMachine.state = (StateMachine::State) state;
        this->stateMachine.revCounter = counter;
    };

    void SphericalWrapping::UpdateForcePoints() {
        prevCoord = prevPoint->globalCoordinates;
        nextCoord = nextPoint->globalCoordinates;

        stateMachine.UpdateState(prevCoord, nextCoord, globalCoordinates, radius);

        //if muscle is not wrapping, use straight line calculation
        if (stateMachine.state == StateMachine::NOTWRAPPING) {
            prevForcePoint = nextCoord;
            nextForcePoint = prevCoord;
            previousSegmentLength = 0;
            return;
        } else {

            //compute tangent points
            //compute unit vectors and according length
            double l_j1 = (prevCoord - this->globalCoordinates).Length();
            gazebo::math::Vector3d j1 = (prevCoord - this->globalCoordinates) / l_j1;
            double l_j2 = (nextCoord - this->globalCoordinates).Length();
            gazebo::math::Vector3d j2 = (nextCoord - this->globalCoordinates) / l_j2;

            //compute normal,
            gazebo::math::Vector3d normal_temp = j1.Cross(j2);
            normal = normal_temp / normal_temp.Length();

            //compute k1, k2
            gazebo::math::Vector3d k1 = j1.Cross(normal);
            k1 = k1 / k1.Length();
            gazebo::math::Vector3d k2 = normal.Cross(j2);
            k2 = k2 / k2.Length();

            //compute length of a1, a2, b1, b2
            double a1 = radius * radius / l_j1;
            double a2 = radius * radius / l_j2;
            double b1 = sqrt(radius * radius - a1 * a1);
            double b2 = sqrt(radius * radius - a2 * a2);

            if (stateMachine.state == StateMachine::POSITIVE) {
                this->prevForcePoint = this->globalCoordinates + a1 * j1 - b1 * k1;
                this->nextForcePoint = this->globalCoordinates + a2 * j2 - b2 * k2;
            } else if (stateMachine.state == StateMachine::NEGATIVE) {
                this->prevForcePoint = this->globalCoordinates + a1 * j1 + b1 * k1;
                this->nextForcePoint = this->globalCoordinates + a2 * j2 + b2 * k2;
            }

            //update revolution counter
            double projection = (prevCoord - this->prevForcePoint).Dot(this->nextForcePoint - this->prevForcePoint);
            stateMachine.UpdateRevCounter(projection);

            //calculate the wrapping angle
            double angle = acos(
                    1 - (pow((this->prevForcePoint - this->nextForcePoint).Length(), 2) / (2 * radius * radius)));
            arcAngle = 2 * (boost::math::constants::pi<double>()) * ceil(stateMachine.revCounter / 2);
            arcAngle += (stateMachine.revCounter % 2 == 0) ? (angle) : (-angle);

            //calculate the lines of action and the muscle's length
            previousSegmentLength = (prevCoord - this->prevForcePoint).Length() + arcAngle * radius;
        }
    };

    void SphericalWrapping::CalculateForce() {
        if (stateMachine.state == StateMachine::NOTWRAPPING) {
            prevForce = 0;
            nextForce = 0;
            return;
        }
        if (prevPoint && nextPoint) {
            //TODO: change fa, fb with respect to friction
            // need to know if before or behind see
            //previousSegmentKiteLineVelocity needed
        }

        if (prevPoint) {
            gazebo::math::Vector3d A = prevPoint->nextForcePoint - this->prevForcePoint;
            prevForce = A / A.Length() * fa;
            //link->AddForceAtRelativePosition(Fa, this->prevForcePoint);
        } else if (nextPoint) {
            gazebo::math::Vector3d B = nextPoint->prevForcePoint - this->nextForcePoint;
            nextForce = B / B.Length() * fb;
            //link->AddForceAtRelativePosition(Fb, this->nextForcePoint);
        }
    };

}