#include "cardsflow_gazebo/muscle/CylindricalWrapping.hpp"
#include "cardsflow_gazebo/muscle/StateMachine.hpp"

namespace cardsflow_gazebo {

    CylindricalWrapping::CylindricalWrapping() : IViaPoints(ignition::math::Vector3d(0, 0, 0), Type::CYLINDRICAL, nullptr),
                                                 stateMachine(StateMachine()),
                                                 radius(0), prevCoord(ignition::math::Vector3d(0, 0, 0)),
                                                 nextCoord(ignition::math::Vector3d(0, 0, 0)),
                                                 prevCoordPlane(ignition::math::Vector3d(0, 0, 0)),
                                                 nextCoordPlane(ignition::math::Vector3d(0, 0, 0)),
                                                 prevForcePointPlane(ignition::math::Vector3d(0, 0, 0)),
                                                 nextForcePointPlane(ignition::math::Vector3d(0, 0, 0)),
                                                 normal(ignition::math::Vector3d(0, 0, 0)), arcAngle(0) {

    };

    CylindricalWrapping::CylindricalWrapping(ignition::math::Vector3d point, physics::LinkPtr link) : CylindricalWrapping() {
        localCoordinates = point;
        this->link = link;
    };

    CylindricalWrapping::CylindricalWrapping(ignition::math::Vector3d point, double radius, int state, int counter,
                                             physics::LinkPtr link) : CylindricalWrapping(point, link) {
        this->radius = radius;
        this->stateMachine.state = (StateMachine::State) state;
        this->stateMachine.revCounter = counter;
    };

    void CylindricalWrapping::UpdateForcePoints() {
        prevCoord = prevPoint->globalCoordinates;
        nextCoord = nextPoint->globalCoordinates;

        //calculate normal onto plane
        ignition::math::Vector3d unit_normal = linkRotation.RotateVector(ignition::math::Vector3d(0, 0, 1));
        unit_normal = unit_normal / unit_normal.Length();
        //project insertion and fixation point onto xy plane of the cylinder
        double prevDist = (prevCoord - globalCoordinates).Dot(unit_normal);
        double nextDist = (nextCoord - globalCoordinates).Dot(unit_normal);
        prevCoordPlane = prevCoord - (prevDist) * unit_normal;
        nextCoordPlane = nextCoord - (nextDist) * unit_normal;

        stateMachine.UpdateState(prevCoordPlane, nextCoordPlane, globalCoordinates, radius);

        //if muscle is not wrapping, use straight line calculation
        if (stateMachine.state == StateMachine::NOTWRAPPING) {
            prevForcePoint = nextCoord;
            nextForcePoint = prevCoord;
            previousSegmentLength = 0;
            return;
        }

        //compute tangent points
        //compute unit vectors and according length
        double l_j1 = (prevCoordPlane - this->globalCoordinates).Length();
        ignition::math::Vector3d j1 = (prevCoordPlane - this->globalCoordinates) / l_j1;
        double l_j2 = (nextCoordPlane - this->globalCoordinates).Length();
        ignition::math::Vector3d j2 = (nextCoordPlane - this->globalCoordinates) / l_j2;

        //compute normal,
        normal = j1.Cross(j2);

        //compute k1, k2
        ignition::math::Vector3d k1 = j1.Cross(normal);
        k1 = k1 / k1.Length();
        ignition::math::Vector3d k2 = normal.Cross(j2);
        k2 = k2 / k2.Length();

        //compute length of a1, a2, b1, b2
        double a1 = radius * radius / l_j1;
        double a2 = radius * radius / l_j2;
        double b1 = sqrt(radius * radius - a1 * a1);
        double b2 = sqrt(radius * radius - a2 * a2);

        if (stateMachine.state == StateMachine::POSITIVE) {
            this->prevForcePointPlane = this->globalCoordinates + a1 * j1 - b1 * k1;
            this->nextForcePointPlane = this->globalCoordinates + a2 * j2 - b2 * k2;
        } else if (stateMachine.state == StateMachine::NEGATIVE) {
            this->prevForcePointPlane = this->globalCoordinates + a1 * j1 + b1 * k1;
            this->nextForcePointPlane = this->globalCoordinates + a2 * j2 + b2 * k2;
        }

        //update revolution counter
        double projection = (prevCoordPlane - this->prevForcePointPlane).Dot(
                this->nextForcePointPlane - this->prevForcePointPlane);
        stateMachine.UpdateRevCounter(projection);

        //calculate the wrapping angle
        double angle = acos(1 - (pow((this->prevForcePointPlane - this->nextForcePointPlane).Length(), 2) /
                                 (2 * radius * radius)));
        arcAngle = 2 * (boost::math::constants::pi<double>()) * ceil(stateMachine.revCounter / 2);
        arcAngle += (stateMachine.revCounter % 2 == 0) ? (angle) : (-angle);

        double l_insertion = (prevCoordPlane - prevForcePointPlane).Length();
        double l_fixation = (nextCoordPlane - nextForcePointPlane).Length();
        double l_arc = arcAngle * radius;

        //calculate tangent point distance to the plane
        double iTDistance = prevDist + (l_insertion) * (nextDist - prevDist) / (l_insertion + l_arc + l_fixation);
        //project tangent point into R3
        prevForcePoint = prevForcePointPlane + iTDistance * unit_normal;

        //calculate tangent point distance to the plane
        double fTDistance = nextDist + (l_fixation) * (prevDist - nextDist) / (l_fixation + l_arc + l_insertion);
        //project tangent point into R3
        nextForcePoint = nextForcePointPlane + fTDistance * unit_normal;

        //calculate the lines of action and the muscle's length
        double distance = iTDistance - fTDistance;

        //calculate the lines of action and the muscle's length

        previousSegmentLength =
                (prevCoord - this->prevForcePoint).Length() + sqrt(distance * distance + l_arc * l_arc);
        // ROS_INFO_STREAM("previousSegmentLength " << previousSegmentLength);

    };


    void CylindricalWrapping::CalculateForce() {
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
            ignition::math::Vector3d A = prevPoint->nextForcePoint - this->prevForcePoint;
            if (A.Length() > 0.0) {
              prevForce = A / A.Length() * fa;
              // ROS_WARN_STREAM("fa: " << fa << " prevForce: " << prevForce);
            }

            else
                prevForce = ignition::math::Vector3d::Zero;

            //link->AddForceAtRelativePosition(Fa, this->prevForcePoint);
        }
        if (nextPoint) {
            ignition::math::Vector3d B = nextPoint->prevForcePoint - this->nextForcePoint;
            if (B.Length() > 0.0) {
                nextForce = B / B.Length() * fb;
                // ROS_WARN_STREAM("nextForce: " << nextForce);
            }

            else
                nextForce = ignition::math::Vector3d::Zero;

            //link->AddForceAtRelativePosition(Fb, this->nextForcePoint);
        }
    };

}
