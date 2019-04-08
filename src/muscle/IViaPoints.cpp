#include "cardsflow_gazebo/muscle/IViaPoints.hpp"

namespace cardsflow_gazebo {

    IViaPoints::IViaPoints() : linkPosition(ignition::math::Vector3d(0, 0, 0)), linkRotation(ignition::math::Quaterniond(0, 0, 0, 0)),
                               localCoordinates(ignition::math::Vector3d(0, 0, 0)),
                               globalCoordinates(ignition::math::Vector3d(0, 0, 0)), type(Type::FIXPOINT), prevPoint(nullptr),
                               nextPoint(nullptr),
                               prevForcePoint(ignition::math::Vector3d(0, 0, 0)), nextForcePoint(ignition::math::Vector3d(0, 0, 0)), fa(0),
                               fb(0), prevForce(ignition::math::Vector3d(0, 0, 0)),
                               nextForce(ignition::math::Vector3d(0, 0, 0)), previousSegmentLength(0), link(nullptr) {

    };

    IViaPoints::IViaPoints(ignition::math::Vector3d point) : IViaPoints() {
        localCoordinates = point;
    };

    IViaPoints::IViaPoints(ignition::math::Vector3d point, physics::LinkPtr link) : IViaPoints() {
        localCoordinates = point;
        this->link = link;
    };

    IViaPoints::IViaPoints(ignition::math::Vector3d point, Type t, physics::LinkPtr link) : IViaPoints(point, link) {
        type = t;
    };

    void IViaPoints::UpdateForcePoints() {
        prevForcePoint = globalCoordinates;
        nextForcePoint = globalCoordinates;
        previousSegmentLength = (prevPoint) ? ((prevPoint->nextForcePoint - this->prevForcePoint).Length()) : (0);
    };

    void IViaPoints::CalculateForce() {

//    if(prevPoint && nextPoint)
//    {
//        //TODO: change fa, fb with respect to friction
//        // need to know if before or behind see
//        //previousSegmentKiteLineVelocity needed
//    }

        if (prevPoint) {
            ignition::math::Vector3d A = prevPoint->nextForcePoint - this->prevForcePoint;
//        ROS_INFO_THROTTLE(1,"A length %f, fa %f", A.Length(), fa);
            if (A.Length() > 0.0)
                prevForce = A / A.Length() * fa;
            else
                prevForce = ignition::math::Vector3d::Zero;
        } else if (nextPoint) {
            ignition::math::Vector3d B = nextPoint->prevForcePoint - this->nextForcePoint;
//        ROS_INFO_THROTTLE(1,"B length %f, fb %f", B.Length(), fb);
            if (B.Length() > 0.0)
                nextForce = B / B.Length() * fb;
            else
                nextForce = ignition::math::Vector3d::Zero;
        }
    };
}