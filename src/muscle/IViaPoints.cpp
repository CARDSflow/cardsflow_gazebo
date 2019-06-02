#include "cardsflow_gazebo/muscle/IViaPoints.hpp"

namespace cardsflow_gazebo {

    IViaPoints::IViaPoints() : linkPosition(gazebo::math::Vector3d(0, 0, 0)), linkRotation(gazebo::math::Quaterniond(0, 0, 0, 0)),
                               localCoordinates(gazebo::math::Vector3d(0, 0, 0)),
                               globalCoordinates(gazebo::math::Vector3d(0, 0, 0)), type(Type::FIXPOINT), prevPoint(nullptr),
                               nextPoint(nullptr),
                               prevForcePoint(gazebo::math::Vector3d(0, 0, 0)), nextForcePoint(gazebo::math::Vector3d(0, 0, 0)), fa(0),
                               fb(0), prevForce(gazebo::math::Vector3d(0, 0, 0)),
                               nextForce(gazebo::math::Vector3d(0, 0, 0)), previousSegmentLength(0), link(nullptr) {

    };

    IViaPoints::IViaPoints(gazebo::math::Vector3d point) : IViaPoints() {
        localCoordinates = point;
    };

    IViaPoints::IViaPoints(gazebo::math::Vector3d point, physics::LinkPtr link) : IViaPoints() {
        localCoordinates = point;
        this->link = link;
    };

    IViaPoints::IViaPoints(gazebo::math::Vector3d point, Type t, physics::LinkPtr link) : IViaPoints(point, link) {
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
            gazebo::math::Vector3d A = prevPoint->nextForcePoint - this->prevForcePoint;
//        ROS_INFO_THROTTLE(1,"A length %f, fa %f", A.Length(), fa);
            if (A.Length() > 0.0)
                prevForce = A / A.Length() * fa;
            else
                prevForce = gazebo::math::Vector3d::Zero;
        } else if (nextPoint) {
            gazebo::math::Vector3d B = nextPoint->prevForcePoint - this->nextForcePoint;
//        ROS_INFO_THROTTLE(1,"B length %f, fb %f", B.Length(), fb);
            if (B.Length() > 0.0)
                nextForce = B / B.Length() * fb;
            else
                nextForce = gazebo::math::Vector3d::Zero;
        }
    };
}