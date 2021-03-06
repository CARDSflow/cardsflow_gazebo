#pragma once

#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <math.h>

namespace cardsflow_gazebo
{
    using namespace gazebo;

    class StateMachine{

    public:
        ////////////////////
        /// \brief The state enum for different states of the state machine
        ///
        /// Differentiates between three possible states of the state machine:
        /// 0: Not Wrapping
        /// 1: Positive, for wrapping over angles smaller than 180°
        /// 2: Negative, for wrapping over angles larger than 180°
        enum State
        {
            NOTWRAPPING = 0,
            POSITIVE = 1,
            NEGATIVE = 2
        };

    public:
        StateMachine();

        ////////////////////////////////////////
        /// \brief Decides if the muscle is positive, negative or not wrapping
        /// \param[in] prevPoint the previous attachment point
        /// \param[in] nextPoint the nextattachment point
        /// \param[in] center the center of the wrapping surface
        /// \param[in] radius the radius of the wrapping surface
        void UpdateState(ignition::math::Vector3d& prevPoint, ignition::math::Vector3d& nextPoint, ignition::math::Vector3d& center, double radius);

        ////////////////////////////////////////
        /// \brief updates the RevCounter
        /// \param[in] projection the current projection from vector prevForcePoint->nextForcePoint onto the vector prevForcePoint->prevPoint
        void UpdateRevCounter(double projection);

    public:
        State state;
        int revCounter;
    private:
        bool firstUpdate;
        ignition::math::Vector3d normal;
        double projection;
    };
}