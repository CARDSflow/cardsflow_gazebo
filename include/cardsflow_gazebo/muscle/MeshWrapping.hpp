#pragma once

#include "IViaPoints.hpp"

#include <boost/numeric/odeint.hpp>

using namespace gazebo;

namespace cardsflow_gazebo
{
	class ITendon;

	class MeshWrapping : public IViaPoints
	{

    public:
        MeshWrapping();

        virtual void UpdateForcePoints();

        virtual void CalculateForce();



    public:
        ignition::math::Vector3d prevCoord;
        ignition::math::Vector3d nextCoord;
        ignition::math::Vector3d wrappingDir;
        std::map<int, ignition::math::Vector3d> vertices;
        std::set<ignition::math::Vector3d> facets;
        ignition::math::Vector3d x_axis;
        ignition::math::Vector3d y_axis;
        ignition::math::Vector3d z_axis;
        std::map<int, ignition::math::Vector3d> geodesicPathPoints;
        std::map<int, ignition::math::Vector3d> halfSpace;
        std::map<int, ignition::math::Vector3d> convexHullVertices;
        std::map<int, ignition::math::Vector3d> convexHullFacets;
        std::map<int, ignition::math::Vector3d> facetNormals;
        double alphaMax;

        void CoordinateFrame();
        void HalfSpace();
        void ConvexEnvelope();
        void GeodesicPath();

	};
}