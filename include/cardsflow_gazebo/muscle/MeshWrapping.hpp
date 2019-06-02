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
        gazebo::math::Vector3d prevCoord;
        gazebo::math::Vector3d nextCoord;
        gazebo::math::Vector3d wrappingDir;
        std::map<int, gazebo::math::Vector3d> vertices;
        std::set<gazebo::math::Vector3d> facets;
        gazebo::math::Vector3d x_axis;
        gazebo::math::Vector3d y_axis;
        gazebo::math::Vector3d z_axis;
        std::map<int, gazebo::math::Vector3d> geodesicPathPoints;
        std::map<int, gazebo::math::Vector3d> halfSpace;
        std::map<int, gazebo::math::Vector3d> convexHullVertices;
        std::map<int, gazebo::math::Vector3d> convexHullFacets;
        std::map<int, gazebo::math::Vector3d> facetNormals;
        double alphaMax;

        void CoordinateFrame();
        void HalfSpace();
        void ConvexEnvelope();
        void GeodesicPath();

	};
}