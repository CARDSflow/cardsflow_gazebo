/*
 *  Copyright (c) 2012-2013, MYOROBOTICS consortium
 *  Author: Steffen Wittmeier, Konstantinos Dalamagkidis, Michael Jaentsch
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

#ifndef _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_MESHPULLEY_H_
#define _DE_CALIPER_SIM_EXT_ACTUATORS_ER_PHYSICS_MESHPULLEY_H_

#include <sgal/physics/Body.h>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
#include <QString>
#include <QVector>
#include <QMutex>
#include <dom/domCOLLADA.h>

#include "caliper/de.caliper.sim/ext/actuators/eccerobotMuscle/physics/IMeshPulley.h"
#include "AttachmentPoint.h"
#include "geodesic/geodesic_mesh_elements.h"

#ifdef _DO_PROFILE_
#include "../test/BenchmarkerData.h"
#include <rl/util/Timer.h>
#endif

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

class Mesh
{
public:
	Mesh() :
			transform(rl::math::Transform::Identity()), fileName(""), name("")
	{

	}

	QString fileName;

	QString name;

	rl::math::Transform transform;

	QVector< rl::math::Vector3 > vertices;
};

class MeshPulley: public virtual IMeshPulley, public AttachmentPoint
{
public:
	enum State
	{
		Wrapping, NotWrapping
	};

	MeshPulley(sgal::physics::Body* body, IMuscle* muscle,
			const QDomElement& pulleyElement, const QString& rootPath);

	virtual ~MeshPulley();

	IAttachmentPoint::Type getType() const;

	void updateForcePoints(rl::math::Real time) throw (sgal::Exception);

	void applyForce(rl::math::Real time,
			rl::math::Real previousSegmentKiteLineVelocity, bool beforeSEE)
					throw (sgal::Exception);

	rl::math::Vector3 getPreviousForcePointInGlobal() const;

	rl::math::Vector3 getNextForcePointInGlobal() const;

	rl::math::Real getPreviousSegmentLength() const;

	void getHalfSpace(QVarLengthArray< rl::math::Vector3 >& halfSpace) const;

	void getConvexEnvenlope(QVarLengthArray< rl::math::Vector3 >& vertices,
			QVarLengthArray< Eigen::Matrix< int, 3, 1 > >& facets) const;

	void getTendonNodes(QVector< rl::math::Vector3 >& tendonNodes);

	const rl::math::Vector3& getWrappingVector() const;

	const rl::math::Vector3& getU() const;

	const rl::math::Vector3& getV() const;

	const rl::math::Vector3& getW() const;

	int getNumMeshes() const;

	Mesh& getMesh(int i);

protected:
	QString colladaFile;

	QString geometryName;

	QVector< Mesh > meshes;

	QVector< double > halfSpace;

	QVector< double > envelopeVertices;

	QVector< int > envelopeFacets;

	rl::math::Vector3 u, v, w, z, zInit;

	int sourceVertexIndex, targetVertexIndex;

	rl::math::Vector3 previousAnchorInW;

	rl::math::Vector3 nextAnchorInW;

	std::vector< geodesic::SurfacePoint > path;

	rl::math::Vector3 pF, nF;

	State state;

	double length;

	QString rootPath;

protected:
	void loadMesh(Mesh& mesh);

	void reset();

	void computeCoordinateFrame();

	void computeHalfSpace();

	void computeConvexEnvelope();

	void computeGeodesicPath();

	void addMesh(const QDomElement& domElement);

	void serializeHalfSpace();

#ifdef _DO_PROFILE_
public:
	BenchmarkerData coordTimes;

	BenchmarkerData halfSpaceTimes;

	BenchmarkerData convexEnvelopeTimes;

	BenchmarkerData pathTimes;

	rl::util::Timer totTimer;

	rl::util::Timer coordTimer;

	rl::util::Timer halfSpaceTimer;

	rl::util::Timer convexEnvelopeTimer;

	rl::util::Timer pathTimer;
#endif
};
}
}
}
}
}
}
}
#endif
