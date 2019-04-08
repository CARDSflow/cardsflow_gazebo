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

#include "MeshPulley.h"
#include <dae.h>
#include <rl/util/Timer.h>
#include <rl/math/Rotation.h>
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include "geodesic/geodesic_algorithm_exact.h"
#include "geodesic/geodesic_algorithm_dijkstra.h"
#include "geodesic/geodesic_algorithm_subdivision.h"
extern "C"
{
#include <libqhull/qhull_a.h>
}

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
MeshPulley::MeshPulley(sgal::physics::Body* body, IMuscle* muscle,
		const QDomElement& pulleyElement, const QString& rootPath) :
		AttachmentPoint(body, rl::math::Transform::Identity(), muscle), rootPath(
				rootPath), nF(rl::math::Vector3::Zero()), pF(
				rl::math::Vector3::Zero()), length(0.0)
{
	// load meshes
	rl::math::Transform principal;

	body->getPrincipalTransform(principal);

	for (QDomElement meshElement = pulleyElement.firstChildElement("mesh");
			!meshElement.isNull(); meshElement =
					meshElement.nextSiblingElement())
	{
		addMesh(meshElement);
	}

	for (int i = 0; i < meshes.size(); i++)
	{
		meshes[i].transform = principal.inverse() * meshes[i].transform;
	}

	// initialize z vector
	QDomElement zElement = pulleyElement.firstChildElement("z");

	if (!zElement.isNull())
	{
		zInit(0) = zElement.attribute("x", "0").toDouble();

		zInit(1) = zElement.attribute("y", "0").toDouble();

		zInit(2) = zElement.attribute("z", "0").toDouble();
	}

	zInit.normalize();
}

MeshPulley::~MeshPulley()
{
}

IAttachmentPoint::Type MeshPulley::getType() const
{
	return IAttachmentPoint::MeshPulley;
}

void MeshPulley::updateForcePoints(rl::math::Real time) throw (sgal::Exception)
{
	previousAnchorInW = previous->getNextForcePointInGlobal();

	nextAnchorInW = next->getPreviousForcePointInGlobal();

#ifdef DEBUG
	std::cout << "Muscle=" << muscle->getName() << std::endl;

	std::cout << "\tPreviousAnchor=(" << previousAnchorInW(0) << ", "
	<< previousAnchorInW(1) << ", " << previousAnchorInW(2) << ")"
	<< std::endl;

	std::cout << "\tNextAnchor=(" << nextAnchorInW(0) << ", "
	<< nextAnchorInW(1) << ", " << nextAnchorInW(2) << ")" << std::endl;
#endif

	reset();

	computeCoordinateFrame();

	computeHalfSpace();

#ifdef DEBUG
	std::cout << "\tHalf space computed, contains " << halfSpace.size() / 3
	<< " points" << std::endl;

	serializeHalfSpace();
#endif

	if (halfSpace.size() / 3 > 3)
	{
#ifdef DEBUG
		std::cout << "\tConvex envelope...";

		std::cout.flush();
#endif

		computeConvexEnvelope();

#ifdef DEBUG
		std::cout << "done (" << envelopeVertices.size() / 3 << " points, "
		<< envelopeFacets.size() / 3 << " facets)" << std::endl;

		std::cout << "\tGeodesic path...";

		std::cout.flush();
#endif

		computeGeodesicPath();

#ifdef DEBUG
		std::cout << "done (" << path.size() << " points)" << std::endl;

		std::cout.flush();
#endif

		if (path.size() > 2)
		{
			state = Wrapping;

			pF(0) = path[1].x();
			pF(1) = path[1].y();
			pF(2) = path[1].z();

			nF(0) = path[path.size() - 2].x();
			nF(1) = path[path.size() - 2].y();
			nF(2) = path[path.size() - 2].z();

#ifdef DEBUG
			for (int i = 0; i < path.size(); i++)
			{
				std::cout << "\ti=" << i << ", " << path[i].x() << ", "
				<< path[i].y() << ", " << path[i].z() << std::endl;
			}

			std::cout << "\tPath length...\n";
#endif
			// compute path length
			// from previous->next to last point on the mesh
			length = 0.0;

			for (int i = 1; i < path.size() - 1; i++)
			{
				length += path[i].distance(&path[i - 1]);

#ifdef DEBUG
				std::cout << "\ti=" << i << ", lS="
				<< path[i].distance(&path[i - 1]) << ", lTotal="
				<< length << std::endl;
#endif
			}
		}
		else
		{
			state = NotWrapping;

			length = 0.0;

			pF = nextAnchorInW;

			nF = previousAnchorInW;
		}
	}
	else
	{
		state = NotWrapping;

		length = 0.0;

		pF = nextAnchorInW;

		nF = previousAnchorInW;
	}

#ifdef DEBUG
	std::cout << "\tState=" << state << ", Length=" << length << ", pF=("
	<< pF(0) << ", " << pF(1) << ", " << pF(2) << "), nF=(" << nF(0)
	<< ", " << nF(1) << ", " << nF(2) << ")" << std::endl;
#endif
}

void MeshPulley::applyForce(rl::math::Real time,
		rl::math::Real previousSegmentKiteLineVelocity, bool beforeSEE)
				throw (sgal::Exception)
{
	// no friction in pulley but required to correctly propagate forces through tendon path
	if (beforeSEE)
	{
		fa = fb = next->getPreviousForce();
	}
	else
	{
		fb = fa = previous->getNextForce();
	}

	if (state == Wrapping)
	{
		// vector from previous to tangentPointA
		A = previous->getNextForcePointInGlobal()
				- getPreviousForcePointInGlobal();

		Fa = A.normalized() * fa;

		// vector from tangentPointB to next
		B = next->getPreviousForcePointInGlobal() - getNextForcePointInGlobal();

		Fb = B.normalized() * fb;

		body->applyForce(Fa,
				getPreviousForcePointInGlobal() - bodyFrame.translation());

		body->applyForce(Fb,
				getNextForcePointInGlobal() - bodyFrame.translation());
	}
}

rl::math::Vector3 MeshPulley::getPreviousForcePointInGlobal() const
{
	return pF;
}

rl::math::Vector3 MeshPulley::getNextForcePointInGlobal() const
{
	return nF;
}

rl::math::Real MeshPulley::getPreviousSegmentLength() const
{
	if (state == Wrapping)
	{
		return length;
	}

	return 0.0;
}

void MeshPulley::getHalfSpace(
		QVarLengthArray< rl::math::Vector3 >& halfSpace) const
{
	halfSpace.clear();

	halfSpace.reserve(this->halfSpace.size() / 3);

	rl::math::Vector3 vec;

	for (int i = 0; i < this->halfSpace.size(); i += 3)
	{
		vec(0) = this->halfSpace[i];

		vec(1) = this->halfSpace[i + 1];

		vec(2) = this->halfSpace[i + 2];

		halfSpace.append(vec);
	}
}

void MeshPulley::getConvexEnvenlope(
		QVarLengthArray< rl::math::Vector3 >& vertices,
		QVarLengthArray< Eigen::Matrix< int, 3, 1 > >& facets) const
{
	vertices.clear();

	facets.clear();

	rl::math::Vector3 vertex;

	vertices.reserve(envelopeVertices.size() / 3);

	for (int i = 0; i < envelopeVertices.size(); i += 3)
	{
		vertex(0) = envelopeVertices[i];

		vertex(1) = envelopeVertices[i + 1];

		vertex(2) = envelopeVertices[i + 2];

		vertices.append(vertex);
	}

	facets.reserve(envelopeFacets.size() / 3);

	Eigen::Matrix< int, 3, 1 > facet;

	for (int i = 0; i < envelopeFacets.size(); i += 3)
	{
		facet(0) = envelopeFacets[i];

		facet(1) = envelopeFacets[i + 1];

		facet(2) = envelopeFacets[i + 2];

		facets.append(facet);
	}
}

void MeshPulley::getTendonNodes(QVector< rl::math::Vector3 >& tendonNodes)
{
	if (state == Wrapping)
	{
		rl::math::Vector3 point;

		for (int i = 0; i < path.size(); i++)
		{
			point(0) = path[i].x();

			point(1) = path[i].y();

			point(2) = path[i].z();

			tendonNodes.push_back(point);
		}
	}
}

const rl::math::Vector3& MeshPulley::getWrappingVector() const
{
	return z;
}

const rl::math::Vector3& MeshPulley::getU() const
{
	return u;
}

const rl::math::Vector3& MeshPulley::getV() const
{
	return v;
}

const rl::math::Vector3& MeshPulley::getW() const
{
	return w;
}


int MeshPulley::getNumMeshes() const
{
	return meshes.size();
}

Mesh& MeshPulley::getMesh(int i)
{
	return meshes[i];
}

void MeshPulley::loadMesh(Mesh& mesh)
{
	DAE dae;

	domCOLLADA* root = dynamic_cast< ColladaDOM141::domCOLLADA* >(dae.open(
			mesh.fileName.toStdString()));

	domGeometry* domGeometry = 0;

	for (int i = 0; i < root->getLibrary_geometries_array().getCount(); i++)
	{
		for (int j = 0;
				j
						< root->getLibrary_geometries_array()[i]->getGeometry_array().getCount();
				j++)
		{
			domGeometry =
					root->getLibrary_geometries_array()[i]->getGeometry_array()[j];

			if (QString(domGeometry->getID()) == mesh.name)
			{
				break;
			}
		}
	}

	if (domGeometry->getMesh())
	{
		if (domGeometry->getMesh()->getVertices())
		{
			mesh.vertices.clear();

			for (std::size_t i = 0;
					i
							< domGeometry->getMesh()->getVertices()->getInput_array().getCount();
					++i)
			{
				domInputLocal* inputLocal =
						domGeometry->getMesh()->getVertices()->getInput_array()[i];

				if (0
						== cdom::strcasecmp(inputLocal->getSemantic(),
								"POSITION"))
				{
					domSource* source = daeSafeCast< domSource >(
							inputLocal->getSource().getElement());

					domFloat_array* floatArray = source->getFloat_array();

					domSource::domTechnique_common* techniqueCommon =
							source->getTechnique_common();

					domAccessor* accessor = techniqueCommon->getAccessor();

					rl::math::Vector3 vertice;

					for (::std::size_t j = 0; j < accessor->getCount(); ++j)
					{
						vertice(0) =
								floatArray->getValue()[accessor->getOffset()
										+ j * accessor->getStride() + 0];

						vertice(1) =
								floatArray->getValue()[accessor->getOffset()
										+ j * accessor->getStride() + 1];

						vertice(2) =
								floatArray->getValue()[accessor->getOffset()
										+ j * accessor->getStride() + 2];

						mesh.vertices.push_back(vertice);
					}
				}
			}
		}
	}
}

void MeshPulley::reset()
{
	// resetting of data structures
	path.clear();

	halfSpace.clear();

	envelopeFacets.clear();

	envelopeVertices.clear();

	targetVertexIndex = sourceVertexIndex = -1;
}

void MeshPulley::computeCoordinateFrame()
{
#ifdef _DO_PROFILE_
	coordTimer.start();
#endif

	rl::math::Transform nextFrame;

	next->getBody()->getFrame(nextFrame);

	// update z vector and convert it to the world coordinates
	z = (nextFrame.rotation() * zInit).normalized();

	// compute coordinate system in world
	u = (previousAnchorInW - nextAnchorInW).normalized();

	if (std::abs(u.dot(z)) > 0.98)
	{
		QString what = QString(muscle->getName().c_str())
				+ QString(" angle between z and u vector too small");

		throw sgal::Exception(what.toStdString().c_str());
	}

	v = z.cross(u).normalized();

	w = u.cross(v).normalized();

#ifdef _DO_PROFILE_
	coordTimer.stop();

	coordTimes.addValue(coordTimer.elapsed());
#endif
}

void MeshPulley::computeHalfSpace()
{
#ifdef _DO_PROFILE_
	halfSpaceTimer.start();
#endif

	rl::math::Vector3 s, p;

	for (int i = 0; i < meshes.size(); i++)
	{
		Mesh& mesh = meshes[i];

		rl::math::Transform meshToWorld = bodyFrame * mesh.transform;

		rl::math::Rotation rotation = meshToWorld.rotation();

		rl::math::Vector3 translation = meshToWorld.translation();

		for (int j = 0; j < mesh.vertices.size(); j++)
		{
			// express vertice in world
			s = rotation * mesh.vertices[j] + translation;

			// express vertice in previousAnchor
			p = s - nextAnchorInW;

			if (p.dot(w) > 0 && p.dot(u) > 0)
			{
				halfSpace.push_back(s(0));
				halfSpace.push_back(s(1));
				halfSpace.push_back(s(2));
			}
		}
	}

	halfSpace.push_back(previousAnchorInW(0));
	halfSpace.push_back(previousAnchorInW(1));
	halfSpace.push_back(previousAnchorInW(2));

	halfSpace.push_back(nextAnchorInW(0));
	halfSpace.push_back(nextAnchorInW(1));
	halfSpace.push_back(nextAnchorInW(2));

#ifdef _DO_PROFILE_
	halfSpaceTimer.stop();

	halfSpaceTimes.addValue(halfSpaceTimer.elapsed());
#endif
}

void MeshPulley::computeConvexEnvelope()
{
#ifdef _DO_PROFILE_
	convexEnvelopeTimer.start();
#endif

	FILE *errFile = stderr;

	int returnValue = qh_new_qhull(3, halfSpace.size() / 3, halfSpace.data(),
			false, const_cast< char* >(std::string("qhull QJ").c_str()), NULL,
			errFile);

	if (returnValue == 0)
	{
		rl::math::Vector3 rlVertex;

		vertexT* vertex;

		unsigned int i = 0;

		FORALLvertices
		{
			vertex->id = i;

			rlVertex(0) = vertex->point[0];

			rlVertex(1) = vertex->point[1];

			rlVertex(2) = vertex->point[2];

			if ((rlVertex - nextAnchorInW).norm() < 1e-5)
			{
				targetVertexIndex = i;
			}

			if ((rlVertex - previousAnchorInW).norm() < 1e-5)
			{
				sourceVertexIndex = i;
			}

			envelopeVertices.push_back(vertex->point[0]);

			envelopeVertices.push_back(vertex->point[1]);

			envelopeVertices.push_back(vertex->point[2]);

			i++;
		}

		facetT* facet;

		vertexT** vertexp;

		rl::math::Vector3 n;

		FORALLfacets
		{
			n(0) = facet->normal[0];
			n(1) = facet->normal[1];
			n(2) = facet->normal[2];

			if (n.dot(w) >= 0)
			{
				FOREACHvertex_(facet->vertices)
				{
					envelopeFacets.push_back(vertex->id);
				}

				if (facet->toporient)
				{
					int a = envelopeFacets[envelopeFacets.size() - 3];

					int b = envelopeFacets[envelopeFacets.size() - 2];

					envelopeFacets[envelopeFacets.size() - 3] = b;

					envelopeFacets[envelopeFacets.size() - 2] = a;
				}
			}
		}
	}
	else
	{
		throw sgal::Exception("Qhull error: hull could not be computed.");
	}

	qh_freeqhull(!qh_ALL);

	int curlong, totlong;

	qh_memfreeshort(&curlong, &totlong);

	if (curlong || totlong)
	{
		std::cerr << "qhull internal warning (main): did not free " << totlong
				<< " bytes of long memory(" << curlong << " pieces)"
				<< std::endl;

		throw sgal::Exception("Qhull error: memory could not be freed");
	}

#ifdef _DO_PROFILE_
	convexEnvelopeTimer.stop();

	convexEnvelopeTimes.addValue(convexEnvelopeTimer.elapsed());
#endif
}

void MeshPulley::computeGeodesicPath()
{
#ifdef _DO_PROFILE_
	pathTimer.start();
#endif

	if (envelopeVertices.size() == 0 || envelopeFacets.size() == 0)
	{
		QString string = QString(muscle->getName().c_str())
				+ " convex envelope invalid";

		throw sgal::Exception(string.toStdString().c_str());
	}

	if (sourceVertexIndex == -1 || targetVertexIndex == -1)
	{
		QString string = QString(muscle->getName().c_str())
				+ " source or target vertex invalid";

		throw sgal::Exception(string.toStdString().c_str());
	}

	if (sourceVertexIndex == targetVertexIndex)
	{
		QString string = QString(muscle->getName().c_str())
				+ " source and target vertex identical";

		throw sgal::Exception(string.toStdString().c_str());
	}

	geodesic::Mesh mesh;

	mesh.initialize_mesh_data(envelopeVertices, envelopeFacets);

	geodesic::GeodesicAlgorithmExact algorithm(&mesh);
	//geodesic::GeodesicAlgorithmDijkstra algorithm(&mesh);
	//geodesic::GeodesicAlgorithmSubdivision algorithm(&mesh);

	geodesic::SurfacePoint source(&mesh.vertices()[sourceVertexIndex]);

	geodesic::SurfacePoint target(&mesh.vertices()[targetVertexIndex]);

	// path is traced back from target to source -> therefore source/target are exchanged
	algorithm.geodesic(target, source, path);

#ifdef _DO_PROFILE_
	pathTimer.stop();

	pathTimes.addValue(pathTimer.elapsed());
#endif
}

void MeshPulley::addMesh(const QDomElement& domElement)
{
	Mesh mesh;

	if (domElement.hasAttribute("colladaFile"))
	{
		mesh.fileName = domElement.attribute("colladaFile");

		if (!mesh.fileName.startsWith(QChar('/')))
		{
			mesh.fileName = rootPath + "/" + mesh.fileName;
		}
	}

	if (domElement.hasAttribute("meshName"))
	{
		mesh.name = domElement.attribute("meshName");
	}

	QDomNodeList childNodes = domElement.childNodes();

	for (int i = 0; i < childNodes.size(); i++)
	{
		QDomElement domElement = childNodes.item(i).toElement();

		if (!domElement.isNull())
		{
			rl::math::Transform transform;

			transform.setIdentity();

			if (domElement.nodeName() == "matrix")
			{
				QString text = domElement.text();

				QStringList rows = text.split('\n', QString::SkipEmptyParts);

				for (int j = 0; j < 4; j++)
				{
					QStringList elements = rows[j].split(' ',
							QString::SkipEmptyParts);

					for (int k = 0; k < 4; k++)
					{
						transform(j, k) = elements[k].toDouble();
					}
				}
			}
			else if (domElement.nodeName() == "translate")
			{
			}
			else if (domElement.nodeName() == "rotate")
			{
			}

			mesh.transform = transform * mesh.transform;
		}
	}

	loadMesh(mesh);

	meshes.push_back(mesh);
}

void MeshPulley::serializeHalfSpace()
{
	QFile file("/tmp/halfSpace.txt");

	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		throw sgal::Exception(
				"Half-space serializing error, not possible to generate file.");
	}

	QTextStream out(&file);

	out << "# " << muscle->getName().c_str() << "\n";

	out << 3 << "\n" << halfSpace.size() / 3 << "\n";

	for (int i = 0; i < halfSpace.size(); i += 3)
	{
		out << halfSpace[i] << " " << halfSpace[i + 1] << " "
				<< halfSpace[i + 2] << "\n";
	}

	file.flush();

	file.close();
}
}
}
}
}
}
}
}
