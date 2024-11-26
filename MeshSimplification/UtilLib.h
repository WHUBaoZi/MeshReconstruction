#pragma once
#define _USE_MATH_DEFINES

#include <string>
#include <vector>
#include <queue>

#include "CGALTypes.h"

namespace UtilLib
{
	const Vector_3 unitZ(0.0, 0.0, 1.0);

	constexpr double INF = std::numeric_limits<double>::infinity();

	void meshFiltering(Mesh& mesh, int iterCount);

	Point_3 getFaceCenter(const face_descriptor& face, const Mesh& mesh);

	std::set<face_descriptor> getTriFacesKRing(const face_descriptor& face, int kRing, const Mesh& mesh);

	double getTriFaceArea(const face_descriptor& face, const Mesh& mesh);

	double computeVPlanarityOfKRing(const vertex_descriptor& vertex, int kRing, const Mesh& mesh);

	std::set<vertex_descriptor> getVerticesKRing(const vertex_descriptor& vertex, int kRing, const Mesh& mesh);

	std::vector<vertex_descriptor> getVerticesAroundFace(const CGAL::SM_Face_index& face, const Mesh& mesh);

	CGAL::Color getRandomColor();

	std::set<int> getPartitionNeighbors(int id, const std::map<int, std::set<face_descriptor>>& partitionFacesMap, const Mesh& mesh);

	Vector_3 getPartitionAverageNormal(int id, const std::map<int, std::set<face_descriptor>>& partitionFacesMap, const Mesh& mesh);

	Mesh makeCube(const Point_3& minPoint, const Point_3& maxPoint);

	bool intersection(const Plane_3& plane, const Mesh& mesh);

	bool intersection(const Line_3& line, const face_descriptor& triangleFace, const Mesh& mesh);

	bool isPolyhedronValid(const Mesh& mesh);

	void createPlaneMesh(const Plane_3& plane, const Point_3& centerPoint, Mesh& mesh, double size = 2.0);

	Point_3 getMeshCenterPoint(const Mesh& mesh);

	//Point_3 getPolyhedronCentroid(const Mesh& polyhedron);

	inline Plane_3 reversePlane(const Plane_3& plane) { return Plane_3(-plane.a(), -plane.b(), -plane.c(), -plane.d()); }
}