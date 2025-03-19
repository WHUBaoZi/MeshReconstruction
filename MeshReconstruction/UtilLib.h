﻿#pragma once

#include <string>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <cmath>
#include <chrono>
#include <iostream>
#include <format>

#include <boost/filesystem.hpp>

#include "CGALTypes.h"

#define PI std::acos(-1)

namespace UtilLib
{
	const Vector_3 unitZ(0.0, 0.0, 1.0);

	const double DEG_TO_RAD = PI / 180.0;

	constexpr double INF = std::numeric_limits<double>::infinity();

	void FilterMesh(Mesh& mesh, int iterCount = 20);

	Point_3 GetFaceCenter(const face_descriptor& face, const Mesh& mesh);

	std::set<face_descriptor> GetKRingFaces(const face_descriptor& face, int kRing, const Mesh& mesh);	// Contain self; Only for Triangle Face

	std::set<vertex_descriptor> GetKRingVertices(const vertex_descriptor& vertex, int kRing, const Mesh& mesh);	// Contain self

	std::vector<vertex_descriptor> GetFaceVertices(const CGAL::SM_Face_index& face, const Mesh& mesh);

	Plane_3 FitPlaneFromFaces(const std::set<face_descriptor> faces, const Mesh& mesh);

	CGAL::Color GenerateRandomColor();

	std::set<int> GetPartitionNeighbors(int partitionId, const std::map<int, std::set<face_descriptor>>& partitionFacesMap, const Mesh& mesh);

	Vector_3 GetPartitionAverageNormal(int partitionId, const std::map<int, std::set<face_descriptor>>& partitionFacesMap, const Mesh& mesh);

	Mesh CreateCube(const Point_3& minPoint, const Point_3& maxPoint);

	void CreatePlaneMesh(const Plane_3& plane, const Point_3& centerPoint, Mesh& mesh, double size = 2.0);

	Point_3 GetMeshCenterPoint(const Mesh& mesh);

	void CentralizeMesh(Mesh& mesh);

	std::map<int, std::vector<face_descriptor>> PartitionByNormal(Mesh& mesh);

	inline Plane_3 ReversePlane(const Plane_3& plane) { return Plane_3(-plane.a(), -plane.b(), -plane.c(), -plane.d()); }

	inline Vector_3 PointToVector(const Point_3& point) { return Vector_3(point.x(), point.y(), point.z()); }

	inline Point_3 VectorToPoint(const Vector_3& vector) { return Point_3(vector.x(), vector.y(), vector.z()); }
}