#pragma once

#include <string>
#include <vector>
#include <queue>
#include <cmath>
#include <chrono>
#include <iostream>

#include <boost/filesystem.hpp>

#include "CGALTypes.h"

namespace UtilLib
{
	const Vector_3 unitZ(0.0, 0.0, 1.0);

	constexpr double DEG_TO_RAD = M_PI / 180.0;

	constexpr double INF = std::numeric_limits<double>::infinity();

	void MeshFiltering(Mesh& mesh, int iterCount);

	Point_3 GetFaceCenter(const face_descriptor& face, const Mesh& mesh);

	std::set<face_descriptor> GetTriFacesKRing(const face_descriptor& face, int kRing, const Mesh& mesh);

	double ComputeVPlanarityOfKRing(const vertex_descriptor& vertex, int kRing, const Mesh& mesh);

	std::set<vertex_descriptor> GetVerticesKRing(const vertex_descriptor& vertex, int kRing, const Mesh& mesh);

	std::vector<vertex_descriptor> GetVerticesAroundFace(const CGAL::SM_Face_index& face, const Mesh& mesh);

	CGAL::Color GetRandomColor();

	std::set<int> GetPartitionNeighbors(int id, const std::map<int, std::set<face_descriptor>>& partitionFacesMap, const Mesh& mesh);

	Vector_3 GetPartitionAverageNormal(int id, const std::map<int, std::set<face_descriptor>>& partitionFacesMap, const Mesh& mesh);

	Mesh MakeCube(const Point_3& minPoint, const Point_3& maxPoint);

	bool Intersection(const Plane_3& plane, const Mesh& mesh);

	bool Intersection(const Line_3& line, const face_descriptor& triangleFace, const Mesh& mesh);

	bool IsPolyhedronValid(const Mesh& mesh);

	void CreatePlaneMesh(const Plane_3& plane, const Point_3& centerPoint, Mesh& mesh, double size = 2.0);

	Point_3 GetMeshCenterPoint(const Mesh& mesh);

	void CentralizeMesh(Mesh& mesh);

	inline Plane_3 ReversePlane(const Plane_3& plane) { return Plane_3(-plane.a(), -plane.b(), -plane.c(), -plane.d()); }
}