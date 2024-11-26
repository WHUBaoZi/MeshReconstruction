#pragma once

#include "CGALTypes.h"
#include "UtilLib.h"


class Polyhedron
{
public:
	Mesh polyhedronMesh;
	std::vector<Plane_3> planes;
	std::vector<int> planeIntersectionNums;

public:
	Polyhedron(Mesh polyhedronMesh, std::vector<Plane_3> parentPlanes);

	Mesh drawPlanesMesh(const Point_3& centerPoint);

	inline int GetMinIntersectionIndex(){ return std::distance(planeIntersectionNums.begin(), std::min_element(planeIntersectionNums.begin(), planeIntersectionNums.end())); }
};

class PolyhedronSegmentation
{
public:
	Mesh mesh;
	
	Mesh cubeMesh;
	
	std::map<int, Plane_3> partitionPlaneMap;

	std::queue<Polyhedron> polyhedrons;

	std::vector<Polyhedron> indivisiblePolyhedrons;

public:
	PolyhedronSegmentation(Mesh mesh, std::map<int, Plane_3> partitionPlaneMap);

	Mesh run();
};