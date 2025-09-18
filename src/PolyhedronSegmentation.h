#pragma once

#include "CGALTypes.h"
#include "UtilLib.h"
#include "Partition.h"
#include "Voxel.h"

namespace PolyhedronSegmentationFunctions
{
	inline std::string outputPath;

	Mesh DoSegmentation(const Mesh& mesh, const std::unordered_map<int, std::unordered_set<face_descriptor>>& partitions);
}


struct DividingSurface
{
public:
	int partitionID;

	Point_3 centerPoint;

	Vector_3 averageNormal;

	Vector_3 normal;

	Plane_3 plane;

	Mesh planeMesh;
};

struct Polyhedron
{
public:
	Mesh polyhedronMesh;

	int polyhedronID;

	std::unordered_set<int> dividingSurfaces;

//public:
//
	//Point_3 centroidPoint;
	//Tree polyhedronTree;
	//std::vector<std::shared_ptr<Partition>> partitions;
	//std::vector<int> planeIntersectionNums;
//
//public:
//
//	Polyhedron(Mesh polyhedronMesh, std::vector<std::shared_ptr<Partition>> parentPartitions);
//
//	Polyhedron() {};
//
//	void Remesh();

};


class SegmentationManager
{
public:
	SegmentationManager(const Mesh& mesh, const std::unordered_map<int, std::unordered_set<face_descriptor>>& partitions) : mesh(mesh), partitions(partitions) {}

public:
	Mesh mesh;

	std::unordered_map<int, std::unordered_set<face_descriptor>> partitions;

	std::unordered_map<int, DividingSurface> dividingSurfaces;

	std::unordered_map<int, Polyhedron> polyhedrons;

public:
	int CreateDividingSurface(const std::pair<int, std::unordered_set<face_descriptor>>& pair);

	int CreatePolyhedron(Mesh polyhedronMesh, std::unordered_set<int> parentDividingSurfaces);

private:
	int nextPolyhedronID = 0;

	inline int GetNewPolyhedronID() { return nextPolyhedronID++; }
};

//class PolyhedronSegmentation
//{
//public:
//	Mesh* mesh = nullptr;
//	
//	Mesh cubeMesh;
//
//	Point_3 cubeCenter;
//	
//	PartitionManager* partitionManager = nullptr;
//
//	std::queue<std::shared_ptr<Polyhedron>> polyhedrons;
//
//	std::vector<std::shared_ptr<Polyhedron>> indivisiblePolyhedrons;
//
//public:
//	PolyhedronSegmentation(PartitionManager* partitionManager, Mesh* mesh);
//
//	Mesh Run(std::string outputPath);
//};