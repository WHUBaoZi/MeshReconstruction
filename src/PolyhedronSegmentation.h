#pragma once

#include "CGALTypes.h"
#include "UtilLib.h"
#include "Partition.h"
#include "Voxel.h"

class Polyhedron
{
public:
	Mesh polyhedronMesh;
	Point_3 centroidPoint;
	Tree polyhedronTree;
	std::vector<std::shared_ptr<Partition>> partitions;
	std::vector<PartitionSet*> partitionSets;
	std::vector<int> planeIntersectionNums;

public:
	Polyhedron(Mesh polyhedronMesh, std::vector<PartitionSet*> parentPartitions);

	Polyhedron(Mesh polyhedronMesh, std::vector<std::shared_ptr<Partition>> parentPartitions);

	void Remesh();

};

class PolyhedronSegmentation
{
public:
	Mesh* mesh = nullptr;
	
	Mesh cubeMesh;

	Point_3 cubeCenter;
	
	PartitionManager* partitionManager = nullptr;

	std::queue<std::shared_ptr<Polyhedron>> polyhedrons;

	std::vector<std::shared_ptr<Polyhedron>> indivisiblePolyhedrons;

public:
	PolyhedronSegmentation(PartitionManager* partitionManager, Mesh* mesh);

	Mesh Run(std::string outputPath);
};