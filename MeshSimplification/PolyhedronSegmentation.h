#pragma once

#include "CGALTypes.h"
#include "UtilLib.h"
#include "Partition.h"

class Polyhedron
{
public:
	Mesh polyhedronMesh;
	std::vector<Partition> partitions;
	std::vector<int> planeIntersectionNums;

public:
	Polyhedron(Mesh polyhedronMesh, std::vector<Partition> parentPartitions);

	Mesh DrawPlanesMesh();

	inline int GetMinIntersectionIndex(){ return std::distance(planeIntersectionNums.begin(), std::min_element(planeIntersectionNums.begin(), planeIntersectionNums.end())); }
};

class PolyhedronSegmentation
{
public:
	const Mesh* mesh = nullptr;
	
	Mesh cubeMesh;
	
	const std::map<int, Partition>* partitionMap = nullptr;

	std::set<int>* uselessPartitionIndices = nullptr;

	std::queue<Polyhedron> polyhedrons;

	std::vector<Polyhedron> indivisiblePolyhedrons;

public:
	PolyhedronSegmentation(const std::map<int, Partition>* partitionMap, std::set<int>* uselessPartitionIndices, const Mesh* mesh);

	Mesh Run(std::string outputPath);
};