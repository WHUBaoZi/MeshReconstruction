#pragma once
#include "CGALTypes.h"
#include "UtilLib.h"

class PartitionSet;
class PartitionManager;

class Partition
{
public:
	Partition();
	Partition(int partitionIndex, PartitionManager* partitionManager, Mesh* mesh);
public:
	Mesh* mesh = nullptr;
	int partitionIndex = -1;
	PartitionManager* partitionManager = nullptr;
	std::set<face_descriptor> faces;
	std::set<vertex_descriptor> vertices;
	std::set<Point_3> points;
	std::set<Point_3> borderPoints;
	Plane_3 fitPlane = Plane_3();
	Vector_3 fitPlaneNormal = Vector_3(0.0, 0.0, 0.0);
	Vector_3 facesAverageNormal = Vector_3(0.0, 0.0, 0.0);
	Point_3 centerPoint = Point_3(0.0, 0.0, 0.0);
	double area = 0.0;
	PartitionSet* partitionSet = nullptr;
	bool bIsValid = true;
	Mesh partitionMesh;

public:
	void CalculateAverageNormal();

	void CalculateBorderPoints();

	static bool CheckPartitionProximity(const Partition& partitionA, const Partition& partitionB, double angleTolerance);
};

class PartitionSet
{
public:
	PartitionSet();
	PartitionSet(int partitionSetIndex);
public:
	int partitionSetIndex = -1;
	std::set<Partition*> partitions;
	Plane_3 clipPlane = Plane_3();
	Vector_3 averageNormal = Vector_3(0.0, 0.0, 0.0);
	Point_3 averageCenterPoint = Point_3(0.0, 0.0, 0.0);
	double totalArea = 0.0;
	bool bIsValid = true;

public:
	void InsertPartition(Partition* partition);
	void RemovePartition(Partition* partition);
	void UpdateMean();
	std::set<Point_3> GetCoveredPoints();
	void DrawPlane();
};


class PartitionManager
{
public:
	PartitionManager();
	PartitionManager(Mesh* mesh);

public:
	Mesh* mesh = nullptr;
	std::map<int, std::set<face_descriptor>> partitionFacesMap;
	std::map<int, std::shared_ptr<Partition>> partitionMap;
	std::map<int, PartitionSet> partitionSetMap;

public:
	void RunSegmentation();

	int GetFreeIndex();

	void ReconstructPartitionMesh();
};