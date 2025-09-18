#pragma once
#include "CGALTypes.h"
#include "UtilLib.h"

class PartitionManager;

namespace PartitionFunctions
{
	std::unordered_map<int, std::unordered_set<face_descriptor>> PartitionByPlanarity(Mesh& mesh, int kRing = 3, double dist = 0.3);

	std::unordered_map<int, std::unordered_set<face_descriptor>> PartitionByNormal(Mesh& mesh, double thresholdAngle = 15);

	std::unordered_map<int, std::unordered_set<face_descriptor>> FilterByAreaThreshold(Mesh& mesh, const std::unordered_map<int, std::unordered_set<face_descriptor>>& partitions, float areaTolerancePercent = 0.005);

	std::unordered_map<int, std::unordered_set<face_descriptor>> RefineByAreaThreshold(Mesh& mesh, std::unordered_map<int, std::unordered_set<face_descriptor>>& partitions, float areaTolerancePercent);
}

//class Partition
//{
//public:
//	Partition();
//	Partition(int partitionIndex, PartitionManager* partitionManager, Mesh* mesh);
//public:
//	Mesh* mesh = nullptr;
//	int partitionIndex = -1;
//	PartitionManager* partitionManager = nullptr;
//	std::unordered_set<face_descriptor> faces;
//	std::set<vertex_descriptor> vertices;
//	std::set<Point_3> points;
//	std::set<Point_3> borderPoints;
//	Plane_3 fitPlane = Plane_3();
//	Vector_3 fitPlaneNormal = Vector_3(0.0, 0.0, 0.0);
//	Vector_3 facesAverageNormal = Vector_3(0.0, 0.0, 0.0);
//	Point_3 centerPoint = Point_3(0.0, 0.0, 0.0);
//	double area = 0.0;
//	bool bIsValid = true;
//	Mesh partitionMesh;
//	Mesh partitionPlaneMesh;
//
//public:
//	void CalculateAverageNormal();
//
//	void CalculateBorderPoints();
//
//	static bool CheckPartitionProximity(const Partition& partitionA, const Partition& partitionB, double angleTolerance);
//};
//
//
//class PartitionManager
//{
//public:
//	PartitionManager();
//	PartitionManager(Mesh* mesh);
//
//public:
//	Mesh* mesh = nullptr;
//	std::map<int, std::unordered_set<face_descriptor>> partitionFacesMap;
//	std::map<int, std::shared_ptr<Partition>> partitionMap;
//
//public:
//	void RunSegmentation();
//
//	void ReconstructPartitionMesh();
//};