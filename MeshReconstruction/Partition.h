#pragma once
#include "CGALTypes.h"
#include "UtilLib.h"

class Partition
{
public:
	Partition();
	Partition(std::set<face_descriptor> faces, const Mesh* mesh);

public:
	const Mesh* mesh = nullptr;
	int index = -1;
	std::set<face_descriptor> faces;
	std::set<vertex_descriptor> vertices;
	std::set<Point_3> points;
	Plane_3 fitPlane = Plane_3();
	Vector_3 fitPlaneNormal = Vector_3();
	Vector_3 facesAverageNormal = Vector_3(0.0, 0.0, 0.0);
	Point_3 centerPoint = Point_3(0.0, 0.0, 0.0);
	double area = 0.0;

public:
	void CalculateAverageNormal();

	static bool CheckPartitionProximity(const Partition& partitionA, const Partition& partitionB);
};

