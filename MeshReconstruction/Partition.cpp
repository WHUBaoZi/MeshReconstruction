#include "Partition.h"

Partition::Partition()
{
}

Partition::Partition(std::pair<int, std::set<face_descriptor>> indexFacesPair, std::map<int, Partition>* partitionMap, Mesh* mesh) : partitionIndex(indexFacesPair.first), faces(indexFacesPair.second), partitionMap(partitionMap), mesh(mesh)
{
	CalculateAverageNormal();
	CalculateBorderPoints();
	area = CGAL::Polygon_mesh_processing::area(faces, *mesh);
	for (const auto& face : faces)
	{
		for (const auto& vertex : CGAL::vertices_around_face(mesh->halfedge(face), *mesh))
		{
			vertices.insert(vertex);
			points.insert(mesh->point(vertex));
		}
	}
	Vector_3 sumVector(0.0, 0.0, 0.0);
	for (const auto& point : points)
	{
		sumVector = sumVector + Vector_3(point.x(), point.y(), point.z());
	}
	sumVector /= points.size();
	centerPoint = Point_3(sumVector.x(), sumVector.y(), sumVector.z());

	Plane_3 plane;
	CGAL::linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());
	if (plane.orthogonal_vector() * facesAverageNormal < 0)
	{
		fitPlane = UtilLib::ReversePlane(plane);
	}
	else
	{
		fitPlane = plane;
	}
	fitPlaneNormal = fitPlane.orthogonal_vector() / std::sqrt(fitPlane.orthogonal_vector().squared_length());
}


void Partition::CalculateAverageNormal()
{
	auto fNormalMap = mesh->property_map<face_descriptor, Vector_3>("f:normal").first;
	for (const auto& face : faces)
	{
		facesAverageNormal += fNormalMap[face];
	}
	facesAverageNormal /= faces.size();
}

void Partition::CalculateBorderPoints()
{
	auto fChartMap = mesh->property_map<face_descriptor, int>("f:chart").first;
	for (const auto& face : faces)
	{
		for (const auto& halfedge : CGAL::halfedges_around_face(mesh->halfedge(face), *mesh))
		{
			auto oppositeFace = mesh->face(mesh->opposite(halfedge));
			if (oppositeFace.is_valid())
			{
				if (fChartMap[face] != fChartMap[oppositeFace])
				{
					borderPoints.insert(mesh->point(mesh->source(halfedge)));
					borderPoints.insert(mesh->point(mesh->target(halfedge)));
				}
			}
			else
			{
				borderPoints.insert(mesh->point(mesh->source(halfedge)));
				borderPoints.insert(mesh->point(mesh->target(halfedge)));
			}
		}
	}
}

bool Partition::CheckPartitionProximity(const Partition& partitionA, const Partition& partitionB, double angleTolerance)
{
	double dot = partitionA.fitPlaneNormal * partitionB.fitPlaneNormal;
	if (std::abs(dot) > std::cos(angleTolerance * UtilLib::DEG_TO_RAD))
	{
		double distanceA = CGAL::squared_distance(partitionA.centerPoint, partitionB.fitPlane);
		double distanceB = CGAL::squared_distance(partitionB.centerPoint, partitionA.fitPlane);
		if (distanceA < 1 || distanceB < 1)
		{
			return true;
		}
	}
	return false;
}

PartitionSet::PartitionSet()
{
}

PartitionSet::PartitionSet(int partitionSetIndex): partitionSetIndex(partitionSetIndex)
{
}

void PartitionSet::InsertPartition(Partition* partition)
{
	partitions.insert(partition);
	partition->partitionSet = this;
	totalArea += partition->area;
}

void PartitionSet::RemovePartition(Partition* partition)
{
	partitions.erase(partition);
	partition->partitionSet = nullptr;
	totalArea -= partition->area;
}

void PartitionSet::Activate()
{
	averageNormal = Vector_3(0.0, 0.0, 0.0);
	Vector_3 sumCenterVector(0.0, 0.0, 0.0);
	Vector_3 referenceNormal = (*partitions.begin())->fitPlaneNormal;
	for (const auto& partition : partitions)
	{
		if (referenceNormal * partition->fitPlaneNormal > 0)
		{
			averageNormal += partition->fitPlaneNormal;
		}
		else
		{
			averageNormal -= partition->fitPlaneNormal;
		}
		sumCenterVector += UtilLib::PointToVector(partition->centerPoint);
	}
	averageNormal /= partitions.size();
	averageCenterPoint = UtilLib::VectorToPoint(sumCenterVector / partitions.size());
	clipPlane = Plane_3(averageCenterPoint, averageNormal);
}

std::set<Point_3> PartitionSet::GetCoveredPoints()
{
	std::set<Point_3> borderPoints;
	for (const auto& partition : partitions)
	{
		borderPoints.insert(partition->borderPoints.begin(), partition->borderPoints.end());
	}
	return borderPoints;
}


PartitionManager::PartitionManager()
{
}

PartitionManager::PartitionManager(const std::map<int, Partition>* partitionMap): partitionMap(partitionMap)
{
}

int PartitionManager::GetFreeIndex()
{
	int index = 0;
	while (partitionSetMap.find(index) != partitionSetMap.end()) 
	{
		++index;
	}
	return index;
}
