#include "Partition.h"

Partition::Partition()
{
}

Partition::Partition(int partitionIndex, PartitionManager* partitionManager, Mesh* mesh) : partitionIndex(partitionIndex), partitionManager(partitionManager), mesh(mesh)
{
	faces = partitionManager->partitionFacesMap[partitionIndex];
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

void PartitionSet::UpdateMean()
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

PartitionManager::PartitionManager(Mesh* mesh): mesh(mesh)
{
}

void PartitionManager::RunSegmentation()
{
	int kRing = 3;
	double dist = 0.3;
	double squaredDist = dist * dist;
	auto fChartMap = mesh->add_property_map<face_descriptor, int>("f:chart", -1).first;
	auto fColorMap = mesh->add_property_map<face_descriptor, CGAL::Color>("f:color", CGAL::Color(0, 0, 0)).first;
	auto fNormalMap = mesh->add_property_map<face_descriptor, Vector_3>("f:normal", CGAL::NULL_VECTOR).first;
	auto fPlanarityMap = mesh->add_property_map<face_descriptor, double>("f:planarity", -9999).first;
	auto vPlanarityMap = mesh->add_property_map<vertex_descriptor, double>("v:planarity", -9999).first;
	CGAL::Polygon_mesh_processing::compute_face_normals(*mesh, fNormalMap);
	//UtilLib::FilterMesh(*mesh, 10);

#pragma region Compute Planarity
	for (const auto& vertex : mesh->vertices())
	{
		std::vector<Point_3> points;
		for (auto vertex : UtilLib::GetKRingVertices(vertex, kRing, *mesh))
		{
			points.push_back(mesh->point(vertex));
		}
		Plane_3 plane;
		double planarity = linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());
		vPlanarityMap[vertex] = planarity;
	}
	for (auto face : mesh->faces())
	{
		double sum = 0.0;
		std::vector<vertex_descriptor> vertices = UtilLib::GetFaceVertices(face, *mesh);
		for (const auto& vertex : vertices)
		{
			sum += vPlanarityMap[vertex];
		}
		fPlanarityMap[face] = sum / vertices.size();
	}
#pragma endregion


#pragma region Partition Segmentation
	std::vector<face_descriptor> faces;
	for (const auto& face : mesh->faces())
	{
		faces.push_back(face);
	}
	// 平面度降序
	std::sort(faces.begin(), faces.end(), [&](const face_descriptor& a, const face_descriptor& b) {return fPlanarityMap[a] > fPlanarityMap[b]; });
	std::queue<face_descriptor> facesQueue;
	for (const auto& face : faces)
	{
		facesQueue.push(face);
	}
	int currentType = 0;
	CGAL::Color currentColor = UtilLib::GenerateRandomColor();
	while (!facesQueue.empty())
	{
		face_descriptor seed = facesQueue.front();
		facesQueue.pop();
		if (fChartMap[seed] != -1)
		{
			continue;
		}
		auto seedKRing = UtilLib::GetKRingFaces(seed, kRing, *mesh);
		Plane_3 plane = UtilLib::FitPlaneFromFaces(seedKRing, *mesh);
		std::set<face_descriptor> partitionFaces;
		std::set<face_descriptor> facesInLoop;
		partitionFaces.insert(seed);
		facesInLoop.insert(seed);
		fChartMap[seed] = currentType;
		fColorMap[seed] = currentColor;
		while (!facesInLoop.empty())
		{
			std::set<face_descriptor> neighborFaces;
			for (const auto& face : facesInLoop)
			{
				for (const auto& neighborFace : UtilLib::GetKRingFaces(face, 1, *mesh))
				{
					if (fChartMap[neighborFace] != -1)
					{
						continue;
					}
					bool bIsWithinTolerance = true;
					for (const auto& vertex : UtilLib::GetFaceVertices(neighborFace, *mesh))
					{
						if (CGAL::squared_distance(mesh->point(vertex), plane) > squaredDist)
						{
							bIsWithinTolerance = false;
							break;
						}
					}
					if (bIsWithinTolerance)
					{
						neighborFaces.insert(neighborFace);
						partitionFaces.insert(neighborFace);
						fChartMap[neighborFace] = currentType;
						fColorMap[neighborFace] = currentColor;
					}
				}
			}
			facesInLoop.clear();
			facesInLoop.insert(neighborFaces.begin(), neighborFaces.end());
			plane = UtilLib::FitPlaneFromFaces(partitionFaces, *mesh);
		}
		partitionFacesMap.insert(std::make_pair(currentType, partitionFaces));
		currentType++;
		currentColor = UtilLib::GenerateRandomColor();
	}
#pragma endregion
	for (const auto& pair : partitionFacesMap)
	{
		partitionMap[pair.first] = std::shared_ptr<Partition>(new Partition(pair.first, this, mesh));
	}
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

void PartitionManager::ReconstructPartitionMesh()
{
	for (auto& pair : partitionMap)
	{
		auto& partition = pair.second;
		if (partition->bIsValid)
		{
			std::map<vertex_descriptor, vertex_descriptor> v2v;
			for (auto& face : partition->faces)
			{
				std::vector<vertex_descriptor> vertices;
				vertices.reserve(3);
				for (auto vertex : CGAL::vertices_around_face(mesh->halfedge(face), *mesh))
				{
					if (v2v.find(vertex) == v2v.end())
					{
						v2v[vertex] = partition->partitionMesh.add_vertex(mesh->point(vertex));
					}
					vertices.push_back(v2v[vertex]);
				}
				partition->partitionMesh.add_face(vertices[0], vertices[1], vertices[2]);
			}
		}
	}
}
