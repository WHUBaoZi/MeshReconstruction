#include "Partition.h"

//Partition::Partition()
//{
//}
//
//Partition::Partition(int partitionIndex, PartitionManager* partitionManager, Mesh* mesh) : partitionIndex(partitionIndex), partitionManager(partitionManager), mesh(mesh)
//{
//	faces = partitionManager->partitionFacesMap[partitionIndex];
//	CalculateAverageNormal();
//	CalculateBorderPoints();
//	area = CGAL::Polygon_mesh_processing::area(faces, *mesh);
//	for (const auto& face : faces)
//	{
//		for (const auto& vertex : CGAL::vertices_around_face(mesh->halfedge(face), *mesh))
//		{
//			vertices.insert(vertex);
//			points.insert(mesh->point(vertex));
//		}
//	}
//	Vector_3 sumVector(0.0, 0.0, 0.0);
//	for (const auto& point : points)
//	{
//		sumVector = sumVector + Vector_3(point.x(), point.y(), point.z());
//	}
//	sumVector /= points.size();
//	centerPoint = Point_3(sumVector.x(), sumVector.y(), sumVector.z());
//
//	Plane_3 plane;
//	CGAL::linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());
//	if (plane.orthogonal_vector() * facesAverageNormal < 0)
//	{
//		fitPlane = UtilLib::ReversePlane(plane);
//	}
//	else
//	{
//		fitPlane = plane;
//	}
//	fitPlaneNormal = fitPlane.orthogonal_vector() / std::sqrt(fitPlane.orthogonal_vector().squared_length());
//}
//
//
//void Partition::CalculateAverageNormal()
//{
//	auto fNormalMap = *mesh->property_map<face_descriptor, Vector_3>("f:normal");
//	for (const auto& face : faces)
//	{
//		facesAverageNormal += fNormalMap[face];
//	}
//	facesAverageNormal /= faces.size();
//}
//
//void Partition::CalculateBorderPoints()
//{
//	auto fChartMap = *mesh->property_map<face_descriptor, int>("f:chart");
//	for (const auto& face : faces)
//	{
//		for (const auto& halfedge : CGAL::halfedges_around_face(mesh->halfedge(face), *mesh))
//		{
//			auto oppositeFace = mesh->face(mesh->opposite(halfedge));
//			if (oppositeFace.is_valid())
//			{
//				if (fChartMap[face] != fChartMap[oppositeFace])
//				{
//					borderPoints.insert(mesh->point(mesh->source(halfedge)));
//					borderPoints.insert(mesh->point(mesh->target(halfedge)));
//				}
//			}
//			else
//			{
//				borderPoints.insert(mesh->point(mesh->source(halfedge)));
//				borderPoints.insert(mesh->point(mesh->target(halfedge)));
//			}
//		}
//	}
//}
//
//bool Partition::CheckPartitionProximity(const Partition& partitionA, const Partition& partitionB, double angleTolerance)
//{
//	double dot = partitionA.fitPlaneNormal * partitionB.fitPlaneNormal;
//	if (std::abs(dot) > std::cos(angleTolerance * UtilLib::DEG_TO_RAD))
//	{
//		double distanceA = CGAL::squared_distance(partitionA.centerPoint, partitionB.fitPlane);
//		double distanceB = CGAL::squared_distance(partitionB.centerPoint, partitionA.fitPlane);
//		if (distanceA < 1 || distanceB < 1)
//		{
//			return true;
//		}
//	}
//	return false;
//}
//
//
//PartitionManager::PartitionManager()
//{
//}
//
//PartitionManager::PartitionManager(Mesh* mesh): mesh(mesh)
//{
//}
//
//void PartitionManager::RunSegmentation()
//{
//	int kRing = 3;
//	double dist = 0.3;
//	double squaredDist = dist * dist;
//	auto fChartMap = mesh->add_property_map<face_descriptor, int>("f:chart", -1).first;
//	auto fColorMap = mesh->add_property_map<face_descriptor, CGAL::Color>("f:color", CGAL::Color(0, 0, 0)).first;
//	auto fNormalMap = mesh->add_property_map<face_descriptor, Vector_3>("f:normal", CGAL::NULL_VECTOR).first;
//	auto fPlanarityMap = mesh->add_property_map<face_descriptor, double>("f:planarity", -9999).first;
//	auto vPlanarityMap = mesh->add_property_map<vertex_descriptor, double>("v:planarity", -9999).first;
//	CGAL::Polygon_mesh_processing::compute_face_normals(*mesh, fNormalMap);
//	//UtilLib::FilterMesh(*mesh, 10);
//
//#pragma region Compute Planarity
//	for (const auto& vertex : mesh->vertices())
//	{
//		std::vector<Point_3> points;
//		for (auto vertex : UtilLib::GetKRingVertices(vertex, kRing, *mesh))
//		{
//			points.push_back(mesh->point(vertex));
//		}
//		Plane_3 plane;
//		double planarity = linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());
//		vPlanarityMap[vertex] = planarity;
//	}
//	for (auto face : mesh->faces())
//	{
//		double sum = 0.0;
//		std::vector<vertex_descriptor> vertices = UtilLib::GetFaceVertices(face, *mesh);
//		for (const auto& vertex : vertices)
//		{
//			sum += vPlanarityMap[vertex];
//		}
//		fPlanarityMap[face] = sum / vertices.size();
//	}
//#pragma endregion
//
//
//#pragma region Partition Segmentation
//	std::vector<face_descriptor> faces;
//	for (const auto& face : mesh->faces())
//	{
//		faces.push_back(face);
//	}
//	// 平面度降序
//	std::sort(faces.begin(), faces.end(), [&](const face_descriptor& a, const face_descriptor& b) {return fPlanarityMap[a] > fPlanarityMap[b]; });
//	std::queue<face_descriptor> facesQueue;
//	for (const auto& face : faces)
//	{
//		facesQueue.push(face);
//	}
//	int currentType = 0;
//	CGAL::Color currentColor = UtilLib::GenerateRandomColor();
//	while (!facesQueue.empty())
//	{
//		face_descriptor seed = facesQueue.front();
//		facesQueue.pop();
//		if (fChartMap[seed] != -1)
//		{
//			continue;
//		}
//		auto seedKRing = UtilLib::GetKRingFaces(seed, kRing, *mesh);
//		Plane_3 plane = UtilLib::FitPlaneFromFaces(seedKRing, *mesh);
//		std::unordered_set<face_descriptor> partitionFaces;
//		std::set<face_descriptor> facesInLoop;
//		partitionFaces.insert(seed);
//		facesInLoop.insert(seed);
//		fChartMap[seed] = currentType;
//		fColorMap[seed] = currentColor;
//		while (!facesInLoop.empty())
//		{
//			std::set<face_descriptor> neighborFaces;
//			for (const auto& face : facesInLoop)
//			{
//				for (const auto& neighborFace : UtilLib::GetKRingFaces(face, 1, *mesh))
//				{
//					if (fChartMap[neighborFace] != -1)
//					{
//						continue;
//					}
//					bool bIsWithinTolerance = true;
//					for (const auto& vertex : UtilLib::GetFaceVertices(neighborFace, *mesh))
//					{
//						if (CGAL::squared_distance(mesh->point(vertex), plane) > squaredDist)
//						{
//							bIsWithinTolerance = false;
//							break;
//						}
//					}
//					if (bIsWithinTolerance)
//					{
//						neighborFaces.insert(neighborFace);
//						partitionFaces.insert(neighborFace);
//						fChartMap[neighborFace] = currentType;
//						fColorMap[neighborFace] = currentColor;
//					}
//				}
//			}
//			facesInLoop.clear();
//			facesInLoop.insert(neighborFaces.begin(), neighborFaces.end());
//			plane = UtilLib::FitPlaneFromFaces(partitionFaces, *mesh);
//		}
//		partitionFacesMap.insert(std::make_pair(currentType, partitionFaces));
//		currentType++;
//		currentColor = UtilLib::GenerateRandomColor();
//	}
//#pragma endregion
//	for (const auto& pair : partitionFacesMap)
//	{
//		partitionMap[pair.first] = std::shared_ptr<Partition>(new Partition(pair.first, this, mesh));
//	}
//}
//
//void PartitionManager::ReconstructPartitionMesh()
//{
//	for (auto& pair : partitionMap)
//	{
//		auto& partition = pair.second;
//		if (partition->bIsValid)
//		{
//			std::map<vertex_descriptor, vertex_descriptor> v2v;
//			for (auto& face : partition->faces)
//			{
//				std::vector<vertex_descriptor> vertices;
//				vertices.reserve(3);
//				for (auto vertex : CGAL::vertices_around_face(mesh->halfedge(face), *mesh))
//				{
//					if (v2v.find(vertex) == v2v.end())
//					{
//						v2v[vertex] = partition->partitionMesh.add_vertex(mesh->point(vertex));
//					}
//					vertices.push_back(v2v[vertex]);
//				}
//				partition->partitionMesh.add_face(vertices[0], vertices[1], vertices[2]);
//			}
//
//			std::vector<Point_3> points;
//			for (const auto& point : partition->partitionMesh.points())
//			{
//				points.push_back(point);
//			}
//			Mesh chMesh;
//			CGAL::convex_hull_3(points.begin(), points.end(), chMesh);
//			std::vector<Point_3> convexHullPoints;
//			float maxDistSq = 0.0;
//			for (auto it1 = chMesh.points().begin(); it1 != chMesh.points().end(); ++it1) 
//			{
//				for (auto it2 = std::next(it1); it2 != chMesh.points().end(); ++it2) 
//				{
//					float d = CGAL::squared_distance(*it1, *it2);
//					if (d > maxDistSq) 
//					{
//						maxDistSq = d;
//					}
//				}
//			}
//			partition->partitionPlaneMesh = UtilLib::CreatePlaneMesh(partition->fitPlane, partition->centerPoint, std::sqrt(maxDistSq) * 0.7);
//		}
//	}
//
//
//}

std::unordered_map<int, std::unordered_set<face_descriptor>> PartitionFunctions::PartitionByPlanarity(Mesh& mesh, int kRing, double dist)
{
	std::unordered_map<int, std::unordered_set<face_descriptor>> partitionResult;

	mesh.remove_all_property_maps();
	double squaredDist = dist * dist;
	auto fChartMap = mesh.add_property_map<face_descriptor, int>("f:chart", -1).first;
	auto fColorMap = mesh.add_property_map<face_descriptor, CGAL::Color>("f:color", CGAL::Color(0, 0, 0)).first;
	auto fNormalMap = mesh.add_property_map<face_descriptor, Vector_3>("f:normal", CGAL::NULL_VECTOR).first;
	auto fPlanarityMap = mesh.add_property_map<face_descriptor, double>("f:planarity", -9999).first;
	auto vPlanarityMap = mesh.add_property_map<vertex_descriptor, double>("v:planarity", -9999).first;
	CGAL::Polygon_mesh_processing::compute_face_normals(mesh, fNormalMap);

#pragma region Compute Planarity
	for (const auto& vertex : mesh.vertices())
	{
		std::vector<Point_3> points;
		for (auto vertex : UtilLib::GetKRingVertices(vertex, kRing, mesh))
		{
			points.push_back(mesh.point(vertex));
		}
		Plane_3 plane;
		double planarity = linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());
		vPlanarityMap[vertex] = planarity;
	}
	for (auto face : mesh.faces())
	{
		double sum = 0.0;
		std::vector<vertex_descriptor> vertices = UtilLib::GetFaceVertices(face, mesh);
		for (const auto& vertex : vertices)
		{
			sum += vPlanarityMap[vertex];
		}
		fPlanarityMap[face] = sum / vertices.size();
	}
#pragma endregion

#pragma region Partition Segmentation
	std::vector<face_descriptor> faces;
	for (const auto& face : mesh.faces())
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
		auto seedKRing = UtilLib::GetKRingFaces(seed, kRing, mesh);
		Plane_3 plane = UtilLib::FitPlaneFromFaces(seedKRing, mesh);
		std::unordered_set<face_descriptor> partitionFaces;
		std::unordered_set<face_descriptor> facesInLoop;
		partitionFaces.insert(seed);
		facesInLoop.insert(seed);
		fChartMap[seed] = currentType;
		fColorMap[seed] = currentColor;
		while (!facesInLoop.empty())
		{
			std::set<face_descriptor> neighborFaces;
			for (const auto& face : facesInLoop)
			{
				for (const auto& neighborFace : UtilLib::GetKRingFaces(face, 1, mesh))
				{
					if (fChartMap[neighborFace] != -1)
					{
						continue;
					}
					bool bIsWithinTolerance = true;
					for (const auto& vertex : UtilLib::GetFaceVertices(neighborFace, mesh))
					{
						if (CGAL::squared_distance(mesh.point(vertex), plane) > squaredDist)
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
			plane = UtilLib::FitPlaneFromFaces(partitionFaces, mesh);
		}
		partitionResult[currentType] = partitionFaces;
		currentType++;
		currentColor = UtilLib::GenerateRandomColor();
	}
#pragma endregion

	return partitionResult;
}

std::unordered_map<int, std::unordered_set<face_descriptor>> PartitionFunctions::PartitionByNormal(Mesh& mesh, double thresholdAngle)
{
	auto fNormalMapOpt = mesh.property_map<face_descriptor, Vector_3>("f:normal");
	Mesh::Property_map<face_descriptor, Vector_3> fNormalMap;
	if (!fNormalMapOpt)
	{
		fNormalMap = mesh.add_property_map<face_descriptor, Vector_3>("f:normal", CGAL::NULL_VECTOR).first;
		CGAL::Polygon_mesh_processing::compute_face_normals(mesh, fNormalMap);
	}
	else
	{
		fNormalMap = *fNormalMapOpt;
	}
	auto fChartMapOpt = mesh.property_map<face_descriptor, int>("f:chart");
	Mesh::Property_map<face_descriptor, int> fChartMap;
	if (!fChartMapOpt)
	{
		fChartMap = mesh.add_property_map<face_descriptor, int>("f:chart", -1).first;
	}
	else
	{
		fChartMap = *fChartMapOpt;
	}
	auto fColorMapOpt = mesh.property_map<face_descriptor, CGAL::Color>("f:color");
	Mesh::Property_map<face_descriptor, CGAL::Color> fColorMap;
	if (!fColorMapOpt)
	{
		fColorMap = mesh.add_property_map<face_descriptor, CGAL::Color>("f:color", CGAL::Color(0, 0, 0)).first;
	}
	else
	{
		fColorMap = *fColorMapOpt;
	}

	std::unordered_map<int, std::unordered_set<face_descriptor>> partitions;
	std::unordered_map<int, Vector_3> partitionsNormal;
	size_t currenId = 0;
	CGAL::Color currentColor = UtilLib::GenerateRandomColor();
	const double cosThreshold = std::cos(thresholdAngle * UtilLib::DEG_TO_RAD);

	for (const auto& seed : mesh.faces())
	{
		if (fChartMap[seed] != size_t(-1))
			continue;

		std::queue<face_descriptor> facesQueue;
		fChartMap[seed] = currenId;
		fColorMap[seed] = currentColor;
		partitions[currenId] = {};
		partitions[currenId].insert(seed);
		facesQueue.push(seed);
		Vector_3 avgNormal = fNormalMap[seed];

		while (!facesQueue.empty())
		{
			face_descriptor face = facesQueue.front();
			facesQueue.pop();

			for (const auto& neighborFace : mesh.faces_around_face(mesh.halfedge(face)))
			{
				if (fChartMap[neighborFace] != size_t(-1))
				{
					continue;
				}
				Vector_3 neighborNormal = fNormalMap[neighborFace];
				double dot = (avgNormal * neighborNormal);
				double lenProduct = std::sqrt(avgNormal.squared_length() * neighborNormal.squared_length());
				if (lenProduct == 0) continue;

				double cosAngle = dot / lenProduct;
				if (cosAngle > cosThreshold)
				{
					fChartMap[neighborFace] = currenId;
					fColorMap[neighborFace] = currentColor;
					partitions[currenId].insert(neighborFace);
					facesQueue.push(neighborFace);
					avgNormal = avgNormal + neighborNormal;
				}
			}
		}
		partitionsNormal[currenId] = avgNormal / std::sqrt(avgNormal.squared_length());
		currenId++;
		currentColor = UtilLib::GenerateRandomColor();
	}
	CGAL::IO::write_PLY(TEST_OUTPUT_PATH + "OriginalClassifyMesh.ply", mesh, CGAL::parameters::face_color_map(fColorMap).use_binary_mode(false));
	return partitions;
}


std::unordered_map<int, std::unordered_set<face_descriptor>> PartitionFunctions::FilterByAreaThreshold(Mesh& mesh, const std::unordered_map<int, std::unordered_set<face_descriptor>>& partitions, float areaTolerancePercent)
{
	double totalArea = 0.0;
	std::vector<int> partitionIndices;
	std::unordered_set<int> validPartitionIDs;
	std::unordered_map<int, double> partitionAreas;
	// Can be parallel
	for (const auto& pair : partitions)
	{
		int partitionID = pair.first;
		const auto& partition = pair.second;
		partitionIndices.emplace_back(partitionID);
		double area = CGAL::Polygon_mesh_processing::area(partition, mesh);
		partitionAreas[partitionID] = area;
		totalArea += area;
	}
	float toleranceArea = totalArea * areaTolerancePercent;
	//std::sort(partitionIndices.begin(), partitionIndices.end(), [&](int a, int b) {return partitionAreas[a] > partitionAreas[b]; });
	for (int i = 0; i < partitionIndices.size(); i++)
	{
		int partitionID = partitionIndices[i];
		if (partitionAreas[partitionID] > toleranceArea)
		{
			validPartitionIDs.insert(partitionID);
		}
	}
	std::unordered_map<int, std::unordered_set<face_descriptor>> result;
	for (int validPartitionID : validPartitionIDs)
	{
		result[validPartitionID] = partitions.at(validPartitionID);
	}

	return result;
}



std::unordered_map<int, std::unordered_set<face_descriptor>> PartitionFunctions::RefineByAreaThreshold(Mesh& mesh, std::unordered_map<int, std::unordered_set<face_descriptor>>& partitions, float areaTolerancePercent)
{
	auto fChartMap = *mesh.property_map<face_descriptor, int>("f:chart");
	auto fColorMap = *mesh.property_map<face_descriptor, CGAL::Color>("f:color");
	std::vector<int> smallPartitionIDs;
	std::vector<int> partitionIDs;
	std::map<int, double> partitionsArea;
	std::map<int, double> partitionsPerimeter;
	double totalArea = 0;
	for (auto& pair : partitions)
	{
		int partitionID = pair.first;
		auto& partition = pair.second;
		double perimeter = 0.0;
		double area = 0.0;
		auto boundaries = UtilLib::ExtractBoundaries(mesh, partition);
		for (const auto& boundary : boundaries)
		{
			int size = boundary.size();
			for (int i = 0; i < size; i++)
			{
				vertex_descriptor v = boundary[i];
				vertex_descriptor nextV = boundary[(i + 1) % size];
				Point_3 p = mesh.point(v);
				Point_3 nextP = mesh.point(nextV);
				perimeter += std::sqrt(CGAL::squared_distance(p, nextP));
			}
		}
		partitionIDs.push_back(partitionID);
		partitionsArea[partitionID] = area;
		partitionsPerimeter[partitionID] = perimeter;
		totalArea += area;
	}

	std::sort(partitionIDs.begin(), partitionIDs.end(), [&](const int& indexA, const int& indexB) {return partitionsArea.at(indexA) > partitionsArea.at(indexB); });
	double thresholdArea = areaTolerancePercent * totalArea;
	double sumArea = 0;
	for (int i = 0; i < partitionIDs.size(); i++)
	{
		int partitionID = partitionIDs[i];
		double area = partitionsArea[partitionID];
		if (area < thresholdArea)
		{
			smallPartitionIDs.push_back(partitionID);
		}
		sumArea += area;
	}

	// Merge SmallPartition to neighbor partition
	for (int partitionID : smallPartitionIDs)
	{
		// neighbor partition ID -> appear count
		std::unordered_map<int, int> neighborCountMap;
		const auto& partition = partitions.at(partitionID);
		for (const auto& face : partition)
		{
			for (const auto& neighbor : CGAL::faces_around_face(mesh.halfedge(face), mesh))
			{
				int neighborID = fChartMap[neighbor];
				if (neighborID == partitionID)
				{
					continue;
				}
				neighborCountMap[neighborID]++;
			}
		}
		int maxCountID = -1;
		int maxCount = 0;
		for (auto& pair : neighborCountMap)
		{
			int neighborID = pair.first;
			int count = pair.second;
			if (count > maxCount)
			{
				maxCountID = neighborID;
				maxCount = count;
			}
		}
		if (maxCountID != -1)
		{
			auto& neighborPartition = partitions.at(maxCountID);
			CGAL::Color newColor = fColorMap[*neighborPartition.begin()];
			for (auto& face : partitions.at(partitionID))
			{
				neighborPartition.insert(face);
				fChartMap[face] = maxCountID;
				fColorMap[face] = newColor;
			}
			partitions.erase(partitionID);
		}
	}
	return partitions;
}