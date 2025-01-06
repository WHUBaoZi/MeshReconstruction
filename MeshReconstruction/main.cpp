#include <omp.h>

#include "UtilLib.h"
#include "CGALTypes.h"
#include "PolyhedronSegmentation.h"
#include "Partition.h"

std::map<int, std::set<face_descriptor>> PartitionByNormal(double divideAngle, Mesh& mesh);
void RemoveNoise(double ignoreSize, std::map<int, std::set<face_descriptor>>& partitionFacesMap, Mesh& mesh);
std::map<int, std::set<face_descriptor>> MeshPartition(Mesh& mesh);

int main(int argc, char* argv[])
{
	std::chrono::steady_clock::time_point start;
	std::chrono::steady_clock::time_point end;
	std::vector<std::pair<std::string, long long>> timings;

	double angleTolerance = 10.0;

#pragma region Input mesh file
	int maxPartitionsNum = 20;
	std::string inputFile = "../Data/chosenTestData/test6.obj";
	std::string outputPath = "../Data/Output/";
	std::string fileName = "";
	if (argc > 1)
	{
		maxPartitionsNum = std::stoi(argv[1]);
		inputFile = argv[2];
		outputPath = argv[3];
	}
	{
		size_t lastSlashPos = inputFile.find_last_of("/\\");
		size_t lastDotPos = inputFile.find_last_of('.');
		fileName = inputFile.substr(lastSlashPos + 1, lastDotPos - lastSlashPos - 1);
	}
	Mesh mesh;
	CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(inputFile, mesh);
	UtilLib::CentralizeMesh(mesh);
#pragma endregion


#pragma region Mesh Partition
	std::cout << "Start Mesh Partition..." << std::endl;
	start = std::chrono::high_resolution_clock::now();
	auto partitionFacesMap = MeshPartition(mesh);
	auto fColorMap = mesh.property_map<face_descriptor, CGAL::Color>("f:color").first;
	CGAL::IO::write_OFF(outputPath + fileName + "/" + fileName + "_PartitionMesh.off", mesh, CGAL::parameters::face_color_map(fColorMap));
	end = std::chrono::high_resolution_clock::now();
	timings.emplace_back(std::make_pair("Mesh Partition", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
	std::cout << "Mesh Partition finished. Time taken: " << timings[0].second << " seconds" << std::endl;
#pragma endregion


#pragma region Get Partition Planes
	std::map<int, Partition> partitionMap;
	double thresholdCosine = std::cos(10.0 * UtilLib::DEG_TO_RAD);
	for (const auto& pair : partitionFacesMap)
	{
		Partition partition(pair, &partitionMap, &mesh);
		partitionMap.emplace(std::make_pair(pair.first, partition));
	}
#pragma endregion


#pragma region Remove partitions that has proximal fit plane
	PartitionManager partitionManager(&partitionMap);

	std::vector<int> partitionIndices;
	for (const auto& pair : partitionMap)
	{
		partitionIndices.emplace_back(pair.first);
	}
	for (size_t i = 0; i < partitionIndices.size() - 1; i++)
	{
		int indexI = partitionIndices[i];
		if (partitionMap[indexI].partitionSet)
		{
			continue;
		}
		int partitionSetIndex = partitionManager.GetFreeIndex();
		partitionManager.partitionSetMap.emplace(std::make_pair(partitionSetIndex, PartitionSet(partitionSetIndex)));
		PartitionSet* partitionSet = &partitionManager.partitionSetMap[partitionSetIndex];
		partitionSet->InsertPartition(&partitionMap[indexI]);
		for (size_t j = i + 1; j < partitionIndices.size(); j++)
		{
			int indexJ = partitionIndices[j];
			if (partitionMap[indexJ].partitionSet)
			{
				continue;
			}
			if (Partition::CheckPartitionProximity(partitionMap[indexI], partitionMap[indexJ], angleTolerance))
			{
				partitionSet->InsertPartition(&partitionMap[indexJ]);
			}
		}
	}
#pragma endregion

	for (auto& pair : partitionManager.partitionSetMap)
	{
		pair.second.Activate();
	}

#pragma region Remove partitions with small area
	std::vector<int> partitionSetIndices;
	for (const auto& pair : partitionManager.partitionSetMap)
	{
		partitionSetIndices.push_back(pair.first);
	}
	std::sort(partitionSetIndices.begin(), partitionSetIndices.end(), [&](const int& indexA, const int& indexB) {return partitionManager.partitionSetMap[indexA].totalArea > partitionManager.partitionSetMap[indexB].totalArea; });

	for (size_t i = 0; i < partitionSetIndices.size(); i++)
	{
		if (i > maxPartitionsNum - 1)
		{
			partitionManager.partitionSetMap[partitionSetIndices[i]].bIsValid = false;
		}
	}
#pragma endregion

	PolyhedronSegmentation polyhedronSegmentation(&partitionManager, &mesh);
	polyhedronSegmentation.Run(outputPath + fileName + "/PolyhedronSegmentation/");

	printf("done");
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::map<int, std::set<face_descriptor>> MeshPartition(Mesh& mesh)
{
	int kRing = 3;
	auto fChartMap = mesh.add_property_map<face_descriptor, int>("f:chart", -1).first;
	auto fColorMap = mesh.add_property_map<face_descriptor, CGAL::Color>("f:color", CGAL::Color(0, 0, 0)).first;
	auto fNormalMap = mesh.add_property_map<face_descriptor, Vector_3>("f:normal", CGAL::NULL_VECTOR).first;
	auto fPlanarityMap = mesh.add_property_map<face_descriptor, double>("f:planarity", -9999).first;
	auto vPlanarityMap = mesh.add_property_map<vertex_descriptor, double>("v:planarity", -9999).first;
	CGAL::Polygon_mesh_processing::compute_face_normals(mesh, fNormalMap);

	// Mesh filter
	UtilLib::MeshFiltering(mesh, 10);

	// Planarity compute
	for (auto vertex : mesh.vertices())
	{
		vPlanarityMap[vertex] = UtilLib::ComputeVPlanarityOfKRing(vertex, kRing, mesh);
	}

	for (auto face : mesh.faces())
	{
		double sum = 0.0;
		std::vector<vertex_descriptor> vertices = UtilLib::GetVerticesAroundFace(face, mesh);
		for (const auto& vertex : vertices)
		{
			sum += vPlanarityMap[vertex];
		}
		fPlanarityMap[face] = sum / vertices.size();
	}

	// Classify
	auto partitionFacesMap = PartitionByNormal(10.0, mesh);
	RemoveNoise(20, partitionFacesMap, mesh);
	return partitionFacesMap;
}

std::map<int, std::set<face_descriptor>> PartitionByNormal(double divideAngle, Mesh& mesh)
{
	auto fChartMap = mesh.property_map<face_descriptor, int>("f:chart").first;
	auto fColorMap = mesh.property_map<face_descriptor, CGAL::Color>("f:color").first;
	auto fNormalMap = mesh.property_map<face_descriptor, Vector_3>("f:normal").first;
	auto fPlanarityMap = mesh.property_map<face_descriptor, double>("f:planarity").first;
	std::map<int, std::set<face_descriptor>> partitionFacesMap;

	std::vector<face_descriptor> faces;
	for (const auto& face : mesh.faces())
	{
		faces.push_back(face);
	}
	// 平面度降序
	std::sort(faces.begin(), faces.end(), [&](const face_descriptor& a, const face_descriptor& b) {return fPlanarityMap[a] > fPlanarityMap[b];});
	std::queue<face_descriptor> facesQueue;
	for (const auto& face: faces)
	{
		facesQueue.push(face);
	}

	double biggerDivideAngle = 3.0 * divideAngle;
	const double verticalDot = std::cos(70.0 * PI / 180.0), horizontalDot = std::cos(15.0 * PI / 180.0);
	int currentType = 0;
	CGAL::Color currentColor = UtilLib::GetRandomColor();
	while (!facesQueue.empty())
	{
		face_descriptor seed = facesQueue.front();
		facesQueue.pop();
		if (fChartMap[seed] != -1)
		{
			continue;
		}
		fChartMap[seed] = currentType;
		fColorMap[seed] = currentColor;
		partitionFacesMap[currentType].insert(seed);
		Vector_3 sumNormal = fNormalMap[seed];
		bool bIsVertical = false, bIsHorizontal = false;
		double dot = sumNormal * UtilLib::unitZ;
		if (dot < verticalDot)
		{
			bIsVertical = true;
		}
		else if (dot > horizontalDot)
		{
			bIsHorizontal = true;
		}
		int facesCount = 1;
		std::set<face_descriptor> clusterFaces;
		clusterFaces.insert(seed);
		while (!clusterFaces.empty())
		{
			std::set<face_descriptor> newClusterFaces;
			for (face_descriptor face : clusterFaces)
			{
				std::set<face_descriptor> neighbors;
				for (const auto& neighbor : CGAL::faces_around_face(mesh.halfedge(face), mesh))
				{
					if (neighbor != Mesh::null_face())
					{
						neighbors.insert(neighbor);
					}
				}
				Vector_3 unitNormal = sumNormal / facesCount;
				for (auto neighbor : neighbors)
				{
					if (fChartMap[neighbor] != -1)
					{
						continue;
					}
					Vector_3 neighborUnitNormal = fNormalMap[neighbor];
					double currDivideAngle = divideAngle;
					double currDot = neighborUnitNormal * UtilLib::unitZ;
					if (bIsVertical && currDot < verticalDot)
					{
						currDivideAngle = biggerDivideAngle;
					}
					else if (bIsHorizontal && currDot > horizontalDot)
					{
						currDivideAngle = biggerDivideAngle;
					}
					if (unitNormal * neighborUnitNormal > std::cos(currDivideAngle * PI / 180))
					{
						fChartMap[neighbor] = currentType;
						fColorMap[neighbor] = currentColor;
						partitionFacesMap[currentType].insert(neighbor);
						newClusterFaces.insert(neighbor);
						sumNormal += neighborUnitNormal;
						facesCount++;
					}
				}
			}
			clusterFaces.clear();
			clusterFaces.insert(newClusterFaces.begin(), newClusterFaces.end());
		}
		currentType++;
		currentColor = UtilLib::GetRandomColor();
	}

	return partitionFacesMap;
}

void RemoveNoise(double ignoreSize, std::map<int, std::set<face_descriptor>>& partitionFacesMap, Mesh& mesh)
{
	auto fColorMap = mesh.property_map<face_descriptor, CGAL::Color>("f:color").first;
	auto fChartMap = mesh.property_map<face_descriptor, int>("f:chart").first;
	size_t mergeCount = 0;
	std::vector<int> facesId;
	do
	{
		mergeCount = 0;
		facesId.clear();
		for (auto pair : partitionFacesMap)
		{
			int id = pair.first;
			facesId.push_back(id);
		}
		// 从小面开始消除噪声
		std::sort(facesId.begin(), facesId.end(), [&](const int& a, const int& b) {return partitionFacesMap[a].size() < partitionFacesMap[b].size(); });
		for (size_t i = 0; i < facesId.size(); i++)
		{
			int id = facesId[i];

			size_t currFaceCount = partitionFacesMap[id].size();
			int bestMergeId = -1;
			double maxDot = -99999.0;
			if (currFaceCount < ignoreSize)
			{
				auto neighborIds = UtilLib::GetPartitionNeighbors(id, partitionFacesMap, mesh);
				Vector_3 normal = UtilLib::GetPartitionAverageNormal(id, partitionFacesMap, mesh);
				for (int neighborId : neighborIds)
				{
					size_t neiFaceCount = partitionFacesMap[neighborId].size();
					Vector_3 neighborNormal = UtilLib::GetPartitionAverageNormal(neighborId, partitionFacesMap, mesh);
					double currDot = normal * neighborNormal;
					if (currDot > maxDot)
					{
						maxDot = currDot;
						bestMergeId = neighborId;
					}
				}
				if (bestMergeId != -1)
				{
					// 合并，更新对应列表
					CGAL::Color currentColor = fColorMap[*(partitionFacesMap[bestMergeId].begin())];
					for (auto face : partitionFacesMap[id])
					{
						fColorMap[face] = currentColor;
						fChartMap[face] = bestMergeId;
					}
					partitionFacesMap[bestMergeId].insert(partitionFacesMap[id].begin(), partitionFacesMap[id].end());
					partitionFacesMap.erase(id);
					mergeCount++;
				}
			}
		}
	} while (mergeCount > 0);
}