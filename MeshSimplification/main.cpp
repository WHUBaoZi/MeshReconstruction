#include <omp.h>

#include "UtilLib.h"
#include "CGALTypes.h"
#include "PolyhedronSegmentation.h"

std::map<int, std::set<face_descriptor>> partitionByNormal(double divideAngle, Mesh& mesh);
void removeNoise(double ignoreSize, std::map<int, std::set<face_descriptor>>& partitionFacesMap, Mesh& mesh);
std::map<int, std::set<face_descriptor>> meshPartition(Mesh& mesh);

int main(int argc, char* argv[])
{
#pragma region Input mesh file
	std::string inputFile = "../Data/chosenTestData/test6.obj";
	std::string outputPath = "../Data/Output/";
	std::string fileName = "";
	if (argc > 1)
	{
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
#pragma endregion


#pragma region Mesh Partition
	auto partitionFacesMap = meshPartition(mesh);
	auto fColorMap = mesh.property_map<face_descriptor, CGAL::Color>("f:color").first;
	CGAL::IO::write_OFF(outputPath + fileName + "_PartitionMesh.off", mesh, CGAL::parameters::face_color_map(fColorMap));
#pragma endregion





#pragma region Get Partition Planes
///////////////////////////////////////////
// for test
	std::set<int> keysToRemove;
	for (const auto& pair : partitionFacesMap)
	{
		bool bLoopControl = false;
		for (const auto& face : pair.second)
		{
			for (const auto& vertex : CGAL::vertices_around_face(mesh.halfedge(face), mesh))
			{
				if (mesh.point(vertex).z() < 14.15)
				{
					bLoopControl = true;
					keysToRemove.insert(pair.first);
					break;
				}
			}
			if (bLoopControl)
			{
				break;
			}
		}
		if (bLoopControl)
		{
			continue;
		}
	}
	for (int key : keysToRemove)
	{
		partitionFacesMap.erase(key);
	}
///////////////////////////////////////////

	std::map<int, Plane_3> partitionPlaneMap;
	std::map<int, Vector_3> partitionNormalMap;
	for (const auto& pair : partitionFacesMap)
	{
		for (const auto& face : pair.second)
		{
			std::set<Point_3> pointsSet;
			for (const auto& vertex : CGAL::vertices_around_face(mesh.halfedge(face), mesh))
			{
				pointsSet.insert(mesh.point(vertex));
			}
			Plane_3 plane;
			CGAL::linear_least_squares_fitting_3(pointsSet.begin(), pointsSet.end(), plane, CGAL::Dimension_tag<0>());
			partitionNormalMap.insert(std::make_pair(pair.first, UtilLib::getPartitionAverageNormal(pair.first, partitionFacesMap, mesh)));
			if (plane.orthogonal_vector() * partitionNormalMap[pair.first] < 0)
			{
				partitionPlaneMap.insert(std::make_pair(pair.first, UtilLib::reversePlane(plane)));
			}
			else
			{
				partitionPlaneMap.insert(std::make_pair(pair.first, plane));
			}
		}
	}
#pragma endregion
	
	PolyhedronSegmentation polyhedronSegmentation(mesh, partitionPlaneMap);
	polyhedronSegmentation.run();

	printf("done");
}


std::map<int, std::set<face_descriptor>> meshPartition(Mesh& mesh)
{
	int kRing = 3;
	auto fChartMap = mesh.add_property_map<face_descriptor, int>("f:chart", -1).first;
	auto fColorMap = mesh.add_property_map<face_descriptor, CGAL::Color>("f:color", CGAL::Color(0, 0, 0)).first;
	auto fNormalMap = mesh.add_property_map<face_descriptor, Vector_3>("f:normal", CGAL::NULL_VECTOR).first;
	auto fPlanarityMap = mesh.add_property_map<face_descriptor, double>("f:planarity", -9999).first;
	auto vPlanarityMap = mesh.add_property_map<vertex_descriptor, double>("v:planarity", -9999).first;
	CGAL::Polygon_mesh_processing::compute_face_normals(mesh, fNormalMap);

	// Mesh filter
	UtilLib::meshFiltering(mesh, 20);

	// Planarity compute
	for (auto vertex : mesh.vertices())
	{
		vPlanarityMap[vertex] = UtilLib::computeVPlanarityOfKRing(vertex, kRing, mesh);
	}

	for (auto face : mesh.faces())
	{
		double sum = 0.0;
		std::vector<vertex_descriptor> vertices = UtilLib::getVerticesAroundFace(face, mesh);
		for (const auto& vertex : vertices)
		{
			sum += vPlanarityMap[vertex];
		}
		fPlanarityMap[face] = sum / vertices.size();
	}

	// Classify
	auto partitionFacesMap = partitionByNormal(10.0, mesh);
	removeNoise(40, partitionFacesMap, mesh);
	return partitionFacesMap;
}

std::map<int, std::set<face_descriptor>> partitionByNormal(double divideAngle, Mesh& mesh)
{
	auto fChartMap = mesh.property_map<face_descriptor, int>("f:chart").first;
	auto fColorMap = mesh.property_map<face_descriptor, CGAL::Color>("f:color").first;
	auto fNormalMap = mesh.property_map<face_descriptor, Vector_3>("f:normal").first;
	auto fPlanarityMap = mesh.property_map<face_descriptor, double>("f:planarity").first;
	std::map<int, std::set<face_descriptor>> partitionFacesMap;

	double biggerDivideAngle = 3.0 * divideAngle;
	const double verticalDot = std::cos(70.0 * M_PI / 180.0), horizontalDot = std::cos(15.0 * M_PI / 180.0);
	int currentType = 0;
	CGAL::Color currentColor = UtilLib::getRandomColor();
	std::set<face_descriptor> seeds;

	auto compare = [&](face_descriptor a, face_descriptor b) { return fPlanarityMap[a] > fPlanarityMap[b]; };
	std::set<face_descriptor, decltype(compare) > faces(compare);
	for (auto face : mesh.faces())
	{
		faces.insert(face);
	}
	while (faces.size() != 0)
	{
		auto maxPlanarityFace = *faces.begin();
		seeds.insert(maxPlanarityFace);
		fChartMap[*seeds.begin()] = currentType;
		fColorMap[*seeds.begin()] = currentColor;
		partitionFacesMap[currentType].insert(*seeds.begin());
		Vector_3 sumNormal = fNormalMap[maxPlanarityFace];
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
		while (seeds.size() != 0)
		{
			std::set<face_descriptor> neighbors;
			std::set<face_descriptor> seedDomain;
			std::set<face_descriptor> newSeeds;
			for (auto seed : seeds)
			{
				for (const auto& neighbor : CGAL::faces_around_face(mesh.halfedge(seed), mesh))
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
					if (unitNormal * neighborUnitNormal > std::cos(currDivideAngle * M_PI / 180))
					{
						fChartMap[neighbor] = currentType;
						fColorMap[neighbor] = currentColor;
						partitionFacesMap[currentType].insert(neighbor);
						newSeeds.insert(neighbor);
						sumNormal += neighborUnitNormal;
						facesCount++;
					}
				}
			}
			for (const auto& face : seeds)
			{
				faces.erase(face);
			}
			for (const auto& face : newSeeds)
			{
				faces.erase(face);
			}
			seeds.clear();
			seeds.insert(newSeeds.begin(), newSeeds.end());
		}
		currentType++;
		currentColor = UtilLib::getRandomColor();
	}
	return partitionFacesMap;
}

void removeNoise(double ignoreSize, std::map<int, std::set<face_descriptor>>& partitionFacesMap, Mesh& mesh)
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
				auto neighborIds = UtilLib::getPartitionNeighbors(id, partitionFacesMap, mesh);
				Vector_3 normal = UtilLib::getPartitionAverageNormal(id, partitionFacesMap, mesh);
				for (int neighborId : neighborIds)
				{
					size_t neiFaceCount = partitionFacesMap[neighborId].size();
					Vector_3 neiborNormal = UtilLib::getPartitionAverageNormal(neighborId, partitionFacesMap, mesh);
					double currDot = normal * neiborNormal;
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