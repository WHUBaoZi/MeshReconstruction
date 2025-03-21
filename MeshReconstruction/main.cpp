#include <omp.h>
//#include <Python.h>

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

#pragma region Input mesh file & Centralize Mesh
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
	std::cout << "Start Mesh partition..." << std::endl;
	start = std::chrono::high_resolution_clock::now();
	//auto partitionFacesMap = MeshPartition(mesh);
	// Check Output Path
	if (!outputPath.empty() && !boost::filesystem::exists(outputPath))
	{
		boost::filesystem::create_directories(outputPath);
	}
	PartitionManager partitionManager(&mesh);
	partitionManager.PartitionSegmentation();
	auto fColorMap = mesh.property_map<face_descriptor, CGAL::Color>("f:color").first;
	CGAL::IO::write_OFF(outputPath + fileName + "/" + fileName + "_PartitionMesh.off", mesh, CGAL::parameters::face_color_map(fColorMap));
	CGAL::IO::write_OBJ(outputPath + fileName + "/" + fileName + "_BuildingMesh.obj", mesh);
	end = std::chrono::high_resolution_clock::now();
	timings.emplace_back(std::make_pair("Mesh partition", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
	std::cout << "Mesh partition finished. Time taken: " << timings[0].second << " seconds" << std::endl;
#pragma endregion

#pragma region Merge partitions that has proximal fit plane
	std::cout << "Start partition merge..." << std::endl;
	start = std::chrono::high_resolution_clock::now();
	std::vector<int> partitionIndices;
	for (const auto& pair : partitionManager.partitionMap)
	{
		partitionIndices.emplace_back(pair.first);
	}
	for (size_t i = 0; i < partitionIndices.size() - 1; i++)
	{
		int indexI = partitionIndices[i];
		if (partitionManager.partitionMap[indexI].partitionSet)
		{
			continue;
		}
		int partitionSetIndex = partitionManager.GetFreeIndex();
		partitionManager.partitionSetMap.emplace(std::make_pair(partitionSetIndex, PartitionSet(partitionSetIndex)));
		PartitionSet* partitionSet = &partitionManager.partitionSetMap[partitionSetIndex];
		partitionSet->InsertPartition(&partitionManager.partitionMap[indexI]);
		for (size_t j = i + 1; j < partitionIndices.size(); j++)
		{
			int indexJ = partitionIndices[j];
			if (partitionManager.partitionMap[indexJ].partitionSet)
			{
				continue;
			}
			if (Partition::CheckPartitionProximity(partitionManager.partitionMap[indexI], partitionManager.partitionMap[indexJ], angleTolerance))
			{
				partitionSet->InsertPartition(&partitionManager.partitionMap[indexJ]);
			}
		}
	}
	for (auto& pair : partitionManager.partitionSetMap)
	{
		pair.second.UpdateMean();
	}
	end = std::chrono::high_resolution_clock::now();
	timings.emplace_back(std::make_pair("Partition Merge", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
	std::cout << "Partition Merge finished. Time taken: " << timings[1].second << " seconds" << std::endl;
#pragma endregion

#pragma region Optimize
	std::cout << "Start partition set Optimize..." << std::endl;
	start = std::chrono::high_resolution_clock::now();

	double minArea = 2;		// Min area of a PartitionSet
	double threshold = 0.9;
	std::vector<int> partitionSetIndices;
	for (auto& pair : partitionManager.partitionSetMap)
	{
		partitionSetIndices.push_back(pair.first);
	}
	std::sort(partitionSetIndices.begin(), partitionSetIndices.end(), [&](const int& indexA, const int& indexB) {return partitionManager.partitionSetMap[indexA].totalArea > partitionManager.partitionSetMap[indexB].totalArea; });
	
	double sum = 0.0;
	for (size_t i = 0; i < partitionSetIndices.size(); i++)
	{
		int partitionSetIndex = partitionSetIndices[i];
		sum += partitionManager.partitionSetMap[partitionSetIndex].totalArea;
	}
	double currentSum = 0.0;
	for (size_t i = 0; i < partitionSetIndices.size(); i++)
	{
		int partitionSetIndex = partitionSetIndices[i];
		double area = partitionManager.partitionSetMap[partitionSetIndex].totalArea;
		if (area < minArea)
		{
			break;
		}
		currentSum += area;
		partitionManager.partitionSetMap[partitionSetIndex].bIsValid = true;
		if (currentSum / sum > threshold)
		{
			break;
		}
	}
	end = std::chrono::high_resolution_clock::now();
	timings.emplace_back(std::make_pair("PartitionSet Optimize", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
	std::cout << "Partition set optimize finished. Time taken: " << timings[2].second << " seconds" << std::endl;

	/*std::ofstream outFile("../Python/Source/PartitionSetAreas.txt");
	if (!outFile) 
	{
		std::cerr << "Error: Cannot open file for writing!" << std::endl;
		return 1;
	}
	for (const auto& val : partitionSetArea)
	{
		outFile << val << std::endl;
	}
	outFile.close();
	Py_Initialize();
	if (!Py_IsInitialized())
	{
		std::cerr << "Failed to initialize Python!" << std::endl;
		return -1;
	}
	PyRun_SimpleString("import os,sys");
	PyRun_SimpleString("sys.path.append('../Python/Source')");
	PyObject* pModule;
	PyObject* pFunction;
	PyObject* pArgs;
	PyObject* pRetValue;
	pModule = PyImport_ImportModule("PartitionOptimize");
	if (!pModule)
	{
		std::cerr << "Failed to import Module!" << std::endl;
		return -1;
	}
	pFunction = PyObject_GetAttrString(pModule, "partition_optimize");
	if (!pFunction)
	{
		std::cerr << "Get python function failed!" << std::endl;
		return -1;
	}
	PyObject* pListArea = PyList_New(partitionSetArea.size());
	for (size_t i = 0; i < partitionSetArea.size(); i++)
	{
		PyList_SetItem(pListArea, i, PyFloat_FromDouble(partitionSetArea[i]));
	}
	pArgs = PyTuple_New(1);
	PyTuple_SetItem(pArgs, 0, pListArea);
	std::vector<int> retValue;
	pRetValue = PyObject_CallObject(pFunction, pArgs);
	if (pRetValue && PyList_Check(pRetValue))
	{
		Py_ssize_t size = PyList_Size(pRetValue);
		retValue.reserve(size);
		for (Py_ssize_t i = 0; i < size; i++)
		{
			retValue.push_back(PyLong_AsLong(PyList_GetItem(pRetValue, i)));
		}
	}
	else
	{
		PyErr_Print();
	}
	Py_Finalize();*/
#pragma endregion


	std::cout << "Start Polyhedron Segmentation..." << std::endl;
	start = std::chrono::high_resolution_clock::now();
	PolyhedronSegmentation polyhedronSegmentation(&partitionManager, &mesh);
	polyhedronSegmentation.Run(outputPath + fileName + "/PolyhedronSegmentation/");
	end = std::chrono::high_resolution_clock::now();
	timings.emplace_back(std::make_pair("Polyhedron Segmentation", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
	std::cout << "Polyhedron Segmentation finished. Time taken: " << timings[2].second << " seconds" << std::endl;

	std::cout << "All Done" << std::endl;
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
	UtilLib::FilterMesh(mesh, 10);

	// Planarity compute
	for (auto vertex : mesh.vertices())
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
	CGAL::Color currentColor = UtilLib::GenerateRandomColor();
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
		currentColor = UtilLib::GenerateRandomColor();
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