#include <omp.h>
//#include <Python.h>

#include "UtilLib.h"
#include "CGALTypes.h"
#include "PolyhedronSegmentation.h"
#include "Partition.h"
#include "Remesh.h"

int main(int argc, char* argv[])
{
	std::chrono::steady_clock::time_point start;
	std::chrono::steady_clock::time_point end;
	std::vector<std::pair<std::string, long long>> timings;

#pragma region Input mesh file & Centralize Mesh
	std::string inputFile = "../Data/TestData/test4_HoleFill.obj";
	std::string outputPath = "../Data/Output/";
	std::string fileName = "";
	if (argc > 1)
	{
		inputFile = argv[1];
		outputPath = argv[2];
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

	//Mesh testVoxelMesh;
	//CGAL::Polygon_mesh_processing::IO::read_polygon_mesh("D:\\DATA\\AcademicRelevance\\MeshReconstruction\\MeshReconstruction\\Data\\Output\\test7_HoleFix\\test7_Modify_Voxel.obj", testVoxelMesh);

	//RemeshManager testRemeshManager(&testVoxelMesh);
	//testRemeshManager.Run(outputPath + fileName + "/Remesh/");


#pragma region Mesh Partition
	std::cout << "Start Mesh partition..." << std::endl;
	start = std::chrono::high_resolution_clock::now();
	//auto partitionFacesMap = MeshPartition(mesh);
	// Check Output Path
	if (!outputPath.empty() && !boost::filesystem::exists(outputPath))
	{
		boost::filesystem::create_directories(outputPath);
	}
	if (!boost::filesystem::exists(outputPath + fileName + "/"))
	{
		boost::filesystem::create_directories(outputPath + fileName + "/");
	}
	PartitionManager partitionManager(&mesh);
	partitionManager.RunSegmentation();
	auto fColorMap = mesh.property_map<face_descriptor, CGAL::Color>("f:color").first;
	CGAL::IO::write_PLY(outputPath + fileName + "/" + fileName + "_PartitionMesh.ply", mesh, CGAL::parameters::face_color_map(fColorMap).use_binary_mode(false));
	CGAL::IO::write_OBJ(outputPath + fileName + "/" + fileName + "_BuildingMesh.obj", mesh);
	end = std::chrono::high_resolution_clock::now();
	timings.emplace_back(std::make_pair("Mesh partition", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
	std::cout << "Mesh partition finished. Time taken: " << timings[0].second << " seconds" << std::endl;
#pragma endregion

#pragma region Optimize
	std::cout << "Start partition optimize..." << std::endl;
	start = std::chrono::high_resolution_clock::now();
	float areaTolerancePercent = 0.005;
	double totalArea = 0.0;
	std::vector<int> partitionIndices;
	for (const auto& pair : partitionManager.partitionMap)
	{
		partitionIndices.emplace_back(pair.first);
		totalArea += pair.second->area;
	}
	float toleranceArea = totalArea * areaTolerancePercent;
	std::sort(partitionIndices.begin(), partitionIndices.end(), [&](int a, int b) {return partitionManager.partitionMap[a]->area > partitionManager.partitionMap[b]->area; });
	int num = 0;
	for (size_t i = 0; i < partitionIndices.size(); i++)
	{
		auto& partition = partitionManager.partitionMap[i];
		if (partition->area < toleranceArea)
		{
			partition->bIsValid = false;
		}
		else
		{
			num++;
		}
	}
#pragma endregion



//#pragma region Merge partitions that has proximal fit plane
//	std::cout << "Start partition merge..." << std::endl;
//	start = std::chrono::high_resolution_clock::now();
//	double angleTolerance = 5.0;	// Merge angle tolerance
//	std::vector<int> partitionIndices;
//	for (const auto& pair : partitionManager.partitionMap)
//	{
//		partitionIndices.emplace_back(pair.first);
//	}
//	for (size_t i = 0; i < partitionIndices.size() - 1; i++)
//	{
//		int indexI = partitionIndices[i];
//		if (partitionManager.partitionMap[indexI].partitionSet)
//		{
//			continue;
//		}
//		int partitionSetIndex = partitionManager.GetFreeIndex();
//		partitionManager.partitionSetMap.emplace(std::make_pair(partitionSetIndex, PartitionSet(partitionSetIndex)));
//		PartitionSet* partitionSet = &partitionManager.partitionSetMap[partitionSetIndex];
//		partitionSet->InsertPartition(&partitionManager.partitionMap[indexI]);
//		for (size_t j = i + 1; j < partitionIndices.size(); j++)
//		{
//			int indexJ = partitionIndices[j];
//			if (partitionManager.partitionMap[indexJ].partitionSet)
//			{
//				continue;
//			}
//			if (Partition::CheckPartitionProximity(partitionManager.partitionMap[indexI], partitionManager.partitionMap[indexJ], angleTolerance))
//			{
//				partitionSet->InsertPartition(&partitionManager.partitionMap[indexJ]);
//			}
//		}
//	}
//	for (auto& pair : partitionManager.partitionSetMap)
//	{
//		pair.second.UpdateMean();
//	}
//	end = std::chrono::high_resolution_clock::now();
//	timings.emplace_back(std::make_pair("Partition Merge", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
//	std::cout << "Partition Merge finished. Time taken: " << timings[1].second << " seconds" << std::endl;
//#pragma endregion
//
//#pragma region Optimize
//	std::cout << "Start partition set Optimize..." << std::endl;
//	start = std::chrono::high_resolution_clock::now();
//
//	double partitionsTotalArea = 0.0;
//	std::vector<int> partitionSetIndices;
//	for (auto& pair : partitionManager.partitionSetMap)
//	{
//		partitionSetIndices.push_back(pair.first);
//		partitionsTotalArea += pair.second.totalArea;
//	}
//	std::sort(partitionSetIndices.begin(), partitionSetIndices.end(), [&](const int& indexA, const int& indexB) {return partitionManager.partitionSetMap[indexA].totalArea > partitionManager.partitionSetMap[indexB].totalArea; });
//	double thresholdArea = partitionsTotalArea * 0.005;		// Min area of a PartitionSet
//	double threshold = 0.95;	// Percent of area
//
//	double sum = 0.0;
//	for (size_t i = 0; i < partitionSetIndices.size(); i++)
//	{
//		int partitionSetIndex = partitionSetIndices[i];
//		sum += partitionManager.partitionSetMap[partitionSetIndex].totalArea;
//	}
//	double currentSum = 0.0;
//	for (size_t i = 0; i < partitionSetIndices.size(); i++)
//	{
//		int partitionSetIndex = partitionSetIndices[i];
//		double area = partitionManager.partitionSetMap[partitionSetIndex].totalArea;
//		if (area < thresholdArea)
//		{
//			break;
//		}
//		currentSum += area;
//		partitionManager.partitionSetMap[partitionSetIndex].bIsValid = true;
//		if (currentSum / sum > threshold && area < thresholdArea)
//		{
//			continue;
//		}
//	}
//	end = std::chrono::high_resolution_clock::now();
//	timings.emplace_back(std::make_pair("PartitionSet Optimize", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
//	std::cout << "Partition set optimize finished. Time taken: " << timings[2].second << " seconds" << std::endl;
//#pragma endregion

#pragma region Polyhedron Segmentation
	std::cout << "Start Polyhedron Segmentation..." << std::endl;
	start = std::chrono::high_resolution_clock::now();
	PolyhedronSegmentation polyhedronSegmentation(&partitionManager, &mesh);
	Mesh voxelMesh = polyhedronSegmentation.Run(outputPath + fileName + "/PolyhedronSegmentation/");
	end = std::chrono::high_resolution_clock::now();
	timings.emplace_back(std::make_pair("Polyhedron Segmentation", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
	std::cout << "Polyhedron Segmentation finished. Time taken: " << timings[2].second << " seconds" << std::endl;
#pragma endregion

	
	CGAL::Polygon_mesh_processing::IO::read_polygon_mesh("D:\\DATA\\AcademicRelevance\\MeshReconstruction\\MeshReconstruction\\Data\\Output\\test4_HoleFix\\test4_Modify_Voxel.obj", voxelMesh);

	RemeshManager remeshManager(&voxelMesh);
	remeshManager.Run(outputPath + fileName + "/Remesh/");

	std::cout << "All Done" << std::endl;
}