#pragma once

#include <omp.h>
//#include <Python.h>

#include "UtilLib.h"
#include "CGALTypes.h"
#include "PolyhedronSegmentation.h"
#include "Partition.h"
#include "Remesh.h"
#include "Ransac.h"

#include "MeshReconstruction.h"

int main(int argc, char* argv[])
{
//	std::chrono::steady_clock::time_point start;
//	std::chrono::steady_clock::time_point end;
//	std::vector<std::pair<std::string, long long>> timings;
//
//#pragma region Input mesh file & Centralize Mesh
//	std::string inputFile = "../Data/TestData/test6_HoleFill.obj";
//	std::string outputPath = "../Data/Output/";
//	std::string fileName = "";
//	if (argc > 1)
//	{
//		inputFile = argv[1];
//		outputPath = argv[2];
//	}
//	{
//		size_t lastSlashPos = inputFile.find_last_of("/\\");
//		size_t lastDotPos = inputFile.find_last_of('.');
//		fileName = inputFile.substr(lastSlashPos + 1, lastDotPos - lastSlashPos - 1);
//	}
//	Mesh mesh;
//	CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(inputFile, mesh);
//	UtilLib::CentralizeMesh(mesh);
//	CGAL::IO::write_OBJ(outputPath + fileName + "/" + fileName + "_BuildingMesh.obj", mesh);
//#pragma endregion
//
//	//Mesh testVoxelMesh;
//	//CGAL::Polygon_mesh_processing::IO::read_polygon_mesh("D:\\DATA\\AcademicRelevance\\MeshReconstruction\\MeshReconstruction\\Data\\Output\\test7_HoleFix\\test7_Modify_Voxel.obj", testVoxelMesh);
//
//	//RemeshManager testRemeshManager(&testVoxelMesh);
//	//testRemeshManager.Run(outputPath + fileName + "/Remesh/");
//
//
////#pragma region Mesh Partition
////	std::cout << "Start Mesh partition..." << std::endl;
////	start = std::chrono::high_resolution_clock::now();
////	//auto partitionFacesMap = MeshPartition(mesh);
////	// Check Output Path
////	if (!outputPath.empty() && !boost::filesystem::exists(outputPath))
////	{
////		boost::filesystem::create_directories(outputPath);
////	}
////	if (!boost::filesystem::exists(outputPath + fileName + "/"))
////	{
////		boost::filesystem::create_directories(outputPath + fileName + "/");
////	}
////	auto partitions = PartitionFunctions::PartitionByPlanarity(mesh);
////	auto fColorMap = *mesh.property_map<face_descriptor, CGAL::Color>("f:color");
////	CGAL::IO::write_PLY(outputPath + fileName + "/" + fileName + "_PartitionMesh.ply", mesh, CGAL::parameters::face_color_map(fColorMap).use_binary_mode(false));
////	CGAL::IO::write_OBJ(outputPath + fileName + "/" + fileName + "_BuildingMesh.obj", mesh);
////	end = std::chrono::high_resolution_clock::now();
////	timings.emplace_back(std::make_pair("Mesh partition", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
////	std::cout << "Mesh partition finished. Time taken: " << timings[0].second << " seconds" << std::endl;
////#pragma endregion
////
////
////#pragma region Optimize
////	std::cout << "Start partition optimize..." << std::endl;
////	start = std::chrono::high_resolution_clock::now();
////	auto optimizedPartitions = PartitionFunctions::FilterByAreaThreshold(mesh, partitions);
////	end = std::chrono::high_resolution_clock::now();
////	timings.emplace_back(std::make_pair("Partition optimize", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
////	std::cout << "Partition optimize finished. Time taken: " << timings[0].second << " seconds" << std::endl;
////#pragma endregion
//
//
//#pragma region Ransac
//	// Check Output Path
//	if (!outputPath.empty() && !boost::filesystem::exists(outputPath))
//	{
//		boost::filesystem::create_directories(outputPath);
//	}
//	if (!boost::filesystem::exists(outputPath + fileName + "/"))
//	{
//		boost::filesystem::create_directories(outputPath + fileName + "/");
//	}
//
//
//	std::vector<std::vector<Point_3>> planePoints;
//	std::vector<RansacPlane> planes = Ransac::RansacPlanes(mesh, planePoints);
//#pragma endregion
//
//	PolyhedronSegmentationFunctions::outputPath = outputPath + fileName + "/PolyhedronSegmentation/";
//	Mesh voxelMesh = PolyhedronSegmentationFunctions::DoSegmentation(mesh, planes, planePoints);
//
//	auto partitions = PartitionFunctions::PartitionByRegionGrowing(voxelMesh);
//	Mesh result = Remesh::DoRemesh(voxelMesh, partitions);
//	CGAL::IO::write_OBJ(outputPath + fileName + "/ResultMesh.obj", result);
//
////#pragma region Polyhedron Segmentation
////	std::cout << "Start Polyhedron Segmentation..." << std::endl;
////	start = std::chrono::high_resolution_clock::now();
////	PolyhedronSegmentationFunctions::outputPath = outputPath + fileName + "/PolyhedronSegmentation/";
////	Mesh segMesh = PolyhedronSegmentationFunctions::DoSegmentation(mesh, optimizedPartitions);
////	end = std::chrono::high_resolution_clock::now();
////	timings.emplace_back(std::make_pair("Polyhedron Segmentation", std::chrono::duration_cast<std::chrono::seconds>(end - start).count()));
////	std::cout << "Polyhedron Segmentation finished. Time taken: " << timings[2].second << " seconds" << std::endl;
////#pragma endregion
////
////	auto partitionResult = PartitionFunctions::PartitionByNormal(segMesh);
////	auto refineResult = PartitionFunctions::RefineByAreaThreshold(segMesh, partitionResult, 0.00005);
////	Remesh::DoRemesh(segMesh, refineResult);
//
//	//RemeshManager remeshManager(&voxelMesh);
//	//remeshManager.Run(outputPath + fileName + "/Remesh/");


	std::string inputFile = "../Data/TestData/test6_HoleFill.obj";
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

	Mesh result = MeshReconstruction::DoReconstruction(mesh);
	CGAL::IO::write_OBJ("D:/DATA/AcademicRelevance/MeshReconstruction/MeshReconstruction/Result.obj", result);

	std::cout << "All Done" << std::endl;
}