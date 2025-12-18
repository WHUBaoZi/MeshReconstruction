#pragma once

#include <omp.h>

#include <cxxopts.hpp>

#include "AlgoDebugIO.h"

#include "UtilLib.h"
#include "CGALTypes.h"
#include "PolyhedronSegmentation.h"
#include "Partition.h"
#include "Remesh.h"
#include "Ransac.h"

#include "MeshReconstruction.h"

int main(int argc, char* argv[])
{
	/*std::string inputFile = "";
	std::string outputPath = "";
	std::string fileName = "";
	if (argc > 1)
	{
		inputFile = argv[1];
		outputPath = argv[2];
	}
	else
	{
		inputFile = "../Data/TestData/test6_HoleFill.obj";
		outputPath = "../Data/Output/";
	}
	{
		size_t lastSlashPos = inputFile.find_last_of("/\\");
		size_t lastDotPos = inputFile.find_last_of('.');
		fileName = inputFile.substr(lastSlashPos + 1, lastDotPos - lastSlashPos - 1);
	}*/

	cxxopts::Options options("MeshReconstructionApp.exe", "Input Building Mesh to Reconstruction");
	const auto& rp = MeshReconstruction::ransacParams;
	const std::string epsilon_default_str = std::to_string(rp.epsilon);
	const std::string normal_threshold_default_str = std::to_string(rp.normal_threshold);
	const std::string min_points_default_str = std::to_string(rp.min_points_percent);
	const std::string cluster_epsilon_default_str = std::to_string(rp.cluster_epsilon_percent);
	options.add_options()

		("i,input",
			"Input file path (required)",
			cxxopts::value<std::string>())

		("o,output",
			"Output directory path (optional, defaults to current directory).",
			cxxopts::value<std::string>()->default_value("."))

		("e,epsilon",
			"RANSAC parameter: Maximum Euclidean distance between a point and a shape. "
			"Expressed in the same unit as input points (default: " + epsilon_default_str + ").",
			cxxopts::value<double>()->default_value(epsilon_default_str))

		("n,normal-threshold",
			"RANSAC parameter: Maximum allowed dot product between the estimated shape's normal "
			"and the point's normal (cosine of the angle). "
			"For example, cos(25°) = 0.9 (default: " + normal_threshold_default_str + ").",
			cxxopts::value<double>()->default_value(normal_threshold_default_str))

		("m,min-points",
			"RANSAC parameter: Minimum number of points required in a shape. "
			"Expressed as a fraction of the total input points "
			"(default: " + min_points_default_str + " = "
			+ std::to_string(rp.min_points_percent * 100.0) + "%).",
			cxxopts::value<double>()->default_value(min_points_default_str))

		("c,cluster-epsilon",
			"RANSAC parameter: Maximum Euclidean distance between points considered neighbors. "
			"Expressed as a fraction of the bounding box diagonal "
			"(default: " + cluster_epsilon_default_str + " = "
			+ std::to_string(rp.cluster_epsilon_percent * 100.0) + "%).",
			cxxopts::value<double>()->default_value(cluster_epsilon_default_str));

	auto result = options.parse(argc, argv);

	if (result.count("help"))
	{
		std::cout << options.help() << std::endl;
		return 0;
	}

	std::string inputFile;
	std::string fileName;
	if (result.count("input"))
	{
		inputFile = result["input"].as<std::string>();
		size_t lastSlashPos = inputFile.find_last_of("/\\");
		size_t lastDotPos = inputFile.find_last_of('.');
		fileName = inputFile.substr(lastSlashPos + 1, lastDotPos - lastSlashPos - 1);
	}	
	else
	{
		std::cerr << "Error: input file is required!" << std::endl;
		return 1;
	}

	std::string outputPath = result["output"].as<std::string>();
	MeshReconstruction::ransacParams.cluster_epsilon_percent = result["epsilon"].as<double>();
	MeshReconstruction::ransacParams.normal_threshold = result["normal-threshold"].as<double>();
	MeshReconstruction::ransacParams.min_points_percent = result["min-points"].as<double>();
	MeshReconstruction::ransacParams.cluster_epsilon_percent = result["cluster-epsilon"].as<double>();

#ifdef ENABLE_ALGO_DEBUG
	SetAlgoDebugOutputDir(outputPath + fileName);
#endif

	Mesh mesh;
	CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(inputFile, mesh);

	Mesh resultMesh = MeshReconstruction::DoReconstruction(mesh);

#ifdef ENABLE_ALGO_DEBUG
	CGAL::IO::write_OBJ(GAlgoDebugOutputDir + "ReconstructionResult.obj", resultMesh);
#endif
}