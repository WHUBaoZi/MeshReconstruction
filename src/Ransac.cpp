#include "Ransac.h"
#include "AlgoDebugIO.h"
#include "MeshReconstruction.h"


std::vector<RansacPlane> Ransac::RansacPlanes(const Mesh& mesh, std::vector<std::vector<Point_3>>& planePoints, const RansacParams& params)
{
	std::vector<Point_3> points;
	PMP::sample_triangle_mesh(mesh, std::back_inserter(points));
	return RansacPlanes(points, planePoints, params);
}

std::vector<RansacPlane> Ransac::RansacPlanes(const std::vector<Point_3>& points, std::vector<std::vector<Point_3>>& planePoints, const RansacParams& params)
{
	planePoints.clear();

#ifdef ENABLE_ALGO_DEBUG
	CGAL::IO::write_points(GAlgoDebugOutputDir + "RansacResults/Points.ply", points);
#endif // ENABLE_ALGO_DEBUG

	Pwn_vector pwn;
	pwn.reserve(points.size());
	for (const auto& p : points)
	{
		pwn.emplace_back(p, CGAL::NULL_VECTOR);
	}
	CGAL::jet_estimate_normals<CGAL::Parallel_if_available_tag>(pwn, 24, CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()).degree_fitting(1));

	// Calculate Bounding Box
	CGAL::Bbox_3 bbox;
	for (const auto& p : points) {
		bbox = bbox + p.bbox();
	}

	double dx = bbox.xmax() - bbox.xmin();
	double dy = bbox.ymax() - bbox.ymin();
	double dz = bbox.zmax() - bbox.zmin();
	double diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);

	Efficient_ransac ransac;
	Efficient_ransac::Parameters eParams;
	eParams.epsilon = params.epsilon;
	eParams.normal_threshold = params.normal_threshold;
	eParams.min_points = std::max<size_t>(1, points.size() * params.min_points_percent);
	eParams.cluster_epsilon = params.cluster_epsilon_percent * diagonal;

	ransac.set_input(pwn);
	ransac.add_shape_factory<RansacPlane>();
	ransac.detect(eParams);

#ifdef ENABLE_ALGO_DEBUG
	std::ofstream paramFile(GAlgoDebugOutputDir + "RANSAC_Params_log.txt", std::ios::app);
	if (paramFile.is_open())
	{
		paramFile << "==== New Experiment ====\n";
		paramFile << std::fixed << std::setprecision(6); // 保留小数位
		paramFile << "epsilon: " << params.epsilon << "\n";
		paramFile << "normal_threshold: " << params.normal_threshold << "\n";
		paramFile << "min_points_percent: " << params.min_points_percent << "\n";
		paramFile << "cluster_epsilon: " << params.cluster_epsilon_percent << "\n";
		paramFile << "points_size: " << points.size() << "\n";
		paramFile << "diagonal: " << diagonal << "\n";
		paramFile << "=======================\n\n";
		paramFile.close();
	}
#endif // ENABLE_ALGO_DEBUG


	std::vector<RansacPlane> ransacPlanes;
	auto shapes = ransac.shapes();
	for (auto shape : shapes)
	{
		auto plane = std::dynamic_pointer_cast<RansacPlane>(shape);
		ransacPlanes.push_back(*plane);
	}

	for (int i = 0; i < ransacPlanes.size(); i++)
	{
		auto& ransacPlane = ransacPlanes[i];
		auto indices = ransacPlane.indices_of_assigned_points();
		Vector_3 sum(0.0, 0.0, 0.0);
		planePoints.push_back(std::vector<Point_3>());
		for (auto idx : indices)
		{
			const auto& p = pwn[idx].first;
			planePoints[i].push_back(p);
			sum = sum + (p - CGAL::ORIGIN);
		}
		Point_3 center = CGAL::ORIGIN + (sum / indices.size());
		Plane_3 plane = ransacPlane;

#ifdef ENABLE_ALGO_DEBUG
		Mesh planeMesh;
		UtilLib::CreatePlaneMesh(plane, center, planeMesh);
		CGAL::IO::write_OBJ(GAlgoDebugOutputDir + "RansacResults/Planes/PlaneMesh_" + std::to_string(i) + ".obj", planeMesh);
		CGAL::IO::write_points(GAlgoDebugOutputDir + "RansacResults/Planes/PointsMesh_" + std::to_string(i) + ".ply", planePoints[i]);
#endif // ENABLE_ALGO_DEBUG
	}

#ifdef ENABLE_ALGO_DEBUG
	auto unassigned = ransac.indices_of_unassigned_points();
	std::vector<Point_3> unassignedPoints;
	for (auto index : unassigned)
	{
		unassignedPoints.push_back(points[index]);
	}
	CGAL::IO::write_points(GAlgoDebugOutputDir + "RansacResults/UnassignedPoints.ply", unassignedPoints);
#endif // ENABLE_ALGO_DEBUG
	return ransacPlanes;
}
