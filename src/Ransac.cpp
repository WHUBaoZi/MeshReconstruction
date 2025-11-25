#include "Ransac.h"
#include "AlgoDebugIO.h"

std::vector<RansacPlane> Ransac::RansacPlanes(const Mesh& mesh, std::vector<std::vector<Point_3>>& planePoints)
{
	std::vector<Point_3> points;
	PMP::sample_triangle_mesh(mesh, std::back_inserter(points));
	return RansacPlanes(points, planePoints);
}

std::vector<RansacPlane> Ransac::RansacPlanes(const std::vector<Point_3>& points, std::vector<std::vector<Point_3>>& planePoints)
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

	Efficient_ransac ransac;
	ransac.set_input(pwn);
	ransac.add_shape_factory<RansacPlane>();
	Efficient_ransac::Parameters params;
	params.epsilon = 0.2;
	params.normal_threshold = 0.98;
	params.min_points = 0.003 * points.size();
	ransac.detect(params);

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
	return ransacPlanes;
}
