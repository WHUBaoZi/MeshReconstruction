#pragma once
#include "CGALTypes.h"
#include "UtilLib.h"

#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/IO/write_points.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>

namespace PMP = CGAL::Polygon_mesh_processing;

typedef std::pair<Point_3, Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits
<Kernel, Pwn_vector, Point_map, Normal_map>             RANSACTraits;
typedef CGAL::Shape_detection::Efficient_RANSAC<RANSACTraits> Efficient_ransac;
typedef CGAL::Shape_detection::Plane<RANSACTraits>            RansacPlane;

struct RansacParams;

namespace Ransac
{
	std::vector<RansacPlane> RansacPlanes(const Mesh& mesh, std::vector<std::vector<Point_3>>& planePoints, const RansacParams& params);

	std::vector<RansacPlane> RansacPlanes(const std::vector<Point_3>& points, std::vector<std::vector<Point_3>>& planePoints, const RansacParams& params);
}


