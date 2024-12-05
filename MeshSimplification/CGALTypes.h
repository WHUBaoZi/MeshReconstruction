#pragma once
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangle_3.h>

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/internal/Snapping/snap.h>

#include <CGAL/boost/graph/IO/OBJ.h>
#include <CGAL/boost/graph/IO/OFF.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

#include <CGAL/Shape_regularization/regularize_contours.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/linear_least_squares_fitting_3.h>

//typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Plane_3 Plane_3;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Triangle_3 Triangle_3;
typedef Kernel::Line_3 Line_3;
typedef Kernel::Ray_3 Ray;

typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron_3;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;
typedef Mesh::Edge_index edge_descriptor;
typedef Mesh::Halfedge_index halfedge_descriptor;

typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;

typedef CGAL::Polygon_2<Kernel> Polygon_2;

