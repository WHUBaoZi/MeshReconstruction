#pragma once
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Segment_3.h>

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

#include <CGAL/boost/graph/IO/OBJ.h>
#include <CGAL/boost/graph/IO/OFF.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>

#include <CGAL/Shape_regularization/regularize_contours.h>
#include <CGAL/linear_least_squares_fitting_3.h>

typedef CGAL::Simple_cartesian<double> Kernel;

typedef Kernel::Point_3 Point3;
typedef Kernel::Point_2 Point2;
typedef Kernel::Plane_3 Plane3;
typedef Kernel::Line_3 Line3;
typedef Kernel::Segment_2 Segment2;
typedef Kernel::Segment_3 Segment3;
typedef Kernel::Vector_2 Vector2;
typedef Kernel::Vector_3 Vector3;
typedef Kernel::Triangle_3 Triangle3;

typedef CGAL::Triangulation_2<Kernel> Triangulation;
typedef Triangulation::Vertex_handle Vertex_handle;
typedef Triangulation::Face_handle Face_handle;

typedef CGAL::Polygon_2<Kernel> Polygon2;
typedef CGAL::Surface_mesh<Point3> Mesh;

typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;
typedef Mesh::Edge_index edge_descriptor;
typedef Mesh::Halfedge_index halfedge_descriptor;

typedef CGAL::Face_filtered_graph<Mesh> filtered_graph;
typedef CGAL::Triple<int, int, int> TirangleInt;

