//#include <omp.h>
//#include <CGAL/Polygon_mesh_processing/random_perturbation.h>
//#include <openvdb/openvdb.h>
//
//#include "UtilLib.h"
//#include "CGALTypes.h"
//
//
//int main(int argc, char* argv[])
//{
//	Mesh testMesh;
//	auto v0 = testMesh.add_vertex(Point_3(0.f, 0.f, 0.f));
//	auto v1 = testMesh.add_vertex(Point_3(3.f, 0.f, 0.f));
//	auto v2 = testMesh.add_vertex(Point_3(3.f, 3.f, 0.f));
//	auto v3 = testMesh.add_vertex(Point_3(0.f, 3.f, 0.f));
//
//	auto v4 = testMesh.add_vertex(Point_3(1.f, 1.f, 0.f));
//	auto v5 = testMesh.add_vertex(Point_3(2.f, 1.f, 0.f));
//	auto v6 = testMesh.add_vertex(Point_3(2.f, 2.f, 0.f));
//	auto v7 = testMesh.add_vertex(Point_3(1.f, 2.f, 0.f));
//
//	std::vector<vertex_descriptor> polygon1;
//	std::vector<vertex_descriptor> polygon2;
//	polygon1.push_back(v0);
//	polygon1.push_back(v1);
//	polygon1.push_back(v2);
//	polygon1.push_back(v3);
//
//	polygon2.push_back(v4);
//	polygon2.push_back(v5);
//	polygon2.push_back(v6);
//	polygon2.push_back(v7);
//
//	CDT cdt;
//	std::vector<CDT::Vertex_handle> vhs1, vhs2;
//	std::map<CDT::Vertex_handle, vertex_descriptor> v2v;
//	for (size_t i = 0; i < polygon1.size(); i++)
//	{
//		Point_3 p3 = testMesh.point(polygon1[i]);
//		Point_2 p2(p3.x(), p3.y());
//		auto vh = cdt.insert(p2);
//		v2v[vh] = polygon1[i];
//		vhs1.push_back(vh);
//	}
//	for (size_t i = 0; i < vhs1.size(); i++)
//	{
//		cdt.insert_constraint(vhs1[i], vhs1[(i + 1) % vhs1.size()]);
//	}
//
//	for (size_t i = 0; i < polygon2.size(); i++)
//	{
//		Point_3 p3 = testMesh.point(polygon2[i]);
//		Point_2 p2(p3.x(), p3.y());
//		auto vh = cdt.insert(p2);
//		v2v[vh] = polygon2[i];
//		vhs2.push_back(vh);
//	}
//	for (size_t i = 0; i < vhs2.size(); i++)
//	{
//		cdt.insert_constraint(vhs2[i], vhs2[(i + 1) % vhs2.size()]);
//	}
//
//	std::unordered_map<Face_handle, bool> in_domain_map;
//	boost::associative_property_map<std::unordered_map<Face_handle, bool>> in_domain(in_domain_map);
//	CGAL::mark_domain_in_triangulation(cdt, in_domain);
//
//	for (Face_handle f : cdt.finite_face_handles())
//	{
//		if (!get(in_domain, f))
//		{
//			continue;
//		}
//		std::vector<CDT::Vertex_handle> faceVhs;
//		faceVhs.reserve(3);
//		for (int j = 0; j < 3; ++j)
//		{
//			CDT::Vertex_handle vh = f->vertex(j);
//			faceVhs.push_back(vh);
//		}
//		testMesh.add_face(v2v.at(faceVhs[0]), v2v.at(faceVhs[1]), v2v.at(faceVhs[2]));
//	}
//
//	CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "TestRemesh.obj", testMesh);
//	printf("done");
//	// test
//	openvdb::initialize();
//	openvdb::FloatGrid::Ptr grid = openvdb::FloatGrid::create();
//	grid->setName("MyFirstGrid");
//	std::cout << "Created OpenVDB Grid: " << grid->getName() << std::endl;
//
//
//	std::string inputFileName = "D:/DATA/AcademicRelevance/MeshReconstruction/MeshReconstruction/Data/Output/test10/Blender/test_10_VoxelToTriangle.obj";
//	std::string outputFileName = "D:/DATA/AcademicRelevance/MeshReconstruction/MeshReconstruction/Data/Output/test10/Blender/test_10_Remeshed.obj";
//	std::string outputFileOFFName = "D:/DATA/AcademicRelevance/MeshReconstruction/MeshReconstruction/Data/Output/test10/Blender/test_10_Region.off";
//	Mesh sm, out;
//	CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(inputFileName, sm);
//
//	//UtilLib::FilterMesh(sm, 20);
//
//	std::vector<std::size_t> region_ids(num_faces(sm));
//	std::vector<std::size_t> corner_id_map(num_vertices(sm), -1); // corner status of vertices
//	std::vector<bool> ecm(num_edges(sm), false); // mark edges at the boundary of regions
//	boost::vector_property_map<CGAL::Epick::Vector_3> normal_map; // normal of the supporting planes of the regions detected
//
//	std::size_t nb_regions =
//		CGAL::Polygon_mesh_processing::region_growing_of_planes_on_faces(sm,
//			CGAL::make_random_access_property_map(region_ids),
//			CGAL::parameters::cosine_of_maximum_angle(0.98).
//			region_primitive_map(normal_map).
//			maximum_distance(0.011));
//
//	// detect corner vertices on the boundary of planar regions
//	std::size_t nb_corners =
//		CGAL::Polygon_mesh_processing::detect_corners_of_regions(sm,
//			CGAL::make_random_access_property_map(region_ids),
//			nb_regions,
//			CGAL::make_random_access_property_map(corner_id_map),
//			CGAL::parameters::cosine_of_maximum_angle(0.98).
//			maximum_distance(0.011).
//			edge_is_constrained_map(CGAL::make_random_access_property_map(ecm)));
//
//	// run the remeshing algorithm using filled properties
//	CGAL::Polygon_mesh_processing::remesh_almost_planar_patches(sm,
//		out,
//		nb_regions, nb_corners,
//		CGAL::make_random_access_property_map(region_ids),
//		CGAL::make_random_access_property_map(corner_id_map),
//		CGAL::make_random_access_property_map(ecm),
//		CGAL::parameters::patch_normal_map(normal_map));
//	CGAL::IO::write_polygon_mesh(outputFileOFFName, out);
//
//	CGAL::IO::write_OBJ(outputFileName, out);
//}