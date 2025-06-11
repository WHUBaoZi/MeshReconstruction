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