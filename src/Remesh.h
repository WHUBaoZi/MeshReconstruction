#pragma once
#include "CGALTypes.h"
#include "UtilLib.h"

namespace Remesh
{
	Mesh DoRemesh(Mesh& mesh, const std::unordered_map<int, std::unordered_set<face_descriptor>>& partitions);
}


//class RemeshPartition;
//
//class RemeshManager
//{
//public:
//	RemeshManager(Mesh* inMesh);
//
//public:
//	Mesh* mesh;
//
//	Mesh remeshedMesh;
//	
//	Mesh::Property_map<face_descriptor, int> fChartMap;
//
//	Mesh::Property_map<vertex_descriptor, bool> vCornerMap;
//
//	std::map<int, std::set<face_descriptor>> partitionMap;
//
//	std::map<int, std::unique_ptr<RemeshPartition>> remeshPartitions;
//
//public:
//	Mesh Run(std::string outputPath);
//};
//
//
//class RemeshPartition
//{
//public:
//	RemeshPartition(size_t inId, RemeshManager* inRemeshManager, const std::set<face_descriptor>& inFaces);
//
//public:
//	size_t id;
//
//	RemeshManager* remeshManager;
//
//	Mesh* mesh;
//
//	std::set<face_descriptor> faces;
//
//	std::vector<std::vector<vertex_descriptor>> boundaries;
//
//	std::vector<std::vector<vertex_descriptor>> simpBoundaries;
//
//	bool bIsValid = true;
//
//	std::set<Point_3> points;
//
//	Plane_3 fitPlane;
//
//	Vector_3 fitPlaneNormal;
//
//	Vector_3 u;
//
//	Vector_3 v;
//
//public:
//	std::vector<std::vector<halfedge_descriptor>> ExtractBoundaries(const halfedge_descriptor& startHalfedge, std::map<vertex_descriptor, std::set<halfedge_descriptor>>& sourceHalfdegesMap);
//
//	Point_2 ProjectTo2D(Point_3 point3);
//};