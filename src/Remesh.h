#pragma once
#include "CGALTypes.h"
#include "UtilLib.h"


class RemeshPartition;
class RemeshPoint;

class RemeshManager
{
public:
	RemeshManager(Mesh* inMesh);

public:
	Mesh* mesh;

	Mesh remeshedMesh;
	
	Mesh::Property_map<face_descriptor, size_t> fChartMap;

	Mesh::Property_map<vertex_descriptor, bool> vCornerMap;

	std::map<size_t, std::set<face_descriptor>> partitionMap;

	std::map<size_t, std::unique_ptr<RemeshPartition>> remeshPartitions;

	// vertex_descriptor in mesh -> RemeshPoint
	std::map<vertex_descriptor, std::unique_ptr<RemeshPoint>> remeshPoints;

public:
	Mesh* Run(std::string outputPath);
};


class RemeshPartition
{
public:
	RemeshPartition(size_t inId, RemeshManager* inRemeshManager, const std::set<face_descriptor>& inFaces);

public:
	size_t id;

	RemeshManager* remeshManager;

	Mesh* mesh;

	std::set<face_descriptor> faces;

	std::vector<std::vector<vertex_descriptor>> boundaries;

	std::vector<std::vector<vertex_descriptor>> simpBoundaries;

	bool bIsValid = true;

	std::set<Point_3> points;

	Plane_3 fitPlane;

	Vector_3 fitPlaneNormal;

	Vector_3 u;

	Vector_3 v;

	// cdtVertex -> vertex in mesh
	std::map<CDT::Vertex_handle, vertex_descriptor> v2v;

public:
	std::vector<std::vector<halfedge_descriptor>> ExtractBoundaries(const halfedge_descriptor& startHalfedge, std::map<vertex_descriptor, std::set<halfedge_descriptor>>& sourceHalfdegesMap);

	Point_2 ProjectTo2D(Point_3 point3);

	void ApplyRemesh();
};

class RemeshPoint
{
public:
	RemeshPoint(vertex_descriptor inVertex, RemeshManager* inRemeshManager);

public:
	RemeshManager* remeshManager;

	Point_3 point3;

	vertex_descriptor vertex;

	Point_2 point2;

	CDT::Vertex_handle cdtVertex;

	vertex_descriptor newVertex;

public:
	void Update(Point_2 inPoint2, CDT::Vertex_handle inCdtVertex);
};