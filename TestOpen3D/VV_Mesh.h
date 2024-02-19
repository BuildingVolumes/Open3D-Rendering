#pragma once

#include <vector>
#include <string>

//#include <Eigen/src/Core/Matrix.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Face_count_ratio_stop_predicate.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

//#include ""

class VV_Mesh
{
public:
	std::vector<double> vertices;
	std::vector<double> uvs;
	std::vector<double> normals;

	std::vector<int> triangles_vertices;
	std::vector<int> triangles_uvs;
	std::vector<int> triangles_normals;

	bool ValidVertices() { return (vertices.size() % 3 == 0) && (vertices.size() > 0); }
	bool ValidUVs() { return (uvs.size() % 2 == 0) && (vertices.size() > 0); }
	bool ValidNormals() { return (normals.size() % 3 == 0) && (normals.size() > 0); }

	bool ValidTrianglesVertices() { return (triangles_vertices.size() % 3 == 0) && (triangles_vertices.size() > 0); }
	bool ValidTrianglesUVs() { return (triangles_uvs.size() % 3 == 0) && (triangles_uvs.size() > 0); }
	bool ValidTrianglesNormals() { return (triangles_normals.size() % 3 == 0) && (triangles_normals.size() > 0); }

	//bool DecimateMesh(int vertex_target, int subdivision_level);

	bool WriteOBJ(std::string filename);

	bool ReadOBJ(std::string filename);

	std::shared_ptr<VV_Mesh> DecimateEdges(double decimation_ratio);
private:
	void WriteVertices(std::ofstream &savefile);
	void WriteUVs(std::ofstream &savefile);
	void WriteNormals(std::ofstream &savefile);

	void WriteTriangles(std::ofstream &savefile);

	//void GetQuadricErrorMetric(int index, int* presizedQuadric);
};