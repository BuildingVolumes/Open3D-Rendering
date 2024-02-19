#pragma once

#include <unordered_map>
#include <vector>
#include <fstream>
#include <string>

#include "open3d/geometry/TriangleMesh.h"

class PolygonalMeshReader
{

public:
	//std::vector<double> vertices;
	//std::vector<int> triangle_vertices;
	//
	//std::vector<double> uvs;
	//std::vector<int> triangle_uvs;
	//
	//std::vector<double> normals;
	//std::vector<int> triangle_normals;

	PolygonalMeshReader();

	//bool ReadFromFile(std::string obj_file, open3d::geometry::TriangleMesh* existing_mesh);

	bool TriangulateQuadOBJ(std::string input_file, std::string output_file);
};