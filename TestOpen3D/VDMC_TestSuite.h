#pragma once

#include "TestSuite.h"

#include <string>
#include <vector>
#include <fstream>

#include <open3d/geometry/TriangleMesh.h>
//#include <open3d/io/>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Face_count_ratio_stop_predicate.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Surface_mesh_simplification/Edge_collapse_visitor_base.h>

//namespace PMP = CGAL::Polygon_mesh_processing;
//using K = CGAL::Exact_predicates_inexact_constructions_kernel;
//
//typedef CGAL::Simple_cartesian<double>              Kernel;
//typedef Kernel::Point_3                             Point_3;
//typedef CGAL::Surface_mesh<Point_3>                 Surface_mesh;
//typedef std::vector<std::size_t>                    Polygon_3;
//
//namespace SMS = CGAL::Surface_mesh_simplification;

struct UV_Visitor : CGAL::Surface_mesh_simplification::Edge_collapse_visitor_base<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>
{
	void OnCollapsed(const Profile& prof, vertex_descriptor vd)
	{
		std::cout << "(" << prof.v0().id() << ", " << prof.v1().id() << ") -> " << vd.id() << std::endl;
		//prof.
	}
};

class VDMC_TestSuite : public TestSuite
{
	std::string primary_data_folder = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_TriangulatedMeshes";
	std::string post_decimation_folder = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_DecimatedMeshes";
	std::string post_draco_folder;

	double decimation_ratio = 0.1;

	std::vector<std::string> input_meshes;

	void DecimateMesh(open3d::geometry::TriangleMesh* mesh, double decim_amnt);

	UV_Visitor test_visitor;

public:
	void run(int argc, char** argv);
};