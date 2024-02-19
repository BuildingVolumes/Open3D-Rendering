#include "VDMC_TestSuite.h"

#include "AdditionalUtilities.h"

//#include <algorithm>

void VDMC_TestSuite::DecimateMesh(open3d::geometry::TriangleMesh* mesh, double decim_amnt)
{
	//double decim_amnt = 1.0 / 10.0;

	CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3> sm;

	std::vector<CGAL::Simple_cartesian<double>::Point_3> verts;
	verts.resize(mesh->vertices_.size());

	auto mem_size = mesh->vertices_.size() * sizeof(Eigen::Vector3d);
	memcpy(verts.data(), mesh->vertices_.data(), mem_size);

	CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(verts, mesh->triangles_, sm);

	//SMS::

	std::cout << "Num faces: " << sm.number_of_faces() << ", should be " << mesh->triangles_.size() << std::endl;
	std::cout << "Num verts: " << sm.number_of_vertices() << ", should be " << mesh->vertices_.size() << std::endl;

	std::cout << "Other stats: " << std::endl;

	std::cout << "UVs: " << mesh->triangle_uvs_.size() << std::endl;
	std::cout << "Normals: " << mesh->vertex_normals_.size() << std::endl;

	int face_min = (sm.number_of_faces() < mesh->triangles_.size() ? sm.number_of_faces() : mesh->triangles_.size());

	//int temp_iter = 0;
	//int temp_max = 100;

	//open3d::io::WriteTriangleMeshToOBJ(post_decimation_folder + "/NotDecimatedJustCurious.obj", *mesh, true, false, true, false, true, false);

	//UV_Visitor test_visitor;

	CGAL::Surface_mesh_simplification::Edge_count_ratio_stop_predicate<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>> stop(decim_amnt);

	//int r = CGAL::Surface_mesh_simplification::edge_collapse(sm, stop, CGAL::parameters::visitor(test_visitor));
	int r = CGAL::Surface_mesh_simplification::edge_collapse(sm, stop);



	//for (auto index : sm.faces())
	//{
	//	auto hi = sm.halfedge(index);
	//	for (auto he : sm.halfedges_around_face(hi))
	//	{
	//		std::cout << he.id() << " - ";
	//	}
	//
	//	std::cout << std::endl;
	//
	//	++temp_iter;
	//
	//	if (temp_iter > temp_max)
	//	{
	//		break;
	//	}
	//}

	//for (int i = 0; i < face_min; ++i)
	//{
	//	std::cout << sm.face(sm.halfedge())
	//	std::cout << sm.faces().[0] << std::endl;
	//
	//	return;
	//}

	return;
}

void VDMC_TestSuite::run(int argc, char** argv)
{
	input_meshes.clear();

	input_meshes = GetFiles(primary_data_folder);

	std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::make_shared<open3d::geometry::TriangleMesh>();

	open3d::io::ReadTriangleMeshOptions options;

	//options.update_progress

	for (int i = 0; i < input_meshes.size(); ++i)
	{
		open3d::io::ReadTriangleMeshFromOBJ(input_meshes[i], *mesh, options);

		DecimateMesh(&(*mesh), decimation_ratio);

		return;
	}
}
