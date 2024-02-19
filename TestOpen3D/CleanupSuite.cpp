#include "CleanupSuite.h"

#include "AdditionalUtilities.h"
#include "open3d/geometry/IntersectionTest.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/boost/graph/iterator.h>


#include <boost/lexical_cast.hpp>

#include <CGAL/IO/OBJ.h>

#include <iostream>
#include <iterator>
#include <string>
#include <tuple>
#include <vector>

//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
//typedef Kernel::Point_3                                     Point;
//typedef CGAL::Surface_mesh<Point>                           Mesh;
//
//typedef boost::graph_traits<Mesh>::vertex_descriptor        vertex_descriptor;
//typedef boost::graph_traits<Mesh>::halfedge_descriptor      halfedge_descriptor;
//typedef boost::graph_traits<Mesh>::face_descriptor          face_descriptor;
//
//namespace PMP = CGAL::Polygon_mesh_processing;

void CleanupSuite::TestHoleRemoval()
{
	open3d::io::ReadTriangleMeshOptions params;

	std::string path_to_mesh = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_TriangulatedMeshes/Triangulated_000000.obj";

	std::string save_name = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_VoxeledMeshes/Triangulated_FILLED_000000.obj";

	open3d::geometry::TriangleMesh mesh;

	open3d::io::ReadTriangleMeshFromOBJ(path_to_mesh, mesh, params);

	auto sm = dm.GetOpen3DMeshAsCGAL_EPICK_P3(&mesh);

	std::cout << "Current tri/vert count: " << sm->num_faces() << ", " << sm->number_of_vertices() << std::endl;
	std::cout << std::endl;

	int patch_limit = 1;
	unsigned int nb_holes = 0;

	//TestInhouseHoleRemoval(&mesh);

	//int counter = 0;
	//for (boost::graph_traits<CGAL::Surface_mesh<CGAL::Epick::Point_3>>::vertex_descriptor v : vertices(*sm))
	//{
	//	if (CGAL::Polygon_mesh_processing::is_non_manifold_vertex(v, *sm))
	//	{
	//		std::cout << "vertex " << v << " is non-manifold" << std::endl;
	//		++counter;
	//	}
	//}
	//
	//std::cout << counter << " non-manifold vertices!" << std::endl;

	for (int i = 0; i < patch_limit; ++i)
	{
		int new_path = RemoveHoles(&(*sm));

		if (new_path <= 0)
		{
			std::cout << (i + 1) << "loops run" << std::endl;
			break;
		}

		nb_holes += new_path;
	}

	//unsigned int nb_holes = RemoveHoles(&(*sm));
	//nb_holes += RemoveHoles(&(*sm));

	std::cout << nb_holes << " total holes have been filled" << std::endl;
	
	std::cout << "Final tri/vert count: " << sm->num_faces() << ", " << sm->number_of_vertices() << std::endl;

	//sm->points();
	CGAL::IO::write_OBJ(save_name, *sm);
}

unsigned int CleanupSuite::RemoveHoles(CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3> *sm)
{
	unsigned int nb_holes = 0;
	std::vector<boost::graph_traits<CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>>::halfedge_descriptor> border_cycles;
	// collect one halfedge per boundary cycle
	CGAL::Polygon_mesh_processing::extract_boundary_cycles(*sm, std::back_inserter(border_cycles));
	for (boost::graph_traits<CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>>::halfedge_descriptor h : border_cycles)
	{
		//if (max_hole_diam > 0 && max_num_hole_edges > 0 &&
		//	!is_small_hole(h, mesh, max_hole_diam, max_num_hole_edges))
		//	continue;
		std::vector<boost::graph_traits<CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>>::face_descriptor>  patch_facets;
		std::vector<boost::graph_traits<CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>>::vertex_descriptor> patch_vertices;
		//bool success = std::get<0>(CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(*sm,
		//	h,
		//	CGAL::parameters::face_output_iterator(std::back_inserter(patch_facets))
		//	.vertex_output_iterator(std::back_inserter(patch_vertices))));

		CGAL::Polygon_mesh_processing::triangulate_hole(*sm,
			h,
			CGAL::parameters::face_output_iterator(std::back_inserter(patch_facets))
			.vertex_output_iterator(std::back_inserter(patch_vertices)));
		//std::cout << "\tNumber of facets in constructed patch: " << patch_facets.size() << std::endl;
		//std::cout << "\tNumber of vertices in constructed patch: " << patch_vertices.size() << std::endl;
		//std::cout << "\tIs fairing successful: " << success << std::endl;
		++nb_holes;
	}
	std::cout << nb_holes << " holes have been filled" << std::endl;
	std::cout << std::endl;

	return nb_holes;
}

void CleanupSuite::TestInhouseHoleRemoval(open3d::geometry::TriangleMesh* m)
{
	mem.GetMeshOpenBorders(m);
}

void CleanupSuite::TestSVG()
{
	open3d::io::ReadTriangleMeshOptions params;

	std::string path_to_mesh = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_VoxeledMeshes/Triangulated_FILLED_000000.obj";
	//std::string path_to_mesh = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_TriangulatedMeshes/Triangulated_000000.obj";
	//std::string path_to_mesh = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/TriangleDefaultCube.obj";
	std::string save_name = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_VoxeledMeshes/Triangulated_VOXELED_000000.obj";

	open3d::geometry::TriangleMesh mesh;

	open3d::io::ReadTriangleMeshFromOBJ(path_to_mesh, mesh, params);

	//auto new_mesh = open3d::t::geometry::TriangleMesh::FromLegacyTriangleMesh(mesh);
	
	//new_mesh.

	int grid_base = 100;
	Eigen::Vector3i grid_dims = Eigen::Vector3i(grid_base + 1, 2 * grid_base + 1, grid_base + 1);

	svg.SetGridDimensions(grid_dims);
	svg.SetVoxelSize(1.249 / grid_base);

	svg.CastMesh(mesh);

	auto new_mesh = svg.GetDenseVoxelGrid()->ExtractMesh();

	DrawObject(*new_mesh);

	open3d::io::WriteTriangleMeshToOBJ(save_name, *new_mesh, true, false, true, false, false, false);
}

void CleanupSuite::TestMeshCast()
{
	open3d::io::ReadTriangleMeshOptions params;

	std::string path_to_mesh = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_TriangulatedMeshes/Triangulated_000000.obj";

	open3d::geometry::TriangleMesh mesh;

	open3d::io::ReadTriangleMeshFromOBJ(path_to_mesh, mesh, params);

	std::cout << "Stats: " << std::endl;

	std::cout << "\tVerts: " << mesh.vertices_.size() << std::endl;
	std::cout << "\Triangles: " << mesh.triangles_.size() << std::endl;

	auto grid = CreateFromTriangleMesh(mesh, 9.f / 512.f);

	auto voxels = grid->GetVoxels();

	std::cout << "Voxel grid bounds: " << grid->GetMaxBound() << ", " << grid->GetMinBound() << std::endl;

	//std::cout << voxels[0].

	//DrawObject(mesh);
}

void CleanupSuite::TestBitShaver()
{
	unsigned int test_quant = (1 << 20) + (1 << 10) + 1;

	std::cout << "Initial: " << test_quant << std::endl;

	int shift_amount = 31;

	test_quant = ((test_quant << shift_amount) >> shift_amount);

	std::cout << "Final: " << test_quant << std::endl;

	//int bool_count = 9;
	//
	//bool* test_bool_size = new bool[bool_count];
	//
	//std::cout << sizeof(test_bool_size) << std::endl;
	//
	//delete[] test_bool_size;
}

std::shared_ptr<open3d::geometry::VoxelGrid> CleanupSuite::CreateFromTriangleMesh(const open3d::geometry::TriangleMesh& input, double voxel_size)
{
	Eigen::Vector3d voxel_size3(voxel_size, voxel_size, voxel_size);
	Eigen::Vector3d min_bound = input.GetMinBound() - voxel_size3 * 0.5;
	Eigen::Vector3d max_bound = input.GetMaxBound() + voxel_size3 * 0.5;

	//std::cout << "Bounds: " << min_bound << ", " << max_bound << std::endl;

	auto output = std::make_shared<open3d::geometry::VoxelGrid>();
	if (voxel_size <= 0.0) {
		open3d::utility::LogError("[CreateFromTriangleMesh] voxel_size <= 0.");
	}

	if (voxel_size * std::numeric_limits<int>::max() <
		(max_bound - min_bound).maxCoeff()) {
		open3d::utility::LogError("[CreateFromTriangleMesh] voxel_size is too small.");
	}
	output->voxel_size_ = voxel_size;
	output->origin_ = min_bound;

	Eigen::Vector3d grid_size = max_bound - min_bound;
	int num_w = int(std::round(grid_size(0) / voxel_size));
	int num_h = int(std::round(grid_size(1) / voxel_size));
	int num_d = int(std::round(grid_size(2) / voxel_size));
	const Eigen::Vector3d box_half_size(voxel_size / 2, voxel_size / 2,
		voxel_size / 2);

	std::cout << "Dims: " << num_w << ", " << num_h << ", " << num_d << std::endl;

	for (int widx = 0; widx < num_w; widx++) {
		for (int hidx = 0; hidx < num_h; hidx++) {
			for (int didx = 0; didx < num_d; didx++) {

				const Eigen::Vector3d box_center =
					min_bound +
					Eigen::Vector3d(widx, hidx, didx) * voxel_size;
				for (const Eigen::Vector3i& tria : input.triangles_) {
					const Eigen::Vector3d& v0 = input.vertices_[tria(0)];
					const Eigen::Vector3d& v1 = input.vertices_[tria(1)];
					const Eigen::Vector3d& v2 = input.vertices_[tria(2)];
					if (open3d::geometry::IntersectionTest::TriangleAABB(
						box_center, box_half_size, v0, v1, v2)) {
						Eigen::Vector3i grid_index(widx, hidx, didx);
						output->AddVoxel(open3d::geometry::Voxel(grid_index));

						std::cout << "\tADDED AT: " << widx << ", " << hidx << ", " << didx << std::endl;

						break;
					}
				}
			}
		}
	}

	return output;
}

void CleanupSuite::run(int argc, char** argv)
{
	TestHoleRemoval();
	//TestSVG();
	//TestBitShaver();
	//TestMeshCast();
	return;

	if (cm.IsLoaded())
	{
		cm.Unload();
	}

	cm.LoadTypeLivescan();

	//int loaded_directories = 0;

	int frame = 200;

	auto directories = GetDirectories(root_folder);

	std::vector<std::string> filename_pieces;

	vgd.voxel_size = 9.f / 512.f;
	//vgd.depth_max = 1.5f;

	bool print_individual = false;

	for (int i = 0; i < directories.size(); ++i)
	{
		std::cout << directories[i] << std::endl;

		std::string intrinsic_file = "";
		std::string extrinsic_file = "";

		auto files = GetFiles(directories[i]);

		for (int j = 0; j < files.size(); ++j)
		{
			SplitString(files[j], filename_pieces, "/\\", "");
			std::string main_name = filename_pieces.back();

			auto find_int = main_name.find(intrinsics_handle);
			auto find_ext = main_name.find(extrinsics_handle);

			if (find_int != std::string::npos)
			{
				intrinsic_file = files[j];
			}

			if (find_ext != std::string::npos)
			{
				extrinsic_file = files[j];
			}

			filename_pieces.clear();
		}

		if (intrinsic_file == "" || extrinsic_file == "")
		{
			//cm.AddCameraLivescan(directories[i], intrinsic_file, extrinsic_file, color_handle, depth_handle, matte_handle, 5.0);
			//++loaded_directories;

			std::cout << "ERROR! Bad intrinsic/extrinsics file!" << std::endl;

			return;
		}
		else
		{
			cm.AddCameraLivescan(directories[i], intrinsic_file, extrinsic_file, color_handle, depth_handle, matte_handle, 5.0);
			
			if (print_individual)
			{
				cm.AllCamerasSeekFrame(frame);

				auto grid = cm.GetVoxelGrid(&vgd);
				auto mesh = grid.ExtractSurfaceMesh(0.0).ToLegacyTriangleMesh();
				//auto mesh = cm.GetOldVoxelGrid(&vgd).

				std::cout << "Tri count: " << mesh.triangles_.size() << std::endl;

				//mesh.

				std::string filename = output_folder + "/TestMesh_" + std::to_string(frame) + "_" + std::to_string(i) + ".obj";

				std::cout << "Writing to: " << filename << std::endl;

				open3d::io::WriteTriangleMeshToOBJ(filename, mesh, true, false, true, false, false, false);

				cm.SetCameraEnabled(i, false);
			}
		}
	}

	bool use_open3d = false;


	cm.AllCamerasSeekFrame(frame);

	for (int i = 0; i < directories.size(); ++i)
	{
		cm.SetCameraEnabled(i, true);
	}

	if (use_open3d)
	{
		auto grid = cm.GetVoxelGrid(&vgd);
		auto mesh = grid.ExtractSurfaceMesh(0.0).ToLegacyTriangleMesh();

		std::string filename = output_folder + "/CompleteMeshOpen3D_" + std::to_string(frame) + ".obj";

		std::cout << "Writing to: " << filename << std::endl;

		open3d::io::WriteTriangleMeshToOBJ(filename, mesh, true, false, true, false, false, false);
	}
	else
	{
		//int inc = 128;
		//mvp.points_x = inc + 1;
		//mvp.points_y = 2 * inc + 1;
		//mvp.points_z = inc + 1;
		//mvp.voxel_size = 2.05 / inc;

		//auto grid = cm.GetNewVoxelGrid(mvp);
		//auto open3d_grid = cm.GetVoxelGrid(&vgd);

		//auto mesh = grid->ExtractMesh();

		//grid->LoadFromOpen3D(open3d_grid);



		//std::string filename = output_folder + "/CompleteMeshSelf_" + std::to_string(frame) + ".obj";

		//DrawObject(*mesh);

		//std::cout << "Writing to: " << filename << std::endl;

		//open3d::io::WriteTriangleMeshToOBJ(filename, *mesh, true, false, true, false, false, false);
	}
}
