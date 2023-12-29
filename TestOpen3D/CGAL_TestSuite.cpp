#include "CGAL_TestSuite.h"

//#include <draco/compression/encode.h>
//#include <draco/io/mesh_io.h>
//#include <draco/io/obj_decoder.h>

namespace PMP = CGAL::Polygon_mesh_processing;
using K = CGAL::Exact_predicates_inexact_constructions_kernel;

typedef CGAL::Simple_cartesian<double>              Kernel;
typedef Kernel::Point_3                             Point_3;
typedef CGAL::Surface_mesh<Point_3>                 Surface_mesh;
typedef std::vector<std::size_t>                    Polygon_3;
//typedef CGAL::Polyhedron_3<Kernel>                   Polyhedron;

namespace SMS = CGAL::Surface_mesh_simplification;

void CGAL_TestSuite::SaveOneMesh(int frame_number)
{
    MeshingVoxelParams mvp;

    VolumeSequence vs;

    int voxel_count = 64;

    mvp.center = Eigen::Vector3d(0, 0, 0);
    mvp.points_x = voxel_count + 1;
    mvp.points_y = 2 * voxel_count + 1;
    mvp.points_z = voxel_count + 1;
    mvp.voxel_size = 1.05 / (double)voxel_count;

    vs.LoadImageSequences("_TEST_DATA/july15-spinninghogue_0", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", ".matte");
    vs.SetVoxelGridParams(mvp);

    vs.SaveFrameAsMesh("TestMesh_12_13_2023.obj", frame_number);
}

void CGAL_TestSuite::TestIfFileReadingWorks()
{
    std::vector<Kernel::Point_3> points_ref;
    std::vector<Polygon_3> faces_ref;

    const std::string the_date = "12_21_2023";
    const std::string date_of_last_mesh = "12_13_2023";

    const std::string filename = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\TestMesh_" + date_of_last_mesh + ".obj";

    const std::string decimated_filename = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\TestMeshREDUCED_" + the_date + ".obj";

    const std::string subdivided_filename = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\TestMeshSUBDIV_" + the_date + ".obj";

    //auto d_mesh = draco::ReadMeshFromFile(filename);
    //
    //if (!d_mesh.ok())
    //{
    //    std::cout << "Error - mesh not ok." << std::endl;
    //}
    //else
    //{
    //    std::cout << "SUCCESS!" << std::endl;
    //}
    //
    //return;

    std::ifstream in;

    in.open(filename);

    if (!in.is_open())
    {
        std::cerr << "File not open: " << filename << std::endl;
        return;
    }

    if (!in || !CGAL::IO::read_OBJ(in, points_ref, faces_ref))
    {
        std::cerr << "Failed to read input mesh: " << filename << std::endl;
        in.close();
        return;
    }

    //std::cout << points_ref.size() << std::endl;
    //
    //for (int i = 0; i < 10; ++i)
    //{
    //    if (i >= points_ref.size()) break;
    //
    //    std::cout << points_ref[i].x() << ", " << points_ref[i].y() << ", " << points_ref[i].z() << std::endl;
    //}

    Surface_mesh sm;

    //PMP::orient_polygon_soup(points_ref, faces_ref); // optional if your mesh is not correctly oriented
    PMP::polygon_soup_to_polygon_mesh(points_ref, faces_ref, sm);

    std::cout << "Num faces: " << sm.number_of_faces() << std::endl;
    std::cout << "Num edges: " << sm.number_of_edges() << std::endl;

    double decim_amnt = 1.0 / 16.0;
    //SMS::Edge_count_ratio_stop_predicate<Surface_mesh> stop(decim_amnt);
    SMS::Face_count_ratio_stop_predicate<Surface_mesh> stop(decim_amnt, sm);
    int edges_removed = SMS::edge_collapse(sm, stop);


    if (CGAL::IO::write_OBJ(decimated_filename, sm))
    {
        std::cout << "SUCCESS!" << std::endl;
    }
    else
    {
        std::cout << "ERROR! Mesh did not write!" << std::endl;
    }

    std::cout << "Num faces after " << decim_amnt << " decimation: " << sm.number_of_faces() << std::endl;
    std::cout << "Num edges after " << decim_amnt << " decimation: " << sm.number_of_edges() << std::endl;

    int iters = 2;
    //CGAL::Subdivision_method_3::CatmullClark_subdivision(sm, CGAL::parameters::number_of_iterations(iters));
    CGAL::Subdivision_method_3::Loop_subdivision(sm, CGAL::parameters::number_of_iterations(iters));

    std::cout << "Num faces after " << iters << " iterations: " << sm.number_of_faces() << std::endl;
    std::cout << "Num edges after " << iters << " iterations: " << sm.number_of_edges() << std::endl;

    //sm.

    if (CGAL::IO::write_OBJ(subdivided_filename, sm))
    {
        std::cout << "SUCCESS!" << std::endl;
    }
    else
    {
        std::cout << "ERROR! Mesh did not write!" << std::endl;
    }

    //sm.

    //CGAL::IO::write_OBJ(,)

    in.close();
}

void CGAL_TestSuite::TestMarshalling()
{
    MeshingVoxelParams mvp;

    int voxel_count = 64;

    mvp.center = Eigen::Vector3d(0, 0, 0);
    mvp.points_x = voxel_count + 1;
    mvp.points_y = 2 * voxel_count + 1;
    mvp.points_z = voxel_count + 1;
    mvp.voxel_size = 1.05 / (double)voxel_count;

    //int trim = 25;

    //std::string save_name = "FOURIER_" + std::to_string(voxel_count) + "_TRIM_" + std::to_string(trim);

    std::cout << "Loading images..." << std::endl;
    //vs.LoadImageSequences("_TEST_DATA/_TestingNewExtrinsics", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", "Matte_");
    vs.LoadImageSequences("_TEST_DATA/july15-spinninghogue_0", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", ".matte");

    vs.SetVoxelGridParams(mvp);
    //vs.SetTrim(trim + 1, 2 * trim + 1, trim + 1);

    auto mesh = vs.GetVoxelGridAtFrame(200)->ExtractMesh();

    auto sm = dm.GetOpen3DMeshAsCGAL(&(*mesh));

    std::cout << "Verts: " << sm->number_of_vertices() << std::endl;
    std::cout << "Faces: " << sm->number_of_faces() << std::endl;

    std::cout << "CGAL: " << std::endl;

    for (int i = 0; i < 10; ++i)
    {
        auto vert = sm->point(Surface_mesh::Vertex_index(i));
        auto face = CGAL::halfedges_around_face(sm->halfedge(Surface_mesh::Face_index(i)), *sm);

        std::vector<Surface_mesh::Vertex_index> vertIndices;

        for (auto halfedgeIndex : face)
        {
            vertIndices.push_back(CGAL::target(halfedgeIndex, *sm));
        }

        std::cout << i << ": " << vert.x() << ", " << vert.y() << ", " << vert.z() << ": --- " << vertIndices[0].id() << ", " << vertIndices[1].id() << ", " << vertIndices[2].id() << std::endl;
    }

    auto vert = sm->point(Surface_mesh::Vertex_index(sm->number_of_vertices() - 1));
    auto face = CGAL::halfedges_around_face(sm->halfedge(Surface_mesh::Face_index(sm->number_of_faces() - 1)), *sm);

    std::vector<Surface_mesh::Vertex_index> vertIndices;

    for (auto halfedgeIndex : face)
    {
        vertIndices.push_back(CGAL::target(halfedgeIndex, *sm));
    }

    std::cout << (sm->number_of_vertices() - 1) << ": " << vert.x() << ", " << vert.y() << ", " << vert.z() << ": --- " << vertIndices[0].id() << ", " << vertIndices[1].id() << ", " << vertIndices[2].id() << std::endl;
    std::cout << "Open3D: " << std::endl;

    for (int i = 0; i < 10; ++i)
    {
        auto o3d_vert = mesh->vertices_[i];
        auto tri = mesh->triangles_[i];
        std::cout << i << ": " << o3d_vert.x() << ", " << o3d_vert.y() << ", " << o3d_vert.z() << ": --- " << tri.x() << ", " << tri.y() << ", " << tri.z() << std::endl;
    }

    auto o3d_vert = mesh->vertices_[mesh->vertices_.size() - 1];
    auto tri = mesh->triangles_[mesh->triangles_.size() - 1];
    std::cout << (mesh->vertices_.size() - 1) << ": " << o3d_vert.x() << ", " << o3d_vert.y() << ", " << o3d_vert.z() << ": --- " << tri.x() << ", " << tri.y() << ", " << tri.z() << std::endl;

    //std::cout << "Saving grid..." << std::endl;
    //vs.SaveVolumeStream(save_name, SaveFileFormat::FOURIER);
    //vs.SaveVolumeStreamFourier("TestingCompleteFourier.vgff");
    //std::cout << "Saved!" << std::endl;
}

void CGAL_TestSuite::ArbitraryTests()
{
    std::vector<Kernel::Point_3> points_ref;
    std::vector<Polygon_3> faces_ref;

    const std::string filename = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\TestMesh_12_13_2023.obj";

    std::ifstream in;

    in.open(filename);

    if (!in.is_open())
    {
        std::cerr << "File not open: " << filename << std::endl;
        return;
    }

    if (!in || !CGAL::IO::read_OBJ(in, points_ref, faces_ref))
    {
        std::cerr << "Failed to read input mesh: " << filename << std::endl;
        in.close();
        return;
    }

    in.close();

    std::cout << "Verts: " << points_ref.size() << std::endl;

    for (int i = 0; i < 10; ++i)
    {
        std::cout << i << ": " << points_ref[i].x() << ", " << points_ref[i].y() << points_ref[i].z() << std::endl;
    }
}

void CGAL_TestSuite::SaveFileArray()
{
    std::string root_folder = "";

    MeshingVoxelParams mvp;

    int voxel_count = 64;

    mvp.center = Eigen::Vector3d(0, 0, 0);
    mvp.points_x = voxel_count + 1;
    mvp.points_y = 2 * voxel_count + 1;
    mvp.points_z = voxel_count + 1;
    mvp.voxel_size = 1.05 / (double)voxel_count;

    LoadImageSequences("_TEST_DATA/july15-spinninghogue_0", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", ".matte");

    int framecount = cm.GetPlayableFrameCount();

    for (int i = 0; i < framecount; ++i)
    {
        //std::cout << "Saving Frame " << i << "..." << std::endl;
        //
        //std::string new_name = root_folder + "/" + mesh_filename_without_extension + "_" + GetNumberFixedLength(i, 8);// std::to_string(i);
        //
        //cm.AllCamerasSeekFrame(i);
        //
        //auto tri_mesh = cm.GetMeshUsingNewVoxelGrid(mvp, 0);
        //
        //std::cout << "verts: " << tri_mesh->HasVertices() << ", count: " << tri_mesh->vertices_.size() << std::endl;
        //std::cout << "tris: " << tri_mesh->HasTriangles() << ", count: " << tri_mesh->triangles_.size() << std::endl;
        //std::cout << "normals: " << tri_mesh->HasTriangleNormals() << ", count: " << tri_mesh->triangle_normals_.size() << std::endl;
        //
        //if (!open3d::io::WriteTriangleMeshToOBJ(new_name + ".obj", *tri_mesh, true, false, true, false, true, false))
        //{
        //    system("pause");
        //    //std::cout << open3d::core::
        //}
        //
        ////main_savefile.write(new_name.c_str(), new_name.size());
        //main_savefile << new_name << "\n";
    }
}

int CGAL_TestSuite::LoadImageSequences(std::string root_folder, std::string intrinsics_handle, std::string extrinsics_handle, std::string color_handle, std::string depth_handle, std::string matte_handle, float FPS)
{
    if (cm.IsLoaded())
    {
        cm.Unload();
    }

    cm.LoadTypeLivescan();

    int loaded_directories = 0;

    auto directories = GetDirectories(root_folder);

    std::vector<std::string> filename_pieces;

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

        if (intrinsic_file != "" && extrinsic_file != "")
        {
            cm.AddCameraLivescan(directories[i], intrinsic_file, extrinsic_file, color_handle, depth_handle, matte_handle, FPS);
            ++loaded_directories;
        }
    }

    return loaded_directories;
}

void CGAL_TestSuite::run(int argc, char** argv)
{
    //SaveOneMesh(200);

    TestIfFileReadingWorks();

    //ArbitraryTests();

    //TestMarshalling();

    //Surface_mesh surface_mesh;
    ////const std::string filename = (argc > 1) ? argv[1] : CGAL::data_file_path("meshes/cube-meshed.off");
    //const std::string filename = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\Hogue_09_14_2022.obj";
    ////const std::string filename = CGAL::data_file_path("meshes/cube-meshed.off");
    //std::ifstream is(filename);
    //
    //CGAL::IO::read_OBJ(filename, );
    //
    //if (!is || !(is >> surface_mesh))
    //{
    //    std::cerr << "Failed to read input mesh: " << filename << std::endl;
    //    return;
    //}
    //if (!CGAL::is_triangle_mesh(surface_mesh))
    //{
    //    std::cerr << "Input geometry is not triangulated." << std::endl;
    //    return;
    //}
    //
    //std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    //// In this example, the simplification stops when the number of undirected edges
    //// drops below 10% of the initial count
    ////double stop_ratio = (argc > 2) ? std::stod(argv[2]) : 0.1;
    //
    //double stop_ratio = 0.5;
    //
    //SMS::Edge_count_ratio_stop_predicate<Surface_mesh> stop(stop_ratio);
    //
    //int r = SMS::edge_collapse(surface_mesh, stop);
    //
    //std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    //
    //std::cout << "\nFinished!\n" << r << " edges removed.\n" << surface_mesh.number_of_edges() << " final edges.\n";
    //std::cout << "Time elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << "ms" << std::endl;
    //
    ////CGAL::IO::write_polygon_mesh((argc > 3) ? argv[3] : "out.off", surface_mesh, CGAL::parameters::stream_precision(17));
    //CGAL::IO::write_polygon_mesh("Hogue_REDUCED_09_14_2022.obj", surface_mesh, CGAL::parameters::stream_precision(17));
    ////CGAL::IO::write_polygon_mesh("Cube_REDUCED.obj", surface_mesh, CGAL::parameters::stream_precision(17));
    //
    //return;
}
