#include "CameraManager.h"
#include "ErrorLogger.h"
#include "VoxelGridData.h"

#include "open3d/io/sensor/azure_kinect/K4aPlugin.h"
#include "open3d/Open3D.h"
#include "open3d/io/sensor/azure_kinect/MKVMetadata.h"
#include "open3d/geometry/RGBDImage.h"


#include "AlembicWriter.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <fstream>
#include <json/json.h>
#include <memory>
#include <turbojpeg.h>


#include <chrono>

using namespace open3d;
using namespace open3d::core;
using namespace open3d::io;
using namespace Eigen;

namespace MKV_Rendering {
    void DrawMesh(geometry::TriangleMesh &object_to_draw)
    {
        std::vector<std::shared_ptr<const geometry::Geometry>> to_draw;

        auto object_ptr = std::make_shared<geometry::TriangleMesh>(
            object_to_draw);

        to_draw.push_back(object_ptr);
        visualization::DrawGeometries(to_draw);
    }

    //Not used currently
    void PrintHelp() {
        using namespace open3d;

        PrintOpen3DVersion();
        // clang-format off
        utility::LogInfo("Usage:");
        utility::LogInfo(">    <executable_name> [mkv_and_calibration_folder] [mesh_name]");
        utility::LogInfo("     Takes a folder containing several .mkv and .log files, and produces a mesh from the frames");
        utility::LogInfo("     [options]");
        utility::LogInfo("     --voxel_size [=0.0058 (m)]");
        utility::LogInfo("     --depth_scale [=1000.0]");
        utility::LogInfo("     --depth_max [=3.0]");
        utility::LogInfo("     --sdf_trunc [=0.04]");
        utility::LogInfo("     --device [CPU:0]");
        // clang-format on
        utility::LogInfo("");
    }

    int render_kinect(int argc, char** argv)
    {
        if (argc == 1 || utility::ProgramOptionExists(argc, argv, "--help") ||
            argc < 4) {
            PrintHelp();
            return 1;
        }

        using MaskCode = t::geometry::TSDFVoxelGrid::SurfaceMaskCode;

        return 0;
    }

    std::vector<Alembic::Abc::float32_t> double3ToAlembic(std::vector<Eigen::Vector3d> source) {
        std::vector<Alembic::Abc::float32_t> result;
        result.resize(source.size()*3); //source stores each element as  (x,y,z) while result stores it sequently

        #pragma omp parallel
        #pragma omp for
        for (int i = 0; i < source.size(); i++) {
            result[i * 3 + 0] = source[i].x();
            result[i * 3 + 1] = source[i].y();
            result[i * 3 + 2] = source[i].z();
        }

        return result;
    }
    void saveMesh(geometry::TriangleMesh& object_to_draw, AlembicWriter& alembicWriter) {
        AlembicMeshData meshData;

        auto start = std::chrono::steady_clock::now();
        meshData.vertices = double3ToAlembic(object_to_draw.vertices_);
        meshData.numVerts = meshData.vertices.size() /3;
        
        meshData.numIndicies = object_to_draw.triangles_.size() * 3;
        meshData.indicies.resize(meshData.numIndicies);
        
        #pragma omp parallel
        #pragma omp for
        for (int i = 0; i < meshData.numIndicies/3; i++) {
            meshData.indicies[i * 3 + 0] = object_to_draw.triangles_[i].x();
            meshData.indicies[i * 3 + 1] = object_to_draw.triangles_[i].y();
            meshData.indicies[i * 3 + 2] = object_to_draw.triangles_[i].z();
        }

        meshData.numCounts = meshData.numIndicies / 3;
        meshData.counts.resize(meshData.numCounts);

        #pragma omp parallel
        #pragma omp for
        for (int i = 0; i < meshData.numCounts; i++) {
            meshData.counts[i] = 3;
        }

        meshData.normals = double3ToAlembic(object_to_draw.vertex_normals_);
        meshData.numNormals = meshData.normals.size() / 3;

        auto end = std::chrono::steady_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;
        std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
        alembicWriter.saveFrame(meshData);

    }


    void refactored_code_test()
    {
        std::string root_folder = "Kinect Test 1";

        CameraManager cm(root_folder);

        VoxelGridData vgd; //Edit values to toy with voxel grid settings

        uint64_t timestamp = 10900000; //Approximately 11 seconds in

        auto mesh = ErrorLogger::EXECUTE(
            "Generate Mesh", &cm, &CameraManager::GetMeshAtTimestamp, &vgd, timestamp
        );

        auto mesh_legacy = std::make_shared<geometry::TriangleMesh>(mesh.ToLegacyTriangleMesh());

        
        //DrawMesh(*mesh_legacy);

        float startTime = 1.0f / 3.0f;
        float deltaTime = 0.25;

        AlembicWriter alembicWriter("open3dMesh.abc", "Hogue", startTime, deltaTime);
        saveMesh(*mesh_legacy, alembicWriter);
        //ErrorLogger::EXECUTE("Test Error Logging", &cm, &CameraManager::MakeAnErrorOnPurpose, true);
    }

    
}

int main() {
    ErrorLogger::EXECUTE("Refactored Code Test", &MKV_Rendering::refactored_code_test);

    
    return 0;
}