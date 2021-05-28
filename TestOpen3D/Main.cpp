#include "CameraManager.h"
#include "ErrorLogger.h"
#include "VoxelGridData.h"

#include "open3d/io/sensor/azure_kinect/K4aPlugin.h"
#include "open3d/Open3D.h"
#include "open3d/io/sensor/azure_kinect/MKVMetadata.h"
#include "open3d/geometry/RGBDImage.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <fstream>
#include <json/json.h>
#include <memory>
#include <turbojpeg.h>

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

        DrawMesh(*mesh_legacy);

        //ErrorLogger::EXECUTE("Test Error Logging", &cm, &CameraManager::MakeAnErrorOnPurpose, true);
    }
}

int main() {
    ErrorLogger::EXECUTE("Refactored Code Test", &MKV_Rendering::refactored_code_test);

    return 0;
}