#pragma once

#include "TestSuite.h"

#include "open3d/visualization/utility/DrawGeometry.h"
#include "open3d/geometry/Geometry.h"
#include "VolumeSequence.h"

class VoxelEncodeDecodeSuite : public TestSuite
{
	VolumeSequence vs;

    template<class T>
    void DrawObject(T& object_to_draw)
    {
        std::vector<std::shared_ptr<const open3d::geometry::Geometry>> to_draw;

        auto object_ptr = std::make_shared<T>(
            object_to_draw);

        to_draw.push_back(object_ptr);

        open3d::visualization::DrawGeometries(to_draw);
    }

	void SingleEncodeDecode();
public:
	void run(int argc, char** argv);
};