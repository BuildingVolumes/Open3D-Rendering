#pragma once

#include "TestSuite.h"
#include <open3d/Open3D.h>

class PointCloudSuite : public TestSuite
{
	void run(int argc, char** argv);

	template<class T>
	void DrawObject(T& object_to_draw)
	{
		std::vector<std::shared_ptr<const open3d::geometry::Geometry>> to_draw;

		auto object_ptr = std::make_shared<T>(
			object_to_draw);

		to_draw.push_back(object_ptr);

		open3d::visualization::DrawGeometries(to_draw);
	}
};