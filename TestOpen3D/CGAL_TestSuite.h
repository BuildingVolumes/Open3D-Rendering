#pragma once

#include "TestSuite.h"

#include "VolumeSequence.h"
#include "DataMarshalling.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
//#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Face_count_ratio_stop_predicate.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
//#include <CGAL/IO/OBJ.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include "AdditionalUtilities.h"

class CGAL_TestSuite : public TestSuite
{
	VolumeSequence vs;

	DataMarshalling dm;

	MKV_Rendering::CameraManager cm;

	void SaveOneMesh(int frame_number);

	void TestIfFileReadingWorks();

	void TestMarshalling();

	void ArbitraryTests();

	void SaveFileArray();

	int LoadImageSequences(std::string root_folder, std::string intrinsics_handle, std::string extrinsics_handle, std::string color_handle, std::string depth_handle, std::string matte_handle = "", float FPS = 5.0);

public:
	void run(int argc, char** argv);

	//void bad_func();
};