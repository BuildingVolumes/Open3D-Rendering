#pragma once

#include "TestSuite.h"
#include <fftw3.h>
#include <open3d/Open3D.h>

class TestFFT : public TestSuite
{
	const int real_num = 0;
	const int imag_num = 1;


	std::string the_date = "3_29_2023";


public:
	void print_signal(int num_count, fftwf_complex* signal);
	void print_signal(int num_count, fftw_complex* signal);

	void basic_test();

	void grid_extraction_test();

	void run(int argc, char** argv);

	void FourierTesting();

	void WriteOBJ(std::string filename, std::string filepath, open3d::geometry::TriangleMesh* mesh);

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