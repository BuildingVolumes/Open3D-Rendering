#pragma once

#include "MeshingVoxelGrid.h"

#include <vector>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
//#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

class DataMarshalling
{
public:
	std::shared_ptr<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>> GetOpen3DMeshAsCGAL_SC_D_P3(open3d::geometry::TriangleMesh *mesh);

	std::shared_ptr<CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>> GetOpen3DMeshAsCGAL_EPICK_P3(open3d::geometry::TriangleMesh *mesh);
};