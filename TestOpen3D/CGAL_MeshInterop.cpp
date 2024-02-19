#include "CGAL_MeshInterop.h"

void CGAL_MeshInterop::Compress(open3d::geometry::TriangleMesh* mesh, double simplification_ratio)
{
	auto CGAL_mesh = dm.GetOpen3DMeshAsCGAL_SC_D_P3(mesh);

	CGAL::Surface_mesh_simplification::Edge_count_ratio_stop_predicate<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>> stop(simplification_ratio);
	int edges_removed = CGAL::Surface_mesh_simplification::edge_collapse(*CGAL_mesh, stop);

	
}
