#include "DataMarshalling.h"

std::shared_ptr<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>> DataMarshalling::GetOpen3DMeshAsCGAL(open3d::geometry::TriangleMesh* mesh)
{
	auto to_return = std::make_shared<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>();

	std::vector<CGAL::Simple_cartesian<double>::Point_3> points;
	points.resize(mesh->vertices_.size());

	memcpy(points.data(), mesh->vertices_.data(), points.size() * sizeof(double) * 3);

	CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, mesh->triangles_, *to_return);

	return to_return;

	//memcpy(,);
}
