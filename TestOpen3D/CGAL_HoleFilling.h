#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <iterator>
#include <string>
#include <tuple>
#include <vector>

//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
//typedef Kernel::Point_3                                     Point;
//typedef CGAL::Surface_mesh<Point>                           Mesh;
//typedef boost::graph_traits<Mesh>::vertex_descriptor        vertex_descriptor;
//typedef boost::graph_traits<Mesh>::halfedge_descriptor      halfedge_descriptor;
//typedef boost::graph_traits<Mesh>::face_descriptor          face_descriptor;
//namespace PMP = CGAL::Polygon_mesh_processing;

class CGAL_HoleFilling
{
public:

    bool is_small_hole(boost::graph_traits<CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>>::halfedge_descriptor h, CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>& mesh,
        double max_hole_diam, int max_num_hole_edges)
    {
        int num_hole_edges = 0;
        CGAL::Bbox_3 hole_bbox;
        for (boost::graph_traits<CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>>::halfedge_descriptor hc : CGAL::halfedges_around_face(h, mesh))
        {
            const CGAL::Exact_predicates_inexact_constructions_kernel::Point_3& p = mesh.point(target(hc, mesh));
            hole_bbox += p.bbox();
            ++num_hole_edges;
            // Exit early, to avoid unnecessary traversal of large holes
            if (num_hole_edges > max_num_hole_edges) return false;
            if (hole_bbox.xmax() - hole_bbox.xmin() > max_hole_diam) return false;
            if (hole_bbox.ymax() - hole_bbox.ymin() > max_hole_diam) return false;
            if (hole_bbox.zmax() - hole_bbox.zmin() > max_hole_diam) return false;
        }
        return true;
    }


    void PseudoMain()
    {
        const std::string filename = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_TriangulatedMeshes/Triangulated_000000.obj"; // (argc > 1) ? argv[1] : CGAL::data_file_path("meshes/mech-holes-shark.off");
        CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3> mesh;
        if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(filename, mesh))
        {
            std::cerr << "Invalid input." << std::endl;
            //return 1;
            return;
        }
        // Both of these must be positive in order to be considered
        double max_hole_diam = -1.0; // (argc > 2) ? boost::lexical_cast<double>(argv[2]) : -1.0;
        int max_num_hole_edges = -1; // (argc > 3) ? boost::lexical_cast<int>(argv[3]) : -1;
        unsigned int nb_holes = 0;
        std::vector<boost::graph_traits<CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>>::halfedge_descriptor> border_cycles;
        // collect one halfedge per boundary cycle
        CGAL::Polygon_mesh_processing::extract_boundary_cycles(mesh, std::back_inserter(border_cycles));
        for (boost::graph_traits<CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>>::halfedge_descriptor h : border_cycles)
        {
            if (max_hole_diam > 0 && max_num_hole_edges > 0 &&
                !is_small_hole(h, mesh, max_hole_diam, max_num_hole_edges))
                continue;
            std::vector<boost::graph_traits<CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>>::face_descriptor>  patch_facets;
            std::vector<boost::graph_traits<CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>>::vertex_descriptor> patch_vertices;
            bool success = std::get<0>(CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(mesh,
                h,
                CGAL::parameters::face_output_iterator(std::back_inserter(patch_facets))
                .vertex_output_iterator(std::back_inserter(patch_vertices))));
            std::cout << "* Number of facets in constructed patch: " << patch_facets.size() << std::endl;
            std::cout << "  Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
            std::cout << "  Is fairing successful: " << success << std::endl;
            ++nb_holes;
        }
        std::cout << std::endl;
        std::cout << nb_holes << " holes have been filled" << std::endl;
        CGAL::IO::write_polygon_mesh("filled_SM.off", mesh, CGAL::parameters::stream_precision(17));
        std::cout << "Mesh written to: filled_SM.off" << std::endl;
        //return 0;
        return;
    }
};