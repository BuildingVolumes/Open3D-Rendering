#pragma once

#include "open3d/Open3D.h"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <vector>

struct edgecost_comparer
{
    bool operator ()(const std::pair<double, std::pair<int, int>>& info, const double val) const
    {
        return info.first < val;
    }
};

class MeshDecimator
{
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;

    std::unordered_set<int> vertices_to_cull;
    std::unordered_set<int> triangles_to_cull;

    std::unordered_map<size_t, Eigen::Vector4d> optimal_vertices;

    //std::vector<std::pair<double, std::pair<int, int>>> edge_costs;

    std::pair<double, std::pair<int, int>> lowest_edge;

    std::vector<std::unordered_set<int>> vertex_links;
    std::vector<std::unordered_set<int>> vertex_triangles;

    std::vector<Eigen::Matrix4d> vertex_Qs;

    int initial_tris = 0;
    int initial_verts = 0;
    int total_tris = 0;

    Eigen::Matrix4d CalculateVertexError(int vertex_index);

    Eigen::Matrix4d CalculateEdgeCost(std::pair<int, int> edge);

    void CullEdge();

    void GenerateEdgeCosts();

    std::pair<int, int> GetPair(int v1, int v2);

    void GetLowestEdgeCost(std::pair<double, std::pair<int, int>> elem);

    inline size_t key(std::pair<int, int> i) { return (size_t)i.first << 32 | (unsigned int)i.second; }

    inline std::pair<int, int> unkey(size_t i) { return std::make_pair((i >> 32), (unsigned int) i); }
public:
    bool DecimateMesh(std::shared_ptr<open3d::geometry::TriangleMesh> to_decimate, int target_faces);

    void Clear();
};