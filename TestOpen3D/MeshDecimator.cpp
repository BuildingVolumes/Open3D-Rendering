#include "MeshDecimator.h"

#include <float.h>

Eigen::Matrix4d MeshDecimator::CalculateVertexError(int vertex_index)
{
    Eigen::Matrix4d total_q = Eigen::Matrix4d::Zero();
    
    auto vert = mesh->vertices_[vertex_index];
    auto vert_4d = Eigen::Vector4d(vert.x(), vert.y(), vert.z(), 1);
    
    for (const auto& elem : vertex_triangles[vertex_index])
    {
        auto tri = mesh->triangles_[elem];
    
        const auto& v0 = mesh->vertices_[tri.x()];
        const auto& v1 = mesh->vertices_[tri.y()];
        const auto& v2 = mesh->vertices_[tri.z()];
    
        auto normal = ((v0 - v2).cross(v0 - v1)).normalized();
        auto d_value = -normal.dot(v0);
        auto plane = Eigen::Vector4d(normal.x(), normal.y(), normal.z(), d_value);
    
        Eigen::Matrix4d q_mat;
    
        q_mat(0, 0) = plane.x() * plane.x();
        q_mat(0, 1) = plane.x() * plane.y();
        q_mat(0, 2) = plane.x() * plane.z();
        q_mat(0, 3) = plane.x() * plane.w();
        q_mat(1, 1) = plane.y() * plane.y();
        q_mat(1, 2) = plane.y() * plane.z();
        q_mat(1, 3) = plane.y() * plane.w();
        q_mat(2, 2) = plane.z() * plane.z();
        q_mat(2, 3) = plane.z() * plane.w();
        q_mat(3, 3) = plane.w() * plane.w();
    
        q_mat(1, 0) = q_mat(0, 1);
        q_mat(2, 0) = q_mat(0, 2);
        q_mat(3, 0) = q_mat(0, 3);
        q_mat(2, 1) = q_mat(1, 2);
        q_mat(3, 1) = q_mat(1, 3);
        q_mat(3, 2) = q_mat(2, 3);
    
        total_q += q_mat;
    }
    
    return total_q;

    return Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d MeshDecimator::CalculateEdgeCost(std::pair<int, int> edge)
{
    Eigen::Matrix4d new_Q = vertex_Qs[edge.first] + vertex_Qs[edge.second];
    const auto& v0 = mesh->vertices_[edge.first];
    const auto& v1 = mesh->vertices_[edge.second];
    
    Eigen::Matrix4d q_inverse = new_Q;
    q_inverse.row(3) = Eigen::Vector4d(0, 0, 0, 1);
    if (q_inverse.determinant() > 0)
    {
        q_inverse = q_inverse.inverse();
        optimal_vertices[key(edge)] = q_inverse.col(3);
    }
    else
    {
        Eigen::Vector3d halfway = (v0 + v1) * 0.5;
        optimal_vertices[key(edge)] = Eigen::Vector4d(halfway.x(), halfway.y(), halfway.z(), 1);
    }
    
    return new_Q;

    //return Eigen::Matrix4d::Identity();
}

void MeshDecimator::CullEdge()
{
    auto target = lowest_edge; //edge_costs[0];
    
    std::cout << lowest_edge.second.first << ", " << lowest_edge.second.second << std::endl;

    //if (optimal_vertices.find(key(target.second)) == optimal_vertices.end())
    //{
    //    edge_costs.erase(edge_costs.begin());
    //    return;
    //}
    
    //std::unordered_set<std::pair<int, int>> to_update;
    
    Eigen::Vector4d opt = optimal_vertices[key(target.second)];
    mesh->vertices_[target.second.first] = Eigen::Vector3d(opt.x(), opt.y(), opt.z());
    
    vertices_to_cull.insert(target.second.second);
    
    //edge_costs.erase(edge_costs.begin());
    
    std::unordered_set<int> tris;
    tris.insert(vertex_triangles[target.second.second].begin(), vertex_triangles[target.second.second].end());

    std::cout << tris.size() << std::endl;
    
    for (auto elem : tris)
    {
        if (vertex_triangles[target.second.first].find(elem) != vertex_triangles[target.second.first].end())
        {
            vertex_triangles[mesh->triangles_[elem].x()].erase(elem);
            vertex_triangles[mesh->triangles_[elem].y()].erase(elem);
            vertex_triangles[mesh->triangles_[elem].z()].erase(elem);
    
            triangles_to_cull.insert(elem);
            --total_tris;
        }
        else
        {
            vertex_triangles[target.second.first].insert(elem);
    
            if (mesh->triangles_[elem].x() == target.second.second)
            {
                mesh->triangles_[elem].x() = target.second.first;
            }
            else if (mesh->triangles_[elem].y() == target.second.second)
            {
                mesh->triangles_[elem].y() = target.second.first;
            }
            else if (mesh->triangles_[elem].z() == target.second.second)
            {
                mesh->triangles_[elem].z() = target.second.first;
            }
        }
    }
    
    vertex_links[target.second.first].erase(target.second.second);
    optimal_vertices.erase(key(GetPair(target.second.first, target.second.second)));
    
    for (auto elem : vertex_links[target.second.second])
    {
        if (elem == target.second.first)
        {
            continue;
        }
    
        vertex_links[elem].erase(target.second.second);
        optimal_vertices.erase(key(GetPair(elem, target.second.second)));
    
        vertex_links[elem].insert(target.second.first);
        optimal_vertices[key(GetPair(elem, target.second.first))] = Eigen::Vector4d(0, 0, 0, 1);
    
        vertex_links[target.second.first].insert(elem);
    }
    
    vertex_Qs[target.second.first] = CalculateVertexError(target.second.first);
    for (auto elem : vertex_links[target.second.first])
    {
        vertex_Qs[elem] = CalculateVertexError(elem);
    }
    
    //for (auto elem : vertex_links[target.second.first])
    //{
    //    auto marker = std::make_pair((double)0, GetPair(elem, target.second.first));
    //
    //    Eigen::Matrix4d new_Q = CalculateEdgeCost(elem);
    //
    //    marker.first = optimal_vertices[elem.first].dot(new_Q * optimal_vertices[elem.first]);
    //
    //    AddElementToEdgeCosts(marker);
    //}
    
    GenerateEdgeCosts();
}

void MeshDecimator::GenerateEdgeCosts()
{
    //edge_costs.clear();
    lowest_edge = std::make_pair(DBL_MAX, std::make_pair(0, 0));
    
    int count = 0;

    for (auto elem : optimal_vertices)
    {
        //if (count % 1000 == 0)
        //{
        //    std::cout << "Count: " << std::to_string(count) << "/" << optimal_vertices.size() << std::endl;
        //}

        auto marker = std::make_pair((double)0, unkey(elem.first));
    
        Eigen::Matrix4d new_Q = CalculateEdgeCost(unkey(elem.first));
    
        marker.first = optimal_vertices[elem.first].dot(new_Q * optimal_vertices[elem.first]);
    
        GetLowestEdgeCost(marker);
    }
}

std::pair<int, int> MeshDecimator::GetPair(int v1, int v2)
{
    if (v1 > v2)
    {
        return std::make_pair(v2, v1);
    }
    else
    {
        return std::make_pair(v2, v1);
    }

    return std::make_pair(0, 0);
}

void MeshDecimator::GetLowestEdgeCost(std::pair<double, std::pair<int, int>> elem)
{
    //auto bound = std::lower_bound(edge_costs.begin(), edge_costs.end(), elem.first, edgecost_comparer());
    //
    //edge_costs.insert(bound, elem);

    if (elem.first < lowest_edge.first)
    {
        lowest_edge = elem;
    }
}

bool MeshDecimator::DecimateMesh(std::shared_ptr<open3d::geometry::TriangleMesh> to_decimate, int target_faces)
{
    std::cout << "Commencing Decimation..." << std::endl;

    mesh = to_decimate;
    
    initial_tris = mesh->triangles_.size();
    total_tris = initial_tris;
    initial_verts = mesh->vertices_.size();
    
    vertex_links.resize(mesh->vertices_.size());
    vertex_triangles.resize(mesh->vertices_.size());
    vertex_Qs.resize(mesh->vertices_.size());
    
    for (int i = 0; i < mesh->triangles_.size(); ++i)
    {
        auto v0 = mesh->triangles_[i].x();
        auto v1 = mesh->triangles_[i].y();
        auto v2 = mesh->triangles_[i].z();
    
        vertex_links[v0].insert(v1);
        vertex_links[v0].insert(v2);
        vertex_triangles[v0].insert(i);
    
        vertex_links[v1].insert(v0);
        vertex_links[v1].insert(v2);
        vertex_triangles[v1].insert(i);
    
        vertex_links[v2].insert(v0);
        vertex_links[v2].insert(v1);
        vertex_triangles[v2].insert(i);
    
        optimal_vertices[key(GetPair(v0, v1))] = Eigen::Vector4d(0, 0, 0, 1);
        optimal_vertices[key(GetPair(v0, v2))] = Eigen::Vector4d(0, 0, 0, 1);
        optimal_vertices[key(GetPair(v1, v2))] = Eigen::Vector4d(0, 0, 0, 1);
    }
    
    std::cout << "Calculating Q matrices..." << std::endl;

    for (int i = 0; i < mesh->vertices_.size(); ++i)
    {
        //if (i % 1 == 0)
        //{
        //    std::cout << "At: " << std::to_string(i) << std::endl;
        //}

        vertex_Qs[i] = CalculateVertexError(i);
    }
    
    std::cout << "Calculating edge costs..." << std::endl;

    GenerateEdgeCosts();

    std::cout << "Culling edges..." << std::endl;
    
    while (total_tris > target_faces)
    {
        if (total_tris % 1 == 0)
        {
            std::cout << "Current Faces: " << std::to_string(total_tris) << std::endl;
        }

        CullEdge();
    }
    
    int front_iter = 0;
    int back_iter = mesh->vertices_.size();
    
    while (front_iter < back_iter)
    {
        if (vertices_to_cull.find(front_iter) != vertices_to_cull.end())
        {
            while (vertices_to_cull.find(back_iter) != vertices_to_cull.end())
            {
                mesh->vertices_.pop_back();
                --back_iter;
            }
    
            if (front_iter >= back_iter)
            {
                break;
            }
    
            mesh->vertices_[front_iter] = mesh->vertices_[back_iter];
    
            for (auto elem : vertex_triangles[back_iter])
            {
                if (mesh->triangles_[elem].x() == back_iter)
                {
                    mesh->triangles_[elem].x() = front_iter;
                }
                else if (mesh->triangles_[elem].y() == back_iter)
                {
                    mesh->triangles_[elem].y() = front_iter;
                }
                else if (mesh->triangles_[elem].z() == back_iter)
                {
                    mesh->triangles_[elem].z() = front_iter;
                }
            }
    
            mesh->vertices_.pop_back();
            --back_iter;
        }
    
        ++front_iter;
    }
    
    front_iter = 0;
    back_iter = mesh->triangles_.size();
    
    while (front_iter < back_iter)
    {
        if (triangles_to_cull.find(front_iter) != triangles_to_cull.end())
        {
            while (triangles_to_cull.find(back_iter) != triangles_to_cull.end())
            {
                mesh->triangles_.pop_back();
                --back_iter;
            }
    
            if (front_iter >= back_iter)
            {
                break;
            }
    
            mesh->triangles_[front_iter] = mesh->triangles_[back_iter];
    
            mesh->triangles_.pop_back();
            --back_iter;
        }
    
        ++front_iter;
    }
    
    std::cout << "Final culling stats: " << std::endl;
    std::cout << "Tris: " << total_tris << "/" << initial_tris << std::endl;
    std::cout << "Verts: " << mesh->vertices_.size() << "/" << initial_verts << std::endl;

    Clear();

    return true;
}

void MeshDecimator::Clear()
{
    vertices_to_cull.clear();
    triangles_to_cull.clear();
    
    optimal_vertices.clear();
    
    //edge_costs.clear();
    lowest_edge = std::make_pair(DBL_MAX, std::make_pair(0, 0));

    vertex_links.clear();
    vertex_triangles.clear();

    vertex_Qs.clear();

    initial_tris = 0;
    initial_verts = 0;
    total_tris = 0;
}
