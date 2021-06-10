#pragma once

#include "open3d/pipelines/color_map/NonRigidOptimizer.h"

#include <memory>
#include <vector>

#include "open3d/io/ImageIO.h"
#include "open3d/io/ImageWarpingFieldIO.h"
#include "open3d/io/PinholeCameraTrajectoryIO.h"
#include "open3d/io/TriangleMeshIO.h"
#include "open3d/pipelines/color_map/ColorMapUtils.h"
#include "open3d/pipelines/color_map/ImageWarpingField.h"
#include "open3d/utility/FileSystem.h"

namespace Eigen {

    typedef Eigen::Matrix<double, 14, 1> Vector14d;
    typedef Eigen::Matrix<int, 14, 1> Vector14i;

}

static std::tuple<float, float, float> Project3DPointAndGetUVDepth(
    const Eigen::Vector3d X,
    const open3d::camera::PinholeCameraParameters& camera_parameter) {
    std::pair<double, double> f = camera_parameter.intrinsic_.GetFocalLength();
    std::pair<double, double> p =
        camera_parameter.intrinsic_.GetPrincipalPoint();
    Eigen::Vector4d Vt =
        camera_parameter.extrinsic_ * Eigen::Vector4d(X(0), X(1), X(2), 1);
    float u = float((Vt(0) * f.first) / Vt(2) + p.first);
    float v = float((Vt(1) * f.second) / Vt(2) + p.second);
    float z = float(Vt(2));
    return std::make_tuple(u, v, z);
}

inline std::tuple<std::vector<open3d::geometry::Image>,
    std::vector<open3d::geometry::Image>,
    std::vector<open3d::geometry::Image>,
    std::vector<open3d::geometry::Image>,
    std::vector<open3d::geometry::Image>>
    CreateUtilImagesFromRGBD(const std::vector<open3d::geometry::RGBDImage>& images_rgbd) {
    std::vector<open3d::geometry::Image> images_gray;
    std::vector<open3d::geometry::Image> images_dx;
    std::vector<open3d::geometry::Image> images_dy;
    std::vector<open3d::geometry::Image> images_color;
    std::vector<open3d::geometry::Image> images_depth;
    for (size_t i = 0; i < images_rgbd.size(); i++) {
        auto gray_image = images_rgbd[i].color_.CreateFloatImage();
        auto gray_image_filtered =
            gray_image->Filter(open3d::geometry::Image::FilterType::Gaussian3);
        images_gray.push_back(*gray_image_filtered);
        images_dx.push_back(*gray_image_filtered->Filter(
            open3d::geometry::Image::FilterType::Sobel3Dx));
        images_dy.push_back(*gray_image_filtered->Filter(
            open3d::geometry::Image::FilterType::Sobel3Dy));
        auto color = std::make_shared<open3d::geometry::Image>(images_rgbd[i].color_);
        auto depth = std::make_shared<open3d::geometry::Image>(images_rgbd[i].depth_);
        images_color.push_back(*color);
        images_depth.push_back(*depth);
    }
    return std::make_tuple(images_gray, images_dx, images_dy, images_color,
        images_depth);
}


static std::vector<open3d::pipelines::color_map::ImageWarpingField> CreateWarpingFields(
    const std::vector<open3d::geometry::Image>& images,
    int number_of_vertical_anchors) {
    std::vector<open3d::pipelines::color_map::ImageWarpingField> fields;
    for (size_t i = 0; i < images.size(); i++) {
        int width = images[i].width_;
        int height = images[i].height_;
        fields.push_back(
            open3d::pipelines::color_map::ImageWarpingField(width, height, number_of_vertical_anchors));
    }
    return fields;
}

inline std::vector<open3d::geometry::Image> CreateDepthBoundaryMasks(
    const std::vector<open3d::geometry::Image>& images_depth,
    double depth_threshold_for_discontinuity_check,
    int half_dilation_kernel_size_for_discontinuity_map) {
    auto n_images = images_depth.size();
    std::vector<open3d::geometry::Image> masks;
    for (size_t i = 0; i < n_images; i++) {
        open3d::utility::LogDebug("[MakeDepthMasks] geometry::Image {:d}/{:d}", i,
            n_images);
        masks.push_back(*images_depth[i].CreateDepthBoundaryMask(
            depth_threshold_for_discontinuity_check,
            half_dilation_kernel_size_for_discontinuity_map));
    }
    return masks;
}

inline std::tuple<std::vector<std::vector<int>>, std::vector<std::vector<int>>>
CreateVertexAndImageVisibility(
    const open3d::geometry::TriangleMesh& mesh,
    const std::vector<open3d::geometry::Image>& images_depth,
    const std::vector<open3d::geometry::Image>& images_mask,
    const open3d::camera::PinholeCameraTrajectory& camera_trajectory,
    double maximum_allowable_depth,
    double depth_threshold_for_visibility_check) {
    size_t n_camera = camera_trajectory.parameters_.size();
    size_t n_vertex = mesh.vertices_.size();
    // visibility_image_to_vertex[c]: vertices visible by camera c.
    std::vector<std::vector<int>> visibility_image_to_vertex;
    visibility_image_to_vertex.resize(n_camera);
    // visibility_vertex_to_image[v]: cameras that can see vertex v.
    std::vector<std::vector<int>> visibility_vertex_to_image;
    visibility_vertex_to_image.resize(n_vertex);

#pragma omp parallel for schedule(static)
    for (int camera_id = 0; camera_id < int(n_camera); camera_id++) {
        for (int vertex_id = 0; vertex_id < int(n_vertex); vertex_id++) {
            Eigen::Vector3d X = mesh.vertices_[vertex_id];
            float u, v, d;
            std::tie(u, v, d) = Project3DPointAndGetUVDepth(
                X, camera_trajectory.parameters_[camera_id]);
            int u_d = int(round(u)), v_d = int(round(v));
            // Skip if vertex in image boundary.
            if (d < 0.0 ||
                !images_depth[camera_id].TestImageBoundary(u_d, v_d)) {
                continue;
            }

            float d_sensor = 0;

            //std::cout << "Doing " << u_d << ", " << v_d << std::endl;
            
            // Skip if vertex's depth is too large (e.g. background).
            d_sensor = images_depth[camera_id].data_[v_d * images_depth[camera_id].width_ + u_d];
            //    *images_depth[camera_id].PointerAt<float>(u_d, v_d);
            //if (d_sensor > maximum_allowable_depth) {
            //    continue;
            //}

            // Check depth boundary mask. If a vertex is located at the boundary
            // of an object, its color will be highly diverse from different
            // viewing angles.
            if (*images_mask[camera_id].PointerAt<uint8_t>(u_d, v_d) == 255) {
                continue;
            }
            // Check depth errors.
            if (std::fabs(d - d_sensor) >=
                depth_threshold_for_visibility_check) {
                continue;
            }
            visibility_image_to_vertex[camera_id].push_back(vertex_id);
#pragma omp critical
            { visibility_vertex_to_image[vertex_id].push_back(camera_id); }
        }
    }

    for (int camera_id = 0; camera_id < int(n_camera); camera_id++) {
        size_t n_visible_vertex = visibility_image_to_vertex[camera_id].size();
        open3d::utility::LogDebug(
            "[cam {:d}]: {:d}/{:d} ({:.5f}%) vertices are visible",
            camera_id, n_visible_vertex, n_vertex,
            double(n_visible_vertex) / n_vertex * 100);
    }

    return std::make_tuple(visibility_vertex_to_image,
        visibility_image_to_vertex);
}

template <typename VecInTypeDouble,
    typename VecInTypeInt,
    typename MatOutType,
    typename VecOutType>
    static std::tuple<MatOutType, VecOutType, double> ComputeJTJandJTrNonRigid(
        std::function<void(int, VecInTypeDouble&, double&, VecInTypeInt&)> f,
        int iteration_num,
        int nonrigidval,
        bool verbose /*=true*/) {
    MatOutType JTJ(6 + nonrigidval, 6 + nonrigidval);
    VecOutType JTr(6 + nonrigidval);
    double r2_sum = 0.0;
    JTJ.setZero();
    JTr.setZero();
#pragma omp parallel
    {
        MatOutType JTJ_private(6 + nonrigidval, 6 + nonrigidval);
        VecOutType JTr_private(6 + nonrigidval);
        double r2_sum_private = 0.0;
        JTJ_private.setZero();
        JTr_private.setZero();
        VecInTypeDouble J_r;
        VecInTypeInt pattern;
        double r;
#pragma omp for nowait
        for (int i = 0; i < iteration_num; i++) {
            f(i, J_r, r, pattern);
            for (auto x = 0; x < J_r.size(); x++) {
                for (auto y = 0; y < J_r.size(); y++) {
                    JTJ_private(pattern(x), pattern(y)) += J_r(x) * J_r(y);
                }
            }
            for (auto x = 0; x < J_r.size(); x++) {
                JTr_private(pattern(x)) += r * J_r(x);
            }
            r2_sum_private += r * r;
        }
#pragma omp critical
        {
            JTJ += JTJ_private;
            JTr += JTr_private;
            r2_sum += r2_sum_private;
        }
    }
    return std::make_tuple(std::move(JTJ), std::move(JTr), r2_sum);
}

static void ComputeJacobianAndResidualNonRigid(
    int row,
    Eigen::Vector14d& J_r,
    double& r,
    Eigen::Vector14i& pattern,
    const open3d::geometry::TriangleMesh& mesh,
    const std::vector<double>& proxy_intensity,
    const open3d::geometry::Image& images_gray,
    const open3d::geometry::Image& images_dx,
    const open3d::geometry::Image& images_dy,
    const open3d::pipelines::color_map::ImageWarpingField& warping_fields,
    const Eigen::Matrix4d& intrinsic,
    const Eigen::Matrix4d& extrinsic,
    const std::vector<int>& visibility_image_to_vertex,
    const int image_boundary_margin) {
    J_r.setZero();
    pattern.setZero();
    r = 0;
    int anchor_w = warping_fields.anchor_w_;
    double anchor_step = warping_fields.anchor_step_;
    int vid = visibility_image_to_vertex[row];
    Eigen::Vector3d V = mesh.vertices_[vid];
    Eigen::Vector4d G = extrinsic * Eigen::Vector4d(V(0), V(1), V(2), 1);
    Eigen::Vector4d L = intrinsic * G;
    double u = L(0) / L(2);
    double v = L(1) / L(2);
    if (!images_gray.TestImageBoundary(u, v, image_boundary_margin)) {
        return;
    }
    int ii = (int)(u / anchor_step);
    int jj = (int)(v / anchor_step);
    if (ii >= warping_fields.anchor_w_ - 1 ||
        jj >= warping_fields.anchor_h_ - 1) {
        return;
    }
    double p = (u - ii * anchor_step) / anchor_step;
    double q = (v - jj * anchor_step) / anchor_step;
    Eigen::Vector2d grids[4] = {
            warping_fields.QueryFlow(ii, jj),
            warping_fields.QueryFlow(ii, jj + 1),
            warping_fields.QueryFlow(ii + 1, jj),
            warping_fields.QueryFlow(ii + 1, jj + 1),
    };
    Eigen::Vector2d uuvv = (1 - p) * (1 - q) * grids[0] +
        (1 - p) * (q)*grids[1] + (p) * (1 - q) * grids[2] +
        (p) * (q)*grids[3];
    double uu = uuvv(0);
    double vv = uuvv(1);
    if (!images_gray.TestImageBoundary(uu, vv, image_boundary_margin)) {
        return;
    }
    bool valid;
    double gray, dIdfx, dIdfy;
    std::tie(valid, gray) = images_gray.FloatValueAt(uu, vv);
    std::tie(valid, dIdfx) = images_dx.FloatValueAt(uu, vv);
    std::tie(valid, dIdfy) = images_dy.FloatValueAt(uu, vv);
    Eigen::Vector2d dIdf(dIdfx, dIdfy);
    Eigen::Vector2d dfdx =
        ((grids[2] - grids[0]) * (1 - q) + (grids[3] - grids[1]) * q) /
        anchor_step;
    Eigen::Vector2d dfdy =
        ((grids[1] - grids[0]) * (1 - p) + (grids[3] - grids[2]) * p) /
        anchor_step;
    double dIdx = dIdf.dot(dfdx);
    double dIdy = dIdf.dot(dfdy);
    double invz = 1. / G(2);
    double v0 = dIdx * intrinsic(0, 0) * invz;
    double v1 = dIdy * intrinsic(1, 1) * invz;
    double v2 = -(v0 * G(0) + v1 * G(1)) * invz;
    J_r(0) = -G(2) * v1 + G(1) * v2;
    J_r(1) = G(2) * v0 - G(0) * v2;
    J_r(2) = -G(1) * v0 + G(0) * v1;
    J_r(3) = v0;
    J_r(4) = v1;
    J_r(5) = v2;
    J_r(6) = dIdf(0) * (1 - p) * (1 - q);
    J_r(7) = dIdf(1) * (1 - p) * (1 - q);
    J_r(8) = dIdf(0) * (1 - p) * (q);
    J_r(9) = dIdf(1) * (1 - p) * (q);
    J_r(10) = dIdf(0) * (p) * (1 - q);
    J_r(11) = dIdf(1) * (p) * (1 - q);
    J_r(12) = dIdf(0) * (p) * (q);
    J_r(13) = dIdf(1) * (p) * (q);
    pattern(0) = 0;
    pattern(1) = 1;
    pattern(2) = 2;
    pattern(3) = 3;
    pattern(4) = 4;
    pattern(5) = 5;
    pattern(6) = 6 + (ii + jj * anchor_w) * 2;
    pattern(7) = 6 + (ii + jj * anchor_w) * 2 + 1;
    pattern(8) = 6 + (ii + (jj + 1) * anchor_w) * 2;
    pattern(9) = 6 + (ii + (jj + 1) * anchor_w) * 2 + 1;
    pattern(10) = 6 + ((ii + 1) + jj * anchor_w) * 2;
    pattern(11) = 6 + ((ii + 1) + jj * anchor_w) * 2 + 1;
    pattern(12) = 6 + ((ii + 1) + (jj + 1) * anchor_w) * 2;
    pattern(13) = 6 + ((ii + 1) + (jj + 1) * anchor_w) * 2 + 1;
    r = (gray - proxy_intensity[vid]);
}

//NonRigid
inline open3d::geometry::TriangleMesh NRColorOptimization(open3d::geometry::TriangleMesh& mesh, std::vector<open3d::geometry::RGBDImage>& rgbd_image_vector,
    open3d::camera::PinholeCameraTrajectory& trajectory, open3d::pipelines::color_map::NonRigidOptimizerOption& option)
{
    // The following properties will change during optimization.
    open3d::geometry::TriangleMesh opt_mesh = mesh;
    open3d::camera::PinholeCameraTrajectory opt_camera_trajectory = trajectory;
    std::vector<open3d::pipelines::color_map::ImageWarpingField> warping_fields;

    // The following properties remain unchanged during optimization.
    std::vector<open3d::geometry::Image> images_gray;
    std::vector<open3d::geometry::Image> images_dx;
    std::vector<open3d::geometry::Image> images_dy;
    std::vector<open3d::geometry::Image> images_color;
    std::vector<open3d::geometry::Image> images_depth;
    std::vector<open3d::geometry::Image> images_mask;
    std::vector<std::vector<int>> visibility_vertex_to_image;
    std::vector<std::vector<int>> visibility_image_to_vertex;
    std::vector<open3d::pipelines::color_map::ImageWarpingField> warping_fields_init;

    // Create all debugging directories. We don't delete any existing files but
    // will overwrite them if the names are the same.
    if (!option.debug_output_dir_.empty()) {
        std::vector<std::string> dirs{
                option.debug_output_dir_,
                option.debug_output_dir_ + "/non_rigid",
                option.debug_output_dir_ + "/non_rigid/images_mask",
                option.debug_output_dir_ + "/non_rigid/opt_mesh",
                option.debug_output_dir_ + "/non_rigid/opt_camera_trajectory",
                option.debug_output_dir_ + "/non_rigid/warping_fields" };
        for (const std::string& dir : dirs) {
            if (open3d::utility::filesystem::DirectoryExists(dir)) {
                std::cout << "Directory exists: " << dir << std::endl;
            }
            else {
                if (open3d::utility::filesystem::MakeDirectoryHierarchy(dir)) {
                    std::cout << "Directory created: " << dir << std::endl;
                }
                else {
                    std::cout << "Directory creation failed: " << dir << std::endl;
                }
            }
        }
    }

    std::cout << "[ColorMapOptimization] CreateUtilImagesFromRGBD" << std::endl;
    std::tie(images_gray, images_dx, images_dy, images_color, images_depth) =
        CreateUtilImagesFromRGBD(rgbd_image_vector);

    std::cout << "[ColorMapOptimization] CreateDepthBoundaryMasks" << std::endl;
    images_mask = CreateDepthBoundaryMasks(
        images_depth, option.depth_threshold_for_discontinuity_check_,
        option.half_dilation_kernel_size_for_discontinuity_map_);
    if (!option.debug_output_dir_.empty()) {
        for (size_t i = 0; i < images_mask.size(); ++i) {
            std::string file_name = fmt::format(
                "{}/{}.png",
                option.debug_output_dir_ + "/non_rigid/images_mask", i);
            open3d::io::WriteImage(file_name, images_mask[i]);
        }
    }

    std::cout << "[ColorMapOptimization] CreateVertexAndImageVisibility" << std::endl;
    std::tie(visibility_vertex_to_image, visibility_image_to_vertex) =
        CreateVertexAndImageVisibility(
            opt_mesh, images_depth, images_mask, opt_camera_trajectory,
            option.maximum_allowable_depth_,
            option.depth_threshold_for_visibility_check_);

    std::cout << "[ColorMapOptimization] Non-Rigid Optimization" << std::endl;
    warping_fields = CreateWarpingFields(images_gray,
        option.number_of_vertical_anchors_);
    warping_fields_init = CreateWarpingFields(
        images_gray, option.number_of_vertical_anchors_);
    std::vector<double> proxy_intensity;
    size_t n_vertex = opt_mesh.vertices_.size();
    int n_camera = int(opt_camera_trajectory.parameters_.size());
    SetProxyIntensityForVertex(opt_mesh, images_gray, warping_fields,
        opt_camera_trajectory,
        visibility_vertex_to_image, proxy_intensity,
        option.image_boundary_margin_);
    for (int itr = 0; itr < option.maximum_iteration_; itr++) {
        std::cout << "Iteration #" << (itr + 1) << std::endl;
        double residual = 0.0;
        double residual_reg = 0.0;
#pragma omp parallel for schedule(static)
        for (int c = 0; c < n_camera; c++) {
            int nonrigidval = warping_fields[c].anchor_w_ *
                warping_fields[c].anchor_h_ * 2;
            double rr_reg = 0.0;

            Eigen::Matrix4d pose;
            pose = opt_camera_trajectory.parameters_[c].extrinsic_;

            auto intrinsic = opt_camera_trajectory.parameters_[c]
                .intrinsic_.intrinsic_matrix_;
            auto extrinsic = opt_camera_trajectory.parameters_[c].extrinsic_;
            Eigen::Matrix4d intr = Eigen::Matrix4d::Zero();
            intr.block<3, 3>(0, 0) = intrinsic;
            intr(3, 3) = 1.0;

            auto f_lambda = [&](int i, Eigen::Vector14d& J_r, double& r,
                Eigen::Vector14i& pattern) {
                    ComputeJacobianAndResidualNonRigid(
                        i, J_r, r, pattern, opt_mesh, proxy_intensity,
                        images_gray[c], images_dx[c], images_dy[c],
                        warping_fields[c], intr, extrinsic,
                        visibility_image_to_vertex[c],
                        option.image_boundary_margin_);
            };
            Eigen::MatrixXd JTJ;
            Eigen::VectorXd JTr;
            double r2;
            std::tie(JTJ, JTr, r2) =
                ComputeJTJandJTrNonRigid<Eigen::Vector14d, Eigen::Vector14i,
                Eigen::MatrixXd, Eigen::VectorXd>(
                    f_lambda, int(visibility_image_to_vertex[c].size()),
                    nonrigidval, false);

            double weight = option.non_rigid_anchor_point_weight_ *
                visibility_image_to_vertex[c].size() / n_vertex;
            for (int j = 0; j < nonrigidval; j++) {
                double r = weight * (warping_fields[c].flow_(j) -
                    warping_fields_init[c].flow_(j));
                JTJ(6 + j, 6 + j) += weight * weight;
                JTr(6 + j) += weight * r;
                rr_reg += r * r;
            }

            bool success;
            Eigen::VectorXd result;
            std::tie(success, result) = open3d::utility::SolveLinearSystemPSD(
                JTJ, -JTr, /*prefer_sparse=*/false,
                /*check_symmetric=*/false,
                /*check_det=*/false, /*check_psd=*/false);
            Eigen::Vector6d result_pose;
            result_pose << result.block(0, 0, 6, 1);
            auto delta = open3d::utility::TransformVector6dToMatrix4d(result_pose);
            pose = delta * pose;

            for (int j = 0; j < nonrigidval; j++) {
                warping_fields[c].flow_(j) += result(6 + j);
            }
            opt_camera_trajectory.parameters_[c].extrinsic_ = pose;

#pragma omp critical
            {
                residual += r2;
                residual_reg += rr_reg;
            }
        }
        std::cout << "Residual error : " << residual << ", reg : " << residual_reg << std::endl;
        SetProxyIntensityForVertex(opt_mesh, images_gray, warping_fields,
            opt_camera_trajectory,
            visibility_vertex_to_image, proxy_intensity,
            option.image_boundary_margin_);

        if (!option.debug_output_dir_.empty()) {
            // Save opt_mesh.
            SetGeometryColorAverage(opt_mesh, images_color, warping_fields,
                opt_camera_trajectory,
                visibility_vertex_to_image,
                option.image_boundary_margin_,
                option.invisible_vertex_color_knn_);
            std::string file_name = fmt::format(
                "{}/iter_{}.ply",
                option.debug_output_dir_ + "/non_rigid/opt_mesh", itr);
            open3d::io::WriteTriangleMesh(file_name, opt_mesh);

            // Save opt_camera_trajectory.
            file_name = fmt::format("{}/iter_{}.json",
                option.debug_output_dir_ +
                "/non_rigid/opt_camera_trajectory",
                itr);
            open3d::io::WritePinholeCameraTrajectory(file_name, opt_camera_trajectory);

            // Save warping_fields.
            for (size_t i = 0; i < warping_fields.size(); ++i) {
                file_name = fmt::format(
                    "{}/iter_{}_camera_{}.json",
                    option.debug_output_dir_ + "/non_rigid/warping_fields",
                    itr, i);
                open3d::io::WriteImageWarpingField(file_name, warping_fields[i]);
            }
        }
    }

    std::cout << "[ColorMapOptimization] Set Mesh Color" << std::endl;
    SetGeometryColorAverage(opt_mesh, images_color, warping_fields,
        opt_camera_trajectory, visibility_vertex_to_image,
        option.image_boundary_margin_,
        option.invisible_vertex_color_knn_);

    return opt_mesh;
}
