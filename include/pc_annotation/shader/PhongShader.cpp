// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "pc_annotation/shader/PhongShader.h"

#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/visualization/shader/Shader.h"
#include "open3d/visualization/utility/ColorMap.h"

namespace open3d {
namespace visualization {

namespace glsl {

bool PhongShaderForAnnotation::BindGeometry(const geometry::Geometry &geometry,
                                            const std::vector<std::vector<int>> &labels,
                                            const RenderOptionForAnnotation &option,
                                            const ViewControl &view) {
    // If there is already geometry, we first unbind it.
    // We use GL_STATIC_DRAW. When geometry changes, we clear buffers and
    // rebind the geometry. Note that this approach is slow. If the geometry is
    // changing per frame, consider implementing a new ShaderWrapper using
    // GL_STREAM_DRAW, and replace UnbindGeometry() with Buffer Object
    // Streaming mechanisms.
    UnbindGeometry();

    // Prepare data to be passed to GPU
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Vector3f> colors;
    if (!PrepareBinding(geometry, option, view, points, normals, colors, labels)) {
        PrintShaderWarning("Binding failed when preparing data.");
        return false;
    }

    // Create buffers and bind the geometry
    glGenBuffers(1, &vertex_position_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_position_buffer_);
    glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(Eigen::Vector3f),
                 points.data(), GL_STATIC_DRAW);
    glGenBuffers(1, &vertex_normal_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_normal_buffer_);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(Eigen::Vector3f),
                 normals.data(), GL_STATIC_DRAW);
    glGenBuffers(1, &vertex_color_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer_);
    glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(Eigen::Vector3f),
                 colors.data(), GL_STATIC_DRAW);
    bound_ = true;
    return true;
}


bool PhongShaderForAnnotationForPointCloud::PrepareRendering(
        const geometry::Geometry &geometry,
        const RenderOptionForAnnotation &option,
        const ViewControl &view) {
    if (geometry.GetGeometryType() !=
        geometry::Geometry::GeometryType::PointCloud) {
        PrintShaderWarning("Rendering type is not geometry::PointCloud.");
        return false;
    }
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GLenum(option.GetGLDepthFunc()));
    glPointSize(GLfloat(option.point_size_));
    SetLighting(view, option);
    return true;
}

bool PhongShaderForAnnotationForPointCloud::PrepareBinding(
        const geometry::Geometry &geometry,
        const RenderOptionForAnnotation &option,
        const ViewControl &view,
        std::vector<Eigen::Vector3f> &points,
        std::vector<Eigen::Vector3f> &normals,
        std::vector<Eigen::Vector3f> &colors) {
    if (geometry.GetGeometryType() !=
        geometry::Geometry::GeometryType::PointCloud) {
        PrintShaderWarning("Rendering type is not geometry::PointCloud.");
        return false;
    }
    const geometry::PointCloud &pointcloud =
            (const geometry::PointCloud &)geometry;
    if (!pointcloud.HasPoints()) {
        PrintShaderWarning("Binding failed with empty pointcloud.");
        return false;
    }
    if (!pointcloud.HasNormals()) {
        PrintShaderWarning("Binding failed with pointcloud with no normals.");
        return false;
    }
    const ColorMap &global_color_map = *GetGlobalColorMap();
    points.resize(pointcloud.points_.size());
    normals.resize(pointcloud.points_.size());
    colors.resize(pointcloud.points_.size());
    for (size_t i = 0; i < pointcloud.points_.size(); i++) {
        const auto &point = pointcloud.points_[i];
        const auto &normal = pointcloud.normals_[i];
        points[i] = point.cast<float>();
        normals[i] = normal.cast<float>();
        Eigen::Vector3d color;
        switch (option.point_color_option_) {
            case RenderOptionForAnnotation::PointColorOption::XCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetXPercentage(point(0)));
                break;
            case RenderOptionForAnnotation::PointColorOption::YCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetYPercentage(point(1)));
                break;
            case RenderOptionForAnnotation::PointColorOption::ZCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetZPercentage(point(2)));
                break;
            case RenderOptionForAnnotation::PointColorOption::Color:
            case RenderOptionForAnnotation::PointColorOption::Default:
            default:
                if (pointcloud.HasColors()) {
                    color = pointcloud.colors_[i];
                } else {
                    // color = global_color_map.GetColor(
                    //         view.GetBoundingBox().GetZPercentage(point(2)));
                    color = Eigen::Vector3d::Zero(3);
                }
                break;
        }
        colors[i] = color.cast<float>();
    }
    draw_arrays_mode_ = GL_POINTS;
    draw_arrays_size_ = GLsizei(points.size());
    return true;
}



bool PhongShaderForAnnotationForPointCloud::PrepareRendering(
        const geometry::Geometry &geometry,
        const RenderOption &option,
        const ViewControl &view) {
    if (geometry.GetGeometryType() !=
        geometry::Geometry::GeometryType::PointCloud) {
        PrintShaderWarning("Rendering type is not geometry::PointCloud.");
        return false;
    }
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GLenum(option.GetGLDepthFunc()));
    glPointSize(GLfloat(option.point_size_));
    SetLighting(view, option);
    return true;
}

bool PhongShaderForAnnotationForPointCloud::PrepareBinding(
        const geometry::Geometry &geometry,
        const RenderOption &option,
        const ViewControl &view,
        std::vector<Eigen::Vector3f> &points,
        std::vector<Eigen::Vector3f> &normals,
        std::vector<Eigen::Vector3f> &colors) {
    if (geometry.GetGeometryType() !=
        geometry::Geometry::GeometryType::PointCloud) {
        PrintShaderWarning("Rendering type is not geometry::PointCloud.");
        return false;
    }
    const geometry::PointCloud &pointcloud =
            (const geometry::PointCloud &)geometry;
    if (!pointcloud.HasPoints()) {
        PrintShaderWarning("Binding failed with empty pointcloud.");
        return false;
    }
    if (!pointcloud.HasNormals()) {
        PrintShaderWarning("Binding failed with pointcloud with no normals.");
        return false;
    }
    const ColorMap &global_color_map = *GetGlobalColorMap();
    points.resize(pointcloud.points_.size());
    normals.resize(pointcloud.points_.size());
    colors.resize(pointcloud.points_.size());
    for (size_t i = 0; i < pointcloud.points_.size(); i++) {
        const auto &point = pointcloud.points_[i];
        const auto &normal = pointcloud.normals_[i];
        points[i] = point.cast<float>();
        normals[i] = normal.cast<float>();
        Eigen::Vector3d color;
        switch (option.point_color_option_) {
            case RenderOption::PointColorOption::XCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetXPercentage(point(0)));
                break;
            case RenderOption::PointColorOption::YCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetYPercentage(point(1)));
                break;
            case RenderOption::PointColorOption::ZCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetZPercentage(point(2)));
                break;
            case RenderOption::PointColorOption::Color:
            case RenderOption::PointColorOption::Default:
            default:
                if (pointcloud.HasColors()) {
                    color = pointcloud.colors_[i];
                } else {
                    // color = global_color_map.GetColor(
                    //         view.GetBoundingBox().GetZPercentage(point(2)));
                    color = Eigen::Vector3d::Zero(3);
                }
                break;
        }
        colors[i] = color.cast<float>();
    }
    draw_arrays_mode_ = GL_POINTS;
    draw_arrays_size_ = GLsizei(points.size());
    return true;
}


bool PhongShaderForAnnotationForPointCloud::PrepareBinding(
        const geometry::Geometry &geometry,
        const RenderOptionForAnnotation &option,
        const ViewControl &view,
        std::vector<Eigen::Vector3f> &points,
        std::vector<Eigen::Vector3f> &normals,
        std::vector<Eigen::Vector3f> &colors,
        const std::vector<std::vector<int>> &labels) {
    if (geometry.GetGeometryType() !=
        geometry::Geometry::GeometryType::PointCloud) {
        PrintShaderWarning("Rendering type is not geometry::PointCloud.");
        return false;
    }
    const geometry::PointCloud &pointcloud =
            (const geometry::PointCloud &)geometry;
    if (!pointcloud.HasPoints()) {
        PrintShaderWarning("Binding failed with empty pointcloud.");
        return false;
    }
    if (!pointcloud.HasNormals()) {
        PrintShaderWarning("Binding failed with pointcloud with no normals.");
        return false;
    }
    const ColorMap &global_color_map = *GetGlobalColorMap();
    points.resize(pointcloud.points_.size());
    normals.resize(pointcloud.points_.size());
    colors.resize(pointcloud.points_.size());
    for (size_t i = 0; i < pointcloud.points_.size(); i++) {
        const auto &point = pointcloud.points_[i];
        const auto &normal = pointcloud.normals_[i];
        points[i] = point.cast<float>();
        normals[i] = normal.cast<float>();
        Eigen::Vector3d color;
        switch (option.point_color_option_) {
            case RenderOptionForAnnotation::PointColorOption::XCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetXPercentage(point(0)));
                break;
            case RenderOptionForAnnotation::PointColorOption::YCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetYPercentage(point(1)));
                break;
            case RenderOptionForAnnotation::PointColorOption::ZCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetZPercentage(point(2)));
                break;
            
            case RenderOptionForAnnotation::PointColorOption::Label:
                if (labels.size() != 0)
                {
                    int size = (int)labels.size();
                    for (int k = 0; k < size; k++)
                    {
                        if (labels[k][i] == 1)
                        {
                            color[k] = primary_color[k];
                        }
                        else
                        {
                            color[k] = 0;
                        }
                    }
                    if (size < 3)
                    {
                        for (int k = 0; k < 3 - size; k++)
                        {
                            color[size + k] = 0;
                        }
                    }
                }
                else
                {
                    // color = global_color_map.GetColor(
                    //         view.GetBoundingBox().GetZPercentage(point(2)));
                    color = Eigen::Vector3d::Zero(3);
                }
                break;
            case RenderOptionForAnnotation::PointColorOption::Color:
            case RenderOptionForAnnotation::PointColorOption::Default:
            default:
                if (pointcloud.HasColors()) {
                    color = pointcloud.colors_[i];
                } else {
                    // color = global_color_map.GetColor(
                    //         view.GetBoundingBox().GetZPercentage(point(2)));
                    color = Eigen::Vector3d::Zero(3);
                }
                break;
        }
        colors[i] = color.cast<float>();
    }
    draw_arrays_mode_ = GL_POINTS;
    draw_arrays_size_ = GLsizei(points.size());
    return true;
}


bool PhongShaderForAnnotationForPointCloud::PrepareBinding(
        const geometry::Geometry &geometry,
        const RenderOption &option,
        const ViewControl &view,
        std::vector<Eigen::Vector3f> &points,
        std::vector<Eigen::Vector3f> &normals,
        std::vector<Eigen::Vector3f> &colors,
        const std::vector<std::vector<int>> &labels) {
    if (geometry.GetGeometryType() !=
        geometry::Geometry::GeometryType::PointCloud) {
        PrintShaderWarning("Rendering type is not geometry::PointCloud.");
        return false;
    }
    const geometry::PointCloud &pointcloud =
            (const geometry::PointCloud &)geometry;
    if (!pointcloud.HasPoints()) {
        PrintShaderWarning("Binding failed with empty pointcloud.");
        return false;
    }
    if (!pointcloud.HasNormals()) {
        PrintShaderWarning("Binding failed with pointcloud with no normals.");
        return false;
    }
    const ColorMap &global_color_map = *GetGlobalColorMap();
    points.resize(pointcloud.points_.size());
    normals.resize(pointcloud.points_.size());
    colors.resize(pointcloud.points_.size());
    for (size_t i = 0; i < pointcloud.points_.size(); i++) {
        const auto &point = pointcloud.points_[i];
        const auto &normal = pointcloud.normals_[i];
        points[i] = point.cast<float>();
        normals[i] = normal.cast<float>();
        Eigen::Vector3d color;
        switch (option.point_color_option_) {
            case RenderOption::PointColorOption::XCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetXPercentage(point(0)));
                break;
            case RenderOption::PointColorOption::YCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetYPercentage(point(1)));
                break;
            case RenderOption::PointColorOption::ZCoordinate:
                color = global_color_map.GetColor(
                        view.GetBoundingBox().GetZPercentage(point(2)));
                break;
            
            case RenderOption::PointColorOption::Label:
                if (labels.size() != 0)
                {
                    int size = (int)labels.size();
                    for (int k = 0; k < size; k++)
                    {
                        if (labels[k][i] == 1)
                        {
                            color[k] = primary_color[k];
                        }
                        else
                        {
                            color[k] = 0;
                        }
                    }
                    if (size < 3)
                    {
                        for (int k = 0; k < 3 - size; k++)
                        {
                            color[size + k] = 0;
                        }
                    }
                }
                else
                {
                    // color = global_color_map.GetColor(
                    //         view.GetBoundingBox().GetZPercentage(point(2)));
                    color = Eigen::Vector3d::Zero(3);
                }
                break;
            case RenderOption::PointColorOption::Color:
            case RenderOption::PointColorOption::Default:
            default:
                if (pointcloud.HasColors()) {
                    color = pointcloud.colors_[i];
                } else {
                    // color = global_color_map.GetColor(
                    //         view.GetBoundingBox().GetZPercentage(point(2)));
                    color = Eigen::Vector3d::Zero(3);
                }
                break;
        }
        colors[i] = color.cast<float>();
    }
    draw_arrays_mode_ = GL_POINTS;
    draw_arrays_size_ = GLsizei(points.size());
    return true;
}

bool PhongShaderForAnnotationForTriangleMesh::PrepareBinding(
        const geometry::Geometry &geometry,
        const RenderOptionForAnnotation &option,
        const ViewControl &view,
        std::vector<Eigen::Vector3f> &points,
        std::vector<Eigen::Vector3f> &normals,
        std::vector<Eigen::Vector3f> &colors,
        const std::vector<std::vector<int>> &labels) {
    if (geometry.GetGeometryType() !=
                geometry::Geometry::GeometryType::TriangleMesh &&
        geometry.GetGeometryType() !=
                geometry::Geometry::GeometryType::HalfEdgeTriangleMesh) {
        PrintShaderWarning("Rendering type is not geometry::TriangleMesh.");
        return false;
    }
    const geometry::TriangleMesh &mesh =
            (const geometry::TriangleMesh &)geometry;
    if (!mesh.HasTriangles()) {
        PrintShaderWarning("Binding failed with empty triangle mesh.");
        return false;
    }
    if (!mesh.HasTriangleNormals() || !mesh.HasVertexNormals()) {
        PrintShaderWarning("Binding failed because mesh has no normals.");
        PrintShaderWarning("Call ComputeVertexNormals() before binding.");
        return false;
    }
    const ColorMap &global_color_map = *GetGlobalColorMap();
    points.resize(mesh.triangles_.size() * 3);
    normals.resize(mesh.triangles_.size() * 3);
    colors.resize(mesh.triangles_.size() * 3);

    for (size_t i = 0; i < mesh.triangles_.size(); i++) {
        const auto &triangle = mesh.triangles_[i];
        for (size_t j = 0; j < 3; j++) {
            size_t idx = i * 3 + j;
            size_t vi = triangle(j);
            const auto &vertex = mesh.vertices_[vi];
            points[idx] = vertex.cast<float>();

            Eigen::Vector3d color;
            switch (option.mesh_color_option_) {
                case RenderOptionForAnnotation::MeshColorOption::XCoordinate:
                    color = global_color_map.GetColor(
                            view.GetBoundingBox().GetXPercentage(vertex(0)));
                    break;
                case RenderOptionForAnnotation::MeshColorOption::YCoordinate:
                    color = global_color_map.GetColor(
                            view.GetBoundingBox().GetYPercentage(vertex(1)));
                    break;
                case RenderOptionForAnnotation::MeshColorOption::ZCoordinate:
                    color = global_color_map.GetColor(
                            view.GetBoundingBox().GetZPercentage(vertex(2)));
                    break;
                case RenderOptionForAnnotation::MeshColorOption::Color:
                    if (mesh.HasVertexColors()) {
                        color = mesh.vertex_colors_[vi];
                        break;
                    }
                    // fallthrough
                case RenderOptionForAnnotation::MeshColorOption::Default:
                default:
                    color = option.default_mesh_color_;
                    break;
            }
            colors[idx] = color.cast<float>();

            if (option.mesh_shade_option_ ==
                RenderOptionForAnnotation::MeshShadeOption::FlatShade) {
                normals[idx] = mesh.triangle_normals_[i].cast<float>();
            } else {
                normals[idx] = mesh.vertex_normals_[vi].cast<float>();
            }
        }
    }
    draw_arrays_mode_ = GL_TRIANGLES;
    draw_arrays_size_ = GLsizei(points.size());
    return true;
}

}  // namespace glsl

}  // namespace visualization
}  // namespace open3d
