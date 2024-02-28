// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <vector>

#include "open3d/visualization/shader/ShaderWrapper.h"
#include "open3d/visualization/shader/PhongShader.h"

#include "pc_annotation/RenderOptionForAnnotation.h"

#include "pc_annotation/shader/ShaderWrapperForAnnotation.h"

namespace open3d {
namespace visualization {

namespace glsl {


class PhongShaderForAnnotation : public ShaderWrapperForAnnotation {
public:
    ~PhongShaderForAnnotation() override { Release(); }

protected:
    PhongShaderForAnnotation(const std::string &name) : ShaderWrapperForAnnotation(name) { Compile(); }

protected:
    bool Compile() final;
    void Release() final;
    bool BindGeometry(const geometry::Geometry &geometry,
                      const RenderOption &option,
                      const ViewControl &view) final;

    bool BindGeometry(const geometry::Geometry &geometry,
                      const std::vector<std::vector<int>> &labels,
                      const RenderOptionForAnnotation &option,
                      const ViewControl &view) final;

    bool RenderGeometry(const geometry::Geometry &geometry,
                        const RenderOption &option,
                        const ViewControl &view) final;
    void UnbindGeometry() final;

protected:
    virtual bool PrepareRendering(const geometry::Geometry &geometry,
                                  const RenderOption &option,
                                  const ViewControl &view) = 0;
    virtual bool PrepareBinding(const geometry::Geometry &geometry,
                                const RenderOption &option,
                                const ViewControl &view,
                                std::vector<Eigen::Vector3f> &points,
                                std::vector<Eigen::Vector3f> &normals,
                                std::vector<Eigen::Vector3f> &colors) = 0;

    virtual bool PrepareBinding(const geometry::Geometry &geometry,
                        const RenderOptionForAnnotation &option,
                        const ViewControl &view,
                        std::vector<Eigen::Vector3f> &points,
                        std::vector<Eigen::Vector3f> &normals,
                        std::vector<Eigen::Vector3f> &colors,
                        const std::vector<std::vector<int>> &labels) = 0;

protected:
    void SetLighting(const ViewControl &view, const RenderOption &option);

protected:
    GLuint vertex_position_;
    GLuint vertex_position_buffer_;
    GLuint vertex_color_;
    GLuint vertex_color_buffer_;
    GLuint vertex_normal_;
    GLuint vertex_normal_buffer_;
    GLuint MVP_;
    GLuint V_;
    GLuint M_;
    GLuint light_position_world_;
    GLuint light_color_;
    GLuint light_diffuse_power_;
    GLuint light_specular_power_;
    GLuint light_specular_shininess_;
    GLuint light_ambient_;

    // At most support 4 lights
    gl_util::GLMatrix4f light_position_world_data_;
    gl_util::GLMatrix4f light_color_data_;
    gl_util::GLVector4f light_diffuse_power_data_;
    gl_util::GLVector4f light_specular_power_data_;
    gl_util::GLVector4f light_specular_shininess_data_;
    gl_util::GLVector4f light_ambient_data_;
};

class PhongShaderForAnnotationForPointCloud : public PhongShaderForAnnotation {
public:
    PhongShaderForAnnotationForPointCloud() : PhongShaderForAnnotation("PhongShaderForAnnotationForPointCloud") {
        primary_color.push_back(255 << 16);
        primary_color.push_back(255 << 8);
        primary_color.push_back(255);
    }

protected:
    bool PrepareRendering(const geometry::Geometry &geometry,
                          const RenderOption &option,
                          const ViewControl &view) final;
    bool PrepareBinding(const geometry::Geometry &geometry,
                        const RenderOption &option,
                        const ViewControl &view,
                        std::vector<Eigen::Vector3f> &points,
                        std::vector<Eigen::Vector3f> &normals,
                        std::vector<Eigen::Vector3f> &colors) final;
    
    bool PrepareRendering(const geometry::Geometry &geometry,
                          const RenderOptionForAnnotation &option,
                          const ViewControl &view);
    bool PrepareBinding(const geometry::Geometry &geometry,
                        const RenderOptionForAnnotation &option,
                        const ViewControl &view,
                        std::vector<Eigen::Vector3f> &points,
                        std::vector<Eigen::Vector3f> &normals,
                        std::vector<Eigen::Vector3f> &colors);
    bool PrepareBinding(const geometry::Geometry &geometry,
                        const RenderOptionForAnnotation &option,
                        const ViewControl &view,
                        std::vector<Eigen::Vector3f> &points,
                        std::vector<Eigen::Vector3f> &normals,
                        std::vector<Eigen::Vector3f> &colors,
                        const std::vector<std::vector<int>> &labels);

    std::vector<long> primary_color;
};

}  // namespace glsl

}  // namespace visualization
}  // namespace open3d
