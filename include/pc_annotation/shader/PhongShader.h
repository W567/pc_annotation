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

namespace open3d {
namespace visualization {

namespace glsl {

class PhongShaderForAnnotation : public PhongShader {
public:
    ~PhongShaderForAnnotation() override { Release(); }

protected:
    PhongShaderForAnnotation(const std::string &name) : PhongShader(name) { Compile(); }

protected:
    bool BindGeometry(const geometry::Geometry &geometry,
                      const std::vector<std::vector<int>> &labels,
                      const RenderOption &option,
                      const ViewControl &view) final;

protected:
    virtual bool PrepareBinding(const geometry::Geometry &geometry,
                        const RenderOption &option,
                        const ViewControl &view,
                        std::vector<Eigen::Vector3f> &points,
                        std::vector<Eigen::Vector3f> &normals,
                        std::vector<Eigen::Vector3f> &colors,
                        const std::vector<std::vector<int>> &labels) = 0;
};

class PhongShaderForAnnotationForPointCloud : public PhongShaderForAnnotation {
public:
    PhongShaderForAnnotationForPointCloud() : PhongShaderForAnnotation("PhongShaderForAnnotationForPointCloud") {
        primary_color.push_back(255 << 16);
        primary_color.push_back(255 << 8);
        primary_color.push_back(255);
    }

protected:
    bool PrepareBinding(const geometry::Geometry &geometry,
                        const RenderOption &option,
                        const ViewControl &view,
                        std::vector<Eigen::Vector3f> &points,
                        std::vector<Eigen::Vector3f> &normals,
                        std::vector<Eigen::Vector3f> &colors,
                        const std::vector<std::vector<int>> &labels) final;
    
    std::vector<long> primary_color;
};

class PhongShaderForAnnotationForTriangleMesh : public PhongShaderForAnnotation {
public:
    PhongShaderForAnnotationForTriangleMesh() : PhongShaderForAnnotation("PhongShaderForAnnotationForTriangleMesh") {}

protected:
    bool PrepareBinding(const geometry::Geometry &geometry,
                        const RenderOption &option,
                        const ViewControl &view,
                        std::vector<Eigen::Vector3f> &points,
                        std::vector<Eigen::Vector3f> &normals,
                        std::vector<Eigen::Vector3f> &colors,
                        const std::vector<std::vector<int>> &labels) final;
};

}  // namespace glsl

}  // namespace visualization
}  // namespace open3d
