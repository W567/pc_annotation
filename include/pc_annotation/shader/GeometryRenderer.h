// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include "open3d/visualization/shader/GeometryRenderer.h"

#include "pc_annotation/shader/PhongShader.h"

namespace open3d {
namespace visualization {

namespace glsl {

class GeometryRendererForAnnotation : public GeometryRenderer{
public:
    virtual ~GeometryRendererForAnnotation() {}

    bool AddLabels(const std::vector<std::vector<int>>& labels);

protected:
    const std::vector<std::vector<int>>* label_ptr_;
};


class PointCloudRendererForAnnotation : public PointCloudRenderer {
public:
    ~PointCloudRendererForAnnotation() override {}

public:
    bool Render(const RenderOption &option, const ViewControl &view) override;

protected:
    PhongShaderForAnnotationForPointCloud phong_point_shader_;
};

}  // namespace glsl

}  // namespace visualization
}  // namespace open3d
