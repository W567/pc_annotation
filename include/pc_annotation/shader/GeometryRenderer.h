// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include "open3d/visualization/shader/GeometryRenderer.h"

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
    PhongShaderForPointCloud phong_point_shader_;  // TODO should be changed to forAnnotation
};

}  // namespace glsl

}  // namespace visualization
}  // namespace open3d
