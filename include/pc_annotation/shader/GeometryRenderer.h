// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include "open3d/visualization/shader/GeometryRenderer.h"

#include "pc_annotation/shader/PhongShader.h"
#include "pc_annotation/RenderOptionForAnnotation.h"

#include "open3d/utility/Logging.h"

namespace open3d {
namespace visualization {

namespace glsl {

class PointCloudRendererForAnnotation : public PointCloudRenderer {
public:
    ~PointCloudRendererForAnnotation() override {}

    bool AddLabels(const std::vector<std::vector<int>>& labels);

    bool UpdateGeometry() override;

public:
    bool Render(const RenderOptionForAnnotation &option, const ViewControl &view);

protected:
    PhongShaderForAnnotationForPointCloud phong_point_shader_;
    const std::vector<std::vector<int>>* label_ptr_;
};

}  // namespace glsl

}  // namespace visualization
}  // namespace open3d
