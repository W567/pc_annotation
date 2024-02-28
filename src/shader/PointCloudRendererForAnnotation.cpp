// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "pc_annotation/shader/PointCloudRendererForAnnotation.h"

#include "open3d/geometry/PointCloud.h"

namespace open3d {
namespace visualization {
namespace glsl {

bool PointCloudRendererForAnnotation::AddLabels(const std::vector<std::vector<int>>& labels) {
    label_ptr_ = &labels;
    return UpdateGeometry();
}

bool PointCloudRendererForAnnotation::UpdateGeometry() {
    simple_point_shader_.InvalidateGeometry();
    phong_point_shader_.InvalidateGeometry();
    normal_point_shader_.InvalidateGeometry();
    simpleblack_normal_shader_.InvalidateGeometry();
    return true;
}

bool PointCloudRendererForAnnotation::Render(const RenderOptionForAnnotation &option,
                                             const ViewControl &view) {
    if (!is_visible_ || geometry_ptr_->IsEmpty()) return true;
    const auto &pointcloud = (const geometry::PointCloud &)(*geometry_ptr_);
    const auto &labels = *label_ptr_;
    bool success = true;
    if (pointcloud.HasNormals()) {
        if (option.point_color_option_ ==
            RenderOptionForAnnotation::PointColorOption::Normal) {
            success &= normal_point_shader_.Render(pointcloud, option, view);
        } else {
            if (label_ptr_ == nullptr) {
                success &= phong_point_shader_.Render(pointcloud, option, view);
            } else if (labels.size() == 0 || pointcloud.points_.size() != labels[0].size()) {
                success &= phong_point_shader_.Render(pointcloud, option, view);
            } else {
                success &= phong_point_shader_.Render(pointcloud, labels, option, view);
            }
        }
        if (option.point_show_normal_) {
            success &= simpleblack_normal_shader_.Render(pointcloud, option, view);
        }
    } else {
        success &= simple_point_shader_.Render(pointcloud, option, view);
    }
    return success;
}

}  // namespace glsl
}  // namespace visualization
}  // namespace open3d
