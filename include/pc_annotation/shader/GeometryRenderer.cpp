// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "pc_annotation/shader/GeometryRenderer.h"

#include "open3d/geometry/Image.h"
#include "open3d/geometry/LineSet.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/visualization/utility/PointCloudPicker.h"
#include "open3d/visualization/utility/SelectionPolygon.h"
#include "open3d/visualization/visualizer/RenderOptionWithEditing.h"
#include "open3d/utility/Logging.h"

namespace open3d {
namespace visualization {

namespace glsl {

bool GeometryRendererForAnnotation::AddLabels(const std::vector<std::vector<int>>& labels)
{
    label_ptr_ = &labels;
    return UpdateGeometry();
}

bool PointCloudRendererForAnnotation::Render(const RenderOption &option,
                                             const ViewControl &view) {
    if (!is_visible_ || geometry_ptr_->IsEmpty()) return true;
    const auto &pointcloud = (const geometry::PointCloud &)(*geometry_ptr_);
    const auto &labels = *label_ptr_;
    bool success = true;
    if (pointcloud.HasNormals()) {
        if (option.point_color_option_ ==
            RenderOption::PointColorOption::Normal) {
            success &= normal_point_shader_.Render(pointcloud, option, view);
        } else {
            if (label_ptr_ == nullptr)
            {
                // utility::LogInfo("[Visualizer] ENTER 0 ");
                success &= phong_point_shader_.Render(pointcloud, option, view);
            }
            else if (labels.size() == 0 || pointcloud.points_.size() != labels[0].size())
            {
                // if (labels.size() == 0)
                // {
                //     utility::LogInfo("[Visualizer] ENTER 1, outer size: {:d}", labels.size());
                // }
                // else
                // {
                //     utility::LogInfo("[Visualizer] ENTER 1, outer size: {:d}, inner size: {:d}", labels.size(), labels[0].size());
                // }
                success &= phong_point_shader_.Render(pointcloud, option, view);
            }
            else
            {
                // utility::LogInfo("[Visualizer] ENTER 2, outer size: {:d}, inner size: {:d}", labels.size(), labels[0].size());
                success &= phong_point_shader_.Render(pointcloud, labels, option, view);
            }
        }
        if (option.point_show_normal_) {
            success &=
                    simpleblack_normal_shader_.Render(pointcloud, option, view);
        }
    } else {
        success &= simple_point_shader_.Render(pointcloud, option, view);
    }
    return success;
}

}  // namespace glsl

}  // namespace visualization
}  // namespace open3d
