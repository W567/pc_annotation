// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "pc_annotation/VisualizerForAnnotation.h"

#include "open3d/geometry/TriangleMesh.h"

namespace open3d {

namespace visualization {

void VisualizerForAnnotation::BuildUtilities() {
    glfwMakeContextCurrent(window_);

    // 0. Build coordinate frame
    const auto boundingbox = GetViewControl().GetBoundingBox();
    double extent = std::max(0.01, boundingbox.GetMaxExtent() * 0.2);
    coordinate_frame_mesh_ptr_ = geometry::TriangleMesh::CreateCoordinateFrame(
            extent); // boundingbox.min_bound_
    coordinate_frame_mesh_renderer_ptr_ =
            std::make_shared<glsl::CoordinateFrameRenderer>();
    if (!coordinate_frame_mesh_renderer_ptr_->AddGeometry(
                coordinate_frame_mesh_ptr_)) {
        return;
    }
    utility_ptrs_.push_back(coordinate_frame_mesh_ptr_);
    utility_renderer_ptrs_.push_back(coordinate_frame_mesh_renderer_ptr_);
}

}  // namespace visualization
}  // namespace open3d
