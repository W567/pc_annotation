// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "open3d/geometry/PointCloud.h"

namespace open3d {
namespace visualization {

bool DrawGeometriesWithAnnotation(
        const std::vector<std::shared_ptr<const geometry::PointCloud>> &geometry_ptrs,
        const std::string &filename,
        const std::string &window_name = "Open3D",
        int width = 640,
        int height = 480,
        int left = 50,
        int top = 50);

}  // namespace visualization
}  // namespace open3d
