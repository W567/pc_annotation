// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "open3d/geometry/Geometry.h"

namespace open3d {
namespace visualization {

bool DrawGeometriesWithAnnotation(
        const std::vector<std::shared_ptr<const geometry::Geometry>>
                &geometry_ptrs,
        const std::string &filename,
        const std::string &window_name = "Open3D",
        int width = 640,
        int height = 480,
        int left = 50,
        int top = 50);

}  // namespace visualization
}  // namespace open3d
