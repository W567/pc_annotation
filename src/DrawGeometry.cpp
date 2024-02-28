// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "pc_annotation/DrawGeometry.h"

#include "open3d/utility/Logging.h"

#include "pc_annotation/VisualizerWithAnnotation.h"

namespace open3d {
namespace visualization {

bool DrawGeometriesWithAnnotation(
        const std::vector<std::shared_ptr<const geometry::PointCloud>> &geometry_ptrs,
        const std::string &filename,
        const std::string &window_name /* = "Open3D"*/,
        int width /* = 640*/,
        int height /* = 480*/,
        int left /* = 50*/,
        int top /* = 50*/) {
    VisualizerWithAnnotation visualizer;
    visualizer.SetFilename(filename);
    if (!visualizer.CreateVisualizerWindow(window_name, width, height, left, top)) {
        utility::LogWarning("[DrawGeometriesWithAnnotation] Failed creating OpenGL window.");
        return false;
    }
    for (const auto &geometry_ptr : geometry_ptrs) {
        if (!visualizer.AddGeometry(geometry_ptr)) {
            utility::LogWarning("[DrawGeometriesWithAnnotation] Failed adding geometry.");
            utility::LogWarning("[DrawGeometriesWithAnnotation] Possibly due to bad "
                                "geometry or wrong geometry type.");
            return false;
        }
    }
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
    return true;
}

}  // namespace visualization
}  // namespace open3d
