// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "pc_annotation/shader/ShaderWrapperForAnnotation.h"

#include "open3d/geometry/Geometry.h"
#include "open3d/utility/Logging.h"

namespace open3d {
namespace visualization {

namespace glsl {

bool ShaderWrapperForAnnotation::Render(const geometry::Geometry &geometry,
                                        const std::vector<std::vector<int>> &labels,
                                        const RenderOption &option,
                                        const ViewControl &view) {
    if (!compiled_) {
        Compile();
    }
    if (!bound_) {
        BindGeometry(geometry, labels, option, view);
    }
    if (!compiled_ || !bound_) {
        PrintShaderWarning("Something is wrong in compiling or binding.");
        return false;
    }
    return RenderGeometry(geometry, option, view);
}

}  // namespace glsl

}  // namespace visualization
}  // namespace open3d
