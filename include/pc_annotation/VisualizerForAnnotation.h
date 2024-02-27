// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include "open3d/visualization/visualizer/Visualizer.h"
#include "pc_annotation/RenderOptionForAnnotation.h"

namespace open3d {

namespace visualization {

class VisualizerForAnnotation : public Visualizer {

protected:
    virtual void KeyPressCallback(
            GLFWwindow *window, int key, int scancode, int action, int mods);

public:
    VisualizerForAnnotation() {}
    virtual void BuildUtilities();
    RenderOptionForAnnotation &GetRenderOption() { return *render_option_ptr_; }

protected:
    // rendering properties
    std::unique_ptr<RenderOptionForAnnotation> render_option_ptr_;
};

}  // namespace visualization
}  // namespace open3d
