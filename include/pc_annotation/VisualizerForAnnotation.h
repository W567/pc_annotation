// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include "open3d/visualization/visualizer/Visualizer.h"
#include "pc_annotation/RenderOptionForAnnotation.h"

#include "pc_annotation/shader/GeometryRenderer.h"

#include "open3d/utility/Logging.h"

namespace open3d {

namespace visualization {

class VisualizerForAnnotation : public Visualizer {

protected:
    void KeyPressCallback(
            GLFWwindow *window, int key, int scancode, int action, int mods);
    
    void Render(bool render_screen = false);

public:
    VisualizerForAnnotation() {}
    void BuildUtilities() override;
    RenderOptionForAnnotation &GetRenderOption() { return *render_option_ptr_; }

    bool PollEvents();

    bool WaitEvents();

    void Run();

    void WindowRefreshCallback(GLFWwindow *window);

    bool AddGeometry(
        std::shared_ptr<const geometry::Geometry> geometry_ptr,
        bool reset_bounding_box = true);

protected:
    // rendering properties
    std::unique_ptr<RenderOptionForAnnotation> render_option_ptr_;

    std::unordered_set<std::shared_ptr<glsl::PointCloudRendererForAnnotation>>
            geometry_renderer_ptrs_;
};

}  // namespace visualization
}  // namespace open3d
