// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "pc_annotation/VisualizerForAnnotation.h"

#include "open3d/utility/Logging.h"

namespace open3d {
namespace visualization {

void VisualizerForAnnotation::Render(bool render_screen) {
    glfwMakeContextCurrent(window_);

    view_control_ptr_->SetViewMatrices();

    if (render_screen) {
        if (render_fbo_ != 0) {
            utility::LogWarning("Render framebuffer is not released.");
        }

        glGenFramebuffers(1, &render_fbo_);
        glBindFramebuffer(GL_FRAMEBUFFER, render_fbo_);

        int tex_w = view_control_ptr_->GetWindowWidth();
        int tex_h = view_control_ptr_->GetWindowHeight();

        glGenTextures(1, &render_rgb_tex_);
        glBindTexture(GL_TEXTURE_2D, render_rgb_tex_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex_w, tex_h, 0, GL_RGB,
                     GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, render_rgb_tex_, 0);

        glGenRenderbuffers(1, &render_depth_stencil_rbo_);
        glBindRenderbuffer(GL_RENDERBUFFER, render_depth_stencil_rbo_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, tex_w,
                              tex_h);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                                  GL_RENDERBUFFER, render_depth_stencil_rbo_);
    }

    glEnable(GL_MULTISAMPLE);
    glDisable(GL_BLEND);
    auto &background_color = render_option_ptr_->background_color_;
    glClearColor((GLclampf)background_color(0), (GLclampf)background_color(1),
                 (GLclampf)background_color(2), 1.0f);
    glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for (const auto &renderer_ptr : geometry_renderer_ptrs_) {
        renderer_ptr->Render(*render_option_ptr_, *view_control_ptr_);
    }
    for (const auto &renderer_ptr : utility_renderer_ptrs_) {
        RenderOption *opt = render_option_ptr_.get();
        auto optIt = utility_renderer_opts_.find(renderer_ptr);
        if (optIt != utility_renderer_opts_.end()) {
            opt = &optIt->second;
        }
        renderer_ptr->Render(*opt, *view_control_ptr_);
    }

    glfwSwapBuffers(window_);
}

bool VisualizerForAnnotation::AddGeometry(
        std::shared_ptr<const geometry::Geometry> geometry_ptr,
        bool reset_bounding_box) {
    if (!is_initialized_) { return false; }
    if (!geometry_ptr.get()) {
        utility::LogWarning("[AddGeometry] Invalid pointer. Possibly a null pointer or "
                            "None was passed in.");
        return false;
    }

    glfwMakeContextCurrent(window_);
    std::shared_ptr<glsl::PointCloudRendererForAnnotation> renderer_ptr;
    if (geometry_ptr->GetGeometryType() == geometry::Geometry::GeometryType::PointCloud) {
        renderer_ptr = std::make_shared<glsl::PointCloudRendererForAnnotation>();
        if (!renderer_ptr->AddGeometry(geometry_ptr)) {
            return false;
        }
    } else {
        return false;
    }
    geometry_renderer_ptrs_.insert(renderer_ptr);
    geometry_ptrs_.insert(geometry_ptr);
    if (reset_bounding_box) {
        view_control_ptr_->FitInGeometry(*geometry_ptr);
        ResetViewPoint();
    }
    utility::LogDebug("Add geometry and update bounding box to {}",
                      view_control_ptr_->GetBoundingBox().GetPrintInfo().c_str());
    return UpdateGeometry(geometry_ptr);
}

}  // namespace visualization
}  // namespace open3d
