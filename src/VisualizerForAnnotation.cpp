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

void VisualizerForAnnotation::Run() {
    BuildUtilities();
    UpdateWindowTitle();
    while (bool(animation_callback_func_) ? PollEvents() : WaitEvents()) {
        if (bool(animation_callback_func_in_loop_)) {
            if (animation_callback_func_in_loop_(this)) {
                UpdateGeometry();
            }
            // Set render flag as dirty anyways, because when we use callback
            // functions, we assume something has been changed in the callback
            // and the redraw event should be triggered.
            UpdateRender();
        }
    }
}

bool VisualizerForAnnotation::WaitEvents() {
    if (!is_initialized_) {
        return false;
    }
    glfwMakeContextCurrent(window_);
    if (is_redraw_required_) {
        WindowRefreshCallback(window_);
    }
    animation_callback_func_in_loop_ = animation_callback_func_;
    glfwWaitEvents();
    return !glfwWindowShouldClose(window_);
}

bool VisualizerForAnnotation::PollEvents() {
    if (!is_initialized_) {
        return false;
    }
    glfwMakeContextCurrent(window_);
    if (is_redraw_required_) {
        WindowRefreshCallback(window_);
    }
    animation_callback_func_in_loop_ = animation_callback_func_;
    glfwPollEvents();
    return !glfwWindowShouldClose(window_);
}


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


}  // namespace visualization
}  // namespace open3d
