// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "pc_annotation/VisualizerForAnnotation.h"

namespace open3d {
namespace visualization {

void VisualizerForAnnotation::WindowRefreshCallback(GLFWwindow *window) {
    if (is_redraw_required_) {
        Render();
        is_redraw_required_ = false;
    }
}

void VisualizerForAnnotation::KeyPressCallback(
        GLFWwindow *window, int key, int scancode, int action, int mods) {
    if (action == GLFW_RELEASE) {
        return;
    }

    switch (key) {
        case GLFW_KEY_ENTER:
            if (mods & GLFW_MOD_ALT) {
                if (IsFullScreen()) {
                    SetFullScreen(false);
                } else {
                    SetFullScreen(true);
                };
            }
            break;
        case GLFW_KEY_LEFT_BRACKET:
            view_control_ptr_->ChangeFieldOfView(-1.0);
            utility::LogDebug("[VisualizerForAnnotation] Field of view set to {:.2f}.",
                              view_control_ptr_->GetFieldOfView());
            break;
        case GLFW_KEY_RIGHT_BRACKET:
            view_control_ptr_->ChangeFieldOfView(1.0);
            utility::LogDebug("[VisualizerForAnnotation] Field of view set to {:.2f}.",
                              view_control_ptr_->GetFieldOfView());
            break;
        case GLFW_KEY_R:
            ResetViewPoint();
            utility::LogDebug("[VisualizerForAnnotation] Reset view point.");
            break;
        case GLFW_KEY_C:
            if (mods & GLFW_MOD_CONTROL || mods & GLFW_MOD_SUPER) {
                CopyViewStatusToClipboard();
            }
            break;
        case GLFW_KEY_V:
            if (mods & GLFW_MOD_CONTROL || mods & GLFW_MOD_SUPER) {
                CopyViewStatusFromClipboard();
            }
            break;
        case GLFW_KEY_ESCAPE:
        case GLFW_KEY_Q:
            Close();
            break;
        case GLFW_KEY_H:
            PrintVisualizerHelp();
            break;
        case GLFW_KEY_P:
        case GLFW_KEY_PRINT_SCREEN:
            CaptureScreenImage();
            break;
        case GLFW_KEY_D:
            CaptureDepthImage();
            break;
        case GLFW_KEY_O:
            CaptureRenderOption();
            break;
        case GLFW_KEY_L:
            render_option_ptr_->ToggleLightOn();
            utility::LogDebug("[VisualizerForAnnotation] Lighting {}.",
                              render_option_ptr_->light_on_ ? "ON" : "OFF");
            break;
        case GLFW_KEY_EQUAL:
            if (mods & GLFW_MOD_SHIFT) {
                render_option_ptr_->ChangeLineWidth(1.0);
                utility::LogDebug("[VisualizerForAnnotation] Line width set to {:.2f}.",
                                  render_option_ptr_->line_width_);
            } else {
                render_option_ptr_->ChangePointSize(1.0);
                if (render_option_ptr_->point_show_normal_) {
                    UpdateGeometry();
                }
                utility::LogDebug("[VisualizerForAnnotation] Point size set to {:.2f}.",
                                  render_option_ptr_->point_size_);
            }
            break;
        case GLFW_KEY_MINUS:
            if (mods & GLFW_MOD_SHIFT) {
                render_option_ptr_->ChangeLineWidth(-1.0);
                utility::LogDebug("[VisualizerForAnnotation] Line width set to {:.2f}.",
                                  render_option_ptr_->line_width_);
            } else {
                render_option_ptr_->ChangePointSize(-1.0);
                if (render_option_ptr_->point_show_normal_) {
                    UpdateGeometry();
                }
                utility::LogDebug("[VisualizerForAnnotation] Point size set to {:.2f}.",
                                  render_option_ptr_->point_size_);
            }
            break;
        case GLFW_KEY_N:
            render_option_ptr_->TogglePointShowNormal();
            if (render_option_ptr_->point_show_normal_) {
                UpdateGeometry();
            }
            utility::LogDebug(
                    "[VisualizerForAnnotation] Point normal rendering {}.",
                    render_option_ptr_->point_show_normal_ ? "ON" : "OFF");
            break;
        case GLFW_KEY_S:
            render_option_ptr_->ToggleShadingOption();
            UpdateGeometry();
            utility::LogDebug(
                    "[VisualizerForAnnotation] Mesh shading mode is {}.",
                    render_option_ptr_->mesh_shade_option_ ==
                                    RenderOptionForAnnotation::MeshShadeOption::FlatShade
                            ? "FLAT"
                            : "SMOOTH");
            break;
        case GLFW_KEY_W:
            render_option_ptr_->ToggleMeshShowWireframe();
            utility::LogDebug(
                    "[VisualizerForAnnotation] Mesh wireframe rendering {}.",
                    render_option_ptr_->mesh_show_wireframe_ ? "ON" : "OFF");
            break;
        case GLFW_KEY_B:
            render_option_ptr_->ToggleMeshShowBackFace();
            utility::LogDebug(
                    "[VisualizerForAnnotation] Mesh back face rendering {}.",
                    render_option_ptr_->mesh_show_back_face_ ? "ON" : "OFF");
            break;
        case GLFW_KEY_I:
            render_option_ptr_->ToggleInterpolationOption();
            UpdateGeometry();
            utility::LogDebug(
                    "[VisualizerForAnnotation] geometry::Image interpolation mode is {}.",
                    render_option_ptr_->interpolation_option_ ==
                                    RenderOptionForAnnotation::TextureInterpolationOption::
                                            Nearest
                            ? "NEARST"
                            : "LINEAR");
            break;
        case GLFW_KEY_T:
            render_option_ptr_->ToggleImageStretchOption();
            utility::LogDebug(
                    "[VisualizerForAnnotation] geometry::Image stretch mode is #{}.",
                    int(render_option_ptr_->image_stretch_option_));
            break;
        case GLFW_KEY_0:
            if (mods & GLFW_MOD_CONTROL) {
                render_option_ptr_->mesh_color_option_ =
                        RenderOptionForAnnotation::MeshColorOption::Default;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Mesh color set to DEFAULT.");
            } else if (mods & GLFW_MOD_SHIFT) {
                SetGlobalColorMap(ColorMap::ColorMapOption::Gray);
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Color map set to GRAY.");
            } else {
                render_option_ptr_->point_color_option_ =
                        RenderOptionForAnnotation::PointColorOption::Default;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Point color set to DEFAULT.");
            }
            break;
        case GLFW_KEY_1:
            if (mods & GLFW_MOD_CONTROL) {
                render_option_ptr_->mesh_color_option_ =
                        RenderOptionForAnnotation::MeshColorOption::Color;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Mesh color set to COLOR.");
            } else if (mods & GLFW_MOD_SHIFT) {
                SetGlobalColorMap(ColorMap::ColorMapOption::Jet);
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Color map set to JET.");
            } else {
                render_option_ptr_->point_color_option_ =
                        RenderOptionForAnnotation::PointColorOption::Color;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Point color set to COLOR.");
            }
            break;
        case GLFW_KEY_2:
            if (mods & GLFW_MOD_CONTROL) {
                render_option_ptr_->mesh_color_option_ =
                        RenderOptionForAnnotation::MeshColorOption::XCoordinate;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Mesh color set to X.");
            } else if (mods & GLFW_MOD_SHIFT) {
                SetGlobalColorMap(ColorMap::ColorMapOption::Summer);
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Color map set to SUMMER.");
            } else {
                render_option_ptr_->point_color_option_ =
                        RenderOptionForAnnotation::PointColorOption::XCoordinate;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Point color set to X.");
            }
            break;
        case GLFW_KEY_3:
            if (mods & GLFW_MOD_CONTROL) {
                render_option_ptr_->mesh_color_option_ =
                        RenderOptionForAnnotation::MeshColorOption::YCoordinate;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Mesh color set to Y.");
            } else if (mods & GLFW_MOD_SHIFT) {
                SetGlobalColorMap(ColorMap::ColorMapOption::Winter);
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Color map set to WINTER.");
            } else {
                render_option_ptr_->point_color_option_ =
                        RenderOptionForAnnotation::PointColorOption::YCoordinate;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Point color set to Y.");
            }
            break;
        case GLFW_KEY_4:
            if (mods & GLFW_MOD_CONTROL) {
                render_option_ptr_->mesh_color_option_ =
                        RenderOptionForAnnotation::MeshColorOption::ZCoordinate;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Mesh color set to Z.");
            } else if (mods & GLFW_MOD_SHIFT) {
                SetGlobalColorMap(ColorMap::ColorMapOption::Hot);
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Color map set to HOT.");
            } else {
                render_option_ptr_->point_color_option_ =
                        RenderOptionForAnnotation::PointColorOption::ZCoordinate;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Point color set to Z.");
            }
            break;
        case GLFW_KEY_5:
            render_option_ptr_->point_color_option_ =
                    RenderOptionForAnnotation::PointColorOption::Label;
            UpdateGeometry();
            utility::LogDebug("[VisualizerForAnnotation] Point color set to LABEL");
            break;
        case GLFW_KEY_9:
            if (mods & GLFW_MOD_CONTROL) {
                render_option_ptr_->mesh_color_option_ =
                        RenderOptionForAnnotation::MeshColorOption::Normal;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Mesh color set to NORMAL.");
            } else if (mods & GLFW_MOD_SHIFT) {
            } else {
                render_option_ptr_->point_color_option_ =
                        RenderOptionForAnnotation::PointColorOption::Normal;
                UpdateGeometry();
                utility::LogDebug("[VisualizerForAnnotation] Point color set to NORMAL.");
            }
            break;
        default:
            break;
    }

    is_redraw_required_ = true;
}

}  // namespace visualization
}  // namespace open3d
