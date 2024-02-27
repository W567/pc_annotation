// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "pc_annotation/VisualizerWithAnnotation.h"

// #include <tinyfiledialogs/tinyfiledialogs.h>

#include "open3d/geometry/HalfEdgeTriangleMesh.h"
#include "open3d/geometry/Image.h"
#include "open3d/geometry/LineSet.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/TetraMesh.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/io/IJsonConvertibleIO.h"
#include "open3d/io/PointCloudIO.h"
#include "open3d/io/TriangleMeshIO.h"
#include "open3d/utility/FileSystem.h"
#include "open3d/visualization/utility/GLHelper.h"
#include "open3d/visualization/utility/PointCloudPicker.h"
#include "open3d/visualization/utility/SelectionPolygon.h"
#include "open3d/visualization/utility/SelectionPolygonVolume.h"
#include "open3d/visualization/visualizer/RenderOptionWithEditing.h"
#include "open3d/visualization/visualizer/ViewControlWithEditing.h"

#include <fstream>
#include  <experimental/filesystem>

namespace open3d {
namespace visualization {

namespace {
static const double POINT_SIZE = 9.0;
static const double MIN_POINT_SIZE = 3.0;
static const Eigen::Vector3d CHOOSE_POINTS_COLOR(1, 0, 1);
static const Eigen::Vector3d SELECTED_POINTS_COLOR(0, 1, 0);
static const int START_RECT_DIST = 3;

bool BindFramebuffer(int width, int height) {
    GLuint frame_buffer_name = 0;
    glGenFramebuffers(1, &frame_buffer_name);
    glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_name);
    GLuint fbo_texture;
    glGenTextures(1, &fbo_texture);
    glBindTexture(GL_TEXTURE_2D, fbo_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (!GLEW_ARB_framebuffer_object) {
        // OpenGL 2.1 doesn't require this, 3.1+ does
        utility::LogWarning(
                "[BindFramebuffwer] Your GPU does not provide framebuffer "
                "objects. "
                "Use a texture instead.");
        return false;
    }
    GLuint depth_render_buffer;
    glGenRenderbuffers(1, &depth_render_buffer);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_render_buffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, depth_render_buffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           fbo_texture, 0);
    GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, DrawBuffers);  // "1" is the size of DrawBuffers
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        utility::LogWarning("[BindFramebuffer] Something is wrong with FBO.");
        return false;
    }
    return true;
}

}  // namespace

bool VisualizerWithAnnotation::AddGeometry(
        std::shared_ptr<const geometry::Geometry> geometry_in_ptr,
        bool reset_bounding_box) {
    if (!is_initialized_ || !geometry_ptrs_.empty()) {
        utility::LogInfo(
                "VisualizerWithAnnotation only supports one geometry");
        return false;
    }
    glfwMakeContextCurrent(window_);

    // Add the geometry/renderer
    geometry_ptr_ = geometry_in_ptr;
    switch (geometry_ptr_->GetGeometryType()) {
        case geometry::Geometry::GeometryType::PointCloud:
            geometry_renderer_ptr_ =
                    std::make_shared<glsl::PointCloudRenderer>();
            break;
        case geometry::Geometry::GeometryType::LineSet:
            geometry_renderer_ptr_ = std::make_shared<glsl::LineSetRenderer>();
            break;
        case geometry::Geometry::GeometryType::TriangleMesh:  // fall-through
        case geometry::Geometry::GeometryType::HalfEdgeTriangleMesh:
            geometry_renderer_ptr_ =
                    std::make_shared<glsl::TriangleMeshRenderer>();
            break;
        case geometry::Geometry::GeometryType::TetraMesh:
            geometry_renderer_ptr_ =
                    std::make_shared<glsl::TetraMeshRenderer>();
            break;
        case geometry::Geometry::GeometryType::Image:
            geometry_renderer_ptr_ = std::make_shared<glsl::ImageRenderer>();
            break;
        case geometry::Geometry::GeometryType::MeshBase:
            // MeshBase is too general, can't render. Fall-through.
        case geometry::Geometry::GeometryType::RGBDImage:
        case geometry::Geometry::GeometryType::VoxelGrid:
        case geometry::Geometry::GeometryType::Octree:
        case geometry::Geometry::GeometryType::OrientedBoundingBox:
        case geometry::Geometry::GeometryType::AxisAlignedBoundingBox:
        case geometry::Geometry::GeometryType::Unspecified:
            return false;
    }

    if (!geometry_renderer_ptr_->AddGeometry(geometry_ptr_)) {
        return false;
    }
    geometry_renderer_ptr_->AddLabels(labels);
    
    geometry_ptrs_.insert(geometry_ptr_);
    geometry_renderer_ptrs_.insert(geometry_renderer_ptr_);

    // Add the point selection renderers
    ui_points_geometry_ptr_ = std::make_shared<geometry::PointCloud>();
    ui_points_renderer_ptr_ = std::make_shared<glsl::PointCloudRenderer>();
    ui_points_renderer_ptr_->AddGeometry(ui_points_geometry_ptr_);
    ui_selected_points_geometry_ptr_ = std::make_shared<geometry::PointCloud>();
    ui_selected_points_renderer_ptr_ =
            std::make_shared<glsl::PointCloudRenderer>();
    ui_selected_points_renderer_ptr_->AddGeometry(
            ui_selected_points_geometry_ptr_);
    utility_renderer_ptrs_.push_back(ui_selected_points_renderer_ptr_);

    utility_renderer_opts_[ui_points_renderer_ptr_].depthFunc_ =
            RenderOption::DepthFunc::Less;
    utility_renderer_opts_[ui_selected_points_renderer_ptr_].depthFunc_ =
            RenderOption::DepthFunc::LEqual;
    SetPointSize(POINT_SIZE);

    if (reset_bounding_box) {
        ResetViewPoint(true);
    }
    utility::LogDebug(
            "Add geometry and update bounding box to {}",
            view_control_ptr_->GetBoundingBox().GetPrintInfo().c_str());
    return UpdateGeometry();
}

bool VisualizerWithAnnotation::UpdateGeometry(
        std::shared_ptr<const geometry::Geometry> geometry_ptr /*= nullptr*/) {
    if (geometry_ptr) {
        utility::LogDebug(
                "VisualizerWithAnnotation::UpdateGeometry() does not "
                "support "
                "passing a new geometry. However, you may update the geometry "
                "you"
                "passed to AddGeometry() and call UpdateGeometry().");
        return false;
    }
    geometry_ptr = geometry_ptr_;

    bool result = Visualizer::UpdateGeometry(geometry_ptr);

    switch (geometry_ptr_->GetGeometryType()) {
        case geometry::Geometry::GeometryType::PointCloud: {
            auto cloud = std::static_pointer_cast<const geometry::PointCloud>(
                    geometry_ptr_);
            const std::vector<Eigen::Vector3d> *geometry_points =
                    GetGeometryPoints(ui_points_geometry_ptr_);
            if (geometry_points &&
                cloud->points_.size() != geometry_points->size()) {
                ClearPickedPoints();
            }
            ui_points_geometry_ptr_->points_ = cloud->points_;
            ui_points_geometry_ptr_->normals_ = cloud->normals_;
            break;
        }
        case geometry::Geometry::GeometryType::LineSet: {
            auto lines = std::static_pointer_cast<const geometry::LineSet>(
                    geometry_ptr_);
            const std::vector<Eigen::Vector3d> *geometry_points =
                    GetGeometryPoints(ui_points_geometry_ptr_);
            if (geometry_points &&
                lines->points_.size() != geometry_points->size()) {
                ClearPickedPoints();
            }
            ui_points_geometry_ptr_->points_ = lines->points_;
            break;
        }
        case geometry::Geometry::GeometryType::MeshBase:
        case geometry::Geometry::GeometryType::TriangleMesh:
        case geometry::Geometry::GeometryType::HalfEdgeTriangleMesh:
        case geometry::Geometry::GeometryType::TetraMesh: {
            auto mesh = std::static_pointer_cast<const geometry::MeshBase>(
                    geometry_ptr_);
            const std::vector<Eigen::Vector3d> *geometry_points =
                    GetGeometryPoints(ui_points_geometry_ptr_);
            if (geometry_points &&
                mesh->vertices_.size() != geometry_points->size()) {
                ClearPickedPoints();
            }
            ui_points_geometry_ptr_->points_ = mesh->vertices_;
            ui_points_geometry_ptr_->normals_ = mesh->vertex_normals_;
            break;
        }
        case geometry::Geometry::GeometryType::Image:
        case geometry::Geometry::GeometryType::RGBDImage:
        case geometry::Geometry::GeometryType::VoxelGrid:
        case geometry::Geometry::GeometryType::Octree:
        case geometry::Geometry::GeometryType::OrientedBoundingBox:
        case geometry::Geometry::GeometryType::AxisAlignedBoundingBox:
        case geometry::Geometry::GeometryType::Unspecified:
            break;
    }

    ui_points_geometry_ptr_->PaintUniformColor(CHOOSE_POINTS_COLOR);
    ui_points_renderer_ptr_->UpdateGeometry();

    geometry_renderer_ptr_->UpdateGeometry();

    return result;
}

void VisualizerWithAnnotation::PrintVisualizerHelp() {
    Visualizer::PrintVisualizerHelp();
    // clang-format off
    utility::LogInfo("  -- Editing control --");
    utility::LogInfo("    F            : Enter free view.");
    utility::LogInfo("    X            : Enter orthogonal view along X axis, press again to flip.");
    utility::LogInfo("    Y            : Enter orthogonal view along Y axis, press again to flip.");
    utility::LogInfo("    Z            : Enter orthogonal view along Z axis, press again to flip.");
    utility::LogInfo("    Ctrl + R     : Clear selection.");
    utility::LogInfo("    Shift + +/-  : Increase/decrease picked point size.");
    utility::LogInfo("    Shift + mouse left button   : Pick a point and add to selection. If it is");
    utility::LogInfo("                                  already in the selection it will be removed.");
    utility::LogInfo("    Shift + mouse left drag     : Defines a rectangle, which will add all the ");
    utility::LogInfo("                                  points in it to the selection.");
    utility::LogInfo("    mouse right drag            : Moves selected points.");
    utility::LogInfo("    Delete / Backspace          : Removes all points in the rectangle from the");
    utility::LogInfo("                                  selection.");
    utility::LogInfo("    Space        :        Annotate with current tag.");
    utility::LogInfo("    Ctrl + N     :        Skip to next tag.");
    utility::LogInfo("    Ctrl + B     :        Back to previous tag.");
    utility::LogInfo("    Ctrl + S     :        Save labels.");
    utility::LogInfo("    Ctrl + F     :        Toggle coordinate visualization.");
    utility::LogInfo("    Q / Esc      :        Quit (auto labels saving).");
    utility::LogInfo("");
    // clang-format on
}

void VisualizerWithAnnotation::UpdateWindowTitle() {
    if (window_ != NULL) {
        auto &view_control = (ViewControlWithEditing &)(*view_control_ptr_);
        auto title = window_name_ + " - " + view_control.GetStatusString() + " - Current Tag:" + std::to_string(tag);
        glfwSetWindowTitle(window_, title.c_str());
    }
}

void VisualizerWithAnnotation::BuildUtilities() {
    Visualizer::BuildUtilities();
    bool success;

    // 1. Build selection polygon
    success = true;
    selection_polygon_ptr_ = std::make_shared<SelectionPolygon>();
    selection_polygon_renderer_ptr_ =
            std::make_shared<glsl::SelectionPolygonRenderer>();
    if (!selection_polygon_renderer_ptr_->AddGeometry(selection_polygon_ptr_)) {
        success = false;
    }
    if (success) {
        utility_ptrs_.push_back(selection_polygon_ptr_);
        utility_renderer_ptrs_.push_back(selection_polygon_renderer_ptr_);
    }

    // 2. Build pointcloud picker
    success = true;
    pointcloud_picker_ptr_ = std::make_shared<PointCloudPicker>();
    if (geometry_ptrs_.empty() ||
        !pointcloud_picker_ptr_->SetPointCloud(geometry_ptr_)) {
        success = false;
    }
    pointcloud_picker_renderer_ptr_ =
            std::make_shared<glsl::PointCloudPickerRenderer>();
    if (!pointcloud_picker_renderer_ptr_->AddGeometry(pointcloud_picker_ptr_)) {
        success = false;
    }
    if (success) {
        utility_ptrs_.push_back(pointcloud_picker_ptr_);
        utility_renderer_ptrs_.push_back(pointcloud_picker_renderer_ptr_);
    }
}

float VisualizerWithAnnotation::GetDepth(int winX, int winY) {
    const auto &view = GetViewControl();

    // Render to FBO
    if (!BindFramebuffer(view.GetWindowWidth(), view.GetWindowHeight())) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return {};
    }

    view_control_ptr_->SetViewMatrices();
    // We only need the depth information, so reduce time rendering colors
    glDisable(GL_MULTISAMPLE);
    glDisable(GL_BLEND);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for (auto &renderer : geometry_renderer_ptrs_) {
        renderer->Render(GetRenderOption(), GetViewControl());
    }
    glFinish();

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glEnable(GL_MULTISAMPLE);

    // glReadPixels uses GL coordinates: (x, y) is lower left and +y is up
    int width = 1;
    int height = 1;
    int lowerLeftX = int(winX + 0.5);
    int lowerLeftY = int(view.GetWindowHeight() - winY - height + 0.5);

    float depth;
    glReadPixels(lowerLeftX, lowerLeftY, width, height, GL_DEPTH_COMPONENT,
                 GL_FLOAT, &depth);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return depth;
}

std::vector<int> VisualizerWithAnnotation::PickPoints(double winX,
                                                           double winY,
                                                           double w,
                                                           double h) {
    points_in_rect_.clear();

    auto renderer_ptr = std::make_shared<glsl::PointCloudPickingRenderer>();
    if (!renderer_ptr->AddGeometry(ui_points_geometry_ptr_)) {
        return {};
    }
    const auto &view = GetViewControl();
    // Render to FBO
    if (!BindFramebuffer(view.GetWindowWidth(), view.GetWindowHeight())) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return {};
    }

    view_control_ptr_->SetViewMatrices();
    glDisable(GL_MULTISAMPLE);  // we need pixelation for correct pick colors
    glDisable(GL_BLEND);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render any triangle meshes to the depth buffer only (so that z-buffer
    // prevents points that are behind them being drawn for selection)
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    for (auto &renderer : geometry_renderer_ptrs_) {
        if (renderer->GetGeometry()->GetGeometryType() ==
            geometry::Geometry::GeometryType::TriangleMesh) {
            renderer->Render(GetRenderOption(), GetViewControl());
        }
    }
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Now render the points
    renderer_ptr->Render(pick_point_opts_, GetViewControl());

    glFinish();

    // glReadPixels uses GL coordinates: (x, y) is lower left and +y is up
    int width = int(std::ceil(w));
    int height = int(std::ceil(h));
    int lowerLeftX = int(winX + 0.5);
    int lowerLeftY = int(view.GetWindowHeight() - winY - height + 0.5);
    std::vector<uint8_t> rgba(4 * width * height, 0);

    glReadPixels(lowerLeftX, lowerLeftY, width, height, GL_RGBA,
                 GL_UNSIGNED_BYTE, rgba.data());
    // Recover rendering state
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glEnable(GL_MULTISAMPLE);

    std::unordered_set<int> indexSet;
    for (int i = 0; i < width * height; ++i) {
        const uint8_t *rgbaPtr = rgba.data() + 4 * i;
        int index = gl_util::ColorCodeToPickIndex(Eigen::Vector4i(
                rgbaPtr[0], rgbaPtr[1], rgbaPtr[2], rgbaPtr[3]));
        if (index >= 0) {
            indexSet.insert(index);
        }
    }

    std::vector<int> indices;
    indices.reserve(indexSet.size());
    for (int idx : indexSet) {
        indices.push_back(idx);
    }
    points_in_rect_ = indices;
    return indices;
}

std::vector<VisualizerWithAnnotation::PickedPoint>
VisualizerWithAnnotation::GetPickedPoints() const {
    std::vector<PickedPoint> points;
    points.reserve(selected_points_.size());
    for (auto &kv : selected_points_) {
        points.push_back({kv.first, kv.second});
    }
    return points;
}

bool VisualizerWithAnnotation::InitViewControl() {
    view_control_ptr_ =
            std::unique_ptr<ViewControlWithEditing>(new ViewControlWithEditing);
    ResetViewPoint();
    return true;
}

// TODO: remove this function?
// bool VisualizerWithAnnotation::InitRenderOption() {
//     render_option_ptr_ = std::unique_ptr<RenderOptionWithEditing>(
//             new RenderOptionWithEditing);
//     return true;
// }

void VisualizerWithAnnotation::RegisterSelectionChangedCallback(
        std::function<void()> f) {
    on_selection_changed_ = f;
}

void VisualizerWithAnnotation::RegisterSelectionMovingCallback(
        std::function<void()> f) {
    on_selection_moving_ = f;
}

void VisualizerWithAnnotation::RegisterSelectionMovedCallback(
        std::function<void()> f) {
    on_selection_moved_ = f;
}

void VisualizerWithAnnotation::WindowResizeCallback(GLFWwindow *window,
                                                         int w,
                                                         int h) {
    InvalidateSelectionPolygon();
    Visualizer::WindowResizeCallback(window, w, h);
}

void VisualizerWithAnnotation::SetFilename(std::string name)
{
    filename = name;
}

bool VisualizerWithAnnotation::ReadTag()
{
    // std::fstream file;
    // std::string tmp = filename;
    // tmp.pop_back();
    // tmp.pop_back();
    // tmp.pop_back();
    // std::string ext = "label";
    // if (std::experimental::filesystem::exists(tmp + ext)) {
    //     file.open(tmp + ext);
    //     std::string line;
    //     labels.clear();
    //     std::vector<int> aff1;
    //     std::vector<int> aff2;
    //     std::vector<int> aff3;
    //     while (std::getline(file, line))
    //     {
    //         aff1.push_back(atoi(&line.at(0)));
    //         aff2.push_back(atoi(&line.at(2)));
    //         aff3.push_back(atoi(&line.at(4)));
    //         utility::LogInfo(line.c_str());
    //         utility::LogInfo("tag1, 2, 3: #{:d} #{:d} #{:d}.", atoi(&line.at(0)),
    //                                                            atoi(&line.at(2)),
    //                                                            atoi(&line.at(4)));
    //     }
    //     labels.push_back(aff1);
    //     labels.push_back(aff2);
    //     labels.push_back(aff3);
    //     return true;
    // }
    // else {
    //     return false;
    // }
    return false;
}

void VisualizerWithAnnotation::SaveTag()
{
    std::ofstream file;
    std::string tmp = filename;
    tmp.pop_back();
    tmp.pop_back();
    tmp.pop_back();
    std::string ext = "label";
    file.open(tmp + ext);

    int size = labels.size();
    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < size; j++)
        {
            file << std::to_string(labels[j][i]);
            if (j < size - 1)
            {
                file << " ";
            }
            else
            {
                file << "\n";
            }
        }
    }

    file.close();
    utility::LogInfo("Tag saved.");
}

void VisualizerWithAnnotation::KeyPressCallback(
        GLFWwindow *window, int key, int scancode, int action, int mods) {
    auto &view_control = (ViewControlWithEditing &)(*view_control_ptr_);
    if (key == GLFW_KEY_LEFT_SHIFT || key == GLFW_KEY_RIGHT_SHIFT) {
        if (action == GLFW_PRESS) {
            if (ui_points_renderer_ptr_) {
                utility_renderer_ptrs_.push_back(ui_points_renderer_ptr_);
            }
            is_redraw_required_ = true;
        } else if (action == GLFW_RELEASE) {
            InvalidateSelectionPolygon();
            for (size_t i = 0; i < utility_renderer_ptrs_.size();) {
                if (utility_renderer_ptrs_[i] == ui_points_renderer_ptr_) {
                    utility_renderer_ptrs_.erase(
                            utility_renderer_ptrs_.begin() + i);
                    is_redraw_required_ = true;
                } else {
                    ++i;
                }
            }
        }
    } else if ((key == GLFW_KEY_DELETE || key == GLFW_KEY_BACKSPACE) &&
               action == GLFW_RELEASE) {
        RemovePickedPoints(points_in_rect_);
        InvalidateSelectionPolygon();
        is_redraw_required_ = true;
    }

    if (action != GLFW_RELEASE) {
        switch (key) {
            case GLFW_KEY_X:
                view_control.ToggleEditingX();
                utility::LogDebug(
                        "[Visualizer] Enter orthogonal X editing mode.");
                break;
            case GLFW_KEY_Y:
                view_control.ToggleEditingY();
                utility::LogDebug(
                        "[Visualizer] Enter orthogonal Y editing mode.");
                break;
            case GLFW_KEY_Z:
                view_control.ToggleEditingZ();
                utility::LogInfo(
                        "[Visualizer] Enter orthogonal Z editing mode.");
                break;
            case GLFW_KEY_R:
                if (mods & GLFW_MOD_CONTROL) {
                    ClearPickedPoints();

                    is_redraw_required_ = true;
                } else {
                    ReadTag();
                    // Visualizer::KeyPressCallback(window, key, scancode, action,
                    //                              mods);
                }
                break;
            case GLFW_KEY_SPACE:
                if (selected_points_.size() > 0)
                {
                    auto points = GetGeometryPoints(geometry_ptr_);
                    length = points->size();
                    std::vector<int> tmp(length);
                    for (auto &kv : selected_points_)
                    {
                        tmp[kv.first] = 1;
                    }
                    labels.push_back(tmp);
                    utility::LogInfo(
                        "Annotate #{:d} points with tag #{:d}. Current tag: #{:d}",
                        selected_points_.size(), tag, tag+1);
                    tag++;
                }
                else
                {
                    utility::LogInfo("Please select points before annotation.");
                }
                break;

            case GLFW_KEY_N: {
                if (mods & GLFW_MOD_CONTROL) {
                    auto points = GetGeometryPoints(geometry_ptr_);
                    length = points->size();
                    std::vector<int> tmp(length);
                    labels.push_back(tmp);
                    tag++;
                    utility::LogInfo("Skip. Current tag #{:d}.", tag);
                } else {
                    Visualizer::KeyPressCallback(window, key, scancode, action,
                                                 mods);
                }
                break;
            }
            
            case GLFW_KEY_B: {
                if (mods & GLFW_MOD_CONTROL) {
                    if (tag > 0)
                    {
                        tag--;
                        labels.pop_back();
                        utility::LogInfo("Back. Current tag: #{:d}.", tag);
                    }
                    else
                    {
                        utility::LogInfo("Empty. Current tag: #{:d}.", tag);
                    }
                } else {
                    Visualizer::KeyPressCallback(window, key, scancode, action,
                                                 mods);
                }
                break;
            }

            case GLFW_KEY_S: {
                if (mods & GLFW_MOD_CONTROL) {
                    SaveTag();
                } else {
                    Visualizer::KeyPressCallback(window, key, scancode, action,
                                                 mods);
                }
                break;
            }

            case GLFW_KEY_F: {
                if (mods & GLFW_MOD_CONTROL) {
                  render_option_ptr_->show_coordinate_frame_ = !render_option_ptr_->show_coordinate_frame_;
                } else {
                    view_control.SetEditingMode(
                            ViewControlWithEditing::EditingMode::FreeMode);
                    utility::LogDebug("[Visualizer] Enter freeview mode.");
                }
                break;
            }

            case GLFW_KEY_ESCAPE:
            case GLFW_KEY_Q: {
                SaveTag();
                Close();
                break;
            }

            case GLFW_KEY_MINUS: {
                if (action == GLFW_PRESS) {
                    SetPointSize(pick_point_opts_.point_size_ - 1.0);
                    is_redraw_required_ = true;
                } else {
                    Visualizer::KeyPressCallback(window, key, scancode, action,
                                                 mods);
                }
                break;
            }
            case GLFW_KEY_EQUAL: {
                if (action == GLFW_PRESS) {
                    SetPointSize(pick_point_opts_.point_size_ + 1.0);
                    is_redraw_required_ = true;
                } else {
                    Visualizer::KeyPressCallback(window, key, scancode, action,
                                                 mods);
                }
                break;
            }
            default:
                Visualizer::KeyPressCallback(window, key, scancode, action,
                                             mods);
                break;
        }
    }
    is_redraw_required_ = true;
    UpdateWindowTitle();
}

void VisualizerWithAnnotation::MouseMoveCallback(GLFWwindow *window,
                                                      double x,
                                                      double y) {
    auto &view_control = (ViewControlWithEditing &)(*view_control_ptr_);
    if (selection_mode_ != SelectionMode::None) {
#ifdef __APPLE__
        x /= pixel_to_screen_coordinate_;
        y /= pixel_to_screen_coordinate_;
#endif
        double y_inv = view_control.GetWindowHeight() - y;
        if (selection_mode_ == SelectionMode::Point &&
            std::abs(x - mouse_down_pos_.x()) > START_RECT_DIST &&
            std::abs(y - mouse_down_pos_.y()) > START_RECT_DIST) {
            InvalidateSelectionPolygon();
            selection_mode_ = SelectionMode::Rectangle;
            selection_polygon_ptr_->is_closed_ = true;
            Eigen::Vector2d pt(x, y_inv);
            selection_polygon_ptr_->polygon_.push_back(pt);
            selection_polygon_ptr_->polygon_.push_back(pt);
            selection_polygon_ptr_->polygon_.push_back(pt);
            selection_polygon_ptr_->polygon_.push_back(pt);
            selection_polygon_renderer_ptr_->UpdateGeometry();
        } else if (selection_mode_ == SelectionMode::Rectangle) {
            selection_polygon_ptr_->polygon_[1](0) = x;
            selection_polygon_ptr_->polygon_[2](0) = x;
            selection_polygon_ptr_->polygon_[2](1) = y_inv;
            selection_polygon_ptr_->polygon_[3](1) = y_inv;
            selection_polygon_renderer_ptr_->UpdateGeometry();
        } else if (selection_mode_ == SelectionMode::Moving) {
            DragSelectedPoints(CalcDragDelta(x, y), DRAG_MOVING);
        }
        is_redraw_required_ = true;
    } else {
        Visualizer::MouseMoveCallback(window, x, y);
    }
}

void VisualizerWithAnnotation::MouseScrollCallback(GLFWwindow *window,
                                                        double x,
                                                        double y) {
    Visualizer::MouseScrollCallback(window, x, y);
}

void VisualizerWithAnnotation::MouseButtonCallback(GLFWwindow *window,
                                                        int button,
                                                        int action,
                                                        int mods) {
    double x, y;
    glfwGetCursorPos(window, &x, &y);
#ifdef __APPLE__
    x /= pixel_to_screen_coordinate_;
    y /= pixel_to_screen_coordinate_;
#endif

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        if (mods & GLFW_MOD_SHIFT) {
            selection_mode_ = SelectionMode::Point;
            mouse_down_pos_ = {x, y};
        } else {
            Visualizer::MouseButtonCallback(window, button, action, mods);
        }
    } else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
        if (selection_mode_ == SelectionMode::Point) {
            auto indices = PickPoints(x, y, 1, 1);
            if (indices.empty()) {
                utility::LogInfo("No point has been picked.");
            } else {
                int index = indices[0];
                if (selected_points_.find(index) == selected_points_.end()) {
                    AddPickedPoints({index});
                } else {
                    RemovePickedPoints({index});
                }
            }
            is_redraw_required_ = true;
        } else if (selection_mode_ == SelectionMode::Rectangle) {
            auto &view_control = (ViewControlWithEditing &)(*view_control_ptr_);
            auto winHeight = view_control.GetWindowHeight();
            x = selection_polygon_ptr_->polygon_[0](0);
            y = winHeight - selection_polygon_ptr_->polygon_[0](1);
            double x2 = selection_polygon_ptr_->polygon_[2](0);
            double y2 = winHeight - selection_polygon_ptr_->polygon_[2](1);
            double w = x2 - x;
            double h = y2 - y;
            if (w < 0) {
                x += w;  // w is negative
                w = -w;
            }
            if (h < 0) {
                y += h;  // h is negative
                h = -h;
            }
            auto indices = PickPoints(x, y, w, h);
            if (indices.empty()) {
                utility::LogInfo("No points have been picked.");
            } else {
                AddPickedPoints(indices);
            }
            is_redraw_required_ = true;
        } else {
            Visualizer::MouseButtonCallback(window, button, action, mods);
        }
        if (selection_mode_ != SelectionMode::Rectangle) {
            InvalidateSelectionPolygon();
        }
        selection_mode_ = SelectionMode::None;
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
        mouse_down_pos_ = {x, y};
        selection_mode_ = SelectionMode::Moving;
        selected_points_before_drag_ = selected_points_;
        // The mouse moves on the viewing plane, but we want it to look like
        // we are moving the point we clicked on. One pixel on the viewing
        // plane is a larger distance than one pixel projected onto the
        // viewing plane because perspective shrinks things as they get
        // farther away.
        auto depth = GetDepth(static_cast<int>(x), static_cast<int>(y));
        // If we clicked on something, set the depth, otherwise keep it what
        // it was last time, which should be about right (as good as we're
        // going to get)
        if (depth < 1.0) {
            drag_depth_ = depth;
        }
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {
        DragSelectedPoints(CalcDragDelta(x, y), DRAG_END);
        selection_mode_ = SelectionMode::None;
        is_redraw_required_ = true;
    } else {
        Visualizer::MouseButtonCallback(window, button, action, mods);
    }
}

void VisualizerWithAnnotation::InvalidateSelectionPolygon() {
    points_in_rect_.clear();
    if (selection_polygon_ptr_) selection_polygon_ptr_->Clear();
    if (selection_polygon_renderer_ptr_) {
        selection_polygon_renderer_ptr_->UpdateGeometry();
    }
    selection_mode_ = SelectionMode::None;
}

void VisualizerWithAnnotation::InvalidatePicking() {
    if (pointcloud_picker_ptr_) pointcloud_picker_ptr_->Clear();
    if (pointcloud_picker_renderer_ptr_) {
        pointcloud_picker_renderer_ptr_->UpdateGeometry();
    }
}

void VisualizerWithAnnotation::ClearPickedPoints() {
    // utility::LogInfo("Clearing all points from selection.");
    selection_mode_ = SelectionMode::None;
    selected_points_.clear();
    selected_points_before_drag_.clear();
    if (ui_selected_points_geometry_ptr_) {
        ui_selected_points_geometry_ptr_->points_.clear();
        ui_selected_points_renderer_ptr_->UpdateGeometry();
    }

    if (on_selection_changed_) {
        on_selection_changed_();
    }
}

void VisualizerWithAnnotation::AddPickedPoints(
        const std::vector<int> indices) {
    auto points = GetGeometryPoints(geometry_ptr_);
    if (!points) {
        return;  // can't get points info, so can't add them
    }

    for (auto &index : indices) {
        const auto &point = (*points)[index];
        // utility::LogInfo(
        //         "Adding point #{:d} ({:.2f}, {:.2f}, {:.2f}) to selection.",
        //         index, point.x(), point.y(), point.z());
        selected_points_[index] = point;
        ui_selected_points_geometry_ptr_->points_.push_back(point);
    }
    utility::LogInfo("Adding #{:d} points to selection.", indices.size());
    ui_selected_points_geometry_ptr_->PaintUniformColor(SELECTED_POINTS_COLOR);
    ui_selected_points_renderer_ptr_->UpdateGeometry();

    if (on_selection_changed_) {
        on_selection_changed_();
    }
}

void VisualizerWithAnnotation::RemovePickedPoints(
        const std::vector<int> indices) {
    for (auto &index : indices) {
        utility::LogInfo("Removing point #{:d} from selection.", index);
        selected_points_.erase(index);
    }
    ui_selected_points_geometry_ptr_->points_.clear();
    for (auto &kv : selected_points_) {
        ui_selected_points_geometry_ptr_->points_.push_back(kv.second);
    }
    ui_selected_points_geometry_ptr_->PaintUniformColor(SELECTED_POINTS_COLOR);
    ui_selected_points_renderer_ptr_->UpdateGeometry();

    if (on_selection_changed_) {
        on_selection_changed_();
    }
}

void VisualizerWithAnnotation::DragSelectedPoints(
        const Eigen::Vector3d &delta, DragType type) {
    ui_selected_points_geometry_ptr_->points_.clear();
    for (auto &kv : selected_points_before_drag_) {
        auto index = kv.first;
        auto new_coord = kv.second + delta;
        selected_points_[index] = new_coord;
        ui_selected_points_geometry_ptr_->points_.push_back(new_coord);
    }
    ui_selected_points_geometry_ptr_->PaintUniformColor(SELECTED_POINTS_COLOR);
    ui_selected_points_renderer_ptr_->UpdateGeometry();

    if (type == DRAG_MOVING && on_selection_moving_) {
        on_selection_moving_();
    } else if (type == DRAG_END && on_selection_moved_) {
        on_selection_moved_();
    }
}

const std::vector<Eigen::Vector3d>
        *VisualizerWithAnnotation::GetGeometryPoints(
                std::shared_ptr<const geometry::Geometry> geometry) {
    const std::vector<Eigen::Vector3d> *points = nullptr;
    switch (geometry->GetGeometryType()) {
        case geometry::Geometry::GeometryType::PointCloud: {
            auto cloud = std::static_pointer_cast<const geometry::PointCloud>(
                    geometry);
            points = &cloud->points_;
            break;
        }
        case geometry::Geometry::GeometryType::LineSet: {
            auto lines =
                    std::static_pointer_cast<const geometry::LineSet>(geometry);
            points = &lines->points_;
            break;
        }
        case geometry::Geometry::GeometryType::MeshBase:
        case geometry::Geometry::GeometryType::TriangleMesh:
        case geometry::Geometry::GeometryType::HalfEdgeTriangleMesh:
        case geometry::Geometry::GeometryType::TetraMesh: {
            auto mesh = std::static_pointer_cast<const geometry::MeshBase>(
                    geometry);
            points = &mesh->vertices_;
            break;
        }
        case geometry::Geometry::GeometryType::Image:
        case geometry::Geometry::GeometryType::RGBDImage:
        case geometry::Geometry::GeometryType::VoxelGrid:
        case geometry::Geometry::GeometryType::Octree:
        case geometry::Geometry::GeometryType::OrientedBoundingBox:
        case geometry::Geometry::GeometryType::AxisAlignedBoundingBox:
        case geometry::Geometry::GeometryType::Unspecified:
            points = nullptr;
            break;
    }
    return points;
}

Eigen::Vector3d VisualizerWithAnnotation::CalcDragDelta(double winX,
                                                             double winY) {
    auto &view = (ViewControlWithEditing &)(*view_control_ptr_);
    auto start = gl_util::Unproject(
            Eigen::Vector3d(mouse_down_pos_.x(),
                            view.GetWindowHeight() - mouse_down_pos_.y(),
                            drag_depth_),
            view.GetMVPMatrix(), view.GetWindowWidth(), view.GetWindowHeight());
    auto end = gl_util::Unproject(
            Eigen::Vector3d(winX, view.GetWindowHeight() - winY, drag_depth_),
            view.GetMVPMatrix(), view.GetWindowWidth(), view.GetWindowHeight());
    return end - start;
}

void VisualizerWithAnnotation::SetPointSize(double size) {
    size = std::max(size, MIN_POINT_SIZE);
    pick_point_opts_.SetPointSize(size);
    auto *opt = &utility_renderer_opts_[ui_points_renderer_ptr_];
    opt->SetPointSize(size);
    opt = &utility_renderer_opts_[ui_selected_points_renderer_ptr_];
    opt->SetPointSize(size);
}

}  // namespace visualization
}  // namespace open3d
