// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------
// Reference:
// github.com/isl-org/Open3D/blob/main/cpp/open3d/visualization/visualizer/VisualizerWithVertexSelection.h

#pragma once

#include <unordered_map>
#include <Eigen/Core>

#include "pc_annotation/VisualizerForAnnotation.h"
#include "pc_annotation/RenderOptionForAnnotation.h"
#include "pc_annotation/shader/GeometryRenderer.h"

namespace open3d {

namespace geometry {
class PointCloud;
}

namespace visualization {
class SelectionPolygon;
class PointCloudPicker;

class VisualizerWithAnnotation : public VisualizerForAnnotation {
public:
    enum class SelectionMode { None = 0, Point = 1, Rectangle = 2, Moving = 3 };

public:
    VisualizerWithAnnotation() {}
    ~VisualizerWithAnnotation() override {}
    VisualizerWithAnnotation(const VisualizerWithAnnotation &) =
            delete;
    VisualizerWithAnnotation &operator=(
            const VisualizerWithAnnotation &) = delete;

public:
    bool AddGeometry(std::shared_ptr<const geometry::Geometry> geometry_ptr,
                     bool reset_bounding_box = true) override;
    bool UpdateGeometry(std::shared_ptr<const geometry::Geometry> geometry_ptr =
                                nullptr) override;
    void PrintVisualizerHelp() override;
    void UpdateWindowTitle() override;
    void BuildUtilities() override;
    void SetPointSize(double size);
    std::vector<int> PickPoints(double x, double y, double w, double h);

    struct PickedPoint {
        int index;
        Eigen::Vector3d coord;
    };
    std::vector<PickedPoint> GetPickedPoints() const;
    void ClearPickedPoints();
    void AddPickedPoints(const std::vector<int> indices);
    void RemovePickedPoints(const std::vector<int> indices);

    void RegisterSelectionChangedCallback(std::function<void()> f);
    /// Do not change the number of vertices in geometry, but can change the
    /// vertex values and call UpdateGeometry().
    void RegisterSelectionMovingCallback(std::function<void()> f);
    void RegisterSelectionMovedCallback(std::function<void()> f);
    void SetFilename(std::string name);

protected:
    bool InitViewControl() override;
    bool InitRenderOption() override;
    void WindowResizeCallback(GLFWwindow *window, int w, int h) override;
    void MouseMoveCallback(GLFWwindow *window, double x, double y) override;
    void MouseScrollCallback(GLFWwindow *window, double x, double y) override;
    void MouseButtonCallback(GLFWwindow *window,
                             int button,
                             int action,
                             int mods) override;
    void KeyPressCallback(GLFWwindow *window,
                          int key,
                          int scancode,
                          int action,
                          int mods) override;
    void InvalidateSelectionPolygon();
    void InvalidatePicking();
    float GetDepth(int winX, int winY);
    Eigen::Vector3d CalcDragDelta(double winX, double winY);
    enum DragType { DRAG_MOVING, DRAG_END };
    void DragSelectedPoints(const Eigen::Vector3d &delta, DragType type);
    const std::vector<Eigen::Vector3d> *GetGeometryPoints(
            std::shared_ptr<const geometry::Geometry> geometry);

protected: // [changed] new methods listed below
    void SaveTag();
    bool ReadTag();

protected:
    std::shared_ptr<SelectionPolygon> selection_polygon_ptr_;
    std::shared_ptr<glsl::SelectionPolygonRenderer>
            selection_polygon_renderer_ptr_;
    SelectionMode selection_mode_ = SelectionMode::None;
    Eigen::Vector2d mouse_down_pos_;
    std::vector<int> points_in_rect_;
    float drag_depth_ = 0.0f;

    std::shared_ptr<PointCloudPicker> pointcloud_picker_ptr_;
    std::shared_ptr<glsl::PointCloudPickerRenderer>
            pointcloud_picker_renderer_ptr_;

    std::shared_ptr<const geometry::Geometry> geometry_ptr_;
    std::shared_ptr<glsl::PointCloudRendererForAnnotation> geometry_renderer_ptr_;  // [changed]

    RenderOptionForAnnotation pick_point_opts_;

    std::shared_ptr<geometry::PointCloud> ui_points_geometry_ptr_;
    std::shared_ptr<glsl::GeometryRenderer> ui_points_renderer_ptr_;

    std::unordered_map<int, Eigen::Vector3d> selected_points_;
    std::unordered_map<int, Eigen::Vector3d> selected_points_before_drag_;
    std::shared_ptr<geometry::PointCloud> ui_selected_points_geometry_ptr_;
    std::shared_ptr<glsl::GeometryRenderer> ui_selected_points_renderer_ptr_;

    std::function<void()> on_selection_changed_;
    std::function<void()> on_selection_moving_;
    std::function<void()> on_selection_moved_;

protected: // [changed] new attributes listed below
    int tag = 0;
    int length;
    std::string filename;
    std::vector<std::vector<int>> labels;
};

}  // namespace visualization
}  // namespace open3d
