// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include "open3d/visualization/visualizer/RenderOption.h"

#include "open3d/utility/Logging.h"

namespace open3d {
namespace visualization {

/// \class RenderOptionForAnnotation
///
/// \brief Defines rendering options for visualizer.
class RenderOptionForAnnotation : public RenderOption {
public:
    /// \enum PointColorOption
    ///
    /// \brief Enum class for point color for PointCloud.
    enum class PointColorOption {
        Default = 0,
        Color = 1,
        XCoordinate = 2,
        YCoordinate = 3,
        ZCoordinate = 4,
        Label = 5,
        Normal = 9,
    };

    PointColorOption point_color_option_ = PointColorOption::Default;

public:
    /// \brief Default Constructor.
    RenderOptionForAnnotation() {
        // VS2013 does not fully support C++11
        // Array initialization has to be done in constructors.
        light_position_relative_[0] = Eigen::Vector3d(0, 0, 2);
        light_position_relative_[1] = Eigen::Vector3d(0, 0, 2);
        light_position_relative_[2] = Eigen::Vector3d(0, 0, -2);
        light_position_relative_[3] = Eigen::Vector3d(0, 0, -2);
        light_color_[0] = Eigen::Vector3d::Ones();
        light_color_[1] = Eigen::Vector3d::Ones();
        light_color_[2] = Eigen::Vector3d::Ones();
        light_color_[3] = Eigen::Vector3d::Ones();
        light_diffuse_power_[0] = 0.66;
        light_diffuse_power_[1] = 0.66;
        light_diffuse_power_[2] = 0.66;
        light_diffuse_power_[3] = 0.66;
        light_specular_power_[0] = 0.2;
        light_specular_power_[1] = 0.2;
        light_specular_power_[2] = 0.2;
        light_specular_power_[3] = 0.2;
        light_specular_shininess_[0] = 100.0;
        light_specular_shininess_[1] = 100.0;
        light_specular_shininess_[2] = 100.0;
        light_specular_shininess_[3] = 100.0;
        background_color_[0] = 0.7;
        background_color_[1] = 0.7;
        background_color_[2] = 0.7;
    }
    ~RenderOptionForAnnotation() override {}
};

}  // namespace visualization
}  // namespace open3d
