// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <GL/glew.h>

#include "open3d/geometry/Geometry.h"
#include "open3d/visualization/shader/ShaderWrapper.h"

#include "pc_annotation/RenderOptionForAnnotation.h"

namespace open3d {
namespace visualization {

namespace glsl {

class ShaderWrapperForAnnotation : public ShaderWrapper{
public:
    virtual ~ShaderWrapperForAnnotation() {}
    ShaderWrapperForAnnotation(const ShaderWrapperForAnnotation &) = delete;
    ShaderWrapperForAnnotation &operator=(const ShaderWrapperForAnnotation &) = delete;

protected:
    ShaderWrapperForAnnotation(const std::string &name) : ShaderWrapper(name) {}

public:
    bool Render(const geometry::Geometry &geometry,
               const std::vector<std::vector<int>> &labels,
               const RenderOptionForAnnotation &option,
               const ViewControl &view);

protected:
    virtual bool BindGeometry(const geometry::Geometry &geometry,
                              const std::vector<std::vector<int>> &labels,
                              const RenderOptionForAnnotation &option,
                              const ViewControl &view) = 0;


protected:
    bool compiled_ = false;
    bool bound_ = false;

    void SetShaderName(const std::string &shader_name) {
        shader_name_ = shader_name;
    }

private:
    std::string shader_name_ = "ShaderWrapperForAnnotation";
};

}  // namespace glsl

}  // namespace visualization
}  // namespace open3d
