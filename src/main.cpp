// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"

#include "pc_annotation/DrawGeometry.h"

using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

std::vector<Eigen::Vector3d> MatrixToVector3d(const Eigen::MatrixXd& mat) {
    std::vector<Eigen::Vector3d> vec(mat.rows());
    for (int i = 0; i < mat.rows(); ++i) {
        vec[i] = mat.row(i);
    }
    return vec;
}

bool annotate(
        const RowMatrixXd& xyz,
        const RowMatrixXd& normals,
        const RowMatrixXd colors,
        const std::string &filename,
        const std::string &window_name = "Open3D",
        int width = 640,
        int height = 480,
        int left = 50,
        int top = 50
        ) {
    open3d::geometry::PointCloud cpp_pointcloud;
    cpp_pointcloud.points_ = MatrixToVector3d(xyz);
    cpp_pointcloud.normals_ = MatrixToVector3d(normals);
    cpp_pointcloud.colors_ = MatrixToVector3d(colors);

    std::vector<std::shared_ptr<const open3d::geometry::PointCloud>> geometry_ptrs;
    geometry_ptrs.push_back(std::make_shared<const open3d::geometry::PointCloud>(cpp_pointcloud));

    open3d::visualization::DrawGeometriesWithAnnotation(
        geometry_ptrs, filename, window_name, width, height, left, top);
    return true;
}

PYBIND11_MODULE(pc_annotation, m) {
    m.def("annotate",
          &annotate,
          "Point cloud annotation tool",
          pybind11::arg("xyz"),
          pybind11::arg("normals"),
          pybind11::arg("colors"),
          pybind11::arg("filename"),
          pybind11::arg("window_name") = "Open3D",
          pybind11::arg("width") = 640,
          pybind11::arg("height") = 480,
          pybind11::arg("left") = 50,
          pybind11::arg("top") = 50,
          pybind11::return_value_policy::reference_internal);
}