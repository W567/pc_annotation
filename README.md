# Point Cloud Annotation Toolkit (Open3d-based)

## Dependency

### Open3d

Build Open3D from source based on https://www.open3d.org/docs/release/compilation.html.
```
git clone https://github.com/isl-org/Open3D

# Only needed for Ubuntu
util/install_deps_ubuntu.sh

mkdir build
cd build
cmake ..
make -j
sudo make install
make install-pip-package # To install pip package
# install open3d with generated pip wheel in build/lib
```

## Build from Source

```
cd pc_annotation
mkdir build
cd build
cmake ..
make -j
sudo make install
```

## usage

```
import numpy as np
import open3d as o3d
import pc_annotation

pcd = o3d.io.read_point_cloud(/path/to/pcd/file)

pc_annotation.annotate(np.asarray(pcd.points), np.asarray(pcd.normals), np.asarray(pcd.colors), "filename")

# Press 'h' with Open3D visualizer selected to show help menu
[Open3D INFO] ---------------- Annotation control ----------------
[Open3D INFO]     Space        :        Annotate with current tag.
[Open3D INFO]     N            :        Skip to next tag.
[Open3D INFO]     B            :        Back to previous tag.
[Open3D INFO]     S            :        Save tags.
[Open3D INFO]     R            :        Read tags.
```
