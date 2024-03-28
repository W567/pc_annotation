# Point Cloud Annotation Toolkit (Open3d-based)

## Dependency

### Open3d

Open3d C++ library is required
Reference: https://www.open3d.org/docs/release/compilation.html

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
```


## usage

```
import numpy as np
import open3d as o3d
import pc_annotation

pcd = o3d.io.read_point_cloud(/path/to/pcd/file)

pc_annotation.annotate(np.asarray(pcd.points), np.asarray(pcd.normals), np.asarray(pcd.colors), "filename")
```
