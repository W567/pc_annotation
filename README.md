# Point Cloud Annotation Toolkit (Open3d-based)

## usage

```
import numpy as np
import open3d as o3d
import pc_annotation

pcd = o3d.io.read_point_cloud(/path/to/pcd/file)

pc_annotation.annotate(np.asarray(pcd.points), np.asarray(pcd.normals), np.asarray(pcd.colors), "filename")
```