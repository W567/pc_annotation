import os
import numpy as np
import pc_annotation
import open3d as o3d

pcd_path = f"{os.path.dirname(os.path.abspath(__file__))}/../pcd"

pcd = o3d.io.read_point_cloud(f"{pcd_path}/ycb_021_bleach_cleanser.pcd")

pc_annotation.annotate(np.asarray(pcd.points), np.asarray(pcd.normals), np.asarray(pcd.colors), f"{pcd_path}/ycb_021_bleach_cleanser.label")

# extract the annotated point cloud based on the label
label = np.loadtxt(f"{pcd_path}/ycb_021_bleach_cleanser.label")
label = label.astype(int)
label = label.reshape(len(label), -1)
for i in range(label.shape[1]):
    idx = np.nonzero(label[:, i])
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(np.array(pcd.points)[idx])
    new_pcd.colors = o3d.utility.Vector3dVector(np.array(pcd.colors)[idx])
    new_pcd.normals = o3d.utility.Vector3dVector(np.array(pcd.normals)[idx])
    o3d.io.write_point_cloud(f"{pcd_path}/ycb_021_bleach_cleanser_aff{i}.pcd", new_pcd)

