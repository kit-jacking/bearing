import numpy as np
import open3d as o3d
from open3d.cpu.pybind.utility import Vector3dVector


def select_statistical_outlier(point_cloud, neighbours=30, std_ratio=2.0) -> [o3d.geometry.PointCloud,
                                                                              o3d.geometry.PointCloud]:
    point_cloud_without_outliers, ind = point_cloud.remove_statistical_outlier(nb_neighbors=neighbours,
                                                                               std_ratio=std_ratio)
    outliers = point_cloud.select_by_index(ind, invert=True)

    outliers.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([point_cloud_without_outliers, outliers])
    return point_cloud_without_outliers, outliers


print("Loading point cloud")
# cloud: o3d.geometry.PointCloud = o3d.io.read_point_cloud("dane/otwock_freski18.pcd")
cloud: o3d.geometry.PointCloud = o3d.io.read_point_cloud("dane/otwock_freski18_kdownsampled_100.pcd")

print("Coloring cloud")
no_points = len(cloud.points)
temp = 1 / no_points
r, g, b = 1, 0, 1
cloud.colors = Vector3dVector(np.asarray(
    [[(temp / 2 * x + 0.5) * r, (temp / 2 * x + 0.5) * g, (temp / 2 * x + 0.5) * b] for x in range(1, no_points + 1, 1)]))

# print("Downsampling point cloud")
# n = 100
# downsampled_cloud: o3d.geometry.PointCloud = cloud.uniform_down_sample(every_k_points=n)
# downsampled_cloud: o3d.geometry.PointCloud = cloud.voxel_down_sample(voxel_size=1 / n)

# print("Displaying downsampled point cloud")
# o3d.visualization.draw_geometries([downsampled_cloud])

# print("Saving downsampled point cloud")
# o3d.io.write_point_cloud("dane/otwock_freski18_kdownsampled_100.pcd", downsampled_cloud, write_ascii=False, compressed=False, print_progress=True)

# print("Selecting statistical outliers")
# select_statistical_outlier(cloud)
