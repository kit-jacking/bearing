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


def manual_pcd_croppings(point_cloud):
    print("Function for manual geometry cropping")
    print("The data processing steps:")
    print(" (0) Manual definition of the view point by the mouse or:")
    print(" (0.1) Press 'X' twice to align geometry with direction of x-axis")
    print(" (0.2) Press 'Y' twice to align geometry with direction of y-axis")
    print(" (0.3) Press 'Z' twice to align geometry with direction of z-axis")
    print(" (1) Press 'K' to lock screen and to switch to selection mode")
    print(" (2.1) Drag for rectangle selection or")
    print(" (2.2)or use ctrl + left click for polygon selection")
    print(" (3) Press 'C' to get a selected geometry and to save it")
    print(" (4) Press 'F' to switch to freeview mode")
    o3d.visualization.draw_geometries_with_editing([point_cloud], window_name='Manual point cloud cropping')
    return point_cloud


def manual_point_picking(point_cloud: o3d.geometry.PointCloud):
    print("Manual point measurement")
    print("The data processing steps:")
    print(" (1.1) Point measurement - shift + left mouse button")
    print(" (1.2) The undo point picking - shift + right mouse button")
    print(" (2) End of measurement - press Q button")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name='Manual point picking')
    vis.add_geometry(point_cloud)
    vis.run()  # user picks points
    vis.destroy_window()
    print("The end of measurement")
    return vis.get_picked_points()


def create_spheres_on_points(point_cloud: o3d.geometry.PointCloud, radius: float = 0.1, color: list = [1, 0.7, 0]):
    spheres = []
    for point in point_cloud.points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius)
        sphere.paint_uniform_color(color)
        sphere.compute_vertex_normals()
        sphere = sphere.translate(point)
        spheres.append(sphere)
    return spheres


print("Loading point cloud")
filename = "dane/luk1.ply"
cloud: o3d.geometry.PointCloud = o3d.io.read_point_cloud(filename)

print("Coloring cloud")
no_points = len(cloud.points)
temp = 1 / no_points
r, g, b = 1, 0, 1
cloud.colors = Vector3dVector(np.asarray(
    [[(temp / 2 * x + 0.5) * r, (temp / 2 * x + 0.5) * g, (temp / 2 * x + 0.5) * b] for x in
     range(1, no_points + 1, 1)]))

# cloud_2: o3d.geometry.PointCloud = o3d.io.read_point_cloud("dane/luk3_6punktow.pcd")
# o3d.visualization.draw_geometries([cloud, *create_spheres_on_points(cloud_2)], window_name="Visualizing point cloud " + filename)

# print("Downsampling point cloud")
# n = 100
# downsampled_cloud: o3d.geometry.PointCloud = cloud.uniform_down_sample(every_k_points=n)
# downsampled_cloud: o3d.geometry.PointCloud = cloud.voxel_down_sample(voxel_size=1 / n)

# print("Displaying downsampled point cloud")
# o3d.visualization.draw_geometries([downsampled_cloud])

# print("Saving downsampled point cloud")
# o3d.io.write_point_cloud("dane/otwock_freski18_kdownsampled_100.pcd", downsampled_cloud, write_ascii=False, compressed=False, print_progress=False)

# print("Selecting statistical outliers")
# select_statistical_outlier(cloud)

# cropped_cloud = manual_pcd_croppings(cloud)

selected_points_indexes = manual_point_picking(cloud)
selected_points: o3d.geometry.PointCloud = cloud.select_by_index(selected_points_indexes)
o3d.io.write_point_cloud("dane/luk1_6punktow.pcd", selected_points, write_ascii=False, compressed=False, print_progress=False)

# selected_points_spheres = create_spheres_on_points(selected_points, radius=0.05)
# o3d.visualization.draw_geometries([cloud, *selected_points_spheres])
