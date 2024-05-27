import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import random
import math
import open3d as o3d
import laspy

class RANSACArc:
    """
    RANSAC Class for detecting arcs
    """
    def __init__(self, point_cloud, max_iterations, distance_threshold):
        self.point_cloud = point_cloud
        self.max_iterations = max_iterations
        self.distance_threshold = distance_threshold

    def run(self):
        """
        Method to run the RANSAC algorithm
        """
        inliers, outliers = self._ransac_algorithm(self.max_iterations, self.distance_threshold)
        if inliers is not None and outliers is not None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(inliers.X , inliers.Y,  inliers.Z,  c="green", label='Inliers')
            ax.scatter(outliers.X, outliers.Y, outliers.Z, c="red", label='Outliers')
            plt.show()
        else:
            print("Could not find a circle with given points.")

    def _ransac_algorithm(self, max_iterations, distance_threshold):
        """
        Implementation of the RANSAC logic for detecting arcs
        """
        inliers_result = set()
        while max_iterations:
            max_iterations -= 1
            # Choose 3 random indexes
            random.seed()
            inliers = []
            while len(inliers) < 3:
                random_index = random.randint(0, len(self.point_cloud) - 1)
                inliers.append(random_index)

            # Extract the points
            p1 = self.point_cloud.iloc[inliers[0]]
            p2 = self.point_cloud.iloc[inliers[1]]
            p3 = self.point_cloud.iloc[inliers[2]]

            # Calculate the center and radius of the circle defined by the three points
            center, radius = self._calculate_circle(p1, p2, p3)

            # Find inliers
            current_inliers = []
            for index, point in self.point_cloud.iterrows():
                if index in inliers:
                    continue
                distance = self._point_to_circle_distance(point, center)
                if distance <= distance_threshold:
                    current_inliers.append(index)

            if len(current_inliers) > len(inliers_result):
                inliers_result = set(current_inliers)

        # Segregate inliers and outliers from the point cloud
        inlier_points = self.point_cloud.loc[list(inliers_result)]
        outlier_points = self.point_cloud.drop(index=inliers_result)

        return inlier_points, outlier_points

    def _calculate_circle(self, p1, p2, p3):
        """
        Calculate the center and radius of the circle passing through three points
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        # Intermediate values
        A = x2 - x1
        B = y2 - y1
        C = x3 - x1
        D = y3 - y1
        E = A * (x1 + x2) + B * (y1 + y2)
        F = C * (x1 + x3) + D * (y1 + y3)
        G = 2 * (A * (y3 - y2) - B * (x3 - x2))

        # Check if points are collinear
        if abs(G) < 1e-6:
            return None, None

        # Center coordinates
        center_x = (D * E - B * F) / G
        center_y = (A * F - C * E) / G

        # Radius
        radius = math.sqrt((center_x - x1)**2 + (center_y - y1)**2)

        return (center_x, center_y), radius


    def _point_to_circle_distance(self, point, center):
        """
        Calculate the distance between a point and the center of a circle
        """
        x, y = point
        if center is None:
            return float('inf')  # Return infinity if center is None

        center_x, center_y = center
        distance = math.sqrt((x - center_x)**2 + (y - center_y)**2)
        return distance
    
def las_to_o3d(file): 
    las_pcd = laspy.file.File(file, mode = 'r') 
    x = las_pcd.x 
    y = las_pcd.y
    z = las_pcd.z 

    # Normalizacja szarosci
    r = las_pcd.intensity/max(las_pcd.intensity)
    g = r
    b = r

    # Konwersja do format NumPy do o3d
    las_points = np.vstack((x, y, z)).transpose()
    las_colors = np.vstack((r, g, b)).transpose()

    pcd_o3d = o3d.geometry.PointCloud()
    pcd_o3d.points = o3d.utility.Vector3dVector(las_points)
    pcd_o3d.colors = o3d.utility.Vector3dVector(las_colors)

    return pcd_o3d

if __name__ == "__main__":

    file1 = r"C:\Users\julia\Documents\GEOINFORMATYKA\sem6\APFiWM\projekt_sem\chmury_cc\otwock_freski18.las"
    file1_o3d = las_to_o3d(file1)
    file1_down = file1_o3d.uniform_down_sample(100)
    o3d.visualization.draw_geometries([file1_o3d])
    # print(file1_o3d)

    # point_cloud = pd.DataFrame({'X': x_points, 'Y': y_points})

    # # Definicja parametrów RANSAC
    # max_iterations = 50
    # distance_threshold = 1

    # # Wywołanie algorytmu RANSAC
    # ransac_arc = RANSACArc(point_cloud, max_iterations, distance_threshold)
    # ransac_arc.run()
