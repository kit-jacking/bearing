import pandas as pd
import matplotlib.pyplot as plt
import random
import math
import numpy as np
import open3d as o3d

class RANSAC:
    """
    RANSAC Class
    """
    def __init__(self, point_cloud, max_iterations,            
                 distance_ratio_threshold):
        self.point_cloud = point_cloud
        self.max_iterations = max_iterations
        self.distance_ratio_threshold = distance_ratio_threshold
    
    def run(self):
        """
        method for running the class directly
        :return:
        """
        inliers, outliers = self._ransac_algorithm(self.max_iterations, self.distance_ratio_threshold)
        points = np.vstack((inliers.X, inliers.Y, inliers.Z)).T
        pt_cld = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(points))
        o3d.visualization.draw_geometries([pt_cld])
        # ax = plt.figure().add_subplot(projection='3d')
        # ax.scatter(inliers.X , inliers.Y,  inliers.Z,  c="green", zdir='z')
        # ax.scatter(outliers.X, outliers.Y, outliers.Z, c="red", zdir='z')
        # plt.show()

    def _visualize_point_cloud(self):
        """
        Visualize the 3D data
        :return: None
        """
        points = np.vstack((self.X, self.Y, self.Z)).T
        pt_cld = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(points))
        o3d.visualization.draw_geometries([pt_cld])
        # o3d.visualization.draw_geometries([ori_temp, ref_temp])
        # Visualize the point cloud data
        # fig = plt.figure()
        # ax = Axes3D(fig)
        # ax.scatter(self.point_cloud.X , self.point_cloud.Y, self.point_cloud.Z)
        # plt.show()
        ax = plt.figure().add_subplot(projection='3d')
        ax.scatter(self.point_cloud.X , self.point_cloud.Y, self.point_cloud.Z)
     
    def _ransac_algorithm(self, max_iterations, distance_ratio_threshold):
        """
        Implementation of the RANSAC logic
        :return: inliers(Dataframe), outliers(Dataframe)
        """

        inliers_result = set()
        print('1')
        while max_iterations:
            max_iterations -= 1
            # Add 3 random indexes
            random.seed()
            inliers = []
            while len(inliers) < 3:
                random_index = random.randint(0, len(self.point_cloud.X)-1)
                inliers.append(random_index)
            # print(inliers)
            try:
                # In case of *.xyz data
                x1, y1, z1, _, _, _ = self.point_cloud.loc[inliers[0]]
                x2, y2, z2, _, _, _ = self.point_cloud.loc[inliers[1]]
                x3, y3, z3, _, _, _ = self.point_cloud.loc[inliers[2]]
            except:
                # In case of *.pcd data
                x1, y1, z1 = self.point_cloud.loc[inliers[0]]
                x2, y2, z2 = self.point_cloud.loc[inliers[1]]
                x3, y3, z3 = self.point_cloud.loc[inliers[2]]
            # Plane Equation --> ax + by + cz + d = 0
            # Value of Constants for inlier plane
            a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)
            b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1)
            c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
            d = -(a*x1 + b*y1 + c*z1)
            plane_lenght = max(0.1, math.sqrt(a*a + b*b + c*c))
            print('2')
            print(len(self.point_cloud))
            for point in self.point_cloud.iterrows():
                # print(point)
                index = point[0]
                # Skip iteration if point matches the randomly generated inlier point
                if index in inliers:
                    continue
                try:
                    # In case of *.xyz data
                    x, y, z, _, _, _ = point[1]
                except:
                    # In case of *.pcd data
                    x, y, z = point[1]

                # Calculate the distance of the point to the inlier plane
                distance = math.fabs(a*x + b*y + c*z + d)/plane_lenght
                # Add the point as inlier, if within the threshold distancec ratio
                if distance <= distance_ratio_threshold:
                    inliers.append(index)
            # Update the set for retaining the maximum number of inlier points
            if len(inliers) > len(inliers_result):
                inliers_result.clear()
                inliers_result = inliers
        print('3')
        # Segregate inliers and outliers from the point cloud
        inlier_points = pd.DataFrame(columns=["X", "Y", "Z"])
        outlier_points = pd.DataFrame(columns=["X", "Y", "Z"])
        for point in self.point_cloud.iterrows():
            if point[0] in inliers_result:
                inlier_points = pd.concat([inlier_points, pd.DataFrame({"X": [point[1]["X"]],
                                                      "Y": [point[1]["Y"]],
                                                      "Z": [point[1]["Z"]]})], ignore_index=True)
                continue
            outlier_points = pd.concat([outlier_points,pd.DataFrame({"X": [point[1]["X"]],
                                                      "Y": [point[1]["Y"]],
                                                      "Z": [point[1]["Z"]]})], ignore_index=True)
        print(len(inlier_points))
        print(len(outlier_points))
        return inlier_points, outlier_points
    
    
if __name__ == "__main__":
    # Read the point cloud data
    # point_cloud = pd.read_csv("point_cloud_data_sample.xyz", delimiter=" ", nrows=500)
    pcd = o3d.io.read_point_cloud(r"C:\Users\qattr\Desktop\STUD\SEM 6\FTP2\chmura_punktow_z_dj_phantom_pro_3_cut.pcd")
    # print(np.asarray(pcd.points))
    point_cloud = pd.DataFrame(pcd.points, columns=["X" ,"Y" ,"Z"])
    APPLICATION = RANSAC(point_cloud, max_iterations=3, distance_ratio_threshold=0.1)
    APPLICATION.run()