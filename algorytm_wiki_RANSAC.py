from copy import copy
import numpy as np
from numpy.random import default_rng
import open3d as o3d
import laspy
import matplotlib.pyplot as plt
rng = default_rng()

class RANSAC:
    def __init__(self, n=200, k=2000, t=0.1, d=100, model=None, loss=None, metric=None):
        self.n = n              # `n`: Minimum number of data points to estimate parameters
        self.k = k              # `k`: Maximum iterations allowed
        self.t = t              # `t`: Threshold value to determine if points are fit well
        self.d = d              # `d`: Number of close data points required to assert model fits well
        self.model = model      # `model`: class implementing `fit` and `predict`
        self.loss = loss        # `loss`: function of `y_true` and `y_pred` that returns a vector
        self.metric = metric    # `metric`: function of `y_true` and `y_pred` and returns a float
        self.best_fit = None
        self.best_error = np.inf

    def fit(self, X, y):
        for _ in range(self.k):
            ids = rng.permutation(X.shape[0])

            maybe_inliers = ids[: self.n]
            maybe_model = copy(self.model).fit(X[maybe_inliers], y[maybe_inliers])

            thresholded = (
                self.loss(y[ids][self.n :], maybe_model.predict(X[ids][self.n :]))
                < self.t
            )

            inlier_ids = ids[self.n :][np.flatnonzero(thresholded).flatten()]

            if inlier_ids.size > self.d:
                inlier_points = np.hstack([maybe_inliers, inlier_ids])
                better_model = copy(self.model).fit(X[inlier_points], y[inlier_points])

                this_error = self.metric(
                    y[inlier_points], better_model.predict(X[inlier_points])
                )

                if this_error < self.best_error:
                    self.best_error = this_error
                    self.best_fit = better_model

        return self

    def predict(self, X):
        return self.best_fit.predict(X)

def square_error_loss(y_true, y_pred):
    return (y_true - y_pred) ** 2

def mean_square_error(y_true, y_pred):
    return np.sum(square_error_loss(y_true, y_pred)) / y_true.shape[0]

class LinearRegressor:
    def __init__(self):
        self.params = None

    def fit(self, X: np.ndarray, y: np.ndarray):
        r, _ = X.shape
        X = np.hstack([np.ones((r, 1)), np.cosh(X)])
        self.params = np.linalg.inv(X.T @ X) @ X.T @ y
        return self

    def predict(self, X: np.ndarray):
        r, _ = X.shape
        X = np.hstack([np.ones((r, 1)), np.cosh(X)])
        return X @ self.params
    
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

def equation_plane(x1, y1, z1, x2, y2, z2, x3, y3, z3): 
    a1 = x2 - x1
    b1 = y2 - y1
    c1 = z2 - z1
    a2 = x3 - x1
    b2 = y3 - y1
    c2 = z3 - z1
    a = b1 * c2 - b2 * c1
    b = a2 * c1 - a1 * c2
    c = a1 * b2 - b1 * a2
    d = (- a * x1 - b * y1 - c * z1)
    print ("equation of plane is ", a, "x +", b, "y +", c, "z +", d, "= 0.")

    return a, b, c, d

def point_on_plane(plane_equation, point):
    a, b, c, d = plane_equation
    x, y, z = point
    return a * x + b * y + c * z + d == 0

def map_to_2d(points_3d):
    return points_3d[:, :2]

if __name__ == "__main__":

    # file1 = r"C:\Users\julia\Documents\GEOINFORMATYKA\sem6\APFiWM\projekt_sem\chmury_cc\otwock_freski18.las"
    # file1_o3d = las_to_o3d(file1)
    # file1_down = file1_o3d.uniform_down_sample(100)
    # o3d.visualization.draw_geometries([file1_o3d])

    luk3 = r"C:\Users\julia\Documents\GEOINFORMATYKA\sem6\APFiWM\projekt_sem\luki\luk3.las"
    luk3_o3d = las_to_o3d(luk3)
    luk3_down = luk3_o3d.uniform_down_sample(100)
    xyz_pts = np.asarray(luk3_down.points)
    # print(xyz_pts[:, 0])

    pkt3 = r"C:\Users\julia\Documents\GEOINFORMATYKA\sem6\APFiWM\projekt_sem\luki\luk3_6punktow.las"
    pkt3_o3d = las_to_o3d(pkt3)
    print(np.asarray(pkt3_o3d.points))
    pivot_pts = np.asarray(pkt3_o3d.points)[0]
    edge1 = np.asarray(pkt3_o3d.points)[1]
    edge2 = np.asarray(pkt3_o3d.points)[2]

    plane_equation = equation_plane(pivot_pts[0], pivot_pts[1], pivot_pts[2], edge1[0], edge1[1], edge1[2], edge2[0], edge2[1], edge2[2])

    # Znalezienie punktów należących do płaszczyzny
    points_on_plane = []
    for point in xyz_pts:
        if point_on_plane(plane_equation, point):
            points_on_plane.append(point)
    points_on_plane = np.array(points_on_plane)

    print(points_on_plane)

    # Mapowanie punktów 3D do 2D
    # points_2d = map_to_2d(points_on_plane)

    regressor = RANSAC(model=LinearRegressor(), loss=square_error_loss, metric=mean_square_error)

    for i in range(len(xyz_pts[0])):
        for j in range(len(xyz_pts[0])):
            if i != j:
                X_ = xyz_pts[:, i].reshape(-1,1)
                X = X_ - pivot_pts[i]
                y_ = xyz_pts[:, j].reshape(-1,1)
                y = y_ - pivot_pts[j]

                regressor.fit(X, y)

                ax = plt.figure().add_subplot(projection="3d")
                ax.scatter(X, y, zs=0, zdir="z")

                mn = np.min(np.concatenate([X, y]))
                mx = np.max(np.concatenate([X, y]))

                line = np.linspace(np.floor(mn), np.ceil(mx), num=1000).reshape(-1, 1)
                plt.plot(line, regressor.predict(line), zs=0, zdir="z", c="peru")
                plt.show()