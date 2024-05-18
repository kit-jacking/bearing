import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from copy import copy
from numpy.random import default_rng
import laspy

rng = default_rng()

# Implementacja klasy RANSAC
class RANSAC:
    def __init__(self, n=10, k=1000, t=0.01, d=100, model=None, loss=None, metric=None):
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
    r = las_pcd.intensity / max(las_pcd.intensity)
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
    result = a * x + b * y + c * z + d
    if result > -1 and result < 1:
        return True
    else:
        return False

def map_to_2d(points_3d):
    return points_3d[:, :2]

if __name__ == "__main__":

    luk3 = r"C:\Users\qattr\Desktop\luk1.las"
    luk3_o3d = las_to_o3d(luk3)
    luk3_down = luk3_o3d.uniform_down_sample(100)
    xyz_pts = np.asarray(luk3_o3d.points)

    pkt3 = r"C:\Users\qattr\Desktop\luk1_4pts.las"
    pkt3_o3d = las_to_o3d(pkt3)

    # Przypisanie punktów z programu Tymonowego
    center_point = np.asarray(pkt3_o3d.points)[2]
    left_point = np.asarray(pkt3_o3d.points)[0]
    right_point = np.asarray(pkt3_o3d.points)[1]
    depth_point = np.asarray(pkt3_o3d.points)[3]

    depth_dist = center_point[0] - depth_point[0]

    # Równanie płaszczyzny
    plane_equation = equation_plane(center_point[0], center_point[1], center_point[2], left_point[0], left_point[1], left_point[2], right_point[0], right_point[1], right_point[2])

    # Znalezienie punktów należących do płaszczyzny
    points_on_plane = []
    for point in xyz_pts:
        if point_on_plane(plane_equation, point):
            points_on_plane.append(point)
    points_on_plane = np.array(points_on_plane)

    # Utworzenie punktów przesuniętych o wartość 'depth_dist'
    points_in_depth = points_on_plane.copy()
    points_in_depth[:, 2] = points_in_depth[:, 2] - depth_dist

    # Mapowanie punktów 3D do 2D
    points_2d = map_to_2d(points_on_plane)

    regressor = RANSAC(model=LinearRegressor(), loss=square_error_loss, metric=mean_square_error)

    # Przesunięcie wierzchołka wykresu 
    X_ = points_on_plane[:, 1].reshape(-1,1)
    X = X_ - center_point[1]
    y_ = points_on_plane[:, 2].reshape(-1,1)
    y = y_ - center_point[2]

    regressor.fit(X, y)

    # Wygenerowanie punktów na dopasowanej linii regresji
    line = np.linspace(np.min(X), np.max(X), num=100).reshape(-1, 1)
    line_z = regressor.predict(line)

    # Przywrócenie przesunięcia do oryginalnych współrzędnych
    line_x = np.full_like(line, center_point[0])
    line_y = line + center_point[1]
    line_z += center_point[2]

    # Utworzenie punktów w przestrzeni 3D
    line_points = np.hstack([line_x, line_y, line_z])

    # Łączenie punktów z powierzchnią przesuniętą o głębokość łuku
    depth_line_points = np.copy(line_points)
    depth_line_points[:, 0] -= depth_dist

    # Utworzenie obiektu LineSet
    line_set = o3d.geometry.LineSet()

    # Dodanie punktów do LineSet
    line_points_all = np.vstack((line_points, depth_line_points))
    line_set.points = o3d.utility.Vector3dVector(line_points_all)

    # Definiowanie linii łączących kolejne punkty
    lines = [[i, i+1] for i in range(len(line_points) - 1)]
    lines += [[i, i + len(line_points)] for i in range(len(line_points))]
    line_set.lines = o3d.utility.Vector2iVector(lines)

    # Ustawienie kolorów (opcjonalnie)
    colors = [[1, 0, 0] for _ in range(len(lines))]  # czerwone linie
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # Wizualizacja
    o3d.visualization.draw_geometries([luk3_o3d, pkt3_o3d, line_set])
