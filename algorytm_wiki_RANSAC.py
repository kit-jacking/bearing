import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from copy import copy
from numpy.random import default_rng
import laspy

print(laspy.__version__)

rng = default_rng()

class RANSAC:
    def __init__(self, n=10, k=2000, t=0.001, d=50, model=None, loss=None, metric=None):
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

class HyperbolicRegressor:
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
    
class QuadraticRegressor:
    def __init__(self):
        self.params = None

    def fit(self, X: np.ndarray, y: np.ndarray):
        r, _ = X.shape
        X = np.hstack([np.ones((r, 1)), X, X**2])
        self.params = np.linalg.inv(X.T @ X) @ X.T @ y
        return self

    def predict(self, X: np.ndarray):
        r, _ = X.shape
        X = np.hstack([np.ones((r, 1)), X, X**2])
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

def filter_points_by_limits(points, lower_limit, upper_limit):
    filtered_points = []
    for point in points:
        if lower_limit <= point[2] <= upper_limit:
            filtered_points.append(point)
    return np.array(filtered_points)

def map_to_2d(points_3d):
    return points_3d[:, :2]

def visualize_points(left_point, right_point, center_point, depth_point, points_on_plane, line_points, depth_line_points):
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
    colors = [[0.8, 0.2, 0.4] for _ in range(len(lines))]  # rozowe linie laczace punkty 
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # Tworzenie siatki 3D (TriangleMesh) z punktów
    vertices = line_points_all
    triangles = []
    num_points = len(line_points)

    # Dodawanie trójkątów między punktami
    for i in range(num_points - 1):
        triangles.append([i, i + 1, i + num_points])
        triangles.append([i + 1, i + num_points + 1, i + num_points])

    # Konwersja do odpowiednich formatów Open3D
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)

    # Ustawienie kolorów dla siatki (opcjonalnie)
    mesh.vertex_colors = o3d.utility.Vector3dVector([[0.8, 0.6, 0.7] for _ in range(len(vertices))])  # rozowe linie

    # Wygładzanie siatki (opcjonalnie)
    mesh.compute_vertex_normals()
    
    # Tworzenie obiektu PointCloud dla pierwotnych punktów
    original_points = o3d.geometry.PointCloud()
    original_points.points = o3d.utility.Vector3dVector(points_on_plane)

    # Ustawienie koloru dla pierwotnych punktów (opcjonalnie)
    original_colors = np.array([[0.5, 0.6, 0.6 ] for _ in range(len(points_on_plane))])  # szare punkty
    original_points.colors = o3d.utility.Vector3dVector(original_colors)

    # Zdefiniowanie rozmiaru punktów left, right i center
    point_size = 0.05

    # Utworzenie sfer dla punktów left, right i center
    left_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=point_size)
    left_sphere.compute_vertex_normals()
    left_sphere.paint_uniform_color([1, 0.5, 0])  # Pomarańczowy

    right_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=point_size)
    right_sphere.compute_vertex_normals()
    right_sphere.paint_uniform_color([1, 1, 0])  # Żółty

    center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=point_size)
    center_sphere.compute_vertex_normals()
    center_sphere.paint_uniform_color([1, 0, 1])  # Magenta

    depth_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=point_size)
    depth_sphere.compute_vertex_normals()
    depth_sphere.paint_uniform_color([0.8, 0.3, 1])  # Fioletowy

    # Przesunięcie sfer do odpowiednich pozycji
    left_sphere.translate(left_point)
    right_sphere.translate(right_point)
    center_sphere.translate(center_point)
    depth_sphere.translate(depth_point)

    # Wizualizacja
    # o3d.visualization.draw_geometries([mesh, left_sphere, right_sphere, center_sphere, depth_sphere])

    # Ustawienia wizualizacji
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(mesh)
    # vis.add_geometry(line_set)
    # vis.add_geometry(original_points)
    vis.add_geometry(left_sphere)
    vis.add_geometry(right_sphere)
    vis.add_geometry(center_sphere)
    vis.add_geometry(depth_sphere)

    # Włączenie renderowania tylnej strony mesha
    opt = vis.get_render_option()
    opt.mesh_show_back_face = True

    vis.run()
    vis.destroy_window()

def find_fit(arc_filepath, points_filepath):

    if arc_filepath.endswith(".las"):
        arc_o3d = las_to_o3d(arc_filepath)
    elif arc_filepath.endswith(".ply") or arc_filepath.endswith(".pcd"):
        arc_o3d: o3d.geometry.PointCloud = o3d.io.read_point_cloud(arc_filepath)
    arc_pts = np.asarray(arc_o3d.points)  

    if points_filepath.endswith(".las"):
        pts = las_to_o3d(points_filepath)
    elif points_filepath.endswith(".ply") or points_filepath.endswith(".pcd"):
        pts: o3d.geometry.PointCloud = o3d.io.read_point_cloud(points_filepath)

    arc_pts = np.asarray(arc_o3d.points) 

    left_point = np.asarray(pts.points)[0]
    right_point = np.asarray(pts.points)[1]
    center_point = np.asarray(pts.points)[2]
    depth_point = np.asarray(pts.points)[3]

    depth_dist = center_point[0] - depth_point[0]

    # Zdefiniuj granice wartości punktów dla filtracji
    upper_limit = center_point[2]
    lower_limit = np.min([left_point[2], right_point[2]])

    # Równanie płaszczyzny
    plane_equation = equation_plane(center_point[0], center_point[1], center_point[2], left_point[0], left_point[1], left_point[2], right_point[0], right_point[1], right_point[2])

    # Znalezienie punktów należących do płaszczyzny
    points_on_plane = []
    for point in arc_pts:
        if point_on_plane(plane_equation, point):
            points_on_plane.append(point)
            
    # Testowe zwiekszenie wagi punktow lewego, srodkowego i lewego
    for i in range(int(len(points_on_plane)/8)):
        points_on_plane.extend([left_point, center_point, right_point])
        
        
    points_on_plane = np.array(points_on_plane)
    points_on_plane = filter_points_by_limits(arc_pts, lower_limit, upper_limit)

    # Mapowanie punktów 3D do 2D
    # points_2d = map_to_2d(points_on_plane)

    # Przesunięcie wierzchołka wykresu 
    X_ = points_on_plane[:, 1].reshape(-1,1)
    X = X_ - center_point[1]
    y_ = points_on_plane[:, 2].reshape(-1,1)
    y = y_ - center_point[2]

    # Dopasowanie punktow 
    regressor = RANSAC(model=QuadraticRegressor(), loss=square_error_loss, metric=mean_square_error)
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

    visualize_points(left_point, right_point, center_point, depth_point, points_on_plane, line_points, depth_line_points)

if __name__ == "__main__":
    
    find_fit(r"C:\Users\julia\Documents\GEOINFORMATYKA\sem6\APFiWM\projekt_sem\luki\luk1.las", 
                r"C:\Users\julia\Documents\GEOINFORMATYKA\sem6\APFiWM\projekt_sem\luki\luk1_4pts_julka.las")