from copy import copy
import numpy as np
from numpy.random import default_rng
import open3d as o3d
import laspy

rng = default_rng()

# Czesc z wikipedii
class RANSAC:
    def __init__(self, n=50000, k=1000, t=0.01, d=100, model=None, loss=None, metric=None):
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
        X = np.hstack([np.ones((r, 1)), -np.cosh(X)]) # rysunek z zeszytu
        # print(X)
        self.params = np.linalg.inv(X.T @ X) @ X.T @ y
        return self

    def predict(self, X: np.ndarray):
        r, _ = X.shape
        X = np.hstack([np.ones((r, 1)), -np.cosh(X)]) # rysunek z zeszytu
        return X @ self.params


# Wczytanie chmury
def laspy_load_las_file(path):
    print('Wczytywanie chmury...')
    las_pcd = laspy.file.File(path, mode='rw')
    
    print(type(las_pcd.x))
    print(las_pcd.points)
    X = las_pcd.x
    Y = las_pcd.y
    Z = las_pcd.z
    las_points_xyz = np.vstack((X, Y, Z)).transpose() 
    
    return las_points_xyz, las_pcd, X, Y, Z

# Wyswietlanie chmury
def o3d_load_point_cloud(path):
    las_xyz, las_pcd, x, y, z = laspy_load_las_file(path)
    
    # Normalizacja kolorów
    r = las_pcd.intensity/max(las_pcd.intensity)
    g = r
    b = r
    las_colors = np.vstack((r,g,b)).transpose()
    
    # Utworzenie chmury punktów
    pt_cloud = o3d.geometry.PointCloud()
    pt_cloud.points = o3d.utility.Vector3dVector(las_xyz)
    pt_cloud.colors = o3d.utility.Vector3dVector(las_colors) 
    print(pt_cloud)
    return pt_cloud
    

def visualize_point_cloud(point_cloud):
    o3d.visualization.draw_geometries([point_cloud])
    
# nietestowane
def o3d_choose_points(pt_cloud):
    print("Pomiar punktów na chmurze punktów")
    print("Etapy pomiaru punktów: ")
    print(" (1.1) Pomiar punktu - shift + lewy przycisk myszy")
    print(" (1.2) Cofniecie ostatniego pomiaru - shift + prawy przycisk myszy")
    print(" (2) Koniec pomiaru - wciśnięcie klawisza Q")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name='Wybór punktów')
    vis.add_geometry(pt_cloud)
    vis.run() # user picks points
    vis.destroy_window()
    print("Koniec pomiaru")
    print(vis.get_picked_points())
    return vis.get_picked_points()


if __name__ == "__main__":
    
    pt_cld = o3d_load_point_cloud(r"C:\Users\qattr\Desktop\STUD\SEM 6\FTP2\luk3.las")
    pt_6 = o3d_load_point_cloud(r"C:\Users\qattr\Desktop\STUD\SEM 6\FTP2\luk3_6punktow.las")
    np_arr_pts = np.asarray(pt_cld.points)
    new_arr = np_arr_pts[:,:2]
    print(new_arr)
    new_X = np_arr_pts[:,1].reshape(-1,1)
    new_Y = np_arr_pts[:,2].reshape(-1,1)
    
    new_X = new_X - new_X.mean()
    new_Y = new_Y - new_Y.max()
    print(new_Y)
    
    # user_X = 
    # user_Y = 
    
    
    
    
    
    
    
    # # visualize_point_cloud(o3d_load_point_cloud(r"C:\Users\qattr\Desktop\STUD\SEM 6\FTP2\otwock_freski18_n.las"))
    regressor = RANSAC(model=LinearRegressor(),loss=square_error_loss, metric=mean_square_error)

    X = np.array([-0.848,-0.800,-0.704,-0.632,-0.488,-0.472,-0.368,-0.336,-0.280,-0.200,-0.00800,-0.0840,0.0240,0.100,0.124,0.148,0.232,0.236,0.324,0.356,0.368,0.440,0.512,0.548,0.660,0.640,0.712,0.752,0.776,0.880,0.920,0.944,-0.108,-0.168,-0.720,-0.784,-0.224,-0.604,-0.740,-0.0440,0.388,-0.0200,0.752,0.416,-0.0800,-0.348,0.988,0.776,0.680,0.880,-0.816,-0.424,-0.932,0.272,-0.556,-0.568,-0.600,-0.716,-0.796,-0.880,-0.972,-0.916,0.816,0.892,0.956,0.980,0.988,0.992,0.00400]).reshape(-1,1)
    # ## ogyginalne - z wiki wartosci liniowe
    # print(X)
    Y = np.array([-0.917,-0.833,-0.801,-0.665,-0.605,-0.545,-0.509,-0.433,-0.397,-0.281,-0.205,-0.169,-0.0531,-0.0651,0.0349,0.0829,0.0589,0.175,0.179,0.191,0.259,0.287,0.359,0.395,0.483,0.539,0.543,0.603,0.667,0.679,0.751,0.803,-0.265,-0.341,0.111,-0.113,0.547,0.791,0.551,0.347,0.975,0.943,-0.249,-0.769,-0.625,-0.861,-0.749,-0.945,-0.493,0.163,-0.469,0.0669,0.891,0.623,-0.609,-0.677,-0.721,-0.745,-0.885,-0.897,-0.969,-0.949,0.707,0.783,0.859,0.979,0.811,0.891,-0.137]).reshape(-1,1)
    
    # # # wygenerowane x**2
    # y_values = -(X)**2
    # y = np.array(y_values).reshape(-1,1)

    regressor.fit(new_X, new_Y)

    import matplotlib.pyplot as plt
    ax = plt.figure().add_subplot(projection='3d')

    # Scatter punktow z list X i y
    ax.scatter(new_X, new_Y, zs=0, zdir='z')

    line = np.linspace(new_X.min(), new_X.max(), num=100).reshape(-1, 1)
    ax.plot(line, regressor.predict(line),zs=0, zdir='z', c="peru")
    plt.show()