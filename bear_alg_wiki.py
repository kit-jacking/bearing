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
    
    user_picked_pts = np.asarray(pt_6.points)
    print("user picked: ", user_picked_pts)
    

    user_X = user_picked_pts[:,1].reshape(-1,1)
    user_Y = user_picked_pts[:,2].reshape(-1,1)
    
    new_X = new_X - user_X.mean()
    new_Y = new_Y - user_Y.max()

    
    # # visualize_point_cloud(o3d_load_point_cloud(r"C:\Users\qattr\Desktop\STUD\SEM 6\FTP2\otwock_freski18_n.las"))
    regressor = RANSAC(model=LinearRegressor(),loss=square_error_loss, metric=mean_square_error)

    regressor.fit(new_X, new_Y)

    import matplotlib.pyplot as plt
    ax = plt.figure().add_subplot(projection='3d')

    # Scatter punktow z list X i y
    ax.scatter(new_X, new_Y, zs=0, zdir='z')

    line = np.linspace(new_X.min(), new_X.max(), num=100).reshape(-1, 1)
    ax.plot(line, regressor.predict(line),zs=0, zdir='z', c="peru")
    plt.show()