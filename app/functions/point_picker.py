import tkinter.messagebox

import numpy as np
import open3d as o3d

from app.functions.file_dialogs import get_cloud_filename, get_savefile_path


def manual_point_picking(point_cloud: o3d.geometry.PointCloud):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name='Manual point picking')
    vis.add_geometry(point_cloud)
    vis.run()
    vis.destroy_window()
    return vis.get_picked_points()


def pick_point() -> None:
    radius = 0.05
    color = [1, 0, 0]
    how_many_points_you_wish_to_pick_good_sir_or_madam = 4

    tkinter.messagebox.showinfo(title='Informacja', message="Wybierz chmurę punktów łuku.")
    filename: str = get_cloud_filename()
    tkinter.messagebox.showinfo(title='Informacja', message="Gdzie zapisać wybrany punkt.")
    filetypes = ['.pcd', '.ply']
    save_filename: str = get_savefile_path()
    if save_filename == "" or not any([save_filename.endswith(filetype) for filetype in filetypes]):
        print(f"Aby zapisać należy wybrać plik do zapisu {filetypes}. Przerywanie.")
        return
    message = "Wybierz punkt.\n1. Naciśnij shift + lpm, żeby wybrać punkt.\n2.Naciśnij shift + ppm, żeby cofnąć wybranie punktu.\n3. Naciśnij 'Q' po wybraniu punktu, żeby zakończyć.\n"
    tkinter.messagebox.showinfo(title='Instrukcja do wybierania punktów', message=message)
    message = ("!UWAGA!\n Punkty należy wybrać \n\n!!PO JEDNYM!!\n\n w ściśle określonej klejności! W przeciwnym wypadku łuk zostanie źle wpasowany.\n\n1. Punkt na dole łuku, przy końcu z jednej strony. Tworzy płaszczyznę z punktami 2 i 3.\n\n2. Punkt z drugiej strony łuku, również na dole, w tej samej płaszczyźnie co punkty 1 i 3.\n\n 3. Punkt środkowy u szczytu łuku, w tej samej płaszczyźnie co punkty 1 i 2.\n\n 4. Punkt u szczytu łuku, po drugiej stronie łuku niż punkt 3. Punkt 4 odpowiada za głębię.\n\n Pamiętaj wybrać punkty dokładnie w tej kolejności i o tym, że możesz cofać używając shift + ppm.\n\n Powodzenia!")
    tkinter.messagebox.showwarning(title='Instrukcja do wybierania punktów', message=message)
    cloud: o3d.geometry.PointCloud = o3d.io.read_point_cloud(filename)

    spheres = [None] * how_many_points_you_wish_to_pick_good_sir_or_madam
    points_indexes = [None] * how_many_points_you_wish_to_pick_good_sir_or_madam
    points_clouds = [None] * how_many_points_you_wish_to_pick_good_sir_or_madam
    points_nps = [None] * how_many_points_you_wish_to_pick_good_sir_or_madam

    def pick_point_with_index(index: int):
        temp = (manual_point_picking(cloud))
        if len(temp) != 1:
            print(
                f"Aby zapisać należy wybrać dokładnie 1 punkt, a nie {len(temp)}. Dlatego wybieramy po prostu pierwszy wybrany. Uważaj!")
            temp = temp[:1]
        points_indexes[index] = temp
        points_clouds[index] = (cloud.select_by_index(points_indexes[index]))
        points_nps[index] = (np.asarray(points_clouds[index].points)[0])
        print(f"Point {i}: {points_nps[index]}")

        sphere = o3d.geometry.TriangleMesh.create_sphere(radius)
        sphere.paint_uniform_color(color)
        sphere.compute_vertex_normals()
        sphere = sphere.translate(points_nps[index])
        spheres[index] = (sphere)

    for i in range(how_many_points_you_wish_to_pick_good_sir_or_madam):
        pick_point_with_index(i)

    o3d.visualization.draw_geometries([*spheres, cloud])

    save_cloud = o3d.geometry.PointCloud()
    save_cloud.points = o3d.utility.Vector3dVector(points_nps)

    o3d.io.write_point_cloud(save_filename, save_cloud, write_ascii=False, compressed=False,
                             print_progress=False)


if __name__ == "__main__":
    pick_point()
