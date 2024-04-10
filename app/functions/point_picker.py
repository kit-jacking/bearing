import tkinter.messagebox

import re

import open3d as o3d

from app.common.file_dialogs import get_cloud_filename, get_savefile_path


def manual_point_picking(point_cloud: o3d.geometry.PointCloud):
    message = "Wybierz punkt.\n1. Naciśnij shift + lpm, żeby wybrać punkt.\n2.Naciśnij shift + ppm, żeby cofnąć wybranie punktu.\n4. Naciśnij 'Q', żeby zakończyć i zapisać."
    tkinter.messagebox.showinfo(title='Instrukcja do wybierania punktów', message=message)

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name='Manual point picking')
    vis.add_geometry(point_cloud)
    vis.run()  # user picks points
    vis.destroy_window()
    return vis.get_picked_points()


def pick_point() -> None:
    tkinter.messagebox.showinfo(title='Informacja', message="Wybierz chmurę punktów łuku.")
    filename: str = get_cloud_filename()
    tkinter.messagebox.showinfo(title='Informacja', message="Gdzie zapisać wybrany punkt.")
    filetypes = ['.pcd', '.ply']
    save_filename: str = get_savefile_path()
    cloud: o3d.geometry.PointCloud = o3d.io.read_point_cloud(filename)

    points_indexes = manual_point_picking(cloud)
    if len(points_indexes) != 1:
        print(f"Aby zapisać należy wybrać dokładnie 1 punkt, a nie {len(points_indexes)}. Przerywanie.")
        return

    if save_filename != "" and not bool(re.match(re.compile(".*\..*$"), save_filename)):
        print(f"Z przyczyn niezrozumiałych niedodało się rozszerzenie. Rektyfikuje.")
        save_filename += '.pcd'
    elif save_filename == "" or not any([save_filename.endswith(filetype) for filetype in filetypes]):
        print(f"Aby zapisać należy wybrać plik do zapisu {filetypes}. Przerywanie.")
        return


    o3d.io.write_point_cloud(save_filename, cloud.select_by_index(points_indexes), write_ascii=False, compressed=False,
                             print_progress=False)


if __name__ == "__main__":
    pick_point()
