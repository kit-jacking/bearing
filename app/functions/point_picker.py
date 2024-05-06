import tkinter.messagebox

import open3d as o3d

from app.common.file_dialogs import get_cloud_filename, get_savefile_path


def manual_point_picking(point_cloud: o3d.geometry.PointCloud):
    message = "Wybierz punkt.\n1. Naciśnij shift + lpm, żeby wybrać punkt.\n2.Naciśnij shift + ppm, żeby cofnąć wybranie punktu.\n3. Naciśnij 'Q', żeby zakończyć i zapisać.\n"
    tkinter.messagebox.showinfo(title='Instrukcja do wybierania punktów', message=message)
    message = ("!UWAGA!\n Punkty należy wybrać w ściśle określonej klejności! W przeciwnym wypadku łuk zostanie źle wpasowany.\n\n1. Punkt na dole łuku, przy końcu z jednej strony. Tworzy płaszczyznę z punktami 2 i 3.\n2. Punkt z drugiej strony łuku, również na dole, w tej samej płaszczyźnie co putnk 1 i 3.\n 3. Punkt środkowy u szczytu łuku, w tej samej płaszczyźnie co punkty 1 i 2.\n 4. Punkt u szczytu łuku, po drugiej stronie łuku niż punkt 3. Punkt 4 odpowiada za głębię.\n Pamiętaj wybrać punkty dokładnie w tej kolejności i o tym, że możesz cofać używając shift + ppm.\n\n Powodzenia!")
    tkinter.messagebox.showwarning(title='Instrukcja do wybierania punktów', message=message)

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
    if len(points_indexes) != 4:
        print(f"Aby zapisać należy wybrać dokładnie 4 punkty, a nie {len(points_indexes)}. Przerywanie.")
        return

    # save_filename = get_savefile_path()
    if save_filename == "" or not any([save_filename.endswith(filetype) for filetype in filetypes]):
        print(f"Aby zapisać należy wybrać plik do zapisu {filetypes}. Przerywanie.")
        return

    o3d.io.write_point_cloud(save_filename, cloud.select_by_index(points_indexes), write_ascii=False, compressed=False,
                             print_progress=False)


if __name__ == "__main__":
    pick_point()
