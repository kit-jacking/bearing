import tkinter

import open3d as o3d

from app.functions.file_dialogs import get_cloud_filename


def manual_pcd_croppings(point_cloud):
    message = "1. Naciśnij 'X' dwukrotnie, żeby wyrównać obraz w osi x.\n2. Naciśnij 'Y' dwukrotnie, żeby wyrównać obraz w osi y.\n3. Naciśnij 'Z' dwukrotnie, żeby wyrównać obraz w osi z.4. Naciśnij 'K' żeby przejść do trybu edycji.\n 4.1 Przeciągnij myszką, żeby wybrać prostokąt.\n 4.2 Użyj ctrl + lpm żeby wybrać wielokątem.\n5. Naciśnij 'C' żeby wyciąć wybraną geometrię.\n6. Naciśnij 'F', żeby przejść do wolnego widoku.\n7. Naciśnij 'S', żeby zapisać.\n8. Naciśnij 'Q', żeby wyjść."
    tkinter.messagebox.showinfo(title='Instrukcja do wycinania łuków', message=message)

    o3d.visualization.draw_geometries_with_editing([point_cloud], window_name='Wycinanie łuków')
    return point_cloud


def cut_arch() -> None:
    filename: str = get_cloud_filename()
    cloud: o3d.geometry.PointCloud = o3d.io.read_point_cloud(filename)

    manual_pcd_croppings(cloud)


if __name__ == "__main__":
    cut_arch()
