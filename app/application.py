import open3d as o3d

from app.common.file_dialogs import get_cloud_filename
from functions.arch_cutter import cut_arch
from functions.arch_fitter import fit_arch
from functions.point_picker import pick_point

if __name__ == "__main__":
    exit_commands = ["e", "exit", "leave", "q", "quit", "end", "finish"]

    arch: o3d.geometry.PointCloud = None
    key_points: o3d.geometry.PointCloud = None

    menu_message = ["Co chcesz zrobić?",
                    "0. Wyjść.",
                    "1. Wyciąć łuk z chmury punktów.",
                    "2. Wczytać łuk z pliku.",
                    "3. Wybrać 4 punkty kluczowe łuku z chmury punktów.",
                    "4. Wczytać 4 punkty kluczowe łuku z pliku.",
                    "5. Wpasować płaszczyznę w łuk. \n Najpierw załaduj łuk [" + (
                        "❌" if arch is None else "✔") + "] i 4 punkty kluczowe [" + (
                        "❌" if key_points is None else "✔") + "]"]

    while True:
        menu_message[6] = "5. Wpasować płaszczyznę w łuk. \n Najpierw załaduj łuk [" + (
            "❌" if arch is None else "✔") + "] i 4 punkty kluczowe łuku [" + ("❌" if key_points is None else "✔") + "]"
        print("", *menu_message, sep="\n")

        try:
            typed = input()
            if typed.lower() in exit_commands:
                break
            elif typed.lower() == "1":
                cut_arch()
            elif typed.lower() == "2":
                filename: str = get_cloud_filename()
                arch: o3d.geometry.PointCloud = o3d.io.read_point_cloud(filename)
            elif typed.lower() == "3":
                pick_point()
            elif typed.lower() == "4":
                filename: str = get_cloud_filename()
                key_points: o3d.geometry.PointCloud = o3d.io.read_point_cloud(filename)
                if len(key_points.points) != 4:
                    key_points = None
                    print(
                        "Plik z 4 punktami kluczowymi łuku powinien zawierać dokładnie cztery punkty. Sprawdź czy wybrano odpowiedni plik.")
                    raise RuntimeError("Key points should have exactly four points.")
            elif typed.lower() == "5":
                if key_points is None or arch is None:
                    print("4 punkty kluczowe łuku i łuk muszą zostać najpierw wczytane.")
                    raise RuntimeError("Key points and arch should not be 'None'.")
                fit_arch(key_points, arch)
            else:
                print("Nieznana komenda")
        except:
            print("Wystąpił błąd")
