import open3d as o3d

from app.common.file_dialogs import get_cloud_filename
from functions.arch_cutter import cut_arch
from functions.arch_fitter import fit_arch
from functions.point_picker import pick_point

if __name__ == "__main__":
    exit_commands = ["e", "exit", "leave", "q", "quit", "end", "finish", "3"]

    arch: o3d.geometry.PointCloud = None
    middle_point: o3d.geometry.PointCloud = None

    menu_message = ["Co chcesz zrobić?",
                    "0. Wyjść.",
                    "1. Wyciąć łuk z chmury punktów.",
                    "2. Wczytać łuk z pliku.",
                    "3. Wybrać punkt śrokowy łuku z chmury punktów.",
                    "4. Wczytać punkt środkowy z pliku.",
                    "5. Wpasować płaszczyznę w łuk. \n Najpierw załaduj łuk [" + (
                        "❌" if arch is None else "✔") + "] i punkt środkowy [" + (
                        "❌" if middle_point is None else "✔") + "]"]

    while True:
        menu_message[6] = "5. Wpasować płaszczyznę w łuk. \n Najpierw załaduj łuk [" + (
            "❌" if arch is None else "✔") + "] i punkt środkowy [" + ("❌" if middle_point is None else "✔") + "]"
        print("", *menu_message, sep="\n")

        try:
            typed = input()
            if typed.lower() in exit_commands:
                exit(0)
            elif typed.lower() == "1":
                cut_arch()
            elif typed.lower() == "2":
                filename: str = get_cloud_filename()
                arch: o3d.geometry.PointCloud = o3d.io.read_point_cloud(filename)
            elif typed.lower() == "3":
                pick_point()
            elif typed.lower() == "4":
                filename: str = get_cloud_filename()
                middle_point: o3d.geometry.PointCloud = o3d.io.read_point_cloud(filename)
                if len(middle_point.points) != 1:
                    middle_point = None
                    print("Plik z punktem środkowym powinien zawierać dokładnie jeden punkt. Sprawdź czy wybrano odpowiedni plik.")
                    raise RuntimeError("Middle point should have exactly one point.")
            elif typed.lower() == "5":
                if middle_point is None or arch is None:
                    print("Punkt środkowy i łuk muszą zostać najpierw wczytane.")
                    raise RuntimeError("Middle point and arch should not be None.")
                fit_arch()
            else:
                print("Nieznana komenda")
        except:
            print("Wystąpił błąd")
