from functions.arch_cutter import cut_arch
from functions.arch_fitter import fit_arch
from functions.point_picker import pick_point

menu_message = ["Co chcesz zrobić?",
                "0. Wyjść.",
                "1. Wyciąć łuk z chmury punktów.",
                "2. Wczytać łuk z pliku.",
                "3. Wybrać punkt śrokowy łuku z chmury punktów.",
                "4. Wczytać punkt środkowy z pliku.",
                "5. Wpasować płaszczyznę w łuk (najpierw wczytaj łuk i punkt środkowy)", ]

exit_commands = ["e", "exit", "leave", "q", "quit", "end", "finish"]

if __name__ == "__main__":
    while True:
        print("", *menu_message, sep="\n")

        try:
            typed = input()
            if typed.lower() in exit_commands:
                break
            elif typed.lower() == "1":
                cut_arch()
            elif typed.lower() == "2":
                pass
            elif typed.lower() == "3":
                pick_point()
            elif typed.lower() == "4":
                pass
            elif typed.lower() == "5":
                fit_arch()
            else:
                print("Nieznana komenda")
        except:
            print("Wystąpił błąd")
