from functions.arch_cutter import cut_arch
from functions.arch_fitter import fit_arch

menu_message = ["Co chcesz zrobić?",
                "1. Wyciąć łuk z chmury punktów.",
                "2. Wczytać łuk z pliku i wpasować w niego krzywą.",
                "3. Wyjść."]

exit_commands = ["e", "exit", "leave", "q", "quit", "end", "finish", "3"]


if __name__ == "__main__":
    while True:
        print("", *menu_message, sep="\n")

        typed = input()
        if typed.lower() in exit_commands:
            exit(0)
        elif typed.lower() == "1":
            cut_arch()
        elif typed.lower() == "2":
            fit_arch()
        else:
            print("Nieznana komenda")
