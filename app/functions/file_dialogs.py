import tkinter
from tkinter import filedialog
from typing import Iterable
from plyer import filechooser


def get_cloud_filename(title: str = "Open cloud file") -> str:
    tkinter.Tk().withdraw()  # prevents an empty tkinter window from appearing

    # filename: str = filedialog.askopenfilename(title=title,
    #                                            initialdir="/home/tymon/Documents/gits/bearing/dane",
    #                                            filetypes=(("pcd", "*.pcd"), ("ply", "*.ply")))
    filename: str = filechooser.open_file(multiple = False, filters = ['*.pcd', '*.ply'])[0]
    return filename


def get_savefile_path(filetypes: Iterable[tuple[str, str]] = (("pcd", "*.pcd"), ("ply", "*.ply")),
                      title: str = "Save file") -> str:
    tkinter.Tk().withdraw()

    filename: str = filedialog.asksaveasfilename(title=title,
                                                 initialdir="/home/tymon/Documents/gits/bearing/dane",
                                                 filetypes=filetypes)
    return filename
