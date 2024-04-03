import tkinter
from tkinter import filedialog


def get_cloud_filename() -> str:
    tkinter.Tk().withdraw()  # prevents an empty tkinter window from appearing

    filename: str = filedialog.askopenfilename(initialdir="/home/tymon/Documents/gits/bearing/dane",
                                               filetypes=(("pcd", "*.pcd"), ("ply", "*.ply")))
    return filename
