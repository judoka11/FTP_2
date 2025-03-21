import open3d 
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import laspy

point_cloud = open3d.io.read_point_cloud("ROZALSKA-DYPLOM-JOZE-ZEWNATRZ- Zewnatrz-10.ply")

downpcd = point_cloud.voxel_down_sample(voxel_size=0.05)

open3d.visualization.draw_geometries([downpcd])


open3d.io.write_point_cloud("chmura_ziemia_5cm.ply", downpcd)