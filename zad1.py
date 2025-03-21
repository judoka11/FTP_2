import open3d 
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import laspy


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    open3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

point_cloud = open3d.io.read_point_cloud("open3d.ply")
downpcd = point_cloud.voxel_down_sample(voxel_size=0.1)
cl, ind =downpcd.remove_radius_outlier(nb_points=100, radius=0.5)
open3d.visualization.draw_geometries([cl])
display_inlier_outlier(downpcd, ind)