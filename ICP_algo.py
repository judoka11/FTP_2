import open3d 
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import laspy
import copy


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    open3d.visualization.draw_geometries([source_temp, target_temp],)

point_cloud = open3d.io.read_point_cloud("open3d.ply")
source =point_cloud
target=open3d.io.read_point_cloud("open3d_shifted.ply")
threshold=1
source=source.voxel_down_sample(0.25)
target=target.voxel_down_sample(0.25)
trans_init = np.identity(4)

source.estimate_normals(
        search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=6))
target.estimate_normals(
        search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=6))

# print("Apply point-to-point ICP")
# reg_p2p = open3d.pipelines.registration.registration_icp(
#     source, target, threshold,trans_init,
#     open3d.pipelines.registration.TransformationEstimationPointToPoint())
# print(reg_p2p)
# print("Transformation is:")
# print(reg_p2p.transformation)
# draw_registration_result(source, target, reg_p2p.transformation)

print("Apply point-to-plane ICP")
reg_p2l = open3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    open3d.pipelines.registration.TransformationEstimationPointToPlane())
print(reg_p2l)
print("Transformation is:")
print(reg_p2l.transformation)
draw_registration_result(source, target, reg_p2l.transformation)