import open3d 
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import laspy
import copy
import matplotlib as plt

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    open3d.visualization.draw_geometries([source_temp, target_temp])

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        open3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = open3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        open3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size, source, target):
    print(":: Load two point clouds and disturb initial pose.")
    trans_init = np.identity(4)
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    visualize_fpfh_features(source_down, source_fpfh, "Source FPFH Features")
    visualize_fpfh_features(target_down, target_fpfh, "Target FPFH Features")
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def visualize_fpfh_features(pcd_down, fpfh, window_name):
    # Visualize the point cloud colored based on FPFH features.
    print(f":: Visualizing {window_name}.")

    # We will use the magnitudes of FPFH features (specifically the first feature in the FPFH) to color the points.
    fpfh_data = np.asarray(fpfh.data).T  # Transpose to match point cloud data shape

    # Get the first feature from FPFH as an example for coloring
    feature_magnitudes = np.linalg.norm(fpfh_data, axis=1)

    # Normalize feature magnitudes to range [0, 1] for color mapping
    feature_magnitudes = (feature_magnitudes - np.min(feature_magnitudes)) / (np.max(feature_magnitudes) - np.min(feature_magnitudes))

    # Map feature magnitudes to colors (using colormap)
    colors = plt.cm.jet(feature_magnitudes)[:, :3]  # Use 'jet' colormap and extract RGB values

    # Apply the colors to the point cloud
    pcd_down.colors = open3d.utility.Vector3dVector(colors)

    # Visualize
    open3d.visualization.draw_geometries([pcd_down], window_name=window_name)


point_cloud = open3d.io.read_point_cloud("open3d.ply")
source =point_cloud
target=open3d.io.read_point_cloud("open3d_shifted.ply")
threshold=1
source=source.voxel_down_sample(0.25)
target=target.voxel_down_sample(0.25)
trans_init = np.identity(4)


voxel_size = 0.05  # means 5cm for this dataset
source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
    voxel_size,source,target)
