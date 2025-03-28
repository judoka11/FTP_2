import open3d 
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import laspy
import copy
import matplotlib as plt


def draw_result(source, target,transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    open3d.visualization.draw_geometries([source_temp, target_temp])

def pomiar_punktow_na_chmurze(chmura_punktow):
    print("Pomiar punktów na chmurze punktów")
    print("Etapy pomiaru punktów: ")
    print(" (1.1) Pomiar punktu - shift + lewy przycisk myszy")
    print(" (1.2) Cofniecie ostatniego pomiaru - shift + prawy przycisk myszy")
    print(" (2) Koniec pomiaru - wciśnięcie klawisza Q")
    vis = open3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name='Pomiar punktów')
    vis.add_geometry(chmura_punktow)
    vis.run() # user picks points
    vis.destroy_window()
    print("Koniec pomiaru")
    print(vis.get_picked_points())
    return vis.get_picked_points()

#Orientacja metodą Target-based
def wyswetalnie_par_chmur_punktow(chmura_referencyjna,chmura_orientowana,transformacja):
    ori_temp = copy.deepcopy(chmura_orientowana)
    ref_temp = copy.deepcopy(chmura_referencyjna)
    ori_temp.paint_uniform_color([1, 0, 0])
    ref_temp.paint_uniform_color([0, 1, 0])
    ori_temp.transform(transformacja)
    open3d.visualization.draw_geometries([ori_temp, ref_temp])

def orientacja_target_based(chmura_referencyjna, chmura_orientowana, typ = 'Pomiar',Debug = 'False'):
    print('Orientacja chmur punktów metoda Target based')
    wyswetalnie_par_chmur_punktow (chmura_referencyjna, chmura_orientowana,np.identity(4))
    if typ != 'File':
        print('Pomierz min. 3 punkty na chmurze referencyjnej: ')
        pkt_ref = pomiar_punktow_na_chmurze(chmura_referencyjna)
        print('Pomierz min. 3 punkty orientowanej ')
        pkt_ori = pomiar_punktow_na_chmurze(chmura_orientowana)
    elif typ == 'Plik':
        print('Wyznaczenia parametrów transformacji na podstawie punktów pozyskanych z plików tekstowych')
        #Wczytanie chmur punktów w postaci plików tekstowych
        #Przygotowanie plików ref i ori
    else: #Inna metoda
        print('Wyznaczenie parametrów na podstawie analizy deskryptorów')
        #Analiza deskryptorów
        assert (len(pkt_ref) >= 3 and len(pkt_ori) >= 3)
        assert (len(pkt_ref) == len(pkt_ori))
    corr = np.zeros((len(pkt_ori), 2))
    corr[:, 0] = pkt_ori
    corr[:, 1] = pkt_ref
    print(corr)
    p2p = open3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans = p2p.compute_transformation(chmura_referencyjna,
    chmura_orientowana,open3d.utility.Vector2iVector(corr))
    if Debug == 'True':
        print(trans)
        wyswetalnie_par_chmur_punktow(chmura_orientowana,chmura_referencyjna,trans)
        #analiza_statystyczna(chmura_referencyjna, chmura_orientowana,trans)
    return(trans)


point_cloud = open3d.io.read_point_cloud("dim_normals.ply")
moved_point_cloud =copy.deepcopy(point_cloud).translate((50,50, 0))


point_cloud_fpfh = open3d.pipelines.registration.compute_fpfh_feature(
    point_cloud,    
    open3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=100))

moved_point_cloud_fpfh = open3d.pipelines.registration.compute_fpfh_feature(
    moved_point_cloud,
    open3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=100))

# result = open3d.pipelines.registration.registration_ransac_based_on_feature_matching(
#         point_cloud, moved_point_cloud, point_cloud_fpfh, moved_point_cloud_fpfh,
#         mutual_filter=True,
#         max_correspondence_distance=0.1,
#         estimation_method=open3d.pipelines.registration.TransformationEstimationPointToPoint(False),
#         ransac_n=3,
#         checkers=[
#             open3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
#             open3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.1)
#         ],
#         criteria=open3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
#     )

result=orientacja_target_based(point_cloud,moved_point_cloud)
#wyswetalnie_par_chmur_punktow(point_cloud,moved_point_cloud,result)
draw_result(point_cloud,moved_point_cloud,result)