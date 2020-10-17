"""
2020年10月14日22:03:52

@author: Tu Kechao
"""

from cv2 import cv2
import numpy as np
import math,time,os,shutil
import open3d as o3d
from myMeshData import myMeshData




if __name__ == "__main__":
    a = o3d.io.read_point_cloud(r"E:\OneDrive\CS800Run\CompareScript\Data\PointCloud\mvsnet000_l3.ply")
    b =o3d.geometry.PointCloud.remove_statistical_outlier(a,4,0.1)
    o3d.io.write_point_cloud(r"E:\OneDrive\CS800Run\CompareScript\Data\PointCloud\mvsnet000_l3_2.ply",b[0],write_ascii=1)