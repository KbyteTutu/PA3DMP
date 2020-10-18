"""
2020年10月14日22:03:52

@author: Tu Kechao
"""

from cv2 import cv2
import numpy as np
import math,time,os,shutil
import open3d as o3d
from MeshData import MeshData

class Pa3dmp(object):
    def resizePointCloud(self,PointCloudPath):
        #Resize from center. The import ply was generated from image which biliner resized (1152, 864) from (2048,1536)
        ply = o3d.io.read_point_cloud(PointCloudPath)
        out = ply.scale(1.7778,(0,0,0))
        # o3d.io.write_point_cloud(r"E:\OneDrive\CS800Run\PA3DMP\Data\PointCloud\mvsnet000_l3_scale.ply",out,write_ascii=1)
        return out

    def optimizePointCloud(self,pointCloud):
        # TODO: some how optimize the point cloud
        return pointCloud

    def P2FDistance(self,P,Face):
        # Point To Face Distance
        point1 = np.asarray(Face[0])
        point2 = np.asarray(Face[1])
        point3 = np.asarray(Face[2])
        point4 = np.asarray(P)
        AB = np.asmatrix(point2 - point1)
        AC = np.asmatrix(point3 - point1)
        N = np.cross(AB, AC)  # 向量叉乘，求法向量
        # Ax+By+Cz
        Ax = N[0, 0]
        By = N[0, 1]
        Cz = N[0, 2]
        D = -(Ax * point1[0] + By * point1[1] + Cz * point1[2])
        mod_d = Ax * point4[0] + By * point4[1] + Cz * point4[2] + D
        mod_area = np.sqrt(np.sum(np.square([Ax, By, Cz])))
        d = abs(mod_d) / mod_area

        return d



if __name__ == "__main__":
    # a = o3d.io.read_point_cloud(r"E:\OneDrive\CS800Run\CompareScript\Data\PointCloud\mvsnet000_l3.ply")
    # b =o3d.geometry.PointCloud.remove_statistical_outlier(a,4,0.1)
    # o3d.io.write_point_cloud(r"E:\OneDrive\CS800Run\CompareScript\Data\PointCloud\mvsnet000_l3_2.ply",b[0],write_ascii=1)
    instance = Pa3dmp()
    # instance.resizePointCloud(r"E:\OneDrive\CS800Run\PA3DMP\Data\PointCloud\mvsnet000_l3.ply")

    a = [2, 3, 1]
    b = [4, 1, 2]
    c = [6, 3, 7]
    p = [-5, -4, 8]

    Face = [a,b,c]
    print(instance.P2FDistance(p,Face))
