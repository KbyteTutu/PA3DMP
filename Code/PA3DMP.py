"""
2020年10月14日22:03:52

@author: Tu Kechao
"""

from cv2 import cv2
import numpy as np
import math,time,os,shutil,copy
import open3d as o3d
from tqdm import tqdm
from  UtilityFunctions import Util
from My3DData import *
import matplotlib.pyplot as plt
import seaborn as sns

class Pa3dmp(object):

    def p2fDistance(self,P,p1,p2,p3):
        # Point To Face Distance
        point1 = np.asarray(p1)
        point2 = np.asarray(p2)
        point3 = np.asarray(p3)
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

    def loopForPoints(self,pointCloud,triangleMesh,radius):
        pointAmount = len(pointCloud.points)
        re = np.zeros((0,13))
        # build tree for search
        meshKDtree = o3d.geometry.KDTreeFlann(triangleMesh)
        verticesArr = np.asarray(triangleMesh.vertices)
        #simple timer to estimate the process time
        for pidx in range(len(pointCloud.points)):
            p = pointCloud.points[pidx]
            # m is the idx of all near points.knn is slow an
            # m = meshKDtree.search_radius_vector_3d(p,radius)
            # m = meshKDtree.search_knn_vector_3d(p,15)
            m = meshKDtree.search_hybrid_vector_3d(p,radius,10)
            if m[0] >7:
                # create a box for m, to get sub mesh
                pointsInBox = np.take(verticesArr,m[1],axis=0)
                #box = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pointsInBox))
                box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pointsInBox))
                #no need orientedboundingbox.
                #get sub mesh
                n = triangleMesh.crop(box)
                nTriangles = np.asarray(n.triangles)
                #temp flags
                d = float('inf')
                cur = np.zeros((1,9))
                for pidxSmall in range(len(nTriangles)):
                    idx = nTriangles[pidxSmall]
                    p1= n.vertices[idx[0]]
                    p2= n.vertices[idx[1]]
                    p3= n.vertices[idx[2]]
                    dTemp = self.p2fDistance(p,p1,p2,p3)
                    if dTemp<d:
                        d = dTemp
                        # cur = np.asarray([p1[0],p1[1],p1[2],p2[0],p2[1],p2[2],p3[0],p3[1],p3[2]])
                        cur = np.hstack((p1,p2,p3))
                #write to re
                if d!=float('inf'):
                    temp =np.hstack((p,cur,d)).reshape((1,13))
                    re = np.append(re,temp,axis=0)
            # To stop earlier
            # if pidx == 500:
            #     break
        return re

    @staticmethod
    def compare(pointCloud,mesh):
        #对比点云和三角网络，原理是把三角网络改成均匀点云，然后输出一个点云来看。
        pc1 = pointCloud
        pc2 = mesh.sample_points_uniformly(len(pc1.points),use_triangle_normal=False,seed=-1)
        pc1 = pc1.paint_uniform_color([1,0,0])
        pc2 = pc2.paint_uniform_color([0,0,1])
        pc3 = pc1+pc2
        o3d.io.write_point_cloud(r".\Result\Result_{}.ply".format("CompareSize"),pc3,write_ascii=True,print_progress=True)

    @staticmethod
    def compareWithColoredPC(pointCloud,mesh):
        pc1 = pointCloud
        pc2 = mesh.sample_points_uniformly(len(pc1.points),use_triangle_normal=False,seed=-1)
        pc2 = pc2.paint_uniform_color([0,0,1])
        pc3 = pc1+pc2
        o3d.io.write_point_cloud(r".\Result\Result_{}.ply".format("CompareWithColor"),pc3,write_ascii=True,print_progress=True)

    @staticmethod
    def generateHistgram(txt):
        info = Util.analyzeFilePath(txt)
        sns.set(style="darkgrid")
        data = np.loadtxt(txt)
        dataX = data[:,-1]
        sns.distplot(dataX,bins=200,kde=False)
        plt.savefig(r"Result\Fig_of_{}.png".format(info[0]))
        # plt.show()


if __name__ == "__main__":
    # a = o3d.io.read_point_cloud(r"E:\OneDrive\CS800Run\CompareScript\Data\PointCloud\mvsnet000_l3.ply")
    # b =o3d.geometry.PointCloud.remove_statistical_outlier(a,4,0.1)
    # o3d.io.write_point_cloud(r"E:\OneDrive\CS800Run\CompareScript\Data\PointCloud\mvsnet000_l3_2.ply",b[0],write_ascii=1)
    plyPath = r"E:\OneDrive\CS800Run\PA3DMP\Data\PointCloud\mvsnet000_l3.ply"
    optPath = r"E:\OneDrive\CS800Run\PA3DMP\Data\PointCloud\mvsnet000_l3_opt.ply"
    meshFolder = r"E:\OneDrive\CS800Run\PA3DMP\Data\Mesh\textured_mesh"
    ResultExample = r"Result\Result_13758.92653298378s.txt"
    instance = Pa3dmp()

    instance.generateHistgram(r"Result\Result_BekYGuZK.txt")
    # instance.resizePointCloud(r"E:\OneDrive\CS800Run\PA3DMP\Data\PointCloud\mvsnet000_l3.ply")
    # print(instance.p2fDistance(p,Face))
    #instance.optimizePointCloud(plyPath)
    # tree = o3d.geometry.KDTreeFlann(c)
    # b = tree.search_radius_vector_3d(np.array([0,0,0]),0.02)
    # idx = b[1]
    # verticesArr = np.asarray(c.vertices)
    # points = np.take(verticesArr,idx,axis=0)
    # mesh1 = copy.deepcopy(c)
    # box = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(points))
    # mesh1 = mesh1.crop(box)
    # my = np.asarray(mesh1.triangles)
    # for j in range(len(my)):
    #     xx = my[j]
    #     x = mesh1.vertices[xx[0]]
    #     y = mesh1.vertices[xx[1]]
    #     z = mesh1.vertices[xx[2]]
    #     print(x,y,z)
    # o3d.io.write_triangle_mesh("out.obj",mesh1,print_progress=True)
    # cur = np.ones((1,5))
    # all = np.zeros((5,8))
    # a = np.array([333,333])
    # b = cur[0,:]
    # c = 5
    # line = np.hstack((a,b,c))
    # all[2] = line
    # print(all)

    # plyOpt = instance.loadPointCloud(optPath)
    # mesh = MeshData(meshFolder).getMeshCombined()
    # # re = instance.loopForPoints(plyOpt,mesh,0.005)
    # # np.savetxt("./Result/Result_{}s.txt".format(re[1]),re[0],fmt="%.6f",newline="\n")
    # data = np.loadtxt(ResultExample)
    # colored = instance.paintColor(plyOpt,data,"Test1")
    # # instance.compare(plyOpt,mesh)
    # instance.compareWithColoredPC(colored,mesh)

    print("Succeed")

