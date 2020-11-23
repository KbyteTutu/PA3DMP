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
        try:
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
            if mod_area == 0:
                d = 0
                raise RuntimeWarning("divide_0")
            d = abs(mod_d) / mod_area
        except Exception as e:
            #TODO:输出debug的log，看看报错是怎么回事。
            info = [point1,point2,point3,point4,mod_area,mod_d]
            Util.tempLog("{}".format(info))
        finally:
            return d

    def loopForPoints(self,pointCloud,triangleMesh,flag=0):
        #flag决定是使用hybrid还是radius。前者模糊但是更快，后者准确但是更慢
        try:
            #储存矩阵
            savingMatrix = np.zeros((0,13))
            #mesh的KD树生成
            meshKDtree = o3d.geometry.KDTreeFlann(triangleMesh)
            verticesArr = np.asarray(triangleMesh.vertices)
            #对每个PointCloud点开始循环遍历
            for pidx in range(len(pointCloud.points)):
                p = pointCloud.points[pidx]
                #取目标点最近的几个点，并据此动态设定搜索半径。以p与第二近的点的距离做半径搜索第一次，如果满足最小数量条件则去生成包
                #此处需要测试，但是我觉得五个紧邻点应该足够了，太稀疏的点云应该没有测试价值。
                #包生成的报错情况则需要raise.然后如果生成错误，继续用下一个点。如果n个点都不行，则真正raise

                #补充，实测以近邻距离做r样本太少了，改成while，取到15个点为止吧
                try:
                    box = o3d.geometry.AxisAlignedBoundingBox()
                    n=100
                    k=5
                    #取n个备选的点，从近到远。然后采样间隔为k,即循环步长是k
                    tList = meshKDtree.search_knn_vector_3d(p,n)
                    if tList[0] <=k:
                        raise RuntimeError("The point cloud is too sparse. Probably input an empty mesh.")
                    for idx in range(k,n,k):
                        #测试范围内的点数量，如果数量足够则生成包，如不够或者包错误则raise。
                        r = self.computeRadius(p,tList[1][idx],triangleMesh)
                        if flag == 0:
                            m = meshKDtree.search_hybrid_vector_3d(p,r,15)
                        else:
                            m = meshKDtree.search_radius_vector_3d(p,r)
                        if m[0]<15:
                            if idx == int(n-k):
                                raise RuntimeWarning("Search ends with no enough points to generate bounding box for this query point")
                            continue
                        else:
                            #create a box from m
                            pointsInBox = np.take(verticesArr,m[1],axis=0)
                            tempbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pointsInBox))
                            if tempbox.is_empty()==True:
                                if idx == int(n-k):
                                    raise RuntimeWarning("Search ends")
                                continue
                            else:
                                box = copy.deepcopy(tempbox)
                                break
                except Exception as e:
                    continue

                try:
                    #根据上面得到的box取点，然后计算最近的面
                    nMesh = triangleMesh.crop(box)
                    nTriangles = np.asarray(nMesh.triangles)
                    cur = np.zeros((1,9))
                    d = float('inf')
                    for pidxSmall in range(len(nTriangles)):
                        idxx = nTriangles[pidxSmall]
                        p1= nMesh.vertices[idxx[0]]
                        p2= nMesh.vertices[idxx[1]]
                        p3= nMesh.vertices[idxx[2]]
                        dTemp = self.p2fDistance(p,p1,p2,p3)
                        #If the distance is same,do an extra comparation
                        if dTemp ==d:
                            #TODO:加上相等的时候的函数
                            pass
                        elif dTemp<d:
                            d = dTemp
                            # cur = np.asarray([p1[0],p1[1],p1[2],p2[0],p2[1],p2[2],p3[0],p3[1],p3[2]])
                            cur = np.hstack((p1,p2,p3))
                    if d!=float('inf'):
                        temp =np.hstack((p,cur,d)).reshape((1,13))
                        savingMatrix = np.append(savingMatrix,temp,axis=0)
                except Exception as e:
                    print(e)
        except RuntimeError as e:
            print(e)
        finally:
            return savingMatrix


    def loopForMesh(self,mesh,pointCloud):
        try:
            PCKdTree = o3d.geometry.KDTreeFlann(pointCloud)
            savingMatrix = np.zeros((0,7))
            for idx in range(len(mesh.vertices)):
                mp = mesh.vertices[idx]
                re = PCKdTree.search_knn_vector_3d(mp,1)
                target = pointCloud.points[re[1][0]]
                d = Util.L2Norm3(mp,target)
                cur = np.hstack((mp,target,d)).reshape(1,7)
                savingMatrix = np.append(savingMatrix,cur,axis=0)
        except Exception as e:
            print(e)
        finally:
            return savingMatrix


    @staticmethod
    def computeRadius(p,idx,mesh):
        #根据输入的idx计算p和mesh中idx位置的点的距离
        t = mesh.vertices[idx]
        return Util.L2Norm3(p,t)


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
    def generateHistogram(txt,prefix,rId):
        sns.set(style="darkgrid")
        data = np.loadtxt(txt)
        if len(data)>0:
            dataX = data[:,-1]
            sns.distplot(dataX,bins=50,kde=False)
            plt.savefig(r"Result\{}\Fig_of_{}.png".format(rId,prefix))
        # plt.show()
        else:
            print("No enough data")


if __name__ == "__main__":
    #ALLTEST CODE BELOW,JUST IGNORE
    instance = Pa3dmp()

    # a = o3d.io.read_point_cloud(r"E:\OneDrive\CS800Run\CompareScript\Data\PointCloud\mvsnet000_l3.ply")
    # b =o3d.geometry.PointCloud.remove_statistical_outlier(a,4,0.1)
    # o3d.io.write_point_cloud(r"E:\OneDrive\CS800Run\CompareScript\Data\PointCloud\mvsnet000_l3_2.ply",b[0],write_ascii=1)
    # plyPath = r"Data\Test\FusionedPointCloud.ply"
    # optPath = r"Data\Test\mvsnet000_l3.ply"
    # meshFolder = r"E:\OneDrive\CS800Run\PA3DMP\Data\Test\textured_mesh"
    # ResultExample = r"Result\Result_13758.92653298378s.txt"

    # arr = np.array([[0,0,0],[1,1,1]])
    # c = np.linalg.norm(arr,ord=2)
    # print(c)

    # m = MeshData(meshFolder)
    # p = o3d.io.read_point_cloud(plyPath)
    # instance.compare(p,m.getMeshCombined())

    # instance.generateHistogram(r"Result\Result_BekYGuZK.txt")
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
    # # # re = instance.loopForPoints(plyOpt,mesh,0.005)
    # # # np.savetxt("./Result/Result_{}s.txt".format(re[1]),re[0],fmt="%.6f",newline="\n")
    # data = np.loadtxt(ResultExample)
    # colored = instance.paintColor(plyOpt,data,"Test1")
    # # instance.compare(plyOpt,mesh)
    # instance.compareWithColoredPC(colored,mesh)

    smallPly = o3d.io.read_point_cloud(r"WorkspaceTest\ply\tile_2_4.ply")
    smallMesh = o3d.io.read_triangle_mesh(r"WorkspaceTest\textured_mesh\tile_2_4.obj")
    # verticesArr = np.asarray(smallMesh.vertices)

    # p = smallPly.points[5000]
    # meshTree = o3d.geometry.KDTreeFlann(smallMesh)
    # r = meshTree.search_knn_vector_3d(p,100)
    # a = instance.computeRadius(p,r[1][20],smallMesh)
    # m = meshTree.search_hybrid_vector_3d(p,a,20)
    # pointsInBox = np.take(verticesArr,m[1],axis=0)
    # tempbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pointsInBox))
    # print(tempbox.is_empty())

    # a = np.asarray(smallMesh.vertices)
    tree = o3d.geometry.KDTreeFlann(smallPly)
    re =tree.search_knn_vector_3d(smallMesh.vertices[1000],1)
    rrr = np.zeros((0,7))
    print(smallMesh.vertices[1000])
    print(smallPly.points[re[1][0]])
    d = Util.L2Norm3(smallMesh.vertices[1000],smallPly.points[re[1][0]])
    cur = np.hstack((smallMesh.vertices[1000],smallPly.points[re[1][0]],d)).reshape(1,7)
    rrr = np.append(rrr,cur,axis=0)


    print("Succeed")

