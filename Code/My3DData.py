"""
2020年10月13日15:02:11

@author: Tu Kechao
"""
from cv2 import cv2
import numpy as np
import math,time,os,shutil,copy
import open3d as o3d
from tqdm import tqdm
from  UtilityFunctions import Util

class MeshData(object):
    def __init__(self, folderPath):
        self.folderPath = folderPath
        self.__objPathList = []
        self.__meshList= []
        self.__objNameList =[]
        self.loadFolder()
        self.getMeshList()


    def loadFolder(self):
        info = os.walk(self.folderPath)
        objList = []
        objNameList = []
        # assert self.__checkValid(info),"文件内容有误"
        for path,dirs,files in info:
            for file in files:
                nInfo = os.path.splitext(file)
                if nInfo[1] == ".obj":
                    objList.append(os.path.join(self.folderPath,file))
                    objNameList.append(nInfo[0])
        self.__objPathList = objList
        self.__objNameList = objNameList

    def getMeshList(self):
        if self.__meshList != []:
            print("Already Loaded.")
            return self.__meshList
            #避免每次get都去读取一遍
        else:
            print("Loading,please wait..")
            temp = []
            for path in self.__objPathList:
                mesh = o3d.io.read_triangle_mesh(path)
                temp.append(mesh)
            self.__meshList = temp
            print("Triangle Mesh Loaded.")
            return temp

    def getMeshCombined(self):
        if len(self.__meshList)==0:
            self.getMeshList()
        outMesh = o3d.open3d_pybind.geometry.TriangleMesh()
        for m in self.__meshList:
            outMesh += m
        print("Mesh Combined.")
        return outMesh

    def getAxisBoundingBoxList(self,scale):
        try:
            boxList = []
            for i in self.__meshList:
                box = i.get_axis_aligned_bounding_box()
                box.scale(scale,box.get_center())
                boxList.append(box)
            return boxList
        except Exception as e:
            print(e)

    def getObjNameList(self):
        return self.__objNameList

    def getMeshDict(self):
        # generate dict for mesh.
        mDict = {}
        for i in range(len(self.__objNameList)):
            mDict[self.__objNameList[i]] = self.__meshList[i]
        return mDict


class PointCloudData(object):
    def __init__(self,pointCloudPath):
        self.pointCloudPath = pointCloudPath
        try:
            self.pointCloudData = o3d.io.read_point_cloud(self.pointCloudPath)
        except Exception as e:
            print(e)

    def optimizePointCloud(self,PointCloudPath):
        # TODO: some how optimize the point cloud
        p = o3d.io.read_point_cloud(PointCloudPath)
        pointCloud = o3d.geometry.PointCloud.remove_statistical_outlier(p,4,0.1)
        o3d.io.write_point_cloud(r"E:\OneDrive\CS800Run\PA3DMP\Data\PointCloud\mvsnet000_l3_opt.ply",pointCloud[0],write_ascii=1)

    def paintColor(self,pointCloud,dataArr,namePrefix):
        dataArr = np.asarray(dataArr)
        out = copy.deepcopy(pointCloud)
        maxDis = np.max(dataArr[:,13])
        minDis = np.min(dataArr[:,13])
        dif = maxDis-minDis
        out = out.paint_uniform_color([1,1,1])
        for i in range(len(dataArr[:,0])):
            line = dataArr[i]
            rate = 1 - line[13]/dif
            out.colors[int(line[0])] = np.array([1,rate,0])
        o3d.io.write_point_cloud(r".\Result\Result_{}.ply".format(namePrefix),out,write_ascii=True,print_progress=True)
        return out

    def seperatePointCloud(self,meshBoxList,nameList,saveFlag):
        # TODO:把点云按照mesh的分块来分割成小块
        if saveFlag:
            Util.mkCleanDir(r"Workspace\ply")
        try:
            pointCloud = self.pointCloudData
            pDict = {}
            for i in range(len(meshBoxList)):
                temp = pointCloud.crop(meshBoxList[i])
                if saveFlag>0:
                    o3d.io.write_point_cloud(r"Workspace\ply\{}.ply".format(nameList[i]),temp,write_ascii=True)
                else:
                    pDict[nameList[i]] = temp
            if saveFlag>0:
                print("Seperated point clouds saved.")
                return
            else:
                return pDict
        except Exception as e:
            print(e)


if __name__ == "__main__":
    # path = r"Test\textured_mesh\tile_5_5.obj"
    # path2 = r"Test\textured_mesh\tile_5_6.obj"
    # path3 = r"Test\textured_mesh\tile_5_7.obj"
    # path4 = r"Test\textured_mesh\tile_6_0.obj"
    # a = o3d.io.read_triangle_mesh(path,print_progress=True)
    # b = o3d.io.read_triangle_mesh(path2,print_progress=True)
    # c = o3d.io.read_triangle_mesh(path3,print_progress=True)
    # d = o3d.io.read_triangle_mesh(path4,print_progress=True)
    # o3d.visualization.draw_geometries([a,b,c,d])

    # a = os.path.join(os.getcwd(),"Test\\textured_mesh")
    # out = os.path.join(os.getcwd(),"out.ply")
    # b = MeshData(a)
    # dd = b.getMeshList()
    # outMesh = o3d.open3d_pybind.geometry.TriangleMesh()
    # for m in dd:
    #     outMesh += m
    # o3d.io.write_triangle_mesh("out.obj",outMesh,print_progress=True)
    # o3d.visualization.draw_geometries(dd)
    # o3d.visualization.draw_geometries(b.getMeshList())

    p = PointCloudData(r"Data\PointCloud\mvsnet000_l3.ply")
    m = MeshData(r"Data\Mesh\textured_mesh")

    a =p.seperatePointCloud(m.getAxisBoundingBoxList(1.2),m.getObjNameList(),0)
    print(a)
