"""
2020年10月13日15:02:11

@author: Tu Kechao
"""

import open3d as o3d
import os
import copy
import UtilityFunctions

class MeshData(object):
    def __init__(self, folderPath):
        self.folderPath = folderPath
        self.__objPathList = []
        self.__meshList= []
        self.loadFolder()


    def loadFolder(self):
        info = os.walk(self.folderPath)
        objList = []
        # assert self.__checkValid(info),"文件内容有误"
        for path,dirs,files in info:
            for file in files:
                if file.endswith(".obj"):
                    objList.append(os.path.join(self.folderPath,file))
        self.__objPathList = objList

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
            return temp

    def getMeshCombined(self):
        outMesh = o3d.open3d_pybind.geometry.TriangleMesh()
        for m in self.__meshList:
            outMesh += m
        return outMesh


    # def __checkValid(self,info):
    #     for root,dirs,files in info:
    #         if "blended_images"




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

    a = os.path.join(os.getcwd(),"Test\\textured_mesh")
    out = os.path.join(os.getcwd(),"out.ply")
    b = myMeshData(a)
    dd = b.getMeshList()
    outMesh = o3d.open3d_pybind.geometry.TriangleMesh()
    for m in dd:
        outMesh += m
    o3d.io.write_triangle_mesh("out.obj",outMesh,print_progress=True)
    # o3d.visualization.draw_geometries(dd)
    # o3d.visualization.draw_geometries(b.getMeshList())