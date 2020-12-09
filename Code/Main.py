'''
Tring to implement multiprocessing here
'''

from cv2 import cv2
import numpy as np
import math,time,os,shutil,copy,random,string
import open3d as o3d
from  UtilityFunctions import Util
from My3DData import MeshData,PointCloudData
from PA3DMP import Pa3dmp
from multiprocessing import Pool


class MultiPa3dmp(object):
    def __init__(self,pointCloudPath):
        self.pointCloudPath= pointCloudPath
        self.randomId = ''.join(random.sample(string.ascii_letters + string.digits, 8))
        try:
            self.pointCloud = PointCloudData(self.pointCloudPath)
        except Exception as e:
            print(e)

    def mergeTxt(self,pathP,prefix):
        #Merge the result together.
        path = "Result/{}/Result_{}.txt".format(self.randomId,prefix)
        with open(path,'w') as f:
            for (root,dirs,files) in os.walk(pathP):
                for file in files:
                    with open(os.path.join(root,file),'r') as k:
                        f.write(k.read())
        print("Txt Merged")
        return path

    def prefixMeshes(self,meshPath,threshold):
        info = os.walk(meshPath)
        for path,dirs,files in info:
            for file in files:
                nInfo = os.path.splitext(file)
                if nInfo[1] == ".obj":
                    m = o3d.io.read_triangle_mesh(os.path.join(path,file))
                    if len(m.vertices) >threshold:
                        self.recursionCutMesh(m,nInfo,threshold)
                    else:
                        o3d.io.write_triangle_mesh(r"Workspace\mesh\{}".format(file),m,write_triangle_uvs=False)

    def recursionCutMesh(self,m,nInfo,threshold):
        #递归的裁切函数
        scale = 1.2
        if len(m.vertices)>threshold:
            [x0,y0,z0] = m.get_min_bound()
            [x1,y1,z1] = m.get_max_bound()
            x_offset = (x1-x0)/2
            y_offset = (y1-y0)/2
            ld_box= o3d.geometry.AxisAlignedBoundingBox([x0,y0,z0],[x0+x_offset,y0+y_offset,z1])
            lu_box= o3d.geometry.AxisAlignedBoundingBox([x0,y0+y_offset,z0],[x0+x_offset,y1,z1])
            rd_box= o3d.geometry.AxisAlignedBoundingBox([x0+x_offset,y0,z0],[x1,y0+y_offset,z1])
            ru_box= o3d.geometry.AxisAlignedBoundingBox([x0+x_offset,y0+y_offset,z0],[x1,y1,z1])
            ld_box = ld_box.scale(scale,ld_box.get_center())
            lu_box = lu_box.scale(scale,lu_box.get_center())
            ru_box = ru_box.scale(scale,ru_box.get_center())
            rd_box = rd_box.scale(scale,rd_box.get_center())
            ld_mesh = m.crop(ld_box)
            lu_mesh = m.crop(lu_box)
            rd_mesh = m.crop(rd_box)
            ru_mesh = m.crop(ru_box)
            if len(ld_mesh.vertices)<threshold:
                o3d.io.write_triangle_mesh(r"Workspace\mesh\{}{}{}".format(nInfo[0],"_ld",nInfo[1]),ld_mesh,write_triangle_uvs=False)
            else:
                nInfo = ["{}_sub".format(nInfo[0]),nInfo[1]]
                self.recursionCutMesh(ld_mesh,nInfo,threshold)
            if len(lu_mesh.vertices)<threshold:
                o3d.io.write_triangle_mesh(r"Workspace\mesh\{}{}{}".format(nInfo[0],"_lu",nInfo[1]),lu_mesh,write_triangle_uvs=False)
            else:
                nInfo = ["{}_sub".format(nInfo[0]),nInfo[1]]
                self.recursionCutMesh(lu_mesh,nInfo,threshold)
            if len(rd_mesh.vertices)<threshold:
                o3d.io.write_triangle_mesh(r"Workspace\mesh\{}{}{}".format(nInfo[0],"_rd",nInfo[1]),rd_mesh,write_triangle_uvs=False)
            else:
                nInfo = ["{}_sub".format(nInfo[0]),nInfo[1]]
                self.recursionCutMesh(rd_mesh,nInfo,threshold)
            if len(ru_mesh.vertices)<threshold:
                o3d.io.write_triangle_mesh(r"Workspace\mesh\{}{}{}".format(nInfo[0],"_ru",nInfo[1]),ru_mesh,write_triangle_uvs=False)
            else:
                nInfo = ["{}_sub".format(nInfo[0]),nInfo[1]]
                self.recursionCutMesh(ru_mesh,nInfo,threshold)
            # o3d.io.write_triangle_mesh(r"Workspace\mesh\{}{}{}".format(nInfo[0],"_lu",nInfo[1]),lu_mesh,write_triangle_uvs=False)
            # o3d.io.write_triangle_mesh(r"Workspace\mesh\{}{}{}".format(nInfo[0],"_rd",nInfo[1]),rd_mesh,write_triangle_uvs=False)
            # o3d.io.write_triangle_mesh(r"Workspace\mesh\{}{}{}".format(nInfo[0],"_ru",nInfo[1]),ru_mesh,write_triangle_uvs=False)

def funcWraper(name,meshpath,progress):
    #包装一下，调用对象
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    p = Pa3dmp()
    plyPath = r"Workspace/ply/{}.ply".format(name)
    pointCloud = o3d.io.read_point_cloud(plyPath)
    if os.path.exists(r"{}/{}.obj".format(meshpath,name)):
        mesh = o3d.io.read_triangle_mesh(r"{}/{}.obj".format(meshpath,name))
        # re = p.loopForPoints(pointCloud,mesh,20)
        re = p.loopForPoints(pointCloud,mesh,flag = 0)
        np.savetxt(r"Workspace/txt/{}.txt".format(name),re,fmt="%.6f",newline="\n")
        fCnt = int(Util.getFileCnt(r"Workspace/txt"))
        percent = round((fCnt/int(progress))*100,2)
        print("{} Processed - {}/{} - {}%".format(name,fCnt,progress,percent))
    # a = np.array([0,2,5])
    # np.savetxt("Workspace/{}_.txt".format(name),a)
    # print("ok")


def funcWraperMesh(name,meshpath):
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    p = Pa3dmp()
    plyPath = r"Workspace/ply/{}.ply".format(name)
    pointCloud = o3d.io.read_point_cloud(plyPath)
    if os.path.exists(r"{}/{}.obj".format(meshpath,name)):
        mesh = o3d.io.read_triangle_mesh(r"{}/{}.obj".format(meshpath,name))
        # re = p.loopForPoints(pointCloud,mesh,20)
        re = p.loopForMesh(mesh,pointCloud)
        np.savetxt(r"Workspace/txtMesh/{}.txt".format(name),re,fmt="%.6f",newline="\n")
    # pp = Pa3dmp()
    # pp.loopForMesh(curMesh.getMeshCombined(),o3d.io.read_point_cloud(plyPath))


def DoPA3DMP(plyPath,meshPath,poolNum):
    start = time.time()
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    plyPath = plyPath
    meshPath = meshPath
    instance = MultiPa3dmp(plyPath)
    Util.mkCleanDir(r"Workspace")
    Util.mkCleanDir(r"Workspace/txt")
    Util.mkCleanDir(r"Workspace/txtMesh")
    Util.mkCleanDir(r"Workspace/mesh")

    #2020年11月20日00:56:00 New updated
    instance.prefixMeshes(meshPath,4000)

    curMesh = MeshData(r"Workspace/mesh")
    nameList = curMesh.getObjNameList()
    meshList = curMesh.getMeshDict()
    pointCloudsList = instance.pointCloud.seperatePointCloud(curMesh.getAxisBoundingBoxList(1.2),curMesh.getObjNameList(),1)

    print("The random ID of this task is:{}".format(instance.randomId))

    # #Testing Code
    # cur = nameList[20]
    # # funcWraper(cur,r"Workspace/mesh")
    # funcWraperMesh(cur,r"Workspace/mesh")

    #Running Code
    p = Pool(processes=poolNum)
    for i in range(len(meshList)):
        try:
            cur = nameList[i]
            total = len(meshList)
            p.apply_async(func = funcWraper,args=(cur,r"Workspace/mesh",total))
        except Exception as e:
            print(e)
    p.close()
    p.join()


    Util.mkCleanDir(r"Result/{}".format(instance.randomId))
    txtPath = instance.mergeTxt(r"Workspace/txt","P2M")
    Pa3dmp.generateHistogram(txtPath,"P2M",instance.randomId)

    print("Point Cloud to Mesh work done!")

    #Running Code
    p2 = Pool(processes=10)
    for k in range(len(meshList)):
        try:
            cur = nameList[k]
            p2.apply_async(func = funcWraperMesh,args=(cur,r"Workspace/mesh"))
        except Exception as e:
            print(e)
    p2.close()
    p2.join()

    txtPath2 = instance.mergeTxt(r"Workspace/txtMesh","M2P")
    Pa3dmp.generateHistogram(txtPath2,"M2P",instance.randomId)


    end = time.time()

    shutil.rmtree("Workspace")
    print("Result Txt is {} and {}".format(txtPath,txtPath2))
    print("Executed Time:{}s".format(end-start))



if __name__ == '__main__':
    path = r"E:\OneDrive\CS800Run\PA3DMP\Data\ForReport"
    for dir in os.listdir(path):
        f = os.path.join(path,dir)
        plyPath = os.path.join(f,"pc.ply")
        meshPath = os.path.join(f,"mesh")
        DoPA3DMP(plyPath,meshPath,14)
