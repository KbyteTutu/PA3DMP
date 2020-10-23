'''
Tring to implement multiprocessing here
'''

from cv2 import cv2
import numpy as np
import math,time,os,shutil,copy,random,string
import open3d as o3d
from tqdm import tqdm
from  UtilityFunctions import Util
from My3DData import MeshData,PointCloudData
from PA3DMP import Pa3dmp
from multiprocessing import Pool


class MultiPa3dmp(object):
    def __init__(self,pointCloudPath,meshPath):
        self.pointCloudPath= pointCloudPath
        self.meshPath = meshPath
        self.randomId = ''.join(random.sample(string.ascii_letters + string.digits, 8))
        try:
            self.mesh = MeshData(self.meshPath)
            self.pointCloud = PointCloudData(self.pointCloudPath)
        except Exception as e:
            print(e)

    def mergeTxt(self):
        #Merge the result together.
        path = "Result/Result_{}.txt".format(self.randomId)
        with open(path,'w') as f:
            for (root,dirs,files) in os.walk(r"Workspace/txt"):
                for file in files:
                    with open(os.path.join(root,file),'r') as k:
                        f.write(k.read())
        print("Txt Merged")
        return path

def funcWraper(name,meshpath):
    #包装一下，调用对象
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    p = Pa3dmp()
    plyPath = r"Workspace/ply/{}.ply".format(name)
    pointCloud = o3d.io.read_point_cloud(plyPath)
    mesh = o3d.io.read_triangle_mesh(r"{}/{}.obj".format(meshpath,name))
    re = p.loopForPoints(pointCloud,mesh,0.01)
    np.savetxt(r"Workspace/txt/{}.txt".format(name),re,fmt="%.6f",newline="\n")
    print("{} - done.".format(name))
    # a = np.array([0,2,5])
    # np.savetxt("Workspace/{}_.txt".format(name),a)
    # print("ok")

if __name__ == '__main__':
    start = time.time()
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    instance = MultiPa3dmp(r"Data\\mvsnet000_l3.ply",r"Data\\5b21e18c58e2823a67a10dd8\\textured_mesh")
    Util.mkCleanDir(r"Workspace")
    Util.mkCleanDir(r"Workspace/txt")
    nameList = instance.mesh.getObjNameList()
    meshList = instance.mesh.getMeshDict()
    pointCloudsList = instance.pointCloud.seperatePointCloud(instance.mesh.getAxisBoundingBoxList(1.2),instance.mesh.getObjNameList(),1)

    print("The random ID of this task is:{}".format(instance.randomId))

    # for i in range(len(meshList)):
    #     cur = nameList[i]
    #     funcWraper(cur,instance.meshPath)
    #     if i == 1:
    #         break
    # r = []

    p = Pool()
    for i in range(len(meshList)):
        cur = nameList[i]
        p.apply_async(func = funcWraper,args=(cur,instance.meshPath))
    p.close()
    p.join()

    end = time.time()
    txtPath = instance.mergeTxt()
    Pa3dmp.generateHistgram(txtPath)
    shutil.rmtree("Workspace")
    print("Result Txt is {} and the histgram fig of this txt".format(txtPath))
    print("Executed Time:{}s".format(end-start))
