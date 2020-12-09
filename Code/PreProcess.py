# -*- coding: utf-8 -*-
"""
2020年10月8日01:43:36
This is for preprocess BlendedMVS DataSet to fit pytorch version MVSNet

@author: Tu Kechao
"""
import os,sys,shutil

def getFoldersToProcess(path):
    # for root,dirs,files in os.listdir(path):
    #     for dir in dirs:
    #         print(dir)
    dirsToProcess = []
    for d in os.listdir(path):
        f = os.path.join(path,d)
        if os.path.isdir(f):
            if checkDir(os.path.join(f,d,d)):
                dirsToProcess.append(d)
            else:
                if d != "Output_Dataset":
                    print("Found Invalid Folder:" + d)
    print("Found Valid Folder Number:" + str(len(dirsToProcess)))
    return dirsToProcess

def checkDir(path):
    blended_images_flag = os.path.isdir(os.path.join(path,"blended_images"))
    cams_flag = os.path.isdir(os.path.join(path,"cams"))
    rendered_depth_maps_flag = os.path.isdir(os.path.join(path,"rendered_depth_maps"))
    return blended_images_flag and cams_flag and rendered_depth_maps_flag

def Process(list,path,flag):
    # mkDir
    mkCleanDir(os.path.join(path,"Output_Dataset"))
    cnt = 0
    for f in list:
        targetdir = os.path.join(path,"Output_Dataset","scan{}".format(cnt))
        workdir = os.path.join(path,f,f,f)
        # copy images
        shutil.copytree(os.path.join(workdir,"blended_images"),os.path.join(targetdir,"images"))
        # copy cams
        shutil.copytree(os.path.join(workdir,"cams"),os.path.join(targetdir,"cams"))
        shutil.copy(os.path.join(targetdir,"cams","pair.txt"),os.path.join(targetdir,"pair.txt"))
        os.remove(os.path.join(targetdir,"cams","pair.txt"))
        # delFlag
        if flag == False:
            shutil.rmtree(os.path.join(path,f))
        print("Folder " + f +" is Processed.")
        with open(os.path.join(os.path.join(path,"Output_Dataset"),"test.txt"), 'a') as f:
            if cnt+1 == len(list):
                f.writelines("scan{}".format(cnt))
            else:
                f.writelines("scan{}\n".format(cnt))
        cnt+=1
# Util
def mkCleanDir(path):
    if os.path.isdir(path):
        #先清空再创建
        shutil.rmtree(path)
        os.mkdir(path)
    else:
        os.mkdir(path)

if __name__ == "__main__":
    initPath = sys.argv[1]
    if eval(sys.argv[2])>0:
        keepFileFlag = True
    else:
        keepFileFlag = False
    if os.path.isdir(initPath) == False:
        print("Invalid input, not a folder")
        sys.exit()
    listToProcess = getFoldersToProcess(initPath)
    if  keepFileFlag :
        print("File will be remain.")
    else:
        print("File will be rearranged.")
    print("Started to Process..")

    Process(listToProcess,initPath,keepFileFlag)
