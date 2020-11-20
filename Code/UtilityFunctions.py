"""
2020年6月28日02:57:35

@author: Tu Kechao
"""

from cv2 import cv2
import numpy as np
import math
import time
import os
import shutil

class Util(object):
    @staticmethod
    def perf_time(func):
        def wrap(*args):
            start = time.time()
            result = func(*args)
            cost = time.time() - start
            print("{} used {} s".format(func.__name__, cost))
            return result
        return wrap

    @staticmethod
    def perf_time_output(func):
        def wrap(*args):
            start = time.time()
            result = func(*args)
            cost = time.time() - start
            print("{} used {} s".format(func.__name__, cost))
            return (result,cost)
        return wrap

    @staticmethod
    def L2Norm(actualValue,predictedValue):
        return np.sum(np.power((actualValue-predictedValue),2))

    @staticmethod
    def L2Norm3(x,y):
        arr = np.array([x[0]-y[0],x[1]-y[1],x[2]-y[2]])
        return np.linalg.norm(arr)


    @staticmethod
    def loadImage(path):
        return cv2.imread(path)

    @staticmethod
    def loadImageGray(path):
        return cv2.imread(path,0)

    @staticmethod
    def analyzeFilePath(path):
        (filepath,tempfilename) = os.path.split(path)
        (filename,extension) = os.path.splitext(tempfilename)
        return (filename,filepath,extension)

    @staticmethod
    def mkCleanDir(path):
        if os.path.isdir(path):
            #先清空再创建
            shutil.rmtree(path)
            os.mkdir(path)
        else:
            os.mkdir(path)

    @staticmethod
    def edgeStrength(fx,fy):
        # get edge strength
        edge = np.sqrt(np.power(fx, 2) + np.power(fy, 2))
        fx = np.maximum(fx, 1e-5)

        # get edge angle
        angle = np.arctan(fy / fx)
        # angle = np.degrees(angle)

        return edge, angle

    @staticmethod
    def checkInputIsSquare(num):
        try:
            n = float(np.sqrt(num))
            if n.is_integer():
                return int(n)
            else:
                return False
        except:
            return False

if __name__ == "__main__":
    #Ignore these, a playground

    # a = np.cbrt(64)
    # # print(isinstance(4.0,float))
    pass